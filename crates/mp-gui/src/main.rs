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

//! `planner` - the graphical front end.
//!
//! Built on gpui, Zed's GPU-accelerated UI framework (DELIVERABLES.md Deliverable 6/Deliverable 7).
//!
//! Run with a link URL to connect, or with no arguments for a disconnected shell:
//!   planner tcp:127.0.0.1:5760

#![allow(clippy::print_stderr)]


use mp_os::fs::FsExt as _;
/// The product's name, as the owner has it (2026-10-03): the window's title and the header over
/// the screen buttons both carry it. Mission Planner shows its own name and version there
/// (`MainV2.cs:815, 1739`); this application is not Mission Planner, and its title says which it
/// is - which is also how the GUI runner tells our window from a real Mission Planner's.
pub const PRODUCT_NAME: &str = "MissionPlannerRust";

mod camera_photos;
mod cmd_keys;
mod config;
mod config_coverage;
mod connect;
mod coords;
mod coverage;
mod crash;
// ---- Display view (row 71) ----
mod display_view;
// ---- end Display view ----
mod facts;
mod fly;
mod gauge;
// The flight map's Gimbal Video: GimbalVideoControl and its forms.
mod gimbal_video;
// ---- Geo Reference ----
mod georef_ui;
mod glyph_text;
// ---- end Geo Reference ----
mod help;
mod http_server;
mod hud;
mod i18n;
mod inject_map;
mod joystick;
mod layout_guard;

mod logbrowse;
mod logdownload;
mod logs_tab;
mod mapview;
mod metadata;
mod params;
mod payload;
mod pictures;
// ---- row 82 ----
mod raw_params;
// ---- end row 82 ----
// ---- ConfigRawParams remainder ----
mod raw_params_grid;
mod raw_sensor;
mod scripts_tab;
// ---- end ConfigRawParams remainder ----
mod plan;
mod planner_coverage;
mod plotline;
// ---- row 96 ----
mod experimental;
mod plugin_manager;
mod plugins_ui;
// The owner's Welcome-Demo-Sitl plugin's pointer: a drawn cursor and real clicks (not in the C#).
mod demo_pointer;
// ---- end row 96 ----
mod platform;
mod poi;
mod prefetch_ui;
mod probe;
mod quick;
mod settings;
mod setup;
// ---- SITL ----
mod sitl;
// ---- end SITL ----
mod smoke;
mod srtm;
mod stderr_log;
mod storm;
// What each screen's frames cost in an ordinary run (`MP_FRAMES`).
mod frametimes;
// The browser's file picker and downloads for the file boxes, in a page.
mod page_files;
#[cfg(target_family = "wasm")]
mod page_storage;
mod repaint;
mod survey_ui;
mod telemetry;
mod tour;
mod textfield;
mod transponder;
mod tuning;
mod ui;
// ---- Warning Manager ----
mod warnings;
// ---- end Warning Manager ----

use std::time::Duration;

use gpui::{
    App, Bounds, Context, MouseButton, SharedString, TitlebarOptions, Window, WindowBounds,
    WindowOptions, div, prelude::*, px, rgb, size,
};
use mapview::MapViewport;
use mp_tiles::cache::TileCache;
use mp_tiles::store::TileStore;
use plan::Plan;

/// `FP_docking`: `panelAction.Dock`, Right by default, Bottom after Switch Docking.
/// `// C#: GCSViews/FlightPlanner.cs:6762-6778`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Docking {
    /// `DockStyle.Right`: the panels' column beside the map, the waypoints under it.
    Right,
    /// `DockStyle.Bottom`: the panels along the bottom, the waypoints at the right.
    Bottom,
}

impl Docking {
    /// `panelAction.Dock.ToString()`, the saved text.
    const fn name(self) -> &'static str {
        match self {
            Self::Right => "Right",
            Self::Bottom => "Bottom",
        }
    }
}

/// A combo's open list: below its box, above everything else, thirty rows at most before it
/// scrolls, as this application draws every drop-down. Measured under `id`, so a script can
/// wheel it to an entry past the rows it shows (`reveal main-port-list main-port-TCP`).
fn dropdown(id: &'static str, rows: Vec<gpui::AnyElement>) -> gpui::AnyElement {
    // The measured box is the scrolling container's, the rows it shows - not the union of every
    // row, which reached below the window and told `reveal` an entry was in view when it was not.
    gpui::deferred(
        gpui::anchored().snap_to_window().child(
            probe::measured(id, div()).child(
                div()
                    .id(id)
                    .mt(px(22.0))
                    .flex()
                    .flex_col()
                    .max_h(px(30.0 * 20.0))
                    .overflow_y_scroll()
                    .bg(rgb(theme::PANEL))
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .rounded_sm()
                    .occlude()
                    .children(rows),
            ),
        ),
    )
    .with_priority(2)
    .into_any_element()
}

/// Which of the planning screen's panels to build.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlanGroup {
    /// `panelAction`'s.
    Actions,
    /// `panelWaypoints`': the grid filling the height it is given, the editor beside it - under
    /// the map, or at its right after Switch Docking.
    Grid,
}
use telemetry::{Telemetry, TelemetryView};
use ui::{action, theme};

/// Auto Pan's setting, `CHK_autopan`, as the C#'s `bool.ToString()` writes it: "True" or "False".
/// `// C#: GCSViews/FlightData.cs:1929-1933`
const AUTO_PAN_SETTING: &str = "CHK_autopan";

/// The mission file name used when nothing has been typed.
const DEFAULT_PLAN_FILE: &str = "mission.waypoints";

/// Where dialogs drawn over the whole window hang in the element tree: a box of no size at its
/// parent's corner. Their window-sized backdrops (`deferred`, `anchored` at the window's origin)
/// draw over everything wherever they hang, but laid out inside a screen's body they made the
/// body measure as running past the window, and the layout guard said so whenever a dialog was
/// open (the owner's Mac, 2026-10-05: SETUP's body "cut off" by its own dialog's backdrop).
fn overlay_layer() -> gpui::Div {
    div().absolute().top_0().left_0().size_0()
}

/// Under the product's name in the header (the owner, 2026-10-04).
const BYLINE: &str = "by David Buzz";
/// What the byline says when the pointer is over it (the owner's words, 2026-10-04).
const BYLINE_TIP: &str = ".. and with thanks to the OG Michael Oborne";

/// A path's last part, as `Path.GetFileName` gives it.
fn file_name_of(path: &std::path::Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |name| name.to_string_lossy().into_owned(),
    )
}

/// The name a parameter backup gets if the operator does not choose one.
const DEFAULT_PARAM_FILE: &str = "vehicle.param";

/// Repaint interval when measuring the renderer: as fast as the executor will schedule, so paint
/// cost is measured rather than the timer.
const REFRESH_BENCH: Duration = Duration::from_millis(1);

/// Which screen is showing.
///
/// Mission Planner's tab order, and for the same reason: flying is what the application is for,
/// planning is what you do before flying, and setup is what you do once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Screen {
    /// Flight data: what the aircraft is doing now.
    Fly,
    /// Flight plan: the mission.
    Plan,
    /// Initial setup and calibration: `MainV2`'s SETUP button, `InitialSetup`.
    /// `// C#: MainV2.cs:3179`
    Setup,
    /// Configuration and tuning: `MainV2`'s CONFIG button, beside SETUP, `SoftwareConfig`.
    /// `// C#: MainV2.cs:3180; MainV2.Designer.cs:150, 158`
    Config,
    // ---- SITL ----
    /// Simulation: `MainV2`'s SIMULATION button, beside CONFIG, `GCSViews/SITL.cs`.
    /// `// C#: MainV2.cs:583, 872; MainV2.Designer.cs (MenuSimulation)`
    Sitl,
    // ---- end SITL ----
    /// Help: `MainV2`'s HELP button, beside SIMULATION, `GCSViews/Help.cs`.
    /// `// C#: MainV2.cs:4055-4058; MainV2.Designer.cs:75, 168-174`
    Help,
    /// The vehicle's parameters.
    Params,
    /// Reviewing a dataflash log.
    ///
    /// Mission Planner opens `Log/LogBrowse.cs` as a separate window from a button on the flight
    /// screen (`BUT_logbrowse_Click`, `GCSViews/FlightData.cs:1380`). A single-window application
    /// makes it a tab; what it holds is the same - the list of fields the log declares, and a
    /// chart of the chosen one.
    Logs,
    /// Mission Planner's temp form, Ctrl+F's (`temp.cs`, experimental.rs), as a tab of its own
    /// between LOGS and PLUGINS - the owner's addition, 2026-10-04.
    Experimental,
    /// The plugins: the plugin manager Ctrl+P opens (`Plugin/PluginUI.cs`, plugin_manager.rs) as a
    /// tab of its own, between LOGS and HELP - the owner's addition, 2026-10-04.
    Plugins,
}

impl Screen {
    /// The tabs, in order: PARAMS, LOGS, EXPERIMENTAL, PLUGINS, and HELP the very last (the
    /// owner, 2026-10-04).
    const ALL: [Self; 10] = [
        Self::Fly,
        Self::Plan,
        Self::Setup,
        Self::Config,
        // ---- SITL ----
        Self::Sitl,
        // ---- end SITL ----
        Self::Params,
        Self::Logs,
        Self::Experimental,
        Self::Plugins,
        Self::Help,
    ];

    /// The screen to open on, from `MP_SCREEN`.
    ///
    /// Exists so a screenshot can be taken of any screen without driving the tab strip with
    /// synthetic clicks, which is the sort of test that breaks whenever the layout moves.
    fn initial(requested: Option<&str>, saved: Option<&str>) -> Self {
        let named = match requested {
            Some(name) => name.to_owned(),
            None => match std::env::var("MP_SCREEN") {
                Ok(name) => name,
                Err(_) => saved.unwrap_or_default().to_owned(),
            },
        };
        match named.as_str() {
            "plan" => Self::Plan,
            "setup" => Self::Setup,
            "config" => Self::Config,
            // ---- SITL ----
            "simulation" => Self::Sitl,
            // ---- end SITL ----
            "help" => Self::Help,
            "params" => Self::Params,
            "logs" => Self::Logs,
            "experimental" => Self::Experimental,
            "plugins" => Self::Plugins,
            // Anything else, including nothing and a typo, opens on the flight screen. An operator
            // who mistypes a screen name should still get the one the application is for.
            _ => Self::Fly,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Fly => "fly",
            Self::Plan => "plan",
            Self::Setup => "setup",
            Self::Config => "config",
            // ---- SITL ----
            Self::Sitl => "simulation",
            // ---- end SITL ----
            Self::Help => "help",
            Self::Params => "params",
            Self::Logs => "logs",
            Self::Experimental => "experimental",
            Self::Plugins => "plugins",
        }
    }

    const fn id(self) -> &'static str {
        match self {
            Self::Fly => "tab-fly",
            Self::Plan => "tab-plan",
            Self::Setup => "tab-setup",
            Self::Config => "tab-config",
            // ---- SITL ----
            Self::Sitl => "tab-simulation",
            // ---- end SITL ----
            Self::Help => "tab-help",
            Self::Params => "tab-params",
            Self::Logs => "tab-logs",
            Self::Experimental => "tab-experimental",
            Self::Plugins => "tab-plugins",
        }
    }
}

struct MissionPlanner {
    telemetry: Telemetry,
    map: std::rc::Rc<std::cell::RefCell<MapViewport>>,
    /// Whether to read the mission automatically once a vehicle appears.
    ///
    /// Off by default and on with `--read-mission`. A ground station that silently pulls the
    /// mission on connect makes it impossible to tell whether what is on screen came from the
    /// vehicle or from the operator, which is exactly the confusion that loses a flight plan.
    auto_read_mission: bool,
    /// Whether the parameters have been asked for on this connection: `MAVLinkInterface.Open`'s
    /// `getParamListMavftp` (`:938`), run here once a vehicle is heard and nothing is held.
    params_requested: bool,
    /// Ctrl+T's `comPort.Open(false)`: the link the connect flow opens next is not to fetch the
    /// parameters. `// C#: MainV2.cs:4146-4157`
    blind_connect: bool,
    /// Whether that automatic read has already happened.
    mission_requested: bool,
    /// Which screen is showing.
    screen: Screen,
    /// The mission the operator is editing.
    plan: Plan,
    /// Whether the next completed fence download should replace the fence on screen.
    adopt_vehicle_fence: bool,
    /// Whether the next completed rally download should replace the rally points on screen.
    adopt_vehicle_rally: bool,
    /// Whether the next completed download should replace the plan on screen.
    ///
    /// Set when the operator presses "read from vehicle" and cleared once the items arrive. Without
    /// it, a download started for the map would silently overwrite an edit in progress.
    adopt_vehicle_mission: bool,
    /// The last thing a file operation did, shown so a save is not silent.
    file_status: Option<String>,
    /// The waypoint being dragged on the map, if one is.
    dragging_waypoint: Option<u16>,
    /// The altitude frame new waypoints are created in.
    ///
    /// A screen-level choice that a new item copies, as `CMB_altmode` is. Remembered across runs,
    /// because Mission Planner does: `Settings.Instance.GetInt32("FPaltmode", ...)`.
    /// `// C#: GCSViews/FlightPlanner.cs:102`
    altitude_frame: plan::AltitudeFrame,
    /// Where the left button went down on the map, and whether it has moved since.
    ///
    /// A press that never moves is a click, and a click on empty map adds a waypoint. `MainMap`
    /// decides the same way with `isMouseDraging`, set by any movement while the button is down.
    /// `// C#: GCSViews/FlightPlanner.cs:7436, 7736-7745`
    map_press: Option<(f32, f32)>,
    /// What was loaded from the settings file, and what will be written back to it.
    settings: settings::Settings,
    /// Mission Planner's `config.xml`, as `Settings.Instance` holds it: what the screens keep
    /// there, written whole on the C#'s events.
    persisted: settings::Persisted,
    /// The planning map's right-click menu and the dialogs it opens.
    plan_menus: plan::PlanMenus,
    /// Inject Custom Map's run, while one is on.
    inject_map: Option<inject_map::Injection>,
    /// The built-in HTTP server, `MainV2`'s `httpthread`.
    http: http_server::Host,
    /// What View KML's last click did: the URL opened, or why it was not.
    kml_link: Option<Result<(), String>>,
    /// Focus for those dialogs, which take the keyboard while they show.
    plan_prompt_focus: gpui::FocusHandle,
    /// The Survey (Grid) dialog the map menu's Auto WP opens.
    survey: survey_ui::SurveyUi,
    /// Focus for the Home Location boxes: Lat, Long and ASL.
    plan_home_focus: [gpui::FocusHandle; 3],
    /// Focus for the panel boxes: WP Radius, Loiter Radius and Default Alt.
    plan_panel_focus: plan::PanelFocus,
    /// The mission file name to save to or load from.
    plan_name: textfield::TextField,
    /// Focus for that field.
    plan_name_focus: gpui::FocusHandle,
    /// What has been typed into the parameter search.
    param_search: textfield::TextField,
    /// Focus for that field.
    param_search_focus: gpui::FocusHandle,
    /// The parameter group being browsed.
    selected_param_group: Option<String>,
    /// The parameter being looked at.
    selected_param: Option<String>,
    /// `chk_none_default`: only the parameters off their defaults listed.
    param_none_default: bool,
    /// The `.param` file name to save to, load from or compare against.
    param_file_name: textfield::TextField,
    /// Focus for that field.
    param_file_focus: gpui::FocusHandle,
    /// Joystick state: the device, the mapping and the failsafe.
    sticks: joystick::Sticks,
    /// The keyboard for the joystick page's expo boxes, its forms' boxes and its file dialogs.
    joystick_focus: gpui::FocusHandle,
    /// The primary flight display's inputs for this frame, and the clocks behind its banners.
    hud: hud::HudInputs,
    hud_timing: hud::Timing,
    /// The custom warnings and the loop that checks them (`warnings.rs`).
    warnings: warnings::WarningEngine,
    /// The parameter documentation fetch for the connected firmware.
    metadata: metadata::Fetch,
    /// The live tuning graph.
    tuning: tuning::Tuning,
    /// The RAW Sensor window.
    raw_sensor: raw_sensor::RawSensor,
    /// Initial Setup's Flight Modes page.
    flight_modes: config::flight_modes::FlightModes,
    /// The log being reviewed.
    log_browse: logbrowse::LogBrowse,
    /// The log file name to open.
    log_name: textfield::TextField,
    /// Focus for that field.
    log_name_focus: gpui::FocusHandle,
    /// The LOGS tab's page: Telemetry Logs, DataFlash Logs or Review a Log.
    logs_page: logs_tab::LogsPage,
    /// The debug build's cut-off guard, said in a red strip over the window's foot.
    cut_off: layout_guard::Banner,
    /// The layout tour, under `MP_TOUR`.
    tour: Option<tour::Tour>,
    /// Focus for the log browser's prompt: Ctrl+G's line, a field's scaler, an export's name.
    log_prompt_focus: gpui::FocusHandle,
    /// Focus for the log browser itself, which Ctrl+G is heard through.
    log_screen_focus: gpui::FocusHandle,
    /// The log browser's `txt_info`.
    log_info_focus: gpui::FocusHandle,
    /// What has been typed into the log field search.
    /// The result of the last comparison against a file, newest first.
    ///
    /// Held rather than applied. A comparison is something an operator reads before deciding, and
    /// the decision is a second, deliberate press - loading a tune because it was compared would
    /// be the worst possible reading of "show me what this would change".
    param_differences: Vec<mp_params::param_file::Difference>,
    /// Initial Setup's Radio Calibration page.
    radio_input: config::radio::RadioInput,
    /// Initial Setup's Motor Test page, and its boxes' focus.
    motor_test: config::motor_test::MotorTest,
    motor_focus: config::motor_test::Focus,
    /// Whether this session has turned the vehicle's arming checks off.
    ///
    /// Only to offer putting them back. The parameter is the vehicle's, not ours, so this says
    /// what we did rather than what the vehicle currently holds.
    disabled_arming_checks: bool,
    /// When the last forced arm command went out, so the retry does not flood.
    ///
    /// The armed flag comes from the heartbeat, which is 1 Hz. Retrying at the render rate sends
    /// ten commands before the state can possibly catch up, and the vehicle acknowledges every one
    /// of them - which buries the message log under its own retries.
    last_force_arm: Option<web_time::Instant>,
    /// While a forced arm is in progress, when to stop re-sending it.
    ///
    /// The parameter write that disables the checks takes effect asynchronously, so one arm
    /// command sent straight after it can arrive too early. Re-sending for a couple of seconds
    /// costs nothing and removes the race.
    forcing_arm_until: Option<web_time::Instant>,
    /// The forced arm the link may still be retrying, and when it was made, so the next is sent
    /// only once it has ended.
    force_arm_request: Option<(mp_link::RequestId, web_time::Instant)>,
    /// Lists of parameter writes under way, each made one write at a time.
    param_writes: Vec<params::ParamWrites>,
    /// The last parameter write to end, for the facts.
    last_param_write: Option<params::Written>,
    // ---- row 82 ----
    /// The Full Parameter List's right-hand column: `_changes`, Modified, the tree's collapse,
    /// Reset to Default, Load Presaved and `ParamCompare` over it.
    raw_params: raw_params::RawParams,
    // ---- end row 82 ----
    // ---- ConfigRawParams remainder ----
    /// The Full Parameter List's grid: Fav, the Options control, the typed Value cell and its
    /// questions, Write Params under way, the widths and the splitter, RawParamWarning.
    param_grid: raw_params_grid::RawGrid,
    /// The grid's keyboard focus, for F2, a key that begins typing, and Ctrl+S.
    param_grid_focus: gpui::FocusHandle,
    /// The Value cell's being typed into.
    param_edit_focus: gpui::FocusHandle,
    // ---- end ConfigRawParams remainder ----
    /// Scroll position of the flight screen's panel column, so an indicator can be drawn for it.
    fly_scroll: gpui::ScrollHandle,
    /// The flight screen's Actions tab: what its boxes and lists hold between presses.
    fly_actions: fly::Actions,
    /// Focus for the Actions tab's text boxes.
    fly_focus: fly::ActionsFocus,
    /// Which page of the flight screen's `tabControlactions` is showing.
    fly_pages: fly::Pages,
    /// The main window's own focus, held whenever no control has it, so that a key reaches the
    /// root's `ProcessCmdKey` (cmd_keys.rs): gpui gives a key to the focused element and its
    /// parents, and with nothing focused to the window's top alone.
    root_focus: gpui::FocusHandle,
    /// The EXPERIMENTAL tab's state (experimental.rs).
    experimental: experimental::Experimental,
    /// The flight screen's other state: the Quick and Telemetry Logs pages, the points of
    /// interest, the Log Downloader and the windows the HUD opens.
    fly_data: fly::FlightData,
    /// Scroll position of the plan screen's panel column.
    plan_scroll: gpui::ScrollHandle,
    /// `FP_docking`: where `panelAction` and `panelWaypoints` sit.
    plan_docking: Docking,
    /// `MainV2`'s connection box and CONNECT button.
    connect_box: connect::ConnectBox,
    /// The box a network kind's question is typed into.
    connect_field: textfield::TextField,
    /// Its keyboard focus, and the dialogs' when they have no box.
    connect_focus: gpui::FocusHandle,
    /// Initial Setup's FailSafe page.
    failsafe: config::failsafe::FailSafe,
    /// The keyboard focus of its number being typed into.
    failsafe_focus: gpui::FocusHandle,
    /// The SETUP screen's backstage view: `InitialSetup`'s list and the page chosen from it.
    setup_list: setup::Backstage,
    /// The CONFIG screen's: `SoftwareConfig`'s.
    config_list: setup::Backstage,
    /// Initial Setup's Frame Type page.
    frame_type: config::frame_type::FrameType,
    /// Initial Setup's Battery Monitor page, and its text boxes' focus.
    battery_monitor: config::battery_monitor::BatteryMonitor,
    battery_focus: config::battery_monitor::Focus,
    /// Initial Setup's Install Firmware page, and the firmware catalogue it keeps.
    install_firmware: config::firmware::InstallFirmware,
    /// Initial Setup's Compass page, and its boxes' focus.
    compass: config::compass::Compass,
    compass_focus: config::compass::Focus,
    /// Initial Setup's Servo Output page, and the focus of the number being typed into.
    servo_output: config::servo_output::ServoOutput,
    servo_focus: gpui::FocusHandle,
    /// Initial Setup's Serial Ports page.
    serial_ports: config::serial_ports::SerialPorts,
    /// Initial Setup's ESC Calibration page, and the focus of the number being typed into.
    esc_calibration: config::esc_calibration::EscCalibration,
    esc_focus: gpui::FocusHandle,
    // ---- Mandatory Hardware pages: Accel Calibration (ConfigAccelerometerCalibration), Frame
    // Type before 3.5 (ConfigFrameType), Secure (ConfigSecureAP) ----
    /// Initial Setup's Accel Calibration page.
    accel_calibration: config::accel_calibration::AccelCalibration,
    /// Initial Setup's Frame Type page for a copter older than 3.5.
    frame_type_legacy: config::frame_type_legacy::FrameTypeLegacy,
    /// Initial Setup's Secure page: its key, its boxes and its file dialog.
    secure: config::secure::Secure,
    /// The keyboard focus of that file dialog's path.
    secure_focus: gpui::FocusHandle,
    // ---- end Mandatory Hardware pages ----
    /// CONFIG's Planner page, and its boxes' focus.
    planner: config::planner::Planner,
    planner_focus: config::planner::Focus,
    /// `MainV2.cam`: the capture the Planner page's Start opened, until its Stop.
    /// `// C#: MainV2.cs:514`
    video: Option<mp_video::Capture>,
    /// The capture's latest frame and the image made of it for the HUD, made once per frame.
    video_frame: Option<(
        std::sync::Arc<mp_video::Frame>,
        std::sync::Arc<gpui::RenderImage>,
    )>,
    /// While a capture runs, a repaint at the rate `Capture`'s timer hands the HUD a picture;
    /// dropping it stops the repaints.
    video_repaint: Option<gpui::Task<()>>,
    // Optional Hardware pages: ADSB, Battery Monitor 2, Range Finder, Airspeed, Optical Flow and
    // Camera Gimbal (`config/optional.rs`).
    /// The six page objects.
    optional: config::optional::Optional,
    /// Their boxes' focus.
    optional_focus: config::optional::Focus,
    // end Optional Hardware pages
    // ---- Basic Tuning / Advanced ----
    /// CONFIG's Basic Tuning page for a plane (`ConfigArduplane`); Advanced (`ConfigAdvanced`)
    /// keeps no state.
    basic_tuning: config::basic_tuning::BasicTuning,
    /// The focus of its box being typed into.
    basic_tuning_focus: gpui::FocusHandle,
    // ---- end Basic Tuning / Advanced ----
    // ---- Extended Tuning ----
    /// CONFIG's Extended Tuning page (`ConfigArducopter`), and the focus of the box being typed
    /// into.
    extended_tuning: config::extended_tuning::ExtendedTuning,
    extended_focus: config::extended_tuning::Focus,
    // ---- end Extended Tuning ----
    // ---- GeoFence / rover Basic Tuning / User Params ----
    /// CONFIG's GeoFence, rover Basic Tuning and User Params pages (`config/software_pages.rs`).
    software_pages: config::software_pages::SoftwarePages,
    /// Their boxes' focus.
    software_focus: config::software_pages::Focus,
    // ---- end GeoFence / rover Basic Tuning / User Params ----
    // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
    /// CONFIG's Standard Params, Advanced Params and MAVFtp pages and SETUP's Heli Setup
    /// (`config/software_pages2.rs`).
    software_pages2: config::software_pages2::SoftwarePages2,
    /// Their boxes' focus.
    software2_focus: config::software_pages2::Focus,
    // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
    // ---- SETUP's small pages (row 70) ----
    /// SETUP's Parachute, OSD, CAN GPS Order, HW ID, Compass/Motor Calib and Initial Tune
    /// Parameter pages (`config/extra_setup.rs`).
    extra: config::extra_setup::ExtraSetup,
    /// Their boxes' focus.
    extra_focus: config::extra_setup::Focus,
    // ---- end SETUP's small pages ----
    // ---- RTK/GPS Inject ----
    /// Initial Setup's RTK/GPS Inject page, with its statics and the thread it starts.
    rtk_inject: config::rtk_inject::RtkInject,
    /// Its boxes' focus.
    rtk_focus: config::rtk_inject::Focus,
    // ---- end RTK/GPS Inject ----
    // ---- Firmware Legacy / Ateryx ----
    /// Initial Setup's Install Firmware Legacy page (`ConfigFirmware`).
    firmware_legacy: config::firmware_legacy::FirmwareLegacy,
    /// The focus of the firmware pages' file dialog, both pages' Load custom firmware.
    firmware_focus: gpui::FocusHandle,
    /// The Install Firmware page, for its Ctrl+Q (`ProcessCmdKey`).
    firmware_page_focus: gpui::FocusHandle,
    /// CONFIG's Ateryx Pids page (`ConfigAteryx`), and the focus of its box being typed into.
    ateryx: config::ateryx::Ateryx,
    ateryx_focus: gpui::FocusHandle,
    // ---- end Firmware Legacy / Ateryx ----
    // ---- Geo Reference ----
    /// The Geo Reference Images form (`GeoRef/georefimage.cs`) the DataFlash Logs page opens.
    georef: georef_ui::GeorefUi,
    // ---- end Geo Reference ----
    /// The forms `ProcessCmdKey`'s Ctrl+X, Ctrl+W and Ctrl+J open, and Ctrl+Z's camera test.
    key_forms: cmd_keys::KeyForms,
    // ---- SITL ----
    /// The SIMULATION screen's page object (`GCSViews/SITL.cs`), and its keyboard focus.
    sitl: sitl::Sitl,
    sitl_focus: sitl::Focus,
    /// The Flight Modes page, for its Ctrl+S (`ProcessCmdKey`).
    flight_modes_focus: gpui::FocusHandle,
    // ---- end SITL ----
    /// The HELP screen's page object (`GCSViews/Help.cs`), and the update it may be running.
    help: help::Help,
    /// The crash reports the last runs left and the question about them (`Program.
    /// handleException`), and the message box's keyboard.
    crash: crash::Crash,
    crash_focus: gpui::FocusHandle,
    // ---- row 96 ----
    /// The WebAssembly plugins (`PluginLoader.Plugins`) and what the window shows of them.
    plugins: plugins_ui::Plugins,
    // ---- end row 96 ----
}

impl MissionPlanner {
    fn new(
        target: Option<String>,
        read_mission: bool,
        screen: Screen,
        cx: &mut Context<Self>,
    ) -> Self {
        // Mission Planner's config.xml, read once as `Settings.Instance` is, before anything
        // below reads it: the map's access mode is one of its keys. `// C#: MainV2.cs:782-808`
        let mut persisted = settings::Persisted::load();
        // The screens' culture, from its `language`, before the first screen asks for a word -
        // the flight screen's state below already does.
        // `// C#: MainV2.cs:660-661, 697-700; L10N.cs:12-25`
        i18n::init(persisted.get(i18n::SETTING));
        // A log to replay plays as the Telemetry Logs page plays one: at its own pace, under the
        // page's controls. `// C#: GCSViews/FlightData.cs:669-701`
        // MP_STORM puts a synthetic vehicle behind the screens in place of any link: storm.rs.
        let mut fly_data = fly::FlightData::new();
        // The link this start opens, which Mission Planner's Connect would save.
        let opened = target.clone().filter(|_| !storm::enabled());
        // ---- Display view (row 71) ----
        // `MainV2.DisplayConfiguration` as its start-up makes it, before any list is built.
        // `// C#: MainV2.cs:351-353, 898-933`
        display_view::start(&mut persisted);
        // ---- end Display view ----
        // `CurrentState`'s statics, as `MainV2`'s start-up sets them from config.xml: the
        // telemetry rates' saved defaults, the custom fields' names, the planned home put back to
        // 0,0,0 when it is off the globe, and the K-index - today's saved one, or a download on a
        // thread of its own. Set before the link opens: a vehicle takes its `rate*` from the
        // backups as it is made, on the link's thread, at its first heartbeat - which came before
        // this block when it sat after the connect, so the saved `CMB_raterc` reached the vehicle
        // only when start-up won the race. `// C#: MainV2.cs:981-1000, 1010-1028, 3306, 3940-3962`
        mp_vehicle::StreamRates::set_backups(persisted.rate_backups());
        for (index, name) in persisted.custom_field_names() {
            mp_vehicle::VehicleState::add_custom_field_name(index, &name);
        }
        let planned = plan::planned_home_from_config(Some(persisted.config()));
        mp_vehicle::VehicleState::set_planned_home(mp_vehicle::LatLngAlt {
            lat: planned.lat,
            lng: planned.lng,
            alt: planned.alt,
        });
        if persisted.kindex_at_start(&settings::short_date_today()) {
            settings::download_kindex(mp_firmware::manifest::Http);
        }
        let telemetry = match (storm::telemetry(), target) {
            (Some(storm), _) => storm,
            (None, Some(url)) => match url.parse::<mp_transport::LinkUrl>() {
                Ok(mp_transport::LinkUrl::File { path }) => match fly::replay(&path) {
                    Ok((telemetry, control)) => {
                        fly_data.playback.load(&path, control);
                        telemetry
                    }
                    Err(_) => Telemetry::connect(&url),
                },
                _ => Telemetry::connect(&url),
            },
            (None, None) => Telemetry::idle(),
        };

        // Repaint on a timer. The link thread owns the data and publishes snapshots; the UI only
        // ever reads one, so this cannot block on I/O. MP_REPAINT chooses another way, to be
        // measured: repaint.rs.
        cx.spawn(async move |this, cx| {
            let policy = repaint::Policy::chosen(
                std::env::var("MP_BENCH").is_ok().then_some(REFRESH_BENCH),
                storm::enabled().then_some(storm::REFRESH),
            );
            let mut watch = repaint::Watch::new(web_time::Instant::now());
            loop {
                cx.background_executor().timer(policy.between_looks()).await;
                let looked = this.update(cx, |this, cx| {
                    let now = web_time::Instant::now();
                    let (mark, asked) = match policy {
                        repaint::Policy::Tick(_) => (0, false),
                        repaint::Policy::Data { .. } => (
                            this.telemetry.change_mark() ^ mp_os::wakes().rotate_left(32),
                            repaint::take_due(now),
                        ),
                    };
                    if watch.repaint(policy, mark, asked, now) {
                        cx.notify();
                    }
                });
                if looked.is_err() {
                    break;
                }
            }
        })
        .detach();

        // No timer for the sticks. They are read and sent on threads of their own in `mp_input`
        // (Deliverable 15's 5 ms from stick to wire is not reachable from a timer on this executor, and a
        // slow frame must not become a control problem); the screen only keeps the reader
        // pointed at the vehicle, once a frame in `render`.

        // The synthetic scene is off unless asked for. MP_TRACK_POINTS=100000 MP_MARKERS=2000
        // MP_MAP_DEMO=1 reproduces the benchmark the renderer was measured with; those were the
        // defaults, which meant connecting to a real flight controller indoors filled the map
        // with a hundred thousand points of meaningless squiggle.
        let track_points = std::env::var("MP_TRACK_POINTS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        let markers = std::env::var("MP_MARKERS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);

        // Map imagery. Off with MP_NO_TILES, which is how the offline behaviour gets exercised
        // and how a screenshot avoids depending on a tile server being up.
        let mut map = MapViewport::new(track_points, markers);
        // `FlightData.Activate`: the map opens where it was last left.
        // `// C#: GCSViews/FlightData.cs:524-548`
        if let Some((at, zoom)) = persisted.flight_map_start() {
            map.start_at(at, zoom);
        }
        // `CHK_autopan.Checked = Settings.Instance.GetBoolean("CHK_autopan")` when it is set.
        // `// C#: GCSViews/FlightData.cs:2732-2733`
        if let Some(ticked) = persisted.get(AUTO_PAN_SETTING) {
            map.set_auto_pan(raw_params::get_boolean(Some(ticked)));
        }
        if std::env::var("MP_NO_TILES").is_err() {
            let cache = TileCache::new(TileCache::default_root());
            // The environment wins over the remembered choice, so a screenshot or a test can
            // pin a provider without disturbing what the operator picked. With neither, Mission
            // Planner's default, GoogleSatelliteMap.
            // `// C#: GCSViews/FlightPlanner.cs:7282`
            let source = std::env::var("MP_TILE_SOURCE")
                .ok()
                .or_else(|| settings::Settings::load().tile_source)
                .and_then(|id| mp_tiles::source::source_by_id(&id))
                .unwrap_or_else(mp_tiles::source::default_source);
            // `mapCache`, the Planner page's Map Access Mode. `// C#: Program.cs:321-325`
            let store =
                if std::env::var("MP_OFFLINE").is_ok() || config::planner::cache_only(&persisted) {
                    TileStore::offline(source, cache)
                } else {
                    TileStore::new(source, cache)
                };
            map.set_tiles(std::sync::Arc::new(store));
        }

        // From Mission Planner's config.xml: the home the planning screen remembers, which
        // `MainV2` reads at start-up, the panel boxes it saved, which `FlightPlanner_Load` reads,
        // and the quick views `FlightData.Activate` binds. Then the link opened above, as
        // Connect's `Open` puts its port and host in the dictionary.
        let mut plan = Plan::default();
        plan.set_planned_home(plan::planned_home_from_config(Some(persisted.config())));
        plan.apply_panel_config(Some(persisted.config()));
        plan.apply_mavftp_config(Some(persisted.config()));
        persisted.restore_quick_views(&mut fly_data.quick);
        fly_data.hud_settings.load_icons(&persisted);
        if let Some(url) = &opened {
            persisted.link_opened(url);
        }
        // The Planner page's keys `MainV2` sets up from before any page shows: the units, the
        // telemetry rates and the GCS id. `// C#: MainV2.cs:683, 836, 981-1002`
        let planner = config::planner::Planner::new(&persisted);
        // `if (Settings.Instance["FP_docking"] == "Bottom") switchDockingToolStripMenuItem_Click`.
        // `// C#: GCSViews/FlightPlanner.cs:3487-3490`
        let plan_docking = if persisted.get("FP_docking") == Some("Bottom") {
            Docking::Bottom
        } else {
            Docking::Right
        };
        // `CMB_serialport` and `CMB_baudrate` as the settings left them: `comport`, its baud.
        // `// C#: MainV2.cs:961-975`
        let connect_box = connect::ConnectBox::new(
            persisted.get("comport").unwrap_or_default(),
            persisted.baud(),
        );
        // ---- row 96 ----
        // `PluginLoader.LoadAll`, less `DisabledPlugins`. `// C#: MainV2.cs:3185-3196`
        // `loadTabControlActions`: the flight screen's pages as the setting saved them.
        // `// C#: GCSViews/FlightData.cs:733-791`
        let mut fly_pages = fly::Pages::default();
        if let Some(saved) = persisted.get(fly::TAB_SETTING) {
            fly_pages.load_tab_control_actions(saved);
        }
        let plugins = plugins_ui::Plugins::start(&persisted, cx);
        // ---- end row 96 ----
        // `WarningEngine`'s `LoadConfig`, and speech as the settings left it.
        // `// C#: MainV2.cs:1035-1036`
        let warnings = warnings::WarningEngine::start(&persisted);
        let mut this = Self {
            telemetry,
            map: std::rc::Rc::new(std::cell::RefCell::new(map)),
            // `loadwpsonconnect`, the Planner page's Load Waypoints on connect.
            // `// C#: MainV2.cs:1750-1759`
            auto_read_mission: read_mission || config::planner::load_wps_on_connect(&persisted),
            mission_requested: false,
            params_requested: false,
            blind_connect: false,
            screen,
            plan,
            adopt_vehicle_mission: false,
            adopt_vehicle_fence: false,
            adopt_vehicle_rally: false,
            file_status: None,
            dragging_waypoint: None,
            // `CMB_altmode` as `config(false)` restores it, else this application's own choice.
            // `// C#: GCSViews/FlightPlanner.cs:2609-2610`
            altitude_frame: persisted.altitude_frame().unwrap_or_else(|| {
                plan::AltitudeFrame::from_key(
                    settings::Settings::load()
                        .altitude_frame
                        .as_deref()
                        .unwrap_or_default(),
                )
            }),
            map_press: None,
            settings: settings::Settings::load(),
            persisted,
            plan_name: {
                let mut field = textfield::TextField::new("mission.waypoints");
                field.set(DEFAULT_PLAN_FILE);
                field
            },
            plan_name_focus: cx.focus_handle(),
            plan_menus: plan::PlanMenus::default(),
            inject_map: None,
            http: http_server::Host::start(),
            kml_link: None,
            plan_prompt_focus: cx.focus_handle(),
            survey: survey_ui::SurveyUi::new(cx),
            plan_home_focus: [cx.focus_handle(), cx.focus_handle(), cx.focus_handle()],
            plan_panel_focus: plan::PanelFocus::new(cx),
            param_search: textfield::TextField::new("search parameters"),
            param_search_focus: cx.focus_handle(),
            selected_param_group: None,
            selected_param: None,
            param_none_default: false,
            param_file_name: {
                let mut field = textfield::TextField::new(DEFAULT_PARAM_FILE);
                field.set(DEFAULT_PARAM_FILE);
                field
            },
            param_file_focus: cx.focus_handle(),
            sticks: joystick::Sticks::new(),
            joystick_focus: cx.focus_handle(),
            hud: hud::HudInputs::default(),
            hud_timing: hud::Timing::default(),
            warnings,
            metadata: metadata::Fetch::default(),
            tuning: tuning::Tuning::new(),
            raw_sensor: raw_sensor::RawSensor::new(),
            flight_modes: config::flight_modes::FlightModes::default(),
            log_browse: logbrowse::LogBrowse::new(),
            log_name: textfield::TextField::new("a .BIN or .log in the plan directory"),
            log_name_focus: cx.focus_handle(),
            logs_page: logs_tab::LogsPage::default(),
            cut_off: layout_guard::Banner::default(),
            tour: tour::Tour::from_env(),
            log_prompt_focus: cx.focus_handle(),
            log_screen_focus: cx.focus_handle(),
            log_info_focus: cx.focus_handle(),
            param_differences: Vec::new(),
            radio_input: config::radio::RadioInput::default(),
            motor_test: config::motor_test::MotorTest::default(),
            motor_focus: config::motor_test::Focus::new(cx),
            disabled_arming_checks: false,
            forcing_arm_until: None,
            last_force_arm: None,
            force_arm_request: None,
            param_writes: Vec::new(),
            last_param_write: None,
            // ---- row 82 ----
            raw_params: raw_params::RawParams::default(),
            // ---- end row 82 ----
            // ---- ConfigRawParams remainder ----
            param_grid: raw_params_grid::RawGrid::default(),
            param_grid_focus: cx.focus_handle(),
            param_edit_focus: cx.focus_handle(),
            // ---- end ConfigRawParams remainder ----
            fly_scroll: gpui::ScrollHandle::new(),
            plan_scroll: gpui::ScrollHandle::new(),
            plan_docking,
            connect_box,
            connect_field: textfield::TextField::new(""),
            connect_focus: cx.focus_handle(),
            fly_actions: fly::Actions::default(),
            fly_focus: fly::ActionsFocus::new(cx),
            fly_pages,
            root_focus: cx.focus_handle(),
            experimental: experimental::Experimental::default(),
            fly_data,
            failsafe: config::failsafe::FailSafe::default(),
            failsafe_focus: cx.focus_handle(),
            setup_list: setup::Backstage::new(setup::List::Setup),
            config_list: setup::Backstage::new(setup::List::Config),
            frame_type: config::frame_type::FrameType::default(),
            battery_monitor: config::battery_monitor::BatteryMonitor::default(),
            battery_focus: config::battery_monitor::Focus::new(cx),
            install_firmware: config::firmware::InstallFirmware::default(),
            compass: config::compass::Compass::default(),
            compass_focus: config::compass::Focus::new(cx),
            servo_output: config::servo_output::ServoOutput::default(),
            servo_focus: cx.focus_handle(),
            serial_ports: config::serial_ports::SerialPorts::default(),
            esc_calibration: config::esc_calibration::EscCalibration::default(),
            esc_focus: cx.focus_handle(),
            // ---- Mandatory Hardware pages ----
            accel_calibration: config::accel_calibration::AccelCalibration::default(),
            frame_type_legacy: config::frame_type_legacy::FrameTypeLegacy::default(),
            secure: config::secure::Secure::default(),
            secure_focus: cx.focus_handle(),
            // ---- end Mandatory Hardware pages ----
            planner,
            planner_focus: config::planner::Focus::new(cx),
            video: None,
            video_frame: None,
            video_repaint: None,
            // Optional Hardware pages
            optional: config::optional::Optional::default(),
            optional_focus: config::optional::Focus::new(cx),
            // end Optional Hardware pages
            // ---- Basic Tuning / Advanced ----
            basic_tuning: config::basic_tuning::BasicTuning::default(),
            basic_tuning_focus: cx.focus_handle(),
            // ---- end Basic Tuning / Advanced ----
            // ---- Extended Tuning ----
            extended_tuning: config::extended_tuning::ExtendedTuning::default(),
            extended_focus: config::extended_tuning::Focus::new(cx),
            // ---- end Extended Tuning ----
            // ---- GeoFence / rover Basic Tuning / User Params ----
            software_pages: config::software_pages::SoftwarePages::default(),
            software_focus: config::software_pages::Focus::new(cx),
            // ---- end GeoFence / rover Basic Tuning / User Params ----
            // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
            software_pages2: config::software_pages2::SoftwarePages2::default(),
            software2_focus: config::software_pages2::Focus::new(cx),
            // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
            // ---- SETUP's small pages (row 70) ----
            extra: config::extra_setup::ExtraSetup::default(),
            extra_focus: config::extra_setup::Focus::new(cx),
            // ---- end SETUP's small pages ----
            // ---- RTK/GPS Inject ----
            rtk_inject: config::rtk_inject::RtkInject::default(),
            rtk_focus: config::rtk_inject::Focus::new(cx),
            // ---- end RTK/GPS Inject ----
            // ---- Firmware Legacy / Ateryx ----
            firmware_legacy: config::firmware_legacy::FirmwareLegacy::default(),
            firmware_focus: cx.focus_handle(),
            firmware_page_focus: cx.focus_handle(),
            ateryx: config::ateryx::Ateryx::default(),
            ateryx_focus: cx.focus_handle(),
            // ---- end Firmware Legacy / Ateryx ----
            // ---- Geo Reference ----
            georef: georef_ui::GeorefUi::new(cx),
            // ---- end Geo Reference ----
            key_forms: cmd_keys::KeyForms::new(cx),
            // ---- SITL ----
            sitl: sitl::Sitl::new(),
            sitl_focus: sitl::Focus::new(cx),
            flight_modes_focus: cx.focus_handle(),
            // ---- end SITL ----
            help: help::Help::new(),
            crash: crash::Crash::new(mp_settings::data_directory().as_deref()),
            crash_focus: cx.focus_handle(),
            // ---- row 96 ----
            plugins,
            // ---- end row 96 ----
        };
        // Opening on the planning screen activates it, as switching to it does.
        if this.screen == Screen::Plan {
            plan::activate(&mut this);
        }
        // ---- SITL ----
        if this.screen == Screen::Sitl {
            this.sitl_activate();
        }
        // ---- end SITL ----
        if this.screen == Screen::Help {
            this.help.activate(&this.persisted);
        }
        // `MainV2`'s update check, once a day, on a thread of its own.
        // `// C#: MainV2.cs:3661-3671`
        this.help.startup_check(&mut this.persisted);
        // `SaveConfig` at the end of `MainV2`'s constructor, "to test we have write access" - and
        // Connect's, for the link opened above.
        // `// C#: MainV2.cs:1106-1107, 1841-1847`
        this.save_config(settings::SaveEvent::Startup);
        this
    }

    /// `MainV2.SaveConfig`. Its failure the C# shows in a message box; here, the status line.
    /// `// C#: MainV2.cs:2219-2237`
    fn save_config(&mut self, event: settings::SaveEvent) {
        if let Err(err) = self.persisted.save_config(event) {
            self.file_status = Some(format!(
                "could not save Mission Planner's config.xml: {err}"
            ));
        }
    }

    /// `MainV2_FormClosing`, on the window's close box: `MyView.Dispose` deactivates the screen
    /// showing - the planning screen's is `config(true)` - and `SaveConfig` writes the file.
    /// (It also saves the window's size and place, which this application keeps in its own
    /// settings, and not on closing.)
    /// `// C#: MainV2.cs:2010-2015, 2129-2132, 2170-2171; ExtLibs/Controls/MainSwitcher.cs:249-254`
    fn form_closing(&mut self) {
        if self.screen == Screen::Plan {
            self.persisted
                .planner_deactivated(&self.plan, self.altitude_frame);
        }
        if self.screen == Screen::Fly {
            self.persisted
                .flight_data_deactivated(self.map.borrow().position_and_zoom());
        }
        self.persisted.observe_quick_views(&self.fly_data.quick);
        self.save_config(settings::SaveEvent::Close);
        // "closing httpthread": `httpserver.Stop()`. `// C#: MainV2.cs:2104-2108`
        self.http.server.stop();
    }

    /// The folder missions, logs and the dialogs' files start in: `MP_PLAN_DIR` (the harness's),
    /// else the planner's data folder - Mission Planner's dialogs open in the documents folder -
    /// and only failing both the folder it was started in, where a mission once landed in the
    /// source tree it was run from (the owner, 2026-10-04).
    fn plan_directory() -> std::path::PathBuf {
        std::env::var("MP_PLAN_DIR").map_or_else(
            |_| {
                mp_settings::user_data_directory()
                    .or_else(|| std::env::current_dir().ok())
                    .unwrap_or_else(|| std::path::PathBuf::from("."))
            },
            std::path::PathBuf::from,
        )
    }

    /// Where the planner's file dialogs open: `MP_PLAN_DIR` when it is set, else the folder a
    /// mission was last loaded from or saved to (`WPFileDirectory`), else the plan directory.
    /// `// C#: GCSViews/FlightPlanner.cs:1821-1822, 6074`
    fn dialog_directory(&self) -> std::path::PathBuf {
        if std::env::var_os("MP_PLAN_DIR").is_none()
            && let Some(folder) = self
                .persisted
                .get("WPFileDirectory")
                .map(std::path::PathBuf::from)
                .filter(|folder| folder.os_is_dir())
        {
            return folder;
        }
        Self::plan_directory()
    }

    /// `Settings.Instance["WPFileDirectory"] = Path.GetDirectoryName(file)`: where the next
    /// dialog opens.
    /// `// C#: GCSViews/FlightPlanner.cs:1830, 6080`
    fn remember_dialog_directory(&mut self, file: &std::path::Path) {
        if let Some(folder) = file.parent() {
            self.persisted
                .set("WPFileDirectory", folder.to_string_lossy().into_owned());
        }
    }

    /// Where the window is, for the layout guard's record: the screen, and on FLIGHT DATA, SETUP,
    /// CONFIG and LOGS the page within it, on FLIGHT PLAN the Survey (Grid) dialog when it shows.
    fn place(&self) -> String {
        let screen = self.screen.label();
        match self.screen {
            Screen::Fly => format!("{screen}/{}", self.fly_pages.selected().id()),
            Screen::Setup | Screen::Config => {
                let list = if self.screen == Screen::Setup {
                    setup::List::Setup
                } else {
                    setup::List::Config
                };
                match self.backstage_page_class(list) {
                    Some(class) => format!("{screen}/{class}"),
                    None => screen.to_owned(),
                }
            }
            Screen::Logs => format!("{screen}/{}", self.logs_page.name()),
            Screen::Plan if self.survey.is_open() => format!("{screen}/Survey (Grid)"),
            _ => screen.to_owned(),
        }
    }

    /// The layout tour's frame: once the vehicle's parameters are in, the next stop when the one
    /// showing has had its time, and the application closed after the last (`crate::tour`).
    fn tour_step(&mut self, cx: &mut Context<Self>) {
        let Some(mut tour) = self.tour.take() else {
            return;
        };
        // The tour's stops are timed: looked at as the timer looked (repaint.rs).
        crate::repaint::in_flight();
        let now = web_time::Instant::now();
        let view = self.telemetry.view();
        if !tour.waiting(view.parameters.len(), usize::from(view.parameters_expected), now) {
            if !tour.has_begun() {
                let fly_pages = self.fly_pages.shown().to_vec();
                tour.begin(tour::stops(&fly_pages), now);
            }
            match tour.next(now) {
                // The list's pages, now its screen has shown and built it; the first next frame.
                Some(Some(tour::Stop::Pages(list))) => {
                    let pages = self.backstage_pages(list);
                    tour.expand(tour::pages(list, &pages), now);
                }
                Some(Some(stop)) => self.tour_show(stop),
                Some(None) => {
                    log::warn!("layout tour done: {}", tour.progress());
                    cx.quit();
                }
                None => {}
            }
        }
        facts::record("tour.at", tour.progress());
        self.tour = Some(tour);
    }

    /// Shows one of the tour's stops, as the tab or the list entry would.
    fn tour_show(&mut self, stop: tour::Stop) {
        match stop {
            tour::Stop::Screen(screen) => self.choose_screen(screen),
            tour::Stop::Fly(page) => self.fly_pages.select(page),
            tour::Stop::Backstage(list, index) => self.choose_page(list, index),
            tour::Stop::Pages(_) => {}
            tour::Stop::Logs(page) => self.logs_page = page,
        }
    }

    /// The mission file's name, `wpfilename`, which Save File's dialog opens on.
    fn plan_file_name(&self) -> String {
        let name = self.plan_name.value().trim();
        if name.is_empty() {
            DEFAULT_PLAN_FILE.to_owned()
        } else {
            name.to_owned()
        }
    }

    /// Writes the plan to `path` in QGC WPL 110 format, the one every ground station reads:
    /// `savewaypoints` once its dialog has returned.
    fn save_plan_to(&mut self, path: &std::path::Path) {
        // Home at record 0 from the Home Location boxes, then the rows: `savewaypoints`.
        let text = self.plan.waypoints_file();
        self.file_status = match mp_os::fs::write(path, text) {
            Ok(()) => {
                self.plan_name.set(file_name_of(path));
                page_files::saved(path);
                Some(format!(
                    "saved {} items to {}",
                    self.plan.items().len(),
                    path.display()
                ))
            }
            Err(err) => Some(format!("could not save to {}: {err}", path.display())),
        };
    }

    /// Reads a waypoint file: `readQGC110wpfile`, once Load File's dialog has returned it.
    fn load_plan_from(&mut self, path: &std::path::Path) {
        let text = match mp_os::fs::read_to_string(path) {
            Ok(text) => text,
            Err(err) => {
                self.file_status = Some(format!("could not read {}: {err}", path.display()));
                return;
            }
        };
        match mp_mission::read_waypoints(&text) {
            Ok(items) => {
                let name = file_name_of(path);
                // Item 0 is home and leaves the rows; if it is not the boxes' home, the
                // operator is asked whether to take it.
                if let Some(home) = self.plan.adopt_from_file(name.clone(), &items) {
                    self.plan_menus.offer_home_reset(home);
                }
                // `processToScreen` ends with `setWPParams`.
                // `// C#: GCSViews/FlightPlanner.cs:5630`
                self.plan.set_wp_params(&self.telemetry.view().parameters);
                // `wpfilename = file`: what Save File opens on next.
                self.plan_name.set(name);
                let count = self.plan.items().len();
                self.file_status = Some(format!("loaded {count} items from {}", path.display()));
            }
            Err(err) => {
                self.file_status = Some(format!("{} is not a mission file: {err}", path.display()));
            }
        }
    }

    /// Where a named parameter file lives.
    ///
    /// The same directory as missions and by the same rule: a name, not a path, so a typed
    /// separator cannot write outside it.
    fn param_path(&self) -> std::path::PathBuf {
        let name = self.param_file_name.value().trim();
        let leaf = name
            .rsplit(['/', '\\'])
            .next()
            .filter(|part| !part.is_empty() && *part != "." && *part != "..")
            .unwrap_or(DEFAULT_PARAM_FILE);
        let leaf = if leaf.contains('.') {
            leaf.to_owned()
        } else {
            format!("{leaf}.param")
        };
        Self::plan_directory().join(leaf)
    }

    /// The parameters currently held for the vehicle, as a file would hold them.
    fn params_as_file(&self) -> mp_params::param_file::ParamFile {
        let view = self.telemetry.view();
        mp_params::param_file::ParamFile::from_values(
            view.parameters
                .iter()
                .map(|(name, value)| (name.clone(), *value)),
        )
    }

    /// Writes the vehicle's parameters to a file.
    ///
    /// Refused while the download is incomplete. A backup missing four hundred parameters is
    /// indistinguishable from a complete one once it is on disk, and it will be found and loaded
    /// by somebody who believes it is a backup.
    fn save_params(&mut self) {
        let view = self.telemetry.view();
        let expected = usize::from(view.parameters_expected);
        let held = view.parameters.len();
        if held == 0 {
            self.file_status = Some("no parameters to save - download them first".to_owned());
            return;
        }
        if expected > held {
            self.file_status = Some(format!(
                "only {held} of {expected} parameters are here; wait for the download to finish"
            ));
            return;
        }
        let path = self.param_path();
        let file = self.params_as_file();
        self.file_status = match file.save(&path) {
            Ok(()) => {
                page_files::saved(&path);
                Some(format!(
                    "saved {} of {held} parameters to {}",
                    file.len(),
                    path.display()
                ))
            }
            Err(err) => Some(format!("could not save to {}: {err}", path.display())),
        };
    }

    /// Compares a file against the vehicle, without changing anything.
    fn compare_params(&mut self) {
        let path = self.param_path();
        let proposed = match mp_params::param_file::ParamFile::load(&path) {
            Ok(file) => file,
            Err(err) => {
                self.param_differences.clear();
                self.file_status = Some(format!("could not read {}: {err}", path.display()));
                return;
            }
        };
        let current = self.params_as_file();
        if current.is_empty() {
            self.file_status = Some("no parameters from the vehicle to compare against".to_owned());
            return;
        }
        self.param_differences = current.compare(&proposed);
        let unreadable = proposed.rejected().len();
        self.file_status = Some(match (self.param_differences.len(), unreadable) {
            (0, 0) => format!("the vehicle already matches {}", path.display()),
            (0, bad) => format!(
                "the vehicle matches {}, but {bad} lines of it could not be read",
                path.display()
            ),
            (n, 0) => format!("{n} parameters differ from {}", path.display()),
            (n, bad) => format!(
                "{n} parameters differ from {}; {bad} lines could not be read",
                path.display()
            ),
        });
    }

    /// Writes the differences found by the last comparison to the vehicle.
    ///
    /// Only what the comparison found, and only what the vehicle already has. A parameter in the
    /// file that this firmware does not know is skipped rather than sent: ArduPilot ignores a set
    /// for an unknown name silently, so sending it would report success for nothing happening.
    ///
    /// One at a time, each waiting for the vehicle's echo and sent again until it comes, as the
    /// C#'s Write Params loop does; the status line follows it and ends with the C#'s summary.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:313-371`
    fn apply_params(&mut self) {
        if self.param_differences.is_empty() {
            self.file_status = Some("compare a file first - there is nothing to apply".to_owned());
            return;
        }
        let mut writes = Vec::new();
        let mut skipped = 0usize;
        for difference in &self.param_differences {
            match difference.kind {
                mp_params::param_file::Change::Changed { to, .. } => {
                    writes.push((difference.name.clone(), to));
                }
                mp_params::param_file::Change::Added { .. }
                | mp_params::param_file::Change::Missing { .. } => skipped += 1,
            }
        }
        // The comparison is now stale - it describes a vehicle that no longer exists. Cleared
        // rather than left on screen, because a list of differences beside an "apply" button that
        // has already been pressed invites pressing it again.
        self.param_differences.clear();
        self.start_param_writes(params::ParamWrites::apply(writes, skipped));
    }

    /// Switches the map to another tile provider, and remembers it.
    ///
    /// Mission Planner's `comboBoxMapType`, which sits on the planning screen and changes the
    /// flight screen's map with it - `FlightData.mymap.MapProvider` is set from the same handler -
    /// so an operator picks imagery once rather than twice.
    /// `// C#: GCSViews/FlightPlanner.cs:2209-2237`
    fn set_tile_source(&mut self, source: &'static mp_tiles::source::TileSource) {
        if std::env::var("MP_NO_TILES").is_ok() {
            return;
        }
        let cache = TileCache::new(TileCache::default_root());
        // A provider with no URL - Custom - is its cache and nothing else.
        let store = if std::env::var("MP_OFFLINE").is_ok()
            || !source.fetches()
            || config::planner::cache_only(&self.persisted)
        {
            TileStore::offline(source, cache)
        } else {
            TileStore::new(source, cache)
        };
        self.map.borrow_mut().set_tiles(std::sync::Arc::new(store));
        // `Settings.Instance["MapType"] = comboBoxMapType.Text`, saved with the rest later.
        self.persisted.map_type_changed(source);
        self.settings.tile_source = Some(source.id.to_owned());
        if let Err(err) = self.settings.save() {
            self.file_status = Some(format!("could not remember the map provider: {err}"));
        }
    }

    /// Which provider the map is showing.
    fn tile_source_id(&self) -> Option<&'static str> {
        self.map.borrow().source_id()
    }

    /// Chooses the altitude frame new waypoints are created in, and remembers it.
    ///
    /// Remembered because Mission Planner remembers it (`FPaltmode`), and because a planner that
    /// forgets makes every session start in relative - which is the safe default and the wrong
    /// one for somebody who plans over terrain every time.
    /// `switchDockingToolStripMenuItem_Click`: `panelAction` between the right (131 wide, the
    /// waypoints along the bottom, 166 high) and the bottom (120 high, the waypoints at the
    /// right, half the width), the choice kept as `FP_docking`.
    /// `// C#: GCSViews/FlightPlanner.cs:6762-6778`
    fn toggle_docking(&mut self) {
        self.plan_docking = match self.plan_docking {
            Docking::Right => Docking::Bottom,
            Docking::Bottom => Docking::Right,
        };
        self.persisted.set("FP_docking", self.plan_docking.name());
    }

    /// `ConnectionControl` and `MenuConnect`, right-aligned in the menu strip: the port box, the
    /// baud box (off for the kinds without one, and both off while connected, as `IsConnected`
    /// sets them), and the button, CONNECT or DISCONNECT by the link.
    /// `// C#: MainV2.Designer.cs:176-190; Controls/ConnectionControl.Designer.cs; Controls/ConnectionControl.cs:24-30`
    fn connection_controls(
        &self,
        view: &TelemetryView,
        cx: &mut Context<Self>,
    ) -> impl IntoElement {
        let connected = view.connected && !view.target.starts_with("file:");
        let port = self.connect_box.port.clone();
        let baud_on = connect::baud_enabled(&port) && !connected;
        let mut strip = div().flex().items_center().gap_2().pb_2();
        // `cmb_Connection`: the choice, and its list below it when clicked.
        strip = strip.child(
            probe::measured("main-port", div())
                .id("main-port")
                .relative()
                .px_2()
                .py(px(1.0))
                .min_w(px(121.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(if connected { theme::DIM } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(if port.is_empty() {
                    "port".to_owned()
                } else {
                    port.clone()
                })
                .on_click(cx.listener(|this, _event, _window, cx| {
                    // `CMB_serialport_Click`: the list filled afresh, the old choice kept if
                    // it is still there.
                    // `// C#: MainV2.cs:1283-1290`
                    if this.telemetry.view().connected {
                        return;
                    }
                    let serial: Vec<String> = mp_transport::list_ports()
                        .into_iter()
                        .map(|port| port.name)
                        .collect();
                    this.connect_box.ports = connect::port_list(&serial);
                    this.connect_box.ports_open = !this.connect_box.ports_open;
                    this.connect_box.bauds_open = false;
                    cx.notify();
                }))
                .children(self.connect_box.ports_open.then(|| {
                    let rows: Vec<gpui::AnyElement> = self
                        .connect_box
                        .ports
                        .iter()
                        .map(|name| {
                            let choice = name.clone();
                            let id = format!("main-port-{}", name.replace('/', "-"));
                            probe::measured(id.clone(), div())
                                .id(gpui::SharedString::from(id))
                                .px_2()
                                .py(px(2.0))
                                .text_xs()
                                .text_color(rgb(theme::TEXT))
                                .cursor_pointer()
                                .hover(|style| style.bg(rgb(theme::BORDER)))
                                .child(name.clone())
                                .on_click(cx.listener(move |this, _event, _window, cx| {
                                    // `CMB_serialport_SelectedIndexChanged`: `comPortName`,
                                    // and the baud saved for the port put back.
                                    // `// C#: MainV2.cs:1962-1984`
                                    this.connect_box.port.clone_from(&choice);
                                    this.persisted.select_port(&choice);
                                    this.connect_box.baud = this.persisted.baud().to_owned();
                                    this.connect_box.ports_open = false;
                                    cx.notify();
                                }))
                                .into_any_element()
                        })
                        .collect();
                    dropdown("main-port-list", rows)
                })),
        );
        // `cmb_Baud`: its text, and the sixteen rates below it when clicked.
        strip = strip.child(
            probe::measured("main-baud", div())
                .id("main-baud")
                .relative()
                .px_2()
                .py(px(1.0))
                .min_w(px(70.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(if baud_on { theme::TEXT } else { theme::DIM }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(self.connect_box.baud.clone())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    if !baud_on {
                        return;
                    }
                    this.connect_box.bauds_open = !this.connect_box.bauds_open;
                    this.connect_box.ports_open = false;
                    cx.notify();
                }))
                .children(self.connect_box.bauds_open.then(|| {
                    let rows: Vec<gpui::AnyElement> = connect::BAUDS
                        .iter()
                        .map(|rate| {
                            let id = format!("main-baud-{rate}");
                            probe::measured(id.clone(), div())
                                .id(gpui::SharedString::from(id))
                                .px_2()
                                .py(px(2.0))
                                .text_xs()
                                .text_color(rgb(theme::TEXT))
                                .cursor_pointer()
                                .hover(|style| style.bg(rgb(theme::BORDER)))
                                .child(*rate)
                                .on_click(cx.listener(move |this, _event, _window, cx| {
                                    // `CMB_baudrate_TextChanged`: a number, its digits kept.
                                    // `// C#: MainV2.cs:4333-4350`
                                    match connect::baud_changed(rate) {
                                        Ok(baud) => {
                                            this.connect_box.baud = baud;
                                            this.persisted.set_baud(&this.connect_box.baud);
                                        }
                                        // `Strings.InvalidBaudRate` is a box in the C#; here a
                                        // status line - the owner's ruling of 2026-09-25: no
                                        // box for an error the window can show as state.
                                        Err(why) => this.file_status = Some(why.to_owned()),
                                    }
                                    this.connect_box.bauds_open = false;
                                    cx.notify();
                                }))
                                .into_any_element()
                        })
                        .collect();
                    dropdown("main-baud-list", rows)
                })),
        );
        // `MenuConnect`: CONNECT, or DISCONNECT while the link is open.
        // `// C#: MainV2.cs:2459-2482`
        strip = strip.child(ui::action(
            "main-connect",
            if connected {
                connect::DISCONNECT
            } else {
                connect::CONNECT
            },
            if connected { theme::WARN } else { theme::OK },
            true,
            cx.listener(|this, _event: &(), window, cx| {
                this.connect_clicked(window, cx);
                cx.notify();
            }),
        ));
        strip
    }

    /// `MenuConnect_Click` → `Connect`: a moving model is asked about first; then the link is
    /// closed if it is open, else opened from the boxes; and the settings are saved either way.
    /// `// C#: MainV2.cs:1841-1880`
    fn connect_clicked(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let view = self.telemetry.view();
        let connected = view.connected && !view.target.starts_with("file:");
        let groundspeed = view
            .state
            .as_deref()
            .map_or(0.0, |state| state.ground_speed.0);
        if connect::asks_before_disconnecting(connected, groundspeed) {
            self.connect_box.still_moving = true;
            self.connect_focus.focus(window, cx);
            return;
        }
        if connected {
            self.do_disconnect();
        } else {
            self.do_connect(window, cx);
        }
    }

    /// `doDisconnect`: the port closed, the recording with it, and the settings saved as
    /// `MenuConnect_Click` saves them.
    /// `// C#: MainV2.cs:1389-1447, 1844-1845`
    fn do_disconnect(&mut self) {
        self.telemetry = Telemetry::idle();
        self.mission_requested = false;
        self.params_requested = false;
        self.file_status = Some("disconnected".to_owned());
        self.save_config(settings::SaveEvent::Connect);
    }

    /// `doConnect` from the boxes: AUTO's port scan is not ported and is refused as such; a
    /// serial port opens at once; a network kind asks its transport's questions first.
    /// `// C#: MainV2.cs:1448-1526`
    fn do_connect(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.do_connect_with(false, window, cx);
    }

    /// [`MissionPlanner::do_connect`], `blind` for Ctrl+T: the transport's questions asked as its
    /// `Open` asks them, the parameters not fetched once it is open.
    fn do_connect_with(&mut self, blind: bool, window: &mut Window, cx: &mut Context<Self>) {
        self.blind_connect = blind;
        let port = self.connect_box.port.clone();
        let kind = connect::kind(&port);
        if port.is_empty() {
            self.file_status = Some("choose a port first".to_owned());
            return;
        }
        if kind == connect::Kind::Auto {
            // `CommsSerialScan` is not ported: the C#'s scan of every port for a heartbeat.
            self.file_status = Some("AUTO is not ported; choose the port".to_owned());
            return;
        }
        let questions = connect::questions(kind);
        if questions.is_empty() {
            if let Some(url) = connect::url(kind, &port, &self.connect_box.baud, &[]) {
                self.open_link(&url);
            }
            return;
        }
        let Some(first) = questions.first().cloned() else {
            return;
        };
        self.connect_box.asking = Some(connect::Asking {
            kind,
            questions,
            answers: Vec::new(),
        });
        self.connect_field
            .set(self.persisted.get(first.key).unwrap_or(first.default));
        self.connect_focus.focus(window, cx);
    }

    /// The link opened and, as `MenuConnect_Click` then does, the settings saved: the box's
    /// port and baud, a network kind's answers under its keys.
    /// `// C#: MainV2.cs:1841-1847; ExtLibs/Comms/CommsTCPSerial.cs:142-143`
    fn open_link(&mut self, url: &str) {
        self.telemetry = Telemetry::connect(url);
        self.mission_requested = false;
        // `Open(false)`, Ctrl+T's: no `getParamList` - as if already asked for.
        self.params_requested = std::mem::take(&mut self.blind_connect);
        if let Some(err) = self.telemetry.error() {
            self.file_status = Some(format!("could not open {url}: {err}"));
        } else {
            self.file_status = Some(format!("connected to {url}"));
            self.persisted.link_opened(url);
            self.remember();
        }
        self.save_config(settings::SaveEvent::Connect);
    }

    /// `lnk_kml_LinkClicked`: `Process.Start("http://127.0.0.1:56781/network.kml")`, the desktop's
    /// browser on the built-in server's Google Earth network link; "Failed to open url ..." when
    /// it cannot be - on the status line, the owner's rule. The server (`Utilities/httpserver.cs`)
    /// is its own row: until it is here the browser finds nothing at the address, as it does when
    /// the C#'s server is not running. `// C#: GCSViews/FlightPlanner.cs:4318-4328`
    fn view_kml_clicked(&mut self) {
        let result =
            scripts_tab::open_with_shell(std::path::Path::new(plan::NETWORK_KML_URL));
        if result.is_err() {
            self.file_status = Some(format!("Failed to open url {}", plan::NETWORK_KML_URL));
        }
        self.kml_link = Some(result);
    }

    /// `BUT_InjectCustomMap_Click`: while a run is on the button reads "Cancel" and stops it;
    /// otherwise the folder dialog. `// C#: GCSViews/FlightPlanner.cs:8416-8445`
    fn inject_map_clicked(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if let Some(job) = &self.inject_map {
            job.cancel();
            return;
        }
        self.plan_menus.ask_inject_folder();
        self.plan_prompt_focus.focus(window, cx);
    }

    /// The folder named: `Directory.GetFiles` of its images, the run started over them, the bar
    /// shown. An empty name is the dialog cancelled. `// C#: GCSViews/FlightPlanner.cs:8428-8447`
    fn inject_map_begin(&mut self, folder: &str) {
        let folder = folder.trim();
        if folder.is_empty() {
            return;
        }
        let files = inject_map::scan(std::path::Path::new(folder));
        let cache = TileCache::new(TileCache::default_root());
        self.inject_map = Some(inject_map::Injection::start(files, cache));
    }

    /// The run's end, on the frame: the button's text back, the bar hidden, the map type to
    /// Custom - a new store, which is the memory cache cleared and the maps reloaded - and the
    /// results box; the exception that ended it, if one did, on the status line.
    /// `// C#: GCSViews/FlightPlanner.cs:8488-8527`
    fn inject_map_tick(&mut self) {
        if !self
            .inject_map
            .as_ref()
            .is_some_and(inject_map::Injection::finished)
        {
            return;
        }
        let Some(mut job) = self.inject_map.take() else {
            return;
        };
        job.join();
        if let Some(why) = job.failure() {
            self.file_status = Some(format!("Inject Custom Map: {why}"));
        }
        self.set_tile_source(&mp_tiles::source::CUSTOM);
        self.plan_menus.say(inject_map::RESULTS_TITLE, job.results());
    }

    /// A question's OK (or Enter): the answer kept under its settings key and as `InputBox`
    /// keeps it, the next question asked, and the link opened after the last.
    fn connect_answered(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let Some(mut asking) = self.connect_box.asking.take() else {
            return;
        };
        let Some(question) = asking.current().cloned() else {
            return;
        };
        let answer = connect::answered(&mut self.persisted, &question, self.connect_field.value());
        asking.answers.push(answer);
        if let Some(next) = asking.current().cloned() {
            self.connect_field
                .set(self.persisted.get(next.key).unwrap_or(next.default));
            self.connect_box.asking = Some(asking);
            self.connect_focus.focus(window, cx);
            return;
        }
        let port = self.connect_box.port.clone();
        let baud = self.connect_box.baud.clone();
        let url = connect::url(asking.kind, &port, &baud, &asking.answers);
        // Asked for Install Firmware's Bootloader Update: its own link, not the window's.
        if self.install_firmware.take_bl_asking() {
            self.install_firmware
                .bl_open(url.as_deref(), web_time::Instant::now());
            return;
        }
        if let Some(url) = url {
            self.open_link(&url);
        }
    }

    /// The dialogs of the connection box: a network kind's question with its box, and "Your
    /// model is still moving ..." with Yes and No - each modal over the window, as the C#'s
    /// are. (`Strings.InvalidBaudRate`'s box is a status line here, the owner's ruling.)
    fn connect_dialogs(&self, window: &Window, cx: &mut Context<Self>) -> Option<gpui::AnyElement> {
        let focus = &self.connect_focus;
        let (title, text, has_box, yes_no) = if let Some(asking) = &self.connect_box.asking {
            let question = asking.current()?;
            (question.title, question.text, true, false)
        } else if self.connect_box.still_moving {
            (
                connect::DISCONNECT_TITLE,
                connect::STILL_MOVING,
                false,
                true,
            )
        } else {
            return None;
        };
        let on_key = cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
            let outcome = if this.connect_box.asking.is_some() {
                this.connect_field.key(event)
            } else {
                fly::answer_key(event)
            };
            match outcome {
                textfield::KeyOutcome::Submitted => this.connect_answer(true, window, cx),
                textfield::KeyOutcome::Cancelled => this.connect_answer(false, window, cx),
                textfield::KeyOutcome::Changed => cx.notify(),
                textfield::KeyOutcome::Ignored => {}
            }
        });
        let size = window.viewport_size();
        let mut dialog = probe::measured("main-connect-prompt", div())
            .flex()
            .flex_col()
            .gap_2()
            .w(px(340.0))
            .p_3()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::ACCENT))
            .rounded_md()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(title))
            .child(div().text_sm().text_color(rgb(theme::TEXT)).child(text));
        let mut on_key = Some(on_key);
        if has_box && let Some(on_key) = on_key.take() {
            dialog = dialog.child(textfield::text_field(
                "main-connect-field",
                &self.connect_field,
                focus,
                focus.is_focused(window),
                px(310.0),
                on_key,
            ));
        }
        let (yes, no) = if yes_no {
            ("Yes", Some("No"))
        } else if has_box {
            ("OK", Some("Cancel"))
        } else {
            ("OK", None)
        };
        dialog = dialog.child(
            div()
                .flex()
                .justify_end()
                .gap_2()
                .child(ui::action(
                    "main-connect-ok",
                    yes,
                    theme::ACCENT,
                    true,
                    cx.listener(|this, _event: &(), window, cx| {
                        this.connect_answer(true, window, cx);
                    }),
                ))
                .children(no.map(|label| {
                    ui::action(
                        "main-connect-cancel",
                        label,
                        theme::TEXT,
                        true,
                        cx.listener(|this, _event: &(), window, cx| {
                            this.connect_answer(false, window, cx);
                        }),
                    )
                })),
        );
        let body = match on_key {
            Some(on_key) => dialog
                .id("main-connect-keys")
                .track_focus(focus)
                .on_key_down(on_key)
                .into_any_element(),
            None => dialog.into_any_element(),
        };
        Some(
            gpui::deferred(
                gpui::anchored()
                    .position(gpui::point(px(0.0), px(0.0)))
                    .child(
                        div()
                            .id("main-connect-backdrop")
                            .w(size.width)
                            .h(size.height)
                            .flex()
                            .items_center()
                            .justify_center()
                            .occlude()
                            .child(body),
                    ),
            )
            .with_priority(3)
            .into_any_element(),
        )
    }

    /// A connection dialog answered: Yes disconnects the moving model, OK takes a question's
    /// answer, Cancel or No leaves things as they were.
    fn connect_answer(&mut self, yes: bool, window: &mut Window, cx: &mut Context<Self>) {
        if self.connect_box.still_moving {
            self.connect_box.still_moving = false;
            if yes {
                self.do_disconnect();
            }
            cx.notify();
            return;
        }
        if self.connect_box.asking.is_some() {
            if yes {
                self.connect_answered(window, cx);
            } else {
                // "Canceled by request": the transport's Open throws, and nothing opens - for
                // Bootloader Update, "Failed to find device on mavlink".
                self.connect_box.asking = None;
                if self.install_firmware.take_bl_asking() {
                    self.install_firmware
                        .bl_open(None, web_time::Instant::now());
                }
            }
        }
        cx.notify();
    }

    fn set_altitude_frame(&mut self, frame: plan::AltitudeFrame) {
        self.altitude_frame = frame;
        // `CMB_altmode_SelectedIndexChanged`'s `FPaltmode`, saved with the rest later.
        self.persisted.altitude_frame_changed(frame);
        self.settings.altitude_frame = Some(frame.key().to_owned());
        // A settings file that cannot be written is not a reason to refuse the change: the choice
        // applies to this session either way, and the worst case is that it is not remembered.
        if let Err(err) = self.settings.save() {
            self.file_status = Some(format!("could not remember the altitude frame: {err}"));
        }
    }

    /// Opens the log named in the log field.
    ///
    /// The same rule the mission file field follows: a name, not a path, resolved inside the plan
    /// directory, so a typed separator cannot read from anywhere else.
    fn open_log(&mut self) {
        let name = self.log_name.value().trim();
        let leaf = name
            .rsplit(['/', '\\'])
            .next()
            .filter(|part| !part.is_empty() && *part != "." && *part != "..");
        let Some(leaf) = leaf else {
            self.file_status = Some("type the name of a log to open".to_owned());
            return;
        };
        let path = Self::plan_directory().join(leaf);
        self.log_browse.open(&path);
        // `add_field_node`'s tooltips, from the connected vehicle's parameters.
        // `// C#: Log/LogBrowse.cs:685`
        let parameters = self.telemetry.view().parameters;
        self.log_browse
            .add_field_tips(&parameters, crate::metadata::lookup);
        // `LoadLog2` sets six of the strip's boxes from config.xml once the log is read.
        // `// C#: Log/LogBrowse.cs:444-449`
        if self.log_browse.is_open() {
            let persisted = &self.persisted;
            self.log_browse
                .apply_remembered(|key| persisted.get(key).map(str::to_owned));
        }
    }

    /// Commands a guided move to a position, holding the current height.
    ///
    /// Keeping the aircraft's own altitude is the only safe default: a fixed one would descend a
    /// vehicle that is above it, and a click meant to redirect a flight is not a click meant to
    /// change height. On the ground it does nothing beyond what the vehicle's own checks allow -
    /// a disarmed vehicle refuses, and says so in the message pane.
    fn fly_here(&mut self, position: mp_units::LatLon) {
        // Once Fly To Here Alt has set a height, Fly To Here flies at it, in its frame, as
        // `goHereToolStripMenuItem_Click` does with `GuidedMode.z`.
        // `// C#: GCSViews/FlightData.cs:3090-3113`
        if self.fly_actions.guided.z != 0.0 {
            self.fly_to_here_guided(position);
            return;
        }
        let view = self.telemetry.view();
        let Some(state) = view.state.as_ref() else {
            self.file_status = Some("no vehicle to send anywhere".to_owned());
            return;
        };
        #[allow(clippy::cast_possible_truncation)] // altitudes are metres; f32 is ample
        let altitude = state.altitude_relative.0 as f32;
        let altitude = if altitude > 1.0 {
            altitude
        } else {
            fly::TAKEOFF_ALTITUDE
        };
        self.telemetry.goto(position, altitude);
        self.file_status = Some(format!(
            "fly to {:.6}, {:.6} at {altitude:.0} m",
            position.latitude(),
            position.longitude()
        ));
    }

    /// Disables the vehicle's arming checks and arms it, re-sending until it takes.
    ///
    /// `ARMING_CHECK` stays off afterwards. That is what "always arm the vehicle, no matter" asks
    /// for, and restoring it quietly would mean the next arm behaved differently for reasons the
    /// operator could not see. It is said plainly on the status line instead, and the actions
    /// panel offers putting the checks back.
    fn begin_force_arm(&mut self) {
        /// Long enough for a parameter write to be applied and echoed, short enough that a vehicle
        /// which is never going to arm stops being asked.
        const GIVE_UP_AFTER: std::time::Duration = std::time::Duration::from_secs(6);

        let now = web_time::Instant::now();
        self.force_arm_request = self.telemetry.force_arm().map(|id| (id, now));
        self.last_force_arm = Some(now);
        self.disabled_arming_checks = true;
        self.forcing_arm_until = Some(web_time::Instant::now() + GIVE_UP_AFTER);
        self.file_status =
            Some("arming checks disabled (ARMING_SKIPCHK=-1); forcing arm".to_owned());
    }

    /// Changes one parameter by a step, clamped to its documented range.
    ///
    /// Clamped rather than refused: the operator asked to move it, and stopping at the limit is
    /// what they meant. ArduPilot accepts an out-of-range write and then behaves oddly, so the
    /// editor is the last place to catch it.
    ///
    /// `setParam`, sent again until the vehicle echoes it; the status line then says what the
    /// vehicle holds, or the C#'s "Set NAME Failed".
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:323, 359-363`
    fn nudge_parameter(&mut self, name: &str, delta: f64) {
        let view = self.telemetry.view();
        let Some((_, current)) = view.parameters.iter().find(|(held, _)| held == name) else {
            return;
        };
        let mut next = current + delta;
        if let Some(meta) = metadata::lookup(name)
            && let Some((low, high)) = meta.range
        {
            next = next.clamp(low, high);
        }
        // ---- ConfigRawParams remainder ----
        // The step is an edit of the Value cell like any other, and meets its ReadOnly box.
        // C#: GCSViews/ConfigurationView/ConfigRawParams.cs:454-533
        self.param_grid_edit(name, &next.to_string());
        // ---- end ConfigRawParams remainder ----
    }

    /// Pushes the plan to the map after an edit.
    ///
    /// The render pass does this too, but only on the next frame; doing it at the edit means the
    /// map never shows a waypoint the operator has just deleted.
    fn sync_map_mission(&self) {
        match self.survey_preview() {
            Some((preview, _)) => self.map.borrow_mut().set_mission(&preview),
            None => self.map.borrow_mut().set_mission(self.plan.items()),
        }
    }

    /// Pushes the survey area to the map after an edit: the Survey (Grid) dialog's boundary
    /// while it is open.
    fn sync_map_polygon(&self) {
        match self.survey_preview() {
            Some((_, boundary)) => self.map.borrow_mut().set_polygon(&boundary),
            None => self.map.borrow_mut().set_polygon(self.plan.shown_polygon()),
        }
    }

    /// What the Survey (Grid) dialog shows on the map, while it is open on the planning screen.
    fn survey_preview(&self) -> Option<(Vec<mp_mission::MissionItem>, Vec<mp_units::LatLon>)> {
        (self.screen == Screen::Plan)
            .then(|| self.survey.preview())
            .flatten()
    }

    /// Pushes the geofence to the map after an edit: `geofenceoverlay`'s polygon, its return
    /// marker (`GeoFence Return`) and the exclusions.
    fn sync_map_fence(&self) {
        let mut map = self.map.borrow_mut();
        map.set_fence(self.plan.fence());
        map.set_fence_return(self.plan.fence_return());
        map.set_fence_exclusions(self.plan.fence_exclusions());
    }

    /// Pushes the rally points to the map after an edit.
    fn sync_map_rally(&self) {
        let positions: Vec<mp_units::LatLon> = self
            .plan
            .rally()
            .iter()
            .map(|point| point.position)
            .collect();
        self.map.borrow_mut().set_rally(&positions);
    }

    /// Remembers the current choices, so the next launch starts where this one left off.
    ///
    /// Called when something worth remembering changes rather than on a timer, and a failure is
    /// reported on the status line rather than raised: losing a preference is not worth
    /// interrupting anyone over.
    fn remember(&mut self) {
        self.settings.screen = Some(self.screen.label().to_owned());
        // The URL opened, not the link's description: the browser's "tcp:127.0.0.1:5760 (through
        // the page)", remembered, failed to open at the next visit (the owner's report, 2026-10-05).
        if let Some(link) = link_to_remember(self.telemetry.url().to_owned(), storm::enabled()) {
            self.settings.link = Some(link);
        }
        if let Err(err) = self.settings.save() {
            self.file_status = Some(format!(
                "could not save settings to {}: {err}",
                settings::Settings::path().display()
            ));
        }
    }

    /// One chip per vehicle on the link, when there is more than one.
    ///
    /// Hidden with a single vehicle, which is the usual case: a control that only ever has one
    /// option is clutter that teaches an operator to ignore that part of the screen.
    fn vehicle_picker(&self, view: &TelemetryView, cx: &mut Context<Self>) -> impl IntoElement {
        let vehicles = self.telemetry.vehicles();
        let current = view.vehicle;
        let mut strip = div().flex().items_center().gap_1().pb_2();
        if vehicles.len() < 2 {
            return strip;
        }

        strip = strip.child(div().text_xs().text_color(rgb(theme::DIM)).child("vehicle"));
        for id in vehicles {
            let selected = current == Some(id);
            let label = format!("{}:{}", id.sysid, id.compid);
            strip = strip.child(
                probe::measured(format!("vehicle-{}-{}", id.sysid, id.compid), div())
                    .id(gpui::SharedString::from(format!(
                        "veh-{}-{}",
                        id.sysid, id.compid
                    )))
                    .px_2()
                    .py(px(1.0))
                    .rounded_sm()
                    .border_1()
                    .border_color(rgb(if selected {
                        theme::ACCENT
                    } else {
                        theme::BORDER
                    }))
                    .bg(rgb(if selected {
                        theme::ACTION
                    } else {
                        theme::PANEL
                    }))
                    .text_xs()
                    .text_color(rgb(if selected { theme::ACCENT } else { theme::TEXT }))
                    .cursor_pointer()
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(label)
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.telemetry.select(id);
                        // A different vehicle has a different mission, a different fence and a
                        // different parameter set. Keeping the old ones on screen while the
                        // heading and position switched would be the worst kind of wrong.
                        this.plan.clear();
                        this.selected_param_group = None;
                        this.selected_param = None;
                        this.map.borrow_mut().follow_vehicle();
                        this.file_status =
                            Some(format!("showing vehicle {}:{}", id.sysid, id.compid));
                        cx.notify();
                    })),
            );
        }
        strip
    }

    /// The tab strip.
    fn tabs(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let current = self.screen;
        let mut strip = div().flex().gap_1();
        for screen in Screen::ALL {
            let selected = screen == current;
            strip = strip.child(
                probe::measured(screen.id(), div())
                    .id(screen.id())
                    .px_3()
                    .py_1()
                    .rounded_t_md()
                    .text_sm()
                    .cursor_pointer()
                    .bg(rgb(if selected { theme::PANEL } else { theme::BG }))
                    .text_color(rgb(if selected { theme::ACCENT } else { theme::DIM }))
                    .border_b_2()
                    .border_color(rgb(if selected { theme::ACCENT } else { theme::BG }))
                    .hover(|style| style.text_color(rgb(theme::TEXT)))
                    .child(screen.label())
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.choose_screen(screen);
                        cx.notify();
                    })),
            );
        }
        strip
    }

    /// A screen chosen, from its tab or from a menu: `MainSwitcher.ShowScreen`.
    pub(crate) fn choose_screen(&mut self, screen: Screen) {
        let this = self;
        {
            // `MainSwitcher.ShowScreen` deactivates the screen showing first, the
            // same one again included: the planning screen's `Deactivate` is its
            // `config(true)`.
            // `// C#: ExtLibs/Controls/MainSwitcher.cs:112-125; GCSViews/FlightPlanner.cs:340-344`
            if this.screen == Screen::Plan {
                this.persisted
                    .planner_deactivated(&this.plan, this.altitude_frame);
            }
            // `FlightData.Deactivate` keeps the map's place for the next start.
            // `// C#: GCSViews/FlightData.cs:662-664`
            if this.screen == Screen::Fly {
                this.persisted
                    .flight_data_deactivated(this.map.borrow().position_and_zoom());
            }
            // ---- SITL ----
            if this.screen == Screen::Sitl {
                this.sitl.deactivate();
            }
            // ---- end SITL ----
            // ---- Display view (row 71) ----
            // SETUP and CONFIG are made anew when shown again.
            if this.screen == screen {
                this.show_screen_again(screen);
            }
            // ---- end Display view ----
            // The PLUGINS tab's form goes with it; chosen, it is filled anew, as Ctrl+P fills it.
            if this.screen == Screen::Plugins && screen != Screen::Plugins {
                this.plugins.manager.close();
            }
            this.screen = screen;
            if screen == Screen::Plugins {
                this.plugins.open_manager(&this.persisted);
            }
            if screen == Screen::Plan {
                plan::activate(this);
            }
            // ---- SITL ----
            if screen == Screen::Sitl {
                this.sitl_activate();
            }
            // ---- end SITL ----
            if screen == Screen::Help {
                this.help.activate(&this.persisted);
            }
            // Remembered here rather than at exit: gpui gives no reliable hook for a
            // window closing, and a ground station is as likely to be killed as
            // closed.
            this.remember();
            // FLIGHT DATA and FLIGHT PLAN save Mission Planner's config.xml once the
            // screen is shown; SETUP and CONFIG do not.
            // `// C#: MainV2.cs:1309-1323`
            match screen {
                Screen::Fly => this.save_config(settings::SaveEvent::FlightData),
                Screen::Plan => this.save_config(settings::SaveEvent::FlightPlanner),
                Screen::Setup
                | Screen::Config
                | Screen::Sitl
                | Screen::Help
                | Screen::Params
                | Screen::Logs
                | Screen::Experimental
                | Screen::Plugins => {}
            }
        }
    }
}

impl MissionPlanner {
    /// `GMapMarkerBase`'s statics as `MainV2` reads them from the settings - `GetInt32
    /// ("GMapMarkerBase_length", 500)` and `GetBoolean("GMapMarkerBase_Display*", true)` - which
    /// the Planner page's check boxes write.
    /// `// C#: MainV2.cs:3855-3860; GCSViews/ConfigurationView/ConfigPlanner.cs:1080-1108`
    fn marker_settings(&self) -> mapview::MarkerSettings {
        let flag = |key: &str| {
            self.persisted
                .get(key)
                .is_none_or(|value| !value.trim().eq_ignore_ascii_case("false"))
        };
        let length = self
            .persisted
            .get("GMapMarkerBase_length")
            .and_then(|value| value.trim().parse::<f32>().ok())
            .unwrap_or(500.0);
        mapview::MarkerSettings {
            length,
            cog: flag("GMapMarkerBase_DisplayCOG"),
            heading: flag("GMapMarkerBase_DisplayHeading"),
            nav_bearing: flag("GMapMarkerBase_DisplayNavBearing"),
            radius: flag("GMapMarkerBase_DisplayRadius"),
            target: flag("GMapMarkerBase_DisplayTarget"),
        }
    }

    /// The primary flight display's inputs for this frame.
    ///
    /// The clocks - how long since arming, since the mode changed, since a message was raised -
    /// live in `hud_timing` and are advanced here, once a frame, because the vehicle state says
    /// only what is true now and the C# HUD shows things for a while after they change. The
    /// display's `displayAOASSA` lives there too: it turns on at the first angle of attack or
    /// sideslip that is not 0, and stays on.
    fn hud_inputs(&mut self, view: &TelemetryView) -> hud::HudInputs {
        let Some(state) = view.state.as_deref() else {
            return hud::HudInputs::default();
        };
        // One path with the golden frames, which draw a recorded flight through it.
        hud::live_inputs(
            state,
            &mut self.hud_timing,
            web_time::Instant::now(),
            chrono::Local::now().format("%H:%M:%S").to_string(),
            &view.parameters,
        )
    }

    /// The left column on the flight screen: `SubMainLeft`, the HUD above `tabControlactions`.
    ///
    /// The HUD is pinned, then the page strip, then the one page showing. Only one page shows at
    /// a time, which is what keeps the column inside the window - the panels used to stack under
    /// the HUD and ran hundreds of pixels past the bottom of it. The page still scrolls, so a
    /// window smaller than any page is laid out rather than cut off; the HUD and the strip do
    /// not, because scrolling the whole column once dragged the HUD off the top of the screen as
    /// the mode list arrived under it.
    /// `// C#: GCSViews/FlightData.Designer.cs:321-329`
    fn fly_sidebar(
        &self,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> impl IntoElement {
        // A plugin's page has its form; the others their panels.
        let page = match self.fly_pages.selected() {
            fly::Page::Plugin(slot) => fly::plugin_page(slot)
                .map(|page| plugins_ui::page_form(self, page.plugin, window, cx))
                .into_iter()
                .collect(),
            page => fly::page_content(
                page,
                &fly::PageInputs {
                    view,
                    data: &self.fly_data,
                    actions: &self.fly_actions,
                    focus: &self.fly_focus,
                    checks_disabled: self.disabled_arming_checks,
                },
                window,
                cx,
            ),
        };
        probe::measured("fly-column", div())
            .flex()
            .flex_col()
            .flex_shrink_0()
            .min_h(px(0.0))
            .gap_2()
            .w(px(400.0))
            // `SwapHud1AndMap`: `tableMap` - the tuning graph over the map - where `hud1` was.
            // `// C#: GCSViews/FlightData.cs:5139-5159, GCSViews/FlightData.Designer.cs:2472-2473`
            .child(if self.fly_data.swapped {
                div()
                    .flex()
                    .flex_col()
                    .flex_shrink_0()
                    .gap_2()
                    .h(px(260.0))
                    .child(
                        div()
                            .flex_shrink_0()
                            .child(tuning::panel_for(&self.tuning, cx)),
                    )
                    .child(self.map_pane(window, cx))
                    .into_any_element()
            } else {
                fly::hud_panel(&self.hud, &self.fly_data, cx).into_any_element()
            })
            .child(fly::page_strip(&self.fly_pages, cx))
            .child(
                // The scrolling page and its indicator share a positioned parent, so the
                // indicator can sit over the page's right edge without taking width from it.
                //
                // As high as the page needs, the HUD above giving way to it down to its least:
                // the Actions page's mode buttons, with a vehicle's modes, ran 160 past the 499
                // a fixed HUD left (the layout guard on the owner's Mac, 2026-10-05). The page
                // shrinks, and scrolls, only once the HUD has given all it can.
                div()
                    .relative()
                    .flex()
                    .flex_col()
                    .flex_grow(1.0)
                    .flex_shrink(0.001)
                    .min_h(px(0.0))
                    .child(
                        probe::measured("fly-sidebar", div())
                            .id("fly-sidebar")
                            .flex()
                            .flex_col()
                            .flex_grow(1.0)
                            .min_h(px(0.0))
                            .gap_2()
                            .pr_2()
                            .overflow_y_scroll()
                            .track_scroll(&self.fly_scroll)
                            // `SubMainLeft.Panel2.ContextMenuStrip`: the strip's menu, Customize
                            // and MultiLine, anywhere on the page that has no menu of its own.
                            // `// C#: GCSViews/FlightData.Designer.cs:327`
                            .on_mouse_up(
                                MouseButton::Right,
                                cx.listener(|this, event: &gpui::MouseUpEvent, _window, cx| {
                                    if this.fly_data.menu.is_none() {
                                        this.fly_data.menu = Some((
                                            fly::MenuKind::Tabs,
                                            (
                                                f32::from(event.position.x),
                                                f32::from(event.position.y),
                                            ),
                                        ));
                                        cx.notify();
                                    }
                                }),
                            )
                            .children(page),
                    )
                    .children(ui::scroll_indicator(&self.fly_scroll)),
            )
    }

    /// The planning screen's panels, or one of its two groups: `panelAction`'s (Read, Write,
    /// Home Location, the drawing panel, the checks) or `panelWaypoints`' (the strip and the
    /// grid, the row editor).
    fn plan_panels(
        &self,
        group: PlanGroup,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> Vec<gpui::AnyElement> {
        // Copied out of the plan before building the elements: the listeners the panels install
        // take `&mut self`, so holding a borrow of `self.plan` across them would not compile.
        let items = self.plan.items().to_vec();
        let origin = self.plan.origin().clone();
        let selected = self.plan.selected();
        let fence_error = self.plan.fence_error().map(ToOwned::to_owned);
        let rally_error = self.plan.rally_error().map(ToOwned::to_owned);
        let draw = plan::DrawState {
            mode: self.plan.draw_mode(),
            area_vertices: self.plan.polygon().len(),
            fence_vertices: self.plan.fence().len(),
            fence_error: fence_error.as_deref(),
            rally_points: self.plan.rally().len(),
            rally_error: rally_error.as_deref(),
            fence_busy: self.plan.fence_busy(),
        };
        let actions = group == PlanGroup::Actions;
        let waypoints = group == PlanGroup::Grid;
        let mut out = Vec::new();
        if actions {
            out.push(
                plan::actions_panel(
                    &items,
                    &origin,
                    view,
                    &plan::NameField {
                        field: &self.plan_name,
                        focus: &self.plan_name_focus,
                        focused: self.plan_name_focus.is_focused(window),
                    },
                    self.tile_source_id(),
                    &plan::ActionExtras {
                        inject: self.inject_map.as_ref().map(inject_map::Injection::progress),
                        grid: self.plan.grid(),
                        tiles_loading: {
                            let map = self.map.borrow();
                            map.painted().then(|| map.tile_counts().2 > 0)
                        },
                        coords: self.plan.coords(),
                        mission_ftp: self.plan.mission_ftp(),
                    },
                    cx,
                )
                .into_any_element(),
            );
            out.push(
                plan::home_panel(
                    &self.plan,
                    &plan::HomeFocus {
                        handles: &self.plan_home_focus,
                        focused: self
                            .plan_home_focus
                            .each_ref()
                            .map(|handle| handle.is_focused(window)),
                    },
                    cx,
                )
                .into_any_element(),
            );
            out.push(plan::draw_panel(&draw, view, cx).into_any_element());
        }
        if waypoints {
            let strip = plan::waypoint_strip(
                &self.plan,
                &plan::StripState {
                    focus: &self.plan_panel_focus,
                    focused: self
                        .plan_panel_focus
                        .handles
                        .each_ref()
                        .map(|handle| handle.is_focused(window)),
                    frame: self.altitude_frame,
                    spline_visible: plan::firmware_is_copter(view),
                },
                cx,
            );
            out.push(
                plan::items_panel(&items, selected, strip, self.plan.commands_minimised(), cx)
                    .into_any_element(),
            );
            out.push(
                div()
                    .id("plan-editor")
                    .flex_shrink_0()
                    .w(px(340.0))
                    .child(plan::editor_panel(&items, selected, cx))
                    .into_any_element(),
            );
        }
        out
    }

    /// The default docking's `panelAction` (`DockStyle.Right`): the action panels in a column at
    /// the map's right, scrolling when they are taller than the window.
    /// `// C#: GCSViews/FlightPlanner.resx (panelAction: Dock Right, 975,0, 131x488)`
    fn plan_sidebar(
        &self,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> impl IntoElement {
        div()
            .relative()
            .flex()
            .flex_col()
            .flex_shrink_0()
            .min_h(px(0.0))
            .w(px(400.0))
            .child(
                div()
                    .id("plan-sidebar")
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(0.0))
                    .gap_2()
                    .pr_2()
                    .overflow_y_scroll()
                    .track_scroll(&self.plan_scroll)
                    .children(self.plan_panels(PlanGroup::Actions, view, window, cx)),
            )
            .children(ui::scroll_indicator(&self.plan_scroll))
    }

    /// The planning screen by its docking: as Mission Planner lays it out (`FP_docking` "Right"),
    /// the map over `panelWaypoints` - the mission grid in the lower third, under the map (the
    /// owner, 2026-10-04: it had been at the foot of one long column beside the map, below the
    /// window) - and `panelAction`'s column at the right; or - after Switch Docking -
    /// `panelWaypoints` at the right, half the window wide,
    /// and `panelAction`'s panels in a row along the bottom (120 high in the C#; these panels are
    /// taller, so the row is as high as the tallest of them and scrolls sideways). The menus and dialogs go over either.
    /// `// C#: GCSViews/FlightPlanner.cs:6762-6778`
    fn plan_screen(
        &self,
        view: &TelemetryView,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) -> gpui::AnyElement {
        let overlays: Vec<gpui::AnyElement> = plan::overlays(
            &self.plan_menus,
            self.plan.draw_mode() == plan::DrawMode::Fence,
            &self.plan_prompt_focus,
            window,
            cx,
        )
        .into_iter()
        .chain(prefetch_ui::overlays(&self.plan_menus, window, cx))
        .collect();
        match self.plan_docking {
            Docking::Right => div()
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .gap_2()
                .p_2()
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .flex_1()
                        .min_w(px(0.0))
                        .min_h(px(0.0))
                        .gap_2()
                        // `panelMap`, `DockStyle.Fill`: what the grid leaves.
                        .child(
                            div()
                                .flex()
                                .flex_1()
                                .min_h(px(0.0))
                                .child(self.map_pane(window, cx)),
                        )
                        // `panelWaypoints`, `DockStyle.Bottom`: at least the lower third, and
                        // as tall as the selected item's editor beside the grid needs, so
                        // nothing in it is cut off (the owner, 2026-10-04); the grid fills it
                        // and scrolls its rows. Folded (`but_mincommands`), it is as high as the
                        // button and the map has the rest.
                        .child(
                            div()
                                .id("plan-waypoints")
                                .flex()
                                .flex_shrink_0()
                                .when(!self.plan.commands_minimised(), |grid| {
                                    grid.min_h(gpui::relative(1.0 / 3.0))
                                })
                                .gap_2()
                                .children(self.plan_panels(
                                    PlanGroup::Grid,
                                    view,
                                    window,
                                    cx,
                                )),
                        ),
                )
                .child(self.plan_sidebar(view, window, cx))
                .children(overlays)
                .into_any_element(),
            Docking::Bottom => {
                // `panelWaypoints.Width = Width / 2`.
                let half = window.viewport_size().width * 0.5;
                div()
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(0.0))
                    .gap_2()
                    .p_2()
                    .child(
                        div()
                            .flex()
                            .flex_1()
                            .min_h(px(0.0))
                            .gap_2()
                            .child(self.map_pane(window, cx))
                            // The grid and the editor side by side, as under the map in the
                            // default docking: stacked in one scrolling column, the editor's lower
                            // controls scrolled out of a 357-high half (the layout guard on the
                            // owner's Mac, 2026-10-05).
                            .child(
                                div()
                                    .id("plan-waypoints")
                                    .flex()
                                    .flex_shrink_0()
                                    .w(half)
                                    .min_h(px(0.0))
                                    .gap_2()
                                    .children(self.plan_panels(
                                        PlanGroup::Grid,
                                        view,
                                        window,
                                        cx,
                                    )),
                            ),
                    )
                                        .child(
                        // As high as its tallest panel, so a panel is never cut: the strip was a
                        // fixed 240 px and the Mission box is 414, which put Read WPs and Write
                        // WPs below the window's bottom edge (the owner's report, 2026-10-03);
                        // then capped at half the window, which cut the Mission box by 11 in a
                        // 920-high one (the layout guard on the owner's Mac, 2026-10-04). The
                        // map has what is left.
                        div()
                            .id("plan-action")
                            .flex()
                            .flex_shrink_0()
                            .gap_2()
                            .overflow_x_scroll()
                            .children(
                                self.plan_panels(PlanGroup::Actions, view, window, cx)
                                    .into_iter()
                                    .map(|panel| div().flex_shrink_0().w(px(400.0)).child(panel)),
                            ),
                    )
                    .children(overlays)
                    .into_any_element()
            }
        }
    }

    /// The parameter screen's panels: the Params tab, and CONFIG's Full Parameter List page.
    fn params_body(
        &self,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> gpui::AnyElement {
        let parameters = params::collect(view);
        let group = self.selected_param_group.clone();
        let selected = self.selected_param.clone();
        // ---- ConfigRawParams remainder ----
        let grid = raw_params_grid::GridView {
            grid: &self.param_grid,
            changes: self.raw_params.changes(),
            grid_focus: &self.param_grid_focus,
            edit_focus: &self.param_edit_focus,
            edit_focused: self.param_edit_focus.is_focused(window),
        };
        // ---- end ConfigRawParams remainder ----
        // Not scrolling: the grid's panel takes the height the others leave and scrolls its rows,
        // where the whole column had scrolled the editor and file panels below the window (the
        // layout guard on the owner's Mac, 2026-10-05).
        let main = div()
            .id("params-body")
            .flex()
            .flex_col()
            .flex_1()
            .min_w(px(0.0))
            .min_h(px(0.0))
            .gap_2()
            .p_2()
            .child(params::browser_panel(
                view,
                &parameters,
                &self.param_search,
                &self.param_search_focus,
                self.param_search_focus.is_focused(window),
                self.param_none_default,
                cx,
            ))
            // ---- row 82 ----
            .child(raw_params::controls(
                &self.raw_params,
                view.connected,
                raw_params::get_boolean(self.persisted.get(raw_params::SLOW_MACHINE)),
                cx,
            ))
            .children(raw_params::overlays(&self.raw_params, window, cx))
            // ---- ConfigRawParams remainder ----
            .children(raw_params_grid::overlays(&self.param_grid, window, cx))
            // ---- end ConfigRawParams remainder ----
            .child(params::list_panel(
                &parameters,
                group.as_deref(),
                self.param_search.value(),
                selected.as_deref(),
                &params::Filters {
                    none_default: self.param_none_default,
                    modified: self.raw_params.modified(),
                    changes: self.raw_params.changes(),
                    collapsed: self.raw_params.collapsed(),
                },
                &grid,
                cx,
            ))
            // ---- end row 82 ----
            .child(params::editor_panel(&parameters, selected.as_deref(), cx))
            .child(params::file_panel(
                view,
                &self.param_file_name,
                &self.param_file_focus,
                self.param_file_focus.is_focused(window),
                &self.param_differences,
                cx,
            ))
            .into_any_element();
        // ---- ConfigRawParams remainder ----
        // `splitContainer1`: the tree at the splitter's distance, `but_collapse` at the grid's
        // left, and the rest; the tree gone while it is collapsed.
        // C#: GCSViews/ConfigurationView/ConfigRawParams.resx (splitContainer1, but_collapse)
        let collapsed = self.raw_params.collapsed();
        let tree = (!collapsed).then(|| params::tree_panel(&parameters, group.as_deref(), cx));
        raw_params_grid::split(
            tree,
            raw_params::collapse_button(collapsed, cx),
            main,
            self.param_grid.layout(),
            cx,
        )
        // ---- end ConfigRawParams remainder ----
    }
    /// The map, with the handlers that make it a map rather than a picture.
    fn map_pane(&self, window: &Window, cx: &mut Context<Self>) -> impl IntoElement {
        let following = self.map.borrow().is_following();
        let attribution = self.map.borrow().attribution();
        let planning = self.screen == Screen::Plan;
        // The Zoom box and bar in their strip at the planning map's right.
        let zoom_column = planning
            .then(|| plan::zoom_column(self.map.borrow().zoom_level(), &self.plan_menus, cx));

        let column = div()
            .flex()
            .flex_col()
            .flex_1()
            .min_w(px(0.0))
            .overflow_hidden()
            .gap_2()
            .child(
                probe::measured("map", div())
                    .relative()
                    .flex()
                    .flex_1()
                    .bg(rgb(theme::PANEL))
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .rounded_md()
                    .id("map")
                    // Dragging pans, the wheel zooms about the cursor. The handlers convert window
                    // coordinates to viewport-relative ones using the bounds the painter recorded,
                    // so the map does not need to know where it sits in the layout.
                    // Pressing on a waypoint while planning grabs it; pressing anywhere else
                    // pans. Deciding at press time rather than on movement is what makes the two
                    // gestures feel like one control: the operator is never told which mode they
                    // are in, because the thing under the cursor already says.
                    .on_mouse_down(
                        MouseButton::Left,
                        cx.listener(move |this, event: &gpui::MouseDownEvent, window, cx| {
                            let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                            // The map takes the focus in the C#, which is a panel box's Leave.
                            if planning {
                                plan::leave_panel_boxes(this, window, cx);
                            }
                            if this.plan_menus.swallows_press((x, y)) {
                                return;
                            }
                            // `MouseDownStart`, which the flight map's menu entries act at.
                            // `// C#: GCSViews/FlightData.cs:2956-2959`
                            if !planning {
                                let at = this.map.borrow().position_at(x, y);
                                this.fly_data.mouse_down_start = at.map(|at| (at, (x, y)));
                            }
                            let grabbed = planning
                                .then(|| this.map.borrow().waypoint_at(x, y))
                                .flatten();
                            this.map_press = Some((x, y));
                            match grabbed {
                                Some(seq) => {
                                    this.dragging_waypoint = Some(seq);
                                    this.plan.select(Some(seq));
                                    plan::waypoint_grabbed(this, seq);
                                    // The same reason as placing one: a refit mid-drag would move
                                    // the waypoint away from the cursor holding it.
                                    this.map.borrow_mut().freeze_view();
                                    cx.notify();
                                }
                                None => this.map.borrow_mut().begin_drag(x, y),
                            }
                        }),
                    )
                    .on_mouse_move(cx.listener(
                        move |this, event: &gpui::MouseMoveEvent, window, cx| {
                            let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                            // With no button down the pointer is only passing over the markers:
                            // GMap's hover, which leaves them be while a button is held
                            // (`GMapControl.OnMouseMove`, `if (Core.mouseDown.IsEmpty)`).
                            // `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/
                            // GMapControl.cs:2134`
                            if event.pressed_button.is_none() {
                                // `MainMap_MouseMove` → `SetMouseDisplay`: the planner's
                                // pointer read-out follows the mouse.
                                // `// C#: GCSViews/FlightPlanner.cs:2778-2797`
                                if planning && let Some(at) = this.map.borrow().position_at(x, y) {
                                    this.plan.set_mouse_display(at);
                                    cx.notify();
                                }
                                if plan::map_hover(this, planning, Some((x, y))) {
                                    window.refresh();
                                    cx.notify();
                                }
                                return;
                            }
                            if event.pressed_button != Some(MouseButton::Left) {
                                return;
                            }
                            if let Some(seq) = this.dragging_waypoint {
                                let position = this.map.borrow().position_at(x, y);
                                if let Some(position) = position {
                                    this.plan.move_to(seq, position);
                                    this.sync_map_mission();
                                    cx.notify();
                                }
                            } else {
                                this.map.borrow_mut().drag_to(x, y);
                            }
                            // Repaint immediately: a map that only updates on the next telemetry
                            // tick feels broken to drag.
                            window.refresh();
                        },
                    ))
                    .on_mouse_up(
                        MouseButton::Left,
                        cx.listener(move |this, event: &gpui::MouseUpEvent, window, cx| {
                            let grabbed = this.dragging_waypoint.take();
                            let press = this.map_press.take();
                            this.map.borrow_mut().end_drag();
                            if let Some(seq) = grabbed {
                                plan::waypoint_dropped(this, seq);
                            }

                            // A press that did not move, on empty map, while planning, adds a
                            // waypoint where it landed. `MainMap_MouseUp` does the same:
                            //
                            //   if (!isMouseDraging) {
                            //       if (CurentRectMarker != null) { /* cant add WP in existing
                            //       rect */ } else { AddWPToMap(...); }
                            //   }
                            //
                            // so a click that grabbed an existing waypoint adds nothing, and a
                            // drag - of the map or of a waypoint - adds nothing either.
                            // `// C#: GCSViews/FlightPlanner.cs:7736-7745`
                            let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                            if plan::map_release(planning, grabbed, press, (x, y))
                                != plan::MapRelease::AddWaypoint
                            {
                                return;
                            }
                            let Some(position) = this.map.borrow().position_at(x, y) else {
                                return;
                            };
                            // What the click adds follows what is being drawn: AddWPToMap.
                            // `// C#: GCSViews/FlightPlanner.cs:558-600`
                            plan::map_click(this, position, window, cx);
                            this.sync_map_mission();
                            this.sync_map_polygon();
                            this.sync_map_fence();
                            this.sync_map_rally();
                            cx.notify();
                        }),
                    )
                    .on_scroll_wheel({
                        let map = self.map.clone();
                        move |event, window, _cx| {
                            let delta = event.delta.pixel_delta(px(20.0));
                            let steps = f32::from(delta.y) / 20.0;
                            map.borrow_mut().zoom(
                                f32::from(event.position.x),
                                f32::from(event.position.y),
                                steps,
                            );
                            window.refresh();
                        }
                    })
                    // Right-click opens the map's menu: the planning map's
                    // `MainMap.ContextMenuStrip`, or the flight map's `contextMenuStripMap`,
                    // whose entries act where it was opened (`MouseDownStart`). Not left-click:
                    // left is pan, and a gesture that both moves the map and commits something
                    // would fire on every failed drag.
                    // `// C#: GCSViews/FlightPlanner.Designer.cs:875; GCSViews/FlightData.Designer.cs:2518-2531, 2975`
                    .on_mouse_up(
                        MouseButton::Right,
                        cx.listener(move |this, event: &gpui::MouseUpEvent, window, cx| {
                            let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                            // The map's, and nobody else's: WinForms gives the button to the
                            // topmost control, so the gimbal video under the mini map does not
                            // open its own menu as well (`VideoBox.ContextMenuStrip`), whose
                            // backdrop would then take the next click.
                            cx.stop_propagation();
                            if planning {
                                plan::leave_panel_boxes(this, window, cx);
                                plan::open_map_menu(this, x, y);
                            } else {
                                let position = this.map.borrow().position_at(x, y);
                                this.fly_data.mouse_down_start = position.map(|at| (at, (x, y)));
                                this.fly_data.current_poi = this.poi_under_press((x, y));
                                this.fly_data.menu = Some((fly::MenuKind::Map, (x, y)));
                                this.fly_data.menu_sub = None;
                            }
                            cx.notify();
                        }),
                    )
                    // The pointer leaving the map leaves every marker.
                    .on_hover(cx.listener(move |this, hovered: &bool, window, cx| {
                        if !*hovered && plan::map_hover(this, planning, None) {
                            window.refresh();
                            cx.notify();
                        }
                    }))
                    .child(mapview::map_element(self.map.clone()))
                    // The planning map's zoom icon, `zoomicon`, and its menu.
                    .children(planning.then(|| plan::zoom_icon(cx)))
                    .children(planning.then(|| plan::poly_icon(cx)))
                    // The points of interest, over the flight map: `poioverlay`.
                    // `// C#: GCSViews/FlightData.cs:52, 410, 4473-4476`
                    // The planner has its own `poioverlay` on `MainMap` (`FlightPlanner.cs:85,
                    // 215`), so the points are drawn on both screens.
                    .child(poi::layer(&self.fly_data.pois, self.map.clone()))
                    // Attribution. Required by both providers' licences, so it is drawn over the
                    // map rather than in a settings screen nobody opens - if the imagery is on
                    // screen, so is the credit for it.
                    .children(attribution.map(|text| {
                        div()
                            .absolute()
                            .bottom_1()
                            .right_2()
                            .px_1()
                            .rounded_sm()
                            .bg(rgb(theme::PANEL))
                            .text_xs()
                            .text_color(rgb(theme::DIM))
                            .child(text)
                    }))
                    // Auto Pan (`CHK_autopan`) is the flight screen's: Mission Planner's planning
                    // map has none, and a button there would say it follows while nothing pans.
                    .when(!planning, |pane| {
                        pane.child(
                        div()
                            .absolute()
                            .top_2()
                            .right_2()
                            .flex()
                            .gap_2()
                            .child(action(
                                "map-follow",
                                if following {
                                    "following"
                                } else {
                                    "follow vehicle"
                                },
                                if following { theme::OK } else { theme::ACCENT },
                                // `CHK_autopan`, a check box: a click ticks or unticks it.
                                true,
                                {
                                    let map = self.map.clone();
                                    move |_event: &(), window: &mut Window, _cx: &mut gpui::App| {
                                        let ticked = map.borrow().is_following();
                                        map.borrow_mut().set_auto_pan(!ticked);
                                        window.refresh();
                                    }
                                },
                            )),
                        )
                    }),
            )
            .child(self.map_status());
        let pane = div()
            .flex()
            .flex_1()
            .min_w(px(0.0))
            .child(column)
            .children(zoom_column);
        // The flight map's panel holds the gimbal video too (`splitContainer1.Panel2`).
        if planning {
            pane.into_any_element()
        } else {
            gimbal_video::map_place(self, pane.into_any_element(), window, cx)
        }
    }

    /// The strip under the map: what it drew and how long it took.
    fn map_status(&self) -> impl IntoElement {
        let map = self.map.borrow();
        let (drawn, approximate, missing) = map.tile_counts();
        let tiles = if map.has_tiles() {
            format!("tiles: {drawn} drawn, {approximate} coarse, {missing} pending  -  ")
        } else {
            String::new()
        };
        let text = if map.has_fix() {
            format!(
                "{tiles}flight path: {} points recorded, {} drawn in {} path(s), {} refused  -  paint {:.2} ms avg, {:.2} ms worst over {} frames",
                map.path_len(),
                map.drawn_points(),
                map.track_paths(),
                map.track_path_failures(),
                map.paint_ema().as_secs_f64() * 1000.0,
                map.paint_worst().as_secs_f64() * 1000.0,
                map.paints(),
            )
        } else {
            format!(
                "{tiles}waiting for a position fix  -  paint {:.2} ms avg over {} frames",
                map.paint_ema().as_secs_f64() * 1000.0,
                map.paints(),
            )
        };
        let text = match &self.file_status {
            Some(status) => format!("{status}  -  {text}"),
            None => text,
        };

        // min_w(0) plus truncation is load-bearing, not cosmetic: a flex item defaults to
        // min-width:auto, so this line's intrinsic text width was widening the whole column
        // whenever a number grew a digit. The map viewport resized with it - 744px to 755px
        // between frames - which invalidated cached geometry every frame and made the renderer
        // look four times slower than it is.
        div()
            .flex()
            .w_full()
            .min_w(px(0.0))
            .px_2()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .truncate()
            .child(text)
    }
}

impl Render for MissionPlanner {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        // The PLUGINS tab always shows the form: filled again after Save && Close closes it, and
        // at a start on that screen.
        if self.screen == Screen::Plugins && !self.plugins.manager.is_open() {
            self.plugins.open_manager(&self.persisted);
        }
        // No control holding the keyboard: the main window takes it, for `ProcessCmdKey`.
        if window.focused(cx).is_none() {
            self.root_focus.focus(window, cx);
        }
        // In a page, a file the browser's picker gave, typed into the box that asked for it.
        page_files::deliver(window, cx);
        // Who holds the keyboard as this frame starts; see the end of `render`.
        let focused_at_start = window.focused(cx);
        // Counted here because this is the one place that only runs when a frame is actually
        // painted. See smoke.rs: the failure being looked for is a backend that will not
        // initialise, and every earlier signal - a window handle, a running executor - survives
        // that.
        smoke::painted();
        // And whether the window's text system shapes text at all (smoke.rs).
        if smoke::enabled() && smoke::text_unasked() {
            let run = gpui::TextRun {
                len: smoke::TEXT_PROBE.len(),
                font: window.text_style().font(),
                color: gpui::black(),
                background_color: None,
                underline: None,
                strikethrough: None,
            };
            let line =
                window
                    .text_system()
                    .shape_line(smoke::TEXT_PROBE.into(), px(14.0), &[run], None);
            smoke::text_shaped(smoke::different_glyphs(&line));
        }
        // Controls that were not measured in the frame just finished have left the screen.
        probe::begin_frame();
        // What that frame left cut off, for the debug build's banner.
        let viewport = window.viewport_size();
        let place = self.place();
        self.cut_off.update(
            &place,
            layout_guard::important_now(self.screen, self.survey.is_open()),
            (f32::from(viewport.width), f32::from(viewport.height)),
        );
        self.tour_step(cx);
        // Where the planner's file dialogs open, for their lists.
        self.plan_menus.dialog_directory = self.dialog_directory();
        // Under MP_STORM, the frame's cost is timed from here to the marker at the end of the
        // root, less the facts' own work: storm.rs. Under MP_FRAMES, each screen's: frametimes.rs.
        storm::frame_started();
        frametimes::begin(self.screen.label());
        let view = self.telemetry.view();
        frametimes::showing(
            view.state
                .as_ref()
                .map(|state| (state.messages_applied, state.packet_in)),
        );
        frametimes::lap("view");

        // The sticks send from their own thread; this keeps them addressed to the vehicle being
        // flown, notices a device that has gone, runs the Joystick page's timer and does the
        // button functions pressed. Once a frame, whether or not anything shows.
        self.joystick_tick(&view);
        // The warning engine's pass, before the HUD's inputs: its messages are the HUD's.
        self.warnings_tick(&view, window);
        self.hud = self.hud_inputs(&view);
        // What the HUD's menu has set: the Russian flag and the user's items.
        self.fly_data
            .hud_settings
            .apply(&mut self.hud, view.state.as_deref());
        // The units the Planner page set, which `CurrentState`'s getters apply to what the HUD
        // and the quick views show and `FlightData.Activate` names on the HUD.
        // `// C#: GCSViews/FlightData.cs:442-444, ExtLibs/ArduPilot/CurrentState.cs:23-38`
        let units = self.planner.units();
        self.hud.units = units;
        self.fly_data.quick.set_units(units);
        // `hudon`, which `MainV2` read from `CHK_hudshow` and Enable HUD Overlay sets.
        // `// C#: MainV2.cs:938-939; ConfigPlanner.cs:374-378; ExtLibs/Controls/HUD.cs:2005-2008`
        self.hud.hud_on = self.planner.hud_on();
        // The vehicle's banner names its firmware; its parameter documentation follows from it.
        self.telemetry.tick();
        // Handed over once a frame: the shown vehicle's fence as the link has seen it, which the
        // quick view's GeoFenceDist measures from (`CurrentState.cs:1632`); the Planner page's
        // telemetry rates, which its combos set as `cs.rateX` and the saved defaults
        // (`ConfigPlanner.cs:573-640`); and a K-index the start-up download has fetched, which
        // `KIndex_KIndex` writes as `kindex` (`MainV2.cs:3977-3981`).
        quick::set_fence(self.telemetry.fence_points());
        let [attitude, position, status, rc, sensors] = self.planner.rates();
        self.telemetry.hand_over_rates(mp_vehicle::StreamRates {
            attitude,
            position,
            status,
            sensors,
            rc,
        });
        self.persisted.kindex_downloaded();
        frametimes::lap("state");
        // SETUP's and CONFIG's lists: built when their screen shows, built again when MainV2
        // would reload it, closed - deactivating the page showing - when it is left.
        self.backstage_tick(&view);
        // Save Modes' writes go one at a time, each after the last is answered.
        self.flight_modes.tick(&self.telemetry);
        // The Frame Type page's FRAME_CLASS and FRAME_TYPE writes, in the same way.
        self.frame_type.tick(&self.telemetry);
        // The FailSafe page's timers and writes, a number the focus left read, and closing it
        // when the screen changes. A write that failed is said on the status line, where the C#
        // shows a box (the owner's ruling of 2026-09-25).
        if let Some(status) = self.failsafe.tick(
            &self.telemetry,
            &view,
            self.screen == Screen::Setup,
            self.failsafe_focus.is_focused(window),
        ) {
            self.file_status = Some(status);
        }
        // The Battery Monitor's boxes validated as the focus leaves them, its timer, its writes.
        self.battery_monitor.tick(
            &self.telemetry,
            &view,
            self.battery_focus.focused(window),
            self.screen == Screen::Setup,
            &self.persisted,
        );
        // Its write failures on the status line, not in a box: the owner's ruling of 2026-09-25.
        if let Some(status) = self.battery_monitor.take_status() {
            self.file_status = Some(status);
        }
        // Install Firmware's catalogue arriving, a device's arrival probed, the page closing when
        // the screen changes; Force Bootloader's link - either Install Firmware page's - and
        // Bootloader Update's.
        self.install_firmware.tick(self.screen == Screen::Setup);
        self.install_firmware_links();
        // Bootloader Update's second Yes: `doCommand(MAV_CMD.FLASH_BOOTLOADER, 0, 0, 0, 0,
        // 290876, 0, 0)`, waited for as the C# waits (once more after 25 s), then "Upgraded
        // bootloader" or "Failed to upgrade bootloader" - on the status line, where this
        // application says what the C# puts in a message box. Unanswered, `doCommand` throws,
        // and the handler's `catch` shows the exception: its message here.
        // `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:30-44;
        // ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2753-2757, 2784-2797`
        if self.install_firmware.take_bootloader_command()
            && let Some(id) = view.vehicle
        {
            let report = telemetry::Report {
                accepted: Some(config::firmware::UPGRADED_BOOTLOADER.to_owned()),
                refused: Some(config::firmware::FAILED_TO_UPGRADE_BOOTLOADER.to_owned()),
                timed_out: Some(config::firmware::DO_COMMAND_TIMEOUT.to_owned()),
                fallback: None,
            };
            self.telemetry.command(
                id,
                mp_link::requests::CMD_FLASH_BOOTLOADER,
                [0.0, 0.0, 0.0, 0.0, 290_876.0, 0.0, 0.0],
                report,
            );
        }
        // The Radio Calibration page's bars, its calibration loop, and its writes and binds.
        self.radio_input
            .tick(&mut self.telemetry, &view, self.screen == Screen::Setup);
        // The Motor Test page's boxes validated as the focus leaves them, its commands' outcomes,
        // and its Spin write.
        self.motor_test
            .tick(&self.telemetry, self.motor_focus.focused(window));
        // The Planner page's number boxes validated as the focus leaves them.
        self.planner_tick(window);
        // The camera's latest frame made the HUD's picture, and the capture's state for the page.
        self.video_tick(window, cx);
        // The Compass page's writes and commands, its calibration timer, and its boxes, which
        // take the keyboard while they show.
        self.compass.tick(
            &mut self.telemetry,
            &view,
            self.screen == Screen::Setup,
            self.compass_focus.declination(window),
            web_time::Instant::now(),
        );
        if self.compass.dialog().is_some() && !self.compass_focus.dialog.is_focused(window) {
            self.compass_focus.dialog.focus(window, cx);
        }
        // The Servo Output, Serial Ports and ESC Calibration pages: each page object disposed
        // with its screen, a number that lost the focus read, the numbers' timers, and every
        // write's answer.
        let on_setup = self.screen == Screen::Setup;
        let now = web_time::Instant::now();
        self.servo_output.tick(
            &self.telemetry,
            &view,
            on_setup,
            self.servo_focus.is_focused(window),
            now,
        );
        self.serial_ports
            .tick(&self.telemetry, &view, on_setup, metadata::lookup);
        self.esc_calibration.tick(
            &self.telemetry,
            &view,
            on_setup,
            self.esc_focus.is_focused(window),
            now,
        );
        // ---- Mandatory Hardware pages ----
        // The Accel Calibration page: what the vehicle says to its subscriptions, shown or not,
        // and its blocking commands' answers; the page object disposed with its screen.
        self.accel_calibration
            .tick(&self.telemetry, &view, on_setup);
        // The older Frame Type page's FRAME writes, one at a time, and its Default Settings
        // control's fetches and ParamCompare writes; a write or fetch that failed is said on the
        // status line (the owner's ruling of 2026-09-25).
        self.frame_type_legacy.dispatch();
        self.frame_type_legacy
            .tick(&self.telemetry, &view, on_setup);
        if let Some(status) = self.frame_type_legacy.take_link_errors() {
            self.file_status = Some(status);
        }
        // The Secure page object let go with its screen; what its handlers threw - the C#'s
        // unhandled-exception box - said on the status line.
        self.secure.tick(&view, on_setup);
        if let Some(status) = self.secure.take_thrown() {
            self.file_status = Some(status);
        }
        // ---- end Mandatory Hardware pages ----
        // Optional Hardware pages: their page objects, timers, boxes and writes.
        self.optional_tick(&view, window);
        // ---- Basic Tuning / Advanced ----
        // Basic Tuning's page object, the box the focus left, Refresh Params and Write Params.
        self.basic_tuning_tick(&view, window);
        // ---- end Basic Tuning / Advanced ----
        // ---- Extended Tuning ----
        // CONFIG's Extended Tuning page: its page object, the box the focus left, Write Params'
        // and Refresh Screen's calls.
        self.extended_tuning_tick(&view, window);
        // ---- end Extended Tuning ----
        // ---- GeoFence / rover Basic Tuning / User Params ----
        // Their page objects, the box the focus left, the timers and the writes.
        self.software_tick(&view, window);
        // ---- end GeoFence / rover Basic Tuning / User Params ----
        // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
        // Their page objects, the box the focus left, the timers, the writes and the MAVFtp
        // commands.
        self.software2_tick(&view, window);
        // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
        // ---- SETUP's small pages (row 70) ----
        // Their page objects, the box the focus left, the timers, the calibration's statuses
        // and the writes.
        self.extra_setup_tick(&view, window);
        // ---- end SETUP's small pages ----
        // `ProcessCmdKey`'s forms: the box the focus left, DevOps' answer, the camera test.
        self.key_forms_tick(window);
        // ---- RTK/GPS Inject ----
        // The page object disposed with its screen, what its thread did, its timer, and where
        // its thread sends.
        self.rtk_inject_tick(&view, window, cx);
        // ---- end RTK/GPS Inject ----
        // ---- Firmware Legacy / Ateryx ----
        // Install Firmware Legacy's list and flow threads and its page object, and the focus of
        // the firmware pages' file dialog; Ateryx Pids' page object, box, refresh, flash command
        // and Write Params' sets.
        self.firmware_legacy_tick(window, cx);
        self.ateryx_tick(&view, window);
        // ---- end Firmware Legacy / Ateryx ----
        // ---- Geo Reference ----
        // The Geo Reference Images form: a run's lines and its end, the map's fit, and its
        // dialog's hold on the keyboard.
        self.georef.tick(window, cx);
        // ---- end Geo Reference ----
        // ---- SITL ----
        // The SITL page: a box the focus left, a start's news and its connection, the probe.
        self.sitl_tick(window);
        // ---- end SITL ----
        // The HELP screen's update check and update: what their threads have said.
        self.help_tick(cx);
        self.crash_tick();
        let banner = self.telemetry.firmware_banner().map(str::to_owned);
        let mav_type = view.state.as_ref().map_or(0, |state| state.vehicle_type);
        self.metadata.advance(banner.as_deref(), mav_type);
        frametimes::lap("pages");
        // The flight screen's clock: `cs.lastautowp`, a Resume Mission moved on a step, and the
        // Transponder page's look for a status.
        self.fly_tick(&view, window);
        // The RAW Sensor window's 10 ms sample.
        self.raw_sensor_tick(&view);
        // The built-in HTTP server's clients: their commands, the packet tap, the snapshot.
        self.http_tick(&view);
        frametimes::lap("fly");
        // ---- row 96 ----
        // The plugins' snapshot, and what they did and asked since the last frame.
        self.plugins_tick(&view, window, cx);
        // ---- end row 96 ----
        frametimes::lap("plugins");
        // The sets and commands the link is retrying: what those that ended say goes on the
        // status line, where this application says what the C# puts in a message box, and
        // parameter writes move on to their next.
        for said in self.telemetry.take_reports() {
            self.file_status = Some(said);
        }
        // A plain reboot on a serial port: the port looked at half a second on and, gone,
        // opened again as the connect button opens it, its parameters fetched afresh; a port
        // that will not open is said on the status line. `// C#: MAVLinkInterface.cs:2573-2583`
        match self
            .telemetry
            .reopen_after_reboot(web_time::Instant::now(), Telemetry::connect)
        {
            None => {}
            Some(telemetry::Reopened::Connecting) => {
                self.file_status = Some(telemetry::CONNECTING_MAVLINK.to_owned());
            }
            Some(telemetry::Reopened::Opened) => {
                self.params_requested = false;
                self.file_status = Some(format!("connected to {}", self.telemetry.view().target));
            }
            Some(telemetry::Reopened::Failed(text)) => self.file_status = Some(text),
        }
        self.advance_param_writes();
        // ---- row 82 ----
        // The Full Parameter List's Activate and Deactivate, its fetches and a reset under way.
        self.raw_params_tick();
        // ---- end row 82 ----

        // Home on the map: the planner's boxes on the planning screen, the vehicle's home on the
        // flight screen once its mission is held - one map, two homes.
        let map_home = if self.screen == Screen::Plan {
            plan::planner_map_home(&self.plan)
        } else {
            plan::flight_map_home(
                view.state.as_ref().and_then(|state| state.home),
                !view.wps.is_empty(),
                self.plan.planned_home_location(),
            )
        };
        self.map.borrow_mut().set_home(map_home);
        // The mission overlay each screen builds: the planner's with its WP Radius and Loiter
        // Radius boxes, the flight screen's with none; and home's altitude, for its tooltip -
        // `homeplla`'s on the flight screen, the vehicle's home or else the planned one.
        // `// C#: GCSViews/FlightPlanner.cs:1423-1434; GCSViews/FlightData.cs:3812-3843`
        let (overlay, home_altitude) = if self.screen == Screen::Plan {
            (
                plan::map_overlay(&self.plan),
                self.plan.home().map(|home| home.alt),
            )
        } else {
            let vehicle = view.state.as_ref().and_then(|state| {
                state
                    .home
                    .filter(|home| home.latitude() != 0.0 || home.longitude() != 0.0)
                    .map(|_| state.home_altitude.0)
            });
            let planned = self.plan.planned_home_location().alt;
            (
                Some(mapview::Overlay::FLIGHT),
                map_home.map(|_| vehicle.unwrap_or(planned)),
            )
        };
        self.map.borrow_mut().set_overlay(overlay);
        self.map.borrow_mut().set_home_altitude(home_altitude);
        // And the flight screen's Guided Mode marker, with its WP radius circle.
        let guided = if self.screen == Screen::Plan {
            None
        } else {
            plan::guided_marker(
                view.state.as_deref().and_then(fly::mode_name),
                self.fly_actions.guided,
                &self.plan,
            )
        };
        self.map.borrow_mut().set_guided(guided);
        // A panel box that lost the keyboard since the last frame has its Leave, and a row of
        // parameter sets moves on.
        if self.screen == Screen::Plan {
            plan::track_panel_focus(self, window);
            // The Lat box entered since the last frame: `TXT_homelat_Enter`.
            plan::track_home_focus(self, window, cx);
        }
        // `GMaps.Instance.Mode`, as `srtm.getAltitude` reads it: the Planner page's Map Access
        // Mode (MP_OFFLINE standing in for CacheOnly, as it does for the map's tiles).
        // `// C#: Program.cs:321-325; ExtLibs/Utilities/srtm.cs:385`
        srtm::set_cache_only(
            std::env::var_os("MP_OFFLINE").is_some()
                || config::planner::cache_only(&self.persisted),
        );
        plan::drop_suppressed_prompt(self);
        plan::drive_writes(self, &view, window, cx);
        // A quick view chosen since the last frame goes into Mission Planner's config.xml, as the
        // chooser's check box puts it there.
        self.persisted.observe_quick_views(&self.fly_data.quick);
        // Map Tool > Zoom To's answer, once the geocoder has sent it.
        plan::drive_geocode(self, window, cx);
        frametimes::lap("plan");

        // Facts a UI test can assert on. Recorded from render because that is where every one of
        // them is already in hand, and published at the end of the frame so a reader never sees
        // half a set. Costs nothing unless MP_FACTS names a file.
        if facts::enabled() {
            let harness = web_time::Instant::now();
            facts::record("screen", self.screen.label());
            logs_tab::record_facts(self.logs_page);
            self.persisted.record_facts();
            i18n::record_facts();
            // Where the map draws home, as `latitude,longitude`, read back from the map, and
            // whether the last paint wrote its "H".
            {
                let map = self.map.borrow();
                facts::record(
                    "map.home",
                    map.home().map_or_else(
                        || "none".to_owned(),
                        |home| format!("{},{}", home.latitude(), home.longitude()),
                    ),
                );
                facts::record("map.home.label", map.home_label_drawn());
            }
            facts::record("mission.items", self.plan.items().len());
            facts::record("mission.origin", self.plan.origin().label());
            // The mission transfer's words, as the action panel shows them, and the map's zoom.
            facts::record(
                "plan.transfer",
                view.transfer
                    .as_ref()
                    .map_or("none", |status| status.label.as_str()),
            );
            facts::record(
                "map.zoom",
                self.map
                    .borrow()
                    .zoom_level()
                    .map_or_else(|| "none".to_owned(), |zoom| format!("{zoom:.2}")),
            );
            facts::record("plan.frame", self.altitude_frame.key());
            facts::record("map.source", self.tile_source_id().unwrap_or("none"));
            facts::record(
                "map.attribution",
                self.map.borrow().attribution().unwrap_or(""),
            );
            // What the last paint drew, and where the store got it. Together they let a test
            // prove imagery came from the disk cache and nothing was fetched - the one claim a
            // screenshot cannot make, since a fetched tile and a cached one look the same. The
            // counts are the previous paint's, because painting happens after this.
            {
                let map = self.map.borrow();
                let (drawn, approximate, missing) = map.tile_counts();
                let stats = map.tile_stats().unwrap_or_default();
                facts::record("map.ready", map.has_view());
                facts::record("map.tiles.drawn", drawn);
                facts::record("map.tiles.approximate", approximate);
                facts::record("map.tiles.missing", missing);
                facts::record("map.tiles.disk", stats.disk_hits);
                facts::record("map.tiles.fetched", stats.fetched);
                facts::record("map.grid.lines", map.grid_lines_drawn());
            }
            // Every frame in the mission, deduplicated. A test asserting on this catches a
            // waypoint created in the wrong frame, which every other field would hide.
            facts::record("mission.frames", {
                let mut frames: Vec<String> = self
                    .plan
                    .items()
                    .iter()
                    .map(|item| plan::frame_label(item.frame))
                    .collect();
                frames.dedup();
                frames.join(",")
            });
            facts::record("fence.points", self.plan.fence().len());
            facts::record("rally.points", self.plan.rally().len());
            // The rally pins the map draws: `rallypointoverlay`'s markers.
            facts::record("map.rally", self.map.borrow().rally_count());
            // The mission waypoints the map draws, and the vehicle's lists as the link holds
            // them: `MAV.wps.Count` and `MAV.rallypoints.Count`.
            facts::record("map.mission", self.map.borrow().mission_len());
            facts::record("vehicle.wps", view.wps.len());
            facts::record("vehicle.rally", view.rally_points.len());
            facts::record("vehicle.connected", view.connected);
            facts::record("vehicle.count", view.vehicle_count);
            // `cs.HomeLocation`, once the vehicle has sent HOME_POSITION.
            facts::record(
                "vehicle.home",
                view.state
                    .as_ref()
                    .and_then(|state| state.home)
                    .map_or_else(
                        || "none".to_owned(),
                        |home| format!("{},{}", home.latitude(), home.longitude()),
                    ),
            );
            facts::record("link.frames", view.frames);
            // What the link was opened on and why it is not open, for a script that finds no
            // vehicle: `MainV2.comPort.BaseStream.PortName` and `OpenBg`'s exception.
                        facts::record("link.target", &view.target);
            facts::record("link.error", self.telemetry.error().unwrap_or("none"));
            // A transport that closed under the link, and the link's tries to open it again.
            facts::record("link.reconnecting", view.reconnecting);
            facts::record("link.reconnects", view.reconnects);
            facts::record("link.reconnect.attempts", view.reconnect_attempts);
            facts::record(
                "link.reconnect.error",
                view.reconnect_error.as_deref().unwrap_or("none"),
            );
            facts::record("params.held", view.parameters.len());
            facts::record("params.expected", view.parameters_expected);
            facts::record("params.fetch", &view.parameters_fetch);
            facts::record("params.defaults", view.parameters_defaults.len());
            facts::record("params.none_default", self.param_none_default);
            // The search box, and what is selected in it: `start,end` in characters, or `none`.
            facts::record("params.search", self.param_search.value());
            facts::record(
                "params.search.selection",
                self.param_search.selection_fact(),
            );
            // ---- row 82 ----
            self.raw_params_facts(&params::collect(&view));
            // ---- end row 82 ----
            // ---- ConfigRawParams remainder ----
            self.param_grid_facts(&params::collect(&view));
            // ---- end ConfigRawParams remainder ----
            // The last parameter write to end: which, how the vehicle answered, and how many
            // times the link put the PARAM_SET on the wire - one, unless it had to ask again.
            let written = self.last_param_write.as_ref();
            facts::record(
                "params.write.name",
                written.map_or("none", |written| written.name.as_str()),
            );
            facts::record(
                "params.write.outcome",
                written.map_or("none", params::Written::outcome_word),
            );
            facts::record(
                "params.write.sends",
                written.map_or(0, |written| written.sends),
            );
            facts::record("tuning.visible", self.tuning.is_visible());
            facts::record("tuning.series", self.tuning.series().len());
            facts::record("log.open", self.log_browse.is_open());
            facts::record("log.fields", self.log_browse.fields().len());
            facts::record("log.plotted", self.log_browse.plotted().len());
            // The two axes. A test that right-clicks a field can prove it went on the right
            // rather than merely on the plot, and that the left side split by unit.
            facts::record("log.plotted.right", self.log_browse.right_count());
            facts::record("log.axes.left", self.log_browse.left_units().len());
            // The data grid: rows it holds (all records, or one type's when filtered), records in
            // the log, the field its current cell resolves to, and the last refusal of Graph
            // Left or Graph Right in the C#'s words.
            facts::record("log.grid.rows", self.log_browse.grid_rows());
            facts::record("log.grid.records", self.log_browse.grid_records());
            facts::record("log.grid.selected", self.log_browse.selected_field());
            facts::record("log.refused", self.log_browse.refused().unwrap_or("none"));
            // The map beside the chart: points of the route drawn, and the logged mission.
            facts::record("log.map.points", self.log_browse.map_contents().points);
            facts::record(
                "log.map.waypoints",
                self.log_browse.map_contents().waypoints,
            );
            // The strip's check boxes, the chart's labels, and the cursor a double click puts on
            // it with the map's marker: listed where they are made, in `LogBrowse::facts`.
            for (key, value) in self.log_browse.facts() {
                facts::record(key, value);
            }
            // What the primary flight display drew, by name, and how many of HUD.cs's elements
            // it cannot show for want of a value - so a port that regresses an element fails a
            // test. Then the health readouts as painted: Vibe's and EKF's colours, the pre-arm
            // line, and whether the angle-of-attack elements showed.
            let hud_scene = hud::scene(&self.hud, 800.0, 260.0);
            facts::record("hud.drawn", hud_scene.drawn_names());
            facts::record("hud.missing", hud::missing().len());
            facts::record("hud.missing.list", hud::missing_report());
            for (key, value) in hud::health_facts(&hud_scene) {
                facts::record(key, value);
            }
            // And the numbers with their units, the battery and GPS lines, and the pictures.
            for (key, value) in hud::readout_facts(&hud_scene) {
                facts::record(key, value);
            }
            // How much of FlightData this screen has, from the coverage table, so the number in
            // the plan is the number the application reports.
            let (done, elsewhere, missing, plumbing, dropped) = coverage::counts();
            facts::record("coverage.flightdata.done", done + elsewhere);
            facts::record("coverage.flightdata.missing", missing);
            facts::record("coverage.flightdata.total", coverage::FLIGHTDATA.len());
            let _ = (plumbing, dropped);
            // The same for the planning screen, and what its map menu has done to the mission.
            let (done, elsewhere, missing, _, _) = planner_coverage::counts();
            facts::record("coverage.flightplanner.done", done + elsewhere);
            facts::record("coverage.flightplanner.missing", missing);
            facts::record(
                "coverage.flightplanner.total",
                planner_coverage::FLIGHTPLANNER.len(),
            );
            // And Mission Planner's setup and configuration panels (Deliverable 12), from their ledger.
            for (key, value) in config_coverage::facts() {
                facts::record(key, value);
            }
            plan::record_facts(&self.plan, &self.plan_menus);
            inject_map::record_facts(self.inject_map.as_ref());
            self.http.server.record_facts();
            facts::record(
                "plan.kml.link",
                match &self.kml_link {
                    None => "none".to_owned(),
                    Some(Ok(())) => format!("opened {}", plan::NETWORK_KML_URL),
                    Some(Err(why)) => format!("failed: {why}"),
                },
            );
                        connect::record_facts(&self.connect_box, view.connected);
            // The important controls of this screen that are not wholly on screen - the owner's
            // self-test of 2026-10-03, read by every GUI run (`layout_guard`).
            layout_guard::record_facts(layout_guard::important_now(
                self.screen,
                self.survey.is_open(),
            ));
            prefetch_ui::record_facts(&self.plan_menus);
            facts::record("plan.docking", self.plan_docking.name());
            // The map's zoom and centre, its radius circles and what the pointer is over, and the
            // zoom controls beside it.
            {
                let map = self.map.borrow();
                for (key, value) in map.facts() {
                    facts::record(key, value);
                }
                for (key, value) in plan::zoom_facts(&self.plan_menus, map.zoom_level()) {
                    facts::record(key, value);
                }
            }
            survey_ui::record_facts(&self.survey);
            // Where the parameter documentation comes from and how much of this vehicle it
            // covers: PLAN.md 10.5's measurement, live.
            facts::record("params.metadata.source", metadata::source());
            facts::record("params.metadata.documented", metadata::documented());
            facts::record(
                "params.metadata.covered",
                view.parameters
                    .iter()
                    .filter(|(name, _)| metadata::lookup(name).is_some())
                    .count(),
            );
            facts::record(
                "params.metadata.status",
                self.metadata.status.as_deref().unwrap_or("idle"),
            );
            facts::record("sticks.enabled", self.sticks.is_enabled());
            joystick::record_facts(&self.sticks);
            // Frames the link accepted and the measured stick-to-link latency, so a test with a
            // device attached can prove frames go out and how fast.
            facts::record("sticks.sent", self.sticks.sent());
            facts::record(
                "sticks.p99_us",
                self.sticks.latency().map_or(0, |(_, p99)| p99.as_micros()),
            );
            config::flight_modes::record_facts(&self.flight_modes, &view);
            facts::record("recording", self.telemetry.recording().is_some());
            facts::record("status", self.file_status.as_deref().unwrap_or(""));
            // What the Actions tab last put on the wire and what the vehicle said back.
            self.fly_actions.record_facts(&view);
            self.fly_data.record_facts(
                &view,
                self.telemetry.log_listings().len(),
                fly::alt_offset_home(&view),
            );
            self.raw_sensor.record_facts(&view);
            // The state's clock, its counts, its rates and the fence handed to the quick view.
            self.telemetry.record_facts(&view);
            self.fly_pages
                .record_facts(f32::from(self.fly_scroll.max_offset().y));
            // The flown route's points, which Clear Track empties.
            facts::record("fly.track", self.map.borrow().path_len());
            config::failsafe::record_facts(&self.failsafe, &view);
            setup::record_facts([&self.setup_list, &self.config_list]);
            config::frame_type::record_facts(&self.frame_type, &view);
            config::battery_monitor::record_facts(&self.battery_monitor, &view);
            config::firmware::record_facts(&self.install_firmware, &self.persisted);
            config::radio::record_facts(&self.radio_input, &view);
            config::motor_test::record_facts(&self.motor_test, &view);
            config::compass::record_facts(&self.compass, &view);
            config::servo_output::record_facts(&self.servo_output, &view);
            config::serial_ports::record_facts(&self.serial_ports, &view);
            config::esc_calibration::record_facts(&self.esc_calibration, &view);
            // Optional Hardware pages
            config::optional::record_facts(&self.optional, &view);
            experimental::record_facts(&self.experimental);
            // ---- RTK/GPS Inject ----
            config::rtk_inject::record_facts(&self.rtk_inject, &view, &self.persisted);
            // ---- end RTK/GPS Inject ----
            // ---- Mandatory Hardware pages ----
            config::accel_calibration::record_facts(&self.accel_calibration, &view);
            config::frame_type_legacy::record_facts(&self.frame_type_legacy, &view);
            config::secure::record_facts(&self.secure);
            // ---- end Mandatory Hardware pages ----
            // ---- Basic Tuning / Advanced ----
            config::basic_tuning::record_facts(
                &self.basic_tuning,
                self.config_list
                    .pages()
                    .iter()
                    .any(|row| row.class == config::basic_tuning::CLASS),
            );
            config::advanced::record_facts(
                self.setup_list
                    .page()
                    .is_some_and(|entry| entry.class == "ConfigAdvanced"),
            );
            // ---- end Basic Tuning / Advanced ----
            config::planner::record_facts(&self.planner, &self.persisted, self.auto_read_mission);
            // ---- Extended Tuning ----
            config::extended_tuning::record_facts(&self.extended_tuning, &view);
            // ---- end Extended Tuning ----
            // ---- GeoFence / rover Basic Tuning / User Params ----
            config::software_pages::record_facts(
                &self.software_pages,
                self.config_list
                    .pages()
                    .iter()
                    .any(|row| row.class == config::rover_tuning::CLASS),
                &view,
            );
            // ---- end GeoFence / rover Basic Tuning / User Params ----
            // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
            config::software_pages2::record_facts(&self.software_pages2, &view);
            // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
            // ---- SETUP's small pages (row 70) ----
            config::extra_setup::record_facts(&self.extra, &view);
            warnings::record_facts(&self.warnings, self.hud.message.as_ref());
            // ---- end SETUP's small pages ----
            // ---- Firmware Legacy / Ateryx ----
            config::firmware_legacy::record_facts(
                &self.firmware_legacy,
                &self.persisted,
                self.setup_list
                    .pages()
                    .iter()
                    .any(|row| row.class == config::firmware_legacy::CLASS),
            );
            config::ateryx::record_facts(
                &self.ateryx,
                self.config_list
                    .pages()
                    .iter()
                    .any(|row| row.class == config::ateryx::CLASS),
            );
            // ---- end Firmware Legacy / Ateryx ----
            // ---- Geo Reference ----
            georef_ui::record_facts(&self.georef);
            // ---- end Geo Reference ----
            cmd_keys::record_facts(&self.key_forms, &self.persisted);
            // ---- SITL ----
            sitl::record_facts(&self.sitl, &self.persisted);
            help::record_facts(&self.help);
            crash::record_facts(&self.crash);
            // ---- end SITL ----
            facts::publish();
            // The harness's work, which a normal run does not do, is not the frame's.
            storm::exclude(harness.elapsed());
        }

        // The tuning graph is fed here because this is where a fresh snapshot arrives. It samples
        // only when the snapshot is new - a repeated sample draws a horizontal line that looks
        // exactly like a steady measurement, and on a tuning graph "steady" and "nothing arriving"
        // lead to opposite conclusions.
        self.tuning.sample(&view);

        // Feed the map from the same snapshot the panels read, so the two can never disagree
        // about where the vehicle is. The marker is `getMAVMarker`'s for the vehicle's type, its
        // `Heading` `cs.yaw` - ATTITUDE's yaw as degrees from 0 to 360, not GLOBAL_POSITION_INT's
        // heading - and the avoidance radii its `AVD_W_DIST_XY` and `AVD_F_DIST_XY`, both or
        // neither. `// C#: Common.cs:71-215`
        if let Some(state) = view.state.as_ref() {
            let yaw = quick::value("yaw", state).unwrap_or(0.0);
            let parameter = |name: &str| {
                view.parameters
                    .iter()
                    .find(|(held, _)| held == name)
                    .map(|(_, value)| *value)
            };
            let (warn, danger) = match (parameter("AVD_W_DIST_XY"), parameter("AVD_F_DIST_XY")) {
                #[allow(clippy::cast_possible_truncation)] // `(int)` of the parameters' values
                (Some(w), Some(f)) => ((w as i32) as f32, (f as i32) as f32),
                _ => (-1.0, -1.0),
            };
            #[allow(clippy::cast_possible_truncation)] // `(float)` of the C#'s double
            let details = mapview::MarkerDetails {
                kind: mapview::MarkerKind::of(state.vehicle_type),
                cog: state.gps.course,
                nav_bearing: state.nav.bearing,
                target: state.nav.target_bearing,
                sysid: state.sysid,
                warn,
                danger,
                radius: quick::value("radius", state).unwrap_or(0.0) as f32,
            };
            let settings = self.marker_settings();
            let on_flight_screen = self.screen == Screen::Fly;
            let mut map = self.map.borrow_mut();
                        // A position at 0,0 - a GPS before its fix - draws no marker and no route point,
            // as `addMAVMarker` and the route add none (the owner's report, 2026-10-03).
            match state.position.filter(|position| mapview::is_fixed(*position)) {
                Some(position) => {
                    map.observe(position, mp_units::Bearing(mp_units::Degrees(yaw)));
                    // Auto Pan, the flight screen's alone, as Mission Planner's planning map has
                    // none. `// C#: GCSViews/FlightData.cs:4242-4253`
                    if on_flight_screen {
                        map.auto_pan(web_time::Instant::now());
                    }
                    // `Settings.Instance["CHK_autopan"] = CHK_autopan.Checked.ToString()` on a
                    // change. `// C#: GCSViews/FlightData.cs:1929-1933`
                    let ticked = if map.is_following() { "True" } else { "False" };
                    if self.persisted.get(AUTO_PAN_SETTING) != Some(ticked) {
                        self.persisted.set(AUTO_PAN_SETTING, ticked);
                    }
                }
                None => map.vehicle_unfixed(),
            }
            map.set_marker(details, settings);
        }

        // A completed download replaces the plan only if the operator asked for one. Otherwise it
        // just goes to the map, so a read started for display cannot overwrite an edit.
        if !view.mission.is_empty() && view.mission_complete && self.adopt_vehicle_mission {
            self.adopt_vehicle_mission = false;
            if let Some(home) = self.plan.adopt_from_vehicle(&view.mission) {
                self.plan_menus.offer_home_reset(home);
                self.plan_prompt_focus.focus(window, cx);
            }
            // `processToScreen` ends with `setWPParams`.
            // `// C#: GCSViews/FlightPlanner.cs:5630`
            self.plan.set_wp_params(&view.parameters);
            // The read's item 0 as the vehicle sent it: its home, or 0,0 while it has none.
            facts::record(
                "mission.read.home",
                view.mission.first().map_or_else(
                    || "none".to_owned(),
                    |item| format!("{},{}", item.x, item.y),
                ),
            );
            self.file_status = Some(format!(
                "read {} items from the vehicle",
                view.mission.len()
            ));
        }

        // The map shows the plan being edited when there is one, and what the vehicle holds
        // otherwise - `MAV.wps`, the mission as the link's traffic has shown it: read, written,
        // or put there by a script's `setWP`s, which is what the C#'s flight map draws. Showing
        // the vehicle's mission while the operator draws a different one is how people fly the
        // mission they thought they had replaced. While the Survey (Grid) dialog is open it is
        // the dialog's map, showing its grid.
        // `// C#: GCSViews/FlightData.cs:3810-3843`
        if let Some((preview, _)) = self.survey_preview() {
            self.map.borrow_mut().set_mission(&preview);
        } else if self.plan.is_empty() {
            self.map.borrow_mut().set_mission(&view.wps);
        } else {
            self.map.borrow_mut().set_mission(self.plan.items());
        }
        // The rally markers likewise: the plan's while it has any, else the vehicle's as the
        // traffic has shown them - `MAV.rallypoints`, which the flight screen draws.
        // `// C#: GCSViews/FlightData.cs:3898-3905`
        if self.plan.rally().is_empty() {
            let positions: Vec<mp_units::LatLon> = view
                .rally_points
                .iter()
                .filter_map(|item| item.position().ok().flatten())
                .collect();
            self.map.borrow_mut().set_rally(&positions);
        }

        // Other aircraft. Read every frame because the link forgets stale ones on read, and a
        // display that only updated on an event would keep a symbol after its aircraft had gone.
        {
            let now = web_time::Instant::now();
            let traffic: Vec<(mp_units::LatLon, bool)> = self
                .telemetry
                .traffic()
                .into_iter()
                .map(|aircraft| (aircraft.position, aircraft.is_stale(now)))
                .collect();
            self.map.borrow_mut().set_traffic(&traffic);
        }

        // The camera's shots: `photosoverlay`'s photo markers and, with Camera Overlap checked,
        // `kmlpolygons`' overlap count - the map loop's camera half, built again only when a
        // shot, the toggle, CAM_MIN_INTERVAL or the fields of view have changed.
        // `// C#: GCSViews/FlightData.cs:4001-4082`
        {
            let points = self.telemetry.camera_points();
            let min_interval = view
                .vehicle
                .and_then(|id| self.telemetry.parameter_of(id, "CAM_MIN_INTERVAL"))
                .map_or(0.0, |value| value / 1000.0);
            let fov = camera_photos::fov(|key| self.persisted.get(key).map(str::to_owned));
            let overlap = self.fly_data.camera_overlap;
            self.fly_data.photos.refresh(
                &points,
                min_interval,
                fov,
                overlap,
                &camera_photos::PlannerTerrain,
            );
            let mut map = self.map.borrow_mut();
            map.set_photos(self.fly_data.photos.photos());
            map.set_coverage(self.fly_data.photos.coverage());
        }

        // Keep a log download moving, write it out when it finishes and start the next of the
        // Log Downloader's batch. Driven from the render pass because that is the only thing
        // ticking; the link cannot write files and should not decide where they go.
        self.logs_tick();
        // Inject Custom Map's run: its end switches the map to Custom and shows the results.
        self.inject_map_tick();

        // Keep re-sending a forced arm until it takes, or until we give up. The parameter write
        // that disabled the checks may not have been applied when the first command arrived.
        if let Some(deadline) = self.forcing_arm_until {
            let armed = view.state.as_ref().is_some_and(|state| state.armed);
            if armed {
                self.forcing_arm_until = None;
                self.file_status = Some("armed with arming checks disabled".to_owned());
            } else if web_time::Instant::now() >= deadline {
                self.forcing_arm_until = None;
                self.file_status = Some(
                    "forced arm gave up; the vehicle is still refusing - see the messages"
                        .to_owned(),
                );
            } else {
                // Once per heartbeat interval, so each attempt is judged against a state that
                // could have changed since the last one - and never while the link is still
                // retrying the last: `doARM` blocks its caller until the vehicle answers or its
                // retries run out, so the C# cannot ask again before then either.
                // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2632-2645, 2764-2768`
                const BETWEEN_ATTEMPTS: std::time::Duration =
                    std::time::Duration::from_millis(1000);
                let now = web_time::Instant::now();
                let in_flight = self.force_arm_request.is_some_and(|(id, made)| {
                    match self.telemetry.lookup(id, made) {
                        telemetry::Lookup::Found(request) => !request.is_finished(),
                        telemetry::Lookup::PickingUp => true,
                        telemetry::Lookup::Gone => false,
                    }
                });
                if !in_flight
                    && self
                        .last_force_arm
                        .is_none_or(|last| now.duration_since(last) >= BETWEEN_ATTEMPTS)
                {
                    self.last_force_arm = Some(now);
                    self.force_arm_request = self.telemetry.force_arm().map(|id| (id, now));
                }
            }
        }

        // A completed fence download replaces the fence only if the operator asked for one.
        if self.adopt_vehicle_fence {
            let fence = self.telemetry.fence_items();
            if !fence.is_empty() {
                self.adopt_vehicle_fence = false;
                self.plan.adopt_fence(&fence);
                self.sync_map_fence();
                self.file_status = Some(format!(
                    "read a fence of {} items from the vehicle",
                    fence.len()
                ));
            }
        }

        if self.adopt_vehicle_rally {
            let rally = self.telemetry.rally_items();
            if !rally.is_empty() {
                self.adopt_vehicle_rally = false;
                self.plan.adopt_rally(&rally);
                self.sync_map_rally();
                self.file_status = Some(format!(
                    "read {} rally points from the vehicle",
                    rally.len()
                ));
            }
        }

        if self.auto_read_mission && !self.mission_requested && view.vehicle.is_some() {
            self.mission_requested = true;
            self.adopt_vehicle_mission = true;
            self.telemetry.request_mission();
        }

        // `Open`'s `getParamListMavftp` (`MAVLinkInterface.cs:930-939`): the parameters fetched
        // as soon as a vehicle is heard - MAVFTP first, the stream after - unless the whole
        // table is already held (the owner's rule, 2026-09-25: only when we have none). A few
        // names read one by one - the pages' `ReadParam`s, which arrive before this runs - are
        // not a copy: found 2026-09-25, when 33 held of 1,400 left the table unfetched.
        if !self.params_requested && view.vehicle.is_some() && !view.target.starts_with("file:") {
            self.params_requested = true;
            if auto_fetch_wanted(view.parameters.len(), view.parameters_expected) {
                self.telemetry.download_parameters();
            }
        }

        let (status, status_colour) = if let Some(err) = self.telemetry.error() {
            (format!("link failed: {err}"), theme::ALERT)
        } else if !view.connected && view.target.is_empty() {
            (
                "no link - start with a url, e.g. tcp:127.0.0.1:5760".to_owned(),
                theme::WARN,
            )
                } else if view.reconnecting {
            // The transport went: the link keeps the vehicle and opens it again every second,
            // and this line is the whole of what the operator is told - no box (the owner's
            // ruling, PLAN.md section 12 D23).
            (
                match &view.reconnect_error {
                    Some(err) => format!(
                        "{}  -  reconnecting, try {}: {err}",
                        view.target, view.reconnect_attempts
                    ),
                    None => format!(
                        "{}  -  reconnecting, try {}",
                        view.target, view.reconnect_attempts
                    ),
                },
                theme::WARN,
            )
        } else if view.connected && view.frames > 0 {
            (
                format!("{}  -  {} frames", view.target, view.frames),
                theme::OK,
            )
        } else if view.connected {
            (
                format!("{}  -  waiting for telemetry", view.target),
                theme::WARN,
            )
        } else {
            (format!("{}  -  closed", view.target), theme::ALERT)
        };

        frametimes::lap("feeds");
        let body = match self.screen {
            Screen::Fly => probe::measured("body", div())
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .gap_2()
                .p_2()
                .child(self.fly_sidebar(&view, window, cx))
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .flex_1()
                        .min_w(px(0.0))
                        .min_h(px(0.0))
                        .gap_2()
                        // The tuning graph above the map, where the C# has it: `zg1` is in
                        // `splitContainer1.Panel1`, over the map in `Panel2`, collapsed until
                        // `CB_tuning` opens it. It is on no page of `tabControlactions`.
                        // `// C#: GCSViews/FlightData.Designer.cs:2472, 2483, 2500`
                        // Swapped, the HUD is here in their place.
                        .children((!self.fly_data.swapped).then(|| {
                            div()
                                .flex_shrink_0()
                                .child(tuning::panel_for(&self.tuning, cx))
                        }))
                        .child(if self.fly_data.swapped {
                            fly::hud_panel(&self.hud, &self.fly_data, cx).into_any_element()
                        } else {
                            self.map_pane(window, cx).into_any_element()
                        })
                        .child(div().flex_shrink_0().child(fly::messages_panel(&view, cx))),
                )
                // The flight screen's dialogs and forms, drawn over the window: hung from a box of
                // no size, so their backdrops are not read as this body running past the window
                // (the layout guard on the owner's Linux desktop, 2026-10-05).
                .child(
                    overlay_layer()
                .children(fly::prompt_dialog(
                    &self.fly_actions,
                    &self.fly_focus,
                    window,
                    cx,
                ))
                // The forms the flight screen shows with `Show()`: over it, and it stays usable.
                .children(fly::hud_windows(&self.fly_data, &view, cx))
                .children(raw_sensor::window(&self.raw_sensor, &view, cx))
                .children(logdownload::window(
                    &self.fly_data.logs,
                    &self.telemetry.log_listings(),
                    window,
                    cx,
                ))
                // `ShowDialog()`: the quick view's chooser, over everything.
                .children(quick::chooser(&self.fly_data.quick, window, cx))
                // The HUD's menu and its User Items form, and Auto Analysis's report.
                .children(fly::overlays(
                    &self.fly_data,
                    &self.plugins.flight_entries(),
                    window,
                    cx,
                ))
                // ---- Geo Reference ----
                // `new Georefimage().Show()`: its form over the screen, which stays usable.
                .children(georef_ui::window(self, window, cx))
                // ---- end Geo Reference ----
                )
                .into_any_element(),
            // The Survey (Grid) dialog is modal: while it shows it is the screen.
            Screen::Plan if self.survey.is_open() => div()
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .p_2()
                .children(survey_ui::form(self, window, cx))
                .into_any_element(),
            Screen::Plan => self.plan_screen(&view, window, cx),
            Screen::Params => self.params_body(&view, window, cx),
            // The flight screen's two log pages and the log browser, each a tab of its own
            // (`logs_tab`, the owner's word of 2026-10-04).
            Screen::Logs => div()
                .flex()
                .flex_col()
                .flex_1()
                .min_h(px(0.0))
                .min_w(px(0.0))
                .child(logs_tab::strip(self.logs_page, cx))
                .child(match self.logs_page {
                    logs_tab::LogsPage::TLogs => logs_tab::page_box(
                        "logs-tlogs",
                        fly::playback_page(&self.fly_data.playback, cx),
                    )
                    .into_any_element(),
                    logs_tab::LogsPage::DataFlash => logs_tab::page_box(
                        "logs-dataflash",
                        fly::dataflash_page(&self.fly_data, cx),
                    )
                    .into_any_element(),
                    logs_tab::LogsPage::Review => div()
                        .id("logs-body")
                        .flex()
                        .flex_1()
                        .min_h(px(0.0))
                        .min_w(px(0.0))
                        .child(logbrowse::screen(
                            &self.log_browse,
                            &self.log_name,
                            &logbrowse::Focus {
                                name: &self.log_name_focus,
                                name_focused: self.log_name_focus.is_focused(window),
                                prompt: &self.log_prompt_focus,
                                prompt_focused: self.log_prompt_focus.is_focused(window),
                                screen: &self.log_screen_focus,
                                info: &self.log_info_focus,
                                info_focused: self.log_info_focus.is_focused(window),
                            },
                            cx,
                        ))
                        .into_any_element(),
                })
                .into_any_element(),
            // Each a backstage view: the list down the left, the chosen page beside it, each
            // scrolling on its own as `pnlMenu` and `pnlPages` do.
            // `// C#: ExtLibs/Controls/BackstageView/BackstageView.Designer.cs:35-52; ExtLibs/Controls/BackstageView/BackStageViewMenuPanel.cs:22`
            Screen::Setup => probe::measured("setup-body", div())
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .child(self.backstage_screen(setup::List::Setup, &view, window, cx))
                // The screen's dialogs, drawn over the whole window: hung from a box of no size,
                // so their backdrops are not read as this body running past the window.
                .child(
                    overlay_layer()
                .children(config::failsafe::overlay(&self.failsafe, window, cx))
                .children(config::battery_monitor::overlay(
                    &self.battery_monitor,
                    &self.battery_focus,
                    window,
                    cx,
                ))
                .children(config::radio::overlay(&self.radio_input, window, cx))
                .children(config::servo_output::overlay(
                    &self.servo_output,
                    window,
                    cx,
                ))
                .children(joystick::overlay(
                    &self.sticks,
                    &self.joystick_focus,
                    window,
                    cx,
                ))
                .children(config::serial_ports::overlay(
                    &self.serial_ports,
                    window,
                    cx,
                ))
                .children(config::esc_calibration::overlay(
                    &self.esc_calibration,
                    window,
                    cx,
                ))
                // ---- Mandatory Hardware pages ----
                .children(config::accel_calibration::overlay(
                    &self.accel_calibration,
                    window,
                    cx,
                ))
                .children(config::frame_type_legacy::overlay(
                    &self.frame_type_legacy,
                    window,
                    cx,
                ))
                .children(config::secure::overlay(
                    &self.secure,
                    &self.secure_focus,
                    window,
                    cx,
                ))
                // ---- end Mandatory Hardware pages ----
                // Optional Hardware pages
                .children(self.optional_overlay(window, cx))
                // ---- RTK/GPS Inject ----
                .children(config::rtk_inject::overlay(
                    &self.rtk_inject,
                    &self.rtk_focus,
                    window,
                    cx,
                ))
                // ---- end RTK/GPS Inject ----
                // ---- SETUP's small pages (row 70) ----
                // Their boxes, and the FFT window that FFT Setup's and Advanced's FFT open: all
                // of them SETUP's pages (`GCSViews/InitialSetup.cs:237-342`).
                .children(self.extra_setup_overlay(window, cx))
                // ---- end SETUP's small pages ----
                // ---- Firmware Legacy / Ateryx ----
                .children(config::firmware::overlay(
                    &self.install_firmware,
                    &self.firmware_focus,
                    window,
                    cx,
                ))
                .children(config::firmware_legacy::overlay(
                    &self.firmware_legacy,
                    &self.firmware_focus,
                    window,
                    cx,
                ))
                // ---- end Firmware Legacy / Ateryx ----
                )
                .into_any_element(),
            Screen::Config => probe::measured("config-body", div())
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .child(self.backstage_screen(setup::List::Config, &view, window, cx))
                // ---- Basic Tuning / Advanced ----
                // The screen's dialogs, drawn over the whole window: hung from a box of no size,
                // so their backdrops are not read as this body running past the window.
                .child(
                    overlay_layer()
                .children(config::basic_tuning::overlay(
                    &self.basic_tuning,
                    window,
                    cx,
                ))
                // ---- end Basic Tuning / Advanced ----
                // ---- Extended Tuning ----
                .children(self.extended_tuning_overlay(window, cx))
                // ---- end Extended Tuning ----
                // ---- GeoFence / rover Basic Tuning / User Params ----
                .children(self.software_overlay(window, cx))
                // ---- end GeoFence / rover Basic Tuning / User Params ----
                // ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
                .children(self.software2_overlay(window, cx))
                // ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
                // ---- Firmware Legacy / Ateryx ----
                .children(config::ateryx::overlay(&self.ateryx, window, cx))
                // ---- end Firmware Legacy / Ateryx ----
                )
                .into_any_element(),
            // ---- SITL ----
            Screen::Sitl => div()
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .child(sitl::view::screen(self, window, cx))
                .children(sitl::view::overlay(self, window, cx))
                .into_any_element(),
            // ---- end SITL ----
            Screen::Help => help::screen(self, window, cx),
            Screen::Experimental => experimental::screen(self, window, cx),
            Screen::Plugins => plugin_manager::screen(self, cx),
        };
        frametimes::lap("body");

        let cut_off = self
            .cut_off
            .text_at(web_time::Instant::now())
            .map(ToOwned::to_owned);
        let root = probe::measured("root", div())
            .relative()
            .flex()
            .flex_col()
            .size_full()
            .overflow_hidden()
            // Lines 1.3 times the text, not gpui's 1.618: 12-pixel text was 19 high, a pixel past
            // every 20-high box's inside, on page after page (the layout guard on the owner's
            // Mac, 2026-10-05). Closer to Mission Planner's own spacing, and it only makes things
            // smaller.
            .line_height(gpui::relative(1.3))
            // The debug build's cut-off guard: over the window's foot, so it moves nothing, and
            // painted after everything else, so nothing covers it.
            .children(cut_off.map(|text| {
                gpui::deferred(
                    div()
                        .absolute()
                        .left_0()
                        .right_0()
                        .bottom_0()
                        .px_2()
                        .py_1()
                        .bg(rgb(theme::ALERT))
                        .text_color(rgb(theme::BG))
                        .text_xs()
                        .child(format!("CUT OFF (the layout guard): {text}")),
                )
                .with_priority(3)
            }))
            // The demo pointer's cursor (the owner's Welcome-Demo-Sitl): over everything, the
            // dialogs and the guard's strip too, as a mouse pointer is.
            .children(
                self.plugins
                    .demo
                    .cursor(web_time::Instant::now())
                    .map(|cursor| gpui::deferred(cursor).with_priority(4)),
            )
            .bg(rgb(theme::BG))
            .text_color(rgb(theme::TEXT))
            // `MainV2.ProcessCmdKey`: a key no element inside took, on its way out.
            // `// C#: MainV2.cs:4067-4182`
            .track_focus(&self.root_focus)
            .on_key_down(cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
                if this.process_cmd_key(&event.keystroke, window, cx) {
                    cx.stop_propagation();
                    cx.notify();
                }
            }))
            .child(
                // Wrapping, both halves, rather than running past the window's right edge: at
                // 1600 wide the recording, the link and the tabs' end were cut off (the owner's
                // Mac, 2026-10-04, as the layout guard's strip said: header 1709 wide).
                probe::measured("header", div())
                    .flex()
                    .flex_wrap()
                    .items_center()
                    .justify_between()
                    .gap_x_4()
                    .px_4()
                    .pt_2()
                    .bg(rgb(theme::PANEL))
                    .border_b_1()
                    .border_color(rgb(theme::BORDER))
                    .child(
                        div()
                            .flex()
                            .flex_wrap()
                            .items_end()
                            .gap_4()
                            .child(
                                div()
                                    .flex()
                                    .flex_col()
                                    .pb_2()
                                                                        .child(div().text_xl().child(PRODUCT_NAME))
                                    // The byline, and Mission Planner's author when the pointer
                                    // is over it (the owner, 2026-10-04).
                                    .child(
                                        div()
                                            .id("header-byline")
                                            .text_xs()
                                            .text_color(rgb(theme::DIM))
                                            .child(BYLINE)
                                            .tooltip(|_window, cx| -> gpui::AnyView {
                                                cx.new(|_| {
                                                    config::rover_tuning::Tip(SharedString::from(
                                                        BYLINE_TIP,
                                                    ))
                                                })
                                                .into()
                                            }),
                                    ),
                            )
                            .child(self.tabs(cx))
                            .child(self.vehicle_picker(&view, cx))
                            .child(self.connection_controls(&view, cx)),
                    )
                    // The connect, update and crash dialogs, drawn over the whole window: hung
                    // from a box of no size, so their backdrops are not read as the header
                    // running past the window.
                    .child(
                        overlay_layer()
                            .children(self.connect_dialogs(window, cx))
                            // The update's question, progress and boxes, on whatever screen is
                            // showing: the once-a-day check asks from the main thread, wherever
                            // the user is.
                            .children(help::overlay(self, window, cx))
                            // A crash report from a last run: its question, on the main thread
                            // at start.
                            .children(crash::overlay(self, window, cx)),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_wrap()
                            .items_center()
                            .gap_2()
                            .pb_2()
                            // The sticks, when they have control - on every screen, because the
                            // panel that switches them on is the seventh of nine on a screen
                            // nobody flies from. A pilot must be able to see that a gamepad is
                            // driving the aircraft, and stop it, without first finding the tab it
                            // was started on.
                            .children(self.sticks.is_enabled().then(|| {
                                div()
                                    .flex()
                                    .items_center()
                                    .gap_1()
                                    .id("sticks-live")
                                    .px_2()
                                    .py(px(1.0))
                                    .rounded_sm()
                                    .border_1()
                                    .border_color(rgb(theme::ALERT))
                                    .cursor_pointer()
                                    .hover(|style| style.bg(rgb(theme::BORDER)))
                                    .child(div().size_2().rounded_full().bg(rgb(theme::ALERT)))
                                    .child(
                                        div()
                                            .text_xs()
                                            .text_color(rgb(theme::ALERT))
                                            .child("sticks flying - click to stop"),
                                    )
                                    .on_click(cx.listener(|this, _event, _window, cx| {
                                        this.sticks.disable_joystick();
                                        cx.notify();
                                    }))
                            }))
                            // Recording, said on screen rather than assumed. The link reports a
                            // failed recording to stderr and carries on, which in an application
                            // launched from a desktop icon means a failed recording and a working
                            // one look identical.
                            .children(self.telemetry.recording().map(|path| {
                                div()
                                    .flex()
                                    .items_center()
                                    .gap_1()
                                    .child(div().size_2().rounded_full().bg(rgb(theme::ALERT)))
                                    .child(div().text_xs().text_color(rgb(theme::DIM)).child(
                                        format!(
                                            "recording {}",
                                            path.file_name().map_or_else(
                                                || path.display().to_string(),
                                                |name| name.to_string_lossy().into_owned()
                                            )
                                        ),
                                    ))
                            }))
                            .child(div().size_2().rounded_full().bg(rgb(status_colour)))
                            .child(div().text_sm().text_color(rgb(theme::DIM)).child(status)),
                    ),
            )
            .child(body)
            // Dialogs over every screen, hung from a box of no size so their backdrops are not
            // read as the window's own content running past it.
            .child(
                overlay_layer()
                    // The Compass page's boxes: modal over every screen, since leaving SETUP
                    // can ask one.
                    .children(config::compass::overlay(
                        &self.compass,
                        &self.compass_focus,
                        window,
                        cx,
                    ))
                    // `ProcessCmdKey`'s forms, free forms in the C#: over whichever screen shows.
                    .children(cmd_keys::overlay(self, window, cx))
                    // ---- row 96 ----
                    // The plugins' questions and forms.
                    .children(plugins_ui::overlay(self, window, cx)),
                // ---- end row 96 ----
            )
            // MP_FRAMES's readout of the last second's frames, over the bottom-right corner.
            .children(frametimes::readout())
            // Last, so its paint ends the frame's measurement; absent without MP_STORM.
            .children(storm::marker(
                view.frames,
                view.state.as_ref().and_then(|state| state.packet_in),
            ))
            // The same for MP_FRAMES; absent without it.
            .children(frametimes::marker());
        frametimes::rendered();
        // A control given the keyboard while this frame was built - a prompt opened and focused
        // as it is drawn - shows its caret, and the controls drawn before it their lost focus,
        // only on the frame after; repainting on new data draws none unasked, so one is asked for.
        // Without it the caret came a second late, and a script's keys waited out the floor
        // (plan-circle-survey.gui's prompts, 2026-10-06).
        if window.focused(cx) != focused_at_start {
            // Asked of the repaint loop: gpui drops a refresh asked while it draws.
            repaint::again_in(std::time::Duration::ZERO);
        }
        root
    }
}


/// The link to remember for the next launch: the one in use, unless there is none or it is the
/// storm's in-memory link, which no later launch could open.
fn link_to_remember(target: String, storm: bool) -> Option<String> {
    (!target.is_empty() && !storm).then_some(target)
}

/// The initial window size, from `MP_WINDOW` or the default.
///
/// A malformed value falls back to the default rather than failing to start: a ground station that
/// refuses to open because an environment variable is wrong is worse than one that opens at the
/// wrong size.
fn window_size(requested: Option<&str>, saved: Option<(u32, u32)>) -> (f32, f32) {
    const DEFAULT: (f32, f32) = (1600.0, 1200.0);

    let value = match requested {
        Some(value) => value.to_owned(),
        None => match std::env::var("MP_WINDOW") {
            Ok(value) => value,
            // Whatever the window was last time. A flag beats the variable, the variable beats
            // what was remembered, and what was remembered beats the default.
            Err(_) => {
                // Window sizes are a few thousand pixels; f32 represents every integer up to
                // 16,777,216 exactly, so there is nothing here to lose.
                return saved.map_or(DEFAULT, |(width, height)| {
                    (
                        u16::try_from(width).map_or(DEFAULT.0, f32::from),
                        u16::try_from(height).map_or(DEFAULT.1, f32::from),
                    )
                });
            }
        },
    };
    parse_window_size(&value).unwrap_or_else(|| {
        eprintln!("window size should look like 1600x1200, at least 640x480; using the default");
        DEFAULT
    })
}

/// A remembered or default window size cut to the screen it opens on: no wider than the display's
/// usable area (less the taskbar or dock and the menu bar), and no taller less a title bar, so the
/// window's bottom edge - and every page's last row - is never below the screen. 1600x1200 on a
/// 1440x900 laptop opened 300 pixels past its bottom edge, which no layout inside it can fix (the
/// owner's cut-off reports of 2026-10-04).
fn fit_to_display(wanted: (f32, f32), usable: (f32, f32)) -> (f32, f32) {
    /// A title bar's height and a little: the size gpui opens is the content's.
    const TITLE_BAR: f32 = 40.0;
    /// Never below what `parse_window_size` accepts.
    const SMALLEST: (f32, f32) = (640.0, 480.0);
    (
        wanted.0.min(usable.0).max(SMALLEST.0),
        wanted.1.min(usable.1 - TITLE_BAR).max(SMALLEST.1),
    )
}

/// Parses a `WIDTHxHEIGHT` window size.
///
/// `None` for anything the caller should not act on, including sizes too small to lay out - the
/// sidebar alone is 400 pixels wide, so a 320-pixel window would show nothing but a sliver of it.
fn parse_window_size(value: &str) -> Option<(f32, f32)> {
    let (width, height) = value.split_once(['x', 'X'])?;
    let width: f32 = width.trim().parse().ok()?;
    let height: f32 = height.trim().parse().ok()?;
    (width >= 640.0 && height >= 480.0).then_some((width, height))
}

/// What the command line asked for.
#[derive(Debug)]
struct Arguments {
    /// The link to open, if one was named.
    target: Option<String>,
    /// Whether to read the mission automatically once a vehicle appears.
    read_mission: bool,
    /// The screen to open on, if one was named.
    screen: Option<String>,
    /// The window size, if one was named.
    window: Option<String>,
}

/// Usage, for `--help`.
///
/// Written out rather than generated by an argument-parsing crate: there are four options, and the
/// part worth writing carefully is the list of link forms, because that is the question people
/// actually have.
const USAGE: &str = "planner - Mission Planner, in Rust

USAGE:
    planner [OPTIONS] [LINK]

LINK:
    A serial port, a network address or a log to replay. With no link, the
    application opens disconnected.

    /dev/serial/by-id/usb-ArduPilot_...-if-mavlink   a flight controller, by its stable name
    /dev/ttyACM0                                     the same, by its transient name
    COM3                                             the same, on Windows
    serial:/dev/ttyACM0:57600                        an explicit baud rate
    tcp:127.0.0.1:5760                               SITL, or a TCP telemetry bridge
    tcpin:5760                                       wait for something to connect to us
    udp:14550                                        listen for telemetry, the usual default
    udp:192.168.1.10:14550                           listen on one interface
    flight.tlog                                      replay a telemetry log
    00000042.BIN                                     replay a dataflash log

OPTIONS:
    -h, --help              print this and exit
    -V, --version           print the version and exit
        --read-mission      read the vehicle's mission once it appears
        --screen SCREEN     open on fly, plan, setup, config, params or logs (default: fly)
        --window WIDTHxHEIGHT
                            initial window size (default: 1600x1200)

ENVIRONMENT:
    MP_WINDOW    same as --window
    MP_SCREEN    same as --screen
    MP_PROBE     write control positions to this file, for UI tests
    MP_FACTS     write what the application believes to this file, for UI tests to assert on
    MP_SMOKE     exit 0 once the window has painted, non-zero if it does not
    MP_LOG_DIR   where flights are recorded (default: logs in the MissionPlannerRust directory)
    MP_NO_RECORD do not record this flight
    MP_NO_TILES  do not fetch map imagery
    MP_CONFIG_XML  read and write Mission Planner's config.xml here rather than in its data
                 directory (tests: the application saves it as Mission Planner does)
    MP_STORM     development only: replace the link with a synthetic vehicle sending this many
                 Hz of telemetry, and measure each frame (see crates/mp-gui/src/storm.rs)
    MP_FRAMES    development only: 1 measures each screen's frames in an ordinary run, as
                 facts (see crates/mp-gui/src/frametimes.rs)
    MP_REPAINT   development only: tick:<ms>, data or data:<ms> - when to repaint (see
                 crates/mp-gui/src/repaint.rs)
    MP_PLUGINS   load the WebAssembly plugins from this folder rather than plugins/ beside the
                 executable (tests)
";

/// Parses the command line.
///
/// Hand-rolled, because an argument-parsing crate for four options is a dependency to justify. The
/// rule that matters: anything starting with `-` is an option, never the link. Taking the first
/// argument as the link regardless meant `planner --help` tried to connect to a serial port called
/// "--help", and so did `--read-mission`.
/// Whether the connect-time fetch is due: yes unless every parameter the vehicle has announced
/// is already held. A table with no count yet is not a copy, nor is one short of the count.
const fn auto_fetch_wanted(held: usize, expected: u16) -> bool {
    expected == 0 || held < expected as usize
}

fn parse_arguments(arguments: impl IntoIterator<Item = String>) -> Result<Arguments, String> {
    let mut parsed = Arguments {
        target: None,
        read_mission: false,
        screen: None,
        window: None,
    };
    let mut arguments = arguments.into_iter();

    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--read-mission" => parsed.read_mission = true,
            "--screen" => {
                parsed.screen = Some(arguments.next().ok_or("--screen needs a name")?);
            }
            "--window" => {
                parsed.window = Some(arguments.next().ok_or("--window needs a size")?);
            }
            other if other.starts_with('-') => {
                return Err(format!("unknown option {other}"));
            }
            other if parsed.target.is_some() => {
                return Err(format!("more than one link given: {other}"));
            }
            other => parsed.target = Some(other.to_owned()),
        }
    }
    Ok(parsed)
}

/// Says on stderr what the start's one-shot import copied, when it copied or failed anything.
fn report_import(imported: &mp_settings::migrate::Import) {
    match imported {
        mp_settings::migrate::Import::Imported { from, failed, .. } => {
            eprintln!(
                "planner: imported from {} (left as it was): {}",
                from.display(),
                imported.summary()
            );
            for failure in failed {
                eprintln!("planner: not imported: {failure}");
            }
        }
        mp_settings::migrate::Import::Failed(why) => {
            eprintln!("planner: importing Mission Planner's files: {why}");
        }
        _ => {}
    }
}

fn main() {
    // In a web page a panic says nothing unless it is sent to the console, and `log` goes there
    // too: what gpui_platform's `web_init` does (the browser experiment).
    #[cfg(target_family = "wasm")]
    {
        console_error_panic_hook::set_once();
        // The files the last visit kept, before anything reads one (page_storage.rs).
        mp_os::fs::preload_from_page();
        gpui_web::init_logging();
    }
    // First: what gpui cannot do - open a Metal device, load a font - it says only through `log`.
    stderr_log::install();
    let raw: Vec<String> = std::env::args().skip(1).collect();

    // Help and version before anything else, so they work with no display and answer instantly.
    if raw.iter().any(|a| a == "--help" || a == "-h") {
        println!("{USAGE}");
        return;
    }
    if raw.iter().any(|a| a == "--version" || a == "-V") {
        println!("planner {}", env!("CARGO_PKG_VERSION"));
        return;
    }

    // `/update` and `/updatebeta`: the update alone, no window. `// C#: Program.cs:192-203`
    if let Some(first) = raw.first()
        && (first == "/update" || first == "/updatebeta")
    {
        std::process::exit(help::update_from_command_line(first == "/updatebeta"));
    }

    let arguments = match parse_arguments(raw) {
        Ok(arguments) => arguments,
        Err(problem) => {
            eprintln!("planner: {problem}\n\n{USAGE}");
            std::process::exit(2);
        }
    };

    // This application's own data directory, `MissionPlannerRust` (PLAN.md section 12, D11): on
    // the first start that finds it missing or empty, Mission Planner's files are copied into it
    // once, by name, before anything below reads the directory or writes to it. The C#'s copies
    // are left as they were.
    let imported = mp_settings::migrate::import_at_start();
    report_import(&imported);

    // `Program.CleanupFiles`: a new updater left beside the program by the last update, copied
    // into place. `// C#: Program.cs:614-626`
    help::cleanup_files();

    // `Program`'s `UnhandledException` and `ThreadException` handlers: a panic's report, written
    // for the next start to ask about. `// C#: Program.cs:80, 190, 717`
    crash::install_hook(mp_settings::data_directory());

    // `ThreadPool.QueueUserWorkItem(BGLogMessagesMetaData)`: the log browser's field
    // descriptions, fetched and read in the background. `// C#: MainV2.cs:3299`
    logbrowse::metadata::start();

    // A flag beats its environment variable: the variable is the standing preference and the flag
    // is this run. Passed down as values rather than written back into the environment, which
    // would mean mutating a process-global from one thread and is why this crate forbids unsafe.
    // What was remembered last time, under everything given explicitly. A settings file that is
    // missing, unreadable or full of rubbish yields defaults rather than stopping startup.
    let saved = settings::Settings::load();
    // A size asked for is opened as asked; a remembered or default one is fitted to the screen.
    let asked = arguments.window.is_some() || std::env::var_os("MP_WINDOW").is_some();
    let (width, height) = window_size(arguments.window.as_deref(), saved.window);
    let screen = Screen::initial(arguments.screen.as_deref(), saved.screen.as_deref());
    // A link given on the command line wins; otherwise offer the one last connected to.
    let target = arguments.target.or_else(|| saved.link.clone());
    let read_mission = arguments.read_mission;

    platform::application().run(move |cx: &mut App| {
        // A web page has no system fonts for gpui to find, so the browser build brings its own
        // (the experiment's IBM Plex Sans, OFL), as Zed's web examples do.
        #[cfg(target_family = "wasm")]
        if let Err(err) = cx.text_system().add_fonts(vec![
            std::borrow::Cow::Borrowed(
                include_bytes!("../fonts/IBMPlexSans-Regular.ttf")
                    .as_slice(),
            ),
            std::borrow::Cow::Borrowed(
                include_bytes!("../fonts/IBMPlexSans-SemiBold.ttf")
                    .as_slice(),
            ),
        ]) {
            log::error!("planner: the fonts did not load: {err:#}");
        }
        // 1600x1200. Room for the panel columns and a map worth looking at side by side. Smaller
        // windows work - the panel columns scroll and the map takes what is left, which is what
        // the scrolling was added for - but this is the size the application is laid out for.
        //
        // MP_WINDOW overrides it, as WIDTHxHEIGHT. Trying a size should not need a rebuild, and a
        // screenshot at a particular size should not need a code change that then has to be
        // remembered and undone.
        let usable = cx
            .primary_display()
            .map(|display| display.visible_bounds())
            .filter(|_| !asked);
        let bounds = match usable {
            Some(usable) => {
                let (width, height) = fit_to_display(
                    (width, height),
                    (
                        f32::from(usable.size.width),
                        f32::from(usable.size.height),
                    ),
                );
                Bounds::centered_at(usable.center(), size(px(width), px(height)))
            }
            None => Bounds::centered(None, size(px(width), px(height)), cx),
        };
        let options = WindowOptions {
            window_bounds: Some(WindowBounds::Windowed(bounds)),
            titlebar: Some(TitlebarOptions {
                                title: Some(SharedString::from(PRODUCT_NAME)),
                ..Default::default()
            }),
            ..Default::default()
        };

        let opened = cx.open_window(options, |window, cx| {
            let app = cx.new(|cx| {
                let mut app = MissionPlanner::new(target, read_mission, screen, cx);
                app.persisted.set_imported(imported.summary());
                app
            });
            // The close box is `MainV2_FormClosing`, which saves Mission Planner's config.xml.
            // A kill is not, and saves nothing, in either application.
            let closing = app.downgrade();
            window.on_window_should_close(cx, move |_window, cx| {
                closing.update(cx, |this, _cx| this.form_closing()).ok();
                true
            });
            app
        });
        match opened {
            // `Application.Run(new MainV2())`: the application ends when its main window does,
            // whatever else it has open. `// C#: Program.cs:478`
            Ok(main_window) => {
                let main_window = main_window.window_id();
                cx.on_window_closed(move |cx, closed| {
                    if closed == main_window {
                        cx.quit();
                    }
                })
                .detach();
            }
            Err(err) => {
                eprintln!("could not open a window: {err}");
                // A non-zero exit, because this is the failure a smoke test exists to catch and a
                // process that prints an error and exits 0 is a process CI calls a success.
                std::process::exit(1);
            }
        }
        cx.activate(true);
        if smoke::enabled() {
            smoke::watch();
        }
    });
}

#[cfg(test)]
mod tests {

    /// The window's title and the header say which program this is: the owner's product name,
    /// one word, never Mission Planner's own - the GUI runner tells our window from a real
    /// Mission Planner's by it (tools/gui-test.sh's WINDOW_TITLE).
    #[test]
    fn the_product_name_is_the_owners_and_not_mission_planners() {
        assert_eq!(super::PRODUCT_NAME, "MissionPlannerRust");
        assert!(!super::PRODUCT_NAME.contains(' '));
        let runner = mp_os::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/gui-test.sh"),
        )
        .expect("the GUI runner");
        assert!(
            runner.contains(&format!("WINDOW_TITLE=\"{}\"", super::PRODUCT_NAME)),
            "the runner looks for the window by another title"
        );
    }

    /// The connect-time fetch runs unless the whole announced table is held: nothing held and
    /// no count, a few names read one by one, or a table short of the count all fetch; the full
    /// table does not.
    #[test]
    fn the_connect_time_fetch_runs_unless_the_whole_table_is_held() {
        assert!(super::auto_fetch_wanted(0, 0));
        assert!(super::auto_fetch_wanted(33, 1400));
        assert!(super::auto_fetch_wanted(1399, 1400));
        assert!(!super::auto_fetch_wanted(1400, 1400));
        assert!(!super::auto_fetch_wanted(1401, 1400));
        // No count yet but names held: still fetched - the count comes with the first answer.
        assert!(super::auto_fetch_wanted(5, 0));
    }

    use super::*;

    #[test]
    fn a_window_too_big_for_the_screen_is_cut_to_it() {
        // The default on a 1440x900 laptop whose dock and menu bar leave 1440x875.
        assert_eq!(
            fit_to_display((1600.0, 1200.0), (1440.0, 875.0)),
            (1440.0, 835.0)
        );
        // One that fits is left alone.
        assert_eq!(
            fit_to_display((1280.0, 800.0), (2560.0, 1400.0)),
            (1280.0, 800.0)
        );
        // A tiny screen still gets the smallest window the planner lays out.
        assert_eq!(fit_to_display((1600.0, 1200.0), (600.0, 400.0)), (640.0, 480.0));
    }

    #[test]
    fn a_malformed_window_size_falls_back_rather_than_refusing_to_start() {
        // A ground station that will not open because an environment variable is wrong is worse
        // than one that opens at the wrong size. The parsing is exercised directly because the
        // variable is process-wide and tests run in parallel.
        assert_eq!(parse_window_size("1600x1200"), Some((1600.0, 1200.0)));
        assert_eq!(parse_window_size("1024X768"), Some((1024.0, 768.0)));
        assert_eq!(parse_window_size(" 1280 x 720 "), Some((1280.0, 720.0)));
        assert_eq!(parse_window_size("wide"), None);
        assert_eq!(parse_window_size("1600"), None);
        assert_eq!(parse_window_size("1600x"), None);
        // Too small to lay out: the sidebar alone is 400px wide.
        assert_eq!(parse_window_size("320x240"), None);
        assert_eq!(parse_window_size("-1600x1200"), None);
    }

    fn parse(arguments: &[&str]) -> Arguments {
        parse_arguments(arguments.iter().map(|a| (*a).to_owned())).expect("should parse")
    }

    #[test]
    fn a_bare_device_path_is_the_link_not_an_option() {
        // The form a shell completes, and the reason this parser exists.
        let parsed = parse(&["/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176_x-if-mavlink"]);
        assert_eq!(
            parsed.target.as_deref(),
            Some("/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176_x-if-mavlink")
        );
        assert!(!parsed.read_mission);
    }

    #[test]
    fn an_option_is_never_mistaken_for_the_link() {
        // Taking the first argument as the link regardless meant `planner --read-mission` tried to
        // open a serial port called "--read-mission", and reported that it could not.
        let parsed = parse(&["--read-mission"]);
        assert_eq!(parsed.target, None);
        assert!(parsed.read_mission);
    }

    #[test]
    fn options_and_a_link_can_be_given_in_either_order() {
        let before = parse(&["--read-mission", "tcp:127.0.0.1:5760"]);
        let after = parse(&["tcp:127.0.0.1:5760", "--read-mission"]);
        assert_eq!(before.target, after.target);
        assert_eq!(before.read_mission, after.read_mission);
        assert_eq!(before.target.as_deref(), Some("tcp:127.0.0.1:5760"));
    }

    #[test]
    fn options_that_take_a_value_get_one() {
        let parsed = parse(&["--screen", "plan", "--window", "1280x800", "udp:14550"]);
        assert_eq!(parsed.screen.as_deref(), Some("plan"));
        assert_eq!(parsed.window.as_deref(), Some("1280x800"));
        assert_eq!(parsed.target.as_deref(), Some("udp:14550"));
    }

    #[test]
    fn a_missing_value_is_an_error_rather_than_a_silent_default() {
        // Silently ignoring it would leave the operator looking at the wrong screen wondering why
        // their flag did nothing.
        assert!(parse_arguments(["--screen".to_owned()]).is_err());
        assert!(parse_arguments(["--window".to_owned()]).is_err());
    }

    #[test]
    fn an_unknown_option_is_refused_rather_than_treated_as_a_link() {
        let error = parse_arguments(["--nonsense".to_owned()]).expect_err("should fail");
        assert!(error.contains("--nonsense"), "{error}");
    }

    #[test]
    fn two_links_are_refused_rather_than_one_being_dropped() {
        // Quietly using the first would connect to something the operator did not choose.
        let error = parse_arguments(["udp:14550".to_owned(), "tcp:host:5760".to_owned()])
            .expect_err("should fail");
        assert!(error.contains("tcp:host:5760"), "{error}");
    }

    #[test]
    fn the_usage_text_lists_every_option_it_accepts() {
        // A help text that has drifted from the parser is worse than none.
        for option in [
            "--help",
            "--version",
            "--read-mission",
            "--screen",
            "--window",
        ] {
            assert!(USAGE.contains(option), "usage does not mention {option}");
        }
        // And the link forms people actually type.
        for form in ["/dev/ttyACM0", "COM3", "tcp:", "udp:", ".tlog"] {
            assert!(USAGE.contains(form), "usage does not mention {form}");
        }
        // The environment variables that change what a run does. An undocumented variable is one
        // whose behaviour looks like a bug to whoever inherits a machine that has it set.
        for variable in [
            "MP_WINDOW",
            "MP_SCREEN",
            "MP_PROBE",
            "MP_FACTS",
            "MP_SMOKE",
            "MP_LOG_DIR",
            "MP_NO_RECORD",
            "MP_NO_TILES",
            "MP_CONFIG_XML",
            "MP_STORM",
            "MP_FRAMES",
            "MP_REPAINT",
        ] {
            assert!(
                USAGE.contains(variable),
                "usage does not mention {variable}"
            );
        }
    }

    #[test]
    fn a_flag_beats_its_environment_variable() {
        // The variable is a standing preference; the flag is this run.
        assert_eq!(window_size(Some("1280x800"), None), (1280.0, 800.0));
        assert_eq!(Screen::initial(Some("plan"), None), Screen::Plan);
        assert_eq!(Screen::initial(Some("setup"), None), Screen::Setup);
        assert_eq!(Screen::initial(Some("config"), None), Screen::Config);
        // A name that is not a screen opens on the one the application is for.
        assert_eq!(Screen::initial(Some("nonsense"), None), Screen::Fly);

        // And a flag beats what was remembered, which beats the default.
        assert_eq!(
            window_size(Some("1280x800"), Some((800, 600))),
            (1280.0, 800.0)
        );
        assert_eq!(window_size(None, Some((800, 600))), (800.0, 600.0));
        assert_eq!(Screen::initial(None, Some("setup")), Screen::Setup);
        assert_eq!(Screen::initial(Some("plan"), Some("setup")), Screen::Plan);
    }

    /// The header names this port's author, and Mission Planner's when hovered (the owner,
    /// 2026-10-04); the old sub-title is gone.
    #[test]
    fn the_byline_names_the_author_and_thanks_mission_planners() {
        assert_eq!(BYLINE, "by David Buzz");
        assert_eq!(BYLINE_TIP, ".. and with thanks to the OG Michael Oborne");
        let source = include_str!("main.rs");
        assert!(!source.contains(concat!("Rust port", " - gpui")));
    }

    #[test]
    fn the_screens_have_distinct_labels_and_ids() {
        // The ids address controls a test script clicks; two screens sharing one would make a
        // click silently land on the wrong tab.
        let mut ids: Vec<&str> = Screen::ALL.iter().map(|s| s.id()).collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count);

        let mut labels: Vec<&str> = Screen::ALL.iter().map(|s| s.label()).collect();
        labels.sort_unstable();
        labels.dedup();
        assert_eq!(labels.len(), count);
    }

    #[test]
    fn config_is_beside_setup_as_mainv2_has_them() {
        // MainV2's menu: SETUP, then CONFIG (MainV2.Designer.cs:150, 158).
        let setup = Screen::ALL
            .iter()
            .position(|screen| *screen == Screen::Setup);
        let config = Screen::ALL
            .iter()
            .position(|screen| *screen == Screen::Config);
        assert_eq!(setup.map(|at| at + 1), config);
    }

    #[test]
    fn a_storm_repaints_at_display_rate_and_is_never_remembered() {
        // The storm's frames are what is measured, so there have to be display-rate frames of
        // them; a benchmark still repaints flat out; a normal run on new data (repaint.rs).
        if std::env::var("MP_REPAINT").is_err() {
            let chosen = |bench: bool, storm: bool| {
                repaint::Policy::chosen(
                    bench.then_some(REFRESH_BENCH),
                    storm.then_some(storm::REFRESH),
                )
            };
            assert_eq!(
                chosen(false, false),
                repaint::Policy::Data {
                    floor: repaint::FLOOR
                }
            );
            assert_eq!(chosen(false, true), repaint::Policy::Tick(storm::REFRESH));
            assert!(storm::REFRESH < repaint::IN_FLIGHT);
            assert_eq!(chosen(true, true), repaint::Policy::Tick(REFRESH_BENCH));
        }
        // Remembering the storm's in-memory link would have the next launch try to open it.
        assert_eq!(
            link_to_remember("tcp:127.0.0.1:5760".to_owned(), false).as_deref(),
            Some("tcp:127.0.0.1:5760")
        );
        assert_eq!(link_to_remember("loopback:b".to_owned(), true), None);
        assert_eq!(link_to_remember(String::new(), false), None);
    }

    #[test]
    fn help_is_the_last_tab_after_params_logs_and_plugins() {
        // The owner, 2026-10-04: params, logs, experimental, plugins, then help.
        assert_eq!(
            Screen::ALL[Screen::ALL.len() - 5..],
            [
                Screen::Params,
                Screen::Logs,
                Screen::Experimental,
                Screen::Plugins,
                Screen::Help
            ]
        );
        assert_eq!(Screen::initial(Some("plugins"), None), Screen::Plugins);
        assert_eq!(
            Screen::initial(Some("experimental"), None),
            Screen::Experimental
        );
    }

    #[test]
    fn flying_is_the_screen_it_opens_on() {
        // Not a preference: it is what the application is for, and an operator who connects to a
        // vehicle in flight should not have to find the right tab first.
        assert_eq!(Screen::ALL.first().copied(), Some(Screen::Fly));
    }
}
