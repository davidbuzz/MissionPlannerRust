//! `mpr-gui` - the graphical front end.
//!
//! Built on gpui, Zed's GPU-accelerated UI framework (DELIVERABLES.md D6/D7).
//!
//! Run with a link URL to connect, or with no arguments for a disconnected shell:
//!   mpr-gui tcp:127.0.0.1:5760

#![allow(clippy::print_stderr)]

mod facts;
mod fly;
mod hud;
mod joystick;
mod logbrowse;
mod mapview;
mod params;
mod plan;
mod platform;
mod probe;
mod settings;
mod setup;
mod smoke;
mod telemetry;
mod textfield;
mod tuning;
mod ui;

use std::time::Duration;

use gpui::{
    App, Bounds, Context, MouseButton, SharedString, TitlebarOptions, Window, WindowBounds,
    WindowOptions, div, prelude::*, px, rgb, size,
};
use mapview::MapViewport;
use mp_tiles::cache::TileCache;
use mp_tiles::store::TileStore;
use plan::Plan;
use telemetry::{Telemetry, TelemetryView};
use ui::{action, theme};

/// How often to repaint. 10 Hz is plenty for numeric readouts and keeps an idle GCS cheap; the
/// map and HUD (D7-D9) will drive their own higher-rate rendering.
const REFRESH: Duration = Duration::from_millis(100);

/// The mission file name used when nothing has been typed.
const DEFAULT_PLAN_FILE: &str = "mission.waypoints";

/// The name a parameter backup gets if the operator does not choose one.
const DEFAULT_PARAM_FILE: &str = "vehicle.param";

/// The altitude a waypoint gets when there is no previous one to copy.
///
/// Mission Planner takes it from `TXT_DefaultAlt`, a box on the planning screen
/// (`FlightPlanner.cs:6873`). There is no such box here yet, so a new waypoint copies the one
/// before it and falls back to this - which is the behaviour an operator gets from that box
/// anyway, since they set it once and every waypoint after inherits it.
const DEFAULT_WAYPOINT_ALTITUDE: f64 = 50.0;

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
    /// Initial setup and calibration.
    Setup,
    /// The vehicle's parameters.
    Params,
    /// Reviewing a dataflash log.
    ///
    /// Mission Planner opens `Log/LogBrowse.cs` as a separate window from a button on the flight
    /// screen (`BUT_logbrowse_Click`, `GCSViews/FlightData.cs:1380`). A single-window application
    /// makes it a tab; what it holds is the same - the list of fields the log declares, and a
    /// chart of the chosen one.
    Logs,
}

impl Screen {
    /// The tabs, in order.
    const ALL: [Self; 5] = [Self::Fly, Self::Plan, Self::Setup, Self::Params, Self::Logs];

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
            "params" => Self::Params,
            "logs" => Self::Logs,
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
            Self::Params => "params",
            Self::Logs => "logs",
        }
    }

    const fn id(self) -> &'static str {
        match self {
            Self::Fly => "tab-fly",
            Self::Plan => "tab-plan",
            Self::Setup => "tab-setup",
            Self::Params => "tab-params",
            Self::Logs => "tab-logs",
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
    /// The `.param` file name to save to, load from or compare against.
    param_file_name: textfield::TextField,
    /// Focus for that field.
    param_file_focus: gpui::FocusHandle,
    /// Joystick state: the device, the mapping and the failsafe.
    sticks: joystick::Sticks,
    /// The live tuning graph.
    tuning: tuning::Tuning,
    /// The log being reviewed.
    log_browse: logbrowse::LogBrowse,
    /// The log file name to open.
    log_name: textfield::TextField,
    /// Focus for that field.
    log_name_focus: gpui::FocusHandle,
    /// What has been typed into the log field search.
    log_search: textfield::TextField,
    /// The result of the last comparison against a file, newest first.
    ///
    /// Held rather than applied. A comparison is something an operator reads before deciding, and
    /// the decision is a second, deliberate press - loading a tune because it was compared would
    /// be the worst possible reading of "show me what this would change".
    param_differences: Vec<mp_link::param_file::Difference>,
    /// Throttle a motor test uses, as a percentage.
    motor_throttle: f32,
    /// Whether a radio calibration is recording stick limits.
    capturing_radio: bool,
    /// The limits recorded so far.
    radio_range: mp_vehicle::RcRange,
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
    last_force_arm: Option<std::time::Instant>,
    /// While a forced arm is in progress, when to stop re-sending it.
    ///
    /// The parameter write that disables the checks takes effect asynchronously, so one arm
    /// command sent straight after it can arrive too early. Re-sending for a couple of seconds
    /// costs nothing and removes the race.
    forcing_arm_until: Option<std::time::Instant>,
    /// Scroll position of the flight screen's panel column, so an indicator can be drawn for it.
    fly_scroll: gpui::ScrollHandle,
    /// Scroll position of the plan screen's panel column.
    plan_scroll: gpui::ScrollHandle,
}

impl MissionPlanner {
    fn new(
        target: Option<String>,
        read_mission: bool,
        screen: Screen,
        cx: &mut Context<Self>,
    ) -> Self {
        let telemetry = match target {
            Some(url) => Telemetry::connect(&url),
            None => Telemetry::idle(),
        };

        // Repaint on a timer. The link thread owns the data and publishes snapshots; the UI only
        // ever reads one, so this cannot block on I/O.
        cx.spawn(async move |this, cx| {
            loop {
                let interval = if std::env::var("MP_BENCH").is_ok() {
                    REFRESH_BENCH
                } else {
                    REFRESH
                };
                cx.background_executor().timer(interval).await;
                if this.update(cx, |_, cx| cx.notify()).is_err() {
                    break;
                }
            }
        })
        .detach();

        // No timer for the sticks. They are read and sent on threads of their own in `mp_input`
        // (D15's 5 ms from stick to wire is not reachable from a timer on this executor, and a
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
        if std::env::var("MP_NO_TILES").is_err() {
            let cache = TileCache::new(TileCache::default_root());
            // The environment wins over the remembered choice, so a screenshot or a test can
            // pin a provider without disturbing what the operator picked.
            let source = std::env::var("MP_TILE_SOURCE")
                .ok()
                .or_else(|| settings::Settings::load().tile_source)
                .and_then(|id| mp_tiles::source::source_by_id(&id))
                .unwrap_or(&mp_tiles::source::OPENSTREETMAP);
            let store = if std::env::var("MP_OFFLINE").is_ok() {
                TileStore::offline(source, cache)
            } else {
                TileStore::new(source, cache)
            };
            map.set_tiles(std::sync::Arc::new(store));
        }

        Self {
            telemetry,
            map: std::rc::Rc::new(std::cell::RefCell::new(map)),
            auto_read_mission: read_mission,
            mission_requested: false,
            screen,
            plan: Plan::default(),
            adopt_vehicle_mission: false,
            adopt_vehicle_fence: false,
            adopt_vehicle_rally: false,
            file_status: None,
            dragging_waypoint: None,
            altitude_frame: plan::AltitudeFrame::from_key(
                settings::Settings::load()
                    .altitude_frame
                    .as_deref()
                    .unwrap_or_default(),
            ),
            map_press: None,
            settings: settings::Settings::load(),
            plan_name: {
                let mut field = textfield::TextField::new("mission.waypoints");
                field.set(DEFAULT_PLAN_FILE);
                field
            },
            plan_name_focus: cx.focus_handle(),
            param_search: textfield::TextField::new("search parameters"),
            param_search_focus: cx.focus_handle(),
            selected_param_group: None,
            selected_param: None,
            param_file_name: {
                let mut field = textfield::TextField::new(DEFAULT_PARAM_FILE);
                field.set(DEFAULT_PARAM_FILE);
                field
            },
            param_file_focus: cx.focus_handle(),
            sticks: joystick::Sticks::new(),
            tuning: tuning::Tuning::new(),
            log_browse: logbrowse::LogBrowse::new(),
            log_name: textfield::TextField::new("a .BIN or .log in the plan directory"),
            log_name_focus: cx.focus_handle(),
            log_search: textfield::TextField::new("filter fields"),
            param_differences: Vec::new(),
            motor_throttle: 5.0,
            capturing_radio: false,
            radio_range: mp_vehicle::RcRange::new(),
            disabled_arming_checks: false,
            forcing_arm_until: None,
            last_force_arm: None,
            fly_scroll: gpui::ScrollHandle::new(),
            plan_scroll: gpui::ScrollHandle::new(),
        }
    }

    /// The directory missions are read from and written to.
    ///
    /// A file dialog needs a platform integration gpui does not give us for free. A typed name in
    /// a known directory is the next best thing, and better than the fixed path this had before -
    /// which meant a second mission silently overwrote the first.
    fn plan_directory() -> std::path::PathBuf {
        std::env::var("MP_PLAN_DIR").map_or_else(
            |_| std::env::current_dir().unwrap_or_else(|_| std::path::PathBuf::from(".")),
            std::path::PathBuf::from,
        )
    }

    /// Where a named mission lives.
    ///
    /// The name is treated as a name, not a path: anything with a separator in it is reduced to
    /// its last component. A typed "../../etc/passwd" writing outside the mission directory would
    /// be a surprise at best.
    fn plan_path_named(name: &str) -> std::path::PathBuf {
        let trimmed = name.trim();
        let leaf = trimmed
            .rsplit(['/', '\\'])
            .next()
            .filter(|part| !part.is_empty() && *part != "." && *part != "..")
            .unwrap_or(DEFAULT_PLAN_FILE);
        // A missing extension is added rather than refused, because the operator meant a mission
        // file and typing the suffix is not the interesting part.
        let leaf = if leaf.contains('.') {
            leaf.to_owned()
        } else {
            format!("{leaf}.waypoints")
        };
        Self::plan_directory().join(leaf)
    }

    /// The path the currently typed name refers to.
    fn plan_path(&self) -> std::path::PathBuf {
        Self::plan_path_named(self.plan_name.value())
    }

    /// Writes the plan in QGC WPL 110 format, the one every ground station reads.
    fn save_plan(&mut self) {
        let path = self.plan_path();
        let text = mp_mission::write_waypoints(self.plan.items());
        self.file_status = match std::fs::write(&path, text) {
            Ok(()) => Some(format!(
                "saved {} items to {}",
                self.plan.items().len(),
                path.display()
            )),
            Err(err) => Some(format!("could not save to {}: {err}", path.display())),
        };
    }

    /// Reads a plan from the same location.
    fn load_plan(&mut self) {
        let path = self.plan_path();
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(err) => {
                self.file_status = Some(format!("could not read {}: {err}", path.display()));
                return;
            }
        };
        match mp_mission::read_waypoints(&text) {
            Ok(items) => {
                let count = items.len();
                let name = path.file_name().map_or_else(
                    || path.display().to_string(),
                    |n| n.to_string_lossy().into_owned(),
                );
                self.plan.adopt_from_file(name, items);
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
    fn params_as_file(&self) -> mp_link::param_file::ParamFile {
        let view = self.telemetry.view();
        mp_link::param_file::ParamFile::from_values(
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
            Ok(()) => Some(format!(
                "saved {} of {held} parameters to {}",
                file.len(),
                path.display()
            )),
            Err(err) => Some(format!("could not save to {}: {err}", path.display())),
        };
    }

    /// Compares a file against the vehicle, without changing anything.
    fn compare_params(&mut self) {
        let path = self.param_path();
        let proposed = match mp_link::param_file::ParamFile::load(&path) {
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
    fn apply_params(&mut self) {
        if self.param_differences.is_empty() {
            self.file_status = Some("compare a file first - there is nothing to apply".to_owned());
            return;
        }
        let mut written = 0usize;
        let mut skipped = 0usize;
        for difference in &self.param_differences {
            match difference.kind {
                mp_link::param_file::Change::Changed { to, .. } => {
                    #[allow(clippy::cast_possible_truncation)] // parameters are f32 on the wire
                    self.telemetry.set_parameter(&difference.name, to as f32);
                    written += 1;
                }
                mp_link::param_file::Change::Added { .. }
                | mp_link::param_file::Change::Missing { .. } => skipped += 1,
            }
        }
        // The comparison is now stale - it describes a vehicle that no longer exists. Cleared
        // rather than left on screen, because a list of differences beside an "apply" button that
        // has already been pressed invites pressing it again.
        self.param_differences.clear();
        self.file_status = Some(if skipped == 0 {
            format!("wrote {written} parameters - refresh to confirm")
        } else {
            format!(
                "wrote {written} parameters, skipped {skipped} this firmware does not have - refresh to confirm"
            )
        });
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
        let store = if std::env::var("MP_OFFLINE").is_ok() {
            TileStore::offline(source, cache)
        } else {
            TileStore::new(source, cache)
        };
        self.map.borrow_mut().set_tiles(std::sync::Arc::new(store));
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
    fn set_altitude_frame(&mut self, frame: plan::AltitudeFrame) {
        self.altitude_frame = frame;
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
    }

    /// Commands a guided move to a position, holding the current height.
    ///
    /// Keeping the aircraft's own altitude is the only safe default: a fixed one would descend a
    /// vehicle that is above it, and a click meant to redirect a flight is not a click meant to
    /// change height. On the ground it does nothing beyond what the vehicle's own checks allow -
    /// a disarmed vehicle refuses, and says so in the message pane.
    fn fly_here(&mut self, position: mp_units::LatLon) {
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

        self.telemetry.force_arm();
        self.last_force_arm = Some(std::time::Instant::now());
        self.disabled_arming_checks = true;
        self.forcing_arm_until = Some(std::time::Instant::now() + GIVE_UP_AFTER);
        self.file_status =
            Some("arming checks disabled (ARMING_SKIPCHK=-1); forcing arm".to_owned());
    }

    /// Changes one parameter by a step, clamped to its documented range.
    ///
    /// Clamped rather than refused: the operator asked to move it, and stopping at the limit is
    /// what they meant. ArduPilot accepts an out-of-range write and then behaves oddly, so the
    /// editor is the last place to catch it.
    fn nudge_parameter(&mut self, name: &str, delta: f64) {
        let view = self.telemetry.view();
        let Some((_, current)) = view.parameters.iter().find(|(held, _)| held == name) else {
            return;
        };
        let mut next = current + delta;
        if let Some(meta) = mp_vehicle::param_meta::lookup(name)
            && let Some((low, high)) = meta.range
        {
            next = next.clamp(low, high);
        }
        #[allow(clippy::cast_possible_truncation)] // parameters are f32 on the wire
        self.telemetry.set_parameter(name, next as f32);
        self.file_status = Some(format!("{name} = {next}"));
    }

    /// Starts recording the radio's stick limits from scratch.
    ///
    /// From scratch, not continuing: a calibration that kept limits from a previous sweep would
    /// carry over a stick position the operator has since changed, and nothing on screen would
    /// say so.
    fn begin_radio_capture(&mut self) {
        self.radio_range = mp_vehicle::RcRange::new();
        self.capturing_radio = true;
        self.file_status = Some("recording radio limits - sweep every control".to_owned());
    }

    /// Writes the recorded limits to the vehicle as RCn_MIN and RCn_MAX.
    ///
    /// Only channels that actually moved. A channel left alone has a minimum equal to its
    /// maximum, and writing that is a stick with no travel or a switch with one position.
    fn save_radio_limits(&mut self) {
        let mut written = 0;
        for number in 1..=mp_vehicle::rc::CHANNELS {
            let Some((minimum, maximum)) = self.radio_range.channel(number) else {
                continue;
            };
            self.telemetry
                .set_parameter(&format!("RC{number}_MIN"), f32::from(minimum));
            self.telemetry
                .set_parameter(&format!("RC{number}_MAX"), f32::from(maximum));
            written += 1;
        }
        self.file_status = Some(format!(
            "wrote limits for {written} channels; reboot for them to take effect"
        ));
    }

    /// Pushes the plan to the map after an edit.
    ///
    /// The render pass does this too, but only on the next frame; doing it at the edit means the
    /// map never shows a waypoint the operator has just deleted.
    fn sync_map_mission(&self) {
        self.map.borrow_mut().set_mission(self.plan.items());
    }

    /// Pushes the survey area to the map after an edit.
    fn sync_map_polygon(&self) {
        self.map.borrow_mut().set_polygon(self.plan.polygon());
    }

    /// Pushes the geofence to the map after an edit.
    fn sync_map_fence(&self) {
        self.map.borrow_mut().set_fence(self.plan.fence());
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
        let target = self.telemetry.view().target;
        if !target.is_empty() {
            self.settings.link = Some(target);
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
                        this.screen = screen;
                        // Remembered here rather than at exit: gpui gives no reliable hook for a
                        // window closing, and a ground station is as likely to be killed as
                        // closed.
                        this.remember();
                        cx.notify();
                    })),
            );
        }
        strip
    }
}

impl MissionPlanner {
    /// The left column on the flight screen.
    ///
    /// The HUD is pinned and everything below it scrolls. The mode list is as long as the
    /// airframe's - twenty-seven entries on a copter - so a fixed column cut off the panels below
    /// it, and the link panel was unreachable at any window size anyone uses. Scrolling the whole
    /// column instead was worse: the mode list appears only once a vehicle is heard from, and the
    /// content growing under the scroll container dragged the view down, so the application
    /// started with its primary flight display already off the top of the screen.
    fn fly_sidebar(&self, view: &TelemetryView, cx: &mut Context<Self>) -> impl IntoElement {
        probe::measured("fly-column", div())
            .flex()
            .flex_col()
            .flex_shrink_0()
            .min_h(px(0.0))
            .gap_2()
            .w(px(400.0))
            .child(fly::hud_panel(view))
            .child(
                // The scrolling column and its indicator share a positioned parent, so the
                // indicator can sit over the column's right edge without taking width from it.
                div()
                    .relative()
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(0.0))
                    .child(
                        probe::measured("fly-sidebar", div())
                            .id("fly-sidebar")
                            .flex()
                            .flex_col()
                            .flex_1()
                            .min_h(px(0.0))
                            .gap_2()
                            .pr_2()
                            .overflow_y_scroll()
                            .track_scroll(&self.fly_scroll)
                            // The tuning graph first, because that is where the C# puts it: its
                            // chart lives in `splitContainer1.Panel1`, which sits above and is
                            // collapsed until `CB_tuning` uncollapses it, pushing the rest down.
                            // It is also the only placement that is any use - at the bottom of a
                            // column that already scrolls, a plot nobody can see without
                            // scrolling to it is a plot nobody watches.
                            .child(tuning::panel_for(&self.tuning, cx))
                            .child(fly::actions_panel(view, self.disabled_arming_checks, cx))
                            .child(fly::prearm_panel(view))
                            .child(fly::vehicle_panel(view))
                            .child(fly::health_panel(view)),
                    )
                    .children(ui::scroll_indicator(&self.fly_scroll)),
            )
    }

    /// The left column on the plan screen.
    fn plan_sidebar(
        &self,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> impl IntoElement {
        // Copied out of the plan before building the elements: the listeners the panels install
        // take `&mut self`, so holding a borrow of `self.plan` across them would not compile.
        let items = self.plan.items().to_vec();
        let origin = self.plan.origin().clone();
        let selected = self.plan.selected();
        let survey_error = self.plan.survey_error().map(ToOwned::to_owned);
        let fence_error = self.plan.fence_error().map(ToOwned::to_owned);
        let rally_error = self.plan.rally_error().map(ToOwned::to_owned);
        let draw = plan::DrawState {
            mode: self.plan.draw_mode(),
            area_vertices: self.plan.polygon().len(),
            survey: self.plan.survey_options(),
            survey_error: survey_error.as_deref(),
            fence_vertices: self.plan.fence().len(),
            fence_error: fence_error.as_deref(),
            rally_points: self.plan.rally().len(),
            rally_error: rally_error.as_deref(),
        };

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
                    .child(plan::actions_panel(
                        &items,
                        &origin,
                        view,
                        &plan::NameField {
                            field: &self.plan_name,
                            focus: &self.plan_name_focus,
                            focused: self.plan_name_focus.is_focused(window),
                        },
                        self.altitude_frame,
                        self.tile_source_id(),
                        cx,
                    ))
                    .child(plan::draw_panel(&draw, view, cx))
                    .child(plan::items_panel(&items, selected, cx))
                    .child(plan::editor_panel(&items, selected, cx))
                    .child(plan::checks_panel(&items, view)),
            )
            .children(ui::scroll_indicator(&self.plan_scroll))
    }

    /// The setup screen, which is one column and no map.
    fn setup_body(&self, view: &TelemetryView, cx: &mut Context<Self>) -> impl IntoElement {
        let calibration = self.telemetry.accel_calibration();
        let compass = self.telemetry.compass_calibration();
        let listings = self.telemetry.log_listings();
        let log_progress = self.telemetry.log_progress();
        div()
            .flex()
            .flex_col()
            .gap_2()
            .p_2()
            .w(px(760.0))
            .child(setup::identity_panel(view))
            // Second, above the calibrations. The reason somebody opens this screen mid-session
            // is usually that the vehicle is behaving oddly, and this is the panel that says why -
            // putting it below six calibration wizards buries the answer under the treatments.
            .child(fly::estimator_panel(view))
            .child(setup::accelerometer_panel(calibration, view, cx))
            .child(setup::compass_panel(&compass, view, cx))
            .child(setup::radio_panel(
                view,
                &self.radio_range,
                self.capturing_radio,
                cx,
            ))
            .child(setup::motor_panel(view, self.motor_throttle, cx))
            .child(joystick::panel_for(view, &self.sticks, cx))
            .child(setup::logs_panel(&listings, log_progress, view, cx))
            .child(setup::calibration_panel(view, cx))
    }

    /// The map, with the handlers that make it a map rather than a picture.
    fn map_pane(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let following = self.map.borrow().is_following();
        let attribution = self.map.borrow().attribution();
        let planning = self.screen == Screen::Plan;

        div()
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
                        cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                            let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                            let grabbed = planning
                                .then(|| this.map.borrow().waypoint_at(x, y))
                                .flatten();
                            this.map_press = Some((x, y));
                            match grabbed {
                                Some(seq) => {
                                    this.dragging_waypoint = Some(seq);
                                    this.plan.select(Some(seq));
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
                            if event.pressed_button != Some(MouseButton::Left) {
                                return;
                            }
                            let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
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
                        cx.listener(move |this, event: &gpui::MouseUpEvent, _window, cx| {
                            let grabbed = this.dragging_waypoint.take();
                            let press = this.map_press.take();
                            this.map.borrow_mut().end_drag();

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
                            let altitude = this
                                .plan
                                .items()
                                .last()
                                .map_or(DEFAULT_WAYPOINT_ALTITUDE, |last| last.z);
                            this.plan
                                .add_waypoint_in(position, altitude, this.altitude_frame);
                            this.sync_map_mission();
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
                    // Right-click adds a waypoint while planning. Not left-click: left is pan,
                    // and a gesture that both moves the map and drops a waypoint would put one
                    // down on every failed drag.
                    // Right-click adds a waypoint while planning, and commands a guided move
                    // while flying. Not left-click in either case: left is pan, and a gesture
                    // that both moves the map and commits something would fire on every failed
                    // drag - which while flying means the aircraft moves.
                    .on_mouse_up(
                        MouseButton::Right,
                        cx.listener(move |this, event: &gpui::MouseUpEvent, _window, cx| {
                            let Some(position) = this.map.borrow().position_at(
                                f32::from(event.position.x),
                                f32::from(event.position.y),
                            ) else {
                                return;
                            };
                            if planning {
                                // Freeze the view before the edit, not after: the automatic fit
                                // frames everything it knows about, so adding a waypoint changes
                                // what it has to frame and the map jumps - putting the next click
                                // somewhere the operator did not aim at.
                                this.map.borrow_mut().freeze_view();
                                match this.plan.draw_mode() {
                                    plan::DrawMode::Waypoints => {
                                        this.plan.add_waypoint(position, plan::DEFAULT_ALTITUDE);
                                        this.sync_map_mission();
                                    }
                                    plan::DrawMode::Area => {
                                        this.plan.add_area_vertex(position);
                                        this.sync_map_polygon();
                                    }
                                    plan::DrawMode::Fence => {
                                        this.plan.add_fence_vertex(position);
                                        this.sync_map_fence();
                                    }
                                    plan::DrawMode::Rally => {
                                        this.plan.add_rally_point(position);
                                        this.sync_map_rally();
                                    }
                                }
                            } else {
                                this.fly_here(position);
                            }
                            cx.notify();
                        }),
                    )
                    .child(mapview::map_element(self.map.clone()))
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
                    .child(
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
                                !following,
                                {
                                    let map = self.map.clone();
                                    move |_event: &(), window: &mut Window, _cx: &mut gpui::App| {
                                        map.borrow_mut().follow_vehicle();
                                        window.refresh();
                                    }
                                },
                            )),
                    ),
            )
            .child(self.map_status())
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
        // Counted here because this is the one place that only runs when a frame is actually
        // painted. See smoke.rs: the failure being looked for is a backend that will not
        // initialise, and every earlier signal - a window handle, a running executor - survives
        // that.
        smoke::painted();
        let view = self.telemetry.view();

        // The sticks send from their own thread; this keeps them addressed to the vehicle being
        // flown and notices a device that has gone. Once a frame, whether or not anything shows.
        self.sticks.tick(self.telemetry.send_handle());

        // Facts a UI test can assert on. Recorded from render because that is where every one of
        // them is already in hand, and published at the end of the frame so a reader never sees
        // half a set. Costs nothing unless MP_FACTS names a file.
        if facts::enabled() {
            facts::record("screen", self.screen.label());
            facts::record("mission.items", self.plan.items().len());
            facts::record("mission.origin", self.plan.origin().label());
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
                facts::record("map.tiles.drawn", drawn);
                facts::record("map.tiles.approximate", approximate);
                facts::record("map.tiles.missing", missing);
                facts::record("map.tiles.disk", stats.disk_hits);
                facts::record("map.tiles.fetched", stats.fetched);
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
            facts::record("vehicle.connected", view.connected);
            facts::record("vehicle.count", view.vehicle_count);
            facts::record("link.frames", view.frames);
            facts::record("params.held", view.parameters.len());
            facts::record("params.expected", view.parameters_expected);
            facts::record("tuning.visible", self.tuning.is_visible());
            facts::record("tuning.series", self.tuning.series().len());
            facts::record("log.open", self.log_browse.is_open());
            facts::record("log.fields", self.log_browse.fields().len());
            facts::record("log.plotted", self.log_browse.plotted().len());
            // The two axes. A test that right-clicks a field can prove it went on the right
            // rather than merely on the plot, and that the left side split by unit.
            facts::record("log.plotted.right", self.log_browse.right_count());
            facts::record("log.axes.left", self.log_browse.left_units().len());
            facts::record("sticks.enabled", self.sticks.is_enabled());
            // Frames the link accepted and the measured stick-to-link latency, so a test with a
            // device attached can prove frames go out and how fast.
            facts::record("sticks.sent", self.sticks.sent());
            facts::record(
                "sticks.p99_us",
                self.sticks.latency().map_or(0, |(_, p99)| p99.as_micros()),
            );
            facts::record("recording", self.telemetry.recording().is_some());
            facts::record("status", self.file_status.as_deref().unwrap_or(""));
            facts::publish();
        }

        // The tuning graph is fed here because this is where a fresh snapshot arrives. It samples
        // only when the snapshot is new - a repeated sample draws a horizontal line that looks
        // exactly like a steady measurement, and on a tuning graph "steady" and "nothing arriving"
        // lead to opposite conclusions.
        self.tuning.sample(&view);

        // Feed the map from the same snapshot the panels read, so the two can never disagree
        // about where the vehicle is.
        if let Some(state) = view.state.as_ref() {
            let mut map = self.map.borrow_mut();
            if let Some(position) = state.position {
                map.observe(position, state.heading);
            }
            if let Some(home) = state.home {
                map.set_home(home);
            }
        }

        // A completed download replaces the plan only if the operator asked for one. Otherwise it
        // just goes to the map, so a read started for display cannot overwrite an edit.
        if !view.mission.is_empty() && self.adopt_vehicle_mission {
            self.adopt_vehicle_mission = false;
            self.plan.adopt_from_vehicle(view.mission.clone());
            self.file_status = Some(format!(
                "read {} items from the vehicle",
                view.mission.len()
            ));
        }

        // The map shows the plan being edited when there is one, and what the vehicle holds
        // otherwise. Showing the vehicle's mission while the operator draws a different one is
        // how people fly the mission they thought they had replaced.
        if self.plan.is_empty() {
            if !view.mission.is_empty() {
                self.map.borrow_mut().set_mission(&view.mission);
            }
        } else {
            self.map.borrow_mut().set_mission(self.plan.items());
        }

        // Other aircraft. Read every frame because the link forgets stale ones on read, and a
        // display that only updated on an event would keep a symbol after its aircraft had gone.
        {
            let now = std::time::Instant::now();
            let traffic: Vec<(mp_units::LatLon, bool)> = self
                .telemetry
                .traffic()
                .into_iter()
                .map(|aircraft| (aircraft.position, aircraft.is_stale(now)))
                .collect();
            self.map.borrow_mut().set_traffic(&traffic);
        }

        // Keep a log download moving, and write it out when it finishes. Driven from the render
        // pass because that is the only thing ticking; the link cannot write files and should not
        // decide where they go.
        if self.telemetry.log_progress().is_some() {
            if let Some((id, bytes)) = self.telemetry.finished_log() {
                self.telemetry.clear_log_download();
                let path = Self::plan_directory().join(format!("log_{id}.bin"));
                self.file_status = Some(match std::fs::write(&path, &bytes) {
                    Ok(()) => format!("wrote {} ({} bytes)", path.display(), bytes.len()),
                    Err(err) => format!("could not write {}: {err}", path.display()),
                });
            } else {
                self.telemetry.nudge_log_download();
            }
        }

        // Fold live channel values into the recorded limits while a radio calibration runs.
        // Done here because this is the only place that sees every snapshot; sampling on a timer
        // would miss the extremes, which are exactly what is being recorded.
        if self.capturing_radio
            && let Some(state) = view.state.as_ref()
        {
            self.radio_range.observe(&state.rc);
        }

        // Keep re-sending a forced arm until it takes, or until we give up. The parameter write
        // that disabled the checks may not have been applied when the first command arrived.
        if let Some(deadline) = self.forcing_arm_until {
            let armed = view.state.as_ref().is_some_and(|state| state.armed);
            if armed {
                self.forcing_arm_until = None;
                self.file_status = Some("armed with arming checks disabled".to_owned());
            } else if std::time::Instant::now() >= deadline {
                self.forcing_arm_until = None;
                self.file_status = Some(
                    "forced arm gave up; the vehicle is still refusing - see the messages"
                        .to_owned(),
                );
            } else {
                // Once per heartbeat interval, so each attempt is judged against a state that
                // could have changed since the last one.
                const BETWEEN_ATTEMPTS: std::time::Duration =
                    std::time::Duration::from_millis(1000);
                let now = std::time::Instant::now();
                if self
                    .last_force_arm
                    .is_none_or(|last| now.duration_since(last) >= BETWEEN_ATTEMPTS)
                {
                    self.last_force_arm = Some(now);
                    self.telemetry.force_arm();
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

        let (status, status_colour) = if let Some(err) = self.telemetry.error() {
            (format!("link failed: {err}"), theme::ALERT)
        } else if !view.connected && view.target.is_empty() {
            (
                "no link - start with a url, e.g. tcp:127.0.0.1:5760".to_owned(),
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

        let body = match self.screen {
            Screen::Fly => probe::measured("body", div())
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .gap_2()
                .p_2()
                .child(self.fly_sidebar(&view, cx))
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .flex_1()
                        .min_w(px(0.0))
                        .min_h(px(0.0))
                        .gap_2()
                        .child(self.map_pane(cx))
                        .child(div().flex_shrink_0().child(fly::messages_panel(&view))),
                )
                .into_any_element(),
            Screen::Plan => div()
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .gap_2()
                .p_2()
                .child(self.plan_sidebar(&view, window, cx))
                .child(self.map_pane(cx))
                .into_any_element(),
            Screen::Params => {
                let parameters = params::collect(&view);
                let group = self.selected_param_group.clone();
                let selected = self.selected_param.clone();
                div()
                    .id("params-body")
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(0.0))
                    .gap_2()
                    .p_2()
                    .overflow_y_scroll()
                    .child(params::browser_panel(
                        &view,
                        &parameters,
                        group.as_deref(),
                        &self.param_search,
                        &self.param_search_focus,
                        self.param_search_focus.is_focused(window),
                        cx,
                    ))
                    .child(params::list_panel(
                        &parameters,
                        group.as_deref(),
                        self.param_search.value(),
                        selected.as_deref(),
                        cx,
                    ))
                    .child(params::editor_panel(&parameters, selected.as_deref(), cx))
                    .child(params::file_panel(
                        &view,
                        &self.param_file_name,
                        &self.param_file_focus,
                        self.param_file_focus.is_focused(window),
                        &self.param_differences,
                        cx,
                    ))
                    .into_any_element()
            }
            // Measured so UI tests can address the body itself rather than only the controls in
            // it. The setup screen runs to nine panels and is taller than most windows, so a test
            // that wants the screen rather than a button needs a handle on the container.
            Screen::Logs => div()
                .id("logs-body")
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .child(logbrowse::screen(
                    &self.log_browse,
                    &self.log_name,
                    &self.log_name_focus,
                    self.log_name_focus.is_focused(window),
                    self.log_search.value(),
                    cx,
                ))
                .into_any_element(),
            Screen::Setup => probe::measured("setup-body", div())
                .id("setup-body")
                .flex()
                .flex_1()
                .overflow_y_scroll()
                .child(self.setup_body(&view, cx))
                .into_any_element(),
        };

        probe::measured("root", div())
            .flex()
            .flex_col()
            .size_full()
            .overflow_hidden()
            .bg(rgb(theme::BG))
            .text_color(rgb(theme::TEXT))
            .child(
                probe::measured("header", div())
                    .flex()
                    .items_center()
                    .justify_between()
                    .px_4()
                    .pt_2()
                    .bg(rgb(theme::PANEL))
                    .border_b_1()
                    .border_color(rgb(theme::BORDER))
                    .child(
                        div()
                            .flex()
                            .items_end()
                            .gap_4()
                            .child(
                                div()
                                    .flex()
                                    .flex_col()
                                    .pb_2()
                                    .child(div().text_xl().child("Mission Planner"))
                                    .child(
                                        div()
                                            .text_xs()
                                            .text_color(rgb(theme::DIM))
                                            .child("Rust port - gpui"),
                                    ),
                            )
                            .child(self.tabs(cx))
                            .child(self.vehicle_picker(&view, cx)),
                    )
                    .child(
                        div()
                            .flex()
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
                                        this.sticks.set_enabled(false);
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
    }
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
const USAGE: &str = "mpr-gui - Mission Planner, in Rust

USAGE:
    mpr-gui [OPTIONS] [LINK]

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
        --screen SCREEN     open on fly, plan or setup (default: fly)
        --window WIDTHxHEIGHT
                            initial window size (default: 1600x1200)

ENVIRONMENT:
    MP_WINDOW    same as --window
    MP_SCREEN    same as --screen
    MP_PROBE     write control positions to this file, for UI tests
    MP_FACTS     write what the application believes to this file, for UI tests to assert on
    MP_SMOKE     exit 0 once the window has painted, non-zero if it does not
    MP_LOG_DIR   where flights are recorded (default: Mission Planner's own logs directory)
    MP_NO_RECORD do not record this flight
    MP_NO_TILES  do not fetch map imagery
";

/// Parses the command line.
///
/// Hand-rolled, because an argument-parsing crate for four options is a dependency to justify. The
/// rule that matters: anything starting with `-` is an option, never the link. Taking the first
/// argument as the link regardless meant `mpr-gui --help` tried to connect to a serial port called
/// "--help", and so did `--read-mission`.
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

fn main() {
    let raw: Vec<String> = std::env::args().skip(1).collect();

    // Help and version before anything else, so they work with no display and answer instantly.
    if raw.iter().any(|a| a == "--help" || a == "-h") {
        println!("{USAGE}");
        return;
    }
    if raw.iter().any(|a| a == "--version" || a == "-V") {
        println!("mpr-gui {}", env!("CARGO_PKG_VERSION"));
        return;
    }

    let arguments = match parse_arguments(raw) {
        Ok(arguments) => arguments,
        Err(problem) => {
            eprintln!("mpr-gui: {problem}\n\n{USAGE}");
            std::process::exit(2);
        }
    };

    // A flag beats its environment variable: the variable is the standing preference and the flag
    // is this run. Passed down as values rather than written back into the environment, which
    // would mean mutating a process-global from one thread and is why this crate forbids unsafe.
    // What was remembered last time, under everything given explicitly. A settings file that is
    // missing, unreadable or full of rubbish yields defaults rather than stopping startup.
    let saved = settings::Settings::load();
    let (width, height) = window_size(arguments.window.as_deref(), saved.window);
    let screen = Screen::initial(arguments.screen.as_deref(), saved.screen.as_deref());
    // A link given on the command line wins; otherwise offer the one last connected to.
    let target = arguments.target.or_else(|| saved.link.clone());
    let read_mission = arguments.read_mission;

    platform::application().run(move |cx: &mut App| {
        // 1600x1200. Room for the panel columns and a map worth looking at side by side. Smaller
        // windows work - the panel columns scroll and the map takes what is left, which is what
        // the scrolling was added for - but this is the size the application is laid out for.
        //
        // MP_WINDOW overrides it, as WIDTHxHEIGHT. Trying a size should not need a rebuild, and a
        // screenshot at a particular size should not need a code change that then has to be
        // remembered and undone.
        let bounds = Bounds::centered(None, size(px(width), px(height)), cx);
        let options = WindowOptions {
            window_bounds: Some(WindowBounds::Windowed(bounds)),
            titlebar: Some(TitlebarOptions {
                title: Some(SharedString::from("Mission Planner (Rust)")),
                ..Default::default()
            }),
            ..Default::default()
        };

        if let Err(err) = cx.open_window(options, |_, cx| {
            cx.new(|cx| MissionPlanner::new(target, read_mission, screen, cx))
        }) {
            eprintln!("could not open a window: {err}");
            // A non-zero exit, because this is the failure a smoke test exists to catch and a
            // process that prints an error and exits 0 is a process CI calls a success.
            std::process::exit(1);
        }
        cx.activate(true);
        if smoke::enabled() {
            smoke::watch();
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

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
        // Taking the first argument as the link regardless meant `mpr-gui --read-mission` tried to
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

    #[test]
    fn a_mission_name_is_a_name_not_a_path() {
        // A typed "../../etc/passwd" writing outside the mission directory would be a surprise at
        // best. The name is reduced to its last component.
        let directory = MissionPlanner::plan_directory();
        for typed in ["../../etc/passwd", "/etc/passwd", "a/b/c.waypoints"] {
            let path = MissionPlanner::plan_path_named(typed);
            assert_eq!(
                path.parent(),
                Some(directory.as_path()),
                "{typed} escaped the mission directory: {}",
                path.display()
            );
        }
    }

    #[test]
    fn a_name_without_an_extension_gets_one() {
        // The operator meant a mission file; typing the suffix is not the interesting part.
        let path = MissionPlanner::plan_path_named("survey");
        assert_eq!(
            path.file_name().and_then(|n| n.to_str()),
            Some("survey.waypoints")
        );
    }

    #[test]
    fn an_existing_extension_is_left_alone() {
        let path = MissionPlanner::plan_path_named("survey.txt");
        assert_eq!(
            path.file_name().and_then(|n| n.to_str()),
            Some("survey.txt")
        );
    }

    #[test]
    fn an_empty_or_useless_name_falls_back_to_the_default() {
        // Rather than writing to a file called "" or to the directory itself.
        for typed in ["", "   ", ".", "..", "/"] {
            let path = MissionPlanner::plan_path_named(typed);
            assert_eq!(
                path.file_name().and_then(|n| n.to_str()),
                Some(DEFAULT_PLAN_FILE),
                "{typed:?} should fall back"
            );
        }
    }

    #[test]
    fn two_missions_do_not_overwrite_each_other() {
        // The whole point of the change: the path used to be fixed.
        let first = MissionPlanner::plan_path_named("survey");
        let second = MissionPlanner::plan_path_named("delivery");
        assert_ne!(first, second);
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
    fn flying_is_the_screen_it_opens_on() {
        // Not a preference: it is what the application is for, and an operator who connects to a
        // vehicle in flight should not have to find the right tab first.
        assert_eq!(Screen::ALL.first().copied(), Some(Screen::Fly));
    }
}
