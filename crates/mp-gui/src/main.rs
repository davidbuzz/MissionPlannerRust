//! `mpr-gui` - the graphical front end.
//!
//! Built on gpui, Zed's GPU-accelerated UI framework (DELIVERABLES.md D6/D7).
//!
//! Run with a link URL to connect, or with no arguments for a disconnected shell:
//!   mpr-gui tcp:127.0.0.1:5760

#![allow(clippy::print_stderr)]

mod fly;
mod hud;
mod mapview;
mod plan;
mod platform;
mod probe;
mod setup;
mod telemetry;
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
}

impl Screen {
    /// The tabs, in order.
    const ALL: [Self; 3] = [Self::Fly, Self::Plan, Self::Setup];

    /// The screen to open on, from `MP_SCREEN`.
    ///
    /// Exists so a screenshot can be taken of any screen without driving the tab strip with
    /// synthetic clicks, which is the sort of test that breaks whenever the layout moves.
    fn initial(requested: Option<&str>) -> Self {
        let named = match requested {
            Some(name) => name.to_owned(),
            None => std::env::var("MP_SCREEN").unwrap_or_default(),
        };
        match named.as_str() {
            "plan" => Self::Plan,
            "setup" => Self::Setup,
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
        }
    }

    const fn id(self) -> &'static str {
        match self {
            Self::Fly => "tab-fly",
            Self::Plan => "tab-plan",
            Self::Setup => "tab-setup",
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
            let source = std::env::var("MP_TILE_SOURCE")
                .ok()
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
        }
    }

    /// Where missions are read and written.
    ///
    /// A file dialog needs a platform integration gpui does not give us for free, so for now the
    /// path is fixed and reported in the UI. Silently writing somewhere the operator cannot find
    /// would be worse than a fixed location they can.
    fn plan_path() -> std::path::PathBuf {
        std::env::var("MP_PLAN_FILE").map_or_else(
            |_| {
                std::env::current_dir()
                    .unwrap_or_else(|_| std::path::PathBuf::from("."))
                    .join("mission.waypoints")
            },
            std::path::PathBuf::from,
        )
    }

    /// Writes the plan in QGC WPL 110 format, the one every ground station reads.
    fn save_plan(&mut self) {
        let path = Self::plan_path();
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
        let path = Self::plan_path();
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
                probe::measured("fly-sidebar", div())
                    .id("fly-sidebar")
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(0.0))
                    .gap_2()
                    .pr_2()
                    .overflow_y_scroll()
                    .child(fly::actions_panel(view, cx))
                    .child(fly::vehicle_panel(view))
                    .child(fly::health_panel(view)),
            )
    }

    /// The left column on the plan screen.
    fn plan_sidebar(&self, view: &TelemetryView, cx: &mut Context<Self>) -> impl IntoElement {
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
            .id("plan-sidebar")
            .flex()
            .flex_col()
            .flex_shrink_0()
            .gap_2()
            .pr_2()
            .overflow_y_scroll()
            .w(px(400.0))
            .child(plan::actions_panel(&items, &origin, view, cx))
            .child(plan::draw_panel(&draw, view, cx))
            .child(plan::items_panel(&items, selected, cx))
            .child(plan::editor_panel(&items, selected, cx))
            .child(plan::checks_panel(&items, view))
    }

    /// The setup screen, which is one column and no map.
    fn setup_body(&self, view: &TelemetryView) -> impl IntoElement {
        div()
            .flex()
            .flex_col()
            .gap_2()
            .p_2()
            .child(setup::identity_panel(view))
            .child(setup::calibration_panel())
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
                        cx.listener(move |this, _event: &gpui::MouseUpEvent, _window, _cx| {
                            this.dragging_waypoint = None;
                            this.map.borrow_mut().end_drag();
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
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let view = self.telemetry.view();

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
                .child(self.plan_sidebar(&view, cx))
                .child(self.map_pane(cx))
                .into_any_element(),
            Screen::Setup => div()
                .id("setup-body")
                .flex()
                .flex_1()
                .overflow_y_scroll()
                .child(self.setup_body(&view))
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
                            .child(self.tabs(cx)),
                    )
                    .child(
                        div()
                            .flex()
                            .items_center()
                            .gap_2()
                            .pb_2()
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
fn window_size(requested: Option<&str>) -> (f32, f32) {
    const DEFAULT: (f32, f32) = (1600.0, 1200.0);

    let value = match requested {
        Some(value) => value.to_owned(),
        None => match std::env::var("MP_WINDOW") {
            Ok(value) => value,
            Err(_) => return DEFAULT,
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
    let (width, height) = window_size(arguments.window.as_deref());
    let screen = Screen::initial(arguments.screen.as_deref());
    let target = arguments.target;
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
            return;
        }
        cx.activate(true);
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
    }

    #[test]
    fn a_flag_beats_its_environment_variable() {
        // The variable is a standing preference; the flag is this run.
        assert_eq!(window_size(Some("1280x800")), (1280.0, 800.0));
        assert_eq!(Screen::initial(Some("plan")), Screen::Plan);
        assert_eq!(Screen::initial(Some("setup")), Screen::Setup);
        // A name that is not a screen opens on the one the application is for.
        assert_eq!(Screen::initial(Some("nonsense")), Screen::Fly);
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
