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
mod setup;
mod telemetry;
mod ui;

use std::time::Duration;

use gpui::{
    App, Bounds, Context, MouseButton, SharedString, TitlebarOptions, Window, WindowBounds,
    WindowOptions, div, prelude::*, px, rgb, size,
};
use mapview::MapViewport;
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
    fn initial() -> Self {
        match std::env::var("MP_SCREEN").as_deref() {
            Ok("plan") => Self::Plan,
            Ok("setup") => Self::Setup,
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
    /// Whether the next completed download should replace the plan on screen.
    ///
    /// Set when the operator presses "read from vehicle" and cleared once the items arrive. Without
    /// it, a download started for the map would silently overwrite an edit in progress.
    adopt_vehicle_mission: bool,
    /// The last thing a file operation did, shown so a save is not silent.
    file_status: Option<String>,
}

impl MissionPlanner {
    fn new(target: Option<String>, cx: &mut Context<Self>) -> Self {
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

        // Spike sizes: a 100k-point survey track and 2,000 markers, which is heavier than a
        // typical flight and in the range the plan sets as the map target.
        let track_points = std::env::var("MP_TRACK_POINTS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(100_000);
        let markers = std::env::var("MP_MARKERS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(2_000);

        Self {
            telemetry,
            map: std::rc::Rc::new(std::cell::RefCell::new(MapViewport::new(
                track_points,
                markers,
            ))),
            auto_read_mission: std::env::args().any(|a| a == "--read-mission"),
            mission_requested: false,
            screen: Screen::initial(),
            plan: Plan::default(),
            adopt_vehicle_mission: false,
            file_status: None,
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

    /// The tab strip.
    fn tabs(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let current = self.screen;
        let mut strip = div().flex().gap_1();
        for screen in Screen::ALL {
            let selected = screen == current;
            strip = strip.child(
                div()
                    .id(screen.id())
                    .px_4()
                    .py_2()
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
        div()
            .flex()
            .flex_col()
            .flex_shrink_0()
            .min_h(px(0.0))
            .gap_4()
            .w(px(400.0))
            .child(fly::hud_panel(view))
            .child(
                div()
                    .id("fly-sidebar")
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(0.0))
                    .gap_4()
                    .pr_2()
                    .overflow_y_scroll()
                    .child(fly::actions_panel(view, cx))
                    .child(fly::vehicle_panel(view))
                    .child(fly::gps_panel(view))
                    .child(fly::link_panel(view)),
            )
    }

    /// The left column on the plan screen.
    fn plan_sidebar(&self, view: &TelemetryView, cx: &mut Context<Self>) -> impl IntoElement {
        // Copied out of the plan before building the elements: the listeners the panels install
        // take `&mut self`, so holding a borrow of `self.plan` across them would not compile.
        let items = self.plan.items().to_vec();
        let origin = self.plan.origin().clone();
        let selected = self.plan.selected();

        div()
            .id("plan-sidebar")
            .flex()
            .flex_col()
            .flex_shrink_0()
            .gap_4()
            .pr_2()
            .overflow_y_scroll()
            .w(px(440.0))
            .child(plan::actions_panel(&items, &origin, view, cx))
            .child(plan::items_panel(&items, selected, cx))
            .child(plan::checks_panel(&items, view))
    }

    /// The setup screen, which is one column and no map.
    fn setup_body(&self, view: &TelemetryView) -> impl IntoElement {
        div()
            .flex()
            .flex_col()
            .gap_4()
            .p_4()
            .child(setup::identity_panel(view))
            .child(setup::calibration_panel())
    }

    /// The map, with the handlers that make it a map rather than a picture.
    fn map_pane(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let following = self.map.borrow().is_following();
        let planning = self.screen == Screen::Plan;

        div()
            .flex()
            .flex_col()
            .flex_1()
            .min_w(px(0.0))
            .overflow_hidden()
            .gap_2()
            .child(
                div()
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
                    .on_mouse_down(MouseButton::Left, {
                        let map = self.map.clone();
                        move |event, _window, _cx| {
                            map.borrow_mut().begin_drag(
                                f32::from(event.position.x),
                                f32::from(event.position.y),
                            );
                        }
                    })
                    .on_mouse_move({
                        let map = self.map.clone();
                        move |event, window, _cx| {
                            if event.pressed_button == Some(MouseButton::Left) {
                                map.borrow_mut().drag_to(
                                    f32::from(event.position.x),
                                    f32::from(event.position.y),
                                );
                                // Repaint immediately: a map that only updates on the next
                                // telemetry tick feels broken to drag.
                                window.refresh();
                            }
                        }
                    })
                    .on_mouse_up(MouseButton::Left, {
                        let map = self.map.clone();
                        move |_event, _window, _cx| {
                            map.borrow_mut().end_drag();
                        }
                    })
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
                                this.plan.add_waypoint(position, plan::DEFAULT_ALTITUDE);
                                this.sync_map_mission();
                            } else {
                                this.fly_here(position);
                            }
                            cx.notify();
                        }),
                    )
                    .child(mapview::map_element(self.map.clone()))
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
        let text = if map.has_fix() {
            format!(
                "flight path: {} points recorded, {} drawn in {} path(s), {} refused  -  paint {:.2} ms avg, {:.2} ms worst over {} frames",
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
                "no position yet  -  synthetic scene: {} points, {} markers  -  paint {:.2} ms avg over {} frames",
                map.track_len(),
                map.marker_len(),
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
            Screen::Fly => div()
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .gap_4()
                .p_4()
                .child(self.fly_sidebar(&view, cx))
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .flex_1()
                        .min_w(px(0.0))
                        .min_h(px(0.0))
                        .gap_4()
                        .child(self.map_pane(cx))
                        .child(div().flex_shrink_0().child(fly::messages_panel(&view))),
                )
                .into_any_element(),
            Screen::Plan => div()
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .gap_4()
                .p_4()
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

        div()
            .flex()
            .flex_col()
            .size_full()
            .overflow_hidden()
            .bg(rgb(theme::BG))
            .text_color(rgb(theme::TEXT))
            .child(
                div()
                    .flex()
                    .items_center()
                    .justify_between()
                    .px_5()
                    .pt_3()
                    .bg(rgb(theme::PANEL))
                    .border_b_1()
                    .border_color(rgb(theme::BORDER))
                    .child(
                        div()
                            .flex()
                            .items_end()
                            .gap_6()
                            .child(
                                div()
                                    .flex()
                                    .flex_col()
                                    .pb_3()
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
                            .pb_3()
                            .child(div().size_2().rounded_full().bg(rgb(status_colour)))
                            .child(div().text_sm().text_color(rgb(theme::DIM)).child(status)),
                    ),
            )
            .child(body)
    }
}

fn main() {
    let target = std::env::args().nth(1);

    platform::application().run(move |cx: &mut App| {
        // Tall enough for the HUD plus all three panels without clipping. Grown twice from the
        // original 760: once when the link panel was cut off, again when the HUD was added above
        // it. Worth keeping generous - a control station that hides its bottom row is worse than
        // one that needs a scroll.
        let bounds = Bounds::centered(None, size(px(1180.0), px(980.0)), cx);
        let options = WindowOptions {
            window_bounds: Some(WindowBounds::Windowed(bounds)),
            titlebar: Some(TitlebarOptions {
                title: Some(SharedString::from("Mission Planner (Rust)")),
                ..Default::default()
            }),
            ..Default::default()
        };

        if let Err(err) = cx.open_window(options, |_, cx| {
            cx.new(|cx| MissionPlanner::new(target, cx))
        }) {
            eprintln!("could not open a window: {err}");
            return;
        }
        cx.activate(true);
    });
}
