//! `mpr-gui` - the graphical front end.
//!
//! Built on gpui, Zed's GPU-accelerated UI framework (DELIVERABLES.md D6/D7).
//!
//! Run with a link URL to connect, or with no arguments for a disconnected shell:
//!   mpr-gui tcp:127.0.0.1:5760

#![allow(clippy::print_stderr)]

mod hud;
mod mapview;
mod platform;
mod telemetry;

use std::time::Duration;

use gpui::{
    App, Bounds, Context, SharedString, TitlebarOptions, Window, WindowBounds, WindowOptions, div,
    prelude::*, px, rgb, size,
};
use mapview::MapViewport;
use telemetry::{Telemetry, TelemetryView};

/// Palette. Deliberately close to Mission Planner's dark theme so the port feels familiar rather
/// than merely new.
#[allow(unreachable_pub)] // a private module; pub here is documentation, not API
mod theme {
    /// Window background.
    pub const BG: u32 = 0x1b1f23;
    /// Panel background.
    pub const PANEL: u32 = 0x24292e;
    /// Panel border.
    pub const BORDER: u32 = 0x3a4048;
    /// Primary text.
    pub const TEXT: u32 = 0xe6edf3;
    /// Secondary text.
    pub const DIM: u32 = 0x8b949e;
    /// Good: link up, telemetry flowing.
    pub const OK: u32 = 0x3fb950;
    /// Caution: connected but incomplete.
    pub const WARN: u32 = 0xd29922;
    /// Bad: no link, or armed.
    pub const ALERT: u32 = 0xf85149;
}

/// How often to repaint. 10 Hz is plenty for numeric readouts and keeps an idle GCS cheap; the
/// map and HUD (D7-D9) will drive their own higher-rate rendering.
const REFRESH: Duration = Duration::from_millis(100);

/// Repaint interval when measuring the renderer: as fast as the executor will schedule, so paint
/// cost is measured rather than the timer.
const REFRESH_BENCH: Duration = Duration::from_millis(1);

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
        }
    }

    /// A labelled value, the unit this UI is mostly made of.
    fn field(label: &str, value: impl Into<SharedString>, colour: u32) -> impl IntoElement {
        div()
            .flex()
            .flex_col()
            .gap_1()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(label.to_owned()),
            )
            .child(div().text_lg().text_color(rgb(colour)).child(value.into()))
    }

    fn panel(title: &str, body: impl IntoElement) -> impl IntoElement {
        div()
            .flex()
            .flex_col()
            .gap_3()
            .p_4()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_md()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(title.to_uppercase()),
            )
            .child(body)
    }

    /// The primary flight display: canvas graphics with numeric readouts overlaid.
    ///
    /// Text is overlaid rather than painted inside the canvas because gpui shapes text through
    /// its own element pipeline; reproducing that inside a paint callback would be a lot of work
    /// for four numbers.
    fn hud_panel(view: &TelemetryView) -> impl IntoElement {
        let state = view.state.clone();
        let (speed, altitude, heading, mode) = view.state.as_ref().map_or_else(
            || {
                (
                    "--".to_owned(),
                    "--".to_owned(),
                    "--".to_owned(),
                    "no vehicle".to_owned(),
                )
            },
            |s| {
                (
                    format!("{:.0}", s.ground_speed.0),
                    format!("{:.0}", s.altitude_relative.0),
                    format!("{:03.0}", s.heading.degrees()),
                    if s.armed {
                        "ARMED".to_owned()
                    } else {
                        "disarmed".to_owned()
                    },
                )
            },
        );
        let mode_colour = if view.state.as_ref().is_some_and(|s| s.armed) {
            theme::ALERT
        } else {
            theme::DIM
        };

        div()
            .relative()
            .h(px(260.0))
            .w_full()
            .overflow_hidden()
            .rounded_md()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .child(
                gpui::canvas(
                    |_bounds, _window, _cx| (),
                    move |bounds, (), window, _cx| {
                        hud::paint_hud(state.as_deref(), bounds, window);
                    },
                )
                .size_full(),
            )
            .child(
                div()
                    .absolute()
                    .top_2()
                    .left_2()
                    .flex()
                    .flex_col()
                    .child(div().text_xs().text_color(rgb(theme::DIM)).child("m/s"))
                    .child(div().text_lg().text_color(rgb(theme::TEXT)).child(speed)),
            )
            .child(
                div()
                    .absolute()
                    .top_2()
                    .right_2()
                    .flex()
                    .flex_col()
                    .items_end()
                    .child(div().text_xs().text_color(rgb(theme::DIM)).child("m"))
                    .child(div().text_lg().text_color(rgb(theme::TEXT)).child(altitude)),
            )
            .child(
                div()
                    .absolute()
                    .bottom_2()
                    .left_0()
                    .w_full()
                    .flex()
                    .justify_center()
                    .gap_4()
                    .child(div().text_sm().text_color(rgb(theme::TEXT)).child(heading))
                    .child(div().text_sm().text_color(rgb(mode_colour)).child(mode)),
            )
    }

    /// A clickable control. gpui needs an id on anything interactive so it can track hover and
    /// press state across frames.
    fn button(
        id: &'static str,
        label: impl Into<SharedString>,
        on_click: impl Fn(&(), &mut Window, &mut App) + 'static,
    ) -> impl IntoElement {
        div()
            .id(id)
            .px_3()
            .py_1()
            .rounded_md()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .text_sm()
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(label.into())
            .on_click(move |_event, window, cx| on_click(&(), window, cx))
    }

    fn vehicle_panel(view: &TelemetryView) -> impl IntoElement {
        let (armed, armed_colour) = match view.state.as_ref() {
            Some(s) if s.armed => ("ARMED".to_owned(), theme::ALERT),
            Some(_) => ("disarmed".to_owned(), theme::TEXT),
            None => ("no vehicle".to_owned(), theme::DIM),
        };
        let position = view.state.as_ref().and_then(|s| s.position).map_or_else(
            || "no position".to_owned(),
            |p| format!("{:.6}, {:.6}", p.latitude(), p.longitude()),
        );
        let altitude = view.state.as_ref().map_or_else(
            || "--".to_owned(),
            |s| format!("{:.1} m", s.altitude_relative.0),
        );
        let speed = view.state.as_ref().map_or_else(
            || "--".to_owned(),
            |s| format!("{:.1} m/s", s.ground_speed.0),
        );
        let heading = view.state.as_ref().map_or_else(
            || "--".to_owned(),
            |s| format!("{:.0}°", s.heading.degrees()),
        );

        Self::panel(
            "vehicle",
            div()
                .flex()
                .flex_col()
                .gap_3()
                .child(Self::field("state", armed, armed_colour))
                .child(Self::field("position", position, theme::TEXT))
                .child(
                    div()
                        .flex()
                        .gap_4()
                        .child(Self::field("altitude", altitude, theme::TEXT))
                        .child(Self::field("ground speed", speed, theme::TEXT))
                        .child(Self::field("heading", heading, theme::TEXT)),
                ),
        )
    }

    fn gps_panel(view: &TelemetryView) -> impl IntoElement {
        let (fix_text, fix_colour) = match view.state.as_ref().map(|s| s.gps.fix_type) {
            Some(0 | 1) | None => ("no fix".to_owned(), theme::ALERT),
            Some(2) => ("2D".to_owned(), theme::WARN),
            Some(3) => ("3D".to_owned(), theme::OK),
            Some(4) => ("DGPS".to_owned(), theme::OK),
            Some(5) => ("RTK float".to_owned(), theme::OK),
            Some(other) if other >= 6 => ("RTK fixed".to_owned(), theme::OK),
            Some(other) => (format!("fix {other}"), theme::WARN),
        };
        let sats = view
            .state
            .as_ref()
            .map_or_else(|| "--".to_owned(), |s| s.gps.satellites_visible.to_string());
        let battery = view.state.as_ref().map_or_else(
            || "--".to_owned(),
            |s| {
                format!(
                    "{:.2} V  {}%",
                    s.battery.voltage, s.battery.remaining_percent
                )
            },
        );

        Self::panel(
            "gps and power",
            div()
                .flex()
                .flex_col()
                .gap_3()
                .child(
                    div()
                        .flex()
                        .gap_4()
                        .child(Self::field("fix", fix_text, fix_colour))
                        .child(Self::field("satellites", sats, theme::TEXT)),
                )
                .child(Self::field("battery", battery, theme::TEXT)),
        )
    }

    fn link_panel(view: &TelemetryView) -> impl IntoElement {
        let loss = view.state.as_ref().map_or_else(
            || "--".to_owned(),
            |s| format!("{:.2} %", s.link.loss_percent()),
        );
        let loss_colour = view.state.as_ref().map_or(theme::DIM, |s| {
            if s.link.loss_percent() > 5.0 {
                theme::ALERT
            } else {
                theme::TEXT
            }
        });
        let crc_colour = if view.crc_errors > 0 {
            theme::WARN
        } else {
            theme::TEXT
        };

        Self::panel(
            "link",
            div()
                .flex()
                .flex_col()
                .gap_3()
                .child(
                    div()
                        .flex()
                        .gap_4()
                        .child(Self::field("frames", view.frames.to_string(), theme::TEXT))
                        .child(Self::field("loss", loss, loss_colour))
                        .child(Self::field(
                            "crc errors",
                            view.crc_errors.to_string(),
                            crc_colour,
                        )),
                )
                .child(Self::field(
                    "systems on link",
                    view.vehicle_count.to_string(),
                    theme::TEXT,
                )),
        )
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
        if !view.mission.is_empty() {
            self.map.borrow_mut().set_mission(&view.mission);
        }
        if self.auto_read_mission && !self.mission_requested && view.vehicle.is_some() {
            self.mission_requested = true;
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

        let map_stats = {
            let map = self.map.borrow();
            if map.has_fix() {
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
            }
        };

        let mission_label = if view.mission.is_empty() {
            "read mission".to_owned()
        } else {
            format!("mission: {} items", view.mission.len())
        };

        let vehicle_label = view
            .vehicle
            .map_or_else(|| "no vehicle".to_owned(), |id| format!("vehicle {id}"));

        div()
            .flex()
            .flex_col()
            .size_full()
            .bg(rgb(theme::BG))
            .text_color(rgb(theme::TEXT))
            .child(
                div()
                    .flex()
                    .items_center()
                    .justify_between()
                    .px_5()
                    .py_3()
                    .bg(rgb(theme::PANEL))
                    .border_b_1()
                    .border_color(rgb(theme::BORDER))
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .child(div().text_xl().child("Mission Planner"))
                            .child(
                                div()
                                    .text_xs()
                                    .text_color(rgb(theme::DIM))
                                    .child("Rust port - gpui"),
                            ),
                    )
                    .child(
                        div()
                            .flex()
                            .items_center()
                            .gap_2()
                            .child(div().size_2().rounded_full().bg(rgb(status_colour)))
                            .child(div().text_sm().text_color(rgb(theme::DIM)).child(status)),
                    ),
            )
            .child(
                div()
                    .flex()
                    .flex_1()
                    .gap_4()
                    .p_4()
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .flex_shrink_0()
                            .gap_4()
                            .w(px(400.0))
                            .child(
                                div()
                                    .flex()
                                    .items_center()
                                    .justify_between()
                                    .child(
                                        div()
                                            .text_sm()
                                            .text_color(rgb(theme::DIM))
                                            .child(vehicle_label),
                                    )
                                    .child(Self::button(
                                        "read-mission",
                                        mission_label,
                                        cx.listener(|this, _event: &(), _window, cx| {
                                            this.telemetry.request_mission();
                                            cx.notify();
                                        }),
                                    )),
                            )
                            .child(Self::hud_panel(&view))
                            .child(Self::vehicle_panel(&view))
                            .child(Self::gps_panel(&view))
                            .child(Self::link_panel(&view)),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .flex_1()
                            .min_w(px(0.0))
                            .overflow_hidden()
                            .gap_2()
                            .child(
                                div()
                                    .flex()
                                    .flex_1()
                                    .bg(rgb(theme::PANEL))
                                    .border_1()
                                    .border_color(rgb(theme::BORDER))
                                    .rounded_md()
                                    .child(mapview::map_element(self.map.clone())),
                            )
                            .child(
                                // min_w(0) plus truncation is load-bearing, not cosmetic: a flex
                                // item defaults to min-width:auto, so this line's intrinsic text
                                // width was widening the whole column whenever a number grew a
                                // digit. The map viewport resized with it - 744px to 755px
                                // between frames - which invalidated cached geometry every frame
                                // and made the renderer look four times slower than it is.
                                div()
                                    .flex()
                                    .w_full()
                                    .min_w(px(0.0))
                                    .px_2()
                                    .text_xs()
                                    .text_color(rgb(theme::DIM))
                                    .truncate()
                                    .child(map_stats),
                            ),
                    ),
            )
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
