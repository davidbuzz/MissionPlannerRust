//! `mpr-gui` - the graphical front end.
//!
//! Built on gpui, Zed's GPU-accelerated UI framework (DELIVERABLES.md D6/D7).
//!
//! Run with a link URL to connect, or with no arguments for a disconnected shell:
//!   mpr-gui tcp:127.0.0.1:5760

#![allow(clippy::print_stderr)]

mod platform;
mod telemetry;

use std::time::Duration;

use gpui::{
    App, Bounds, Context, SharedString, TitlebarOptions, Window, WindowBounds, WindowOptions, div,
    prelude::*, px, rgb, size,
};
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

struct MissionPlanner {
    telemetry: Telemetry,
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
                cx.background_executor().timer(REFRESH).await;
                if this.update(cx, |_, cx| cx.notify()).is_err() {
                    break;
                }
            }
        })
        .detach();

        Self { telemetry }
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
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        let view = self.telemetry.view();

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
                            .gap_4()
                            .w(px(400.0))
                            .child(
                                div()
                                    .text_sm()
                                    .text_color(rgb(theme::DIM))
                                    .child(vehicle_label),
                            )
                            .child(Self::vehicle_panel(&view))
                            .child(Self::gps_panel(&view))
                            .child(Self::link_panel(&view)),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_1()
                            .items_center()
                            .justify_center()
                            .bg(rgb(theme::PANEL))
                            .border_1()
                            .border_color(rgb(theme::BORDER))
                            .rounded_md()
                            .child(
                                div()
                                    .text_color(rgb(theme::DIM))
                                    .child("map / HUD viewport - D7, D8, D9"),
                            ),
                    ),
            )
    }
}

fn main() {
    let target = std::env::args().nth(1);

    platform::application().run(move |cx: &mut App| {
        // Tall enough that the left column's panels are fully visible without scrolling; the
        // first version clipped the link panel against the bottom edge.
        let bounds = Bounds::centered(None, size(px(1180.0), px(880.0)), cx);
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
