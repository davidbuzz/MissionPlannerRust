//! `mpr-gui` - the graphical front end.
//!
//! Built on gpui, Zed's GPU-accelerated UI framework (DELIVERABLES.md D6/D7). This is the
//! skeleton: a window, a theme and a layout the telemetry panels will grow into.
//!
//! Run with a link URL to connect, or with no arguments for a disconnected shell:
//!   mpr-gui tcp:127.0.0.1:5760

#![allow(clippy::print_stderr)]

use gpui::{
    App, Application, Bounds, Context, SharedString, TitlebarOptions, Window, WindowBounds,
    WindowOptions, div, prelude::*, px, rgb, size,
};

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
    /// Accent, used for the active link indicator.
    pub const ACCENT: u32 = 0x3fb950;
    /// Warning / disconnected.
    pub const WARN: u32 = 0xd29922;
}

struct MissionPlanner {
    target: Option<SharedString>,
}

impl MissionPlanner {
    fn new(target: Option<String>) -> Self {
        Self { target: target.map(SharedString::from) }
    }

    /// A labelled value, the unit this UI is mostly made of.
    fn field(label: &str, value: impl Into<SharedString>) -> impl IntoElement {
        div()
            .flex()
            .flex_col()
            .gap_1()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(label.to_owned()))
            .child(div().text_lg().text_color(rgb(theme::TEXT)).child(value.into()))
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
                    .text_sm()
                    .text_color(rgb(theme::DIM))
                    .child(title.to_uppercase()),
            )
            .child(body)
    }
}

impl Render for MissionPlanner {
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        let (status, status_colour) = match &self.target {
            Some(target) => (format!("link target {target}"), theme::ACCENT),
            None => ("no link - start with a url, e.g. tcp:127.0.0.1:5760".to_owned(), theme::WARN),
        };

        div()
            .flex()
            .flex_col()
            .size_full()
            .bg(rgb(theme::BG))
            .text_color(rgb(theme::TEXT))
            // Header.
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
                                    .child("Rust port - gpui / wgpu"),
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
            // Body.
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
                            .w(px(320.0))
                            .child(Self::panel(
                                "vehicle",
                                div()
                                    .flex()
                                    .flex_col()
                                    .gap_3()
                                    .child(Self::field("state", "no vehicle"))
                                    .child(Self::field("position", "--")),
                            ))
                            .child(Self::panel(
                                "link",
                                div()
                                    .flex()
                                    .flex_col()
                                    .gap_3()
                                    .child(Self::field("frames", "0"))
                                    .child(Self::field("loss", "0.0 %")),
                            )),
                    )
                    // The map and HUD land here once the GPU render core exists (D7/D8/D9).
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

    Application::new().run(move |cx: &mut App| {
        let bounds = Bounds::centered(None, size(px(1100.0), px(700.0)), cx);
        let options = WindowOptions {
            window_bounds: Some(WindowBounds::Windowed(bounds)),
            titlebar: Some(TitlebarOptions {
                title: Some(SharedString::from("Mission Planner (Rust)")),
                ..Default::default()
            }),
            ..Default::default()
        };

        if let Err(err) = cx.open_window(options, |_, cx| cx.new(|_| MissionPlanner::new(target)))
        {
            eprintln!("could not open a window: {err}");
            return;
        }
        cx.activate(true);
    });
}
