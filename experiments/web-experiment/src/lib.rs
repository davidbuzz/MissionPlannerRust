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
// SPDX-License-Identifier: GPL-3.0-only

//! The browser experiment's first question: does a gpui window open in a web page through gpui_web, at
//! the revision the planner uses? One window, one line of text, a click counter to prove input.

use std::borrow::Cow;

use gpui::{App, Context, Window, WindowOptions, div, prelude::*, rgb};
use wasm_bindgen::prelude::*;

struct Hello {
    clicks: u32,
}

impl Render for Hello {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .size_full()
            .flex()
            .flex_col()
            .items_center()
            .justify_center()
            .gap_4()
            .bg(rgb(0x1e1e2e))
            .text_color(rgb(0xcdd6f4))
            .child(div().text_3xl().child("MissionPlannerRust in the browser"))
            .child(
                div()
                    .id("click-me")
                    .px_4()
                    .py_2()
                    .rounded_md()
                    .bg(rgb(0x45475a))
                    .cursor_pointer()
                    .child(format!("clicked {} times", self.clicks))
                    .on_click(cx.listener(|this, _event, _window, cx| {
                        this.clicks += 1;
                        cx.notify();
                    })),
            )
    }
}

/// A web page has no system fonts for gpui to find, so the page brings its own, as Zed's web
/// examples do (crates/gpui/examples/example_support/fonts.rs).
fn load_fonts(cx: &App) -> bool {
    let fonts = [
        Cow::Borrowed(include_bytes!("../fonts/IBMPlexSans-Regular.ttf").as_slice()),
        Cow::Borrowed(include_bytes!("../fonts/IBMPlexSans-SemiBold.ttf").as_slice()),
    ];
    if let Err(err) = cx.text_system().add_fonts(fonts.into()) {
        web_sys::console::error_1(&format!("could not load the fonts: {err:#}").into());
        return false;
    }
    true
}

#[wasm_bindgen(start)]
pub fn start() {
    gpui_platform::web_init();
    gpui_platform::application().run(|cx: &mut App| {
        if !load_fonts(cx) {
            return;
        }
        if let Err(err) = cx.open_window(WindowOptions::default(), |_window, cx| {
            cx.new(|_| Hello { clicks: 0 })
        }) {
            web_sys::console::error_1(&format!("could not open the window: {err:?}").into());
        }
        cx.activate(true);
    });
}
