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

//! Palette and the small set of widgets every screen is built from.
//!
//! Kept separate from the screens so that the flight, plan and setup views cannot drift apart
//! visually - in Mission Planner they did, because each tab grew its own buttons.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, App, ScrollHandle, SharedString, Window, div, prelude::*, px, rgb};

/// Palette. Deliberately close to Mission Planner's dark theme so the port feels familiar rather
/// than merely new.
pub mod theme {
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
    /// The accent used for a selected tab or an active control.
    pub const ACCENT: u32 = 0x58a6ff;
    /// Background of a control that commits something to the aircraft.
    pub const ACTION: u32 = 0x2d333b;
    /// Behind the selected text of a text box: the accent, darkened so the text stays legible.
    pub const SELECTION: u32 = 0x1f4f82;
}

/// A labelled value, the unit this UI is mostly made of.
///
/// The value is `text_base`, not `text_lg`. There are fourteen of these down the flight screen's
/// left column and the two extra pixels each cost pushed the bottom panel off the end of it. The
/// numbers a pilot reads at a glance - speed, altitude, heading - are large on the HUD above,
/// which is where the eye goes; these are the same numbers to check rather than to fly by.
pub fn field(label: &str, value: impl Into<SharedString>, colour: u32) -> impl IntoElement {
    div()
        .flex()
        .flex_col()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(label.to_owned()),
        )
        .child(
            div()
                .text_base()
                .text_color(rgb(colour))
                .child(value.into()),
        )
}

/// A titled box.
///
/// Measured under `MP_PROBE`, keyed on its title, so a layout test can assert that no panel ends
/// up outside the window. Overflow is the failure this UI keeps having, and it is invisible in a
/// screenshot of the part that did fit.
pub fn panel(title: &str, body: impl IntoElement) -> gpui::Div {
    crate::probe::measured(format!("panel:{title}"), div())
        .flex()
        .flex_col()
        .gap_2()
        .p_3()
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

/// A control that commands the aircraft.
///
/// Visually distinct from [`button`] on purpose. Reading telemetry and changing what the aircraft
/// is doing are different kinds of act, and a UI where "read mission" and "arm" look the same
/// invites the second when the operator meant the first.
///
/// `enabled` false renders the control dimmed and inert rather than hiding it, so the layout does
/// not move as the vehicle's state changes - a button that appears under the cursor is a button
/// that gets pressed by accident.
pub fn action(
    id: &'static str,
    label: impl Into<SharedString>,
    colour: u32,
    enabled: bool,
    on_click: impl Fn(&(), &mut Window, &mut App) + 'static,
) -> AnyElement {
    action_sized(id, label, colour, enabled, None, on_click)
}

/// An action control with a minimum width.
///
/// Used to make controls that belong together the same size regardless of how long their labels
/// are. Two buttons that do nearly the same thing looking like different kinds of control is a
/// small thing that reads as carelessness.
pub fn action_sized(
    id: &'static str,
    label: impl Into<SharedString>,
    colour: u32,
    enabled: bool,
    min_width: Option<gpui::Pixels>,
    on_click: impl Fn(&(), &mut Window, &mut App) + 'static,
) -> AnyElement {
    // measured() wraps the plain div before `.id()`, because `.id()` yields a Stateful<Div> and
    // the bounds hook lives on Div. It reports where this control ended up, so a test script can
    // click it by name rather than by a coordinate that goes stale the next time the layout
    // changes, and it adds nothing to the element tree when probing is off.
    let mut base = crate::probe::measured(id, div())
        .id(id)
        .px_3()
        .py_1()
        .rounded_md()
        .border_1()
        .text_sm()
        .child(label.into());
    if let Some(min_width) = min_width {
        // Centred, so a short label in a wide button does not sit against the left edge looking
        // like the button was stretched by accident.
        base = base.min_w(min_width).flex().justify_center();
    }

    if enabled {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(colour))
            .text_color(rgb(colour))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(move |_event, window, cx| on_click(&(), window, cx))
            .into_any_element()
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .into_any_element()
    }
}

/// A horizontal progress bar with a caption.
pub fn progress(fraction: f32, colour: u32) -> impl IntoElement {
    let fraction = fraction.clamp(0.0, 1.0);
    div()
        .w_full()
        .h(px(4.0))
        .rounded_sm()
        .bg(rgb(theme::BORDER))
        .child(
            div()
                .h_full()
                .w(gpui::relative(fraction))
                .rounded_sm()
                .bg(rgb(colour)),
        )
}

/// How much of the track the thumb takes up.
///
/// Fixed rather than proportional. A proportional thumb needs the viewport height as well as the
/// scrollable distance, and gpui's `ScrollHandle` reports only the latter; guessing the ratio
/// would make the thumb lie about how much is hidden. A fixed thumb says "there is more, and you
/// are here", which is the whole job.
const THUMB_FRACTION: f32 = 0.2;

/// A scroll indicator for a container that scrolls.
///
/// gpui draws none. `overflow_y_scroll()` sets one style field: it clips the content and captures
/// the wheel, and that is all. There is no scrollbar geometry anywhere in gpui - Zed draws its own
/// with a component from a crate this does not depend on. The consequence is that a correctly
/// clipping column is pixel-identical to one whose content runs off the bottom of the window,
/// which is exactly the confusion that cost two rounds of layout debugging here.
///
/// An indicator rather than a control: it cannot be dragged, because the wheel already works and a
/// thumb that looks draggable and is not would be worse than none.
///
/// Note what is deliberately not used. `scrollbar_width()` exists on gpui's `Styled` and looks
/// like the way to reserve a gutter, but Taffy takes that width out of the container's content
/// box. On a column that was already too tall for its space, reserving ten pixels would have made
/// the problem it is meant to reveal slightly worse.
pub fn scroll_indicator(handle: &ScrollHandle) -> Option<AnyElement> {
    let max = f32::from(handle.max_offset().y);
    // Nothing is hidden, so there is nothing to indicate. An always-visible track on a column that
    // fits would be clutter claiming something is out of sight when nothing is.
    if max <= 1.0 {
        return None;
    }

    let offset = f32::from(handle.offset().y).abs();
    let travel = (offset / max).clamp(0.0, 1.0) * (1.0 - THUMB_FRACTION);

    Some(
        div()
            .absolute()
            .top_0()
            .right_0()
            .bottom_0()
            .w(px(4.0))
            .child(
                div()
                    .absolute()
                    .left_0()
                    .right_0()
                    .top(gpui::relative(travel))
                    .h(gpui::relative(THUMB_FRACTION))
                    .rounded_full()
                    // DIM rather than BORDER: at four pixels wide the border colour is almost
                    // invisible against the panel, which defeats the point of drawing it.
                    .bg(rgb(theme::DIM)),
            )
            .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_column_with_nothing_hidden_gets_no_indicator() {
        // An always-visible track on a column that fits would be clutter claiming something is out
        // of sight when nothing is.
        let handle = ScrollHandle::new();
        assert!(scroll_indicator(&handle).is_none());
    }

    #[test]
    fn the_thumb_leaves_room_to_travel() {
        // A thumb as tall as its track could not move, and a moving thumb is the only thing that
        // says which part of the content is on screen. Checked at compile time, because both
        // sides are constants.
        const _: () = assert!(THUMB_FRACTION > 0.0);
        const _: () = assert!(THUMB_FRACTION < 1.0);
    }
}
