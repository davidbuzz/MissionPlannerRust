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

use gpui::{
    AnyElement, App, ClickEvent, ScrollHandle, SharedString, Window, div, prelude::*, px, rgb,
};

use crate::MissionPlanner;

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
    /// A link's text, underlined as a web page's link is: the accent's blue, which the pages'
    /// own link labels (Motor Test's motor order, Flight Modes' help) are drawn in.
    pub const LINK: u32 = ACCENT;
    /// A link's text with the pointer over it: a lighter blue.
    pub const LINK_HOVER: u32 = 0x79c0ff;
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

/// A titled box with a control at the title's right, on the same row: the mission grid's fold
/// button, which would otherwise take a row of its own.
pub fn panel_with_corner(
    title: &str,
    corner: impl IntoElement,
    body: impl IntoElement,
) -> gpui::Div {
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
                .flex()
                .items_center()
                .justify_between()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(title.to_uppercase()),
                )
                .child(corner),
        )
        .child(body)
}

/// Whether a control takes its click, and why not while it is dimmed.
///
/// A control drawn dimmed - one the C# sets `Enabled = false` in that state, or one a ruling
/// leaves out - takes no click, as the C#'s takes none. But a press that did nothing is taken for
/// a dead control, so the click of a dimmed control says why on the status line, quietly, and does
/// nothing else: the owner's word of 2026-10-16 on the click crawl's 45 dimmed controls
/// (docs/click-report.md), his rule of 2026-10-14 for a press the C# answers with nothing
/// ([`crate::fly::Refusal::Quiet`]). The reason is the state that dims the control, in the words
/// the port uses for it - "no vehicle", "no log selected" - or the ruling's.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Live {
    /// Drawn live: a click runs the control's handler.
    Yes,
    /// Drawn dimmed, and why: a click puts the words on the status line and runs nothing.
    Dimmed(SharedString),
    /// Drawn dimmed by a caller that gives no reason - a plain `false` - so a click does nothing.
    Inert,
}

impl Live {
    /// Live while `live` holds, else dimmed for `why`.
    #[must_use]
    pub fn when(live: bool, why: impl Into<SharedString>) -> Self {
        if live {
            Self::Yes
        } else {
            Self::Dimmed(why.into())
        }
    }

    /// Dimmed for `why`.
    #[must_use]
    pub fn dimmed(why: impl Into<SharedString>) -> Self {
        Self::Dimmed(why.into())
    }

    /// Whether the control is drawn live.
    #[must_use]
    pub const fn is_live(&self) -> bool {
        matches!(self, Self::Yes)
    }
}

impl From<bool> for Live {
    /// A control's plain `enabled`, as the widgets took it before a dimmed control could say why:
    /// live, or dimmed with no reason given.
    fn from(live: bool) -> Self {
        if live { Self::Yes } else { Self::Inert }
    }
}

/// A click on a control in the state `live`, the status line being `status` (the window's
/// `file_status`): whether the control's handler runs. A dimmed control's runs nothing, and why it
/// is dimmed goes on the status line; an inert one's runs nothing and says nothing.
#[must_use]
pub fn click(live: &Live, status: &mut Option<String>) -> bool {
    match live {
        Live::Yes => true,
        Live::Dimmed(why) => {
            *status = Some(why.to_string());
            false
        }
        Live::Inert => false,
    }
}

/// The click of a control dimmed for `why` ([`click`]): the words on the main window's status
/// line, which is drawn again, and nothing else. The window's [`MissionPlanner`] is found from the
/// window the click lands in - it is the root view of the only one - so a widget that is handed no
/// `Context` can give its dimmed form this click all the same.
pub fn says_why(why: SharedString) -> impl Fn(&ClickEvent, &mut Window, &mut App) + 'static {
    let live = Live::Dimmed(why);
    move |_event, window, cx| {
        let Some(Some(planner)) = window.root::<MissionPlanner>() else {
            return;
        };
        planner.update(cx, |this, cx| {
            let _runs = click(&live, &mut this.file_status);
            cx.notify();
        });
    }
}

/// A control's element as it is drawn dimmed: given [`says_why`]'s click while `live` is dimmed
/// for a reason, and left as it is otherwise - a live control's handler is its widget's own, and
/// an inert one takes no click.
pub fn dimmed_click<E: StatefulInteractiveElement>(element: E, live: &Live) -> E {
    match live {
        Live::Dimmed(why) => element.on_click(says_why(why.clone())),
        Live::Yes | Live::Inert => element,
    }
}

/// A control that commands the aircraft.
///
/// Visually distinct from [`button`] on purpose. Reading telemetry and changing what the aircraft
/// is doing are different kinds of act, and a UI where "read mission" and "arm" look the same
/// invites the second when the operator meant the first.
///
/// `enabled` false - or [`Live::Dimmed`] - renders the control dimmed and inert rather than hiding
/// it, so the layout does not move as the vehicle's state changes - a button that appears under
/// the cursor is a button that gets pressed by accident. Dimmed for a reason, its click says why
/// ([`dimmed_click`]).
pub fn action(
    id: &'static str,
    label: impl Into<SharedString>,
    colour: u32,
    enabled: impl Into<Live>,
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
    enabled: impl Into<Live>,
    min_width: Option<gpui::Pixels>,
    on_click: impl Fn(&(), &mut Window, &mut App) + 'static,
) -> AnyElement {
    let live = enabled.into();
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

    if live.is_live() {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(colour))
            .text_color(rgb(colour))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(move |_event, window, cx| on_click(&(), window, cx))
            .into_any_element()
    } else {
        dimmed_click(
            base.bg(rgb(theme::PANEL))
                .border_color(rgb(theme::BORDER))
                .text_color(rgb(theme::DIM)),
            &live,
        )
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

    /// A dimmed control's click: its reason on the status line, and its handler not run - the
    /// owner's word of 2026-10-16. A live one's runs and leaves the status line alone; one dimmed
    /// with no reason runs nothing and says nothing.
    #[test]
    fn a_dimmed_controls_click_says_why_and_does_nothing_else() {
        let mut status = Some("connected".to_owned());
        assert!(!click(&Live::dimmed("no vehicle"), &mut status));
        assert_eq!(status.as_deref(), Some("no vehicle"));

        let mut status = Some("connected".to_owned());
        assert!(click(&Live::Yes, &mut status));
        assert_eq!(status.as_deref(), Some("connected"));

        assert!(!click(&Live::Inert, &mut status));
        assert_eq!(status.as_deref(), Some("connected"));
    }

    #[test]
    fn a_control_is_dimmed_for_its_reason_when_its_state_does_not_hold() {
        assert_eq!(Live::when(true, "no log selected"), Live::Yes);
        assert_eq!(
            Live::when(false, "no log selected"),
            Live::Dimmed(SharedString::from("no log selected"))
        );
        assert!(Live::Yes.is_live());
        assert!(!Live::dimmed("x").is_live() && !Live::Inert.is_live());
        // A widget's plain `enabled`: live, or dimmed with nothing to say.
        assert_eq!(Live::from(true), Live::Yes);
        assert_eq!(Live::from(false), Live::Inert);
    }

    #[test]
    fn a_link_changes_under_the_pointer() {
        // A link with no hover state does not read as one, and the owner asked for SETUP's links
        // between pages to look like a web page's (2026-10-14). Checked at compile time, because
        // both sides are constants.
        const _: () = assert!(theme::LINK_HOVER != theme::LINK);
    }
}
