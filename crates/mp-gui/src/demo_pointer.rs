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

//! The demo pointer - the owner's Welcome-Demo-Sitl plugin (2026-10-05), not in the C#: a cursor
//! drawn over the window that moves to a control, or a place on the map, as a hand would, and
//! clicks it there. The click is a real one: a mouse down and up dispatched through the window,
//! which gpui hit-tests as it does a pilot's, so the demo drives the planner by the path a user
//! takes. Plugins ask for it through plugin.wit's demo calls (plugins_ui.rs serves them); the
//! planner's probe (probe.rs) says where each named control is.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{
    AnyElement, Context, KeyDownEvent, Keystroke, Modifiers, MouseButton, MouseDownEvent,
    MouseMoveEvent, MouseUpEvent, PathBuilder, PlatformInput, Window, canvas, div, point,
    prelude::*, px, rgb,
};
use web_time::{Duration, Instant};

use crate::MissionPlanner;

/// How long a click's ring is drawn after it.
const RING: Duration = Duration::from_millis(400);

/// A move under way: from where, to where, since when, for how long, and the button clicked at
/// its end.
#[derive(Debug, Clone, Copy)]
struct Gesture {
    from: (f32, f32),
    to: (f32, f32),
    started: Instant,
    took: Duration,
    button: MouseButton,
}

/// The pointer: where it is drawn, the move under way, and what to type after it.
#[derive(Debug, Default)]
pub struct DemoPointer {
    at: Option<(f32, f32)>,
    gesture: Option<Gesture>,
    typing: VecDeque<String>,
    clicked: Option<((f32, f32), Instant)>,
    /// The clicks asked for so far, and the last one's target: a control's name, or `map`.
    clicks: usize,
    last: Option<String>,
    /// The demo is over: the pointer goes once its last gesture and ring are done.
    ended: bool,
}

impl DemoPointer {
    /// The pointer moves to `to`, window pixels, over `millis`, and clicks there on `target`
    /// (a control's name, or `map`). It starts from where it is, or at `to` the first time. The
    /// probe is switched on so the planner keeps measuring its controls for the next asks.
    pub fn click_at(&mut self, to: (f32, f32), target: &str, millis: u32, now: Instant) {
        self.move_to(to, target, millis, now, MouseButton::Left);
    }

    /// The same with the right button: what opens the map's menu where it lands.
    pub fn right_click_at(&mut self, to: (f32, f32), target: &str, millis: u32, now: Instant) {
        self.move_to(to, target, millis, now, MouseButton::Right);
    }

    /// The demo over: the pointer is drawn no more once its last gesture and its ring are done.
    pub fn end(&mut self) {
        self.ended = true;
    }

    /// Whether the pointer is drawn at `now`.
    #[must_use]
    pub fn shown(&self, now: Instant) -> bool {
        self.at.is_some() && (!self.ended || self.active(now))
    }

    /// A move to `to` begun, ending in a click of `button`.
    fn move_to(
        &mut self,
        to: (f32, f32),
        target: &str,
        millis: u32,
        now: Instant,
        button: MouseButton,
    ) {
        crate::probe::enable();
        self.clicks += 1;
        self.last = Some(target.to_owned());
        let from = self.at.unwrap_or(to);
        self.gesture = Some(Gesture {
            from,
            to,
            started: now,
            took: Duration::from_millis(u64::from(millis)),
            button,
        });
        self.at = Some(from);
    }

    /// The pointer to the centre of the control the probe names `control`, and a click there;
    /// false when it is not on screen (the probe is switched on, so a later ask can find it).
    pub fn click_control(&mut self, control: &str, millis: u32, now: Instant) -> bool {
        match visible_centre(control) {
            Some(centre) => {
                self.click_at(centre, control, millis, now);
                true
            }
            None => false,
        }
    }

    /// `text`, then Enter, typed into what has the keyboard once the move under way is done.
    pub fn type_text(&mut self, text: String) {
        self.typing.push_back(text);
    }

    /// Whether a move or typing is still to come.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.gesture.is_some() || !self.typing.is_empty()
    }

    /// The clicks asked for so far, and the last one's target, for the facts.
    #[must_use]
    pub fn clicks(&self) -> (usize, Option<&str>) {
        (self.clicks, self.last.as_deref())
    }

    /// Whether anything is drawn or under way, so the window keeps painting.
    #[must_use]
    pub fn active(&self, now: Instant) -> bool {
        self.busy()
            || self
                .clicked
                .is_some_and(|(_, at)| now.saturating_duration_since(at) < RING)
    }

    /// Once a frame: the pointer moved along its gesture (eased in and out, as a hand moves), the
    /// click dispatched when it arrives, then any typing. Events go through the window after this
    /// update of the planner, whose own handlers they reach.
    pub fn tick(&mut self, now: Instant, window: &mut Window, cx: &mut Context<MissionPlanner>) {
        if let Some(gesture) = self.gesture {
            let elapsed = now.saturating_duration_since(gesture.started);
            let done = gesture.took.is_zero() || elapsed >= gesture.took;
            let t = if done {
                1.0
            } else {
                elapsed.as_secs_f32() / gesture.took.as_secs_f32()
            };
            let eased = t * t * (3.0 - 2.0 * t);
            let at = (
                (gesture.to.0 - gesture.from.0).mul_add(eased, gesture.from.0),
                (gesture.to.1 - gesture.from.1).mul_add(eased, gesture.from.1),
            );
            self.at = Some(at);
            if done {
                self.gesture = None;
                self.clicked = Some((gesture.to, now));
                let position = point(px(gesture.to.0), px(gesture.to.1));
                let button = gesture.button;
                window.defer(cx, move |window, cx| click(position, button, window, cx));
            }
            return;
        }
        if let Some(text) = self.typing.pop_front() {
            window.defer(cx, move |window, cx| type_keys(&text, window, cx));
        }
    }

    /// The cursor and a click's ring, drawn over everything, or nothing before the first move.
    #[must_use]
    pub fn cursor(&self, now: Instant) -> Option<AnyElement> {
        if !self.shown(now) {
            return None;
        }
        let (x, y) = self.at?;
        let ring = self
            .clicked
            .filter(|(_, at)| now.saturating_duration_since(*at) < RING)
            .map(|(centre, at)| {
                (
                    centre,
                    now.saturating_duration_since(at).as_secs_f32() / RING.as_secs_f32(),
                )
            });
        Some(
            div()
                .absolute()
                .top_0()
                .left_0()
                .size_full()
                .child(
                    canvas(
                        |_, _, _| (),
                        move |_bounds, (), window, _cx| paint_cursor(window, (x, y), ring),
                    )
                    .size_full(),
                )
                .into_any_element(),
        )
    }
}

/// Whether the control the probe names is on screen, some of it seen.
#[must_use]
pub fn visible(control: &str) -> bool {
    visible_centre(control).is_some()
}

/// The centre of what can be seen of a control the probe saw in its last frame: the control cut
/// to the window and the boxes that clip it, so a control half scrolled away is clicked on its
/// shown half.
fn visible_centre(control: &str) -> Option<(f32, f32)> {
    let (rect, seen) = crate::probe::placement(control)?;
    let left = rect.x.max(seen.x);
    let top = rect.y.max(seen.y);
    let right = (rect.x + rect.width).min(seen.x + seen.width);
    let bottom = (rect.y + rect.height).min(seen.y + seen.height);
    (right > left && bottom > top).then(|| ((left + right) / 2.0, (top + bottom) / 2.0))
}

/// A click of `button` at `position`: the pointer there, the button down and up, as a mouse gives
/// them.
fn click(
    position: gpui::Point<gpui::Pixels>,
    button: MouseButton,
    window: &mut Window,
    cx: &mut gpui::App,
) {
    let modifiers = Modifiers::default();
    window.dispatch_event(
        PlatformInput::MouseMove(MouseMoveEvent {
            position,
            pressed_button: None,
            modifiers,
        }),
        cx,
    );
    window.dispatch_event(
        PlatformInput::MouseDown(MouseDownEvent {
            button,
            position,
            modifiers,
            click_count: 1,
            first_mouse: false,
        }),
        cx,
    );
    window.dispatch_event(
        PlatformInput::MouseUp(MouseUpEvent {
            button,
            position,
            modifiers,
            click_count: 1,
        }),
        cx,
    );
}

/// `text` a key at a time, then Enter, into what has the keyboard.
fn type_keys(text: &str, window: &mut Window, cx: &mut gpui::App) {
    let key = |key: String, key_char: Option<String>| {
        PlatformInput::KeyDown(KeyDownEvent {
            keystroke: Keystroke {
                modifiers: Modifiers::default(),
                key,
                key_char,
            },
            is_held: false,
            prefer_character_input: false,
        })
    };
    for c in text.chars() {
        window.dispatch_event(key(c.to_string(), Some(c.to_string())), cx);
    }
    window.dispatch_event(key("enter".to_owned(), None), cx);
}

/// An arrow cursor at `tip`, white with a dark edge, and a widening ring where it last clicked.
fn paint_cursor(window: &mut Window, (x, y): (f32, f32), ring: Option<((f32, f32), f32)>) {
    if let Some(((cx, cy), progress)) = ring {
        let radius = 6.0 + 18.0 * progress;
        let mut path = PathBuilder::stroke(px(2.0));
        for step in 0..=32u8 {
            let angle = f32::from(step) / 32.0 * std::f32::consts::TAU;
            let p = point(
                px(radius.mul_add(angle.cos(), cx)),
                px(radius.mul_add(angle.sin(), cy)),
            );
            if step == 0 {
                path.move_to(p);
            } else {
                path.line_to(p);
            }
        }
        if let Ok(path) = path.build() {
            window.paint_path(
                path,
                gpui::Hsla::from(rgb(0x00_d4_ff)).opacity(1.0 - progress),
            );
        }
    }
    let arrow = [
        (0.0, 0.0),
        (0.0, 18.0),
        (4.5, 14.0),
        (7.5, 21.0),
        (10.5, 19.5),
        (7.5, 13.0),
        (13.0, 13.0),
    ];
    let shape = |builder: &mut PathBuilder, grow: f32| {
        for (i, (ax, ay)) in arrow.iter().enumerate() {
            let p = point(px(x + ax * grow), px(y + ay * grow));
            if i == 0 {
                builder.move_to(p);
            } else {
                builder.line_to(p);
            }
        }
        builder.close();
    };
    let mut edge = PathBuilder::fill();
    shape(&mut edge, 1.15);
    if let Ok(path) = edge.build() {
        window.paint_path(path, rgb(0x0d_11_17));
    }
    let mut body = PathBuilder::fill();
    shape(&mut body, 1.0);
    if let Ok(path) = body.build() {
        window.paint_path(path, rgb(0xff_ff_ff));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pointer is drawn from its first move, and once the demo says it is over it goes when
    /// its last gesture and the click's ring are done - not before.
    #[test]
    fn the_pointer_goes_when_the_demo_ends() {
        let start = Instant::now();
        let mut pointer = DemoPointer::default();
        assert!(!pointer.shown(start));
        pointer.right_click_at((10.0, 10.0), "map", 100, start);
        assert!(pointer.shown(start));
        pointer.end();
        // The gesture still under way: drawn.
        assert!(pointer.shown(start));
        // Its click made (as `tick` makes it) and the ring drawing: drawn.
        pointer.gesture = None;
        pointer.clicked = Some(((10.0, 10.0), start));
        assert!(pointer.shown(start + Duration::from_millis(100)));
        // The ring done: gone.
        assert!(!pointer.shown(start + RING + Duration::from_millis(1)));
        assert!(
            pointer
                .cursor(start + RING + Duration::from_millis(1))
                .is_none()
        );
    }
}
