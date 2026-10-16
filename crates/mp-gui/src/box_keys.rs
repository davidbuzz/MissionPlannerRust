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

//! Enter and Escape for the shared message box ([`crate::config::servo_output::modal`]): while a
//! box shows, Enter presses its accept button and Escape its cancel button, as `CustomMessageBox`
//! makes its OK or Yes the form's `AcceptButton` and its No or Cancel the form's `CancelButton`,
//! and `MessageShowAgain` its OK and Cancel the same.
//! `// C#: ExtLibs/Controls/CustomMessageBox.cs:286-360; Common.cs:438-439`
//!
//! The owner's word of 2026-10-16, over a box that stood taller than the window: "i am unable to
//! press [enter] or click ok, as nav box is offscreen. do we need an 'always available' keypress
//! ... [Enter] button that always 'accepts' an open box". A few boxes heard the keys and most did
//! not; now every shared box does, by one rule.
//!
//! Each box, as it is drawn, records what its buttons do ([`show`], from `modal`); the record is
//! emptied as each frame starts ([`clear`]), so a box that has closed leaves no answer behind; and
//! the window's keystroke interceptor ([`intercept`]), which gpui runs before any element hears a
//! key, gives Enter and Escape to the box drawn last - the one on top - and keeps them from the
//! screen beneath, as a modal form keeps the keyboard from the form that showed it. An interceptor
//! rather than the root element's key handler: a box can open as a dialog's text box closes - the
//! Full Parameter List's "Missing N params" after Load from file - and gpui then holds the keyboard
//! on a control no longer drawn and hands every key to the window's view alone, past each
//! element's handler, the root's included.
//!
//! A box of one button, `MessageBoxButtons.OK`'s, has an `AcceptButton` and no `CancelButton`
//! (CustomMessageBox.cs:288-300; `MessageShowAgain` without Cancel, Common.cs:439): Enter presses
//! its OK, and Escape presses nothing and leaves the box showing, as a WinForms form presses its
//! cancel button on Escape only when it has one.
//!
//! The button's click and its key run one closure, an [`Answer`]: [`answer`] makes it and
//! [`click`] gives a button its listener of it, so the two cannot drift apart.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::rc::Rc;

use gpui::{AnyWindowHandle, App, Context, Keystroke, WeakEntity, Window};

use crate::MissionPlanner;

/// What a box's button does: its click's work, and its key's.
pub type Answer = Rc<dyn Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>)>;

/// An [`Answer`] of a closure, its argument types written here so a box need not write them.
pub fn answer(
    work: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
) -> Answer {
    Rc::new(work)
}

/// A box button's click: its [`Answer`] - the one its key runs - and the window redrawn. The box
/// is forgotten first, so a key pressed before the next frame cannot answer it a second time.
pub fn click(
    answer: &Answer,
    cx: &Context<MissionPlanner>,
) -> impl Fn(&(), &mut Window, &mut App) + 'static {
    let answer = Rc::clone(answer);
    cx.listener(move |this, _event: &(), window, cx| {
        OPEN.with(|open| open.borrow_mut().clear());
        answer(this, window, cx);
        cx.notify();
    })
}

/// What a box's keys press: Enter its `AcceptButton`, Escape its `CancelButton`, either absent.
#[derive(Clone)]
pub struct BoxKeys<A = Answer> {
    /// Enter's: OK, or Yes.
    accept: Option<A>,
    /// Escape's: No, or Cancel.
    cancel: Option<A>,
}

impl<A: Clone> BoxKeys<A> {
    /// `MessageBoxButtons.OK`: OK the `AcceptButton`, and no `CancelButton`, so Enter presses OK
    /// and Escape nothing. `// C#: ExtLibs/Controls/CustomMessageBox.cs:288-300`
    pub fn ok(ok: &A) -> Self {
        Self {
            accept: Some(ok.clone()),
            cancel: None,
        }
    }

    /// `MessageBoxButtons.YesNo` and `OKCancel`: Yes or OK the `AcceptButton`, No or Cancel the
    /// `CancelButton`. `// C#: ExtLibs/Controls/CustomMessageBox.cs:302-360`
    pub fn yes_no(yes: &A, no: &A) -> Self {
        Self {
            accept: Some(yes.clone()),
            cancel: Some(no.clone()),
        }
    }

    /// Cancel alone, a progress box's while it works: Escape presses it, and Enter nothing - there
    /// is nothing to accept, and a work stopped by Enter would be stopped by a key meant for
    /// something else.
    pub fn cancel(cancel: &A) -> Self {
        Self {
            accept: None,
            cancel: Some(cancel.clone()),
        }
    }
}

/// Which of a box's buttons a key presses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoxKey {
    /// Enter: the form's `AcceptButton`.
    Accept,
    /// Escape: the form's `CancelButton`.
    Cancel,
}

/// Enter or Escape, as WinForms' `Form.ProcessDialogKey` takes them: with neither Alt nor Control
/// held - nor the platform key, the third chord key here - and Shift making no difference.
pub fn box_key(keystroke: &Keystroke) -> Option<BoxKey> {
    let modifiers = &keystroke.modifiers;
    if modifiers.control || modifiers.alt || modifiers.platform {
        return None;
    }
    match keystroke.key.as_str() {
        "enter" => Some(BoxKey::Accept),
        "escape" => Some(BoxKey::Cancel),
        _ => None,
    }
}

/// A key a box took: the box's id, the key, and the button's answer - none for a key the box has
/// no button for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pressed<A> {
    /// The box's id, as `modal` was given it.
    pub id: &'static str,
    /// The key.
    pub key: BoxKey,
    /// What its button does, if the box has that button.
    pub answer: Option<A>,
}

impl<A> Pressed<A> {
    /// The fact `ui.box.key`: the box and what the key did - `accept`, `cancel`, or `nothing`.
    pub fn fact(&self) -> String {
        let did = match (self.key, self.answer.is_some()) {
            (_, false) => "nothing",
            (BoxKey::Accept, true) => "accept",
            (BoxKey::Cancel, true) => "cancel",
        };
        format!("{} {did}", self.id)
    }
}

/// The box the keys answer: the one drawn last this frame, which is the one on top.
pub struct Registry<A> {
    /// Its id and its keys.
    open: Option<(&'static str, BoxKeys<A>)>,
}

impl<A> Default for Registry<A> {
    fn default() -> Self {
        Self { open: None }
    }
}

impl<A: Clone> Registry<A> {
    /// No box: a frame starting, or a box answered.
    pub fn clear(&mut self) {
        self.open = None;
    }

    /// A box drawn, over any drawn before it this frame.
    pub fn show(&mut self, id: &'static str, keys: BoxKeys<A>) {
        self.open = Some((id, keys));
    }

    /// The box showing, by its id.
    pub fn id(&self) -> Option<&'static str> {
        self.open.as_ref().map(|(id, _)| *id)
    }

    /// `key` pressed: nothing when no box shows - the key goes on to the screen - else the box's
    /// button for it. A box answered is forgotten until it is drawn again; a key it has no button
    /// for leaves it showing.
    pub fn press(&mut self, key: BoxKey) -> Option<Pressed<A>> {
        let (id, keys) = self.open.as_ref()?;
        let id = *id;
        let answer = match key {
            BoxKey::Accept => keys.accept.clone(),
            BoxKey::Cancel => keys.cancel.clone(),
        };
        if answer.is_some() {
            self.open = None;
        }
        Some(Pressed { id, key, answer })
    }
}

thread_local! {
    /// The window's: everything here runs on gpui's main thread.
    static OPEN: RefCell<Registry<Answer>> = const { RefCell::new(Registry { open: None }) };
}

/// A frame starting: the box the last frame drew is the fact `ui.box` (`none` without one), and
/// no box answers the keys until one is drawn again. Called before anything is drawn.
pub fn clear() {
    OPEN.with(|open| {
        let mut open = open.borrow_mut();
        crate::facts::record("ui.box", open.id().unwrap_or("none"));
        open.clear();
    });
}

/// A box drawn, `id` and its keys: what Enter and Escape press until the next frame starts.
pub fn show(id: &'static str, keys: BoxKeys) {
    OPEN.with(|open| open.borrow_mut().show(id, keys));
}

/// The window's keystroke interceptor: Enter or Escape in the main window while a box shows is
/// the box's, whatever holds the keyboard - its button's answer run, or nothing for Escape on a
/// box with no cancel button - and goes no further. Any other key, or any key with no box
/// showing, goes on as it always has.
pub fn intercept(
    planner: &WeakEntity<MissionPlanner>,
    main: AnyWindowHandle,
    keystroke: &Keystroke,
    window: &mut Window,
    cx: &mut App,
) {
    if window.window_handle() != main {
        return;
    }
    let Some(key) = box_key(keystroke) else {
        return;
    };
    // Taken out before the answer runs: an answer is free to draw nothing, or a box of its own.
    let Some(pressed) = OPEN.with(|open| open.borrow_mut().press(key)) else {
        return;
    };
    cx.stop_propagation();
    crate::facts::record("ui.box.key", pressed.fact());
    match pressed.answer {
        Some(answer) => {
            planner
                .update(cx, |this, cx| {
                    answer(this, window, cx);
                    cx.notify();
                })
                .ok();
        }
        // Nothing changes, but a frame is drawn all the same, so the fact is published.
        None => window.refresh(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(text: &str) -> Keystroke {
        Keystroke::parse(text).expect("a keystroke")
    }

    fn pressed(
        id: &'static str,
        key: BoxKey,
        answer: Option<&'static str>,
    ) -> Pressed<&'static str> {
        Pressed { id, key, answer }
    }

    /// Enter and Escape are the box's, Shift or not; with Control, Alt or the platform key held,
    /// or any other key, they are not.
    #[test]
    fn enter_and_escape_with_no_chord_are_the_box_keys() {
        assert_eq!(box_key(&key("enter")), Some(BoxKey::Accept));
        assert_eq!(box_key(&key("escape")), Some(BoxKey::Cancel));
        assert_eq!(box_key(&key("shift-enter")), Some(BoxKey::Accept));
        assert_eq!(box_key(&key("shift-escape")), Some(BoxKey::Cancel));
        for text in [
            "ctrl-enter",
            "alt-enter",
            "cmd-enter",
            "ctrl-escape",
            "alt-escape",
            "space",
            "tab",
            "a",
            "f12",
        ] {
            assert_eq!(box_key(&key(text)), None, "{text}");
        }
    }

    /// No box showing: neither key is taken, so it reaches the screen as before.
    #[test]
    fn with_no_box_no_key_is_taken() {
        let mut open = Registry::<&str>::default();
        assert_eq!(open.id(), None);
        assert_eq!(open.press(BoxKey::Accept), None);
        assert_eq!(open.press(BoxKey::Cancel), None);
    }

    /// Yes and No: Enter is Yes, Escape is No, and a box answered answers nothing more until it
    /// is drawn again - a second Enter before the next frame is the screen's.
    #[test]
    fn a_two_button_box_answers_enter_with_yes_and_escape_with_no() {
        let mut open = Registry::default();
        open.show("question", BoxKeys::yes_no(&"yes", &"no"));
        assert_eq!(open.id(), Some("question"));
        assert_eq!(
            open.press(BoxKey::Accept),
            Some(pressed("question", BoxKey::Accept, Some("yes")))
        );
        assert_eq!(open.id(), None);
        assert_eq!(open.press(BoxKey::Accept), None);

        open.show("question", BoxKeys::yes_no(&"yes", &"no"));
        assert_eq!(
            open.press(BoxKey::Cancel),
            Some(pressed("question", BoxKey::Cancel, Some("no")))
        );
        assert_eq!(open.press(BoxKey::Cancel), None);
    }

    /// OK alone, as `MessageBoxButtons.OK` has it: Enter presses OK; Escape is taken - the
    /// screen beneath does not hear it - presses nothing, and the box stays to be answered.
    #[test]
    fn a_one_button_box_answers_enter_and_keeps_escape() {
        let mut open = Registry::default();
        open.show("message", BoxKeys::ok(&"ok"));
        assert_eq!(
            open.press(BoxKey::Cancel),
            Some(pressed("message", BoxKey::Cancel, None))
        );
        assert_eq!(open.id(), Some("message"));
        assert_eq!(
            open.press(BoxKey::Accept),
            Some(pressed("message", BoxKey::Accept, Some("ok")))
        );
        assert_eq!(open.id(), None);
    }

    /// Cancel alone, a progress box's: Escape presses it; Enter is taken and presses nothing.
    #[test]
    fn a_cancel_box_answers_escape_and_keeps_enter() {
        let mut open = Registry::default();
        open.show("progress", BoxKeys::cancel(&"cancel"));
        assert_eq!(
            open.press(BoxKey::Accept),
            Some(pressed("progress", BoxKey::Accept, None))
        );
        assert_eq!(open.id(), Some("progress"));
        assert_eq!(
            open.press(BoxKey::Cancel),
            Some(pressed("progress", BoxKey::Cancel, Some("cancel")))
        );
        assert_eq!(open.id(), None);
    }

    /// A box with a button for neither key still keeps them from the screen beneath, as any
    /// modal form does.
    #[test]
    fn a_box_without_keys_still_takes_them() {
        let mut open = Registry::<&str>::default();
        let keys = BoxKeys {
            accept: None,
            cancel: None,
        };
        open.show("plain", keys);
        assert_eq!(
            open.press(BoxKey::Accept),
            Some(pressed("plain", BoxKey::Accept, None))
        );
        assert_eq!(
            open.press(BoxKey::Cancel),
            Some(pressed("plain", BoxKey::Cancel, None))
        );
        assert_eq!(open.id(), Some("plain"));
    }

    /// Two boxes drawn in one frame: the later is on top, and the keys are its.
    #[test]
    fn the_box_drawn_last_takes_the_keys() {
        let mut open = Registry::default();
        open.show("under", BoxKeys::yes_no(&"under-yes", &"under-no"));
        open.show("over", BoxKeys::ok(&"over-ok"));
        assert_eq!(open.id(), Some("over"));
        assert_eq!(
            open.press(BoxKey::Accept),
            Some(pressed("over", BoxKey::Accept, Some("over-ok")))
        );
    }

    /// A frame that draws no box leaves none behind: a box closed by its click, or by its page
    /// going, answers no key after.
    #[test]
    fn a_frame_without_the_box_clears_it() {
        let mut open = Registry::default();
        open.show("message", BoxKeys::ok(&"ok"));
        open.clear();
        assert_eq!(open.id(), None);
        assert_eq!(open.press(BoxKey::Accept), None);
    }

    /// The fact says which box took the key and what it pressed.
    #[test]
    fn the_fact_names_the_box_and_what_the_key_did() {
        assert_eq!(
            pressed("param-box", BoxKey::Accept, Some("ok")).fact(),
            "param-box accept"
        );
        assert_eq!(
            pressed("param-box", BoxKey::Cancel, Some("no")).fact(),
            "param-box cancel"
        );
        assert_eq!(
            pressed("param-box", BoxKey::Cancel, None).fact(),
            "param-box nothing"
        );
    }

    /// The window's record, as `modal` and the frame's start use it: a box shown is the one the
    /// keys answer, and the next frame's start empties it - with the answers of real boxes, which
    /// are never run here.
    #[test]
    fn the_windows_record_holds_a_box_until_the_next_frame() {
        let open_box = || OPEN.with(|open| open.borrow().id());
        clear();
        assert_eq!(open_box(), None);
        let ok = answer(|_this, _window, _cx| {});
        let no = answer(|_this, _window, _cx| {});
        show("first", BoxKeys::ok(&ok));
        show("second", BoxKeys::yes_no(&ok, &no));
        assert_eq!(open_box(), Some("second"));
        // Enter: the second box's Yes, taken out, so the record holds no box after.
        let pressed = OPEN.with(|open| open.borrow_mut().press(BoxKey::Accept));
        assert!(pressed.is_some_and(|pressed| pressed.id == "second" && pressed.answer.is_some()));
        assert_eq!(open_box(), None);
        show("second", BoxKeys::yes_no(&ok, &no));
        clear();
        assert_eq!(open_box(), None);
    }
}
