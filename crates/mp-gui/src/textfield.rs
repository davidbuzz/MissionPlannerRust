//! A small text field.
//!
//! gpui ships no input element. Its `examples/input.rs` builds a full one - selection, IME, a
//! custom element that measures glyphs to place a cursor - in 784 lines. None of that is needed
//! for a search box, a file name or a typed number, and all of it would have to be maintained.
//!
//! So this is the small version, and it is honest about being small: a focusable box that takes
//! printable characters, backspace, delete-to-start, enter and escape, and draws a caret at the
//! end. No selection, no mouse positioning of the caret, no input method support. What it does
//! cover is everything this application currently needs to type, and the three screens that were
//! shaped around not having it.
//!
//! If real text editing is wanted later, gpui's example is the reference and this is the thing to
//! replace rather than to extend.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{FocusHandle, KeyDownEvent, div, prelude::*, px, rgb};

use crate::ui::theme;

/// The state of one text field.
///
/// Held by the screen that owns it rather than by the widget, so its value survives a re-render
/// and can be read without asking the view for it.
///
/// The focus handle is deliberately *not* here. A `FocusHandle` can only be made from an `App`,
/// and keeping it out means the whole of the key handling - which is where the bugs are - can be
/// tested without a running application.
#[derive(Debug)]
pub struct TextField {
    /// What has been typed.
    value: String,
    /// Shown when empty, to say what the field is for.
    placeholder: &'static str,
}

impl TextField {
    /// A field with a placeholder and no content.
    #[must_use]
    pub const fn new(placeholder: &'static str) -> Self {
        Self {
            value: String::new(),
            placeholder,
        }
    }

    /// What has been typed.
    #[must_use]
    pub fn value(&self) -> &str {
        &self.value
    }

    /// Whether anything has been typed.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.value.is_empty()
    }

    /// Replaces the contents.
    pub fn set(&mut self, value: impl Into<String>) {
        self.value = value.into();
    }

    /// Empties it.
    pub fn clear(&mut self) {
        self.value.clear();
    }

    /// Applies a keystroke. Returns what the owner should do about it.
    ///
    /// Returning an outcome rather than calling back keeps the decision with the screen: a search
    /// box filters as you type, a file name acts on enter, and the field should not have to know
    /// which it is.
    pub fn key(&mut self, event: &KeyDownEvent) -> KeyOutcome {
        let keystroke = &event.keystroke;
        match keystroke.key.as_str() {
            "backspace" => {
                // Pop a character, not a byte. Truncating mid-codepoint panics, and a search box
                // is exactly where someone pastes a name with an accent in it.
                self.value.pop();
                return KeyOutcome::Changed;
            }
            "enter" => return KeyOutcome::Submitted,
            "escape" => return KeyOutcome::Cancelled,
            _ => {}
        }

        // Control and command chords are shortcuts, not text. Without this, ctrl-c types "c".
        if keystroke.modifiers.control || keystroke.modifiers.platform {
            if keystroke.key == "u" {
                // The one chord worth handling: clear the line, as a terminal does.
                self.value.clear();
                return KeyOutcome::Changed;
            }
            return KeyOutcome::Ignored;
        }

        // key_char is the character that would have been typed, which already accounts for shift
        // and the keyboard layout. Reading `key` instead would type "a" for shift-a.
        let Some(text) = keystroke.key_char.as_ref() else {
            return KeyOutcome::Ignored;
        };
        // Control characters arrive here too; a tab or a newline in a search box is not wanted.
        if text.chars().any(char::is_control) {
            return KeyOutcome::Ignored;
        }
        self.value.push_str(text);
        KeyOutcome::Changed
    }
}

/// What a keystroke meant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyOutcome {
    /// The text changed.
    Changed,
    /// Enter was pressed.
    Submitted,
    /// Escape was pressed.
    Cancelled,
    /// Nothing this field cares about.
    Ignored,
}

/// Renders a text field.
///
/// `id` must be stable across frames, as it is for any interactive gpui element.
pub fn text_field(
    id: &'static str,
    field: &TextField,
    focus: &FocusHandle,
    focused: bool,
    width: gpui::Pixels,
    on_key: impl Fn(&KeyDownEvent, &mut gpui::Window, &mut gpui::App) + 'static,
) -> impl IntoElement {
    let empty = field.is_empty();
    let shown = if empty {
        field.placeholder.to_owned()
    } else {
        field.value.clone()
    };

    crate::probe::measured(id, div())
        .id(id)
        .track_focus(focus)
        .key_context("TextField")
        .on_key_down(move |event, window, cx| on_key(event, window, cx))
        .flex()
        .items_center()
        .gap_1()
        .w(width)
        .px_2()
        .py_1()
        .rounded_md()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::ACTION))
        .text_sm()
        .text_color(rgb(if empty { theme::DIM } else { theme::TEXT }))
        .cursor_text()
        .child(shown)
        // A caret only while focused, and always at the end, because that is the only place the
        // insertion point can be. A caret that could sit anywhere would be a promise this field
        // does not keep.
        .children(focused.then(|| div().w(px(1.0)).h(px(14.0)).bg(rgb(theme::ACCENT))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use gpui::{Keystroke, Modifiers};

    fn press(key: &str, key_char: Option<&str>) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: Keystroke {
                modifiers: Modifiers::default(),
                key: key.to_owned(),
                key_char: key_char.map(ToOwned::to_owned),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    fn chord(key: &str) -> KeyDownEvent {
        let mut event = press(key, Some(key));
        event.keystroke.modifiers.control = true;
        event
    }

    fn field() -> TextField {
        TextField::new("search")
    }

    #[test]
    fn typing_appends_what_would_have_been_typed() {
        // key_char, not key: reading `key` types "a" for shift-a.
        let mut field = field();
        assert_eq!(field.key(&press("a", Some("a"))), KeyOutcome::Changed);
        assert_eq!(field.key(&press("a", Some("A"))), KeyOutcome::Changed);
        assert_eq!(field.value(), "aA");
    }

    #[test]
    fn backspace_removes_a_character_not_a_byte() {
        // Truncating mid-codepoint panics, and a search box is exactly where someone pastes a
        // name with an accent in it.
        let mut field = field();
        field.set("café");
        assert_eq!(field.key(&press("backspace", None)), KeyOutcome::Changed);
        assert_eq!(field.value(), "caf");
    }

    #[test]
    fn backspace_on_an_empty_field_does_nothing_rather_than_panicking() {
        let mut field = field();
        assert_eq!(field.key(&press("backspace", None)), KeyOutcome::Changed);
        assert_eq!(field.value(), "");
    }

    #[test]
    fn enter_and_escape_are_reported_rather_than_typed() {
        let mut field = field();
        assert_eq!(
            field.key(&press("enter", Some("\r"))),
            KeyOutcome::Submitted
        );
        assert_eq!(field.key(&press("escape", None)), KeyOutcome::Cancelled);
        assert_eq!(field.value(), "", "neither should have typed anything");
    }

    #[test]
    fn control_chords_are_shortcuts_not_text() {
        // Without this, ctrl-c types "c" into whatever has focus.
        let mut field = field();
        assert_eq!(field.key(&chord("c")), KeyOutcome::Ignored);
        assert_eq!(field.value(), "");
    }

    #[test]
    fn control_u_clears_the_line_as_a_terminal_does() {
        let mut field = field();
        field.set("WPNAV_SPEED");
        assert_eq!(field.key(&chord("u")), KeyOutcome::Changed);
        assert_eq!(field.value(), "");
    }

    #[test]
    fn control_characters_are_not_typed() {
        // A tab or a newline in a search box is not wanted, and they arrive as key_char.
        let mut field = field();
        assert_eq!(field.key(&press("tab", Some("\t"))), KeyOutcome::Ignored);
        assert_eq!(field.value(), "");
    }

    #[test]
    fn a_key_with_no_character_is_ignored() {
        // Arrow keys, function keys, modifiers on their own.
        let mut field = field();
        assert_eq!(field.key(&press("left", None)), KeyOutcome::Ignored);
        assert_eq!(field.key(&press("f5", None)), KeyOutcome::Ignored);
        assert_eq!(field.value(), "");
    }

    #[test]
    fn clearing_and_setting_work_as_the_owner_expects() {
        let mut field = field();
        field.set("BATT");
        assert!(!field.is_empty());
        assert_eq!(field.value(), "BATT");
        field.clear();
        assert!(field.is_empty());
    }
}
