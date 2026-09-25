//! A small text field, with the caret, selection and clipboard of WinForms' `TextBox`.
//!
//! gpui ships no input element. Its `examples/input.rs` builds a full one - IME, a custom element
//! that measures glyphs - in 784 lines. This is smaller: a focusable box whose key handling is
//! plain data in, outcome out, so every path is testable without a window, and whose drawing
//! leans on gpui's own `StyledText` for everything that needs measuring.
//!
//! What it does, as Mission Planner's boxes do it (the framework's `TextBox`, not anything the C#
//! writes): a caret that Left, Right, Home and End move, Ctrl with the arrows by words and with
//! Home and End to either end; Shift with any of those extending a selection; Ctrl+A selecting
//! all; typing replacing the selection; Backspace and Delete taking the selection or one
//! character; Ctrl+C, Ctrl+X and Ctrl+V (and Ctrl+Insert, Shift+Delete, Shift+Insert) through the
//! system clipboard; a click putting the caret at the character clicked, a drag or a
//! Shift+click selecting, a double click selecting a word. A multi-line box also takes Up and
//! Down between lines and Enter as a new line, where a single-line box reports Enter to its owner.
//!
//! What it does not: undo, input methods, a blinking caret, scrolling sideways to keep the caret
//! in view, or the right-click menu.
//!
//! **A box that is never drawn by [`text_field`] or [`text_area`] keeps the old behaviour** -
//! typing at the end, arrows ignored - because a page that draws its own box draws no caret, and a
//! caret that moves where nobody can see it would put typing in the middle of the text unseen.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, OnceLock};

use gpui::{
    Bounds, ClipboardItem, FocusHandle, HighlightStyle, KeyDownEvent, Keystroke, MouseButton,
    MouseDownEvent, MouseMoveEvent, SharedString, StyledText, TextLayout, canvas, div, fill,
    prelude::*, px, rgb, size,
};

use crate::ui::theme;

/// What a multi-line box puts in for Enter: `TextBox.Text`'s line break.
const NEW_LINE: &str = "\r\n";

/// Where the caret and the selection's other end are, and whether the box is being drawn.
///
/// Shared with the drawn box's mouse handlers, which run later and hold no borrow of the owner's
/// state; the text itself stays with the owner. Atomics rather than cells only so that a page
/// holding a field stays `Send`.
#[derive(Debug, Default)]
struct Cursor {
    /// The caret, as a character index.
    caret: AtomicUsize,
    /// The selection's fixed end: equal to the caret when nothing is selected.
    anchor: AtomicUsize,
    /// Whether [`text_field`] or [`text_area`] has drawn this box, so that a caret shows.
    drawn: AtomicBool,
    /// A left button pressed in the box and not yet released: a move selects.
    dragging: AtomicBool,
}

impl Cursor {
    fn get(&self) -> (usize, usize) {
        (
            self.anchor.load(Ordering::Relaxed),
            self.caret.load(Ordering::Relaxed),
        )
    }

    fn put(&self, anchor: usize, caret: usize) {
        self.anchor.store(anchor, Ordering::Relaxed);
        self.caret.store(caret, Ordering::Relaxed);
    }

    /// The left button down at `index` of `chars`, `count` being the click count.
    fn click(&self, chars: &[char], index: usize, extend: bool, count: usize) {
        let index = settle(chars, index);
        if count >= 2 {
            let (start, end) = word_at(chars, index);
            self.put(start, end);
            self.dragging.store(false, Ordering::Relaxed);
        } else {
            let (anchor, _) = self.get();
            self.put(if extend { anchor } else { index }, index);
            self.dragging.store(true, Ordering::Relaxed);
        }
    }

    /// The pointer moved to `index` with the button still down.
    fn drag(&self, chars: &[char], index: usize) -> bool {
        if !self.dragging.load(Ordering::Relaxed) {
            return false;
        }
        let (anchor, _) = self.get();
        self.put(anchor, settle(chars, index));
        true
    }

    fn release(&self) {
        self.dragging.store(false, Ordering::Relaxed);
    }
}

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
    /// `TextBox.Multiline`: Enter is a new line, Up and Down move between lines.
    multiline: bool,
    /// The caret and the selection, shared with the drawn box; made on first use, so that
    /// [`TextField::new`] stays `const`.
    cursor: OnceLock<Arc<Cursor>>,
}

impl TextField {
    /// A field with a placeholder and no content.
    #[must_use]
    pub const fn new(placeholder: &'static str) -> Self {
        Self {
            value: String::new(),
            placeholder,
            multiline: false,
            cursor: OnceLock::new(),
        }
    }

    fn cursor(&self) -> &Arc<Cursor> {
        self.cursor.get_or_init(Arc::default)
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

    /// Replaces the contents, the caret after them. The same text again leaves the caret and the
    /// selection where they are, so a page that sets its box from its model every frame does not
    /// throw the caret to the end each time.
    pub fn set(&mut self, value: impl Into<String>) {
        let value = value.into();
        if value == self.value {
            return;
        }
        self.value = value;
        let end = self.len();
        self.cursor().put(end, end);
    }

    /// Empties it.
    pub fn clear(&mut self) {
        self.value.clear();
        self.cursor().put(0, 0);
    }

    /// `TextBox.Multiline`.
    pub fn set_multiline(&mut self, multiline: bool) {
        self.multiline = multiline;
    }

    /// Whether Enter is a new line.
    #[cfg(test)]
    #[must_use]
    pub const fn is_multiline(&self) -> bool {
        self.multiline
    }

    /// Gives the box its caret: done by [`text_field`] and [`text_area`] as they draw it, and by
    /// a test that wants the drawn box's keys. Until then the box types at its end only.
    pub fn show_caret(&self) {
        self.cursor().drawn.store(true, Ordering::Relaxed);
    }

    fn caret_shown(&self) -> bool {
        self.cursor().drawn.load(Ordering::Relaxed)
    }

    /// Characters, not bytes: every index this field hands out or takes is a character index.
    fn len(&self) -> usize {
        self.value.chars().count()
    }

    fn chars(&self) -> Vec<char> {
        self.value.chars().collect()
    }

    /// The caret, as a character index. At the end in a box that has never been drawn.
    #[must_use]
    pub fn caret(&self) -> usize {
        self.ends().1
    }

    /// The anchor and the caret, clamped to the text.
    fn ends(&self) -> (usize, usize) {
        let len = self.len();
        if !self.caret_shown() {
            return (len, len);
        }
        let (anchor, caret) = self.cursor().get();
        (anchor.min(len), caret.min(len))
    }

    /// The selection as `(start, end)` character indices, start first; `None` when nothing is.
    #[must_use]
    pub fn selection(&self) -> Option<(usize, usize)> {
        let (anchor, caret) = self.ends();
        (anchor != caret).then(|| (anchor.min(caret), anchor.max(caret)))
    }

    /// `TextBox.SelectedText`.
    #[must_use]
    pub fn selected_text(&self) -> &str {
        self.selection().map_or("", |(start, end)| {
            self.value
                .get(self.byte_at(start)..self.byte_at(end))
                .unwrap_or("")
        })
    }

    /// The selection as a fact: `start,end` in characters, or `none`.
    #[must_use]
    pub fn selection_fact(&self) -> String {
        self.selection().map_or_else(
            || "none".to_owned(),
            |(start, end)| format!("{start},{end}"),
        )
    }

    /// `TextBox.Select(start, length)`, as the two ends: the caret goes to `caret`.
    #[cfg(test)]
    pub fn select(&mut self, anchor: usize, caret: usize) {
        let len = self.len();
        self.cursor().put(anchor.min(len), caret.min(len));
    }

    /// Puts `text` where the caret is, over the selection if there is one, the caret after it:
    /// what typing and pasting do.
    pub fn insert(&mut self, text: &str) {
        let (anchor, caret) = self.ends();
        self.replace(anchor.min(caret), anchor.max(caret), text);
    }

    /// The byte offset of character `index`, or the end.
    fn byte_at(&self, index: usize) -> usize {
        self.value
            .char_indices()
            .nth(index)
            .map_or(self.value.len(), |(byte, _)| byte)
    }

    /// Replaces characters `start..end` with `text`, the caret after it.
    fn replace(&mut self, start: usize, end: usize, text: &str) {
        let (from, to) = (self.byte_at(start), self.byte_at(end));
        self.value.replace_range(from..to, text);
        let caret = start + text.chars().count();
        self.cursor().put(caret, caret);
    }

    /// Moves the caret to `target`, the anchor with it unless the selection is being extended.
    fn move_to(&self, target: usize, extend: bool) {
        let (anchor, _) = self.ends();
        self.cursor()
            .put(if extend { anchor } else { target }, target);
    }

    /// A click in the drawn box at character `index`, `count` being the click count: the caret
    /// there, or with Shift the selection extended to there, or on a double click the word.
    #[cfg(test)]
    pub fn click(&mut self, index: usize, extend: bool, count: usize) {
        self.cursor().click(&self.chars(), index, extend, count);
    }

    /// Applies a keystroke. Returns what the owner should do about it.
    ///
    /// Returning an outcome rather than calling back keeps the decision with the screen: a search
    /// box filters as you type, a file name acts on enter, and the field should not have to know
    /// which it is. A key that only moves the caret or the selection, or copies, is `Ignored` - the
    /// text is as it was - and the drawn box redraws itself and keeps the key from its parents.
    ///
    /// The clipboard is the system's, handed over by the drawn box around this call; the tests
    /// hand `key_with` one of their own.
    pub fn key(&mut self, event: &KeyDownEvent) -> KeyOutcome {
        let (outcome, consumed) = self.apply(event, &mut SystemClipboard);
        if consumed {
            EXCHANGE.with(|exchange| exchange.borrow_mut().consumed = true);
        }
        outcome
    }

    /// [`TextField::key`] with a clipboard of the caller's.
    #[cfg(test)]
    pub fn key_with(&mut self, event: &KeyDownEvent, clipboard: &mut dyn Clipboard) -> KeyOutcome {
        self.apply(event, clipboard).0
    }

    /// The outcome, and whether the key was the field's own - moved, selected or clipboard - and
    /// so is kept from the box's parents. Typing, Backspace, Enter, Escape and Ctrl+U reach them
    /// as they always have.
    fn apply(&mut self, event: &KeyDownEvent, clipboard: &mut dyn Clipboard) -> (KeyOutcome, bool) {
        let keystroke = &event.keystroke;
        if !self.caret_shown() {
            return (self.type_at_end(keystroke), false);
        }
        let modifiers = &keystroke.modifiers;
        let chord = modifiers.control || modifiers.platform;
        let shift = modifiers.shift;
        let chars = self.chars();
        let caret = self.caret();
        let selection = self.selection();
        match keystroke.key.as_str() {
            "backspace" => {
                // Any modifier: Backspace has always taken a character here, Ctrl or not.
                match selection {
                    Some((start, end)) => self.replace(start, end, ""),
                    None if caret > 0 => self.replace(previous(&chars, caret), caret, ""),
                    None => {}
                }
                return (KeyOutcome::Changed, false);
            }
            "delete" if shift && !chord => return (self.cut(clipboard), true),
            "delete" => {
                match selection {
                    Some((start, end)) => self.replace(start, end, ""),
                    None if caret < chars.len() => self.replace(caret, next(&chars, caret), ""),
                    None => return (KeyOutcome::Ignored, true),
                }
                return (KeyOutcome::Changed, true);
            }
            "enter" if self.multiline && !chord => {
                self.insert(NEW_LINE);
                return (KeyOutcome::Changed, true);
            }
            "enter" => return (KeyOutcome::Submitted, false),
            "escape" => return (KeyOutcome::Cancelled, false),
            "left" => {
                let target = match selection {
                    Some((start, _)) if !shift && !chord => start,
                    _ if chord => word_left(&chars, caret),
                    _ => previous(&chars, caret),
                };
                self.move_to(target, shift);
                return (KeyOutcome::Ignored, true);
            }
            "right" => {
                let target = match selection {
                    Some((_, end)) if !shift && !chord => end,
                    _ if chord => word_right(&chars, caret),
                    _ => next(&chars, caret),
                };
                self.move_to(target, shift);
                return (KeyOutcome::Ignored, true);
            }
            "home" => {
                let target = if chord { 0 } else { line_start(&chars, caret) };
                self.move_to(target, shift);
                return (KeyOutcome::Ignored, true);
            }
            "end" => {
                let target = if chord {
                    chars.len()
                } else {
                    line_end(&chars, caret)
                };
                self.move_to(target, shift);
                return (KeyOutcome::Ignored, true);
            }
            "up" if self.multiline => {
                self.move_to(line_above(&chars, caret), shift);
                return (KeyOutcome::Ignored, true);
            }
            "down" if self.multiline => {
                self.move_to(line_below(&chars, caret), shift);
                return (KeyOutcome::Ignored, true);
            }
            "insert" if chord && !shift => return (self.copy(clipboard), true),
            "insert" if shift && !chord => return (self.paste(clipboard), true),
            _ => {}
        }

        // Control and command chords are shortcuts, not text. Without this, ctrl-c types "c".
        if chord {
            return match keystroke.key.to_ascii_lowercase().as_str() {
                "a" => {
                    self.cursor().put(0, chars.len());
                    (KeyOutcome::Ignored, true)
                }
                "c" => (self.copy(clipboard), true),
                "x" => (self.cut(clipboard), true),
                "v" => (self.paste(clipboard), true),
                // Not the framework's: clear the line, as a terminal does. The scripts use it.
                "u" => {
                    self.clear();
                    (KeyOutcome::Changed, false)
                }
                _ => (KeyOutcome::Ignored, false),
            };
        }
        match typed(keystroke) {
            Some(text) => {
                self.insert(text);
                (KeyOutcome::Changed, false)
            }
            None => (KeyOutcome::Ignored, false),
        }
    }

    /// The box as it was before it had a caret: typing at the end, Backspace, Enter, Escape and
    /// Ctrl+U, nothing else.
    fn type_at_end(&mut self, keystroke: &Keystroke) -> KeyOutcome {
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
        if keystroke.modifiers.control || keystroke.modifiers.platform {
            if keystroke.key == "u" {
                self.value.clear();
                return KeyOutcome::Changed;
            }
            return KeyOutcome::Ignored;
        }
        match typed(keystroke) {
            Some(text) => {
                self.value.push_str(text);
                KeyOutcome::Changed
            }
            None => KeyOutcome::Ignored,
        }
    }

    /// `TextBox.Copy`: the selection to the clipboard; nothing selected, nothing copied.
    fn copy(&self, clipboard: &mut dyn Clipboard) -> KeyOutcome {
        if self.selection().is_some() {
            clipboard.write(self.selected_text().to_owned());
        }
        KeyOutcome::Ignored
    }

    /// `TextBox.Cut`: copied, then removed.
    fn cut(&mut self, clipboard: &mut dyn Clipboard) -> KeyOutcome {
        let Some((start, end)) = self.selection() else {
            return KeyOutcome::Ignored;
        };
        clipboard.write(self.selected_text().to_owned());
        self.replace(start, end, "");
        KeyOutcome::Changed
    }

    /// `TextBox.Paste`: over the selection, at the caret. A single-line box takes the first line
    /// of what is pasted, as the Win32 edit control does; a multi-line one takes every line.
    fn paste(&mut self, clipboard: &mut dyn Clipboard) -> KeyOutcome {
        let Some(text) = clipboard.read() else {
            return KeyOutcome::Ignored;
        };
        let text = self.pasteable(&text);
        if text.is_empty() && self.selection().is_none() {
            return KeyOutcome::Ignored;
        }
        self.insert(&text);
        KeyOutcome::Changed
    }

    fn pasteable(&self, text: &str) -> String {
        if self.multiline {
            let lines = text.replace("\r\n", "\n").replace('\r', "\n");
            let mut clean = String::with_capacity(lines.len());
            for character in lines.chars() {
                match character {
                    '\n' => clean.push_str(NEW_LINE),
                    '\t' => clean.push('\t'),
                    other if other.is_control() => {}
                    other => clean.push(other),
                }
            }
            clean
        } else {
            text.split(['\r', '\n'])
                .next()
                .unwrap_or("")
                .chars()
                .filter(|character| !character.is_control())
                .collect()
        }
    }
}

/// What a keystroke types: `key_char`, which already accounts for shift and the keyboard layout
/// (reading `key` instead would type "a" for shift-a), unless it is a control character - a tab or
/// a newline arrives here too and is not wanted.
fn typed(keystroke: &Keystroke) -> Option<&str> {
    keystroke
        .key_char
        .as_deref()
        .filter(|text| !text.chars().any(char::is_control))
}

/// A clipboard: the system's for the drawn box, a fake for the tests.
pub trait Clipboard {
    /// The text on it, if there is text.
    fn read(&mut self) -> Option<String>;
    /// Puts text on it.
    fn write(&mut self, text: String);
}

/// What passes between the drawn box's key handler, which has the application and so the
/// system clipboard, and [`TextField::key`], called from the owner's handler inside it, which has
/// the field and not the application. One key at a time, on the one UI thread.
#[derive(Debug, Default)]
struct Exchange {
    /// The clipboard's text, read before a paste chord reaches the owner.
    paste: Option<String>,
    /// Text copied or cut, written to the clipboard after the owner returns.
    copied: Option<String>,
    /// The key was the field's own: kept from the box's parents, and the box redrawn.
    consumed: bool,
}

thread_local! {
    static EXCHANGE: RefCell<Exchange> = RefCell::new(Exchange::default());
}

/// The system clipboard, through the [`Exchange`].
struct SystemClipboard;

impl Clipboard for SystemClipboard {
    fn read(&mut self) -> Option<String> {
        EXCHANGE.with(|exchange| exchange.borrow().paste.clone())
    }

    fn write(&mut self, text: String) {
        EXCHANGE.with(|exchange| exchange.borrow_mut().copied = Some(text));
    }
}

/// Whether a keystroke pastes: Ctrl+V, or Shift+Insert.
fn is_paste(keystroke: &Keystroke) -> bool {
    let modifiers = &keystroke.modifiers;
    let chord = modifiers.control || modifiers.platform;
    (chord && keystroke.key.eq_ignore_ascii_case("v"))
        || (modifiers.shift && !chord && keystroke.key == "insert")
}

// --- Positions ------------------------------------------------------------------------------------
//
// All in characters. A line break is `\n`, or `\r\n` taken as one: the caret never stops between
// the two, and Backspace or Delete takes both.

/// Not between the `\r` and the `\n` of a line break.
fn settle(chars: &[char], index: usize) -> usize {
    let index = index.min(chars.len());
    if index > 0 && chars.get(index - 1) == Some(&'\r') && chars.get(index) == Some(&'\n') {
        index - 1
    } else {
        index
    }
}

/// One character, or one line break, to the left.
fn previous(chars: &[char], index: usize) -> usize {
    match index {
        0 => 0,
        _ if index >= 2
            && chars.get(index - 1) == Some(&'\n')
            && chars.get(index - 2) == Some(&'\r') =>
        {
            index - 2
        }
        _ => index - 1,
    }
}

/// One character, or one line break, to the right.
fn next(chars: &[char], index: usize) -> usize {
    if index >= chars.len() {
        chars.len()
    } else if chars.get(index) == Some(&'\r') && chars.get(index + 1) == Some(&'\n') {
        index + 2
    } else {
        index + 1
    }
}

/// The start of the line `index` is on.
fn line_start(chars: &[char], index: usize) -> usize {
    let mut start = index.min(chars.len());
    while start > 0 && chars.get(start - 1) != Some(&'\n') {
        start -= 1;
    }
    start
}

/// The end of the line `index` is on, before its line break.
fn line_end(chars: &[char], index: usize) -> usize {
    let mut end = index.min(chars.len());
    while end < chars.len() && chars.get(end) != Some(&'\n') {
        end += 1;
    }
    if end > 0 && chars.get(end) == Some(&'\n') && chars.get(end - 1) == Some(&'\r') {
        end -= 1;
    }
    end
}

/// The same column on the line above, or the line's end if it is shorter; on the first line,
/// where the caret is.
fn line_above(chars: &[char], index: usize) -> usize {
    let start = line_start(chars, index);
    if start == 0 {
        return index;
    }
    let column = index - start;
    let above_end = line_end(chars, start - 1);
    let above_start = line_start(chars, above_end);
    (above_start + column).min(above_end)
}

/// The same column on the line below, or its end if it is shorter; on the last line, where the
/// caret is.
fn line_below(chars: &[char], index: usize) -> usize {
    let start = line_start(chars, index);
    let end = line_end(chars, index);
    if end >= chars.len() {
        return index;
    }
    let column = index - start;
    let below_start = next(chars, end);
    let below_end = line_end(chars, below_start);
    (below_start + column).min(below_end)
}

/// Spaces, a word's letters, digits and underscores, or anything else: what a word is made of.
fn class(character: char) -> u8 {
    if character.is_whitespace() {
        0
    } else if character.is_alphanumeric() || character == '_' {
        1
    } else {
        2
    }
}

fn class_at(chars: &[char], index: usize) -> Option<u8> {
    chars.get(index).copied().map(class)
}

/// Ctrl+Right: past the word the caret is in, and the spaces after it.
fn word_right(chars: &[char], index: usize) -> usize {
    let mut at = index.min(chars.len());
    if let Some(kind) = class_at(chars, at).filter(|kind| *kind != 0) {
        while class_at(chars, at) == Some(kind) {
            at += 1;
        }
    }
    while class_at(chars, at) == Some(0) {
        at += 1;
    }
    at
}

/// Ctrl+Left: back over spaces, then to the start of the word before them.
fn word_left(chars: &[char], index: usize) -> usize {
    let mut at = index.min(chars.len());
    while at > 0 && class_at(chars, at - 1) == Some(0) {
        at -= 1;
    }
    if at > 0 {
        let kind = class_at(chars, at - 1);
        while at > 0 && class_at(chars, at - 1) == kind {
            at -= 1;
        }
    }
    at
}

/// The word a double click at `index` selects: the run of like characters it falls in.
fn word_at(chars: &[char], index: usize) -> (usize, usize) {
    if chars.is_empty() {
        return (0, 0);
    }
    let at = index.min(chars.len() - 1);
    let kind = class_at(chars, at);
    let mut start = at;
    while start > 0 && class_at(chars, start - 1) == kind {
        start -= 1;
    }
    let mut end = at + 1;
    while end < chars.len() && class_at(chars, end) == kind {
        end += 1;
    }
    (start, end)
}

/// The character a byte offset of the text falls in.
fn char_index(chars: &[char], byte: usize) -> usize {
    let mut bytes = 0;
    for (index, character) in chars.iter().enumerate() {
        if bytes >= byte {
            return index;
        }
        bytes += character.len_utf8();
    }
    chars.len()
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

// --- Drawing -------------------------------------------------------------------------------------

/// Renders a single-line text field.
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
    field_box(id, field, focus, focused, on_key)
        .flex()
        .items_center()
        .w(width)
        .px_2()
        .py_1()
        .rounded_md()
        .text_sm()
}

/// Renders a multi-line text box of a fixed size, its lines wrapped at its width as `WordWrap`
/// wraps them. The field should be [`TextField::set_multiline`].
pub fn text_area(
    id: &'static str,
    field: &TextField,
    focus: &FocusHandle,
    focused: bool,
    width: impl Into<gpui::Length> + Clone,
    height: gpui::Pixels,
    on_key: impl Fn(&KeyDownEvent, &mut gpui::Window, &mut gpui::App) + 'static,
) -> impl IntoElement {
    field_box(id, field, focus, focused, on_key)
        .flex()
        .flex_col()
        .w(width)
        .h(height)
        .px_1()
        .overflow_hidden()
        .rounded_sm()
        .text_xs()
}

/// The box both draw: the text with its selection highlighted, the caret, and the handlers.
///
/// The text is gpui's `StyledText`, and its `TextLayout` - the glyphs as laid out and painted - is
/// what places the caret and maps a click to the character under it; no width is guessed.
fn field_box(
    id: &'static str,
    field: &TextField,
    focus: &FocusHandle,
    focused: bool,
    on_key: impl Fn(&KeyDownEvent, &mut gpui::Window, &mut gpui::App) + 'static,
) -> gpui::Stateful<gpui::Div> {
    field.show_caret();
    let empty = field.is_empty();
    // A line break's `\r` and a tab are drawn as spaces: one byte for one byte, so the layout's
    // offsets stay the text's.
    let shown: SharedString = if empty {
        field.placeholder.into()
    } else {
        field.value.replace(['\r', '\t'], " ").into()
    };
    let mut text = StyledText::new(shown);
    if !empty && let Some((start, end)) = field.selection() {
        text = text.with_highlights([(
            field.byte_at(start)..field.byte_at(end),
            HighlightStyle {
                background_color: Some(rgb(theme::SELECTION).into()),
                ..HighlightStyle::default()
            },
        )]);
    }
    let layout = text.layout().clone();
    let chars: Arc<[char]> = if empty {
        Arc::from(Vec::new())
    } else {
        field.value.chars().collect()
    };
    let caret_byte = if empty {
        0
    } else {
        field.byte_at(field.caret())
    };
    let cursor = Arc::clone(field.cursor());

    let down = {
        let (layout, chars, cursor) = (layout.clone(), Arc::clone(&chars), Arc::clone(&cursor));
        move |event: &MouseDownEvent, window: &mut gpui::Window, _cx: &mut gpui::App| {
            let index = pointed(&layout, &chars, event.position);
            cursor.click(&chars, index, event.modifiers.shift, event.click_count);
            window.refresh();
        }
    };
    let moved = {
        let (layout, chars, cursor) = (layout.clone(), Arc::clone(&chars), Arc::clone(&cursor));
        move |event: &MouseMoveEvent, window: &mut gpui::Window, _cx: &mut gpui::App| {
            if event.pressed_button == Some(MouseButton::Left)
                && cursor.drag(&chars, pointed(&layout, &chars, event.position))
            {
                window.refresh();
            } else if event.pressed_button.is_none() {
                cursor.release();
            }
        }
    };
    let released = Arc::clone(&cursor);
    let released_out = Arc::clone(&cursor);

    crate::probe::measured(id, div())
        .id(id)
        .track_focus(focus)
        .key_context("TextField")
        .on_key_down(move |event, window, cx| {
            // The owner's handler has the field and not the clipboard; this has the clipboard.
            let paste = is_paste(&event.keystroke)
                .then(|| cx.read_from_clipboard().and_then(|item| item.text()))
                .flatten();
            EXCHANGE.with(|exchange| {
                *exchange.borrow_mut() = Exchange {
                    paste,
                    ..Exchange::default()
                };
            });
            on_key(event, window, cx);
            let done = EXCHANGE.with(|exchange| std::mem::take(&mut *exchange.borrow_mut()));
            if let Some(text) = done.copied {
                cx.write_to_clipboard(ClipboardItem::new_string(text));
            }
            if done.consumed {
                // A caret move changes no text, and an owner that redraws only on a change
                // would leave the caret drawn where it was.
                cx.stop_propagation();
                window.refresh();
            }
        })
        .on_mouse_down(MouseButton::Left, down)
        .on_mouse_move(moved)
        .on_mouse_up(MouseButton::Left, move |_event, _window, _cx| {
            released.release()
        })
        .on_mouse_up_out(MouseButton::Left, move |_event, _window, _cx| {
            released_out.release();
        })
        .relative()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::ACTION))
        .text_color(rgb(if empty { theme::DIM } else { theme::TEXT }))
        .cursor_text()
        .child(text)
        // A caret only while focused, where the layout put the character after it.
        .children(focused.then(|| {
            canvas(
                |_bounds, _window, _cx| {},
                move |_bounds, (), window, _cx| {
                    if let Some(origin) = layout.position_for_index(caret_byte) {
                        window.paint_quad(fill(
                            Bounds::new(origin, size(px(1.0), layout.line_height())),
                            rgb(theme::ACCENT),
                        ));
                    }
                },
            )
            .absolute()
            .top_0()
            .left_0()
            .size_full()
        }))
}

/// The character under a point of the window, by the text's own layout.
fn pointed(layout: &TextLayout, chars: &[char], position: gpui::Point<gpui::Pixels>) -> usize {
    if chars.is_empty() {
        return 0;
    }
    let byte = layout
        .index_for_position(position)
        .unwrap_or_else(|nearest| nearest);
    char_index(chars, byte)
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

    fn shifted(key: &str) -> KeyDownEvent {
        let mut event = press(key, None);
        event.keystroke.modifiers.shift = true;
        event
    }

    fn control(key: &str) -> KeyDownEvent {
        let mut event = press(key, None);
        event.keystroke.modifiers.control = true;
        event
    }

    fn control_shift(key: &str) -> KeyDownEvent {
        let mut event = control(key);
        event.keystroke.modifiers.shift = true;
        event
    }

    fn field() -> TextField {
        TextField::new("search")
    }

    /// A field as the drawn box has it: with its caret.
    fn drawn(text: &str) -> TextField {
        let mut field = field();
        field.set(text);
        field.show_caret();
        field
    }

    fn area(text: &str) -> TextField {
        let mut field = drawn("");
        field.set_multiline(true);
        field.set(text);
        field
    }

    /// A clipboard in memory, so the tests need no window.
    #[derive(Default)]
    struct Fake(Option<String>);

    impl Clipboard for Fake {
        fn read(&mut self) -> Option<String> {
            self.0.clone()
        }
        fn write(&mut self, text: String) {
            self.0 = Some(text);
        }
    }

    fn keys(field: &mut TextField, events: &[KeyDownEvent]) {
        for event in events {
            field.key_with(event, &mut Fake::default());
        }
    }

    // --- The box as it has always been --------------------------------------------------------

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

    #[test]
    fn a_box_never_drawn_keeps_its_caret_at_the_end() {
        // A page that draws its own box draws no caret; Left there must not move one unseen.
        let mut field = field();
        field.set("abc");
        assert_eq!(field.key(&press("left", None)), KeyOutcome::Ignored);
        assert_eq!(field.key(&press("x", Some("x"))), KeyOutcome::Changed);
        assert_eq!(field.value(), "abcx");
        assert_eq!(field.caret(), 4);
        assert_eq!(field.selection(), None);
        assert_eq!(field.key(&chord("a")), KeyOutcome::Ignored);
        assert_eq!(field.selection(), None);
    }

    // --- The caret ----------------------------------------------------------------------------

    #[test]
    fn set_puts_the_caret_after_the_text_and_the_same_text_leaves_it() {
        let mut field = drawn("BATT");
        assert_eq!(field.caret(), 4);
        field.select(1, 1);
        field.set("BATT");
        assert_eq!(field.caret(), 1, "the same text again moves nothing");
        field.set("RTL");
        assert_eq!(field.caret(), 3);
        field.clear();
        assert_eq!(field.caret(), 0);
    }

    #[test]
    fn left_and_right_move_the_caret_one_character_and_stop_at_the_ends() {
        let mut field = drawn("café");
        assert_eq!(
            field.key_with(&press("left", None), &mut Fake::default()),
            KeyOutcome::Ignored
        );
        assert_eq!(field.caret(), 3);
        keys(&mut field, &[press("left", None), press("left", None)]);
        keys(&mut field, &[press("left", None), press("left", None)]);
        assert_eq!(field.caret(), 0, "and no further");
        keys(&mut field, &[press("right", None)]);
        assert_eq!(field.caret(), 1);
        keys(&mut field, &[press("end", None), press("right", None)]);
        assert_eq!(field.caret(), 4, "and no further");
    }

    #[test]
    fn home_and_end_go_to_the_ends_of_a_line() {
        let mut field = drawn("WPNAV_SPEED");
        keys(&mut field, &[press("home", None)]);
        assert_eq!(field.caret(), 0);
        keys(&mut field, &[press("end", None)]);
        assert_eq!(field.caret(), 11);
    }

    #[test]
    fn typing_goes_in_at_the_caret() {
        let mut field = drawn("WPSPEED");
        keys(&mut field, &[press("home", None)]);
        for _ in 0..2 {
            keys(&mut field, &[press("right", None)]);
        }
        keys(&mut field, &[press("n", Some("N")), press("a", Some("A"))]);
        assert_eq!(field.value(), "WPNASPEED");
        assert_eq!(field.caret(), 4);
    }

    #[test]
    fn control_with_the_arrows_moves_by_words() {
        let mut field = drawn("set WP_SPEED now");
        keys(&mut field, &[control("left")]);
        assert_eq!(field.caret(), 13, "to the start of the last word");
        keys(&mut field, &[control("left")]);
        assert_eq!(field.caret(), 4, "underscores are part of a word");
        keys(&mut field, &[control("right")]);
        assert_eq!(field.caret(), 13, "past the word and the space after it");
        keys(&mut field, &[control("home")]);
        assert_eq!(field.caret(), 0);
        keys(&mut field, &[control("end")]);
        assert_eq!(field.caret(), 16);
    }

    #[test]
    fn up_and_down_are_ignored_in_a_single_line_box() {
        // They go on to the page, as they always have.
        let mut field = drawn("abc");
        assert_eq!(
            field.apply(&press("up", None), &mut Fake::default()),
            (KeyOutcome::Ignored, false)
        );
        assert_eq!(field.caret(), 3);
    }

    // --- The selection ------------------------------------------------------------------------

    #[test]
    fn shift_with_the_arrows_extends_the_selection_and_either_arrow_collapses_it() {
        let mut field = drawn("BATT_CAPACITY");
        keys(&mut field, &[shifted("left"), shifted("left")]);
        assert_eq!(field.selection(), Some((11, 13)));
        assert_eq!(field.selected_text(), "TY");
        assert_eq!(field.caret(), 11);
        keys(&mut field, &[shifted("right")]);
        assert_eq!(field.selection(), Some((12, 13)), "back toward the anchor");
        keys(&mut field, &[shifted("left"), shifted("left")]);
        keys(&mut field, &[press("left", None)]);
        assert_eq!(field.selection(), None, "Left collapses to the start");
        assert_eq!(field.caret(), 10);
        keys(&mut field, &[shifted("right"), shifted("right")]);
        keys(&mut field, &[press("right", None)]);
        assert_eq!(field.selection(), None, "Right collapses to the end");
        assert_eq!(field.caret(), 12);
    }

    #[test]
    fn shift_with_home_and_end_selects_to_the_ends() {
        let mut field = drawn("BATT_CAPACITY");
        keys(&mut field, &[press("home", None), press("right", None)]);
        keys(&mut field, &[shifted("end")]);
        assert_eq!(field.selected_text(), "ATT_CAPACITY");
        keys(&mut field, &[shifted("home")]);
        assert_eq!(field.selected_text(), "B", "from the anchor the other way");
        keys(&mut field, &[control_shift("left")]);
        assert_eq!(field.selected_text(), "B");
        keys(&mut field, &[control_shift("end")]);
        assert_eq!(field.selected_text(), "ATT_CAPACITY");
    }

    #[test]
    fn control_a_selects_everything() {
        let mut field = drawn("RTL_ALT");
        assert_eq!(
            field.apply(&chord("a"), &mut Fake::default()),
            (KeyOutcome::Ignored, true)
        );
        assert_eq!(field.selection(), Some((0, 7)));
        assert_eq!(field.selection_fact(), "0,7");
        keys(&mut field, &[press("end", None)]);
        assert_eq!(field.selection_fact(), "none");
    }

    #[test]
    fn typing_replaces_the_selection() {
        let mut field = drawn("RTL_ALT");
        keys(&mut field, &[chord("a"), press("w", Some("W"))]);
        assert_eq!(field.value(), "W");
        assert_eq!(field.selection(), None);
        assert_eq!(field.caret(), 1);
    }

    #[test]
    fn backspace_and_delete_take_the_selection_or_one_character() {
        let mut field = drawn("WPNAV_SPEED");
        keys(&mut field, &[shifted("left"), shifted("left")]);
        keys(&mut field, &[press("backspace", None)]);
        assert_eq!(field.value(), "WPNAV_SPE");
        keys(&mut field, &[press("home", None), shifted("right")]);
        keys(&mut field, &[press("delete", None)]);
        assert_eq!(field.value(), "PNAV_SPE");
        keys(&mut field, &[press("delete", None)]);
        assert_eq!(field.value(), "NAV_SPE", "Delete takes the character after");
        keys(&mut field, &[press("end", None), press("backspace", None)]);
        assert_eq!(field.value(), "NAV_SP", "Backspace the one before");
        assert_eq!(
            field.key_with(&press("delete", None), &mut Fake::default()),
            KeyOutcome::Ignored,
            "nothing after the caret"
        );
    }

    #[test]
    fn a_click_puts_the_caret_and_shift_click_extends_from_it() {
        let mut field = drawn("BATT_CAPACITY");
        field.click(4, false, 1);
        assert_eq!((field.caret(), field.selection()), (4, None));
        field.click(8, true, 1);
        assert_eq!(field.selected_text(), "_CAP");
        field.click(99, false, 1);
        assert_eq!(field.caret(), 13, "past the end is the end");
    }

    #[test]
    fn a_drag_selects_while_the_button_is_down() {
        let mut field = drawn("BATT_CAPACITY");
        field.click(0, false, 1);
        let chars = field.chars();
        assert!(field.cursor().drag(&chars, 4));
        assert_eq!(field.selected_text(), "BATT");
        field.cursor().release();
        assert!(
            !field.cursor().drag(&chars, 8),
            "released, a move selects nothing"
        );
        assert_eq!(field.selected_text(), "BATT");
    }

    #[test]
    fn a_double_click_selects_the_word() {
        let mut field = drawn("set WP_SPEED, now");
        field.click(6, false, 2);
        assert_eq!(field.selected_text(), "WP_SPEED");
        field.click(12, false, 2);
        assert_eq!(field.selected_text(), ",");
        field.click(17, false, 2);
        assert_eq!(
            field.selected_text(),
            "now",
            "at the very end, the last word"
        );
        let mut empty = drawn("");
        empty.click(0, false, 2);
        assert_eq!(empty.selection(), None);
    }

    // --- The clipboard ------------------------------------------------------------------------

    #[test]
    fn copy_puts_the_selection_on_the_clipboard_and_leaves_the_text() {
        let mut field = drawn("BATT_CAPACITY");
        let mut clipboard = Fake::default();
        assert_eq!(
            field.key_with(&chord("c"), &mut clipboard),
            KeyOutcome::Ignored
        );
        assert_eq!(clipboard.0, None, "nothing selected, nothing copied");
        field.select(0, 4);
        assert_eq!(
            field.key_with(&chord("c"), &mut clipboard),
            KeyOutcome::Ignored
        );
        assert_eq!(clipboard.0.as_deref(), Some("BATT"));
        assert_eq!(field.value(), "BATT_CAPACITY");
        clipboard.0 = None;
        assert_eq!(
            field.key_with(&control("insert"), &mut clipboard),
            KeyOutcome::Ignored
        );
        assert_eq!(
            clipboard.0.as_deref(),
            Some("BATT"),
            "Ctrl+Insert copies too"
        );
    }

    #[test]
    fn cut_copies_then_removes() {
        let mut field = drawn("BATT_CAPACITY");
        let mut clipboard = Fake::default();
        assert_eq!(
            field.key_with(&chord("x"), &mut clipboard),
            KeyOutcome::Ignored
        );
        field.select(4, 13);
        assert_eq!(
            field.key_with(&chord("x"), &mut clipboard),
            KeyOutcome::Changed
        );
        assert_eq!(clipboard.0.as_deref(), Some("_CAPACITY"));
        assert_eq!(field.value(), "BATT");
        field.select(0, 1);
        assert_eq!(
            field.key_with(&shifted("delete"), &mut clipboard),
            KeyOutcome::Changed
        );
        assert_eq!(clipboard.0.as_deref(), Some("B"), "Shift+Delete cuts");
        assert_eq!(field.value(), "ATT");
    }

    #[test]
    fn paste_goes_in_at_the_caret_over_the_selection() {
        let mut field = drawn("BATT_CAPACITY");
        let mut clipboard = Fake(Some("MON".to_owned()));
        field.select(5, 13);
        assert_eq!(
            field.key_with(&chord("v"), &mut clipboard),
            KeyOutcome::Changed
        );
        assert_eq!(field.value(), "BATT_MON");
        assert_eq!(field.caret(), 8);
        keys(&mut field, &[press("home", None)]);
        assert_eq!(
            field.key_with(&shifted("insert"), &mut clipboard),
            KeyOutcome::Changed
        );
        assert_eq!(field.value(), "MONBATT_MON", "Shift+Insert pastes too");
        assert_eq!(
            field.key_with(&chord("v"), &mut Fake::default()),
            KeyOutcome::Ignored,
            "an empty clipboard pastes nothing"
        );
    }

    #[test]
    fn a_single_line_box_pastes_the_first_line_only() {
        let mut field = drawn("");
        let mut clipboard = Fake(Some("RTL_ALT\r\nWPNAV\tSPEED".to_owned()));
        field.key_with(&chord("v"), &mut clipboard);
        assert_eq!(field.value(), "RTL_ALT");
    }

    #[test]
    fn copy_select_all_clear_and_paste_round_trips() {
        // What tests/gui/textfield-clipboard.gui does to the parameter search box.
        let mut field = drawn("");
        let mut clipboard = Fake::default();
        for character in ["B", "A", "T", "T"] {
            field.key_with(
                &press(&character.to_lowercase(), Some(character)),
                &mut clipboard,
            );
        }
        field.key_with(&chord("a"), &mut clipboard);
        field.key_with(&chord("c"), &mut clipboard);
        field.key_with(&press("delete", None), &mut clipboard);
        assert_eq!(field.value(), "");
        field.key_with(&chord("v"), &mut clipboard);
        field.key_with(&chord("v"), &mut clipboard);
        assert_eq!(field.value(), "BATTBATT");
    }

    #[test]
    fn a_paste_chord_is_recognised_before_it_reaches_the_owner() {
        assert!(is_paste(&chord("v").keystroke));
        assert!(is_paste(&shifted("insert").keystroke));
        assert!(!is_paste(&chord("c").keystroke));
        assert!(!is_paste(&press("v", Some("v")).keystroke));
    }

    #[test]
    fn the_system_clipboard_passes_through_the_exchange() {
        // The drawn box reads the clipboard into the exchange before the owner's handler runs,
        // and writes what the field copied after; `key` is what the owner calls in between.
        let mut field = drawn("BATT");
        EXCHANGE.with(|exchange| {
            *exchange.borrow_mut() = Exchange {
                paste: Some("_MON".to_owned()),
                ..Exchange::default()
            };
        });
        assert_eq!(field.key(&chord("v")), KeyOutcome::Changed);
        assert_eq!(field.value(), "BATT_MON");
        field.select(0, 4);
        assert_eq!(field.key(&chord("c")), KeyOutcome::Ignored);
        let done = EXCHANGE.with(|exchange| std::mem::take(&mut *exchange.borrow_mut()));
        assert_eq!(done.copied.as_deref(), Some("BATT"));
        assert!(done.consumed, "kept from the parents, and redrawn");
        // Typing is not the field's alone: it reaches the parents as it always has.
        assert_eq!(field.key(&press("x", Some("x"))), KeyOutcome::Changed);
        let done = EXCHANGE.with(|exchange| std::mem::take(&mut *exchange.borrow_mut()));
        assert!(!done.consumed);
    }

    // --- Multi-line ---------------------------------------------------------------------------

    #[test]
    fn enter_is_a_new_line_in_a_multi_line_box_and_submits_in_a_single_line_one() {
        let mut area = area("RC7_OPTION");
        assert_eq!(
            area.key_with(&press("enter", None), &mut Fake::default()),
            KeyOutcome::Changed
        );
        keys(&mut area, &[press("r", Some("R"))]);
        assert_eq!(area.value(), "RC7_OPTION\r\nR");
        let mut line = drawn("RC7_OPTION");
        assert_eq!(
            line.key_with(&press("enter", None), &mut Fake::default()),
            KeyOutcome::Submitted
        );
        assert_eq!(line.value(), "RC7_OPTION");
    }

    #[test]
    fn up_and_down_keep_the_column_between_lines() {
        let mut area = area("RC7_OPTION\r\nRTL\r\nWPNAV_SPEED");
        keys(&mut area, &[press("up", None)]);
        assert_eq!(area.caret(), 15, "the end of the shorter line above");
        keys(&mut area, &[press("up", None)]);
        assert_eq!(area.caret(), 3, "column 3 of the first line");
        keys(&mut area, &[press("up", None)]);
        assert_eq!(area.caret(), 3, "the first line has nothing above");
        keys(&mut area, &[press("down", None), press("down", None)]);
        assert_eq!(area.caret(), 20, "column 3 of the last line");
        keys(&mut area, &[press("down", None)]);
        assert_eq!(area.caret(), 20, "the last line has nothing below");
        keys(&mut area, &[shifted("up")]);
        assert_eq!(area.selected_text(), "\r\nWPN");
    }

    #[test]
    fn home_and_end_stay_on_the_line_and_a_line_break_is_one_step() {
        let mut area = area("RC7\r\nRTL");
        keys(&mut area, &[press("home", None)]);
        assert_eq!(area.caret(), 5);
        keys(&mut area, &[press("left", None)]);
        assert_eq!(area.caret(), 3, "over \\r\\n at once");
        keys(&mut area, &[press("end", None)]);
        assert_eq!(area.caret(), 3, "before the break");
        keys(&mut area, &[press("right", None)]);
        assert_eq!(area.caret(), 5);
        keys(&mut area, &[press("backspace", None)]);
        assert_eq!(area.value(), "RC7RTL", "the whole break goes");
        area.click(4, false, 1);
        assert_eq!(area.caret(), 4);
        let mut area = self::area("A\r\nB");
        area.click(2, false, 1);
        assert_eq!(area.caret(), 1, "never between \\r and \\n");
        keys(&mut area, &[press("delete", None)]);
        assert_eq!(area.value(), "AB");
    }

    #[test]
    fn a_multi_line_box_pastes_every_line_with_its_breaks() {
        let mut area = area("");
        let mut clipboard = Fake(Some("RC7_OPTION\nRTL\r\nWP\tNAV".to_owned()));
        area.key_with(&chord("v"), &mut clipboard);
        assert_eq!(area.value(), "RC7_OPTION\r\nRTL\r\nWP\tNAV");
    }

    #[test]
    fn insert_goes_in_at_the_caret_as_a_page_s_own_keys_want() {
        // User Params' InputBox puts its Enter and Tab in through this.
        let mut area = area("AB");
        keys(&mut area, &[press("left", None)]);
        area.insert(NEW_LINE);
        assert_eq!(area.value(), "A\r\nB");
        let mut plain = field();
        plain.set("A");
        plain.insert("\t");
        assert_eq!(plain.value(), "A\t", "never drawn: at the end");
    }

    #[test]
    fn a_byte_offset_maps_to_the_character_it_falls_in() {
        let chars: Vec<char> = "café!".chars().collect();
        assert_eq!(char_index(&chars, 0), 0);
        assert_eq!(char_index(&chars, 3), 3);
        assert_eq!(char_index(&chars, 5), 4, "after the two bytes of é");
        assert_eq!(char_index(&chars, 99), 5);
    }
}
