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

//! Six pages of Initial Setup's list, ported together: ADSB (`ConfigADSB.cs`, under Mandatory
//! Hardware, `GCSViews/InitialSetup.cs:262-263`), and five Optional Hardware pages - Battery
//! Monitor 2 (`ConfigBatteryMonitoring2.cs`, `:271`), Range Finder (`ConfigHWRangeFinder.cs`,
//! `:289`), Airspeed (`ConfigHWAirspeed.cs`, `:293`), Optical Flow (`ConfigHWOptFlow.cs`, `:301`)
//! and Camera Gimbal (`ConfigMount.cs`, `:309`). Each page is its own module; this one holds what
//! they share and their part in the application:
//!
//! * [`SetQueue`], the handlers' `setParam` calls. The C# calls `setParam` on the UI thread, one
//!   after another, each blocking until the vehicle echoes the value or three retries go
//!   unanswered (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1628-1770`): `false` for a name
//!   the vehicle has not listed, `true` for a value it already holds, a `TimeoutException` when no
//!   answer comes. Here each handler's calls are a [`Job`], run one call at a time in the order the
//!   handlers ran, through the link's retrying set; what a `catch` or a `false` shows is carried
//!   with each call.
//! * The drawing the pages share: a picture where the C# has one ([`crate::pictures`]), the
//!   five-pixel group boxes that draw as rules, the 12-point headings, a text box.
//! * [`Optional`], the six page objects, and [`Focus`], their keyboard focus; the arms of
//!   `setup.rs` and the lines of `main.rs` call into `impl MissionPlanner` here.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use web_time::Instant;

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px,
    rgb,
};
use mp_link::requests::RequestOutcome;

use crate::MissionPlanner;
use crate::config::flight_modes::{ParamWriter, Progress};
use crate::config::servo_output::{ERROR_TITLE, Message, Write};
use crate::setup::Key;
use crate::telemetry::TelemetryView;
use crate::textfield::TextField;
use crate::ui::{action, theme};

use super::{adsb, airspeed, battery_monitor2, mount, optical_flow, rangefinder};

// ---------------------------------------------------------------------------------------------
// The handlers' setParam calls.
// ---------------------------------------------------------------------------------------------

/// `Strings.ErrorSetValueFailed`, "Set {0} Failed".
/// `// C#: ExtLibs/Strings/Strings.resx:171-173`
#[must_use]
pub fn set_failed(param: &str) -> String {
    format!("Set {param} Failed")
}

/// `Strings.ErrorFeatureNotEnabled`.
/// `// C#: ExtLibs/Strings/Strings.resx:143-145`
pub const FEATURE_NOT_ENABLED: &str = "This feature is not enabled in your firmware.";

/// `Strings.InvalidNumberEntered`, newline and all.
/// `// C#: ExtLibs/Strings/Strings.resx:186-188`
pub const INVALID_NUMBER: &str = "Invalid number entered\n";

/// What `setParam`'s `TimeoutException` says of itself, `ex.ToString()`'s first line: the stack
/// trace after it is the .NET runtime's, and is not reproduced.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1765`
#[must_use]
pub fn timeout_text(param: &str) -> String {
    format!("System.TimeoutException: Timeout on read - setParam {param}")
}

/// An error box: `CustomMessageBox.Show(text, Strings.ERROR)`.
#[must_use]
pub fn error(text: impl Into<String>) -> Message {
    Message {
        title: ERROR_TITLE,
        text: text.into(),
    }
}

/// A box with no caption: `CustomMessageBox.Show(text)`.
#[must_use]
pub fn plain(text: impl Into<String>) -> Message {
    Message {
        title: "",
        text: text.into(),
    }
}

/// One `setParam`, and what the code around it shows when it returns false or throws.
#[derive(Debug, Clone, PartialEq)]
pub struct Set {
    /// The name.
    pub param: String,
    /// The value.
    pub value: f64,
    /// Shown when `setParam` returns false: the vehicle has not listed the name.
    pub on_false: Option<Message>,
    /// Shown when it throws: every retry went unanswered.
    pub on_throw: Option<Message>,
}

impl Set {
    /// A call whose `bool` the handler ignores, inside a `try` that the [`Job`] speaks for.
    #[must_use]
    pub fn plain(param: impl Into<String>, value: f64) -> Self {
        Self {
            param: param.into(),
            value,
            on_false: None,
            on_throw: None,
        }
    }

    /// A call whose `catch` shows `text` in an error box.
    #[must_use]
    pub fn caught(param: impl Into<String>, value: f64, text: impl Into<String>) -> Self {
        Self {
            on_throw: Some(error(text)),
            ..Self::plain(param, value)
        }
    }

    /// A `Mavlink*` control's own write: the same box for a false and for a throw.
    /// `// C#: Controls/MavlinkComboBox.cs:180-198; Controls/MavlinkCheckBox.cs:116-141;
    /// Controls/MavlinkNumericUpDown.cs:169-176`
    #[must_use]
    pub fn control(write: Write) -> Self {
        let message = error(write.failure);
        Self {
            param: write.param,
            value: write.value,
            on_false: Some(message.clone()),
            on_throw: Some(message),
        }
    }
}

/// One handler's calls, as it makes them.
#[derive(Debug, Clone)]
pub struct Job {
    /// Which handler, for the page to act on when it ends.
    pub tag: &'static str,
    /// A box the handler shows before its calls.
    pub before: Option<Message>,
    /// The calls, in order.
    pub sets: VecDeque<Set>,
    /// Whether each call has a `try` of its own, so a throw ends only it (ADSB's write loop);
    /// otherwise one `try` holds them all and a throw ends the job.
    pub each_caught: bool,
    /// The box the job's own `catch` shows for a throw that ends it, from the name that threw.
    pub on_throw: Option<fn(&str) -> Message>,
}

impl Job {
    /// Calls inside one `try` whose `catch` says nothing beyond each call's own box.
    #[must_use]
    pub fn new(tag: &'static str, sets: impl IntoIterator<Item = Set>) -> Self {
        Self {
            tag,
            before: None,
            sets: sets.into_iter().collect(),
            each_caught: false,
            on_throw: None,
        }
    }

    /// A box and nothing else.
    #[must_use]
    pub fn show(tag: &'static str, message: Message) -> Self {
        Self {
            before: Some(message),
            ..Self::new(tag, [])
        }
    }

    /// A control's own write.
    #[must_use]
    pub fn control(write: Write) -> Self {
        Self::new("control", [Set::control(write)])
    }
}

/// How one call ended.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// `true`, after the vehicle echoed this.
    Accepted(f64),
    /// `true` without a send: the vehicle already held it ("not modified as same").
    Unchanged,
    /// `false`: a name the vehicle has not listed, or no vehicle.
    False,
    /// The `TimeoutException`.
    Threw,
}

/// What running the jobs did.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A call ended.
    Set {
        /// The name.
        param: String,
        /// The value asked for.
        value: f64,
        /// How.
        outcome: Outcome,
    },
    /// A job ended; whether any of its calls threw.
    Done {
        /// Its tag.
        tag: &'static str,
        /// Whether a call threw.
        threw: bool,
    },
}

/// Where a box the jobs put up comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Raised {
    /// A job's `before`: the handler's own box, ahead of its calls.
    Handler,
    /// A call's `false` or throw, or the `catch` a throw ends its job in: the link failing.
    Link,
}

/// The job running and where it is.
#[derive(Debug)]
struct Running<H> {
    job: Job,
    waiting: Option<H>,
    threw: bool,
}

/// The handlers' jobs, one at a time, one call at a time.
#[derive(Debug)]
pub struct SetQueue<H = mp_link::RequestId> {
    queue: VecDeque<Job>,
    running: Option<Running<H>>,
    /// How the last call ended, for the facts.
    last: Option<String>,
}

impl<H> Default for SetQueue<H> {
    fn default() -> Self {
        Self {
            queue: VecDeque::new(),
            running: None,
            last: None,
        }
    }
}

impl<H: Copy> SetQueue<H> {
    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: impl IntoIterator<Item = Job>) {
        self.queue.extend(jobs);
    }

    /// How many jobs are queued or running.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.len() + usize::from(self.running.is_some())
    }

    /// How the last call ended: "NAME value accepted", "unchanged", "false" or "failed".
    #[must_use]
    pub fn last(&self) -> Option<&str> {
        self.last.as_deref()
    }

    /// Moves the jobs on as far as the link's answers allow, putting up the boxes they show.
    pub fn advance<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        messages: &mut VecDeque<Message>,
    ) -> Vec<Event> {
        self.run(writer, |_, message| messages.push_back(message))
    }

    /// [`Self::advance`], with the boxes sorted by where they come from: a handler's own box
    /// ahead of its calls - a refusal of what was typed - into `boxes`, and what a call's
    /// `false`, its throw or the job's `catch` says into `failures`, for the status line. The
    /// owner's ruling of 2026-09-25: an error the window can show as state never gets a box.
    pub fn advance_split<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        boxes: &mut VecDeque<Message>,
        failures: &mut Vec<Message>,
    ) -> Vec<Event> {
        self.run(writer, |raised, message| match raised {
            Raised::Handler => boxes.push_back(message),
            Raised::Link => failures.push(message),
        })
    }

    /// The jobs moved on, each box handed to `raise` with where it comes from.
    fn run<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        mut raise: impl FnMut(Raised, Message),
    ) -> Vec<Event> {
        let mut events = Vec::new();
        loop {
            let Some(running) = self.running.as_mut() else {
                let Some(mut job) = self.queue.pop_front() else {
                    return events;
                };
                if let Some(message) = job.before.take() {
                    raise(Raised::Handler, message);
                }
                self.running = Some(Running {
                    job,
                    waiting: None,
                    threw: false,
                });
                continue;
            };
            let outcome = if let Some(handle) = running.waiting {
                match writer.progress(handle) {
                    Progress::Waiting => return events,
                    Progress::Lost | Progress::Finished(RequestOutcome::TimedOut) => Outcome::Threw,
                    Progress::Finished(
                        RequestOutcome::UnknownParameter | RequestOutcome::Rejected(_),
                    ) => Outcome::False,
                    Progress::Finished(RequestOutcome::Accepted { value }) => {
                        let asked = running.job.sets.front().map_or(0.0, |set| set.value);
                        Outcome::Accepted(value.map_or(asked, mp_params::ParamValue::as_f64))
                    }
                    Progress::Finished(RequestOutcome::Unchanged | RequestOutcome::Sent) => {
                        Outcome::Unchanged
                    }
                }
            } else {
                let Some(set) = running.job.sets.front() else {
                    events.push(Event::Done {
                        tag: running.job.tag,
                        threw: running.threw,
                    });
                    self.running = None;
                    continue;
                };
                match writer.write(&set.param, set.value) {
                    Some(handle) => {
                        running.waiting = Some(handle);
                        continue;
                    }
                    // No vehicle: `setParam` finds no such name in an empty list.
                    None => Outcome::False,
                }
            };
            running.waiting = None;
            let Some(set) = running.job.sets.pop_front() else {
                continue;
            };
            self.last = Some(match &outcome {
                Outcome::Accepted(echo) => format!("{} {echo} accepted", set.param),
                Outcome::Unchanged => format!("{} {} unchanged", set.param, set.value),
                Outcome::False => format!("{} {} false", set.param, set.value),
                Outcome::Threw => format!("{} {} failed", set.param, set.value),
            });
            match outcome {
                Outcome::False => {
                    if let Some(message) = set.on_false.clone() {
                        raise(Raised::Link, message);
                    }
                }
                Outcome::Threw => {
                    if let Some(message) = set.on_throw.clone() {
                        raise(Raised::Link, message);
                    }
                    running.threw = true;
                    if !running.job.each_caught {
                        if let Some(catch) = running.job.on_throw {
                            raise(Raised::Link, catch(&set.param));
                        }
                        running.job.sets.clear();
                    }
                }
                Outcome::Accepted(_) | Outcome::Unchanged => {}
            }
            events.push(Event::Set {
                param: set.param,
                value: set.value,
                outcome,
            });
        }
    }
}

/// A parameter's value in the vehicle's table.
#[must_use]
pub fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    crate::config::servo_output::value_of(parameters, name)
}

/// `MAV.param.ContainsKey(name)`.
#[must_use]
pub fn has(parameters: &[(String, f64)], name: &str) -> bool {
    value_of(parameters, name).is_some()
}

/// The status line's words for the last of the link failures [`SetQueue::advance_split`] sorted
/// out, as `extra_setup` words its pages' - the C#'s box's text on one line.
#[must_use]
pub fn failure_status(failures: &[Message]) -> Option<String> {
    failures.last().map(super::extra_setup::status_words)
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// A box at a `.resx` `Location` and `Size`.
#[must_use]
pub fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A label at its `Location`, dimmed under a disabled page.
#[must_use]
pub fn label(x: f32, y: f32, text: impl Into<SharedString>, enabled: bool) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .text_xs()
        .whitespace_nowrap()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(text.into())
}

/// A label in "Microsoft Sans Serif, 12pt", as the pages' headings are.
#[must_use]
pub fn heading(x: f32, y: f32, text: &'static str, enabled: bool) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .text_base()
        .whitespace_nowrap()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(text)
}

/// A group box five pixels high, which draws as the rule it looks like.
#[must_use]
pub fn rule(x: f32, y: f32, width: f32) -> AnyElement {
    div()
        .absolute()
        .left(px(x))
        .top(px(y + 2.0))
        .w(px(width))
        .h(px(1.0))
        .bg(rgb(theme::BORDER))
        .into_any_element()
}

/// A group box with its caption, the controls in it placed from its corner.
#[must_use]
pub fn group(
    (x, y, width, height): (f32, f32, f32, f32),
    caption: &'static str,
    enabled: bool,
) -> Div {
    at(x, y, width, height)
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .child(
            div()
                .absolute()
                .left(px(6.0))
                .top(px(-8.0))
                .px_1()
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
                .child(caption),
        )
}

/// A `PictureBox` showing its resource: [`crate::pictures::picture`].
pub use crate::pictures::picture;

/// A plain button (`MyButton`) at its place; dimmed and inert while disabled.
pub fn button(
    id: &'static str,
    text: &'static str,
    (x, y, width, height): (f32, f32, f32, f32),
    enabled: bool,
    on_click: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .size_full()
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .text_xs()
        .child(text);
    let base = if enabled {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                on_click(this, window, cx);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// A text box at its place: its text, and while it has the focus, typing and a caret. A box that
/// is read-only or disabled draws its text and takes nothing.
#[allow(clippy::too_many_arguments)]
pub fn text_box(
    id: &'static str,
    text: &str,
    typing: Option<&FocusHandle>,
    focused: bool,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    on_begin: impl Fn(&mut MissionPlanner) + 'static,
    on_key: impl Fn(&mut MissionPlanner, &KeyDownEvent) -> bool + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .size_full()
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .text_xs()
        // A line that fits a 20-high box inside its border (the layout guard on the owner's Mac,
        // 2026-10-04: SIMULATION's command line was 19 high in 18).
        .line_height(px(16.0))
        .overflow_hidden()
        .whitespace_nowrap()
        .child(text.to_owned());
    let base = match typing {
        Some(handle) if enabled => {
            let handle = handle.clone();
            let base = if focused {
                base.track_focus(&handle)
                    .key_context("TextField")
                    .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                        if on_key(this, event) {
                            cx.notify();
                        }
                    }))
                    .child(div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT)))
            } else {
                base.on_click(cx.listener(move |this, _event, window, cx| {
                    on_begin(this);
                    handle.focus(window, cx);
                    cx.notify();
                }))
            };
            base.bg(rgb(theme::ACTION))
                .border_color(rgb(if focused {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .text_color(rgb(theme::TEXT))
                .cursor_text()
        }
        _ => base
            .bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM })),
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// An `InputBox`: a caption, a question and an answer box, with OK and Cancel.
/// `// C#: ExtLibs/Controls/InputBox.cs:62-190`
#[derive(Debug)]
pub struct InputBox {
    /// The caption.
    pub title: &'static str,
    /// The question.
    pub prompt: &'static str,
    /// The answer box, holding the value passed in.
    pub field: TextField,
}

impl InputBox {
    /// `InputBox.Show(title, prompt, ref value)`.
    #[must_use]
    pub fn new(title: &'static str, prompt: &'static str, value: &str) -> Self {
        let mut field = TextField::new("");
        field.set(value);
        Self {
            title,
            prompt,
            field,
        }
    }

    /// OK: the answer kept in `Settings.Instance`, as [`remember_answer`] keeps it.
    pub fn remember(&self, settings: &mut crate::settings::Persisted) {
        remember_answer(settings, self.title, self.prompt, self.field.value());
    }
}

/// The key `InputBox` keeps a question's answers under: `"InputBox" + title.CleanString() +
/// promptText.CleanString()`, `CleanString` keeping the letters and digits.
/// `// C#: ExtLibs/Controls/InputBox.cs:75, 183; ExtLibs/Utilities/Extensions.cs:494-497`
#[must_use]
pub fn answers_key(title: &str, prompt: &str) -> String {
    // `Char.IsLetterOrDigit`; Rust's `is_alphanumeric` also takes letter-numbers and marks some
    // scripts class as alphabetic, which no question here holds.
    let clean = |text: &str| {
        text.chars()
            .filter(|c| c.is_alphanumeric())
            .collect::<String>()
    };
    format!("InputBox{}{}", clean(title), clean(prompt))
}

/// `WebUtility.UrlEncode`: letters, digits and `-_.!*()` as they are, a space `+`, every other
/// UTF-8 byte `%XX` in capitals.
#[must_use]
pub fn url_encode(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for byte in text.bytes() {
        match byte {
            b'a'..=b'z'
            | b'A'..=b'Z'
            | b'0'..=b'9'
            | b'-'
            | b'_'
            | b'.'
            | b'!'
            | b'*'
            | b'('
            | b')' => out.push(char::from(byte)),
            b' ' => out.push('+'),
            _ => {
                use std::fmt::Write as _;
                let _ = write!(out, "%{byte:02X}");
            }
        }
    }
    out
}

/// What `InputBox.Show` keeps when its OK is pressed: the box's autocomplete list with the answer
/// added, `SetList` into `Settings.Instance` - distinct, URL-encoded, `;`-joined. The list it
/// started from is `GetList` of the key filtered to its null or empty entries (`Where(a => a ==
/// null || a == "")`), so the list kept is an empty entry, if the setting had one, then the
/// answer; and the suggestions the box offers are only ever empty. Nothing on the pages that ask
/// reads the list back.
/// `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184; ExtLibs/Utilities/Settings.cs:164-176`
pub fn remember_answer(
    settings: &mut crate::settings::Persisted,
    title: &str,
    prompt: &str,
    answer: &str,
) {
    // `if (title != "")`: an untitled box has no list, and `AutoCompleteCustomSource` stays null.
    if title.is_empty() {
        return;
    }
    let key = answers_key(title, prompt);
    // `GetList`: split on `;`, each URL-decoded; an entry decodes to "" only when it is "".
    let had_empty = settings
        .get(&key)
        .is_some_and(|list| list.split(';').any(str::is_empty));
    let mut list: Vec<&str> = Vec::new();
    if had_empty {
        list.push("");
    }
    if !list.contains(&answer) {
        list.push(answer);
    }
    let encoded: Vec<String> = list.iter().map(|entry| url_encode(entry)).collect();
    settings.set(&key, encoded.join(";"));
}

/// An `InputBox` drawn over the window, modal as the C#'s is.
#[allow(clippy::too_many_arguments)]
pub fn input_box(
    id: &'static str,
    input: &InputBox,
    handle: &FocusHandle,
    window: &Window,
    on_key: impl Fn(&mut MissionPlanner, &KeyDownEvent) -> bool + 'static,
    on_ok: impl Fn(&mut MissionPlanner) + 'static,
    on_cancel: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = handle.is_focused(window);
    let (ok_id, cancel_id, field_id) = match id {
        "adsb-find-box" => ("adsb-find-ok", "adsb-find-cancel", "adsb-find-value"),
        // ---- crash reports ----
        "crash-message" => ("crash-message-ok", "crash-message-cancel", "crash-message-value"),
        // ---- end crash reports ----
        // ---- SITL ----
        "sitl-howmany" => (
            "sitl-howmany-ok",
            "sitl-howmany-cancel",
            "sitl-howmany-value",
        ),
        // ---- end SITL ----
        // ---- Standard / Advanced Params, MAVFtp ----
        "standardparams-find-box" => (
            "standardparams-find-ok",
            "standardparams-find-cancel",
            "standardparams-find-value",
        ),
        "advancedparams-find-box" => (
            "advancedparams-find-ok",
            "advancedparams-find-cancel",
            "advancedparams-find-value",
        ),
        "mavftp-prompt-box" => (
            "mavftp-prompt-ok",
            "mavftp-prompt-cancel",
            "mavftp-prompt-value",
        ),
        // ---- end Standard / Advanced Params, MAVFtp ----
        // ---- FFT Setup ----
        "fft-rate-box" => ("fft-rate-ok", "fft-rate-cancel", "fft-rate-value"),
        // ---- end FFT Setup ----
        // The MAVLink Inspector's "Points of history?".
        "nmea-prompt-box" => ("nmea-prompt-ok", "nmea-prompt-cancel", "nmea-prompt-value"),
        // The EXPERIMENTAL tab's QNH.
        "experimental-input-box" => (
            "experimental-input-ok",
            "experimental-input-cancel",
            "experimental-input-value",
        ),
        "dronecan-inspector-points-box" => (
            "dronecan-inspector-points-ok",
            "dronecan-inspector-points-cancel",
            "dronecan-inspector-points-value",
        ),
        "inspector-points-box" => (
            "inspector-points-ok",
            "inspector-points-cancel",
            "inspector-points-value",
        ),
        _ => (
            "battery2-prompt-ok",
            "battery2-prompt-cancel",
            "battery2-prompt-value",
        ),
    };
    let buttons = vec![
        action(
            ok_id,
            "OK",
            theme::ACCENT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                on_ok(this);
                cx.notify();
            }),
        ),
        action(
            cancel_id,
            "Cancel",
            theme::DIM,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                on_cancel(this);
                cx.notify();
            }),
        ),
    ];
    let dialog = crate::probe::measured(id, div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(340.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(input.title),
        )
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(input.prompt),
        )
        .child(crate::textfield::text_field(
            field_id,
            &input.field,
            handle,
            focused,
            px(310.0),
            cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if on_key(this, event) {
                    cx.notify();
                }
            }),
        ))
        .child(div().flex().justify_end().gap_2().children(buttons));
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id(SharedString::from(format!("{id}-backdrop")))
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(dialog),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// A message box, with OK.
pub fn message_box(
    id: &'static str,
    ok_id: &'static str,
    message: &Message,
    window: &Window,
    on_ok: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let ok = action(
        ok_id,
        "OK",
        theme::ACCENT,
        true,
        cx.listener(move |this, _event: &(), _window, cx| {
            on_ok(this);
            cx.notify();
        }),
    );
    crate::config::servo_output::modal(
        id,
        message.title,
        message.text.trim_end(),
        message.title == ERROR_TITLE,
        vec![ok],
        window,
    )
}

// ---------------------------------------------------------------------------------------------
// The six page objects, and their part in the application.
// ---------------------------------------------------------------------------------------------

/// Which of the six.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Page {
    /// `ConfigADSB`.
    Adsb,
    /// `ConfigBatteryMonitoring2`.
    Battery2,
    /// `ConfigHWRangeFinder`.
    RangeFinder,
    /// `ConfigHWAirspeed`.
    Airspeed,
    /// `ConfigHWOptFlow`.
    OpticalFlow,
    /// `ConfigMount`.
    Mount,
}

impl Page {
    /// The page a C# class is.
    #[must_use]
    pub fn of(class: &str) -> Option<Self> {
        Some(match class {
            "ConfigADSB" => Self::Adsb,
            "ConfigBatteryMonitoring2" => Self::Battery2,
            "ConfigHWRangeFinder" => Self::RangeFinder,
            "ConfigHWAirspeed" => Self::Airspeed,
            "ConfigHWOptFlow" => Self::OpticalFlow,
            "ConfigMount" => Self::Mount,
            _ => return None,
        })
    }
}

/// The six page objects.
#[derive(Debug, Default)]
pub struct Optional {
    /// ADSB.
    pub adsb: adsb::Adsb,
    /// Battery Monitor 2.
    pub battery2: battery_monitor2::BatteryMonitor2,
    /// Range Finder.
    pub rangefinder: rangefinder::RangeFinder,
    /// Airspeed.
    pub airspeed: airspeed::Airspeed,
    /// Optical Flow.
    pub optflow: optical_flow::OpticalFlow,
    /// Camera Gimbal.
    pub mount: mount::Mount,
}

/// The keyboard focus of the six pages' boxes: one for the number being typed into, one for the
/// text box, one for an `InputBox`'s answer.
pub struct Focus {
    /// The `NumericUpDown` being typed into.
    pub number: FocusHandle,
    /// The `TextBox` being typed into.
    pub text: FocusHandle,
    /// An `InputBox`'s answer box.
    pub prompt: FocusHandle,
    /// The ADSB page itself, for `ProcessCmdKey`'s Ctrl+S.
    pub page: FocusHandle,
    /// A `RangeControl`'s track bar, for its keys.
    pub track: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
            text: cx.focus_handle(),
            prompt: cx.focus_handle(),
            page: cx.focus_handle(),
            track: cx.focus_handle(),
        }
    }
}

/// Facts a UI test asserts on, for the six pages.
pub fn record_facts(optional: &Optional, view: &TelemetryView) {
    adsb::record_facts(&optional.adsb, view);
    battery_monitor2::record_facts(&optional.battery2, view);
    rangefinder::record_facts(&optional.rangefinder, view);
    airspeed::record_facts(&optional.airspeed, view);
    optical_flow::record_facts(&optional.optflow, view);
    mount::record_facts(&optional.mount, view);
}

impl MissionPlanner {
    /// `Activate`, when one of the six is chosen.
    pub(crate) fn optional_activate(&mut self, class: &str) {
        let Some(page) = Page::of(class) else {
            return;
        };
        let view = self.telemetry.view();
        let key = Key::of(&view);
        let connected = view.connected && view.vehicle.is_some();
        let banner = self.telemetry.firmware_banner().map(str::to_owned);
        let lookup = crate::metadata::lookup;
        match page {
            Page::Adsb => {
                let favourites = adsb::favourites(adsb::ADSB.favourites);
                let jobs = self.optional.adsb.activate(
                    &view.parameters,
                    &|name| view.parameter_type(name),
                    key,
                    lookup,
                    &favourites,
                );
                self.optional.adsb.push(jobs);
            }
            Page::Battery2 => {
                self.optional
                    .battery2
                    .activate(&view, key, connected, lookup, &self.persisted);
            }
            Page::RangeFinder => {
                self.optional
                    .rangefinder
                    .activate(&view, key, connected, lookup, Instant::now());
            }
            Page::Airspeed => {
                self.optional
                    .airspeed
                    .activate(&view.parameters, key, connected, lookup);
            }
            Page::OpticalFlow => {
                let firmware = crate::setup::Vehicle::of(&view, banner.as_deref()).firmware;
                self.optional.optflow.activate(
                    &view.parameters,
                    key,
                    connected,
                    banner.as_deref(),
                    firmware,
                    lookup,
                );
            }
            Page::Mount => {
                let firmware = crate::setup::Vehicle::of(&view, banner.as_deref()).firmware;
                let jobs = self
                    .optional
                    .mount
                    .activate(&view.parameters, key, firmware, lookup);
                self.optional.mount.push(jobs);
            }
        }
    }

    /// `Deactivate`, or the page hidden, when another is chosen.
    pub(crate) fn optional_deactivate(&mut self, class: &str) {
        let now = Instant::now();
        match Page::of(class) {
            Some(Page::Adsb) => self.optional.adsb.deactivate(),
            Some(Page::Battery2) => self.optional.battery2.deactivate(),
            Some(Page::RangeFinder) => self.optional.rangefinder.deactivate(),
            Some(Page::Airspeed) => self.optional.airspeed.hide(),
            Some(Page::OpticalFlow) => self.optional.optflow.hide(now),
            Some(Page::Mount) => self.optional.mount.hide(now),
            None => {}
        }
    }

    /// The page, when one of the six is showing.
    pub(crate) fn optional_page(
        &self,
        class: &str,
        view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        let optional = &self.optional;
        let focus = &self.optional_focus;
        match Page::of(class) {
            Some(Page::Adsb) => adsb::page(&optional.adsb, focus, window, cx),
            Some(Page::Battery2) => battery_monitor2::page(&optional.battery2, focus, window, cx),
            Some(Page::RangeFinder) => rangefinder::page(&optional.rangefinder, cx),
            Some(Page::Airspeed) => airspeed::page(&optional.airspeed, cx),
            Some(Page::OpticalFlow) => optical_flow::page(&optional.optflow, focus, window, cx),
            Some(Page::Mount) => mount::page(&optional.mount, focus, view, window, cx),
            None => div().into_any_element(),
        }
    }

    /// Once a frame: each page object disposed with its screen, the timers, a box the focus has
    /// left, and the writes moved on.
    pub(crate) fn optional_tick(&mut self, view: &TelemetryView, window: &Window) {
        let on_setup = self.screen == crate::Screen::Setup;
        let now = Instant::now();
        let number_focused = self.optional_focus.number.is_focused(window);
        let text_focused = self.optional_focus.text.is_focused(window);
        let telemetry = &self.telemetry;
        let optional = &mut self.optional;
        optional.adsb.tick(telemetry, view, on_setup, now);
        optional
            .battery2
            .tick(telemetry, view, on_setup, text_focused, now);
        optional.rangefinder.tick(telemetry, view, on_setup, now);
        optional.airspeed.tick(telemetry, view, on_setup);
        optional
            .optflow
            .tick(telemetry, view, on_setup, number_focused, now);
        optional
            .mount
            .tick(telemetry, view, on_setup, number_focused, now);
        // What the C#'s boxes say when a write or a list fails - a `setParam` returning false or
        // timing out, "Failed to set Param", "Set X Failed", "Error receiving list", the
        // unhandled-exception box - goes on the status line instead: the owner's ruling of
        // 2026-09-25. Each page moves those out of its boxes in its own tick.
        let failures = [
            optional.battery2.take_status(),
            optional.rangefinder.take_status(),
            optional.mount.take_status(),
            optional.adsb.take_status(),
            optional.airspeed.take_status(),
            optional.optflow.take_status(),
        ];
        for status in failures.into_iter().flatten() {
            self.file_status = Some(status);
        }
    }

    /// Battery Monitor 2's handlers that change `Settings.Instance` - Mission Planner's
    /// config.xml, written whole at the next `SaveConfig`, as every page's keys are.
    pub(crate) fn battery2_settings(
        &mut self,
        handler: impl FnOnce(&mut battery_monitor2::BatteryMonitor2, &mut crate::settings::Persisted),
    ) {
        handler(&mut self.optional.battery2, &mut self.persisted);
    }

    /// The box or question one of the six is showing, over the whole window.
    pub(crate) fn optional_overlay(
        &self,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> Option<AnyElement> {
        let optional = &self.optional;
        let focus = &self.optional_focus;
        adsb::overlay(&optional.adsb, focus, window, cx)
            .or_else(|| battery_monitor2::overlay(&optional.battery2, focus, window, cx))
            .or_else(|| rangefinder::overlay(&optional.rangefinder, window, cx))
            .or_else(|| airspeed::overlay(&optional.airspeed, window, cx))
            .or_else(|| optical_flow::overlay(&optional.optflow, window, cx))
            .or_else(|| mount::overlay(&optional.mount, window, cx))
    }
}

#[cfg(test)]
pub mod tests {
    use std::cell::RefCell;
    use std::collections::BTreeMap;

    use super::*;

    /// A link that answers each name as told, and remembers what was written.
    #[derive(Default)]
    pub struct Answering {
        pub written: RefCell<Vec<(String, f64)>>,
        pub outcomes: BTreeMap<String, Progress>,
        /// No vehicle: every write returns `None`.
        pub closed: bool,
    }

    impl Answering {
        pub fn new(outcomes: &[(&str, Progress)]) -> Self {
            Self {
                written: RefCell::new(Vec::new()),
                outcomes: outcomes
                    .iter()
                    .map(|(name, progress)| ((*name).to_owned(), *progress))
                    .collect(),
                closed: false,
            }
        }

        pub fn taken(&self) -> Vec<(String, f64)> {
            self.written.borrow().clone()
        }
    }

    impl ParamWriter for Answering {
        type Handle = usize;

        fn write(&self, name: &str, value: f64) -> Option<usize> {
            if self.closed {
                return None;
            }
            let mut written = self.written.borrow_mut();
            written.push((name.to_owned(), value));
            Some(written.len() - 1)
        }

        fn progress(&self, handle: usize) -> Progress {
            let written = self.written.borrow();
            let Some((name, _)) = written.get(handle) else {
                return Progress::Lost;
            };
            self.outcomes
                .get(name.as_str())
                .copied()
                .unwrap_or(Progress::Finished(RequestOutcome::Accepted { value: None }))
        }
    }

    /// Runs a queue until it is empty, returning the boxes it put up.
    pub fn drain(queue: &mut SetQueue<usize>, link: &Answering) -> Vec<Message> {
        let mut messages = VecDeque::new();
        for _ in 0..100 {
            queue.advance(link, &mut messages);
            if queue.pending() == 0 {
                break;
            }
        }
        messages.into_iter().collect()
    }

    /// `advance_split`: a handler's box ahead of its calls is a box; a call's false, its throw
    /// and the job's `catch` are link failures, for the status line.
    #[test]
    fn a_handlers_box_is_a_box_and_a_failed_call_a_status_line() {
        let link = Answering::new(&[
            ("B", Progress::Finished(RequestOutcome::UnknownParameter)),
            ("C", Progress::Finished(RequestOutcome::TimedOut)),
        ]);
        let mut queue = SetQueue::<usize>::default();
        let refused = error("refused");
        let mut caught = Job::new(
            "caught",
            [Set::caught("C", 3.0, "C threw"), Set::plain("D", 4.0)],
        );
        caught.on_throw = Some(|param: &str| plain(format!("catch {param}")));
        queue.push([
            Job::show("typed", refused.clone()),
            Job::control(Write::other("B", 2.0)),
            caught,
        ]);
        let (mut boxes, mut failures) = (VecDeque::new(), Vec::new());
        while queue.pending() > 0 {
            queue.advance_split(&link, &mut boxes, &mut failures);
        }
        assert_eq!(boxes, [refused]);
        assert_eq!(
            failures,
            [error("Set B Failed"), error("C threw"), plain("catch C")]
        );
        assert_eq!(failure_status(&failures).as_deref(), Some("catch C"));
        assert_eq!(failure_status(&[]), None);
        assert_eq!(
            link.taken(),
            [("B".to_owned(), 2.0), ("C".to_owned(), 3.0)],
            "D never sent"
        );
    }

    /// `InputBox`'s list: the key is the caption and question's letters and digits; the answer
    /// URL-encoded as `WebUtility.UrlEncode` does, after an empty entry the list had.
    #[test]
    fn an_input_box_keeps_its_answer_as_the_csharp_does() {
        assert_eq!(
            answers_key("Battery Level", "What Voltage do you want to warn at?"),
            "InputBoxBatteryLevelWhatVoltagedoyouwanttowarnat"
        );
        assert_eq!(
            url_encode("a b,c{d}-_.!*()é/"),
            "a+b%2Cc%7Bd%7D-_.!*()%C3%A9%2F"
        );
        let mut settings = crate::settings::Persisted::at(None);
        let key = answers_key("T", "Q?");
        remember_answer(&mut settings, "T", "Q?", "x y");
        assert_eq!(settings.get(&key), Some("x+y"));
        // The earlier answer is not a suggestion: only empty entries are kept.
        remember_answer(&mut settings, "T", "Q?", "z");
        assert_eq!(settings.get(&key), Some("z"));
        settings.set(&key, "a;;b");
        remember_answer(&mut settings, "T", "Q?", "z");
        assert_eq!(settings.get(&key), Some(";z"));
        remember_answer(&mut settings, "T", "Q?", "");
        assert_eq!(settings.get(&key), Some(""));
        // An untitled box keeps nothing.
        remember_answer(&mut settings, "", "Q?", "w");
        assert_eq!(settings.get(&answers_key("", "Q?")), None);
    }

    #[test]
    fn a_job_runs_its_calls_in_order_one_at_a_time() {
        let link = Answering::new(&[]);
        let mut queue = SetQueue::<usize>::default();
        queue.push([
            Job::new("a", [Set::plain("A", 1.0), Set::plain("B", 2.0)]),
            Job::new("b", [Set::plain("C", 3.0)]),
        ]);
        let mut messages = VecDeque::new();
        let events = queue.advance(&link, &mut messages);
        assert_eq!(
            link.taken(),
            [
                ("A".to_owned(), 1.0),
                ("B".to_owned(), 2.0),
                ("C".to_owned(), 3.0)
            ]
        );
        assert!(events.contains(&Event::Done {
            tag: "a",
            threw: false
        }));
        assert_eq!(queue.last(), Some("C 3 accepted"));
        assert_eq!(queue.pending(), 0);
    }

    #[test]
    fn a_throw_ends_a_job_with_one_try_and_only_its_call_with_one_each() {
        let timed_out = Progress::Finished(RequestOutcome::TimedOut);
        let link = Answering::new(&[("A", timed_out)]);
        let mut queue = SetQueue::<usize>::default();
        let mut job = Job::new("one", [Set::caught("A", 1.0, "A!"), Set::plain("B", 2.0)]);
        job.on_throw = Some(|param| plain(format!("caught {param}")));
        queue.push([job]);
        let messages = drain(&mut queue, &link);
        assert_eq!(link.taken(), [("A".to_owned(), 1.0)], "B is not reached");
        assert_eq!(messages, [error("A!"), plain("caught A")]);

        let link = Answering::new(&[("A", timed_out)]);
        let mut queue = SetQueue::<usize>::default();
        let mut job = Job::new("each", [Set::caught("A", 1.0, "A!"), Set::plain("B", 2.0)]);
        job.each_caught = true;
        queue.push([job]);
        let messages = drain(&mut queue, &link);
        assert_eq!(link.taken().len(), 2, "B is still made");
        assert_eq!(messages, [error("A!")]);
    }

    #[test]
    fn a_false_shows_only_what_the_call_says_for_one() {
        let unknown = Progress::Finished(RequestOutcome::UnknownParameter);
        let link = Answering::new(&[("A", unknown), ("B", unknown)]);
        let mut queue = SetQueue::<usize>::default();
        queue.push([Job::new(
            "x",
            [Set::plain("A", 1.0), Set::control(Write::combo("B", 2.0))],
        )]);
        let messages = drain(&mut queue, &link);
        assert_eq!(messages, [error("Set B Failed!")]);
        assert_eq!(queue.last(), Some("B 2 false"));
        // No vehicle is a false too: `setParam` finds no such name.
        let mut closed = Answering::new(&[]);
        closed.closed = true;
        let mut queue = SetQueue::<usize>::default();
        queue.push([Job::control(Write::other("C", 1.0))]);
        assert_eq!(drain(&mut queue, &closed), [error("Set C Failed")]);
    }

    #[test]
    fn a_box_before_the_calls_shows_when_the_job_starts() {
        let link = Answering::new(&[]);
        let mut queue = SetQueue::<usize>::default();
        queue.push([Job::show("m", error(FEATURE_NOT_ENABLED))]);
        assert_eq!(drain(&mut queue, &link), [error(FEATURE_NOT_ENABLED)]);
    }
}
