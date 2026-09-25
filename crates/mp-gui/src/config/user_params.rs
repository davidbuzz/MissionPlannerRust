//! User Params: `GCSViews/ConfigurationView/ConfigUserDefined.cs`, the page CONFIG's list adds as
//! "User Params" once every parameter is in (`GCSViews/SoftwareConfig.cs:219-222`).
//!
//! What it shows: a Modify button, then for each name in the page's list that the vehicle has, the
//! name and - where the parameter's documentation lists values - a `MavlinkComboBox` of them, in a
//! two-column `TableLayoutPanel` that fills the page (`ConfigUserDefined.cs:46-86`). The list is
//! the `UserParams` setting, split at commas, or the RC6 to RC16 options under both their names
//! when there is none; the constructor reads it once (`:11-44`). Each combo writes its parameter
//! when it changes (`Controls/MavlinkComboBox.cs:133-200`). Modify asks for the names in an
//! `InputBox`, one to a line, whose answer - or the text it started with, on Cancel, since the
//! handler does not look at which button closed it - becomes the list, saved as `UserParams`, and
//! the page is built again (`:52-60`).
//!
//! What the C# does that looks wrong, kept as it does it:
//!
//! * a parameter whose documentation lists no values gets a `MavlinkNumericUpDown` that `setup`
//!   binds and nothing adds to the table (`:72-77`): the row is its name alone, and the next
//!   control flows into the cell the number would have taken;
//! * an answer with no names in it leaves `Options` empty and throws from `Aggregate` before the
//!   setting is saved - an `InvalidOperationException`, which `Program.handleException` drops
//!   (`Program.cs:765-770`) - so the page keeps its rows until it is next activated, and Modify
//!   then throws before its `InputBox` opens, as the C#'s does, until the screen is loaded again;
//! * the setting is joined with `Aggregate((a, b) => a.Trim() + "," + b.Trim())`, which leaves a
//!   lone name untrimmed; the names themselves are not trimmed, so " RC7_OPTION" is not found.
//!
//! The layout is the table's: every added control is a new cell, two to a row, the button across
//! both; each cell sized by WinForms' defaults for what it holds (a `Label` 100 x 23, a `ComboBox`
//! 121 x 21, a `MyButton` 75 x 23, each with a margin of 3), the columns and rows sized to their
//! largest. What is not ported: the `InputBox`'s remembered answers (`InputBox.cs:177-181`), which
//! the other ported pages do not carry either.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};

use super::optional::{Job, SetQueue, at, button, label, message_box};
use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::config::flight_modes::ParamWriter;
use crate::config::servo_output::{Combo, Message, combo_box, dropdown, value_of};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, panel, theme};

/// The page's title in CONFIG's list, `Strings.User_Params`.
/// `// C#: GCSViews/SoftwareConfig.cs:221; ExtLibs/Strings/Strings.resx:652-654`
pub const TITLE: &str = "User Params";

/// The class, as the lists name it.
pub const CLASS: &str = "ConfigUserDefined";

/// The setting the list is kept in.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:15-16, 58`
pub const SETTING: &str = "UserParams";

/// `Size`: the table's, which is docked to fill the page.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.Designer.cs:45, 54`
pub const PAGE_SIZE: (f32, f32) = (427.0, 388.0);

/// `Options`' initial value.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:19-44`
pub const DEFAULT_OPTIONS: [&str; 22] = [
    "CH6_OPT",
    "CH7_OPT",
    "CH8_OPT",
    "CH9_OPT",
    "CH10_OPT",
    "CH11_OPT",
    "CH12_OPT",
    "CH13_OPT",
    "CH14_OPT",
    "CH15_OPT",
    "CH16_OPT",
    "RC6_OPTION",
    "RC7_OPTION",
    "RC8_OPTION",
    "RC9_OPTION",
    "RC10_OPTION",
    "RC11_OPTION",
    "RC12_OPTION",
    "RC13_OPTION",
    "RC14_OPTION",
    "RC15_OPTION",
    "RC16_OPTION",
];

/// The button's `Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:52`
pub const MODIFY: &str = "Modify";

/// The `InputBox`'s caption and question.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:56`
pub const INPUT: (&str, &str) = ("Params", "Enter Param Names");

/// WinForms' default margin, on every side of every control.
const MARGIN: f32 = 3.0;
/// A `MyButton`'s default size, `Button.DefaultSize`.
const BUTTON_SIZE: (f32, f32) = (75.0, 23.0);
/// A `Label`'s default size, `Label.DefaultSize`, not auto-sized.
const LABEL_SIZE: (f32, f32) = (100.0, 23.0);
/// A `ComboBox`'s default size, `ComboBox.DefaultSize`.
const COMBO_SIZE: (f32, f32) = (121.0, 21.0);

/// One name the vehicle has: its label, and its combo where its documentation lists values.
#[derive(Debug, Clone)]
pub struct Row {
    /// The name, the label's `Text` and `Name`.
    pub name: String,
    /// The combo, set up with the documented values.
    pub combo: Option<Combo>,
}

/// A control of the table, in the order `LoadOptions` adds them after the button.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cell {
    /// A row's label, by row.
    Label(usize),
    /// A row's combo, by row.
    Combo(usize),
}

/// The page object.
#[derive(Debug)]
pub struct UserParams<H = mp_link::RequestId> {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// `Options`.
    options: Vec<String>,
    /// What `LoadOptions` built.
    rows: Vec<Row>,
    /// The row whose combo's list is down.
    dropdown: Option<usize>,
    /// The `InputBox` showing: its text box, and the text it opened with.
    input: Option<(TextField, String)>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The combos' writes.
    queue: SetQueue<H>,
}

impl<H> Default for UserParams<H> {
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            options: DEFAULT_OPTIONS
                .iter()
                .map(|name| (*name).to_owned())
                .collect(),
            rows: Vec::new(),
            dropdown: None,
            input: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

/// `Aggregate((a, b) => a.Trim() + "," + b.Trim())`: the first name as it is, each later one
/// trimmed and joined to the trimmed total.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:58`
#[must_use]
pub fn joined(options: &[String]) -> Option<String> {
    let (first, rest) = options.split_first()?;
    Some(rest.iter().fold(first.clone(), |total, name| {
        format!("{},{}", total.trim(), name.trim())
    }))
}

/// `opts.Split(new[] { ',', '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries)`.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:57`
#[must_use]
pub fn split(text: &str) -> Vec<String> {
    text.split([',', '\n', '\r'])
        .filter(|name| !name.is_empty())
        .map(str::to_owned)
        .collect()
}

impl<H: Copy> UserParams<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// `Options`.
    #[must_use]
    pub fn options(&self) -> &[String] {
        &self.options
    }

    /// The rows `LoadOptions` built.
    #[must_use]
    pub fn rows(&self) -> &[Row] {
        &self.rows
    }

    /// The `InputBox`'s text, while it shows.
    #[must_use]
    pub fn input(&self) -> Option<&str> {
        self.input.as_ref().map(|(field, _)| field.value())
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// How the last write ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.queue.last()
    }

    /// How many writes are queued or on their way.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.pending()
    }

    /// The table's controls after the button, in the order they were added.
    #[must_use]
    pub fn cells(&self) -> Vec<Cell> {
        let mut cells = Vec::new();
        for (index, row) in self.rows.iter().enumerate() {
            cells.push(Cell::Label(index));
            if row.combo.is_some() {
                cells.push(Cell::Combo(index));
            }
        }
        cells
    }

    /// Where each cell's control sits, and its size: two columns, each as wide as its widest
    /// control and its margins, rows as tall as their tallest; the button's row first.
    #[must_use]
    pub fn layout(&self) -> Vec<(Cell, (f32, f32, f32, f32))> {
        let cells = self.cells();
        let size = |cell: Cell| match cell {
            Cell::Label(_) => LABEL_SIZE,
            Cell::Combo(_) => COMBO_SIZE,
        };
        let mut widths = [0.0_f32; 2];
        for (index, cell) in cells.iter().enumerate() {
            if let Some(width) = widths.get_mut(index % 2) {
                *width = width.max(size(*cell).0 + 2.0 * MARGIN);
            }
        }
        let first_row = BUTTON_SIZE.1 + 2.0 * MARGIN;
        let mut y = first_row;
        let mut out = Vec::new();
        for pair in cells.chunks(2) {
            let height = pair
                .iter()
                .map(|cell| size(*cell).1 + 2.0 * MARGIN)
                .fold(0.0_f32, f32::max);
            let mut x = 0.0;
            for (column, cell) in pair.iter().enumerate() {
                let (width, height) = size(*cell);
                out.push((*cell, (x + MARGIN, y + MARGIN, width, height)));
                x += widths.get(column).copied().unwrap_or(0.0);
            }
            y += height;
        }
        out
    }

    /// The page object: its constructor reads the setting, split at commas.
    /// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:11-17`
    fn construct(&mut self, setting: Option<&str>) {
        if let Some(setting) = setting {
            self.options = setting.split(',').map(str::to_owned).collect();
        }
    }

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:88-91`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        setting: Option<&str>,
        lookup: Lookup,
    ) {
        if self.made_for != Some(key) {
            *self = Self {
                messages: std::mem::take(&mut self.messages),
                queue: std::mem::take(&mut self.queue),
                ..Self::default()
            };
            self.made_for = Some(key);
            self.construct(setting);
        }
        self.active = true;
        self.load_options(parameters, lookup);
    }

    /// `LoadOptions`: the table emptied, then a row for each name the vehicle has.
    /// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:46-86`
    fn load_options(&mut self, parameters: &[(String, f64)], lookup: Lookup) {
        self.dropdown = None;
        self.rows = self
            .options
            .iter()
            .filter(|name| value_of(parameters, name).is_some())
            .map(|name| {
                let values = options(name, lookup);
                // No values: a `MavlinkNumericUpDown` is set up and never added. `// C#: :72-77`
                let combo = (!values.is_empty()).then(|| {
                    let mut combo = Combo::default();
                    combo.setup(values, name, parameters);
                    combo
                });
                Row {
                    name: name.clone(),
                    combo,
                }
            })
            .collect();
    }

    /// `IDeactivate.Deactivate`, which does nothing; the page is hidden.
    /// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:93-96`
    pub fn deactivate(&mut self) {
        self.active = false;
        self.dropdown = None;
    }

    /// Modify clicked: the `InputBox`, holding the names one to a line. With no names,
    /// `Aggregate` throws first, and the exception is dropped.
    /// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:53-56; Program.cs:765-770`
    pub fn modify(&mut self) {
        self.dropdown = None;
        let Some((first, rest)) = self.options.split_first() else {
            return;
        };
        let text = rest
            .iter()
            .fold(first.clone(), |total, name| format!("{total}\r\n{name}"));
        let mut field = TextField::new("");
        field.set(text.clone());
        self.input = Some((field, text));
    }

    /// A key for the `InputBox`'s text box: multiline, so Enter is a new line and Tab a tab
    /// (`AcceptsReturn`, `AcceptsTab`); Escape is Cancel. Returns whether the box closed.
    /// `// C#: ExtLibs/Controls/InputBox.cs:84-91, 145-146`
    pub fn input_key(&mut self, event: &KeyDownEvent) -> Option<bool> {
        let (field, _) = self.input.as_mut()?;
        let keystroke = &event.keystroke;
        let plain = !keystroke.modifiers.control && !keystroke.modifiers.platform;
        match keystroke.key.as_str() {
            "enter" if plain => {
                let text = format!("{}\r\n", field.value());
                field.set(text);
                None
            }
            "tab" if plain => {
                let text = format!("{}\t", field.value());
                field.set(text);
                None
            }
            _ => match field.key(event) {
                KeyOutcome::Cancelled => Some(false),
                KeyOutcome::Changed | KeyOutcome::Submitted | KeyOutcome::Ignored => None,
            },
        }
    }

    /// The `InputBox` closed: OK gives its text, Cancel the text it opened with. The names split
    /// out of it become `Options`; the setting is saved and the page built again - unless there
    /// are none, when `Aggregate` throws first. Returns the setting to save.
    /// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:56-59; ExtLibs/Controls/InputBox.cs:37-60, 173-185`
    pub fn close_input(
        &mut self,
        ok: bool,
        parameters: &[(String, f64)],
        lookup: Lookup,
    ) -> Option<String> {
        let (field, opened_with) = self.input.take()?;
        let text = if ok {
            field.value().to_owned()
        } else {
            opened_with
        };
        self.options = split(&text);
        let setting = joined(&self.options)?;
        self.load_options(parameters, lookup);
        Some(setting)
    }

    /// Drops a row's list down, or back up.
    pub fn toggle_dropdown(&mut self, row: usize) {
        let enabled = self
            .rows
            .get(row)
            .and_then(|row| row.combo.as_ref())
            .is_some_and(|combo| combo.enabled);
        self.dropdown = if self.dropdown == Some(row) || !enabled {
            None
        } else {
            if let Some(combo) = self.rows.get_mut(row).and_then(|row| row.combo.as_mut()) {
                combo.open_list();
            }
            Some(row)
        };
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, row: usize, lines: i32) {
        if self.dropdown == Some(row)
            && let Some(combo) = self.rows.get_mut(row).and_then(|row| row.combo.as_mut())
        {
            combo.scroll_list(lines);
        }
    }

    /// A row chosen: the combo's own write.
    /// `// C#: Controls/MavlinkComboBox.cs:133-200`
    pub fn choose(&mut self, row: usize, key: i64) -> Vec<Job> {
        self.dropdown = None;
        self.rows
            .get_mut(row)
            .and_then(|row| row.combo.as_mut())
            .and_then(|combo| combo.choose(key))
            .map(Job::control)
            .into_iter()
            .collect()
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// The writes, as far as the link's answers allow.
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W) {
        self.queue.advance(writer, &mut self.messages);
    }
}

impl UserParams {
    /// Once a frame: a page object whose screen has gone is let go, and the writes.
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, on_config: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_config || self.made_for != Some(Key::of(view)))
        {
            *self = Self {
                messages: std::mem::take(&mut self.messages),
                queue: std::mem::take(&mut self.queue),
                ..Self::default()
            };
        }
        self.advance(telemetry);
    }
}

/// Facts a UI test asserts on.
pub fn record_facts<H: Copy>(page: &UserParams<H>, view: &TelemetryView) {
    use crate::facts::record;
    record("config.userparams.active", page.is_active());
    record("config.userparams.options", page.options().join(","));
    let rows: Vec<&str> = page.rows().iter().map(|row| row.name.as_str()).collect();
    record("config.userparams.rows", rows.join(","));
    for row in page.rows() {
        let key = format!("config.userparams.{}", row.name);
        match &row.combo {
            Some(combo) => {
                record(format!("{key}.text"), combo.text());
                record(format!("{key}.enabled"), combo.enabled);
                record(
                    key,
                    combo
                        .selected
                        .map_or_else(|| "none".to_owned(), |value| value.to_string()),
                );
            }
            None => record(key, "label"),
        }
        if let Some(value) = value_of(&view.parameters, &row.name) {
            record(format!("params.value.{}", row.name), value);
        }
    }
    record(
        "config.userparams.input",
        page.input()
            .map_or_else(|| "none".to_owned(), |text| text.replace("\r\n", "|")),
    );
    record(
        "config.userparams.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.userparams.write",
        page.last_write().unwrap_or("none"),
    );
    record("config.userparams.writes.pending", page.pending());
}

/// The page: the table's cells at their places.
/// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:46-86; ConfigUserDefined.Designer.cs:29-61`
pub fn page(user: &UserParams, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let layout = user.layout();
    // The table fills the page; it grows here with its rows, as the page does in the C#'s
    // backstage, which is the window's height.
    let bottom = layout
        .iter()
        .map(|(_, (_, y, _, height))| y + height + MARGIN)
        .fold(PAGE_SIZE.1, f32::max);
    let mut body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(bottom))
        .child(button(
            "userparams-Modify",
            MODIFY,
            (MARGIN, MARGIN, BUTTON_SIZE.0, BUTTON_SIZE.1),
            true,
            |this, window, cx| {
                this.software_pages.user.modify();
                this.software_focus.prompt.focus(window, cx);
            },
            cx,
        ));
    for (cell, (x, y, width, height)) in &layout {
        match *cell {
            Cell::Label(index) => {
                let Some(row) = user.rows().get(index) else {
                    continue;
                };
                body = body.child(at(*x, *y, *width, *height).overflow_hidden().child(label(
                    0.0,
                    0.0,
                    row.name.clone(),
                    true,
                )));
            }
            Cell::Combo(index) => {
                let Some(combo) = user.rows().get(index).and_then(|row| row.combo.as_ref()) else {
                    continue;
                };
                body = body.child(combo_box(
                    format!("userparams-{}", combo.param),
                    combo,
                    (*x, *y, *width, *height),
                    move |this| this.software_pages.user.toggle_dropdown(index),
                    cx,
                ));
                if user.dropdown == Some(index) {
                    body = body.child(dropdown(
                        &format!("userparams-{}", combo.param),
                        combo,
                        (*x, y + height, *width),
                        move |this, key| {
                            let jobs = this.software_pages.user.choose(index, key);
                            this.software_pages.user.push(jobs);
                        },
                        move |this, lines| this.software_pages.user.scroll_list(index, lines),
                        cx,
                    ));
                }
            }
        }
    }
    panel(TITLE, body).into_any_element()
}

/// The `InputBox` or the message box showing, over the whole window.
/// `// C#: ExtLibs/Controls/InputBox.cs:62-170`
pub fn overlay(
    user: &UserParams,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(text) = user.input() {
        let focused = handle.is_focused(window);
        let close = |ok: bool| {
            move |this: &mut MissionPlanner,
                  _event: &(),
                  _window: &mut Window,
                  cx: &mut Context<MissionPlanner>| {
                this.user_params_close_input(ok);
                cx.notify();
            }
        };
        let buttons = vec![
            action(
                "userparams-input-ok",
                "OK",
                theme::ACCENT,
                true,
                cx.listener(close(true)),
            ),
            action(
                "userparams-input-cancel",
                "Cancel",
                theme::DIM,
                true,
                cx.listener(close(false)),
            ),
        ];
        // The text box: multiline, 372 x 400, a line of the text to a line of the box.
        let lines: Vec<String> = text
            .split('\n')
            .map(|line| line.trim_end_matches('\r').to_owned())
            .collect();
        let last = lines.len().saturating_sub(1);
        let box_lines = lines.into_iter().enumerate().map(|(index, line)| {
            div()
                .flex()
                .items_center()
                .min_h(px(14.0))
                .whitespace_nowrap()
                .child(line)
                .children(
                    (focused && index == last)
                        .then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))),
                )
        });
        let refocus = handle.clone();
        let field = crate::probe::measured("userparams-input-value", div())
            .id("userparams-input-value")
            .track_focus(handle)
            .on_click(move |_event, window, cx| refocus.focus(window, cx))
            .key_context("TextField")
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                this.user_params_input_key(event);
                cx.notify();
            }))
            .w(px(372.0))
            .h(px(400.0))
            .p_1()
            .overflow_hidden()
            .flex()
            .flex_col()
            .rounded_sm()
            .border_1()
            .border_color(rgb(if focused {
                theme::ACCENT
            } else {
                theme::BORDER
            }))
            .bg(rgb(theme::ACTION))
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .cursor_text()
            .children(box_lines);
        let dialog = crate::probe::measured("userparams-input", div())
            .flex()
            .flex_col()
            .gap_2()
            .w(px(396.0))
            .p_3()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::ACCENT))
            .rounded_md()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(INPUT.0))
            .child(div().text_sm().text_color(rgb(theme::TEXT)).child(INPUT.1))
            .child(field)
            .child(div().flex().justify_end().gap_2().children(buttons));
        let size = window.viewport_size();
        return Some(
            gpui::deferred(
                gpui::anchored()
                    .position(gpui::point(px(0.0), px(0.0)))
                    .child(
                        div()
                            .id(SharedString::from("userparams-input-backdrop"))
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
            .into_any_element(),
        );
    }
    let message = user.message()?;
    Some(message_box(
        "userparams-message",
        "userparams-message-ok",
        message,
        window,
        |this| this.software_pages.user.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use gpui::{Keystroke, Modifiers};
    use mp_link::requests::RequestOutcome;

    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::error;
    use crate::config::optional::tests::Answering;
    use crate::config_coverage::source::csharp;

    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// Another screen: a vehicle heard from.
    fn other_key() -> Key {
        let mut view = TelemetryView::disconnected("other");
        view.vehicle = Some(mp_vehicle::VehicleId::new(1, 1));
        Key::of(&view)
    }

    /// SITL's copter: RC6 to RC16's options, under the new names only.
    fn sitl() -> Vec<(String, f64)> {
        (6..=16)
            .map(|channel| {
                (
                    format!("RC{channel}_OPTION"),
                    if channel == 7 { 7.0 } else { 0.0 },
                )
            })
            .chain([("RTL_ALT_M".to_owned(), 15.0)])
            .collect()
    }

    fn shown(setting: Option<&str>) -> UserParams<usize> {
        let mut page = UserParams::<usize>::default();
        page.activate(&sitl(), key(), setting, bundled);
        page
    }

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

    fn type_text(page: &mut UserParams<usize>, text: &str) {
        for c in text.chars() {
            let s = c.to_string();
            page.input_key(&press(&s, Some(&s)));
        }
    }

    fn clear(page: &mut UserParams<usize>) {
        let mut event = press("u", Some("u"));
        event.keystroke.modifiers.control = true;
        page.input_key(&event);
    }

    /// The default list is the C#'s.
    #[test]
    fn the_default_list_is_the_csharps() {
        let Some(source) = csharp("GCSViews/ConfigurationView/ConfigUserDefined.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let start = source.find("Options { get; set; }").expect("Options");
        let end = start + source[start..].find("};").expect("its end");
        let names: Vec<&str> = source[start..end].split('"').skip(1).step_by(2).collect();
        assert_eq!(names, DEFAULT_OPTIONS);
    }

    /// With no setting: a row for each default name the vehicle has, each a combo of the
    /// documented values showing the vehicle's.
    #[test]
    fn the_default_list_on_the_sitl_copter() {
        let page = shown(None);
        let rows: Vec<&str> = page.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(
            rows,
            [
                "RC6_OPTION",
                "RC7_OPTION",
                "RC8_OPTION",
                "RC9_OPTION",
                "RC10_OPTION",
                "RC11_OPTION",
                "RC12_OPTION",
                "RC13_OPTION",
                "RC14_OPTION",
                "RC15_OPTION",
                "RC16_OPTION"
            ]
        );
        let rc7 = page.rows()[1].combo.as_ref().expect("RC7_OPTION's values");
        assert!(rc7.enabled);
        assert_eq!(rc7.selected, Some(7));
        assert_eq!(rc7.text(), "Save WP");
        // The button's row, then a label and a combo to each row.
        let layout = page.layout();
        assert_eq!(layout.len(), 22);
        assert_eq!(layout[0], (Cell::Label(0), (3.0, 32.0, 100.0, 23.0)));
        assert_eq!(layout[1], (Cell::Combo(0), (109.0, 32.0, 121.0, 21.0)));
        assert_eq!(layout[2], (Cell::Label(1), (3.0, 61.0, 100.0, 23.0)));
    }

    /// The setting, split at commas and untrimmed; a name with no values is its label alone, and
    /// the next label flows into the cell its number would have had.
    #[test]
    fn the_setting_is_the_list_and_a_number_is_never_added() {
        let page = shown(Some("RTL_ALT_M,RC7_OPTION, RC8_OPTION,NOPE"));
        let rows: Vec<&str> = page.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(
            rows,
            ["RTL_ALT_M", "RC7_OPTION"],
            "\" RC8_OPTION\" is not found"
        );
        assert!(page.rows()[0].combo.is_none());
        assert_eq!(
            page.cells(),
            [Cell::Label(0), Cell::Label(1), Cell::Combo(1)]
        );
        let layout = page.layout();
        // RTL_ALT_M's label, then RC7_OPTION's beside it, its combo on the next row.
        assert_eq!(layout[0].1, (3.0, 32.0, 100.0, 23.0));
        // The first column is as wide as its widest control, the combo below.
        assert_eq!(layout[1].1, (130.0, 32.0, 100.0, 23.0));
        assert_eq!(layout[2].1, (3.0, 61.0, 121.0, 21.0));
    }

    /// Choosing a value writes it, as `MavlinkComboBox` does.
    #[test]
    fn choosing_a_value_writes_it() {
        let mut page = shown(None);
        page.toggle_dropdown(2);
        let jobs = page.choose(2, 9);
        page.push(jobs);
        let link = Answering::new(&[]);
        page.advance(&link);
        assert_eq!(link.taken(), [("RC8_OPTION".to_owned(), 9.0)]);
        assert_eq!(page.last_write(), Some("RC8_OPTION 9 accepted"));
        let timed_out =
            Answering::new(&[("RC8_OPTION", Progress::Finished(RequestOutcome::TimedOut))]);
        let jobs = page.choose(2, 0);
        page.push(jobs);
        page.advance(&timed_out);
        assert_eq!(page.message(), Some(&error("Set RC8_OPTION Failed!")));
    }

    /// Modify: the names one to a line; OK makes the list what was typed, saves it and builds
    /// the page again.
    #[test]
    fn modify_takes_the_typed_names() {
        let mut page = shown(Some("RC7_OPTION,RC8_OPTION"));
        page.modify();
        assert_eq!(page.input(), Some("RC7_OPTION\r\nRC8_OPTION"));
        clear(&mut page);
        type_text(&mut page, "RC9_OPTION");
        assert_eq!(page.input_key(&press("enter", None)), None, "a new line");
        type_text(&mut page, "RC7_OPTION,RTL_ALT_M");
        assert_eq!(page.input(), Some("RC9_OPTION\r\nRC7_OPTION,RTL_ALT_M"));
        let setting = page.close_input(true, &sitl(), bundled);
        assert_eq!(setting.as_deref(), Some("RC9_OPTION,RC7_OPTION,RTL_ALT_M"));
        assert_eq!(page.options(), ["RC9_OPTION", "RC7_OPTION", "RTL_ALT_M"]);
        let rows: Vec<&str> = page.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(rows, ["RC9_OPTION", "RC7_OPTION", "RTL_ALT_M"]);
        assert!(page.input().is_none());
    }

    /// Cancel still saves and rebuilds: the handler ignores the answer, and the text is the one
    /// the box opened with. Escape is Cancel.
    #[test]
    fn cancel_saves_what_the_box_opened_with() {
        let mut page = shown(Some("RC7_OPTION , RC8_OPTION"));
        page.modify();
        type_text(&mut page, "RC9_OPTION");
        assert_eq!(page.input_key(&press("escape", None)), Some(false));
        let setting = page.close_input(false, &sitl(), bundled);
        assert_eq!(setting.as_deref(), Some("RC7_OPTION,RC8_OPTION"));
        assert_eq!(page.options(), ["RC7_OPTION ", " RC8_OPTION"]);
    }

    /// A lone name is saved untrimmed, as `Aggregate` leaves one element.
    #[test]
    fn a_lone_name_is_saved_as_it_is() {
        assert_eq!(
            joined(&[" RC7_OPTION ".to_owned()]).as_deref(),
            Some(" RC7_OPTION ")
        );
        assert_eq!(
            joined(&["A ".to_owned(), " B".to_owned(), "C ".to_owned()]).as_deref(),
            Some("A,B,C")
        );
        assert_eq!(joined(&[]), None);
        assert_eq!(split("A,,B\r\n\r\nC"), ["A", "B", "C"]);
    }

    /// No names: `Aggregate` throws before the setting is saved; the rows stay until the page is
    /// next activated, and Modify then opens nothing.
    #[test]
    fn no_names_saves_nothing_and_modify_then_throws() {
        let mut page = shown(None);
        page.modify();
        clear(&mut page);
        assert_eq!(page.close_input(true, &sitl(), bundled), None);
        assert!(page.options().is_empty());
        assert_eq!(page.rows().len(), 11, "not built again");
        page.modify();
        assert!(page.input().is_none());
        page.activate(&sitl(), key(), None, bundled);
        assert!(page.rows().is_empty());
        // A new page object reads the setting again.
        page.deactivate();
        let other = other_key();
        page.activate(&sitl(), other, Some("RC7_OPTION"), bundled);
        assert_eq!(page.rows().len(), 1);
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-user-params.gui");
        let source = include_str!("user_params.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.userparams.") => {
                    // A row's facts are under its parameter's name.
                    let rest = key.trim_start_matches("config.userparams.");
                    let row = rest.split('.').next().unwrap_or(rest);
                    let parameter = row
                        .chars()
                        .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_');
                    assert!(
                        source.contains(&format!("\"{key}\"")) || parameter,
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("userparams-") => {
                    let id = id.split(':').next().unwrap_or(id);
                    let control = id.trim_start_matches("userparams-");
                    let control = control.split('-').next().unwrap_or(control);
                    assert!(
                        source.contains(&format!("\"{id}\"")) || control.ends_with("_OPTION"),
                        "{id} is not drawn"
                    );
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 6 && clicks >= 3, "{facts} facts, {clicks} clicks");
    }
}
