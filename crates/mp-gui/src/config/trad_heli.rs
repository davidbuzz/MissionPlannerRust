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

//! Heli Setup: `GCSViews/ConfigurationView/ConfigTradHeli4.cs`, the Mandatory Hardware entry
//! Initial Setup's list adds for a traditional helicopter once every parameter is in
//! (`GCSViews/InitialSetup.cs:186-187`: `ConfigTradHeli` is commented out, and this is the page).
//!
//! What it shows, in a `FlowLayoutPanel` that wraps them to the page's width: "Servo Setup", a
//! Designer table of eight servo rows - the number, `SERVOn_FUNCTION` as a combo of its documented
//! values, `SERVOn_MIN`, `SERVOn_MAX` and `SERVOn_TRIM` as numbers of 800 to 2200, and
//! `SERVOn_REVERSED` as a check box - then four tables `Activate` fills from lists of names:
//! "Swashplate Setup" (thirteen `H_SV_MAN` .. `H_COL_LAND_MIN`), "Throttle Settings" (twelve
//! `H_RSC_*`), "Governor Settings" (nine `H_RSC_GOV_*`) and "Misc Settings" (`IM_STB_COL_1..4`,
//! the tail and `H_COLYAW`). Each row of those is a label - the documented display name, else the
//! parameter's name, and its units in brackets - and a `MavlinkComboBox` or a
//! `MavlinkNumericUpDown` bound to the parameter, the label and the control carrying its
//! description as a tooltip (`ConfigTradHeli4.cs:28-172`). `Deactivate` empties the four tables
//! (`:187-193`); the servo rows are the Designer's and stay.
//!
//! Every control writes its own parameter, as a `Mavlink*` control does: a combo or a check box
//! when it changes, a number 300 ms after it changes; there is no save button. A control whose
//! parameter the vehicle lacks is disabled: a servo row's controls by their `setup`, a table's
//! combo by its constructor with an empty list, a table's number by its constructor.
//!
//! The servo table is at the Designer's `Location`s inside its group; the four tables are
//! `AutoSize`, so here each is a label column as wide as its widest label beside a column of
//! controls at their default sizes - a combo 121 by 21, a number 120 by 20 - in rows of 27, the
//! control and its margins.
//!
//! Where this differs from the C#, and why:
//!
//! * a failed write ("Set X Failed", "Set X Failed!") goes on the status line, not in a box: the
//!   owner's ruling of 2026-09-25, as `extra_setup::link_error` has it;
//! * a table's number whose write timer is running when `Deactivate` empties its table writes at
//!   once: the C#'s control is taken off the table, not disposed, and its timer writes 300 ms on;
//! * `Visible = false` around `Activate` (`:30, 171`), which only hides the page while it is
//!   built, is not kept: the page is built between two frames;
//! * the flow panel's own scroll bar: the page scrolls, with the groups in it.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{
    AnyElement, AnyView, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div,
    prelude::*, px, rgb,
};

use super::optional::{Job, Set, SetQueue, message_box};
use super::rover_tuning::Tip;
use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::config::flight_modes::ParamWriter;
use crate::config::servo_output::{
    Column, Combo, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE, Question,
    ServoRow, Setup, Write, check_box, combo_box, dropdown, label, modal, number_box, value_of,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list: `backstageViewPagetradheli.Text`.
/// `// C#: GCSViews/InitialSetup.cs:187; GCSViews/InitialSetup.resx:891-893`
pub const TITLE: &str = "Heli Setup";

/// The class, as the list names it.
pub const CLASS: &str = "ConfigTradHeli4";

/// How many servo rows the Designer has.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:32-47`
pub const SERVOS: usize = 8;

/// `uitype`: the control a table's row gets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ui {
    /// A `MavlinkComboBox`.
    Combo,
    /// A `MavlinkNumericUpDown`.
    Num,
}

/// `ItemInfo`: a parameter and its control.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:22-26`
pub type ItemInfo = (&'static str, Ui);

/// `swashplatelist`.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:88-103`
pub const SWASHPLATE: [ItemInfo; 13] = [
    ("H_SV_MAN", Ui::Combo),
    ("H_SW_TYPE", Ui::Combo),
    ("H_SW_COL_DIR", Ui::Combo),
    ("H_SW_LIN_SVO", Ui::Combo),
    ("H_FLYBAR_MODE", Ui::Combo),
    ("H_CYC_MAX", Ui::Num),
    ("H_COL_MAX", Ui::Num),
    ("H_COL_MID", Ui::Num),
    ("H_COL_MIN", Ui::Num),
    ("H_COL_ANG_MIN", Ui::Num),
    ("H_COL_ANG_MAX", Ui::Num),
    ("H_COL_ZERO_THRST", Ui::Num),
    ("H_COL_LAND_MIN", Ui::Num),
];

/// `throttlelist`.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:111-125`
pub const THROTTLE: [ItemInfo; 12] = [
    ("H_RSC_MODE", Ui::Combo),
    ("H_RSC_CRITICAL", Ui::Num),
    ("H_RSC_RAMP_TIME", Ui::Num),
    ("H_RSC_RUNUP_TIME", Ui::Num),
    ("H_RSC_CLDWN_TIME", Ui::Num),
    ("H_RSC_SETPOINT", Ui::Num),
    ("H_RSC_IDLE", Ui::Num),
    ("H_RSC_THRCRV_0", Ui::Num),
    ("H_RSC_THRCRV_25", Ui::Num),
    ("H_RSC_THRCRV_50", Ui::Num),
    ("H_RSC_THRCRV_75", Ui::Num),
    ("H_RSC_THRCRV_100", Ui::Num),
];

/// `governor`.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:133-144`
pub const GOVERNOR: [ItemInfo; 9] = [
    ("H_RSC_GOV_COMP", Ui::Num),
    ("H_RSC_GOV_SETPNT", Ui::Num),
    ("H_RSC_GOV_DISGAG", Ui::Num),
    ("H_RSC_GOV_DROOP", Ui::Num),
    ("H_RSC_GOV_FF", Ui::Num),
    ("H_RSC_GOV_TCGAIN", Ui::Num),
    ("H_RSC_GOV_RANGE", Ui::Num),
    ("H_RSC_GOV_RPM", Ui::Num),
    ("H_RSC_GOV_TORQUE", Ui::Num),
];

/// `misc`.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:151-164`
pub const MISC: [ItemInfo; 9] = [
    ("IM_STB_COL_1", Ui::Num),
    ("IM_STB_COL_2", Ui::Num),
    ("IM_STB_COL_3", Ui::Num),
    ("IM_STB_COL_4", Ui::Num),
    ("H_TAIL_TYPE", Ui::Combo),
    ("H_TAIL_SPEED", Ui::Num),
    ("H_GYR_GAIN", Ui::Num),
    ("H_GYR_GAIN_ACRO", Ui::Num),
    ("H_COLYAW", Ui::Num),
];

/// `setup(0, 0, 1, 1, new[] { a.name }, ...)`: a table's numbers take their range from the
/// documentation or none.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:78`
const TABLE_SETUP: Setup = Setup {
    minimum: 0.0,
    maximum: 0.0,
    scale: 1.0,
    increment: 1.0,
};

/// The four tables `Activate` fills, in the flow panel's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Table {
    /// `groupBoxswash`, `tableLayoutPanel5`.
    Swash,
    /// `groupBoxthrot`, `tableLayoutPanel3`.
    Throttle,
    /// `groupBoxgover`, `tableLayoutPanel2`.
    Governor,
    /// `groupBoxmisc`, `tableLayoutPanel1`.
    Misc,
}

impl Table {
    /// The four, as `flowLayoutPanel1` holds their groups.
    /// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs (flowLayoutPanel1.Controls.Add)`
    pub const ALL: [Self; 4] = [Self::Swash, Self::Throttle, Self::Governor, Self::Misc];

    /// The group box's `Text`.
    /// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs (groupBox*.Text)`
    #[must_use]
    pub const fn caption(self) -> &'static str {
        match self {
            Self::Swash => "Swashplate Setup",
            Self::Throttle => "Throttle Settings",
            Self::Governor => "Governor Settings",
            Self::Misc => "Misc Settings",
        }
    }

    /// The word a fact and an id carry.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Swash => "swash",
            Self::Throttle => "throttle",
            Self::Governor => "governor",
            Self::Misc => "misc",
        }
    }

    /// The list `Activate` fills it from.
    #[must_use]
    pub const fn items(self) -> &'static [ItemInfo] {
        match self {
            Self::Swash => &SWASHPLATE,
            Self::Throttle => &THROTTLE,
            Self::Governor => &GOVERNOR,
            Self::Misc => &MISC,
        }
    }

    const fn index(self) -> usize {
        match self {
            Self::Swash => 0,
            Self::Throttle => 1,
            Self::Governor => 2,
            Self::Misc => 3,
        }
    }
}

/// A table row's control.
#[derive(Debug)]
pub enum Control {
    /// A `MavlinkComboBox`.
    Combo(Combo),
    /// A `MavlinkNumericUpDown`.
    Number(Number),
}

/// A table row: its label, its tooltip and its control.
#[derive(Debug)]
pub struct Row {
    /// The parameter.
    pub name: &'static str,
    /// The label's text.
    pub label: String,
    /// The description, the label's and the control's tooltip.
    pub tip: String,
    /// The control.
    pub control: Control,
}

/// `populatetable` for one name: the label - the display name, else the name, and the units in
/// brackets - and the control, set up from the vehicle's parameters.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:51-85`
#[must_use]
pub fn populate(item: ItemInfo, parameters: &[(String, f64)], lookup: Lookup) -> Row {
    let (name, ui) = item;
    let meta = lookup(name);
    let display = meta.map_or("", |meta| meta.display_name);
    let unit = meta.map_or("", |meta| meta.units);
    let desc = meta.map_or("", |meta| meta.description);
    let mut label = if display.is_empty() {
        name.to_owned()
    } else {
        display.to_owned()
    };
    if !unit.is_empty() {
        label.push_str(&format!(" ({unit})"));
    }
    let control = match ui {
        // `setup(new[] { a.name }, paramlist)`: nothing at all for a name the vehicle lacks - no
        // list, no name, the constructor's disabled box. `// C#: Controls/MavlinkComboBox.cs:37-71`
        Ui::Combo => {
            let mut combo = Combo::default();
            if value_of(parameters, name).is_some() {
                combo.setup(options(name, lookup), name, parameters);
            } else {
                name.clone_into(&mut combo.param);
            }
            Control::Combo(combo)
        }
        Ui::Num => {
            let mut number = Number::new(NUMERIC_DEFAULTS);
            number.setup(TABLE_SETUP, name, parameters, lookup);
            Control::Number(number)
        }
    };
    Row {
        name,
        label,
        tip: desc.to_owned(),
        control,
    }
}

/// A control on the page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target {
    /// A servo row's function (row 0 is servo 1).
    Function(usize),
    /// A servo row's number.
    Servo(usize, Column),
    /// A table row's control.
    Item(Table, usize),
}

/// The page object.
#[derive(Debug)]
pub struct TradHeli<H = mp_link::RequestId> {
    made_for: Option<Key>,
    active: bool,
    /// The servo rows, servo 1 first.
    servos: Vec<ServoRow>,
    /// The four tables' rows, in [`Table::ALL`] order; empty until `Activate`, and after
    /// `Deactivate`.
    tables: [Vec<Row>; 4],
    /// The combo whose list is down.
    dropdown: Option<Target>,
    /// The number being typed into.
    editing: Option<Target>,
    /// A number's "Out of range" question.
    question: Option<(Target, Question)>,
    messages: VecDeque<Message>,
    queue: SetQueue<H>,
}

impl<H> Default for TradHeli<H> {
    /// `InitializeComponent`: every servo control disabled, the tables empty.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            servos: (1..=SERVOS)
                .map(|servo| ServoRow::setup_from(servo, NUMERIC_DEFAULTS, &[], |_| None))
                .collect(),
            tables: std::array::from_fn(|_| Vec::new()),
            dropdown: None,
            editing: None,
            question: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

impl<H: Copy> TradHeli<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The servo rows.
    #[must_use]
    pub fn servos(&self) -> &[ServoRow] {
        &self.servos
    }

    /// A table's rows.
    #[must_use]
    pub fn rows(&self, table: Table) -> &[Row] {
        self.tables.get(table.index()).map_or(&[], Vec::as_slice)
    }

    /// A table row by its parameter.
    #[cfg(test)]
    #[must_use]
    pub fn row(&self, name: &str) -> Option<&Row> {
        self.tables.iter().flatten().find(|row| row.name == name)
    }

    /// The number being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<Target> {
        self.editing
    }

    /// The question showing.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
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

    /// The writes the four tables' number timers hold, taken now.
    fn flush_tables(&mut self) -> Vec<Job> {
        self.tables
            .iter_mut()
            .flatten()
            .filter_map(|row| match &mut row.control {
                Control::Number(number) => number.flush(),
                Control::Combo(_) => None,
            })
            .map(Job::control)
            .collect()
    }

    /// The page object let go with its screen: what its numbers' timers held is written.
    fn dispose(&mut self) -> Vec<Job> {
        let mut jobs = self.flush_tables();
        jobs.extend(
            self.servos
                .iter_mut()
                .flat_map(|row| [&mut row.min, &mut row.trim, &mut row.max])
                .filter_map(Number::flush)
                .map(Job::control),
        );
        *self = Self {
            messages: std::mem::take(&mut self.messages),
            queue: std::mem::take(&mut self.queue),
            ..Self::default()
        };
        jobs
    }

    /// Shows the page: a new page object for a new screen, then `Activate` - the eight servo
    /// rows set up, and the four tables filled. Returns the writes a disposed page object's
    /// timers held.
    /// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:28-172`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key, lookup: Lookup) -> Vec<Job> {
        let mut jobs = Vec::new();
        if self.made_for != Some(key) {
            jobs.extend(self.dispose());
            self.made_for = Some(key);
        }
        self.active = true;
        self.dropdown = None;
        // `// C#: :32-47, 174-185`
        self.servos = (1..=SERVOS)
            .map(|servo| ServoRow::setup_from(servo, NUMERIC_DEFAULTS, parameters, lookup))
            .collect();
        // `// C#: :87-169`
        for table in Table::ALL {
            let rows = table
                .items()
                .iter()
                .map(|item| populate(*item, parameters, lookup))
                .collect();
            if let Some(slot) = self.tables.get_mut(table.index()) {
                *slot = rows;
            }
        }
        jobs
    }

    /// `Deactivate`: the four tables emptied, and the number being typed into read first.
    /// Returns the writes the emptied tables' timers held.
    /// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:187-193`
    pub fn deactivate(&mut self, now: Instant) -> Vec<Job> {
        self.leave(now);
        self.active = false;
        self.dropdown = None;
        self.question = None;
        let jobs = self.flush_tables();
        for rows in &mut self.tables {
            rows.clear();
        }
        jobs
    }

    fn combo_mut(&mut self, target: Target) -> Option<&mut Combo> {
        match target {
            Target::Function(row) => self.servos.get_mut(row).map(|row| &mut row.function),
            Target::Item(table, index) => match self
                .tables
                .get_mut(table.index())
                .and_then(|rows| rows.get_mut(index))
                .map(|row| &mut row.control)
            {
                Some(Control::Combo(combo)) => Some(combo),
                _ => None,
            },
            Target::Servo(..) => None,
        }
    }

    /// A combo, by where it is.
    #[must_use]
    pub fn combo(&self, target: Target) -> Option<&Combo> {
        match target {
            Target::Function(row) => self.servos.get(row).map(|row| &row.function),
            Target::Item(table, index) => match self.rows(table).get(index).map(|row| &row.control)
            {
                Some(Control::Combo(combo)) => Some(combo),
                _ => None,
            },
            Target::Servo(..) => None,
        }
    }

    fn number_mut(&mut self, target: Target) -> Option<&mut Number> {
        match target {
            Target::Servo(row, column) => self.servos.get_mut(row).map(|row| match column {
                Column::Min => &mut row.min,
                Column::Trim => &mut row.trim,
                Column::Max => &mut row.max,
            }),
            Target::Item(table, index) => match self
                .tables
                .get_mut(table.index())
                .and_then(|rows| rows.get_mut(index))
                .map(|row| &mut row.control)
            {
                Some(Control::Number(number)) => Some(number),
                _ => None,
            },
            Target::Function(_) => None,
        }
    }

    /// A servo row's Reversed box clicked: the control's own write.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    pub fn click_reversed(&mut self, row: usize, now: Instant) -> Vec<Job> {
        self.leave(now);
        self.dropdown = None;
        self.servos
            .get_mut(row)
            .and_then(|row| row.reversed.click())
            .map(|write| Job::new("reversed", [Set::control(write)]))
            .into_iter()
            .collect()
    }

    /// Drops a combo's list down, or back up.
    pub fn toggle_dropdown(&mut self, target: Target, now: Instant) {
        self.leave(now);
        let open =
            self.dropdown != Some(target) && self.combo(target).is_some_and(|combo| combo.enabled);
        self.dropdown = None;
        if open && let Some(combo) = self.combo_mut(target) {
            combo.open_list();
            self.dropdown = Some(target);
        }
    }

    /// The wheel over a list.
    pub fn scroll_list(&mut self, target: Target, lines: i32) {
        if self.dropdown == Some(target)
            && let Some(combo) = self.combo_mut(target)
        {
            combo.scroll_list(lines);
        }
    }

    /// A row chosen from a list: the control's own write.
    /// `// C#: Controls/MavlinkComboBox.cs:133-200`
    pub fn choose(&mut self, target: Target, key: i64) -> Vec<Job> {
        self.dropdown = None;
        self.combo_mut(target)
            .and_then(|combo| combo.choose(key))
            .map(Job::control)
            .into_iter()
            .collect()
    }

    /// A number clicked into.
    pub fn begin(&mut self, target: Target, now: Instant) {
        if self.editing == Some(target) {
            return;
        }
        self.leave(now);
        self.dropdown = None;
        if self.number_mut(target).is_some_and(|number| number.enabled) {
            self.editing = Some(target);
        }
    }

    /// The number being typed into loses the focus, which reads its text.
    pub fn leave(&mut self, now: Instant) {
        let Some(target) = self.editing.take() else {
            return;
        };
        if let Some(question) = self
            .number_mut(target)
            .and_then(|number| number.commit(now))
        {
            self.question = Some((target, question));
        }
    }

    /// A key for the number being typed into.
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(target) = self.editing else {
            return false;
        };
        let Some(number) = self.number_mut(target) else {
            return false;
        };
        let (handled, question) = number.key(event, now);
        if let Some(question) = question {
            self.question = Some((target, question));
        }
        handled
    }

    /// A number's arrow.
    pub fn step(&mut self, target: Target, up: bool, now: Instant) {
        self.begin(target, now);
        if self.question.is_some() {
            return;
        }
        if let Some(question) = self
            .number_mut(target)
            .and_then(|number| number.step(up, now))
        {
            self.question = Some((target, question));
        }
    }

    /// Types into a number, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, target: Target, text: &str, now: Instant) {
        self.begin(target, now);
        if let Some(number) = self.number_mut(target) {
            number.type_text(text);
        }
    }

    /// The out-of-range question answered.
    pub fn answer(&mut self, yes: bool, now: Instant) {
        let Some((target, question)) = self.question.take() else {
            return;
        };
        if let Some(number) = self.number_mut(target) {
            number.answer(&question, yes, now);
        }
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// The numbers' timers, then the writes as far as the link's answers allow.
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W, now: Instant) {
        let mut due: Vec<Write> = self
            .servos
            .iter_mut()
            .flat_map(|row| [&mut row.min, &mut row.trim, &mut row.max])
            .filter_map(|number| number.due(now))
            .collect();
        due.extend(
            self.tables
                .iter_mut()
                .flatten()
                .filter_map(|row| match &mut row.control {
                    Control::Number(number) => number.due(now),
                    Control::Combo(_) => None,
                }),
        );
        self.queue.push(due.into_iter().map(Job::control));
        let _ = self.queue.advance(writer, &mut self.messages);
    }
}

impl TradHeli {
    /// Once a frame: a page object whose screen has gone is let go, a number the focus has left
    /// is read, the timers, and the writes.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        focused: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            let pending = self.dispose();
            self.queue.push(pending);
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        self.advance(telemetry, now);
    }
}

/// A combo's selection for the facts, "none" when nothing is selected.
fn selection(combo: &Combo) -> String {
    combo
        .selected
        .map_or_else(|| "none".to_owned(), |value| value.to_string())
}

/// Facts a UI test asserts on.
pub fn record_facts<H: Copy>(page: &TradHeli<H>, view: &TelemetryView) {
    use crate::facts::record;
    record("config.heli.active", page.is_active());
    for row in page.servos() {
        let key = format!("config.heli.servo{}", row.servo);
        record(format!("{key}.function"), selection(&row.function));
        record(format!("{key}.function.text"), row.function.text());
        record(format!("{key}.function.enabled"), row.function.enabled);
        record(format!("{key}.reversed"), row.reversed.state.key());
        for column in Column::ALL {
            let number = row.number(column);
            record(format!("{key}.{}", column.key()), number.shown());
            record(format!("{key}.{}.enabled", column.key()), number.enabled);
        }
    }
    for table in Table::ALL {
        let rows = page.rows(table);
        record(format!("config.heli.{}.rows", table.key()), rows.len());
        for row in rows {
            let key = format!("config.heli.{}", row.name);
            record(format!("{key}.label"), &row.label);
            match &row.control {
                Control::Combo(combo) => {
                    record(key.clone(), selection(combo));
                    record(format!("{key}.text"), combo.text());
                    record(format!("{key}.enabled"), combo.enabled);
                    record(format!("{key}.options"), combo.options.len());
                }
                Control::Number(number) => {
                    record(key.clone(), number.shown());
                    record(format!("{key}.enabled"), number.enabled);
                }
            }
        }
    }
    record(
        "config.heli.question",
        page.question()
            .map_or_else(|| "none".to_owned(), Question::text),
    );
    record(
        "config.heli.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record("config.heli.write", page.last_write().unwrap_or("none"));
    record("config.heli.writes.pending", page.pending());
    for (name, value) in view.parameters.iter() {
        let heli = Table::ALL
            .iter()
            .any(|table| table.items().iter().any(|(item, _)| item == name));
        let servo = name.strip_prefix("SERVO").is_some_and(|rest| {
            rest.split_once('_')
                .is_some_and(|(n, _)| n.parse::<usize>().is_ok_and(|n| (1..=SERVOS).contains(&n)))
        });
        if heli || servo {
            record(format!("params.value.{name}"), value);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// `groupBoxservo`'s `Size`, and `tableLayoutPanel4`'s `Location` in it.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs (groupBoxservo.Size, tableLayoutPanel4.Location)`
const SERVO_GROUP: (f32, f32) = (444.0, 256.0);
/// Where the servo table's controls are placed from.
const SERVO_TABLE_AT: (f32, f32) = (3.0, 16.0);
/// The servo table's header labels: `Text` and `Location`.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs (label1..label6)`
const SERVO_HEADERS: [(&str, f32); 6] = [
    ("Servo", 4.0),
    ("Function", 47.0),
    ("Min", 213.0),
    ("Max", 269.0),
    ("Trim", 325.0),
    ("Reversed", 381.0),
];
/// Each servo row's `y`: row 1 at 24, the rows 27 apart.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs (mavlinkComboBoxfunc1..8.Location)`
const SERVO_FIRST_Y: f32 = 24.0;
/// The distance between servo rows.
const SERVO_ROW: f32 = 27.0;
/// The servo row's controls' `x`, and sizes.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs (mavlinkComboBoxfunc1, mavlinkNumericUpDownmin1/max1/trim1, mavlinkCheckBoxrev1)`
const FUNCTION_AT: (f32, f32, f32) = (46.0, 160.0, 21.0);
/// The three numbers' `x`: Min, Max, Trim.
const NUMBER_XS: [(Column, f32); 3] = [
    (Column::Min, 212.0),
    (Column::Max, 268.0),
    (Column::Trim, 324.0),
];
/// A servo number's size.
const SERVO_NUMBER: (f32, f32) = (50.0, 20.0);
/// The Reversed box's `x`.
const REVERSED_X: f32 = 380.0;
/// A table's combo: `ComboBox`'s default size.
const TABLE_COMBO: (f32, f32) = (121.0, 21.0);
/// A table's number: `NumericUpDown`'s default size.
const TABLE_NUMBER: (f32, f32) = (120.0, 20.0);
/// A table row: the control and its three-pixel margins.
const TABLE_ROW: f32 = 27.0;
/// A group box's `MinimumSize`.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs (groupBoxswash.MinimumSize)`
const GROUP_MINIMUM: (f32, f32) = (200.0, 100.0);

/// A box carrying a tooltip when it has one.
fn tipped(id: String, tip: &str) -> gpui::Stateful<Div> {
    let wrapper = div().id(SharedString::from(format!("{id}-tip")));
    if tip.is_empty() {
        return wrapper;
    }
    let tip = SharedString::from(tip.to_owned());
    wrapper.tooltip(move |_window, cx| -> AnyView {
        let tip = tip.clone();
        cx.new(|_| Tip(tip)).into()
    })
}

/// What a number does when used, for the one at `target`.
#[allow(clippy::type_complexity)]
fn handlers(
    target: Target,
) -> NumberHandlers<
    impl Fn(&mut MissionPlanner) + 'static,
    impl Fn(&mut MissionPlanner, &KeyDownEvent) -> bool + 'static,
    impl Fn(&mut MissionPlanner, bool) + 'static,
> {
    NumberHandlers {
        begin: move |this: &mut MissionPlanner| {
            this.software_pages2.heli.begin(target, Instant::now());
        },
        key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
            this.software_pages2.heli.key(event, Instant::now())
        },
        step: move |this: &mut MissionPlanner, up: bool| {
            this.software_pages2.heli.step(target, up, Instant::now());
        },
    }
}

/// A combo's list, dropped down under it.
fn list_under(
    heli: &TradHeli,
    id: &str,
    target: Target,
    (x, y, width): (f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if heli.dropdown != Some(target) {
        return None;
    }
    let combo = heli.combo(target)?;
    Some(dropdown(
        id,
        combo,
        (x, y, width),
        move |this, key| {
            let jobs = this.software_pages2.heli.choose(target, key);
            this.software_pages2.heli.push(jobs);
        },
        move |this, lines| this.software_pages2.heli.scroll_list(target, lines),
        cx,
    ))
}

/// "Servo Setup": the Designer's table in its group box.
fn servo_group(
    heli: &TradHeli,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (tx, ty) = SERVO_TABLE_AT;
    let mut group = group_box(Some(SERVO_GROUP), "Servo Setup");
    for (text, x) in SERVO_HEADERS {
        group = group.child(label(tx + x, ty + 4.0, text));
    }
    for (index, row) in heli.servos().iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let y = ty + SERVO_FIRST_Y + SERVO_ROW * index as f32;
        let servo = row.servo;
        group = group.child(label(tx + 4.0, y + 1.0, servo.to_string()));
        let (fx, fw, fh) = FUNCTION_AT;
        let target = Target::Function(index);
        let id = format!("heli-SERVO{servo}_FUNCTION");
        group = group.child(combo_box(
            id.clone(),
            &row.function,
            (tx + fx, y, fw, fh),
            move |this| {
                this.software_pages2
                    .heli
                    .toggle_dropdown(target, Instant::now());
            },
            cx,
        ));
        group = group.children(list_under(heli, &id, target, (tx + fx, y + fh, fw), cx));
        for (column, x) in NUMBER_XS {
            let target = Target::Servo(index, column);
            group = group.child(number_box(
                format!("heli-SERVO{servo}_{}", column.suffix()),
                row.number(column),
                heli.editing() == Some(target),
                handle,
                (tx + x, y, SERVO_NUMBER.0, SERVO_NUMBER.1),
                handlers(target),
                window,
                cx,
            ));
        }
        group = group.child(check_box(
            format!("heli-SERVO{servo}_REVERSED"),
            &row.reversed,
            "",
            (tx + REVERSED_X, y),
            move |this| {
                let jobs = this
                    .software_pages2
                    .heli
                    .click_reversed(index, Instant::now());
                this.software_pages2.heli.push(jobs);
            },
            cx,
        ));
    }
    group.into_any_element()
}

/// A group box: its border and caption, at a fixed size or as big as what is in it.
fn group_box(size: Option<(f32, f32)>, caption: &'static str) -> Div {
    let group = div()
        .relative()
        .m(px(3.0))
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
                .text_color(rgb(theme::TEXT))
                .child(caption),
        );
    match size {
        Some((width, height)) => group.w(px(width)).h(px(height)).flex_shrink_0(),
        None => group
            .min_w(px(GROUP_MINIMUM.0))
            .min_h(px(GROUP_MINIMUM.1))
            .flex_shrink_0(),
    }
}

/// One of the four tables in its group box: a column of labels beside a column of controls.
fn table_group(
    heli: &TradHeli,
    table: Table,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut labels = div().flex().flex_col();
    let mut controls = div().flex().flex_col();
    for (index, row) in heli.rows(table).iter().enumerate() {
        let id = format!("heli-{}", row.name);
        labels = labels.child(
            tipped(format!("{id}-label"), &row.tip)
                .h(px(TABLE_ROW))
                .px(px(3.0))
                .flex()
                .items_center()
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(row.label.clone()),
        );
        let target = Target::Item(table, index);
        let cell = match &row.control {
            Control::Combo(combo) => {
                let (width, height) = TABLE_COMBO;
                tipped(id.clone(), &row.tip)
                    .relative()
                    .w(px(width + 6.0))
                    .h(px(TABLE_ROW))
                    .child(combo_box(
                        id.clone(),
                        combo,
                        (3.0, 3.0, width, height),
                        move |this| {
                            this.software_pages2
                                .heli
                                .toggle_dropdown(target, Instant::now());
                        },
                        cx,
                    ))
                    .children(list_under(
                        heli,
                        &id,
                        target,
                        (3.0, 3.0 + height, width),
                        cx,
                    ))
            }
            Control::Number(number) => {
                let (width, height) = TABLE_NUMBER;
                tipped(id.clone(), &row.tip)
                    .relative()
                    .w(px(width + 6.0))
                    .h(px(TABLE_ROW))
                    .child(number_box(
                        id.clone(),
                        number,
                        heli.editing() == Some(target),
                        handle,
                        (3.0, 3.0, width, height),
                        handlers(target),
                        window,
                        cx,
                    ))
            }
        };
        controls = controls.child(cell);
    }
    group_box(None, table.caption())
        .child(
            div()
                .pt(px(16.0))
                .px(px(3.0))
                .pb(px(3.0))
                .flex()
                .child(labels)
                .child(controls),
        )
        .into_any_element()
}

/// The page: the flow panel of the five groups, wrapping to the page's width.
/// `// C#: GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs:29-1068`
pub fn page(
    heli: &TradHeli,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !heli.is_active() {
        return div().into_any_element();
    }
    let mut flow = div()
        .flex()
        .flex_row()
        .flex_wrap()
        .items_start()
        .w_full()
        .pt(px(8.0))
        .child(servo_group(heli, handle, window, cx));
    for table in Table::ALL {
        flow = flow.child(table_group(heli, table, handle, window, cx));
    }
    panel(TITLE, flow).into_any_element()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    heli: &TradHeli,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = heli.question() {
        let buttons = vec![
            action(
                "heli-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.software_pages2.heli.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "heli-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.software_pages2.heli.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "heli-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = heli.message()?;
    Some(message_box(
        "heli-message",
        "heli-message-ok",
        message,
        window,
        |this| this.software_pages2.heli.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use mp_link::requests::RequestOutcome;

    use super::*;
    use crate::config::failsafe::CheckState;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::error;
    use crate::config::optional::tests::Answering;
    use crate::config::servo_output::WRITE_DELAY;
    use crate::config_coverage::source::csharp;

    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// A heli's parameters, as SITL's `heli` frame lists some of them.
    fn heli() -> Vec<(String, f64)> {
        table(&[
            ("SERVO1_FUNCTION", 33.0),
            ("SERVO1_MIN", 1000.0),
            ("SERVO1_MAX", 2000.0),
            ("SERVO1_TRIM", 1500.0),
            ("SERVO1_REVERSED", 0.0),
            ("SERVO4_FUNCTION", 36.0),
            ("SERVO4_REVERSED", 1.0),
            ("H_SW_TYPE", 3.0),
            ("H_COL_MAX", 9.0),
            ("H_RSC_MODE", 1.0),
            ("H_RSC_SETPOINT", 70.0),
            ("H_TAIL_TYPE", 0.0),
            ("IM_STB_COL_1", 40.0),
        ])
    }

    fn shown(parameters: &[(String, f64)]) -> TradHeli<usize> {
        let mut page = TradHeli::<usize>::default();
        page.activate(parameters, key(), bundled);
        page
    }

    fn run(page: &mut TradHeli<usize>, link: &Answering, now: Instant) {
        for _ in 0..50 {
            page.advance(link, now);
            if page.pending() == 0 {
                break;
            }
        }
    }

    /// The lists are the C#'s, in its order, with its controls.
    #[test]
    fn the_lists_are_the_csharps() {
        let Some(source) = csharp("GCSViews/ConfigurationView/ConfigTradHeli4.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let mut ours = Vec::new();
        for table in Table::ALL {
            for (name, ui) in table.items() {
                let kind = match ui {
                    Ui::Combo => "Combo",
                    Ui::Num => "Num",
                };
                ours.push(format!(
                    "new ItemInfo {{name = \"{name}\", type = uitype.{kind}}}"
                ));
            }
        }
        let theirs: Vec<String> = source
            .lines()
            .map(str::trim)
            .filter(|line| line.starts_with("new ItemInfo"))
            .map(|line| line.trim_end_matches(',').to_owned())
            .collect();
        assert_eq!(ours, theirs);
    }

    /// The Designer's servo table: its headers, and row 1's controls at their places.
    #[test]
    fn the_servo_table_is_the_designers() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigTradHeli4.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        for (text, x) in SERVO_HEADERS {
            #[allow(clippy::cast_possible_truncation)]
            let x = x as i32;
            assert!(designer.contains(&format!("Location = new System.Drawing.Point({x}, 4);")));
            assert!(designer.contains(&format!(".Text = \"{text}\";")));
        }
        assert!(
            designer.contains("mavlinkComboBoxfunc1.Location = new System.Drawing.Point(46, 24);")
        );
        assert!(
            designer.contains("mavlinkComboBoxfunc8.Location = new System.Drawing.Point(46, 213);")
        );
        assert!(
            designer
                .contains("mavlinkNumericUpDownmin1.Location = new System.Drawing.Point(212, 24);")
        );
        assert!(
            designer
                .contains("mavlinkNumericUpDownmax1.Location = new System.Drawing.Point(268, 24);")
        );
        assert!(
            designer.contains(
                "mavlinkNumericUpDowntrim1.Location = new System.Drawing.Point(324, 24);"
            )
        );
        assert!(
            designer.contains("mavlinkCheckBoxrev1.Location = new System.Drawing.Point(380, 24);")
        );
        assert!(designer.contains("groupBoxservo.Size = new System.Drawing.Size(444, 256);"));
        assert!(
            designer.contains("groupBoxswash.MinimumSize = new System.Drawing.Size(200, 100);")
        );
        for table in Table::ALL {
            assert!(designer.contains(&format!(".Text = \"{}\";", table.caption())));
        }
    }

    /// `Activate` over a heli: the servo rows bound, the tables filled with a label and a
    /// control per name, enabled for what the vehicle has.
    #[test]
    fn activate_binds_what_the_csharp_binds() {
        let page = shown(&heli());
        let servo1 = page.servos().first().expect("servo 1");
        assert!(servo1.function.enabled);
        assert_eq!(servo1.function.selected, Some(33));
        assert_eq!(servo1.min.shown(), "1000");
        assert_eq!(servo1.max.shown(), "2000");
        assert_eq!(servo1.trim.shown(), "1500");
        assert_eq!(servo1.reversed.state, CheckState::Unchecked);
        let servo4 = page.servos().get(3).expect("servo 4");
        assert_eq!(servo4.reversed.state, CheckState::Checked);
        assert!(!servo4.min.enabled, "SERVO4_MIN is not in the list");
        let servo8 = page.servos().get(7).expect("servo 8");
        assert!(!servo8.function.enabled);

        assert_eq!(page.rows(Table::Swash).len(), 13);
        assert_eq!(page.rows(Table::Throttle).len(), 12);
        assert_eq!(page.rows(Table::Governor).len(), 9);
        assert_eq!(page.rows(Table::Misc).len(), 9);

        let sw = page.row("H_SW_TYPE").expect("H_SW_TYPE");
        let Control::Combo(combo) = &sw.control else {
            panic!("a combo");
        };
        assert!(combo.enabled);
        assert_eq!(combo.selected, Some(3));
        assert!(!combo.options.is_empty());
        let meta = bundled("H_SW_TYPE").expect("documented");
        assert_eq!(sw.label, meta.display_name);
        assert_eq!(sw.tip, meta.description);

        // A name the vehicle lacks: the constructor's disabled combo, with no list.
        let man = page.row("H_SV_MAN").expect("H_SV_MAN");
        let Control::Combo(combo) = &man.control else {
            panic!("a combo");
        };
        assert!(!combo.enabled);
        assert!(combo.options.is_empty());

        let col = page.row("H_COL_MAX").expect("H_COL_MAX");
        let Control::Number(number) = &col.control else {
            panic!("a number");
        };
        assert!(number.enabled);
        assert_eq!(number.shown(), "9");
        let unit = bundled("H_COL_MAX").map_or("", |meta| meta.units);
        if !unit.is_empty() {
            assert!(col.label.ends_with(&format!(" ({unit})")));
        }
        let governor = page.row("H_RSC_GOV_RPM").expect("H_RSC_GOV_RPM");
        let Control::Number(number) = &governor.control else {
            panic!("a number");
        };
        assert!(!number.enabled);
    }

    /// The label is the display name, or the name when the documentation has none.
    #[test]
    fn a_label_is_the_display_name_or_the_name() {
        let row = populate(("NOT_A_PARAM", Ui::Num), &[], |_| None);
        assert_eq!(row.label, "NOT_A_PARAM");
        assert_eq!(row.tip, "");
    }

    /// `Deactivate` empties the four tables; the servo rows stay.
    #[test]
    fn deactivate_empties_the_tables() {
        let mut page = shown(&heli());
        let jobs = page.deactivate(Instant::now());
        assert!(jobs.is_empty());
        for table in Table::ALL {
            assert!(page.rows(table).is_empty());
        }
        assert_eq!(page.servos().len(), SERVOS);
        assert!(
            page.servos()
                .first()
                .is_some_and(|row| row.function.enabled)
        );
    }

    /// A combo writes on its change, the Reversed box on its click.
    #[test]
    fn combos_and_the_reversed_box_write_at_once() {
        let mut page = shown(&heli());
        let now = Instant::now();
        let target = Target::Item(Table::Swash, 1);
        page.toggle_dropdown(target, now);
        let jobs = page.choose(target, 0);
        page.push(jobs);
        let jobs = page.click_reversed(0, now);
        page.push(jobs);
        let link = Answering::new(&[]);
        run(&mut page, &link, now);
        assert_eq!(
            link.taken(),
            [
                ("H_SW_TYPE".to_owned(), 0.0),
                ("SERVO1_REVERSED".to_owned(), 1.0)
            ]
        );
    }

    /// A number writes 300 ms after it changes.
    #[test]
    fn a_number_writes_after_the_timer() {
        let mut page = shown(&heli());
        let now = Instant::now();
        let target = Target::Servo(0, Column::Min);
        page.step(target, true, now);
        let link = Answering::new(&[]);
        run(&mut page, &link, now);
        assert!(link.taken().is_empty(), "not yet");
        run(
            &mut page,
            &link,
            now + WRITE_DELAY + Duration::from_millis(1),
        );
        assert_eq!(link.taken(), [("SERVO1_MIN".to_owned(), 1001.0)]);
    }

    /// A table's number whose timer runs when `Deactivate` empties the table still writes.
    #[test]
    fn a_running_timer_writes_when_the_table_is_emptied() {
        let mut page = shown(&heli());
        let now = Instant::now();
        let index = SWASHPLATE
            .iter()
            .position(|(name, _)| *name == "H_COL_MAX")
            .expect("H_COL_MAX");
        page.step(Target::Item(Table::Swash, index), true, now);
        let jobs = page.deactivate(now);
        assert_eq!(jobs.len(), 1);
        page.push(jobs);
        let link = Answering::new(&[]);
        run(&mut page, &link, now);
        assert_eq!(link.taken().len(), 1);
        assert_eq!(
            link.taken().first().map(|(name, _)| name.as_str()),
            Some("H_COL_MAX")
        );
    }

    /// A failed write is the control's "Set X Failed!", a link error the page puts on the status
    /// line.
    #[test]
    fn a_failed_write_is_a_link_error() {
        let mut page = shown(&heli());
        let now = Instant::now();
        let target = Target::Item(Table::Throttle, 0);
        let jobs = page.choose(target, 2);
        page.push(jobs);
        let link = Answering::new(&[("H_RSC_MODE", Progress::Finished(RequestOutcome::TimedOut))]);
        run(&mut page, &link, now);
        assert_eq!(page.message(), Some(&error("Set H_RSC_MODE Failed!")));
        assert!(
            page.message()
                .is_some_and(crate::config::extra_setup::link_error)
        );
    }

    /// A typed value above a number's maximum asks first, as `MavlinkNumericUpDown` does; Yes
    /// takes it and writes it after the timer.
    #[test]
    fn a_value_above_the_maximum_asks() {
        let mut page = shown(&heli());
        let now = Instant::now();
        let target = Target::Servo(0, Column::Max);
        page.type_into(target, "2500", now);
        page.leave(now);
        assert_eq!(
            page.question().map(Question::text).as_deref(),
            Some("SERVO1_MAX Value out of range\nDo you want to accept the new value?")
        );
        page.answer(true, now);
        let link = Answering::new(&[]);
        run(
            &mut page,
            &link,
            now + WRITE_DELAY + Duration::from_millis(1),
        );
        assert_eq!(link.taken(), [("SERVO1_MAX".to_owned(), 2500.0)]);
    }

    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-heli.gui");
        let source = include_str!("trad_heli.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.heli.") => {
                    let what = key.trim_start_matches("config.heli.");
                    let recorded = source.contains(&format!("\"{key}\""))
                        || what.starts_with("servo")
                        || what.starts_with("H_")
                        || what.starts_with("IM_")
                        || what.ends_with(".rows");
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("heli-") => {
                    let drawn = source.contains(&format!("\"{id}\""))
                        || id.starts_with("heli-SERVO")
                        || id.starts_with("heli-H_")
                        || id.starts_with("heli-IM_");
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts >= 4, "{facts} facts");
    }
}
