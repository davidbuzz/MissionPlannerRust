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

//! Ctrl+W's Propagation Settings: `Controls/PropagationSettings.cs`, which `MainV2.ProcessCmdKey`
//! opens with `new PropagationSettings().Show()` (`MainV2.cs:4147-4152`). The settings of the
//! map's propagation overlay (`ExtLibs/Maps/Propagation.cs`): which layers it draws and how. Every
//! control is a `config.xml` key - the check boxes through `Maps.Propagation`'s properties, which
//! read and write their own keys (`Propagation.cs:68-111`) - so the form needs nothing of the
//! overlay, which is not ported (its own ledger row); what it writes waits for it.
//!
//! What it shows, as `PropagationSettings.resx` places it in a 694 x 237 tool window: Elevation,
//! Terrain and RF Map down the left, Home Dist Left, Drone Dist Left and Altitude Filter beside
//! them; under them the "Map Overlay" group - Show Scale and Clearance [m], Resolution, Azimuth
//! Step, Convergance (sic), Range [Km], Base Height [m], Tolerance, and the altitude filter's Min
//! Alt and Max Alt. The numbers have one decimal place and step by 0.1, from 0 to 2000 (Tolerance
//! to 1); the three lists are drop-down lists.
//!
//! What it does (`PropagationSettings.cs:40-64, 434-513`):
//!
//! * the constructor sets each control from its key - `GetFloat` with its default for a number,
//!   `GetInt32` as text for a list, `GetBoolean` for a box - in its order, and each control's
//!   handler writes its key back when that changes it: a number not 0, a list whose text is an
//!   item, a box whose key is not true. A number out of its range is the `Value` setter's throw,
//!   and the form does not open; what was written before it stays;
//! * the boxes come from the Designer `Checked` and `Indeterminate`; setting `Checked` true
//!   leaves that alone, so a key that is true shows the box indeterminate, and false unticks it;
//!   a click ticks an unticked box and unticks any other, and writes "True" or "False";
//! * a number's `ValueChanged` writes `Value.ToString()`; a list's `SelectedIndexChanged` its text.
//!
//! Where this is not the C#, each at its site:
//!
//! * what the constructor throws is said on the status line, by the owner's ruling of 2026-09-25;
//! * `chk_showscale_CheckedChanged`'s `Console.WriteLine` is not written;
//! * the form is drawn over the window, modal, as the other windows here are; a second Ctrl+W
//!   opens a fresh one in its place;
//! * the colours are this application's.
//!
//! `// C#: Controls/PropagationSettings.cs:1-514; Controls/PropagationSettings.resx`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, FocusHandle, KeyDownEvent, Window, div, prelude::*, px, rgb};
use mp_mission::dotnet::{Decimal, bool_text, parse_f32};

use crate::MissionPlanner;
use crate::config::optional::{group, label};
use crate::config::servo_output::{Check as CheckBox, CheckState};
use crate::settings::Persisted;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// `$this.Text` and `$this.ClientSize`. `// C#: Controls/PropagationSettings.resx`
pub const FORM_TEXT: &str = "Propagation Settings";
pub const CLIENT: (f32, f32) = (694.0, 237.0);

/// `groupBox1`: its place and `Text`. `// C#: Controls/PropagationSettings.resx (groupBox1)`
pub const GROUP_AT: (f32, f32, f32, f32) = (13.0, 82.0, 674.0, 152.0);
pub const GROUP_TEXT: &str = "Map Overlay";

/// The group's labels: name, `Text`, `Location` in the group.
/// `// C#: Controls/PropagationSettings.resx (label*)`
pub const LABELS: [(&str, &str, (f32, f32)); 13] = [
    ("label89", "Clearance [m]", (97.0, 22.0)),
    ("label91", "Elevation", (16.0, 51.0)),
    ("label90", "Resolution", (97.0, 48.0)),
    ("label100", "Propagation", (16.0, 75.0)),
    ("label109", "Azimuth Step", (97.0, 75.0)),
    ("label110", "Convergance", (236.0, 75.0)),
    ("label111", "Range [Km]", (372.0, 75.0)),
    ("label112", "Base Height [m]", (511.0, 75.0)),
    ("label113", "Kmleft", (18.0, 101.0)),
    ("label114", "Tolerance", (97.0, 101.0)),
    ("label3", "Altitude Filter", (16.0, 126.0)),
    ("label1", "Min Alt", (97.0, 126.0)),
    ("label2", "Max Alt", (236.0, 126.0)),
];

/// The value the `Value` setter throws over, as `ArgumentOutOfRangeException` says it.
fn out_of_range(value: Decimal) -> String {
    format!(
        "Value of '{}' is not valid for 'Value'. 'Value' should be between 'Minimum' and 'Maximum'.",
        value.to_text()
    )
}

/// `(decimal)float`'s `OverflowException`.
pub const DECIMAL_OVERFLOW: &str = "Value was either too large or too small for a Decimal.";

/// `NumericUpDown.DecimalPlaces` and `Increment` (1 at scale 1), the same for all six.
/// `// C#: Controls/PropagationSettings.cs:198-203, 230-235, 247-252, 341-346, 363-368, 385-390`
const PLACES: u32 = 1;
const INCREMENT: Decimal = Decimal::new(1, 1);

/// The six numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Number {
    /// `Clearance`.
    Clearance,
    /// `NUM_range`.
    Range,
    /// `NUM_height`.
    Height,
    /// `Tolerance`.
    Tolerance,
    /// `NUM_min`.
    Min,
    /// `NUM_max`.
    Max,
}

impl Number {
    /// In the constructor's order.
    pub const ALL: [Self; 6] = [
        Self::Clearance,
        Self::Range,
        Self::Height,
        Self::Tolerance,
        Self::Min,
        Self::Max,
    ];

    /// The Designer's name, which the probe and the facts carry after `propagation-`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Clearance => "Clearance",
            Self::Range => "NUM_range",
            Self::Height => "NUM_height",
            Self::Tolerance => "Tolerance",
            Self::Min => "NUM_min",
            Self::Max => "NUM_max",
        }
    }

    /// The probe id.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Clearance => "propagation-Clearance",
            Self::Range => "propagation-NUM_range",
            Self::Height => "propagation-NUM_height",
            Self::Tolerance => "propagation-Tolerance",
            Self::Min => "propagation-NUM_min",
            Self::Max => "propagation-NUM_max",
        }
    }

    /// The key its `ValueChanged` writes, and the constructor's `GetFloat` default.
    /// `// C#: Controls/PropagationSettings.cs:46-54, 465-508`
    #[must_use]
    pub const fn setting(self) -> (&'static str, f32) {
        match self {
            Self::Clearance => ("Propagation_Clearance", 5.0),
            Self::Range => ("Propagation_Range", 2.0),
            Self::Height => ("Propagation_Height", 2.0),
            Self::Tolerance => ("Propagation_Tolerance", 0.8),
            Self::Min => ("Propagation_Minalt", 100.0),
            Self::Max => ("Propagation_Maxalt", 400.0),
        }
    }

    /// `Maximum`: 2000, but Tolerance's 1; `Minimum` is 0 for all.
    /// `// C#: Controls/PropagationSettings.cs:205-209, 392-396`
    #[must_use]
    pub const fn maximum(self) -> Decimal {
        match self {
            Self::Tolerance => Decimal::new(1, 0),
            _ => Decimal::new(2000, 0),
        }
    }

    /// Its place in the group. `// C#: Controls/PropagationSettings.resx`
    const fn place(self) -> (f32, f32, f32, f32) {
        match self {
            Self::Clearance => (174.0, 19.0, 56.0, 20.0),
            Self::Range => (442.0, 73.0, 56.0, 20.0),
            Self::Height => (601.0, 73.0, 46.0, 20.0),
            Self::Tolerance => (174.0, 99.0, 56.0, 20.0),
            Self::Min => (174.0, 124.0, 56.0, 20.0),
            Self::Max => (317.0, 124.0, 56.0, 20.0),
        }
    }
}

/// The three drop-down lists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Combo {
    /// `CMB_Resolution`.
    Resolution,
    /// `CMB_Rotational`.
    Rotational,
    /// `CMB_Angular`.
    Angular,
}

impl Combo {
    /// In the constructor's order.
    pub const ALL: [Self; 3] = [Self::Resolution, Self::Rotational, Self::Angular];

    /// The Designer's name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Resolution => "CMB_Resolution",
            Self::Rotational => "CMB_Rotational",
            Self::Angular => "CMB_Angular",
        }
    }

    /// The probe id.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Resolution => "propagation-CMB_Resolution",
            Self::Rotational => "propagation-CMB_Rotational",
            Self::Angular => "propagation-CMB_Angular",
        }
    }

    /// The key its `SelectedIndexChanged` writes, and the constructor's `GetInt32` default.
    /// `// C#: Controls/PropagationSettings.cs:47-49, 470-483`
    #[must_use]
    pub const fn setting(self) -> (&'static str, i32) {
        match self {
            Self::Resolution => ("Propagation_Resolution", 4),
            Self::Rotational => ("Propagation_Rotational", 1),
            Self::Angular => ("Propagation_Converge", 1),
        }
    }

    /// `Items`. `// C#: Controls/PropagationSettings.resx (CMB_*.Items*)`
    #[must_use]
    pub const fn items(self) -> [&'static str; 5] {
        match self {
            Self::Resolution => ["2", "4", "6", "8", "10"],
            Self::Rotational => ["0.5", "1", "2", "5", "10"],
            Self::Angular => ["0", "1", "5", "10", "15"],
        }
    }

    /// Its place in the group, 40 by 21.
    const fn place(self) -> (f32, f32, f32, f32) {
        match self {
            Self::Resolution => (174.0, 45.0, 40.0, 21.0),
            Self::Rotational => (174.0, 72.0, 40.0, 21.0),
            Self::Angular => (317.0, 72.0, 40.0, 21.0),
        }
    }
}

/// The seven check boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Check {
    /// `chk_showscale`, in the group.
    ShowScale,
    /// `chk_ele`.
    Elevation,
    /// `chk_terrain`.
    Terrain,
    /// `chk_rf`.
    Rf,
    /// `chk_homedist`.
    HomeDist,
    /// `chk_dronedist`.
    DroneDist,
    /// `chk_setalt`.
    SetAlt,
}

impl Check {
    /// In the constructor's order.
    pub const ALL: [Self; 7] = [
        Self::ShowScale,
        Self::Elevation,
        Self::Terrain,
        Self::Rf,
        Self::HomeDist,
        Self::DroneDist,
        Self::SetAlt,
    ];

    /// The `.resx`'s name for it - `cb_showscale` for `chk_showscale` - and its `Text`.
    #[must_use]
    pub const fn resx(self) -> (&'static str, &'static str) {
        match self {
            Self::ShowScale => ("cb_showscale", "Show Scale"),
            Self::Elevation => ("chk_ele", "Elevation"),
            Self::Terrain => ("chk_terrain", "Terrain"),
            Self::Rf => ("chk_rf", "RF Map"),
            Self::HomeDist => ("chk_homedist", "Home Dist Left"),
            Self::DroneDist => ("chk_dronedist", "Drone Dist Left"),
            Self::SetAlt => ("chk_setalt", "Altitude Filter"),
        }
    }

    /// The key `Maps.Propagation`'s property reads and writes.
    /// `// C#: ExtLibs/Maps/Propagation.cs:68-111`
    #[must_use]
    pub const fn setting(self) -> &'static str {
        match self {
            Self::ShowScale => "Propagation_ShowScale",
            Self::Elevation => "Propagation_Elemap",
            Self::Terrain => "Propagation_Termap",
            Self::Rf => "Propagation_RFmap",
            Self::HomeDist => "Propagation_home_kmleft",
            Self::DroneDist => "Propagation_drone_kmleft",
            Self::SetAlt => "Propagation_Setalt",
        }
    }

    /// Its `Location`: Show Scale's in the group, the others' on the form.
    /// `// C#: Controls/PropagationSettings.resx`
    const fn place(self) -> (f32, f32) {
        match self {
            Self::ShowScale => (8.0, 22.0),
            Self::Elevation => (13.0, 13.0),
            Self::Terrain => (13.0, 36.0),
            Self::Rf => (13.0, 59.0),
            Self::HomeDist => (107.0, 13.0),
            Self::DroneDist => (107.0, 36.0),
            Self::SetAlt => (107.0, 60.0),
        }
    }

    /// The probe id.
    fn id(self) -> String {
        format!("propagation-{}", self.resx().0)
    }
}

/// `Settings.GetFloat(key, default)`: `float.TryParse`, else the default.
/// `// C#: ExtLibs/Utilities/Settings.cs:234-243`
fn get_float(settings: &Persisted, key: &str, default: f32) -> f32 {
    settings.get(key).and_then(parse_f32).unwrap_or(default)
}

/// The form, while it is open.
#[derive(Debug)]
pub struct Form {
    /// Each number's `Value`, by [`Number::ALL`]'s order.
    values: [Decimal; 6],
    /// The number being typed into, and its text.
    pub editing: Option<(Number, TextField)>,
    /// Each list's `SelectedIndex`.
    selected: [Option<usize>; 3],
    /// The list dropped down.
    pub open: Option<Combo>,
    /// Each box's `CheckState`.
    checks: [CheckState; 7],
}

impl Form {
    /// `new PropagationSettings()`: the Designer's controls - numbers 0, no list item chosen,
    /// each box checked and indeterminate - then each set from its key, the handlers writing the
    /// keys they change. What the `Value` setter or the `decimal` conversion throws, the form not
    /// made.
    /// `// C#: Controls/PropagationSettings.cs:40-64, 107-407`
    pub fn new(settings: &mut Persisted) -> Result<Self, String> {
        let mut form = Self {
            values: [Decimal::ZERO; 6],
            editing: None,
            selected: [None; 3],
            open: None,
            checks: [CheckState::Indeterminate; 7],
        };
        let read = |number: Number, settings: &Persisted| {
            let (key, default) = number.setting();
            Decimal::from_f32(get_float(settings, key, default))
                .ok_or_else(|| DECIMAL_OVERFLOW.to_owned())
        };
        // `Clearance.Value` first, then the three lists, then the other five numbers.
        let clearance = read(Number::Clearance, settings)?;
        form.set_value(Number::Clearance, clearance, settings)?;
        for combo in Combo::ALL {
            let (key, default) = combo.setting();
            let text = crate::raw_params_grid::get_int32(settings.get(key), default).to_string();
            form.set_text(combo, &text, settings);
        }
        for number in [
            Number::Range,
            Number::Height,
            Number::Tolerance,
            Number::Min,
            Number::Max,
        ] {
            let value = read(number, settings)?;
            form.set_value(number, value, settings)?;
        }
        for check in Check::ALL {
            let on = crate::raw_params::get_boolean(settings.get(check.setting()));
            form.set_checked(check, on, settings);
        }
        Ok(form)
    }

    /// A number's `Value`.
    #[must_use]
    pub fn value(&self, number: Number) -> Decimal {
        Number::ALL
            .iter()
            .zip(self.values)
            .find_map(|(each, value)| (*each == number).then_some(value))
            .unwrap_or(Decimal::ZERO)
    }

    fn value_mut(&mut self, number: Number) -> Option<&mut Decimal> {
        Number::ALL
            .iter()
            .zip(self.values.iter_mut())
            .find_map(|(each, value)| (*each == number).then_some(value))
    }

    /// What a number's box shows: what is being typed, else `Value` at one place.
    #[must_use]
    pub fn shown(&self, number: Number) -> String {
        match &self.editing {
            Some((editing, field)) if *editing == number => field.value().to_owned(),
            _ => self.value(number).to_fixed(PLACES),
        }
    }

    /// The `Value` setter: outside 0 to `Maximum` the throw; changed, `ValueChanged`, which
    /// writes `Value.ToString()`.
    /// `// C#: Controls/PropagationSettings.cs:465-468, 485-508`
    fn set_value(
        &mut self,
        number: Number,
        value: Decimal,
        settings: &mut Persisted,
    ) -> Result<(), String> {
        if value < Decimal::ZERO || value > number.maximum() {
            return Err(out_of_range(value));
        }
        let Some(held) = self.value_mut(number) else {
            return Ok(());
        };
        if *held == value {
            return Ok(());
        }
        *held = value;
        settings.set(number.setting().0, value.to_text());
        Ok(())
    }

    /// A list's `SelectedIndex`.
    #[must_use]
    pub fn selected(&self, combo: Combo) -> Option<usize> {
        Combo::ALL
            .iter()
            .zip(self.selected)
            .find_map(|(each, index)| (*each == combo).then_some(index))
            .flatten()
    }

    /// A list's `Text`: its item, or nothing.
    #[must_use]
    pub fn text(&self, combo: Combo) -> &'static str {
        self.selected(combo)
            .and_then(|index| combo.items().get(index).copied())
            .unwrap_or("")
    }

    /// `SelectedIndex = index`: changed, `SelectedIndexChanged`, which writes the text.
    /// `// C#: Controls/PropagationSettings.cs:470-483`
    fn select(&mut self, combo: Combo, index: usize, settings: &mut Persisted) {
        let Some(held) = Combo::ALL
            .iter()
            .zip(self.selected.iter_mut())
            .find_map(|(each, held)| (*each == combo).then_some(held))
        else {
            return;
        };
        if *held == Some(index) {
            return;
        }
        *held = Some(index);
        settings.set(combo.setting().0, self.text(combo));
    }

    /// A drop-down list's `Text = text`: the item with that text chosen; none, nothing.
    /// `// C#: Controls/PropagationSettings.cs:47-49`
    fn set_text(&mut self, combo: Combo, text: &str, settings: &mut Persisted) {
        if let Some(index) = combo.items().iter().position(|item| *item == text) {
            self.select(combo, index, settings);
        }
    }

    /// A box's `CheckState`.
    #[must_use]
    pub fn check(&self, check: Check) -> CheckState {
        Check::ALL
            .iter()
            .zip(self.checks)
            .find_map(|(each, state)| (*each == check).then_some(state))
            .unwrap_or(CheckState::Unchecked)
    }

    /// Sets a box's state; when `Checked` - anything but unticked - changes, `CheckedChanged`,
    /// which writes the property's key.
    /// `// C#: Controls/PropagationSettings.cs:434-463, 510-513; ExtLibs/Maps/Propagation.cs:68-111`
    fn set_state(&mut self, check: Check, state: CheckState, settings: &mut Persisted) {
        let Some(held) = Check::ALL
            .iter()
            .zip(self.checks.iter_mut())
            .find_map(|(each, held)| (*each == check).then_some(held))
        else {
            return;
        };
        let was = *held != CheckState::Unchecked;
        *held = state;
        let now = state != CheckState::Unchecked;
        if was != now {
            settings.set(check.setting(), bool_text(now));
        }
    }

    /// `Checked = on`: true leaves an indeterminate box as it is, being `Checked` already.
    fn set_checked(&mut self, check: Check, on: bool, settings: &mut Persisted) {
        let checked = self.check(check) != CheckState::Unchecked;
        if checked == on {
            return;
        }
        let state = if on {
            CheckState::Checked
        } else {
            CheckState::Unchecked
        };
        self.set_state(check, state, settings);
    }

    /// A click: an unticked box ticked, any other unticked (`ThreeState` is false).
    pub fn click(&mut self, check: Check, settings: &mut Persisted) {
        self.commit(settings);
        let state = match self.check(check) {
            CheckState::Unchecked => CheckState::Checked,
            CheckState::Checked | CheckState::Indeterminate => CheckState::Unchecked,
        };
        self.set_state(check, state, settings);
    }

    /// A list's arrow: its items dropped down, or put away.
    pub fn toggle(&mut self, combo: Combo, settings: &mut Persisted) {
        self.commit(settings);
        self.open = if self.open == Some(combo) {
            None
        } else {
            Some(combo)
        };
    }

    /// An item chosen from the list dropped down.
    pub fn choose(&mut self, combo: Combo, index: usize, settings: &mut Persisted) {
        self.open = None;
        if index < combo.items().len() {
            self.select(combo, index, settings);
        }
    }

    /// A number's box clicked into: its text to type over, the caret after it.
    pub fn begin(&mut self, number: Number, settings: &mut Persisted) {
        if self
            .editing
            .as_ref()
            .is_some_and(|(editing, _)| *editing == number)
        {
            return;
        }
        self.leave(settings);
        self.open = None;
        let mut field = TextField::new("");
        field.set(self.value(number).to_fixed(PLACES));
        self.editing = Some((number, field));
    }

    /// `ParseEditText`: what `decimal.Parse` reads of the typed text, held to the range, as
    /// `Value`; nothing when it does not parse. The box shows `Value` again.
    /// `// C#: Controls/PropagationSettings.cs:465-508 (ValueChanged)`
    fn commit(&mut self, settings: &mut Persisted) {
        let Some((number, typed)) = self
            .editing
            .as_ref()
            .map(|(number, field)| (*number, Decimal::parse(field.value())))
        else {
            return;
        };
        if let Some(value) = typed {
            let value = if value < Decimal::ZERO {
                Decimal::ZERO
            } else if value > number.maximum() {
                number.maximum()
            } else {
                value
            };
            // Within the range, so the setter cannot throw.
            let _ = self.set_value(number, value, settings);
        }
        let shown = self.value(number).to_fixed(PLACES);
        if let Some((_, field)) = self.editing.as_mut() {
            field.set(shown);
        }
    }

    /// The keyboard left the box: its text read.
    pub fn leave(&mut self, settings: &mut Persisted) {
        self.commit(settings);
        self.editing = None;
    }

    /// An arrow, `UpButton` or `DownButton`: the typed text read, then `Increment` more or less,
    /// held to the range.
    pub fn step(&mut self, number: Number, up: bool, settings: &mut Persisted) {
        self.commit(settings);
        self.open = None;
        let value = self.value(number);
        let stepped = if up {
            value
                .checked_add(INCREMENT)
                .filter(|value| *value <= number.maximum())
                .unwrap_or_else(|| number.maximum())
        } else {
            value
                .checked_sub(INCREMENT)
                .filter(|value| *value >= Decimal::ZERO)
                .unwrap_or(Decimal::ZERO)
        };
        let _ = self.set_value(number, stepped, settings);
        let shown = self.value(number).to_fixed(PLACES);
        if let Some((_, field)) = self
            .editing
            .as_mut()
            .filter(|(editing, _)| *editing == number)
        {
            field.set(shown);
        }
    }

    /// A key in the number being typed into: the arrow keys step (`InterceptArrowKeys`), Enter
    /// reads the text, anything else is typing.
    pub fn key(&mut self, event: &KeyDownEvent, settings: &mut Persisted) -> bool {
        let Some(number) = self.editing.as_ref().map(|(number, _)| *number) else {
            return false;
        };
        match event.keystroke.key.as_str() {
            "up" | "down" => {
                self.step(number, event.keystroke.key == "up", settings);
                return true;
            }
            _ => {}
        }
        let outcome = self
            .editing
            .as_mut()
            .map_or(KeyOutcome::Ignored, |(_, field)| field.key(event));
        match outcome {
            KeyOutcome::Changed => true,
            KeyOutcome::Submitted => {
                self.commit(settings);
                true
            }
            KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
        }
    }
}

/// The window, held for the application.
#[derive(Debug, Default)]
pub struct Propagation {
    /// The form, while it is open.
    pub window: Option<Form>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl Propagation {
    /// Ctrl+W: a fresh form, or what its constructor threw.
    pub fn show(&mut self, settings: &mut Persisted) -> Result<(), String> {
        let form = Form::new(settings)?;
        self.window = Some(form);
        self.opened += 1;
        Ok(())
    }

    /// The close box: a number being typed into is read first, as leaving it reads it.
    pub fn close(&mut self, settings: &mut Persisted) {
        if let Some(form) = self.window.as_mut() {
            form.leave(settings);
        }
        self.window = None;
    }
}

/// Every key the form reads and writes.
#[must_use]
pub fn settings_keys() -> Vec<&'static str> {
    Number::ALL
        .iter()
        .map(|number| number.setting().0)
        .chain(Combo::ALL.iter().map(|combo| combo.setting().0))
        .chain(Check::ALL.iter().map(|check| check.setting()))
        .collect()
}

/// Facts a UI test asserts on, under `propagation.`: each control as it shows, and each key as
/// the dictionary holds it.
pub fn record_facts(holder: &Propagation, settings: &Persisted) {
    use crate::facts::record;
    record("propagation.window", holder.window.is_some());
    record("propagation.opened", holder.opened);
    for key in settings_keys() {
        record(
            format!("propagation.setting.{key}"),
            settings.get(key).unwrap_or("none"),
        );
    }
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    for number in Number::ALL {
        record(format!("propagation.{}", number.name()), form.shown(number));
    }
    for combo in Combo::ALL {
        let text = form.text(combo);
        record(
            format!("propagation.{}", combo.name()),
            if text.is_empty() { "none" } else { text },
        );
    }
    for check in Check::ALL {
        record(
            format!("propagation.{}", check.resx().0),
            form.check(check).key(),
        );
    }
    record("propagation.list", form.open.map_or("none", Combo::name));
    record(
        "propagation.editing",
        form.editing
            .as_ref()
            .map_or("none", |(number, _)| number.name()),
    );
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// How the drawing reaches the form and the settings it writes.
fn with_form(this: &mut MissionPlanner, act: impl FnOnce(&mut Form, &mut Persisted)) {
    if let Some(form) = this.key_forms.propagation.window.as_mut() {
        act(form, &mut this.persisted);
    }
}

/// Once a frame: a number the keyboard has left is read.
pub fn tick(
    holder: &mut Propagation,
    settings: &mut Persisted,
    number: &FocusHandle,
    window: &Window,
) {
    if let Some(form) = holder.window.as_mut()
        && form.editing.is_some()
        && !number.is_focused(window)
    {
        form.leave(settings);
    }
}

/// A number: its text - typed into, with a caret, while it has the keyboard - and its arrows,
/// `<id>-up` and `<id>-down`.
fn number_box(
    form: &Form,
    number: Number,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let editing = form
        .editing
        .as_ref()
        .is_some_and(|(editing, _)| *editing == number);
    super::form_box(
        number.id(),
        form.shown(number),
        number.place(),
        true,
        editing,
        true,
        handle,
        super::BoxHandlers {
            begin: move |this: &mut MissionPlanner| {
                with_form(this, |form, settings| form.begin(number, settings));
            },
            key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                let mut taken = false;
                with_form(this, |form, settings| taken = form.key(event, settings));
                taken
            },
            step: move |this: &mut MissionPlanner, up: bool| {
                with_form(this, |form, settings| form.step(number, up, settings));
            },
        },
        window,
        cx,
    )
}

/// A check box at its place.
fn check_box(form: &Form, check: Check, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut state = CheckBox::default();
    state.state = form.check(check);
    state.enabled = true;
    crate::config::servo_output::check_box(
        check.id(),
        &state,
        check.resx().1,
        check.place(),
        move |this| with_form(this, |form, settings| form.click(check, settings)),
        cx,
    )
}

/// A drop-down list at its place, its items under it while it is dropped down.
fn combo_box(form: &Form, combo: Combo, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let items: Vec<String> = combo
        .items()
        .iter()
        .map(|item| (*item).to_owned())
        .collect();
    crate::config::serial_output::combo(
        combo.id(),
        form.text(combo),
        &items,
        form.open == Some(combo),
        true,
        combo.place(),
        move |this| with_form(this, |form, settings| form.toggle(combo, settings)),
        move |this, index| with_form(this, |form, settings| form.choose(combo, index, settings)),
        cx,
    )
}

/// The form over the window: the six boxes down the left, and the group under them.
/// `// C#: Controls/PropagationSettings.cs:66-432; Controls/PropagationSettings.resx`
pub fn overlay(
    holder: &Propagation,
    number: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let mut inside = group(GROUP_AT, GROUP_TEXT, true);
    for (_, text, (x, y)) in LABELS {
        inside = inside.child(label(x, y, text, true));
    }
    inside = inside.child(check_box(form, Check::ShowScale, cx));
    for each in Number::ALL {
        inside = inside.child(number_box(form, each, number, window, cx));
    }
    // The lists last, the lowest first, so the one dropped down is drawn over what is under it.
    for combo in [Combo::Angular, Combo::Rotational, Combo::Resolution] {
        inside = inside.child(combo_box(form, combo, cx));
    }
    let mut client = div().relative().w(px(CLIENT.0)).h(px(CLIENT.1));
    for check in Check::ALL.into_iter().skip(1) {
        client = client.child(check_box(form, check, cx));
    }
    client = client.child(inside);
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "propagation-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.key_forms.propagation.close(&mut this.persisted);
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("propagation", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("propagation-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings(pairs: &[(&str, &str)]) -> Persisted {
        let mut settings = Persisted::at(None);
        for (key, value) in pairs {
            settings.set(key, *value);
        }
        settings
    }

    fn get(settings: &Persisted, key: &str) -> Option<String> {
        settings.get(key).map(ToOwned::to_owned)
    }

    /// No keys: each control at its default, every number and list writing its key back, and
    /// every box unticked - its key false - and writing "False".
    #[test]
    fn opening_with_no_keys_writes_the_defaults() {
        let mut settings = settings(&[]);
        let form = Form::new(&mut settings).expect("the form");
        let shown: Vec<String> = Number::ALL
            .iter()
            .map(|number| form.shown(*number))
            .collect();
        assert_eq!(shown, ["5.0", "2.0", "2.0", "0.8", "100.0", "400.0"]);
        assert_eq!(
            get(&settings, "Propagation_Clearance").as_deref(),
            Some("5")
        );
        assert_eq!(
            get(&settings, "Propagation_Tolerance").as_deref(),
            Some("0.8")
        );
        assert_eq!(get(&settings, "Propagation_Maxalt").as_deref(), Some("400"));
        assert_eq!(form.text(Combo::Resolution), "4");
        assert_eq!(form.text(Combo::Rotational), "1");
        assert_eq!(form.text(Combo::Angular), "1");
        assert_eq!(
            get(&settings, "Propagation_Resolution").as_deref(),
            Some("4")
        );
        assert_eq!(get(&settings, "Propagation_Converge").as_deref(), Some("1"));
        for check in Check::ALL {
            assert_eq!(form.check(check), CheckState::Unchecked, "{check:?}");
            assert_eq!(get(&settings, check.setting()).as_deref(), Some("False"));
        }
    }

    /// Keys held: a true box shows indeterminate and writes nothing; a number at 0 writes
    /// nothing, being `Value` already; `GetInt32` of "0.5" is not a number, so the default.
    #[test]
    fn opening_reads_the_keys() {
        let mut settings = settings(&[
            ("Propagation_Clearance", "0"),
            ("Propagation_Range", "12.5"),
            ("Propagation_Rotational", "0.5"),
            ("Propagation_Converge", "3"),
            ("Propagation_Elemap", "True"),
            ("Propagation_RFmap", "true"),
        ]);
        let form = Form::new(&mut settings).expect("the form");
        assert_eq!(form.shown(Number::Clearance), "0.0");
        assert_eq!(
            get(&settings, "Propagation_Clearance").as_deref(),
            Some("0")
        );
        assert_eq!(form.shown(Number::Range), "12.5");
        assert_eq!(form.text(Combo::Rotational), "1");
        assert_eq!(
            get(&settings, "Propagation_Rotational").as_deref(),
            Some("1")
        );
        // 3 is no item: nothing chosen, nothing written.
        assert_eq!(form.selected(Combo::Angular), None);
        assert_eq!(get(&settings, "Propagation_Converge").as_deref(), Some("3"));
        assert_eq!(form.check(Check::Elevation), CheckState::Indeterminate);
        assert_eq!(form.check(Check::Rf), CheckState::Indeterminate);
        assert_eq!(
            get(&settings, "Propagation_Elemap").as_deref(),
            Some("True")
        );
        assert_eq!(form.check(Check::Terrain), CheckState::Unchecked);
    }

    /// A key out of a number's range is the `Value` setter's throw: no form, and what was written
    /// before it stays.
    #[test]
    fn a_number_out_of_range_is_the_setters_throw() {
        let mut settings = settings(&[("Propagation_Tolerance", "1.5")]);
        let mut holder = Propagation::default();
        let error = holder.show(&mut settings).expect_err("the throw");
        assert_eq!(
            error,
            "Value of '1.5' is not valid for 'Value'. 'Value' should be between 'Minimum' and \
             'Maximum'."
        );
        assert!(holder.window.is_none());
        assert_eq!(holder.opened, 0);
        // Clearance, the lists, Range and Height came first.
        assert_eq!(get(&settings, "Propagation_Height").as_deref(), Some("2"));
        assert_eq!(get(&settings, "Propagation_Minalt"), None);
        let mut negative = self::settings(&[("Propagation_Clearance", "-1")]);
        assert!(Form::new(&mut negative).is_err());
        let mut huge = self::settings(&[("Propagation_Maxalt", "1e30")]);
        assert_eq!(
            Form::new(&mut huge).err().as_deref(),
            Some(DECIMAL_OVERFLOW)
        );
    }

    /// The arrows step by 0.1 in `decimal`, held to the range, each change written.
    #[test]
    fn the_arrows_step_a_tenth() {
        let mut settings = settings(&[]);
        let mut form = Form::new(&mut settings).expect("the form");
        form.step(Number::Tolerance, true, &mut settings);
        assert_eq!(form.shown(Number::Tolerance), "0.9");
        assert_eq!(
            get(&settings, "Propagation_Tolerance").as_deref(),
            Some("0.9")
        );
        form.step(Number::Tolerance, true, &mut settings);
        form.step(Number::Tolerance, true, &mut settings);
        assert_eq!(form.shown(Number::Tolerance), "1.0");
        assert_eq!(
            get(&settings, "Propagation_Tolerance").as_deref(),
            Some("1.0")
        );
        form.step(Number::Clearance, false, &mut settings);
        assert_eq!(
            get(&settings, "Propagation_Clearance").as_deref(),
            Some("4.9")
        );
    }

    /// Typing: the text read on leaving, held to the range; what does not parse leaves `Value`.
    #[test]
    fn typing_is_read_on_leaving() {
        let mut settings = settings(&[]);
        let mut form = Form::new(&mut settings).expect("the form");
        form.begin(Number::Height, &mut settings);
        if let Some((_, field)) = form.editing.as_mut() {
            field.set("2500");
        }
        assert_eq!(form.shown(Number::Height), "2500");
        form.leave(&mut settings);
        assert_eq!(form.shown(Number::Height), "2000.0");
        assert_eq!(
            get(&settings, "Propagation_Height").as_deref(),
            Some("2000")
        );
        form.begin(Number::Height, &mut settings);
        if let Some((_, field)) = form.editing.as_mut() {
            field.set("abc");
        }
        form.leave(&mut settings);
        assert_eq!(form.shown(Number::Height), "2000.0");
        assert!(form.editing.is_none());
    }

    /// A click: indeterminate or ticked unticks, unticked ticks; each writes its key.
    #[test]
    fn a_click_ticks_and_unticks() {
        let mut settings = settings(&[("Propagation_Termap", "True")]);
        let mut form = Form::new(&mut settings).expect("the form");
        form.click(Check::Terrain, &mut settings);
        assert_eq!(form.check(Check::Terrain), CheckState::Unchecked);
        assert_eq!(
            get(&settings, "Propagation_Termap").as_deref(),
            Some("False")
        );
        form.click(Check::Terrain, &mut settings);
        assert_eq!(form.check(Check::Terrain), CheckState::Checked);
        assert_eq!(
            get(&settings, "Propagation_Termap").as_deref(),
            Some("True")
        );
        form.click(Check::ShowScale, &mut settings);
        assert_eq!(
            get(&settings, "Propagation_ShowScale").as_deref(),
            Some("True")
        );
    }

    /// A list: dropped down, an item chosen, its text written; the same item again, nothing.
    #[test]
    fn choosing_an_item_writes_its_text() {
        let mut settings = settings(&[]);
        let mut form = Form::new(&mut settings).expect("the form");
        form.toggle(Combo::Rotational, &mut settings);
        assert_eq!(form.open, Some(Combo::Rotational));
        form.choose(Combo::Rotational, 0, &mut settings);
        assert_eq!(form.open, None);
        assert_eq!(form.text(Combo::Rotational), "0.5");
        assert_eq!(
            get(&settings, "Propagation_Rotational").as_deref(),
            Some("0.5")
        );
        settings.set("Propagation_Rotational", "x");
        form.choose(Combo::Rotational, 0, &mut settings);
        assert_eq!(
            get(&settings, "Propagation_Rotational").as_deref(),
            Some("x")
        );
    }

    /// The words, places, ranges and keys are the C#'s, read from the tree when it is here.
    #[test]
    fn the_form_is_the_csharps() {
        let (Some(code), Some(resx), Some(maps)) = (
            crate::config_coverage::source::csharp("Controls/PropagationSettings.cs"),
            crate::config_coverage::source::csharp("Controls/PropagationSettings.resx"),
            crate::config_coverage::source::csharp("ExtLibs/Maps/Propagation.cs"),
        ) else {
            eprintln!(
                "skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner"
            );
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let pair = |(x, y): (f32, f32)| format!("{x}, {y}");
        assert_eq!(
            values.get("$this.Text").map(String::as_str),
            Some(FORM_TEXT)
        );
        assert_eq!(values.get("$this.ClientSize"), Some(&pair(CLIENT)));
        assert_eq!(
            values.get("groupBox1.Text").map(String::as_str),
            Some(GROUP_TEXT)
        );
        let (x, y, w, h) = GROUP_AT;
        assert_eq!(values.get("groupBox1.Location"), Some(&pair((x, y))));
        assert_eq!(values.get("groupBox1.Size"), Some(&pair((w, h))));
        for (name, text, at) in LABELS {
            assert_eq!(
                values.get(&format!("{name}.Text")).map(String::as_str),
                Some(text)
            );
            assert_eq!(
                values.get(&format!("{name}.Location")),
                Some(&pair(at)),
                "{name}"
            );
        }
        for check in Check::ALL {
            let (name, text) = check.resx();
            assert_eq!(
                values.get(&format!("{name}.Text")).map(String::as_str),
                Some(text)
            );
            assert_eq!(
                values.get(&format!("{name}.Location")),
                Some(&pair(check.place()))
            );
            assert!(maps.contains(&format!("\"{}\"", check.setting())), "{name}");
        }
        for number in Number::ALL {
            let (x, y, w, h) = number.place();
            let name = number.name();
            assert_eq!(values.get(&format!("{name}.Location")), Some(&pair((x, y))));
            assert_eq!(values.get(&format!("{name}.Size")), Some(&pair((w, h))));
            let (key, default) = number.setting();
            assert!(
                code.contains(&format!("GetFloat(\"{key}\", {default}")),
                "{key} {default}"
            );
            assert!(code.contains(&format!("Settings.Instance[\"{key}\"] = {name}.Value")));
        }
        for combo in Combo::ALL {
            let name = combo.name();
            for (index, item) in combo.items().iter().enumerate() {
                let suffix = if index == 0 {
                    String::new()
                } else {
                    index.to_string()
                };
                assert_eq!(
                    values
                        .get(&format!("{name}.Items{suffix}"))
                        .map(String::as_str),
                    Some(*item)
                );
            }
            let (x, y, w, h) = combo.place();
            assert_eq!(values.get(&format!("{name}.Location")), Some(&pair((x, y))));
            assert_eq!(values.get(&format!("{name}.Size")), Some(&pair((w, h))));
            let (key, default) = combo.setting();
            assert!(
                code.contains(&format!("GetInt32(\"{key}\", {default})")),
                "{key}"
            );
        }
        // Tolerance's `Maximum` is 1: the first of the `decimal`'s four ints.
        let tolerance = code
            .split("this.Tolerance.Maximum = new decimal(new int[] {")
            .nth(1)
            .map(str::trim_start);
        assert!(tolerance.is_some_and(|rest| rest.starts_with("1,")));
    }
}
