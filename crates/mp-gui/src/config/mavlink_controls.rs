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

//! The shared controls, one of each as the C# has them: `MavlinkCheckBox`, `MavlinkComboBox` and
//! `MavlinkNumericUpDown` (`Controls/`), and `InputBox`, `HorizontalProgressBar` and
//! `HorizontalProgressBar2` (`ExtLibs/Controls/`). Every page that shows one holds the state
//! below - drawn with the shared look (`servo_output`'s `check_box`, `combo_box`, `dropdown` and
//! `number_box`, `optional`'s `input_box`) or at the page's own `.resx` places - and what the
//! control does is here alone: how `setup` binds it to a parameter, what a change writes and what
//! its failure says, when it is disabled or hidden, what `InputBox.Show` hands back and keeps, and
//! how a bar holds its value. `servo_output`, `failsafe` and `optional`, where the pages found
//! these before, name them still.
//!
//! * [`Check`], `MavlinkCheckBox`: `setup`'s three forms, the box checked for the on value,
//!   unchecked for the off value and indeterminate for anything else, disabled without its
//!   parameter, and a click writing the value it leaves the box at.
//! * [`Combo`], `MavlinkComboBox`: bound to a list - the parameter's documented values, the
//!   page's own, or an enum's names - selecting the vehicle's value, and a choice writing it.
//! * [`Number`], `MavlinkNumericUpDown`: the documented range and increment over the page's,
//!   the value widening them, the typed text read as `NumericUpDown` reads it, a value typed above
//!   the maximum asking "Out of range" first, and the write 300 ms after a change.
//! * [`InputBox`]: a caption, a question and an answer box with OK and Cancel; `Show`'s `ref
//!   value` afterwards - the text on OK, the value passed in on Cancel - and the titled answer kept
//!   in `Settings.Instance`.
//! * [`ProgressBar`]: `HorizontalProgressBar`'s and `HorizontalProgressBar2`'s `Minimum`,
//!   `Maximum` and `Value` over the base bar they draw, the label, `reverse` and the red lines.
//!
//! A control's write is a [`Write`]: the name, the value, and what the C#'s box says when
//! `setParam` returns false or throws. The page hands it to the link - `servo_output`'s `Writes`
//! or `optional`'s `SetQueue` - and, by the owner's ruling of 2026-09-25, shows the failure on the
//! status line where the page can.
//!
//! What is not ported, and why:
//!
//! * `InputBox`'s password mode - `UseSystemPasswordChar`, and the answer not kept - which no page
//!   here asks for; the owner's ruling (PLAN.md §12 D18) keeps a password's answer, shown plainly,
//!   like any other;
//! * the controls' events as events: `CheckedChanged`, `SelectedIndexChanged` and `ValueChanged`
//!   are the calls that return a [`Write`], and a page subscribed to `ValueUpdated` takes the value
//!   from [`Number::flush`] instead of the timer;
//! * the `enabledisable` and `SubControl` controls as references: the page applies
//!   [`Check::background_enabled`], or a number's `Enabled`, to its own control;
//! * `MavlinkComboBox`'s second parameter, `paramname2`, which nothing sets
//!   (`Controls/MavlinkComboBox.cs:20`, its `setup` argument commented out at `:103`);
//! * `HorizontalProgressBar`'s animation workaround - the base value set past and back - and its
//!   labels' places, which are WinForms' drawing.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use web_time::{Duration, Instant};

use gpui::KeyDownEvent;
use mp_params::ParamMeta;

use crate::settings::Persisted;
use crate::textfield::{KeyOutcome, TextField};

// ---------------------------------------------------------------------------------------------
// The parameter list and its documentation.
// ---------------------------------------------------------------------------------------------

/// Where parameter documentation comes from: [`crate::metadata::lookup`] on screen, the bundled
/// table in the tests.
pub type Lookup = fn(&str) -> Option<&'static ParamMeta>;

/// A parameter's value in the vehicle's table, `paramlist[name].Value`.
#[must_use]
pub fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// `paramlist.ContainsKey(name)`.
#[must_use]
pub fn has(parameters: &[(String, f64)], name: &str) -> bool {
    value_of(parameters, name).is_some()
}

/// The options a parameter's documentation lists, trimmed as the C# trims them.
/// `// C#: ExtLibs/Utilities/ParameterMetaDataRepository.cs:76-102`
#[must_use]
pub fn options(param: &str, lookup: Lookup) -> Vec<(i64, String)> {
    lookup(param).map_or_else(Vec::new, |meta| {
        meta.values
            .iter()
            .map(|(key, text)| (*key, text.trim().to_owned()))
            .collect()
    })
}

// ---------------------------------------------------------------------------------------------
// What a control writes, and what its failure says.
// ---------------------------------------------------------------------------------------------

/// `Strings.ErrorSetValueFailed`, "Set {0} Failed".
/// `// C#: ExtLibs/Strings/Strings.resx:171-173`
#[must_use]
pub fn set_failed(param: &str) -> String {
    format!("Set {param} Failed")
}

/// The caption of `MavlinkNumericUpDown`'s out-of-range question.
/// `// C#: Controls/MavlinkNumericUpDown.cs:139-140`
pub const OUT_OF_RANGE_TITLE: &str = "Out of range";

/// How long a number waits before it writes: a change sets a 300 ms timer going - only when it is
/// not going already, so a burst of changes is written once, 300 ms after the first - and the
/// timer writes the value the box holds when it fires.
/// `// C#: Controls/MavlinkNumericUpDown.cs:154-180`
pub const WRITE_DELAY: Duration = Duration::from_millis(300);

/// A parameter write a control asks for - `MainV2.comPort.setParam(name, value)` - and the text
/// of the message box the C# shows when it fails.
#[derive(Debug, Clone, PartialEq)]
pub struct Write {
    /// The parameter.
    pub param: String,
    /// The value, as the C# computes it.
    pub value: f64,
    /// What the C# shows when the set fails.
    pub failure: String,
}

impl Write {
    /// A combo box's write from a list, whose failure is its own text with an exclamation mark.
    /// `// C#: Controls/MavlinkComboBox.cs:180-183, 195-198`
    #[must_use]
    pub fn combo(param: &str, value: f64) -> Self {
        Self {
            param: param.to_owned(),
            value,
            failure: format!("Set {param} Failed!"),
        }
    }

    /// A check box's, a number's, or a combo box's bound to an enum's names, which fails with
    /// `Strings.ErrorSetValueFailed`, "Set {0} Failed".
    /// `// C#: Controls/MavlinkCheckBox.cs:116-140; Controls/MavlinkNumericUpDown.cs:169-176;
    /// Controls/MavlinkComboBox.cs:149-152, 164-167; ExtLibs/Strings/Strings.resx:171-173`
    #[must_use]
    pub fn other(param: &str, value: f64) -> Self {
        Self {
            param: param.to_owned(),
            value,
            failure: set_failed(param),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// MavlinkCheckBox.
// ---------------------------------------------------------------------------------------------

/// A check box's three states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CheckState {
    /// Off.
    #[default]
    Unchecked,
    /// On.
    Checked,
    /// The vehicle holds neither the on value nor the off value.
    Indeterminate,
}

impl CheckState {
    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Unchecked => "unchecked",
            Self::Checked => "checked",
            Self::Indeterminate => "indeterminate",
        }
    }

    /// Where a click leaves a two-state box (`ThreeState` false, as every box here is): checked
    /// or indeterminate to unchecked, unchecked to checked - each of them a change of `Checked`.
    #[must_use]
    pub const fn clicked(self) -> Self {
        match self {
            Self::Unchecked => Self::Checked,
            Self::Checked | Self::Indeterminate => Self::Unchecked,
        }
    }
}

/// A `MavlinkCheckBox`: bound by `setup` to a parameter and its on and off values, showing which
/// the vehicle holds, and writing the one a click leaves it at. Also the plain `CheckBox`es pages
/// draw with its look, set by hand and never set up.
/// `// C#: Controls/MavlinkCheckBox.cs:6-145`
#[derive(Debug, Clone, PartialEq)]
pub struct Check {
    /// `ParamName`: empty until a `setup` names one, as the C#'s is null.
    pub param: String,
    /// What the box shows.
    pub state: CheckState,
    /// `Enabled`: false from the constructor, true once `setup` finds the parameter, false again
    /// when a later `setup` does not.
    pub enabled: bool,
    /// `Visible`: as the page's Designer leaves it - true unless it hides the box - and made true
    /// by a `setup` that finds the parameter; nothing here hides it again.
    pub visible: bool,
    /// `OnValue`.
    on: f64,
    /// `OffValue`.
    off: f64,
}

impl Default for Check {
    /// The constructor: `OnValue = 1`, `OffValue = 0`, disabled; shown, as a `CheckBox` is.
    /// `// C#: Controls/MavlinkCheckBox.cs:24-30`
    fn default() -> Self {
        Self {
            param: String::new(),
            state: CheckState::Unchecked,
            enabled: false,
            visible: true,
            on: 1.0,
            off: 0.0,
        }
    }
}

impl Check {
    /// A box the page's `.resx` hides (`Visible` false) until `setup` finds its parameter.
    #[must_use]
    pub fn hidden() -> Self {
        Self {
            visible: false,
            ..Self::default()
        }
    }

    /// `setup(OnValue, OffValue, paramname, paramlist)`: the name and the two values taken
    /// whether or not the vehicle has the parameter. For one it has, the box enabled and shown,
    /// checked for the on value, unchecked for the off value and indeterminate for anything else;
    /// for one it lacks, disabled and otherwise left as it was.
    /// `// C#: Controls/MavlinkCheckBox.cs:60-98`
    pub fn setup(&mut self, on: f64, off: f64, param: &str, parameters: &[(String, f64)]) {
        self.on = on;
        self.off = off;
        param.clone_into(&mut self.param);
        let Some(value) = value_of(parameters, param) else {
            self.enabled = false;
            return;
        };
        self.enabled = true;
        self.visible = true;
        // `paramlist[paramname].Value == OnValue`: an exact comparison, as the C# makes it, of the
        // same reading on both sides.
        #[allow(clippy::float_cmp)]
        let state = if value == on {
            CheckState::Checked
        } else if value == off {
            CheckState::Unchecked
        } else {
            CheckState::Indeterminate
        };
        self.state = state;
    }

    /// `setup(OnValue, OffValue, string[] paramname, paramlist)`: set up for the first of the
    /// names the vehicle has; when it has none, nothing at all - the box keeps its name, its
    /// values, its state and its `Enabled`.
    /// `// C#: Controls/MavlinkCheckBox.cs:47-58`
    pub fn setup_names(&mut self, on: f64, off: f64, names: &[&str], parameters: &[(String, f64)]) {
        if let Some(name) = names.iter().find(|name| has(parameters, name)) {
            self.setup(on, off, name, parameters);
        }
    }

    /// `setup(double[] OnValue, double[] OffValue, string[] paramname, paramlist)`: the first name
    /// the vehicle has, with its own on and off values; nothing at all when it has none.
    /// `// C#: Controls/MavlinkCheckBox.cs:32-45`
    pub fn setup_choices<S: AsRef<str>>(
        &mut self,
        choices: &[(f64, f64, S)],
        parameters: &[(String, f64)],
    ) {
        if let Some((on, off, name)) = choices
            .iter()
            .find(|(_, _, name)| has(parameters, name.as_ref()))
        {
            self.setup(*on, *off, name.as_ref(), parameters);
        }
    }

    /// `ParamName`, once a `setup` has named one.
    #[must_use]
    pub fn param_name(&self) -> Option<&str> {
        (!self.param.is_empty()).then_some(self.param.as_str())
    }

    /// `Checked`: on, or indeterminate.
    #[must_use]
    pub const fn checked(&self) -> bool {
        !matches!(self.state, CheckState::Unchecked)
    }

    /// What `enableBGControl` makes the `enabledisable` control: enabled while the box is
    /// checked, from a `setup` that finds the parameter and from each change. The page applies it
    /// to its own control.
    /// `// C#: Controls/MavlinkCheckBox.cs:76-90, 100-104, 111-129`
    #[must_use]
    pub fn background_enabled(&self) -> bool {
        self.state == CheckState::Checked
    }

    /// A click. A disabled or hidden box takes none, and a box never set up names nothing to
    /// write. Otherwise the click changes `Checked`, which raises `CheckedChanged`, so every click
    /// writes: `OnValue` when it ends checked, `OffValue` otherwise.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    pub fn click(&mut self) -> Option<Write> {
        if !self.enabled || !self.visible || self.param.is_empty() {
            return None;
        }
        self.state = self.state.clicked();
        let value = if self.state == CheckState::Checked {
            self.on
        } else {
            self.off
        };
        Some(Write::other(&self.param, value))
    }
}

// ---------------------------------------------------------------------------------------------
// MavlinkComboBox.
// ---------------------------------------------------------------------------------------------

/// How many rows a dropped-down list shows at once; the rest come into view on the wheel, as a
/// WinForms drop-down scrolls (it shows `MaxDropDownItems`, 8, and scrolls the rest).
pub const LIST_ROWS_SHOWN: usize = 30;
/// A line of the wheel, in rows. gpui reports the wheel in lines already multiplied by the
/// system's lines per notch - three on X11 and on Windows (`SPI_GETWHEELSCROLLLINES`) - and a
/// WinForms list scrolls one item per line, so a notch moves three rows.
pub const WHEEL_ROWS: usize = 1;

/// A `MavlinkComboBox`: bound by `setup` to a list - the parameter's documented values, or the
/// page's own, or an enum's names - selecting the vehicle's value, and writing a choice. Also the
/// plain `ComboBox`es pages draw with its look - the Serial Ports page's, which bind a list the
/// same way and write through the page - and the lists that are not bound at all. `Visible`,
/// which `setup` makes true with `Enabled`, is the page's to keep: a box its `.resx` hides shows
/// while it is enabled.
/// `// C#: Controls/MavlinkComboBox.cs:9-201`
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Combo {
    /// `ParamName`, which `setup` also makes the control's `Name`.
    pub param: String,
    /// `DataSource`: the values and their text, `GetParameterOptionsInt`.
    pub options: Vec<(i64, String)>,
    /// `SelectedValue`; `None` is `SelectedIndex` -1, a value the list does not hold.
    pub selected: Option<i64>,
    /// `Enabled`: false from the constructor, true once `setup` finds the parameter.
    pub enabled: bool,
    /// The dropped-down list's `TopIndex`: the first of the [`LIST_ROWS_SHOWN`] rows showing.
    /// Zero until the list is opened; a list longer than the rows shown scrolls on the wheel.
    pub top_index: usize,
}

impl Combo {
    /// Drops the list down as WinForms does: scrolled so the selected row shows, from the top
    /// when the selected row is within the rows shown.
    pub fn open_list(&mut self) {
        let selected = self
            .selected
            .and_then(|key| self.options.iter().position(|(k, _)| *k == key))
            .unwrap_or(0);
        self.top_index = selected
            .saturating_sub(LIST_ROWS_SHOWN - 1)
            .min(self.max_top_index());
    }

    /// The wheel: `lines` down (negative up), [`WHEEL_ROWS`] rows each, the window kept
    /// within the list.
    pub fn scroll_list(&mut self, lines: i32) {
        let rows = usize::try_from(lines.unsigned_abs()).unwrap_or(usize::MAX);
        let rows = rows.saturating_mul(WHEEL_ROWS);
        let top = if lines < 0 {
            self.top_index.saturating_sub(rows)
        } else {
            self.top_index.saturating_add(rows)
        };
        self.top_index = top.min(self.max_top_index());
    }

    /// The last `top_index` that still fills the rows shown.
    #[must_use]
    pub fn max_top_index(&self) -> usize {
        self.options.len().saturating_sub(LIST_ROWS_SHOWN)
    }

    /// The rows showing, with their index in the list.
    pub fn showing(&self) -> impl Iterator<Item = (usize, &(i64, String))> {
        self.options
            .iter()
            .enumerate()
            .skip(self.top_index)
            .take(LIST_ROWS_SHOWN)
    }

    /// `setup(List<KeyValuePair<int, string>>, paramname, paramlist)`: binds the list, which
    /// selects its first row as a WinForms list control does when its `DataSource` is set, and
    /// takes the name; then - for a parameter the vehicle has - enables the box and selects
    /// `(int)` its value, or nothing for a value the list does not hold. A parameter it lacks
    /// leaves `Enabled` as it was.
    /// `// C#: Controls/MavlinkComboBox.cs:73-99`
    pub fn setup(
        &mut self,
        options: Vec<(i64, String)>,
        param: &str,
        parameters: &[(String, f64)],
    ) {
        self.options = options;
        self.param = param.to_owned();
        self.selected = self.options.first().map(|(key, _)| *key);
        if let Some(value) = value_of(parameters, param) {
            self.enabled = true;
            // `(int)paramlist[paramname].Value`: a cast, so truncated.
            #[allow(clippy::cast_possible_truncation)]
            let value = value as i64;
            self.selected = self
                .options
                .iter()
                .any(|(key, _)| *key == value)
                .then_some(value);
        }
    }

    /// The selected row's text, blank when nothing is selected.
    #[must_use]
    pub fn text(&self) -> &str {
        self.selected
            .and_then(|selected| self.options.iter().find(|(key, _)| *key == selected))
            .map_or("", |(_, text)| text.as_str())
    }

    /// Selects a row as `SelectedValue = key` does. Whether the selection changed, which is when
    /// WinForms raises `SelectedIndexChanged`; a value the list does not hold changes nothing.
    pub fn select(&mut self, key: i64) -> bool {
        if self.selected == Some(key) || !self.options.iter().any(|(option, _)| *option == key) {
            return false;
        }
        self.selected = Some(key);
        true
    }

    /// A row chosen from the list: the write `SelectedIndexChanged` makes, `(float)(int)
    /// SelectedValue`, when the selection changed.
    /// `// C#: Controls/MavlinkComboBox.cs:133-200`
    pub fn choose(&mut self, key: i64) -> Option<Write> {
        if !self.enabled || !self.select(key) {
            return None;
        }
        #[allow(clippy::cast_precision_loss)] // option values are small integers
        let value = f64::from(key as f32);
        Some(Write::combo(&self.param, value))
    }

    /// `setup(string[] paramnames, paramlist)`: the first of the names the vehicle has, bound to
    /// that parameter's documented values (`GetParameterOptionsInt`) as the list form binds them;
    /// when it has none, nothing at all - no list, no name, the box as it was.
    /// `// C#: Controls/MavlinkComboBox.cs:37-71`
    pub fn setup_names(&mut self, names: &[&str], parameters: &[(String, f64)], lookup: Lookup) {
        if let Some(name) = names.iter().find(|name| has(parameters, name)) {
            self.setup(options(name, lookup), name, parameters);
        }
    }

    /// `SelectedIndex = index` from code: the row at that place selected, and - the selection
    /// having changed - the write `SelectedIndexChanged` makes, which the handler makes whether
    /// or not the box is enabled.
    /// `// C#: Controls/MavlinkComboBox.cs:133-136, 169-199`
    pub fn select_index(&mut self, index: usize) -> Option<Write> {
        let key = self.options.get(index).map(|(key, _)| *key)?;
        if !self.select(key) {
            return None;
        }
        #[allow(clippy::cast_precision_loss)] // option values are small integers
        let value = f64::from(key as f32);
        Some(Write::combo(&self.param, value))
    }

    /// A row chosen from a list bound to an enum's names - `setup(typeof(...), paramname,
    /// paramlist)`, the list of `Enum.GetNames` and the vehicle's value's name as `Text` - when
    /// the selection changed: the write is the name's value, `(float)(Int32)Enum.Parse(_source,
    /// Text)`, and its failure `Strings.ErrorSetValueFailed`, without the list form's "!".
    /// `// C#: Controls/MavlinkComboBox.cs:102-125, 138-168`
    pub fn choose_named(&mut self, key: i64) -> Option<Write> {
        if !self.enabled || !self.select(key) {
            return None;
        }
        #[allow(clippy::cast_precision_loss)] // an enum's values are small integers
        let value = f64::from(key as f32);
        Some(Write::other(&self.param, value))
    }
}

// ---------------------------------------------------------------------------------------------
// MavlinkNumericUpDown.
// ---------------------------------------------------------------------------------------------

/// A decimal as .NET's `(decimal)float` makes one: seven significant digits, trailing zeros
/// dropped. Returned as a mantissa and the number of places after the point, which is what
/// `decimal.GetBits(...)[3]`'s scale byte reads back.
/// `// C#: Controls/MavlinkNumericUpDown.cs:90, 101-103`
#[must_use]
pub fn decimal_of(value: f32) -> (i64, u32) {
    if value == 0.0 || !value.is_finite() {
        return (0, 0);
    }
    let text = format!("{:.6e}", value.abs());
    let Some((mantissa, exponent)) = text.split_once('e') else {
        return (0, 0);
    };
    let exponent: i64 = exponent.parse().unwrap_or(0);
    let digits: String = mantissa.chars().filter(char::is_ascii_digit).collect();
    let digits = digits.trim_end_matches('0');
    let mut mantissa: i64 = digits.parse().unwrap_or(0);
    // The number of places: digits after the first, less the power of ten.
    let places = i64::try_from(digits.len()).unwrap_or(0) - 1 - exponent;
    let scale = if places < 0 {
        mantissa =
            mantissa.saturating_mul(10_i64.saturating_pow(u32::try_from(-places).unwrap_or(0)));
        0
    } else {
        u32::try_from(places).unwrap_or(0)
    };
    if value < 0.0 {
        mantissa = -mantissa;
    }
    (mantissa, scale)
}

/// A `NumericUpDown` as its Designer or constructor leaves it, before `setup`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Designer {
    /// `Minimum`.
    pub minimum: f64,
    /// `Maximum`.
    pub maximum: f64,
    /// `Value`.
    pub value: f64,
    /// `DecimalPlaces`.
    pub decimals: u32,
}

/// `NumericUpDown`'s own defaults: 0 to 100, 0, no decimal places.
pub const NUMERIC_DEFAULTS: Designer = Designer {
    minimum: 0.0,
    maximum: 100.0,
    value: 0.0,
    decimals: 0,
};

/// `MavlinkNumericUpDown.setup`'s first four arguments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Setup {
    /// `Min`.
    pub minimum: f32,
    /// `Max`.
    pub maximum: f32,
    /// `Scale`: the parameter is the value shown times this.
    pub scale: f32,
    /// `Increment`.
    pub increment: f32,
}

/// `MavlinkNumericUpDown`'s question when a typed value is above the maximum.
#[derive(Debug, Clone, PartialEq)]
pub struct Question {
    /// `ParamName`.
    pub param: String,
    /// What was typed.
    pub typed: f64,
}

impl Question {
    /// Its text.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:139`
    #[must_use]
    pub fn text(&self) -> String {
        format!(
            "{} Value out of range\nDo you want to accept the new value?",
            self.param
        )
    }
}

/// A `decimal` a float converts to, as an `f64`: seven significant digits.
/// `(decimal)value`: a float to the decimal it converts to, seven significant digits.
pub fn decimal(value: f32) -> f64 {
    let (mantissa, places) = decimal_of(value);
    #[allow(clippy::cast_precision_loss)] // seven digits fit
    let mantissa = mantissa as f64;
    mantissa / 10_f64.powi(i32::try_from(places).unwrap_or(0))
}

/// `decimal.ToString("F<places>")`: rounded half away from zero, as `f64::round` rounds.
#[must_use]
pub fn decimal_text(value: f64, places: u32) -> String {
    let power = 10_f64.powi(i32::try_from(places).unwrap_or(0));
    // Adding zero makes a negative zero positive: no "-0".
    let rounded = (value * power).round() / power + 0.0;
    format!("{:.*}", places as usize, rounded)
}

/// Sums and differences of decimals, kept to nine places so a float's error does not show.
fn snap(value: f64) -> f64 {
    (value * 1e9).round() / 1e9
}

/// `decimal.Parse(Text)`: white space, a sign, digits, a point and group separators.
fn parse_decimal(text: &str) -> Option<f64> {
    let text = text.trim();
    if text.is_empty()
        || !text
            .chars()
            .all(|c| c.is_ascii_digit() || matches!(c, '+' | '-' | '.' | ','))
    {
        return None;
    }
    text.replace(',', "").parse().ok()
}

/// `MavlinkNumericUpDown.setup`'s choice of name: "default to first item", then the first the
/// vehicle has; empty for no names.
/// `// C#: Controls/MavlinkNumericUpDown.cs:54-64`
#[must_use]
pub fn bound_name<'a>(names: &[&'a str], parameters: &[(String, f64)]) -> &'a str {
    names
        .iter()
        .copied()
        .find(|name| has(parameters, name))
        .or_else(|| names.first().copied())
        .unwrap_or_default()
}

/// A `MavlinkNumericUpDown`: bounds, increment and value as the C#'s `decimal`s, the text in its
/// box, and the timer that writes it.
/// `// C#: Controls/MavlinkNumericUpDown.cs:9-181`
#[derive(Debug)]
pub struct Number {
    /// `ParamName`.
    pub param: String,
    /// `Enabled`: false from the constructor, true once `setup` finds the parameter.
    pub enabled: bool,
    /// `Visible`: as the page's Designer leaves it - true unless it hides the box - and made true
    /// by a `setup` that finds the parameter.
    pub visible: bool,
    /// `Minimum`.
    pub minimum: f64,
    /// `Maximum`.
    pub maximum: f64,
    /// `DecimalPlaces`.
    pub decimals: u32,
    /// `Increment`.
    increment: f64,
    /// `Value`.
    value: f64,
    /// `_scale`.
    scale: f32,
    /// The box's text, which is `Value`'s until something is typed.
    field: TextField,
    /// `UserEdit`: typed into since the text was last read.
    edited: bool,
    /// When the write timer was started, while it runs.
    timer: Option<Instant>,
}

impl Number {
    /// A box as its Designer leaves it: disabled, as the constructor makes it.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:31-41`
    #[must_use]
    pub fn new(designer: Designer) -> Self {
        let mut number = Self {
            param: String::new(),
            enabled: false,
            visible: true,
            minimum: designer.minimum,
            maximum: designer.maximum,
            decimals: designer.decimals,
            increment: 1.0,
            value: designer.value,
            scale: 1.0,
            field: TextField::new(""),
            edited: false,
            timer: None,
        };
        number.field.set(number.text());
        number
    }

    /// A box the page's `.resx` hides (`Visible` false) until `setup` finds its parameter.
    #[must_use]
    pub fn hidden(designer: Designer) -> Self {
        Self {
            visible: false,
            ..Self::new(designer)
        }
    }

    /// `setup(Min, Max, Scale, Increment, paramname, paramlist)`: the documented range over the
    /// one passed, the documented increment when it is larger than the decimal places the box
    /// has - a step compared with a count, as the C# compares them - the decimal places the
    /// increment has, and for a parameter the vehicle has, its value, which widens the range and
    /// the places when it needs to, and shows it. A parameter the vehicle lacks disables the box
    /// and leaves its value.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:49-122`
    pub fn setup(&mut self, how: Setup, param: &str, parameters: &[(String, f64)], lookup: Lookup) {
        self.param = param.to_owned();
        let meta = lookup(param);
        let (mut minimum, mut maximum) = (how.minimum, how.maximum);
        if let Some((low, high)) = meta.and_then(|meta| meta.range) {
            // `Min = (float)mint`. `// C#: :69-75`
            #[allow(clippy::cast_possible_truncation)]
            let (low, high) = (low as f32, high as f32);
            minimum = low;
            maximum = high;
        }
        // `// C#: :80-84`; `GetParameterIncrement` reads it as a float.
        let mut increment = how.increment;
        if let Some(documented) = meta.and_then(|meta| meta.increment) {
            #[allow(clippy::cast_possible_truncation)]
            let documented = documented as f32;
            if f64::from(documented) > f64::from(self.decimals) {
                increment = documented;
            }
        }
        self.scale = how.scale;
        // `Minimum` and `Maximum` hold the value inside them as they are set. `// C#: :86-90`
        self.set_minimum(decimal(minimum));
        self.set_maximum(decimal(maximum));
        let (step, places) = decimal_of(increment);
        #[allow(clippy::cast_precision_loss)]
        let step = step as f64 / 10_f64.powi(i32::try_from(places).unwrap_or(0));
        self.increment = step;
        self.decimals = places;
        match value_of(parameters, param) {
            Some(value) => {
                // `(decimal)((float)paramlist[ParamName] / _scale)`. `// C#: :94-113`
                self.enabled = true;
                self.visible = true;
                #[allow(clippy::cast_possible_truncation)]
                let shown = value as f32 / self.scale;
                let (_, places) = decimal_of(shown);
                if places > self.decimals {
                    self.decimals = places;
                }
                let exact = decimal(shown);
                if exact < self.minimum {
                    self.minimum = exact;
                }
                if exact > self.maximum {
                    self.maximum = exact;
                }
                self.value = exact;
            }
            None => self.enabled = false,
        }
        self.edited = false;
        self.field.set(self.text());
    }

    /// `setup(Min, Max, Scale, Increment, string[] paramname, paramlist)`: set up for the first
    /// of the names the vehicle has, or for the first name when it has none ([`bound_name`]).
    /// `// C#: Controls/MavlinkNumericUpDown.cs:49-64`
    pub fn setup_names(
        &mut self,
        how: Setup,
        names: &[&str],
        parameters: &[(String, f64)],
        lookup: Lookup,
    ) {
        self.setup(how, bound_name(names, parameters), parameters, lookup);
    }

    /// The `Minimum` setter: the maximum raised to it if below, the value held inside.
    fn set_minimum(&mut self, minimum: f64) {
        self.minimum = minimum;
        if self.minimum > self.maximum {
            self.maximum = self.minimum;
        }
        self.value = self.value.clamp(self.minimum, self.maximum);
    }

    /// The `Maximum` setter.
    fn set_maximum(&mut self, maximum: f64) {
        self.maximum = maximum;
        if self.minimum > self.maximum {
            self.minimum = self.maximum;
        }
        self.value = self.value.clamp(self.minimum, self.maximum);
    }

    /// `Increment`.
    #[cfg(test)]
    #[must_use]
    pub const fn increment(&self) -> f64 {
        self.increment
    }

    /// `Increment`'s setter, for a box set up by hand rather than by `setup`: a `RangeControl`'s
    /// plain `NumericUpDown` takes the increment its owner gives it.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:31-39`
    pub const fn set_increment(&mut self, increment: f64) {
        self.increment = increment;
    }

    /// `Value`.
    #[must_use]
    pub const fn value(&self) -> f64 {
        self.value
    }

    /// `Value`'s setter from code: held inside the bounds, the box showing it, nothing typed
    /// over it, and no timer - the owner decides what a change does.
    pub fn set_value(&mut self, value: f64) {
        self.value = value.clamp(self.minimum, self.maximum);
        self.edited = false;
        self.field.set(self.text());
    }

    /// `Value` as the box shows it, to `DecimalPlaces` places.
    #[must_use]
    pub fn text(&self) -> String {
        decimal_text(self.value, self.decimals)
    }

    /// What the box shows now: its value, or what has been typed over it.
    #[must_use]
    pub fn shown(&self) -> &str {
        self.field.value()
    }

    /// Whether the write timer is running.
    #[cfg(test)]
    #[must_use]
    pub const fn pending(&self) -> bool {
        self.timer.is_some()
    }

    /// `ValueChanged`: while the box's text is what was typed, a typed value above the maximum
    /// asks first, and the timer starts once it is answered; otherwise the timer starts now.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:133-161`
    fn changed(&mut self, now: Instant, typed: Option<f64>) -> Option<Question> {
        if let Some(typed) = typed
            && typed > self.maximum
        {
            return Some(Question {
                param: self.param.clone(),
                typed,
            });
        }
        self.start_timer(now);
        None
    }

    /// `if (!timer.Enabled) timer.Start()`: a running timer is left to run.
    fn start_timer(&mut self, now: Instant) {
        if self.timer.is_none() {
            self.timer = Some(now);
        }
    }

    /// `ValidateEditText`, when the box is left, on Enter, and before an arrow: the typed text,
    /// if it parses, held to the bounds and made the value - which raises `ValueChanged` only
    /// when it differs - and the box showing the value again.
    #[allow(clippy::float_cmp)] // `Value`'s setter compares decimals exactly
    pub fn commit(&mut self, now: Instant) -> Option<Question> {
        if !self.edited {
            return None;
        }
        self.edited = false;
        let text = self.field.value().trim().to_owned();
        let mut question = None;
        // `ParseEditText` leaves an empty box, a lone "-" and text that does not parse alone.
        if text != "-"
            && let Some(typed) = parse_decimal(&text)
        {
            let held = typed.clamp(self.minimum, self.maximum);
            if held != self.value {
                self.value = held;
                question = self.changed(now, Some(typed));
            }
        }
        self.field.set(self.text());
        question
    }

    /// The question answered. Yes raises the maximum to what was typed and takes it; either way
    /// the handler goes on to start the timer.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:138-160`
    pub fn answer(&mut self, question: &Question, yes: bool, now: Instant) {
        if yes {
            self.maximum = question.typed;
            self.value = question.typed;
        }
        self.start_timer(now);
        self.field.set(self.text());
    }

    /// The up or down arrow, or key: the typed text read first, then one `Increment`, stopping at
    /// the bound.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:133-161`, WinForms' `NumericUpDown.UpButton` and
    /// `DownButton`
    #[allow(clippy::float_cmp)]
    pub fn step(&mut self, up: bool, now: Instant) -> Option<Question> {
        if !self.enabled {
            return None;
        }
        // A question raised by the text read first is modal; the step it interrupted is not made.
        let question = self.commit(now);
        if question.is_some() {
            return question;
        }
        let next = if up {
            if self.value > self.maximum - self.increment {
                self.maximum
            } else {
                snap(self.value + self.increment)
            }
        } else if self.value < self.minimum + self.increment {
            self.minimum
        } else {
            snap(self.value - self.increment)
        };
        if next != self.value {
            self.value = next;
            // The text is still the old value, which is inside the range: no question.
            self.start_timer(now);
        }
        self.field.set(self.text());
        None
    }

    /// A key while the box has the focus: the arrow keys step, Enter reads the text, and anything
    /// else is typing.
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> (bool, Option<Question>) {
        if !self.enabled {
            return (false, None);
        }
        match event.keystroke.key.as_str() {
            "up" => return (true, self.step(true, now)),
            "down" => return (true, self.step(false, now)),
            _ => {}
        }
        match self.field.key(event) {
            KeyOutcome::Changed => {
                self.edited = true;
                (true, None)
            }
            KeyOutcome::Submitted => (true, self.commit(now)),
            KeyOutcome::Cancelled | KeyOutcome::Ignored => (false, None),
        }
    }

    /// Replaces the box's text as typing would.
    #[cfg(test)]
    pub fn type_text(&mut self, text: &str) {
        self.field.set(text);
        self.edited = true;
    }

    /// What the timer writes: `(float)base.Value * (float)_scale`.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:169`
    #[must_use]
    pub fn written(&self) -> f64 {
        #[allow(clippy::cast_possible_truncation)]
        let value = self.value as f32;
        f64::from(value * self.scale)
    }

    /// The write, once the timer has run its 300 ms.
    pub fn due(&mut self, now: Instant) -> Option<Write> {
        let started = self.timer?;
        if now.duration_since(started) < WRITE_DELAY {
            return None;
        }
        self.timer = None;
        Some(Write::other(&self.param, self.written()))
    }

    /// The write, now, whatever the timer says: the C#'s timer is not the page's, and outlives it.
    pub fn flush(&mut self) -> Option<Write> {
        self.timer.take()?;
        Some(Write::other(&self.param, self.written()))
    }
}

// ---------------------------------------------------------------------------------------------
// InputBox.
// ---------------------------------------------------------------------------------------------

/// An `InputBox`: a caption, a question and an answer box, with OK and Cancel.
/// `// C#: ExtLibs/Controls/InputBox.cs:11-205`
#[derive(Debug)]
pub struct InputBox {
    /// The caption.
    pub title: &'static str,
    /// The question.
    pub prompt: &'static str,
    /// The answer box, holding the value passed in.
    pub field: TextField,
    /// `InputBox.value` as `Show` sets it before the box opens: the value passed in, which is
    /// what `ref value` gets back when the box closes any way but OK.
    passed: String,
    /// `multiline`: Enter and Tab type into the box (`AcceptsReturn`, `AcceptsTab`), so only the
    /// buttons and Escape close it.
    multiline: bool,
}

impl InputBox {
    /// `InputBox.Show(title, prompt, ref value)`: the box holding the value passed in.
    /// `// C#: ExtLibs/Controls/InputBox.cs:37-42, 116-117`
    #[must_use]
    pub fn new(title: &'static str, prompt: &'static str, value: &str) -> Self {
        let mut field = TextField::new("");
        field.set(value);
        Self {
            title,
            prompt,
            field,
            passed: value.to_owned(),
            multiline: false,
        }
    }

    /// `InputBox.Show(title, prompt, ref value, false, true)`: a box of several lines.
    /// `// C#: ExtLibs/Controls/InputBox.cs:37, 86-93, 120-123`
    #[must_use]
    pub fn multiline(title: &'static str, prompt: &'static str, value: &str) -> Self {
        let mut input = Self::new(title, prompt, value);
        input.field.set_multiline(true);
        input.multiline = true;
        input
    }

    /// OK: the answer kept in `Settings.Instance`, as [`remember_answer`] keeps it.
    pub fn remember(&self, settings: &mut Persisted) {
        remember_answer(settings, self.title, self.prompt, self.field.value());
    }

    /// The box closed - by OK when `ok` - and what `Show`'s `ref value` holds after it: OK keeps
    /// the text in `Settings.Instance` and gives it; anything else gives the value passed in and
    /// keeps nothing.
    /// `// C#: ExtLibs/Controls/InputBox.cs:37-60, 176-188`
    pub fn answer(&self, ok: bool, settings: &mut Persisted) -> String {
        if ok {
            self.remember(settings);
            self.field.value().to_owned()
        } else {
            self.passed.clone()
        }
    }

    /// A key in the answer box: `Submitted` for OK - Enter, the form's `AcceptButton`, in a box
    /// of one line - `Cancelled` for Cancel - Escape, its `CancelButton` - `Changed` for typing,
    /// and `Ignored` for a key the box does not take. A box of several lines takes Enter as a new
    /// line and Tab as a tab, each at the caret, and no key is its OK.
    /// `// C#: ExtLibs/Controls/InputBox.cs:86-93, 151-152`
    pub fn key(&mut self, event: &KeyDownEvent) -> KeyOutcome {
        let keystroke = &event.keystroke;
        let plain = !keystroke.modifiers.control && !keystroke.modifiers.platform;
        if self.multiline && plain {
            match keystroke.key.as_str() {
                "enter" => {
                    self.field.insert("\r\n");
                    return KeyOutcome::Changed;
                }
                "tab" => {
                    self.field.insert("\t");
                    return KeyOutcome::Changed;
                }
                _ => {}
            }
        }
        match self.field.key(event) {
            KeyOutcome::Submitted if self.multiline => KeyOutcome::Ignored,
            outcome => outcome,
        }
    }
}

/// The key `InputBox` keeps a question's answers under: `"InputBox" + title.CleanString() +
/// promptText.CleanString()`, `CleanString` keeping the letters and digits.
/// `// C#: ExtLibs/Controls/InputBox.cs:75, 183; ExtLibs/Utilities/Extensions.cs:495-498`
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
/// reads the list back. A password box's answer is kept too, by the owner's ruling (PLAN.md §12
/// D18), where the C# keeps none.
/// `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184; ExtLibs/Utilities/Settings.cs:164-176`
pub fn remember_answer(settings: &mut Persisted, title: &str, prompt: &str, answer: &str) {
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

// ---------------------------------------------------------------------------------------------
// HorizontalProgressBar and HorizontalProgressBar2.
// ---------------------------------------------------------------------------------------------

/// Which way a `HorizontalProgressBar2` fills: from the left, or - its subclass
/// `VerticalProgressBar2`, the class `drawlbl` asks for - from the bottom.
/// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:197; ExtLibs/Controls/VerticalProgressBar2.cs:5-13`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    /// `HorizontalProgressBar2`: filled from the left.
    Horizontal,
    /// `VerticalProgressBar2`: filled from the bottom.
    Vertical,
}

/// A `HorizontalProgressBar`, or a `HorizontalProgressBar2` and its `VerticalProgressBar2`: what
/// `Minimum`, `Maximum` and `Value` read back, and the bar the base `ProgressBar` draws - whose
/// value is held one above its minimum at the bottom and at its maximum at the top, and moved up
/// by half the range once a negative minimum has been set, as both classes' setters do it.
/// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:29-228; ExtLibs/Controls/HorizontalProgressBar2.cs:31-255`
#[derive(Debug, Clone, PartialEq)]
pub struct ProgressBar {
    /// `Label`: the text `HorizontalProgressBar` draws under the bar, and `HorizontalProgressBar2`
    /// in it.
    pub label: String,
    /// `reverse`, `HorizontalProgressBar2`'s: a value set is mirrored about the range.
    /// `HorizontalProgressBar` has none, and leaves it false.
    pub reverse: bool,
    /// `minline`: a red line.
    pub minline: i32,
    /// `maxline`: another.
    pub maxline: i32,
    /// `DisplayScale`, `HorizontalProgressBar2`'s: what the value is multiplied by in its text.
    pub display_scale: f32,
    /// `_min`: what `Minimum` reads back.
    minimum: i32,
    /// `_max`: what `Maximum` reads back.
    maximum: i32,
    /// `_value`: what `Value` reads back - on a reversed bar, the mirrored value.
    value: i32,
    /// `offset`: half the range, once a negative `Minimum` has been set.
    offset: i32,
    /// The base bar's `Minimum`.
    base_minimum: i32,
    /// The base bar's `Maximum`.
    base_maximum: i32,
    /// The base bar's `Value`: what is drawn.
    base_value: i32,
}

impl Default for ProgressBar {
    /// `new HorizontalProgressBar()`.
    fn default() -> Self {
        Self::new()
    }
}

impl ProgressBar {
    /// The base bar as its constructor leaves it, 0 to 100 at 0, under a class whose own fields
    /// are still zero.
    /// `// C#: ExtLibs/BSE.Windows.Forms/ProgressBar/ProgressBar.cs:225-239`
    const fn base() -> Self {
        Self {
            label: String::new(),
            reverse: false,
            minline: 0,
            maxline: 0,
            display_scale: 1.0,
            minimum: 0,
            maximum: 0,
            value: 0,
            offset: 0,
            base_minimum: 0,
            base_maximum: 100,
            base_value: 0,
        }
    }

    /// `new HorizontalProgressBar()`: `Maximum = 100`, then `Minimum = 0`.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:66-72`
    #[must_use]
    pub fn new() -> Self {
        let mut bar = Self::base();
        bar.set_maximum(100);
        bar.set_minimum(0);
        bar
    }

    /// `new HorizontalProgressBar2()`: its own `Minimum` and `Maximum` 0 until they are set, over
    /// the base bar's 0 to 100.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:34-37, 70-83`
    #[must_use]
    pub const fn new2() -> Self {
        Self::base()
    }

    /// A bar given its range as the Designers give one - `Maximum`, then `Minimum` - over
    /// `new HorizontalProgressBar()`.
    #[must_use]
    pub fn ranged(minimum: i32, maximum: i32) -> Self {
        let mut bar = Self::new();
        bar.set_maximum(maximum);
        bar.set_minimum(minimum);
        bar
    }

    /// `Minimum`'s setter. A negative minimum puts the base bar's at zero and its maximum the
    /// range above it, and moves every value up by `(_max - value) / 2` - the maximum as it is at
    /// that moment, the division an integer's; any other minimum is the base bar's own, and leaves
    /// an offset already set as it is.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:117-142; ExtLibs/Controls/HorizontalProgressBar2.cs:119-134`
    pub fn set_minimum(&mut self, minimum: i32) {
        self.minimum = minimum;
        if minimum < 0 {
            self.set_base_minimum(0);
            self.offset = self.maximum.saturating_sub(minimum) / 2;
            self.set_base_maximum(self.maximum.saturating_sub(minimum));
        } else {
            self.set_base_minimum(minimum);
        }
    }

    /// `Maximum`'s setter: the base bar's maximum too.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:144-145; ExtLibs/Controls/HorizontalProgressBar2.cs:136`
    pub fn set_maximum(&mut self, maximum: i32) {
        self.maximum = maximum;
        self.set_base_maximum(maximum);
    }

    /// The base bar's `Minimum` setter: its maximum raised to it, its value held above it.
    /// `// C#: ExtLibs/BSE.Windows.Forms/ProgressBar/ProgressBar.cs:169-193`
    const fn set_base_minimum(&mut self, minimum: i32) {
        if self.base_maximum < minimum {
            self.base_maximum = minimum;
        }
        self.base_minimum = minimum;
        if self.base_value < minimum {
            self.base_value = minimum;
        }
    }

    /// The base bar's `Maximum` setter: its minimum lowered to it, its value held below it.
    /// `// C#: ExtLibs/BSE.Windows.Forms/ProgressBar/ProgressBar.cs:135-159`
    const fn set_base_maximum(&mut self, maximum: i32) {
        if self.base_minimum > maximum {
            self.base_minimum = maximum;
        }
        self.base_maximum = maximum;
        if self.base_value > maximum {
            self.base_value = maximum;
        }
    }

    /// `Value`'s setter: nothing for the value already stored - which, on a reversed bar, is the
    /// mirrored one; otherwise stored, mirrored about `Minimum` and `Maximum` on a reversed bar,
    /// and drawn moved up by the offset, held one above the base bar's minimum ("prevent an
    /// exception for the -1 hack") and at most its maximum.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:74-115; ExtLibs/Controls/HorizontalProgressBar2.cs:85-117`
    pub fn set_value(&mut self, value: i32) {
        if self.value == value {
            return;
        }
        self.value = value;
        if self.reverse {
            let dif = value.saturating_sub(self.minimum);
            self.value = self.maximum.saturating_sub(dif);
        }
        self.base_value = self.base_value_for(self.value);
    }

    /// The base bar's value for a value the setter takes: moved up by the offset, held one above
    /// the base bar's minimum and at most its maximum.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:83-104; ExtLibs/Controls/HorizontalProgressBar2.cs:101-111`
    const fn base_value_for(&self, value: i32) -> i32 {
        let ans = value.saturating_add(self.offset);
        if ans <= self.base_minimum {
            self.base_minimum.saturating_add(1)
        } else if ans >= self.base_maximum {
            self.base_maximum
        } else {
            ans
        }
    }

    /// `Value`'s getter: the value set, mirrored on a reversed bar - what the label writes.
    #[must_use]
    pub const fn value(&self) -> i32 {
        self.value
    }

    /// How full the base bar is drawn, from nothing to the whole: its value along its range.
    #[must_use]
    pub fn fraction(&self) -> f32 {
        self.fraction_of(self.base_value)
    }

    /// How full the base bar is drawn once the setter has taken `value` - a value it changed to -
    /// on a bar that is not reversed: what a page drawing its bar from the vehicle's state each
    /// frame draws. The C#'s bar also keeps the value it last had, which tells only before its
    /// first change: one never changed from its first value draws its base bar as the Designer
    /// left it.
    #[must_use]
    pub fn fraction_at(&self, value: i32) -> f32 {
        self.fraction_of(self.base_value_for(value))
    }

    /// A base bar's value along its range.
    fn fraction_of(&self, base_value: i32) -> f32 {
        let range = self.base_maximum.saturating_sub(self.base_minimum);
        if range <= 0 {
            return 0.0;
        }
        #[allow(clippy::cast_precision_loss)] // a bar's values are small integers
        let fraction = base_value.saturating_sub(self.base_minimum) as f32 / range as f32;
        fraction.clamp(0.0, 1.0)
    }

    /// The text `HorizontalProgressBar2` draws across itself: `(Label + "  " + (Value *
    /// DisplayScale).ToString() + " ").Trim()`, the product a float as .NET writes one.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:185`
    #[must_use]
    pub fn text(&self) -> String {
        #[allow(clippy::cast_precision_loss)] // a bar's values are small integers
        let shown = self.value as f32 * self.display_scale;
        format!("{}  {} ", self.label, mp_log::netfmt::single(shown))
            .trim()
            .to_owned()
    }

    /// Where `HorizontalProgressBar2` draws its red lines along a bar `length` long, `minline`'s
    /// first, or `None` when it draws none - unless both lines are set - or the range is empty. A
    /// vertical bar's are offsets from its top, the second six pixels further down; a reversed
    /// bar's run the other way.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:189-233`
    #[must_use]
    pub fn lines(&self, orientation: Orientation, length: f32) -> Option<(f32, f32)> {
        if self.minline == 0 || self.maxline == 0 || self.maximum == self.minimum {
            return None;
        }
        #[allow(clippy::cast_precision_loss)] // a bar's values are small integers
        let along = |from: i32, to: i32| {
            to.saturating_sub(from) as f32 / self.maximum.saturating_sub(self.minimum) as f32
        };
        let (minimum, maximum) = (self.minimum, self.maximum);
        let (min, max) = (self.minline, self.maxline);
        Some(match (orientation, self.reverse) {
            (Orientation::Vertical, true) => (
                along(minimum, min) * length + 6.0,
                along(minimum, max) * length,
            ),
            (Orientation::Vertical, false) => (
                along(min, maximum) * length,
                along(max, maximum) * length + 6.0,
            ),
            (Orientation::Horizontal, true) => {
                (along(min, maximum) * length, along(max, maximum) * length)
            }
            (Orientation::Horizontal, false) => {
                (along(minimum, min) * length, along(minimum, max) * length)
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gpui::{Keystroke, Modifiers};
    use mp_params::UserLevel;

    /// A parameter's documentation, this test's own: a range and an increment, or values.
    const fn meta(
        name: &'static str,
        range: Option<(f64, f64)>,
        increment: Option<f64>,
        values: &'static [(i64, &'static str)],
    ) -> ParamMeta {
        ParamMeta {
            name,
            display_name: name,
            description: "",
            units: "",
            range,
            range_text: "",
            increment,
            values,
            bitmask: &[],
            user_level: UserLevel::Standard,
            reboot_required: false,
        }
    }

    static RANGED: ParamMeta = meta("RANGED", Some((0.0, 10.0)), Some(0.5), &[]);
    static LISTED: ParamMeta = meta(
        "LISTED",
        None,
        None,
        &[(0, " Off "), (1, "On"), (5, "Five")],
    );
    static OTHER: ParamMeta = meta("OTHER", None, None, &[(7, "Seven")]);

    fn lookup(name: &str) -> Option<&'static ParamMeta> {
        match name {
            "RANGED" => Some(&RANGED),
            "LISTED" => Some(&LISTED),
            "OTHER" => Some(&OTHER),
            _ => None,
        }
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
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

    // --- MavlinkCheckBox ---------------------------------------------------------------------

    /// `setup` checks the box for the on value, unchecks it for the off value and leaves it
    /// indeterminate for anything else; every click writes the value it leaves the box at, and
    /// fails as `Strings.ErrorSetValueFailed` says.
    /// `// C#: Controls/MavlinkCheckBox.cs:60-143`
    #[test]
    fn a_check_box_shows_the_vehicles_value_and_writes_each_click() {
        let mut check = Check::default();
        check.setup(-1.0, 1.0, "RC1_REV", &table(&[("RC1_REV", -1.0)]));
        assert_eq!(check.state, CheckState::Checked);
        assert!(check.enabled && check.visible);
        assert_eq!(check.param_name(), Some("RC1_REV"));
        let write = check.click().expect("a write");
        assert_eq!(write, Write::other("RC1_REV", 1.0));
        assert_eq!(write.failure, "Set RC1_REV Failed");
        assert_eq!(check.state, CheckState::Unchecked);
        assert_eq!(check.click().map(|write| write.value), Some(-1.0));
        assert_eq!(check.state, CheckState::Checked);

        // Neither value: indeterminate, which a click unchecks.
        check.setup(1.0, 0.0, "X", &table(&[("X", 2.0)]));
        assert_eq!(check.state, CheckState::Indeterminate);
        assert!(check.checked(), "an indeterminate box is Checked");
        assert_eq!(check.click().map(|write| write.value), Some(0.0));
        assert_eq!(check.state, CheckState::Unchecked);
    }

    /// Without its parameter the box is disabled and takes no click, keeps the name and its state;
    /// a box the `.resx` hides shows only once a `setup` finds its parameter.
    /// `// C#: Controls/MavlinkCheckBox.cs:24-30, 60-98`
    #[test]
    fn a_check_box_without_its_parameter_is_disabled_and_keeps_its_state() {
        let mut fresh = Check::default();
        assert!(!fresh.enabled && fresh.visible);
        assert_eq!(fresh.param_name(), None);
        assert_eq!(fresh.click(), None, "a box never set up writes nothing");

        let mut check = Check::default();
        check.setup(1.0, 0.0, "A", &table(&[("A", 1.0)]));
        check.setup(1.0, 0.0, "B", &[]);
        assert!(!check.enabled);
        assert_eq!(check.param, "B");
        assert_eq!(check.state, CheckState::Checked, "left as it was");
        assert_eq!(check.click(), None);

        let mut hidden = Check::hidden();
        hidden.setup(1.0, 0.0, "A", &[]);
        assert!(!hidden.visible && !hidden.enabled);
        hidden.setup(1.0, 0.0, "A", &table(&[("A", 0.0)]));
        assert!(hidden.visible && hidden.enabled);
        hidden.visible = false;
        assert_eq!(hidden.click(), None, "a hidden box takes no click");
    }

    /// The array forms: the first name the vehicle has - with its own values for the choices -
    /// and nothing at all when it has none.
    /// `// C#: Controls/MavlinkCheckBox.cs:32-58`
    #[test]
    fn a_check_box_takes_the_first_name_or_choice_the_vehicle_has() {
        let mut check = Check::default();
        let parameters = table(&[("COMPASS1_USE", 1.0)]);
        check.setup_names(1.0, 0.0, &["COMPASS_USE", "COMPASS1_USE"], &parameters);
        assert_eq!(check.param_name(), Some("COMPASS1_USE"));
        assert_eq!(check.state, CheckState::Checked);
        check.setup_names(1.0, 0.0, &["NONE"], &[]);
        assert_eq!(check.param_name(), Some("COMPASS1_USE"), "untouched");
        assert!(check.enabled);

        let choices = [(-1.0, 1.0, "RC2_REV"), (1.0, 0.0, "RC2_REVERSED")];
        let mut reverse = Check::default();
        reverse.setup_choices(&choices, &table(&[("RC2_REVERSED", 1.0)]));
        assert_eq!(reverse.param_name(), Some("RC2_REVERSED"));
        assert_eq!(reverse.state, CheckState::Checked);
        assert_eq!(reverse.click().map(|write| write.value), Some(0.0));
        let mut none = Check::default();
        none.setup_choices(&choices, &[]);
        assert_eq!(none, Check::default());
    }

    /// `enableBGControl`: the control behind the box enabled while it is checked.
    /// `// C#: Controls/MavlinkCheckBox.cs:76-90, 106-143`
    #[test]
    fn the_control_behind_a_check_box_follows_its_tick() {
        let mut check = Check::default();
        check.setup(1.0, 0.0, "THR_FAILSAFE", &table(&[("THR_FAILSAFE", 1.0)]));
        assert!(check.background_enabled());
        check.click();
        assert!(!check.background_enabled());
        check.setup(1.0, 0.0, "THR_FAILSAFE", &table(&[("THR_FAILSAFE", 2.0)]));
        assert!(!check.background_enabled(), "indeterminate disables it");
    }

    // --- MavlinkComboBox ---------------------------------------------------------------------

    /// The list form: the list bound, the vehicle's `(int)` value selected, and a choice that
    /// changes the selection written as `(float)(int)SelectedValue`, failing with "!".
    /// `// C#: Controls/MavlinkComboBox.cs:73-99, 169-199`
    #[test]
    fn a_combo_box_selects_the_vehicles_value_and_writes_a_change() {
        let mut combo = Combo::default();
        combo.setup(
            options("LISTED", lookup),
            "LISTED",
            &table(&[("LISTED", 1.9)]),
        );
        assert_eq!(combo.options[0], (0, "Off".to_owned()), "trimmed");
        assert_eq!(combo.selected, Some(1), "(int) truncates");
        assert_eq!(combo.text(), "On");
        assert!(combo.enabled);
        assert_eq!(combo.choose(1), None, "no change, no write");
        assert_eq!(combo.choose(99), None, "not in the list");
        let write = combo.choose(5).expect("a write");
        assert_eq!(write, Write::combo("LISTED", 5.0));
        assert_eq!(write.failure, "Set LISTED Failed!");
        assert_eq!(combo.text(), "Five");

        // A value the list does not hold selects nothing.
        combo.setup(
            options("LISTED", lookup),
            "LISTED",
            &table(&[("LISTED", 3.0)]),
        );
        assert_eq!(combo.selected, None);
        assert_eq!(combo.text(), "");
    }

    /// Without its parameter the list is bound - its first row showing, as `DataSource` leaves
    /// it - the name taken, and the box stays disabled and takes no choice.
    /// `// C#: Controls/MavlinkComboBox.cs:31-35, 73-99`
    #[test]
    fn a_combo_box_without_its_parameter_shows_its_first_row_disabled() {
        let mut combo = Combo::default();
        combo.setup(options("LISTED", lookup), "LISTED", &[]);
        assert!(!combo.enabled);
        assert_eq!(combo.param, "LISTED");
        assert_eq!(combo.text(), "Off");
        assert_eq!(combo.choose(5), None);
        assert_eq!(combo.selected, Some(0));
    }

    /// The names form: the first name the vehicle has, bound to its documented values; with none
    /// of them, nothing at all.
    /// `// C#: Controls/MavlinkComboBox.cs:37-71`
    #[test]
    fn a_combo_box_by_names_binds_the_first_listed_names_values() {
        let mut combo = Combo::default();
        combo.setup_names(&["LISTED", "OTHER"], &table(&[("OTHER", 7.0)]), lookup);
        assert_eq!(combo.param, "OTHER");
        assert_eq!(combo.options, [(7, "Seven".to_owned())]);
        assert_eq!(combo.selected, Some(7));
        assert!(combo.enabled);

        let mut untouched = Combo::default();
        untouched.setup_names(&["LISTED"], &[], lookup);
        assert_eq!(untouched, Combo::default());
    }

    /// `SelectedIndex` from code writes whether or not the box is enabled; a list of an enum's
    /// names writes the name's value and fails without the "!".
    /// `// C#: Controls/MavlinkComboBox.cs:102-168`
    #[test]
    fn an_index_from_code_writes_and_an_enum_choice_fails_without_the_bang() {
        let mut combo = Combo::default();
        combo.setup(options("LISTED", lookup), "LISTED", &[]);
        assert!(!combo.enabled);
        assert_eq!(combo.select_index(0), None, "already selected");
        assert_eq!(combo.select_index(2), Some(Write::combo("LISTED", 5.0)));
        assert_eq!(combo.select_index(3), None, "no such row");

        let names = vec![(0, "Disable".to_owned()), (5, "RC5".to_owned())];
        let mut input = Combo::default();
        input.setup(
            names,
            "MNT1_RC_IN_TILT",
            &table(&[("MNT1_RC_IN_TILT", 0.0)]),
        );
        let write = input.choose_named(5).expect("a write");
        assert_eq!(write.value, 5.0);
        assert_eq!(write.failure, "Set MNT1_RC_IN_TILT Failed");
        assert_eq!(input.choose_named(5), None, "no change");
    }

    // --- MavlinkNumericUpDown ----------------------------------------------------------------

    fn stepped(how: Setup, name: &str, parameters: &[(String, f64)]) -> Number {
        let mut number = Number::new(NUMERIC_DEFAULTS);
        number.setup(how, name, parameters, lookup);
        number
    }

    const ONE: Setup = Setup {
        minimum: 0.0,
        maximum: 1.0,
        scale: 1.0,
        increment: 1.0,
    };

    /// `setup`: the documented range over the one passed, the documented increment when larger
    /// than the decimal places, the places the increment has, and the vehicle's value - divided by
    /// the scale - widening the range when it is outside it.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:49-122`
    #[test]
    fn a_number_takes_the_documented_range_and_increment_and_the_vehicles_value() {
        let number = stepped(ONE, "RANGED", &table(&[("RANGED", 2.5)]));
        assert!(number.enabled && number.visible);
        assert_eq!(number.param, "RANGED");
        assert_eq!((number.minimum, number.maximum), (0.0, 10.0));
        assert!((number.increment() - 0.5).abs() < 1e-9);
        assert_eq!(number.decimals, 1);
        assert!((number.value() - 2.5).abs() < 1e-9);
        assert_eq!(number.shown(), "2.5");

        let wide = stepped(ONE, "RANGED", &table(&[("RANGED", 12.0)]));
        assert_eq!(wide.maximum, 12.0, "the value widens the range");

        let scaled = stepped(
            Setup {
                scale: 100.0,
                ..ONE
            },
            "UNDOCUMENTED",
            &table(&[("UNDOCUMENTED", 250.0)]),
        );
        assert_eq!((scaled.minimum, scaled.maximum), (0.0, 2.5));
        assert_eq!(scaled.shown(), "2.5");
        assert!((scaled.written() - 250.0).abs() < 1e-3);
    }

    /// Without its parameter the box is disabled and takes no step, and a hidden one stays hidden;
    /// the names form binds the first the vehicle has, else the first.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:31-64, 115-119`
    #[test]
    fn a_number_without_its_parameter_is_disabled_and_stays_hidden() {
        let designer = Designer {
            minimum: 0.0,
            maximum: 100.0,
            value: 13.1,
            decimals: 1,
        };
        let mut number = Number::hidden(designer);
        number.setup(ONE, "RANGED", &[], lookup);
        assert!(!number.enabled && !number.visible);
        assert_eq!(number.param, "RANGED");
        assert_eq!(number.step(true, Instant::now()), None);
        assert!(!number.pending(), "a disabled box does not step");

        assert_eq!(bound_name(&["A", "B"], &table(&[("B", 1.0)])), "B");
        assert_eq!(bound_name(&["A", "B"], &[]), "A");
        assert_eq!(bound_name(&[], &[]), "");
        let mut named = Number::new(NUMERIC_DEFAULTS);
        let parameters = table(&[("RANGED", 1.0)]);
        named.setup_names(ONE, &["OLD", "RANGED"], &parameters, lookup);
        assert_eq!(named.param, "RANGED");
        assert!(named.enabled);
    }

    /// A value typed above the maximum asks "Out of range" first; Yes takes it as the maximum and
    /// the value. Either way the write follows 300 ms after, once, with the box's value.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:133-180`
    #[test]
    fn a_value_typed_above_the_maximum_asks_and_the_write_waits() {
        let now = Instant::now();
        let mut number = stepped(ONE, "RANGED", &table(&[("RANGED", 2.5)]));
        number.type_text("20");
        let question = number.commit(now).expect("the question");
        assert_eq!(
            question.text(),
            "RANGED Value out of range\nDo you want to accept the new value?"
        );
        assert!(!number.pending(), "the timer waits for the answer");
        number.answer(&question, true, now);
        assert_eq!((number.maximum, number.value()), (20.0, 20.0));
        assert_eq!(number.due(now), None, "not yet");
        let write = number.due(now + WRITE_DELAY).expect("the write");
        assert_eq!(write, Write::other("RANGED", 20.0));
        assert_eq!(number.due(now + WRITE_DELAY), None, "once");

        let mut no = stepped(ONE, "RANGED", &table(&[("RANGED", 2.5)]));
        no.type_text("20");
        let question = no.commit(now).expect("the question");
        no.answer(&question, false, now);
        assert_eq!(no.value(), 10.0, "held to the maximum");
        let written = no.due(now + WRITE_DELAY).map(|write| write.value);
        assert_eq!(written, Some(10.0));

        // A step: one increment, written after the timer.
        let mut step = stepped(ONE, "RANGED", &table(&[("RANGED", 2.5)]));
        assert_eq!(step.step(true, now), None);
        assert_eq!(step.shown(), "3.0");
        assert_eq!(step.flush().map(|write| write.value), Some(3.0));
    }

    // --- InputBox ----------------------------------------------------------------------------

    /// `Show`'s `ref value`: OK gives the text and keeps it under the question's key; Cancel gives
    /// the value passed in and keeps nothing.
    /// `// C#: ExtLibs/Controls/InputBox.cs:37-60, 73-84, 176-188`
    #[test]
    fn an_input_box_gives_its_text_on_ok_and_the_value_passed_in_on_cancel() {
        let mut settings = Persisted::at(None);
        let key = answers_key("Change Throttle", "Enter arm throttle % (deadzone + 2%)");
        assert_eq!(key, "InputBoxChangeThrottleEnterarmthrottledeadzone2");
        let question = "Enter arm throttle % (deadzone + 2%)";
        let mut input = InputBox::new("Change Throttle", question, "7");
        assert_eq!(input.field.value(), "7");
        input.field.set("9");
        assert_eq!(input.answer(false, &mut settings), "7");
        assert_eq!(settings.get(&key), None, "Cancel keeps nothing");
        assert_eq!(input.answer(true, &mut settings), "9");
        assert_eq!(settings.get(&key), Some("9"));

        // Enter is OK and Escape Cancel in a box of one line.
        assert_eq!(input.key(&press("a", Some("a"))), KeyOutcome::Changed);
        assert_eq!(input.field.value(), "9a");
        assert_eq!(input.key(&press("enter", None)), KeyOutcome::Submitted);
        assert_eq!(input.key(&press("escape", None)), KeyOutcome::Cancelled);
    }

    /// A box of several lines takes Enter as a new line and Tab as a tab; only Escape closes it.
    /// `// C#: ExtLibs/Controls/InputBox.cs:86-93`
    #[test]
    fn a_multi_line_input_box_types_enter_and_tab() {
        let mut input = InputBox::multiline("Params", "Enter Param Names", "A");
        assert_eq!(input.key(&press("enter", None)), KeyOutcome::Changed);
        assert_eq!(input.key(&press("tab", None)), KeyOutcome::Changed);
        assert_eq!(input.field.value(), "A\r\n\t");
        assert_eq!(input.key(&press("escape", None)), KeyOutcome::Cancelled);
        let mut settings = Persisted::at(None);
        assert_eq!(input.answer(false, &mut settings), "A");
    }

    // --- HorizontalProgressBar and HorizontalProgressBar2 ------------------------------------

    /// The value is held one above the minimum and at the maximum for drawing, and read back as
    /// set; a value the bar already has changes nothing.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:66-145`
    #[test]
    fn a_bar_holds_its_value_one_above_the_minimum_and_at_the_maximum() {
        let mut bar = ProgressBar::new();
        bar.set_value(0);
        assert!(
            bar.fraction().abs() < 1e-6,
            "0 is the value it has: nothing set"
        );
        bar.set_value(40);
        assert!((bar.fraction() - 0.4).abs() < 1e-6);
        bar.set_value(0);
        assert!(
            (bar.fraction() - 0.01).abs() < 1e-6,
            "one above the minimum"
        );
        bar.set_value(250);
        assert_eq!(bar.value(), 250);
        assert!((bar.fraction() - 1.0).abs() < 1e-6);

        let radio = ProgressBar::ranged(1000, 2000);
        assert!((radio.fraction_at(0) - 0.001).abs() < 1e-6);
        assert!((radio.fraction_at(1500) - 0.5).abs() < 1e-6);
        assert!((radio.fraction_at(65_535) - 1.0).abs() < 1e-6);
        assert!(
            radio.fraction().abs() < 1e-6,
            "never set: the base bar at its minimum"
        );
    }

    /// A negative minimum moves every value up by half the range, the maximum read as it is when
    /// the minimum is set.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:117-145`
    #[test]
    fn a_negative_minimum_moves_the_bar_up_by_half_the_range() {
        let mut bar = ProgressBar::ranged(-100, 100);
        bar.set_value(1);
        assert!((bar.fraction() - 0.505).abs() < 1e-6);
        bar.set_value(-100);
        assert!((bar.fraction() - 0.005).abs() < 1e-6);
        // Set before the maximum, the offset is half of 0 - -50, and the maximum set after it the
        // base bar's own.
        let mut early = ProgressBar::new2();
        early.set_minimum(-50);
        early.set_maximum(100);
        early.set_value(10);
        assert!((early.fraction() - 0.35).abs() < 1e-6);
    }

    /// `reverse`: a value set is mirrored about the range, and that is what the bar draws and
    /// writes; its text is `Label`, two spaces and the value.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:85-117, 185`
    #[test]
    fn a_reversed_bar_mirrors_its_value_and_writes_it() {
        let mut bar = ProgressBar::new2();
        bar.label = "Roll".to_owned();
        bar.set_maximum(2200);
        bar.set_minimum(800);
        bar.set_value(1500);
        assert_eq!(bar.text(), "Roll  1500");
        bar.reverse = true;
        bar.set_value(1100);
        assert_eq!(bar.value(), 1900);
        assert_eq!(bar.text(), "Roll  1900");
        // The mirrored value is the one stored: setting it again changes nothing.
        bar.set_value(1900);
        assert_eq!(bar.value(), 1900);
        bar.label.clear();
        assert_eq!(bar.text(), "1900", "trimmed");
    }

    /// The red lines: drawn only when both are set, each along the range, a vertical bar's second
    /// six pixels further down.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:189-233`
    #[test]
    fn the_red_lines_are_drawn_only_when_both_are_set() {
        let mut bar = ProgressBar::new2();
        bar.set_maximum(2200);
        bar.set_minimum(800);
        bar.minline = 1100;
        assert_eq!(bar.lines(Orientation::Horizontal, 140.0), None);
        bar.maxline = 1900;
        let near = |got: Option<(f32, f32)>, want: (f32, f32)| {
            got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-3 && (b - want.1).abs() < 1e-3)
        };
        assert!(near(
            bar.lines(Orientation::Horizontal, 140.0),
            (30.0, 110.0)
        ));
        assert!(near(bar.lines(Orientation::Vertical, 140.0), (110.0, 36.0)));
        bar.reverse = true;
        assert!(near(
            bar.lines(Orientation::Horizontal, 140.0),
            (110.0, 30.0)
        ));
        assert!(near(bar.lines(Orientation::Vertical, 140.0), (36.0, 110.0)));
    }
}
