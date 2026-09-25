//! The Servo Output page of Initial Setup: `GCSViews/ConfigurationView/ConfigRadioOutput.cs`, a
//! Mandatory Hardware entry (`GCSViews/InitialSetup.cs:213-216`), listed once every parameter is
//! in.
//!
//! What it shows: a table with one row per servo output - its number, a bar with the pulse width
//! the vehicle is putting out on it (`SERVO_OUTPUT_RAW`), `SERVOn_REVERSED` as a check box,
//! `SERVOn_FUNCTION` as a combo box of the functions the parameter documentation lists, and
//! `SERVOn_MIN`, `SERVOn_TRIM` and `SERVOn_MAX` as numbers. Sixteen rows, or thirty-two when
//! `SERVO_32_ENABLE` is above zero (`ConfigRadioOutput.cs:17-30`); every row is built whether or
//! not the vehicle has its parameters, and a control whose parameter it lacks stays disabled.
//! Each control is one of the `Mavlink*` controls and writes its own parameter: a combo box or a
//! check box from its change handler, a number 300 ms after it changes. There is no save button
//! in the C# and there is none here.
//!
//! The rows are made by the page's constructor, not by `Activate`: the page object is created the
//! first time it is shown after the list is loaded (`BackstageViewPage.cs:50-62`) and kept, with
//! whatever its controls hold, until the screen is left or loaded again. `Activate` and
//! `Deactivate` start and stop the timer that moves the bars (`ConfigRadioOutput.cs:71-92`); here
//! the bars are drawn from the vehicle's state whenever the page shows, which is when the timer
//! runs.
//!
//! The three controls - `MavlinkComboBox`, `MavlinkCheckBox`, `MavlinkNumericUpDown` - are here as
//! [`Combo`], [`Check`] and [`Number`], with their drawing, for the ESC Calibration page too.
//! A number takes typed text and its arrows, as a `NumericUpDown` does: the text is read into its
//! value when the box is left, on Enter and on an arrow; a typed value above the maximum asks
//! whether to accept it (`MavlinkNumericUpDown.cs:133-161`).
//!
//! The geometry is the Designer's table: seven auto-sized columns whose widths are the widest
//! control each holds - the `#` label, the 100-pixel bar, the `Reverse` header, the 160-pixel
//! combo box, three 50-pixel numbers - each with WinForms' three-pixel margins. The colours are
//! this application's.
//!
//! What is not ported, and why:
//!
//! * an `RCn_*` fallback for firmware without `SERVOn_*`: the C# has none. On such a vehicle every
//!   row's controls stay disabled, as the C#'s do;
//! * the controls' `ValueUpdated` events, which nothing on this page subscribes to, and the
//!   numbers' mouse wheel;
//! * the drop-down list's eight-row height (`MaxDropDownItems`): the list shows up to thirty rows
//!   and scrolls, because gpui's list does not open scrolled to the selected row as WinForms'
//!   does, and a function beyond the eighth would otherwise be out of sight.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::rc::Rc;
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px,
    rgb,
};
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;

use crate::MissionPlanner;
pub use crate::config::failsafe::{CheckState, Lookup};
use crate::config::failsafe::{decimal_of, options};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:215`
pub const TITLE: &str = "Servo Output";

/// `Strings.ERROR`: the caption of a failed write's message box.
/// `// C#: ExtLibs/Strings/Strings.resx:130-132`
pub const ERROR_TITLE: &str = "Error";

/// The caption of `MavlinkNumericUpDown`'s out-of-range question.
/// `// C#: Controls/MavlinkNumericUpDown.cs:139-140`
pub const OUT_OF_RANGE_TITLE: &str = "Out of range";

/// How long a number waits before it writes: a change sets a 300 ms timer going - only when it is
/// not going already, so a burst of changes is written once, 300 ms after the first - and the
/// timer writes the value the box holds when it fires.
/// `// C#: Controls/MavlinkNumericUpDown.cs:154-180`
pub const WRITE_DELAY: Duration = Duration::from_millis(300);

/// The bars' scale: `Minimum = 800, Maximum = 2200`.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:41-45`
const BAR_MINIMUM: i64 = 800;
/// The top of that scale.
const BAR_MAXIMUM: i64 = 2200;

/// Each number as the constructor makes it: `Minimum = 800, Maximum = 2200, Value = 1500`, and the
/// `NumericUpDown` defaults for the rest.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:50-52`
const PWM_BOX: Designer = Designer {
    minimum: 800.0,
    maximum: 2200.0,
    value: 1500.0,
    decimals: 0,
};

/// `setup(800, 2200, 1, 1, ...)` for each of the three numbers; Heli Setup's servo rows too.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:66-68; ConfigTradHeli4.cs:182-184`
pub const PWM_SETUP: Setup = Setup {
    minimum: 800.0,
    maximum: 2200.0,
    scale: 1.0,
    increment: 1.0,
};

/// The header row: `label15`, `label1` to `label6`, in their columns.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.Designer.cs:58-64, 88-148`
const HEADERS: [&str; 7] = ["#", "Position", "Reverse", "Function", "Min", "Trim", "Max"];

/// The seven columns' widths: each the widest control in it and its two three-pixel margins - the
/// header labels' `Size`s, the bar's default 100, the combo box's `Width = 160`, the numbers'
/// `Width = 50`.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:39-52; ConfigRadioOutput.Designer.cs:88-148`
const COLUMNS: [f32; 7] = [20.0, 106.0, 53.0, 166.0, 56.0, 56.0, 56.0];

/// The header row's height: a 13-pixel label and its margins.
const HEADER_HEIGHT: f32 = 19.0;
/// A servo row's: the 23-pixel bar and its margins.
const ROW_HEIGHT: f32 = 29.0;
/// The table's `Location`.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.Designer.cs:65`
const TABLE_AT: (f32, f32) = (3.0, 3.0);
/// A drop-down list's row, and how many show before it scrolls.
const LIST_ROW: f32 = 16.0;
/// See the module's notes.
/// How many rows a dropped-down list shows at once; the rest come into view on the wheel, as a
/// WinForms drop-down scrolls (it shows `MaxDropDownItems`, 8, and scrolls the rest).
pub const LIST_ROWS_SHOWN: usize = 30;
/// A line of the wheel, in rows. gpui reports the wheel in lines already multiplied by the
/// system's lines per notch - three on X11 and on Windows (`SPI_GETWHEELSCROLLLINES`) - and a
/// WinForms list scrolls one item per line, so a notch moves three rows.
pub const WHEEL_ROWS: usize = 1;

// ---------------------------------------------------------------------------------------------
// The Mavlink* controls.
// ---------------------------------------------------------------------------------------------

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
    /// A combo box's write, whose failure is its own text with an exclamation mark.
    /// `// C#: Controls/MavlinkComboBox.cs:180-183, 195-198`
    #[must_use]
    pub fn combo(param: &str, value: f64) -> Self {
        Self {
            param: param.to_owned(),
            value,
            failure: format!("Set {param} Failed!"),
        }
    }

    /// A check box's or a number's write, which fails with `Strings.ErrorSetValueFailed`,
    /// "Set {0} Failed".
    /// `// C#: Controls/MavlinkCheckBox.cs:116-140; Controls/MavlinkNumericUpDown.cs:169-176;
    /// ExtLibs/Strings/Strings.resx:171-173`
    #[must_use]
    pub fn other(param: &str, value: f64) -> Self {
        Self {
            param: param.to_owned(),
            value,
            failure: format!("Set {param} Failed"),
        }
    }
}

/// A modal message box, `CustomMessageBox.Show(text, caption)`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    /// The caption.
    pub title: &'static str,
    /// The text.
    pub text: String,
}

/// A parameter's value in the vehicle's table.
#[must_use]
pub fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// A `MavlinkComboBox` set up with a list: `setup(List<KeyValuePair<int, string>>, paramname,
/// paramlist)`. Also the plain `ComboBox`es of the Serial Ports page, which bind a list the same
/// way and write through the page.
/// `// C#: Controls/MavlinkComboBox.cs:73-99`
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

    /// `setup`: binds the list, which selects its first row as a WinForms list control does when
    /// its `DataSource` is set, then - for a parameter the vehicle has - enables the box and
    /// selects `(int)` its value, or nothing for a value the list does not hold.
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
}

/// A `MavlinkCheckBox`: `setup(OnValue, OffValue, paramname, paramlist)`.
/// `// C#: Controls/MavlinkCheckBox.cs:24-98`
#[derive(Debug, Clone, PartialEq)]
pub struct Check {
    /// `ParamName`.
    pub param: String,
    /// What the box shows.
    pub state: CheckState,
    /// `Enabled`: false from the constructor, true for a parameter the vehicle has.
    pub enabled: bool,
    /// `OnValue`.
    on: f64,
    /// `OffValue`.
    off: f64,
}

impl Default for Check {
    /// The constructor: `OnValue = 1`, `OffValue = 0`, disabled.
    fn default() -> Self {
        Self {
            param: String::new(),
            state: CheckState::Unchecked,
            enabled: false,
            on: 1.0,
            off: 0.0,
        }
    }
}

impl Check {
    /// `setup`: checked for the on value, unchecked for the off value, indeterminate for anything
    /// else; disabled, and left as it was, for a parameter the vehicle lacks.
    pub fn setup(&mut self, on: f64, off: f64, param: &str, parameters: &[(String, f64)]) {
        self.on = on;
        self.off = off;
        self.param = param.to_owned();
        let Some(value) = value_of(parameters, param) else {
            self.enabled = false;
            return;
        };
        self.enabled = true;
        // `paramlist[paramname].Value == OnValue`: the C#'s exact comparison.
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

    /// A click: WinForms takes a two-state box from checked or indeterminate to unchecked and from
    /// unchecked to checked, and each changes `Checked`, so each writes - `OnValue` when it ends
    /// checked, `OffValue` otherwise.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    pub fn click(&mut self) -> Option<Write> {
        if !self.enabled {
            return None;
        }
        self.state = match self.state {
            CheckState::Unchecked => CheckState::Checked,
            CheckState::Checked | CheckState::Indeterminate => CheckState::Unchecked,
        };
        let value = if self.state == CheckState::Checked {
            self.on
        } else {
            self.off
        };
        Some(Write::other(&self.param, value))
    }
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

/// A `MavlinkNumericUpDown`: bounds, increment and value as the C#'s `decimal`s, the text in its
/// box, and the timer that writes it.
/// `// C#: Controls/MavlinkNumericUpDown.cs:9-181`
#[derive(Debug)]
pub struct Number {
    /// `ParamName`.
    pub param: String,
    /// `Enabled`.
    pub enabled: bool,
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
    /// A box as its Designer leaves it.
    #[must_use]
    pub fn new(designer: Designer) -> Self {
        let mut number = Self {
            param: String::new(),
            enabled: false,
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

    /// `setup(Min, Max, Scale, Increment, paramname, paramlist)`: the documented range over the
    /// one passed, the documented increment when it is larger than the decimal places the box
    /// has - a step compared with a count, as the C# compares them - the decimal places the
    /// increment has, and for a parameter the vehicle has, its value, which widens the range and
    /// the places when it needs to. A parameter the vehicle lacks disables the box and leaves its
    /// value.
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

/// A write on its way.
#[derive(Debug, Clone)]
struct InFlight {
    id: RequestId,
    write: Write,
}

/// The writes a page's controls have handed to the link's retrying set, and what became of them.
#[derive(Debug, Default)]
pub struct Writes {
    in_flight: Vec<InFlight>,
    /// How the last write ended, for the facts.
    last: Option<String>,
}

impl Writes {
    /// Sends a write through the retrying set; with no vehicle it has failed already, and says
    /// so as the control does.
    pub fn issue(&mut self, telemetry: &Telemetry, write: Write, messages: &mut VecDeque<Message>) {
        match telemetry.set_parameter_confirmed(&write.param, write.value) {
            Some(id) => self.in_flight.push(InFlight { id, write }),
            None => self.failed(&write, "no vehicle", messages),
        }
    }

    /// A failed write: the control's message box.
    fn failed(&mut self, write: &Write, why: &str, messages: &mut VecDeque<Message>) {
        self.last = Some(format!("{} {} failed: {why}", write.param, write.value));
        messages.push_back(Message {
            title: ERROR_TITLE,
            text: write.failure.clone(),
        });
    }

    /// Reads back how each write ended: `setParam`'s true for an echo, for a value the vehicle
    /// already held and for one not waited on; its false - the message box - for a name the
    /// vehicle has not listed and for a set every retry of which went unanswered.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1638-1770`
    pub fn settle(&mut self, telemetry: &Telemetry, messages: &mut VecDeque<Message>) {
        for flight in std::mem::take(&mut self.in_flight) {
            let Some(request) = telemetry.request(flight.id) else {
                continue;
            };
            match request.outcome() {
                None => self.in_flight.push(flight),
                Some(RequestOutcome::Accepted { value }) => {
                    let echoed = value.map_or(flight.write.value, |value| value.as_f64());
                    self.last = Some(format!("{} {echoed} accepted", flight.write.param));
                }
                Some(RequestOutcome::Unchanged | RequestOutcome::Sent) => {
                    self.last = Some(format!(
                        "{} {} unchanged",
                        flight.write.param, flight.write.value
                    ));
                }
                Some(RequestOutcome::UnknownParameter) => {
                    self.failed(&flight.write, "not on the vehicle", messages);
                }
                Some(RequestOutcome::TimedOut) => {
                    self.failed(&flight.write, "timed out", messages);
                }
                Some(RequestOutcome::Rejected(result)) => {
                    self.failed(&flight.write, &format!("rejected {result}"), messages);
                }
            }
        }
    }

    /// How the last write ended.
    #[must_use]
    pub fn last(&self) -> Option<&str> {
        self.last.as_deref()
    }

    /// How many are on their way.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.in_flight.len()
    }
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// How many rows: sixteen, or thirty-two when the vehicle has `SERVO_32_ENABLE` above zero.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:17-24`
#[must_use]
pub fn servo_count(parameters: &[(String, f64)]) -> usize {
    if value_of(parameters, "SERVO_32_ENABLE").is_some_and(|value| value > 0.0) {
        32
    } else {
        16
    }
}

/// `ch1out` to `ch32out`: `SERVO_OUTPUT_RAW` port 0 for the first sixteen, port 1 for the rest,
/// zero before one has arrived.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3634-3678`
#[must_use]
pub fn servo_out(view: &TelemetryView, servo: usize) -> i64 {
    view.state.as_deref().map_or(0, |state| {
        state
            .servo_outputs
            .get(servo.wrapping_sub(1))
            .map_or(0, |value| i64::from(*value))
    })
}

/// How full a bar is drawn: `HorizontalProgressBar2.Value`'s clamp, one above the minimum at the
/// bottom and the maximum at the top, over the bar's 800 to 2200.
/// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:85-117`
#[must_use]
pub fn bar_fraction(value: i64) -> f32 {
    let shown = if value <= BAR_MINIMUM {
        BAR_MINIMUM + 1
    } else if value >= BAR_MAXIMUM {
        BAR_MAXIMUM
    } else {
        value
    };
    #[allow(clippy::cast_precision_loss)]
    let fraction = (shown - BAR_MINIMUM) as f32 / (BAR_MAXIMUM - BAR_MINIMUM) as f32;
    fraction
}

/// Which of a row's three numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Column {
    /// `min1`, `SERVOn_MIN`.
    Min,
    /// `trim1`, `SERVOn_TRIM`.
    Trim,
    /// `max1`, `SERVOn_MAX`.
    Max,
}

impl Column {
    /// The three, left to right.
    pub const ALL: [Self; 3] = [Self::Min, Self::Trim, Self::Max];

    /// The parameter's suffix.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            Self::Min => "MIN",
            Self::Trim => "TRIM",
            Self::Max => "MAX",
        }
    }

    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Min => "min",
            Self::Trim => "trim",
            Self::Max => "max",
        }
    }

    /// The table column it is in.
    const fn index(self) -> usize {
        match self {
            Self::Min => 4,
            Self::Trim => 5,
            Self::Max => 6,
        }
    }
}

/// One output's row: `setup(servono)`.
#[derive(Debug)]
pub struct ServoRow {
    /// The output's number, from 1.
    pub servo: usize,
    /// `rev1`, `SERVOn_REVERSED`.
    pub reversed: Check,
    /// `func1`, `SERVOn_FUNCTION`.
    pub function: Combo,
    /// `min1`, `SERVOn_MIN`.
    pub min: Number,
    /// `trim1`, `SERVOn_TRIM`.
    pub trim: Number,
    /// `max1`, `SERVOn_MAX`.
    pub max: Number,
}

impl ServoRow {
    /// `setup(servono)`: the controls made disabled, then each set up from its parameter, in the
    /// C#'s order.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:35-69`
    #[must_use]
    pub fn setup(servo: usize, parameters: &[(String, f64)], lookup: Lookup) -> Self {
        Self::setup_from(servo, PWM_BOX, parameters, lookup)
    }

    /// `setup(servono)` over numbers as a Designer left them: this page's constructor makes
    /// them 800 to 2200 at 1500, Heli Setup's Designer leaves them `NumericUpDown`'s own.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:35-69; ConfigTradHeli4.cs:174-185`
    #[must_use]
    pub fn setup_from(
        servo: usize,
        designer: Designer,
        parameters: &[(String, f64)],
        lookup: Lookup,
    ) -> Self {
        let name = format!("SERVO{servo}");
        let mut reversed = Check::default();
        reversed.setup(1.0, 0.0, &format!("{name}_REVERSED"), parameters);
        let function_param = format!("{name}_FUNCTION");
        let mut function = Combo::default();
        function.setup(
            options(&function_param, lookup),
            &function_param,
            parameters,
        );
        let number = |column: Column| {
            let mut number = Number::new(designer);
            number.setup(
                PWM_SETUP,
                &format!("{name}_{}", column.suffix()),
                parameters,
                lookup,
            );
            number
        };
        Self {
            servo,
            reversed,
            function,
            min: number(Column::Min),
            trim: number(Column::Trim),
            max: number(Column::Max),
        }
    }

    /// One of the three numbers.
    #[must_use]
    pub const fn number(&self, column: Column) -> &Number {
        match column {
            Column::Min => &self.min,
            Column::Trim => &self.trim,
            Column::Max => &self.max,
        }
    }

    const fn number_mut(&mut self, column: Column) -> &mut Number {
        match column {
            Column::Min => &mut self.min,
            Column::Trim => &mut self.trim,
            Column::Max => &mut self.max,
        }
    }
}

/// The page object and what it keeps.
#[derive(Debug, Default)]
pub struct ServoOutput {
    /// The rows the constructor made.
    rows: Vec<ServoRow>,
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Between `Activate` and `Deactivate`: the bars' timer is running.
    active: bool,
    /// The row whose function list is dropped down.
    dropdown: Option<usize>,
    /// The number the keyboard is typing into, by row.
    editing: Option<(usize, Column)>,
    /// A number's out-of-range question, waiting for its answer.
    question: Option<((usize, Column), Question)>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The writes on their way.
    writes: Writes,
}

impl ServoOutput {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The rows.
    #[must_use]
    pub fn rows(&self) -> &[ServoRow] {
        &self.rows
    }

    /// The row whose list is down.
    #[must_use]
    pub const fn dropdown(&self) -> Option<usize> {
        self.dropdown
    }

    /// The number being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<(usize, Column)> {
        self.editing
    }

    /// The question waiting, if one is.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
    }

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// How the last write ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.writes.last()
    }

    /// Shows the page: the page object made first if this screen has none - the constructor,
    /// which builds every row - and then `Activate`, which starts the bars' timer.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:11-33, 71-74`
    pub fn activate(
        &mut self,
        telemetry: &Telemetry,
        parameters: &[(String, f64)],
        key: Key,
        lookup: Lookup,
    ) {
        if self.made_for != Some(key) {
            for write in self.dispose() {
                self.issue(telemetry, write);
            }
            self.rows = (1..=servo_count(parameters))
                .map(|servo| ServoRow::setup(servo, parameters, lookup))
                .collect();
            self.made_for = Some(key);
        }
        self.active = true;
    }

    /// `Deactivate`, which stops the timer, and the page hidden, which takes the focus from a
    /// number being typed into - so its text is read.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioOutput.cs:76-79`
    pub fn deactivate(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = None;
        self.leave(now);
    }

    /// The page object disposed with its screen. What the numbers' timers still held is
    /// returned to be written: those timers are not the page's and outlive it.
    pub fn dispose(&mut self) -> Vec<Write> {
        let pending = self
            .rows
            .iter_mut()
            .flat_map(|row| [&mut row.min, &mut row.trim, &mut row.max])
            .filter_map(Number::flush)
            .collect();
        self.rows.clear();
        self.made_for = None;
        self.active = false;
        self.dropdown = None;
        self.editing = None;
        self.question = None;
        pending
    }

    /// Drops a row's function list down, or back up; the number being typed into loses the focus.
    pub fn toggle_dropdown(&mut self, row: usize, now: Instant) {
        self.leave(now);
        let enabled = self
            .rows
            .get(row)
            .is_some_and(|servo| servo.function.enabled);
        self.dropdown = if self.dropdown == Some(row) || !enabled {
            None
        } else {
            if let Some(servo) = self.rows.get_mut(row) {
                servo.function.open_list();
            }
            Some(row)
        };
    }

    /// The wheel over a row's dropped-down list.
    pub fn scroll_list(&mut self, row: usize, lines: i32) {
        if self.dropdown == Some(row)
            && let Some(servo) = self.rows.get_mut(row)
        {
            servo.function.scroll_list(lines);
        }
    }

    /// Chooses a function from a row's list, closing it.
    pub fn choose(&mut self, row: usize, key: i64) -> Option<Write> {
        self.dropdown = None;
        self.rows.get_mut(row)?.function.choose(key)
    }

    /// Clicks a row's Reverse box.
    pub fn click_reversed(&mut self, row: usize, now: Instant) -> Option<Write> {
        self.leave(now);
        self.dropdown = None;
        self.rows.get_mut(row)?.reversed.click()
    }

    /// A number clicked into: the one being typed into before it loses the focus.
    pub fn begin(&mut self, row: usize, column: Column, now: Instant) {
        if self.editing == Some((row, column)) {
            return;
        }
        self.leave(now);
        self.dropdown = None;
        if self
            .rows
            .get(row)
            .is_some_and(|servo| servo.number(column).enabled)
        {
            self.editing = Some((row, column));
        }
    }

    /// The number being typed into loses the focus, which reads its text.
    pub fn leave(&mut self, now: Instant) {
        let Some(at @ (row, column)) = self.editing.take() else {
            return;
        };
        let question = self
            .rows
            .get_mut(row)
            .and_then(|servo| servo.number_mut(column).commit(now));
        if let Some(question) = question {
            self.question = Some((at, question));
        }
    }

    /// A key for the number being typed into.
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(at @ (row, column)) = self.editing else {
            return false;
        };
        let Some(servo) = self.rows.get_mut(row) else {
            return false;
        };
        let (handled, question) = servo.number_mut(column).key(event, now);
        if let Some(question) = question {
            self.question = Some((at, question));
        }
        handled
    }

    /// A number's up or down arrow.
    pub fn step(&mut self, row: usize, column: Column, up: bool, now: Instant) {
        self.begin(row, column, now);
        let question = self
            .rows
            .get_mut(row)
            .and_then(|servo| servo.number_mut(column).step(up, now));
        if let Some(question) = question {
            self.question = Some(((row, column), question));
        }
    }

    /// The out-of-range question answered.
    pub fn answer(&mut self, yes: bool, now: Instant) {
        let Some(((row, column), question)) = self.question.take() else {
            return;
        };
        if let Some(servo) = self.rows.get_mut(row) {
            servo.number_mut(column).answer(&question, yes, now);
        }
    }

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// The numbers whose timers have run out.
    pub fn due(&mut self, now: Instant) -> Vec<Write> {
        self.rows
            .iter_mut()
            .flat_map(|row| [&mut row.min, &mut row.trim, &mut row.max])
            .filter_map(|number| number.due(now))
            .collect()
    }

    /// Sends a write.
    pub fn issue(&mut self, telemetry: &Telemetry, write: Write) {
        self.writes.issue(telemetry, write, &mut self.messages);
    }

    /// Once a frame: a page object whose screen has gone is disposed, a number that lost the
    /// focus has its text read, the timers that ran out write, and the writes on their way are
    /// read back.
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
            for write in self.dispose() {
                self.issue(telemetry, write);
            }
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        for write in self.due(now) {
            self.issue(telemetry, write);
        }
        self.writes.settle(telemetry, &mut self.messages);
    }
}

/// Facts a UI test asserts on: what each row shows, the bars, and what the vehicle holds.
pub fn record_facts(servo: &ServoOutput, view: &TelemetryView) {
    use crate::facts::record;
    record("config.servo.active", servo.is_active());
    record("config.servo.rows", servo.rows().len());
    record(
        "config.servo.message",
        servo
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.servo.question",
        servo
            .question()
            .map_or_else(|| "none".to_owned(), Question::text),
    );
    record(
        "config.servo.list.top",
        servo
            .dropdown()
            .and_then(|index| servo.rows().get(index))
            .map_or_else(
                || "none".to_owned(),
                |row| row.function.top_index.to_string(),
            ),
    );
    record("config.servo.write", servo.last_write().unwrap_or("none"));
    record("config.servo.writes.pending", servo.writes.pending());
    record(
        "config.servo.editing",
        servo
            .editing()
            .and_then(|(row, column)| {
                servo
                    .rows()
                    .get(row)
                    .map(|servo| servo.number(column).param.as_str())
            })
            .unwrap_or("none"),
    );
    for row in servo.rows() {
        let n = row.servo;
        record(
            format!("config.servo.{n}.function"),
            row.function
                .selected
                .map_or_else(|| "none".to_owned(), |key| key.to_string()),
        );
        record(
            format!("config.servo.{n}.function.text"),
            row.function.text(),
        );
        record(
            format!("config.servo.{n}.function.enabled"),
            row.function.enabled,
        );
        record(
            format!("config.servo.{n}.reversed"),
            row.reversed.state.key(),
        );
        for column in Column::ALL {
            record(
                format!("config.servo.{n}.{}", column.key()),
                row.number(column).shown(),
            );
        }
        record(format!("config.servo.{n}.position"), servo_out(view, n));
    }
    for (name, value) in view.parameters.iter() {
        if name.starts_with("SERVO") {
            record(format!("params.value.{name}"), value);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing the controls.
// ---------------------------------------------------------------------------------------------

/// An absolutely placed box.
fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A label at its place.
pub fn label(x: f32, y: f32, text: impl Into<SharedString>) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(text.into())
}

/// A combo box: its selected row's text and the arrow, a click dropping its list down; dimmed and
/// inert while disabled.
pub fn combo_box(
    id: String,
    combo: &Combo,
    (x, y, width, height): (f32, f32, f32, f32),
    on_open: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .child(
            div()
                .flex_1()
                .overflow_hidden()
                .whitespace_nowrap()
                .child(combo.text().to_owned()),
        )
        .child(div().text_size(px(7.0)).child("▼"));
    let base = if combo.enabled {
        base.bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                on_open(this);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL)).text_color(rgb(theme::DIM))
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// A combo box's list, dropped down over what is under it; each row `<id>-<value>`.
///
/// Drawn deferred and anchored at the combo box, so it lies over the page rather than inside
/// it: a page is a scrolling area that clips what runs past its edge, and a list is a popup that
/// does not (`ComboBox`'s drop-down is its own window). It shows [`LIST_ROWS_SHOWN`] rows from
/// `combo.top_index` and the wheel moves that, which is how a row past the thirtieth - the
/// documentation lists a `SERVOn_FUNCTION`'s 128 values alphabetically - gets chosen.
pub fn dropdown(
    id: &str,
    combo: &Combo,
    (x, y, width): (f32, f32, f32),
    on_choose: impl Fn(&mut MissionPlanner, i64) + 'static,
    on_scroll: impl Fn(&mut MissionPlanner, i32) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let on_choose = Rc::new(on_choose);
    let mut list = crate::probe::measured(format!("{id}-list"), div())
        .id(SharedString::from(format!("{id}-list")))
        .min_w(px(width))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .occlude()
        .on_scroll_wheel(
            cx.listener(move |this, event: &gpui::ScrollWheelEvent, _window, cx| {
                // gpui reports lines or pixels by backend; taken as lines of LIST_ROW either
                // way, and a negative y is the wheel rolled towards the user - down.
                let delta = event.delta.pixel_delta(px(LIST_ROW));
                let lines = (f32::from(delta.y) / LIST_ROW).round();
                #[allow(clippy::cast_possible_truncation)]
                let lines = -(lines as i32);
                if lines != 0 {
                    on_scroll(this, lines);
                    cx.notify();
                }
                cx.stop_propagation();
            }),
        );
    for (_, (key, text)) in combo.showing() {
        let key = *key;
        let name = format!("{id}-{key}");
        let on_choose = Rc::clone(&on_choose);
        list = list.child(
            crate::probe::measured(name.clone(), div())
                .id(SharedString::from(name))
                .flex_shrink_0()
                .h(px(LIST_ROW))
                .px_1()
                .text_xs()
                .whitespace_nowrap()
                .bg(rgb(if combo.selected == Some(key) {
                    theme::BORDER
                } else {
                    theme::PANEL
                }))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::ACTION)))
                .child(text.clone())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    on_choose(this, key);
                    cx.notify();
                })),
        );
    }
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .child(gpui::deferred(gpui::anchored().snap_to_window().child(list)).with_priority(1))
        .into_any_element()
}

/// A check box and its text; dimmed and inert while disabled.
pub fn check_box(
    id: String,
    check: &Check,
    text: &'static str,
    (x, y): (f32, f32),
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let colour = if check.enabled {
        theme::ACCENT
    } else {
        theme::DIM
    };
    let mark = match check.state {
        CheckState::Unchecked => None,
        CheckState::Checked => Some(div().size(px(6.0)).bg(rgb(colour))),
        CheckState::Indeterminate => Some(div().w(px(6.0)).h(px(2.0)).bg(rgb(colour))),
    };
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .flex()
        .items_center()
        .gap_1()
        .child(
            div()
                .size(px(13.0))
                .flex()
                .items_center()
                .justify_center()
                .border_1()
                .border_color(rgb(theme::DIM))
                .bg(rgb(if check.enabled {
                    theme::BG
                } else {
                    theme::PANEL
                }))
                .children(mark),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(if check.enabled {
                    theme::TEXT
                } else {
                    theme::DIM
                }))
                .child(text),
        );
    let base = if check.enabled {
        base.cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                on_click(this);
                cx.notify();
            }))
    } else {
        base
    };
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .child(base)
        .into_any_element()
}

/// What a number box does when it is used.
pub struct NumberHandlers<B, K, S> {
    /// Clicked into.
    pub begin: B,
    /// A key while it is being typed into.
    pub key: K,
    /// An arrow: up when true.
    pub step: S,
}

/// A number at its place: its text - typed into, with a caret, while it has the focus - and the
/// up and down arrows at its right, `<id>-up` and `<id>-down`; dimmed and inert while disabled.
#[allow(clippy::too_many_arguments)]
pub fn number_box<B, K, S>(
    id: String,
    number: &Number,
    editing: bool,
    handle: &FocusHandle,
    (x, y, width, height): (f32, f32, f32, f32),
    handlers: NumberHandlers<B, K, S>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement
where
    B: Fn(&mut MissionPlanner) + 'static,
    K: Fn(&mut MissionPlanner, &KeyDownEvent) -> bool + 'static,
    S: Fn(&mut MissionPlanner, bool) + 'static,
{
    let enabled = number.enabled;
    let focused = enabled && editing && handle.is_focused(window);
    let text = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id.clone()))
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        .child(
            div()
                .flex_1()
                .whitespace_nowrap()
                .child(number.shown().to_owned()),
        )
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text = if !enabled {
        text.text_color(rgb(theme::DIM))
    } else if editing {
        let key = handlers.key;
        text.track_focus(handle)
            .key_context("TextField")
            .cursor_text()
            .text_color(rgb(theme::TEXT))
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if key(this, event) {
                    cx.notify();
                }
            }))
    } else {
        let begin = handlers.begin;
        let handle = handle.clone();
        text.cursor_text()
            .text_color(rgb(theme::TEXT))
            .on_click(cx.listener(move |this, _event, window, cx| {
                begin(this);
                handle.focus(window, cx);
                cx.notify();
            }))
    };
    let step = Rc::new(handlers.step);
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let name = format!("{id}-{}", if up { "up" } else { "down" });
        let base = crate::probe::measured(name.clone(), div())
            .id(SharedString::from(name))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .child(if up { "▲" } else { "▼" });
        if enabled {
            let step = Rc::clone(&step);
            base.text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    step(this, up);
                    cx.notify();
                }))
                .into_any_element()
        } else {
            base.text_color(rgb(theme::DIM)).into_any_element()
        }
    };
    let arrows = div()
        .w(px(12.0))
        .h_full()
        .flex()
        .flex_col()
        .border_l_1()
        .border_color(rgb(theme::BORDER))
        .child(arrow(true, cx))
        .child(arrow(false, cx));
    at(x, y, width, height)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }))
        .child(text)
        .child(arrows)
        .into_any_element()
}

/// A modal box drawn over the whole window, as `CustomMessageBox.Show` is: its caption, its
/// text and its buttons.
pub fn modal(
    id: &'static str,
    title: &str,
    text: &str,
    alert: bool,
    buttons: Vec<AnyElement>,
    window: &Window,
) -> AnyElement {
    let size = window.viewport_size();
    let dialog = crate::probe::measured(id, div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(360.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(if alert { theme::ALERT } else { theme::WARN }))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(title.to_owned()),
        )
        // Each line and the button row are held to the box's width: a flex item's minimum
        // width is its content's, and a long line let the row grow past the border, which put
        // Raw Param Warning's OK 140 px outside the box (the owner, 2026-09-25).
        .children(text.lines().map(|line| {
            div()
                .w_full()
                .min_w_0()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(line.to_owned())
        }))
        .child(
            div()
                .w_full()
                .min_w_0()
                .flex()
                .gap_2()
                .justify_end()
                .children(buttons),
        );
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

// ---------------------------------------------------------------------------------------------
// Drawing the page.
// ---------------------------------------------------------------------------------------------

/// Where a column starts.
fn column_x(index: usize) -> f32 {
    TABLE_AT.0 + COLUMNS.iter().take(index).sum::<f32>()
}

/// Where a row starts; row 0 is the first servo.
#[allow(clippy::cast_precision_loss)] // at most 32 rows
fn row_y(index: usize) -> f32 {
    TABLE_AT.1 + HEADER_HEIGHT + ROW_HEIGHT * index as f32
}

/// One `HorizontalProgressBar2`: the bar, with its value drawn across its middle.
/// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:169-187`
fn bar(x: f32, y: f32, value: i64) -> AnyElement {
    at(x, y, 100.0, 23.0)
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .child(
            div()
                .absolute()
                .left_0()
                .top_0()
                .h_full()
                .w(gpui::relative(bar_fraction(value)))
                .bg(rgb(theme::OK)),
        )
        .child(
            div()
                .absolute()
                .left_0()
                .top_0()
                .size_full()
                .flex()
                .items_center()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(value.to_string()),
        )
        .into_any_element()
}

/// The page, laid out as `ConfigRadioOutput`'s table is.
pub fn page(
    servo: &ServoOutput,
    handle: &FocusHandle,
    view: &TelemetryView,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !servo.is_active() {
        return None;
    }
    let rows = servo.rows();
    let height = row_y(rows.len()) + 3.0;
    let width = column_x(COLUMNS.len()) + 3.0;
    let mut body = div().relative().w(px(width)).h(px(height));
    for (index, header) in HEADERS.iter().enumerate() {
        body = body.child(label(column_x(index) + 3.0, TABLE_AT.1 + 3.0, *header));
    }
    for (index, row) in rows.iter().enumerate() {
        let y = row_y(index) + 3.0;
        let n = row.servo;
        body = body
            .child(label(column_x(0) + 3.0, y + 5.0, n.to_string()))
            .child(bar(column_x(1) + 3.0, y, servo_out(view, n)))
            .child(check_box(
                format!("servo-{}", row.reversed.param),
                &row.reversed,
                "",
                (column_x(2) + 3.0, y + 5.0),
                move |this| {
                    let now = Instant::now();
                    if let Some(write) = this.servo_output.click_reversed(index, now) {
                        this.servo_output.issue(&this.telemetry, write);
                    }
                },
                cx,
            ))
            .child(combo_box(
                format!("servo-{}", row.function.param),
                &row.function,
                (column_x(3) + 3.0, y + 1.0, 160.0, 21.0),
                move |this| this.servo_output.toggle_dropdown(index, Instant::now()),
                cx,
            ));
        for column in Column::ALL {
            let number = row.number(column);
            body = body.child(number_box(
                format!("servo-{}", number.param),
                number,
                servo.editing() == Some((index, column)),
                handle,
                (column_x(column.index()) + 3.0, y + 1.0, 50.0, 20.0),
                NumberHandlers {
                    begin: move |this: &mut MissionPlanner| {
                        this.servo_output.begin(index, column, Instant::now());
                    },
                    key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                        this.servo_output.key(event, Instant::now())
                    },
                    step: move |this: &mut MissionPlanner, up: bool| {
                        this.servo_output.step(index, column, up, Instant::now());
                    },
                },
                window,
                cx,
            ));
        }
    }
    // A dropped-down list goes last, so it draws over the rows below it.
    if let Some(index) = servo.dropdown()
        && let Some(row) = rows.get(index)
    {
        body = body.child(dropdown(
            &format!("servo-{}", row.function.param),
            &row.function,
            (column_x(3) + 3.0, row_y(index) + 4.0 + 21.0, 160.0),
            move |this, key| {
                if let Some(write) = this.servo_output.choose(index, key) {
                    this.servo_output.issue(&this.telemetry, write);
                }
            },
            move |this, lines| this.servo_output.scroll_list(index, lines),
            cx,
        ));
    }
    Some(panel(TITLE, body).into_any_element())
}

/// The question or message box showing, drawn over the whole window.
pub fn overlay(
    servo: &ServoOutput,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = servo.question() {
        let buttons = vec![
            action(
                "servo-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.servo_output.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "servo-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.servo_output.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "servo-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = servo.message()?;
    let ok = action(
        "servo-message-ok",
        "OK",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.servo_output.dismiss_message();
            cx.notify();
        }),
    );
    Some(modal(
        "servo-message",
        message.title,
        &message.text,
        message.title == ERROR_TITLE,
        vec![ok],
        window,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{Vehicle, param, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;
    use mp_params::ParamMeta;

    /// The bundled documentation, so no test depends on what another has installed.
    fn bundled(name: &str) -> Option<&'static ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// The whole of a real SITL copter's parameters, as `headless-planner param save` wrote them.
    fn sitl() -> Vec<(String, f64)> {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let file = mp_params::param_file::ParamFile::load(&fixture).expect("the SITL dump");
        file.iter()
            .map(|(name, value)| (name.to_owned(), value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    #[test]
    fn sixteen_rows_or_thirty_two_with_servo_32_enable() {
        assert_eq!(servo_count(&table(&[])), 16);
        assert_eq!(servo_count(&table(&[("SERVO_32_ENABLE", 0.0)])), 16);
        assert_eq!(servo_count(&table(&[("SERVO_32_ENABLE", 1.0)])), 32);
        let telemetry = Telemetry::idle();
        let mut page = ServoOutput::default();
        page.activate(
            &telemetry,
            &table(&[("SERVO_32_ENABLE", 1.0)]),
            key(),
            bundled,
        );
        assert_eq!(page.rows().len(), 32);
        assert_eq!(page.rows().last().map(|row| row.servo), Some(32));
    }

    /// The SITL copter's outputs: four motors on 1 to 4, nothing on the rest, every output 1100
    /// to 1900 about 1500, none reversed.
    #[test]
    fn the_sitl_copter_binds_every_row() {
        let telemetry = Telemetry::idle();
        let mut page = ServoOutput::default();
        page.activate(&telemetry, &sitl(), key(), bundled);
        assert!(page.is_active());
        assert_eq!(page.rows().len(), 16);
        let first = &page.rows()[0];
        assert_eq!(first.function.param, "SERVO1_FUNCTION");
        assert!(first.function.enabled);
        assert_eq!(first.function.selected, Some(33));
        assert_eq!(first.function.text(), "Motor1");
        assert_eq!(first.reversed.param, "SERVO1_REVERSED");
        assert!(first.reversed.enabled);
        assert_eq!(first.reversed.state, CheckState::Unchecked);
        assert_eq!(first.min.shown(), "1100");
        assert_eq!(first.trim.shown(), "1500");
        assert_eq!(first.max.shown(), "1900");
        assert!(first.min.enabled && first.trim.enabled && first.max.enabled);
        assert_eq!(page.rows()[3].function.text(), "Motor4");
        assert_eq!(page.rows()[8].function.selected, Some(0));
        assert_eq!(page.rows()[8].function.text(), "Disabled");
        // The documented ranges: MIN 500 to 2200, TRIM and MAX 800 to 2200, in steps of 1.
        assert!((first.min.minimum - 500.0).abs() < f64::EPSILON);
        assert!((first.trim.minimum - 800.0).abs() < f64::EPSILON);
        assert!((first.max.maximum - 2200.0).abs() < f64::EPSILON);
        assert!((first.min.increment() - 1.0).abs() < f64::EPSILON);
        assert_eq!(first.min.decimals, 0);
    }

    /// Without `SERVOn_*`, as on firmware that had only `RCn_*`, every row is still built and
    /// every control stays disabled: the C# has no fallback.
    #[test]
    fn a_vehicle_without_servo_parameters_gets_disabled_rows() {
        let telemetry = Telemetry::idle();
        let mut page = ServoOutput::default();
        page.activate(
            &telemetry,
            &table(&[("RC1_MIN", 1100.0), ("RC1_FUNCTION", 33.0)]),
            key(),
            bundled,
        );
        assert_eq!(page.rows().len(), 16);
        for row in page.rows() {
            assert!(!row.reversed.enabled);
            assert!(!row.function.enabled);
            assert!(!row.min.enabled && !row.trim.enabled && !row.max.enabled);
            // The constructor's 1500, and the list's first row, bound but not chosen.
            assert_eq!(row.trim.shown(), "1500");
            assert_eq!(row.function.selected, Some(-1));
        }
        assert!(
            page.choose(0, 1).is_none(),
            "a disabled combo box writes nothing"
        );
        assert!(page.click_reversed(0, Instant::now()).is_none());
    }

    #[test]
    fn a_function_the_documentation_does_not_list_selects_nothing() {
        let mut combo = Combo::default();
        combo.setup(
            options("SERVO1_FUNCTION", bundled),
            "SERVO1_FUNCTION",
            &table(&[("SERVO1_FUNCTION", 9999.0)]),
        );
        assert!(combo.enabled);
        assert_eq!(combo.selected, None);
        assert_eq!(combo.text(), "");
    }

    #[test]
    fn choosing_a_function_writes_it_once() {
        let telemetry = Telemetry::idle();
        let mut page = ServoOutput::default();
        page.activate(&telemetry, &sitl(), key(), bundled);
        page.toggle_dropdown(8, Instant::now());
        assert_eq!(page.dropdown(), Some(8));
        let write = page.choose(8, 1).expect("a change");
        assert_eq!(page.dropdown(), None);
        assert_eq!(write.param, "SERVO9_FUNCTION");
        assert!((write.value - 1.0).abs() < f64::EPSILON);
        assert_eq!(write.failure, "Set SERVO9_FUNCTION Failed!");
        assert_eq!(page.rows()[8].function.text(), "RCPassThru");
        assert!(
            page.choose(8, 1).is_none(),
            "the same row again changes nothing"
        );
    }

    #[test]
    fn the_reverse_box_writes_one_and_zero() {
        let telemetry = Telemetry::idle();
        let mut page = ServoOutput::default();
        page.activate(&telemetry, &sitl(), key(), bundled);
        let write = page.click_reversed(8, Instant::now()).expect("a write");
        assert_eq!(write.param, "SERVO9_REVERSED");
        assert!((write.value - 1.0).abs() < f64::EPSILON);
        assert_eq!(write.failure, "Set SERVO9_REVERSED Failed");
        assert_eq!(page.rows()[8].reversed.state, CheckState::Checked);
        let write = page.click_reversed(8, Instant::now()).expect("a write");
        assert!(write.value.abs() < f64::EPSILON);
        // A value that is neither 1 nor 0 is indeterminate, and a click turns it off.
        let mut check = Check::default();
        check.setup(
            1.0,
            0.0,
            "SERVO1_REVERSED",
            &table(&[("SERVO1_REVERSED", 2.0)]),
        );
        assert_eq!(check.state, CheckState::Indeterminate);
        assert!(
            check
                .click()
                .is_some_and(|write| write.value.abs() < f64::EPSILON)
        );
    }

    /// The list shows thirty rows from `TopIndex`: opened on the selected row, moved a row a
    /// line by the wheel (three lines a notch), never past either end. A `SERVOn_FUNCTION` list runs to 128 values
    /// and the documentation orders them alphabetically, so the row wanted is usually past the
    /// thirtieth.
    #[test]
    fn the_list_opens_on_the_selected_row_and_the_wheel_moves_it_a_row_a_line() {
        let options: Vec<(i64, String)> = (0..128).map(|i| (i, format!("f{i}"))).collect();
        let mut combo = Combo::default();
        combo.setup(
            options,
            "SERVO9_FUNCTION",
            &[("SERVO9_FUNCTION".to_owned(), 96.0)],
        );
        combo.open_list();
        assert_eq!(combo.top_index, 96 + 1 - LIST_ROWS_SHOWN);
        let showing: Vec<usize> = combo.showing().map(|(index, _)| index).collect();
        assert_eq!(showing.len(), LIST_ROWS_SHOWN);
        assert_eq!(showing.last(), Some(&96));
        combo.scroll_list(-1);
        assert_eq!(combo.top_index, 96 + 1 - LIST_ROWS_SHOWN - WHEEL_ROWS);
        combo.scroll_list(100);
        assert_eq!(combo.top_index, 128 - LIST_ROWS_SHOWN, "clamped at the end");
        combo.scroll_list(-100);
        assert_eq!(combo.top_index, 0, "clamped at the start");
        // A selected row within the first thirty opens from the top; a short list never moves.
        combo.select(1);
        combo.open_list();
        assert_eq!(combo.top_index, 0);
        let mut short = Combo::default();
        short.setup(
            vec![(0, "a".to_owned()), (1, "b".to_owned())],
            "MOT_PWM_TYPE",
            &[("MOT_PWM_TYPE".to_owned(), 1.0)],
        );
        short.open_list();
        short.scroll_list(5);
        assert_eq!(short.top_index, 0);
        assert_eq!(short.showing().count(), 2);
    }

    /// Opening a row's list scrolls it to the row's function; the page routes the wheel to the
    /// open list and to no other.
    #[test]
    fn the_page_opens_the_list_on_the_function_and_scrolls_only_the_open_one() {
        // RCIN13 (63) is the 41st of the bundled table's 92 SERVO9_FUNCTION values.
        let telemetry = Telemetry::idle();
        let mut servo = ServoOutput::default();
        let mut table = sitl();
        for (name, value) in &mut table {
            if name == "SERVO9_FUNCTION" {
                *value = 63.0;
            }
        }
        servo.activate(&telemetry, &table, key(), bundled);
        let index = servo.rows()[8]
            .function
            .options
            .iter()
            .position(|(key, _)| *key == 63)
            .expect("RCIN13 in the bundled list");
        assert!(
            index >= LIST_ROWS_SHOWN,
            "the test needs a row past the thirtieth: {index}"
        );
        servo.toggle_dropdown(8, Instant::now());
        assert_eq!(servo.dropdown(), Some(8));
        assert_eq!(
            servo.rows()[8].function.top_index,
            index + 1 - LIST_ROWS_SHOWN
        );
        servo.scroll_list(7, 1);
        assert_eq!(servo.rows()[7].function.top_index, 0, "not the open list");
        servo.scroll_list(8, 1);
        assert_eq!(
            servo.rows()[8].function.top_index,
            index + 1 - LIST_ROWS_SHOWN + WHEEL_ROWS
        );
    }

    #[test]
    fn a_typed_number_is_read_on_enter_and_written_300_ms_after() {
        let telemetry = Telemetry::idle();
        let mut page = ServoOutput::default();
        page.activate(&telemetry, &sitl(), key(), bundled);
        let start = Instant::now();
        page.begin(8, Column::Trim, start);
        assert_eq!(page.editing(), Some((8, Column::Trim)));
        let row = &mut page.rows[8];
        row.trim.type_text("1510");
        assert!(row.trim.commit(start).is_none());
        assert_eq!(row.trim.shown(), "1510");
        assert!(page.due(start).is_empty(), "the timer has not run");
        let writes = page.due(start + WRITE_DELAY);
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].param, "SERVO9_TRIM");
        assert!((writes[0].value - 1510.0).abs() < f64::EPSILON);
        assert_eq!(writes[0].failure, "Set SERVO9_TRIM Failed");
        assert!(page.due(start + WRITE_DELAY * 2).is_empty(), "written once");
    }

    #[test]
    fn a_running_timer_is_not_restarted_and_writes_the_latest_value() {
        let mut number = Number::new(PWM_BOX);
        number.setup(PWM_SETUP, "SERVO9_TRIM", &sitl(), bundled);
        let start = Instant::now();
        number.step(true, start);
        number.step(true, start + Duration::from_millis(200));
        assert_eq!(number.shown(), "1502");
        // `if (!timer.Enabled) timer.Start()`: the first change's 300 ms, not the second's.
        let write = number.due(start + WRITE_DELAY).expect("due");
        assert!((write.value - 1502.0).abs() < f64::EPSILON);
    }

    #[test]
    fn a_typed_value_is_held_to_the_range_and_one_above_the_maximum_asks() {
        let mut number = Number::new(PWM_BOX);
        number.setup(PWM_SETUP, "SERVO9_MAX", &sitl(), bundled);
        let now = Instant::now();
        // Below the minimum: held to 800, no question.
        number.type_text("100");
        assert!(number.commit(now).is_none());
        assert_eq!(number.shown(), "800");
        assert!(number.pending());
        // Above the maximum: held to 2200, and asked.
        let mut number = Number::new(PWM_BOX);
        number.setup(PWM_SETUP, "SERVO9_MAX", &sitl(), bundled);
        number.type_text("2500");
        let question = number.commit(now).expect("a question");
        assert_eq!(
            question.text(),
            "SERVO9_MAX Value out of range\nDo you want to accept the new value?"
        );
        assert_eq!(number.shown(), "2200");
        assert!(!number.pending(), "the timer waits for the answer");
        number.answer(&question, true, now);
        assert_eq!(number.shown(), "2500");
        assert!((number.maximum - 2500.0).abs() < f64::EPSILON);
        assert!(number.pending());
        // No keeps the held value, and still writes it.
        let mut number = Number::new(PWM_BOX);
        number.setup(PWM_SETUP, "SERVO9_MAX", &sitl(), bundled);
        number.type_text("2500");
        let question = number.commit(now).expect("a question");
        number.answer(&question, false, now);
        assert_eq!(number.shown(), "2200");
        assert!(number.pending());
    }

    #[test]
    fn text_that_does_not_parse_or_does_not_change_the_value_writes_nothing() {
        let mut number = Number::new(PWM_BOX);
        number.setup(PWM_SETUP, "SERVO9_TRIM", &sitl(), bundled);
        let now = Instant::now();
        for text in ["", "-", "abc", "1500"] {
            number.type_text(text);
            assert!(number.commit(now).is_none());
            assert_eq!(number.shown(), "1500", "{text:?}");
            assert!(!number.pending(), "{text:?}");
        }
    }

    #[test]
    fn the_arrows_step_by_the_increment_and_stop_at_the_bounds() {
        let mut number = Number::new(PWM_BOX);
        number.setup(
            PWM_SETUP,
            "SERVO9_MAX",
            &table(&[("SERVO9_MAX", 2199.5)]),
            bundled,
        );
        // 2199.5 has a place the increment does not: the box shows it, and steps to the top.
        assert_eq!(number.decimals, 1);
        let now = Instant::now();
        number.step(true, now);
        assert_eq!(number.shown(), "2200.0");
        number.step(true, now);
        assert_eq!(number.shown(), "2200.0");
        number.step(false, now);
        assert_eq!(number.shown(), "2199.0");
    }

    #[test]
    fn a_disabled_number_does_not_step_or_type() {
        let mut number = Number::new(PWM_BOX);
        number.setup(PWM_SETUP, "SERVO9_TRIM", &table(&[]), bundled);
        assert!(!number.enabled);
        assert_eq!(number.shown(), "1500");
        number.step(true, Instant::now());
        assert_eq!(number.shown(), "1500");
        assert!(!number.pending());
    }

    #[test]
    fn a_number_being_typed_into_is_read_when_another_control_is_used() {
        let telemetry = Telemetry::idle();
        let mut page = ServoOutput::default();
        page.activate(&telemetry, &sitl(), key(), bundled);
        let now = Instant::now();
        page.begin(8, Column::Min, now);
        page.rows[8].min.type_text("1000");
        page.toggle_dropdown(8, now);
        assert_eq!(page.editing(), None);
        assert_eq!(page.rows()[8].min.shown(), "1000");
        assert!(page.rows()[8].min.pending());
    }

    #[test]
    fn leaving_the_screen_disposes_the_page_and_its_timers_still_write() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut page = ServoOutput::default();
        page.activate(&telemetry, &sitl(), Key::of(&view), bundled);
        let now = Instant::now();
        page.step(0, Column::Trim, true, now);
        page.deactivate(now);
        // Still on the screen: the page object and what its controls hold are kept.
        page.tick(&telemetry, &view, true, false, now);
        assert_eq!(page.rows().len(), 16);
        page.activate(&telemetry, &sitl(), Key::of(&view), bundled);
        assert_eq!(page.rows()[0].trim.shown(), "1501", "kept from before");
        page.deactivate(now);
        // Off the screen: disposed, and the pending write went out - with no vehicle, failed.
        page.tick(&telemetry, &view, false, false, now);
        assert!(page.rows().is_empty());
        assert_eq!(
            page.message().map(|message| message.text.as_str()),
            Some("Set SERVO1_TRIM Failed")
        );
    }

    #[test]
    fn a_bar_is_held_one_above_its_minimum_and_at_its_maximum() {
        assert!((bar_fraction(0) - 1.0 / 1400.0).abs() < 1e-6);
        assert!((bar_fraction(1500) - 0.5).abs() < 1e-6);
        assert!((bar_fraction(2200) - 1.0).abs() < 1e-6);
        assert!((bar_fraction(65535) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn decimal_text_rounds_half_away_from_zero() {
        assert_eq!(decimal_text(0.125, 2), "0.13");
        assert_eq!(decimal_text(1500.0, 0), "1500");
        assert_eq!(decimal_text(0.1, 2), "0.10");
        assert_eq!(decimal_text(-0.0, 1), "0.0");
        assert_eq!(decimal_text(2.5, 0), "3");
    }

    /// A change goes through the link's retrying set, and the vehicle's echo is what the page
    /// reports.
    #[test]
    fn a_write_reaches_the_vehicle_and_its_echo_is_reported() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("SERVO9_FUNCTION", 0.0, 4));
        until("the parameter to be held", || {
            telemetry.holds_parameter("SERVO9_FUNCTION")
        });
        let mut page = ServoOutput::default();
        let view = telemetry.view();
        page.activate(&telemetry, &view.parameters, Key::of(&view), bundled);
        let write = page.choose(8, 1).expect("a change");
        page.issue(&telemetry, write);
        let mut written = None;
        until("the PARAM_SET", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    written = Some(set.param_value);
                    vehicle.send(&param("SERVO9_FUNCTION", set.param_value, 4));
                }
            }
            written.is_some()
        });
        assert_eq!(written, Some(1.0));
        until("the echo", || {
            page.tick(&telemetry, &telemetry.view(), true, false, Instant::now());
            page.last_write() == Some("SERVO9_FUNCTION 1 accepted")
        });
        assert!(page.message().is_none());
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-servo.gui");
        let source = include_str!("servo_output.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.servo.") => {
                    let generic = key
                        .split('.')
                        .map(|part| {
                            if part.chars().all(|c| c.is_ascii_digit()) {
                                "{n}"
                            } else {
                                part
                            }
                        })
                        .collect::<Vec<_>>()
                        .join(".");
                    let recorded = source.contains(&format!("\"{key}\""))
                        || source.contains(&format!("\"{generic}\""))
                        || (generic.starts_with("config.servo.{n}.")
                            && ["min", "trim", "max"]
                                .iter()
                                .any(|column| generic.ends_with(column)));
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("servo-") => {
                    let drawn =
                        id.starts_with("servo-SERVO") || source.contains(&format!("\"{id}\""));
                    assert!(drawn, "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 15 && clicks > 5, "{facts} facts, {clicks} clicks");
    }
}
