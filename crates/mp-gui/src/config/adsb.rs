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

//! ADSB: `GCSViews/ConfigurationView/ConfigADSB.cs`, the last entry under Mandatory Hardware in
//! Initial Setup's list (`GCSViews/InitialSetup.cs:262-263`, whose `mand` puts it there), listed
//! once every parameter is in.
//!
//! What it shows: Write Params, Refresh Params and Find along the top; a panel for the flight
//! identification and aircraft registration, disabled - the code that would fill and save them is
//! commented out in the C# (`ConfigADSB.cs:260-304, 673-709`); and below, one control per `ADSB_`
//! and `AVD_` parameter the vehicle holds and documents with a display name, in the order of their
//! names, favourites (`fav_adsb`) first (`:349-419`). Each is built from the documentation as the
//! C# builds it (`:421-620`): a range with an increment is a `RangeControl` - a number and a track
//! bar - a bitmask is a `MavlinkCheckBoxBitMask`, a list of values is a `ValuesControl`, and
//! anything else gets no control.
//!
//! Changing a control writes nothing: it records the value (`Control_ValueChanged`, `:622-625`),
//! and Write Params writes every recorded value, `ENABLE` parameters first, each in its own `try`,
//! then says "Parameters successfully saved." when none failed (`:179-205`). Refresh Params asks
//! "Update Params" with its "Show me again?" box, fetches the parameters again and rebuilds
//! (`:212-236`); the box, unticked, is kept in `Settings.Instance` as `SHOWAGAIN_Refresh_Params`
//! "False" - on the click, whichever button then closes it - and the question is not asked again
//! (`Common.cs:260-270, 445-448`). Find asks for a word and shows only the controls whose name or description has
//! it, filtering as it is typed, half a second after the last key (`:21-116`).
//!
//! Showing the page again updates the controls it has from the vehicle's values. A bitmask whose
//! bits change there writes each change straight to the vehicle, as the C#'s does: its
//! `ValueChanged` is unsubscribed for the update, and with no subscriber the control writes
//! itself (`:451-457`; `Controls/MavlinkCheckBoxBitMask.cs:143-160`).
//!
//! The layout is `ConfigADSB.resx`'s: the buttons and the panel at their `Location`s, the controls
//! stacked from (12, 67) at their own heights - `RangeControl` 108, `ValuesControl` 89, a bitmask
//! as its rows of check boxes make it - 578 wide, a `RangeControl`'s track bar and maximum
//! anchored to its right as its Designer anchors them.
//!
//! Standard Params and Advanced Params (`ConfigFriendlyParams.cs`) are this page's code with
//! another list, and their page object is this one with another [`Spec`]; see
//! `friendly_params.rs`.
//!
//! The controls take the keyboard and the pointer as WinForms' do. Ctrl+S anywhere on the page is
//! Write Params (`ProcessCmdKey`, `:159-168`): the page takes the focus when it is clicked, as the
//! Flight Modes page does, so the chord reaches it from any of its controls. A `RangeControl`'s
//! track bar moves `LargeChange` - ten of its thousand - on a click beside its thumb, follows the
//! thumb while it is dragged, and with the focus takes the arrows (`SmallChange`, ten), Page Up
//! and Page Down, Home and End (`RangeControl.Designer.cs:53-58`). A `ValuesControl`'s box is a
//! `DropDown` combo: a click on its text puts the caret in it, and the row whose text the typed
//! text matches is selected, as `FindStringExact` selects it, and recorded by
//! `SelectedIndexChanged`. Typed text that names no row leaves `SelectedValue` null, which the
//! C#'s `Value` getter throws a `NullReferenceException` on, into Mission Planner's
//! unhandled-exception box; here the exception's text ([`NULL_REFERENCE`]) goes on the status
//! line and the box keeps the text, no row selected, until it names one. A bitmask's `Value` is
//! narrowed to the parameter's integer type (`TypeAP`) as the C# casts it - "int8 255 = -1" -
//! from the type the vehicle's table carries with each value.
//!
//! What is not ported, and why:
//!
//! * the list panel's own scroll bar: the page scrolls, with the list in it;
//!
//! Where the C# is wrong and this is not: a control added by a later `Activate` - a parameter
//! the vehicle has begun listing - is placed at the top, over the first, because `y` starts again
//! at 10 and the existing controls do not advance it (`:255, 432-457`); here it goes below the
//! last.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::Cell;
use std::cmp::Ordering;
use std::collections::{BTreeMap, VecDeque};
use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Bounds, Context, FocusHandle, KeyDownEvent, MouseButton, MouseDownEvent,
    MouseMoveEvent, MouseUpEvent, Pixels, SharedString, Window, div, prelude::*, px, rgb,
};

use super::battery_monitor::float_text;
use super::optional::{
    Event, Focus, InputBox, Job, Set, SetQueue, at, button, error, input_box, label, message_box,
    set_failed, text_box,
};
use crate::MissionPlanner;
use crate::config::extra_setup::{link_error, take_link_errors};
use crate::config::failsafe::{CheckState, Lookup, decimal_of};
use crate::config::flight_modes::ParamWriter;
use crate::config::servo_output::{Check, Combo, Message, check_box, decimal_text, dropdown};
use crate::setup::Key;
use crate::telemetry::TelemetryView;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, panel, theme};
use mp_params::{ParamMeta, ParamType};

/// Where a parameter's type comes from: the vehicle's table, through the view
/// (`TelemetryView::parameter_type`), or nothing for a parameter the table has no type for.
pub type Types<'a> = &'a dyn Fn(&str) -> Option<ParamType>;

/// The page's title in Initial Setup's list: the literal "ADSB".
/// `// C#: GCSViews/InitialSetup.cs:263`
pub const TITLE: &str = "ADSB";

/// `Strings.WarningUpdateParamList`, Refresh Params' question.
/// `// C#: ExtLibs/Strings/Strings.resx:221-225`
pub const REFRESH_WARNING: &str = "Update Params\nDON'T DO THIS IF YOU ARE IN THE AIR\n";

/// `Strings.ShowMeAgain`.
/// `// C#: ExtLibs/Strings/Strings.resx:441-443`
pub const SHOW_ME_AGAIN: &str = "Show me again?";

/// The setting `MessageShowAgain` keeps the answer under: `SHOWAGAIN_` and the title, "Refresh
/// Params", with its spaces made underscores.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:217; Common.cs:264-268`
pub const SHOW_AGAIN_KEY: &str = "SHOWAGAIN_Refresh_Params";

/// Write Params' box when every write went.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:203`
pub const SAVED: &str = "Parameters successfully saved.";

/// `Strings.ErrorReceivingParams`.
/// `// C#: ExtLibs/Strings/Strings.resx:158-160`
pub const ERROR_RECEIVING: &str = "Error receiving list\n";

/// How long Find waits after a key before it filters.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:42`
pub const FILTER_DELAY: Duration = Duration::from_millis(500);

/// The first control's `y` on a page that places its controls itself.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:62, 255, 530`
const FIRST_Y: f32 = 10.0;

/// A control's `Margin` in a `FlowLayoutPanel`, WinForms' default three pixels each side.
const FLOW_MARGIN: f32 = 3.0;

/// The fixed control ids of a page built from the documentation.
#[derive(Debug, Clone, Copy)]
pub struct Ids {
    /// Write Params.
    pub write: &'static str,
    /// Refresh Params.
    pub refresh: &'static str,
    /// Find.
    pub find: &'static str,
    /// Find's `InputBox`.
    pub find_box: &'static str,
    /// A message box, and its OK.
    pub message: &'static str,
    /// The message box's OK.
    pub message_ok: &'static str,
    /// Refresh Params' question, its "Show me again?", OK and Cancel.
    pub confirm: &'static str,
    /// Its "Show me again?".
    pub confirm_showagain: &'static str,
    /// Its OK.
    pub confirm_ok: &'static str,
    /// Its Cancel.
    pub confirm_cancel: &'static str,
}

/// What sets one page built from the documentation apart from another. `ConfigADSB` is
/// `ConfigFriendlyParams`' code with its own list: the same buttons at the same places, the same
/// `AddControl`, `filterList` and handlers (`diff ConfigADSB.cs ConfigFriendlyParams.cs`), a
/// `tableLayoutPanel1` whose controls it places in place of a `flowLayoutPanel1` that places them,
/// and which parameters get a control.
#[derive(Debug)]
pub struct Spec {
    /// The start of the page's control ids and facts.
    pub name: &'static str,
    /// The page's title in its list.
    pub title: &'static str,
    /// The `Settings` list whose parameters go first.
    pub favourites: &'static str,
    /// Whether a parameter the vehicle has, with its documentation, gets a control.
    pub select: fn(&str, &ParamMeta) -> bool,
    /// The list panel's `Location`.
    pub list_at: (f32, f32),
    /// Its `Size`: the controls are `Width - 50` wide, and `FitDescriptionText` breaks for its
    /// width (a bitmask's, for the control's).
    pub list_size: (f32, f32),
    /// Whether the panel is a `FlowLayoutPanel`, which lays the controls out itself - each three
    /// pixels in, three pixels apart, a hidden one taking no room - rather than a panel whose
    /// controls the page stacks from `y = 10`.
    pub flow: bool,
    /// The fixed control ids.
    pub ids: Ids,
}

impl Spec {
    /// A control's width: `Width - 50`.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:513, 553, 581`
    #[must_use]
    pub fn control_width(&self) -> f32 {
        self.list_size.0 - 50.0
    }

    /// `FitDescriptionText`'s width for a range and a list of values: the panel's.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:508, 583-584`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // a Designer width
    pub fn description_width(&self) -> i32 {
        self.list_size.0 as i32
    }

    /// Its width for a bitmask: the control's.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:552`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // a Designer width
    pub fn bitmask_description_width(&self) -> i32 {
        self.control_width() as i32
    }
}

/// `ConfigADSB`: the `ADSB_` and `AVD_` parameters with a display name, in `tableLayoutPanel1`
/// at (12, 67), 628 by 154.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:324-343, 397-408; ConfigADSB.resx
/// tableLayoutPanel1.Location, .Size`
pub static ADSB: Spec = Spec {
    name: "adsb",
    title: TITLE,
    favourites: "fav_adsb",
    select: |name, meta| {
        (name.starts_with("ADSB_") || name.starts_with("AVD_")) && !meta.display_name.is_empty()
    },
    list_at: (12.0, 67.0),
    list_size: (628.0, 154.0),
    flow: false,
    ids: Ids {
        write: "adsb-write",
        refresh: "adsb-refresh",
        find: "adsb-find",
        find_box: "adsb-find-box",
        message: "adsb-message",
        message_ok: "adsb-message-ok",
        confirm: "adsb-confirm",
        confirm_showagain: "adsb-confirm-showagain",
        confirm_ok: "adsb-confirm-ok",
        confirm_cancel: "adsb-confirm-cancel",
    },
};

/// How a page's drawing reaches its page object in the application: a page built from the
/// documentation is drawn by the same code whichever it is.
pub type Access = fn(&mut MissionPlanner) -> &mut Adsb;

/// `RangeControl`'s and `ValuesControl`'s heights.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:88; ExtLibs/Controls/ValuesControl.Designer.cs:78`
const RANGE_HEIGHT: f32 = 108.0;
/// `ValuesControl`'s.
const VALUES_HEIGHT: f32 = 89.0;

/// `trackBar1.LargeChange`: a click on the channel, Page Up and Page Down.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:53`
const TRACK_PAGE: i32 = 10;
/// `trackBar1.SmallChange`: the arrow keys. `// C#: ExtLibs/Controls/RangeControl.Designer.cs:58`
const TRACK_SMALL: i32 = 10;
/// `trackBar1.Maximum`. `// C#: ExtLibs/Controls/RangeControl.Designer.cs:55`
const TRACK_MAX: i32 = 1000;
/// The thumb's width as drawn; the channel it travels is the bar less this.
const THUMB_WIDTH: f32 = 10.0;

/// `NullReferenceException`'s message: what the C#'s `ValuesControl.Value` throws for typed text
/// that names no row, said on the status line here as `mavftp.rs` says the C#'s crashes.
pub const NULL_REFERENCE: &str = "Object reference not set to an instance of an object.";

// ---------------------------------------------------------------------------------------------
// Text as .NET makes it.
// ---------------------------------------------------------------------------------------------

/// `double.ToString("0.###")`: at most three places, rounded half away from zero, no trailing
/// zeros, and no "-0".
#[must_use]
pub fn three_places(value: f64) -> String {
    let rounded = (value * 1000.0).round() / 1000.0;
    let text = format!("{rounded:.3}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" || text.is_empty() {
        "0".to_owned()
    } else {
        text.to_owned()
    }
}

/// `(decimal)x` for a float: seven significant digits, as an `f64`, with its places.
fn decimal(value: f32) -> (f64, u32) {
    let (mantissa, places) = decimal_of(value);
    #[allow(clippy::cast_precision_loss)] // seven digits fit
    let exact = mantissa as f64 / 10_f64.powi(i32::try_from(places).unwrap_or(0));
    (exact, places)
}

/// `decimal.ToString()`: its digits at its own scale.
fn decimal_string(value: f32) -> String {
    let (exact, places) = decimal(value);
    decimal_text(exact, places)
}

/// A character's place in the order .NET's culture-sensitive comparison gives the characters
/// parameter names use: the underscore before the digits, the digits before the letters, a letter
/// with its other case.
fn collation(c: char) -> (u8, u32) {
    match c {
        '0'..='9' => (1, u32::from(c)),
        'a'..='z' | 'A'..='Z' => (2, u32::from(c.to_ascii_lowercase())),
        _ => (0, u32::from(c)),
    }
}

/// `string.CompareTo` and the default `OrderBy` comparer in an English culture, for parameter
/// names; case alone, which names do not differ by, falls back to the characters' order.
#[must_use]
pub fn culture_cmp(a: &str, b: &str) -> Ordering {
    a.chars()
        .map(collation)
        .cmp(b.chars().map(collation))
        .then_with(|| a.cmp(b))
}

/// `List<string>.SortENABLE`: names ending `ENABLE` first - whatever its comment says - each part
/// in culture order.
/// `// C#: ExtLibs/Utilities/ListExtension.cs:10-28`
pub fn sort_enable(names: &mut [String]) {
    names.sort_by(
        |a, b| match (a.ends_with("ENABLE"), b.ends_with("ENABLE")) {
            (true, false) => Ordering::Less,
            (false, true) => Ordering::Greater,
            _ => culture_cmp(a, b),
        },
    );
}

/// `FitDescriptionText`: the units on a line of their own, then "Description: " and the words, a
/// line break after every `width / 40`th word but the first.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:633-663; ExtLibs/Strings/Strings.resx:244-267`
#[must_use]
pub fn fit_description(units: &str, description: &str, width: i32) -> String {
    let mut text = String::new();
    if !units.is_empty() {
        text.push_str(&format!("Units: {units}\r\n"));
    }
    if !description.is_empty() {
        text.push_str("Description: ");
        let every = width / 40;
        for (index, word) in description.split(' ').enumerate() {
            text.push_str(word);
            text.push(' ');
            let index = i32::try_from(index).unwrap_or(i32::MAX);
            if index != 0 && every != 0 && index % every == 0 {
                text.push_str("\r\n");
            }
        }
    }
    text
}

// ---------------------------------------------------------------------------------------------
// The three controls.
// ---------------------------------------------------------------------------------------------

/// A `RangeControl`: a `NumericUpDown` and a track bar over the documented range.
/// `// C#: ExtLibs/Controls/RangeControl.cs:13-236`
#[derive(Debug)]
pub struct RangeControl {
    /// `_minrange`, `_maxrange`: the documented range.
    min_range: f32,
    max_range: f32,
    /// `DisplayScale`.
    scale: f32,
    /// `Increment`.
    increment: f32,
    /// `numericUpDown1.Minimum`, `.Maximum`, `.DecimalPlaces`, `.Increment`, `.Value`.
    pub minimum: f64,
    /// Its maximum.
    pub maximum: f64,
    /// Its places.
    pub decimals: u32,
    step: f64,
    value: f64,
    /// `trackBar1.Value`, 0 to 1000.
    pub trackbar: i32,
    /// `LBL_min.Text`, `LBL_max.Text`.
    pub lbl_min: String,
    /// The maximum's.
    pub lbl_max: String,
    /// The box's orange: the value outside the documented range.
    pub orange: bool,
    field: TextField,
    edited: bool,
    /// Where the track bar was last laid out, which a drag measures the pointer along.
    pub bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
}

/// `map`, in decimals; `None` for the `DivideByZeroException` of an empty range.
/// `// C#: ExtLibs/Controls/RangeControl.cs:196-199`
fn map(x: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> Option<f64> {
    #[allow(clippy::float_cmp)] // decimals compare exactly
    if in_max == in_min {
        return None;
    }
    Some((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
}

/// `Math.Round(decimal, 0)`: half to even.
fn round_even(value: f64) -> f64 {
    let floor = value.floor();
    let diff = value - floor;
    #[allow(clippy::float_cmp)]
    if diff == 0.5 {
        if floor % 2.0 == 0.0 {
            floor
        } else {
            floor + 1.0
        }
    } else {
        value.round()
    }
}

impl RangeControl {
    /// `new RangeControl(param, Desc, Label, increment, displayscale, minrange, maxrange, value)`,
    /// then the page's own colouring; `None` where the C# throws and `AddControl`'s `catch`
    /// leaves the parameter without a control.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:120-147; GCSViews/ConfigurationView/ConfigADSB.cs:472-537`
    #[must_use]
    pub fn new(
        increment: f32,
        scale: f32,
        min_range: f32,
        max_range: f32,
        value: &str,
    ) -> Option<Self> {
        // `InitializeComponent`: 0 to 100, three places, the labels' text.
        let mut control = Self {
            min_range: 0.0,
            max_range: 10.0,
            scale,
            increment,
            minimum: 0.0,
            maximum: 100.0,
            decimals: 3,
            step: 1.0,
            value: 0.0,
            trackbar: 0,
            lbl_min: "0".to_owned(),
            lbl_max: "65535".to_owned(),
            orange: false,
            field: TextField::new(""),
            edited: false,
            bounds: Rc::new(Cell::new(None)),
        };
        // `Increment = increment`: its step, and `ToString().Length - 1` places.
        let (step, _) = decimal(increment);
        control.step = step;
        control.decimals =
            u32::try_from(float_text(increment).len().saturating_sub(1)).unwrap_or(0);
        control.set_min_range(min_range);
        control.set_max_range(max_range);
        control.set_value(value)?;
        // `AddControl`: orange for a value outside the documented range.
        let parsed: f32 = value.parse().unwrap_or(0.0);
        if parsed < min_range || parsed > max_range {
            control.orange = true;
        }
        Some(control)
    }

    /// `MinRange`'s setter.
    fn set_min_range(&mut self, value: f32) {
        self.min_range = value;
        let (minimum, _) = decimal(value / self.scale);
        self.minimum = minimum;
        if self.maximum < minimum {
            self.maximum = minimum;
        }
        self.value = self.value.clamp(self.minimum, self.maximum);
        self.lbl_min = decimal_string(value / self.scale);
    }

    /// `MaxRange`'s setter.
    fn set_max_range(&mut self, value: f32) {
        self.max_range = value;
        let (maximum, _) = decimal(value / self.scale);
        self.maximum = maximum;
        if self.minimum > maximum {
            self.minimum = maximum;
        }
        self.value = self.value.clamp(self.minimum, self.maximum);
        self.lbl_max = decimal_string(value / self.scale);
    }

    /// The `Value` setter: the range widened to hold the value - the documented range kept - and
    /// the value set, as the vehicle holds it.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:77-102`
    pub fn set_value(&mut self, text: &str) -> Option<()> {
        let parsed: f64 = text.parse().ok()?;
        let (low, high) = (self.min_range, self.max_range);
        #[allow(clippy::cast_possible_truncation)] // `(float)Math.Min(...)`
        let widened_low = f64::from(low).min(parsed) as f32;
        #[allow(clippy::cast_possible_truncation)]
        let widened_high = f64::from(high).max(parsed) as f32;
        self.set_min_range(widened_low);
        self.set_max_range(widened_high);
        self.min_range = low;
        self.max_range = high;
        #[allow(clippy::cast_possible_truncation)] // `(float)decimal.Parse(value)`
        let single = parsed as f32;
        let (shown, _) = decimal(single / self.scale);
        self.value = shown.clamp(self.minimum, self.maximum);
        self.value_changed(false)
    }

    /// `numericUpDown1_ValueChanged`: the track bar follows unless it moved the value, a whole
    /// increment rounds the value, and the box is orange outside the documented range.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:201-226`
    fn value_changed(&mut self, from_trackbar: bool) -> Option<()> {
        if !from_trackbar {
            let mapped = map(
                self.value,
                self.minimum,
                self.maximum,
                0.0,
                f64::from(TRACK_MAX),
            )?;
            #[allow(clippy::cast_possible_truncation)] // `(int)` truncates
            let mapped = mapped as i32;
            self.trackbar = mapped.clamp(0, TRACK_MAX);
        }
        #[allow(clippy::float_cmp)]
        if self.increment % 1.0 == 0.0 {
            let round = round_even(self.value);
            if round > self.minimum && round != self.value {
                self.value = round;
                // The assignment raises the handler again, which settles here.
                return self.value_changed(from_trackbar);
            }
        }
        #[allow(clippy::cast_possible_truncation)] // `(float)numericUpDown1.Value`
        let single = self.value as f32;
        self.orange = single < self.min_range || single > self.max_range;
        self.field.set(self.text());
        self.edited = false;
        Some(())
    }

    /// The box's text: `Value` to `DecimalPlaces` places.
    #[must_use]
    pub fn text(&self) -> String {
        decimal_text(self.value, self.decimals)
    }

    /// What the box shows now, typed or not.
    #[must_use]
    pub fn shown(&self) -> &str {
        self.field.value()
    }

    /// `Value`: `((float)numericUpDown1.Value * DisplayScale).ToString(InvariantCulture)`.
    #[must_use]
    pub fn value_text(&self) -> String {
        #[allow(clippy::cast_possible_truncation)]
        let single = self.value as f32;
        float_text(single * self.scale)
    }

    /// Sets the value as a user's change does; whether it changed, raising `ValueChanged`.
    fn change_to(&mut self, value: f64, from_trackbar: bool) -> bool {
        let value = value.clamp(self.minimum, self.maximum);
        #[allow(clippy::float_cmp)]
        if value == self.value {
            self.field.set(self.text());
            self.edited = false;
            return false;
        }
        self.value = value;
        self.value_changed(from_trackbar).is_some()
    }

    /// `ValidateEditText`: typed text that parses becomes the value, held to the bounds.
    pub fn commit(&mut self) -> bool {
        if !self.edited {
            return false;
        }
        self.edited = false;
        match self.field.value().trim().replace(',', "").parse::<f64>() {
            Ok(typed) => self.change_to(typed, false),
            Err(_) => {
                self.field.set(self.text());
                false
            }
        }
    }

    /// `UpButton` and `DownButton`: the typed text read, then one `Increment`, stopping at the
    /// bound.
    pub fn step(&mut self, up: bool) -> bool {
        let committed = self.commit();
        let next = if up {
            (self.value + self.step).min(self.maximum)
        } else {
            (self.value - self.step).max(self.minimum)
        };
        let next = (next * 1e9).round() / 1e9;
        self.change_to(next, false) || committed
    }

    /// A click on the track bar's channel: `LargeChange` towards the click, then the value from
    /// the bar.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:228-233`
    pub fn page(&mut self, up: bool) -> bool {
        self.nudge(if up { TRACK_PAGE } else { -TRACK_PAGE })
    }

    /// The bar moved `delta` of its thousand, held within it: a channel click, or a key with the
    /// bar focused. Whether the value changed.
    pub fn nudge(&mut self, delta: i32) -> bool {
        let committed = self.commit();
        let next = (self.trackbar + delta).clamp(0, TRACK_MAX);
        self.move_trackbar(next) || committed
    }

    /// The thumb dragged to `fraction` of its travel: `trackBar1.Value` set to the nearest of
    /// the thousand, then the value from the bar. Whether the value changed.
    pub fn drag(&mut self, fraction: f64) -> bool {
        #[allow(clippy::cast_possible_truncation)] // 0 to 1000
        let next = (fraction.clamp(0.0, 1.0) * f64::from(TRACK_MAX)).round() as i32;
        let committed = self.commit();
        self.move_trackbar(next) || committed
    }

    /// `trackBar1.Value = next`: nothing for the value it has, else `trackBar1_ValueChanged`
    /// maps it onto the number.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:228-233`
    fn move_trackbar(&mut self, next: i32) -> bool {
        if next == self.trackbar {
            return false;
        }
        self.trackbar = next;
        let Some(value) = map(
            f64::from(next),
            0.0,
            f64::from(TRACK_MAX),
            self.minimum,
            self.maximum,
        ) else {
            return false;
        };
        self.change_to(value, true)
    }

    /// Where `position` falls along the thumb's travel, 0 to 1, from the bar as it was last laid
    /// out: the pointer less half the thumb, over the bar less the thumb. `None` before it has
    /// been laid out.
    #[must_use]
    pub fn fraction_of(&self, position: gpui::Point<Pixels>) -> Option<f64> {
        let laid_out = self.bounds.get()?;
        let along = f32::from(position.x - laid_out.origin.x) - THUMB_WIDTH / 2.0;
        let travel = (f32::from(laid_out.size.width) - THUMB_WIDTH).max(1.0);
        Some(f64::from((along / travel).clamp(0.0, 1.0)))
    }

    /// A key while the track bar has the focus, as a Win32 trackbar takes them: Left and Up are
    /// `TB_LINEUP`, `SmallChange` down; Right and Down `TB_LINEDOWN`, up; Page Up and Page Down
    /// `LargeChange`; Home the minimum and End the maximum. Whether the key was one of those,
    /// and whether the value changed.
    pub fn trackbar_key(&mut self, event: &KeyDownEvent) -> (bool, bool) {
        let changed = match event.keystroke.key.as_str() {
            "left" | "up" => self.nudge(-TRACK_SMALL),
            "right" | "down" => self.nudge(TRACK_SMALL),
            "pageup" => self.nudge(-TRACK_PAGE),
            "pagedown" => self.nudge(TRACK_PAGE),
            "home" => self.nudge(-TRACK_MAX),
            "end" => self.nudge(TRACK_MAX),
            _ => return (false, false),
        };
        (true, changed)
    }

    /// A key while the box has the focus.
    pub fn key(&mut self, event: &KeyDownEvent) -> (bool, bool) {
        match event.keystroke.key.as_str() {
            "up" => return (true, self.step(true)),
            "down" => return (true, self.step(false)),
            _ => {}
        }
        match self.field.key(event) {
            KeyOutcome::Changed => {
                self.edited = true;
                (true, false)
            }
            KeyOutcome::Submitted => (true, self.commit()),
            KeyOutcome::Cancelled | KeyOutcome::Ignored => (false, false),
        }
    }

    /// Replaces the box's text as typing would.
    #[cfg(test)]
    pub fn type_text(&mut self, text: &str) {
        self.field.set(text);
        self.edited = true;
    }
}

/// A `MavlinkCheckBoxBitMask`: a box per documented bit.
/// `// C#: Controls/MavlinkCheckBoxBitMask.cs:8-160`
#[derive(Debug, Clone)]
pub struct Bitmask {
    /// Each bit, its name and whether it is ticked.
    pub bits: Vec<(u32, &'static str, bool)>,
    /// `Type`: the parameter's `TypeAP`, which `Value` is narrowed to; `REAL32` where the table
    /// has none, as the C#'s field starts.
    pub kind: ParamType,
}

impl Bitmask {
    /// `setup`: each bit ticked as the vehicle's value has it, and the parameter's type kept.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:90-91`
    #[must_use]
    pub fn new(bits: &'static [(u32, &'static str)], value: f64, kind: ParamType) -> Self {
        let mut mask = Self {
            bits: bits
                .iter()
                .map(|(bit, name)| (*bit, *name, false))
                .collect(),
            kind,
        };
        mask.set(value);
        mask
    }

    /// The `Value` setter: returns the bits that changed, in order.
    fn set(&mut self, value: f64) -> Vec<usize> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // `(uint)value`
        let value = value as i64 as u32;
        let mut changed = Vec::new();
        for (index, (bit, _, checked)) in self.bits.iter_mut().enumerate() {
            let on = value & 1_u32.checked_shl(*bit).unwrap_or(0) != 0;
            if *checked != on {
                *checked = on;
                changed.push(index);
            }
        }
        changed
    }

    /// The `Value` getter: the ticked bits added up, then narrowed to the parameter's integer
    /// type - "ie int8 255 = -1" - as the C# casts its float through `sbyte`, `short` or `int`,
    /// which the JIT truncates to the integer's width.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:21-43`
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_possible_wrap
    )]
    pub fn value(&self) -> f32 {
        let bits = self
            .bits
            .iter()
            .filter(|(_, _, checked)| *checked)
            .fold(0_u32, |sum, (bit, _, _)| {
                sum.wrapping_add(1_u32.checked_shl(*bit).unwrap_or(0))
            });
        match self.kind {
            ParamType::Int8 => f32::from(bits as u8 as i8),
            ParamType::Int16 => f32::from(bits as u16 as i16),
            ParamType::Int32 => bits as i32 as f32,
            _ => bits as f32,
        }
    }

    /// Where each box goes and the rows' bottom: 9 from the left and top, each box's width and
    /// five more along, a new row, 22 lower, past 500. A check box's width is its text's at six
    /// pixels a character and the box's eighteen.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:93-130`
    #[must_use]
    pub fn layout(&self) -> (Vec<(f32, f32)>, f32) {
        let (mut left, mut top, mut bottom) = (9.0_f32, 9.0_f32, 0.0_f32);
        let mut places = Vec::new();
        for (_, name, _) in &self.bits {
            places.push((left, top));
            bottom = top + 17.0;
            #[allow(clippy::cast_precision_loss)]
            let width = 18.0 + 6.0 * name.chars().count() as f32;
            left += width + 5.0;
            if left > 500.0 {
                top += 22.0;
                left = 9.0;
            }
        }
        (places, bottom)
    }
}

/// What kind of control a parameter got.
#[derive(Debug)]
pub enum Kind {
    /// A `RangeControl`.
    Range(RangeControl),
    /// A `MavlinkCheckBoxBitMask`.
    Bitmask(Bitmask),
    /// A `ValuesControl`: its combo box, keyed by the documented values.
    Values(Combo),
}

/// One parameter's control.
#[derive(Debug)]
pub struct Control {
    /// The parameter, the control's `Name`.
    pub name: String,
    /// `LabelText`: the display name and, in brackets, the parameter.
    pub label: String,
    /// `DescriptionText`, from `FitDescriptionText`.
    pub description: String,
    /// The control.
    pub kind: Kind,
    /// Whether Find left it showing.
    pub visible: bool,
}

impl Control {
    /// What a fact calls its kind.
    #[must_use]
    pub const fn kind_name(&self) -> &'static str {
        match self.kind {
            Kind::Range(_) => "range",
            Kind::Bitmask(_) => "bitmask",
            Kind::Values(_) => "values",
        }
    }

    /// Its height in the list.
    fn height(&self) -> f32 {
        match &self.kind {
            Kind::Range(_) => RANGE_HEIGHT,
            Kind::Values(_) => VALUES_HEIGHT,
            // `Height = myLabel1.Height + tableLayoutPanel1.Height + 25`, the table its padding,
            // the description's lines and the boxes' rows with their margins.
            Kind::Bitmask(mask) => {
                let (_, bottom) = mask.layout();
                #[allow(clippy::cast_precision_loss)]
                let lines = self.description.lines().count().max(1) as f32;
                23.0 + (5.0 + (13.0 * lines + 6.0) + (bottom + 6.0) + 10.0) + 25.0
            }
        }
    }

    /// What it shows, for the facts.
    #[must_use]
    pub fn shown(&self) -> String {
        match &self.kind {
            Kind::Range(range) => range.shown().to_owned(),
            Kind::Bitmask(mask) => float_text(mask.value()),
            Kind::Values(combo) => combo.text().to_owned(),
        }
    }
}

/// `ValuesControl`'s combo: the documented values, the one the vehicle holds selected - by the
/// text of the values, as `SelectedValue` compares them.
fn values_combo(name: &str, values: &'static [(i64, &'static str)], value: &str) -> Combo {
    let options: Vec<(i64, String)> = values
        .iter()
        .map(|(key, text)| (*key, text.trim().to_owned()))
        .collect();
    let selected = options
        .iter()
        .find(|(key, _)| key.to_string() == value)
        .map(|(key, _)| *key);
    Combo {
        param: name.to_owned(),
        options,
        selected,
        enabled: true,
        top_index: 0,
    }
}

/// `AddControl` for a parameter without one: the control its documentation makes, or `None`.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:460-613; ConfigFriendlyParams.cs:380-536`
#[must_use]
pub fn build(
    spec: &Spec,
    name: &str,
    display: &str,
    value: f64,
    kind: ParamType,
    lookup: Lookup,
) -> Option<Control> {
    let meta = lookup(name)?;
    let text = three_places(value);
    let label = format!("{display} ({name})");
    let units = meta.units;
    if let (Some((low, high)), Some(increment)) = (meta.range, meta.increment) {
        #[allow(clippy::cast_possible_truncation)]
        let (low, high, mut increment) = (low as f32, high as f32, increment as f32);
        if increment > 0.0 {
            let mut scale = 1.0;
            let mut units = units.to_owned();
            if units.eq_ignore_ascii_case("centi-degrees") {
                scale = 100.0;
                units = "Degrees (Scaled)".to_owned();
                increment /= 100.0;
            }
            let description = fit_description(&units, meta.description, spec.description_width());
            // A range the control cannot map throws in its constructor: no control.
            return RangeControl::new(increment, scale, low, high, &text).map(|range| Control {
                name: name.to_owned(),
                label,
                description,
                kind: Kind::Range(range),
                visible: true,
            });
        }
    }
    if !meta.bitmask.is_empty() {
        return Some(Control {
            name: name.to_owned(),
            label,
            description: fit_description(units, meta.description, spec.bitmask_description_width()),
            kind: Kind::Bitmask(Bitmask::new(meta.bitmask, value, kind)),
            visible: true,
        });
    }
    if !meta.values.is_empty() {
        return Some(Control {
            name: name.to_owned(),
            label,
            description: fit_description(units, meta.description, spec.description_width()),
            kind: Kind::Values(values_combo(name, meta.values, &text)),
            visible: true,
        });
    }
    None
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// Find's `InputBox` and what it began with.
#[derive(Debug)]
struct Find {
    input: InputBox,
    /// `searchfor` when it opened, which Cancel puts back.
    before: String,
}

/// The page object: ADSB's, and Standard and Advanced Params', whose code it is. `H` is what the
/// link knows a write by: the vehicle's request, or a test's.
#[derive(Debug)]
pub struct Adsb<H = mp_link::RequestId> {
    /// Which page it is.
    spec: &'static Spec,
    made_for: Option<Key>,
    active: bool,
    /// `tableLayoutPanel1.Controls`, in the order they were added.
    controls: Vec<Control>,
    /// `_params_changed`.
    changed: BTreeMap<String, String>,
    /// `searchfor`.
    search: String,
    /// Find's box, while it is open.
    find: Option<Find>,
    /// Find's box as its OK closed it, until the holder keeps the answer in `Settings.Instance`.
    answered: Option<InputBox>,
    /// When the filter timer fires, while it runs.
    filter_due: Option<Instant>,
    /// Refresh Params' question, while it is asked, and its "Show me again?".
    confirm: Option<bool>,
    /// While the parameters are being fetched again: the table they were fetched over.
    refreshing: Option<Arc<[(String, f64)]>>,
    /// How many times Refresh Params fetched, for the fact a script asserts on: the fetch over
    /// MAVFTP replaces the list too quickly for the disabled button to be seen.
    refreshes: u32,
    /// What the last Refresh Params press decided, for the fact: "fetch", "ask", or why it did
    /// nothing.
    last_press: &'static str,
    /// The combo whose list is down, by the control's index.
    dropdown: Option<usize>,
    /// The number being typed into, by the control's index.
    editing: Option<usize>,
    /// The values box being typed into, by the control's index, and its edit box.
    typed: Option<(usize, TextField)>,
    /// The track bar whose thumb the pointer holds, by the control's index.
    dragging: Option<usize>,
    /// The track bar with the keyboard, by the control's index.
    track_focus: Option<usize>,
    messages: VecDeque<Message>,
    /// The last link failure the C# boxes, for the status line (the owner's ruling of
    /// 2026-09-25), until the holder takes it.
    status: Option<String>,
    queue: SetQueue<H>,
}

impl Default for Adsb {
    /// ADSB's page object, as nothing has shown it.
    fn default() -> Self {
        Self::new(&ADSB)
    }
}

/// `Settings.Instance.GetList(key)` - `"fav_adsb"`, `"fav_params"` - from Mission Planner's
/// `config.xml`: the favourites, `;`-separated and URL-encoded. Nothing on these pages adds to
/// it.
/// `// C#: ExtLibs/Utilities/Settings.cs:164-169`
#[must_use]
pub fn favourites(key: &str) -> Vec<String> {
    mp_settings::Config::default_path()
        .and_then(|path| mp_settings::Config::load(&path).ok())
        .and_then(|config| config.get(key).map(str::to_owned))
        .map(|list| list.split(';').map(url_decode).collect::<Vec<_>>())
        .unwrap_or_default()
}

/// `WebUtility.UrlDecode`: `+` a space, `%XX` a byte.
fn url_decode(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut index = 0;
    while let Some(&byte) = bytes.get(index) {
        match byte {
            b'+' => out.push(b' '),
            b'%' => {
                let hex = bytes
                    .get(index + 1..index + 3)
                    .and_then(|pair| std::str::from_utf8(pair).ok())
                    .and_then(|pair| u8::from_str_radix(pair, 16).ok());
                if let Some(value) = hex {
                    out.push(value);
                    index += 2;
                } else {
                    out.push(b'%');
                }
            }
            other => out.push(other),
        }
        index += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

impl<H: Copy> Adsb<H> {
    /// A page object as nothing has shown it.
    #[must_use]
    pub fn new(spec: &'static Spec) -> Self {
        Self {
            spec,
            made_for: None,
            active: false,
            controls: Vec::new(),
            changed: BTreeMap::new(),
            search: String::new(),
            find: None,
            answered: None,
            filter_due: None,
            confirm: None,
            refreshing: None,
            refreshes: 0,
            last_press: "none",
            dropdown: None,
            editing: None,
            typed: None,
            dragging: None,
            track_focus: None,
            messages: VecDeque::new(),
            status: None,
            queue: SetQueue::default(),
        }
    }

    /// Which page it is.
    #[must_use]
    pub const fn spec(&self) -> &'static Spec {
        self.spec
    }

    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Find's question, while its box is open.
    #[must_use]
    pub fn find_prompt(&self) -> Option<&'static str> {
        self.find.as_ref().map(|find| find.input.prompt)
    }

    /// Whether Refresh Params' question is showing.
    #[must_use]
    pub const fn confirming(&self) -> bool {
        self.confirm.is_some()
    }

    /// The controls, in the order they were added.
    #[must_use]
    pub fn controls(&self) -> &[Control] {
        &self.controls
    }

    /// A control by its parameter.
    #[cfg(test)]
    #[must_use]
    pub fn control(&self, name: &str) -> Option<&Control> {
        self.controls.iter().find(|control| control.name == name)
    }

    /// `_params_changed`.
    #[must_use]
    pub const fn changed(&self) -> &BTreeMap<String, String> {
        &self.changed
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

    /// The words of the last link failure since the holder last asked, for the status line.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// Whether Refresh Params can be pressed: not while it is fetching.
    #[must_use]
    pub fn refresh_enabled(&self) -> bool {
        self.refreshing.is_none()
    }

    /// Shows the page: a new page object for a new screen, then `Activate`, which binds the
    /// list. Returns the writes a changed bitmask makes.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:253-258, 324-419; ConfigFriendlyParams.cs:253-337`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        types: Types<'_>,
        key: Key,
        lookup: Lookup,
        favourites: &[String],
    ) -> Vec<Job> {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let status = self.status.take();
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(key),
                messages,
                status,
                queue,
                ..Self::new(self.spec)
            };
        }
        self.active = true;
        self.dropdown = None;
        self.bind(parameters, types, lookup, favourites)
    }

    /// `BindParamList`: each parameter the page's list takes (for ADSB each `ADSB_` and `AVD_`
    /// parameter with a display name), favourites first, by name, given its control or its
    /// control given the vehicle's value. `ConfigFriendlyParams` builds the new controls first
    /// and sorts them after, by the same key, which comes to the same order.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:349-419; ConfigFriendlyParams.cs:299-337`
    fn bind(
        &mut self,
        parameters: &[(String, f64)],
        types: Types<'_>,
        lookup: Lookup,
        favourites: &[String],
    ) -> Vec<Job> {
        let select = self.spec.select;
        let mut names: Vec<(String, String, f64)> = parameters
            .iter()
            .filter_map(|(name, value)| {
                let meta = lookup(name)?;
                select(name, meta).then(|| (name.clone(), meta.display_name.to_owned(), *value))
            })
            .collect();
        let order = |name: &str| {
            if favourites.iter().any(|favourite| favourite == name) {
                format!("0{name}")
            } else {
                name.to_owned()
            }
        };
        names.sort_by(|a, b| culture_cmp(&order(&a.0), &order(&b.0)));
        let mut jobs = Vec::new();
        for (name, display, value) in names {
            if let Some(existing) = self
                .controls
                .iter_mut()
                .find(|control| control.name == name)
            {
                let text = three_places(value);
                match &mut existing.kind {
                    Kind::Range(range) => {
                        let _ = range.set_value(&text);
                    }
                    Kind::Values(combo) => {
                        combo.selected = combo
                            .options
                            .iter()
                            .find(|(key, _)| key.to_string() == text)
                            .map(|(key, _)| *key);
                    }
                    Kind::Bitmask(mask) => {
                        // Each box that changes raises the control's own write, with the value
                        // its boxes add up to so far.
                        let mut sets = Vec::new();
                        let target: f32 = text.parse().unwrap_or(0.0);
                        let mut partial = mask.clone();
                        for index in mask.set(f64::from(target)) {
                            if let Some(bit) = partial.bits.get_mut(index) {
                                bit.2 = !bit.2;
                            }
                            sets.push(Set::control(crate::config::servo_output::Write::other(
                                &name,
                                f64::from(partial.value()),
                            )));
                        }
                        if !sets.is_empty() {
                            jobs.push(Job::new("bitmask", sets));
                        }
                    }
                }
                continue;
            }
            let kind = types(&name).unwrap_or(ParamType::Real32);
            if let Some(control) = build(self.spec, &name, &display, value, kind, lookup) {
                self.controls.push(control);
            }
        }
        jobs
    }

    /// `Deactivate`: its unsubscriptions are of packets the C# no longer subscribes to.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:667-671`
    pub fn deactivate(&mut self) {
        self.active = false;
        self.dropdown = None;
        self.typed = None;
        self.dragging = None;
        self.track_focus = None;
        if let Some(index) = self.editing.take() {
            self.commit(index);
        }
    }

    /// `Control_ValueChanged`: the value recorded for Write Params.
    fn record_change(&mut self, index: usize) {
        let Some(control) = self.controls.get(index) else {
            return;
        };
        let value = match &control.kind {
            Kind::Range(range) => range.value_text(),
            Kind::Bitmask(mask) => float_text(mask.value()),
            Kind::Values(combo) => match combo.selected {
                Some(key) => key.to_string(),
                None => return,
            },
        };
        self.changed.insert(control.name.clone(), value);
    }

    /// Drops a values control's list down, or back up.
    pub fn toggle_dropdown(&mut self, index: usize) {
        self.leave();
        self.dropdown = if self.dropdown == Some(index) {
            None
        } else {
            if let Some(Control {
                kind: Kind::Values(combo),
                ..
            }) = self.controls.get_mut(index)
            {
                combo.open_list();
            }
            Some(index)
        };
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, index: usize, lines: i32) {
        if self.dropdown == Some(index)
            && let Some(Control {
                kind: Kind::Values(combo),
                ..
            }) = self.controls.get_mut(index)
        {
            combo.scroll_list(lines);
        }
    }

    /// A value chosen: `SelectedIndexChanged`, recorded when the row changed.
    pub fn choose(&mut self, index: usize, key: i64) {
        self.dropdown = None;
        let changed = match self.controls.get_mut(index) {
            Some(Control {
                kind: Kind::Values(combo),
                ..
            }) => combo.select(key),
            _ => false,
        };
        if changed {
            self.record_change(index);
        }
    }

    /// A bitmask's box clicked.
    pub fn click_bit(&mut self, index: usize, bit: usize) {
        self.leave();
        self.dropdown = None;
        let changed = match self.controls.get_mut(index) {
            Some(Control {
                kind: Kind::Bitmask(mask),
                ..
            }) => mask
                .bits
                .get_mut(bit)
                .map(|entry| entry.2 = !entry.2)
                .is_some(),
            _ => false,
        };
        if changed {
            self.record_change(index);
        }
    }

    fn range_mut(&mut self, index: usize) -> Option<&mut RangeControl> {
        match self.controls.get_mut(index) {
            Some(Control {
                kind: Kind::Range(range),
                ..
            }) => Some(range),
            _ => None,
        }
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize) {
        if self.editing == Some(index) {
            return;
        }
        self.leave();
        self.dropdown = None;
        if self.range_mut(index).is_some() {
            self.editing = Some(index);
        }
    }

    /// The number or values box being typed into loses the focus: the number is validated, and
    /// the values box shows its row's text again.
    pub fn leave(&mut self) {
        self.typed = None;
        if let Some(index) = self.editing.take() {
            self.commit(index);
        }
    }

    fn commit(&mut self, index: usize) {
        if self.range_mut(index).is_some_and(RangeControl::commit) {
            self.record_change(index);
        }
    }

    /// A key for the number being typed into.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
        let Some((handled, changed)) = self.range_mut(index).map(|range| range.key(event)) else {
            return false;
        };
        if changed {
            self.record_change(index);
        }
        handled
    }

    /// A number's arrow.
    pub fn step(&mut self, index: usize, up: bool) {
        self.begin(index);
        if self.range_mut(index).is_some_and(|range| range.step(up)) {
            self.record_change(index);
        }
    }

    /// A click on a track bar's channel, above or below its thumb: the bar takes the focus, and
    /// moves a `LargeChange` towards the click.
    pub fn page_trackbar(&mut self, index: usize, up: bool) {
        self.leave();
        self.dropdown = None;
        if self.range_mut(index).is_none() {
            return;
        }
        self.track_focus = Some(index);
        if self.range_mut(index).is_some_and(|range| range.page(up)) {
            self.record_change(index);
        }
    }

    /// The thumb taken by the pointer: the bar takes the focus and follows the pointer until
    /// the button is let go ([`Self::drag_thumb`], [`Self::release_thumb`]).
    pub fn grab_thumb(&mut self, index: usize) {
        self.leave();
        self.dropdown = None;
        if self.range_mut(index).is_some() {
            self.dragging = Some(index);
            self.track_focus = Some(index);
        }
    }

    /// The pointer moved with a thumb held: the bar follows it along its travel, and every value
    /// it passes is recorded, as `trackBar1_ValueChanged` raises each.
    pub fn drag_thumb(&mut self, position: gpui::Point<Pixels>) {
        let Some(index) = self.dragging else {
            return;
        };
        let Some(range) = self.range_mut(index) else {
            return;
        };
        let Some(fraction) = range.fraction_of(position) else {
            return;
        };
        if range.drag(fraction) {
            self.record_change(index);
        }
    }

    /// The button let go.
    pub fn release_thumb(&mut self) {
        self.dragging = None;
    }

    /// The track bar the pointer holds.
    #[must_use]
    pub const fn dragging(&self) -> Option<usize> {
        self.dragging
    }

    /// The track bar with the keyboard.
    #[must_use]
    pub const fn track_focus(&self) -> Option<usize> {
        self.track_focus
    }

    /// A key for the track bar with the keyboard; whether it took it.
    pub fn trackbar_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some(index) = self.track_focus else {
            return false;
        };
        let Some((handled, changed)) = self.range_mut(index).map(|range| range.trackbar_key(event))
        else {
            return false;
        };
        if changed {
            self.record_change(index);
        }
        handled
    }

    /// `ProcessCmdKey`: Ctrl+S anywhere on the page is Write Params. Its jobs, when it was.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:159-168; ConfigFriendlyParams.cs:159-168`
    pub fn chord(&mut self, event: &KeyDownEvent) -> Option<Vec<Job>> {
        let keystroke = &event.keystroke;
        let chord = keystroke.modifiers.control || keystroke.modifiers.platform;
        (chord && keystroke.key == "s").then(|| self.write_params())
    }

    /// A values box clicked into: the `DropDown` combo's edit box, with its row's text.
    pub fn begin_typing(&mut self, index: usize) {
        if self.typing() == Some(index) {
            return;
        }
        self.leave();
        self.dropdown = None;
        if let Some(Control {
            kind: Kind::Values(combo),
            ..
        }) = self.controls.get(index)
        {
            let mut field = TextField::new("");
            field.set(combo.text().to_owned());
            self.typed = Some((index, field));
        }
    }

    /// The values box being typed into, by the control's index.
    #[must_use]
    pub fn typing(&self) -> Option<usize> {
        self.typed.as_ref().map(|(index, _)| *index)
    }

    /// What the values box being typed into shows.
    #[must_use]
    pub fn typed_text(&self) -> Option<&str> {
        self.typed.as_ref().map(|(_, field)| field.value())
    }

    /// A key in the values box: text changed selects the row it names, or none; Enter or Escape
    /// leaves the box. Whether the key was taken.
    pub fn type_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some((index, field)) = self.typed.as_mut() else {
            return false;
        };
        let index = *index;
        match field.key(event) {
            KeyOutcome::Changed => {
                let text = field.value().to_owned();
                self.typed_changed(index, &text);
                true
            }
            KeyOutcome::Submitted | KeyOutcome::Cancelled => {
                self.leave();
                true
            }
            KeyOutcome::Ignored => false,
        }
    }

    /// The edit box's text as the combo takes it: the row whose text it is - `FindStringExact`,
    /// case aside - selected, and `SelectedIndexChanged` records it; none is `SelectedIndex` -1,
    /// on which the C#'s `Value` getter throws ([`NULL_REFERENCE`], here the status line).
    /// `// C#: ExtLibs/Controls/ValuesControl.cs:33-41, 68-72`
    fn typed_changed(&mut self, index: usize, text: &str) {
        let Some(Control {
            kind: Kind::Values(combo),
            ..
        }) = self.controls.get_mut(index)
        else {
            return;
        };
        let named = combo
            .options
            .iter()
            .find(|(_, option)| option.eq_ignore_ascii_case(text.trim()))
            .map(|(key, _)| *key);
        match named {
            Some(key) => {
                if combo.select(key) {
                    self.record_change(index);
                }
            }
            None => {
                combo.selected = None;
                self.status = Some(NULL_REFERENCE.to_owned());
            }
        }
    }

    /// Types into the values box, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, text: &str) {
        let Some((index, field)) = self.typed.as_mut() else {
            return;
        };
        let index = *index;
        field.set(text);
        self.typed_changed(index, text);
    }

    /// Write Params: every recorded value, `ENABLE` names first, each in its own `try`.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:179-205`
    pub fn write_params(&mut self) -> Vec<Job> {
        self.leave();
        self.dropdown = None;
        let mut names: Vec<String> = self.changed.keys().cloned().collect();
        sort_enable(&mut names);
        let sets = names.into_iter().filter_map(|name| {
            let value: f32 = self.changed.get(&name)?.parse().ok()?;
            Some(Set::caught(
                name.clone(),
                f64::from(value),
                set_failed(&name),
            ))
        });
        let mut job = Job::new("write", sets);
        job.each_caught = true;
        vec![job]
    }

    /// Refresh Params, before its question: with no link, nothing; with "Show me again?" turned
    /// off - `shown_again`, the `SHOWAGAIN_Refresh_Params` setting, there and not `GetBoolean`'s
    /// true - straight to the fetch, as `MessageShowAgain`'s OK.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:212-218; Common.cs:260-270`
    /// Returns whether to fetch the parameters now: `getParamList`, which the caller starts.
    pub fn press_refresh(
        &mut self,
        connected: bool,
        view: &TelemetryView,
        shown_again: Option<&str>,
    ) -> bool {
        self.leave();
        self.dropdown = None;
        if !connected {
            self.last_press = "refused: not connected";
            return false;
        }
        if !self.refresh_enabled() {
            self.last_press = "refused: fetching";
            return false;
        }
        // `ContainsKey(key) && GetBoolean(key) == false`: a value `bool.TryParse` refuses is
        // false too.
        let suppressed =
            shown_again.is_some_and(|value| !crate::raw_params::get_boolean(Some(value)));
        if suppressed {
            self.last_press = "fetch";
            self.refresh(view)
        } else {
            self.last_press = "ask";
            self.confirm = Some(true);
            false
        }
    }

    /// The question's "Show me again?" box clicked: the value `chk_CheckStateChanged` writes
    /// under [`SHOW_AGAIN_KEY`] at once, `Checked.ToString()`, for the holder to write.
    /// `// C#: Common.cs:388-399, 445-448`
    pub fn toggle_show_again(&mut self) -> Option<&'static str> {
        let checked = self.confirm.as_mut()?;
        *checked = !*checked;
        Some(if *checked { "True" } else { "False" })
    }

    /// The question answered: OK fetches, Cancel does nothing. Returns whether to fetch.
    pub fn answer_refresh(&mut self, ok: bool, view: &TelemetryView) -> bool {
        self.confirm.take().is_some() && ok && self.refresh(view)
    }

    /// `getParamList`, the button disabled until the list is whole again: always a fetch.
    fn refresh(&mut self, view: &TelemetryView) -> bool {
        self.refreshing = Some(Arc::clone(&view.parameters));
        self.refreshes += 1;
        true
    }

    /// Find: its `InputBox`, holding the last word.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:21-32`
    pub fn open_find(&mut self) {
        self.leave();
        self.dropdown = None;
        self.find = Some(Find {
            input: InputBox::new(
                "Search For",
                "Enter a single word to search for",
                &self.search,
            ),
            before: self.search.clone(),
        });
    }

    /// A key in Find's box: the word follows the text and the timer starts again.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:34-45`
    pub fn find_key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(find) = self.find.as_mut() else {
            return false;
        };
        match find.input.field.key(event) {
            KeyOutcome::Changed => {
                self.search = find.input.field.value().to_owned();
                self.filter_due = Some(now + FILTER_DELAY);
            }
            KeyOutcome::Submitted => self.close_find(true),
            KeyOutcome::Cancelled => self.close_find(false),
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// Find's box closed: OK filters by what it holds; Cancel puts the word back and shows all.
    pub fn close_find(&mut self, ok: bool) {
        let Some(find) = self.find.take() else {
            return;
        };
        if ok {
            self.search = find.input.field.value().to_owned();
            let search = self.search.clone();
            self.answered = Some(find.input);
            self.filter(&search);
        } else {
            self.search = find.before;
            self.filter("");
        }
    }

    /// Find's box as its OK closed it, once: the answer `InputBox` keeps in `Settings.Instance`
    /// under `InputBoxSearchForEnterasinglewordtosearchfor`, which [`keep_find_answer`] writes.
    /// `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    pub fn take_answered(&mut self) -> Option<InputBox> {
        self.answered.take()
    }

    /// Types into Find's box, for a test.
    #[cfg(test)]
    pub fn type_find(&mut self, text: &str, now: Instant) {
        if let Some(find) = self.find.as_mut() {
            find.input.field.set(text);
            self.search = text.to_owned();
            self.filter_due = Some(now + FILTER_DELAY);
        }
    }

    /// `filterList`: two letters or none, matched without case against each control's name and
    /// description.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:58-116`
    pub fn filter(&mut self, search: &str) {
        let count = search.chars().count();
        if count < 2 && count != 0 {
            return;
        }
        let word = search.to_lowercase();
        for control in &mut self.controls {
            control.visible = control.label.to_lowercase().contains(&word)
                || control.description.to_lowercase().contains(&word);
        }
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// Once a frame: a page object whose screen has gone is let go, Find's timer, the fetch's
    /// end, and the writes, with Write Params' box when they are done. `on_screen` is whether the
    /// page's screen - SETUP for ADSB, CONFIG for the others - is showing.
    pub fn tick<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        view: &TelemetryView,
        on_screen: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_screen || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
            self.controls.clear();
            self.changed.clear();
            self.refreshing = None;
            self.find = None;
            self.confirm = None;
            self.filter_due = None;
        }
        if let Some(due) = self.filter_due
            && now >= due
        {
            self.filter_due = None;
            let search = self.search.clone();
            self.filter(&search);
        }
        if let Some(before) = &self.refreshing {
            let whole = !view.parameters.is_empty()
                && view.parameters.len() >= usize::from(view.parameters_expected);
            if !view.connected {
                self.refreshing = None;
                // `Strings.ErrorReceivingParams` in an error box: a status line instead, below.
                // `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:228; ConfigFriendlyParams.cs:228`
                self.messages.push_back(error(ERROR_RECEIVING));
            } else if whole && !Arc::ptr_eq(before, &view.parameters) {
                self.refreshing = None;
                if self.active {
                    let favourites = favourites(self.spec.favourites);
                    let jobs = self.bind(
                        &view.parameters,
                        &|name| view.parameter_type(name),
                        crate::metadata::lookup,
                        &favourites,
                    );
                    self.queue.push(jobs);
                }
            }
        }
        for event in self.queue.advance(writer, &mut self.messages) {
            if let Event::Done {
                tag: "write",
                threw,
            } = event
            {
                self.written(threw);
            }
        }
        // The C#'s boxes for the link failing go on the status line, never in a box: the owner's
        // ruling of 2026-09-25 (PLAN.md §12) - the link's state is always at the top right.
        // Write Params' `catch`, "Set X Failed" (`ConfigADSB.cs:197`, `ConfigFriendlyParams.cs:197`),
        // a bitmask's own write (`Controls/MavlinkCheckBoxBitMask.cs:154, 158`) and the fetch's "Error
        // receiving list" (`:228`) are all `Strings.ERROR` boxes, and every `Strings.ERROR` box
        // this page puts up is one of them. Write Params' "Parameters successfully saved." (`:203`)
        // is a report of success, and Refresh Params' question and Find's `InputBox` are
        // questions: they keep their boxes.
        if let Some(words) = take_link_errors(&mut self.messages, link_error) {
            self.status = Some(words);
        }
    }

    /// Write Params' end: with nothing thrown, the recorded values cleared and the box.
    fn written(&mut self, threw: bool) {
        if threw {
            return;
        }
        self.changed.clear();
        self.messages.push_back(Message {
            title: "Saved",
            text: SAVED.to_owned(),
        });
    }
}

/// Facts a UI test asserts on, under `config.<name>.`: `config.adsb.` for ADSB.
pub fn record_facts(page: &Adsb, view: &TelemetryView) {
    use crate::facts::record;
    let fact = |what: &str| format!("config.{}.{what}", page.spec.name);
    record(fact("active"), page.is_active());
    record(fact("controls"), page.controls().len());
    record(
        fact("visible"),
        page.controls()
            .iter()
            .filter(|control| control.visible)
            .count(),
    );
    record(
        fact("order"),
        page.controls()
            .iter()
            .map(|control| control.name.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    record(fact("search"), &page.search);
    record(fact("find"), page.find_prompt().unwrap_or("none"));
    record(
        fact("confirm"),
        if page.confirming() {
            REFRESH_WARNING.lines().next().unwrap_or("")
        } else {
            "none"
        },
    );
    record(fact("refresh.enabled"), page.refresh_enabled());
    record(fact("refreshes"), page.refreshes);
    record(fact("press"), page.last_press);
    record(fact("changed"), page.changed().len());
    for (name, value) in page.changed() {
        record(fact(&format!("changed.{name}")), value);
    }
    for control in page.controls() {
        let name = &control.name;
        record(fact(&format!("{name}.kind")), control.kind_name());
        record(fact(&format!("{name}.text")), control.shown());
        record(fact(&format!("{name}.visible")), control.visible);
        record(fact(&format!("{name}.label")), &control.label);
        if let Kind::Range(range) = &control.kind {
            record(fact(&format!("{name}.trackbar")), range.trackbar);
        }
    }
    let name_of = |index: Option<usize>| {
        index
            .and_then(|index| page.controls().get(index))
            .map_or("none", |control| control.name.as_str())
    };
    record(fact("typing"), name_of(page.typing()));
    record(fact("dragging"), name_of(page.dragging()));
    record(fact("track.focus"), name_of(page.track_focus()));
    record(fact("write"), page.queue.last().unwrap_or("none"));
    record(fact("writes.pending"), page.queue.pending());
    record(
        fact("message"),
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    for (name, value) in view.parameters.iter() {
        if page.controls().iter().any(|control| control.name == *name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// A page whose `RangeControl`s [`range_number`] and [`trackbar`] drive: ADSB's list, and FFT
/// Setup's one (`fft.rs`).
pub trait RangeHost {
    /// A key for the number being typed into; whether it was handled.
    fn key(&mut self, event: &KeyDownEvent) -> bool;
    /// A number clicked into.
    fn begin(&mut self, index: usize);
    /// A number's arrow.
    fn step(&mut self, index: usize, up: bool);
    /// A click on a track bar's channel, above or below its thumb.
    fn page_trackbar(&mut self, index: usize, up: bool);
    /// A track bar's thumb taken by the pointer.
    fn grab_thumb(&mut self, index: usize);
    /// The pointer moved with a thumb held.
    fn drag_thumb(&mut self, position: gpui::Point<Pixels>);
    /// The button let go.
    fn release_thumb(&mut self);
    /// A key for the track bar with the keyboard; whether it took it.
    fn trackbar_key(&mut self, event: &KeyDownEvent) -> bool;
    /// The track bar the pointer holds.
    fn dragging(&self) -> Option<usize>;
}

impl RangeHost for Adsb {
    fn key(&mut self, event: &KeyDownEvent) -> bool {
        Self::key(self, event)
    }

    fn begin(&mut self, index: usize) {
        Self::begin(self, index);
    }

    fn step(&mut self, index: usize, up: bool) {
        Self::step(self, index, up);
    }

    fn page_trackbar(&mut self, index: usize, up: bool) {
        Self::page_trackbar(self, index, up);
    }

    fn grab_thumb(&mut self, index: usize) {
        Self::grab_thumb(self, index);
    }

    fn drag_thumb(&mut self, position: gpui::Point<Pixels>) {
        Self::drag_thumb(self, position);
    }

    fn release_thumb(&mut self) {
        Self::release_thumb(self);
    }

    fn trackbar_key(&mut self, event: &KeyDownEvent) -> bool {
        Self::trackbar_key(self, event)
    }

    fn dragging(&self) -> Option<usize> {
        Self::dragging(self)
    }
}

/// A control's name and description, as `RangeControl` paints them and the others label them.
fn texts(control: &Control, width: f32, x: f32, y: f32) -> AnyElement {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width - 6.0))
        .flex()
        .flex_col()
        .child(
            div()
                .text_sm()
                .font_weight(gpui::FontWeight::BOLD)
                .text_color(rgb(theme::TEXT))
                .whitespace_nowrap()
                .child(control.label.clone()),
        )
        .children(control.description.replace('\r', "").lines().map(|line| {
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .whitespace_nowrap()
                .child(line.to_owned())
        }))
        .into_any_element()
}

/// A `RangeControl`'s number: its text, the arrows, orange outside the documented range.
#[allow(clippy::too_many_arguments)]
pub fn range_number<H: RangeHost + 'static>(
    id: String,
    range: &RangeControl,
    index: usize,
    editing: bool,
    access: fn(&mut MissionPlanner) -> &mut H,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = editing && handle.is_focused(window);
    let text = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id.clone()))
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_text()
        .child(
            div()
                .flex_1()
                .whitespace_nowrap()
                .child(range.shown().to_owned()),
        )
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text = if editing {
        text.track_focus(handle)
            .key_context("TextField")
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if access(this).key(event) {
                    cx.notify();
                }
            }))
    } else {
        let handle = handle.clone();
        text.on_click(cx.listener(move |this, _event, window, cx| {
            access(this).begin(index);
            handle.focus(window, cx);
            cx.notify();
        }))
    };
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let name = format!("{id}-{}", if up { "up" } else { "down" });
        crate::probe::measured(name.clone(), div())
            .id(SharedString::from(name))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(if up { "▲" } else { "▼" })
            .on_click(cx.listener(move |this, _event, _window, cx| {
                access(this).step(index, up);
                cx.notify();
            }))
    };
    at(6.0, 58.0, 57.0, 20.0)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if range.orange {
            theme::WARN
        } else if focused {
            theme::ACCENT
        } else {
            theme::OK
        }))
        .bg(rgb(theme::ACTION))
        .child(text)
        .child(
            div()
                .w(px(12.0))
                .h_full()
                .flex()
                .flex_col()
                .border_l_1()
                .border_color(rgb(theme::BORDER))
                .child(arrow(true, cx))
                .child(arrow(false, cx)),
        )
        .into_any_element()
}

/// A `RangeControl`'s track bar: the channel either side of the thumb takes a click, the thumb
/// is dragged, and with the focus - a click on either gives it - the bar takes the keys. Anchored
/// left and right, it keeps the Designer's margins as the control is made wider: 69 in from the
/// left, 3 from the right. `holds_keys` says the host gave this bar the keyboard; it has it while
/// `track` is focused, as the Simple PIDs page's bars have theirs.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:48-58`
#[allow(clippy::too_many_arguments)] // the bar's host, its focus and the window it is drawn in
pub fn trackbar<H: RangeHost + 'static>(
    id: &str,
    range: &RangeControl,
    index: usize,
    width: f32,
    access: fn(&mut MissionPlanner) -> &mut H,
    holds_keys: bool,
    track: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    #[allow(clippy::cast_precision_loss)]
    let thumb = (width - THUMB_WIDTH) * range.trackbar as f32 / TRACK_MAX as f32;
    let focused = holds_keys && track.is_focused(window);
    let laid = Rc::clone(&range.bounds);
    let side = |up: bool, cx: &mut Context<MissionPlanner>| {
        let name = format!("{id}-track-{}", if up { "up" } else { "down" });
        let handle = track.clone();
        crate::probe::measured(name.clone(), div())
            .id(SharedString::from(name))
            .h_full()
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, window, cx| {
                access(this).page_trackbar(index, up);
                handle.focus(window, cx);
                cx.notify();
            }))
    };
    let thumb_id = format!("{id}-thumb");
    let grab = track.clone();
    let track_id = format!("{id}-track");
    let bar = crate::probe::measured(track_id.clone(), at(69.0, 58.0, width, 20.0))
        .id(SharedString::from(track_id))
        .flex()
        .items_center()
        .child(
            gpui::canvas(
                move |laid_out, _window, _cx| laid.set(Some(laid_out)),
                |_bounds, (), _window, _cx| {},
            )
            .absolute()
            .inset_0(),
        )
        .child(
            div()
                .absolute()
                .left_0()
                .top(px(9.0))
                .w_full()
                .h(px(2.0))
                .bg(rgb(theme::BORDER)),
        )
        .child(side(false, cx).w(px(thumb)))
        .child(
            crate::probe::measured(thumb_id.clone(), div())
                .id(SharedString::from(thumb_id))
                .w(px(THUMB_WIDTH))
                .h(px(18.0))
                .flex_shrink_0()
                .rounded_sm()
                .bg(rgb(if focused { theme::OK } else { theme::ACCENT }))
                .cursor_pointer()
                // The probe measures an element by its children: one, the thumb's size.
                .child(div().size_full())
                .on_mouse_down(
                    MouseButton::Left,
                    cx.listener(move |this, _event: &MouseDownEvent, window, cx| {
                        access(this).grab_thumb(index);
                        grab.focus(window, cx);
                        // The page under it would take the focus back.
                        cx.stop_propagation();
                        cx.notify();
                    }),
                ),
        )
        .child(side(true, cx).flex_1())
        .on_mouse_move(
            cx.listener(move |this, event: &MouseMoveEvent, _window, cx| {
                if event.pressed_button != Some(MouseButton::Left)
                    || access(this).dragging() != Some(index)
                {
                    return;
                }
                access(this).drag_thumb(event.position);
                cx.notify();
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(move |this, _event: &MouseUpEvent, _window, cx| {
                access(this).release_thumb();
                cx.notify();
            }),
        )
        .on_mouse_up_out(
            MouseButton::Left,
            cx.listener(move |this, _event: &MouseUpEvent, _window, cx| {
                access(this).release_thumb();
                cx.notify();
            }),
        );
    if focused {
        bar.track_focus(track)
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if access(this).trackbar_key(event) {
                    cx.stop_propagation();
                    cx.notify();
                }
            }))
            .into_any_element()
    } else {
        bar.into_any_element()
    }
}

/// A `ValuesControl`'s combo box, a `DropDown`: its edit box, which a click puts the caret in
/// and typing selects rows from, and its arrow, which drops the list down.
/// `// C#: ExtLibs/Controls/ValuesControl.cs:33-41, 68-72; ValuesControl.Designer.cs`
#[allow(clippy::too_many_arguments)] // the box's host, its focus and the window it is drawn in
fn values_box(
    id: String,
    combo: &Combo,
    index: usize,
    typing: bool,
    typed: Option<&str>,
    access: Access,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = typing && handle.is_focused(window);
    let shown = typed.unwrap_or_else(|| combo.text()).to_owned();
    let text_id = format!("{id}-text");
    let text = crate::probe::measured(text_id.clone(), div())
        .id(SharedString::from(text_id))
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_text()
        .child(div().flex_1().whitespace_nowrap().child(shown))
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text = if typing {
        text.track_focus(handle)
            .key_context("TextField")
            .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if access(this).type_key(event) {
                    cx.stop_propagation();
                    cx.notify();
                }
            }))
    } else {
        let handle = handle.clone();
        text.on_click(cx.listener(move |this, _event, window, cx| {
            access(this).begin_typing(index);
            handle.focus(window, cx);
            cx.notify();
        }))
    };
    let arrow = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .w(px(16.0))
        .h_full()
        .flex()
        .items_center()
        .justify_center()
        .border_l_1()
        .border_color(rgb(theme::BORDER))
        .text_size(px(7.0))
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child("▼")
        .on_click(cx.listener(move |this, _event, _window, cx| {
            access(this).toggle_dropdown(index);
            cx.notify();
        }));
    at(3.0, 65.0, 207.0, 21.0)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::ACTION))
        .child(text)
        .child(arrow)
        .into_any_element()
}

/// Write Params, Refresh Params and Find at their places, and the list panel's controls - one
/// per parameter Find leaves showing, each at its height - with a values control's list dropped
/// down over them. The page's own panel is the caller's.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.Designer.cs; ConfigFriendlyParams.Designer.cs:29-74;
/// ConfigFriendlyParams.resx`
#[allow(clippy::too_many_arguments)] // the page's four focuses, and the window they are read in
pub fn list_body(
    list: &Adsb,
    access: Access,
    number: &FocusHandle,
    prompt: &FocusHandle,
    page: &FocusHandle,
    track: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Stateful<gpui::Div> {
    let spec = list.spec;
    let ids = spec.ids;
    let prompt = prompt.clone();
    let page_handle = page.clone();
    // The page takes the focus when it is clicked, so `ProcessCmdKey`'s Ctrl+S reaches it from
    // any of its controls: a key bubbles from the focused box up to here.
    let mut body = crate::probe::measured(format!("{}-page", spec.name), div())
        .id(SharedString::from(format!("{}-page", spec.name)))
        .track_focus(page)
        .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
            let list = access(this);
            if let Some(jobs) = list.chord(event) {
                list.push(jobs);
                cx.stop_propagation();
                cx.notify();
            }
        }))
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(move |_this, _event: &MouseDownEvent, window, cx| {
                page_handle.focus(window, cx);
            }),
        )
        .relative()
        .w(px(spec.list_at.0 + spec.list_size.0 + 14.0))
        .child(button(
            ids.write,
            "Write Params",
            (12.0, 11.0, 103.0, 19.0),
            true,
            move |this, _window, _cx| {
                let jobs = access(this).write_params();
                access(this).push(jobs);
            },
            cx,
        ))
        .child(button(
            ids.refresh,
            "Refresh Params",
            (121.0, 11.0, 103.0, 19.0),
            list.refresh_enabled(),
            move |this, _window, _cx| {
                let view = this.telemetry.view();
                let connected = view.connected && view.vehicle.is_some();
                let shown_again = this.persisted.get(SHOW_AGAIN_KEY).map(str::to_owned);
                if access(this).press_refresh(connected, &view, shown_again.as_deref()) {
                    this.telemetry.download_parameters();
                }
            },
            cx,
        ))
        .child(button(
            ids.find,
            "Find",
            (230.0, 11.0, 103.0, 19.0),
            true,
            move |this, window, cx| {
                access(this).open_find();
                prompt.focus(window, cx);
            },
            cx,
        ));

    let (list_x, list_y) = spec.list_at;
    let width = spec.control_width();
    // A panel the page stacks from y = 10, or a flow panel's three-pixel margins.
    let (x, mut y, gap) = if spec.flow {
        (FLOW_MARGIN, FLOW_MARGIN, 2.0 * FLOW_MARGIN)
    } else {
        (0.0, FIRST_Y, 0.0)
    };
    let mut dropdown_at = None;
    for (index, control) in list.controls().iter().enumerate() {
        if !control.visible {
            continue;
        }
        let height = control.height();
        let mut item = at(list_x + x, list_y + y, width, height)
            .border_b_1()
            .border_color(rgb(theme::BORDER));
        let id = format!("{}-{}", spec.name, control.name);
        match &control.kind {
            Kind::Range(range) => {
                item = item
                    .child(texts(control, width, 3.0, 0.0))
                    .child(range_number(
                        id.clone(),
                        range,
                        index,
                        list.editing == Some(index),
                        access,
                        number,
                        window,
                        cx,
                    ))
                    .child(trackbar(
                        &id,
                        range,
                        index,
                        width - 72.0,
                        access,
                        list.track_focus() == Some(index),
                        track,
                        window,
                        cx,
                    ))
                    .child(label(72.0, 90.0, range.lbl_min.clone(), true))
                    .child(label(width - 69.0, 90.0, range.lbl_max.clone(), true));
            }
            Kind::Values(combo) => {
                item = item
                    .child(texts(control, width, 4.0, 3.0))
                    .child(values_box(
                        id.clone(),
                        combo,
                        index,
                        list.typing() == Some(index),
                        list.typed_text(),
                        access,
                        number,
                        window,
                        cx,
                    ));
                if list.dropdown == Some(index) {
                    dropdown_at = Some((index, list_x + x + 3.0, list_y + y + 65.0 + 21.0));
                }
            }
            Kind::Bitmask(mask) => {
                item = item.child(texts(control, width, 3.0, 3.0));
                let (places, _) = mask.layout();
                #[allow(clippy::cast_precision_loss)]
                let lines = control.description.lines().count().max(1) as f32;
                let top = 28.0 + 5.0 + 13.0 * lines + 6.0;
                for (bit, ((_, name, checked), (bx, by))) in
                    mask.bits.iter().zip(places).enumerate()
                {
                    let mut check = Check::default();
                    check.enabled = true;
                    check.state = if *checked {
                        CheckState::Checked
                    } else {
                        CheckState::Unchecked
                    };
                    item = item.child(check_box(
                        format!("{id}-bit{bit}"),
                        &check,
                        name,
                        (bx + 3.0, top + by),
                        move |this| access(this).click_bit(index, bit),
                        cx,
                    ));
                }
            }
        }
        body = body.child(item);
        y += height + gap;
    }
    let height = list_y + y.max(spec.list_size.1) + 9.0;
    body = body.h(px(height));
    if let Some((index, x, y)) = dropdown_at
        && let Some(Control {
            kind: Kind::Values(combo),
            name,
            ..
        }) = list.controls().get(index)
    {
        body = body.child(dropdown(
            &format!("{}-{name}", spec.name),
            combo,
            (x, y, 207.0),
            move |this, key| access(this).choose(index, key),
            move |this, lines| access(this).scroll_list(index, lines),
            cx,
        ));
    }
    body
}

/// The page, laid out as `ConfigADSB.resx` lays it out: the list's buttons, `panel1` - disabled,
/// as the Designer leaves it; nothing enables it - and the controls.
pub fn page(
    adsb: &Adsb,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !adsb.is_active() {
        return div().into_any_element();
    }
    let access: Access = |this| &mut this.optional.adsb;
    let panel1 = at(12.0, 36.0, 628.0, 25.0)
        .child(label(5.0, 4.0, "Flight Identification:", false))
        .child(text_box(
            "adsb-flid",
            "",
            None,
            false,
            false,
            (109.0, 2.0, 86.0, 20.0),
            |_| {},
            |_, _| false,
            cx,
        ))
        .child(button(
            "adsb-saveflid",
            "Save",
            (201.0, 2.0, 103.0, 19.0),
            false,
            |_, _, _| {},
            cx,
        ))
        .child(label(310.0, 5.0, "Aircraft Registration", false))
        .child(text_box(
            "adsb-acreg",
            "",
            None,
            false,
            false,
            (415.0, 3.0, 100.0, 20.0),
            |_| {},
            |_, _| false,
            cx,
        ))
        .child(button(
            "adsb-saveacreg",
            "Save",
            (521.0, 3.0, 103.0, 19.0),
            false,
            |_, _, _| {},
            cx,
        ));
    let body = list_body(
        adsb,
        access,
        &focus.number,
        &focus.prompt,
        &focus.page,
        &focus.track,
        window,
        cx,
    )
    .child(panel1);
    panel(TITLE, body).into_any_element()
}

/// Find's answer kept as `InputBox` keeps it, after a key or a button that may have closed the
/// box with OK: the page object holds no settings, the window does.
/// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:24; ExtLibs/Controls/InputBox.cs:178-184`
pub fn keep_find_answer(this: &mut MissionPlanner, access: Access) {
    if let Some(input) = access(this).take_answered() {
        input.remember(&mut this.persisted);
    }
}

/// Find's box, Refresh Params' question or a message box, over the whole window.
pub fn list_overlay(
    list: &Adsb,
    access: Access,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let ids = list.spec.ids;
    if let Some(message) = list.message() {
        return Some(message_box(
            ids.message,
            ids.message_ok,
            message,
            window,
            move |this| access(this).dismiss_message(),
            cx,
        ));
    }
    if let Some(find) = &list.find {
        return Some(input_box(
            ids.find_box,
            &find.input,
            prompt,
            window,
            move |this, event| {
                let used = access(this).find_key(event, Instant::now());
                keep_find_answer(this, access);
                used
            },
            move |this| {
                access(this).close_find(true);
                keep_find_answer(this, access);
            },
            move |this| access(this).close_find(false),
            cx,
        ));
    }
    let show_again = list.confirm?;
    let mut check = Check::default();
    check.enabled = true;
    check.state = if show_again {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    let buttons = vec![
        div()
            .relative()
            .w(px(120.0))
            .h(px(20.0))
            .child(check_box(
                ids.confirm_showagain.to_owned(),
                &check,
                SHOW_ME_AGAIN,
                (0.0, 2.0),
                move |this| {
                    if let Some(value) = access(this).toggle_show_again() {
                        this.persisted.set(SHOW_AGAIN_KEY, value);
                    }
                },
                cx,
            ))
            .into_any_element(),
        action(
            ids.confirm_ok,
            "OK",
            theme::ACCENT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                let view = this.telemetry.view();
                if access(this).answer_refresh(true, &view) {
                    this.telemetry.download_parameters();
                }
                cx.notify();
            }),
        ),
        action(
            ids.confirm_cancel,
            "Cancel",
            theme::DIM,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                let view = this.telemetry.view();
                if access(this).answer_refresh(false, &view) {
                    this.telemetry.download_parameters();
                }
                cx.notify();
            }),
        ),
    ];
    Some(crate::config::servo_output::modal(
        ids.confirm,
        "Refresh Params",
        REFRESH_WARNING.trim_end(),
        true,
        buttons,
        window,
    ))
}

/// ADSB's box or question, over the whole window.
pub fn overlay(
    adsb: &Adsb,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    list_overlay(
        adsb,
        |this| &mut this.optional.adsb,
        &focus.prompt,
        window,
        cx,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use gpui::{point, size};

    /// A table that says no parameter's type: a mask is left `REAL32`, as the C# starts it.
    fn untyped(_: &str) -> Option<ParamType> {
        None
    }
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::Answering;
    use crate::telemetry::Telemetry;
    use mp_link::requests::RequestOutcome;

    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    /// The bundled documentation, with a range and an increment for `AVD_F_DIST_XY`, which it
    /// documents with neither: no `ADSB_` or `AVD_` parameter has both there.
    fn documented(name: &str) -> Option<&'static mp_params::ParamMeta> {
        static META: std::sync::OnceLock<mp_params::ParamMeta> = std::sync::OnceLock::new();
        if name == "AVD_F_DIST_XY" {
            let base = bundled(name)?;
            return Some(META.get_or_init(|| mp_params::ParamMeta {
                range: Some((0.0, 5000.0)),
                range_text: "",
                increment: Some(1.0),
                ..*base
            }));
        }
        bundled(name)
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

    fn run(jobs: Vec<Job>, link: &Answering) -> (Vec<Message>, Vec<Event>) {
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let mut messages = VecDeque::new();
        let mut events = Vec::new();
        for _ in 0..100 {
            events.extend(queue.advance(link, &mut messages));
            if queue.pending() == 0 {
                break;
            }
        }
        (messages.into_iter().collect(), events)
    }

    /// A vehicle with ADSB configured, a parameter of each kind.
    fn configured() -> Vec<(String, f64)> {
        table(&[
            ("ADSB_TYPE", 1.0),
            ("ADSB_RF_SELECT", 1.0),
            ("ADSB_LIST_MAX", 25.0),
            ("ADSB_LIST_RADIUS", 10000.0),
            ("AVD_ENABLE", 0.0),
            ("AVD_F_DIST_XY", 300.0),
            ("RTL_ALT", 1500.0),
        ])
    }

    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigADSB.resx")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("BUT_writePIDS.Text"), Some("Write Params"));
        assert_eq!(get("BUT_writePIDS.Location"), Some("12, 11"));
        assert_eq!(get("BUT_rerequestparams.Text"), Some("Refresh Params"));
        assert_eq!(get("BUT_rerequestparams.Location"), Some("121, 11"));
        assert_eq!(get("BUT_Find.Text"), Some("Find"));
        assert_eq!(get("BUT_Find.Location"), Some("230, 11"));
        assert_eq!(get("label2.Text"), Some("Flight Identification:"));
        assert_eq!(get("label1.Text"), Some("Aircraft Registration"));
        assert_eq!(get("panel1.Enabled"), Some("False"));
        assert_eq!(get("tableLayoutPanel1.Location"), Some("12, 67"));
        assert_eq!(get("tableLayoutPanel1.Size"), Some("628, 154"));
    }

    #[test]
    fn numbers_are_written_to_three_places() {
        assert_eq!(three_places(1.0), "1");
        assert_eq!(three_places(0.1_f32.into()), "0.1");
        assert_eq!(three_places(2.0005), "2.001");
        assert_eq!(three_places(-0.0001), "0");
        assert_eq!(three_places(16_777_215.0), "16777215");
    }

    #[test]
    fn names_sort_as_the_culture_sorts_them() {
        // The underscore before the letters, where ordinal order puts it after.
        assert_eq!(culture_cmp("A_B", "AB"), Ordering::Less);
        assert_eq!("A_B".cmp("AB"), Ordering::Greater);
        assert_eq!(culture_cmp("ADSB_1", "ADSB_A"), Ordering::Less);
        let mut names = vec![
            "ADSB_TYPE".to_owned(),
            "AVD_ENABLE".to_owned(),
            "ADSB_LIST_MAX".to_owned(),
            "ADSB_ENABLE".to_owned(),
        ];
        sort_enable(&mut names);
        assert_eq!(
            names,
            ["ADSB_ENABLE", "AVD_ENABLE", "ADSB_LIST_MAX", "ADSB_TYPE"]
        );
    }

    #[test]
    fn a_description_breaks_every_width_over_forty_words() {
        let words: Vec<String> = (0..20).map(|n| format!("w{n}")).collect();
        let text = fit_description("m", &words.join(" "), 628);
        let lines: Vec<&str> = text.split("\r\n").collect();
        assert_eq!(lines[0], "Units: m");
        // 628 / 40 is 15: a break after the fifteenth word, index 15.
        assert!(lines[1].starts_with("Description: w0 "));
        assert!(lines[1].ends_with("w15 "));
        assert_eq!(fit_description("", "", 628), "");
    }

    /// SITL's copter: the two enable parameters, each a list of values.
    #[test]
    fn the_sitl_copter_gets_two_value_lists() {
        let mut page = Adsb::default();
        let parameters = table(&[("ADSB_TYPE", 0.0), ("AVD_ENABLE", 0.0), ("RTL_ALT", 1500.0)]);
        let jobs = page.activate(&parameters, &untyped, key(), bundled, &[]);
        assert!(jobs.is_empty());
        let names: Vec<&str> = page.controls().iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["ADSB_TYPE", "AVD_ENABLE"]);
        let adsb_type = page.control("ADSB_TYPE").expect("ADSB_TYPE");
        assert_eq!(adsb_type.kind_name(), "values");
        assert_eq!(adsb_type.shown(), "Disabled");
        assert_eq!(adsb_type.label, "ADSB Type (ADSB_TYPE)");
        assert!(
            adsb_type
                .description
                .starts_with("Description: Type of ADS-B")
        );
        assert_eq!(
            page.control("AVD_ENABLE").map(Control::shown).as_deref(),
            Some("Disabled")
        );
    }

    /// Each kind from the documentation, and none for a range with no increment.
    #[test]
    fn each_parameter_gets_the_control_its_documentation_makes() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), documented, &[]);
        assert_eq!(
            page.control("ADSB_RF_SELECT").map(Control::kind_name),
            Some("bitmask")
        );
        assert!(
            page.control("ADSB_LIST_MAX").is_none(),
            "a range with no increment"
        );
        let avd = page.control("AVD_F_DIST_XY").expect("a range");
        assert_eq!(avd.kind_name(), "range");
        let Kind::Range(range) = &avd.kind else {
            panic!("a range");
        };
        assert_eq!(range.value_text(), "300");
        // An increment of 1 is `"1".Length - 1`, no places.
        assert_eq!(range.decimals, 0);
        assert_eq!(range.shown(), "300");
        // Favourites first, then by name.
        let mut page = Adsb::default();
        page.activate(
            &configured(),
            &untyped,
            key(),
            bundled,
            &["AVD_ENABLE".to_owned()],
        );
        assert_eq!(
            page.controls().first().map(|c| c.name.as_str()),
            Some("AVD_ENABLE")
        );
    }

    /// A changed value is recorded, not written; Write Params writes it, `ENABLE`s first, and
    /// says so; the record is cleared.
    #[test]
    fn write_params_writes_what_changed() {
        let mut page = Adsb::default();
        let parameters = configured();
        page.activate(&parameters, &untyped, key(), bundled, &[]);
        let type_index = page
            .controls()
            .iter()
            .position(|c| c.name == "ADSB_TYPE")
            .unwrap_or(0);
        let avd_index = page
            .controls()
            .iter()
            .position(|c| c.name == "AVD_ENABLE")
            .unwrap_or(0);
        page.toggle_dropdown(type_index);
        page.choose(type_index, 0);
        page.choose(avd_index, 1);
        page.choose(avd_index, 1);
        assert_eq!(page.changed().len(), 2);
        assert_eq!(
            page.changed().get("AVD_ENABLE").map(String::as_str),
            Some("1")
        );
        let jobs = page.write_params();
        let link = Answering::new(&[]);
        let (messages, events) = run(jobs, &link);
        assert!(messages.is_empty());
        assert_eq!(
            link.taken(),
            [
                ("AVD_ENABLE".to_owned(), 1.0),
                ("ADSB_TYPE".to_owned(), 0.0)
            ]
        );
        for event in events {
            if let Event::Done {
                tag: "write",
                threw,
            } = event
            {
                page.written(threw);
            }
        }
        assert!(page.changed().is_empty());
        assert_eq!(page.message().map(|m| m.text.as_str()), Some(SAVED));
        assert_eq!(page.message().map(|m| m.title), Some("Saved"));
    }

    /// A write that times out: its own box, the rest still written, no "saved", and the record
    /// kept.
    #[test]
    fn a_failed_write_keeps_the_record() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        let type_index = page
            .controls()
            .iter()
            .position(|c| c.name == "ADSB_TYPE")
            .unwrap_or(0);
        let avd_index = page
            .controls()
            .iter()
            .position(|c| c.name == "AVD_ENABLE")
            .unwrap_or(0);
        page.choose(type_index, 2);
        page.choose(avd_index, 1);
        let link = Answering::new(&[("AVD_ENABLE", Progress::Finished(RequestOutcome::TimedOut))]);
        let (messages, events) = run(page.write_params(), &link);
        assert_eq!(messages, [error("Set AVD_ENABLE Failed")]);
        assert_eq!(link.taken().len(), 2, "ADSB_TYPE after it");
        for event in events {
            if let Event::Done {
                tag: "write",
                threw,
            } = event
            {
                assert!(threw);
                page.written(threw);
            }
        }
        assert_eq!(page.changed().len(), 2);
        assert!(page.message().is_none());
    }

    /// The owner's ruling of 2026-09-25: Write Params' "Set X Failed" (`ConfigADSB.cs:197`) is
    /// the status line's, not a box - through the page's own tick, as the application runs it -
    /// and with a write failed there is no "saved" either. Standard and Advanced Params are this
    /// page object, so this holds for them too.
    #[test]
    fn a_timed_out_write_is_a_status_line_not_a_box() {
        let view = TelemetryView::disconnected("test");
        let mut page = Adsb::<usize>::new(&ADSB);
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        let avd_index = page
            .controls()
            .iter()
            .position(|c| c.name == "AVD_ENABLE")
            .unwrap_or(0);
        page.choose(avd_index, 1);
        let jobs = page.write_params();
        page.push(jobs);
        let link = Answering::new(&[("AVD_ENABLE", Progress::Finished(RequestOutcome::TimedOut))]);
        let now = Instant::now();
        for _ in 0..10 {
            page.tick(&link, &view, true, now);
        }
        assert_eq!(page.queue.pending(), 0);
        assert!(page.message().is_none(), "no box: {:?}", page.message());
        assert_eq!(page.take_status().as_deref(), Some("Set AVD_ENABLE Failed"));
        assert_eq!(page.take_status(), None, "taken once");
        assert_eq!(page.changed().len(), 1, "the record kept");

        // Written, the report of success is still a box.
        let link = Answering::new(&[]);
        let jobs = page.write_params();
        page.push(jobs);
        for _ in 0..10 {
            page.tick(&link, &view, true, now);
        }
        assert_eq!(page.message().map(|m| m.text.as_str()), Some(SAVED));
        assert_eq!(page.take_status(), None);
    }

    /// The range: arrows step by the increment, the track bar pages by five thousandths, typing
    /// is held to the bounds, and each is recorded as the float the value makes.
    #[test]
    #[allow(clippy::cast_possible_truncation)] // the floats the control writes
    fn a_range_steps_pages_and_types() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), documented, &[]);
        let index = page
            .controls()
            .iter()
            .position(|c| c.name == "AVD_F_DIST_XY")
            .unwrap_or(0);
        page.step(index, true);
        assert_eq!(
            page.changed().get("AVD_F_DIST_XY").map(String::as_str),
            Some("301")
        );
        let Some(Kind::Range(range)) = page.controls().get(index).map(|c| &c.kind) else {
            panic!("a range");
        };
        let (minimum, maximum) = (range.minimum, range.maximum);
        let before = range.trackbar;
        page.page_trackbar(index, true);
        let Some(Kind::Range(range)) = page.controls().get(index).map(|c| &c.kind) else {
            panic!("a range");
        };
        assert_eq!(range.trackbar, before + TRACK_PAGE);
        let expected = map(
            f64::from(before + TRACK_PAGE),
            0.0,
            1000.0,
            minimum,
            maximum,
        )
        .map(round_even)
        .unwrap_or(0.0);
        assert_eq!(range.value_text(), float_text(expected as f32));
        page.begin(index);
        if let Some(range) = page.range_mut(index) {
            range.type_text("999999999");
        }
        page.leave();
        let Some(Kind::Range(range)) = page.controls().get(index).map(|c| &c.kind) else {
            panic!("a range");
        };
        assert_eq!(
            range.value_text(),
            float_text(maximum as f32),
            "held to the maximum"
        );
    }

    /// A value the vehicle holds outside the documented range widens the box's bounds and turns
    /// it orange.
    #[test]
    fn an_out_of_range_value_widens_and_is_orange() {
        let range = RangeControl::new(1.0, 1.0, 0.0, 100.0, "150").expect("a control");
        assert!((range.maximum - 150.0).abs() < 1e-9);
        assert_eq!(range.lbl_max, "150");
        assert!(range.orange);
        assert_eq!(range.trackbar, 1000);
        let inside = RangeControl::new(0.01, 1.0, 0.0, 1.0, "0.5").expect("a control");
        // "0.01".Length - 1: three places.
        assert_eq!(inside.decimals, 3);
        assert_eq!(inside.shown(), "0.500");
        assert!(!inside.orange);
        assert!(
            RangeControl::new(1.0, 1.0, 5.0, 5.0, "5").is_none(),
            "an empty range throws"
        );
    }

    fn press(key: &str, control: bool) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers {
                    control,
                    ..gpui::Modifiers::default()
                },
                key: key.to_owned(),
                key_char: None,
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    /// The track bar's thumb dragged along its channel: the bar follows the pointer to the nearest
    /// of its thousand and the value with it, recorded as each `ValueChanged`; nothing moves while
    /// the thumb is not held. With the bar's focus the arrows step it `SmallChange`, Page Up and
    /// Page Down `LargeChange`, Home and End take it to its ends - a Win32 trackbar's keys.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:228-233; RangeControl.Designer.cs:53-58`
    #[test]
    #[allow(clippy::cast_possible_truncation)] // the C#'s float
    fn a_thumb_dragged_and_the_bars_keys_move_the_value() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), documented, &[]);
        let index = page
            .controls()
            .iter()
            .position(|c| c.name == "AVD_F_DIST_XY")
            .unwrap_or(0);
        let range_of = |page: &Adsb| match page.controls().get(index).map(|c| &c.kind) {
            Some(Kind::Range(range)) => (range.trackbar, range.minimum, range.maximum),
            _ => panic!("a range"),
        };
        let (_, minimum, maximum) = range_of(&page);
        // The bar as the painter laid it out: 300 wide from x = 100.
        let laid_out = Bounds {
            origin: point(px(100.0), px(50.0)),
            size: size(px(300.0), px(20.0)),
        };
        page.range_mut(index)
            .expect("a range")
            .bounds
            .set(Some(laid_out));
        // Not held: the pointer over the bar moves nothing.
        page.drag_thumb(point(px(250.0), px(60.0)));
        assert!(page.changed().is_empty());
        page.grab_thumb(index);
        assert_eq!(page.dragging(), Some(index));
        assert_eq!(page.track_focus(), Some(index));
        // The thumb's centre at the channel's middle: half its width in, then half the travel.
        page.drag_thumb(point(px(100.0 + 5.0 + 145.0), px(60.0)));
        assert_eq!(range_of(&page).0, 500);
        let expected = map(500.0, 0.0, 1000.0, minimum, maximum)
            .map(round_even)
            .unwrap_or(0.0);
        assert_eq!(
            page.changed().get("AVD_F_DIST_XY").map(String::as_str),
            Some(float_text(expected as f32).as_str())
        );
        // Past the end: held to it.
        page.drag_thumb(point(px(900.0), px(60.0)));
        assert_eq!(range_of(&page).0, 1000);
        page.release_thumb();
        assert_eq!(page.dragging(), None);
        page.drag_thumb(point(px(100.0), px(60.0)));
        assert_eq!(range_of(&page).0, 1000, "let go: the bar stays");
        // The keys.
        assert!(page.trackbar_key(&press("left", false)));
        assert_eq!(range_of(&page).0, 1000 - TRACK_SMALL);
        assert!(page.trackbar_key(&press("pagedown", false)));
        assert_eq!(range_of(&page).0, 1000, "held to the maximum");
        assert!(page.trackbar_key(&press("home", false)));
        assert_eq!(range_of(&page).0, 0);
        assert!(page.trackbar_key(&press("right", false)));
        assert_eq!(range_of(&page).0, TRACK_SMALL);
        assert!(page.trackbar_key(&press("pageup", false)));
        assert_eq!(range_of(&page).0, 0);
        assert!(page.trackbar_key(&press("end", false)));
        assert_eq!(range_of(&page).0, 1000);
        assert!(!page.trackbar_key(&press("a", false)), "not a bar's key");
        // Leaving the page takes the keyboard from the bar.
        page.deactivate();
        assert_eq!(page.track_focus(), None);
        assert!(!page.trackbar_key(&press("home", false)));
    }

    /// The values box typed into, a `DropDown` combo: the row whose text is typed, case aside,
    /// is selected and recorded; text that names no row leaves none selected and puts the C#'s
    /// `NullReferenceException` on the status line; leaving the box ends the typing.
    /// `// C#: ExtLibs/Controls/ValuesControl.cs:33-41, 68-72`
    #[test]
    fn typing_into_a_values_box_selects_the_row_it_names() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        let index = page
            .controls()
            .iter()
            .position(|c| c.name == "AVD_ENABLE")
            .expect("AVD_ENABLE");
        assert_eq!(page.controls()[index].kind_name(), "values");
        assert_eq!(page.controls()[index].shown(), "Disabled");
        page.begin_typing(index);
        assert_eq!(page.typing(), Some(index));
        assert_eq!(page.typed_text(), Some("Disabled"));
        page.type_into("enabled");
        assert_eq!(page.controls()[index].shown(), "Enabled");
        assert_eq!(
            page.changed().get("AVD_ENABLE").map(String::as_str),
            Some("1")
        );
        assert_eq!(page.take_status(), None);
        page.type_into("enabledx");
        assert_eq!(page.controls()[index].shown(), "", "no row selected");
        assert_eq!(page.take_status().as_deref(), Some(NULL_REFERENCE));
        assert_eq!(
            page.changed().get("AVD_ENABLE").map(String::as_str),
            Some("1"),
            "nothing recorded for no row"
        );
        page.type_into("Disabled");
        assert_eq!(
            page.changed().get("AVD_ENABLE").map(String::as_str),
            Some("0")
        );
        page.leave();
        assert_eq!(page.typing(), None);
        // A number clicked into ends the typing too.
        page.begin_typing(index);
        page.begin(0);
        assert_eq!(page.typing(), None);
    }

    /// `ProcessCmdKey`: Ctrl+S is Write Params, with its jobs - with nothing changed, the write
    /// of nothing that `BUT_writePIDS_Click` ends with "Parameters successfully saved."; S alone
    /// is not.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:159-168, 179-205`
    #[test]
    fn ctrl_s_is_write_params() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        let nothing = page.chord(&press("s", true)).expect("Write Params");
        assert_eq!(nothing.len(), 1);
        assert!(nothing[0].sets.is_empty());
        let index = page
            .controls()
            .iter()
            .position(|c| c.name == "AVD_F_DIST_XY")
            .unwrap_or(0);
        page.step(index, true);
        assert!(page.chord(&press("s", false)).is_none());
        let jobs = page.chord(&press("s", true)).expect("Write Params");
        assert_eq!(jobs.len(), 1);
    }

    /// `Value` narrowed to the parameter's type as the C# casts it: an INT8 mask with bit 7 set
    /// is negative - "int8 255 = -1" - an INT16's bit 15, an INT32's bit 31; a REAL32's or an
    /// unsigned type's is the sum as it is.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:21-43`
    #[test]
    fn a_bitmask_is_narrowed_to_its_parameters_type() {
        let bits: &[(u32, &str)] = &[(0, "a"), (7, "b"), (15, "c"), (31, "d")];
        let value = |kind: ParamType| {
            let mut mask = Bitmask::new(bits, 0.0, kind);
            for bit in &mut mask.bits {
                bit.2 = true;
            }
            mask.value()
        };
        assert_eq!(value(ParamType::Int8), -127.0);
        assert_eq!(value(ParamType::Int16), -32_639.0);
        assert_eq!(value(ParamType::Int32), -2_147_450_751.0);
        assert_eq!(value(ParamType::Uint8), 2_147_516_545.0);
        assert_eq!(value(ParamType::Real32), 2_147_516_545.0);
        let eight: &[(u32, &str)] = &[
            (0, "a"),
            (1, "b"),
            (2, "c"),
            (3, "d"),
            (4, "e"),
            (5, "f"),
            (6, "g"),
            (7, "h"),
        ];
        assert_eq!(Bitmask::new(eight, 255.0, ParamType::Int8).value(), -1.0);
        assert_eq!(Bitmask::new(eight, 255.0, ParamType::Uint8).value(), 255.0);
        // The vehicle's -1 for an INT8 mask reads back as all eight bits.
        let mask = Bitmask::new(eight, -1.0, ParamType::Int8);
        assert!(mask.bits.iter().all(|(_, _, checked)| *checked));
    }

    /// Showing the page again takes the vehicle's values; a bitmask whose bits changed writes
    /// each change as it makes it.
    #[test]
    fn showing_again_takes_the_vehicles_values_and_a_bitmask_writes() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        let mut later = configured();
        for (name, value) in &mut later {
            match name.as_str() {
                "ADSB_TYPE" => *value = 2.0,
                "ADSB_RF_SELECT" => *value = 2.0,
                _ => {}
            }
        }
        let jobs = page.activate(&later, &untyped, key(), bundled, &[]);
        assert_eq!(
            page.control("ADSB_TYPE").map(Control::shown).as_deref(),
            Some("Sagetech")
        );
        let link = Answering::new(&[]);
        run(jobs, &link);
        // Bit 0 off (0), then bit 1 on (2).
        assert_eq!(
            link.taken(),
            [
                ("ADSB_RF_SELECT".to_owned(), 0.0),
                ("ADSB_RF_SELECT".to_owned(), 2.0)
            ]
        );
        assert!(page.changed().is_empty(), "nothing recorded");
    }

    /// Find filters two letters or none, as typed after half a second, and on OK; Cancel shows
    /// every control and puts the word back.
    #[test]
    fn find_filters_by_name_and_description() {
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        let total = page.controls().len();
        let now = Instant::now();
        page.open_find();
        page.type_find("rf", now);
        let telemetry = Telemetry::idle();
        let view = TelemetryView::disconnected("test");
        page.active = true;
        page.tick(&telemetry, &view, true, now);
        assert_eq!(
            page.controls().iter().filter(|c| c.visible).count(),
            total,
            "not yet"
        );
        page.made_for = Some(Key::of(&view));
        page.tick(&telemetry, &view, true, now + FILTER_DELAY);
        let shown: Vec<&str> = page
            .controls()
            .iter()
            .filter(|c| c.visible)
            .map(|c| c.name.as_str())
            .collect();
        assert!(shown.contains(&"ADSB_RF_SELECT"));
        assert!(shown.len() < total);
        page.type_find("r", now);
        page.close_find(true);
        assert_eq!(page.search, "r");
        assert!(
            page.controls().iter().filter(|c| c.visible).count() < total,
            "one letter changes nothing"
        );
        page.open_find();
        page.type_find("avd", now);
        page.close_find(false);
        assert_eq!(page.search, "r", "Cancel puts the word back");
        assert_eq!(page.controls().iter().filter(|c| c.visible).count(), total);
    }

    /// Find's OK keeps the word as `InputBox` keeps every titled answer, under the caption and
    /// question with all but letters and digits taken out; Cancel keeps nothing. The key is
    /// ADSB's, Standard Params' and Advanced Params' alike: the three ask the same question.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:24; ConfigFriendlyParams.cs:24;
    /// ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    #[test]
    fn find_ok_keeps_the_answer_under_the_input_box_key() {
        let key_name =
            crate::config::optional::answers_key("Search For", "Enter a single word to search for");
        assert_eq!(key_name, "InputBoxSearchForEnterasinglewordtosearchfor");
        assert!(crate::settings::PUBLISHED.contains(&key_name.as_str()));
        let now = Instant::now();
        for spec in [
            &ADSB,
            &crate::config::friendly_params::STANDARD,
            &crate::config::friendly_params::ADVANCED,
        ] {
            let mut settings = crate::settings::Persisted::at(None);
            let mut page: Adsb = Adsb::new(spec);
            page.open_find();
            page.type_find("avd", now);
            page.close_find(false);
            assert!(page.take_answered().is_none(), "Cancel keeps nothing");
            page.open_find();
            page.type_find("rf select", now);
            page.close_find(true);
            let answered = page.take_answered().expect("OK's box");
            assert_eq!(answered.field.value(), "rf select");
            answered.remember(&mut settings);
            assert!(page.take_answered().is_none(), "kept once");
            assert_eq!(
                settings.get(&key_name),
                Some("rf+select"),
                "{}",
                spec.ids.find_box
            );
        }
    }

    #[test]
    fn refresh_asks_first_and_fetches_on_ok() {
        let telemetry = Telemetry::idle();
        let view = TelemetryView::disconnected("test");
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        assert!(!page.press_refresh(false, &view, None));
        assert!(page.confirm.is_none(), "no link, no question");
        assert!(!page.press_refresh(true, &view, None), "asks first");
        assert_eq!(page.confirm, Some(true));
        assert!(!page.answer_refresh(false, &view));
        assert!(
            page.confirm.is_none() && page.refresh_enabled(),
            "Cancel does nothing"
        );
        assert!(!page.press_refresh(true, &view, Some("True")));
        assert_eq!(page.toggle_show_again(), Some("False"));
        assert!(page.answer_refresh(true, &view), "OK fetches");
        assert!(!page.refresh_enabled(), "fetching");
        // The link went: the fetch fails, and says so on the status line, not in the C#'s box
        // (`ConfigADSB.cs:228`; the owner's ruling of 2026-09-25).
        page.tick(&telemetry, &view, true, Instant::now());
        assert!(page.refresh_enabled());
        assert!(page.message().is_none());
        assert_eq!(
            page.take_status().as_deref(),
            Some(ERROR_RECEIVING.trim_end())
        );
    }

    /// "Show me again?" unticked is kept in `Settings.Instance` at the click, "False" - whichever
    /// button then closes the box - and Refresh Params then fetches without asking, on every
    /// page that shares the key; ticked again it is "True" and the question is asked. A value
    /// `bool.TryParse` refuses is false; no value at all asks.
    /// `// C#: GCSViews/ConfigurationView/ConfigADSB.cs:217; ConfigFriendlyParams.cs:217;
    /// Common.cs:260-270, 388-399, 445-448`
    #[test]
    fn show_again_unticked_is_kept_and_the_question_is_not_asked_again() {
        assert_eq!(SHOW_AGAIN_KEY, "SHOWAGAIN_Refresh_Params");
        assert!(crate::settings::PUBLISHED.contains(&SHOW_AGAIN_KEY));
        let view = TelemetryView::disconnected("test");
        let mut settings = crate::settings::Persisted::at(None);
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        assert!(!page.press_refresh(true, &view, settings.get(SHOW_AGAIN_KEY)));
        assert!(page.confirming(), "asked while nothing is kept");
        if let Some(value) = page.toggle_show_again() {
            settings.set(SHOW_AGAIN_KEY, value);
        }
        assert_eq!(
            settings.get(SHOW_AGAIN_KEY),
            Some("False"),
            "kept at the click"
        );
        assert!(!page.answer_refresh(false, &view), "Cancel fetches nothing");
        assert_eq!(settings.get(SHOW_AGAIN_KEY), Some("False"), "and keeps it");
        for spec in [
            &ADSB,
            &crate::config::friendly_params::STANDARD,
            &crate::config::friendly_params::ADVANCED,
        ] {
            let mut next: Adsb = Adsb::new(spec);
            next.activate(&configured(), &untyped, key(), bundled, &[]);
            assert!(
                next.press_refresh(true, &view, settings.get(SHOW_AGAIN_KEY)),
                "fetches at once"
            );
            assert!(!next.confirming(), "not asked: {}", spec.ids.refresh);
        }
        // Ticked again: "True", and asked.
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        assert!(
            page.press_refresh(true, &view, Some("maybe")),
            "not a bool: false"
        );
        let mut page = Adsb::default();
        page.activate(&configured(), &untyped, key(), bundled, &[]);
        settings.set(SHOW_AGAIN_KEY, "True");
        assert!(!page.press_refresh(true, &view, settings.get(SHOW_AGAIN_KEY)));
        assert_eq!(page.toggle_show_again(), Some("False"));
        assert_eq!(page.toggle_show_again(), Some("True"));
    }

    #[test]
    fn favourites_are_url_decoded() {
        assert_eq!(url_decode("ADSB_TYPE"), "ADSB_TYPE");
        assert_eq!(url_decode("A%5FB+C"), "A_B C");
    }

    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-adsb.gui");
        let source = include_str!("adsb.rs");
        let per_control = [".kind", ".text", ".visible", ".label"];
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.adsb.") => {
                    // Recorded as `fact("<what>")`, under the page's name.
                    let what = key.trim_start_matches("config.adsb.");
                    let recorded = source.contains(&format!("fact(\"{what}\")"))
                        || key.starts_with("config.adsb.changed.")
                        || per_control.iter().any(|suffix| key.ends_with(suffix));
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("adsb-") => {
                    // Find's OK and Cancel are the shared `InputBox`'s, named in its table.
                    let drawn = source.contains(&format!("\"{id}\""))
                        || include_str!("optional.rs").contains(&format!("\"{id}\""))
                        || id.starts_with("adsb-ADSB_")
                        || id.starts_with("adsb-AVD_");
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts > 10, "{facts} facts");
    }
}
