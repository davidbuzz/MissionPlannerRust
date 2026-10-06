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

//! Basic Tuning for a copter: `GCSViews/ConfigurationView/ConfigSimplePids.cs`, the page CONFIG's
//! list adds as "Basic Tuning" - and opens first - when the vehicle is an ArduCopter and the
//! display view shows Basic Tuning (`GCSViews/SoftwareConfig.cs:160-165`). A plane's Basic Tuning
//! is `ConfigArduplane` (`basic_tuning.rs`), a rover's `ConfigArdurover` (`rover_tuning.rs`).
//!
//! What it shows: `panel1`, filled on every `Activate` from the template `acsimplepids.xml`
//! shipped beside the executable - one `RangeControl` for each `<param>` of the template the
//! vehicle has, stacked from y = 10 - and `TXT_info` under it, "NOTE: using this interface may
//! reset some off your custom pids." until a change, then a "set NAME VALUE" line for each
//! parameter written (`ConfigSimplePids.cs:26-36, 46-120, 122-170, 172-209`;
//! `ConfigSimplePids.resx`).
//!
//! A `RangeControl` (`ExtLibs/Controls/RangeControl.cs`, its Designer) is the item's title in
//! bold, its description under it, a `NumericUpDown` at the left, a `TrackBar` of 0 to 1000
//! beside it, and the range's ends under the bar. `ProcessItem` widens the template's range to
//! the vehicle's value, takes the documented range and increment over the template's (0.01
//! without one), and `setup` gives the number the increment's decimal places
//! (`Increment.ToString().Length - 1`), the range as its bounds - widened once more to the
//! value, which also widens the labels - and the value; the number is orange outside the range,
//! green inside, and the bar sits where the value is between the bounds (`RangeControl.cs:31-
//! 39, 46-64, 70-96, 121-141, 188-219`).
//!
//! A change - typed and read on Enter or when the box is left, an arrow, the bar - writes the
//! parameter at once, then each `<relation>` as the value times its multiplier, and lists them
//! in `TXT_info`; a write that times out ends the handler with "Failed to change setting ..."
//! (`ConfigSimplePids.cs:172-209`). A whole-number increment rounds the value to an integer
//! (`RangeControl.cs:197-204`).
//!
//! Where this differs from the C#, and why:
//!
//! * `setParam` holds the UI thread until the vehicle answers; here the writes go through the
//!   link's retrying set in turn, and the "set" lines appear as each is answered;
//! * the C# attaches the number's and the bar's handlers twice (`setup` and `ProcessItem` both
//!   call `AttachEvents`), so every change writes each parameter twice; here once;
//! * the bar writes on every notch while it is dragged, one blocking `setParam` per notch; here a
//!   held thumb moves the value as it goes and writes once when it is let go (a press beside the
//!   thumb writes at once, as its one notch does);
//! * "Failed to change setting" goes on the status line, never in a box (the owner's ruling of
//!   2026-09-25, PLAN.md §12);
//! * the template is the copy this application ships (`assets/acsimplepids.xml`), read from the
//!   executable's directory in the C#; the executable's directory is a build directory here;
//! * `TXT_info` is a plain text box in the C# that nothing reads; here it is drawn, not typed
//!   into, and `panel1`, which has no `AutoScroll`, clips what does not fit as the C#'s does;
//! * the number's green and orange are a border here, where the C# colours the box's background.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::Cell;
use std::collections::VecDeque;
use std::rc::Rc;
use web_time::Instant;

use gpui::{
    AnyElement, Bounds, Context, FocusHandle, KeyDownEvent, Pixels, SharedString, Window, div,
    prelude::*, px, rgb,
};

use super::optional::{Event, Job, Outcome, Set, SetQueue, at, label};
use crate::MissionPlanner;
use crate::config::extra_setup::take_link_errors;
use crate::config::failsafe::Lookup;
use crate::config::flight_modes::ParamWriter;
use crate::config::servo_output::{
    Designer, Message, Number, NumberHandlers, decimal, number_box, value_of,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{panel, theme};

/// The page's title in CONFIG's list, `Strings.BasicTuning`.
/// `// C#: GCSViews/SoftwareConfig.cs:164; ExtLibs/Strings/Strings.resx:241-243`
pub const TITLE: &str = "Basic Tuning";

/// The class, as the lists name it.
pub const CLASS: &str = "ConfigSimplePids";

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.resx ($this.Size)`
pub const PAGE_SIZE: (f32, f32) = (563.0, 287.0);

/// `panel1`'s `Location` and `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.resx (panel1.Location, panel1.Size)`
pub const PANEL: (f32, f32, f32, f32) = (4.0, 4.0, 556.0, 218.0);

/// `TXT_info`'s `Location` and `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.resx (TXT_info.Location, TXT_info.Size)`
pub const INFO_BOX: (f32, f32, f32, f32) = (4.0, 227.0, 556.0, 58.0);

/// `TXT_info.Text` as the Designer leaves it.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.resx (TXT_info.Text)`
pub const NOTE: &str = "NOTE: using this interface may reset some off your custom pids.";

/// The template, `acsimplepids.xml`, as Mission Planner ships it.
/// `// C#: acsimplepids.xml; GCSViews/ConfigurationView/ConfigSimplePids.cs:35`
pub const TEMPLATE: &str = include_str!("../../../../assets/acsimplepids.xml");

/// Where the first control goes: `y = 10`, `Location = new Point(10, y)`.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:18, 33, 156`
pub const FIRST_Y: f32 = 10.0;

/// Every control's x.
pub const ITEM_X: f32 = 10.0;

/// A `RangeControl`'s `Size`; each control is placed `Height` under the last.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:88; ConfigSimplePids.cs:164`
pub const CONTROL_SIZE: (f32, f32) = (365.0, 108.0);

/// Where `OnPaint` draws `LabelText`, in the 9.75 pt bold font.
/// `// C#: ExtLibs/Controls/RangeControl.cs:148, 164`
pub const LABEL_AT: (f32, f32) = (3.0, 0.0);

/// Where `OnPaint` draws `DescriptionText`: x, y and the height of its rectangle, as wide as the
/// control.
/// `// C#: ExtLibs/Controls/RangeControl.cs:166-167`
pub const DESCRIPTION_AT: (f32, f32, f32) = (3.0, 15.0, 39.0);

/// `numericUpDown1`'s `Location` and `Size`.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:43-45`
pub const NUMBER_AT: (f32, f32, f32, f32) = (6.0, 58.0, 57.0, 20.0);

/// `trackBar1`'s `Location` and `Size`.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:54-57`
pub const TRACK_AT: (f32, f32, f32, f32) = (69.0, 58.0, 293.0, 45.0);

/// `LBL_min`'s `Location` and `Size`.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:75-77`
pub const MIN_LABEL_AT: (f32, f32, f32, f32) = (72.0, 90.0, 77.0, 13.0);

/// `LBL_max`'s `Location` and `Size`; its text sits at the bottom right.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:65-70`
pub const MAX_LABEL_AT: (f32, f32, f32, f32) = (296.0, 90.0, 66.0, 13.0);

/// `trackBar1.Maximum`: the bar runs from 0 to 1000 whatever the range.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:56`
pub const TRACK_MAXIMUM: i32 = 1000;

/// `trackBar1.LargeChange`: how far a press beside the thumb moves it.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:53`
pub const LARGE_CHANGE: i32 = 10;

/// `trackBar1.TickFrequency`.
/// `// C#: ExtLibs/Controls/RangeControl.Designer.cs:60`
pub const TICK_FREQUENCY: i32 = 100;

/// `incrementf`'s value before the documentation is read: what an undocumented parameter steps by.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:145`
pub const DEFAULT_INCREMENT: f32 = 0.01;

/// What the handler's `catch` shows, ahead of the exception's message.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:186-190, 199-203`
pub const FAILED_PREFIX: &str = "Failed to change setting ";

/// How far the thumb's centre stops short of each end of the bar, in pixels, as a WinForms
/// track bar keeps its thumb inside the control.
const MARGIN: f32 = 8.0;

/// The thumb's length along the bar.
const THUMB: f32 = 10.0;

// ---------------------------------------------------------------------------------------------
// The template.
// ---------------------------------------------------------------------------------------------

/// A `<relation>`: another parameter written as the item's value times a multiplier.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:230-234`
#[derive(Debug, Clone, PartialEq)]
pub struct Relation {
    /// `paramaname`.
    pub param: String,
    /// `multiplier`; 0 when the template gives none, as the field is left.
    pub multiplier: f32,
}

/// A `<param>` of the template: `configitem`.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:211-228`
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ConfigItem {
    /// `title`.
    pub title: String,
    /// `desc`.
    pub desc: String,
    /// `paramname`.
    pub param: String,
    /// `paramvalue`, which nothing reads.
    pub value: f32,
    /// `min`.
    pub min: f32,
    /// `max`.
    pub max: f32,
    /// `relations`.
    pub relations: Vec<Relation>,
}

/// `float.Parse(text, NumberFormatInfo.InvariantInfo)`.
fn number(text: &str) -> Result<f32, String> {
    text.trim()
        .parse::<f32>()
        .map_err(|_| format!("'{}' is not a number", text.trim()))
}

/// `LoadXML`: the `<param>`s under each `<ac>` of `<simple>`, in the file's order. The field
/// names are matched without case, as the C# lowers them; text is taken as it is, as
/// `ReadString` gives it. A number that does not parse is the exception `float.Parse` throws.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:46-120`
pub fn load_xml(text: &str) -> Result<Vec<ConfigItem>, String> {
    let document = roxmltree::Document::parse(text).map_err(|why| why.to_string())?;
    let root = document.root_element();
    if root.tag_name().name() != "simple" {
        return Err(format!(
            "expected <simple>, found <{}>",
            root.tag_name().name()
        ));
    }
    fn elements<'a, 'input>(
        node: roxmltree::Node<'a, 'input>,
        name: &'static str,
    ) -> impl Iterator<Item = roxmltree::Node<'a, 'input>> {
        node.children()
            .filter(move |child| child.is_element() && child.tag_name().name() == name)
    }
    let mut items = Vec::new();
    for ac in elements(root, "ac") {
        for param in elements(ac, "param") {
            let mut item = ConfigItem::default();
            for field in param.children().filter(roxmltree::Node::is_element) {
                let text = field.text().unwrap_or("");
                match field.tag_name().name().to_ascii_lowercase().as_str() {
                    "title" => item.title = text.to_owned(),
                    "desc" => item.desc = text.to_owned(),
                    "name" => item.param = text.to_owned(),
                    "value" => item.value = number(text)?,
                    "min" => item.min = number(text)?,
                    "max" => item.max = number(text)?,
                    "relation" => {
                        let mut relation = Relation {
                            param: String::new(),
                            multiplier: 0.0,
                        };
                        for part in field.children().filter(roxmltree::Node::is_element) {
                            let text = part.text().unwrap_or("");
                            match part.tag_name().name() {
                                "param" => relation.param = text.to_owned(),
                                "multiplier" => relation.multiplier = number(text)?,
                                _ => {}
                            }
                        }
                        item.relations.push(relation);
                    }
                    _ => {}
                }
            }
            items.push(item);
        }
    }
    Ok(items)
}

// ---------------------------------------------------------------------------------------------
// A RangeControl.
// ---------------------------------------------------------------------------------------------

/// `float.ToString()`: the shortest text that reads back as the number, "1" for a whole one.
#[must_use]
pub fn float_text(value: f32) -> String {
    format!("{value}")
}

/// `numericUpDown1.DecimalPlaces = _increment.ToString().Length - 1`: three for 0.01, none for
/// 1, and one for 10 - the C#'s arithmetic, as it is.
/// `// C#: ExtLibs/Controls/RangeControl.cs:34-38`
#[must_use]
pub fn decimal_places(increment: f32) -> u32 {
    u32::try_from(float_text(increment).len().saturating_sub(1)).unwrap_or(0)
}

/// `Math.Round(value, 0)`: to the nearest whole number, halves to the even one.
fn round_half_even(value: f64) -> f64 {
    let rounded = value.round();
    if (value - value.trunc()).abs() == 0.5 && rounded % 2.0 != 0.0 {
        rounded - value.signum()
    } else {
        rounded
    }
}

/// One `RangeControl` with the `configitem` it was made for as its `Tag`.
#[derive(Debug)]
pub struct Control {
    /// The template's item, its range widened to the value and then replaced by the
    /// documentation's, as `ProcessItem` leaves it.
    pub item: ConfigItem,
    /// `_minrange` and `_maxrange`: the item's range, which the number is orange outside of.
    pub range: (f32, f32),
    /// `Increment`.
    pub increment: f32,
    /// `numericUpDown1`: its bounds the range widened to the value, its places the
    /// increment's.
    pub number: Number,
    /// `trackBar1.Value`, 0 to 1000.
    pub track: i32,
    /// `LBL_min.Text`: the number's minimum.
    pub min_label: String,
    /// `LBL_max.Text`: the number's maximum.
    pub max_label: String,
    /// Whether the thumb is held.
    dragging: bool,
    /// Whether the held thumb has moved the value, so letting go writes it.
    drag_pending: bool,
    /// Where the bar was laid out.
    pub bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
}

impl Control {
    /// `ProcessItem` for a parameter the vehicle has, and the `RangeControl` it makes: the
    /// template's range widened to the value, the documented range and increment over it,
    /// `setup(param, desc, title, increment, 1, min, max, value)`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:128-154; ExtLibs/Controls/RangeControl.cs:112-134`
    #[must_use]
    pub fn new(mut item: ConfigItem, value: f32, lookup: Lookup) -> Self {
        if value < item.min {
            item.min = value;
        }
        if value > item.max {
            item.max = value;
        }
        let meta = lookup(&item.param);
        if let Some((low, high)) = meta.and_then(|meta| meta.range) {
            #[allow(clippy::cast_possible_truncation)]
            let (low, high) = (low as f32, high as f32);
            item.min = low;
            item.max = high;
        }
        #[allow(clippy::cast_possible_truncation)]
        let increment = meta
            .and_then(|meta| meta.increment)
            .map_or(DEFAULT_INCREMENT, |increment| increment as f32);
        let range = (item.min, item.max);
        // `Value`'s setter: `MinRange = Math.Min(MinRange, value)` and `MaxRange` likewise set
        // the number's bounds and the labels, then `_minrange` and `_maxrange` are put back.
        // `// C#: ExtLibs/Controls/RangeControl.cs:74-96`
        let minimum = range.0.min(value);
        let maximum = range.1.max(value);
        // `(decimal)` of each float: the C#'s bounds, value and increment are decimals of seven
        // significant digits, not the float's binary expansion.
        let mut number = Number::new(Designer {
            minimum: decimal(minimum),
            maximum: decimal(maximum),
            value: decimal(value),
            decimals: decimal_places(increment),
        });
        number.param.clone_from(&item.param);
        number.enabled = true;
        number.set_increment(decimal(increment));
        let mut control = Self {
            item,
            range,
            increment,
            number,
            track: 0,
            min_label: float_text(minimum),
            max_label: float_text(maximum),
            dragging: false,
            drag_pending: false,
            bounds: Rc::new(Cell::new(None)),
        };
        // `numericUpDown1_ValueChanged(null, null)`, with nothing yet listening for the write.
        control.settle(false);
        control
    }

    /// `numericUpDown1_ValueChanged`, less the write: the bar put where the value is - unless
    /// the bar moved the value - and a whole-number increment rounding the value to an integer
    /// when the rounded value is above the minimum.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:188-208`
    fn settle(&mut self, from_track: bool) {
        if !from_track {
            self.track = self.track_of(self.number.value());
        }
        if self.increment % 1.0 == 0.0 {
            let rounded = round_half_even(self.number.value());
            if rounded > self.number.minimum {
                self.number.set_value(rounded);
                if !from_track {
                    self.track = self.track_of(rounded);
                }
            }
        }
    }

    /// `map(value, Minimum, Maximum, 0, 1000)` cast to `int`: where on the bar a value is.
    /// A range with no width would divide by zero in the C#; the bar stays at 0 here.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:183-186, 192-193`
    #[must_use]
    pub fn track_of(&self, value: f64) -> i32 {
        let span = self.number.maximum - self.number.minimum;
        if span <= 0.0 {
            return 0;
        }
        #[allow(clippy::cast_possible_truncation)]
        let track =
            ((value - self.number.minimum) * f64::from(TRACK_MAXIMUM) / span).trunc() as i32;
        track.clamp(0, TRACK_MAXIMUM)
    }

    /// `map(trackBar1.Value, 0, 1000, Minimum, Maximum)`: the value a bar position stands for,
    /// to the decimal's precision.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:213`
    #[must_use]
    pub fn value_at_track(&self, track: i32) -> f64 {
        let span = self.number.maximum - self.number.minimum;
        f64::from(track) * span / f64::from(TRACK_MAXIMUM) + self.number.minimum
    }

    /// `numericUpDown1.BackColor == Color.Orange`: the value outside `_minrange`..`_maxrange`.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:80-83, 199-206`
    #[must_use]
    pub fn orange(&self) -> bool {
        #[allow(clippy::cast_possible_truncation)]
        let value = self.number.value() as f32;
        value < self.range.0 || value > self.range.1
    }

    /// How far along the bar the thumb is, as a fraction.
    #[must_use]
    pub fn fraction(&self) -> f32 {
        #[allow(clippy::cast_precision_loss)]
        let fraction = self.track as f32 / TRACK_MAXIMUM as f32;
        fraction
    }

    /// A press `fraction` of the way along the bar, `thumb` the thumb's length as a fraction:
    /// on the thumb it takes hold of it, beside it the thumb moves one `LargeChange` toward the
    /// press, which changes the value. Returns whether the value changed.
    pub fn press(&mut self, fraction: f32, thumb: f32) -> bool {
        let at = self.fraction();
        if (fraction - at).abs() <= thumb / 2.0 {
            self.dragging = true;
            return false;
        }
        let step = if fraction > at {
            LARGE_CHANGE
        } else {
            -LARGE_CHANGE
        };
        self.set_track(self.track + step)
    }

    /// The held thumb dragged to `fraction`: the value follows, the write waits for the release.
    /// Returns whether the value changed.
    pub fn drag(&mut self, fraction: f32) -> bool {
        if !self.dragging {
            return false;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
        let track = (fraction.clamp(0.0, 1.0) * TRACK_MAXIMUM as f32).round() as i32;
        let changed = self.set_track(track);
        if changed {
            self.drag_pending = true;
        }
        changed
    }

    /// The thumb let go: whether it moved the value while held, so the value is to be written.
    pub fn release(&mut self) -> bool {
        self.dragging = false;
        std::mem::take(&mut self.drag_pending)
    }

    /// `trackBar1_ValueChanged`: the bar at `track`, the number set from it with the bar left
    /// where it is. Returns whether the value changed.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:210-215`
    fn set_track(&mut self, track: i32) -> bool {
        let track = track.clamp(0, TRACK_MAXIMUM);
        if track == self.track {
            return false;
        }
        self.track = track;
        let value = self.value_at_track(track);
        self.number.set_value(value);
        self.settle(true);
        true
    }

    /// Where a point in the window is along the bar and the thumb's length, both as fractions of
    /// the travel; `None` before the bar is laid out.
    fn fraction_of(&self, position: gpui::Point<Pixels>) -> Option<(f32, f32)> {
        let laid_out = self.bounds.get()?;
        let along = f32::from(position.x - laid_out.origin.x);
        let travel = (f32::from(laid_out.size.width) - 2.0 * MARGIN).max(1.0);
        Some(((along - MARGIN) / travel, THUMB / travel))
    }
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// The page object.
#[derive(Debug)]
pub struct SimplePids<H = mp_link::RequestId> {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// `panel1.Controls`, in the template's order.
    controls: Vec<Control>,
    /// The number being typed into.
    editing: Option<usize>,
    /// `TXT_info.Text`.
    info: String,
    /// What the template could not give: the exception `Activate` would throw.
    template_error: Option<String>,
    /// The boxes the writes put up, which the status line takes.
    messages: VecDeque<Message>,
    /// The last write failure, for the status line, until the holder takes it.
    status: Option<String>,
    /// The handler's writes.
    queue: SetQueue<H>,
}

impl<H> Default for SimplePids<H> {
    /// `InitializeComponent`: an empty panel and the note.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            controls: Vec::new(),
            editing: None,
            info: NOTE.to_owned(),
            template_error: None,
            messages: VecDeque::new(),
            status: None,
            queue: SetQueue::default(),
        }
    }
}

impl<H: Copy> SimplePids<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The controls, in the template's order.
    #[must_use]
    pub fn controls(&self) -> &[Control] {
        &self.controls
    }

    /// The number being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<usize> {
        self.editing
    }

    /// `TXT_info.Text`.
    #[must_use]
    pub fn info(&self) -> &str {
        &self.info
    }

    /// What reading the template said, if it failed.
    #[must_use]
    pub fn template_error(&self) -> Option<&str> {
        self.template_error.as_deref()
    }

    /// The link failure the C# boxes, for the status line, once.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// How the last write ended, for the facts.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.queue.last()
    }

    /// How many writes are queued or on their way.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.pending()
    }

    /// `Activate`, every time the page is shown: every control disposed, the template read
    /// again and a control made for each item the vehicle has a parameter for. A template that
    /// does not read is the exception `Activate` would throw, kept for the status line.
    /// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.cs:26-36, 128-131`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key, lookup: Lookup) {
        self.made_for = Some(key);
        self.active = true;
        self.editing = None;
        self.controls.clear();
        match load_xml(TEMPLATE) {
            Ok(items) => {
                for item in items {
                    if let Some(value) = value_of(parameters, &item.param) {
                        #[allow(clippy::cast_possible_truncation)]
                        let value = value as f32;
                        self.controls.push(Control::new(item, value, lookup));
                    }
                }
            }
            Err(why) => {
                let words = format!("acsimplepids.xml: {why}");
                self.template_error = Some(words.clone());
                self.status = Some(words);
            }
        }
    }

    /// The page hidden - it has no `Deactivate` - which takes the focus from a number being typed
    /// into, so its text is read.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.leave(now);
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) {
            return;
        }
        self.leave(now);
        if index < self.controls.len() {
            self.editing = Some(index);
        }
    }

    /// The number being typed into loses the focus, which reads its text.
    pub fn leave(&mut self, now: Instant) {
        if let Some(index) = self.editing.take() {
            self.read(index, now);
        }
    }

    /// `ValidateEditText`: the typed text read and held to the bounds - a plain `NumericUpDown`
    /// asks no question of a value above its maximum - and a changed value written.
    fn read(&mut self, index: usize, now: Instant) {
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        if let Some(question) = control.number.commit(now) {
            control.number.answer(&question, false, now);
        }
        if control.number.flush().is_some() {
            self.value_changed(index, false);
        }
    }

    /// A key for the number being typed into: typing, Enter reading the text, the arrows
    /// stepping.
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
        let Some(control) = self.controls.get_mut(index) else {
            return false;
        };
        let (handled, question) = control.number.key(event, now);
        if let Some(question) = question {
            control.number.answer(&question, false, now);
        }
        if control.number.flush().is_some() {
            self.value_changed(index, false);
        }
        handled
    }

    /// A number's arrow: the typed text read first, then one increment.
    pub fn step(&mut self, index: usize, up: bool, now: Instant) {
        self.begin(index, now);
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        if let Some(question) = control.number.step(up, now) {
            // The text read was above the maximum: held there, then the step it interrupted.
            control.number.answer(&question, false, now);
            control.number.step(up, now);
        }
        if control.number.flush().is_some() {
            self.value_changed(index, false);
        }
    }

    /// The bar pressed at `position`: the focus leaves a number being typed into first; on the
    /// thumb the press takes hold of it, beside it one `LargeChange` changes the value and
    /// writes it.
    pub fn press(&mut self, index: usize, position: gpui::Point<Pixels>, now: Instant) {
        self.leave(now);
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        let Some((fraction, thumb)) = control.fraction_of(position) else {
            return;
        };
        if control.press(fraction, thumb) {
            self.value_changed(index, true);
        }
    }

    /// The held thumb dragged to `position`. Returns whether the value changed.
    pub fn drag(&mut self, index: usize, position: gpui::Point<Pixels>) -> bool {
        let Some(control) = self.controls.get_mut(index) else {
            return false;
        };
        let Some((fraction, _)) = control.fraction_of(position) else {
            return false;
        };
        control.drag(fraction)
    }

    /// The thumb let go: the value it moved to, written once.
    pub fn release(&mut self, index: usize) {
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        if control.release() {
            self.value_changed(index, true);
        }
    }

    /// The bar moved by a test, to `track`.
    #[cfg(test)]
    pub fn set_track(&mut self, index: usize, track: i32) {
        if let Some(control) = self.controls.get_mut(index)
            && control.set_track(track)
        {
            self.value_changed(index, true);
        }
    }

    /// Types into a number, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, index: usize, text: &str, now: Instant) {
        self.begin(index, now);
        if let Some(control) = self.controls.get_mut(index) {
            control.number.type_text(text);
        }
    }

    /// `numericUpDown1_ValueChanged` - the bar put where the value is, unless the bar moved it,
    /// and the whole-number rounding - then `RNG_ValueChanged`: `TXT_info` cleared,
    /// `setParam(name, value)`, then each relation as the value times its multiplier, all in one
    /// handler that the first timeout ends with "Failed to change setting" and the exception's
    /// words.
    /// `// C#: ExtLibs/Controls/RangeControl.cs:188-208; GCSViews/ConfigurationView/ConfigSimplePids.cs:172-209;
    /// ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1762`
    fn value_changed(&mut self, index: usize, from_track: bool) {
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        if !from_track {
            control.settle(false);
        }
        let control = &*control;
        self.info.clear();
        // `(float)Value`, to the decimal it converts to: what goes on the wire is the float
        // either way, and the facts read as the C#'s text does.
        #[allow(clippy::cast_possible_truncation)]
        let value = decimal(control.number.written() as f32);
        let failed = |param: &str| Message {
            title: "",
            text: format!("{FAILED_PREFIX}Timeout on read - setParam {param}"),
        };
        let caught = |param: &str, value: f64| Set {
            param: param.to_owned(),
            value,
            on_false: None,
            on_throw: Some(failed(param)),
        };
        let mut sets = vec![caught(&control.item.param, value)];
        for relation in &control.item.relations {
            #[allow(clippy::cast_possible_truncation)]
            let scaled = decimal(value as f32 * relation.multiplier);
            sets.push(caught(&relation.param, scaled));
        }
        self.queue.push([Job::new("RNG_ValueChanged", sets)]);
    }

    /// The writes as far as the link's answers allow: each answered call its "set NAME VALUE"
    /// line, a timeout's box to the status line.
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W) {
        for event in self.queue.advance(writer, &mut self.messages) {
            if let Event::Set {
                param,
                value,
                outcome,
            } = event
                && !matches!(outcome, Outcome::Threw)
            {
                #[allow(clippy::cast_possible_truncation)]
                let shown = float_text(value as f32);
                self.info.push_str(&format!("set {param} {shown}\r\n"));
            }
        }
        // The owner's ruling of 2026-09-25 (PLAN.md §12): a link failure the window shows as
        // state gets no box; "Failed to change setting" goes on the status line.
        if let Some(words) = take_link_errors(&mut self.messages, |message| {
            message.text.starts_with(FAILED_PREFIX)
        }) {
            self.status = Some(words);
        }
    }
}

impl SimplePids {
    /// Once a frame: a page object whose screen has gone is let go, a number the focus has left
    /// is read, and the writes.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_config: bool,
        focused: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_config || self.made_for != Some(Key::of(view)))
        {
            // Disposed with its screen; the writes on their way go on.
            self.made_for = None;
            self.controls.clear();
            self.editing = None;
            self.info = NOTE.to_owned();
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        self.advance(telemetry);
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on: whether the page shows, which items it shows, each control's
/// value, bounds, bar and colour, the info text and how the last write went.
pub fn record_facts<H: Copy>(page: &SimplePids<H>) {
    use crate::facts::record;
    record("config.simplepids.active", page.is_active());
    let items: Vec<&str> = page
        .controls()
        .iter()
        .map(|control| control.item.param.as_str())
        .collect();
    record("config.simplepids.items", items.join(","));
    record(
        "config.simplepids.info",
        page.info().replace("\r\n", " | ").trim_end_matches(" | "),
    );
    record(
        "config.simplepids.last",
        page.last_write().unwrap_or("none"),
    );
    record("config.simplepids.pending", page.pending());
    record(
        "config.simplepids.editing",
        page.editing()
            .and_then(|index| page.controls().get(index))
            .map_or("none", |control| control.item.param.as_str()),
    );
    if let Some(why) = page.template_error() {
        record("config.simplepids.error", why);
    }
    for control in page.controls() {
        let key = format!("config.simplepids.{}", control.item.param);
        record(&key, control.number.shown());
        record(format!("{key}.title"), &control.item.title);
        record(format!("{key}.min"), &control.min_label);
        record(format!("{key}.max"), &control.max_label);
        record(format!("{key}.track"), control.track);
        record(format!("{key}.decimals"), control.number.decimals);
        record(
            format!("{key}.colour"),
            if control.orange() { "orange" } else { "green" },
        );
        let relations: Vec<String> = control
            .item
            .relations
            .iter()
            .map(|relation| format!("{}*{}", relation.param, float_text(relation.multiplier)))
            .collect();
        record(format!("{key}.relations"), relations.join(","));
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// The page: `panel1` with a `RangeControl` per item, and `TXT_info` under it.
/// `// C#: GCSViews/ConfigurationView/ConfigSimplePids.Designer.cs:29-56; ConfigSimplePids.resx;
/// ExtLibs/Controls/RangeControl.Designer.cs:29-91; RangeControl.cs:159-168`
pub fn page(
    pids: &SimplePids,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut body = div().relative().w(px(PAGE_SIZE.0)).h(px(PAGE_SIZE.1));
    let (panel_x, panel_y, panel_w, panel_h) = PANEL;
    let mut panel1 =
        crate::probe::measured("simplepids-panel1", at(panel_x, panel_y, panel_w, panel_h))
            .overflow_hidden();
    for (index, control) in pids.controls().iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let y = FIRST_Y + index as f32 * CONTROL_SIZE.1;
        let id = format!("simplepids-{}", control.item.param);
        let (nx, ny, nw, nh) = NUMBER_AT;
        let colour = if control.orange() {
            theme::WARN
        } else {
            theme::OK
        };
        let range_control = crate::probe::measured(
            format!("{id}-control"),
            at(ITEM_X, y, CONTROL_SIZE.0, CONTROL_SIZE.1),
        )
        .child(
            div()
                .absolute()
                .left(px(LABEL_AT.0))
                .top(px(LABEL_AT.1))
                .text_sm()
                .font_weight(gpui::FontWeight::BOLD)
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(control.item.title.clone()),
        )
        .child(
            div()
                .absolute()
                .left(px(DESCRIPTION_AT.0))
                .top(px(DESCRIPTION_AT.1))
                .w(px(CONTROL_SIZE.0 - DESCRIPTION_AT.0))
                .h(px(DESCRIPTION_AT.2))
                .overflow_hidden()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(control.item.desc.clone()),
        )
        // `BackColor`, Green or Orange, as a border round the box.
        .child(
            at(nx - 1.0, ny - 1.0, nw + 2.0, nh + 2.0)
                .border_1()
                .border_color(rgb(colour))
                .rounded_sm(),
        )
        .child(number_box(
            id.clone(),
            &control.number,
            pids.editing() == Some(index),
            handle,
            NUMBER_AT,
            NumberHandlers {
                begin: move |this: &mut MissionPlanner| {
                    this.software_pages.simple.begin(index, Instant::now());
                },
                key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                    this.software_pages.simple.key(event, Instant::now())
                },
                step: move |this: &mut MissionPlanner, up: bool| {
                    this.software_pages.simple.step(index, up, Instant::now());
                },
            },
            window,
            cx,
        ))
        .child(track_bar(index, &id, control, cx))
        .child(label(
            MIN_LABEL_AT.0,
            MIN_LABEL_AT.1,
            control.min_label.clone(),
            true,
        ))
        .child(
            at(
                MAX_LABEL_AT.0,
                MAX_LABEL_AT.1,
                MAX_LABEL_AT.2,
                MAX_LABEL_AT.3,
            )
            .flex()
            .justify_end()
            .items_end()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child(control.max_label.clone()),
        );
        panel1 = panel1.child(range_control);
    }
    body = body.child(panel1).child(info_box(pids.info()));
    panel(TITLE, body).into_any_element()
}

/// `trackBar1`: the channel, a tick every `TickFrequency`, the thumb; a press takes hold of the
/// thumb or moves it a `LargeChange`, a drag moves it, letting go writes.
fn track_bar(
    index: usize,
    id: &str,
    control: &Control,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (left, top, width, height) = TRACK_AT;
    let laid = Rc::clone(&control.bounds);
    let travel = width - 2.0 * MARGIN;
    let along = |fraction: f32| MARGIN + fraction * travel;
    let mut channel = div().relative().size_full().child(
        gpui::canvas(
            move |laid_out, _window, _cx| laid.set(Some(laid_out)),
            |_bounds, (), _window, _cx| {},
        )
        .absolute()
        .inset_0(),
    );
    channel = channel.child(
        div()
            .absolute()
            .top(px(10.0))
            .left(px(MARGIN))
            .h(px(4.0))
            .w(px(travel))
            .rounded_sm()
            .bg(rgb(theme::BORDER)),
    );
    let mut tick = 0;
    while tick <= TRACK_MAXIMUM {
        #[allow(clippy::cast_precision_loss)]
        let fraction = tick as f32 / TRACK_MAXIMUM as f32;
        channel = channel.child(
            div()
                .absolute()
                .top(px(24.0))
                .left(px(along(fraction)))
                .w(px(1.0))
                .h(px(4.0))
                .bg(rgb(theme::DIM)),
        );
        tick += TICK_FREQUENCY;
    }
    let thumb_at = along(control.fraction());
    channel = channel.child(
        div()
            .absolute()
            .top(px(2.0))
            .left(px(thumb_at - THUMB / 2.0))
            .w(px(THUMB))
            .h(px(20.0))
            .rounded_sm()
            .bg(rgb(theme::ACCENT)),
    );
    let track_id = format!("{id}-track");
    crate::probe::measured(track_id.clone(), at(left, top, width, height))
        .id(SharedString::from(track_id))
        .cursor_pointer()
        .child(channel)
        .on_mouse_down(
            gpui::MouseButton::Left,
            cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                this.software_pages
                    .simple
                    .press(index, event.position, Instant::now());
                cx.notify();
            }),
        )
        .on_mouse_move(
            cx.listener(move |this, event: &gpui::MouseMoveEvent, _window, cx| {
                if event.pressed_button != Some(gpui::MouseButton::Left) {
                    return;
                }
                if this.software_pages.simple.drag(index, event.position) {
                    cx.notify();
                }
            }),
        )
        .on_mouse_up(
            gpui::MouseButton::Left,
            cx.listener(move |this, _event: &gpui::MouseUpEvent, _window, cx| {
                this.software_pages.simple.release(index);
                cx.notify();
            }),
        )
        .into_any_element()
}

/// `TXT_info`: the note, or the "set" lines of the last change.
fn info_box(text: &str) -> gpui::Div {
    let (x, y, width, height) = INFO_BOX;
    let info = crate::probe::measured("simplepids-info", at(x, y, width, height))
        .flex()
        .flex_col()
        .px_1()
        .overflow_hidden()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .text_xs()
        .text_color(rgb(theme::TEXT));
    // The lines in a child of the box's own size, clipped there as the C#'s multi-line TextBox
    // keeps lines past its foot: the box is what is measured, not the lines a change's "set"
    // notes added (simplepids-info 64 high in its 56, config-simple-pids.gui, 2026-10-06).
    // `// C#: GCSViews/ConfigurationView/ConfigSimplePids.resx:134, 165 (ScrollBars, Multiline)`
    let mut lines = div().flex().flex_col().min_h(px(0.0)).overflow_hidden();
    for line in text.split("\r\n").filter(|line| !line.is_empty()) {
        lines = lines.child(div().whitespace_nowrap().child(line.to_owned()));
    }
    info.child(lines)
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use gpui::{Keystroke, Modifiers};
    use mp_link::requests::RequestOutcome;
    use mp_params::{ParamMeta, UserLevel};

    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::Answering;
    use crate::config_coverage::source::{csharp, resx};

    /// A parameter's documentation, as the copter's `apm.pdef.xml` gives it.
    const fn meta(
        name: &'static str,
        range: Option<(f64, f64)>,
        increment: Option<f64>,
    ) -> ParamMeta {
        ParamMeta {
            name,
            display_name: name,
            description: "",
            units: "",
            range,
            range_text: "",
            increment,
            values: &[],
            bitmask: &[],
            user_level: UserLevel::Standard,
            reboot_required: false,
        }
    }

    /// Part of ArduCopter 4.5's documentation, and a legacy name with a whole-number increment.
    static COPTER: [ParamMeta; 3] = [
        meta("ATC_RAT_RLL_P", Some((0.01, 0.5)), Some(0.005)),
        meta("ACCEL_Z_P", None, None),
        meta("THR_MID", Some((200.0, 800.0)), Some(1.0)),
    ];

    fn copter_meta(name: &str) -> Option<&'static ParamMeta> {
        COPTER.iter().find(|meta| meta.name == name)
    }

    fn params(values: &[(&str, f64)]) -> Vec<(String, f64)> {
        values
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// A copter of today: the `ATC_RAT_*` names and nothing legacy.
    fn copter() -> Vec<(String, f64)> {
        params(&[
            ("ATC_RAT_RLL_P", 0.135),
            ("ATC_RAT_RLL_I", 0.135),
            ("ATC_RAT_PIT_P", 0.135),
            ("ATC_RAT_PIT_I", 0.135),
        ])
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("loopback"))
    }

    fn shown(parameters: &[(String, f64)]) -> SimplePids<usize> {
        let mut page = SimplePids::default();
        page.activate(parameters, key(), copter_meta);
        page
    }

    fn run(page: &mut SimplePids<usize>, link: &Answering) {
        for _ in 0..100 {
            page.advance(link);
            if page.pending() == 0 {
                break;
            }
        }
    }

    fn press(page: &mut SimplePids<usize>, key: &str) {
        let event = KeyDownEvent {
            keystroke: Keystroke {
                modifiers: Modifiers::default(),
                key: key.to_owned(),
                key_char: None,
            },
            is_held: false,
            prefer_character_input: false,
        };
        page.key(&event, Instant::now());
    }

    /// The template shipped is Mission Planner's, byte for byte less its byte-order mark, and
    /// reads as `LoadXML` reads it: six items, three with relations.
    #[test]
    fn the_template_is_the_csharps_and_reads_as_load_xml_reads_it() {
        if let Some(theirs) = csharp("acsimplepids.xml") {
            assert_eq!(
                theirs.trim_start_matches('\u{feff}').replace("\r\n", "\n"),
                TEMPLATE.replace("\r\n", "\n")
            );
        }
        let items = load_xml(TEMPLATE).expect("the template reads");
        let names: Vec<&str> = items.iter().map(|item| item.param.as_str()).collect();
        assert_eq!(
            names,
            [
                "RC_FEEL_RP",
                "RATE_RLL_P",
                "ATC_RAT_RLL_P",
                "THR_MID",
                "THR_ACCEL_P",
                "ACCEL_Z_P"
            ]
        );
        let roll = &items[2];
        assert_eq!(roll.title, "Roll/Pitch Sensitivity:");
        assert!(
            roll.desc
                .starts_with("Slide to the right if the copter is sluggish")
        );
        assert_eq!((roll.value, roll.min, roll.max), (0.15, 0.08, 0.4));
        assert_eq!(
            roll.relations,
            [
                Relation {
                    param: "ATC_RAT_RLL_I".to_owned(),
                    multiplier: 1.0
                },
                Relation {
                    param: "ATC_RAT_PIT_P".to_owned(),
                    multiplier: 1.0
                },
                Relation {
                    param: "ATC_RAT_PIT_I".to_owned(),
                    multiplier: 1.0
                },
            ]
        );
        // `ReadString` keeps the text as it is: Throttle Hover's description has its spaces.
        assert_eq!(
            items[3].desc,
            " How much throttle is needed to maintain a steady hover. "
        );
        assert_eq!(items[4].relations[0].multiplier, 2.0);
        assert_eq!(items[5].relations[0].param, "ACCEL_Z_I");
        assert!(items[0].relations.is_empty());
    }

    /// A number the template gives that does not parse is the exception `float.Parse` throws.
    #[test]
    fn a_bad_number_in_the_template_is_an_error() {
        let bad = "<simple><ac><param><name>X</name><min>zero</min></param></ac></simple>";
        assert_eq!(load_xml(bad), Err("'zero' is not a number".to_owned()));
        assert!(load_xml("<other/>").is_err());
        assert!(load_xml("not xml").is_err());
    }

    /// `ProcessItem` skips a name the vehicle lacks: a copter of today shows the one item of the
    /// six whose parameter it has, with the template's title, at the vehicle's value.
    #[test]
    fn only_items_the_vehicle_has_a_parameter_for_get_a_control() {
        let page = shown(&copter());
        assert!(page.is_active());
        let names: Vec<&str> = page
            .controls()
            .iter()
            .map(|control| control.item.param.as_str())
            .collect();
        assert_eq!(names, ["ATC_RAT_RLL_P"]);
        let control = &page.controls()[0];
        assert_eq!(control.item.title, "Roll/Pitch Sensitivity:");
        assert_eq!(control.number.text(), "0.1350");
        assert_eq!(page.info(), NOTE);
        assert!(shown(&params(&[("OTHER", 1.0)])).controls().is_empty());
    }

    /// The documented range replaces the template's, the documented increment gives the number
    /// its places (`"0.005".Length - 1`), the labels are the bounds, and the bar sits where the
    /// value is between them: (0.135 - 0.01) * 1000 / 0.49, cast to `int`.
    #[test]
    fn the_range_and_increment_are_the_documentations() {
        let page = shown(&copter());
        let control = &page.controls()[0];
        assert_eq!(control.range, (0.01, 0.5));
        assert_eq!(control.increment, 0.005);
        assert_eq!(control.number.decimals, 4);
        assert_eq!(control.number.increment(), 0.005);
        assert_eq!(control.min_label, "0.01");
        assert_eq!(control.max_label, "0.5");
        assert_eq!(control.track, 255);
        assert!(!control.orange());
        // Undocumented: the template's range widened to the value, an increment of 0.01.
        let page = shown(&params(&[("ACCEL_Z_P", 1.2), ("ACCEL_Z_I", 2.4)]));
        let control = &page.controls()[0];
        assert_eq!(control.range, (0.3, 1.2));
        assert_eq!(control.increment, DEFAULT_INCREMENT);
        assert_eq!(control.number.decimals, 3);
        assert_eq!(
            (control.min_label.as_str(), control.max_label.as_str()),
            ("0.3", "1.2")
        );
        assert_eq!(control.track, 1000);
        assert!(!control.orange());
    }

    /// A value outside the documented range widens the number's bounds and the labels, keeps
    /// the range, and is orange.
    #[test]
    fn a_value_outside_the_documented_range_widens_the_bounds_and_is_orange() {
        let page = shown(&params(&[("ATC_RAT_RLL_P", 0.6)]));
        let control = &page.controls()[0];
        assert_eq!(control.range, (0.01, 0.5));
        assert_eq!(control.number.maximum, 0.6);
        assert_eq!(control.max_label, "0.6");
        assert_eq!(control.track, 1000);
        assert!(control.orange());
    }

    /// `decimal_places` is the C#'s arithmetic on the increment's text.
    #[test]
    fn the_places_are_the_increments_text_less_one() {
        assert_eq!(decimal_places(0.01), 3);
        assert_eq!(decimal_places(0.005), 4);
        assert_eq!(decimal_places(1.0), 0);
        assert_eq!(decimal_places(0.5), 2);
        assert_eq!(decimal_places(10.0), 1);
    }

    /// A value typed and read on Enter writes the parameter and then each relation at the value
    /// times its multiplier, listing each in `TXT_info` as it is answered; the bar follows.
    #[test]
    fn a_typed_value_writes_the_parameter_and_its_relations() {
        let mut page = shown(&copter());
        let link = Answering::new(&[]);
        let now = Instant::now();
        page.type_into(0, "0.2", now);
        press(&mut page, "enter");
        assert_eq!(page.controls()[0].number.text(), "0.2000");
        assert_eq!(page.controls()[0].track, 387);
        assert_eq!(page.info(), "");
        run(&mut page, &link);
        assert_eq!(
            link.taken(),
            [
                ("ATC_RAT_RLL_P".to_owned(), 0.2),
                ("ATC_RAT_RLL_I".to_owned(), 0.2),
                ("ATC_RAT_PIT_P".to_owned(), 0.2),
                ("ATC_RAT_PIT_I".to_owned(), 0.2),
            ]
        );
        assert_eq!(
            page.info(),
            "set ATC_RAT_RLL_P 0.2\r\nset ATC_RAT_RLL_I 0.2\r\nset ATC_RAT_PIT_P 0.2\r\nset ATC_RAT_PIT_I 0.2\r\n"
        );
        assert_eq!(page.last_write(), Some("ATC_RAT_PIT_I 0.2 accepted"));
        assert!(page.take_status().is_none());
    }

    /// A relation's multiplier: Climb Sensitivity writes its I term at twice the P.
    #[test]
    fn a_relation_is_the_value_times_its_multiplier() {
        let mut page = shown(&params(&[("ACCEL_Z_P", 0.5), ("ACCEL_Z_I", 1.0)]));
        let link = Answering::new(&[]);
        page.step(0, true, Instant::now());
        run(&mut page, &link);
        assert_eq!(
            link.taken(),
            [
                ("ACCEL_Z_P".to_owned(), 0.51),
                ("ACCEL_Z_I".to_owned(), 1.02),
            ]
        );
        assert_eq!(page.info(), "set ACCEL_Z_P 0.51\r\nset ACCEL_Z_I 1.02\r\n");
    }

    /// A value typed above the maximum is held to it, as a plain `NumericUpDown` holds it, with
    /// no question; one typed that does not change the value writes nothing.
    #[test]
    fn a_typed_value_is_held_to_the_bounds_without_a_question() {
        let mut page = shown(&copter());
        let link = Answering::new(&[]);
        let now = Instant::now();
        page.type_into(0, "9", now);
        page.leave(now);
        assert_eq!(page.controls()[0].number.text(), "0.5000");
        run(&mut page, &link);
        assert_eq!(link.taken()[0], ("ATC_RAT_RLL_P".to_owned(), 0.5));
        let before = link.taken().len();
        page.type_into(0, "0.5", now);
        page.leave(now);
        run(&mut page, &link);
        assert_eq!(link.taken().len(), before);
    }

    /// An arrow steps by the increment, at once, and the bar follows.
    #[test]
    fn an_arrow_steps_by_the_increment_and_writes_at_once() {
        let mut page = shown(&copter());
        let link = Answering::new(&[]);
        page.step(0, true, Instant::now());
        assert_eq!(page.controls()[0].number.text(), "0.1400");
        assert_eq!(page.pending(), 1);
        run(&mut page, &link);
        assert_eq!(link.taken()[0], ("ATC_RAT_RLL_P".to_owned(), 0.14));
        press(&mut page, "down");
        run(&mut page, &link);
        assert_eq!(page.controls()[0].number.text(), "0.1350");
        assert_eq!(link.taken()[4], ("ATC_RAT_RLL_P".to_owned(), 0.135));
    }

    /// The bar: a press beside the thumb moves it one `LargeChange` toward the press and sets
    /// the number to `map(track, 0, 1000, Minimum, Maximum)` at once, written; a press on the
    /// thumb takes hold of it, and the value follows the drag but is written once, when the
    /// thumb is let go.
    #[test]
    fn the_bar_moves_the_value_and_a_drag_writes_when_let_go() {
        let mut page = shown(&copter());
        let link = Answering::new(&[]);
        // Beside the thumb, to the right: 255 to 265.
        assert!(page.controls[0].press(0.9, 0.05));
        assert_eq!(page.controls()[0].track, 265);
        let expected = 265.0 * 0.49 / 1000.0 + 0.01;
        assert!((page.controls()[0].number.value() - expected).abs() < 1e-12);
        page.value_changed(0, true);
        run(&mut page, &link);
        assert_eq!(link.taken().len(), 4);
        // On the thumb: held, then dragged; nothing written until it is let go.
        let at = page.controls()[0].fraction();
        assert!(!page.controls[0].press(at, 0.05));
        assert!(page.controls[0].drag(0.5));
        assert_eq!(page.controls()[0].track, 500);
        assert!(page.controls[0].drag(0.75));
        assert_eq!(page.controls()[0].track, 750);
        assert!(!page.controls[0].drag(0.75));
        assert_eq!(page.pending(), 0);
        page.release(0);
        run(&mut page, &link);
        assert_eq!(link.taken().len(), 8);
        let written = 750.0 * 0.49 / 1000.0 + 0.01;
        #[allow(clippy::cast_possible_truncation)]
        let written = decimal(written as f32);
        assert_eq!(link.taken()[4], ("ATC_RAT_RLL_P".to_owned(), written));
        // Let go without moving: nothing.
        let at = page.controls()[0].fraction();
        assert!(!page.controls[0].press(at, 0.05));
        page.release(0);
        assert_eq!(page.pending(), 0);
    }

    /// A whole-number increment rounds the bar's value to an integer, halves to the even one,
    /// when the rounded value is above the minimum.
    #[test]
    fn a_whole_number_increment_rounds_the_bars_value() {
        let mut page = shown(&params(&[("THR_MID", 480.0)]));
        assert_eq!(page.controls()[0].number.decimals, 0);
        assert_eq!(page.controls()[0].track, 466);
        // 467 of 1000 over 200..800 is 480.2: rounded to 480.
        page.set_track(0, 467);
        assert_eq!(page.controls()[0].number.value(), 480.0);
        assert_eq!(page.controls()[0].track, 467);
        // 475 is 485: whole already.
        page.set_track(0, 475);
        assert_eq!(page.controls()[0].number.value(), 485.0);
        assert_eq!(round_half_even(2.5), 2.0);
        assert_eq!(round_half_even(3.5), 4.0);
        assert_eq!(round_half_even(-2.5), -2.0);
        assert_eq!(round_half_even(2.4), 2.0);
    }

    /// A write that times out ends the handler: "Failed to change setting" and the exception's
    /// words on the status line, the relations after it not written, and no "set" line for it.
    #[test]
    fn a_timeout_ends_the_handler_with_failed_to_change_setting_on_the_status_line() {
        let mut page = shown(&copter());
        let link = Answering::new(&[(
            "ATC_RAT_PIT_P",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        page.step(0, true, Instant::now());
        run(&mut page, &link);
        assert_eq!(link.taken().len(), 3);
        assert_eq!(
            page.take_status(),
            Some("Failed to change setting Timeout on read - setParam ATC_RAT_PIT_P".to_owned())
        );
        assert_eq!(
            page.info(),
            "set ATC_RAT_RLL_P 0.14\r\nset ATC_RAT_RLL_I 0.14\r\n"
        );
        assert!(page.messages.is_empty());
    }

    /// Activated again, the controls are made afresh at the vehicle's values; hidden, a number
    /// being typed into is read.
    #[test]
    fn activate_makes_the_controls_afresh_and_hiding_reads_the_number() {
        let mut page = shown(&copter());
        let link = Answering::new(&[]);
        let now = Instant::now();
        page.type_into(0, "0.3", now);
        assert_eq!(page.editing(), Some(0));
        page.hide(now);
        assert!(!page.is_active());
        assert_eq!(page.editing(), None);
        run(&mut page, &link);
        assert_eq!(link.taken()[0], ("ATC_RAT_RLL_P".to_owned(), 0.3));
        page.activate(&params(&[("ATC_RAT_RLL_P", 0.25)]), key(), copter_meta);
        assert_eq!(page.controls()[0].number.text(), "0.2500");
        assert_eq!(page.controls()[0].item.relations.len(), 3);
    }

    /// The layout is the Designer's: the page, its panel and info box from the `.resx`, the
    /// `RangeControl`'s parts from its Designer.
    #[test]
    fn the_layout_is_the_designers() {
        let pair = |(x, y): (f32, f32)| format!("{x}, {y}");
        if let Some(text) = csharp("GCSViews/ConfigurationView/ConfigSimplePids.resx") {
            let values = resx(&text);
            assert_eq!(values["$this.Size"], pair(PAGE_SIZE));
            assert_eq!(values["panel1.Location"], pair((PANEL.0, PANEL.1)));
            assert_eq!(values["panel1.Size"], pair((PANEL.2, PANEL.3)));
            assert_eq!(values["TXT_info.Location"], pair((INFO_BOX.0, INFO_BOX.1)));
            assert_eq!(values["TXT_info.Size"], pair((INFO_BOX.2, INFO_BOX.3)));
            assert_eq!(values["TXT_info.Text"], NOTE);
        }
        if let Some(designer) = csharp("ExtLibs/Controls/RangeControl.Designer.cs") {
            let point = |(x, y, _, _): (f32, f32, f32, f32)| {
                format!("Location = new System.Drawing.Point({x}, {y});")
            };
            let size = |(_, _, w, h): (f32, f32, f32, f32)| {
                format!("Size = new System.Drawing.Size({w}, {h});")
            };
            for place in [NUMBER_AT, TRACK_AT, MIN_LABEL_AT, MAX_LABEL_AT] {
                assert!(designer.contains(&point(place)), "{}", point(place));
                assert!(designer.contains(&size(place)), "{}", size(place));
            }
            assert!(designer.contains(&format!(
                "this.Size = new System.Drawing.Size({}, {});",
                CONTROL_SIZE.0, CONTROL_SIZE.1
            )));
            assert!(designer.contains(&format!("trackBar1.Maximum = {TRACK_MAXIMUM};")));
            assert!(designer.contains(&format!("trackBar1.LargeChange = {LARGE_CHANGE};")));
            assert!(designer.contains(&format!("trackBar1.TickFrequency = {TICK_FREQUENCY};")));
        }
        if let Some(source) = csharp("GCSViews/ConfigurationView/ConfigSimplePids.cs") {
            assert!(source.contains("RNG.Location = new Point(10, y);"));
            assert!(source.contains("y = 10;"));
            assert!(source.contains("var incrementf = 0.01f;"));
            assert!(source.contains("\"Failed to change setting \" + ex.Message"));
        }
        let _ = Duration::ZERO;
    }
}
