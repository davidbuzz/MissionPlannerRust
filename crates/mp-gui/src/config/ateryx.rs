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

//! Ateryx Pids: `GCSViews/ConfigurationView/ConfigAteryx.cs`, the page CONFIG's list adds when
//! the vehicle is an Ateryx - a heartbeat from a `MAV_AUTOPILOT_GENERIC` fixed wing
//! (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6761-6769`) - with the link open
//! (`GCSViews/SoftwareConfig.cs:231-237`), beside Flight Modes and Ateryx Zero Sensors.
//!
//! What it shows, as `ConfigAteryx.resx` places it in a 975 x 587 page: thirteen group boxes of
//! plain `NumericUpDown`s, each named for the parameter it shows - the servo roll and pitch loops,
//! energy height, heading to roll, airspeed to pitch, yaw, track and orbit, the navigation angles,
//! the altimeter, take-off and landing, the throttle, the airspeeds and roll to elevator: 48 boxes
//! and 47 labels - and four buttons: Write Params, Refresh Params, Write Flash and Read Flash.
//!
//! What it does (`ConfigAteryx.cs`):
//!
//! * `Activate` disables the page unless the link is open and the firmware is Ateryx; otherwise it
//!   clears `changes` and `processToScreen` disables every box, then binds each box whose name the
//!   vehicle has: range -9000 to 9000, the value, increment 0.001 and three places - or, for a
//!   whole value not named `_P`, `_I`, `_D`, `_LOW` or `_HIGH`, increment 1 and one place - and an
//!   `_IMAX` held to 180. A value outside the range throws out of the binding, which the C#
//!   catches: the box stays disabled. `tooltips`, the static the tooltips come from, is never
//!   filled, so no box has one;
//! * a box the focus leaves raises `Validated`: its text goes into `changes` and it turns green -
//!   whether or not it was changed;
//! * Write Params sets each of `changes` through the link, each in its own `try`: a set that
//!   returns gives its box its grey back; one that throws says "Set NAME Failed". Nothing leaves
//!   `changes`, so a second Write Params writes them all again;
//! * Refresh Params downloads the parameter list and activates the page. Its "Error receiving
//!   list" is for `getParamList` throwing, which here is the link closing - the screen is shown
//!   again without the page - so it has nothing to say;
//! * Write Flash sends `MAV_CMD_PREFLIGHT_STORAGE` with 0 - unless the vehicle is flying, "Unable -
//!   UAV airborne" - and Read Flash asks "Reset Flash to Factory Defaults?" and sends it with 1.
//!   A command that goes unanswered says "The Command failed to execute".
//!
//! The Designer wires four empty handlers too - three boxes' `ValueChanged` and a group box's
//! `Enter` - which do nothing here either. Where this differs from the C#, and why, is said at the
//! site: the sets and commands go through the link's retrying requests one at a time, the page's
//! controls doing nothing until they return, as the C#'s handlers hold the UI thread; and the
//! green, grey and red of a box are rings in this application's palette.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::sync::Arc;

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::requests::RequestOutcome;

use super::optional::{
    Event, Job, Outcome, Set, SetQueue, at, button, group, label, plain, set_failed,
};
use crate::MissionPlanner;
use crate::config::flight_modes::{Firmware, ParamWriter, Progress};
use crate::config::servo_output::{Message, decimal_text, modal, value_of};
use crate::setup::Key;
use crate::telemetry::{Report, Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, panel, theme};

/// The page's title in CONFIG's list.
/// `// C#: GCSViews/SoftwareConfig.cs:237`
pub const TITLE: &str = "Ateryx Pids";

/// The class, as the lists name it.
pub const CLASS: &str = "ConfigAteryx";

/// `$this.Size`. `// C#: GCSViews/ConfigurationView/ConfigAteryx.resx ($this.Size)`
pub const PAGE_SIZE: (f32, f32) = (975.0, 587.0);

/// Every box's `Size`. `// C#: GCSViews/ConfigurationView/ConfigAteryx.resx (*.Size)`
pub const BOX_SIZE: (f32, f32) = (104.0, 22.0);

/// `MAV_CMD_PREFLIGHT_STORAGE`. `// C#: ExtLibs/Mavlink/Mavlink.cs:1103`
pub const PREFLIGHT_STORAGE: u16 = 245;

/// What the flash buttons say while the vehicle flies.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:298, 322`
pub const AIRBORNE: &str = "Unable - UAV airborne";

/// What they say when the command throws.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:308, 332`
pub const COMMAND_FAILED: &str = "The Command failed to execute";

/// Read Flash's question, caption "Continue".
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:317`
pub const RESET_FLASH: &str = "Reset Flash to Factory Defaults?";

/// `Color.FromArgb(0x43, 0x44, 0x45)`: a bound box's colour.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:147, 233`
pub const BOUND_COLOUR: u32 = 0x43_44_45;

/// A group box: its Designer name, `Text`, and `Location` and `Size`.
pub type GroupSpec = (&'static str, &'static str, (f32, f32, f32, f32));

/// The thirteen group boxes.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.resx (groupBox*.Location, .Size, .Text)`
pub const GROUPS: [GroupSpec; 13] = [
    ("groupBox3", "Throttle 0-100%", (552.0, 295.0, 260.0, 133.0)),
    ("groupBox8", "Servo Roll Pid", (16.0, 18.0, 260.0, 163.0)),
    ("groupBox1", "Servo Pitch Pid", (284.0, 18.0, 260.0, 163.0)),
    ("groupBox2", "Energy Height", (552.0, 18.0, 260.0, 188.0)),
    ("groupBox4", "Heading to Roll", (16.0, 189.0, 260.0, 163.0)),
    (
        "groupBox5",
        "Airspeed to Pitch",
        (284.0, 189.0, 260.0, 163.0),
    ),
    ("groupBox6", "Servo Yaw Pid", (552.0, 214.0, 260.0, 77.0)),
    ("groupBox7", "Track and Orbit", (16.0, 360.0, 260.0, 134.0)),
    (
        "groupBox9",
        "Navigation Angles",
        (284.0, 360.0, 260.0, 134.0),
    ),
    (
        "groupBox10",
        "Altimeter Settings",
        (16.0, 502.0, 260.0, 77.0),
    ),
    (
        "groupBox11",
        "Take Off / Landing",
        (284.0, 502.0, 260.0, 77.0),
    ),
    ("groupBox12", "Airspeed m/s", (552.0, 433.0, 260.0, 106.0)),
    ("groupBox13", "Roll to Elev", (552.0, 536.0, 260.0, 42.0)),
];

/// One `NumericUpDown`: its name - the parameter - the group it is in, its `Location` there, and
/// its label's name, text and `Location`, if it has one.
#[derive(Debug, Clone, Copy)]
pub struct Boxed {
    /// The Designer's name, which is the parameter's.
    pub name: &'static str,
    /// The group box, an index into [`GROUPS`].
    pub group: usize,
    /// Its `Location` in the group.
    pub at: (f32, f32),
    /// Its label.
    pub label: Option<(&'static str, &'static str, (f32, f32))>,
}

const fn boxed(
    name: &'static str,
    group: usize,
    at: (f32, f32),
    label: (&'static str, &'static str, (f32, f32)),
) -> Boxed {
    Boxed {
        name,
        group,
        at,
        label: Some(label),
    }
}

/// Group indices, for the table.
const THROTTLE: usize = 0;
const ROLL: usize = 1;
const PITCH: usize = 2;
const EH: usize = 3;
const HDG: usize = 4;
const ARSPD: usize = 5;
const YAW: usize = 6;
const TRACK: usize = 7;
const NAV: usize = 8;
const ALT: usize = 9;
const TAKEOFF: usize = 10;
const AIRSPEED: usize = 11;
const ROLL_TO_ELEV: usize = 12;

/// Every box, in the Designer's order.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.Designer.cs:33-141; ConfigAteryx.resx`
#[rustfmt::skip]
pub const BOXES: [Boxed; 48] = [
    boxed("THR_FS_VALUE", THROTTLE, (148.0, 101.0), ("label5", "FS Value", (8.0, 106.0))),
    boxed("THR_MAX", THROTTLE, (148.0, 73.0), ("label6", "Max", (8.0, 78.0))),
    boxed("THR_MIN", THROTTLE, (148.0, 44.0), ("label7", "Min", (8.0, 49.0))),
    boxed("TRIM_THROTTLE", THROTTLE, (148.0, 16.0), ("label8", "Cruise", (8.0, 21.0))),
    boxed("ROLL_INT_LIM", ROLL, (148.0, 131.0), ("label1", "Int Lim", (8.0, 133.0))),
    boxed("ROLL_LIM", ROLL, (148.0, 101.0), ("label49", "Effort Lim", (8.0, 106.0))),
    boxed("ROLL_RATE_KP", ROLL, (148.0, 73.0), ("label50", "D", (8.0, 78.0))),
    boxed("ROLL_KI", ROLL, (148.0, 44.0), ("label51", "I", (8.0, 49.0))),
    boxed("ROLL_KP", ROLL, (148.0, 16.0), ("label52", "P", (8.0, 21.0))),
    boxed("PITCH_INT_LIM", PITCH, (148.0, 131.0), ("label2", "Int Lim", (8.0, 133.0))),
    boxed("PITCH_LIM", PITCH, (148.0, 101.0), ("label3", "Effort Lim", (8.0, 106.0))),
    boxed("PITCH_RATE_KP", PITCH, (148.0, 73.0), ("label4", "D", (8.0, 78.0))),
    boxed("PITCH_KI", PITCH, (148.0, 44.0), ("label9", "I", (8.0, 49.0))),
    boxed("PITCH_KP", PITCH, (148.0, 16.0), ("label10", "P", (8.0, 21.0))),
    boxed("EH_RANGE", EH, (148.0, 159.0), ("label26", "EH Range", (8.0, 161.0))),
    boxed("EH_INT_LIM", EH, (148.0, 131.0), ("label11", "Int Lim", (8.0, 133.0))),
    boxed("EH_LIM", EH, (148.0, 101.0), ("label12", "Effort Lim", (8.0, 106.0))),
    boxed("EH_KD", EH, (148.0, 73.0), ("label13", "D", (8.0, 78.0))),
    boxed("EH_KI", EH, (148.0, 44.0), ("label14", "I", (8.0, 49.0))),
    boxed("EH_KP", EH, (148.0, 16.0), ("label15", "P", (8.0, 21.0))),
    boxed("HDG_INT_LIM", HDG, (148.0, 131.0), ("label16", "Int Lim", (8.0, 133.0))),
    boxed("HDG_LIM", HDG, (148.0, 101.0), ("label17", "Effort Lim", (8.0, 106.0))),
    boxed("HDG_KD", HDG, (148.0, 73.0), ("label18", "D", (8.0, 78.0))),
    boxed("HDG_KI", HDG, (148.0, 44.0), ("label19", "I", (8.0, 49.0))),
    boxed("HDG_KP", HDG, (148.0, 16.0), ("label20", "P", (8.0, 21.0))),
    boxed("ARSPD_INT_LIM", ARSPD, (148.0, 131.0), ("label21", "Int Lim", (8.0, 133.0))),
    boxed("ARSPD_LIM", ARSPD, (148.0, 101.0), ("label22", "Effort Lim", (8.0, 106.0))),
    boxed("ARSPD_KD", ARSPD, (148.0, 73.0), ("label23", "D", (8.0, 78.0))),
    boxed("ARSPD_KI", ARSPD, (148.0, 44.0), ("label24", "I", (8.0, 49.0))),
    boxed("ARSPD_KP", ARSPD, (148.0, 16.0), ("label25", "P", (8.0, 21.0))),
    boxed("YAW_RATE_LIM", YAW, (148.0, 44.0), ("label30", "Yaw Rate Limit", (8.0, 49.0))),
    boxed("YAW_RATE_KP", YAW, (148.0, 16.0), ("label31", "P", (8.0, 21.0))),
    boxed("LOITER_RD", TRACK, (148.0, 101.0), ("label28", "Default Radius", (8.0, 106.0))),
    boxed("ORBIT_GAIN", TRACK, (148.0, 73.0), ("label29", "Orbit Gain", (8.0, 78.0))),
    boxed("ENTRY_ANGLE", TRACK, (148.0, 44.0), ("label32", "Entry Angle", (8.0, 49.0))),
    boxed("XTRACK_GAIN", TRACK, (148.0, 16.0), ("label33", "Track Gain", (8.0, 21.0))),
    boxed("PITCH_L_LIM", NAV, (148.0, 101.0), ("label27", "Pitch Min", (8.0, 106.0))),
    boxed("PITCH_U_LIM", NAV, (148.0, 73.0), ("label34", "Pitch Max", (8.0, 78.0))),
    boxed("ROLL_L_LIM", NAV, (148.0, 44.0), ("label35", "Bank Min", (8.0, 49.0))),
    boxed("ROLL_U_LIM", NAV, (148.0, 16.0), ("label36", "Bank Max", (8.0, 21.0))),
    boxed("FLD_ATM_PRESS", ALT, (148.0, 44.0), ("label37", "in Hg", (8.0, 49.0))),
    boxed("FLD_ELEV", ALT, (148.0, 16.0), ("label38", "Field Elev", (8.0, 21.0))),
    boxed("LND_APCH_SPD", TAKEOFF, (148.0, 44.0), ("label39", "Approach Speed", (8.0, 49.0))),
    boxed("T_OFF_PITCH", TAKEOFF, (148.0, 16.0), ("label40", "Min TO Pitch", (8.0, 21.0))),
    boxed("ARSPD_U_LIM", AIRSPEED, (148.0, 73.0), ("label42", "Max Airspeed", (8.0, 78.0))),
    boxed("ARSPD_L_LIM", AIRSPEED, (148.0, 44.0), ("label43", "Min Airspeed", (8.0, 49.0))),
    boxed("CRUISE_ARSPD", AIRSPEED, (148.0, 16.0), ("label44", "Cruise", (8.0, 21.0))),
    Boxed { name: "ROLL_TO_ELEV", group: ROLL_TO_ELEV, at: (148.0, 12.0), label: None },
];

/// A button: its Designer name, `Text`, `Location` and `Size`, and its `Click` handler.
pub type ButtonSpec = (
    &'static str,
    &'static str,
    (f32, f32, f32, f32),
    &'static str,
);

/// The four buttons.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.resx (BUT_*); ConfigAteryx.Designer.cs:866-893`
pub const BUTTONS: [ButtonSpec; 4] = [
    (
        "BUT_writePIDS",
        "Write Params",
        (820.0, 32.0, 137.0, 23.0),
        "BUT_writePIDS_Click",
    ),
    (
        "BUT_rerequestparams",
        "Refresh Params",
        (820.0, 61.0, 137.0, 23.0),
        "BUT_rerequestparams_Click",
    ),
    (
        "BUT_write_flash",
        "Write Flash",
        (820.0, 514.0, 137.0, 23.0),
        "BUT_write_flash_Click",
    ),
    (
        "BUT_read_flash",
        "Read Flash",
        (820.0, 545.0, 137.0, 23.0),
        "BUT_read_flash_Click",
    ),
];

/// The Designer's empty handlers: wired, and doing nothing - so nothing here handles them, and the
/// tests hold them to the C#.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:274-288; ConfigAteryx.Designer.cs:288, 361, 531, 657`
#[cfg_attr(not(test), allow(dead_code))]
pub const EMPTY_HANDLERS: [(&str, &str, &str); 4] = [
    (
        "ROLL_INT_LIM",
        "ValueChanged",
        "numericUpDown1_ValueChanged",
    ),
    (
        "PITCH_INT_LIM",
        "ValueChanged",
        "numericUpDown2_ValueChanged",
    ),
    ("HDG_KI", "ValueChanged", "numericUpDown15_ValueChanged"),
    ("groupBox7", "Enter", "groupBox7_Enter"),
];

/// A box's `BackColor`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Back {
    /// The Designer's.
    Designer,
    /// `0x434445`: bound, or written.
    Bound,
    /// `Color.Green`: in `changes`.
    Green,
    /// `Color.Red`: its text would not parse.
    Red,
}

/// `(decimal)(float)value`: a float's seven significant digits.
fn to_decimal(value: f64) -> f64 {
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
    let single = value as f32;
    format!("{single:.6e}").parse().unwrap_or(0.0)
}

/// A plain `NumericUpDown`: its range, increment, places and value, the text in it, whether it is
/// enabled, and its colour.
/// `// C#: System.Windows.Forms.NumericUpDown, as ConfigAteryx.cs:119-160 sets it`
#[derive(Debug)]
pub struct Numeric {
    /// `Enabled`.
    pub enabled: bool,
    /// `Minimum`.
    pub minimum: f64,
    /// `Maximum`.
    pub maximum: f64,
    /// `Increment`.
    pub increment: f64,
    /// `DecimalPlaces`.
    pub decimals: u32,
    /// `Value`.
    value: f64,
    /// The text in the box.
    field: TextField,
    /// `UserEdit`: typed into since the text was last read.
    edited: bool,
    /// `BackColor`.
    pub back: Back,
}

impl Default for Numeric {
    /// The Designer's: 0 to 100, 0, increment 1, no places, enabled.
    fn default() -> Self {
        let mut numeric = Self {
            enabled: true,
            minimum: 0.0,
            maximum: 100.0,
            increment: 1.0,
            decimals: 0,
            value: 0.0,
            field: TextField::new(""),
            edited: false,
            back: Back::Designer,
        };
        numeric.show_value();
        numeric
    }
}

impl Numeric {
    /// `UpdateEditText`: the value to its places.
    fn show_value(&mut self) {
        self.field.set(decimal_text(self.value, self.decimals));
        self.edited = false;
    }

    /// What the box shows.
    #[must_use]
    pub fn shown(&self) -> &str {
        self.field.value()
    }

    /// The `Maximum` setter: the minimum lowered to it, the value held under it.
    fn set_maximum(&mut self, maximum: f64) {
        self.maximum = maximum;
        if self.minimum > self.maximum {
            self.minimum = self.maximum;
        }
        if self.value > self.maximum {
            self.value = self.maximum;
        }
        self.show_value();
    }

    /// The `Minimum` setter.
    fn set_minimum(&mut self, minimum: f64) {
        self.minimum = minimum;
        if self.minimum > self.maximum {
            self.maximum = self.minimum;
        }
        if self.value < self.minimum {
            self.value = self.minimum;
        }
        self.show_value();
    }

    /// The `Value` setter: a value outside the range throws `ArgumentOutOfRangeException`.
    fn set_value(&mut self, value: f64) -> Result<(), ()> {
        if value < self.minimum || value > self.maximum {
            return Err(());
        }
        self.value = value;
        self.show_value();
        Ok(())
    }

    /// `processToScreen`'s binding for a parameter the vehicle has. `Err` where the C# throws:
    /// the rest of the binding - the places, `Enabled`, the colour - is not done.
    /// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:119-159`
    fn bind(&mut self, name: &str, value: f64) -> Result<(), ()> {
        self.set_maximum(9000.0);
        self.set_minimum(-9000.0);
        self.set_value(to_decimal(value))?;
        self.increment = 0.001;
        // `Value.ToString("0.###", en-US).Contains(".")`: a fraction in the first three places.
        #[allow(clippy::cast_possible_truncation)] // within +-9000, times 1000
        let thousandths = (self.value.abs() * 1000.0).round() as i64;
        let fractional = thousandths % 1000 != 0;
        let named = ["_P", "_I", "_D", "_LOW", "_HIGH"]
            .iter()
            .any(|suffix| name.ends_with(suffix));
        if named || self.value == 0.0 || fractional {
            self.decimals = 3;
        } else {
            self.increment = 1.0;
            self.decimals = 1;
        }
        if name.ends_with("_IMAX") {
            self.set_maximum(180.0);
            self.set_minimum(-180.0);
        }
        self.enabled = true;
        self.back = Back::Bound;
        self.show_value();
        Ok(())
    }

    /// `ValidateEditText`: typed text that parses is held to the range and made the value; the
    /// box shows the value again either way.
    fn commit(&mut self) {
        if self.edited {
            let text = self.field.value().trim().replace(',', "");
            if let Ok(typed) = text.parse::<f64>() {
                self.value = typed.clamp(self.minimum, self.maximum);
            }
        }
        self.show_value();
    }

    /// `UpButton` and `DownButton`: the text read, then one increment, stopping at the bound.
    fn step(&mut self, up: bool) {
        if !self.enabled {
            return;
        }
        self.commit();
        let next = if up {
            (self.value + self.increment).min(self.maximum)
        } else {
            (self.value - self.increment).max(self.minimum)
        };
        self.value = (next * 1e9).round() / 1e9;
        self.show_value();
    }
}

/// A question the page is asking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ask {
    /// Read Flash's "Reset Flash to Factory Defaults?".
    ResetFlash,
}

/// Which flash button a command is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Flash {
    /// Write Flash: `PREFLIGHT_STORAGE` with 0.
    Write,
    /// Read Flash: `PREFLIGHT_STORAGE` with 1.
    Read,
}

/// The page object.
#[derive(Debug)]
pub struct Ateryx<H = mp_link::RequestId> {
    /// The screen the page object belongs to.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// The boxes, in [`BOXES`]' order.
    numbers: Vec<Numeric>,
    /// `changes`: each parameter and its value, in the order first validated.
    changes: Vec<(String, f64)>,
    /// The box with the focus.
    editing: Option<usize>,
    /// The question showing.
    question: Option<Ask>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// Write Params's sets.
    queue: SetQueue<H>,
    /// Refresh Params, while its download runs: the table it was asked over.
    refreshing: Option<Arc<[(String, f64)]>>,
    /// A flash button's command on its way.
    command: Option<(Flash, H)>,
    /// How many commands have been sent, by button, for the facts.
    sent: [usize; 2],
}

impl<H> Default for Ateryx<H> {
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            numbers: BOXES.iter().map(|_| Numeric::default()).collect(),
            changes: Vec::new(),
            editing: None,
            question: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
            refreshing: None,
            command: None,
            sent: [0; 2],
        }
    }
}

impl<H: Copy> Ateryx<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The page's `Enabled`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enabled
    }

    /// The boxes.
    #[must_use]
    pub fn numbers(&self) -> &[Numeric] {
        &self.numbers
    }

    /// `changes`.
    #[must_use]
    pub fn changes(&self) -> &[(String, f64)] {
        &self.changes
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// The box with the focus.
    #[must_use]
    pub const fn editing(&self) -> Option<usize> {
        self.editing
    }

    /// Whether the controls take input: enabled, and not held by a set, a download, a command
    /// or a question - the C#'s handlers hold the UI thread until they return.
    #[must_use]
    pub fn live(&self) -> bool {
        self.enabled
            && self.queue.pending() == 0
            && self.refreshing.is_none()
            && self.command.is_none()
            && self.question.is_none()
            && self.messages.is_empty()
    }

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:32-56`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        connected: bool,
        firmware: Firmware,
    ) {
        if self.made_for != Some(key) {
            self.dispose();
            self.made_for = Some(key);
        }
        self.active = true;
        self.bind(parameters, connected, firmware);
    }

    /// `Activate` itself.
    fn bind(&mut self, parameters: &[(String, f64)], connected: bool, firmware: Firmware) {
        if !connected || firmware != Firmware::Ateryx {
            self.enabled = false;
            return;
        }
        self.enabled = true;
        self.changes.clear();
        // `processToScreen`: every box disabled, then each the vehicle names bound.
        for number in &mut self.numbers {
            number.enabled = false;
        }
        for (number, spec) in self.numbers.iter_mut().zip(BOXES) {
            if let Some(value) = value_of(parameters, spec.name) {
                let _ = number.bind(spec.name, value);
            }
        }
    }

    /// The page hidden - it has no `Deactivate` - which takes the focus from its box.
    pub fn hide(&mut self) {
        self.leave();
        self.active = false;
    }

    /// The page object let go with its screen; sets on their way finish.
    fn dispose(&mut self) {
        *self = Self {
            messages: std::mem::take(&mut self.messages),
            queue: std::mem::take(&mut self.queue),
            ..Self::default()
        };
    }

    /// A box clicked into: the one with the focus leaves it first.
    pub fn begin(&mut self, index: usize) {
        if self.editing == Some(index) || !self.live() {
            return;
        }
        self.leave();
        if self.numbers.get(index).is_some_and(|number| number.enabled) {
            self.editing = Some(index);
        }
    }

    /// The focus leaves the box: its text read, then `Validated` - `changes` takes the value its
    /// text shows, and the box turns green.
    /// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:159, 191-215`
    pub fn leave(&mut self) {
        let Some(index) = self.editing.take() else {
            return;
        };
        let (Some(number), Some(spec)) = (self.numbers.get_mut(index), BOXES.get(index)) else {
            return;
        };
        number.commit();
        // `float.Parse(Text)`.
        match number.shown().parse::<f32>() {
            Ok(value) => {
                let value = f64::from(value);
                match self.changes.iter_mut().find(|(name, _)| name == spec.name) {
                    Some((_, held)) => *held = value,
                    None => self.changes.push((spec.name.to_owned(), value)),
                }
                number.back = Back::Green;
            }
            Err(_) => number.back = Back::Red,
        }
    }

    /// A key for the box with the focus: the arrows step, Enter reads the text, Tab moves on -
    /// which validates it - and the rest is typing.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
        if !self.live() {
            return false;
        }
        match event.keystroke.key.as_str() {
            "up" | "down" => {
                let up = event.keystroke.key == "up";
                if let Some(number) = self.numbers.get_mut(index) {
                    number.step(up);
                }
                return true;
            }
            "tab" => {
                self.leave();
                return true;
            }
            _ => {}
        }
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        match number.field.key(event) {
            KeyOutcome::Changed => {
                number.edited = true;
                true
            }
            KeyOutcome::Submitted => {
                number.commit();
                true
            }
            KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
        }
    }

    /// A box's arrow: the box takes the focus, and steps.
    pub fn step(&mut self, index: usize, up: bool) {
        if !self.live() {
            return;
        }
        self.begin(index);
        if let Some(number) = self.numbers.get_mut(index) {
            number.step(up);
        }
    }

    /// Types into a box, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, index: usize, text: &str) {
        self.begin(index);
        if let Some(number) = self.numbers.get_mut(index) {
            number.field.set(text);
            number.edited = true;
        }
    }

    /// Write Params clicked: the focus leaving the box validates it, then `BUT_writePIDS_Click` -
    /// every change set, each in its own `try`.
    /// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:217-245`
    pub fn press_write(&mut self) {
        if !self.live() {
            return;
        }
        self.leave();
        let sets: Vec<Set> = self
            .changes
            .iter()
            .map(|(name, value)| Set::caught(name.clone(), *value, set_failed(name)))
            .collect();
        if sets.is_empty() {
            return;
        }
        self.queue.push([Job {
            each_caught: true,
            ..Job::new("write", sets)
        }]);
    }

    /// Refresh Params: nothing without a link; else the list downloaded, and `Activate` once it
    /// is whole.
    /// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:252-272`
    pub fn refresh_params(&mut self, telemetry: &Telemetry, view: &TelemetryView) {
        if !self.live() {
            return;
        }
        self.leave();
        if !view.connected || view.vehicle.is_none() {
            return;
        }
        self.refreshing = Some(Arc::clone(&view.parameters));
        telemetry.download_parameters();
    }

    /// Write Flash or Read Flash clicked. Read Flash asks first; either refuses while the
    /// vehicle flies - Write Flash above 7 m/s through the air or 10 over the ground, Read Flash
    /// above 7 either way, in the speeds the screens show - and otherwise sends
    /// `PREFLIGHT_STORAGE`. `speeds` is `cs.airspeed` and `cs.groundspeed`.
    /// `// C#: GCSViews/ConfigurationView/ConfigAteryx.cs:290-335`
    pub fn press_flash(
        &mut self,
        flash: Flash,
        speeds: (f64, f64),
        send: impl FnOnce([f32; 7]) -> Option<H>,
    ) {
        if !self.live() {
            return;
        }
        self.leave();
        match flash {
            Flash::Write => self.flash(Flash::Write, speeds, send),
            Flash::Read => self.question = Some(Ask::ResetFlash),
        }
    }

    /// Read Flash's question answered: Yes goes on to the command.
    pub fn answer(
        &mut self,
        yes: bool,
        speeds: (f64, f64),
        send: impl FnOnce([f32; 7]) -> Option<H>,
    ) {
        if self.question.take().is_some() && yes {
            self.flash(Flash::Read, speeds, send);
        }
    }

    /// The airborne check, and the command.
    fn flash(
        &mut self,
        flash: Flash,
        (airspeed, groundspeed): (f64, f64),
        send: impl FnOnce([f32; 7]) -> Option<H>,
    ) {
        let ground_limit = match flash {
            Flash::Write => 10.0,
            Flash::Read => 7.0,
        };
        if airspeed > 7.0 || groundspeed > ground_limit {
            self.messages.push_back(plain(AIRBORNE));
            return;
        }
        let param1 = match flash {
            Flash::Write => 0.0,
            Flash::Read => 1.0,
        };
        match send([param1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) {
            Some(handle) => {
                self.command = Some((flash, handle));
                if let Some(count) = self.sent.get_mut(usize::from(flash == Flash::Read)) {
                    *count += 1;
                }
            }
            // No vehicle to send to: the send throws.
            None => self.messages.push_back(plain(COMMAND_FAILED)),
        }
    }

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Once a frame: the page object let go with its screen, the focus's box validated when the
    /// focus has gone, the download seen back, the command's answer read, and the sets moved on.
    #[allow(clippy::too_many_arguments)]
    pub fn tick<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        view: &TelemetryView,
        on_config: bool,
        focused: bool,
        (connected, firmware): (bool, Firmware),
        command_progress: impl Fn(H) -> Progress,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_config || self.made_for != Some(Key::of(view)))
        {
            self.dispose();
        }
        if self.editing.is_some() && !focused {
            self.leave();
        }
        if let Some(before) = &self.refreshing {
            let whole = !view.parameters.is_empty()
                && view.parameters.len() >= usize::from(view.parameters_expected);
            if !connected {
                self.refreshing = None;
            } else if whole && !Arc::ptr_eq(before, &view.parameters) {
                self.refreshing = None;
                self.bind(&view.parameters, connected, firmware);
            }
        }
        if let Some((_, handle)) = self.command {
            match command_progress(handle) {
                Progress::Waiting => {}
                // `doCommand` threw `TimeoutException`.
                Progress::Lost | Progress::Finished(RequestOutcome::TimedOut) => {
                    self.command = None;
                    self.messages.push_back(plain(COMMAND_FAILED));
                }
                // A refusal is `false`, which the handler does not look at.
                Progress::Finished(_) => self.command = None,
            }
        }
        self.advance(writer);
    }

    /// Write Params's sets as far as the link's answers allow: each that returns gives its box its
    /// colour back.
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W) {
        for event in self.queue.advance(writer, &mut self.messages) {
            if let Event::Set { param, outcome, .. } = event
                && outcome != Outcome::Threw
                && let Some(index) = BOXES.iter().position(|spec| spec.name == param)
                && let Some(number) = self.numbers.get_mut(index)
            {
                number.back = Back::Bound;
            }
        }
    }

    /// How many sets are on their way.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.pending()
    }
}

/// `MainV2.comPort.doCommand(sysid, compid, PREFLIGHT_STORAGE, ...)` through the link's retrying
/// request, to the vehicle being flown.
fn send_storage(telemetry: &mut Telemetry, params: [f32; 7]) -> Option<mp_link::RequestId> {
    let (_, vehicle) = telemetry.send_handle()?;
    telemetry.command(vehicle, PREFLIGHT_STORAGE, params, Report::default())
}

/// A request's progress, as the link has it.
fn request_progress(telemetry: &Telemetry, id: mp_link::RequestId) -> Progress {
    match telemetry.request(id) {
        None => Progress::Lost,
        Some(request) => request
            .outcome()
            .map_or(Progress::Waiting, Progress::Finished),
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on.
pub fn record_facts<H: Copy>(page: &Ateryx<H>, listed: bool) {
    use crate::facts::record;
    record("config.ateryx.listed", listed);
    record("config.ateryx.active", page.is_active());
    record("config.ateryx.enabled", page.enabled());
    for (number, spec) in page.numbers().iter().zip(BOXES) {
        let key = format!("config.ateryx.{}", spec.name);
        record(format!("{key}.enabled"), number.enabled);
        record(
            format!("{key}.back"),
            match number.back {
                Back::Designer => "designer",
                Back::Bound => "bound",
                Back::Green => "green",
                Back::Red => "red",
            },
        );
        record(key, number.shown());
    }
    let changes: Vec<String> = page
        .changes()
        .iter()
        .map(|(name, value)| format!("{name}={value}"))
        .collect();
    record(
        "config.ateryx.changes",
        if changes.is_empty() {
            "none".to_owned()
        } else {
            changes.join(",")
        },
    );
    record(
        "config.ateryx.question",
        page.question.map_or("none", |Ask::ResetFlash| RESET_FLASH),
    );
    record(
        "config.ateryx.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record("config.ateryx.write", page.queue.last().unwrap_or("none"));
    record("config.ateryx.writes.pending", page.pending());
    record("config.ateryx.refreshing", page.refreshing.is_some());
    record(
        "config.ateryx.command",
        match page.command {
            Some((Flash::Write, _)) => "write flash",
            Some((Flash::Read, _)) => "read flash",
            None => "none",
        },
    );
    record("config.ateryx.sent.write", page.sent[0]);
    record("config.ateryx.sent.read", page.sent[1]);
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// One box: its text - typed into, with a caret, while it has the focus - and its arrows,
/// `<id>-up` and `<id>-down`; its colour a ring; dimmed and inert while disabled.
fn numeric_box(
    index: usize,
    number: &Numeric,
    editing: bool,
    live: bool,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(spec) = BOXES.get(index).copied() else {
        return div().into_any_element();
    };
    let id = format!("ateryx-{}", spec.name);
    let enabled = number.enabled && live;
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
        text.track_focus(handle)
            .key_context("TextField")
            .cursor_text()
            .text_color(rgb(theme::TEXT))
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                if this.ateryx.key(event) {
                    cx.notify();
                }
            }))
    } else {
        let handle = handle.clone();
        text.cursor_text()
            .text_color(rgb(theme::TEXT))
            .on_click(cx.listener(move |this, _event, window, cx| {
                this.ateryx.begin(index);
                handle.focus(window, cx);
                cx.notify();
            }))
    };
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
            let handle = handle.clone();
            base.text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .on_click(cx.listener(move |this, _event, window, cx| {
                    this.ateryx.step(index, up);
                    handle.focus(window, cx);
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
    // The colours as rings: green in `changes`, red unparsed, the page's accent while focused.
    let ring = match number.back {
        Back::Green => theme::OK,
        Back::Red => theme::ALERT,
        Back::Designer | Back::Bound if focused => theme::ACCENT,
        Back::Designer | Back::Bound => theme::BORDER,
    };
    let (x, y) = spec.at;
    at(x, y, BOX_SIZE.0, BOX_SIZE.1)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(ring))
        .bg(rgb(match (enabled, number.back) {
            (false, _) => theme::PANEL,
            (true, Back::Bound) => BOUND_COLOUR,
            (true, _) => theme::ACTION,
        }))
        .child(text)
        .child(arrows)
        .into_any_element()
}

/// The page, laid out as `ConfigAteryx.resx` lays it out.
/// `// C#: GCSViews/ConfigurationView/ConfigAteryx.Designer.cs:29-906; ConfigAteryx.resx`
pub fn page(
    ateryx: &Ateryx,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let enabled = ateryx.enabled();
    let live = ateryx.live();
    let mut body = div().relative().w(px(PAGE_SIZE.0)).h(px(PAGE_SIZE.1));
    for (group_index, &(_, caption, place)) in GROUPS.iter().enumerate() {
        let mut boxes = group(place, caption, enabled);
        for (index, (number, spec)) in ateryx.numbers().iter().zip(BOXES).enumerate() {
            if spec.group != group_index {
                continue;
            }
            if let Some((_, text, (lx, ly))) = spec.label {
                boxes = boxes.child(label(lx, ly, text, enabled));
            }
            boxes = boxes.child(numeric_box(
                index,
                number,
                ateryx.editing() == Some(index),
                live,
                handle,
                window,
                cx,
            ));
        }
        body = body.child(boxes);
    }
    let [write, params, write_flash, read_flash] = BUTTONS;
    body = body
        .child(button(
            "ateryx-BUT_writePIDS",
            write.1,
            write.2,
            live,
            |this, _window, _cx| this.ateryx.press_write(),
            cx,
        ))
        .child(button(
            "ateryx-BUT_rerequestparams",
            params.1,
            params.2,
            live,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                this.ateryx.refresh_params(&this.telemetry, &view);
            },
            cx,
        ))
        .child(button(
            "ateryx-BUT_write_flash",
            write_flash.1,
            write_flash.2,
            live,
            |this, _window, _cx| {
                let speeds = this.ateryx_speeds();
                let telemetry = &mut this.telemetry;
                this.ateryx.press_flash(Flash::Write, speeds, |params| {
                    send_storage(telemetry, params)
                });
            },
            cx,
        ))
        .child(button(
            "ateryx-BUT_read_flash",
            read_flash.1,
            read_flash.2,
            live,
            |this, _window, _cx| {
                let speeds = this.ateryx_speeds();
                let telemetry = &mut this.telemetry;
                this.ateryx.press_flash(Flash::Read, speeds, |params| {
                    send_storage(telemetry, params)
                });
            },
            cx,
        ));
    panel(TITLE, body).into_any_element()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    ateryx: &Ateryx,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(message) = ateryx.message() {
        return Some(super::optional::message_box(
            "ateryx-message",
            "ateryx-message-ok",
            message,
            window,
            |this| this.ateryx.dismiss_message(),
            cx,
        ));
    }
    ateryx.question?;
    let answer = |yes: bool| {
        move |this: &mut MissionPlanner,
              _event: &(),
              _window: &mut Window,
              cx: &mut Context<MissionPlanner>| {
            let speeds = this.ateryx_speeds();
            let telemetry = &mut this.telemetry;
            this.ateryx
                .answer(yes, speeds, |params| send_storage(telemetry, params));
            cx.notify();
        }
    };
    let buttons = vec![
        action(
            "ateryx-question-yes",
            "Yes",
            theme::ACCENT,
            true,
            cx.listener(answer(true)),
        ),
        action(
            "ateryx-question-no",
            "No",
            theme::ACCENT,
            true,
            cx.listener(answer(false)),
        ),
    ];
    Some(modal(
        "ateryx-question",
        "Continue",
        RESET_FLASH,
        false,
        buttons,
        window,
    ))
}

impl MissionPlanner {
    /// `cs.airspeed` and `cs.groundspeed`: the SI speeds through the user's speed unit, as the
    /// C#'s getters multiply them.
    fn ateryx_speeds(&self) -> (f64, f64) {
        let view = self.telemetry.view();
        let units = self.planner.units();
        view.state.as_ref().map_or((0.0, 0.0), |state| {
            (
                units.to_speed(state.air_speed.0),
                units.to_speed(state.ground_speed.0),
            )
        })
    }

    /// `Activate`, when CONFIG's list shows the page.
    pub(crate) fn ateryx_activate(&mut self) {
        let view = self.telemetry.view();
        let vehicle = crate::setup::Vehicle::of(&view, self.telemetry.firmware_banner());
        self.ateryx.activate(
            &view.parameters,
            Key::of(&view),
            vehicle.connected,
            vehicle.firmware,
        );
    }

    /// Once a frame: the page object, its box's focus, its refresh, its command and its sets.
    pub(crate) fn ateryx_tick(&mut self, view: &TelemetryView, window: &Window) {
        let focused = self.ateryx_focus.is_focused(window);
        let vehicle = crate::setup::Vehicle::of(view, self.telemetry.firmware_banner());
        let state = (vehicle.connected, vehicle.firmware);
        let on_config = self.screen == crate::Screen::Config;
        let telemetry = &self.telemetry;
        self.ateryx
            .tick(telemetry, view, on_config, focused, state, |id| {
                request_progress(telemetry, id)
            });
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;
    use crate::config::optional::tests::Answering;
    use crate::config_coverage::source::{csharp, resx};
    use crate::setup::{List, Vehicle as Listed};

    /// An Ateryx's parameters for some of the page's names.
    fn ateryx_params() -> Vec<(String, f64)> {
        [
            ("ROLL_KP", 0.5),
            ("ROLL_KI", 0.02),
            ("ROLL_INT_LIM", 20.0),
            ("THR_MAX", 100.0),
            ("THR_MIN", 0.0),
            ("LOITER_RD", 60.0),
            ("XTRACK_GAIN", 12345.0),
            ("ARSPD_KP", 1.0),
        ]
        .iter()
        .map(|(name, value)| ((*name).to_owned(), *value))
        .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn index(name: &str) -> usize {
        BOXES
            .iter()
            .position(|spec| spec.name == name)
            .unwrap_or_else(|| panic!("{name} is not a box"))
    }

    fn shown() -> Ateryx<usize> {
        let mut page = Ateryx::<usize>::default();
        page.activate(&ateryx_params(), key(), true, Firmware::Ateryx);
        page
    }

    fn text(page: &Ateryx<usize>, name: &str) -> String {
        page.numbers()[index(name)].shown().to_owned()
    }

    fn pair((x, y): (f32, f32)) -> String {
        format!("{x}, {y}")
    }

    const PARENT: &str = "&gt;&gt;";

    /// Every control the Designer makes is drawn: 13 group boxes, 48 boxes, 47 labels, four
    /// buttons - and `toolTip1`, which no box has a tip in.
    #[test]
    fn every_designer_control_is_drawn() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigAteryx.Designer.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let made: BTreeSet<&str> = designer
            .lines()
            .filter_map(|line| line.trim().strip_prefix("this."))
            .filter(|line| line.contains(" = new ") && !line.starts_with("components"))
            .filter_map(|line| line.split_once(" = new "))
            .map(|(name, _)| name)
            .collect();
        let ours: BTreeSet<&str> = GROUPS
            .iter()
            .map(|(name, ..)| *name)
            .chain(BOXES.iter().map(|spec| spec.name))
            .chain(
                BOXES
                    .iter()
                    .filter_map(|spec| spec.label.map(|(name, ..)| name)),
            )
            .chain(BUTTONS.iter().map(|(name, ..)| *name))
            .chain(std::iter::once("toolTip1"))
            .collect();
        assert_eq!(made, ours);
        assert_eq!(ours.len(), 13 + 48 + 47 + 4 + 1);
    }

    /// The eight wirings: four buttons, and four empty handlers.
    #[test]
    fn every_wiring_is_handled() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigAteryx.Designer.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for (name, _, _, handler) in BUTTONS {
            let line = format!("this.{name}.Click += new System.EventHandler(this.{handler});");
            assert!(designer.contains(&line), "{line}");
        }
        for (name, event, handler) in EMPTY_HANDLERS {
            let line = format!("this.{name}.{event} += new System.EventHandler(this.{handler});");
            assert!(designer.contains(&line), "{line}");
        }
        assert_eq!(designer.matches(" += new ").count(), 8);
        let source = csharp("GCSViews/ConfigurationView/ConfigAteryx.cs").expect("the page");
        for (_, _, handler) in EMPTY_HANDLERS {
            let body = source
                .split(&format!("void {handler}(object sender, EventArgs e)"))
                .nth(1)
                .expect(handler);
            let inside: String = body
                .chars()
                .skip_while(|c| *c != '{')
                .skip(1)
                .take_while(|c| *c != '}')
                .filter(|c| !c.is_whitespace())
                .collect();
            assert!(inside.is_empty(), "{handler} is not empty");
        }
    }

    /// Every place, size, text and parent is the `.resx`'s.
    #[test]
    fn every_place_and_text_is_the_resx() {
        let Some(text) = csharp("GCSViews/ConfigurationView/ConfigAteryx.resx") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = resx(&text);
        let get = |key: String| values.get(&key).cloned().unwrap_or_default();
        assert_eq!(get("$this.Size".to_owned()), pair(PAGE_SIZE));
        for (name, caption, (x, y, w, h)) in GROUPS {
            assert_eq!(get(format!("{name}.Location")), pair((x, y)), "{name}");
            assert_eq!(get(format!("{name}.Size")), pair((w, h)), "{name}");
            assert_eq!(get(format!("{name}.Text")), caption, "{name}");
        }
        for spec in BOXES {
            let (group, ..) = GROUPS[spec.group];
            let name = spec.name;
            assert_eq!(get(format!("{name}.Location")), pair(spec.at), "{name}");
            assert_eq!(get(format!("{name}.Size")), pair(BOX_SIZE), "{name}");
            assert_eq!(get(format!("{PARENT}{name}.Parent")), group, "{name}");
            if let Some((label, caption, at)) = spec.label {
                assert_eq!(get(format!("{label}.Location")), pair(at), "{label}");
                assert_eq!(get(format!("{label}.Text")), caption, "{label}");
                assert_eq!(get(format!("{PARENT}{label}.Parent")), group, "{label}");
            }
        }
        for (name, caption, (x, y, w, h), _) in BUTTONS {
            assert_eq!(get(format!("{name}.Location")), pair((x, y)), "{name}");
            assert_eq!(get(format!("{name}.Size")), pair((w, h)), "{name}");
            assert_eq!(get(format!("{name}.Text")), caption, "{name}");
        }
    }

    /// CONFIG adds the page, with Flight Modes and Ateryx Zero Sensors, for an Ateryx with the
    /// link open - parameters in or not - and for nothing else: SITL's copter never sees it.
    #[test]
    fn the_config_list_adds_the_page_for_an_ateryx_only() {
        let parameters = ateryx_params();
        let classes = |firmware, connected, got_all_params| -> Vec<&'static str> {
            let vehicle = Listed {
                connected,
                got_all_params,
                firmware,
                mav_type: 1,
                version: (0, 0),
                capabilities: 0,
                parameters: &parameters,
            };
            let built = crate::setup::build(List::Config, &vehicle);
            let entries = crate::setup::entries(List::Config);
            built
                .items
                .iter()
                .map(|(at, _)| entries[*at].class)
                .collect()
        };
        for got_all_params in [true, false] {
            let pages = classes(Firmware::Ateryx, true, got_all_params);
            assert!(pages.contains(&CLASS), "{pages:?}");
            assert!(pages.contains(&"ConfigAteryxSensors"));
            assert!(pages.contains(&"ConfigFlightModes"));
        }
        assert!(!classes(Firmware::Ateryx, false, true).contains(&CLASS));
        for firmware in [
            Firmware::ArduCopter2,
            Firmware::ArduPlane,
            Firmware::ArduRover,
        ] {
            assert!(
                !classes(firmware, true, true).contains(&CLASS),
                "{firmware:?}"
            );
        }
        // The heartbeat that makes an Ateryx: MAV_AUTOPILOT_GENERIC, MAV_TYPE_FIXED_WING.
        assert_eq!(
            crate::config::flight_modes::firmware_of(0, 1, None),
            Firmware::Ateryx
        );
    }

    /// `Activate` with no link, or not an Ateryx: the page disabled, nothing bound.
    #[test]
    fn anything_but_an_ateryx_disables_the_page() {
        for (connected, firmware) in [(false, Firmware::Ateryx), (true, Firmware::ArduPlane)] {
            let mut page = Ateryx::<usize>::default();
            page.activate(&ateryx_params(), key(), connected, firmware);
            assert!(!page.enabled());
            assert!(!page.live());
            assert_eq!(text(&page, "ROLL_KP"), "0", "the Designer's value");
        }
    }

    /// Each box the vehicle names is bound with processToScreen's range, increment and places;
    /// the rest are disabled.
    #[test]
    fn process_to_screen_binds_each_box_it_finds() {
        let page = shown();
        assert!(page.enabled());
        let number = |name: &str| &page.numbers()[index(name)];
        // `_P`-style names, zero and fractions: increment 0.001, three places.
        assert_eq!(text(&page, "ROLL_KP"), "0.500");
        assert_eq!(number("ROLL_KP").increment, 0.001);
        assert_eq!(text(&page, "ROLL_KI"), "0.020");
        assert_eq!(text(&page, "THR_MIN"), "0.000", "zero: three places");
        // Whole values under other names: increment 1, one place.
        assert_eq!(text(&page, "THR_MAX"), "100.0");
        assert_eq!(number("THR_MAX").increment, 1.0);
        assert_eq!(text(&page, "LOITER_RD"), "60.0");
        assert_eq!(text(&page, "ROLL_INT_LIM"), "20.0");
        assert_eq!(number("ROLL_INT_LIM").minimum, -9000.0);
        assert_eq!(number("ROLL_INT_LIM").maximum, 9000.0);
        // A value outside -9000 to 9000 throws out of the binding: disabled, its range set.
        assert!(!number("XTRACK_GAIN").enabled);
        assert_eq!(number("XTRACK_GAIN").maximum, 9000.0);
        assert!(!number("EH_KP").enabled, "not named by the vehicle");
        assert_eq!(number("ROLL_KP").back, Back::Bound);
        let enabled = page.numbers().iter().filter(|n| n.enabled).count();
        assert_eq!(enabled, 7);
        assert!(page.changes().is_empty());
    }

    /// Leaving a box validates it - changed or not - into `changes`, green; Write Params sets each
    /// in turn, greys each set that returns, and keeps `changes`.
    #[test]
    fn validated_boxes_are_written_and_kept() {
        let mut page = shown();
        page.type_into(index("ROLL_KP"), "0.75");
        page.begin(index("THR_MAX"));
        page.leave();
        assert_eq!(
            page.changes(),
            [("ROLL_KP".to_owned(), 0.75), ("THR_MAX".to_owned(), 100.0)]
        );
        assert_eq!(page.numbers()[index("ROLL_KP")].back, Back::Green);
        assert_eq!(text(&page, "ROLL_KP"), "0.750");
        // An arrow steps by the increment, and the box takes the focus.
        page.step(index("LOITER_RD"), true);
        assert_eq!(text(&page, "LOITER_RD"), "61.0");
        page.press_write();
        assert!(!page.live(), "held while it writes");
        let link = Answering::new(&[("THR_MAX", Progress::Finished(RequestOutcome::TimedOut))]);
        for _ in 0..20 {
            page.advance(&link);
        }
        assert_eq!(
            link.taken(),
            [
                ("ROLL_KP".to_owned(), 0.75),
                ("THR_MAX".to_owned(), 100.0),
                ("LOITER_RD".to_owned(), 61.0)
            ]
        );
        assert_eq!(page.numbers()[index("ROLL_KP")].back, Back::Bound);
        assert_eq!(
            page.numbers()[index("THR_MAX")].back,
            Back::Green,
            "it threw"
        );
        assert_eq!(
            page.message().map(|m| m.text.as_str()),
            Some("Set THR_MAX Failed")
        );
        assert_eq!(page.changes().len(), 3, "changes is never emptied");
        page.dismiss_message();
        // Refresh: Activate clears them.
        page.activate(&ateryx_params(), key(), true, Firmware::Ateryx);
        assert!(page.changes().is_empty());
    }

    /// The product's path, over the real link to a scripted vehicle: a `MAV_AUTOPILOT_GENERIC`
    /// fixed wing is an Ateryx, CONFIG lists the page for it, `Activate` enables it, and Write
    /// Flash's `COMMAND_LONG` reaches the vehicle - whose acknowledgement ends it quietly.
    #[test]
    fn a_scripted_ateryx_lists_the_page_and_hears_its_flash_command() {
        use crate::telemetry::scripted::{Vehicle as Scripted, ack, until};
        use mp_mavlink_dialects::all::{Heartbeat, MavMessage};

        let (mut telemetry, mut vehicle) = Scripted::connect(mp_link::ProtocolTimeouts::default());
        vehicle.send(&MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 1,
            autopilot: 0,
            base_mode: 81,
            system_status: 3,
            mavlink_version: 3,
        }));
        until("the Ateryx's heartbeat", || {
            telemetry
                .view()
                .state
                .as_ref()
                .is_some_and(|state| state.autopilot == 0 && state.vehicle_type == 1)
        });
        let view = telemetry.view();
        let listed = Listed::of(&view, None);
        assert!(listed.connected);
        assert_eq!(listed.firmware, Firmware::Ateryx);
        let built = crate::setup::build(List::Config, &listed);
        let entries = crate::setup::entries(List::Config);
        assert!(
            built
                .items
                .iter()
                .any(|(at, _)| entries[*at].class == CLASS)
        );

        let mut page = Ateryx::default();
        page.activate(&view.parameters, Key::of(&view), true, listed.firmware);
        assert!(page.enabled());
        page.press_flash(Flash::Write, (0.0, 0.0), |params| {
            send_storage(&mut telemetry, params)
        });
        assert_eq!(page.sent, [1, 0]);
        until("the command", || {
            vehicle.read();
            vehicle.count(|message| {
                matches!(message, MavMessage::CommandLong(long)
                    if long.command == PREFLIGHT_STORAGE && long.param1 == 0.0)
            }) > 0
        });
        vehicle.send(&ack(PREFLIGHT_STORAGE, 0));
        until("the acknowledgement", || {
            let view = telemetry.view();
            page.tick(
                &telemetry,
                &view,
                true,
                false,
                (true, Firmware::Ateryx),
                |id| request_progress(&telemetry, id),
            );
            page.command.is_none()
        });
        assert!(page.message().is_none());
    }

    /// The flash buttons: Write Flash sends PREFLIGHT_STORAGE 0; Read Flash asks, then sends 1;
    /// both refuse while flying; an unanswered command says so.
    #[test]
    fn the_flash_buttons_send_preflight_storage() {
        let mut page = shown();
        let mut sent = Vec::new();
        page.press_flash(Flash::Write, (0.0, 0.0), |params| {
            sent.push(params);
            Some(7)
        });
        assert_eq!(sent, [[0.0; 7]]);
        assert!(!page.live(), "held while the command runs");
        let link = Answering::new(&[]);
        let view = TelemetryView::disconnected("test");
        page.active = true;
        page.tick(&link, &view, true, false, (true, Firmware::Ateryx), |_| {
            Progress::Finished(RequestOutcome::TimedOut)
        });
        assert_eq!(
            page.message().map(|m| m.text.as_str()),
            Some(COMMAND_FAILED)
        );
        page.dismiss_message();

        page.press_flash(Flash::Read, (0.0, 0.0), |_| panic!("asks first"));
        assert_eq!(page.question, Some(Ask::ResetFlash));
        page.answer(true, (0.0, 0.0), |params| {
            sent.push(params);
            Some(8)
        });
        assert_eq!(sent[1][0], 1.0);
        page.tick(&link, &view, true, false, (true, Firmware::Ateryx), |_| {
            Progress::Finished(RequestOutcome::Rejected(4))
        });
        assert!(page.message().is_none(), "false is not looked at");

        // Airborne: 8 m/s through the air stops both; 9 over the ground stops only Read Flash.
        page.press_flash(Flash::Write, (8.0, 0.0), |_| panic!("airborne"));
        assert_eq!(page.message().map(|m| m.text.as_str()), Some(AIRBORNE));
        page.dismiss_message();
        page.press_flash(Flash::Write, (0.0, 9.0), |_| Some(9));
        page.tick(&link, &view, true, false, (true, Firmware::Ateryx), |_| {
            Progress::Finished(RequestOutcome::Accepted { value: None })
        });
        page.press_flash(Flash::Read, (0.0, 9.0), |_| panic!("asks first"));
        page.answer(true, (0.0, 9.0), |_| panic!("airborne"));
        assert_eq!(page.message().map(|m| m.text.as_str()), Some(AIRBORNE));
        assert_eq!(page.sent, [2, 1]);
    }
}
