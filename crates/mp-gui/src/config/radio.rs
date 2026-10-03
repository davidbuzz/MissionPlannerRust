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

//! Radio Calibration: the receiver's sixteen channels as bars, Calibrate Radio's conversation, the
//! four Reverse boxes, the plane's elevon boxes and the Spektrum bind buttons.
//!
//! Mission Planner's `ConfigRadioInput`, a page of Initial Setup under Mandatory Hardware
//! (`GCSViews/InitialSetup.cs:211`). Roll, pitch, throttle and yaw are drawn as two horizontal and
//! two vertical bars, each bound to the channel `RCMAP_*` names and labelled with it; channels 5 to
//! 16 as small bars in two columns beside them. Calibrate Radio is a conversation of message boxes
//! around a loop that folds every reading into each channel's minimum and maximum until the button
//! is pressed again; then the trims are read with the sticks centred and `RCn_MIN`, `RCn_MAX` and
//! `RCn_TRIM` written, forced, for every channel that moved. A Reverse box writes `RCn_REV`
//! (-1/1) or `RCn_REVERSED` (1/0), whichever the vehicle has, and mirrors its bar; on a copter
//! the four are hidden. The elevon boxes show on a plane only. Each bind button sends
//! `MAV_CMD_START_RX_PAIR` with the Spektrum protocol in its second parameter.
//!
//! What the C# does on the UI thread - blocking `setParam` and `doCommand` calls, modal message
//! boxes between them, and a `while (run)` loop pumping `Application.DoEvents` - is a queue of
//! tasks here, advanced once a frame in the C#'s order, with a message box holding everything
//! after it until it is dismissed; the loop is the page folding each frame's reading while it runs.
//! The arithmetic is `mp_calibration::radio`'s.
//!
//! The layout is `ConfigRadioInput.resx`'s: every control at its `Location` and `Size` inside a
//! 628 by 406 page. The colours are this application's: a bar is the theme's dark background with a
//! green value, and a reversed one the green background with a dark value, as `ThemeManager` and
//! `reverseChannel` colour them.
//!
//! What is not ported, and why:
//!
//! * `requestDatastream`'s `hzratecheck`, which skips the request when `RC_CHANNELS_RAW` already
//!   arrives at the rate asked for: this application does not measure a message's rate, and
//!   asking a vehicle for the rate it is already sending changes nothing;
//! * the calibration's writes of `cs.raterc` (10), `rateattitude`, `rateposition` and
//!   `ratestatus` (0) and their restore: they are the rates the link re-requests every stream at,
//!   38 seconds after it last did, and `Telemetry` sets a vehicle's rates only through the
//!   Planner page's hand-over, which saves them as the defaults too. So a calibration that runs
//!   past a re-request has `RC_CHANNELS` asked for at the vehicle's `raterc` and the others at
//!   theirs, where the C# asks 10 and 0. What the old rates feed at the end - `RC_CHANNELS` asked
//!   for at `oldrc` once the save is done - is sent, at the vehicle's `raterc` as it was when the
//!   calibration started, which is `oldrc` since nothing here changes it in between;
//! * the vertical bars' text, which WinForms draws rotated: gpui draws no rotated text, so each
//!   word of it is a line of its own, centred in the bar;
//! * a page made again when `MainV2` reloads Initial Setup on a connect, a disconnect or a change of
//!   vehicle while it shows: the page is made anew when the setup screen is left, as leaving
//!   disposes Initial Setup, and not on those reloads.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{AnyElement, Context, Div, SharedString, Window, div, prelude::*, px, rgb};
use mp_calibration::radio::{
    DATA_STREAM_RC_CHANNELS, RadioCalibration, Spektrum, ch_in, start_rx_pair,
};
use mp_link::current_settings::request_datastream;
use mp_link::requests::RequestOutcome;
use mp_vehicle::StreamRates;
use mp_vehicle::rc::CHANNELS;

use crate::MissionPlanner;
use crate::config::flight_modes::{Firmware, firmware_of};
use crate::telemetry::{Lookup, Report, Telemetry, TelemetryView};
use crate::ui::{action, action_sized, panel, theme};

/// The first message box of Calibrate Radio.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:206-207`
pub const TRANSMITTER_ON: &str = "Ensure your transmitter is on and receiver is powered and \
                                  connected\nEnsure your motor does not have power/no props!!!";
/// The second, before the loop.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:229-230`
pub const MOVE_STICKS: &str = "Click OK and move all RC sticks and switches to their\nextreme \
                               positions so the red bars hit the limits.";
/// When channel 1 never read a real pulse.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:327`
pub const BAD_CHANNEL_ONE: &str = "Bad channel 1 input, canceling";
/// Before the trims are read.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:331`
pub const CENTRE_STICKS: &str =
    "Ensure all your sticks are centered and throttle is down, and click ok to continue";
/// The summary's text, before the data.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:401-403`
pub const SUMMARY: &str = "Here are the detected radio options\nNOTE Channels not connected are \
                           displayed as 1500 +-2\nNormal values are around 1100 | 1900\nChannel:Min \
                           | Max \n";
/// The summary's title.
const SUMMARY_TITLE: &str = "Radio";
/// After `SWITCH_ENABLE` is set to 0.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:462`
pub const DIP_DISABLED: &str = "Disabled Dip Switchs";
/// When that write throws.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:466`
pub const DIP_ERROR: &str = "Error Disableing Dip Switch";
/// `Strings.Put_the_transmitter_in_bind_mode__Receiver_is_waiting`.
/// `// C#: ExtLibs/Strings/Strings.resx:534-536`
pub const BIND_WAITING: &str = "Put the transmitter in bind mode. Receiver is waiting.";
/// `Strings.Error_binding`.
/// `// C#: ExtLibs/Strings/Strings.resx:531-533`
pub const BIND_ERROR: &str = "Error binding";
/// `Strings.ErrorReceivingParams`, newline and all.
/// `// C#: ExtLibs/Strings/Strings.resx:158-160`
pub const ERROR_RECEIVING_PARAMS: &str = "Error receiving list\n";
/// `Strings.ERROR`.
/// `// C#: ExtLibs/Strings/Strings.resx:130-132`
const ERROR_TITLE: &str = "Error";

/// `BUT_Calibrateradio.Text` from the `.resx`.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.resx (BUT_Calibrateradio.Text)`
pub const CALIBRATE: &str = "Calibrate Radio";
/// `Strings.Click_when_Done`.
/// `// C#: ExtLibs/Strings/Strings.resx:540-542`
pub const CLICK_WHEN_DONE: &str = "Click when Done";
/// `Strings.Completed`.
/// `// C#: ExtLibs/Strings/Strings.resx:537-539`
pub const COMPLETED: &str = "Completed";
/// `Strings.Saving`.
/// `// C#: ExtLibs/Strings/Strings.resx:543-545`
pub const SAVING: &str = "Saving";

/// The `RC_CHANNELS` rate `Activate` asks for, "to force this screen to work".
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:115-122`
pub const ACTIVATE_RATE: i32 = 2;
/// The rate the calibration asks for while it runs.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:214, 219-225`
pub const CALIBRATION_RATE: i32 = 10;

/// `MainV2.comPort.MAV.cs`'s stream rates: the shown vehicle's own, which the calibration reads
/// `oldrc` from and puts `RC_CHANNELS` back to once it is saved. With no vehicle, `MAV` is the
/// placeholder state, made with the saved defaults as every `CurrentState` is.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:209, 393-395; ExtLibs/ArduPilot/CurrentState.cs:4393-4397`
#[must_use]
pub fn vehicle_rates(view: &TelemetryView) -> StreamRates {
    view.state
        .as_deref()
        .map_or_else(StreamRates::backups, |state| state.rates)
}

/// Every bar's scale: `Minimum = 800`, `Maximum = 2200`.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.Designer.cs:144-146`
pub const BAR_MINIMUM: i32 = 800;
/// The top of that scale.
pub const BAR_MAXIMUM: i32 = 2200;

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.resx ($this.Size)`
const PAGE: (f32, f32) = (628.0, 406.0);

/// A bar's direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    /// `HorizontalProgressBar2`: filled from the left.
    Horizontal,
    /// `VerticalProgressBar2`: filled from the bottom.
    Vertical,
}

/// One bar as the Designer and the `.resx` make it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BarSpec {
    /// The fact key and probe id suffix: the C# name without `BAR`.
    pub key: &'static str,
    /// `Label`.
    pub label: &'static str,
    /// `Location` and `Size`.
    pub at: (f32, f32, f32, f32),
    /// Which kind of bar.
    pub orientation: Orientation,
    /// `Value`.
    pub value: i32,
}

/// A horizontal channel bar, `BAR5` to `BAR16`: 118 by 25, `Value` 1500.
const fn small(key: &'static str, label: &'static str, x: f32, y: f32) -> BarSpec {
    BarSpec {
        key,
        label,
        at: (x, y, 118.0, 25.0),
        orientation: Orientation::Horizontal,
        value: 1500,
    }
}

/// The sixteen bars: roll, pitch, throttle and yaw, then `BAR5` to `BAR16`.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.Designer.cs:136-400; ConfigRadioInput.resx (BAR*.Location, BAR*.Size)`
pub const BARS: [BarSpec; CHANNELS] = [
    BarSpec {
        key: "roll",
        label: "Roll",
        at: (12.0, 13.0, 288.0, 23.0),
        orientation: Orientation::Horizontal,
        value: 1500,
    },
    BarSpec {
        key: "pitch",
        label: "Pitch",
        at: (12.0, 64.0, 47.0, 211.0),
        orientation: Orientation::Vertical,
        value: 1500,
    },
    BarSpec {
        key: "throttle",
        label: "Throttle",
        at: (251.0, 64.0, 47.0, 211.0),
        orientation: Orientation::Vertical,
        value: 1000,
    },
    BarSpec {
        key: "yaw",
        label: "Yaw",
        at: (12.0, 307.0, 288.0, 23.0),
        orientation: Orientation::Horizontal,
        value: 1500,
    },
    small("5", "Radio 5", 384.0, 13.0),
    small("6", "Radio 6", 384.0, 68.0),
    small("7", "Radio 7", 384.0, 123.0),
    small("8", "Radio 8", 384.0, 178.0),
    small("9", "Radio 9", 384.0, 234.0),
    small("10", "Radio 10", 510.0, 13.0),
    small("11", "Radio 11", 510.0, 68.0),
    small("12", "Radio 12", 510.0, 123.0),
    small("13", "Radio 13", 510.0, 178.0),
    small("14", "Radio 14", 510.0, 234.0),
    small("15", "Radio 15", 384.0, 290.0),
    small("16", "Radio 16", 510.0, 290.0),
];

/// The four Reverse boxes, `CHK_revch1` to `CHK_revch4`, each beside the bar it mirrors: its
/// probe id and `Location`. Each is 66 by 17 with the text "Reverse".
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.resx (CHK_revch1-4.Location, .Size, .Text)`
const REVERSE: [(&str, f32, f32); 4] = [
    ("radio-rev-1", 306.0, 19.0),
    ("radio-rev-2", 65.0, 161.0),
    ("radio-rev-3", 304.0, 161.0),
    ("radio-rev-4", 306.0, 313.0),
];

/// One elevon box: its probe id and fact key, its text, its parameter, and its `Location` in
/// `groupBoxElevons`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevonSpec {
    /// The probe id.
    pub id: &'static str,
    /// The fact key.
    pub key: &'static str,
    /// `Text`.
    pub text: &'static str,
    /// The parameter `Activate` binds it to on a plane.
    pub param: &'static str,
    /// `Location.X`; every one is at `Y` 19.
    pub x: f32,
    /// `Size.Width`.
    pub width: f32,
}

/// `CHK_mixmode`, `CHK_elevonrev`, `CHK_elevonch1rev` and `CHK_elevonch2rev`, in the order
/// `Activate` sets them up.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:129-132; ConfigRadioInput.resx (CHK_mixmode, CHK_elevon*)`
pub const ELEVONS: [ElevonSpec; 4] = [
    ElevonSpec {
        id: "radio-elevons",
        key: "mixmode",
        text: "Elevons",
        param: "ELEVON_MIXING",
        x: 13.0,
        width: 64.0,
    },
    ElevonSpec {
        id: "radio-elevonrev",
        key: "elevonrev",
        text: "Elevons Rev",
        param: "ELEVON_REVERSE",
        x: 83.0,
        width: 87.0,
    },
    ElevonSpec {
        id: "radio-elevonch1rev",
        key: "elevonch1rev",
        text: "Elevons CH1 Rev",
        param: "ELEVON_CH1_REV",
        x: 176.0,
        width: 111.0,
    },
    ElevonSpec {
        id: "radio-elevonch2rev",
        key: "elevonch2rev",
        text: "Elevons CH2 Rev",
        param: "ELEVON_CH2_REV",
        x: 293.0,
        width: 111.0,
    },
];

/// The bind buttons in `groupBox1`: the protocol, the probe id, the text, and `Location.X` and
/// `Size.Width`; each is at `Y` 15 and 23 high.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.resx (BUT_BindDSM2, BUT_BindDSMX, BUT_BindDSM8)`
const BIND: [(Spektrum, &str, &str, f32, f32); 3] = [
    (Spektrum::Dsm2, "radio-bind-dsm2", "Bind DSM2", 6.0, 63.0),
    (Spektrum::DsmX, "radio-bind-dsmx", "Bind DSMX", 71.0, 65.0),
    (Spektrum::Dsm8, "radio-bind-dsm8", "Bind DSM8", 138.0, 65.0),
];

/// A parameter's value, if the vehicle has listed it.
fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// One bar: `HorizontalProgressBar2`'s state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bar {
    /// `Label`.
    pub label: String,
    /// `_value`: what the setter stored, mirrored when the bar is reversed.
    value: i32,
    /// `reverse`.
    pub reverse: bool,
    /// `minline`: a red line, drawn with `maxline` when neither is zero.
    pub minline: i32,
    /// `maxline`.
    pub maxline: i32,
}

impl Bar {
    /// As `InitializeComponent` leaves it: its label, and its designer value set.
    fn designer(spec: &BarSpec) -> Self {
        let mut bar = Self {
            label: spec.label.to_owned(),
            value: 0,
            reverse: false,
            minline: 0,
            maxline: 0,
        };
        bar.set(spec.value);
        bar
    }

    /// `Value`'s setter: nothing when the value is the one stored - which, on a reversed bar, is
    /// the mirrored one - and otherwise the value, mirrored about the scale when reversed.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:85-117`
    pub const fn set(&mut self, value: i32) {
        if self.value == value {
            return;
        }
        self.value = value;
        if self.reverse {
            let dif = value - BAR_MINIMUM;
            self.value = BAR_MAXIMUM - dif;
        }
    }

    /// `Value`'s getter, which is what the bar draws and writes in its text.
    #[must_use]
    pub const fn value(&self) -> i32 {
        self.value
    }

    /// The text drawn in the bar: `(Label + "  " + Value).Trim()`.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:185`
    #[must_use]
    pub fn text(&self) -> String {
        format!("{}  {}", self.label, self.value).trim().to_owned()
    }

    /// How full the bar is drawn: the base bar's value, one above the minimum at the bottom and
    /// the maximum at the top.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:101-111`
    #[must_use]
    pub fn fraction(&self) -> f32 {
        let ans = if self.value <= BAR_MINIMUM {
            BAR_MINIMUM + 1
        } else if self.value >= BAR_MAXIMUM {
            BAR_MAXIMUM
        } else {
            self.value
        };
        #[allow(clippy::cast_precision_loss)] // pulse widths are small integers
        let fraction = (ans - BAR_MINIMUM) as f32 / (BAR_MAXIMUM - BAR_MINIMUM) as f32;
        fraction
    }

    /// Where the red lines are drawn along a bar `length` long, `minline`'s first, or `None` when
    /// they are not drawn. A vertical bar's are offsets from its top, the second six pixels
    /// further down; a reversed bar's run the other way.
    /// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:189-233`
    #[must_use]
    pub fn lines(&self, orientation: Orientation, length: f32) -> Option<(f32, f32)> {
        if self.minline == 0 || self.maxline == 0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)] // pulse widths are small integers
        let along = |from: i32, to: i32| (to - from) as f32 / (BAR_MAXIMUM - BAR_MINIMUM) as f32;
        let (min, max) = (self.minline, self.maxline);
        Some(match (orientation, self.reverse) {
            (Orientation::Vertical, true) => (
                along(BAR_MINIMUM, min) * length + 6.0,
                along(BAR_MINIMUM, max) * length,
            ),
            (Orientation::Vertical, false) => (
                along(min, BAR_MAXIMUM) * length,
                along(max, BAR_MAXIMUM) * length + 6.0,
            ),
            (Orientation::Horizontal, true) => (
                along(min, BAR_MAXIMUM) * length,
                along(max, BAR_MAXIMUM) * length,
            ),
            (Orientation::Horizontal, false) => (
                along(BAR_MINIMUM, min) * length,
                along(BAR_MINIMUM, max) * length,
            ),
        })
    }
}

/// A check box's three states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckState {
    /// Off.
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
}

/// A `MavlinkCheckBox`.
/// `// C#: Controls/MavlinkCheckBox.cs`
#[derive(Debug, Clone, PartialEq)]
pub struct CheckBox {
    /// `ParamName`: none until `setup` finds a parameter the vehicle has.
    pub param: Option<String>,
    /// `OnValue`.
    on: f64,
    /// `OffValue`.
    off: f64,
    /// What it shows.
    pub state: CheckState,
    /// `Enabled`: false from the constructor, true once bound to a parameter the vehicle has.
    pub enabled: bool,
    /// `Visible`.
    pub visible: bool,
}

impl CheckBox {
    /// As the constructor leaves it: disabled, on 1 and off 0.
    /// `// C#: Controls/MavlinkCheckBox.cs:24-30`
    const fn new() -> Self {
        Self {
            param: None,
            on: 1.0,
            off: 0.0,
            state: CheckState::Unchecked,
            enabled: false,
            visible: true,
        }
    }

    /// `setup(OnValue, OffValue, paramname, paramlist)`: bound and shown for a parameter the
    /// vehicle has, checked for the on value, unchecked for the off value, indeterminate for
    /// anything else; disabled, and otherwise untouched, for one it lacks.
    /// `// C#: Controls/MavlinkCheckBox.cs:60-98`
    fn setup(&mut self, on: f64, off: f64, param: &str, parameters: &[(String, f64)]) {
        self.on = on;
        self.off = off;
        self.param = Some(param.to_owned());
        match value_of(parameters, param) {
            Some(value) => {
                self.enabled = true;
                self.visible = true;
                // `paramlist[paramname].Value == OnValue`: an exact comparison, as the C# makes.
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
            None => self.enabled = false,
        }
    }

    /// `setup(double[] OnValue, double[] OffValue, string[] paramname, paramlist)`: the first name
    /// the vehicle has, with its own on and off values; nothing at all when it has none.
    /// `// C#: Controls/MavlinkCheckBox.cs:32-45`
    fn setup_first(&mut self, choices: &[(f64, f64, String)], parameters: &[(String, f64)]) {
        if let Some((on, off, param)) = choices
            .iter()
            .find(|(_, _, name)| value_of(parameters, name).is_some())
        {
            self.setup(*on, *off, param, parameters);
        }
    }

    /// A click. A two-state box goes from checked or indeterminate to unchecked and from
    /// unchecked to checked, and every one of those changes `Checked`, so every click writes:
    /// `OnValue` when it ends checked, `OffValue` otherwise.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    fn click(&mut self) -> Option<(String, f64)> {
        if !self.enabled || !self.visible {
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
        // `ParamName` is null only for a box never set up, which is never enabled.
        Some((self.param.clone()?, value))
    }
}

/// A modal message box, `CustomMessageBox.Show(text, caption)`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    /// The caption; empty for `Show(text)`.
    pub title: &'static str,
    /// The text.
    pub text: String,
}

impl Message {
    /// `CustomMessageBox.Show(text)`.
    fn plain(text: impl Into<String>) -> Self {
        Self {
            title: "",
            text: text.into(),
        }
    }
}

/// What the code after a `setParam` does with how it went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Then {
    /// A `MavlinkCheckBox`: `false` or an exception shows `Set NAME Failed`.
    CheckBox,
    /// `reverseChannel`'s `SWITCH_ENABLE`: "Disabled Dip Switchs" unless it throws.
    DipSwitch,
    /// The calibration's save of one channel: an exception ends the channel's writes, and says
    /// "Failed to set Channel N" if the vehicle has `RCn_MIN`.
    Channel(usize),
}

/// What dismissing a message box goes on to do: the code after the C#'s `Show` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AfterOk {
    /// Nothing: the handler ends.
    Nothing,
    /// The calibration's rates, button text and second message.
    StartCapture,
    /// `run = true`: the loop.
    Run,
    /// The trims, and the save.
    Trim,
    /// The button reads `Completed`.
    Completed,
}

/// One thing the C#'s handlers do, in order.
#[derive(Debug, Clone, PartialEq)]
enum Task {
    /// `setParam(name, value, force)`, and what follows it.
    Set {
        name: String,
        value: f64,
        force: bool,
        then: Then,
    },
    /// `doCommand(START_RX_PAIR)` and its message.
    Bind(Spektrum),
    /// `requestDatastream(RC_CHANNELS, hz)`.
    Stream(i32),
    /// `cs.raterc = ...; cs.rateattitude = ...; ...`: the vehicle's stream rates set, so the
    /// periodic re-request asks for them. `// C#: ConfigRadioInput.cs:214-217, 388-391`
    Rates(mp_vehicle::StreamRates),
    /// A message box, holding everything after it until dismissed.
    Say(Message, AfterOk),
}

/// What a request under way was for.
#[derive(Debug, Clone, PartialEq)]
enum Asked {
    /// A parameter write.
    Set {
        name: String,
        value: f64,
        then: Then,
    },
    /// A bind.
    Bind(Spektrum),
}

/// A request the link is carrying.
#[derive(Debug, Clone)]
struct Waiting {
    id: mp_link::RequestId,
    made: Instant,
    asked: Asked,
}

/// How a request ended.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Ended {
    /// The vehicle's answer, or the link's giving up.
    Outcome(RequestOutcome),
    /// Never sent: no link to send it on. `setParam` throws on a closed port; `doCommand`
    /// returns false.
    NotSent,
}

/// The page: the Designer's controls, the calibration's arrays, and the queue the handlers' work
/// waits in.
#[derive(Debug)]
pub struct RadioInput {
    /// Between `Activate` and `Deactivate`: the timer runs.
    active: bool,
    /// Whether an `Activate` has run on this page since it was made.
    touched: bool,
    /// The page's `Enabled`: false for good once the channel map could not be read.
    enabled: bool,
    /// `cs.firmware` when the page opened.
    firmware: Firmware,
    /// `chroll`, `chpitch`, `chthro` and `chyaw`: -1 until read.
    map: [i32; 4],
    /// The channel each bar is bound to, from one; `None` unbound.
    bindings: [Option<usize>; CHANNELS],
    bars: [Bar; CHANNELS],
    /// `CHK_revch1` to `CHK_revch4`, beside roll, pitch, throttle and yaw.
    reverse: [CheckBox; 4],
    /// `CHK_mixmode`, `CHK_elevonrev`, `CHK_elevonch1rev`, `CHK_elevonch2rev`.
    elevons: [CheckBox; 4],
    /// `groupBoxElevons.Visible`: hidden by the first `Activate` on anything but a plane.
    elevons_visible: bool,
    /// `rcmin`, `rcmax`, `rctrim`.
    calibration: RadioCalibration,
    /// `run`: the loop is folding readings in.
    running: bool,
    /// `BUT_Calibrateradio.Text`.
    button: &'static str,
    tasks: VecDeque<Task>,
    waiting: Option<Waiting>,
    /// How the last write ended, for a test to read.
    last_write: Option<String>,
    /// How the last bind ended.
    last_bind: Option<String>,
    /// The last `RC_CHANNELS` rate asked for.
    stream: Option<i32>,
    /// `oldrc`: the vehicle's `cs.raterc` when the calibration started, which its end asks
    /// `RC_CHANNELS` for at again. `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:209`
    old_rc: Option<i32>,
    /// `oldatt`, `oldpos`, `oldstatus` with it: the vehicle's rates before the capture set
    /// RC to 10 and those three to 0, put back after the save.
    old_rates: Option<mp_vehicle::StreamRates>,
}

impl Default for RadioInput {
    /// As the constructor leaves it.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:25-40`
    fn default() -> Self {
        Self {
            active: false,
            touched: false,
            enabled: true,
            firmware: Firmware::default(),
            map: [-1; 4],
            bindings: [None; CHANNELS],
            bars: BARS.map(|spec| Bar::designer(&spec)),
            reverse: [
                CheckBox::new(),
                CheckBox::new(),
                CheckBox::new(),
                CheckBox::new(),
            ],
            elevons: [
                CheckBox::new(),
                CheckBox::new(),
                CheckBox::new(),
                CheckBox::new(),
            ],
            elevons_visible: true,
            calibration: RadioCalibration::new(),
            running: false,
            button: CALIBRATE,
            tasks: VecDeque::new(),
            waiting: None,
            last_write: None,
            last_bind: None,
            stream: None,
            old_rc: None,
            old_rates: None,
        }
    }
}

impl RadioInput {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Opens the page if it is closed and closes it if it is open.
    pub fn toggle(&mut self, telemetry: &Telemetry) {
        if self.active {
            self.deactivate();
        } else {
            let view = telemetry.view();
            let firmware = view.state.as_ref().map_or(Firmware::ArduCopter2, |state| {
                firmware_of(
                    state.autopilot,
                    state.vehicle_type,
                    telemetry.firmware_banner(),
                )
            });
            self.activate(&view.parameters, firmware);
        }
    }

    /// `Activate`: starts the timer; reads the channel map, or stops with an error and disables
    /// the page when part of it is missing; binds the bars and adds each stick's channel to its
    /// label; asks for `RC_CHANNELS` at 2 Hz; sets up the elevon boxes on a plane and hides them
    /// otherwise; binds the Reverse boxes and mirrors each bar whose `RCn_REVERSED` is 1; and on a
    /// copter hides the Reverse boxes.
    ///
    /// The labels are added to, not set, as the C#'s `Label + " (rcN)"` adds to them: a page
    /// opened twice in one visit to Initial Setup reads "Roll (rc1) (rc1)", as Mission Planner's
    /// does.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:42-177`
    pub fn activate(&mut self, parameters: &[(String, f64)], firmware: Firmware) {
        self.active = true;
        self.touched = true;
        self.firmware = firmware;

        let map = if value_of(parameters, "RCMAP_ROLL").is_none() {
            [1, 2, 3, 4]
        } else {
            let read = ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW"]
                .map(|name| value_of(parameters, name));
            let [Some(roll), Some(pitch), Some(throttle), Some(yaw)] = read else {
                self.tasks.push_back(Task::Say(
                    Message {
                        title: ERROR_TITLE,
                        text: ERROR_RECEIVING_PARAMS.to_owned(),
                    },
                    AfterOk::Nothing,
                ));
                self.enabled = false;
                return;
            };
            // `(int)(float)param`: truncated.
            #[allow(clippy::cast_possible_truncation)]
            let map = [roll, pitch, throttle, yaw].map(|value| value as f32 as i32);
            map
        };
        self.map = map;

        // `DataBindings.Clear()` on every bar, then each bound again. A mapped channel with no
        // `chNin` of its name throws where it is bound, and `Activate` ends there.
        self.bindings = [None; CHANNELS];
        for (binding, channel) in self.bindings.iter_mut().zip(map) {
            let Some(channel) = usize::try_from(channel)
                .ok()
                .filter(|channel| (1..=CHANNELS).contains(channel))
            else {
                return;
            };
            *binding = Some(channel);
        }
        for (index, binding) in self.bindings.iter_mut().enumerate().skip(4) {
            *binding = Some(index + 1);
        }
        for (bar, channel) in self.bars.iter_mut().zip(map) {
            bar.label.push_str(&format!(" (rc{channel})"));
        }

        self.tasks.push_back(Task::Stream(ACTIVATE_RATE));

        if matches!(firmware, Firmware::ArduPlane | Firmware::Ateryx) {
            for (check, spec) in self.elevons.iter_mut().zip(ELEVONS) {
                check.setup(1.0, 0.0, spec.param, parameters);
            }
        } else {
            self.elevons_visible = false;
        }

        // "this controls the direction of the output, not the input."
        for (check, channel) in self.reverse.iter_mut().zip(map) {
            check.setup_first(
                &[
                    (-1.0, 1.0, format!("RC{channel}_REV")),
                    (1.0, 0.0, format!("RC{channel}_REVERSED")),
                ],
                parameters,
            );
        }
        for (index, channel) in map.into_iter().enumerate() {
            // `param["RCn_REVERSED"]?.Value == 1`.
            let reversed = value_of(parameters, &format!("RC{channel}_REVERSED")) == Some(1.0);
            if reversed {
                self.reverse_bar(index, true);
            }
        }

        // "run after to ensure they are disabled on copter"
        if firmware == Firmware::ArduCopter2 {
            for check in &mut self.reverse {
                check.visible = false;
            }
        }
    }

    /// `Deactivate`: the timer stops. The calibration's loop, and any write under way, carry on.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:179-182`
    pub const fn deactivate(&mut self) {
        self.active = false;
    }

    /// Whether nothing is under way: no loop, no queued work, no message showing.
    fn idle(&self) -> bool {
        !self.running && self.tasks.is_empty() && self.waiting.is_none()
    }

    /// Once a frame: the page made anew once the setup screen has been left, as leaving disposes
    /// Initial Setup and its pages - unless the calibration or a write is still going, which the
    /// C#'s handler would still be running; the bars and the loop fed from the vehicle's
    /// reading; and the queue moved on.
    pub fn tick(&mut self, telemetry: &mut Telemetry, view: &TelemetryView, on_setup: bool) {
        if !on_setup && self.touched && self.idle() {
            *self = Self::default();
        }
        let rc = view
            .state
            .as_deref()
            .map(|state| state.rc)
            .unwrap_or_default();
        self.observe(&ch_in(&rc));
        self.advance(telemetry);
    }

    /// One reading of `ch1in` to `ch16in`: the timer's rebinding while the page shows, and the
    /// loop's folding while it runs - which rebinds too, whether or not the page shows.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:184-195, 235-320`
    pub fn observe(&mut self, inputs: &[f32; CHANNELS]) {
        if self.active || self.running {
            for (bar, binding) in self.bars.iter_mut().zip(self.bindings) {
                if let Some(reading) = binding.and_then(|channel| inputs.get(channel - 1)) {
                    // A formatted binding's float to int: pulse widths are whole.
                    #[allow(clippy::cast_possible_truncation)]
                    bar.set(*reading as i32);
                }
            }
        }
        if self.running && self.calibration.observe(inputs) {
            self.move_lines();
        }
    }

    /// The red lines: each stick's bar at its mapped channel's minimum and maximum, `BAR5` to
    /// `BAR16` at channels 5 to 16's.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:294-318`
    fn move_lines(&mut self) {
        let map = self.map;
        let calibration = self.calibration;
        for (index, bar) in self.bars.iter_mut().enumerate() {
            let channel = match map.get(index) {
                Some(mapped) => usize::try_from(*mapped).ok(),
                None => Some(index + 1),
            };
            let Some(slot) = channel.and_then(|channel| channel.checked_sub(1)) else {
                continue;
            };
            if let (Some(min), Some(max)) = (calibration.min.get(slot), calibration.max.get(slot)) {
                // `(int)rcmin[...]`: truncated.
                #[allow(clippy::cast_possible_truncation)]
                let (min, max) = (*min as i32, *max as i32);
                bar.minline = min;
                bar.maxline = max;
            }
        }
    }

    /// `reverseChannel`'s first half: the bar mirrored, or not, with its colours.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:439-452`
    fn reverse_bar(&mut self, index: usize, reverse: bool) {
        if let Some(bar) = self.bars.get_mut(index) {
            bar.reverse = reverse;
        }
    }

    /// Whether the page takes a click: open, enabled, and not blocked in a handler - the C#'s
    /// `setParam` and `doCommand` hold the UI thread, and a message box is modal.
    fn takes_clicks(&self) -> bool {
        self.active && self.enabled && self.tasks.is_empty() && self.waiting.is_none()
    }

    /// Calibrate Radio pressed. While the loop runs, that ends it: the button reads `Completed`,
    /// and the loop's end follows - "Bad channel 1 input, canceling" when channel 1 never read a
    /// real pulse, otherwise the sticks-centred message whose OK takes the trims. Otherwise it is
    /// the first message of a new calibration.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:197-207, 320-331`
    pub fn click_calibrate(&mut self) {
        if !self.takes_clicks() {
            return;
        }
        if self.running {
            self.button = COMPLETED;
            self.running = false;
            if self.calibration.channel_one_good() {
                self.say(Message::plain(CENTRE_STICKS), AfterOk::Trim);
            } else {
                self.say(Message::plain(BAD_CHANNEL_ONE), AfterOk::Nothing);
            }
            return;
        }
        self.say(Message::plain(TRANSMITTER_ON), AfterOk::StartCapture);
    }

    /// A Reverse box clicked: `CHK_revchN_CheckedChanged`, raised from inside the box's own
    /// handler before its write - the bar mirrored, then, if the vehicle has `SWITCH_ENABLE` at 1,
    /// that written 0 and "Disabled Dip Switchs" said - and then the box's write of `RCn_REV` or
    /// `RCn_REVERSED`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:419-469; Controls/MavlinkCheckBox.cs:106-143`
    pub fn click_reverse(&mut self, index: usize, parameters: &[(String, f64)]) {
        if !self.takes_clicks() {
            return;
        }
        let Some((param, value)) = self.reverse.get_mut(index).and_then(CheckBox::click) else {
            return;
        };
        let checked = self
            .reverse
            .get(index)
            .is_some_and(|check| check.state == CheckState::Checked);
        self.reverse_bar(index, checked);
        // `(float)param["SWITCH_ENABLE"] == 1`.
        #[allow(clippy::float_cmp, clippy::cast_possible_truncation)]
        let dip_switches =
            value_of(parameters, "SWITCH_ENABLE").is_some_and(|value| value as f32 == 1.0);
        if dip_switches {
            self.tasks.push_back(Task::Set {
                name: "SWITCH_ENABLE".to_owned(),
                value: 0.0,
                force: false,
                then: Then::DipSwitch,
            });
        }
        self.tasks.push_back(Task::Set {
            name: param,
            value,
            force: false,
            then: Then::CheckBox,
        });
    }

    /// An elevon box clicked: its write, and nothing else.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    pub fn click_elevon(&mut self, index: usize) {
        if !self.takes_clicks() || !self.elevons_visible {
            return;
        }
        if let Some((name, value)) = self.elevons.get_mut(index).and_then(CheckBox::click) {
            self.tasks.push_back(Task::Set {
                name,
                value,
                force: false,
                then: Then::CheckBox,
            });
        }
    }

    /// A bind button: `START_RX_PAIR`, then its message.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:471-508`
    pub fn click_bind(&mut self, spektrum: Spektrum) {
        if self.takes_clicks() {
            self.tasks.push_back(Task::Bind(spektrum));
        }
    }

    /// Queues a message box.
    fn say(&mut self, message: Message, after: AfterOk) {
        self.tasks.push_back(Task::Say(message, after));
    }

    /// Puts work at the head of the queue, in order: what the handler does next, before
    /// anything queued behind the message just dismissed.
    fn next(&mut self, tasks: Vec<Task>) {
        for task in tasks.into_iter().rev() {
            self.tasks.push_front(task);
        }
    }

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        match self.tasks.front() {
            Some(Task::Say(message, _)) if self.waiting.is_none() => Some(message),
            _ => None,
        }
    }

    /// The message box's OK clicked: [`RadioInput::dismiss`] with the shown vehicle's `ch1in` to
    /// `ch16in` and its stream rates as they are now.
    pub fn click_ok(&mut self, view: &TelemetryView) {
        let rc = view
            .state
            .as_deref()
            .map(|state| state.rc)
            .unwrap_or_default();
        self.dismiss(&ch_in(&rc), vehicle_rates(view));
    }

    /// OK on the message box showing, and what the handler does after it, given the reading and
    /// the vehicle's stream rates ([`vehicle_rates`]) at that moment.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:209-232, 331-351, 353-405`
    pub fn dismiss(&mut self, inputs: &[f32; CHANNELS], rates: StreamRates) {
        if self.message().is_none() {
            return;
        }
        let Some(Task::Say(_, after)) = self.tasks.pop_front() else {
            return;
        };
        match after {
            AfterOk::Nothing => {}
            AfterOk::StartCapture => {
                // C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:209, `var oldrc =
                // MainV2.comPort.MAV.cs.raterc;` - read once the first message is dismissed.
                self.old_rc = Some(rates.rc);
                self.old_rates = Some(rates);
                self.button = CLICK_WHEN_DONE;
                // C#: ConfigRadioInput.cs:214-217 - `cs.raterc = 10; cs.rateattitude = 0;
                // cs.rateposition = 0; cs.ratestatus = 0;` before the RC request.
                self.next(vec![
                    Task::Rates(mp_vehicle::StreamRates {
                        rc: CALIBRATION_RATE,
                        attitude: 0,
                        position: 0,
                        status: 0,
                        ..rates
                    }),
                    Task::Stream(CALIBRATION_RATE),
                    Task::Say(Message::plain(MOVE_STICKS), AfterOk::Run),
                ]);
            }
            AfterOk::Run => self.running = true,
            AfterOk::Trim => {
                self.calibration.take_trims(inputs);
                self.button = SAVING;
                let mut tail: Vec<Task> = self
                    .calibration
                    .writes()
                    .into_iter()
                    .flat_map(|channel| {
                        channel.params().map(|(name, value)| Task::Set {
                            name,
                            value,
                            force: true,
                            then: Then::Channel(channel.number),
                        })
                    })
                    .collect();
                // C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:388-395, `cs.raterc =
                // oldrc;` then `requestDatastream(RC_CHANNELS, oldrc)`. The C# always has `oldrc`
                // here, the loop being reachable only through the first message; without it the
                // vehicle's `raterc` now is the same number, as nothing here changes it between.
                let old_rc = self.old_rc.take().unwrap_or(rates.rc);
                // C#: ConfigRadioInput.cs:388-391 - the four rates put back, then the request.
                if let Some(old) = self.old_rates.take() {
                    tail.push(Task::Rates(old));
                }
                tail.push(Task::Stream(old_rc));
                tail.push(Task::Say(
                    Message {
                        title: SUMMARY_TITLE,
                        text: format!("{SUMMARY}{}", self.calibration.data()),
                    },
                    AfterOk::Completed,
                ));
                self.next(tail);
            }
            AfterOk::Completed => self.button = COMPLETED,
        }
    }

    /// Moves the queue on as far as the link allows: a request under way checked, and the tasks
    /// after it started in order, up to a message box.
    pub fn advance(&mut self, telemetry: &mut Telemetry) {
        loop {
            if let Some(waiting) = self.waiting.take() {
                let ended = match telemetry.lookup(waiting.id, waiting.made) {
                    Lookup::PickingUp => None,
                    Lookup::Gone => Some(RequestOutcome::TimedOut),
                    Lookup::Found(request) => request.outcome(),
                };
                let Some(outcome) = ended else {
                    self.waiting = Some(waiting);
                    return;
                };
                self.ended(waiting.asked, Ended::Outcome(outcome), telemetry);
                continue;
            }
            if matches!(self.tasks.front(), None | Some(Task::Say(..))) {
                return;
            }
            let Some(task) = self.tasks.pop_front() else {
                return;
            };
            match task {
                Task::Set {
                    name,
                    value,
                    force,
                    then,
                } => {
                    let started = telemetry.write_parameter(&name, value, force);
                    let asked = Asked::Set { name, value, then };
                    match started {
                        Some(id) => {
                            self.waiting = Some(Waiting {
                                id,
                                made: Instant::now(),
                                asked,
                            });
                        }
                        None => self.ended(asked, Ended::NotSent, telemetry),
                    }
                }
                Task::Bind(spektrum) => {
                    let target = telemetry.send_handle().map(|(_, id)| id);
                    let id = target.and_then(|target| {
                        telemetry
                            .command_message(&start_rx_pair(target, spektrum), Report::default())
                    });
                    match id {
                        Some(id) => {
                            self.waiting = Some(Waiting {
                                id,
                                made: Instant::now(),
                                asked: Asked::Bind(spektrum),
                            });
                        }
                        None => self.ended(Asked::Bind(spektrum), Ended::NotSent, telemetry),
                    }
                }
                Task::Rates(rates) => telemetry.set_stream_rates(rates),
                Task::Stream(hz) => {
                    // Nothing for a rate of -1, and the rate as a byte otherwise.
                    // C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3061-3070, 3256
                    if let Some((sender, id)) = telemetry.send_handle()
                        && let Some(request) = request_datastream(id, DATA_STREAM_RC_CHANNELS, hz)
                    {
                        sender.send(&request);
                    }
                    self.stream = Some(hz);
                }
                Task::Say(message, after) => {
                    self.tasks.push_front(Task::Say(message, after));
                    return;
                }
            }
        }
    }

    /// What the C# does once a request returns or throws.
    fn ended(&mut self, asked: Asked, ended: Ended, telemetry: &Telemetry) {
        match asked {
            Asked::Set { name, value, then } => {
                self.set_ended(&name, value, then, ended, telemetry)
            }
            // `doCommand`'s result is not looked at: the message is the same for yes and no;
            // only its `TimeoutException` says otherwise.
            // `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:473-481`
            Asked::Bind(spektrum) => {
                let (word, thrown) = match ended {
                    Ended::Outcome(RequestOutcome::TimedOut) => ("timed out".to_owned(), true),
                    Ended::Outcome(RequestOutcome::Rejected(result)) => {
                        (format!("refused {result}"), false)
                    }
                    Ended::Outcome(_) => ("accepted".to_owned(), false),
                    Ended::NotSent => ("not sent".to_owned(), false),
                };
                self.last_bind = Some(format!("START_RX_PAIR {} {word}", spektrum.param2()));
                let text = if thrown { BIND_ERROR } else { BIND_WAITING };
                self.tasks
                    .push_front(Task::Say(Message::plain(text), AfterOk::Nothing));
            }
        }
    }

    /// A write's end: `true`, `false` or an exception, and the code that follows it.
    fn set_ended(
        &mut self,
        name: &str,
        value: f64,
        then: Then,
        ended: Ended,
        telemetry: &Telemetry,
    ) {
        // `setParam`: true for an echo and for a value already held, false for a name the vehicle
        // has not listed, and a `TimeoutException` when every retry goes unanswered.
        // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1640-1651, 1748-1766`
        let (returned, word) = match ended {
            Ended::Outcome(RequestOutcome::Accepted { value: echoed }) => {
                let echoed = echoed.map_or(value, |echoed| echoed.as_f64());
                (Some(true), format!("{name} {echoed} accepted"))
            }
            Ended::Outcome(RequestOutcome::Unchanged | RequestOutcome::Sent) => {
                (Some(true), format!("{name} {value} unchanged"))
            }
            Ended::Outcome(RequestOutcome::UnknownParameter) => {
                (Some(false), format!("{name} {value} not on the vehicle"))
            }
            Ended::Outcome(RequestOutcome::Rejected(result)) => {
                (Some(false), format!("{name} {value} rejected {result}"))
            }
            Ended::Outcome(RequestOutcome::TimedOut) => {
                (None, format!("{name} {value} failed: timed out"))
            }
            Ended::NotSent => (None, format!("{name} {value} failed: no vehicle")),
        };
        self.last_write = Some(word);
        match then {
            Then::CheckBox => {
                if returned != Some(true) {
                    self.tasks.push_front(Task::Say(
                        Message {
                            title: ERROR_TITLE,
                            text: format!("Set {name} Failed"),
                        },
                        AfterOk::Nothing,
                    ));
                }
            }
            Then::DipSwitch => {
                let text = if returned.is_some() {
                    DIP_DISABLED
                } else {
                    DIP_ERROR
                };
                self.tasks
                    .push_front(Task::Say(Message::plain(text), AfterOk::Nothing));
            }
            Then::Channel(channel) => {
                if returned.is_none() {
                    self.tasks.retain(|task| {
                        !matches!(task, Task::Set { then: Then::Channel(other), .. } if *other == channel)
                    });
                    if telemetry.holds_parameter(&format!("RC{channel}_MIN")) {
                        self.tasks.push_front(Task::Say(
                            Message::plain(format!("Failed to set Channel {channel}")),
                            AfterOk::Nothing,
                        ));
                    }
                }
            }
        }
    }

    /// Requests not yet ended and work not yet started, message boxes aside.
    fn pending(&self) -> usize {
        usize::from(self.waiting.is_some())
            + self
                .tasks
                .iter()
                .filter(|task| !matches!(task, Task::Say(..)))
                .count()
    }
}

/// Facts for a test: what the page read and shows, the calibration's arrays and what they would
/// write, how its writes and binds went, and the vehicle's `RCn_` values as `params.value.<name>`.
pub fn record_facts(radio: &RadioInput, view: &TelemetryView) {
    use crate::facts::record;
    record("config.radio.active", radio.active);
    record("config.radio.enabled", radio.enabled);
    record("config.radio.firmware", radio.firmware.label());
    record(
        "config.radio.map",
        if radio.map.contains(&-1) {
            "none".to_owned()
        } else {
            radio
                .map
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(",")
        },
    );
    for ((spec, bar), binding) in BARS.iter().zip(&radio.bars).zip(radio.bindings) {
        let key = spec.key;
        record(format!("config.radio.bar.{key}"), bar.value());
        record(format!("config.radio.bar.{key}.text"), bar.text());
        record(format!("config.radio.bar.{key}.reverse"), bar.reverse);
        record(format!("config.radio.bar.{key}.min"), bar.minline);
        record(format!("config.radio.bar.{key}.max"), bar.maxline);
        record(
            format!("config.radio.bar.{key}.channel"),
            binding.map_or_else(|| "none".to_owned(), |channel| channel.to_string()),
        );
    }
    let check = |key: String, boxed: &CheckBox| {
        record(
            format!("{key}.param"),
            boxed.param.as_deref().unwrap_or("none"),
        );
        record(key.clone(), boxed.state.key());
        record(format!("{key}.enabled"), boxed.enabled);
        record(format!("{key}.visible"), boxed.visible);
    };
    for (index, reverse) in radio.reverse.iter().enumerate() {
        check(format!("config.radio.reverse.{}", index + 1), reverse);
    }
    record("config.radio.elevons.visible", radio.elevons_visible);
    for (spec, elevon) in ELEVONS.iter().zip(&radio.elevons) {
        check(format!("config.radio.elevons.{}", spec.key), elevon);
    }
    record("config.radio.button", radio.button);
    record("config.radio.running", radio.running);
    for channel in 1..=CHANNELS {
        let slot = channel - 1;
        let read = |values: &[f32; CHANNELS]| values.get(slot).copied().unwrap_or_default();
        record(
            format!("config.radio.min.{channel}"),
            read(&radio.calibration.min),
        );
        record(
            format!("config.radio.max.{channel}"),
            read(&radio.calibration.max),
        );
        record(
            format!("config.radio.trim.{channel}"),
            read(&radio.calibration.trim),
        );
    }
    let writes: Vec<String> = radio
        .calibration
        .writes()
        .iter()
        .flat_map(|channel| channel.params())
        .map(|(name, value)| format!("{name}={value}"))
        .collect();
    record(
        "config.radio.writes",
        if writes.is_empty() {
            "none".to_owned()
        } else {
            writes.join(",")
        },
    );
    record(
        "config.radio.message",
        radio
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.radio.message.title",
        radio.message().map_or("none", |message| message.title),
    );
    record("config.radio.pending", radio.pending());
    record(
        "config.radio.write",
        radio.last_write.as_deref().unwrap_or("none"),
    );
    record(
        "config.radio.bind",
        radio.last_bind.as_deref().unwrap_or("none"),
    );
    record(
        "config.radio.stream",
        radio
            .stream
            .map_or_else(|| "none".to_owned(), |hz| hz.to_string()),
    );
    for channel in 1..=CHANNELS {
        for end in ["MIN", "MAX", "TRIM", "REVERSED"] {
            let name = format!("RC{channel}_{end}");
            if let Some(value) = value_of(&view.parameters, &name) {
                record(format!("params.value.{name}"), value);
            }
        }
    }
    if let Some(value) = value_of(&view.parameters, "SWITCH_ENABLE") {
        record("params.value.SWITCH_ENABLE", value);
    }
}

/// An absolutely placed box, at a `.resx` `Location` and `Size`.
fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A `GroupBox`: a border with its caption.
fn group(x: f32, y: f32, width: f32, height: f32, title: &'static str, enabled: bool) -> Div {
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
                .text_color(rgb(if enabled { theme::DIM } else { theme::BORDER }))
                .child(title),
        )
}

/// One bar: its background, its value from the left or the bottom, its text in the middle, and
/// the red lines once a calibration has moved them.
fn bar_element(spec: &BarSpec, bar: &Bar) -> AnyElement {
    let (x, y, width, height) = spec.at;
    // `reverseChannel` swaps the two: green background and dark value when reversed.
    let (background, value) = if bar.reverse {
        (theme::OK, theme::BG)
    } else {
        (theme::BG, theme::OK)
    };
    let fraction = bar.fraction();
    let fill = match spec.orientation {
        Orientation::Horizontal => div()
            .absolute()
            .left_0()
            .top_0()
            .h_full()
            .w(gpui::relative(fraction)),
        Orientation::Vertical => div()
            .absolute()
            .left_0()
            .bottom_0()
            .w_full()
            .h(gpui::relative(fraction)),
    }
    .bg(rgb(value));
    let text = bar.text();
    let label = match spec.orientation {
        Orientation::Horizontal => div().child(text),
        // Drawn rotated in the C#; a word to a line here.
        Orientation::Vertical => div().flex().flex_col().items_center().children(
            text.split_whitespace()
                .map(|word| div().child(word.to_owned())),
        ),
    };
    let id = format!("radio-bar-{}", spec.key);
    let mut element = crate::probe::measured(id.clone(), at(x, y, width, height))
        .id(SharedString::from(id))
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(background))
        .child(fill)
        .child(
            div()
                .absolute()
                .inset_0()
                .flex()
                .items_center()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(label),
        );
    let length = match spec.orientation {
        Orientation::Horizontal => width,
        Orientation::Vertical => height,
    };
    if let Some((min, max)) = bar.lines(spec.orientation, length) {
        for (position, number) in [(min, bar.minline), (max, bar.maxline)] {
            let line = match spec.orientation {
                Orientation::Horizontal => div()
                    .absolute()
                    .top_0()
                    .h_full()
                    .w(px(2.0))
                    .left(px(position - 1.0)),
                Orientation::Vertical => div()
                    .absolute()
                    .left_0()
                    .w_full()
                    .h(px(2.0))
                    .top(px(position - 1.0)),
            };
            let note = match spec.orientation {
                Orientation::Horizontal => div().absolute().top(px(1.0)).left(px(position + 2.0)),
                Orientation::Vertical => div().absolute().left(px(3.0)).top(px(position + 2.0)),
            };
            element = element.child(line.bg(rgb(theme::ALERT))).child(
                note.text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(number.to_string()),
            );
        }
    }
    element.into_any_element()
}

/// A check box: a square that fills when checked, a bar across it when indeterminate, and its
/// text. Dimmed and inert when disabled or the page takes no click.
fn check_box(
    id: &'static str,
    text: &'static str,
    check: &CheckBox,
    clickable: bool,
    on_click: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
) -> gpui::Stateful<Div> {
    let live = clickable && check.enabled;
    let colour = if live { theme::ACCENT } else { theme::BORDER };
    let square = div()
        .size(px(13.0))
        .flex_shrink_0()
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(colour))
        .bg(rgb(theme::BG))
        .children(match check.state {
            CheckState::Checked => Some(div().size(px(7.0)).bg(rgb(colour))),
            CheckState::Indeterminate => Some(div().w(px(7.0)).h(px(2.0)).bg(rgb(colour))),
            CheckState::Unchecked => None,
        });
    let element = crate::probe::measured(id, div())
        .id(id)
        .flex()
        .items_center()
        .gap_1()
        .text_xs()
        .text_color(rgb(if check.enabled {
            theme::TEXT
        } else {
            theme::DIM
        }))
        .child(square)
        .child(text);
    if live {
        element.cursor_pointer().on_click(on_click)
    } else {
        element
    }
}

/// The page, while it is showing.
pub fn page(radio: &RadioInput, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    if !radio.active {
        return None;
    }
    let clickable = radio.takes_clicks();
    let mut body = div().relative().w(px(PAGE.0)).h(px(PAGE.1));

    for (spec, bar) in BARS.iter().zip(&radio.bars) {
        body = body.child(bar_element(spec, bar));
    }

    for (index, ((id, x, y), check)) in REVERSE.into_iter().zip(&radio.reverse).enumerate() {
        if !check.visible {
            continue;
        }
        body = body.child(at(x, y, 66.0, 17.0).child(check_box(
            id,
            "Reverse",
            check,
            clickable,
            cx.listener(move |this, _event, _window, cx| {
                let parameters = this.telemetry.view().parameters;
                this.radio_input.click_reverse(index, &parameters);
                cx.notify();
            }),
        )));
    }

    // `BUT_Calibrateradio`, 134 by 23 at (491, 327).
    body = body.child(at(491.0, 327.0, 134.0, 23.0).child(action_sized(
        "radio-calibrate",
        radio.button,
        theme::WARN,
        clickable,
        Some(px(134.0)),
        cx.listener(|this, _event: &(), _window, cx| {
            this.radio_input.click_calibrate();
            cx.notify();
        }),
    )));

    // `groupBoxElevons`, "Elevon Config", 409 by 42 at (3, 356).
    if radio.elevons_visible {
        let mut elevons = group(3.0, 356.0, 409.0, 42.0, "Elevon Config", radio.enabled);
        for (index, (spec, check)) in ELEVONS.iter().zip(&radio.elevons).enumerate() {
            elevons = elevons.child(at(spec.x, 19.0, spec.width, 17.0).child(check_box(
                spec.id,
                spec.text,
                check,
                clickable,
                cx.listener(move |this, _event, _window, cx| {
                    this.radio_input.click_elevon(index);
                    cx.notify();
                }),
            )));
        }
        body = body.child(elevons);
    }

    // `groupBox1`, "Spektrum Bind", 207 by 42 at (418, 356).
    let mut bind = group(418.0, 356.0, 207.0, 42.0, "Spektrum Bind", radio.enabled);
    for (spektrum, id, text, x, width) in BIND {
        bind = bind.child(at(x, 15.0, width, 23.0).child(action_sized(
            id,
            text,
            theme::ACCENT,
            clickable,
            Some(px(width)),
            cx.listener(move |this, _event: &(), _window, cx| {
                this.radio_input.click_bind(spektrum);
                cx.notify();
            }),
        )));
    }
    body = body.child(bind);

    Some(panel("radio calibration", body).into_any_element())
}

/// The message box showing, drawn over the whole window as `CustomMessageBox.Show` is modal.
pub fn overlay(
    radio: &RadioInput,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let message = radio.message()?;
    let size = window.viewport_size();
    let dialog = crate::probe::measured("radio-message", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(380.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(if message.title == ERROR_TITLE {
            theme::ALERT
        } else {
            theme::WARN
        }))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(message.title),
        )
        .child(
            div()
                .flex()
                .flex_col()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .children(
                    message
                        .text
                        .lines()
                        .map(|line| div().child(line.to_owned())),
                ),
        )
        .child(div().flex().justify_end().child(action(
            "radio-message-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.radio_input.click_ok(&this.telemetry.view());
                cx.notify();
            }),
        )));
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("radio-message-backdrop")
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
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{Vehicle, ack, param, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;

    /// A vehicle's `cs.rateX`, its `raterc` told apart from every rate the page asks for itself
    /// and from the link's.
    const RATES: StreamRates = StreamRates {
        attitude: 4,
        position: 2,
        status: 2,
        sensors: 2,
        rc: 7,
    };

    /// `MAV_PARAM_TYPE_INT8`, as ArduPilot declares `RCn_REVERSED`.
    const INT8: u8 = 2;
    /// `MAV_PARAM_TYPE_INT16`, as it declares `RCn_MIN`, `_MAX` and `_TRIM`.
    const INT16: u8 = 4;
    /// `MAV_RESULT_ACCEPTED`.
    const ACCEPTED: u8 = 0;
    /// `MAV_RESULT_DENIED`.
    const DENIED: u8 = 2;

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// A 4.x copter's radio parameters, as SITL's defaults leave them.
    fn copter() -> Vec<(String, f64)> {
        let mut parameters = table(&[
            ("RCMAP_ROLL", 1.0),
            ("RCMAP_PITCH", 2.0),
            ("RCMAP_THROTTLE", 3.0),
            ("RCMAP_YAW", 4.0),
        ]);
        for channel in 1..=16 {
            parameters.push((format!("RC{channel}_REVERSED"), 0.0));
            parameters.push((format!("RC{channel}_MIN"), 1000.0));
        }
        parameters
    }

    /// `chNin`, channel 1 first, the rest zero.
    fn inputs(values: &[f32]) -> [f32; CHANNELS] {
        std::array::from_fn(|index| values.get(index).copied().unwrap_or(0.0))
    }

    /// SITL's receiver: eight channels, static
    /// (`libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp`, `set_default_pwm_input_values`).
    fn sitl() -> [f32; CHANNELS] {
        inputs(&[
            1500.0, 1500.0, 1000.0, 1500.0, 1800.0, 1000.0, 1000.0, 1800.0,
        ])
    }

    /// The link's waits, divided so the C#'s whole retry ladder runs in a blink.
    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// Dismisses the message showing, which must be `text`, on a vehicle whose rates are
    /// [`RATES`].
    fn ok(radio: &mut RadioInput, text: &str, reading: &[f32; CHANNELS]) {
        ok_at(radio, text, reading, RATES);
    }

    /// Dismisses the message showing, which must be `text`, on a vehicle whose rates are `rates`.
    fn ok_at(radio: &mut RadioInput, text: &str, reading: &[f32; CHANNELS], rates: StreamRates) {
        assert_eq!(
            radio.message().map(|message| message.text.as_str()),
            Some(text)
        );
        radio.dismiss(reading, rates);
    }

    /// Runs the queue against a scripted vehicle until nothing is under way, answering what the
    /// link sends with `answer`.
    fn drive(
        radio: &mut RadioInput,
        telemetry: &mut Telemetry,
        vehicle: &mut Vehicle,
        mut answer: impl FnMut(&mut Vehicle, &MavMessage),
    ) {
        until("the page's work to reach a message or the end", || {
            for message in vehicle.read() {
                answer(&mut *vehicle, &message);
            }
            radio.advance(telemetry);
            radio.waiting.is_none() && matches!(radio.tasks.front(), None | Some(Task::Say(..)))
        });
    }

    /// The `PARAM_SET`s the vehicle heard, as (name, value).
    fn sets(vehicle: &Vehicle) -> Vec<(String, f32)> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::ParamSet(set) => {
                    Some((mp_params::decode_param_id(&set.param_id), set.param_value))
                }
                _ => None,
            })
            .collect()
    }

    /// The `RC_CHANNELS` rates the vehicle was asked for, in order.
    fn streams(vehicle: &Vehicle) -> Vec<u16> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::RequestDataStream(request) if request.req_stream_id == 3 => {
                    Some(request.req_message_rate)
                }
                _ => None,
            })
            .collect()
    }

    /// Lists parameters on the link, as a download would.
    fn list(telemetry: &Telemetry, vehicle: &mut Vehicle, entries: &[(&str, f32, u8)]) {
        for (name, value, kind) in entries {
            vehicle.send(&param(name, *value, *kind));
        }
        until("the parameters to be listed", || {
            entries
                .iter()
                .all(|(name, _, _)| telemetry.holds_parameter(name))
        });
    }

    /// A `.resx` entry's value: the `<value>` after `<data name="NAME"`.
    fn resx_value<'a>(resx: &'a str, name: &str) -> Option<&'a str> {
        let data = resx.find(&format!("<data name=\"{name}\""))?;
        let rest = resx.get(data..)?;
        let open = rest.find("<value>")? + "<value>".len();
        let close = rest.find("</value>")?;
        rest.get(open..close)
    }

    /// Every control at the `.resx`'s `Location` and `Size`, read from the file when the C# tree
    /// is beside the workspace; and inside the page's 628 by 406 regardless.
    #[test]
    fn the_bars_and_boxes_are_where_the_resx_puts_them() {
        for spec in BARS {
            let (x, y, width, height) = spec.at;
            assert!(x + width <= PAGE.0 && y + height <= PAGE.1, "{spec:?}");
        }
                let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigRadioInput.resx",
        ) else {
            return;
        };
        let names = [
            "BARroll",
            "BARpitch",
            "BARthrottle",
            "BARyaw",
            "BAR5",
            "BAR6",
            "BAR7",
            "BAR8",
            "BAR9",
            "BAR10",
            "BAR11",
            "BAR12",
            "BAR13",
            "BAR14",
            "BAR15",
            "BAR16",
        ];
        for (name, spec) in names.iter().zip(BARS) {
            let (x, y, width, height) = spec.at;
            assert_eq!(
                resx_value(&resx, &format!("{name}.Location")),
                Some(format!("{x}, {y}").as_str()),
                "{name}"
            );
            assert_eq!(
                resx_value(&resx, &format!("{name}.Size")),
                Some(format!("{width}, {height}").as_str()),
                "{name}"
            );
        }
        for (index, (_, x, y)) in REVERSE.iter().enumerate() {
            let name = format!("CHK_revch{}", index + 1);
            assert_eq!(
                resx_value(&resx, &format!("{name}.Location")),
                Some(format!("{x}, {y}").as_str())
            );
            assert_eq!(resx_value(&resx, &format!("{name}.Text")), Some("Reverse"));
        }
        for (name, spec) in [
            "CHK_mixmode",
            "CHK_elevonrev",
            "CHK_elevonch1rev",
            "CHK_elevonch2rev",
        ]
        .iter()
        .zip(ELEVONS)
        {
            assert_eq!(
                resx_value(&resx, &format!("{name}.Location")),
                Some(format!("{}, 19", spec.x).as_str())
            );
            assert_eq!(resx_value(&resx, &format!("{name}.Text")), Some(spec.text));
        }
        for (name, (_, _, text, x, width)) in ["BUT_BindDSM2", "BUT_BindDSMX", "BUT_BindDSM8"]
            .iter()
            .zip(BIND)
        {
            assert_eq!(
                resx_value(&resx, &format!("{name}.Location")),
                Some(format!("{x}, 15").as_str())
            );
            assert_eq!(
                resx_value(&resx, &format!("{name}.Size")),
                Some(format!("{width}, 23").as_str())
            );
            assert_eq!(resx_value(&resx, &format!("{name}.Text")), Some(text));
        }
        assert_eq!(
            resx_value(&resx, "BUT_Calibrateradio.Text"),
            Some(CALIBRATE)
        );
        assert_eq!(
            resx_value(&resx, "BUT_Calibrateradio.Location"),
            Some("491, 327")
        );
        assert_eq!(resx_value(&resx, "$this.Size"), Some("628, 406"));
        assert_eq!(
            resx_value(&resx, "groupBoxElevons.Text"),
            Some("Elevon Config")
        );
        assert_eq!(resx_value(&resx, "groupBox1.Text"), Some("Spektrum Bind"));
    }

    #[test]
    fn a_bar_mirrors_its_value_when_reversed_and_skips_a_value_it_already_holds() {
        let mut bar = Bar::designer(&BARS[0]);
        assert_eq!(bar.value(), 1500);
        assert_eq!(bar.text(), "Roll  1500");
        bar.set(1100);
        assert_eq!(bar.value(), 1100);
        bar.reverse = true;
        // The setter compares with what it stored, so the same reading again changes nothing -
        // not even to mirror it.
        bar.set(1100);
        assert_eq!(bar.value(), 1100);
        bar.set(1200);
        assert_eq!(bar.value(), 1800, "2200 - (1200 - 800)");
        // Then 1800 read from the receiver equals what is stored, and is skipped.
        bar.set(1800);
        assert_eq!(bar.value(), 1800);
        // The fill's clamp: one above the minimum at the bottom.
        bar.reverse = false;
        bar.set(0);
        assert_eq!(bar.value(), 0);
        assert!((bar.fraction() - 1.0 / 1400.0).abs() < 1e-6);
        bar.set(2500);
        assert!((bar.fraction() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn the_red_lines_are_drawn_once_both_are_set() {
        let mut bar = Bar::designer(&BARS[0]);
        assert_eq!(bar.lines(Orientation::Horizontal, 280.0), None);
        bar.minline = 1100;
        bar.maxline = 1900;
        let near = |lines: Option<(f32, f32)>, want: (f32, f32)| {
            lines.is_some_and(|(min, max)| {
                (min - want.0).abs() < 1e-3 && (max - want.1).abs() < 1e-3
            })
        };
        assert!(near(
            bar.lines(Orientation::Horizontal, 280.0),
            (60.0, 220.0)
        ));
        bar.reverse = true;
        assert!(near(
            bar.lines(Orientation::Horizontal, 280.0),
            (220.0, 60.0)
        ));
        assert!(near(bar.lines(Orientation::Vertical, 280.0), (66.0, 220.0)));
        bar.reverse = false;
        assert!(near(bar.lines(Orientation::Vertical, 280.0), (220.0, 66.0)));
    }

    #[test]
    fn activate_binds_the_sticks_to_rcmap_and_labels_them() {
        let mut parameters = copter();
        for (name, value) in [("RCMAP_ROLL", 2.0), ("RCMAP_PITCH", 1.0)] {
            if let Some(entry) = parameters.iter_mut().find(|(held, _)| held == name) {
                entry.1 = value;
            }
        }
        let mut radio = RadioInput::default();
        radio.activate(&parameters, Firmware::ArduCopter2);
        assert_eq!(radio.map, [2, 1, 3, 4]);
        let bound: Vec<_> = radio.bindings.iter().map(|b| b.unwrap_or(0)).collect();
        assert_eq!(
            bound,
            [2, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        );
        assert_eq!(radio.bars[0].label, "Roll (rc2)");
        assert_eq!(radio.bars[1].label, "Pitch (rc1)");
        radio.observe(&sitl());
        let values: Vec<_> = radio.bars.iter().map(Bar::value).collect();
        assert_eq!(
            values,
            [
                1500, 1500, 1000, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
        // A second opening in the same visit adds to the labels again, as the C# does.
        radio.deactivate();
        radio.activate(&parameters, Firmware::ArduCopter2);
        assert_eq!(radio.bars[0].label, "Roll (rc2) (rc2)");
        // The stream asked for on each opening.
        assert_eq!(
            radio
                .tasks
                .iter()
                .filter(|task| **task == Task::Stream(2))
                .count(),
            2
        );
    }

    #[test]
    fn without_rcmap_the_sticks_are_the_first_four_channels() {
        let mut radio = RadioInput::default();
        radio.activate(&table(&[("RC1_REVERSED", 0.0)]), Firmware::ArduPlane);
        assert_eq!(radio.map, [1, 2, 3, 4]);
        assert!(radio.enabled);
    }

    #[test]
    fn part_of_the_map_missing_disables_the_page() {
        let mut radio = RadioInput::default();
        radio.activate(&table(&[("RCMAP_ROLL", 1.0)]), Firmware::ArduCopter2);
        assert!(!radio.enabled);
        assert_eq!(
            radio.message(),
            Some(&Message {
                title: "Error",
                text: "Error receiving list\n".to_owned()
            })
        );
        // Nothing after the map was done: no bindings, labels as designed, no stream.
        assert_eq!(radio.bindings, [None; CHANNELS]);
        assert_eq!(radio.bars[0].label, "Roll");
        radio.dismiss(&sitl(), RATES);
        assert!(radio.tasks.is_empty());
        // And it takes no click.
        radio.click_calibrate();
        assert!(radio.tasks.is_empty());
    }

    #[test]
    fn a_copter_hides_the_reverse_boxes_after_binding_them() {
        let mut radio = RadioInput::default();
        radio.activate(&copter(), Firmware::ArduCopter2);
        for check in &radio.reverse {
            assert!(!check.visible);
            assert!(check.enabled);
            assert_eq!(check.state, CheckState::Unchecked);
        }
        assert_eq!(radio.reverse[2].param.as_deref(), Some("RC3_REVERSED"));
        // And not on a copter, the elevons: the group is hidden.
        assert!(!radio.elevons_visible);
        // A hidden box takes no click.
        radio.tasks.clear();
        radio.click_reverse(0, &copter());
        assert!(radio.tasks.is_empty());
    }

    #[test]
    fn the_older_rev_parameter_is_bound_first_with_minus_one_for_reversed() {
        let parameters = table(&[
            ("RC1_REV", -1.0),
            ("RC1_REVERSED", 0.0),
            ("RC2_REVERSED", 1.0),
            ("RC3_REVERSED", 0.5),
        ]);
        let mut radio = RadioInput::default();
        radio.activate(&parameters, Firmware::ArduPlane);
        assert_eq!(radio.reverse[0].param.as_deref(), Some("RC1_REV"));
        assert_eq!(radio.reverse[0].state, CheckState::Checked);
        assert_eq!(radio.reverse[1].state, CheckState::Checked);
        assert_eq!(radio.reverse[2].state, CheckState::Indeterminate);
        // Channel 4 has neither: left unbound and disabled, but shown.
        assert_eq!(radio.reverse[3].param, None);
        assert!(!radio.reverse[3].enabled);
        assert!(radio.reverse[3].visible);
        // Only RCn_REVERSED at 1 mirrors a bar when the page opens - not RC1_REV at -1.
        let mirrored: Vec<_> = radio.bars.iter().take(4).map(|bar| bar.reverse).collect();
        assert_eq!(mirrored, [false, true, false, false]);
    }

    #[test]
    fn a_plane_shows_the_elevon_boxes_bound_to_what_it_has() {
        let mut radio = RadioInput::default();
        radio.activate(
            &table(&[("ELEVON_MIXING", 1.0), ("ELEVON_REVERSE", 0.0)]),
            Firmware::ArduPlane,
        );
        assert!(radio.elevons_visible);
        let states: Vec<_> = radio
            .elevons
            .iter()
            .map(|check| (check.enabled, check.state))
            .collect();
        assert_eq!(
            states,
            [
                (true, CheckState::Checked),
                (true, CheckState::Unchecked),
                (false, CheckState::Unchecked),
                (false, CheckState::Unchecked),
            ]
        );
        radio.tasks.clear();
        radio.click_elevon(0);
        assert_eq!(
            radio.tasks.front(),
            Some(&Task::Set {
                name: "ELEVON_MIXING".to_owned(),
                value: 0.0,
                force: false,
                then: Then::CheckBox
            })
        );
    }

    /// The conversation on SITL's static sticks, step by step: the messages in order, the button's
    /// text at each, the stream rates, what the loop folded, the trims, nothing to write, and the
    /// summary with only its rule.
    #[test]
    fn the_calibration_conversation_on_static_sticks_writes_nothing() {
        let mut telemetry = Telemetry::idle();
        let mut radio = RadioInput::default();
        radio.activate(&copter(), Firmware::ArduCopter2);
        radio.advance(&mut telemetry);
        assert_eq!(radio.stream, Some(2));
        assert_eq!(radio.button, "Calibrate Radio");

        radio.click_calibrate();
        radio.advance(&mut telemetry);
        assert_eq!(radio.button, "Calibrate Radio");
        ok(&mut radio, TRANSMITTER_ON, &sitl());
        assert_eq!(radio.button, "Click when Done");
        // C#: ConfigRadioInput.cs:214-217 - RC to 10 and attitude, position and status to 0,
        // before the RC request.
        assert!(
            matches!(radio.tasks.front(), Some(Task::Rates(r)) if r.rc == 10 && r.attitude == 0 && r.position == 0 && r.status == 0),
            "{:?}",
            radio.tasks.front()
        );
        radio.advance(&mut telemetry);
        assert_eq!(radio.stream, Some(10));
        ok(&mut radio, MOVE_STICKS, &sitl());
        assert!(radio.running);

        for _ in 0..3 {
            radio.observe(&sitl());
        }
        assert_eq!(radio.bars[0].minline, 1500);
        assert_eq!(radio.bars[2].maxline, 1000);
        assert_eq!(radio.bars[8].minline, 0, "BAR9 folded SITL's zero");

        radio.click_calibrate();
        assert_eq!(radio.button, "Completed");
        assert!(!radio.running);
        // The vehicle's `raterc` changed since the first message: the rate put back is still
        // `oldrc`, the one read then.
        ok_at(
            &mut radio,
            CENTRE_STICKS,
            &sitl(),
            StreamRates { rc: 9, ..RATES },
        );
        assert_eq!(radio.calibration.trim, sitl());
        assert_eq!(radio.button, "Saving");
        // C#: ConfigRadioInput.cs:388-391 - the four rates read at the first message put back,
        // before the RC request.
        let restored = radio
            .tasks
            .iter()
            .position(|task| matches!(task, Task::Rates(r) if *r == RATES));
        let stream = radio
            .tasks
            .iter()
            .position(|task| matches!(task, Task::Stream(_)));
        assert!(
            restored.is_some() && stream.is_some() && restored < stream,
            "{:?}",
            radio.tasks
        );
        radio.advance(&mut telemetry);
        assert_eq!(radio.stream, Some(RATES.rc), "oldrc, the vehicle's raterc");
        assert_eq!(
            radio.message(),
            Some(&Message {
                title: "Radio",
                text: "Here are the detected radio options\nNOTE Channels not connected are \
                       displayed as 1500 +-2\nNormal values are around 1100 | 1900\nChannel:Min \
                       | Max \n---------------\n"
                    .to_owned()
            })
        );
        assert_eq!(radio.last_write, None, "nothing written");
        radio.dismiss(&sitl(), RATES);
        assert_eq!(radio.button, "Completed");
        assert!(radio.idle());
    }

    #[test]
    fn no_real_channel_one_cancels_without_restoring_the_stream() {
        let mut telemetry = Telemetry::idle();
        let mut radio = RadioInput::default();
        radio.activate(&copter(), Firmware::ArduCopter2);
        radio.advance(&mut telemetry);
        radio.click_calibrate();
        radio.advance(&mut telemetry);
        ok(&mut radio, TRANSMITTER_ON, &sitl());
        radio.advance(&mut telemetry);
        ok(&mut radio, MOVE_STICKS, &sitl());
        // A receiver reporting nothing: channel 1 is zero, and nothing is folded.
        radio.observe(&inputs(&[]));
        radio.click_calibrate();
        ok(&mut radio, BAD_CHANNEL_ONE, &sitl());
        radio.advance(&mut telemetry);
        assert!(radio.idle());
        assert_eq!(radio.button, "Completed");
        // The C# returns before putting the rate back.
        assert_eq!(radio.stream, Some(10));
    }

    /// A swept radio on the real link: `RCn_MIN`, `RCn_MAX` and `RCn_TRIM` for each channel that
    /// moved, forced - sent even where the vehicle already holds the value - then `RC_CHANNELS`
    /// put back, then the summary.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:355-405`
    #[test]
    fn a_swept_radio_writes_min_max_and_trim_forced_then_the_summary() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        list(
            &telemetry,
            &mut vehicle,
            &[
                ("RC1_MIN", 1100.0, INT16),
                ("RC1_MAX", 1900.0, INT16),
                ("RC1_TRIM", 1500.0, INT16),
                ("RC3_MIN", 1000.0, INT16),
                ("RC3_MAX", 2000.0, INT16),
                ("RC3_TRIM", 1500.0, INT16),
            ],
        );
        let mut radio = RadioInput::default();
        radio.activate(&copter(), Firmware::ArduCopter2);
        drive(&mut radio, &mut telemetry, &mut vehicle, |_, _| {});
        // The vehicle's own `cs.raterc`, which the calibration reads as `oldrc`.
        let raterc = vehicle_rates(&telemetry.view()).rc;
        radio.click_calibrate();
        drive(&mut radio, &mut telemetry, &mut vehicle, |_, _| {});
        radio.click_ok(&telemetry.view());
        drive(&mut radio, &mut telemetry, &mut vehicle, |_, _| {});
        radio.click_ok(&telemetry.view());
        for (roll, throttle) in [(1500.0, 1000.0), (1100.0, 1500.0), (1900.0, 2000.0)] {
            radio.observe(&inputs(&[roll, 1500.0, throttle]));
        }
        radio.click_calibrate();
        radio.dismiss(
            &inputs(&[1500.0, 1500.0, 1000.0]),
            vehicle_rates(&telemetry.view()),
        );
        drive(
            &mut radio,
            &mut telemetry,
            &mut vehicle,
            |vehicle, message| {
                if let MavMessage::ParamSet(set) = message {
                    let name = mp_params::decode_param_id(&set.param_id);
                    vehicle.send(&param(&name, set.param_value, INT16));
                }
            },
        );

        assert_eq!(
            sets(&vehicle),
            [
                ("RC1_MIN".to_owned(), 1100.0),
                ("RC1_MAX".to_owned(), 1900.0),
                ("RC1_TRIM".to_owned(), 1500.0),
                ("RC3_MIN".to_owned(), 1000.0),
                ("RC3_MAX".to_owned(), 2000.0),
                ("RC3_TRIM".to_owned(), 1000.0),
            ]
        );
        until("the last stream request to go out", || {
            vehicle.read();
            streams(&vehicle).len() == 3
        });
        // Put back at the vehicle's `raterc` - `CurrentState`'s 2 unless the saved defaults say
        // otherwise - and not at anything of the link's own.
        assert_eq!(
            streams(&vehicle),
            [2, 10, u16::try_from(raterc).expect("a rate")]
        );
        assert_eq!(
            radio.message().map(|message| message.text.clone()),
            Some(format!(
                "{SUMMARY}---------------\nCH1 1100 | 1900\nCH3 1000 | 2000\n"
            ))
        );
        assert_eq!(radio.last_write.as_deref(), Some("RC3_TRIM 1000 accepted"));
    }

    /// A channel whose `RCn_MIN` is never echoed: its `_MAX` and `_TRIM` are not sent, "Failed to
    /// set Channel 1" holds the save until dismissed, and the next channel is written after it.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:359-383`
    #[test]
    fn a_channel_whose_write_times_out_is_abandoned_and_said() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        list(
            &telemetry,
            &mut vehicle,
            &[
                ("RC1_MIN", 1000.0, INT16),
                ("RC1_MAX", 2000.0, INT16),
                ("RC1_TRIM", 1500.0, INT16),
                ("RC2_MIN", 1000.0, INT16),
                ("RC2_MAX", 2000.0, INT16),
                ("RC2_TRIM", 1500.0, INT16),
            ],
        );
        let mut radio = RadioInput::default();
        radio.activate(&copter(), Firmware::ArduCopter2);
        radio.tasks.clear();
        radio.running = true;
        radio.active = true;
        for value in [1100.0, 1900.0] {
            radio.observe(&inputs(&[value, value]));
        }
        radio.click_calibrate();
        radio.dismiss(&inputs(&[1500.0, 1500.0]), RATES);
        let answer = |vehicle: &mut Vehicle, message: &MavMessage| {
            if let MavMessage::ParamSet(set) = message {
                let name = mp_params::decode_param_id(&set.param_id);
                if name != "RC1_MIN" {
                    vehicle.send(&param(&name, set.param_value, INT16));
                }
            }
        };
        drive(&mut radio, &mut telemetry, &mut vehicle, answer);
        assert_eq!(
            radio.message().map(|message| message.text.as_str()),
            Some("Failed to set Channel 1")
        );
        let names: Vec<String> = sets(&vehicle).into_iter().map(|(name, _)| name).collect();
        assert!(names.iter().all(|name| name == "RC1_MIN"), "{names:?}");
        assert_eq!(names.len(), 4, "the first send and three retries");
        radio.dismiss(&sitl(), RATES);
        drive(&mut radio, &mut telemetry, &mut vehicle, answer);
        let names: Vec<String> = sets(&vehicle).into_iter().map(|(name, _)| name).collect();
        assert_eq!(
            names.get(4..),
            Some(
                &[
                    "RC2_MIN".to_owned(),
                    "RC2_MAX".to_owned(),
                    "RC2_TRIM".to_owned()
                ][..]
            )
        );
        assert_eq!(radio.message().map(|message| message.title), Some("Radio"));
        // The summary lists both: the failed channel's line is appended after its catch.
        assert!(
            radio.message().is_some_and(|message| message
                .text
                .ends_with("CH1 1100 | 1900\nCH2 1100 | 1900\n"))
        );
    }

    /// A Reverse box on a plane, through the real link: `RC1_REVERSED` written 1 and the roll bar
    /// mirrored, then written back to 0.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:140-147, 419-452; Controls/MavlinkCheckBox.cs:106-143`
    #[test]
    fn a_reverse_box_writes_its_parameter_and_mirrors_its_bar() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        list(&telemetry, &mut vehicle, &[("RC1_REVERSED", 0.0, INT8)]);
        let parameters = table(&[("RC1_REVERSED", 0.0)]);
        let mut radio = RadioInput::default();
        radio.activate(&parameters, Firmware::ArduPlane);
        let echo = |vehicle: &mut Vehicle, message: &MavMessage| {
            if let MavMessage::ParamSet(set) = message {
                let name = mp_params::decode_param_id(&set.param_id);
                vehicle.send(&param(&name, set.param_value, INT8));
            }
        };
        drive(&mut radio, &mut telemetry, &mut vehicle, echo);

        radio.click_reverse(0, &parameters);
        assert!(radio.bars[0].reverse);
        drive(&mut radio, &mut telemetry, &mut vehicle, echo);
        assert_eq!(radio.last_write.as_deref(), Some("RC1_REVERSED 1 accepted"));
        assert_eq!(radio.message(), None);

        radio.click_reverse(0, &parameters);
        assert!(!radio.bars[0].reverse);
        drive(&mut radio, &mut telemetry, &mut vehicle, echo);
        assert_eq!(radio.last_write.as_deref(), Some("RC1_REVERSED 0 accepted"));
        assert_eq!(
            sets(&vehicle),
            [
                ("RC1_REVERSED".to_owned(), 1.0),
                ("RC1_REVERSED".to_owned(), 0.0)
            ]
        );
    }

    /// With `SWITCH_ENABLE` at 1 a Reverse click first writes it 0 and says "Disabled Dip
    /// Switchs", and only after that is dismissed writes the box's own parameter; a box whose
    /// write is never echoed says "Set NAME Failed".
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:454-468; Controls/MavlinkCheckBox.cs:114-125`
    #[test]
    fn the_dip_switches_are_disabled_before_the_reverse_is_written() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        list(
            &telemetry,
            &mut vehicle,
            &[("SWITCH_ENABLE", 1.0, INT8), ("RC2_REVERSED", 0.0, INT8)],
        );
        let parameters = table(&[("SWITCH_ENABLE", 1.0), ("RC2_REVERSED", 0.0)]);
        let mut radio = RadioInput::default();
        radio.activate(&parameters, Firmware::ArduPlane);
        drive(&mut radio, &mut telemetry, &mut vehicle, |_, _| {});
        radio.click_reverse(1, &parameters);
        // SWITCH_ENABLE is echoed; RC2_REVERSED never is.
        let answer = |vehicle: &mut Vehicle, message: &MavMessage| {
            if let MavMessage::ParamSet(set) = message {
                let name = mp_params::decode_param_id(&set.param_id);
                if name == "SWITCH_ENABLE" {
                    vehicle.send(&param(&name, set.param_value, INT8));
                }
            }
        };
        drive(&mut radio, &mut telemetry, &mut vehicle, answer);
        assert_eq!(
            radio.message().map(|message| message.text.as_str()),
            Some(DIP_DISABLED)
        );
        assert_eq!(sets(&vehicle), [("SWITCH_ENABLE".to_owned(), 0.0)]);
        radio.dismiss(&sitl(), RATES);
        drive(&mut radio, &mut telemetry, &mut vehicle, answer);
        assert_eq!(
            radio.message(),
            Some(&Message {
                title: "Error",
                text: "Set RC2_REVERSED Failed".to_owned()
            })
        );
        assert_eq!(
            sets(&vehicle).last(),
            Some(&("RC2_REVERSED".to_owned(), 1.0))
        );
    }

    /// Each bind button sends `START_RX_PAIR` with its protocol; accepted or refused, the C# says
    /// to put the transmitter in bind mode, and only a command never answered is an error.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:471-508`
    #[test]
    fn a_bind_says_the_receiver_is_waiting_unless_it_times_out() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut radio = RadioInput::default();
        radio.activate(&copter(), Firmware::ArduCopter2);
        drive(&mut radio, &mut telemetry, &mut vehicle, |_, _| {});
        for (spektrum, result) in [
            (Spektrum::Dsm2, Some(ACCEPTED)),
            (Spektrum::DsmX, Some(DENIED)),
            (Spektrum::Dsm8, None),
        ] {
            radio.click_bind(spektrum);
            drive(
                &mut radio,
                &mut telemetry,
                &mut vehicle,
                |vehicle, message| {
                    if let (MavMessage::CommandLong(long), Some(result)) = (message, result) {
                        assert_eq!(long.param2, spektrum.param2());
                        vehicle.send(&ack(long.command, result));
                    }
                },
            );
            let expected = if result.is_some() {
                BIND_WAITING
            } else {
                BIND_ERROR
            };
            assert_eq!(
                radio.message().map(|message| message.text.as_str()),
                Some(expected),
                "{spektrum:?}"
            );
            radio.dismiss(&sitl(), RATES);
        }
        assert_eq!(
            radio.last_bind.as_deref(),
            Some("START_RX_PAIR 2 timed out")
        );
        let binds = vehicle
            .heard
            .iter()
            .filter(
                |message| matches!(message, MavMessage::CommandLong(long) if long.command == 500),
            )
            .count();
        assert_eq!(
            binds,
            1 + 1 + 4,
            "the unanswered one sent again three times"
        );
    }

    /// The page is opened and closed by the list, drawn in its arm of `page_body`, ticked, its
    /// facts published and its message box drawn over the window - the application's own path,
    /// which a test of the page alone does not take.
    #[test]
    fn the_page_is_wired_into_the_application() {
        let main = include_str!("../main.rs");
        for wiring in [
            ".tick(&mut self.telemetry, &view, self.screen == Screen::Setup)",
            "config::radio::record_facts(&self.radio_input, &view)",
            "config::radio::overlay(&self.radio_input, window, cx)",
        ] {
            assert!(main.contains(wiring), "main.rs lacks {wiring}");
        }
        let setup = include_str!("../setup.rs");
        for wiring in [
            "Some(\"ConfigRadioInput\") if !self.radio_input.is_active() => {\n                \
             self.radio_input.toggle(&self.telemetry);",
            "Some(\"ConfigRadioInput\") if self.radio_input.is_active() => {\n                \
             self.radio_input.deactivate();",
            "crate::config::radio::page(&self.radio_input, cx)",
        ] {
            assert!(setup.contains(wiring), "setup.rs lacks {wiring}");
        }
    }

    #[test]
    fn leaving_the_setup_screen_makes_the_page_anew_unless_it_is_working() {
        let mut telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut radio = RadioInput::default();
        radio.activate(&copter(), Firmware::ArduCopter2);
        radio.tick(&mut telemetry, &view, true);
        radio.deactivate();
        radio.tasks.clear();
        radio.running = true;
        radio.tick(&mut telemetry, &view, false);
        assert!(radio.touched, "a running calibration keeps its page");
        radio.running = false;
        radio.tick(&mut telemetry, &view, false);
        assert!(!radio.touched);
        assert_eq!(radio.bars[0].label, "Roll");
    }
}
