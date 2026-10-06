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

//! Frame Type: the multicopter's frame class and the motor layout within it.
//!
//! Mission Planner's `ConfigFrameClassType`, a page of Initial Setup under Mandatory Hardware,
//! listed before Accel Calibration (`GCSViews/InitialSetup.cs:184-192`). Two group boxes: "Frame
//! Class", eight push-button radio buttons - Undefined, Quad, Hexa, Octa, OctaQuad, Y6, Heli, Tri -
//! each writing `FRAME_CLASS`; and "Frame Type", six radio buttons with a picture of each layout -
//! Plus, X, V, H, V Tail, Y6B - and an "Other" button, each writing `FRAME_TYPE`. Which types a
//! class takes is `ArduPilot.Common.ValidList`: the ones it does not are disabled and their
//! pictures faded, and a class that takes none disables the whole group.
//!
//! A click writes at once, with no confirmation: `SetFrameParam` sets `FRAME_CLASS` and then
//! `FRAME_TYPE`, both, whichever was clicked, each blocking until the vehicle echoes it. A write
//! that times out ends that pair and shows "Set FRAME_CLASS OR FRAME_TYPE Failed". Opening the
//! page runs the same code on the vehicle's own values, so it writes them back - which
//! `setParam`'s "not modified as same" turns into nothing on the wire.
//!
//! What the C# does on the UI thread - two blocking `setParam` calls per click - is a queue here,
//! advanced once a frame, one write at a time and in the same order.
//!
//! The geometry is `ConfigFrameClassType.resx`'s: the class buttons 74 by 74 along the top of a
//! 652 by 100 group, the types down the left of a 504 by 420 group, each control at its
//! `Location`. The colours are this application's.
//!
//! The pictures are the C#'s ([`crate::pictures`], resources named in [`CLASSES`] and [`TYPES`]):
//! each type's zoomed in its box at the opacity the C# gives it, each class button's stretched to
//! 60 by 60 as `new Bitmap(image, 60, 60)` makes it; V Tail's box has no image in the C# and is
//! empty here.
//!
//! What is not ported, and why:
//!
//! * the fade itself, a 400 ms linear `Transition` (`ConfigFrameClassType.cs:295-300`), which is
//!   drawn at the opacity it ends on;
//! * `ConfigFrameType`, the page a copter older than 3.5 gets instead (`InitialSetup.cs:188`):
//!   a separate page, for firmware that has no `FRAME_CLASS`, in
//!   [`crate::config::frame_type_legacy`];
//! * there is no "show all" or per-vehicle filter in the C#: the eight classes and six types are
//!   fixed controls, and `ValidList` is the only filter.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, Div, SharedString, div, prelude::*, px, rgb};
use mp_link::requests::RequestOutcome;

use crate::MissionPlanner;
use crate::config::flight_modes::{Firmware, ParamWriter, Progress, firmware_of};
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// `DisabledOpacity`: a picture whose type the class does not take, or that is not chosen.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:18`
pub const DISABLED_OPACITY: f32 = 0.2;
/// `EnabledOpacity`.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:19`
pub const ENABLED_OPACITY: f32 = 1.0;

/// `MOTOR_FRAME_TYPE_VTAIL`, the one type radio `DoType`'s fallback does not uncheck.
/// `// C#: ExtLibs/ArduPilot/motor_frame_type.cs:12`
const TYPE_VTAIL: i32 = 4;

/// The two parameters the page reads and writes.
const WATCHED: [&str; 2] = ["FRAME_CLASS", "FRAME_TYPE"];

/// What `SetFrameParam` shows when either write fails: `Strings.ErrorSetValueFailed` with
/// "FRAME_CLASS OR FRAME_TYPE", under `Strings.ERROR`.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:288-292; ExtLibs/Strings/Strings.resx:130-132, 171-173`
pub const FAILED: &str = "Set FRAME_CLASS OR FRAME_TYPE Failed";
/// That message's title.
const ERROR_TITLE: &str = "Error";

/// The two notes beside the H and Y6B rows: `label4` and `label7`.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.resx (label4.Text, label7.Text)`
const NOTE_H: &str = "NOTE: X and H are NOT interchangable the prop rotation changes";
/// `label7.Text`.
const NOTE_Y6B: &str = "NOTE: This is the Y6B and prop rotation changes from the old Y6A";

/// A Frame Class button: `radioButton<Name>` in `groupBox3`, `Appearance.Button`, 74 by 74 at
/// `Y` 18, its image resized to 60 by 60 above its text.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClassButton {
    /// The `motor_frame_class` value it selects.
    pub class: i32,
    /// Its text.
    pub text: &'static str,
    /// Its `Location.X` in the group.
    pub x: f32,
    /// The resource the C# draws on it ([`crate::pictures`]); the Undefined button has none.
    pub image: Option<&'static str>,
}

/// The Frame Class buttons, left to right.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:27-33, 84-110, 318-336; ConfigFrameClassType.Designer.cs:255-340 (radioButton*.Image: 261, 272, 283, 294, 305, 316, 327); ConfigFrameClassType.resx (radioButton*.Location, .Text)`
pub const CLASSES: [ClassButton; 8] = [
    ClassButton {
        class: 0,
        text: "Undefined",
        x: 6.0,
        image: None,
    },
    ClassButton {
        class: 1,
        text: "Quad",
        x: 86.0,
        image: Some("FW_icons_2013_logos_03"),
    },
    ClassButton {
        class: 2,
        text: "Hexa",
        x: 166.0,
        image: Some("FW_icons_2013_logos_09"),
    },
    ClassButton {
        class: 3,
        text: "Octa",
        x: 246.0,
        image: Some("FW_icons_2013_logos_12"),
    },
    ClassButton {
        class: 4,
        text: "OctaQuad",
        x: 326.0,
        image: Some("FW_icons_2013_logos_06"),
    },
    ClassButton {
        class: 5,
        text: "Y6",
        x: 406.0,
        image: Some("FW_icons_2013_logos_07"),
    },
    ClassButton {
        class: 6,
        text: "Heli",
        x: 486.0,
        image: Some("FW_icons_2013_logos_13"),
    },
    ClassButton {
        class: 7,
        text: "Tri",
        x: 566.0,
        image: Some("FW_icons_2013_logos_08"),
    },
];

/// A Frame Type row: `radioButton_<name>`, its label, and its `PictureBoxWithPseudoOpacity`, all
/// in `groupBox2`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TypeOption {
    /// The `motor_frame_type` value it selects.
    pub frame_type: i32,
    /// The radio button's name, after `radioButton_`.
    pub name: &'static str,
    /// The label's text, quotes and all.
    pub label: &'static str,
    /// The label's `Location`.
    pub label_at: (f32, f32),
    /// The radio button's `Location`; each is 14 by 13.
    pub radio_at: (f32, f32),
    /// The picture's `Location` and `Size`.
    pub picture: (f32, f32, f32, f32),
    /// The resource the C# draws there ([`crate::pictures`]). V Tail's picture box has none.
    pub image: Option<&'static str>,
    /// Whether a click on the picture chooses the type: every picture but V Tail's is wired to
    /// `radioButtonType_CheckedChanged`.
    pub picture_clicks: bool,
}

/// The Frame Type rows, in the order `DoType` and the click handler take them.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:178-277, 302-316; ConfigFrameClassType.Designer.cs:75-239 (pictureBox*.Image: 103, 112, 134, 156, 183); ConfigFrameClassType.resx (label1-9, radioButton_*, pictureBox* .Location, .Size, .SizeMode, .Text)`
pub const TYPES: [TypeOption; 6] = [
    TypeOption {
        frame_type: 0,
        name: "Plus",
        label: "'Plus'",
        label_at: (25.0, 38.0),
        radio_at: (62.0, 38.0),
        picture: (83.0, 9.0, 248.0, 75.0),
        image: Some("frames_plus"),
        picture_clicks: true,
    },
    TypeOption {
        frame_type: 1,
        name: "X",
        label: "'X' , 'Y6A'",
        label_at: (8.0, 118.0),
        radio_at: (62.0, 118.0),
        picture: (83.0, 88.0, 406.0, 78.0),
        image: Some("frames_x"),
        picture_clicks: true,
    },
    TypeOption {
        frame_type: 2,
        name: "V",
        label: "'V'",
        label_at: (38.0, 203.0),
        radio_at: (62.0, 203.0),
        picture: (83.0, 173.0, 111.0, 78.0),
        image: Some("new_3DR_04"),
        picture_clicks: true,
    },
    TypeOption {
        frame_type: 3,
        name: "H",
        label: "'H'",
        label_at: (38.0, 287.0),
        radio_at: (62.0, 287.0),
        picture: (83.0, 257.0, 111.0, 78.0),
        image: Some("frames_h"),
        picture_clicks: true,
    },
    TypeOption {
        frame_type: 10,
        name: "Y",
        label: "'Y6B'",
        label_at: (19.0, 371.0),
        radio_at: (62.0, 371.0),
        picture: (83.0, 341.0, 111.0, 77.0),
        image: Some("y6b"),
        picture_clicks: true,
    },
    TypeOption {
        frame_type: TYPE_VTAIL,
        name: "VTail",
        label: "'V Tail'",
        label_at: (313.0, 203.0),
        radio_at: (357.0, 203.0),
        picture: (378.0, 173.0, 111.0, 78.0),
        image: None,
        picture_clicks: false,
    },
];

/// `radio_type_other`: "Other", at 357, 272. Checked in the designer and given no handler.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.Designer.cs:342-348; ConfigFrameClassType.resx (radio_type_other.Location, .Text)`
const OTHER: (&str, f32, f32) = ("Other", 357.0, 272.0);

/// `ArduPilot.Common.ValidList`: each class, and each type it takes; `None` is a class that
/// takes none. DodecaHexa is listed with two types and with none.
/// `// C#: ExtLibs/ArduPilot/Common.cs:15-85`
pub const VALID_LIST: [(i32, Option<i32>); 29] = [
    (1, Some(0)),
    (1, Some(1)),
    (1, Some(2)),
    (1, Some(3)),
    (1, Some(4)),
    (1, Some(5)),
    (2, Some(0)),
    (2, Some(1)),
    (3, Some(0)),
    (3, Some(1)),
    (3, Some(2)),
    (3, Some(3)),
    (4, Some(0)),
    (4, Some(1)),
    (4, Some(2)),
    (4, Some(3)),
    (12, Some(0)),
    (12, Some(1)),
    (5, Some(10)),
    (5, Some(1)),
    (6, None),
    (7, None),
    (8, None),
    (9, None),
    (10, None),
    (11, None),
    (12, None),
    (13, None),
    (0, None),
];

/// The types `ValidList` gives a class, in its order.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:72`
#[must_use]
pub fn valid_types(frame_class: i32) -> Vec<Option<i32>> {
    VALID_LIST
        .iter()
        .filter(|(class, _)| *class == frame_class)
        .map(|(_, frame_type)| *frame_type)
        .collect()
}

/// `groupBox2.Enabled`: off for a class `ValidList` does not list, or lists only with no type.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:74-82`
#[must_use]
pub fn types_enabled(frame_class: i32) -> bool {
    let valid = valid_types(frame_class);
    !(valid.is_empty() || valid.len() == 1 && valid.first() == Some(&None))
}

/// Where a type sits in [`TYPES`], for the six that have a radio button.
fn type_index(frame_type: i32) -> Option<usize> {
    TYPES
        .iter()
        .position(|option| option.frame_type == frame_type)
}

/// A parameter's value as `Enum.Parse(typeof(...), param.ToString())` reads it: a whole number
/// parses to that value whether or not the enum names it; anything else throws.
/// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:45-48; ExtLibs/Mavlink/MAVLinkParam.cs:219-224`
fn parse(value: f64) -> Option<i32> {
    let whole = value.is_finite()
        && value.fract() == 0.0
        && value >= f64::from(i32::MIN)
        && value <= f64::from(i32::MAX);
    #[allow(clippy::cast_possible_truncation)] // checked whole and in range above
    whole.then_some(value as i32)
}

/// A parameter's value, if the vehicle has listed it.
fn value_of(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// Whether Initial Setup lists the page: the vehicle has `FRAME_CLASS`, or it is a connected
/// copter of 3.5 or later with every parameter in.
///
/// The firmware is read from the heartbeat alone, not the version banner, since the setup strip
/// has only the view: that differs from `setAPType` for a blimp, which the banner calls a copter.
///
/// One condition the C# does not have, as the FailSafe page has it: `gotAllParams` needs a table
/// that is not empty. Mission Planner downloads the parameters as part of connecting, so none
/// received of none reported does not arise there; here they are downloaded when asked for.
/// `// C#: GCSViews/InitialSetup.cs:79-87, 119-131, 189-191; ExtLibs/ArduPilot/CurrentState.cs:2360-2374, 4406`
#[must_use]
pub fn available(view: &TelemetryView) -> bool {
    let has_frame_class = value_of(&view.parameters, "FRAME_CLASS").is_some();
    let got_all_params = !view.parameters.is_empty()
        && view.parameters.len() >= usize::from(view.parameters_expected);
    let copter_35_plus = view.state.as_deref().is_some_and(|state| {
        let [major, minor, ..] = state.autopilot_info.version;
        view.connected
            && firmware_of(state.autopilot, state.vehicle_type, None) == Firmware::ArduCopter2
            && (major, minor) >= (3, 5)
    });
    has_frame_class || copter_35_plus && got_all_params
}

/// One `SetFrameParam(frame_class, frame_type)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Call {
    /// `FRAME_CLASS`.
    pub frame_class: i32,
    /// `FRAME_TYPE`.
    pub frame_type: i32,
}

impl Call {
    /// Its two writes, in the C#'s order: `FRAME_CLASS`, then `FRAME_TYPE`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:281-287`
    #[must_use]
    pub fn writes(self) -> [(&'static str, f64); 2] {
        [
            ("FRAME_CLASS", f64::from(self.frame_class)),
            ("FRAME_TYPE", f64::from(self.frame_type)),
        ]
    }
}

/// The call being written: which of its writes is next, and the one waiting for an answer.
#[derive(Debug, Clone, Copy)]
struct Current<H> {
    call: Call,
    step: usize,
    waiting: Option<(&'static str, f64, H)>,
}

/// `SetFrameParam`'s writes, one at a time and in order, as the C#'s blocking `setParam` calls
/// go; and the message boxes a failed call shows.
#[derive(Debug)]
pub struct Writes<H> {
    calls: VecDeque<Call>,
    current: Option<Current<H>>,
    messages: VecDeque<String>,
    last: Option<String>,
}

impl<H> Default for Writes<H> {
    fn default() -> Self {
        Self {
            calls: VecDeque::new(),
            current: None,
            messages: VecDeque::new(),
            last: None,
        }
    }
}

impl<H: Copy> Writes<H> {
    /// Queues a `SetFrameParam`.
    pub fn call(&mut self, call: Call) {
        self.calls.push_back(call);
    }

    /// The writes not yet started, in order.
    #[must_use]
    pub fn queued(&self) -> Vec<(&'static str, f64)> {
        let started = self.current.map(|current| {
            current
                .call
                .writes()
                .into_iter()
                .skip(current.step + usize::from(current.waiting.is_some()))
        });
        started
            .into_iter()
            .flatten()
            .chain(self.calls.iter().flat_map(|call| call.writes()))
            .collect()
    }

    /// How many writes are still to be answered: the queued ones and the one waiting.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queued().len()
            + usize::from(
                self.current
                    .is_some_and(|current| current.waiting.is_some()),
            )
    }

    /// How the last write ended.
    #[must_use]
    pub fn last(&self) -> Option<&str> {
        self.last.as_deref()
    }

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&str> {
        self.messages.front().map(String::as_str)
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Moves the writes on: collects an answered one and starts the next.
    ///
    /// A write that times out ends its call - `FRAME_TYPE` is not written after a `FRAME_CLASS`
    /// that timed out - and shows [`FAILED`], as `setParam`'s `TimeoutException` does inside
    /// `SetFrameParam`'s `try`. One the vehicle does not have, or already holds the value of, does
    /// not: `setParam` returns false or true for those and `SetFrameParam` does not look.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:281-293; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1637-1648, 1762`
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W) {
        loop {
            let mut current = match self.current.take() {
                Some(current) => current,
                None => match self.calls.pop_front() {
                    Some(call) => Current {
                        call,
                        step: 0,
                        waiting: None,
                    },
                    None => return,
                },
            };
            if let Some((name, value, handle)) = current.waiting {
                match writer.progress(handle) {
                    Progress::Waiting => {
                        self.current = Some(current);
                        return;
                    }
                    Progress::Lost => {
                        self.fail(name, value, "lost");
                        continue;
                    }
                    Progress::Finished(outcome) => {
                        if let Err(why) = self.settle(name, value, outcome) {
                            self.fail(name, value, &why);
                            continue;
                        }
                        current.step += 1;
                        current.waiting = None;
                    }
                }
            }
            // Both written: the call is over, and the next one starts.
            let Some((name, value)) = current.call.writes().get(current.step).copied() else {
                continue;
            };
            match writer.write(name, value) {
                Some(handle) => {
                    current.waiting = Some((name, value, handle));
                    self.current = Some(current);
                }
                None => self.fail(name, value, "no vehicle"),
            }
        }
    }

    /// Records how a write ended; an error is the C#'s exception.
    fn settle(&mut self, name: &str, value: f64, outcome: RequestOutcome) -> Result<(), String> {
        self.last = Some(match outcome {
            RequestOutcome::Accepted { value: echoed } => {
                let echoed = echoed.map_or(value, |echoed| echoed.as_f64());
                format!("{name} {echoed} accepted")
            }
            // "not modified as same": `setParam` returns true without sending.
            RequestOutcome::Unchanged | RequestOutcome::Sent => format!("{name} {value} unchanged"),
            // "Trying to set Param that doesnt exist": false, and no exception.
            RequestOutcome::UnknownParameter => format!("{name} {value} not on the vehicle"),
            RequestOutcome::TimedOut => return Err("timed out".to_owned()),
            RequestOutcome::Rejected(result) => return Err(format!("rejected {result}")),
        });
        Ok(())
    }

    /// A call that threw: the rest of it is not written, and the message box shows.
    fn fail(&mut self, name: &str, value: f64, why: &str) {
        self.current = None;
        self.last = Some(format!("{name} {value} failed: {why}"));
        self.messages.push_back(FAILED.to_owned());
    }
}

/// Which Frame Type radio button is checked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TypeRadio {
    /// One of [`TYPES`], by value.
    Type(i32),
    /// `radio_type_other`.
    Other,
}

/// The page's state: what `Activate` read, what the controls show, and the writes.
#[derive(Debug)]
pub struct FrameType {
    /// Between `Activate` and `Deactivate`: the page is showing.
    active: bool,
    /// The page's `Enabled`: false when the vehicle lacks `FRAME_CLASS` or `FRAME_TYPE`.
    enabled: bool,
    /// Whether `Activate` read both values.
    read: bool,
    /// `work_frame_class`.
    frame_class: i32,
    /// `work_frame_type`.
    frame_type: i32,
    /// The class button that is down; none for a class without a button.
    checked_class: Option<i32>,
    /// The type radio button that is checked.
    checked_type: Option<TypeRadio>,
    /// `groupBox2.Enabled`.
    types_enabled: bool,
    /// Each type radio button's `Enabled`, in [`TYPES`]' order.
    radios: [bool; TYPES.len()],
    /// Whether each picture is at [`ENABLED_OPACITY`] rather than [`DISABLED_OPACITY`].
    bright: [bool; TYPES.len()],
    writes: Writes<mp_link::RequestId>,
}

impl Default for FrameType {
    /// As the designer leaves it: everything enabled, every picture opaque, "Other" checked.
    fn default() -> Self {
        Self {
            active: false,
            enabled: true,
            read: false,
            frame_class: 0,
            frame_type: 0,
            checked_class: None,
            checked_type: Some(TypeRadio::Other),
            types_enabled: true,
            radios: [true; TYPES.len()],
            bright: [true; TYPES.len()],
            writes: Writes::default(),
        }
    }
}

impl FrameType {
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
            self.activate(&telemetry.view().parameters);
        }
    }

    /// `Activate`: disables the page for a vehicle without both parameters; otherwise reads
    /// them and runs `DoClass` and `DoType` on them, which set the controls and write the values
    /// back.
    ///
    /// A value that is not a whole number stops it where `Enum.Parse` throws, and
    /// `BackstageView` logs the exception and shows the page as it is.
    ///
    /// Each opening starts from the designer's state, as the first opening after Initial Setup
    /// loads does; the C# keeps the control between openings within one visit, which matters
    /// only to which radio button is left checked for a type none of them names.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:36-54; ExtLibs/Controls/BackstageView/BackstageView.cs:495-503`
    pub fn activate(&mut self, parameters: &[(String, f64)]) {
        *self = Self {
            active: true,
            writes: std::mem::take(&mut self.writes),
            ..Self::default()
        };
        let (Some(frame_class), Some(frame_type)) = (
            value_of(parameters, "FRAME_CLASS"),
            value_of(parameters, "FRAME_TYPE"),
        ) else {
            self.enabled = false;
            return;
        };
        let Some(frame_class) = parse(frame_class) else {
            return;
        };
        self.frame_class = frame_class;
        let Some(frame_type) = parse(frame_type) else {
            return;
        };
        self.frame_type = frame_type;
        self.read = true;
        self.do_class(frame_class);
        self.do_type(frame_type);
    }

    /// `Deactivate`, which does nothing: the writes carry on.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:56-59`
    pub fn deactivate(&mut self) {
        self.active = false;
    }

    /// Once a frame: moves the writes on.
    pub fn tick(&mut self, telemetry: &Telemetry) {
        self.writes.advance(telemetry);
    }

    /// `DoClass`: checks the class's button, enables the type group if the class takes any
    /// type, enables and brightens the types it takes and disables and fades the rest, and
    /// writes the class with the current type.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:61-167`
    fn do_class(&mut self, frame_class: i32) {
        self.frame_class = frame_class;
        self.types_enabled = types_enabled(frame_class);
        if CLASSES.iter().any(|button| button.class == frame_class) {
            self.checked_class = Some(frame_class);
        }
        self.radios = [false; TYPES.len()];
        self.bright = [false; TYPES.len()];
        for index in valid_types(frame_class)
            .into_iter()
            .flatten()
            .filter_map(type_index)
        {
            if let Some(radio) = self.radios.get_mut(index) {
                *radio = true;
            }
            if let Some(bright) = self.bright.get_mut(index) {
                *bright = true;
            }
        }
        self.writes.call(Call {
            frame_class: self.frame_class,
            frame_type: self.frame_type,
        });
    }

    /// `DoType`: for one of the six, checks its radio button, brightens its picture alone and
    /// writes the class with it; for any other type, unchecks five of the six - not V Tail - and
    /// writes nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:169-279`
    fn do_type(&mut self, frame_type: i32) {
        self.frame_type = frame_type;
        match type_index(frame_type) {
            Some(index) => {
                self.bright = std::array::from_fn(|at| at == index);
                self.checked_type = Some(TypeRadio::Type(frame_type));
                self.writes.call(Call {
                    frame_class: self.frame_class,
                    frame_type,
                });
            }
            None => {
                if let Some(TypeRadio::Type(checked)) = self.checked_type
                    && checked != TYPE_VTAIL
                {
                    self.checked_type = None;
                }
            }
        }
    }

    /// Whether the page's controls take clicks at all.
    const fn takes_clicks(&self) -> bool {
        self.active && self.enabled
    }

    /// A class button pressed: `radioButtonClass_CheckedChanged`. The one already down raises
    /// no `CheckedChanged`, so pressing it again does nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:318-336`
    pub fn click_class(&mut self, frame_class: i32) {
        if !self.takes_clicks()
            || self.checked_class == Some(frame_class)
            || !CLASSES.iter().any(|button| button.class == frame_class)
        {
            return;
        }
        self.do_class(frame_class);
    }

    /// A type radio button clicked: `radioButtonType_CheckedChanged`. A disabled one, or one in
    /// a disabled group, takes no click; the checked one raises no `CheckedChanged`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:302-316`
    pub fn click_radio(&mut self, frame_type: i32) {
        let Some(index) = type_index(frame_type) else {
            return;
        };
        if !self.takes_clicks()
            || !self.types_enabled
            || !self.radios.get(index).copied().unwrap_or(false)
            || self.checked_type == Some(TypeRadio::Type(frame_type))
        {
            return;
        }
        self.do_type(frame_type);
    }

    /// A picture clicked: the same handler, from `Click`. A picture is only ever faded, never
    /// disabled, so a faded one still chooses its type - unless the whole group is disabled.
    /// V Tail's picture has no handler.
    /// `// C#: GCSViews/ConfigurationView/ConfigFrameClassType.Designer.cs:107, 116, 138, 160, 187, 234-239`
    pub fn click_picture(&mut self, frame_type: i32) {
        let clicks = TYPES
            .iter()
            .any(|option| option.frame_type == frame_type && option.picture_clicks);
        if !self.takes_clicks() || !self.types_enabled || !clicks {
            return;
        }
        self.do_type(frame_type);
    }

    /// The class button that is down, by its text.
    fn class_text(&self) -> Option<&'static str> {
        let checked = self.checked_class?;
        CLASSES
            .iter()
            .find(|button| button.class == checked)
            .map(|button| button.text)
    }

    /// The checked type radio button's name.
    fn type_text(&self) -> Option<&'static str> {
        match self.checked_type? {
            TypeRadio::Other => Some(OTHER.0),
            TypeRadio::Type(value) => TYPES
                .iter()
                .find(|option| option.frame_type == value)
                .map(|option| option.name),
        }
    }
}

/// Facts for a test: what the page read and shows, how its writes went, and the vehicle's
/// `FRAME_CLASS` and `FRAME_TYPE` as `params.value.<name>`.
pub fn record_facts(frame: &FrameType, view: &TelemetryView) {
    use crate::facts::record;
    let joined = |values: Vec<String>| {
        if values.is_empty() {
            "none".to_owned()
        } else {
            values.join(",")
        }
    };
    let held = frame.active && frame.read;
    record("config.frame.available", available(view));
    record("config.frame.active", frame.active);
    record("config.frame.enabled", frame.enabled);
    record(
        "config.frame.classes",
        joined(
            CLASSES
                .iter()
                .map(|button| button.text.to_owned())
                .collect(),
        ),
    );
    record(
        "config.frame.class",
        if held {
            frame.frame_class.to_string()
        } else {
            "none".to_owned()
        },
    );
    record(
        "config.frame.class.button",
        frame.class_text().unwrap_or("none"),
    );
    record(
        "config.frame.type",
        if held {
            frame.frame_type.to_string()
        } else {
            "none".to_owned()
        },
    );
    record(
        "config.frame.type.radio",
        frame.type_text().unwrap_or("none"),
    );
    record("config.frame.types.enabled", frame.types_enabled);
    let listed = |flags: &[bool; TYPES.len()]| {
        joined(
            TYPES
                .iter()
                .zip(flags)
                .filter(|(_, on)| **on)
                .map(|(option, _)| option.frame_type.to_string())
                .collect(),
        )
    };
    record("config.frame.types", listed(&frame.radios));
    record("config.frame.bright", listed(&frame.bright));
    record("config.frame.pending", frame.writes.pending());
    record("config.frame.write", frame.writes.last().unwrap_or("none"));
    record(
        "config.frame.message",
        frame.writes.message().unwrap_or("none"),
    );
    for name in WATCHED {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
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

/// Where a picture is when its resource is not carried: a box of its size with the name in it.
fn placeholder(name: &str, enabled: bool) -> Div {
    div()
        .size_full()
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .bg(rgb(theme::BG))
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(name.to_owned())
}

/// One Frame Class button: its picture above its text, down when chosen.
fn class_button(
    frame: &FrameType,
    button: ClassButton,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = format!("frame-class-{}", button.class);
    let down = frame.checked_class == Some(button.class);
    let enabled = frame.takes_clicks();
    let colour = match (enabled, down) {
        (false, _) => theme::DIM,
        (true, true) => theme::ACCENT,
        (true, false) => theme::TEXT,
    };
    // `new Bitmap(radioButton<Name>.Image, 60, 60)`: the image stretched to 60 by 60, above the
    // text. C#: GCSViews/ConfigurationView/ConfigFrameClassType.cs:27-33; ConfigFrameClassType.Designer.cs:261-327
    let picture = button.image.map(|resource| {
        let slot = div().w(px(60.0)).h(px(60.0)).flex_none();
        let drawn = crate::pictures::image(resource, crate::pictures::Layout::Stretch);
        crate::pictures::record("frame", &id, drawn.as_ref().map(|_| resource));
        match drawn {
            Some(image) => slot.child(image),
            None => slot.child(placeholder(button.text, enabled)),
        }
    });
    let body = crate::probe::measured(id.clone(), at(button.x, 18.0, 74.0, 74.0))
        .id(SharedString::from(id))
        .flex()
        .flex_col()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_2()
        .border_color(rgb(if down { theme::ACCENT } else { theme::BORDER }))
        .bg(rgb(if down { theme::ACTION } else { theme::BG }))
        .text_xs()
        .text_color(rgb(colour))
        .children(picture)
        .child(
            div()
                .w_full()
                .line_height(px(12.0))
                .text_center()
                .child(button.text),
        );
    if enabled {
        let class = button.class;
        body.cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.frame_type.click_class(class);
                cx.notify();
            }))
            .into_any_element()
    } else {
        body.into_any_element()
    }
}

/// A radio button: a circle, filled when checked. Dimmed and inert when disabled.
fn radio(id: String, x: f32, y: f32, checked: bool, enabled: bool) -> gpui::Stateful<Div> {
    let ring = if enabled {
        theme::ACCENT
    } else {
        theme::BORDER
    };
    crate::probe::measured(id.clone(), at(x, y, 14.0, 13.0))
        .id(SharedString::from(id))
        .child(
            div()
                .size(px(13.0))
                .flex()
                .items_center()
                .justify_center()
                .rounded_full()
                .border_1()
                .border_color(rgb(ring))
                .bg(rgb(theme::BG))
                .children(checked.then(|| {
                    div().size(px(5.0)).rounded_full().bg(rgb(if enabled {
                        theme::ACCENT
                    } else {
                        theme::DIM
                    }))
                })),
        )
}

/// One Frame Type row: its label, its radio button and its picture.
fn type_row(
    frame: &FrameType,
    index: usize,
    option: TypeOption,
    cx: &mut Context<MissionPlanner>,
) -> [AnyElement; 3] {
    let group_on = frame.takes_clicks() && frame.types_enabled;
    let radio_on = group_on && frame.radios.get(index).copied().unwrap_or(false);
    let bright = frame.bright.get(index).copied().unwrap_or(true);
    let checked = frame.checked_type == Some(TypeRadio::Type(option.frame_type));
    let value = option.frame_type;

    let label = at(option.label_at.0, option.label_at.1, 60.0, 13.0)
        .text_xs()
        .text_color(rgb(if radio_on { theme::TEXT } else { theme::DIM }))
        .child(option.label)
        .into_any_element();

    let button = radio(
        format!("frame-type-{value}"),
        option.radio_at.0,
        option.radio_at.1,
        checked,
        radio_on,
    );
    let button = if radio_on {
        button
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.frame_type.click_radio(value);
                cx.notify();
            }))
            .into_any_element()
    } else {
        button.into_any_element()
    };

    let (x, y, width, height) = option.picture;
    let id = format!("frame-picture-{value}");
    // Each `PictureBoxWithPseudoOpacity`'s `Image`, zoomed; V Tail's has none and is empty.
    // C#: GCSViews/ConfigurationView/ConfigFrameClassType.Designer.cs:103, 112, 134, 156, 183;
    // ConfigFrameClassType.resx (pictureBox*.SizeMode)
    let drawn = option
        .image
        .and_then(|resource| crate::pictures::image(resource, crate::pictures::Layout::ZoomImage));
    crate::pictures::record("frame", &id, option.image.filter(|_| drawn.is_some()));
    let content = match (drawn, option.image) {
        (Some(image), _) => image,
        (None, Some(_)) => placeholder(option.name, group_on).into_any_element(),
        (None, None) => div().size_full().into_any_element(),
    };
    let picture = crate::probe::measured(id.clone(), at(x, y, width, height))
        .id(SharedString::from(id))
        .opacity(if bright {
            ENABLED_OPACITY
        } else {
            DISABLED_OPACITY
        })
        .child(content);
    let picture = if group_on && option.picture_clicks {
        picture
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.frame_type.click_picture(value);
                cx.notify();
            }))
            .into_any_element()
    } else {
        picture.into_any_element()
    };

    [label, button, picture]
}

/// The page, while it is showing.
pub fn page(frame: &FrameType, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    if !frame.active {
        return None;
    }
    // `$this.Size` is 787 x 532; the two groups reach 655 across and 529 down.
    let mut body = div().relative().w(px(658.0)).h(px(532.0));

    // groupBox3, "Frame Class", at 3, 3.
    let mut classes = group(3.0, 3.0, 652.0, 100.0, "Frame Class", frame.enabled);
    for button in CLASSES {
        classes = classes.child(class_button(frame, button, cx));
    }

    // groupBox2, "Frame Type", at 3, 109.
    let group_on = frame.takes_clicks() && frame.types_enabled;
    let mut types = group(3.0, 109.0, 504.0, 420.0, "Frame Type", group_on);
    for (index, option) in TYPES.into_iter().enumerate() {
        types = types.children(type_row(frame, index, option, cx));
    }
    let note = |x: f32, y: f32, width: f32, text: &'static str| {
        at(x, y, width, 43.0)
            .text_xs()
            .text_color(rgb(if group_on { theme::TEXT } else { theme::DIM }))
            .child(text)
    };
    let other_checked = frame.checked_type == Some(TypeRadio::Other);
    types = types
        .child(note(200.0, 272.0, 131.0, NOTE_H))
        .child(note(200.0, 356.0, 142.0, NOTE_Y6B))
        // `radio_type_other` has no handler, so it is drawn and not wired: whatever it would
        // uncheck puts itself back through its own handler.
        .child(radio(
            "frame-type-other".to_owned(),
            OTHER.1,
            OTHER.2 + 2.0,
            other_checked,
            group_on,
        ))
        .child(
            at(OTHER.1 + 17.0, OTHER.2 + 2.0, 40.0, 13.0)
                .text_xs()
                .text_color(rgb(if group_on { theme::TEXT } else { theme::DIM }))
                .child(OTHER.0),
        );

    body = body.child(classes).child(types);

    // `CustomMessageBox.Show` is modal: drawn over the page, which takes no click until it is
    // dismissed.
    if let Some(message) = frame.writes.message() {
        body = body.child(
            div()
                .id("frame-message-backdrop")
                .absolute()
                .inset_0()
                .flex()
                .items_center()
                .justify_center()
                .occlude()
                .child(
                    crate::probe::measured("frame-message", div())
                        .flex()
                        .flex_col()
                        .gap_2()
                        .w(px(340.0))
                        .p_3()
                        .bg(rgb(theme::PANEL))
                        .border_1()
                        .border_color(rgb(theme::ALERT))
                        .rounded_md()
                        .child(
                            div()
                                .text_xs()
                                .text_color(rgb(theme::DIM))
                                .child(ERROR_TITLE),
                        )
                        .child(
                            div()
                                .text_sm()
                                .text_color(rgb(theme::TEXT))
                                .child(message.to_owned()),
                        )
                        .child(div().flex().justify_end().child(action(
                            "frame-message-ok",
                            "OK",
                            theme::ACCENT,
                            true,
                            cx.listener(|this, _event: &(), _window, cx| {
                                this.frame_type.writes.dismiss_message();
                                cx.notify();
                            }),
                        ))),
                ),
        );
    }

    Some(panel("frame type", body).into_any_element())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// SITL's copter, as `tools/sitl/params/copter.parm` leaves it: a quad, plus.
    fn sitl() -> Vec<(String, f64)> {
        table(&[("FRAME_CLASS", 1.0), ("FRAME_TYPE", 0.0), ("FLTMODE1", 7.0)])
    }

    /// A page opened on `parameters`, with `Activate`'s own writes taken off the queue.
    fn opened(parameters: &[(String, f64)]) -> FrameType {
        let mut frame = FrameType::default();
        frame.activate(parameters);
        frame.writes = Writes::default();
        frame
    }

    /// The bundled documentation's name for a value of a parameter.
    fn documented(param: &str, value: i32) -> Option<&'static str> {
        mp_params::param_meta::lookup(param)?.value_name(i64::from(value))
    }

    /// Each class button is the `motor_frame_class` value its text names, and the vehicle's own
    /// documentation of `FRAME_CLASS` calls that value by the same name.
    #[test]
    fn the_class_buttons_are_the_documented_frame_classes() {
        for button in CLASSES {
            assert_eq!(
                documented("FRAME_CLASS", button.class),
                Some(button.text),
                "{button:?}"
            );
        }
        let values: Vec<i32> = CLASSES.iter().map(|button| button.class).collect();
        assert_eq!(
            values,
            [0, 1, 2, 3, 4, 5, 6, 7],
            "left to right, as the resx has them"
        );
        let xs: Vec<f32> = CLASSES.iter().map(|button| button.x).collect();
        assert_eq!(xs, [6.0, 86.0, 166.0, 246.0, 326.0, 406.0, 486.0, 566.0]);
        assert!(CLASSES.iter().skip(1).all(|button| button.image.is_some()));
        assert_eq!(CLASSES.first().and_then(|button| button.image), None);
    }

    /// Each type radio button is the `motor_frame_type` value its label names, and `FRAME_TYPE`'s
    /// documentation agrees - V Tail is "V-Tail" there.
    #[test]
    fn the_type_radios_are_the_documented_frame_types() {
        let expected = [
            (0, "Plus"),
            (1, "X"),
            (2, "V"),
            (3, "H"),
            (10, "Y6B"),
            (4, "V-Tail"),
        ];
        for (option, (value, name)) in TYPES.iter().zip(expected) {
            assert_eq!(option.frame_type, value, "{option:?}");
            assert_eq!(documented("FRAME_TYPE", value), Some(name), "{option:?}");
            assert!(
                option
                    .label
                    .replace('-', " ")
                    .contains(&name.replace('-', " ")),
                "{option:?} is labelled for {name}"
            );
        }
        // Only V Tail's picture has neither an image nor a click handler.
        for option in TYPES {
            assert_eq!(option.picture_clicks, option.frame_type != TYPE_VTAIL);
            assert_eq!(option.image.is_some(), option.frame_type != TYPE_VTAIL);
        }
    }

    /// `ValidList` as Common.cs has it, per class.
    #[test]
    fn the_valid_list_is_common_cs() {
        assert_eq!(VALID_LIST.len(), 29);
        assert_eq!(
            valid_types(1),
            [Some(0), Some(1), Some(2), Some(3), Some(4), Some(5)]
        );
        assert_eq!(valid_types(2), [Some(0), Some(1)]);
        assert_eq!(valid_types(3), [Some(0), Some(1), Some(2), Some(3)]);
        assert_eq!(valid_types(4), [Some(0), Some(1), Some(2), Some(3)]);
        assert_eq!(valid_types(5), [Some(10), Some(1)]);
        assert_eq!(valid_types(12), [Some(0), Some(1), None]);
        assert_eq!(valid_types(6), [None]);
        assert!(valid_types(14).is_empty(), "Deca is not in the list");
        // Every class the list names is one FRAME_CLASS documents.
        for (class, frame_type) in VALID_LIST {
            assert!(documented("FRAME_CLASS", class).is_some(), "{class}");
            if let Some(frame_type) = frame_type {
                assert!(
                    documented("FRAME_TYPE", frame_type).is_some(),
                    "{frame_type}"
                );
            }
        }
    }

    /// The type group is enabled for a class with a type, and off for one with none or unlisted.
    #[test]
    fn the_type_group_is_enabled_only_for_a_class_with_types() {
        for class in [1, 2, 3, 4, 5, 12] {
            assert!(types_enabled(class), "{class}");
        }
        for class in [0, 6, 7, 8, 9, 10, 11, 13, 14, 15] {
            assert!(!types_enabled(class), "{class}");
        }
    }

    /// Opening the page on SITL's quad: Quad down, Plus checked, Quad's types enabled, Plus's
    /// picture alone bright - and `DoClass` and `DoType` each write the two values back.
    #[test]
    fn activate_reads_the_frame_and_writes_it_back() {
        let mut frame = FrameType::default();
        frame.activate(&sitl());
        assert!(frame.is_active());
        assert!(frame.enabled);
        assert_eq!(frame.checked_class, Some(1));
        assert_eq!(frame.class_text(), Some("Quad"));
        assert_eq!(frame.checked_type, Some(TypeRadio::Type(0)));
        assert_eq!(frame.type_text(), Some("Plus"));
        assert!(frame.types_enabled);
        // Plus, X, V, H and V Tail; A-Tail has no radio and Y6B is not a quad's.
        assert_eq!(frame.radios, [true, true, true, true, false, true]);
        assert_eq!(frame.bright, [true, false, false, false, false, false]);
        assert_eq!(
            frame.writes.queued(),
            [
                ("FRAME_CLASS", 1.0),
                ("FRAME_TYPE", 0.0),
                ("FRAME_CLASS", 1.0),
                ("FRAME_TYPE", 0.0)
            ]
        );
    }

    /// Without both parameters the page is disabled and writes nothing; clicks do nothing.
    #[test]
    fn a_vehicle_without_frame_type_disables_the_page() {
        let mut frame = FrameType::default();
        frame.activate(&table(&[("FRAME_CLASS", 1.0)]));
        assert!(!frame.enabled);
        assert!(frame.writes.queued().is_empty());
        frame.click_class(2);
        frame.click_radio(1);
        frame.click_picture(1);
        assert!(frame.writes.queued().is_empty());
        assert_eq!(frame.checked_type, Some(TypeRadio::Other), "the designer's");
    }

    /// A value that is not a whole number stops `Activate` where `Enum.Parse` throws.
    #[test]
    fn a_fractional_value_stops_activate() {
        let mut frame = FrameType::default();
        frame.activate(&table(&[("FRAME_CLASS", 1.5), ("FRAME_TYPE", 1.0)]));
        assert!(frame.enabled, "only a missing parameter disables it");
        assert_eq!(frame.checked_class, None);
        assert!(frame.writes.queued().is_empty());
    }

    /// Choosing X on a quad writes FRAME_CLASS then FRAME_TYPE, checks X and brightens it alone.
    #[test]
    fn a_type_click_writes_the_class_then_the_type() {
        let mut frame = opened(&sitl());
        frame.click_radio(1);
        assert_eq!(
            frame.writes.queued(),
            [("FRAME_CLASS", 1.0), ("FRAME_TYPE", 1.0)]
        );
        assert_eq!(frame.checked_type, Some(TypeRadio::Type(1)));
        assert_eq!(frame.bright, [false, true, false, false, false, false]);
        // The checked one again raises no CheckedChanged.
        frame.click_radio(1);
        assert_eq!(frame.writes.queued().len(), 2);
        // Y6B is not a quad's, so its radio is disabled; its picture still takes the click.
        frame.click_radio(10);
        assert_eq!(frame.writes.queued().len(), 2);
        frame.click_picture(10);
        assert_eq!(
            frame.writes.queued().get(2..),
            Some(&[("FRAME_CLASS", 1.0), ("FRAME_TYPE", 10.0)][..])
        );
        assert_eq!(frame.checked_type, Some(TypeRadio::Type(10)));
        // V Tail's picture has no handler; its radio does.
        frame.click_picture(TYPE_VTAIL);
        assert_eq!(frame.writes.queued().len(), 4);
        frame.click_radio(TYPE_VTAIL);
        assert_eq!(
            frame.writes.queued().last(),
            Some(&("FRAME_TYPE", f64::from(TYPE_VTAIL)))
        );
    }

    /// Choosing a class writes it with the current type, enables and brightens the types it
    /// takes, and leaves the checked type alone.
    #[test]
    fn a_class_click_writes_the_class_with_the_current_type() {
        let mut frame = opened(&table(&[("FRAME_CLASS", 1.0), ("FRAME_TYPE", 1.0)]));
        frame.click_class(2);
        assert_eq!(
            frame.writes.queued(),
            [("FRAME_CLASS", 2.0), ("FRAME_TYPE", 1.0)]
        );
        assert_eq!(frame.checked_class, Some(2));
        assert_eq!(frame.class_text(), Some("Hexa"));
        assert_eq!(frame.radios, [true, true, false, false, false, false]);
        assert_eq!(frame.bright, [true, true, false, false, false, false]);
        assert_eq!(frame.checked_type, Some(TypeRadio::Type(1)));
        // The class that is down again does nothing.
        frame.click_class(2);
        assert_eq!(frame.writes.queued().len(), 2);
        // Y6 takes X and Y6B.
        frame.click_class(5);
        assert_eq!(frame.radios, [false, true, false, false, true, false]);
        // A heli takes no type: the group goes off, every picture fades, and the type written is
        // still the current one.
        frame.click_class(6);
        assert!(!frame.types_enabled);
        assert_eq!(frame.bright, [false; TYPES.len()]);
        assert_eq!(
            frame.writes.queued().get(4..),
            Some(&[("FRAME_CLASS", 6.0), ("FRAME_TYPE", 1.0)][..])
        );
        // ... and nothing in it takes a click.
        frame.click_radio(0);
        frame.click_picture(0);
        assert_eq!(frame.writes.queued().len(), 6);
        // A class with no button is not clickable.
        frame.click_class(12);
        assert_eq!(frame.writes.queued().len(), 6);
    }

    /// A type none of the six radio buttons names leaves "Other" checked, and `DoType` writes
    /// nothing for it - only `DoClass`'s pair goes out.
    #[test]
    fn an_unnamed_type_is_other() {
        let mut frame = FrameType::default();
        // A-Tail.
        frame.activate(&table(&[("FRAME_CLASS", 1.0), ("FRAME_TYPE", 5.0)]));
        assert_eq!(frame.checked_type, Some(TypeRadio::Other));
        assert_eq!(frame.type_text(), Some("Other"));
        assert_eq!(
            frame.writes.queued(),
            [("FRAME_CLASS", 1.0), ("FRAME_TYPE", 5.0)]
        );
        // `DoClass`'s fade stands: the quad's types bright.
        assert_eq!(frame.bright, [true, true, true, true, false, true]);
        // The fallback unchecks five of the six, and never V Tail.
        frame.checked_type = Some(TypeRadio::Type(1));
        frame.do_type(13);
        assert_eq!(frame.checked_type, None);
        frame.checked_type = Some(TypeRadio::Type(TYPE_VTAIL));
        frame.do_type(13);
        assert_eq!(frame.checked_type, Some(TypeRadio::Type(TYPE_VTAIL)));
    }

    /// A class without a button - DodecaHexa - leaves every button up and still enables its
    /// types.
    #[test]
    fn a_class_without_a_button() {
        let mut frame = FrameType::default();
        frame.activate(&table(&[("FRAME_CLASS", 12.0), ("FRAME_TYPE", 1.0)]));
        assert_eq!(frame.checked_class, None);
        assert!(frame.types_enabled);
        assert_eq!(frame.radios, [true, true, false, false, false, false]);
    }

    /// The page is listed for a vehicle with FRAME_CLASS, or a connected copter of 3.5 or later
    /// with every parameter in.
    #[test]
    fn the_page_is_listed_for_frame_class_or_a_recent_copter() {
        let mut view = TelemetryView::disconnected("test");
        assert!(!available(&view));
        view.parameters = sitl().into();
        assert!(available(&view), "FRAME_CLASS alone lists it");

        let vehicle = |vehicle_type: u8, version: [u8; 4]| {
            let mut state = mp_vehicle::VehicleState::default();
            state.autopilot = 3;
            state.vehicle_type = vehicle_type;
            state.autopilot_info.version = version;
            Some(std::sync::Arc::new(state))
        };
        view.parameters = table(&[("FLTMODE1", 7.0)]).into();
        view.parameters_expected = 1;
        view.state = vehicle(2, [4, 5, 7, 255]);
        assert!(!available(&view), "not connected");
        view.connected = true;
        assert!(available(&view));
        view.parameters_expected = 2;
        assert!(!available(&view), "not every parameter in");
        view.parameters_expected = 0;
        view.parameters = Default::default();
        assert!(!available(&view), "none downloaded yet");
        view.parameters = table(&[("FLTMODE1", 7.0)]).into();
        view.parameters_expected = 1;

        view.state = vehicle(2, [3, 4, 0, 255]);
        assert!(!available(&view), "3.4 has ConfigFrameType instead");
        view.state = vehicle(2, [3, 5, 0, 0]);
        assert!(available(&view));
        view.state = vehicle(1, [4, 5, 7, 255]);
        assert!(!available(&view), "a plane without FRAME_CLASS");
    }

    /// A stand-in link: every write answered at once with the outcome given for its name, or
    /// accepted.
    struct Answering {
        outcomes: std::collections::BTreeMap<&'static str, Progress>,
        written: std::cell::RefCell<Vec<(String, f64)>>,
    }

    impl Answering {
        fn new(outcomes: &[(&'static str, Progress)]) -> Self {
            Self {
                outcomes: outcomes.iter().copied().collect(),
                written: std::cell::RefCell::new(Vec::new()),
            }
        }

        fn written(&self) -> Vec<(String, f64)> {
            self.written.borrow().clone()
        }
    }

    impl ParamWriter for Answering {
        type Handle = usize;

        fn write(&self, name: &str, value: f64) -> Option<usize> {
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

    fn call(frame_class: i32, frame_type: i32) -> Call {
        Call {
            frame_class,
            frame_type,
        }
    }

    /// Each call's two writes go out class first, one after the other, and calls in order.
    #[test]
    fn writes_go_class_then_type_in_order() {
        let link = Answering::new(&[]);
        let mut writes = Writes::default();
        writes.call(call(1, 1));
        writes.call(call(1, 0));
        assert_eq!(writes.pending(), 4);
        writes.advance(&link);
        assert_eq!(
            link.written(),
            [
                ("FRAME_CLASS".to_owned(), 1.0),
                ("FRAME_TYPE".to_owned(), 1.0),
                ("FRAME_CLASS".to_owned(), 1.0),
                ("FRAME_TYPE".to_owned(), 0.0)
            ]
        );
        assert_eq!(writes.pending(), 0);
        assert_eq!(writes.last(), Some("FRAME_TYPE 0 accepted"));
        assert_eq!(writes.message(), None);
    }

    /// While a write is unanswered nothing more is sent.
    #[test]
    fn writes_wait_for_each_answer() {
        let link = Answering::new(&[("FRAME_CLASS", Progress::Waiting)]);
        let mut writes = Writes::default();
        writes.call(call(1, 1));
        writes.advance(&link);
        writes.advance(&link);
        assert_eq!(link.written(), [("FRAME_CLASS".to_owned(), 1.0)]);
        assert_eq!(writes.pending(), 2, "the class waiting, the type queued");
        assert_eq!(writes.queued(), [("FRAME_TYPE", 1.0)]);
    }

    /// A class write that times out skips that call's type and shows the C#'s message; the next
    /// call still goes out.
    #[test]
    fn a_timed_out_write_ends_its_call_with_the_message() {
        let link = Answering::new(&[("FRAME_CLASS", Progress::Finished(RequestOutcome::TimedOut))]);
        let mut writes = Writes::default();
        writes.call(call(2, 1));
        writes.advance(&link);
        assert_eq!(link.written(), [("FRAME_CLASS".to_owned(), 2.0)]);
        assert_eq!(writes.message(), Some(FAILED));
        assert_eq!(
            writes.message(),
            Some("Set FRAME_CLASS OR FRAME_TYPE Failed")
        );
        assert_eq!(writes.last(), Some("FRAME_CLASS 2 failed: timed out"));
        writes.dismiss_message();
        assert_eq!(writes.message(), None);

        let link = Answering::new(&[("FRAME_TYPE", Progress::Finished(RequestOutcome::TimedOut))]);
        let mut writes = Writes::default();
        writes.call(call(1, 3));
        writes.call(call(1, 0));
        writes.advance(&link);
        assert_eq!(link.written().len(), 4, "the second call is not held up");
        assert_eq!(writes.messages.len(), 2);
    }

    /// A parameter the vehicle lacks, or already holds, is no error: `setParam` returns and the
    /// type is still written.
    #[test]
    fn unknown_and_unchanged_are_not_errors() {
        let link = Answering::new(&[
            (
                "FRAME_CLASS",
                Progress::Finished(RequestOutcome::UnknownParameter),
            ),
            ("FRAME_TYPE", Progress::Finished(RequestOutcome::Unchanged)),
        ]);
        let mut writes = Writes::default();
        writes.call(call(1, 1));
        writes.advance(&link);
        assert_eq!(link.written().len(), 2);
        assert_eq!(writes.message(), None);
        assert_eq!(writes.last(), Some("FRAME_TYPE 1 unchanged"));
    }

    /// With no vehicle to write to, the call fails as the C#'s would.
    #[test]
    fn no_vehicle_fails_the_call() {
        struct Nowhere;
        impl ParamWriter for Nowhere {
            type Handle = usize;
            fn write(&self, _name: &str, _value: f64) -> Option<usize> {
                None
            }
            fn progress(&self, _handle: usize) -> Progress {
                Progress::Lost
            }
        }
        let mut writes = Writes::default();
        writes.call(call(1, 1));
        writes.advance(&Nowhere);
        assert_eq!(writes.message(), Some(FAILED));
        assert_eq!(writes.pending(), 0);
    }

    /// Opening the page on SITL's quad and clicking X, through the writer: the pairs go out in
    /// the order the C# sends them.
    #[test]
    fn a_click_reaches_the_link_class_first() {
        let link = Answering::new(&[]);
        let mut writes: Writes<usize> = Writes::default();
        let mut frame = FrameType::default();
        frame.activate(&sitl());
        writes.calls = std::mem::take(&mut frame.writes.calls);
        writes.advance(&link);
        frame.click_radio(1);
        writes.calls = std::mem::take(&mut frame.writes.calls);
        writes.advance(&link);
        let written = link.written();
        assert_eq!(written.len(), 6);
        assert_eq!(
            written.get(4..),
            Some(
                &[
                    ("FRAME_CLASS".to_owned(), 1.0),
                    ("FRAME_TYPE".to_owned(), 1.0)
                ][..]
            )
        );
    }
}
