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

//! The Accel Calibration page of Initial Setup: `GCSViews/ConfigurationView/
//! ConfigAccelerometerCalibration.cs`, a Mandatory Hardware entry (`GCSViews/InitialSetup.cs:
//! 193-197`), listed once every parameter is in.
//!
//! What it shows, top to bottom as the `.resx` places it in a 479 x 275 page: the heading
//! "Accelerometer Calibration" over a rule; then three calibrations, each a sentence and a
//! button - Calibrate Accel (the six-position calibration, "Min/Max (3 axis)"), Calibrate Level
//! ("offsets (1 axis/AHRS trims)") and Simple Accel Cal ("scale factors for level flight (1
//! axis)") - with, under Calibrate Accel, the label the vehicle's prompts are written into.
//!
//! The three buttons are `MAV_CMD_PREFLIGHT_CALIBRATION` with param5 1, 2 and 4.
//!
//! * Calibrate Accel starts a conversation. `doCommand` returns at once for param5 1 - "for
//!   advanced accel offsets, and blocks execution" - so the command goes out once and is not
//!   waited on; the page then listens for the vehicle's `STATUSTEXT` and `COMMAND_LONG`, and the
//!   button reads "Click when Done". The vehicle asks for each position with
//!   `MAV_CMD_ACCELCAL_VEHICLE_POS` - repeated every second - and the page writes "Please place
//!   vehicle LEVEL" (the position's name) into its label; a `STATUSTEXT` that says "place
//!   vehicle" or "calibration", such as ArduPilot's "Place vehicle level and press any key.", is
//!   written there too. Each click of the button then sends `MAV_CMD_ACCELCAL_VEHICLE_POS` back
//!   with the position last asked for, on the wire as the C#'s `sendPacket` puts it: not waited
//!   on, and addressed to nobody (target 0/0, the struct literal's defaults, which ArduPilot takes
//!   as a broadcast). A `STATUSTEXT` saying "calibration successful" or "calibration failed" ends
//!   it: the button reads "Done" and is disabled, and the page stops listening.
//! * Calibrate Level and Simple Accel Cal are single commands the C# blocks on - a calibration's
//!   `doCommand` waits 25 seconds, and sends again once - and each button's text becomes
//!   "Completed" when the vehicle accepts. A refusal is "The Command failed to execute", a
//!   timeout the handler's own `catch`.
//!
//! What is disabled, and when, is the C#'s: nothing while the six-position conversation runs -
//! Calibrate Level and Simple Accel Cal stay live, and the vehicle refuses them while it is
//! calibrating; Calibrate Accel once the vehicle says the calibration succeeded or failed, until
//! the page is shown again (`Activate` enables it, leaving its text); and every button while
//! Calibrate Level or Simple Accel Cal waits for its answer, when the C#'s UI thread is inside
//! `doCommand` and takes no click.
//!
//! Leaving the page (`Deactivate`) ends the conversation as far as the button is concerned -
//! `_incalibrate` false, so the next click starts a new calibration - but not the listening: the
//! C# does not unsubscribe there, so the label and the button still follow the vehicle while the
//! page is hidden, until the screen is left.
//!
//! What is not carried over, and why:
//!
//! * a `COMMAND_LONG` whose position is not one the link knows. The link keeps the last
//!   `MAV_CMD_ACCELCAL_VEHICLE_POS` it heard as a position, a success or a failure
//!   ([`mp_calibration::AccelCalibration`]), and a value that is none of those as nothing; the
//!   page takes that value and clears it, so each request the vehicle repeats is one event, as
//!   the C#'s subscription sees each packet. ArduPilot sends no other value;
//! * the order of a `COMMAND_LONG` and a `STATUSTEXT` that arrive within one frame: the position
//!   is written first and the text after. The vehicle repeats the position every second, so the
//!   label is the C#'s within a second either way;
//! * `MainV2.comPort.giveComport = false` in `Deactivate`: the flag that stops the C#'s own
//!   reader while a blocking call reads the port. This application's link reads on its own
//!   thread and has no such flag;
//! * the trailing spaces and NULs of a `STATUSTEXT`: the link trims them before the page sees the
//!   text, and a label does not show them.
//!
//! The colours are this application's; the positions and sizes are the `.resx`'s.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, Div, SharedString, Window, div, prelude::*, px, rgb};
use mp_link::RequestId;
use mp_link::messages::LogMessage;
use mp_link::requests::RequestOutcome;
use mp_mavlink_dialects::all::{CommandLong, MavMessage};
use mp_vehicle::VehicleId;

use crate::MissionPlanner;
use crate::config::servo_output::{ERROR_TITLE, Message, modal, value_of};
use crate::fly::strings::{COMMAND_FAILED, COMPLETED};
use crate::setup::Key;
use crate::telemetry::{Report, Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:196`
pub const TITLE: &str = "Accel Calibration";

/// `label5.Text`, the heading, in 12 point.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.resx label5.Text, label5.Font`
pub const HEADING: &str = "Accelerometer Calibration";

/// `label4.Text`, above Calibrate Accel.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.resx label4.Text`
pub const ACCEL_TEXT: &str = "Level your Autopilot to set default accelerometer Min/Max (3 \
                              axis).\nThis will ask you to place your autopilot on each edge.";

/// `label1.Text`, above Calibrate Level.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.resx label1.Text`
pub const LEVEL_TEXT: &str = "Level your Autopilot to set default accelerometer offsets (1 \
                              axis/AHRS trims).\nThis requires you to place your autopilot flat \
                              and level.";

/// `label2.Text`, above Simple Accel Cal.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.resx label2.Text`
pub const SIMPLE_TEXT: &str = "Level your Autopilot to set default accelerometer scale factors \
                               for level flight (1 axis).\nThis requires you to place your \
                               autopilot flat and level.";

/// `BUT_calib_accell.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.resx BUT_calib_accell.Text`
pub const CALIBRATE_ACCEL: &str = "Calibrate Accel";
/// `BUT_level.Text`.
pub const CALIBRATE_LEVEL: &str = "Calibrate Level";
/// `BUT_simpleAccelCal.Text`.
pub const SIMPLE_ACCEL_CAL: &str = "Simple Accel Cal";

/// `Strings.Click_when_Done`: Calibrate Accel's text while the conversation runs.
/// `// C#: ExtLibs/Strings/Strings.resx:540-542`
pub const CLICK_WHEN_DONE: &str = "Click when Done";
/// `Strings.Done`: its text once the vehicle says it succeeded or failed.
/// `// C#: ExtLibs/Strings/Strings.resx:306-308`
pub const DONE: &str = "Done";

/// `BUT_calib_accell_Click`'s and `BUT_level_Click`'s `catch`.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:87, 161`
pub const FAILED_TO_LEVEL: &str = "Failed to level";
/// `BUT_simpleAccelCal_Click`'s `catch`.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:183`
pub const FAILED_SIMPLE: &str = "Failed to simple accelerometer calibration";

/// What `receivedPacket` writes for the vehicle's request, before the position's name.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:127`
pub const PLEASE_PLACE: &str = "Please place vehicle ";

/// `MAV_CMD_PREFLIGHT_CALIBRATION`'s param5 for each button: Calibrate Accel's, which
/// [`Telemetry::start_accelerometer_calibration`] sends and the tests read back.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:69, 149, 171`
#[cfg(test)]
pub const ACCEL_PARAM5: f32 = 1.0;
/// Calibrate Level's.
pub const LEVEL_PARAM5: f32 = 2.0;
/// Simple Accel Cal's.
pub const SIMPLE_PARAM5: f32 = 4.0;

/// A control's `Location` and `Size`.
pub type Place = (f32, f32, f32, f32);

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.resx $this.Size`
pub const PAGE_SIZE: (f32, f32) = (479.0, 275.0);
/// `label5.Location`.
pub const HEADING_AT: (f32, f32) = (3.0, 8.0);
/// `lineSeparator2`.
pub const RULE: Place = (7.0, 30.0, 460.0, 2.0);
/// `label4.Location`.
pub const ACCEL_TEXT_AT: (f32, f32) = (83.0, 45.0);
/// `BUT_calib_accell`.
pub const ACCEL_BUTTON: Place = (188.0, 72.0, 102.0, 21.0);
/// `lbl_Accel_user`, its text centred (`TextAlign` `MiddleCenter`).
pub const USER_LABEL: Place = (7.0, 96.0, 460.0, 37.0);
/// `label1.Location`.
pub const LEVEL_TEXT_AT: (f32, f32) = (60.0, 133.0);
/// `BUT_level`.
pub const LEVEL_BUTTON: Place = (188.0, 168.0, 102.0, 21.0);
/// `label2.Location`.
pub const SIMPLE_TEXT_AT: (f32, f32) = (60.0, 220.0);
/// `BUT_simpleAccelCal`.
pub const SIMPLE_BUTTON: Place = (188.0, 249.0, 102.0, 21.0);

/// `MAVLink.ACCELCAL_VEHICLE_POS`, the names `pos.ToString()` gives; any other value is its
/// number.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:793-820`
pub const POSITIONS: [(i64, &str); 8] = [
    (1, "LEVEL"),
    (2, "LEFT"),
    (3, "RIGHT"),
    (4, "NOSEDOWN"),
    (5, "NOSEUP"),
    (6, "BACK"),
    (16_777_215, "SUCCESS"),
    (16_777_216, "FAILED"),
];

/// `pos.ToString()`.
#[must_use]
pub fn position_name(pos: i64) -> String {
    POSITIONS
        .iter()
        .find(|(value, _)| *value == pos)
        .map_or_else(|| pos.to_string(), |(_, name)| (*name).to_owned())
}

/// The value the link last latched, as the C#'s `(ACCELCAL_VEHICLE_POS)message.param1`.
fn latched_value(latched: mp_calibration::AccelCalibration) -> Option<i64> {
    match latched {
        mp_calibration::AccelCalibration::Idle => None,
        mp_calibration::AccelCalibration::Waiting(position) => Some(i64::from(position.to_wire())),
        mp_calibration::AccelCalibration::Succeeded => {
            Some(i64::from(mp_calibration::ACCELCAL_SUCCESS))
        }
        mp_calibration::AccelCalibration::Failed => {
            Some(i64::from(mp_calibration::ACCELCAL_FAILED))
        }
    }
}

/// Whether a line of the link's message log is a `COMMAND_ACK` rather than a `STATUSTEXT`.
///
/// The link writes both into one log, an acknowledgement as "<MAV_CMD name>: <result>" - or
/// "command <number>: <result>" for a command it cannot name - with the result one of
/// `command_result_name`'s words. The C# listens to `STATUSTEXT` alone, and "MAV_CMD_PREFLIGHT_
/// CALIBRATION: accepted", the vehicle's answer to the start, contains "calibration": read as a
/// prompt, it would be written into the label, which the C# never does.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:73-74`
#[must_use]
pub fn is_command_ack_line(text: &str) -> bool {
    let Some((command, result)) = text.rsplit_once(": ") else {
        return false;
    };
    let known_result = (0..=u8::MAX).any(|r| mp_link::messages::command_result_name(r) == result);
    let named = command.strip_prefix("MAV_CMD_").is_some_and(|rest| {
        !rest.is_empty()
            && rest
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
    });
    let numbered = command
        .strip_prefix("command ")
        .is_some_and(|number| !number.is_empty() && number.chars().all(|c| c.is_ascii_digit()));
    known_result && (named || numbered)
}

/// `SubscribeToPacketType(STATUSTEXT)` and `(COMMAND_LONG)`, filtered to the vehicle: what the
/// page has read of the message log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Subscription {
    /// `sysidcurrent`/`compidcurrent` when it was made.
    vehicle: VehicleId,
    /// The sequence number of the last log line read, or `None` for none yet.
    seen: Option<u64>,
}

/// A blocking `doCommand` the page is waiting on: Calibrate Level's or Simple Accel Cal's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    /// `BUT_level_Click`.
    Level(RequestId),
    /// `BUT_simpleAccelCal_Click`.
    Simple(RequestId),
}

/// The page object and what it keeps.
#[derive(Debug)]
pub struct AccelCalibration {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Between `Activate` and `Deactivate`.
    active: bool,
    /// `BUT_calib_accell.Text`.
    accel_text: &'static str,
    /// `BUT_calib_accell.Enabled`.
    accel_enabled: bool,
    /// `_incalibrate`.
    in_calibrate: bool,
    /// `count`, a `byte`: the clicks since the start.
    count: u8,
    /// `pos`: the position the vehicle last asked for.
    pos: i64,
    /// `sub1` and `sub2`, while they are subscribed.
    subscription: Option<Subscription>,
    /// `lbl_Accel_user.Text`.
    label: String,
    /// `BUT_level.Text`.
    level_text: &'static str,
    /// `BUT_simpleAccelCal.Text`.
    simple_text: &'static str,
    /// The blocking command, while it is answered.
    pending: Option<Pending>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// What each button has put on the wire: starts, positions, levels and simple calibrations.
    sent: [usize; 4],
}

impl Default for AccelCalibration {
    /// `InitializeComponent`: the `.resx`'s texts, every button enabled, the label empty, `pos`
    /// its enum's default, 0.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            accel_text: CALIBRATE_ACCEL,
            accel_enabled: true,
            in_calibrate: false,
            count: 0,
            pos: 0,
            subscription: None,
            label: String::new(),
            level_text: CALIBRATE_LEVEL,
            simple_text: SIMPLE_ACCEL_CAL,
            pending: None,
            messages: VecDeque::new(),
            sent: [0; 4],
        }
    }
}

impl AccelCalibration {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Calibrate Accel's text.
    #[must_use]
    pub const fn accel_text(&self) -> &'static str {
        self.accel_text
    }

    /// Whether Calibrate Accel takes a click.
    #[must_use]
    pub const fn accel_enabled(&self) -> bool {
        self.accel_enabled && self.pending.is_none()
    }

    /// Calibrate Level's text.
    #[must_use]
    pub const fn level_text(&self) -> &'static str {
        self.level_text
    }

    /// Simple Accel Cal's text.
    #[must_use]
    pub const fn simple_text(&self) -> &'static str {
        self.simple_text
    }

    /// Whether Calibrate Level and Simple Accel Cal take a click: always, but while one of them
    /// waits for its answer.
    #[must_use]
    pub const fn others_enabled(&self) -> bool {
        self.pending.is_none()
    }

    /// `lbl_Accel_user.Text`.
    #[must_use]
    pub fn label(&self) -> &str {
        &self.label
    }

    /// `_incalibrate`.
    #[must_use]
    pub const fn in_calibrate(&self) -> bool {
        self.in_calibrate
    }

    /// Whether the page is listening to the vehicle.
    #[must_use]
    pub const fn subscribed(&self) -> bool {
        self.subscription.is_some()
    }

    /// `count`.
    #[must_use]
    pub const fn count(&self) -> u8 {
        self.count
    }

    /// `pos`.
    #[must_use]
    pub const fn pos(&self) -> i64 {
        self.pos
    }

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// What the buttons have sent: starts, positions, levels, simple calibrations.
    #[must_use]
    pub const fn sent(&self) -> [usize; 4] {
        self.sent
    }

    /// `Activate`, on a new page object if this screen has none: Calibrate Accel enabled again,
    /// with whatever text it had, and `_incalibrate` false.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:27-31`
    pub fn activate(&mut self, key: Key) {
        if self.made_for != Some(key) {
            *self = Self {
                sent: self.sent,
                ..Self::default()
            };
            self.made_for = Some(key);
        }
        self.accel_enabled = true;
        self.in_calibrate = false;
        self.active = true;
    }

    /// `Deactivate`: `_incalibrate` false. The subscriptions stay.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:33-37`
    pub fn deactivate(&mut self) {
        self.in_calibrate = false;
        self.active = false;
    }

    /// The page object disposed with its screen: what it held goes, and with it the
    /// subscriptions.
    fn dispose(&mut self) {
        *self = Self {
            sent: self.sent,
            ..Self::default()
        };
    }

    /// `BUT_calib_accell_Click`.
    ///
    /// While calibrating, `count` goes up and `MAV_CMD_ACCELCAL_VEHICLE_POS` goes out with the
    /// position last asked for, as `sendPacket` sends it: once, not waited on, target 0/0. With
    /// no vehicle `sendPacket` returns without sending; a send the link refuses is the `catch`,
    /// `Strings.CommandFailed`.
    ///
    /// Otherwise `count` is 0 and `doCommand(PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0)` goes out,
    /// which for param5 1 returns true as soon as it is sent. Then the page listens and the
    /// button reads "Click when Done"; with no vehicle `doCommand` is false, and the box says
    /// `Strings.CommandFailed`.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:39-89; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1215-1237, 2689-2736`
    pub fn click_accel(&mut self, telemetry: &Telemetry, view: &TelemetryView) {
        if !self.accel_enabled() {
            return;
        }
        if self.in_calibrate {
            self.count = self.count.wrapping_add(1);
            let Some((sender, _)) = telemetry.send_handle() else {
                return;
            };
            if sender.send(&position_message(self.pos)) {
                self.sent[1] += 1;
            } else {
                self.command_failed();
            }
            return;
        }
        self.count = 0;
        let Some((_, vehicle)) = telemetry.send_handle() else {
            self.command_failed();
            return;
        };
        // What the log holds before the command goes out is not the conversation's.
        let seen = view.messages.last().map(|line| line.seq);
        telemetry.start_accelerometer_calibration();
        self.sent[0] += 1;
        self.in_calibrate = true;
        self.subscription = Some(Subscription { vehicle, seen });
        self.accel_text = CLICK_WHEN_DONE;
    }

    /// `BUT_level_Click`: `doCommand(PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 2, 0, 0)`, waited on.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:143-163`
    pub fn click_level(&mut self, telemetry: &mut Telemetry) {
        if !self.others_enabled() {
            return;
        }
        let Some((_, vehicle)) = telemetry.send_handle() else {
            self.command_failed();
            return;
        };
        let params = [0.0, 0.0, 0.0, 0.0, LEVEL_PARAM5, 0.0, 0.0];
        match telemetry.command(
            vehicle,
            mp_calibration::CMD_PREFLIGHT_CALIBRATION,
            params,
            Report::default(),
        ) {
            Some(id) => {
                self.sent[2] += 1;
                self.pending = Some(Pending::Level(id));
            }
            None => self.command_failed(),
        }
    }

    /// `BUT_simpleAccelCal_Click`: `doCommand(PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 4, 0, 0)`, waited
    /// on.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:165-185`
    pub fn click_simple(&mut self, telemetry: &mut Telemetry) {
        if !self.others_enabled() {
            return;
        }
        let Some((_, vehicle)) = telemetry.send_handle() else {
            self.command_failed();
            return;
        };
        let params = [0.0, 0.0, 0.0, 0.0, SIMPLE_PARAM5, 0.0, 0.0];
        match telemetry.command(
            vehicle,
            mp_calibration::CMD_PREFLIGHT_CALIBRATION,
            params,
            Report::default(),
        ) {
            Some(id) => {
                self.sent[3] += 1;
                self.pending = Some(Pending::Simple(id));
            }
            None => self.command_failed(),
        }
    }

    /// `CustomMessageBox.Show(Strings.CommandFailed, Strings.ERROR)`.
    fn command_failed(&mut self) {
        self.messages.push_back(Message {
            title: ERROR_TITLE,
            text: COMMAND_FAILED.to_owned(),
        });
    }

    /// `UpdateUserMessage`: the label takes a message that says "place vehicle" or
    /// "calibration", in any case.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:134-141`
    fn update_user_message(&mut self, message: &str) {
        let lower = message.to_lowercase();
        if lower.contains("place vehicle") || lower.contains("calibration") {
            message.clone_into(&mut self.label);
        }
    }

    /// `receivedPacket` for one `COMMAND_LONG` asking for a position.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:120-129`
    fn position_requested(&mut self, pos: i64) {
        self.pos = pos;
        self.update_user_message(&format!("{PLEASE_PLACE}{}", position_name(pos)));
    }

    /// `receivedPacket` for one `STATUSTEXT`: written into the label if it is a prompt, and the
    /// end of the conversation if it says the calibration succeeded or failed.
    /// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:93-118`
    fn status_text(&mut self, message: &str) {
        self.update_user_message(message);
        let lower = message.to_lowercase();
        if lower.contains("calibration successful") || lower.contains("calibration failed") {
            self.accel_text = DONE;
            self.accel_enabled = false;
            self.in_calibrate = false;
            self.subscription = None;
        }
    }

    /// What the subscriptions deliver this frame: the position the link latched, taken, and
    /// then the vehicle's new `STATUSTEXT` lines in the order they came.
    fn listen(&mut self, telemetry: &Telemetry, messages: &[LogMessage]) {
        let Some(subscription) = self.subscription else {
            return;
        };
        if let Some(pos) = latched_value(telemetry.accel_calibration()) {
            telemetry.clear_accel_calibration();
            self.position_requested(pos);
        }
        let fresh: Vec<&LogMessage> = messages
            .iter()
            .filter(|line| subscription.seen.is_none_or(|seen| line.seq > seen))
            .collect();
        for line in fresh {
            let Some(listening) = self.subscription.as_mut() else {
                break;
            };
            listening.seen = Some(line.seq);
            if line.from == subscription.vehicle && !is_command_ack_line(&line.text) {
                self.status_text(&line.text);
            }
        }
    }

    /// Reads back how Calibrate Level's or Simple Accel Cal's command ended: accepted, the
    /// button reads "Completed"; refused, `Strings.CommandFailed`; unanswered, the handler's
    /// `catch`.
    fn settle(&mut self, telemetry: &Telemetry) {
        let Some(pending) = self.pending else {
            return;
        };
        let id = match pending {
            Pending::Level(id) | Pending::Simple(id) => id,
        };
        let Some(request) = telemetry.request(id) else {
            self.pending = None;
            return;
        };
        let Some(outcome) = request.outcome() else {
            return;
        };
        self.pending = None;
        match outcome {
            RequestOutcome::Accepted { .. } | RequestOutcome::Sent | RequestOutcome::Unchanged => {
                match pending {
                    Pending::Level(_) => self.level_text = COMPLETED,
                    Pending::Simple(_) => self.simple_text = COMPLETED,
                }
            }
            RequestOutcome::Rejected(_) | RequestOutcome::UnknownParameter => {
                self.command_failed();
            }
            RequestOutcome::TimedOut => self.messages.push_back(Message {
                title: ERROR_TITLE,
                text: match pending {
                    Pending::Level(_) => FAILED_TO_LEVEL,
                    Pending::Simple(_) => FAILED_SIMPLE,
                }
                .to_owned(),
            }),
        }
    }

    /// Once a frame: a page object whose screen has gone is disposed; the subscriptions read
    /// what arrived, shown or not; and a blocking command's answer is read.
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, on_setup: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.dispose();
        }
        self.listen(telemetry, &view.messages);
        self.settle(telemetry);
    }
}

/// The `COMMAND_LONG` a click sends while calibrating: `param1 = (float)pos`, `command =
/// ACCELCAL_VEHICLE_POS`, and every other field the struct literal's default.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.cs:50-51`
#[must_use]
pub fn position_message(pos: i64) -> MavMessage {
    #[allow(clippy::cast_precision_loss)] // `(float)pos`, as the C# casts it
    let param1 = pos as f32;
    MavMessage::CommandLong(CommandLong {
        param1,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: mp_calibration::CMD_ACCELCAL_VEHICLE_POS,
        target_system: 0,
        target_component: 0,
        confirmation: 0,
    })
}

/// The parameters the three calibrations write, whose values the facts carry: Calibrate Level's
/// trims and the accelerometer offsets and scales the other two replace.
pub const WATCHED: [&str; 8] = [
    "AHRS_TRIM_X",
    "AHRS_TRIM_Y",
    "INS_ACCOFFS_X",
    "INS_ACCOFFS_Y",
    "INS_ACCOFFS_Z",
    "INS_ACCSCAL_X",
    "INS_ACCSCAL_Y",
    "INS_ACCSCAL_Z",
];

/// Facts a UI test asserts on: the page's words, the three buttons, the label, the conversation
/// and what was sent, and the parameters the calibrations write as `params.value.<name>`.
pub fn record_facts(accel: &AccelCalibration, view: &TelemetryView) {
    use crate::facts::record;
    record("config.accel.active", accel.is_active());
    record("config.accel.heading", HEADING);
    record("config.accel.text.accel", ACCEL_TEXT);
    record("config.accel.text.level", LEVEL_TEXT);
    record("config.accel.text.simple", SIMPLE_TEXT);
    record("config.accel.calib", accel.accel_text());
    record("config.accel.calib.enabled", accel.accel_enabled());
    record("config.accel.level", accel.level_text());
    record("config.accel.level.enabled", accel.others_enabled());
    record("config.accel.simple", accel.simple_text());
    record("config.accel.simple.enabled", accel.others_enabled());
    record(
        "config.accel.label",
        if accel.label().is_empty() {
            "none"
        } else {
            accel.label()
        },
    );
    record("config.accel.position", position_name(accel.pos()));
    record("config.accel.incalibrate", accel.in_calibrate());
    record("config.accel.subscribed", accel.subscribed());
    record("config.accel.count", accel.count());
    let [starts, positions, levels, simples] = accel.sent();
    record("config.accel.sent.start", starts);
    record("config.accel.sent.position", positions);
    record("config.accel.sent.level", levels);
    record("config.accel.sent.simple", simples);
    record(
        "config.accel.message",
        accel
            .message()
            .map_or("none", |message| message.text.trim_end()),
    );
    for name in WATCHED {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// An absolutely placed box.
fn at((x, y, width, height): Place) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A label at its place, a line to a line: `AutoSize`, so its text is not wrapped.
fn text_block(id: &'static str, (x, y): (f32, f32), text: &'static str) -> AnyElement {
    crate::probe::measured(id, div())
        .absolute()
        .left(px(x))
        .top(px(y))
        .flex()
        .flex_col()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .whitespace_nowrap()
        .children(
            text.lines()
                .map(|line| div().child(SharedString::from(line))),
        )
        .into_any_element()
}

/// A `MyButton` at its place: its text centred; dimmed and inert while disabled.
fn button(
    id: &'static str,
    text: &'static str,
    place: Place,
    enabled: bool,
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .size_full()
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .text_xs()
        .whitespace_nowrap()
        .child(text);
    let base = if enabled {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::WARN))
            .text_color(rgb(theme::WARN))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                on_click(this);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
    };
    at(place).child(base).into_any_element()
}

/// The page, laid out as `ConfigAccelerometerCalibration.resx` lays it out. The page is docked
/// to fill the page area, so its `AutoSize` labels, wider than the Designer's 479, are drawn
/// whole: the body is as wide as the page column.
/// `// C#: GCSViews/ConfigurationView/ConfigAccelerometerCalibration.Designer.cs:31-121`
pub fn page(accel: &AccelCalibration, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    if !accel.is_active() {
        return None;
    }
    let (rule_x, rule_y, rule_w, rule_h) = RULE;
    let body = div()
        .relative()
        .w(px(760.0))
        .h(px(PAGE_SIZE.1))
        .child(
            div()
                .absolute()
                .left(px(HEADING_AT.0))
                .top(px(HEADING_AT.1))
                .text_base()
                .text_color(rgb(theme::TEXT))
                .child(HEADING),
        )
        .child(at((rule_x, rule_y, rule_w, rule_h)).bg(rgb(theme::BORDER)))
        .child(text_block("accel-text-accel", ACCEL_TEXT_AT, ACCEL_TEXT))
        .child(button(
            "accel-calib",
            accel.accel_text(),
            ACCEL_BUTTON,
            accel.accel_enabled(),
            |this| {
                let view = this.telemetry.view();
                this.accel_calibration.click_accel(&this.telemetry, &view);
            },
            cx,
        ))
        .child(
            at(USER_LABEL).child(
                crate::probe::measured("accel-user", div())
                    .size_full()
                    .flex()
                    .items_center()
                    .justify_center()
                    .text_sm()
                    .text_color(rgb(theme::WARN))
                    .child(accel.label().to_owned()),
            ),
        )
        .child(text_block("accel-text-level", LEVEL_TEXT_AT, LEVEL_TEXT))
        .child(button(
            "accel-level",
            accel.level_text(),
            LEVEL_BUTTON,
            accel.others_enabled(),
            |this| this.accel_calibration.click_level(&mut this.telemetry),
            cx,
        ))
        .child(text_block("accel-text-simple", SIMPLE_TEXT_AT, SIMPLE_TEXT))
        .child(button(
            "accel-simple",
            accel.simple_text(),
            SIMPLE_BUTTON,
            accel.others_enabled(),
            |this| this.accel_calibration.click_simple(&mut this.telemetry),
            cx,
        ));
    Some(panel(TITLE, body).into_any_element())
}

/// The message box showing, drawn over the whole window.
pub fn overlay(
    accel: &AccelCalibration,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let message = accel.message()?;
    let ok = action(
        "accel-message-ok",
        "OK",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.accel_calibration.dismiss_message();
            cx.notify();
        }),
    );
    Some(modal(
        "accel-message",
        message.title,
        &message.text,
        true,
        vec![ok],
        window,
    ))
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use mp_link::ProtocolTimeouts;
    use mp_link::messages::Severity;
    use mp_mavlink_dialects::all::Statustext;

    use super::*;
    use crate::telemetry::scripted::{VEHICLE, Vehicle, ack, until};

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    fn key(view: &TelemetryView) -> Key {
        Key::of(view)
    }

    /// A `STATUSTEXT` as ArduPilot sends it.
    fn statustext(text: &str) -> MavMessage {
        let mut raw = [0_u8; 50];
        for (slot, byte) in raw.iter_mut().zip(text.bytes()) {
            *slot = byte;
        }
        MavMessage::Statustext(Statustext {
            severity: 6,
            text: raw,
            id: 0,
            chunk_seq: 0,
        })
    }

    /// The vehicle asking for a position: `COMMAND_LONG` `ACCELCAL_VEHICLE_POS`, to the ground
    /// station.
    fn request(pos: u32) -> MavMessage {
        #[allow(clippy::cast_precision_loss)]
        let param1 = pos as f32;
        MavMessage::CommandLong(CommandLong {
            param1,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mp_calibration::CMD_ACCELCAL_VEHICLE_POS,
            target_system: 255,
            target_component: 190,
            confirmation: 0,
        })
    }

    /// The page, shown, over a connected scripted vehicle.
    fn shown() -> (Telemetry, Vehicle, AccelCalibration) {
        shown_with(fast())
    }

    /// The same, with these waits.
    fn shown_with(timeouts: ProtocolTimeouts) -> (Telemetry, Vehicle, AccelCalibration) {
        let (telemetry, vehicle) = Vehicle::connect(timeouts);
        let mut accel = AccelCalibration::default();
        accel.activate(key(&telemetry.view()));
        (telemetry, vehicle, accel)
    }

    /// Ticks until `check` holds.
    fn tick_until(
        telemetry: &Telemetry,
        accel: &mut AccelCalibration,
        what: &str,
        check: impl Fn(&AccelCalibration) -> bool,
    ) {
        until(what, || {
            accel.tick(telemetry, &telemetry.view(), true);
            check(accel)
        });
    }

    /// The calibration `COMMAND_LONG`s the vehicle has heard, as (command, param1, param5,
    /// target).
    fn commands(vehicle: &mut Vehicle) -> Vec<(u16, f32, f32, (u8, u8))> {
        vehicle
            .read()
            .into_iter()
            .filter_map(|message| match message {
                MavMessage::CommandLong(long)
                    if long.command == mp_calibration::CMD_PREFLIGHT_CALIBRATION
                        || long.command == mp_calibration::CMD_ACCELCAL_VEHICLE_POS =>
                {
                    Some((
                        long.command,
                        long.param1,
                        long.param5,
                        (long.target_system, long.target_component),
                    ))
                }
                _ => None,
            })
            .collect()
    }

    /// The `.resx`'s words and places, read from the tree when it is here.
    #[test]
    fn the_text_and_places_are_the_resx() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigAccelerometerCalibration.resx",
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let text = |name: &str| {
            values
                .get(&format!("{name}.Text"))
                .map(|text| text.replace("\r\n", "\n"))
        };
        assert_eq!(text("label5").as_deref(), Some(HEADING));
        assert_eq!(text("label4").as_deref(), Some(ACCEL_TEXT));
        assert_eq!(text("label1").as_deref(), Some(LEVEL_TEXT));
        assert_eq!(text("label2").as_deref(), Some(SIMPLE_TEXT));
        assert_eq!(text("BUT_calib_accell").as_deref(), Some(CALIBRATE_ACCEL));
        assert_eq!(text("BUT_level").as_deref(), Some(CALIBRATE_LEVEL));
        assert_eq!(
            text("BUT_simpleAccelCal").as_deref(),
            Some(SIMPLE_ACCEL_CAL)
        );
        let pair = |(x, y): (f32, f32)| format!("{x}, {y}");
        let location = |name: &str| values.get(&format!("{name}.Location")).cloned();
        let size = |name: &str| values.get(&format!("{name}.Size")).cloned();
        for (name, (x, y, w, h)) in [
            ("lineSeparator2", RULE),
            ("BUT_calib_accell", ACCEL_BUTTON),
            ("lbl_Accel_user", USER_LABEL),
            ("BUT_level", LEVEL_BUTTON),
            ("BUT_simpleAccelCal", SIMPLE_BUTTON),
        ] {
            assert_eq!(location(name), Some(pair((x, y))), "{name}");
            assert_eq!(size(name), Some(pair((w, h))), "{name}");
        }
        for (name, place) in [
            ("label5", HEADING_AT),
            ("label4", ACCEL_TEXT_AT),
            ("label1", LEVEL_TEXT_AT),
            ("label2", SIMPLE_TEXT_AT),
        ] {
            assert_eq!(location(name), Some(pair(place)), "{name}");
        }
        assert_eq!(size("$this"), Some(pair(PAGE_SIZE)));
        assert_eq!(
            values.get("lbl_Accel_user.TextAlign").map(String::as_str),
            Some("MiddleCenter")
        );
    }

    /// `Strings.resx`'s words for the buttons' changing texts.
    #[test]
    fn the_changing_texts_are_strings_resx() {
        let Some(resx) = crate::config_coverage::source::csharp("ExtLibs/Strings/Strings.resx")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |name: &str| values.get(name).map(|text| text.trim_end().to_owned());
        assert_eq!(get("Click_when_Done").as_deref(), Some(CLICK_WHEN_DONE));
        assert_eq!(get("Done").as_deref(), Some(DONE));
        assert_eq!(get("Completed").as_deref(), Some(COMPLETED));
        assert_eq!(get("CommandFailed").as_deref(), Some(COMMAND_FAILED));
        assert_eq!(get("ERROR").as_deref(), Some(ERROR_TITLE));
    }

    /// The names `ToString` gives, from the C#'s enum.
    #[test]
    fn the_position_names_are_the_enums() {
        if let Some(source) = crate::config_coverage::source::csharp("ExtLibs/Mavlink/Mavlink.cs") {
            let start = source
                .find("public enum ACCELCAL_VEHICLE_POS")
                .expect("the enum");
            let body = &source[start..start + source[start..].find("};").expect("its end")];
            for (value, name) in POSITIONS {
                assert!(body.contains(&format!("{name}={value},")), "{name}={value}");
            }
        }
        assert_eq!(position_name(1), "LEVEL");
        assert_eq!(position_name(16_777_216), "FAILED");
        assert_eq!(position_name(0), "0", "an undefined value is its number");
    }

    #[test]
    fn an_acknowledgement_line_is_told_from_a_statustext() {
        assert!(is_command_ack_line(
            "MAV_CMD_PREFLIGHT_CALIBRATION: accepted"
        ));
        assert!(is_command_ack_line(
            "MAV_CMD_ACCELCAL_VEHICLE_POS: temporarily rejected"
        ));
        assert!(is_command_ack_line("command 60000: unknown result"));
        assert!(!is_command_ack_line(
            "Place vehicle level and press any key."
        ));
        assert!(!is_command_ack_line("Calibration successful"));
        assert!(!is_command_ack_line("PreArm: Accels calibration: failed"));
        assert!(!is_command_ack_line("MAV_CMD_: accepted"));
    }

    /// The six-position conversation, driven as the C# drives it: the start goes out once with
    /// param5 1; the vehicle's requests and prompts are written into the label; each click sends
    /// the position back, broadcast; the vehicle's success ends it.
    #[test]
    fn the_conversation_runs_as_the_csharp_drives_it() {
        let (telemetry, mut vehicle, mut accel) = shown();
        accel.click_accel(&telemetry, &telemetry.view());
        assert_eq!(accel.accel_text(), CLICK_WHEN_DONE);
        assert!(accel.in_calibrate());
        assert!(accel.subscribed());
        assert!(accel.accel_enabled(), "nothing is disabled while it runs");
        assert!(accel.others_enabled());
        let mut heard = Vec::new();
        until("the start", || {
            heard.extend(commands(&mut vehicle));
            !heard.is_empty()
        });
        assert_eq!(
            heard,
            [(
                mp_calibration::CMD_PREFLIGHT_CALIBRATION,
                0.0,
                ACCEL_PARAM5,
                (VEHICLE.payload_target(), VEHICLE.compid)
            )]
        );
        // The vehicle's answer to the start is an acknowledgement, which says "calibration" and
        // is not a STATUSTEXT: the label is not written.
        vehicle.send(&ack(mp_calibration::CMD_PREFLIGHT_CALIBRATION, 0));
        wasm_thread::sleep(Duration::from_millis(20));
        accel.tick(&telemetry, &telemetry.view(), true);
        assert_eq!(accel.label(), "");

        // The request, then the prompt: the label is the last to arrive.
        vehicle.send(&request(1));
        tick_until(&telemetry, &mut accel, "the request", |accel| {
            accel.label() == "Please place vehicle LEVEL"
        });
        assert_eq!(accel.pos(), 1);
        vehicle.send(&statustext("Place vehicle level and press any key."));
        tick_until(&telemetry, &mut accel, "the prompt", |accel| {
            accel.label() == "Place vehicle level and press any key."
        });
        // The vehicle repeats its request each second, and each repetition is written again.
        vehicle.send(&request(1));
        tick_until(&telemetry, &mut accel, "the repeat", |accel| {
            accel.label() == "Please place vehicle LEVEL"
        });
        // A message that is not a prompt is not written.
        vehicle.send(&statustext("EKF3 IMU0 is using GPS"));
        wasm_thread::sleep(Duration::from_millis(20));
        accel.tick(&telemetry, &telemetry.view(), true);
        assert_eq!(accel.label(), "Please place vehicle LEVEL");

        // Click when Done: the position goes back, once, to nobody in particular.
        accel.click_accel(&telemetry, &telemetry.view());
        assert_eq!(accel.count(), 1);
        let mut heard = Vec::new();
        until("the position", || {
            heard.extend(commands(&mut vehicle));
            !heard.is_empty()
        });
        assert_eq!(
            heard,
            [(mp_calibration::CMD_ACCELCAL_VEHICLE_POS, 1.0, 0.0, (0, 0))]
        );

        // The next position.
        vehicle.send(&request(2));
        tick_until(&telemetry, &mut accel, "LEFT", |accel| {
            accel.label() == "Please place vehicle LEFT"
        });
        accel.click_accel(&telemetry, &telemetry.view());
        let mut heard = Vec::new();
        until("LEFT back", || {
            heard.extend(commands(&mut vehicle));
            !heard.is_empty()
        });
        assert_eq!(heard[0].1, 2.0);
        assert_eq!(accel.count(), 2);

        // Success ends it.
        vehicle.send(&statustext("Calibration successful"));
        tick_until(&telemetry, &mut accel, "the end", |accel| {
            !accel.subscribed()
        });
        assert_eq!(accel.label(), "Calibration successful");
        assert_eq!(accel.accel_text(), DONE);
        assert!(!accel.accel_enabled());
        assert!(!accel.in_calibrate());
        // What the vehicle says after is not heard.
        vehicle.send(&request(16_777_215));
        wasm_thread::sleep(Duration::from_millis(20));
        accel.tick(&telemetry, &telemetry.view(), true);
        assert_eq!(accel.label(), "Calibration successful");
        assert_eq!(accel.pos(), 2);
        // A disabled button takes no click.
        accel.click_accel(&telemetry, &telemetry.view());
        assert_eq!(accel.sent(), [1, 2, 0, 0]);
    }

    /// A failure ends it the same way, and `Activate` enables the button again with its text
    /// left, so the next click starts anew.
    #[test]
    fn a_failure_ends_it_and_showing_the_page_again_starts_anew() {
        let (telemetry, mut vehicle, mut accel) = shown();
        accel.click_accel(&telemetry, &telemetry.view());
        vehicle.send(&request(1));
        tick_until(&telemetry, &mut accel, "LEVEL", |accel| accel.pos() == 1);
        vehicle.send(&statustext("Calibration FAILED"));
        tick_until(&telemetry, &mut accel, "the failure", |accel| {
            !accel.subscribed()
        });
        assert_eq!(accel.label(), "Calibration FAILED");
        assert_eq!(accel.accel_text(), DONE);
        assert!(!accel.accel_enabled());

        accel.deactivate();
        accel.activate(key(&telemetry.view()));
        assert_eq!(accel.accel_text(), DONE, "Activate leaves the text");
        assert!(accel.accel_enabled());
        assert_eq!(accel.label(), "Calibration FAILED");
        commands(&mut vehicle);
        accel.click_accel(&telemetry, &telemetry.view());
        assert_eq!(accel.accel_text(), CLICK_WHEN_DONE);
        let mut heard = Vec::new();
        until("a second start", || {
            heard.extend(commands(&mut vehicle));
            !heard.is_empty()
        });
        assert_eq!(heard[0].0, mp_calibration::CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(heard[0].2, ACCEL_PARAM5);
        assert_eq!(accel.count(), 0);
    }

    /// Leaving the page mid-calibration ends `_incalibrate` - the next click starts again - but
    /// the page still listens while hidden.
    #[test]
    fn a_hidden_page_still_listens_and_its_next_click_starts_again() {
        let (telemetry, mut vehicle, mut accel) = shown();
        accel.click_accel(&telemetry, &telemetry.view());
        accel.deactivate();
        assert!(!accel.in_calibrate());
        assert!(accel.subscribed());
        vehicle.send(&request(3));
        tick_until(&telemetry, &mut accel, "RIGHT while hidden", |accel| {
            accel.label() == "Please place vehicle RIGHT"
        });
        accel.activate(key(&telemetry.view()));
        assert_eq!(accel.accel_text(), CLICK_WHEN_DONE);
        commands(&mut vehicle);
        accel.click_accel(&telemetry, &telemetry.view());
        let mut heard = Vec::new();
        until("the start again", || {
            heard.extend(commands(&mut vehicle));
            !heard.is_empty()
        });
        assert_eq!(heard[0].0, mp_calibration::CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(accel.sent()[..2], [2, 0]);
    }

    /// Leaving the screen disposes the page object: what it listened to and showed goes.
    #[test]
    fn leaving_the_screen_disposes_the_page() {
        let (telemetry, mut vehicle, mut accel) = shown();
        accel.click_accel(&telemetry, &telemetry.view());
        accel.deactivate();
        accel.tick(&telemetry, &telemetry.view(), false);
        assert!(!accel.subscribed());
        assert_eq!(accel.accel_text(), CALIBRATE_ACCEL);
        vehicle.send(&request(1));
        wasm_thread::sleep(Duration::from_millis(20));
        accel.tick(&telemetry, &telemetry.view(), false);
        assert_eq!(accel.label(), "");
        assert_eq!(accel.sent()[0], 1, "the counts outlive the page object");
    }

    /// Calibrate Level: param5 2, waited on; accepted, the button reads "Completed", and while it
    /// waits nothing on the page takes a click.
    #[test]
    fn calibrate_level_completes_when_the_vehicle_accepts() {
        let (mut telemetry, mut vehicle, mut accel) = shown();
        accel.click_level(&mut telemetry);
        assert!(!accel.others_enabled());
        assert!(!accel.accel_enabled());
        accel.click_simple(&mut telemetry);
        accel.click_accel(&telemetry, &telemetry.view());
        let mut heard = Vec::new();
        until("the level command", || {
            heard.extend(commands(&mut vehicle));
            !heard.is_empty()
        });
        assert_eq!(
            heard[0],
            (
                mp_calibration::CMD_PREFLIGHT_CALIBRATION,
                0.0,
                LEVEL_PARAM5,
                (VEHICLE.payload_target(), VEHICLE.compid)
            )
        );
        vehicle.send(&ack(mp_calibration::CMD_PREFLIGHT_CALIBRATION, 0));
        tick_until(&telemetry, &mut accel, "Completed", |accel| {
            accel.level_text() == COMPLETED
        });
        assert!(accel.others_enabled());
        assert!(accel.message().is_none());
        assert_eq!(accel.simple_text(), SIMPLE_ACCEL_CAL);
        assert_eq!(accel.sent(), [0, 0, 1, 0]);
    }

    /// Simple Accel Cal: param5 4; refused, `Strings.CommandFailed`, and the text stays.
    #[test]
    fn simple_accel_cal_refused_says_the_command_failed() {
        let (mut telemetry, mut vehicle, mut accel) = shown();
        accel.click_simple(&mut telemetry);
        let mut heard = Vec::new();
        until("the simple command", || {
            heard.extend(commands(&mut vehicle));
            !heard.is_empty()
        });
        assert_eq!(heard[0].2, SIMPLE_PARAM5);
        vehicle.send(&ack(mp_calibration::CMD_PREFLIGHT_CALIBRATION, 4));
        tick_until(&telemetry, &mut accel, "the refusal", |accel| {
            accel.message().is_some()
        });
        let message = accel.message().expect("a box");
        assert_eq!(message.text, COMMAND_FAILED);
        assert_eq!(message.title, ERROR_TITLE);
        assert_eq!(accel.simple_text(), SIMPLE_ACCEL_CAL);
        accel.dismiss_message();
        assert!(accel.message().is_none());
    }

    /// Unanswered, each says its own `catch`.
    #[test]
    fn an_unanswered_command_says_its_catch() {
        let (mut telemetry, _vehicle, mut accel) =
            shown_with(ProtocolTimeouts::default().faster(100));
        accel.click_level(&mut telemetry);
        tick_until(&telemetry, &mut accel, "the level timeout", |accel| {
            accel.message().is_some()
        });
        assert_eq!(
            accel.message().map(|m| m.text.as_str()),
            Some(FAILED_TO_LEVEL)
        );
        accel.dismiss_message();
        accel.click_simple(&mut telemetry);
        tick_until(&telemetry, &mut accel, "the simple timeout", |accel| {
            accel.message().is_some()
        });
        assert_eq!(
            accel.message().map(|m| m.text.as_str()),
            Some(FAILED_SIMPLE)
        );
        assert_eq!(accel.level_text(), CALIBRATE_LEVEL);
    }

    /// With no vehicle `doCommand` is false: the box, and nothing starts.
    #[test]
    fn without_a_vehicle_every_button_says_the_command_failed() {
        let mut telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut accel = AccelCalibration::default();
        accel.activate(key(&view));
        accel.click_accel(&telemetry, &view);
        assert!(!accel.in_calibrate());
        assert_eq!(accel.accel_text(), CALIBRATE_ACCEL);
        accel.click_level(&mut telemetry);
        accel.click_simple(&mut telemetry);
        assert_eq!(accel.messages.len(), 3);
        assert!(
            accel
                .messages
                .iter()
                .all(|message| message.text == COMMAND_FAILED)
        );
        assert_eq!(accel.sent(), [0; 4]);
    }

    /// Lines from before the start, and from another vehicle, are not the conversation's.
    #[test]
    fn only_the_vehicles_lines_after_the_start_are_read() {
        let mut accel = AccelCalibration::default();
        let line = |seq: u64, from: VehicleId, text: &str| LogMessage {
            from,
            severity: Severity::Info,
            text: text.to_owned(),
            seq,
            received: 0,
        };
        accel.subscription = Some(Subscription {
            vehicle: VEHICLE,
            seen: Some(4),
        });
        accel.in_calibrate = true;
        let telemetry = Telemetry::idle();
        let lines = [
            line(3, VEHICLE, "Calibration FAILED"),
            line(
                5,
                VehicleId::new(2, 1),
                "Place vehicle left and press any key.",
            ),
            line(6, VEHICLE, "Place vehicle level and press any key."),
        ];
        accel.listen(&telemetry, &lines);
        assert_eq!(accel.label(), "Place vehicle level and press any key.");
        assert!(accel.subscribed());
        assert_eq!(accel.subscription.and_then(|s| s.seen), Some(6));
    }

    /// Every fact the GUI script asserts on is one this page records, every control it clicks
    /// is one this page draws; and the script presses Calibrate Level alone.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-accel.gui");
        let source = include_str!("accel_calibration.rs");
        let mut facts = 0;
        let mut level = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.accel.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key}");
                    facts += 1;
                }
                (Some("expect"), Some(key)) if key.starts_with("params.value.") => {
                    let name = key.trim_start_matches("params.value.");
                    assert!(WATCHED.contains(&name), "{key} is not recorded here");
                }
                (Some("click"), Some(id)) if id.starts_with("accel-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id} is not drawn");
                    assert_ne!(id, "accel-calib", "the full calibration is not run on SITL");
                    assert_ne!(id, "accel-simple", "Simple Accel Cal is not run on SITL");
                    level += usize::from(id == "accel-level");
                }
                _ => {}
            }
        }
        assert!(facts > 15, "{facts} facts");
        assert_eq!(level, 1, "Calibrate Level is pressed once");
    }
}
