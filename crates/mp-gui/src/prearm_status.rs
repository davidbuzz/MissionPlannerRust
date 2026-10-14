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

//! The Prearm Checks window, `Controls/PrearmStatus.cs`: a click on the HUD's pre-arm line
//! while the vehicle is disarmed (`prearmhitzone`) makes a new form and shows it with `Show()`,
//! `TopMost`, over the flight screen and beside it (`hud1_prearmclick`). One multi-line text box
//! fills the form, ten pixels in from its edges.
//!
//! Two timers drive the form. `requestchecktimer`, every five seconds and once as the form is
//! made, asks the vehicle to run its pre-arm checks - `MAV_CMD_RUN_PREARM_CHECKS`, not waited
//! for - unless they already pass, and notes when it asked. `updatetexttimer`, every 100 ms,
//! closes the form once the vehicle is armed, and otherwise writes the box: "Ready to Arm" while
//! the checks pass; else each message of the vehicle's that begins "PreArm:", once, since the
//! last request that is more than a second old (everything it has said, until one is); or, with
//! none, that the checks are failing and it is waiting for the messages.
//!
//! Divergences, each at its site: the form is one window here, where every click opens another
//! in the C#, and it opens at the same place each time (`RestoreStartupLocation` is not kept, as
//! for the HUD's other two windows); the timers run on the frame; a message's time is its place
//! in the planner's list of the vehicle's messages, so "since the request" is "after the newest
//! message held when it was made"; the box is not for typing into, which the C# allows and
//! writes over a tenth of a second later.
//! `// C#: Controls/PrearmStatus.cs, Controls/PrearmStatus.Designer.cs, Controls/PrearmStatus.resx,
//! GCSViews/FlightData.cs:3439-3447, ExtLibs/Controls/HUD.cs:1227-1231`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use web_time::{Duration, Instant};

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::theme;

/// `$this.Text`. `// C#: Controls/PrearmStatus.Designer.cs:65`
pub const TITLE: &str = "Prearm Checks";
/// The box while the checks pass. `// C#: Controls/PrearmStatus.cs:63`
pub const READY: &str = "Ready to Arm";
/// The box while they fail and the vehicle has not said why. `// C#: Controls/PrearmStatus.cs:75`
pub const WAITING: [&str; 2] = ["Prearm checks failing", "Waiting for error messages..."];
/// `MAV_CMD.RUN_PREARM_CHECKS`.
const RUN_PREARM_CHECKS: u16 = 401;
/// `requestchecktimer.Interval`. `// C#: Controls/PrearmStatus.Designer.cs:52`
const REQUEST: Duration = Duration::from_millis(5000);
/// `updatetexttimer.Interval`, which the Designer leaves at a `Timer`'s own 100 ms.
/// `// C#: Controls/PrearmStatus.Designer.cs:48`
const UPDATE: Duration = Duration::from_millis(100);
/// How old the last request is before only what came after it is shown:
/// `lastRequestTime.AddSeconds(1)`. `// C#: Controls/PrearmStatus.cs:68`
const ANSWER: Duration = Duration::from_secs(1);
/// `ClientSize`. `// C#: Controls/PrearmStatus.Designer.cs:59`
const CLIENT: (f32, f32) = (384.0, 261.0);
/// `Padding`, inside which `TXT_PrearmErrors` is docked: at (10, 10), 364 by 241.
/// `// C#: Controls/PrearmStatus.Designer.cs:39-43, 63`
const PADDING: f32 = 10.0;
/// Where the window sits over the screen: beside EKF Status's place, which it is read with.
const AT: (f32, f32) = (920.0, 80.0);

/// The form's state: one form, made anew each time the HUD's line opens it.
#[derive(Debug, Default)]
pub struct PrearmStatus {
    /// Whether the form is showing.
    open: bool,
    /// `lastRequestTime`, with the newest message the planner held then: the messages after it
    /// came after the request. `None` before the first, `DateTime.MinValue`.
    last_request: Option<(Instant, Option<u64>)>,
    /// `searchTime`, as the newest message held when the request it was taken from was made:
    /// the box lists the messages after it, and every message while it is `None`.
    search_after: Option<u64>,
    /// When `requestchecktimer` last ticked.
    last_check: Option<Instant>,
    /// When `updatetexttimer` last ticked.
    last_update: Option<Instant>,
    /// `TXT_PrearmErrors.Text`, a line each.
    lines: Vec<String>,
    /// How many times the form has asked the vehicle to run its checks.
    requests: usize,
}

impl PrearmStatus {
    /// Whether the form is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open
    }

    /// `TXT_PrearmErrors.Text`, a line each: empty until `updatetexttimer` first ticks.
    #[must_use]
    pub fn lines(&self) -> &[String] {
        &self.lines
    }

    /// `new PrearmStatus()`, shown: the box empty, both timers started, and
    /// `requestchecktimer_Tick` run at once. Whether to ask the vehicle to run its checks, which
    /// is the caller's to send. The form is new each time, so nothing carries over.
    /// `// C#: Controls/PrearmStatus.cs:21-30, GCSViews/FlightData.cs:3439-3447`
    #[must_use]
    pub fn open(&mut self, view: &TelemetryView, now: Instant) -> bool {
        *self = Self {
            open: true,
            last_check: Some(now),
            last_update: Some(now),
            ..Self::default()
        };
        self.request_check(view, now)
    }

    /// The close box, or the vehicle armed: `Close()`, which stops both timers.
    pub fn close(&mut self) {
        *self = Self::default();
    }

    /// Both timers, run on the frame: `updatetexttimer` once 100 ms have passed, then - unless
    /// that has closed the form - `requestchecktimer` once five seconds have. Whether to ask the
    /// vehicle to run its checks, which is the caller's to send.
    #[must_use]
    pub fn tick(&mut self, view: &TelemetryView, now: Instant) -> bool {
        if !self.open {
            return false;
        }
        if due(self.last_update, UPDATE, now) {
            self.last_update = Some(now);
            self.update_text(view, now);
        }
        if !self.open || !due(self.last_check, REQUEST, now) {
            return false;
        }
        self.last_check = Some(now);
        self.request_check(view, now)
    }

    /// `requestchecktimer_Tick`: nothing while the checks pass; else the request, and when it
    /// was made. `doCommand` returns false rather than throwing with no port open, so the time
    /// is taken whether or not anything could be sent. `// C#: Controls/PrearmStatus.cs:32-53,
    /// ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2689-2690`
    fn request_check(&mut self, view: &TelemetryView, now: Instant) -> bool {
        if ready(view) {
            return false;
        }
        self.last_request = Some((now, view.messages.last().map(|message| message.seq)));
        self.requests += 1;
        true
    }

    /// `updatetexttimer_Tick`: the form closed once the vehicle is armed; "Ready to Arm" while
    /// its checks pass; else the pre-arm messages since `searchTime` - which moves to the last
    /// request once that is more than a second old - or, with none, the two lines saying the
    /// checks fail and it waits. `// C#: Controls/PrearmStatus.cs:55-77`
    fn update_text(&mut self, view: &TelemetryView, now: Instant) {
        // `this.Close()`: the C# goes on to write into the closed form, which shows nothing.
        if view.state.as_deref().is_some_and(|state| state.armed) {
            self.close();
            return;
        }
        if ready(view) {
            self.lines = vec![READY.to_owned()];
            return;
        }
        if let Some((at, newest)) = self.last_request
            && now.saturating_duration_since(at) > ANSWER
        {
            self.search_after = newest;
        }
        let mut lines = prearm_messages(view, self.search_after);
        if lines.is_empty() {
            lines = WAITING.map(str::to_owned).into();
        }
        self.lines = lines;
    }

    /// What a script can see: whether the form is showing, the box's text - "none" while it is
    /// empty or shut - and its lines, and how many times it has asked for the checks.
    pub fn record_facts(&self) {
        crate::facts::record("fly.prearm.open", self.open);
        crate::facts::record(
            "fly.prearm.text",
            if self.lines.is_empty() {
                "none".to_owned()
            } else {
                self.lines.join("\n")
            },
        );
        crate::facts::record("fly.prearm.lines", self.lines.len());
        crate::facts::record("fly.prearm.requests", self.requests);
    }
}

/// Whether a timer `every` long, last run at `last`, is due at `now`.
fn due(last: Option<Instant>, every: Duration, now: Instant) -> bool {
    last.is_none_or(|last| now.saturating_duration_since(last) >= every)
}

/// `cs.prearmstatus`: a link up - a port open or a log being read - and the vehicle's pre-arm
/// checks passing, or not enabled at all (`crate::hud::prearm_ready`); with no vehicle, a blank
/// `CurrentState`'s, which has neither bit and so reads as passing.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:179-182, 2012-2013`
#[must_use]
pub fn ready(view: &TelemetryView) -> bool {
    view.connected && view.state.as_deref().is_none_or(crate::hud::prearm_ready)
}

/// The vehicle's messages after `after` - all of them for `None` - that begin "prearm:" in any
/// case, each once, oldest first: `cs.messages.Where(m => m.time > searchTime &&
/// m.message.ToLower().StartsWith("prearm:")).Select(m => m.message).Distinct()`. Only the
/// vehicle being flown's, as `cs.messages` is its `CurrentState`'s.
/// `// C#: Controls/PrearmStatus.cs:71-72`
#[must_use]
pub fn prearm_messages(view: &TelemetryView, after: Option<u64>) -> Vec<String> {
    let mut found: Vec<String> = Vec::new();
    let Some(vehicle) = view.vehicle else {
        return found;
    };
    for message in &view.messages {
        if message.from == vehicle
            && after.is_none_or(|after| message.seq > after)
            && message.text.to_lowercase().starts_with("prearm:")
            && !found.contains(&message.text)
        {
            found.push(message.text.clone());
        }
    }
    found
}

impl MissionPlanner {
    /// `hud1_prearmclick`: `new PrearmStatus()` shown, which asks the vehicle to run its checks
    /// unless they pass. `ThemeManager.ApplyThemeTo` is the palette's business here.
    /// `// C#: GCSViews/FlightData.cs:3439-3447, Controls/PrearmStatus.cs:21-30`
    pub(crate) fn prearm_status_open(&mut self) {
        let view = self.telemetry.view();
        if self.fly_data.prearm.open(&view, Instant::now()) {
            self.prearm_status_ask();
        }
    }

    /// The form's two timers, on the frame.
    pub(crate) fn prearm_status_tick(&mut self, view: &TelemetryView) {
        if self.fly_data.prearm.tick(view, Instant::now()) {
            self.prearm_status_ask();
        }
    }

    /// `doCommand(sysidcurrent, compidcurrent, RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, false)`:
    /// one `COMMAND_LONG` to the vehicle being flown, not waited for. With no vehicle to send it
    /// to nothing goes, as `doCommand` sends nothing with no port open, and the C# does not look
    /// at what it returns. `// C#: Controls/PrearmStatus.cs:39-44`
    fn prearm_status_ask(&self) {
        let _ = self
            .telemetry
            .command_unacknowledged(RUN_PREARM_CHECKS, [0.0; 7]);
    }
}

/// The window, while the form is showing: `TXT_PrearmErrors` docked in the form's ten-pixel
/// padding, a line of its text a row.
/// `// C#: Controls/PrearmStatus.Designer.cs:37-65`
#[must_use]
pub fn window(prearm: &PrearmStatus, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    if !prearm.is_open() {
        return None;
    }
    let mut text = div()
        .size_full()
        .flex()
        .flex_col()
        .p_1()
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .text_xs()
        .text_color(rgb(theme::TEXT));
    for line in prearm.lines() {
        text = text.child(div().child(line.clone()));
    }
    let body = div()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .p(px(PADDING))
        .child(text);
    Some(crate::fly::floating_window(
        "fly-prearm-status",
        TITLE,
        "fly-prearm-close",
        AT,
        body,
        cx.listener(|this, _event: &(), _window, cx| {
            this.fly_data.prearm.close();
            cx.notify();
        }),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_link::messages::{LogMessage, Severity};
    use mp_mavlink_dialects::all::MavCmd;
    use mp_vehicle::{VehicleId, VehicleState};
    use std::sync::Arc;

    /// The vehicle being flown.
    const VEHICLE: VehicleId = VehicleId::new(1, 1);
    /// `MAV_SYS_STATUS_PREARM_CHECK`, in `SYS_STATUS`'s sensor words.
    const PREARM_CHECK: u32 = 0x1000_0000;

    fn said(seq: u64, from: VehicleId, text: &str) -> LogMessage {
        LogMessage {
            from,
            severity: Severity::Critical,
            text: text.to_owned(),
            seq,
            received: 0,
        }
    }

    /// A link up to a vehicle that checks before arming - passing or not, armed or not - which
    /// has said `messages`.
    fn view(passing: bool, armed: bool, messages: Vec<LogMessage>) -> TelemetryView {
        let mut state = VehicleState::default();
        state.sensors.enabled = PREARM_CHECK;
        if passing {
            state.sensors.health = PREARM_CHECK;
        }
        state.armed = armed;
        let mut view = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        view.connected = true;
        view.vehicle = Some(VEHICLE);
        view.state = Some(Arc::new(state));
        view.messages = messages;
        view
    }

    fn lines(form: &PrearmStatus) -> Vec<&str> {
        form.lines().iter().map(String::as_str).collect()
    }

    #[test]
    fn the_command_is_run_prearm_checks() {
        assert_eq!(
            u32::from(RUN_PREARM_CHECKS),
            MavCmd::MAV_CMD_RUN_PREARM_CHECKS.0
        );
    }

    /// Made: shown, the box empty, and the vehicle asked to run its checks at once - unless
    /// they pass. `// C#: Controls/PrearmStatus.cs:21-34`
    #[test]
    fn the_form_asks_for_the_checks_as_it_opens_unless_they_pass() {
        let now = Instant::now();
        let mut form = PrearmStatus::default();
        assert!(!form.is_open());
        assert!(form.open(&view(false, false, Vec::new()), now));
        assert!(form.is_open());
        assert!(form.lines().is_empty());
        assert_eq!(form.requests, 1);
        let mut passing = PrearmStatus::default();
        assert!(!passing.open(&view(true, false, Vec::new()), now));
        assert!(passing.is_open());
        assert_eq!(passing.requests, 0);
        // No link: `prearmstatus` is false whatever the bits say, so it asks.
        let mut unlinked = view(true, false, Vec::new());
        unlinked.connected = false;
        assert!(!ready(&unlinked));
        assert!(PrearmStatus::default().open(&unlinked, now));
        // A link and no vehicle: a blank state, which reads as passing.
        let mut blank = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        blank.connected = true;
        assert!(ready(&blank));
    }

    /// Every 100 ms the box is written: "Ready to Arm" while the checks pass, the two waiting
    /// lines while they fail with nothing said. `// C#: Controls/PrearmStatus.cs:55-76`
    #[test]
    fn the_box_says_ready_or_waits_for_the_messages() {
        let passing = view(true, false, Vec::new());
        let now = Instant::now();
        let mut form = PrearmStatus::default();
        assert!(!form.open(&passing, now));
        assert!(!form.tick(&passing, now + Duration::from_millis(50)));
        assert!(form.lines().is_empty(), "not yet 100 ms");
        assert!(!form.tick(&passing, now + UPDATE));
        assert_eq!(lines(&form), [READY]);
        let failing = view(false, false, Vec::new());
        assert!(!form.tick(&failing, now + Duration::from_millis(200)));
        assert_eq!(lines(&form), WAITING);
    }

    /// The vehicle's own messages beginning "PreArm:" in any case, each once, oldest first;
    /// not another component's or another vehicle's, and nothing without a vehicle.
    /// `// C#: Controls/PrearmStatus.cs:71-72`
    #[test]
    fn the_box_lists_each_prearm_message_once() {
        let gimbal = VehicleId::new(1, 154);
        let messages = vec![
            said(0, VEHICLE, "PreArm: Need Position Estimate"),
            said(1, VEHICLE, "Arming motors"),
            said(2, VEHICLE, "prearm: RC not calibrated"),
            said(3, gimbal, "PreArm: gimbal not healthy"),
            said(4, VEHICLE, "PreArm: Need Position Estimate"),
            said(5, VEHICLE, "Arm: Throttle too high"),
        ];
        let failing = view(false, false, messages);
        assert_eq!(
            prearm_messages(&failing, None),
            [
                "PreArm: Need Position Estimate",
                "prearm: RC not calibrated"
            ]
        );
        assert_eq!(
            prearm_messages(&failing, Some(2)),
            ["PreArm: Need Position Estimate"]
        );
        let mut nobody = failing.clone();
        nobody.vehicle = None;
        assert!(prearm_messages(&nobody, None).is_empty());

        let now = Instant::now();
        let mut form = PrearmStatus::default();
        assert!(form.open(&failing, now));
        assert!(!form.tick(&failing, now + UPDATE));
        assert_eq!(
            lines(&form),
            [
                "PreArm: Need Position Estimate",
                "prearm: RC not calibrated"
            ]
        );
    }

    /// For a second after a request the box keeps to what it searched before - everything,
    /// before the first - and then lists only what came after the request.
    /// `// C#: Controls/PrearmStatus.cs:67-72`
    #[test]
    fn a_second_after_a_request_only_what_followed_it_is_listed() {
        let old = said(0, VEHICLE, "PreArm: Compass not calibrated");
        let new = said(1, VEHICLE, "PreArm: Battery 1 low voltage");
        let before = view(false, false, vec![old.clone()]);
        let both = view(false, false, vec![old, new]);
        let now = Instant::now();
        let mut form = PrearmStatus::default();
        assert!(form.open(&before, now));
        assert!(!form.tick(&before, now + Duration::from_millis(500)));
        assert_eq!(lines(&form), ["PreArm: Compass not calibrated"]);
        // A second on, not more: the old one still, with the new.
        assert!(!form.tick(&both, now + ANSWER));
        assert_eq!(
            lines(&form),
            [
                "PreArm: Compass not calibrated",
                "PreArm: Battery 1 low voltage"
            ]
        );
        // Past it: only what came after the request.
        assert!(!form.tick(&both, now + Duration::from_millis(1100)));
        assert_eq!(lines(&form), ["PreArm: Battery 1 low voltage"]);
        assert!(!form.tick(&before, now + Duration::from_millis(1200)));
        assert_eq!(lines(&form), WAITING);
    }

    /// The request timer: every five seconds while the checks fail, nothing while they pass.
    /// `// C#: Controls/PrearmStatus.cs:32-53, Controls/PrearmStatus.Designer.cs:52`
    #[test]
    fn the_checks_are_asked_for_every_five_seconds_while_they_fail() {
        let failing = view(false, false, Vec::new());
        let now = Instant::now();
        let mut form = PrearmStatus::default();
        assert!(form.open(&failing, now));
        assert!(!form.tick(&failing, now + Duration::from_millis(4900)));
        assert!(form.tick(&failing, now + REQUEST));
        assert_eq!(form.requests, 2);
        assert!(!form.tick(&view(true, false, Vec::new()), now + REQUEST * 2));
        assert_eq!(form.requests, 2);
        assert!(form.tick(&failing, now + REQUEST * 3));
        assert_eq!(form.requests, 3);
    }

    /// Armed: the form closes itself, and its timers with it. `// C#: Controls/PrearmStatus.cs:58`
    #[test]
    fn arming_closes_the_form() {
        let now = Instant::now();
        let mut form = PrearmStatus::default();
        assert!(form.open(&view(false, false, Vec::new()), now));
        let armed = view(false, true, Vec::new());
        assert!(!form.tick(&armed, now + REQUEST));
        assert!(!form.is_open());
        assert!(form.lines().is_empty());
        assert_eq!(form.requests, 0);
        assert!(!form.tick(&armed, now + REQUEST * 2));
    }
}
