//! The flight screen: what the aircraft is doing, and the controls that change it.
//!
//! Equivalent to Mission Planner's Flight Data tab. The ordering on screen follows what a pilot
//! looks at, not what is easiest to lay out: attitude and mode first, then position and health,
//! then the message log, then the controls that commit something to the aircraft.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::Cell;
use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Bounds, Context, FocusHandle, KeyDownEvent, Pixels, SharedString, Window, div,
    prelude::*, px, rgb,
};
use mp_link::messages::{LogMessage, Severity, time_of_day};
use mp_link::requests::{self, RequestOutcome, RequestState};
use mp_link::{RequestId, commands};
use mp_mavlink_dialects::all::{DigicamControl, MavCmd, MavMessage};
use mp_mission::MissionItem;
use mp_vehicle::{VehicleFamily, VehicleId};

use crate::MissionPlanner;
use crate::i18n::fl;
use crate::telemetry::{Lookup, Report, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, action_sized, field, panel, theme};
// Aliased because `mp_link::messages::Severity` is already `Severity` here and means something
// else entirely: that one is how loud a STATUSTEXT was, this one is how bad a reading is.
use mp_vehicle::health::Severity as Health;

/// The height a right click on the map flies a vehicle still on the ground to, in metres above
/// home.
pub const TAKEOFF_ALTITUDE: f32 = 10.0;

/// `Settings.Instance["takeoff_alt", "5"]`: TakeOff's box offers the height last given, or this.
/// `// C#: GCSViews/FlightData.cs:5294`
pub const TAKEOFF_ALT_DEFAULT: &str = "5";

/// Width shared by the arm and force arm buttons.
///
/// Wide enough for "force arm" so both are the same size. They are the same decision with
/// different force, and two controls that do nearly the same thing should not look like different
/// kinds of control.
const ARM_BUTTON_WIDTH: gpui::Pixels = px(104.0);

/// The controls that command the aircraft.
///
/// Arm and disarm are separate buttons rather than one toggle. A toggle whose meaning depends on
/// state you have to read first is exactly the control you press wrongly when something is going
/// badly, and "disarm" pressed in the air is not recoverable.
///
/// Force arm is a third button rather than a modifier on the first, for the same reason: a control
/// whose effect depends on something you cannot see is one you press wrongly. It is coloured as
/// the hazard it is, and it is the only control here that tells the aircraft to ignore its own
/// judgement.
///
/// Order is arm, force arm, disarm - the two ways of arming together, because they are the same
/// intent with different force, and reaching for one and hitting the other is a mistake between
/// two things you meant. Only one of the two groups is ever live: force arm is disabled while
/// armed and disarm is disabled while disarmed, so there is no moment when a slip toward disarm
/// can arm anything.
///
/// Under them, Mission Planner's Actions tab (`tabActions`), then the mode list.
pub fn actions_panel(
    view: &TelemetryView,
    checks_disabled: bool,
    tab: &Actions,
    focus: &ActionsFocus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let armed = view.state.as_ref().is_some_and(|s| s.armed);
    let has_vehicle = view.vehicle.is_some();

    let controls = div()
        .flex()
        .flex_wrap()
        .gap_2()
        .child(action_sized(
            "arm",
            "arm",
            theme::OK,
            has_vehicle && !armed,
            Some(ARM_BUTTON_WIDTH),
            cx.listener(|this, _event: &(), _window, cx| {
                this.telemetry.arm(true);
                cx.notify();
            }),
        ))
        .child(action_sized(
            "force-arm",
            "force arm",
            theme::ALERT,
            has_vehicle && !armed,
            Some(ARM_BUTTON_WIDTH),
            cx.listener(|this, _event: &(), _window, cx| {
                this.begin_force_arm();
                cx.notify();
            }),
        ))
        .child(action(
            "disarm",
            "disarm",
            theme::ALERT,
            has_vehicle && armed,
            cx.listener(|this, _event: &(), _window, cx| {
                this.telemetry.arm(false);
                cx.notify();
            }),
        ))
        // The map menu's TakeOff (`takeOffToolStripMenuItem`), which this application's map has
        // no menu for: live while the link is open, as the handler asks only `BaseStream.IsOpen`,
        // armed or not - it does not arm.
        // `// C#: GCSViews/FlightData.cs:5290-5315; FlightData.resx:5443-5445`
        .child(action(
            "takeoff",
            "TakeOff",
            theme::ACCENT,
            has_vehicle,
            cx.listener(|this, _event: &(), window, cx| {
                let alt = this
                    .persisted
                    .get("takeoff_alt")
                    .unwrap_or(TAKEOFF_ALT_DEFAULT)
                    .to_owned();
                this.fly_actions.ask(Prompt::TakeOff, &alt);
                this.fly_focus.prompt.focus(window, cx);
                cx.notify();
            }),
        ))
        .child(action(
            "land",
            "land",
            theme::ACCENT,
            has_vehicle,
            cx.listener(|this, _event: &(), _window, cx| {
                this.telemetry.land();
                cx.notify();
            }),
        ))
        .child(action(
            "reboot",
            "reboot",
            theme::WARN,
            has_vehicle && !armed,
            cx.listener(|this, _event: &(), _window, cx| {
                this.telemetry.reboot();
                this.file_status = Some("reboot sent; the link will drop".to_owned());
                cx.notify();
            }),
        ))
        // Only after this session has turned them off. A vehicle that never had its checks
        // disabled does not need a button offering to restore them, and forcing is meant to stay
        // in force until the operator decides otherwise.
        .children(checks_disabled.then(|| {
            action(
                "restore-checks",
                "restore arming checks",
                theme::OK,
                has_vehicle,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.telemetry.enable_arming_checks();
                    this.disabled_arming_checks = false;
                    this.file_status =
                        Some("arming checks restored to the firmware default".to_owned());
                    cx.notify();
                }),
            )
        }));

    panel(
        "actions",
        div()
            .flex()
            .flex_col()
            .gap_3()
            .child(controls)
            .child(actions_tab(view, tab, focus, window, cx))
            .child(mode_controls(view, cx)),
    )
}

/// One button per flight mode this airframe offers.
///
/// The list is generated from the same parameter metadata the C# application reads, so mode 4 is
/// Guided on a copter and ACRO on a plane without this code knowing anything about either.
///
/// Not the C#'s: its Actions tab picks the mode with `CMB_modes` and Set Mode in the grid. Kept by
/// the owner's ruling of 2026-09-26 over restoring those - a mode is one click here where the
/// drop-down takes three - the Actions page scrolling where the vehicle's modes run past its
/// column.
fn mode_controls(view: &TelemetryView, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let modes = crate::telemetry::Telemetry::modes_for(view);
    if modes.is_empty() {
        return div()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .child("no mode list for this airframe")
            .into_any_element();
    }

    let current = view.state.as_ref().map(|s| s.custom_mode);
    let mut row = div().flex().flex_wrap().gap_1();
    for (number, name) in modes {
        let selected = current == Some(*number);
        let colour = if selected { theme::OK } else { theme::TEXT };
        let number = *number;
        row = row.child(
            crate::probe::measured(*name, div())
                .id(*name)
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(if selected { theme::OK } else { theme::BORDER }))
                .bg(rgb(if selected {
                    theme::ACTION
                } else {
                    theme::PANEL
                }))
                .text_xs()
                .text_color(rgb(colour))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child((*name).to_owned())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.telemetry.set_mode(number);
                    cx.notify();
                })),
        );
    }

    div()
        .flex()
        .flex_col()
        .gap_2()
        .child(div().text_xs().text_color(rgb(theme::DIM)).child("mode"))
        .child(row)
        .into_any_element()
}

/// Colour for a severity. Warnings and worse are coloured; routine chatter is not, so that a red
/// line on screen always means something.
const fn severity_colour(severity: Severity) -> u32 {
    match severity {
        Severity::Emergency | Severity::Alert | Severity::Critical | Severity::Error => {
            theme::ALERT
        }
        Severity::Warning => theme::WARN,
        Severity::Notice => theme::TEXT,
        Severity::Info | Severity::Debug => theme::DIM,
    }
}

/// What the aircraft has said.
///
/// Newest first, with a UTC timestamp per line, and it scrolls. Newest-first rather than the
/// console convention of appending at the bottom, because the pane has no way to follow the end on
/// its own: oldest-first would put every new message below the fold, and an operator would have to
/// scroll to see the one that just arrived - exactly when they have least attention to spare.
/// Scrolling down is for history, which can wait.
///
/// `STATUSTEXT` is how ArduPilot explains a refusal, so this pane is the difference between a pilot
/// who fixes a failed pre-arm check and one who keeps pressing the button.
pub fn messages_panel(view: &TelemetryView) -> impl IntoElement {
    let mut lines = div().flex().flex_col().gap_1();

    if view.messages.is_empty() {
        lines = lines.child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("nothing from the vehicle yet"),
        );
    }

    for message in view.messages.iter().rev() {
        lines = lines.child(
            div()
                .flex()
                .flex_shrink_0()
                .gap_2()
                .text_xs()
                .child(
                    div()
                        .flex_shrink_0()
                        .w(px(58.0))
                        .text_color(rgb(theme::DIM))
                        .child(time_of_day(message.received)),
                )
                .child(
                    div()
                        .flex_shrink_0()
                        .w(px(52.0))
                        .text_color(rgb(theme::DIM))
                        .child(message.severity.label()),
                )
                .child(
                    // One line per message, truncated. Wrapping would make each row a different
                    // height, which rules out virtualising the list later and stops the timestamp
                    // column lining up.
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .truncate()
                        .text_color(rgb(severity_colour(message.severity)))
                        .child(message.text.clone()),
                ),
        );
    }

    panel(
        "messages",
        div()
            .flex()
            .flex_col()
            .child(
                div()
                    .id("messages")
                    .flex()
                    .flex_col()
                    .h(px(190.0))
                    .overflow_y_scroll()
                    .child(lines),
            )
            .child(
                div()
                    .pt_1()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(message_footer(view)),
            ),
    )
}

/// The line under the message list: how much is shown, and how much was lost.
///
/// Saying what was dropped matters. A pane that silently discards the oldest messages implies the
/// flight was quiet when it was not, and the discarded ones are usually the boot-time narration
/// that explains everything after them.
fn message_footer(view: &TelemetryView) -> String {
    let shown = view.messages.len();
    match view.messages_dropped {
        0 if shown == 0 => "times are UTC".to_owned(),
        0 => format!("{shown} messages, newest first, times UTC"),
        dropped => format!(
            "{shown} messages, newest first, times UTC - {dropped} older dropped; the telemetry log has all of them"
        ),
    }
}

/// Why the aircraft will not arm.
///
/// Two sources, because neither alone is enough. `SYS_STATUS` names which sensors are switched on
/// and not working, which is structured and always current. `STATUSTEXT` carries ArduPilot's own
/// explanations - "PreArm: Compass not calibrated" - which are specific but arrive only when the
/// vehicle decides to say them, and scroll away in the message log.
///
/// Shown only while disarmed. Once the aircraft is flying, a pre-arm panel is a distraction from
/// the ones that matter.
pub fn prearm_panel(view: &TelemetryView) -> AnyElement {
    let Some(state) = view.state.as_ref() else {
        return div().into_any_element();
    };
    if state.armed {
        return div().into_any_element();
    }

    let unhealthy = state.sensors.unhealthy();
    let unnamed = state
        .sensors
        .unhealthy_count()
        .saturating_sub(u32::try_from(unhealthy.len()).unwrap_or(u32::MAX));

    // The most recent of each distinct complaint, newest first. ArduPilot repeats them every few
    // seconds, so without collapsing duplicates the panel would be one message twenty times.
    let mut seen = std::collections::BTreeSet::new();
    let mut complaints: Vec<String> = Vec::new();
    for message in view.messages.iter().rev() {
        let text = message.text.trim();
        let is_prearm = text.starts_with("PreArm:") || text.starts_with("Arm:");
        if is_prearm && seen.insert(text.to_owned()) {
            complaints.push(text.to_owned());
        }
        if complaints.len() >= 6 {
            break;
        }
    }

    if unhealthy.is_empty() && unnamed == 0 && complaints.is_empty() {
        let ready = state.sensors.all_healthy();
        return panel(
            "pre-arm",
            div()
                .text_xs()
                .text_color(rgb(if ready { theme::OK } else { theme::DIM }))
                .child(if ready {
                    "every sensor that is switched on is healthy"
                } else {
                    "waiting for the vehicle to report its sensors"
                }),
        )
        .into_any_element();
    }

    let mut lines = div().flex().flex_col().gap_1();

    if !unhealthy.is_empty() {
        lines = lines.child(
            div()
                .flex()
                .gap_2()
                .text_xs()
                .child(
                    div()
                        .flex_shrink_0()
                        .w(px(56.0))
                        .text_color(rgb(theme::DIM))
                        .child("sensors"),
                )
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .text_color(rgb(theme::ALERT))
                        .child(unhealthy.join(", ").to_lowercase()),
                ),
        );
    }
    if unnamed > 0 {
        // Counted from the bits, so a sensor this dialect has no name for is still reported. A
        // pilot told everything is healthy while one is failing is worse off than one told
        // "1 unnamed" with no name for it.
        lines = lines.child(div().text_xs().text_color(rgb(theme::WARN)).child(format!(
            "{unnamed} unhealthy sensor(s) this dialect cannot name"
        )));
    }
    for complaint in &complaints {
        lines = lines.child(
            div()
                .flex()
                .gap_2()
                .text_xs()
                .child(
                    div()
                        .flex_shrink_0()
                        .w(px(56.0))
                        .text_color(rgb(theme::DIM))
                        .child("vehicle"),
                )
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .text_color(rgb(theme::ALERT))
                        .child(complaint.clone()),
                ),
        );
    }

    panel("pre-arm", lines).into_any_element()
}

/// Fix quality, power and link health: everything that decides whether to fly, and whether the
/// aircraft is still listening.
///
/// One panel rather than two. They were separate and each cost a title, a border and twenty-four
/// pixels of padding for four numbers - which is how the panel below them ended up off the bottom
/// of the column. They answer one question between them: is this aircraft fit to fly right now.
pub fn health_panel(view: &TelemetryView) -> impl IntoElement {
    let (fix_text, fix_colour) = match view.state.as_ref().map(|s| s.gps.fix_type) {
        Some(0 | 1) | None => ("no fix".to_owned(), theme::ALERT),
        Some(2) => ("2D".to_owned(), theme::WARN),
        Some(3) => ("3D".to_owned(), theme::OK),
        Some(4) => ("DGPS".to_owned(), theme::OK),
        Some(5) => ("RTK float".to_owned(), theme::OK),
        Some(other) if other >= 6 => ("RTK fixed".to_owned(), theme::OK),
        Some(other) => (format!("fix {other}"), theme::WARN),
    };
    let sats = view
        .state
        .as_ref()
        .map_or_else(|| "--".to_owned(), |s| s.gps.satellites_visible.to_string());
    // MAVLink uses -1 for "this vehicle does not know", which is not the same as zero and must
    // not be printed as "-1%". A bench board with no battery attached reports exactly that.
    let battery = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |s| {
            let volts = if s.battery.voltage > 0.0 {
                format!("{:.1} V", s.battery.voltage)
            } else {
                "-- V".to_owned()
            };
            if s.battery.remaining_percent < 0 {
                volts
            } else {
                format!("{volts}  {}%", s.battery.remaining_percent)
            }
        },
    );
    let battery_colour = view.state.as_ref().map_or(theme::DIM, |s| {
        // Ten percent is where a copter's own failsafe starts acting, so it is where the number
        // should start shouting rather than an arbitrary round figure.
        if s.battery.remaining_percent >= 0 && s.battery.remaining_percent < 10 {
            theme::ALERT
        } else {
            theme::TEXT
        }
    });
    let loss = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |s| format!("{:.1} %", s.link.loss_percent()),
    );
    let loss_colour = view.state.as_ref().map_or(theme::DIM, |s| {
        if s.link.loss_percent() > 5.0 {
            theme::ALERT
        } else {
            theme::TEXT
        }
    });
    // The two readouts that explain a vehicle nobody can explain, said in a word rather than a
    // number: "0.62" means nothing to a pilot in a paddock.
    let (ekf_text, ekf_severity) = view.state.as_ref().map_or_else(
        || ("--".to_owned(), Health::Good),
        |s| {
            if s.ekf.seen {
                let (name, variance) = s.ekf.worst();
                let severity = s.ekf.severity();
                (
                    match severity {
                        Health::Good => format!("ok ({variance:.2})"),
                        _ => format!("{} {name} {variance:.2}", severity.label()),
                    },
                    severity,
                )
            } else {
                ("--".to_owned(), Health::Good)
            }
        },
    );
    let (vibration_text, vibration_severity) = view.state.as_ref().map_or_else(
        || ("--".to_owned(), Health::Good),
        |s| {
            if s.vibration.seen {
                let (axis, level) = s.vibration.worst();
                let clipping = s.vibration.total_clipping();
                let severity = s.vibration.severity();
                // The axis is named only when it matters. On a healthy vehicle "0 x" reads as
                // "zero times" rather than "zero on the x axis", and the axis is only interesting
                // once there is something to chase down.
                let text = match (severity, clipping) {
                    (Health::Good, 0) => format!("{level:.0} m/s\u{b2}"),
                    (Health::Good, clips) => format!("{level:.0} m/s\u{b2}, {clips} clips"),
                    (_, 0) => format!("{level:.0} on {axis}"),
                    (_, clips) => format!("{level:.0} on {axis}, {clips} clips"),
                };
                (text, severity)
            } else {
                ("--".to_owned(), Health::Good)
            }
        },
    );
    let crc_colour = if view.crc_errors > 0 {
        theme::WARN
    } else {
        theme::TEXT
    };

    panel(
        "health",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .flex()
                    .gap_3()
                    .child(field("fix", fix_text, fix_colour))
                    .child(field("satellites", sats, theme::TEXT))
                    .child(field("battery", battery, battery_colour)),
            )
            .child(
                div()
                    .flex()
                    .gap_3()
                    .child(field("frames", view.frames.to_string(), theme::TEXT))
                    .child(field("loss", loss, loss_colour))
                    .child(field("crc errors", view.crc_errors.to_string(), crc_colour))
                    .child(field(
                        "systems",
                        view.vehicle_count.to_string(),
                        theme::TEXT,
                    )),
            )
            .child(
                div()
                    .flex()
                    .gap_3()
                    .child(field("ekf", ekf_text, health_colour(ekf_severity)))
                    .child(field(
                        "vibration",
                        vibration_text,
                        health_colour(vibration_severity),
                    )),
            ),
    )
}

/// The colour a verdict gets.
const fn health_colour(health: Health) -> u32 {
    match health {
        Health::Good => theme::TEXT,
        Health::Warning => theme::WARN,
        Health::Bad => theme::ALERT,
    }
}

// -------------------------------------------------------------------------------------------------
// The Actions tab.
//
// `tabActions` in `GCSViews/FlightData.Designer.cs:736-760`: a 5x5 `TableLayoutPanel` of combo
// boxes, buttons and `ModifyandSet` number boxes, every column and row 20%. Each control ported
// here sits in the cell `tableLayoutPanel1.LayoutSettings` gives it (`GCSViews/FlightData.resx:1371`).
// The ones this application already has elsewhere - the quick mode buttons and `CMB_modes` (the
// mode chips above), Arm/Disarm (the buttons above), Joystick (the setup screen) - are not drawn a
// second time, so their cells are empty; so are the ones not ported yet.
//
// Everything that decides what goes on the wire is in plain functions below, tested without a
// window; the rendering and the handlers that send come after them.
// -------------------------------------------------------------------------------------------------

/// `CurrentState.multiplieralt`: 1 unless Mission Planner's `altunits` setting is feet. This
/// application shows metres throughout, so it is 1 - kept as a name so that each division the C#
/// makes by it is visible where the C# makes it.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:37, MainV2.cs:4272-4292`
pub const MULTIPLIER_ALT: f32 = 1.0;

/// `CurrentState.multiplierdist`, for the same reason.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:27, MainV2.cs:4249-4270`
pub const MULTIPLIER_DIST: f32 = 1.0;

/// The texts the C#'s message boxes show, from `Strings.resx`.
/// `// C#: ExtLibs/Strings/Strings.resx:126-128 (CommandFailed) and by name`
pub mod strings {
    /// `Strings.CommandFailed`.
    pub const COMMAND_FAILED: &str = "The Command failed to execute";
    /// `Strings.InvalidField`.
    pub const INVALID_FIELD: &str = "Invalid Field";
    /// `Strings.BadCoords`.
    pub const BAD_COORDS: &str = "Bad Lat/Lng";
    /// `Strings.ErrorNoResponse`.
    pub const ERROR_NO_RESPONSE: &str = "Error: no response from MAV";
    /// `Strings.ErrorCommunicating`.
    /// `// C#: ExtLibs/Strings/Strings.resx:136-138`
    pub const ERROR_COMMUNICATING: &str = "Error communicating with the autopilot";
    /// `Strings.Completed`.
    /// `// C#: ExtLibs/Strings/Strings.resx:537-539`
    pub const COMPLETED: &str = "Completed";
    /// The literal `CustomMessageBox.Show("Bad Alt")` in `flyToHereAltToolStripMenuItem_Click`.
    pub const BAD_ALT: &str = "Bad Alt";
}

/// A message box titled `Strings.ERROR`, as the status line says it.
#[must_use]
pub fn error_box(text: impl std::fmt::Display) -> String {
    format!("Error: {text}")
}

/// Why a press sent nothing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Refusal {
    /// The C# shows a message box: said on the status line as an error.
    Error(String),
    /// The C# returns without a word: said on the status line, quietly, so a press that did
    /// nothing is not mistaken for one that did.
    Quiet(String),
}

impl Refusal {
    fn error(text: impl Into<String>) -> Self {
        Self::Error(text.into())
    }

    fn quiet(text: impl Into<String>) -> Self {
        Self::Quiet(text.into())
    }

    /// The words, whichever kind it is.
    #[must_use]
    pub fn text(&self) -> &str {
        match self {
            Self::Error(text) | Self::Quiet(text) => text,
        }
    }
}

/// What a press puts on the wire, or why it puts nothing.
pub type Sends = Result<Vec<MavMessage>, Refusal>;

/// How one of a press's messages reaches the vehicle.
///
/// Through the link's retrying request where the C# blocks on the call that sends it - the link
/// then sends it again as the C# does, and the press's [`Report`] is said when it ends - and
/// straight onto the wire where the C# does not wait.
#[derive(Debug, Clone, PartialEq)]
pub enum Route {
    /// `setWPCurrent`: `MISSION_SET_CURRENT` until a `MISSION_CURRENT` arrives.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2452-2501`
    SetCurrent {
        /// The vehicle.
        target: VehicleId,
        /// The item made current.
        seq: u16,
    },
    /// `doCommand` with `requireack`: `COMMAND_LONG` until its `COMMAND_ACK`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2688-2836`
    Command {
        /// The vehicle.
        target: VehicleId,
        /// `MAV_CMD`.
        command: u16,
        /// param1 to param7.
        params: [f32; 7],
    },
    /// `doCommandInt`: `COMMAND_INT` until its `COMMAND_ACK`, anything but accepted a refusal.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2847-2951`
    CommandInt {
        /// The vehicle.
        target: VehicleId,
        /// `MAV_CMD`.
        command: u16,
        /// `MAV_FRAME`.
        frame: u8,
        /// param1 to param4.
        params: [f32; 4],
        /// param5, an integer.
        x: i32,
        /// param6, an integer.
        y: i32,
        /// param7.
        z: f32,
    },
    /// `setWP` for one item: a `MISSION_ITEM` or `MISSION_ITEM_INT` until the vehicle
    /// acknowledges it or asks for the next - Change Alt, ArduPlane's guided target.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3975-4380`
    SetWp {
        /// The vehicle.
        target: VehicleId,
        /// The item, as built. Boxed: a message is the largest thing a route holds.
        item: Box<MavMessage>,
    },
    /// `getHomePosition`: `GET_HOME_POSITION` until a `HOME_POSITION` arrives.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3343-3387`
    GetHome {
        /// The vehicle.
        target: VehicleId,
    },
    /// `setParam`: `PARAM_SET` until the vehicle echoes the parameter.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1628-1770`
    SetParam {
        /// The vehicle.
        target: VehicleId,
        /// The parameter.
        name: String,
        /// The value asked for.
        value: f64,
    },
    /// Sent once and not waited for.
    Raw,
}

/// Which way a message the flight screen sends goes: [`Route`].
///
/// Decided by what the C# does with the message the screen builds, and the screen builds each
/// one only where the C# sends it:
///
/// * `MISSION_SET_CURRENT` is only ever `setWPCurrent`'s, which waits.
/// * `COMMAND_LONG` is `doCommand`'s, which waits - except `DO_SET_MODE`, which `setMode` sends
///   with `requireack` false (:4636), and `PREFLIGHT_REBOOT_SHUTDOWN`, which `doCommand` sends
///   twice and does not wait for (:2758-2763); the press already holds both of those sends.
/// * `PARAM_SET` is only ever `setParam`'s, which waits.
/// * Everything else goes once. `SET_MODE`, `SET_POSITION_TARGET_GLOBAL_INT` and `SYSTEM_TIME`
///   are `generatePacket` and `sendPacket` in the C#, not waited for.
/// * `COMMAND_INT` is `doCommandInt`'s, which waits for its ack (:2847-2951); a `MISSION_ITEM`
///   or `MISSION_ITEM_INT` from a press - Change Alt, ArduPlane's guided target - is `setWP`'s,
///   which waits for `MISSION_ACK` or the request for the next item (:3975-4380); and
///   `GET_HOME_POSITION` from a press is `getHomePosition`'s, which waits for `HOME_POSITION`
///   (:3343-3387). Each has its request in the link since PLAN.md §13.6 row 74.
///
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs` for every line above.
#[must_use]
pub fn route(message: &MavMessage) -> Route {
    match message {
        MavMessage::MissionSetCurrent(m) => Route::SetCurrent {
            target: VehicleId::new(m.target_system, m.target_component),
            seq: m.seq,
        },
        MavMessage::CommandLong(m)
            if m.command == commands::CMD_DO_SET_MODE
                || m.command == requests::CMD_PREFLIGHT_REBOOT_SHUTDOWN =>
        {
            Route::Raw
        }
        MavMessage::CommandLong(m) if m.command == requests::CMD_GET_HOME_POSITION => {
            Route::GetHome {
                target: VehicleId::new(m.target_system, m.target_component),
            }
        }
        MavMessage::CommandInt(m) => Route::CommandInt {
            target: VehicleId::new(m.target_system, m.target_component),
            command: m.command,
            frame: m.frame,
            params: [m.param1, m.param2, m.param3, m.param4],
            x: m.x,
            y: m.y,
            z: m.z,
        },
        MavMessage::MissionItem(m) => Route::SetWp {
            target: VehicleId::new(m.target_system, m.target_component),
            item: Box::new(*message),
        },
        MavMessage::MissionItemInt(m) => Route::SetWp {
            target: VehicleId::new(m.target_system, m.target_component),
            item: Box::new(*message),
        },
        MavMessage::CommandLong(m) => Route::Command {
            target: VehicleId::new(m.target_system, m.target_component),
            command: m.command,
            params: [
                m.param1, m.param2, m.param3, m.param4, m.param5, m.param6, m.param7,
            ],
        },
        MavMessage::ParamSet(m) => Route::SetParam {
            target: VehicleId::new(m.target_system, m.target_component),
            name: mp_params::decode_param_id(&m.param_id),
            value: f64::from(m.param_value),
        },
        _ => Route::Raw,
    }
}

/// One `ModifyandSet`: a `NumericUpDown` beside a button.
/// `// C#: Controls/ModifyandSet.cs`
#[derive(Debug)]
pub struct ModifyAndSet {
    /// What the number box shows.
    pub field: TextField,
    value: f64,
    minimum: f64,
    maximum: f64,
    decimals: usize,
}

impl ModifyAndSet {
    fn new(value: f64, minimum: f64, maximum: f64, decimals: usize) -> Self {
        let mut field = TextField::new("");
        field.set(format!("{value:.decimals$}"));
        Self {
            field,
            value,
            minimum,
            maximum,
            decimals,
        }
    }

    /// `modifyandSetSpeed`: one decimal place, 0 to 1000, 100 to start.
    /// `// C#: GCSViews/FlightData.Designer.cs:895-922`
    #[must_use]
    pub fn speed() -> Self {
        Self::new(100.0, 0.0, 1000.0, 1)
    }

    /// `modifyandSetAlt`: one decimal place, 0 to 10000, 100 to start.
    /// `// C#: GCSViews/FlightData.Designer.cs:867-893`
    #[must_use]
    pub fn alt() -> Self {
        Self::new(100.0, 0.0, 10_000.0, 1)
    }

    /// `modifyandSetLoiterRad`: whole numbers, -10000 to 10000, 100 to start.
    /// `// C#: GCSViews/FlightData.Designer.cs:796-822`
    #[must_use]
    pub fn loiter_rad() -> Self {
        Self::new(100.0, -10_000.0, 10_000.0, 0)
    }

    /// The value the button acts on.
    ///
    /// A `NumericUpDown` validates when focus leaves it for the button: text that parses becomes
    /// the value, constrained to the box's range, and text that does not is dropped and the last
    /// good value shown again. Either way the box then shows the value at its decimal places.
    pub fn commit(&mut self) -> f64 {
        if let Ok(typed) = self.field.value().trim().parse::<f64>()
            && typed.is_finite()
        {
            self.value = typed.clamp(self.minimum, self.maximum);
        }
        let decimals = self.decimals;
        self.field.set(format!("{:.decimals$}", self.value));
        self.value
    }
}

/// `MAV.GuidedMode`, the fields the flight screen reads: where the vehicle was last sent in
/// Guided, and the height and frame the next Fly To Here uses.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:324`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GuidedMode {
    /// Latitude, degrees x 1e7.
    pub x: i32,
    /// Longitude, degrees x 1e7.
    pub y: i32,
    /// Height, metres.
    pub z: f32,
    /// `MAV_FRAME` of the height.
    pub frame: u8,
}

/// The frames `AltInputBox` offers, in its order, with its names.
/// `// C#: ExtLibs/Controls/AltInputBox.cs:77-79`
pub const ALT_FRAMES: [(&str, u8); 3] = [
    ("Absolute", commands::FRAME_GLOBAL),
    ("Relative", commands::FRAME_GLOBAL_RELATIVE_ALT),
    ("Terrain", commands::FRAME_GLOBAL_TERRAIN_ALT),
];

/// A question the C# asks in a dialog, asked here in a strip under the Actions grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prompt {
    /// `CustomMessageBox.Show("Are you sure you want to do " + CMB_action.Text + " ?", "Action",
    /// YesNo)`. `// C#: GCSViews/FlightData.cs:1774-1776`
    ConfirmAction(usize),
    /// `Common.MessageShowAgain("Resume Mission", ...)`. `// C#: GCSViews/FlightData.cs:1483-1487`
    ResumeWarning,
    /// `InputBox.Show("Resume at", "Resume mission at waypoint#", ref lastwp)`.
    /// `// C#: GCSViews/FlightData.cs:1497`
    ResumeAt,
    /// `InputBox.Show("Enter Fly To Coords", ...)`. `// C#: GCSViews/FlightData.cs:5941`
    FlyToCoords,
    /// `AltInputBox.Show("Enter Alt", "Enter Guided Mode Alt", ref alt, ref frame)`, with the
    /// frame chosen so far. `// C#: GCSViews/FlightData.cs:2918`
    FlyToHereAlt {
        /// The `MAV_FRAME` selected in the box.
        frame: u8,
    },
    /// TakeOff's `InputBox.Show("Enter Alt", "Enter Takeoff Alt", ref alt)`.
    /// `// C#: GCSViews/FlightData.cs:5296`
    TakeOff,
    /// `InputBox.Show("POI", "Enter ID", ref output)`. `// C#: Utilities/POI.cs:73`
    PoiId,
    /// `InputBox.Show("Enter POI Coords", ...)`. `// C#: GCSViews/FlightData.cs:6014`
    PoiCoords,
    /// Load Log's `OpenFileDialog`, which this application has no platform dialog for: the path
    /// is typed, into a box that starts in the dialog's `InitialDirectory`, `tlogdir`, and the
    /// words are the dialog's first filter. `// C#: GCSViews/FlightData.cs:1276-1302`
    LoadLog,
    /// One of the DataFlash Logs page's conversions: its `OpenFileDialog`, typed as Load Log's
    /// is, titled with the button and worded with the dialog's first filter.
    /// `// C#: GCSViews/FlightData.cs:1084-1089, 1137-1151, 1313-1317, Log/MatLabForms.cs:45-59`
    Convert(Conversion),
    /// `InputBox.Show("Jump to Tag", "Tag Id:", ref tag_str)`. `// C#: GCSViews/FlightData.cs:6504-6509`
    JumpToTag,
    /// `InputBox.Show("Hud Header", "Please enter your item prefix", ref prefix)`, for a User
    /// Items box just checked. `// C#: GCSViews/FlightData.cs:2445-2455`
    HudHeader,
    /// Set Home Here's `CustomMessageBox.Show("This will reset ...", "Are you sure?", OKCancel)`.
    /// `// C#: GCSViews/FlightData.cs:4866-4869`
    SetHome,
    /// Message's `InputBox.Show("Enter Message", "Enter Message to be logged", ref txt)`.
    /// `// C#: GCSViews/FlightData.cs:1264`
    SendMessage,
    /// Point Camera Here's `InputBox.Show("Enter Alt", "Enter Target Alt (Relative to home)",
    /// ref alt)`. `// C#: GCSViews/FlightData.cs:4519-4522`
    PointCameraAlt,
    /// Point Camera Coords' `InputBox.Show("Enter Coords", ..., ref location)`.
    /// `// C#: GCSViews/FlightData.cs:4481`
    PointCameraCoords,
    /// The POI menu's Save File: `POISave`'s `SaveFileDialog`, typed as Load Log's is, titled
    /// with the entry and worded with the dialog's filter. `// C#: Utilities/POI.cs:143-154`
    PoiSave,
    /// The POI menu's Load File: `POILoad`'s `OpenFileDialog`. `// C#: Utilities/POI.cs:171-182`
    PoiLoad,
    /// `openScriptDialog`: the Scripts tab's Select Script.
    SelectScript,
    /// Set View Count's first question, `InputBox.Show("Columns", "Enter number of columns to
    /// have.", ref cols)`. `// C#: GCSViews/FlightData.cs:5088`
    ViewColumns,
    /// Its second, `InputBox.Show("Rows", "Enter number of rows to have.", ref rows)`.
    /// `// C#: GCSViews/FlightData.cs:5090`
    ViewRows,
    /// Battery Cell Voltage's `InputBox.Show("Battery Cell Count", "Cell Count", ref
    /// CellCount)`. `// C#: GCSViews/FlightData.cs:6127`
    CellCount,
    /// The speed dial's double click: `InputBox.Show("Enter Max Speed", "Enter Max Speed", ref
    /// max)`. `// C#: GCSViews/FlightData.cs:3140-3143`
    GaugeMax,
    /// Set MJPEG source's `InputBox.Show("Mjpeg url", "Enter the url to the mjpeg source url",
    /// ref url)`. `// C#: GCSViews/FlightData.cs:4898`
    MjpegUrl,
    /// Set GStreamer Source's `InputBox.Show("GStreamer url", "Enter the source pipeline\n...",
    /// ref url)`. `// C#: GCSViews/FlightData.cs:4819-4821`
    GStreamerUrl,
    /// HereLink Video's `InputBox.Show("herelink ip", "Enter herelink ip address", ref ipaddr)`.
    /// `// C#: GCSViews/FlightData.cs:3162`
    HereLinkIp,
}

impl Prompt {
    /// The dialog's title.
    #[must_use]
    pub fn title(self) -> &'static str {
        match self {
            Self::ConfirmAction(_) => "Action",
            Self::ResumeWarning => "Resume Mission",
            Self::ResumeAt => "Resume at",
            Self::FlyToCoords => "Enter Fly To Coords",
            Self::FlyToHereAlt { .. } | Self::TakeOff => "Enter Alt",
            Self::PoiId => crate::poi::ID_TITLE,
            Self::PoiCoords => crate::poi::COORDS_TITLE,
            Self::LoadLog => "Load Log",
            Self::Convert(kind) => kind.text(),
            Self::JumpToTag => "Jump to Tag",
            Self::HudHeader => "Hud Header",
            Self::SetHome => "Are you sure?",
            Self::SendMessage => "Enter Message",
            Self::PointCameraAlt => "Enter Alt",
            Self::PointCameraCoords => "Enter Coords",
            Self::PoiSave => "Save File",
            Self::PoiLoad => "Load File",
            Self::SelectScript => "Select Script",
            Self::ViewColumns => "Columns",
            Self::ViewRows => "Rows",
            Self::CellCount => "Battery Cell Count",
            Self::GaugeMax => "Enter Max Speed",
            Self::MjpegUrl => "Mjpeg url",
            Self::GStreamerUrl => mp_video::gstreamer::PIPELINE_TITLE,
            Self::HereLinkIp => mp_video::gstreamer::HERELINK_TITLE,
        }
    }

    /// What the dialog says.
    #[must_use]
    pub fn text(self) -> String {
        match self {
            Self::ConfirmAction(index) => format!(
                "Are you sure you want to do {} ?",
                ACTIONS.get(index).copied().unwrap_or_default()
            ),
            Self::ResumeWarning => {
                "Warning this will reprogram your mission, arm and issue a takeoff command (copter)"
                    .to_owned()
            }
            Self::ResumeAt => "Resume mission at waypoint#".to_owned(),
            Self::FlyToCoords => "Please enter the coords 'lat;long;alt' or 'lat;long'".to_owned(),
            Self::FlyToHereAlt { .. } => "Enter Guided Mode Alt".to_owned(),
            Self::TakeOff => "Enter Takeoff Alt".to_owned(),
            Self::PoiId => crate::poi::ID_TEXT.to_owned(),
            Self::PoiCoords => crate::poi::COORDS_TEXT.to_owned(),
            Self::LoadLog => "Telemetry log (*.tlog)".to_owned(),
            Self::Convert(kind) => kind.filter().to_owned(),
            Self::JumpToTag => "Tag Id:".to_owned(),
            Self::HudHeader => "Please enter your item prefix".to_owned(),
            Self::SetHome => {
                "This will reset the onboard home position (effects RTL etc). Are you Sure?"
                    .to_owned()
            }
            Self::SendMessage => "Enter Message to be logged".to_owned(),
            Self::PointCameraAlt => "Enter Target Alt (Relative to home)".to_owned(),
            Self::PointCameraCoords => {
                "Please enter the coords 'lat;long;alt(abs)' or 'lat;long'".to_owned()
            }
            // `sfd.Filter = "Poi File|*.txt"`.
            Self::PoiSave | Self::PoiLoad => "Poi File".to_owned(),
            Self::ViewColumns => "Enter number of columns to have.".to_owned(),
            Self::ViewRows => "Enter number of rows to have.".to_owned(),
            // `openScriptDialog`, an `OpenFileDialog` for scripts: the path, typed.
            // `// C#: GCSViews/FlightData.cs:1630-1641`
            Self::SelectScript => "Python script (*.py)".to_owned(),
            Self::CellCount => "Cell Count".to_owned(),
            Self::GaugeMax => "Enter Max Speed".to_owned(),
            Self::MjpegUrl => "Enter the url to the mjpeg source url".to_owned(),
            Self::GStreamerUrl => mp_video::gstreamer::PIPELINE_TEXT.to_owned(),
            Self::HereLinkIp => mp_video::gstreamer::HERELINK_TEXT.to_owned(),
        }
    }

    /// Whether it takes typed text, as an `InputBox` does.
    #[must_use]
    pub const fn takes_text(self) -> bool {
        matches!(
            self,
            Self::ResumeAt
                | Self::SelectScript
                | Self::FlyToCoords
                | Self::FlyToHereAlt { .. }
                | Self::TakeOff
                | Self::PoiId
                | Self::PoiCoords
                | Self::LoadLog
                | Self::Convert(_)
                | Self::JumpToTag
                | Self::HudHeader
                | Self::SendMessage
                | Self::PointCameraAlt
                | Self::PointCameraCoords
                | Self::PoiSave
                | Self::PoiLoad
                | Self::ViewColumns
                | Self::ViewRows
                | Self::CellCount
                | Self::GaugeMax
                | Self::MjpegUrl
                | Self::GStreamerUrl
                | Self::HereLinkIp
        )
    }

    /// The accept and decline buttons' words. `MessageShowAgain` has only OK; its Cancel here
    /// stands for closing that dialog's window, which is how it is declined.
    #[must_use]
    pub const fn buttons(self) -> (&'static str, &'static str) {
        match self {
            Self::ConfirmAction(_) => ("Yes", "No"),
            _ => ("OK", "Cancel"),
        }
    }
}

// --- CMB_setwp ------------------------------------------------------------------------------------

/// What `CMB_setwp` lists: "0 (Home)", then 1 up to the largest of the mission-size parameters
/// and the number of mission items held - inclusive, as the C#'s `z <= max` loop is.
/// `// C#: GCSViews/FlightData.cs:2542-2582`
#[must_use]
pub fn setwp_items(parameters: &[(String, f64)], mission_items: usize) -> Vec<String> {
    let mut max: i64 = 0;
    for name in ["CMD_TOTAL", "WP_TOTAL", "MIS_TOTAL"] {
        if let Some((_, value)) = parameters.iter().find(|(held, _)| held == name) {
            // `int.Parse(param.ToString())`: the parameters are whole numbers.
            #[allow(clippy::cast_possible_truncation)]
            let total = *value as i64;
            max = max.max(total);
        }
    }
    if mission_items > 0 {
        max = max.max(i64::try_from(mission_items).unwrap_or(i64::MAX));
    }
    let mut items = vec!["0 (Home)".to_owned()];
    items.extend((1..=max.min(i64::from(u16::MAX))).map(|index| index.to_string()));
    items
}

// --- CMB_action -----------------------------------------------------------------------------------

/// What `CMB_action` lists: the names of `FlightData.actions`, in its order.
/// `// C#: GCSViews/FlightData.cs:183-206, 351`
pub const ACTIONS: [&str; 19] = [
    "Loiter_Unlim",
    "Return_To_Launch",
    "Preflight_Calibration",
    "Mission_Start",
    "Preflight_Reboot_Shutdown",
    "Trigger_Camera",
    "System_Time",
    "Battery_Reset",
    "ADSB_Out_Ident",
    "Scripting_cmd_stop_and_restart",
    "Scripting_cmd_stop",
    "HighLatency_Enable",
    "HighLatency_Disable",
    "Toggle_Safety_Switch",
    "Do_Parachute",
    "Engine_Start",
    "Engine_Stop",
    "Terminate_Flight",
    "Format_SD_Card",
];

/// Whether Do Action asks "Are you sure" first. Five entries are handled before the question
/// and are sent straight away.
/// `// C#: GCSViews/FlightData.cs:1697-1772`
#[must_use]
pub fn needs_confirmation(action: &str) -> bool {
    !matches!(
        action,
        "Format_SD_Card"
            | "Trigger_Camera"
            | "Scripting_cmd_stop_and_restart"
            | "Scripting_cmd_stop"
            | "System_Time"
    )
}

/// The C#'s `Enum.Parse(typeof(MAVLink.MAV_CMD), text)`, for the names Do Action's generic path
/// can reach: `MAV_CMD` values as the C# enum names them, without the `MAV_CMD_` and `NAV_`
/// prefixes. `ADSB_OUT_IDENT` and `DO_START_ADSB_OUT_IDENT` are not in that enum - it has
/// `DO_ADSB_OUT_IDENT` - so neither parses, and the C# reports the command failed.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:827 (enum MAV_CMD)`
#[must_use]
pub fn csharp_mav_cmd(name: &str) -> Option<u16> {
    Some(match name {
        "LOITER_UNLIM" => commands::CMD_NAV_LOITER_UNLIM,
        "RETURN_TO_LAUNCH" => commands::CMD_NAV_RETURN_TO_LAUNCH,
        "PREFLIGHT_CALIBRATION" => commands::CMD_PREFLIGHT_CALIBRATION,
        "MISSION_START" => commands::CMD_MISSION_START,
        "BATTERY_RESET" => commands::CMD_BATTERY_RESET,
        "DO_PARACHUTE" => commands::CMD_DO_PARACHUTE,
        _ => return None,
    })
}

/// What Do Action needs to know about the vehicle.
#[derive(Debug, Clone, Copy)]
pub struct ActionContext {
    /// Where it goes.
    pub target: VehicleId,
    /// `cs.firmware == Firmwares.ArduCopter2`.
    pub copter: bool,
    /// `cs.sensors_enabled.motor_control && cs.sensors_enabled.seen`.
    pub motor_outputs_enabled: bool,
    /// Now, in microseconds since the Unix epoch.
    pub now_unix_usec: u64,
}

/// The messages Do Action sends for one `CMB_action` entry, once any question has been answered.
///
/// Each branch is the C#'s, in its order. `Trigger_Camera`'s fallback - a `DIGICAM_CONTROL`
/// message when the vehicle refuses the command - needs the refusal first, so it travels with
/// the press's [`action_report`] and goes when the refusal arrives.
/// `// C#: GCSViews/FlightData.cs:1680-1878`
pub fn action_messages(action: &str, context: &ActionContext) -> Sends {
    let target = context.target;
    let long = |command_id: u16, p1: f32, p2: f32, p3: f32| {
        commands::command_long(target, command_id, [p1, p2, p3, 0.0, 0.0, 0.0, 0.0])
    };
    let messages = match action {
        // "p1 and p2 must be 1 to initate SD card format" `// C#: FlightData.cs:1697-1710`
        "Format_SD_Card" => vec![commands::command_int(
            target,
            commands::CMD_STORAGE_FORMAT,
            commands::FRAME_GLOBAL,
            [1.0, 1.0, 0.0, 0.0],
            0,
            0,
            0.0,
        )],
        // `setDigicamControl(true)`. `// C#: MAVLinkInterface.cs:4558-4570`
        "Trigger_Camera" => vec![commands::command_long(
            target,
            commands::CMD_DO_DIGICAM_CONTROL,
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        )],
        // `SCRIPTING_CMD.STOP_AND_RESTART` is 3, `STOP` is 2. `// C#: Mavlink.cs:1514-1525`
        "Scripting_cmd_stop_and_restart" | "Scripting_cmd_stop" => {
            let op = if action == "Scripting_cmd_stop" {
                2.0
            } else {
                3.0
            };
            vec![commands::command_int(
                target,
                commands::CMD_SCRIPTING,
                commands::FRAME_GLOBAL,
                [op, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            )]
        }
        "System_Time" => vec![commands::system_time(context.now_unix_usec)],
        "Terminate_Flight" => vec![long(commands::CMD_DO_FLIGHTTERMINATION, 1.0, 0.0, 0.0)],
        // `doReboot()`: `doCommand` sends `PREFLIGHT_REBOOT_SHUTDOWN` twice and does not wait.
        // `// C#: MAVLinkInterface.cs:2553-2563, 2758-2763`
        "Preflight_Reboot_Shutdown" => vec![commands::reboot(target), commands::reboot(target)],
        // `doHighLatency(onoff)`. `// C#: MAVLinkInterface.cs:2524-2532`
        "HighLatency_Enable" => vec![long(commands::CMD_CONTROL_HIGH_LATENCY, 1.0, 0.0, 0.0)],
        "HighLatency_Disable" => vec![long(commands::CMD_CONTROL_HIGH_LATENCY, 0.0, 0.0, 0.0)],
        // `setMode(mode, SAFETY_ARMED)`: `DO_SET_MODE` with base 128, then `SET_MODE` twice.
        // `// C#: FlightData.cs:1819-1830, MAVLinkInterface.cs:4631-4641`
        "Toggle_Safety_Switch" => {
            if target.sysid == 0 {
                return Err(Refusal::quiet("Not toggling safety on sysid 0"));
            }
            let custom_mode = u32::from(context.motor_outputs_enabled);
            let base = commands::MODE_FLAG_SAFETY_ARMED;
            vec![
                commands::do_set_mode(target, base, custom_mode),
                commands::set_mode_with_base(target.sysid, base, custom_mode),
                commands::set_mode_with_base(target.sysid, base, custom_mode),
            ]
        }
        // `doEngineControl(onoff)`. `// C#: MAVLinkInterface.cs:2539-2545`
        "Engine_Start" => vec![long(commands::CMD_DO_ENGINE_CONTROL, 1.0, 0.0, 0.0)],
        "Engine_Stop" => vec![long(commands::CMD_DO_ENGINE_CONTROL, 0.0, 0.0, 0.0)],
        // The generic path: `param1 = 0, param2 = 0, param3 = 1`, adjusted for two entries, and the
        // command found by name. `// C#: FlightData.cs:1781-1869`
        _ => {
            let (mut p1, mut p2, mut p3) = (0.0, 0.0, 1.0);
            if action == "Preflight_Calibration" {
                if context.copter {
                    p1 = 1.0; // gyro
                }
                p3 = 1.0; // baro / airspeed
            }
            if action == "Battery_Reset" {
                p1 = 255.0; // batt 1
                p2 = 100.0; // 100%
                p3 = 0.0;
            }
            let upper = action.to_uppercase();
            let command_id = csharp_mav_cmd(&upper)
                .or_else(|| csharp_mav_cmd(&format!("DO_START_{upper}")))
                .ok_or_else(|| Refusal::error(strings::COMMAND_FAILED))?;
            vec![long(command_id, p1, p2, p3)]
        }
    };
    Ok(messages)
}

/// What Do Action says when the vehicle refuses one `CMB_action` entry or never answers it.
///
/// The generic path shows `Strings.CommandFailed` and the command's name when `doCommand`
/// returns false, and `Strings.CommandFailed` alone from its `catch`. The entries handled before
/// it ignore what `doCommand` returns and have only the `catch`. `Trigger_Camera` is
/// `setDigicamControl`, which sends `DIGICAM_CONTROL` when the command is refused. The rest are
/// not waited for at all - `doReboot`, `setMode`, `sendPacket` - or are `doCommandInt`, which the
/// link has no request for (see [`route`]); they have nothing to say.
/// `// C#: GCSViews/FlightData.cs:1697-1878, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4557-4569`
#[must_use]
pub fn action_report(action: &str, target: VehicleId) -> Report {
    let failed = error_box(strings::COMMAND_FAILED);
    match action {
        "Format_SD_Card"
        | "Scripting_cmd_stop_and_restart"
        | "Scripting_cmd_stop"
        | "System_Time"
        | "Preflight_Reboot_Shutdown"
        | "Toggle_Safety_Switch" => Report::default(),
        "Trigger_Camera" => Report {
            fallback: Some(MavMessage::DigicamControl(DigicamControl {
                extra_value: 0.0,
                target_system: target.sysid,
                target_component: target.compid,
                session: 0,
                zoom_pos: 0,
                zoom_step: 0,
                focus_lock: 0,
                shot: 1,
                command_id: 0,
                extra_param: 0,
            })),
            ..Report::on_timeout(failed)
        },
        "Terminate_Flight"
        | "HighLatency_Enable"
        | "HighLatency_Disable"
        | "Engine_Start"
        | "Engine_Stop" => Report::on_timeout(failed),
        _ => {
            // `cmd`, as `Enum.Parse` found it: the name itself, or with `DO_START_` in front.
            let upper = action.to_uppercase();
            let name = if csharp_mav_cmd(&upper).is_some() {
                upper
            } else {
                format!("DO_START_{upper}")
            };
            Report {
                refused: Some(error_box(format!("{} {name}", strings::COMMAND_FAILED))),
                ..Report::on_timeout(failed)
            }
        }
    }
}

// --- Modes ----------------------------------------------------------------------------------------

/// `setMode(sysid, compid, name)`: the mode found by name, case-insensitively, then
/// `DO_SET_MODE` without waiting and `SET_MODE` twice. Nothing for a name the vehicle does not
/// have, as `translateMode` refuses it.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4614-4641, 6660-6693`
#[must_use]
pub fn set_mode_messages(
    target: VehicleId,
    family: Option<VehicleFamily>,
    name: &str,
) -> Vec<MavMessage> {
    let Some((custom_mode, _)) = family.and_then(|family| {
        family
            .modes()
            .iter()
            .find(|(_, mode)| mode.eq_ignore_ascii_case(name))
    }) else {
        return Vec::new();
    };
    let base = commands::MODE_FLAG_CUSTOM_MODE_ENABLED;
    vec![
        commands::do_set_mode(target, base, *custom_mode),
        commands::set_mode(target, *custom_mode),
        commands::set_mode(target, *custom_mode),
    ]
}

/// TakeOff's answer: `float.Parse(alt, CultureInfo.InvariantCulture)` - its `FormatException`,
/// which the C# leaves to the application's last-chance handler, said here - and the messages of
/// `setMode("GUIDED")` that go before the take-off (none for a vehicle whose family has no
/// Guided, as `translateMode` refuses it).
/// `// C#: GCSViews/FlightData.cs:5299-5305`
pub fn takeoff_plan(
    text: &str,
    target: VehicleId,
    family: Option<VehicleFamily>,
) -> Result<(f32, Vec<MavMessage>), &'static str> {
    let altitude = mp_mission::dotnet::parse_f32(text).ok_or(crate::plan::FORMAT_EXCEPTION)?;
    Ok((altitude, set_mode_messages(target, family, "GUIDED")))
}

/// What `setGuidedModeWP` needs to know about the vehicle.
#[derive(Debug, Clone, Copy)]
pub struct GuidedContext<'a> {
    /// Where it goes.
    pub target: VehicleId,
    /// Which family's mode list applies, and whether it is ArduPlane.
    pub family: Option<VehicleFamily>,
    /// `cs.mode`, the current mode's name.
    pub mode: Option<&'a str>,
}

/// `setGuidedModeWP`: Guided if the vehicle is not already in it, then the target - a
/// `MISSION_ITEM` for ArduPlane, `SET_POSITION_TARGET_GLOBAL_INT` for everything else - and
/// `GuidedMode` updated to match. Nothing at all if the height, latitude or longitude is zero.
///
/// One deliberate difference: the C# copies the whole target into `GuidedMode` for ArduPlane but
/// only x, y and z for everything else, leaving its frame at whatever it was - zero, `GLOBAL`,
/// until Fly To Here Alt sets it. The next Fly To Coords on a copter then reads that frame and
/// sends its height as above sea level rather than above home. The frame is recorded here for
/// every vehicle, as the ArduPlane path records it.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4423-4461, 4538-4545`
pub fn set_guided_mode_wp(
    guided: &mut GuidedMode,
    context: &GuidedContext<'_>,
    (latitude, longitude, altitude): (f64, f64, f32),
    frame: u8,
) -> Vec<MavMessage> {
    if altitude == 0.0 || latitude == 0.0 || longitude == 0.0 {
        return Vec::new();
    }
    let mut messages = Vec::new();
    if !context
        .mode
        .is_some_and(|mode| mode.eq_ignore_ascii_case("GUIDED"))
    {
        messages.extend(set_mode_messages(context.target, context.family, "GUIDED"));
    }
    if context.family == Some(VehicleFamily::Plane) {
        messages.push(commands::guided_mission_item(
            context.target,
            frame,
            latitude,
            longitude,
            altitude,
        ));
    } else {
        messages.push(commands::guided_position_target(
            context.target,
            frame,
            latitude,
            longitude,
            f64::from(altitude),
        ));
    }
    #[allow(clippy::cast_possible_truncation)] // `(int)(lat * 1e7)`
    {
        guided.x = (latitude * 1e7) as i32;
        guided.y = (longitude * 1e7) as i32;
    }
    guided.z = altitude;
    guided.frame = frame;
    messages
}

// --- Fly To Coords / Fly To Here Alt --------------------------------------------------------------

/// What Fly To Coords was given.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Coords {
    /// `lat;long;alt`.
    Full {
        /// Degrees.
        latitude: f64,
        /// Degrees.
        longitude: f64,
        /// In the display's altitude unit.
        altitude: f64,
    },
    /// `lat;long`: the height is `GuidedMode.z`.
    Position {
        /// Degrees.
        latitude: f64,
        /// Degrees.
        longitude: f64,
    },
}

/// Reads Fly To Coords' text: three or two numbers separated by `;`.
///
/// Each is parsed as a `float` - single precision - and widened, as the C#'s `float.Parse` into a
/// `PointLatLngAlt` does; a latitude keeps about half a metre of resolution that way. Any other
/// count of parts is `Strings.InvalidField`. A part that is not a number throws in the C# and
/// reaches no handler at all; here it is the same `InvalidField`.
/// `// C#: GCSViews/FlightData.cs:5938-6008`
pub fn parse_coords(text: &str) -> Result<Coords, Refusal> {
    let parts: Vec<&str> = text.split(';').collect();
    let number = |part: &str| {
        part.trim()
            .parse::<f32>()
            .map(f64::from)
            .map_err(|_| Refusal::error(strings::INVALID_FIELD))
    };
    match parts.as_slice() {
        [lat, lng, alt] => Ok(Coords::Full {
            latitude: number(lat)?,
            longitude: number(lng)?,
            altitude: number(alt)?,
        }),
        [lat, lng] => Ok(Coords::Position {
            latitude: number(lat)?,
            longitude: number(lng)?,
        }),
        _ => Err(Refusal::error(strings::INVALID_FIELD)),
    }
}

/// The frame Fly To Coords sends in: `GuidedMode`'s once anything has been sent in Guided, else
/// the remembered `guided_alt_frame`, else relative.
/// `// C#: GCSViews/FlightData.cs:5943-5951`
#[must_use]
pub fn fly_to_coords_frame(guided: &GuidedMode, remembered: Option<u8>) -> u8 {
    if *guided == GuidedMode::default() {
        remembered.unwrap_or(commands::FRAME_GLOBAL_RELATIVE_ALT)
    } else {
        guided.frame
    }
}

/// The height Fly To Here Alt offers: 10 on a copter and 100 otherwise, in the display unit,
/// unless one was entered before.
/// `// C#: GCSViews/FlightData.cs:2901-2916`
#[must_use]
pub fn fly_to_here_alt_default(copter: bool, remembered: Option<&str>) -> String {
    if let Some(alt) = remembered {
        return alt.to_owned();
    }
    let base = if copter { 10.0 } else { 100.0 };
    format!("{:.0}", base * MULTIPLIER_ALT)
}

// --- Set Home Alt ---------------------------------------------------------------------------------

/// `cs.HomeAlt`: `HomeLocation.Alt`, the vehicle state's home altitude - what `HOME_POSITION`
/// last said, in metres above sea level. Until the vehicle has reported home it is 0, as
/// `_homelocation` starts as an empty `PointLatLngAlt`; with no vehicle it is the placeholder
/// state's, the same 0.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:40, 1568-1582; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5701-5707`
#[must_use]
pub fn home_alt(state: Option<&mp_vehicle::VehicleState>) -> f64 {
    state.map_or(0.0, |state| state.home_altitude.0)
}

/// Set Home Alt: `cs.altoffsethome` goes to zero if it is set, and to minus `cs.HomeAlt`
/// ([`home_alt`]) if it is not - which makes every altitude shown a height above sea level
/// instead of above home. With home not yet reported that is minus zero, which the next click
/// reads as not set, as the C#'s `altoffsethome != 0` does: nothing changes until home is known.
/// `// C#: GCSViews/FlightData.cs:1236-1247`
#[must_use]
pub fn toggle_home_alt(offset: f32, home_altitude: f64) -> f32 {
    if offset == 0.0 {
        #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
        let offset = (-home_altitude / f64::from(MULTIPLIER_ALT)) as f32;
        offset
    } else {
        0.0
    }
}

/// `cs.altoffsethome` of the vehicle shown: the state's field, which Set Home Alt writes through
/// the link and every altitude shown reads; 0 with no vehicle.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:381-383; GCSViews/FlightData.cs:1236-1247`
#[must_use]
pub fn alt_offset_home(view: &TelemetryView) -> f32 {
    view.state
        .as_deref()
        .map_or(0.0, |state| state.alt_offset_home)
}

/// `cs.alt`: `(_alt - altoffsethome) * multiplieralt`.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:325-328`
#[must_use]
pub fn displayed_altitude(relative: f64, offset: f32) -> f64 {
    (relative - f64::from(offset)) * f64::from(MULTIPLIER_ALT)
}

// --- Change Alt / Change Speed -------------------------------------------------------------------

/// Change Alt: `int newalt = (int) modifyandSetAlt.Value;` - the box's decimal place dropped,
/// toward zero - then `setNewWPAlt(new Locationwp {alt = newalt / CurrentState.multiplieralt})`.
/// `// C#: GCSViews/FlightData.cs:4399-4410`
pub fn change_alt_sends(target: VehicleId, box_value: f64) -> Sends {
    #[allow(clippy::cast_possible_truncation)]
    let whole = box_value as i32;
    #[allow(clippy::cast_precision_loss)] // at most 10000
    let altitude = whole as f32 / MULTIPLIER_ALT;
    Ok(vec![commands::change_alt(target, altitude)])
}

/// Change Speed: `(float) modifyandSetSpeed.Value`, not divided by `multiplierspeed`.
/// `// C#: GCSViews/FlightData.cs:4426-4438`
pub fn change_speed_sends(target: VehicleId, box_value: f64) -> Sends {
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
    let speed = box_value as f32;
    Ok(vec![commands::change_speed(target, speed)])
}

// --- Set Loiter Rad -------------------------------------------------------------------------------

/// Set Loiter Rad: `setParam(new[] {"LOITER_RAD", "WP_LOITER_RAD"}, newrad / multiplierdist)`.
///
/// `setParam` with a list writes the first name the vehicle has. A parameter already holding the
/// value is not written again. A vehicle with neither - every copter - gets nothing, and the C#
/// says nothing.
/// `// C#: GCSViews/FlightData.cs:4412-4424, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1609-1650`
pub fn loiter_rad_messages(
    target: VehicleId,
    parameters: &[(String, f64)],
    box_value: f64,
) -> Sends {
    // `int newrad = (int) modifyandSetLoiterRad.Value;` truncates toward zero.
    #[allow(clippy::cast_possible_truncation)]
    let radius = box_value as i32;
    #[allow(clippy::cast_precision_loss)] // a radius of at most 10000
    let value = radius as f32 / MULTIPLIER_DIST;
    for name in ["LOITER_RAD", "WP_LOITER_RAD"] {
        let Some((_, held)) = parameters.iter().find(|(held, _)| held == name) else {
            continue;
        };
        if (*held - f64::from(value)).abs() < f64::EPSILON {
            return Err(Refusal::quiet(format!("{name} is already {value}")));
        }
        return Ok(vec![commands::param_set(target, name, value)]);
    }
    Err(Refusal::quiet(
        "neither LOITER_RAD nor WP_LOITER_RAD is on this vehicle",
    ))
}

// --- Resume Mission -------------------------------------------------------------------------------

/// `MAV_CMD.LAST`: the end of the navigation commands.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:921`
const MAV_CMD_LAST: u16 = 95;

/// `MAV_CMD.DO_LAST`: the end of the "do" commands.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:1087`
const MAV_CMD_DO_LAST: u16 = 240;

/// The mission Resume Mission uploads: every item from the resume point on, and before it only
/// home, takeoffs and the "do" commands - the navigation it skips, it skips, but a camera trigger
/// or speed change on the way still applies. Numbered from zero, `current` 0 and `autocontinue` 1,
/// as `setWP(loc, wpno, frame)` sends them.
/// `// C#: GCSViews/FlightData.cs:1508-1541`
#[must_use]
pub fn resume_items(items: &[MissionItem], resume_at: u16) -> Vec<MissionItem> {
    items
        .iter()
        .enumerate()
        .filter(|&(index, item)| {
            let index = u16::try_from(index).unwrap_or(u16::MAX);
            if index < resume_at && index != 0 {
                if item.command != commands::CMD_NAV_TAKEOFF && item.command < MAV_CMD_LAST {
                    return false;
                }
                if item.command > MAV_CMD_DO_LAST {
                    return false;
                }
            }
            true
        })
        .enumerate()
        .map(|(seq, (_, item))| MissionItem {
            seq: u16::try_from(seq).unwrap_or(u16::MAX),
            current: 0,
            autocontinue: 1,
            ..*item
        })
        .collect()
}

/// Where a Resume Mission has got to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResumePhase {
    /// Reading the vehicle's mission: `getWPCount` and `getWP`.
    Download,
    /// Writing the trimmed one: `setWPTotal`, `setWP`, `setWPACK`.
    Upload,
    /// Reading it back into the planner: `FlightPlanner.instance.BUT_read_Click`.
    Read,
    /// Copter: asking for Guided until the vehicle is in it.
    Guided,
    /// Copter: arming until armed.
    Arm,
    /// Copter: taking off until within 2 m of the resume waypoint's height.
    Takeoff,
    /// Asking for Auto until the vehicle is in it.
    Auto,
    /// Flying the mission from the chosen waypoint.
    Done,
    /// Gave up, with what the C# says.
    Failed(String),
}

/// What a Resume Mission needs done this frame.
#[derive(Debug, Clone, PartialEq)]
pub enum ResumeStep {
    /// Start a mission download.
    DownloadMission,
    /// Start a mission upload.
    UploadMission(Vec<MissionItem>),
    /// Start a download that replaces the plan on the planning screen.
    ReadIntoPlan,
    /// Put these on the wire.
    Send(Vec<MavMessage>),
}

/// What a Resume Mission sees of the vehicle each frame.
#[derive(Debug, Clone, Copy)]
pub struct ResumeInput<'a> {
    /// Now.
    pub now: Instant,
    /// The mission transfer: whether it has finished, whether it failed, and what it says.
    pub transfer: Option<(bool, bool, &'a str)>,
    /// The items the transfer holds.
    pub mission: &'a [MissionItem],
    /// `cs.mode`.
    pub mode: Option<&'a str>,
    /// `cs.armed`.
    pub armed: bool,
    /// `cs.alt`, as displayed.
    pub altitude: f64,
    /// The vehicle's family.
    pub family: Option<VehicleFamily>,
    /// Where commands go.
    pub target: Option<VehicleId>,
    /// The request the last [`ResumeStep::Send`] made, as the link has it: `None` if that step
    /// made none, or the link has forgotten it.
    pub request: Option<RequestState>,
}

/// A call inside Resume Mission that the C# blocks on until the vehicle answers or its retries
/// run out, and so a request the next step waits for.
/// `// C#: GCSViews/FlightData.cs:1553, 1574, 1589-1594`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Blocking {
    /// `setWPCurrent(..., 1)`.
    SetCurrent,
    /// `doARM(true)`.
    Arm,
    /// `doCommand(..., TAKEOFF, ...)`.
    Takeoff,
}

/// How long the C# sleeps between attempts in each of Resume Mission's waiting loops.
/// `// C#: GCSViews/FlightData.cs:1558, 1573, 1592, 1611`
const RESUME_RETRY: Duration = Duration::from_secs(1);

/// How long a finished transfer must be old news before it is taken as this step's.
///
/// The link picks a queued transfer up within a millisecond, so after this the transfer on show is
/// the one just asked for - finished already, if it was quick. Before it, a finished one may be
/// the previous step's.
const TRANSFER_SETTLE: Duration = Duration::from_secs(1);

/// How long to wait for a transfer to appear at all.
const TRANSFER_START: Duration = Duration::from_secs(10);

/// One Resume Mission, run a step a frame instead of blocking the screen as the C# does.
///
/// The C# runs the whole sequence on the UI thread with `Thread.Sleep` and `Application.DoEvents`
/// between attempts. Here each loop is a phase and each attempt is one frame's work, at the C#'s
/// one-second spacing and with its attempt limits: 30 for Guided, arming and Auto, 40 for the
/// climb. `setWPCurrent`, `doARM` and `doCommand` block the C# until the vehicle answers or their
/// retries run out; here they are the link's retrying requests, and the sequence waits for the
/// one it made before it moves on, with the second between attempts counted from when it ended,
/// as the C#'s `Thread.Sleep(1000)` follows the call's return.
/// `// C#: GCSViews/FlightData.cs:1481-1627`
#[derive(Debug, Clone)]
pub struct Resume {
    /// The waypoint to resume at.
    resume_at: u16,
    phase: ResumePhase,
    /// When the phase began.
    since: Instant,
    /// Whether this phase's transfer has been seen in progress.
    saw_active: bool,
    /// Attempts in this phase's loop: the C#'s `timeout`.
    attempts: u32,
    last_attempt: Option<Instant>,
    /// `lastwpdata.alt`.
    takeoff_altitude: f64,
    /// The call the last step made that the C# would still be inside.
    blocked_on: Option<Blocking>,
}

/// What a waiting loop does this frame.
enum Attempt {
    Wait,
    Met,
    Again,
    GiveUp,
}

impl Resume {
    /// Starts at a waypoint: the first thing is reading the vehicle's mission.
    #[must_use]
    pub fn start(resume_at: u16, now: Instant) -> (Self, Vec<ResumeStep>) {
        (
            Self {
                resume_at,
                phase: ResumePhase::Download,
                since: now,
                saw_active: false,
                attempts: 0,
                last_attempt: None,
                takeoff_altitude: 0.0,
                blocked_on: None,
            },
            vec![ResumeStep::DownloadMission],
        )
    }

    /// Where it has got to.
    #[must_use]
    pub const fn phase(&self) -> &ResumePhase {
        &self.phase
    }

    /// Whether there is nothing more to do.
    #[must_use]
    pub const fn finished(&self) -> bool {
        matches!(self.phase, ResumePhase::Done | ResumePhase::Failed(_))
    }

    /// Said on screen and in the facts.
    #[must_use]
    pub fn label(&self) -> String {
        match &self.phase {
            ResumePhase::Download => "reading the mission".to_owned(),
            ResumePhase::Upload => "writing the mission".to_owned(),
            ResumePhase::Read => "reading it back".to_owned(),
            ResumePhase::Guided => "asking for Guided".to_owned(),
            ResumePhase::Arm => "arming".to_owned(),
            ResumePhase::Takeoff => format!("taking off to {:.1} m", self.takeoff_altitude),
            ResumePhase::Auto => "asking for Auto".to_owned(),
            ResumePhase::Done => format!("in Auto from waypoint {}", self.resume_at),
            ResumePhase::Failed(why) => format!("failed: {why}"),
        }
    }

    fn enter(&mut self, phase: ResumePhase, now: Instant) {
        self.phase = phase;
        self.since = now;
        self.saw_active = false;
        self.attempts = 0;
        self.last_attempt = None;
    }

    fn fail(&mut self, why: impl Into<String>) -> Vec<ResumeStep> {
        self.phase = ResumePhase::Failed(why.into());
        Vec::new()
    }

    /// Whether the transfer this phase started has finished, and how.
    fn transfer_done(&mut self, input: &ResumeInput<'_>) -> Option<Result<(), String>> {
        let waited = input.now.saturating_duration_since(self.since);
        match input.transfer {
            Some((false, _, _)) => {
                self.saw_active = true;
                None
            }
            Some((true, failed, label)) if self.saw_active || waited >= TRANSFER_SETTLE => {
                Some(if failed {
                    Err(label.to_owned())
                } else {
                    Ok(())
                })
            }
            None if waited >= TRANSFER_START => Some(Err(strings::ERROR_NO_RESPONSE.to_owned())),
            _ => None,
        }
    }

    /// One turn of a `while (!done) { attempt; Sleep(1000); if (++timeout > limit) fail; }` loop.
    fn attempt(&mut self, now: Instant, met: bool, limit: u32) -> Attempt {
        if let Some(last) = self.last_attempt {
            if now.saturating_duration_since(last) < RESUME_RETRY {
                return Attempt::Wait;
            }
            if self.attempts > limit {
                return Attempt::GiveUp;
            }
        }
        if met {
            return Attempt::Met;
        }
        self.attempts += 1;
        self.last_attempt = Some(now);
        Attempt::Again
    }

    /// Whether the call the last step made has returned, and what its return means.
    ///
    /// `None` to carry on this frame; `Some` to stop here - still inside the call, or failed by
    /// it. A timeout throws in the C#, and the outer `catch` shows `Strings.CommandFailed`; a
    /// refused take-off is `doCommand` returning false, which shows the same. `doARM`'s answer is
    /// not looked at, so a refused arm is asked again a second later.
    /// `// C#: GCSViews/FlightData.cs:1553, 1574, 1589-1594, 1624-1627`
    fn answered(&mut self, input: &ResumeInput<'_>) -> Option<Vec<ResumeStep>> {
        let blocking = self.blocked_on?;
        match input.request {
            Some(RequestState::Queued | RequestState::Waiting) => {
                // The second's sleep starts once the call returns.
                if self.last_attempt.is_some() {
                    self.last_attempt = Some(input.now);
                }
                Some(Vec::new())
            }
            Some(RequestState::Finished(outcome)) => {
                self.blocked_on = None;
                let failed = matches!(
                    (blocking, outcome),
                    (_, RequestOutcome::TimedOut)
                        | (Blocking::Takeoff, RequestOutcome::Rejected(_))
                );
                failed.then(|| self.fail(strings::COMMAND_FAILED))
            }
            // Nothing to wait for: the step made no request, or the link has let it go.
            None => {
                self.blocked_on = None;
                None
            }
        }
    }

    /// Moves on as far as this frame allows.
    pub fn advance(&mut self, input: &ResumeInput<'_>) -> Vec<ResumeStep> {
        let now = input.now;
        if self.finished() {
            return Vec::new();
        }
        let Some(target) = input.target else {
            return self.fail(strings::ERROR_NO_RESPONSE);
        };
        if let Some(steps) = self.answered(input) {
            return steps;
        }
        let mode_is = |name: &str| {
            input
                .mode
                .is_some_and(|mode| mode.eq_ignore_ascii_case(name))
        };
        match self.phase.clone() {
            ResumePhase::Download => match self.transfer_done(input) {
                None => Vec::new(),
                Some(Err(why)) => self.fail(format!("{}: {why}", strings::COMMAND_FAILED)),
                Some(Ok(())) => {
                    // `getWP(lastwpno)` for a waypoint the vehicle does not have times out.
                    let Some(resume_item) = input.mission.get(usize::from(self.resume_at)) else {
                        return self.fail(strings::COMMAND_FAILED);
                    };
                    self.takeoff_altitude = resume_item.z;
                    let items = resume_items(input.mission, self.resume_at);
                    self.enter(ResumePhase::Upload, now);
                    vec![ResumeStep::UploadMission(items)]
                }
            },
            ResumePhase::Upload => match self.transfer_done(input) {
                None => Vec::new(),
                Some(Err(why)) => self.fail(format!("Upload wps failed {why}")),
                Some(Ok(())) => {
                    self.enter(ResumePhase::Read, now);
                    vec![ResumeStep::ReadIntoPlan]
                }
            },
            ResumePhase::Read => match self.transfer_done(input) {
                None => Vec::new(),
                Some(Err(why)) => self.fail(format!("{}: {why}", strings::COMMAND_FAILED)),
                Some(Ok(())) => {
                    // "set index back to 1"
                    let next = if input.family == Some(VehicleFamily::Copter) {
                        ResumePhase::Guided
                    } else {
                        ResumePhase::Auto
                    };
                    self.enter(next, now);
                    self.blocked_on = Some(Blocking::SetCurrent);
                    vec![ResumeStep::Send(vec![commands::mission_set_current(
                        target, 1,
                    )])]
                }
            },
            ResumePhase::Guided => match self.attempt(now, mode_is("guided"), 30) {
                Attempt::Wait => Vec::new(),
                Attempt::GiveUp => self.fail(strings::ERROR_NO_RESPONSE),
                Attempt::Met => {
                    self.enter(ResumePhase::Arm, now);
                    self.advance(input)
                }
                Attempt::Again => vec![ResumeStep::Send(set_mode_messages(
                    target,
                    input.family,
                    "GUIDED",
                ))],
            },
            ResumePhase::Arm => match self.attempt(now, input.armed, 30) {
                Attempt::Wait => Vec::new(),
                Attempt::GiveUp => self.fail(strings::ERROR_NO_RESPONSE),
                Attempt::Met => {
                    self.enter(ResumePhase::Takeoff, now);
                    self.advance(input)
                }
                Attempt::Again => {
                    self.blocked_on = Some(Blocking::Arm);
                    vec![ResumeStep::Send(vec![commands::arm(target, true, false)])]
                }
            },
            ResumePhase::Takeoff => {
                // `if (!doCommand(... TAKEOFF ...)) { Show(CommandFailed); return; }` is
                // `answered`, on the frame the request ends.
                let climbed = input.altitude >= self.takeoff_altitude - 2.0;
                match self.attempt(now, climbed, 40) {
                    Attempt::Wait => Vec::new(),
                    Attempt::GiveUp => self.fail(strings::ERROR_NO_RESPONSE),
                    Attempt::Met => {
                        self.enter(ResumePhase::Auto, now);
                        self.advance(input)
                    }
                    Attempt::Again => {
                        self.blocked_on = Some(Blocking::Takeoff);
                        #[allow(clippy::cast_possible_truncation)] // the C# passes a float
                        let altitude = self.takeoff_altitude as f32;
                        vec![ResumeStep::Send(vec![commands::takeoff(target, altitude)])]
                    }
                }
            }
            ResumePhase::Auto => match self.attempt(now, mode_is("auto"), 30) {
                Attempt::Wait => Vec::new(),
                Attempt::GiveUp => self.fail(strings::ERROR_NO_RESPONSE),
                Attempt::Met => {
                    self.enter(ResumePhase::Done, now);
                    Vec::new()
                }
                Attempt::Again => vec![ResumeStep::Send(set_mode_messages(
                    target,
                    input.family,
                    "AUTO",
                ))],
            },
            ResumePhase::Done | ResumePhase::Failed(_) => Vec::new(),
        }
    }
}

// --- What was sent, and what came back ------------------------------------------------------------

/// A message as a line a person - or a test - can read: its name and the fields that matter.
#[must_use]
pub fn describe(message: &MavMessage) -> String {
    let command_name = |id: u16| {
        MavCmd(u32::from(id))
            .name()
            .map_or_else(|| format!("command {id}"), ToOwned::to_owned)
    };
    let numbers = |values: &[f32]| {
        values
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(",")
    };
    match message {
        MavMessage::CommandLong(m) => format!(
            "COMMAND_LONG {} {}",
            command_name(m.command),
            numbers(&[
                m.param1, m.param2, m.param3, m.param4, m.param5, m.param6, m.param7
            ])
        ),
        MavMessage::CommandInt(m) => format!(
            "COMMAND_INT {} {} x={} y={} z={} frame={}",
            command_name(m.command),
            numbers(&[m.param1, m.param2, m.param3, m.param4]),
            m.x,
            m.y,
            m.z,
            m.frame
        ),
        MavMessage::MissionSetCurrent(m) => format!("MISSION_SET_CURRENT seq={}", m.seq),
        MavMessage::MissionItem(m) => format!(
            "MISSION_ITEM seq={} cmd={} frame={} current={} x={} y={} z={}",
            m.seq, m.command, m.frame, m.current, m.x, m.y, m.z
        ),
        MavMessage::SetPositionTargetGlobalInt(m) => format!(
            "SET_POSITION_TARGET_GLOBAL_INT frame={} mask={:#06X} lat={} lon={} alt={}",
            m.coordinate_frame, m.type_mask, m.lat_int, m.lon_int, m.alt
        ),
        MavMessage::SetMode(m) => format!("SET_MODE base={} custom={}", m.base_mode, m.custom_mode),
        MavMessage::ParamSet(m) => format!(
            "PARAM_SET {}={}",
            mp_params::decode_param_id(&m.param_id),
            m.param_value
        ),
        MavMessage::SetGpsGlobalOrigin(m) => format!(
            "SET_GPS_GLOBAL_ORIGIN lat={} lon={} alt={}",
            m.latitude, m.longitude, m.altitude
        ),
        MavMessage::SystemTime(m) => format!("SYSTEM_TIME time_unix_usec={}", m.time_unix_usec),
        MavMessage::Statustext(m) => format!(
            "STATUSTEXT severity={} text={}",
            m.severity,
            String::from_utf8_lossy(&m.text).trim_end_matches('\0')
        ),
        MavMessage::UavionixAdsbOutControl(m) => format!(
            "UAVIONIX_ADSB_OUT_CONTROL state={} squawk={} flight_id={} baroaltmsl={}",
            m.state,
            m.squawk,
            String::from_utf8_lossy(&m.flight_id).trim_end_matches('\0'),
            m.baroaltmsl
        ),
        other => other.name().to_owned(),
    }
}

/// The command a message carries, if it is one the vehicle acknowledges with `COMMAND_ACK`.
fn acknowledged_command(message: &MavMessage) -> Option<u16> {
    match message {
        MavMessage::CommandLong(m) => Some(m.command),
        MavMessage::CommandInt(m) => Some(m.command),
        _ => None,
    }
}

/// The vehicle's answer to a command, from the message log: the newest `COMMAND_ACK` line for it
/// that arrived after `after`.
#[must_use]
pub fn ack_line(messages: &[LogMessage], command_id: u16, after: u64) -> Option<&LogMessage> {
    let name = MavCmd(u32::from(command_id))
        .name()
        .map_or_else(|| format!("command {command_id}"), ToOwned::to_owned);
    let prefix = format!("{name}:");
    messages
        .iter()
        .rev()
        .take_while(|message| message.seq > after)
        .find(|message| message.text.starts_with(&prefix))
}

// --- The tab's state ------------------------------------------------------------------------------

/// Everything the Actions tab holds between presses.
///
/// Focus handles are not here: they can only be made from a running application, and keeping
/// them out means all of this can be tested without one. See [`ActionsFocus`].
#[derive(Debug)]
pub struct Actions {
    /// `CMB_setwp`'s items.
    pub setwp_items: Vec<String>,
    /// Which is chosen.
    pub setwp_selected: usize,
    /// Whether its list is open.
    pub setwp_open: bool,
    /// Which `CMB_action` entry is chosen.
    pub action_selected: usize,
    /// Whether its list is open.
    pub action_open: bool,
    /// Which of `CMB_mountmode`'s items is chosen: the first, as a bound list starts.
    pub mount_selected: usize,
    /// Whether its list is open.
    pub mount_open: bool,
    /// `modifyandSetSpeed`.
    pub speed: ModifyAndSet,
    /// `modifyandSetAlt`.
    pub alt: ModifyAndSet,
    /// `modifyandSetLoiterRad`.
    pub loiter_rad: ModifyAndSet,
    /// The question being asked, if one is.
    pub prompt: Option<Prompt>,
    /// Its text box.
    pub prompt_field: TextField,
    /// `MAV.GuidedMode`.
    pub guided: GuidedMode,
    /// `Settings.Instance["guided_alt"]` and `["guided_alt_frame"]`. For this session: the
    /// settings file is not this module's to extend.
    pub guided_alt_setting: Option<String>,
    /// See `guided_alt_setting`.
    pub guided_frame_setting: Option<u8>,
    /// `cs.lastautowp`: the last waypoint flown to in Auto, -1 before there is one.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:119, 3422`
    pub last_auto_wp: i32,
    /// A Resume Mission in progress, or the last one.
    pub resume: Option<Resume>,
    /// The request the Resume Mission's last step made, which its next step waits for, and when.
    pub resume_request: Option<(RequestId, Instant)>,
    /// What the last press put on the wire, or why it put nothing.
    pub last_sent: String,
    /// The command last sent that the vehicle answers with `COMMAND_ACK`, and the newest message
    /// when it went.
    awaiting_ack: Option<(u16, u64)>,
}

impl Default for Actions {
    fn default() -> Self {
        Self {
            // `CMB_setwp.Items.AddRange(resources.GetString("CMB_setwp.Items"))` and
            // `SelectedIndex = 0`. `// C#: GCSViews/FlightData.Designer.cs:924-931, FlightData.cs:360`
            setwp_items: vec!["0 (Home)".to_owned()],
            setwp_selected: 0,
            setwp_open: false,
            action_selected: 0,
            action_open: false,
            mount_selected: 0,
            mount_open: false,
            speed: ModifyAndSet::speed(),
            alt: ModifyAndSet::alt(),
            loiter_rad: ModifyAndSet::loiter_rad(),
            prompt: None,
            prompt_field: TextField::new(""),
            guided: GuidedMode::default(),
            guided_alt_setting: None,
            guided_frame_setting: None,
            last_auto_wp: -1,
            resume: None,
            resume_request: None,
            last_sent: "nothing yet".to_owned(),
            awaiting_ack: None,
        }
    }
}

impl Actions {
    /// The action `CMB_action` shows.
    #[must_use]
    pub fn action(&self) -> &'static str {
        ACTIONS
            .get(self.action_selected)
            .copied()
            .unwrap_or(ACTIONS[0])
    }

    /// `CMB_setwp_Click`: the list rebuilt from what is known now.
    ///
    /// The C# clears the list, which leaves `SelectedIndex` at -1 until something is picked, and
    /// Set WP pressed then sends `(ushort)-1` - waypoint 65535. The selection is kept here when it
    /// is still in the list, and is item 0 otherwise.
    pub fn refresh_setwp(&mut self, parameters: &[(String, f64)], mission_items: usize) {
        self.setwp_items = setwp_items(parameters, mission_items);
        if self.setwp_selected >= self.setwp_items.len() {
            self.setwp_selected = 0;
        }
    }

    /// Opens a question, with its box holding `text`.
    pub fn ask(&mut self, prompt: Prompt, text: &str) {
        self.prompt = Some(prompt);
        self.prompt_field.set(text);
    }

    /// What the question's box holds while a question with a box is open, and "none" otherwise:
    /// published, so a test can see its typing arrive before it answers.
    #[must_use]
    pub fn prompt_value(&self) -> &str {
        match self.prompt {
            Some(prompt) if prompt.takes_text() => self.prompt_field.value(),
            _ => "none",
        }
    }

    /// Records what a press sent.
    pub fn record(&mut self, messages: &[MavMessage], newest_message: u64) {
        self.last_sent = messages.iter().map(describe).collect::<Vec<_>>().join("; ");
        if let Some(command_id) = messages.iter().rev().find_map(acknowledged_command) {
            self.awaiting_ack = Some((command_id, newest_message));
        }
    }

    /// Records a press that sent nothing.
    pub fn record_nothing(&mut self, why: &str) {
        self.last_sent = format!("nothing: {why}");
    }

    /// The vehicle's answer to the last command sent, as the message log has it.
    #[must_use]
    pub fn ack(&self, messages: &[LogMessage]) -> String {
        self.awaiting_ack
            .and_then(|(command_id, after)| ack_line(messages, command_id, after))
            .map_or_else(|| "none".to_owned(), |line| line.text.clone())
    }

    /// Publishes what a UI test asserts on.
    pub fn record_facts(&self, view: &TelemetryView) {
        crate::facts::record("fly.sent", &self.last_sent);
        crate::facts::record("fly.ack", self.ack(&view.messages));
        crate::facts::record(
            "fly.prompt",
            self.prompt.map_or_else(|| "none".to_owned(), Prompt::text),
        );
        crate::facts::record("fly.prompt.value", self.prompt_value());
        crate::facts::record("fly.setwp.items", self.setwp_items.len());
        crate::facts::record(
            "fly.setwp.selected",
            self.setwp_items
                .get(self.setwp_selected)
                .map_or("", String::as_str),
        );
        crate::facts::record("fly.action", self.action());
        crate::facts::record(
            "fly.mountmode",
            mount_modes(crate::metadata::lookup)
                .get(self.mount_selected)
                .map_or_else(|| "none".to_owned(), |(key, text)| format!("{key} {text}")),
        );
        crate::facts::record(
            "fly.home_alt",
            if alt_offset_home(view) == 0.0 {
                "off"
            } else {
                "on"
            },
        );
        crate::facts::record("fly.guided.alt", self.guided.z);
        crate::facts::record("fly.guided.frame", self.guided.frame);
        crate::facts::record(
            "fly.resume",
            self.resume
                .as_ref()
                .map_or_else(|| "idle".to_owned(), Resume::label),
        );
        let state = view.state.as_deref();
        crate::facts::record("vehicle.mode", state.and_then(mode_name).unwrap_or("none"));
        crate::facts::record("vehicle.armed", state.is_some_and(|s| s.armed));
        crate::facts::record(
            "vehicle.mission_current",
            state.map_or(0, |s| s.mission_current),
        );
        // `cs.HomeLocation`, which Set Home Here moves: the vehicle's own, as it reports it.
        crate::facts::record(
            "vehicle.home",
            state.and_then(|s| s.home).map_or_else(
                || "none".to_owned(),
                |home| format!("{:.6},{:.6}", home.latitude(), home.longitude()),
            ),
        );
    }
}

/// The name of the mode a vehicle is in: `cs.mode`.
#[must_use]
pub fn mode_name(state: &mp_vehicle::VehicleState) -> Option<&'static str> {
    mp_vehicle::flight_mode_name(state.vehicle_type, state.custom_mode)
}

/// The vehicle's family, from its heartbeat type: `cs.firmware`.
#[must_use]
pub fn family(view: &TelemetryView) -> Option<VehicleFamily> {
    view.state
        .as_ref()
        .and_then(|state| VehicleFamily::from_mav_type(state.vehicle_type))
}

/// The text boxes' focus handles.
#[derive(Debug)]
pub struct ActionsFocus {
    /// `modifyandSetSpeed`'s box.
    pub speed: FocusHandle,
    /// `modifyandSetAlt`'s box.
    pub alt: FocusHandle,
    /// `modifyandSetLoiterRad`'s box.
    pub loiter_rad: FocusHandle,
    /// The question's box, or the dialog itself when the question has no box.
    pub prompt: FocusHandle,
    /// The Transponder page's flight ID and squawk boxes.
    pub xpdr: crate::transponder::Focus,
}

impl ActionsFocus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            speed: cx.focus_handle(),
            alt: cx.focus_handle(),
            loiter_rad: cx.focus_handle(),
            prompt: cx.focus_handle(),
            xpdr: crate::transponder::Focus::new(cx),
        }
    }
}

// --- Drawing the tab ------------------------------------------------------------------------------

/// One cell of `tableLayoutPanel1`, at the column and row the `.resx` gives, counted from zero.
fn cell(column: i16, row: i16, content: impl IntoElement) -> gpui::Div {
    div()
        .col_start(column + 1)
        .row_start(row + 1)
        .min_w(px(0.0))
        .flex()
        .flex_col()
        .gap_1()
        .child(content)
}

/// A button in the grid. The same look as [`crate::ui::action`], filling its cell with the label
/// wrapped, because a fifth of the column is narrower than "Restart Mission" - as a fifth of the
/// C#'s tab is.
fn grid_button(
    id: &'static str,
    label: &'static str,
    colour: u32,
    enabled: bool,
    on_click: impl Fn(&(), &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .w_full()
        .px_1()
        // Two pixels of padding a side, not four: the C#'s Actions tab is 290x175 with 30 px buttons
        // at a 36 px pitch (FlightData.resx), and at four the five wrapped rows plus the map-menu
        // row ran one pixel past the page at 1600x1200 (fly.page.overflow 1).
        .py_0p5()
        .rounded_md()
        .border_1()
        .text_xs()
        .text_center()
        .child(label);
    if enabled {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(colour))
            .text_color(rgb(colour))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(move |_event, window, cx| on_click(&(), window, cx))
            .into_any_element()
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .into_any_element()
    }
}

/// A `ComboBox` stood in for: the chosen entry, which opens its list under the grid when clicked.
/// gpui has no combo box; the plan screen stands its combos in with buttons for the same reason.
fn combo(
    id: &'static str,
    text: &str,
    open: bool,
    on_click: impl Fn(&(), &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    crate::probe::measured(id, div())
        .id(id)
        .flex()
        .items_center()
        .gap_1()
        .w_full()
        .px_1()
        .py_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if open { theme::ACCENT } else { theme::BORDER }))
        .bg(rgb(theme::ACTION))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(
            div()
                .flex_1()
                .min_w(px(0.0))
                .truncate()
                .child(text.to_owned()),
        )
        .child(div().text_color(rgb(theme::DIM)).child("\u{25be}"))
        .on_click(move |_event, window, cx| on_click(&(), window, cx))
        .into_any_element()
}

/// A chip in an open list; clicking it chooses it.
fn list_chip(
    id: String,
    text: &str,
    chosen: bool,
    on_click: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .px_2()
        .py(px(1.0))
        .rounded_sm()
        .border_1()
        .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
        .text_xs()
        .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(text.to_owned())
        .on_click(on_click)
        .into_any_element()
}

/// An entry of the map's menu, drawn as the menu's small type rather than as a command button:
/// the menu is a column of them in the C#, and here a row that wraps. Dimmed, a click says why it
/// does nothing, as the HUD menu's dimmed rows do.
fn menu_entry(
    id: &'static str,
    label: &'static str,
    dimmed: Option<&'static str>,
    cx: &mut Context<MissionPlanner>,
    on_click: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .px_2()
        .py(px(1.0))
        .rounded_sm()
        .border_1()
        .text_xs()
        .child(label);
    match dimmed {
        None => base
            .bg(rgb(theme::ACTION))
            .border_color(rgb(theme::ACCENT))
            .text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                on_click(this, window, cx);
                cx.notify();
            }))
            .into_any_element(),
        Some(why) => base
            .bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.file_status = Some(format!("{label} is not ported: {why}"));
                cx.notify();
            }))
            .into_any_element(),
    }
}

/// Why Camera Overlap is dimmed. `// C#: GCSViews/FlightData.cs:4003-4082, 4457-4471`
const NO_PHOTOS: &str = "it shows or hides the overlap of the camera's photo footprints, and the \
                         CAMERA_FEEDBACK photo markers, photosoverlay, are not drawn here";

/// Which `ModifyandSet` a listener acts on.
type Select = fn(&mut Actions) -> &mut ModifyAndSet;

/// A `ModifyandSet`: its number box over its button, since a fifth of the column cannot hold them
/// side by side as the C#'s flow layout does.
struct ModifyAndSetCell<'a> {
    ids: (&'static str, &'static str),
    label: &'static str,
    control: &'a ModifyAndSet,
    focus: &'a FocusHandle,
    enabled: bool,
    select: Select,
    press: fn(&mut MissionPlanner),
}

impl ModifyAndSetCell<'_> {
    fn render(self, window: &Window, cx: &mut Context<MissionPlanner>) -> gpui::Div {
        let select = self.select;
        let press = self.press;
        div()
            .flex()
            .flex_col()
            .gap_1()
            .child(crate::textfield::text_field(
                self.ids.0,
                &self.control.field,
                self.focus,
                self.focus.is_focused(window),
                px(66.0),
                cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                    let control = select(&mut this.fly_actions);
                    match control.field.key(event) {
                        // Enter validates a `NumericUpDown`; it does not press the button.
                        KeyOutcome::Submitted => {
                            control.commit();
                        }
                        KeyOutcome::Ignored => return,
                        KeyOutcome::Changed | KeyOutcome::Cancelled => {}
                    }
                    cx.notify();
                }),
            ))
            .child(grid_button(
                self.ids.1,
                self.label,
                theme::ACCENT,
                self.enabled,
                cx.listener(move |this, _event: &(), _window, cx| {
                    press(this);
                    cx.notify();
                }),
            ))
    }
}

/// The Actions tab: the grid, the list a combo has open, and the map menu's entries. The question
/// a press asks is not here: it is a dialog over the window, [`prompt_dialog`].
fn actions_tab(
    view: &TelemetryView,
    tab: &Actions,
    focus: &ActionsFocus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let has_vehicle = view.vehicle.is_some();
    let setwp_text = tab
        .setwp_items
        .get(tab.setwp_selected)
        .cloned()
        .unwrap_or_default();
    let mount_options = mount_modes(crate::metadata::lookup);
    let mount_text = mount_options
        .get(tab.mount_selected)
        .map_or("", |(_, text)| text.as_str());

    let grid = div()
        .grid()
        .grid_cols(5)
        .gap_1()
        // Row 0: CMB_action, BUTactiondo, (BUT_quickauto), BUT_Homealt, modifyandSetSpeed.
        .child(cell(
            0,
            0,
            combo(
                "fly-action-list",
                tab.action(),
                tab.action_open,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_actions.action_open = !this.fly_actions.action_open;
                    this.fly_actions.setwp_open = false;
                    this.fly_actions.mount_open = false;
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            1,
            0,
            grid_button(
                "fly-doaction",
                fl!("flightdata-BUTactiondo-Text"),
                theme::WARN,
                has_vehicle,
                cx.listener(|this, _event: &(), window, cx| {
                    this.fly_do_action(window, cx);
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            3,
            0,
            grid_button(
                "fly-homealt",
                fl!("flightdata-BUT_Homealt-Text"),
                if alt_offset_home(view) == 0.0 {
                    theme::TEXT
                } else {
                    theme::OK
                },
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_home_alt();
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            4,
            0,
            ModifyAndSetCell {
                ids: ("fly-speed-value", "fly-changespeed"),
                label: "Change Speed",
                control: &tab.speed,
                focus: &focus.speed,
                enabled: has_vehicle,
                select: |actions| &mut actions.speed,
                press: MissionPlanner::fly_change_speed,
            }
            .render(window, cx),
        ))
        // Row 1: CMB_setwp, BUT_setwp, (BUT_quickmanual), BUTrestartmission, modifyandSetAlt.
        .child(cell(
            0,
            1,
            combo(
                "fly-setwp-list",
                &setwp_text,
                tab.setwp_open,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_setwp_list_click();
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            1,
            1,
            grid_button(
                "fly-setwp",
                fl!("flightdata-BUT_setwp-Text"),
                theme::ACCENT,
                has_vehicle,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_set_wp();
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            3,
            1,
            grid_button(
                "fly-restartmission",
                fl!("flightdata-BUTrestartmission-Text"),
                theme::ACCENT,
                has_vehicle,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_restart_mission();
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            4,
            1,
            ModifyAndSetCell {
                ids: ("fly-alt-value", "fly-changealt"),
                label: "Change Alt",
                control: &tab.alt,
                focus: &focus.alt,
                enabled: has_vehicle,
                select: |actions| &mut actions.alt,
                press: MissionPlanner::fly_change_alt,
            }
            .render(window, cx),
        ))
        // Row 2: (CMB_modes), (BUT_setmode), (BUT_quickrtl), (BUT_RAWSensor),
        // modifyandSetLoiterRad.
        .child(cell(
            4,
            2,
            ModifyAndSetCell {
                ids: ("fly-loiterrad-value", "fly-setloiterrad"),
                label: "Set Loiter Rad",
                control: &tab.loiter_rad,
                focus: &focus.loiter_rad,
                enabled: has_vehicle,
                select: |actions| &mut actions.loiter_rad,
                press: MissionPlanner::fly_set_loiter_rad,
            }
            .render(window, cx),
        ))
        // `BUT_RAWSensor`: `new RAW_Sensor().Show()`, a window of its own, not ported.
        // `// C#: GCSViews/FlightData.cs:1464-1469`
        .child(cell(
            3,
            2,
            grid_button(
                "fly-rawsensor",
                fl!("flightdata-BUT_RAWSensor-Text"),
                theme::ACCENT,
                false,
                |_event: &(), _window, _cx| {},
            ),
        ))
        // Row 3: CMB_mountmode, BUT_mountmode, (BUT_joystick), (BUT_ARM), BUT_clear_track.
        .child(cell(
            0,
            3,
            combo(
                "fly-mountmode-list",
                mount_text,
                tab.mount_open,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_actions.mount_open = !this.fly_actions.mount_open;
                    this.fly_actions.action_open = false;
                    this.fly_actions.setwp_open = false;
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            1,
            3,
            grid_button(
                "fly-mountmode",
                fl!("flightdata-BUT_mountmode-Text"),
                theme::ACCENT,
                has_vehicle,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_set_mount();
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            4,
            3,
            grid_button(
                "fly-cleartrack",
                fl!("flightdata-BUT_clear_track-Text"),
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_clear_track();
                    cx.notify();
                }),
            ),
        ))
        // Row 4: -, -, BUT_SendMSG, BUT_resumemis, BUT_abortland.
        .child(cell(
            2,
            4,
            grid_button(
                "fly-sendmsg",
                fl!("flightdata-BUT_SendMSG-Text"),
                theme::ACCENT,
                has_vehicle,
                cx.listener(|this, _event: &(), window, cx| {
                    this.fly_ask_message(window, cx);
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            3,
            4,
            grid_button(
                "fly-resumemis",
                fl!("flightdata-BUT_resumemis-Text"),
                theme::WARN,
                has_vehicle,
                cx.listener(|this, _event: &(), window, cx| {
                    this.fly_actions.ask(Prompt::ResumeWarning, "");
                    // No box, but OK and closing it are Enter and Escape.
                    this.fly_focus.prompt.focus(window, cx);
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            4,
            4,
            grid_button(
                "fly-abortland",
                fl!("flightdata-BUT_abortland-Text"),
                theme::WARN,
                has_vehicle,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.fly_abort_land();
                    cx.notify();
                }),
            ),
        ));

    let mut body = div().flex().flex_col().gap_2().child(grid);

    if tab.action_open {
        let mut list = div().flex().flex_wrap().gap_1();
        for (index, name) in ACTIONS.iter().enumerate() {
            list = list.child(list_chip(
                format!("fly-action-{name}"),
                name,
                index == tab.action_selected,
                cx.listener(move |this, _event, _window, cx| {
                    this.fly_actions.action_selected = index;
                    this.fly_actions.action_open = false;
                    cx.notify();
                }),
            ));
        }
        body = body.child(list);
    }
    if tab.setwp_open {
        let mut list = div().flex().flex_wrap().gap_1();
        for (index, item) in tab.setwp_items.iter().enumerate() {
            list = list.child(list_chip(
                format!("fly-setwp-{index}"),
                item,
                index == tab.setwp_selected,
                cx.listener(move |this, _event, _window, cx| {
                    this.fly_actions.setwp_selected = index;
                    this.fly_actions.setwp_open = false;
                    cx.notify();
                }),
            ));
        }
        body = body.child(list);
    }
    if tab.mount_open {
        let mut list = div().flex().flex_wrap().gap_1();
        for (index, (key, text)) in mount_options.iter().enumerate() {
            list = list.child(list_chip(
                format!("fly-mountmode-{key}"),
                text,
                index == tab.mount_selected,
                cx.listener(move |this, _event, _window, cx| {
                    this.fly_actions.mount_selected = index;
                    this.fly_actions.mount_open = false;
                    cx.notify();
                }),
            ));
        }
        body = body.child(list);
    }

    // The map's context menu in the C#, `contextMenuStripMap`, in its order. This application's
    // map has no menu - a right click flies there, which is the menu's Fly To Here, and the
    // planner is the FLIGHT PLAN tab and TakeOff a button over the grid - so the rest of its
    // entries are here, under the grid, acting where the map was last pressed as the C#'s act
    // where it was pressed to open the menu. Set Home Here's drop-down is its two entries, and
    // Gimbal Video's its three (`gimbal_video.rs`). The row wraps where the column is too narrow
    // for them all.
    // `// C#: GCSViews/FlightData.Designer.cs:2518-2531, 2612-2630`
    body = body.child(
        div()
            .flex()
            .flex_wrap()
            .items_center()
            .gap_1()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("map menu"),
            )
            .child(menu_entry(
                "fly-flytohere-alt",
                "Fly To Here Alt",
                None,
                cx,
                |this, window, cx| {
                    this.fly_ask_guided_alt();
                    this.fly_focus.prompt.focus(window, cx);
                },
            ))
            .child(menu_entry(
                "fly-flytocoords",
                "Fly To Coords",
                None,
                cx,
                |this, window, cx| {
                    this.fly_actions.ask(Prompt::FlyToCoords, "");
                    this.fly_focus.prompt.focus(window, cx);
                },
            ))
            .child(menu_entry(
                "fly-pointcamerahere",
                "Point Camera Here",
                None,
                cx,
                |this, window, cx| this.fly_ask_point_camera_here(window, cx),
            ))
            .child(menu_entry(
                "fly-pointcameracoords",
                "Point Camera Coords",
                None,
                cx,
                |this, window, cx| {
                    this.fly_actions.ask(Prompt::PointCameraCoords, "");
                    this.fly_focus.prompt.focus(window, cx);
                },
            ))
            .child(menu_entry(
                "fly-triggercamera",
                "Trigger Camera NOW",
                None,
                cx,
                |this, _window, _cx| this.fly_trigger_camera(),
            ))
            .child(menu_entry(
                "fly-setekforigin",
                "Set EKF Origin Here",
                None,
                cx,
                |this, _window, _cx| this.fly_set_ekf_origin(),
            ))
            .child(menu_entry(
                "fly-sethome",
                "Set Home Here",
                None,
                cx,
                |this, window, cx| this.fly_ask_set_home(window, cx),
            ))
            .child(menu_entry(
                "fly-cameraoverlap",
                "Camera Overlap",
                Some(NO_PHOTOS),
                cx,
                |_this, _window, _cx| {},
            ))
            // `jumpToTagToolStripMenuItem`. `// C#: GCSViews/FlightData.Designer.cs:2645-2649`
            .child(menu_entry(
                "fly-jumptotag",
                "Jump To Tag",
                None,
                cx,
                |this, window, cx| {
                    this.fly_actions.ask(Prompt::JumpToTag, "");
                    this.fly_focus.prompt.focus(window, cx);
                },
            ))
            // `gimbalVideoToolStripMenuItem`'s Full Sized, Mini and Pop Out.
            // `// C#: GCSViews/FlightData.Designer.cs:2651-2675`
            .child(menu_entry(
                "fly-gimbalvideo-full",
                "Gimbal Video: Full Sized",
                None,
                cx,
                |this, window, cx| this.gimbal_video_full_sized(window, cx),
            ))
            .child(menu_entry(
                "fly-gimbalvideo-mini",
                "Mini",
                None,
                cx,
                |this, window, cx| this.gimbal_video_mini(window, cx),
            ))
            .child(menu_entry(
                "fly-gimbalvideo-popout",
                "Pop Out",
                None,
                cx,
                |this, window, cx| this.gimbal_video_pop_out(window, cx),
            )),
    );
    // The menu's POI entry and its drop-down: Add Poi at the point the map was last pressed,
    // Delete the one under it, Save File and Load File, and Coords.
    // `// C#: GCSViews/FlightData.Designer.cs:2553-2586`
    body = body.child(
        div()
            .flex()
            .flex_wrap()
            .items_center()
            .gap_1()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child("POI"))
            .child(menu_entry(
                "fly-poi-add",
                "Add Poi",
                None,
                cx,
                |this, window, cx| this.poi_add(window, cx),
            ))
            .child(menu_entry(
                "fly-poi-delete",
                "Delete",
                None,
                cx,
                |this, _window, _cx| this.poi_delete(),
            ))
            .child(menu_entry(
                "fly-poi-save",
                "Save File",
                None,
                cx,
                |this, window, cx| this.poi_ask_file(Prompt::PoiSave, window, cx),
            ))
            .child(menu_entry(
                "fly-poi-load",
                "Load File",
                None,
                cx,
                |this, window, cx| this.poi_ask_file(Prompt::PoiLoad, window, cx),
            ))
            .child(menu_entry(
                "fly-poi-coords",
                "Coords",
                None,
                cx,
                |this, window, cx| {
                    this.fly_actions.ask(Prompt::PoiCoords, "");
                    this.fly_focus.prompt.focus(window, cx);
                },
            )),
    );

    if let Some(resume) = &tab.resume {
        let colour = match resume.phase() {
            ResumePhase::Failed(_) => theme::ALERT,
            ResumePhase::Done => theme::OK,
            _ => theme::ACCENT,
        };
        body = body.child(
            div()
                .text_xs()
                .text_color(rgb(colour))
                .child(format!("Resume Mission: {}", resume.label())),
        );
    }
    body.into_any_element()
}

impl Prompt {
    /// Whether this stands for an `OpenFileDialog` or a `SaveFileDialog`: its answer is a path,
    /// typed, and the box is drawn wider for it.
    #[must_use]
    pub const fn is_file_dialog(self) -> bool {
        matches!(
            self,
            Self::LoadLog | Self::PoiSave | Self::PoiLoad | Self::SelectScript
        )
    }
}

/// What a key does to a question with no box: Enter is the dialog's accept button and Escape its
/// cancel button, as on the C#'s message boxes, and nothing else is anything.
/// `// C#: ExtLibs/Controls/CustomMessageBox.cs:305, 317, ExtLibs/Controls/InputBox.cs:151-152`
#[must_use]
pub fn answer_key(event: &KeyDownEvent) -> KeyOutcome {
    match event.keystroke.key.as_str() {
        "enter" => KeyOutcome::Submitted,
        "escape" => KeyOutcome::Cancelled,
        _ => KeyOutcome::Ignored,
    }
}

/// The question being asked, as the C#'s dialog: centred over the window with everything behind
/// it inert, as `ShowDialog` makes it, with its title, its words, its box if it has one, and its
/// two buttons. Enter answers yes and Escape no, as the dialogs' accept and cancel buttons do.
///
/// A dialog rather than a strip in the Actions page, because a strip grew the page by the height
/// of the question and pushed the mode list off the bottom of the column. The planning screen's
/// dialog is the same shape (`plan.rs`, `prompt_dialog`), and takes keys the same way: through the
/// box when there is one, and through the dialog itself when there is not - a question that has
/// no box still has to hear Enter and Escape.
pub fn prompt_dialog(
    tab: &Actions,
    focus: &ActionsFocus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let prompt = tab.prompt?;
    let (yes, no) = prompt.buttons();
    let size = window.viewport_size();

    let on_key = cx.listener(|this, event: &KeyDownEvent, window, cx| {
        let Some(prompt) = this.fly_actions.prompt else {
            return;
        };
        let outcome = if prompt.takes_text() {
            this.fly_actions.prompt_field.key(event)
        } else {
            answer_key(event)
        };
        match outcome {
            KeyOutcome::Submitted => this.fly_answer(true, window, cx),
            KeyOutcome::Cancelled => this.fly_answer(false, window, cx),
            KeyOutcome::Ignored => return,
            KeyOutcome::Changed => {}
        }
        cx.notify();
    });

    let mut dialog = crate::probe::measured("fly-prompt", div())
        .flex()
        .flex_col()
        .gap_2()
        // A file question is half again as wide, for the path it takes (the owner, 2026-09-25).
        .w(px(if prompt.is_file_dialog() { 510.0 } else { 340.0 }))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::WARN))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(prompt.title()),
        )
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(prompt.text()),
        );
    let mut on_key = Some(on_key);
    if prompt.takes_text()
        && let Some(on_key) = on_key.take()
    {
        dialog = dialog.child(crate::textfield::text_field(
            "fly-prompt-value",
            &tab.prompt_field,
            &focus.prompt,
            focus.prompt.is_focused(window),
            px(310.0),
            on_key,
        ));
    }
    if let Prompt::FlyToHereAlt { frame } = prompt {
        let mut frames = div().flex().gap_1();
        for (name, value) in ALT_FRAMES {
            frames = frames.child(list_chip(
                format!("fly-prompt-frame-{}", name.to_lowercase()),
                name,
                value == frame,
                cx.listener(move |this, _event, _window, cx| {
                    this.fly_actions.prompt = Some(Prompt::FlyToHereAlt { frame: value });
                    cx.notify();
                }),
            ));
        }
        dialog = dialog.child(frames);
    }
    dialog = dialog.child(
        div()
            .flex()
            .justify_end()
            .gap_2()
            .child(action(
                "fly-prompt-ok",
                yes,
                theme::OK,
                true,
                cx.listener(|this, _event: &(), window, cx| {
                    this.fly_answer(true, window, cx);
                    cx.notify();
                }),
            ))
            .child(action(
                "fly-prompt-cancel",
                no,
                theme::TEXT,
                true,
                cx.listener(|this, _event: &(), window, cx| {
                    this.fly_answer(false, window, cx);
                    cx.notify();
                }),
            )),
    );
    let body = match on_key {
        // No box to type in: the dialog itself holds the focus, so Enter and Escape reach it.
        Some(on_key) => dialog
            .id("fly-prompt-keys")
            .track_focus(&focus.prompt)
            .on_key_down(on_key)
            .into_any_element(),
        None => dialog.into_any_element(),
    };

    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("fly-prompt-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(body),
                ),
        )
        // Over the HUD's User Items form, which asks its header through this dialog.
        .with_priority(3)
        .into_any_element(),
    )
}

// -------------------------------------------------------------------------------------------------
// The pages under the HUD.
//
// `tabControlactions`, the `TabControl` that fills `SubMainLeft.Panel2` under `hud1`
// (`GCSViews/FlightData.Designer.cs:321-329`). One page shows at a time, which is what keeps the
// left column inside the window: everything that used to stack under the HUD now sits on the page
// the C# puts it on, and the pages this application has nothing for say which C# control belongs
// there rather than being left out, so the strip is the C#'s strip.
// -------------------------------------------------------------------------------------------------

/// A page of `tabControlactions`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Page {
    /// `tabQuick`.
    Quick,
    /// `tabActions`.
    Actions,
    /// `tabPagemessages`.
    Messages,
    /// `tabActionsSimple`.
    ActionsSimple,
    /// `tabPagePreFlight`.
    PreFlight,
    /// `tabGauges`.
    Gauges,
    /// `tabTransponder`.
    Transponder,
    /// `tabStatus`.
    Status,
    /// `tabServo`.
    Servo,
    /// `tabAuxFunction`.
    AuxFunction,
    /// `tabScripts`.
    Scripts,
    /// `tabPayload`.
    Payload,
    /// `tabTLogs`.
    TLogs,
    /// `tablogbrowse`.
    LogBrowse,
}

impl Page {
    /// Every page, in the order the Designer adds them to the control - which is the order of the
    /// headers. With no `tabcontrolactions` setting saved, `loadTabControlActions` returns before
    /// touching the pages, so a first run shows all fourteen in this order.
    /// `// C#: GCSViews/FlightData.Designer.cs:595-608, GCSViews/FlightData.cs:733-738`
    pub const ALL: [Self; 14] = [
        Self::Quick,
        Self::Actions,
        Self::Messages,
        Self::ActionsSimple,
        Self::PreFlight,
        Self::Gauges,
        Self::Transponder,
        Self::Status,
        Self::Servo,
        Self::AuxFunction,
        Self::Scripts,
        Self::Payload,
        Self::TLogs,
        Self::LogBrowse,
    ];

    /// The page's `Name` in the Designer.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Quick => "tabQuick",
            Self::Actions => "tabActions",
            Self::Messages => "tabPagemessages",
            Self::ActionsSimple => "tabActionsSimple",
            Self::PreFlight => "tabPagePreFlight",
            Self::Gauges => "tabGauges",
            Self::Transponder => "tabTransponder",
            Self::Status => "tabStatus",
            Self::Servo => "tabServo",
            Self::AuxFunction => "tabAuxFunction",
            Self::Scripts => "tabScripts",
            Self::Payload => "tabPayload",
            Self::TLogs => "tabTLogs",
            Self::LogBrowse => "tablogbrowse",
        }
    }

    /// The header's words: the page's `Text` in `FlightData.resx`, in the configured culture
    /// (`crate::i18n`) as `ComponentResourceManager` applies the culture's `.resx`. Both Actions
    /// pages are "Actions" in English, and so both are here.
    /// `// C#: GCSViews/FlightData.resx:580, 1384, 1441, 1555, 1606, 2531, 3014, 3044, 3692, 3890, 4124, 4424, 4925, 5171; MainV2.cs:4214-4243`
    #[must_use]
    pub fn text(self) -> &'static str {
        match self {
            Self::Quick => fl!("flightdata-tabQuick-Text"),
            Self::Actions => fl!("flightdata-tabActions-Text"),
            Self::Messages => fl!("flightdata-tabPagemessages-Text"),
            Self::ActionsSimple => fl!("flightdata-tabActionsSimple-Text"),
            Self::PreFlight => fl!("flightdata-tabPagePreFlight-Text"),
            Self::Gauges => fl!("flightdata-tabGauges-Text"),
            Self::Transponder => fl!("flightdata-tabTransponder-Text"),
            Self::Status => fl!("flightdata-tabStatus-Text"),
            Self::Servo => fl!("flightdata-tabServo-Text"),
            Self::AuxFunction => fl!("flightdata-tabAuxFunction-Text"),
            Self::Scripts => fl!("flightdata-tabScripts-Text"),
            Self::Payload => fl!("flightdata-tabPayload-Text"),
            Self::TLogs => fl!("flightdata-tabTLogs-Text"),
            Self::LogBrowse => fl!("flightdata-tablogbrowse-Text"),
        }
    }

    /// The id a script clicks the header by. From the Designer's name, since the two Actions
    /// headers read the same.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Quick => "fly-tab-quick",
            Self::Actions => "fly-tab-actions",
            Self::Messages => "fly-tab-messages",
            Self::ActionsSimple => "fly-tab-actionssimple",
            Self::PreFlight => "fly-tab-preflight",
            Self::Gauges => "fly-tab-gauges",
            Self::Transponder => "fly-tab-transponder",
            Self::Status => "fly-tab-status",
            Self::Servo => "fly-tab-servo",
            Self::AuxFunction => "fly-tab-auxfunction",
            Self::Scripts => "fly-tab-scripts",
            Self::Payload => "fly-tab-payload",
            Self::TLogs => "fly-tab-tlogs",
            Self::LogBrowse => "fly-tab-logbrowse",
        }
    }

    /// What the C# has on the page that this one does not, said on the page, so a page with
    /// nothing on it reads as not ported rather than as broken. The controls are the ones the
    /// Designer adds to each page.
    #[must_use]
    pub const fn note(self) -> Option<&'static str> {
        Some(match self {
            Self::Quick | Self::Actions | Self::Transponder => return None,
            // `// C#: GCSViews/FlightData.Designer.cs:1086`
            Self::Messages => "txt_messagebox: the messages are under the map on this screen.",
            // `// C#: GCSViews/FlightData.Designer.cs:1098-1140`, each wired to a quick-mode
            // handler.
            Self::ActionsSimple => {
                "myButton1-3 (Loiter, RTL, Auto) are not ported; the mode list on the first \
                 Actions page sets those modes."
            }
            // `// C#: GCSViews/FlightData.Designer.cs:1143, Controls/PreFlight/CheckListControl.cs`
            Self::PreFlight => "checkListControl1, the pre-flight checklist, is not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:1155-1158`
            Self::Gauges => {
                "Galt, Gheading and Gvspeed, the altitude, heading and vertical speed dials, are \
                 not ported."
            }
            // `// C#: GCSViews/FlightData.cs:6045-6072`, painted from `cs.GetItemList`.
            Self::Status => "tabStatus, every CurrentState field by name, is not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:1754, 1762-1789`
            Self::Servo => "servoOptions1-12 and relayOptions1-16 are not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:1962, 1969-1975`
            Self::AuxFunction => "auxOptions1-7 are not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:2016-2022`
            Self::Scripts => {
                "The script console is drawn under the buttons, not in a form of its own; MAV, \
                 MainV2, the screens, Ports and Joystick are not handed to scripts."
            }
            // `// C#: GCSViews/FlightData.Designer.cs:2091, 2098-2103, GCSViews/FlightData.cs:6678-6700`
            Self::Payload => {
                "Video Control, the gimbal's video in a window of its own, is not ported."
            }
            // `// C#: GCSViews/FlightData.Designer.cs:2208`
            Self::TLogs => "Tlog > Kml or Graph is headless-planner kml, on the command line.",
            // The conversions' and Geo Reference Images' file dialogs are boxes.
            // `// C#: GCSViews/FlightData.cs:1084-1089, 1137-1151, 1313-1317; GeoRef/georefimage.cs:87-138`
            Self::LogBrowse => "A log is named by typing it, from the log directory.",
        })
    }
}

/// A panel this screen draws on a page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Panel {
    /// `quickView1-6`: [`crate::quick::page`].
    Quick,
    /// [`actions_panel`]: the arm and command buttons, the Actions grid and the mode list.
    Actions,
    /// [`prearm_panel`].
    PreArm,
    /// [`health_panel`].
    Health,
    /// `tableLayoutPaneltlogs`: [`playback_page`].
    Playback,
    /// `tableLayoutPanel2`: [`dataflash_page`].
    DataFlash,
    /// `tabTransponder`'s controls: [`crate::transponder::page`].
    Transponder,
    /// `tabPayload`'s controls: [`crate::payload::page`].
    Payload,
    /// `tabGauges`' speed dial: [`gauges_page`].
    Gauges,
    /// `tabScripts`: [`crate::scripts_tab::page`].
    Scripts,
}

impl Page {
    /// The panels the page shows, top to bottom.
    ///
    /// - Quick: `quickView1-6`, the page's only control (`// C#: GCSViews/FlightData.Designer.cs:618`).
    /// - Actions: the arm, mode and command controls with the 5x5 grid - `BUT_ARM` and
    ///   `CMB_modes` are cells of `tableLayoutPanel1` on `tabActions`
    ///   (`// C#: GCSViews/FlightData.Designer.cs:739, 757, 767`).
    /// - PreFlight: the pre-arm and health panels. Neither has a page in the C#: the pre-arm and
    ///   EKF and vibration detail are windows the HUD's own indicators open
    ///   (`// C#: ExtLibs/Controls/HUD.cs:1207-1231`). This is the page about being ready to fly,
    ///   and the fix, satellites, link and battery the health panel shows are what
    ///   `checkListControl1` checks by default (`// C#: checklistDefault.xml`).
    ///
    /// - Telemetry Logs: `tableLayoutPaneltlogs`, the playback controls
    ///   (`// C#: GCSViews/FlightData.Designer.cs:2195-2211`).
    /// - DataFlash Logs: `tableLayoutPanel2`, the log tools, whose first opens the Log Downloader
    ///   (`// C#: GCSViews/FlightData.Designer.cs:2374-2389`).
    /// - Transponder, Payload Control and Gauges: their controls where the `.resx` puts them -
    ///   the transponder's, the gimbal's, and the speed dial
    ///   (`// C#: GCSViews/FlightData.Designer.cs:1155-1158, 1609-1626, 2088-2095`).
    ///
    /// The messages stay under the map, where this screen has always had them, and the tuning
    /// graph is above the map, where the C# has it.
    #[must_use]
    pub const fn panels(self) -> &'static [Panel] {
        match self {
            Self::Quick => &[Panel::Quick],
            Self::Actions => &[Panel::Actions],
            Self::PreFlight => &[Panel::PreArm, Panel::Health],
            Self::TLogs => &[Panel::Playback],
            Self::LogBrowse => &[Panel::DataFlash],
            Self::Transponder => &[Panel::Transponder],
            Self::Payload => &[Panel::Payload],
            Self::Gauges => &[Panel::Gauges],
            Self::Scripts => &[Panel::Scripts],
            _ => &[],
        }
    }
}

/// The page shown at start: `tabControlactions.SelectedIndex = 0`, the first page added.
/// `// C#: GCSViews/FlightData.Designer.cs:611`
pub const DEFAULT_PAGE: Page = Page::ALL[0];

/// Which page is showing, which pages the strip has, and which header the strip starts from.
///
/// `Multiline` is off by default (`// C#: GCSViews/FlightData.cs:429`), so the headers are one
/// row and the ones that do not fit are reached with the two arrows a `TabControl` puts at the
/// end of the row, each moving the row along by one header. MultiLine, on the strip's menu, lets
/// the headers wrap onto as many rows as they need instead, with no arrows.
///
/// The pages are all fourteen until Customize chooses others: `loadTabControlActions` returns
/// before touching them while the `tabcontrolactions` setting is empty. The setting and the
/// multi-line choice last for the session; the settings file is not this module's to extend.
#[derive(Debug)]
pub struct Pages {
    selected: Page,
    first_shown: usize,
    /// `tabControlactions.TabPages`, in their order.
    shown: Vec<Page>,
    /// `tabControlactions.Multiline`.
    pub multiline: bool,
    /// `Settings.Instance["tabcontrolactions"]`: page names, each followed by a `;`.
    setting: Option<String>,
}

impl Default for Pages {
    fn default() -> Self {
        Self {
            selected: DEFAULT_PAGE,
            first_shown: 0,
            shown: Page::ALL.to_vec(),
            multiline: false,
            setting: None,
        }
    }
}

impl Pages {
    /// The page showing.
    #[must_use]
    pub const fn selected(&self) -> Page {
        self.selected
    }

    /// Shows a page.
    pub fn select(&mut self, page: Page) {
        self.selected = page;
    }

    /// The pages the strip has, in its order.
    #[must_use]
    pub fn shown(&self) -> &[Page] {
        &self.shown
    }

    /// The index of the first header in the row.
    #[must_use]
    pub const fn first_shown(&self) -> usize {
        self.first_shown
    }

    /// Whether the left arrow has anywhere to go.
    #[must_use]
    pub const fn can_scroll_left(&self) -> bool {
        self.first_shown > 0
    }

    /// Whether the right arrow has anywhere to go: until the last header is the first shown.
    #[must_use]
    pub fn can_scroll_right(&self) -> bool {
        self.first_shown + 1 < self.shown.len()
    }

    /// The left arrow: one header back.
    pub fn scroll_left(&mut self) {
        self.first_shown = self.first_shown.saturating_sub(1);
    }

    /// The right arrow: one header on.
    pub fn scroll_right(&mut self) {
        if self.can_scroll_right() {
            self.first_shown += 1;
        }
    }

    /// MultiLine: `tabControlactions.Multiline` turned over.
    /// `// C#: GCSViews/FlightData.cs:6498-6502`
    pub fn toggle_multiline(&mut self) {
        self.multiline = !self.multiline;
    }

    /// Customize's list: every page `TabListOriginal` holds - all are displayed by the default
    /// display configuration - checked where the setting names it. With no setting yet, the
    /// pages the strip has are saved as it first, as `saveTabControlActions` saves them.
    /// `// C#: GCSViews/FlightData.cs:2584-2615, 4747-4757, ExtLibs/Utilities/DisplayView.cs:146-159`
    pub fn customize_list(&mut self) -> Vec<(Page, bool)> {
        let setting = self
            .setting
            .get_or_insert_with(|| page_names(&self.shown))
            .clone();
        let names: Vec<&str> = setting.split(';').collect();
        Page::ALL
            .iter()
            .map(|page| (*page, names.contains(&page.name())))
            .collect()
    }

    /// Customize closed: the checked pages saved as the setting, then `updateDisplayView` - the
    /// strip rebuilt from the setting in its order, unless the setting is empty, when
    /// `loadTabControlActions` returns before touching the pages. A rebuilt strip shows its first
    /// page, as a `TabControl` cleared and filled again selects the first added.
    /// `// C#: GCSViews/FlightData.cs:2617-2627, 733-791`
    pub fn customize(&mut self, list: &[(Page, bool)]) {
        let checked: Vec<Page> = list
            .iter()
            .filter(|(_, on)| *on)
            .map(|(page, _)| *page)
            .collect();
        let answer = page_names(&checked);
        self.setting = Some(answer.clone());
        if answer.is_empty() {
            return;
        }
        let mut shown = Vec::new();
        for name in answer.split(';') {
            if let Some(page) = Page::ALL.iter().find(|page| page.name() == name) {
                shown.push(*page);
            }
        }
        // `updateDisplayView`: at least one page - Quick. `// C#: GCSViews/FlightData.cs:775-780`
        let first = *shown.first().unwrap_or(&Page::Quick);
        if shown.is_empty() {
            shown.push(first);
        }
        self.selected = first;
        self.shown = shown;
        self.first_shown = 0;
    }

    /// Publishes what a UI test asserts on: the page showing, by the Designer's name and by its
    /// header, the strip's pages in order both ways, where the row starts, whether it wraps, and
    /// how far the page runs past the bottom of the column - `overflow`, the page's scroll
    /// range, which is zero when it fits.
    pub fn record_facts(&self, overflow: f32) {
        crate::facts::record("fly.tab", self.selected.name());
        crate::facts::record("fly.tab.text", self.selected.text());
        crate::facts::record("fly.tabs", page_list(&self.shown, Page::name));
        crate::facts::record("fly.tabs.text", page_list(&self.shown, Page::text));
        crate::facts::record("fly.tabs.first", self.first_shown);
        crate::facts::record("fly.tabs.multiline", self.multiline);
        crate::facts::record("fly.page.overflow", format!("{:.0}", overflow.max(0.0)));
    }
}

/// Pages' names each followed by `;`, as `saveTabControlActions` and Customize write them.
fn page_names(pages: &[Page]) -> String {
    pages
        .iter()
        .map(|page| format!("{};", page.name()))
        .collect()
}

/// Pages' names or texts, joined with commas.
fn page_list(pages: &[Page], of: fn(Page) -> &'static str) -> String {
    pages
        .iter()
        .map(|page| of(*page))
        .collect::<Vec<_>>()
        .join(",")
}

/// The header row: one header per page from the first shown on, clipped at the column's edge,
/// and the two arrows at the end of the row - or, MultiLine, every header on as many rows as
/// they take, and no arrows. The right button opens the strip's menu, Customize and MultiLine,
/// `tabControlactions.ContextMenuStrip`. `// C#: GCSViews/FlightData.Designer.cs:327, 594`
///
/// A multi-line `TabControl` also moves the row holding the selected header next to the page;
/// the rows stay in the pages' order here.
pub fn page_strip(pages: &Pages, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    let mut row = div()
        .flex()
        .flex_1()
        .min_w(px(0.0))
        .overflow_hidden()
        .gap_1()
        .when(pages.multiline, gpui::Styled::flex_wrap);
    let skip = if pages.multiline {
        0
    } else {
        pages.first_shown()
    };
    for page in pages.shown().iter().copied().skip(skip) {
        let selected = page == pages.selected();
        row = row.child(
            crate::probe::measured(page.id(), div())
                .id(page.id())
                .flex_shrink_0()
                .px_2()
                .py_1()
                .rounded_t_md()
                .text_xs()
                .cursor_pointer()
                .bg(rgb(if selected { theme::PANEL } else { theme::BG }))
                .text_color(rgb(if selected { theme::ACCENT } else { theme::DIM }))
                .border_b_2()
                .border_color(rgb(if selected { theme::ACCENT } else { theme::BG }))
                .hover(|style| style.text_color(rgb(theme::TEXT)))
                .child(page.text())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.fly_pages.select(page);
                    // Each page starts at its top, not wherever the last one was scrolled to.
                    this.fly_scroll.set_offset(gpui::point(px(0.0), px(0.0)));
                    cx.notify();
                })),
        );
    }

    crate::probe::measured("fly-tabs", div())
        .id("fly-tabs")
        .flex()
        .flex_shrink_0()
        .items_end()
        .gap_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .on_mouse_up(
            gpui::MouseButton::Right,
            cx.listener(|this, event: &gpui::MouseUpEvent, _window, cx| {
                this.fly_data.menu = Some((
                    MenuKind::Tabs,
                    (f32::from(event.position.x), f32::from(event.position.y)),
                ));
                cx.notify();
            }),
        )
        .child(row)
        .children((!pages.multiline).then(|| {
            div()
                .flex()
                .flex_shrink_0()
                .gap_1()
                .pb_1()
                .child(strip_arrow(
                    "fly-tabs-left",
                    "\u{25c2}",
                    pages.can_scroll_left(),
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.fly_pages.scroll_left();
                        cx.notify();
                    }),
                ))
                .child(strip_arrow(
                    "fly-tabs-right",
                    "\u{25b8}",
                    pages.can_scroll_right(),
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.fly_pages.scroll_right();
                        cx.notify();
                    }),
                ))
        }))
}

/// One of the strip's two arrows.
fn strip_arrow(
    id: &'static str,
    label: &'static str,
    enabled: bool,
    on_click: impl Fn(&(), &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .px_1()
        .rounded_sm()
        .border_1()
        .text_xs()
        .child(label);
    if enabled {
        base.border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(move |_event, window, cx| on_click(&(), window, cx))
            .into_any_element()
    } else {
        base.border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .into_any_element()
    }
}

/// What a page is drawn from.
pub struct PageInputs<'a> {
    /// The vehicle, as the panels read it.
    pub view: &'a TelemetryView,
    /// The Quick, Telemetry Logs and DataFlash Logs pages' state.
    pub data: &'a FlightData,
    /// The Actions page's state.
    pub actions: &'a Actions,
    /// Its text boxes' focus.
    pub focus: &'a ActionsFocus,
    /// Whether this session has turned the arming checks off.
    pub checks_disabled: bool,
}

/// The page showing: its panels ([`Page::panels`]), then what the C# has on it that this does
/// not ([`Page::note`]).
pub fn page_content(
    page: Page,
    inputs: &PageInputs<'_>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let view = inputs.view;
    let mut content: Vec<AnyElement> = page
        .panels()
        .iter()
        .map(|panel| match panel {
            Panel::Quick => {
                // `cs.alt` is above home, or above sea level while Set Home Alt is on.
                // `// C#: ExtLibs/ArduPilot/CurrentState.cs:325-328`
                let shown = view.state.as_deref().map(|state| {
                    let mut shown = *state;
                    shown.altitude_relative = mp_units::Metres(displayed_altitude(
                        state.altitude_relative.0,
                        state.alt_offset_home,
                    ));
                    shown
                });
                // Each view's `ContextMenuStrip` is `contextMenuStripQuickView`: the right button
                // opens it over the view. `// C#: GCSViews/FlightData.Designer.cs:636-725`
                div()
                    .id("fly-quick-views")
                    .flex()
                    .flex_col()
                    .flex_1()
                    .min_h(px(180.0))
                    .child(crate::quick::page(&inputs.data.quick, shown.as_ref(), cx))
                    .on_mouse_up(
                        gpui::MouseButton::Right,
                        cx.listener(|this, event: &gpui::MouseUpEvent, _window, cx| {
                            this.fly_data.menu = Some((
                                MenuKind::Quick,
                                (f32::from(event.position.x), f32::from(event.position.y)),
                            ));
                            cx.stop_propagation();
                            cx.notify();
                        }),
                    )
                    .into_any_element()
            }
            Panel::Transponder => {
                crate::transponder::page(&inputs.data.transponder, &inputs.focus.xpdr, window, cx)
            }
            Panel::Payload => crate::payload::page(&inputs.data.payload, view.state.as_deref(), cx),
            Panel::Scripts => crate::scripts_tab::page(&inputs.data.scripts, cx),
            Panel::Gauges => gauges_page(inputs.data, view.state.as_deref(), cx),
            Panel::Playback => playback_page(&inputs.data.playback, cx),
            Panel::DataFlash => dataflash_page(inputs.data, cx),
            Panel::Actions => actions_panel(
                view,
                inputs.checks_disabled,
                inputs.actions,
                inputs.focus,
                window,
                cx,
            )
            .into_any_element(),
            Panel::PreArm => prearm_panel(view),
            Panel::Health => health_panel(view).into_any_element(),
        })
        .collect();
    if let Some(note) = page.note() {
        content.push(
            crate::probe::measured("fly-page-note", div())
                .px_1()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(note)
                .into_any_element(),
        );
    }
    content
}

// --- The handlers ---------------------------------------------------------------------------------

impl MissionPlanner {
    /// Puts a press's messages on the wire to the vehicle being flown and records them - or
    /// records why nothing went.
    ///
    /// Each message goes the way [`route`] says: the ones the C# blocks on become the link's
    /// retrying requests, with `report` said on the status line when each ends, and the rest go
    /// once. Returns the requests made, in order.
    fn fly_send(&mut self, sends: Sends, view: &TelemetryView, report: &Report) -> Vec<RequestId> {
        self.fly_send_by(sends, view, Some(report))
    }

    /// The same, every message sent once and none waited for: `doCommand` with `requireack`
    /// false, as `setMountControl` and `setMountConfigure` send.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2717-2723`
    pub(crate) fn fly_send_once(&mut self, sends: Sends) {
        let view = self.telemetry.view();
        let sends = match self.telemetry.send_handle() {
            Some(_) => sends,
            None => Err(Refusal::quiet("no vehicle")),
        };
        self.fly_send_by(sends, &view, None);
    }

    /// [`MissionPlanner::fly_send`], each message routed where `report` is given and sent once
    /// where it is not.
    fn fly_send_by(
        &mut self,
        sends: Sends,
        view: &TelemetryView,
        report: Option<&Report>,
    ) -> Vec<RequestId> {
        let messages = match sends {
            Ok(messages) if !messages.is_empty() => messages,
            Ok(_) => {
                self.fly_actions.record_nothing("nothing to send");
                return Vec::new();
            }
            Err(refusal) => {
                self.fly_actions.record_nothing(refusal.text());
                self.file_status = Some(match refusal {
                    Refusal::Error(text) => error_box(text),
                    Refusal::Quiet(text) => text,
                });
                return Vec::new();
            }
        };
        let Some((sender, _)) = self.telemetry.send_handle() else {
            self.fly_actions.record_nothing("no vehicle");
            self.file_status = Some("no vehicle to send to".to_owned());
            return Vec::new();
        };
        let (requests, queued) = match report {
            Some(report) => send_routed(
                &mut self.telemetry,
                &sender,
                &messages,
                report,
                view.connected,
            ),
            None => {
                let mut queued = true;
                for message in &messages {
                    queued &= sender.send(message);
                }
                (Vec::new(), queued)
            }
        };
        self.fly_actions
            .record(&messages, view.messages.last().map_or(0, |m| m.seq));
        self.file_status = Some(if queued {
            format!("sent {}", self.fly_actions.last_sent)
        } else {
            "the link has closed; nothing was sent".to_owned()
        });
        requests
    }

    /// Builds a press's messages for the vehicle being flown and sends them, with `report` said
    /// when a request among them ends.
    pub(crate) fn fly_press(
        &mut self,
        report: &Report,
        build: impl FnOnce(&mut Actions, VehicleId, &TelemetryView) -> Sends,
    ) -> Vec<RequestId> {
        let view = self.telemetry.view();
        let sends = match self.telemetry.send_handle() {
            Some((_, target)) => build(&mut self.fly_actions, target, &view),
            None => Err(Refusal::quiet("no vehicle")),
        };
        self.fly_send(sends, &view, report)
    }

    /// `CMB_setwp_Click`: the list rebuilt, and opened.
    /// `// C#: GCSViews/FlightData.cs:2542-2582`
    fn fly_setwp_list_click(&mut self) {
        let view = self.telemetry.view();
        self.fly_actions
            .refresh_setwp(&view.parameters, view.mission.len());
        self.fly_actions.setwp_open = !self.fly_actions.setwp_open;
        self.fly_actions.action_open = false;
        self.fly_actions.mount_open = false;
    }

    /// Set WP: `setWPCurrent(sysid, compid, (ushort) CMB_setwp.SelectedIndex)`, and
    /// `Strings.CommandFailed` from its `catch` when every retry goes unanswered.
    /// `// C#: GCSViews/FlightData.cs:1658-1672`
    fn fly_set_wp(&mut self) {
        let seq = u16::try_from(self.fly_actions.setwp_selected).unwrap_or(u16::MAX);
        let report = Report::on_timeout(error_box(strings::COMMAND_FAILED));
        self.fly_press(&report, |_, target, _| {
            Ok(vec![commands::mission_set_current(target, seq)])
        });
    }

    /// Restart Mission: `setWPCurrent(sysid, compid, 0)`, with the same `catch`.
    /// `// C#: GCSViews/FlightData.cs:1881-1895`
    fn fly_restart_mission(&mut self) {
        let report = Report::on_timeout(error_box(strings::COMMAND_FAILED));
        self.fly_press(&report, |_, target, _| {
            Ok(vec![commands::mission_set_current(target, 0)])
        });
    }

    /// Change Alt: `setNewWPAlt(new Locationwp {alt = (int) Value / multiplieralt})`.
    ///
    /// Sent once: `setNewWPAlt` is `setWP`, which waits for `MISSION_ACK`, and the link has no
    /// single-item request to do that with (see [`route`]).
    /// `// C#: GCSViews/FlightData.cs:4399-4410`
    fn fly_change_alt(&mut self) {
        self.fly_press(&Report::default(), |actions, target, _| {
            change_alt_sends(target, actions.alt.commit())
        });
    }

    /// Change Speed: `DO_CHANGE_SPEED` with the box's number, undivided; its answer not looked
    /// at, and `Strings.ErrorCommunicating` from its `catch`.
    /// `// C#: GCSViews/FlightData.cs:4426-4438`
    fn fly_change_speed(&mut self) {
        let report = Report::on_timeout(error_box(strings::ERROR_COMMUNICATING));
        self.fly_press(&report, |actions, target, _| {
            change_speed_sends(target, actions.speed.commit())
        });
    }

    /// Set Loiter Rad: the first of `LOITER_RAD` and `WP_LOITER_RAD` the vehicle has, and
    /// `Strings.ErrorCommunicating` from the `catch`.
    /// `// C#: GCSViews/FlightData.cs:4412-4424`
    fn fly_set_loiter_rad(&mut self) {
        let report = Report::on_timeout(error_box(strings::ERROR_COMMUNICATING));
        self.fly_press(&report, |actions, target, view| {
            let value = actions.loiter_rad.commit();
            loiter_rad_messages(target, &view.parameters, value)
        });
    }

    /// Abort Landing: `doAbortLand`, only with the link open; its answer not looked at, and
    /// `Strings.CommandFailed` from its `catch`.
    /// `// C#: GCSViews/FlightData.cs:1019-1032`
    fn fly_abort_land(&mut self) {
        let report = Report::on_timeout(error_box(strings::COMMAND_FAILED));
        self.fly_press(&report, |_, target, view| {
            if view.connected {
                Ok(vec![commands::go_around(target)])
            } else {
                Err(Refusal::quiet("the link is not open"))
            }
        });
    }

    /// Set Home Alt: altitudes shown above sea level, or back to above home.
    /// `// C#: GCSViews/FlightData.cs:1236-1247`
    fn fly_home_alt(&mut self) {
        let view = self.telemetry.view();
        // C#: GCSViews/FlightData.cs:1245, `MainV2.comPort.MAV.cs.HomeAlt`.
        let home = home_alt(view.state.as_deref());
        // `MainV2.comPort.MAV.cs.altoffsethome`: the vehicle state's own field, which the link
        // sets on its next pass.
        let offset = toggle_home_alt(alt_offset_home(&view), home);
        self.telemetry.set_alt_offset_home(offset);
        self.file_status = Some(if offset == 0.0 {
            "altitudes are above home".to_owned()
        } else {
            format!("altitudes are above sea level (home is {home:.1} m)")
        });
    }

    /// Do Action: straight away for the five entries the C# handles before asking, and after
    /// "Are you sure" for the rest.
    /// `// C#: GCSViews/FlightData.cs:1680-1878`
    fn fly_do_action(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.fly_actions.action_open = false;
        let index = self.fly_actions.action_selected;
        if needs_confirmation(self.fly_actions.action()) {
            self.fly_actions.ask(Prompt::ConfirmAction(index), "");
            // Nothing to type, but Enter and Escape should still answer it.
            self.fly_focus.prompt.focus(window, cx);
        } else {
            self.fly_run_action(index);
        }
    }

    /// Sends one `CMB_action` entry, with its [`action_report`].
    fn fly_run_action(&mut self, index: usize) {
        let action = ACTIONS.get(index).copied().unwrap_or(ACTIONS[0]);
        let report = self
            .telemetry
            .send_handle()
            .map(|(_, target)| action_report(action, target))
            .unwrap_or_default();
        self.fly_press(&report, |_, target, view| {
            let state = view.state.as_deref();
            let context = ActionContext {
                target,
                copter: family(view) == Some(VehicleFamily::Copter),
                // `MAV_SYS_STATUS_SENSOR.MOTOR_OUTPUTS` is bit 15. Read here from the dialect's
                // own constant: `Sensors::motor_outputs_enabled` in mp-vehicle tests bit 14.
                motor_outputs_enabled: state.is_some_and(|state| {
                    state.sensors.reported
                        && state.sensors.enabled
                            & mp_mavlink_dialects::all::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS.0
                            != 0
                }),
                now_unix_usec: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |since| u64::try_from(since.as_micros()).unwrap_or(u64::MAX)),
            };
            action_messages(action, &context)
        });
    }

    /// Fly To Here Alt: the box opened with the height and frame it last had.
    /// `// C#: GCSViews/FlightData.cs:2899-2918`
    fn fly_ask_guided_alt(&mut self) {
        let copter = family(&self.telemetry.view()) == Some(VehicleFamily::Copter);
        let alt = fly_to_here_alt_default(copter, self.fly_actions.guided_alt_setting.as_deref());
        let frame = self
            .fly_actions
            .guided_frame_setting
            .unwrap_or(commands::FRAME_GLOBAL_RELATIVE_ALT);
        self.fly_actions.ask(Prompt::FlyToHereAlt { frame }, &alt);
    }

    /// The answer to the question being asked.
    fn fly_answer(&mut self, accepted: bool, window: &mut Window, cx: &mut Context<Self>) {
        let Some(prompt) = self.fly_actions.prompt.take() else {
            return;
        };
        let text = self.fly_actions.prompt_field.value().to_owned();
        match prompt {
            Prompt::ConfirmAction(index) => {
                if accepted {
                    self.fly_run_action(index);
                }
            }
            Prompt::ResumeWarning => {
                // Everything after the warning is inside `if (BaseStream.IsOpen)`.
                if accepted && self.telemetry.view().connected {
                    let last = self.fly_actions.last_auto_wp;
                    let text = if last == -1 {
                        "1".to_owned()
                    } else {
                        last.to_string()
                    };
                    self.fly_actions.ask(Prompt::ResumeAt, &text);
                    self.fly_focus.prompt.focus(window, cx);
                }
            }
            Prompt::ResumeAt => {
                if accepted {
                    self.fly_resume_at(&text);
                }
            }
            // The C# ignores whether the box was cancelled: the text stays empty, and an empty
            // text is `Strings.InvalidField`.
            Prompt::FlyToCoords => {
                self.fly_to_coords(if accepted { &text } else { "" });
            }
            Prompt::FlyToHereAlt { frame } => {
                if accepted {
                    self.fly_to_here_alt(&text, frame);
                }
            }
            Prompt::TakeOff => {
                if accepted {
                    self.fly_takeoff(&text);
                }
            }
            Prompt::PoiId => {
                let pending = self.fly_data.pois.pending.take();
                if accepted && let Some((lat, lng, alt)) = pending {
                    self.fly_data.pois.add(lat, lng, alt, &text);
                }
            }
            // Like Fly To Coords, the answer is read whether or not the box was cancelled.
            Prompt::PoiCoords => self.poi_at_coords(if accepted { &text } else { "" }, window, cx),
            // `openScriptDialog`: OK selects, Cancel clears. `// C#: GCSViews/FlightData.cs:1630-1641`
            Prompt::SelectScript => {
                self.fly_data
                    .scripts
                    .select(if accepted { &text } else { "" });
            }
            Prompt::LoadLog => {
                if accepted {
                    self.fly_load_log(&text);
                }
            }
            Prompt::Convert(kind) => {
                if accepted {
                    self.fly_convert(kind, &text);
                }
            }
            Prompt::JumpToTag => {
                if accepted {
                    self.fly_jump_to_tag(&text, window, cx);
                }
            }
            Prompt::HudHeader => {
                let pending = self.fly_data.hud_settings.pending.take();
                // Cancel leaves the box unchecked. `// C#: GCSViews/FlightData.cs:2451-2455`
                if accepted && let Some(name) = pending {
                    self.fly_data.hud_settings.add_item(&name, &text);
                }
            }
            Prompt::SetHome => {
                if let Some(point) = self.fly_data.pending_home.take() {
                    self.fly_press(&Report::default(), |_, target, _| {
                        Ok(set_home_messages(target, point, accepted))
                    });
                }
            }
            Prompt::SendMessage => {
                // `if (DialogResult.Cancel == ...) return;`, then `send_text(5, txt)`.
                if accepted {
                    self.fly_press(&Report::default(), |_, _, _| {
                        Ok(vec![statustext(MESSAGE_SEVERITY, &text)])
                    });
                }
            }
            Prompt::PointCameraAlt => {
                if accepted {
                    let point = mouse_down_point(&self.fly_data);
                    self.fly_press(&Report::default(), |_, target, _| {
                        point_camera_here_sends(target, point, &text)
                    });
                }
            }
            // The C# does not look at the dialog's answer: a cancelled box is empty, which is
            // neither two parts nor three.
            Prompt::PointCameraCoords => {
                let text = if accepted { text } else { String::new() };
                self.fly_press(&Report::default(), |_, target, _| {
                    point_camera_coords_sends(target, &text, |lat, lng| {
                        crate::srtm::altitude(lat, lng).alt
                    })
                });
            }
            Prompt::PoiSave => {
                if accepted {
                    self.poi_save(&text);
                }
            }
            Prompt::PoiLoad => {
                if accepted {
                    self.poi_load(&text);
                }
            }
            Prompt::ViewColumns => {
                if accepted {
                    let rows = self
                        .fly_data
                        .quick
                        .saved_grid()
                        .map_or_else(|| "3".to_owned(), |(_, rows)| rows.to_string());
                    self.fly_data.quick_cols = Some(text);
                    self.fly_actions.ask(Prompt::ViewRows, &rows);
                    self.fly_focus.prompt.focus(window, cx);
                }
            }
            Prompt::ViewRows => {
                let cols = self.fly_data.quick_cols.take();
                if accepted && let Some(cols) = cols {
                    self.fly_view_count(&cols, &text);
                }
            }
            Prompt::CellCount => {
                if accepted {
                    match dotnet_int(&text) {
                        Some(count) => self.fly_data.hud_settings.cells = Some(count),
                        None => self.file_status = Some(error_box(BAD_RADIUS)),
                    }
                }
            }
            Prompt::GaugeMax => {
                if accepted && let Err(why) = self.fly_data.speed_gauge.set_max(&text) {
                    self.file_status = Some(error_box(why));
                }
            }
            Prompt::MjpegUrl => {
                self.fly_mjpeg_source(accepted, &text);
                self.video_repaint_keep(cx);
            }
            Prompt::GStreamerUrl => {
                self.fly_gstreamer_source(accepted, &text);
                self.video_repaint_keep(cx);
            }
            Prompt::HereLinkIp => {
                self.fly_herelink_video(accepted, &text);
                self.video_repaint_keep(cx);
            }
        }
    }

    /// Resume Mission, once the waypoint is given: `int.Parse`, then the sequence.
    /// `// C#: GCSViews/FlightData.cs:1497-1621`
    fn fly_resume_at(&mut self, text: &str) {
        let view = self.telemetry.view();
        let Some(resume_at) = text
            .trim()
            .parse::<i32>()
            .ok()
            .and_then(|number| u16::try_from(number).ok())
        else {
            self.fly_send(
                Err(Refusal::error(strings::COMMAND_FAILED)),
                &view,
                &Report::default(),
            );
            return;
        };
        let (resume, steps) = Resume::start(resume_at, Instant::now());
        self.fly_actions.resume = Some(resume);
        self.fly_actions.resume_request = None;
        self.fly_resume_steps(steps, &view);
    }

    /// Does what a Resume Mission asked for this frame.
    ///
    /// A send's request, if it made one, is what the sequence waits for next. The sequence says
    /// its own failures, so the requests say nothing.
    fn fly_resume_steps(&mut self, steps: Vec<ResumeStep>, view: &TelemetryView) {
        for step in steps {
            match step {
                ResumeStep::DownloadMission => self.telemetry.request_mission(),
                ResumeStep::UploadMission(items) => self.telemetry.upload_mission(items),
                ResumeStep::ReadIntoPlan => {
                    self.adopt_vehicle_mission = true;
                    self.telemetry.request_mission();
                }
                ResumeStep::Send(messages) => {
                    let requests = self.fly_send(Ok(messages), view, &Report::default());
                    self.fly_actions.resume_request =
                        requests.last().map(|&id| (id, Instant::now()));
                }
            }
        }
    }

    /// Fly To Coords: `lat;long;alt` or `lat;long`, flown to in Guided. Everything it sends goes
    /// once: `setMode` and `setGuidedModeWP` do not wait (see [`route`]).
    /// `// C#: GCSViews/FlightData.cs:5938-6008`
    fn fly_to_coords(&mut self, text: &str) {
        let coords = parse_coords(text);
        self.fly_press(&Report::default(), |actions, target, view| {
            let frame = fly_to_coords_frame(&actions.guided, actions.guided_frame_setting);
            let (latitude, longitude, altitude) = match coords? {
                Coords::Full {
                    latitude,
                    longitude,
                    altitude,
                } => {
                    // `(float) plla.Alt / CurrentState.multiplieralt`
                    #[allow(clippy::cast_possible_truncation)]
                    let altitude = altitude as f32 / MULTIPLIER_ALT;
                    (latitude, longitude, altitude)
                }
                // The C# looks up the terrain height here and then flies at `GuidedMode.z`
                // regardless; the lookup changes nothing, so it is not made.
                Coords::Position {
                    latitude,
                    longitude,
                } => (latitude, longitude, actions.guided.z),
            };
            guided_sends(
                actions,
                target,
                view,
                (latitude, longitude, altitude),
                frame,
            )
        });
    }

    /// TakeOff, once answered: the height saved as `takeoff_alt`, the vehicle put in Guided, and
    /// `doCommand(TAKEOFF, 0, 0, 0, 0, 0, 0, alt)` - its answer not looked at, a timeout
    /// "Command Failed". One press, in the C#'s order: `setMode`'s messages go straight onto the
    /// wire and the take-off is the link's request, sent on its next pass, so the vehicle hears
    /// Guided first. ArduCopter takes off only in Guided, which is why the C# sets it.
    /// `// C#: GCSViews/FlightData.cs:5290-5315`
    fn fly_takeoff(&mut self, text: &str) {
        let view = self.telemetry.view();
        let Some(target) = view.vehicle else {
            return;
        };
        let sends = takeoff_plan(text, target, family(&view)).map(|(altitude, mut sends)| {
            self.persisted
                .set("takeoff_alt", mp_mission::dotnet::general_f32(altitude));
            sends.push(commands::takeoff(target, altitude));
            sends
        });
        let report = Report::on_timeout(error_box(strings::COMMAND_FAILED));
        self.fly_send(sends.map_err(Refusal::error), &view, &report);
    }

    /// Fly To Here Alt, once answered: the height and frame the next Fly To Here uses, and - if
    /// the vehicle is already in Guided - the current target moved to that height.
    /// `// C#: GCSViews/FlightData.cs:2920-2943`
    fn fly_to_here_alt(&mut self, text: &str, frame: u8) {
        self.fly_actions.guided_alt_setting = Some(text.to_owned());
        self.fly_actions.guided_frame_setting = Some(frame);
        let Ok(whole) = text.trim().parse::<i32>() else {
            let view = self.telemetry.view();
            self.fly_send(
                Err(Refusal::error(strings::BAD_ALT)),
                &view,
                &Report::default(),
            );
            return;
        };
        #[allow(clippy::cast_precision_loss)] // a height in metres
        let altitude = whole as f32 / MULTIPLIER_ALT;
        self.fly_actions.guided.z = altitude;
        self.fly_actions.guided.frame = frame;
        let view = self.telemetry.view();
        let in_guided = view.state.as_deref().and_then(mode_name) == Some("Guided");
        if !in_guided {
            self.file_status = Some(format!(
                "Fly To Here will fly at {altitude} m (frame {frame})"
            ));
            return;
        }
        self.fly_press(&Report::default(), |actions, target, view| {
            let guided = actions.guided;
            guided_sends(
                actions,
                target,
                view,
                (
                    f64::from(guided.x) / 1e7,
                    f64::from(guided.y) / 1e7,
                    guided.z,
                ),
                frame,
            )
        });
    }

    /// Fly To Here once Fly To Here Alt has set a height: the C#'s `goHereToolStripMenuItem_Click`,
    /// which flies to the clicked point at `GuidedMode.z` in `GuidedMode.frame`.
    /// `// C#: GCSViews/FlightData.cs:3082-3120`
    pub(crate) fn fly_to_here_guided(&mut self, position: mp_units::LatLon) {
        self.fly_press(&Report::default(), |actions, target, view| {
            if position.latitude() == 0.0 || position.longitude() == 0.0 {
                return Err(Refusal::error(strings::BAD_COORDS));
            }
            let (altitude, frame) = (actions.guided.z, actions.guided.frame);
            guided_sends(
                actions,
                target,
                view,
                (position.latitude(), position.longitude(), altitude),
                frame,
            )
        });
    }

    /// Once a frame: `cs.lastautowp`, a Resume Mission moved on, the Telemetry Logs page
    /// following the log it plays, the Transponder page's wait and look for a status, and the
    /// DataFlash Logs page's conversion finishing.
    pub(crate) fn fly_tick(&mut self, view: &TelemetryView, window: &Window) {
        self.fly_data.playback.tick();
        // The Scripts tab: the run's output and end, and the requests its script has made.
        if let Some(status) =
            self.fly_data
                .scripts
                .tick(&self.telemetry, view, &mut self.fly_actions.guided)
        {
            self.file_status = Some(status);
        }
        self.fly_xpdr_tick(view, window);
        // A conversion that has finished says so on the status line.
        if let Some(outcome) = self.fly_data.conversions.poll() {
            self.file_status = Some(conversion_status(&outcome));
        }
        if let Some(state) = view.state.as_deref()
            && mode_name(state).is_some_and(|mode| mode.eq_ignore_ascii_case("auto"))
            && state.mission_current != 0
        {
            self.fly_actions.last_auto_wp = i32::from(state.mission_current);
        }
        let Some(mut resume) = self.fly_actions.resume.take() else {
            return;
        };
        if resume.finished() {
            self.fly_actions.resume = Some(resume);
            return;
        }
        let state = view.state.as_deref();
        let steps = resume.advance(&ResumeInput {
            now: Instant::now(),
            transfer: view
                .transfer
                .as_ref()
                .map(|status| (status.finished, status.failed, status.label.as_str())),
            mission: &view.mission,
            mode: state.and_then(mode_name),
            armed: state.is_some_and(|state| state.armed),
            altitude: state.map_or(0.0, |state| {
                displayed_altitude(state.altitude_relative.0, state.alt_offset_home)
            }),
            family: family(view),
            target: self.telemetry.send_handle().map(|(_, id)| id),
            request: self.fly_actions.resume_request.and_then(|(id, made)| {
                match self.telemetry.lookup(id, made) {
                    Lookup::Found(request) => Some(request.state()),
                    Lookup::PickingUp => Some(RequestState::Queued),
                    Lookup::Gone => None,
                }
            }),
        });
        // Said once, on the frame it ends; the strip under the grid keeps saying it after that.
        match resume.phase() {
            ResumePhase::Failed(why) => self.file_status = Some(error_box(why)),
            ResumePhase::Done => {
                self.file_status = Some(format!("Resume Mission: {}", resume.label()))
            }
            _ => {}
        }
        self.fly_actions.resume = Some(resume);
        self.fly_resume_steps(steps, view);
    }
}

/// A press's messages on their way, each as [`route`] says: the requests made, in order, and
/// whether everything was queued - false once the link has closed.
///
/// Every message is offered to the link, even after one is refused: a loop rather than `all`,
/// which would stop at the first refusal.
pub fn send_routed(
    telemetry: &mut crate::telemetry::Telemetry,
    sender: &mp_link::LinkSender,
    messages: &[MavMessage],
    report: &Report,
    connected: bool,
) -> (Vec<RequestId>, bool) {
    let mut queued = true;
    let mut requests = Vec::new();
    for message in messages {
        let request = match route(message) {
            Route::Raw => {
                queued &= sender.send(message);
                continue;
            }
            Route::SetCurrent { target, seq } => {
                telemetry.set_current_waypoint(target, seq, report.clone())
            }
            Route::Command {
                target,
                command,
                params,
            } => telemetry.command(target, command, params, report.clone()),
            // `setParam(name, value)`: `force` is false.
            Route::SetParam {
                target,
                name,
                value,
            } => telemetry.set_parameter_on(target, &name, value, false, report.clone()),
            Route::CommandInt {
                target,
                command,
                frame,
                params,
                x,
                y,
                z,
            } => telemetry.command_int(target, command, frame, params, x, y, z, report.clone()),
            Route::SetWp { target, item } => telemetry.set_wp(target, *item, report.clone()),
            Route::GetHome { target } => telemetry.get_home_position(target, report.clone()),
        };
        // A request is queued for the link thread, which is only there while the link is.
        queued &= request.is_some() && connected;
        requests.extend(request);
    }
    (requests, queued)
}

/// `setGuidedModeWP` for the vehicle being flown, as a press's sends.
fn guided_sends(
    actions: &mut Actions,
    target: VehicleId,
    view: &TelemetryView,
    point: (f64, f64, f32),
    frame: u8,
) -> Sends {
    let context = GuidedContext {
        target,
        family: family(view),
        mode: view.state.as_deref().and_then(mode_name),
    };
    let messages = set_guided_mode_wp(&mut actions.guided, &context, point, frame);
    if messages.is_empty() {
        // `if (gotohere.alt == 0 || gotohere.lat == 0 || gotohere.lng == 0) return;`
        Err(Refusal::quiet(
            "no guided height, latitude or longitude yet; nothing sent",
        ))
    } else {
        Ok(messages)
    }
}

// -------------------------------------------------------------------------------------------------
// The rest of the screen: the Quick page's views, the Telemetry Logs page's playback, the points
// of interest, the Log Downloader the DataFlash Logs page opens, and the windows a click on the
// HUD opens.
// -------------------------------------------------------------------------------------------------

/// What the flight screen keeps besides the Actions tab and the page strip.
#[derive(Debug)]
pub struct FlightData {
    /// `quickView1-6`.
    pub quick: crate::quick::QuickViews,
    /// The Telemetry Logs page.
    pub playback: Playback,
    /// The points of interest.
    pub pois: crate::poi::Pois,
    /// The Log Downloader.
    pub logs: crate::logdownload::LogDownloader,
    /// Whether the `EKFStatus` window is showing.
    pub ekf_open: bool,
    /// Whether the `Vibration` window is showing.
    pub vibration_open: bool,
    /// `MouseDownStart`: where the flight map was last pressed, and where that was in the window.
    /// `// C#: GCSViews/FlightData.cs:2956-2959`
    pub mouse_down_start: Option<(mp_units::LatLon, (f32, f32))>,
    /// Where the HUD was laid out, for its click zones.
    pub hud_bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
    /// `hud1.bgimage`: the camera's latest frame, which the HUD draws under everything, set
    /// once a frame from the Planner page's capture (`MissionPlanner::video_tick`).
    /// `// C#: GCSViews/FlightData.cs:1897-1900`
    pub camera: Option<std::sync::Arc<gpui::RenderImage>>,
    /// The DataFlash Logs page's conversions.
    pub conversions: Conversions,
    /// The HUD's menu.
    pub hud_menu: HudMenu,
    /// What the HUD's menu has set.
    pub hud_settings: HudSettings,
    /// Whether Swap With Map has put the HUD where the map was: the C#'s `HudSwap`.
    /// `// C#: GCSViews/FlightData.cs:5139-5159`
    pub swapped: bool,
    /// The strip's or the quick views' context menu, where the right button came up.
    pub menu: Option<(MenuKind, (f32, f32))>,
    /// Customize's list while its form is open: each page, and whether it is checked.
    pub customizing: Option<Vec<(Page, bool)>>,
    /// Set View Count's columns, between its two questions.
    pub quick_cols: Option<String>,
    /// Set Home Here's point and terrain height, while its question is asked.
    pub pending_home: Option<(f64, f64, f64)>,
    /// The Transponder page.
    pub transponder: crate::transponder::Transponder,
    /// The Payload Control page's gimbal bars.
    pub payload: crate::payload::Payload,
    /// The Scripts page.
    pub scripts: crate::scripts_tab::ScriptsTab,
    /// How many points the last Clear Track took off the map.
    pub track_cleared: Option<usize>,
    /// The Gauges page's speed dial, `Gspeed`.
    pub speed_gauge: crate::gauge::SpeedGauge,
    /// Where the Gauges page was laid out, which places the dial as `tabPage1_Resize` does.
    pub gauges_bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
    /// `FlightData.hudGStreamer`: the pipeline Set GStreamer Source or HereLink Video plays
    /// into the HUD. `// C#: GCSViews/FlightData.cs:46`
    pub gstreamer: mp_video::gstreamer::GStreamer,
    /// `CaptureMJPEG`, while Set MJPEG source has it reading.
    /// `// C#: ExtLibs/Utilities/CaptureMJPEG.cs:13-51`
    pub mjpeg: Option<mp_video::mjpeg::CaptureMjpeg>,
    /// The map menu's Gimbal Video: `gimbalVideoControl` and where it and the map are.
    /// `// C#: GCSViews/FlightData.cs:6534-6712`
    pub gimbal_video: crate::gimbal_video::GimbalVideo,
    /// `GStreamerUI.DownloadGStreamer`, while the runtime is being fetched.
    pub gst_download: Option<GstDownload>,
}

/// `GStreamerUI.DownloadGStreamer`'s progress dialog, on the status line: the download's thread,
/// what it says, and the pipeline to start once the runtime is there.
/// `// C#: Utilities/GStreamerUI.cs:8-23`
#[derive(Debug)]
pub struct GstDownload {
    /// `UpdateProgressAndStatus`'s words.
    said: std::sync::mpsc::Receiver<(i32, String)>,
    /// The download.
    thread: std::thread::JoinHandle<()>,
    /// What the menu entry starts once the runtime is found.
    then: String,
}

/// `inflate` for the runtime's zip: its deflated entries through `flate2`.
fn inflate(data: &[u8], size: usize) -> Result<Vec<u8>, String> {
    use std::io::Read as _;
    let mut out = Vec::with_capacity(size);
    flate2::read::DeflateDecoder::new(data)
        .read_to_end(&mut out)
        .map_err(|why| why.to_string())?;
    Ok(out)
}

/// `GStreamer.LookForGstreamer()`, over this process's `PATH`, its program directory, the data
/// directory and, on Windows, the fixed drives.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1417-1485`
fn look_for_gstreamer() -> Option<std::path::PathBuf> {
    let path = std::env::var_os("PATH");
    let program = std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(std::path::Path::to_path_buf));
    let data = mp_settings::data_directory();
    let (on_path, dirs) = mp_video::gstreamer::search_dirs(
        path.as_ref(),
        program.as_deref(),
        data.as_deref(),
        &mp_video::gstreamer::drives(),
    );
    mp_video::gstreamer::look_for_gstreamer(&on_path, &dirs)
}

// --- The Video drop-down's sources: MJPEG, GStreamer, HereLink ------------------------------------

/// Where the GStreamer launcher is looked for: [`look_for_gstreamer`], or a test's own.
pub type LookFor<'a> = &'a dyn Fn() -> Option<std::path::PathBuf>;

/// Set MJPEG source, answered: on OK the URL saved as `mjpeg_url` and the capture started over
/// on it; on Cancel only stopped. The C#'s `CaptureMJPEG.Stop()` then `runAsync()`. Returns the
/// status line's words, if any.
/// `// C#: GCSViews/FlightData.cs:4898-4911`
pub fn mjpeg_source(
    data: &mut FlightData,
    persisted: &mut crate::settings::Persisted,
    accepted: bool,
    url: &str,
) -> Option<String> {
    data.mjpeg = None;
    if !accepted {
        return None;
    }
    persisted.set("mjpeg_url", url);
    match mp_video::mjpeg::CaptureMjpeg::start(url) {
        Ok(capture) => {
            data.mjpeg = Some(capture);
            None
        }
        Err(why) => Some(error_box(why)),
    }
}

/// Set GStreamer Source, answered: on OK the pipeline saved as `gstreamer_url` and played
/// ([`gstreamer_play`]); on Cancel `hudGStreamer.Stop()`.
/// `// C#: GCSViews/FlightData.cs:4819-4849`
pub fn gstreamer_source(
    data: &mut FlightData,
    persisted: &mut crate::settings::Persisted,
    accepted: bool,
    pipeline: &str,
    look: LookFor<'_>,
) -> Option<String> {
    if accepted {
        persisted.set("gstreamer_url", pipeline);
        gstreamer_play(data, persisted, pipeline, look)
    } else {
        data.gstreamer.stop();
        None
    }
}

/// HereLink Video, answered. The C# does not look at the answer, so Cancel plays the address
/// the box started with. The address saved as `herelinkip`, then the air unit's RTSP stream
/// played ([`gstreamer_play`]).
/// `// C#: GCSViews/FlightData.cs:3155-3183`
pub fn herelink_video(
    data: &mut FlightData,
    persisted: &mut crate::settings::Persisted,
    accepted: bool,
    typed: &str,
    look: LookFor<'_>,
) -> Option<String> {
    let ip = if accepted {
        typed.to_owned()
    } else {
        herelink_ip(persisted)
    };
    persisted.set("herelinkip", ip.as_str());
    gstreamer_play(
        data,
        persisted,
        &mp_video::gstreamer::herelink_pipeline(&ip),
        look,
    )
}

/// `herelinkip`, or the C#'s first address. `// C#: GCSViews/FlightData.cs:3157-3160`
#[must_use]
pub fn herelink_ip(persisted: &crate::settings::Persisted) -> String {
    persisted
        .get("herelinkip")
        .unwrap_or(mp_video::gstreamer::HERELINK_IP)
        .to_owned()
}

/// `GStreamer.GstLaunch = GStreamer.LookForGstreamer()`, saved as `gstlaunchexe`; then with no
/// runtime `GStreamerUI.DownloadGStreamer()` and the pipeline started once it is there
/// ([`gst_download_tick`]), else `hudGStreamer.Start(url)`. A refusal is said on the status
/// line, where the C# shows a message box (the owner's rule: no dialog for what the window can
/// show).
/// `// C#: GCSViews/FlightData.cs:3170-3182, 4825-4844`
fn gstreamer_play(
    data: &mut FlightData,
    persisted: &mut crate::settings::Persisted,
    pipeline: &str,
    look: LookFor<'_>,
) -> Option<String> {
    let gst_launch = look()
        .map(|path| path.display().to_string())
        .unwrap_or_default();
    persisted.set("gstlaunchexe", gst_launch.as_str());
    if mp_video::gstreamer::gst_launch_exists(&gst_launch) {
        gstreamer_start(data, &gst_launch, pipeline)
    } else if mp_video::gstreamer::can_download() {
        gstreamer_download(data, pipeline)
    } else {
        // No runtime, and none to fetch for this machine: what the C#'s `Start` says when the
        // runtime's library will not load.
        Some(
            mp_video::VideoError::NotFound {
                launch: mp_video::gstreamer::LAUNCHER.to_owned(),
                why: NO_RUNTIME.to_owned(),
            }
            .to_string(),
        )
    }
}

/// Why there is no runtime where none can be downloaded.
const NO_RUNTIME: &str = "it is not on the PATH; install the GStreamer runtime";

/// `hudGStreamer.Start(url)`, its refusal for the status line.
/// `// C#: GCSViews/FlightData.cs:4837-4844`
fn gstreamer_start(data: &mut FlightData, gst_launch: &str, pipeline: &str) -> Option<String> {
    data.gstreamer
        .start(std::path::Path::new(gst_launch), pipeline)
        .err()
        .map(|why| match why {
            not_found @ mp_video::VideoError::NotFound { .. } => not_found.to_string(),
            other => error_box(other),
        })
}

/// `GStreamerUI.DownloadGStreamer()`: the runtime fetched into the data directory on a thread of
/// its own, its progress on the status line, and `pipeline` started once it is found.
/// `// C#: Utilities/GStreamerUI.cs:8-23; ExtLibs/Utilities/GStreamer.cs:1497-1547`
fn gstreamer_download(data: &mut FlightData, pipeline: &str) -> Option<String> {
    if data.gst_download.is_some() {
        return None;
    }
    let Some(directory) = mp_settings::data_directory() else {
        return Some(error_box("no home directory to keep GStreamer in"));
    };
    let (tell, said) = std::sync::mpsc::channel();
    let spawned = std::thread::Builder::new()
        .name("gstreamer-download".to_owned())
        .spawn(move || {
            let _ = std::fs::create_dir_all(&directory);
            let mut status = |percent: i32, words: &str| {
                let _ = tell.send((percent, words.to_owned()));
            };
            mp_video::gstreamer::download_gstreamer(
                &directory,
                cfg!(target_pointer_width = "64"),
                &mut |url, to, status| {
                    mp_firmware::flow::get_file_from_net(
                        &mp_firmware::manifest::Http,
                        url,
                        to,
                        status,
                    )
                },
                &inflate,
                &mut status,
            );
        });
    match spawned {
        Ok(thread) => {
            data.gst_download = Some(GstDownload {
                said,
                thread,
                then: pipeline.to_owned(),
            });
            None
        }
        Err(why) => Some(error_box(why)),
    }
}

/// Once a frame: the runtime's download said on the status line, and once it ends the runtime
/// looked for again and the pipeline started if it is there - or nothing, as the C# returns
/// when `GstLaunchExists` is still false.
/// `// C#: Utilities/GStreamerUI.cs:22; GCSViews/FlightData.cs:3172-3182, 4827-4835`
pub fn gst_download_tick(
    data: &mut FlightData,
    persisted: &mut crate::settings::Persisted,
    look: LookFor<'_>,
) -> Option<String> {
    let download = data.gst_download.as_ref()?;
    let mut said = None;
    while let Ok((_, words)) = download.said.try_recv() {
        said = Some(words);
    }
    if !download.thread.is_finished() {
        return said;
    }
    let download = data.gst_download.take()?;
    let _ = download.thread.join();
    let found = look()
        .map(|path| path.display().to_string())
        .unwrap_or_default();
    persisted.set("gstlaunchexe", found.as_str());
    if mp_video::gstreamer::gst_launch_exists(&found) {
        return gstreamer_start(data, &found, &download.then).or(said);
    }
    said
}

impl FlightData {
    /// The screen at start, with the points of interest the C# keeps read from its file.
    #[must_use]
    pub fn new() -> Self {
        Self::with_pois(crate::poi::Pois::load_default())
    }

    /// The same, with the points of interest given.
    #[must_use]
    pub fn with_pois(pois: crate::poi::Pois) -> Self {
        Self {
            quick: crate::quick::QuickViews::default(),
            playback: Playback::default(),
            pois,
            logs: crate::logdownload::LogDownloader::default(),
            ekf_open: false,
            vibration_open: false,
            mouse_down_start: None,
            hud_bounds: Rc::new(Cell::new(None)),
            camera: None,
            conversions: Conversions::default(),
            hud_menu: HudMenu::default(),
            hud_settings: HudSettings::default(),
            swapped: false,
            menu: None,
            customizing: None,
            quick_cols: None,
            pending_home: None,
            transponder: crate::transponder::Transponder::default(),
            payload: crate::payload::Payload::default(),
            scripts: crate::scripts_tab::ScriptsTab::default(),
            track_cleared: None,
            speed_gauge: crate::gauge::SpeedGauge::default(),
            gauges_bounds: Rc::new(Cell::new(None)),
            gstreamer: mp_video::gstreamer::GStreamer::default(),
            mjpeg: None,
            gimbal_video: crate::gimbal_video::GimbalVideo::default(),
            gst_download: None,
        }
    }

    /// The latest frame of the HUD menu's sources: GStreamer's, else MJPEG's. The C#'s sources
    /// all set `hud1.bgimage`, the last frame to arrive showing; two at once flicker between
    /// their pictures there, and here one is shown, in that order before the Planner page's
    /// camera. `// C#: MainV2.cs:3421-3486`
    #[must_use]
    pub fn hud_video_latest(&self) -> Option<std::sync::Arc<mp_video::Frame>> {
        self.gstreamer.latest().or_else(|| {
            self.mjpeg
                .as_ref()
                .and_then(mp_video::mjpeg::CaptureMjpeg::latest)
        })
    }

    /// Whether a HUD menu source is playing or the runtime is being fetched: the window is
    /// drawn at the camera's rate while it is.
    #[must_use]
    pub fn hud_video_running(&self) -> bool {
        self.gstreamer.is_running() || self.mjpeg.is_some() || self.gst_download.is_some()
    }

    /// Publishes what a UI test asserts on. `alt_offset_home` is Set Home Alt's, which the
    /// Quick page's altitude includes.
    pub fn record_facts(&self, view: &TelemetryView, listed: usize, alt_offset_home: f32) {
        let shown = view.state.as_deref().map(|state| {
            let mut shown = *state;
            shown.altitude_relative = mp_units::Metres(displayed_altitude(
                state.altitude_relative.0,
                alt_offset_home,
            ));
            shown
        });
        self.quick.record_facts(shown.as_ref());
        self.playback.record_facts();
        self.pois.record_facts();
        self.logs.record_facts(listed);
        crate::facts::record("fly.ekf.open", self.ekf_open);
        crate::facts::record("fly.vibration.open", self.vibration_open);
        let state = view.state.as_deref();
        crate::facts::record(
            "fly.ekf.values",
            state.map_or_else(
                || "none".to_owned(),
                |state| {
                    ekf_values(state)
                        .iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>()
                        .join(",")
                },
            ),
        );
        crate::facts::record(
            "fly.ekf.flags",
            state.map_or_else(
                || "none".to_owned(),
                |state| {
                    ekf_flags(state.ekf.flags)
                        .iter()
                        .map(|(text, _)| text.trim_end().to_owned())
                        .collect::<Vec<_>>()
                        .join(",")
                },
            ),
        );
        crate::facts::record(
            "fly.vibration.values",
            state.map_or_else(
                || "none".to_owned(),
                |state| {
                    let (bars, clips) = vibration_values(state);
                    format!(
                        "{};{}",
                        bars.map(|value| value.to_string()).join(","),
                        clips.map(|value| value.to_string()).join(",")
                    )
                },
            ),
        );
        crate::facts::record(
            "fly.mousedown",
            self.mouse_down_start.map_or_else(
                || "none".to_owned(),
                |(at, _)| format!("{:.6};{:.6}", at.latitude(), at.longitude()),
            ),
        );
        // How far the vehicle's home is from the point last pressed, in whole metres: where Set
        // Home Here puts it, this is nothing.
        crate::facts::record(
            "fly.mousedown.home",
            match (self.mouse_down_start, state.and_then(|state| state.home)) {
                (Some((at, _)), Some(home)) => format!("{:.0}", at.distance_to(home).0),
                _ => "none".to_owned(),
            },
        );
        self.conversions.record_facts();
        crate::facts::record("fly.hud.menu", self.hud_menu.open.is_some());
        crate::facts::record("fly.hud.menu.video", self.hud_menu.video);
        crate::facts::record("fly.hud.russian", self.hud_settings.russian);
        crate::facts::record("fly.hud.icons", self.hud_settings.icons);
        crate::facts::record(
            "fly.hud.ground",
            match self.hud_settings.ground {
                None => "hud",
                Some(true) => "brown",
                Some(false) => "green",
            },
        );
        crate::facts::record(
            "fly.hud.items",
            if self.hud_settings.items.is_empty() {
                "none".to_owned()
            } else {
                self.hud_settings
                    .items
                    .iter()
                    .map(|(name, header)| format!("{name}={header}"))
                    .collect::<Vec<_>>()
                    .join(",")
            },
        );
        crate::facts::record("fly.hud.items.open", self.hud_settings.choosing);
        // The Video drop-down's sources: GStreamer's pipeline and frames, MJPEG's, and the
        // runtime's download.
        let size = |frame: Option<std::sync::Arc<mp_video::Frame>>| {
            frame.map_or_else(
                || "none".to_owned(),
                |frame| format!("{}x{}", frame.width, frame.height),
            )
        };
        crate::facts::record("fly.hud.gstreamer", self.gstreamer.is_running());
        crate::facts::record(
            "fly.hud.gstreamer.pipeline",
            self.gstreamer.pipeline().unwrap_or("none"),
        );
        crate::facts::record("fly.hud.gstreamer.frames", self.gstreamer.frames());
        crate::facts::record("fly.hud.gstreamer.frame", size(self.gstreamer.latest()));
        crate::facts::record(
            "fly.hud.gstreamer.error",
            self.gstreamer.error().unwrap_or_else(|| "none".to_owned()),
        );
        crate::facts::record("fly.hud.gstreamer.download", self.gst_download.is_some());
        let mjpeg = self.mjpeg.as_ref();
        crate::facts::record("fly.hud.mjpeg", mjpeg.is_some());
        crate::facts::record(
            "fly.hud.mjpeg.frames",
            mjpeg.map_or(0, mp_video::mjpeg::CaptureMjpeg::frames),
        );
        crate::facts::record(
            "fly.hud.mjpeg.frame",
            size(mjpeg.and_then(mp_video::mjpeg::CaptureMjpeg::latest)),
        );
        crate::facts::record(
            "fly.hud.mjpeg.error",
            mjpeg
                .and_then(mp_video::mjpeg::CaptureMjpeg::error)
                .unwrap_or_else(|| "none".to_owned()),
        );
        crate::facts::record("fly.hud.items.choices", hud_item_choices().len());
        crate::facts::record("fly.swapped", self.swapped);
        // Where the HUD was laid out, so a swap is seen to move it: the column's left edge, or
        // the map's place right of the column.
        crate::facts::record(
            "fly.hud.left",
            self.hud_bounds.get().map_or_else(
                || "none".to_owned(),
                |laid_out| format!("{:.0}", f32::from(laid_out.origin.x)),
            ),
        );
        // Battery Cell Voltage: the count, and the line the HUD draws with it.
        crate::facts::record(
            "fly.hud.cells",
            self.hud_settings
                .cells
                .map_or_else(|| "off".to_owned(), |count| count.to_string()),
        );
        crate::facts::record(
            "fly.hud.cell",
            match (self.hud_settings.cells, state) {
                (Some(count), Some(state)) if count != 0 => {
                    #[allow(clippy::cast_precision_loss)] // a cell count
                    let per_cell = state.battery.voltage / count as f32;
                    format!("Cell {per_cell:.2}v")
                }
                _ => "none".to_owned(),
            },
        );
        crate::facts::record(
            "fly.menu",
            self.menu.map_or("none", |(kind, _)| kind.name()),
        );
        crate::facts::record(
            "fly.customize",
            self.customizing.as_ref().map_or_else(
                || "none".to_owned(),
                |list| {
                    list.iter()
                        .map(|(page, on)| format!("{}={}", page.name(), u8::from(*on)))
                        .collect::<Vec<_>>()
                        .join(",")
                },
            ),
        );
        // What the last Clear Track took off the map: the route starts again from the vehicle's
        // next position, so the count on the map is back to one a frame later.
        crate::facts::record(
            "fly.track.cleared",
            self.track_cleared
                .map_or_else(|| "none".to_owned(), |points| points.to_string()),
        );
        self.transponder.record_facts();
        self.payload.record_facts(state);
        self.scripts.record_facts();
        crate::facts::record("fly.gauge.speed.max", self.speed_gauge.max);
        crate::facts::record(
            "fly.gauge.speed.size",
            format!("{:.0}", gauge_place(self).2),
        );
        crate::facts::record(
            "fly.gauge.speed.needles",
            state.map_or_else(
                || "none".to_owned(),
                |state| {
                    let [air, ground] = gauge_needles(&self.speed_gauge, state);
                    format!("{};{}", air.value, ground.value)
                },
            ),
        );
    }
}

/// Opens a `.tlog` to play: `LoadLogFile`'s `logplaybackfile`, here a replay link paced by the
/// file's own timestamps, with the controls the Telemetry Logs page drives. Nothing is recorded
/// and no heartbeat is sent: the C# plays a log with its port closed.
/// `// C#: GCSViews/FlightData.cs:669-701`
pub fn replay(
    path: &str,
) -> Result<
    (
        crate::telemetry::Telemetry,
        Arc<mp_transport::replay::Playback>,
    ),
    String,
> {
    let transport = mp_transport::ReplayTransport::open(path).map_err(|err| err.to_string())?;
    let (transport, control) = transport.paced();
    let config = mp_link::LinkConfig {
        record_path: None,
        send_heartbeat: false,
        ..mp_link::LinkConfig::default()
    };
    let link = mp_link::Link::from_transport(Box::new(transport), config);
    Ok((
        crate::telemetry::Telemetry::over(link, &format!("file:{path}")),
        control,
    ))
}

/// `MainV2.comPort.BaseStream.IsOpen`: a link to a vehicle is up. A log being played is not a
/// port: the C# plays one with its port closed.
#[must_use]
pub fn port_open(view: &TelemetryView) -> bool {
    view.connected && !view.target.starts_with("file:")
}

/// `Settings.Instance.LogDir`, where Load Log's dialog opens: the directory flights are recorded
/// into, as the recorder finds it. `// C#: GCSViews/FlightData.cs:1274, ExtLibs/Utilities/Settings.cs:127-140`
fn log_directory() -> Option<std::path::PathBuf> {
    std::env::var_os("MP_LOG_DIR")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            mp_settings::Config::default_path()
                .and_then(|path| mp_settings::Config::load(&path).ok())
                .and_then(|config| config.log_directory())
        })
        .or_else(mp_settings::default_log_directory)
}

/// A button's words, looked up when it is drawn (`crate::i18n`).
pub type Words = fn() -> &'static str;

/// The speed buttons of `panel2`: each one's text in the configured culture, the `Tag`
/// `BUT_speed1_Click` parses, its id, and where the Designer puts it - the first four on one
/// row, the other three under them.
/// `// C#: GCSViews/FlightData.Designer.cs:2231-2317, GCSViews/FlightData.resx (BUT_speed*.Text)`
pub const SPEEDS: [(Words, f64, &str); 7] = [
    (
        || fl!("flightdata-BUT_speed1_10-Text"),
        0.1,
        "fly-speed1_10",
    ),
    (|| fl!("flightdata-BUT_speed1_4-Text"), 0.25, "fly-speed1_4"),
    (|| fl!("flightdata-BUT_speed1_2-Text"), 0.5, "fly-speed1_2"),
    (|| fl!("flightdata-BUT_speed1-Text"), 1.0, "fly-speed1"),
    (|| fl!("flightdata-BUT_speed2-Text"), 2.0, "fly-speed2"),
    (|| fl!("flightdata-BUT_speed5-Text"), 5.0, "fly-speed5"),
    (|| fl!("flightdata-BUT_speed10-Text"), 10.0, "fly-speed10"),
];

/// `TrackBar.LargeChange`, which the Designer leaves at its default: how far a press on the
/// channel beside the thumb moves it.
pub const TRACK_STEP: u8 = 5;

/// The Telemetry Logs page: the log playing, and the page's labels as the C# last set them.
#[derive(Debug)]
pub struct Playback {
    /// `MainV2.comPort.logplaybackfile`, as the replay's controls.
    control: Option<Arc<mp_transport::replay::Playback>>,
    /// `LBL_logfn.Text`.
    file_name: String,
    /// `tlogdir`: where Load Log's dialog opens. `// C#: GCSViews/FlightData.cs:1274`
    directory: Option<std::path::PathBuf>,
    /// `LogPlayBackSpeed`.
    speed: f64,
    /// `lbl_playbackspeed.Text`: "x 1.0" in the `.resx` until something sets it.
    speed_label: String,
    /// `lbl_logpercent.Text`: "0.00 %" in the `.resx` until something sets it.
    percent_label: String,
    /// `tracklog.Value`, 0 to 100.
    tracklog: u8,
    /// Whether the thumb is held.
    dragging: bool,
    /// Where the track bar was laid out.
    bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
}

impl Default for Playback {
    fn default() -> Self {
        Self {
            control: None,
            file_name: String::new(),
            directory: log_directory(),
            speed: 1.0,
            speed_label: "x 1.0".to_owned(),
            percent_label: "0.00 %".to_owned(),
            tracklog: 0,
            dragging: false,
            bounds: Rc::new(Cell::new(None)),
        }
    }
}

impl Playback {
    /// `LoadLogFile`: the log's name on the page, the track bar back to 0, playing.
    /// `// C#: GCSViews/FlightData.cs:669-701`
    pub fn load(&mut self, path: &str, control: Arc<mp_transport::replay::Playback>) {
        let path = std::path::Path::new(path);
        self.directory = path.parent().map(std::path::Path::to_path_buf);
        self.file_name = path
            .file_name()
            .map_or_else(String::new, |name| name.to_string_lossy().into_owned());
        control.set_speed(self.speed);
        control.set_paused(false);
        self.control = Some(control);
        self.tracklog = 0;
    }

    /// `logplaybackfile.Close()`: nothing plays until another log is loaded.
    pub fn close(&mut self) {
        if let Some(control) = self.control.take() {
            control.set_paused(true);
        }
    }

    /// Load Log with a port open: the name is shown, and the C#'s main loop closes the file at
    /// once, so nothing plays. `// C#: GCSViews/FlightData.cs:3439-3453`
    pub fn load_name_only(&mut self, path: &str) {
        self.file_name = std::path::Path::new(path)
            .file_name()
            .map_or_else(String::new, |name| name.to_string_lossy().into_owned());
        self.control = None;
    }

    /// `logreadmode`: a log is playing.
    #[must_use]
    pub fn playing(&self) -> bool {
        self.control
            .as_ref()
            .is_some_and(|control| control.is_playing())
    }

    /// `BUT_playlog.Text`, as `updatePlayPauseButton` sets it: "Pause" while playing, "Play"
    /// otherwise. `// C#: GCSViews/FlightData.cs:5630-5660`
    #[must_use]
    pub fn button(&self) -> &'static str {
        if self.playing() { "Pause" } else { "Play" }
    }

    /// `BUT_playlog_Click`: `logreadmode` toggled. With no log loaded the main loop turns it
    /// straight back off, so nothing happens. `// C#: GCSViews/FlightData.cs:556-594`
    pub fn toggle(&self) {
        if let Some(control) = &self.control {
            control.set_paused(!control.is_paused());
        }
    }

    /// `BUT_speed1_Click`: the button's tag as the speed, said on the label.
    /// `// C#: GCSViews/FlightData.cs:1674-1678`
    pub fn set_speed(&mut self, speed: f64) {
        self.speed = speed;
        if let Some(control) = &self.control {
            control.set_speed(speed);
        }
        self.speed_label = format!("x {}", mp_params::param_file::invariant_double(speed));
    }

    /// `tracklog_Scroll`: the file moved to the value's percentage of its length, and the percent
    /// label said again. `// C#: GCSViews/FlightData.cs:5361-5378`
    pub fn scroll(&mut self, value: u8) {
        self.tracklog = value.min(100);
        if let Some(control) = &self.control {
            control.seek_fraction(f64::from(self.tracklog) / 100.0);
            self.percent_label = percent(control);
        }
    }

    /// A press on the track bar, `fraction` of the way along it, `thumb` wide in fractions: on
    /// the thumb it takes hold of it; beside it, the thumb moves one `LargeChange` towards the
    /// press, which is a scroll.
    pub fn press(&mut self, fraction: f32, thumb: f32) {
        let at = f32::from(self.tracklog) / 100.0;
        if (fraction - at).abs() <= thumb / 2.0 {
            self.dragging = true;
            return;
        }
        let value = if fraction > at {
            self.tracklog.saturating_add(TRACK_STEP).min(100)
        } else {
            self.tracklog.saturating_sub(TRACK_STEP)
        };
        self.scroll(value);
    }

    /// The thumb dragged to `fraction` of the way along.
    pub fn drag(&mut self, fraction: f32) {
        if !self.dragging {
            return;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let value = (fraction.clamp(0.0, 1.0) * 100.0).round() as u8;
        if value != self.tracklog {
            self.scroll(value);
        }
    }

    /// The thumb let go.
    pub fn release(&mut self) {
        self.dragging = false;
    }

    /// Once a frame while a log plays: `updateLogPlayPosition`, which the main loop runs every
    /// 300 ms - the track bar at the file's position, the percentage, the speed.
    /// `// C#: GCSViews/FlightData.cs:3462-3470, 5543-5577`
    pub fn tick(&mut self) {
        let Some(control) = self.control.clone() else {
            return;
        };
        let at_end = !control.is_paused() && control.position() >= control.len();
        if !self.playing() && !at_end {
            return;
        }
        if !self.dragging && !control.is_empty() {
            #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
            #[allow(clippy::cast_sign_loss)]
            let value = (control.position() as f64 / control.len() as f64 * 100.0) as u8;
            self.tracklog = value.min(100);
        }
        self.percent_label = percent(&control);
        self.speed_label = format!("x {}", mp_params::param_file::invariant_double(self.speed));
        // The end of the file: the main loop sets `logreadmode` false, so a scroll back finds it
        // stopped and Play starts it again. `// C#: GCSViews/FlightData.cs:3556-3563`
        if at_end {
            control.set_paused(true);
        }
    }

    /// Publishes what a UI test asserts on.
    pub fn record_facts(&self) {
        crate::facts::record("fly.playback.file", &self.file_name);
        crate::facts::record("fly.playback.button", self.button());
        crate::facts::record("fly.playback.playing", self.playing());
        crate::facts::record("fly.playback.speed", &self.speed_label);
        crate::facts::record("fly.playback.percent", &self.percent_label);
        crate::facts::record("fly.playback.tracklog", self.tracklog);
        crate::facts::record(
            "fly.playback.position",
            self.control
                .as_ref()
                .map_or(0, |control| control.position()),
        );
    }
}

/// `(Position / (double)Length).ToString("0.00%")`: a hundred times the fraction, to two places,
/// and a percent sign.
fn percent(control: &mp_transport::replay::Playback) -> String {
    if control.is_empty() {
        return "0.00%".to_owned();
    }
    #[allow(clippy::cast_precision_loss)] // a file's length
    let fraction = control.position() as f64 / control.len() as f64;
    format!("{:.2}%", fraction * 100.0)
}

/// The Telemetry Logs page: `tableLayoutPaneltlogs`, three columns - 91 pixels, what is left, 36
/// pixels - of three rows. Load Log beside the log's name; Play/Pause beside the track bar and the
/// percentage; under them the speed buttons and the speed. Tlog > Kml or Graph's cell is empty:
/// that tool is `headless-planner kml` here.
/// `// C#: GCSViews/FlightData.Designer.cs:2193-2320, GCSViews/FlightData.resx (tableLayoutPaneltlogs.LayoutSettings)`
pub fn playback_page(playback: &Playback, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let left = |content: AnyElement| div().w(px(91.0)).flex_shrink_0().child(content);
    let right = |text: String| {
        div()
            .w(px(60.0))
            .flex_shrink_0()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child(text)
    };

    let row0 = div()
        .flex()
        .items_center()
        .gap_1()
        .h(px(33.0))
        .child(left(action(
            "fly-loadtelem",
            fl!("flightdata-BUT_loadtelem-Text"),
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                // `LBL_logfn.Text = ""` and the log playing closed, then the dialog.
                // `// C#: GCSViews/FlightData.cs:1276-1291`
                this.fly_data.playback.file_name.clear();
                this.fly_data.playback.close();
                let start = this
                    .fly_data
                    .playback
                    .directory
                    .as_ref()
                    .map_or_else(String::new, |dir| {
                        format!("{}{}", dir.display(), std::path::MAIN_SEPARATOR)
                    });
                this.fly_actions.ask(Prompt::LoadLog, &start);
                this.fly_focus.prompt.focus(window, cx);
                cx.notify();
            }),
        )))
        .child(
            crate::probe::measured("fly-logfn", div())
                .flex_1()
                .min_w(px(0.0))
                .truncate()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(playback.file_name.clone()),
        );

    let row1 = div()
        .flex()
        .items_center()
        .gap_1()
        .h(px(33.0))
        .child(left(action(
            "fly-playlog",
            playback.button(),
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.fly_data.playback.toggle();
                cx.notify();
            }),
        )))
        .child(track_bar(playback, cx))
        .child(right(playback.percent_label.clone()));

    // `panel2`: "Speed" over two rows of buttons.
    let mut first = div().flex().gap_1();
    let mut second = div().flex().gap_1();
    for (index, (label, speed, id)) in SPEEDS.iter().enumerate() {
        let button = action(
            id,
            label(),
            theme::TEXT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                this.fly_data.playback.set_speed(*speed);
                cx.notify();
            }),
        );
        if index < 4 {
            first = first.child(button);
        } else {
            second = second.child(button);
        }
    }
    let row2 = div()
        .flex()
        .gap_1()
        .child(div().w(px(91.0)).flex_shrink_0())
        .child(
            div()
                .flex()
                .flex_col()
                .gap_1()
                .flex_1()
                .min_w(px(0.0))
                .child(div().text_xs().text_color(rgb(theme::DIM)).child("Speed"))
                .child(first)
                .child(second),
        )
        .child(right(playback.speed_label.clone()));

    div()
        .flex()
        .flex_col()
        .gap_1()
        .child(row0)
        .child(row1)
        .child(row2)
        .into_any_element()
}

/// `tracklog`: a channel with a tick every five, and the thumb at the value.
fn track_bar(playback: &Playback, cx: &mut Context<MissionPlanner>) -> AnyElement {
    /// The thumb's width, in pixels.
    const THUMB: f32 = 10.0;
    let bounds = Rc::clone(&playback.bounds);
    let pressed = Rc::clone(&playback.bounds);
    let moved = Rc::clone(&playback.bounds);
    let value = f32::from(playback.tracklog) / 100.0;
    let fraction_at = |bounds: &Rc<Cell<Option<Bounds<Pixels>>>>, x: Pixels| {
        bounds.get().map(|laid_out| {
            let width = f32::from(laid_out.size.width).max(1.0);
            (
                (f32::from(x - laid_out.origin.x) / width).clamp(0.0, 1.0),
                THUMB / width,
            )
        })
    };
    let mut channel = div()
        .relative()
        .h(px(20.0))
        .child(
            gpui::canvas(
                move |laid_out, _window, _cx| bounds.set(Some(laid_out)),
                |_bounds, (), _window, _cx| {},
            )
            .absolute()
            .inset_0(),
        )
        .child(
            div()
                .absolute()
                .left_0()
                .right_0()
                .top(px(8.0))
                .h(px(4.0))
                .rounded_sm()
                .bg(rgb(theme::BORDER)),
        );
    for tick in (0..=100u8).step_by(5) {
        channel = channel.child(
            div()
                .absolute()
                .left(gpui::relative(f32::from(tick) / 100.0))
                .bottom_0()
                .w(px(1.0))
                .h(px(3.0))
                .bg(rgb(theme::DIM)),
        );
    }
    channel = channel.child(
        div()
            .absolute()
            .left(gpui::relative(value))
            .ml(px(-THUMB / 2.0))
            .top(px(2.0))
            .w(px(THUMB))
            .h(px(16.0))
            .rounded_sm()
            .bg(rgb(theme::ACCENT)),
    );
    crate::probe::measured("fly-tracklog", div())
        .id("fly-tracklog")
        .flex_1()
        .min_w(px(0.0))
        .cursor_pointer()
        .child(channel)
        .on_mouse_down(
            gpui::MouseButton::Left,
            cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                if let Some((fraction, thumb)) = fraction_at(&pressed, event.position.x) {
                    this.fly_data.playback.press(fraction, thumb);
                    cx.notify();
                }
            }),
        )
        .on_mouse_move(
            cx.listener(move |this, event: &gpui::MouseMoveEvent, _window, cx| {
                if event.pressed_button != Some(gpui::MouseButton::Left) {
                    return;
                }
                if let Some((fraction, _)) = fraction_at(&moved, event.position.x) {
                    this.fly_data.playback.drag(fraction);
                    cx.notify();
                }
            }),
        )
        .on_mouse_up(
            gpui::MouseButton::Left,
            cx.listener(|this, _event: &gpui::MouseUpEvent, _window, _cx| {
                this.fly_data.playback.release();
            }),
        )
        .into_any_element()
}

/// The DataFlash Logs page: `tableLayoutPanel2`, three columns of three rows, as its
/// `LayoutSettings` place each button. Download DataFlash Log Via Mavlink opens the Log
/// Downloader and Review a Log the log browser; the four conversions ask for a log and convert it
/// on a thread of their own, the page's conversion buttons waiting until it is done as the C#'s
/// window waits; Geo Reference Images opens its form (`crate::georef_ui`).
/// `// C#: GCSViews/FlightData.Designer.cs:2379-2389, GCSViews/FlightData.resx (tableLayoutPanel2.LayoutSettings)`
pub fn dataflash_page(data: &FlightData, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let idle = data.conversions.running().is_none();
    let mut grid = div()
        .grid()
        .grid_cols(3)
        .gap_1()
        .child(cell(
            0,
            0,
            grid_button(
                "fly-dfmavlink",
                fl!("flightdata-BUT_DFMavlink-Text"),
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.logs_open();
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            1,
            0,
            grid_button(
                "fly-logbrowse",
                fl!("flightdata-BUT_logbrowse-Text"),
                theme::ACCENT,
                true,
                // `new LogBrowse().Show()`: the log browser, which is a screen of its own here.
                // `// C#: GCSViews/FlightData.cs:1380-1385`
                cx.listener(|this, _event: &(), _window, cx| {
                    this.screen = crate::Screen::Logs;
                    this.remember();
                    cx.notify();
                }),
            ),
        ));
    for kind in Conversion::ALL {
        let (column, row) = kind.cell();
        grid = grid.child(cell(
            column,
            row,
            grid_button(
                kind.id(),
                kind.text(),
                theme::ACCENT,
                idle,
                cx.listener(move |this, _event: &(), window, cx| {
                    this.fly_ask_convert(kind, window, cx);
                    cx.notify();
                }),
            ),
        ));
    }
    // `new Georefimage().Show()`: the Geo Reference Images form, over the screen.
    // `// C#: GCSViews/FlightData.cs:5933-5936`
    grid = grid.child(cell(
        0,
        2,
        grid_button(
            "fly-georefimage",
            fl!("flightdata-BUT_georefimage-Text"),
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                crate::georef_ui::open(this);
                cx.notify();
            }),
        ),
    ));
    let mut page = div().flex().flex_col().gap_1().child(grid);
    if let Some(kind) = data.conversions.running() {
        page = page.child(
            crate::probe::measured("fly-convert-running", div())
                .text_xs()
                .text_color(rgb(theme::WARN))
                .child(format!("{}: working", kind.text())),
        );
    }
    page.into_any_element()
}

// --- The DataFlash Logs page's conversions --------------------------------------------------------

/// One of the DataFlash Logs page's conversion buttons. Each asks for a log and writes what the
/// C#'s button writes, where it writes it.
/// `// C#: GCSViews/FlightData.cs:1082-1098, 1135-1202, 1311-1378, 1387-1390`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Conversion {
    /// `but_bintolog`: `BinaryLog.ConvertBin` to `<name>.log` beside the log.
    BinToLog,
    /// `but_dflogtokml`: `LogOutput.writeKML(<log>.kml)` - the `.kmz`, the `.gpx` and the rest.
    DflogToKml,
    /// `BUT_matlab`: `MatLab.ProcessLog`, `<log>-<lines>.mat`.
    Matlab,
    /// `BUT_loganalysis`: ArduPilot's analyzer run on the log, and its report shown.
    LogAnalysis,
}

impl Conversion {
    /// The four, in the page's order: by row, then by column.
    pub const ALL: [Self; 4] = [
        Self::LogAnalysis,
        Self::DflogToKml,
        Self::BinToLog,
        Self::Matlab,
    ];

    /// The id a script clicks the button by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::BinToLog => "fly-bintolog",
            Self::DflogToKml => "fly-dflogtokml",
            Self::Matlab => "fly-matlab",
            Self::LogAnalysis => "fly-loganalysis",
        }
    }

    /// The button's `Text` in `FlightData.resx`, in the configured culture (`crate::i18n`).
    /// `// C#: GCSViews/FlightData.resx (but_bintolog.Text, but_dflogtokml.Text, BUT_matlab.Text, BUT_loganalysis.Text)`
    #[must_use]
    pub fn text(self) -> &'static str {
        match self {
            Self::BinToLog => fl!("flightdata-but_bintolog-Text"),
            Self::DflogToKml => fl!("flightdata-but_dflogtokml-Text"),
            Self::Matlab => fl!("flightdata-BUT_matlab-Text"),
            Self::LogAnalysis => fl!("flightdata-BUT_loganalysis-Text"),
        }
    }

    /// The name the facts use.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::BinToLog => "bintolog",
            Self::DflogToKml => "dflogtokml",
            Self::Matlab => "matlab",
            Self::LogAnalysis => "loganalysis",
        }
    }

    /// The cell `tableLayoutPanel2.LayoutSettings` gives the button: (column, row).
    /// `// C#: GCSViews/FlightData.resx:5156`
    #[must_use]
    pub const fn cell(self) -> (i16, i16) {
        match self {
            Self::LogAnalysis => (2, 0),
            Self::DflogToKml => (0, 1),
            Self::BinToLog => (1, 1),
            Self::Matlab => (2, 1),
        }
    }

    /// The first description of the dialog's `Filter`, which the prompt says as Load Log's does.
    /// `// C#: GCSViews/FlightData.cs:1086, 1139, 1315, Log/MatLabForms.cs:47`
    #[must_use]
    pub const fn filter(self) -> &'static str {
        match self {
            Self::BinToLog => "Binary Log",
            Self::DflogToKml | Self::Matlab => "Log Files",
            Self::LogAnalysis => "*.log;*.bin",
        }
    }

    /// Where the dialog opens: `tlogdir` - the log directory, or the folder of the last log
    /// loaded - for Create KML + gpx and Auto Analysis, and the log directory for Create Matlab
    /// File. Convert .Bin to .Log sets no `InitialDirectory`, so the system's dialog opens where
    /// it was last; here that is the log directory too.
    /// `// C#: GCSViews/FlightData.cs:1145, 1316, Log/MatLabForms.cs:53`
    #[must_use]
    pub fn directory(
        self,
        tlogdir: Option<&std::path::Path>,
        logdir: Option<&std::path::Path>,
    ) -> Option<std::path::PathBuf> {
        match self {
            Self::DflogToKml | Self::LogAnalysis => tlogdir.or(logdir),
            Self::BinToLog | Self::Matlab => logdir,
        }
        .map(std::path::Path::to_path_buf)
    }
}

/// What a conversion made.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Converted {
    /// The files written, each with its size in bytes.
    Files(Vec<(std::path::PathBuf, u64)>),
    /// Auto Analysis's report: the text `Controls.LogAnalyzer` shows.
    Report(String),
}

/// The files, each with its size as it lies on the disk.
fn sized(paths: Vec<std::path::PathBuf>) -> Converted {
    Converted::Files(
        paths
            .into_iter()
            .map(|path| {
                let size = std::fs::metadata(&path).map_or(0, |meta| meta.len());
                (path, size)
            })
            .collect(),
    )
}

/// One conversion of `log`, as its button's handler runs it once the dialog has given it the
/// file, with the application's flight mode names. `analyzer` is where Auto Analysis keeps
/// ArduPilot's analyzer, and `fetch` downloads it (`Download.getFilefromNet`); the other three use
/// neither.
///
/// # Errors
///
/// What the C#'s message box says: "Error processing file..." from Create KML + gpx's `catch`,
/// "Error converting file" from Create Matlab File's, and Auto Analysis's boxes. Convert .Bin to
/// .Log has no `catch`, and its failure is said as it is.
/// `// C#: GCSViews/FlightData.cs:1091-1097, 1151-1198, 1319-1377, Log/MatLabForms.cs:59-72`
pub fn convert(
    kind: Conversion,
    log: &std::path::Path,
    analyzer: Option<&std::path::Path>,
    fetch: &mut dyn FnMut(&str, &std::path::Path) -> bool,
) -> Result<Converted, String> {
    let modes = &mp_log::convert::flight_mode_name;
    match kind {
        Conversion::BinToLog => {
            let target = mp_log::convert::log_path_for(log);
            mp_log::convert::convert_bin_file(log, &target, modes)
                .map_err(|err| format!("{}: {err}", log.display()))?;
            Ok(sized(vec![target]))
        }
        Conversion::DflogToKml => {
            match mp_kml::dflog::dflog_to_kml(log, modes, &mp_kml::dflog::local_zone) {
                Ok(paths) => Ok(sized(paths)),
                Err(mp_kml::dflog::DflogKmlError::Io(err)) => Err(format!(
                    "Error processing file. Make sure the file is not in use.\n{err}"
                )),
                // `writeKML` throws after the side files, outside the `catch`.
                Err(err) => Err(err.to_string()),
            }
        }
        Conversion::Matlab => mp_log::matlab::process_log_file(log, modes)
            .map(|path| sized(vec![path]))
            .map_err(|err| format!("Error converting file {err}")),
        Conversion::LogAnalysis => {
            let dir = analyzer.ok_or("no home directory to keep the analyzer in")?;
            let analysis =
                mp_log::analysis::analyse(log, dir, fetch, modes).map_err(|err| err.to_string())?;
            Ok(Converted::Report(mp_log::analysis::report(&analysis)))
        }
    }
}

/// `Download.getFilefromNet(url, saveto)`: whether the analyzer arrived.
/// `// C#: Utilities/LogAnalyzer.cs:42-56`
fn download(url: &str, to: &std::path::Path) -> bool {
    use mp_firmware::manifest::Fetch as _;
    mp_firmware::manifest::Http
        .get(url)
        .is_ok_and(|bytes| std::fs::write(to, bytes).is_ok())
}

/// A conversion's outcome, with the log it was of.
pub type Outcome = (Conversion, std::path::PathBuf, Result<Converted, String>);

/// A conversion on its way: which, of what, and where its outcome arrives.
type Running = (
    Conversion,
    std::path::PathBuf,
    std::sync::mpsc::Receiver<Result<Converted, String>>,
);

/// The conversion running, what the last one made, and Auto Analysis's report window.
#[derive(Debug, Default)]
pub struct Conversions {
    /// The one running.
    running: Option<Running>,
    /// The last one to finish.
    last: Option<Outcome>,
    /// `Controls.LogAnalyzer`'s report, while its window is open.
    pub report: Option<String>,
}

impl Conversions {
    /// The conversion running, if one is.
    #[must_use]
    pub fn running(&self) -> Option<Conversion> {
        self.running.as_ref().map(|(kind, _, _)| *kind)
    }

    /// Starts `kind` on `log` on a thread of its own, returning whether it started. The C#
    /// converts on the window's thread and the window waits until it is done; here the screen
    /// goes on, and the page's conversion buttons wait instead, so nothing starts while one runs.
    pub fn start(
        &mut self,
        kind: Conversion,
        log: std::path::PathBuf,
        analyzer: Option<std::path::PathBuf>,
    ) -> bool {
        if self.running.is_some() {
            return false;
        }
        let (sender, receiver) = std::sync::mpsc::channel();
        let path = log.clone();
        let spawned = std::thread::Builder::new()
            .name(format!("mp-convert-{}", kind.name()))
            .spawn(move || {
                let outcome = convert(kind, &path, analyzer.as_deref(), &mut download);
                let _ = sender.send(outcome);
            });
        if spawned.is_err() {
            return false;
        }
        self.running = Some((kind, log, receiver));
        true
    }

    /// Once a frame: the outcome of a conversion that has just finished, once. An Auto Analysis
    /// report opens its window. `// C#: GCSViews/FlightData.cs:1348-1354`
    pub fn poll(&mut self) -> Option<Outcome> {
        let (kind, log, receiver) = self.running.as_ref()?;
        let result = match receiver.try_recv() {
            Ok(result) => result,
            Err(std::sync::mpsc::TryRecvError::Empty) => return None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                Err("the conversion stopped without an answer".to_owned())
            }
        };
        let finished = (*kind, log.clone(), result);
        self.running = None;
        if let (_, _, Ok(Converted::Report(text))) = &finished {
            self.report = Some(text.clone());
        }
        self.last = Some(finished.clone());
        Some(finished)
    }

    /// Publishes what a UI test asserts on: what is running, what finished last, the files it
    /// wrote by name and size, its error, and the report window's text.
    pub fn record_facts(&self) {
        crate::facts::record(
            "fly.logs.convert.running",
            self.running().map_or("none", Conversion::name),
        );
        let last = self.last.as_ref();
        crate::facts::record(
            "fly.logs.convert.last",
            last.map_or("none", |(kind, _, _)| kind.name()),
        );
        let files = match last {
            Some((_, _, Ok(Converted::Files(files)))) => files
                .iter()
                .map(|(path, size)| {
                    let name = path
                        .file_name()
                        .map_or_else(String::new, |name| name.to_string_lossy().into_owned());
                    format!("{name}:{size}")
                })
                .collect::<Vec<_>>()
                .join(","),
            _ => "none".to_owned(),
        };
        crate::facts::record("fly.logs.convert.files", files);
        crate::facts::record(
            "fly.logs.convert.error",
            match last {
                Some((_, _, Err(why))) => why.as_str(),
                _ => "none",
            },
        );
        crate::facts::record(
            "fly.logs.convert.report",
            self.report.as_deref().unwrap_or("none"),
        );
    }
}

/// The status line's words for a finished conversion: the files it wrote, or the error as its
/// message box says it. The C# says nothing when a conversion works - its window simply comes
/// back to life - so this says where the files went.
#[must_use]
pub fn conversion_status(outcome: &Outcome) -> String {
    let (kind, log, result) = outcome;
    match result {
        Ok(Converted::Files(files)) => format!(
            "{}: {}",
            kind.text(),
            files
                .iter()
                .map(|(path, _)| path.display().to_string())
                .collect::<Vec<_>>()
                .join(", ")
        ),
        Ok(Converted::Report(_)) => format!("{}: {}", kind.text(), log.display()),
        Err(why) => error_box(why),
    }
}

/// `Controls.LogAnalyzer`: "LogAnalyzer", its text box holding the report, shown with `Show()`.
/// `// C#: GCSViews/FlightData.cs:1348-1354, Controls/LogAnalyzer.Designer.cs:31-52`
fn log_analyzer_window(report: &str, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut lines = div().flex().flex_col();
    for line in report.lines() {
        lines = lines.child(
            div()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(line.to_owned()),
        );
    }
    let body = div().child(
        crate::probe::measured("fly-loganalyzer-text", div())
            .id("fly-loganalyzer-text")
            .w(px(622.0))
            .h(px(434.0))
            .p_1()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::BG))
            .overflow_y_scroll()
            .child(lines),
    );
    floating_window(
        "fly-loganalyzer",
        "LogAnalyzer",
        "fly-loganalyzer-close",
        (420.0, 80.0),
        body,
        cx.listener(|this, _event: &(), _window, cx| {
            this.fly_data.conversions.report = None;
            cx.notify();
        }),
    )
}

// --- The windows the HUD opens --------------------------------------------------------------------

/// Which window a click on the HUD opens.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HudWindow {
    /// `EKFStatus`, from `ekfhitzone`.
    Ekf,
    /// `Vibration`, from `vibehitzone`.
    Vibration,
}

impl HudWindow {
    /// The click target's id.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Ekf => "hud-ekf",
            Self::Vibration => "hud-vibe",
        }
    }
}

/// Where a click on the HUD opens a window: the rectangle `doPaint` sets where it draws "EKF"
/// or "Vibe" - at the text, 40 wide and twice the font high - grown up and left by the five
/// pixels of the box `OnMouseClick` tests against it. `(left, top, width, height)` in the HUD.
/// `// C#: ExtLibs/Controls/HUD.cs:1203-1231, 3156-3158, 3212-3215`
#[must_use]
pub fn hud_zone(scene: &crate::hud::Scene, window: HudWindow) -> Option<(f32, f32, f32, f32)> {
    let (element, text) = match window {
        HudWindow::Ekf => (crate::hud::Element::Ekf, "EKF"),
        HudWindow::Vibration => (crate::hud::Element::Vibe, "Vibe"),
    };
    scene
        .owners
        .iter()
        .filter(|(owner, _)| *owner == element)
        .find_map(|(_, index)| match scene.items.get(*index) {
            Some(crate::hud::Item::Label {
                text: drawn,
                at,
                size,
                ..
            }) if drawn == text => {
                // The text is drawn at `fontsize + 2`.
                let fontsize = size - 2.0;
                Some((at.0 - 5.0, at.1 - 5.0, 45.0, fontsize * 2.0 + 5.0))
            }
            _ => None,
        })
        // With the pictures showing there is no label: the picture's own rectangle, padded as
        // the C# pads its click zones. `// C#: ExtLibs/Controls/HUD.cs:3150-3301`
        .or_else(|| {
            scene
                .zone(element)
                .map(|(x, y, w, h)| (x - 5.0, y - 5.0, w + 5.0, h + 5.0))
        })
}

/// The primary flight display, with the two places a click opens a window and its menu on the
/// right button. Where the HUD and the map have been swapped it fills the map's place rather than
/// the column's top.
pub fn hud_panel(
    inputs: &crate::hud::HudInputs,
    data: &FlightData,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    // Everything on the display is in the scene now - the tapes carry their own numbers, the
    // mode and waypoint sit under the altitude scroller, battery and GPS along the bottom - as
    // `HUD.cs` paints them, so nothing is overlaid as widgets any more. The box is 16:9-ish and
    // tall enough that the C#'s `Height / 30` font is legible.
    let painted = inputs.clone();
    let ground = data.hud_settings.ground_colours();
    let camera = data.camera.clone();
    let bounds = Rc::clone(&data.hud_bounds);
    // The zones are where the last frame drew the text; before the first, the column's size.
    let (width, height) = data.hud_bounds.get().map_or((398.0, 258.0), |laid_out| {
        (
            f32::from(laid_out.size.width),
            f32::from(laid_out.size.height),
        )
    });
    let scene = hud_scene(inputs, ground, width, height);
    let hud = crate::probe::measured("hud", div())
        .id("hud")
        .relative()
        .w_full()
        .overflow_hidden()
        .rounded_md()
        .border_1()
        .border_color(rgb(theme::BORDER));
    // `SwapHud1AndMap` puts `hud1` in `MainH.Panel2`, which it fills; otherwise it is the top of
    // the column. `// C#: GCSViews/FlightData.cs:5139-5159`
    let hud = if data.swapped {
        hud.flex_1().min_h(px(0.0))
    } else {
        hud.h(px(260.0))
    };
    let mut hud = hud
        .child(
            gpui::canvas(
                move |laid_out, _window, _cx| bounds.set(Some(laid_out)),
                move |bounds, (), window, cx| {
                    let height = f32::from(bounds.size.height);
                    // Over a picture there is no sky or ground to colour (`bgon = false`), so
                    // Ground Color's is left out and the ground fill is the one the camera
                    // painter knows to skip. `// C#: ExtLibs/Controls/HUD.cs:1988-1990, 2067-2099`
                    let ground = if camera.is_some() { None } else { ground };
                    let scene = hud_scene(&painted, ground, f32::from(bounds.size.width), height);
                    crate::hud::paint_over_camera(
                        camera.as_ref(),
                        &scene,
                        painted.hud_on,
                        bounds,
                        window,
                        cx,
                    );
                },
            )
            .size_full(),
        )
        // `hud1.ContextMenuStrip = contextMenuStripHud`: the menu where the right button comes up.
        // `// C#: GCSViews/FlightData.Designer.cs:346`
        .on_mouse_up(
            gpui::MouseButton::Right,
            cx.listener(|this, event: &gpui::MouseUpEvent, _window, cx| {
                this.fly_data.hud_menu = HudMenu {
                    open: Some((f32::from(event.position.x), f32::from(event.position.y))),
                    video: false,
                };
                cx.notify();
            }),
        );
    for which in [HudWindow::Ekf, HudWindow::Vibration] {
        let Some((left, top, width, height)) = hud_zone(&scene, which) else {
            continue;
        };
        hud = hud.child(
            crate::probe::measured(which.id(), div())
                .id(which.id())
                .absolute()
                .left(px(left))
                .top(px(top))
                .w(px(width))
                .h(px(height))
                .cursor_pointer()
                // Measured through a child, as the probe measures: an empty box has none.
                .child(div().size_full())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    // `hud1_ekfclick` and `hud1_vibeclick`: the form shown.
                    // `// C#: GCSViews/FlightData.cs:3293-3323`
                    match which {
                        HudWindow::Ekf => this.fly_data.ekf_open = true,
                        HudWindow::Vibration => this.fly_data.vibration_open = true,
                    }
                    cx.notify();
                })),
        );
    }
    hud
}

/// `EKFStatus.timer1_Tick`'s five bars: each variance times a hundred, as an `int`.
/// `// C#: Controls/EKFStatus.cs:22-27`
#[must_use]
pub fn ekf_values(state: &mp_vehicle::VehicleState) -> [i32; 5] {
    let ekf = &state.ekf;
    #[allow(clippy::cast_possible_truncation)]
    [
        ekf.velocity_variance,
        ekf.position_horizontal_variance,
        ekf.position_vertical_variance,
        ekf.compass_variance,
        ekf.terrain_altitude_variance,
    ]
    .map(|variance| (variance * 100.0) as i32)
}

/// The bars' labels, under them. `// C#: Controls/EKFStatus.resx (label2-label6)`
pub const EKF_LABELS: [&str; 5] = [
    "Velocity",
    "Position (Horiz)",
    "Position (Vert)",
    "Compass",
    "Terrain",
];

/// A bar's colour: `ValueColor` from the Designer, orange past 50 and red past 80.
/// `// C#: Controls/EKFStatus.cs:32-39, Controls/EKFStatus.Designer.cs:57`
#[must_use]
pub const fn ekf_colour(value: i32) -> u32 {
    if value > 80 {
        0xff_00_00
    } else if value > 50 {
        0xff_a5_00
    } else {
        0xff_00_ff
    }
}

/// The flags list: every `EKF_STATUS_FLAGS` bit up to `EKF_UNINITIALIZED`, named without its
/// `EKF_` in lower case, then "On " or "Off"; red when horizontal velocity or either absolute
/// position is off. `// C#: Controls/EKFStatus.cs:41-67`
#[must_use]
pub fn ekf_flags(flags: u16) -> Vec<(String, bool)> {
    use mp_mavlink_dialects::all::EkfStatusFlags;
    let mut lines = Vec::new();
    let mut bit: u32 = 1;
    while bit <= EkfStatusFlags::EKF_UNINITIALIZED.0 {
        let on = u32::from(flags) & bit != 0;
        let name = EkfStatusFlags(bit)
            .name()
            .unwrap_or_default()
            .replace("EKF_", "")
            .to_lowercase();
        let red = !on
            && [
                EkfStatusFlags::EKF_VELOCITY_HORIZ.0,
                EkfStatusFlags::EKF_POS_HORIZ_ABS.0,
                EkfStatusFlags::EKF_POS_VERT_ABS.0,
            ]
            .contains(&bit);
        lines.push((format!("{name} {}", if on { "On " } else { "Off" }), red));
        bit <<= 1;
    }
    lines
}

/// `Vibration.timer1_Tick`: the three axes as `int`s, and the three clipping counts.
/// `// C#: Controls/Vibration.cs:21-29`
#[must_use]
pub fn vibration_values(state: &mp_vehicle::VehicleState) -> ([i32; 3], [u32; 3]) {
    let vibration = &state.vibration;
    #[allow(clippy::cast_possible_truncation)]
    let bars = [vibration.x, vibration.y, vibration.z].map(|level| level as i32);
    (bars, vibration.clipping)
}

/// A `VerticalProgressBar2`: the value from the bottom, and the red lines at `minline` and
/// `maxline` with their values, scaled for display, beside them.
/// `// C#: ExtLibs/Controls/HorizontalProgressBar2.cs:185-230`
fn vertical_bar(value: i32, maximum: i32, lines: (i32, i32), scale: f32, colour: u32) -> gpui::Div {
    #[allow(clippy::cast_precision_loss)]
    let fraction = |at: i32| at.clamp(0, maximum) as f32 / maximum.max(1) as f32;
    let mut bar = div()
        .relative()
        .w(px(58.0))
        .h(px(174.0))
        .bg(rgb(theme::BORDER))
        .border_1()
        .border_color(rgb(theme::DIM))
        .child(
            div()
                .absolute()
                .left_0()
                .right_0()
                .bottom_0()
                .h(gpui::relative(fraction(value)))
                .bg(rgb(colour)),
        );
    for line in [lines.0, lines.1] {
        // `(minline * _displayscale).ToString()`: a float, written as its shortest form.
        #[allow(clippy::cast_precision_loss)]
        let label = format!("{}", line as f32 * scale);
        bar = bar.child(
            div()
                .absolute()
                .left_0()
                .right_0()
                .top(gpui::relative(1.0 - fraction(line)))
                .h(px(2.0))
                .bg(rgb(0xff_00_00))
                .child(
                    div()
                        .absolute()
                        .left(px(5.0))
                        .top(px(-14.0))
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(label),
                ),
        );
    }
    bar
}

/// A form as `Show()` shows it: a titled box over the screen that leaves the rest usable, with
/// its close box.
fn floating_window(
    id: &'static str,
    title: &'static str,
    close: &'static str,
    at: (f32, f32),
    body: gpui::Div,
    on_close: impl Fn(&(), &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(at.0), px(at.1)))
            .child(
                crate::probe::measured(id, div())
                    .id(id)
                    .flex()
                    .flex_col()
                    .gap_2()
                    .p_3()
                    .bg(rgb(theme::PANEL))
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .rounded_md()
                    .occlude()
                    .child(
                        div()
                            .flex()
                            .justify_between()
                            .gap_3()
                            .child(div().text_xs().text_color(rgb(theme::DIM)).child(title))
                            .child(action(close, "\u{2715}", theme::TEXT, true, on_close)),
                    )
                    .child(body),
            ),
    )
    .with_priority(1)
    .into_any_element()
}

/// The windows the HUD has open: `EKFStatus` and `Vibration`, each `TopMost` and each updated
/// by its own timer - here, every frame.
pub fn hud_windows(
    data: &FlightData,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let mut windows = Vec::new();
    let state = view.state.as_deref().copied().unwrap_or_default();
    if data.ekf_open {
        // `tableLayoutPanel1`: "EKF Status" over five bars with their names under them, and
        // "Flags" beside them over the list. `// C#: Controls/EKFStatus.Designer.cs:81-96`
        let mut bars = div().flex().gap_1();
        for (value, label) in ekf_values(&state).into_iter().zip(EKF_LABELS) {
            bars = bars.child(
                div()
                    .flex()
                    .flex_col()
                    .items_center()
                    .gap_1()
                    .w(px(58.0))
                    .child(vertical_bar(value, 100, (50, 80), 0.01, ekf_colour(value)))
                    .child(div().text_xs().text_color(rgb(theme::TEXT)).child(label)),
            );
        }
        let mut flags = div()
            .flex()
            .flex_col()
            .w(px(142.0))
            .child(div().text_sm().text_color(rgb(theme::TEXT)).child("Flags"));
        for (text, red) in ekf_flags(state.ekf.flags) {
            flags = flags.child(
                div()
                    .text_xs()
                    .text_color(rgb(if red { 0xff_00_00 } else { theme::TEXT }))
                    .child(text),
            );
        }
        let body = div()
            .flex()
            .gap_3()
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap_1()
                    .child(
                        div()
                            .text_sm()
                            .text_color(rgb(theme::TEXT))
                            .child("EKF Status"),
                    )
                    .child(bars),
            )
            .child(flags);
        windows.push(floating_window(
            "fly-ekf-status",
            "EKF Status",
            "fly-ekf-close",
            (420.0, 80.0),
            body,
            cx.listener(|this, _event: &(), _window, cx| {
                this.fly_data.ekf_open = false;
                cx.notify();
            }),
        ));
    }
    if data.vibration_open {
        // "Vibration" over the X, Y and Z bars, and "Clipping" beside them over the three
        // counts. `// C#: Controls/Vibration.Designer.cs:104-120`
        let (values, clips) = vibration_values(&state);
        let mut bars = div().flex().gap_1();
        for (value, label) in values.into_iter().zip(["X", "Y", "Z"]) {
            bars = bars.child(
                div()
                    .flex()
                    .flex_col()
                    .items_center()
                    .gap_1()
                    .w(px(62.0))
                    .child(vertical_bar(value, 90, (30, 60), 1.0, 0xff_00_ff))
                    .child(div().text_xs().text_color(rgb(theme::TEXT)).child(label)),
            );
        }
        let mut clipping = div().flex().flex_col().gap_1().child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child("Clipping"),
        );
        for (label, count) in ["Primary", "Secondary", "Tertiary"].into_iter().zip(clips) {
            clipping = clipping.child(
                div()
                    .flex()
                    .items_center()
                    .gap_1()
                    .child(
                        div()
                            .w(px(62.0))
                            .text_xs()
                            .text_color(rgb(theme::TEXT))
                            .child(label),
                    )
                    .child(
                        div()
                            .w(px(47.0))
                            .px_1()
                            .border_1()
                            .border_color(rgb(theme::BORDER))
                            .bg(rgb(theme::BG))
                            .text_xs()
                            .text_color(rgb(theme::TEXT))
                            .child(count.to_string()),
                    ),
            );
        }
        let body = div()
            .flex()
            .gap_3()
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap_1()
                    .child(
                        div()
                            .text_sm()
                            .text_color(rgb(theme::TEXT))
                            .child("Vibration"),
                    )
                    .child(bars),
            )
            .child(clipping);
        windows.push(floating_window(
            "fly-vibration",
            "Vibration",
            "fly-vibration-close",
            (420.0, 420.0),
            body,
            cx.listener(|this, _event: &(), _window, cx| {
                this.fly_data.vibration_open = false;
                cx.notify();
            }),
        ));
    }
    windows
}

// --- The HUD's menu -------------------------------------------------------------------------------

/// What a row of the HUD's menu does here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HudAction {
    /// `videoToolStripMenuItem`: shows its drop-down.
    Video,
    /// `userItemsToolStripMenuItem`: `hud_UserItem`, the "Display This" form.
    UserItems,
    /// `russianHudToolStripMenuItem`: `hud1.Russian` turned over.
    Russian,
    /// `swapWithMapToolStripMenuItem`: `SwapHud1AndMap`.
    SwapWithMap,
    /// `groundColorToolStripMenuItem`: checked or unchecked by the click, the ground following.
    GroundColor,
    /// `showIconsToolStripMenuItem`: `hud1.displayicons` turned over, saved as `HUD_showicons`,
    /// the entry reading "Show text" while the pictures show.
    ShowIcons,
    /// `setBatteryCellCountToolStripMenuItem`: the cell voltage line off, or its count asked.
    BatteryCells,
    /// `setMJPEGSourceToolStripMenuItem`: the MJPEG source's URL asked, then `CaptureMJPEG`
    /// started on it, or stopped on Cancel.
    MjpegSource,
    /// `setGStreamerSourceToolStripMenuItem`: the pipeline asked, then `hudGStreamer` started
    /// on it, or stopped on Cancel.
    GStreamerSource,
    /// `hereLinkVideoToolStripMenuItem`: the HereLink's address asked, then its RTSP stream
    /// played through `hudGStreamer`.
    HereLinkVideo,
    /// `gStreamerStopToolStripMenuItem`: `hudGStreamer.Stop()`.
    GStreamerStop,
}

impl HudAction {
    /// Whether it is a row of the Video drop-down, which keeps the drop-down open while the
    /// pointer is on it.
    #[must_use]
    pub const fn in_video_menu(self) -> bool {
        matches!(
            self,
            Self::MjpegSource | Self::GStreamerSource | Self::HereLinkVideo | Self::GStreamerStop
        )
    }
}

/// A row of `contextMenuStripHud`, or of its Video drop-down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HudRow {
    /// The Designer's name.
    pub control: &'static str,
    /// Its `Text` in `FlightData.resx`.
    pub text: &'static str,
    /// The id a script clicks it by.
    pub id: &'static str,
    /// What it does here, or why it is drawn dimmed.
    pub does: Result<HudAction, &'static str>,
}

/// Why Start Camera is dimmed.
const NO_CAMERA: &str = "Start Camera, the capture opened in its default format, is not ported; \
                         the Planner page's Start opens the camera";

/// Why Record Hud to AVI and Stop Record are dimmed.
const NO_AVI: &str = "there is no AVI encoder here, the C#'s AviWriter";

/// `contextMenuStripHud`, in the Designer's order, with each row's words from the `.resx`.
/// `// C#: GCSViews/FlightData.Designer.cs:458-466, 525-566, GCSViews/FlightData.resx`
pub const HUD_MENU: [HudRow; 8] = [
    HudRow {
        control: "videoToolStripMenuItem",
        text: "Video",
        id: "fly-hud-video",
        does: Ok(HudAction::Video),
    },
    HudRow {
        control: "setAspectRatioToolStripMenuItem",
        text: "Set Aspect Ratio",
        id: "fly-hud-aspect",
        // `// C#: GCSViews/FlightData.cs:4783-4787, ExtLibs/Controls/HUD.cs:3739-3765`
        does: Err(
            "the C# makes the HUD 4:3 - or 16:9, once toggled - from its width, and this \
             screen's HUD is 260 pixels high; the C#'s 4:3 would change the column's layout",
        ),
    },
    HudRow {
        control: "userItemsToolStripMenuItem",
        text: "User Items",
        id: "fly-hud-useritems",
        does: Ok(HudAction::UserItems),
    },
    HudRow {
        control: "russianHudToolStripMenuItem",
        text: "Russian Hud",
        id: "fly-hud-russian",
        does: Ok(HudAction::Russian),
    },
    HudRow {
        control: "swapWithMapToolStripMenuItem",
        text: "Swap With Map",
        id: "fly-hud-swap",
        does: Ok(HudAction::SwapWithMap),
    },
    HudRow {
        control: "groundColorToolStripMenuItem",
        text: "Ground Color",
        id: "fly-hud-groundcolor",
        does: Ok(HudAction::GroundColor),
    },
    HudRow {
        control: "setBatteryCellCountToolStripMenuItem",
        text: "Battery Cell Voltage",
        id: "fly-hud-batterycells",
        // `// C#: GCSViews/FlightData.cs:6115-6140, ExtLibs/Controls/HUD.cs:2896-2906`
        does: Ok(HudAction::BatteryCells),
    },
    HudRow {
        control: "showIconsToolStripMenuItem",
        text: "Show icons",
        id: "fly-hud-showicons",
        // `// C#: GCSViews/FlightData.cs:6484-6496, ExtLibs/Controls/HUD.cs:2867-3293`
        does: Ok(HudAction::ShowIcons),
    },
];

/// `videoToolStripMenuItem`'s drop-down, in the Designer's order.
/// `// C#: GCSViews/FlightData.Designer.cs:470-523`
pub const HUD_VIDEO_MENU: [HudRow; 7] = [
    HudRow {
        control: "recordHudToAVIToolStripMenuItem",
        text: "Record Hud to AVI",
        id: "fly-hud-recordavi",
        // `// C#: GCSViews/FlightData.cs:4653-4672`
        does: Err(NO_AVI),
    },
    HudRow {
        control: "stopRecordToolStripMenuItem",
        text: "Stop Record",
        id: "fly-hud-stoprecord",
        // `// C#: GCSViews/FlightData.cs:5121-5137`
        does: Err(NO_AVI),
    },
    HudRow {
        control: "setMJPEGSourceToolStripMenuItem",
        text: "Set MJPEG source",
        id: "fly-hud-mjpeg",
        // `// C#: GCSViews/FlightData.cs:4892-4912`
        does: Ok(HudAction::MjpegSource),
    },
    HudRow {
        control: "startCameraToolStripMenuItem",
        text: "Start Camera",
        id: "fly-hud-startcamera",
        // `// C#: GCSViews/FlightData.cs:5100-5119`
        does: Err(NO_CAMERA),
    },
    HudRow {
        control: "setGStreamerSourceToolStripMenuItem",
        text: "Set GStreamer Source",
        id: "fly-hud-gstreamer",
        // `// C#: GCSViews/FlightData.cs:4813-4850`
        does: Ok(HudAction::GStreamerSource),
    },
    HudRow {
        control: "hereLinkVideoToolStripMenuItem",
        text: "HereLink Video",
        id: "fly-hud-herelink",
        // `// C#: GCSViews/FlightData.cs:3155-3183`
        does: Ok(HudAction::HereLinkVideo),
    },
    HudRow {
        control: "gStreamerStopToolStripMenuItem",
        text: "GStreamer Stop",
        id: "fly-hud-gstreamerstop",
        // `// C#: GCSViews/FlightData.cs:3150-3153`
        does: Ok(HudAction::GStreamerStop),
    },
];

/// The HUD's menu: where it is open, and whether its Video drop-down shows.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct HudMenu {
    /// Where the right button came up, in the window, while the menu is open.
    pub open: Option<(f32, f32)>,
    /// Whether the Video drop-down shows.
    pub video: bool,
}

/// `groundColor1` and `groundColor2` as Ground Color sets them when it is checked: brown.
/// `// C#: GCSViews/FlightData.cs:3124-3129`
pub const GROUND_BROWN: (u32, u32) = (0x93_4e_01, 0x3c_21_04);

/// The same when it is not: green. `// C#: GCSViews/FlightData.cs:3130-3135`
pub const GROUND_GREEN: (u32, u32) = (0x9b_b8_24, 0x41_4f_07);

/// What the HUD's menu has set on `hud1`.
///
/// For the session: the C# keeps each in its settings - `russian_hud`,
/// `groundColorToolStripMenuItem`, `hud1_useritem_<name>` - and the settings file is not this
/// module's to extend, as the quick views' choices are not.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct HudSettings {
    /// `hud1.Russian`.
    pub russian: bool,
    /// `groundColorToolStripMenuItem.Checked`, once the entry has been clicked. Until then the
    /// ground is the display's own.
    pub ground: Option<bool>,
    /// `hud1.CustomItems`: each property shown, with its header, in the order checked.
    pub items: Vec<(String, String)>,
    /// Whether the "Display This" form is open.
    pub choosing: bool,
    /// The property whose header is being asked for.
    pub pending: Option<String>,
    /// `hud1.displayicons`: pictures for the battery, GPS, vibration, EKF and pre-arm readouts
    /// instead of text. Read from `HUD_showicons` when the flight screen loads.
    /// `// C#: GCSViews/FlightData.cs:427`
    pub icons: bool,
    /// `hud1.displayCellVoltage` with `hud1.batterycellcount`: the count while the line is on.
    pub cells: Option<i32>,
}

impl HudSettings {
    /// Show icons: `myhud.displayicons = !myhud.displayicons`, and the flag saved as
    /// `HUD_showicons` in Mission Planner's config.xml. `// C#: GCSViews/FlightData.cs:6484-6496`
    pub fn toggle_icons(&mut self, persisted: &mut crate::settings::Persisted) {
        self.icons = !self.icons;
        persisted.set("HUD_showicons", if self.icons { "True" } else { "False" });
    }

    /// `Settings.Instance.GetBoolean("HUD_showicons", false)`, at the flight screen's load.
    /// `// C#: GCSViews/FlightData.cs:427`
    pub fn load_icons(&mut self, persisted: &crate::settings::Persisted) {
        self.icons = persisted
            .get("HUD_showicons")
            .is_some_and(|value| value.eq_ignore_ascii_case("true"));
    }

    /// The Show icons entry's text: "Show text" while the pictures show.
    /// `// C#: GCSViews/FlightData.cs:6488-6495`
    #[must_use]
    pub const fn icons_entry_text(&self) -> &'static str {
        if self.icons {
            "Show text"
        } else {
            "Show icons"
        }
    }

    /// Russian Hud: `hud1.Russian = !hud1.Russian`. `// C#: GCSViews/FlightData.cs:4735-4739`
    pub fn toggle_russian(&mut self) {
        self.russian = !self.russian;
    }

    /// Ground Color: the entry is `CheckOnClick`, so a click checks or unchecks it, and the
    /// handler then paints the ground brown or green by what it now is.
    /// `// C#: GCSViews/FlightData.Designer.cs:551, GCSViews/FlightData.cs:3122-3139`
    pub fn toggle_ground(&mut self) {
        self.ground = Some(!self.ground.unwrap_or(false));
    }

    /// The ground's two colours, once Ground Color has been clicked.
    #[must_use]
    pub fn ground_colours(&self) -> Option<(u32, u32)> {
        self.ground
            .map(|brown| if brown { GROUND_BROWN } else { GROUND_GREEN })
    }

    /// `hud1.CustomItems.ContainsKey(name)`.
    #[must_use]
    pub fn shows(&self, name: &str) -> bool {
        self.items.iter().any(|(shown, _)| shown == name)
    }

    /// A box of "Display This" clicked: `chk_box_hud_UserItem_CheckedChanged`. A checked box is
    /// unchecked and its item comes off the HUD. An unchecked one asks for its header, starting
    /// at the box's text and ": " - returned for the question, whose answer is [`Self::add_item`].
    /// `// C#: GCSViews/FlightData.cs:2436-2472`
    pub fn click_item(&mut self, name: &str) -> Option<String> {
        if self.shows(name) {
            self.items.retain(|(shown, _)| shown != name);
            return None;
        }
        self.pending = Some(name.to_owned());
        Some(format!("{name}: "))
    }

    /// `addHudUserItem`: `hud1.CustomItems[name] = cust`, the header replaced where the item is
    /// already there. `// C#: GCSViews/FlightData.cs:948-955`
    pub fn add_item(&mut self, name: &str, header: &str) {
        match self.items.iter_mut().find(|(shown, _)| shown == name) {
            Some(item) => header.clone_into(&mut item.1),
            None => self.items.push((name.to_owned(), header.to_owned())),
        }
    }

    /// Hands the display what the menu has set: the Russian flag, and each user item with its
    /// value read from the vehicle, as `HUD.Custom` reads its property by reflection as it paints.
    /// `// C#: ExtLibs/Controls/HUD.cs:949-968`
    pub fn apply(
        &self,
        inputs: &mut crate::hud::HudInputs,
        state: Option<&mp_vehicle::VehicleState>,
    ) {
        inputs.russian = self.russian;
        inputs.display_icons = self.icons;
        // `hud1.displayCellVoltage` and `hud1.batterycellcount`, which HUD.cs:2896-2923 draws.
        inputs.display_cell_voltage = self.cells.is_some();
        inputs.battery_cell_count = self.cells.unwrap_or(0);
        inputs.custom_items = self
            .items
            .iter()
            .map(|(name, header)| crate::hud::CustomItem {
                header: header.clone(),
                name: name.clone(),
                value: state.and_then(|state| crate::quick::value(name, state)),
            })
            .collect();
    }
}

/// `CurrentState.StringCompareTo`, the order "Display This" lists its boxes in: character by
/// character ignoring case, a run of digits against a run of digits as numbers, and the shorter
/// first when one runs out.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4409-4459`
#[must_use]
pub fn string_compare_to(a: &str, b: &str) -> std::cmp::Ordering {
    use std::cmp::Ordering;
    let (arr1, arr2): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let number = |chars: &[char], at: &mut usize| {
        let mut digits = String::new();
        while let Some(&c) = chars.get(*at)
            && c.is_ascii_digit()
        {
            digits.push(c);
            *at += 1;
        }
        digits.parse::<u128>().unwrap_or(u128::MAX)
    };
    let lower = |c: char| c.to_lowercase().next().unwrap_or(c);
    let (mut i, mut j) = (0, 0);
    while let (Some(&x), Some(&y)) = (arr1.get(i), arr2.get(j)) {
        if x.is_ascii_digit() && y.is_ascii_digit() {
            let (s1, s2) = (number(&arr1, &mut i), number(&arr2, &mut j));
            match s1.cmp(&s2) {
                Ordering::Equal => {}
                unequal => return unequal,
            }
        } else {
            match lower(x).cmp(&lower(y)) {
                Ordering::Equal => {}
                unequal => return unequal,
            }
            i += 1;
            j += 1;
        }
    }
    arr1.len().cmp(&arr2.len())
}

/// What "Display This" offers: every numeric `CurrentState` property this application holds -
/// the quick view's chooser's, less the `bool`s, which `IsNumber` refuses here where the quick
/// view's form turns them into 0 and 1 - in `StringCompareTo`'s order.
/// `// C#: GCSViews/FlightData.cs:3203-3236, ExtLibs/Utilities/Extensions.cs:681-705`
#[must_use]
pub fn hud_item_choices() -> Vec<&'static str> {
    let mut names: Vec<&'static str> = crate::quick::choices()
        .into_iter()
        .filter(|name| {
            mp_vehicle::coverage::CURRENTSTATE
                .iter()
                .any(|field| field.name == *name && field.ty != "bool")
        })
        .collect();
    names.sort_by(|a, b| string_compare_to(a, b));
    names
}

/// The display's scene, with the ground in the colours Ground Color chose.
///
/// `hud.rs` paints the sky and the ground each as one fill - the scene's first two items - where
/// the C# paints each as a gradient; the ground here takes `groundColor1`, the colour the C#'s
/// gradient has along the horizon. Until Ground Color is clicked the ground is the display's own.
/// `// C#: ExtLibs/Controls/HUD.cs:2080-2094`
#[must_use]
pub fn hud_scene(
    inputs: &crate::hud::HudInputs,
    ground: Option<(u32, u32)>,
    w: f32,
    h: f32,
) -> crate::hud::Scene {
    let mut scene = crate::hud::scene(inputs, w, h);
    if let Some((top, _)) = ground
        && let Some(crate::hud::Item::Fill { colour, .. }) = scene.items.get_mut(1)
    {
        *colour = top;
    }
    scene
}

/// A menu row's height: fixed, as the planning map's menu has it.
const HUD_MENU_ROW: f32 = 22.0;
/// The menu's padding above its first row and below its last.
const HUD_MENU_PADDING: f32 = 4.0;
/// The menu's width and its drop-down's.
const HUD_MENU_WIDTH: f32 = 190.0;

/// One row, as a `ToolStripMenuItem` draws: a check where it is checked, its text, an arrow where
/// it has a drop-down, and dimmed where it does nothing here. A dimmed row says why on the status
/// line when it is clicked. The planning map's `menu_row` (`plan.rs`) is that screen's own; this
/// is the same drawing for this menu.
fn hud_menu_row(
    row: &'static HudRow,
    text: &'static str,
    checked: bool,
    highlighted: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(row.id, div())
        .id(row.id)
        .h(px(HUD_MENU_ROW))
        .px_2()
        .flex()
        .items_center()
        .justify_between()
        .gap_2()
        .text_xs()
        .child(
            div()
                .flex()
                .gap_1()
                .child(
                    div()
                        .w(px(10.0))
                        .child(if checked { "\u{2713}" } else { "" }),
                )
                .child(text),
        )
        .children((row.does == Ok(HudAction::Video)).then_some("\u{203a}"));
    match row.does {
        Ok(action) => base
            .text_color(rgb(theme::TEXT))
            .bg(rgb(if highlighted {
                theme::ACTION
            } else {
                theme::PANEL
            }))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            // The Video row shows its drop-down while the pointer is on it, and any other row
            // hides it, as a ToolStrip does.
            .on_hover(cx.listener(move |this, hovered: &bool, _window, cx| {
                if *hovered && this.fly_data.hud_menu.open.is_some() {
                    this.fly_data.hud_menu.video =
                        action == HudAction::Video || action.in_video_menu();
                    cx.notify();
                }
            }))
            .on_click(cx.listener(move |this, _event, window, cx| {
                // A question asked takes the keys.
                if this.fly_hud_menu(action) {
                    this.fly_focus.prompt.focus(window, cx);
                }
                cx.notify();
            }))
            .into_any_element(),
        Err(why) => base
            .text_color(rgb(theme::DIM))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.file_status = Some(format!("{} is not ported: {why}", row.text));
                cx.notify();
            }))
            .into_any_element(),
    }
}

/// A column of rows: the menu, or its drop-down.
fn hud_menu_column(id: &'static str, rows: Vec<AnyElement>) -> gpui::Stateful<gpui::Div> {
    div()
        .id(id)
        .flex()
        .flex_col()
        .w(px(HUD_MENU_WIDTH))
        .py(px(HUD_MENU_PADDING))
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .occlude()
        .children(rows)
}

/// `contextMenuStripHud`, where the right button came up over the HUD, moved in to fit the
/// window, with the Video drop-down beside its row while it shows. A press anywhere else closes
/// it, and goes no further, as a `ContextMenuStrip` closes.
/// `// C#: GCSViews/FlightData.Designer.cs:346, 458-468`
fn hud_menu(
    menu: HudMenu,
    settings: &HudSettings,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let (x, y) = menu.open?;
    let size = window.viewport_size();
    #[allow(clippy::cast_precision_loss)] // eight rows
    let height = 2.0f32.mul_add(HUD_MENU_PADDING, 2.0) + HUD_MENU.len() as f32 * HUD_MENU_ROW;
    let left = x.min(f32::from(size.width) - HUD_MENU_WIDTH).max(0.0);
    let top = y.min(f32::from(size.height) - height).max(0.0);
    let rows = HUD_MENU
        .iter()
        .map(|row| {
            let checked = match row.does {
                Ok(HudAction::GroundColor) => settings.ground == Some(true),
                _ => false,
            };
            let highlighted = row.does == Ok(HudAction::Video) && menu.video;
            // Show icons reads "Show text" while the pictures show.
            let text = if row.does == Ok(HudAction::ShowIcons) {
                settings.icons_entry_text()
            } else {
                row.text
            };
            hud_menu_row(row, text, checked, highlighted, cx)
        })
        .collect();
    let mut body = div()
        .absolute()
        .left(px(left))
        .top(px(top))
        .child(hud_menu_column("fly-hud-menu", rows));
    if menu.video {
        let rows = HUD_VIDEO_MENU
            .iter()
            .map(|row| hud_menu_row(row, row.text, false, false, cx))
            .collect();
        // Beside the Video row, the menu's first: on its right, or on its left where the window
        // ends first, as a `ToolStripDropDown` opens.
        let beside = if left + 2.0 * HUD_MENU_WIDTH > f32::from(size.width) {
            -HUD_MENU_WIDTH
        } else {
            HUD_MENU_WIDTH
        };
        body = body.child(
            div()
                .absolute()
                .left(px(beside))
                .top(px(0.0))
                .child(hud_menu_column("fly-hud-video-menu", rows)),
        );
    }
    let close = cx.listener(|this, _event: &gpui::MouseDownEvent, _window, cx| {
        this.fly_data.hud_menu = HudMenu::default();
        cx.notify();
    });
    let close_right = cx.listener(|this, _event: &gpui::MouseDownEvent, _window, cx| {
        this.fly_data.hud_menu = HudMenu::default();
        cx.notify();
    });
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .relative()
                        .w(size.width)
                        .h(size.height)
                        .child(
                            div()
                                .id("fly-hud-menu-backdrop")
                                .absolute()
                                .inset_0()
                                .occlude()
                                .on_mouse_down(gpui::MouseButton::Left, close)
                                .on_mouse_down(gpui::MouseButton::Right, close_right),
                        )
                        .child(body),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

/// `hud_UserItem`'s form, "Display This": every property as a check box, in as many columns as
/// fit in four fifths of the window, each as wide as the longest text and 15 more, filled top to
/// bottom; the ones on the HUD checked and green. A box checked asks for its header; one
/// unchecked takes its item off. The form sizes itself to its boxes, up to the window.
/// `// C#: GCSViews/FlightData.cs:3185-3271`
fn hud_items_chooser(
    settings: &HudSettings,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let names = hud_item_choices();
    let size = window.viewport_size();
    // `TextRenderer.MeasureText` of the longest, at the form's 8.25pt font: about seven pixels a
    // character, as the quick view's chooser takes it.
    #[allow(clippy::cast_precision_loss)]
    let max_length = names.iter().map(|name| name.len()).max().unwrap_or(1) as f32 * 7.0 + 15.0;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let columns = ((f32::from(size.width) * 0.8) / max_length).max(1.0) as usize;
    let rows = names.len().div_ceil(columns).max(1);

    let mut table = div().flex().gap_1();
    for column in names.chunks(rows) {
        let mut list = div().flex().flex_col().w(px(max_length));
        for name in column {
            let checked = settings.shows(name);
            let id = format!("fly-hud-item-{name}");
            let chosen = (*name).to_owned();
            list = list.child(
                crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .flex()
                    .items_center()
                    .gap_1()
                    .h(px(20.0))
                    .px_1()
                    .text_xs()
                    .cursor_pointer()
                    .bg(rgb(if checked { 0x00_80_00 } else { theme::PANEL }))
                    .text_color(rgb(theme::TEXT))
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(if checked { "\u{2611}" } else { "\u{2610}" })
                    .child((*name).to_owned())
                    .on_click(cx.listener(move |this, _event, window, cx| {
                        if let Some(header) = this.fly_data.hud_settings.click_item(&chosen) {
                            this.fly_actions.ask(Prompt::HudHeader, &header);
                            this.fly_focus.prompt.focus(window, cx);
                        }
                        cx.notify();
                    })),
            );
        }
        table = table.child(list);
    }

    let dialog = crate::probe::measured("fly-hud-items", div())
        .id("fly-hud-items")
        .flex()
        .flex_col()
        .gap_2()
        .max_w(size.width - px(100.0))
        .max_h(size.height - px(100.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .child(
            div()
                .flex()
                .justify_between()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child("Display This"),
                )
                .child(action(
                    "fly-hud-items-close",
                    "\u{2715}",
                    theme::TEXT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.fly_data.hud_settings.choosing = false;
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .id("fly-hud-items-choices")
                .flex_1()
                .min_h(px(0.0))
                .overflow_y_scroll()
                .child(table),
        );

    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("fly-hud-items-backdrop")
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

/// What the flight screen shows over itself besides the question, the HUD's windows, the Log
/// Downloader and the quick view's chooser: the HUD's menu, its User Items form, Auto
/// Analysis's report, Customize's form, and the strip's and quick views' menus.
pub fn overlays(
    data: &FlightData,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let mut shown = Vec::new();
    if let Some(report) = &data.conversions.report {
        shown.push(log_analyzer_window(report, cx));
    }
    if data.hud_settings.choosing {
        shown.push(hud_items_chooser(&data.hud_settings, window, cx));
    }
    shown.extend(hud_menu(data.hud_menu, &data.hud_settings, window, cx));
    if let Some(list) = &data.customizing {
        shown.push(customize_form(list, window, cx));
    }
    if let Some(menu) = data.menu {
        shown.push(context_menu(menu, window, cx));
    }
    shown
}

// --- Jump To Tag ----------------------------------------------------------------------------------

/// The message box Jump To Tag shows for a tag that is not one, before asking again.
/// `// C#: GCSViews/FlightData.cs:6512-6516`
pub const INVALID_TAG: &str = "Invalid Tag. Must be a number from 0 to 65535";

/// `UInt16.TryParse` of the tag: an unsigned 16-bit number, with white space around it and a sign
/// allowed, as `NumberStyles.Integer` allows them. `// C#: GCSViews/FlightData.cs:6512`
#[must_use]
pub fn parse_tag(text: &str) -> Option<u16> {
    let text = text.trim();
    // "-0" is a zero to .NET; any other minus is out of range.
    if let Some(rest) = text.strip_prefix('-') {
        return (!rest.is_empty() && rest.bytes().all(|b| b == b'0')).then_some(0);
    }
    text.parse().ok()
}

/// `doCommand(MAV_CMD.DO_JUMP_TAG, tag, 0, 0, 0, 0, 0, 0)` to the vehicle flown.
/// `// C#: GCSViews/FlightData.cs:6521`
#[must_use]
pub fn jump_to_tag_message(target: VehicleId, tag: u16) -> MavMessage {
    let command = u16::try_from(MavCmd::MAV_CMD_DO_JUMP_TAG.0).unwrap_or(u16::MAX);
    commands::command_long(
        target,
        command,
        [f32::from(tag), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

// --- The map menu's camera and home entries, and the grid's Message and Set Mount ---------------

/// Set Home Here's and Set EKF Origin Here's message box where the terrain has no height for the
/// point. `// C#: GCSViews/FlightData.cs:4798, 4861`
pub const NO_SRTM: &str = "No SRTM data for this area";

/// Point Camera Here's message box with no link. `// C#: GCSViews/FlightData.cs:4513-4517`
pub const PLEASE_CONNECT: &str = "Please Connect First";

/// Point Camera Here's message box before the map has been pressed - not `Strings.BadCoords`,
/// which reads "Lng". `// C#: GCSViews/FlightData.cs:4530-4534`
pub const BAD_LAT_LONG: &str = "Bad Lat/Long";

/// Battery Cell Voltage's message box for a count that is not a whole number - the C#'s words.
/// `// C#: GCSViews/FlightData.cs:6130-6134`
pub const BAD_RADIUS: &str = "Bad Radius";

/// `send_text`'s severity for Message: 5, `MAV_SEVERITY_NOTICE`. `// C#: GCSViews/FlightData.cs:1266`
pub const MESSAGE_SEVERITY: u8 = 5;

/// `MouseDownStart`, as degrees: where the flight map was last pressed, or `(0, 0)` -
/// `PointLatLng`'s default - before any press. `// C#: GCSViews/FlightData.cs:58, 2956-2959`
#[must_use]
pub fn mouse_down_point(data: &FlightData) -> (f64, f64) {
    data.mouse_down_start
        .map_or((0.0, 0.0), |(at, _)| (at.latitude(), at.longitude()))
}

/// `GET_HOME_POSITION`, which `getHomePositionAsync` sends without waiting for an answer to it -
/// it waits for `HOME_POSITION` instead. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3343-3362`
#[must_use]
pub fn get_home_position(target: VehicleId) -> MavMessage {
    commands::command_long(target, requests::CMD_GET_HOME_POSITION, [0.0; 7])
}

/// Set Home Here's terrain check: the height at the point, from a tile or the sea; anything else
/// is [`NO_SRTM`]. `// C#: GCSViews/FlightData.cs:4858-4863`
pub fn set_home_height(answer: crate::srtm::AltResponse) -> Result<f64, Refusal> {
    match answer.current_type {
        crate::srtm::TileType::Valid | crate::srtm::TileType::Ocean => Ok(answer.alt),
        crate::srtm::TileType::Invalid => Err(Refusal::error(NO_SRTM)),
    }
}

/// Set Home Here, once asked: `DO_SET_HOME` as a `COMMAND_INT` at the point and the terrain's
/// height if the answer was OK, and then - whatever the answer - the home position asked for,
/// as the C#'s `getHomePositionAsync` is outside its `if`.
///
/// Each waits as the C# waits, through [`route`]: `doCommandInt` for its `COMMAND_ACK`, three
/// more sends two seconds apart, and `getHomePositionAsync` for `HOME_POSITION`, asking again
/// three times 700 ms apart. The home the vehicle then reports is what the map draws.
/// `// C#: GCSViews/FlightData.cs:4852-4885`
#[must_use]
pub fn set_home_messages(
    target: VehicleId,
    (latitude, longitude, altitude): (f64, f64, f64),
    accepted: bool,
) -> Vec<MavMessage> {
    let mut messages = Vec::new();
    if accepted {
        messages.push(commands::set_home(target, latitude, longitude, altitude));
    }
    messages.push(get_home_position(target));
    messages
}

/// Set EKF Origin Here: `SET_GPS_GLOBAL_ORIGIN` at the point with the terrain's height, which
/// must come from a tile - the sea will not do here. `// C#: GCSViews/FlightData.cs:4789-4811`
pub fn set_ekf_origin_sends(
    target: VehicleId,
    (latitude, longitude): (f64, f64),
    answer: crate::srtm::AltResponse,
) -> Sends {
    if answer.current_type != crate::srtm::TileType::Valid {
        return Err(Refusal::error(NO_SRTM));
    }
    Ok(vec![commands::set_gps_global_origin(
        target.sysid,
        latitude,
        longitude,
        answer.alt,
    )])
}

/// `MAV_CMD_DO_SET_ROI` as `doCommandInt` sends it: the point at 1e7 in `x` and `y`.
#[must_use]
pub fn do_set_roi(
    target: VehicleId,
    frame: u8,
    latitude: f64,
    longitude: f64,
    z: f32,
) -> MavMessage {
    let command = u16::try_from(MavCmd::MAV_CMD_DO_SET_ROI.0).unwrap_or(u16::MAX);
    #[allow(clippy::cast_possible_truncation)] // `(int)(lat * 1e7)`
    let (x, y) = ((latitude * 1e7) as i32, (longitude * 1e7) as i32);
    commands::command_int(target, command, frame, [0.0; 4], x, y, z)
}

/// `float.TryParse`: a single, white space around it allowed.
#[must_use]
pub fn dotnet_float(text: &str) -> Option<f32> {
    text.trim()
        .parse::<f32>()
        .ok()
        .filter(|value| !value.is_infinite())
}

/// `int.TryParse`: a whole number with an optional sign, white space around it allowed.
#[must_use]
pub fn dotnet_int(text: &str) -> Option<i32> {
    text.trim().parse::<i32>().ok()
}

/// `string.IsNumber()`: `decimal.TryParse` with `NumberStyles.Number` - white space around it,
/// a sign before or after, thousands separators and one decimal point, and at least one digit.
/// `// C#: ExtLibs/Utilities/Extensions.cs:670-674`
#[must_use]
pub fn is_number(text: &str) -> bool {
    let text = text.trim();
    let text = text
        .strip_prefix(['-', '+'])
        .or_else(|| text.strip_suffix(['-', '+']))
        .unwrap_or(text);
    let (whole, fraction) = text.split_once('.').unwrap_or((text, ""));
    let digits = whole.chars().filter(char::is_ascii_digit).count()
        + fraction.chars().filter(char::is_ascii_digit).count();
    digits > 0
        && whole.chars().all(|c| c.is_ascii_digit() || c == ',')
        && fraction.chars().all(|c| c.is_ascii_digit())
        && !whole.starts_with(',')
}

/// Point Camera Here, once its height is given: `DO_SET_ROI` at the point last pressed, the
/// height above home in the relative frame. A height that is not a number is "Bad Alt", and a
/// point with a zero latitude or longitude - the map not pressed yet - is "Bad Lat/Long".
/// `// C#: GCSViews/FlightData.cs:4524-4541`
pub fn point_camera_here_sends(target: VehicleId, (lat, lng): (f64, f64), text: &str) -> Sends {
    let Some(alt) = dotnet_float(text) else {
        return Err(Refusal::error(strings::BAD_ALT));
    };
    if lat == 0.0 || lng == 0.0 {
        return Err(Refusal::error(BAD_LAT_LONG));
    }
    Ok(vec![do_set_roi(
        target,
        commands::FRAME_GLOBAL_RELATIVE_ALT,
        lat,
        lng,
        alt / MULTIPLIER_ALT,
    )])
}

/// Point Camera Coords: `lat;long;alt` sends `DO_SET_ROI` at that height above sea level, and
/// `lat;long` at the terrain's height there - its `alt` whatever the answer, 0 where there is no
/// tile - both in `doCommandInt`'s default frame, `GLOBAL`. Each part is a `float`, as Fly To
/// Coords reads them, and anything else is `Strings.InvalidField`; a part that is not a number
/// throws in the C#, which has no `catch` here, and is the same `InvalidField` here.
/// `// C#: GCSViews/FlightData.cs:4478-4509`
pub fn point_camera_coords_sends(
    target: VehicleId,
    text: &str,
    terrain: impl Fn(f64, f64) -> f64,
) -> Sends {
    let (latitude, longitude, z) = match parse_coords(text)? {
        Coords::Full {
            latitude,
            longitude,
            altitude,
        } => {
            // `alt / CurrentState.multiplieralt`, a float.
            #[allow(clippy::cast_possible_truncation)]
            let z = altitude as f32 / MULTIPLIER_ALT;
            (latitude, longitude, z)
        }
        Coords::Position {
            latitude,
            longitude,
        } => {
            // `(float)srtm.getAltitude(lat, lng).alt`.
            #[allow(clippy::cast_possible_truncation)]
            let z = terrain(latitude, longitude) as f32;
            (latitude, longitude, z)
        }
    };
    Ok(vec![do_set_roi(
        target,
        commands::FRAME_GLOBAL,
        latitude,
        longitude,
        z,
    )])
}

/// `send_text(5, txt)`: a `STATUSTEXT` the vehicle writes to its log, the text's ASCII bytes cut
/// or padded with zeros to fifty, as `StructureToByteArray` fits an array to its field.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6493-6498, ExtLibs/Mavlink/MavlinkUtil.cs:270-297`
#[must_use]
pub fn statustext(severity: u8, text: &str) -> MavMessage {
    let mut bytes = [0u8; 50];
    for (slot, c) in bytes.iter_mut().zip(text.chars()) {
        // `Encoding.ASCII` writes anything outside ASCII as '?'.
        *slot = u8::try_from(c).ok().filter(u8::is_ascii).unwrap_or(b'?');
    }
    MavMessage::Statustext(mp_mavlink_dialects::all::Statustext {
        severity,
        text: bytes,
        id: 0,
        chunk_seq: 0,
    })
}

/// What `CMB_mountmode` lists: the documented values of the first of `MNT1_DEFLT_MODE`,
/// `MNT_DEFLT_MODE` and `MNT_MODE` that has any, as `FlightData_Load` binds it. A bound list
/// selects its first item.
///
/// The C# binds it once, when the screen loads, from the documentation for the firmware it has
/// then; this reads the documentation each time, so a file fetched for the vehicle since is used.
/// `// C#: GCSViews/FlightData.cs:2718-2729`
#[must_use]
pub fn mount_modes(
    lookup: fn(&str) -> Option<&'static mp_params::ParamMeta>,
) -> Vec<(i64, String)> {
    ["MNT1_DEFLT_MODE", "MNT_DEFLT_MODE", "MNT_MODE"]
        .iter()
        .filter_map(|name| lookup(name))
        .map(|meta| {
            meta.values
                .iter()
                .map(|(key, text)| (*key, text.trim().to_owned()))
                .collect::<Vec<_>>()
        })
        .find(|options| !options.is_empty())
        .unwrap_or_default()
}

/// Set Mount: `MNT_MODE` set to the chosen mode where the vehicle has that parameter, and
/// `DO_MOUNT_CONTROL` with the mode in its seventh parameter where it does not - "copter 3.3
/// acks with an error, but is ok". Both wait for their answer. With nothing chosen, the C#'s
/// `(int) CMB_mountmode.SelectedValue` throws into its `catch`: `Strings.ErrorNoResponse`.
/// `// C#: GCSViews/FlightData.cs:1392-1415`
pub fn mount_mode_sends(
    target: VehicleId,
    parameters: &[(String, f64)],
    chosen: Option<i64>,
) -> Sends {
    let Some(mode) = chosen else {
        return Err(Refusal::error(strings::ERROR_NO_RESPONSE));
    };
    #[allow(clippy::cast_precision_loss)] // a mode number
    let value = mode as f32;
    if parameters.iter().any(|(name, _)| name == "MNT_MODE") {
        return Ok(vec![commands::param_set(target, "MNT_MODE", value)]);
    }
    let command = u16::try_from(MavCmd::MAV_CMD_DO_MOUNT_CONTROL.0).unwrap_or(u16::MAX);
    Ok(vec![commands::command_long(
        target,
        command,
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, value],
    )])
}

/// A `CheckedListBox` item's check.
fn check_glyph(checked: bool) -> &'static str {
    if checked { "\u{2611}" } else { "\u{2610}" }
}

// --- The Gauges page -------------------------------------------------------------------------

/// The speed dial's place on the Gauges page, from where the page was last laid out - the
/// column's width and the page's height before the first.
fn gauge_place(data: &FlightData) -> (f32, f32, f32) {
    let (width, height) = data.gauges_bounds.get().map_or((392.0, 770.0), |laid_out| {
        (
            f32::from(laid_out.size.width),
            f32::from(laid_out.size.height),
        )
    });
    crate::gauge::speed_place(width, height)
}

/// The speed dial's two needles: `Value0` bound to `airspeed` and `Value1` to `groundspeed`.
/// `// C#: GCSViews/FlightData.Designer.cs:1496-1497`
fn gauge_needles(
    gauge: &crate::gauge::SpeedGauge,
    state: &mp_vehicle::VehicleState,
) -> [crate::gauge::Needle; 2] {
    #[allow(clippy::cast_possible_truncation)] // speeds in m/s
    gauge.needles(state.air_speed.0 as f32, state.ground_speed.0 as f32)
}

/// The Gauges page: `Gspeed` where `tabPage1_Resize` puts it on a page this size, and a double
/// click on it asking for its maximum.
/// `// C#: GCSViews/FlightData.cs:3140-3148, 5217-5278`
fn gauges_page(
    data: &FlightData,
    state: Option<&mp_vehicle::VehicleState>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (left, top, side) = gauge_place(data);
    let gauge = data.speed_gauge.clone();
    let needles = state.map(|state| gauge_needles(&gauge, state));
    let bounds = Rc::clone(&data.gauges_bounds);
    div()
        .relative()
        .flex_1()
        .min_h(px(side))
        .child(
            gpui::canvas(
                move |laid_out, _window, _cx| bounds.set(Some(laid_out)),
                |_bounds, (), _window, _cx| {},
            )
            .absolute()
            .inset_0(),
        )
        .child(
            crate::probe::measured("fly-gauge-speed", div())
                .id("fly-gauge-speed")
                .absolute()
                .left(px(left))
                .top(px(top))
                .w(px(side))
                .h(px(side))
                .cursor_pointer()
                .child(
                    gpui::canvas(
                        |_bounds, _window, _cx| (),
                        move |bounds, (), window, cx| {
                            let scene = gauge.scene(side, needles.as_ref().map_or(&[], |n| n));
                            crate::hud::paint(&scene, bounds, window, cx);
                        },
                    )
                    .size_full(),
                )
                // `DoubleClick`, which Windows raises on the second press. The box starts at 60
                // whatever the maximum is. `// C#: GCSViews/FlightData.cs:3142`
                .on_mouse_down(
                    gpui::MouseButton::Left,
                    cx.listener(|this, event: &gpui::MouseDownEvent, window, cx| {
                        if event.click_count == 2 {
                            this.fly_actions.ask(Prompt::GaugeMax, "60");
                            this.fly_focus.prompt.focus(window, cx);
                            cx.notify();
                        }
                    }),
                ),
        )
        .into_any_element()
}

// --- The two context menus under the HUD ---------------------------------------------------------

/// Which of the column's small context menus is open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuKind {
    /// `contextMenuStripactionstab`, on the page strip: Customize and MultiLine.
    /// `// C#: GCSViews/FlightData.Designer.cs:327, 572-594`
    Tabs,
    /// `contextMenuStripQuickView`, on each quick view: Set View Count and Undock.
    /// `// C#: GCSViews/FlightData.Designer.cs:636, 647-660`
    Quick,
}

/// An entry of one of those menus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuEntry {
    /// `customizeToolStripMenuItem`.
    Customize,
    /// `multiLineToolStripMenuItem`.
    MultiLine,
    /// `setViewCountToolStripMenuItem`.
    SetViewCount,
    /// `undockToolStripMenuItem`, dropped: one window.
    Undock,
}

impl MenuEntry {
    /// The entry's text in the `.resx`.
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Customize => "Customize",
            Self::MultiLine => "MultiLine",
            Self::SetViewCount => "Set View Count",
            Self::Undock => "Undock",
        }
    }

    /// The id a script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Customize => "fly-tabs-customize",
            Self::MultiLine => "fly-tabs-multiline",
            Self::SetViewCount => "fly-quick-setviewcount",
            Self::Undock => "fly-quick-undock",
        }
    }

    /// Why the entry is dimmed, where it is.
    #[must_use]
    pub const fn dimmed(self) -> Option<&'static str> {
        match self {
            Self::Undock => Some("one window: nothing to undock from"),
            _ => None,
        }
    }
}

impl MenuKind {
    /// The menu's entries, in the Designer's order.
    #[must_use]
    pub const fn entries(self) -> &'static [MenuEntry] {
        match self {
            Self::Tabs => &[MenuEntry::Customize, MenuEntry::MultiLine],
            Self::Quick => &[MenuEntry::SetViewCount, MenuEntry::Undock],
        }
    }

    /// Its name, for a fact.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Tabs => "tabs",
            Self::Quick => "quick",
        }
    }
}

/// `setQuickViewRowsCols`'s numbers: each `Math.Max(1, int.Parse(text))`. `int.Parse` throws on
/// a number `IsNumber` allowed but is not whole - "2.5" - and nothing is set; the message is
/// .NET's. `// C#: GCSViews/FlightData.cs:4925-4933`
pub fn view_count(cols: &str, rows: &str) -> Result<(i32, i32), &'static str> {
    const FORMAT: &str = "Input string was not in a correct format.";
    let cols = dotnet_int(cols).ok_or(FORMAT)?;
    let rows = dotnet_int(rows).ok_or(FORMAT)?;
    Ok((cols.max(1), rows.max(1)))
}

// --- The handlers for the pages and the map's POI entries -----------------------------------------

impl MissionPlanner {
    /// A conversion's button: its dialog, as a question whose box starts in the folder the
    /// dialog opens in, as Load Log's does. `// C#: GCSViews/FlightData.cs:1084-1089, 1137-1151,
    /// 1313-1317, Log/MatLabForms.cs:45-59`
    fn fly_ask_convert(&mut self, kind: Conversion, window: &mut Window, cx: &mut Context<Self>) {
        let logdir = log_directory();
        let start = kind
            .directory(
                self.fly_data.playback.directory.as_deref(),
                logdir.as_deref(),
            )
            .map_or_else(String::new, |dir| {
                format!("{}{}", dir.display(), std::path::MAIN_SEPARATOR)
            });
        self.fly_actions.ask(Prompt::Convert(kind), &start);
        self.fly_focus.prompt.focus(window, cx);
    }

    /// A conversion, once its log is named: started on a thread of its own. A name that is empty
    /// or only the folder is the dialog closed without a file, which does nothing.
    /// `// C#: GCSViews/FlightData.cs:1091, 1151, 1319, Log/MatLabForms.cs:59`
    fn fly_convert(&mut self, kind: Conversion, text: &str) {
        let path = text.trim();
        if path.is_empty() || std::path::Path::new(path).is_dir() {
            return;
        }
        let analyzer =
            mp_settings::data_directory().map(|dir| mp_log::analysis::analyzer_dir(&dir));
        if self
            .fly_data
            .conversions
            .start(kind, std::path::PathBuf::from(path), analyzer)
        {
            self.file_status = Some(format!("{}: {path}", kind.text()));
        }
    }

    /// Jump To Tag, once a tag is given: `DO_JUMP_TAG` through `doCommand`, which waits for its
    /// answer, and `Strings.CommandFailed` when the vehicle refuses or never answers. A tag that
    /// is not a number from 0 to 65535 is said, and the question asked again, as the C#'s handler
    /// calls itself. `// C#: GCSViews/FlightData.cs:6504-6531`
    fn fly_jump_to_tag(&mut self, text: &str, window: &mut Window, cx: &mut Context<Self>) {
        let Some(tag) = parse_tag(text) else {
            self.file_status = Some(INVALID_TAG.to_owned());
            self.fly_actions.ask(Prompt::JumpToTag, "");
            self.fly_focus.prompt.focus(window, cx);
            return;
        };
        let report = Report::on_failure(error_box(strings::COMMAND_FAILED));
        self.fly_press(&report, |_, target, _| {
            Ok(vec![jump_to_tag_message(target, tag)])
        });
    }

    /// A row of the HUD's menu clicked. Video shows its drop-down, as a click on an entry with
    /// one opens it; the others do what the C#'s handler does, and the menu closes, as a
    /// `ToolStripMenuItem`'s click closes it.
    /// `// C#: GCSViews/FlightData.cs:3185, 4735-4739, 5161-5164, 3122-3139`
    fn fly_hud_menu(&mut self, action: HudAction) -> bool {
        match action {
            HudAction::Video => {
                self.fly_data.hud_menu.video = true;
                return false;
            }
            HudAction::UserItems => self.fly_data.hud_settings.choosing = true,
            HudAction::Russian => self.fly_data.hud_settings.toggle_russian(),
            HudAction::SwapWithMap => self.fly_data.swapped = !self.fly_data.swapped,
            HudAction::GroundColor => self.fly_data.hud_settings.toggle_ground(),
            HudAction::ShowIcons => self.fly_data.hud_settings.toggle_icons(&mut self.persisted),
            // On, a click turns the line off; off, it asks the count, starting at 4 each time.
            // `// C#: GCSViews/FlightData.cs:6115-6128`
            HudAction::BatteryCells => {
                if self.fly_data.hud_settings.cells.take().is_none() {
                    self.fly_actions.ask(Prompt::CellCount, "4");
                    self.fly_data.hud_menu = HudMenu::default();
                    return true;
                }
            }
            // The saved answer, or the C#'s first.
            // `// C#: GCSViews/FlightData.cs:4894-4896, 4815-4817, 3157-3160`
            HudAction::MjpegSource => {
                let url = self
                    .persisted
                    .get("mjpeg_url")
                    .unwrap_or(mp_video::mjpeg::DEFAULT_URL)
                    .to_owned();
                return self.fly_hud_ask(Prompt::MjpegUrl, &url);
            }
            HudAction::GStreamerSource => {
                let url = self
                    .persisted
                    .get("gstreamer_url")
                    .unwrap_or(mp_video::gstreamer::DEFAULT_PIPELINE)
                    .to_owned();
                return self.fly_hud_ask(Prompt::GStreamerUrl, &url);
            }
            HudAction::HereLinkVideo => {
                let ip = herelink_ip(&self.persisted);
                return self.fly_hud_ask(Prompt::HereLinkIp, &ip);
            }
            // `hudGStreamer.Stop()`. `// C#: GCSViews/FlightData.cs:3150-3153`
            HudAction::GStreamerStop => self.fly_data.gstreamer.stop(),
        }
        self.fly_data.hud_menu = HudMenu::default();
        false
    }

    /// A question asked from the HUD's menu, which closes.
    fn fly_hud_ask(&mut self, prompt: Prompt, text: &str) -> bool {
        self.fly_actions.ask(prompt, text);
        self.fly_data.hud_menu = HudMenu::default();
        true
    }

    /// Set MJPEG source, answered ([`mjpeg_source`]).
    fn fly_mjpeg_source(&mut self, accepted: bool, url: &str) {
        if let Some(words) = mjpeg_source(&mut self.fly_data, &mut self.persisted, accepted, url) {
            self.file_status = Some(words);
        }
    }

    /// Set GStreamer Source, answered ([`gstreamer_source`]).
    fn fly_gstreamer_source(&mut self, accepted: bool, pipeline: &str) {
        let said = gstreamer_source(
            &mut self.fly_data,
            &mut self.persisted,
            accepted,
            pipeline,
            &look_for_gstreamer,
        );
        if let Some(words) = said {
            self.file_status = Some(words);
        }
    }

    /// HereLink Video, answered ([`herelink_video`]).
    fn fly_herelink_video(&mut self, accepted: bool, typed: &str) {
        let said = herelink_video(
            &mut self.fly_data,
            &mut self.persisted,
            accepted,
            typed,
            &look_for_gstreamer,
        );
        if let Some(words) = said {
            self.file_status = Some(words);
        }
    }

    /// Once a frame: the GStreamer runtime's download ([`gst_download_tick`]).
    pub(crate) fn hud_video_tick(&mut self) {
        if let Some(words) =
            gst_download_tick(&mut self.fly_data, &mut self.persisted, &look_for_gstreamer)
        {
            self.file_status = Some(words);
        }
    }

    /// Load Log, once a path is given: the link given over to playing it - or, with a port
    /// open, only its name shown, as the C#'s main loop closes the file straight away.
    /// `// C#: GCSViews/FlightData.cs:669-701, 1276-1302, 3439-3453`
    fn fly_load_log(&mut self, path: &str) {
        let path = path.trim();
        if path.is_empty() {
            return;
        }
        if port_open(&self.telemetry.view()) {
            self.fly_data.playback.load_name_only(path);
            return;
        }
        match replay(path) {
            Ok((telemetry, control)) => {
                self.telemetry = telemetry;
                self.fly_data.playback.load(path, control);
            }
            Err(_) => {
                self.file_status = Some(error_box("Please load a valid file"));
            }
        }
    }

    /// Add Poi: `POI.POIAdd(MouseDownStart)` - the ID asked for, then the point added where the
    /// map was last pressed. Before any press the C#'s `MouseDownStart` is null and `POIAdd`
    /// returns at once. `// C#: GCSViews/FlightData.cs:1007-1010, Utilities/POI.cs:70-82`
    fn poi_add(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let Some((at, _)) = self.fly_data.mouse_down_start else {
            return;
        };
        self.fly_data.pois.pending = Some((at.latitude(), at.longitude(), 0.0));
        self.fly_actions.ask(Prompt::PoiId, "");
        self.fly_focus.prompt.focus(window, cx);
    }

    /// Delete: `POI.POIDelete` on the marker the map was pressed on, if it was pressed on one.
    /// The C# deletes the marker under the pointer when its menu opens; the menu here is under
    /// the grid, so it is the marker under the last press.
    /// `// C#: GCSViews/FlightData.cs:2632-2638, Utilities/POI.cs:84-100`
    fn poi_delete(&mut self) {
        let Some((_, press)) = self.fly_data.mouse_down_start else {
            return;
        };
        let drawn: Vec<Option<(f32, f32)>> = {
            let map = self.map.borrow();
            self.fly_data
                .pois
                .points()
                .iter()
                .map(|poi| poi.position().and_then(|at| map.screen_of(at)))
                .collect()
        };
        if let Some(index) = crate::poi::under(&drawn, press) {
            self.fly_data.pois.delete(index);
        }
    }

    /// Coords, once answered: the typed point, then its ID asked for as Add Poi asks.
    /// `// C#: GCSViews/FlightData.cs:6011-6038`
    fn poi_at_coords(&mut self, text: &str, window: &mut Window, cx: &mut Context<Self>) {
        match crate::poi::parse_coords(text) {
            Ok(point) => {
                self.fly_data.pois.pending = Some(point);
                self.fly_actions.ask(Prompt::PoiId, "");
                self.fly_focus.prompt.focus(window, cx);
            }
            Err(why) => self.file_status = Some(error_box(why)),
        }
    }
}

// --- The fourth batch's handlers: the map menu's camera and home entries, the grid's Message,
// --- Set Mount and Clear Track, the POI files, the strip's and quick views' menus, the
// --- Transponder page and the gimbal ---------------------------------------------------------

impl MissionPlanner {
    /// Message: nothing without a link, else the question. `// C#: GCSViews/FlightData.cs:1255-1264`
    fn fly_ask_message(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if !port_open(&self.telemetry.view()) {
            return;
        }
        self.fly_actions.ask(Prompt::SendMessage, "");
        self.fly_focus.prompt.focus(window, cx);
    }

    /// Set Mount: the chosen mode as `MNT_MODE` or `DO_MOUNT_CONTROL`, `Strings.ErrorNoResponse`
    /// from the `catch`. `// C#: GCSViews/FlightData.cs:1392-1415`
    fn fly_set_mount(&mut self) {
        self.fly_actions.mount_open = false;
        let chosen = mount_modes(crate::metadata::lookup)
            .get(self.fly_actions.mount_selected)
            .map(|(key, _)| *key);
        let report = Report::on_timeout(error_box(strings::ERROR_NO_RESPONSE));
        self.fly_press(&report, |_, target, view| {
            mount_mode_sends(target, &view.parameters, chosen)
        });
    }

    /// Clear Track: the route flown so far taken off the map, which records it again from the
    /// vehicle's next position. The C# also empties `MAV.camerapoints`, which nothing here holds.
    /// `// C#: GCSViews/FlightData.cs:1101-1107`
    fn fly_clear_track(&mut self) {
        let mut map = self.map.borrow_mut();
        self.fly_data.track_cleared = Some(map.path_len());
        map.clear_track();
    }

    /// Point Camera Here: "Please Connect First" without a link, else the height asked for,
    /// starting at 0. `// C#: GCSViews/FlightData.cs:4511-4522`
    fn fly_ask_point_camera_here(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if !port_open(&self.telemetry.view()) {
            self.file_status = Some(error_box(PLEASE_CONNECT));
            return;
        }
        self.fly_actions.ask(Prompt::PointCameraAlt, "0");
        self.fly_focus.prompt.focus(window, cx);
    }

    /// Trigger Camera NOW: `setDigicamControl(true)`, as Do Action's `Trigger_Camera` sends it -
    /// `DO_DIGICAM_CONTROL`, and `DIGICAM_CONTROL` if the vehicle refuses it - with
    /// `Strings.CommandFailed` from the `catch`. `// C#: GCSViews/FlightData.cs:5381-5391`
    fn fly_trigger_camera(&mut self) {
        let report = self
            .telemetry
            .send_handle()
            .map(|(_, target)| action_report("Trigger_Camera", target))
            .unwrap_or_default();
        self.fly_press(&report, |_, target, _| {
            action_messages(
                "Trigger_Camera",
                &ActionContext {
                    target,
                    copter: false,
                    motor_outputs_enabled: false,
                    now_unix_usec: 0,
                },
            )
        });
    }

    /// Set EKF Origin Here: nothing without a link; the terrain's height at the point last
    /// pressed, or [`NO_SRTM`]; then `SET_GPS_GLOBAL_ORIGIN`, sent once.
    /// `// C#: GCSViews/FlightData.cs:4789-4811`
    fn fly_set_ekf_origin(&mut self) {
        if !port_open(&self.telemetry.view()) {
            return;
        }
        let (lat, lng) = mouse_down_point(&self.fly_data);
        let answer = crate::srtm::altitude(lat, lng);
        self.fly_press(&Report::default(), |_, target, _| {
            set_ekf_origin_sends(target, (lat, lng), answer)
        });
    }

    /// Set Home Here: nothing without a link; the terrain's height at the point last pressed,
    /// from a tile or the sea, or [`NO_SRTM`]; then "Are you sure?".
    /// `// C#: GCSViews/FlightData.cs:4852-4870`
    fn fly_ask_set_home(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if !port_open(&self.telemetry.view()) {
            return;
        }
        let (lat, lng) = mouse_down_point(&self.fly_data);
        match set_home_height(crate::srtm::altitude(lat, lng)) {
            Ok(alt) => {
                self.fly_data.pending_home = Some((lat, lng, alt));
                self.fly_actions.ask(Prompt::SetHome, "");
                // No box, but OK and closing it are Enter and Escape.
                self.fly_focus.prompt.focus(window, cx);
            }
            Err(refusal) => {
                self.fly_actions.record_nothing(refusal.text());
                self.file_status = Some(error_box(refusal.text()));
            }
        }
    }

    /// Save File or Load File on the POI menu: the dialog, as a question whose box starts in the
    /// folder the POI file is kept in.
    ///
    /// The C#'s dialogs set no starting folder, so they open wherever the platform's dialog last
    /// was; a typed path needs somewhere to start, and this starts it beside `poi.txt`.
    /// `// C#: Utilities/POI.cs:143-182`
    fn poi_ask_file(&mut self, prompt: Prompt, window: &mut Window, cx: &mut Context<Self>) {
        let start = self
            .fly_data
            .pois
            .file()
            .and_then(std::path::Path::parent)
            .map_or_else(String::new, |dir| {
                format!("{}{}", dir.display(), std::path::MAIN_SEPARATOR)
            });
        self.fly_actions.ask(prompt, &start);
        self.fly_focus.prompt.focus(window, cx);
    }

    /// A typed path that names a file: empty or only the folder is the dialog closed without one.
    fn poi_path(text: &str) -> Option<std::path::PathBuf> {
        let path = text.trim();
        (!path.is_empty() && !std::path::Path::new(path).is_dir())
            .then(|| std::path::PathBuf::from(path))
    }

    /// Save File, once named: `SaveFile(sfd.FileName)` - the points as the C# writes `poi.txt`.
    /// The C# says nothing either way, and a failure reaches no handler; both are said here.
    /// `// C#: Utilities/POI.cs:143-168`
    fn poi_save(&mut self, text: &str) {
        let Some(path) = Self::poi_path(text) else {
            return;
        };
        let rendered = crate::poi::render(self.fly_data.pois.points());
        self.file_status = Some(match std::fs::write(&path, rendered) {
            Ok(()) => format!("Save File: {}", path.display()),
            Err(err) => error_box(format!("{}: {err}", path.display())),
        });
    }

    /// Load File, once named: `LoadFile` - each line's point added to the ones there are.
    ///
    /// The C# adds them with `poi.txt`'s saving held off until the load is done, and the next
    /// change saves them all; `Pois::add` saves as it adds, so the file has them straight away.
    /// `// C#: Utilities/POI.cs:171-207`
    fn poi_load(&mut self, text: &str) {
        let Some(path) = Self::poi_path(text) else {
            return;
        };
        match std::fs::read(&path) {
            Ok(bytes) => {
                for poi in crate::poi::parse(&String::from_utf8_lossy(&bytes)) {
                    self.fly_data.pois.add(poi.lat, poi.lng, 0.0, poi.id());
                }
            }
            Err(err) => self.file_status = Some(error_box(format!("{}: {err}", path.display()))),
        }
    }

    /// A row of the strip's or the quick views' menu clicked: the menu closes, as a
    /// `ToolStripMenuItem`'s click closes it, and the entry does what the C#'s handler does.
    fn fly_menu_entry(&mut self, entry: MenuEntry, window: &mut Window, cx: &mut Context<Self>) {
        self.fly_data.menu = None;
        if let Some(why) = entry.dimmed() {
            self.file_status = Some(format!("{} is not ported: {why}", entry.text()));
            return;
        }
        match entry {
            MenuEntry::MultiLine => self.fly_pages.toggle_multiline(),
            // `customForm.ShowDialog()`: the list, over everything, until it is closed.
            MenuEntry::Customize => {
                self.fly_data.customizing = Some(self.fly_pages.customize_list());
            }
            // The columns asked first, from the setting or 2; the rows after, from it or 3.
            // `// C#: GCSViews/FlightData.cs:5078-5099`
            MenuEntry::SetViewCount => {
                let cols = self
                    .fly_data
                    .quick
                    .saved_grid()
                    .map_or_else(|| "2".to_owned(), |(cols, _)| cols.to_string());
                self.fly_actions.ask(Prompt::ViewColumns, &cols);
                self.fly_focus.prompt.focus(window, cx);
            }
            MenuEntry::Undock => {}
        }
    }

    /// Customize's form closed: the checked pages are the strip's.
    /// `// C#: GCSViews/FlightData.cs:2617-2627`
    fn fly_customize_close(&mut self) {
        if let Some(list) = self.fly_data.customizing.take() {
            self.fly_pages.customize(&list);
            self.fly_scroll.set_offset(gpui::point(px(0.0), px(0.0)));
        }
    }

    /// Set View Count, once both are given: `IsNumber` on both, then `setQuickViewRowsCols`,
    /// which resizes the quick views' grid and keeps the numbers as the settings it writes
    /// (`crate::quick::QuickViews::set_rows_cols`). A number `IsNumber` allows that `int.Parse`
    /// refuses throws in the C#; here it is the error box.
    /// `// C#: GCSViews/FlightData.cs:5092-5096, 4914-5060`
    fn fly_view_count(&mut self, cols: &str, rows: &str) {
        if !(is_number(rows) && is_number(cols)) {
            return;
        }
        if let Err(why) = self.fly_data.quick.set_rows_cols(cols, rows) {
            self.file_status = Some(error_box(why));
        }
    }

    /// STBY, ON, ALT or IDENT: the control message, sent once.
    pub(crate) fn fly_xpdr_press(&mut self, button: crate::transponder::Button) {
        let message = self.fly_data.transponder.press(button);
        self.fly_press(&Report::default(), |_, _, _| Ok(vec![message]));
    }

    /// A keystroke in the flight ID box.
    pub(crate) fn fly_xpdr_flight_id(&mut self) {
        let message = self.fly_data.transponder.flight_id_changed();
        self.fly_press(&Report::default(), |_, _, _| Ok(vec![message]));
    }

    /// Enter in the squawk box.
    pub(crate) fn fly_xpdr_squawk_commit(&mut self) {
        if let Some(message) = self.fly_data.transponder.commit_squawk() {
            self.fly_press(&Report::default(), |_, _, _| Ok(vec![message]));
        }
    }

    /// The squawk's wheel, or its up and down buttons.
    pub(crate) fn fly_xpdr_squawk_step(&mut self, up: bool) {
        if let Some(message) = self.fly_data.transponder.step_squawk(up) {
            self.fly_press(&Report::default(), |_, _, _| Ok(vec![message]));
        }
    }

    /// Connect to Transponder: the status asked for through `doCommand`, which waits for its
    /// answer - "Timeout." where none comes - and then up to three seconds for a status. Without
    /// a vehicle `doCommand` returns at once and the three seconds start.
    /// `// C#: GCSViews/FlightData.cs:6348-6365`
    pub(crate) fn fly_xpdr_connect(&mut self) {
        let now = Instant::now();
        let requests = if self.telemetry.send_handle().is_some() {
            self.fly_press(
                &Report::on_timeout(crate::transponder::TIMEOUT),
                |_, target, _| Ok(vec![crate::transponder::set_message_interval(target)]),
            )
        } else {
            Vec::new()
        };
        let request = requests.first().map(|id| (*id, now));
        self.fly_data.transponder.connecting = Some(crate::transponder::Connecting {
            request,
            waiting_since: request.is_none().then_some(now),
        });
    }

    /// Once a frame: Connect's wait, and the main loop's `updateTransponder` - on a status the
    /// page has not shown, or every five seconds. `// C#: GCSViews/FlightData.cs:4314-4318, 6352-6358`
    fn fly_xpdr_tick(&mut self, view: &TelemetryView, window: &Window) {
        let now = Instant::now();
        let status = view.state.as_deref().map(|state| state.transponder);
        let open = port_open(view);
        let focus = (
            self.fly_focus.xpdr.flight_id.is_focused(window),
            self.fly_focus.xpdr.squawk.is_focused(window),
        );
        if let Some(mut connecting) = self.fly_data.transponder.connecting {
            if connecting.waiting_since.is_none()
                && let Some((id, made)) = connecting.request
            {
                match self.telemetry.lookup(id, made) {
                    Lookup::Found(request) => match request.outcome() {
                        // The command's own "Timeout.", said by its report; the wait is over.
                        Some(RequestOutcome::TimedOut) => {
                            self.fly_data.transponder.connecting = None;
                            return;
                        }
                        Some(_) => connecting.waiting_since = Some(now),
                        None => {}
                    },
                    Lookup::PickingUp => {}
                    Lookup::Gone => connecting.waiting_since = Some(now),
                }
            }
            if let Some(since) = connecting.waiting_since {
                // A status since the page last looked, not "one has ever arrived": the count the
                // C#'s xpdr_status_pending flag stands for. `// C#: GCSViews/FlightData.cs:6461-6481`
                let arrived = self.fly_data.transponder.pending(status.as_ref());
                if arrived {
                    self.fly_data.transponder.connecting = None;
                    self.fly_xpdr_update(status.as_ref(), open, focus, now);
                    return;
                }
                if now.duration_since(since) >= crate::transponder::STATUS_WAIT {
                    self.fly_data.transponder.connecting = None;
                    self.file_status = Some(crate::transponder::NO_STATUS.to_owned());
                    return;
                }
            }
            self.fly_data.transponder.connecting = Some(connecting);
        }
        if self.fly_data.transponder.due(status.as_ref(), now) {
            self.fly_xpdr_update(status.as_ref(), open, focus, now);
        }
    }

    /// `updateTransponder`, and the subscription its first status sends.
    fn fly_xpdr_update(
        &mut self,
        status: Option<&mp_vehicle::onboard::Transponder>,
        open: bool,
        focus: (bool, bool),
        now: Instant,
    ) {
        if self.fly_data.transponder.update(status, open, focus, now) {
            self.fly_press(&Report::default(), |_, target, _| {
                Ok(vec![crate::transponder::set_message_interval(target)])
            });
        }
    }

    /// `gimbalTrackbar_Scroll`: the three bars' values, sent once.
    pub(crate) fn fly_gimbal_scroll(&mut self) {
        let Some((_, target)) = self.telemetry.send_handle() else {
            self.fly_send_once(Ok(Vec::new()));
            return;
        };
        let message = self.fly_data.payload.scroll_message(target);
        self.fly_send_once(Ok(vec![message]));
    }

    /// Reset Position: the bars to zero, MAVLink targeting, and the zeros - each sent once.
    pub(crate) fn fly_gimbal_reset(&mut self) {
        let Some((_, target)) = self.telemetry.send_handle() else {
            // The bars go back whatever the link; nothing is sent.
            self.fly_data.payload.reset(VehicleId::new(0, 0));
            self.fly_send_once(Ok(Vec::new()));
            return;
        };
        let messages = self.fly_data.payload.reset(target);
        self.fly_send_once(Ok(messages));
    }
}

/// One row of the strip's or the quick views' menu, as the HUD menu's rows are drawn.
fn menu_row(entry: MenuEntry, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let base = crate::probe::measured(entry.id(), div())
        .id(entry.id())
        .h(px(HUD_MENU_ROW))
        .px_2()
        .flex()
        .items_center()
        .text_xs()
        .child(entry.text())
        .on_click(cx.listener(move |this, _event, window, cx| {
            this.fly_menu_entry(entry, window, cx);
            cx.notify();
        }));
    if entry.dimmed().is_some() {
        base.text_color(rgb(theme::DIM)).into_any_element()
    } else {
        base.text_color(rgb(theme::TEXT))
            .bg(rgb(theme::PANEL))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .into_any_element()
    }
}

/// A press that closes the strip's or the quick views' menu.
fn close_menu(
    cx: &mut Context<MissionPlanner>,
) -> impl Fn(&gpui::MouseDownEvent, &mut Window, &mut gpui::App) + 'static {
    cx.listener(|this, _event: &gpui::MouseDownEvent, _window, cx| {
        this.fly_data.menu = None;
        cx.notify();
    })
}

/// The strip's or the quick views' context menu, where the right button came up, moved in to fit
/// the window. A press anywhere else closes it and goes no further.
fn context_menu(
    (kind, (x, y)): (MenuKind, (f32, f32)),
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let entries = kind.entries();
    #[allow(clippy::cast_precision_loss)] // two rows
    let height = 2.0f32.mul_add(HUD_MENU_PADDING, 2.0) + entries.len() as f32 * HUD_MENU_ROW;
    let left = x.min(f32::from(size.width) - HUD_MENU_WIDTH).max(0.0);
    let top = y.min(f32::from(size.height) - height).max(0.0);
    let rows = entries.iter().map(|entry| menu_row(*entry, cx)).collect();
    let id = match kind {
        MenuKind::Tabs => "fly-tabs-menu",
        MenuKind::Quick => "fly-quick-menu",
    };
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .relative()
                    .w(size.width)
                    .h(size.height)
                    .child(
                        div()
                            .id("fly-menu-backdrop")
                            .absolute()
                            .inset_0()
                            .occlude()
                            .on_mouse_down(gpui::MouseButton::Left, close_menu(cx))
                            .on_mouse_down(gpui::MouseButton::Right, close_menu(cx)),
                    )
                    .child(
                        div()
                            .absolute()
                            .left(px(left))
                            .top(px(top))
                            .child(hud_menu_column(id, rows)),
                    ),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// Customize's form: a `CheckedListBox` of every page's name, `CheckOnClick`, filling a form of
/// its own that applies the list when it is closed - drawn here over the window, as
/// `ShowDialog` shows it. `// C#: GCSViews/FlightData.cs:2584-2627`
fn customize_form(
    list: &[(Page, bool)],
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let mut items = div().flex().flex_col();
    for (index, (page, checked)) in list.iter().enumerate() {
        let id = format!("fly-customize-{}", page.name());
        items = items.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .flex()
                .items_center()
                .gap_1()
                .h(px(18.0))
                .px_1()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(check_glyph(*checked))
                .child(page.name())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    if let Some(list) = &mut this.fly_data.customizing
                        && let Some(item) = list.get_mut(index)
                    {
                        item.1 = !item.1;
                    }
                    cx.notify();
                })),
        );
    }
    let form = crate::probe::measured("fly-customize", div())
        .id("fly-customize")
        .flex()
        .flex_col()
        .gap_2()
        .w(px(300.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .child(div().flex().justify_end().child(action(
            "fly-customize-close",
            "\u{2715}",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.fly_customize_close();
                cx.notify();
            }),
        )))
        .child(items);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("fly-customize-backdrop")
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(form),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    /// A file from the C# tree, when it is checked out here.
    fn csharp(path: &str) -> Option<String> {
        std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../references/missionplanner")
                .join(path),
        )
        .ok()
    }

    /// The members of a C# enum, as `(name, value)`; value is `None` for an enum that lists
    /// names only.
    fn csharp_enum(source: &str, header: &str) -> Vec<(String, Option<i64>)> {
        let Some(start) = source.find(header) else {
            return Vec::new();
        };
        source[start..]
            .lines()
            .skip(1)
            .map(str::trim)
            .skip_while(|line| *line == "{")
            .take_while(|line| !line.starts_with('}'))
            .filter(|line| !line.is_empty() && !line.starts_with("//") && !line.starts_with('['))
            .map(|line| {
                let line = line.trim_end_matches(',').trim();
                match line.split_once('=') {
                    Some((name, value)) => (name.trim().to_owned(), value.trim().parse().ok()),
                    None => (line.to_owned(), None),
                }
            })
            .collect()
    }

    fn long(message: &MavMessage) -> (u16, [f32; 7]) {
        match message {
            MavMessage::CommandLong(m) => (
                m.command,
                [
                    m.param1, m.param2, m.param3, m.param4, m.param5, m.param6, m.param7,
                ],
            ),
            other => panic!("expected a COMMAND_LONG, got {}", other.name()),
        }
    }

    fn context() -> ActionContext {
        ActionContext {
            target: target(),
            copter: true,
            motor_outputs_enabled: true,
            now_unix_usec: 1_700_000_000_000_000,
        }
    }

    fn sent(action: &str) -> Vec<MavMessage> {
        action_messages(action, &context()).unwrap_or_else(|why| panic!("{action}: {why:?}"))
    }

    // --- CMB_action ---------------------------------------------------------------------------

    /// The list is `FlightData.actions`, name for name and in order.
    #[test]
    fn the_action_list_is_the_csharp_enum() {
        let Some(source) = csharp("GCSViews/FlightData.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let names: Vec<String> = csharp_enum(&source, "public enum actions")
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(names, ACTIONS.map(str::to_owned).to_vec());
    }

    /// The generic path finds each command the way `Enum.Parse` over the C#'s own `MAV_CMD` does:
    /// the name upper-cased, then with `DO_START_` in front - and fails where it fails.
    #[test]
    fn the_generic_path_finds_what_enum_parse_finds() {
        let Some(source) = csharp("ExtLibs/Mavlink/Mavlink.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let mav_cmd = csharp_enum(&source, "public enum MAV_CMD: ushort");
        assert!(mav_cmd.len() > 100, "MAV_CMD did not parse");
        let parse = |name: &str| {
            mav_cmd
                .iter()
                .find(|(member, _)| member == name)
                .and_then(|(_, value)| *value)
        };
        let special = [
            "Format_SD_Card",
            "Trigger_Camera",
            "Scripting_cmd_stop_and_restart",
            "Scripting_cmd_stop",
            "System_Time",
            "Terminate_Flight",
            "Preflight_Reboot_Shutdown",
            "HighLatency_Enable",
            "HighLatency_Disable",
            "Toggle_Safety_Switch",
            "Engine_Start",
            "Engine_Stop",
        ];
        for action in ACTIONS.iter().filter(|name| !special.contains(name)) {
            let upper = action.to_uppercase();
            let expected = parse(&upper).or_else(|| parse(&format!("DO_START_{upper}")));
            let ours = action_messages(action, &context())
                .ok()
                .and_then(|messages| messages.first().map(|m| i64::from(long(m).0)));
            assert_eq!(ours, expected, "{action}");
        }
        // The two limits Resume Mission filters with.
        assert_eq!(parse("LAST"), Some(i64::from(MAV_CMD_LAST)));
        assert_eq!(parse("DO_LAST"), Some(i64::from(MAV_CMD_DO_LAST)));
    }

    /// `ADSB_Out_Ident` has no `MAV_CMD` of that name in the C# - its enum says
    /// `DO_ADSB_OUT_IDENT` - so the C# reports the command failed and sends nothing.
    #[test]
    fn adsb_out_ident_fails_as_it_does_in_the_csharp() {
        assert_eq!(
            action_messages("ADSB_Out_Ident", &context()),
            Err(Refusal::Error(strings::COMMAND_FAILED.to_owned()))
        );
    }

    /// Five entries go straight away; the rest ask first.
    #[test]
    fn only_five_actions_go_without_asking() {
        let unasked: Vec<&str> = ACTIONS
            .iter()
            .copied()
            .filter(|action| !needs_confirmation(action))
            .collect();
        assert_eq!(
            unasked,
            [
                "Trigger_Camera",
                "System_Time",
                "Scripting_cmd_stop_and_restart",
                "Scripting_cmd_stop",
                "Format_SD_Card"
            ]
        );
    }

    /// Every entry's message, with the C#'s values.
    #[test]
    fn each_action_sends_the_csharps_message() {
        assert_eq!(
            long(&sent("Loiter_Unlim")[0]),
            (17, [0., 0., 1., 0., 0., 0., 0.])
        );
        assert_eq!(
            long(&sent("Return_To_Launch")[0]),
            (20, [0., 0., 1., 0., 0., 0., 0.])
        );
        assert_eq!(
            long(&sent("Mission_Start")[0]),
            (300, [0., 0., 1., 0., 0., 0., 0.])
        );
        assert_eq!(
            long(&sent("Do_Parachute")[0]),
            (208, [0., 0., 1., 0., 0., 0., 0.])
        );
        // "param1 = 0xff; // batt 1", "param2 = 100; // 100%", "param3 = 0".
        assert_eq!(
            long(&sent("Battery_Reset")[0]),
            (42_651, [255., 100., 0., 0., 0., 0., 0.])
        );
        // Gyro only on a copter; baro either way.
        assert_eq!(
            long(&sent("Preflight_Calibration")[0]),
            (241, [1., 0., 1., 0., 0., 0., 0.])
        );
        let plane = ActionContext {
            copter: false,
            ..context()
        };
        let calibration = action_messages("Preflight_Calibration", &plane).expect("sends");
        assert_eq!(long(&calibration[0]), (241, [0., 0., 1., 0., 0., 0., 0.]));

        assert_eq!(
            long(&sent("Terminate_Flight")[0]),
            (185, [1., 0., 0., 0., 0., 0., 0.])
        );
        assert_eq!(
            long(&sent("HighLatency_Enable")[0]),
            (2600, [1., 0., 0., 0., 0., 0., 0.])
        );
        assert_eq!(long(&sent("HighLatency_Disable")[0]), (2600, [0.; 7]));
        assert_eq!(
            long(&sent("Engine_Start")[0]),
            (223, [1., 0., 0., 0., 0., 0., 0.])
        );
        assert_eq!(long(&sent("Engine_Stop")[0]), (223, [0.; 7]));
        assert_eq!(
            long(&sent("Trigger_Camera")[0]),
            (203, [0., 0., 0., 0., 1., 0., 0.])
        );

        // `doReboot` sends the command twice.
        let reboot = sent("Preflight_Reboot_Shutdown");
        assert_eq!(reboot.len(), 2);
        assert!(
            reboot
                .iter()
                .all(|m| long(m) == (246, [1., 0., 0., 0., 0., 0., 0.]))
        );

        let MavMessage::CommandInt(format) = &sent("Format_SD_Card")[0] else {
            panic!("Format_SD_Card is a COMMAND_INT");
        };
        assert_eq!(
            (format.command, format.param1, format.param2, format.frame),
            (526, 1.0, 1.0, 0)
        );
        for (action, op) in [
            ("Scripting_cmd_stop_and_restart", 3.0),
            ("Scripting_cmd_stop", 2.0),
        ] {
            let MavMessage::CommandInt(scripting) = &sent(action)[0] else {
                panic!("{action} is a COMMAND_INT");
            };
            assert_eq!(
                (scripting.command, scripting.param1),
                (42_701, op),
                "{action}"
            );
        }

        let MavMessage::SystemTime(time) = &sent("System_Time")[0] else {
            panic!("System_Time is a SYSTEM_TIME");
        };
        assert_eq!(time.time_unix_usec, 1_700_000_000_000_000);
        assert_eq!(time.time_boot_ms, 0);
    }

    /// Toggle Safety Switch: `DO_SET_MODE` with base 128 and `SET_MODE` twice, custom mode 1 while
    /// the motor outputs are enabled and 0 while the safety holds them - and nothing to sysid 0.
    #[test]
    fn the_safety_toggle_is_a_set_mode_with_safety_armed() {
        let messages = sent("Toggle_Safety_Switch");
        assert_eq!(messages.len(), 3);
        assert_eq!(long(&messages[0]), (176, [128., 1., 0., 0., 0., 0., 0.]));
        for message in &messages[1..] {
            let MavMessage::SetMode(mode) = message else {
                panic!("expected SET_MODE");
            };
            assert_eq!(
                (mode.base_mode, mode.custom_mode, mode.target_system),
                (128, 1, 1)
            );
        }
        let held = ActionContext {
            motor_outputs_enabled: false,
            ..context()
        };
        let messages = action_messages("Toggle_Safety_Switch", &held).expect("sends");
        assert_eq!(long(&messages[0]).1[1], 0.0);

        let nobody = ActionContext {
            target: VehicleId::new(0, 1),
            ..context()
        };
        assert!(matches!(
            action_messages("Toggle_Safety_Switch", &nobody),
            Err(Refusal::Quiet(_))
        ));
    }

    // --- CMB_setwp ----------------------------------------------------------------------------

    /// "0 (Home)", then 1 to the largest of the totals and the mission - inclusive.
    #[test]
    fn the_waypoint_list_runs_to_the_largest_total() {
        assert_eq!(setwp_items(&[], 0), ["0 (Home)"]);
        let params = vec![("CMD_TOTAL".to_owned(), 5.0), ("OTHER".to_owned(), 50.0)];
        assert_eq!(
            setwp_items(&params, 0),
            ["0 (Home)", "1", "2", "3", "4", "5"]
        );
        // A mission of seven items held (home included) lists 1 to 7, as the C# does.
        assert_eq!(setwp_items(&params, 7).len(), 8);
        let params = vec![("MIS_TOTAL".to_owned(), 9.0), ("WP_TOTAL".to_owned(), 3.0)];
        assert_eq!(
            setwp_items(&params, 2).last().map(String::as_str),
            Some("9")
        );
    }

    /// The selection survives a rebuild that still contains it, rather than becoming the C#'s -1.
    #[test]
    fn a_rebuild_keeps_the_chosen_waypoint() {
        let mut actions = Actions::default();
        actions.refresh_setwp(&[("CMD_TOTAL".to_owned(), 4.0)], 0);
        actions.setwp_selected = 3;
        actions.refresh_setwp(&[("CMD_TOTAL".to_owned(), 6.0)], 0);
        assert_eq!(actions.setwp_selected, 3);
        actions.refresh_setwp(&[], 0);
        assert_eq!(actions.setwp_selected, 0);
    }

    // --- ModifyandSet ---------------------------------------------------------------------------

    /// The Designer's values: 100 in each, shown at its decimal places.
    #[test]
    fn the_number_boxes_start_where_the_designer_puts_them() {
        assert_eq!(ModifyAndSet::speed().field.value(), "100.0");
        assert_eq!(ModifyAndSet::alt().field.value(), "100.0");
        assert_eq!(ModifyAndSet::loiter_rad().field.value(), "100");
    }

    /// Typed text becomes the value inside the range; rubbish reverts to the last good value.
    #[test]
    fn a_number_box_validates_as_a_numericupdown_does() {
        let mut speed = ModifyAndSet::speed();
        speed.field.set("7.26");
        assert_eq!(speed.commit(), 7.26);
        assert_eq!(speed.field.value(), "7.3");
        speed.field.set("5000");
        assert_eq!(speed.commit(), 1000.0, "clamped to Maximum");
        speed.field.set("fast");
        assert_eq!(speed.commit(), 1000.0, "rubbish keeps the last value");
        let mut radius = ModifyAndSet::loiter_rad();
        radius.field.set("-20000");
        assert_eq!(radius.commit(), -10_000.0, "clamped to Minimum");
    }

    // --- Change Alt / Change Speed / Set Loiter Rad ---------------------------------------------

    /// `(int) Value`: 25.9 in the box is 25 m on the wire.
    #[test]
    fn change_alt_drops_the_decimal_as_the_csharp_int_cast_does() {
        let messages = change_alt_sends(target(), 25.9).expect("sends");
        let MavMessage::MissionItem(item) = &messages[0] else {
            panic!("Change Alt is a MISSION_ITEM");
        };
        assert_eq!((item.z, item.current, item.frame), (25.0, 3, 3));
    }

    /// The speed goes as typed - `(float) Value`, no unit conversion.
    #[test]
    fn change_speed_sends_the_box_as_typed() {
        let messages = change_speed_sends(target(), 7.5).expect("sends");
        assert_eq!(long(&messages[0]), (178, [0., 7.5, 0., 0., 0., 0., 0.]));
    }

    /// The first of `LOITER_RAD` and `WP_LOITER_RAD` the vehicle has; nothing when it holds the
    /// value already; nothing, quietly, on a vehicle with neither.
    #[test]
    fn set_loiter_rad_writes_the_first_parameter_the_vehicle_has() {
        let param = |message: &MavMessage| match message {
            MavMessage::ParamSet(set) => {
                (mp_params::decode_param_id(&set.param_id), set.param_value)
            }
            other => panic!("expected PARAM_SET, got {}", other.name()),
        };
        let plane = vec![("WP_LOITER_RAD".to_owned(), 60.0)];
        let messages = loiter_rad_messages(target(), &plane, 80.7).expect("sends");
        assert_eq!(param(&messages[0]), ("WP_LOITER_RAD".to_owned(), 80.0));

        let both = vec![
            ("WP_LOITER_RAD".to_owned(), 60.0),
            ("LOITER_RAD".to_owned(), 60.0),
        ];
        let messages = loiter_rad_messages(target(), &both, -12.7).expect("sends");
        assert_eq!(param(&messages[0]), ("LOITER_RAD".to_owned(), -12.0));

        assert!(matches!(
            loiter_rad_messages(target(), &plane, 60.0),
            Err(Refusal::Quiet(_))
        ));
        let copter = vec![("CIRCLE_RADIUS".to_owned(), 1000.0)];
        assert!(matches!(
            loiter_rad_messages(target(), &copter, 80.0),
            Err(Refusal::Quiet(_))
        ));
    }

    // --- Guided: Fly To Coords, Fly To Here Alt ------------------------------------------------

    fn guided(family: Option<VehicleFamily>, mode: Option<&str>) -> GuidedContext<'_> {
        GuidedContext {
            target: target(),
            family,
            mode,
        }
    }

    /// Not in Guided: Guided first (`DO_SET_MODE` and `SET_MODE` twice), then the target with the
    /// C#'s mask in the operator's frame, and `GuidedMode` updated - frame included.
    #[test]
    fn a_guided_target_asks_for_guided_first() {
        let mut state = GuidedMode::default();
        let messages = set_guided_mode_wp(
            &mut state,
            &guided(Some(VehicleFamily::Copter), Some("Stabilize")),
            (-35.363_261_7, 149.165_23, 20.0),
            commands::FRAME_GLOBAL_TERRAIN_ALT,
        );
        assert_eq!(messages.len(), 4);
        assert_eq!(long(&messages[0]), (176, [1., 4., 0., 0., 0., 0., 0.]));
        let MavMessage::SetPositionTargetGlobalInt(target) = &messages[3] else {
            panic!("the target is a SET_POSITION_TARGET_GLOBAL_INT");
        };
        assert_eq!((target.type_mask, target.coordinate_frame), (0xFDF8, 10));
        assert_eq!(
            state,
            GuidedMode {
                x: -353_632_617,
                y: 1_491_652_300,
                z: 20.0,
                frame: 10
            }
        );

        // Already in Guided: the target alone.
        let messages = set_guided_mode_wp(
            &mut state,
            &guided(Some(VehicleFamily::Copter), Some("Guided")),
            (-35.3, 149.1, 20.0),
            3,
        );
        assert_eq!(messages.len(), 1);
    }

    /// ArduPlane gets a `current` 2 `MISSION_ITEM` instead, and nothing at all goes without a
    /// height or a position.
    #[test]
    fn a_plane_is_sent_a_mission_item_and_zero_sends_nothing() {
        let mut state = GuidedMode::default();
        let messages = set_guided_mode_wp(
            &mut state,
            &guided(Some(VehicleFamily::Plane), Some("Guided")),
            (-35.3, 149.1, 100.0),
            3,
        );
        let MavMessage::MissionItem(item) = &messages[0] else {
            panic!("ArduPlane's guided target is a MISSION_ITEM");
        };
        assert_eq!(item.current, 2);
        for point in [(-35.3, 149.1, 0.0), (0.0, 149.1, 20.0), (-35.3, 0.0, 20.0)] {
            assert!(set_guided_mode_wp(&mut state, &guided(None, None), point, 3).is_empty());
        }
    }

    /// `lat;long;alt` or `lat;long`, parsed as single-precision floats; anything else is Invalid
    /// Field.
    #[test]
    fn fly_to_coords_reads_two_or_three_floats() {
        assert_eq!(
            parse_coords(" -35.363262 ; 149.16523 ; 20 "),
            Ok(Coords::Full {
                latitude: f64::from(-35.363_262_f32),
                longitude: f64::from(149.165_23_f32),
                altitude: 20.0
            })
        );
        assert_eq!(
            parse_coords("-35.36;149.16"),
            Ok(Coords::Position {
                latitude: f64::from(-35.36_f32),
                longitude: f64::from(149.16_f32)
            })
        );
        for bad in ["", "1", "1;2;3;4", "north;east", "1;2;high"] {
            assert_eq!(
                parse_coords(bad),
                Err(Refusal::Error(strings::INVALID_FIELD.to_owned())),
                "{bad:?}"
            );
        }
    }

    /// The frame: `GuidedMode`'s once one was sent, else the remembered one, else relative.
    #[test]
    fn fly_to_coords_takes_its_frame_as_the_csharp_does() {
        assert_eq!(fly_to_coords_frame(&GuidedMode::default(), None), 3);
        assert_eq!(fly_to_coords_frame(&GuidedMode::default(), Some(10)), 10);
        let sent = GuidedMode {
            x: 1,
            y: 1,
            z: 5.0,
            frame: 0,
        };
        assert_eq!(fly_to_coords_frame(&sent, Some(10)), 0);
    }

    /// 10 on a copter, 100 on anything else, unless a height was entered before.
    #[test]
    fn fly_to_here_alt_offers_the_csharps_default() {
        assert_eq!(fly_to_here_alt_default(true, None), "10");
        assert_eq!(fly_to_here_alt_default(false, None), "100");
        assert_eq!(fly_to_here_alt_default(true, Some("35")), "35");
    }

    /// Set Mode by name finds the vehicle's own number for it, or sends nothing.
    #[test]
    fn a_mode_is_found_by_name_in_the_vehicles_list() {
        let copter = set_mode_messages(target(), Some(VehicleFamily::Copter), "AUTO");
        assert_eq!(long(&copter[0]), (176, [1., 3., 0., 0., 0., 0., 0.]));
        let plane = set_mode_messages(target(), Some(VehicleFamily::Plane), "GUIDED");
        assert_eq!(long(&plane[0]).1[1], 15.0);
        assert_eq!(copter.len(), 3, "DO_SET_MODE, then SET_MODE twice");
        assert!(set_mode_messages(target(), None, "GUIDED").is_empty());
    }

    // --- Set Home Alt -----------------------------------------------------------------------------

    /// On: minus the home height, so the shown altitude is above sea level. Off again: zero.
    #[test]
    fn set_home_alt_toggles_between_home_and_sea_level() {
        let offset = toggle_home_alt(0.0, 584.25);
        assert_eq!(offset, -584.25);
        assert!((displayed_altitude(10.0, offset) - 594.25).abs() < 1e-9);
        assert_eq!(toggle_home_alt(offset, 584.25), 0.0);
        assert_eq!(displayed_altitude(10.0, 0.0), 10.0);
    }

    /// Set Home Alt on a scripted vehicle, through the real link: before `HOME_POSITION` the
    /// offset is minus zero, which the next click reads as not set; after it, minus the home
    /// altitude it reported - not the GPS's altitude less its height above home, which here says
    /// otherwise - and every altitude shown is above sea level; again, and it is back to zero.
    /// `// C#: GCSViews/FlightData.cs:1236-1247; ExtLibs/ArduPilot/CurrentState.cs:40, 1568-1582`
    #[test]
    fn set_home_alt_takes_the_home_the_vehicle_reported() {
        use crate::telemetry::scripted::{Vehicle, until};
        use mp_link::ProtocolTimeouts;
        use mp_mavlink_dialects::all::{GlobalPositionInt, HomePosition, MavMessage};

        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        // 600 m above sea level and 10 m above home: the GPS makes home 590 m.
        vehicle.send(&MavMessage::GlobalPositionInt(GlobalPositionInt {
            time_boot_ms: 0,
            lat: -353_632_620,
            lon: 1_491_652_370,
            alt: 600_000,
            relative_alt: 10_000,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: 0,
        }));
        until("a position", || {
            telemetry
                .view()
                .state
                .is_some_and(|state| state.position.is_some())
        });
        // A click with `HomeAlt` still `new PointLatLngAlt().Alt`: minus zero, not set.
        let click = |telemetry: &crate::telemetry::Telemetry| {
            let view = telemetry.view();
            toggle_home_alt(alt_offset_home(&view), home_alt(view.state.as_deref()))
        };
        assert_eq!(home_alt(telemetry.view().state.as_deref()), 0.0);
        let offset = click(&telemetry);
        assert!(offset == 0.0 && offset.is_sign_negative(), "{offset}");
        telemetry.set_alt_offset_home(offset);
        assert_eq!(click(&telemetry), 0.0, "minus zero reads as not set");

        vehicle.send(&MavMessage::HomePosition(HomePosition {
            latitude: -353_632_620,
            longitude: 1_491_652_370,
            altitude: 584_090,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            q: [1.0, 0.0, 0.0, 0.0],
            approach_x: 0.0,
            approach_y: 0.0,
            approach_z: 0.0,
            time_usec: 0,
        }));
        until("home", || {
            telemetry
                .view()
                .state
                .is_some_and(|state| state.home.is_some())
        });
        assert!((home_alt(telemetry.view().state.as_deref()) - 584.09).abs() < 1e-9);
        let offset = click(&telemetry);
        assert_eq!(offset, -584.09_f32);
        telemetry.set_alt_offset_home(offset);
        until("the offset on the state", || {
            alt_offset_home(&telemetry.view()) == offset
        });
        let relative = telemetry
            .view()
            .state
            .map_or(0.0, |state| state.altitude_relative.0);
        assert!((displayed_altitude(relative, offset) - 594.09).abs() < 1e-3);

        let offset = click(&telemetry);
        assert_eq!(offset, 0.0);
        telemetry.set_alt_offset_home(offset);
        until("the offset back to zero", || {
            alt_offset_home(&telemetry.view()) == 0.0
        });
    }

    /// `tests/gui/fly-homealt.gui` asks the vehicle for home before it presses Set Home Alt - a
    /// SITL whose home was set before the run sends none, and `HomeAlt` is 0 until it does - and
    /// shows `HomeAlt` in a quick view the chooser offers.
    #[test]
    fn the_home_alt_script_asks_for_home_first() {
        let script = include_str!("../../../tests/gui/fly-homealt.gui");
        let lines: Vec<&str> = script
            .lines()
            .filter(|line| !line.starts_with('#'))
            .collect();
        let at = |wanted: &str| {
            lines
                .iter()
                .position(|line| line.trim() == wanted)
                .unwrap_or_else(|| panic!("the script has no `{wanted}`"))
        };
        let asked = at("expect fly.sent COMMAND_LONG MAV_CMD_GET_HOME_POSITION 0,0,0,0,0,0,0");
        assert!(at("click fly-prompt-cancel") < asked);
        assert!(asked < at("click fly-homealt"));
        let choices = crate::quick::choices();
        for line in &lines {
            if let Some(name) = line.trim().strip_prefix("click fly-quick-choice-") {
                assert!(choices.contains(&name), "{name} is not offered");
            }
        }
        assert!(choices.contains(&"HomeAlt"));
        at("click fly-quick-choice-HomeAlt");
    }

    /// SITL on 5763 answers `getHomePositionAsync`'s `GET_HOME_POSITION` with `HOME_POSITION`,
    /// and Set Home Alt takes the altitude in it. SITL sends `HOME_POSITION` of its own accord
    /// only when home is set - on its first fix, or on arming - so a link that joins a SITL whose
    /// home is already set hears none until it asks, as Mission Planner's does: until then
    /// `HomeAlt` is 0. Run with `--ignored` while SITL is up; it only asks.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3343-3386, 5701-5707`
    #[test]
    #[ignore = "needs SITL on tcp:127.0.0.1:5763"]
    fn sitl_reports_the_home_set_home_alt_takes() {
        use std::time::{Duration, Instant};
        let link = mp_link::Link::connect("tcp:127.0.0.1:5763", mp_link::LinkConfig::default())
            .expect("SITL on 5763");
        let deadline = Instant::now() + Duration::from_secs(10);
        let (id, handle) = loop {
            if let Some(vehicle) = link.primary_vehicle() {
                break vehicle;
            }
            assert!(Instant::now() < deadline, "no vehicle on 5763");
            std::thread::sleep(Duration::from_millis(50));
        };
        let unasked = handle.load();
        eprintln!(
            "before asking: home {:?}, HomeAlt {} m",
            unasked.home,
            home_alt(Some(&unasked))
        );
        // `getHomePositionAsync`: asked, and again every 700 ms, three more times.
        let mut state = None;
        for _ in 0..4 {
            link.sender().send(&get_home_position(id));
            let asked = Instant::now();
            while asked.elapsed() < Duration::from_millis(700) {
                let now = handle.load();
                if now.home.is_some() {
                    state = Some(now);
                    break;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
            if state.is_some() {
                break;
            }
        }
        let state = state.expect("SITL answers GET_HOME_POSITION with HOME_POSITION");
        let home = home_alt(Some(&state));
        eprintln!(
            "HomeAlt {home} m; GPS less relative {} m",
            state.altitude_msl.0 - state.altitude_relative.0
        );
        assert!(home != 0.0);
        #[allow(clippy::cast_possible_truncation)]
        let expected = -home as f32;
        assert_eq!(toggle_home_alt(0.0, home), expected);
    }

    // --- Resume Mission -------------------------------------------------------------------------

    fn item(seq: u16, command: u16, z: f64) -> MissionItem {
        MissionItem {
            seq,
            command,
            z,
            current: u8::from(seq == 0),
            autocontinue: 0,
            ..MissionItem::default()
        }
    }

    fn mission() -> Vec<MissionItem> {
        vec![
            item(0, 16, 0.0),   // home
            item(1, 22, 20.0),  // takeoff: kept
            item(2, 16, 30.0),  // waypoint before the resume point: skipped
            item(3, 178, 0.0),  // DO_CHANGE_SPEED: kept
            item(4, 112, 0.0),  // CONDITION_DELAY: between LAST and DO_LAST, kept
            item(5, 2000, 0.0), // DO_IMAGE_START_CAPTURE: past DO_LAST, skipped
            item(6, 16, 40.0),  // the resume point
            item(7, 16, 50.0),  // after it: kept whatever it is
            item(8, 2000, 0.0),
        ]
    }

    /// Before the resume point only home, takeoffs and 95-240 survive; everything from it on
    /// does; the result is renumbered, not current, and auto-continuing.
    #[test]
    fn resume_keeps_the_do_commands_it_skips_past() {
        let kept = resume_items(&mission(), 6);
        let commands: Vec<u16> = kept.iter().map(|item| item.command).collect();
        assert_eq!(commands, [16, 22, 178, 112, 16, 16, 2000]);
        assert!(
            kept.iter()
                .enumerate()
                .all(|(index, item)| usize::from(item.seq) == index)
        );
        assert!(
            kept.iter()
                .all(|item| item.current == 0 && item.autocontinue == 1)
        );
    }

    struct Vehicle {
        now: Instant,
        transfer: Option<(bool, bool, &'static str)>,
        mission: Vec<MissionItem>,
        mode: &'static str,
        armed: bool,
        altitude: f64,
        /// The last step's request, as the link would report it.
        request: Option<RequestState>,
    }

    impl Vehicle {
        fn new() -> Self {
            Self {
                now: Instant::now(),
                transfer: None,
                mission: mission(),
                mode: "Stabilize",
                armed: false,
                altitude: 0.0,
                request: None,
            }
        }

        fn input(&self) -> ResumeInput<'_> {
            ResumeInput {
                now: self.now,
                transfer: self.transfer,
                mission: &self.mission,
                mode: Some(self.mode),
                armed: self.armed,
                altitude: self.altitude,
                family: Some(VehicleFamily::Copter),
                target: Some(target()),
                request: self.request,
            }
        }

        fn later(&mut self, millis: u64) {
            self.now += Duration::from_millis(millis);
        }
    }

    fn sends(steps: &[ResumeStep]) -> Vec<(u16, [f32; 7])> {
        steps
            .iter()
            .flat_map(|step| match step {
                ResumeStep::Send(messages) => messages
                    .iter()
                    .filter(|m| matches!(m, MavMessage::CommandLong(_)))
                    .map(long)
                    .collect(),
                _ => Vec::new(),
            })
            .collect()
    }

    /// The whole sequence against a vehicle that does as it is told: download, trimmed upload,
    /// read back, waypoint 1 current, Guided, armed, climbed to within 2 m, Auto.
    #[test]
    fn resume_runs_the_csharps_sequence() {
        let mut vehicle = Vehicle::new();
        // Last time's transfer, finished, is still on show.
        vehicle.transfer = Some((true, false, "mission transferred"));
        let (mut resume, steps) = Resume::start(6, vehicle.now);
        assert_eq!(steps, [ResumeStep::DownloadMission]);

        // Not taken as this step's until it has had time to be replaced.
        assert!(resume.advance(&vehicle.input()).is_empty());
        vehicle.transfer = Some((false, false, "reading item 3 of 9"));
        assert!(resume.advance(&vehicle.input()).is_empty());
        vehicle.transfer = Some((true, false, "mission transferred"));
        let steps = resume.advance(&vehicle.input());
        let [ResumeStep::UploadMission(items)] = steps.as_slice() else {
            panic!("expected the trimmed upload, got {steps:?}");
        };
        assert_eq!(items.len(), 7);
        assert_eq!(resume.phase(), &ResumePhase::Upload);

        // A quick upload that finished between frames is accepted once it is old news.
        assert!(resume.advance(&vehicle.input()).is_empty());
        vehicle.later(1100);
        assert_eq!(resume.advance(&vehicle.input()), [ResumeStep::ReadIntoPlan]);

        vehicle.later(1100);
        let steps = resume.advance(&vehicle.input());
        let [ResumeStep::Send(set_current)] = steps.as_slice() else {
            panic!("expected MISSION_SET_CURRENT, got {steps:?}");
        };
        assert!(matches!(&set_current[0], MavMessage::MissionSetCurrent(m) if m.seq == 1));
        assert_eq!(resume.phase(), &ResumePhase::Guided);

        // Guided asked for at once, then not again inside the second.
        let steps = resume.advance(&vehicle.input());
        assert_eq!(sends(&steps)[0], (176, [1., 4., 0., 0., 0., 0., 0.]));
        vehicle.later(500);
        assert!(resume.advance(&vehicle.input()).is_empty());
        vehicle.later(600);
        vehicle.mode = "Guided";
        // In Guided: straight on to arming.
        let steps = resume.advance(&vehicle.input());
        assert_eq!(sends(&steps), [(400, [1., 0., 0., 0., 0., 0., 0.])]);

        vehicle.later(1000);
        vehicle.armed = true;
        let steps = resume.advance(&vehicle.input());
        assert_eq!(sends(&steps), [(22, [0., 0., 0., 0., 0., 0., 40.])]);

        // Within 2 m of the resume waypoint's 40 m is high enough.
        vehicle.later(1000);
        vehicle.altitude = 38.5;
        let steps = resume.advance(&vehicle.input());
        assert_eq!(sends(&steps)[0], (176, [1., 3., 0., 0., 0., 0., 0.]));

        vehicle.later(1000);
        vehicle.mode = "Auto";
        assert!(resume.advance(&vehicle.input()).is_empty());
        assert_eq!(resume.phase(), &ResumePhase::Done);
        assert!(resume.finished());
    }

    /// A vehicle that never changes mode is asked 31 times, a second apart, then given up on.
    #[test]
    fn resume_gives_up_after_the_csharps_thirty() {
        let mut vehicle = Vehicle::new();
        let (mut resume, _) = Resume::start(6, vehicle.now);
        vehicle.transfer = Some((false, false, ""));
        resume.advance(&vehicle.input());
        for _ in 0..3 {
            vehicle.transfer = Some((true, false, ""));
            vehicle.later(1100);
            resume.advance(&vehicle.input());
        }
        assert_eq!(resume.phase(), &ResumePhase::Guided);
        let mut asked = 0;
        for _ in 0..40 {
            if !sends(&resume.advance(&vehicle.input())).is_empty() {
                asked += 1;
            }
            vehicle.later(1000);
        }
        assert_eq!(asked, 31);
        assert_eq!(
            resume.phase(),
            &ResumePhase::Failed(strings::ERROR_NO_RESPONSE.to_owned())
        );
    }

    /// A waypoint the vehicle does not have is the C#'s failed `getWP`; a refused takeoff is its
    /// `doCommand` returning false.
    #[test]
    fn resume_fails_where_the_csharp_fails() {
        let mut vehicle = Vehicle::new();
        let (mut resume, _) = Resume::start(30, vehicle.now);
        vehicle.transfer = Some((false, false, ""));
        resume.advance(&vehicle.input());
        vehicle.transfer = Some((true, false, ""));
        resume.advance(&vehicle.input());
        assert_eq!(
            resume.phase(),
            &ResumePhase::Failed(strings::COMMAND_FAILED.to_owned())
        );

        let mut vehicle = Vehicle::new();
        vehicle.mode = "Guided";
        vehicle.armed = true;
        let (mut resume, _) = Resume::start(6, vehicle.now);
        vehicle.transfer = Some((false, false, ""));
        resume.advance(&vehicle.input());
        for _ in 0..3 {
            vehicle.transfer = Some((true, false, ""));
            vehicle.later(1100);
            resume.advance(&vehicle.input());
        }
        // Already in Guided and armed: straight through to the first takeoff.
        resume.advance(&vehicle.input());
        assert_eq!(resume.phase(), &ResumePhase::Takeoff);
        vehicle.request = Some(RequestState::Finished(RequestOutcome::Rejected(4)));
        resume.advance(&vehicle.input());
        assert_eq!(
            resume.phase(),
            &ResumePhase::Failed(strings::COMMAND_FAILED.to_owned())
        );
    }

    /// A Resume Mission run to the frame it asks for waypoint 1 to be made current.
    fn at_set_current(vehicle: &mut Vehicle) -> Resume {
        let (mut resume, _) = Resume::start(6, vehicle.now);
        vehicle.transfer = Some((false, false, ""));
        resume.advance(&vehicle.input());
        let mut last = Vec::new();
        for _ in 0..3 {
            vehicle.transfer = Some((true, false, ""));
            vehicle.later(1100);
            last = resume.advance(&vehicle.input());
        }
        let [ResumeStep::Send(set_current)] = last.as_slice() else {
            panic!("expected MISSION_SET_CURRENT, got {last:?}");
        };
        assert_eq!(
            route(&set_current[0]),
            Route::SetCurrent {
                target: target(),
                seq: 1
            }
        );
        resume
    }

    /// `setWPCurrent`, `doARM` and `doCommand` block the C#: nothing more is sent while the link
    /// is still retrying the call, and the second before the next attempt starts when it ends.
    /// `// C#: GCSViews/FlightData.cs:1553-1597`
    #[test]
    fn resume_waits_inside_each_call_it_blocks_on() {
        let mut vehicle = Vehicle::new();
        let mut resume = at_set_current(&mut vehicle);

        // Still asking for waypoint 1: Guided is not asked for, however long it takes.
        vehicle.request = Some(RequestState::Waiting);
        vehicle.later(9000);
        assert!(resume.advance(&vehicle.input()).is_empty());
        // Answered: Guided at once.
        vehicle.request = Some(RequestState::Finished(RequestOutcome::Accepted {
            value: None,
        }));
        let steps = resume.advance(&vehicle.input());
        assert_eq!(sends(&steps)[0].0, commands::CMD_DO_SET_MODE);

        // The mode step makes no request; in Guided, arming is asked for.
        vehicle.request = None;
        vehicle.mode = "Guided";
        vehicle.later(1000);
        let arm = (400, [1., 0., 0., 0., 0., 0., 0.]);
        assert_eq!(sends(&resume.advance(&vehicle.input())), [arm]);

        // `doARM` waiting ten seconds a try for its answer: not asked again meanwhile.
        vehicle.request = Some(RequestState::Waiting);
        for _ in 0..5 {
            vehicle.later(1000);
            assert!(resume.advance(&vehicle.input()).is_empty());
        }
        // Refused - its answer is not looked at - so asked again, a second after it returned.
        vehicle.request = Some(RequestState::Finished(RequestOutcome::Rejected(4)));
        assert!(resume.advance(&vehicle.input()).is_empty());
        assert_eq!(resume.phase(), &ResumePhase::Arm);
        vehicle.later(500);
        assert!(resume.advance(&vehicle.input()).is_empty());
        vehicle.later(500);
        assert_eq!(sends(&resume.advance(&vehicle.input())), [arm]);
    }

    /// Every retry unanswered throws in the C#, and the outer `catch` says `CommandFailed`.
    /// `// C#: GCSViews/FlightData.cs:1624-1627`
    #[test]
    fn resume_fails_when_a_call_it_blocks_on_times_out() {
        let mut vehicle = Vehicle::new();
        let mut resume = at_set_current(&mut vehicle);
        vehicle.request = Some(RequestState::Finished(RequestOutcome::TimedOut));
        assert!(resume.advance(&vehicle.input()).is_empty());
        assert_eq!(
            resume.phase(),
            &ResumePhase::Failed(strings::COMMAND_FAILED.to_owned())
        );

        let mut vehicle = Vehicle::new();
        vehicle.mode = "Guided";
        let mut resume = at_set_current(&mut vehicle);
        let steps = resume.advance(&vehicle.input());
        assert_eq!(sends(&steps), [(400, [1., 0., 0., 0., 0., 0., 0.])]);
        vehicle.request = Some(RequestState::Finished(RequestOutcome::TimedOut));
        vehicle.later(40_000);
        resume.advance(&vehicle.input());
        assert_eq!(
            resume.phase(),
            &ResumePhase::Failed(strings::COMMAND_FAILED.to_owned())
        );
    }

    // --- Resume Mission against ArduCopter, as tests/gui/fly-resumemis.gui flies it ------------

    /// The mission `tests/gui/fly-resumemis.gui` writes to the vehicle before it starts: home, a
    /// take-off to 10 m, a waypoint, `DO_CHANGE_SPEED` 5, a waypoint at 10 m.
    fn the_scripts_mission() -> Vec<MissionItem> {
        let item = |seq: u16, frame: u8, command: u16, param2: f64, z: f64| MissionItem {
            seq,
            current: u8::from(seq == 0),
            frame,
            command,
            param2,
            z,
            autocontinue: 1,
            ..MissionItem::default()
        };
        vec![
            item(0, 0, 16, 0.0, 584.09),
            item(1, 3, 22, 0.0, 10.0),
            item(2, 3, 16, 0.0, 10.0),
            item(3, 3, 178, 5.0, 0.0),
            item(4, 3, 16, 0.0, 10.0),
        ]
    }

    /// `WPNAV_SPEED_UP`'s default, 250 cm/s: how fast ArduCopter's Guided take-off climbs.
    const CLIMB_METRES_PER_SECOND: f64 = 2.5;

    /// What one Resume Mission left behind, as the facts the script reads would have it.
    struct Flown {
        resume: Resume,
        mode: &'static str,
        armed: bool,
        /// Each `NAV_TAKEOFF` sent, and whether it was accepted.
        takeoffs: Vec<bool>,
        /// `fly.sent`: the last send, as `Actions::record` describes it.
        sent: String,
        /// `fly.ack`: the answer to the last acknowledged command sent, as the link logs it.
        ack: String,
        /// `mission.items`: the planner's rows, home not among them.
        rows: usize,
    }

    /// Resume Mission at waypoint 4 of the script's mission, a frame every 50 ms, against a
    /// copter that answers as ArduCopter does. `Mode::do_user_takeoff_U_m` refuses a take-off
    /// once the vehicle is no longer landed ("can't takeoff again!"), and the take-off lets go
    /// of landed as the climb starts - `leaves_ground` after the take-off is accepted: the
    /// motors' half-second spool, then a tenth of the climb rate (`ArduCopter/takeoff.cpp:18-47`,
    /// `_AutoTakeoff::run`). With `refuses_airborne` false it is a vehicle that takes every
    /// take-off it is sent.
    fn resume_on_arducopter(leaves_ground: Duration, refuses_airborne: bool) -> Flown {
        let mut vehicle = Vehicle::new();
        vehicle.mission = Vec::new();
        let mut on_board = the_scripts_mission();
        let mut takeoff_at: Option<Instant> = None;
        let mut transferring = false;
        let (resume, mut steps) = Resume::start(4, vehicle.now);
        let mut flown = Flown {
            resume,
            mode: "",
            armed: false,
            takeoffs: Vec::new(),
            sent: String::new(),
            ack: "none".to_owned(),
            rows: 0,
        };
        let accepted = RequestState::Finished(RequestOutcome::Accepted { value: None });
        for _ in 0..1200 {
            for step in steps {
                match step {
                    ResumeStep::DownloadMission | ResumeStep::ReadIntoPlan => {
                        vehicle.mission.clone_from(&on_board);
                        vehicle.transfer = Some((false, false, "reading"));
                        transferring = true;
                    }
                    ResumeStep::UploadMission(items) => {
                        on_board = items;
                        vehicle.transfer = Some((false, false, "writing"));
                        transferring = true;
                    }
                    ResumeStep::Send(messages) => {
                        flown.sent = messages.iter().map(describe).collect::<Vec<_>>().join("; ");
                        vehicle.request = None;
                        for message in &messages {
                            let landed = takeoff_at.is_none_or(|at| {
                                vehicle.now.saturating_duration_since(at) < leaves_ground
                            });
                            match (route(message), message) {
                                (Route::SetCurrent { .. }, _) => vehicle.request = Some(accepted),
                                (Route::Command { command: 400, .. }, _) => {
                                    vehicle.armed = true;
                                    flown.ack = "MAV_CMD_COMPONENT_ARM_DISARM: accepted".to_owned();
                                    vehicle.request = Some(accepted);
                                }
                                (Route::Command { command: 22, .. }, _) => {
                                    let accept = landed || !refuses_airborne;
                                    flown.takeoffs.push(accept);
                                    let result = if accept { 0 } else { 4 };
                                    flown.ack = format!(
                                        "MAV_CMD_NAV_TAKEOFF: {}",
                                        mp_link::messages::command_result_name(result)
                                    );
                                    vehicle.request = Some(if accept {
                                        takeoff_at.get_or_insert(vehicle.now);
                                        accepted
                                    } else {
                                        RequestState::Finished(RequestOutcome::Rejected(result))
                                    });
                                }
                                // `DO_SET_MODE` goes unacknowledged: no request, and the ack on
                                // show is still the last command's.
                                (Route::Raw, MavMessage::CommandLong(m))
                                    if m.command == commands::CMD_DO_SET_MODE =>
                                {
                                    vehicle.mode = match m.param2 {
                                        4.0 => "Guided",
                                        3.0 => "Auto",
                                        other => panic!("mode {other} was not asked for"),
                                    };
                                }
                                (Route::Raw, _) => {}
                                (other, _) => panic!("Resume Mission does not send {other:?}"),
                            }
                        }
                    }
                }
            }
            if flown.resume.finished() {
                break;
            }
            vehicle.later(50);
            // A transfer is seen in progress for a frame, then finished.
            if transferring {
                transferring = false;
            } else if vehicle.transfer.is_some() {
                vehicle.transfer = Some((true, false, "mission transferred"));
            }
            if let Some(at) = takeoff_at {
                let climbing = vehicle
                    .now
                    .saturating_duration_since(at)
                    .saturating_sub(leaves_ground);
                vehicle.altitude = (climbing.as_secs_f64() * CLIMB_METRES_PER_SECOND).min(10.0);
            }
            steps = flown.resume.advance(&vehicle.input());
        }
        flown.mode = vehicle.mode;
        flown.armed = vehicle.armed;
        flown.rows = vehicle.mission.len().saturating_sub(1);
        flown
    }

    /// Resume Mission on ArduCopter ends in `Strings.CommandFailed`, however quickly or slowly
    /// the vehicle leaves the ground. The C# asks for the take-off again every second until the
    /// vehicle is within 2 m of the waypoint's height, and `doCommand` returning false for any
    /// of them is "Command Failed" and the end of it; ArduCopter refuses every take-off once
    /// the vehicle has left the ground, and a climb to 8 m at 2.5 m/s outlasts three of the C#'s
    /// seconds, so a repeat is always refused. The vehicle, armed in Guided, goes on climbing to
    /// the first take-off's 10 m, and Auto is never asked for. A vehicle that takes every
    /// take-off is flown into Auto by the same sequence: the sequence is the C#'s, the refusal
    /// the firmware's.
    /// `// C#: GCSViews/FlightData.cs:1587-1621`
    #[test]
    fn resume_on_arducopter_ends_in_the_csharps_command_failed() {
        for millis in [300, 600, 900, 1200, 1500, 1900, 2500] {
            let flown = resume_on_arducopter(Duration::from_millis(millis), true);
            let ctx = format!("leaving the ground {millis} ms after the take-off");
            assert_eq!(
                flown.resume.phase(),
                &ResumePhase::Failed(strings::COMMAND_FAILED.to_owned()),
                "{ctx}"
            );
            let (last, before) = flown.takeoffs.split_last().expect("a take-off");
            assert!(
                !last && before.iter().all(|&accepted| accepted),
                "{ctx}: {:?}",
                flown.takeoffs
            );
            assert!(
                (2..=4).contains(&flown.takeoffs.len()),
                "{ctx}: {:?}",
                flown.takeoffs
            );
            assert_eq!((flown.mode, flown.armed), ("Guided", true), "{ctx}");
        }

        let flown = resume_on_arducopter(Duration::from_millis(600), false);
        assert_eq!(flown.resume.phase(), &ResumePhase::Done);
        assert_eq!(flown.mode, "Auto");
        assert!(flown.takeoffs.len() > 3, "{:?}", flown.takeoffs);
    }

    /// `tests/gui/fly-resumemis.gui` asserts, once the resume has run, what the C# leaves on
    /// ArduCopter and nothing that cannot hold. Every `expect` between the waypoint's OK and the
    /// Land click is checked against [`resume_on_arducopter`], however early or late the
    /// vehicle leaves the ground, by `tools/gui-test.sh`'s rules: `~` is containment, otherwise
    /// equality. Its runs of 2026-09-24 16:36 and 16:38 said `fly.resume` was
    /// 'failed: The Command failed to execute', `fly.sent` the take-off to 10 m and `fly.ack`
    /// 'MAV_CMD_NAV_TAKEOFF: failed' - what the model says.
    #[test]
    fn the_resume_script_expects_what_the_csharp_leaves_on_arducopter() {
        let script = include_str!("../../../tests/gui/fly-resumemis.gui");
        let lines: Vec<&str> = script
            .lines()
            .map(|line| line.split('#').next().unwrap_or("").trim())
            .filter(|line| !line.is_empty())
            .collect();
        let asked = lines
            .iter()
            .position(|line| line.starts_with("expect fly.prompt ~ Resume mission at waypoint"))
            .expect("the script answers the waypoint question");
        let from = asked
            + lines[asked..]
                .iter()
                .position(|line| *line == "click fly-prompt-ok")
                .expect("the script accepts the waypoint");
        let to = from
            + lines[from..]
                .iter()
                .position(|line| *line == "click land")
                .expect("the script lands afterwards");
        let expects: Vec<&str> = lines[from..to]
            .iter()
            .filter_map(|line| line.strip_prefix("expect "))
            .collect();
        assert!(
            expects.iter().any(|line| line.starts_with("fly.resume ")),
            "the script says where the resume got to"
        );
        for millis in [300, 900, 1500, 2500] {
            let flown = resume_on_arducopter(Duration::from_millis(millis), true);
            let fact = |key: &str| match key {
                "fly.resume" => flown.resume.label(),
                "fly.sent" => flown.sent.clone(),
                "fly.ack" => flown.ack.clone(),
                "vehicle.mode" => flown.mode.to_owned(),
                "vehicle.armed" => flown.armed.to_string(),
                "mission.items" => flown.rows.to_string(),
                other => panic!("the model does not know `{other}`"),
            };
            for line in &expects {
                let (key, rest) = line.split_once(' ').expect("a key and a value");
                let got = fact(key);
                let holds = match rest.strip_prefix("~ ") {
                    Some(wanted) => got.contains(wanted),
                    None => got == rest,
                };
                assert!(
                    holds,
                    "`expect {line}` does not hold: {key} is '{got}' (leaving the ground \
                     {millis} ms after the take-off)"
                );
            }
        }
    }

    // --- Which way each message goes ----------------------------------------------------------

    /// The calls the C# blocks on become the link's retrying requests; what it sends and forgets
    /// goes once.
    /// TakeOff's answer: `float.Parse` of it, and Guided's messages to go before the take-off -
    /// none for a vehicle whose family has no Guided; a word that is not a number is the
    /// `FormatException`'s message. Its box is the C#'s.
    /// `// C#: GCSViews/FlightData.cs:5294-5305`
    #[test]
    fn takeoff_parses_its_height_and_puts_the_vehicle_in_guided_first() {
        let t = target();
        let copter = Some(VehicleFamily::Copter);
        let (altitude, guided) = takeoff_plan("5", t, copter).expect("a height");
        assert!((altitude - 5.0).abs() < f32::EPSILON);
        assert_eq!(guided, set_mode_messages(t, copter, "GUIDED"));
        assert_eq!(guided.len(), 3, "DO_SET_MODE, then SET_MODE twice");
        let (altitude, _) = takeoff_plan("12.5", t, copter).expect("a height");
        assert!((altitude - 12.5).abs() < f32::EPSILON);
        assert!(matches!(
            takeoff_plan("ten", t, copter),
            Err(why) if why == crate::plan::FORMAT_EXCEPTION
        ));
        let (_, none) = takeoff_plan("5", t, None).expect("a height");
        assert!(none.is_empty());
        assert_eq!(Prompt::TakeOff.title(), "Enter Alt");
        assert_eq!(Prompt::TakeOff.text(), "Enter Takeoff Alt");
        assert!(Prompt::TakeOff.takes_text());
        assert_eq!(TAKEOFF_ALT_DEFAULT, "5");
    }

    /// TakeOff on the wire, as the window's press sends it (`send_routed`): Guided's `DO_SET_MODE`
    /// and two `SET_MODE`s straight out, then the take-off as the link's request - and the vehicle
    /// hears them in that order, the request going out only on the link's next pass. ArduCopter refuses a take-off
    /// outside Guided, which is what the owner met on 2026-09-26 with the button that sent the
    /// take-off alone.
    #[test]
    fn takeoff_reaches_the_vehicle_after_guided() {
        use crate::telemetry::scripted::{VEHICLE, Vehicle, until};
        use mp_mavlink_dialects::all::MavMessage;
        let (mut telemetry, mut vehicle) =
            Vehicle::connect(mp_link::ProtocolTimeouts::default().faster(20));
        let (altitude, mut sends) =
            takeoff_plan("7", VEHICLE, Some(VehicleFamily::Copter)).expect("a height");
        sends.push(commands::takeoff(VEHICLE, altitude));
        let (sender, _) = telemetry.send_handle().expect("a link");
        let report = Report::on_timeout(error_box(strings::COMMAND_FAILED));
        let (requests, queued) = send_routed(&mut telemetry, &sender, &sends, &report, true);
        assert!(queued);
        assert_eq!(requests.len(), 1, "the take-off alone is waited on");
        until("the take-off to be heard", || {
            vehicle.read();
            vehicle.count(|m| {
                matches!(m, MavMessage::CommandLong(l) if l.command == commands::CMD_NAV_TAKEOFF)
            }) > 0
        });
        let order: Vec<String> = vehicle
            .heard
            .iter()
            .filter_map(|m| match m {
                MavMessage::CommandLong(l)
                    if [commands::CMD_DO_SET_MODE, commands::CMD_NAV_TAKEOFF]
                        .contains(&l.command) =>
                {
                    Some(format!("cmd{}", l.command))
                }
                MavMessage::SetMode(mode) => Some(format!("mode{}", mode.custom_mode)),
                _ => None,
            })
            .collect();
        let takeoff = format!("cmd{}", commands::CMD_NAV_TAKEOFF);
        let first_takeoff = order.iter().position(|m| *m == takeoff).expect("heard");
        assert_eq!(
            order[..first_takeoff],
            [format!("cmd{}", commands::CMD_DO_SET_MODE), "mode4".to_owned(), "mode4".to_owned()],
            "{order:?}"
        );
        let heard_altitude = vehicle.heard.iter().find_map(|m| match m {
            MavMessage::CommandLong(l) if l.command == commands::CMD_NAV_TAKEOFF => Some(l.param7),
            _ => None,
        });
        assert_eq!(heard_altitude, Some(7.0));
    }

    #[test]
    fn the_calls_the_csharp_waits_on_become_requests_and_the_rest_go_once() {
        let t = target();
        assert_eq!(
            route(&commands::mission_set_current(t, 2)),
            Route::SetCurrent { target: t, seq: 2 }
        );
        assert_eq!(
            route(&commands::change_speed(t, 7.5)),
            Route::Command {
                target: t,
                command: commands::CMD_DO_CHANGE_SPEED,
                params: [0.0, 7.5, 0.0, 0.0, 0.0, 0.0, 0.0],
            }
        );
        assert_eq!(
            route(&commands::param_set(t, "LOITER_RAD", 80.0)),
            Route::SetParam {
                target: t,
                name: "LOITER_RAD".to_owned(),
                value: 80.0,
            }
        );
        // `setMode`, `doReboot`, `setGuidedModeWP`, `sendPacket`.
        let once = [
            set_mode_messages(t, Some(VehicleFamily::Copter), "GUIDED"),
            vec![
                commands::reboot(t),
                commands::system_time(1),
                commands::guided_position_target(t, 3, -35.36, 149.16, 20.0),
            ],
        ]
        .concat();
        for message in &once {
            assert_eq!(route(message), Route::Raw, "{}", describe(message));
        }
        // `setWP` and `doCommandInt`, each with its request (PLAN.md §13.6 row 74).
        let change_alt = commands::change_alt(t, 25.0);
        assert_eq!(
            route(&change_alt),
            Route::SetWp {
                target: t,
                item: Box::new(change_alt)
            }
        );
        assert_eq!(
            route(&commands::command_int(
                t,
                commands::CMD_STORAGE_FORMAT,
                commands::FRAME_GLOBAL,
                [1.0, 1.0, 0.0, 0.0],
                0,
                0,
                0.0,
            )),
            Route::CommandInt {
                target: t,
                command: commands::CMD_STORAGE_FORMAT,
                frame: commands::FRAME_GLOBAL,
                params: [1.0, 1.0, 0.0, 0.0],
                x: 0,
                y: 0,
                z: 0.0,
            }
        );
    }

    /// Do Action's message boxes, per entry: the generic path's refusal names the command,
    /// `Trigger_Camera` falls back to `DIGICAM_CONTROL`, and what is not waited on says nothing.
    /// `// C#: GCSViews/FlightData.cs:1697-1878`
    #[test]
    fn do_action_says_what_the_csharp_says_when_refused_or_unanswered() {
        let generic = action_report("Battery_Reset", target());
        assert_eq!(
            generic.refused.as_deref(),
            Some("Error: The Command failed to execute BATTERY_RESET")
        );
        assert_eq!(
            generic.timed_out.as_deref(),
            Some("Error: The Command failed to execute")
        );
        let engine = action_report("Engine_Start", target());
        assert_eq!(
            engine.refused, None,
            "doEngineControl's answer is not looked at"
        );
        assert!(engine.timed_out.is_some());
        let camera = action_report("Trigger_Camera", target());
        assert!(
            matches!(camera.fallback, Some(MavMessage::DigicamControl(m)) if m.shot == 1 && m.target_system == 1),
            "{camera:?}"
        );
        assert_eq!(camera.refused, None);
        for quiet in ["Preflight_Reboot_Shutdown", "System_Time", "Format_SD_Card"] {
            assert_eq!(action_report(quiet, target()), Report::default(), "{quiet}");
        }
    }

    // --- What was sent and what came back ---------------------------------------------------------

    /// The answer is the newest acknowledgement of that command after it was sent.
    #[test]
    fn the_answer_is_the_newest_ack_after_the_send() {
        let line = |seq, text: &str| LogMessage {
            from: target(),
            severity: Severity::Info,
            text: text.to_owned(),
            seq,
            received: 0,
        };
        let messages = vec![
            line(1, "MAV_CMD_DO_CHANGE_SPEED: denied"),
            line(2, "PreArm: nothing"),
            line(3, "MAV_CMD_DO_CHANGE_SPEED: accepted"),
            line(4, "MAV_CMD_DO_GO_AROUND: unsupported"),
        ];
        assert_eq!(
            ack_line(&messages, 178, 1).map(|m| m.text.as_str()),
            Some("MAV_CMD_DO_CHANGE_SPEED: accepted")
        );
        assert!(ack_line(&messages, 178, 3).is_none(), "nothing newer");

        let mut actions = Actions::default();
        actions.record(&[commands::go_around(target())], 3);
        assert_eq!(actions.ack(&messages), "MAV_CMD_DO_GO_AROUND: unsupported");
        assert!(
            actions
                .last_sent
                .starts_with("COMMAND_LONG MAV_CMD_DO_GO_AROUND 0,0,0")
        );
    }

    /// The line a test asserts on names the message and the values that matter.
    #[test]
    fn a_sent_message_is_described_by_its_values() {
        assert_eq!(
            describe(&commands::mission_set_current(target(), 2)),
            "MISSION_SET_CURRENT seq=2"
        );
        assert_eq!(
            describe(&commands::change_speed(target(), 7.5)),
            "COMMAND_LONG MAV_CMD_DO_CHANGE_SPEED 0,7.5,0,0,0,0,0"
        );
        assert_eq!(
            describe(&commands::change_alt(target(), 25.0)),
            "MISSION_ITEM seq=0 cmd=16 frame=3 current=3 x=0 y=0 z=25"
        );
        assert_eq!(
            describe(&commands::param_set(target(), "WP_LOITER_RAD", 80.0)),
            "PARAM_SET WP_LOITER_RAD=80"
        );
    }

    /// The questions are the C#'s words.
    #[test]
    fn the_questions_are_the_csharps() {
        assert_eq!(
            Prompt::ConfirmAction(3).text(),
            "Are you sure you want to do Mission_Start ?"
        );
        assert_eq!(Prompt::ConfirmAction(3).buttons(), ("Yes", "No"));
        assert_eq!(Prompt::ResumeAt.text(), "Resume mission at waypoint#");
        assert!(Prompt::FlyToCoords.takes_text());
        assert!(!Prompt::ResumeWarning.takes_text());
    }

    // --- tabControlactions --------------------------------------------------------------------

    /// `this.tabControlactions.Controls.Add(this.X);`, in the Designer's order.
    fn designer_pages(designer: &str) -> Vec<String> {
        designer
            .lines()
            .filter_map(|line| {
                line.trim()
                    .strip_prefix("this.tabControlactions.Controls.Add(this.")?
                    .strip_suffix(");")
                    .map(ToOwned::to_owned)
            })
            .collect()
    }

    /// `<data name="X.Text" ...>` then `<value>T</value>`: a control's text in the `.resx`.
    fn resx_text(resx: &str, control: &str) -> Option<String> {
        let marker = format!("<data name=\"{control}.Text\"");
        let mut lines = resx
            .lines()
            .skip_while(|line| !line.trim().starts_with(&marker));
        lines.next()?;
        let value = lines.next()?.trim();
        Some(
            value
                .strip_prefix("<value>")?
                .strip_suffix("</value>")?
                .to_owned(),
        )
    }

    /// The strip is the Designer's fourteen pages, in the order it adds them.
    #[test]
    fn the_strip_is_the_designers_pages_in_the_designers_order() {
        let Some(designer) = csharp("GCSViews/FlightData.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let ours: Vec<String> = Page::ALL
            .iter()
            .map(|page| page.name().to_owned())
            .collect();
        assert_eq!(designer_pages(&designer), ours);
    }

    /// Each header reads as the page's `Text` in the `.resx`.
    #[test]
    fn each_header_is_the_pages_text_in_the_resx() {
        let Some(resx) = csharp("GCSViews/FlightData.resx") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        for page in Page::ALL {
            assert_eq!(
                resx_text(&resx, page.name()).as_deref(),
                Some(page.text()),
                "{}",
                page.name()
            );
        }
    }

    /// The page shown at start is the one `SelectedIndex` names.
    #[test]
    fn the_page_at_start_is_the_designers_selected_index() {
        assert_eq!(Pages::default().selected(), DEFAULT_PAGE);
        let Some(designer) = csharp("GCSViews/FlightData.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let index: usize = designer
            .lines()
            .find_map(|line| {
                line.trim()
                    .strip_prefix("this.tabControlactions.SelectedIndex = ")?
                    .strip_suffix(';')?
                    .parse()
                    .ok()
            })
            .expect("the Designer sets tabControlactions.SelectedIndex");
        assert_eq!(Page::ALL.get(index).copied(), Some(DEFAULT_PAGE));
    }

    /// The same, written out, so a reordering fails here even where the C# tree is absent.
    #[test]
    fn the_strip_reads_quick_actions_messages_and_on() {
        assert_eq!(
            page_list(&Page::ALL, Page::text),
            "Quick,Actions,Messages,Actions,PreFlight,Gauges,Transponder,Status,Servo/Relay,\
             Aux Function,Scripts,Payload Control,Telemetry Logs,DataFlash Logs"
        );
        assert_eq!(
            page_list(&Page::ALL, Page::name),
            "tabQuick,tabActions,tabPagemessages,tabActionsSimple,tabPagePreFlight,tabGauges,\
             tabTransponder,tabStatus,tabServo,tabAuxFunction,tabScripts,tabPayload,tabTLogs,\
             tablogbrowse"
        );
        assert_eq!(DEFAULT_PAGE, Page::Quick);
    }

    /// Two headers read "Actions"; a script still clicks each by a name of its own.
    #[test]
    fn every_header_has_an_id_of_its_own() {
        let mut ids: Vec<&str> = Page::ALL.iter().map(|page| page.id()).collect();
        assert!(ids.iter().all(|id| id.starts_with("fly-tab-")));
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), Page::ALL.len());
    }

    /// Every panel the column had is on one page, once; the rest of the pages say what is
    /// missing from them.
    #[test]
    fn each_panel_is_on_one_page_and_every_other_page_says_what_it_lacks() {
        for panel in [
            Panel::Quick,
            Panel::Actions,
            Panel::PreArm,
            Panel::Health,
            Panel::Playback,
            Panel::DataFlash,
            Panel::Transponder,
            Panel::Payload,
            Panel::Gauges,
        ] {
            let pages: Vec<Page> = Page::ALL
                .into_iter()
                .filter(|page| page.panels().contains(&panel))
                .collect();
            assert_eq!(pages.len(), 1, "{panel:?} is on {pages:?}");
        }
        assert_eq!(Page::Actions.panels(), &[Panel::Actions]);
        assert_eq!(Page::Quick.panels(), &[Panel::Quick]);
        assert_eq!(Page::TLogs.panels(), &[Panel::Playback]);
        assert_eq!(Page::LogBrowse.panels(), &[Panel::DataFlash]);
        assert_eq!(Page::PreFlight.panels(), &[Panel::PreArm, Panel::Health]);
        assert_eq!(Page::Transponder.panels(), &[Panel::Transponder]);
        assert_eq!(Page::Payload.panels(), &[Panel::Payload]);
        assert_eq!(Page::Gauges.panels(), &[Panel::Gauges]);
        for page in Page::ALL {
            assert_eq!(
                page.note().is_none(),
                matches!(page, Page::Actions | Page::Quick | Page::Transponder),
                "{} should say what the C# has on it that this does not",
                page.name()
            );
        }
    }

    /// The arrows move the row one header at a time and stop at either end; choosing a page
    /// does not move the row.
    #[test]
    fn the_arrows_move_the_row_one_header_and_stop_at_the_ends() {
        let mut pages = Pages::default();
        assert!(!pages.can_scroll_left());
        pages.scroll_left();
        assert_eq!(pages.first_shown(), 0);
        for expected in 1..Page::ALL.len() {
            assert!(pages.can_scroll_right());
            pages.scroll_right();
            assert_eq!(pages.first_shown(), expected);
        }
        assert!(!pages.can_scroll_right());
        pages.scroll_right();
        assert_eq!(pages.first_shown(), Page::ALL.len() - 1);
        pages.select(Page::Status);
        assert_eq!(pages.selected(), Page::Status);
        assert_eq!(pages.first_shown(), Page::ALL.len() - 1);
        pages.scroll_left();
        assert_eq!(pages.first_shown(), Page::ALL.len() - 2);
    }

    // --- The dialog's keys --------------------------------------------------------------------

    /// A keystroke as gpui's X11 backend builds it: `key` is the key's name and `key_char` what
    /// it types (`keystroke_from_xkb` in gpui_linux).
    fn press(key: &str, key_char: Option<&str>) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: key.to_owned(),
                key_char: key_char.map(ToOwned::to_owned),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    /// A question with no box answers to Enter and Escape and to nothing else.
    #[test]
    fn a_question_without_a_box_hears_enter_and_escape() {
        assert_eq!(
            answer_key(&press("enter", Some("\r"))),
            KeyOutcome::Submitted
        );
        assert_eq!(answer_key(&press("escape", None)), KeyOutcome::Cancelled);
        assert_eq!(answer_key(&press("y", Some("y"))), KeyOutcome::Ignored);
        assert_eq!(answer_key(&press("space", Some(" "))), KeyOutcome::Ignored);
    }

    /// Fly To Coords, typed the way `fly-flytocoords.gui` types it - the minus as its own key,
    /// then the rest - reaches the box whole and reads as three numbers.
    ///
    /// And why the script once failed: a click on OK that overtakes the last keystrokes answers
    /// with the text so far. With one `;` that is `lat;long`, which flies at `GuidedMode.z` - zero
    /// before anything has set it - and sends nothing, the refusal the run reported. Every prefix
    /// that has a second number but not a third does the same.
    #[test]
    fn fly_to_coords_typed_key_by_key_reaches_the_box_whole() {
        let mut actions = Actions::default();
        actions.ask(Prompt::FlyToCoords, "");
        let mut keys = vec![press("-", Some("-"))];
        for c in "35.3625;149.1657;20".chars() {
            let text = c.to_string();
            let key = if c == ';' { ";" } else { text.as_str() };
            keys.push(press(key, Some(&text)));
        }
        for key in &keys {
            assert_eq!(actions.prompt_field.key(key), KeyOutcome::Changed);
        }
        assert_eq!(actions.prompt_field.value(), "-35.3625;149.1657;20");
        let Ok(Coords::Full {
            latitude,
            longitude,
            altitude,
        }) = parse_coords(actions.prompt_field.value())
        else {
            panic!("three numbers");
        };
        assert!((latitude + 35.3625).abs() < 1e-5);
        assert!((longitude - 149.1657).abs() < 1e-4);
        assert!((altitude - 20.0).abs() < f64::EPSILON);

        let context = GuidedContext {
            target: target(),
            family: Some(VehicleFamily::Copter),
            mode: Some("Stabilize"),
        };
        let mut guided = GuidedMode::default();
        #[allow(clippy::cast_possible_truncation)]
        let whole = set_guided_mode_wp(
            &mut guided,
            &context,
            (latitude, longitude, altitude as f32),
            commands::FRAME_GLOBAL_RELATIVE_ALT,
        );
        assert!(!whole.is_empty(), "the whole text flies");

        for cut in ["-35.3625;1", "-35.3625;149.1657", "-35.3625;149.1657;"] {
            let sends = match parse_coords(cut) {
                Ok(Coords::Position {
                    latitude,
                    longitude,
                }) => set_guided_mode_wp(
                    &mut GuidedMode::default(),
                    &context,
                    (latitude, longitude, GuidedMode::default().z),
                    commands::FRAME_GLOBAL_RELATIVE_ALT,
                ),
                Ok(Coords::Full { .. }) => panic!("{cut} is not three numbers"),
                Err(_) => Vec::new(),
            };
            assert!(sends.is_empty(), "{cut} sends nothing");
        }
    }

    /// What the dialog's box holds is published while a question with a box is open, and only
    /// then, so a script can see its typing arrive before it answers.
    #[test]
    fn the_prompts_box_is_published_while_it_is_open() {
        let mut actions = Actions::default();
        assert_eq!(actions.prompt_value(), "none");
        actions.ask(Prompt::FlyToCoords, "");
        assert_eq!(actions.prompt_value(), "");
        for key in [
            press("1", Some("1")),
            press(";", Some(";")),
            press("2", Some("2")),
        ] {
            actions.prompt_field.key(&key);
        }
        assert_eq!(actions.prompt_value(), "1;2");
        actions.ask(Prompt::ResumeWarning, "");
        assert_eq!(actions.prompt_value(), "none");
    }

    // --- The HUD's windows, the Telemetry Logs page and the POI questions ------------------------

    /// `IntersectsWith(new Rectangle(x, y, 5, 5))` against the C#'s own rectangle, for holding
    /// the zones to it.
    fn csharp_hit(zone: (f32, f32, f32, f32), (x, y): (f32, f32)) -> bool {
        let (left, top, width, height) = zone;
        x < left + width && left < x + 5.0 && y < top + height && top < y + 5.0
    }

    /// The zones are where "EKF" and "Vibe" are drawn, 40 wide and twice the font high, grown
    /// by the C#'s five-pixel box: every point the C# counts as a hit is inside one, and none
    /// outside it.
    #[test]
    fn a_click_on_ekf_or_vibe_is_a_click_in_the_csharps_rectangle() {
        // Without a vehicle the HUD draws neither, and there is nothing to click.
        let idle = crate::hud::scene(&crate::hud::HudInputs::default(), 398.0, 258.0);
        assert_eq!(hud_zone(&idle, HudWindow::Ekf), None);
        let inputs = crate::hud::HudInputs {
            has_vehicle: true,
            ..crate::hud::HudInputs::default()
        };
        let (w, h) = (398.0, 258.0);
        let scene = crate::hud::scene(&inputs, w, h);
        // `fontsize` as `doPaint` sets it for this height. `// C#: ExtLibs/Controls/HUD.cs:2040-2048`
        let fontsize = (h / 30.0f32).max(9.0);
        for (which, text, x) in [
            (HudWindow::Ekf, "EKF", w - 23.0 * fontsize),
            (HudWindow::Vibration, "Vibe", w - 18.0 * fontsize),
        ] {
            let zone = hud_zone(&scene, which).expect("the text is drawn");
            let label_y = scene
                .owners
                .iter()
                .find_map(|(_, index)| match scene.items.get(*index) {
                    Some(crate::hud::Item::Label {
                        text: drawn, at, ..
                    }) if drawn == text => Some(at.1),
                    _ => None,
                })
                .expect("the label");
            let csharp = (x, label_y, 40.0, fontsize * 2.0);
            for dx in -8..50 {
                for dy in -8..30 {
                    #[allow(clippy::cast_precision_loss)]
                    let point = (x + dx as f32, label_y + dy as f32);
                    let (left, top, width, height) = zone;
                    let ours = point.0 > left
                        && point.0 < left + width
                        && point.1 > top
                        && point.1 < top + height;
                    assert_eq!(ours, csharp_hit(csharp, point), "{which:?} at {point:?}");
                }
            }
        }
        assert_eq!(HudWindow::Ekf.id(), "hud-ekf");
        assert_eq!(HudWindow::Vibration.id(), "hud-vibe");
    }

    #[test]
    fn the_ekf_window_lists_every_flag_to_uninitialized() {
        let lines = ekf_flags(0x0001 | 0x0004);
        let texts: Vec<&str> = lines.iter().map(|(text, _)| text.as_str()).collect();
        assert_eq!(
            texts,
            [
                "attitude On ",
                "velocity_horiz Off",
                "velocity_vert On ",
                "pos_horiz_rel Off",
                "pos_horiz_abs Off",
                "pos_vert_abs Off",
                "pos_vert_agl Off",
                "const_pos_mode Off",
                "pred_pos_horiz_rel Off",
                "pred_pos_horiz_abs Off",
                "uninitialized Off",
            ]
        );
        // Red: horizontal velocity and the two absolute positions, when off.
        let red: Vec<&str> = lines
            .iter()
            .filter(|(_, red)| *red)
            .map(|(text, _)| text.as_str())
            .collect();
        assert_eq!(
            red,
            [
                "velocity_horiz Off",
                "pos_horiz_abs Off",
                "pos_vert_abs Off"
            ]
        );
        assert!(ekf_flags(0xffff).iter().all(|(_, red)| !red));
    }

    #[test]
    fn the_windows_bars_are_the_csharps_numbers() {
        let mut state = mp_vehicle::VehicleState::default();
        state.ekf.velocity_variance = 0.567;
        state.ekf.compass_variance = 0.9;
        state.vibration.x = 31.9;
        state.vibration.clipping = [1, 2, 3];
        assert_eq!(ekf_values(&state), [56, 0, 0, 90, 0]);
        assert_eq!(ekf_colour(50), 0xff_00_ff);
        assert_eq!(ekf_colour(56), 0xff_a5_00);
        assert_eq!(ekf_colour(81), 0xff_00_00);
        assert_eq!(vibration_values(&state), ([31, 0, 0], [1, 2, 3]));
    }

    /// A replay's controls, over a log of `records` frames 10 ms apart.
    fn control(records: u64) -> Arc<mp_transport::replay::Playback> {
        let mut data = Vec::new();
        for i in 0..records {
            data.extend_from_slice(&(i * 10_000).to_be_bytes());
            data.extend_from_slice(&[0xFE, 1, 0, 1, 1, 0, 0, 0, 0]);
        }
        mp_transport::ReplayTransport::from_bytes("test.tlog", data)
            .paced()
            .1
    }

    #[test]
    fn the_speed_buttons_set_the_speed_and_say_it() {
        let mut playback = Playback::default();
        // The `.resx`'s text until something sets it.
        assert_eq!(playback.speed_label, "x 1.0");
        for (label, speed, _) in SPEEDS {
            playback.set_speed(speed);
            let said = label().trim_end_matches('x');
            assert_eq!(playback.speed_label, format!("x {said}"));
        }
        // A log loaded afterwards plays at the speed chosen.
        let control = control(4);
        playback.load("/somewhere/flight.tlog", Arc::clone(&control));
        assert_eq!(control.speed(), 10.0);
        assert_eq!(playback.file_name, "flight.tlog");
    }

    #[test]
    fn play_pause_toggles_a_loaded_log_and_nothing_else() {
        let mut playback = Playback::default();
        playback.toggle();
        assert_eq!(playback.button(), "Play");
        let control = control(10);
        playback.load("flight.tlog", Arc::clone(&control));
        assert!(playback.playing());
        assert_eq!(playback.button(), "Pause");
        playback.toggle();
        assert!(!playback.playing());
        assert_eq!(playback.button(), "Play");
        playback.toggle();
        assert!(playback.playing());
    }

    #[test]
    fn the_track_bar_moves_by_its_large_change_and_seeks() {
        let mut playback = Playback::default();
        let control = control(100);
        playback.load("flight.tlog", Arc::clone(&control));
        playback.toggle();
        // Beside the thumb, at 0: five on, and the file moved to 5%.
        playback.press(0.9, 0.05);
        assert_eq!(playback.tracklog, 5);
        assert_eq!(control.position(), control.len() / 20);
        assert_eq!(playback.percent_label, "5.00%");
        playback.press(0.9, 0.05);
        assert_eq!(playback.tracklog, 10);
        playback.press(0.0, 0.05);
        assert_eq!(playback.tracklog, 5);
        // On the thumb: held, and dragged.
        playback.press(0.05, 0.05);
        assert_eq!(playback.tracklog, 5);
        playback.drag(0.5);
        assert_eq!(playback.tracklog, 50);
        playback.release();
        playback.drag(0.7);
        assert_eq!(playback.tracklog, 50);
        // Never past either end.
        playback.scroll(100);
        playback.press(1.0, 0.0);
        assert_eq!(playback.tracklog, 100);
    }

    #[test]
    fn the_end_of_the_log_stops_it_as_the_main_loop_does() {
        let mut playback = Playback::default();
        let control = control(3);
        playback.load("flight.tlog", Arc::clone(&control));
        control.seek_fraction(1.0);
        playback.tick();
        assert_eq!(playback.tracklog, 100);
        assert_eq!(playback.percent_label, "100.00%");
        assert!(control.is_paused());
        assert_eq!(playback.button(), "Play");
        // Back along the bar, it is still stopped until Play.
        playback.scroll(50);
        assert!(!playback.playing());
        playback.toggle();
        assert!(playback.playing());
    }

    /// The product's path: a recorded flight opened as Load Log opens it, played through the
    /// real link, reaches the screen - and stops reaching it when paused.
    #[test]
    fn a_loaded_log_plays_through_the_link_and_pauses() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/mavlink/autotest.tlog");
        if !path.exists() {
            eprintln!("skipped: {} is not here", path.display());
            return;
        }
        assert!(replay("/no/such/flight.tlog").is_err());
        let (telemetry, control) = replay(&path.display().to_string()).expect("opens");
        assert_eq!(
            u64::try_from(control.len()).unwrap(),
            std::fs::metadata(&path).unwrap().len()
        );
        control.set_speed(1000.0);
        let started = Instant::now();
        while telemetry.view().state.is_none() && started.elapsed() < Duration::from_secs(10) {
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(telemetry.view().state.is_some(), "no vehicle after 10 s");
        assert!(telemetry.recording().is_none(), "a replay is not recorded");
        control.set_paused(true);
        std::thread::sleep(Duration::from_millis(300));
        let held = control.position();
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(control.position(), held);
        assert!(held < control.len());
    }

    /// A log being played is not a port: Load Log replaces it, and the Log Downloader asks
    /// for a drone. A link to a vehicle is.
    #[test]
    fn a_played_log_is_not_an_open_port() {
        let mut view = TelemetryView::disconnected("tcp:127.0.0.1:5760");
        assert!(!port_open(&view));
        view.connected = true;
        assert!(port_open(&view));
        let mut played = TelemetryView::disconnected("file:flight.tlog");
        played.connected = true;
        assert!(!port_open(&played));
    }

    #[test]
    fn load_log_closes_the_log_playing() {
        let mut playback = Playback::default();
        let control = control(10);
        playback.load("flight.tlog", Arc::clone(&control));
        playback.close();
        assert!(control.is_paused());
        assert!(!playback.playing());
        playback.toggle();
        assert!(
            control.is_paused(),
            "Play has nothing to play once it is closed"
        );
    }

    #[test]
    fn the_poi_questions_are_the_csharps() {
        assert_eq!(Prompt::PoiId.title(), "POI");
        assert_eq!(Prompt::PoiId.text(), "Enter ID");
        assert_eq!(Prompt::PoiCoords.title(), "Enter POI Coords");
        assert_eq!(
            Prompt::PoiCoords.text(),
            "Please enter the coords 'lat;long;alt' or 'lat;long'"
        );
        assert!(Prompt::PoiId.takes_text());
        assert!(Prompt::PoiCoords.takes_text());
        assert!(Prompt::LoadLog.takes_text());
        assert_eq!(Prompt::LoadLog.title(), "Load Log");
    }

    // --- The DataFlash Logs page's conversions ------------------------------------------------

    fn testdata(name: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    /// A directory of its own for one test, with the checked-in log copied in as `name`.
    fn scratch_log(test: &str, name: &str) -> (std::path::PathBuf, std::path::PathBuf) {
        let dir = std::env::temp_dir().join(format!("mp-gui-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        let log = dir.join(name);
        std::fs::copy(testdata("dataflash.bin"), &log).expect("the log copied");
        (dir, log)
    }

    /// Waits for a started conversion, as the frames' polling does.
    fn finish(conversions: &mut Conversions) -> Outcome {
        let deadline = Instant::now() + Duration::from_secs(300);
        loop {
            if let Some(outcome) = conversions.poll() {
                return outcome;
            }
            assert!(Instant::now() < deadline, "the conversion never finished");
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    /// The buttons are the Designer's: their words from the `.resx`, each in the cell
    /// `tableLayoutPanel2.LayoutSettings` gives it.
    #[test]
    fn the_conversion_buttons_are_the_designers() {
        let Some(resx) = csharp("GCSViews/FlightData.resx") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let layout = resx
            .lines()
            .find(|line| line.contains("&lt;Control Name=\"BUT_DFMavlink\""))
            .expect("tableLayoutPanel2's layout");
        for (kind, control) in [
            (Conversion::BinToLog, "but_bintolog"),
            (Conversion::DflogToKml, "but_dflogtokml"),
            (Conversion::Matlab, "BUT_matlab"),
            (Conversion::LogAnalysis, "BUT_loganalysis"),
        ] {
            assert_eq!(resx_text(&resx, control).as_deref(), Some(kind.text()));
            let (column, row) = kind.cell();
            assert!(
                layout.contains(&format!(
                    "&lt;Control Name=\"{control}\" Row=\"{row}\" RowSpan=\"1\" Column=\"{column}\""
                )),
                "{control} is at column {column}, row {row}"
            );
            assert_eq!(kind.id(), format!("fly-{}", kind.name()));
        }
        assert_eq!(
            resx_text(&resx, "BUT_georefimage").as_deref(),
            Some("Geo Reference Images")
        );
        assert!(
            layout.contains(
                "&lt;Control Name=\"BUT_georefimage\" Row=\"2\" RowSpan=\"1\" Column=\"0\""
            )
        );
    }

    /// Each prompt is the dialog's: titled with the button, worded with the filter's first
    /// description, and starting in the folder the dialog opens in.
    #[test]
    fn the_conversion_prompts_open_where_the_dialogs_open() {
        use std::path::Path;
        let (tlogdir, logdir) = (Path::new("/flights/last"), Path::new("/logs"));
        assert_eq!(
            Conversion::DflogToKml.directory(Some(tlogdir), Some(logdir)),
            Some(tlogdir.to_path_buf())
        );
        assert_eq!(
            Conversion::LogAnalysis.directory(Some(tlogdir), Some(logdir)),
            Some(tlogdir.to_path_buf())
        );
        assert_eq!(
            Conversion::Matlab.directory(Some(tlogdir), Some(logdir)),
            Some(logdir.to_path_buf())
        );
        assert_eq!(
            Conversion::BinToLog.directory(Some(tlogdir), Some(logdir)),
            Some(logdir.to_path_buf())
        );
        assert_eq!(
            Conversion::DflogToKml.directory(None, Some(logdir)),
            Some(logdir.to_path_buf())
        );
        let prompt = Prompt::Convert(Conversion::BinToLog);
        assert_eq!(prompt.title(), "Convert .Bin to .Log");
        assert_eq!(prompt.text(), "Binary Log");
        assert!(prompt.takes_text());
        assert_eq!(Prompt::Convert(Conversion::DflogToKml).text(), "Log Files");
        assert_eq!(Prompt::Convert(Conversion::Matlab).text(), "Log Files");
        assert_eq!(
            Prompt::Convert(Conversion::LogAnalysis).text(),
            "*.log;*.bin"
        );
    }

    /// The three conversions that write files, each started as its button starts it - on a
    /// thread of its own, one at a time - and each leaving what Mission Planner's code leaves:
    /// the `.log` byte for byte, the KML button's side files at their golden sizes, and the
    /// `.mat` named for its lines at the golden size.
    #[test]
    fn each_conversion_writes_what_its_button_writes() {
        let (dir, log) = scratch_log("convert", "dataflash.bin");
        let mut conversions = Conversions::default();

        assert!(conversions.start(Conversion::BinToLog, log.clone(), None));
        assert_eq!(conversions.running(), Some(Conversion::BinToLog));
        assert!(
            !conversions.start(Conversion::Matlab, log.clone(), None),
            "nothing starts while one runs"
        );
        let outcome = finish(&mut conversions);
        assert_eq!(conversions.running(), None);
        let target = dir.join("dataflash.log");
        assert_eq!(
            outcome.2,
            Ok(Converted::Files(vec![(target.clone(), 868_884)]))
        );
        assert_eq!(
            std::fs::read(&target).expect("the .log"),
            std::fs::read(testdata("dataflash/golden/dataflash.log")).expect("the golden")
        );
        assert_eq!(
            conversion_status(&outcome),
            format!("Convert .Bin to .Log: {}", target.display())
        );

        assert!(conversions.start(Conversion::DflogToKml, log.clone(), None));
        let outcome = finish(&mut conversions);
        let Ok(Converted::Files(files)) = &outcome.2 else {
            panic!("Create KML + gpx failed: {:?}", outcome.2);
        };
        let named: Vec<(String, u64)> = files
            .iter()
            .map(|(path, size)| {
                (
                    path.file_name()
                        .expect("a file")
                        .to_string_lossy()
                        .into_owned(),
                    *size,
                )
            })
            .collect();
        eprintln!("Create KML + gpx wrote {named:?}");
        for (name, golden) in [
            (
                "dataflash.bin.gpx",
                "dataflash/golden/kml/dataflash.bin.gpx",
            ),
            (
                "dataflash.bin0wp.txt",
                "dataflash/golden/kml/dataflash.bin0wp.txt",
            ),
            (
                "dataflash.bin.param",
                "dataflash/golden/kml/dataflash.bin.param",
            ),
        ] {
            let size = std::fs::metadata(testdata(golden)).expect("a golden").len();
            assert!(
                named.contains(&(name.to_owned(), size)),
                "{name} at {size} bytes in {named:?}"
            );
        }
        assert!(
            named
                .iter()
                .any(|(name, size)| name == "dataflash.kmz" && *size > 0)
        );

        assert!(conversions.start(Conversion::Matlab, log, None));
        let outcome = finish(&mut conversions);
        let mat = dir.join("dataflash.bin-11439.mat");
        let golden = std::fs::metadata(testdata("dataflash/golden/matlab/dataflash.bin-11439.mat"))
            .expect("the golden")
            .len();
        assert_eq!(outcome.2, Ok(Converted::Files(vec![(mat, golden)])));
        std::fs::remove_dir_all(&dir).expect("the scratch directory removed");
    }

    /// Where the C# shows a message box, the status line says the same words.
    #[test]
    fn a_conversion_that_fails_says_what_the_csharps_box_says() {
        let missing = std::env::temp_dir().join(format!(
            "mp-gui-convert-missing-{}/none.bin",
            std::process::id()
        ));
        let mut no_fetch = |_: &str, _: &std::path::Path| false;
        let kml = convert(Conversion::DflogToKml, &missing, None, &mut no_fetch);
        assert!(
            kml.as_ref()
                .is_err_and(|why| why
                    .starts_with("Error processing file. Make sure the file is not in use.\n")),
            "{kml:?}"
        );
        let mat = convert(Conversion::Matlab, &missing, None, &mut no_fetch);
        assert!(
            mat.as_ref()
                .is_err_and(|why| why.starts_with("Error converting file ")),
            "{mat:?}"
        );
        let outcome = (Conversion::Matlab, missing, mat);
        assert!(conversion_status(&outcome).starts_with("Error: Error converting file "));
    }

    /// Auto Analysis downloads the analyzer - here the download fails, and the one from before is
    /// used - runs it on the log converted to a temporary `.log`, and shows the report the C#'s
    /// window shows. The stand-in analyzer is a script that writes the example output where it
    /// is told to, so this runs where a script can be a program.
    #[cfg(unix)]
    #[test]
    fn auto_analysis_runs_the_analyzer_and_shows_its_report() {
        use std::os::unix::fs::PermissionsExt;
        let (dir, log) = scratch_log("analysis", "flight.bin");
        let analyzer = dir.join("LogAnalyzer");
        std::fs::create_dir_all(&analyzer).expect("the analyzer's directory");
        let runner = analyzer.join("runner.exe");
        std::fs::write(
            &runner,
            format!(
                "#!/bin/sh\ncp '{}' \"$2\"\n",
                testdata("dataflash/example_output.xml").display()
            ),
        )
        .expect("the stand-in runner");
        std::fs::set_permissions(&runner, std::fs::Permissions::from_mode(0o755))
            .expect("the runner made runnable");
        let mut fetched = Vec::new();
        let mut fetch = |url: &str, _: &std::path::Path| {
            fetched.push(url.to_owned());
            false
        };
        let outcome = convert(Conversion::LogAnalysis, &log, Some(&analyzer), &mut fetch);
        assert_eq!(fetched, [mp_log::analysis::analyzer_url()]);
        let golden =
            std::fs::read_to_string(testdata("dataflash/golden/loganalysis/example_output.txt"))
                .expect("the golden report");
        assert_eq!(outcome, Ok(Converted::Report(golden)));
        let outcome = (Conversion::LogAnalysis, log.clone(), outcome);
        assert_eq!(
            conversion_status(&outcome),
            format!("Auto Analysis: {}", log.display())
        );

        // No download and no analyzer from before: the C#'s "Failed to download LogAnalyzer".
        std::fs::remove_file(&runner).expect("the runner removed");
        let failed = convert(
            Conversion::LogAnalysis,
            &log,
            Some(&analyzer),
            &mut |_, _| false,
        );
        assert_eq!(failed, Err("Failed to download LogAnalyzer".to_owned()));
        let _ = std::fs::remove_dir_all(&dir);
    }

    // --- The HUD's menu -----------------------------------------------------------------------

    /// The menu is `contextMenuStripHud`'s rows in the Designer's order, and the Video drop-down
    /// `videoToolStripMenuItem`'s, each with its words from the `.resx`.
    #[test]
    fn the_hud_menu_is_the_designers() {
        let (Some(designer), Some(resx)) = (
            csharp("GCSViews/FlightData.Designer.cs"),
            csharp("GCSViews/FlightData.resx"),
        ) else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let items = |header: &str| -> Vec<String> {
            designer
                .lines()
                .skip_while(|line| !line.trim().starts_with(header))
                .skip(1)
                .map(str::trim)
                // Each item a line of its own, the last closing the array; the statement after
                // it is an assignment.
                .take_while(|line| line.starts_with("this.") && !line.contains(" = "))
                .map(|line| {
                    line.trim_start_matches("this.")
                        .trim_end_matches(['}', ')', ';', ','])
                        .to_owned()
                })
                .collect()
        };
        let menu = items("this.contextMenuStripHud.Items.AddRange(");
        let video = items("this.videoToolStripMenuItem.DropDownItems.AddRange(");
        assert_eq!(
            menu,
            HUD_MENU.iter().map(|row| row.control).collect::<Vec<_>>()
        );
        assert_eq!(
            video,
            HUD_VIDEO_MENU
                .iter()
                .map(|row| row.control)
                .collect::<Vec<_>>()
        );
        for row in HUD_MENU.iter().chain(&HUD_VIDEO_MENU) {
            assert_eq!(
                resx_text(&resx, row.control).as_deref(),
                Some(row.text),
                "{}",
                row.control
            );
        }
        // What is ported, and every other row says why it is not.
        let live: Vec<&str> = HUD_MENU
            .iter()
            .filter(|row| row.does.is_ok())
            .map(|row| row.text)
            .collect();
        assert_eq!(
            live,
            [
                "Video",
                "User Items",
                "Russian Hud",
                "Swap With Map",
                "Ground Color",
                "Battery Cell Voltage",
                "Show icons"
            ]
        );
        let live: Vec<&str> = HUD_VIDEO_MENU
            .iter()
            .filter(|row| row.does.is_ok())
            .map(|row| row.text)
            .collect();
        assert_eq!(
            live,
            [
                "Set MJPEG source",
                "Set GStreamer Source",
                "HereLink Video",
                "GStreamer Stop"
            ]
        );
        assert!(
            HUD_VIDEO_MENU
                .iter()
                .filter_map(|row| row.does.ok())
                .all(HudAction::in_video_menu)
        );
    }

    /// Ground Color is `CheckOnClick`: the first click checks it and paints the ground brown,
    /// the next unchecks it and paints it green. The ground is the scene's second fill, under
    /// the horizon; before the entry is clicked it is the display's own.
    #[test]
    fn ground_color_checks_on_a_click_and_paints_the_ground() {
        let inputs = crate::hud::HudInputs {
            has_vehicle: true,
            ..crate::hud::HudInputs::default()
        };
        let (w, h) = (398.0, 258.0);
        let own = crate::hud::scene(&inputs, w, h);
        let mean_y = |item: Option<&crate::hud::Item>| match item {
            Some(crate::hud::Item::Fill { points, .. }) => {
                #[allow(clippy::cast_precision_loss)]
                let count = points.len() as f32;
                points.iter().map(|(_, y)| y).sum::<f32>() / count
            }
            other => panic!("not a fill: {other:?}"),
        };
        assert!(mean_y(own.items.first()) < h / 2.0, "the sky is first");
        assert!(mean_y(own.items.get(1)) > h / 2.0, "the ground is second");

        let mut settings = HudSettings::default();
        assert_eq!(settings.ground_colours(), None);
        assert_eq!(hud_scene(&inputs, settings.ground_colours(), w, h), own);

        let fill_colour = |scene: &crate::hud::Scene, index: usize| match scene.items.get(index) {
            Some(crate::hud::Item::Fill { colour, .. }) => *colour,
            other => panic!("not a fill: {other:?}"),
        };
        settings.toggle_ground();
        assert_eq!(settings.ground, Some(true));
        let brown = hud_scene(&inputs, settings.ground_colours(), w, h);
        assert_eq!(fill_colour(&brown, 1), 0x93_4e_01);
        assert_eq!(
            fill_colour(&brown, 0),
            fill_colour(&own, 0),
            "the sky is left"
        );
        assert_eq!(brown.items.len(), own.items.len());

        settings.toggle_ground();
        assert_eq!(settings.ground, Some(false));
        let green = hud_scene(&inputs, settings.ground_colours(), w, h);
        assert_eq!(fill_colour(&green, 1), 0x9b_b8_24);
    }

    /// A box checked asks for its header, starting at its text and ": ", and the item goes on
    /// the HUD under the header given, its value read from the vehicle; checked again it comes
    /// off. A cancelled header leaves nothing.
    #[test]
    fn user_items_ask_a_header_and_come_off_when_unchecked() {
        let mut settings = HudSettings::default();
        assert_eq!(
            settings.click_item("groundspeed").as_deref(),
            Some("groundspeed: ")
        );
        assert_eq!(settings.pending.as_deref(), Some("groundspeed"));
        // The header's answer is `add_item`, which the prompt's OK calls.
        settings.pending = None;
        settings.add_item("groundspeed", "GS ");
        assert!(settings.shows("groundspeed"));

        let mut state = mp_vehicle::VehicleState::default();
        state.ground_speed = mp_units::MetresPerSecond(3.5);
        let mut inputs = crate::hud::HudInputs {
            has_vehicle: true,
            ..crate::hud::HudInputs::default()
        };
        settings.apply(&mut inputs, Some(&state));
        assert_eq!(
            inputs.custom_items,
            [crate::hud::CustomItem {
                header: "GS ".to_owned(),
                name: "groundspeed".to_owned(),
                value: Some(3.5),
            }]
        );
        let scene = crate::hud::scene(&inputs, 398.0, 258.0);
        assert_eq!(
            scene.labels_of(crate::hud::Element::CustomItems).len(),
            1,
            "drawn on the HUD"
        );

        assert_eq!(settings.click_item("groundspeed"), None, "unchecked");
        assert!(!settings.shows("groundspeed"));
        settings.apply(&mut inputs, Some(&state));
        assert!(inputs.custom_items.is_empty());
    }

    /// Russian Hud turns the flag over, and the display is told.
    #[test]
    fn russian_hud_turns_the_flag_over() {
        let mut settings = HudSettings::default();
        let mut inputs = crate::hud::HudInputs::default();
        settings.toggle_russian();
        settings.apply(&mut inputs, None);
        assert!(inputs.russian);
        settings.toggle_russian();
        settings.apply(&mut inputs, None);
        assert!(!inputs.russian);
    }

    /// `StringCompareTo`: runs of digits as numbers, case ignored, the shorter first.
    #[test]
    fn display_this_lists_numbers_in_string_compare_to_order() {
        use std::cmp::Ordering::{Equal, Greater, Less};
        assert_eq!(string_compare_to("ch9in", "ch10in"), Less);
        assert_eq!(string_compare_to("ch10in", "ch9in"), Greater);
        assert_eq!(string_compare_to("Alt", "alt"), Equal);
        assert_eq!(string_compare_to("alt", "altasl"), Less);
        assert_eq!(string_compare_to("b", "Alt"), Greater);
        // "01" and "1" are the same number, and then the longer is after.
        assert_eq!(string_compare_to("a01", "a1"), Greater);

        let names = hud_item_choices();
        assert!(names.contains(&"groundspeed"));
        assert!(!names.contains(&"armed"), "a bool is not IsNumber");
        assert!(
            names
                .windows(2)
                .all(|pair| string_compare_to(pair[0], pair[1]) != Greater),
            "in StringCompareTo's order"
        );
        assert!(names.len() < crate::quick::choices().len());
    }

    // --- Jump To Tag ----------------------------------------------------------------------------

    /// `UInt16.TryParse`, then `DO_JUMP_TAG` through `doCommand`, which waits for its answer.
    #[test]
    fn jump_to_tag_takes_a_uint16_and_sends_do_jump_tag() {
        assert_eq!(parse_tag("5"), Some(5));
        assert_eq!(parse_tag(" 65535 "), Some(65_535));
        assert_eq!(parse_tag("+7"), Some(7));
        assert_eq!(parse_tag("-0"), Some(0));
        assert_eq!(parse_tag("65536"), None);
        assert_eq!(parse_tag("-1"), None);
        assert_eq!(parse_tag(""), None);
        assert_eq!(parse_tag("tag"), None);

        let message = jump_to_tag_message(target(), 42);
        assert_eq!(
            describe(&message),
            "COMMAND_LONG MAV_CMD_DO_JUMP_TAG 42,0,0,0,0,0,0"
        );
        assert_eq!(
            route(&message),
            Route::Command {
                target: target(),
                command: 601,
                params: [42.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }
        );
        assert_eq!(Prompt::JumpToTag.title(), "Jump to Tag");
        assert_eq!(Prompt::JumpToTag.text(), "Tag Id:");
        assert!(Prompt::JumpToTag.takes_text());
        assert_eq!(INVALID_TAG, "Invalid Tag. Must be a number from 0 to 65535");
    }

    #[test]
    fn the_hud_header_question_is_the_csharps() {
        assert_eq!(Prompt::HudHeader.title(), "Hud Header");
        assert_eq!(Prompt::HudHeader.text(), "Please enter your item prefix");
        assert!(Prompt::HudHeader.takes_text());
    }

    // --- The fourth batch ---------------------------------------------------------------------

    /// Every question and message box of the fourth batch is worded as `FlightData.cs` and
    /// `POI.cs` word it.
    #[test]
    fn the_fourth_batchs_words_are_the_csharps() {
        let (Some(source), Some(poi)) =
            (csharp("GCSViews/FlightData.cs"), csharp("Utilities/POI.cs"))
        else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        for prompt in [
            Prompt::SetHome,
            Prompt::SendMessage,
            Prompt::PointCameraAlt,
            Prompt::PointCameraCoords,
            Prompt::ViewColumns,
            Prompt::ViewRows,
            Prompt::CellCount,
            Prompt::GaugeMax,
        ] {
            assert!(
                source.contains(&format!("\"{}\"", prompt.title())),
                "{prompt:?}'s title"
            );
            assert!(
                source.contains(&format!("\"{}\"", prompt.text())),
                "{prompt:?}'s text"
            );
        }
        assert_eq!(Prompt::SetHome.buttons(), ("OK", "Cancel"));
        assert!(!Prompt::SetHome.takes_text());
        for text in [
            NO_SRTM,
            PLEASE_CONNECT,
            BAD_LAT_LONG,
            BAD_RADIUS,
            crate::transponder::CONNECT_AGAIN,
            crate::transponder::STATUS_LOST,
            crate::transponder::CONNECTED,
            crate::transponder::OFFLINE,
            crate::transponder::NO_STATUS,
            crate::transponder::TIMEOUT,
        ] {
            assert!(source.contains(&format!("\"{text}\"")), "{text}");
        }
        assert!(poi.contains("\"Poi File|*.txt\""));
        assert_eq!(Prompt::PoiSave.text(), "Poi File");
        assert_eq!(
            (Prompt::PoiSave.title(), Prompt::PoiLoad.title()),
            ("Save File", "Load File")
        );
    }

    /// `this.X.Items.AddRange(` or `.DropDownItems.AddRange(`'s items, by name.
    fn designer_items(designer: &str, header: &str) -> Vec<String> {
        designer
            .lines()
            .skip_while(|line| !line.trim().starts_with(header))
            .skip(1)
            .map(str::trim)
            .take_while(|line| line.starts_with("this.") && !line.contains(" = "))
            .map(|line| {
                line.trim_start_matches("this.")
                    .trim_end_matches(['}', ')', ';', ','])
                    .to_owned()
            })
            .collect()
    }

    /// The strip's menu and the quick views' menu are the Designer's, in its order, with the
    /// `.resx`'s words.
    #[test]
    fn the_strip_and_quick_view_menus_are_the_designers() {
        let (Some(designer), Some(resx)) = (
            csharp("GCSViews/FlightData.Designer.cs"),
            csharp("GCSViews/FlightData.resx"),
        ) else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let control = |entry: MenuEntry| match entry {
            MenuEntry::Customize => "customizeToolStripMenuItem",
            MenuEntry::MultiLine => "multiLineToolStripMenuItem",
            MenuEntry::SetViewCount => "setViewCountToolStripMenuItem",
            MenuEntry::Undock => "undockToolStripMenuItem",
        };
        for (kind, header) in [
            (
                MenuKind::Tabs,
                "this.contextMenuStripactionstab.Items.AddRange(",
            ),
            (
                MenuKind::Quick,
                "this.contextMenuStripQuickView.Items.AddRange(",
            ),
        ] {
            assert_eq!(
                designer_items(&designer, header),
                kind.entries()
                    .iter()
                    .map(|entry| control(*entry))
                    .collect::<Vec<_>>()
            );
            for entry in kind.entries() {
                assert_eq!(
                    resx_text(&resx, control(*entry)).as_deref(),
                    Some(entry.text())
                );
            }
        }
        assert!(MenuEntry::Undock.dimmed().is_some(), "one window");
        assert!(MenuEntry::SetViewCount.dimmed().is_none());
    }

    /// Set Home Here takes a height from a tile or the sea and refuses anything else; OK sends
    /// `DO_SET_HOME` at the point and then asks for home, Cancel only asks. The ask goes as a
    /// command the link does not wait on; `DO_SET_HOME` goes once, as every `COMMAND_INT` does.
    #[test]
    fn set_home_here_sends_do_set_home_and_asks_for_home_either_way() {
        use crate::srtm::{AltResponse, TileType};
        let valid = AltResponse {
            current_type: TileType::Valid,
            alt: 584.5,
            alt_source: "SRTM",
        };
        let sea = AltResponse {
            current_type: TileType::Ocean,
            alt: 0.0,
            alt_source: "Ocean",
        };
        assert_eq!(set_home_height(valid), Ok(584.5));
        assert_eq!(set_home_height(sea), Ok(0.0));
        assert_eq!(
            set_home_height(AltResponse::INVALID),
            Err(Refusal::error(NO_SRTM))
        );
        let described =
            |messages: Vec<MavMessage>| messages.iter().map(describe).collect::<Vec<_>>();
        let point = (-35.5, 149.25, 584.5);
        assert_eq!(
            described(set_home_messages(target(), point, true)),
            [
                "COMMAND_INT MAV_CMD_DO_SET_HOME 0,0,0,0 x=-355000000 y=1492500000 z=584.5 frame=0",
                "COMMAND_LONG MAV_CMD_GET_HOME_POSITION 0,0,0,0,0,0,0",
            ]
        );
        assert_eq!(
            described(set_home_messages(target(), point, false)),
            ["COMMAND_LONG MAV_CMD_GET_HOME_POSITION 0,0,0,0,0,0,0"]
        );
        let home = set_home_messages(target(), point, true);
        assert!(matches!(
            route(&home[0]),
            Route::CommandInt {
                command: commands::CMD_DO_SET_HOME,
                ..
            }
        ));
        assert_eq!(route(&home[1]), Route::GetHome { target: target() });
    }

    /// Set EKF Origin Here wants a tile's height - the sea will not do - and sends the whole
    /// metres as millimetres.
    #[test]
    fn set_ekf_origin_here_wants_a_tile() {
        use crate::srtm::{AltResponse, TileType};
        let valid = AltResponse {
            current_type: TileType::Valid,
            alt: 584.7,
            alt_source: "SRTM",
        };
        let sent = set_ekf_origin_sends(target(), (-35.5, 149.25), valid).expect("sent");
        assert_eq!(
            sent.iter().map(describe).collect::<Vec<_>>(),
            ["SET_GPS_GLOBAL_ORIGIN lat=-355000000 lon=1492500000 alt=584000"]
        );
        let sea = AltResponse {
            current_type: TileType::Ocean,
            alt: 0.0,
            alt_source: "Ocean",
        };
        assert_eq!(
            set_ekf_origin_sends(target(), (-35.5, 149.25), sea),
            Err(Refusal::error(NO_SRTM))
        );
        assert_eq!(
            set_ekf_origin_sends(target(), (0.0, 0.0), AltResponse::INVALID),
            Err(Refusal::error(NO_SRTM))
        );
    }

    /// The whole path from a tile on disk: the terrain height the SRTM reader finds is the
    /// height Set Home Here sends.
    #[test]
    fn set_home_here_sends_the_height_of_the_tile_under_the_press() {
        let dir =
            std::env::temp_dir().join(format!("headless-planner-fly-srtm-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("scratch folder");
        let mut tile = Vec::with_capacity(1201 * 1201 * 2);
        for _ in 0..1201 * 1201 {
            tile.extend_from_slice(&584i16.to_be_bytes());
        }
        std::fs::write(dir.join("S36E149.hgt"), tile).expect("the tile");
        let (lat, lng) = (-35.363_262_1, 149.165_237_4);
        let height = set_home_height(crate::srtm::altitude_in(&dir, lat, lng)).expect("a height");
        let sent = set_home_messages(target(), (lat, lng, height), true);
        assert!(
            describe(&sent[0]).contains("z=584 frame=0"),
            "{}",
            describe(&sent[0])
        );
        // No tile a degree west.
        assert_eq!(
            set_home_height(crate::srtm::altitude_in(&dir, lat, 148.9)),
            Err(Refusal::error(NO_SRTM))
        );
        std::fs::remove_dir_all(dir).ok();
    }

    /// Point Camera Here: the height is a float or "Bad Alt", then a point that is not the
    /// unpressed (0, 0) or "Bad Lat/Long", and `DO_SET_ROI` above home.
    #[test]
    fn point_camera_here_sends_set_roi_above_home() {
        let at = (-35.5, 149.25);
        let sent = point_camera_here_sends(target(), at, " 12.5 ").expect("sent");
        assert_eq!(
            describe(&sent[0]),
            "COMMAND_INT MAV_CMD_DO_SET_ROI 0,0,0,0 x=-355000000 y=1492500000 z=12.5 frame=3"
        );
        assert!(matches!(route(&sent[0]), Route::CommandInt { .. }));
        assert_eq!(
            point_camera_here_sends(target(), at, "high"),
            Err(Refusal::error(strings::BAD_ALT))
        );
        assert_eq!(
            point_camera_here_sends(target(), (0.0, 0.0), "5"),
            Err(Refusal::error(BAD_LAT_LONG))
        );
        // The height is looked at first.
        assert_eq!(
            point_camera_here_sends(target(), (0.0, 0.0), "x"),
            Err(Refusal::error(strings::BAD_ALT))
        );
    }

    /// Point Camera Coords: three floats are the height above sea level, two take the
    /// terrain's, in `doCommandInt`'s default frame; anything else is Invalid Field.
    #[test]
    fn point_camera_coords_takes_a_height_or_the_terrains() {
        let terrain = |_: f64, _: f64| 584.25;
        let sent = point_camera_coords_sends(target(), "-35.5;149.25;600", terrain).expect("sent");
        assert_eq!(
            describe(&sent[0]),
            "COMMAND_INT MAV_CMD_DO_SET_ROI 0,0,0,0 x=-355000000 y=1492500000 z=600 frame=0"
        );
        let sent = point_camera_coords_sends(target(), "-35.5;149.25", terrain).expect("sent");
        assert!(describe(&sent[0]).ends_with("z=584.25 frame=0"));
        for text in ["", "junk", "-35.5", "1;2;3;4", "a;b"] {
            assert_eq!(
                point_camera_coords_sends(target(), text, terrain),
                Err(Refusal::error(strings::INVALID_FIELD)),
                "{text}"
            );
        }
    }

    /// Message: a `STATUSTEXT` of severity 5, the text in ASCII cut to fifty.
    #[test]
    fn message_sends_the_text_as_a_notice() {
        assert_eq!(
            describe(&statustext(MESSAGE_SEVERITY, "hello")),
            "STATUSTEXT severity=5 text=hello"
        );
        assert_eq!(
            describe(&statustext(5, "h\u{e9}llo")),
            "STATUSTEXT severity=5 text=h?llo"
        );
        let long = "x".repeat(60);
        assert_eq!(
            describe(&statustext(5, &long)),
            format!("STATUSTEXT severity=5 text={}", "x".repeat(50))
        );
        assert_eq!(route(&statustext(5, "a")), Route::Raw);
    }

    /// `CMB_mountmode` lists the first documented mount-mode parameter's values; Set Mount writes
    /// `MNT_MODE` where the vehicle has it and sends `DO_MOUNT_CONTROL` where not, both waited
    /// for; with nothing to choose it is `Strings.ErrorNoResponse`.
    #[test]
    fn set_mount_writes_mnt_mode_or_sends_mount_control() {
        let modes = mount_modes(mp_params::param_meta::lookup);
        assert_eq!(modes.first(), Some(&(0, "Retracted".to_owned())));
        assert!(modes.contains(&(2, "MavLink Targeting".to_owned())));
        let with = vec![("MNT_MODE".to_owned(), 0.0)];
        let sent = mount_mode_sends(target(), &with, Some(2)).expect("sent");
        assert_eq!(describe(&sent[0]), "PARAM_SET MNT_MODE=2");
        let sent = mount_mode_sends(target(), &[], Some(2)).expect("sent");
        assert_eq!(
            describe(&sent[0]),
            "COMMAND_LONG MAV_CMD_DO_MOUNT_CONTROL 0,0,0,0,0,0,2"
        );
        assert!(matches!(
            route(&sent[0]),
            Route::Command { command: 205, .. }
        ));
        assert_eq!(
            mount_mode_sends(target(), &[], None),
            Err(Refusal::error(strings::ERROR_NO_RESPONSE))
        );
        fn nothing(_: &str) -> Option<&'static mp_params::ParamMeta> {
            None
        }
        assert!(mount_modes(nothing).is_empty());
    }

    /// `IsNumber` is `decimal.TryParse`; `setQuickViewRowsCols` then wants whole numbers, and at
    /// least one of each.
    #[test]
    fn set_view_count_takes_numbers_and_keeps_at_least_one() {
        for yes in ["3", " 2 ", "2.5", "-1", "+4", "1,000", "5-"] {
            assert!(is_number(yes), "{yes}");
        }
        for no in ["", "abc", "1.2.3", ".", "-", ",5"] {
            assert!(!is_number(no), "{no}");
        }
        assert_eq!(view_count("3", "2"), Ok((3, 2)));
        assert_eq!(view_count("0", "-5"), Ok((1, 1)));
        assert!(view_count("2.5", "3").is_err());
    }

    /// Battery Cell Voltage: with a count, the HUD is told to draw the cell line for it; off, it
    /// is told not to. The line itself is `hud.rs`'s (HUD.cs:2896-2923), from those inputs.
    #[test]
    fn the_cell_count_reaches_the_hud_as_its_own_inputs() {
        let mut settings = HudSettings::default();
        let mut inputs = crate::hud::HudInputs {
            has_vehicle: true,
            battery_voltage: 12.6,
            ..crate::hud::HudInputs::default()
        };
        settings.apply(&mut inputs, None);
        assert!(!inputs.display_cell_voltage);
        assert_eq!(inputs.battery_cell_count, 0);
        settings.cells = Some(3);
        settings.apply(&mut inputs, None);
        assert!(inputs.display_cell_voltage);
        assert_eq!(inputs.battery_cell_count, 3);
        let scene = crate::hud::scene(&inputs, 398.0, 258.0);
        assert!(
            scene.items.iter().any(|item| matches!(item, crate::hud::Item::Label { text, .. } if text.starts_with("Cell "))),
            "the HUD draws the cell line from the inputs"
        );
    }

    /// Battery Cell Voltage from the HUD's menu: off, it asks for the count from 4; on, it
    /// turns the line off without asking.
    #[test]
    fn battery_cell_voltage_asks_its_count_or_turns_off() {
        let row = HUD_MENU
            .iter()
            .find(|row| row.control == "setBatteryCellCountToolStripMenuItem")
            .expect("the row");
        assert_eq!(row.does, Ok(HudAction::BatteryCells));
        assert_eq!(Prompt::CellCount.title(), "Battery Cell Count");
        assert_eq!(dotnet_int(" 4 "), Some(4));
        assert_eq!(dotnet_int("four"), None);
        assert_eq!(dotnet_int("3.5"), None);
    }

    /// Customize lists every page, checked while the strip has it; closing it makes the checked
    /// ones the strip, showing the first. Nothing checked leaves the strip as it was, since
    /// `loadTabControlActions` returns on an empty setting - and the next Customize shows them
    /// all unchecked, as the empty setting names none.
    #[test]
    fn customize_chooses_the_strips_pages() {
        let mut pages = Pages::default();
        pages.select(Page::Status);
        let mut list = pages.customize_list();
        assert_eq!(list.len(), 14);
        assert!(list.iter().all(|(_, on)| *on));
        for (page, on) in &mut list {
            if matches!(page, Page::Gauges | Page::Status) {
                *on = false;
            }
        }
        pages.customize(&list);
        assert_eq!(pages.shown().len(), 12);
        assert!(!pages.shown().contains(&Page::Gauges));
        assert_eq!(pages.selected(), Page::Quick);
        assert_eq!(pages.first_shown(), 0);
        let again = pages.customize_list();
        assert!(again.iter().any(|(page, on)| *page == Page::Gauges && !on));
        assert!(again.iter().any(|(page, on)| *page == Page::Quick && *on));

        let none: Vec<(Page, bool)> = again.iter().map(|(page, _)| (*page, false)).collect();
        pages.customize(&none);
        assert_eq!(pages.shown().len(), 12, "an empty setting changes nothing");
        assert!(pages.customize_list().iter().all(|(_, on)| !on));

        // The arrows stop at the strip's own last header.
        let mut short = Pages::default();
        short.customize(&[(Page::Quick, true), (Page::Actions, true)]);
        short.scroll_right();
        assert!(!short.can_scroll_right());
        assert_eq!(short.first_shown(), 1);
    }

    /// MultiLine turns the strip's wrapping over.
    #[test]
    fn multiline_turns_the_strips_wrapping_over() {
        let mut pages = Pages::default();
        assert!(!pages.multiline);
        pages.toggle_multiline();
        assert!(pages.multiline);
        pages.toggle_multiline();
        assert!(!pages.multiline);
    }

    /// The transponder's messages read as the sent-message fact shows them.
    #[test]
    fn a_transponder_control_is_described_by_its_fields() {
        let message = crate::transponder::control(1200, 176, "QFA1");
        assert_eq!(
            describe(&message),
            "UAVIONIX_ADSB_OUT_CONTROL state=176 squawk=1200 flight_id=QFA1 baroaltmsl=2147483647"
        );
        assert_eq!(route(&message), Route::Raw);
    }

    // --- The Video drop-down's sources -----------------------------------------------------------

    /// The three questions, their first answers and HereLink's pipeline are worded as
    /// `FlightData.cs` words them.
    #[test]
    fn the_video_sources_questions_are_the_csharps() {
        assert_eq!(Prompt::MjpegUrl.title(), "Mjpeg url");
        assert_eq!(
            Prompt::MjpegUrl.text(),
            "Enter the url to the mjpeg source url"
        );
        assert_eq!(Prompt::GStreamerUrl.title(), "GStreamer url");
        assert!(
            Prompt::GStreamerUrl
                .text()
                .starts_with("Enter the source pipeline\nEnsure")
        );
        assert_eq!(Prompt::HereLinkIp.title(), "herelink ip");
        assert_eq!(Prompt::HereLinkIp.text(), "Enter herelink ip address");
        for prompt in [Prompt::MjpegUrl, Prompt::GStreamerUrl, Prompt::HereLinkIp] {
            assert!(prompt.takes_text());
            assert_eq!(prompt.buttons(), ("OK", "Cancel"));
        }
        let Some(source) = csharp("GCSViews/FlightData.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        for prompt in [Prompt::MjpegUrl, Prompt::GStreamerUrl, Prompt::HereLinkIp] {
            assert!(
                source.contains(&format!("\"{}\"", prompt.title())),
                "{prompt:?}'s title"
            );
            assert!(
                source.contains(&format!("\"{}\"", prompt.text().replace('\n', "\\n"))),
                "{prompt:?}'s text"
            );
        }
        assert!(source.contains(&format!("@\"{}\"", mp_video::gstreamer::DEFAULT_PIPELINE)));
        assert!(source.contains(&format!("@\"{}\"", mp_video::mjpeg::DEFAULT_URL)));
        assert!(source.contains(&format!(
            "string ipaddr = \"{}\";",
            mp_video::gstreamer::HERELINK_IP
        )));
        // `String.Format`'s `{0}` where the address goes.
        assert!(source.contains(&mp_video::gstreamer::herelink_pipeline("{0}")));
    }

    /// A flight screen with no points of interest and settings kept nowhere.
    fn video_screen() -> (FlightData, crate::settings::Persisted) {
        (
            FlightData::with_pois(crate::poi::Pois::kept_in(None)),
            crate::settings::Persisted::at(None),
        )
    }

    /// The runtime installed here, if it is: `gst-launch-1.0` on the `PATH`.
    fn installed_launcher() -> Option<std::path::PathBuf> {
        let path = std::env::var_os("PATH");
        let (on_path, _) = mp_video::gstreamer::search_dirs(path.as_ref(), None, None, &[]);
        let found = mp_video::gstreamer::look_for_gstreamer(&on_path, &[]);
        if found.is_none() {
            eprintln!("skipped: no gst-launch-1.0 on the PATH - install the GStreamer runtime");
        }
        found
    }

    /// Set GStreamer Source's OK, through the installed runtime: `gstreamer_url` and
    /// `gstlaunchexe` saved, the pipeline playing, its frame the HUD's picture at the caps'
    /// size; asked again, Cancel stops it and the picture goes.
    #[test]
    fn set_gstreamer_source_plays_the_pipeline_into_the_hud() {
        let Some(launcher) = installed_launcher() else {
            return;
        };
        let (mut data, mut persisted) = video_screen();
        let look = || Some(launcher.clone());
        let pipeline = "videotestsrc pattern=green ! video/x-raw,width=64,height=48 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink";
        assert_eq!(
            gstreamer_source(&mut data, &mut persisted, true, pipeline, &look),
            None
        );
        assert_eq!(persisted.get("gstreamer_url"), Some(pipeline));
        assert_eq!(
            persisted.get("gstlaunchexe"),
            Some(launcher.display().to_string().as_str())
        );
        assert!(data.hud_video_running());
        let deadline = Instant::now() + Duration::from_secs(20);
        let frame = loop {
            if let Some(frame) = data.hud_video_latest() {
                break frame;
            }
            assert!(Instant::now() < deadline, "{:?}", data.gstreamer.error());
            std::thread::sleep(Duration::from_millis(5));
        };
        assert_eq!((frame.width, frame.height), (64, 48));
        // videotestsrc's green is pure green.
        assert_eq!(&frame.rgba[..4], &[0, 255, 0, 255]);
        assert_eq!(
            gstreamer_source(&mut data, &mut persisted, false, "ignored", &look),
            None
        );
        assert!(!data.gstreamer.is_running());
        assert!(data.hud_video_latest().is_none());
        assert!(!data.hud_video_running());
        assert_eq!(persisted.get("gstreamer_url"), Some(pipeline));
    }

    /// No runtime and none to download here: the C#'s "The file was not found at" on the status
    /// line, `gstlaunchexe` emptied as `LookForGstreamer`'s `""` empties it, nothing playing.
    #[test]
    fn with_no_runtime_the_status_line_says_so() {
        if mp_video::gstreamer::can_download() {
            eprintln!("skipped: this machine would download the runtime");
            return;
        }
        let (mut data, mut persisted) = video_screen();
        let said = gstreamer_source(
            &mut data,
            &mut persisted,
            true,
            "videotestsrc ! appsink name=outsink",
            &|| None,
        );
        assert_eq!(
            said.as_deref(),
            Some(
                "The file was not found at gst-launch-1.0\nPlease verify permissions it is not on \
                 the PATH; install the GStreamer runtime"
            )
        );
        assert_eq!(persisted.get("gstlaunchexe"), Some(""));
        assert!(!data.gstreamer.is_running());
        assert!(data.gst_download.is_none());
    }

    /// HereLink Video saves the address typed, or on Cancel the one the box started with, and
    /// plays the air unit's RTSP stream.
    #[test]
    fn herelink_video_saves_the_address_and_plays_its_stream() {
        let (mut data, mut persisted) = video_screen();
        assert_eq!(herelink_ip(&persisted), "192.168.43.1");
        // A launcher that cannot run stands in, so nothing is reached over the network.
        let missing = || Some(std::path::PathBuf::from("/nonexistent/gst-launch-1.0"));
        let _ = herelink_video(&mut data, &mut persisted, true, "10.0.0.9", &missing);
        assert_eq!(persisted.get("herelinkip"), Some("10.0.0.9"));
        let _ = herelink_video(
            &mut data,
            &mut persisted,
            false,
            "typed then cancelled",
            &missing,
        );
        assert_eq!(persisted.get("herelinkip"), Some("10.0.0.9"));
        let Some(launcher) = installed_launcher() else {
            return;
        };
        let look = || Some(launcher.clone());
        assert_eq!(
            herelink_video(&mut data, &mut persisted, true, "127.0.0.1", &look),
            None
        );
        assert_eq!(
            data.gstreamer.pipeline(),
            Some(mp_video::gstreamer::herelink_pipeline("127.0.0.1").as_str())
        );
        data.gstreamer.stop();
    }

    /// Set MJPEG source: OK saves `mjpeg_url` and starts the capture over; Cancel stops it.
    #[test]
    fn set_mjpeg_source_starts_and_stops_the_capture() {
        let (mut data, mut persisted) = video_screen();
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let url = format!("http://127.0.0.1:{port}/video");
        assert_eq!(mjpeg_source(&mut data, &mut persisted, true, &url), None);
        assert_eq!(persisted.get("mjpeg_url"), Some(url.as_str()));
        assert_eq!(
            data.mjpeg.as_ref().map(mp_video::mjpeg::CaptureMjpeg::url),
            Some(url.as_str())
        );
        assert!(data.hud_video_running());
        assert_eq!(mjpeg_source(&mut data, &mut persisted, false, "x"), None);
        assert!(data.mjpeg.is_none());
        assert_eq!(persisted.get("mjpeg_url"), Some(url.as_str()));
    }
}
