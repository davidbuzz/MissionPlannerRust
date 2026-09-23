//! The flight screen: what the aircraft is doing, and the controls that change it.
//!
//! Equivalent to Mission Planner's Flight Data tab. The ordering on screen follows what a pilot
//! looks at, not what is easiest to lay out: attitude and mode first, then position and health,
//! then the message log, then the controls that commit something to the aircraft.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::commands;
use mp_link::messages::{LogMessage, Severity, time_of_day};
use mp_mavlink_dialects::all::{MavCmd, MavMessage};
use mp_mission::MissionItem;
use mp_vehicle::{VehicleFamily, VehicleId};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, action_sized, field, panel, progress, theme};
// Aliased because `mp_link::messages::Severity` is already `Severity` here and means something
// else entirely: that one is how loud a STATUSTEXT was, this one is how bad a reading is.
use mp_vehicle::health::Severity as Health;

/// Default height for the takeoff button, in metres above home.
///
/// Mission Planner asks for this in a dialog every time. A fixed, visible default is better for a
/// first pass than a text field nobody reads, and it is deliberately low: a wrong 10 is a hover,
/// a wrong 100 is an incident.
pub const TAKEOFF_ALTITUDE: f32 = 10.0;

/// Width shared by the arm and force arm buttons.
///
/// Wide enough for "force arm" so both are the same size. They are the same decision with
/// different force, and two controls that do nearly the same thing should not look like different
/// kinds of control.
const ARM_BUTTON_WIDTH: gpui::Pixels = px(104.0);

/// What the aircraft is and whether it is armed.
///
/// The altitude is `cs.alt`: above home, or above sea level while Set Home Alt is on.
pub fn vehicle_panel(view: &TelemetryView, alt_offset_home: f32) -> impl IntoElement {
    let (armed, armed_colour) = match view.state.as_ref() {
        Some(s) if s.armed => ("ARMED".to_owned(), theme::ALERT),
        Some(_) => ("disarmed".to_owned(), theme::TEXT),
        None => ("no vehicle".to_owned(), theme::DIM),
    };
    let position = view.state.as_ref().and_then(|s| s.position).map_or_else(
        || "no position".to_owned(),
        |p| format!("{:.6}, {:.6}", p.latitude(), p.longitude()),
    );
    let altitude = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |s| {
            format!(
                "{:.1} m",
                displayed_altitude(s.altitude_relative.0, alt_offset_home)
            )
        },
    );
    let speed = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |s| format!("{:.1} m/s", s.ground_speed.0),
    );
    let heading = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |s| format!("{:.0}°", s.heading.degrees()),
    );

    panel(
        "vehicle",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(field("state", armed, armed_colour))
            .child(field("position", position, theme::TEXT))
            .child(
                div()
                    .flex()
                    .gap_3()
                    .child(field("altitude", altitude, theme::TEXT))
                    .child(field("ground speed", speed, theme::TEXT))
                    .child(field("heading", heading, theme::TEXT)),
            ),
    )
}

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
        .child(action(
            "takeoff",
            format!("take off {TAKEOFF_ALTITUDE:.0} m"),
            theme::ACCENT,
            has_vehicle && armed,
            cx.listener(|this, _event: &(), _window, cx| {
                this.telemetry.takeoff(TAKEOFF_ALTITUDE);
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

pub fn hud_panel(inputs: &crate::hud::HudInputs) -> impl IntoElement {
    // Everything on the display is in the scene now - the tapes carry their own numbers, the
    // mode and waypoint sit under the altitude scroller, battery and GPS along the bottom - as
    // `HUD.cs` paints them, so nothing is overlaid as widgets any more. The box is 16:9-ish and
    // tall enough that the C#'s `Height / 30` font is legible.
    let inputs = inputs.clone();
    div()
        .relative()
        .h(px(260.0))
        .w_full()
        .overflow_hidden()
        .rounded_md()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .child(
            gpui::canvas(
                |_bounds, _window, _cx| (),
                move |bounds, (), window, cx| {
                    let scene = crate::hud::scene(
                        &inputs,
                        f32::from(bounds.size.width),
                        f32::from(bounds.size.height),
                    );
                    crate::hud::paint(&scene, bounds, window, cx);
                },
            )
            .size_full(),
        )
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

/// The estimator and the frame, in detail.
///
/// A vehicle that will not arm, drifts in a hover or climbs when told to hold is almost always one
/// of these two, and neither is visible anywhere else. The bars are the point: a variance is a
/// number nobody can read at a glance, and a bar against a threshold is a picture anybody can.
pub fn estimator_panel(view: &TelemetryView) -> AnyElement {
    let Some(state) = view.state.as_ref() else {
        return div().into_any_element();
    };
    // Nothing shown until something has been heard. An estimator panel full of zeroes on a vehicle
    // that has never sent one reads as a perfectly healthy vehicle, which is the opposite of the
    // truth.
    if !state.ekf.seen && !state.vibration.seen {
        return panel(
            "estimator",
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("no EKF or vibration reports yet"),
        )
        .into_any_element();
    }

    let mut rows = div().flex().flex_col().gap_1();
    if state.ekf.seen {
        for (name, variance) in state.ekf.variances() {
            let severity = if variance >= mp_vehicle::health::VARIANCE_BAD {
                Health::Bad
            } else if variance >= mp_vehicle::health::VARIANCE_WARNING {
                Health::Warning
            } else {
                Health::Good
            };
            rows = rows.child(bar_row(
                name,
                variance,
                1.0,
                severity,
                format!("{variance:.2}"),
            ));
        }
        // The flags say *why* a mode change was refused, which the variances never do.
        //
        // Two lines, because two different things are being said. Amber is what a pilot should do
        // something about; grey is the rest, shown because this is the detail view and somebody
        // reading it wants the lot - a vehicle with no rangefinder never gets a height-above-
        // ground estimate and that is not a fault.
        let missing = state.ekf.unhealthy_estimates();
        if !missing.is_empty() {
            rows = rows.child(
                div()
                    .pt_1()
                    .text_xs()
                    .text_color(rgb(theme::WARN))
                    .child(format!("not yet good: {}", missing.join(", "))),
            );
        }
        let unavailable: Vec<_> = state
            .ekf
            .all_unhealthy_estimates()
            .into_iter()
            .filter(|name| !missing.contains(name))
            .collect();
        if !unavailable.is_empty() {
            rows = rows.child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(format!("not estimated: {}", unavailable.join(", "))),
            );
        }
        for (set, text) in [
            (
                state.ekf.uninitialised(),
                "the estimator has never been healthy",
            ),
            (
                state.ekf.constant_position_mode(),
                "holding attitude only - no position source",
            ),
            (
                state.ekf.gps_glitching(),
                "the estimator is rejecting the GPS",
            ),
        ] {
            if set {
                rows = rows.child(div().text_xs().text_color(rgb(theme::ALERT)).child(text));
            }
        }
    }

    if state.vibration.seen {
        for (axis, level) in state.vibration.axes() {
            let severity = if level >= mp_vehicle::health::VIBRATION_BAD {
                Health::Bad
            } else if level >= mp_vehicle::health::VIBRATION_WARNING {
                Health::Warning
            } else {
                Health::Good
            };
            rows = rows.child(bar_row(
                &format!("vibration {axis}"),
                level,
                mp_vehicle::health::VIBRATION_BAD,
                severity,
                format!("{level:.1}"),
            ));
        }
        let clipping = state.vibration.total_clipping();
        if clipping > 0 {
            rows = rows.child(
                div()
                    .pt_1()
                    .text_xs()
                    .text_color(rgb(theme::ALERT))
                    .child(format!(
                        "{clipping} clipping events - an accelerometer has been driven past its range"
                    )),
            );
        }
    }

    panel("estimator", rows).into_any_element()
}

/// A labelled bar against a full-scale value.
fn bar_row(
    label: &str,
    value: f32,
    full_scale: f32,
    severity: Health,
    shown: String,
) -> impl IntoElement {
    let fraction = if full_scale > 0.0 {
        (value / full_scale).clamp(0.0, 1.0)
    } else {
        0.0
    };
    div()
        .flex()
        .items_center()
        .gap_2()
        .child(
            div()
                .w(px(120.0))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(label.to_owned()),
        )
        .child(
            div()
                .flex_1()
                .min_w(px(0.0))
                .child(progress(fraction, health_colour(severity))),
        )
        .child(
            div()
                .w(px(40.0))
                .text_xs()
                .text_color(rgb(health_colour(severity)))
                .child(shown),
        )
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
    /// The literal `CustomMessageBox.Show("Bad Alt")` in `flyToHereAltToolStripMenuItem_Click`.
    pub const BAD_ALT: &str = "Bad Alt";
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
}

impl Prompt {
    /// The dialog's title.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::ConfirmAction(_) => "Action",
            Self::ResumeWarning => "Resume Mission",
            Self::ResumeAt => "Resume at",
            Self::FlyToCoords => "Enter Fly To Coords",
            Self::FlyToHereAlt { .. } => "Enter Alt",
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
        }
    }

    /// Whether it takes typed text, as an `InputBox` does.
    #[must_use]
    pub const fn takes_text(self) -> bool {
        matches!(
            self,
            Self::ResumeAt | Self::FlyToCoords | Self::FlyToHereAlt { .. }
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
/// message when the vehicle refuses the command - needs the refusal first, and is the only part
/// of this list not sent here.
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

/// Set Home Alt: `cs.altoffsethome` goes to zero if it is set, and to minus the home altitude if
/// it is not - which makes every altitude shown a height above sea level instead of above home.
///
/// `HomeAlt` is `HomeLocation.Alt`. This application's vehicle state keeps the home position but
/// not its height, so it is taken as `GLOBAL_POSITION_INT.alt - relative_alt`, which ArduPilot
/// computes as exactly that.
/// `// C#: GCSViews/FlightData.cs:1236-1247, ExtLibs/ArduPilot/CurrentState.cs:1568-1572`
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
    /// The message log, for the vehicle's answer to a takeoff.
    pub messages: &'a [LogMessage],
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
/// climb. Each attempt is one send; the C#'s `doARM` and `doCommand` also wait for the vehicle's
/// acknowledgement inside the attempt, which this does not.
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
    /// The newest message when the last takeoff went out, so its answer can be found.
    takeoff_sent_after: u64,
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
                takeoff_sent_after: 0,
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

    /// Moves on as far as this frame allows.
    pub fn advance(&mut self, input: &ResumeInput<'_>) -> Vec<ResumeStep> {
        let now = input.now;
        if self.finished() {
            return Vec::new();
        }
        let Some(target) = input.target else {
            return self.fail(strings::ERROR_NO_RESPONSE);
        };
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
                Attempt::Again => vec![ResumeStep::Send(vec![commands::arm(target, true, false)])],
            },
            ResumePhase::Takeoff => {
                // `if (!doCommand(... TAKEOFF ...)) { Show(CommandFailed); return; }`
                if self.last_attempt.is_some()
                    && let Some(answer) = ack_line(
                        input.messages,
                        commands::CMD_NAV_TAKEOFF,
                        self.takeoff_sent_after,
                    )
                    && answer.severity == Severity::Error
                {
                    return self.fail(strings::COMMAND_FAILED);
                }
                let climbed = input.altitude >= self.takeoff_altitude - 2.0;
                match self.attempt(now, climbed, 40) {
                    Attempt::Wait => Vec::new(),
                    Attempt::GiveUp => self.fail(strings::ERROR_NO_RESPONSE),
                    Attempt::Met => {
                        self.enter(ResumePhase::Auto, now);
                        self.advance(input)
                    }
                    Attempt::Again => {
                        self.takeoff_sent_after = input.messages.last().map_or(0, |m| m.seq);
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
    /// `cs.altoffsethome`.
    pub alt_offset_home: f32,
    /// `cs.lastautowp`: the last waypoint flown to in Auto, -1 before there is one.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:119, 3422`
    pub last_auto_wp: i32,
    /// A Resume Mission in progress, or the last one.
    pub resume: Option<Resume>,
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
            speed: ModifyAndSet::speed(),
            alt: ModifyAndSet::alt(),
            loiter_rad: ModifyAndSet::loiter_rad(),
            prompt: None,
            prompt_field: TextField::new(""),
            guided: GuidedMode::default(),
            guided_alt_setting: None,
            guided_frame_setting: None,
            alt_offset_home: 0.0,
            last_auto_wp: -1,
            resume: None,
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
            "fly.home_alt",
            if self.alt_offset_home == 0.0 {
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
}

impl ActionsFocus {
    /// Four new handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            speed: cx.focus_handle(),
            alt: cx.focus_handle(),
            loiter_rad: cx.focus_handle(),
            prompt: cx.focus_handle(),
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
        .py_1()
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

/// The Actions tab: the grid, the list a combo has open, and the map menu's two entries. The
/// question a press asks is not here: it is a dialog over the window, [`prompt_dialog`].
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
                    cx.notify();
                }),
            ),
        ))
        .child(cell(
            1,
            0,
            grid_button(
                "fly-doaction",
                "Do Action",
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
                "Set Home Alt",
                if tab.alt_offset_home == 0.0 {
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
                "Set WP",
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
                "Restart Mission",
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
        // Row 4: -, -, (BUT_SendMSG), BUT_resumemis, BUT_abortland.
        .child(cell(
            3,
            4,
            grid_button(
                "fly-resumemis",
                "Resume Mission",
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
                "Abort Landing",
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

    // The map's context menu in the C#. This application's map has no menu - a right click flies
    // there, which is the menu's Fly To Here - so the two entries that need no point on the map
    // are here, under the grid.
    body = body.child(
        div()
            .flex()
            .items_center()
            .gap_1()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("map menu"),
            )
            .child(action(
                "fly-flytocoords",
                "Fly To Coords",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), window, cx| {
                    this.fly_actions.ask(Prompt::FlyToCoords, "");
                    this.fly_focus.prompt.focus(window, cx);
                    cx.notify();
                }),
            ))
            .child(action(
                "fly-flytohere-alt",
                "Fly To Here Alt",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), window, cx| {
                    this.fly_ask_guided_alt();
                    this.fly_focus.prompt.focus(window, cx);
                    cx.notify();
                }),
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
        .w(px(340.0))
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
        .with_priority(2)
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

    /// The header's words: the page's `Text` in `FlightData.resx`. Both Actions pages are
    /// "Actions" there, and so both are here.
    /// `// C#: GCSViews/FlightData.resx:580, 1384, 1441, 1555, 1606, 2531, 3014, 3044, 3692, 3890, 4124, 4424, 4925, 5171`
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Quick => "Quick",
            Self::Actions | Self::ActionsSimple => "Actions",
            Self::Messages => "Messages",
            Self::PreFlight => "PreFlight",
            Self::Gauges => "Gauges",
            Self::Transponder => "Transponder",
            Self::Status => "Status",
            Self::Servo => "Servo/Relay",
            Self::AuxFunction => "Aux Function",
            Self::Scripts => "Scripts",
            Self::Payload => "Payload Control",
            Self::TLogs => "Telemetry Logs",
            Self::LogBrowse => "DataFlash Logs",
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
            // `// C#: GCSViews/FlightData.Designer.cs:626-631, 713, 726` - quickView1 to 6, bound to alt,
            // groundspeed, wp_dist, yaw, verticalspeed and DistToHome.
            Self::Quick => {
                "quickView1-6 are not ported; the vehicle panel shows three of their six numbers: \
                 altitude, ground speed and heading."
            }
            Self::Actions => return None,
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
            Self::Gauges => "Gspeed, Galt, Gheading and Gvspeed, the four dials, are not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:1612-1626`
            Self::Transponder => {
                "The transponder controls (Connect to Transponder, STBY, ON, ALT, IDENT, squawk, \
                 flight ID) are not ported."
            }
            // `// C#: GCSViews/FlightData.cs:6045-6072`, painted from `cs.GetItemList`.
            Self::Status => "tabStatus, every CurrentState field by name, is not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:1754, 1762-1789`
            Self::Servo => "servoOptions1-12 and relayOptions1-16 are not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:1962, 1969-1975`
            Self::AuxFunction => "auxOptions1-7 are not ported.",
            // `// C#: GCSViews/FlightData.Designer.cs:2016-2022`
            Self::Scripts => {
                "Select Script, Run Script, Abort Running Script and Edit Selected Script are \
                 not ported."
            }
            // `// C#: GCSViews/FlightData.Designer.cs:2091-2095`
            Self::Payload => {
                "The gimbal controls (pitch, roll and yaw, Reset Position, Video Control) are \
                 not ported."
            }
            // `// C#: GCSViews/FlightData.Designer.cs:2195, 2203-2207`
            Self::TLogs => {
                "Load Log is a file: link here; Play/Pause and the rest of \
                 tableLayoutPaneltlogs are not ported."
            }
            // `// C#: GCSViews/FlightData.Designer.cs:2374`
            Self::LogBrowse => {
                "Review a Log is the Logs screen and Download DataFlash Log Via Mavlink is on the \
                 Setup screen; the rest of tableLayoutPanel2 is not ported."
            }
        })
    }
}

/// A panel this screen draws on a page.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Panel {
    /// [`vehicle_panel`].
    Vehicle,
    /// [`actions_panel`]: the arm and command buttons, the Actions grid and the mode list.
    Actions,
    /// [`prearm_panel`].
    PreArm,
    /// [`health_panel`].
    Health,
}

impl Page {
    /// The panels the page shows, top to bottom.
    ///
    /// - Quick: the vehicle panel, whose altitude, ground speed and heading are three of the six
    ///   numbers `quickView1-6` show by default.
    /// - Actions: the arm, mode and command controls with the 5x5 grid - `BUT_ARM` and
    ///   `CMB_modes` are cells of `tableLayoutPanel1` on `tabActions`
    ///   (`// C#: GCSViews/FlightData.Designer.cs:739, 757, 767`).
    /// - PreFlight: the pre-arm and health panels. Neither has a page in the C#: the pre-arm and
    ///   EKF and vibration detail are windows the HUD's own indicators open
    ///   (`// C#: ExtLibs/Controls/HUD.cs:1207-1231`). This is the page about being ready to fly,
    ///   and the fix, satellites, link and battery the health panel shows are what
    ///   `checkListControl1` checks by default (`// C#: checklistDefault.xml`).
    ///
    /// The messages stay under the map, where this screen has always had them, and the tuning
    /// graph is above the map, where the C# has it.
    #[must_use]
    pub const fn panels(self) -> &'static [Panel] {
        match self {
            Self::Quick => &[Panel::Vehicle],
            Self::Actions => &[Panel::Actions],
            Self::PreFlight => &[Panel::PreArm, Panel::Health],
            _ => &[],
        }
    }
}

/// The page shown at start: `tabControlactions.SelectedIndex = 0`, the first page added.
/// `// C#: GCSViews/FlightData.Designer.cs:611`
pub const DEFAULT_PAGE: Page = Page::ALL[0];

/// Which page is showing, and which header the strip starts from.
///
/// `Multiline` is off by default (`// C#: GCSViews/FlightData.cs:429`), so the headers are one
/// row and the ones that do not fit are reached with the two arrows a `TabControl` puts at the
/// end of the row, each moving the row along by one header.
#[derive(Debug)]
pub struct Pages {
    selected: Page,
    first_shown: usize,
}

impl Default for Pages {
    fn default() -> Self {
        Self {
            selected: DEFAULT_PAGE,
            first_shown: 0,
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
    pub const fn can_scroll_right(&self) -> bool {
        self.first_shown + 1 < Page::ALL.len()
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

    /// Publishes what a UI test asserts on: the page showing, by the Designer's name and by its
    /// header, the strip's order both ways, where the row starts, and how far the page runs past
    /// the bottom of the column - `overflow`, the page's scroll range, which is zero when it fits.
    pub fn record_facts(&self, overflow: f32) {
        crate::facts::record("fly.tab", self.selected.name());
        crate::facts::record("fly.tab.text", self.selected.text());
        crate::facts::record("fly.tabs", page_list(Page::name));
        crate::facts::record("fly.tabs.text", page_list(Page::text));
        crate::facts::record("fly.tabs.first", self.first_shown);
        crate::facts::record("fly.page.overflow", format!("{:.0}", overflow.max(0.0)));
    }
}

/// Every page's name or text, in the strip's order, joined with commas.
fn page_list(of: fn(Page) -> &'static str) -> String {
    Page::ALL.map(of).join(",")
}

/// The header row: one header per page from the first shown on, clipped at the column's edge,
/// and the two arrows at the end of the row.
pub fn page_strip(pages: &Pages, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    let mut row = div()
        .flex()
        .flex_1()
        .min_w(px(0.0))
        .overflow_hidden()
        .gap_1();
    for page in Page::ALL.iter().copied().skip(pages.first_shown()) {
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
        .flex()
        .flex_shrink_0()
        .items_end()
        .gap_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(row)
        .child(
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
                )),
        )
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
            Panel::Vehicle => {
                vehicle_panel(view, inputs.actions.alt_offset_home).into_any_element()
            }
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
    fn fly_send(&mut self, sends: Sends, view: &TelemetryView) {
        let messages = match sends {
            Ok(messages) if !messages.is_empty() => messages,
            Ok(_) => {
                self.fly_actions.record_nothing("nothing to send");
                return;
            }
            Err(refusal) => {
                self.fly_actions.record_nothing(refusal.text());
                self.file_status = Some(match refusal {
                    Refusal::Error(text) => format!("Error: {text}"),
                    Refusal::Quiet(text) => text,
                });
                return;
            }
        };
        let Some((sender, _)) = self.telemetry.send_handle() else {
            self.fly_actions.record_nothing("no vehicle");
            self.file_status = Some("no vehicle to send to".to_owned());
            return;
        };
        // Every message is offered to the link, even after one is refused: a loop rather than
        // `all`, which would stop at the first refusal.
        let mut queued = true;
        for message in &messages {
            queued &= sender.send(message);
        }
        self.fly_actions
            .record(&messages, view.messages.last().map_or(0, |m| m.seq));
        self.file_status = Some(if queued {
            format!("sent {}", self.fly_actions.last_sent)
        } else {
            "the link has closed; nothing was sent".to_owned()
        });
    }

    /// Builds a press's messages for the vehicle being flown and sends them.
    fn fly_press(&mut self, build: impl FnOnce(&mut Actions, VehicleId, &TelemetryView) -> Sends) {
        let view = self.telemetry.view();
        let sends = match self.telemetry.send_handle() {
            Some((_, target)) => build(&mut self.fly_actions, target, &view),
            None => Err(Refusal::quiet("no vehicle")),
        };
        self.fly_send(sends, &view);
    }

    /// `CMB_setwp_Click`: the list rebuilt, and opened.
    /// `// C#: GCSViews/FlightData.cs:2542-2582`
    fn fly_setwp_list_click(&mut self) {
        let view = self.telemetry.view();
        self.fly_actions
            .refresh_setwp(&view.parameters, view.mission.len());
        self.fly_actions.setwp_open = !self.fly_actions.setwp_open;
        self.fly_actions.action_open = false;
    }

    /// Set WP: `setWPCurrent(sysid, compid, (ushort) CMB_setwp.SelectedIndex)`.
    /// `// C#: GCSViews/FlightData.cs:1658-1672`
    fn fly_set_wp(&mut self) {
        let seq = u16::try_from(self.fly_actions.setwp_selected).unwrap_or(u16::MAX);
        self.fly_press(|_, target, _| Ok(vec![commands::mission_set_current(target, seq)]));
    }

    /// Restart Mission: `setWPCurrent(sysid, compid, 0)`.
    /// `// C#: GCSViews/FlightData.cs:1881-1895`
    fn fly_restart_mission(&mut self) {
        self.fly_press(|_, target, _| Ok(vec![commands::mission_set_current(target, 0)]));
    }

    /// Change Alt: `setNewWPAlt(new Locationwp {alt = (int) Value / multiplieralt})`.
    /// `// C#: GCSViews/FlightData.cs:4399-4410`
    fn fly_change_alt(&mut self) {
        self.fly_press(|actions, target, _| change_alt_sends(target, actions.alt.commit()));
    }

    /// Change Speed: `DO_CHANGE_SPEED` with the box's number, undivided.
    /// `// C#: GCSViews/FlightData.cs:4426-4438`
    fn fly_change_speed(&mut self) {
        self.fly_press(|actions, target, _| change_speed_sends(target, actions.speed.commit()));
    }

    /// Set Loiter Rad: the first of `LOITER_RAD` and `WP_LOITER_RAD` the vehicle has.
    /// `// C#: GCSViews/FlightData.cs:4412-4424`
    fn fly_set_loiter_rad(&mut self) {
        self.fly_press(|actions, target, view| {
            let value = actions.loiter_rad.commit();
            loiter_rad_messages(target, &view.parameters, value)
        });
    }

    /// Abort Landing: `doAbortLand`, only with the link open.
    /// `// C#: GCSViews/FlightData.cs:1019-1032`
    fn fly_abort_land(&mut self) {
        self.fly_press(|_, target, view| {
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
        let home = view.state.as_ref().map_or(0.0, |state| {
            state.altitude_msl.0 - state.altitude_relative.0
        });
        self.fly_actions.alt_offset_home = toggle_home_alt(self.fly_actions.alt_offset_home, home);
        self.file_status = Some(if self.fly_actions.alt_offset_home == 0.0 {
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

    /// Sends one `CMB_action` entry.
    fn fly_run_action(&mut self, index: usize) {
        let action = ACTIONS.get(index).copied().unwrap_or(ACTIONS[0]);
        self.fly_press(|_, target, view| {
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
            self.fly_send(Err(Refusal::error(strings::COMMAND_FAILED)), &view);
            return;
        };
        let (resume, steps) = Resume::start(resume_at, Instant::now());
        self.fly_actions.resume = Some(resume);
        self.fly_resume_steps(steps, &view);
    }

    /// Does what a Resume Mission asked for this frame.
    fn fly_resume_steps(&mut self, steps: Vec<ResumeStep>, view: &TelemetryView) {
        for step in steps {
            match step {
                ResumeStep::DownloadMission => self.telemetry.request_mission(),
                ResumeStep::UploadMission(items) => self.telemetry.upload_mission(items),
                ResumeStep::ReadIntoPlan => {
                    self.adopt_vehicle_mission = true;
                    self.telemetry.request_mission();
                }
                ResumeStep::Send(messages) => self.fly_send(Ok(messages), view),
            }
        }
    }

    /// Fly To Coords: `lat;long;alt` or `lat;long`, flown to in Guided.
    /// `// C#: GCSViews/FlightData.cs:5938-6008`
    fn fly_to_coords(&mut self, text: &str) {
        let coords = parse_coords(text);
        self.fly_press(|actions, target, view| {
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

    /// Fly To Here Alt, once answered: the height and frame the next Fly To Here uses, and - if
    /// the vehicle is already in Guided - the current target moved to that height.
    /// `// C#: GCSViews/FlightData.cs:2920-2943`
    fn fly_to_here_alt(&mut self, text: &str, frame: u8) {
        self.fly_actions.guided_alt_setting = Some(text.to_owned());
        self.fly_actions.guided_frame_setting = Some(frame);
        let Ok(whole) = text.trim().parse::<i32>() else {
            let view = self.telemetry.view();
            self.fly_send(Err(Refusal::error(strings::BAD_ALT)), &view);
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
        self.fly_press(|actions, target, view| {
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
        self.fly_press(|actions, target, view| {
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

    /// Once a frame: `cs.lastautowp`, and a Resume Mission moved on.
    pub(crate) fn fly_tick(&mut self, view: &TelemetryView) {
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
                displayed_altitude(state.altitude_relative.0, self.fly_actions.alt_offset_home)
            }),
            family: family(view),
            target: self.telemetry.send_handle().map(|(_, id)| id),
            messages: &view.messages,
        });
        // Said once, on the frame it ends; the strip under the grid keeps saying it after that.
        match resume.phase() {
            ResumePhase::Failed(why) => self.file_status = Some(format!("Error: {why}")),
            ResumePhase::Done => {
                self.file_status = Some(format!("Resume Mission: {}", resume.label()))
            }
            _ => {}
        }
        self.fly_actions.resume = Some(resume);
        self.fly_resume_steps(steps, view);
    }
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
                .join("../../referneces/missionplanner")
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
        messages: Vec<LogMessage>,
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
                messages: Vec::new(),
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
                messages: &self.messages,
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
        vehicle.messages.push(LogMessage {
            from: target(),
            severity: Severity::Error,
            text: "MAV_CMD_NAV_TAKEOFF: denied".to_owned(),
            seq: 1,
            received: 0,
        });
        vehicle.later(1000);
        resume.advance(&vehicle.input());
        assert_eq!(
            resume.phase(),
            &ResumePhase::Failed(strings::COMMAND_FAILED.to_owned())
        );
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
            page_list(Page::text),
            "Quick,Actions,Messages,Actions,PreFlight,Gauges,Transponder,Status,Servo/Relay,\
             Aux Function,Scripts,Payload Control,Telemetry Logs,DataFlash Logs"
        );
        assert_eq!(
            page_list(Page::name),
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
        for panel in [Panel::Vehicle, Panel::Actions, Panel::PreArm, Panel::Health] {
            let pages: Vec<Page> = Page::ALL
                .into_iter()
                .filter(|page| page.panels().contains(&panel))
                .collect();
            assert_eq!(pages.len(), 1, "{panel:?} is on {pages:?}");
        }
        assert_eq!(Page::Actions.panels(), &[Panel::Actions]);
        assert_eq!(Page::Quick.panels(), &[Panel::Vehicle]);
        assert_eq!(Page::PreFlight.panels(), &[Panel::PreArm, Panel::Health]);
        for page in Page::ALL {
            assert_eq!(
                page.note().is_none(),
                page == Page::Actions,
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
}
