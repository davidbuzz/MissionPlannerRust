//! The setup screen: calibration and first-flight configuration.
//!
//! Equivalent to Mission Planner's Initial Setup tab. Deliberately last: an aircraft that cannot
//! be flown or planned for is not made useful by being configurable, and the calibration
//! procedures each need their own protocol work (`MAV_CMD_PREFLIGHT_CALIBRATION`, the accelerometer
//! position sequence over `COMMAND_ACK`, compass sampling over `MAG_CAL_PROGRESS`).
//!
//! What is here now is the part that needs no new protocol: the vehicle's identity, and an honest
//! list of what each calibration will do once implemented. A screen that showed a Calibrate button
//! which did nothing would be worse than one that says so.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_link::calibration::{AccelCalibration, AccelPosition, CompassProgress, CompassStatus};
use mp_mavlink_dialects::all::{MavAutopilot, MavType};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, field, panel, progress as progress_bar, theme};

/// The calibrations that are a single command.
///
/// Each is `MAV_CMD_PREFLIGHT_CALIBRATION` with one parameter set - the definitions are explicit
/// that only one may be set per message - and each completes on its own without a conversation.
const SINGLE_SHOT: &[(&str, &str, &str)] = &[
    (
        "cal-level",
        "level",
        "tells the vehicle that however it is sitting now is level; run it after mounting the \
         autopilot even slightly askew",
    ),
    (
        "cal-baro",
        "ground pressure",
        "re-zeroes the barometer; worth doing before a flight when the weather has changed",
    ),
];

/// What the aircraft says it is.
pub fn identity_panel(view: &TelemetryView) -> impl IntoElement {
    let vehicle = view.state.as_ref().map_or_else(
        || "no vehicle".to_owned(),
        |state| {
            MavType(u32::from(state.vehicle_type)).name().map_or_else(
                || format!("type {}", state.vehicle_type),
                |name| name.trim_start_matches("MAV_TYPE_").to_lowercase(),
            )
        },
    );
    let autopilot = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |state| {
            MavAutopilot(u32::from(state.autopilot)).name().map_or_else(
                || format!("autopilot {}", state.autopilot),
                |name| name.trim_start_matches("MAV_AUTOPILOT_").to_lowercase(),
            )
        },
    );
    let identity = view.vehicle.map_or_else(
        || "--".to_owned(),
        |id| format!("system {}, component {}", id.sysid, id.compid),
    );

    panel(
        "airframe",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .flex()
                    .gap_2()
                    .child(field("type", vehicle, theme::TEXT))
                    .child(field("autopilot", autopilot, theme::TEXT)),
            )
            .child(field("mavlink identity", identity, theme::TEXT)),
    )
}

/// The accelerometer calibration: a conversation, one position at a time.
///
/// The vehicle asks for each of six orientations and waits to be told the airframe is in it. The
/// instruction is shown as something to do rather than as a name, because getting an orientation
/// wrong produces a calibration that is wrong in a way which only shows up in flight.
pub fn accelerometer_panel(
    state: AccelCalibration,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    let armed = view.state.as_ref().is_some_and(|state| state.armed);

    let body = match state {
        AccelCalibration::Waiting(position) => div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .text_lg()
                    .text_color(rgb(theme::WARN))
                    .child(position.instruction()),
            )
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("hold it still, then confirm"),
            )
            .child(action(
                "cal-accel-confirm",
                format!("{} - done", position.label()),
                theme::OK,
                true,
                cx.listener(move |this, _event: &(), _window, cx| {
                    this.telemetry.confirm_accelerometer_position(position);
                    cx.notify();
                }),
            ))
            .into_any_element(),
        AccelCalibration::Succeeded => div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::OK))
                    .child("calibration accepted - reboot for it to take effect"),
            )
            .child(restart_button(cx))
            .into_any_element(),
        AccelCalibration::Failed => div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::ALERT))
                    .child("calibration rejected - the airframe usually moved during a sample"),
            )
            .child(restart_button(cx))
            .into_any_element(),
        AccelCalibration::Idle => div()
            .flex()
            .flex_col()
            .gap_2()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(
                "six positions, in this order: level, left, right, nose down, nose up, \
                         back. The vehicle asks for each one.",
            ))
            .child(action(
                "cal-accel-start",
                "start accelerometer calibration",
                theme::WARN,
                has_vehicle && !armed,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.telemetry.start_accelerometer_calibration();
                    cx.notify();
                }),
            ))
            .into_any_element(),
    };

    // Where in the sequence we are. ArduPilot does not report a step number, so this is derived
    // from the position it asked for - which is the order it always asks in.
    let mut steps = div().flex().flex_wrap().gap_1();
    let current = match state {
        AccelCalibration::Waiting(position) => Some(position),
        _ => None,
    };
    let reached = current.map_or(
        usize::from(state == AccelCalibration::Succeeded) * AccelPosition::ALL.len(),
        |position| {
            AccelPosition::ALL
                .iter()
                .position(|candidate| *candidate == position)
                .unwrap_or(0)
        },
    );
    for (index, position) in AccelPosition::ALL.iter().enumerate() {
        let done = index < reached;
        let now = current == Some(*position);
        steps = steps.child(
            div()
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .bg(rgb(theme::ACTION))
                .text_xs()
                .text_color(rgb(if now {
                    theme::WARN
                } else if done {
                    theme::OK
                } else {
                    theme::DIM
                }))
                .child(position.label()),
        );
    }

    panel(
        "accelerometer",
        div().flex().flex_col().gap_2().child(steps).child(body),
    )
}

/// Starts the sequence again from the beginning.
fn restart_button(cx: &mut Context<MissionPlanner>) -> AnyElement {
    action(
        "cal-accel-restart",
        "start again",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.telemetry.clear_accel_calibration();
            cx.notify();
        }),
    )
}

/// Compass calibration, with a progress bar per compass.
///
/// One row per compass because ArduPilot calibrates every enabled one at once and reports each
/// separately. A vehicle with an external and an internal compass can have the first pass and the
/// second fail, and saying only "failed" would send the operator to the wrong hardware.
pub fn compass_panel(
    progress: &[CompassProgress],
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    let armed = view.state.as_ref().is_some_and(|state| state.armed);
    let running = progress.iter().any(|entry| entry.status.in_progress());

    let mut rows = div().flex().flex_col().gap_2();
    for entry in progress {
        let colour = match entry.status {
            CompassStatus::Succeeded => theme::OK,
            CompassStatus::Failed | CompassStatus::BadOrientation => theme::ALERT,
            CompassStatus::Running | CompassStatus::WaitingToStart => theme::WARN,
            _ => theme::DIM,
        };
        rows = rows.child(
            div()
                .flex()
                .flex_col()
                .gap_1()
                .child(
                    div()
                        .flex()
                        .items_center()
                        .gap_3()
                        .child(
                            div()
                                .w(px(80.0))
                                .text_sm()
                                .text_color(rgb(theme::TEXT))
                                .child(format!("compass {}", entry.compass_id)),
                        )
                        .child(
                            div()
                                .w(px(48.0))
                                .text_sm()
                                .text_color(rgb(colour))
                                .child(format!("{}%", entry.percent)),
                        )
                        .child(
                            div()
                                .flex_1()
                                .min_w(px(0.0))
                                .text_xs()
                                .text_color(rgb(colour))
                                .child(entry.status.describe()),
                        ),
                )
                .child(progress_bar(f32::from(entry.percent) / 100.0, colour))
                .children(entry.fitness.map(|fitness| {
                    // ArduPilot's own threshold for a good fit is well under 100. A large value
                    // means the samples did not describe a sphere, which means the airframe was
                    // not rotated enough - a different fix from trying again the same way.
                    let good = fitness < 100.0;
                    div()
                        .text_xs()
                        .text_color(rgb(if good { theme::DIM } else { theme::WARN }))
                        .child(if good {
                            format!("fitness {fitness:.1}")
                        } else {
                            format!("fitness {fitness:.1} - poor; rotate through more orientations")
                        })
                })),
        );
    }

    if progress.is_empty() {
        rows = rows.child(div().text_xs().text_color(rgb(theme::DIM)).child(
            "start, then rotate the airframe slowly through every orientation - nose up, \
                     nose down, on each side, and upside down",
        ));
    }

    panel(
        "compass",
        div().flex().flex_col().gap_2().child(rows).child(
            div()
                .flex()
                .gap_2()
                .child(action(
                    "cal-compass-start",
                    if running {
                        "sampling"
                    } else {
                        "start compass calibration"
                    },
                    theme::WARN,
                    has_vehicle && !armed && !running,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.calibrate_compass();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "cal-compass-cancel",
                    "cancel",
                    theme::ALERT,
                    running,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.cancel_compass_calibration();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "cal-compass-clear",
                    "clear",
                    theme::TEXT,
                    !progress.is_empty() && !running,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.clear_compass_calibration();
                        cx.notify();
                    }),
                )),
        ),
    )
}

/// The calibrations that are a single command.
pub fn calibration_panel(
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    let armed = view.state.as_ref().is_some_and(|state| state.armed);

    let mut rows = div().flex().flex_col().gap_2();
    for (id, name, explanation) in SINGLE_SHOT {
        rows = rows.child(
            div()
                .flex()
                .items_center()
                .gap_3()
                .child(action(
                    id,
                    *name,
                    theme::WARN,
                    has_vehicle && !armed,
                    cx.listener(move |this, _event: &(), _window, cx| {
                        match *id {
                            "cal-level" => this.telemetry.calibrate_level(),
                            _ => this.telemetry.calibrate_ground_pressure(),
                        }
                        this.file_status = Some(format!("{name} calibration started"));
                        cx.notify();
                    }),
                ))
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(*explanation),
                ),
        );
    }

    panel(
        "calibration",
        div().flex().flex_col().gap_2().child(rows).child(
            div().text_xs().text_color(rgb(theme::DIM)).child(
                "watch the messages pane on the flight screen: the vehicle reports what \
                         it is doing and whether it worked",
            ),
        ),
    )
}
