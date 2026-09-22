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

use gpui::{div, prelude::*, px, rgb};
use mp_mavlink_dialects::all::{MavAutopilot, MavType};

use crate::telemetry::TelemetryView;
use crate::ui::{field, panel, theme};

/// What each calibration needs before it can be offered, so the gap is visible rather than
/// implied. Each entry becomes a deliverable in its own right.
const PLANNED: &[(&str, &str)] = &[
    (
        "accelerometer",
        "six-position sequence driven by COMMAND_ACK progress",
    ),
    (
        "compass",
        "onboard sampling, progress over MAG_CAL_PROGRESS and MAG_CAL_REPORT",
    ),
    ("radio", "RC_CHANNELS min/max capture, then trim"),
    ("level", "single-shot PREFLIGHT_CALIBRATION with param5"),
    (
        "escs and motors",
        "MAV_CMD_DO_MOTOR_TEST, one motor at a time",
    ),
    (
        "frame and airframe",
        "FRAME_CLASS and FRAME_TYPE parameters",
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
            .gap_3()
            .child(
                div()
                    .flex()
                    .gap_4()
                    .child(field("type", vehicle, theme::TEXT))
                    .child(field("autopilot", autopilot, theme::TEXT)),
            )
            .child(field("mavlink identity", identity, theme::TEXT)),
    )
}

/// The calibrations, and what each one still needs.
pub fn calibration_panel() -> impl IntoElement {
    let mut rows = div().flex().flex_col().gap_2();
    for (name, needs) in PLANNED {
        rows = rows.child(
            div()
                .flex()
                .gap_3()
                .items_center()
                .child(
                    div()
                        .flex_shrink_0()
                        .w(px(130.0))
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(*name),
                )
                .child(
                    div()
                        .flex_shrink_0()
                        .px_2()
                        .py(px(1.0))
                        .rounded_sm()
                        .bg(rgb(theme::ACTION))
                        .text_xs()
                        .text_color(rgb(theme::WARN))
                        .child("not built"),
                )
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(*needs),
                ),
        );
    }

    panel(
        "calibration",
        div().flex().flex_col().gap_3().child(rows).child(
            div().text_xs().text_color(rgb(theme::DIM)).child(
                "flying and planning come first; each calibration lands with its own \
                         protocol handling and tests",
            ),
        ),
    )
}
