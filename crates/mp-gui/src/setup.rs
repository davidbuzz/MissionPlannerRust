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
use mp_calibration::radio::RcRange;
use mp_calibration::{AccelCalibration, AccelPosition, CompassProgress, CompassStatus};
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

/// Initial Setup's Mandatory Hardware pages that open here as pages, in the order
/// `InitialSetup.cs` adds them: Flight Modes, after ESC Calibration and before FailSafe.
///
/// Each is offered only with a vehicle whose parameters have all arrived, as the C# adds a page
/// only when `isConnected && gotAllParams`; a press opens the page beneath this and another
/// closes it, as `Activate` and `Deactivate` bracket a backstage page being shown.
/// `// C#: GCSViews/InitialSetup.cs:119-131, 182, 226-229; GCSViews/InitialSetup.resx (backstageViewPagemand.Text, backstageViewPageflmode.Text)`
pub fn mandatory_hardware_panel(
    view: &TelemetryView,
    flight_modes_open: bool,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let got_all_params = view.parameters.len() >= usize::from(view.parameters_expected);
    let available = view.vehicle.is_some() && got_all_params;
    panel(
        "mandatory hardware",
        div().flex().flex_wrap().gap_2().child(action(
            "setup-flightmodes",
            "Flight Modes",
            if flight_modes_open {
                theme::OK
            } else {
                theme::ACCENT
            },
            available || flight_modes_open,
            cx.listener(|this, _event: &(), _window, cx| {
                this.flight_modes.toggle(&this.telemetry);
                cx.notify();
            }),
        )),
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

/// The pulse widths a bar is drawn between.
///
/// Not the observed range: a bar scaled to what has been seen so far would move its own endpoints
/// as the stick is swept, which makes it impossible to tell travel from rescaling. These are the
/// limits an RC receiver works within, so a stick at its stop sits at the end of the bar.
const PULSE_MINIMUM: f32 = 900.0;
/// The upper end of that scale.
const PULSE_MAXIMUM: f32 = 2100.0;

/// Radio calibration: live channel values, and the limits recorded while the sticks are swept.
///
/// All sixteen channels ArduPilot maps to functions, not the eight of the older message. A radio
/// with switches on channels nine and up is ordinary, and a calibration screen that stopped at
/// eight would leave them uncalibrated with nothing saying so.
pub fn radio_panel(
    view: &TelemetryView,
    range: &RcRange,
    capturing: bool,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let rc = view
        .state
        .as_ref()
        .map(|state| state.rc)
        .unwrap_or_default();
    let has_vehicle = view.vehicle.is_some();
    let armed = view.state.as_ref().is_some_and(|state| state.armed);

    let mut rows = div().flex().flex_col().gap_1();
    for number in 1..=mp_vehicle::rc::CHANNELS {
        let Some(value) = rc.channel(number) else {
            continue;
        };
        let fraction =
            ((f32::from(value) - PULSE_MINIMUM) / (PULSE_MAXIMUM - PULSE_MINIMUM)).clamp(0.0, 1.0);
        let recorded = range.channel(number);
        rows = rows.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(28.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(number.to_string()),
                )
                .child(
                    div()
                        .w(px(52.0))
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(value.to_string()),
                )
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .child(progress_bar(fraction, theme::ACCENT)),
                )
                .child(
                    div()
                        .w(px(110.0))
                        .text_xs()
                        .text_color(rgb(if recorded.is_some() {
                            theme::OK
                        } else {
                            theme::DIM
                        }))
                        .child(recorded.map_or_else(
                            || "not swept".to_owned(),
                            |(low, high)| format!("{low} - {high}"),
                        )),
                ),
        );
    }

    if rc.live() == 0 {
        rows = rows.child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(if rc.reported {
                    "the vehicle reports no live channels - is the receiver bound and powered?"
                } else {
                    "no channel data yet"
                }),
        );
    }

    let usable = range.usable();

    panel(
        "radio",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .children(rc.rssi_percent().map(|rssi| {
                div()
                    .text_xs()
                    .text_color(rgb(if rssi < 30 { theme::WARN } else { theme::DIM }))
                    .child(format!("signal {rssi}%"))
            }))
            .child(rows)
            .child(
                div()
                    .flex()
                    .flex_wrap()
                    .gap_2()
                    .child(action(
                        "radio-capture",
                        if capturing {
                            "recording"
                        } else {
                            "record limits"
                        },
                        theme::WARN,
                        has_vehicle && !armed && !capturing,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.begin_radio_capture();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "radio-stop",
                        "stop",
                        theme::ACCENT,
                        capturing,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.capturing_radio = false;
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "radio-save",
                        format!("save {usable} channels"),
                        theme::ALERT,
                        has_vehicle && !armed && !capturing && usable > 0,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.save_radio_limits();
                            cx.notify();
                        }),
                    )),
            )
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(if capturing {
                        "move every stick and switch to both of its limits, then stop"
                    } else {
                        "record, sweep every control to its limits, stop, then save"
                    }),
            ),
    )
}

/// How many motors to offer.
///
/// Eight covers every common multirotor up to an octocopter. A frame with more is a frame whose
/// operator is not learning their motor order from a ground station for the first time.
const MOTORS: u8 = 8;

/// Motor test: spins one motor at a time, briefly.
///
/// Disabled while armed, and deliberately not offered as a sequence. Testing motors in order is
/// how an operator loses track of which one is about to move, and this is the only control in the
/// application that turns something sharp.
pub fn motor_panel(
    view: &TelemetryView,
    throttle: f32,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    let armed = view.state.as_ref().is_some_and(|state| state.armed);
    let enabled = has_vehicle && !armed;

    let mut buttons = div().flex().flex_wrap().gap_2();
    for motor in 1..=MOTORS {
        buttons = buttons.child(
            crate::probe::measured(format!("motor-{motor}"), div())
                .id(("motor", usize::from(motor)))
                .px_3()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(if enabled { theme::ALERT } else { theme::BORDER }))
                .bg(rgb(theme::ACTION))
                .text_sm()
                .text_color(rgb(if enabled { theme::ALERT } else { theme::DIM }))
                .cursor_pointer()
                .child(format!("{motor}"))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    if this.telemetry.view().state.is_some_and(|state| state.armed) {
                        return;
                    }
                    this.telemetry.test_motor(motor, this.motor_throttle);
                    this.file_status = Some(format!(
                        "motor {motor} at {:.0}% for {:.0}s",
                        this.motor_throttle,
                        mp_calibration::MOTOR_TEST_SECONDS
                    ));
                    cx.notify();
                })),
        );
    }

    panel(
        "motors",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::ALERT))
                    .child("REMOVE THE PROPELLERS BEFORE USING THIS"),
            )
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(
                        div()
                            .w(px(64.0))
                            .text_xs()
                            .text_color(rgb(theme::DIM))
                            .child("throttle"),
                    )
                    .child(
                        div()
                            .w(px(52.0))
                            .text_sm()
                            .text_color(rgb(theme::TEXT))
                            .child(format!("{throttle:.0}%")),
                    )
                    .child(throttle_step("motor-throttle-down", -1.0, "-1", cx))
                    .child(throttle_step("motor-throttle-up", 1.0, "+1", cx)),
            )
            .child(buttons)
            .child(action(
                "motor-stop",
                "stop all",
                theme::OK,
                enabled,
                cx.listener(|this, _event: &(), _window, cx| {
                    for motor in 1..=MOTORS {
                        this.telemetry.stop_motor(motor);
                    }
                    this.file_status = Some("motor test stopped".to_owned());
                    cx.notify();
                }),
            ))
            .child(div().text_xs().text_color(rgb(theme::DIM)).child(format!(
                "each press spins one motor for {:.0} seconds, then the vehicle stops it \
                         on its own",
                mp_calibration::MOTOR_TEST_SECONDS
            ))),
    )
}

/// One step of the motor test throttle.
fn throttle_step(
    id: &'static str,
    delta: f32,
    label: &'static str,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    crate::probe::measured(id, div())
        .id(id)
        .px_2()
        .py_1()
        .rounded_md()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(label)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            this.motor_throttle =
                (this.motor_throttle + delta).clamp(1.0, mp_calibration::MAX_MOTOR_TEST_THROTTLE);
            cx.notify();
        }))
}

/// The vehicle's dataflash logs, and downloading one.
pub fn logs_panel(
    listings: &[mp_ftp::logs::LogListing],
    progress: Option<(u16, u32, u32)>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    let downloading = progress.is_some();

    let mut rows = div().flex().flex_col().gap_1();
    for listing in listings {
        let id = listing.id;
        let size = listing.size;
        let megabytes = f64::from(size) / (1024.0 * 1024.0);
        let active = progress.is_some_and(|(running, _, _)| running == id);
        rows = rows.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(40.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(id.to_string()),
                )
                .child(
                    div()
                        .w(px(80.0))
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(format!("{megabytes:.1} MiB")),
                )
                .child(
                    div()
                        .w(px(96.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        // A flight controller with no GPS fix and no clock reports zero. Showing
                        // a date in 1970 for every log is less useful than saying we do not know.
                        .child(if listing.has_timestamp() {
                            format!("utc {}", listing.time_utc)
                        } else {
                            "no clock".to_owned()
                        }),
                )
                .child(action_for_log(
                    id,
                    size,
                    has_vehicle && !downloading,
                    active,
                    cx,
                )),
        );
    }

    if listings.is_empty() {
        rows = rows.child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("no logs listed yet"),
        );
    }

    panel(
        "logs",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(action(
                "logs-list",
                "list logs",
                theme::ACCENT,
                has_vehicle && !downloading,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.telemetry.request_log_list();
                    cx.notify();
                }),
            ))
            .child(rows)
            .children(progress.map(|(id, filled, size)| {
                #[allow(clippy::cast_precision_loss)] // log sizes are megabytes
                let fraction = if size > 0 {
                    (filled as f32 / size as f32).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                div()
                    .flex()
                    .flex_col()
                    .gap_1()
                    .child(
                        div()
                            .text_xs()
                            .text_color(rgb(theme::ACCENT))
                            .child(format!(
                                "log {id}: {:.1} of {:.1} MiB",
                                f64::from(filled) / (1024.0 * 1024.0),
                                f64::from(size) / (1024.0 * 1024.0)
                            )),
                    )
                    .child(progress_bar(fraction, theme::ACCENT))
            })),
    )
}

/// The download button for one log.
fn action_for_log(
    id: u16,
    size: u32,
    enabled: bool,
    active: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    crate::probe::measured(format!("log-{id}"), div())
        .id(gpui::SharedString::from(format!("log-{id}")))
        .px_2()
        .py(px(1.0))
        .rounded_sm()
        .border_1()
        .border_color(rgb(if active { theme::ACCENT } else { theme::BORDER }))
        .text_xs()
        .text_color(rgb(if enabled || active {
            theme::TEXT
        } else {
            theme::DIM
        }))
        .cursor_pointer()
        .child(if active { "downloading" } else { "download" })
        .on_click(cx.listener(move |this, _event, _window, cx| {
            if !enabled {
                return;
            }
            this.telemetry.download_log(id, size);
            this.file_status = Some(format!("downloading log {id}"));
            cx.notify();
        }))
        .into_any_element()
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
