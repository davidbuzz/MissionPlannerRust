//! The flight screen: what the aircraft is doing, and the controls that change it.
//!
//! Equivalent to Mission Planner's Flight Data tab. The ordering on screen follows what a pilot
//! looks at, not what is easiest to lay out: attitude and mode first, then position and health,
//! then the message log, then the controls that commit something to the aircraft.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_link::messages::{Severity, time_of_day};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, field, panel, theme};

/// Default height for the takeoff button, in metres above home.
///
/// Mission Planner asks for this in a dialog every time. A fixed, visible default is better for a
/// first pass than a text field nobody reads, and it is deliberately low: a wrong 10 is a hover,
/// a wrong 100 is an incident.
pub const TAKEOFF_ALTITUDE: f32 = 10.0;

/// What the aircraft is and whether it is armed.
pub fn vehicle_panel(view: &TelemetryView) -> impl IntoElement {
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
        |s| format!("{:.1} m", s.altitude_relative.0),
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
            .gap_3()
            .child(field("state", armed, armed_colour))
            .child(field("position", position, theme::TEXT))
            .child(
                div()
                    .flex()
                    .gap_4()
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
pub fn actions_panel(view: &TelemetryView, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    let armed = view.state.as_ref().is_some_and(|s| s.armed);
    let has_vehicle = view.vehicle.is_some();

    let controls = div()
        .flex()
        .flex_wrap()
        .gap_2()
        .child(action(
            "arm",
            "arm",
            theme::WARN,
            has_vehicle && !armed,
            cx.listener(|this, _event: &(), _window, cx| {
                this.telemetry.arm(true);
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
        ));

    panel(
        "actions",
        div()
            .flex()
            .flex_col()
            .gap_3()
            .child(controls)
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

pub fn hud_panel(view: &TelemetryView) -> impl IntoElement {
    let state = view.state.clone();
    let (speed, altitude, heading, mode) = view.state.as_ref().map_or_else(
        || {
            (
                "--".to_owned(),
                "--".to_owned(),
                "--".to_owned(),
                "no vehicle".to_owned(),
            )
        },
        |s| {
            (
                format!("{:.0}", s.ground_speed.0),
                format!("{:.0}", s.altitude_relative.0),
                format!("{:03.0}", s.heading.degrees()),
                {
                    // The flight mode matters more than the armed flag on a HUD, so show
                    // both: the mode by name, with the armed state as a suffix and as the
                    // colour. An unknown mode shows its number rather than nothing.
                    let mode = mp_vehicle::flight_mode_name(s.vehicle_type, s.custom_mode)
                        .map_or_else(|| format!("mode {}", s.custom_mode), ToOwned::to_owned);
                    if s.armed {
                        format!("{mode}  ARMED")
                    } else {
                        mode
                    }
                },
            )
        },
    );
    let mode_colour = if view.state.as_ref().is_some_and(|s| s.armed) {
        theme::ALERT
    } else {
        theme::DIM
    };

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
                move |bounds, (), window, _cx| {
                    crate::hud::paint_hud(state.as_deref(), bounds, window);
                },
            )
            .size_full(),
        )
        // The readouts sit on backing strips rather than directly on the artificial horizon.
        // Without them the pitch ladder runs straight through the text at exactly the attitude
        // where a pilot most wants to read it.
        .child(
            div()
                .absolute()
                .top_2()
                .left_2()
                .flex()
                .flex_col()
                .px_2()
                .rounded_md()
                .bg(rgb(theme::PANEL))
                .child(div().text_xs().text_color(rgb(theme::DIM)).child("m/s"))
                .child(div().text_lg().text_color(rgb(theme::TEXT)).child(speed)),
        )
        .child(
            div()
                .absolute()
                .top_2()
                .right_2()
                .flex()
                .flex_col()
                .items_end()
                .px_2()
                .rounded_md()
                .bg(rgb(theme::PANEL))
                .child(div().text_xs().text_color(rgb(theme::DIM)).child("m"))
                .child(div().text_lg().text_color(rgb(theme::TEXT)).child(altitude)),
        )
        .child(
            div()
                .absolute()
                .bottom_2()
                .left_0()
                .w_full()
                .flex()
                .justify_center()
                .child(
                    div()
                        .flex()
                        .gap_4()
                        .px_3()
                        .py(px(2.0))
                        .rounded_md()
                        .bg(rgb(theme::PANEL))
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .child(div().text_sm().text_color(rgb(theme::TEXT)).child(heading))
                        .child(div().text_sm().text_color(rgb(mode_colour)).child(mode)),
                ),
        )
}

/// Satellite fix quality and battery state: the two numbers that decide whether to fly.
pub fn gps_panel(view: &TelemetryView) -> impl IntoElement {
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
    let battery = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |s| {
            format!(
                "{:.2} V  {}%",
                s.battery.voltage, s.battery.remaining_percent
            )
        },
    );

    panel(
        "gps and power",
        div()
            .flex()
            .flex_col()
            .gap_3()
            .child(
                div()
                    .flex()
                    .gap_4()
                    .child(field("fix", fix_text, fix_colour))
                    .child(field("satellites", sats, theme::TEXT)),
            )
            .child(field("battery", battery, theme::TEXT)),
    )
}

/// Telemetry link health.
pub fn link_panel(view: &TelemetryView) -> impl IntoElement {
    let loss = view.state.as_ref().map_or_else(
        || "--".to_owned(),
        |s| format!("{:.2} %", s.link.loss_percent()),
    );
    let loss_colour = view.state.as_ref().map_or(theme::DIM, |s| {
        if s.link.loss_percent() > 5.0 {
            theme::ALERT
        } else {
            theme::TEXT
        }
    });
    let crc_colour = if view.crc_errors > 0 {
        theme::WARN
    } else {
        theme::TEXT
    };

    panel(
        "link",
        div()
            .flex()
            .flex_col()
            .gap_3()
            .child(
                div()
                    .flex()
                    .gap_4()
                    .child(field("frames", view.frames.to_string(), theme::TEXT))
                    .child(field("loss", loss, loss_colour))
                    .child(field("crc errors", view.crc_errors.to_string(), crc_colour)),
            )
            .child(field(
                "systems on link",
                view.vehicle_count.to_string(),
                theme::TEXT,
            )),
    )
}
