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

//! EXPERIMENTAL's adjust aircraft baro height: `but_acbarohight_Click`. The ground pressure the
//! barometer's altitude is measured from - `GND_ABS_PRESS` where the vehicle has it, else
//! `BARO1_GND_PRESS` - is read from the vehicle (`GetParam`), "use at your own risk!!!" is said,
//! and a `NumericUpDown` from -100 to 100 shows in a window of its own (`ShowUserControl`): each
//! change of its value writes the pressure read plus 11.1 Pa a step, as a float. The window's title
//! is its control's `Text` when it opened - the number, "0".
//!
//! What the C# throws - no vehicle, the read unanswered, a write unanswered - is said on the status
//! line, by the owner's ruling. One window at a time: a press while one shows reads the pressure
//! again and starts it over, where the C# opens another.
//!
//! **Not working yet (2026-10-06):** typing a number into the box. In the headless test the box
//! never got the keyboard after a click, for a reason not found; the ▲ and ▼ buttons change the
//! number and write, and are what experimental-baro.gui tests.
//!
//! `// C#: temp.cs:1169-1187; Utilities/ExtensionsMP.cs:110-132`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{
    AnyElement, Context, InteractiveElement, IntoElement, KeyDownEvent, ParentElement,
    SharedString, StatefulInteractiveElement, Styled, Window, div, px, rgb,
};
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;
use web_time::Instant;

use crate::config::motor_test::NumericUpDown;
use crate::fly::{PLEASE_CONNECT, error_box};
use crate::telemetry::{Lookup, Report};
use crate::ui::{action, theme};
use crate::{MissionPlanner, facts};

/// "use at your own risk!!!", `CustomMessageBox.Show`'s text. `// C#: temp.cs:1175`
pub const WARNING: &str = "use at your own risk!!!";
/// The `NumericUpDown`: its value when made, and its bounds. `// C#: temp.cs:1177-1179`
const NUMBER: (f64, f64, f64) = (0.0, -100.0, 100.0);
/// Pascals a step: "338.6388 pa => 100' = 30.48m", near enough a metre. `// C#: temp.cs:1174, 1183`
const PASCALS_A_STEP: f64 = 11.1;
/// `GetParam`'s `TimeoutException`. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2342`
const READ_TIMEOUT: &str = "Timeout on read - GetParam";
/// The window: a `NumericUpDown`'s default size, 120 by 20, and its `Padding` of 20 a side.
const FORM_WIDTH: f32 = 160.0;

/// The tool under way: the number box's keyboard, made with it, and where it is.
pub struct BaroHeight {
    focus: gpui::FocusHandle,
    step: Step,
}

/// Where it is.
enum Step {
    /// `GetParam(paramname)`: the pressure asked for.
    Reading {
        param: &'static str,
        request: RequestId,
        made: Instant,
    },
    /// "use at your own risk!!!", the pressure read.
    Warning { param: &'static str, current: f64 },
    /// The window.
    Form(Form),
}

/// The window's state.
struct Form {
    /// The parameter it writes.
    param: &'static str,
    /// `double.Parse(currentQNH)`: the pressure read, as its text parses.
    current: f64,
    /// `mavlinkNumericUpDown`.
    number: NumericUpDown,
    /// `Value` when `ValueChanged` last fired.
    value: f64,
    /// How many writes `ValueChanged` has made, and the pressure the last wrote.
    writes: usize,
    last: Option<f64>,
}

impl std::fmt::Debug for BaroHeight {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.fact())
    }
}

impl BaroHeight {
    /// `experimental.baro`: where it is.
    const fn fact(&self) -> &'static str {
        match self.step {
            Step::Reading { .. } => "reading",
            Step::Warning { .. } => "warning",
            Step::Form(_) => "form",
        }
    }

    /// The window, while it shows.
    const fn form_mut(&mut self) -> Option<&mut Form> {
        match &mut self.step {
            Step::Form(form) => Some(form),
            _ => None,
        }
    }
}

/// `double.Parse(MainV2.comPort.GetParam(paramname).ToString())`: the float the vehicle sent,
/// written as `float.ToString()` writes it and read back as a double.
/// `// C#: temp.cs:1172, 1183`
#[must_use]
pub fn read_back(value: f32) -> f64 {
    mp_log::netfmt::single(value)
        .parse()
        .unwrap_or_else(|_| f64::from(value))
}

/// `(float)(double.Parse(currentQNH) + (double)Value * 11.1)`: what a step writes.
/// `// C#: temp.cs:1183`
#[must_use]
#[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
#[allow(clippy::suboptimal_flops)] // the C# multiplies, then adds: two roundings, not one
pub fn pressure(current: f64, value: f64) -> f64 {
    f64::from((current + value * PASCALS_A_STEP) as f32)
}

/// The button: the parameter chosen as QNH chooses it, and read from the vehicle.
/// `// C#: temp.cs:1171-1172`
pub fn open(this: &mut MissionPlanner, cx: &mut Context<MissionPlanner>) {
    let view = this.telemetry.view();
    if view.vehicle.is_none() {
        this.file_status = Some(error_box(PLEASE_CONNECT));
        return;
    }
    let param = crate::experimental::qnh_param(&view.parameters);
    this.experimental.baro = match this.telemetry.read_parameter(param) {
        Some(request) => Some(BaroHeight {
            focus: cx.focus_handle(),
            step: Step::Reading {
                param,
                request,
                made: Instant::now(),
            },
        }),
        None => {
            this.file_status = Some(error_box(PLEASE_CONNECT));
            None
        }
    };
}

/// Once a frame: the read's answer, when it comes.
pub fn tick(this: &mut MissionPlanner) {
    let Some(BaroHeight {
        step: Step::Reading {
            param,
            request,
            made,
        },
        ..
    }) = this.experimental.baro
    else {
        return;
    };
    let outcome = match this.telemetry.lookup(request, made) {
        Lookup::Found(request) => request.outcome(),
        Lookup::PickingUp => None,
        Lookup::Gone => Some(RequestOutcome::TimedOut),
    };
    let Some(outcome) = outcome else {
        crate::repaint::in_flight();
        return;
    };
    match outcome {
        RequestOutcome::Accepted { value: Some(value) } => {
            if let Some(baro) = this.experimental.baro.as_mut() {
                baro.step = Step::Warning {
                    param,
                    #[allow(clippy::cast_possible_truncation)] // `GetParam`'s float, as sent
                    current: read_back(value.as_f64() as f32),
                };
            }
        }
        _ => {
            this.file_status = Some(error_box(READ_TIMEOUT));
            this.experimental.baro = None;
        }
    }
}

/// The warning's OK: the window shown.
fn warned(this: &mut MissionPlanner) {
    if let Some(baro) = this.experimental.baro.as_mut()
        && let Step::Warning { param, current } = baro.step
    {
        baro.step = Step::Form(Form {
            param,
            current,
            number: NumericUpDown::new(NUMBER),
            value: NUMBER.0,
            writes: 0,
            last: None,
        });
    }
}

/// `ValueChanged`: the box's value, once validated, if it has changed - the pressure read plus
/// 11.1 Pa a step, written to the vehicle being flown.
/// `// C#: temp.cs:1181-1184`
fn value_changed(this: &mut MissionPlanner) {
    let Some(form) = this
        .experimental
        .baro
        .as_mut()
        .and_then(BaroHeight::form_mut)
    else {
        return;
    };
    let value = form.number.held();
    #[allow(clippy::float_cmp)] // a `decimal`'s whole numbers, compared as `ValueChanged` does
    if value == form.value {
        return;
    }
    form.value = value;
    let (param, target) = (form.param, pressure(form.current, value));
    form.writes += 1;
    form.last = Some(target);
    let vehicle = this.telemetry.send_handle().map(|(_, vehicle)| vehicle);
    let sent = vehicle.and_then(|vehicle| {
        this.telemetry.set_parameter_on(
            vehicle,
            param,
            target,
            false,
            Report::on_failure(error_box(format!("Timeout on read - setParam {param}"))),
        )
    });
    if sent.is_none() {
        this.file_status = Some(error_box(PLEASE_CONNECT));
    }
}

/// `UpButton` and `DownButton`, by mouse or arrow key: the text typed validated - `ValueChanged`
/// if that changed it - then the value stepped, `ValueChanged` again.
fn arrow(this: &mut MissionPlanner, delta: f64) {
    if let Some(form) = this
        .experimental
        .baro
        .as_mut()
        .and_then(BaroHeight::form_mut)
    {
        form.number.commit();
    }
    value_changed(this);
    if let Some(form) = this
        .experimental
        .baro
        .as_mut()
        .and_then(BaroHeight::form_mut)
    {
        form.number.step(delta);
    }
    value_changed(this);
}

/// The window over the tab: the warning, or the number with its arrows.
pub fn window(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let baro = this.experimental.baro.as_ref()?;
    match &baro.step {
        Step::Reading { .. } => None,
        Step::Warning { .. } => Some(crate::config::firmware::message_box(
            crate::experimental::IDS,
            &crate::config::firmware::Waiting {
                text: WARNING.to_owned(),
                caption: String::new(),
                buttons: None,
            },
            window,
            warned,
            cx,
        )),
        Step::Form(form) => Some(form_window(form, &baro.focus, window, cx)),
    }
}

/// The number box: typed into while it has the keyboard, its arrows at its right.
fn form_window(
    form: &Form,
    focus: &gpui::FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = focus.is_focused(window);
    let text = crate::probe::measured("experimental-baro-value", div())
        .id("experimental-baro-value")
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_text()
        .track_focus(focus)
        .key_context("TextField")
        .child(form.number.field.value().to_owned())
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))))
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            match event.keystroke.key.as_str() {
                "up" => arrow(this, 1.0),
                "down" => arrow(this, -1.0),
                _ => {
                    let Some(form) = this
                        .experimental
                        .baro
                        .as_mut()
                        .and_then(BaroHeight::form_mut)
                    else {
                        return;
                    };
                    if !form.number.key(event) {
                        return;
                    }
                    value_changed(this);
                }
            }
            cx.notify();
        }));
    let arrow_button = |suffix: &'static str, glyph: &'static str, delta: f64| {
        let id = format!("experimental-baro-{suffix}");
        crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(glyph)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                arrow(this, delta);
                cx.notify();
            }))
    };
    let number = div()
        .h(px(20.0))
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::ACTION))
        .child(text)
        .child(
            div()
                .w(px(14.0))
                .h_full()
                .flex()
                .flex_col()
                .border_l_1()
                .border_color(rgb(theme::BORDER))
                .child(arrow_button("up", "▲", 1.0))
                .child(arrow_button("down", "▼", -1.0)),
        );
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        // `frm.Text = ctl.Text`: the number box's text when the window opened.
        .child(div().text_xs().text_color(rgb(theme::DIM)).child("0"))
        .child(action(
            "experimental-baro-close",
            "\u{2715}",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.experimental.baro = None;
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("experimental-baro", div())
        .id("experimental-baro")
        .w(px(FORM_WIDTH))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .occlude()
        .child(caption)
        .child(div().p(px(20.0)).child(number));
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(
                ((size.width - px(FORM_WIDTH)) / 2.0).max(px(0.0)),
                px(120.0),
            ))
            .child(body),
    )
    .with_priority(1)
    .into_any_element()
}

/// `experimental.baro.*`: where it is, the parameter, the pressure read and the box's value.
pub fn record_facts(state: Option<&BaroHeight>) {
    facts::record("experimental.baro", state.map_or("none", BaroHeight::fact));
    let form = match state.map(|baro| &baro.step) {
        Some(Step::Form(form)) => Some(form),
        _ => None,
    };
    facts::record(
        "experimental.baro.param",
        form.map_or("none", |form| form.param),
    );
    facts::record(
        "experimental.baro.read",
        form.map_or_else(|| "none".to_owned(), |form| form.current.to_string()),
    );
    facts::record(
        "experimental.baro.value",
        form.map_or_else(
            || "none".to_owned(),
            |form| form.number.field.value().to_owned(),
        ),
    );
    facts::record(
        "experimental.baro.writes",
        form.map_or(0, |form| form.writes),
    );
    // The pressure last written, as the float goes on the wire.
    #[allow(clippy::cast_possible_truncation)] // `pressure` is a float's value already
    facts::record(
        "experimental.baro.last",
        form.and_then(|form| form.last).map_or_else(
            || "none".to_owned(),
            |last| mp_log::netfmt::single(last as f32),
        ),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `GetParam`'s float read back through its text: 101325.4 sent as a float is
    /// 101325.3984375, written "101325.4" and parsed as the double 101325.4.
    #[test]
    fn the_pressure_read_is_the_floats_text() {
        assert_eq!(read_back(101_325.4), 101_325.4);
        assert_eq!(read_back(0.0), 0.0);
        assert_eq!(read_back(98_765.0), 98_765.0);
    }

    /// A step is 11.1 Pa, the sum made a float: 101325.4 + 3 * 11.1 = 101358.7, as a float
    /// 101358.703125; and the bounds' -1110 and +1110.
    #[test]
    fn a_step_is_eleven_pascals_as_a_float() {
        assert_eq!(pressure(101_325.4, 3.0), 101_358.703_125);
        assert_eq!(pressure(101_325.4, 0.0), f64::from(101_325.4_f32));
        assert_eq!(pressure(100_000.0, -100.0), 98_890.0);
        assert_eq!(pressure(100_000.0, 100.0), 101_110.0);
    }
}
