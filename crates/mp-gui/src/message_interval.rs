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

//! EXPERIMENTAL's Message Interval: `but_messageinterval_Click`'s form, shown beside the tab as
//! the C# shows it (`form.Show(this)`, nothing behind it held). A `FlowLayoutPanel` of four
//! controls: `cmb`, every `MAVLINK_MSG_ID` by name, sorted, 50 wider than a combo; `cmbrate`,
//! the rates 0 to 199, which can be typed; Set, which asks the vehicle to send the message chosen
//! at that rate - `SET_MESSAGE_INTERVAL` with the message's id and the interval in microseconds,
//! waited for; and Set All, the same for every message in the list, none waited for. A rate of 0
//! or less goes as it is - the vehicle's own rate, or none.
//!
//! What the C# shows in a box when a command fails, or when the rate is no number, is said on the
//! status line here, by the owner's ruling.
//!
//! `// C#: temp.cs:1026-1098`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::OnceLock;

use gpui::{
    AnyElement, Context, InteractiveElement, IntoElement, ParentElement, SharedString,
    StatefulInteractiveElement, Styled, Window, div, px, rgb,
};

use crate::config::firmware::DO_COMMAND_TIMEOUT;
use crate::fly::{PLEASE_CONNECT, error_box};
use crate::telemetry::Report;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, theme};
use crate::{MissionPlanner, facts};

/// `MAV_CMD.SET_MESSAGE_INTERVAL`.
const SET_MESSAGE_INTERVAL: u16 = 511;
/// `cmbrate.DataSource`: `Enumerable.Range(0, 200)`.
const RATES: u32 = 200;
/// `cmb.Width += 50`, on a combo's 121.
const MESSAGE_WIDTH: f32 = 171.0;
/// A combo's own width.
const RATE_WIDTH: f32 = 121.0;
/// `new Form()`'s 300 wide.
const FORM_WIDTH: f32 = 300.0;

/// `Enum.GetNames(typeof(MAVLink.MAVLINK_MSG_ID)).ToSortedList(...)`: every message, by name, with
/// its id - the dialect's table, which holds the same 353 as the C#'s enum. Sorted ordinally, where
/// the C#'s `string.CompareTo` sorts by the culture's rules; for these names of capitals, digits
/// and underscores the two orders differ only where an underscore meets a letter or digit.
pub fn messages() -> &'static [(&'static str, u32)] {
    static SORTED: OnceLock<Vec<(&'static str, u32)>> = OnceLock::new();
    SORTED.get_or_init(|| {
        let mut all: Vec<(&'static str, u32)> = mp_mavlink_dialects::all::MESSAGES
            .iter()
            .map(|message| (message.name, message.id))
            .collect();
        all.sort_unstable_by(|a, b| a.0.cmp(b.0));
        all
    })
}

/// The interval `SET_MESSAGE_INTERVAL` is given for a rate in hertz: as it is at 0 or less, else
/// a millionth of a second over the rate, in `float` as the C# works it.
/// `// C#: temp.cs:1042-1048`
#[must_use]
pub fn interval(rate: f64) -> f32 {
    #[allow(clippy::cast_possible_truncation)] // `(float) rate`
    let rate = rate as f32;
    if rate <= 0.0 {
        rate
    } else {
        1.0 / rate * 1_000_000.0
    }
}

/// `double.Parse(cmbrate.Text)`: what the rate box holds, or the exception's words.
fn parse_rate(text: &str) -> Result<f64, String> {
    text.trim()
        .parse::<f64>()
        .map_err(|_| format!("Input string was not in a correct format. ({text})"))
}

/// The form, while it shows.
pub struct IntervalForm {
    /// `cmb`'s choice, an index into [`messages`].
    chosen: usize,
    /// Whether `cmb`'s list is down.
    messages_open: bool,
    /// `cmbrate`'s text.
    rate: TextField,
    /// Whether `cmbrate`'s list is down.
    rates_open: bool,
    /// `cmbrate`'s keyboard.
    focus: gpui::FocusHandle,
}

impl std::fmt::Debug for IntervalForm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("IntervalForm")
            .field("chosen", &self.chosen)
            .field("rate", &self.rate.value())
            .finish_non_exhaustive()
    }
}

impl IntervalForm {
    /// As the C# makes it: the first message, and rate 0, each its list's first.
    #[must_use]
    pub fn new(focus: gpui::FocusHandle) -> Self {
        let mut rate = TextField::new("");
        rate.set("0");
        Self {
            chosen: 0,
            messages_open: false,
            rate,
            rates_open: false,
            focus,
        }
    }

    /// The message chosen, and its id.
    #[must_use]
    pub fn message(&self) -> (&'static str, u32) {
        messages().get(self.chosen).copied().unwrap_or(("", 0))
    }
}

/// `but_messageinterval_Click`: the form shown, or brought back as it was.
pub fn open(this: &mut MissionPlanner, cx: &mut Context<MissionPlanner>) {
    if this.experimental.interval.is_none() {
        this.experimental.interval = Some(IntervalForm::new(cx.focus_handle()));
    }
}

/// Set: the message chosen, at the rate in the box, waited for.
/// `// C#: temp.cs:1040-1060`
fn set(this: &mut MissionPlanner) {
    let Some(form) = this.experimental.interval.as_ref() else {
        return;
    };
    let rate = match parse_rate(form.rate.value()) {
        Ok(rate) => rate,
        Err(why) => {
            this.file_status = Some(error_box(why));
            return;
        }
    };
    let (_, id) = form.message();
    #[allow(clippy::cast_precision_loss)] // `(float) (int) value`: ids are below 2^24
    let params = [id as f32, interval(rate), 0.0, 0.0, 0.0, 0.0, 0.0];
    let sent = this.telemetry.send_handle().is_some_and(|(_, vehicle)| {
        this.telemetry
            .command(
                vehicle,
                SET_MESSAGE_INTERVAL,
                params,
                Report::on_timeout(error_box(DO_COMMAND_TIMEOUT)),
            )
            .is_some()
    });
    if !sent {
        this.file_status = Some(error_box(PLEASE_CONNECT));
    }
}

/// Set All: every message in the list, at the rate in the box, none waited for.
/// `// C#: temp.cs:1063-1084`
fn set_all(this: &mut MissionPlanner) {
    let Some(form) = this.experimental.interval.as_ref() else {
        return;
    };
    let rate = match parse_rate(form.rate.value()) {
        Ok(rate) => rate,
        Err(why) => {
            this.file_status = Some(error_box(why));
            return;
        }
    };
    let ratio = interval(rate);
    for &(_, id) in messages() {
        #[allow(clippy::cast_precision_loss)]
        let params = [id as f32, ratio, 0.0, 0.0, 0.0, 0.0, 0.0];
        if !this
            .telemetry
            .command_unacknowledged(SET_MESSAGE_INTERVAL, params)
        {
            this.file_status = Some(error_box(PLEASE_CONNECT));
            return;
        }
    }
}

/// A combo's box: its text, and a click that drops its list.
fn combo(
    id: &'static str,
    text: impl Into<SharedString>,
    width: f32,
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Stateful<gpui::Div> {
    crate::probe::measured(id, div())
        .id(id)
        .relative()
        .w(px(width))
        .px_2()
        .py(px(1.0))
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .flex()
        .justify_between()
        .child(
            div()
                .min_w(px(0.0))
                .overflow_hidden()
                .text_ellipsis()
                .child(text.into()),
        )
        .child("\u{25be}")
        .on_click(cx.listener(move |this, _event, _window, cx| {
            on_click(this);
            cx.notify();
        }))
}

/// A row of an open list.
fn row(
    id: String,
    text: String,
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .px_2()
        .py(px(2.0))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(text)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            on_click(this);
            cx.notify();
        }))
        .into_any_element()
}

/// The form, beside the tab: its four controls flowing left to right as `FlowLayoutPanel` lays
/// them out in a 300-wide form, and its close box.
pub fn window(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = this.experimental.interval.as_ref()?;
    let (name, _) = form.message();
    let mut messages_box = combo(
        "experimental-interval-message",
        name,
        MESSAGE_WIDTH,
        |this| {
            if let Some(form) = this.experimental.interval.as_mut() {
                form.messages_open = !form.messages_open;
                form.rates_open = false;
            }
        },
        cx,
    );
    if form.messages_open {
        let rows = messages()
            .iter()
            .enumerate()
            .map(|(index, &(name, _))| {
                row(
                    format!("experimental-interval-message-{name}"),
                    name.to_owned(),
                    move |this| {
                        if let Some(form) = this.experimental.interval.as_mut() {
                            form.chosen = index;
                            form.messages_open = false;
                        }
                    },
                    cx,
                )
            })
            .collect();
        messages_box =
            messages_box.child(crate::dropdown("experimental-interval-message-list", rows));
    }
    let focused = form.focus.is_focused(window);
    let mut rate_box = div()
        .relative()
        .w(px(RATE_WIDTH))
        .flex()
        .items_center()
        .child(crate::textfield::text_field(
            "experimental-interval-rate",
            &form.rate,
            &form.focus,
            focused,
            px(RATE_WIDTH - 22.0),
            cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                let Some(form) = this.experimental.interval.as_mut() else {
                    return;
                };
                match form.rate.key(event) {
                    KeyOutcome::Changed => cx.notify(),
                    KeyOutcome::Submitted => {
                        set(this);
                        cx.notify();
                    }
                    KeyOutcome::Cancelled | KeyOutcome::Ignored => {}
                }
            }),
        ))
        .child(
            crate::probe::measured("experimental-interval-rates", div())
                .id("experimental-interval-rates")
                .px_1()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .child("\u{25be}")
                .on_click(cx.listener(|this, _event, _window, cx| {
                    if let Some(form) = this.experimental.interval.as_mut() {
                        form.rates_open = !form.rates_open;
                        form.messages_open = false;
                    }
                    cx.notify();
                })),
        );
    if form.rates_open {
        let rows = (0..RATES)
            .map(|rate| {
                row(
                    format!("experimental-interval-rate-{rate}"),
                    rate.to_string(),
                    move |this| {
                        if let Some(form) = this.experimental.interval.as_mut() {
                            form.rate.set(rate.to_string());
                            form.rates_open = false;
                        }
                    },
                    cx,
                )
            })
            .collect();
        rate_box = rate_box.child(crate::dropdown("experimental-interval-rate-list", rows));
    }
    let body = crate::probe::measured("experimental-interval", div())
        .id("experimental-interval")
        .flex()
        .flex_col()
        .gap_2()
        .w(px(FORM_WIDTH))
        .p_2()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .occlude()
        .child(div().flex().justify_end().child(action(
            "experimental-interval-close",
            "\u{2715}",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.experimental.interval = None;
                cx.notify();
            }),
        )))
        .child(
            div()
                .flex()
                .flex_wrap()
                .items_center()
                .gap(px(6.0))
                .child(messages_box)
                .child(rate_box)
                .child(action(
                    "experimental-interval-set",
                    "Set",
                    theme::ACCENT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        set(this);
                        cx.notify();
                    }),
                ))
                .child(action(
                    "experimental-interval-setall",
                    "Set All",
                    theme::ACCENT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        set_all(this);
                        cx.notify();
                    }),
                )),
        );
    let size = window.viewport_size();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(
                    ((size.width - px(FORM_WIDTH)) / 2.0).max(px(0.0)),
                    px(120.0),
                ))
                .child(body),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// `experimental.interval.*`: whether the form shows, its message and its rate.
pub fn record_facts(form: Option<&IntervalForm>) {
    facts::record("experimental.interval.open", form.is_some());
    facts::record(
        "experimental.interval.message",
        form.map_or("none", |form| form.message().0),
    );
    facts::record(
        "experimental.interval.rate",
        form.map_or("none", |form| form.rate.value()),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_list_is_every_message_by_name() {
        let all = messages();
        assert_eq!(all.len(), 353, "MAVLINK_MSG_ID's names");
        assert!(all.windows(2).all(|pair| pair[0].0 < pair[1].0), "sorted");
        assert!(all.contains(&("ATTITUDE", 30)));
        assert!(all.contains(&("HEARTBEAT", 0)));
    }

    #[test]
    fn a_rate_is_an_interval_in_microseconds_and_zero_or_less_goes_as_it_is() {
        assert_eq!(interval(2.0), 500_000.0);
        assert_eq!(interval(4.0), 250_000.0);
        assert_eq!(interval(0.0), 0.0);
        assert_eq!(interval(-1.0), -1.0);
        // In float, as the C# works it.
        assert_eq!(interval(3.0), 1.0_f32 / 3.0 * 1_000_000.0);
        assert!(parse_rate("x").is_err());
        assert_eq!(parse_rate(" 7 "), Ok(7.0));
    }
}
