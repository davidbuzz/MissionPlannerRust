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

//! The flight screen's Payload Control page, `tabPayload`: the gimbal's tilt, pan and roll.
//!
//! Three track bars - Tilt standing up, Pan and Roll lying down - each beside a box showing the
//! mount's reported angle (`campointa`, `campointc`, `campointb`), and Reset Position. Moving a
//! bar sends all three angles as `DO_MOUNT_CONTROL` in the MAVLink-targeting mode; Reset Position
//! puts the bars back to zero, sets the mount to MAVLink targeting with no stabilisation
//! (`DO_MOUNT_CONFIGURE`) and sends the zeros. Neither waits for an answer: `setMountControl` and
//! `setMountConfigure` send with `requireack` false.
//! `// C#: GCSViews/FlightData.cs:1473-1480, 2959-2963, GCSViews/FlightData.Designer.cs:2088-2191,
//! ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4561-4594`
//!
//! Video Control, the page's fifth control, opens the gimbal's video in a window of its own: the
//! map menu's Gimbal Video Pop Out (`crate::gimbal_video`).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::Cell;
use std::rc::Rc;

use gpui::{AnyElement, Bounds, Context, Pixels, div, prelude::*, px, rgb};
use mp_link::commands;
use mp_mavlink_dialects::all::{MavCmd, MavMessage};
use mp_vehicle::VehicleId;

use crate::MissionPlanner;
use crate::i18n::fl;
use crate::ui::theme;

/// `MAV_MOUNT_MODE.MAVLINK_TARGETING`. `// C#: ExtLibs/Mavlink/Mavlink.cs (enum MAV_MOUNT_MODE)`
pub const MAVLINK_TARGETING: u8 = 2;

/// `setMountControl(pa, pb, pc, false)` with the bars' values times a hundred: centi-degrees made
/// degrees again - pitch, roll and yaw in the first three parameters, the mode in the seventh.
/// `// C#: GCSViews/FlightData.cs:2959-2963, ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4581-4587`
#[must_use]
pub fn mount_control(target: VehicleId, pitch: i32, roll: i32, yaw: i32) -> MavMessage {
    // `(float) trackBar.Value * 100.0f`, then `(float)(pa * 0.01)`.
    #[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)] // 180 at most
    let degrees = |value: i32| (f64::from(value as f32 * 100.0) * 0.01) as f32;
    let command = u16::try_from(MavCmd::MAV_CMD_DO_MOUNT_CONTROL.0).unwrap_or(u16::MAX);
    commands::command_long(
        target,
        command,
        [
            degrees(pitch),
            degrees(roll),
            degrees(yaw),
            0.0,
            0.0,
            0.0,
            f32::from(MAVLINK_TARGETING),
        ],
    )
}

/// `setMountConfigure(mode, stabroll, stabpitch, stabyaw)`: the mode and the three
/// stabilisation flags as 1 or 0. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4561-4574`
#[must_use]
pub fn mount_configure(target: VehicleId, mode: u8, stabilise: [bool; 3]) -> MavMessage {
    let command = u16::try_from(MavCmd::MAV_CMD_DO_MOUNT_CONFIGURE.0).unwrap_or(u16::MAX);
    let flag = |on: bool| if on { 1.0 } else { 0.0 };
    commands::command_long(
        target,
        command,
        [
            f32::from(mode),
            flag(stabilise[0]),
            flag(stabilise[1]),
            flag(stabilise[2]),
            0.0,
            0.0,
            0.0,
        ],
    )
}

/// One of the three bars.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    /// `trackBarPitch`, in the Tilt box.
    Pitch,
    /// `trackBarYaw`, in the Pan box.
    Yaw,
    /// `trackBarRoll`, in the Roll box.
    Roll,
}

impl Axis {
    /// The id a script presses it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Pitch => "fly-gimbal-pitch",
            Self::Yaw => "fly-gimbal-yaw",
            Self::Roll => "fly-gimbal-roll",
        }
    }
}

/// How far the thumb's centre stops short of each end of the bar, in pixels, as a WinForms
/// track bar keeps its thumb inside the control.
const MARGIN: f32 = 8.0;
/// The thumb's length along the bar.
const THUMB: f32 = 10.0;

/// A `TrackBar`: its value within its range, and a press on it.
#[derive(Debug)]
pub struct TrackBar {
    /// `Value`.
    pub value: i32,
    /// `Minimum`.
    pub minimum: i32,
    /// `Maximum`.
    pub maximum: i32,
    /// `LargeChange`: how far a press beside the thumb moves it.
    pub large_change: i32,
    /// `Orientation.Vertical`, with the maximum at the top.
    pub vertical: bool,
    /// Whether the thumb is held.
    dragging: bool,
    /// Where the bar was laid out.
    pub bounds: Rc<Cell<Option<Bounds<Pixels>>>>,
}

impl TrackBar {
    fn new(minimum: i32, maximum: i32, vertical: bool) -> Self {
        Self {
            value: 0,
            minimum,
            maximum,
            large_change: 10,
            vertical,
            dragging: false,
            bounds: Rc::new(Cell::new(None)),
        }
    }

    /// How far along the bar the value is, from the minimum's end.
    #[must_use]
    pub fn fraction(&self) -> f32 {
        let span = self.maximum - self.minimum;
        if span == 0 {
            return 0.0;
        }
        #[allow(clippy::cast_precision_loss)] // a few hundred at most
        let fraction = (self.value - self.minimum) as f32 / span as f32;
        fraction
    }

    /// The value at `fraction` of the way from the minimum's end, to the nearest step.
    fn value_at(&self, fraction: f32) -> i32 {
        #[allow(clippy::cast_precision_loss)]
        let span = (self.maximum - self.minimum) as f32;
        #[allow(clippy::cast_possible_truncation)]
        let value = self.minimum + (fraction.clamp(0.0, 1.0) * span).round() as i32;
        value
    }

    /// A press `fraction` of the way from the minimum's end, `thumb` the thumb's length as a
    /// fraction: on the thumb it takes hold of it, beside it the thumb moves one `LargeChange`
    /// toward the press. Returns whether the value changed - a `Scroll`.
    pub fn press(&mut self, fraction: f32, thumb: f32) -> bool {
        let at = self.fraction();
        if (fraction - at).abs() <= thumb / 2.0 {
            self.dragging = true;
            return false;
        }
        let step = if fraction > at {
            self.large_change
        } else {
            -self.large_change
        };
        self.set(self.value + step)
    }

    /// The thumb dragged to `fraction`. Returns whether the value changed.
    pub fn drag(&mut self, fraction: f32) -> bool {
        if !self.dragging {
            return false;
        }
        let value = self.value_at(fraction);
        self.set(value)
    }

    /// The thumb let go.
    pub fn release(&mut self) {
        self.dragging = false;
    }

    /// The value, constrained to the range. Returns whether it changed.
    pub fn set(&mut self, value: i32) -> bool {
        let value = value.clamp(self.minimum, self.maximum);
        let changed = value != self.value;
        self.value = value;
        changed
    }

    /// Where a point in the window is along the bar, from the minimum's end, and the thumb's
    /// length, both as fractions of the travel; `None` before the bar is laid out.
    fn fraction_of(&self, position: gpui::Point<Pixels>) -> Option<(f32, f32)> {
        let laid_out = self.bounds.get()?;
        let (along, length) = if self.vertical {
            (
                // The maximum is at the top.
                f32::from(laid_out.origin.y + laid_out.size.height - position.y),
                f32::from(laid_out.size.height),
            )
        } else {
            (
                f32::from(position.x - laid_out.origin.x),
                f32::from(laid_out.size.width),
            )
        };
        let travel = (length - 2.0 * MARGIN).max(1.0);
        Some(((along - MARGIN) / travel, THUMB / travel))
    }
}

/// The page's three bars.
#[derive(Debug)]
pub struct Payload {
    /// `trackBarPitch`: -90 to 90, standing up.
    pub pitch: TrackBar,
    /// `trackBarYaw`: -180 to 180.
    pub yaw: TrackBar,
    /// `trackBarRoll`: -90 to 90.
    pub roll: TrackBar,
}

impl Default for Payload {
    /// The Designer's ranges; every `LargeChange` is 10 and every value 0.
    /// `// C#: GCSViews/FlightData.Designer.cs:2126-2132, 2148-2154, 2179-2186`
    fn default() -> Self {
        Self {
            pitch: TrackBar::new(-90, 90, true),
            yaw: TrackBar::new(-180, 180, false),
            roll: TrackBar::new(-90, 90, false),
        }
    }
}

impl Payload {
    /// One bar, to move.
    pub const fn bar_mut(&mut self, axis: Axis) -> &mut TrackBar {
        match axis {
            Axis::Pitch => &mut self.pitch,
            Axis::Yaw => &mut self.yaw,
            Axis::Roll => &mut self.roll,
        }
    }

    /// `gimbalTrackbar_Scroll`: what a bar's move sends - the three values.
    #[must_use]
    pub fn scroll_message(&self, target: VehicleId) -> MavMessage {
        mount_control(target, self.pitch.value, self.roll.value, self.yaw.value)
    }

    /// `BUT_resetGimbalPos_Click`: the bars to zero, then MAVLink targeting and the zeros.
    pub fn reset(&mut self, target: VehicleId) -> Vec<MavMessage> {
        self.pitch.value = 0;
        self.roll.value = 0;
        self.yaw.value = 0;
        vec![
            mount_configure(target, MAVLINK_TARGETING, [false; 3]),
            self.scroll_message(target),
        ]
    }

    /// Publishes what a UI test asserts on.
    pub fn record_facts(&self, state: Option<&mp_vehicle::VehicleState>) {
        crate::facts::record("fly.gimbal.pitch", self.pitch.value);
        crate::facts::record("fly.gimbal.yaw", self.yaw.value);
        crate::facts::record("fly.gimbal.roll", self.roll.value);
        crate::facts::record(
            "fly.gimbal.campoint",
            state.map_or_else(|| "none".to_owned(), |state| campoint(state).join(";")),
        );
    }
}

/// `campointa`, `campointb` and `campointc` as the boxes bound to them show them: a `float`'s
/// shortest text.
#[must_use]
pub fn campoint(state: &mp_vehicle::VehicleState) -> [String; 3] {
    let mount = state.mount;
    [
        mount.pointing_a.to_string(),
        mount.pointing_b.to_string(),
        mount.pointing_c.to_string(),
    ]
}

/// A control placed where the `.resx` puts it, within its parent.
fn at(left: f32, top: f32, width: f32, height: f32) -> gpui::Div {
    div()
        .absolute()
        .left(px(left))
        .top(px(top))
        .w(px(width))
        .h(px(height))
}

/// A `GroupBox`: a border with its caption on it.
fn group_box(caption: &'static str, bounds: (f32, f32, f32, f32)) -> gpui::Div {
    let (left, top, width, height) = bounds;
    at(left, top, width, height)
        .child(
            div()
                .absolute()
                .left_0()
                .right_0()
                .top(px(6.0))
                .bottom_0()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER)),
        )
        .child(
            div()
                .absolute()
                .left(px(6.0))
                .top_0()
                .px_1()
                .bg(rgb(theme::BG))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(caption),
        )
}

/// A box showing one `campoint`.
fn angle_box(id: &'static str, text: String, bounds: (f32, f32, f32, f32)) -> AnyElement {
    let (left, top, width, height) = bounds;
    crate::probe::measured(id, at(left, top, width, height))
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .overflow_hidden()
        .child(text)
        .into_any_element()
}

/// A track bar: its channel, a tick every ten, and the thumb at the value. A press on the thumb
/// takes hold of it; beside it, one `LargeChange`.
fn track_bar(
    axis: Axis,
    bar: &TrackBar,
    bounds: (f32, f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (left, top, width, height) = bounds;
    let laid = Rc::clone(&bar.bounds);
    let length = if bar.vertical { height } else { width };
    let travel = length - 2.0 * MARGIN;
    let along = |fraction: f32| MARGIN + fraction * travel;
    let mut channel = div().relative().size_full().child(
        gpui::canvas(
            move |laid_out, _window, _cx| laid.set(Some(laid_out)),
            |_bounds, (), _window, _cx| {},
        )
        .absolute()
        .inset_0(),
    );
    channel = channel.child(if bar.vertical {
        div()
            .absolute()
            .left(px(10.0))
            .top(px(MARGIN))
            .w(px(4.0))
            .h(px(travel))
            .rounded_sm()
            .bg(rgb(theme::BORDER))
    } else {
        div()
            .absolute()
            .top(px(10.0))
            .left(px(MARGIN))
            .h(px(4.0))
            .w(px(travel))
            .rounded_sm()
            .bg(rgb(theme::BORDER))
    });
    // `TickFrequency = 10`.
    let mut tick = bar.minimum;
    while tick <= bar.maximum {
        #[allow(clippy::cast_precision_loss)]
        let fraction = (tick - bar.minimum) as f32 / (bar.maximum - bar.minimum).max(1) as f32;
        let at_px = along(fraction);
        channel = channel.child(if bar.vertical {
            div()
                .absolute()
                .left(px(24.0))
                .top(px(length - at_px))
                .w(px(4.0))
                .h(px(1.0))
                .bg(rgb(theme::DIM))
        } else {
            div()
                .absolute()
                .top(px(24.0))
                .left(px(at_px))
                .w(px(1.0))
                .h(px(4.0))
                .bg(rgb(theme::DIM))
        });
        tick += 10;
    }
    let thumb_at = along(bar.fraction());
    channel = channel.child(if bar.vertical {
        div()
            .absolute()
            .left(px(2.0))
            .top(px(length - thumb_at - THUMB / 2.0))
            .w(px(20.0))
            .h(px(THUMB))
            .rounded_sm()
            .bg(rgb(theme::ACCENT))
    } else {
        div()
            .absolute()
            .top(px(2.0))
            .left(px(thumb_at - THUMB / 2.0))
            .w(px(THUMB))
            .h(px(20.0))
            .rounded_sm()
            .bg(rgb(theme::ACCENT))
    });
    crate::probe::measured(axis.id(), at(left, top, width, height))
        .id(axis.id())
        .cursor_pointer()
        .child(channel)
        .on_mouse_down(
            gpui::MouseButton::Left,
            cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                let bar = this.fly_data.payload.bar_mut(axis);
                if let Some((fraction, thumb)) = bar.fraction_of(event.position)
                    && bar.press(fraction, thumb)
                {
                    this.fly_gimbal_scroll();
                }
                cx.notify();
            }),
        )
        .on_mouse_move(
            cx.listener(move |this, event: &gpui::MouseMoveEvent, _window, cx| {
                if event.pressed_button != Some(gpui::MouseButton::Left) {
                    return;
                }
                let bar = this.fly_data.payload.bar_mut(axis);
                if let Some((fraction, _)) = bar.fraction_of(event.position)
                    && bar.drag(fraction)
                {
                    this.fly_gimbal_scroll();
                    cx.notify();
                }
            }),
        )
        .on_mouse_up(
            gpui::MouseButton::Left,
            cx.listener(move |this, _event: &gpui::MouseUpEvent, _window, _cx| {
                this.fly_data.payload.bar_mut(axis).release();
            }),
        )
        .into_any_element()
}

/// A button: `MyButton`, at its place, its text in two lines where one does not fit.
fn button(
    id: &'static str,
    label: &'static str,
    bounds: (f32, f32, f32, f32),
    enabled: bool,
    on_click: impl Fn(&gpui::ClickEvent, &mut gpui::Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let (left, top, width, height) = bounds;
    let base = crate::probe::measured(id, at(left, top, width, height))
        .id(id)
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .text_size(px(10.0))
        .line_height(px(11.0))
        .text_center()
        .child(label);
    if enabled {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::ACCENT))
            .text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(on_click)
            .into_any_element()
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .on_click(on_click)
            .into_any_element()
    }
}

/// The page, each control where the `.resx` puts it in `tabPayload`.
/// `// C#: GCSViews/FlightData.resx (tabPayload and its controls)`
pub fn page(
    payload: &Payload,
    state: Option<&mp_vehicle::VehicleState>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let [a, b, c] = state.map_or_else(|| [String::new(), String::new(), String::new()], campoint);
    let tilt = group_box("Tilt", (10.0, 6.0, 119.0, 124.0))
        .child(angle_box(
            "fly-gimbal-pitch-pos",
            a,
            (6.0, 18.0, 50.0, 20.0),
        ))
        .child(track_bar(
            Axis::Pitch,
            &payload.pitch,
            (62.0, 11.0, 45.0, 104.0),
            cx,
        ));
    let pan = group_box("Pan", (144.0, 6.0, 178.0, 76.0))
        .child(angle_box("fly-gimbal-yaw-pos", c, (6.0, 18.0, 51.0, 20.0)))
        .child(track_bar(
            Axis::Yaw,
            &payload.yaw,
            (63.0, 18.0, 104.0, 45.0),
            cx,
        ));
    let roll = group_box("Roll", (144.0, 82.0, 178.0, 76.0))
        .child(angle_box("fly-gimbal-roll-pos", b, (6.0, 19.0, 51.0, 20.0)))
        .child(track_bar(
            Axis::Roll,
            &payload.roll,
            (63.0, 19.0, 104.0, 45.0),
            cx,
        ));
    crate::probe::measured("panel:payload", div())
        .relative()
        .flex_shrink_0()
        .w(px(322.0))
        .h(px(158.0))
        .child(tilt)
        .child(pan)
        .child(roll)
        // The buttons' words in the configured culture (`crate::i18n`).
        // `// C#: GCSViews/FlightData.resx (BUT_resetGimbalPos.Text, BUT_GimbalVideo.Text)`
        .child(button(
            "fly-gimbal-reset",
            fl!("flightdata-BUT_resetGimbalPos-Text"),
            (10.0, 134.0, 56.0, 23.0),
            true,
            cx.listener(|this, _event, _window, cx| {
                this.fly_gimbal_reset();
                cx.notify();
            }),
        ))
        // `BUT_GimbalVideo.Click += gimbalVideoPopOutToolStripMenuItem_Click`.
        // `// C#: GCSViews/FlightData.Designer.cs:2106`
        .child(button(
            "fly-gimbal-video",
            fl!("flightdata-BUT_GimbalVideo-Text"),
            (73.0, 134.0, 56.0, 23.0),
            true,
            cx.listener(|this, _event, window, cx| {
                this.gimbal_video_pop_out(window, cx);
                cx.notify();
            }),
        ))
        .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    /// Each bar's range and step is the Designer's.
    #[test]
    fn the_bars_are_the_designers() {
        let payload = Payload::default();
        assert_eq!((payload.pitch.minimum, payload.pitch.maximum), (-90, 90));
        assert!(payload.pitch.vertical);
        assert_eq!((payload.yaw.minimum, payload.yaw.maximum), (-180, 180));
        assert_eq!((payload.roll.minimum, payload.roll.maximum), (-90, 90));
        for bar in [&payload.pitch, &payload.yaw, &payload.roll] {
            assert_eq!(bar.large_change, 10);
            assert_eq!(bar.value, 0);
        }
    }

    /// A press beside the thumb moves it one `LargeChange` toward the press and stops at the
    /// ends; a press on it takes hold, and a drag moves it to the pointer.
    #[test]
    fn a_press_beside_the_thumb_steps_and_on_it_drags() {
        let mut bar = TrackBar::new(-90, 90, false);
        assert!(bar.press(0.9, 0.05));
        assert_eq!(bar.value, 10);
        assert!(bar.press(0.1, 0.05));
        assert!(bar.press(0.1, 0.05));
        assert_eq!(bar.value, -10);
        // On the thumb, at -10's place: hold, no scroll.
        assert!(!bar.press(bar.fraction(), 0.05));
        assert!(bar.drag(1.0));
        assert_eq!(bar.value, 90);
        assert!(!bar.drag(1.5), "clamped: no change");
        bar.release();
        assert!(!bar.drag(0.0), "let go");
        for _ in 0..30 {
            bar.press(0.0, 0.05);
        }
        assert_eq!(bar.value, -90);
    }

    /// A bar's move sends the three values in degrees with the MAVLink-targeting mode; Reset
    /// Position zeroes them and configures the mount first.
    #[test]
    fn the_bars_send_mount_control_and_reset_configures_first() {
        let mut payload = Payload::default();
        payload.pitch.set(-30);
        payload.roll.set(10);
        payload.yaw.set(170);
        assert_eq!(
            crate::fly::describe(&payload.scroll_message(target())),
            "COMMAND_LONG MAV_CMD_DO_MOUNT_CONTROL -30,10,170,0,0,0,2"
        );
        let sent: Vec<String> = payload
            .reset(target())
            .iter()
            .map(crate::fly::describe)
            .collect();
        assert_eq!(
            sent,
            [
                "COMMAND_LONG MAV_CMD_DO_MOUNT_CONFIGURE 2,0,0,0,0,0,0",
                "COMMAND_LONG MAV_CMD_DO_MOUNT_CONTROL 0,0,0,0,0,0,2",
            ]
        );
        assert_eq!(
            (payload.pitch.value, payload.roll.value, payload.yaw.value),
            (0, 0, 0)
        );
    }

    /// The boxes show the mount's reported angles as a `float` prints.
    #[test]
    fn the_boxes_show_the_mounts_angles() {
        let mut state = mp_vehicle::VehicleState::default();
        state.mount.pointing_a = -12.5;
        state.mount.pointing_b = 0.0;
        state.mount.pointing_c = 270.25;
        assert_eq!(campoint(&state), ["-12.5", "0", "270.25"]);
    }
}
