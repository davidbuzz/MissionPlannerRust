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

//! Drawing the Joystick Setup page at its Designer's places.
//!
//! The page's own controls are at `JoystickSetup.resx`'s `Location` and `Size`; each channel row is
//! a `JoystickAxis` - 427 by 28, its six controls at that control's Designer places - stacked from
//! `label8`'s bottom; each button row is where `doButtontoUI` puts it, 100 pixels right of the
//! channel rows, its controls side by side from their widths. A combo box's list drops down over
//! the page (deferred and anchored, thirty rows, the wheel moving them), and the boxes and forms
//! are drawn over the whole window, as `CustomMessageBox` and `ShowDialog` draw theirs. The
//! colours are this application's.
//! `// C#: Joystick/JoystickSetup.Designer.cs; JoystickSetup.resx; JoystickAxis.cs:88-191;
//! JoystickSetup.cs:71-118, 347-442`

use std::rc::Rc;

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px,
    rgb,
};

use super::{BUTTON_ROWS, Editing, List, Sticks, forms};
use crate::MissionPlanner;
use crate::ui::{action, theme};

/// A place: x, y, width, height.
type Place = (f32, f32, f32, f32);

/// `CMB_joysticks`.
const CMB_JOYSTICKS: Place = (72.0, 12.0, 202.0, 21.0);
/// `BUT_enable`.
const BUT_ENABLE: Place = (280.0, 12.0, 58.0, 23.0);
/// `BUT_save`.
const BUT_SAVE: Place = (344.0, 12.0, 44.0, 23.0);
/// `but_export`.
const BUT_EXPORT: Place = (394.0, 12.0, 49.0, 23.0);
/// `but_import`.
const BUT_IMPORT: Place = (449.0, 12.0, 49.0, 23.0);
/// `$this.Size`.
const PAGE: (f32, f32) = (702.0, 331.0);
/// The first channel row: `label8.Bottom`, 47 + 13.
const ROWS_TOP: f32 = 60.0;
/// `JoystickAxis`'s size.
const ROW: (f32, f32) = (427.0, 28.0);
/// The button rows' left: the first channel row's right plus 100.
const BUTTONS_LEFT: f32 = ROW.0 + 100.0;
/// A `Button`'s default size, which `but_detect` and `but_settings` keep.
const BUTTON: (f32, f32) = (75.0, 23.0);
/// A list row's height.
const LIST_ROW: f32 = 16.0;

/// The expo boxes' ids, which a text field needs to be `'static`.
const EXPO_IDS: [&str; super::CHANNEL_ROWS] = [
    "joystick-rc1-expo",
    "joystick-rc2-expo",
    "joystick-rc3-expo",
    "joystick-rc4-expo",
    "joystick-rc5-expo",
    "joystick-rc6-expo",
    "joystick-rc7-expo",
    "joystick-rc8-expo",
    "joystick-rc9-expo",
    "joystick-rc10-expo",
    "joystick-rc11-expo",
    "joystick-rc12-expo",
    "joystick-rc13-expo",
    "joystick-rc14-expo",
    "joystick-rc15-expo",
    "joystick-rc16-expo",
];

/// A form's boxes' ids.
const FORM_IDS: [&str; 4] = [
    "joystick-form-n0",
    "joystick-form-n1",
    "joystick-form-n2",
    "joystick-form-n3",
];

/// A channel row's top.
#[allow(clippy::cast_precision_loss)] // sixteen rows
fn row_top(row: usize) -> f32 {
    ROWS_TOP + ROW.1 * row as f32
}

/// The page's size as the C# grows it: the channel rows' bottom, and for each button row made the
/// 25 pixels `doButtontoUI` adds when its Settings comes within 30 of the bottom, and the width
/// out to the last Settings.
/// `// C#: Joystick/JoystickSetup.cs:113-117, 437-441`
#[must_use]
pub fn page_size(sticks: &Sticks) -> (f32, f32) {
    let mut height = PAGE.1.max(row_top(super::CHANNEL_ROWS));
    let mut width = PAGE.0.max(ROW.0);
    for slot in 0..sticks.page.buttons.len() {
        let (x, y) = (BUTTONS_LEFT, row_top(slot));
        let bottom = y + BUTTON.1;
        if bottom + 30.0 > height {
            height += 25.0;
        }
        let right = x + 402.0 + BUTTON.0;
        if right > width {
            width = right + 5.0;
        }
    }
    (width, height)
}

fn at((x, y, width, height): Place) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

fn label(x: f32, y: f32, text: impl Into<SharedString>) -> Div {
    crate::config::servo_output::label(x, y, text)
}

/// A `MyButton` at its place.
fn button(
    id: impl Into<String>,
    text: impl Into<SharedString>,
    place: Place,
    on_click: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = id.into();
    at(place)
        .child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .size_full()
                .flex()
                .items_center()
                .justify_center()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::ACTION))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .overflow_hidden()
                .whitespace_nowrap()
                .cursor_pointer()
                .hover(|style| style.border_color(rgb(theme::ACCENT)))
                .child(text.into())
                .on_click(cx.listener(move |this, _event, window, cx| {
                    on_click(this, window, cx);
                    cx.notify();
                })),
        )
        .into_any_element()
}

/// A combo box: its text and the arrow, a click dropping its list down.
fn combo(
    id: impl Into<String>,
    text: impl Into<SharedString>,
    place: Place,
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = id.into();
    at(place)
        .child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .size_full()
                .flex()
                .items_center()
                .px_1()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::ACTION))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.border_color(rgb(theme::ACCENT)))
                .child(
                    div()
                        .flex_1()
                        .overflow_hidden()
                        .whitespace_nowrap()
                        .child(text.into()),
                )
                .child(div().text_size(px(7.0)).child("▼"))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    on_click(this);
                    cx.notify();
                })),
        )
        .into_any_element()
}

/// A check box and its text.
fn check(
    id: impl Into<String>,
    text: &'static str,
    checked: bool,
    (x, y): (f32, f32),
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let id = id.into();
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .flex()
                .items_center()
                .gap_1()
                .cursor_pointer()
                .child(
                    div()
                        .size(px(13.0))
                        .flex()
                        .items_center()
                        .justify_center()
                        .border_1()
                        .border_color(rgb(theme::DIM))
                        .bg(rgb(theme::BG))
                        .children(checked.then(|| div().size(px(6.0)).bg(rgb(theme::ACCENT)))),
                )
                .children((!text.is_empty()).then(|| {
                    div()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(text)
                }))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    on_click(this);
                    cx.notify();
                })),
        )
        .into_any_element()
}

/// A `HorizontalProgressBar`: filled to the value within its range, held one above its minimum
/// and at its maximum; with its label, the value at a red line where it lies (`maxline`).
/// `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:74-116, 166-205; Joystick/JoystickAxis.cs:215-222`
fn bar(id: String, place: Place, value: i32, (min, max): (i32, i32), labelled: bool) -> AnyElement {
    let shown = if value <= min {
        min + 1
    } else {
        value.min(max)
    };
    #[allow(clippy::cast_precision_loss)] // PWM values
    let fraction = (shown - min) as f32 / (max - min).max(1) as f32;
    let mut body = crate::probe::measured(id, div())
        .relative()
        .size_full()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .overflow_hidden()
        .child(
            div()
                .absolute()
                .left_0()
                .top_0()
                .h_full()
                .w(gpui::relative(fraction))
                .bg(rgb(theme::OK)),
        );
    if labelled && value != 0 {
        body = body
            .child(
                div()
                    .absolute()
                    .top_0()
                    .h_full()
                    .w(px(2.0))
                    .left(gpui::relative(fraction))
                    .bg(rgb(theme::ALERT)),
            )
            .child(
                div()
                    .absolute()
                    .left_0()
                    .top_0()
                    .size_full()
                    .flex()
                    .items_center()
                    .justify_center()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(value.to_string()),
            );
    }
    at(place).child(body).into_any_element()
}

/// A list dropped down under its combo, over the page: thirty rows from the top shown, the
/// wheel moving them, each row `<id>-<suffix>`.
fn list(
    id: &str,
    sticks: &Sticks,
    which: List,
    (x, y, width): (f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (rows, selected) = sticks.list_rows(which);
    let shown = crate::config::servo_output::LIST_ROWS_SHOWN;
    let mut body = crate::probe::measured(format!("{id}-list"), div())
        .id(SharedString::from(format!("{id}-list")))
        .min_w(px(width))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .occlude()
        .on_scroll_wheel(
            cx.listener(|this, event: &gpui::ScrollWheelEvent, _window, cx| {
                let delta = event.delta.pixel_delta(px(LIST_ROW));
                let lines = (f32::from(delta.y) / LIST_ROW).round();
                #[allow(clippy::cast_possible_truncation)]
                let lines = -(lines as i32);
                if lines != 0 {
                    this.sticks.scroll_list(lines);
                    cx.notify();
                }
                cx.stop_propagation();
            }),
        );
    for (index, (suffix, text)) in rows
        .into_iter()
        .enumerate()
        .skip(sticks.page.top)
        .take(shown)
    {
        let name = format!("{id}-{suffix}");
        body = body.child(
            crate::probe::measured(name.clone(), div())
                .id(SharedString::from(name))
                .flex_shrink_0()
                .h(px(LIST_ROW))
                .px_1()
                .text_xs()
                .whitespace_nowrap()
                .bg(rgb(if selected == Some(index) {
                    theme::BORDER
                } else {
                    theme::PANEL
                }))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::ACTION)))
                .child(text)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.sticks.choose_from_list(index);
                    cx.notify();
                })),
        );
    }
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .child(gpui::deferred(gpui::anchored().snap_to_window().child(body)).with_priority(1))
        .into_any_element()
}

/// The keyboard handler every text box here shares.
fn on_key(
    cx: &mut Context<MissionPlanner>,
) -> impl Fn(&KeyDownEvent, &mut Window, &mut gpui::App) + 'static {
    cx.listener(|this, event: &KeyDownEvent, _window, cx| {
        if this.sticks.key(event) {
            cx.stop_propagation();
            cx.notify();
        }
    })
}

/// A text box: typed into while it has the keyboard, a click giving it the keyboard otherwise.
#[allow(clippy::too_many_arguments)]
fn text_box(
    id: &'static str,
    field: &crate::textfield::TextField,
    editing: bool,
    handle: &FocusHandle,
    place: Place,
    on_begin: impl Fn(&mut MissionPlanner) + 'static,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if editing {
        return at(place)
            .overflow_hidden()
            .child(crate::textfield::text_field(
                id,
                field,
                handle,
                handle.is_focused(window),
                px(place.2),
                on_key(cx),
            ))
            .into_any_element();
    }
    let handle = handle.clone();
    at(place)
        .child(
            crate::probe::measured(id, div())
                .id(id)
                .size_full()
                .flex()
                .items_center()
                .px_1()
                .bg(rgb(theme::BG))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .overflow_hidden()
                .whitespace_nowrap()
                .cursor_text()
                .child(field.value().to_owned())
                .on_click(cx.listener(move |this, _event, window, cx| {
                    on_begin(this);
                    handle.focus(window, cx);
                    cx.notify();
                })),
        )
        .into_any_element()
}

/// The page.
pub fn page(
    sticks: &Sticks,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let page = &sticks.page;
    let (width, height) = page_size(sticks);
    let mut body = crate::probe::measured("joystick-page", div())
        .relative()
        .w(px(width))
        .h(px(height))
        .child(label(19.0, 15.0, "Joystick"))
        .child(combo(
            "joystick-devices",
            page.text.clone(),
            CMB_JOYSTICKS,
            |this| this.sticks.devices_click(),
            cx,
        ))
        .child(button(
            "joystick-enable",
            sticks.enable_text(),
            BUT_ENABLE,
            |this, _window, _cx| this.sticks.enable_click(&mut this.persisted),
            cx,
        ))
        .child(button(
            "joystick-save",
            "Save",
            BUT_SAVE,
            |this, _window, _cx| this.sticks.save_click(&mut this.persisted),
            cx,
        ))
        .child(button(
            "joystick-export",
            "Export",
            BUT_EXPORT,
            |this, window, cx| {
                this.sticks.export_click();
                this.joystick_focus.focus(window, cx);
            },
            cx,
        ))
        .child(button(
            "joystick-import",
            "Import",
            BUT_IMPORT,
            |this, _window, _cx| this.sticks.import_click(),
            cx,
        ))
        .child(label(500.0, 17.0, page.label.clone()))
        .child(label(69.0, 47.0, "Controller Axis"))
        .child(label(197.0, 47.0, "Output"))
        .child(label(307.0, 47.0, "Expo"))
        .child(label(411.0, 47.0, "Reverse"))
        .child(check(
            "joystick-manual",
            "Manual Control",
            page.manual,
            (464.0, 47.0),
            |this| this.sticks.toggle_manual(),
            cx,
        ))
        .child(check(
            "joystick-elevons",
            "Elevons",
            page.elevons,
            (434.0, 81.0),
            |this| this.sticks.toggle_elevons(),
            cx,
        ));
    for (index, row) in page.rows.iter().enumerate() {
        let channel = index + 1;
        let y = row_top(index);
        body = body
            .child(label(2.0, y + 8.0, format!("RC {channel}")))
            .child(combo(
                format!("joystick-rc{channel}-axis"),
                row.axis.clone(),
                (65.0, y + 3.0, 70.0, 21.0),
                move |this| this.sticks.toggle_list(List::Axis(index)),
                cx,
            ))
            .child(button(
                format!("joystick-rc{channel}-detect"),
                "Auto Detect",
                (141.0, y + 3.0, 45.0, 23.0),
                move |this, _window, _cx| this.sticks.detect_axis(index),
                cx,
            ))
            .child(bar(
                format!("joystick-rc{channel}-bar"),
                (192.0, y + 2.0, 100.0, 23.0),
                i32::from(page.rc.get(index).copied().unwrap_or(0)),
                (800, 2200),
                true,
            ))
            .children(EXPO_IDS.get(index).map(|id| {
                text_box(
                    id,
                    &row.expo,
                    page.editing == Some(Editing::Expo(index)),
                    handle,
                    (300.0, y + 7.0, 100.0, 13.0),
                    move |this| this.sticks.begin_expo(index),
                    window,
                    cx,
                )
            }))
            .child(check(
                format!("joystick-rc{channel}-reverse"),
                "",
                row.reverse,
                (406.0, y + 6.0),
                move |this| this.sticks.toggle_reverse(index),
                cx,
            ));
    }
    for (slot, row) in page.buttons.iter().enumerate().take(BUTTON_ROWS) {
        let (x, y) = (BUTTONS_LEFT, row_top(slot));
        body = body
            .child(label(x, y + 4.0, format!("But {}", slot + 1)))
            .child(combo(
                format!("joystick-but{slot}-no"),
                row.number.clone(),
                (x + 47.0, y, 70.0, 21.0),
                move |this| this.sticks.toggle_list(List::Number(slot)),
                cx,
            ))
            .child(button(
                format!("joystick-but{slot}-detect"),
                "Detect",
                (x + 117.0, y, BUTTON.0, BUTTON.1),
                move |this, _window, _cx| this.sticks.detect_button(slot),
                cx,
            ))
            .child(bar(
                format!("joystick-but{slot}-bar"),
                (x + 192.0, y, 100.0, 21.0),
                if page.pressed.get(slot).copied().unwrap_or(false) {
                    100
                } else {
                    0
                },
                (0, 100),
                false,
            ))
            .child(combo(
                format!("joystick-but{slot}-action"),
                row.action.clone(),
                (x + 297.0, y, 100.0, 21.0),
                move |this| this.sticks.toggle_list(List::Action(slot)),
                cx,
            ))
            .child(button(
                format!("joystick-but{slot}-settings"),
                "Settings",
                (x + 402.0, y, BUTTON.0, BUTTON.1),
                move |this, _window, _cx| this.sticks.settings_click(slot),
                cx,
            ));
    }
    // An open list goes last, so it lies over the rows below its combo.
    let open = match page.open {
        Some(List::Devices) => Some((
            "joystick-devices".to_owned(),
            (CMB_JOYSTICKS.0, CMB_JOYSTICKS.1 + 21.0, CMB_JOYSTICKS.2),
        )),
        Some(List::Axis(row)) => Some((
            format!("joystick-rc{}-axis", row + 1),
            (65.0, row_top(row) + 24.0, 70.0),
        )),
        Some(List::Number(slot)) => Some((
            format!("joystick-but{slot}-no"),
            (BUTTONS_LEFT + 47.0, row_top(slot) + 21.0, 70.0),
        )),
        Some(List::Action(slot)) => Some((
            format!("joystick-but{slot}-action"),
            (BUTTONS_LEFT + 297.0, row_top(slot) + 21.0, 100.0),
        )),
        Some(List::Form) | None => None,
    };
    if let (Some(which), Some((id, place))) = (page.open, open) {
        body = body.child(list(&id, sticks, which, place, cx));
    }
    body.into_any_element()
}

/// What is drawn over the whole window: a message box, Import's question, a file dialog, or a
/// settings form.
pub fn overlay(
    sticks: &Sticks,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let page = &sticks.page;
    page.host?;
    if let Some(message) = &page.message {
        let ok = action(
            "joystick-message-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.sticks.message_ok();
                cx.notify();
            }),
        );
        return Some(crate::config::servo_output::modal(
            "joystick-message",
            message.title,
            message.text,
            false,
            vec![ok],
            window,
        ));
    }
    if page.question {
        let answer = |ok: bool| {
            move |this: &mut MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>| {
                this.sticks.question_answer(ok);
                if ok {
                    this.joystick_focus.focus(window, cx);
                }
            }
        };
        let (on_ok, on_cancel) = (answer(true), answer(false));
        let buttons = vec![
            action(
                "joystick-question-ok",
                "OK",
                theme::ACCENT,
                true,
                cx.listener(move |this, _event: &(), window, cx| {
                    on_ok(this, window, cx);
                    cx.notify();
                }),
            ),
            action(
                "joystick-question-cancel",
                "Cancel",
                theme::ACCENT,
                true,
                cx.listener(move |this, _event: &(), window, cx| {
                    on_cancel(this, window, cx);
                    cx.notify();
                }),
            ),
        ];
        return Some(crate::config::servo_output::modal(
            "joystick-question",
            super::IMPORT_TITLE,
            super::IMPORT_WARNING,
            false,
            buttons,
            window,
        ));
    }
    if let Some((_, path)) = &page.path {
        return Some(crate::config::firmware::path_box(
            crate::config::firmware::BoxIds {
                question: "joystick-question",
                yes: "joystick-question-ok",
                no: "joystick-question-cancel",
                message: "joystick-message",
                ok: "joystick-message-ok",
                path: "joystick-path",
                path_value: "joystick-path-value",
                path_ok: "joystick-path-ok",
                path_cancel: "joystick-path-cancel",
            },
            path,
            handle,
            window,
            |this, event| this.sticks.key(event),
            |this, ok| this.sticks.path_done(ok),
            cx,
        ));
    }
    let form = page.form.as_ref()?;
    Some(form_window(sticks, form, handle, window, cx))
}

/// A settings form, `ShowDialog`ed: its caption and close box, and its controls at their
/// Designer places in a client area of its `ClientSize`.
fn form_window(
    sticks: &Sticks,
    form: &forms::Form,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (width, height) = form.kind.client();
    let mut client = div().relative().w(px(width)).h(px(height));
    if let Some(combo_box) = &form.combo {
        let place = combo_box.place;
        client = client.child(combo(
            "joystick-form-combo",
            combo_box.text().to_owned(),
            place,
            |this| this.sticks.toggle_list(List::Form),
            cx,
        ));
        if sticks.page.open == Some(List::Form) {
            client = client.child(list(
                "joystick-form-combo",
                sticks,
                List::Form,
                (place.0, place.1 + place.3, place.2),
                cx,
            ));
        }
    }
    if form.kind == forms::FormKind::MountMode {
        let (x, y) = forms::MOUNT_HELP_AT;
        client = client.child(
            div()
                .absolute()
                .left(px(x))
                .top(px(y))
                .flex()
                .flex_col()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .children(forms::MOUNT_MODE_HELP.lines().map(str::to_owned)),
        );
    }
    for (index, number) in form.numbers.iter().enumerate() {
        let (x, y, w, h) = number.place;
        client = client.child(label(number.label_at.0, number.label_at.1, number.label));
        if let Some(id) = FORM_IDS.get(index).copied() {
            client = client
                .child(text_box(
                    id,
                    &number.text,
                    sticks.page.editing == Some(Editing::Form(index)),
                    handle,
                    (x, y, w - 12.0, h),
                    move |this| this.sticks.form_begin(index),
                    window,
                    cx,
                ))
                .child(arrows(id, (x + w - 12.0, y, 12.0, h), index, cx));
        }
    }
    let dialog = crate::probe::measured("joystick-form", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .flex()
                .items_center()
                .justify_between()
                .px_2()
                .py_1()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(form.kind.title()),
                )
                .child(action(
                    "joystick-form-close",
                    "X",
                    theme::DIM,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.sticks.form_close();
                        cx.notify();
                    }),
                )),
        )
        .child(client);
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("joystick-form-backdrop")
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

/// A `NumericUpDown`'s arrows, `<id>-up` and `<id>-down`.
fn arrows(id: &str, place: Place, index: usize, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let step = Rc::new(move |this: &mut MissionPlanner, up: bool| {
        this.sticks.form_step(index, up);
    });
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let name = format!("{id}-{}", if up { "up" } else { "down" });
        let step = Rc::clone(&step);
        crate::probe::measured(name.clone(), div())
            .id(SharedString::from(name))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(if up { "▲" } else { "▼" })
            .on_click(cx.listener(move |this, _event, _window, cx| {
                step(this, up);
                cx.notify();
            }))
    };
    at(place)
        .flex()
        .flex_col()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::ACTION))
        .child(arrow(true, cx))
        .child(arrow(false, cx))
        .into_any_element()
}
