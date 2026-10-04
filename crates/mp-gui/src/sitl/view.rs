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

//! Drawing the SITL page in the Designer's arrangement: `groupBox1` with the map filling the
//! page's top, `groupBox3` and `groupBox4` in a row under it, and `groupBox2` with the pictures
//! along the bottom - each at the `.resx`'s `Location` and `Size`, the anchors kept: the map
//! stretches, the rest keep to the bottom, `panel1` stays centred.
//! `// C#: GCSViews/SITL.Designer.cs:31-380; GCSViews/SITL.resx`
//!
//! One arrangement differs: `groupBox3` and `groupBox4` are both anchored left and right, so on
//! a page wider than the Designer's the first widens under the second; here `groupBox3` keeps its
//! 140 and `groupBox4` takes the rest.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{
    AnyElement, Context, Div, KeyDownEvent, MouseButton, SharedString, Window, div, prelude::*, px,
    rgb,
};

use super::model::{self, Vehicle};
use super::{Field, Spin};
use crate::MissionPlanner;
use crate::config::optional::{at, input_box};
use crate::config::servo_output::dropdown;
use crate::pictures::Layout;
use crate::ui::theme;

/// `GMarkerGoogle`'s area from its point, which `GMapMarkerWP` is: where `OnMarkerEnter` fires.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/Markers/GMarkerGoogle.cs:111-127`
pub const PIN_AREA: (f32, f32, f32, f32) = (-15.0, -31.0, 32.0, 32.0);

/// Why the three Multilink buttons and Ctrl+D do nothing.
pub const MULTILINK_REASON: &str =
    "Multilink swarms connect one link per vehicle (MainV2.Comports), which is not ported";

/// `$this.Size`'s margins round the groups: `groupBox1` at (13, 3), 854 wide in 879; `groupBox2`
/// ending at 520 in 532. `// C#: GCSViews/SITL.resx ($this.Size, groupBox1.Location, groupBox2.Location)`
const MARGIN: (f32, f32, f32, f32) = (13.0, 3.0, 12.0, 12.0);
/// The gaps between the rows: `groupBox3` at 290 under `groupBox1`'s 284, `groupBox2` at 368
/// under the row's 362.
const GAP: f32 = 6.0;
/// `groupBox3`'s and `groupBox4`'s height. `// C#: GCSViews/SITL.resx (groupBox3.Size)`
const ROW: f32 = 72.0;
/// `groupBox3`'s width, and `groupBox4`'s left from the row's. `// C#: GCSViews/SITL.resx`
const OPTIONS: (f32, f32) = (140.0, 139.0);
/// `groupBox2`'s height. `// C#: GCSViews/SITL.resx (groupBox2.Size)`
const FIRMWARE: f32 = 152.0;
/// `panel1`'s top, width and height; it is anchored to the top alone, so centred.
/// `// C#: GCSViews/SITL.resx (panel1.*)`
const PANEL1: (f32, f32, f32) = (12.0, 716.0, 137.0);
/// The type size of the WinForms default font, 8.25 pt.
const FONT: f32 = 11.0;

/// The page.
pub fn screen(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let sitl = &this.sitl;
    let (left, top, right, bottom) = MARGIN;
    crate::probe::measured("sitl-body", div())
        .id("sitl-body")
        .track_focus(&this.sitl_focus.page)
        .key_context("Sitl")
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            if this.sitl_command_key(event) {
                cx.stop_propagation();
                cx.notify();
            }
        }))
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, _event, window, cx| {
                if this.sitl.typing.is_none() {
                    this.sitl_focus.page.focus(window, cx);
                }
            }),
        )
        .flex()
        .flex_col()
        .flex_1()
        .min_h(px(0.0))
        .min_w(px(0.0))
        .pl(px(left))
        .pt(px(top))
        .pr(px(right))
        .pb(px(bottom))
        .bg(rgb(theme::BG))
        .text_size(px(FONT))
        .child(
            group("sitl-group-home", model::GROUP_HOME)
                .flex_1()
                .min_h(px(0.0))
                .child(map_view(sitl, cx))
                .children(sitl.note.as_ref().map(|note| {
                    crate::probe::measured("sitl-note", div())
                        .absolute()
                        .left(px(3.0))
                        .right(px(3.0))
                        .bottom(px(3.0))
                        .px_2()
                        .py_1()
                        .bg(rgb(theme::PANEL))
                        .border_1()
                        .border_color(rgb(theme::WARN))
                        .text_color(rgb(theme::WARN))
                        .child(note.clone())
                }))
                .children(sitl.saying.as_ref().map(|text| loading(text))),
        )
        .child(
            div()
                .relative()
                .flex_shrink_0()
                .mt(px(GAP))
                .h(px(ROW))
                .child(options(this, window, cx))
                .child(advanced(this, window, cx)),
        )
        .child(
            group("sitl-group-firmware", model::GROUP_FIRMWARE)
                .flex_shrink_0()
                .mt(px(GAP))
                .h(px(FIRMWARE))
                .child(
                    div()
                        .absolute()
                        .left_0()
                        .right_0()
                        .top(px(PANEL1.0))
                        .h(px(PANEL1.2))
                        .flex()
                        .justify_center()
                        .child(pictures(sitl, sitl.running(), cx)),
                ),
        )
        .into_any_element()
}

/// "how many?", over the window, while it asks.
pub fn overlay(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let input = this.sitl.how_many()?;
    Some(input_box(
        "sitl-howmany",
        input,
        &this.sitl_focus.field,
        window,
        |this, event| match this.sitl.how_many_key(event) {
            crate::textfield::KeyOutcome::Submitted => {
                this.sitl_how_many_ok();
                true
            }
            crate::textfield::KeyOutcome::Cancelled => {
                this.sitl.how_many_cancel();
                true
            }
            crate::textfield::KeyOutcome::Changed => true,
            crate::textfield::KeyOutcome::Ignored => false,
        },
        MissionPlanner::sitl_how_many_ok,
        |this| this.sitl.how_many_cancel(),
        cx,
    ))
}

/// A `GroupBox`: its frame, its text on the frame, and its controls at their places inside.
fn group(id: &'static str, caption: &'static str) -> Div {
    crate::probe::measured(id, div())
        .relative()
        .child(
            div()
                .absolute()
                .left_0()
                .right_0()
                .top(px(7.0))
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
                .text_color(rgb(theme::DIM))
                .whitespace_nowrap()
                .child(caption),
        )
}

/// A `Label` at its `Location`.
fn label(x: f32, y: f32, text: &'static str) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .whitespace_nowrap()
        .text_color(rgb(theme::TEXT))
        .child(text)
}

/// `groupBox1`'s map: the marker dragged, the map panned and zoomed.
/// `// C#: GCSViews/SITL.cs:762-810; GCSViews/SITL.resx (myGMAP1.*)`
fn map_view(sitl: &super::Sitl, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let wheel_map = sitl.map();
    crate::probe::measured("sitl-map", div())
        .id("sitl-map")
        .absolute()
        .left(px(3.0))
        .top(px(16.0))
        .right(px(3.0))
        .bottom(px(3.0))
        .overflow_hidden()
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                this.sitl
                    .map_down(f32::from(event.position.x), f32::from(event.position.y));
                cx.notify();
            }),
        )
        .on_mouse_move(
            cx.listener(|this, event: &gpui::MouseMoveEvent, window, _cx| {
                if event.pressed_button == Some(MouseButton::Left) {
                    this.sitl
                        .map_move(f32::from(event.position.x), f32::from(event.position.y));
                    window.refresh();
                }
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, _event, _window, cx| {
                this.sitl.map_up();
                cx.notify();
            }),
        )
        .on_scroll_wheel(move |event, window, _cx| {
            // `MouseWheelZoomType.MousePositionWithoutCenter`: about the pointer.
            let delta = event.delta.pixel_delta(px(20.0));
            wheel_map.borrow_mut().zoom(
                f32::from(event.position.x),
                f32::from(event.position.y),
                f32::from(delta.y) / 20.0,
            );
            window.refresh();
        })
        .child(crate::mapview::map_element(sitl.map()))
        .into_any_element()
}

/// `Common.LoadingBox("Downloading", "Downloading sitl software")`, while the download runs.
/// `// C#: GCSViews/SITL.cs:422-452`
fn loading(text: &str) -> Div {
    div()
        .absolute()
        .top_0()
        .left_0()
        .right_0()
        .bottom_0()
        .flex()
        .items_center()
        .justify_center()
        .child(
            crate::probe::measured("sitl-loading", div())
                .flex()
                .flex_col()
                .gap_1()
                .p_3()
                .bg(rgb(theme::PANEL))
                .border_1()
                .border_color(rgb(theme::ACCENT))
                .rounded_md()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child("Downloading"),
                )
                .child(div().text_color(rgb(theme::TEXT)).child(text.to_owned())),
        )
}

/// `groupBox3`: Heading and the version. `// C#: GCSViews/SITL.Designer.cs:195-217`
fn options(this: &MissionPlanner, window: &Window, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let sitl = &this.sitl;
    let (width, _) = OPTIONS;
    let mut body = group("sitl-group-options", model::GROUP_OPTIONS)
        .absolute()
        .left_0()
        .top_0()
        .w(px(width))
        .h_full()
        .child(label(7.0, 20.0, model::HEADING))
        .child(spin(
            "sitl-heading",
            Field::Heading,
            &sitl.heading,
            this,
            window,
            (60.0, 18.0, 51.0, 20.0),
            cx,
        ));
    // `cmb_version` at (10, 44), 123 by 21.
    let place = (10.0, 44.0, 123.0, 21.0);
    let text = model::VERSIONS
        .get(sitl.version)
        .map_or("", |(name, _)| *name);
    body = body.child(
        at(place.0, place.1, place.2, place.3).child(
            crate::probe::measured("sitl-version", div())
                .id("sitl-version")
                .size_full()
                .flex()
                .items_center()
                .rounded_sm()
                .border_1()
                .border_color(rgb(if sitl.version_open {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(theme::ACTION))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .child(div().flex_1().px_1().whitespace_nowrap().child(text))
                .child(arrow_cell())
                .on_click(cx.listener(|this, _event, _window, cx| {
                    this.sitl.toggle_version();
                    cx.notify();
                })),
        ),
    );
    if sitl.version_open {
        body = body.child(dropdown(
            "sitl-version",
            &sitl.version_combo(),
            (place.0, place.1 + place.3, place.2),
            |this, key| this.sitl.choose_version(usize::try_from(key).unwrap_or(0)),
            |_this, _lines| {},
            cx,
        ));
    }
    body.into_any_element()
}

/// The ▼ of a combo box.
fn arrow_cell() -> Div {
    div()
        .w(px(14.0))
        .h_full()
        .flex()
        .items_center()
        .justify_center()
        .border_l_1()
        .border_color(rgb(theme::BORDER))
        .text_size(px(7.0))
        .child("▼")
}

/// `groupBox4`: Sim Speed, Model, the extra command line, Wipe and the swarm buttons.
/// `// C#: GCSViews/SITL.Designer.cs:219-348`
fn advanced(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let sitl = &this.sitl;
    let (_, left) = OPTIONS;
    let field = &this.sitl_focus.field;
    let typing = |which: Field| sitl.typing == Some(which) && field.is_focused(window);
    let mut body = group("sitl-group-advanced", model::GROUP_ADVANCED)
        .absolute()
        .left(px(left))
        .right_0()
        .top_0()
        .h_full()
        .child(label(6.0, 20.0, model::SIM_SPEED))
        .child(spin(
            "sitl-speed",
            Field::Speed,
            &sitl.speed,
            this,
            window,
            (70.0, 18.0, 51.0, 20.0),
            cx,
        ))
        .child(label(127.0, 20.0, model::MODEL))
        .child(label(296.0, 20.0, model::EXTRA_COMMAND_LINE))
        .child(crate::config::optional::text_box(
            "sitl-cmdline",
            sitl.cmdline.value(),
            Some(field),
            typing(Field::Cmdline),
            true,
            (401.0, 17.0, 230.0, 20.0),
            |this| this.sitl.begin_typing(Field::Cmdline),
            |this, event| this.sitl.key(event),
            cx,
        ))
        .child(wipe(sitl.wipe, cx))
        .child(local_wasm(sitl.try_local_wasm, cx));
    // `cmb_model` at (169, 17), 121 by 21: a text to type into and a list to choose from.
    let place = (169.0, 17.0, 121.0, 21.0);
    let model_typing = typing(Field::Model);
    let text_part = crate::probe::measured("sitl-model", div())
        .id("sitl-model")
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .whitespace_nowrap()
        .cursor_text()
        .child(sitl.model.value().to_owned())
        .children(model_typing.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text_part = if model_typing {
        text_part
            .track_focus(field)
            .key_context("TextField")
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                if this.sitl.key(event) {
                    cx.notify();
                }
            }))
    } else {
        let handle = field.clone();
        text_part.on_click(cx.listener(move |this, _event, window, cx| {
            this.sitl.begin_typing(Field::Model);
            handle.focus(window, cx);
            cx.notify();
        }))
    };
    body = body.child(
        at(place.0, place.1, place.2, place.3).child(
            div()
                .size_full()
                .flex()
                .rounded_sm()
                .border_1()
                .border_color(rgb(if model_typing || sitl.model_open {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(theme::ACTION))
                .text_color(rgb(theme::TEXT))
                .child(text_part)
                .child(
                    crate::probe::measured("sitl-model-arrow", arrow_cell())
                        .id("sitl-model-arrow")
                        .cursor_pointer()
                        .on_click(cx.listener(|this, _event, _window, cx| {
                            this.sitl.toggle_model();
                            cx.notify();
                        })),
                ),
        ),
    );
    if sitl.model_open {
        body = body.child(dropdown(
            "sitl-model",
            &sitl.model_combo(),
            (place.0, place.1 + place.3, place.2),
            |this, key| this.sitl.choose_model(usize::try_from(key).unwrap_or(0)),
            |this, lines| this.sitl.scroll_models(lines),
            cx,
        ));
    }
    let busy = sitl.running();
    let handle = field.clone();
    body.child(swarm_button(
        "sitl-swarm-seq",
        model::SWARM_SEQ,
        9.0,
        !busy,
        cx.listener(move |this, _event, window, cx| {
            this.sitl_ask_how_many();
            handle.focus(window, cx);
            cx.notify();
        }),
    ))
    .child(swarm_button(
        "sitl-swarm-link",
        model::SWARM_LINK,
        104.0,
        false,
        cx.listener(|_, _, _, _| {}),
    ))
    .child(swarm_button(
        "sitl-swarm-plane",
        model::SWARM_PLANE,
        199.0,
        false,
        cx.listener(|_, _, _, _| {}),
    ))
    .child(swarm_button(
        "sitl-swarm-rover",
        model::SWARM_ROVER,
        294.0,
        false,
        cx.listener(|_, _, _, _| {}),
    ))
    .into_any_element()
}

/// A `MyButton` of the swarm row, 89 by 23 at y 44; dimmed and inert while disabled.
/// `// C#: GCSViews/SITL.resx (but_swarm*.Location, but_swarm*.Size)`
fn swarm_button(
    id: &'static str,
    text: &'static str,
    x: f32,
    enabled: bool,
    on_click: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let base = crate::probe::measured(id, div())
        .id(id)
        .size_full()
        .flex()
        .items_center()
        .justify_center()
        .overflow_hidden()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_size(px(9.0))
        .line_height(px(10.0))
        .text_center()
        .child(text);
    let base = if enabled {
        base.bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(on_click)
    } else {
        base.bg(rgb(theme::PANEL)).text_color(rgb(theme::DIM))
    };
    at(x, 44.0, 89.0, 23.0).child(base).into_any_element()
}

/// `chk_wipe` at (638, 17). `// C#: GCSViews/SITL.resx (chk_wipe.*)`
fn wipe(checked: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    div()
        .absolute()
        .left(px(638.0))
        .top(px(17.0))
        .child(
            crate::probe::measured("sitl-wipe", div())
                .id("sitl-wipe")
                .flex()
                .items_center()
                .gap_1()
                .cursor_pointer()
                .text_color(rgb(theme::TEXT))
                .child(
                    div()
                        .size(px(12.0))
                        .flex()
                        .items_center()
                        .justify_center()
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .bg(rgb(theme::ACTION))
                        .children(checked.then(|| div().size(px(6.0)).bg(rgb(theme::ACCENT)))),
                )
                .child(model::WIPE)
                .on_click(cx.listener(|this, _event, _window, cx| {
                    this.sitl.toggle_wipe();
                    cx.notify();
                })),
        )
        .into_any_element()
}

/// The owner's "try local wasm" box (2026-10-04), beside Wipe: the four pictures start the
/// WebAssembly builds in `tools/sitl/wasm` under Node. Not in the C#.
fn local_wasm(checked: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    div()
        .absolute()
        .left(px(700.0))
        .top(px(17.0))
        .child(
            crate::probe::measured("sitl-local-wasm", div())
                .id("sitl-local-wasm")
                .flex()
                .items_center()
                .gap_1()
                .cursor_pointer()
                .text_color(rgb(theme::TEXT))
                .child(
                    div()
                        .size(px(12.0))
                        .flex()
                        .items_center()
                        .justify_center()
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .bg(rgb(theme::ACTION))
                        .children(checked.then(|| div().size(px(6.0)).bg(rgb(theme::ACCENT)))),
                )
                .child("try local wasm")
                .on_click(cx.listener(|this, _event, _window, cx| {
                    this.sitl.toggle_local_wasm();
                    cx.notify();
                })),
        )
        .into_any_element()
}

/// A `NumericUpDown` at its place: the number, typed into while focused, and its arrows,
/// `<id>-up` and `<id>-down`.
fn spin(
    id: &'static str,
    which: Field,
    value: &Spin,
    this: &MissionPlanner,
    window: &Window,
    (x, y, width, height): (f32, f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let handle = &this.sitl_focus.field;
    let focused = this.sitl.typing == Some(which) && handle.is_focused(window);
    let text = crate::probe::measured(id, div())
        .id(id)
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .whitespace_nowrap()
        .cursor_text()
        .text_color(rgb(theme::TEXT))
        .child(value.shown())
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))));
    let text = if focused {
        text.track_focus(handle)
            .key_context("TextField")
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                if this.sitl.key(event) {
                    cx.notify();
                }
            }))
    } else {
        let handle = handle.clone();
        text.on_click(cx.listener(move |this, _event, window, cx| {
            this.sitl.begin_typing(which);
            handle.focus(window, cx);
            cx.notify();
        }))
    };
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let name = format!("{id}-{}", if up { "up" } else { "down" });
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
                this.sitl.step(which, up);
                cx.notify();
            }))
    };
    at(x, y, width, height)
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
                .w(px(12.0))
                .h_full()
                .flex()
                .flex_col()
                .border_l_1()
                .border_color(rgb(theme::BORDER))
                .child(arrow(true, cx))
                .child(arrow(false, cx)),
        )
        .into_any_element()
}

/// How a vehicle's bitmap sits in its box: `SizeMode`, which the `.resx` leaves at `Normal` - at
/// the top left, unscaled, clipped. The bitmaps are 132 high in boxes 112 high, so their bottom
/// 20 rows are cut off, and the plane's 139 columns and the rover's and quad's are cut at the
/// right, as in the C#.
/// `// C#: GCSViews/SITL.resx (pictureBox*.Size; no pictureBox*.SizeMode)`
pub const PICTURE_LAYOUT: Layout = Layout::None;

/// `panel1`: the four pictures, a click on which starts that vehicle, and their labels. Each
/// shows its `ImageNormal`, and its `ImageOver` while the pointer is on it
/// (`PictureBoxMouseOver`); a picture not carried is the named box drawn before. Inert while a
/// start is on its way.
/// `// C#: GCSViews/SITL.Designer.cs:104-179; ExtLibs/Controls/PictureBoxMouseOver.cs`
fn pictures(sitl: &super::Sitl, busy: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (_, width, height) = PANEL1;
    let mut panel = div().relative().w(px(width)).h(px(height));
    for vehicle in Vehicle::ALL {
        let (x, y, w, h) = vehicle.picture();
        let body = crate::probe::measured(vehicle.id(), div())
            .id(vehicle.id())
            .size_full()
            .on_hover(cx.listener(move |this, hovered: &bool, _window, cx| {
                this.sitl.hover(vehicle, *hovered);
                cx.notify();
            }));
        let picture = match crate::pictures::image(sitl.picture(vehicle), PICTURE_LAYOUT) {
            Some(image) => body.child(image),
            None => body
                .flex()
                .items_center()
                .justify_center()
                .rounded_md()
                .border_1()
                .border_color(rgb(if sitl.picture(vehicle) == vehicle.image(true) {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(if busy { theme::PANEL } else { theme::ACTION }))
                .text_lg()
                .text_color(rgb(if busy { theme::DIM } else { theme::TEXT }))
                .child(vehicle.label()),
        };
        let picture = if busy {
            picture
        } else {
            picture
                .cursor_pointer()
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.sitl_click_picture(vehicle);
                    cx.notify();
                }))
        };
        let (label_x, label_y) = vehicle.label_at();
        panel = panel.child(at(x, y, w, h).child(picture)).child(label(
            label_x,
            label_y,
            vehicle.label(),
        ));
    }
    panel.into_any_element()
}
