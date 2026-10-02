//! The Onboard OSD page drawn: `ConfigOSD.Designer.cs`'s page - the OSD control filling it and
//! the five buttons down its right - and `ExtLibs/OSDConfigurator/GUI`'s controls: the tab strip
//! (`OSDUserControl.Designer.cs`), each screen's canvas, item list and three groups
//! (`ScreenControl.Designer.cs`), a row a setting (`OptionControls/*.Designer.cs`,
//! `ItemControls/CommonItemControl.Designer.cs`), and the OSD5/6 slots dialog
//! (`Osd56ItemsSetup/*.Designer.cs`); the questions and the progress dialogs over them.
//!
//! The arrangement is the Designers': the page 1147 x 823 with 8 of padding, the OSD control
//! docked to fill and the button panel 182 wide docked right; a screen's tab in two columns, the
//! canvas and the item list left, the Editor Options, Screen Options and Item Options groups in
//! a 350-wide column right; every control at its `Location`. What the C# sizes by docking and
//! percentages is sized here from the same numbers.
// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::rc::Rc;
use std::sync::Arc;
use std::time::Instant;

use gpui::{
    AnyElement, Bounds, Context, Corners, FocusHandle, KeyDownEvent, MouseButton, MouseDownEvent,
    MouseMoveEvent, MouseUpEvent, Pixels, RenderImage, SharedString, Window, canvas, div, point,
    prelude::*, px, quad, rgb, size,
};

use super::adsb::{REFRESH_WARNING, SHOW_AGAIN_KEY, SHOW_ME_AGAIN};
use super::onboard_osd::{
    ARE_YOU_SURE, AUTO_WRITE, CLEAR_ALL, COPY_LAYOUT, Control, DECREASE, DISCARD, DJI_COLS,
    DJI_ROWS, Drawn, EDITOR_OPTIONS, Font, Geometry, HD_LAYOUT, ITEM_OPTIONS, NTSC_ROWS,
    OnboardOsd, PASTE_LAYOUT, Question, REFRESH, REFRESH_PARAMS, RESET_CHANGES, SCREEN_OPTIONS,
    SETTINGS_TAB, SHOW_NAMES, SLOTS_CONFIG, TITLE, Tab, TextTarget, WRITE, WRITING, render, rows,
    value_text,
};
use super::onboard_osd_slots::{Box_, ComboKind, DIALOG_TITLE, DONE_REFRESH, Dialog};
use super::optional::{at, button, group, label};
use super::serial_ports::{Bar, ProgressIds, progress_dialog};
use super::servo_output::{
    Check, CheckState, Combo, NumberHandlers, check_box, combo_box, dropdown, modal, number_box,
};
use crate::MissionPlanner;
use crate::ui::{action, panel, theme};

/// The page, `ConfigOSD.Size`. `// C#: ConfigOSD.Designer.cs:122`
pub const PAGE_SIZE: (f32, f32) = (1147.0, 823.0);
/// `Padding = 8`. `// C#: ConfigOSD.Designer.cs:121`
const PADDING: f32 = 8.0;
/// `panel1.Size.Width`. `// C#: ConfigOSD.Designer.cs:60`
const BUTTONS_WIDTH: f32 = 182.0;
/// `tabControl.ItemSize.Height`. `// C#: GUI/OSDUserControl.Designer.cs:74`
const TAB_HEIGHT: f32 = 35.0;
/// `tableRight.Size.Width`. `// C#: GUI/ScreenControl.Designer.cs:286`
const RIGHT_WIDTH: f32 = 350.0;
/// `grEditorOptions.Size`. `// C#: GUI/ScreenControl.Designer.cs:304`
const EDITOR_SIZE: (f32, f32) = (344.0, 114.0);
/// A `CommonItemControl`. `// C#: GUI/ItemControls/CommonItemControl.Designer.cs:61`
const ITEM_SIZE: (f32, f32) = (175.0, 35.0);
/// The `TableLayoutPanel` margin round each control.
const MARGIN: f32 = 3.0;

/// The page's state, from the window.
fn osd(this: &mut MissionPlanner) -> &mut OnboardOsd {
    &mut this.software_pages2.onboard_osd
}

/// The page, laid out as `ConfigOSD.Designer.cs` lays it out.
pub fn page(
    page: &OnboardOsd,
    number_focus: &FocusHandle,
    text_focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !page.is_active() {
        return div().into_any_element();
    }
    let enabled = !page.refreshing();
    let (width, height) = PAGE_SIZE;
    let control_width = width - 2.0 * PADDING - BUTTONS_WIDTH;
    let control_height = height - 2.0 * PADDING;
    let body = div()
        .relative()
        .w(px(width))
        .h(px(height))
        // `osdUserControl`, docked to fill.
        .child(
            at(PADDING, PADDING, control_width, control_height)
                .child(tab_strip(page, cx))
                .child(match page.tab {
                    Tab::Settings => settings_tab(
                        page,
                        number_focus,
                        text_focus,
                        enabled,
                        (
                            0.0,
                            TAB_HEIGHT + 4.0,
                            control_width,
                            control_height - TAB_HEIGHT - 4.0,
                        ),
                        window,
                        cx,
                    ),
                    Tab::Screen(screen) => screen_tab(
                        page,
                        screen,
                        number_focus,
                        text_focus,
                        enabled,
                        (
                            0.0,
                            TAB_HEIGHT + 4.0,
                            control_width,
                            control_height - TAB_HEIGHT - 4.0,
                        ),
                        window,
                        cx,
                    ),
                }),
        )
        // `panel1`, docked right.
        .child(buttons(
            page,
            enabled,
            (PADDING + control_width, PADDING),
            cx,
        ));
    panel(TITLE, body).into_any_element()
}

/// `panel1`'s five controls at their places.
/// `// C#: ConfigOSD.Designer.cs:63-108`
fn buttons(
    page: &OnboardOsd,
    enabled: bool,
    (x, y): (f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut auto = Check::default();
    auto.enabled = enabled;
    auto.state = if page.auto_write {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    at(x, y, BUTTONS_WIDTH, PAGE_SIZE.1 - 2.0 * PADDING)
        .child(button(
            "osd-write",
            WRITE,
            (20.0, 23.0, 145.0, 32.0),
            enabled,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                let connected = view.connected && view.vehicle.is_some();
                let telemetry = &this.telemetry;
                this.software_pages2
                    .onboard_osd
                    .write(telemetry, connected, false);
            },
            cx,
        ))
        .child(check_box(
            "osd-auto-write".to_owned(),
            &auto,
            AUTO_WRITE,
            (24.0, 63.0),
            |this| {
                let page = osd(this);
                page.auto_write = !page.auto_write;
            },
            cx,
        ))
        .child(button(
            "osd-slots",
            SLOTS_CONFIG,
            (20.0, 112.0, 145.0, 42.0),
            enabled,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                let telemetry = &this.telemetry;
                let words = this.software_pages2.onboard_osd.slots.click_config(
                    telemetry,
                    &view,
                    Instant::now(),
                );
                if words.is_some() {
                    this.file_status = words;
                }
            },
            cx,
        ))
        .child(button(
            "osd-refresh",
            REFRESH,
            (20.0, 197.0, 145.0, 32.0),
            enabled,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                let shown_again = this.persisted.get(SHOW_AGAIN_KEY).map(str::to_owned);
                let telemetry = &this.telemetry;
                this.software_pages2.onboard_osd.click_refresh(
                    telemetry,
                    &view,
                    shown_again.as_deref(),
                );
            },
            cx,
        ))
        .child(button(
            "osd-discard",
            DISCARD,
            (20.0, 235.0, 145.0, 32.0),
            enabled,
            |this, _window, _cx| osd(this).click_discard(),
            cx,
        ))
        .into_any_element()
}

/// `tabControl`'s tabs: Settings, then a screen each.
/// `// C#: GUI/OSDUserControl.Designer.cs:70-88; GUI/OSDUserControl.cs:34-41`
fn tab_strip(page: &OnboardOsd, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut strip = at(0.0, 0.0, PAGE_SIZE.0, TAB_HEIGHT).flex().gap_1();
    let mut tabs: Vec<(String, String, Tab)> = vec![(
        "osd-tab-settings".to_owned(),
        SETTINGS_TAB.to_owned(),
        Tab::Settings,
    )];
    for (index, screen) in page.config.screens.iter().enumerate() {
        tabs.push((
            format!("osd-tab-screen-{}", screen.number),
            screen.name.clone(),
            Tab::Screen(index),
        ));
    }
    for (id, text, tab) in tabs {
        let selected = page.tab == tab;
        strip = strip.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .min_w(px(70.0))
                .h(px(TAB_HEIGHT))
                .px_3()
                .flex()
                .items_center()
                .justify_center()
                .rounded_t_md()
                .text_sm()
                .cursor_pointer()
                .bg(rgb(if selected { theme::PANEL } else { theme::BG }))
                .text_color(rgb(if selected { theme::ACCENT } else { theme::DIM }))
                .border_b_2()
                .border_color(rgb(if selected { theme::ACCENT } else { theme::BG }))
                .hover(|style| style.text_color(rgb(theme::TEXT)))
                .child(text)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    osd(this).choose_tab(tab);
                    cx.notify();
                })),
        );
    }
    strip.into_any_element()
}

/// `tabSettings`: the `OSD_` options' rows, docked top over each other, scrolling.
/// `// C#: GUI/OSDUserControl.cs:22-29; GUI/OSDUserControl.Designer.cs:81-88`
#[allow(clippy::too_many_arguments)]
fn settings_tab(
    page: &OnboardOsd,
    number_focus: &FocusHandle,
    text_focus: &FocusHandle,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let list = rows(&page.settings, &page.config.options);
    let scroll = &page.scrolls[0];
    let (column, lifted) = option_rows(
        page,
        &list,
        number_focus,
        text_focus,
        enabled,
        width - 20.0,
        (PADDING + x, PADDING + y),
        scroll,
        window,
        cx,
    );
    at(x, y, width, height)
        .child(
            crate::probe::measured("osd-settings", div())
                .id("osd-settings")
                .size_full()
                .overflow_y_scroll()
                .track_scroll(scroll)
                .child(column),
        )
        .children(lifted)
        .into_any_element()
}

/// A screen's tab: `ScreenControl`, the canvas and the item list left, the three groups right.
/// `// C#: GUI/ScreenControl.Designer.cs`
#[allow(clippy::too_many_arguments)]
fn screen_tab(
    page: &OnboardOsd,
    screen: usize,
    number_focus: &FocusHandle,
    text_focus: &FocusHandle,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(state) = page.screens.get(screen) else {
        return div().into_any_element();
    };
    let Some(scr) = page.config.screens.get(screen) else {
        return div().into_any_element();
    };
    let left_width = width - RIGHT_WIDTH - 2.0 * MARGIN;
    let geometry = state.geometry;
    let (canvas_w, canvas_h) = geometry.canvas_size();
    #[allow(clippy::cast_precision_loss)]
    let (canvas_w, canvas_h) = (canvas_w as f32, canvas_h as f32);
    // `tableLeft`: the canvas anchored top, centred when it fits; the item list under it.
    let canvas_x = ((left_width - canvas_w) / 2.0).max(0.0);
    let canvas_top = 4.0;
    let list_top = canvas_top + canvas_h + 6.0;
    let left = crate::probe::measured("osd-left", div())
        .id("osd-left")
        .absolute()
        .left(px(MARGIN))
        .top(px(MARGIN))
        .w(px(left_width))
        .h(px(height - 2.0 * MARGIN))
        .overflow_scroll()
        .child(
            div()
                .relative()
                .w(px(left_width.max(canvas_w)))
                .h(px(list_top + items_height(scr.items.len(), left_width)))
                .child(
                    div()
                        .absolute()
                        .left(px(canvas_x))
                        .top(px(canvas_top))
                        .child(canvas_element(page, screen, cx)),
                )
                .child(item_list(page, scr, (0.0, list_top, left_width), cx)),
        );
    // `tableRight`: Editor Options at its size, then Screen Options and Item Options halving
    // the rest.
    let right_x = MARGIN + left_width + MARGIN;
    let groups_top = MARGIN + EDITOR_SIZE.1 + 2.0 * MARGIN;
    let group_height = ((height - groups_top - MARGIN) / 2.0 - MARGIN).max(60.0);
    let screen_rows = rows(&page.settings, &scr.options);
    let selected = state.selected.and_then(|index| scr.items.get(index));
    let item_rows = selected.map_or_else(Vec::new, |item| rows(&page.settings, &item.options));
    // The rows' columns' origins in the page, for the dropped list that is lifted out of them.
    let screen_origin = (PADDING + x + right_x, PADDING + y + groups_top + 16.0);
    let item_origin = (
        PADDING + x + right_x,
        PADDING + y + groups_top + group_height + 2.0 * MARGIN + 16.0,
    );
    let (screen_column, screen_lifted) = option_rows(
        page,
        &screen_rows,
        number_focus,
        text_focus,
        enabled,
        EDITOR_SIZE.0 - 4.0,
        screen_origin,
        &page.scrolls[1],
        window,
        cx,
    );
    let (item_column, item_lifted) = option_rows(
        page,
        &item_rows,
        number_focus,
        text_focus,
        enabled,
        EDITOR_SIZE.0 - 4.0,
        item_origin,
        &page.scrolls[2],
        window,
        cx,
    );
    // `groupOptions.Text = selectedItem.Name`.
    let item_title: SharedString = selected.map_or_else(
        || SharedString::from(ITEM_OPTIONS),
        |item| SharedString::from(item.name.clone()),
    );
    at(x, y, width, height)
        .child(left)
        .child(editor_group(page, screen, enabled, (right_x, MARGIN), cx))
        .child(
            group(
                (right_x, groups_top, EDITOR_SIZE.0, group_height),
                SCREEN_OPTIONS,
                enabled,
            )
            .child(
                crate::probe::measured("osd-screen-options", div())
                    .id("osd-screen-options")
                    .absolute()
                    .left(px(0.0))
                    .top(px(16.0))
                    .w(px(EDITOR_SIZE.0 - 2.0))
                    .h(px(group_height - 18.0))
                    .overflow_y_scroll()
                    .track_scroll(&page.scrolls[1])
                    .child(screen_column),
            ),
        )
        .child(
            group_titled(
                (
                    right_x,
                    groups_top + group_height + 2.0 * MARGIN,
                    EDITOR_SIZE.0,
                    group_height,
                ),
                item_title,
                enabled,
            )
            .child(
                crate::probe::measured("osd-item-options", div())
                    .id("osd-item-options")
                    .absolute()
                    .left(px(0.0))
                    .top(px(16.0))
                    .w(px(EDITOR_SIZE.0 - 2.0))
                    .h(px(group_height - 18.0))
                    .overflow_y_scroll()
                    .track_scroll(&page.scrolls[2])
                    .child(item_column),
            ),
        )
        .children(screen_lifted)
        .children(item_lifted)
        .into_any_element()
}

/// How tall the item list is: `FlowLayoutPanel`'s wrap of 175 x 35 controls.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn items_height(count: usize, width: f32) -> f32 {
    let per_row = ((width / ITEM_SIZE.0).floor() as usize).max(1);
    let lines = count.div_ceil(per_row) as f32;
    lines * ITEM_SIZE.1 + 8.0
}

/// [`group`] with a caption made for the frame: the Item Options group, titled with the
/// selected item's name.
fn group_titled(
    (x, y, width, height): (f32, f32, f32, f32),
    caption: SharedString,
    enabled: bool,
) -> gpui::Div {
    at(x, y, width, height)
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .child(
            div()
                .absolute()
                .left(px(6.0))
                .top(px(-8.0))
                .px_1()
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
                .child(caption),
        )
}

/// `grEditorOptions`: Decrease, Copy Layout, Paste Layout, Show Names, Clear All, HD Layout.
/// `// C#: GUI/ScreenControl.Designer.cs:295-371`
fn editor_group(
    page: &OnboardOsd,
    screen: usize,
    enabled: bool,
    (x, y): (f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(state) = page.screens.get(screen) else {
        return div().into_any_element();
    };
    // `grEditorOptions.Enabled = false` for a screen with no items.
    let enabled = enabled
        && page
            .config
            .screens
            .get(screen)
            .is_some_and(|s| !s.items.is_empty());
    let check = |on: bool| {
        let mut check = Check::default();
        check.enabled = enabled;
        check.state = if on {
            CheckState::Checked
        } else {
            CheckState::Unchecked
        };
        check
    };
    group(
        (x, y, EDITOR_SIZE.0, EDITOR_SIZE.1),
        EDITOR_OPTIONS,
        enabled,
    )
    .child(check_box(
        "osd-decrease".to_owned(),
        &check(state.geometry.reduced),
        DECREASE,
        (23.0, 28.0),
        move |this| {
            let page = osd(this);
            let reduced = page.screens.get(screen).is_some_and(|s| s.geometry.reduced);
            page.set_reduced(screen, !reduced);
        },
        cx,
    ))
    .child(button(
        "osd-copy",
        COPY_LAYOUT,
        (127.0, 19.0, 93.0, 33.0),
        enabled,
        move |this, _window, _cx| osd(this).copy_layout(screen),
        cx,
    ))
    .child(button(
        "osd-paste",
        PASTE_LAYOUT,
        (226.0, 19.0, 93.0, 33.0),
        enabled,
        move |this, _window, _cx| osd(this).paste_layout(screen),
        cx,
    ))
    .child(check_box(
        "osd-names".to_owned(),
        &check(page.caption_mode == super::onboard_osd::CaptionMode::Names),
        SHOW_NAMES,
        (23.0, 58.0),
        |this| {
            let page = osd(this);
            let names = page.caption_mode == super::onboard_osd::CaptionMode::Names;
            page.set_show_names(!names);
        },
        cx,
    ))
    .child(button(
        "osd-clear-all",
        CLEAR_ALL,
        (127.0, 58.0, 93.0, 33.0),
        enabled,
        move |this, _window, _cx| osd(this).clear_all(screen),
        cx,
    ))
    .child(check_box(
        "osd-hd".to_owned(),
        &check(state.geometry.high_def),
        HD_LAYOUT,
        (23.0, 88.0),
        move |this| {
            let page = osd(this);
            let hd = page
                .screens
                .get(screen)
                .is_some_and(|s| s.geometry.high_def);
            page.set_high_def(screen, !hd);
        },
        cx,
    ))
    .into_any_element()
}

/// `panelItemList`: a `CommonItemControl` an item - its check box and its name - flowing
/// left to right and wrapping, the last item first as `OnLoad` reverses them before adding; a
/// click on one selects it. The items come in the parameter table's order, which is by name
/// here where the C#'s `MAV.param` keeps arrival order (a divergence of order only).
/// `// C#: GUI/ScreenControl.cs:180-208; GUI/ItemControls/CommonItemControl.cs`
fn item_list(
    page: &OnboardOsd,
    screen: &super::onboard_osd::Screen,
    (x, y, width): (f32, f32, f32),
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let screen_index = page
        .config
        .screens
        .iter()
        .position(|s| s == screen)
        .unwrap_or(0);
    let mut flow = crate::probe::measured("osd-items", div())
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .flex()
        .flex_wrap();
    for (index, item) in screen.items.iter().enumerate().rev() {
        let enabled_index = item.enabled(&page.settings);
        let mut check = Check::default();
        check.enabled = true;
        check.state = if item.is_enabled(&page.settings) {
            CheckState::Checked
        } else {
            CheckState::Unchecked
        };
        let row_id = format!("osd-item-{}", item.name);
        let name = SharedString::from(item.name.clone());
        flow = flow.child(
            crate::probe::measured(row_id.clone(), div())
                .id(SharedString::from(row_id))
                .relative()
                .w(px(ITEM_SIZE.0))
                .h(px(ITEM_SIZE.1))
                .cursor_pointer()
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    osd(this).select(screen_index, Some(index));
                    cx.notify();
                }))
                // The box's click is its own: a WinForms child control's click never reaches the
                // parent's MouseClick, so it is kept from the row's selecting click here.
                .child(
                    div()
                        .absolute()
                        .left(px(15.0))
                        .top(px(11.0))
                        .w(px(15.0))
                        .h(px(14.0))
                        .on_mouse_down(
                            MouseButton::Left,
                            cx.listener(|_this, _event: &MouseDownEvent, _window, cx| {
                                cx.stop_propagation();
                            }),
                        )
                        .child(check_box(
                            format!("osd-item-{}-check", item.name),
                            &check,
                            "",
                            (0.0, 0.0),
                            move |this| {
                                if let Some(enabled_index) = enabled_index {
                                    osd(this).toggle(enabled_index);
                                }
                            },
                            cx,
                        )),
                )
                .child(
                    div()
                        .absolute()
                        .left(px(36.0))
                        .top(px(12.0))
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(name),
                ),
        );
    }
    flow.into_any_element()
}

/// The raster the canvas shows, remade when what it draws changes.
fn prepare_raster(page: &OnboardOsd, screen: usize, geometry: Geometry) -> Vec<Drawn> {
    let drawn = page.drawn(screen);
    let mut raster = page.raster.borrow_mut();
    let key = (geometry, drawn.clone());
    if raster.key.as_ref() != Some(&key) {
        raster.pending = Some(render(Font::get(), geometry, &drawn));
        raster.key = Some(key);
    }
    drawn
}

/// `LayoutControl`: the canvas, painted from the raster, its NTSC/PAL or HD notes over it, the
/// selected item's yellow rectangle, and the mouse that selects and drags.
/// `// C#: GUI/LayoutControl.cs; GUI/Visualizer.cs:61-68, 103-117`
fn canvas_element(
    page: &OnboardOsd,
    screen: usize,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(state) = page.screens.get(screen) else {
        return div().into_any_element();
    };
    let geometry = state.geometry;
    let _ = prepare_raster(page, screen, geometry);
    let (w, h) = geometry.canvas_size();
    #[allow(clippy::cast_precision_loss)]
    let (wf, hf) = (w as f32, h as f32);
    let (cw, ch) = geometry.char_size();
    #[allow(clippy::cast_precision_loss)]
    let (cwf, chf) = (cw as f32, ch as f32);
    let raster_for_prepaint = Rc::clone(&page.raster);
    let raster_for_paint = Rc::clone(&page.raster);
    let selection = page.selection_rectangle(screen);
    let painted = canvas(
        move |bounds: Bounds<Pixels>, _window: &mut Window, _cx: &mut gpui::App| {
            raster_for_prepaint.borrow_mut().origin =
                (f32::from(bounds.origin.x), f32::from(bounds.origin.y));
        },
        move |bounds: Bounds<Pixels>, (), window: &mut Window, cx: &mut gpui::App| {
            let mut raster = raster_for_paint.borrow_mut();
            if let Some(image) = raster.pending.take() {
                let image = Arc::new(RenderImage::new(vec![image::Frame::new(image)]));
                if let Some(old) = raster.shown.replace(image) {
                    cx.drop_image(old, Some(window));
                }
            }
            if let Some(image) = raster.shown.as_ref() {
                let _ = window.paint_image(
                    bounds,
                    bounds,
                    Corners::default(),
                    Arc::clone(image),
                    0,
                    false,
                );
            }
            // `DrawSelection`: `Pens.Yellow`, the rectangle five pixels out.
            if let Some((x, y, rw, rh)) = selection {
                let yellow = gpui::Hsla::from(rgb(0xff_ff_00));
                #[allow(clippy::cast_precision_loss)]
                let (x, y, rw, rh) = (x as f32, y as f32, rw as f32, rh as f32);
                let edges = [
                    (x, y, rw, 1.0),
                    (x, y + rh - 1.0, rw, 1.0),
                    (x, y, 1.0, rh),
                    (x + rw - 1.0, y, 1.0, rh),
                ];
                for (ex, ey, ew, eh) in edges {
                    let rect = Bounds {
                        origin: point(bounds.origin.x + px(ex), bounds.origin.y + px(ey)),
                        size: size(px(ew), px(eh)),
                    };
                    window.paint_quad(quad(
                        rect,
                        Corners::default(),
                        yellow,
                        gpui::Edges::default(),
                        yellow,
                        gpui::BorderStyle::default(),
                    ));
                }
            }
        },
    )
    .size_full();
    // The notes: "NTSC" above and "PAL" below the line at row 13 (Arial 6), or "50x18" and
    // "60x22" at the corners the HD lines mark (Arial 8).
    let note = |text: &'static str, x: f32, y: f32, size: f32| {
        div()
            .absolute()
            .left(px(x))
            .top(px(y))
            .text_size(px(size))
            .text_color(rgb(0x00_00_00))
            .child(text)
    };
    #[allow(clippy::cast_precision_loss)]
    let notes: Vec<AnyElement> = if geometry.high_def {
        vec![
            note(
                "50x18",
                cwf * DJI_COLS as f32 - 4.5 * 8.0,
                chf * DJI_ROWS as f32 - 8.0 * 2.0,
                8.0,
            )
            .into_any_element(),
            note("60x22", wf - 4.5 * 8.0, hf - 8.0 * 2.0, 8.0).into_any_element(),
        ]
    } else {
        vec![
            note("NTSC", 6.0, chf * NTSC_ROWS as f32 - 6.0 * 2.0, 6.0).into_any_element(),
            note("PAL", 6.0, chf * NTSC_ROWS as f32 + 6.0 * 1.5, 6.0).into_any_element(),
        ]
    };
    let local = |this: &MissionPlanner, position: gpui::Point<Pixels>| -> (i32, i32) {
        let (ox, oy) = this.software_pages2.onboard_osd.raster.borrow().origin;
        #[allow(clippy::cast_possible_truncation)]
        (
            (f32::from(position.x) - ox).round() as i32,
            (f32::from(position.y) - oy).round() as i32,
        )
    };
    let local_down = local;
    let local_move = local;
    crate::probe::measured("osd-layout", div())
        .id("osd-layout")
        .relative()
        .w(px(wf))
        .h(px(hf))
        .cursor_pointer()
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(move |this, event: &MouseDownEvent, _window, cx| {
                let at = local_down(this, event.position);
                if osd(this).press(screen, at) {
                    cx.notify();
                }
            }),
        )
        .on_mouse_move(
            cx.listener(move |this, event: &MouseMoveEvent, _window, cx| {
                if event.pressed_button != Some(MouseButton::Left) {
                    return;
                }
                let at = local_move(this, event.position);
                if osd(this).drag(screen, at) {
                    cx.notify();
                }
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, _event: &MouseUpEvent, _window, _cx| osd(this).release()),
        )
        // A child the probe can measure: a canvas has no bounds of its own to record.
        .child(
            div()
                .absolute()
                .left(px(0.0))
                .top(px(0.0))
                .w(px(wf))
                .h(px(hf)),
        )
        .child(painted)
        .children(notes)
        .into_any_element()
}

/// The rows of settings, each docked under the last: `OptionControlFactory.Create`'s controls.
/// The column, and the dropped list of a dropdown row if one is open - drawn at the page's level,
/// at the row's place less the column's scroll, since a deferred element inside a scrolling
/// column is not laid out.
#[allow(clippy::too_many_arguments)]
fn option_rows(
    page: &OnboardOsd,
    list: &[(usize, Control)],
    number_focus: &FocusHandle,
    text_focus: &FocusHandle,
    enabled: bool,
    width: f32,
    origin: (f32, f32),
    scroll: &gpui::ScrollHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> (AnyElement, Option<AnyElement>) {
    let mut column = div().relative().w(px(width));
    let mut y = 0.0;
    let mut lifted = None;
    for &(index, control) in list {
        column = column.child(option_row(
            page,
            index,
            control,
            number_focus,
            text_focus,
            enabled,
            (0.0, y, width),
            window,
            cx,
        ));
        if let Control::Dropdown(choices) = control
            && page.open_combo == Some(index)
        {
            let mut combo = page.combo(index, choices);
            combo.enabled = enabled;
            let id = page
                .settings
                .get(index)
                .map_or_else(String::new, |s| format!("osd-set-{}", s.name));
            let offset = scroll.offset();
            lifted = Some(dropdown(
                &id,
                &combo,
                (
                    origin.0 + 153.0,
                    origin.1 + y + 32.0 + f32::from(offset.y),
                    152.0,
                ),
                move |this, chosen| osd(this).choose(index, chosen),
                |_this, _lines| {},
                cx,
            ));
        }
        y += control.height();
    }
    (column.h(px(y)).into_any_element(), lifted)
}

/// One setting's row: its name at (16, 15) and its control at (153, 13).
/// `// C#: GUI/OptionControls/*.Designer.cs`
#[allow(clippy::too_many_arguments)]
fn option_row(
    page: &OnboardOsd,
    index: usize,
    control: Control,
    number_focus: &FocusHandle,
    text_focus: &FocusHandle,
    enabled: bool,
    (x, y, width): (f32, f32, f32),
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(setting) = page.settings.get(index) else {
        return div().into_any_element();
    };
    let name = setting.name.clone();
    let id = format!("osd-set-{name}");
    let value = setting.value();
    let mut row = crate::probe::measured(format!("{id}-row"), div())
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(control.height()))
        .child(
            label(16.0, 15.0, SharedString::from(name.clone()), enabled).text_color(rgb(
                if setting.changed() {
                    theme::WARN
                } else if enabled {
                    theme::TEXT
                } else {
                    theme::DIM
                },
            )),
        );
    match control {
        Control::Bool => {
            let mut check = Check::default();
            check.enabled = enabled;
            check.state = if value > 0.0 {
                CheckState::Checked
            } else {
                CheckState::Unchecked
            };
            row = row.child(check_box(
                format!("{id}-check"),
                &check,
                "",
                (155.0, 15.0),
                move |this| osd(this).toggle(index),
                cx,
            ));
        }
        Control::Spin => {
            if let Some(number) = page.numbers.get(index) {
                row = row.child(number_box(
                    id.clone(),
                    number,
                    page.editing == Some(index),
                    number_focus,
                    (153.0, 13.0, 152.0, 20.0),
                    NumberHandlers {
                        begin: move |this: &mut MissionPlanner| osd(this).begin_number(index),
                        key: move |this: &mut MissionPlanner, event: &KeyDownEvent| {
                            osd(this).number_key(event, Instant::now())
                        },
                        step: move |this: &mut MissionPlanner, up: bool| {
                            osd(this).step(index, up, Instant::now());
                        },
                    },
                    window,
                    cx,
                ));
            }
        }
        Control::Dropdown(choices) => {
            let mut combo = page.combo(index, choices);
            combo.enabled = enabled;
            // Its list, when dropped, is drawn by `option_rows` at the page's level.
            row = row.child(combo_box(
                id.clone(),
                &combo,
                (153.0, 11.0, 152.0, 21.0),
                move |this| osd(this).toggle_combo(index),
                cx,
            ));
        }
        Control::Bitwise(bits) => {
            let typing = page.typing == Some(TextTarget::Setting(index));
            row = row.child(text_box_id(
                id.clone(),
                &if typing {
                    page.field.value().to_owned()
                } else {
                    value_text(value)
                },
                Some(text_focus),
                typing,
                enabled,
                (152.0, 12.0, 152.0, 20.0),
                move |this| osd(this).begin_text(TextTarget::Setting(index)),
                |this, event| osd(this).text_key(event),
                cx,
            ));
            #[allow(clippy::cast_possible_truncation)]
            let held = value as i32;
            for (bit, text) in bits.iter().enumerate() {
                let mut check = Check::default();
                check.enabled = enabled;
                #[allow(clippy::cast_possible_truncation)]
                let on = held & (1_i32 << bit) != 0;
                check.state = if on {
                    CheckState::Checked
                } else {
                    CheckState::Unchecked
                };
                #[allow(clippy::cast_precision_loss)]
                let top = 35.0 + bit as f32 * 16.0;
                let bit_u32 = u32::try_from(bit).unwrap_or(0);
                row = row.child(check_box(
                    format!("{id}-bit-{bit}"),
                    &check,
                    text,
                    (152.0, top),
                    move |this| osd(this).toggle_bit(index, bit_u32),
                    cx,
                ));
            }
        }
        Control::Int => {
            let typing = page.typing == Some(TextTarget::Setting(index));
            row = row.child(text_box_id(
                id.clone(),
                &if typing {
                    page.field.value().to_owned()
                } else {
                    value_text(value)
                },
                Some(text_focus),
                typing,
                enabled,
                (153.0, 13.0, 152.0, 20.0),
                move |this| osd(this).begin_text(TextTarget::Setting(index)),
                |this, event| osd(this).text_key(event),
                cx,
            ));
        }
    }
    row.into_any_element()
}

/// [`super::optional::text_box`] with an id made for the row: the same box.
#[allow(clippy::too_many_arguments)]
fn text_box_id(
    id: String,
    text: &str,
    typing: Option<&FocusHandle>,
    focused: bool,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    on_begin: impl Fn(&mut MissionPlanner) + 'static,
    on_key: impl Fn(&mut MissionPlanner, &KeyDownEvent) -> bool + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .text_xs()
        .overflow_hidden()
        .whitespace_nowrap()
        .child(text.to_owned());
    let base = match typing {
        Some(handle) if enabled => {
            let handle = handle.clone();
            let base = if focused {
                base.track_focus(&handle)
                    .key_context("TextField")
                    .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                        if on_key(this, event) {
                            cx.notify();
                        }
                    }))
                    .child(div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT)))
            } else {
                base.on_click(cx.listener(move |this, _event, window, cx| {
                    on_begin(this);
                    handle.focus(window, cx);
                    cx.notify();
                }))
            };
            base.bg(rgb(theme::ACTION))
                .border_color(rgb(if focused {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .text_color(rgb(theme::TEXT))
                .cursor_text()
        }
        _ => base
            .bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM })),
    };
    at(x, y, width, height).child(base).into_any_element()
}

// ---------------------------------------------------------------------------------------------
// Over the window: the questions, the progress dialogs, the slots dialog.
// ---------------------------------------------------------------------------------------------

/// What the page shows over the whole window, if anything.
pub fn overlay(
    page: &OnboardOsd,
    text_focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = page.question {
        return Some(question_box(question, window, cx));
    }
    if let Some(writing) = &page.writing {
        return Some(progress_dialog(
            ProgressIds {
                frame: "osd-progress",
                bar: "osd-progress-bar",
                cancel: "osd-progress-cancel",
                backdrop: "osd-progress-backdrop",
            },
            WRITING,
            Bar {
                marquee: true,
                value: 0,
            },
            writing.started,
            Some(|this: &mut MissionPlanner| osd(this).cancel_write()),
            window,
            cx,
        ));
    }
    if let Some((caption, started)) = page.slots.busy() {
        return Some(progress_dialog(
            ProgressIds {
                frame: "osd-progress",
                bar: "osd-progress-bar",
                cancel: "osd-progress-cancel",
                backdrop: "osd-progress-backdrop",
            },
            caption,
            Bar {
                marquee: true,
                value: 0,
            },
            started,
            Some(|this: &mut MissionPlanner| osd(this).slots.cancel()),
            window,
            cx,
        ));
    }
    page.slots
        .dialog
        .as_ref()
        .map(|dialog| slots_dialog(dialog, text_focus, window, cx))
}

/// A question: Discard's OK/Cancel, Reset's Yes/No, or Update Params' OK/Cancel with its
/// Show-me-again box.
fn question_box(
    question: Question,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let answer = |yes: bool| {
        move |this: &mut MissionPlanner,
              _event: &(),
              _window: &mut Window,
              cx: &mut Context<MissionPlanner>| {
            let view = this.telemetry.view();
            let shown_again = this.persisted.get(SHOW_AGAIN_KEY).map(str::to_owned);
            let telemetry = &this.telemetry;
            this.software_pages2
                .onboard_osd
                .answer(yes, telemetry, &view, shown_again.as_deref());
            cx.notify();
        }
    };
    let (title, text, yes_text, no_text): (&str, String, &'static str, &'static str) =
        match question {
            Question::Discard => ("", ARE_YOU_SURE.to_owned(), "OK", "Cancel"),
            Question::ResetChanges => ("", RESET_CHANGES.to_owned(), "Yes", "No"),
            Question::RefreshArmed(_) => {
                (REFRESH_PARAMS, REFRESH_WARNING.to_owned(), "OK", "Cancel")
            }
        };
    let mut buttons: Vec<AnyElement> = Vec::new();
    if let Question::RefreshArmed(ticked) = question {
        let mut check = Check::default();
        check.enabled = true;
        check.state = if ticked {
            CheckState::Checked
        } else {
            CheckState::Unchecked
        };
        buttons.push(
            div()
                .relative()
                .w(px(120.0))
                .h(px(20.0))
                .child(check_box(
                    "osd-question-showagain".to_owned(),
                    &check,
                    SHOW_ME_AGAIN,
                    (0.0, 2.0),
                    |this| {
                        if let Some(value) = osd(this).toggle_show_again() {
                            this.persisted.set(SHOW_AGAIN_KEY, value);
                        }
                    },
                    cx,
                ))
                .into_any_element(),
        );
    }
    buttons.push(action(
        "osd-question-yes",
        yes_text,
        theme::ACCENT,
        true,
        cx.listener(answer(true)),
    ));
    buttons.push(action(
        "osd-question-no",
        no_text,
        theme::ACCENT,
        true,
        cx.listener(answer(false)),
    ));
    modal("osd-question", title, &text, false, buttons, window)
}

/// `SetupDialog`: "OSD/Telemetry Parameters Setup" - a heading a screen, a row a slot, and
/// "Done && Refresh".
/// `// C#: GUI/Osd56ItemsSetup/SetupDialog.Designer.cs; SetupDialog.cs:55-76; ItemControl.Designer.cs`
fn slots_dialog(
    dialog: &Dialog,
    text_focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    const WIDTH: f32 = 778.0;
    const ROW: f32 = 27.0 + 3.0;
    const HEADING: f32 = 30.0;
    const BOTTOM: f32 = 41.0;
    const TITLE_BAR: f32 = 24.0;
    let screens = dialog.screens();
    #[allow(clippy::cast_precision_loss)]
    let content_height = screens.len() as f32 * HEADING + dialog.rows.len() as f32 * ROW;
    let height = TITLE_BAR + content_height + BOTTOM;
    let viewport = window.viewport_size();
    let left = ((f32::from(viewport.width) - WIDTH) / 2.0).max(0.0);
    let top = ((f32::from(viewport.height) - height) / 2.0).max(0.0);
    let mut content = div().relative().w(px(WIDTH)).h(px(content_height));
    let mut y = 0.0;
    for screen in screens {
        content = content.child(
            at(0.0, y, WIDTH, HEADING)
                .flex()
                .items_center()
                .justify_center()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(format!("OSD {screen}")),
        );
        y += HEADING;
        for (index, row) in dialog.rows.iter().enumerate() {
            if row.screen != screen {
                continue;
            }
            content = content.child(slot_row(dialog, index, row, (0.0, y), text_focus, cx));
            y += ROW;
        }
    }
    let frame = crate::probe::measured("osd-slots-dialog", div())
        .absolute()
        .left(px(left))
        .top(px(top))
        .w(px(WIDTH))
        .h(px(height))
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .child(
            at(0.0, 0.0, WIDTH, TITLE_BAR)
                .flex()
                .items_center()
                .px_2()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(DIALOG_TITLE)
                .child(div().flex_1())
                .child(action(
                    "osd-slots-close",
                    "X",
                    theme::DIM,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        osd(this).slots.close_dialog();
                        cx.notify();
                    }),
                )),
        )
        .child(at(0.0, TITLE_BAR, WIDTH, content_height).child(content))
        .child(
            at(0.0, TITLE_BAR + content_height, WIDTH, BOTTOM).child(button(
                "osd-slots-done",
                DONE_REFRESH,
                (332.0, 9.0, 117.0, 23.0),
                true,
                |this, _window, _cx| {
                    let view = this.telemetry.view();
                    let telemetry = &this.telemetry;
                    this.software_pages2
                        .onboard_osd
                        .slots
                        .done(telemetry, &view, Instant::now());
                },
                cx,
            )),
        );
    crate::probe::measured("osd-slots-backdrop", div())
        .id("osd-slots-backdrop")
        .absolute()
        .left(px(0.0))
        .top(px(0.0))
        .w(viewport.width)
        .h(viewport.height)
        .bg(gpui::rgba(0x00_00_00_80))
        .on_click(cx.listener(|_this, _event, _window, _cx| {}))
        .child(frame)
        .into_any_element()
}

/// `ItemControl`: the name combo at (3, 3), the type combo at (193, 3), Min at (413, 3), Max at
/// (535, 4) and Increment at (689, 4), with their labels.
/// `// C#: GUI/Osd56ItemsSetup/ItemControl.Designer.cs:43-107`
fn slot_row(
    dialog: &Dialog,
    index: usize,
    row: &super::onboard_osd_slots::Row,
    (x, y): (f32, f32),
    text_focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = format!("osd-slot-{}-{}", row.screen, row.index);
    let mut element = crate::probe::measured(base.clone(), div())
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(775.0))
        .h(px(27.0));
    for (kind, id, (cx_, cy, cw)) in [
        (ComboKind::Name, format!("{base}-name"), (3.0, 3.0, 184.0)),
        (ComboKind::Type, format!("{base}-type"), (193.0, 3.0, 184.0)),
    ] {
        let combo: Combo = match kind {
            ComboKind::Name => dialog.name_combo(index),
            ComboKind::Type => dialog.type_combo(index),
        };
        let open = dialog.open == Some((index, kind));
        let mut holder = div().absolute().left(px(0.0)).top(px(0.0)).child(combo_box(
            id.clone(),
            &combo,
            (cx_, cy, cw, 21.0),
            move |this| {
                if let Some(dialog) = osd(this).slots.dialog.as_mut() {
                    dialog.toggle_combo(index, kind);
                }
            },
            cx,
        ));
        if open {
            holder = holder.child(dropdown(
                &id,
                &combo,
                (cx_, cy + 21.0, cw),
                move |this, chosen| {
                    if let Some(dialog) = osd(this).slots.dialog.as_mut() {
                        dialog.choose(index, kind, chosen);
                    }
                },
                |this, lines| {
                    if let Some(dialog) = osd(this).slots.dialog.as_mut() {
                        dialog.scroll(lines);
                    }
                },
                cx,
            ));
        }
        element = element.child(holder);
    }
    let enabled = row.numbers_enabled();
    for (which, caption, label_x, (bx, by), text) in [
        (Box_::Min, "Min:", 383.0, (413.0, 3.0), &row.min),
        (Box_::Max, "Max:", 502.0, (535.0, 4.0), &row.max),
        (
            Box_::Increment,
            "Increment:",
            629.0,
            (689.0, 4.0),
            &row.increment,
        ),
    ] {
        let typing = dialog.typing == Some((index, which));
        let shown = if typing {
            dialog.field.value().to_owned()
        } else {
            text.clone()
        };
        let id = format!(
            "{base}-{}",
            match which {
                Box_::Min => "min",
                Box_::Max => "max",
                Box_::Increment => "incr",
            }
        );
        element = element
            .child(label(label_x, 7.0, caption, true))
            .child(text_box_id(
                id,
                &shown,
                Some(text_focus),
                typing,
                enabled,
                (bx, by, 83.0, 20.0),
                move |this| {
                    if let Some(dialog) = osd(this).slots.dialog.as_mut() {
                        dialog.begin_box(index, which);
                    }
                },
                |this, event| {
                    osd(this)
                        .slots
                        .dialog
                        .as_mut()
                        .is_some_and(|dialog| dialog.key(event))
                },
                cx,
            ));
    }
    element.into_any_element()
}
