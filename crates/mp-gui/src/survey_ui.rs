//! The Survey (Grid) dialog: `Grid/GridUI.cs`, opened from the planning map's Auto WP menu.
//!
//! What the dialog decides - every control's value, the events they raise, the grid regenerated
//! on each change, the Stats, what Accept adds - is `mp_mission::gridui`, held to the C# run under
//! mono by `crates/mp-mission/tests/gridui_vectors.rs`. This module is its form and its place in the
//! planner:
//!
//! - **The layout is `GridUI.resx`'s.** `tabControl1` docked right, 253 pixels wide, with its
//!   pages Simple, Grid Options and Camera Config - the last two only while Advanced Options is
//!   checked - drawn as the flight screen's page strip draws its tabs; `groupBox5`, Stats, docked
//!   along the bottom of what is left, 80 pixels tall; the map in the rest. Every group box and
//!   control sits at its `.resx` location and size. The dialog is modal (`ShowDialog`), so while it
//!   shows it takes the planning screen, map and all, and the planner's sidebar and menu wait.
//! - **The map is the planner's own**, showing the dialog's grid as `routesOverlay` does - the
//!   lanes and their markers (Grid, Markers), the trigger points with Internals, the polygon
//!   (Boundary) - through the map's mission and polygon layers. Drag pans, the wheel zooms.
//! - **Accept** puts the rows in the planner as `FlightPlanner.AddCommand` fills them
//!   (`FillCommand` and `setfromMap`): in the screen's altitude frame, a `WAYPOINT` as a
//!   `SPLINE_WAYPOINT` while the planner's Spline box is ticked, a waypoint's position to seven
//!   decimals and its altitude through Default Alt, its delay rounded to a tenth.
//!
//! Not ported, each for a reason the C# gives: `TRK_zoom` (the map zooms with the wheel);
//! dragging the polygon's corners on the dialog's map; the camera footprints (`CHK_footprints` is
//! kept and saved, nothing is drawn); `BUT_samplephoto` (a JPEG's EXIF), `BUT_save` and the
//! `cameras.xml` the constructor writes when there is none (this application does not write
//! Mission Planner's data directory); Control-S and Control-O with `label38`, their hint (the
//! `.grid` files are `XmlSerializer` output); "No polygon defined. Load a file?", whose Yes is
//! that `.grid` loader - with fewer than three corners this says its No, "Please define a
//! polygon.". The settings Accept saves are kept for the session, over what `config.xml` holds:
//! this application does not write `config.xml`.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;

use gpui::{
    AnyElement, Context, Div, FocusHandle, MouseButton, SharedString, Window, div, prelude::*, px,
    rgb,
};
use mp_mission::MissionItem;
use mp_mission::cameras::Cameras;
use mp_mission::dotnet::{bool_text, format_f64, parse_f64, to_int};
use mp_mission::grid::{GridTag, StartPosition};
use mp_mission::gridui::{
    self, Call, Check, Dialog, Num, Refused, Stat, Step, Tab, Text, Trigger, cmd,
};
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::plan::{AltitudeFrame, Plan};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// WinForms' 8.25 pt Microsoft Sans Serif is eleven pixels; the `.resx` sizes are laid out for it.
const FONT: f32 = 11.0;

/// The dialog, while it shows, and what outlives it.
pub struct SurveyUi {
    open: Option<Open>,
    focus: FocusHandle,
    /// `Grid.StartPointLatLngAlt`: a static, so it carries from one dialog to the next.
    start_point: LatLon,
    /// What Accept saved this session (`savesettings`), which the next dialog loads over
    /// `config.xml`'s values.
    saved: BTreeMap<String, String>,
    /// The rows the last Accept added.
    added: usize,
}

/// One showing of the dialog.
struct Open {
    dialog: Dialog,
    /// `tabControl1.SelectedTab`.
    tab: Tab,
    /// The box being typed into, and what has been typed.
    editing: Option<(Field, TextField)>,
    /// A combo box's list, while it is open.
    dropdown: Option<Combo>,
    /// `InputBox` or `CustomMessageBox`, over the dialog.
    prompt: Option<Prompt>,
}

/// A box that takes typing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Field {
    Num(Num),
    Text(Text),
}

impl Field {
    const fn name(self) -> &'static str {
        match self {
            Self::Num(num) => num.name(),
            Self::Text(text) => text.name(),
        }
    }
}

/// The two combo boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Combo {
    /// `CMB_camera`.
    Camera,
    /// `CMB_startfrom`.
    StartFrom,
}

impl Combo {
    const fn name(self) -> &'static str {
        match self {
            Self::Camera => "CMB_camera",
            Self::StartFrom => "CMB_startfrom",
        }
    }
}

/// A box over the dialog.
enum Prompt {
    /// `InputBox.Show("Enter point #", "Please enter a boundary point number", ref pnt)`.
    Point(TextField),
    /// `CustomMessageBox.Show(text, caption)`.
    Message(String, &'static str),
}

impl SurveyUi {
    /// Closed, with nothing saved yet.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            open: None,
            focus: cx.focus_handle(),
            start_point: LatLon::default(),
            saved: BTreeMap::new(),
            added: 0,
        }
    }

    /// Whether the dialog is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open.is_some()
    }

    /// What the map shows while the dialog is open: the grid as waypoints, and the boundary - or
    /// `None` while it is closed.
    #[must_use]
    pub fn preview(&self) -> Option<(Vec<MissionItem>, Vec<LatLon>)> {
        let open = self.open.as_ref()?;
        Some(preview(&open.dialog))
    }
}

/// `routesOverlay` as `domainUpDown1_ValueChanged` fills it: the lanes' ends and turns with the
/// route between them (Grid, Markers), the trigger points as well with Internals, and the polygon
/// with Boundary. The map draws a mission's markers and its path together, so either box shows
/// both.
/// `// C#: Grid/GridUI.cs:626-770, 922-955`
fn preview(dialog: &Dialog) -> (Vec<MissionItem>, Vec<LatLon>) {
    let route = dialog.check(Check::Grid) || dialog.check(Check::Markers);
    let internals = dialog.check(Check::Internals);
    let items = if route {
        dialog
            .grid()
            .iter()
            .filter(|point| internals || point.tag != GridTag::Middle)
            .zip(1_u16..)
            .map(|(point, seq)| MissionItem {
                seq,
                command: cmd::WAYPOINT,
                x: point.lat,
                y: point.lng,
                z: point.alt,
                ..MissionItem::default()
            })
            .collect()
    } else {
        Vec::new()
    };
    let boundary = if dialog.check(Check::Boundary) {
        dialog.polygon().to_vec()
    } else {
        Vec::new()
    };
    (items, boundary)
}

// ---------------------------------------------------------------------------------------------
// Opening, accepting and closing.
// ---------------------------------------------------------------------------------------------

/// `surveyGridToolStripMenuItem_Click`: `GridPlugin.but_Click`, which shows the dialog when the
/// drawn polygon has more than two points.
/// `// C#: GCSViews/FlightPlanner.cs:6755-6760; Grid/GridPlugin.cs:37-61`
pub fn open(this: &mut MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>) {
    let polygon = this.plan.polygon().to_vec();
    if polygon.len() <= 2 {
        // The C# asks "No polygon defined. Load a file?" first; its Yes loads a `.grid`, which is
        // not ported, so this is its No.
        this.plan_menus.say("Error", "Please define a polygon.");
        return;
    }
    let view = this.telemetry.view();
    let home = this.plan.planned_home_location();
    let context = gridui::Context {
        home: (home.lat, home.lng, home.alt),
        plane: firmware_is_plane(&view),
        start_point: this.survey.start_point,
    };
    // `plugin.Host.config`: what this session's Accepts saved, over `config.xml`.
    let config =
        mp_settings::Config::default_path().and_then(|path| mp_settings::Config::load(&path).ok());
    let session = &this.survey.saved;
    let saved = |key: &str| {
        session.get(key).cloned().or_else(|| {
            config
                .as_ref()
                .and_then(|c| c.get(key))
                .map(ToOwned::to_owned)
        })
    };
    let cameras = Cameras::load(mp_settings::user_data_directory().as_deref());
    let dialog = Dialog::open(&polygon, cameras, &context, &saved);
    this.survey.open = Some(Open {
        dialog,
        tab: Tab::Simple,
        editing: None,
        dropdown: None,
        prompt: None,
    });
    this.survey.added = 0;
    changed(this, window, cx);
}

/// The form's close box: the dialog goes, and nothing is added.
fn close(this: &mut MissionPlanner) {
    if let Some(open) = this.survey.open.take() {
        this.survey.start_point = open.dialog.start_point();
    }
    this.sync_map_mission();
    this.sync_map_polygon();
}

/// `cs.firmware == Firmwares.ArduPlane`: a fixed wing, a flapping wing or a VTOL.
fn firmware_is_plane(view: &crate::telemetry::TelemetryView) -> bool {
    view.state
        .as_ref()
        .and_then(|state| mp_vehicle::VehicleFamily::from_mav_type(state.vehicle_type))
        == Some(mp_vehicle::VehicleFamily::Plane)
}

/// How the planner fills the rows Accept hands it.
#[derive(Debug, Clone, Copy)]
pub struct Fill {
    /// `CMB_altmode`: the frame a new row gets.
    pub frame: AltitudeFrame,
    /// `cs.firmware == Firmwares.ArduCopter2`, for a Default Alt of 0.
    pub copter: bool,
    /// Whether the command headers are ArduPlane's, whose `WAYPOINT` has no "Delay" column.
    pub plane: bool,
}

/// `FillCommand` and `setfromMap`: one of Accept's calls as the `Commands` grid holds it. A
/// `WAYPOINT` becomes a `SPLINE_WAYPOINT` while the planner's Spline box is ticked; it takes its
/// position as `ToString("0.0000000")` writes it, its altitude through Default Alt, and its delay,
/// under a "Delay" header, `Math.Round(p1, 1)`. Anything else takes the numbers as they come.
/// Default Alt that is not a whole number is "Your default alt is not valid", with the row left
/// at the altitude `Commands_RowsAdded` gave it, 0.
/// `// C#: GCSViews/FlightPlanner.cs:3376-3413, 1111-1260`
#[must_use]
pub fn fill_row(plan: &Plan, call: &Call, fill: &Fill) -> (MissionItem, Option<&'static str>) {
    let mut item = MissionItem {
        seq: 0,
        current: 0,
        frame: fill.frame.mav_frame(),
        command: call.command,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        autocontinue: 1,
    };
    // switch wp to spline if spline checked
    if plan.spline() && call.command == cmd::WAYPOINT {
        item.command = cmd::SPLINE_WAYPOINT;
    }
    if call.command == cmd::WAYPOINT {
        item.param1 = call.params[0];
        // lat.ToString("0.0000000"), read back as the grid's cells are when the mission is written
        item.x = seven_decimals(call.y);
        item.y = seven_decimals(call.x);
        match plan.new_row_altitude(f64::from(to_int(call.z)), fill.copter) {
            // `setfromMap`'s Verify Height branch applies to the grid's rows as to any other:
            // `AddWPtoList` goes through `AddCommand` and `setfromMap`.
            // `// C#: Grid/GridUI.cs:896-902; GCSViews/FlightPlanner.cs:1209-1236`
            Ok(altitude) => item.z = plan.verified_altitude(item.x, item.y, altitude, fill.frame),
            Err(why) => return (item, Some(why)),
        }
        if !fill.plane {
            // Math.Round(p1, 1): scaled, rounded half to even, scaled back.
            item.param1 = (call.params[0] * 10.0).round_ties_even() / 10.0;
        }
    } else {
        [item.param1, item.param2, item.param3, item.param4] = call.params;
        item.x = call.y;
        item.y = call.x;
        item.z = call.z;
    }
    (item, None)
}

/// A coordinate as `ToString("0.0000000")` writes it and `double.Parse` reads it back.
fn seven_decimals(value: f64) -> f64 {
    parse_f64(&format_f64(value, "0.0000000")).unwrap_or(value)
}

/// Accept's rows into the planner: `AddCommand` at the end, `InsertCommand` at a row, or at the
/// end past the last. The first thing `setfromMap` refused, if anything was.
pub fn apply_steps(plan: &mut Plan, steps: &[Step], fill: &Fill) -> Option<&'static str> {
    let mut refused = None;
    for step in steps {
        let (index, call) = match step {
            Step::Add(call) => (None, call),
            Step::Insert(index, call) => (Some(*index), call),
        };
        let (row, why) = fill_row(plan, call, fill);
        refused = refused.or(why);
        match index {
            Some(index) => {
                if plan.insert_after(&index.to_string(), row).is_err() {
                    plan.append(row);
                }
            }
            None => plan.append(row),
        }
    }
    refused
}

/// `BUT_Accept_Click`, and what the planner does with it: the rows, the polygon redrawn
/// (`RedrawFPPolygon`), the settings kept, the dialog closed - or its message, over the dialog.
/// `// C#: Grid/GridUI.cs:1595-1880`
fn accept(this: &mut MissionPlanner) {
    let view = this.telemetry.view();
    let param = |name: &str| {
        view.parameters
            .iter()
            .find(|(held, _)| held == name)
            .map(|(_, value)| *value)
    };
    let planner = gridui::Planner {
        rows: this.plan.items().len(),
        wpnav_speed: param("WPNAV_SPEED"),
        wp_spd: param("WP_SPD"),
    };
    let fill = Fill {
        frame: this.altitude_frame,
        copter: crate::plan::firmware_is_copter(&view),
        plane: firmware_is_plane(&view),
    };
    let Some(open) = this.survey.open.as_mut() else {
        return;
    };
    commit_edit(open);
    match open.dialog.accept(&planner) {
        Ok(accepted) => {
            let refused = apply_steps(&mut this.plan, &accepted.steps, &fill);
            this.plan.clear_area();
            for vertex in &accepted.polygon {
                this.plan.add_area_vertex(*vertex);
            }
            for (key, value) in accepted.settings {
                this.survey.saved.insert(key.to_owned(), value);
            }
            this.survey.added = accepted.steps.len();
            close(this);
            if let Some(why) = refused {
                this.plan_menus.say("", why);
            }
        }
        Err(Refused::Message(text, caption)) => {
            open.prompt = Some(Prompt::Message(text.to_owned(), caption));
        }
        Err(Refused::Exception(steps, text)) => {
            open.prompt = Some(Prompt::Message(text.to_owned(), ""));
            let _ = apply_steps(&mut this.plan, &steps, &fill);
            this.survey.added = steps.len();
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Typing.
// ---------------------------------------------------------------------------------------------

/// Leaving a box: a `NumericUpDown`'s `ValidateEditText`. A `TextBox` has already had each
/// keystroke.
fn commit_edit(open: &mut Open) {
    if let Some((Field::Num(num), field)) = open.editing.take() {
        open.dialog.type_num(num, field.value());
    }
}

/// After any change to the dialog: the map shows its grid again, and the screen redraws, as
/// `domainUpDown1_ValueChanged` ends with `map.Invalidate()`.
fn changed(this: &mut MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>) {
    this.sync_map_mission();
    this.sync_map_polygon();
    window.refresh();
    cx.notify();
}

/// A click on a box: whatever was being typed is committed, and this one takes the keyboard.
fn start_edit(
    this: &mut MissionPlanner,
    field: Field,
    window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    let Some(open) = this.survey.open.as_mut() else {
        return;
    };
    if open
        .editing
        .as_ref()
        .is_some_and(|(held, _)| *held == field)
    {
        return;
    }
    commit_edit(open);
    open.dropdown = None;
    if let Field::Text(Text::HeadingHold) = field
        && (!open.dialog.heading_enabled() || open.dialog.heading_read_only())
    {
        return;
    }
    let mut text = TextField::new("");
    text.set(match field {
        Field::Num(num) => open.dialog.num_text(num),
        Field::Text(which) => open.dialog.text(which).to_owned(),
    });
    open.editing = Some((field, text));
    this.survey.focus.focus(window, cx);
    changed(this, window, cx);
}

/// A key in the box being typed into. A `NumericUpDown` takes digits, a sign and the separators
/// and parses on Enter; a `TextBox` raises `TextChanged` on every change.
fn key(
    this: &mut MissionPlanner,
    event: &gpui::KeyDownEvent,
    window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    let Some(open) = this.survey.open.as_mut() else {
        return;
    };
    if let Some(Prompt::Point(field)) = open.prompt.as_mut() {
        match field.key(event) {
            KeyOutcome::Submitted => {
                let answer = field.value().to_owned();
                open.prompt = None;
                open.dialog.answer_point(Some(&answer));
            }
            KeyOutcome::Cancelled => {
                open.prompt = None;
                open.dialog.answer_point(None);
            }
            KeyOutcome::Changed | KeyOutcome::Ignored => {}
        }
        changed(this, window, cx);
        return;
    }
    let Some((field, text)) = open.editing.as_mut() else {
        return;
    };
    let field = *field;
    // NumericUpDown's key filter: a printable character that is not part of a number is refused.
    if let Field::Num(_) = field
        && let Some(typed) = event.keystroke.key_char.as_deref()
        && !event.keystroke.modifiers.control
        && typed
            .chars()
            .any(|c| !c.is_control() && !c.is_ascii_digit() && !matches!(c, '.' | ',' | '-' | '+'))
    {
        return;
    }
    match text.key(event) {
        KeyOutcome::Changed => {
            if let Field::Text(which) = field {
                let value = text.value().to_owned();
                open.dialog.set_text(which, &value);
            }
        }
        KeyOutcome::Submitted => commit_edit(open),
        KeyOutcome::Cancelled => open.editing = None,
        KeyOutcome::Ignored => return,
    }
    changed(this, window, cx);
}

// ---------------------------------------------------------------------------------------------
// The form.
// ---------------------------------------------------------------------------------------------

/// A control at its `.resx` location.
fn at(x: f32, y: f32) -> Div {
    div().absolute().left(px(x)).top(px(y))
}

/// A `Label`.
fn label(x: f32, y: f32, text: &str) -> AnyElement {
    at(x, y)
        .whitespace_nowrap()
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .child(text.to_owned())
        .into_any_element()
}

/// A `GroupBox`: its caption over a border, its controls at their locations inside it.
fn group(title: &str, x: f32, y: f32, w: f32, h: f32, children: Vec<AnyElement>) -> AnyElement {
    at(x, y)
        .w(px(w))
        .h(px(h))
        .child(
            div()
                .absolute()
                .left(px(0.0))
                .top(px(6.0))
                .w(px(w))
                .h(px(h - 6.0))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .rounded_sm(),
        )
        .child(
            at(6.0, 0.0)
                .px_1()
                .bg(rgb(theme::PANEL))
                .whitespace_nowrap()
                .text_size(px(FONT))
                .text_color(rgb(theme::DIM))
                .child(title.to_owned()),
        )
        .children(children)
        .into_any_element()
}

fn control_id(name: &str) -> SharedString {
    SharedString::from(format!("survey-{name}"))
}

/// A `NumericUpDown`: the value at its places, or what is being typed, and the two arrows.
#[allow(clippy::too_many_arguments)]
fn numeric(
    open: &Open,
    num: Num,
    x: f32,
    y: f32,
    w: f32,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let editing = open
        .editing
        .as_ref()
        .filter(|(field, _)| *field == Field::Num(num));
    let shown = editing.map_or_else(
        || open.dialog.num_text(num),
        |(_, text)| text.value().to_owned(),
    );
    let focused = editing.is_some() && focus.is_focused(window);
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let name = format!("{}-{}", num.name(), if up { "up" } else { "down" });
        crate::probe::measured(format!("survey-{name}"), div())
            .id(control_id(&name))
            .h(px(9.0))
            .w(px(14.0))
            .flex()
            .items_center()
            .justify_center()
            .bg(rgb(theme::ACTION))
            .text_size(px(7.0))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(if up { "\u{25b2}" } else { "\u{25bc}" })
            .on_click(
                cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                    let Some(open) = this.survey.open.as_mut() else {
                        return;
                    };
                    // UpButton: `if (UserEdit) ParseEditText()` first.
                    commit_edit(open);
                    open.dropdown = None;
                    if up {
                        open.dialog.up(num);
                    } else {
                        open.dialog.down(num);
                    }
                    changed(this, window, cx);
                }),
            )
    };
    let text = crate::probe::measured(format!("survey-{}", num.name()), div())
        .id(control_id(num.name()))
        .flex_1()
        .h_full()
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .cursor_text()
        .child(shown)
        .children(focused.then(|| div().w(px(1.0)).h(px(11.0)).bg(rgb(theme::ACCENT))))
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                start_edit(this, Field::Num(num), window, cx);
            }),
        );
    let mut body = at(x, y)
        .w(px(w))
        .h(px(20.0))
        .flex()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::BG))
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .child(text)
        .child(
            div()
                .flex()
                .flex_col()
                .justify_between()
                .child(arrow(true, cx))
                .child(arrow(false, cx)),
        );
    if editing.is_some() {
        body =
            body.track_focus(focus).on_key_down(cx.listener(
                |this, event: &gpui::KeyDownEvent, window, cx| key(this, event, window, cx),
            ));
    }
    body.into_any_element()
}

/// A `TextBox`.
#[allow(clippy::too_many_arguments)]
fn textbox(
    open: &Open,
    which: Text,
    x: f32,
    y: f32,
    w: f32,
    enabled: bool,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let editing = open
        .editing
        .as_ref()
        .filter(|(field, _)| *field == Field::Text(which));
    let shown = editing.map_or_else(
        || open.dialog.text(which).to_owned(),
        |(_, text)| text.value().to_owned(),
    );
    let focused = editing.is_some() && focus.is_focused(window);
    let mut body = crate::probe::measured(format!("survey-{}", which.name()), at(x, y))
        .id(control_id(which.name()))
        .w(px(w))
        .h(px(20.0))
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if enabled { theme::BG } else { theme::PANEL }))
        .text_size(px(FONT))
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(if shown.is_empty() {
            " ".to_owned()
        } else {
            shown
        })
        .children(focused.then(|| div().w(px(1.0)).h(px(11.0)).bg(rgb(theme::ACCENT))));
    if enabled {
        body = body.cursor_text().on_click(cx.listener(
            move |this, _event: &gpui::ClickEvent, window, cx| {
                start_edit(this, Field::Text(which), window, cx);
            },
        ));
    }
    if editing.is_some() {
        body =
            body.track_focus(focus).on_key_down(cx.listener(
                |this, event: &gpui::KeyDownEvent, window, cx| key(this, event, window, cx),
            ));
    }
    body.into_any_element()
}

/// A box and its text, for a `CheckBox` or, round, a `RadioButton`.
fn tick(
    name: &str,
    text: &str,
    checked: bool,
    round: bool,
    enabled: bool,
    x: f32,
    y: f32,
) -> gpui::Stateful<Div> {
    let mark = div()
        .w(px(11.0))
        .h(px(11.0))
        .flex_shrink_0()
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(if enabled { theme::TEXT } else { theme::BORDER }))
        .bg(rgb(theme::BG))
        .text_size(px(9.0))
        .text_color(rgb(theme::ACCENT))
        .children(checked.then_some(if round { "\u{25cf}" } else { "\u{2713}" }));
    let mark = if round { mark.rounded_full() } else { mark };
    crate::probe::measured(format!("survey-{name}"), at(x, y))
        .id(control_id(name))
        .flex()
        .items_center()
        .gap_1()
        .whitespace_nowrap()
        .text_size(px(FONT))
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(mark)
        .child(text.to_owned())
}

/// A `CheckBox`: a click toggles it and raises its events.
fn checkbox(
    open: &Open,
    check: Check,
    x: f32,
    y: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let enabled = check != Check::HeadingHoldLock || open.dialog.heading_enabled();
    let body = tick(
        check.name(),
        check.text(),
        open.dialog.check(check),
        false,
        enabled,
        x,
        y,
    );
    if !enabled {
        return body.into_any_element();
    }
    body.cursor_pointer()
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                let Some(open) = this.survey.open.as_mut() else {
                    return;
                };
                commit_edit(open);
                open.dropdown = None;
                open.dialog.click_check(check);
                if !open.dialog.tabs().contains(&open.tab) {
                    open.tab = Tab::Simple;
                }
                changed(this, window, cx);
            }),
        )
        .into_any_element()
}

/// One of the trigger method's `RadioButton`s.
fn radio(
    open: &Open,
    trigger: Trigger,
    x: f32,
    y: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    tick(
        trigger.name(),
        trigger.text(),
        open.dialog.trigger(trigger),
        true,
        true,
        x,
        y,
    )
    .cursor_pointer()
    .on_click(
        cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
            let Some(open) = this.survey.open.as_mut() else {
                return;
            };
            commit_edit(open);
            open.dropdown = None;
            open.dialog.click_trigger(trigger);
            changed(this, window, cx);
        }),
    )
    .into_any_element()
}

/// A `Button`.
#[allow(clippy::too_many_arguments)]
fn button(
    name: &str,
    text: &str,
    x: f32,
    y: f32,
    w: f32,
    h: f32,
    enabled: bool,
    on_click: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let body = crate::probe::measured(format!("survey-{name}"), at(x, y))
        .id(control_id(name))
        .w(px(w))
        .h(px(h))
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .rounded_sm()
        .whitespace_nowrap()
        .text_size(px(FONT))
        .child(text.to_owned());
    if !enabled {
        return body
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::PANEL))
            .text_color(rgb(theme::DIM))
            .into_any_element();
    }
    body.border_color(rgb(theme::ACCENT))
        .bg(rgb(theme::ACTION))
        .text_color(rgb(theme::ACCENT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                if let Some(open) = this.survey.open.as_mut() {
                    commit_edit(open);
                    open.dropdown = None;
                }
                on_click(this, window, cx);
                changed(this, window, cx);
            }),
        )
        .into_any_element()
}

/// A `ComboBox`: its text and the arrow that opens its list.
fn combo(
    open: &Open,
    which: Combo,
    x: f32,
    y: f32,
    w: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let text = match which {
        Combo::Camera => open.dialog.camera().to_owned(),
        Combo::StartFrom => open.dialog.startfrom().to_owned(),
    };
    crate::probe::measured(format!("survey-{}", which.name()), at(x, y))
        .id(control_id(which.name()))
        .w(px(w))
        .h(px(21.0))
        .px_1()
        .flex()
        .items_center()
        .justify_between()
        .overflow_hidden()
        .whitespace_nowrap()
        .border_1()
        .border_color(rgb(if open.dropdown == Some(which) {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::BG))
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .child(div().overflow_hidden().child(if text.is_empty() {
            " ".to_owned()
        } else {
            text
        }))
        .child(div().text_size(px(8.0)).child("\u{25bc}"))
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                let Some(open) = this.survey.open.as_mut() else {
                    return;
                };
                commit_edit(open);
                open.dropdown = if open.dropdown == Some(which) {
                    None
                } else {
                    Some(which)
                };
                changed(this, window, cx);
            }),
        )
        .into_any_element()
}

/// A combo box's list, under it, over the controls below.
fn dropdown(
    open: &Open,
    which: Combo,
    x: f32,
    y: f32,
    w: f32,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let items: Vec<(String, String)> = match which {
        Combo::Camera => open
            .dialog
            .camera_items()
            .iter()
            .enumerate()
            .map(|(index, name)| (format!("survey-camera-{index}"), name.clone()))
            .collect(),
        Combo::StartFrom => StartPosition::ALL
            .iter()
            .map(|start| {
                (
                    format!("survey-startfrom-{}", start.name()),
                    start.name().to_owned(),
                )
            })
            .collect(),
    };
    let rows = items.into_iter().map(|(id, name)| {
        crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .px_1()
            .h(px(15.0))
            .flex()
            .items_center()
            .whitespace_nowrap()
            .overflow_hidden()
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::ACTION)))
            .child(name.clone())
            .on_click(
                cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                    let Some(open) = this.survey.open.as_mut() else {
                        return;
                    };
                    open.dropdown = None;
                    match which {
                        Combo::Camera => open.dialog.select_camera(&name),
                        Combo::StartFrom => {
                            if let Some(start) = StartPosition::from_name(&name)
                                && open.dialog.select_startfrom(start)
                            {
                                let mut field = TextField::new("");
                                field.set("1");
                                open.prompt = Some(Prompt::Point(field));
                                this.survey.focus.focus(window, cx);
                            }
                        }
                    }
                    changed(this, window, cx);
                }),
            )
    });
    at(x, y)
        .id(SharedString::from(format!("survey-{}-list", which.name())))
        .w(px(w))
        .max_h(px(300.0))
        .overflow_y_scroll()
        .occlude()
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .bg(rgb(theme::PANEL))
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .children(rows)
        .into_any_element()
}

/// `tabSimple`.
/// `// C#: Grid/GridUI.resx (groupBox6, groupBox4, BUT_Accept)`
fn simple_page(
    open: &Open,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let mut page = vec![
        group(
            "Simple Options",
            6.0,
            6.0,
            236.0,
            248.0,
            vec![
                label(6.0, 16.0, "Camera"),
                combo(open, Combo::Camera, 105.0, 13.0, 117.0, cx),
                // label1.Text += " (" + CurrentState.DistanceUnit + ")", GridUI.cs:142
                label(6.0, 42.0, "Altitude (m)"),
                numeric(open, Num::Altitude, 142.0, 40.0, 51.0, window, focus, cx),
                label(6.0, 68.0, "Angle [deg]"),
                numeric(open, Num::Angle, 142.0, 66.0, 51.0, window, focus, cx),
                checkbox(open, Check::CamDirection, 43.0, 92.0, cx),
                // label24.Text += " (" + CurrentState.SpeedUnit + ")", GridUI.cs:143
                label(6.0, 117.0, "Flying Speed (est) (m/s)"),
                numeric(open, Num::FlySpeed, 142.0, 115.0, 51.0, window, focus, cx),
                checkbox(open, Check::UseSpeed, 43.0, 141.0, cx),
                checkbox(open, Check::ToAndLand, 11.0, 171.0, cx),
                checkbox(open, Check::ToAndLandRtl, 29.0, 194.0, cx),
                label(7.0, 214.0, "Split into x segments"),
                numeric(open, Num::Split, 143.0, 212.0, 51.0, window, focus, cx),
            ],
        ),
        group(
            "Display",
            6.0,
            260.0,
            236.0,
            160.0,
            vec![
                checkbox(open, Check::Boundary, 8.0, 20.0, cx),
                checkbox(open, Check::Markers, 8.0, 43.0, cx),
                checkbox(open, Check::Grid, 8.0, 66.0, cx),
                checkbox(open, Check::Internals, 8.0, 89.0, cx),
                checkbox(open, Check::Footprints, 8.0, 112.0, cx),
                checkbox(open, Check::Advanced, 8.0, 135.0, cx),
            ],
        ),
        button(
            "BUT_Accept",
            "Accept",
            84.0,
            587.0,
            75.0,
            23.0,
            true,
            |this, _window, _cx| accept(this),
            cx,
        ),
    ];
    if open.dropdown == Some(Combo::Camera) {
        page.push(dropdown(
            open,
            Combo::Camera,
            6.0 + 105.0,
            6.0 + 13.0 + 21.0,
            130.0,
            cx,
        ));
    }
    page
}

/// `tabGrid`.
/// `// C#: Grid/GridUI.resx (groupBox1, groupBox_copter, groupBox7, groupBoxSpiral)`
fn grid_page(
    open: &Open,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let heading = open.dialog.heading_enabled();
    let mut page = vec![
        group(
            "Grid Options",
            6.0,
            6.0,
            233.0,
            222.0,
            vec![
                label(6.0, 16.0, "Distance between lines [m]"),
                numeric(open, Num::Distance, 169.0, 14.0, 51.0, window, focus, cx),
                label(7.0, 42.0, "OverShoot [m]"),
                numeric(open, Num::Overshoot, 111.0, 40.0, 51.0, window, focus, cx),
                numeric(open, Num::Overshoot2, 168.0, 40.0, 51.0, window, focus, cx),
                label(7.0, 68.0, "LeadIn [m]"),
                numeric(open, Num::Leadin, 111.0, 66.0, 51.0, window, focus, cx),
                numeric(open, Num::Leadin2, 168.0, 66.0, 51.0, window, focus, cx),
                label(5.0, 95.0, "StartFrom"),
                combo(open, Combo::StartFrom, 112.0, 92.0, 107.0, cx),
                label(5.0, 121.0, "Overlap [%]"),
                numeric(open, Num::Overlap, 167.0, 119.0, 51.0, window, focus, cx),
                label(5.0, 147.0, "Sidelap [%]"),
                numeric(open, Num::Sidelap, 167.0, 145.0, 51.0, window, focus, cx),
                checkbox(open, Check::CrossGrid, 8.0, 171.0, cx),
                checkbox(open, Check::Corridor, 92.0, 171.0, cx),
                checkbox(open, Check::Spiral, 169.0, 171.0, cx),
                label(5.0, 197.0, "Corridor Width [m]"),
                numeric(
                    open,
                    Num::CorridorWidth,
                    167.0,
                    195.0,
                    51.0,
                    window,
                    focus,
                    cx,
                ),
            ],
        ),
        group(
            "Copter Options",
            6.0,
            234.0,
            233.0,
            123.0,
            vec![
                label(6.0, 23.0, "Delay at WP (sec)"),
                numeric(open, Num::CopterDelay, 143.0, 21.0, 51.0, window, focus, cx),
                checkbox(open, Check::HeadingHold, 9.0, 50.0, cx),
                textbox(
                    open,
                    Text::HeadingHold,
                    143.0,
                    48.0,
                    31.0,
                    heading && !open.dialog.heading_read_only(),
                    window,
                    focus,
                    cx,
                ),
                button(
                    "BUT_headingholdplus",
                    "+",
                    180.0,
                    46.0,
                    23.0,
                    23.0,
                    heading,
                    |this, _w, _cx| {
                        if let Some(open) = this.survey.open.as_mut() {
                            open.dialog.heading_plus();
                        }
                    },
                    cx,
                ),
                button(
                    "BUT_headingholdminus",
                    "--",
                    204.0,
                    46.0,
                    23.0,
                    23.0,
                    heading,
                    |this, _w, _cx| {
                        if let Some(open) = this.survey.open.as_mut() {
                            open.dialog.heading_minus();
                        }
                    },
                    cx,
                ),
                checkbox(open, Check::HeadingHoldLock, 32.0, 73.0, cx),
                checkbox(open, Check::Spline, 9.0, 96.0, cx),
            ],
        ),
        group(
            "Plane Options",
            6.0,
            363.0,
            233.0,
            88.0,
            vec![
                label(7.0, 19.0, "Alternate Lanes"),
                label(26.0, 40.0, "Min Lane separation"),
                numeric(open, Num::LaneDist, 143.0, 38.0, 51.0, window, focus, cx),
                checkbox(open, Check::OptimizeForDistance, 10.0, 63.0, cx),
            ],
        ),
        group(
            "Spiral Options",
            6.0,
            457.0,
            233.0,
            122.0,
            vec![
                label(7.0, 25.0, "Number of Laps"),
                numeric(open, Num::Laps, 143.0, 23.0, 51.0, window, focus, cx),
                label(7.0, 47.0, "Number of Clockwise Laps"),
                label(18.0, 71.0, "(-1 for Clockwise Spiral)"),
                numeric(
                    open,
                    Num::ClockwiseLaps,
                    143.0,
                    69.0,
                    51.0,
                    window,
                    focus,
                    cx,
                ),
                checkbox(open, Check::MatchSpiralPerimeter, 10.0, 94.0, cx),
            ],
        ),
    ];
    if open.dropdown == Some(Combo::StartFrom) {
        page.push(dropdown(
            open,
            Combo::StartFrom,
            6.0 + 112.0,
            6.0 + 92.0 + 21.0,
            107.0,
            cx,
        ));
    }
    page
}

/// `tabCamera`.
/// `// C#: Grid/GridUI.resx (groupBox2, groupBox3)`
fn camera_page(
    open: &Open,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    vec![
        group(
            "Camera Options",
            6.0,
            6.0,
            231.0,
            298.0,
            vec![
                label(7.0, 21.0, "Focal Length [mm]"),
                numeric(open, Num::FocalLength, 143.0, 19.0, 51.0, window, focus, cx),
                label(7.0, 43.0, "Image Width [Pixels]"),
                textbox(
                    open,
                    Text::ImgWidth,
                    143.0,
                    45.0,
                    51.0,
                    true,
                    window,
                    focus,
                    cx,
                ),
                label(7.0, 69.0, "Image Height [Pixels]"),
                textbox(
                    open,
                    Text::ImgHeight,
                    143.0,
                    71.0,
                    51.0,
                    true,
                    window,
                    focus,
                    cx,
                ),
                label(7.0, 95.0, "Sensor Width [mm]"),
                textbox(
                    open,
                    Text::SensWidth,
                    143.0,
                    97.0,
                    51.0,
                    true,
                    window,
                    focus,
                    cx,
                ),
                label(7.0, 121.0, "Sensor Height [mm]"),
                textbox(
                    open,
                    Text::SensHeight,
                    143.0,
                    123.0,
                    51.0,
                    true,
                    window,
                    focus,
                    cx,
                ),
                // Not ported: a JPEG's EXIF, and writing cameras.xml.
                button(
                    "BUT_samplephoto",
                    "Load Sample Photo",
                    58.0,
                    149.0,
                    110.0,
                    23.0,
                    false,
                    |_, _, _| {},
                    cx,
                ),
                button(
                    "BUT_save",
                    "Save",
                    83.0,
                    178.0,
                    64.0,
                    23.0,
                    false,
                    |_, _, _| {},
                    cx,
                ),
                label(55.0, 204.0, "Calculated Values"),
                label(7.0, 224.0, "cm/pixel"),
                textbox(
                    open,
                    Text::CmPixel,
                    143.0,
                    221.0,
                    51.0,
                    true,
                    window,
                    focus,
                    cx,
                ),
                label(4.0, 250.0, "Field of View Horizontal [m]"),
                textbox(
                    open,
                    Text::FovH,
                    143.0,
                    247.0,
                    51.0,
                    true,
                    window,
                    focus,
                    cx,
                ),
                label(5.0, 277.0, "Field of View Vertical [m]"),
                textbox(
                    open,
                    Text::FovV,
                    143.0,
                    274.0,
                    51.0,
                    true,
                    window,
                    focus,
                    cx,
                ),
            ],
        ),
        group(
            "Trigger Method",
            7.0,
            310.0,
            231.0,
            195.0,
            vec![
                radio(open, Trigger::Distance, 9.0, 19.0, cx),
                checkbox(open, Check::StopStart, 134.0, 19.0, cx),
                radio(open, Trigger::Digicam, 9.0, 42.0, cx),
                radio(open, Trigger::RepeatServo, 9.0, 65.0, cx),
                label(6.0, 85.0, "Servo"),
                label(63.0, 85.0, "PWM"),
                label(120.0, 85.0, "Cycle Time [s]"),
                numeric(open, Num::ReptServo, 9.0, 101.0, 51.0, window, focus, cx),
                numeric(open, Num::ReptPwm, 66.0, 101.0, 51.0, window, focus, cx),
                numeric(open, Num::ReptTime, 123.0, 101.0, 51.0, window, focus, cx),
                radio(open, Trigger::SetServo, 9.0, 127.0, cx),
                label(6.0, 148.0, "Servo"),
                label(64.0, 147.0, "PWM L"),
                label(120.0, 147.0, "PWM H"),
                numeric(open, Num::SetServoNo, 9.0, 164.0, 51.0, window, focus, cx),
                numeric(open, Num::SetServoLow, 66.0, 164.0, 51.0, window, focus, cx),
                numeric(
                    open,
                    Num::SetServoHigh,
                    123.0,
                    164.0,
                    51.0,
                    window,
                    focus,
                    cx,
                ),
            ],
        ),
    ]
}

/// `groupBox5`, Stats: four columns of captions and values at the `.resx` locations.
/// `// C#: Grid/GridUI.resx (groupBox5)`
fn stats(open: &Open) -> AnyElement {
    const CELLS: [(Stat, &str, f32, f32, f32); 13] = [
        (Stat::Area, "Area: ", 13.0, 120.0, 20.0),
        (Stat::Distance, "Distance: ", 13.0, 120.0, 33.0),
        (Stat::Spacing, "Dist between images: ", 13.0, 120.0, 46.0),
        (
            Stat::GroundResolution,
            "Ground Resolution: ",
            13.0,
            120.0,
            59.0,
        ),
        (Stat::Pictures, "Pictures:", 197.0, 299.0, 20.0),
        (Stat::Strips, "No of Strips:", 197.0, 299.0, 33.0),
        (Stat::Footprint, "Footprint:", 197.0, 299.0, 46.0),
        (
            Stat::DistBetweenLines,
            "Dist between lines:",
            197.0,
            299.0,
            59.0,
        ),
        (Stat::FlightTime, "Flight Time (est):", 382.0, 480.0, 20.0),
        (Stat::PhotoEvery, "Photo every (est):", 382.0, 480.0, 33.0),
        (Stat::TurnRadius, "Turn Dia (at 45d):", 382.0, 480.0, 46.0),
        (
            Stat::GroundElevation,
            "Ground Elevation:",
            382.0,
            480.0,
            59.0,
        ),
        (Stat::MinShutter, "Min Shutter Speed:", 559.0, 657.0, 20.0),
    ];
    let mut cells = Vec::new();
    for (stat, caption, x, value_x, y) in CELLS {
        cells.push(label(x, y, caption));
        cells.push(
            crate::probe::measured(format!("survey-{}", stat.name()), at(value_x, y))
                .whitespace_nowrap()
                .text_size(px(FONT))
                .text_color(rgb(theme::ACCENT))
                .child(open.dialog.stat(stat).to_owned())
                .into_any_element(),
        );
    }
    // Docked along the bottom, so as wide as the map above it.
    div()
        .relative()
        .flex_shrink_0()
        .h(px(80.0))
        .overflow_hidden()
        .child(
            div()
                .absolute()
                .left(px(0.0))
                .top(px(6.0))
                .w_full()
                .h(px(74.0))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .rounded_sm(),
        )
        .child(
            at(6.0, 0.0)
                .px_1()
                .bg(rgb(theme::BG))
                .text_size(px(FONT))
                .text_color(rgb(theme::DIM))
                .child("Stats"),
        )
        .children(cells)
        .into_any_element()
}

/// The box over the dialog: "Enter point #", or a message.
fn prompt_box(
    open: &Open,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let prompt = open.prompt.as_ref()?;
    let (title, text, field) = match prompt {
        Prompt::Point(field) => (
            "Enter point #",
            "Please enter a boundary point number".to_owned(),
            Some(field),
        ),
        Prompt::Message(text, caption) => (*caption, text.clone(), None),
    };
    let ok = crate::ui::action(
        "survey-prompt-ok",
        "OK",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), window, cx| {
            if let Some(open) = this.survey.open.as_mut() {
                match open.prompt.take() {
                    Some(Prompt::Point(field)) => open.dialog.answer_point(Some(field.value())),
                    Some(Prompt::Message(..)) | None => {}
                }
            }
            changed(this, window, cx);
        }),
    );
    let mut buttons = div().flex().justify_end().gap_2().child(ok);
    if field.is_some() {
        buttons = buttons.child(crate::ui::action(
            "survey-prompt-cancel",
            "Cancel",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                if let Some(open) = this.survey.open.as_mut()
                    && let Some(Prompt::Point(_)) = open.prompt.take()
                {
                    open.dialog.answer_point(None);
                }
                changed(this, window, cx);
            }),
        ));
    }
    let mut body = crate::probe::measured("survey-prompt", div())
        .id("survey-prompt")
        .flex()
        .flex_col()
        .gap_2()
        .w(px(300.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .occlude()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(title.to_owned()),
        )
        .child(div().text_sm().text_color(rgb(theme::TEXT)).child(text));
    if let Some(field) = field {
        let focused = focus.is_focused(window);
        body = body.child(
            div()
                .id("survey-prompt-field")
                .track_focus(focus)
                .on_key_down(cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
                    key(this, event, window, cx)
                }))
                .px_2()
                .py_1()
                .border_1()
                .border_color(rgb(if focused {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(theme::ACTION))
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(field.value().to_owned()),
        );
    }
    Some(
        div()
            .absolute()
            .left(px(0.0))
            .top(px(0.0))
            .size_full()
            .flex()
            .items_center()
            .justify_center()
            .child(body.child(buttons))
            .into_any_element(),
    )
}

/// The dialog's form, in place of the planning screen's sidebar and map while it shows.
pub fn form(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let open = this.survey.open.as_ref()?;
    let focus = &this.survey.focus;

    // tabControl1's headers: the pages it holds, as the flight screen's strip draws them.
    let mut headers = div().flex().gap_1().items_end();
    for tab in open.dialog.tabs() {
        let tab = *tab;
        let selected = tab == open.tab;
        headers = headers.child(
            crate::probe::measured(format!("survey-{}", tab.name()), div())
                .id(control_id(tab.name()))
                .px_2()
                .py_1()
                .rounded_t_md()
                .text_xs()
                .cursor_pointer()
                .bg(rgb(if selected { theme::PANEL } else { theme::BG }))
                .text_color(rgb(if selected { theme::ACCENT } else { theme::DIM }))
                .border_b_2()
                .border_color(rgb(if selected { theme::ACCENT } else { theme::BG }))
                .hover(|style| style.text_color(rgb(theme::TEXT)))
                .child(tab.text())
                .on_click(
                    cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                        if let Some(open) = this.survey.open.as_mut() {
                            commit_edit(open);
                            open.dropdown = None;
                            open.tab = tab;
                        }
                        changed(this, window, cx);
                    }),
                ),
        );
    }
    let page = match open.tab {
        Tab::Simple => simple_page(open, window, focus, cx),
        Tab::Grid => grid_page(open, window, focus, cx),
        Tab::Camera => camera_page(open, window, focus, cx),
    };

    let map = this.map.clone();
    let map_region = crate::probe::measured("survey-map", div())
        .id("survey-map")
        .relative()
        .flex()
        .flex_1()
        .min_h(px(0.0))
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseDownEvent, window, cx| {
                if let Some(open) = this.survey.open.as_mut() {
                    commit_edit(open);
                    open.dropdown = None;
                }
                this.map
                    .borrow_mut()
                    .begin_drag(f32::from(event.position.x), f32::from(event.position.y));
                changed(this, window, cx);
            }),
        )
        .on_mouse_move(
            cx.listener(|this, event: &gpui::MouseMoveEvent, window, _cx| {
                if event.pressed_button != Some(MouseButton::Left) {
                    return;
                }
                this.map
                    .borrow_mut()
                    .drag_to(f32::from(event.position.x), f32::from(event.position.y));
                window.refresh();
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, _event: &gpui::MouseUpEvent, window, _cx| {
                this.map.borrow_mut().end_drag();
                window.refresh();
            }),
        )
        .on_scroll_wheel(move |event, window, _cx| {
            let delta = event.delta.pixel_delta(px(20.0));
            map.borrow_mut().zoom(
                f32::from(event.position.x),
                f32::from(event.position.y),
                f32::from(delta.y) / 20.0,
            );
            window.refresh();
        })
        .child(crate::mapview::map_element(this.map.clone()));

    let tab_control = div()
        .flex()
        .flex_col()
        .flex_shrink_0()
        .w(px(253.0))
        .min_h(px(0.0))
        .child(
            div()
                .flex()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .child(headers),
        )
        .child(
            div()
                .id("survey-page")
                .flex_1()
                .min_h(px(0.0))
                .overflow_y_scroll()
                .bg(rgb(theme::PANEL))
                .child(div().relative().w(px(245.0)).h(px(619.0)).children(page)),
        );

    let title = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .bg(rgb(theme::PANEL))
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child("Survey (Grid)"),
        )
        .child(
            crate::probe::measured("survey-close", div())
                .id("survey-close")
                .px_2()
                .cursor_pointer()
                .text_color(rgb(theme::DIM))
                .hover(|style| style.text_color(rgb(theme::ALERT)))
                .child("\u{2715}")
                .on_click(cx.listener(|this, _event: &gpui::ClickEvent, window, cx| {
                    close(this);
                    window.refresh();
                    cx.notify();
                })),
        );

    Some(
        crate::probe::measured("panel:Survey (Grid)", div())
            .relative()
            .flex()
            .flex_col()
            .flex_1()
            .min_h(px(0.0))
            .min_w(px(0.0))
            .bg(rgb(theme::BG))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .child(title)
            .child(
                div()
                    .flex()
                    .flex_1()
                    .min_h(px(0.0))
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .flex_1()
                            .min_w(px(0.0))
                            .child(map_region)
                            .child(stats(open)),
                    )
                    .child(tab_control),
            )
            .children(prompt_box(open, window, focus, cx))
            .into_any_element(),
    )
}

/// Facts a UI test asserts on: whether the dialog shows, its tabs, every control as it reads,
/// the Stats, the grid's size, the box over it, and how many rows the last Accept added.
pub fn record_facts(survey: &SurveyUi) {
    use crate::facts::record;
    record(
        "survey.dialog",
        if survey.open.is_some() {
            "open"
        } else {
            "closed"
        },
    );
    record("survey.dialog.added", survey.added);
    let Some(open) = survey.open.as_ref() else {
        return;
    };
    let dialog = &open.dialog;
    record("survey.dialog.tab", open.tab.name());
    record(
        "survey.dialog.tabs",
        dialog
            .tabs()
            .iter()
            .map(|tab| tab.name())
            .collect::<Vec<_>>()
            .join(","),
    );
    for num in Num::ALL {
        record(
            format!("survey.dialog.{}", num.name()),
            dialog.num_text(num),
        );
    }
    for check in Check::ALL {
        record(
            format!("survey.dialog.{}", check.name()),
            bool_text(dialog.check(check)),
        );
    }
    for trigger in Trigger::ALL {
        record(
            format!("survey.dialog.{}", trigger.name()),
            bool_text(dialog.trigger(trigger)),
        );
    }
    for text in Text::ALL {
        record(format!("survey.dialog.{}", text.name()), dialog.text(text));
    }
    record("survey.dialog.CMB_camera", dialog.camera());
    record("survey.dialog.CMB_startfrom", dialog.startfrom());
    record("survey.dialog.cameras", dialog.camera_items().len());
    for stat in Stat::ALL {
        record(format!("survey.dialog.{}", stat.name()), dialog.stat(stat));
    }
    record("survey.dialog.grid.points", dialog.grid().len());
    record("survey.dialog.recomputes", dialog.recomputes());
    record("survey.dialog.heading.enabled", dialog.heading_enabled());
    record(
        "survey.dialog.editing",
        open.editing
            .as_ref()
            .map_or("none", |(field, _)| field.name()),
    );
    record(
        "survey.dialog.dropdown",
        open.dropdown.map_or("none", Combo::name),
    );
    let (prompt, text) = match &open.prompt {
        None => ("none", String::new()),
        Some(Prompt::Point(field)) => ("Enter point #", field.value().to_owned()),
        Some(Prompt::Message(text, caption)) => (
            if caption.is_empty() {
                "message"
            } else {
                caption
            },
            text.clone(),
        ),
    };
    record("survey.dialog.prompt", prompt);
    record("survey.dialog.prompt.text", text);
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mission::gridui::Context as GridContext;

    fn square() -> Vec<LatLon> {
        [
            (-35.3600, 149.1600),
            (-35.3600, 149.1710),
            (-35.3690, 149.1710),
            (-35.3690, 149.1600),
        ]
        .into_iter()
        .map(|(lat, lng)| LatLon::new(lat, lng).expect("a vertex"))
        .collect()
    }

    /// The dialog the planning screen's script opens: the square, the SITL home, a copter.
    fn gui_dialog() -> Dialog {
        Dialog::open(
            &square(),
            Cameras::builtin(),
            &GridContext {
                home: (-35.363_262_1, 149.165_237_4, 584.1),
                plane: false,
                start_point: LatLon::default(),
            },
            &|_| None,
        )
    }

    const COPTER: Fill = Fill {
        frame: AltitudeFrame::Relative,
        copter: true,
        plane: false,
    };

    /// The script's path: the angle and the altitude typed, Accept, and the rows in the plan -
    /// the golden `accept_gui_angle_alt`'s commands, filled as the planner fills them.
    #[test]
    fn accept_puts_the_golden_rows_in_the_plan() {
        let mut dialog = gui_dialog();
        dialog.type_num(Num::Angle, "45");
        dialog.type_num(Num::Altitude, "120");
        let accepted = dialog
            .accept(&gridui::Planner::default())
            .expect("a grid to accept");
        let mut plan = Plan::default();
        assert_eq!(apply_steps(&mut plan, &accepted.steps, &COPTER), None);
        let golden = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata/grid/golden/accept/accept_gui_angle_alt.csv"),
        )
        .expect("the golden");
        let commands = golden
            .lines()
            .find_map(|line| line.strip_prefix("commands,"))
            .expect("commands");
        let ours: Vec<String> = plan
            .items()
            .iter()
            .map(|item| item.command.to_string())
            .collect();
        assert_eq!(ours.join(" "), commands);
        assert_eq!(plan.items().len(), 115);
        // The takeoff as the C# fills a TAKEOFF: its numbers as given.
        let takeoff = plan.items()[0];
        assert_eq!(
            (takeoff.command, takeoff.param1, takeoff.z),
            (22, 20.0, 30.0)
        );
        // A waypoint at the dialog's altitude, its position to seven decimals.
        let waypoint = plan.items()[1];
        assert_eq!(waypoint.command, 16);
        assert_eq!(waypoint.z, 120.0);
        assert_eq!(
            format!("{:.7}", waypoint.x).parse::<f64>().ok(),
            Some(waypoint.x)
        );
        assert_eq!(waypoint.frame, AltitudeFrame::Relative.mav_frame());
        let last = plan.items()[114];
        assert_eq!(last.command, 20);
    }

    /// Verify Height reaches the grid's rows as it reaches a map click's: with the box ticked
    /// and no terrain under the row, an Absolute row is the ground (0) plus Default Alt rather
    /// than the altitude the grid passed.
    #[test]
    fn verify_height_applies_to_the_grids_rows() {
        let mut plan = Plan::default();
        plan.set_panel_text(crate::plan::PanelBox::DefaultAlt, "50");
        let call = Call {
            command: cmd::WAYPOINT,
            params: [0.0, 0.0, 0.0, 0.0],
            x: 149.16,
            y: -35.36,
            z: 100.0,
        };
        let fill = Fill {
            frame: AltitudeFrame::Absolute,
            copter: false,
            plane: true,
        };
        let (unverified, _) = fill_row(&plan, &call, &fill);
        plan.set_verify_height(true);
        let (verified, _) = fill_row(&plan, &call, &fill);
        assert_eq!(
            verified.z,
            plan.verified_altitude(verified.x, verified.y, unverified.z, fill.frame)
        );
        assert_ne!(
            verified.z, unverified.z,
            "the box changes an Absolute row's altitude"
        );
        assert_eq!(verified.z, 50.0, "ground 0 plus Default Alt");
    }

    /// Spline on the planner turns the grid's waypoints into spline waypoints; a delay is rounded
    /// to a tenth under a copter's "Delay" header and kept as given under a plane's.
    #[test]
    fn filling_follows_the_planner() {
        let mut plan = Plan::default();
        let call = Call {
            command: cmd::WAYPOINT,
            params: [2.25, 0.0, 0.0, 0.0],
            x: 149.160_117_973_968_54,
            y: -35.360_000_005_311_164,
            z: 100.0,
        };
        let (row, _) = fill_row(&plan, &call, &COPTER);
        assert_eq!(row.param1, 2.2);
        assert_eq!(row.x, -35.36);
        assert_eq!(row.y, 149.160_118);
        let (row, _) = fill_row(
            &plan,
            &call,
            &Fill {
                plane: true,
                ..COPTER
            },
        );
        assert_eq!(row.param1, 2.25);
        plan.set_spline(true);
        let (row, _) = fill_row(&plan, &call, &COPTER);
        assert_eq!(row.command, cmd::SPLINE_WAYPOINT);
        // A DO command's numbers as they come.
        let trigger = Call {
            command: cmd::DO_SET_CAM_TRIGG_DIST,
            params: [46.2, 0.0, 1.0, 0.0],
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let (row, _) = fill_row(&plan, &trigger, &COPTER);
        assert_eq!((row.command, row.param1, row.param3), (206, 46.2, 1.0));
    }

    /// The split's jumps go in at the top of the grid, before the rows already there, pointing at
    /// each segment's takeoff.
    #[test]
    fn a_split_inserts_its_jumps_at_the_top() {
        let mut dialog = gui_dialog();
        dialog.type_num(Num::Split, "2");
        let mut plan = Plan::default();
        plan.append(mp_mission::commands::waypoint(square()[0], 50.0, 3));
        let accepted = dialog
            .accept(&gridui::Planner {
                rows: plan.items().len(),
                ..gridui::Planner::default()
            })
            .expect("a grid");
        apply_steps(&mut plan, &accepted.steps, &COPTER);
        let first: Vec<u16> = plan
            .items()
            .iter()
            .take(3)
            .map(|item| item.command)
            .collect();
        assert_eq!(first, [cmd::DO_JUMP, cmd::DO_JUMP, cmd::WAYPOINT]);
        // The first segment's takeoff was row 1, after the row already there; two jumps above it
        // make it row 3, waypoint 4.
        assert_eq!(plan.items()[0].param1, 4.0);
        assert_eq!(plan.items()[3].command, cmd::TAKEOFF);
    }

    /// Every fact `tests/gui/plan-survey.gui` asserts on is one [`record_facts`] records, and
    /// every control it clicks is one the form draws: a renamed fact or id fails here rather than
    /// as a script that cannot find what it looks for.
    #[test]
    fn the_gui_script_names_facts_and_controls_the_dialog_has() {
        let script = include_str!("../../../tests/gui/plan-survey.gui");
        let mut names: Vec<String> = [
            "added",
            "tab",
            "tabs",
            "CMB_camera",
            "CMB_startfrom",
            "cameras",
            "grid.points",
            "recomputes",
            "heading.enabled",
            "editing",
            "dropdown",
            "prompt",
            "prompt.text",
        ]
        .map(ToOwned::to_owned)
        .to_vec();
        names.extend(Num::ALL.map(|num| num.name().to_owned()));
        names.extend(Check::ALL.map(|check| check.name().to_owned()));
        names.extend(Trigger::ALL.map(|trigger| trigger.name().to_owned()));
        names.extend(Text::ALL.map(|text| text.name().to_owned()));
        names.extend(Stat::ALL.map(|stat| stat.name().to_owned()));
        let mut ids: Vec<String> = names.clone();
        ids.extend(
            ["tabSimple", "tabGrid", "tabCamera", "close", "BUT_Accept"].map(ToOwned::to_owned),
        );
        let source = include_str!("survey_ui.rs");
        let (mut facts, mut clicks) = (0, 0);
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("survey.dialog") => {
                    let name = key.strip_prefix("survey.dialog.").unwrap_or("");
                    assert!(
                        key == "survey.dialog" || names.iter().any(|known| known == name),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("survey-") => {
                    let name = id.strip_prefix("survey-").unwrap_or("");
                    let base = name
                        .strip_suffix("-up")
                        .or_else(|| name.strip_suffix("-down"))
                        .unwrap_or(name);
                    assert!(ids.iter().any(|known| known == base), "{id} is not drawn");
                    // NUM_spacing is the one control the form leaves out: `Visible` False.
                    assert_ne!(base, Num::Spacing.name(), "{id} is hidden");
                    if !name.contains('_') && !name.starts_with("tab") {
                        assert!(source.contains(&format!("\"survey-{name}\"")), "{id}");
                    }
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 40 && clicks > 8, "{facts} facts, {clicks} clicks");
    }

    #[test]
    fn the_preview_follows_the_display_boxes() {
        let mut dialog = gui_dialog();
        let (items, boundary) = preview(&dialog);
        assert_eq!(items.len(), 80);
        assert_eq!(boundary.len(), 4);
        dialog.click_check(Check::Boundary);
        dialog.click_check(Check::Grid);
        dialog.click_check(Check::Markers);
        let (items, boundary) = preview(&dialog);
        assert!(items.is_empty() && boundary.is_empty());
    }
}
