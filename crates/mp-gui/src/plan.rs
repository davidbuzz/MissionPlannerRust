//! The plan screen: building a mission and moving it to and from the aircraft.
//!
//! Equivalent to Mission Planner's Flight Plan tab. The plan held here is the operator's, and it
//! is deliberately distinct from whatever the vehicle holds: the two are only equal immediately
//! after a read or a successful write, and pretending otherwise is how people fly a mission they
//! thought they had replaced.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_mavlink_dialects::all::MavCmd;
use mp_mission::validate::{Context as ValidationContext, validate_with};
use mp_mission::{GridOptions, MissionItem, Severity, grid};
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, panel, progress, theme};

/// `MAV_CMD_NAV_WAYPOINT`, the command a click on the map creates.
pub const CMD_WAYPOINT: u16 = 16;
/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`: altitude above home, which is what every pilot means.
pub const FRAME_RELATIVE: u8 = 3;
/// Altitude given to a waypoint created by clicking the map, in metres above home.
pub const DEFAULT_ALTITUDE: f64 = 50.0;
/// `MAV_CMD_NAV_RETURN_TO_LAUNCH`, which takes no position.
const CMD_RTL: u16 = 20;
/// `MAV_CMD_NAV_LAND` with no coordinates: land where the vehicle is.
const CMD_LAND_NO_POSITION: u16 = 21;

/// What a click on the map does.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum DrawMode {
    /// Add a waypoint.
    #[default]
    Waypoints,
    /// Add a vertex to the survey area.
    Area,
}

/// The mission being edited, separate from the vehicle's.
#[derive(Debug, Default)]
pub struct Plan {
    items: Vec<MissionItem>,
    /// Where the current contents came from, for the header line.
    origin: Origin,
    /// The item the operator has selected, if any.
    selected: Option<u16>,
    /// What a click on the map does.
    draw_mode: DrawMode,
    /// The survey area, in the order its vertices were drawn.
    polygon: Vec<LatLon>,
    /// How the survey should be flown.
    survey: GridOptions,
    /// Why the last survey could not be generated, if it could not.
    survey_error: Option<String>,
}

/// Where the plan on screen came from.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub enum Origin {
    /// Nothing has been loaded or drawn.
    #[default]
    Empty,
    /// Read back from the aircraft.
    Vehicle,
    /// Drawn on the map.
    Edited,
    /// Loaded from a file.
    File(String),
}

impl Origin {
    fn label(&self) -> String {
        match self {
            Self::Empty => "empty".to_owned(),
            Self::Vehicle => "read from the vehicle".to_owned(),
            Self::Edited => "edited here, not yet written".to_owned(),
            Self::File(name) => format!("loaded from {name}"),
        }
    }
}

impl Plan {
    /// The items, in sequence order.
    #[must_use]
    pub fn items(&self) -> &[MissionItem] {
        &self.items
    }

    /// Whether anything is planned.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Where the contents came from.
    #[must_use]
    pub fn origin(&self) -> &Origin {
        &self.origin
    }

    /// The selected item's sequence number.
    #[must_use]
    pub const fn selected(&self) -> Option<u16> {
        self.selected
    }

    /// Selects an item.
    pub fn select(&mut self, seq: Option<u16>) {
        self.selected = seq;
    }

    /// Replaces the plan with what the vehicle reported.
    pub fn adopt_from_vehicle(&mut self, items: Vec<MissionItem>) {
        self.items = items;
        self.origin = Origin::Vehicle;
        self.selected = None;
    }

    /// Replaces the plan with the contents of a file.
    pub fn adopt_from_file(&mut self, name: impl Into<String>, items: Vec<MissionItem>) {
        self.items = items;
        self.origin = Origin::File(name.into());
        self.selected = None;
    }

    /// Appends a waypoint at a position.
    ///
    /// Sequence numbers are reassigned from scratch rather than incremented, because a mission
    /// with a gap in its sequence is rejected by the vehicle at upload time - a failure that
    /// happens minutes after the mistake, with no indication of which edit caused it.
    pub fn add_waypoint(&mut self, position: LatLon, altitude: f64) {
        self.items.push(MissionItem {
            seq: 0,
            current: 0,
            frame: FRAME_RELATIVE,
            command: CMD_WAYPOINT,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: position.latitude(),
            y: position.longitude(),
            z: altitude,
            autocontinue: 1,
        });
        self.renumber();
        self.origin = Origin::Edited;
    }

    /// Removes an item.
    pub fn remove(&mut self, seq: u16) {
        self.items.retain(|item| item.seq != seq);
        self.renumber();
        self.origin = Origin::Edited;
        if self.selected == Some(seq) {
            self.selected = None;
        }
    }

    /// Moves an item up or down the list.
    pub fn move_item(&mut self, seq: u16, delta: i32) {
        let Some(index) = self.items.iter().position(|item| item.seq == seq) else {
            return;
        };
        let Ok(index) = i32::try_from(index) else {
            return;
        };
        let target = index + delta;
        let Ok(target) = usize::try_from(target) else {
            return;
        };
        let Ok(index) = usize::try_from(index) else {
            return;
        };
        if target >= self.items.len() {
            return;
        }
        self.items.swap(index, target);
        self.renumber();
        self.origin = Origin::Edited;
        self.selected = u16::try_from(target).ok();
    }

    /// Changes an item's altitude by a step, clamped so a held button cannot run away.
    ///
    /// Negative altitudes are allowed: a relative-frame waypoint below home is meaningful on a
    /// vehicle launched from a cliff or a rooftop, and refusing it would be the ground station
    /// deciding it knows the terrain better than the operator.
    pub fn nudge_altitude(&mut self, seq: u16, delta: f64) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        item.z = (item.z + delta).clamp(-MAX_STEPPED_ALTITUDE, MAX_STEPPED_ALTITUDE);
        self.origin = Origin::Edited;
    }

    /// Changes what an item does.
    ///
    /// Return-to-launch and land take no position, so changing to one zeroes the coordinates.
    /// Leaving stale coordinates on a command that ignores them is how a mission looks right on
    /// the map and flies somewhere else: the map would keep drawing a waypoint the vehicle has no
    /// intention of visiting.
    pub fn set_command(&mut self, seq: u16, command: u16) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        item.command = command;
        if matches!(command, CMD_RTL | CMD_LAND_NO_POSITION) {
            item.x = 0.0;
            item.y = 0.0;
        }
        self.origin = Origin::Edited;
    }

    /// Discards everything, including the survey area.
    pub fn clear(&mut self) {
        self.items.clear();
        self.origin = Origin::Empty;
        self.selected = None;
        self.polygon.clear();
        self.survey_error = None;
    }

    /// What a click on the map does.
    #[must_use]
    pub const fn draw_mode(&self) -> DrawMode {
        self.draw_mode
    }

    /// Changes what a click on the map does.
    pub fn set_draw_mode(&mut self, mode: DrawMode) {
        self.draw_mode = mode;
    }

    /// The survey area's vertices.
    #[must_use]
    pub fn polygon(&self) -> &[LatLon] {
        &self.polygon
    }

    /// Adds a vertex to the survey area.
    pub fn add_area_vertex(&mut self, position: LatLon) {
        self.polygon.push(position);
        self.survey_error = None;
    }

    /// Removes the last vertex, which is the undo an operator reaches for while drawing.
    pub fn undo_area_vertex(&mut self) {
        self.polygon.pop();
        self.survey_error = None;
    }

    /// Discards the survey area, leaving the mission alone.
    pub fn clear_area(&mut self) {
        self.polygon.clear();
        self.survey_error = None;
    }

    /// How the survey should be flown.
    #[must_use]
    pub const fn survey_options(&self) -> GridOptions {
        self.survey
    }

    /// Adjusts the survey settings, keeping each within a range that produces a flyable grid.
    ///
    /// Spacing has a floor because a survey at one metre spacing over a field generates tens of
    /// thousands of waypoints, which no autopilot will accept and no operator intended. The angle
    /// wraps rather than clamps: a bearing is circular, and stopping at 359 would be arbitrary.
    pub fn adjust_survey(&mut self, spacing: f64, angle: f64, altitude: f64) {
        self.survey.spacing = (self.survey.spacing + spacing).clamp(5.0, 1000.0);
        self.survey.angle = (self.survey.angle + angle).rem_euclid(360.0);
        self.survey.altitude = (self.survey.altitude + altitude).clamp(1.0, MAX_STEPPED_ALTITUDE);
        self.survey_error = None;
    }

    /// Why the last survey could not be generated.
    #[must_use]
    pub fn survey_error(&self) -> Option<&str> {
        self.survey_error.as_deref()
    }

    /// Replaces the mission with a lawnmower pattern covering the survey area.
    ///
    /// Replaces rather than appends. A survey joined onto an existing mission would fly the old
    /// waypoints first and then transit to the area, which is almost never what was meant, and
    /// the operator who wanted that can save the two separately.
    pub fn generate_survey(&mut self) {
        match grid(&self.polygon, &self.survey) {
            Ok(positions) if positions.is_empty() => {
                self.survey_error = Some(
                    "the area is smaller than one line spacing; reduce the spacing".to_owned(),
                );
            }
            Ok(positions) => {
                let altitude = self.survey.altitude;
                self.items = positions
                    .into_iter()
                    .map(|position| MissionItem {
                        seq: 0,
                        current: 0,
                        frame: FRAME_RELATIVE,
                        command: CMD_WAYPOINT,
                        param1: 0.0,
                        param2: 0.0,
                        param3: 0.0,
                        param4: 0.0,
                        x: position.latitude(),
                        y: position.longitude(),
                        z: altitude,
                        autocontinue: 1,
                    })
                    .collect();
                self.renumber();
                self.origin = Origin::Edited;
                self.selected = None;
                self.survey_error = None;
            }
            Err(err) => self.survey_error = Some(err.to_string()),
        }
    }

    /// Renumbers items 0..n so the sequence has no gaps.
    fn renumber(&mut self) {
        for (index, item) in self.items.iter_mut().enumerate() {
            item.seq = u16::try_from(index).unwrap_or(u16::MAX);
        }
    }
}

/// The commands the editor offers, and what their altitude means.
///
/// Not every `MAV_CMD` - there are hundreds, most of which belong in a full command editor rather
/// than a list a pilot scans while planning. These are the navigation commands that make up the
/// shape of a mission, which is what the map shows.
pub const EDITABLE_COMMANDS: &[(u16, &str)] = &[
    (16, "waypoint"),
    (22, "takeoff"),
    (21, "land"),
    (20, "return to launch"),
    (19, "loiter for time"),
    (17, "loiter unlimited"),
    (18, "loiter turns"),
    (82, "spline waypoint"),
];

/// Altitude steps offered, in metres, with the name a test script clicks them by.
///
/// Coarse and fine, because both are needed and neither alone is enough: 10 m steps make setting a
/// survey height quick, and 1 m steps matter near the ground.
pub const ALTITUDE_STEPS: [(f64, &str); 4] = [
    (-10.0, "alt-minus-10"),
    (-1.0, "alt-minus-1"),
    (1.0, "alt-plus-1"),
    (10.0, "alt-plus-10"),
];

/// The highest altitude the stepper will reach, in metres above home.
///
/// Not a limit on what can be flown - a mission loaded from a file keeps whatever it holds. It
/// stops a held button from walking a waypoint into the stratosphere, which is a data entry
/// accident rather than a decision.
pub const MAX_STEPPED_ALTITUDE: f64 = 1000.0;

/// A short name for a mission command.
///
/// The generated enum gives `MAV_CMD_NAV_WAYPOINT`; a table of those is unreadable, so the common
/// prefix is stripped. An unknown command shows its number rather than a guess.
fn command_label(command: u16) -> String {
    MavCmd(u32::from(command)).name().map_or_else(
        || format!("cmd {command}"),
        |name| {
            name.strip_prefix("MAV_CMD_NAV_")
                .or_else(|| name.strip_prefix("MAV_CMD_DO_"))
                .or_else(|| name.strip_prefix("MAV_CMD_CONDITION_"))
                .or_else(|| name.strip_prefix("MAV_CMD_"))
                .unwrap_or(name)
                .to_lowercase()
        },
    )
}

/// Move and delete, shown only on the selected row.
///
/// Always-visible controls on every row turn a list into a minefield: the click that selects an
/// item is a pixel away from the click that deletes it. Requiring a selection first means a
/// destructive action always takes two deliberate clicks.
fn row_controls(seq: u16, selected: bool, cx: &mut Context<MissionPlanner>) -> Vec<AnyElement> {
    if !selected {
        return Vec::new();
    }

    let small = |id: (&'static str, usize), label: &'static str, colour: u32| {
        div()
            .id(id)
            .px_1()
            .rounded_sm()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(colour))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(label)
    };

    vec![
        small(("plan-up", usize::from(seq)), "up", theme::TEXT)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.move_item(seq, -1);
                this.sync_map_mission();
                cx.notify();
            }))
            .into_any_element(),
        small(("plan-down", usize::from(seq)), "down", theme::TEXT)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.move_item(seq, 1);
                this.sync_map_mission();
                cx.notify();
            }))
            .into_any_element(),
        small(("plan-delete", usize::from(seq)), "delete", theme::ALERT)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.remove(seq);
                this.sync_map_mission();
                cx.notify();
            }))
            .into_any_element(),
    ]
}

/// The waypoint table.
pub fn items_panel(
    plan_items: &[MissionItem],
    selected: Option<u16>,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let mut rows = div().flex().flex_col();

    rows = rows.child(
        div()
            .flex()
            .gap_2()
            .pb_1()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .child(div().w(px(28.0)).child("#"))
            .child(div().w(px(120.0)).child("command"))
            .child(div().w(px(150.0)).child("position"))
            .child(div().w(px(70.0)).child("alt")),
    );

    if plan_items.is_empty() {
        rows =
            rows.child(
                div().pt_2().text_xs().text_color(rgb(theme::DIM)).child(
                    "no items - click the map to add a waypoint, or read one from the vehicle",
                ),
            );
    }

    for item in plan_items {
        let seq = item.seq;
        let is_selected = selected == Some(seq);
        let position = match item.position() {
            Ok(Some(position)) => {
                format!("{:.6}, {:.6}", position.latitude(), position.longitude())
            }
            // A non-navigation command carries parameters in x and y, not coordinates. Showing
            // 0.000000, 0.000000 for a DO_SET_SERVO would be a lie that looks like a bad waypoint.
            Ok(None) => "-".to_owned(),
            Err(_) => "invalid".to_owned(),
        };
        rows = rows.child(
            crate::probe::measured(format!("plan-row-{seq}"), div())
                .id(("plan-row", usize::from(seq)))
                .flex()
                .gap_2()
                .py_1()
                .text_xs()
                .cursor_pointer()
                .text_color(rgb(if is_selected {
                    theme::ACCENT
                } else {
                    theme::TEXT
                }))
                .bg(rgb(if is_selected {
                    theme::ACTION
                } else {
                    theme::PANEL
                }))
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(
                    div()
                        .w(px(28.0))
                        .text_color(rgb(theme::DIM))
                        .child(seq.to_string()),
                )
                .child(div().w(px(120.0)).child(command_label(item.command)))
                .child(div().w(px(150.0)).child(position))
                .child(div().w(px(70.0)).child(format!("{:.0} m", item.z)))
                .children(row_controls(seq, is_selected, cx))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    // Clicking the selected row clears the selection, so there is always a way
                    // back to "nothing selected" without hunting for empty space.
                    let already = this.plan.selected() == Some(seq);
                    this.plan.select(if already { None } else { Some(seq) });
                    cx.notify();
                })),
        );
    }

    // Scrolls, rather than clipping at a fixed height. A survey over a modest area generates
    // dozens of waypoints and clipping made everything past the twelfth unreachable - the operator
    // could see that item 40 existed, because the checks panel named it, and could not select it
    // to change or delete it.
    let count = plan_items.len();
    panel(
        "mission items",
        div()
            .flex()
            .flex_col()
            .child(
                div()
                    .id("plan-items")
                    .flex()
                    .flex_col()
                    .max_h(px(280.0))
                    .overflow_y_scroll()
                    .child(rows),
            )
            .children((count > 8).then(|| {
                div()
                    .pt_1()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(format!("{count} items - scroll for the rest"))
            })),
    )
}

/// The editor for the selected item: what it does and how high.
///
/// Steppers rather than typed numbers. Text entry in gpui needs a focus-managing input element
/// that does not exist here yet, and a stepper cannot produce a half-typed altitude that looks
/// like a number - "5" on the way to "50" is a valid altitude, and a mission editor that can
/// briefly hold one is a mission editor that can upload one.
pub fn editor_panel(
    plan_items: &[MissionItem],
    selected: Option<u16>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(item) = selected.and_then(|seq| plan_items.iter().find(|item| item.seq == seq)) else {
        return panel(
            "item",
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("select an item to change what it does"),
        )
        .into_any_element();
    };
    let seq = item.seq;
    let current_command = item.command;

    let mut commands = div().flex().flex_wrap().gap_1();
    for (command, label) in EDITABLE_COMMANDS {
        let chosen = *command == current_command;
        let command = *command;
        commands = commands.child(
            crate::probe::measured(*label, div())
                .id(("plan-cmd", usize::from(command)))
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .bg(rgb(if chosen { theme::ACTION } else { theme::PANEL }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(*label)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plan.set_command(seq, command);
                    this.sync_map_mission();
                    cx.notify();
                })),
        );
    }

    let mut steppers = div().flex().items_center().gap_2().child(
        div()
            .w(px(80.0))
            .text_lg()
            .text_color(rgb(theme::TEXT))
            .child(format!("{:.0} m", item.z)),
    );
    for (delta, name) in ALTITUDE_STEPS {
        steppers = steppers.child(
            crate::probe::measured(name, div())
                .id(name)
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(if delta > 0.0 {
                    format!("+{delta:.0}")
                } else {
                    format!("{delta:.0}")
                })
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plan.nudge_altitude(seq, delta);
                    this.sync_map_mission();
                    cx.notify();
                })),
        );
    }

    panel(
        format!("item {seq}").as_str(),
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(div().text_xs().text_color(rgb(theme::DIM)).child("command"))
            .child(commands)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("altitude above home"),
            )
            .child(steppers),
    )
    .into_any_element()
}

/// The survey tool: draw an area, choose how to fly it, generate the pattern.
pub fn survey_panel(
    mode: DrawMode,
    vertices: usize,
    options: GridOptions,
    error: Option<&str>,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let drawing = mode == DrawMode::Area;

    let mode_row = div()
        .flex()
        .flex_wrap()
        .gap_2()
        .child(action(
            "survey-draw",
            if drawing { "drawing area" } else { "draw area" },
            if drawing { theme::WARN } else { theme::ACCENT },
            !drawing,
            cx.listener(|this, _event: &(), _window, cx| {
                this.plan.set_draw_mode(DrawMode::Area);
                cx.notify();
            }),
        ))
        .child(action(
            "survey-waypoints",
            "add waypoints",
            theme::ACCENT,
            drawing,
            cx.listener(|this, _event: &(), _window, cx| {
                this.plan.set_draw_mode(DrawMode::Waypoints);
                cx.notify();
            }),
        ))
        .child(action(
            "survey-undo",
            "undo vertex",
            theme::TEXT,
            vertices > 0,
            cx.listener(|this, _event: &(), _window, cx| {
                this.plan.undo_area_vertex();
                this.sync_map_polygon();
                cx.notify();
            }),
        ))
        .child(action(
            "survey-clear",
            "clear area",
            theme::TEXT,
            vertices > 0,
            cx.listener(|this, _event: &(), _window, cx| {
                this.plan.clear_area();
                this.sync_map_polygon();
                cx.notify();
            }),
        ));

    // Three settings, each a value and a pair of steps. Overshoot is left at its default for now:
    // it only matters with a camera, and a control that does nothing visible is a control that
    // gets changed by accident.
    let setting = |name: &'static str,
                   label: &'static str,
                   value: String,
                   down: &'static str,
                   up: &'static str,
                   step: f64,
                   which: usize,
                   cx: &mut Context<MissionPlanner>| {
        div()
            .flex()
            .items_center()
            .gap_2()
            .child(
                div()
                    .w(px(70.0))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(label),
            )
            .child(
                div()
                    .w(px(72.0))
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(value),
            )
            .child(stepper(down, name, -step, which, cx))
            .child(stepper(up, name, step, which, cx))
    };

    panel(
        "survey",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(mode_row)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(if vertices >= 3 { theme::OK } else { theme::DIM }))
                    .child(match vertices {
                        0 => "right-click the map to place the area's corners".to_owned(),
                        1 | 2 => format!("{vertices} of at least 3 corners placed"),
                        _ => format!("{vertices} corners"),
                    }),
            )
            .child(setting(
                "survey-spacing",
                "spacing",
                format!("{:.0} m", options.spacing),
                "-5",
                "+5",
                5.0,
                0,
                cx,
            ))
            .child(setting(
                "survey-angle",
                "angle",
                format!("{:.0}°", options.angle),
                "-15",
                "+15",
                15.0,
                1,
                cx,
            ))
            .child(setting(
                "survey-altitude",
                "altitude",
                format!("{:.0} m", options.altitude),
                "-10",
                "+10",
                10.0,
                2,
                cx,
            ))
            .child(action(
                "survey-generate",
                "generate survey",
                theme::WARN,
                vertices >= 3,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.plan.generate_survey();
                    this.sync_map_mission();
                    cx.notify();
                }),
            ))
            .children(error.map(|error| {
                div()
                    .text_xs()
                    .text_color(rgb(theme::ALERT))
                    .child(error.to_owned())
            })),
    )
}

/// One step button for a survey setting.
///
/// `which` selects the setting rather than passing a closure, because the three settings clamp
/// differently and that logic belongs on the plan, not in the view.
fn stepper(
    label: &'static str,
    name: &'static str,
    delta: f64,
    which: usize,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    crate::probe::measured(
        format!("{name}{}", if delta > 0.0 { "-up" } else { "-down" }),
        div(),
    )
    .id((name, which * 2 + usize::from(delta > 0.0)))
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
        let (spacing, angle, altitude) = match which {
            0 => (delta, 0.0, 0.0),
            1 => (0.0, delta, 0.0),
            _ => (0.0, 0.0, delta),
        };
        this.plan.adjust_survey(spacing, angle, altitude);
        cx.notify();
    }))
}

/// What the validator says about the plan.
pub fn checks_panel(plan_items: &[MissionItem], view: &TelemetryView) -> AnyElement {
    let findings = validate_with(
        plan_items,
        ValidationContext {
            home: view.state.as_ref().and_then(|s| s.home),
            vehicle_type: view.state.as_ref().map(|s| s.vehicle_type),
        },
    );

    if findings.is_empty() {
        return panel(
            "checks",
            div()
                .text_xs()
                .text_color(rgb(theme::OK))
                .child("nothing to report"),
        )
        .into_any_element();
    }

    let mut lines = div().flex().flex_col().gap_1();
    for finding in &findings {
        let colour = match finding.severity {
            Severity::Danger => theme::ALERT,
            Severity::Warning => theme::WARN,
            Severity::Note => theme::DIM,
        };
        let prefix = finding
            .seq
            .map_or_else(|| "mission".to_owned(), |seq| format!("item {seq}"));
        lines = lines.child(
            div()
                .flex()
                .gap_2()
                .text_xs()
                .child(
                    div()
                        .flex_shrink_0()
                        .w(px(56.0))
                        .text_color(rgb(theme::DIM))
                        .child(prefix),
                )
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .text_color(rgb(colour))
                        .child(finding.message.clone()),
                ),
        );
    }

    panel("checks", lines).into_any_element()
}

/// Read, write, load, save and clear.
pub fn actions_panel(
    plan_items: &[MissionItem],
    origin: &Origin,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    let has_items = !plan_items.is_empty();

    let transfer_line = view.transfer.as_ref().map_or_else(
        || {
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(origin.label())
        },
        |status| {
            let colour = if status.failed {
                theme::ALERT
            } else if status.finished {
                theme::OK
            } else {
                theme::ACCENT
            };
            div()
                .text_xs()
                .text_color(rgb(colour))
                .child(status.label.clone())
        },
    );

    panel(
        "mission",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .flex()
                    .flex_wrap()
                    .gap_2()
                    .child(action(
                        "plan-read",
                        "read from vehicle",
                        theme::ACCENT,
                        has_vehicle,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.telemetry.request_mission();
                            this.adopt_vehicle_mission = true;
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-write",
                        "write to vehicle",
                        theme::WARN,
                        has_vehicle && has_items,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.telemetry.upload_mission(this.plan.items().to_vec());
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-save",
                        "save file",
                        theme::TEXT,
                        has_items,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.save_plan();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-load",
                        "load file",
                        theme::TEXT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.load_plan();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-clear",
                        "clear",
                        theme::TEXT,
                        has_items,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.plan.clear();
                            cx.notify();
                        }),
                    )),
            )
            .child(transfer_line)
            .children(view.transfer.as_ref().map(|status| {
                let colour = if status.failed {
                    theme::ALERT
                } else if status.finished {
                    theme::OK
                } else {
                    theme::ACCENT
                };
                progress(status.fraction, colour)
            })),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(lat: f64, lon: f64) -> LatLon {
        LatLon::new(lat, lon).expect("valid position")
    }

    #[test]
    fn a_new_plan_is_empty_and_says_so() {
        let plan = Plan::default();
        assert!(plan.is_empty());
        assert_eq!(*plan.origin(), Origin::Empty);
        assert_eq!(plan.origin().label(), "empty");
    }

    #[test]
    fn waypoints_are_numbered_from_zero_without_gaps() {
        let mut plan = Plan::default();
        for n in 0..4 {
            plan.add_waypoint(at(-35.36 + f64::from(n) * 0.001, 149.16), 50.0);
        }
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2, 3]);
    }

    #[test]
    fn deleting_renumbers_so_the_vehicle_will_accept_the_mission() {
        // A mission with a gap in its sequence is rejected at upload time, minutes after the edit
        // that caused it and with no indication of which one it was.
        let mut plan = Plan::default();
        for n in 0..4 {
            plan.add_waypoint(at(-35.36 + f64::from(n) * 0.001, 149.16), 50.0);
        }
        plan.remove(1);
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2]);
        assert_eq!(plan.items().len(), 3);
    }

    #[test]
    fn deleting_the_selected_item_clears_the_selection() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.add_waypoint(at(-35.37, 149.16), 50.0);
        plan.select(Some(1));
        plan.remove(1);
        assert_eq!(plan.selected(), None);
    }

    #[test]
    fn moving_an_item_reorders_positions_not_just_numbers() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.360, 149.16), 50.0);
        plan.add_waypoint(at(-35.361, 149.16), 50.0);
        plan.add_waypoint(at(-35.362, 149.16), 50.0);

        plan.move_item(2, -1);

        let latitudes: Vec<f64> = plan.items().iter().map(|i| i.x).collect();
        assert!((latitudes[1] - -35.362).abs() < 1e-9, "{latitudes:?}");
        assert!((latitudes[2] - -35.361).abs() < 1e-9, "{latitudes:?}");
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2]);
    }

    #[test]
    fn moving_past_either_end_does_nothing() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.add_waypoint(at(-35.37, 149.16), 50.0);

        plan.move_item(0, -1);
        plan.move_item(1, 1);

        let latitudes: Vec<f64> = plan.items().iter().map(|i| i.x).collect();
        assert!((latitudes[0] - -35.36).abs() < 1e-9, "{latitudes:?}");
        assert!((latitudes[1] - -35.37).abs() < 1e-9, "{latitudes:?}");
    }

    #[test]
    fn moving_an_item_that_is_not_there_is_ignored() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.move_item(99, 1);
        assert_eq!(plan.items().len(), 1);
    }

    #[test]
    fn an_edit_marks_the_plan_as_no_longer_the_vehicles() {
        // The header line is the only thing telling the operator that what they see is not what
        // the aircraft holds, so it has to change the instant an edit happens.
        let mut plan = Plan::default();
        plan.adopt_from_vehicle(vec![]);
        assert_eq!(*plan.origin(), Origin::Vehicle);
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn a_loaded_plan_names_its_file() {
        let mut plan = Plan::default();
        plan.adopt_from_file("survey.waypoints", vec![]);
        assert_eq!(plan.origin().label(), "loaded from survey.waypoints");
    }

    #[test]
    fn adopting_clears_a_stale_selection() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.select(Some(0));
        plan.adopt_from_vehicle(vec![]);
        assert_eq!(plan.selected(), None);
    }

    #[test]
    fn map_clicks_produce_items_a_vehicle_will_fly() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.363, 149.165), DEFAULT_ALTITUDE);
        let item = plan.items().first().expect("one item");
        assert_eq!(item.command, CMD_WAYPOINT);
        // Relative to home, not above the ellipsoid: an operator typing 50 means 50 above where
        // they are standing, and the difference in Canberra is about 600 metres.
        assert_eq!(item.frame, FRAME_RELATIVE);
        assert_eq!(item.autocontinue, 1);
        assert_eq!(item.current, 0);
    }

    #[test]
    fn command_labels_are_readable_and_never_guessed() {
        assert_eq!(command_label(16), "waypoint");
        assert_eq!(command_label(22), "takeoff");
        assert_eq!(command_label(20), "return_to_launch");
        // An unknown command shows its number rather than a wrong name.
        assert_eq!(command_label(60_000), "cmd 60000");
    }

    #[test]
    fn stepping_an_altitude_changes_only_that_item() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.add_waypoint(at(-35.37, 149.16), 50.0);

        plan.nudge_altitude(1, 10.0);

        assert!((plan.items()[0].z - 50.0).abs() < 1e-9);
        assert!((plan.items()[1].z - 60.0).abs() < 1e-9);
    }

    #[test]
    fn a_held_stepper_cannot_walk_a_waypoint_into_the_stratosphere() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for _ in 0..500 {
            plan.nudge_altitude(0, 10.0);
        }
        assert!((plan.items()[0].z - MAX_STEPPED_ALTITUDE).abs() < 1e-9);
    }

    #[test]
    fn a_waypoint_below_home_is_allowed() {
        // Meaningful on a vehicle launched from a cliff or a rooftop. Refusing it would be the
        // ground station deciding it knows the terrain better than the operator.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 5.0);
        plan.nudge_altitude(0, -10.0);
        assert!(plan.items()[0].z < 0.0);
    }

    #[test]
    fn changing_to_a_positionless_command_clears_the_coordinates() {
        // Leaving stale coordinates on a command that ignores them is how a mission looks right
        // on the map and flies somewhere else: the map would keep drawing a waypoint the vehicle
        // has no intention of visiting.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(0, CMD_RTL);
        assert!((plan.items()[0].x).abs() < f64::EPSILON);
        assert!((plan.items()[0].y).abs() < f64::EPSILON);
        assert_eq!(plan.items()[0].command, CMD_RTL);
    }

    #[test]
    fn changing_to_a_positioned_command_keeps_the_coordinates() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(0, 22);
        assert!((plan.items()[0].x - -35.36).abs() < 1e-9);
        assert_eq!(plan.items()[0].command, 22);
    }

    #[test]
    fn editing_an_item_marks_the_plan_as_no_longer_the_vehicles() {
        let mut plan = Plan::default();
        plan.adopt_from_vehicle(vec![MissionItem {
            seq: 0,
            current: 0,
            frame: FRAME_RELATIVE,
            command: CMD_WAYPOINT,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: -35.36,
            y: 149.16,
            z: 50.0,
            autocontinue: 1,
        }]);
        plan.nudge_altitude(0, 10.0);
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn editing_an_item_that_is_not_there_is_ignored() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.nudge_altitude(99, 10.0);
        plan.set_command(99, 22);
        assert!((plan.items()[0].z - 50.0).abs() < 1e-9);
        assert_eq!(plan.items()[0].command, CMD_WAYPOINT);
    }

    #[test]
    fn every_offered_command_has_a_readable_label() {
        // The chips in the editor are how a command is chosen; one showing a bare number would be
        // unusable.
        for (command, label) in EDITABLE_COMMANDS {
            assert!(!label.is_empty());
            assert!(
                !command_label(*command).starts_with("cmd "),
                "command {command} has no name in this dialect"
            );
        }
    }

    #[test]
    fn the_altitude_steps_are_coarse_and_fine_in_both_directions() {
        let deltas: Vec<f64> = ALTITUDE_STEPS.iter().map(|(delta, _)| *delta).collect();
        assert!(deltas.iter().any(|d| *d > 0.0));
        assert!(deltas.iter().any(|d| *d < 0.0));
        // Names must be unique: they address the controls a test script clicks.
        let mut names: Vec<&str> = ALTITUDE_STEPS.iter().map(|(_, name)| *name).collect();
        names.sort_unstable();
        let count = names.len();
        names.dedup();
        assert_eq!(names.len(), count, "duplicate stepper names");
    }

    /// A square about 300 m on a side near Canberra.
    fn area() -> Vec<LatLon> {
        vec![
            at(-35.3640, 149.1640),
            at(-35.3640, 149.1680),
            at(-35.3610, 149.1680),
            at(-35.3610, 149.1640),
        ]
    }

    #[test]
    fn clicks_build_the_area_in_the_order_they_are_made() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        assert_eq!(plan.polygon().len(), 4);
        assert!((plan.polygon()[0].latitude() - -35.3640).abs() < 1e-9);
    }

    #[test]
    fn undo_removes_the_last_vertex_drawn() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.undo_area_vertex();
        assert_eq!(plan.polygon().len(), 3);
        assert!((plan.polygon()[2].latitude() - -35.3610).abs() < 1e-9);
    }

    #[test]
    fn undo_on_an_empty_area_does_not_panic() {
        let mut plan = Plan::default();
        plan.undo_area_vertex();
        assert!(plan.polygon().is_empty());
    }

    #[test]
    fn a_survey_covers_the_area_it_was_drawn_over() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.adjust_survey(0.0, 0.0, 0.0);
        plan.generate_survey();

        assert!(plan.survey_error().is_none(), "{:?}", plan.survey_error());
        assert!(
            plan.items().len() >= 4,
            "a 300 m square at default spacing should need several lines, got {}",
            plan.items().len()
        );
        // Every generated point must be inside the area, or the aircraft flies outside what the
        // operator drew.
        for item in plan.items() {
            let position = item
                .position()
                .expect("a valid position")
                .expect("a position");
            assert!(
                mp_mission::survey::contains_within(plan.polygon(), position, 5.0),
                "generated {position:?} outside the drawn area"
            );
        }
    }

    #[test]
    fn a_survey_uses_the_chosen_altitude_and_a_relative_frame() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.adjust_survey(0.0, 0.0, 25.0);
        let expected = plan.survey_options().altitude;
        plan.generate_survey();

        assert!(!plan.items().is_empty());
        for item in plan.items() {
            assert!((item.z - expected).abs() < 1e-9, "{} vs {expected}", item.z);
            assert_eq!(item.frame, FRAME_RELATIVE);
            assert_eq!(item.command, CMD_WAYPOINT);
        }
    }

    #[test]
    fn a_survey_replaces_the_mission_rather_than_appending_to_it() {
        // A survey joined onto an existing mission would fly the old waypoints first and then
        // transit to the area, which is almost never what was meant.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.30, 149.10), 50.0);
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.generate_survey();

        let strays = plan
            .items()
            .iter()
            .filter(|item| (item.x - -35.30).abs() < 1e-9)
            .count();
        assert_eq!(strays, 0, "the old waypoint survived the survey");
    }

    #[test]
    fn too_few_corners_reports_why_rather_than_generating_nothing() {
        let mut plan = Plan::default();
        plan.add_area_vertex(at(-35.36, 149.16));
        plan.add_area_vertex(at(-35.36, 149.17));
        plan.generate_survey();

        assert!(plan.items().is_empty());
        let error = plan.survey_error().expect("an explanation");
        assert!(error.contains('3'), "{error}");
    }

    #[test]
    fn spacing_cannot_be_set_low_enough_to_generate_an_unflyable_mission() {
        // At one metre spacing a field-sized area generates tens of thousands of waypoints, which
        // no autopilot accepts and no operator intended.
        let mut plan = Plan::default();
        for _ in 0..100 {
            plan.adjust_survey(-100.0, 0.0, 0.0);
        }
        assert!(plan.survey_options().spacing >= 5.0);
    }

    #[test]
    fn the_survey_angle_wraps_rather_than_sticking_at_one_end() {
        // A bearing is circular; stopping at 359 would be arbitrary.
        let mut plan = Plan::default();
        plan.adjust_survey(0.0, -15.0, 0.0);
        assert!((plan.survey_options().angle - 345.0).abs() < 1e-9);
        plan.adjust_survey(0.0, 30.0, 0.0);
        assert!((plan.survey_options().angle - 15.0).abs() < 1e-9);
    }

    #[test]
    fn changing_the_angle_changes_the_pattern() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.generate_survey();
        let north_south: Vec<f64> = plan.items().iter().map(|item| item.x).collect();

        plan.adjust_survey(0.0, 90.0, 0.0);
        plan.generate_survey();
        let east_west: Vec<f64> = plan.items().iter().map(|item| item.x).collect();

        assert_ne!(north_south, east_west, "the angle had no effect");
    }

    #[test]
    fn clearing_the_area_leaves_the_mission_alone() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for vertex in area() {
            plan.add_area_vertex(vertex);
        }
        plan.clear_area();
        assert!(plan.polygon().is_empty());
        assert_eq!(plan.items().len(), 1);
    }

    #[test]
    fn drawing_mode_decides_what_a_map_click_does() {
        let mut plan = Plan::default();
        assert_eq!(plan.draw_mode(), DrawMode::Waypoints);
        plan.set_draw_mode(DrawMode::Area);
        assert_eq!(plan.draw_mode(), DrawMode::Area);
    }

    #[test]
    fn clearing_resets_everything() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.select(Some(0));
        plan.clear();
        assert!(plan.is_empty());
        assert_eq!(plan.selected(), None);
        assert_eq!(*plan.origin(), Origin::Empty);
    }
}
