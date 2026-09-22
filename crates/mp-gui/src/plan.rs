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
use mp_mission::{MissionItem, Severity};
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

/// The mission being edited, separate from the vehicle's.
#[derive(Debug, Default)]
pub struct Plan {
    items: Vec<MissionItem>,
    /// Where the current contents came from, for the header line.
    origin: Origin,
    /// The item the operator has selected, if any.
    selected: Option<u16>,
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

    /// Discards everything.
    pub fn clear(&mut self) {
        self.items.clear();
        self.origin = Origin::Empty;
        self.selected = None;
    }

    /// Renumbers items 0..n so the sequence has no gaps.
    fn renumber(&mut self) {
        for (index, item) in self.items.iter_mut().enumerate() {
            item.seq = u16::try_from(index).unwrap_or(u16::MAX);
        }
    }
}

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
            div()
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

    panel(
        "mission items",
        div()
            .flex()
            .flex_col()
            .max_h(px(320.0))
            .overflow_hidden()
            .child(rows),
    )
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
            .gap_3()
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
