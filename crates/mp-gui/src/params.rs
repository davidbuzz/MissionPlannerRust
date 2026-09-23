//! The parameter screen.
//!
//! A vehicle holds around fourteen hundred parameters. The problem is not showing them, it is
//! finding the one you want, and this has no text input to search with - gpui provides no input
//! element and writing one is its own project.
//!
//! So: group by prefix. Every ArduPilot parameter name begins with a subsystem - `BATT_`,
//! `COMPASS_`, `WPNAV_` - and choosing the subsystem first turns fourteen hundred into a list of
//! about eighty groups and then a handful of parameters. That is navigable with a mouse, which is
//! what an operator has in a field, and it matches how the documentation is organised.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_vehicle::{ParamMeta, UserLevel};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, panel, progress, theme};

/// One parameter as the screen needs it.
#[derive(Debug, Clone)]
pub struct Parameter {
    /// Name as the vehicle reports it.
    pub name: String,
    /// Current value.
    pub value: f64,
    /// Documentation, when this firmware's parameters are ones we have metadata for.
    pub meta: Option<&'static ParamMeta>,
}

impl Parameter {
    /// The group this belongs to: everything before the first underscore.
    ///
    /// `ATC_RAT_RLL_P` groups under `ATC`, not `ATC_RAT_RLL`. A deeper split makes groups of one,
    /// which is a list of fourteen hundred groups rather than fourteen hundred parameters.
    #[must_use]
    pub fn group(&self) -> &str {
        self.name
            .split_once('_')
            .map_or(self.name.as_str(), |(prefix, _)| prefix)
    }

    /// What to show as the value: a name when the parameter is an enumeration, else a number.
    ///
    /// An enumerated parameter shown as "4" tells the operator nothing; shown as "Loiter" it
    /// tells them what the vehicle will do.
    #[must_use]
    pub fn shown(&self) -> String {
        #[allow(clippy::cast_possible_truncation)] // parameter values are small integers here
        let as_integer = self.value as i64;
        if let Some(meta) = self.meta
            && let Some(name) = meta.value_name(as_integer)
        {
            return format!("{as_integer}  {name}");
        }
        if (self.value - self.value.round()).abs() < f64::EPSILON {
            format!("{:.0}", self.value)
        } else {
            format!("{:.4}", self.value)
        }
    }
}

/// Narrows a parameter list to those matching a search.
///
/// Case-insensitive substring over the name, and over the display name when there is metadata for
/// it - somebody looking for the loiter speed is as likely to type "speed" as "WPNAV". A search
/// crosses groups, because the point of typing is to stop having to know which group a parameter
/// is in.
#[must_use]
pub fn matching<'a>(parameters: &'a [Parameter], search: &str) -> Vec<&'a Parameter> {
    let needle = search.trim().to_ascii_uppercase();
    if needle.is_empty() {
        return parameters.iter().collect();
    }
    parameters
        .iter()
        .filter(|parameter| {
            parameter.name.to_ascii_uppercase().contains(&needle)
                || parameter
                    .meta
                    .is_some_and(|meta| meta.display_name.to_ascii_uppercase().contains(&needle))
        })
        .collect()
}

/// Collects the vehicle's parameters into the form the screen uses.
#[must_use]
pub fn collect(view: &TelemetryView) -> Vec<Parameter> {
    view.parameters
        .iter()
        .map(|(name, value)| Parameter {
            name: name.clone(),
            value: *value,
            meta: mp_vehicle::param_meta::lookup(name),
        })
        .collect()
}

/// The groups present, in order, with how many parameters each holds.
#[must_use]
pub fn groups(parameters: &[Parameter]) -> Vec<(String, usize)> {
    let mut counted: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    for parameter in parameters {
        *counted.entry(parameter.group().to_owned()).or_default() += 1;
    }
    counted.into_iter().collect()
}

/// Download progress and the group list.
pub fn browser_panel(
    view: &TelemetryView,
    parameters: &[Parameter],
    selected_group: Option<&str>,
    search: &crate::textfield::TextField,
    search_focus: &gpui::FocusHandle,
    focused: bool,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();
    let expected = view.parameters_expected;
    let held = parameters.len();
    let fraction = if expected > 0 {
        #[allow(clippy::cast_precision_loss)] // counts are in the low thousands
        {
            (held as f32 / expected as f32).clamp(0.0, 1.0)
        }
    } else {
        0.0
    };
    let complete = expected > 0 && held >= usize::from(expected);

    let mut group_list = div().flex().flex_wrap().gap_1();
    for (group, count) in groups(parameters) {
        let chosen = selected_group == Some(group.as_str());
        let label = format!("{group} {count}");
        group_list = group_list.child(
            crate::probe::measured(format!("param-group-{group}"), div())
                .id(gpui::SharedString::from(format!("group-{group}")))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .bg(rgb(if chosen { theme::ACTION } else { theme::PANEL }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(label)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.selected_param_group = Some(group.clone());
                    this.selected_param = None;
                    cx.notify();
                })),
        );
    }

    panel(
        "parameters",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(action(
                        "param-download",
                        if held == 0 { "download" } else { "refresh" },
                        theme::ACCENT,
                        has_vehicle,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.telemetry.download_parameters();
                            cx.notify();
                        }),
                    ))
                    .child(crate::textfield::text_field(
                        "param-search",
                        search,
                        search_focus,
                        focused,
                        px(220.0),
                        cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                            match this.param_search.key(event) {
                                crate::textfield::KeyOutcome::Cancelled => {
                                    this.param_search.clear();
                                }
                                crate::textfield::KeyOutcome::Ignored => return,
                                _ => {}
                            }
                            // A search that crosses groups makes the chosen group meaningless, so
                            // it is dropped rather than left highlighted while showing something
                            // else.
                            if !this.param_search.is_empty() {
                                this.selected_param_group = None;
                            }
                            cx.notify();
                        }),
                    ))
                    .child(
                        div()
                            .flex_1()
                            .min_w(px(0.0))
                            .text_xs()
                            .text_color(rgb(if complete { theme::OK } else { theme::DIM }))
                            .child(if expected == 0 {
                                format!("{held} parameters")
                            } else {
                                format!("{held} of {expected}")
                            }),
                    ),
            )
            .children((expected > 0 && !complete).then(|| progress(fraction, theme::ACCENT)))
            .child(group_list),
    )
}

/// The parameters of the chosen group.
pub fn list_panel(
    parameters: &[Parameter],
    group: Option<&str>,
    search: &str,
    selected: Option<&str>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    // A search crosses groups: the point of typing is to stop having to know which group a
    // parameter is in. With nothing typed, the chosen group is the filter instead.
    let searching = !search.trim().is_empty();
    let shown: Vec<&Parameter> = if searching {
        matching(parameters, search)
    } else {
        let Some(group) = group else {
            return panel(
                "values",
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("choose a group above, or type to search"),
            )
            .into_any_element();
        };
        parameters.iter().filter(|p| p.group() == group).collect()
    };

    if shown.is_empty() {
        return panel(
            "values",
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(format!("nothing matches \"{}\"", search.trim())),
        )
        .into_any_element();
    }

    let mut rows = div().flex().flex_col();
    for parameter in shown {
        let chosen = selected == Some(parameter.name.as_str());
        let name = parameter.name.clone();
        let units = parameter.meta.map_or("", |meta| meta.units);
        rows = rows.child(
            crate::probe::measured(format!("param-{name}"), div())
                .id(gpui::SharedString::from(format!("row-{name}")))
                .flex()
                .gap_2()
                .py(px(1.0))
                .text_xs()
                .cursor_pointer()
                .bg(rgb(if chosen { theme::ACTION } else { theme::PANEL }))
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(div().w(px(150.0)).child(parameter.name.clone()))
                .child(div().w(px(130.0)).child(parameter.shown()))
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .text_color(rgb(theme::DIM))
                        .child(units.to_owned()),
                )
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    let already = this.selected_param.as_deref() == Some(name.as_str());
                    this.selected_param = (!already).then(|| name.clone());
                    cx.notify();
                })),
        );
    }

    panel(
        "values",
        div()
            .id("param-values")
            .flex()
            .flex_col()
            .max_h(px(420.0))
            .overflow_y_scroll()
            .child(rows),
    )
    .into_any_element()
}

/// What the selected parameter is, and controls to change it.
pub fn editor_panel(
    parameters: &[Parameter],
    selected: Option<&str>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(parameter) = selected.and_then(|name| parameters.iter().find(|p| p.name == name))
    else {
        return panel(
            "parameter",
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("choose a parameter to see what it does"),
        )
        .into_any_element();
    };

    let Some(meta) = parameter.meta else {
        // A parameter this firmware has and our metadata does not. Saying so is better than
        // showing an empty description that looks like the parameter has none.
        return panel(
            &parameter.name,
            div()
                .flex()
                .flex_col()
                .gap_2()
                .child(
                    div()
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(parameter.shown()),
                )
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::WARN))
                        .child("no documentation for this parameter in the bundled metadata"),
                ),
        )
        .into_any_element();
    };

    let name = parameter.name.clone();
    let step = meta.increment.unwrap_or(1.0);

    let mut body = div()
        .flex()
        .flex_col()
        .gap_2()
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(meta.display_name.to_owned()),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(meta.description.to_owned()),
        )
        .child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(140.0))
                        .text_lg()
                        .text_color(rgb(theme::TEXT))
                        .child(parameter.shown()),
                )
                .child(step_button(&name, -step, "-", cx))
                .child(step_button(&name, step, "+", cx)),
        );

    if let Some((low, high)) = meta.range {
        let inside = meta.accepts(parameter.value);
        body = body.child(
            div()
                .text_xs()
                .text_color(rgb(if inside { theme::DIM } else { theme::ALERT }))
                .child(if inside {
                    format!("range {low} to {high}")
                } else {
                    // ArduPilot accepts a write it considers invalid and then behaves oddly, so
                    // the editor is the last chance to notice.
                    format!("outside the documented range {low} to {high}")
                }),
        );
    }
    if !meta.bitmask.is_empty() {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let bits = meta.bit_names(parameter.value as u32);
        body = body.child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(if bits.is_empty() {
                    "no bits set".to_owned()
                } else {
                    bits.join(", ")
                }),
        );
    }
    if meta.reboot_required {
        body = body.child(
            div()
                .text_xs()
                .text_color(rgb(theme::WARN))
                .child("a change takes effect only after a reboot"),
        );
    }
    if meta.user_level == UserLevel::Advanced {
        body = body.child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("advanced"),
        );
    }

    panel(&parameter.name, body).into_any_element()
}

/// One step of a parameter's value.
///
/// Writes immediately. A staged edit needs somewhere to stage it and a way to tell the operator
/// what is pending, and a parameter write is already acknowledged by the vehicle echoing the new
/// value - which the screen shows, so the confirmation is the number changing.
fn step_button(
    name: &str,
    delta: f64,
    label: &'static str,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let name = name.to_owned();
    let id = gpui::SharedString::from(format!("step-{label}-{name}"));
    crate::probe::measured(format!("param-step-{label}"), div())
        .id(id)
        .px_3()
        .py_1()
        .rounded_md()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_sm()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(label)
        .on_click(cx.listener(move |this, _event, _window, cx| {
            this.nudge_parameter(&name, delta);
            cx.notify();
        }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parameter(name: &str, value: f64) -> Parameter {
        Parameter {
            name: name.to_owned(),
            value,
            meta: mp_vehicle::param_meta::lookup(name),
        }
    }

    #[test]
    fn a_group_is_the_first_segment_of_the_name() {
        // ATC_RAT_RLL_P groups under ATC. A deeper split makes groups of one, which is a list of
        // fourteen hundred groups rather than fourteen hundred parameters.
        assert_eq!(parameter("ATC_RAT_RLL_P", 0.0).group(), "ATC");
        assert_eq!(parameter("COMPASS_DEC", 0.0).group(), "COMPASS");
        // A name with no underscore is its own group rather than being dropped.
        assert_eq!(parameter("FORMAT", 0.0).group(), "FORMAT");
    }

    #[test]
    fn groups_are_counted_and_ordered() {
        let parameters = vec![
            parameter("BATT_CAPACITY", 0.0),
            parameter("BATT_MONITOR", 0.0),
            parameter("ARMING_CHECK", 0.0),
        ];
        let groups = groups(&parameters);
        assert_eq!(
            groups,
            vec![("ARMING".to_owned(), 1), ("BATT".to_owned(), 2)]
        );
    }

    #[test]
    fn an_enumerated_parameter_shows_what_the_value_means() {
        // "4" tells the operator nothing; "4 Loiter" tells them what the vehicle will do.
        let monitor = parameter("BATT_MONITOR", 4.0);
        assert!(monitor.meta.is_some(), "BATT_MONITOR should have metadata");
        let shown = monitor.shown();
        assert!(shown.starts_with('4'), "{shown}");
        assert!(
            shown.len() > 1,
            "expected a name alongside the number: {shown}"
        );
    }

    #[test]
    fn a_plain_number_is_shown_without_spurious_precision() {
        // A parameter that holds 50 should not read "50.0000".
        let whole = Parameter {
            name: "NOT_A_REAL_PARAM".to_owned(),
            value: 50.0,
            meta: None,
        };
        assert_eq!(whole.shown(), "50");

        let fractional = Parameter {
            name: "NOT_A_REAL_PARAM".to_owned(),
            value: 0.2234,
            meta: None,
        };
        assert!(
            fractional.shown().starts_with("0.22"),
            "{}",
            fractional.shown()
        );
    }

    #[test]
    fn a_search_matches_part_of_a_name_in_any_case() {
        let parameters = vec![
            parameter("WPNAV_SPEED", 0.0),
            parameter("WPNAV_SPEED_UP", 0.0),
            parameter("BATT_CAPACITY", 0.0),
        ];
        let found = matching(&parameters, "wpnav");
        assert_eq!(
            found.len(),
            2,
            "{:?}",
            found.iter().map(|p| &p.name).collect::<Vec<_>>()
        );

        // A fragment from the middle works too, which is the point of a substring search.
        assert_eq!(matching(&parameters, "CAPAC").len(), 1);
    }

    #[test]
    fn a_search_also_looks_at_what_the_parameter_is_called_in_words() {
        // Somebody looking for the loiter speed is as likely to type "speed" as "WPNAV".
        let parameters = vec![parameter("WPNAV_SPEED", 0.0)];
        let by_display = matching(&parameters, "speed");
        assert_eq!(by_display.len(), 1, "should match on the display name too");
    }

    #[test]
    fn an_empty_search_matches_everything() {
        // Otherwise clearing the box would empty the list rather than restoring it.
        let parameters = vec![parameter("A_ONE", 0.0), parameter("B_TWO", 0.0)];
        assert_eq!(matching(&parameters, "").len(), 2);
        assert_eq!(matching(&parameters, "   ").len(), 2);
    }

    #[test]
    fn a_search_that_matches_nothing_returns_nothing_rather_than_everything() {
        let parameters = vec![parameter("WPNAV_SPEED", 0.0)];
        assert!(matching(&parameters, "zzzz").is_empty());
    }

    #[test]
    fn a_parameter_with_no_metadata_still_appears() {
        // A firmware can have parameters the bundled metadata does not. Dropping them would hide
        // settings the vehicle actually has.
        let unknown = Parameter {
            name: "ZZ_NOT_IN_METADATA".to_owned(),
            value: 7.0,
            meta: None,
        };
        assert_eq!(unknown.group(), "ZZ");
        assert_eq!(unknown.shown(), "7");
    }

    #[test]
    fn documented_ranges_are_available_for_clamping() {
        // The editor clamps a step to the documented range, because ArduPilot accepts an
        // out-of-range write and then behaves oddly.
        // ACCEL_Z_D documents one; BATT_CAPACITY, checked first, does not - not every parameter
        // has a range, which is why the editor only shows one when there is one to show.
        let meta = mp_vehicle::param_meta::lookup("ACCEL_Z_D").expect("metadata");
        let (low, high) = meta.range.expect("ACCEL_Z_D documents a range");
        assert!(low < high);
        assert!(!meta.accepts(high + 1.0));
        assert!(meta.accepts((low + high) / 2.0));

        // And a parameter without one is not treated as having an empty range, which would make
        // every value out of bounds.
        let no_range = mp_vehicle::param_meta::lookup("BATT_CAPACITY").expect("metadata");
        assert!(no_range.range.is_none());
        assert!(
            no_range.accepts(3300.0),
            "a parameter with no range accepts anything"
        );
    }
}
