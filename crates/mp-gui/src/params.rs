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

use std::cell::RefCell;
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Instant;

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;
use mp_params::{ParamMeta, UserLevel};

use crate::MissionPlanner;
use crate::telemetry::{Lookup, Telemetry, TelemetryView};
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
///
/// Over the list [`collect`] handed out, the answer is worked out once per search and kept with
/// the list (`positions_of`), so a frame that shows the same search of the same parameters as
/// the last does not search again.
#[must_use]
pub fn matching<'a>(parameters: &'a [Parameter], search: &str) -> Vec<&'a Parameter> {
    positions_of(parameters, search)
        .iter()
        .filter_map(|&position| parameters.get(position))
        .collect()
}

/// Whether a parameter matches a search already trimmed and upper-cased. Everything matches an
/// empty one.
fn matches(parameter: &Parameter, needle: &str) -> bool {
    needle.is_empty()
        || parameter.name.to_ascii_uppercase().contains(needle)
        || parameter
            .meta
            .is_some_and(|meta| meta.display_name.to_ascii_uppercase().contains(needle))
}

/// Collects the vehicle's parameters into the form the screen uses.
///
/// Each row carries its documentation, which is fourteen hundred lookups, so what was collected
/// is kept and handed out again while nothing it was made from has changed: the view's list,
/// which [`crate::telemetry::Telemetry::view`] shares between frames until a parameter arrives,
/// and the documentation, which changes when a fetched file is installed
/// ([`crate::metadata::generation`]).
#[must_use]
pub fn collect(view: &TelemetryView) -> Arc<[Parameter]> {
    collected(view, crate::metadata::generation(), crate::metadata::lookup)
}

/// What [`collect`] made last, and from what.
struct Collected {
    /// The view's list it was made from, held so the allocation cannot be reused by another list
    /// and mistaken for this one.
    from: Arc<[(String, f64)]>,
    /// The documentation's generation at the time.
    documentation: u64,
    parameters: Arc<[Parameter]>,
    /// [`groups`] of `parameters`, once the screen has asked.
    groups: Option<Arc<[(String, usize)]>>,
    /// The last search the screen made of `parameters`, and the positions of what it matched.
    search: Option<(String, Arc<[usize]>)>,
}

thread_local! {
    /// The screen draws on one thread; a test's collections stay on its own.
    static COLLECTED: RefCell<Option<Collected>> = const { RefCell::new(None) };
}

/// [`collect`] with the documentation, and which generation of it, given.
fn collected(
    view: &TelemetryView,
    documentation: u64,
    lookup: fn(&str) -> Option<&'static ParamMeta>,
) -> Arc<[Parameter]> {
    COLLECTED.with(|held| {
        let mut held = held.borrow_mut();
        if let Some(held) = held.as_ref()
            && Arc::ptr_eq(&held.from, &view.parameters)
            && held.documentation == documentation
        {
            return Arc::clone(&held.parameters);
        }
        let parameters: Arc<[Parameter]> = view
            .parameters
            .iter()
            .map(|(name, value)| Parameter {
                name: name.clone(),
                value: *value,
                meta: lookup(name),
            })
            .collect();
        *held = Some(Collected {
            from: Arc::clone(&view.parameters),
            documentation,
            parameters: Arc::clone(&parameters),
            groups: None,
            search: None,
        });
        parameters
    })
}

/// Runs `work` with the collection `parameters` is, if it is the one [`collect`] handed out last,
/// so what is worked out from it can be kept beside it; with `None` for any other list.
///
/// "Is" means the same allocation, which the collection's own handle keeps from being freed and
/// reused, so a list that merely looks the same is never taken for it.
fn with_collection<R>(
    parameters: &[Parameter],
    work: impl FnOnce(Option<&mut Collected>) -> R,
) -> R {
    COLLECTED.with(|held| {
        let mut held = held.borrow_mut();
        work(held.as_mut().filter(|held| {
            std::ptr::eq(held.parameters.as_ptr(), parameters.as_ptr())
                && held.parameters.len() == parameters.len()
        }))
    })
}

/// [`groups`], worked out once per collection rather than once per frame.
fn groups_of(parameters: &[Parameter]) -> Arc<[(String, usize)]> {
    with_collection(parameters, |held| match held {
        Some(held) => Arc::clone(held.groups.get_or_insert_with(|| groups(parameters).into())),
        None => groups(parameters).into(),
    })
}

/// Where in `parameters` what a search matches is, worked out once per collection and search
/// rather than once per frame: [`matching`], by position.
fn positions_of(parameters: &[Parameter], search: &str) -> Arc<[usize]> {
    let find = || -> Arc<[usize]> {
        let needle = search.trim().to_ascii_uppercase();
        parameters
            .iter()
            .enumerate()
            .filter(|(_, parameter)| matches(parameter, &needle))
            .map(|(position, _)| position)
            .collect()
    };
    with_collection(parameters, |held| match held {
        Some(held) => {
            if let Some((searched, found)) = &held.search
                && searched == search
            {
                return Arc::clone(found);
            }
            let found = find();
            held.search = Some((search.to_owned(), Arc::clone(&found)));
            found
        }
        None => find(),
    })
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
    for (group, count) in groups_of(parameters).iter() {
        let group = group.clone();
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

/// Saving, loading and comparing `.param` files.
///
/// This is how a build gets backed up before it is changed, how a setup is cloned onto a second
/// airframe, and how a tune somebody posted gets read before it is trusted. The order of the
/// buttons is the order of the task: save what is there, compare what is proposed, then - and
/// only then - apply it.
///
/// Compare and apply are deliberately two presses. A comparison that applied itself would make
/// "show me what this changes" the most destructive button on the screen.
pub fn file_panel(
    view: &TelemetryView,
    name: &crate::textfield::TextField,
    name_focus: &gpui::FocusHandle,
    focused: bool,
    differences: &[mp_params::param_file::Difference],
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let held = view.parameters.len();
    let expected = usize::from(view.parameters_expected);
    let complete = held > 0 && held >= expected;

    let mut rows = div().flex().flex_col();
    // Bounded. A comparison against another firmware version runs to hundreds of lines, and a
    // panel that long buries the buttons under it; the count above says how many there are.
    const SHOWN: usize = 40;
    for difference in differences.iter().take(SHOWN) {
        let (value, colour) = match difference.kind {
            mp_params::param_file::Change::Changed { from, to } => {
                (format!("{from} -> {to}"), theme::WARN)
            }
            mp_params::param_file::Change::Added { to } => {
                (format!("not on the vehicle -> {to}"), theme::DIM)
            }
            mp_params::param_file::Change::Missing { from } => {
                (format!("{from} -> not in the file"), theme::DIM)
            }
        };
        rows = rows.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .py(px(1.0))
                .child(
                    div()
                        .w(px(150.0))
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(difference.name.clone()),
                )
                .child(
                    div()
                        .flex_1()
                        .text_xs()
                        .text_color(rgb(colour))
                        .child(value),
                ),
        );
    }
    if differences.len() > SHOWN {
        rows = rows.child(
            div()
                .pt_1()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(format!("... and {} more", differences.len() - SHOWN)),
        );
    }

    let changed = differences
        .iter()
        .filter(|difference| {
            matches!(
                difference.kind,
                mp_params::param_file::Change::Changed { .. }
            )
        })
        .count();

    panel(
        "parameter files",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(crate::textfield::text_field(
                        "param-file-name",
                        name,
                        name_focus,
                        focused,
                        px(200.0),
                        cx.listener(|this, event: &gpui::KeyDownEvent, _window, cx| {
                            match this.param_file_name.key(event) {
                                // Enter saves, matching the mission name field beside it. A file
                                // name box where enter does nothing is a box that swallows the
                                // press an operator expects to act on it.
                                crate::textfield::KeyOutcome::Submitted => this.save_params(),
                                crate::textfield::KeyOutcome::Cancelled => {
                                    this.param_file_name.clear();
                                }
                                crate::textfield::KeyOutcome::Ignored => return,
                                crate::textfield::KeyOutcome::Changed => {}
                            }
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "param-save",
                        "save",
                        theme::ACCENT,
                        complete,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.save_params();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "param-compare",
                        "compare",
                        theme::ACCENT,
                        held > 0,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.compare_params();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "param-apply",
                        if changed == 0 {
                            "apply".to_owned()
                        } else {
                            format!("apply {changed}")
                        },
                        theme::WARN,
                        changed > 0,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.apply_params();
                            cx.notify();
                        }),
                    )),
            )
            // Why save is refused, said where the refusal happens rather than only after the
            // press. A greyed button with no reason reads as a broken screen.
            .children((!complete && held > 0).then(|| {
                div().text_xs().text_color(rgb(theme::DIM)).child(format!(
                    "{held} of {expected} downloaded - saving waits for the rest"
                ))
            }))
            .children((!differences.is_empty()).then_some(rows)),
    )
    .into_any_element()
}

// --- Writing -------------------------------------------------------------------------------------

/// One parameter write in a list, and what the C# says when it fails.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamWrite {
    /// The parameter.
    pub name: String,
    /// The value asked for.
    pub value: f64,
    /// Written even when the vehicle already holds the value: `setParam(..., force: true)`.
    pub force: bool,
    /// Said when every retry of the write goes unanswered.
    pub failure: String,
}

/// What a list of writes says when it has finished.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Finish {
    /// One step of a value in the editor: the value the vehicle now holds.
    Nudge,
    /// A file's differences: Write Params' summary, and how many the firmware does not have.
    Apply {
        /// Differences not written because this firmware has no such parameter.
        skipped: usize,
    },
}

/// How one write ended: for the facts, so a test can see the retry from outside.
#[derive(Debug, Clone, PartialEq)]
pub struct Written {
    /// The parameter.
    pub name: String,
    /// How the vehicle answered.
    pub outcome: RequestOutcome,
    /// How many times the `PARAM_SET` went on the wire.
    pub sends: u16,
}

impl Written {
    /// The outcome in a word or two.
    #[must_use]
    pub const fn outcome_word(&self) -> &'static str {
        match self.outcome {
            RequestOutcome::Accepted { .. } => "accepted",
            RequestOutcome::Unchanged => "unchanged",
            RequestOutcome::TimedOut => "timed out",
            RequestOutcome::UnknownParameter => "not on this vehicle",
            RequestOutcome::Rejected(_) => "refused",
            RequestOutcome::Sent => "sent",
        }
    }
}

/// Where the write under way is.
#[derive(Debug, Clone, Copy)]
enum Step {
    /// `GetParam` first. `setParam` sends only a name the vehicle has listed, and the C# always
    /// holds the whole list by the time a screen writes; this application downloads it only when
    /// asked, so a name not yet heard of is read before it is written.
    Reading(RequestId),
    /// `setParam`, retried by the link until the vehicle echoes the parameter.
    Writing(RequestId),
}

/// What one turn of [`ParamWrites::advance`] has to say.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Progress {
    /// For the status line, when it changes.
    pub status: Option<String>,
    /// A write that ended this turn.
    pub written: Option<Written>,
}

/// A list of parameter writes made one after another, as the C#'s are: each `setParam` blocks,
/// sending again every 700 ms up to three times, until the vehicle echoes the parameter, and only
/// then is the next one sent. A write that is never echoed is said in the C#'s words and the list
/// goes on.
/// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:313-370`
#[derive(Debug)]
pub struct ParamWrites {
    queue: VecDeque<ParamWrite>,
    /// The write under way, its step, and when that step's request was made.
    current: Option<(ParamWrite, Step, Instant)>,
    total: usize,
    failures: Vec<String>,
    finish: Finish,
    finished: bool,
    /// The last write that ended and the value the vehicle now holds for it.
    reported: Option<(String, f64)>,
}

impl ParamWrites {
    fn new(writes: Vec<ParamWrite>, finish: Finish) -> Self {
        Self {
            total: writes.len(),
            queue: writes.into(),
            current: None,
            failures: Vec::new(),
            finish,
            finished: false,
            reported: None,
        }
    }

    /// The editor's one step: `setParam(name, value)`, and "Set NAME Failed" if it is never
    /// echoed.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:323, 359-363`
    #[must_use]
    pub fn nudge(name: &str, value: f64) -> Self {
        Self::new(
            vec![ParamWrite {
                name: name.to_owned(),
                value,
                force: false,
                failure: format!("Set {name} Failed"),
            }],
            Finish::Nudge,
        )
    }

    /// Write Params: every difference, one after another, each "Set NAME Failed" if it is never
    /// echoed.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:313-370`
    #[must_use]
    pub fn apply(writes: impl IntoIterator<Item = (String, f64)>, skipped: usize) -> Self {
        let writes = writes
            .into_iter()
            .map(|(name, value)| ParamWrite {
                failure: format!("Set {name} Failed"),
                name,
                value,
                force: false,
            })
            .collect();
        Self::new(writes, Finish::Apply { skipped })
    }

    /// Whether every write has ended and the summary has been said.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        self.finished
    }

    /// Moves on as far as the link allows: the write under way checked, the next one started
    /// when it has ended, and the summary when the last one has.
    pub fn advance(&mut self, telemetry: &Telemetry) -> Progress {
        let mut progress = Progress::default();
        if self.finished {
            return progress;
        }
        loop {
            if let Some((write, step, made)) = self.current.take() {
                let id = match step {
                    Step::Reading(id) | Step::Writing(id) => id,
                };
                // A request the link has let go, or a link that has gone, is a write that never
                // got its answer.
                let request = match telemetry.lookup(id, made) {
                    Lookup::Found(request) => Some(request),
                    Lookup::PickingUp => {
                        self.current = Some((write, step, made));
                        return progress;
                    }
                    Lookup::Gone => None,
                };
                let outcome = request
                    .as_ref()
                    .map_or(Some(RequestOutcome::TimedOut), |request| request.outcome());
                let Some(outcome) = outcome else {
                    self.current = Some((write, step, made));
                    return progress;
                };
                match step {
                    // Heard of now: write it.
                    Step::Reading(_) if matches!(outcome, RequestOutcome::Accepted { .. }) => {
                        self.write(write, telemetry);
                        continue;
                    }
                    // The vehicle has no such parameter. `setParam` returns false for that
                    // without an exception, which none of its callers counts as a failure.
                    Step::Reading(_) => {
                        progress.written = Some(Written {
                            name: write.name,
                            outcome: RequestOutcome::UnknownParameter,
                            sends: 0,
                        });
                    }
                    Step::Writing(_) => {
                        if outcome == RequestOutcome::TimedOut
                            && !self.failures.contains(&write.failure)
                        {
                            self.failures.push(write.failure.clone());
                        }
                        let held = match outcome {
                            RequestOutcome::Accepted { value: Some(value) } => value.as_f64(),
                            _ => write.value,
                        };
                        self.reported = Some((write.name.clone(), held));
                        progress.written = Some(Written {
                            name: write.name,
                            outcome,
                            sends: request.map_or(0, |request| request.sends()),
                        });
                    }
                }
            }
            let Some(next) = self.queue.pop_front() else {
                self.finished = true;
                progress.status = self.summary();
                return progress;
            };
            if !matches!(self.finish, Finish::Nudge) {
                progress.status = Some(format!(
                    "writing {} of {}: {}",
                    self.total - self.queue.len(),
                    self.total,
                    next.name
                ));
            }
            if telemetry.holds_parameter(&next.name) {
                self.write(next, telemetry);
            } else {
                match telemetry.read_parameter(&next.name) {
                    Some(id) => self.current = Some((next, Step::Reading(id), Instant::now())),
                    None => self.fail_to_start(next),
                }
            }
            if self.current.is_some() {
                return progress;
            }
        }
    }

    /// Starts the write itself.
    fn write(&mut self, write: ParamWrite, telemetry: &Telemetry) {
        match telemetry.write_parameter(&write.name, write.value, write.force) {
            Some(id) => self.current = Some((write, Step::Writing(id), Instant::now())),
            None => self.fail_to_start(write),
        }
    }

    /// No link to write on: the write fails as one never echoed does.
    fn fail_to_start(&mut self, write: ParamWrite) {
        if !self.failures.contains(&write.failure) {
            self.failures.push(write.failure);
        }
    }

    /// What the C# says when the list is done, if anything.
    fn summary(&self) -> Option<String> {
        let failed = self.failures.join("; ");
        Some(match self.finish {
            // What the vehicle now holds, which is what the C#'s grid shows once `setParam` has
            // stored the echo - not necessarily what was asked for.
            Finish::Nudge if failed.is_empty() => {
                let (name, value) = self.reported.as_ref()?;
                format!("{name} = {value}")
            }
            Finish::Nudge => failed,
            // "Set X Failed" for each, then the summary box.
            // `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:362-371`
            Finish::Apply { skipped } => {
                let said = if failed.is_empty() {
                    format!("{} parameters successfully saved.", self.total)
                } else {
                    format!("{failed}. Not all parameters successfully saved.")
                };
                if skipped > 0 {
                    format!("{said} Skipped {skipped} this firmware does not have.")
                } else {
                    said
                }
            }
        })
    }
}

impl MissionPlanner {
    /// Starts a list of parameter writes.
    pub(crate) fn start_param_writes(&mut self, writes: ParamWrites) {
        self.param_writes.push(writes);
        self.advance_param_writes();
    }

    /// Once a frame: every list of writes moved on, what they say put on the status line, and
    /// the last write to end kept for the facts.
    pub(crate) fn advance_param_writes(&mut self) {
        let telemetry = &self.telemetry;
        for writes in &mut self.param_writes {
            let progress = writes.advance(telemetry);
            if let Some(text) = progress.status {
                self.file_status = Some(text);
            }
            if let Some(written) = progress.written {
                self.last_param_write = Some(written);
            }
        }
        self.param_writes.retain(|writes| !writes.is_finished());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SITL's 1,408 parameters, as a view carries them: every line of the dump, including the
    /// dozen a `.param` file load drops (`WP_TOTAL` and the like), since the vehicle sends those.
    fn sitl_view() -> TelemetryView {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let text = std::fs::read_to_string(&fixture).expect("the SITL dump");
        let mut view = TelemetryView::disconnected("test");
        view.parameters = text
            .lines()
            .filter_map(|line| {
                let (name, value) = line.split_once(',')?;
                Some((name.to_owned(), value.trim().parse().ok()?))
            })
            .collect();
        assert_eq!(view.parameters.len(), 1408);
        view
    }

    /// The bundled documentation, which no test installs over as one can the fetched file.
    fn bundled(name: &str) -> Option<&'static ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    /// A frame whose view carries the same list as the last collects nothing: it is handed what
    /// the last one collected. A new list - a parameter arrived - or new documentation is
    /// collected afresh, and what is collected is the list, row for row, with its documentation.
    #[test]
    fn collecting_the_same_list_twice_collects_it_once() {
        let view = sitl_view();
        let first = collected(&view, 1, bundled);
        assert_eq!(first.len(), view.parameters.len());
        for (row, (name, value)) in first.iter().zip(view.parameters.iter()) {
            assert_eq!(&row.name, name);
            assert!((row.value - value).abs() < f64::EPSILON);
            assert_eq!(row.meta, bundled(name), "{name}");
        }

        let started = Instant::now();
        let again = collected(&view, 1, bundled);
        let reused = started.elapsed();
        assert!(
            Arc::ptr_eq(&first, &again),
            "the same list was collected again"
        );
        let frame = view.clone();
        assert!(
            Arc::ptr_eq(&first, &collected(&frame, 1, bundled)),
            "a view cloned from the same frame shares its list"
        );

        let mut arrived = view.clone();
        let mut rows: Vec<(String, f64)> = view.parameters.to_vec();
        if let Some(row) = rows.iter_mut().find(|(name, _)| name == "RTL_ALT_M") {
            row.1 = 42.5;
        }
        arrived.parameters = rows.into();
        let fresh = collected(&arrived, 1, bundled);
        assert!(!Arc::ptr_eq(&first, &fresh), "a new list must be collected");
        let rtl = fresh
            .iter()
            .find(|row| row.name == "RTL_ALT_M")
            .expect("held");
        assert!(
            (rtl.value - 42.5).abs() < f64::EPSILON,
            "the new value is shown"
        );
        assert!(Arc::ptr_eq(&fresh, &collected(&arrived, 1, bundled)));

        let documented = collected(&arrived, 2, bundled);
        assert!(
            !Arc::ptr_eq(&fresh, &documented),
            "new documentation must be looked up"
        );

        let started = Instant::now();
        let _ = std::hint::black_box(collected(&view, 3, bundled));
        let collecting = started.elapsed();
        eprintln!(
            "collecting SITL's 1,408 parameters: {collecting:?}; handing the last collection \
             out again: {reused:?}"
        );
    }

    /// The group list and a search's matches are worked out once per collection and search, not
    /// once per frame, and are what [`groups`] and [`matching`] say. A new collection, or a
    /// different search, works them out again; a list that is not the collection is never
    /// answered from it.
    #[test]
    fn groups_and_matches_are_worked_out_once_per_collection_and_search() {
        let view = sitl_view();
        let parameters = collected(&view, 1, bundled);

        let started = Instant::now();
        let listed = groups_of(&parameters);
        let grouping = started.elapsed();
        assert_eq!(listed.to_vec(), groups(&parameters));
        let started = Instant::now();
        assert!(
            Arc::ptr_eq(&listed, &groups_of(&parameters)),
            "grouped again"
        );
        let regrouping = started.elapsed();

        let started = Instant::now();
        let found = positions_of(&parameters, "batt");
        let searching = started.elapsed();
        let names = |positions: &[usize]| -> Vec<String> {
            positions
                .iter()
                .map(|&position| parameters[position].name.clone())
                .collect()
        };
        // What a search is, said again from scratch: the fragment in the name or the display
        // name, in any case, in list order.
        let spelled_out = |fragment: &str| -> Vec<String> {
            parameters
                .iter()
                .filter(|parameter| {
                    parameter.name.to_ascii_uppercase().contains(fragment)
                        || parameter.meta.is_some_and(|meta| {
                            meta.display_name.to_ascii_uppercase().contains(fragment)
                        })
                })
                .map(|parameter| parameter.name.clone())
                .collect()
        };
        let expected = spelled_out("BATT");
        assert!(expected.len() > 10, "{expected:?}");
        assert_eq!(names(&found), expected);
        assert_eq!(
            matching(&parameters, "batt")
                .into_iter()
                .map(|parameter| parameter.name.clone())
                .collect::<Vec<_>>(),
            expected
        );
        let started = Instant::now();
        assert!(
            Arc::ptr_eq(&found, &positions_of(&parameters, "batt")),
            "searched again"
        );
        let researching = started.elapsed();
        let other = positions_of(&parameters, "wpnav");
        assert!(!Arc::ptr_eq(&found, &other));
        assert_eq!(names(&other), spelled_out("WPNAV"));
        eprintln!(
            "groups: {grouping:?} worked out, {regrouping:?} kept; search: {searching:?} worked \
             out, {researching:?} kept"
        );

        // A list that looks the same but is not the collection is worked out for itself.
        let copy: Vec<Parameter> = parameters.to_vec();
        assert!(!Arc::ptr_eq(&listed, &groups_of(&copy)));
        assert_eq!(groups_of(&copy).to_vec(), groups(&copy));
        assert!(!Arc::ptr_eq(&other, &positions_of(&copy, "wpnav")));

        // A new collection starts with nothing worked out.
        let fresh = collected(&view, 2, bundled);
        assert!(!Arc::ptr_eq(&listed, &groups_of(&fresh)));
        assert!(!Arc::ptr_eq(&other, &positions_of(&fresh, "wpnav")));
    }

    fn parameter(name: &str, value: f64) -> Parameter {
        Parameter {
            name: name.to_owned(),
            value,
            meta: mp_params::param_meta::lookup(name),
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
        let meta = mp_params::param_meta::lookup("ACCEL_Z_D").expect("metadata");
        let (low, high) = meta.range.expect("ACCEL_Z_D documents a range");
        assert!(low < high);
        assert!(!meta.accepts(high + 1.0));
        assert!(meta.accepts((low + high) / 2.0));

        // And a parameter without one is not treated as having an empty range, which would make
        // every value out of bounds.
        let no_range = mp_params::param_meta::lookup("BATT_CAPACITY").expect("metadata");
        assert!(no_range.range.is_none());
        assert!(
            no_range.accepts(3300.0),
            "a parameter with no range accepts anything"
        );
    }

    // --- Writing, through the real link to a scripted vehicle ------------------------------------

    use crate::telemetry::scripted::{INT32, Vehicle, param, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;

    /// `MAV_PARAM_TYPE_INT16`, as ArduPilot declares `RCn_MIN` and `RCn_MAX`.
    const INT16: u8 = 4;

    /// The link's waits, divided so a test runs the C#'s whole retry ladder in a blink; every
    /// count stays the C#'s.
    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// The name of each `PARAM_SET` and `PARAM_REQUEST_READ` the vehicle heard, in order.
    fn traffic(vehicle: &Vehicle) -> Vec<String> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::ParamSet(set) => {
                    Some(format!("set {}", mp_params::decode_param_id(&set.param_id)))
                }
                MavMessage::ParamRequestRead(read) => Some(format!(
                    "read {}",
                    mp_params::decode_param_id(&read.param_id)
                )),
                _ => None,
            })
            .collect()
    }

    /// Runs a list of writes against a vehicle playing `answer`, and returns everything it said.
    fn run(
        writes: &mut ParamWrites,
        telemetry: &Telemetry,
        vehicle: &mut Vehicle,
        mut answer: impl FnMut(&mut Vehicle, &MavMessage),
    ) -> (Vec<String>, Vec<Written>) {
        let mut said = Vec::new();
        let mut written = Vec::new();
        until("the writes to finish", || {
            for message in vehicle.read() {
                answer(&mut *vehicle, &message);
            }
            let progress = writes.advance(telemetry);
            said.extend(progress.status);
            written.extend(progress.written);
            writes.is_finished()
        });
        (said, written)
    }

    /// The editor's one step, whose first `PARAM_SET` is lost: the link sends it again, the
    /// vehicle echoes the second, and the status line says what the vehicle now holds. The C#'s
    /// `setParam` retry, driven through the screen's own list of writes and the real link.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1748-1770`
    #[test]
    fn a_write_whose_first_send_is_lost_is_sent_again_and_confirmed() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("RTL_ALT", 1500.0, INT32));
        until("RTL_ALT to be listed", || {
            telemetry.holds_parameter("RTL_ALT")
        });

        let mut writes = ParamWrites::nudge("RTL_ALT", 1501.0);
        let mut sets = 0;
        let (said, written) = run(&mut writes, &telemetry, &mut vehicle, |vehicle, message| {
            if let MavMessage::ParamSet(set) = message {
                sets += 1;
                // The first is lost on the way.
                if sets == 2 {
                    vehicle.send(&param("RTL_ALT", set.param_value, INT32));
                }
            }
        });

        assert_eq!(traffic(&vehicle), ["set RTL_ALT", "set RTL_ALT"]);
        assert_eq!(said, ["RTL_ALT = 1501"]);
        let [written] = written.as_slice() else {
            panic!("one write: {written:?}");
        };
        assert_eq!(written.name, "RTL_ALT");
        assert_eq!(written.outcome_word(), "accepted");
        assert_eq!(written.sends, 2);
    }

    /// A file's differences go one at a time: the second `PARAM_SET` waits until the first has
    /// had every retry, and the one never echoed is named, as the C#'s Write Params names it.
    /// `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:313-371`
    #[test]
    fn a_file_is_written_one_parameter_at_a_time_and_the_lost_one_is_named() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("RTL_ALT", 1500.0, INT32));
        until("RTL_ALT", || telemetry.holds_parameter("RTL_ALT"));
        vehicle.send(&param("WPNAV_SPEED", 500.0, INT32));
        until("WPNAV_SPEED", || telemetry.holds_parameter("WPNAV_SPEED"));

        let mut writes = ParamWrites::apply(
            [
                ("RTL_ALT".to_owned(), 2000.0),
                ("WPNAV_SPEED".to_owned(), 750.0),
            ],
            1,
        );
        // RTL_ALT is never echoed; WPNAV_SPEED is, first time.
        let (said, written) = run(&mut writes, &telemetry, &mut vehicle, |vehicle, message| {
            if let MavMessage::ParamSet(set) = message
                && mp_params::decode_param_id(&set.param_id) == "WPNAV_SPEED"
            {
                vehicle.send(&param("WPNAV_SPEED", set.param_value, INT32));
            }
        });

        assert_eq!(
            traffic(&vehicle),
            [
                "set RTL_ALT",
                "set RTL_ALT",
                "set RTL_ALT",
                "set RTL_ALT",
                "set WPNAV_SPEED"
            ]
        );
        assert_eq!(
            said.last().map(String::as_str),
            Some(
                "Set RTL_ALT Failed. Not all parameters successfully saved. \
                 Skipped 1 this firmware does not have."
            )
        );
        assert!(
            said.contains(&"writing 2 of 2: WPNAV_SPEED".to_owned()),
            "{said:?}"
        );
        let outcomes: Vec<_> = written
            .iter()
            .map(|w| (w.name.as_str(), w.outcome_word(), w.sends))
            .collect();
        assert_eq!(
            outcomes,
            [("RTL_ALT", "timed out", 4), ("WPNAV_SPEED", "accepted", 1)]
        );
    }

    /// Parameters on a vehicle whose list was never downloaded: each name is read before it is
    /// written - `setParam` sends only a name the vehicle has listed - and then written.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1640-1644`
    #[test]
    fn a_parameter_not_yet_listed_is_read_then_written() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut writes = ParamWrites::apply(
            [
                ("RC1_MIN".to_owned(), 1100.0),
                ("RC1_MAX".to_owned(), 1900.0),
            ],
            0,
        );
        let (said, _) = run(
            &mut writes,
            &telemetry,
            &mut vehicle,
            |vehicle, message| match message {
                MavMessage::ParamRequestRead(read) => {
                    let name = mp_params::decode_param_id(&read.param_id);
                    let held = if name.ends_with("MIN") {
                        1000.0
                    } else {
                        2000.0
                    };
                    vehicle.send(&param(&name, held, INT16));
                }
                MavMessage::ParamSet(set) => {
                    let name = mp_params::decode_param_id(&set.param_id);
                    vehicle.send(&param(&name, set.param_value, INT16));
                }
                _ => {}
            },
        );

        assert_eq!(
            traffic(&vehicle),
            ["read RC1_MIN", "set RC1_MIN", "read RC1_MAX", "set RC1_MAX"]
        );
        assert_eq!(
            said.last().map(String::as_str),
            Some("2 parameters successfully saved.")
        );
    }
}
