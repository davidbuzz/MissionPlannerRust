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

//! `Controls/paramcompare.cs`, the `ParamCompare` form: the parameters two lists hold with
//! different values, each with a Use box, and a button that writes the ones ticked.
//!
//! Ported for Initial Tune Parameter (`ConfigInitialParams.cs:230-241`), which opens it over the
//! vehicle's table and its own calculated values, renames its button "Write to FC", and shows it
//! as a dialog. That is the only way it is used here: with no `DataGridView` to write into and
//! no `dtlvcallback`, so the button's branch is the one that calls `setParam` for each row ticked
//! (`paramcompare.cs:65-89`).
//!
//! * The rows: every name of the first list (the vehicle's) that the second also has, when the
//!   two values' `ToString()` differ - .NET Framework's fifteen significant digits, so a value
//!   the vehicle holds as 0.1 and a new 0.1 are the same - each ticked; then sorted by name
//!   ascending, which for a grid of strings is a culture comparison (`:28-63`).
//! * "Check/Uncheck All" starts ticked and sets every row's box to its own (`:112-118`).
//! * The button writes each ticked row's new value, parsed back from its text, in one `try`: a
//!   timeout is `Strings.ErrorSettingParameter` in an error box and the form stays open; otherwise
//!   the form closes with `DialogResult.OK` (`:65-110`). A name the vehicle does not list is
//!   `setParam`'s false, which the loop ignores.
//!
//! The layout is the Designer's, a 428 x 523 client area: the grid at (12, 12), 399 x 474, its
//! four columns at their widths and no row headers; the button at (171, 492); the box at
//! (306, 496).
//!
//! What is not carried over, and why: resizing the form (`SizableToolWindow`) - it is drawn at
//! the Designer's size; and the button while its writes run - the C#'s UI thread is inside
//! `setParam` then and takes no click, so the button is drawn disabled until they end.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, SharedString, Window, div, prelude::*, px, rgb};

use super::compass::ERROR_SETTING_PARAMETER;
use super::optional::{Job, Set, error};
use crate::MissionPlanner;
use crate::ui::theme;

/// The form's `Text`, its caption.
/// `// C#: Controls/paramcompare.Designer.cs:121`
pub const FORM_TEXT: &str = "ParamCompare";
/// `CHK_toggleall.Text`.
/// `// C#: Controls/paramcompare.Designer.cs:103`
pub const TOGGLE_ALL: &str = "Check/Uncheck All";
/// The column headers, and their widths: the three text columns at the default 100, Use at 50.
/// `// C#: Controls/paramcompare.Designer.cs:60-85`
pub const COLUMNS: [(&str, f32); 4] = [
    ("Command", 100.0),
    ("Value", 100.0),
    ("New Value", 100.0),
    ("Use", 50.0),
];

/// The client area.
/// `// C#: Controls/paramcompare.Designer.cs:113`
const CLIENT: (f32, f32) = (428.0, 523.0);
/// `Params`.
const GRID: (f32, f32, f32, f32) = (12.0, 12.0, 399.0, 474.0);
/// `BUT_save`.
const SAVE: (f32, f32, f32, f32) = (171.0, 492.0, 75.0, 23.0);
/// `CHK_toggleall.Location`.
const TOGGLE: (f32, f32) = (306.0, 496.0);
/// The header row's height, `AutoSize` for one line.
const HEADER_HEIGHT: f32 = 23.0;
/// A row's, `DataGridView`'s default template.
const ROW_HEIGHT: f32 = 22.0;

/// One row of `Params`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompareRow {
    /// `Command`: the name.
    pub name: String,
    /// `Value`: the first list's value, as `ToString()` writes it.
    pub value: String,
    /// `newvalue`: the second's.
    pub new_value: String,
    /// `Use`.
    pub used: bool,
}

/// The form.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParamCompare {
    /// The rows, sorted.
    rows: Vec<CompareRow>,
    /// `CHK_toggleall.Checked`.
    toggle_all: bool,
    /// `BUT_save.Text`, which the caller may change.
    save_text: &'static str,
}

impl ParamCompare {
    /// `new ParamCompare(null, param, param2)`: its `processToScreen`.
    /// `// C#: Controls/paramcompare.cs:18-63`
    #[must_use]
    pub fn new(param: &[(String, f64)], param2: &[(String, f64)]) -> Self {
        let mut rows: Vec<CompareRow> = param
            .iter()
            .filter(|(name, _)| !name.is_empty())
            .filter_map(|(name, value)| {
                let (_, new) = param2.iter().find(|(held, _)| held == name)?;
                // `param[value].ToString() != param2[value].ToString()`.
                let value = mp_log::netfmt::double(*value);
                let new_value = mp_log::netfmt::double(*new);
                (value != new_value).then(|| CompareRow {
                    name: name.clone(),
                    value,
                    new_value,
                    used: true,
                })
            })
            .collect();
        rows.sort_by(|a, b| mp_log::netfmt::culture_compare(&a.name, &b.name));
        Self {
            rows,
            toggle_all: true,
            save_text: "Continue",
        }
    }

    /// `BUT_save.Text = text`, as a caller renames it.
    #[must_use]
    pub const fn with_save_text(mut self, text: &'static str) -> Self {
        self.save_text = text;
        self
    }

    /// The rows.
    #[must_use]
    pub fn rows(&self) -> &[CompareRow] {
        &self.rows
    }

    /// `CHK_toggleall.Checked`.
    #[must_use]
    pub const fn toggle_all(&self) -> bool {
        self.toggle_all
    }

    /// `BUT_save.Text`.
    #[must_use]
    pub const fn save_text(&self) -> &'static str {
        self.save_text
    }

    /// A row's Use box clicked.
    pub fn toggle_row(&mut self, index: usize) {
        if let Some(row) = self.rows.get_mut(index) {
            row.used = !row.used;
        }
    }

    /// "Check/Uncheck All" clicked: its `CheckedChanged`, every row's box set to its own.
    /// `// C#: Controls/paramcompare.cs:112-118`
    pub fn click_toggle_all(&mut self) {
        self.toggle_all = !self.toggle_all;
        for row in &mut self.rows {
            row.used = self.toggle_all;
        }
    }

    /// The button: each ticked row's new value, `double.Parse` of its text, in one `try` whose
    /// `catch` is `Strings.ErrorSettingParameter`. `tag` names the job for its caller.
    /// `// C#: Controls/paramcompare.cs:65-89`
    #[must_use]
    pub fn save(&self, tag: &'static str) -> Job {
        let sets = self.rows.iter().filter(|row| row.used).filter_map(|row| {
            let value = mp_log::netfmt::parse_double(&row.new_value)?;
            Some(Set::plain(row.name.trim(), value))
        });
        let mut job = Job::new(tag, sets);
        job.on_throw = Some(|_| error(ERROR_SETTING_PARAMETER));
        job
    }
}

/// Where the form's controls send their clicks.
pub struct Handlers<T, R, S, C> {
    /// "Check/Uncheck All".
    pub toggle_all: T,
    /// A row's Use box, by index.
    pub toggle_row: R,
    /// The button.
    pub save: S,
    /// The caption's close box: `DialogResult.Cancel`.
    pub close: C,
}

/// The form, drawn over the window as the C# shows it, `ShowDialog` centred on its parent.
pub fn dialog<T, R, S, C>(
    form: &ParamCompare,
    writing: bool,
    handlers: Handlers<T, R, S, C>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement
where
    T: Fn(&mut MissionPlanner) + 'static,
    R: Fn(&mut MissionPlanner, usize) + Clone + 'static,
    S: Fn(&mut MissionPlanner) + 'static,
    C: Fn(&mut MissionPlanner) + 'static,
{
    let Handlers {
        toggle_all,
        toggle_row,
        save,
        close,
    } = handlers;
    let size = window.viewport_size();
    let cell = |width: f32, height: f32, text: String, header: bool| {
        div()
            .flex_shrink_0()
            .w(px(width))
            .h(px(height))
            .flex()
            .items_center()
            .px_1()
            .border_r_1()
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .bg(rgb(if header { theme::ACTION } else { theme::BG }))
            .text_xs()
            .overflow_hidden()
            .whitespace_nowrap()
            .text_color(rgb(theme::TEXT))
            .child(text)
    };
    let tick = |ticked: bool| {
        div()
            .size(px(12.0))
            .flex()
            .items_center()
            .justify_center()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .children(ticked.then(|| div().size(px(6.0)).bg(rgb(theme::ACCENT))))
    };

    let mut header = div().flex().flex_shrink_0();
    for (name, width) in COLUMNS {
        header = header.child(cell(width, HEADER_HEIGHT, name.to_owned(), true));
    }
    let (gx, gy, gw, gh) = GRID;
    let mut grid = crate::probe::measured("paramcompare-grid", div())
        .id("paramcompare-grid")
        .absolute()
        .left(px(gx))
        .top(px(gy))
        .w(px(gw))
        .h(px(gh))
        .flex()
        .flex_col()
        .overflow_y_scroll()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .child(header);
    let [
        (_, name_width),
        (_, value_width),
        (_, new_width),
        (_, use_width),
    ] = COLUMNS;
    for (index, row) in form.rows().iter().enumerate() {
        let id = format!("paramcompare-use-{}", row.name);
        let toggle_row = toggle_row.clone();
        let use_cell = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex_shrink_0()
            .w(px(use_width))
            .h(px(ROW_HEIGHT))
            .flex()
            .items_center()
            .justify_center()
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .cursor_pointer()
            .child(tick(row.used))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                toggle_row(this, index);
                cx.notify();
            }));
        grid = grid.child(
            div()
                .flex()
                .flex_shrink_0()
                .child(cell(name_width, ROW_HEIGHT, row.name.clone(), false))
                .child(cell(value_width, ROW_HEIGHT, row.value.clone(), false))
                .child(cell(new_width, ROW_HEIGHT, row.new_value.clone(), false))
                .child(use_cell),
        );
    }

    let (sx, sy, sw, sh) = SAVE;
    let save_button = crate::probe::measured("paramcompare-save", div())
        .id("paramcompare-save")
        .absolute()
        .left(px(sx))
        .top(px(sy))
        .w(px(sw))
        .h(px(sh))
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .whitespace_nowrap()
        .child(form.save_text());
    let save_button = if writing {
        save_button
            .bg(rgb(theme::PANEL))
            .text_color(rgb(theme::DIM))
    } else {
        save_button
            .bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                save(this);
                cx.notify();
            }))
    };
    let toggle = crate::probe::measured("paramcompare-toggleall", div())
        .id("paramcompare-toggleall")
        .absolute()
        .left(px(TOGGLE.0))
        .top(px(TOGGLE.1))
        .flex()
        .items_center()
        .gap_1()
        .cursor_pointer()
        .child(tick(form.toggle_all()))
        .child(
            div()
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(TOGGLE_ALL),
        )
        .on_click(cx.listener(move |this, _event, _window, cx| {
            toggle_all(this);
            cx.notify();
        }));

    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "paramcompare-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                close(this);
                cx.notify();
            }),
        ));
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(grid)
        .child(save_button)
        .child(toggle);
    let form = crate::probe::measured("paramcompare", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("paramcompare-backdrop")
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(form),
            ),
    )
    .with_priority(1)
    .into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::SetQueue;
    use crate::config::optional::tests::{Answering, drain};
    use mp_link::requests::RequestOutcome;

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// The Designer's words, read from the tree when it is here.
    #[test]
    fn the_text_is_the_designers() {
        let Some(designer) =
            crate::config_coverage::source::csharp("Controls/paramcompare.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        for (name, _) in COLUMNS {
            assert!(
                designer.contains(&format!(".HeaderText = \"{name}\";")),
                "{name}"
            );
        }
        assert!(designer.contains("this.Use.Width = 50;"));
        assert!(designer.contains(&format!("this.CHK_toggleall.Text = \"{TOGGLE_ALL}\";")));
        assert!(designer.contains("this.BUT_save.Text = \"Continue\";"));
        assert!(designer.contains("this.Params.Location = new System.Drawing.Point(12, 12);"));
        assert!(designer.contains("this.ClientSize = new System.Drawing.Size(428, 523);"));
    }

    /// Only names both lists hold, only where the values' texts differ, sorted as a culture
    /// sorts: `INS_ACC_ID` before `INS_ACC2_ID`.
    #[test]
    fn the_rows_are_the_differences_in_culture_order() {
        let vehicle = table(&[
            ("INS_GYRO_FILTER", 20.0),
            ("ATC_THR_MIX_MAN", 0.1),
            ("BATT_ARM_VOLT", 0.0),
            ("INS_ACC2_X", 1.0),
            ("INS_ACC_X", 1.0),
            ("ONLY_HERE", 5.0),
        ]);
        let new = table(&[
            ("INS_GYRO_FILTER", 46.0),
            ("ATC_THR_MIX_MAN", 0.1),
            ("BATT_ARM_VOLT", 14.700_000_000_000_001),
            ("INS_ACC2_X", 2.0),
            ("INS_ACC_X", 2.0),
            ("NOT_ON_THE_VEHICLE", 1.0),
        ]);
        let form = ParamCompare::new(&vehicle, &new);
        let names: Vec<&str> = form.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "BATT_ARM_VOLT",
                "INS_ACC_X",
                "INS_ACC2_X",
                "INS_GYRO_FILTER"
            ]
        );
        // Fifteen digits: 14.700000000000001 is "14.7".
        assert_eq!(form.rows()[0].new_value, "14.7");
        assert!(form.rows().iter().all(|row| row.used));
        assert!(form.toggle_all());
        assert_eq!(form.save_text(), "Continue");
    }

    /// The button writes the ticked rows, parsed back from their text.
    #[test]
    fn save_writes_the_ticked_rows() {
        let vehicle = table(&[("A", 1.0), ("B", 2.0), ("C", 3.0)]);
        let new = table(&[("A", 1.5), ("B", 2.5), ("C", 3.5)]);
        let mut form = ParamCompare::new(&vehicle, &new).with_save_text("Write to FC");
        assert_eq!(form.save_text(), "Write to FC");
        form.toggle_row(1);
        let link = Answering::new(&[]);
        let mut queue = SetQueue::<usize>::default();
        queue.push([form.save("save")]);
        assert!(drain(&mut queue, &link).is_empty());
        assert_eq!(link.taken(), [("A".to_owned(), 1.5), ("C".to_owned(), 3.5)]);
        // Check/Uncheck All: off, then on again.
        form.click_toggle_all();
        assert!(form.rows().iter().all(|row| !row.used));
        form.click_toggle_all();
        assert!(form.rows().iter().all(|row| row.used));
    }

    /// A timeout ends the loop with `Strings.ErrorSettingParameter`.
    #[test]
    fn a_timeout_is_error_setting_parameter() {
        let vehicle = table(&[("A", 1.0), ("B", 2.0)]);
        let new = table(&[("A", 1.5), ("B", 2.5)]);
        let form = ParamCompare::new(&vehicle, &new);
        let link = Answering::new(&[("A", Progress::Finished(RequestOutcome::TimedOut))]);
        let mut queue = SetQueue::<usize>::default();
        queue.push([form.save("save")]);
        assert_eq!(drain(&mut queue, &link), [error(ERROR_SETTING_PARAMETER)]);
        assert_eq!(link.taken().len(), 1, "B is not reached");
    }
}
