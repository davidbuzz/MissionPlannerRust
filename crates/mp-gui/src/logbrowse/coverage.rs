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

//! What Mission Planner's log browser can do, and what this one can: `Log/LogBrowse.cs`, ledgered.
//!
//! `Log/LogBrowse.designer.cs` wires 37 events to handlers - every button, check box, menu item
//! and mouse event of the window. [`LOGBROWSE`] is one row for each: the control, the event, the
//! handler, what the control says, and what stands in for it here. [`BEYOND`] lists what the
//! window does that the designer does not wire: handlers `LogBrowse.cs` adds itself, and the
//! behaviour of the chart control, ZedGraph, that the C# leaves at its defaults - its zoom, its
//! pan, its context menu, its point values. The report rendered from both lives at
//! `docs/coverage/logbrowse.md`, as `coverage.rs` keeps `docs/coverage/flightdata.md` for the
//! flight screen.
//!
//! Tests hold it to the truth: every wiring in the designer has exactly one row, when the C#
//! tree is present to parse; every row that claims an implementation names an id or a function
//! this module's screen has; and the committed report is what the tables render.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

pub use crate::coverage::{Action, Ours};
use Ours::{Done, Dropped, Missing, Plumbing};

const fn row(
    control: &'static str,
    event: &'static str,
    handler: &'static str,
    text: &'static str,
    ours: Ours,
) -> Action {
    Action {
        control,
        event,
        handler,
        text,
        ours,
    }
}

/// Every event wiring in `Log/LogBrowse.designer.cs`, in the designer's order.
pub const LOGBROWSE: &[Action] = &[
    row(
        "exportVisibleToolStripMenuItem",
        "Click",
        "exportVisibleToolStripMenuItem_Click",
        "Export Visible (the grid's menu)",
        Done("loggrid-menu-visible"),
    ),
    row(
        "exportFilesToolStripMenuItem",
        "Click",
        "exportFilesToolStripMenuItem_Click",
        "Export Files (the grid's menu)",
        Done("loggrid-menu-files"),
    ),
    row(
        "BUT_Graphit",
        "Click",
        "Graphit_Click",
        "Graph Left",
        Done("log-graph-left"),
    ),
    row(
        "BUT_cleargraph",
        "Click",
        "BUT_cleargraph_Click",
        "Clear Graph",
        Done("log-clear"),
    ),
    row(
        "BUT_loadlog",
        "Click",
        "BUT_loadlog_Click",
        "Load A Log",
        Done("log-open"),
    ),
    row(
        "splitContainerZgGrid",
        "Resize",
        "splitContainer1_Resize",
        "the chart and grid's splitter, repainted",
        Plumbing,
    ),
    row(
        "splitContainerZgMap",
        "Resize",
        "splitContainer2_Resize",
        "the chart and map's splitter, repainted",
        Plumbing,
    ),
    row(
        "zg1",
        "ZoomEvent",
        "zg1_ZoomEvent",
        "a zoom or pan: the labels and the map redrawn",
        Done("fn zoom_event"),
    ),
    row(
        "zg1",
        "MouseMoveEvent",
        "zg1_MouseMoveEvent",
        "the pointer over the chart: a debounce",
        Plumbing,
    ),
    row(
        "zg1",
        "MouseDoubleClick",
        "zg1_MouseDoubleClick",
        "the chart double-clicked: the cursor",
        Done("fn double_click"),
    ),
    row(
        "myGMAP1",
        "OnRouteClick",
        "myGMAP1_OnRouteClick",
        "a route clicked",
        Dropped(
            "dead in the C#: `DrawMap` makes every route `IsHitTestVisible = false`, so GMap \
             never raises it",
        ),
    ),
    row(
        "myGMAP1",
        "MouseDown",
        "myGMAP1_MouseDown",
        "the map dragged",
        Done("log-map"),
    ),
    row(
        "myGMAP1",
        "MouseMove",
        "myGMAP1_MouseMove",
        "the map dragged",
        Done("log-map"),
    ),
    row(
        "myGMAP1",
        "MouseUp",
        "myGMAP1_MouseUp",
        "the map dragged",
        Done("log-map"),
    ),
    row(
        "chk_params",
        "CheckedChanged",
        "chk_params_CheckedChanged",
        "Show Params",
        Done("log-chk-params"),
    ),
    row(
        "chk_events",
        "CheckedChanged",
        "chk_events_CheckedChanged",
        "Events",
        Done("log-chk-events"),
    ),
    row(
        "chk_datagrid",
        "CheckedChanged",
        "chk_datagrid_CheckedChanged",
        "Data Table",
        Done("log-chk-datagrid"),
    ),
    row(
        "chk_msg",
        "CheckedChanged",
        "chk_msg_CheckedChanged",
        "MSG",
        Done("log-chk-msg"),
    ),
    row(
        "chk_errors",
        "CheckedChanged",
        "chk_errors_CheckedChanged",
        "Errors",
        Done("log-chk-errors"),
    ),
    row(
        "chk_mode",
        "CheckedChanged",
        "chk_mode_CheckedChanged",
        "Mode",
        Done("log-chk-mode"),
    ),
    row(
        "BUT_Graphit_R",
        "Click",
        "BUT_Graphit_R_Click",
        "Graph Right",
        Done("log-graph-right"),
    ),
    row(
        "chk_time",
        "CheckedChanged",
        "chk_time_CheckedChanged",
        "Time",
        Done("log-chk-time"),
    ),
    row(
        "CHK_map",
        "CheckedChanged",
        "CHK_map_CheckedChanged",
        "Map",
        Done("log-chk-map"),
    ),
    row(
        "CMB_preselect",
        "SelectedIndexChanged",
        "CMB_preselect_SelectedIndexChanged",
        "the preselected graphs",
        Done("log-preselect"),
    ),
    row(
        "BUT_removeitem",
        "Click",
        "BUT_removeitem_Click",
        "Remove Item",
        Dropped("hidden in the C#: `BUT_removeitem.Visible = False` in the resx"),
    ),
    row(
        "dataGridView1",
        "CellDoubleClick",
        "dataGridView1_CellDoubleClick",
        "a row double-clicked: the map and the chart go to it",
        Done("fn grid_double_click"),
    ),
    row(
        "dataGridView1",
        "CellValueNeeded",
        "dataGridView1_CellValueNeeded",
        "a cell's text",
        Done("fn build_row"),
    ),
    row(
        "dataGridView1",
        "ColumnHeaderMouseClick",
        "dataGridView1_ColumnHeaderMouseClick",
        "a header clicked: the grid filtered to a type",
        Done("fn toggle_grid_chooser"),
    ),
    row(
        "dataGridView1",
        "RowEnter",
        "dataGridView1_RowEnter",
        "the headers named for the current row",
        Done("fn headers"),
    ),
    row(
        "treeView1",
        "AfterCheck",
        "treeView1_AfterCheck",
        "a field ticked: graphed, or unticked: removed",
        Done("fn graph"),
    ),
    row(
        "treeView1",
        "DrawNode",
        "treeView1_DrawNode",
        "the tree's text, owner-drawn",
        Plumbing,
    ),
    row(
        "treeView1",
        "NodeMouseHover",
        "treeView1_TreeNodeMouseHover",
        "a field's description in `txt_info`",
        Done("fn hover_field"),
    ),
    row(
        "treeView1",
        "DoubleClick",
        "treeView1_DoubleClick",
        "a field's scaler and offset",
        Done("fn ask_modifier"),
    ),
    row(
        "treeView1",
        "MouseDown",
        "treeView1_MouseDown",
        "a right click: the right axis",
        Done("fn graph"),
    ),
    row(
        "LogBrowse",
        "FormClosed",
        "LogBrowse_FormClosed",
        "the log let go",
        Plumbing,
    ),
    row(
        "LogBrowse",
        "Load",
        "LogBrowse_Load",
        "a log opened",
        Done("fn open"),
    ),
    row(
        "LogBrowse",
        "Resize",
        "LogBrowse_Resize",
        "the splitters placed",
        Plumbing,
    ),
];

/// What the window does that the designer does not wire: `LogBrowse.cs`'s own handlers, and
/// ZedGraph's behaviour at the defaults `zg1` keeps.
pub const BEYOND: &[Action] = &[
    row(
        "LogBrowse",
        "ProcessCmdKey",
        "Ctrl+G",
        "Line no: the grid to a line",
        Done("fn go_to_line"),
    ),
    row(
        "chk_datagrid, chk_time, CHK_map, chk_errors, chk_mode, chk_msg",
        "CheckedChanged",
        "(lambdas in LoadLog2)",
        "the boxes remembered as LB_Grid, LB_Time, LB_Map, LB_Error, LB_Mode, LB_MSG",
        Done("fn apply_remembered"),
    ),
    row(
        "dataGridView1",
        "RowUnshared",
        "dataGridView1_RowUnshared",
        "an empty handler",
        Plumbing,
    ),
    row(
        "zg1",
        "ContextMenuBuilder",
        "Zg1_ContextMenuBuilder",
        "adds nothing: its three items are commented out",
        Plumbing,
    ),
    row(
        "zg1",
        "MouseDown/MouseUp",
        "(ZedGraph) HandleZoomFinish",
        "a left drag: every axis zoomed to the rectangle",
        Done("fn chart_drag_zoom"),
    ),
    row(
        "zg1",
        "MouseWheel",
        "(ZedGraph) ZedGraphControl_MouseWheel",
        "the wheel: every axis zoomed a tenth",
        Done("fn chart_wheel"),
    ),
    row(
        "zg1",
        "MouseDown/MouseMove",
        "(ZedGraph) HandlePanDrag",
        "Ctrl and a left drag, or a middle drag: every axis panned",
        Done("fn chart_pan_to"),
    ),
    row(
        "zg1",
        "MouseMove",
        "(ZedGraph) HandlePointValues",
        "the point under the pointer, when Show Point Values is on",
        Done("fn point_tooltip"),
    ),
    row(
        "zg1 menu",
        "Click",
        "(ZedGraph) MenuClick_Copy",
        "Copy",
        Dropped("an image of the chart to the clipboard: gpui renders no image of a view to copy"),
    ),
    row(
        "zg1 menu",
        "Click",
        "(ZedGraph) MenuClick_SaveAs",
        "Save Image As...",
        Dropped("an image of the chart to a file: gpui renders no image of a view to save"),
    ),
    row(
        "zg1 menu",
        "Click",
        "(ZedGraph) MenuClick_ShowValues",
        "Show Point Values",
        Done("log-chart-menu-show_val"),
    ),
    row(
        "zg1 menu",
        "Click",
        "(ZedGraph) MenuClick_ZoomOut",
        "Un-Zoom / Un-Pan",
        Done("log-chart-menu-unzoom"),
    ),
    row(
        "zg1 menu",
        "Click",
        "(ZedGraph) MenuClick_ZoomOutAll",
        "Undo All Zoom/Pan",
        Done("log-chart-menu-undo_all"),
    ),
    row(
        "zg1 menu",
        "Click",
        "(ZedGraph) MenuClick_RestoreScale",
        "Set Scale to Default",
        Done("log-chart-menu-set_default"),
    ),
    row(
        "treeView1",
        "AfterCheck (a bit's node)",
        "add_field_node; GraphItem(..., bitmask)",
        "a bitmask field's bits as child nodes, each graphed through its mask as MSG.Field.BIT",
        Done("fn graph_bit"),
    ),
    row(
        "zg1",
        "Scroll",
        "(ZedGraph) hScrollBar1",
        "the scroll bars: hidden, `IsShowHScrollBar` is false",
        Plumbing,
    ),
];

/// Why each row still missing is missing, rendered beside it in the report.
#[cfg(test)]
pub const WHY_MISSING: &[(&str, &str, &str)] = &[];

/// How many rows of a table are in each state: (done, elsewhere, missing, plumbing, dropped).
#[must_use]
pub fn counts(table: &[Action]) -> (usize, usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0, 0);
    for action in table {
        match action.ours {
            Done(_) => counts.0 += 1,
            Ours::Elsewhere(_) => counts.1 += 1,
            Missing => counts.2 += 1,
            Plumbing => counts.3 += 1,
            Dropped(_) => counts.4 += 1,
        }
    }
    counts
}

/// One table, as Markdown rows.
#[cfg(test)]
fn table(out: &mut String, rows: &[Action]) {
    out.push_str("| control | event | handler | text | ours |\n|---|---|---|---|---|\n");
    for action in rows {
        let ours = match action.ours {
            Done(id) => format!("done: `{id}`"),
            Ours::Elsewhere(what) => format!("elsewhere: {what}"),
            Missing => WHY_MISSING
                .iter()
                .find(|(control, event, _)| *control == action.control && *event == action.event)
                .map_or_else(
                    || "**missing**".to_owned(),
                    |(_, _, why)| format!("**missing** - {why}"),
                ),
            Plumbing => "plumbing".to_owned(),
            Dropped(reason) => format!("dropped: {reason}"),
        };
        out.push_str(&format!(
            "| `{}` | {} | `{}` | {} | {} |\n",
            action.control, action.event, action.handler, action.text, ours
        ));
    }
}

/// The report, as Markdown: the counts, then every row.
#[cfg(test)]
#[must_use]
pub fn report() -> String {
    let mut out = String::new();
    out.push_str("# LogBrowse action coverage\n\n");
    out.push_str(
        "Generated from `crates/mp-gui/src/logbrowse/coverage.rs` by `cargo test -p mp-gui \
         logbrowse::coverage -- --ignored update_report`; a test fails when this file is stale. \
         One row per event wiring in `Log/LogBrowse.designer.cs`, then what the window does \
         beyond them: `LogBrowse.cs`'s own handlers and the ZedGraph chart's default \
         behaviour.\n\n",
    );
    out.push_str("| table | total | done | elsewhere | missing | plumbing | dropped |\n");
    out.push_str("|---|---:|---:|---:|---:|---:|---:|\n");
    for (name, rows) in [("designer", LOGBROWSE), ("beyond the designer", BEYOND)] {
        let (done, elsewhere, missing, plumbing, dropped) = counts(rows);
        out.push_str(&format!(
            "| {name} | {} | {done} | {elsewhere} | {missing} | {plumbing} | {dropped} |\n",
            rows.len()
        ));
    }
    out.push_str("\n## `Log/LogBrowse.designer.cs`\n\n");
    table(&mut out, LOGBROWSE);
    out.push_str("\n## Beyond the designer\n\n");
    table(&mut out, BEYOND);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Where the committed report lives, relative to this crate.
    const REPORT: &str = "../../docs/coverage/logbrowse.md";

    /// The log browser's source, for checking that a claimed id or function exists.
    const SOURCES: &[&str] = &[
        include_str!("../logbrowse.rs"),
        include_str!("grid.rs"),
        include_str!("view.rs"),
        include_str!("export.rs"),
    ];

        fn designer() -> Option<String> {
        crate::config_coverage::source::csharp("Log/LogBrowse.designer.cs")
    }

    /// `this.X.Y += new Z(this.H);` → (X, Y, H); `this.Y += ...` is the form itself.
    fn wirings(designer: &str) -> Vec<(String, String, String)> {
        designer
            .lines()
            .filter_map(|line| {
                let line = line.trim();
                let (left, right) = line.split_once(" += new ")?;
                let left = left.strip_prefix("this.")?;
                let (control, event) = left.split_once('.').unwrap_or(("LogBrowse", left));
                let handler = right.rsplit_once("(this.")?.1.strip_suffix(");")?;
                Some((control.to_owned(), event.to_owned(), handler.to_owned()))
            })
            .collect()
    }

    /// Every wiring in the designer has one row, with the same handler, and nothing else does.
    #[test]
    fn every_designer_wiring_has_exactly_one_row() {
        let Some(designer) = designer() else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let mut wired = wirings(&designer);
        wired.sort();
        wired.dedup();
        assert_eq!(wired.len(), 37, "the designer wires 37 events");
        let mut ours: Vec<(String, String, String)> = LOGBROWSE
            .iter()
            .map(|a| {
                (
                    a.control.to_owned(),
                    a.event.to_owned(),
                    a.handler.to_owned(),
                )
            })
            .collect();
        ours.sort();
        assert_eq!(ours, wired);
    }

    /// Nothing is claimed that the source does not have.
    #[test]
    fn every_claimed_id_exists_in_the_log_browser() {
        for action in LOGBROWSE.iter().chain(BEYOND) {
            let Done(id) = action.ours else { continue };
            let found = SOURCES.iter().any(|source| {
                id.strip_prefix("fn ").map_or_else(
                    || source.contains(&format!("\"{id}\"")),
                    |name| source.contains(&format!("fn {name}(")),
                )
            });
            assert!(
                found,
                "{}.{} claims `{id}`, which is not in the source",
                action.control, action.event
            );
        }
    }

    /// Every reason given for a missing row is for a row that is there and missing, once.
    #[test]
    fn each_reason_is_for_a_missing_row() {
        for (index, (control, event, _)) in WHY_MISSING.iter().enumerate() {
            let row = LOGBROWSE
                .iter()
                .chain(BEYOND)
                .find(|action| action.control == *control && action.event == *event);
            assert!(
                matches!(row, Some(Action { ours: Missing, .. })),
                "{control}.{event} is not a missing row"
            );
            assert!(
                !WHY_MISSING
                    .get(..index)
                    .unwrap_or_default()
                    .iter()
                    .any(|(c, e, _)| c == control && e == event),
                "{control}.{event} twice"
            );
        }
        for action in LOGBROWSE.iter().chain(BEYOND) {
            if action.ours == Missing {
                assert!(
                    WHY_MISSING
                        .iter()
                        .any(|(c, e, _)| *c == action.control && *e == action.event),
                    "{}.{} is missing without a reason",
                    action.control,
                    action.event
                );
            }
        }
    }

    /// The committed report matches the tables.
    #[test]
    fn the_committed_report_is_current() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        let committed = std::fs::read_to_string(&path).unwrap_or_default();
        assert!(
            committed == report(),
            "docs/coverage/logbrowse.md is stale; run `cargo test -p mp-gui logbrowse::coverage \
             -- --ignored update_report`"
        );
    }

    /// Rewrites the report. Run on purpose, not on every test.
    #[test]
    #[ignore = "writes docs/coverage/logbrowse.md"]
    fn update_report() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        std::fs::write(&path, report()).expect("write the report");
    }

    /// The counts, so a change in either direction is a deliberate edit.
    #[test]
    fn the_counts_are_the_ones_recorded() {
        let designer = counts(LOGBROWSE);
        let beyond = counts(BEYOND);
        eprintln!("LogBrowse designer: {designer:?}; beyond: {beyond:?}");
        assert_eq!(designer, (29, 0, 0, 6, 2));
        assert_eq!(beyond, (11, 0, 0, 3, 2));
    }
}
