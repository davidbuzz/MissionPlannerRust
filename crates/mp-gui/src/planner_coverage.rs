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

//! What Mission Planner's planning screen can do, and what this one can (DELIVERABLES.md Deliverable 11).
//!
//! `GCSViews/FlightPlanner.Designer.cs` wires 121 events to handlers - the Read and Write
//! buttons, the waypoint grid, the home and default-altitude boxes, and the map's right-click
//! menu with its drop-downs, which is most of them. [`FLIGHTPLANNER`] is that list, one row per
//! wiring in the Designer's order, with what stands in for each here. The report it renders lives
//! at `docs/coverage/flightplanner.md` and a test keeps it current, as `coverage.rs` does for the
//! flight screen.
//!
//! The tests hold the list to the truth the same way. When the C# tree is present the Designer is
//! parsed and every wiring must have exactly one row. Every row that claims an implementation
//! names an id or a function that must appear in this crate's source, and a `menu-` id must be a
//! live entry of the menu `plan.rs` draws. The menu itself is checked against the Designer too:
//! its entries in `contextMenuStrip1.Items.AddRange` order, each drop-down in its own order, with
//! the `.resx` text. And the committed report must match what the table renders.
//!
//! The types mirror `coverage.rs`'s rather than sharing them, so the two tables can change
//! independently; the report format is the same.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

/// What stands in for a C# action here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // Elsewhere and Dropped have no row yet; the table says so, not the type
pub enum Ours {
    /// On the planning screen, under this control id (a `probe::measured` or `action` id, or a
    /// `menu-` entry of the map's menu) or `fn name` for a handler.
    Done(&'static str),
    /// Implemented, but somewhere other than the planning screen - a tab, the CLI.
    Elsewhere(&'static str),
    /// Not implemented.
    Missing,
    /// WinForms mechanics with no user action behind them: timers, paint and resize handlers, the
    /// grid's validation plumbing. Listed so the count is honest, not owed.
    Plumbing,
    /// Deliberately not carried over, with the reason.
    Dropped(&'static str),
}

/// One wiring in `FlightPlanner.Designer.cs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Action {
    /// The control's name in the Designer.
    pub control: &'static str,
    /// The event.
    pub event: &'static str,
    /// The handler the event is wired to.
    pub handler: &'static str,
    /// The control's text in `FlightPlanner.resx`, or a description where it has none.
    pub text: &'static str,
    /// What this application has for it.
    pub ours: Ours,
}

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

use Ours::{Done, Missing, Plumbing};

/// Every event wiring in `GCSViews/FlightPlanner.Designer.cs`, in the Designer's order.
pub const FLIGHTPLANNER: &[Action] = &[
    row(
        "TXT_WPRad",
        "KeyPress",
        "TXT_WPRad_KeyPress",
        "WP Radius",
        Done("plan-wprad"),
    ),
    row(
        "TXT_WPRad",
        "Leave",
        "TXT_WPRad_Leave",
        "WP Radius",
        Done("fn panel_leave"),
    ),
    row(
        "TXT_DefaultAlt",
        "KeyPress",
        "TXT_DefaultAlt_KeyPress",
        "Default Alt",
        Done("plan-defaultalt"),
    ),
    row(
        "TXT_DefaultAlt",
        "Leave",
        "TXT_DefaultAlt_Leave",
        "Default Alt",
        Done("fn panel_leave"),
    ),
    row(
        "TXT_loiterrad",
        "KeyPress",
        "TXT_loiterrad_KeyPress",
        "Loiter Radius",
        Done("plan-loiterrad"),
    ),
    row(
        "TXT_loiterrad",
        "Leave",
        "TXT_loiterrad_Leave",
        "Loiter Radius",
        Done("fn panel_leave"),
    ),
    row(
        "but_writewpfast",
        "Click",
        "but_writewpfast_Click",
        "Write Fast",
        Done("plan-writefast"),
    ),
    row(
        "BUT_write",
        "Click",
        "BUT_write_Click",
        "Write",
        Done("plan-write"),
    ),
    row(
        "BUT_read",
        "Click",
        "BUT_read_Click",
        "Read",
        Done("plan-read"),
    ),
    row(
        "label4",
        "LinkClicked",
        "label4_LinkClicked",
        "Home Location",
        Done("plan-home-link"),
    ),
    row(
        "TXT_homealt",
        "TextChanged",
        "TXT_homealt_TextChanged",
        "Home Location: ASL",
        Done("plan-home-alt"),
    ),
    row(
        "TXT_homelng",
        "TextChanged",
        "TXT_homelng_TextChanged",
        "Home Location: Long",
        Done("plan-home-lng"),
    ),
    row(
        "TXT_homelat",
        "TextChanged",
        "TXT_homelat_TextChanged",
        "Home Location: Lat",
        Done("plan-home-lat"),
    ),
    row(
        "TXT_homelat",
        "Enter",
        "TXT_homelat_Enter",
        "Home Location: Lat",
        Done("fn track_home_focus"),
    ),
    row(
        "coords1",
        "SystemChanged",
        "coords1_SystemChanged",
        "the pointer coordinates: system",
        Done("plan-coords-geo"),
    ),
    row(
        "chk_usemavftp",
        "CheckedChanged",
        "chk_usemavftp_CheckedChanged",
        "MAVFTP",
        Done("plan-mavftp"),
    ),
    row(
        "but_mincommands",
        "Click",
        "but_mincommands_Click",
        "˅",
        Done("plan-mincommands"),
    ),
    row(
        "CMB_altmode",
        "SelectedIndexChanged",
        "CMB_altmode_SelectedIndexChanged",
        "the altitude frame",
        Done("fn set_altitude_frame"),
    ),
    row(
        "CHK_splinedefault",
        "CheckedChanged",
        "CHK_splinedefault_CheckedChanged",
        "Spline",
        Done("plan-spline"),
    ),
    row(
        "Commands",
        "CellContentClick",
        "Commands_CellContentClick",
        "the waypoint grid: Delete, Up, Down",
        Done("fn row_controls"),
    ),
    row(
        "Commands",
        "CellEndEdit",
        "Commands_CellEndEdit",
        "the waypoint grid: a cell edited",
        Done("fn editor_panel"),
    ),
    row(
        "Commands",
        "DataError",
        "Commands_DataError",
        "the waypoint grid",
        Plumbing,
    ),
    row(
        "Commands",
        "DefaultValuesNeeded",
        "Commands_DefaultValuesNeeded",
        "the waypoint grid: a new row's frame",
        Done("fn add_waypoint_in"),
    ),
    row(
        "Commands",
        "EditingControlShowing",
        "Commands_EditingControlShowing",
        "the waypoint grid",
        Plumbing,
    ),
    row(
        "Commands",
        "RowEnter",
        "Commands_RowEnter",
        "the waypoint grid: the row's parameter names",
        Done("fn editor_panel"),
    ),
    row(
        "Commands",
        "RowsAdded",
        "Commands_RowsAdded",
        "the waypoint grid",
        Plumbing,
    ),
    row(
        "Commands",
        "RowsRemoved",
        "Commands_RowsRemoved",
        "the waypoint grid",
        Plumbing,
    ),
    row(
        "Commands",
        "RowValidating",
        "Commands_RowValidating",
        "the waypoint grid",
        Plumbing,
    ),
    row(
        "BUT_Add",
        "Click",
        "BUT_Add_Click",
        "Add Below",
        Done("plan-add-below"),
    ),
    row(
        "BUT_InjectCustomMap",
        "Click",
        "BUT_InjectCustomMap_Click",
        "Inject Custom Map",
        Done("plan-inject"),
    ),
    row(
        "chk_grid",
        "CheckedChanged",
        "chk_grid_CheckedChanged",
        "Grid",
        Done("plan-grid"),
    ),
    row(
        "lnk_kml",
        "LinkClicked",
        "lnk_kml_LinkClicked",
        "View KML",
        Done("plan-kml"),
    ),
    row(
        "BUT_loadwpfile",
        "Click",
        "BUT_loadwpfile_Click",
        "Load File",
        Done("plan-load"),
    ),
    row(
        "BUT_saveWPFile",
        "Click",
        "BUT_saveWPFile_Click",
        "Save File",
        Done("plan-save"),
    ),
    row(
        "panelMap",
        "Resize",
        "panelMap_Resize",
        "the map panel",
        Plumbing,
    ),
    row(
        "Zoomlevel",
        "ValueChanged",
        "Zoomlevel_ValueChanged",
        "Zoom",
        Done("plan-zoomlevel"),
    ),
    row(
        "TRK_zoom",
        "Scroll",
        "TRK_zoom_Scroll",
        "Zoom",
        Done("plan-trk-zoom"),
    ),
    row(
        "cmb_missiontype",
        "SelectedIndexChanged",
        "Cmb_missiontype_SelectedIndexChanged",
        "Mission / Fence / Rally",
        Done("draw-fence"),
    ),
    row("MainMap", "Paint", "MainMap_Paint", "the map", Plumbing),
    row(
        "contextMenuStrip1",
        "Closed",
        "contextMenuStrip1_Closed",
        "the map's right-click menu",
        Plumbing,
    ),
    row(
        "contextMenuStrip1",
        "Opening",
        "contextMenuStrip1_Opening",
        "the map's right-click menu",
        Done("fn open_map_menu"),
    ),
    row(
        "deleteWPToolStripMenuItem",
        "Click",
        "deleteWPToolStripMenuItem_Click",
        "Delete WP",
        Done("menu-deleteWP"),
    ),
    row(
        "insertWpToolStripMenuItem",
        "Click",
        "insertWpToolStripMenuItem_Click",
        "Insert Wp",
        Done("menu-insertWp"),
    ),
    row(
        "currentPositionToolStripMenuItem",
        "Click",
        "currentPositionToolStripMenuItem_Click",
        "At Current Position",
        Done("menu-currentPosition"),
    ),
    row(
        "insertSplineWPToolStripMenuItem",
        "Click",
        "insertSplineWPToolStripMenuItem_Click",
        "Insert Spline WP",
        Done("menu-insertSplineWP"),
    ),
    row(
        "loiterForeverToolStripMenuItem",
        "Click",
        "loiterForeverToolStripMenuItem_Click",
        "Forever",
        Done("menu-loiterForever"),
    ),
    row(
        "loitertimeToolStripMenuItem",
        "Click",
        "loitertimeToolStripMenuItem_Click",
        "Time",
        Done("menu-loitertime"),
    ),
    row(
        "loitercirclesToolStripMenuItem",
        "Click",
        "loitercirclesToolStripMenuItem_Click",
        "Circles",
        Done("menu-loitercircles"),
    ),
    row(
        "jumpstartToolStripMenuItem",
        "Click",
        "jumpstartToolStripMenuItem_Click",
        "Start",
        Done("menu-jumpstart"),
    ),
    row(
        "jumpwPToolStripMenuItem",
        "Click",
        "jumpwPToolStripMenuItem_Click",
        "WP #",
        Done("menu-jumpwP"),
    ),
    row(
        "rTLToolStripMenuItem",
        "Click",
        "rTLToolStripMenuItem_Click",
        "RTL",
        Done("menu-rTL"),
    ),
    row(
        "landToolStripMenuItem",
        "Click",
        "landToolStripMenuItem_Click",
        "Land",
        Done("menu-land"),
    ),
    row(
        "takeoffToolStripMenuItem",
        "Click",
        "takeoffToolStripMenuItem_Click",
        "Takeoff",
        Done("menu-takeoff"),
    ),
    row(
        "setROIToolStripMenuItem",
        "Click",
        "setROIToolStripMenuItem_Click",
        "DO_SET_ROI",
        Done("menu-setROI"),
    ),
    row(
        "clearMissionToolStripMenuItem",
        "Click",
        "clearMissionToolStripMenuItem_Click",
        "Clear Mission",
        Done("menu-clearMission"),
    ),
    row(
        "addPolygonPointToolStripMenuItem2",
        "Click",
        "addPolygonPointToolStripMenuItem_Click",
        "Draw a Polygon",
        Done("menu-addPolygonPoint2"),
    ),
    row(
        "clearPolygonToolStripMenuItem2",
        "Click",
        "clearPolygonToolStripMenuItem_Click",
        "Clear Polygon",
        Done("menu-clearPolygon2"),
    ),
    row(
        "savePolygonToolStripMenuItem2",
        "Click",
        "savePolygonToolStripMenuItem_Click",
        "Save Polygon",
        Done("menu-savePolygon2"),
    ),
    row(
        "loadPolygonToolStripMenuItem2",
        "Click",
        "loadPolygonToolStripMenuItem_Click",
        "Load Polygon",
        Done("menu-loadPolygon2"),
    ),
    row(
        "fromSHPToolStripMenuItem2",
        "Click",
        "fromSHPToolStripMenuItem_Click",
        "From SHP",
        Done("menu-fromSHP2"),
    ),
    row(
        "fromCurrentWaypointsToolStripMenuItem",
        "Click",
        "fromCurrentWaypointsMenuItem_Click",
        "From Current Waypoints",
        Done("menu-fromCurrentWaypoints"),
    ),
    row(
        "offsetPolygonToolStripMenuItem2",
        "Click",
        "offsetPolygonToolStripMenuItem_Click",
        "Offset Polygon",
        Done("menu-offsetPolygon2"),
    ),
    row(
        "areaToolStripMenuItem2",
        "Click",
        "areaToolStripMenuItem_Click",
        "Area",
        Done("menu-area2"),
    ),
    row(
        "GeoFenceuploadToolStripMenuItem",
        "Click",
        "GeoFenceuploadToolStripMenuItem_Click",
        "Upload",
        Done("menu-GeoFenceupload"),
    ),
    row(
        "GeoFencedownloadToolStripMenuItem",
        "Click",
        "GeoFencedownloadToolStripMenuItem_Click",
        "Download",
        Done("menu-GeoFencedownload"),
    ),
    row(
        "setReturnLocationToolStripMenuItem",
        "Click",
        "setReturnLocationToolStripMenuItem_Click",
        "Set Return Location",
        Done("menu-setReturnLocation"),
    ),
    row(
        "loadFromFileToolStripMenuItem",
        "Click",
        "loadFromFileToolStripMenuItem_Click",
        "Load from File",
        Done("menu-loadFromFile"),
    ),
    row(
        "saveToFileToolStripMenuItem",
        "Click",
        "saveToFileToolStripMenuItem_Click",
        "Save to File",
        Done("menu-saveToFile"),
    ),
    row(
        "clearToolStripMenuItem",
        "Click",
        "clearToolStripMenuItem_Click",
        "Clear",
        Done("menu-clear"),
    ),
    row(
        "setRallyPointToolStripMenuItem",
        "Click",
        "setRallyPointToolStripMenuItem_Click",
        "Set Rally Point",
        Done("menu-setRallyPoint"),
    ),
    row(
        "getRallyPointsToolStripMenuItem",
        "Click",
        "getRallyPointsToolStripMenuItem_Click",
        "Download",
        Done("menu-getRallyPoints"),
    ),
    row(
        "saveRallyPointsToolStripMenuItem",
        "Click",
        "saveRallyPointsToolStripMenuItem_Click",
        "Upload",
        Done("menu-saveRallyPoints"),
    ),
    row(
        "clearRallyPointsToolStripMenuItem",
        "Click",
        "clearRallyPointsToolStripMenuItem_Click",
        "Clear Rally Points",
        Done("menu-clearRallyPoints"),
    ),
    row(
        "saveToFileToolStripMenuItem1",
        "Click",
        "saveToFileToolStripMenuItem1_Click",
        "Save Rally to File",
        Done("menu-saveToFile1"),
    ),
    row(
        "loadFromFileToolStripMenuItem1",
        "Click",
        "loadFromFileToolStripMenuItem1_Click",
        "Load Rally from File",
        Done("menu-loadFromFile1"),
    ),
    row(
        "createWpCircleToolStripMenuItem",
        "Click",
        "createWpCircleToolStripMenuItem_Click",
        "Create Wp Circle",
        Done("menu-createWpCircle"),
    ),
    row(
        "createSplineCircleToolStripMenuItem",
        "Click",
        "createSplineCircleToolStripMenuItem_Click",
        "Create Spline Circle",
        Done("menu-createSplineCircle"),
    ),
    row(
        "areaToolStripMenuItem1",
        "Click",
        "areaToolStripMenuItem_Click",
        "Area",
        Done("menu-area1"),
    ),
    row(
        "textToolStripMenuItem",
        "Click",
        "textToolStripMenuItem_Click",
        "Text",
        Done("menu-text"),
    ),
    row(
        "createCircleSurveyToolStripMenuItem",
        "Click",
        "createCircleSurveyToolStripMenuItem_Click",
        "Create Circle Survey",
        Done("menu-createCircleSurvey"),
    ),
    row(
        "surveyGridToolStripMenuItem",
        "Click",
        "surveyGridToolStripMenuItem_Click",
        "Survey (Grid)",
        Done("menu-surveyGrid"),
    ),
    row(
        "ContextMeasure",
        "Click",
        "ContextMeasure_Click",
        "Measure Distance",
        Done("menu-ContextMeasure"),
    ),
    row(
        "rotateMapToolStripMenuItem",
        "Click",
        "rotateMapToolStripMenuItem_Click",
        "Rotate Map",
        Missing,
    ),
    row(
        "zoomToToolStripMenuItem",
        "Click",
        "zoomToToolStripMenuItem_Click",
        "Zoom To",
        Done("menu-zoomTo"),
    ),
    row(
        "prefetchToolStripMenuItem",
        "Click",
        "prefetchToolStripMenuItem_Click",
        "Prefetch",
        Done("menu-prefetch"),
    ),
    row(
        "prefetchWPPathToolStripMenuItem",
        "Click",
        "prefetchWPPathToolStripMenuItem_Click",
        "Prefetch WP Path",
        Done("menu-prefetchWPPath"),
    ),
    row(
        "kMLOverlayToolStripMenuItem",
        "Click",
        "kMLOverlayToolStripMenuItem_Click",
        "KML Overlay",
        Done("menu-kMLOverlay"),
    ),
    row(
        "elevationGraphToolStripMenuItem",
        "Click",
        "elevationGraphToolStripMenuItem_Click",
        "Elevation Graph",
        Done("menu-elevationGraph"),
    ),
    row(
        "reverseWPsToolStripMenuItem",
        "Click",
        "reverseWPsToolStripMenuItem_Click",
        "Reverse WPs",
        Done("menu-reverseWPs"),
    ),
    row(
        "loadWPFileToolStripMenuItem",
        "Click",
        "loadWPFileToolStripMenuItem_Click",
        "Load WP File",
        Done("menu-loadWPFile"),
    ),
    row(
        "loadAndAppendToolStripMenuItem",
        "Click",
        "loadAndAppendToolStripMenuItem_Click",
        "Load and Append",
        Done("menu-loadAndAppend"),
    ),
    row(
        "saveWPFileToolStripMenuItem",
        "Click",
        "saveWPFileToolStripMenuItem_Click",
        "Save WP File",
        Done("menu-saveWPFile"),
    ),
    row(
        "loadKMLFileToolStripMenuItem",
        "Click",
        "loadKMLFileToolStripMenuItem_Click",
        "Load KML File",
        Done("menu-loadKMLFile"),
    ),
    row(
        "loadSHPFileToolStripMenuItem",
        "Click",
        "loadSHPFileToolStripMenuItem_Click",
        "Load SHP File",
        Done("menu-loadSHPFile"),
    ),
    row(
        "poiaddToolStripMenuItem",
        "Click",
        "poiaddToolStripMenuItem_Click",
        "Add",
        Done("menu-poiadd"),
    ),
    row(
        "poideleteToolStripMenuItem",
        "Click",
        "poideleteToolStripMenuItem_Click",
        "Delete",
        Done("menu-poidelete"),
    ),
    row(
        "poieditToolStripMenuItem",
        "Click",
        "poieditToolStripMenuItem_Click",
        "Edit",
        Done("menu-poiedit"),
    ),
    row(
        "trackerHomeToolStripMenuItem",
        "Click",
        "trackerHomeToolStripMenuItem_Click",
        "Tracker Home",
        Done("menu-trackerHome"),
    ),
    row(
        "modifyAltToolStripMenuItem",
        "Click",
        "modifyAltToolStripMenuItem_Click",
        "Modify Alt",
        Done("menu-modifyAlt"),
    ),
    row(
        "enterUTMCoordToolStripMenuItem",
        "Click",
        "enterUTMCoordToolStripMenuItem_Click",
        "Enter UTM Coord",
        Done("menu-enterUTMCoord"),
    ),
    row(
        "switchDockingToolStripMenuItem",
        "Click",
        "switchDockingToolStripMenuItem_Click",
        "Switch Docking",
        Done("menu-switchDocking"),
    ),
    row(
        "setHomeHereToolStripMenuItem",
        "Click",
        "setHomeHereToolStripMenuItem_Click",
        "Set Home Here",
        Done("menu-setHomeHere"),
    ),
    row(
        "addPolygonPointToolStripMenuItem",
        "Click",
        "addPolygonPointToolStripMenuItem_Click",
        "Draw a Polygon",
        Done("menu-poly-addPolygonPoint"),
    ),
    row(
        "clearPolygonToolStripMenuItem",
        "Click",
        "clearPolygonToolStripMenuItem_Click",
        "Clear Polygon",
        Done("menu-poly-clearPolygon"),
    ),
    row(
        "savePolygonToolStripMenuItem",
        "Click",
        "savePolygonToolStripMenuItem_Click",
        "Save Polygon",
        Done("menu-poly-savePolygon"),
    ),
    row(
        "loadPolygonToolStripMenuItem",
        "Click",
        "loadPolygonToolStripMenuItem_Click",
        "Load Polygon",
        Done("menu-poly-loadPolygon"),
    ),
    row(
        "fromSHPToolStripMenuItem",
        "Click",
        "fromSHPToolStripMenuItem_Click",
        "From SHP",
        Done("menu-poly-fromSHP"),
    ),
    row(
        "areaToolStripMenuItem",
        "Click",
        "areaToolStripMenuItem_Click",
        "Area",
        Done("menu-poly-area"),
    ),
    row(
        "fenceInclusionToolStripMenuItem",
        "Click",
        "FenceInclusionToolStripMenuItem_Click",
        "Fence Inclusion",
        Done("menu-fenceInclusion"),
    ),
    row(
        "fenceExclusionToolStripMenuItem",
        "Click",
        "FenceExclusionToolStripMenuItem_Click",
        "Fence Exclusion",
        Done("menu-fenceExclusion"),
    ),
    row(
        "timer1",
        "Tick",
        "timer1_Tick",
        "the map refresh timer",
        Plumbing,
    ),
    row(
        "contextMenuStripPoly",
        "Opening",
        "ContextMenuStripPoly_Opening",
        "the polygon icon's menu",
        Done("plan-polyicon"),
    ),
    row(
        "convertWPToPolygonToolStripMenuItem",
        "Click",
        "fromCurrentWaypointsMenuItem_Click",
        "From Current Waypoints",
        Done("menu-poly-convertWPToPolygon"),
    ),
    row(
        "offsetPolygonToolStripMenuItem",
        "Click",
        "offsetPolygonToolStripMenuItem_Click",
        "Offset Polygon",
        Done("menu-poly-offsetPolygon"),
    ),
    row(
        "zoomToVehicleToolStripMenuItem",
        "Click",
        "zoomToVehicleToolStripMenuItem_Click",
        "Zoom to Vehicle",
        Done("menu-zoomToVehicle"),
    ),
    row(
        "zoomToMissionToolStripMenuItem",
        "Click",
        "zoomToMissionToolStripMenuItem_Click",
        "Zoom to Mission",
        Done("menu-zoomToMission"),
    ),
    row(
        "zoomToHomeToolStripMenuItem",
        "Click",
        "zoomToHomeToolStripMenuItem_Click",
        "Zoom to Home",
        Done("menu-zoomToHome"),
    ),
    row(
        "gDALOpacityToolStripMenuItem",
        "Click",
        "gDALOpacityToolStripMenuItem_Click",
        "GDAL Opacity",
        Missing,
    ),
    row(
        "FlightPlanner",
        "FormClosing",
        "FlightPlanner_FormClosing",
        "the screen",
        Plumbing,
    ),
    row(
        "FlightPlanner",
        "Load",
        "FlightPlanner_Load",
        "the screen",
        Plumbing,
    ),
    row(
        "FlightPlanner",
        "Resize",
        "Planner_Resize",
        "the screen",
        Plumbing,
    ),
];

/// How many rows are in each state: (done, elsewhere, missing, plumbing, dropped).
#[must_use]
pub fn counts() -> (usize, usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0, 0);
    for action in FLIGHTPLANNER {
        match action.ours {
            Ours::Done(_) => counts.0 += 1,
            Ours::Elsewhere(_) => counts.1 += 1,
            Ours::Missing => counts.2 += 1,
            Ours::Plumbing => counts.3 += 1,
            Ours::Dropped(_) => counts.4 += 1,
        }
    }
    counts
}

/// The report, as Markdown: the counts, the largest groups still missing, then every row.
#[cfg(test)]
#[must_use]
pub fn report() -> String {
    let (done, elsewhere, missing, plumbing, dropped) = counts();
    let mut out = String::new();
    out.push_str("# FlightPlanner action coverage\n\n");
    out.push_str(
        "Generated from `crates/mp-gui/src/planner_coverage.rs` by `cargo test -p mp-gui \
         planner_coverage -- --ignored update_report`; a test fails when this file is stale. One \
         row per event wiring in `GCSViews/FlightPlanner.Designer.cs`.\n\n",
    );
    out.push_str(&format!(
        "| total | done | elsewhere | missing | plumbing | dropped |\n|---:|---:|---:|---:|---:|---:|\n| {} | {done} | {elsewhere} | {missing} | {plumbing} | {dropped} |\n\n",
        FLIGHTPLANNER.len()
    ));
    out.push_str("Missing, by where the control sits:\n\n| group | missing |\n|---|---:|\n");
    for (group, count) in missing_groups() {
        out.push_str(&format!("| {group} | {count} |\n"));
    }
    out.push('\n');
    out.push_str("| control | event | handler | text | ours |\n|---|---|---|---|---|\n");
    for action in FLIGHTPLANNER {
        let ours = match action.ours {
            Ours::Done(id) => format!("done: `{id}`"),
            Ours::Elsewhere(what) => format!("elsewhere: {what}"),
            Ours::Missing => "**missing**".to_owned(),
            Ours::Plumbing => "plumbing".to_owned(),
            Ours::Dropped(reason) => format!("dropped: {reason}"),
        };
        out.push_str(&format!(
            "| `{}` | {} | `{}` | {} | {} |\n",
            action.control, action.event, action.handler, action.text, ours
        ));
    }
    out
}

/// Where a control sits: the map menu's drop-down that holds it, or the panel.
#[cfg(test)]
fn group_of(control: &str) -> &'static str {
    for entry in crate::plan::MAP_MENU {
        if entry.control == control {
            return "map menu";
        }
        if entry.children.iter().any(|child| child.control == control) {
            return entry.text;
        }
    }
    match control {
        "addPolygonPointToolStripMenuItem"
        | "clearPolygonToolStripMenuItem"
        | "savePolygonToolStripMenuItem"
        | "loadPolygonToolStripMenuItem"
        | "fromSHPToolStripMenuItem"
        | "convertWPToPolygonToolStripMenuItem"
        | "offsetPolygonToolStripMenuItem"
        | "areaToolStripMenuItem"
        | "fenceInclusionToolStripMenuItem"
        | "fenceExclusionToolStripMenuItem"
        | "contextMenuStripPoly" => "polygon icon menu",
        "zoomToVehicleToolStripMenuItem"
        | "zoomToMissionToolStripMenuItem"
        | "zoomToHomeToolStripMenuItem" => "zoom icon menu",
        _ => "planning panel",
    }
}

/// The missing rows counted by group, largest first.
#[cfg(test)]
fn missing_groups() -> Vec<(&'static str, usize)> {
    let mut groups: Vec<(&'static str, usize)> = Vec::new();
    for action in FLIGHTPLANNER {
        if action.ours != Ours::Missing {
            continue;
        }
        let group = group_of(action.control);
        match groups.iter_mut().find(|(name, _)| *name == group) {
            Some((_, count)) => *count += 1,
            None => groups.push((group, 1)),
        }
    }
    groups.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(b.0)));
    groups
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::{MAP_MENU, MenuEntry, menu_entries};

    /// Where the committed report lives, relative to this crate.
    const REPORT: &str = "../../docs/coverage/flightplanner.md";

    /// This crate's source, for checking that a claimed id or function exists.
    const SOURCES: &[&str] = &[
        include_str!("plan.rs"),
        include_str!("main.rs"),
        include_str!("mapview.rs"),
        include_str!("coords.rs"),
    ];

        fn reference(name: &str) -> Option<String> {
        crate::config_coverage::source::csharp(&format!("GCSViews/{name}"))
    }

    /// `this.X.Y += new Z(this.H);` → (X, Y, H). `this.Y += ...` is the form itself, named
    /// `FlightPlanner` here.
    fn wirings(designer: &str) -> Vec<(String, String, String)> {
        designer
            .lines()
            .filter_map(|line| {
                let line = line.trim();
                let (left, right) = line.split_once(" += new ")?;
                let left = left.strip_prefix("this.")?;
                let (control, event) = left.split_once('.').unwrap_or(("FlightPlanner", left));
                let handler = right.rsplit_once("(this.")?.1.strip_suffix(");")?;
                Some((control.to_owned(), event.to_owned(), handler.to_owned()))
            })
            .collect()
    }

    /// The items a `this.X.Items.AddRange` or `this.X.DropDownItems.AddRange` adds, in order.
    fn added_to(designer: &str, owner: &str) -> Option<Vec<String>> {
        let opener = [
            format!("this.{owner}.Items.AddRange("),
            format!("this.{owner}.DropDownItems.AddRange("),
        ];
        let start = opener
            .iter()
            .find_map(|opener| designer.find(opener.as_str()))?;
        let rest = &designer[start..];
        let body = &rest[rest.find('{')? + 1..rest.find('}')?];
        Some(
            body.split(',')
                .filter_map(|item| item.trim().strip_prefix("this."))
                .map(ToOwned::to_owned)
                .collect(),
        )
    }

    /// The `Text` of every control in the `.resx`.
    fn resx_texts(resx: &str) -> std::collections::BTreeMap<String, String> {
        let mut texts = std::collections::BTreeMap::new();
        for data in resx.split("<data name=\"").skip(1) {
            let Some((name, rest)) = data.split_once('"') else {
                continue;
            };
            let Some(control) = name.strip_suffix(".Text") else {
                continue;
            };
            let Some(value) = rest
                .split_once("<value>")
                .and_then(|(_, value)| value.split_once("</value>"))
            else {
                continue;
            };
            texts.insert(control.to_owned(), value.0.to_owned());
        }
        texts
    }

    /// Every wiring in the Designer has one row, with the same handler, and nothing else does.
    #[test]
    fn every_designer_wiring_has_exactly_one_row() {
        let Some(designer) = reference("FlightPlanner.Designer.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let mut wired = wirings(&designer);
        wired.sort();
        wired.dedup();
        assert_eq!(wired.len(), 121, "the Designer wires 121 events");
        let mut ours: Vec<(String, String, String)> = FLIGHTPLANNER
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
        for (control, event, handler) in &wired {
            assert!(
                ours.iter()
                    .any(|(c, e, h)| c == control && e == event && h == handler),
                "no row for this.{control}.{event} -> {handler}"
            );
        }
        for (control, event, handler) in &ours {
            assert!(
                wired
                    .iter()
                    .any(|(c, e, h)| c == control && e == event && h == handler),
                "row {control}.{event} -> {handler} is not in the Designer"
            );
        }
        let before = ours.len();
        ours.dedup();
        assert_eq!(ours.len(), before, "a wiring has two rows");
        assert_eq!(ours.len(), wired.len());
    }

    /// A row's text is the control's `.resx` text where the control is a button, a check box, a
    /// link or a menu item - the ones whose text is what the operator reads.
    #[test]
    fn a_labelled_control_carries_its_resx_text() {
        let Some(resx) = reference("FlightPlanner.resx") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let texts = resx_texts(&resx);
        let mut checked = 0;
        for action in FLIGHTPLANNER {
            let labelled = action.control.ends_with("ToolStripMenuItem")
                || action.control.ends_with("ToolStripMenuItem1")
                || action.control.ends_with("ToolStripMenuItem2")
                || action.control == "ContextMeasure"
                || ["BUT_", "but_", "chk_", "CHK_", "lnk_", "label4"]
                    .iter()
                    .any(|prefix| action.control.starts_with(prefix));
            if !labelled {
                continue;
            }
            let text = texts
                .get(action.control)
                .unwrap_or_else(|| panic!("{} has no Text in FlightPlanner.resx", action.control));
            assert_eq!(action.text, text, "{}", action.control);
            checked += 1;
        }
        assert!(checked >= 80, "only {checked} labelled rows were checked");
    }

    /// Nothing is claimed that the source does not have.
    #[test]
    fn every_claimed_id_exists_in_this_crate() {
        for action in FLIGHTPLANNER {
            let Ours::Done(id) = action.ours else {
                continue;
            };
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

    /// A row claiming a menu entry names one the menu draws live, with the same handler's
    /// control or one that runs the same handler.
    #[test]
    fn a_claimed_menu_entry_is_live_in_the_menu() {
        for action in FLIGHTPLANNER {
            let Ours::Done(id) = action.ours else {
                continue;
            };
            if !id.starts_with("menu-") {
                continue;
            }
            let entry = menu_entries().find(|entry| entry.id == id);
            let Some(entry) = entry else {
                panic!(
                    "{} claims {id}, which the menu does not have",
                    action.control
                );
            };
            assert!(
                entry.action.is_some(),
                "{} claims {id}, which the menu draws dimmed",
                action.control
            );
            // The entry is this control, or another the Designer wires to the same handler.
            let same_handler = FLIGHTPLANNER
                .iter()
                .any(|other| other.control == entry.control && other.handler == action.handler);
            assert!(
                entry.control == action.control || same_handler,
                "{} claims {id}, which is {} and runs another handler",
                action.control,
                entry.control
            );
        }
    }

    /// And the other way: every live menu entry is claimed by its own row.
    #[test]
    fn every_live_menu_entry_is_counted_as_done() {
        for entry in menu_entries().filter(|entry| entry.action.is_some()) {
            let row = FLIGHTPLANNER
                .iter()
                .find(|action| action.control == entry.control)
                .unwrap_or_else(|| panic!("{} has no row", entry.control));
            assert_eq!(row.ours, Ours::Done(entry.id), "{}", entry.control);
        }
    }

    /// The menu is the Designer's: `contextMenuStrip1.Items.AddRange` order, each drop-down in its
    /// `DropDownItems.AddRange` order, with the `.resx` text on every entry.
    #[test]
    fn the_map_menu_is_in_the_designers_order_with_its_resx_text() {
        let (Some(designer), Some(resx)) = (
            reference("FlightPlanner.Designer.cs"),
            reference("FlightPlanner.resx"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let texts = resx_texts(&resx);
        let check = |owner: &str, entries: &[MenuEntry]| {
            let added = added_to(&designer, owner)
                .unwrap_or_else(|| panic!("{owner} adds nothing in the Designer"));
            let ours: Vec<&str> = entries.iter().map(|entry| entry.control).collect();
            assert_eq!(ours, added, "{owner}'s entries");
            for entry in entries {
                if entry.is_separator() {
                    continue;
                }
                assert_eq!(
                    Some(entry.text),
                    texts.get(entry.control).map(String::as_str),
                    "{}",
                    entry.control
                );
                let id = entry.control.replace("ToolStripMenuItem", "");
                assert_eq!(entry.id, format!("menu-{id}"), "{}'s id", entry.control);
            }
        };
        check("contextMenuStrip1", MAP_MENU);
        for entry in MAP_MENU.iter().filter(|entry| !entry.children.is_empty()) {
            check(entry.control, entry.children);
        }
        // Entries with a drop-down in the Designer have one here, and no others do.
        for entry in MAP_MENU {
            assert_eq!(
                added_to(&designer, entry.control).is_some(),
                !entry.children.is_empty(),
                "{}",
                entry.control
            );
        }
    }

    /// The zoom icon's menu is the Designer's `contextMenuStripZoom`: its entries in
    /// `Items.AddRange` order with the `.resx` text, each live.
    #[test]
    fn the_zoom_menu_is_the_designers_with_its_resx_text() {
        let (Some(designer), Some(resx)) = (
            reference("FlightPlanner.Designer.cs"),
            reference("FlightPlanner.resx"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let texts = resx_texts(&resx);
        let added = added_to(&designer, "contextMenuStripZoom").expect("contextMenuStripZoom");
        let ours: Vec<&str> = crate::plan::ZOOM_MENU
            .iter()
            .map(|entry| entry.control)
            .collect();
        assert_eq!(ours, added);
        for entry in crate::plan::ZOOM_MENU {
            assert_eq!(
                Some(entry.text),
                texts.get(entry.control).map(String::as_str),
                "{}",
                entry.control
            );
            let id = entry.control.replace("ToolStripMenuItem", "");
            assert_eq!(entry.id, format!("menu-{id}"));
            assert!(entry.is_live(), "{}", entry.control);
        }
    }

    /// The committed report matches the table.
    #[test]
    fn the_committed_report_is_current() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        let committed = mp_os::fs::read_to_string(&path).unwrap_or_default();
        assert!(
            committed == report(),
            "docs/coverage/flightplanner.md is stale; run `cargo test -p mp-gui planner_coverage -- --ignored update_report`"
        );
    }

    /// Rewrites the report. Run on purpose, not on every test.
    #[test]
    #[ignore = "writes docs/coverage/flightplanner.md"]
    fn update_report() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        mp_os::fs::write(&path, report()).expect("write the report");
    }

    /// The count is the recorded one, so a change in either direction is a deliberate edit.
    #[test]
    fn the_counts_are_the_ones_recorded() {
        let (done, elsewhere, missing, plumbing, dropped) = counts();
        assert_eq!(
            done + elsewhere + missing + plumbing + dropped,
            FLIGHTPLANNER.len()
        );
        eprintln!(
            "FlightPlanner: {done} done, {elsewhere} elsewhere, {missing} missing, {plumbing} plumbing, {dropped} dropped"
        );
        assert_eq!(
            (done, elsewhere, missing, plumbing, dropped),
            (107, 0, 2, 12, 0)
        );
    }
}
