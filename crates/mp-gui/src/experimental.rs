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

//! The EXPERIMENTAL tab: Mission Planner's temp form, `temp.cs`, which Ctrl+F opens
//! (MainV2.ProcessCmdKey's `new temp().Show()`, MainV2.cs:4099-4105) and the welcome text lists as
//! "Control-F - Temp screen". By the owner's word (2026-10-04) a tab of its own, EXPERIMENTAL
//! between LOGS and PLUGINS, which Ctrl+F shows, where the C# opens a form.
//!
//! The form is `tableLayoutPanel1`: four columns, a button and what it does twice over, 16.375,
//! 33.625, 19.25 and 30.75 per cent of its width, and 32 rows of equal height (3.125 per cent
//! each); every control's cell and text are temp.Designer.cs's and temp.resx's ([`CELLS`]).
//!
//! Each of the 63 buttons is one of ([`tool`]):
//! * a window this application has, opened as CONFIG > Advanced's button for the same tool opens
//!   it (`MissionPlanner::open_advanced_tool`): Warning Manager, NMEA, Mavlink, MAVLink Inspector,
//!   Param gen, FFT, signing, Proximity; and Geo ref images (`georef_ui::open`);
//! * out of scope by a ruling, dimmed, its press saying why on the status line: Follow Me, OSDVideo,
//!   Moving Base and the four Swarm tools (PLAN.md section 12 D13, 2026-09-25), Lang Edit (the
//!   translation editor, with languages muted, 2026-09-25), Custom GDAL (no GDAL bindings, the
//!   matrix's GDAL row);
//! * not ported yet, dimmed, its press saying so - the matrix's EXPERIMENTAL row lists them, to be
//!   ported one by one.
//!
//! `// C#: temp.cs:1-1434; temp.Designer.cs:31-1123; temp.resx; MainV2.cs:4099-4105`

use gpui::{
    AnyElement, Context, InteractiveElement, IntoElement, ParentElement, SharedString,
    StatefulInteractiveElement, Styled, Window, div, px, relative, rgb,
};

use crate::{MissionPlanner, facts, theme};

/// `tableLayoutPanel1`'s cells, row by row: (row, column, the control's name, its text, whether it
/// is a button). Generated from temp.Designer.cs (`Controls.Add(control, column, row)`) and
/// temp.resx (each `.Text`). `// C#: temp.Designer.cs:162-282; temp.resx`
const CELLS: &[(u8, u8, &str, &str, bool)] = &[
    (0, 0, "BUT_georefimage", "Geo ref images", true),
    (0, 1, "label1", "moved to dataflash tab", false),
    (0, 2, "but_hexmavlink", "hex Mavlink decode", true),
    (1, 0, "button3", "Warning Manager", true),
    (1, 1, "label2", "Create custom audio warnings", false),
    (1, 2, "but_driverclean", "driver clean", true),
    (1, 3, "label30", "remove installed drivers", false),
    (2, 0, "BUT_follow_me", "Follow Me", true),
    (2, 1, "label3", "use a nmea gps to follow me", false),
    (2, 2, "but_disablearmswitch", "Toggle Safety Switch", true),
    (2, 3, "label31", "virtual press the safety button", false),
    (3, 0, "BUT_outputnmea", "NMEA", true),
    (3, 1, "label4", "outputs the mav location in nmea", false),
    (3, 2, "but_messageinterval", "Message Interval", true),
    (3, 3, "label32", "set custom message intervals for messages", false),
    (4, 0, "BUT_outputMD", "MicroDrone", true),
    (4, 1, "label5", "outputs the mav location in microdrone format", false),
    (4, 2, "but_mavinspector", "MAVLink Inspector", true),
    (4, 3, "label33", "Inspect all mavlink packets being transmitted", false),
    (5, 0, "BUT_outputMavlink", "Mavlink", true),
    (5, 1, "label6", "mirrors the mavlink stream received by mp", false),
    (5, 2, "but_blupdate", "Bootloader Upgrade", true),
    (5, 3, "label34", "update the bootloader", false),
    (6, 0, "BUT_paramgen", "Param gen", true),
    (6, 1, "label7", "regenerate the param info used inside mp", false),
    (6, 2, "but_3dmap", "3D Map", true),
    (6, 3, "label35", "3d map testing", false),
    (7, 0, "BUT_lang_edit", "Lang Edit", true),
    (7, 1, "label8", "translation language editor", false),
    (7, 2, "but_hwids", "decode HWIDs", true),
    (7, 3, "label36", "display info about a hardware id typed in", false),
    (8, 0, "but_osdvideo", "OSDVideo", true),
    (8, 1, "label9", "overlay the hud into your recorded videos", false),
    (8, 2, "but_packetbytes", "parse packet bytes", true),
    (8, 3, "label37", "debug a hex string mavlink packet", false),
    (9, 0, "BUT_movingbase", "Moving Base", true),
    (9, 1, "label10", "show an extra icon on the map of your current location.", false),
    (9, 2, "but_acbarohight", "adjust aircraft baro height", true),
    (9, 3, "label38", "modify baro alt reference alt", false),
    (10, 0, "BUT_shptopoly", "Shp to Poly", true),
    (10, 1, "label11", "convert shp file to a polygon file", false),
    (10, 2, "but_lockup", "Lockup MAV", true),
    (10, 3, "label39", "cause the autopilot to lockup", false),
    (11, 0, "but_anonlog", "Anon Log", true),
    (11, 2, "but_dem", "DEM", true),
    (11, 3, "label40", "display information about the currently loaded DEMs", false),
    (12, 0, "BUT_swarm", "Swarm", true),
    (12, 1, "label13", "multi mav swarm interface", false),
    (12, 2, "but_logdlscp", "logdownload scp", true),
    (12, 3, "label41", "logdownload via scp - ssh (apsync)", false),
    (13, 0, "BUT_followleader", "Follow the leader", true),
    (13, 1, "label14", "follow the leader swarm", false),
    (13, 2, "but_sortlogs", "ReSort All logs", true),
    (13, 3, "label42", "resort all the logs in the MP logging folder", false),
    (14, 0, "but_mavserialport", "MAVSerial pass", true),
    (14, 1, "label15", "create a exclusive passthrough to the gps (port 500)", false),
    (14, 2, "but_GDAL", "Custom GDAL", true),
    (14, 3, "label43", "load a custom map tile source via GDAL", false),
    (15, 0, "but_remotedflogger", "Start Remote df logger", true),
    (15, 2, "but_sitl_comb", "sitl streamcombiner", true),
    (16, 0, "BUT_sorttlogs", "Sort TLogs", true),
    (16, 1, "label17", "Sort TLogs into their type and sysid directories", false),
    (16, 2, "but_paramrestore", "Param Restore", true),
    (16, 3, "label44", "....", false),
    (17, 0, "but_getfw", "rip all fw", true),
    (17, 1, "label18", "download all current fw's", false),
    (17, 2, "BUT_fft", "FFT", true),
    (17, 3, "label45", "....", false),
    (18, 0, "BUT_geinjection", "Inject GE", true),
    (18, 1, "label19", "add custom imagery to mp", false),
    (18, 2, "but_td", "grab threads.txt", true),
    (18, 3, "label46", "...", false),
    (19, 0, "BUT_clearcustommaps", "Clear Custom Maps", true),
    (19, 1, "label20", "wipe custom imagery", false),
    (19, 2, "but_reboot", "reboot pixhawk", true),
    (19, 3, "label47", "reboot the autopilot", false),
    (20, 0, "but_structtest", "structtest", true),
    (20, 1, "label21", "struct conversion speed test", false),
    (20, 2, "BUT_QNH", "QNH", true),
    (20, 3, "label48", "adjust the qnh", false),
    (21, 0, "but_dashware", "DashWare", true),
    (21, 1, "label22", "Create dashware date input file", false),
    (21, 2, "but_trimble", "Sequence Swarm", true),
    (21, 3, "label49", "label49", false),
    (22, 0, "but_armandtakeoff", "arm and takeoff", true),
    (22, 1, "label23", "quad: arm and takeoff", false),
    (22, 2, "myButton_vlc", "vlc", true),
    (22, 3, "label50", "display video stream via vlc - USE Gstream instead", false),
    (23, 0, "but_gimbaltest", "gimbal test", true),
    (23, 1, "label24", "run the gimbal pointing algo", false),
    (23, 2, "but_agemapdata", "Age Map Data", true),
    (23, 3, "label51", "remove image tiles older than 30 days", false),
    (24, 0, "but_maplogs", "map logs", true),
    (24, 1, "label25", "create map JPGs for all TLogs in a dir", false),
    (24, 2, "myButton1", "Split DFLog", true),
    (24, 3, "label52", "split dflog into x pieces", false),
    (25, 0, "butlogindex", "logindex", true),
    (25, 1, "label26", "tlog browser", false),
    (25, 2, "but_signkey", "signing", true),
    (25, 3, "label16", "mavlink2 signing configuration", false),
    (26, 0, "but_optflowcalib", "opticalflow calib", true),
    (26, 1, "label29", "display the image data from the px4 optical flow sensor", false),
    (26, 2, "but_gpsinj", "extract gps_inject", true),
    (26, 3, "label53", "extract rtcm data from tlog", false),
    (27, 0, "but_apjtool", "APJ Tool", true),
    (27, 2, "but_proximity", "Proximity", true),
    (27, 3, "label54", "display the proximity ui", false),
    (28, 0, "BUT_magfit2", "mag calb log", true),
    (28, 1, "label27", "get mag offsets from a log", false),
    (28, 2, "but_followswarm", "Follow Swarm", true),
    (28, 3, "label55", "swarm style", false),
    (29, 0, "BUT_CoT", "CoT", true),
    (29, 1, "label12", "Outputs Cursor-on-Target", false),
    (29, 2, "but_ManageCMDList", "Manage Command List", true),
    (29, 3, "label28", "Manage Planner's Command List", false),
    (30, 0, "BUT_forcecal_accel", "Force Accel Cal", true),
    (30, 1, "label57", "Mark accel as cal'd after param restore", false),
    (30, 2, "but_dfumode", "DFU Mode", true),
    (30, 3, "label56", "DFU Mode", false),
    (31, 0, "BUT_forcecal_mag", "Force Compass Cal", true),
    (31, 1, "label58", "Mark mag as cal'd after param restore", false),
];

/// `tableLayoutPanel1`'s column widths, per cent of its width (temp.resx's `ColumnStyles`).
const COLUMNS: [f32; 4] = [16.375, 33.625, 19.25, 30.75];
/// Its rows: 32, of equal height; each drawn this tall, so its words can be read.
const ROWS: u8 = 32;
const ROW_HEIGHT: f32 = 28.0;

/// The owner's rulings a dimmed button cites.
const SECTION_12_D13: &str = "out of scope, the owner's ruling (PLAN.md section 12 D13, 2026-09-25)";
const LANGUAGES_MUTED: &str =
    "out of scope: translation is muted by the owner's ruling (2026-09-25)";
const NO_GDAL: &str =
    "not available: this application has no GDAL (NOT_DONE_YET_MATRIX.md, the GDAL row)";
const NOT_PORTED: &str = "not ported yet (NOT_DONE_YET_MATRIX.md, the EXPERIMENTAL row)";

/// What a button does here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Tool {
    /// Opens a window this application has, as CONFIG > Advanced's button of this name opens it.
    Advanced(&'static str),
    /// Geo ref images: `new Georefimage().Show()`.
    Georef,
    /// Not here, and why.
    Unavailable(&'static str),
}

/// The button named `name` (the C#'s control name), here.
/// `// C#: temp.cs (each button's _Click)`
#[must_use]
pub(crate) fn tool(name: &str) -> Tool {
    match name {
        // `button3_Click`: `new WarningsManager().Show()`.
        "button3" => Tool::Advanced("but_warningmanager"),
        "BUT_outputnmea" => Tool::Advanced("BUT_outputnmea"),
        // `BUT_outputMavlink_Click`: `new SerialOutputPass().Show()`.
        "BUT_outputMavlink" => Tool::Advanced("BUT_outputMavlink"),
        "but_mavinspector" => Tool::Advanced("but_mavinspector"),
        "BUT_paramgen" => Tool::Advanced("BUT_paramgen"),
        // `BUT_fft_Click`: `new fftui().Show()`, Advanced's `but_fft`.
        "BUT_fft" => Tool::Advanced("but_fft"),
        "but_signkey" => Tool::Advanced("but_signkey"),
        "but_proximity" => Tool::Advanced("but_proximity"),
        "BUT_georefimage" => Tool::Georef,
        "BUT_follow_me" | "but_osdvideo" | "BUT_movingbase" | "BUT_swarm" | "BUT_followleader"
        | "but_trimble" | "but_followswarm" => Tool::Unavailable(SECTION_12_D13),
        "BUT_lang_edit" => Tool::Unavailable(LANGUAGES_MUTED),
        "but_GDAL" => Tool::Unavailable(NO_GDAL),
        _ => Tool::Unavailable(NOT_PORTED),
    }
}

/// The tab's state: the last button pressed, for the facts.
#[derive(Debug, Default)]
pub(crate) struct Experimental {
    last: Option<&'static str>,
}

/// A button pressed: its window opened, else why not on the status line.
fn press(
    this: &mut MissionPlanner,
    name: &'static str,
    text: &'static str,
    window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    window.blur(cx);
    this.experimental.last = Some(name);
    match tool(name) {
        Tool::Advanced(button) => {
            this.open_advanced_tool(button, window, cx);
        }
        Tool::Georef => crate::georef_ui::open(this),
        Tool::Unavailable(why) => this.file_status = Some(format!("{text}: {why}")),
    }
}

/// One cell of the table: a button, its words, or nothing.
fn cell(
    found: Option<&'static (u8, u8, &'static str, &'static str, bool)>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let Some(&(_, _, name, text, is_button)) = found else {
        return div().into_any_element();
    };
    if !is_button {
        return div()
            .h_full()
            .px_2()
            .flex()
            .items_center()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .child(text)
            .into_any_element();
    }
    let available = !matches!(tool(name), Tool::Unavailable(_));
    let id = SharedString::from(format!("experimental-{name}"));
    crate::probe::measured(id.clone(), div())
        .id(id)
        .h_full()
        .mx_1()
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .cursor_pointer()
        .bg(rgb(if available { theme::ACTION } else { theme::PANEL }))
        .text_color(rgb(if available { theme::TEXT } else { theme::DIM }))
        .hover(|style| style.border_color(rgb(theme::ACCENT)))
        .child(text)
        .on_click(cx.listener(move |this, _event, window, cx| {
            press(this, name, text, window, cx);
            cx.notify();
        }))
        .into_any_element()
}

/// The tab: the table, and the windows its buttons open over it.
/// `// C#: temp.Designer.cs:31-1123`
pub(crate) fn screen(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut table = div().flex().flex_col().w_full().gap_1();
    for row in 0..ROWS {
        let mut line = div().flex().w_full().h(px(ROW_HEIGHT));
        for (column, width) in (0u8..).zip(COLUMNS) {
            let found = CELLS
                .iter()
                .find(|&&(r, c, ..)| r == row && c == column);
            line = line.child(div().w(relative(width / 100.0)).h_full().child(cell(found, cx)));
        }
        table = table.child(line);
    }
    div()
        .id("experimental-body")
        .flex()
        .flex_col()
        .flex_1()
        .min_h(px(0.0))
        .overflow_y_scroll()
        .p_3()
        .child(table)
        // The windows the buttons open, over the tab as over the pages that open them elsewhere.
        .children(this.extra_setup_overlay(window, cx))
        .children(crate::georef_ui::window(this, window, cx))
        .into_any_element()
}

/// The facts a script asserts on: the buttons, how many work here, the last pressed.
pub(crate) fn record_facts(state: &Experimental) {
    let buttons: Vec<&str> = CELLS
        .iter()
        .filter(|cell| cell.4)
        .map(|cell| cell.2)
        .collect();
    let working = buttons
        .iter()
        .filter(|name| !matches!(tool(name), Tool::Unavailable(_)))
        .count();
    facts::record("experimental.buttons", buttons.len());
    facts::record("experimental.working", working);
    facts::record("experimental.last", state.last.unwrap_or("none"));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The table as the designer has it: 121 controls, 63 of them buttons, in four columns and
    /// 32 rows, no two in one cell.
    #[test]
    fn the_table_is_the_designers() {
        assert_eq!(CELLS.len(), 121);
        assert_eq!(CELLS.iter().filter(|cell| cell.4).count(), 63);
        let mut places: Vec<(u8, u8)> = CELLS.iter().map(|cell| (cell.0, cell.1)).collect();
        assert!(places.iter().all(|&(row, column)| row < ROWS && column < 4));
        places.sort_unstable();
        places.dedup();
        assert_eq!(places.len(), CELLS.len());
        let total: f32 = COLUMNS.iter().sum();
        assert!((total - 100.0).abs() < 1e-3);
    }

    /// Every button that opens a window opens one the Advanced page knows by that name; the
    /// ruled-out ones cite their rulings; the rest are not ported yet.
    #[test]
    fn every_button_is_classified() {
        let advanced: Vec<&str> = crate::config::advanced::ROWS
            .iter()
            .map(|row| row.button)
            .collect();
        let mut opens = 0;
        for &(_, _, name, text, is_button) in CELLS {
            if !is_button {
                continue;
            }
            match tool(name) {
                Tool::Advanced(button) => {
                    assert!(advanced.contains(&button), "{name} ({text}): {button}");
                    opens += 1;
                }
                Tool::Georef => opens += 1,
                Tool::Unavailable(why) => assert!(!why.is_empty()),
            }
        }
        assert_eq!(opens, 9);
        assert_eq!(tool("BUT_swarm"), Tool::Unavailable(SECTION_12_D13));
        assert_eq!(tool("but_GDAL"), Tool::Unavailable(NO_GDAL));
        assert_eq!(tool("but_reboot"), Tool::Unavailable(NOT_PORTED));
    }

    /// The words as Mission Planner shows them: the first row's, and the last button's.
    #[test]
    fn the_words_are_the_resx() {
        assert!(CELLS.contains(&(0, 0, "BUT_georefimage", "Geo ref images", true)));
        assert!(CELLS.contains(&(31, 0, "BUT_forcecal_mag", "Force Compass Cal", true)));
        assert!(CELLS
            .iter()
            .any(|cell| cell.3 == "Outputs Cursor-on-Target" && !cell.4));
    }
}
