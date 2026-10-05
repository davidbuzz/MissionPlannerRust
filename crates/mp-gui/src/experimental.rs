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
//! * a command to the vehicle, ported here with its questions ([`Act`]): reboot pixhawk ("Are you
//!   sure?", `doReboot(false, true)`), Force Accel Cal and Force Compass Cal (`PREFLIGHT_CALIBRATION`
//!   with 76 as param 5 or param 2), DFU Mode (`doDFUBoot`), QNH (an `InputBox` for
//!   GND_ABS_PRESS, else BARO1_GND_PRESS), Lockup MAV (asked twice), arm and takeoff (Stabilize,
//!   armed, Guided and a take-off to 10 m, each in turn), Bootloader Upgrade (the "BL
//!   Update" questions, then `FLASH_BOOTLOADER`), Toggle Safety Switch ("Are you sure?", then the
//!   flight screen's Toggle_Safety_Switch, the same `setMode` with `SAFETY_ARMED`); what the C#
//!   shows in a box when one fails is said on the status line, by the owner's ruling; and decode
//!   HWIDs, the ids typed taken apart as `Device.DeviceStructure` does, in a box (one line of ids
//!   here, where the C#'s box takes several);
//! * a tool with files: Param Restore (a parameter file written as `but_paramrestore_Click` writes
//!   it), mag calb log (`MagCalib.ProcessLog`: a log read and fitted, `magoffset.dxf` drawn, and
//!   the offsets to the compass page's `SaveOffsets`) and Split DFLog (`DFLogBuffer.SplitLog`);
//! * the map cache's two: Clear Custom Maps (every Custom tile) and Age Map Data (the map's
//!   provider's tiles older than thirty days), each "Removed N images" in a box;
//! * out of scope by a ruling, dimmed, its press saying why on the status line: Follow Me, OSDVideo,
//!   Moving Base and the four Swarm tools (PLAN.md section 12 D13, 2026-09-25), Anon Log (the same
//!   section, 2026-10-02: `Privacy.anonymise`, "beta and not interesting"), Lang Edit (the
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

use crate::config::firmware::{
    BL_QUESTIONS, BL_UPDATE, BoxIds, DO_COMMAND_TIMEOUT, FAILED_TO_UPGRADE_BOOTLOADER,
    UPGRADED_BOOTLOADER, Waiting, message_box, question_box,
};
use crate::config::optional::{InputBox, input_box};
use crate::fly::{PLEASE_CONNECT, error_box};
use crate::telemetry::Report;
use crate::{MissionPlanner, facts, theme};
use mp_log::magcal_log::{DXF_NAME, Processed, process_log};
use mp_firmware::flow::Buttons;
use mp_link::requests::CMD_FLASH_BOOTLOADER;

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
/// Its rows: 32 of equal height, sharing the tab's height as the form's 3.125 per cent rows share
/// its height - so the whole table shows, as the C#'s does - but none shorter than its words need;
/// below that the tab scrolls, with the indicator saying so. (Found by the owner, 2026-10-04: rows
/// of a fixed 28 pixels ran off the bottom of a smaller window, and nothing showed there was more.)
const ROWS: u8 = 32;
const MIN_ROW_HEIGHT: f32 = 20.0;
/// The space between rows.
const ROW_GAP: f32 = 2.0;

/// The owner's rulings a dimmed button cites.
const SECTION_12_D13: &str = "out of scope, the owner's ruling (PLAN.md section 12 D13, 2026-09-25)";
const LANGUAGES_MUTED: &str =
    "out of scope: translation is muted by the owner's ruling (2026-09-25)";
const ANON_LOG_RULED: &str = "out of scope, the owner's ruling (PLAN.md section 12 D13, 2026-10-02: \
     beta and not interesting)";
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
    /// Message Interval's form (message_interval.rs).
    MessageInterval,
    /// A command to the vehicle, ported here.
    Act(Act),
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
        "but_messageinterval" => Tool::MessageInterval,
        "but_paramrestore" => Tool::Act(Act::ParamRestore),
        "but_reboot" => Tool::Act(Act::Reboot),
        "BUT_forcecal_accel" => Tool::Act(Act::ForceAccelCal),
        "BUT_forcecal_mag" => Tool::Act(Act::ForceCompassCal),
        "but_dfumode" => Tool::Act(Act::DfuMode),
        "BUT_QNH" => Tool::Act(Act::Qnh),
        "but_lockup" => Tool::Act(Act::Lockup),
        "but_hwids" => Tool::Act(Act::DecodeHwids),
        "but_blupdate" => Tool::Act(Act::BootloaderUpgrade),
        "but_disablearmswitch" => Tool::Act(Act::ToggleSafety),
        "BUT_magfit2" => Tool::Act(Act::MagCalLog),
        "myButton1" => Tool::Act(Act::SplitDfLog),
        "BUT_clearcustommaps" => Tool::Act(Act::ClearCustomMaps),
        "but_agemapdata" => Tool::Act(Act::AgeMapData),
        "but_armandtakeoff" => Tool::Act(Act::ArmAndTakeoff),
        "BUT_follow_me" | "but_osdvideo" | "BUT_movingbase" | "BUT_swarm" | "BUT_followleader"
        | "but_trimble" | "but_followswarm" => Tool::Unavailable(SECTION_12_D13),
        "but_anonlog" => Tool::Unavailable(ANON_LOG_RULED),
        "BUT_lang_edit" => Tool::Unavailable(LANGUAGES_MUTED),
        "but_GDAL" => Tool::Unavailable(NO_GDAL),
        _ => Tool::Unavailable(NOT_PORTED),
    }
}

/// The temp form's commands to the vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Act {
    /// `but_reboot_Click`: "Are you sure?", then `doReboot(false, true)`. `// C#: temp.cs:662-666`
    Reboot,
    /// `BUT_forcecal_accel_Click`: `PREFLIGHT_CALIBRATION` with 76 as param 5, the accelerometers
    /// marked calibrated. `// C#: temp.cs:1404-1416`
    ForceAccelCal,
    /// `BUT_forcecal_mag_Click`: `PREFLIGHT_CALIBRATION` with 76 as param 2, the compasses.
    /// `// C#: temp.cs:1420-1432`
    ForceCompassCal,
    /// `but_dfumode_Click`: `doDFUBoot`, `PREFLIGHT_REBOOT_SHUTDOWN` 42, 24, 71, 99 not waited for.
    /// `// C#: temp.cs:1397-1400; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2511-2517`
    DfuMode,
    /// `BUT_QNH_Click`: the QNH asked for, offered as it is, and set. `// C#: temp.cs:668-682`
    Qnh,
    /// `but_lockup_Click`: asked twice, then `PREFLIGHT_REBOOT_SHUTDOWN` 42, 24, 71, 93 not waited
    /// for. `// C#: temp.cs:1230-1239`
    Lockup,
    /// `but_hwids_Click`: ids asked for, each taken apart as `Device.DeviceStructure` does.
    /// `// C#: temp.cs:1120-1148`
    DecodeHwids,
    /// `but_paramrestore_Click`: "This process make take a some time", a parameter file asked
    /// for, and its parameters written as Param Restore writes them (params.rs's `restore`).
    /// `// C#: temp.cs:1267-1362`
    ParamRestore,
    /// `but_blupdate_Click`: the two "BL Update" questions, then `FLASH_BOOTLOADER` with 290876 as
    /// param 5, waited for: "Upgraded bootloader" or "Failed to upgrade bootloader".
    /// `// C#: temp.cs:969-993`
    BootloaderUpgrade,
    /// `but_disablearmswitch_Click`: "Are you sure?", then `setMode` with `SAFETY_ARMED` and the
    /// motor outputs' state as the custom mode - the flight screen's Toggle_Safety_Switch, which
    /// is the same code. `// C#: temp.cs:1105-1118; GCSViews/FlightData.cs:1819-1830`
    ToggleSafety,
    /// `BUT_magfit2_Click`: `MagCalib.ProcessLog(0)` - a log asked for, read and fitted off the
    /// window's thread (magcal_log's `process_log`), `magoffset.dxf` drawn, and the offsets
    /// handed to `SaveOffsets` (the compass page's, its boxes over every screen).
    /// `// C#: temp.cs:410-413; MagCalib.cs:93-133`
    MagCalLog,
    /// `myButton1_Click_2`: a log asked for, "How Many" pieces asked (10 offered), and
    /// `DFLogBuffer.SplitLog` writing `<log>_split<i>.bin` beside it, off the window's thread
    /// (mp-log's `split_file`). `// C#: temp.cs:720-734; ExtLibs/Utilities/DFLogBuffer.cs:417-501`
    SplitDfLog,
    /// `BUT_clearcustommaps_Click`: every tile of the Custom provider - the imagery Inject GE and
    /// Inject Custom Map put in the cache - deleted (`DeleteOlderThan(DateTime.Now, Custom)`), and
    /// "Removed N images" in a box. `// C#: temp.cs:149-161; ExtLibs/Maps/MyImageCache.cs:132-184`
    ClearCustomMaps,
    /// `but_agemapdata_Click`: the flight map's provider's tiles older than thirty days deleted,
    /// and "Removed N images" in a box. `// C#: temp.cs:710-718`
    AgeMapData,
    /// `but_armandtakeoff_Click`: `setMode("Stabilize")`, then `doARM(true)` waited for; armed,
    /// `setMode("GUIDED")`, 300 ms, and `doCommand(TAKEOFF)` to 10 m, waited for, its answer not
    /// read. A refusal to arm ends it quietly, as `doARM`'s false does; a command never answered
    /// is the `catch`'s box, on the status line. `// C#: temp.cs:614-634`
    ArmAndTakeoff,
}

/// arm and takeoff's steps, which the C#'s handler blocks on in turn.
#[derive(Debug, Clone, Copy)]
enum Takeoff {
    /// `doARM(true)` sent, its answer awaited.
    Arming {
        id: mp_link::RequestId,
        made: web_time::Instant,
        target: mp_vehicle::VehicleId,
    },
    /// Armed and Guided asked for: `Thread.Sleep(300)`, until `until`.
    Sleeping {
        until: web_time::Instant,
        target: mp_vehicle::VehicleId,
    },
    /// The take-off sent, its answer awaited - for the facts; the C# does not read it.
    TakingOff {
        id: mp_link::RequestId,
        made: web_time::Instant,
    },
}

/// `Thread.Sleep(300)` between Guided and the take-off. `// C#: temp.cs:625`
const TAKEOFF_PAUSE: web_time::Duration = web_time::Duration::from_millis(300);
/// The take-off's height, `doCommand(..., TAKEOFF, 0, 0, 0, 0, 0, 0, 10)`. `// C#: temp.cs:627`
const TAKEOFF_ALTITUDE: f32 = 10.0;

/// Param Restore's first box.
/// `// C#: temp.cs:1269`
const RESTORE_NOTICE: &str = "This process make take a some time";
/// `ParamFile.FileMask`, the file dialog's filter.
/// `// C#: ExtLibs/Utilities/ParamFile.cs:15`
const PARAM_FILE_MASK: &str = "Parameter File|*.param;*.parm|All Files|*.*";
/// `ProcessLog`'s dialog's filter. `// C#: MagCalib.cs:97`
const LOG_FILE_MASK: &str = "Log Files|*.tlog;*.log;*.bin";
/// Split DFLog's dialog's filter, and its question with the count it offers.
/// `// C#: temp.cs:723, 730-731`
const DFLOG_FILE_MASK: &str = "Log Files|*.log;*.bin;*.BIN;*.LOG";
const SPLIT_TITLE: &str = "How Many";
const SPLIT_PROMPT: &str = "Enter how many pieces to split into";
const SPLIT_OFFERED: i32 = 10;
/// Age Map Data's age: `DateTime.Now.AddDays(-30)`. `// C#: temp.cs:712`
const AGE_MAP_DATA: web_time::Duration = web_time::Duration::from_secs(30 * 24 * 60 * 60);

/// Clear Custom Maps' and Age Map Data's deletions at `now`, with the map showing the provider
/// `source`: every Custom tile made before now, or the provider's made before thirty days ago.
/// `// C#: temp.cs:154, 712-713`
fn removed_by(
    what: Act,
    tile_root: &std::path::Path,
    source: Option<&str>,
    now: web_time::SystemTime,
) -> usize {
    if what == Act::ClearCustomMaps {
        crate::cmd_keys::delete_older_than(tile_root, mp_tiles::source::CUSTOM.cache_name, now)
    } else {
        source
            .and_then(mp_tiles::source::source_by_id)
            .map_or(0, |source| {
                crate::cmd_keys::delete_older_than(tile_root, source.cache_name, now - AGE_MAP_DATA)
            })
    }
}

/// The box both map tools show, and log. `// C#: temp.cs:156-158, 715-717`
fn removed_images(removed: usize) -> String {
    format!("Removed {removed} images")
}

/// `but_hwids_Click`'s report: for every whole number in each line, the line (its tabs as
/// spaces) and the device that id names, a line each.
/// `// C#: temp.cs:1125-1145`
#[must_use]
fn decode_hwids(value: &str) -> String {
    let mut out = String::new();
    for line in value.split(['\r', '\n']).filter(|line| !line.is_empty()) {
        for piece in line.split([' ', '\t']) {
            if let Ok(id) = piece.parse::<u32>() {
                out.push_str(&format!(
                    "{} = {}\n",
                    line.replace('\t', " "),
                    crate::config::hw_ids::device_structure("", id)
                ));
            }
        }
    }
    out
}

/// `MAV_CMD.PREFLIGHT_CALIBRATION` and `MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN`.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs`
const PREFLIGHT_CALIBRATION: u16 = 241;
const PREFLIGHT_REBOOT_SHUTDOWN: u16 = 246;
/// The commands' parameters, as the C# passes them.
const FORCE_ACCEL: [f32; 7] = [0.0, 0.0, 0.0, 0.0, 76.0, 0.0, 0.0];
const FORCE_COMPASS: [f32; 7] = [0.0, 76.0, 0.0, 0.0, 0.0, 0.0, 0.0];
const DFU_BOOT: [f32; 7] = [42.0, 24.0, 71.0, 99.0, 0.0, 0.0, 0.0];
const LOCKUP: [f32; 7] = [42.0, 24.0, 71.0, 93.0, 0.0, 0.0, 0.0];
/// `FLASH_BOOTLOADER`'s: the bootloader's magic number as param 5.
const FLASH_BOOTLOADER: [f32; 7] = [0.0, 0.0, 0.0, 0.0, 290_876.0, 0.0, 0.0];
/// The questions' words.
const ARE_YOU_SURE: &str = "Are you sure?";
const LOCKUP_CAPTION: &str = "Lockup";
const LOCKUP_TEXT: &str = "Lockup the autopilot??? this can cause a CRASH!!!!!!";
const QNH_TITLE: &str = "QNH";
const QNH_PROMPT: &str = "Enter the QNH in pascals (103040 = 1030.4 hPa)";
const HWID_TITLE: &str = "hwid";
const HWID_PROMPT: &str = "Enter the ID number";

/// The parameter QNH sets: `GND_ABS_PRESS` where the vehicle has it, else `BARO1_GND_PRESS`.
/// `// C#: temp.cs:670`
#[must_use]
fn qnh_param(parameters: &[(String, f64)]) -> &'static str {
    if parameters.iter().any(|(name, _)| name == "GND_ABS_PRESS") {
        "GND_ABS_PRESS"
    } else {
        "BARO1_GND_PRESS"
    }
}

/// A question showing, and what its Yes or OK goes on to.
enum Asking {
    /// `CustomMessageBox.Show(text, caption, YesNo)`.
    Confirm {
        caption: &'static str,
        text: &'static str,
        then: Act,
        /// The commands asked twice: the words a Yes asks next.
        again: Option<&'static str>,
    },
    /// `InputBox.Show(title, prompt, ref value)`, and what its OK goes on to.
    Input { input: InputBox, then: Answered },
    /// `CustomMessageBox.Show(text)`: what a tool found.
    Message { text: String },
    /// Param Restore's notice, whose OK asks for the file.
    Notice,
    /// An `OpenFileDialog`, the path typed (as every file dialog here), and whose it is.
    Path(crate::config::firmware::PathBox, Opened),
}

/// Whose file dialog is showing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Opened {
    /// Param Restore's parameter file.
    ParamRestore,
    /// mag calb log's log.
    MagCalLog,
    /// Split DFLog's log.
    SplitDfLog,
}

/// What an input box's answer is for.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Answered {
    /// QNH: the parameter it sets.
    Qnh(&'static str),
    /// decode HWIDs.
    Hwids,
    /// Split DFLog's count: the log it splits.
    SplitPieces(std::path::PathBuf),
}

/// The tab's state: the last button pressed, for the facts, the question showing, and where the
/// table is scrolled to.
pub(crate) struct Experimental {
    last: Option<&'static str>,
    /// Message Interval's form, while it shows.
    pub(crate) interval: Option<crate::message_interval::IntervalForm>,
    asking: Option<Asking>,
    /// The input box's keyboard, made the first time one shows.
    focus: Option<gpui::FocusHandle>,
    scroll: gpui::ScrollHandle,
    /// mag calb log's reading and fitting, on its thread, until its answer comes.
    magcal: Option<std::sync::mpsc::Receiver<Processed>>,
    /// What the last one came to, for the facts.
    magcal_last: Option<String>,
    /// arm and takeoff's step under way.
    takeoff: Option<Takeoff>,
    /// How its last step ended, for the facts.
    takeoff_last: Option<&'static str>,
    /// Split DFLog's splitting, on its thread, until its answer comes.
    split: Option<std::sync::mpsc::Receiver<Result<usize, String>>>,
    /// What the last one came to, for the facts.
    split_last: Option<String>,
}

impl Default for Experimental {
    fn default() -> Self {
        Self {
            last: None,
            interval: None,
            asking: None,
            focus: None,
            scroll: gpui::ScrollHandle::new(),
            magcal: None,
            magcal_last: None,
            split: None,
            split_last: None,
            takeoff: None,
            takeoff_last: None,
        }
    }
}

impl std::fmt::Debug for Experimental {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Experimental")
            .field("last", &self.last)
            .field("asking", &self.asking.is_some())
            .finish_non_exhaustive()
    }
}

/// The box ids of this tab's questions.
const IDS: BoxIds = BoxIds {
    question: "experimental-question",
    yes: "experimental-question-yes",
    no: "experimental-question-no",
    message: "experimental-message",
    ok: "experimental-message-ok",
    path: "experimental-path",
    path_value: "experimental-path-value",
    path_ok: "experimental-path-ok",
    path_cancel: "experimental-path-cancel",
};

/// A command pressed: its question asked, or sent.
fn act(
    this: &mut MissionPlanner,
    what: Act,
    window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    match what {
        Act::Reboot => {
            this.experimental.asking = Some(Asking::Confirm {
                caption: "",
                text: ARE_YOU_SURE,
                then: Act::Reboot,
                again: None,
            });
        }
        Act::Lockup => {
            this.experimental.asking = Some(Asking::Confirm {
                caption: LOCKUP_CAPTION,
                text: LOCKUP_TEXT,
                then: Act::Lockup,
                again: Some(LOCKUP_TEXT),
            });
        }
        Act::BootloaderUpgrade => {
            let [first, second] = BL_QUESTIONS;
            this.experimental.asking = Some(Asking::Confirm {
                caption: BL_UPDATE,
                text: first,
                then: Act::BootloaderUpgrade,
                again: Some(second),
            });
        }
        Act::ToggleSafety => {
            this.experimental.asking = Some(Asking::Confirm {
                caption: "",
                text: ARE_YOU_SURE,
                then: Act::ToggleSafety,
                again: None,
            });
        }
        Act::Qnh => {
            let view = this.telemetry.view();
            if view.vehicle.is_none() {
                // `GetParam` with no link throws: said on the status line.
                this.file_status = Some(error_box(PLEASE_CONNECT));
                return;
            }
            let param = qnh_param(&view.parameters);
            let current = view
                .parameters
                .iter()
                .find(|(name, _)| name == param)
                .map_or_else(|| "0".to_owned(), |(_, value)| value.to_string());
            this.experimental.asking = Some(Asking::Input {
                input: InputBox::new(QNH_TITLE, QNH_PROMPT, &current),
                then: Answered::Qnh(param),
            });
            focus_input(this, window, cx);
        }
        Act::DecodeHwids => {
            this.experimental.asking = Some(Asking::Input {
                input: InputBox::new(HWID_TITLE, HWID_PROMPT, "0"),
                then: Answered::Hwids,
            });
            focus_input(this, window, cx);
        }
        Act::ForceAccelCal | Act::ForceCompassCal | Act::DfuMode => send(this, what),
        Act::ParamRestore => {
            this.experimental.asking = Some(Asking::Notice);
            focus_input(this, window, cx);
        }
        // `InitialDirectory = Settings.Instance.LogDir`.
        Act::MagCalLog => {
            let folder = crate::fly::log_directory()
                .map(|folder| folder.to_string_lossy().into_owned())
                .unwrap_or_default();
            this.experimental.asking = Some(Asking::Path(
                crate::config::firmware::PathBox::new(&folder, LOG_FILE_MASK),
                Opened::MagCalLog,
            ));
            focus_input(this, window, cx);
        }
        // The cache `MyImageCache` keeps, under `CacheLocator.Location`; the provider the one map
        // both screens show, `FlightData.instance.gMapControl1.MapProvider`.
        Act::ClearCustomMaps | Act::AgeMapData => {
            let tile_root =
                mp_tiles::TileCache::new(mp_tiles::TileCache::default_root()).tile_root();
            let removed = removed_by(
                what,
                &tile_root,
                this.tile_source_id(),
                web_time::SystemTime::now(),
            );
            log::info!("{}", removed_images(removed));
            this.experimental.asking = Some(Asking::Message {
                text: removed_images(removed),
            });
        }
        Act::ArmAndTakeoff => arm_and_takeoff(this),
        // `InitialDirectory = Settings.Instance.LogDir`.
        Act::SplitDfLog => {
            let folder = crate::fly::log_directory()
                .map(|folder| folder.to_string_lossy().into_owned())
                .unwrap_or_default();
            this.experimental.asking = Some(Asking::Path(
                crate::config::firmware::PathBox::new(&folder, DFLOG_FILE_MASK),
                Opened::SplitDfLog,
            ));
            focus_input(this, window, cx);
        }
    }
}

/// The input box given the keyboard, its handle made the first time.
fn focus_input(this: &mut MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>) {
    let focus = this
        .experimental
        .focus
        .get_or_insert_with(|| cx.focus_handle())
        .clone();
    focus.focus(window, cx);
}

/// A command sent, once its questions are answered: what the C# shows in a box on failure said
/// on the status line.
fn send(this: &mut MissionPlanner, what: Act) {
    let sent = match what {
        Act::Reboot => this.telemetry.reboot(),
        Act::ForceAccelCal | Act::ForceCompassCal => {
            let params = if what == Act::ForceAccelCal {
                FORCE_ACCEL
            } else {
                FORCE_COMPASS
            };
            this.telemetry.send_handle().is_some_and(|(_, vehicle)| {
                this.telemetry
                    .command(
                        vehicle,
                        PREFLIGHT_CALIBRATION,
                        params,
                        Report::on_timeout(error_box(DO_COMMAND_TIMEOUT)),
                    )
                    .is_some()
            })
        }
        Act::DfuMode => this
            .telemetry
            .command_unacknowledged(PREFLIGHT_REBOOT_SHUTDOWN, DFU_BOOT),
        Act::Lockup => this
            .telemetry
            .command_unacknowledged(PREFLIGHT_REBOOT_SHUTDOWN, LOCKUP),
        // `doCommand` waited for, its answer said where the C# shows a box; unanswered, the
        // `catch`'s exception.
        Act::BootloaderUpgrade => this.telemetry.send_handle().is_some_and(|(_, vehicle)| {
            let report = Report {
                accepted: Some(UPGRADED_BOOTLOADER.to_owned()),
                refused: Some(FAILED_TO_UPGRADE_BOOTLOADER.to_owned()),
                timed_out: Some(error_box(DO_COMMAND_TIMEOUT)),
                fallback: None,
            };
            this.telemetry
                .command(vehicle, CMD_FLASH_BOOTLOADER, FLASH_BOOTLOADER, report)
                .is_some()
        }),
        // No vehicle is `sysidcurrent` 0, which the C# only logs.
        Act::ToggleSafety => {
            if this.telemetry.send_handle().is_some() {
                this.fly_press(&Report::default(), |_, target, view| {
                    crate::fly::action_messages(
                        "Toggle_Safety_Switch",
                        &crate::fly::action_context(target, view),
                    )
                });
            } else {
                log::info!("Not toggling safety on sysid 0");
            }
            true
        }
        Act::Qnh
        | Act::DecodeHwids
        | Act::ParamRestore
        | Act::MagCalLog
        | Act::SplitDfLog
        | Act::ClearCustomMaps
        | Act::AgeMapData
        | Act::ArmAndTakeoff => true,
    };
    if !sent {
        this.file_status = Some(error_box(PLEASE_CONNECT));
    }
}

/// The question's Yes or No, OK or Cancel.
fn answer(this: &mut MissionPlanner, yes: bool) {
    let Some(asking) = this.experimental.asking.take() else {
        return;
    };
    if !yes {
        // Split DFLog never reads the box's answer: Cancel splits into the count it offered.
        if let Asking::Input {
            then: Answered::SplitPieces(file),
            ..
        } = asking
        {
            split_df_log(this, file, &SPLIT_OFFERED.to_string());
        }
        return;
    }
    match asking {
        // Lockup's and Bootloader Upgrade's second asking.
        Asking::Confirm {
            caption,
            then,
            again: Some(text),
            ..
        } => {
            this.experimental.asking = Some(Asking::Confirm {
                caption,
                text,
                then,
                again: None,
            });
        }
        Asking::Confirm { then, .. } => send(this, then),
        Asking::Message { .. } | Asking::Notice | Asking::Path(..) => {}
        Asking::Input {
            input,
            then: Answered::SplitPieces(file),
        } => split_df_log(this, file, input.field.value()),
        Asking::Input {
            input,
            then: Answered::Hwids,
        } => {
            this.experimental.asking = Some(Asking::Message {
                text: decode_hwids(input.field.value()),
            });
        }
        Asking::Input {
            input,
            then: Answered::Qnh(param),
        } => match input.field.value().trim().parse::<f64>() {
            Ok(value) => {
                let target = this.telemetry.send_handle().map(|(_, vehicle)| vehicle);
                let sent = target.and_then(|vehicle| {
                    this.telemetry.set_parameter_on(
                        vehicle,
                        param,
                        value,
                        false,
                        Report::on_failure(error_box(format!("Timeout on read - setParam {param}"))),
                    )
                });
                if sent.is_none() {
                    this.file_status = Some(error_box(PLEASE_CONNECT));
                }
            }
            // `double.Parse` throws on what is not a number.
            Err(_) => {
                this.file_status = Some(error_box(format!(
                    "Input string was not in a correct format. ({})",
                    input.field.value()
                )));
            }
        },
    }
}

/// Param Restore's notice answered: the file asked for, in a box with the parameter files' filter.
/// `// C#: temp.cs:1269-1281`
fn notice_ok(this: &mut MissionPlanner) {
    this.experimental.asking = Some(Asking::Path(
        crate::config::firmware::PathBox::new("", PARAM_FILE_MASK),
        Opened::ParamRestore,
    ));
}

/// A file dialog answered: Cancel does nothing; a name that is no file keeps the box, as
/// `OpenFileDialog` keeps asking; a file goes to the tool that asked.
fn path_answered(this: &mut MissionPlanner, ok: bool) {
    let Some(Asking::Path(path, opened)) = this.experimental.asking.take() else {
        return;
    };
    if !ok {
        return;
    }
    let Some(file) = path.chosen() else {
        this.experimental.asking = Some(Asking::Path(path, opened));
        return;
    };
    match opened {
        Opened::ParamRestore => restore_file(this, &file),
        Opened::MagCalLog => read_mag_log(this, file),
        // `InputBox.Show("How Many", ..., ref a)` with `a = 10`.
        Opened::SplitDfLog => {
            this.experimental.asking = Some(Asking::Input {
                input: InputBox::new(SPLIT_TITLE, SPLIT_PROMPT, &SPLIT_OFFERED.to_string()),
                then: Answered::SplitPieces(file),
            });
        }
    }
}

/// Split DFLog's count answered: `int.Parse(answer)` - a count that is no number throws, said on
/// the status line as QNH's is - then `new DFLogBuffer(file).SplitLog(a)` on a thread of its own,
/// where the C#'s window waits on it; [`tick`] takes its answer. One at a time.
/// `// C#: temp.cs:730-732; ExtLibs/Controls/InputBox.cs:21-27`
fn split_df_log(this: &mut MissionPlanner, file: std::path::PathBuf, answer: &str) {
    let Ok(pieces) = answer.trim().parse::<i32>() else {
        this.file_status = Some(error_box(format!(
            "Input string was not in a correct format. ({answer})"
        )));
        return;
    };
    if this.experimental.split.is_some() {
        return;
    }
    let (sender, receiver) = std::sync::mpsc::channel();
    let spawned = wasm_thread::Builder::new()
        .name("mp-split-dflog".to_owned())
        .spawn(move || {
            let _ = sender.send(mp_log::dflogbuffer::split_file(&file, pieces));
        });
    match spawned {
        Ok(_) => this.experimental.split = Some(receiver),
        Err(error) => log::debug!("Split DFLog: {error}"),
    }
}

/// mag calb log's log chosen: `ProcessLog`'s reading, fitting and drawing on a thread of its own,
/// where the C#'s window waits on them; [`tick`] takes its answer. One at a time: a second log
/// chosen while one is read waits for nothing and is not read, as the C#'s waiting window allows
/// no second.
/// `// C#: MagCalib.cs:109-131`
fn read_mag_log(this: &mut MissionPlanner, file: std::path::PathBuf) {
    if this.experimental.magcal.is_some() {
        return;
    }
    let (sender, receiver) = std::sync::mpsc::channel();
    let spawned = wasm_thread::Builder::new()
        .name("mp-magcal-log".to_owned())
        .spawn(move || {
            let data = mp_settings::user_data_directory();
            let _ = sender.send(process_log(&file, 0, data.as_deref()));
        });
    match spawned {
        Ok(_) => this.experimental.magcal = Some(receiver),
        Err(error) => log::debug!("mag calb log: {error}"),
    }
}

/// Once a frame: mag calb log's answer, when it comes - the offsets to `SaveOffsets`, the box the
/// C# shows on the status line (the owner's ruling: no box for it), or nothing, as its `catch`
/// shows nothing. `// C#: MagCalib.cs:115-130`
pub(crate) fn tick(this: &mut MissionPlanner) {
    split_tick(this);
    takeoff_tick(this);
    let Some(receiver) = this.experimental.magcal.as_ref() else {
        return;
    };
    let processed = match receiver.try_recv() {
        Ok(processed) => processed,
        Err(std::sync::mpsc::TryRecvError::Empty) => {
            crate::repaint::in_flight();
            return;
        }
        Err(std::sync::mpsc::TryRecvError::Disconnected) => {
            Processed::Quiet("the reading stopped without an answer".to_owned())
        }
    };
    this.experimental.magcal = None;
    this.experimental.magcal_last = Some(match &processed {
        Processed::Offsets {
            offsets: [x, y, z],
            drawing,
        } => format!("offsets {x} {y} {z}; {DXF_NAME} {drawing} bytes"),
        Processed::Said(text) => format!("said: {text}"),
        Processed::Quiet(why) => format!("quiet: {why}"),
    });
    match processed {
        Processed::Offsets { offsets, .. } => {
            let view = this.telemetry.view();
            this.compass
                .save_offsets(&offsets, &view.parameters, this.telemetry.is_open());
        }
        Processed::Said(text) => this.file_status = Some(text.to_owned()),
        Processed::Quiet(why) => log::debug!("mag calb log: {why}"),
    }
}

/// arm and takeoff pressed: Stabilize asked for, then the arming sent - with no vehicle, what the
/// C#'s `catch` meets said on the status line, as this tab's other commands say it.
/// `// C#: temp.cs:618-620`
fn arm_and_takeoff(this: &mut MissionPlanner) {
    let Some((_, target)) = this.telemetry.send_handle() else {
        this.file_status = Some(error_box(PLEASE_CONNECT));
        return;
    };
    let family = crate::fly::family(&this.telemetry.view());
    for message in crate::fly::set_mode_messages(target, family, "Stabilize") {
        this.telemetry.send(&message);
    }
    let arm = mp_link::commands::arm(target, true, false);
    if let Some(id) = this
        .telemetry
        .command_message(&arm, Report::on_timeout(error_box(DO_COMMAND_TIMEOUT)))
    {
        this.experimental.takeoff = Some(Takeoff::Arming {
            id,
            made: web_time::Instant::now(),
            target,
        });
        this.experimental.takeoff_last = None;
    }
}

/// How a request the tab waits on ended: `None` while it has not; a request the link has let go
/// as timed out, which its report has said.
fn ended(
    telemetry: &crate::telemetry::Telemetry,
    id: mp_link::RequestId,
    made: web_time::Instant,
) -> Option<mp_link::requests::RequestOutcome> {
    match telemetry.lookup(id, made) {
        crate::telemetry::Lookup::Found(request) => request.outcome(),
        crate::telemetry::Lookup::PickingUp => None,
        crate::telemetry::Lookup::Gone => Some(mp_link::requests::RequestOutcome::TimedOut),
    }
}

/// arm and takeoff's next step, once a frame: armed, Guided and the pause; the pause over, the
/// take-off; refused or unanswered, the end. `// C#: temp.cs:620-628`
fn takeoff_tick(this: &mut MissionPlanner) {
    use mp_link::requests::RequestOutcome;
    let Some(step) = this.experimental.takeoff else {
        return;
    };
    crate::repaint::in_flight();
    match step {
        Takeoff::Arming { id, made, target } => {
            let Some(outcome) = ended(&this.telemetry, id, made) else {
                return;
            };
            this.experimental.takeoff = None;
            if !matches!(outcome, RequestOutcome::Accepted { .. }) {
                // `doARM` false, or its throw, which the report has said.
                this.experimental.takeoff_last = Some(if outcome == RequestOutcome::TimedOut {
                    "arm: timed out"
                } else {
                    "arm: refused"
                });
                return;
            }
            let family = crate::fly::family(&this.telemetry.view());
            for message in crate::fly::set_mode_messages(target, family, "GUIDED") {
                this.telemetry.send(&message);
            }
            this.experimental.takeoff = Some(Takeoff::Sleeping {
                until: web_time::Instant::now() + TAKEOFF_PAUSE,
                target,
            });
        }
        Takeoff::Sleeping { until, target } => {
            if web_time::Instant::now() < until {
                return;
            }
            let takeoff = mp_link::commands::takeoff(target, TAKEOFF_ALTITUDE);
            this.experimental.takeoff = this
                .telemetry
                .command_message(&takeoff, Report::on_timeout(error_box(DO_COMMAND_TIMEOUT)))
                .map(|id| Takeoff::TakingOff {
                    id,
                    made: web_time::Instant::now(),
                });
        }
        Takeoff::TakingOff { id, made } => {
            let Some(outcome) = ended(&this.telemetry, id, made) else {
                return;
            };
            this.experimental.takeoff = None;
            this.experimental.takeoff_last = Some(match outcome {
                RequestOutcome::Accepted { .. } => "takeoff: accepted",
                RequestOutcome::TimedOut => "takeoff: timed out",
                _ => "takeoff: refused",
            });
        }
    }
}

/// Split DFLog's answer, when it comes: nothing said when it is done, as the C# says nothing; what
/// `SplitLog` threw on the status line, where the C#'s error box shows it.
fn split_tick(this: &mut MissionPlanner) {
    let Some(receiver) = this.experimental.split.as_ref() else {
        return;
    };
    let split = match receiver.try_recv() {
        Ok(split) => split,
        Err(std::sync::mpsc::TryRecvError::Empty) => {
            crate::repaint::in_flight();
            return;
        }
        Err(std::sync::mpsc::TryRecvError::Disconnected) => {
            Err("the split stopped without an answer".to_owned())
        }
    };
    this.experimental.split = None;
    match split {
        Ok(pieces) => this.experimental.split_last = Some(format!("wrote {pieces}")),
        Err(why) => {
            this.experimental.split_last = Some(format!("failed: {why}"));
            this.file_status = Some(error_box(why));
        }
    }
}

/// Param Restore's file: `ParamFile.loadParamFile` and its parameters restored (params.rs's
/// `restore`, on the status line as it goes). The file's parameters go in their name order, which
/// the planner's file reader keeps, where the C#'s dictionary keeps the file's - the same for every
/// file Mission Planner writes, which it writes sorted.
/// `// C#: temp.cs:1279-1290`
fn restore_file(this: &mut MissionPlanner, file: &std::path::Path) {
    match mp_params::param_file::ParamFile::load(file) {
        Ok(params) => this.start_param_writes(crate::params::ParamWrites::restore(
            params.iter().map(|(name, value)| (name.to_owned(), value)),
        )),
        Err(err) => this.file_status = Some(error_box(format!("{}: {err}", file.display()))),
    }
}

/// The question showing, over the tab.
fn asking_box(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    match this.experimental.asking.as_ref()? {
        Asking::Confirm { caption, text, .. } => Some(question_box(
            IDS,
            caption,
            text,
            Buttons::YesNo,
            window,
            answer,
            cx,
        )),
        Asking::Message { text } => Some(message_box(
            IDS,
            &Waiting {
                text: text.clone(),
                caption: String::new(),
                buttons: None,
            },
            window,
            |this| this.experimental.asking = None,
            cx,
        )),
        Asking::Notice => Some(message_box(
            IDS,
            &Waiting {
                text: RESTORE_NOTICE.to_owned(),
                caption: String::new(),
                buttons: None,
            },
            window,
            notice_ok,
            cx,
        )),
        Asking::Path(path, _) => {
            let focus = this.experimental.focus.as_ref()?;
            Some(crate::config::firmware::path_box(
                IDS,
                path,
                focus,
                window,
                |this, event| {
                    let outcome = match this.experimental.asking.as_mut() {
                        Some(Asking::Path(path, _)) => path.field.key(event),
                        _ => return false,
                    };
                    match outcome {
                        crate::textfield::KeyOutcome::Submitted => path_answered(this, true),
                        crate::textfield::KeyOutcome::Cancelled => path_answered(this, false),
                        crate::textfield::KeyOutcome::Changed => {}
                        crate::textfield::KeyOutcome::Ignored => return false,
                    }
                    true
                },
                path_answered,
                cx,
            ))
        }
        Asking::Input { input, .. } => {
            let focus = this.experimental.focus.as_ref()?;
            Some(input_box(
                "experimental-input-box",
                input,
                focus,
                window,
                |this, event| {
                    let outcome = match this.experimental.asking.as_mut() {
                        Some(Asking::Input { input, .. }) => Some(input.field.key(event)),
                        _ => None,
                    };
                    match outcome {
                        Some(crate::textfield::KeyOutcome::Submitted) => {
                            answer(this, true);
                            true
                        }
                        Some(crate::textfield::KeyOutcome::Cancelled) => {
                            answer(this, false);
                            true
                        }
                        Some(crate::textfield::KeyOutcome::Changed) => true,
                        _ => false,
                    }
                },
                |this| answer(this, true),
                |this| answer(this, false),
                cx,
            ))
        }
    }
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
        Tool::MessageInterval => crate::message_interval::open(this, cx),
        Tool::Act(what) => act(this, what, window, cx),
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
    #[allow(clippy::cast_precision_loss)] // 32 rows
    let least = f32::from(ROWS) * MIN_ROW_HEIGHT + f32::from(ROWS - 1) * ROW_GAP;
    let mut table = crate::probe::measured("experimental-table", div())
        .flex()
        .flex_col()
        .w_full()
        .flex_1()
        .min_h(px(least))
        .gap(px(ROW_GAP));
    for row in 0..ROWS {
        let mut line = div()
            .flex()
            .w_full()
            .flex_1()
            .min_h(px(MIN_ROW_HEIGHT));
        for (column, width) in (0u8..).zip(COLUMNS) {
            let found = CELLS
                .iter()
                .find(|&&(r, c, ..)| r == row && c == column);
            line = line.child(div().w(relative(width / 100.0)).h_full().child(cell(found, cx)));
        }
        table = table.child(line);
    }
    div()
        .relative()
        .flex()
        .flex_col()
        .flex_1()
        .min_h(px(0.0))
        .child(
            div()
                .id("experimental-body")
                .flex()
                .flex_col()
                .size_full()
                .overflow_y_scroll()
                .track_scroll(&this.experimental.scroll)
                .p_3()
                .child(table),
        )
        .children(crate::ui::scroll_indicator(&this.experimental.scroll))
        // The windows the buttons open, over the tab as over the pages that open them elsewhere.
        .children(this.extra_setup_overlay(window, cx))
        .children(crate::georef_ui::window(this, window, cx))
        .children(crate::message_interval::window(this, window, cx))
        .children(asking_box(this, window, cx))
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
    crate::message_interval::record_facts(state.interval.as_ref());
    facts::record(
        "experimental.asking",
        match state.asking.as_ref() {
            None => "none".to_owned(),
            Some(Asking::Confirm { text, .. }) => (*text).to_owned(),
            Some(Asking::Input { input, .. }) => format!("{}: {}", input.title, input.prompt),
            Some(Asking::Message { text }) => format!("message: {}", text.trim_end()),
            Some(Asking::Notice) => RESTORE_NOTICE.to_owned(),
            Some(Asking::Path(path, _)) => format!("{}: {}", path.caption, path.field.value()),
        },
    );
    // arm and takeoff: the step under way, or how the last one ended.
    facts::record(
        "experimental.takeoff",
        match state.takeoff {
            Some(Takeoff::Arming { .. }) => "arming",
            Some(Takeoff::Sleeping { .. }) => "sleeping",
            Some(Takeoff::TakingOff { .. }) => "taking off",
            None => state.takeoff_last.unwrap_or("none"),
        },
    );
    // Split DFLog: splitting, or what the last split came to.
    facts::record(
        "experimental.split",
        if state.split.is_some() {
            "splitting"
        } else {
            state.split_last.as_deref().unwrap_or("none")
        },
    );
    // mag calb log: reading, or what the last reading came to.
    facts::record(
        "experimental.magcal",
        if state.magcal.is_some() {
            "reading"
        } else {
            state.magcal_last.as_deref().unwrap_or("none")
        },
    );
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
                Tool::Georef | Tool::MessageInterval | Tool::Act(_) => opens += 1,
                Tool::Unavailable(why) => assert!(!why.is_empty()),
            }
        }
        assert_eq!(opens, 25);
        assert_eq!(tool("but_paramrestore"), Tool::Act(Act::ParamRestore));
        assert_eq!(tool("BUT_magfit2"), Tool::Act(Act::MagCalLog));
        assert_eq!(tool("myButton1"), Tool::Act(Act::SplitDfLog));
        assert_eq!(tool("BUT_clearcustommaps"), Tool::Act(Act::ClearCustomMaps));
        assert_eq!(tool("but_agemapdata"), Tool::Act(Act::AgeMapData));
        assert_eq!(tool("but_armandtakeoff"), Tool::Act(Act::ArmAndTakeoff));
        assert_eq!(tool("but_blupdate"), Tool::Act(Act::BootloaderUpgrade));
        assert_eq!(tool("but_disablearmswitch"), Tool::Act(Act::ToggleSafety));
        assert_eq!(tool("but_messageinterval"), Tool::MessageInterval);
        assert_eq!(tool("BUT_swarm"), Tool::Unavailable(SECTION_12_D13));
        assert_eq!(tool("but_GDAL"), Tool::Unavailable(NO_GDAL));
        assert_eq!(tool("but_anonlog"), Tool::Unavailable(ANON_LOG_RULED));
        assert_eq!(tool("but_reboot"), Tool::Act(Act::Reboot));
        assert_eq!(tool("but_structtest"), Tool::Unavailable(NOT_PORTED));
    }

    /// The commands' parameters are the C#'s: 76 as param 5 for the accelerometers and param 2
    /// for the compasses; DFU's and Lockup's PREFLIGHT_REBOOT_SHUTDOWN magic.
    #[test]
    fn the_commands_are_the_csharps() {
        assert_eq!(FORCE_ACCEL[4], 76.0);
        assert_eq!(FORCE_ACCEL.iter().filter(|p| **p != 0.0).count(), 1);
        assert_eq!(FORCE_COMPASS[1], 76.0);
        assert_eq!(FORCE_COMPASS.iter().filter(|p| **p != 0.0).count(), 1);
        assert_eq!(DFU_BOOT[..4], [42.0, 24.0, 71.0, 99.0]);
        assert_eq!(LOCKUP[..4], [42.0, 24.0, 71.0, 93.0]);
        assert_eq!(FLASH_BOOTLOADER[4], 290_876.0);
        assert_eq!(FLASH_BOOTLOADER.iter().filter(|p| **p != 0.0).count(), 1);
        assert_eq!(CMD_FLASH_BOOTLOADER, 42_650);
    }

    /// decode HWIDs: each whole number on a line, the line and its device; words skipped; a
    /// line with two ids reported twice, as the C#'s loop reports the whole line for each.
    #[test]
    fn decode_hwids_takes_each_id_apart() {
        let id = 97_539;
        let expected = format!("{id} = {}\n", crate::config::hw_ids::device_structure("", id));
        assert_eq!(decode_hwids("97539"), expected);
        assert_eq!(decode_hwids("compass\n97539"), expected);
        assert_eq!(decode_hwids("1\t2").lines().count(), 2);
        assert!(decode_hwids("1\t2").starts_with("1 2 = "));
        assert_eq!(decode_hwids(""), "");
    }

    /// QNH sets GND_ABS_PRESS where the vehicle has it, else BARO1_GND_PRESS.
    #[test]
    fn qnh_sets_the_parameter_the_vehicle_has() {
        assert_eq!(qnh_param(&[]), "BARO1_GND_PRESS");
        assert_eq!(
            qnh_param(&[("GND_ABS_PRESS".to_owned(), 101_325.0)]),
            "GND_ABS_PRESS"
        );
        assert_eq!(
            qnh_param(&[("BARO1_GND_PRESS".to_owned(), 101_325.0)]),
            "BARO1_GND_PRESS"
        );
    }

    /// Clear Custom Maps takes every Custom tile, `.jpg` and `.png`, and no other provider's; Age
    /// Map Data takes the map's provider's tiles made before thirty days ago - none of them now,
    /// all of them thirty-one days on - and none with no provider.
    #[test]
    fn the_map_tools_delete_what_the_csharp_does() {
        let root = mp_os::temp_dir().join(format!("mp-experimental-maps-{}", std::process::id()));
        let _ = mp_os::fs::remove_dir_all(&root);
        let osm = mp_tiles::source::source_by_id("osm").expect("osm");
        for (provider, file) in [
            ("Custom", "1/2/3.jpg"),
            ("Custom", "4/5/6.png"),
            (osm.cache_name, "1/2/3.png"),
        ] {
            let path = root.join(provider).join(file);
            mp_os::fs::create_dir_all(path.parent().expect("a folder")).expect("folder");
            mp_os::fs::write(&path, b"tile").expect("tile");
        }
        let now = web_time::SystemTime::now() + web_time::Duration::from_secs(1);
        assert_eq!(removed_by(Act::AgeMapData, &root, Some("osm"), now), 0);
        assert_eq!(removed_by(Act::AgeMapData, &root, None, now), 0);
        assert_eq!(removed_by(Act::ClearCustomMaps, &root, Some("osm"), now), 2);
        assert_eq!(removed_by(Act::ClearCustomMaps, &root, Some("osm"), now), 0);
        let later = now + web_time::Duration::from_secs(31 * 24 * 60 * 60);
        assert_eq!(removed_by(Act::AgeMapData, &root, Some("osm"), later), 1);
        assert_eq!(removed_images(2), "Removed 2 images");
        let _ = mp_os::fs::remove_dir_all(&root);
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
