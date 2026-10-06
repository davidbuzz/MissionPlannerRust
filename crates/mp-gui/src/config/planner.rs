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

//! The Planner page of the CONFIG screen: `GCSViews/ConfigurationView/ConfigPlanner.cs`.
//!
//! Mission Planner's own settings: every control sits at its `.resx` `Location` and `Size` on a
//! 949 x 693 page (`ConfigPlanner.resx` `$this.Size`), bound to the `Settings.Instance` key its
//! handler writes. `Settings.Instance` is Mission Planner's `config.xml` (`settings::Persisted`):
//! each handler puts its key in that dictionary as its control changes, under the C#'s own name,
//! and the file is written at the next `SaveConfig` - on the FLIGHT DATA and FLIGHT PLAN buttons,
//! after Connect and on closing - with every other key in it. The page saves nothing itself; the
//! C# has no save button either (`MainV2.cs:1109, 1311-1325, 1853, 2178`).
//!
//! What acts at once, as it does in the C#:
//!
//! * the three unit combos run `MainV2.ChangeUnits` (`mp_vehicle::units`), and the page holds the
//!   multipliers and unit names that sets (`CurrentState`'s statics);
//! * the five telemetry rate combos set `cs.rateX` and its backup and put `REQUEST_DATA_STREAM`
//!   on the link for their streams, twice each as `getDatastream` sends it;
//! * the speech boxes show and hide with Enable Speech, and each one ticked asks, in the
//!   `InputBox`es the C# asks in, for its text templates and levels, each OK keeping its answer
//!   as `InputBox` keeps every titled answer, under `"InputBox"` and the caption and question
//!   with all but letters and digits taken out (`InputBox.cs:73-84, 178-184`);
//! * Map is rotated and No Fly untick each other; Load Waypoints on connect sets whether the
//!   mission is read when a vehicle connects; the map access mode rebuilds the map's tile store
//!   (`CacheOnly` is the store's offline mode); Joystick Setup opens the joystick page over this
//!   one; Browse asks for a folder; Open Map Cache opens the tile cache's directory;
//! * Video Device lists the capture devices when it is clicked and Video Format the chosen
//!   device's formats; Start opens the capture (`mp_video`, V4L2 on Linux where the C# has
//!   DirectShow and does nothing under mono) and the flight screen's HUD draws its frames under
//!   everything, and Stop ends it. A camera that will not start says "Camera Fail: " and why on
//!   the status line, where the C# has a message box (the owner's ruling of 2026-09-25). Enable
//!   HUD Overlay sets the HUD's `hudon`, saved as `CHK_hudshow`: unticked, the HUD draws the
//!   camera's picture alone while there is one, and everything as before while there is none
//!   (`ConfigPlanner.cs:375-379`, `HUD.cs:1988-2009`).
//!
//! Dimmed, each naming what it stands for here, are the controls whose handler drives something
//! this application does not have: GDI+ (gpui draws the HUD); the UI language (English only); the
//! theme and Custom (the dark palette is ratified); OSD Color (its handler's body is commented out, `ConfigPlanner.cs:433-440`);
//! Start/Stop Vario; Password Protect Config; ADSB (no ADSB server client); OptOut Anon Stats (no
//! analytics); Beta Updates (no updater); Mavlink Message Debug; Testing Screen.
//! `CHK_AutoParamCommit` is not drawn: `Activate` hides it outside a display view with the
//! parameter commit button, and neither preset has one (`DisplayView.cs:214`); a Custom view that
//! turns it on shows it in the C#, for a commit button this application's parameter list does
//! not have.
//!
//! Layout picks the display view (`display_view.rs`): Basic, Advanced or Custom, each saved as
//! `displayview` (`ConfigPlanner.cs:1019-1034`). `Activate` selects the view's name, and, the
//! handler checking no `startup`, a selection that changes runs it: the preset of that name is
//! made the view again (`:55-73`). The page object here lives as long as the application, where
//! the C#'s is made with its screen, so that happens at its first `Activate`, and after only when
//! the view's name has changed; a view the preset makes again is the same view, except a Custom
//! file changed on disk in between. `label5` and the box are hidden for a view without
//! `displayPlannerLayout` (`:75-79`).
//!
//! What is not ported, and why:
//!
//! * `requestDatastream`'s `hzratecheck`, which skips a request when the vehicle already sends at
//!   about that rate: this application does not count packets per message, so every rate but -1
//!   is sent (`MAVLinkInterface.cs:3058-3235`);
//! * `GetDefaultLogDir` creating the log directory when the Log Path box is filled
//!   (`Settings.cs:146-158`): a settings page does not make directories here.
//!
//! The keys that other screens read - the track length, the aircraft icon's lines, the map's
//! rotation, airports, TFRs and no-fly zones, distance to home, the GCS id, the connect resets,
//! the speech templates - are written and kept; the screens that read them in the C# do not read
//! them here yet. Nor does the flight screen show its values in the units this page sets
//! (`fly.rs` holds `multiplieralt` at 1), nor does the link take the GCS id or these rates when
//! it connects: it asks for every stream at `LinkConfig::stream_rate_hz` (`MainV2.cs:983-1004`).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
use std::collections::{BTreeMap, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px,
    rgb,
};
use mp_mavlink_dialects::all::{MavDataStream, MavMessage, RequestDataStream};
use mp_vehicle::units::DisplayUnits;

use super::servo_output::Combo;
use crate::MissionPlanner;
use crate::display_view::{DisplayName, DisplayView};
use crate::settings::Persisted;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.resx ($this.Size)`
const PAGE: (f32, f32) = (949.0, 693.0);

/// `bool.ToString()`.
const fn bool_text(value: bool) -> &'static str {
    if value { "True" } else { "False" }
}

/// `Settings.GetBoolean`: `bool.TryParse` of the value - which trims white space and nulls - else
/// the default.
/// `// C#: ExtLibs/Utilities/Settings.cs:223-232`
fn get_bool(settings: &Persisted, key: &str, default: bool) -> bool {
    let value = settings
        .get(key)
        .map(|value| value.trim_matches(|c: char| c.is_whitespace() || c == '\0'));
    match value {
        Some(value) if value.eq_ignore_ascii_case("true") => true,
        Some(value) if value.eq_ignore_ascii_case("false") => false,
        _ => default,
    }
}

/// `Settings.GetInt32`: `int.TryParse` of the value, else the default.
/// `// C#: ExtLibs/Utilities/Settings.cs:201-210`
fn get_int(settings: &Persisted, key: &str, default: i32) -> i32 {
    settings
        .get(key)
        .and_then(|value| value.trim().parse().ok())
        .unwrap_or(default)
}

/// Every `Settings.Instance` key `ConfigPlanner` reads or writes, by the C#'s name, each published
/// as `config.planner.<key>`. Keys the C# spells two ways (`GMapMarkerBase_Length` read,
/// `GMapMarkerBase_length` written) are both here, as the C# has both.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:47-1167`
const KEYS: &[&str] = &[
    // C#: ConfigPlanner.cs:96-103, 414
    "severity",
    // C#: ConfigPlanner.cs:149-167
    "speechenable",
    "speechwaypointenabled",
    "speechmodeenabled",
    "speechcustomenabled",
    "speechbatteryenabled",
    "speechaltenabled",
    "speecharmenabled",
    "speechlowspeedenabled",
    "beta_updates",
    "password_protect",
    "showairports",
    "enableadsb",
    "norcreceiver",
    "showtfr",
    "autoParamCommit",
    "ShowNoFly",
    "Params_BG",
    "SlowMachine",
    "speech_armed_only",
    // C#: ConfigPlanner.cs:170, 691
    "NUM_tracklength",
    // C#: ConfigPlanner.cs:173-177
    "loadwpsonconnect",
    "CHK_resetapmonconnect",
    "CHK_rtsresetesp32",
    // C#: ConfigPlanner.cs:574-641, by each combo's `Name`
    "CMB_rateattitude",
    "CMB_rateposition",
    "CMB_ratestatus",
    "CMB_raterc",
    "CMB_ratesensors",
    // C#: ConfigPlanner.cs:185-190
    "analyticsoptout",
    "CHK_GDIPlus",
    "CHK_maprotation",
    "CHK_disttohomeflightdata",
    // C#: ConfigPlanner.cs:195
    "hudcolor",
    // C#: ConfigPlanner.cs:209-214
    "distunits",
    "speedunits",
    "altunits",
    // `Settings.LogDir`. C#: ConfigPlanner.cs:236, 793; ExtLibs/Utilities/Settings.cs:127-140
    "logdirectory",
    // C#: ConfigPlanner.cs:239-245, 1080-1160
    "GMapMarkerBase_DisplayCOG",
    "GMapMarkerBase_DisplayHeading",
    "GMapMarkerBase_DisplayNavBearing",
    "GMapMarkerBase_DisplayRadius",
    "GMapMarkerBase_DisplayTarget",
    "mapicondesc",
    "mapicondesc_default",
    "GMapMarkerBase_Length",
    "GMapMarkerBase_length",
    "GMapMarkerBase_InactiveDisplayStyle",
    // C#: ConfigPlanner.cs:250, 1166
    "mapCache",
    // C#: ConfigPlanner.cs:1033; MainV2.cs:363
    "displayview",
    // The speech templates and levels. C#: ConfigPlanner.cs:443-550, 661-687, 812-917
    "speechwaypoint",
    "speechmode",
    "speechcustom",
    "speechbattery",
    "speechbatteryvolt",
    "speechbatterypercent",
    "speechalt",
    "speechaltheight",
    "speecharm",
    "speechdisarm",
    "speechlowgroundspeed",
    "speechlowgroundspeedtrigger",
    "speechlowairspeed",
    "speechlowairspeedtrigger",
    // C#: ConfigPlanner.cs:1061; MainV2.cs:683
    "gcsid",
    // `ThemeManager.thmColor.strThemeName`, which CMB_theme shows. C#: Utilities/ThemeManager.cs:287
    "theme",
    // C#: ConfigPlanner.cs:216-233, 277-279
    "video_device",
    "video_options",
    // `hudon`. C#: ConfigPlanner.cs:375-379; MainV2.cs:940-941
    "CHK_hudshow",
];

// -------------------------------------------------------------------------------------------------
// The Designer's controls.
// -------------------------------------------------------------------------------------------------

/// A `Label`: its `Location`, `Size` and `Text`.
type Label = (f32, f32, f32, f32, &'static str);

/// Every label, as `ConfigPlanner.resx` places it. `label5`, "Layout", is hidden with the Layout
/// box for a view without `displayPlannerLayout` (`ConfigPlanner.cs:75-79`).
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.resx (labelN.Location, .Size, .Text)`
const LABELS: &[Label] = &[
    (9.0, 11.0, 71.0, 13.0, "Video Device"),
    (9.0, 38.0, 69.0, 13.0, "Video Format"),
    (9.0, 65.0, 57.0, 13.0, "OSD Color"),
    (9.0, 90.0, 44.0, 13.0, "Speech"),
    (
        252.0,
        108.0,
        157.0,
        31.0,
        "NOTE: Set the low level of SEVERITY to speak",
    ),
    (9.0, 142.0, 69.0, 13.0, "UI Language"),
    (9.0, 171.0, 45.0, 13.0, "Joystick"),
    (9.0, 198.0, 52.0, 13.0, "Dist Units"),
    (249.0, 198.0, 46.0, 13.0, "Alt Units"),
    (
        480.0,
        198.0,
        241.0,
        31.0,
        "NOTE: The Configuration Tab will NOT display these units, as those are raw values.",
    ),
    (9.0, 225.0, 65.0, 13.0, "Speed Units"),
    (9.0, 254.0, 84.0, 13.0, "Telemetry Rates"),
    (104.0, 254.0, 56.0, 13.0, "Attitude"),
    (220.0, 254.0, 56.0, 13.0, "Position"),
    (330.0, 254.0, 79.0, 13.0, "Mode/Status"),
    (461.0, 254.0, 22.0, 13.0, "RC"),
    (535.0, 254.0, 43.0, 13.0, "Sensor"),
    (9.0, 278.0, 78.0, 13.0, "Connect Reset"),
    (9.0, 301.0, 71.0, 13.0, "Track Length"),
    (180.0, 303.0, 81.0, 17.0, "Dist to Home"),
    (9.0, 327.0, 57.0, 13.0, "Waypoints"),
    (9.0, 350.0, 31.0, 13.0, "HUD"),
    (9.0, 373.0, 61.0, 13.0, "Map Follow"),
    (9.0, 398.0, 50.0, 13.0, "Log Path"),
    (9.0, 424.0, 40.0, 13.0, "Theme"),
    (9.0, 451.0, 39.0, 13.0, "Layout"),
    (9.0, 477.0, 43.0, 13.0, "GCS ID"),
    (9.0, 502.0, 64.0, 13.0, "Aircraft Icon"),
    (841.0, 502.0, 63.0, 13.0, "Line Length"),
    (9.0, 527.0, 81.0, 13.0, "Inactive Aircraft"),
    (9.0, 656.0, 96.0, 13.0, "Map Access Mode"),
];

/// A `CheckBox`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CheckSpec {
    /// Its Designer `Name`, which is also its control id after `planner-`.
    pub name: &'static str,
    /// Its `Text`.
    pub text: &'static str,
    /// `Location` and `Size`.
    pub at: (u16, u16, u16, u16),
    /// The key it is set from and written to, where it has one.
    pub key: Option<&'static str>,
    /// `Checked` in the Designer.
    pub designer: bool,
    /// Why it is dimmed here, for a box whose handler drives something this application lacks.
    pub dim: Option<&'static str>,
}

const fn check(
    name: &'static str,
    text: &'static str,
    at: (u16, u16, u16, u16),
    key: Option<&'static str>,
    designer: bool,
    dim: Option<&'static str>,
) -> CheckSpec {
    CheckSpec {
        name,
        text,
        at,
        key,
        designer,
        dim,
    }
}

/// The speech boxes Enable Speech shows and hides.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:388-409`
pub const SPEECH_BOXES: [&str; 8] = [
    "CHK_speechArmedOnly",
    "CHK_speechwaypoint",
    "CHK_speechaltwarning",
    "CHK_speechbattery",
    "CHK_speechcustom",
    "CHK_speechmode",
    "CHK_speecharmdisarm",
    "CHK_speechlowspeed",
];

/// Every check box the page draws, with the key `Activate` sets it from.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.resx; ConfigPlanner.Designer.cs:37-892;
/// ConfigPlanner.cs:149-190, 239-244`
pub const CHECKS: &[CheckSpec] = &[
    check(
        "CHK_hudshow",
        "Enable HUD Overlay",
        (520, 10, 133, 18),
        // Written by the handler; read by `MainV2` into `hudon`, which `Activate` shows while a
        // capture runs. C#: ConfigPlanner.cs:137-146, 375-379; MainV2.cs:940-941
        Some("CHK_hudshow"),
        true,
        None,
    ),
    check(
        "CHK_enablespeech",
        "Enable Speech",
        (107, 89, 99, 17),
        Some("speechenable"),
        false,
        None,
    ),
    check(
        "CHK_speechArmedOnly",
        "Only when Armed",
        (212, 89, 109, 17),
        Some("speech_armed_only"),
        false,
        None,
    ),
    check(
        "CHK_speechwaypoint",
        "Waypoint",
        (332, 89, 71, 17),
        Some("speechwaypointenabled"),
        false,
        None,
    ),
    check(
        "CHK_speechmode",
        "Mode ",
        (408, 89, 56, 17),
        Some("speechmodeenabled"),
        false,
        None,
    ),
    check(
        "CHK_speechcustom",
        "30s Interval",
        (469, 89, 81, 17),
        Some("speechcustomenabled"),
        false,
        None,
    ),
    check(
        "CHK_speechbattery",
        "Battery Warning",
        (559, 89, 102, 17),
        Some("speechbatteryenabled"),
        false,
        None,
    ),
    check(
        "CHK_speechaltwarning",
        "Alt Warning",
        (671, 89, 81, 17),
        Some("speechaltenabled"),
        false,
        None,
    ),
    check(
        "CHK_speecharmdisarm",
        "Arm/Disarm",
        (760, 89, 81, 17),
        Some("speecharmenabled"),
        false,
        None,
    ),
    check(
        "CHK_speechlowspeed",
        "Low Speed",
        (853, 89, 80, 17),
        Some("speechlowspeedenabled"),
        false,
        None,
    ),
    check(
        "CHK_resetapmonconnect",
        "Reset on USB Connect (toggle DTR)",
        (107, 277, 217, 17),
        Some("CHK_resetapmonconnect"),
        false,
        None,
    ),
    check(
        "CHK_rtsresetesp32",
        "Disable RTS reset on ESP32 SerialUSB",
        (320, 277, 251, 19),
        Some("CHK_rtsresetesp32"),
        false,
        None,
    ),
    check(
        "CHK_disttohomeflightdata",
        "Display in Flightdata",
        (267, 302, 129, 17),
        Some("CHK_disttohomeflightdata"),
        true,
        None,
    ),
    check(
        "CHK_loadwponconnect",
        "Load Waypoints on connect?",
        (107, 326, 177, 17),
        Some("loadwpsonconnect"),
        false,
        None,
    ),
    check(
        "CHK_GDIPlus",
        "GDI+ (old type/no HW acceleration)",
        (107, 349, 220, 17),
        Some("CHK_GDIPlus"),
        false,
        Some("gpui draws the HUD: there is no GDI+ renderer to choose"),
    ),
    check(
        "CHK_maprotation",
        "Map is rotated to follow the plane",
        (107, 372, 205, 17),
        Some("CHK_maprotation"),
        false,
        None,
    ),
    check(
        "chk_displaycog",
        "Display COG",
        (107, 501, 86, 17),
        Some("GMapMarkerBase_DisplayCOG"),
        false,
        None,
    ),
    check(
        "chk_displayheading",
        "Display Heading",
        (199, 501, 103, 17),
        Some("GMapMarkerBase_DisplayHeading"),
        false,
        None,
    ),
    check(
        "chk_displaynavbearing",
        "Display Nav Bearing",
        (308, 501, 122, 17),
        Some("GMapMarkerBase_DisplayNavBearing"),
        false,
        None,
    ),
    check(
        "chk_displayradius",
        "Display Turn Radius",
        (436, 501, 121, 17),
        Some("GMapMarkerBase_DisplayRadius"),
        false,
        None,
    ),
    check(
        "chk_displaytarget",
        "Display Target",
        (563, 501, 94, 17),
        Some("GMapMarkerBase_DisplayTarget"),
        false,
        None,
    ),
    check(
        "chk_displaytooltip",
        "Display ToolTip",
        (663, 501, 99, 17),
        Some("mapicondesc"),
        false,
        None,
    ),
    check(
        "CHK_Password",
        "Password Protect Config",
        (340, 554, 142, 17),
        Some("password_protect"),
        false,
        Some("no password protects the configuration screens here"),
    ),
    check(
        "CHK_showairports",
        "Show Airports",
        (496, 554, 91, 17),
        Some("showairports"),
        true,
        None,
    ),
    check(
        "chk_ADSB",
        "ADSB",
        (598, 554, 55, 17),
        Some("enableadsb"),
        false,
        Some("no ADSB server client: traffic comes from the vehicle's ADSB_VEHICLE"),
    ),
    check(
        "chk_shownofly",
        "No Fly",
        (671, 554, 56, 17),
        Some("ShowNoFly"),
        true,
        None,
    ),
    check(
        "chk_analytics",
        "OptOut Anon Stats",
        (107, 577, 115, 17),
        Some("analyticsoptout"),
        false,
        Some("no anonymous statistics are sent: nothing to opt out of"),
    ),
    check(
        "CHK_beta",
        "Beta Updates",
        (228, 577, 91, 17),
        Some("beta_updates"),
        false,
        Some("no updater"),
    ),
    check(
        "chk_norcreceiver",
        "No RC Receiver",
        (340, 577, 104, 17),
        Some("norcreceiver"),
        false,
        None,
    ),
    check(
        "chk_tfr",
        "TFR's",
        (496, 577, 54, 17),
        Some("showtfr"),
        true,
        None,
    ),
    check(
        "CHK_params_bg",
        "Params Download in BackGround",
        (107, 600, 186, 17),
        Some("Params_BG"),
        false,
        None,
    ),
    check(
        "chk_slowMachine",
        "Runing on a slow computer",
        (340, 600, 155, 17),
        Some("SlowMachine"),
        false,
        None,
    ),
    check(
        "CHK_mavdebug",
        "Mavlink Message Debug",
        (106, 623, 155, 17),
        None,
        false,
        Some("no MAVLink message debug log"),
    ),
    check(
        "chk_temp",
        "Testing Screen",
        (340, 623, 144, 17),
        None,
        false,
        Some("no Testing Screen"),
    ),
];

/// A `ComboBox`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ComboSpec {
    /// Its Designer `Name`.
    pub name: &'static str,
    /// `Location` and `Size`.
    pub at: (u16, u16, u16, u16),
    /// Its items, as the Designer adds them or `Activate` binds them.
    pub items: &'static [&'static str],
    /// Why it is dimmed here.
    pub dim: Option<&'static str>,
}

const fn combo(
    name: &'static str,
    at: (u16, u16, u16, u16),
    items: &'static [&'static str],
    dim: Option<&'static str>,
) -> ComboSpec {
    ComboSpec {
        name,
        at,
        items,
        dim,
    }
}

/// `SeverityLevel`, which the constructor adds to `CMB_severity`.
/// `// C#: ExtLibs/Utilities/DisplayView.cs:22-32; ConfigPlanner.cs:37-44`
const SEVERITIES: &[&str] = &[
    "Emergency",
    "Alert",
    "Critical",
    "Error",
    "Warning",
    "Notice",
    "Info",
    "Debug",
];
/// `distances` and `altitudes`. `// C#: ExtLibs/Utilities/distances.cs:3-7, altitudes.cs:3-7`
const DISTANCES: &[&str] = &["Meters", "Feet"];
/// `speeds`. `// C#: ExtLibs/Utilities/speeds.cs:3-10`
const SPEEDS: &[&str] = &["meters_per_second", "fps", "kph", "mph", "knots"];
/// `GMapMarkerBase.InactiveDisplayStyleEnum`. `// C#: ExtLibs/Maps/GMapMarkerBase.cs:34-39`
const INACTIVE_STYLES: &[&str] = &["Normal", "Transparent", "Hidden"];
/// `GMap.NET.AccessMode`. `// C#: ExtLibs/GMap.NET.Core/GMap.NET/AccessMode.cs:6-22`
const ACCESS_MODES: &[&str] = &["ServerOnly", "ServerAndCache", "CacheOnly"];
/// `DisplayNames`, which the constructor adds to `CMB_Layout`.
/// `// C#: ExtLibs/Utilities/DisplayView.cs:15-20; ConfigPlanner.cs:31-33`
const LAYOUTS: &[&str] = &["Basic", "Advanced", "Custom"];

/// The rate combos' items, as the Designer adds them.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.resx (CMB_rate*.Items*)`
const RATES_ATTITUDE: &[&str] = &[
    "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "50", "100",
];
const RATES_POSITION: &[&str] = &[
    "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "20", "50", "100",
];
const RATES_STATUS: &[&str] = &[
    "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "25", "35", "50",
];
const RATES_SENSORS: &[&str] = &[
    "-1", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "25", "50", "100",
];

/// Every combo box the page draws.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.resx; ConfigPlanner.cs:46, 81-88, 118-119, 247`
pub const COMBOS: &[ComboSpec] = &[
    // Bound at run time, to the devices and to a device's formats: [`Video`]'s lists.
    // `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:353, 742-751`
    combo("CMB_videosources", (107, 8, 245, 21), &[], None),
    combo("CMB_videoresolutions", (107, 35, 408, 21), &[], None),
    combo(
        "CMB_osdcolor",
        (107, 62, 138, 21),
        &[],
        Some("its handler's body is commented out: choosing a colour changes nothing"),
    ),
    combo("CMB_severity", (107, 111, 138, 21), SEVERITIES, None),
    combo(
        "CMB_language",
        (107, 139, 138, 21),
        &[],
        Some("the language is config.xml's `language`, read at start-up"),
    ),
    combo("CMB_distunits", (107, 195, 138, 21), DISTANCES, None),
    combo("CMB_altunits", (320, 195, 138, 21), DISTANCES, None),
    combo("CMB_speedunits", (107, 222, 138, 21), SPEEDS, None),
    combo("CMB_rateattitude", (166, 251, 40, 21), RATES_ATTITUDE, None),
    combo("CMB_rateposition", (282, 251, 40, 21), RATES_POSITION, None),
    combo("CMB_ratestatus", (415, 251, 40, 21), RATES_STATUS, None),
    combo("CMB_raterc", (489, 251, 40, 21), RATES_STATUS, None),
    combo("CMB_ratesensors", (584, 251, 40, 21), RATES_SENSORS, None),
    combo(
        "CMB_theme",
        (107, 421, 138, 21),
        &[],
        Some("the dark palette is ratified: there is no theme to load"),
    ),
    combo("CMB_Layout", (107, 448, 138, 21), LAYOUTS, None),
    combo(
        "cmb_secondarydisplaystyle",
        (107, 524, 138, 21),
        INACTIVE_STYLES,
        None,
    ),
    combo("CMB_mapCache", (107, 653, 138, 21), ACCESS_MODES, None),
];

/// A `MyButton`: `Name`, `Location`, `Size`, `Text`, and why it is dimmed.
type ButtonSpec = (
    &'static str,
    (u16, u16, u16, u16),
    &'static str,
    Option<&'static str>,
);

/// Every button the page draws.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.resx (BUT_*)`
pub const BUTTONS: &[ButtonSpec] = &[
    ("BUT_videostart", (358, 6, 75, 23), "Start", None),
    ("BUT_videostop", (439, 6, 75, 23), "Stop", None),
    ("BUT_Joystick", (107, 166, 99, 23), "Joystick Setup", None),
    ("BUT_logdirbrowse", (496, 393, 75, 23), "Browse", None),
    (
        "BUT_themecustom",
        (249, 421, 75, 20),
        "Custom",
        Some("the dark palette is ratified: there is no theme editor"),
    ),
    (
        "BUT_Vario",
        (107, 551, 99, 20),
        "Start/Stop Vario",
        Some("no audio variometer"),
    ),
    (
        "BUT_mapCacheDir",
        (251, 653, 94, 21),
        "Open Map Cache",
        None,
    ),
];

/// A `NumericUpDown`: `Name`, `Location`, `Size`, and `Minimum`, `Maximum`, `Increment` and the
/// Designer's `Value`.
type NumberSpec = (&'static str, (u16, u16, u16, u16), (f64, f64, f64, f64));

/// The three number boxes.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.Designer.cs:203-225, 727-742, 834-852`
pub const NUMBERS: [NumberSpec; 3] = [
    (
        "NUM_tracklength",
        (107, 300, 67, 20),
        (100.0, 200_000.0, 100.0, 200.0),
    ),
    ("num_gcsid", (107, 475, 53, 20), (1.0, 255.0, 1.0, 255.0)),
    (
        "num_linelength",
        (768, 500, 67, 20),
        (10.0, 2000.0, 10.0, 200.0),
    ),
];

/// `txt_log_dir`.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.resx (txt_log_dir.Location, .Size)`
const LOG_DIR_AT: (f32, f32, f32, f32) = (107.0, 395.0, 386.0, 20.0);

/// Every control drawn dimmed, by name.
#[must_use]
pub fn dimmed() -> Vec<&'static str> {
    CHECKS
        .iter()
        .filter(|spec| spec.dim.is_some())
        .map(|spec| spec.name)
        .chain(
            COMBOS
                .iter()
                .filter(|spec| spec.dim.is_some())
                .map(|spec| spec.name),
        )
        .chain(
            BUTTONS
                .iter()
                .filter(|(_, _, _, dim)| dim.is_some())
                .map(|(name, ..)| *name),
        )
        .collect()
}

// -------------------------------------------------------------------------------------------------
// The telemetry rates.
// -------------------------------------------------------------------------------------------------

/// One telemetry rate: its combo, the streams its handler asks for, and `CurrentState`'s default
/// for its backup.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rate {
    /// The combo, whose `Name` is also the key it writes.
    pub combo: &'static str,
    /// The `MAV_DATA_STREAM`s asked for, in the handler's order.
    pub streams: &'static [u8],
    /// `rateXbackup`'s initial value.
    pub default: i32,
}

#[allow(clippy::cast_possible_truncation)] // the stream ids are all below 13
const fn stream(id: MavDataStream) -> u8 {
    id.0 as u8
}

/// The five rates, in `cs`'s order: attitude, position, status, RC, sensors.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:574-641; ExtLibs/ArduPilot/CurrentState.cs:199-206`
pub const RATES: [Rate; 5] = [
    Rate {
        combo: "CMB_rateattitude",
        streams: &[
            stream(MavDataStream::MAV_DATA_STREAM_EXTRA1),
            stream(MavDataStream::MAV_DATA_STREAM_EXTRA2),
        ],
        default: 4,
    },
    Rate {
        combo: "CMB_rateposition",
        streams: &[stream(MavDataStream::MAV_DATA_STREAM_POSITION)],
        default: 2,
    },
    Rate {
        combo: "CMB_ratestatus",
        streams: &[stream(MavDataStream::MAV_DATA_STREAM_EXTENDED_STATUS)],
        default: 2,
    },
    Rate {
        combo: "CMB_raterc",
        streams: &[stream(MavDataStream::MAV_DATA_STREAM_RC_CHANNELS)],
        default: 2,
    },
    Rate {
        combo: "CMB_ratesensors",
        streams: &[
            stream(MavDataStream::MAV_DATA_STREAM_EXTRA3),
            stream(MavDataStream::MAV_DATA_STREAM_RAW_SENSORS),
        ],
        default: 2,
    },
];

/// `requestDatastream(id, hzrate)` for the vehicle being shown: `REQUEST_DATA_STREAM`, started,
/// the rate as a byte, sent twice as `getDatastream` sends it. -1 sends nothing. Returns the
/// number of requests put on the link.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3058-3069, 3214-3216, 3243-3262`
pub fn request_datastream(telemetry: &Telemetry, stream: u8, hz: i32) -> usize {
    if hz == -1 {
        return 0;
    }
    let Some((sender, target)) = telemetry.send_handle() else {
        return 0;
    };
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // the C#'s `(byte) hzrate`
    let rate = u16::from(hz as u8);
    let request = MavMessage::RequestDataStream(RequestDataStream {
        req_message_rate: rate,
        target_system: target.payload_target(),
        target_component: target.compid,
        req_stream_id: stream,
        start_stop: 1,
    });
    usize::from(sender.send(&request)) + usize::from(sender.send(&request))
}

// -------------------------------------------------------------------------------------------------
// The InputBoxes the speech boxes ask in.
// -------------------------------------------------------------------------------------------------

/// How an answer is written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Store {
    /// As typed, under the step's key.
    Text,
    /// `double.Parse(answer) / CurrentState.multiplieralt`, saved in metres.
    AltHeight,
    /// Under `mapicondesc` and `mapicondesc_default` both.
    IconDescription,
}

/// One `InputBox.Show` a handler makes: its title, its question, the key its default is read from
/// and its answer written to, and the default when the key is not set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Step {
    /// The box's title.
    pub title: &'static str,
    /// Its question.
    pub question: &'static str,
    /// The key.
    pub key: &'static str,
    /// The literal the handler starts from.
    pub default: &'static str,
    store: Store,
}

const fn ask(
    title: &'static str,
    question: &'static str,
    key: &'static str,
    default: &'static str,
) -> Step {
    Step {
        title,
        question,
        key,
        default,
        store: Store::Text,
    }
}

/// `InputBox.Show`'s question for the templates.
const SAY: &str = "What do you want it to say?";

/// What each speech box asks when it is ticked, in order; a Cancel ends the handler.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:443-459, 461-477, 479-495, 519-551,
/// 660-686, 811-833, 877-916, 1109-1134`
fn steps(name: &str) -> &'static [Step] {
    const WAYPOINT: &[Step] = &[ask(
        "Notification",
        SAY,
        "speechwaypoint",
        "Heading to Waypoint {wpn}",
    )];
    const MODE: &[Step] = &[ask(
        "Notification",
        SAY,
        "speechmode",
        "Mode changed to {mode}",
    )];
    const CUSTOM: &[Step] = &[ask(
        "Notification",
        SAY,
        "speechcustom",
        "Heading to Waypoint {wpn}, altitude is {alt}, Ground speed is {gsp} ",
    )];
    const BATTERY: &[Step] = &[
        ask(
            "Notification",
            SAY,
            "speechbattery",
            "WARNING, Battery at {batv} Volt, {batp} percent",
        ),
        ask(
            "Battery Level",
            "What Voltage do you want to warn at?",
            "speechbatteryvolt",
            "9.6",
        ),
        ask(
            "Battery Level",
            "What percentage do you want to warn at?",
            "speechbatterypercent",
            "20",
        ),
    ];
    const ALT: &[Step] = &[
        ask(
            "Notification",
            SAY,
            "speechalt",
            "WARNING, low altitude {alt}",
        ),
        Step {
            title: "Min Alt",
            question: "What altitude do you want to warn at? (relative to home)",
            key: "speechaltheight",
            default: "2",
            store: Store::AltHeight,
        },
    ];
    const ARM: &[Step] = &[
        ask("Arm", SAY, "speecharm", "Armed"),
        ask("Disarmed", SAY, "speechdisarm", "Disarmed"),
    ];
    const LOW_SPEED: &[Step] = &[
        ask(
            "Ground Speed",
            SAY,
            "speechlowgroundspeed",
            "Low Ground Speed {gsp}",
        ),
        ask(
            "speed trigger",
            "What speed do you want to warn at (m/s)?",
            "speechlowgroundspeedtrigger",
            "0",
        ),
        ask("Air Speed", SAY, "speechlowairspeed", "Low Air Speed {asp}"),
        ask(
            "speed trigger",
            "What speed do you want to warn at (m/s)?",
            "speechlowairspeedtrigger",
            "0",
        ),
    ];
    const TOOLTIP: &[Step] = &[Step {
        title: "Description",
        question: "What do you want it to show?",
        key: "mapicondesc_default",
        default: "{alt}{altunit} {airspeed}{speedunit} id:{sysid} Sats:{satcount} HDOP:{gpshdop} \
                  Volts:{battery_voltage}",
        store: Store::IconDescription,
    }];
    match name {
        "CHK_speechwaypoint" => WAYPOINT,
        "CHK_speechmode" => MODE,
        "CHK_speechcustom" => CUSTOM,
        "CHK_speechbattery" => BATTERY,
        "CHK_speechaltwarning" => ALT,
        "CHK_speecharmdisarm" => ARM,
        "CHK_speechlowspeed" => LOW_SPEED,
        "chk_displaytooltip" => TOOLTIP,
        _ => &[],
    }
}

/// The `InputBox` showing: the step, and the text in its box.
#[derive(Debug)]
pub struct Prompt {
    /// What it asks.
    pub step: Step,
    /// The box's text, which starts as the key's value or the step's default.
    pub field: TextField,
}

/// A message box: its caption and its text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    /// The caption.
    pub title: &'static str,
    /// The text.
    pub text: String,
}

/// `Program.handleException`'s box, which an exception out of a handler reaches: its caption, and
/// the text before the exception.
/// `// C#: Program.cs:793-795`
const UNHANDLED: (&str, &str) = ("Send Error", "An error has occurred\n");

// -------------------------------------------------------------------------------------------------
// The number boxes.
// -------------------------------------------------------------------------------------------------

/// A `NumericUpDown` with no decimal places: its `Value`, its bounds and increment, and the text
/// in its box, which is read into `Value` on Enter, on an arrow, and on leaving the box.
#[derive(Debug)]
pub struct Number {
    /// The box's text.
    pub field: TextField,
    value: f64,
    minimum: f64,
    maximum: f64,
    increment: f64,
    edited: bool,
}

impl Number {
    fn new((minimum, maximum, increment, value): (f64, f64, f64, f64)) -> Self {
        let mut field = TextField::new("");
        field.set(number_text(value));
        Self {
            field,
            value,
            minimum,
            maximum,
            increment,
            edited: false,
        }
    }

    /// `Value`.
    #[must_use]
    pub const fn value(&self) -> f64 {
        self.value
    }

    /// `Value = v`: returns whether it changed, which is when `ValueChanged` fires. The setter
    /// throws on a value out of range; a stored setting out of range is held to it instead.
    fn set(&mut self, value: f64) -> bool {
        let value = value.clamp(self.minimum, self.maximum);
        self.field.set(number_text(value));
        self.edited = false;
        let changed = (value - self.value).abs() > f64::EPSILON;
        self.value = value;
        changed
    }

    /// `ValidateEditText`: the typed text read into `Value` when it parses, constrained to the
    /// bounds. Returns whether `Value` changed.
    fn commit(&mut self) -> bool {
        if !self.edited {
            return false;
        }
        let typed = self
            .field
            .value()
            .trim()
            .parse::<f64>()
            .ok()
            .filter(|typed| typed.is_finite());
        self.set(typed.unwrap_or(self.value))
    }

    /// `UpButton` and `DownButton`: validated, stepped by the increment, constrained.
    fn step(&mut self, up: bool) -> bool {
        let committed = self.commit();
        let delta = if up { self.increment } else { -self.increment };
        self.set(self.value + delta) || committed
    }
}

/// `Value` as the box shows it at `DecimalPlaces` 0: `decimal.ToString("F0")`.
#[allow(clippy::cast_possible_truncation)] // a box's value, within its bounds
fn number_text(value: f64) -> String {
    format!("{}", value.round() as i64)
}

/// `decimal.ToString()`, which the handlers write.
fn decimal_text(value: f64) -> String {
    format!("{value}")
}

// -------------------------------------------------------------------------------------------------
// The page.
// -------------------------------------------------------------------------------------------------

/// What a handler asks of the rest of the application.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Effect {
    /// `requestDatastream(stream, hz)`.
    Stream(u8, i32),
    /// `loadwpsonconnect`: whether the mission is read when a vehicle connects.
    /// `// C#: MainV2.cs:1752-1761`
    ReadMissionOnConnect(bool),
    /// `GMaps.Instance.Mode` changed: the map's tile store is made again.
    MapAccess,
    /// `BUT_mapCacheDir`: the directory opened.
    OpenDirectory(PathBuf),
    /// `BUT_logdirbrowse`: the folder dialog.
    BrowseLogDirectory,
    /// `BUT_videostart`: `new Capture(index, media)` and `Start()` for the device and format
    /// chosen; [`run_video`] does it and, when it starts, writes the two settings.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:262-286`
    VideoStart {
        /// `CMB_videosources.SelectedIndex`'s device.
        device: mp_video::Device,
        /// `CMB_videoresolutions.SelectedItem`'s format.
        mode: mp_video::Mode,
    },
    /// `BUT_videostop`: `MainV2.cam.Dispose()`, and `MainV2.cam = null`.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:288-295`
    VideoStop,
    /// Words for the status line: "Camera Fail: " and why, where the C# has a message box.
    Status(String),
}

/// The video controls: `CMB_videosources` and `CMB_videoresolutions` with what they are bound
/// to, and whether a capture runs, which is `MainV2.cam` being set and `BUT_videostart` disabled.
/// The capture itself is the application's (`MissionPlanner::video`), as `MainV2.cam` is.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:137-146, 216-233, 262-361, 742-751`
pub struct Video {
    /// Where devices come from: V4L2 on Linux (`mp_video::platform_source`). None elsewhere yet,
    /// where the lists stay empty as the C#'s do under mono (`MainV2.MONO`).
    source: Option<Arc<dyn mp_video::Source>>,
    /// `CMB_videosources.DataSource`: `Capture.getDevices()`, by name.
    devices: Vec<mp_video::Device>,
    /// `CMB_videoresolutions.DataSource`: the device's `GCSBitmapInfo`s.
    modes: Vec<mp_video::Mode>,
    /// The two lists as their combos show them, each row keyed by its index.
    device_list: Combo,
    mode_list: Combo,
    /// `MainV2.cam != null`: `BUT_videostart` is disabled.
    running: bool,
    /// The running capture's frames decoded, as the application last saw them.
    frames: u64,
    /// Why the last Start was refused, or the running capture's last read failure.
    error: Option<String>,
}

impl std::fmt::Debug for Video {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Video")
            .field("source", &self.source.is_some())
            .field("devices", &self.devices)
            .field("modes", &self.modes)
            .field("running", &self.running)
            .field("frames", &self.frames)
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

impl Video {
    fn new(source: Option<Arc<dyn mp_video::Source>>) -> Self {
        Self {
            source,
            devices: Vec::new(),
            modes: Vec::new(),
            device_list: Combo::default(),
            mode_list: Combo::default(),
            running: false,
            frames: 0,
            error: None,
        }
    }

    /// The row selected, as an index.
    fn index(list: &Combo) -> Option<usize> {
        list.selected.and_then(|key| usize::try_from(key).ok())
    }

    /// Binds a list, as setting `DataSource` does: its first row selected, or none for an empty
    /// list.
    fn bind(list: &mut Combo, texts: impl Iterator<Item = String>) {
        list.options = (0_i64..).zip(texts).collect();
        list.selected = (!list.options.is_empty()).then_some(0);
        list.top_index = 0;
        list.enabled = true;
    }

    /// `CMB_videosources.SelectedIndex`'s device.
    fn device(&self) -> Option<&mp_video::Device> {
        Self::index(&self.device_list).and_then(|index| self.devices.get(index))
    }

    /// `CMB_videoresolutions.SelectedItem`.
    fn mode(&self) -> Option<&mp_video::Mode> {
        Self::index(&self.mode_list).and_then(|index| self.modes.get(index))
    }

    /// A combo's list, for the two combos this holds.
    fn list(&self, name: &str) -> Option<&Combo> {
        match name {
            "CMB_videosources" => Some(&self.device_list),
            "CMB_videoresolutions" => Some(&self.mode_list),
            _ => None,
        }
    }

    fn list_mut(&mut self, name: &str) -> Option<&mut Combo> {
        match name {
            "CMB_videosources" => Some(&mut self.device_list),
            "CMB_videoresolutions" => Some(&mut self.mode_list),
            _ => None,
        }
    }
}

/// The Planner page: what each control holds, the box or dialog showing, and what the handlers
/// have asked for.
#[derive(Debug)]
pub struct Planner {
    active: bool,
    checks: BTreeMap<&'static str, bool>,
    /// Each combo's `SelectedIndex`, `None` for -1.
    selected: BTreeMap<&'static str, Option<usize>>,
    /// The text a dimmed combo shows.
    dim_text: BTreeMap<&'static str, String>,
    /// The combo dropped down.
    open: Option<&'static str>,
    numbers: [Number; 3],
    log_dir: TextField,
    prompt: Option<Prompt>,
    queued: VecDeque<Step>,
    messages: VecDeque<Message>,
    joystick: bool,
    /// `CurrentState`'s multipliers and unit names, as `ChangeUnits` last set them.
    units: DisplayUnits,
    /// `cs.rateattitude`, `rateposition`, `ratestatus`, `raterc`, `ratesensors`.
    rates: [i32; 5],
    /// `MAVLinkInterface.gcssysid`.
    gcssysid: u8,
    /// The number box that had the focus last frame.
    focused: Option<usize>,
    effects: Vec<Effect>,
    /// The requests put on the link, as `stream@hz`, oldest first.
    sent: Vec<(u8, i32)>,
    /// `label5.Visible` and `CMB_Layout.Visible` set false by `Activate`, which nothing sets
    /// back. `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:75-79`
    layout_hidden: bool,
    /// The video controls.
    video: Video,
    /// `FlightData.myhud.hudon`: whether the HUD's instruments are drawn over the camera's
    /// picture. The HUD's property, held here as `MAVLinkInterface.gcssysid` is: `MainV2` sets it
    /// from `CHK_hudshow` before any page shows, and Enable HUD Overlay sets it.
    /// `// C#: ExtLibs/Controls/HUD.cs:211-212, 274; MainV2.cs:940-941; ConfigPlanner.cs:375-379`
    hudon: bool,
}

impl Planner {
    /// What `MainV2` sets up from the settings before any page shows: `ChangeUnits`, the rates'
    /// backups (`ResetInternals` copies them into `cs`), `gcssysid`, and the HUD's `hudon`.
    /// `// C#: MainV2.cs:683, 838, 940-941, 983-1004; ExtLibs/ArduPilot/CurrentState.cs:199-206,
    /// 4385-4397`
    #[must_use]
    pub fn new(settings: &Persisted) -> Self {
        let mut rates = [0; 5];
        for (rate, held) in RATES.iter().zip(rates.iter_mut()) {
            *held = if settings.get(rate.combo).is_some() {
                get_int(settings, rate.combo, 0)
            } else {
                rate.default
            };
        }
        let mut planner = Self {
            active: false,
            checks: CHECKS
                .iter()
                .map(|spec| (spec.name, spec.designer))
                .collect(),
            selected: COMBOS.iter().map(|spec| (spec.name, None)).collect(),
            dim_text: BTreeMap::new(),
            open: None,
            numbers: NUMBERS.map(|(_, _, bounds)| Number::new(bounds)),
            log_dir: TextField::new(""),
            prompt: None,
            queued: VecDeque::new(),
            messages: VecDeque::new(),
            joystick: false,
            units: DisplayUnits::default(),
            rates,
            gcssysid: settings
                .get("gcsid")
                .and_then(|value| value.trim().parse().ok())
                .unwrap_or(255),
            focused: None,
            effects: Vec::new(),
            sent: Vec::new(),
            layout_hidden: false,
            video: Video::new(mp_video::platform_source()),
            // `hudon = bool.Parse(Settings.Instance["CHK_hudshow"])` when the key is there, else
            // the HUD's own true. A value `bool.Parse` refuses throws out of `MainV2`'s
            // constructor; here it leaves the HUD's true.
            // C#: MainV2.cs:940-941; ExtLibs/Controls/HUD.cs:268-274
            hudon: get_bool(settings, "CHK_hudshow", true),
        };
        planner.change_units(settings);
        planner
    }

    /// The same page with its devices from `source`: a test's scripted camera.
    #[cfg(test)]
    #[must_use]
    pub fn with_video_source(mut self, source: Arc<dyn mp_video::Source>) -> Self {
        self.video.source = Some(source);
        self
    }

    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Whether a box is ticked.
    #[must_use]
    pub fn checked(&self, name: &str) -> bool {
        self.checks.get(name).copied().unwrap_or(false)
    }

    /// Whether a box is drawn: the speech boxes only while Enable Speech is ticked. Their `.resx`
    /// `Visible` is false, and Enable Speech's handler sets it to its own state.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:381-410`
    #[must_use]
    pub fn shown(&self, name: &str) -> bool {
        !SPEECH_BOXES.contains(&name) || self.checked("CHK_enablespeech")
    }

    /// The text a combo shows.
    #[must_use]
    pub fn combo_text(&self, name: &str) -> String {
        if let Some(text) = self.dim_text.get(name) {
            return text.clone();
        }
        if let Some(list) = self.video.list(name) {
            return list
                .selected
                .and_then(|key| list.options.iter().find(|(held, _)| *held == key))
                .map_or_else(String::new, |(_, text)| text.clone());
        }
        let Some(spec) = COMBOS.iter().find(|spec| spec.name == name) else {
            return String::new();
        };
        self.selected
            .get(name)
            .copied()
            .flatten()
            .and_then(|index| spec.items.get(index))
            .map_or_else(String::new, |text| (*text).to_owned())
    }

    /// A combo's items: the Designer's, or what a video combo is bound to.
    #[must_use]
    pub fn combo_items(&self, name: &str) -> Vec<String> {
        if let Some(list) = self.video.list(name) {
            return list.options.iter().map(|(_, text)| text.clone()).collect();
        }
        COMBOS
            .iter()
            .find(|spec| spec.name == name)
            .map(|spec| spec.items.iter().map(|item| (*item).to_owned()).collect())
            .unwrap_or_default()
    }

    /// The combo dropped down.
    #[must_use]
    pub const fn dropdown(&self) -> Option<&'static str> {
        self.open
    }

    /// Whether a capture runs: `MainV2.cam != null`, and `BUT_videostart` disabled.
    #[must_use]
    pub const fn video_running(&self) -> bool {
        self.video.running
    }

    /// What the application's capture says, once a frame: how many frames it has decoded and
    /// its last failure. `None` once it has gone.
    pub fn video_status(&mut self, capture: Option<&mp_video::Capture>) {
        if let Some(capture) = capture {
            self.video.frames = capture.frames();
            if let Some(error) = capture.error() {
                self.video.error = Some(error);
            }
        }
    }

    /// `CMB_videosources_Click`: the device list bound again, which selects its first device
    /// and so runs `SelectedIndexChanged` for it. Under mono the C# returns at once; with no
    /// source here the list stays as it was.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:742-751`
    fn video_click(&mut self, settings: &Persisted) {
        let Some(source) = self.video.source.clone() else {
            return;
        };
        self.video.devices = source.devices();
        Video::bind(
            &mut self.video.device_list,
            self.video.devices.iter().map(|device| device.name.clone()),
        );
        if self.video.devices.is_empty() {
            // Nothing to list the formats of: the C# indexes the empty device array and throws.
            self.video.modes.clear();
            Video::bind(&mut self.video.mode_list, std::iter::empty());
        } else {
            self.video_device_changed(settings);
        }
    }

    /// `CMB_videosources_SelectedIndexChanged`: the device's capture pin's stream capabilities
    /// bound to `CMB_videoresolutions` as `GCSBitmapInfo`s, then the saved `video_options`
    /// selected when it is an index the list has. A device that will not say is the C#'s "Can
    /// not add video source" box, and the format list is left as it was.
    ///
    /// Divergence: V4L2 where the C# has DirectShow's `IAMStreamConfig`, and only the MJPEG and
    /// YUYV formats are listed - the two `mp_video` decodes.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:297-361, 969-990`
    fn video_device_changed(&mut self, settings: &Persisted) {
        let (Some(source), Some(device)) =
            (self.video.source.clone(), self.video.device().cloned())
        else {
            return;
        };
        let modes = match source.modes(&device) {
            Ok(modes) => modes,
            Err(why) => {
                self.messages.push_back(Message {
                    title: "",
                    text: format!("Can not add video source\n{why}"),
                });
                return;
            }
        };
        Video::bind(
            &mut self.video.mode_list,
            modes.iter().map(mp_video::Mode::label),
        );
        self.video.modes = modes;
        // `Settings.Instance["video_options"] != ""`, which a missing key also passes; a bad
        // entry throws in `SelectedIndex`'s setter and is ignored.
        if settings.get("video_options") != Some("") && !device.name.is_empty() {
            let wanted = usize::try_from(get_int(settings, "video_options", 0)).ok();
            if let Some(index) = wanted.filter(|index| *index < self.video.modes.len()) {
                self.video.mode_list.selected = i64::try_from(index).ok();
            }
        }
    }

    /// A number box's value.
    #[must_use]
    pub fn number(&self, name: &str) -> Option<f64> {
        NUMBERS
            .iter()
            .position(|(held, ..)| *held == name)
            .and_then(|index| self.numbers.get(index))
            .map(Number::value)
    }

    /// The Log Path box's text.
    #[must_use]
    pub fn log_dir(&self) -> &str {
        self.log_dir.value()
    }

    /// The `InputBox` showing.
    #[must_use]
    pub const fn prompt(&self) -> Option<&Prompt> {
        self.prompt.as_ref()
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Whether the joystick page is open over this one.
    #[must_use]
    pub const fn joystick_open(&self) -> bool {
        self.joystick
    }

    /// `CurrentState`'s units.
    #[must_use]
    pub const fn units(&self) -> DisplayUnits {
        self.units
    }

    /// `cs.rateX`, in [`RATES`]' order.
    #[must_use]
    pub const fn rates(&self) -> [i32; 5] {
        self.rates
    }

    /// `MAVLinkInterface.gcssysid`.
    #[must_use]
    pub const fn gcssysid(&self) -> u8 {
        self.gcssysid
    }

    /// `FlightData.myhud.hudon`, which the flight screen hands the HUD every frame.
    /// `// C#: ExtLibs/Controls/HUD.cs:2005-2008`
    #[must_use]
    pub const fn hud_on(&self) -> bool {
        self.hudon
    }

    /// What the handlers have asked for since the last call.
    pub fn take_effects(&mut self) -> Vec<Effect> {
        std::mem::take(&mut self.effects)
    }

    /// Notes a request the link carried.
    pub fn sent(&mut self, stream: u8, hz: i32) {
        self.sent.push((stream, hz));
    }

    /// `MainV2.ChangeUnits`, from the settings as they are now.
    /// `// C#: MainV2.cs:4254-4337`
    fn change_units(&mut self, settings: &Persisted) {
        self.units = self.units.change_units(
            settings.get("distunits"),
            settings.get("altunits"),
            settings.get("speedunits"),
        );
    }

    /// Selects the item whose text is `text`, as setting a `DropDownList`'s `Text` does: one not
    /// in the list leaves the selection as it was.
    fn select_text(&mut self, name: &'static str, text: &str) {
        let Some(spec) = COMBOS.iter().find(|spec| spec.name == name) else {
            return;
        };
        if let Some(index) = spec.items.iter().position(|item| *item == text) {
            self.selected.insert(name, Some(index));
        }
    }

    /// `SetCheckboxFromConfig`: the key's value, when it is set.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:768-772`
    fn set_from_config(&mut self, name: &'static str, settings: &Persisted) {
        let Some(key) = CHECKS
            .iter()
            .find(|spec| spec.name == name)
            .and_then(|spec| spec.key)
        else {
            return;
        };
        if settings.get(key).is_some() {
            self.checks.insert(name, get_bool(settings, key, false));
        }
    }

    /// `Activate`: every control set from the settings. The handlers that do not check `startup`
    /// run when a control changes from what the Designer gave it, and three of them write: the
    /// severity's default, the aircraft icon's five boxes (read with a default of true, from a
    /// Designer that leaves them unticked), and the line length - which is read as
    /// `GMapMarkerBase_Length` and written as `GMapMarkerBase_length`, so it comes back as 500
    /// and is written as 500 on every activation. The Log Path box's text is set to `LogDir`,
    /// and its `TextChanged` writes the directory back when it exists.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:26-51, 55-257, 788-795, 1080-1141`
    pub fn activate(&mut self, settings: &mut Persisted, default_log_dir: Option<&Path>) {
        self.active = true;
        self.open = None;
        self.dim_text.clear();

        // C#: ConfigPlanner.cs:58-73 - the view's name selected; a change runs the handler.
        let index = match crate::display_view::current().name {
            DisplayName::Basic => 0,
            DisplayName::Advanced => 1,
            DisplayName::Custom => 2,
        };
        if self.selected.insert("CMB_Layout", Some(index)) != Some(Some(index)) {
            self.layout_changed(index, settings);
        }
        // C#: ConfigPlanner.cs:75-79
        if !crate::display_view::flag("displayPlannerLayout") {
            self.layout_hidden = true;
        }
        // C#: ConfigPlanner.cs:81 - `KnownColor`'s names; `hudcolor` selects one.
        // C#: ConfigPlanner.cs:195-206
        self.dim_text.insert(
            "CMB_osdcolor",
            settings.get("hudcolor").unwrap_or("").to_owned(),
        );
        // C#: ConfigPlanner.cs:84-86 - binding a list selects its first item.
        for name in ["CMB_distunits", "CMB_speedunits", "CMB_altunits"] {
            self.selected.insert(name, Some(0));
        }
        // C#: ConfigPlanner.cs:88-90; MainV2.cs:714-719 - the theme loaded at start-up.
        self.dim_text.insert(
            "CMB_theme",
            settings
                .get("theme")
                .unwrap_or("BurntKermit.mpsystheme")
                .to_owned(),
        );
        // C#: ConfigPlanner.cs:93
        let gcsid = f64::from(self.gcssysid);
        if self.numbers[1].set(gcsid) {
            self.number_changed(1, settings);
        }
        // C#: ConfigPlanner.cs:96-104
        if settings.get("severity").is_some() {
            let index = usize::try_from(get_int(settings, "severity", 0)).ok();
            self.selected.insert(
                "CMB_severity",
                index.filter(|index| *index < SEVERITIES.len()),
            );
        } else {
            self.selected.insert("CMB_severity", Some(4));
            settings.set("severity", "4");
        }
        // C#: ConfigPlanner.cs:107-135 - the UI culture's language; English only here.
        self.dim_text.insert("CMB_language", String::new());
        // C#: ConfigPlanner.cs:137-146 - Start disabled while a capture runs, which
        // `Video::running` holds across activations, and the overlay box set to `hudon`, its
        // handler run when that changes it. With no capture the box is what the Designer gave
        // the page the C# makes with the CONFIG screen - ticked, set before the handler is
        // hooked, so nothing runs - whatever `hudon` is (ConfigPlanner.Designer.cs:487-492;
        // BackstageView.cs:533-573 disposes the page when the screen closes). The page here
        // lives as long as the application, so every activation is the first of a new page's.
        if self.video.running {
            let hudon = self.hudon;
            if self.checks.insert("CHK_hudshow", hudon) != Some(hudon) {
                self.checked_changed("CHK_hudshow", hudon, settings);
            }
        } else {
            self.checks.insert("CHK_hudshow", true);
        }
        // C#: ConfigPlanner.cs:149-167
        for name in [
            "CHK_enablespeech",
            "CHK_speechwaypoint",
            "CHK_speechmode",
            "CHK_speechcustom",
            "CHK_speechbattery",
            "CHK_speechaltwarning",
            "CHK_speecharmdisarm",
            "CHK_speechlowspeed",
            "CHK_beta",
            "CHK_Password",
            "CHK_showairports",
            "chk_ADSB",
            "chk_norcreceiver",
            "chk_tfr",
            "chk_shownofly",
            "CHK_params_bg",
            "chk_slowMachine",
            "CHK_speechArmedOnly",
        ] {
            self.set_from_config(name, settings);
        }
        // C#: ConfigPlanner.cs:170
        let track = f64::from(get_int(settings, "NUM_tracklength", 200));
        if self.numbers[0].set(track) {
            self.number_changed(0, settings);
        }
        // C#: ConfigPlanner.cs:173-177
        for name in [
            "CHK_loadwponconnect",
            "CHK_resetapmonconnect",
            "CHK_rtsresetesp32",
        ] {
            self.set_from_config(name, settings);
        }
        // C#: ConfigPlanner.cs:179-183
        for (rate, value) in RATES.iter().zip(self.rates) {
            self.select_text(rate.combo, &value.to_string());
        }
        // C#: ConfigPlanner.cs:185-190
        for name in [
            "chk_analytics",
            "CHK_GDIPlus",
            "CHK_maprotation",
            "CHK_disttohomeflightdata",
        ] {
            self.set_from_config(name, settings);
        }
        // C#: ConfigPlanner.cs:209-214
        for (name, key) in [
            ("CMB_distunits", "distunits"),
            ("CMB_speedunits", "speedunits"),
            ("CMB_altunits", "altunits"),
        ] {
            if let Some(value) = settings.get(key).map(str::to_owned) {
                self.select_text(name, &value);
            }
        }
        // C#: ConfigPlanner.cs:216-233 - the saved device and format, by index. The indexes are
        // the C#'s and as fragile as they are there: a camera plugged in before this one, or a
        // node renumbered, selects another device (kept so, the owner not having said otherwise).
        if settings.get("video_device").is_some() {
            self.video_click(settings);
            let device = get_int(settings, "video_device", 0);
            if let Some(index) = usize::try_from(device)
                .ok()
                .filter(|index| *index < self.video.devices.len())
            {
                let key = i64::try_from(index).ok();
                if self.video.device_list.selected != key {
                    self.video.device_list.selected = key;
                    self.video_device_changed(settings);
                }
            }
            // Selected again as the C# does, whatever the device's handler selected.
            if settings.get("video_options") != Some("")
                && !self.combo_text("CMB_videosources").is_empty()
            {
                let wanted = usize::try_from(get_int(settings, "video_options", 0)).ok();
                if let Some(index) = wanted.filter(|index| *index < self.video.modes.len()) {
                    self.video.mode_list.selected = i64::try_from(index).ok();
                }
            }
        }
        // C#: ConfigPlanner.cs:236, 788-795; ExtLibs/Utilities/Settings.cs:127-140
        let log_dir = settings
            .get("logdirectory")
            .filter(|dir| !dir.is_empty())
            .map(PathBuf::from)
            .or_else(|| default_log_dir.map(Path::to_path_buf))
            .map(|dir| dir.display().to_string())
            .unwrap_or_default();
        if log_dir != self.log_dir.value() {
            self.log_dir.set(log_dir);
            self.log_dir_changed(settings);
        }
        // C#: ConfigPlanner.cs:239-243, 1080-1108
        for (name, key) in [
            ("chk_displaycog", "GMapMarkerBase_DisplayCOG"),
            ("chk_displayheading", "GMapMarkerBase_DisplayHeading"),
            ("chk_displaynavbearing", "GMapMarkerBase_DisplayNavBearing"),
            ("chk_displayradius", "GMapMarkerBase_DisplayRadius"),
            ("chk_displaytarget", "GMapMarkerBase_DisplayTarget"),
        ] {
            let value = get_bool(settings, key, true);
            if self.checks.insert(name, value) != Some(value) {
                settings.set(key, bool_text(value));
            }
        }
        // C#: ConfigPlanner.cs:244
        let tooltip = settings
            .get("mapicondesc")
            .is_some_and(|text| !text.is_empty());
        self.checks.insert("chk_displaytooltip", tooltip);
        // C#: ConfigPlanner.cs:245, 1137-1141
        let length = f64::from(get_int(settings, "GMapMarkerBase_Length", 500));
        if self.numbers[2].set(length) {
            self.number_changed(2, settings);
        }
        // C#: ConfigPlanner.cs:47-50 (the constructor)
        let style = settings
            .get("GMapMarkerBase_InactiveDisplayStyle")
            .unwrap_or("Normal")
            .to_owned();
        self.selected.insert("cmb_secondarydisplaystyle", Some(0));
        self.select_text("cmb_secondarydisplaystyle", &style);
        // C#: ConfigPlanner.cs:247-254; Program.cs:323-327 - the mode is the setting's, or
        // GMap's default.
        let mode = settings
            .get("mapCache")
            .unwrap_or("ServerAndCache")
            .to_owned();
        self.selected.insert(
            "CMB_mapCache",
            ACCESS_MODES.iter().position(|item| *item == mode),
        );
    }

    /// Leaving the page. `ConfigPlanner` is `IActivate` only; its `InputBox`es are modal, so the
    /// page cannot be left with one open in the C#, and here leaving cancels it.
    pub fn deactivate(&mut self) {
        self.active = false;
        self.open = None;
        self.prompt = None;
        self.queued.clear();
        self.messages.clear();
        self.joystick = false;
        self.focused = None;
    }

    /// A click on a check box: the box toggled and its `CheckedChanged` handler run. A dimmed box
    /// does nothing.
    pub fn click(&mut self, name: &'static str, settings: &mut Persisted) {
        let Some(spec) = CHECKS.iter().find(|spec| spec.name == name) else {
            return;
        };
        if spec.dim.is_some() || !self.shown(name) || self.blocked() {
            return;
        }
        let checked = !self.checked(name);
        self.checks.insert(name, checked);
        self.checked_changed(name, checked, settings);
    }

    /// Whether a modal box is showing, which takes every click on the page.
    fn blocked(&self) -> bool {
        self.prompt.is_some() || !self.messages.is_empty() || self.joystick
    }

    /// A box's `CheckedChanged`.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:381-1135`
    fn checked_changed(&mut self, name: &'static str, checked: bool, settings: &mut Persisted) {
        let key = CHECKS
            .iter()
            .find(|spec| spec.name == name)
            .and_then(|spec| spec.key);
        match name {
            // C#: ConfigPlanner.cs:381-410 - the speech boxes show and hide with it.
            // C#: ConfigPlanner.cs:1074-1078; 647-655; 693-696; 773-776; 918-922; 966;
            // 1013-1016; 1063-1071; 1079-1107
            "CHK_enablespeech"
            | "CHK_speechArmedOnly"
            | "CHK_resetapmonconnect"
            | "CHK_rtsresetesp32"
            | "CHK_disttohomeflightdata"
            | "CHK_showairports"
            | "chk_tfr"
            | "chk_norcreceiver"
            | "CHK_params_bg"
            | "chk_slowMachine"
            | "chk_displaycog"
            | "chk_displayheading"
            | "chk_displaynavbearing"
            | "chk_displayradius"
            | "chk_displaytarget" => {
                if let Some(key) = key {
                    settings.set(key, bool_text(checked));
                }
            }
            // `FlightData.myhud.hudon = CHK_hudshow.Checked`, saved as `CHK_hudshow`: the HUD
            // draws its instruments over the camera's picture, or the picture alone.
            // C#: ConfigPlanner.cs:375-379; ExtLibs/Controls/HUD.cs:2005-2008
            "CHK_hudshow" => {
                self.hudon = checked;
                settings.set("CHK_hudshow", bool_text(checked));
            }
            // C#: ConfigPlanner.cs:694-697; MainV2.cs:1752-1761
            "CHK_loadwponconnect" => {
                settings.set("loadwpsonconnect", bool_text(checked));
                self.effects.push(Effect::ReadMissionOnConnect(checked));
            }
            // C#: ConfigPlanner.cs:443-459, 461-477, 479-495, 519-551, 661-687, 812-834, 878-917
            "CHK_speechwaypoint"
            | "CHK_speechmode"
            | "CHK_speechcustom"
            | "CHK_speechbattery"
            | "CHK_speechaltwarning"
            | "CHK_speecharmdisarm"
            | "CHK_speechlowspeed" => {
                if let Some(key) = key {
                    settings.set(key, bool_text(checked));
                }
                if checked {
                    self.ask(steps(name), settings);
                }
            }
            // C#: ConfigPlanner.cs:1110-1135
            "chk_displaytooltip" => {
                if checked {
                    self.ask(steps(name), settings);
                } else {
                    settings.set("mapicondesc", "");
                }
            }
            // C#: ConfigPlanner.cs:756-766 - and the map's bearing put back to 0, which a map
            // that does not rotate is already at.
            "CHK_maprotation" => {
                settings.set("CHK_maprotation", bool_text(checked));
                if checked && self.checked("chk_shownofly") {
                    self.checks.insert("chk_shownofly", false);
                    self.checked_changed("chk_shownofly", false, settings);
                }
            }
            // C#: ConfigPlanner.cs:1041-1048
            "chk_shownofly" => {
                settings.set("ShowNoFly", bool_text(checked));
                if checked && self.checked("CHK_maprotation") {
                    self.checks.insert("CHK_maprotation", false);
                    self.checked_changed("CHK_maprotation", false, settings);
                }
            }
            _ => {}
        }
    }

    /// Starts a handler's `InputBox`es.
    fn ask(&mut self, steps: &[Step], settings: &Persisted) {
        self.queued = steps.iter().copied().collect();
        self.next_prompt(settings);
    }

    /// Shows the next `InputBox`, its box holding the key's value or the handler's literal.
    fn next_prompt(&mut self, settings: &Persisted) {
        self.prompt = self.queued.pop_front().map(|step| {
            let mut field = TextField::new("");
            field.set(settings.get(step.key).unwrap_or(step.default));
            Prompt { step, field }
        });
    }

    /// OK on the `InputBox`: the answer kept as `InputBox` keeps it, then written by the handler,
    /// and the handler's next box shown. The box keeps it before the handler parses it, so an
    /// altitude that is not a number is kept too.
    /// `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184; GCSViews/ConfigurationView/ConfigPlanner.cs:681-684`
    pub fn answer(&mut self, settings: &mut Persisted) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let answer = prompt.field.value().to_owned();
        super::optional::remember_answer(
            settings,
            prompt.step.title,
            prompt.step.question,
            &answer,
        );
        match prompt.step.store {
            Store::Text => settings.set(prompt.step.key, answer),
            // C#: ConfigPlanner.cs:684 - saved in metres.
            Store::AltHeight => {
                let Ok(value) = answer.trim().parse::<f64>() else {
                    self.queued.clear();
                    self.messages.push_back(Message {
                        title: UNHANDLED.0,
                        text: format!(
                            "{}System.FormatException: Input string was not in a correct format.",
                            UNHANDLED.1
                        ),
                    });
                    return;
                };
                let metres = value / f64::from(self.units.alt);
                settings.set(prompt.step.key, mp_log::netfmt::double(metres));
            }
            // C#: ConfigPlanner.cs:1127-1128
            Store::IconDescription => {
                settings.set("mapicondesc", answer.clone());
                settings.set("mapicondesc_default", answer);
            }
        }
        self.next_prompt(settings);
    }

    /// Cancel on the `InputBox`: the handler returns, leaving what it wrote and the box ticked.
    pub fn cancel(&mut self) {
        self.prompt = None;
        self.queued.clear();
    }

    /// A key in the `InputBox`: Enter is OK, Escape is Cancel.
    pub fn prompt_key(&mut self, event: &KeyDownEvent, settings: &mut Persisted) -> bool {
        let Some(prompt) = self.prompt.as_mut() else {
            return false;
        };
        match prompt.field.key(event) {
            KeyOutcome::Submitted => self.answer(settings),
            KeyOutcome::Cancelled => self.cancel(),
            KeyOutcome::Changed => {}
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// OK on the message box.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// A click on a combo: its list dropped down, or put away. Video Device's `Click` binds its
    /// list first (`ConfigPlanner.cs:742-751`).
    pub fn toggle_dropdown(&mut self, name: &'static str, settings: &Persisted) {
        let live = COMBOS
            .iter()
            .any(|spec| spec.name == name && spec.dim.is_none());
        if !live || self.blocked() {
            return;
        }
        if name == "CMB_videosources" {
            self.video_click(settings);
        }
        self.open = if self.open == Some(name) {
            None
        } else {
            Some(name)
        };
        if self.open == Some(name)
            && let Some(list) = self.video.list_mut(name)
        {
            list.open_list();
        }
    }

    /// The wheel over a video combo's list.
    pub fn scroll_list(&mut self, name: &'static str, lines: i32) {
        if let Some(list) = self.video.list_mut(name) {
            list.scroll_list(lines);
        }
    }

    /// An item chosen from a combo's list: its `SelectedIndexChanged`, when the selection changed.
    pub fn choose(&mut self, name: &'static str, index: usize, settings: &mut Persisted) {
        self.open = None;
        let Some(spec) = COMBOS
            .iter()
            .find(|spec| spec.name == name && spec.dim.is_none())
        else {
            return;
        };
        if let Some(list) = self.video.list_mut(name) {
            let key = i64::try_from(index).ok();
            if index >= list.options.len() || list.selected == key {
                return;
            }
            list.selected = key;
            // C#: ConfigPlanner.cs:297-361 - Video Format has no handler.
            if name == "CMB_videosources" {
                self.video_device_changed(settings);
            }
            return;
        }
        let Some(text) = spec.items.get(index).copied() else {
            return;
        };
        if self.selected.insert(name, Some(index)) == Some(Some(index)) {
            return;
        }
        match name {
            // C#: ConfigPlanner.cs:1019-1034
            "CMB_Layout" => self.layout_changed(index, settings),
            // C#: ConfigPlanner.cs:412-415
            "CMB_severity" => settings.set("severity", index.to_string()),
            // C#: ConfigPlanner.cs:558-572, 1050-1056
            "CMB_distunits" | "CMB_speedunits" | "CMB_altunits" => {
                let key = match name {
                    "CMB_distunits" => "distunits",
                    "CMB_speedunits" => "speedunits",
                    _ => "altunits",
                };
                settings.set(key, text);
                self.change_units(settings);
            }
            // C#: ConfigPlanner.cs:1143-1160
            "cmb_secondarydisplaystyle" => {
                settings.set("GMapMarkerBase_InactiveDisplayStyle", text);
            }
            // C#: ConfigPlanner.cs:1162-1168
            "CMB_mapCache" => {
                settings.set("mapCache", text);
                self.effects.push(Effect::MapAccess);
            }
            // C#: ConfigPlanner.cs:574-641
            _ => {
                let Some((rate, held)) = RATES
                    .iter()
                    .zip(self.rates.iter_mut())
                    .find(|(rate, _)| rate.combo == name)
                else {
                    return;
                };
                settings.set(name, text);
                let Ok(hz) = text.parse::<i32>() else {
                    return;
                };
                *held = hz;
                for stream in rate.streams {
                    self.effects.push(Effect::Stream(*stream, hz));
                }
            }
        }
    }

    /// `CMB_Layout_SelectedIndexChanged`: the preset of the name chosen made the view - which the
    /// setter saves as `displayview` - and saved again.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:1019-1034; MainV2.cs:357-366`
    fn layout_changed(&mut self, index: usize, settings: &mut Persisted) {
        let Some(name) = DisplayName::ALL.get(index).copied() else {
            return;
        };
        let custom = crate::display_view::custom_path(settings);
        crate::display_view::set(DisplayView::named(name, custom.as_deref()), settings);
    }

    /// Whether `label5` and the Layout box are drawn.
    #[must_use]
    pub const fn layout_shown(&self) -> bool {
        !self.layout_hidden
    }

    /// A number box's arrow.
    pub fn step(&mut self, index: usize, up: bool, settings: &mut Persisted) {
        if self.blocked() {
            return;
        }
        if let Some(number) = self.numbers.get_mut(index)
            && number.step(up)
        {
            self.number_changed(index, settings);
        }
    }

    /// A key in a number box: the arrows step, Enter validates, anything else is typing.
    pub fn number_key(
        &mut self,
        index: usize,
        event: &KeyDownEvent,
        settings: &mut Persisted,
    ) -> bool {
        match event.keystroke.key.as_str() {
            "up" | "down" => {
                self.step(index, event.keystroke.key == "up", settings);
                return true;
            }
            _ => {}
        }
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        match number.field.key(event) {
            KeyOutcome::Changed => {
                number.edited = true;
                true
            }
            KeyOutcome::Submitted => {
                if number.commit() {
                    self.number_changed(index, settings);
                }
                true
            }
            KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
        }
    }

    /// Once a frame: a number box the focus has left is validated.
    pub fn tick(&mut self, focused: Option<usize>, settings: &mut Persisted) {
        if let Some(left) = self.focused.filter(|left| Some(*left) != focused)
            && self.numbers.get_mut(left).is_some_and(Number::commit)
        {
            self.number_changed(left, settings);
        }
        self.focused = focused;
    }

    /// A number box's `ValueChanged`.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:689-692, 1058-1062, 1137-1141`
    fn number_changed(&mut self, index: usize, settings: &mut Persisted) {
        let Some(value) = self.numbers.get(index).map(Number::value) else {
            return;
        };
        match index {
            0 => settings.set("NUM_tracklength", decimal_text(value)),
            1 => {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                // `(byte)num_gcsid.Value`, within 1..255
                let id = value as u8;
                self.gcssysid = id;
                settings.set("gcsid", decimal_text(value));
            }
            _ => settings.set("GMapMarkerBase_length", decimal_text(value)),
        }
    }

    /// A key in the Log Path box: its `TextChanged` on every change.
    pub fn log_dir_key(&mut self, event: &KeyDownEvent, settings: &mut Persisted) -> bool {
        match self.log_dir.key(event) {
            KeyOutcome::Changed => {
                self.log_dir_changed(settings);
                true
            }
            KeyOutcome::Submitted | KeyOutcome::Cancelled | KeyOutcome::Ignored => false,
        }
    }

    /// The folder the Browse dialog returned, put in the box.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:779-786`
    pub fn browsed(&mut self, folder: &Path, settings: &mut Persisted) {
        self.log_dir.set(folder.display().to_string());
        self.log_dir_changed(settings);
    }

    /// `OnLogDirTextChanged`: `LogDir` set to the text when it names a directory that exists.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:788-795`
    fn log_dir_changed(&self, settings: &mut Persisted) {
        let path = self.log_dir.value();
        if !path.is_empty() && Path::new(path).os_is_dir() {
            settings.set("logdirectory", path);
        }
    }

    /// A click on a button. A dimmed one does nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:553-556, 779-786, 1170-1193`
    pub fn press(&mut self, name: &'static str, map_cache: &Path) {
        let live = BUTTONS
            .iter()
            .any(|(held, _, _, dim)| *held == name && dim.is_none());
        if !live || self.blocked() {
            return;
        }
        match name {
            // C#: ConfigPlanner.cs:262-286 - disabled while a capture runs; else stop first,
            // then the device and format chosen opened. The settings are written once it has
            // started, by `run_video`.
            "BUT_videostart" if !self.video.running => {
                self.effects.push(Effect::VideoStop);
                match (self.video.device(), self.video.mode()) {
                    (Some(device), Some(mode)) => self.effects.push(Effect::VideoStart {
                        device: device.clone(),
                        mode: *mode,
                    }),
                    // The C# reads `bmp.Media` of no selected item and its `catch` shows the
                    // NullReferenceException's words; these say what is missing.
                    _ => {
                        let why = "Camera Fail: no Video Device and Video Format chosen";
                        self.video.error = Some(why.to_owned());
                        self.effects.push(Effect::Status(why.to_owned()));
                    }
                }
            }
            // C#: ConfigPlanner.cs:288-295
            "BUT_videostop" => {
                self.video.running = false;
                self.effects.push(Effect::VideoStop);
            }
            "BUT_Joystick" => self.joystick = true,
            "BUT_logdirbrowse" => self.effects.push(Effect::BrowseLogDirectory),
            "BUT_mapCacheDir" => {
                if map_cache.os_is_dir() {
                    self.effects
                        .push(Effect::OpenDirectory(map_cache.to_path_buf()));
                } else {
                    self.messages.push_back(Message {
                        title: "",
                        text: format!("{} Directory does not exist!", map_cache.display()),
                    });
                }
            }
            _ => {}
        }
    }

    /// Closes the joystick page's window.
    pub fn close_joystick(&mut self) {
        self.joystick = false;
    }
}

/// What Start and Stop ask of the application, which holds the capture as `MainV2` holds `cam`.
/// Stop drops it: `Dispose`, which here stops the reading thread and closes the device. Start
/// opens the device in the format and starts it; once it runs, the two settings are written as
/// the C# writes them - `video_device` and `video_options`, the two combos' `SelectedIndex` - and
/// Start is disabled. A refusal is "Camera Fail: " and the reason, for the status line (the
/// owner's ruling of 2026-09-25: no message box for an error the window shows as state). Returns
/// the status line's words, if any.
///
/// Divergence: `mp_video` over V4L2 where the C# builds a DirectShow graph.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:262-295; ExtLibs/WebCamService/Capture.cs:
/// 77-111, 142-157, 220-231`
pub fn run_video(
    planner: &mut Planner,
    settings: &mut Persisted,
    capture: &mut Option<mp_video::Capture>,
    effect: &Effect,
) -> Option<String> {
    match effect {
        Effect::VideoStop => {
            *capture = None;
            planner.video.running = false;
            None
        }
        Effect::VideoStart { device, mode } => {
            let started = planner.video.source.clone().map_or_else(
                || Err("no video capture on this platform".to_owned()),
                |source| {
                    mp_video::Capture::start(source.as_ref(), device, mode)
                        .map_err(|why| why.to_string())
                },
            );
            match started {
                Ok(started) => {
                    *capture = Some(started);
                    // `SelectedIndex`, -1 for none.
                    let index = |list: &Combo| list.selected.unwrap_or(-1).to_string();
                    settings.set("video_device", index(&planner.video.device_list));
                    settings.set("video_options", index(&planner.video.mode_list));
                    planner.video.running = true;
                    planner.video.frames = 0;
                    planner.video.error = None;
                    None
                }
                Err(why) => {
                    let words = format!("Camera Fail: {why}");
                    planner.video.error = Some(words.clone());
                    Some(words)
                }
            }
        }
        Effect::Status(words) => Some(words.clone()),
        _ => None,
    }
}

/// While a capture runs the window is drawn every 40 ms: `Capture`'s timer hands the HUD a
/// picture 25 times a second (`Thread.Sleep(1000/25)`), faster than the 10 Hz the rest of the
/// screen is drawn at.
/// `// C#: ExtLibs/WebCamService/Capture.cs:114-140`
const VIDEO_REPAINT: Duration = Duration::from_millis(40);

/// The `hud.camera` fact: the size of the picture under the HUD, `<width>x<height>`, or `none`.
fn camera_fact(frame: Option<&mp_video::Frame>) -> String {
    frame.map_or_else(
        || "none".to_owned(),
        |frame| format!("{}x{}", frame.width, frame.height),
    )
}

/// The picture the HUD draws for a frame: a gpui image, in the BGRA gpui keeps its images in.
fn camera_image(frame: &mp_video::Frame) -> Option<Arc<gpui::RenderImage>> {
    image::RgbaImage::from_raw(frame.width, frame.height, frame.to_bgra())
        .map(|buffer| Arc::new(gpui::RenderImage::new(vec![image::Frame::new(buffer)])))
}

/// Whether the map reads tiles from its cache only: `mapCache` is `CacheOnly`. The map's tile
/// store has no mode that fetches without caching, so `ServerOnly` is `ServerAndCache` here.
/// `// C#: Program.cs:323-327; ConfigPlanner.cs:1162-1168`
#[must_use]
pub fn cache_only(settings: &Persisted) -> bool {
    settings.get("mapCache") == Some("CacheOnly")
}

/// Whether the mission is read when a vehicle connects: `loadwpsonconnect`.
/// `// C#: MainV2.cs:1752-1761`
#[must_use]
pub fn load_wps_on_connect(settings: &Persisted) -> bool {
    get_bool(settings, "loadwpsonconnect", false)
}

/// The focus handles of the Log Path box, the three number boxes and the `InputBox`.
pub struct Focus {
    log_dir: FocusHandle,
    numbers: [FocusHandle; 3],
    prompt: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            log_dir: cx.focus_handle(),
            numbers: [cx.focus_handle(), cx.focus_handle(), cx.focus_handle()],
            prompt: cx.focus_handle(),
        }
    }

    /// The number box with the focus.
    #[must_use]
    pub fn number(&self, window: &Window) -> Option<usize> {
        self.numbers
            .iter()
            .position(|handle| handle.is_focused(window))
    }
}

// -------------------------------------------------------------------------------------------------
// The page's part in the application.
// -------------------------------------------------------------------------------------------------

impl MissionPlanner {
    /// Runs one of the page's handlers over `Settings.Instance` and does what it asked for that
    /// needs no window; returns the rest. What the handler writes is in the dictionary at once and
    /// in `config.xml` at the next `SaveConfig`, as the C#'s handlers leave it.
    /// `// C#: ExtLibs/Utilities/Settings.cs:58-61; MainV2.cs:2226-2244`
    fn planner_apply(&mut self, handler: impl FnOnce(&mut Planner, &mut Persisted)) -> Vec<Effect> {
        handler(&mut self.planner, &mut self.persisted);
        let mut rest = Vec::new();
        for effect in self.planner.take_effects() {
            match effect {
                Effect::Stream(stream, hz) => {
                    if request_datastream(&self.telemetry, stream, hz) > 0 {
                        self.planner.sent(stream, hz);
                    }
                }
                Effect::ReadMissionOnConnect(on) => {
                    self.auto_read_mission = on;
                    // On connecting, not now: a vehicle already connected has connected.
                    if on && self.telemetry.view().vehicle.is_some() {
                        self.mission_requested = true;
                    }
                }
                Effect::MapAccess => {
                    if let Some(source) = self
                        .tile_source_id()
                        .and_then(mp_tiles::source::source_by_id)
                    {
                        self.set_tile_source(source);
                    }
                }
                other => rest.push(other),
            }
        }
        rest
    }

    /// Runs one of the page's handlers and does everything it asked for.
    fn planner_run(
        &mut self,
        cx: &mut Context<Self>,
        handler: impl FnOnce(&mut Planner, &mut Persisted),
    ) {
        let rest = self.planner_apply(handler);
        self.planner_window_effects(rest, cx);
    }

    /// What a handler asked for that needs the window: the folder dialog and the file manager.
    fn planner_window_effects(&mut self, effects: Vec<Effect>, cx: &mut Context<Self>) {
        for effect in effects {
            match effect {
                Effect::OpenDirectory(path) => cx.open_with_system(&path),
                Effect::BrowseLogDirectory => {
                    let chosen = cx.prompt_for_paths(gpui::PathPromptOptions {
                        files: false,
                        directories: true,
                        multiple: false,
                        prompt: None,
                    });
                    cx.spawn(async move |this, cx| {
                        let Ok(Ok(Some(paths))) = chosen.await else {
                            return;
                        };
                        let Some(folder) = paths.into_iter().next() else {
                            return;
                        };
                        let _ = this.update(cx, |this, cx| {
                            let rest = this.planner_apply(|planner, settings| {
                                planner.browsed(&folder, settings);
                            });
                            this.planner_window_effects(rest, cx);
                            cx.notify();
                        });
                    })
                    .detach();
                }
                effect @ (Effect::VideoStart { .. } | Effect::VideoStop | Effect::Status(_)) => {
                    let words = run_video(
                        &mut self.planner,
                        &mut self.persisted,
                        &mut self.video,
                        &effect,
                    );
                    if let Some(words) = words {
                        self.file_status = Some(words);
                    }
                    if matches!(effect, Effect::Status(_)) {
                        continue;
                    }
                    self.video_repaint_keep(cx);
                }
                Effect::Stream(..) | Effect::ReadMissionOnConnect(_) | Effect::MapAccess => {}
            }
        }
    }

    /// Once a frame: the capture's latest frame made the HUD's picture - once per new frame, the
    /// last one's image taken out of the window's atlas - and what the capture says handed to the
    /// page. With no capture, no picture: `Dispose` clears the HUD's (`camimage(null)`).
    /// `// C#: ExtLibs/WebCamService/Capture.cs:142-157; GCSViews/FlightData.cs:1907-1910`
    pub(crate) fn video_tick(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.planner.video_status(self.video.as_ref());
        // The HUD menu's sources - GStreamer, MJPEG - set the same picture, and a runtime being
        // downloaded for GStreamer says how it goes. `// C#: MainV2.cs:3428-3493`
        self.hud_video_tick();
        let latest = self
            .fly_data
            .hud_video_latest()
            .or_else(|| self.video.as_ref().and_then(mp_video::Capture::latest));
        let shown = match (&latest, &self.video_frame) {
            (Some(new), Some((old, _))) => Arc::ptr_eq(new, old),
            (None, None) => true,
            _ => false,
        };
        if !shown {
            let made = latest.and_then(|frame| camera_image(&frame).map(|image| (frame, image)));
            self.fly_data.camera = made.as_ref().map(|(_, image)| Arc::clone(image));
            if let Some((_, old)) = std::mem::replace(&mut self.video_frame, made) {
                cx.drop_image(old, Some(window));
            }
        }
        crate::facts::record(
            "hud.camera",
            camera_fact(self.video_frame.as_ref().map(|(frame, _)| frame.as_ref())),
        );
        // `hudon`, and whether the HUD paints its instruments with it: over the picture only
        // while it is true, and always with no picture. `// C#: ExtLibs/Controls/HUD.cs:1988-2013`
        crate::facts::record("hud.hudon", self.planner.hud_on());
        crate::facts::record(
            "hud.instruments",
            crate::hud::draws_instruments(self.fly_data.camera.is_some(), self.planner.hud_on()),
        );
        // A GStreamer pipeline that ended by itself stops the repaints.
        self.video_repaint_keep(cx);
    }

    /// The repaints at the camera's rate: kept while the Planner page's capture or a HUD menu
    /// source runs, started when one starts, dropped - which ends them - when none does.
    pub(crate) fn video_repaint_keep(&mut self, cx: &mut Context<Self>) {
        if self.video.is_none() && !self.fly_data.hud_video_running() {
            self.video_repaint = None;
            return;
        }
        if self.video_repaint.is_some() {
            return;
        }
        self.video_repaint = Some(cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor().timer(VIDEO_REPAINT).await;
                if this.update(cx, |_, cx| cx.notify()).is_err() {
                    break;
                }
            }
        }));
    }

    /// `Activate`, when the page is chosen.
    pub(crate) fn planner_activate(&mut self) {
        let default_log_dir = mp_settings::default_log_directory();
        // `Activate` asks for nothing that needs the window.
        let _ = self.planner_apply(|planner, settings| {
            planner.activate(settings, default_log_dir.as_deref());
        });
    }

    /// Once a frame: the number box the focus left is validated.
    pub(crate) fn planner_tick(&mut self, window: &Window) {
        let focused = self.planner_focus.number(window);
        // Only when the focus has moved: otherwise `tick` has nothing to do.
        if !self.planner.is_active() || self.planner.focused == focused {
            return;
        }
        let _ = self.planner_apply(|planner, settings| planner.tick(focused, settings));
    }

    /// The page, laid out as `ConfigPlanner.resx` lays it out.
    pub(crate) fn planner_page(
        &self,
        _view: &TelemetryView,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        let planner = &self.planner;
        let focus = &self.planner_focus;
        let mut body = div().relative().w(px(PAGE.0)).h(px(PAGE.1));
        for (x, y, width, height, text) in LABELS {
            if *text == "Layout" && !planner.layout_shown() {
                continue;
            }
            body = body.child(
                at(*x, *y, *width, *height)
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(*text),
            );
        }
        for spec in CHECKS {
            if planner.shown(spec.name) {
                body = body.child(check_box(planner, spec, cx));
            }
        }
        for spec in COMBOS {
            if spec.name == "CMB_Layout" && !planner.layout_shown() {
                continue;
            }
            body = body.child(combo_box(planner, spec, cx));
        }
        for spec in BUTTONS {
            // Start is disabled while a capture runs. C#: ConfigPlanner.cs:137-146, 281
            let disabled = spec.0 == "BUT_videostart" && planner.video_running();
            body = body.child(button(spec, planner.blocked() || disabled, cx));
        }
        for (index, ((spec, number), handle)) in NUMBERS
            .iter()
            .zip(&planner.numbers)
            .zip(&focus.numbers)
            .enumerate()
        {
            body = body.child(number_box(index, spec, number, handle, window, cx));
        }
        body = body.child(log_dir_box(planner, focus, window, cx));
        if let Some(name) = planner.dropdown()
            && let Some(spec) = COMBOS.iter().find(|spec| spec.name == name)
        {
            body = body.child(dropdown(planner, spec, cx));
        }
        if let Some(dialog) = dialog(planner, focus, window, cx) {
            body = body.child(dialog);
        } else if planner.joystick_open() {
            body = body.child(joystick_window(
                &self.sticks,
                &self.joystick_focus,
                window,
                cx,
            ));
        }
        body.into_any_element()
    }
}

// -------------------------------------------------------------------------------------------------
// Drawing.
// -------------------------------------------------------------------------------------------------

/// An absolutely placed box, at a `.resx` `Location` and `Size`.
fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// The same, from the specs' whole-pixel geometry.
fn at_spec((x, y, width, height): (u16, u16, u16, u16)) -> Div {
    at(
        f32::from(x),
        f32::from(y),
        f32::from(width),
        f32::from(height),
    )
}

/// The id of a control: `planner-` and its Designer name.
fn control_id(name: &str) -> String {
    format!("planner-{name}")
}

/// A check box and its text. A dimmed one is drawn inert.
fn check_box(planner: &Planner, spec: &CheckSpec, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let live = spec.dim.is_none();
    let name = spec.name;
    let mark = planner.checked(name).then(|| {
        div()
            .size(px(6.0))
            .bg(rgb(if live { theme::ACCENT } else { theme::DIM }))
    });
    let id = control_id(name);
    let base = crate::probe::measured(id.clone(), at_spec(spec.at))
        .id(SharedString::from(id))
        .flex()
        .items_center()
        .gap_1()
        .child(
            div()
                .flex_shrink_0()
                .size(px(12.0))
                .flex()
                .items_center()
                .justify_center()
                .border_1()
                .border_color(rgb(theme::DIM))
                .bg(rgb(if live { theme::BG } else { theme::PANEL }))
                .children(mark),
        )
        .child(
            div()
                .whitespace_nowrap()
                .text_xs()
                .text_color(rgb(if live { theme::TEXT } else { theme::DIM }))
                .child(spec.text),
        );
    if live {
        base.cursor_pointer()
            .on_click(cx.listener(move |this, _event, window, cx| {
                this.planner_run(cx, |planner, settings| planner.click(name, settings));
                if this.planner.prompt().is_some() {
                    this.planner_focus.prompt.focus(window, cx);
                }
                cx.notify();
            }))
            .into_any_element()
    } else {
        base.into_any_element()
    }
}

/// A combo box: the selected item's text, and a click to drop its list down.
fn combo_box(planner: &Planner, spec: &ComboSpec, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let live = spec.dim.is_none();
    let name = spec.name;
    let id = control_id(name);
    let base = crate::probe::measured(id.clone(), at_spec(spec.at))
        .id(SharedString::from(id))
        .flex()
        .items_center()
        .justify_between()
        .px_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if planner.dropdown() == Some(name) {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if live { theme::ACTION } else { theme::PANEL }))
        .overflow_hidden()
        .whitespace_nowrap()
        .text_xs()
        .text_color(rgb(if live { theme::TEXT } else { theme::DIM }))
        .child(div().overflow_hidden().child(planner.combo_text(name)))
        .child(div().flex_shrink_0().text_size(px(7.0)).child("▼"));
    if live {
        base.cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.planner.toggle_dropdown(name, &this.persisted);
                cx.notify();
            }))
            .into_any_element()
    } else {
        base.into_any_element()
    }
}

/// A combo box's list, dropped down over whatever is under it as a WinForms list is.
fn dropdown(planner: &Planner, spec: &ComboSpec, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (x, y, width, height) = spec.at;
    let name = spec.name;
    // The video lists are the devices' and the formats': as long as a camera makes them, so
    // drawn over the page, thirty rows at a time, each row `planner-<combo>-<index>`.
    if let Some(list) = planner.video.list(name) {
        return super::servo_output::dropdown(
            &control_id(name),
            list,
            (
                f32::from(x),
                f32::from(y) + f32::from(height),
                f32::from(width),
            ),
            move |this, key| {
                let Ok(index) = usize::try_from(key) else {
                    return;
                };
                // Choosing a device or a format asks for nothing that needs the window.
                let _ = this.planner_apply(|planner, settings| {
                    planner.choose(name, index, settings);
                });
            },
            move |this, lines| this.planner.scroll_list(name, lines),
            cx,
        );
    }
    let selected = planner.selected.get(name).copied().flatten();
    let mut list = div()
        .absolute()
        .left(px(f32::from(x)))
        .top(px(f32::from(y) + f32::from(height)))
        .min_w(px(f32::from(width)))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .occlude();
    for (index, text) in spec.items.iter().enumerate() {
        let id = format!("planner-{name}-{text}");
        list = list.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .px_1()
                .py(px(1.0))
                .whitespace_nowrap()
                .text_xs()
                .bg(rgb(if selected == Some(index) {
                    theme::BORDER
                } else {
                    theme::PANEL
                }))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::ACTION)))
                .child(*text)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.planner_run(cx, |planner, settings| {
                        planner.choose(name, index, settings);
                    });
                    cx.notify();
                })),
        );
    }
    list.into_any_element()
}

/// A button at its place. A dimmed one, or any while a box is modal, is drawn inert.
fn button(spec: &ButtonSpec, blocked: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (name, geometry, text, dim) = *spec;
    let live = dim.is_none();
    let id = control_id(name);
    let base = crate::probe::measured(id.clone(), at_spec(geometry))
        .id(SharedString::from(id))
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .whitespace_nowrap()
        .text_xs();
    if live && !blocked {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::ACCENT))
            .text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(text)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                let cache = mp_tiles::TileCache::default_root();
                this.planner_run(cx, |planner, _settings| planner.press(name, &cache));
                cx.notify();
            }))
            .into_any_element()
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .child(text)
            .into_any_element()
    }
}

/// A number box: its text, typed into while it has the focus, and the up and down arrows.
fn number_box(
    index: usize,
    spec: &NumberSpec,
    number: &Number,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (name, geometry, _) = *spec;
    let focused = handle.is_focused(window);
    let id = control_id(name);
    let text = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        // A line that fits the Designer's 20-high box inside its border: the default line height
        // made the text 19 high in 18 (the layout guard on the owner's Mac, 2026-10-04).
        .line_height(px(16.0))
        .text_color(rgb(theme::TEXT))
        .track_focus(handle)
        .key_context("TextField")
        .cursor_text()
        .child(number.field.value().to_owned())
        .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))))
        .on_key_down(cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
            let mut used = false;
            this.planner_run(cx, |planner, settings| {
                used = planner.number_key(index, event, settings);
            });
            if used {
                cx.notify();
            }
        }));
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let id = format!("planner-{name}-{}", if up { "up" } else { "down" });
        crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex_1()
            .flex()
            .items_center()
            .justify_center()
            .text_size(px(7.0))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(if up { "▲" } else { "▼" })
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.planner_run(cx, |planner, settings| planner.step(index, up, settings));
                cx.notify();
            }))
    };
    at_spec(geometry)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::ACTION))
        .child(text)
        .child(
            div()
                .w(px(14.0))
                .h_full()
                .flex()
                .flex_col()
                .border_l_1()
                .border_color(rgb(theme::BORDER))
                .child(arrow(true, cx))
                .child(arrow(false, cx)),
        )
        .into_any_element()
}

/// The Log Path box.
fn log_dir_box(
    planner: &Planner,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (x, y, width, height) = LOG_DIR_AT;
    let focused = focus.log_dir.is_focused(window);
    at(x, y, width, height)
        .child(
            crate::probe::measured("planner-txt_log_dir", div())
                .id("planner-txt_log_dir")
                .size_full()
                .flex()
                .items_center()
                .px_1()
                .overflow_hidden()
                .whitespace_nowrap()
                .rounded_sm()
                .border_1()
                .border_color(rgb(if focused {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(theme::ACTION))
                .text_xs()
                .line_height(px(16.0))
                .text_color(rgb(theme::TEXT))
                .track_focus(&focus.log_dir)
                .key_context("TextField")
                .cursor_text()
                .child(planner.log_dir().to_owned())
                .children(focused.then(|| div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT))))
                .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    let mut used = false;
                    this.planner_run(cx, |planner, settings| {
                        used = planner.log_dir_key(event, settings);
                    });
                    if used {
                        cx.notify();
                    }
                })),
        )
        .into_any_element()
}

/// The message box or the `InputBox` showing, over the page: both are modal in the C#.
/// `// C#: ExtLibs/Controls/InputBox.cs:61-172`
fn dialog(
    planner: &Planner,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let card = |border: u32| {
        div()
            .flex()
            .flex_col()
            .gap_2()
            .w(px(396.0))
            .p_3()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(border))
            .rounded_md()
    };
    let dialog = if let Some(message) = planner.message() {
        crate::probe::measured("planner-message", card(theme::ALERT))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(message.title),
            )
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(message.text.clone()),
            )
            .child(div().flex().justify_end().child(crate::ui::action(
                "planner-message-ok",
                "OK",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.planner.dismiss_message();
                    cx.notify();
                }),
            )))
    } else {
        let prompt = planner.prompt()?;
        let focused = focus.prompt.is_focused(window);
        crate::probe::measured("planner-prompt", card(theme::ACCENT))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(prompt.step.title),
            )
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(prompt.step.question),
            )
            .child(crate::textfield::text_field(
                "planner-prompt-value",
                &prompt.field,
                &focus.prompt,
                focused,
                px(372.0),
                cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    let mut used = false;
                    this.planner_run(cx, |planner, settings| {
                        used = planner.prompt_key(event, settings);
                    });
                    if used {
                        cx.notify();
                    }
                }),
            ))
            .child(
                div()
                    .flex()
                    .justify_end()
                    .gap_2()
                    .child(crate::ui::action_sized(
                        "planner-prompt-ok",
                        "OK",
                        theme::ACCENT,
                        true,
                        Some(px(75.0)),
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.planner_run(cx, |planner, settings| planner.answer(settings));
                            cx.notify();
                        }),
                    ))
                    .child(crate::ui::action_sized(
                        "planner-prompt-cancel",
                        "Cancel",
                        theme::DIM,
                        true,
                        Some(px(75.0)),
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.planner.cancel();
                            cx.notify();
                        }),
                    )),
            )
    };
    Some(backdrop(
        "planner-dialog-backdrop",
        dialog.into_any_element(),
    ))
}

/// What a modal window sits on: the page, covered.
fn backdrop(id: &'static str, window: AnyElement) -> AnyElement {
    div()
        .id(id)
        .absolute()
        .inset_0()
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(window)
        .into_any_element()
}

/// `new JoystickSetup().ShowUserControl()`: the joystick page in a window of its own size - 702 by
/// 331, grown as the page grows its rows (`Ctl_SizeChanged`) - with no title and a close box.
/// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:553-556; Utilities/ExtensionsMP.cs:110-144;
/// Joystick/JoystickSetup.resx ($this.Size)`
fn joystick_window(
    sticks: &crate::joystick::Sticks,
    focus: &gpui::FocusHandle,
    viewport: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (width, height) = crate::joystick::page_size(sticks);
    let window = div()
        .w(px(width + 16.0))
        .min_h(px(height))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(div().flex().justify_end().p_1().child(crate::ui::action(
            "planner-joystick-close",
            "X",
            theme::DIM,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.planner.close_joystick();
                cx.notify();
            }),
        )))
        .child(
            div()
                .p_2()
                .child(crate::joystick::page(sticks, focus, viewport, cx)),
        );
    backdrop(
        "planner-joystick-backdrop",
        crate::probe::measured("planner-joystick", window).into_any_element(),
    )
}

// -------------------------------------------------------------------------------------------------
// Facts.
// -------------------------------------------------------------------------------------------------

/// Facts a UI test asserts on: each `Settings.Instance` key the page reads or writes, as the
/// dictionary holds it (`config.planner.<key>`, `none` when unset), each control as it shows
/// (`.check.<Name>` - `hidden` for a box not drawn -, `.combo.<Name>`, `.number.<Name>`,
/// `.logdir`), what the handlers changed at once (`.units.*`, `.rates`, `.streams`, `.cacheonly`,
/// `.readmissiononconnect`), and the box or window showing. Whether and when the dictionary was
/// saved is `settings::Persisted`'s to say (`config.saved`, `config.saves`, `config.error`).
pub fn record_facts(planner: &Planner, settings: &Persisted, read_mission_on_connect: bool) {
    for (key, value) in facts(planner, settings, read_mission_on_connect).0 {
        crate::facts::record(key, value);
    }
}

/// The facts gathered for [`record_facts`], key and value, in the order they are recorded.
#[derive(Default)]
struct Facts(Vec<(String, String)>);

impl Facts {
    fn record(&mut self, key: impl Into<String>, value: impl std::fmt::Display) {
        self.0.push((key.into(), value.to_string()));
    }
}

/// What [`record_facts`] publishes.
fn facts(planner: &Planner, settings: &Persisted, read_mission_on_connect: bool) -> Facts {
    let mut facts = Facts::default();
    facts.record("config.planner.active", planner.is_active());
    for key in KEYS {
        facts.record(
            format!("config.planner.{key}"),
            settings.get(key).unwrap_or("none"),
        );
    }
    for spec in CHECKS {
        let state = if !planner.shown(spec.name) {
            "hidden"
        } else if planner.checked(spec.name) {
            "true"
        } else {
            "false"
        };
        facts.record(format!("config.planner.check.{}", spec.name), state);
    }
    for spec in COMBOS {
        facts.record(
            format!("config.planner.combo.{}", spec.name),
            planner.combo_text(spec.name),
        );
        facts.record(
            format!("config.planner.combo.{}.items", spec.name),
            planner.combo_items(spec.name).join(","),
        );
    }
    for (name, ..) in NUMBERS {
        facts.record(
            format!("config.planner.number.{name}"),
            planner.number(name).map_or_else(String::new, decimal_text),
        );
    }
    facts.record("config.planner.logdir", planner.log_dir());
    facts.record("config.planner.dimmed", dimmed().join(","));
    facts.record(
        "config.planner.dropdown",
        planner.dropdown().unwrap_or("none"),
    );
    let units = planner.units();
    facts.record("config.planner.units.dist", units.dist_unit);
    facts.record("config.planner.units.alt", units.alt_unit);
    facts.record("config.planner.units.speed", units.speed_unit);
    facts.record("config.planner.units.dist.multiplier", units.dist);
    facts.record("config.planner.units.alt.multiplier", units.alt);
    facts.record("config.planner.units.speed.multiplier", units.speed);
    facts.record(
        "config.planner.rates",
        planner
            .rates()
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(","),
    );
    facts.record(
        "config.planner.streams",
        if planner.sent.is_empty() {
            "none".to_owned()
        } else {
            planner
                .sent
                .iter()
                .map(|(stream, hz)| format!("{stream}@{hz}"))
                .collect::<Vec<_>>()
                .join(",")
        },
    );
    facts.record("config.planner.gcssysid", planner.gcssysid());
    facts.record("config.planner.cacheonly", cache_only(settings));
    facts.record(
        "config.planner.readmissiononconnect",
        read_mission_on_connect,
    );
    facts.record(
        "config.planner.prompt",
        planner.prompt().map_or_else(
            || "none".to_owned(),
            |prompt| format!("{}: {}", prompt.step.title, prompt.step.question),
        ),
    );
    facts.record(
        "config.planner.prompt.text",
        planner
            .prompt()
            .map_or_else(String::new, |prompt| prompt.field.value().to_owned()),
    );
    facts.record(
        "config.planner.message",
        planner
            .message()
            .map_or_else(|| "none".to_owned(), |message| message.text.clone()),
    );
    facts.record("config.planner.joystick", planner.joystick_open());
    // The video controls: how many devices and formats are listed and which are chosen, whether
    // a capture runs, the frames it has decoded, and why a Start was refused or a read failed.
    let video = &planner.video;
    facts.record("config.planner.video.devices", video.devices.len());
    facts.record(
        "config.planner.video.device",
        planner.combo_text("CMB_videosources"),
    );
    facts.record("config.planner.video.modes", video.modes.len());
    facts.record(
        "config.planner.video.mode",
        planner.combo_text("CMB_videoresolutions"),
    );
    facts.record("config.planner.video.running", video.running);
    facts.record("config.planner.video.frames", video.frames);
    facts.record(
        "config.planner.video.error",
        video.error.as_deref().unwrap_or("none"),
    );
    facts.record(
        "config.planner.video.list.top",
        planner
            .dropdown()
            .and_then(|name| video.list(name))
            .map_or(0, |list| list.top_index),
    );
    facts
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use super::*;
    use crate::settings::SaveEvent;
    use crate::telemetry::scripted::{VEHICLE, Vehicle, until};
    use mp_link::ProtocolTimeouts;
    use mp_video::Source as _;
    use mp_video::testing::FakeSource;

    /// A file Mission Planner's `XmlTextWriter` wrote, from `mp-settings`'s fixtures.
    const CSHARP_FILE: &str = include_str!("../../../mp-settings/tests/fixtures/config.xml");

    /// A directory of its own for one test, removed afterwards.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = mp_os::temp_dir().join(format!(
                "mp-gui-config-planner-{name}-{}",
                mp_os::process_id()
            ));
            let _ = mp_os::fs::remove_dir_all(&path);
            mp_os::fs::create_dir_all(&path).expect("scratch directory");
            Self(path)
        }

        /// Where `config.xml` goes, under a data directory that does not exist yet.
        fn config(&self) -> PathBuf {
            self.0.join("MissionPlannerRust").join("config.xml")
        }

        fn seed(&self, text: &str) -> PathBuf {
            let path = self.config();
            mp_os::fs::create_dir_all(path.parent().expect("parent")).expect("data directory");
            mp_os::fs::write(&path, text).expect("seed config.xml");
            path
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = mp_os::fs::remove_dir_all(&self.0);
        }
    }

    fn key(name: &str, character: Option<&str>) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: name.to_owned(),
                key_char: character.map(ToOwned::to_owned),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    /// A page activated over these settings, with no log directory on disk.
    fn activated(settings: &mut Persisted) -> Planner {
        let mut planner = Planner::new(settings).with_video_source(Arc::new(FakeSource::empty()));
        planner.activate(settings, None);
        planner
    }

    /// The page over this machine's scripted webcam and a second camera, activated.
    fn with_cameras(settings: &mut Persisted, source: FakeSource) -> Planner {
        let mut planner = Planner::new(settings).with_video_source(Arc::new(source));
        planner.activate(settings, None);
        planner
    }

    fn two_cameras() -> FakeSource {
        FakeSource::webcam().with_device(
            "/dev/video4",
            "USB Capture (/dev/video4)",
            &[mp_video::Mode {
                format: mp_video::PixelFormat::Mjpeg,
                width: 1920,
                height: 1080,
                interval: (1, 60),
            }],
        )
    }

    /// Video Device lists the devices when it is clicked, which selects the first and lists its
    /// formats; choosing another lists that one's; Video Format has no handler.
    /// `// C#: ConfigPlanner.cs:297-361, 742-751, 969-990`
    #[test]
    fn clicking_video_device_lists_the_devices_and_the_first_ones_formats() {
        let mut settings = Persisted::at(None);
        let mut planner = with_cameras(&mut settings, two_cameras());
        // Nothing saved: Activate lists nothing.
        assert_eq!(
            planner.combo_items("CMB_videosources"),
            Vec::<String>::new()
        );
        planner.toggle_dropdown("CMB_videosources", &settings);
        assert_eq!(planner.dropdown(), Some("CMB_videosources"));
        assert_eq!(
            planner.combo_items("CMB_videosources"),
            [
                "Integrated_Webcam_HD: Integrate (/dev/video0)",
                "USB Capture (/dev/video4)"
            ]
        );
        assert_eq!(
            planner.combo_text("CMB_videosources"),
            "Integrated_Webcam_HD: Integrate (/dev/video0)"
        );
        assert_eq!(
            planner.combo_items("CMB_videoresolutions"),
            [
                "1280 x 720 30.00 fps MJPG",
                "640 x 480 30.00 fps YUYV",
                "320 x 240 15.00 fps YUYV"
            ]
        );
        assert_eq!(
            planner.combo_text("CMB_videoresolutions"),
            "1280 x 720 30.00 fps MJPG"
        );
        planner.choose("CMB_videosources", 1, &mut settings);
        assert_eq!(planner.dropdown(), None);
        assert_eq!(
            planner.combo_items("CMB_videoresolutions"),
            ["1920 x 1080 60.00 fps MJPG"]
        );
        planner.toggle_dropdown("CMB_videoresolutions", &settings);
        planner.choose("CMB_videoresolutions", 0, &mut settings);
        // Out of the list: nothing.
        planner.choose("CMB_videoresolutions", 5, &mut settings);
        assert_eq!(
            planner.combo_text("CMB_videoresolutions"),
            "1920 x 1080 60.00 fps MJPG"
        );
        // Choosing writes nothing: only a Start that starts does.
        assert_eq!(settings.get("video_device"), None);
        assert_eq!(settings.get("video_options"), None);
        assert!(planner.take_effects().is_empty());
        let facts = facts(&planner, &settings, false).0;
        let fact = |key: &str| {
            facts
                .iter()
                .find(|(held, _)| held == key)
                .map(|(_, value)| value.clone())
                .unwrap_or_else(|| panic!("{key}"))
        };
        assert_eq!(fact("config.planner.video.devices"), "2");
        assert_eq!(
            fact("config.planner.video.device"),
            "USB Capture (/dev/video4)"
        );
        assert_eq!(fact("config.planner.video.modes"), "1");
        assert_eq!(fact("config.planner.video.running"), "false");
        assert_eq!(fact("config.planner.video.error"), "none");
        assert_eq!(
            fact("config.planner.combo.CMB_videosources.items"),
            "Integrated_Webcam_HD: Integrate (/dev/video0),USB Capture (/dev/video4)"
        );
    }

    /// Start stops first, opens the device in the format, and once it runs writes the two
    /// indexes and disables itself; the frames arrive; Stop ends it and enables Start again.
    /// `// C#: ConfigPlanner.cs:262-295`
    #[test]
    fn start_captures_and_writes_the_two_indexes_and_stop_ends_it() {
        let mut settings = Persisted::at(None);
        let source = FakeSource::webcam();
        let mut planner = with_cameras(&mut settings, source.clone());
        planner.toggle_dropdown("CMB_videosources", &settings);
        planner.choose("CMB_videosources", 0, &mut settings);
        planner.choose("CMB_videoresolutions", 2, &mut settings);
        planner.press("BUT_videostart", Path::new("/"));
        let device = source.devices()[0].clone();
        let mode = source.modes(&device).unwrap()[2];
        let effects = planner.take_effects();
        assert_eq!(
            effects,
            [
                Effect::VideoStop,
                Effect::VideoStart {
                    device: device.clone(),
                    mode
                }
            ]
        );
        let mut capture = None;
        for effect in &effects {
            assert_eq!(
                run_video(&mut planner, &mut settings, &mut capture, effect),
                None
            );
        }
        let running = capture.as_ref().expect("a capture");
        assert_eq!(running.device(), &device);
        assert_eq!(running.mode(), &mode);
        assert_eq!(settings.get("video_device"), Some("0"));
        assert_eq!(settings.get("video_options"), Some("2"));
        assert!(planner.video_running());
        // Disabled while it runs.
        planner.press("BUT_videostart", Path::new("/"));
        assert!(planner.take_effects().is_empty());
        let deadline = web_time::Instant::now() + std::time::Duration::from_secs(5);
        while capture.as_ref().is_some_and(|c| c.frames() < 3) {
            assert!(web_time::Instant::now() < deadline);
            wasm_thread::sleep(std::time::Duration::from_millis(2));
        }
        planner.video_status(capture.as_ref());
        assert!(planner.video.frames >= 3);
        let frame = capture
            .as_ref()
            .and_then(mp_video::Capture::latest)
            .unwrap();
        let image = camera_image(&frame).expect("an image");
        let size = image.size(0);
        assert_eq!((size.width.0, size.height.0), (320, 240));
        assert_eq!(image.as_bytes(0).map(<[u8]>::len), Some(320 * 240 * 4));

        planner.press("BUT_videostop", Path::new("/"));
        assert!(!planner.video_running());
        let effects = planner.take_effects();
        assert_eq!(effects, [Effect::VideoStop]);
        run_video(&mut planner, &mut settings, &mut capture, &effects[0]);
        assert!(capture.is_none());
        assert_eq!(source.streams_stopped(), 1);
        // The settings stay: Activate selects them next time.
        assert_eq!(settings.get("video_options"), Some("2"));
    }

    /// A camera that will not open: "Camera Fail: " and the reason, for the status line; nothing
    /// written, Start still enabled. With nothing chosen, Start fails the same way.
    #[test]
    fn a_refused_start_goes_on_the_status_line() {
        let mut settings = Persisted::at(None);
        let mut planner = with_cameras(&mut settings, FakeSource::webcam().refusing_open("busy"));
        planner.press("BUT_videostart", Path::new("/"));
        let effects = planner.take_effects();
        assert_eq!(
            effects,
            [
                Effect::VideoStop,
                Effect::Status("Camera Fail: no Video Device and Video Format chosen".to_owned())
            ]
        );
        let mut capture = None;
        assert_eq!(
            run_video(&mut planner, &mut settings, &mut capture, &effects[1]).as_deref(),
            Some("Camera Fail: no Video Device and Video Format chosen")
        );

        planner.toggle_dropdown("CMB_videosources", &settings);
        planner.press("BUT_videostart", Path::new("/"));
        let mut status = None;
        for effect in planner.take_effects() {
            status = status.or(run_video(
                &mut planner,
                &mut settings,
                &mut capture,
                &effect,
            ));
        }
        assert_eq!(status.as_deref(), Some("Camera Fail: busy"));
        assert!(capture.is_none());
        assert!(!planner.video_running());
        assert_eq!(settings.get("video_device"), None);
        assert_eq!(planner.video.error.as_deref(), Some("Camera Fail: busy"));
        // No message box: the words are the status line's.
        assert!(planner.message().is_none());
    }

    /// A device that will not list its formats is the C#'s "Can not add video source" box.
    #[test]
    fn a_device_that_will_not_list_its_formats_says_so() {
        let mut settings = Persisted::at(None);
        let mut planner = with_cameras(
            &mut settings,
            FakeSource::webcam().refusing_modes("no such device"),
        );
        planner.toggle_dropdown("CMB_videosources", &settings);
        assert_eq!(
            planner.message().map(|m| m.text.as_str()),
            Some("Can not add video source\nno such device")
        );
        assert!(planner.combo_items("CMB_videoresolutions").is_empty());
    }

    /// Activate with the two indexes saved selects the device and the format by index; an index
    /// the lists do not have leaves the first; no key lists nothing.
    /// `// C#: ConfigPlanner.cs:216-233`
    #[test]
    fn activate_selects_the_saved_device_and_format_by_index() {
        let mut settings = Persisted::at(None);
        settings.set("video_device", "1");
        settings.set("video_options", "0");
        let planner = with_cameras(&mut settings, two_cameras());
        assert_eq!(
            planner.combo_text("CMB_videosources"),
            "USB Capture (/dev/video4)"
        );
        assert_eq!(
            planner.combo_text("CMB_videoresolutions"),
            "1920 x 1080 60.00 fps MJPG"
        );

        let mut settings = Persisted::at(None);
        settings.set("video_device", "0");
        settings.set("video_options", "1");
        let planner = with_cameras(&mut settings, two_cameras());
        assert_eq!(
            planner.combo_text("CMB_videosources"),
            "Integrated_Webcam_HD: Integrate (/dev/video0)"
        );
        assert_eq!(
            planner.combo_text("CMB_videoresolutions"),
            "640 x 480 30.00 fps YUYV"
        );

        // A camera gone: the index is past the list, and the first device stays selected.
        let mut settings = Persisted::at(None);
        settings.set("video_device", "5");
        settings.set("video_options", "9");
        let planner = with_cameras(&mut settings, two_cameras());
        assert_eq!(
            planner.combo_text("CMB_videosources"),
            "Integrated_Webcam_HD: Integrate (/dev/video0)"
        );
        assert_eq!(
            planner.combo_text("CMB_videoresolutions"),
            "1280 x 720 30.00 fps MJPG"
        );
        // Activate writes neither.
        assert_eq!(settings.get("video_device"), Some("5"));
    }

    /// A list longer than thirty rows scrolls on the wheel, as the other pages' lists do.
    #[test]
    fn a_long_format_list_scrolls() {
        let modes: Vec<mp_video::Mode> = (1..=40)
            .map(|step| mp_video::Mode {
                format: mp_video::PixelFormat::Yuyv,
                width: 16 * step,
                height: 9 * step,
                interval: (1, 30),
            })
            .collect();
        let mut settings = Persisted::at(None);
        let mut planner = with_cameras(
            &mut settings,
            FakeSource::empty().with_device("/dev/video0", "cam", &modes),
        );
        planner.toggle_dropdown("CMB_videosources", &settings);
        planner.toggle_dropdown("CMB_videosources", &settings);
        planner.toggle_dropdown("CMB_videoresolutions", &settings);
        planner.scroll_list("CMB_videoresolutions", 60);
        let facts = facts(&planner, &settings, false).0;
        assert!(
            facts
                .iter()
                .any(|(key, value)| key == "config.planner.video.list.top" && value == "10")
        );
    }

    fn index_of(name: &str, text: &str) -> usize {
        COMBOS
            .iter()
            .find(|spec| spec.name == name)
            .and_then(|spec| spec.items.iter().position(|item| *item == text))
            .expect("an item of the combo")
    }

    /// `Settings.Instance` as the next start reads it: the file, reloaded.
    fn reload(path: &Path) -> mp_settings::Config {
        mp_settings::Config::load(path).expect("the saved file reads back")
    }

    #[test]
    fn activating_with_nothing_set_writes_the_severity_and_the_icon_defaults() {
        let mut settings = Persisted::at(None);
        let planner = activated(&mut settings);
        assert!(planner.is_active());
        // C#: ConfigPlanner.cs:100-104 - Warning, and written.
        assert_eq!(settings.get("severity"), Some("4"));
        assert_eq!(planner.combo_text("CMB_severity"), "Warning");
        // C#: ConfigPlanner.cs:239-243 - read with a default of true, ticked from the Designer's
        // unticked, so each handler writes True.
        for key in [
            "GMapMarkerBase_DisplayCOG",
            "GMapMarkerBase_DisplayHeading",
            "GMapMarkerBase_DisplayNavBearing",
            "GMapMarkerBase_DisplayRadius",
            "GMapMarkerBase_DisplayTarget",
        ] {
            assert_eq!(settings.get(key), Some("True"), "{key}");
        }
        // C#: ConfigPlanner.cs:245, 1139 - read under one spelling, written under another.
        assert_eq!(settings.get("GMapMarkerBase_length"), Some("500"));
        assert_eq!(settings.get("GMapMarkerBase_Length"), None);
        assert_eq!(planner.number("num_linelength"), Some(500.0));
        // The Designer's values for the rest.
        assert_eq!(planner.number("NUM_tracklength"), Some(200.0));
        assert_eq!(planner.number("num_gcsid"), Some(255.0));
        assert!(planner.checked("CHK_disttohomeflightdata"));
        assert!(planner.checked("CHK_showairports"));
        assert!(planner.checked("chk_shownofly"));
        assert!(!planner.checked("CHK_maprotation"));
        // Binding the unit lists selects their first items; the units are ChangeUnits' with no
        // settings.
        assert_eq!(planner.combo_text("CMB_distunits"), "Meters");
        assert_eq!(planner.combo_text("CMB_speedunits"), "meters_per_second");
        assert_eq!(planner.units().alt_unit, "m");
        // cs's rates: CurrentState's defaults.
        assert_eq!(planner.rates(), [4, 2, 2, 2, 2]);
        assert_eq!(planner.combo_text("CMB_rateattitude"), "4");
        assert_eq!(planner.combo_text("CMB_raterc"), "2");
        assert_eq!(planner.combo_text("cmb_secondarydisplaystyle"), "Normal");
        assert_eq!(planner.combo_text("CMB_mapCache"), "ServerAndCache");
        assert_eq!(planner.combo_text("CMB_Layout"), "Advanced");
        // The speech boxes are hidden until speech is enabled.
        assert!(!planner.shown("CHK_speechwaypoint"));
        assert!(planner.shown("CHK_enablespeech"));
    }

    #[test]
    fn activating_reads_every_key_the_csharp_reads() {
        let mut settings = Persisted::at(None);
        for (key, value) in [
            ("speechenable", "True"),
            ("speechwaypointenabled", "True"),
            ("showairports", "False"),
            ("CHK_maprotation", "true"),
            ("loadwpsonconnect", "True"),
            ("distunits", "Feet"),
            ("altunits", "Feet"),
            ("speedunits", "knots"),
            ("severity", "2"),
            ("NUM_tracklength", "1500"),
            ("CMB_rateattitude", "10"),
            ("CMB_raterc", "7"),
            ("GMapMarkerBase_DisplayCOG", "False"),
            ("mapicondesc", "{alt}"),
            ("GMapMarkerBase_InactiveDisplayStyle", "Hidden"),
            ("mapCache", "CacheOnly"),
            ("gcsid", "250"),
        ] {
            settings.set(key, value);
        }
        let planner = activated(&mut settings);
        assert!(planner.checked("CHK_enablespeech"));
        assert!(planner.shown("CHK_speechwaypoint"));
        assert!(planner.checked("CHK_speechwaypoint"));
        assert!(!planner.checked("CHK_showairports"));
        assert!(planner.checked("CHK_maprotation"));
        assert!(planner.checked("CHK_loadwponconnect"));
        assert_eq!(planner.combo_text("CMB_distunits"), "Feet");
        assert_eq!(planner.combo_text("CMB_speedunits"), "knots");
        assert_eq!(
            (
                planner.units().dist_unit,
                planner.units().alt_unit,
                planner.units().speed_unit
            ),
            ("ft", "ft", "kts")
        );
        assert_eq!(planner.combo_text("CMB_severity"), "Critical");
        assert_eq!(planner.number("NUM_tracklength"), Some(1500.0));
        assert_eq!(planner.rates(), [10, 2, 2, 7, 2]);
        assert_eq!(planner.combo_text("CMB_rateattitude"), "10");
        assert_eq!(planner.combo_text("CMB_raterc"), "7");
        assert!(!planner.checked("chk_displaycog"));
        assert_eq!(settings.get("GMapMarkerBase_DisplayCOG"), Some("False"));
        assert!(planner.checked("chk_displaytooltip"));
        assert_eq!(planner.combo_text("cmb_secondarydisplaystyle"), "Hidden");
        assert_eq!(planner.combo_text("CMB_mapCache"), "CacheOnly");
        assert!(cache_only(&settings));
        assert!(load_wps_on_connect(&settings));
        assert_eq!(planner.gcssysid(), 250);
        assert_eq!(planner.number("num_gcsid"), Some(250.0));
    }

    #[test]
    fn a_rate_not_in_its_list_shows_nothing() {
        let mut settings = Persisted::at(None);
        settings.set("CMB_rateposition", "3000");
        let planner = activated(&mut settings);
        assert_eq!(planner.rates()[1], 3000);
        assert_eq!(planner.combo_text("CMB_rateposition"), "");
    }

    #[test]
    fn a_unit_combo_writes_its_key_and_changes_the_units_at_once() {
        let scratch = Scratch::new("units");
        let mut settings = Persisted::at(Some(scratch.config()));
        let mut planner = activated(&mut settings);
        planner.toggle_dropdown("CMB_altunits", &settings);
        assert_eq!(planner.dropdown(), Some("CMB_altunits"));
        planner.choose(
            "CMB_altunits",
            index_of("CMB_altunits", "Feet"),
            &mut settings,
        );
        assert_eq!(planner.dropdown(), None);
        assert_eq!(settings.get("altunits"), Some("Feet"));
        assert_eq!(planner.combo_text("CMB_altunits"), "Feet");
        // C#: MainV2.cs:4280-4299 - 3.2808399f and "ft".
        assert_eq!(planner.units().alt_unit, "ft");
        assert_eq!(planner.units().alt, "3.2808399".parse::<f32>().unwrap());
        // The distance and speed are ChangeUnits' defaults, set with it.
        assert_eq!(planner.units().dist_unit, "m");
        planner.choose(
            "CMB_distunits",
            index_of("CMB_distunits", "Feet"),
            &mut settings,
        );
        planner.choose(
            "CMB_speedunits",
            index_of("CMB_speedunits", "kph"),
            &mut settings,
        );
        assert_eq!(settings.get("distunits"), Some("Feet"));
        assert_eq!(settings.get("speedunits"), Some("kph"));
        assert_eq!(planner.units().dist_unit, "ft");
        assert_eq!(planner.units().speed_unit, "kph");
        // And the units are what the next start-up sets, from the file the next save writes.
        settings.save_config(SaveEvent::FlightData).expect("saved");
        let restarted = Planner::new(&Persisted::at(Some(scratch.config())));
        assert_eq!(restarted.units(), planner.units());
    }

    #[test]
    fn a_rate_writes_its_combos_name_and_asks_for_its_streams() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.choose(
            "CMB_rateattitude",
            index_of("CMB_rateattitude", "10"),
            &mut settings,
        );
        assert_eq!(settings.get("CMB_rateattitude"), Some("10"));
        assert_eq!(planner.rates()[0], 10);
        // C#: ConfigPlanner.cs:583-585 - EXTRA1 then EXTRA2.
        assert_eq!(
            planner.take_effects(),
            [Effect::Stream(10, 10), Effect::Stream(11, 10)]
        );
        planner.choose(
            "CMB_ratesensors",
            index_of("CMB_ratesensors", "25"),
            &mut settings,
        );
        // C#: ConfigPlanner.cs:637-639 - EXTRA3 then RAW_SENSORS.
        assert_eq!(
            planner.take_effects(),
            [Effect::Stream(12, 25), Effect::Stream(1, 25)]
        );
        for (combo, stream) in [
            ("CMB_rateposition", 6),
            ("CMB_ratestatus", 2),
            ("CMB_raterc", 3),
        ] {
            planner.choose(combo, index_of(combo, "5"), &mut settings);
            assert_eq!(planner.take_effects(), [Effect::Stream(stream, 5)]);
            assert_eq!(settings.get(combo), Some("5"));
        }
        assert_eq!(planner.rates(), [10, 5, 5, 5, 25]);
        // The same item again is no change, and no handler.
        planner.choose("CMB_raterc", index_of("CMB_raterc", "5"), &mut settings);
        assert!(planner.take_effects().is_empty());
    }

    /// The rate combo's requests, through the real link to a scripted vehicle: each stream the
    /// handler names, at the rate chosen, twice, and nothing for -1.
    #[test]
    fn the_rate_requests_reach_the_vehicle_twice_each() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.choose(
            "CMB_rateattitude",
            index_of("CMB_rateattitude", "50"),
            &mut settings,
        );
        let mut sent = 0;
        for effect in planner.take_effects() {
            if let Effect::Stream(stream, hz) = effect {
                sent += request_datastream(&telemetry, stream, hz);
            }
        }
        assert_eq!(sent, 4);
        let mut heard = Vec::new();
        until("the four requests", || {
            heard.extend(
                vehicle
                    .read()
                    .into_iter()
                    .filter_map(|message| match message {
                        MavMessage::RequestDataStream(request) => Some(request),
                        _ => None,
                    }),
            );
            heard.len() >= 4
        });
        let streams: Vec<(u8, u16, u8, u8, u8)> = heard
            .iter()
            .map(|request| {
                (
                    request.req_stream_id,
                    request.req_message_rate,
                    request.start_stop,
                    request.target_system,
                    request.target_component,
                )
            })
            .collect();
        let (sysid, compid) = (VEHICLE.payload_target(), VEHICLE.compid);
        assert_eq!(
            streams,
            [
                (10, 50, 1, sysid, compid),
                (10, 50, 1, sysid, compid),
                (11, 50, 1, sysid, compid),
                (11, 50, 1, sysid, compid),
            ]
        );
        // -1: `requestDatastream` returns before sending.
        assert_eq!(request_datastream(&telemetry, 10, -1), 0);
        // No link: nothing to send on.
        assert_eq!(request_datastream(&Telemetry::idle(), 10, 4), 0);
    }

    #[test]
    fn enable_speech_writes_and_shows_the_speech_boxes() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        // Hidden boxes take no clicks.
        planner.click("CHK_speechwaypoint", &mut settings);
        assert!(planner.prompt().is_none());
        planner.click("CHK_enablespeech", &mut settings);
        assert_eq!(settings.get("speechenable"), Some("True"));
        for name in SPEECH_BOXES {
            assert!(planner.shown(name), "{name}");
        }
        planner.click("CHK_speechArmedOnly", &mut settings);
        assert_eq!(settings.get("speech_armed_only"), Some("True"));
        planner.click("CHK_enablespeech", &mut settings);
        assert_eq!(settings.get("speechenable"), Some("False"));
        assert!(!planner.shown("CHK_speechbattery"));
    }

    #[test]
    fn a_speech_box_asks_for_its_template_and_writes_the_answer() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.click("CHK_enablespeech", &mut settings);
        planner.click("CHK_speechwaypoint", &mut settings);
        // The enabled key is written before the box asks.
        assert_eq!(settings.get("speechwaypointenabled"), Some("True"));
        let prompt = planner.prompt().expect("the InputBox");
        assert_eq!(prompt.step.title, "Notification");
        assert_eq!(prompt.step.question, "What do you want it to say?");
        assert_eq!(prompt.field.value(), "Heading to Waypoint {wpn}");
        // Typing, then Enter.
        assert!(planner.prompt_key(&key("backspace", None), &mut settings));
        assert!(planner.prompt_key(&key("x", Some("x")), &mut settings));
        assert!(planner.prompt_key(&key("enter", None), &mut settings));
        assert!(planner.prompt().is_none());
        assert_eq!(
            settings.get("speechwaypoint"),
            Some("Heading to Waypoint {wpnx")
        );
        // Unticking writes False and asks nothing.
        planner.click("CHK_speechwaypoint", &mut settings);
        assert_eq!(settings.get("speechwaypointenabled"), Some("False"));
        assert!(planner.prompt().is_none());
        // Ticked again, the box starts from what was saved.
        planner.click("CHK_speechwaypoint", &mut settings);
        assert_eq!(
            planner.prompt().map(|prompt| prompt.field.value()),
            Some("Heading to Waypoint {wpnx")
        );
    }

    #[test]
    fn the_battery_warning_asks_three_times_and_a_cancel_ends_it() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.click("CHK_enablespeech", &mut settings);
        planner.click("CHK_speechbattery", &mut settings);
        planner.answer(&mut settings);
        assert_eq!(
            settings.get("speechbattery"),
            Some("WARNING, Battery at {batv} Volt, {batp} percent")
        );
        let prompt = planner.prompt().expect("the voltage");
        assert_eq!(prompt.step.title, "Battery Level");
        assert_eq!(prompt.field.value(), "9.6");
        planner.cancel();
        assert!(planner.prompt().is_none());
        assert_eq!(settings.get("speechbatteryvolt"), None);
        assert_eq!(settings.get("speechbatterypercent"), None);
        // The box stays ticked and its key True, as the handler wrote it first.
        assert!(planner.checked("CHK_speechbattery"));
        assert_eq!(settings.get("speechbatteryenabled"), Some("True"));
        // All the way through.
        planner.click("CHK_speechbattery", &mut settings);
        planner.click("CHK_speechbattery", &mut settings);
        for _ in 0..3 {
            planner.answer(&mut settings);
        }
        assert!(planner.prompt().is_none());
        assert_eq!(settings.get("speechbatteryvolt"), Some("9.6"));
        assert_eq!(settings.get("speechbatterypercent"), Some("20"));
    }

    #[test]
    fn every_speech_box_asks_what_its_handler_asks() {
        let expected: [(&str, &[(&str, &str)]); 5] = [
            (
                "CHK_speechmode",
                &[("speechmode", "Mode changed to {mode}")],
            ),
            (
                "CHK_speechcustom",
                &[(
                    "speechcustom",
                    "Heading to Waypoint {wpn}, altitude is {alt}, Ground speed is {gsp} ",
                )],
            ),
            (
                "CHK_speechaltwarning",
                &[
                    ("speechalt", "WARNING, low altitude {alt}"),
                    ("speechaltheight", "2"),
                ],
            ),
            (
                "CHK_speecharmdisarm",
                &[("speecharm", "Armed"), ("speechdisarm", "Disarmed")],
            ),
            (
                "CHK_speechlowspeed",
                &[
                    ("speechlowgroundspeed", "Low Ground Speed {gsp}"),
                    ("speechlowgroundspeedtrigger", "0"),
                    ("speechlowairspeed", "Low Air Speed {asp}"),
                    ("speechlowairspeedtrigger", "0"),
                ],
            ),
        ];
        for (name, answers) in expected {
            let mut settings = Persisted::at(None);
            let mut planner = activated(&mut settings);
            planner.click("CHK_enablespeech", &mut settings);
            planner.click(name, &mut settings);
            for (key, default) in answers {
                let prompt = planner.prompt().expect("a box");
                assert_eq!(prompt.step.key, *key, "{name}");
                assert_eq!(prompt.field.value(), *default, "{name}");
                planner.answer(&mut settings);
                assert_eq!(settings.get(key), Some(*default), "{name}");
            }
            assert!(planner.prompt().is_none(), "{name}");
        }
    }

    #[test]
    fn the_warning_altitude_is_saved_in_metres() {
        let mut settings = Persisted::at(None);
        settings.set("altunits", "Feet");
        let mut planner = activated(&mut settings);
        planner.click("CHK_enablespeech", &mut settings);
        planner.click("CHK_speechaltwarning", &mut settings);
        planner.answer(&mut settings);
        let prompt = planner.prompt.as_mut().expect("the altitude");
        prompt.field.set("10");
        planner.answer(&mut settings);
        // C#: ConfigPlanner.cs:684 - 10 / 3.2808399f, as `double.ToString()` writes it.
        let expected = mp_log::netfmt::double(10.0 / f64::from(3.280_84_f32));
        assert_eq!(settings.get("speechaltheight"), Some(expected.as_str()));
        assert!(expected.starts_with("3.0479999"), "{expected}");
        // Text that is not a number is `double.Parse`'s FormatException.
        planner.click("CHK_speechaltwarning", &mut settings);
        planner.click("CHK_speechaltwarning", &mut settings);
        planner.answer(&mut settings);
        planner
            .prompt
            .as_mut()
            .expect("the altitude")
            .field
            .set("low");
        planner.answer(&mut settings);
        let message = planner.message().expect("the unhandled exception's box");
        assert_eq!(message.title, "Send Error");
        assert!(message.text.contains("FormatException"), "{}", message.text);
        assert_eq!(settings.get("speechaltheight"), Some(expected.as_str()));
        planner.dismiss_message();
        assert!(planner.message().is_none());
    }

    #[test]
    fn map_rotation_and_no_fly_untick_each_other() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        assert!(planner.checked("chk_shownofly"));
        planner.click("CHK_maprotation", &mut settings);
        assert_eq!(settings.get("CHK_maprotation"), Some("True"));
        assert!(!planner.checked("chk_shownofly"));
        assert_eq!(settings.get("ShowNoFly"), Some("False"));
        planner.click("chk_shownofly", &mut settings);
        assert_eq!(settings.get("ShowNoFly"), Some("True"));
        assert!(!planner.checked("CHK_maprotation"));
        assert_eq!(settings.get("CHK_maprotation"), Some("False"));
    }

    #[test]
    fn the_settings_only_boxes_write_their_keys() {
        for (name, key, designer) in [
            ("CHK_resetapmonconnect", "CHK_resetapmonconnect", false),
            ("CHK_rtsresetesp32", "CHK_rtsresetesp32", false),
            ("CHK_disttohomeflightdata", "CHK_disttohomeflightdata", true),
            ("CHK_showairports", "showairports", true),
            ("chk_tfr", "showtfr", true),
            ("chk_norcreceiver", "norcreceiver", false),
            ("CHK_params_bg", "Params_BG", false),
            ("chk_slowMachine", "SlowMachine", false),
            ("chk_displayheading", "GMapMarkerBase_DisplayHeading", true),
        ] {
            let mut settings = Persisted::at(None);
            let mut planner = activated(&mut settings);
            assert_eq!(planner.checked(name), designer, "{name}");
            planner.click(name, &mut settings);
            assert_eq!(settings.get(key), Some(bool_text(!designer)), "{name}");
        }
    }

    #[test]
    fn load_waypoints_on_connect_sets_the_read_on_connect() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.click("CHK_loadwponconnect", &mut settings);
        assert_eq!(settings.get("loadwpsonconnect"), Some("True"));
        assert!(load_wps_on_connect(&settings));
        assert_eq!(planner.take_effects(), [Effect::ReadMissionOnConnect(true)]);
    }

    /// Each of the page's questions keeps its OK's answer as `InputBox` keeps every titled answer:
    /// under `"InputBox"` and the caption and question with all but letters and digits taken
    /// out, URL-encoded, before the handler looks at it - so an altitude that is not a number is
    /// kept too. Cancel keeps nothing.
    /// `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184; GCSViews/ConfigurationView/ConfigPlanner.cs:455-547, 673-684, 823-830, 890-913, 1122`
    #[test]
    fn each_question_keeps_its_ok_answer_under_the_input_box_key() {
        let mut keys: Vec<String> = SPEECH_BOXES
            .iter()
            .chain(&["chk_displaytooltip"])
            .flat_map(|name| steps(name))
            .map(|step| crate::config::optional::answers_key(step.title, step.question))
            .collect();
        keys.sort();
        keys.dedup();
        assert_eq!(
            keys,
            [
                "InputBoxAirSpeedWhatdoyouwantittosay",
                "InputBoxArmWhatdoyouwantittosay",
                "InputBoxBatteryLevelWhatVoltagedoyouwanttowarnat",
                "InputBoxBatteryLevelWhatpercentagedoyouwanttowarnat",
                "InputBoxDescriptionWhatdoyouwantittoshow",
                "InputBoxDisarmedWhatdoyouwantittosay",
                "InputBoxGroundSpeedWhatdoyouwantittosay",
                "InputBoxMinAltWhataltitudedoyouwanttowarnatrelativetohome",
                "InputBoxNotificationWhatdoyouwantittosay",
                "InputBoxspeedtriggerWhatspeeddoyouwanttowarnatms",
            ]
        );
        for key in &keys {
            assert!(crate::settings::PUBLISHED.contains(&key.as_str()), "{key}");
        }
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.click("CHK_enablespeech", &mut settings);
        // Cancel on Arm: nothing kept.
        planner.click("CHK_speecharmdisarm", &mut settings);
        planner.cancel();
        assert_eq!(settings.get("InputBoxArmWhatdoyouwantittosay"), None);
        // OK on both: each kept under its own caption.
        planner.click("CHK_speecharmdisarm", &mut settings);
        planner.click("CHK_speecharmdisarm", &mut settings);
        planner.prompt.as_mut().expect("Arm").field.set("Armed, go");
        planner.answer(&mut settings);
        planner.answer(&mut settings);
        assert_eq!(
            settings.get("InputBoxArmWhatdoyouwantittosay"),
            Some("Armed%2C+go")
        );
        assert_eq!(
            settings.get("InputBoxDisarmedWhatdoyouwantittosay"),
            Some("Disarmed")
        );
        // The altitude is kept before `double.Parse` refuses it.
        planner.click("CHK_speechaltwarning", &mut settings);
        planner.answer(&mut settings);
        assert_eq!(
            settings.get("InputBoxNotificationWhatdoyouwantittosay"),
            Some("WARNING%2C+low+altitude+%7Balt%7D")
        );
        planner.prompt.as_mut().expect("Min Alt").field.set("low");
        planner.answer(&mut settings);
        assert!(planner.message().is_some(), "the FormatException's box");
        assert_eq!(
            settings.get("InputBoxMinAltWhataltitudedoyouwanttowarnatrelativetohome"),
            Some("low")
        );
        assert_eq!(settings.get("speechaltheight"), None);
    }

    #[test]
    fn the_tooltip_box_asks_for_the_description_or_clears_it() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.click("chk_displaytooltip", &mut settings);
        let prompt = planner.prompt().expect("the description");
        assert_eq!(prompt.step.title, "Description");
        assert_eq!(
            prompt.field.value(),
            "{alt}{altunit} {airspeed}{speedunit} id:{sysid} Sats:{satcount} HDOP:{gpshdop} \
             Volts:{battery_voltage}"
        );
        planner.prompt.as_mut().expect("open").field.set("{alt}");
        planner.answer(&mut settings);
        assert_eq!(settings.get("mapicondesc"), Some("{alt}"));
        assert_eq!(settings.get("mapicondesc_default"), Some("{alt}"));
        planner.click("chk_displaytooltip", &mut settings);
        assert_eq!(settings.get("mapicondesc"), Some(""));
        assert_eq!(settings.get("mapicondesc_default"), Some("{alt}"));
    }

    #[test]
    fn the_number_boxes_write_on_every_change() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.step(0, true, &mut settings);
        assert_eq!(settings.get("NUM_tracklength"), Some("300"));
        // Bounded below at 100.
        for _ in 0..5 {
            planner.step(0, false, &mut settings);
        }
        assert_eq!(settings.get("NUM_tracklength"), Some("100"));
        // Typed, and read on Enter.
        planner.numbers[0].field.set("");
        for character in ["1", "2", "3", "4"] {
            planner.number_key(0, &key(character, Some(character)), &mut settings);
        }
        assert_eq!(settings.get("NUM_tracklength"), Some("100"));
        planner.number_key(0, &key("enter", None), &mut settings);
        assert_eq!(settings.get("NUM_tracklength"), Some("1234"));
        // The GCS id, and the byte it sets.
        planner.step(1, false, &mut settings);
        assert_eq!(settings.get("gcsid"), Some("254"));
        assert_eq!(planner.gcssysid(), 254);
        // Typed and left: read as the focus leaves.
        planner.tick(Some(2), &mut settings);
        planner.numbers[2].field.set("");
        for character in ["7", "0"] {
            planner.number_key(2, &key(character, Some(character)), &mut settings);
        }
        planner.tick(None, &mut settings);
        assert_eq!(settings.get("GMapMarkerBase_length"), Some("70"));
    }

    #[test]
    fn the_severity_and_the_inactive_style_and_access_mode_write_their_keys() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.choose(
            "CMB_severity",
            index_of("CMB_severity", "Debug"),
            &mut settings,
        );
        assert_eq!(settings.get("severity"), Some("7"));
        planner.choose(
            "cmb_secondarydisplaystyle",
            index_of("cmb_secondarydisplaystyle", "Transparent"),
            &mut settings,
        );
        assert_eq!(
            settings.get("GMapMarkerBase_InactiveDisplayStyle"),
            Some("Transparent")
        );
        planner.choose(
            "CMB_mapCache",
            index_of("CMB_mapCache", "CacheOnly"),
            &mut settings,
        );
        assert_eq!(settings.get("mapCache"), Some("CacheOnly"));
        assert!(cache_only(&settings));
        assert_eq!(planner.take_effects(), [Effect::MapAccess]);
    }

    #[test]
    fn a_dimmed_control_does_nothing() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        let before = settings.config().clone();
        for name in ["CHK_GDIPlus", "chk_ADSB", "CHK_beta", "chk_temp"] {
            let was = planner.checked(name);
            planner.click(name, &mut settings);
            assert_eq!(planner.checked(name), was, "{name}");
        }
        for name in ["CMB_theme", "CMB_language"] {
            planner.toggle_dropdown(name, &settings);
            assert_eq!(planner.dropdown(), None, "{name}");
            planner.choose(name, 0, &mut settings);
        }
        for (name, ..) in BUTTONS.iter().filter(|(_, _, _, dim)| dim.is_some()) {
            planner.press(name, Path::new("/"));
        }
        assert_eq!(settings.config(), &before);
        assert!(planner.take_effects().is_empty());
        assert!(planner.message().is_none());
        assert_eq!(
            dimmed(),
            [
                "CHK_GDIPlus",
                "CHK_Password",
                "chk_ADSB",
                "chk_analytics",
                "CHK_beta",
                "CHK_mavdebug",
                "chk_temp",
                "CMB_osdcolor",
                "CMB_language",
                "CMB_theme",
                "BUT_themecustom",
                "BUT_Vario",
            ]
        );
    }

    /// Layout: `Activate` selects the view's name - its first time raising the handler, which
    /// makes the preset the view and saves it - and a choice makes that preset the view: Basic
    /// takes the Advanced pages away, Custom with no file is Advanced again.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:55-73, 1019-1034; MainV2.cs:357-366`
    #[test]
    fn layout_chooses_the_display_view() {
        use crate::display_view::{DisplayName, DisplayView, SETTING, current, flag};
        let dir = mp_os::temp_dir().join(format!("mpr-planner-layout-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        let _ = mp_os::fs::create_dir_all(&dir);
        let mut settings = Persisted::at(Some(dir.join("config.xml")));
        let mut planner = Planner::new(&settings);
        planner.activate(&mut settings, None);
        assert_eq!(planner.combo_text("CMB_Layout"), "Advanced");
        assert_eq!(
            settings.get(SETTING).map(str::to_owned),
            Some(DisplayView::advanced().convert_to_string()),
            "the first Activate's handler saves the view"
        );
        assert!(planner.layout_shown());
        planner.choose("CMB_Layout", 0, &mut settings);
        assert_eq!(current(), DisplayView::basic());
        assert!(!flag("isAdvancedMode"));
        assert_eq!(
            settings.get(SETTING).map(str::to_owned),
            Some(DisplayView::basic().convert_to_string())
        );
        // Custom with no file: the Advanced preset, named Advanced.
        planner.choose("CMB_Layout", 2, &mut settings);
        assert_eq!(current().name, DisplayName::Advanced);
        // With the file beside config.xml: read, named Custom.
        let _ = mp_os::fs::write(
            dir.join(crate::display_view::CUSTOM_FILE),
            "{\"displayStandardParams\": true, \"displayPlannerLayout\": false}",
        );
        planner.choose("CMB_Layout", 0, &mut settings);
        planner.choose("CMB_Layout", 2, &mut settings);
        assert_eq!(current().name, DisplayName::Custom);
        assert!(flag("displayStandardParams"));
        // Activated again in that view: Custom selected, and the box hidden.
        planner.deactivate();
        planner.activate(&mut settings, None);
        assert_eq!(planner.combo_text("CMB_Layout"), "Custom");
        assert!(!planner.layout_shown());
        crate::display_view::set(DisplayView::advanced(), &mut settings);
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_dimmed_box_still_shows_its_setting() {
        let mut settings = Persisted::at(None);
        settings.set("CHK_GDIPlus", "True");
        settings.set("hudcolor", "Red");
        settings.set("theme", "custom.mpsystheme");
        let planner = activated(&mut settings);
        assert!(planner.checked("CHK_GDIPlus"));
        assert_eq!(planner.combo_text("CMB_osdcolor"), "Red");
        assert_eq!(planner.combo_text("CMB_theme"), "custom.mpsystheme");
    }

    /// Enable HUD Overlay: `MainV2` reads `hudon` from `CHK_hudshow` before any page shows, true
    /// without it; a click sets `hudon` and writes the key. `Activate` shows `hudon` only while a
    /// capture runs - running the handler when that changes the box - and otherwise the
    /// Designer's tick, whatever `hudon` is.
    /// `// C#: ConfigPlanner.cs:137-146, 375-379; MainV2.cs:940-941; HUD.cs:274`
    #[test]
    fn enable_hud_overlay_sets_hudon_and_activate_shows_it_while_capturing() {
        // Nothing saved: the HUD's own true.
        let mut settings = Persisted::at(None);
        let mut planner = with_cameras(&mut settings, FakeSource::webcam());
        assert!(planner.hud_on());
        assert!(planner.checked("CHK_hudshow"));
        assert_eq!(settings.get("CHK_hudshow"), None);
        assert!(!dimmed().contains(&"CHK_hudshow"));
        // Unticked: `hudon` false and saved.
        planner.click("CHK_hudshow", &mut settings);
        assert!(!planner.checked("CHK_hudshow"));
        assert!(!planner.hud_on());
        assert_eq!(settings.get("CHK_hudshow"), Some("False"));
        assert!(planner.take_effects().is_empty());
        // Activated again with no capture: the Designer's tick, `hudon` as it was.
        planner.deactivate();
        planner.activate(&mut settings, None);
        assert!(planner.checked("CHK_hudshow"));
        assert!(!planner.hud_on());
        assert_eq!(settings.get("CHK_hudshow"), Some("False"));

        // Read at start-up, whatever the case; a value `bool.Parse` refuses leaves true.
        for (saved, hudon) in [("False", false), (" true ", true), ("no", true)] {
            let mut settings = Persisted::at(None);
            settings.set("CHK_hudshow", saved);
            assert_eq!(Planner::new(&settings).hud_on(), hudon, "{saved:?}");
        }

        // While a capture runs, `Activate` shows `hudon`; the handler writes it again.
        let mut settings = Persisted::at(None);
        settings.set("CHK_hudshow", "False");
        let mut planner = with_cameras(&mut settings, FakeSource::webcam());
        assert!(planner.checked("CHK_hudshow"), "no capture: the Designer's");
        planner.toggle_dropdown("CMB_videosources", &settings);
        planner.choose("CMB_videosources", 0, &mut settings);
        planner.press("BUT_videostart", Path::new("/"));
        let mut capture = None;
        for effect in planner.take_effects() {
            run_video(&mut planner, &mut settings, &mut capture, &effect);
        }
        assert!(planner.video_running());
        settings.set("CHK_hudshow", "untouched");
        planner.deactivate();
        planner.activate(&mut settings, None);
        assert!(!planner.checked("CHK_hudshow"));
        assert!(!planner.hud_on());
        assert_eq!(settings.get("CHK_hudshow"), Some("False"));
        // Ticked while capturing: `hudon` true, and the next Activate leaves it.
        planner.click("CHK_hudshow", &mut settings);
        assert!(planner.hud_on());
        assert_eq!(settings.get("CHK_hudshow"), Some("True"));
        planner.deactivate();
        planner.activate(&mut settings, None);
        assert!(planner.checked("CHK_hudshow"));
        drop(capture);
    }

    #[test]
    fn the_buttons_open_the_joystick_the_folder_dialog_and_the_cache() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.press("BUT_Joystick", Path::new("/"));
        assert!(planner.joystick_open());
        // The window is modal: nothing else on the page takes a click while it shows.
        planner.click("CHK_maprotation", &mut settings);
        assert!(!planner.checked("CHK_maprotation"));
        planner.close_joystick();
        planner.press("BUT_logdirbrowse", Path::new("/"));
        assert_eq!(planner.take_effects(), [Effect::BrowseLogDirectory]);
        let cache = mp_os::temp_dir();
        planner.press("BUT_mapCacheDir", &cache);
        assert_eq!(planner.take_effects(), [Effect::OpenDirectory(cache)]);
        let missing = Path::new("/no/such/gmapcache");
        planner.press("BUT_mapCacheDir", missing);
        assert!(planner.take_effects().is_empty());
        assert_eq!(
            planner.message().map(|message| message.text.as_str()),
            Some("/no/such/gmapcache Directory does not exist!")
        );
    }

    #[test]
    fn the_log_path_is_written_when_it_names_a_directory() {
        let mut settings = Persisted::at(None);
        let existing = mp_os::temp_dir();
        let mut planner = Planner::new(&settings);
        planner.activate(&mut settings, Some(&existing));
        // `TextChanged` from `Activate`'s own assignment.
        assert_eq!(planner.log_dir(), existing.display().to_string());
        assert_eq!(
            settings.get("logdirectory"),
            Some(existing.display().to_string().as_str())
        );
        // A path that does not exist is not taken.
        planner.browsed(Path::new("/no/such/logs"), &mut settings);
        assert_eq!(planner.log_dir(), "/no/such/logs");
        assert_eq!(
            settings.get("logdirectory"),
            Some(existing.display().to_string().as_str())
        );
        // Typed back to one that does: taken on the keystroke that makes it.
        let mut settings = Persisted::at(None);
        let mut planner = Planner::new(&settings);
        planner.activate(&mut settings, None);
        assert_eq!(planner.log_dir(), "");
        assert_eq!(settings.get("logdirectory"), None);
        for character in ["/", "t", "m", "p"] {
            planner.log_dir_key(&key(character, Some(character)), &mut settings);
        }
        if Path::new("/tmp").os_is_dir() {
            assert_eq!(settings.get("logdirectory"), Some("/tmp"));
        }
    }

    #[test]
    fn leaving_the_page_cancels_what_is_open() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        planner.click("chk_displaytooltip", &mut settings);
        assert!(planner.prompt().is_some());
        planner.deactivate();
        assert!(!planner.is_active());
        assert!(planner.prompt().is_none());
        assert!(!planner.joystick_open());
    }

    /// Every key the page writes is one it publishes as `config.planner.<key>` - or, for the
    /// lists its `InputBox`es keep, as `config.<key>` - so a script can assert on each, and the
    /// dictionary holds nothing else the page did not put there.
    #[test]
    fn every_key_written_is_one_the_page_publishes() {
        let mut settings = Persisted::at(None);
        let mut planner = activated(&mut settings);
        for spec in CHECKS {
            planner.click(spec.name, &mut settings);
            while planner.prompt().is_some() {
                planner.answer(&mut settings);
            }
        }
        planner.click("CHK_enablespeech", &mut settings);
        for name in SPEECH_BOXES {
            planner.click(name, &mut settings);
            while planner.prompt().is_some() {
                planner.answer(&mut settings);
            }
        }
        for spec in COMBOS {
            for index in 0..spec.items.len() {
                planner.choose(spec.name, index, &mut settings);
            }
        }
        for index in 0..NUMBERS.len() {
            planner.step(index, true, &mut settings);
        }
        // The lists `InputBox` keeps are published as `config.<key>`, with the other screens'.
        for key in settings.config().keys() {
            assert!(
                KEYS.contains(&key)
                    || (key.starts_with("InputBox") && crate::settings::PUBLISHED.contains(&key)),
                "{key} is written but not published"
            );
        }
        assert!(settings.config().len() > 50, "{}", settings.config().len());
    }

    /// `Settings.Instance[key] = ...` puts the key in the dictionary and nothing on disk; the
    /// next `SaveConfig` writes it with every key the page does not have, as the C# wrote them;
    /// and the next start reads it back - `MainV2`'s `ChangeUnits`, then `Activate`.
    /// `// C#: ExtLibs/Utilities/Settings.cs:58-61, 507-550; MainV2.cs:838, 1311-1317`
    #[test]
    fn a_change_is_in_the_dictionary_at_once_and_in_config_xml_after_the_next_save() {
        let scratch = Scratch::new("save");
        let path = scratch.seed(CSHARP_FILE);
        let mut settings = Persisted::at(Some(path.clone()));
        let mut planner = activated(&mut settings);
        planner.choose(
            "CMB_distunits",
            index_of("CMB_distunits", "Feet"),
            &mut settings,
        );
        planner.click("CHK_enablespeech", &mut settings);
        // `speechcustom`'s default ends in a space. C#: ConfigPlanner.cs:487
        planner.click("CHK_speechcustom", &mut settings);
        planner.answer(&mut settings);
        let custom = "Heading to Waypoint {wpn}, altitude is {alt}, Ground speed is {gsp} ";

        // In the dictionary at once, Activate's own writes with them ...
        assert_eq!(settings.get("distunits"), Some("Feet"));
        assert_eq!(settings.get("speechenable"), Some("True"));
        assert_eq!(settings.get("speechcustom"), Some(custom));
        assert_eq!(settings.get("severity"), Some("4"));
        // ... and not on disk: a handler saves nothing.
        assert_eq!(mp_os::fs::read_to_string(&path).expect("read"), CSHARP_FILE);

        // FLIGHT DATA's SaveConfig writes the whole dictionary.
        settings.save_config(SaveEvent::FlightData).expect("saved");
        let saved = reload(&path);
        for key in KEYS {
            assert_eq!(saved.get(key), settings.get(key), "{key}");
        }
        assert_eq!(saved.get("distunits"), Some("Feet"));
        assert_eq!(saved.get("speechcustom"), Some(custom));
        assert_eq!(saved.get("GMapMarkerBase_length"), Some("500"));
        // Every key of the C#'s file is still there, as it was: none is the page's to change but
        // `logdirectory`, which names no directory here and so is not written.
        let before = mp_settings::Config::parse(CSHARP_FILE).expect("parses");
        for key in before.keys() {
            assert_eq!(saved.get(key), before.get(key), "{key}");
        }
        assert!(saved.len() > before.len());

        // The next start.
        let mut restarted = Persisted::at(Some(path));
        let mut planner = Planner::new(&restarted);
        assert_eq!(planner.units().dist_unit, "ft");
        planner.activate(&mut restarted, None);
        assert_eq!(planner.combo_text("CMB_distunits"), "Feet");
        assert!(planner.checked("CHK_enablespeech"));
        assert!(planner.checked("CHK_speechcustom"));
        assert_eq!(restarted.get("speechcustom"), Some(custom));
    }

    /// The real `config.xml` on this machine, copied: the page activated over it, a unit changed,
    /// and a save - every key the page does not have is written back as it was read.
    #[test]
    fn the_real_config_keeps_every_key_the_page_does_not_have() {
        let Some(real) = mp_settings::Config::csharp_path() else {
            eprintln!("skipped: no home directory");
            return;
        };
        let Ok(text) = mp_os::fs::read_to_string(&real) else {
            eprintln!("skipped: no Mission Planner config at {}", real.display());
            return;
        };
        let scratch = Scratch::new("real");
        let path = scratch.seed(&text);
        let mut settings = Persisted::at(Some(path.clone()));
        let before = settings.config().clone();
        let mut planner = activated(&mut settings);
        planner.choose(
            "CMB_altunits",
            index_of("CMB_altunits", "Feet"),
            &mut settings,
        );
        settings.save_config(SaveEvent::FlightData).expect("saved");
        let after = reload(&path);
        let mut kept = 0;
        for key in before.keys().into_iter().filter(|key| !KEYS.contains(key)) {
            assert_eq!(after.get(key), before.get(key), "{key}");
            kept += 1;
        }
        assert_eq!(after.get("altunits"), Some("Feet"));
        eprintln!("{kept} keys the page does not have, kept");
    }

    /// A key with Ctrl held, as `xdotool key ctrl+<key>` sends it.
    fn chord(name: &str) -> KeyDownEvent {
        let mut event = key(name, None);
        event.keystroke.modifiers.control = true;
        event
    }

    /// One run of the application under a `tests/gui/config-*.gui` script of this page, as the
    /// model holds it.
    struct Run {
        settings: Persisted,
        planner: Planner,
        /// `MissionPlanner::auto_read_mission`.
        read_mission: bool,
        /// `MissionPlanner::video`: the capture Start opened.
        capture: Option<mp_video::Capture>,
        /// `MissionPlanner::file_status`: the status line.
        status: Option<String>,
    }

    impl Run {
        /// `MissionPlanner::new` on the CONFIG screen with no link: `Settings.Instance` read, what
        /// `MainV2` takes from it, the start-up save - then the page's `Activate`, which the
        /// first frame of the CONFIG screen runs. The cameras are `source`'s.
        fn start(path: &Path, logs: &Path, source: &FakeSource) -> Self {
            let mut settings = Persisted::at(Some(path.to_path_buf()));
            let planner = Planner::new(&settings).with_video_source(Arc::new(source.clone()));
            let read_mission = load_wps_on_connect(&settings);
            settings.save_config(SaveEvent::Startup).expect("saved");
            let mut run = Self {
                settings,
                planner,
                read_mission,
                capture: None,
                status: None,
            };
            run.planner.activate(&mut run.settings, Some(logs));
            run
        }

        /// What `planner_apply` and `planner_window_effects` do with a handler's effects, with
        /// no vehicle and no map.
        fn effects(&mut self) {
            for effect in self.planner.take_effects() {
                match effect {
                    Effect::Stream(stream, hz)
                        if request_datastream(&Telemetry::idle(), stream, hz) > 0 =>
                    {
                        self.planner.sent(stream, hz);
                    }
                    Effect::ReadMissionOnConnect(on) => self.read_mission = on,
                    effect
                    @ (Effect::VideoStart { .. } | Effect::VideoStop | Effect::Status(_)) => {
                        let words = run_video(
                            &mut self.planner,
                            &mut self.settings,
                            &mut self.capture,
                            &effect,
                        );
                        if words.is_some() {
                            self.status = words;
                        }
                    }
                    _ => {}
                }
            }
        }

        /// A click on one of the page's controls, by its probe id.
        fn click(&mut self, id: &str, cache: &Path) {
            let settings = &mut self.settings;
            let planner = &mut self.planner;
            match id {
                "planner-prompt-ok" => return planner.answer(settings),
                "planner-prompt-cancel" => return planner.cancel(),
                "planner-joystick-close" => return planner.close_joystick(),
                _ => {}
            }
            let name = id
                .strip_prefix("planner-")
                .unwrap_or_else(|| panic!("{id} is not the page's"));
            if let Some(spec) = COMBOS.iter().find(|spec| spec.name == name) {
                planner.toggle_dropdown(spec.name, settings);
            } else if let Some((spec, index)) = COMBOS.iter().find_map(|spec| {
                let item = name.strip_prefix(spec.name)?.strip_prefix('-')?;
                Some((spec, spec.items.iter().position(|text| *text == item)?))
            }) {
                planner.choose(spec.name, index, settings);
            } else if let Some((combo, index)) = ["CMB_videosources", "CMB_videoresolutions"]
                .into_iter()
                .find_map(|combo| {
                    let row = name.strip_prefix(combo)?.strip_prefix('-')?;
                    Some((combo, row.parse::<usize>().ok()?))
                })
            {
                // A video list's row, `<combo>-<index>`, as `servo_output::dropdown` names it.
                planner.choose(combo, index, settings);
            } else if let Some((index, up)) =
                NUMBERS
                    .iter()
                    .enumerate()
                    .find_map(|(index, (number, ..))| {
                        let arrow = name.strip_prefix(*number)?.strip_prefix('-')?;
                        Some((index, arrow == "up"))
                    })
            {
                planner.step(index, up, settings);
            } else if let Some(spec) = CHECKS.iter().find(|spec| spec.name == name) {
                planner.click(spec.name, settings);
            } else if let Some((button, ..)) = BUTTONS.iter().find(|(button, ..)| *button == name) {
                planner.press(button, cache);
            } else {
                panic!("{id} is not a control of the page");
            }
        }

        /// `settle`: the capture reads for `seconds` - here until the scripted camera has given
        /// twenty frames more, which a 30 fps camera gives in under a second, or `seconds` have
        /// passed - and the next frame's `video_tick` hands its state to the page.
        fn settle(&mut self, seconds: f32) {
            if let Some(capture) = &self.capture {
                let wanted = capture.frames() + 20;
                let deadline =
                    web_time::Instant::now() + std::time::Duration::from_secs_f32(seconds);
                while capture.frames() < wanted && web_time::Instant::now() < deadline {
                    wasm_thread::sleep(std::time::Duration::from_millis(5));
                }
            }
            self.planner.video_status(self.capture.as_ref());
        }

        /// What the facts say of `fact`, or `None` for one that is not the page's or the
        /// dictionary's (the CONFIG screen's own).
        fn fact(&self, fact: &str) -> Option<String> {
            if fact.starts_with("config.planner.") {
                let (_, value) = facts(&self.planner, &self.settings, self.read_mission)
                    .0
                    .into_iter()
                    .find(|(key, _)| key == fact)
                    .unwrap_or_else(|| panic!("{fact} is not recorded"));
                return Some(value);
            }
            let (saves, saved) = self.settings.saves();
            match fact {
                "config.saves" => Some(saves.to_string()),
                "config.saved" => Some(saved.map_or("none", SaveEvent::label).to_owned()),
                // Every save the model made was `expect`ed to succeed.
                "config.error" => Some("none".to_owned()),
                "config.page" | "config.title" => None,
                "status" => Some(self.status.clone().unwrap_or_default()),
                // `video_tick`'s: the size of the picture under the HUD.
                "hud.camera" => Some(camera_fact(
                    self.capture
                        .as_ref()
                        .and_then(mp_video::Capture::latest)
                        .as_deref(),
                )),
                // `video_tick`'s: `hudon`, and whether the instruments are painted with it.
                "hud.hudon" => Some(self.planner.hud_on().to_string()),
                "hud.instruments" => {
                    let camera = self
                        .capture
                        .as_ref()
                        .and_then(mp_video::Capture::latest)
                        .is_some();
                    Some(crate::hud::draws_instruments(camera, self.planner.hud_on()).to_string())
                }
                fact => {
                    let key = fact
                        .strip_prefix("config.")
                        .filter(|key| crate::settings::PUBLISHED.contains(key))
                        .unwrap_or_else(|| panic!("{fact} is not published"));
                    Some(self.settings.get(key).unwrap_or("none").to_owned())
                }
            }
        }
    }

    /// A script of this page, step for step, against the model, with `source`'s cameras:
    /// returns how many facts it checked and how many restarts it made.
    /// A script line as this platform's runner takes it: a `linux:` or `windows:` line without
    /// its prefix where it is that platform's, `None` where it is another's; any other line as
    /// it is.
    fn for_this_platform(line: &str) -> Option<&str> {
        for (prefix, here) in [
            ("linux:", cfg!(target_os = "linux")),
            ("windows:", cfg!(windows)),
        ] {
            if let Some(rest) = line.strip_prefix(prefix) {
                return here.then(|| rest.trim_start());
            }
        }
        Some(line)
    }

    #[test]
    fn a_platform_line_runs_on_its_platform_alone() {
        let linux = for_this_platform("linux: expect a ~ (/dev/video");
        let windows = for_this_platform("windows: setup exit 3");
        if cfg!(target_os = "linux") {
            assert_eq!(linux, Some("expect a ~ (/dev/video"));
            assert_eq!(windows, None);
        }
        if cfg!(windows) {
            assert_eq!(linux, None);
            assert_eq!(windows, Some("setup exit 3"));
        }
        assert_eq!(for_this_platform("expect b 1"), Some("expect b 1"));
    }

    fn run_script(script: &str, source: &FakeSource) -> (usize, usize) {
        // Two script tests run at once; each needs a scratch directory of its own, or one
        // removes the other's config.xml between its write and its read.
        run_script_in(
            script,
            source,
            if source.devices().is_empty() {
                "script-planner"
            } else {
                "script-video"
            },
        )
    }

    fn run_script_in(script: &str, source: &FakeSource, scratch: &str) -> (usize, usize) {
        assert!(
            script
                .lines()
                .any(|line| line == "env MP_CONFIG_XML $WORK/config.xml"),
            "the script has a config.xml of its own"
        );
        let scratch = Scratch::new(scratch);
        let path = scratch.config();
        // `env XDG_DATA_HOME $WORK`: a data directory, and so a log directory, that do not exist.
        let logs = scratch.0.join("MissionPlannerRust").join("logs");
        let cache = scratch.0.join("gmapcache");
        let mut run = Run::start(&path, &logs, source);
        let (mut checked, mut restarts) = (0, 0);
        for (number, line) in script.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            let at = number + 1;
            // A line for one platform, `linux: expect ...`, is this platform's without its
            // prefix and not run on another: the runners' rule (tools/gui-test.sh).
            let Some(line) = for_this_platform(line) else {
                continue;
            };
            if line == "restart" {
                // `MainV2_FormClosing` on the CONFIG screen: `SaveConfig`, then a new process.
                run.settings.save_config(SaveEvent::Close).expect("saved");
                run = Run::start(&path, &logs, source);
                restarts += 1;
                continue;
            }
            let Some((verb, rest)) = line.split_once(' ') else {
                assert!(line.is_empty(), "line {at}: {line}");
                continue;
            };
            match (verb, rest) {
                // The runner's own lines: where the run starts, its budget, one expect's wait,
                // and what it runs before the application.
                ("screen" | "window" | "env" | "budget" | "within" | "setup", _) => {}
                ("settle", seconds) => run.settle(seconds.parse().expect("seconds")),
                // FLIGHT DATA: the page is hidden with its screen, then `SaveConfig`.
                ("click", "tab-fly") => {
                    run.planner.deactivate();
                    run.settings
                        .save_config(SaveEvent::FlightData)
                        .expect("saved");
                }
                // CONFIG again: the list opens its first page, and `ActivatePage` activates it.
                ("click", "tab-config") => {
                    run.planner
                        .activate(&mut run.settings, Some(logs.as_path()));
                }
                ("click", id) => {
                    run.click(id, &cache);
                    run.effects();
                }
                ("key", key_name) => {
                    let event = match key_name {
                        "Return" => key("enter", None),
                        chorded => chord(
                            chorded
                                .strip_prefix("ctrl+")
                                .unwrap_or_else(|| panic!("line {at}: key {chorded}")),
                        ),
                    };
                    assert!(
                        run.planner.prompt_key(&event, &mut run.settings),
                        "line {at}: no box took {key_name}"
                    );
                }
                ("type", text) => {
                    for character in text.chars() {
                        let character = character.to_string();
                        assert!(
                            run.planner
                                .prompt_key(&key(&character, Some(&character)), &mut run.settings),
                            "line {at}: no box took {text}"
                        );
                    }
                }
                ("expect", rest) => {
                    let (fact, want) = rest.split_once(' ').expect("expect key value");
                    let Some(mut got) = run.fact(fact) else {
                        continue;
                    };
                    let holds = |got: &str| -> bool {
                        if let Some(part) = want.strip_prefix("~ ") {
                            got.contains(part)
                        } else if let Some(least) = want.strip_prefix("> ") {
                            let got: f64 = got.parse().expect("a number");
                            let least: f64 = least.parse().expect("a number");
                            got > least
                        } else {
                            got == want
                        }
                    };
                    // The runner polls a fact for up to ten seconds; the model waits the same
                    // way, a second at a time, for what the capture thread has yet to do.
                    let mut waited = 0;
                    while !holds(&got) && waited < 10 {
                        run.settle(1.0);
                        got = run.fact(fact).expect("the fact was there");
                        waited += 1;
                    }
                    assert!(holds(&got), "line {at}: {fact} is {got}, expected {want}");
                    checked += 1;
                }
                _ => panic!("line {at}: {line} is not modelled"),
            }
        }
        (checked, restarts)
    }

    /// `tests/gui/config-planner.gui`, step for step, against the model: the page's handlers over
    /// a `config.xml` of the script's own, the saves `main.rs` makes - at start-up, on the FLIGHT
    /// DATA button and on the close box - and the restart. Every fact the script expects of the
    /// page, of the dictionary and of its saves is what the model publishes at that step, so
    /// what the script expects after the restart is what the model read back from the file. The
    /// script runs with a window; this does not.
    #[test]
    fn the_gui_script_expects_what_the_model_does() {
        let script = include_str!("../../../../tests/gui/config-planner.gui");
        let (checked, restarts) = run_script(script, &FakeSource::empty());
        assert_eq!(restarts, 1);
        assert!(checked > 80, "{checked} facts checked");
    }

    /// `tests/gui/config-video.gui`, step for step, against the model with a scripted webcam
    /// laid out as this machine's: what the script expects of the real one, the model does of
    /// the scripted one - the lists, the capture's frames, the settings, the picture under the
    /// HUD and its going on Stop.
    #[test]
    fn the_video_script_expects_what_the_model_does() {
        let script = include_str!("../../../../tests/gui/config-video.gui");
        let (checked, restarts) = run_script(script, &FakeSource::webcam());
        assert_eq!(restarts, 0);
        assert!(checked > 15, "{checked} facts checked");
    }

    /// The layout is the `.resx`'s: every control this page draws is one the C# page has, at
    /// the `Location` and `Size` its `.resx` gives.
    #[test]
    fn every_control_is_where_the_resx_puts_it() {
        let Some(text) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigPlanner.resx")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let resx = crate::config_coverage::source::resx(&text);
        let geometry = |name: &str| {
            let pair = |property: &str| -> (u16, u16) {
                let value = resx
                    .get(&format!("{name}.{property}"))
                    .unwrap_or_else(|| panic!("{name}.{property}"));
                let (a, b) = value.split_once(',').expect("a pair");
                (a.trim().parse().unwrap(), b.trim().parse().unwrap())
            };
            let ((x, y), (width, height)) = (pair("Location"), pair("Size"));
            (x, y, width, height)
        };
        let mut checked = 0;
        for spec in CHECKS {
            assert_eq!(spec.at, geometry(spec.name), "{}", spec.name);
            assert_eq!(
                resx.get(&format!("{}.Text", spec.name)).map(String::as_str),
                Some(spec.text),
                "{}",
                spec.name
            );
            checked += 1;
        }
        for spec in COMBOS {
            assert_eq!(spec.at, geometry(spec.name), "{}", spec.name);
            checked += 1;
        }
        for (name, at, text, _) in BUTTONS {
            assert_eq!(*at, geometry(name), "{name}");
            assert_eq!(
                resx.get(&format!("{name}.Text")).map(String::as_str),
                Some(*text),
                "{name}"
            );
            checked += 1;
        }
        for (name, at, _) in NUMBERS {
            assert_eq!(at, geometry(name), "{name}");
            checked += 1;
        }
        // Every label's text is one the `.resx` has, at its place.
        for (x, y, width, height, text) in LABELS {
            let found = resx.iter().any(|(key, value)| {
                key.ends_with(".Text")
                    && value.trim_end() == *text
                    && key.strip_suffix(".Text").is_some_and(|name| {
                        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                        let at = (*x as u16, *y as u16, *width as u16, *height as u16);
                        resx.contains_key(&format!("{name}.Location")) && geometry(name) == at
                    })
            });
            assert!(found, "{text}");
            checked += 1;
        }
        // Each rate combo's items are the Designer's.
        for spec in COMBOS
            .iter()
            .filter(|spec| spec.name.starts_with("CMB_rate"))
        {
            let items: Vec<&str> = (0..)
                .map(|index| {
                    if index == 0 {
                        format!("{}.Items", spec.name)
                    } else {
                        format!("{}.Items{index}", spec.name)
                    }
                })
                .map_while(|key| resx.get(&key).map(String::as_str))
                .collect();
            assert_eq!(items, spec.items, "{}", spec.name);
        }
        // Every control the Designer declares is drawn, or is a label, or is the one hidden.
        let designer = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigPlanner.Designer.cs",
        )
        .expect("the Designer beside the resx");
        let drawn: Vec<&str> = CHECKS
            .iter()
            .map(|spec| spec.name)
            .chain(COMBOS.iter().map(|spec| spec.name))
            .chain(BUTTONS.iter().map(|(name, ..)| *name))
            .chain(NUMBERS.iter().map(|(name, ..)| *name))
            .chain(["txt_log_dir", "CHK_AutoParamCommit"])
            .collect();
        for line in designer.lines() {
            let Some(rest) = line.trim().strip_prefix("this.") else {
                continue;
            };
            let Some((name, value)) = rest.split_once(" = new ") else {
                continue;
            };
            if name.contains('.')
                || name.starts_with("label")
                || value.starts_with("System.ComponentModel")
            {
                continue;
            }
            assert!(drawn.contains(&name), "{name} is not drawn");
        }
        assert!(checked > 90, "{checked}");
    }

    /// Every fact the GUI script asserts is one this page records, and every control it clicks
    /// is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-planner.gui");
        let source = include_str!("planner.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.planner.") => {
                    let name = &key["config.planner.".len()..];
                    let recorded = source.contains(&format!("\"{key}\""))
                        || KEYS.contains(&name)
                        || ["check.", "combo.", "number."].iter().any(|kind| {
                            name.strip_prefix(kind)
                                .is_some_and(|control| source.contains(&format!("\"{control}\"")))
                        });
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("planner-") => {
                    let name = &id["planner-".len()..];
                    let drawn = source.contains(&format!("\"{id}\""))
                        || source.contains(&format!("\"{name}\""))
                        || COMBOS.iter().any(|spec| {
                            spec.items
                                .iter()
                                .any(|item| name == format!("{}-{item}", spec.name))
                        })
                        || ["-up", "-down"].iter().any(|arrow| {
                            name.strip_suffix(arrow)
                                .is_some_and(|base| source.contains(&format!("\"{base}\"")))
                        });
                    assert!(drawn, "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 30 && clicks > 10, "{facts} facts, {clicks} clicks");
    }
}
