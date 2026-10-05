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

//! The plan screen: building a mission and moving it to and from the aircraft.
//!
//! Equivalent to Mission Planner's Flight Plan tab. The plan held here is the operator's, and it
//! is deliberately distinct from whatever the vehicle holds: the two are only equal immediately
//! after a read or a successful write, and pretending otherwise is how people fly a mission they
//! thought they had replaced.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_link::mavftp::{FtpOutcome, FtpRequest};
use mp_mavlink_dialects::all::MavCmd;
use mp_mission::fence::{FenceItem, RallyPoint};
use mp_mission::rows::Home;
use mp_mission::MissionItem;
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::mapview::{self, MapViewport};
use crate::telemetry::TelemetryView;
use crate::textfield::TextField;
use crate::ui::{action, panel, progress, theme};

/// `MAV_CMD_NAV_WAYPOINT`, the command a click on the map creates.
pub const CMD_WAYPOINT: u16 = 16;
/// How far the pointer may move between press and release and still count as a click.
///
/// **A divergence from the C#, small and deliberate.** `MainMap_MouseMove` sets `isMouseDraging`
/// on *any* movement - it compares the press position to the current one and returns early only
/// when they are exactly equal, and at any real zoom a one-pixel move is a different latitude. So
/// in Mission Planner, adding a waypoint by clicking requires a click that does not move a single
/// pixel, which is a thing people complain about. Three pixels absorbs a hand on a mouse and is
/// far too small to swallow a deliberate drag. Set to 0.0 to match the original exactly.
pub const CLICK_SLOP: f32 = 3.0;

/// What a release of the left button over the map should do.
///
/// Extracted from the event handler so it can be tested. The decision is four lines of `if`, and
/// four lines of `if` inside a gpui closure is four lines nothing can reach - which is how a rule
/// as load-bearing as "does this click add a waypoint to the mission" ends up verified by looking
/// at a screenshot.
///
/// `// C#: GCSViews/FlightPlanner.cs:7736-7745`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapRelease {
    /// Add a waypoint where the button came up.
    AddWaypoint,
    /// Do nothing: a drag, a grabbed waypoint, or not the planning screen.
    Nothing,
}

/// Decides what a release means.
///
/// `grabbed` is the waypoint the press landed on, if any - `CurentRectMarker` in the C#, and the
/// reason its comment reads "cant add WP in existing rect". `press` is where the button went
/// down, absent if the press was never seen.
#[must_use]
pub fn map_release(
    planning: bool,
    grabbed: Option<u16>,
    press: Option<(f32, f32)>,
    release: (f32, f32),
) -> MapRelease {
    if !planning || grabbed.is_some() {
        return MapRelease::Nothing;
    }
    // No recorded press means the button went down somewhere else and came up here - dragging in
    // from off the map, or a press the window never saw. Adding a waypoint for it would put one
    // wherever a stray release landed.
    let Some((press_x, press_y)) = press else {
        return MapRelease::Nothing;
    };
    let moved =
        (release.0 - press_x).abs() > CLICK_SLOP || (release.1 - press_y).abs() > CLICK_SLOP;
    if moved {
        MapRelease::Nothing
    } else {
        MapRelease::AddWaypoint
    }
}

/// `MAV_FRAME_GLOBAL_RELATIVE_ALT`: altitude above home, which is what every pilot means.
pub const FRAME_RELATIVE: u8 = 3;
/// `MAV_FRAME_GLOBAL`: altitude above mean sea level. What the home item always uses.
pub const FRAME_ABSOLUTE: u8 = 0;
/// `MAV_FRAME_GLOBAL_TERRAIN_ALT`: altitude above the ground beneath the waypoint.
///
/// The one that changes what can be flown. A mission at 50 m relative, planned over flat ground
/// and flown over a hill, is a mission into the hill; the same mission in this frame clears it.
/// It needs terrain data on the vehicle - `TERRAIN_ENABLE`, and either an SD card of SRTM tiles
/// or a ground station feeding `TERRAIN_DATA` - and a vehicle without it will refuse the mission
/// rather than guess.
pub const FRAME_TERRAIN: u8 = 10;

/// Which altitude frame new items are created in.
///
/// Mission Planner's `altmode`, and the same three values:
///
/// ```csharp
/// public enum altmode {
///     Relative = MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
///     Absolute = MAVLink.MAV_FRAME.GLOBAL,
///     Terrain  = MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT
/// }
/// ```
///
/// A per-screen choice, not a per-item one: `CMB_altmode` sets the frame a *new* row gets
/// (`FlightPlanner.cs:2347`), and every row keeps its own afterwards. So a mission can mix frames
/// and usually does not.
/// `// C#: GCSViews/FlightPlanner.cs:416-421`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AltitudeFrame {
    /// Above home. What almost every mission uses.
    #[default]
    Relative,
    /// Above mean sea level.
    Absolute,
    /// Above the ground beneath the waypoint.
    Terrain,
}

impl AltitudeFrame {
    /// The `MAV_FRAME` value.
    #[must_use]
    pub const fn mav_frame(self) -> u8 {
        match self {
            Self::Relative => FRAME_RELATIVE,
            Self::Absolute => FRAME_ABSOLUTE,
            Self::Terrain => FRAME_TERRAIN,
        }
    }

    /// The frame a `MAV_FRAME` value names, if it is one of the three a mission uses.
    #[must_use]
    pub const fn from_mav_frame(frame: u8) -> Option<Self> {
        match frame {
            FRAME_RELATIVE => Some(Self::Relative),
            FRAME_ABSOLUTE => Some(Self::Absolute),
            FRAME_TERRAIN => Some(Self::Terrain),
            _ => None,
        }
    }

    /// What to call it on screen.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Relative => "relative",
            Self::Absolute => "absolute",
            Self::Terrain => "terrain",
        }
    }

    /// What `CMB_altmode` lists it as: the enum's name (`EnumTranslator.EnumToList<altmode>()`).
    /// `// C#: GCSViews/FlightPlanner.cs:234-236, 416-421`
    #[must_use]
    pub const fn combo_text(self) -> &'static str {
        match self {
            Self::Relative => "Relative",
            Self::Absolute => "Absolute",
            Self::Terrain => "Terrain",
        }
    }

    /// The name the settings file stores, and reads back.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Relative => "relative",
            Self::Absolute => "absolute",
            Self::Terrain => "terrain",
        }
    }

    /// The frame a stored name means, defaulting to relative.
    ///
    /// A setting file that has been hand-edited into nonsense gives the frame every mission uses,
    /// rather than refusing to start or silently choosing terrain - which a vehicle without
    /// terrain data will reject at upload time, long after the choice was made.
    #[must_use]
    pub fn from_key(key: &str) -> Self {
        match key {
            "absolute" => Self::Absolute,
            "terrain" => Self::Terrain,
            _ => Self::Relative,
        }
    }

    /// All three, in the order `CMB_altmode` lists them.
    #[must_use]
    pub const fn all() -> [Self; 3] {
        [Self::Relative, Self::Absolute, Self::Terrain]
    }
}

/// A short label for an item's frame, for the mission list.
///
/// A frame the three-value enum does not cover is shown as its number rather than hidden: a
/// mission read back from a vehicle can hold `MAV_FRAME_LOCAL_NED` or anything else, and showing
/// nothing would say the item is relative when it is not.
#[must_use]
pub fn frame_label(frame: u8) -> String {
    AltitudeFrame::from_mav_frame(frame).map_or_else(
        || format!("frame {frame}"),
        |known| known.label().to_owned(),
    )
}
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
    /// Add a vertex to the geofence.
    Fence,
    /// Place a rally point.
    Rally,
}

impl DrawMode {
    /// The modes, in the order they are offered.
    pub const ALL: [Self; 4] = [Self::Waypoints, Self::Area, Self::Fence, Self::Rally];

    /// What the button says.
    pub const fn label(self) -> &'static str {
        match self {
            Self::Waypoints => "waypoints",
            Self::Area => "survey area",
            Self::Fence => "geofence",
            Self::Rally => "rally points",
        }
    }

    /// The id a test script clicks it by.
    pub const fn id(self) -> &'static str {
        match self {
            Self::Waypoints => "draw-waypoints",
            Self::Area => "draw-area",
            Self::Fence => "draw-fence",
            Self::Rally => "draw-rally",
        }
    }

    /// What a click does in this mode, said plainly.
    pub const fn hint(self) -> &'static str {
        match self {
            Self::Waypoints => "click the map to add a waypoint; right-click for the menu",
            Self::Area => "click the map to place the survey area's corners",
            Self::Fence => "click the map to place the fence's corners",
            Self::Rally => "click the map to place a rally point",
        }
    }
}

/// The mission being edited, separate from the vehicle's.
#[derive(Debug, Default)]
pub struct Plan {
    /// The rows of the `Commands` grid: waypoint 1 onwards, numbered from 1.
    ///
    /// Home is not one of them. Mission Planner keeps home in the Home Location boxes and puts it
    /// back at item 0 only when a mission is written, so the first thing clicked on an empty map
    /// is waypoint 1 and never home. See `mp_mission::rows`.
    items: Vec<MissionItem>,
    /// Where the current contents came from, for the header line.
    origin: Origin,
    /// The row the operator has selected, by its number, if any.
    selected: Option<u16>,
    /// What a click on the map does.
    draw_mode: DrawMode,
    /// The survey area, in the order its vertices were drawn.
    polygon: Vec<LatLon>,
    /// Whether the survey area is off the map while its corners are kept: Geo-Fence > Clear
    /// empties `drawnpolygonsoverlay` and leaves `drawnpolygon.Points`, and the corners come back
    /// the next time the polygon is redrawn - by an edit of it.
    /// `// C#: GCSViews/FlightPlanner.cs:2150-2151, 1014-1030`
    polygon_hidden: bool,
    /// The geofence the vehicle must stay inside, in the order its vertices were drawn.
    fence: Vec<LatLon>,
    /// Rally points: where the vehicle goes on a failsafe instead of all the way home.
    rally: Vec<RallyPoint>,
    /// Why the rally points could not be sent, if they could not.
    rally_error: Option<String>,
    /// Why the fence could not be built or sent, if it could not.
    fence_error: Option<String>,
    /// The Home Location boxes, `TXT_homelat`, `TXT_homelng` and `TXT_homealt`: home, as the
    /// planning screen holds it and every write takes it.
    home: HomeBoxes,
    /// `cs.PlannedHomeLocation`: the home the operator set, which the boxes show when the vehicle
    /// has not sent one. Every edit of a box that parses writes through to it.
    planned_home: Home,
    /// The boxes at the head of `panelWaypoints`: WP Radius, Loiter Radius, Default Alt, Spline.
    panel: PanelBoxes,
    /// The geofence's return location, `geofenceoverlay.Markers[0]`, once one is set.
    fence_return: Option<LatLon>,
    /// A row of parameter sets under way - Write's radii, Geo-Fence > Clear's three - one at a
    /// time as the C# makes them.
    writes: Option<ParamWrites>,
    /// What the last row of sets did, parameter by parameter, for the facts.
    write_results: String,
    /// A Write whose upload has not finished, and the sets that follow it.
    pending_write: Option<PendingWrite>,
    /// Rally Points > Upload under way: `RALLY_TOTAL`, then one point at a time.
    rally_upload: Option<RallyUpload>,
    /// What the last Rally Points > Upload did, point by point, for the facts.
    rally_upload_results: String,
    /// Whether Rally Points > Download is waiting for the vehicle's list.
    rally_download: bool,
    /// `MainV2.comPort.MAV.rallypoints`: the rally list the vehicle last sent, which Download
    /// fills and Clear Rally Points empties. The planner's markers are [`Plan::rally`].
    vehicle_rally: Vec<MissionItem>,
    /// `CHK_verifyheight.Checked`: whether `setfromMap` takes a new row's altitude, and a dragged
    /// one's, against the terrain. Clear, as the Designer leaves it.
    /// `// C#: GCSViews/FlightPlanner.Designer.cs:246-250`
    verify_height: bool,
    /// `sethome`: set when the Lat box is entered, cleared by any change to the three boxes; while
    /// it is set, the next click on the map moves home there instead of adding a row.
    /// `// C#: GCSViews/FlightPlanner.cs:136, 566-571, 7003, 7016-7041`
    sethome: bool,
    /// The row being dragged on the map and where it was when the drag began: what the grid's Lat
    /// and Long cells hold until `setfromMap` writes the new position, and what Verify Height
    /// reads the old ground height at.
    drag_origin: Option<(u16, LatLon)>,
    /// `srtm.getAltitude`.
    terrain: Terrain,
    /// `kmlpolygonsoverlay`: what Map Tool > KML Overlay last read onto the map.
    kml_overlay: Option<mp_kml::read::Overlay>,
    /// Whether `FlightData.kmlpolygons` holds its polygons and routes too: Yes to "Do you want
    /// to load this into the flight data screen?".
    kml_on_flight: bool,
    /// The exclusion polygons of the geofence: Fence Exclusion's, and the vehicle's on a read.
    fence_exclusions: Vec<Vec<LatLon>>,
    /// `grid`, `chk_grid.Checked`: the UTM grid over the map.
    grid: bool,
    /// `chk_usemavftp.Checked`, kept as `UseMissionMAVFTP`.
    use_mavftp: bool,
    /// `panelWaypoints` collapsed to `but_mincommands`' height.
    commands_minimised: bool,
    /// `coords1`: the pointer read-out.
    coords: crate::coords::Coords,
    /// A mission going over MAVFTP, Read or Write with the box ticked.
    mission_ftp: Option<MissionFtp>,
    /// How many times a finished Write asked for the home position: `getHomePositionAsync`.
    home_requests: u32,
    /// Geo-Fence > Upload while its altitude boxes are asked.
    fence_ask: Option<FenceUploadAsk>,
    /// Geo-Fence > Upload under way.
    fence_upload: Option<FenceUpload>,
    /// What the last Geo-Fence > Upload said: its refusal, its failure, or "done".
    fence_upload_text: String,
    /// What the last Geo-Fence > Upload's calls did, call by call.
    fence_upload_results: String,
    /// Geo-Fence > Download under way.
    fence_download: Option<FenceDownloading>,
    /// What the last Geo-Fence > Download said: the number of points or items read, or why none.
    fence_download_text: String,
    /// Words for the status line from Geo-Fence > Upload or Download, until the screen takes them.
    fence_say: Option<String>,
}

/// A mission transfer over MAVFTP: `saveWPs`' and `getWPs`' `chk_usemavftp.Checked` branches,
/// a `@MISSION/mission.dat` written or read by `MAVFtp`, with the ordinary transfer as the
/// fallback when the FTP one throws (`catch (Exception ex) { log.Error(ex); }` then on).
/// `// C#: GCSViews/FlightPlanner.cs:3985-4014, 6237-6256`
#[derive(Debug, Clone, PartialEq)]
pub enum MissionFtp {
    /// `ftp.UploadFile("@MISSION/mission.dat", ...)`, `items` being what it carries.
    Uploading {
        /// Whose FTP client.
        vehicle: mp_vehicle::VehicleId,
        /// The list packed into the file, home first.
        items: Vec<MissionItem>,
        /// `Progress`'s last words.
        status: String,
    },
    /// `ftp.GetFile("@MISSION/mission.dat", null, true, 110)`.
    Downloading {
        /// Whose FTP client.
        vehicle: mp_vehicle::VehicleId,
        /// `Progress`'s last words.
        status: String,
    },
    /// The transfer ended, one way or the other; `text` says how.
    Ended {
        /// Whether it succeeded.
        ok: bool,
        /// What is shown for it.
        text: String,
    },
}

impl MissionFtp {
    /// The file `saveWPs` and `getWPs` name for the mission.
    pub const MISSION_FILE: &'static str = "@MISSION/mission.dat";

    /// The words for the status line.
    #[must_use]
    pub fn label(&self) -> String {
        match self {
            Self::Uploading { status, .. } => {
                if status.is_empty() {
                    "MAVFTP: writing the mission".to_owned()
                } else {
                    format!("MAVFTP: {status}")
                }
            }
            Self::Downloading { status, .. } => {
                if status.is_empty() {
                    "MAVFTP: reading the mission".to_owned()
                } else {
                    format!("MAVFTP: {status}")
                }
            }
            Self::Ended { text, .. } => text.clone(),
        }
    }

    /// Whether the transfer is still going.
    #[must_use]
    pub const fn running(&self) -> bool {
        matches!(self, Self::Uploading { .. } | Self::Downloading { .. })
    }

    /// What the facts call the state.
    #[must_use]
    pub const fn state_name(&self) -> &'static str {
        match self {
            Self::Uploading { .. } => "uploading",
            Self::Downloading { .. } => "downloading",
            Self::Ended { ok: true, .. } => "done",
            Self::Ended { ok: false, .. } => "failed",
        }
    }
}

/// A Write or Write Fast on its way through the handler's questions: the Alt Mode question,
/// then each row's checks in turn with `checkZeroAlts`' question where a row has none.
/// `// C#: GCSViews/FlightPlanner.cs:1755-1830, 1895-1970`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WriteFlow {
    /// `but_writewpfast_Click` rather than `BUT_write_Click`.
    pub fast: bool,
    /// The first row whose checks have not run.
    pub next_row: usize,
}

/// What a Write question's button meant for the flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WriteAnswer {
    /// Yes, or OK: on with it.
    Continue,
    /// No to "Absolute Alt is selected are you sure?": `CMB_altmode.SelectedValue = Relative`,
    /// then on with it.
    RelativeThenContinue,
    /// Cancel to the zero altitude warning: `return`.
    Abort,
}

/// `MAV_CMD.LAST` in Mission Planner's dialect: the commands below it are the navigation ones.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:921`
const MAV_CMD_LAST: u16 = 95;

/// The grid checks both Write buttons run over row `index` before sending: every number column
/// a number ("There are errors in your mission"), and a navigation command's altitude not under
/// Alt Warn ("Low alt on WP#n"), take-off, land and RTL excepted. An empty Alt Warn is 0; a word
/// there is `double.Parse`'s exception, as in the C#.
///
/// # Errors
///
/// The message box's words.
/// `// C#: GCSViews/FlightPlanner.cs:1783-1815`
pub fn write_row_checks(item: &MissionItem, index: usize, alt_warn: &str) -> Result<(), String> {
    let numbers = [
        item.param1,
        item.param2,
        item.param3,
        item.param4,
        item.x,
        item.y,
        item.z,
    ];
    if numbers.iter().any(|number| !number.is_finite()) {
        return Err(MISSION_ERRORS.to_owned());
    }
    let alt_warn = if alt_warn.trim().is_empty() {
        "0"
    } else {
        alt_warn
    };
    let Some(warn) = mp_mission::dotnet::parse_f64(alt_warn) else {
        return Err(format_exception(alt_warn));
    };
    if item.command < MAV_CMD_LAST && item.z < warn && !matches!(item.command, 20..=22) {
        return Err(format!(
            "Low alt on WP#{}\nPlease reduce the alt warning, or increase the altitude",
            index + 1
        ));
    }
    Ok(())
}

/// `checkZeroAlts`: on a plane or a copter, a WAYPOINT, LOITER_TIME, LOITER_UNLIM,
/// LOITER_TURNS or LOITER_TO_ALT at exactly zero altitude gets the firmware's warning with the
/// row's number, and its show-again key.
/// `// C#: GCSViews/FlightPlanner.cs:1831-1873; Common.cs (MessageShowAgain)`
#[must_use]
pub fn zero_alt_warning(
    item: &MissionItem,
    index: usize,
    family: Option<mp_vehicle::VehicleFamily>,
) -> Option<(String, &'static str)> {
    let plane = match family {
        Some(mp_vehicle::VehicleFamily::Plane) => true,
        Some(mp_vehicle::VehicleFamily::Copter) => false,
        _ => return None,
    };
    if item.z != 0.0 || !matches!(item.command, 16 | 19 | 17 | 18 | 31) {
        return None;
    }
    let warning = if plane {
        ZERO_ALT_PLANE
    } else {
        ZERO_ALT_COPTER
    };
    // `SHOWAGAIN_` + the tag with its spaces made underscores.
    let key = if plane {
        "SHOWAGAIN_Zero_Altitude_Warning_Plane"
    } else {
        "SHOWAGAIN_Zero_Altitude_Warning"
    };
    Some((warning.replace("{0}", &(index + 1).to_string()), key))
}

/// `srtm.getAltitude(lat, lng)` as the planning screen asks it: the process's lookup over Mission
/// Planner's `srtm` folder, or a test's own.
#[derive(Clone, Copy)]
pub struct Terrain(pub fn(f64, f64) -> crate::srtm::AltResponse);

impl Default for Terrain {
    /// The process's lookup. Under `cargo test` a plan a test has not given terrain of its own
    /// knows none, so no test reads this machine's `srtm` folder or queues a download from it.
    fn default() -> Self {
        #[cfg(not(test))]
        let terrain = Self(crate::srtm::altitude);
        #[cfg(test)]
        let terrain = Self(|_, _| crate::srtm::AltResponse::INVALID);
        terrain
    }
}

impl std::fmt::Debug for Terrain {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Terrain")
    }
}

impl Terrain {
    /// The ground at a point.
    #[must_use]
    pub fn at(self, lat: f64, lng: f64) -> crate::srtm::AltResponse {
        (self.0)(lat, lng)
    }
}

/// `CurrentState.multiplieralt`: 1, this application being metric throughout.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:37`
const MULTIPLIER_ALT: f32 = crate::fly::MULTIPLIER_ALT;

/// `(int)` of a `double` or a `float` in C#: toward zero. (An out-of-range value, unspecified in
/// C#, saturates here.)
#[allow(clippy::cast_possible_truncation)] // the C#'s cast, on purpose
fn c_sharp_int(value: f64) -> i32 {
    value as i32
}

/// What entering the Lat box says, the trailing space the C#'s own.
/// `// C#: GCSViews/FlightPlanner.cs:7016-7022`
pub const CLICK_TO_SET_HOME: &str = "Click on the Map to set Home ";

/// The command set `readCMDXML` reads from `mavcmd.xml` for the firmware: `APM` for a plane,
/// `APRover` for a rover, `AC2` for anything else. Which commands' `Z` column is headed "Alt"
/// there - what `setfromMap` asks of the row before it touches its altitude.
/// `// C#: GCSViews/FlightPlanner.cs:1119, 1170, 2036-2051, 5680-5720; mavcmd.xml`
#[must_use]
pub fn alt_column(command: u16, family: Option<mp_vehicle::VehicleFamily>) -> bool {
    // The `Z` heading of each section, by the `MAV_CMD` it names.
    const AC2: &[u16] = &[36, 179, 201, 195, 21, 19, 18, 17, 94, 82, 22, 16];
    const APM: &[u16] = &[30, 179, 201, 195, 21, 19, 31, 18, 17, 94, 22, 85, 84, 16];
    const AP_ROVER: &[u16] = &[179, 201, 195, 19, 18, 17, 16];
    let section = match family {
        Some(mp_vehicle::VehicleFamily::Plane) => APM,
        Some(mp_vehicle::VehicleFamily::Rover) => AP_ROVER,
        _ => AC2,
    };
    section.contains(&command)
}

/// Which of the three Home Location boxes.
///
/// `// C#: GCSViews/FlightPlanner.Designer.cs:320-372; GCSViews/FlightPlanner.resx (panel1)`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HomeBox {
    /// `TXT_homelat`, labelled "Lat".
    Lat,
    /// `TXT_homelng`, labelled "Long".
    Lng,
    /// `TXT_homealt`, labelled "ASL".
    Alt,
}

impl HomeBox {
    /// The three, top to bottom as `panel1` stacks them.
    pub const ALL: [Self; 3] = [Self::Lat, Self::Lng, Self::Alt];

    /// The label beside the box: `Label1`, `label2` and `label3` in the `.resx`.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Lat => "Lat",
            Self::Lng => "Long",
            Self::Alt => "ASL",
        }
    }

    /// The id a test script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Lat => "plan-home-lat",
            Self::Lng => "plan-home-lng",
            Self::Alt => "plan-home-alt",
        }
    }
}

/// The three boxes' text, which is what home is on this screen until something parses it.
#[derive(Debug)]
pub struct HomeBoxes {
    lat: TextField,
    lng: TextField,
    alt: TextField,
}

impl HomeBoxes {
    const fn get(&self, which: HomeBox) -> &TextField {
        match which {
            HomeBox::Lat => &self.lat,
            HomeBox::Lng => &self.lng,
            HomeBox::Alt => &self.alt,
        }
    }

    const fn get_mut(&mut self, which: HomeBox) -> &mut TextField {
        match which {
            HomeBox::Lat => &mut self.lat,
            HomeBox::Lng => &mut self.lng,
            HomeBox::Alt => &mut self.alt,
        }
    }
}

impl Default for HomeBoxes {
    /// Empty, as the `.resx` leaves them: no home until one is typed, set or sent.
    fn default() -> Self {
        Self {
            lat: TextField::new(""),
            lng: TextField::new(""),
            alt: TextField::new(""),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The planning panel's boxes: WP Radius, Loiter Radius, Default Alt, and the Spline check box.
// ---------------------------------------------------------------------------------------------

/// One of the three number boxes at the head of `panelWaypoints`, left to right as the `.resx`
/// places them, each with its label above it.
/// `// C#: GCSViews/FlightPlanner.resx (LBL_WPRad, TXT_WPRad, label5, TXT_loiterrad, LBL_defalutalt, TXT_DefaultAlt)`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanelBox {
    /// `TXT_WPRad`: how near a waypoint counts as reaching it.
    WpRadius,
    /// `TXT_loiterrad`: the loiter circle's radius.
    LoiterRadius,
    /// `TXT_DefaultAlt`: the altitude a new row gets.
    DefaultAlt,
    /// `TXT_altwarn`, "Alt Warn": the height under which Write refuses a navigation command.
    /// `// C#: GCSViews/FlightPlanner.resx (label17, TXT_altwarn); GCSViews/FlightPlanner.cs:1826-1843`
    AltWarn,
}

impl PanelBox {
    /// The four, left to right.
    pub const ALL: [Self; 4] = [
        Self::WpRadius,
        Self::LoiterRadius,
        Self::DefaultAlt,
        Self::AltWarn,
    ];

    /// The label above the box: `LBL_WPRad`, `label5` and `LBL_defalutalt` in the `.resx`.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::WpRadius => "WP Radius",
            Self::LoiterRadius => "Loiter Radius",
            Self::DefaultAlt => "Default Alt",
            Self::AltWarn => "Alt Warn",
        }
    }

    /// The id a test script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::WpRadius => "plan-wprad",
            Self::LoiterRadius => "plan-loiterrad",
            Self::DefaultAlt => "plan-defaultalt",
            Self::AltWarn => "plan-altwarn",
        }
    }

    /// The box's `Text` in the `.resx`, what it holds before anything is loaded.
    /// `// C#: GCSViews/FlightPlanner.resx (TXT_WPRad.Text, TXT_loiterrad.Text, TXT_DefaultAlt.Text)`
    #[must_use]
    pub const fn resx_text(self) -> &'static str {
        match self {
            Self::WpRadius => "30",
            Self::LoiterRadius => "45",
            Self::DefaultAlt => "100",
            Self::AltWarn => "0",
        }
    }

    /// The key `config(true)` saves the box under in `config.xml`, and `config(false)` reads.
    /// `// C#: GCSViews/FlightPlanner.cs:2581-2585, 2598-2608`
    #[must_use]
    pub const fn config_key(self) -> &'static str {
        match self {
            Self::WpRadius => "TXT_WPRad",
            Self::LoiterRadius => "TXT_loiterrad",
            Self::DefaultAlt => "TXT_DefaultAlt",
            // `config(true)`: `Settings.Instance["fpminaltwarning"] = TXT_altwarn.Text`.
            // `// C#: GCSViews/FlightPlanner.cs:2600, 2622-2624`
            Self::AltWarn => "fpminaltwarning",
        }
    }

    /// `TXT_*_KeyPress`: whether a typed character goes in. The three handlers let backspace
    /// through and otherwise keep a character only when `float.TryParse` takes it on its own,
    /// which is a digit; WP Radius lets a `.` through as well ("Allow floating values to be
    /// set") and Loiter Radius a `-`, for a loiter the other way round.
    /// `// C#: GCSViews/FlightPlanner.cs:6984-6990, 7054-7064, 7075-7085`
    #[must_use]
    pub fn accepts(self, character: char) -> bool {
        if character.is_ascii_digit() {
            return true;
        }
        match self {
            Self::WpRadius => character == '.',
            Self::LoiterRadius => character == '-',
            Self::DefaultAlt => false,
            // `TXT_altwarn` has no KeyPress handler: it takes what is typed and `double.Parse`
            // reads it at Write.
            Self::AltWarn => character == '.' || character == '-',
        }
    }
}

/// The boxes' text, and what goes with them.
#[derive(Debug)]
pub struct PanelBoxes {
    wp_radius: TextField,
    loiter_radius: TextField,
    default_alt: TextField,
    alt_warn: TextField,
    /// `startupWPradius`: what an emptied WP Radius goes back to - the saved value, else "5.0".
    /// `// C#: GCSViews/FlightPlanner.cs:118, 2600-2601`
    startup_wp_radius: String,
    /// `TXT_loiterrad.Enabled`, which `setWPParams` turns off unless the vehicle has a loiter
    /// radius parameter.
    loiter_enabled: bool,
    /// `CHK_splinedefault.Checked`, the C#'s `splinemode`.
    spline: bool,
}

impl PanelBoxes {
    const fn get(&self, which: PanelBox) -> &TextField {
        match which {
            PanelBox::WpRadius => &self.wp_radius,
            PanelBox::LoiterRadius => &self.loiter_radius,
            PanelBox::DefaultAlt => &self.default_alt,
            PanelBox::AltWarn => &self.alt_warn,
        }
    }

    const fn get_mut(&mut self, which: PanelBox) -> &mut TextField {
        match which {
            PanelBox::WpRadius => &mut self.wp_radius,
            PanelBox::LoiterRadius => &mut self.loiter_radius,
            PanelBox::DefaultAlt => &mut self.default_alt,
            PanelBox::AltWarn => &mut self.alt_warn,
        }
    }
}

impl Default for PanelBoxes {
    /// As the `.resx` has them: 30, 45 and 100, Loiter Radius enabled, Spline clear.
    fn default() -> Self {
        let field = |which: PanelBox| {
            let mut field = TextField::new("");
            field.set(which.resx_text());
            field
        };
        Self {
            wp_radius: field(PanelBox::WpRadius),
            loiter_radius: field(PanelBox::LoiterRadius),
            default_alt: field(PanelBox::DefaultAlt),
            alt_warn: field(PanelBox::AltWarn),
            startup_wp_radius: "5.0".to_owned(),
            loiter_enabled: true,
            spline: false,
        }
    }
}

/// `string.Format("{0:N2}", value)` in the invariant culture's shape: two decimals, the half
/// rounded away from zero, and the thousands grouped with commas.
///
/// The .NET Framework formats a `double` from its fifteen significant digits and rounds that
/// decimal, not the binary value: 2.345 is stored a hair below itself and is still "2.35".
#[must_use]
pub fn number_n2(value: f64) -> String {
    let hundredths = hundredths_from_fifteen_digits(value.abs()).unwrap_or(0);
    let (whole, fraction) = (hundredths / 100, hundredths % 100);
    let digits = whole.to_string();
    let mut grouped = String::new();
    for (index, digit) in digits.chars().enumerate() {
        if index > 0 && (digits.len() - index) % 3 == 0 {
            grouped.push(',');
        }
        grouped.push(digit);
    }
    let sign = if value < 0.0 && hundredths > 0 {
        "-"
    } else {
        ""
    };
    format!("{sign}{grouped}.{fraction:02}")
}

/// A non-negative number in hundredths, from its fifteen significant digits, the half rounded
/// up. `None` for one too large to count.
fn hundredths_from_fifteen_digits(value: f64) -> Option<u128> {
    if !value.is_finite() || value == 0.0 {
        return Some(0);
    }
    let scientific = format!("{value:.14e}");
    let (mantissa, exponent) = scientific.split_once('e')?;
    let exponent = exponent.parse::<i64>().ok()?;
    let digits: Vec<u8> = mantissa
        .bytes()
        .filter(u8::is_ascii_digit)
        .map(|digit| digit - b'0')
        .collect();
    // The digits kept are those down to the hundredths: the integer digits and two more.
    let kept = exponent + 1 + 2;
    let mut hundredths: u128 = 0;
    for index in 0..kept.max(0) {
        let digit = usize::try_from(index)
            .ok()
            .and_then(|index| digits.get(index))
            .copied()
            .unwrap_or(0);
        hundredths = hundredths.checked_mul(10)?.checked_add(u128::from(digit))?;
    }
    let next = usize::try_from(kept)
        .ok()
        .and_then(|index| digits.get(index))
        .copied()
        .unwrap_or(0);
    if next >= 5 {
        hundredths = hundredths.checked_add(1)?;
    }
    Some(hundredths)
}

/// `double.ToString()`: the shortest text that reads back as the same number, which is what the
/// C# puts in a box when it sets one from a number.
#[must_use]
pub fn double_text(value: f64) -> String {
    format!("{value}")
}

/// `ToString("0.00")`: two decimals, the half rounded away from zero as .NET's custom formats do.
#[must_use]
pub fn two_decimals(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    format!("{rounded:.2}")
}

/// `Strings.ERROR`.
pub const ERROR: &str = "Error";

/// `but_writewpfast_Click`'s refusal of a fence or rally list.
/// `// C#: GCSViews/FlightPlanner.cs:1907-1910`
pub const ONLY_FOR_MISSIONS: &str = "Only available for missions";
/// The Alt Mode question both Write buttons ask with Absolute selected.
/// `// C#: GCSViews/FlightPlanner.cs:1757-1765, 1897-1905`
pub const ALT_MODE_TITLE: &str = "Alt Mode";
pub const ALT_MODE_QUESTION: &str = "Absolute Alt is selected are you sure?";
/// `MessageShowAgain("Measure Dist", ...)`'s setting: the title with its space made an
/// underscore.
/// `// C#: GCSViews/FlightPlanner.cs:2632; Common.cs:268`
pub const MEASURE_DIST_KEY: &str = "SHOWAGAIN_Measure_Dist";

/// `MessageShowAgain("FlightPlan Fence", ...)`, shown each time the mission type is set to
/// FENCE, and its setting.
/// `// C#: GCSViews/FlightPlanner.cs:2190-2199; Common.cs:268`
pub const FENCE_TITLE: &str = "FlightPlan Fence";
/// Its setting.
pub const FENCE_KEY: &str = "SHOWAGAIN_FlightPlan_Fence";
/// Its text.
pub const FENCE_TEXT: &str = "Please use the Polygon drawing tool to draw Inclusion and Exclusion areas (round circle to the left), once drawn use the same icon to convert it to a inclusion or exclusion fence";

/// `Strings.ZeroAltWarningTitle`, and the two warnings with their `{0}`.
/// `// C#: ExtLibs/Strings/Strings.resx (ZeroAltWarningTitle, ZeroAltWarningCopter, ZeroAltWarningPlane)`
pub const ZERO_ALT_TITLE: &str = "Zero Altitude Warning";
pub const ZERO_ALT_COPTER: &str = "WP# {0} has zero altitude, this means no altitude change! If you want zero altitude, change it to 0.01. Do you want to continue or cancel wp upload?";
pub const ZERO_ALT_PLANE: &str = "WP# {0} has zero altitude. If you actually want zero altitude, change it to 0.01 instead for predictable behavior.\n\nOn ArduPlane, zero altitudes may interpreted in two different ways: actual zero altitude, or keep current altitude. WAYPOINT commands will generally honor a zero altitude and LOITER commands will generally interpret it as \"use current altitude\", but only if the altitude frame is \"relative\". There may be more exceptions and this could change in future versions.\n\nONLY SET TO ZERO IF YOU REALLY UNDERSTAND EXACTLY WHAT IS GOING TO HAPPEN IN YOUR CASE (use a simulator to be sure).[link;https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html;Command Documentation]";
/// The write handlers' grid checks.
/// `// C#: GCSViews/FlightPlanner.cs:1783-1815`
pub const MISSION_ERRORS: &str = "There are errors in your mission";
/// What `BUT_write_Click` says when the Home Location boxes do not parse.
/// `// C#: GCSViews/FlightPlanner.cs:670`
pub const HOME_INVALID: &str = "Your home location is invalid";
/// What the Home Location link says without a GPS position to take.
/// `// C#: GCSViews/FlightPlanner.cs:4297-4298`
pub const HOME_NEEDS_A_FIX: &str = "If you're at the field, connect to your APM and wait for GPS lock. Then click 'Home Location' link to set home to your location";
/// `MAV_AUTOPILOT_ARDUPILOTMEGA`: the one autopilot `saveWPs` puts home in front for.
pub const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;
/// What `setfromMap` says when Default Alt is not a whole number.
/// `// C#: GCSViews/FlightPlanner.cs:1182-1186`
pub const DEFAULT_ALT_INVALID: &str = "Your default alt is not valid";
/// What `Activate` says when Default Alt is not a whole number, before putting 50 in it.
/// `// C#: GCSViews/FlightPlanner.cs:329-337`
pub const DEFAULT_ALT_FIX: &str = "Please fix your default alt value";
/// `FormatException`'s message, which is what `float.Parse` on a WP Radius it cannot read ends
/// the write's "Setting params" with.
pub const FORMAT_EXCEPTION: &str = "Input string was not in a correct format.";
/// What Geo-Fence > Save to File says without a return location.
/// `// C#: GCSViews/FlightPlanner.cs:5961-5965`
pub const SET_RETURN_LOCATION: &str = "Please set a return location";
/// What Geo-Fence > Save to File says when writing throws.
/// `// C#: GCSViews/FlightPlanner.cs:6012-6016`
pub const FENCE_FILE_FAILED: &str = "Failed to write fence file";

/// `float.TryParse`, for the Leave handlers: surrounding white space, a sign, a point and an
/// exponent, and nothing a `float` cannot hold - .NET Framework refuses an overflow, where Rust
/// would give infinity.
#[must_use]
pub fn float_parses(text: &str) -> bool {
    text.trim().parse::<f32>().is_ok_and(f32::is_finite)
}

// ---------------------------------------------------------------------------------------------
// Parameter sets in a row, as the C# makes them: one `setParam` after another, each waiting for
// the vehicle's echo with the link's retries.
// ---------------------------------------------------------------------------------------------

/// What a set that goes unanswered does to the rest of its row.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OnTimeout {
    /// The C# throws, or catches and shows a message and returns: the row stops there, and this
    /// is said.
    Stop {
        /// The message box's caption.
        title: &'static str,
        /// Its text.
        text: String,
    },
    /// The C# catches it and carries on.
    CarryOn,
}

/// One `setParam` in a row of them.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamStep {
    /// The names to try, in order: the first the vehicle has takes the value, as
    /// `setParam(string[] paramnames, double value)` does.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1609-1620`
    pub names: Vec<&'static str>,
    /// The value.
    pub value: f64,
    /// What a timeout does.
    pub on_timeout: OnTimeout,
}

impl ParamStep {
    /// A set of one parameter.
    #[must_use]
    pub fn one(name: &'static str, value: f64, on_timeout: OnTimeout) -> Self {
        Self {
            names: vec![name],
            value,
            on_timeout,
        }
    }
}

/// What a row of sets goes on to do once every set in it has been made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfterWrites {
    /// Nothing: Write's radii.
    Nothing,
    /// Geo-Fence > Clear's clearing of the map.
    ClearFence,
    /// Clear Rally Points' clearing of the markers and of `MAV.rallypoints`.
    ClearRally,
}

/// How a row of sets ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WritesEnd {
    /// Every set was made, or passed over as the C# passes it over.
    Done,
    /// A set went unanswered where the C# stops: this is said.
    Stopped {
        /// The caption.
        title: &'static str,
        /// The text.
        text: String,
    },
}

/// A row of `setParam`s, made one at a time: the next is sent only once the last has been
/// answered, as the C#'s blocking calls make them.
///
/// `setParamAsync` returns false without sending for a parameter the vehicle has not listed,
/// true without sending for one that already holds the value, and throws `TimeoutException` when
/// its retries go unanswered (`MAVLinkInterface.cs:1636-1765`); the link's `set_param` ends in
/// the same three ways (`mp_link::requests`), and this decides what each does to the row.
#[derive(Debug, Clone)]
pub struct ParamWrites {
    steps: std::collections::VecDeque<ParamStep>,
    /// The step being made, and which of its names.
    current: Option<(ParamStep, usize)>,
    /// The request carrying the set in flight.
    request: Option<mp_link::RequestId>,
    /// What each set did: `NAME=value` with the value the vehicle echoed, `NAME=value unchanged`
    /// for one it already held, `NAME=unknown` for one it does not have, `NAME=timeout`.
    results: Vec<String>,
    after: AfterWrites,
    end: Option<WritesEnd>,
}

impl ParamWrites {
    /// A row of sets, the first ready to send.
    #[must_use]
    pub fn new(steps: Vec<ParamStep>, after: AfterWrites) -> Self {
        let mut writes = Self {
            steps: steps.into(),
            current: None,
            request: None,
            results: Vec::new(),
            after,
            end: None,
        };
        writes.advance();
        writes
    }

    fn advance(&mut self) {
        if self.current.is_some() || self.end.is_some() {
            return;
        }
        match self.steps.pop_front() {
            Some(step) => self.current = Some((step, 0)),
            None => self.end = Some(WritesEnd::Done),
        }
    }

    /// The set to put on the wire now: none while one is out, or once the row has ended.
    #[must_use]
    pub fn due(&self) -> Option<(&'static str, f64)> {
        if self.request.is_some() || self.end.is_some() {
            return None;
        }
        let (step, index) = self.current.as_ref()?;
        Some((*step.names.get(*index)?, step.value))
    }

    /// The set that is due has gone out, carried by `request`.
    pub const fn sent(&mut self, request: mp_link::RequestId) {
        self.request = Some(request);
    }

    /// The request carrying the set in flight.
    #[must_use]
    pub const fn in_flight(&self) -> Option<mp_link::RequestId> {
        self.request
    }

    /// How the set in flight ended. `None` is a set that could not be sent - no vehicle, or no
    /// link - which the C#, finding no such parameter in an empty list, treats as one the vehicle
    /// does not have.
    pub fn answer(&mut self, outcome: Option<mp_link::requests::RequestOutcome>) {
        use mp_link::requests::RequestOutcome;
        self.request = None;
        let Some((step, index)) = self.current.take() else {
            return;
        };
        let name = step.names.get(index).copied().unwrap_or("");
        match outcome {
            Some(RequestOutcome::Accepted { value }) => {
                let value = value.map_or(step.value, |value| value.as_f64());
                self.results.push(format!("{name}={value}"));
            }
            Some(RequestOutcome::Unchanged) => {
                self.results
                    .push(format!("{name}={} unchanged", step.value));
            }
            None | Some(RequestOutcome::UnknownParameter) => {
                self.results.push(format!("{name}=unknown"));
                if index + 1 < step.names.len() {
                    self.current = Some((step, index + 1));
                    return;
                }
            }
            Some(RequestOutcome::TimedOut) => {
                self.results.push(format!("{name}=timeout"));
                if let OnTimeout::Stop { title, text } = step.on_timeout {
                    self.steps.clear();
                    self.end = Some(WritesEnd::Stopped { title, text });
                    return;
                }
            }
            // A command's answers, which a set does not get.
            Some(RequestOutcome::Rejected(_) | RequestOutcome::Sent) => {
                self.results.push(format!("{name}=sent"));
            }
        }
        self.advance();
    }

    /// How the row ended, once it has.
    #[must_use]
    pub const fn end(&self) -> Option<&WritesEnd> {
        self.end.as_ref()
    }

    /// What it goes on to do.
    #[must_use]
    pub const fn after(&self) -> AfterWrites {
        self.after
    }

    /// What each set did, in order.
    #[must_use]
    pub fn results(&self) -> String {
        self.results.join(",")
    }
}

/// A Write waiting for its upload, and the sets `saveWPs` makes once it is done.
#[derive(Debug, Clone)]
pub struct PendingWrite {
    /// What was sent, to know the transfer that is finishing is this one.
    items: Vec<MissionItem>,
    /// The sets, or why `float.Parse` refused WP Radius.
    steps: Result<Vec<ParamStep>, &'static str>,
    /// Whether the transfer has been seen under way since.
    seen_running: bool,
}

impl PendingWrite {
    /// A write of `items`, to be followed by `steps`.
    #[must_use]
    pub const fn new(items: Vec<MissionItem>, steps: Result<Vec<ParamStep>, &'static str>) -> Self {
        Self {
            items,
            steps,
            seen_running: false,
        }
    }

    /// Whether the upload has ended, and how: `None` while it runs, `Some(true)` once the vehicle
    /// has taken it. The link reports the last transfer it ran, which just after Write can still
    /// be an earlier one; the upload is this one once it has been seen running, or once it holds
    /// exactly what was sent.
    pub fn upload_ended(
        &mut self,
        transfer: Option<&crate::telemetry::TransferStatus>,
        transferred: &[MissionItem],
    ) -> Option<bool> {
        let status = transfer?;
        if !status.finished {
            self.seen_running = true;
            return None;
        }
        if !self.seen_running && transferred != self.items.as_slice() {
            return None;
        }
        Some(!status.failed)
    }
}

/// What Rally Points > Upload puts on the wire next.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RallyDue {
    /// `setParam("RALLY_TOTAL", rallypointoverlay.Markers.Count)`.
    Total(f64),
    /// `setRallyPoint` of the next marker.
    Point(mp_link::requests::RallyPointSet),
}

/// Rally Points > Upload: `saveRallyPointsToolStripMenuItem_Click`, one blocking call at a time
/// as the C# makes them.
///
/// `RALLY_TOTAL` is set to the number of markers first, its outcome not looked at - `setParam`
/// returns false without sending for a vehicle that has not listed it, and the handler goes on.
/// Then each marker in turn goes to `setRallyPoint` with the index counted from 0 and
/// `(byte)(float) MAV.param["RALLY_TOTAL"]` as the count - which throws where the vehicle has not
/// listed `RALLY_TOTAL`, inside the handler's `try`, as a read-back that times out does: both say
/// "Failed to save rally point" and stop. A set that went out and never read back the same is
/// passed over, as the C# ignores `setRallyPoint`'s `false`.
///
/// `RALLY_TOTAL` is set outside that `try`, so a set that times out throws out of the handler
/// altogether; here that is said with the exception's message and the upload stops.
/// `// C#: GCSViews/FlightPlanner.cs:5936-5957`
#[derive(Debug, Clone)]
pub struct RallyUpload {
    /// The markers, as they were when Upload was chosen.
    markers: Vec<(LatLon, i32)>,
    /// The next marker to send, once `RALLY_TOTAL` has been set.
    next: Option<usize>,
    /// The request carrying the call in flight.
    request: Option<mp_link::RequestId>,
    /// What each call did: `RALLY_TOTAL=n` and its outcome, then `index=set|sent` per marker.
    results: Vec<String>,
    end: Option<WritesEnd>,
}

impl RallyUpload {
    /// An upload of `markers`, `RALLY_TOTAL` due first.
    #[must_use]
    pub const fn new(markers: Vec<(LatLon, i32)>) -> Self {
        Self {
            markers,
            next: None,
            request: None,
            results: Vec::new(),
            end: None,
        }
    }

    fn stop(&mut self, title: &'static str, text: impl Into<String>) {
        self.end = Some(WritesEnd::Stopped {
            title,
            text: text.into(),
        });
    }

    /// What to send now, given the vehicle's `RALLY_TOTAL` as its parameter list holds it; none
    /// while a call is out or once the upload has ended. Ends it where the next marker cannot be
    /// sent: all sent, or no `RALLY_TOTAL` to count them by.
    pub fn due(&mut self, rally_total: Option<f64>) -> Option<RallyDue> {
        if self.request.is_some() || self.end.is_some() {
            return None;
        }
        let Some(index) = self.next else {
            #[allow(clippy::cast_precision_loss)] // `rallypointoverlay.Markers.Count`
            return Some(RallyDue::Total(self.markers.len() as f64));
        };
        let Some(&(position, altitude)) = self.markers.get(index) else {
            self.end = Some(WritesEnd::Done);
            return None;
        };
        let Some(total) = rally_total else {
            // `(float) MAV.param["RALLY_TOTAL"]` on a parameter the vehicle has not listed.
            self.stop(ERROR, RALLY_SAVE_FAILED);
            return None;
        };
        // `byte count` counts up from 0; `(int)(plla.Lat * t7)`, `(short) plla.Alt`,
        // `(byte)(float)` of the count: the C#'s casts, which truncate.
        // `// C#: GCSViews/FlightPlanner.cs:5938, 5947-5948; MAVLinkInterface.cs:6447-6451`
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let set = mp_link::requests::RallyPointSet {
            idx: index as u8,
            count: total as f32 as u8,
            lat: (position.latitude() * 1e7) as i32,
            lng: (position.longitude() * 1e7) as i32,
            alt: altitude as i16,
            break_alt: 0,
            land_dir: 0,
            flags: 0,
        };
        Some(RallyDue::Point(set))
    }

    /// The call that is due has gone out, carried by `request`.
    pub const fn sent(&mut self, request: mp_link::RequestId) {
        self.request = Some(request);
    }

    /// The request carrying the call in flight.
    #[must_use]
    pub const fn in_flight(&self) -> Option<mp_link::RequestId> {
        self.request
    }

    /// How the call in flight ended. `None` is one that could not be sent - no vehicle, or no
    /// link: `RALLY_TOTAL` not in the list, and a rally point whose read-back cannot come.
    pub fn answer(&mut self, outcome: Option<mp_link::requests::RequestOutcome>) {
        use mp_link::requests::RequestOutcome;
        self.request = None;
        match self.next {
            None => {
                let total = self.markers.len();
                match outcome {
                    Some(RequestOutcome::TimedOut) => {
                        self.results.push(format!("RALLY_TOTAL={total} timeout"));
                        self.stop(ERROR, "Timeout on read - setParam RALLY_TOTAL");
                        return;
                    }
                    Some(RequestOutcome::Accepted { .. }) => {
                        self.results.push(format!("RALLY_TOTAL={total}"));
                    }
                    Some(RequestOutcome::Unchanged) => {
                        self.results.push(format!("RALLY_TOTAL={total} unchanged"));
                    }
                    _ => self.results.push("RALLY_TOTAL=unknown".to_owned()),
                }
                self.next = Some(0);
            }
            Some(index) => {
                match outcome {
                    Some(RequestOutcome::Accepted { .. }) => {
                        self.results.push(format!("{index}=set"));
                    }
                    None | Some(RequestOutcome::TimedOut) => {
                        self.results.push(format!("{index}=timeout"));
                        self.stop(ERROR, RALLY_SAVE_FAILED);
                        return;
                    }
                    Some(_) => self.results.push(format!("{index}=sent")),
                }
                self.next = Some(index + 1);
            }
        }
    }

    /// How the upload ended, once it has.
    #[must_use]
    pub const fn end(&self) -> Option<&WritesEnd> {
        self.end.as_ref()
    }

    /// What each call did, in order.
    #[must_use]
    pub fn results(&self) -> String {
        self.results.join(",")
    }
}

// ---------------------------------------------------------------------------------------------
// Geo-Fence > Upload and Download: the map menu's geofence, sent and read with the legacy
// `FENCE_POINT` / `FENCE_FETCH_POINT` protocol and the `FENCE_*` parameters, as
// `GeoFenceuploadToolStripMenuItem_Click` and `GeoFencedownloadToolStripMenuItem_Click` do.
// ---------------------------------------------------------------------------------------------

/// `MAV_PROTOCOL_CAPABILITY_MISSION_FENCE` (16384): a vehicle that takes its fence as mission
/// items - every ArduPilot 4.x, the SITL here (capabilities 0xfbef) among them.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:7103`
pub const CAPABILITY_MISSION_FENCE: u32 =
    mp_mavlink_dialects::all::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_FENCE.0;
/// Geo-Fence > Upload and Download on a vehicle without the fence parameters.
/// `// C#: GCSViews/FlightPlanner.cs:850-854, 3718-3724`
pub const FENCE_NOT_SUPPORTED: &str = "Not Supported";
/// `// C#: GCSViews/FlightPlanner.cs:3730-3736`
pub const NO_RETURN_LOCATION: &str = "No return location set";
/// `// C#: GCSViews/FlightPlanner.cs:3738-3742`
pub const NO_POLYGON_DRAWN: &str = "No polygon drawn";
/// `// C#: GCSViews/FlightPlanner.cs:3744-3755`
pub const RETURN_OUTSIDE: &str = "Your return location is outside the polygon";
/// `// C#: GCSViews/FlightPlanner.cs:3769-3773`
pub const BAD_MIN_ALT: &str = "Bad Min Alt";
/// `// C#: GCSViews/FlightPlanner.cs:3785-3789`
pub const BAD_MAX_ALT: &str = "Bad Max Alt";
/// `// C#: GCSViews/FlightPlanner.cs:3800-3806`
pub const FENCE_ALT_FAILED: &str = "Failed to set min/max fence alt";
/// `// C#: GCSViews/FlightPlanner.cs:3815-3819`
pub const FENCE_ACTION_FAILED: &str = "Failed to set FENCE_ACTION";
/// `// C#: GCSViews/FlightPlanner.cs:3830-3834`
pub const FENCE_TOTAL_FAILED: &str = "Failed to set FENCE_TOTAL";
/// `// C#: GCSViews/FlightPlanner.cs:3855-3859`
pub const FENCE_RESTORE_FAILED: &str = "Failed to restore FENCE_ACTION";
/// `// C#: GCSViews/FlightPlanner.cs:856-860`
pub const NOTHING_TO_DOWNLOAD: &str = "Nothing to download";
/// `// C#: GCSViews/FlightPlanner.cs:842-845, 874-878`
pub const FENCE_POINT_FAILED: &str = "Failed to get fence point";
/// `getFencePoint`'s `TimeoutException`, which `DoGeofencePointsUpload` does not catch and the
/// progress reporter shows. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5942`
pub const FENCE_POINT_TIMEOUT: &str = "Timeout on read - getFencePoint";
/// `setFencePoint`'s exception once three sends have not read back the same.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6438`
pub const FENCE_POINT_UNVERIFIED: &str = "Could not verify GeoFence Point";
/// What the progress reporter shows for an exception its work threw, `ErrorMessage` unset as
/// `DoGeofencePointsUpload` leaves it: "There was an unexpected error (" and the message.
/// `// C#: Controls/ProgressReporterDialogue.cs:128, 226`
#[must_use]
pub fn unexpected_error(message: &str) -> String {
    format!("There was an unexpected error ({message})")
}
/// `NullReferenceException`'s message: what `(float) MAV.param["FENCE_ACTION"]` throws on a
/// vehicle that has `FENCE_ENABLE` and not `FENCE_ACTION`.
pub const NULL_REFERENCE: &str = "Object reference not set to an instance of an object.";
/// The progress reporter's words, `frmProgressReporter.UpdateProgressAndStatus(-1, ...)` and
/// `DoGeofencePointsUpload`'s three.
/// `// C#: GCSViews/FlightPlanner.cs:3698, 3704, 3710, 3839-3845`
pub const SENDING_FENCE_POINTS: &str = "Sending fence points";
/// `PRD.UpdateProgressAndStatus(0, "Sending return location")`.
pub const SENDING_RETURN: &str = "Sending return location";
/// `PRD.UpdateProgressAndStatus(a / pointcount * 100, "Sending polygon points")`.
pub const SENDING_POLYGON: &str = "Sending polygon points";
/// `PRD.UpdateProgressAndStatus(a / pointcount * 100, "Sending polygon close")`.
pub const SENDING_CLOSE: &str = "Sending polygon close";

/// `pnpoly`: whether (`testx`, `testy`) - a latitude and a longitude - is inside the polygon
/// `array`, by the even-odd rule, each edge `i`-`j` counted when the point's longitude is between
/// its ends and the point lies below the edge's latitude there. Ported as the C# writes it: the
/// division is by the edge's longitude span, never zero where it is reached, as the first test
/// fails for an edge whose ends share a longitude.
/// `// C#: GCSViews/FlightPlanner.cs:4987-5002`
#[must_use]
pub fn pnpoly(array: &[LatLon], testx: f64, testy: f64) -> bool {
    let mut c = false;
    let Some(mut j) = array.last().copied() else {
        return false;
    };
    for &i in array {
        if ((i.longitude() > testy) != (j.longitude() > testy))
            && (testx
                < (j.latitude() - i.latitude()) * (testy - i.longitude())
                    / (j.longitude() - i.longitude())
                    + i.latitude())
        {
            c = !c;
        }
        j = i;
    }
    c
}

/// A parameter's value as the vehicle's list holds it, by name.
fn listed(parameters: &[(String, f64)], name: &str) -> Option<f64> {
    parameters
        .iter()
        .find(|(listed, _)| listed == name)
        .map(|(_, value)| *value)
}

/// One of Geo-Fence > Upload's altitude boxes: its caption, its question, the text it offers,
/// and which box it is.
pub type FenceQuestion = (&'static str, &'static str, String, PromptKind);

/// Geo-Fence > Upload once its checks have passed, while its altitude boxes are being asked.
///
/// `FENCE_MINALT` and `FENCE_MAXALT` are asked only on a vehicle that lists them (ArduPlane's);
/// ArduCopter has neither, and nothing is asked there.
/// `// C#: GCSViews/FlightPlanner.cs:3761-3793`
#[derive(Debug, Clone, PartialEq)]
pub struct FenceUploadAsk {
    /// `geofenceoverlay.Markers[0].Position`.
    return_point: LatLon,
    /// `drawnpolygon.Points`.
    polygon: Vec<LatLon>,
    /// `FENCE_MINALT` as listed, and the answer once given.
    min_alt: Option<(f64, Option<i32>)>,
    /// `FENCE_MAXALT` likewise.
    max_alt: Option<(f64, Option<i32>)>,
    /// `MAV.param["FENCE_ACTION"]`, `None` where the vehicle lists only `FENCE_ENABLE`.
    old_action: Option<f64>,
}

impl FenceUploadAsk {
    /// The box to show next - its caption, question and the text it offers - or `None` once
    /// every box the vehicle calls for has been answered. The offer is
    /// `(int.Parse(param.ToString()) * CurrentState.multiplieralt).ToString("0")`: a parameter
    /// holding a fraction is `int.Parse`'s `FormatException`.
    /// `// C#: GCSViews/FlightPlanner.cs:3764-3793`
    pub fn next_question(&self) -> Option<Result<FenceQuestion, &'static str>> {
        let offer = |value: f64| -> Result<String, &'static str> {
            if value.fract() != 0.0 || !value.is_finite() {
                return Err(FORMAT_EXCEPTION);
            }
            // int.Parse, then int times float: a float, "0" rounding it to a whole number.
            #[allow(clippy::cast_possible_truncation)]
            let whole = value as i32;
            #[allow(clippy::cast_precision_loss)]
            let times = whole as f32 * MULTIPLIER_ALT;
            Ok(format!("{:.0}", f64::from(times)))
        };
        if let Some((value, None)) = self.min_alt {
            return Some(offer(value).map(|text| {
                (
                    "Min Alt",
                    "Box Minimum Altitude?",
                    text,
                    PromptKind::FenceMinAlt,
                )
            }));
        }
        if let Some((value, None)) = self.max_alt {
            return Some(offer(value).map(|text| {
                (
                    "Max Alt",
                    "Box Maximum Altitude?",
                    text,
                    PromptKind::FenceMaxAlt,
                )
            }));
        }
        None
    }

    /// A box's OK: `int.TryParse`, or "Bad Min Alt" / "Bad Max Alt".
    /// `// C#: GCSViews/FlightPlanner.cs:3769-3773, 3785-3789`
    pub fn answer(&mut self, kind: &PromptKind, text: &str) -> Result<(), &'static str> {
        let parsed = mp_mission::dotnet::parse_i32(text);
        match kind {
            PromptKind::FenceMinAlt => {
                let (value, _) = self.min_alt.ok_or(BAD_MIN_ALT)?;
                self.min_alt = Some((value, Some(parsed.ok_or(BAD_MIN_ALT)?)));
            }
            PromptKind::FenceMaxAlt => {
                let (value, _) = self.max_alt.ok_or(BAD_MAX_ALT)?;
                self.max_alt = Some((value, Some(parsed.ok_or(BAD_MAX_ALT)?)));
            }
            _ => {}
        }
        Ok(())
    }
}

/// `GeoFenceuploadToolStripMenuItem_Click`'s checks, in its order: the fence parameters, a return
/// location, a drawn polygon, and the return location inside it.
///
/// The C#'s second check, "No polygon to upload" for `drawnpolygon == null`, is never true - the
/// constructor makes `drawnpolygon` (`FlightPlanner.cs:278`) and nothing sets it to null - so it is
/// not here. The polygon checked and sent is the drawn polygon's corners whether or not
/// Geo-Fence > Clear took it off the map, as `drawnpolygon.Points` keeps them.
/// `// C#: GCSViews/FlightPlanner.cs:3717-3755`
pub fn fence_upload_checks(
    plan: &Plan,
    parameters: &[(String, f64)],
) -> Result<FenceUploadAsk, &'static str> {
    // FENCE_ENABLE ON COPTER, FENCE_ACTION ON PLANE.
    let action = listed(parameters, "FENCE_ACTION");
    if listed(parameters, "FENCE_ENABLE").is_none() && action.is_none() {
        return Err(FENCE_NOT_SUPPORTED);
    }
    let return_point = plan.fence_return.ok_or(NO_RETURN_LOCATION)?;
    if plan.polygon.is_empty() {
        return Err(NO_POLYGON_DRAWN);
    }
    // The polygon closed by its first corner, and the return location tested against it.
    let mut closed = plan.polygon.clone();
    closed.extend(plan.polygon.first().copied());
    if !pnpoly(&closed, return_point.latitude(), return_point.longitude()) {
        return Err(RETURN_OUTSIDE);
    }
    Ok(FenceUploadAsk {
        return_point,
        polygon: plan.polygon.clone(),
        min_alt: listed(parameters, "FENCE_MINALT").map(|value| (value, None)),
        max_alt: listed(parameters, "FENCE_MAXALT").map(|value| (value, None)),
        old_action: action,
    })
}

/// One of Geo-Fence > Upload's blocking calls.
#[derive(Debug, Clone, PartialEq)]
enum FenceCall {
    /// `setParam(name, value)`, and what a `TimeoutException` from it says.
    Set {
        name: &'static str,
        value: f64,
        on_timeout: &'static str,
    },
    /// `(float) MAV.param["FENCE_ACTION"]` on a vehicle without it.
    NoAction,
    /// `setFencePoint`, and what the progress reporter says as it goes.
    Point {
        set: mp_link::requests::FencePointSet,
        status: &'static str,
    },
}

/// What Geo-Fence > Upload puts on the wire next.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FenceDue {
    /// `setParam`.
    Param(&'static str, f64),
    /// `setFencePoint`.
    Point(mp_link::requests::FencePointSet),
}

/// How Geo-Fence > Upload ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FenceUploadEnd {
    /// `FENCE_ACTION` was put back: the drawn polygon becomes the geofence. `error` is what the
    /// progress reporter showed, if a point failed - the C# goes on to put `FENCE_ACTION` back and
    /// redraw all the same, as `RunBackgroundOperationAsync` returns after showing it.
    Done {
        /// The points' failure, if one failed.
        error: Option<&'static str>,
    },
    /// A `setParam` threw where the handler catches it, says so and returns: nothing redrawn.
    Stopped(&'static str),
}

/// Geo-Fence > Upload under way: `GeoFenceuploadToolStripMenuItem_Click` from its `setParam`s on,
/// and `DoGeofencePointsUpload`, one blocking call at a time as the C# makes them.
///
/// `FENCE_MINALT` and `FENCE_MAXALT` to the answers where the vehicle lists them (the typed
/// number as it is: the C# offers the parameter times `multiplieralt` and does not divide the
/// answer back), `FENCE_ACTION` to 0, `FENCE_TOTAL` to the corners plus two; then `setFencePoint`
/// for the return location, each corner and the first corner again, `count` the corners plus two
/// as a `byte`; then `FENCE_ACTION` back as it was. A set the vehicle does not list is passed
/// over, as `setParam` returns false for it; one that goes unanswered stops the upload with the
/// handler's words. A point that times out or never reads back the same ends the points - the
/// exception leaves `DoGeofencePointsUpload` - and the upload goes on to `FENCE_ACTION`.
/// `// C#: GCSViews/FlightPlanner.cs:3691-3712, 3795-3898`
#[derive(Debug, Clone)]
pub struct FenceUpload {
    calls: std::collections::VecDeque<FenceCall>,
    /// The call in flight, once due.
    current: Option<FenceCall>,
    /// The request carrying it.
    request: Option<mp_link::RequestId>,
    /// The polygon sent, which becomes the geofence.
    polygon: Vec<LatLon>,
    /// The progress reporter's text.
    status: &'static str,
    /// What each call did: `NAME=value`, `NAME=value unchanged`, `NAME=unknown`, `NAME=timeout`;
    /// `index=set`, `index=timeout`, `index=unverified`.
    results: Vec<String>,
    error: Option<&'static str>,
    end: Option<FenceUploadEnd>,
}

impl FenceUpload {
    /// The calls for an upload whose boxes have been answered.
    #[must_use]
    pub fn new(ask: FenceUploadAsk) -> Self {
        let mut calls = std::collections::VecDeque::new();
        if let Some((_, answer)) = ask.min_alt {
            calls.push_back(FenceCall::Set {
                name: "FENCE_MINALT",
                value: f64::from(answer.unwrap_or(0)),
                on_timeout: FENCE_ALT_FAILED,
            });
        }
        if let Some((_, answer)) = ask.max_alt {
            calls.push_back(FenceCall::Set {
                name: "FENCE_MAXALT",
                value: f64::from(answer.unwrap_or(0)),
                on_timeout: FENCE_ALT_FAILED,
            });
        }
        let Some(old_action) = ask.old_action else {
            calls.push_back(FenceCall::NoAction);
            return Self::with(calls, ask.polygon);
        };
        // points + return + close, `(byte)`.
        #[allow(clippy::cast_possible_truncation)]
        let pointcount = (ask.polygon.len() + 2) as u8;
        calls.push_back(FenceCall::Set {
            name: "FENCE_ACTION",
            value: 0.0,
            on_timeout: FENCE_ACTION_FAILED,
        });
        calls.push_back(FenceCall::Set {
            name: "FENCE_TOTAL",
            value: f64::from(pointcount),
            on_timeout: FENCE_TOTAL_FAILED,
        });
        let point = |index: usize, at: LatLon, status: &'static str| {
            // `byte a`, counted up from 0.
            #[allow(clippy::cast_possible_truncation)]
            let idx = index as u8;
            FenceCall::Point {
                set: mp_link::requests::FencePointSet {
                    idx,
                    count: pointcount,
                    lat: at.latitude(),
                    lng: at.longitude(),
                },
                status,
            }
        };
        calls.push_back(point(0, ask.return_point, SENDING_RETURN));
        for (index, corner) in ask.polygon.iter().enumerate() {
            calls.push_back(point(index + 1, *corner, SENDING_POLYGON));
        }
        if let Some(first) = ask.polygon.first() {
            calls.push_back(point(ask.polygon.len() + 1, *first, SENDING_CLOSE));
        }
        calls.push_back(FenceCall::Set {
            name: "FENCE_ACTION",
            value: old_action,
            on_timeout: FENCE_RESTORE_FAILED,
        });
        Self::with(calls, ask.polygon)
    }

    fn with(calls: std::collections::VecDeque<FenceCall>, polygon: Vec<LatLon>) -> Self {
        Self {
            calls,
            current: None,
            request: None,
            polygon,
            status: "",
            results: Vec::new(),
            error: None,
            end: None,
        }
    }

    /// What to send now: none while a call is out, or once the upload has ended.
    pub fn due(&mut self) -> Option<FenceDue> {
        if self.request.is_some() || self.end.is_some() {
            return None;
        }
        if self.current.is_none() {
            let Some(call) = self.calls.pop_front() else {
                self.end = Some(FenceUploadEnd::Done { error: self.error });
                return None;
            };
            self.current = Some(call);
        }
        match self.current.clone()? {
            FenceCall::Set { name, value, .. } => Some(FenceDue::Param(name, value)),
            FenceCall::NoAction => {
                self.current = None;
                self.end = Some(FenceUploadEnd::Stopped(NULL_REFERENCE));
                None
            }
            FenceCall::Point { set, status } => {
                self.status = status;
                Some(FenceDue::Point(set))
            }
        }
    }

    /// The call that is due has gone out, carried by `request`.
    pub const fn sent(&mut self, request: mp_link::RequestId) {
        self.request = Some(request);
    }

    /// The request carrying the call in flight.
    #[must_use]
    pub const fn in_flight(&self) -> Option<mp_link::RequestId> {
        self.request
    }

    /// How the call in flight ended. `None` is one that could not be sent or whose request the
    /// link no longer has - the link gone, as after Disconnect - which stops a set as its timeout
    /// does: the C#'s `setParam` on a closed port finds the name (`Close` keeps `MAV.param`),
    /// sends nothing and times out (`MAVLinkInterface.cs:1262-1264, 1765`). A parameter the list
    /// does not hold is the link's `UnknownParameter`, passed over as `setParam`'s `false` is.
    pub fn answer(&mut self, outcome: Option<mp_link::requests::RequestOutcome>) {
        use mp_link::requests::RequestOutcome;
        self.request = None;
        let Some(call) = self.current.take() else {
            return;
        };
        match call {
            FenceCall::Set {
                name,
                value,
                on_timeout,
            } => match outcome {
                Some(RequestOutcome::Accepted { value: echoed }) => {
                    let value = echoed.map_or(value, |echoed| echoed.as_f64());
                    self.results.push(format!("{name}={value}"));
                }
                Some(RequestOutcome::Unchanged) => {
                    self.results.push(format!("{name}={value} unchanged"));
                }
                None | Some(RequestOutcome::TimedOut) => {
                    self.results.push(format!("{name}=timeout"));
                    self.calls.clear();
                    self.end = Some(FenceUploadEnd::Stopped(on_timeout));
                }
                Some(_) => self.results.push(format!("{name}=unknown")),
            },
            FenceCall::Point { set, .. } => {
                let idx = set.idx;
                let failed = match outcome {
                    Some(RequestOutcome::Accepted { .. }) => {
                        self.results.push(format!("{idx}=set"));
                        None
                    }
                    None | Some(RequestOutcome::TimedOut) => {
                        self.results.push(format!("{idx}=timeout"));
                        Some(FENCE_POINT_TIMEOUT)
                    }
                    Some(_) => {
                        self.results.push(format!("{idx}=unverified"));
                        Some(FENCE_POINT_UNVERIFIED)
                    }
                };
                if let Some(error) = failed {
                    // The exception leaves DoGeofencePointsUpload: no more points.
                    self.error = Some(error);
                    self.calls
                        .retain(|call| !matches!(call, FenceCall::Point { .. }));
                }
            }
            FenceCall::NoAction => {}
        }
    }

    /// The progress reporter's text: the point being sent, or "Sending fence points" before one.
    #[must_use]
    pub fn status(&self) -> &'static str {
        self.progress().unwrap_or(SENDING_FENCE_POINTS)
    }

    /// The progress window's text once it is up: the C# opens it after `FENCE_TOTAL` is set, so
    /// while the parameters go there is none (`FlightPlanner.cs:3837-3847`).
    #[must_use]
    pub fn progress(&self) -> Option<&'static str> {
        (!self.status.is_empty()).then_some(self.status)
    }

    /// The polygon being sent.
    #[must_use]
    pub fn polygon(&self) -> &[LatLon] {
        &self.polygon
    }

    /// How the upload ended, once it has.
    #[must_use]
    pub const fn end(&self) -> Option<&FenceUploadEnd> {
        self.end.as_ref()
    }

    /// What each call did, in order.
    #[must_use]
    pub fn results(&self) -> String {
        self.results.join(",")
    }
}

/// Geo-Fence > Download on a vehicle without `MISSION_FENCE`: `getFencePoint` from 0 while the
/// index is below the count the last point read said, the first read counting as one. The first
/// point is the return location and the rest the polygon - its closing point included, as the C#
/// adds every point read to `geofencepolygon`. A point that is not read ends it: "Failed to get
/// fence point", the geofence already cleared.
/// `// C#: GCSViews/FlightPlanner.cs:827, 862-913`
#[derive(Debug, Clone, Default)]
pub struct FenceDownload {
    /// `a`.
    next: u8,
    /// `count`, 1 until the first point says otherwise.
    count: u8,
    points: Vec<LatLon>,
    request: Option<mp_link::RequestId>,
    end: Option<Result<(), &'static str>>,
}

impl FenceDownload {
    /// A download, point 0 due.
    #[must_use]
    pub fn new() -> Self {
        Self {
            count: 1,
            ..Self::default()
        }
    }

    /// The point to read now: none while one is out, or once the download has ended.
    pub fn due(&mut self) -> Option<u8> {
        if self.request.is_some() || self.end.is_some() {
            return None;
        }
        if self.next >= self.count {
            self.end = Some(Ok(()));
            return None;
        }
        Some(self.next)
    }

    /// The read that is due has gone out, carried by `request`.
    pub const fn sent(&mut self, request: mp_link::RequestId) {
        self.request = Some(request);
    }

    /// The request carrying the read in flight.
    #[must_use]
    pub const fn in_flight(&self) -> Option<mp_link::RequestId> {
        self.request
    }

    /// How the read in flight ended, and the point it read.
    pub fn answer(
        &mut self,
        outcome: Option<mp_link::requests::RequestOutcome>,
        read: Option<mp_link::requests::FencePointRead>,
    ) {
        self.request = None;
        let point = match (outcome, read) {
            (Some(mp_link::requests::RequestOutcome::Accepted { .. }), Some(read)) => {
                LatLon::new(f64::from(read.lat), f64::from(read.lng))
                    .ok()
                    .map(|at| (at, read.count))
            }
            _ => None,
        };
        let Some((at, total)) = point else {
            self.end = Some(Err(FENCE_POINT_FAILED));
            return;
        };
        // `count = plla.total`.
        self.count = total;
        self.points.push(at);
        self.next = self.next.saturating_add(1);
    }

    /// How the download ended, once it has.
    #[must_use]
    pub const fn end(&self) -> Option<Result<(), &'static str>> {
        self.end
    }

    /// The points read, in order.
    #[must_use]
    pub fn points(&self) -> &[LatLon] {
        &self.points
    }
}

/// Which Geo-Fence > Download is under way.
#[derive(Debug, Clone)]
pub enum FenceDownloading {
    /// `mav_mission.download(..., MAV_MISSION_TYPE.FENCE)`, for a vehicle with `MISSION_FENCE`.
    Mission,
    /// `getFencePoint`, one point at a time.
    Points(FenceDownload),
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
    pub fn label(&self) -> String {
        match self {
            Self::Empty => "empty".to_owned(),
            Self::Vehicle => "read from the vehicle".to_owned(),
            Self::Edited => "edited here, not yet written".to_owned(),
            Self::File(name) => format!("loaded from {name}"),
        }
    }
}

impl Plan {
    /// The rows of the grid, in order: waypoint 1 onwards. Home is not among them.
    #[must_use]
    pub fn items(&self) -> &[MissionItem] {
        &self.items
    }

    /// Whether the grid has no rows. Home may still be set.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Where the contents came from.
    #[must_use]
    pub fn origin(&self) -> &Origin {
        &self.origin
    }

    /// The selected row's number.
    #[must_use]
    pub const fn selected(&self) -> Option<u16> {
        self.selected
    }

    /// Selects an item.
    pub fn select(&mut self, seq: Option<u16>) {
        self.selected = seq;
    }

    /// Replaces the rows with what the vehicle reported, and returns its home if the operator is
    /// to be asked whether to take it (see [`Plan::home_offer`]).
    ///
    /// `getWPs` hands the download to `processToScreen` whole, home and all, and item 0 leaves the
    /// grid there as it does for a file.
    /// `// C#: GCSViews/FlightPlanner.cs:1351-1373, 3978-4032`
    pub fn adopt_from_vehicle(&mut self, items: &[MissionItem]) -> Option<Home> {
        self.origin = Origin::Vehicle;
        self.adopt(items)
    }

    /// Replaces the rows with the contents of a file, and returns its home if the operator is to
    /// be asked whether to take it.
    ///
    /// `ReadWaypointFile` inserts a blank home when a file does not start at 0, so item 0 is
    /// always home by the time `processToScreen` sees the list.
    /// `// C#: GCSViews/FlightPlanner.cs:990-1010; ExtLibs/Utilities/MissionFile.cs:47-52`
    pub fn adopt_from_file(
        &mut self,
        name: impl Into<String>,
        items: &[MissionItem],
    ) -> Option<Home> {
        self.origin = Origin::File(name.into());
        self.adopt(items)
    }

    /// `processToScreen` for a mission, not appended: the grid replaced by everything after item
    /// 0, and item 0 offered to the Home Location boxes.
    /// `// C#: GCSViews/FlightPlanner.cs:5496-5676`
    fn adopt(&mut self, items: &[MissionItem]) -> Option<Home> {
        let (home, rows) = mp_mission::rows::split_home(items);
        self.items = rows;
        self.selected = None;
        home.and_then(|home| self.home_offer(&home))
    }

    /// Whether a mission's item 0 is to be offered as home, and if so the home it offers:
    /// "Reset Home to loaded coords", asked when its latitude, as the grid cell shows it, is
    /// neither what `TXT_homelat` holds nor `"0"`.
    ///
    /// Only the latitude is compared, as the C# compares it. The altitude the offer carries is
    /// the grid cell's, which holds `Locationwp.alt` - a `float`.
    /// `// C#: GCSViews/FlightPlanner.cs:5640-5660`
    #[must_use]
    pub fn home_offer(&self, item: &MissionItem) -> Option<Home> {
        let lat = double_text(item.x);
        if lat == self.home_text(HomeBox::Lat) || lat == "0" {
            return None;
        }
        #[allow(clippy::cast_possible_truncation)] // `Locationwp.alt` is a float
        let alt = f64::from(item.z as f32);
        Some(Home {
            lat: item.x,
            lng: item.y,
            alt,
        })
    }

    /// Yes to "Reset Home to loaded coords": the boxes take the loaded home, each through its
    /// `TextChanged`.
    /// `// C#: GCSViews/FlightPlanner.cs:5650-5657`
    pub fn reset_home_to(&mut self, loaded: Home) {
        self.set_home_text(HomeBox::Lat, double_text(loaded.lat));
        self.set_home_text(HomeBox::Lng, double_text(loaded.lng));
        self.set_home_text(HomeBox::Alt, two_decimals(loaded.alt));
    }

    /// One of the Home Location boxes.
    #[must_use]
    pub fn home_field(&self, which: HomeBox) -> &TextField {
        self.home.get(which)
    }

    /// What one of the Home Location boxes holds.
    #[must_use]
    pub fn home_text(&self, which: HomeBox) -> &str {
        self.home_field(which).value()
    }

    /// Sets a box as `TXT_home*.Text = ...` does, which raises its `TextChanged` when the text is
    /// different.
    pub fn set_home_text(&mut self, which: HomeBox, text: String) {
        if self.home_text(which) == text {
            return;
        }
        self.home.get_mut(which).set(text);
        self.home_text_changed(which);
    }

    /// A key pressed in one of the boxes, and its `TextChanged` when it changed the text.
    pub fn home_key(
        &mut self,
        which: HomeBox,
        event: &gpui::KeyDownEvent,
    ) -> crate::textfield::KeyOutcome {
        let outcome = self.home.get_mut(which).key(event);
        if outcome == crate::textfield::KeyOutcome::Changed {
            self.home_text_changed(which);
        }
        outcome
    }

    /// `TXT_homelat_TextChanged` and its two siblings: `sethome` cleared, then the planned home
    /// takes the box's number, or keeps what it had when the text does not parse. (Each also
    /// redraws the map, which the caller does.)
    /// `// C#: GCSViews/FlightPlanner.cs:7001-7050`
    fn home_text_changed(&mut self, which: HomeBox) {
        self.sethome = false;
        let Some(value) = mp_mission::rows::parse_number(self.home_text(which)) else {
            return;
        };
        match which {
            HomeBox::Lat => self.planned_home.lat = value,
            HomeBox::Lng => self.planned_home.lng = value,
            HomeBox::Alt => self.planned_home.alt = value,
        }
    }

    /// Home as a write takes it: the three boxes parsed, or `None` - "Your home location is
    /// invalid" - when any one does not.
    /// `// C#: GCSViews/FlightPlanner.cs:658-671`
    #[must_use]
    pub fn home(&self) -> Option<Home> {
        Home::parse(
            self.home_text(HomeBox::Lat),
            self.home_text(HomeBox::Lng),
            self.home_text(HomeBox::Alt),
        )
    }

    /// `cs.PlannedHomeLocation`.
    #[cfg(test)]
    #[must_use]
    pub const fn planned_home(&self) -> Home {
        self.planned_home
    }

    /// Sets the planned home without touching the boxes, as `MainV2` does at start-up from the
    /// saved settings; the boxes show it when the planning screen is next activated.
    pub fn set_planned_home(&mut self, home: Home) {
        self.planned_home = home;
    }

    /// `updateHomeText`, which `Activate` runs each time the planning screen is shown: the boxes
    /// take the vehicle's home if it has sent one, else the planned home if there is one, and
    /// are left as they are otherwise. Setting them runs their `TextChanged`, so a vehicle's home
    /// becomes the planned home too.
    /// `// C#: GCSViews/FlightPlanner.cs:1344-1349, 7195-7218`
    pub fn update_home_text(&mut self, vehicle: Option<Home>) {
        let chosen = match vehicle {
            Some(home) if home.is_set() => home,
            _ if self.planned_home.is_set() => self.planned_home,
            _ => return,
        };
        self.set_home_text(HomeBox::Lat, double_text(chosen.lat));
        self.set_home_text(HomeBox::Lng, double_text(chosen.lng));
        self.set_home_text(HomeBox::Alt, two_decimals(chosen.alt));
    }

    /// The Home Location link, `label4_LinkClicked`: home is where the vehicle is, at its
    /// altitude above sea level - or, with no position, the C#'s advice about getting one. (It
    /// then zooms to home, which is `zoomToHomeToolStripMenuItem`'s and not ported.)
    /// `// C#: GCSViews/FlightPlanner.cs:4283-4300`
    pub fn home_from_vehicle(
        &mut self,
        vehicle: Option<(LatLon, f64)>,
    ) -> Result<(), &'static str> {
        let Some((position, altasl)) = vehicle.filter(|(position, _)| position.latitude() != 0.0)
        else {
            return Err(HOME_NEEDS_A_FIX);
        };
        self.set_home_text(HomeBox::Alt, two_decimals(altasl));
        self.set_home_text(HomeBox::Lat, double_text(position.latitude()));
        self.set_home_text(HomeBox::Lng, double_text(position.longitude()));
        Ok(())
    }

    /// What Write sends the vehicle: home from the boxes at item 0 on an ArduPilot, then the rows;
    /// or "Your home location is invalid", and nothing sent, when the boxes do not parse. That is
    /// checked first, whatever the autopilot.
    /// `// C#: GCSViews/FlightPlanner.cs:646-671, 6196-6227`
    pub fn vehicle_mission(&self, ardupilot: bool) -> Result<Vec<MissionItem>, &'static str> {
        let home = self.home().ok_or(HOME_INVALID)?;
        Ok(mp_mission::rows::upload_list(
            ardupilot.then_some(home),
            &self.items,
        ))
    }

    /// What Save File writes: the `.waypoints` text with home at record 0 and the rows after it.
    /// `// C#: GCSViews/FlightPlanner.cs:6108-6160`
    #[must_use]
    pub fn waypoints_file(&self) -> String {
        mp_mission::waypoints::write_planned(self.home(), &self.items)
    }

    /// `cs.PlannedHomeLocation`, which the flight screen's map falls back on.
    #[must_use]
    pub const fn planned_home_location(&self) -> Home {
        self.planned_home
    }

    // ---- The panel boxes: `TXT_WPRad`, `TXT_loiterrad`, `TXT_DefaultAlt`, `CHK_splinedefault` ----

    /// One of the panel boxes.
    #[must_use]
    pub fn panel_field(&self, which: PanelBox) -> &TextField {
        self.panel.get(which)
    }

    /// What one of the panel boxes holds.
    #[must_use]
    pub fn panel_text(&self, which: PanelBox) -> &str {
        self.panel.get(which).value()
    }

    /// Sets a panel box's text, as the C# does when it loads one or reads a parameter into it.
    pub fn set_panel_text(&mut self, which: PanelBox, text: impl Into<String>) {
        self.panel.get_mut(which).set(text);
    }

    /// A key pressed in a panel box: its `KeyPress` first, which throws away a character the box
    /// does not take (`e.Handled = true`), then the box. A disabled Loiter Radius takes nothing.
    /// `// C#: GCSViews/FlightPlanner.cs:6984-6990, 7054-7064, 7075-7085`
    pub fn panel_key(
        &mut self,
        which: PanelBox,
        event: &gpui::KeyDownEvent,
    ) -> crate::textfield::KeyOutcome {
        if which == PanelBox::LoiterRadius && !self.panel.loiter_enabled {
            return crate::textfield::KeyOutcome::Ignored;
        }
        let keystroke = &event.keystroke;
        let chord = keystroke.modifiers.control || keystroke.modifiers.platform;
        if !chord
            && let Some(text) = keystroke.key_char.as_ref()
            && !text.chars().any(char::is_control)
            && !text.chars().all(|character| which.accepts(character))
        {
            return crate::textfield::KeyOutcome::Ignored;
        }
        self.panel.get_mut(which).key(event)
    }

    /// `TXT_*_Leave`: a box left holding what `float.TryParse` refuses goes back to a number -
    /// Default Alt to "100", Loiter Radius to "45", and WP Radius, only when it is empty, to
    /// `startupWPradius`. (WP Radius's also redraws the waypoints' radius circles, which the
    /// map here reads from the box every frame - see [`map_overlay`].)
    /// `// C#: GCSViews/FlightPlanner.cs:6992-6999, 7066-7073, 7087-7106`
    pub fn panel_leave(&mut self, which: PanelBox) {
        if float_parses(self.panel_text(which)) {
            return;
        }
        match which {
            // No `Leave` handler: Write reads an empty box as 0.
            PanelBox::AltWarn => {}
            PanelBox::DefaultAlt => self.set_panel_text(which, "100"),
            PanelBox::LoiterRadius => self.set_panel_text(which, "45"),
            PanelBox::WpRadius => {
                if self.panel_text(which).is_empty() {
                    let startup = self.panel.startup_wp_radius.clone();
                    self.set_panel_text(which, startup);
                }
            }
        }
    }

    /// `config(false)`, when the planning screen loads: each box takes what `config.xml` saved
    /// for it, and WP Radius's saved value becomes the one an emptied box goes back to.
    /// `// C#: GCSViews/FlightPlanner.cs:2592-2612, 3428`
    pub fn apply_panel_config(&mut self, config: Option<&mp_settings::Config>) {
        let Some(config) = config else {
            return;
        };
        for which in PanelBox::ALL {
            let Some(text) = config.get(which.config_key()) else {
                continue;
            };
            self.set_panel_text(which, text);
            if which == PanelBox::WpRadius {
                text.clone_into(&mut self.panel.startup_wp_radius);
            }
        }
        // `case "fpcoordmouse": coords1.System = ...`: a name the combo has not got leaves it.
        // `// C#: GCSViews/FlightPlanner.cs:2625-2627`
        if let Some(system) = config.get("fpcoordmouse").and_then(|name| {
            crate::coords::CoordSystem::ALL
                .into_iter()
                .find(|system| system.name() == name)
        }) {
            self.coords.system = system;
        }
    }

    /// `setWPParams`, run when the planning screen is shown and when a mission has been read: WP
    /// Radius from `WP_RADIUS`, then `WPNAV_RADIUS` in centimetres, then `WP_RADIUS_M` (4.7 and
    /// later) - the last of them the vehicle has wins - each written `{0:N2}`; Loiter Radius from
    /// `LOITER_RADIUS`, or else `WP_LOITER_RAD`, and disabled when the vehicle has neither.
    /// Distances are metres here, so `multiplierdist` is 1.
    /// `// C#: GCSViews/FlightPlanner.cs:6695-6750`
    pub fn set_wp_params(&mut self, parameters: &[(String, f64)]) {
        let param = |name: &str| {
            parameters
                .iter()
                .find(|(held, _)| held == name)
                .map(|(_, value)| *value)
        };
        if let Some(value) = param("WP_RADIUS") {
            self.set_panel_text(PanelBox::WpRadius, number_n2(value));
        }
        if let Some(value) = param("WPNAV_RADIUS") {
            self.set_panel_text(PanelBox::WpRadius, number_n2(value / 100.0));
        }
        if let Some(value) = param("WP_RADIUS_M") {
            self.set_panel_text(PanelBox::WpRadius, number_n2(value));
        }
        self.panel.loiter_enabled = false;
        if let Some(value) = param("LOITER_RADIUS").or_else(|| param("WP_LOITER_RAD")) {
            self.set_panel_text(
                PanelBox::LoiterRadius,
                mp_params::param_file::invariant_double(value),
            );
            self.panel.loiter_enabled = true;
        }
    }

    /// `TXT_loiterrad.Enabled`.
    #[must_use]
    pub const fn loiter_enabled(&self) -> bool {
        self.panel.loiter_enabled
    }

    /// `CHK_splinedefault.Checked`.
    #[must_use]
    pub const fn spline(&self) -> bool {
        self.panel.spline
    }

    /// `CHK_splinedefault_CheckedChanged`: `splinemode = CHK_splinedefault.Checked`, which makes
    /// a click on the map add a spline waypoint.
    /// `// C#: GCSViews/FlightPlanner.cs:2059-2062, 588-592`
    pub fn set_spline(&mut self, on: bool) {
        self.panel.spline = on;
    }

    /// `Activate`'s check of Default Alt: one `int.Parse` refuses is replaced with 50, and "Please
    /// fix your default alt value" is said.
    /// `// C#: GCSViews/FlightPlanner.cs:329-337`
    pub fn check_default_alt(&mut self) -> Option<&'static str> {
        if self
            .panel_text(PanelBox::DefaultAlt)
            .trim()
            .parse::<i32>()
            .is_ok()
        {
            return None;
        }
        self.set_panel_text(PanelBox::DefaultAlt, "50");
        Some(DEFAULT_ALT_FIX)
    }

    /// The altitude `setfromMap` gives a new row, from Default Alt and the altitude its handler
    /// passes - 0 from a click on the map, the vehicle's for At Current Position, Default Alt's
    /// own for the menu's entries. Default Alt must be a whole number, or "Your default alt is
    /// not valid"; the passed altitude wins over it unless that is 0; and a Default Alt of 0
    /// gives 50, or 15 on a copter, whatever was passed.
    ///
    /// Two things before that are not ported: a Home Location altitude that does not parse asks
    /// for one ("You must have a home altitude"), and a Default Alt of 0 asks for another
    /// ("Default Altitude", offering 100) - the row takes what the C# gives an answer of 0. And
    /// where the C# has already added the row when it refuses, nothing is added here.
    /// `// C#: GCSViews/FlightPlanner.cs:1164-1208`
    pub fn new_row_altitude(&self, passed: f64, copter: bool) -> Result<f64, &'static str> {
        let default = self
            .panel_text(PanelBox::DefaultAlt)
            .trim()
            .parse::<i32>()
            .map_err(|_| DEFAULT_ALT_INVALID)?;
        if default == 0 {
            return Ok(if copter { 15.0 } else { 50.0 });
        }
        Ok(if passed == 0.0 {
            f64::from(default)
        } else {
            passed
        })
    }

    // ---- Terrain: Verify Height, and home taken from the map ----

    /// `CHK_verifyheight.Checked`.
    #[must_use]
    pub const fn verify_height(&self) -> bool {
        self.verify_height
    }

    /// Ticks or clears Verify Height. The box has no handler of its own: `setfromMap` reads it.
    /// `// C#: GCSViews/FlightPlanner.Designer.cs:246-250; GCSViews/FlightPlanner.cs:1123, 1211`
    pub fn set_verify_height(&mut self, on: bool) {
        self.verify_height = on;
    }

    /// The terrain lookup this screen asks.
    #[must_use]
    pub const fn terrain(&self) -> Terrain {
        self.terrain
    }

    /// `chk_grid.Checked`, the static `grid`.
    #[must_use]
    pub const fn grid(&self) -> bool {
        self.grid
    }

    /// `chk_grid_CheckedChanged`: `grid = chk_grid.Checked`; the map repaints with it.
    /// `// C#: GCSViews/FlightPlanner.cs:2053-2057`
    pub fn set_grid(&mut self, on: bool) {
        self.grid = on;
    }

    /// `chk_usemavftp.Checked`.
    #[must_use]
    pub const fn use_mavftp(&self) -> bool {
        self.use_mavftp
    }

    /// The box as `Init` restores it: `Settings.Instance.GetBoolean("UseMissionMAVFTP", false)`,
    /// `bool.TryParse` of the saved text.
    /// `// C#: GCSViews/FlightPlanner.cs:319; ExtLibs/Utilities/Settings.cs:203-212`
    pub fn apply_mavftp_config(&mut self, config: Option<&mp_settings::Config>) {
        self.use_mavftp = config
            .and_then(|config| config.get("UseMissionMAVFTP"))
            .and_then(mp_mission::dotnet::parse_bool)
            .unwrap_or(false);
    }

    /// `chk_usemavftp_CheckedChanged`'s state; the caller saves `UseMissionMAVFTP`.
    /// `// C#: GCSViews/FlightPlanner.cs:8403-8406`
    pub fn set_use_mavftp(&mut self, on: bool) {
        self.use_mavftp = on;
    }

    /// Whether `panelWaypoints` is collapsed to its button.
    #[must_use]
    pub const fn commands_minimised(&self) -> bool {
        self.commands_minimised
    }

    /// `but_mincommands_Click`: a panel 30 high or less grows back to 166 and the button says
    /// ˅; a taller one shrinks to the button's height and it says ˄.
    /// `// C#: GCSViews/FlightPlanner.cs:69-80`
    pub fn toggle_commands_minimised(&mut self) {
        self.commands_minimised = !self.commands_minimised;
    }

    /// `coords1` as it stands.
    #[must_use]
    pub const fn coords(&self) -> &crate::coords::Coords {
        &self.coords
    }

    /// `CMB_coordsystem_SelectedIndexChanged`: the system chosen.
    /// `// C#: ExtLibs/Controls/Coords.cs:172-179`
    pub fn set_coord_system(&mut self, system: crate::coords::CoordSystem) {
        self.coords.system = system;
    }

    /// `SetMouseDisplay(lat, lng, alt)`: the pointer's position into `coords1` with
    /// `srtm.getAltitude`'s height and source, the height in the display unit.
    /// `// C#: GCSViews/FlightPlanner.cs:2778-2797`
    pub fn set_mouse_display(&mut self, at: LatLon) {
        let (lat, lng) = (at.latitude(), at.longitude());
        if (self.coords.lat, self.coords.lng) == (lat, lng) {
            return;
        }
        let answer = self.terrain.at(lat, lng);
        self.coords.lat = lat;
        self.coords.lng = lng;
        self.coords.alt = answer.alt * f64::from(MULTIPLIER_ALT);
        self.coords.alt_source = answer.alt_source;
        self.coords.alt_unit = "m";
    }

    /// The mission going over MAVFTP, if one is.
    #[must_use]
    pub const fn mission_ftp(&self) -> Option<&MissionFtp> {
        self.mission_ftp.as_ref()
    }

    /// Puts a test's terrain in the process's place.
    #[cfg(test)]
    pub fn set_terrain(&mut self, terrain: Terrain) {
        self.terrain = terrain;
    }

    /// `setfromMap`'s Verify Height, for a new row at `lat`, `lng` that the lines before it have
    /// given `altitude` ([`Plan::new_row_altitude`]): with the box ticked the altitude is taken
    /// again from Default Alt and the ground, by the screen's frame -
    ///
    /// - Absolute: the ground at the row plus Default Alt, in `double`;
    /// - Terrain: Default Alt alone;
    /// - Relative: the ground at the row, less the ground at the planned home, plus Default Alt -
    ///   each ground height cut to a whole number first, and the sum made in `float`.
    ///
    /// Whatever altitude the handler passed is not used - Default Alt is, even where it is 0 and
    /// the lines before gave 50. A point with no terrain counts as 0, the `alt` of `srtm`'s
    /// Invalid answer: the C# does not look at the kind of answer. Box clear, `altitude` stands.
    /// `// C#: GCSViews/FlightPlanner.cs:1209-1236`
    #[must_use]
    pub fn verified_altitude(
        &self,
        lat: f64,
        lng: f64,
        altitude: f64,
        frame: AltitudeFrame,
    ) -> f64 {
        if !self.verify_height {
            return altitude;
        }
        // `int.Parse(TXT_DefaultAlt.Text)`, which the `int.TryParse` before it has passed.
        let Ok(default) = self.panel_text(PanelBox::DefaultAlt).trim().parse::<i32>() else {
            return altitude;
        };
        let ground = |lat: f64, lng: f64| self.terrain.at(lat, lng).alt;
        match frame {
            // `(srtm.getAltitude(lat, lng).alt) * multiplieralt + int.Parse(...)`: a double.
            AltitudeFrame::Absolute => {
                ground(lat, lng) * f64::from(MULTIPLIER_ALT) + f64::from(default)
            }
            AltitudeFrame::Terrain => f64::from(default),
            // `(int) (srtm...alt) * multiplieralt + int.Parse(...) - (int) srtm(home).alt *
            // multiplieralt`: int times float is float, and the sum stays float.
            AltitudeFrame::Relative => {
                let here = c_sharp_int(ground(lat, lng));
                let home = c_sharp_int(ground(self.planned_home.lat, self.planned_home.lng));
                #[allow(clippy::cast_precision_loss)] // the C#'s int-to-float conversions
                let sum =
                    here as f32 * MULTIPLIER_ALT + default as f32 - home as f32 * MULTIPLIER_ALT;
                f64::from(sum)
            }
        }
    }

    /// A row grabbed on the map: where it is, which is what the grid's Lat and Long cells hold
    /// until the drop writes the new position.
    pub fn begin_drag(&mut self, seq: u16) {
        self.drag_origin = self
            .items
            .iter()
            .find(|item| item.seq == seq)
            .and_then(|item| LatLon::new(item.x, item.y).ok())
            .map(|origin| (seq, origin));
    }

    /// A grabbed row let go: `callMeDrag(tag, lat, lng, -2)`, which runs `setfromMap(lat, lng,
    /// -2)` - and there, with Verify Height ticked and the screen's frame not Terrain, a row whose
    /// Alt column is headed "Alt" keeps its height above the ground: its altitude, cut to a whole
    /// number, plus the ground where it now is, less the ground where it was, each ground height
    /// cut to whole metres, the sum cut again. A row let go where it was grabbed was never dragged
    /// (`isMouseDraging`), and nothing happens to it.
    /// `// C#: GCSViews/FlightPlanner.cs:1119-1146, 7803-7810`
    pub fn end_drag(
        &mut self,
        seq: u16,
        frame: AltitudeFrame,
        family: Option<mp_vehicle::VehicleFamily>,
    ) {
        let Some((grabbed, origin)) = self.drag_origin.take() else {
            return;
        };
        if grabbed != seq || !self.verify_height || frame == AltitudeFrame::Terrain {
            return;
        }
        let terrain = self.terrain;
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        if (item.x, item.y) == (origin.latitude(), origin.longitude())
            || !alt_column(item.command, family)
        {
            return;
        }
        let ground =
            |lat: f64, lng: f64| c_sharp_int(terrain.at(lat, lng).alt * f64::from(MULTIPLIER_ALT));
        let oldsrtm = ground(origin.latitude(), origin.longitude());
        let newsrtm = ground(item.x, item.y);
        // `float ans; ans = (int) ans;` then `(int) (ans + newsrtm - oldsrtm)`, in float.
        #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)] // the C#'s casts
        let newh = {
            let ans = (item.z as f32).trunc();
            (ans + newsrtm as f32 - oldsrtm as f32) as i32
        };
        item.z = f64::from(newh);
        self.origin = Origin::Edited;
    }

    /// `callMeDrag("H", lat, lng, ...)` and Set Home Here: home moved to a point at the ground's
    /// height there - the ASL box first, `ToString("0.00")`, then Lat and Long, each through its
    /// `TextChanged`. A point with no terrain puts 0 in the box, the `alt` of `srtm`'s Invalid
    /// answer: neither handler looks at the kind of answer.
    /// `// C#: GCSViews/FlightPlanner.cs:745-755, 6625-6632`
    pub fn set_home_at(&mut self, position: LatLon) {
        let ground = self
            .terrain
            .at(position.latitude(), position.longitude())
            .alt;
        self.set_home_text(
            HomeBox::Alt,
            two_decimals(ground * f64::from(MULTIPLIER_ALT)),
        );
        self.set_home_text(HomeBox::Lat, double_text(position.latitude()));
        self.set_home_text(HomeBox::Lng, double_text(position.longitude()));
    }

    /// `TXT_homelat_Enter`: the Lat box entered says "Click on the Map to set Home " unless it
    /// has already, and the next click on the map sets home.
    /// `// C#: GCSViews/FlightPlanner.cs:7016-7022`
    pub fn home_lat_enter(&mut self) -> Option<&'static str> {
        let said = (!self.sethome).then_some(CLICK_TO_SET_HOME);
        self.sethome = true;
        said
    }

    /// `sethome`.
    #[must_use]
    pub const fn sethome(&self) -> bool {
        self.sethome
    }

    /// `AddWPToMap`'s first two branches, for a click that would add something at `position`.
    /// Drawing a polygon, the click is a corner, which the caller adds: `false`. Otherwise, with
    /// `sethome` set, it is cleared and home moved there (`callMeDrag("H", ...)`), and the click
    /// has done its work: `true`.
    /// `// C#: GCSViews/FlightPlanner.cs:558-572`
    pub fn click_sets_home(&mut self, position: LatLon) -> bool {
        if self.draw_mode == DrawMode::Area || !self.sethome {
            return false;
        }
        self.sethome = false;
        self.set_home_at(position);
        true
    }

    /// What `saveWPs` sets once the mission is written, "Setting params": the radius into all
    /// three names ArduPilot has used for it - "use brute force, for all three possible params" -
    /// `WPNAV_RADIUS` in centimetres, then Loiter Radius into `LOITER_RAD`, else `WP_LOITER_RAD`.
    /// `float.Parse` refusing WP Radius ends the write there; a failed loiter set is caught.
    /// `// C#: GCSViews/FlightPlanner.cs:6296-6317`
    pub fn wp_param_steps(&self) -> Result<Vec<ParamStep>, &'static str> {
        let radius = self
            .panel_text(PanelBox::WpRadius)
            .trim()
            .parse::<f32>()
            .map_err(|_| FORMAT_EXCEPTION)?;
        let radius = f64::from(radius);
        let stop = |name: &'static str| OnTimeout::Stop {
            title: ERROR,
            text: format!("Timeout on read - setParam {name}"),
        };
        let mut steps = vec![
            ParamStep::one("WP_RADIUS", radius, stop("WP_RADIUS")),
            ParamStep::one("WP_RADIUS_M", radius, stop("WP_RADIUS_M")),
            ParamStep::one("WPNAV_RADIUS", radius * 100.0, stop("WPNAV_RADIUS")),
        ];
        // `try { port.setParam(new[] {"LOITER_RAD", "WP_LOITER_RAD"}, ...) } catch { }`: a
        // loiter radius that does not parse, or a set that times out, is passed over.
        if let Ok(loiter) = self
            .panel_text(PanelBox::LoiterRadius)
            .trim()
            .parse::<f32>()
        {
            steps.push(ParamStep {
                names: vec!["LOITER_RAD", "WP_LOITER_RAD"],
                value: f64::from(loiter),
                on_timeout: OnTimeout::CarryOn,
            });
        }
        Ok(steps)
    }

    // ---- The Geo-Fence drop-down's state ----

    /// The geofence's return location, `geofenceoverlay.Markers[0]`.
    #[must_use]
    pub const fn fence_return(&self) -> Option<LatLon> {
        self.fence_return
    }

    /// Geo-Fence > Set Return Location: the red marker, moved to where the menu was opened.
    /// `// C#: GCSViews/FlightPlanner.cs:6663-6670`
    pub fn set_fence_return(&mut self, position: LatLon) {
        self.fence_return = Some(position);
    }

    /// Geo-Fence > Load from File, once the file is read: the drawn polygon is emptied and takes
    /// the file's corners, and the file's first line, if it has one, moves the return marker.
    /// The C#'s `drawnpolygon` is this screen's survey area, and it is what Upload would send.
    /// `// C#: GCSViews/FlightPlanner.cs:4345-4412`
    pub fn adopt_fence_file(&mut self, file: mp_mission::fence_file::FenceFile) {
        self.polygon = file.vertices;
        self.polygon_hidden = false;
        if let Some(position) = file.return_point {
            self.fence_return = Some(position);
        }
    }

    /// What Geo-Fence > Save to File writes: the return location, then the drawn polygon, or the
    /// geofence when nothing is drawn, closed by its first corner again. With neither, the C#
    /// throws on the closing corner and says "Failed to write fence file"; with no return
    /// location it has said "Please set a return location" before asking for a file.
    /// `// C#: GCSViews/FlightPlanner.cs:5959-6021`
    pub fn fence_file(&self) -> Result<mp_mission::fence_file::FenceFile, &'static str> {
        let return_point = self.fence_return.ok_or(SET_RETURN_LOCATION)?;
        let vertices = if self.polygon.is_empty() {
            self.fence.clone()
        } else {
            self.polygon.clone()
        };
        if vertices.is_empty() {
            return Err(FENCE_FILE_FAILED);
        }
        Ok(mp_mission::fence_file::FenceFile {
            return_point: Some(return_point),
            vertices,
        })
    }

    /// `polygongridmode = false`: Geo-Fence > Upload and Download stop the drawing of a polygon.
    /// `// C#: GCSViews/FlightPlanner.cs:826, 3716`
    pub fn leave_polygon_mode(&mut self) {
        if self.draw_mode == DrawMode::Area {
            self.draw_mode = DrawMode::Waypoints;
        }
    }

    /// Whether Geo-Fence > Upload or Download is under way, its boxes included: one at a time, as
    /// the C#'s modal calls allow.
    #[must_use]
    pub const fn fence_busy(&self) -> bool {
        self.fence_ask.is_some() || self.fence_upload.is_some() || self.fence_download.is_some()
    }

    /// Geo-Fence > Upload says `text`: on the status line, and in its fact.
    pub fn fence_upload_said(&mut self, text: &str) {
        text.clone_into(&mut self.fence_upload_text);
        self.fence_say = Some(text.to_owned());
    }

    /// Geo-Fence > Download says `text`.
    pub fn fence_download_said(&mut self, text: &str) {
        text.clone_into(&mut self.fence_download_text);
        self.fence_say = Some(text.to_owned());
    }

    /// Geo-Fence > Upload ended as the C# ends it without a word - its progress window closed -
    /// and its fact says `text`.
    pub fn fence_upload_noted(&mut self, text: &str) {
        text.clone_into(&mut self.fence_upload_text);
    }

    /// Geo-Fence > Download ended without a word, and its fact says `text`.
    pub fn fence_download_noted(&mut self, text: &str) {
        text.clone_into(&mut self.fence_download_text);
    }

    /// The end of Geo-Fence > Upload: the drawn polygon is off the map and emptied, and the
    /// geofence is its corners. The return marker stays, as `geofenceoverlay.Markers` is not
    /// cleared.
    /// `// C#: GCSViews/FlightPlanner.cs:3861-3874`
    pub fn fence_uploaded(&mut self, polygon: Vec<LatLon>) {
        self.fence = polygon;
        self.polygon.clear();
        self.polygon_hidden = false;
    }

    /// The end of Geo-Fence > Download's point by point read: the first point is the return
    /// location, the rest the geofence - the closing point, a copy of the first corner, with it.
    /// `// C#: GCSViews/FlightPlanner.cs:881-892`
    pub fn fence_downloaded(&mut self, points: &[LatLon]) {
        if let Some((first, rest)) = points.split_first() {
            self.fence_return = Some(*first);
            self.fence = rest.to_vec();
        }
    }

    /// What the last Geo-Fence > Upload said: while one runs, the progress reporter's text.
    #[must_use]
    pub fn fence_upload_text(&self) -> &str {
        match &self.fence_upload {
            Some(upload) => upload.status(),
            None if self.fence_ask.is_some() => "asking",
            None if self.fence_upload_text.is_empty() => "none",
            None => &self.fence_upload_text,
        }
    }

    /// What the last Geo-Fence > Download said: "busy" while one runs.
    #[must_use]
    pub fn fence_download_text(&self) -> &str {
        if self.fence_download.is_some() {
            "busy"
        } else if self.fence_download_text.is_empty() {
            "none"
        } else {
            &self.fence_download_text
        }
    }

    /// The end of Geo-Fence > Clear, once its three sets are done: the geofence goes, and the drawn
    /// polygon goes off the map - its corners stay, and come back with the next one drawn, as
    /// `drawnpolygon.Points` does. The return marker stays: the C# clears the overlay's polygons,
    /// not its markers.
    /// `// C#: GCSViews/FlightPlanner.cs:2150-2155`
    pub fn clear_geofence(&mut self) {
        self.clear_fence();
        self.hide_polygon();
    }

    /// The row of parameter sets under way, if one is.
    #[must_use]
    pub const fn writes(&self) -> Option<&ParamWrites> {
        self.writes.as_ref()
    }

    /// What the last row of sets did, parameter by parameter.
    #[must_use]
    pub fn write_results(&self) -> &str {
        &self.write_results
    }

    /// Appends a waypoint at a position.
    ///
    /// Sequence numbers are reassigned from scratch rather than incremented, because a mission
    /// with a gap in its sequence is rejected by the vehicle at upload time - a failure that
    /// happens minutes after the mistake, with no indication of which edit caused it.
    #[cfg(test)]
    pub fn add_waypoint(&mut self, position: LatLon, altitude: f64) {
        self.add_waypoint_in(position, altitude, AltitudeFrame::Relative);
    }

    /// Adds a waypoint in a given altitude frame.
    ///
    /// The frame comes from the screen's `CMB_altmode` rather than the item, because that is where
    /// Mission Planner takes it from: `e.Row.Cells[Frame.Index].Value = CMB_altmode.SelectedValue`
    /// on row creation. `// C#: GCSViews/FlightPlanner.cs:2347`
    pub fn add_waypoint_in(&mut self, position: LatLon, altitude: f64, frame: AltitudeFrame) {
        self.items.push(MissionItem {
            seq: 0,
            current: 0,
            frame: frame.mav_frame(),
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

    /// Moves an item to a new position, keeping everything else about it.
    ///
    /// Only items that have a position are moved. Dragging what the map drew for a
    /// return-to-launch would write coordinates into a command that ignores them, and the map
    /// would then draw a waypoint the vehicle has no intention of visiting.
    pub fn move_to(&mut self, seq: u16, position: LatLon) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        if !matches!(item.position(), Ok(Some(_))) {
            return;
        }
        item.x = position.latitude();
        item.y = position.longitude();
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
        self.selected = self.items.get(target).map(|item| item.seq);
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

    /// Changes one of an item's four command parameters.
    ///
    /// Which parameter means what depends on the command, and the meanings come from the MAVLink
    /// definitions rather than from a table here - see `MavCmd::parameters`.
    pub fn set_param(&mut self, seq: u16, index: usize, value: f64) {
        let Some(item) = self.items.iter_mut().find(|item| item.seq == seq) else {
            return;
        };
        match index {
            0 => item.param1 = value,
            1 => item.param2 = value,
            2 => item.param3 = value,
            3 => item.param4 = value,
            _ => return,
        }
        self.origin = Origin::Edited;
    }

    /// One of an item's four command parameters.
    #[must_use]
    pub fn param(&self, seq: u16, index: usize) -> Option<f64> {
        let item = self.items.iter().find(|item| item.seq == seq)?;
        match index {
            0 => Some(item.param1),
            1 => Some(item.param2),
            2 => Some(item.param3),
            3 => Some(item.param4),
            _ => None,
        }
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

    /// Discards everything, including the survey area and the fence - but not home, which is the
    /// Home Location boxes' and not the mission's.
    pub fn clear(&mut self) {
        self.items.clear();
        self.origin = Origin::Empty;
        self.selected = None;
        self.polygon.clear();
        self.polygon_hidden = false;
        self.fence.clear();
        self.fence_error = None;
        self.rally.clear();
        self.rally_error = None;
    }

    /// The geofence's vertices.
    #[must_use]
    pub fn fence(&self) -> &[LatLon] {
        &self.fence
    }

    /// Adds a vertex to the geofence.
    pub fn add_fence_vertex(&mut self, position: LatLon) {
        self.fence.push(position);
        self.fence_error = None;
    }

    /// Removes the last fence vertex drawn.
    pub fn undo_fence_vertex(&mut self) {
        self.fence.pop();
        self.fence_error = None;
    }

    /// Discards the geofence, exclusions included, leaving the mission alone.
    pub fn clear_fence(&mut self) {
        self.fence.clear();
        self.fence_exclusions.clear();
        self.fence_error = None;
    }

    /// The rally points.
    #[must_use]
    pub fn rally(&self) -> &[RallyPoint] {
        &self.rally
    }

    /// Adds a rally point at [`RALLY_ALTITUDE`].
    pub fn add_rally_point(&mut self, position: LatLon) {
        self.rally.push(RallyPoint {
            position,
            altitude: RALLY_ALTITUDE,
            break_altitude: None,
        });
        self.rally_error = None;
    }

    /// Removes the last rally point placed.
    pub fn undo_rally_point(&mut self) {
        self.rally.pop();
        self.rally_error = None;
    }

    /// Discards the rally points.
    pub fn clear_rally(&mut self) {
        self.rally.clear();
        self.rally_error = None;
    }

    /// Set Rally Point's marker: at the menu's position, `alt / CurrentState.multiplieralt` -
    /// metres here, so the altitude typed - kept by the marker as an `int`.
    /// `// C#: GCSViews/FlightPlanner.cs:6646-6653; ExtLibs/Maps/GMapMarkerRallyPt.cs:33-37`
    pub fn add_rally_marker(&mut self, position: LatLon, altitude: i32) {
        self.rally.push(RallyPoint {
            position,
            altitude: f64::from(altitude),
            break_altitude: None,
        });
        self.rally_error = None;
    }

    /// The rally markers as the C# holds them, `rallypointoverlay.Markers`: each position and its
    /// `int` altitude.
    #[must_use]
    pub fn rally_markers(&self) -> Vec<(LatLon, i32)> {
        self.rally
            .iter()
            .map(|point| {
                #[allow(clippy::cast_possible_truncation)] // `(int) plla.Alt`
                let altitude = point.altitude as i32;
                (point.position, altitude)
            })
            .collect()
    }

    /// Load Rally from File, once the file is read: the first row the C# reads clears the markers,
    /// then each row is a marker at its position and altitude. A file with no row it reads leaves
    /// the markers as they were.
    /// `// C#: GCSViews/FlightPlanner.cs:4446-4458`
    pub fn load_rally_file(&mut self, rows: &[mp_mission::fence_file::RallyPointFile]) {
        if rows.is_empty() {
            return;
        }
        self.rally.clear();
        for row in rows {
            #[allow(clippy::cast_possible_truncation)] // `(int) plla.Alt`, of a `short`
            self.add_rally_marker(row.position, row.altitude as i32);
        }
    }

    /// Clear Rally Points, once its set of `RALLY_TOTAL` has been made or failed: the markers go,
    /// and so does `MAV.rallypoints`.
    /// `// C#: GCSViews/FlightPlanner.cs:2108-2109`
    pub fn clear_rally_points(&mut self) {
        self.clear_rally();
        self.vehicle_rally.clear();
    }

    /// `MAV.rallypoints`: the rally list the vehicle last sent.
    #[must_use]
    pub fn vehicle_rally(&self) -> &[MissionItem] {
        &self.vehicle_rally
    }

    /// Rally Points > Upload under way, if one is.
    #[must_use]
    pub const fn rally_upload(&self) -> Option<&RallyUpload> {
        self.rally_upload.as_ref()
    }

    /// Whether Rally Points > Download is waiting for the vehicle.
    #[must_use]
    pub const fn rally_downloading(&self) -> bool {
        self.rally_download
    }

    /// Polygon > Area's figure: `Math.Abs(calcpolygonarea(drawnpolygon.Points))`, square metres,
    /// or `None` where `calcpolygonarea` says "Please define a polygon!" and returns 0. As the C#'s
    /// does, it closes the polygon to measure it and takes the last corner off if it then equals
    /// the first - so a polygon that arrived closed leaves open.
    /// `// C#: GCSViews/FlightPlanner.cs:1762, 1986-2036`
    pub fn polygon_area(&mut self) -> Option<f64> {
        if self.polygon.is_empty() {
            return None;
        }
        Some(mp_mission::gridui::calc_polygon_area(&mut self.polygon))
    }

    /// Polygon > Offset Polygon, once the offset is known: the drawn polygon replaced by what
    /// `redrawPolygonSurvey` is handed, and left as it was where the C# returns first.
    /// `// C#: GCSViews/FlightPlanner.cs:3639-3686, 1014-1050`
    pub fn offset_polygon(&mut self, metres: f64) {
        if let Some(corners) = mp_mission::polygon::offset_polygon(&self.polygon, metres) {
            self.polygon = corners;
            self.polygon_hidden = false;
        }
    }

    /// Polygon > Load Polygon, once the file is read: `drawnpolygon.Points` cleared and given the
    /// file's corners, even none.
    /// `// C#: GCSViews/FlightPlanner.cs:4538-4576`
    pub fn load_polygon(&mut self, corners: Vec<LatLon>) {
        self.polygon = corners;
        self.polygon_hidden = false;
    }

    /// Polygon > From SHP: every feature's coordinates added to the drawn polygon in turn, and
    /// after each feature "remove loop close" - the last corner dropped where it equals the
    /// polygon's first. The polygon was cleared before the file was opened.
    /// `// C#: GCSViews/FlightPlanner.cs:3564-3601`
    pub fn add_shp_features(
        &mut self,
        features: &[Vec<(f64, f64)>],
        projection: Option<mp_mission::shapefile::Projection>,
    ) {
        for feature in features {
            for (x, y) in feature {
                if let Some(position) = mp_mission::shapefile::position(projection, *x, *y) {
                    self.polygon.push(position);
                }
            }
            if self.polygon.len() > 1 && self.polygon.first() == self.polygon.last() {
                self.polygon.pop();
            }
        }
        self.polygon_hidden = false;
    }

    /// Create Wp Circle's rows, once its four answers have parsed: `WAYPOINT` rows round the
    /// menu's position at `(int) float.Parse(TXT_DefaultAlt.Text)` as `setfromMap` takes it, in
    /// the screen's altitude frame. `Radius / CurrentState.multiplierdist` is the radius itself in
    /// metres.
    ///
    /// The C# adds each row and then fills it, so a Default Alt that does not parse leaves a row
    /// with no position and throws, and one that is not a whole number leaves every row empty with
    /// "Your default alt is not valid" said once a row; here it is said once and nothing is added.
    /// `// C#: GCSViews/FlightPlanner.cs:2992, 3015-3047`
    pub fn wp_circle(
        &mut self,
        centre: LatLon,
        radius: i32,
        points: i32,
        direction: i32,
        start_angle: i32,
        context: &MenuContext,
    ) -> Result<(), &'static str> {
        let passed = mp_mission::dotnet::parse_f32(self.panel_text(PanelBox::DefaultAlt))
            .ok_or(FORMAT_EXCEPTION)?;
        let passed = mp_mission::dotnet::to_int(f64::from(passed));
        let altitude = self.new_row_altitude(f64::from(passed), context.copter)?;
        let frame = context.frame.mav_frame();
        for (lat, lng) in mp_mission::circle::wp_circle(
            centre.latitude(),
            centre.longitude(),
            radius,
            points,
            direction,
            start_angle,
        ) {
            // Each point through `setfromMap`, Verify Height with it.
            let altitude = self.verified_altitude(lat, lng, altitude, context.frame);
            self.items.push(circle_row(
                mp_mission::commands::WAYPOINT,
                frame,
                lat,
                lng,
                altitude,
            ));
        }
        self.renumber();
        self.origin = Origin::Edited;
        Ok(())
    }

    /// Create Spline Circle's rows, once its answers have parsed: a `DO_SET_ROI` at the menu's
    /// position, altitude 0 (`AddCommand`, which fills its cells directly), then
    /// `SPLINE_WAYPOINT` rows lap by lap, each at the altitude handed to `setfromMap` - which
    /// writes none for -1 or -2, leaving the row's 0.
    ///
    /// As [`Plan::wp_circle`], a Default Alt `setfromMap` refuses is said once and nothing added.
    /// `// C#: GCSViews/FlightPlanner.cs:2909-2960, 540-550, 3376-3414`
    pub fn spline_circle(
        &mut self,
        centre: LatLon,
        radius: i32,
        min_alt: i32,
        max_alt: i32,
        alt_step: i32,
        context: &MenuContext,
    ) -> Result<(), &'static str> {
        let points = mp_mission::circle::spline_circle(
            centre.latitude(),
            centre.longitude(),
            radius,
            min_alt,
            max_alt,
            alt_step,
        )
        .map_err(|_| "Bad alt step")?;
        let mut rows = Vec::with_capacity(points.len());
        for (lat, lng, step_alt) in points {
            // (A Min Alt of -2 is `setfromMap`'s drag sentinel, and with Verify Height ticked the
            // C# would move a 0 by the ground under the row's "0","0" cells. Not ported: the row
            // keeps its 0, as it does with the box clear.)
            let altitude = if matches!(step_alt, -1 | -2) {
                0.0
            } else {
                let altitude = self.new_row_altitude(f64::from(step_alt), context.copter)?;
                self.verified_altitude(lat, lng, altitude, context.frame)
            };
            rows.push(circle_row(
                mp_mission::commands::SPLINE_WAYPOINT,
                context.frame.mav_frame(),
                lat,
                lng,
                altitude,
            ));
        }
        self.items.push(mp_mission::commands::set_roi(
            centre,
            0.0,
            context.frame.mav_frame(),
        ));
        self.items.extend(rows);
        self.renumber();
        self.origin = Origin::Edited;
        Ok(())
    }

    /// Why the rally points are not usable, if they are not.
    #[must_use]
    pub fn rally_error(&self) -> Option<&str> {
        self.rally_error.as_deref()
    }

    /// The rally points as the items the protocol carries.
    pub fn rally_items(&mut self) -> Option<Vec<MissionItem>> {
        if self.rally.is_empty() {
            self.rally_error = Some("place at least one rally point first".to_owned());
            return None;
        }
        self.rally_error = None;
        Some(
            self.rally
                .iter()
                .enumerate()
                .map(|(index, point)| point.to_item(u16::try_from(index).unwrap_or(u16::MAX)))
                .collect(),
        )
    }

    /// Replaces the rally points with what the vehicle reported.
    pub fn adopt_rally(&mut self, items: &[MissionItem]) {
        self.rally = items
            .iter()
            .filter_map(|item| {
                let position = item.position().ok().flatten()?;
                Some(RallyPoint {
                    position,
                    altitude: item.z,
                    break_altitude: (item.param2 != 0.0).then_some(item.param2),
                })
            })
            .collect();
        self.rally_error = self
            .rally
            .is_empty()
            .then(|| "the vehicle holds no rally points".to_owned());
    }

    /// Why the fence is not usable, if it is not.
    #[must_use]
    pub fn fence_error(&self) -> Option<&str> {
        self.fence_error.as_deref()
    }

    /// The fence as the items the protocol carries, or `None` if it is not valid.
    ///
    /// An inclusion polygon: the vehicle must stay inside it. Exclusion zones and circles exist in
    /// the protocol and are not offered here yet, because a fence drawn wrongly is worse than no
    /// fence - it either does nothing or triggers a return-to-launch in flight.
    pub fn fence_items(&mut self) -> Option<Vec<MissionItem>> {
        let mut polygons = Vec::new();
        // The inclusion polygon, when there is one; a fence of exclusions alone is a fence.
        if !self.fence.is_empty() || self.fence_exclusions.is_empty() {
            polygons.push(FenceItem::Polygon {
                inclusion: true,
                vertices: self.fence.clone(),
            });
        }
        polygons.extend(
            self.fence_exclusions
                .iter()
                .map(|vertices| FenceItem::Polygon {
                    inclusion: false,
                    vertices: vertices.clone(),
                }),
        );
        let mut items = Vec::new();
        for polygon in &polygons {
            if let Err(why) = polygon.validate() {
                self.fence_error = Some(why.to_string());
                return None;
            }
            let seq = u16::try_from(items.len()).unwrap_or(u16::MAX);
            items.extend(polygon.to_items(seq));
        }
        self.fence_error = None;
        Some(items)
    }

    /// Replaces the fence with what the vehicle reported.
    ///
    /// Only the first inclusion polygon is shown. A vehicle can hold several fences and this
    /// editor draws one; saying so is better than silently showing a fraction of what is loaded.
    pub fn adopt_fence(&mut self, items: &[MissionItem]) {
        match mp_mission::fences_from_items(items) {
            Ok(fences) => {
                // The exclusion polygons are kept whole; the inclusions are the one shown.
                self.fence_exclusions = fences
                    .iter()
                    .filter_map(|fence| match fence {
                        FenceItem::Polygon {
                            inclusion: false,
                            vertices,
                        } => Some(vertices.clone()),
                        _ => None,
                    })
                    .collect();
                let polygons: Vec<&FenceItem> = fences
                    .iter()
                    .filter(|fence| {
                        matches!(
                            fence,
                            FenceItem::Polygon {
                                inclusion: true,
                                ..
                            }
                        )
                    })
                    .collect();
                match polygons.first() {
                    Some(FenceItem::Polygon { vertices, .. }) => {
                        self.fence = vertices.clone();
                        self.fence_error = (polygons.len() > 1).then(|| {
                            format!(
                                "the vehicle holds {} polygons; showing the first",
                                polygons.len()
                            )
                        });
                    }
                    _ => {
                        self.fence.clear();
                        self.fence_error = Some("the vehicle holds no polygon fence".to_owned());
                    }
                }
            }
            Err(why) => self.fence_error = Some(why.to_string()),
        }
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

    /// The survey area as the map shows it: its corners, or none while Geo-Fence > Clear has
    /// taken it off the map.
    #[must_use]
    pub fn shown_polygon(&self) -> &[LatLon] {
        if self.polygon_hidden {
            &[]
        } else {
            &self.polygon
        }
    }

    /// Takes the survey area off the map and keeps its corners: `drawnpolygonsoverlay` cleared.
    pub const fn hide_polygon(&mut self) {
        self.polygon_hidden = true;
    }

    /// Adds a vertex to the survey area.
    pub fn add_area_vertex(&mut self, position: LatLon) {
        self.polygon.push(position);
        self.polygon_hidden = false;
    }

    /// Removes the last vertex, which is the undo an operator reaches for while drawing.
    pub fn undo_area_vertex(&mut self) {
        self.polygon.pop();
        self.polygon_hidden = false;
    }

    /// Discards the survey area, leaving the mission alone.
    pub fn clear_area(&mut self) {
        self.polygon.clear();
        self.polygon_hidden = false;
    }

    /// Numbers the rows 1..n, as the grid's headers read, so the sequence has no gaps.
    fn renumber(&mut self) {
        mp_mission::rows::number_rows(&mut self.items);
    }

    /// `Commands.Rows.Add()` and the handler filling the new row: an item at the end.
    pub fn append(&mut self, item: MissionItem) {
        self.items.push(item);
        self.renumber();
        self.origin = Origin::Edited;
    }

    /// Insert Wp and Insert Spline WP: an item after the waypoint numbered `wpno`, or the C#'s
    /// refusal if it would throw on the number. The new row becomes the selected one, as
    /// `selectedrow = int.Parse(wpno)` makes it.
    /// `// C#: GCSViews/FlightPlanner.cs:4070-4091`
    pub fn insert_after(&mut self, wpno: &str, item: MissionItem) -> Result<u16, ()> {
        let index = mp_mission::rows::insert_index(self.items.len(), wpno).ok_or(())?;
        self.items.insert(index, item);
        self.renumber();
        self.origin = Origin::Edited;
        let seq = self.items.get(index).map_or(u16::MAX, |item| item.seq);
        self.selected = Some(seq);
        Ok(seq)
    }

    /// The number Insert Wp offers, `(selectedrow + 1)`: straight after the selected row.
    #[must_use]
    pub fn insert_offer(&self) -> usize {
        let selected_row = self
            .selected
            .and_then(|seq| self.items.iter().position(|item| item.seq == seq));
        mp_mission::rows::insert_offer(self.items.len(), selected_row)
    }

    /// Delete WP on the marker under the cursor: `Commands.Rows.RemoveAt(no - 1); // home is 0`.
    /// Only rows have markers here, so home is never the one under it.
    /// `// C#: GCSViews/FlightPlanner.cs:3118-3135`
    pub fn delete_marker(&mut self, seq: u16) -> bool {
        if !self.items.iter().any(|item| item.seq == seq) {
            return false;
        }
        self.remove(seq);
        true
    }

    /// Add Below: a row as `Commands_RowsAdded` leaves one - `WAYPOINT`, zeroes, the screen's
    /// frame - added after the current row, or at the end when there is at most one row or the
    /// current row is the last. With no current row the C# takes row 0 as current, so with two
    /// rows or more the new one goes after the first. The new row becomes the selected one, as
    /// `Commands_RowValidating` makes it `selectedrow`.
    /// `// C#: GCSViews/FlightPlanner.cs:1780-1811, 2407-2441, 2472-2475`
    pub fn add_below(&mut self, frame: AltitudeFrame) {
        let current = self
            .selected
            .and_then(|seq| self.items.iter().position(|item| item.seq == seq))
            .unwrap_or(0);
        let count = self.items.len();
        let index = if count <= 1 || count == current + 1 {
            count
        } else {
            current + 1
        };
        self.items.insert(
            index,
            circle_row(
                mp_mission::commands::WAYPOINT,
                frame.mav_frame(),
                0.0,
                0.0,
                0.0,
            ),
        );
        self.renumber();
        self.selected = self.items.get(index).map(|item| item.seq);
        self.origin = Origin::Edited;
    }

    /// Clear Mission: `Commands.Rows.Clear()`. Every row goes; home, not being one, stays.
    /// `// C#: GCSViews/FlightPlanner.cs:2064-2082`
    pub fn clear_mission(&mut self) {
        self.items.clear();
        self.selected = None;
        self.origin = Origin::Edited;
    }

    /// Reverse WPs: every row taken from the end and added again, so the grid is reversed.
    /// `// C#: GCSViews/FlightPlanner.cs:5840-5858`
    pub fn reverse_waypoints(&mut self) {
        self.items.reverse();
        self.renumber();
        self.selected = None;
        self.origin = Origin::Edited;
    }

    /// Modify Alt: every row's altitude times a factor plus a change. `"*2"` doubles, `"20"`
    /// adds twenty; anything `float.Parse` refuses is refused.
    /// `// C#: GCSViews/FlightPlanner.cs:4921-4952`
    pub fn modify_altitudes(&mut self, altdif: &str) -> Result<(), ()> {
        let (multiplier, change) = if altdif.contains('*') {
            (
                altdif
                    .replace('*', " ")
                    .trim()
                    .parse::<f32>()
                    .map_err(|_| ())?,
                0.0,
            )
        } else {
            (1.0, altdif.trim().parse::<f32>().map_err(|_| ())?)
        };
        for item in &mut self.items {
            // The C# does this in float: `float.Parse(cell) * multiplyer + altchange`.
            #[allow(clippy::cast_possible_truncation)] // the C#'s float arithmetic, on purpose
            let altitude = item.z as f32;
            item.z = f64::from(altitude * multiplier + change);
        }
        self.origin = Origin::Edited;
        Ok(())
    }

    /// Polygon > Draw a Polygon: the first choice starts drawing (`polygongridmode = true`), and
    /// each one after adds the corner under the menu.
    /// `// C#: GCSViews/FlightPlanner.cs:1731-1756`
    pub fn draw_polygon(&mut self, position: LatLon) {
        if self.draw_mode != DrawMode::Area {
            self.draw_mode = DrawMode::Area;
            return;
        }
        self.add_area_vertex(position);
    }

    /// Polygon > Clear Polygon: the corners go, and drawing stops.
    /// `// C#: GCSViews/FlightPlanner.cs:2084-2094`
    pub fn clear_polygon(&mut self) {
        if self.draw_mode == DrawMode::Area {
            self.draw_mode = DrawMode::Waypoints;
        }
        self.clear_area();
    }

    /// Polygon > From Current Waypoints: the survey polygon becomes the `WAYPOINT` rows. Nothing
    /// happens without rows, and whether to clear the mission afterwards is the operator's
    /// question to answer - the caller asks it.
    /// `// C#: GCSViews/FlightPlanner.cs:3619-3637`
    pub fn polygon_from_waypoints(&mut self) -> bool {
        if self.items.is_empty() {
            return false;
        }
        self.polygon = mp_mission::rows::waypoint_positions(&self.items);
        self.polygon_hidden = false;
        true
    }

    /// `AddWPToMap`: what a click on the map adds, by what is being drawn.
    ///
    /// Drawing a polygon adds a corner; the geofence and rally modes are `cmb_missiontype` set to
    /// FENCE and RALLY; otherwise a waypoint, in the screen's altitude frame.
    /// `// C#: GCSViews/FlightPlanner.cs:558-600`
    pub fn add_wp_to_map(&mut self, position: LatLon, altitude: f64, frame: AltitudeFrame) {
        match self.draw_mode {
            DrawMode::Waypoints => {
                self.add_waypoint_in(position, altitude, frame);
                // `else if (splinemode)`: a SPLINE_WAYPOINT where it would be a WAYPOINT.
                if self.panel.spline
                    && let Some(row) = self.items.last_mut()
                {
                    row.command = mp_mission::commands::SPLINE_WAYPOINT;
                }
            }
            DrawMode::Area => self.add_area_vertex(position),
            DrawMode::Fence => self.add_fence_vertex(position),
            DrawMode::Rally => self.add_rally_point(position),
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

/// The altitude the draw panel's rally mode places a rally point at.
///
/// It was the survey panel's altitude, the only other altitude on this screen, which started at
/// 50 m; that panel is now the Survey (Grid) dialog (`survey_ui.rs`), so the rally points keep the
/// number it started at.
pub const RALLY_ALTITUDE: f64 = 50.0;

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

/// What the strip at the head of the waypoint table needs.
pub struct StripState<'a> {
    /// The panel boxes' focus.
    pub focus: &'a PanelFocus,
    /// Which of them has the keyboard.
    pub focused: [bool; 4],
    /// `CMB_altmode`.
    pub frame: AltitudeFrame,
    /// Whether the Spline check box shows: only on a copter.
    pub spline_visible: bool,
}

/// A check box: a box, ticked or not, and its text.
fn check_box(
    id: &'static str,
    text: &'static str,
    checked: bool,
    enabled: bool,
) -> gpui::Stateful<gpui::Div> {
    let colour = if enabled { theme::TEXT } else { theme::DIM };
    crate::probe::measured(id, div())
        .id(id)
        .flex()
        .items_center()
        .gap_1()
        .text_xs()
        .text_color(rgb(colour))
        .child(
            div()
                .w(px(11.0))
                .h(px(11.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if enabled {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(if checked {
                    theme::ACCENT
                } else {
                    theme::ACTION
                })),
        )
        .child(text)
}

/// The head of `panelWaypoints`, left to right as the `.resx` places it over the `Commands`
/// grid: WP Radius, Loiter Radius and Default Alt with their labels above them, Add Below, then
/// the altitude frame (`CMB_altmode`), Verify Height, and the Spline and MAVFTP check boxes.
///
/// `CMB_altmode` is a combo box there and three buttons here, because gpui has no combo and three
/// values do not need one; its handler keeps the choice for the next session as `FPaltmode` does.
/// MAVFTP is drawn dimmed: this application has no MAVFTP mission transfer to switch to.
///
/// One row, as the `.resx` has it, wrapping only when the grid is too narrow for it (the owner,
/// 2026-10-04: the grid's top third was wasted space).
/// `// C#: GCSViews/FlightPlanner.resx (panelWaypoints); GCSViews/FlightPlanner.cs:234-239, 2157-2168, 8403-8406`
pub fn waypoint_strip(
    plan: &Plan,
    state: &StripState<'_>,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    div()
        .flex()
        .flex_wrap()
        .items_end()
        .gap_3()
        .child(strip_boxes(plan, state, cx))
        .child(strip_checks(plan, state, cx))
        .into_any_element()
}

/// The strip's first row: the boxes with their labels, Add Below, then Alt Warn.
fn strip_boxes(plan: &Plan, state: &StripState<'_>, cx: &mut Context<MissionPlanner>) -> gpui::Div {
    let mut boxes = div().flex().items_end().gap_2();
    for (index, which) in PanelBox::ALL.into_iter().enumerate() {
        // Alt Warn sits right of Add Below in the `.resx` (479 to its 398).
        if which == PanelBox::AltWarn {
            continue;
        }
        boxes = boxes.children(panel_box(plan, state, index, which, cx));
    }
    // `BUT_Add`, "Add Below", to the right of the boxes at (398, 8) in `panelWaypoints`.
    // `// C#: GCSViews/FlightPlanner.resx (BUT_Add.Location, BUT_Add.Text); FlightPlanner.cs:1780`
    boxes = boxes.child(action(
        "plan-add-below",
        "Add Below",
        theme::TEXT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.plan.add_below(this.altitude_frame);
            this.sync_map_mission();
            cx.notify();
        }),
    ));
    // `label17` "Alt Warn" over `TXT_altwarn` at (482, 20).
    boxes.children(panel_box(plan, state, 3, PanelBox::AltWarn, cx))
}

/// One panel box with its label above it, or nothing for an index the focus has no handle for.
fn panel_box(
    plan: &Plan,
    state: &StripState<'_>,
    index: usize,
    which: PanelBox,
    cx: &mut Context<MissionPlanner>,
) -> Option<gpui::Div> {
    {
        let (Some(handle), Some(focused)) = (
            state.focus.handles.get(index),
            state.focused.get(index).copied(),
        ) else {
            return None;
        };
        let enabled = which != PanelBox::LoiterRadius || plan.loiter_enabled();
        let label = div()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .child(which.label());
        let field = if enabled {
            crate::textfield::text_field(
                which.id(),
                plan.panel_field(which),
                handle,
                focused,
                px(64.0),
                cx.listener(move |this, event: &gpui::KeyDownEvent, _window, cx| {
                    if this.plan.panel_key(which, event) == crate::textfield::KeyOutcome::Ignored {
                        return;
                    }
                    cx.notify();
                }),
            )
            .into_any_element()
        } else {
            // `TXT_loiterrad.Enabled = false`: shown, greyed, and not typed into.
            crate::probe::measured(which.id(), div())
                .w(px(64.0))
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .text_sm()
                .text_color(rgb(theme::DIM))
                .child(plan.panel_text(which).to_owned())
                .into_any_element()
        };
        Some(div().flex().flex_col().gap_1().child(label).child(field))
    }
}

/// The strip's second row: the altitude frame, Verify Height, Spline and MAVFTP.
fn strip_checks(
    plan: &Plan,
    state: &StripState<'_>,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Div {
    let mut frames = div().flex().items_center().gap_1();
    for choice in AltitudeFrame::all() {
        let chosen = choice == state.frame;
        frames = frames.child(
            crate::probe::measured(format!("plan-frame-{}", choice.key()), div())
                .id(gpui::SharedString::from(format!("frame-{}", choice.key())))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(choice.combo_text())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.set_altitude_frame(choice);
                    cx.notify();
                })),
        );
    }
    // Choosing Terrain says nothing and touches no row: `CMB_altmode_SelectedIndexChanged` only
    // keeps the choice (`currentaltmode`, `FPaltmode`). (A note about TERRAIN_ENABLE shown here
    // under Terrain was this application's own, not the C#'s, and is gone.)
    // `// C#: GCSViews/FlightPlanner.cs:2157-2168`

    // `CHK_verifyheight`, "Verify Height", right of `CMB_altmode` at (298, 13), shown as
    // `DisplayConfiguration.displayCheckHeightBox` has it - true in every view. Its only handler
    // is `setfromMap` reading it.
    // `// C#: GCSViews/FlightPlanner.resx (CHK_verifyheight); GCSViews/FlightPlanner.cs:1332; ExtLibs/Utilities/DisplayView.cs:169`
    let verify = {
        let checked = plan.verify_height();
        check_box("plan-verifyheight", "Verify Height", checked, true)
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.set_verify_height(!checked);
                cx.notify();
            }))
    };

    let spline = state.spline_visible.then(|| {
        let checked = plan.spline();
        check_box("plan-spline", "Spline", checked, true)
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.set_spline(!checked);
                cx.notify();
            }))
    });
    // `chk_usemavftp`, "MAVFTP", at (589, 12): Read and Write go through `@MISSION/mission.dat`
    // while it is ticked, and the tick is kept as `UseMissionMAVFTP`.
    // `// C#: GCSViews/FlightPlanner.cs:319, 8403-8406`
    let mavftp = {
        let checked = plan.use_mavftp();
        check_box("plan-mavftp", "MAVFTP", checked, true)
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan.set_use_mavftp(!checked);
                this.persisted
                    .set("UseMissionMAVFTP", if checked { "False" } else { "True" });
                cx.notify();
            }))
    };
    let checks = div()
        .flex()
        .items_center()
        .gap_3()
        .children(spline)
        .child(mavftp);

    div()
        .flex()
        .flex_wrap()
        .items_center()
        .gap_3()
        .child(frames)
        .child(verify)
        .child(checks)
}

/// The waypoint table. It takes the height it is given - under the map, or at its right after
/// Switch Docking - and its rows scroll in what is left.
pub fn items_panel(
    plan_items: &[MissionItem],
    selected: Option<u16>,
    strip: AnyElement,
    minimised: bool,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    // `but_mincommands` at (938, 0), anchored top right: ˅ folds `panelWaypoints` to the
    // button's own height, so only it remains; ˄ opens it to 166 again. On the title's row, not
    // a row of its own (the owner, 2026-10-04: the grid's top third was wasted space).
    // `// C#: GCSViews/FlightPlanner.cs:69-80; GCSViews/FlightPlanner.resx (but_mincommands)`
    let min_button = div().child(action(
        "plan-mincommands",
        if minimised { "˄" } else { "˅" },
        theme::TEXT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.plan.toggle_commands_minimised();
            cx.notify();
        }),
    ));
    if minimised {
        return crate::ui::panel_with_corner("mission items", min_button, div());
    }
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
                // Which frame that altitude is in. The C# shows it as a `Frame` column on every
                // row, and it has to be visible: "50 m" means three different heights depending
                // on this, and one of them flies into a hill.
                // `// C#: GCSViews/FlightPlanner.cs:262, 2347`
                .child(
                    div()
                        .w(px(60.0))
                        .text_color(rgb(if item.frame == FRAME_TERRAIN {
                            theme::WARN
                        } else {
                            theme::DIM
                        }))
                        .child(frame_label(item.frame)),
                )
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
    let list = div()
        .id("plan-items")
        .flex()
        .flex_col()
        .overflow_y_scroll()
        .child(rows);
    let body = div()
        .flex()
        .flex_col()
        .gap_2()
        .child(strip)
        // The rows take no height of their own: the area is as tall as the editor beside them
        // needs, and they scroll in it.
        .child(list.flex_1().flex_basis(px(0.0)).min_h(px(0.0)))
        .children((count > 8).then(|| {
            div()
                .pt_1()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(format!("{count} items - scroll for the rest"))
        }));
    crate::ui::panel_with_corner("mission items", min_button, body.flex_1().min_h(px(0.0)))
        .flex_1()
        .min_w(px(0.0))
        .min_h(px(0.0))
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

    // What this command's parameters mean, from the MAVLink definitions. A command that uses none
    // of them - return-to-launch, land - shows none, rather than four controls doing nothing.
    let mut parameters = div().flex().flex_col().gap_1();
    let mut any_parameters = false;
    for (index, described) in MavCmd(u32::from(current_command))
        .parameters()
        .iter()
        .enumerate()
    {
        let Some((label, units)) = described else {
            continue;
        };
        any_parameters = true;
        let value =
            plan_items
                .iter()
                .find(|item| item.seq == seq)
                .map_or(0.0, |item| match index {
                    0 => item.param1,
                    1 => item.param2,
                    2 => item.param3,
                    _ => item.param4,
                });
        let shown = if units.is_empty() {
            format!("{value:.0}")
        } else {
            format!("{value:.0} {units}")
        };
        parameters = parameters.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(96.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child((*label).to_owned()),
                )
                .child(
                    div()
                        .w(px(68.0))
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(shown),
                )
                .child(param_stepper(seq, index, -1.0, "-1", cx))
                .child(param_stepper(seq, index, 1.0, "+1", cx))
                .child(param_stepper(seq, index, 10.0, "+10", cx)),
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
            .child(steppers)
            .children(any_parameters.then(|| {
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child("command parameters")
            }))
            .child(parameters),
    )
    .into_any_element()
}

/// One step button for a command parameter.
///
/// Parameters are clamped at zero below. Every one the editor offers is a count, a duration, a
/// radius or an angle, and a negative loiter time is not a thing a vehicle can fly. The exception
/// the definitions do allow - a negative loiter radius, meaning counter-clockwise - is a direction
/// rather than a magnitude, and belongs in a control that says so rather than in a stepper that
/// happens to go below zero.
fn param_stepper(
    seq: u16,
    index: usize,
    delta: f64,
    label: &'static str,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    crate::probe::measured(format!("param-{index}-{label}"), div())
        .id((label, index))
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
            let current = this.plan.param(seq, index).unwrap_or(0.0);
            this.plan.set_param(seq, index, (current + delta).max(0.0));
            cx.notify();
        }))
}

/// Everything drawn on the map: which mode a click is in, and the controls for that mode.
///
/// One panel rather than four. The survey area, the geofence and the rally points are all placed
/// by clicking, so the question is always "what does a click do now" - and four panels each
/// answering it separately made a sidebar that had to be scrolled to find out.
pub struct DrawState<'a> {
    /// What a click does.
    pub mode: DrawMode,
    /// Survey area corners placed.
    pub area_vertices: usize,
    /// Fence corners placed.
    pub fence_vertices: usize,
    /// Why the fence is unusable, if it is.
    pub fence_error: Option<&'a str>,
    /// Rally points placed.
    pub rally_points: usize,
    /// Why the rally points are unusable, if they are.
    pub rally_error: Option<&'a str>,
    /// Geo-Fence > Upload or Download under way ([`Plan::fence_busy`]).
    pub fence_busy: bool,
}

/// The draw panel.
pub fn draw_panel(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let mut modes = div().flex().flex_wrap().gap_1();
    for mode in DrawMode::ALL {
        let selected = mode == state.mode;
        modes = modes.child(
            crate::probe::measured(mode.id(), div())
                .id(mode.id())
                .px_2()
                .py_1()
                .rounded_md()
                .border_1()
                .border_color(rgb(if selected {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(if selected {
                    theme::ACTION
                } else {
                    theme::PANEL
                }))
                .text_xs()
                .text_color(rgb(if selected { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(mode.label())
                .on_click(cx.listener(move |this, _event, window, cx| {
                    draw_mode_clicked(this, mode, window, cx);
                    cx.notify();
                })),
        );
    }

    panel(
        "draw",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(modes)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(state.mode.hint()),
            )
            .child(mode_controls(state, view, cx)),
    )
}

/// The controls belonging to the active mode.
fn mode_controls(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    match state.mode {
        DrawMode::Waypoints => div()
            .text_xs()
            .text_color(rgb(theme::DIM))
            .child("drag a waypoint to move it; select one to change what it does")
            .into_any_element(),
        DrawMode::Area => survey_controls(state, cx).into_any_element(),
        DrawMode::Fence => fence_controls(state, view, cx).into_any_element(),
        DrawMode::Rally => rally_controls(state, view, cx).into_any_element(),
    }
}

/// Undo and clear for the survey area's corners. How the survey is flown is the Survey (Grid)
/// dialog's, from the map menu's Auto WP (`survey_ui.rs`).
fn survey_controls(state: &DrawState<'_>, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
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
                    "survey-undo",
                    "undo corner",
                    theme::TEXT,
                    state.area_vertices > 0,
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
                    state.area_vertices > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.clear_area();
                        this.sync_map_polygon();
                        cx.notify();
                    }),
                )),
        )
        .child(corner_count(state.area_vertices, "corners"))
}

/// Undo, clear, read and write for the geofence.
///
/// Read and write here are the mission protocol's fence list, `MAV_MISSION_TYPE_FENCE` - the C#'s
/// Read and Write with `cmb_missiontype` set to FENCE - and were this port's only way to the
/// vehicle's fence before the map menu's Geo-Fence > Upload and Download were ported; those speak
/// the legacy `FENCE_POINT` protocol as the C#'s do ([`start_fence_upload`],
/// [`start_fence_download`]), and Download reuses this read on a vehicle with `MISSION_FENCE`.
/// Read and write wait while one of the menu's runs, as the C#'s modal calls would keep them.
fn fence_controls(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some() && !state.fence_busy;
    // Three is the fewest that encloses anything; the protocol and the vehicle both refuse fewer.
    let usable = state.fence_vertices >= 3;

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
                    "fence-undo",
                    "undo corner",
                    theme::TEXT,
                    state.fence_vertices > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.undo_fence_vertex();
                        this.sync_map_fence();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fence-clear",
                    "clear fence",
                    theme::TEXT,
                    state.fence_vertices > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.clear_fence();
                        this.sync_map_fence();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fence-read",
                    "read",
                    theme::ACCENT,
                    has_vehicle,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.request_fence();
                        this.adopt_vehicle_fence = true;
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fence-write",
                    "write",
                    theme::WARN,
                    has_vehicle && usable,
                    cx.listener(|this, _event: &(), _window, cx| {
                        if let Some(items) = this.plan.fence_items() {
                            this.telemetry.upload_fence(items);
                        }
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(if usable { theme::OK } else { theme::DIM }))
                .child(match state.fence_vertices {
                    0 => "no fence".to_owned(),
                    1 | 2 => format!("{} of at least 3 corners", state.fence_vertices),
                    n => format!("{n} corners - the vehicle must stay inside"),
                }),
        )
        .children(state.fence_error.map(problem))
}

/// Undo, clear, read and write for rally points.
fn rally_controls(
    state: &DrawState<'_>,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let has_vehicle = view.vehicle.is_some();

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
                    "rally-undo",
                    "undo point",
                    theme::TEXT,
                    state.rally_points > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.undo_rally_point();
                        this.sync_map_rally();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "rally-clear",
                    "clear",
                    theme::TEXT,
                    state.rally_points > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan.clear_rally();
                        this.sync_map_rally();
                        cx.notify();
                    }),
                ))
                .child(action(
                    "rally-read",
                    "read",
                    theme::ACCENT,
                    has_vehicle,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.telemetry.request_rally();
                        this.adopt_vehicle_rally = true;
                        cx.notify();
                    }),
                ))
                .child(action(
                    "rally-write",
                    "write",
                    theme::WARN,
                    has_vehicle && state.rally_points > 0,
                    cx.listener(|this, _event: &(), _window, cx| {
                        if let Some(items) = this.plan.rally_items() {
                            this.telemetry.upload_rally(items);
                        }
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(if state.rally_points > 0 {
                    theme::OK
                } else {
                    theme::DIM
                }))
                .child(match state.rally_points {
                    0 => "no rally points - a failsafe returns all the way home".to_owned(),
                    1 => "1 rally point".to_owned(),
                    n => format!("{n} rally points"),
                }),
        )
        .children(state.rally_error.map(problem))
}

/// How many corners are placed, and whether that is enough.
fn corner_count(count: usize, noun: &'static str) -> impl IntoElement {
    div()
        .text_xs()
        .text_color(rgb(if count >= 3 { theme::OK } else { theme::DIM }))
        .child(match count {
            0 => format!("no {noun} yet"),
            1 | 2 => format!("{count} of at least 3 {noun}"),
            n => format!("{n} {noun}"),
        })
}

/// A problem, said in the alert colour.
fn problem(text: &str) -> impl IntoElement {
    div()
        .text_xs()
        .text_color(rgb(theme::ALERT))
        .child(text.to_owned())
}

/// Read, write, load, save and clear.
/// The mission file name field and its focus, which travel together everywhere.
pub struct NameField<'a> {
    /// What has been typed.
    pub field: &'a crate::textfield::TextField,
    /// Its focus handle.
    pub focus: &'a gpui::FocusHandle,
    /// Whether it currently has focus.
    pub focused: bool,
}
/// What the action panel shows besides the mission: `panel3`'s Grid box and status label, and
/// `panel4`'s pointer read-out, with a MAVFTP transfer's words when one is going.
pub struct ActionExtras<'a> {
    /// `progressBarInjectCustomMap`'s value and maximum while Inject Custom Map runs; `None`
    /// hides the bar, as `Visible = false` does.
    pub inject: Option<(usize, usize)>,
    /// `chk_grid.Checked`.
    pub grid: bool,
    /// `lbl_status`: `None` before the map has painted, then whether tiles are still loading.
    pub tiles_loading: Option<bool>,
    /// `coords1`.
    pub coords: &'a crate::coords::Coords,
    /// The mission going over MAVFTP, if one is.
    pub mission_ftp: Option<&'a MissionFtp>,
}

/// `coords1`, the `Coords` control with `Vertical` set: the system combo - three buttons here,
/// as the other combos are - with `AltSource` under it, and the position's lines to its right.
/// `// C#: ExtLibs/Controls/Coords.cs:100-170; ExtLibs/Controls/Coords.Designer.cs:24-33`
fn coords_panel(coords: &crate::coords::Coords, cx: &mut Context<MissionPlanner>) -> AnyElement {
    use crate::coords::CoordSystem;
    let mut combo = div().flex().items_center().gap_1();
    for system in CoordSystem::ALL {
        let chosen = coords.system == system;
        combo = combo.child(
            crate::probe::measured(system.id(), div())
                .id(system.id())
                .px_1()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(system.name())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.plan.set_coord_system(system);
                    cx.notify();
                })),
        );
    }
    let text = crate::probe::measured("plan-coords-text", div())
        .flex()
        .flex_col()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .children(coords.lines().into_iter().map(|line| div().child(line)));
    let source = div()
        .text_xs()
        .text_color(rgb(theme::DIM))
        .child(coords.alt_source.to_owned());
    crate::probe::measured("plan-coords", div())
        .flex()
        .items_start()
        .gap_3()
        .child(div().flex().flex_col().gap_1().child(combo).child(source))
        .child(text)
        .into_any_element()
}

pub fn actions_panel(
    plan_items: &[MissionItem],
    origin: &Origin,
    view: &TelemetryView,
    name: &NameField<'_>,
    tile_source: Option<&'static str>,
    extras: &ActionExtras<'_>,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let (name_focus, name_focused) = (name.focus, name.focused);
    let name = name.field;
    let has_vehicle = view.vehicle.is_some();
    let has_items = !plan_items.is_empty();
    // `panel4`, first in `flowLayoutPanel1`: the pointer read-out.
    let coords = coords_panel(extras.coords, cx);
    // `chk_grid` at (3, 3) of `panel3`, above the map type box.
    // `// C#: GCSViews/FlightPlanner.resx (chk_grid); GCSViews/FlightPlanner.cs:2053-2057`
    let grid = {
        let checked = extras.grid;
        check_box("plan-grid", "Grid", checked, true)
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, window, cx| {
                this.plan.set_grid(!checked);
                this.map.borrow_mut().set_grid(!checked);
                window.refresh();
                cx.notify();
            }))
    };
    // `lnk_kml` at (62, 3) of `panel3`, beside the Grid box: "View KML", a `LinkLabel`.
    // `// C#: GCSViews/FlightPlanner.resx (lnk_kml); GCSViews/FlightPlanner.cs:4318-4328`
    let kml_link = crate::probe::measured("plan-kml", div())
        .id("plan-kml")
        .whitespace_nowrap()
        .text_xs()
        .text_color(rgb(theme::ACCENT))
        .cursor_pointer()
        .hover(|style| style.underline())
        .child("View KML")
        .on_click(cx.listener(|this, _event, _window, cx| {
            this.view_kml_clicked();
            cx.notify();
        }));
    // `lbl_status` at (4, 46) of `panel3`: "Status" until the map loads tiles, then
    // `MainMap_OnTileLoadStart`'s "Status: loading tiles..." and `OnTileLoadComplete`'s
    // "Status: loaded tiles".
    // `// C#: GCSViews/FlightPlanner.resx (lbl_status); GCSViews/FlightPlanner.cs:8172, 8190`
    let status = crate::probe::measured("plan-status", div())
        .text_xs()
        .text_color(rgb(theme::DIM))
        .child(match extras.tiles_loading {
            None => "Status",
            Some(true) => "Status: loading tiles...",
            Some(false) => "Status: loaded tiles",
        });
    // The map provider - `comboBoxMapType`, which lives on this screen in the C# and changes the
    // flight screen's map with it. Buttons rather than a combo, because gpui has no combo and
    // three providers do not need one.
    // `// C#: GCSViews/FlightPlanner.cs:176-180`
    let mut providers = div()
        .flex()
        .items_center()
        .flex_wrap()
        .gap_1()
        .child(div().text_xs().text_color(rgb(theme::DIM)).child("map:"));
    for source in mp_tiles::source::SOURCES {
        let chosen = Some(source.id) == tile_source;
        providers = providers.child(
            crate::probe::measured(format!("plan-map-{}", source.id), div())
                .id(gpui::SharedString::from(format!("map-{}", source.id)))
                .px_2()
                .py(px(1.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(source.label)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.set_tile_source(source);
                    cx.notify();
                })),
        );
    }

    let transfer_line = if let Some(ftp) = extras.mission_ftp {
        let colour = if ftp.running() {
            theme::ACCENT
        } else if matches!(ftp, MissionFtp::Ended { ok: false, .. }) {
            theme::ALERT
        } else {
            theme::OK
        };
        div().text_xs().text_color(rgb(colour)).child(ftp.label())
    } else {
        view.transfer.as_ref().map_or_else(
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
        )
    };

    // `BUT_InjectCustomMap` at (3, 68) of `panel3`, 115 by 23, reading "Cancel" while a run is on,
    // and `progressBarInjectCustomMap` under it at (3, 97), 115 by 23, shown while it runs.
    // `// C#: GCSViews/FlightPlanner.resx (BUT_InjectCustomMap, progressBarInjectCustomMap);
    // GCSViews/FlightPlanner.cs:8416-8527`
    let inject = {
        let running = extras.inject.is_some();
        let mut row = div().flex().items_center().gap_2().child(action(
            "plan-inject",
            if running {
                crate::inject_map::CANCEL
            } else {
                crate::inject_map::BUTTON_TEXT
            },
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                this.inject_map_clicked(window, cx);
                cx.notify();
            }),
        ));
        if let Some((value, maximum)) = extras.inject {
            #[allow(clippy::cast_precision_loss)] // a count of files
            let fraction = value as f32 / maximum.max(1) as f32;
            row = row.child(
                crate::probe::measured("plan-inject-bar", div())
                    .w(px(115.0))
                    .h(px(23.0))
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .bg(rgb(theme::BG))
                    .child(
                        div()
                            .h_full()
                            .w(gpui::relative(fraction))
                            .bg(rgb(theme::OK)),
                    ),
            );
        }
        row
    };

    panel(
        "mission",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(coords)
            // `panel5`, the Mission box - Read WPs, Write WPs, Write Fast, Save WP File, Load WP
            // File - straight after `panel4`'s read-out, as `flowLayoutPanel1` flows them; the
            // map type, grid, status and inject rows this panel also carries come after, so a
            // strip that cuts the panel short cuts those and never the buttons (the owner's
            // report, 2026-10-03). `// C#: GCSViews/FlightPlanner.resx (panel4, panel5)`
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
                            read_from_vehicle(this);
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "plan-write",
                        "write to vehicle",
                        theme::WARN,
                        has_vehicle && has_items,
                        cx.listener(|this, _event: &(), window, cx| {
                            start_write(this, false, window, cx);
                        }),
                    ))
                    // `but_writewpfast`, "Write Fast", under Write at (3, 61) of `panel5`.
                    // `// C#: GCSViews/FlightPlanner.resx (but_writewpfast); GCSViews/FlightPlanner.cs:1895-1980`
                    .child(action(
                        "plan-writefast",
                        "Write Fast",
                        theme::WARN,
                        has_vehicle && has_items,
                        cx.listener(|this, _event: &(), window, cx| {
                            start_write(this, true, window, cx);
                        }),
                    ))
                    // `BUT_saveWPFile_Click` and `BUT_loadwpfile_Click`: each opens its file
                    // dialog, the planner's own box (the owner, 2026-10-04: they acted on the
                    // name field at once, and only in one folder).
                    // `// C#: GCSViews/FlightPlanner.cs:1817-1823, 1890-1893, 6069-6077`
                    .child(action(
                        "plan-save",
                        "save file",
                        theme::TEXT,
                        has_items,
                        cx.listener(|this, _event: &(), window, cx| {
                            ask_mission_save(this, window, cx);
                        }),
                    ))
                    .child(action(
                        "plan-load",
                        "load file",
                        theme::TEXT,
                        true,
                        cx.listener(|this, _event: &(), window, cx| {
                            this.plan_menus.ask_mission_load();
                            this.plan_prompt_focus.focus(window, cx);
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
            .child(providers.child(grid).child(kml_link))
            .child(status)
            .child(inject)
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(
                        div()
                            .w(px(36.0))
                            .text_xs()
                            .text_color(rgb(theme::DIM))
                            .child("file"),
                    )
                    .child(crate::textfield::text_field(
                        "plan-name",
                        name,
                        name_focus,
                        name_focused,
                        px(240.0),
                        cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
                            // Enter is Save File. A file name field where enter does nothing is
                            // a field that has to be followed by finding the button.
                            match this.plan_name.key(event) {
                                crate::textfield::KeyOutcome::Submitted => {
                                    ask_mission_save(this, window, cx);
                                    return;
                                }
                                crate::textfield::KeyOutcome::Ignored => return,
                                _ => {}
                            }
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

// ---------------------------------------------------------------------------------------------
// Home: the Home Location panel, and what the rest of the application tells it.
// ---------------------------------------------------------------------------------------------

/// The vehicle's home, `cs.HomeLocation`, once it has sent `HOME_POSITION`.
///
/// **The altitude is a stand-in.** The C# takes `home.altitude / 1000.0` from `HOME_POSITION`
/// (`MAVLinkInterface.cs:5703-5707`), and `VehicleState` in this tree keeps only the message's
/// position. Until it keeps the altitude too, this takes the one ArduPilot's own
/// `GLOBAL_POSITION_INT` implies - `alt` above sea level less `relative_alt` above home - which is
/// the same number on an ArduPilot that has a home. Swap it for the state's home altitude when
/// there is one.
#[must_use]
pub fn vehicle_home(view: &TelemetryView) -> Option<Home> {
    let state = view.state.as_ref()?;
    let home = state.home?;
    Some(Home {
        lat: home.latitude(),
        lng: home.longitude(),
        alt: state.altitude_msl.0 - state.altitude_relative.0,
    })
}

/// The planned home `MainV2` starts with: `TXT_homelat`, `TXT_homelng` and `TXT_homealt` from
/// Mission Planner's `config.xml`, where the planning screen saves its boxes, each `GetDouble` -
/// 0 when absent or unreadable - and no home at all if the position is off the globe.
/// `// C#: MainV2.cs:1012-1025; ExtLibs/Utilities/Settings.cs:245-254; GCSViews/FlightPlanner.cs:2576-2578`
#[must_use]
pub fn planned_home_from_config(config: Option<&mp_settings::Config>) -> Home {
    let Some(config) = config else {
        return Home::default();
    };
    let get = |key: &str| {
        config
            .get(key)
            .and_then(mp_mission::rows::parse_number)
            .unwrap_or(0.0)
    };
    let home = Home {
        lat: get("TXT_homelat"),
        lng: get("TXT_homelng"),
        alt: get("TXT_homealt"),
    };
    // "remove invalid entrys"
    if home.lat.abs() > 90.0 || home.lng.abs() > 180.0 {
        return Home::default();
    }
    home
}

/// `FlightPlanner.Activate`, run each time the planning screen is shown: `updateHome()`, then
/// `setWPParams()`, then the check that Default Alt is a whole number.
/// `// C#: GCSViews/FlightPlanner.cs:321-337, 1344-1349`
pub fn activate(this: &mut MissionPlanner) {
    let view = this.telemetry.view();
    this.plan.update_home_text(vehicle_home(&view));
    this.plan.set_wp_params(&view.parameters);
    if let Some(text) = this.plan.check_default_alt() {
        this.plan_menus.say("", text);
    }
}

/// `cs.firmware == Firmwares.ArduCopter2`, which it is until a vehicle says otherwise: what
/// shows the Spline check box and makes a Default Alt of 0 give 15 rather than 50.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:102; GCSViews/FlightPlanner.cs:313-317, 1207-1208`
#[must_use]
pub fn firmware_is_copter(view: &TelemetryView) -> bool {
    view.state
        .as_ref()
        .and_then(|state| mp_vehicle::VehicleFamily::from_mav_type(state.vehicle_type))
        .is_none_or(|family| family == mp_vehicle::VehicleFamily::Copter)
}

/// Where the planning map draws home: the Home Location boxes, once all three hold a number -
/// `writeKML`'s `home`, which `WPOverlay.CreateOverlay` draws as the "H" marker. A box that is
/// empty or does not parse leaves `new PointLatLngAlt()`, which is not drawn.
///
/// (With a box that is not empty and does not parse, `writeKML` also says "Invalid home
/// location" - on every keystroke that leaves it so, a `-` typed first included. Not ported.)
/// `// C#: GCSViews/FlightPlanner.cs:1400-1415; ExtLibs/Maps/WPOverlay.cs:44-50`
#[must_use]
pub fn planner_map_home(plan: &Plan) -> Option<LatLon> {
    let home = plan.home()?;
    LatLon::new(home.lat, home.lng).ok()
}

/// Where the flight map draws home: the vehicle's `cs.HomeLocation`, or `cs.PlannedHomeLocation`
/// while that is 0,0, and only once the vehicle's mission is held (`MAV.wps.Count >= 1`), because
/// the flight screen draws home as part of the vehicle's mission overlay.
///
/// Where both are 0,0 the C# draws the marker there, off the coast of Africa; this map frames
/// the home it is given, so it is given none.
/// `// C#: GCSViews/FlightData.cs:3808-3845`
#[must_use]
pub fn flight_map_home(
    vehicle: Option<LatLon>,
    mission_held: bool,
    planned: Home,
) -> Option<LatLon> {
    if !mission_held {
        return None;
    }
    if let Some(home) = vehicle.filter(|home| home.latitude() != 0.0 || home.longitude() != 0.0) {
        return Some(home);
    }
    if planned.lat == 0.0 && planned.lng == 0.0 {
        return None;
    }
    LatLon::new(planned.lat, planned.lng).ok()
}

/// The panel boxes' focus handles, and which had the keyboard at the last frame.
pub struct PanelFocus {
    /// One per box, in [`PanelBox::ALL`] order.
    pub handles: [gpui::FocusHandle; 4],
    was: [bool; 4],
    /// Whether the Home Location Lat box had the keyboard at the last frame, for its `Enter`.
    home_lat_was: bool,
}

impl PanelFocus {
    /// Three handles, none focused.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            handles: [
                cx.focus_handle(),
                cx.focus_handle(),
                cx.focus_handle(),
                cx.focus_handle(),
            ],
            was: [false; 4],
            home_lat_was: false,
        }
    }
}

/// `TXT_*_Leave` for a panel box that has the keyboard, and the keyboard taken from it: what a
/// press on the map or on Write does in the C#, where both take the focus. gpui leaves the focus
/// where it was when something that cannot take it is clicked, so it is taken here.
pub fn leave_panel_boxes(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let mut left = false;
    for (index, which) in PanelBox::ALL.into_iter().enumerate() {
        if this
            .plan_panel_focus
            .handles
            .get(index)
            .is_some_and(|handle| handle.is_focused(window))
        {
            this.plan.panel_leave(which);
            left = true;
        }
    }
    if left {
        window.blur(cx);
    }
}

/// `TXT_homelat_Enter` for the Lat box having taken the keyboard since the last frame: the first
/// time, "Click on the Map to set Home ", and the next click on the map moves home.
/// `// C#: GCSViews/FlightPlanner.Designer.cs (TXT_homelat.Enter); GCSViews/FlightPlanner.cs:7016-7022`
pub fn track_home_focus(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let now = this
        .plan_home_focus
        .first()
        .is_some_and(|handle| handle.is_focused(window));
    let entered = now && !this.plan_panel_focus.home_lat_was;
    this.plan_panel_focus.home_lat_was = now;
    if !entered {
        return;
    }
    if let Some(text) = this.plan.home_lat_enter() {
        this.plan_menus.say_home_hint(text);
        this.plan_prompt_focus.focus(window, cx);
        cx.notify();
    }
}

/// `Leave` for a panel box that has lost the keyboard some other way since the last frame - to
/// another box, say.
pub fn track_panel_focus(this: &mut MissionPlanner, window: &gpui::Window) {
    for (index, which) in PanelBox::ALL.into_iter().enumerate() {
        let (Some(handle), Some(was)) = (
            this.plan_panel_focus.handles.get(index),
            this.plan_panel_focus.was.get(index).copied(),
        ) else {
            continue;
        };
        let now = handle.is_focused(window);
        if was && !now {
            this.plan.panel_leave(which);
        }
        if let Some(slot) = this.plan_panel_focus.was.get_mut(index) {
            *slot = now;
        }
    }
}

/// The vehicle family `cs.firmware` names, which picks `mavcmd.xml`'s command set: none until a
/// vehicle says, which reads as a copter's (`AC2`).
/// `// C#: GCSViews/FlightPlanner.cs:5700-5716`
#[must_use]
pub fn firmware_family(view: &TelemetryView) -> Option<mp_vehicle::VehicleFamily> {
    view.state
        .as_ref()
        .and_then(|state| mp_vehicle::VehicleFamily::from_mav_type(state.vehicle_type))
}

/// A row grabbed on the map: where it is now is what its drop measures the ground from.
pub fn waypoint_grabbed(this: &mut MissionPlanner, seq: u16) {
    this.plan.begin_drag(seq);
}

/// A grabbed row let go on the map: `callMeDrag(tag, lat, lng, -2)`, which with Verify Height
/// keeps the row's height above the ground ([`Plan::end_drag`]).
/// `// C#: GCSViews/FlightPlanner.cs:7803-7810, 739-777`
pub fn waypoint_dropped(this: &mut MissionPlanner, seq: u16) {
    let family = firmware_family(&this.telemetry.view());
    this.plan.end_drag(seq, this.altitude_frame, family);
    this.sync_map_mission();
}

/// A click on the map that adds something: `AddWPToMap(lat, lng, 0)` - home moved there when the
/// Lat box has asked for it, otherwise the altitude from Default Alt as `setfromMap` takes it for
/// a waypoint, Verify Height and all, or "Your default alt is not valid", and nothing.
/// `// C#: GCSViews/FlightPlanner.cs:558-600, 1164-1236, 7743`
pub fn map_click(
    this: &mut MissionPlanner,
    position: LatLon,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    if this.plan.click_sets_home(position) {
        return;
    }
    let altitude = if this.plan.draw_mode() == DrawMode::Waypoints {
        let copter = firmware_is_copter(&this.telemetry.view());
        match this.plan.new_row_altitude(0.0, copter) {
            Ok(altitude) => this.plan.verified_altitude(
                position.latitude(),
                position.longitude(),
                altitude,
                this.altitude_frame,
            ),
            Err(why) => {
                this.plan_menus.say("", why);
                this.plan_prompt_focus.focus(window, cx);
                return;
            }
        }
    } else {
        0.0
    };
    this.plan
        .add_wp_to_map(position, altitude, this.altitude_frame);
}

/// Geo-Fence > Clear: `FENCE_ENABLE`, `FENCE_ACTION` and `FENCE_TOTAL` set to 0, one after
/// another, stopping at "Failed to set ..." when one goes unanswered; then the geofence cleared
/// from the map. A parameter the vehicle does not have is passed over, as `setParam` returns
/// false for it rather than throwing.
/// `// C#: GCSViews/FlightPlanner.cs:2112-2155`
fn start_fence_clear(this: &mut MissionPlanner) {
    begin_fence_clear(&mut this.plan);
}

/// [`start_fence_clear`] on the plan: the row of sets, unless one is under way.
pub fn begin_fence_clear(plan: &mut Plan) {
    // One row at a time, as the C#'s blocking calls allow - Geo-Fence > Upload and Download
    // included, whose calls block the C#'s UI as these do.
    if plan.writes.is_some() || plan.fence_busy() {
        return;
    }
    let step = |name: &'static str| {
        ParamStep::one(
            name,
            0.0,
            OnTimeout::Stop {
                title: "",
                text: format!("Failed to set {name}"),
            },
        )
    };
    plan.writes = Some(ParamWrites::new(
        vec![
            step("FENCE_ENABLE"),
            step("FENCE_ACTION"),
            step("FENCE_TOTAL"),
        ],
        AfterWrites::ClearFence,
    ));
}

/// Clear Rally Points: `setParam("RALLY_TOTAL", 0)`, whatever it says - the C# catches and logs
/// a failure - and then the markers and `MAV.rallypoints` cleared.
/// `// C#: GCSViews/FlightPlanner.cs:2096-2110`
fn start_rally_clear(this: &mut MissionPlanner) {
    // One row at a time, as the C#'s blocking calls allow.
    if this.plan.writes.is_some() {
        return;
    }
    this.plan.writes = Some(ParamWrites::new(
        vec![ParamStep::one("RALLY_TOTAL", 0.0, OnTimeout::CarryOn)],
        AfterWrites::ClearRally,
    ));
}

/// Rally Points > Upload: the markers as they are now, sent one call at a time by
/// [`drive_rally`].
/// `// C#: GCSViews/FlightPlanner.cs:5936-5957`
fn start_rally_upload(this: &mut MissionPlanner) {
    if this.plan.rally_upload.is_some() {
        return;
    }
    this.plan.rally_upload = Some(RallyUpload::new(this.plan.rally_markers()));
}

/// Rally Points > Download: `(capabilities & MISSION_RALLY) >= 0` is true of every `uint`, so the
/// C# always takes its first branch - "Please connect first" without a link, else
/// `mav_mission.download` of the rally list, whose answer fills `MAV.rallypoints` and is otherwise
/// thrown away: the markers on this map do not change. The `RALLY_TOTAL` and `getRallyPoint`
/// loop after that branch - "Not Supported", "Rally points - Nothing to download", each point a
/// marker - is never reached, and is not ported.
/// `// C#: GCSViews/FlightPlanner.cs:915-928 (reached), 930-966 (not reached)`
fn start_rally_download(this: &mut MissionPlanner) {
    if !this.telemetry.view().connected || !this.telemetry.download_rally() {
        this.plan_menus.say("", PLEASE_CONNECT);
        return;
    }
    this.plan.rally_download = true;
}

/// Moves Rally Points > Upload and Download on, each frame, as far as they go without waiting.
/// A download that fails says why, as the exception `AwaitSync` rethrows would.
fn drive_rally(
    this: &mut MissionPlanner,
    view: &TelemetryView,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    if this.plan.rally_download
        && let Some(list) = this.telemetry.rally_list()
    {
        this.plan.rally_download = false;
        match list {
            Ok(items) => this.plan.vehicle_rally = items,
            Err(why) => {
                this.plan_menus.say(ERROR, why);
                this.plan_prompt_focus.focus(window, cx);
            }
        }
    }

    let Some(upload) = this.plan.rally_upload.as_mut() else {
        return;
    };
    let rally_total = view
        .parameters
        .iter()
        .find(|(name, _)| name == "RALLY_TOTAL")
        .map(|(_, value)| *value);
    step_rally_upload(&this.telemetry, upload, rally_total);
    if let Some(end) = upload.end().cloned() {
        this.plan.rally_upload_results = upload.results();
        this.plan.rally_upload = None;
        if let WritesEnd::Stopped { title, text } = end {
            this.plan_menus.say(title, text);
            this.plan_prompt_focus.focus(window, cx);
        }
    }
}

/// Moves an upload on as far as it goes without waiting: each answered call lets the next go,
/// through the link - `setParam` for `RALLY_TOTAL`, `setRallyPoint` for each marker - until one
/// is out or the upload has ended.
fn step_rally_upload(
    telemetry: &crate::telemetry::Telemetry,
    upload: &mut RallyUpload,
    rally_total: Option<f64>,
) {
    while upload.end().is_none() {
        if let Some(request) = upload.in_flight() {
            match telemetry.request(request) {
                Some(request) => match request.outcome() {
                    Some(outcome) => upload.answer(Some(outcome)),
                    None => return,
                },
                None => upload.answer(None),
            }
            continue;
        }
        let sent = match upload.due(rally_total) {
            None => continue,
            Some(RallyDue::Total(count)) => telemetry.write_parameter("RALLY_TOTAL", count, false),
            Some(RallyDue::Point(set)) => telemetry.set_rally_point(set),
        };
        match sent {
            Some(request) => upload.sent(request),
            None => upload.answer(None),
        }
    }
}

/// Geo-Fence > Upload: `polygongridmode = false`, the handler's checks, then its altitude boxes if
/// the vehicle calls for any, and the upload [`drive_fence`] moves on.
///
/// Every refusal here is an error the screen can show as state, so it goes to the status line
/// and the `plan.fence.upload` fact, not to a message box as the C#'s `CustomMessageBox.Show`
/// does (the owner's ruling of 2026-09-25); so do the upload's failures.
///
/// The draw panel's fence "write" (and "read") are a different thing: `cmb_missiontype` set to
/// FENCE and the mission protocol's `MAV_MISSION_TYPE_FENCE` list, which this port offers on the
/// draw panel. The menu's Upload is the legacy `FENCE_POINT` protocol, as the C#'s is.
/// `// C#: GCSViews/FlightPlanner.cs:3714-3759`
fn start_fence_upload(this: &mut MissionPlanner, view: &TelemetryView) {
    begin_fence_upload(
        &mut this.plan,
        &mut this.plan_menus,
        &view.parameters,
        this.adopt_vehicle_fence,
    );
}

/// [`start_fence_upload`] on the plan and its menus, with the vehicle's parameters and whether
/// the draw panel's fence read is waiting for its list.
pub fn begin_fence_upload(
    plan: &mut Plan,
    menus: &mut PlanMenus,
    parameters: &[(String, f64)],
    read_pending: bool,
) {
    plan.leave_polygon_mode();
    // One at a time, as the C#'s modal calls allow: another Upload or Download, Geo-Fence >
    // Clear's sets, or the draw panel's fence read waiting for its list.
    if plan.fence_busy() || plan.writes.is_some() || read_pending {
        return;
    }
    match fence_upload_checks(plan, parameters) {
        Err(text) => plan.fence_upload_said(text),
        Ok(ask) => {
            plan.fence_ask = Some(ask);
            menus.continue_fence_upload(plan);
        }
    }
}

/// Whether `cs.capabilities` carry `MAV_PROTOCOL_CAPABILITY_MISSION_FENCE`.
#[must_use]
pub const fn has_mission_fence(capabilities: u32) -> bool {
    capabilities & CAPABILITY_MISSION_FENCE > 0
}

/// The vehicle's `cs.capabilities`, 0 with none heard.
fn capabilities(view: &TelemetryView) -> u32 {
    view.state
        .as_ref()
        .map_or(0, |state| state.autopilot_info.capabilities)
}

/// Geo-Fence > Download: `polygongridmode = false`; on a vehicle with `MISSION_FENCE`, the fence
/// list through the mission protocol - "Please connect first" without a link, "Failed to get
/// fence point" when it fails; otherwise "Not Supported" without `FENCE_ACTION` and `FENCE_TOTAL`,
/// "Nothing to download" for a `FENCE_TOTAL` of 1 or less, and the geofence cleared and read point
/// by point.
///
/// The C#'s mission branch throws its list away and leaves the geofence as it was: what it
/// downloads reaches only `MAV.fencepoints`, which `processInfoFromStream` fills as the items
/// pass (`MAVLinkInterface.cs:5641-5694`) - the link's fence points here - and `writeKML` draws
/// as its own "fence" overlay in MISSION mode (`FlightPlanner.cs:1497-1517`). The branch is
/// reached only when the capabilities change between the menu opening and the click, as the
/// menu hides Geo-Fence over such a vehicle ([`HIDDEN_ON_MISSION_FENCE`]).
/// `// C#: GCSViews/FlightPlanner.cs:824-913`
fn start_fence_download(this: &mut MissionPlanner, view: &TelemetryView) {
    let cleared = begin_fence_download(
        &mut this.plan,
        &this.telemetry,
        view.connected,
        capabilities(view),
        &view.parameters,
        this.adopt_vehicle_fence,
    );
    if cleared {
        this.sync_map_fence();
    }
}

/// [`start_fence_download`] on the plan, through `telemetry`: whether the geofence was cleared
/// for a point by point read, which the map then shows.
pub fn begin_fence_download(
    plan: &mut Plan,
    telemetry: &crate::telemetry::Telemetry,
    connected: bool,
    capabilities: u32,
    parameters: &[(String, f64)],
    read_pending: bool,
) -> bool {
    plan.leave_polygon_mode();
    if plan.fence_busy() || plan.writes.is_some() || read_pending {
        return false;
    }
    if has_mission_fence(capabilities) {
        if !connected {
            plan.fence_download_said(PLEASE_CONNECT);
            return false;
        }
        if telemetry.download_fence() {
            plan.fence_download = Some(FenceDownloading::Mission);
        } else {
            plan.fence_download_said(FENCE_POINT_FAILED);
        }
        return false;
    }
    let (Some(_), Some(total)) = (
        listed(parameters, "FENCE_ACTION"),
        listed(parameters, "FENCE_TOTAL"),
    ) else {
        plan.fence_download_said(FENCE_NOT_SUPPORTED);
        return false;
    };
    // `int.Parse(MAV.param["FENCE_TOTAL"].ToString()) <= 1`.
    if total <= 1.0 {
        plan.fence_download_said(NOTHING_TO_DOWNLOAD);
        return false;
    }
    // geofenceoverlay's polygons and markers, and geofencepolygon's points, cleared first.
    plan.fence.clear();
    plan.fence_return = None;
    plan.fence_download = Some(FenceDownloading::Points(FenceDownload::new()));
    true
}

/// Moves Geo-Fence > Upload and Download on, each frame, as far as they go without waiting, and
/// does what each does at its end: the drawn polygon made the geofence, or the points read made
/// it and its return location.
///
/// The status line carries what the C# shows: the progress window's text while points go, and
/// each failure (the owner's ruling puts errors there, not in boxes). Success says nothing - the
/// progress window closes, and the download shows no box - and only the facts record it.
fn drive_fence(this: &mut MissionPlanner, view: &TelemetryView) {
    let alt_box = this.plan_menus.prompt.as_ref().is_some_and(|prompt| {
        matches!(
            prompt.kind,
            PromptKind::FenceMinAlt | PromptKind::FenceMaxAlt
        )
    });
    let changed = fence_frame(
        &mut this.plan,
        alt_box,
        &this.telemetry,
        view.connected,
        &mut this.file_status,
    );
    if changed.fence {
        this.sync_map_fence();
    }
    if changed.polygon {
        this.sync_map_polygon();
    }
}

/// What a frame of Geo-Fence > Upload and Download changed that the map shows.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct FenceChanged {
    /// The geofence or its return location.
    pub fence: bool,
    /// The drawn polygon.
    pub polygon: bool,
}

/// [`drive_fence`] on the plan, through `telemetry`, with the status line: whether the Min/Max
/// Alt box is the one showing, and whether the link is up.
pub fn fence_frame(
    plan: &mut Plan,
    alt_box: bool,
    telemetry: &crate::telemetry::Telemetry,
    connected: bool,
    status_line: &mut Option<String>,
) -> FenceChanged {
    let mut changed = FenceChanged::default();
    // The Min/Max Alt box replaced by another flow's box is that box's Cancel: `fence_ask`
    // otherwise outlives it and the menu's Upload and Download stay refused.
    if plan.fence_ask.is_some() && !alt_box {
        plan.fence_ask = None;
    }

    if let Some(upload) = plan.fence_upload.as_mut() {
        step_fence_upload(telemetry, upload);
        let progress = upload.progress();
        match upload.end().cloned() {
            None => {
                if let Some(progress) = progress {
                    *status_line = Some(progress.to_owned());
                }
            }
            Some(end) => {
                let polygon = upload.polygon().to_vec();
                plan.fence_upload_results = upload.results();
                plan.fence_upload = None;
                // The progress window's text goes with the window.
                if progress.is_some_and(|text| status_line.as_deref() == Some(text)) {
                    *status_line = None;
                }
                match end {
                    FenceUploadEnd::Done { error } => {
                        plan.fence_uploaded(polygon);
                        changed.fence = true;
                        changed.polygon = true;
                        match error {
                            Some(error) => plan.fence_upload_said(&unexpected_error(error)),
                            None => plan.fence_upload_noted("done"),
                        }
                    }
                    FenceUploadEnd::Stopped(text) => plan.fence_upload_said(text),
                }
            }
        }
    }

    match plan.fence_download.as_mut() {
        Some(FenceDownloading::Points(download)) => {
            step_fence_download(telemetry, download);
            if let Some(end) = download.end() {
                let points = download.points().to_vec();
                plan.fence_download = None;
                match end {
                    Ok(()) => {
                        plan.fence_downloaded(&points);
                        changed.fence = true;
                        plan.fence_download_noted(&points.len().to_string());
                    }
                    Err(text) => plan.fence_download_said(text),
                }
            }
        }
        // `mav_mission.download(...).AwaitSync()`: its end, or its exception - the link gone
        // among them, as the C#'s download throws on a closed port. The list itself goes
        // nowhere but the link's fence points, as the C# throws it away.
        Some(FenceDownloading::Mission) => match telemetry.fence_list() {
            None if connected => {}
            None | Some(Err(_)) => {
                plan.fence_download = None;
                plan.fence_download_said(FENCE_POINT_FAILED);
            }
            Some(Ok(items)) => {
                plan.fence_download = None;
                plan.fence_download_noted(&items.len().to_string());
            }
        },
        None => {}
    }

    if let Some(text) = plan.fence_say.take() {
        *status_line = Some(text);
    }
    changed
}

/// Moves an upload on as far as it goes without waiting: each answered call lets the next go,
/// through the link - `setParam` for the parameters, `setFencePoint` for each point - until one
/// is out or the upload has ended.
fn step_fence_upload(telemetry: &crate::telemetry::Telemetry, upload: &mut FenceUpload) {
    while upload.end().is_none() {
        if let Some(request) = upload.in_flight() {
            match telemetry.request(request) {
                Some(request) => match request.outcome() {
                    Some(outcome) => upload.answer(Some(outcome)),
                    None => return,
                },
                None => upload.answer(None),
            }
            continue;
        }
        let sent = match upload.due() {
            None => continue,
            Some(FenceDue::Param(name, value)) => telemetry.write_parameter(name, value, false),
            Some(FenceDue::Point(set)) => telemetry.set_fence_point(set),
        };
        match sent {
            Some(request) => upload.sent(request),
            None => upload.answer(None),
        }
    }
}

/// Moves a download on as far as it goes without waiting: each point read lets the next be
/// asked for, through the link's `getFencePoint`, until one is out or the download has ended.
fn step_fence_download(telemetry: &crate::telemetry::Telemetry, download: &mut FenceDownload) {
    while download.end().is_none() {
        if let Some(request) = download.in_flight() {
            match telemetry.request(request) {
                Some(request) => match request.outcome() {
                    Some(outcome) => download.answer(Some(outcome), request.fence_point()),
                    None => return,
                },
                None => download.answer(None, None),
            }
            continue;
        }
        let Some(idx) = download.due() else {
            continue;
        };
        match telemetry.get_fence_point(idx) {
            Some(request) => download.sent(request),
            None => download.answer(None, None),
        }
    }
}

/// Moves the rows of parameter sets on, each frame, as far as they go without waiting: a Write
/// whose upload has finished starts its sets, a set that has been answered lets the next one go,
/// and a finished row does what follows it.
pub fn drive_writes(
    this: &mut MissionPlanner,
    view: &TelemetryView,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    drive_rally(this, view, window, cx);
    drive_fence(this, view);
    drive_mission_ftp(this, view, window, cx);
    if let Some(pending) = this.plan.pending_write.as_mut() {
        match pending.upload_ended(view.transfer.as_ref(), &view.mission) {
            None => {}
            Some(false) => this.plan.pending_write = None,
            Some(true) => {
                // `getHomePositionAsync` once the vehicle has the mission - `saveWPs` before its
                // "Setting params", `saveWPsFast` after its ack - waited for as `getHomePosition`
                // waits, the position landing in the vehicle's state.
                // `// C#: GCSViews/FlightPlanner.cs:6275-6277, 6570-6571`
                if let Some(id) = view.vehicle {
                    this.telemetry
                        .get_home_position(id, crate::telemetry::Report::default());
                    this.plan.home_requests += 1;
                }
                let steps = this
                    .plan
                    .pending_write
                    .take()
                    .map_or(Ok(Vec::new()), |pending| pending.steps);
                match steps {
                    Ok(steps) if this.plan.writes.is_none() => {
                        this.plan.writes = Some(ParamWrites::new(steps, AfterWrites::Nothing));
                    }
                    Ok(_) => {}
                    Err(why) => {
                        this.plan_menus.say(ERROR, why);
                        this.plan_prompt_focus.focus(window, cx);
                    }
                }
            }
        }
    }

    loop {
        let Some(writes) = this.plan.writes.as_mut() else {
            return;
        };
        if let Some(request) = writes.in_flight() {
            match this.telemetry.request(request) {
                Some(request) => match request.outcome() {
                    Some(outcome) => writes.answer(Some(outcome)),
                    None => return,
                },
                None => writes.answer(None),
            }
            continue;
        }
        if let Some(end) = writes.end().cloned() {
            this.plan.write_results = writes.results();
            let after = writes.after();
            this.plan.writes = None;
            match end {
                WritesEnd::Done => match after {
                    AfterWrites::ClearFence => {
                        this.plan.clear_geofence();
                        this.sync_map_fence();
                        this.sync_map_polygon();
                    }
                    AfterWrites::ClearRally => {
                        this.plan.clear_rally_points();
                        this.telemetry.clear_rally_points();
                        this.sync_map_rally();
                    }
                    AfterWrites::Nothing => {}
                },
                WritesEnd::Stopped { title, text } => {
                    this.plan_menus.say(title, text);
                    this.plan_prompt_focus.focus(window, cx);
                }
            }
            return;
        }
        let Some((name, value)) = writes.due() else {
            return;
        };
        match this.telemetry.set_parameter_confirmed(name, value) {
            Some(request) => writes.sent(request),
            None => writes.answer(None),
        }
    }
}

/// The file a file dialog's typed answer means, as an `OpenFileDialog` or `SaveFileDialog` takes
/// one (the owner, 2026-10-04: the mission's Load and Save File are to reach any file, as Mission
/// Planner's dialogs do): an absolute path as it is, `~/` from the home folder, anything else from
/// `directory`, the folder the dialog opened in; `extension` - the filter's - added when the file
/// has none, as the dialogs' `AddExtension` does. Nothing for an empty answer or a folder: the C#
/// acts only on `FileName != ""`, and `File.Exists` of a folder is false.
#[must_use]
pub fn dialog_path(typed: &str, extension: &str, directory: &Path) -> Option<PathBuf> {
    let typed = typed.trim();
    if typed.is_empty() || typed.ends_with(['/', '\\']) {
        return None;
    }
    let path = typed_path(typed, directory)?;
    if path.is_dir() {
        return None;
    }
    Some(if path.extension().is_none() && !extension.is_empty() {
        path.with_extension(extension)
    } else {
        path
    })
}

/// A typed path: absolute as it is, `~` from the home folder, anything else from `directory`.
fn typed_path(typed: &str, directory: &Path) -> Option<PathBuf> {
    if typed == "~" {
        return home_directory();
    }
    if let Some(rest) = typed.strip_prefix("~/").or_else(|| typed.strip_prefix("~\\")) {
        return home_directory().map(|home| home.join(rest));
    }
    let path = PathBuf::from(typed);
    Some(if path.is_absolute() {
        path
    } else {
        directory.join(path)
    })
}

/// The user's home folder, as `~` means it.
fn home_directory() -> Option<PathBuf> {
    std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .map(PathBuf::from)
}

/// One row of a file dialog's list: a folder to open, or a file of the dialog's filter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DialogEntry {
    /// What the row says.
    pub name: String,
    /// Where it is.
    pub path: PathBuf,
    /// A folder - `..` among them - rather than a file.
    pub folder: bool,
}

/// The most rows a file dialog lists.
const DIALOG_LIST_MAX: usize = 200;

/// What a file dialog lists, and of which folder: the folder the typed answer names or sits in,
/// else the one it opened in; `..` first, then its folders, then its files with one of
/// `extensions` - folders only when there are none, as for a folder dialog - each by name, hidden
/// ones left out.
#[must_use]
pub fn dialog_listing(
    typed: &str,
    directory: &Path,
    extensions: &[&str],
) -> (PathBuf, Vec<DialogEntry>) {
    let typed = typed.trim();
    let folder = if typed.is_empty() {
        directory.to_path_buf()
    } else {
        match typed_path(typed, directory) {
            Some(path) if path.is_dir() => path,
            Some(path) => path
                .parent()
                .filter(|parent| parent.is_dir())
                .map_or_else(|| directory.to_path_buf(), Path::to_path_buf),
            None => directory.to_path_buf(),
        }
    };
    // Rebuilt from its parts, so a typed "/" is the platform's separator: on Windows "b-sub/"
    // listed "b-sub/deep.waypoints" beside "\\"-separated paths (CI run 37206807094).
    let folder: PathBuf = folder.components().collect();
    let mut folders = Vec::new();
    let mut files = Vec::new();
    for entry in std::fs::read_dir(&folder).into_iter().flatten().flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with('.') {
            continue;
        }
        let path = entry.path();
        if path.is_dir() {
            folders.push(DialogEntry {
                name,
                path,
                folder: true,
            });
        } else if path.extension().is_some_and(|extension| {
            extensions
                .iter()
                .any(|wanted| extension.eq_ignore_ascii_case(wanted))
        }) {
            files.push(DialogEntry {
                name,
                path,
                folder: false,
            });
        }
    }
    let by_name =
        |a: &DialogEntry, b: &DialogEntry| a.name.to_lowercase().cmp(&b.name.to_lowercase());
    folders.sort_by(by_name);
    files.sort_by(by_name);
    let up = folder.parent().map(|parent| DialogEntry {
        name: "..".to_owned(),
        path: parent.to_path_buf(),
        folder: true,
    });
    let entries = up
        .into_iter()
        .chain(folders)
        .chain(files)
        .take(DIALOG_LIST_MAX)
        .collect();
    (folder, entries)
}

/// What a row puts in the dialog's box: a file in the dialog's own folder by its name, anything
/// else by its path, a folder's with a separator after it so the list opens it.
#[must_use]
pub fn dialog_answer(entry: &DialogEntry, directory: &Path) -> String {
    if entry.folder {
        format!("{}{}", entry.path.display(), std::path::MAIN_SEPARATOR)
    } else if entry.path.parent() == Some(directory) {
        entry.name.clone()
    } else {
        entry.path.display().to_string()
    }
}

/// Polygon > From SHP once its dialog has returned: the polygon cleared whatever it returned,
/// then - `if (File.Exists(file))` - the `.prj` of the same name read if there is one, and every
/// feature's coordinates added in turn. A file that cannot be read is said as the C#'s `catch`
/// says it, `Strings.ERROR + "\n" + ex`. The map is fitted to the corners, as
/// `ZoomAndCenterMarkers(drawnpolygonsoverlay.Id)` does after each feature.
/// `// C#: GCSViews/FlightPlanner.cs:3533-3616`
fn load_shp(this: &mut MissionPlanner, name: &str) -> Result<(), String> {
    // Poly Clear
    this.plan.load_polygon(Vec::new());
    let Some(path) = dialog_path(name, "shp", &this.plan_menus.dialog_directory) else {
        return Ok(());
    };
    let Ok(bytes) = std::fs::read(&path) else {
        this.file_status = Some(format!("could not read {}", path.display()));
        return Ok(());
    };
    // `Path.GetFileNameWithoutExtension(file) + ".prj"`, its first line.
    let projection = match std::fs::read_to_string(path.with_extension("prj")) {
        Ok(text) => Some(
            mp_mission::shapefile::Projection::from_esri(text.lines().next().unwrap_or(""))
                .map_err(|why| format!("{ERROR}\n{why}"))?,
        ),
        Err(_) => None,
    };
    let features =
        mp_mission::shapefile::features(&bytes).map_err(|why| format!("{ERROR}\n{why}"))?;
    this.plan.add_shp_features(&features, projection);
    this.map.borrow_mut().zoom_to_fit(this.plan.polygon());
    this.file_status = Some(format!("loaded a polygon from {}", path.display()));
    Ok(())
}

/// Save File's dialog, opened on the mission's file name.
fn ask_mission_save(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let name = this.plan_file_name();
    this.plan_menus.ask_mission_save(&name);
    this.plan_prompt_focus.focus(window, cx);
    cx.notify();
}

/// The dialogs' files, once named: Geo-Fence > Load from File and Save to File, Polygon > Save
/// Polygon, Load Polygon and From SHP, Rally Points > Save Rally to File and Load Rally from File,
/// and the mission's Load File and Save File. Each answer is a path as a file dialog takes one
/// ([`dialog_path`]).
fn file_request(
    this: &mut MissionPlanner,
    request: FileRequest,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let directory = this.plan_menus.dialog_directory.clone();
    let path = |name: &str, extension: &str| dialog_path(name, extension, &directory);
    let mut refused: Option<(&'static str, String)> = None;
    match request {
        // `if (File.Exists(fd.FileName))`, then the polygon replaced by the file's corners and
        // the map fitted to them.
        // `// C#: GCSViews/FlightPlanner.cs:4534-4584`
        FileRequest::LoadPolygon(name) => {
            let Some(path) = path(&name, "poly") else {
                return;
            };
            match std::fs::read_to_string(&path) {
                Ok(text) => {
                    this.plan
                        .load_polygon(mp_mission::fence_file::read_polygon(&text));
                    this.map.borrow_mut().zoom_to_fit(this.plan.polygon());
                    this.file_status = Some(format!("loaded a polygon from {}", path.display()));
                }
                Err(err) => {
                    this.file_status = Some(format!("could not read {}: {err}", path.display()));
                }
            }
        }
        // `// C#: GCSViews/FlightPlanner.cs:5903-5931`
        FileRequest::SavePolygon(name) => {
            let Some(path) = path(&name, "poly") else {
                return;
            };
            let text = mp_mission::fence_file::write_polygon(this.plan.polygon());
            match std::fs::write(&path, text) {
                Ok(()) => {
                    this.file_status = Some(format!("saved the polygon to {}", path.display()));
                }
                // The polygon's writer says the fence's words.
                Err(_) => refused = Some(("", FENCE_FILE_FAILED.to_owned())),
            }
        }
        FileRequest::LoadShp(name) => {
            if let Err(why) = load_shp(this, &name) {
                refused = Some((ERROR, why));
            }
        }
        // `// C#: GCSViews/FlightPlanner.cs:6039-6062`
        FileRequest::SaveRally(name) => {
            let Some(path) = path(&name, "ral") else {
                return;
            };
            let text = mp_mission::fence_file::write_rally(&this.plan.rally_markers());
            match std::fs::write(&path, text) {
                Ok(()) => {
                    this.file_status =
                        Some(format!("saved the rally points to {}", path.display()));
                }
                Err(_) => refused = Some(("", RALLY_FILE_FAILED.to_owned())),
            }
        }
        // `if (File.Exists(fd.FileName))`, then a marker per row.
        // `// C#: GCSViews/FlightPlanner.cs:4421-4462`
        FileRequest::LoadRally(name) => {
            let Some(path) = path(&name, "ral") else {
                return;
            };
            match std::fs::read_to_string(&path) {
                Ok(text) => {
                    this.plan
                        .load_rally_file(&mp_mission::fence_file::read_rally(&text));
                    this.file_status = Some(format!("loaded rally points from {}", path.display()));
                }
                Err(err) => {
                    this.file_status = Some(format!("could not read {}: {err}", path.display()));
                }
            }
        }
        FileRequest::LoadFence(_) | FileRequest::SaveFence(_) => {
            fence_file(this, request, window, cx);
        }
        FileRequest::LoadAndAppend(name) => refused = load_and_append(this, &name),
        FileRequest::LoadKml(name) => refused = load_kml_mission(this, &name),
        FileRequest::LoadShpMission(name) => refused = load_shp_mission(this, &name),
        FileRequest::KmlOverlay(name) => refused = load_kml_overlay(this, &name),
        FileRequest::InjectCustomMap(folder) => this.inject_map_begin(&folder),
        FileRequest::LoadMission(name) => refused = load_mission(this, &name),
        FileRequest::SaveMission(name) => refused = save_mission(this, &name),
    }
    if let Some((title, text)) = refused {
        this.plan_menus.say(title, text);
    }
    // A refusal, or KML Overlay's first question.
    if this.plan_menus.prompt.is_some() {
        this.plan_prompt_focus.focus(window, cx);
    }
}

/// Geo-Fence > Load from File and Save to File, once the file has been named.
fn fence_file(
    this: &mut MissionPlanner,
    request: FileRequest,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    match request {
        FileRequest::LoadFence(name) => {
            let Some(path) = dialog_path(&name, "fen", &this.plan_menus.dialog_directory) else {
                return;
            };
            match std::fs::read_to_string(&path) {
                Ok(text) => {
                    this.plan
                        .adopt_fence_file(mp_mission::fence_file::read_fence(&text));
                    this.file_status = Some(format!("loaded a fence from {}", path.display()));
                }
                // `if (File.Exists(fd.FileName))`: nothing happens, bar the status line.
                Err(err) => {
                    this.file_status = Some(format!("could not read {}: {err}", path.display()));
                }
            }
        }
        FileRequest::SaveFence(name) => {
            let Some(path) = dialog_path(&name, "fen", &this.plan_menus.dialog_directory) else {
                return;
            };
            let written = this.plan.fence_file().and_then(|file| {
                std::fs::write(&path, mp_mission::fence_file::write_fence(&file))
                    .map_err(|_| FENCE_FILE_FAILED)
            });
            match written {
                Ok(()) => {
                    this.file_status = Some(format!("saved the fence to {}", path.display()));
                }
                Err(why) => {
                    this.plan_menus.say("", why);
                    this.plan_prompt_focus.focus(window, cx);
                }
            }
        }
        // `file_request`'s own.
        FileRequest::SavePolygon(_)
        | FileRequest::LoadPolygon(_)
        | FileRequest::LoadShp(_)
        | FileRequest::SaveRally(_)
        | FileRequest::LoadRally(_)
        | FileRequest::LoadAndAppend(_)
        | FileRequest::LoadKml(_)
        | FileRequest::LoadShpMission(_)
        | FileRequest::KmlOverlay(_)
        | FileRequest::InjectCustomMap(_)
        | FileRequest::LoadMission(_)
        | FileRequest::SaveMission(_) => {}
    }
}

/// One of the draw-mode buttons: `cmb_missiontype` set. FENCE also shows
/// `MessageShowAgain("FlightPlan Fence", ...)`, every time, unless its tick was cleared.
/// `// C#: GCSViews/FlightPlanner.cs:2190-2199`
fn draw_mode_clicked(
    this: &mut MissionPlanner,
    mode: DrawMode,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    this.plan.set_draw_mode(mode);
    if mode != DrawMode::Fence || ShowAgain::suppressed(this.persisted.get(FENCE_KEY)) {
        return;
    }
    this.plan_menus
        .ask(Prompt::message(FENCE_TITLE, FENCE_TEXT).with_show_again(FENCE_KEY));
    this.plan_prompt_focus.focus(window, cx);
}

/// Once a frame: a show-again message whose tick was cleared is not shown.
pub fn drop_suppressed_prompt(this: &mut MissionPlanner) {
    let persisted = &this.persisted;
    this.plan_menus
        .drop_suppressed_prompt(|key| ShowAgain::suppressed(persisted.get(key)));
}

/// `BUT_write_Click` and `but_writewpfast_Click` up to their progress dialogue: with Absolute
/// selected, "Absolute Alt is selected are you sure?" (No makes it Relative and goes on); Write
/// Fast refuses a fence or rally list; then the rows' checks, and the upload.
/// `// C#: GCSViews/FlightPlanner.cs:1755-1830, 1895-1980`
pub fn start_write(
    this: &mut MissionPlanner,
    fast: bool,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    // Clicking Write takes the focus, so a panel box being typed in is left first.
    leave_panel_boxes(this, window, cx);
    this.plan_menus.write_flow = Some(WriteFlow { fast, next_row: 0 });
    if this.altitude_frame == AltitudeFrame::Absolute {
        this.plan_menus.ask(Prompt::question(
            ALT_MODE_TITLE,
            ALT_MODE_QUESTION,
            PromptKind::WriteAltMode,
        ));
        this.plan_prompt_focus.focus(window, cx);
        cx.notify();
        return;
    }
    continue_write(this, window, cx);
}

/// A Write question answered: No to Alt Mode makes the frame Relative first, Cancel to the zero
/// altitude warning ends the write, and anything else goes on.
fn write_answered(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    match this.plan_menus.take_write_answer() {
        None | Some(WriteAnswer::Abort) => {}
        Some(WriteAnswer::RelativeThenContinue) => {
            this.set_altitude_frame(AltitudeFrame::Relative);
            continue_write(this, window, cx);
        }
        Some(WriteAnswer::Continue) => continue_write(this, window, cx),
    }
}

/// The write from where its questions left it: the mission-type refusal, home, each row's
/// checks with the zero altitude question where one is due, then the send.
fn continue_write(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let Some(flow) = this.plan_menus.write_flow else {
        return;
    };
    let mut refuse = |this: &mut MissionPlanner, title: &'static str, text: String| {
        this.plan_menus.write_flow = None;
        this.plan_menus.say(title, text);
        this.plan_prompt_focus.focus(window, cx);
        cx.notify();
    };
    // `if (cmb_missiontype.SelectedValue != MISSION) "Only available for missions"`.
    if flow.fast && matches!(this.plan.draw_mode(), DrawMode::Fence | DrawMode::Rally) {
        refuse(this, "", ONLY_FOR_MISSIONS.to_owned());
        return;
    }
    let view = this.telemetry.view();
    let ardupilot = view
        .state
        .as_ref()
        .is_some_and(|state| state.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA);
    let items = match this.plan.vehicle_mission(ardupilot) {
        Ok(items) => items,
        Err(why) => {
            refuse(this, ERROR, why.to_owned());
            return;
        }
    };
    let family = view
        .state
        .as_ref()
        .and_then(|state| mp_vehicle::VehicleFamily::from_mav_type(state.vehicle_type));
    let alt_warn = this.plan.panel_text(PanelBox::AltWarn).to_owned();
    let rows: Vec<MissionItem> = this.plan.items().to_vec();
    for (index, item) in rows.iter().enumerate().skip(flow.next_row) {
        if let Err(why) = write_row_checks(item, index, &alt_warn) {
            refuse(this, "", why);
            return;
        }
        if let Some((warning, key)) = zero_alt_warning(item, index, family) {
            // `MessageShowAgain`: a prompt turned off by its show-again key is OK at once.
            let suppressed = this
                .persisted
                .get(key)
                .and_then(mp_mission::dotnet::parse_bool)
                == Some(false);
            if !suppressed {
                this.plan_menus.write_flow = Some(WriteFlow {
                    next_row: index + 1,
                    ..flow
                });
                this.plan_menus.ask(
                    Prompt::question(ZERO_ALT_TITLE, warning, PromptKind::WriteZeroAlt)
                        .with_show_again(key),
                );
                this.plan_prompt_focus.focus(window, cx);
                cx.notify();
                return;
            }
        }
    }
    this.plan_menus.write_flow = None;
    send_mission(this, flow.fast, items);
    cx.notify();
}

/// The upload itself: `saveWPsFast` for Write Fast (no parameters follow it); for Write, the
/// MAVFTP file while the box is ticked, else `mav_mission.upload` with the radii after it.
/// `// C#: GCSViews/FlightPlanner.cs:6237-6256, 6293-6310, 6340-6582`
fn send_mission(this: &mut MissionPlanner, fast: bool, items: Vec<MissionItem>) {
    if fast {
        // No parameters follow a fast write, but the home position is asked for when it ends.
        this.plan.pending_write = Some(PendingWrite::new(items.clone(), Ok(Vec::new())));
        this.telemetry.upload_mission_fast(items);
        return;
    }
    if this.plan.use_mavftp() {
        let data = mp_mission::missionpck::pack(&items, 0, 0);
        let request = FtpRequest::Put {
            path: MissionFtp::MISSION_FILE.to_owned(),
            data,
        };
        if let Some(vehicle) = this.telemetry.start_ftp(request) {
            this.plan.mission_ftp = Some(MissionFtp::Uploading {
                vehicle,
                items,
                status: String::new(),
            });
            return;
        }
        // No client to start: the C#'s `catch` - logged, and the ordinary upload follows.
    }
    // The radii follow once the vehicle has the mission: `saveWPs`' "Setting params".
    this.plan.pending_write = Some(PendingWrite::new(items.clone(), this.plan.wp_param_steps()));
    this.telemetry.upload_mission(items);
}

/// `BUT_read_Click` → `getWPs`: the MAVFTP file while the box is ticked, else the mission
/// protocol's download, adopted when it completes.
/// `// C#: GCSViews/FlightPlanner.cs:3977-4014`
pub(crate) fn read_from_vehicle(this: &mut MissionPlanner) {
    if this.plan.use_mavftp() {
        let request = FtpRequest::Get {
            path: MissionFtp::MISSION_FILE.to_owned(),
            burst: true,
            readsize: 110,
        };
        if let Some(vehicle) = this.telemetry.start_ftp(request) {
            this.plan.mission_ftp = Some(MissionFtp::Downloading {
                vehicle,
                status: String::new(),
            });
            return;
        }
    }
    this.telemetry.request_mission();
    this.adopt_vehicle_mission = true;
}

/// The MAVFTP transfer moved on each frame: its progress words while it runs; when it stops, a
/// written file is done (no parameters follow, as `saveWPs` returns before "Setting params"), a
/// read file is unpacked and shown (`WPtoScreen`), and a failure of either falls back to the
/// mission protocol, as the C#'s `catch` does.
/// `// C#: GCSViews/FlightPlanner.cs:3985-4014, 6237-6256`
fn drive_mission_ftp(
    this: &mut MissionPlanner,
    view: &TelemetryView,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let Some(ftp) = this.plan.mission_ftp.clone() else {
        return;
    };
    let vehicle = match &ftp {
        MissionFtp::Uploading { vehicle, .. } | MissionFtp::Downloading { vehicle, .. } => *vehicle,
        MissionFtp::Ended { .. } => return,
    };
    if let Some((true, progress)) = this.telemetry.ftp_progress(vehicle) {
        this.plan.mission_ftp = Some(match ftp {
            MissionFtp::Uploading { vehicle, items, .. } => MissionFtp::Uploading {
                vehicle,
                items,
                status: progress.message,
            },
            MissionFtp::Downloading { vehicle, .. } => MissionFtp::Downloading {
                vehicle,
                status: progress.message,
            },
            ended @ MissionFtp::Ended { .. } => ended,
        });
        return;
    }
    let outcome = this.telemetry.take_ftp_outcome(vehicle);
    let why = |outcome: &Option<Result<FtpOutcome, mp_ftp::FtpError>>| match outcome {
        Some(Err(error)) => error.to_string(),
        Some(Ok(_)) => "no file came back".to_owned(),
        None => "no answer".to_owned(),
    };
    match ftp {
        MissionFtp::Uploading { items, .. } => {
            if matches!(outcome, Some(Ok(_))) {
                this.plan.mission_ftp = Some(MissionFtp::Ended {
                    ok: true,
                    text: "MAVFTP: mission written".to_owned(),
                });
                return;
            }
            this.plan.mission_ftp = Some(MissionFtp::Ended {
                ok: false,
                text: format!(
                    "MAVFTP failed ({}); writing over the mission protocol",
                    why(&outcome)
                ),
            });
            this.plan.pending_write =
                Some(PendingWrite::new(items.clone(), this.plan.wp_param_steps()));
            this.telemetry.upload_mission(items);
        }
        MissionFtp::Downloading { .. } => {
            let unpacked = match &outcome {
                Some(Ok(FtpOutcome::File {
                    data: Some(data), ..
                })) => mp_mission::missionpck::unpack(data).map_err(|error| error.to_string()),
                other => Err(why(other)),
            };
            match unpacked {
                Ok(unpacked) => {
                    // `WPtoScreen(values.wps)`: the file's items become the grid, as a read
                    // over the protocol does when it completes.
                    if let Some(home) = this.plan.adopt_from_vehicle(&unpacked.items) {
                        this.plan_menus.offer_home_reset(home);
                        this.plan_prompt_focus.focus(window, cx);
                    }
                    this.plan.set_wp_params(&view.parameters);
                    this.file_status = Some(format!(
                        "read {} items from the vehicle over MAVFTP",
                        unpacked.items.len()
                    ));
                    this.plan.mission_ftp = Some(MissionFtp::Ended {
                        ok: true,
                        text: "MAVFTP: mission read".to_owned(),
                    });
                }
                Err(why) => {
                    this.plan.mission_ftp = Some(MissionFtp::Ended {
                        ok: false,
                        text: format!("MAVFTP failed ({why}); reading over the mission protocol"),
                    });
                    this.telemetry.request_mission();
                    this.adopt_vehicle_mission = true;
                }
            }
        }
        MissionFtp::Ended { .. } => {}
    }
    cx.notify();
}

/// The Home Location link, `label4_LinkClicked`.
/// `// C#: GCSViews/FlightPlanner.cs:4283-4300`
fn home_link_clicked(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let view = this.telemetry.view();
    let vehicle = view.state.as_ref().and_then(|state| {
        state
            .position
            .map(|position| (position, state.altitude_msl.0))
    });
    if let Err(why) = this.plan.home_from_vehicle(vehicle) {
        this.plan_menus.say("", why);
        this.plan_prompt_focus.focus(window, cx);
    }
    cx.notify();
}

/// The Home Location boxes' focus handles, top to bottom, and which one has the keyboard.
pub struct HomeFocus<'a> {
    /// One per box, in [`HomeBox::ALL`] order.
    pub handles: &'a [gpui::FocusHandle; 3],
    /// Whether each has focus.
    pub focused: [bool; 3],
}

/// The Home Location panel, `panel1`: the link that takes the vehicle's position, and the three
/// boxes home is typed into, Lat, Long and ASL, one under the other with their labels to the
/// left - as the `.resx` places them, below the Read and Write buttons.
/// `// C#: GCSViews/FlightPlanner.resx (panel1, label4, Label1-3, TXT_homelat/lng/alt)`
pub fn home_panel(
    plan: &Plan,
    focus: &HomeFocus<'_>,
    cx: &mut Context<MissionPlanner>,
) -> impl IntoElement {
    let link = crate::probe::measured("plan-home-link", div())
        .id("plan-home-link")
        .text_xs()
        .text_color(rgb(theme::ACCENT))
        .underline()
        .cursor_pointer()
        .child("Home Location")
        .on_click(cx.listener(|this, _event, window, cx| {
            home_link_clicked(this, window, cx);
        }));

    let mut boxes = div().flex().flex_col().gap_1();
    for (index, which) in HomeBox::ALL.into_iter().enumerate() {
        let (Some(handle), Some(focused)) =
            (focus.handles.get(index), focus.focused.get(index).copied())
        else {
            continue;
        };
        boxes = boxes.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(36.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(which.label()),
                )
                .child(crate::textfield::text_field(
                    which.id(),
                    plan.home_field(which),
                    handle,
                    focused,
                    px(160.0),
                    cx.listener(move |this, event: &gpui::KeyDownEvent, _window, cx| {
                        if this.plan.home_key(which, event) == crate::textfield::KeyOutcome::Ignored
                        {
                            return;
                        }
                        cx.notify();
                    }),
                )),
        );
    }

    crate::probe::measured("panel:Home Location", div())
        .flex()
        .flex_col()
        .gap_2()
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .child(link)
        .child(boxes)
}

// ---------------------------------------------------------------------------------------------
// The map's right-click menu: `contextMenuStrip1`, and the `InputBox` prompts its items open.
// ---------------------------------------------------------------------------------------------

/// What choosing a menu entry does here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuAction {
    /// `deleteWPToolStripMenuItem_Click`.
    DeleteWp,
    /// `insertWpToolStripMenuItem_Click`.
    InsertWp,
    /// `currentPositionToolStripMenuItem_Click`.
    InsertAtCurrentPosition,
    /// `insertSplineWPToolStripMenuItem_Click`.
    InsertSplineWp,
    /// `loiterForeverToolStripMenuItem_Click`.
    LoiterForever,
    /// `loitertimeToolStripMenuItem_Click`.
    LoiterTime,
    /// `loitercirclesToolStripMenuItem_Click`.
    LoiterCircles,
    /// `jumpstartToolStripMenuItem_Click`.
    JumpStart,
    /// `jumpwPToolStripMenuItem_Click`.
    JumpWp,
    /// `rTLToolStripMenuItem_Click`.
    Rtl,
    /// `landToolStripMenuItem_Click`.
    Land,
    /// `takeoffToolStripMenuItem_Click`.
    Takeoff,
    /// `setROIToolStripMenuItem_Click`.
    SetRoi,
    /// `clearMissionToolStripMenuItem_Click`.
    ClearMission,
    /// `addPolygonPointToolStripMenuItem_Click`.
    DrawPolygon,
    /// `clearPolygonToolStripMenuItem_Click`.
    ClearPolygon,
    /// `fromCurrentWaypointsMenuItem_Click`.
    PolygonFromWaypoints,
    /// `ContextMeasure_Click`.
    MeasureDistance,
    /// `zoomToToolStripMenuItem_Click`: Map Tool > Zoom To, which asks for a place.
    ZoomTo,
    /// `zoomToVehicleToolStripMenuItem_Click`, on the zoom icon's menu.
    ZoomToVehicle,
    /// `zoomToMissionToolStripMenuItem_Click`, on the zoom icon's menu.
    ZoomToMission,
    /// `zoomToHomeToolStripMenuItem_Click`, on the zoom icon's menu.
    ZoomToHome,
    /// `reverseWPsToolStripMenuItem_Click`.
    ReverseWps,
    /// `loadWPFileToolStripMenuItem_Click`, which is `BUT_loadwpfile_Click`.
    LoadWpFile,
    /// `saveWPFileToolStripMenuItem_Click`, which is `SaveFile_Click`.
    SaveWpFile,
    /// `modifyAltToolStripMenuItem_Click`.
    ModifyAlt,
    /// `setReturnLocationToolStripMenuItem_Click`.
    SetReturnLocation,
    /// `loadFromFileToolStripMenuItem_Click`.
    FenceLoadFromFile,
    /// `saveToFileToolStripMenuItem_Click`.
    FenceSaveToFile,
    /// `clearToolStripMenuItem_Click`.
    FenceClear,
    /// `GeoFenceuploadToolStripMenuItem_Click`: Geo-Fence > Upload.
    GeoFenceUpload,
    /// `GeoFencedownloadToolStripMenuItem_Click`: Geo-Fence > Download.
    GeoFenceDownload,
    /// `surveyGridToolStripMenuItem_Click`: the Survey (Grid) dialog, `survey_ui.rs`.
    SurveyGrid,
    /// `savePolygonToolStripMenuItem_Click`.
    SavePolygon,
    /// `loadPolygonToolStripMenuItem_Click`.
    LoadPolygon,
    /// `fromSHPToolStripMenuItem_Click`.
    FromShp,
    /// `offsetPolygonToolStripMenuItem_Click`.
    OffsetPolygon,
    /// `areaToolStripMenuItem_Click`, under Polygon and under Auto WP.
    Area,
    /// `setRallyPointToolStripMenuItem_Click`.
    SetRallyPoint,
    /// `getRallyPointsToolStripMenuItem_Click`: Rally Points > Download.
    GetRallyPoints,
    /// `saveRallyPointsToolStripMenuItem_Click`: Rally Points > Upload.
    SaveRallyPoints,
    /// `clearRallyPointsToolStripMenuItem_Click`.
    ClearRallyPoints,
    /// `saveToFileToolStripMenuItem1_Click`: Save Rally to File.
    SaveRallyToFile,
    /// `loadFromFileToolStripMenuItem1_Click`: Load Rally from File.
    LoadRallyFromFile,
    /// `createWpCircleToolStripMenuItem_Click`.
    CreateWpCircle,
    /// `createSplineCircleToolStripMenuItem_Click`.
    CreateSplineCircle,
    /// `elevationGraphToolStripMenuItem_Click`: Map Tool > Elevation Graph.
    ElevationGraph,
    /// `setHomeHereToolStripMenuItem_Click`.
    SetHomeHere,
    /// `loadAndAppendToolStripMenuItem_Click`: File Load/Save > Load and Append.
    LoadAndAppend,
    /// `loadKMLFileToolStripMenuItem_Click`: File Load/Save > Load KML File.
    LoadKmlFile,
    /// `loadSHPFileToolStripMenuItem_Click`: File Load/Save > Load SHP File.
    LoadShpFile,
    /// `kMLOverlayToolStripMenuItem_Click`: Map Tool > KML Overlay.
    KmlOverlay,
    /// `createCircleSurveyToolStripMenuItem_Click`: Auto WP > Create Circle Survey.
    CreateCircleSurvey,
    /// `enterUTMCoordToolStripMenuItem_Click`: Enter UTM Coord.
    EnterUtmCoord,
    /// `trackerHomeToolStripMenuItem_Click`: Tracker Home.
    TrackerHome,
    /// `poiaddToolStripMenuItem_Click`: POI > Add.
    PoiAdd,
    /// `poideleteToolStripMenuItem_Click`: POI > Delete.
    PoiDelete,
    /// `poieditToolStripMenuItem_Click`: POI > Edit.
    PoiEdit,
    /// `FenceInclusionToolStripMenuItem_Click`, on the polygon icon's menu.
    FenceInclusion,
    /// `FenceExclusionToolStripMenuItem_Click`, on the polygon icon's menu.
    FenceExclusion,
    /// `textToolStripMenuItem_Click`: Auto WP > Text.
    Text,
    /// `prefetchToolStripMenuItem_Click`: Map Tool > Prefetch.
    Prefetch,
    /// `prefetchWPPathToolStripMenuItem_Click`: Map Tool > Prefetch WP Path.
    PrefetchWpPath,
    /// `switchDockingToolStripMenuItem_Click`: Switch Docking.
    SwitchDocking,
}

/// One entry of `contextMenuStrip1` or of one of its drop-downs.
#[derive(Debug, Clone, Copy)]
pub struct MenuEntry {
    /// The id a test script clicks it by: `menu-` and the Designer's name without
    /// `ToolStripMenuItem`. Empty for the separator.
    pub id: &'static str,
    /// The control's name in `FlightPlanner.Designer.cs`. Read by the coverage tests, which hold
    /// the menu to the Designer by it.
    #[cfg_attr(not(test), allow(dead_code))]
    pub control: &'static str,
    /// Its `Text` in `FlightPlanner.resx`. Empty for the separator.
    pub text: &'static str,
    /// What choosing it does. `None` on an entry not ported here, which is drawn dimmed and
    /// inert so the menu keeps the C#'s shape, and on an entry that only opens a drop-down.
    pub action: Option<MenuAction>,
    /// Its drop-down, in the order the Designer adds them.
    pub children: &'static [MenuEntry],
}

impl MenuEntry {
    /// Whether this is `toolStripSeparator1`.
    #[must_use]
    pub const fn is_separator(&self) -> bool {
        self.text.is_empty()
    }

    /// Whether choosing it does anything here.
    #[must_use]
    pub const fn is_live(&self) -> bool {
        self.action.is_some() || !self.children.is_empty()
    }
}

const fn item(
    id: &'static str,
    control: &'static str,
    text: &'static str,
    action: Option<MenuAction>,
) -> MenuEntry {
    MenuEntry {
        id,
        control,
        text,
        action,
        children: &[],
    }
}

const fn drop_down(
    id: &'static str,
    control: &'static str,
    text: &'static str,
    action: Option<MenuAction>,
    children: &'static [MenuEntry],
) -> MenuEntry {
    MenuEntry {
        id,
        control,
        text,
        action,
        children,
    }
}

/// `contextMenuStrip1`, in the order `contextMenuStrip1.Items.AddRange` adds it, each drop-down in
/// its own `DropDownItems.AddRange` order, with the `.resx` text. A test holds all three to the C#
/// tree when it is present.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:899-922`
pub const MAP_MENU: &[MenuEntry] = {
    use MenuAction::{
        Area, ClearMission, ClearPolygon, ClearRallyPoints, CreateCircleSurvey, CreateSplineCircle,
        CreateWpCircle, DeleteWp, DrawPolygon, ElevationGraph, EnterUtmCoord, FenceClear,
        FenceLoadFromFile, FenceSaveToFile, FromShp, GeoFenceDownload, GeoFenceUpload,
        GetRallyPoints, InsertAtCurrentPosition, InsertSplineWp, InsertWp, JumpStart, JumpWp,
        KmlOverlay, Land, LoadAndAppend, LoadKmlFile, LoadPolygon, LoadRallyFromFile, LoadShpFile,
        LoadWpFile, LoiterCircles, LoiterForever, LoiterTime, MeasureDistance, ModifyAlt,
        OffsetPolygon, PoiAdd, PoiDelete, PoiEdit, PolygonFromWaypoints, ReverseWps, Rtl,
        SavePolygon, SaveRallyPoints, SaveRallyToFile, SaveWpFile, SetHomeHere, SetRallyPoint,
        SetReturnLocation, SetRoi, SurveyGrid, Takeoff, Text, TrackerHome, ZoomTo,
    };
    &[
        item(
            "menu-deleteWP",
            "deleteWPToolStripMenuItem",
            "Delete WP",
            Some(DeleteWp),
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:937-938`
        drop_down(
            "menu-insertWp",
            "insertWpToolStripMenuItem",
            "Insert Wp",
            Some(InsertWp),
            &[item(
                "menu-currentPosition",
                "currentPositionToolStripMenuItem",
                "At Current Position",
                Some(InsertAtCurrentPosition),
            )],
        ),
        item(
            "menu-insertSplineWP",
            "insertSplineWPToolStripMenuItem",
            "Insert Spline WP",
            Some(InsertSplineWp),
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:957-960`
        drop_down(
            "menu-loiter",
            "loiterToolStripMenuItem",
            "Loiter",
            None,
            &[
                item(
                    "menu-loiterForever",
                    "loiterForeverToolStripMenuItem",
                    "Forever",
                    Some(LoiterForever),
                ),
                item(
                    "menu-loitertime",
                    "loitertimeToolStripMenuItem",
                    "Time",
                    Some(LoiterTime),
                ),
                item(
                    "menu-loitercircles",
                    "loitercirclesToolStripMenuItem",
                    "Circles",
                    Some(LoiterCircles),
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:984-986`
        drop_down(
            "menu-jump",
            "jumpToolStripMenuItem",
            "Jump",
            None,
            &[
                item(
                    "menu-jumpstart",
                    "jumpstartToolStripMenuItem",
                    "Start",
                    Some(JumpStart),
                ),
                item(
                    "menu-jumpwP",
                    "jumpwPToolStripMenuItem",
                    "WP #",
                    Some(JumpWp),
                ),
            ],
        ),
        item("menu-rTL", "rTLToolStripMenuItem", "RTL", Some(Rtl)),
        item("menu-land", "landToolStripMenuItem", "Land", Some(Land)),
        item(
            "menu-takeoff",
            "takeoffToolStripMenuItem",
            "Takeoff",
            Some(Takeoff),
        ),
        item(
            "menu-setROI",
            "setROIToolStripMenuItem",
            "DO_SET_ROI",
            Some(SetRoi),
        ),
        item(
            "menu-clearMission",
            "clearMissionToolStripMenuItem",
            "Clear Mission",
            Some(ClearMission),
        ),
        item("", "toolStripSeparator1", "", None),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1039-1047`
        drop_down(
            "menu-polygon",
            "polygonToolStripMenuItem",
            "Polygon",
            None,
            &[
                item(
                    "menu-addPolygonPoint2",
                    "addPolygonPointToolStripMenuItem2",
                    "Draw a Polygon",
                    Some(DrawPolygon),
                ),
                item(
                    "menu-clearPolygon2",
                    "clearPolygonToolStripMenuItem2",
                    "Clear Polygon",
                    Some(ClearPolygon),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:5892-5933`
                item(
                    "menu-savePolygon2",
                    "savePolygonToolStripMenuItem2",
                    "Save Polygon",
                    Some(SavePolygon),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:4528-4586`
                item(
                    "menu-loadPolygon2",
                    "loadPolygonToolStripMenuItem2",
                    "Load Polygon",
                    Some(LoadPolygon),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:3533-3616`
                item(
                    "menu-fromSHP2",
                    "fromSHPToolStripMenuItem2",
                    "From SHP",
                    Some(FromShp),
                ),
                item(
                    "menu-fromCurrentWaypoints",
                    "fromCurrentWaypointsToolStripMenuItem",
                    "From Current Waypoints",
                    Some(PolygonFromWaypoints),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:3639-3686`
                item(
                    "menu-offsetPolygon2",
                    "offsetPolygonToolStripMenuItem2",
                    "Offset Polygon",
                    Some(OffsetPolygon),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:1760-1773`
                item("menu-area2", "areaToolStripMenuItem2", "Area", Some(Area)),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1101-1107`
        drop_down(
            "menu-geoFence",
            "geoFenceToolStripMenuItem",
            "Geo-Fence",
            None,
            &[
                // `// C#: GCSViews/FlightPlanner.cs:3714-3899`
                item(
                    "menu-GeoFenceupload",
                    "GeoFenceuploadToolStripMenuItem",
                    "Upload",
                    Some(GeoFenceUpload),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:824-913`
                item(
                    "menu-GeoFencedownload",
                    "GeoFencedownloadToolStripMenuItem",
                    "Download",
                    Some(GeoFenceDownload),
                ),
                item(
                    "menu-setReturnLocation",
                    "setReturnLocationToolStripMenuItem",
                    "Set Return Location",
                    Some(SetReturnLocation),
                ),
                item(
                    "menu-loadFromFile",
                    "loadFromFileToolStripMenuItem",
                    "Load from File",
                    Some(FenceLoadFromFile),
                ),
                item(
                    "menu-saveToFile",
                    "saveToFileToolStripMenuItem",
                    "Save to File",
                    Some(FenceSaveToFile),
                ),
                item(
                    "menu-clear",
                    "clearToolStripMenuItem",
                    "Clear",
                    Some(FenceClear),
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1149-1155`
        drop_down(
            "menu-rallyPoints",
            "rallyPointsToolStripMenuItem",
            "Rally Points",
            None,
            // Hidden with Geo-Fence over a vehicle that reports
            // `MAV_PROTOCOL_CAPABILITY_MISSION_FENCE` (`contextMenuStrip1_Opening`,
            // `FlightPlanner.cs:2680-2691`; [`HIDDEN_ON_MISSION_FENCE`]) - the SITL here
            // (ArduCopter 4.8.0-dev, capabilities 0xfbef, bit 0x4000 set) among them.
            // `rallyPointsToolStripMenuItem.Visible` also follows
            // `DisplayConfiguration.displayRallyPointsMenu` (`:1325`), which is not ported.
            &[
                // `// C#: GCSViews/FlightPlanner.cs:6635-6661`
                item(
                    "menu-setRallyPoint",
                    "setRallyPointToolStripMenuItem",
                    "Set Rally Point",
                    Some(SetRallyPoint),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:915-967`
                item(
                    "menu-getRallyPoints",
                    "getRallyPointsToolStripMenuItem",
                    "Download",
                    Some(GetRallyPoints),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:5936-5957`
                item(
                    "menu-saveRallyPoints",
                    "saveRallyPointsToolStripMenuItem",
                    "Upload",
                    Some(SaveRallyPoints),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:2096-2110`
                item(
                    "menu-clearRallyPoints",
                    "clearRallyPointsToolStripMenuItem",
                    "Clear Rally Points",
                    Some(ClearRallyPoints),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:6020-6063`
                item(
                    "menu-saveToFile1",
                    "saveToFileToolStripMenuItem1",
                    "Save Rally to File",
                    Some(SaveRallyToFile),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:4414-4464`
                item(
                    "menu-loadFromFile1",
                    "loadFromFileToolStripMenuItem1",
                    "Load Rally from File",
                    Some(LoadRallyFromFile),
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1197-1203`
        drop_down(
            "menu-autoWP",
            "autoWPToolStripMenuItem",
            "Auto WP",
            None,
            &[
                // `// C#: GCSViews/FlightPlanner.cs:2963-3048`
                item(
                    "menu-createWpCircle",
                    "createWpCircleToolStripMenuItem",
                    "Create Wp Circle",
                    Some(CreateWpCircle),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:2857-2961`
                item(
                    "menu-createSplineCircle",
                    "createSplineCircleToolStripMenuItem",
                    "Create Spline Circle",
                    Some(CreateSplineCircle),
                ),
                // The Designer wires this Area to Polygon > Area's handler.
                // `// C#: GCSViews/FlightPlanner.Designer.cs:1223`
                item("menu-area1", "areaToolStripMenuItem1", "Area", Some(Area)),
                // `// C#: GCSViews/FlightPlanner.cs:6839-6883`
                item("menu-text", "textToolStripMenuItem", "Text", Some(Text)),
                item(
                    "menu-createCircleSurvey",
                    "createCircleSurveyToolStripMenuItem",
                    "Create Circle Survey",
                    Some(CreateCircleSurvey),
                ),
                item(
                    "menu-surveyGrid",
                    "surveyGridToolStripMenuItem",
                    "Survey (Grid)",
                    Some(SurveyGrid),
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1245-1254`
        drop_down(
            "menu-mapTool",
            "mapToolToolStripMenuItem",
            "Map Tool",
            None,
            &[
                item(
                    "menu-ContextMeasure",
                    "ContextMeasure",
                    "Measure Distance",
                    Some(MeasureDistance),
                ),
                // Dimmed: `rotateMapToolStripMenuItem_Click` sets `MainMap.Bearing`, and this map
                // cannot turn - gpui draws a tile as an axis-aligned image (`PolychromeSprite`
                // carries bounds and no transformation), so there is no bearing to set.
                // `// C#: GCSViews/FlightPlanner.cs:5861-5871`
                item(
                    "menu-rotateMap",
                    "rotateMapToolStripMenuItem",
                    "Rotate Map",
                    None,
                ),
                // `// C#: GCSViews/FlightPlanner.cs:8345-8368`
                item(
                    "menu-zoomTo",
                    "zoomToToolStripMenuItem",
                    "Zoom To",
                    Some(ZoomTo),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:5030-5080`
                item(
                    "menu-prefetch",
                    "prefetchToolStripMenuItem",
                    "Prefetch",
                    Some(MenuAction::Prefetch),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:3305-3365`
                item(
                    "menu-prefetchWPPath",
                    "prefetchWPPathToolStripMenuItem",
                    "Prefetch WP Path",
                    Some(MenuAction::PrefetchWpPath),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:4131-4290`
                item(
                    "menu-kMLOverlay",
                    "kMLOverlayToolStripMenuItem",
                    "KML Overlay",
                    Some(KmlOverlay),
                ),
                // `// C#: GCSViews/FlightPlanner.cs:3248-3256; Controls/ElevationProfile.cs`
                item(
                    "menu-elevationGraph",
                    "elevationGraphToolStripMenuItem",
                    "Elevation Graph",
                    Some(ElevationGraph),
                ),
                item(
                    "menu-reverseWPs",
                    "reverseWPsToolStripMenuItem",
                    "Reverse WPs",
                    Some(ReverseWps),
                ),
                item(
                    "menu-gDALOpacity",
                    "gDALOpacityToolStripMenuItem",
                    "GDAL Opacity",
                    None,
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1308-1313`
        drop_down(
            "menu-fileLoadSave",
            "fileLoadSaveToolStripMenuItem",
            "File Load/Save",
            None,
            &[
                item(
                    "menu-loadWPFile",
                    "loadWPFileToolStripMenuItem",
                    "Load WP File",
                    Some(LoadWpFile),
                ),
                item(
                    "menu-loadAndAppend",
                    "loadAndAppendToolStripMenuItem",
                    "Load and Append",
                    Some(LoadAndAppend),
                ),
                item(
                    "menu-saveWPFile",
                    "saveWPFileToolStripMenuItem",
                    "Save WP File",
                    Some(SaveWpFile),
                ),
                item(
                    "menu-loadKMLFile",
                    "loadKMLFileToolStripMenuItem",
                    "Load KML File",
                    Some(LoadKmlFile),
                ),
                item(
                    "menu-loadSHPFile",
                    "loadSHPFileToolStripMenuItem",
                    "Load SHP File",
                    Some(LoadShpFile),
                ),
            ],
        ),
        // `// C#: GCSViews/FlightPlanner.Designer.cs:1349-1352`
        drop_down(
            "menu-pOI",
            "pOIToolStripMenuItem",
            "POI",
            None,
            &[
                // `// C#: GCSViews/FlightPlanner.cs:5009-5027`
                item(
                    "menu-poiadd",
                    "poiaddToolStripMenuItem",
                    "Add",
                    Some(PoiAdd),
                ),
                item(
                    "menu-poidelete",
                    "poideleteToolStripMenuItem",
                    "Delete",
                    Some(PoiDelete),
                ),
                item(
                    "menu-poiedit",
                    "poieditToolStripMenuItem",
                    "Edit",
                    Some(PoiEdit),
                ),
            ],
        ),
        item(
            "menu-trackerHome",
            "trackerHomeToolStripMenuItem",
            "Tracker Home",
            Some(TrackerHome),
        ),
        item(
            "menu-modifyAlt",
            "modifyAltToolStripMenuItem",
            "Modify Alt",
            Some(ModifyAlt),
        ),
        item(
            "menu-enterUTMCoord",
            "enterUTMCoordToolStripMenuItem",
            "Enter UTM Coord",
            Some(EnterUtmCoord),
        ),
        // `// C#: GCSViews/FlightPlanner.cs:6762-6778`
        item(
            "menu-switchDocking",
            "switchDockingToolStripMenuItem",
            "Switch Docking",
            Some(MenuAction::SwitchDocking),
        ),
        // `// C#: GCSViews/FlightPlanner.cs:6625-6632`
        item(
            "menu-setHomeHere",
            "setHomeHereToolStripMenuItem",
            "Set Home Here",
            Some(SetHomeHere),
        ),
    ]
};

/// `contextMenuStripZoom`, the menu the zoom icon on the map opens, in its `Items.AddRange` order
/// with the `.resx` text.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:1512-1537; GCSViews/FlightPlanner.resx
/// (zoomToVehicleToolStripMenuItem.Text, ...)`
pub const ZOOM_MENU: &[MenuEntry] = &[
    item(
        "menu-zoomToVehicle",
        "zoomToVehicleToolStripMenuItem",
        "Zoom to Vehicle",
        Some(MenuAction::ZoomToVehicle),
    ),
    item(
        "menu-zoomToMission",
        "zoomToMissionToolStripMenuItem",
        "Zoom to Mission",
        Some(MenuAction::ZoomToMission),
    ),
    item(
        "menu-zoomToHome",
        "zoomToHomeToolStripMenuItem",
        "Zoom to Home",
        Some(MenuAction::ZoomToHome),
    ),
];

/// `contextMenuStripPoly`, the polygon icon's menu, in the Designer's order: the polygon entries
/// the map menu's Polygon drop-down also has, then Fence Inclusion and Fence Exclusion, which
/// `ContextMenuStripPoly_Opening` shows only while `cmb_missiontype` is FENCE - here, while the
/// geofence is being drawn.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:1469-1479; GCSViews/FlightPlanner.cs:2697-2715`
pub const POLY_MENU: &[MenuEntry] = &[
    item(
        "menu-poly-addPolygonPoint",
        "addPolygonPointToolStripMenuItem",
        "Add Polygon Point",
        Some(MenuAction::DrawPolygon),
    ),
    item(
        "menu-poly-clearPolygon",
        "clearPolygonToolStripMenuItem",
        "Clear Polygon",
        Some(MenuAction::ClearPolygon),
    ),
    item(
        "menu-poly-savePolygon",
        "savePolygonToolStripMenuItem",
        "Save Polygon",
        Some(MenuAction::SavePolygon),
    ),
    item(
        "menu-poly-loadPolygon",
        "loadPolygonToolStripMenuItem",
        "Load Polygon",
        Some(MenuAction::LoadPolygon),
    ),
    item(
        "menu-poly-fromSHP",
        "fromSHPToolStripMenuItem",
        "From SHP",
        Some(MenuAction::FromShp),
    ),
    item(
        "menu-poly-convertWPToPolygon",
        "convertWPToPolygonToolStripMenuItem",
        "From Current Waypoints",
        Some(MenuAction::PolygonFromWaypoints),
    ),
    item(
        "menu-poly-offsetPolygon",
        "offsetPolygonToolStripMenuItem",
        "Offset Polygon",
        Some(MenuAction::OffsetPolygon),
    ),
    item(
        "menu-poly-area",
        "areaToolStripMenuItem",
        "Area",
        Some(MenuAction::Area),
    ),
    item(
        "menu-fenceInclusion",
        "fenceInclusionToolStripMenuItem",
        "Fence Inclusion",
        Some(MenuAction::FenceInclusion),
    ),
    item(
        "menu-fenceExclusion",
        "fenceExclusionToolStripMenuItem",
        "Fence Exclusion",
        Some(MenuAction::FenceExclusion),
    ),
];

/// The two fence entries of [`POLY_MENU`]: shown while the geofence is being drawn.
pub const POLY_MENU_FENCE_ENTRIES: [&str; 2] = ["menu-fenceInclusion", "menu-fenceExclusion"];

/// Every entry, drop-downs included, in menu order, then the zoom icon's and the polygon icon's.
#[cfg(test)]
pub fn menu_entries() -> impl Iterator<Item = &'static MenuEntry> {
    MAP_MENU
        .iter()
        .flat_map(|entry| std::iter::once(entry).chain(entry.children.iter()))
        .chain(ZOOM_MENU.iter())
        .chain(POLY_MENU.iter())
}

/// The menu while it is open.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpenMenu {
    /// Where the right button came up, in window coordinates: where the menu is drawn.
    pub at: (f32, f32),
    /// The map position there. The C# reads `MouseDownStart` in some handlers and `MouseDownEnd`
    /// in others - where the right button went down and where it came up - and a right click
    /// that did not drag makes them the same point, so one position stands for both.
    pub position: LatLon,
    /// The waypoint under the cursor when the menu opened: `CurentRectMarker`, which is what
    /// decides whether Delete WP is enabled (`FlightPlanner.cs:2669-2677`).
    pub marker: Option<u16>,
    /// Which top-level entry's drop-down is showing, by its index in [`MAP_MENU`].
    pub submenu: Option<usize>,
    /// Whether the vehicle reported `MAV_PROTOCOL_CAPABILITY_MISSION_FENCE` when the menu
    /// opened, which hides Geo-Fence and Rally Points (see [`shown`]).
    pub mission_fence: bool,
}

/// The top-level entries `contextMenuStrip1_Opening` hides on a vehicle that reports
/// `MAV_PROTOCOL_CAPABILITY_MISSION_FENCE` - one that takes its fence and rally points as mission
/// items (every ArduPilot 4.x), which the draw panel's FENCE and RALLY lists send and read - and
/// shows on any other, or with nothing connected (capabilities 0).
/// `// C#: GCSViews/FlightPlanner.cs:2680-2691`
pub const HIDDEN_ON_MISSION_FENCE: [&str; 2] = ["menu-geoFence", "menu-rallyPoints"];

/// Whether a top-level entry shows in a menu opened over a vehicle with `MISSION_FENCE` or not.
#[must_use]
pub fn shown(entry: &MenuEntry, mission_fence: bool) -> bool {
    !(mission_fence && HIDDEN_ON_MISSION_FENCE.contains(&entry.id))
}

// ---------------------------------------------------------------------------------------------
// File Load/Save's Load and Append, Load KML File and Load SHP File, and Map Tool's KML Overlay.
// ---------------------------------------------------------------------------------------------

impl Plan {
    /// `processToScreen(cmds, append: true)`: the rows kept and the file's items added after
    /// them, its first item - home - passed over ("we dont want to add home again"), stopping at
    /// an item of command 0 or 255 that would not be the first row. Returns how many rows were
    /// added. The C# also moves `CMB_altmode` to each navigation item's frame as it goes; as
    /// with the load, the screen's frame box is left alone here.
    /// `// C#: GCSViews/FlightPlanner.cs:5496-5530`
    pub fn append_from_file(&mut self, items: &[MissionItem]) -> usize {
        let before = self.items.len();
        // `int i = Commands.Rows.Count - 1`, then `i++` per item: the row the item would take.
        let mut row = i64::try_from(before).unwrap_or(i64::MAX).saturating_sub(1);
        for (index, item) in items.iter().enumerate() {
            row += 1;
            // "0 and not home" and "bad record": the list ends here.
            if (item.command == 0 || item.command == 255) && row != 0 {
                break;
            }
            if index == 0 {
                row -= 1;
                continue;
            }
            self.items.push(*item);
        }
        self.renumber();
        if self.items.len() != before {
            self.origin = Origin::Edited;
        }
        self.items.len() - before
    }

    /// `Commands.Rows.Add()` then `setfromMap(lat, lng, alt)`: a WAYPOINT row in the screen's
    /// frame at the position, its altitude by `setfromMap`'s rules - which write none for -1 or
    /// -2, leaving the row's 0.
    /// `// C#: GCSViews/FlightPlanner.cs:1164-1236`
    pub fn add_row_from_map(
        &mut self,
        position: LatLon,
        altitude: i32,
        context: &MenuContext,
    ) -> Result<(), &'static str> {
        let altitude = self.set_from_map_altitude(position, altitude, context)?;
        self.add_waypoint_in(position, altitude, context.frame);
        Ok(())
    }

    /// `AddCommand(WAYPOINT, 0, 0, 0, 0, lng, lat, alt)`: `FillCommand` makes the row a
    /// SPLINE_WAYPOINT when the Spline box is ticked, then hands it to `setfromMap` with `(int) z`.
    /// `// C#: GCSViews/FlightPlanner.cs:540-547, 548-578`
    pub fn add_command_waypoint(
        &mut self,
        position: LatLon,
        altitude: i32,
        context: &MenuContext,
    ) -> Result<(), &'static str> {
        let z = self.set_from_map_altitude(position, altitude, context)?;
        let command = if self.spline() {
            mp_mission::commands::SPLINE_WAYPOINT
        } else {
            mp_mission::commands::WAYPOINT
        };
        self.items.push(circle_row(
            command,
            context.frame.mav_frame(),
            position.latitude(),
            position.longitude(),
            z,
        ));
        self.renumber();
        self.origin = Origin::Edited;
        Ok(())
    }

    /// The altitude `setfromMap(lat, lng, alt)` leaves a row: none written for -1 or -2 (the
    /// row's 0), else Default Alt's rules over the passed value and Verify Height.
    fn set_from_map_altitude(
        &self,
        position: LatLon,
        altitude: i32,
        context: &MenuContext,
    ) -> Result<f64, &'static str> {
        if matches!(altitude, -1 | -2) {
            return Ok(0.0);
        }
        row_altitude(self, position, f64::from(altitude), context)
    }

    /// `kmlpolygonsoverlay` replaced - and `FlightData.kmlpolygons` cleared with it, as KML
    /// Overlay clears both before it reads a file.
    /// `// C#: GCSViews/FlightPlanner.cs:4141-4147`
    pub fn set_kml_overlay(&mut self, overlay: Option<mp_kml::read::Overlay>) {
        self.kml_overlay = overlay;
        self.kml_on_flight = false;
    }

    /// Yes to "Do you want to load this into the flight data screen?": the overlay's polygons and
    /// routes copied to `FlightData.kmlpolygons`; its labels stay on the planner.
    /// `// C#: GCSViews/FlightPlanner.cs:4246-4262`
    pub fn show_kml_on_flight_screen(&mut self) {
        self.kml_on_flight = self.kml_overlay.is_some();
    }

    /// What KML Overlay last read, if anything.
    #[must_use]
    pub fn kml_overlay(&self) -> Option<&mp_kml::read::Overlay> {
        self.kml_overlay.as_ref()
    }

    /// Whether the flight screen's map shows the overlay's polygons and routes too.
    #[must_use]
    pub const fn kml_on_flight(&self) -> bool {
        self.kml_on_flight
    }

    /// `GetBoundingLayer(kmlpolygonsoverlay)`: every point of its polygons, routes and markers,
    /// for the zoom to fit them.
    /// `// C#: GCSViews/FlightPlanner.cs:423-460`
    #[must_use]
    pub fn kml_points(&self) -> Vec<LatLon> {
        let Some(overlay) = &self.kml_overlay else {
            return Vec::new();
        };
        overlay
            .polygons
            .iter()
            .chain(&overlay.routes)
            .flat_map(|shape| shape.points.iter())
            .chain(overlay.labels.iter().map(|label| &label.at))
            .filter_map(|coord| LatLon::new(coord.lat, coord.lon).ok())
            .collect()
    }
}

impl Plan {
    /// `FillCommand(row, cmd, p1, p2, p3, p4, x, y, z)` on a row `AddCommand` just added: a
    /// WAYPOINT keeps `p1` and goes through `setfromMap(y, x, (int) z)` - as a SPLINE_WAYPOINT
    /// when the Spline box is ticked; a LOITER_UNLIM goes through `setfromMap` with nothing else;
    /// any other command has its cells written as given, latitude `y`, longitude `x`, altitude
    /// `z`, in the screen's frame.
    /// `// C#: GCSViews/FlightPlanner.cs:540-547, 548-578`
    pub fn add_command(
        &mut self,
        command: u16,
        params: [f64; 4],
        x: f64,
        y: f64,
        z: f64,
        context: &MenuContext,
    ) -> Result<(), &'static str> {
        let frame = context.frame.mav_frame();
        // `setfromMap(y, x, ...)`: a row's position is its latitude and longitude cells, which
        // a command without a position (DO_DIGICAM_CONTROL's 1, 0) still gets.
        let position = LatLon::new(y, x).map_err(|_| "Invalid coord, How did you do this?")?;
        #[allow(clippy::cast_possible_truncation)] // `(int) z`
        let alt = z as i32;
        let mut row = match command {
            mp_mission::commands::WAYPOINT => {
                let altitude = self.set_from_map_altitude(position, alt, context)?;
                let command = if self.spline() {
                    mp_mission::commands::SPLINE_WAYPOINT
                } else {
                    mp_mission::commands::WAYPOINT
                };
                let mut row = circle_row(command, frame, y, x, altitude);
                row.param1 = params[0];
                row
            }
            mp_mission::commands::LOITER_UNLIM => {
                let altitude = self.set_from_map_altitude(position, alt, context)?;
                circle_row(command, frame, y, x, altitude)
            }
            _ => {
                let mut row = circle_row(command, frame, y, x, z);
                row.param1 = params[0];
                row.param2 = params[1];
                row.param3 = params[2];
                row.param4 = params[3];
                row
            }
        };
        row.autocontinue = 1;
        self.items.push(row);
        self.renumber();
        self.origin = Origin::Edited;
        Ok(())
    }

    /// The geofence's exclusion polygons.
    #[must_use]
    pub fn fence_exclusions(&self) -> &[Vec<LatLon>] {
        &self.fence_exclusions
    }

    /// Fence Inclusion: the drawn polygon's corners become the inclusion fence - one polygon
    /// here, where the C# adds a `FENCE_POLYGON_VERTEX_INCLUSION` row per corner to the fence
    /// list and so can hold several - and the polygon is cleared.
    /// `// C#: GCSViews/FlightPlanner.cs:3297-3305`
    pub fn fence_inclusion_from_polygon(&mut self) {
        self.fence = self.polygon.clone();
        self.fence_error = None;
        self.load_polygon(Vec::new());
    }

    /// Fence Exclusion: the drawn polygon's corners become an exclusion polygon of the fence,
    /// and the polygon is cleared.
    /// `// C#: GCSViews/FlightPlanner.cs:3287-3295`
    pub fn fence_exclusion_from_polygon(&mut self) {
        if !self.polygon.is_empty() {
            self.fence_exclusions.push(self.polygon.clone());
        }
        self.load_polygon(Vec::new());
    }
}

/// Load KML File once its dialog has returned: `processKMLMission` over every element the
/// parser added - a POI per `Point` placemark (`POI.POIAdd(point, pm.Name)`, a missing name
/// being `null + "\n"`, an empty ID) and a row per `LineString` coordinate through `setfromMap`
/// with `(int) loc.Altitude`, which throws on a coordinate without one; the rows before it stay.
/// Returns how many rows were added. An error is the exception's text, for "Bad KML File :".
/// `// C#: GCSViews/FlightPlanner.cs:4470-4525, 4527-4577`
pub(crate) fn kml_into_mission(
    plan: &mut Plan,
    pois: &mut crate::poi::Pois,
    text: &str,
    context: &MenuContext,
) -> Result<usize, String> {
    let events = mp_kml::read::mission_events(text).map_err(|why| why.to_string())?;
    let mut rows = 0;
    for event in events {
        match event {
            mp_kml::read::MissionEvent::Poi { at, name } => {
                pois.add(at.lat, at.lon, 0.0, name.as_deref().unwrap_or(""));
            }
            mp_kml::read::MissionEvent::Path(coords) => {
                for coord in coords {
                    let altitude = coord
                        .alt
                        .ok_or_else(|| mp_kml::read::ReadError::NoAltitude.to_string())?;
                    let position = LatLon::new(coord.lat, coord.lon)
                        .map_err(|why| format!("System.ArgumentException: {why}"))?;
                    #[allow(clippy::cast_possible_truncation)] // `(int) loc.Altitude`
                    plan.add_row_from_map(position, altitude as i32, context)
                        .map_err(str::to_owned)?;
                    rows += 1;
                }
            }
        }
    }
    Ok(rows)
}

/// `LoadSHPFile(file)`: the `.prj` beside the file parsed for a reprojection, then a row per
/// record of the table - `fs.Vertex[row * 2]` and `[row * 2 + 1]`, the `row`th vertex of the
/// whole file whatever feature it belongs to; its altitude from an `ELEVATION` column, else an
/// `alt` column, else the shape's Z, else -1 (which `setfromMap` writes as nothing); a `wp`
/// column sorts the rows by it, ascending (`PointLatLngAlt.CompareTo`); then `AddCommand(WAYPOINT,
/// 0, 0, 0, 0, lat, lng, alt)` for each. Returns how many rows were added; an error is anything
/// the handler's `catch` would take, after the rows already added.
/// `// C#: GCSViews/FlightPlanner.cs:4569-4716, 4718-4737; ExtLibs/Utilities/PointLatLngAlt.cs:441-462`
pub(crate) fn shp_into_mission(
    plan: &mut Plan,
    shp: &[u8],
    dbf: &[u8],
    prj: Option<&str>,
    context: &MenuContext,
) -> Result<usize, String> {
    let projection = match prj {
        Some(text) => Some(
            mp_mission::shapefile::Projection::from_esri(text.lines().next().unwrap_or(""))
                .map_err(|why| why.to_string())?,
        ),
        None => None,
    };
    let vertices = mp_mission::shapefile::vertices(shp).map_err(|why| why.to_string())?;
    let table = mp_mission::dbf::read(dbf).map_err(|why| why.to_string())?;
    let elevation = table.column("ELEVATION");
    let alt = table.column("alt");
    let wp = table.column("wp");
    // `(float) Convert.ChangeType(value, TypeCode.Single)`; a value that will not convert throws,
    // is logged, and leaves the altitude as it was.
    let single = |row: usize, column: Option<usize>| -> Option<f64> {
        column
            .and_then(|column| table.value(row, column))
            .and_then(|value| value.parse::<f32>().ok())
            .map(f64::from)
    };
    let mut list: Vec<(LatLon, f64, f64)> = Vec::with_capacity(table.rows());
    let mut sort = false;
    for row in 0..table.rows() {
        let &(x, y) = vertices.xy.get(row).ok_or_else(|| {
            "System.IndexOutOfRangeException: Index was outside the bounds of the array.".to_owned()
        })?;
        let mut z = -1.0;
        if let Some(value) = single(row, elevation) {
            z = value;
        }
        if z == -1.0
            && let Some(value) = single(row, alt)
        {
            z = value;
        }
        // `fs.Z[row]`: null for a file without Z, and the exception leaves -1.
        if z == -1.0
            && let Some(value) = vertices.z.as_ref().and_then(|zs| zs.get(row))
        {
            z = *value;
        }
        let mut order = 0.0;
        if wp.is_some() {
            sort = true;
            if let Some(value) = single(row, wp) {
                order = value;
            }
        }
        // `new PointLatLngAlt(x, y, z, tag)` then `AddCommand(..., item.Lat, item.Lng, ...)` and
        // `setfromMap(y, x, ...)`: swapped twice, the row's latitude is the vertex's Y.
        let position = mp_mission::shapefile::position(projection, x, y)
            .ok_or_else(|| "System.ArgumentException: the vertex is not a position".to_owned())?;
        list.push((position, z, order));
    }
    if sort {
        // `wplist.Sort()` on `Tag`, the wp number as text: ascending, equal tags left as they are.
        list.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
    }
    let added = list.len();
    for (position, z, _) in list {
        #[allow(clippy::cast_possible_truncation)] // `setfromMap(y, x, (int) z, ...)`
        plan.add_command_waypoint(position, z as i32, context)
            .map_err(str::to_owned)?;
    }
    Ok(added)
}

/// A `.kml`'s text, or the first `.kml` at the top of a `.kmz` (`Directory.GetFiles(tempdir,
/// "*.kml")`, its first), snippets dropped. `None` for a `.kmz` with no `.kml` in it, which the
/// C# returns from without a word. An error is the exception's text.
/// `// C#: GCSViews/FlightPlanner.cs:4152-4176, 4481-4513`
pub(crate) fn kml_text(name: &str, bytes: &[u8]) -> Result<Option<String>, String> {
    // `file.ToLower().EndsWith("kmz")`
    let text = if name.to_lowercase().ends_with("kmz") {
        let entries =
            mp_log::zip::read(bytes).map_err(|why| format!("Ionic.Zip.ZipException: {why}"))?;
        let Some(entry) = entries
            .into_iter()
            .find(|entry| !entry.name.contains('/') && entry.name.ends_with(".kml"))
        else {
            return Ok(None);
        };
        String::from_utf8_lossy(&entry.data).into_owned()
    } else {
        String::from_utf8_lossy(bytes).into_owned()
    };
    Ok(Some(mp_kml::read::without_snippets(&text)))
}

/// What one of the four loaders leaves for the screen to say and show.
type Refusal = Option<(&'static str, String)>;

/// A path's last part, as `Path.GetFileName` gives it.
fn file_name_of(path: &Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |name| name.to_string_lossy().into_owned(),
    )
}

/// Load File once its dialog has returned: `if (File.Exists(file))`, the folder remembered as
/// `WPFileDirectory`, and the file read by its kind - a `.shp` through `LoadSHPFile`, a `.kml`
/// through the KML parser, a JSON mission (`MissionFile.ReadFile`, not ported: said on the status
/// line), anything else as a waypoint file.
/// `// C#: GCSViews/FlightPlanner.cs:1817-1887`
fn load_mission(this: &mut MissionPlanner, name: &str) -> Refusal {
    let path = dialog_path(name, "waypoints", &this.plan_menus.dialog_directory)?;
    if !path.is_file() {
        this.file_status = Some(format!(
            "could not read {}: there is no such file",
            path.display()
        ));
        return None;
    }
    this.remember_dialog_directory(&path);
    let lower = path.to_string_lossy().to_lowercase();
    let answer = path.display().to_string();
    if lower.ends_with(".shp") {
        return load_shp_mission(this, &answer);
    }
    if lower.ends_with(".kml") {
        return load_kml_mission(this, &answer);
    }
    // `line = fs.ReadLine(); if (line.StartsWith("{"))`: a JSON mission.
    let json = std::fs::read_to_string(&path)
        .is_ok_and(|text| text.lines().next().is_some_and(|line| line.starts_with('{')));
    if json {
        this.file_status = Some(format!(
            "{} is a JSON mission (MissionFile), which this planner does not read yet",
            path.display()
        ));
        return None;
    }
    this.load_plan_from(&path);
    None
}

/// Save File once its dialog has returned: `savewaypoints`' QGC WPL 110 file, `.waypoints` added
/// to a name without an extension (`DefaultExt`), and the folder remembered as `WPFileDirectory`.
/// A `.mission` - the Mission JSON filter's - is `MissionFile.WriteFile`, not ported: said on the
/// status line.
/// `// C#: GCSViews/FlightPlanner.cs:6069-6140`
fn save_mission(this: &mut MissionPlanner, name: &str) -> Refusal {
    let path = dialog_path(name, "waypoints", &this.plan_menus.dialog_directory)?;
    if path
        .extension()
        .is_some_and(|extension| extension.eq_ignore_ascii_case("mission"))
    {
        this.file_status = Some(format!(
            "{} would be a JSON mission (MissionFile), which this planner does not write yet",
            path.display()
        ));
        return None;
    }
    this.remember_dialog_directory(&path);
    this.save_plan_to(&path);
    None
}

/// Load and Append once its dialog has returned: `readQGC110wpfile(file, true)`.
/// `// C#: GCSViews/FlightPlanner.cs:990-1010, 4330-4344`
fn load_and_append(this: &mut MissionPlanner, name: &str) -> Refusal {
    let path = dialog_path(name, "waypoints", &this.plan_menus.dialog_directory)?;
    let read = std::fs::read_to_string(&path)
        .map_err(|err| format!("System.IO.FileNotFoundException: {err}"))
        .and_then(|text| {
            mp_mission::read_waypoints(&text)
                .map_err(|err| format!("System.FormatException: {err}"))
        });
    match read {
        Ok(items) => {
            let added = this.plan.append_from_file(&items);
            // `processToScreen` ends with `setWPParams`; `readQGC110wpfile` with the map fitted
            // to the mission. `// C#: GCSViews/FlightPlanner.cs:1004-1006, 5630`
            this.plan.set_wp_params(&this.telemetry.view().parameters);
            this.sync_map_mission();
            this.map.borrow_mut().zoom_and_centre_markers();
            this.file_status = Some(format!("appended {added} items from {}", path.display()));
            None
        }
        // `CustomMessageBox.Show("Can't open file! " + ex)`
        Err(why) => Some(("", format!("Can't open file! {why}"))),
    }
}

/// Load KML File once its dialog has returned.
/// `// C#: GCSViews/FlightPlanner.cs:4470-4525`
fn load_kml_mission(this: &mut MissionPlanner, name: &str) -> Refusal {
    let path = dialog_path(name, "kml", &this.plan_menus.dialog_directory)?;
    let name = file_name_of(&path);
    let context = menu_context(this);
    let loaded = std::fs::read(&path)
        .map_err(|err| format!("System.IO.FileNotFoundException: {err}"))
        .and_then(|bytes| kml_text(&name, &bytes))
        .and_then(|text| match text {
            Some(text) => {
                kml_into_mission(&mut this.plan, &mut this.fly_data.pois, &text, &context).map(Some)
            }
            None => Ok(None),
        });
    match loaded {
        Ok(Some(rows)) => {
            this.file_status = Some(format!("loaded {rows} rows from {}", path.display()));
            None
        }
        Ok(None) => None,
        Err(why) => Some(("", format!("{BAD_KML_FILE}{why}"))),
    }
}

/// Load SHP File once its dialog has returned: `LoadSHPFile(file)` inside the handler's `catch`,
/// which says "Error opening File". A name the dialog did not return (`File.Exists("")`) does
/// nothing; so does a file that is not there.
/// `// C#: GCSViews/FlightPlanner.cs:4718-4737`
fn load_shp_mission(this: &mut MissionPlanner, name: &str) -> Refusal {
    let path = dialog_path(name, "shp", &this.plan_menus.dialog_directory)?;
    let Ok(shp) = std::fs::read(&path) else {
        return None;
    };
    let context = menu_context(this);
    // `FeatureSet.Open` wants the table beside the shapes; the `.prj` is optional.
    let loaded = std::fs::read(path.with_extension("dbf"))
        .map_err(|err| err.to_string())
        .and_then(|dbf| {
            let prj = std::fs::read_to_string(path.with_extension("prj")).ok();
            shp_into_mission(&mut this.plan, &shp, &dbf, prj.as_deref(), &context)
        });
    match loaded {
        Ok(rows) => {
            // `writeKML(); MainMap.ZoomAndCenterMarkers("WPOverlay")`
            this.sync_map_mission();
            this.map.borrow_mut().zoom_and_centre_markers();
            this.file_status = Some(format!("loaded {rows} rows from {}", path.display()));
            None
        }
        Err(why) => {
            this.file_status = Some(format!("{}: {why}", path.display()));
            Some((ERROR, "Error opening File".to_owned()))
        }
    }
}

/// KML Overlay once its dialog has returned: both overlays cleared, then the file read by its
/// extension - KML and KMZ here; DXF (netDxf) and GeoPackage (GDAL's OGR) are not ported, and
/// say so on the status line - and the two questions asked.
/// `// C#: GCSViews/FlightPlanner.cs:4131-4290`
fn load_kml_overlay(this: &mut MissionPlanner, name: &str) -> Refusal {
    let path = dialog_path(name, "kml", &this.plan_menus.dialog_directory)?;
    let name = file_name_of(&path);
    this.plan.set_kml_overlay(None);
    let lower = name.to_lowercase();
    if lower.ends_with("gpkg") {
        this.file_status =
            Some("GeoPackage overlays need GDAL's OGR, which is not ported".to_owned());
        return None;
    }
    if lower.ends_with("dxf") {
        this.file_status = Some("DXF overlays need netDxf, which is not ported".to_owned());
        return None;
    }
    let loaded = std::fs::read(&path)
        .map_err(|err| format!("System.IO.FileNotFoundException: {err}"))
        .and_then(|bytes| kml_text(&name, &bytes))
        .and_then(|text| match text {
            Some(text) => mp_kml::read::overlay(&text)
                .map(Some)
                .map_err(|why| why.to_string()),
            None => Ok(None),
        });
    match loaded {
        Ok(Some(overlay)) => {
            this.file_status = Some(format!(
                "overlaid {} polygons, {} routes and {} labels from {}",
                overlay.polygons.len(),
                overlay.routes.len(),
                overlay.labels.len(),
                path.display()
            ));
            this.plan.set_kml_overlay(Some(overlay));
            this.plan_menus.ask_kml_to_flight_screen();
            None
        }
        Ok(None) => None,
        Err(why) => Some(("", format!("{BAD_KML_FILE}{why}"))),
    }
}

/// What an `InputBox` or `CustomMessageBox` the menu opened is for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PromptKind {
    /// "Insert WP after wp#", for a waypoint or a spline waypoint at the menu's position.
    InsertWp {
        /// Insert Spline WP rather than Insert Wp.
        spline: bool,
        /// Where the menu was opened.
        position: LatLon,
    },
    /// "Loiter Time".
    LoiterTime {
        /// Where the menu was opened.
        position: LatLon,
    },
    /// "Loiter Turns".
    LoiterTurns {
        /// Where the menu was opened.
        position: LatLon,
    },
    /// Jump > Start's "Number of times to Repeat".
    JumpStartRepeat,
    /// Jump > WP #'s "Jump to WP no?".
    JumpWpNumber,
    /// Jump > WP #'s "Number of times to Repeat", once the target is known.
    JumpWpRepeat {
        /// The item to jump to.
        target: f64,
    },
    /// Takeoff's "Please enter your takeoff altitude in m".
    TakeoffAltitude {
        /// Whether a pitch is asked for next.
        ask_pitch: bool,
    },
    /// Takeoff's "Please enter your takeoff pitch", on ArduPlane.
    TakeoffPitch {
        /// The altitude already given.
        altitude: f64,
    },
    /// Modify Alt's change.
    ModifyAlt,
    /// From Current Waypoints' "Clear current waypoints?", Yes or No.
    ClearWaypoints,
    /// `processToScreen`'s "Reset Home to loaded coords", Yes or No, with the home a mission
    /// just read carries at item 0.
    ResetHome(Home),
    /// Geo-Fence > Load from File's `OpenFileDialog`, filtered to `Fence (*.fen)`: a name typed
    /// here, read from the plan directory, as the mission file is.
    FenceLoadFile,
    /// Geo-Fence > Save to File's `SaveFileDialog`, likewise.
    FenceSaveFile,
    /// Map Tool > Zoom To's "Enter your location".
    ZoomTo,
    /// Polygon > Save Polygon's `SaveFileDialog`, filtered to `Polygon (*.poly)`.
    PolygonSaveFile,
    /// Polygon > Load Polygon's `OpenFileDialog`, likewise.
    PolygonLoadFile,
    /// Polygon > From SHP's `OpenFileDialog`, filtered to `Shape file`.
    ShpLoadFile,
    /// Polygon > Offset Polygon's "Please enter the offset in meters".
    OffsetPolygon,
    /// Area's "Please define a polygon!", after which the area is said all the same.
    DefinePolygon,
    /// Rally Points > Set Rally Point's "Altitude".
    RallyAltitude {
        /// Where the menu was opened.
        position: LatLon,
    },
    /// Rally Points > Save Rally to File's `SaveFileDialog`, filtered to `Rally (*.ral)`.
    RallySaveFile,
    /// Rally Points > Load Rally from File's `OpenFileDialog`, likewise.
    RallyLoadFile,
    /// Load File's `OpenFileDialog`, filtered to `All Supported Types`.
    /// `// C#: GCSViews/FlightPlanner.cs:1817-1823`
    MissionLoadFile,
    /// Save File's `SaveFileDialog`, filtered to `Mission`, opened on the mission's file name.
    /// `// C#: GCSViews/FlightPlanner.cs:6069-6077`
    MissionSaveFile,
    /// One of Create Wp Circle's or Create Spline Circle's questions, all asked before any is
    /// read; the answers so far are in [`PlanMenus`].
    Circle {
        /// Create Spline Circle rather than Create Wp Circle.
        spline: bool,
        /// Where the menu was opened.
        position: LatLon,
    },
    /// A message with an OK.
    Message,
    /// Write's "Absolute Alt is selected are you sure?".
    WriteAltMode,
    /// `checkZeroAlts`' warning, OK or Cancel.
    WriteZeroAlt,
    /// Prefetch's "No ripp area defined, ripp displayed on screen?".
    PrefetchRipArea,
    /// Prefetch WP Path's "max zoom".
    PrefetchMaxZoom,
    /// `TXT_homelat_Enter`'s "Click on the Map to set Home ": a message, after which the Lat box
    /// has the keyboard again, as it does when the C#'s modal box closes.
    HomeLatEnter,
    /// File Load/Save > Load and Append's `OpenFileDialog`, filtered to `Ardupilot Mission`.
    AppendLoadFile,
    /// File Load/Save > Load KML File's `OpenFileDialog`, filtered to `Google Earth KML `.
    KmlLoadFile,
    /// File Load/Save > Load SHP File's `OpenFileDialog`, filtered to `Shape file`.
    ShpMissionLoadFile,
    /// Map Tool > KML Overlay's `OpenFileDialog`, filtered to `All Supported`.
    KmlOverlayFile,
    /// Inject Custom Map's `FolderBrowserDialog`: the folder of tiles, typed.
    InjectCustomMapFolder,
    /// KML Overlay's "Do you want to load this into the flight data screen?", Yes or No.
    KmlToFlightScreen,
    /// KML Overlay's "Zoom to the center or the loaded file?", Yes or No.
    KmlZoomTo,
    /// One of Create Circle Survey's six boxes, `step` 0 to 5, at the menu's position.
    CircleSurvey {
        /// Which of the six.
        step: u8,
        /// `MouseDownEnd`, the survey's centre.
        position: LatLon,
    },
    /// Enter UTM Coord's "Zone".
    UtmZone,
    /// Enter UTM Coord's "Easting".
    UtmEasting,
    /// Enter UTM Coord's "Northing".
    UtmNorthing,
    /// Tracker Home's "Tracker Alt", at the menu's position.
    TrackerAlt {
        /// `MouseDownEnd`, where the tracker is.
        position: LatLon,
    },
    /// POI > Add's "Enter ID", for a point at the menu's position.
    PlanPoiId {
        /// `MouseDownStart`.
        position: LatLon,
    },
    /// POI > Edit's "Enter ID", for the point the menu opened over.
    PlanPoiEdit {
        /// Its index in the list.
        index: usize,
    },
    /// Text's "Enter String", at the menu's position.
    TextString {
        /// `MouseDownStart`, where the text starts.
        position: LatLon,
    },
    /// Text's "Enter size".
    TextSize {
        /// Where the text starts.
        position: LatLon,
    },
    /// Text's "Enter rotation".
    TextRotation {
        /// Where the text starts.
        position: LatLon,
    },
    /// Geo-Fence > Upload's "Box Minimum Altitude?".
    FenceMinAlt,
    /// Geo-Fence > Upload's "Box Maximum Altitude?".
    FenceMaxAlt,
}

/// What POI > Add or Edit asks the screen to do to the flight screen's list once its ID is typed.
#[derive(Debug, Clone, PartialEq)]
pub enum PoiRequest {
    /// `POI.POIAdd(point, id)`.
    Add {
        /// Latitude.
        lat: f64,
        /// Longitude.
        lng: f64,
        /// The ID typed.
        id: String,
    },
    /// `POI.POIEdit`: the point's tag rewritten with the ID typed.
    Rename {
        /// Its index in the list.
        index: usize,
        /// The ID typed.
        id: String,
    },
}

/// A file a dialog has named, for the screen to read or write.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FileRequest {
    /// Geo-Fence > Load from File.
    LoadFence(String),
    /// Geo-Fence > Save to File.
    SaveFence(String),
    /// Polygon > Save Polygon.
    SavePolygon(String),
    /// Polygon > Load Polygon.
    LoadPolygon(String),
    /// Polygon > From SHP. Empty when the dialog was cancelled, which the C# goes on with.
    LoadShp(String),
    /// Rally Points > Save Rally to File.
    SaveRally(String),
    /// Rally Points > Load Rally from File.
    LoadRally(String),
    /// File Load/Save > Load and Append.
    LoadAndAppend(String),
    /// File Load/Save > Load KML File.
    LoadKml(String),
    /// File Load/Save > Load SHP File.
    LoadShpMission(String),
    /// Map Tool > KML Overlay.
    KmlOverlay(String),
    /// Inject Custom Map's folder.
    InjectCustomMap(String),
    /// Load File.
    LoadMission(String),
    /// Save File.
    SaveMission(String),
}

/// What `lnk_kml` opens: the built-in HTTP server's network link for Google Earth.
/// `// C#: GCSViews/FlightPlanner.cs:4322`
pub const NETWORK_KML_URL: &str = "http://127.0.0.1:56781/network.kml";

/// The caption of the dialog standing in for `OpenFileDialog`: its own default.
pub const OPEN_FILE: &str = "Open";
/// The caption of the dialog standing in for `SaveFileDialog`: its own default.
pub const SAVE_FILE: &str = "Save As";
/// The filter both fence dialogs are given.
/// `// C#: GCSViews/FlightPlanner.cs:4349, 5969`
pub const FENCE_FILTER: &str = "Fence (*.fen)";
/// The filter both polygon dialogs are given.
/// `// C#: GCSViews/FlightPlanner.cs:4532, 5902`
pub const POLYGON_FILTER: &str = "Polygon (*.poly)";
/// The filter From SHP's dialog is given, `"Shape file|*.shp"`.
/// `// C#: GCSViews/FlightPlanner.cs:3537`
pub const SHP_FILTER: &str = "Shape file";
/// The filter both rally dialogs are given.
/// `// C#: GCSViews/FlightPlanner.cs:4419, 6038`
pub const RALLY_FILTER: &str = "Rally (*.ral)";
/// Load File's `All Supported Types|*.txt;*.waypoints;*.shp;*.plan;*.kml`.
/// `// C#: GCSViews/FlightPlanner.cs:1821`
pub const MISSION_LOAD_FILTER: &str = "All Supported Types";
/// Save File's `Mission|*.waypoints;*.txt|Mission JSON|*.mission`.
/// `// C#: GCSViews/FlightPlanner.cs:6073`
pub const MISSION_SAVE_FILTER: &str = "Mission";
/// Load and Append's `Ardupilot Mission|*.waypoints;*.txt`.
/// `// C#: GCSViews/FlightPlanner.cs:4334`
pub const MISSION_FILTER: &str = "Ardupilot Mission";
/// Load KML File's `Google Earth KML |*.kml;*.kmz`, the C#'s trailing space kept.
/// `// C#: GCSViews/FlightPlanner.cs:4474`
pub const KML_FILTER: &str = "Google Earth KML ";
/// KML Overlay's `All Supported|*.kml;*.kmz;*.dxf;*.gpkg|...`.
/// `// C#: GCSViews/FlightPlanner.cs:4135-4136`
pub const KML_OVERLAY_FILTER: &str = "All Supported";
/// `Strings.Load_data`, the title of KML Overlay's first question.
pub const LOAD_DATA: &str = "Load data";
/// `Strings.Do_you_want_to_load_this_into_the_flight_data_screen`.
pub const LOAD_INTO_FLIGHT_SCREEN: &str = "Do you want to load this into the flight data screen?";
/// `Strings.Zoom_To`, the title of KML Overlay's second question.
pub const ZOOM_TO_TITLE: &str = "Zoom To";
/// `Strings.Zoom_to_the_center_or_the_loaded_file`.
pub const ZOOM_TO_LOADED: &str = "Zoom to the center or the loaded file?";
/// `Strings.Bad_KML_File`, which both KML loaders put before the exception.
pub const BAD_KML_FILE: &str = "Bad KML File :";
/// `Strings.InvalidAlt`, what Set Rally Point says of an altitude `int.TryParse` refuses.
/// `// C#: GCSViews/FlightPlanner.cs:6657; ExtLibs/Strings/Strings.resx:174-176`
pub const INVALID_ALT: &str = "Invalid Alt";
/// What Save Rally to File says with no rally points.
/// `// C#: GCSViews/FlightPlanner.cs:6022-6026`
pub const SET_SOME_RALLY_POINTS: &str = "Please set some rally points";
/// What Save Rally to File says when writing throws.
/// `// C#: GCSViews/FlightPlanner.cs:6058-6061`
pub const RALLY_FILE_FAILED: &str = "Failed to write rally file";
/// What Rally Points > Upload says when a point cannot be sent or read back.
/// `// C#: GCSViews/FlightPlanner.cs:5951-5955`
pub const RALLY_SAVE_FAILED: &str = "Failed to save rally point";
/// `Strings.PleaseConnect`, what Rally Points > Download says without a link.
/// `// C#: GCSViews/FlightPlanner.cs:919-923; ExtLibs/Strings/Strings.resx:205-207`
pub const PLEASE_CONNECT: &str = "Please connect first";
/// What Area says with no polygon, before it says the area of nothing.
/// `// C#: GCSViews/FlightPlanner.cs:1992-1996`
pub const DEFINE_POLYGON: &str = "Please define a polygon!";

/// A dialog: an `InputBox` with its field, a Yes/No question, or a message.
///
/// Drawn over the screen and taking the keyboard, as the C#'s modal forms do. Enter is OK and
/// Escape is Cancel, as `InputBox`'s accept and cancel buttons are.
#[derive(Debug)]
pub struct Prompt {
    /// The window caption.
    pub title: &'static str,
    /// The prompt text.
    pub text: String,
    /// The value being typed, for an `InputBox`; `None` for a question or a message.
    pub field: Option<TextField>,
    /// What OK does.
    pub kind: PromptKind,
    /// `MessageShowAgain`'s "Show me again?" tick, for a box that has one.
    pub show_again: Option<ShowAgain>,
}

/// `Common.MessageShowAgain`'s tick: the setting it is kept under and whether it is ticked -
/// ticked to start, and written at every click as `chk_CheckStateChanged` writes it.
/// `// C#: Common.cs:260-270, 390-400, 446-449`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShowAgain {
    /// `SHOWAGAIN_` and the tag with its spaces, `+`, `-` and `.` made underscores.
    pub key: &'static str,
    /// `Checked`.
    pub ticked: bool,
}

impl ShowAgain {
    /// Whether the setting says the box is not to show: there and not true, as
    /// `GetBoolean(key) == false` reads it.
    /// `// C#: Common.cs:268-270`
    #[must_use]
    pub fn suppressed(setting: Option<&str>) -> bool {
        setting.and_then(mp_mission::dotnet::parse_bool) == Some(false)
    }
}

impl Prompt {
    /// An `InputBox.Show(title, text, ref value)`.
    fn input(title: &'static str, text: &str, value: impl Into<String>, kind: PromptKind) -> Self {
        let mut field = TextField::new("");
        field.set(value);
        Self {
            title,
            text: text.to_owned(),
            field: Some(field),
            kind,
            show_again: None,
        }
    }

    /// A `CustomMessageBox.Show(text, title)`.
    fn message(title: &'static str, text: impl Into<String>) -> Self {
        Self {
            title,
            text: text.into(),
            field: None,
            kind: PromptKind::Message,
            show_again: None,
        }
    }

    /// A `CustomMessageBox.Show(text, title, MessageBoxButtons.YesNo)`.
    fn question(title: &'static str, text: impl Into<String>, kind: PromptKind) -> Self {
        Self {
            title,
            text: text.into(),
            field: None,
            kind,
            show_again: None,
        }
    }

    /// The same box as `Common.MessageShowAgain` shows it: "Show me again?" ticked under the
    /// text, OK, and Cancel for a question. The caller checks the setting first, as
    /// `MessageShowAgain` returns OK at once when the tick was cleared.
    /// `// C#: Common.cs:260-443`
    #[must_use]
    pub fn with_show_again(mut self, key: &'static str) -> Self {
        self.show_again = Some(ShowAgain { key, ticked: true });
        self
    }

    /// What has been typed, or nothing.
    #[must_use]
    pub fn value(&self) -> &str {
        self.field.as_ref().map_or("", TextField::value)
    }

    /// Whether this stands for an `OpenFileDialog` or a `SaveFileDialog`, not an `InputBox`: its
    /// answer is a path, and nothing keeps it.
    #[must_use]
    pub const fn is_file_dialog(&self) -> bool {
        matches!(
            self.kind,
            PromptKind::FenceLoadFile
                | PromptKind::FenceSaveFile
                | PromptKind::PolygonSaveFile
                | PromptKind::PolygonLoadFile
                | PromptKind::ShpLoadFile
                | PromptKind::RallySaveFile
                | PromptKind::RallyLoadFile
                | PromptKind::MissionLoadFile
                | PromptKind::MissionSaveFile
                | PromptKind::AppendLoadFile
                | PromptKind::KmlLoadFile
                | PromptKind::ShpMissionLoadFile
                | PromptKind::KmlOverlayFile
                | PromptKind::InjectCustomMapFolder
        )
    }

    /// The extensions a file dialog's list shows, from the C#'s `Filter`; none for the folder
    /// dialog, whose list holds folders only.
    #[must_use]
    pub const fn file_types(&self) -> &'static [&'static str] {
        match self.kind {
            PromptKind::FenceLoadFile | PromptKind::FenceSaveFile => &["fen"],
            PromptKind::PolygonSaveFile | PromptKind::PolygonLoadFile => &["poly"],
            PromptKind::ShpLoadFile | PromptKind::ShpMissionLoadFile => &["shp"],
            PromptKind::RallySaveFile | PromptKind::RallyLoadFile => &["ral"],
            PromptKind::AppendLoadFile => &["waypoints", "txt"],
            PromptKind::KmlLoadFile => &["kml", "kmz"],
            PromptKind::KmlOverlayFile => &["kml", "kmz", "dxf", "gpkg"],
            // `"All Supported Types|*.txt;*.waypoints;*.shp;*.plan;*.kml"`
            PromptKind::MissionLoadFile => &["txt", "waypoints", "shp", "plan", "kml"],
            // `"Mission|*.waypoints;*.txt|Mission JSON|*.mission"`
            PromptKind::MissionSaveFile => &["waypoints", "txt", "mission"],
            _ => &[],
        }
    }

    /// Whether this asks Yes or No.
    #[must_use]
    pub const fn is_question(&self) -> bool {
        matches!(
            self.kind,
            PromptKind::ClearWaypoints
                | PromptKind::ResetHome(_)
                | PromptKind::KmlToFlightScreen
                | PromptKind::KmlZoomTo
                | PromptKind::WriteAltMode
                | PromptKind::WriteZeroAlt
                | PromptKind::PrefetchRipArea
        )
    }
}

/// `Strings.InvalidNumberEntered`, without the resource's trailing newline.
const INVALID_NUMBER: &str = "Invalid number entered";

/// What `int.Parse` or `double.Parse` throws on a word, as the application's handler shows it.
fn format_exception(text: &str) -> String {
    format!(
        "System.FormatException: The input string '{}' was not in a correct format.",
        text.trim()
    )
}

/// The altitude `setfromMap(lat, lng, passed)` gives a new row at `position`: Default Alt and the
/// altitude passed ([`Plan::new_row_altitude`]), then Verify Height ([`Plan::verified_altitude`])
/// in the screen's frame.
/// `// C#: GCSViews/FlightPlanner.cs:1164-1236`
fn row_altitude(
    plan: &Plan,
    position: LatLon,
    passed: f64,
    context: &MenuContext,
) -> Result<f64, &'static str> {
    let altitude = plan.new_row_altitude(passed, context.copter)?;
    Ok(plan.verified_altitude(
        position.latitude(),
        position.longitude(),
        altitude,
        context.frame,
    ))
}

/// A circle's row: `command` at a latitude and longitude as the C# computed them - nothing wraps
/// a longitude past 180, as nothing in the C# does - and `altitude`, the rest as
/// `Commands_RowsAdded` leaves a row.
fn circle_row(command: u16, frame: u8, lat: f64, lng: f64, altitude: f64) -> MissionItem {
    MissionItem {
        seq: 0,
        current: 0,
        frame,
        command,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: lat,
        y: lng,
        z: altitude,
        autocontinue: 1,
    }
}

/// Area's message: `"Area: " + aream2.ToString("0") + " m2\n\t" + areaa.ToString("0.00") +
/// " Acre\n\t" + areaha.ToString("0.00") + " Hectare\n\t" + areasqf.ToString("0") + " sqf"`, the
/// custom formats rounding fifteen significant digits half away from zero.
/// `// C#: GCSViews/FlightPlanner.cs:1764-1772`
#[must_use]
pub fn area_text(aream2: f64) -> String {
    use mp_mission::dotnet::format_f64;
    let areaa = aream2 * 0.000_247_105;
    let areaha = aream2 * 1e-4;
    let areasqf = aream2 * 10.7639;
    format!(
        "Area: {} m2\n\t{} Acre\n\t{} Hectare\n\t{} sqf",
        format_f64(aream2, "0"),
        format_f64(areaa, "0.00"),
        format_f64(areaha, "0.00"),
        format_f64(areasqf, "0")
    )
}

/// What the menu needs to know that the plan does not.
#[derive(Debug, Clone, Copy)]
pub struct MenuContext {
    /// `CMB_altmode`: the frame a new row gets.
    pub frame: AltitudeFrame,
    /// The vehicle's position and altitude above home, for Insert Wp > At Current Position.
    pub vehicle: Option<(LatLon, f64)>,
    /// Whether Takeoff asks for a pitch: on ArduPlane, unless `Q_OPTIONS` lacks bit 1.
    pub takeoff_pitch: bool,
    /// `cs.firmware == Firmwares.ArduCopter2`, for a Default Alt of 0.
    pub copter: bool,
    /// What Tracker Home offers: `cs.TrackerLocation.Alt` when it is not 0, else `cs.HomeAlt`.
    /// `// C#: GCSViews/FlightPlanner.cs:6972-6974`
    pub tracker_alt: f64,
}

impl MenuContext {
    /// Whether Takeoff asks for a pitch, from the vehicle's `MAV_TYPE` and parameters.
    ///
    /// `cs.firmware == Firmwares.ArduPlane` is the C#'s test, and `ArduPlane` is what it sets for
    /// a fixed wing, a flapping wing and every VTOL type (`MAVLinkInterface.cs:6724-6735`). A
    /// quadplane whose `Q_OPTIONS` lacks bit 1 skips the question.
    /// `// C#: GCSViews/FlightPlanner.cs:6797-6813`
    #[must_use]
    pub fn asks_takeoff_pitch(mav_type: Option<u8>, parameters: &[(String, f64)]) -> bool {
        let plane = matches!(mav_type, Some(1 | 16 | 19..=25));
        if !plane {
            return false;
        }
        match parameters.iter().find(|(name, _)| name == "Q_OPTIONS") {
            #[allow(clippy::cast_possible_truncation)] // `(int)MAV.param["Q_OPTIONS"]`
            Some((_, value)) => (*value as i64) & (1 << 1) != 0,
            None => true,
        }
    }
}

/// The distance and bearing Measure Distance reports, in the C#'s words:
/// `"Distance: " + FormatDistance(GetDistance(a, b), true) + " AZ: " + GetBearing(a, b)`.
///
/// `GetDistance` is GMap's `PureProjection.GetDistance`: a haversine on the projection's `Axis`,
/// 6378137 m for Mercator, in kilometres; `FormatDistance(km, true)` in the default metres writes
/// `(km * 1000).ToString("0.00 m")`. The bearing is `PureProjection.GetBearing` as `"0"`.
/// `// C#: GCSViews/FlightPlanner.cs:2649-2655, 3505-3531; ExtLibs/GMap.NET.Core/GMap.NET/PureProjection.cs:436-472`
#[must_use]
pub fn measure_text(from: LatLon, to: LatLon) -> String {
    let (lat1, lng1) = (from.latitude().to_radians(), from.longitude().to_radians());
    let (lat2, lng2) = (to.latitude().to_radians(), to.longitude().to_radians());
    let a = ((lat2 - lat1) / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * ((lng2 - lng1) / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    let kilometres = (6_378_137.0 / 1000.0) * c;
    let bearing = from.bearing_to(to).0.0;
    // .NET's custom numeric formats round half away from zero; `{:.2}` rounds half to even.
    let metres = (kilometres * 1000.0 * 100.0).round() / 100.0;
    format!("Distance: {metres:.2} m AZ: {:.0}", bearing.round())
}

/// The planning screen's menu and the dialogs it opens.
#[derive(Debug, Default)]
pub struct PlanMenus {
    /// The menu, while it is open.
    pub open: Option<OpenMenu>,
    /// The folder the file dialogs open in - the remembered `WPFileDirectory`, else the plan
    /// directory - which the screen keeps up to date each frame, for a dialog's list of what is
    /// there and for the paths typed into it.
    pub dialog_directory: PathBuf,
    /// The dialog, while one is showing.
    pub prompt: Option<Prompt>,
    /// Measure Distance's first point, `startmeasure`, once chosen.
    pub measure_from: Option<LatLon>,
    /// Where the press that closed the menu went down. A click off an open menu closes it and
    /// does nothing else, so the map must not take the same press as a click that adds a
    /// waypoint.
    dismissed_at: Option<(f32, f32)>,
    /// The zoom icon's menu, `contextMenuStripZoom`, while it is open: where it was opened.
    pub zoom_menu: Option<(f32, f32)>,
    /// Where the zoom bar, `TRK_zoom`, was laid out at the last frame: its top and height in
    /// window coordinates, which turn a press on it into a zoom.
    zoom_track: std::rc::Rc<std::cell::Cell<Option<(f32, f32)>>>,
    /// A Zoom To search on its way to the geocoder.
    geocoding: Option<Geocoding>,
    /// The answers Create Wp Circle or Create Spline Circle has had so far: each `InputBox` is
    /// asked before any answer is read.
    circle_answers: Vec<String>,
    /// Map Tool > Elevation Graph's form, `ElevationProfile`, while it is showing.
    pub elevation: Option<elevation::ElevationProfile>,
    /// What a test puts in the geocoder's place, so Zoom To runs its whole course offline.
    #[cfg(test)]
    fake_geocoder: Option<GeocoderFetch>,
    /// Yes to KML Overlay's "Zoom to the center or the loaded file?", until the screen zooms.
    zoom_to_kml: bool,
    /// Create Circle Survey's answers so far, as typed (or as offered, when a box was cancelled).
    survey_answers: Vec<String>,
    /// Create Circle Survey's centre once its sixth box was cancelled: the screen finishes it,
    /// having the menu context the cancel does not.
    survey_pending: Option<LatLon>,
    /// Enter UTM Coord's `zone`, a static the C# keeps from one use to the next: "50s" to start.
    utm_zone: Option<String>,
    /// Enter UTM Coord's easting, between its box and the northing's.
    utm_easting: String,
    /// What POI > Add or Edit wants done, until the screen takes it.
    poi_request: Option<PoiRequest>,
    /// The polygon icon's menu, where the button came up on the icon.
    pub poly_menu: Option<(f32, f32)>,
    /// Text's string and size, between their boxes and the rotation's.
    text_answers: (String, String),
    /// Text's start once its rotation box was cancelled: the screen finishes it.
    text_pending: Option<LatLon>,
    /// A Write or Write Fast on its way through its questions.
    pub write_flow: Option<WriteFlow>,
    write_answer: Option<WriteAnswer>,
    /// `TilePrefetcherMenu`, while it shows.
    pub prefetch_menu: Option<crate::prefetch_ui::PrefetchMenu>,
    /// `TilePrefetcher`, while it runs or until its form is closed.
    pub prefetch: Option<crate::prefetch_ui::PrefetchJob>,
    prefetch_area_wanted: bool,
    prefetch_path_zoom: Option<i32>,
    /// An `InputBox`'s caption, question and text as its OK closed it, until the screen keeps
    /// the answer in `Settings.Instance`.
    answered: Option<(&'static str, String, String)>,
    // ---- row 96 ----
    /// `Host.FPMenuMap.Items` a plugin added, drawn after the menu's own entries.
    pub plugin_entries: Vec<crate::plugins_ui::Entry>,
    /// A plugin's entry chosen - the plugin, its id and `FPMenuMapPosition` - until the
    /// plugins' tick passes it on.
    pub plugin_click: Option<(usize, u32, LatLon)>,
    // ---- end row 96 ----
}

/// How a page is fetched from the geocoder: its URL in, its text or why not out.
type GeocoderFetch = fn(&str) -> Result<String, String>;

/// `GetContentUsingHttp` for the geocoder: a GET with the application's User-Agent, `Accept: */*`
/// and OpenStreetMap's `Referer`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:443-461`
fn geocoder_fetch(url: &str) -> Result<String, String> {
    mp_tiles::TileFetcher::new()
        .fetch_text(url, mapview::GEOCODER_REFERER)
        .map_err(|error| error.to_string())
}

/// A Map Tool > Zoom To search: the place asked for, and where the geocoder's answer arrives.
///
/// The C# asks on the interface thread and waits for the answer; here the request goes on a
/// thread of its own and the answer is taken up by the frame that finds it, so the screen does
/// not stop while a server is slow.
#[derive(Debug)]
struct Geocoding {
    place: String,
    answer: std::sync::mpsc::Receiver<(mapview::GeocoderStatus, Option<LatLon>)>,
}

impl PlanMenus {
    /// Opens the menu where the right button came up. `contextMenuStrip1_Opening` runs here: the
    /// marker under the cursor decides Delete WP, and a vehicle with `MISSION_FENCE` hides
    /// Geo-Fence and Rally Points ([`PlanMenus::open_for`]).
    pub fn open_at(&mut self, at: (f32, f32), position: LatLon, marker: Option<u16>) {
        self.zoom_menu = None;
        self.open = Some(OpenMenu {
            at,
            position,
            marker,
            submenu: None,
            mission_fence: false,
        });
        self.dismissed_at = None;
    }

    /// [`PlanMenus::open_at`] over a vehicle whose `cs.capabilities` are `capabilities`: 0 with
    /// nothing connected, as `MainV2.comPort.MAV` always exists in the C# and reads 0 then.
    /// `// C#: GCSViews/FlightPlanner.cs:2667-2695`
    pub fn open_for(
        &mut self,
        at: (f32, f32),
        position: LatLon,
        marker: Option<u16>,
        capabilities: u32,
    ) {
        self.open_at(at, position, marker);
        if let Some(menu) = self.open.as_mut() {
            menu.mission_fence = has_mission_fence(capabilities);
        }
    }

    /// Closes the menu because a press landed somewhere else.
    pub fn dismiss(&mut self, at: (f32, f32)) {
        if self.open.take().is_some() {
            self.dismissed_at = Some(at);
        }
    }

    /// Whether a press on the map is the one that just closed the menu, which it then consumes.
    pub fn swallows_press(&mut self, at: (f32, f32)) -> bool {
        self.dismissed_at.take() == Some(at)
    }

    /// Whether Delete WP is enabled: only over a marker.
    #[must_use]
    pub fn delete_enabled(&self) -> bool {
        self.open.is_some_and(|menu| menu.marker.is_some())
    }

    /// Shows a top-level entry's drop-down, or none.
    pub fn show_submenu(&mut self, index: Option<usize>) {
        if let Some(menu) = self.open.as_mut() {
            menu.submenu = index;
        }
    }

    /// Load File's `OpenFileDialog`: blank, in the folder the dialogs open in.
    /// `// C#: GCSViews/FlightPlanner.cs:1817-1823`
    pub fn ask_mission_load(&mut self) {
        self.open = None;
        self.ask(Prompt::input(
            OPEN_FILE,
            MISSION_LOAD_FILTER,
            "",
            PromptKind::MissionLoadFile,
        ));
    }

    /// Save File's `SaveFileDialog`, on the mission's file name: `fd.FileName = wpfilename`.
    /// `// C#: GCSViews/FlightPlanner.cs:6069-6077`
    pub fn ask_mission_save(&mut self, file_name: &str) {
        self.open = None;
        self.ask(Prompt::input(
            SAVE_FILE,
            MISSION_SAVE_FILTER,
            file_name,
            PromptKind::MissionSaveFile,
        ));
    }

    fn ask(&mut self, prompt: Prompt) {
        self.prompt = Some(prompt);
    }

    /// The dialog's "Show me again?" clicked: the setting's key and its new value, `Checked`,
    /// which the window writes at once.
    /// `// C#: Common.cs:446-449`
    pub fn toggle_show_again(&mut self) -> Option<(&'static str, bool)> {
        let again = self.prompt.as_mut()?.show_again.as_mut()?;
        again.ticked = !again.ticked;
        Some((again.key, again.ticked))
    }

    /// `MessageShowAgain`'s early return: a message whose tick was cleared on an earlier
    /// showing is not shown - the window calls this once a frame with the settings, as the C#
    /// reads `Settings.Instance` before it makes the form. A question is checked at its site.
    /// `// C#: Common.cs:267-270`
    ///
    /// Only a box whose tick is still set: one the user has just unticked has its key off too,
    /// and the C#'s form stays until OK - dropping it under the pointer was a defect found by
    /// the fence scripts on 2026-09-25.
    pub fn drop_suppressed_prompt(&mut self, is_off: impl Fn(&str) -> bool) {
        if let Some(prompt) = &self.prompt
            && prompt.kind == PromptKind::Message
            && prompt
                .show_again
                .is_some_and(|again| again.ticked && is_off(again.key))
        {
            self.prompt = None;
        }
    }

    fn tell(&mut self, title: &'static str, text: impl Into<String>) {
        self.prompt = Some(Prompt::message(title, text));
    }

    /// Inject Custom Map's `FolderBrowserDialog`, as the planner's file dialogs are asked: the
    /// dialog's own caption, no words, the folder typed.
    /// `// C#: GCSViews/FlightPlanner.cs:8428-8434`
    pub fn ask_inject_folder(&mut self) {
        self.ask(Prompt::input(
            crate::inject_map::FOLDER_TITLE,
            "",
            "",
            PromptKind::InjectCustomMapFolder,
        ));
    }

    /// KML Overlay's first question, once its file is on the map.
    /// `// C#: GCSViews/FlightPlanner.cs:4246-4249`
    pub fn ask_kml_to_flight_screen(&mut self) {
        self.ask(Prompt::question(
            LOAD_DATA,
            LOAD_INTO_FLIGHT_SCREEN,
            PromptKind::KmlToFlightScreen,
        ));
    }

    /// KML Overlay's second question, whatever the first was answered.
    /// `// C#: GCSViews/FlightPlanner.cs:4264-4267`
    fn ask_kml_zoom(&mut self) {
        self.ask(Prompt::question(
            ZOOM_TO_TITLE,
            ZOOM_TO_LOADED,
            PromptKind::KmlZoomTo,
        ));
    }

    /// Whether Yes was answered to the zoom question since the screen last looked.
    pub fn take_zoom_to_kml(&mut self) -> bool {
        std::mem::take(&mut self.zoom_to_kml)
    }

    /// What POI > Add or Edit asked for, once.
    /// What the last Write question's button meant, once.
    pub fn take_write_answer(&mut self) -> Option<WriteAnswer> {
        self.write_answer.take()
    }

    /// Whether Yes was said to ripping the view, once.
    pub fn take_prefetch_area_wanted(&mut self) -> bool {
        std::mem::take(&mut self.prefetch_area_wanted)
    }

    /// Prefetch WP Path's zoom, once it has been typed.
    pub fn take_prefetch_path_zoom(&mut self) -> Option<i32> {
        self.prefetch_path_zoom.take()
    }

    pub fn take_poi_request(&mut self) -> Option<PoiRequest> {
        self.poi_request.take()
    }

    /// POI > Edit's "Enter ID" for the point at `index`.
    /// `// C#: Utilities/POI.cs:104-112`
    pub fn ask_poi_edit(&mut self, index: usize) {
        self.ask(Prompt::input(
            crate::poi::ID_TITLE,
            crate::poi::ID_TEXT,
            "",
            PromptKind::PlanPoiEdit { index },
        ));
    }

    /// Enter UTM Coord's zone: what was last typed, "50s" until then.
    /// `// C#: GCSViews/FlightPlanner.cs:96`
    #[must_use]
    pub fn utm_zone(&self) -> &str {
        self.utm_zone.as_deref().unwrap_or("50s")
    }

    /// Opens the polygon icon's menu where the button came up over the icon; the map's menu
    /// closes, as one `ContextMenuStrip` showing hides another.
    /// `// C#: GCSViews/FlightPlanner.cs:7607-7618`
    pub fn open_poly_menu(&mut self, at: (f32, f32)) {
        self.open = None;
        self.zoom_menu = None;
        self.poly_menu = Some(at);
        self.dismissed_at = None;
    }

    /// Closes the polygon icon's menu because a press landed somewhere else.
    pub fn dismiss_poly_menu(&mut self, at: (f32, f32)) {
        if self.poly_menu.take().is_some() {
            self.dismissed_at = Some(at);
        }
    }

    /// The answer Create Circle Survey's box `step` offers, as `InputBox` shows the `ref int`.
    fn prompt_offered_survey(&self, step: u8) -> String {
        let defaults = mp_mission::circle_survey::Answers::default();
        let offered = match step {
            0 => defaults.start_alt,
            1 => defaults.end_alt,
            2 => defaults.separation,
            3 => defaults.radius,
            4 => defaults.photos,
            _ => defaults.start_heading,
        };
        offered.to_string()
    }

    /// Create Circle Survey's box `step`: title "", the prompt's word, the default offered.
    /// `// C#: Utilities/CircleSurveyMission.cs:17-22`
    fn ask_survey_step(&mut self, step: u8, position: LatLon) {
        let offered = self.prompt_offered_survey(step);
        let text = mp_mission::circle_survey::PROMPTS
            .get(usize::from(step))
            .copied()
            .unwrap_or("");
        self.ask(Prompt::input(
            "",
            text,
            offered,
            PromptKind::CircleSurvey { step, position },
        ));
    }

    /// One of the six answers in: the next box, or - after the sixth - `int.Parse` of each and
    /// the rows through `AddCommand`. A word that is not an integer is `int.Parse`'s
    /// `FormatException`, which reaches the application's handler.
    /// `// C#: Utilities/CircleSurveyMission.cs:17-45; ExtLibs/Controls/InputBox.cs:21-27`
    fn survey_answer(
        &mut self,
        plan: &mut Plan,
        step: u8,
        position: LatLon,
        value: String,
        context: &MenuContext,
    ) {
        self.survey_answers.push(value);
        if step < 5 {
            self.ask_survey_step(step + 1, position);
            return;
        }
        self.finish_circle_survey(plan, position, context);
    }

    /// The sixth box cancelled: the centre the screen is to finish the survey at, once.
    pub fn take_survey_pending(&mut self) -> Option<LatLon> {
        self.survey_pending.take()
    }

    /// Text's string or size in: the next box.
    /// `// C#: GCSViews/FlightPlanner.cs:6843-6846`
    fn text_answer(&mut self, step: u8, position: LatLon, value: String) {
        if step == 0 {
            self.text_answers.0 = value;
            self.ask(Prompt::input(
                "Enter size",
                "Enter size",
                "5",
                PromptKind::TextSize { position },
            ));
        } else {
            self.text_answers.1 = value;
            self.ask(Prompt::input(
                "Enter rotation",
                "Enter rotation",
                "0",
                PromptKind::TextRotation { position },
            ));
        }
    }

    /// The rotation box cancelled: the start the screen is to finish the text at, once.
    pub fn take_text_pending(&mut self) -> Option<LatLon> {
        self.text_pending.take()
    }

    /// Text's three answers in: `float.Parse(size) * 1.35f` and `float.Parse(rotation)` - a word
    /// is `FormatException`, which reaches the application's handler - then the string's outline
    /// in the `1CamBam_Stick_3` font (or what fontconfig gives for it), rotated, every point of
    /// the path a waypoint at Default Alt through `AddWPToMap`. A size the `Font` constructor
    /// refuses (0 or less) is "Bad input options, please try again" with the exception.
    /// `// C#: GCSViews/FlightPlanner.cs:6847-6882`
    pub fn finish_text(
        &mut self,
        plan: &mut Plan,
        position: LatLon,
        rotation: &str,
        context: &MenuContext,
    ) {
        let (text, size) = std::mem::take(&mut self.text_answers);
        let Ok(size) = size.trim().parse::<f32>() else {
            self.tell(ERROR, format_exception(&size));
            return;
        };
        let Ok(rotation) = rotation.trim().parse::<f32>() else {
            self.tell(ERROR, format_exception(rotation));
            return;
        };
        let em_size = size * 1.35_f32;
        if em_size <= 0.0 || !em_size.is_finite() {
            self.tell(
                ERROR,
                format!(
                    "Bad input options, please try again\nSystem.ArgumentException: '{em_size}' is not a valid value for 'emSize'. 'emSize' should be greater than 0 and less than or equal to System.Single.MaxValue.\nParameter name: emSize"
                ),
            );
            return;
        }
        // `int.Parse(TXT_DefaultAlt.Text)`, outside the handler's own catch.
        let default_alt = plan.panel_text(PanelBox::DefaultAlt).trim().to_owned();
        let Ok(default_alt) = default_alt.parse::<i32>() else {
            self.tell(ERROR, format_exception(&default_alt));
            return;
        };
        let outline = crate::glyph_text::font_file()
            .and_then(|path| {
                std::fs::read(&path)
                    .map_err(|err| crate::glyph_text::FontError::NoFont(err.to_string()))
            })
            .and_then(|bytes| crate::glyph_text::outline(&bytes, &text, f64::from(em_size)));
        let segments = match outline {
            Ok(segments) => segments,
            Err(why) => {
                self.tell(ERROR, why.to_string());
                return;
            }
        };
        let points = mp_mission::text_mission::path_points(&segments);
        for (lat, lng) in mp_mission::text_mission::place(
            &points,
            f64::from(rotation),
            position.latitude(),
            position.longitude(),
        ) {
            let Ok(at) = LatLon::new(lat, lng) else {
                continue;
            };
            // `AddWPToMap(lat, lng, int.Parse(TXT_DefaultAlt.Text))`, `quickadd` on.
            match row_altitude(plan, at, f64::from(default_alt), context) {
                Ok(altitude) => plan.add_wp_to_map(at, altitude, context.frame),
                Err(why) => {
                    self.tell("", why);
                    return;
                }
            }
        }
    }

    /// The six answers in: `int.Parse` of each and the rows through `AddCommand`. A word that is
    /// not an integer is `int.Parse`'s `FormatException`, which reaches the application's handler.
    /// `// C#: Utilities/CircleSurveyMission.cs:17-45; ExtLibs/Controls/InputBox.cs:21-27`
    pub fn finish_circle_survey(
        &mut self,
        plan: &mut Plan,
        position: LatLon,
        context: &MenuContext,
    ) {
        let answers = std::mem::take(&mut self.survey_answers);
        let mut parsed = [0_i32; 6];
        for (slot, text) in parsed.iter_mut().zip(&answers) {
            match text.trim().parse::<i32>() {
                Ok(number) => *slot = number,
                Err(_) => {
                    self.tell(ERROR, format_exception(text));
                    return;
                }
            }
        }
        let answers = mp_mission::circle_survey::Answers {
            start_alt: parsed[0],
            end_alt: parsed[1],
            separation: parsed[2],
            radius: parsed[3],
            photos: parsed[4],
            start_heading: parsed[5],
        };
        // `MouseDownEnd` is a `PointLatLng`: its `PointLatLngAlt` has altitude 0.
        match mp_mission::circle_survey::create_grid(
            position.latitude(),
            position.longitude(),
            0.0,
            answers,
        ) {
            Ok(rows) => {
                for row in rows {
                    if let Err(why) =
                        plan.add_command(row.command, row.params, row.x, row.y, row.z, context)
                    {
                        self.tell("", why);
                        return;
                    }
                }
            }
            Err(why) => self.tell(ERROR, why.to_string()),
        }
    }

    /// Enter UTM Coord's northing in: the zone's digits (`s` and `n` blanked, `int.Parse`), the
    /// easting and northing (`double.Parse`), GeoUtility's UTM to WGS 84 - always the southern
    /// hemisphere, as `zone.ToLower().Contains("N")` is never true - and a row through
    /// `setfromMap(lat, lng, 0)`. A number that does not parse is the exception, which reaches
    /// the application's handler.
    /// `// C#: GCSViews/FlightPlanner.cs:3269-3285`
    fn enter_utm(&mut self, plan: &mut Plan, northing: &str, context: &MenuContext) {
        let zone_text = self.utm_zone().to_lowercase().replace(['s', 'n'], " ");
        let Ok(zone) = zone_text.trim().parse::<i32>() else {
            self.tell(ERROR, format_exception(&zone_text));
            return;
        };
        let easting = std::mem::take(&mut self.utm_easting);
        let Ok(east) = easting.trim().parse::<f64>() else {
            self.tell(ERROR, format_exception(&easting));
            return;
        };
        let Ok(north) = northing.trim().parse::<f64>() else {
            self.tell(ERROR, format_exception(northing));
            return;
        };
        let (lat, lng) = mp_mission::geoutility::utm_to_wgs84(zone, true, east, north);
        let Ok(position) = LatLon::new(lat, lng) else {
            self.tell(
                ERROR,
                "System.ArgumentOutOfRangeException: the UTM coordinate is not a position",
            );
            return;
        };
        if let Err(why) = plan.add_row_from_map(position, 0, context) {
            self.tell("", why);
        }
    }

    /// A `CustomMessageBox.Show(text, title)` from the screen rather than the menu: a write the
    /// Home Location boxes refused, the Home Location link without a position.
    pub fn say(&mut self, title: &'static str, text: impl Into<String>) {
        self.tell(title, text);
    }

    /// `TXT_homelat_Enter`'s `CustomMessageBox.Show(text)`, which gives the Lat box the keyboard
    /// back when it closes.
    /// `// C#: GCSViews/FlightPlanner.cs:7016-7022`
    pub fn say_home_hint(&mut self, text: &'static str) {
        self.prompt = Some(Prompt {
            title: "",
            text: text.to_owned(),
            field: None,
            kind: PromptKind::HomeLatEnter,
            show_again: None,
        });
    }

    /// Asks "Reset Home to loaded coords", as `processToScreen` does when a mission it has just
    /// read carries a home at item 0 that the boxes do not hold. Yes puts it in the boxes; No
    /// leaves them.
    /// `// C#: GCSViews/FlightPlanner.cs:5645-5658`
    pub fn offer_home_reset(&mut self, loaded: Home) {
        self.ask(Prompt {
            title: "Reset Home Coords",
            text: "Reset Home to loaded coords".to_owned(),
            field: None,
            kind: PromptKind::ResetHome(loaded),
            show_again: None,
        });
    }

    /// Opens the zoom icon's menu where the button came up over the icon.
    /// `// C#: GCSViews/FlightPlanner.cs:7624-7628`
    pub fn open_zoom_menu(&mut self, at: (f32, f32)) {
        self.open = None;
        self.zoom_menu = Some(at);
        self.dismissed_at = None;
    }

    /// Closes the zoom icon's menu because a press landed somewhere else.
    pub fn dismiss_zoom_menu(&mut self, at: (f32, f32)) {
        if self.zoom_menu.take().is_some() {
            self.dismissed_at = Some(at);
        }
    }

    /// Sends Zoom To's place to the geocoder, on a thread of its own.
    fn start_geocode(&mut self, place: String) {
        #[cfg(test)]
        let fetch: GeocoderFetch = self.fake_geocoder.unwrap_or(geocoder_fetch);
        #[cfg(not(test))]
        let fetch: GeocoderFetch = geocoder_fetch;
        let (send, answer) = std::sync::mpsc::channel();
        let keywords = place.clone();
        let spawned = wasm_thread::Builder::new()
            .name("geocoder".to_owned())
            .spawn(move || {
                let outcome = mapview::geocode(&keywords, fetch);
                // The screen may have gone; nobody is waiting then.
                let _ = send.send(outcome);
            });
        match spawned {
            Ok(_) => self.geocoding = Some(Geocoding { place, answer }),
            Err(_) => self.tell(
                "GMap.NET",
                zoom_to_message(&place, mapview::GeocoderStatus::ExceptionInCode),
            ),
        }
    }

    /// Whether a Zoom To search is waiting on the geocoder.
    #[must_use]
    pub const fn geocoding(&self) -> bool {
        self.geocoding.is_some()
    }

    /// The geocoder's answer, once it has come: the place asked for and what was found.
    fn take_geocode(&mut self) -> Option<(String, (mapview::GeocoderStatus, Option<LatLon>))> {
        let pending = self.geocoding.as_ref()?;
        let outcome = match pending.answer.try_recv() {
            Ok(outcome) => outcome,
            Err(std::sync::mpsc::TryRecvError::Empty) => return None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                (mapview::GeocoderStatus::ExceptionInCode, None)
            }
        };
        let place = self.geocoding.take()?.place;
        Some((place, outcome))
    }

    /// Chooses an entry: what its handler does, up to the first `InputBox` it shows.
    ///
    /// Choosing closes the menu, as clicking a `ToolStripMenuItem` does. Load and Save are the
    /// caller's, because they are the screen's file handling rather than an edit.
    pub fn choose(&mut self, plan: &mut Plan, action: MenuAction, context: &MenuContext) {
        let Some(menu) = self.open.take() else {
            return;
        };
        let position = menu.position;
        let frame = context.frame.mav_frame();
        match action {
            MenuAction::DeleteWp => {
                if let Some(seq) = menu.marker {
                    plan.delete_marker(seq);
                }
            }
            MenuAction::InsertWp | MenuAction::InsertSplineWp => self.ask(Prompt::input(
                "Insert WP",
                "Insert WP after wp#",
                plan.insert_offer().to_string(),
                PromptKind::InsertWp {
                    spline: action == MenuAction::InsertSplineWp,
                    position,
                },
            )),
            MenuAction::InsertAtCurrentPosition => {
                // AddWPToMap(cs.lat, cs.lng, (int) cs.alt): an altitude of 0 takes the default.
                // With no position the C# would place it at 0,0; that is refused here in the
                // words zoomToVehicle uses for the same condition.
                // `// C#: GCSViews/FlightPlanner.cs:3051-3054, 1195-1197`
                let Some((at, vehicle_altitude)) = context.vehicle else {
                    self.tell(ERROR, "Invalid Location");
                    return;
                };
                // Drawing a polygon, AddWPToMap adds a corner at MouseDownStart, the menu's
                // position, not the vehicle's; with `sethome` set, home goes to the vehicle.
                if plan.draw_mode() == DrawMode::Area {
                    plan.add_area_vertex(position);
                    return;
                }
                if plan.click_sets_home(at) {
                    return;
                }
                match row_altitude(plan, at, vehicle_altitude.trunc(), context) {
                    Ok(altitude) => plan.add_wp_to_map(at, altitude, context.frame),
                    Err(why) => self.tell("", why),
                }
            }
            MenuAction::LoiterForever => match row_altitude(plan, position, 0.0, context) {
                Ok(altitude) => plan.append(mp_mission::commands::loiter_unlimited(
                    position, altitude, frame,
                )),
                Err(why) => self.tell("", why),
            },
            MenuAction::LoiterTime => self.ask(Prompt::input(
                "Loiter Time",
                "Loiter Time",
                "5",
                PromptKind::LoiterTime { position },
            )),
            MenuAction::LoiterCircles => self.ask(Prompt::input(
                "Loiter Turns",
                "Loiter Turns",
                "3",
                PromptKind::LoiterTurns { position },
            )),
            MenuAction::JumpStart => self.ask(Prompt::input(
                "Jump repeat",
                "Number of times to Repeat",
                "5",
                PromptKind::JumpStartRepeat,
            )),
            MenuAction::JumpWp => self.ask(Prompt::input(
                "WP No",
                "Jump to WP no?",
                "1",
                PromptKind::JumpWpNumber,
            )),
            MenuAction::Rtl => plan.append(mp_mission::commands::return_to_launch(frame)),
            // `setfromMap(MouseDownEnd.Lat, MouseDownEnd.Lng, 1)`: 1, unless Default Alt says
            // otherwise or Verify Height takes it from the ground.
            // `// C#: GCSViews/FlightPlanner.cs:4302-4316`
            MenuAction::Land => {
                match row_altitude(plan, position, mp_mission::commands::LAND_ALTITUDE, context) {
                    Ok(altitude) => {
                        let mut land = mp_mission::commands::land(position, frame);
                        land.z = altitude;
                        plan.append(land);
                    }
                    Err(why) => self.tell("", why),
                }
            }
            // `CurrentState.AltUnit == "m" ? "10" : "30"`; this application is metric.
            MenuAction::Takeoff => self.ask(Prompt::input(
                "Altitude",
                "Please enter your takeoff altitude in m",
                "10",
                PromptKind::TakeoffAltitude {
                    ask_pitch: context.takeoff_pitch,
                },
            )),
            // `cmdParamNames.ContainsKey("DO_SET_ROI")` is true in all three of mavcmd.xml's
            // vehicle sections, so the C#'s "not enabled in your firmware" never shows.
            MenuAction::SetRoi => match row_altitude(plan, position, 0.0, context) {
                Ok(altitude) => {
                    plan.append(mp_mission::commands::set_roi(position, altitude, frame))
                }
                Err(why) => self.tell("", why),
            },
            MenuAction::ClearMission => plan.clear_mission(),
            MenuAction::DrawPolygon => plan.draw_polygon(position),
            MenuAction::ClearPolygon => plan.clear_polygon(),
            MenuAction::PolygonFromWaypoints => {
                if plan.polygon_from_waypoints() {
                    self.ask(Prompt {
                        title: "Confirm",
                        text: "Clear current waypoints?".to_owned(),
                        field: None,
                        kind: PromptKind::ClearWaypoints,
                        show_again: None,
                    });
                }
            }
            // The C# also drops a red marker at each end; the map here has no overlay for them.
            MenuAction::MeasureDistance => match self.measure_from.take() {
                None => {
                    self.measure_from = Some(position);
                    // `MessageShowAgain("Measure Dist", ...)`, its tick kept as
                    // `SHOWAGAIN_Measure_Dist`. `// C#: GCSViews/FlightPlanner.cs:2632-2633`
                    self.ask(
                        Prompt::message(
                            "Measure Dist",
                            "You can now pan/zoom around.\nClick this option again to get the distance.",
                        )
                        .with_show_again(MEASURE_DIST_KEY),
                    );
                }
                Some(from) => self.tell("", measure_text(from, position)),
            },
            // `InputBox.Show("Location", "Enter your location", ref place)`, the place given as
            // Perth Airport.
            // `// C#: GCSViews/FlightPlanner.cs:8345-8348`
            MenuAction::ZoomTo => self.ask(Prompt::input(
                "Location",
                "Enter your location",
                "Perth Airport, Australia",
                PromptKind::ZoomTo,
            )),
            // The zoom icon's menu is the screen's: it moves the map.
            MenuAction::ZoomToVehicle | MenuAction::ZoomToMission | MenuAction::ZoomToHome => {}
            // Handled by the application before this is reached, as the zoom entries are.
            MenuAction::Prefetch | MenuAction::PrefetchWpPath | MenuAction::SwitchDocking => {}
            MenuAction::ReverseWps => plan.reverse_waypoints(),
            MenuAction::ModifyAlt => self.ask(Prompt::input(
                "Alt Change",
                "Please enter the alitude change you require.\n(20 = up 20, *2 = up by alt * 2)",
                "0",
                PromptKind::ModifyAlt,
            )),
            MenuAction::SetReturnLocation => plan.set_fence_return(position),
            // `OpenFileDialog`, filtered to `.fen`.
            // `// C#: GCSViews/FlightPlanner.cs:4347-4351`
            MenuAction::FenceLoadFromFile => self.ask(Prompt::input(
                OPEN_FILE,
                FENCE_FILTER,
                "",
                PromptKind::FenceLoadFile,
            )),
            // "Please set a return location" first, then `SaveFileDialog`, filtered to `.fen`.
            // `// C#: GCSViews/FlightPlanner.cs:5961-5970`
            MenuAction::FenceSaveToFile => {
                if plan.fence_return().is_none() {
                    self.tell("", SET_RETURN_LOCATION);
                } else {
                    self.ask(Prompt::input(
                        SAVE_FILE,
                        FENCE_FILTER,
                        "",
                        PromptKind::FenceSaveFile,
                    ));
                }
            }
            // Nothing drawn, nothing saved: the C# returns before its dialog.
            // `// C#: GCSViews/FlightPlanner.cs:5894-5897, 5899-5903`
            MenuAction::SavePolygon => {
                if !plan.polygon().is_empty() {
                    self.ask(Prompt::input(
                        SAVE_FILE,
                        POLYGON_FILTER,
                        "",
                        PromptKind::PolygonSaveFile,
                    ));
                }
            }
            // `// C#: GCSViews/FlightPlanner.cs:4530-4533`
            MenuAction::LoadPolygon => self.ask(Prompt::input(
                OPEN_FILE,
                POLYGON_FILTER,
                "",
                PromptKind::PolygonLoadFile,
            )),
            // `// C#: GCSViews/FlightPlanner.cs:3535-3538`
            MenuAction::FromShp => self.ask(Prompt::input(
                OPEN_FILE,
                SHP_FILTER,
                "",
                PromptKind::ShpLoadFile,
            )),
            // `// C#: GCSViews/FlightPlanner.cs:4332-4336`
            MenuAction::LoadAndAppend => self.ask(Prompt::input(
                OPEN_FILE,
                MISSION_FILTER,
                "",
                PromptKind::AppendLoadFile,
            )),
            // `// C#: GCSViews/FlightPlanner.cs:4472-4476`
            MenuAction::LoadKmlFile => self.ask(Prompt::input(
                OPEN_FILE,
                KML_FILTER,
                "",
                PromptKind::KmlLoadFile,
            )),
            // `// C#: GCSViews/FlightPlanner.cs:4720-4724`
            MenuAction::LoadShpFile => self.ask(Prompt::input(
                OPEN_FILE,
                SHP_FILTER,
                "",
                PromptKind::ShpMissionLoadFile,
            )),
            // `// C#: GCSViews/FlightPlanner.cs:4133-4138`
            MenuAction::KmlOverlay => self.ask(Prompt::input(
                OPEN_FILE,
                KML_OVERLAY_FILTER,
                "",
                PromptKind::KmlOverlayFile,
            )),
            // `CircleSurveyMission.createGrid(MouseDownEnd)`: six boxes, then the rows.
            // `// C#: GCSViews/FlightPlanner.cs:2852-2855; Utilities/CircleSurveyMission.cs:10-45`
            MenuAction::CreateCircleSurvey => {
                self.survey_answers.clear();
                self.ask_survey_step(0, position);
            }
            // `// C#: GCSViews/FlightPlanner.cs:3258-3264`
            MenuAction::EnterUtmCoord => {
                let zone = self.utm_zone().to_owned();
                self.ask(Prompt::input(
                    "Zone",
                    "Enter Zone. (eg 50S, 11N)",
                    zone,
                    PromptKind::UtmZone,
                ));
            }
            // `// C#: GCSViews/FlightPlanner.cs:6970-6982`
            MenuAction::TrackerHome => self.ask(Prompt::input(
                "Tracker Alt",
                "Enter tracker ASL alt",
                double_text(context.tracker_alt),
                PromptKind::TrackerAlt { position },
            )),
            // `InputBox.Show("Enter String", "Enter String (requires 1CamBam_Stick_3 font)", ...)`
            // `// C#: GCSViews/FlightPlanner.cs:6839-6842`
            MenuAction::Text => self.ask(Prompt::input(
                "Enter String",
                "Enter String (requires 1CamBam_Stick_3 font)",
                "",
                PromptKind::TextString { position },
            )),
            // `POI.POIAdd(MouseDownStart)`: "Enter ID", then the point.
            // `// C#: GCSViews/FlightPlanner.cs:5009-5012; Utilities/POI.cs:72-85`
            MenuAction::PoiAdd => self.ask(Prompt::input(
                crate::poi::ID_TITLE,
                crate::poi::ID_TEXT,
                "",
                PromptKind::PlanPoiId { position },
            )),
            // `// C#: GCSViews/FlightPlanner.cs:3641-3647`
            MenuAction::OffsetPolygon => {
                if !plan.polygon().is_empty() {
                    self.ask(Prompt::input(
                        "Offset in Meters",
                        "Please enter the offset in meters. Enter a negative value to make the polygon smaller",
                        "0",
                        PromptKind::OffsetPolygon,
                    ));
                }
            }
            // `// C#: GCSViews/FlightPlanner.cs:1760-1773, 1992-1996`
            MenuAction::Area => match plan.polygon_area() {
                Some(aream2) => self.tell("Area", area_text(aream2)),
                None => self.ask(Prompt {
                    title: "",
                    text: DEFINE_POLYGON.to_owned(),
                    field: None,
                    kind: PromptKind::DefinePolygon,
                    show_again: None,
                }),
            },
            // `InputBox.Show("Altitude", "Altitude", ref altstring)`, offering Default Alt.
            // `// C#: GCSViews/FlightPlanner.cs:6637-6640`
            MenuAction::SetRallyPoint => self.ask(Prompt::input(
                "Altitude",
                "Altitude",
                plan.panel_text(PanelBox::DefaultAlt),
                PromptKind::RallyAltitude { position },
            )),
            // `// C#: GCSViews/FlightPlanner.cs:6022-6039`
            MenuAction::SaveRallyToFile => {
                if plan.rally().is_empty() {
                    self.tell("", SET_SOME_RALLY_POINTS);
                } else {
                    self.ask(Prompt::input(
                        SAVE_FILE,
                        RALLY_FILTER,
                        "",
                        PromptKind::RallySaveFile,
                    ));
                }
            }
            // `// C#: GCSViews/FlightPlanner.cs:4416-4420`
            MenuAction::LoadRallyFromFile => self.ask(Prompt::input(
                OPEN_FILE,
                RALLY_FILTER,
                "",
                PromptKind::RallyLoadFile,
            )),
            MenuAction::CreateWpCircle | MenuAction::CreateSplineCircle => {
                self.circle_answers.clear();
                let spline = action == MenuAction::CreateSplineCircle;
                self.ask_circle(spline, position);
            }
            // `writeKML()` for a fresh `pointlist`, then `new ElevationProfile(pointlist,
            // cs.HomeAlt, CMB_altmode)` shown as a dialog - or, planned too little, its "Please
            // plan something first" and no form. (`cs.HomeAlt` is kept by the form and never
            // read, so it is not passed here.)
            // `// C#: GCSViews/FlightPlanner.cs:3248-3256; Controls/ElevationProfile.cs:27-48, 78-84`
            MenuAction::ElevationGraph => {
                let terrain = plan.terrain();
                let pointlist = mapview::point_list(plan.home(), plan.items(), &|lat, lng| {
                    terrain.at(lat, lng)
                });
                match elevation::ElevationProfile::new(pointlist, context.frame, terrain) {
                    Ok(profile) => self.elevation = Some(profile),
                    Err(text) => self.tell(ERROR, text),
                }
            }
            MenuAction::SetHomeHere => plan.set_home_at(position),
            MenuAction::PoiDelete
            | MenuAction::PoiEdit
            | MenuAction::FenceInclusion
            | MenuAction::FenceExclusion
            | MenuAction::LoadWpFile
            | MenuAction::SaveWpFile
            | MenuAction::FenceClear
            | MenuAction::GeoFenceUpload
            | MenuAction::GeoFenceDownload
            | MenuAction::SurveyGrid
            | MenuAction::GetRallyPoints
            | MenuAction::SaveRallyPoints
            | MenuAction::ClearRallyPoints => {}
        }
    }

    /// The next of a circle's questions, by how many have been answered, or `false` once all have.
    /// Create Wp Circle asks four and Create Spline Circle five, each offering the C#'s value.
    /// `// C#: GCSViews/FlightPlanner.cs:2965-2979 (Wp), 2859-2877 (Spline)`
    fn ask_circle(&mut self, spline: bool, position: LatLon) -> bool {
        const WP: [(&str, &str, &str); 4] = [
            ("Radius", "Radius", "50"),
            ("Points", "Number of points to generate Circle", "20"),
            ("Points", "Direction of circle (-1 or 1)", "1"),
            ("angle", "Angle of first point (whole degrees)", "0"),
        ];
        const SPLINE: [(&str, &str, &str); 5] = [
            ("Radius", "Radius", "50"),
            ("min alt", "Min Alt", "5"),
            ("max alt", "Max Alt", "20"),
            ("alt step", "alt step", "5"),
            ("angle", "Angle of first point (whole degrees)", "0"),
        ];
        let questions: &[(&'static str, &str, &str)] = if spline { &SPLINE } else { &WP };
        let Some(&(title, text, value)) = questions.get(self.circle_answers.len()) else {
            return false;
        };
        self.ask(Prompt::input(
            title,
            text,
            value,
            PromptKind::Circle { spline, position },
        ));
        true
    }

    /// A circle's answers read, in the C#'s order, and its rows added - or the first refusal said.
    fn make_circle(
        &mut self,
        plan: &mut Plan,
        context: &MenuContext,
        spline: bool,
        position: LatLon,
    ) {
        let answers = std::mem::take(&mut self.circle_answers);
        let int = |index: usize| {
            answers
                .get(index)
                .and_then(|text| mp_mission::dotnet::parse_i32(text))
        };
        let outcome = if spline {
            // `int.TryParse` of each, the C#'s message for the first that fails. The angle is
            // asked and never read.
            // `// C#: GCSViews/FlightPlanner.cs:2885-2907`
            match (int(0), int(1), int(2), int(3)) {
                (None, ..) => Err("Bad Radius"),
                (_, None, ..) => Err("Bad min alt"),
                (_, _, None, _) => Err("Bad maxalt"),
                (_, _, _, None) => Err("Bad alt step"),
                (Some(radius), Some(min_alt), Some(max_alt), Some(alt_step)) => {
                    plan.spline_circle(position, radius, min_alt, max_alt, alt_step, context)
                }
            }
        } else {
            // `// C#: GCSViews/FlightPlanner.cs:2986-3013`
            match (int(0), int(1), int(2), int(3)) {
                (None, ..) => Err("Bad Radius"),
                (_, None, ..) => Err("Bad Point value"),
                (_, _, None, _) => Err("Bad Direction value"),
                (_, _, _, None) => Err("Bad start angle value"),
                (Some(radius), Some(points), Some(direction), Some(start)) => {
                    plan.wp_circle(position, radius, points, direction, start, context)
                }
            }
        };
        if let Err(why) = outcome {
            let title = if why == FORMAT_EXCEPTION { ERROR } else { "" };
            self.tell(title, why);
        }
    }

    /// An `InputBox`'s OK, once: its caption, question and text, for the screen to keep as
    /// `InputBox` keeps every titled answer. A file dialog's stand-in and a question keep nothing.
    /// `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    pub fn take_answered(&mut self) -> Option<(&'static str, String, String)> {
        self.answered.take()
    }

    /// OK, or Yes: the rest of the handler, with what was typed.
    ///
    /// A value the C# would refuse is refused with its message. The loiter and jump handlers put
    /// the typed text in the grid unchecked and fail later, at "Invalid number on row" when the
    /// mission is written; an item here holds a number, so that text is refused as it is typed,
    /// with `Strings.InvalidNumberEntered`.
    pub fn submit(&mut self, plan: &mut Plan, context: &MenuContext) -> Option<FileRequest> {
        let prompt = self.prompt.take()?;
        let value = prompt.value().to_owned();
        // `InputBox.Show`'s OK keeps the answer before the handler reads it.
        // `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184`
        if prompt.field.is_some() && !prompt.is_file_dialog() {
            self.answered = Some((prompt.title, prompt.text.clone(), value.clone()));
        }
        let frame = context.frame.mav_frame();
        // Default Alt, read once the answer is in, as the handlers read it after `InputBox`.
        let altitude = match prompt.kind {
            PromptKind::InsertWp { position, .. }
            | PromptKind::LoiterTime { position }
            | PromptKind::LoiterTurns { position } => {
                match row_altitude(plan, position, 0.0, context) {
                    Ok(altitude) => altitude,
                    Err(why) => {
                        self.tell("", why);
                        return None;
                    }
                }
            }
            _ => 0.0,
        };
        let number = || {
            value
                .trim()
                .parse::<f64>()
                .ok()
                .filter(|number| number.is_finite())
        };
        // `int.TryParse`, for the takeoff's two numbers.
        let integer = || value.trim().parse::<i32>().ok().map(f64::from);
        match prompt.kind {
            PromptKind::InsertWp { spline, position } => {
                let item = if spline {
                    mp_mission::commands::spline_waypoint(position, altitude, frame)
                } else {
                    mp_mission::commands::waypoint(position, altitude, frame)
                };
                if plan.insert_after(&value, item).is_err() {
                    // `// C#: GCSViews/FlightPlanner.cs:4046, 4081`
                    self.tell(
                        ERROR,
                        if spline {
                            INVALID_NUMBER
                        } else {
                            "Invalid insert position"
                        },
                    );
                }
            }
            PromptKind::LoiterTime { position } => match number() {
                Some(seconds) => plan.append(mp_mission::commands::loiter_time(
                    position, altitude, frame, seconds,
                )),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::LoiterTurns { position } => match number() {
                Some(turns) => plan.append(mp_mission::commands::loiter_turns(
                    position, altitude, frame, turns,
                )),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            // Jump > Start jumps to item 1: `Param1.Value = 1`.
            PromptKind::JumpStartRepeat => match number() {
                Some(repeat) => plan.append(mp_mission::commands::do_jump(1.0, repeat, frame)),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::JumpWpNumber => match number() {
                Some(target) => self.ask(Prompt::input(
                    "Jump repeat",
                    "Number of times to Repeat",
                    "5",
                    PromptKind::JumpWpRepeat { target },
                )),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::JumpWpRepeat { target } => match number() {
                Some(repeat) => plan.append(mp_mission::commands::do_jump(target, repeat, frame)),
                None => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::TakeoffAltitude { ask_pitch } => match integer() {
                Some(altitude) if ask_pitch => self.ask(Prompt::input(
                    "Takeoff Pitch",
                    "Please enter your takeoff pitch",
                    "15",
                    PromptKind::TakeoffPitch { altitude },
                )),
                Some(altitude) => {
                    plan.append(mp_mission::commands::takeoff(altitude, 0.0, frame));
                }
                // `MessageBox.Show("Bad Alt")`. `// C#: GCSViews/FlightPlanner.cs:6792`
                None => self.tell("", "Bad Alt"),
            },
            PromptKind::TakeoffPitch { altitude } => match integer() {
                Some(pitch) => plan.append(mp_mission::commands::takeoff(altitude, pitch, frame)),
                // `// C#: GCSViews/FlightPlanner.cs:6818`
                None => self.tell("", "Bad Takeoff pitch"),
            },
            PromptKind::ModifyAlt => {
                if plan.modify_altitudes(&value).is_err() {
                    self.tell(ERROR, INVALID_NUMBER);
                }
            }
            PromptKind::ClearWaypoints => plan.clear_mission(),
            PromptKind::WriteAltMode | PromptKind::WriteZeroAlt => {
                self.write_answer = Some(WriteAnswer::Continue);
            }
            // Yes: `area = MainMap.ViewArea`, and the menu.
            PromptKind::PrefetchRipArea => self.prefetch_area_wanted = true,
            // `int.TryParse(maxzoomstring)`, else "Invalid number entered".
            // `// C#: GCSViews/FlightPlanner.cs:3309-3316`
            PromptKind::PrefetchMaxZoom => match value.trim().parse::<i32>() {
                Ok(zoom) => self.prefetch_path_zoom = Some(zoom),
                Err(_) => self.tell(ERROR, INVALID_NUMBER),
            },
            PromptKind::ResetHome(loaded) => plan.reset_home_to(loaded),
            PromptKind::FenceLoadFile => return Some(FileRequest::LoadFence(value)),
            PromptKind::FenceSaveFile => return Some(FileRequest::SaveFence(value)),
            PromptKind::ZoomTo => self.start_geocode(value),
            PromptKind::PolygonSaveFile => return Some(FileRequest::SavePolygon(value)),
            PromptKind::PolygonLoadFile => return Some(FileRequest::LoadPolygon(value)),
            PromptKind::ShpLoadFile => return Some(FileRequest::LoadShp(value)),
            PromptKind::RallySaveFile => return Some(FileRequest::SaveRally(value)),
            PromptKind::RallyLoadFile => return Some(FileRequest::LoadRally(value)),
            PromptKind::MissionLoadFile => return Some(FileRequest::LoadMission(value)),
            PromptKind::MissionSaveFile => return Some(FileRequest::SaveMission(value)),
            PromptKind::AppendLoadFile => return Some(FileRequest::LoadAndAppend(value)),
            PromptKind::KmlLoadFile => return Some(FileRequest::LoadKml(value)),
            PromptKind::ShpMissionLoadFile => return Some(FileRequest::LoadShpMission(value)),
            PromptKind::KmlOverlayFile => return Some(FileRequest::KmlOverlay(value)),
            PromptKind::InjectCustomMapFolder => {
                return Some(FileRequest::InjectCustomMap(value));
            }
            // Yes: the polygons and routes go onto the flight screen's map as well, then the
            // zoom question. `// C#: GCSViews/FlightPlanner.cs:4246-4262`
            PromptKind::KmlToFlightScreen => {
                plan.show_kml_on_flight_screen();
                self.ask_kml_zoom();
            }
            // Yes: `MainMap.SetZoomToFitRect(GetBoundingLayer(kmlpolygonsoverlay))`, done by the
            // screen once it sees the flag. `// C#: GCSViews/FlightPlanner.cs:4264-4271`
            PromptKind::KmlZoomTo => self.zoom_to_kml = true,
            PromptKind::CircleSurvey { step, position } => {
                self.survey_answer(plan, step, position, value, context);
            }
            // `InputBox.Show("Zone", ..., ref zone)`: the static keeps what was typed.
            PromptKind::UtmZone => {
                self.utm_zone = Some(value);
                self.ask(Prompt::input(
                    "Easting",
                    "Easting",
                    "578994",
                    PromptKind::UtmEasting,
                ));
            }
            PromptKind::UtmEasting => {
                self.utm_easting = value;
                self.ask(Prompt::input(
                    "Northing",
                    "Northing",
                    "6126244",
                    PromptKind::UtmNorthing,
                ));
            }
            PromptKind::UtmNorthing => self.enter_utm(plan, &value, context),
            // `InputBox.Show("Tracker Alt", ..., ref alt)`: `double.Parse` of the answer, then
            // `cs.TrackerLocation = new PointLatLngAlt(MouseDownEnd) { Alt = alt }`.
            // `// C#: GCSViews/FlightPlanner.cs:6975-6981; ExtLibs/Controls/InputBox.cs:29-35`
            PromptKind::TrackerAlt { position } => match value.trim().parse::<f64>() {
                Ok(alt) => mp_vehicle::VehicleState::set_tracker_location(mp_vehicle::LatLngAlt {
                    lat: position.latitude(),
                    lng: position.longitude(),
                    alt,
                }),
                Err(_) => self.tell(ERROR, format_exception(&value)),
            },
            PromptKind::PlanPoiId { position } => {
                self.poi_request = Some(PoiRequest::Add {
                    lat: position.latitude(),
                    lng: position.longitude(),
                    id: value,
                });
            }
            PromptKind::PlanPoiEdit { index } => {
                self.poi_request = Some(PoiRequest::Rename { index, id: value });
            }
            // The three boxes are read whatever their buttons said (`InputBox.Show(..., ref text)`
            // with no result checked), so an answer and a cancel both go on to the next.
            // `// C#: GCSViews/FlightPlanner.cs:6841-6846`
            PromptKind::TextString { position } => self.text_answer(0, position, value),
            PromptKind::TextSize { position } => self.text_answer(1, position, value),
            PromptKind::TextRotation { position } => {
                self.finish_text(plan, position, &value, context);
            }
            // `if (meter != "0") intmeter = double.Parse(meter);` - a FormatException past that,
            // which reaches the application's handler with its message.
            // `// C#: GCSViews/FlightPlanner.cs:3645-3651`
            PromptKind::OffsetPolygon => {
                let metres = if value == "0" {
                    Some(0.0)
                } else {
                    mp_mission::dotnet::parse_f64(&value)
                };
                match metres {
                    Some(metres) => plan.offset_polygon(metres),
                    None => self.tell(ERROR, FORMAT_EXCEPTION),
                }
            }
            PromptKind::DefinePolygon => self.tell("Area", area_text(0.0)),
            // `int.TryParse(altstring, out alt)`, or "Invalid Alt".
            // `// C#: GCSViews/FlightPlanner.cs:6642-6658`
            PromptKind::RallyAltitude { position } => match mp_mission::dotnet::parse_i32(&value) {
                Some(altitude) => plan.add_rally_marker(position, altitude),
                None => self.tell(ERROR, INVALID_ALT),
            },
            PromptKind::Circle { spline, position } => {
                self.circle_answers.push(value);
                if !self.ask_circle(spline, position) {
                    self.make_circle(plan, context, spline, position);
                }
            }
            PromptKind::Message | PromptKind::HomeLatEnter => {}
            PromptKind::FenceMinAlt | PromptKind::FenceMaxAlt => {
                if let Some(ask) = plan.fence_ask.as_mut() {
                    match ask.answer(&prompt.kind, &value) {
                        Ok(()) => self.continue_fence_upload(plan),
                        Err(text) => {
                            plan.fence_ask = None;
                            plan.fence_upload_said(text);
                        }
                    }
                }
            }
        }
        None
    }

    /// Geo-Fence > Upload's next box, or the upload itself once none is left: a parameter that
    /// cannot be offered is `int.Parse`'s exception, and the upload goes no further.
    /// `// C#: GCSViews/FlightPlanner.cs:3761-3793`
    pub fn continue_fence_upload(&mut self, plan: &mut Plan) {
        let Some(ask) = plan.fence_ask.as_ref() else {
            return;
        };
        match ask.next_question() {
            Some(Ok((title, text, offered, kind))) => {
                self.ask(Prompt::input(title, text, offered, kind));
            }
            Some(Err(text)) => {
                plan.fence_ask = None;
                plan.fence_upload_said(text);
            }
            None => {
                if let Some(ask) = plan.fence_ask.take() {
                    plan.fence_upload = Some(FenceUpload::new(ask));
                }
            }
        }
    }

    /// Cancel, or No: the handler returns - bar three whose C# goes on. Offset Polygon's
    /// `InputBox` returning Cancel leaves the offset at 0 and offsets by it
    /// (`FlightPlanner.cs:3647-3653`); From SHP clears the polygon before it looks at what its
    /// dialog returned (`:3541-3544`); and Area's "Please define a polygon!" is a message whose
    /// closing says the area all the same (`:1994-1995`).
    pub fn cancel(&mut self, plan: &mut Plan) -> Option<FileRequest> {
        let prompt = self.prompt.take()?;
        match prompt.kind {
            PromptKind::OffsetPolygon => plan.offset_polygon(0.0),
            PromptKind::ShpLoadFile => return Some(FileRequest::LoadShp(String::new())),
            // No to the flight screen still asks about the zoom.
            PromptKind::KmlToFlightScreen => self.ask_kml_zoom(),
            // `InputBox.Show("", "startalt", ref startalt)` reads the box whatever its button
            // said: Cancel keeps what was offered and the next box opens.
            // `// C#: ExtLibs/Controls/InputBox.cs:21-27; Utilities/CircleSurveyMission.cs:17-22`
            PromptKind::CircleSurvey { step, position } => {
                let offered = self.prompt_offered_survey(step);
                self.survey_answers.push(offered);
                if step < 5 {
                    self.ask_survey_step(step + 1, position);
                } else {
                    self.survey_pending = Some(position);
                }
            }
            // Text's boxes keep what they offered when cancelled: "", "5" and "0".
            PromptKind::TextString { position } => self.text_answer(0, position, String::new()),
            PromptKind::TextSize { position } => self.text_answer(1, position, "5".to_owned()),
            PromptKind::TextRotation { position } => self.text_pending = Some(position),
            PromptKind::DefinePolygon => self.tell("Area", area_text(0.0)),
            PromptKind::Circle { .. } => self.circle_answers.clear(),
            // `if (DialogResult.Cancel == InputBox.Show(...)) return;`.
            // `// C#: GCSViews/FlightPlanner.cs:3766-3767, 3782-3783`
            PromptKind::FenceMinAlt | PromptKind::FenceMaxAlt => plan.fence_ask = None,
            // No: `CMB_altmode.SelectedValue = (int) altmode.Relative`, and on with the write.
            PromptKind::WriteAltMode => {
                self.write_answer = Some(WriteAnswer::RelativeThenContinue);
            }
            // Cancel: `if (!checkZeroAlts(a)) return;`.
            PromptKind::WriteZeroAlt => {
                self.write_flow = None;
                self.write_answer = Some(WriteAnswer::Abort);
            }
            _ => {}
        }
        None
    }
}

/// The context the menu needs, from what the application knows right now.
pub(crate) fn menu_context(this: &MissionPlanner) -> MenuContext {
    let view = this.telemetry.view();
    let state = view.state.as_ref();
    MenuContext {
        frame: this.altitude_frame,
        vehicle: state.and_then(|state| {
            state
                .position
                .map(|position| (position, state.altitude_relative.0))
        }),
        takeoff_pitch: MenuContext::asks_takeoff_pitch(
            state.map(|state| state.vehicle_type),
            &view.parameters,
        ),
        copter: firmware_is_copter(&view),
        tracker_alt: state.map_or(0.0, |state| {
            let tracker = state.tracker_location();
            if tracker.alt == 0.0 {
                state.home_altitude.0
            } else {
                tracker.alt
            }
        }),
    }
}

/// Pushes everything the menu can change to the map.
fn sync_everything(this: &MissionPlanner) {
    this.sync_map_mission();
    this.sync_map_polygon();
    this.sync_map_fence();
    this.sync_map_rally();
    let mut map = this.map.borrow_mut();
    map.set_kml(this.plan.kml_overlay(), this.plan.kml_on_flight());
    map.set_tracker(tracker_marker(this));
    map.set_grid(this.plan.grid());
}

/// What POI > Delete or Edit found under the press, for a test to read: the index, the press,
/// and where the markers were drawn.
fn record_poi_hit(this: &MissionPlanner, at: Option<(f32, f32)>, hit: Option<usize>) {
    crate::facts::record(
        "plan.poi.hit",
        hit.map_or_else(|| "none".to_owned(), |index| index.to_string()),
    );
    crate::facts::record(
        "plan.poi.press",
        at.map_or_else(|| "none".to_owned(), |(x, y)| format!("{x:.0},{y:.0}")),
    );
    let map = this.map.borrow();
    crate::facts::record(
        "plan.poi.spots",
        this.fly_data
            .pois
            .points()
            .iter()
            .map(|poi| {
                poi.position()
                    .and_then(|at| map.screen_of(at))
                    .map_or_else(|| "off".to_owned(), |(x, y)| format!("{x:.0},{y:.0}"))
            })
            .collect::<Vec<_>>()
            .join("|"),
    );
}

/// `CurrentPOIMarker`: the POI marker under a window point, by the flight screen's own hit test.
/// `// C#: GCSViews/FlightPlanner.cs:8113-8116`
fn poi_under(this: &MissionPlanner, at: (f32, f32)) -> Option<usize> {
    let drawn: Vec<Option<(f32, f32)>> = {
        let map = this.map.borrow();
        this.fly_data
            .pois
            .points()
            .iter()
            .map(|poi| poi.position().and_then(|at| map.screen_of(at)))
            .collect()
    };
    crate::poi::under(&drawn, at)
}

/// `timer1_Tick`'s "Tracker Home" marker: `addpolygonmarker("Tracker Home", TrackerLocation,
/// Color.Blue)` while the tracker's position is not home's and its longitude is not 0.
/// `// C#: GCSViews/FlightPlanner.cs:6910-6916`
fn tracker_marker(this: &MissionPlanner) -> Option<mapview::GuidedMarker> {
    let view = this.telemetry.view();
    let state = view.state.as_ref()?;
    let tracker = state.tracker_location();
    if tracker.lng == 0.0 {
        return None;
    }
    let home = state.home;
    if home.is_some_and(|home| home.latitude() == tracker.lat && home.longitude() == tracker.lng) {
        return None;
    }
    let position = LatLon::new(tracker.lat, tracker.lng).ok()?;
    let wp_radius = this
        .plan
        .panel_text(PanelBox::WpRadius)
        .trim()
        .parse::<f32>()
        .map_or(0.0, f64::from);
    #[allow(clippy::cast_possible_truncation)] // `(int) TrackerLocation.Alt`
    let alt = tracker.alt as i32;
    Some(mapview::GuidedMarker {
        tag: "Tracker Home",
        position,
        alt,
        wp_radius,
    })
}

// ---------------------------------------------------------------------------------------------
// The map's zoom: the zoom icon and its menu, Map Tool > Zoom To, and the Zoom box and bar at the
// map's right (`Zoomlevel`, `TRK_zoom`). Then what the map is handed of the panel's radii, and the
// pointer over its markers.
// ---------------------------------------------------------------------------------------------

/// The zoom Zoom to Vehicle and Zoom to Home bring a wider view in to: `if (MainMap.Zoom < 17)
/// MainMap.Zoom = 17`.
/// `// C#: GCSViews/FlightPlanner.cs:8379-8380, 8398-8399`
pub const ZOOM_IN_TO: f64 = 17.0;
/// The zoom Zoom To leaves a place it found at.
/// `// C#: GCSViews/FlightPlanner.cs:8365`
pub const ZOOM_TO_PLACE: f64 = 15.0;
/// `Zoomlevel.Increment`.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:811-815`
pub const ZOOMLEVEL_INCREMENT: f64 = 0.5;
/// Where the zoom bar's thumb stops short of each end, pixels.
const TRACK_INSET: f32 = 8.0;
/// Where `zoomicon` sits on the map, and its size: ten in from the left and five below
/// `polyicon` at (10, 100), thirty across.
/// `// C#: GCSViews/FlightPlanner.cs:4905-4911; Controls/Icon/Icon.cs:12-13`
pub const ZOOM_ICON: (f32, f32, f32) = (10.0, 135.0, 30.0);

/// The view brought in to [`ZOOM_IN_TO`] if it is wider than that.
fn zoom_in_to_17(map: &mut MapViewport) {
    if map.zoom_level().is_some_and(|zoom| zoom < ZOOM_IN_TO) {
        map.set_zoom(ZOOM_IN_TO);
    }
}

/// `zoomToVehicleToolStripMenuItem_Click`: the view centred on the vehicle and brought in to 17,
/// or "Invalid Location" while its position is 0,0 - as it is before one has been heard.
/// `// C#: GCSViews/FlightPlanner.cs:8370-8381`
pub fn zoom_to_vehicle(map: &mut MapViewport, vehicle: Option<LatLon>) -> Result<(), &'static str> {
    let Some(at) = vehicle.filter(|at| at.latitude() != 0.0 || at.longitude() != 0.0) else {
        return Err("Invalid Location");
    };
    map.centre_on(at);
    zoom_in_to_17(map);
    Ok(())
}

/// `zoomToHomeToolStripMenuItem_Click`: the view centred on the vehicle's home, or failing that
/// the planned home, and brought in to 17 whether either was there or not. The planned home is
/// taken when its latitude is not zero: the C# tests `PlannedHomeLocation.Lat != 0` twice, and
/// never the longitude.
/// `// C#: GCSViews/FlightPlanner.cs:8388-8401`
pub fn zoom_to_home(map: &mut MapViewport, vehicle_home: Option<LatLon>, planned: Home) {
    if let Some(home) =
        vehicle_home.filter(|home| home.latitude() != 0.0 && home.longitude() != 0.0)
    {
        map.centre_on(home);
    } else if planned.lat != 0.0
        && let Ok(home) = LatLon::new(planned.lat, planned.lng)
    {
        map.centre_on(home);
    }
    zoom_in_to_17(map);
}

/// What `zoomToToolStripMenuItem_Click` says, captioned "GMap.NET", when the geocoder does not
/// answer `G_GEO_SUCCESS`.
/// `// C#: GCSViews/FlightPlanner.cs:8358-8362`
#[must_use]
pub fn zoom_to_message(place: &str, status: mapview::GeocoderStatus) -> String {
    format!("Google Maps Geocoder can't find: '{place}', reason: {status}")
}

/// The rest of `zoomToToolStripMenuItem_Click` once the geocoder has answered: the view centred
/// on the first place found (`SetPositionByKeywords` moves it only when there is one) and zoomed
/// to 15, or the message to show.
/// `// C#: GCSViews/FlightPlanner.cs:8349-8367; ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:2457-2477`
pub fn zoom_to_answer(
    map: &mut MapViewport,
    place: &str,
    (status, found): (mapview::GeocoderStatus, Option<LatLon>),
) -> Option<String> {
    if status != mapview::GeocoderStatus::Success {
        return Some(zoom_to_message(place, status));
    }
    if let Some(at) = found {
        map.centre_on(at);
    }
    map.set_zoom(ZOOM_TO_PLACE);
    None
}

/// One of the zoom icon's entries.
fn zoom_menu_entry(this: &mut MissionPlanner, action: MenuAction) {
    let view = this.telemetry.view();
    let state = view.state.as_ref();
    let refused = {
        let mut map = this.map.borrow_mut();
        match action {
            MenuAction::ZoomToVehicle => {
                zoom_to_vehicle(&mut map, state.and_then(|state| state.position)).err()
            }
            MenuAction::ZoomToMission => {
                map.zoom_and_centre_markers();
                None
            }
            MenuAction::ZoomToHome => {
                zoom_to_home(
                    &mut map,
                    state.and_then(|state| state.home),
                    this.plan.planned_home_location(),
                );
                None
            }
            _ => None,
        }
    };
    // `CustomMessageBox.Show(Strings.Invalid_Location, Strings.ERROR)`.
    if let Some(why) = refused {
        this.plan_menus.tell(ERROR, why);
    }
}

/// Takes up Zoom To's answer when the geocoder has sent it.
pub fn drive_geocode(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let Some((place, outcome)) = this.plan_menus.take_geocode() else {
        return;
    };
    let said = zoom_to_answer(&mut this.map.borrow_mut(), &place, outcome);
    if let Some(text) = said {
        this.plan_menus.tell("GMap.NET", text);
        this.plan_prompt_focus.focus(window, cx);
    }
    cx.notify();
}

/// `Zoomlevel`'s text: its value with `DecimalPlaces = 1`, rounded half away from zero as a
/// `decimal` formats.
#[must_use]
pub fn zoomlevel_text(value: f64) -> String {
    format!("{:.1}", (value * 10.0).round() / 10.0)
}

/// `Zoomlevel`'s up or down arrow: its value moved by `Increment` and held to its 0 to 24.
/// `// C#: GCSViews/FlightPlanner.cs:3455-3457; GCSViews/FlightPlanner.Designer.cs:809-833`
#[must_use]
pub fn zoomlevel_step(value: f64, up: bool) -> f64 {
    let step = if up {
        ZOOMLEVEL_INCREMENT
    } else {
        -ZOOMLEVEL_INCREMENT
    };
    (value + step).clamp(mapview::GMAP_MIN_ZOOM, mapview::GMAP_MAX_ZOOM)
}

/// `TRK_zoom`'s value for a press at `y` on a bar laid out from `top`, `height` tall: its maximum
/// at the top and its minimum at the bottom as a vertical `TrackBar` runs, the thumb stopping
/// [`TRACK_INSET`] short of each end, in whole thousandths - `MyTrackBar` keeps its value as an
/// integer, `(int)(value * 1000)`.
///
/// A press away from the thumb puts the thumb there. A WinForms `TrackBar` instead moves it by
/// `LargeChange`, 0.005 here, toward the press, again and again while the button is held; a drag
/// is the same either way.
/// `// C#: ExtLibs/Controls/MyTrackBar.cs:11-22; GCSViews/FlightPlanner.Designer.cs:837-846;
/// GCSViews/FlightPlanner.cs:3451-3453`
#[must_use]
pub fn track_value(y: f32, top: f32, height: f32) -> f32 {
    #[allow(clippy::cast_possible_truncation)] // 0 and 24
    let (min, max) = (mapview::GMAP_MIN_ZOOM as f32, mapview::GMAP_MAX_ZOOM as f32);
    let travel = (height - 2.0 * TRACK_INSET).max(1.0);
    let fraction = ((y - top - TRACK_INSET) / travel).clamp(0.0, 1.0);
    let value = fraction.mul_add(-(max - min), max);
    (value * 1000.0).trunc() / 1000.0
}

/// A press or a drag on the zoom bar: `TRK_zoom_Scroll`, `MainMap.Zoom = TRK_zoom.Value` - and
/// the Zoom box, which shows the map's zoom, with it.
/// `// C#: GCSViews/FlightPlanner.cs:6938-6952`
fn zoom_track_press(this: &mut MissionPlanner, y: f32) {
    let Some((top, height)) = this.plan_menus.zoom_track.get() else {
        return;
    };
    let value = track_value(y, top, height);
    this.map.borrow_mut().set_zoom(f64::from(value));
}

/// The zoom icon, `zoomicon`, on the planning map; a click opens `contextMenuStripZoom` where it
/// was made. Its press is the icon's, not the map's, as `MainMap_MouseUp` returns once it has
/// shown the menu.
/// `// C#: GCSViews/FlightPlanner.cs:133, 4905-4911, 7624-7628`
pub fn zoom_icon(cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured("plan-zoomicon", div())
        .absolute()
        .left(px(ZOOM_ICON.0))
        .top(px(ZOOM_ICON.1))
        .child(
            div()
                .id("plan-zoomicon")
                .size(px(ZOOM_ICON.2))
                .rounded_full()
                .bg(rgb(0x00_00_00))
                .border_1()
                .border_color(rgb(ZOOM_ICON_LINE))
                .occlude()
                .cursor_pointer()
                .child(
                    gpui::canvas(
                        |_bounds, _window, _cx| (),
                        |bounds, (), window, _cx| paint_zoom_glyph(bounds, window),
                    )
                    .size_full(),
                )
                .on_mouse_down(gpui::MouseButton::Left, |_event, _window, cx| {
                    cx.stop_propagation();
                })
                .on_mouse_up(
                    gpui::MouseButton::Left,
                    cx.listener(|this, event: &gpui::MouseUpEvent, _window, cx| {
                        let at = (f32::from(event.position.x), f32::from(event.position.y));
                        this.plan_menus.open_zoom_menu(at);
                        cx.stop_propagation();
                        cx.notify();
                    }),
                ),
        )
        .into_any_element()
}

/// `Icon.ForeColor`, `Color.WhiteSmoke`: the rim and the glass.
/// `// C#: Controls/Icon/Icon.cs:10`
const ZOOM_ICON_LINE: u32 = 0xf5_f5_f5;

/// `Zoom.doPaint`'s magnifying glass, in one-pixel white smoke over the icon's black disc: with
/// `mid` half the icon's 30 and `quartmid` a quarter of that (15 and 3, in whole numbers), a lens
/// `mid` across at (`mid - quartmid`, `quartmid`), a handle from (`mid / 2`, `30 - mid / 2`) to
/// (`mid`, `mid`), and a cross inside the lens four in from its edge.
/// `// C#: Controls/Icon/Zoom.cs:7-26; Controls/Icon/Icon.cs:86-102`
fn paint_zoom_glyph(bounds: gpui::Bounds<gpui::Pixels>, window: &mut gpui::Window) {
    let at = |x: f32, y: f32| gpui::point(bounds.origin.x + px(x), bounds.origin.y + px(y));
    let stroke = |window: &mut gpui::Window, points: &[(f32, f32)]| {
        let mut builder = gpui::PathBuilder::stroke(px(1.0));
        let mut points = points.iter();
        let Some((x, y)) = points.next() else {
            return;
        };
        builder.move_to(at(*x, *y));
        for (x, y) in points {
            builder.line_to(at(*x, *y));
        }
        if let Ok(path) = builder.build() {
            window.paint_path(path, gpui::Hsla::from(rgb(ZOOM_ICON_LINE)));
        }
    };
    let (mid, quartmid) = (15.0_f32, 3.0_f32);
    let lens: Vec<(f32, f32)> = (0..=32_u8)
        .map(|step| {
            let angle = f32::from(step) / 32.0 * std::f32::consts::TAU;
            (
                (mid / 2.0).mul_add(angle.cos(), mid - quartmid + mid / 2.0),
                (mid / 2.0).mul_add(angle.sin(), quartmid + mid / 2.0),
            )
        })
        .collect();
    stroke(window, &lens);
    stroke(window, &[(7.0, 23.0), (mid, mid)]);
    // `arcrect.Inflate(-4, -4)`: (16, 7), seven across.
    let (left, top, side) = (mid - quartmid + 4.0, quartmid + 4.0, mid - 8.0);
    stroke(
        window,
        &[(left, top + side / 2.0), (left + side, top + side / 2.0)],
    );
    stroke(
        window,
        &[(left + side / 2.0, top), (left + side / 2.0, top + side)],
    );
}

/// `contextMenuStripZoom`, where the button came up over the zoom icon.
fn zoom_menu(menus: &PlanMenus, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    let at = menus.zoom_menu?;
    let rows = ZOOM_MENU
        .iter()
        .map(|entry| menu_row(entry, None, true, false, cx))
        .collect();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(at.0), px(at.1)))
                .snap_to_window()
                .child(
                    div()
                        .id("plan-zoom-menu")
                        .occlude()
                        .on_mouse_down_out(cx.listener(
                            |this, event: &gpui::MouseDownEvent, _window, cx| {
                                let at = (f32::from(event.position.x), f32::from(event.position.y));
                                this.plan_menus.dismiss_zoom_menu(at);
                                cx.notify();
                            },
                        ))
                        .child(menu_column(rows)),
                ),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// Where `polyicon` sits on the map, and its size: ten in from the left, a hundred down, thirty
/// across. `// C#: GCSViews/FlightPlanner.cs:4905-4906; Controls/Icon/Icon.cs:12-13`
pub const POLY_ICON: (f32, f32, f32) = (10.0, 100.0, 30.0);

/// The polygon icon, `polyicon`, on the planning map: the left button coming up over it opens
/// `contextMenuStripPoly` there; the right button clears the polygon and hides the map's menu.
/// `// C#: GCSViews/FlightPlanner.cs:132, 4905-4906, 7607-7618`
pub fn poly_icon(cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured("plan-polyicon", div())
        .absolute()
        .left(px(POLY_ICON.0))
        .top(px(POLY_ICON.1))
        .child(
            div()
                .id("plan-polyicon")
                .size(px(POLY_ICON.2))
                .rounded_full()
                .bg(rgb(0x00_00_00))
                .border_1()
                .border_color(rgb(ZOOM_ICON_LINE))
                .occlude()
                .cursor_pointer()
                .child(
                    gpui::canvas(
                        |_bounds, _window, _cx| (),
                        |bounds, (), window, _cx| paint_poly_glyph(bounds, window),
                    )
                    .size_full(),
                )
                .on_mouse_down(gpui::MouseButton::Left, |_event, _window, cx| {
                    cx.stop_propagation();
                })
                .on_mouse_down(gpui::MouseButton::Right, |_event, _window, cx| {
                    cx.stop_propagation();
                })
                .on_mouse_up(
                    gpui::MouseButton::Left,
                    cx.listener(|this, event: &gpui::MouseUpEvent, _window, cx| {
                        let at = (f32::from(event.position.x), f32::from(event.position.y));
                        this.plan_menus.open_poly_menu(at);
                        cx.stop_propagation();
                        cx.notify();
                    }),
                )
                .on_mouse_up(
                    gpui::MouseButton::Right,
                    cx.listener(|this, _event: &gpui::MouseUpEvent, _window, cx| {
                        // `polyicon.IsSelected = false; clearPolygonToolStripMenuItem_Click(...);
                        // contextMenuStrip1.Visible = false;`
                        this.plan.load_polygon(Vec::new());
                        this.plan_menus.open = None;
                        this.plan_menus.poly_menu = None;
                        sync_everything(this);
                        cx.stop_propagation();
                        cx.notify();
                    }),
                ),
        )
        .into_any_element()
}

/// `Polygon.doPaint`: five points of one-pixel white smoke over the icon's black disc, with
/// `mid` half its thirty: (mid - 7, mid - 7), (mid + 7, mid - 10), (mid, mid + 12), (mid - 5,
/// mid + 10) and back to the first.
/// `// C#: Controls/Icon/Polygon.cs:7-20`
fn paint_poly_glyph(bounds: gpui::Bounds<gpui::Pixels>, window: &mut gpui::Window) {
    let mid = 15.0_f32;
    let points = [
        (mid - 7.0, mid - 7.0),
        (mid + 7.0, mid - 10.0),
        (mid, mid + 12.0),
        (mid - 5.0, mid + 10.0),
        (mid - 7.0, mid - 7.0),
    ];
    let mut builder = gpui::PathBuilder::stroke(px(1.0));
    let mut iter = points.iter();
    if let Some((x, y)) = iter.next() {
        builder.move_to(gpui::point(
            bounds.origin.x + px(*x),
            bounds.origin.y + px(*y),
        ));
    }
    for (x, y) in iter {
        builder.line_to(gpui::point(
            bounds.origin.x + px(*x),
            bounds.origin.y + px(*y),
        ));
    }
    if let Ok(path) = builder.build() {
        window.paint_path(path, gpui::Hsla::from(rgb(ZOOM_ICON_LINE)));
    }
}

/// `contextMenuStripPoly`, open where the polygon icon was clicked: its entries, the two fence
/// ones only while the geofence is being drawn (`ContextMenuStripPoly_Opening`).
/// `// C#: GCSViews/FlightPlanner.cs:2697-2715, 7618`
fn poly_menu(
    menus: &PlanMenus,
    fence_mode: bool,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let at = menus.poly_menu?;
    let rows = POLY_MENU
        .iter()
        .filter(|entry| fence_mode || !POLY_MENU_FENCE_ENTRIES.contains(&entry.id))
        .map(|entry| menu_row(entry, None, true, false, cx))
        .collect();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(at.0), px(at.1)))
                .snap_to_window()
                .child(
                    div()
                        .id("plan-poly-menu")
                        .occlude()
                        .on_mouse_down_out(cx.listener(
                            |this, event: &gpui::MouseDownEvent, _window, cx| {
                                let at = (f32::from(event.position.x), f32::from(event.position.y));
                                this.plan_menus.dismiss_poly_menu(at);
                                cx.notify();
                            },
                        ))
                        .child(menu_column(rows)),
                ),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// `label11` "Zoom", `Zoomlevel` and `TRK_zoom`: the strip fifty pixels wide at the planning
/// map's right. `panelMap_Resize` gives the map the panel's width less 50 and puts the bar in the
/// rest, from 42 down to the bottom; the label is at 5 and the box at 25. Both show the map's
/// zoom whenever it changes (`MainMap_OnMapZoomChanged`) - held here to their 0 to 24, where the
/// C#'s refuse a zoom outside it and keep the last.
/// `// C#: GCSViews/FlightPlanner.resx (label11, Zoomlevel, TRK_zoom); GCSViews/FlightPlanner.cs:4960-4968, 8012-8029`
pub fn zoom_column(
    zoom: Option<f64>,
    menus: &PlanMenus,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let value = zoom.map(|zoom| zoom.clamp(mapview::GMAP_MIN_ZOOM, mapview::GMAP_MAX_ZOOM));
    // `Zoomlevel_ValueChanged`: `MainMap.Zoom = Zoomlevel.Value`, the bar following.
    // `// C#: GCSViews/FlightPlanner.cs:6954-6969`
    let arrow =
        |id: &'static str, label: &'static str, up: bool, cx: &mut Context<MissionPlanner>| {
            crate::probe::measured(id, div())
                .child(
                    div()
                        .id(id)
                        .px_1()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .cursor_pointer()
                        .hover(|style| style.bg(rgb(theme::BORDER)))
                        .child(label)
                        .on_click(cx.listener(move |this, _event, window, cx| {
                            let now = this.map.borrow().zoom_level();
                            if let Some(now) = now {
                                this.map.borrow_mut().set_zoom(zoomlevel_step(
                                    now.clamp(mapview::GMAP_MIN_ZOOM, mapview::GMAP_MAX_ZOOM),
                                    up,
                                ));
                            }
                            window.refresh();
                            cx.notify();
                        })),
                )
                .into_any_element()
        };
    let track = std::rc::Rc::clone(&menus.zoom_track);
    let bar = div()
        .id("plan-trk-zoom")
        .size_full()
        .cursor_pointer()
        .child(
            gpui::canvas(
                move |bounds, _window, _cx| {
                    track.set(Some((
                        f32::from(bounds.origin.y),
                        f32::from(bounds.size.height),
                    )));
                },
                move |bounds, (), window, _cx| paint_zoom_track(bounds, value, window),
            )
            .size_full(),
        )
        .on_mouse_down(
            gpui::MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseDownEvent, window, cx| {
                zoom_track_press(this, f32::from(event.position.y));
                window.refresh();
                cx.notify();
            }),
        )
        .on_mouse_move(
            cx.listener(|this, event: &gpui::MouseMoveEvent, window, cx| {
                if event.pressed_button == Some(gpui::MouseButton::Left) {
                    zoom_track_press(this, f32::from(event.position.y));
                    window.refresh();
                    cx.notify();
                }
            }),
        );
    div()
        .flex()
        .flex_col()
        .items_center()
        .w(px(50.0))
        .flex_none()
        .pt(px(5.0))
        .gap_1()
        .child(div().text_xs().text_color(rgb(theme::DIM)).child("Zoom"))
        .child(
            div()
                .flex()
                .items_center()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .rounded_sm()
                .child(
                    crate::probe::measured("plan-zoomlevel", div()).child(
                        div()
                            .id("plan-zoomlevel")
                            .px_1()
                            .text_xs()
                            .text_color(rgb(theme::TEXT))
                            .child(value.map_or_else(String::new, zoomlevel_text)),
                    ),
                )
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .child(arrow("plan-zoomlevel-up", "▴", true, cx))
                        .child(arrow("plan-zoomlevel-down", "▾", false, cx)),
                ),
        )
        .child(
            crate::probe::measured("plan-trk-zoom", div())
                .flex_1()
                .w_full()
                .min_h(px(0.0))
                .child(bar),
        )
        .into_any_element()
}

/// `TRK_zoom` as a vertical `TrackBar` with `TickStyle.TopLeft` draws it: a channel down the
/// middle, a tick at each whole zoom on the left, and the thumb at the value.
fn paint_zoom_track(
    bounds: gpui::Bounds<gpui::Pixels>,
    value: Option<f64>,
    window: &mut gpui::Window,
) {
    let (x, y) = (f32::from(bounds.origin.x), f32::from(bounds.origin.y));
    let (width, height) = (f32::from(bounds.size.width), f32::from(bounds.size.height));
    let travel = (height - 2.0 * TRACK_INSET).max(1.0);
    #[allow(clippy::cast_possible_truncation)] // 0 and 24
    let (min, max) = (mapview::GMAP_MIN_ZOOM as f32, mapview::GMAP_MAX_ZOOM as f32);
    let at_value = |value: f32| y + TRACK_INSET + (max - value) / (max - min) * travel;
    let fill = |window: &mut gpui::Window, left: f32, top: f32, w: f32, h: f32, colour: u32| {
        window.paint_quad(gpui::quad(
            gpui::Bounds {
                origin: gpui::point(px(left), px(top)),
                size: gpui::size(px(w), px(h)),
            },
            gpui::Corners::all(px(1.0)),
            rgb(colour),
            gpui::Edges::default(),
            rgb(colour),
            gpui::BorderStyle::default(),
        ));
    };
    let middle = x + width / 2.0;
    fill(
        window,
        middle - 2.0,
        y + TRACK_INSET,
        4.0,
        travel,
        theme::BORDER,
    );
    for tick in 0..=24_u8 {
        fill(
            window,
            middle - 14.0,
            at_value(f32::from(tick)),
            5.0,
            1.0,
            theme::DIM,
        );
    }
    if let Some(value) = value {
        #[allow(clippy::cast_possible_truncation)] // a zoom, 0 to 24
        let top = at_value(value as f32) - 4.0;
        fill(window, middle - 8.0, top, 16.0, 8.0, theme::ACCENT);
    }
}

/// The WP Radius and Loiter Radius boxes as `writeKML` hands them to `CreateOverlay` for the
/// planning map: an empty WP Radius read as `startupWPradius` and an empty Loiter Radius as 30,
/// each over `multiplierdist`, which is 1 in metres.
///
/// `None` when either does not parse. There `double.Parse` throws, and the C# says "Invalid
/// number entered" and draws an empty overlay; here the circles go and the markers stay, and the
/// message is not ported. `writeKML` also writes those two defaults back into an empty box; the
/// boxes' own Leave puts WP Radius's back before any edit could reach `writeKML`, and this
/// leaves the text alone.
///
/// Read every frame, as home is. The C# reads the boxes at each `writeKML` - every edit, and WP
/// Radius's Leave - so its circles wait for the edit or the Leave where these follow the box as it
/// is typed.
/// `// C#: GCSViews/FlightPlanner.cs:1423-1440, 7087-7106`
#[must_use]
pub fn map_overlay(plan: &Plan) -> Option<mapview::Overlay> {
    let read = |which: PanelBox, empty: &str| {
        let text = plan.panel_text(which);
        let text = if text.is_empty() { empty } else { text };
        text.trim().parse::<f64>().ok()
    };
    Some(mapview::Overlay {
        wp_radius: read(PanelBox::WpRadius, &plan.panel.startup_wp_radius)?,
        loiter_radius: read(PanelBox::LoiterRadius, "30")?,
        planner: true,
    })
}

/// The pointer over the map with no button down, or off it (`None`): GMap's hover, and on the
/// planning screen `MainMap_OnMarkerEnter`, which puts the grid on the row of a rect the pointer
/// has entered - the last, where it entered several; home's "H" is not a row. Returns whether the
/// screen must be drawn again.
/// `// C#: GCSViews/FlightPlanner.cs:8068-8098`
pub fn map_hover(this: &mut MissionPlanner, planning: bool, pointer: Option<(f32, f32)>) -> bool {
    let change = this.map.borrow_mut().hover(pointer);
    if planning && let Some(seq) = entered_row(&change.entered) {
        this.plan.select(Some(seq));
    }
    change.changed
}

/// The row `MainMap_OnMarkerEnter` leaves the grid on after the rects just entered: each one
/// whose inner marker's tag is a number moves it there, in turn, so the last such wins.
/// `// C#: GCSViews/FlightPlanner.cs:8077-8083`
#[must_use]
pub fn entered_row(entered: &[mapview::MarkerTag]) -> Option<u16> {
    entered.iter().rev().find_map(|tag| match tag {
        mapview::MarkerTag::Item(seq) => Some(*seq),
        mapview::MarkerTag::Home
        | mapview::MarkerTag::Guided
        | mapview::MarkerTag::Tracker
        | mapview::MarkerTag::Photo(_) => None,
    })
}

/// The flight screen's Guided Mode marker: there while the vehicle's mode is Guided and
/// `GuidedMode.x` is not zero, at `GuidedMode`'s position and whole height, its rect the saved WP
/// Radius - `Settings.GetFloat("TXT_WPRad")`, 0 where that is not a number. What the planning
/// screen saves under that key is its WP Radius box as it is left, so the box is what is read.
///
/// The C# clears `routes` every five seconds and adds the marker back on each pass while in
/// Guided, so out of Guided it can linger up to five seconds; here it goes with the mode.
/// `// C#: GCSViews/FlightData.cs:3807, 4214-4221, 5518-5526; GCSViews/FlightPlanner.cs:1652-1690,
/// 2581; ExtLibs/Utilities/Settings.cs:234-243`
#[must_use]
pub fn guided_marker(
    mode: Option<&str>,
    guided: crate::fly::GuidedMode,
    plan: &Plan,
) -> Option<mapview::GuidedMarker> {
    if !mode.is_some_and(|mode| mode.eq_ignore_ascii_case("guided")) || guided.x == 0 {
        return None;
    }
    let position = LatLon::new(f64::from(guided.x) / 1e7, f64::from(guided.y) / 1e7).ok()?;
    let wp_radius = plan
        .panel_text(PanelBox::WpRadius)
        .trim()
        .parse::<f32>()
        .map_or(0.0, f64::from);
    #[allow(clippy::cast_possible_truncation)] // `(int) GuidedMode.z`
    let alt = guided.z as i32;
    Some(mapview::GuidedMarker {
        tag: "Guided Mode",
        position,
        alt,
        wp_radius,
    })
}

/// Facts for the zoom: the zoom icon's menu, a Zoom To waiting on the geocoder, and the Zoom
/// box's text.
#[must_use]
pub fn zoom_facts(menus: &PlanMenus, zoom: Option<f64>) -> Vec<(&'static str, String)> {
    vec![
        (
            "plan.zoommenu",
            if menus.zoom_menu.is_some() {
                "open"
            } else {
                "closed"
            }
            .to_owned(),
        ),
        (
            "plan.polymenu",
            if menus.poly_menu.is_some() {
                "open"
            } else {
                "closed"
            }
            .to_owned(),
        ),
        ("plan.utm.zone", menus.utm_zone().to_owned()),
        ("plan.geocoding", menus.geocoding().to_string()),
        (
            "plan.zoomlevel",
            zoom.map_or_else(
                || "none".to_owned(),
                |zoom| zoomlevel_text(zoom.clamp(mapview::GMAP_MIN_ZOOM, mapview::GMAP_MAX_ZOOM)),
            ),
        ),
    ]
}

/// Opens the menu for a right click on the map, as `MainMap.ContextMenuStrip = contextMenuStrip1`
/// does, with the waypoint under the cursor for Delete WP.
/// `// C#: GCSViews/FlightPlanner.Designer.cs:875; GCSViews/FlightPlanner.cs:2667-2695`
pub fn open_map_menu(this: &mut MissionPlanner, x: f32, y: f32) {
    let (position, marker) = {
        let map = this.map.borrow();
        (map.position_at(x, y), map.waypoint_at(x, y))
    };
    let Some(position) = position else {
        return;
    };
    // The same reason as placing a waypoint: an edit made from the menu must not refit the view
    // and move the ground under the cursor.
    this.map.borrow_mut().freeze_view();
    let held = capabilities(&this.telemetry.view());
    this.plan_menus.open_for((x, y), position, marker, held);
}

/// Chooses an entry from the open menu, and focuses the dialog if it opened one.
fn choose_entry(
    this: &mut MissionPlanner,
    action: MenuAction,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    // An entry of the polygon icon's menu: the icon's press stands for the map's, as the C#'s
    // handlers read `MouseDownStart`/`MouseDownEnd` from wherever the button last went down.
    if let Some(at) = this.plan_menus.poly_menu.take() {
        let position = this.map.borrow().position_at(at.0, at.1);
        if let Some(position) = position {
            this.plan_menus.open_at(at, position, None);
        }
    }
    match action {
        MenuAction::LoadWpFile => this.plan_menus.ask_mission_load(),
        // `if (CurrentPOIMarker == null) return; POI.POIDelete(CurrentPOIMarker)`: the marker the
        // menu opened over, found where the button came up.
        // `// C#: GCSViews/FlightPlanner.cs:5014-5019, 8113-8116; Utilities/POI.cs:87-102`
        MenuAction::PoiDelete => {
            let at = this.plan_menus.open.take().map(|menu| menu.at);
            let hit = at.and_then(|at| poi_under(this, at));
            record_poi_hit(this, at, hit);
            if let Some(index) = hit {
                this.fly_data.pois.delete(index);
            }
        }
        // `POI.POIEdit(CurrentPOIMarker)`: "Enter ID" for the marker the menu opened over.
        // `// C#: GCSViews/FlightPlanner.cs:5021-5027; Utilities/POI.cs:104-124`
        MenuAction::PoiEdit => {
            let at = this.plan_menus.open.take().map(|menu| menu.at);
            let hit = at.and_then(|at| poi_under(this, at));
            record_poi_hit(this, at, hit);
            if let Some(index) = hit {
                this.plan_menus.ask_poi_edit(index);
            }
        }
        MenuAction::FenceInclusion => {
            this.plan_menus.open = None;
            this.plan.fence_inclusion_from_polygon();
        }
        MenuAction::FenceExclusion => {
            this.plan_menus.open = None;
            this.plan.fence_exclusion_from_polygon();
        }
        MenuAction::SaveWpFile => {
            let name = this.plan_file_name();
            this.plan_menus.ask_mission_save(&name);
        }
        MenuAction::FenceClear => {
            this.plan_menus.open = None;
            start_fence_clear(this);
        }
        MenuAction::GeoFenceUpload => {
            this.plan_menus.open = None;
            let view = this.telemetry.view();
            start_fence_upload(this, &view);
        }
        MenuAction::GeoFenceDownload => {
            this.plan_menus.open = None;
            let view = this.telemetry.view();
            start_fence_download(this, &view);
        }
        MenuAction::GetRallyPoints => {
            this.plan_menus.open = None;
            start_rally_download(this);
        }
        MenuAction::SaveRallyPoints => {
            this.plan_menus.open = None;
            start_rally_upload(this);
        }
        MenuAction::ClearRallyPoints => {
            this.plan_menus.open = None;
            start_rally_clear(this);
        }
        MenuAction::ZoomToVehicle | MenuAction::ZoomToMission | MenuAction::ZoomToHome => {
            this.plan_menus.zoom_menu = None;
            zoom_menu_entry(this, action);
        }
        // `// C#: GCSViews/FlightPlanner.cs:6762-6778`
        MenuAction::SwitchDocking => {
            this.plan_menus.open = None;
            this.toggle_docking();
        }
        // `MainMap.SelectedArea` is always empty here - this map has no rubber band - so the
        // question comes every time.
        // `// C#: GCSViews/FlightPlanner.cs:5032-5043`
        MenuAction::Prefetch => {
            this.plan_menus.open = None;
            this.plan_menus.ask(Prompt::question(
                crate::prefetch_ui::RIP_TITLE,
                crate::prefetch_ui::RIP_QUESTION,
                PromptKind::PrefetchRipArea,
            ));
        }
        // `// C#: GCSViews/FlightPlanner.cs:3305-3316`
        MenuAction::PrefetchWpPath => {
            this.plan_menus.open = None;
            this.plan_menus.ask(Prompt::input(
                crate::prefetch_ui::MAX_ZOOM_TITLE,
                crate::prefetch_ui::MAX_ZOOM_TEXT,
                crate::prefetch_ui::MAX_ZOOM_OFFERED,
                PromptKind::PrefetchMaxZoom,
            ));
        }
        // `// C#: GCSViews/FlightPlanner.cs:6755-6760`
        MenuAction::SurveyGrid => {
            this.plan_menus.open = None;
            crate::survey_ui::open(this, window, cx);
        }
        _ => {
            let context = menu_context(this);
            this.plan_menus.choose(&mut this.plan, action, &context);
        }
    }
    sync_everything(this);
    if this.plan_menus.prompt.is_some() {
        this.plan_prompt_focus.focus(window, cx);
    }
    cx.notify();
}

/// OK, Yes, or Enter in a dialog.
fn submit_prompt(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let context = menu_context(this);
    let home_hint = home_hint_showing(this);
    let request = this.plan_menus.submit(&mut this.plan, &context);
    // The answer kept as `InputBox` keeps it: the menus hold no settings, the window does.
    // `// C#: ExtLibs/Controls/InputBox.cs:178-184`
    if let Some((title, question, answer)) = this.plan_menus.take_answered() {
        crate::config::optional::remember_answer(&mut this.persisted, title, &question, &answer);
    }
    if let Some(request) = request {
        file_request(this, request, window, cx);
    }
    write_answered(this, window, cx);
    // Prefetch's Yes opens the menu over the view; Prefetch WP Path's zoom starts the walks.
    if this.plan_menus.take_prefetch_area_wanted() {
        crate::prefetch_ui::open_menu_over_view(this);
    }
    if let Some(zoom) = this.plan_menus.take_prefetch_path_zoom() {
        crate::prefetch_ui::start_path(this, zoom);
    }
    // Yes to "Zoom to the center or the loaded file?".
    // `// C#: GCSViews/FlightPlanner.cs:4264-4271`
    if this.plan_menus.take_zoom_to_kml() {
        this.map.borrow_mut().zoom_to_fit(&this.plan.kml_points());
    }
    // POI > Add or Edit, its ID typed: the flight screen's list, saved as it changes.
    // `// C#: Utilities/POI.cs:59-70, 104-124`
    match this.plan_menus.take_poi_request() {
        Some(PoiRequest::Add { lat, lng, id }) => this.fly_data.pois.add(lat, lng, 0.0, &id),
        Some(PoiRequest::Rename { index, id }) => {
            this.fly_data.pois.rename(index, &id);
        }
        None => {}
    }
    sync_everything(this);
    if this.plan_menus.prompt.is_some() {
        this.plan_prompt_focus.focus(window, cx);
    } else if home_hint {
        focus_home_lat(this, window, cx);
    }
    cx.notify();
}

/// Whether the dialog showing is `TXT_homelat_Enter`'s.
fn home_hint_showing(this: &MissionPlanner) -> bool {
    this.plan_menus
        .prompt
        .as_ref()
        .is_some_and(|prompt| prompt.kind == PromptKind::HomeLatEnter)
}

/// The keyboard back to the Lat box, where it was when its message box opened.
fn focus_home_lat(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    if let Some(handle) = this.plan_home_focus.first() {
        handle.focus(window, cx);
    }
}

/// Cancel, No, or Escape in a dialog: what the handler does when its dialog says Cancel.
fn cancel_prompt(
    this: &mut MissionPlanner,
    window: &mut gpui::Window,
    cx: &mut Context<MissionPlanner>,
) {
    let home_hint = home_hint_showing(this);
    if let Some(request) = this.plan_menus.cancel(&mut this.plan) {
        file_request(this, request, window, cx);
    }
    write_answered(this, window, cx);
    // Create Circle Survey's sixth box cancelled: the survey is made all the same.
    if let Some(position) = this.plan_menus.take_survey_pending() {
        let context = menu_context(this);
        this.plan_menus
            .finish_circle_survey(&mut this.plan, position, &context);
    }
    // Text's rotation box cancelled: the rotation is 0 and the text is made all the same.
    if let Some(position) = this.plan_menus.take_text_pending() {
        let context = menu_context(this);
        this.plan_menus
            .finish_text(&mut this.plan, position, "0", &context);
    }
    sync_everything(this);
    if this.plan_menus.prompt.is_some() {
        this.plan_prompt_focus.focus(window, cx);
    } else if home_hint {
        focus_home_lat(this, window, cx);
    }
    cx.notify();
}

/// Height of one menu entry, fixed so a drop-down can line up with the entry that opened it.
const MENU_ROW: f32 = 22.0;
/// Height of the separator.
const MENU_SEPARATOR: f32 = 9.0;
/// The menu's padding above its first entry.
const MENU_PADDING: f32 = 4.0;
/// The width of the menu and of each drop-down. Fixed, where a `ToolStripDropDown` sizes itself
/// to its widest item, because the drop-down is placed beside the menu by arithmetic rather than
/// by layout (see `map_menu`), and that arithmetic needs the width before the text is measured.
/// 190 px holds the widest entry, "From Current Waypoints", at this font with room to spare.
const MENU_WIDTH: f32 = 190.0;

/// The height of one entry: a row, or the separator.
const fn entry_height(entry: &MenuEntry) -> f32 {
    if entry.is_separator() {
        MENU_SEPARATOR
    } else {
        MENU_ROW
    }
}

/// The height of a column of entries: its padding, its border, and each row or separator.
fn column_height(entries: &[MenuEntry]) -> f32 {
    2.0 * MENU_PADDING + 2.0 + entries.iter().map(entry_height).sum::<f32>()
}

/// The height of [`MAP_MENU`]'s column as it opened: without Geo-Fence and Rally Points over a
/// vehicle with `MISSION_FENCE`.
fn map_menu_height(mission_fence: bool) -> f32 {
    2.0 * MENU_PADDING
        + 2.0
        + MAP_MENU
            .iter()
            .filter(|entry| shown(entry, mission_fence))
            .map(entry_height)
            .sum::<f32>()
}

/// Where a drop-down of `height` starts, relative to the menu's top, when the menu's top is at
/// `menu_top` in a window `viewport_height` tall and the entry that opens it starts `offset`
/// down the menu.
///
/// It lines up with its entry when there is room below, and otherwise moves up just enough to
/// fit, as a `ToolStripDropDown` does. What it never does is move the menu: the menu and the
/// drop-down used to be one snapped body, so a long drop-down opening near the bottom of the
/// window pushed the whole menu up, the pointer that had just clicked "Map Tool" found itself on
/// "File Load/Save", and that entry's drop-down replaced the one the click had opened.
/// `tests/gui/plan-reverse-wps.gui` found it.
fn dropdown_top(menu_top: f32, offset: f32, height: f32, viewport_height: f32) -> f32 {
    let aligned = offset - MENU_PADDING;
    let lowest = viewport_height - height - menu_top;
    aligned.min(lowest).max(-menu_top)
}

/// How far down the menu a top-level entry starts, the entries hidden above it not counted.
fn menu_offset(index: usize, mission_fence: bool) -> f32 {
    MENU_PADDING
        + MAP_MENU
            .iter()
            .take(index)
            .filter(|entry| shown(entry, mission_fence))
            .map(entry_height)
            .sum::<f32>()
}

/// One entry, as a `ToolStripMenuItem` draws: its text, an arrow if it has a drop-down, dimmed if
/// it does nothing here.
fn menu_row(
    entry: &'static MenuEntry,
    top_level: Option<usize>,
    enabled: bool,
    highlighted: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if entry.is_separator() {
        return div()
            .h(px(MENU_SEPARATOR))
            .flex()
            .items_center()
            .px_2()
            .child(div().h(px(1.0)).w_full().bg(rgb(theme::BORDER)))
            .into_any_element();
    }
    let row = crate::probe::measured(entry.id, div())
        .id(entry.id)
        .h(px(MENU_ROW))
        .px_3()
        .flex()
        .items_center()
        .justify_between()
        .gap_4()
        .text_xs()
        .child(entry.text)
        .children((!entry.children.is_empty()).then_some("›"));
    if !enabled {
        return row.text_color(rgb(theme::DIM)).into_any_element();
    }
    let row = row
        .text_color(rgb(theme::TEXT))
        .bg(rgb(if highlighted {
            theme::ACTION
        } else {
            theme::PANEL
        }))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)));
    // A top-level entry shows its drop-down while the pointer is on it, and hides any other, as
    // a ToolStrip does; an entry inside a drop-down leaves it be.
    let row = match top_level {
        Some(index) => {
            let opens = (!entry.children.is_empty()).then_some(index);
            row.on_hover(cx.listener(move |this, hovered: &bool, _window, cx| {
                if *hovered {
                    this.plan_menus.show_submenu(opens);
                    cx.notify();
                }
            }))
        }
        None => row,
    };
    let action = entry.action;
    let opens = top_level.filter(|_| !entry.children.is_empty());
    row.on_click(cx.listener(move |this, _event, window, cx| {
        match action {
            Some(action) => choose_entry(this, action, window, cx),
            None => {
                // An entry that only opens a drop-down opens it on a click too.
                this.plan_menus.show_submenu(opens);
                cx.notify();
            }
        }
    }))
    .into_any_element()
}

/// A column of entries: the menu, or a drop-down.
fn menu_column(children: Vec<AnyElement>) -> gpui::Div {
    div()
        .flex()
        .flex_col()
        .w(px(MENU_WIDTH))
        .py(px(MENU_PADDING))
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .children(children)
}

/// `contextMenuStrip1`, drawn where the right button came up, above everything else.
fn map_menu(
    menus: &PlanMenus,
    window: &gpui::Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let menu = menus.open?;
    let delete_enabled = menus.delete_enabled();
    // Where the snapped menu ends up: `snap_to_window` moves a top-left anchored element up and
    // left just enough to fit, which with fixed sizes is this arithmetic.
    let viewport = window.viewport_size();
    let (viewport_width, viewport_height) = (f32::from(viewport.width), f32::from(viewport.height));
    // ---- row 96 ----
    #[allow(clippy::cast_precision_loss)]
    let plugin_height = MENU_ROW * menus.plugin_entries.len() as f32;
    let menu_height = map_menu_height(menu.mission_fence) + plugin_height;
    // ---- end row 96 ----
    let menu_left = menu.at.0.min(viewport_width - MENU_WIDTH).max(0.0);
    let menu_top = menu.at.1.min(viewport_height - menu_height).max(0.0);
    // The drop-down's rectangle in window coordinates, so a press on it is not a press outside
    // the menu. `None` while no drop-down shows.
    let dropdown = menu
        .submenu
        .and_then(|index| Some((index, MAP_MENU.get(index)?)))
        .map(|(index, entry)| {
            let height = column_height(entry.children);
            let top = dropdown_top(
                menu_top,
                menu_offset(index, menu.mission_fence),
                height,
                viewport_height,
            );
            (top, height)
        });
    let dropdown_contains = move |(x, y): (f32, f32)| {
        dropdown.is_some_and(|(top, height)| {
            let left = menu_left + MENU_WIDTH;
            let above = menu_top + top;
            (left..=left + MENU_WIDTH).contains(&x) && (above..=above + height).contains(&y)
        })
    };
    let mut rows: Vec<AnyElement> = MAP_MENU
        .iter()
        .enumerate()
        .filter(|(_, entry)| shown(entry, menu.mission_fence))
        .map(|(index, entry)| {
            let enabled =
                entry.is_live() && (entry.action != Some(MenuAction::DeleteWp) || delete_enabled);
            menu_row(entry, Some(index), enabled, menu.submenu == Some(index), cx)
        })
        .collect();
    // ---- row 96 ----
    // `Host.FPMenuMap.Items.Add`: a plugin's entries at the end, as `Items.Add` puts them.
    rows.extend(menus.plugin_entries.iter().map(|entry| {
        let (plugin, id, position) = (entry.plugin, entry.id, menu.position);
        crate::probe::measured(entry.probe_id(), div())
            .id(entry.probe_id())
            .h(px(MENU_ROW))
            .px_3()
            .flex()
            .items_center()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .bg(rgb(theme::PANEL))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(entry.label())
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.plan_menus.open = None;
                this.plan_menus.plugin_click = Some((plugin, id, position));
                cx.notify();
            }))
            .into_any_element()
    }));
    // ---- end row 96 ----
    let mut body = div()
        .id("plan-menu")
        .flex()
        .items_start()
        .occlude()
        .on_mouse_down_out(
            cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                let at = (f32::from(event.position.x), f32::from(event.position.y));
                // The drop-down sits outside the body's own bounds, so a press on it arrives
                // here as "outside"; it is not.
                if dropdown_contains(at) {
                    return;
                }
                this.plan_menus.dismiss(at);
                cx.notify();
            }),
        )
        .child(menu_column(rows));
    if let Some(((top, _), entry)) =
        dropdown.zip(menu.submenu.and_then(|index| MAP_MENU.get(index)))
    {
        let rows = entry
            .children
            .iter()
            .map(|child| menu_row(child, None, child.is_live(), false, cx))
            .collect();
        // Absolutely positioned, so opening it changes nothing about the body the window snaps,
        // and the menu stays where the pointer is.
        body = body.child(
            div()
                .absolute()
                .left(px(MENU_WIDTH))
                .top(px(top))
                .occlude()
                .child(menu_column(rows)),
        );
    }
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(menu.at.0), px(menu.at.1)))
                .snap_to_window()
                .child(body),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// The dialog, centred over the window with everything behind it inert, as `ShowDialog` makes it.
fn prompt_dialog(
    menus: &PlanMenus,
    focus: &gpui::FocusHandle,
    window: &gpui::Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let prompt = menus.prompt.as_ref()?;
    let focused = focus.is_focused(window);
    let size = window.viewport_size();

    // `CreateMessageShowAgainForm`'s buttons are OK, and Cancel when `show_cancel`.
    // `// C#: Common.cs:412-426`
    let (accept, refuse) = if prompt.show_again.is_some() {
        ("OK", prompt.is_question().then_some("Cancel"))
    } else if prompt.is_question() {
        ("Yes", Some("No"))
    } else if prompt.field.is_some() {
        ("OK", Some("Cancel"))
    } else {
        ("OK", None)
    };
    let buttons = div()
        .flex()
        .justify_end()
        .gap_2()
        .children(prompt.show_again.map(|again| {
            // "Show me again?", ticked to start, at the left of the buttons; its click writes
            // the setting at once. `// C#: Common.cs:390-400, 446-449`
            crate::probe::measured("plan-prompt-again", div())
                .id("plan-prompt-again")
                .flex()
                .flex_1()
                .items_center()
                .gap_1()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .child(
                    div()
                        .size(px(12.0))
                        .rounded_sm()
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .flex()
                        .items_center()
                        .justify_center()
                        .children(
                            again
                                .ticked
                                .then(|| div().size(px(7.0)).bg(rgb(theme::ACCENT))),
                        ),
                )
                .child(crate::config::adsb::SHOW_ME_AGAIN)
                .on_click(cx.listener(|this, _event: &gpui::ClickEvent, _window, cx| {
                    if let Some((key, ticked)) = this.plan_menus.toggle_show_again() {
                        this.persisted
                            .set(key, if ticked { "True" } else { "False" });
                    }
                    cx.notify();
                }))
        }))
        .child(action(
            "plan-prompt-ok",
            accept,
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), window, cx| submit_prompt(this, window, cx)),
        ))
        .children(refuse.map(|label| {
            action(
                "plan-prompt-cancel",
                label,
                theme::TEXT,
                true,
                cx.listener(|this, _event: &(), window, cx| cancel_prompt(this, window, cx)),
            )
        }));

    let on_key = cx.listener(|this, event: &gpui::KeyDownEvent, window, cx| {
        let outcome = match this.plan_menus.prompt.as_mut() {
            Some(Prompt {
                field: Some(field), ..
            }) => field.key(event),
            // A question or a message has no field; Enter and Escape still answer it.
            Some(_) => match event.keystroke.key.as_str() {
                "enter" => crate::textfield::KeyOutcome::Submitted,
                "escape" => crate::textfield::KeyOutcome::Cancelled,
                _ => crate::textfield::KeyOutcome::Ignored,
            },
            None => return,
        };
        match outcome {
            crate::textfield::KeyOutcome::Submitted => submit_prompt(this, window, cx),
            crate::textfield::KeyOutcome::Cancelled => cancel_prompt(this, window, cx),
            crate::textfield::KeyOutcome::Changed => cx.notify(),
            crate::textfield::KeyOutcome::Ignored => {}
        }
    });

    let mut dialog = crate::probe::measured("plan-prompt", div())
        .flex()
        .flex_col()
        .gap_2()
        // A file question is half again as wide, for the path it takes (the owner, 2026-09-25).
        .w(px(if prompt.is_file_dialog() {
            510.0
        } else {
            340.0
        }))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(prompt.title.to_owned()),
        )
        .child(
            div()
                .flex()
                .flex_col()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .children(prompt.text.lines().map(ToOwned::to_owned)),
        );
    let body = match &prompt.field {
        Some(field) => {
            dialog = dialog.child(crate::textfield::text_field(
                "plan-prompt-field",
                field,
                focus,
                focused,
                px(310.0),
                on_key,
            ));
            if prompt.is_file_dialog() {
                dialog = dialog.child(file_list(prompt, &menus.dialog_directory, cx));
            }
            dialog.child(buttons).into_any_element()
        }
        None => dialog
            .child(buttons)
            .id("plan-prompt-keys")
            .track_focus(focus)
            .on_key_down(on_key)
            .into_any_element(),
    };

    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("plan-prompt-backdrop")
                        .w(size.width)
                        .h(size.height)
                        .flex()
                        .items_center()
                        .justify_center()
                        .occlude()
                        .child(body),
                ),
        )
        .with_priority(2)
        .into_any_element(),
    )
}

/// A file dialog's list of what its folder holds, under its box, as an `OpenFileDialog` shows
/// the files to choose from (the owner, 2026-10-04): the folder, then a row per entry - `..`, the
/// folders, the files of the dialog's filter. A click puts the entry in the box, a folder's so
/// the list opens it next; a double click on a file takes it.
fn file_list(prompt: &Prompt, directory: &Path, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (folder, entries) = dialog_listing(prompt.value(), directory, prompt.file_types());
    let rows: Vec<AnyElement> = entries
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            let answer = dialog_answer(entry, directory);
            let is_folder = entry.folder;
            crate::probe::measured(format!("plan-file-{index}"), div())
                .id(("plan-file", index))
                .px_2()
                .text_xs()
                .cursor_pointer()
                .text_color(rgb(if is_folder { theme::ACCENT } else { theme::TEXT }))
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(if is_folder {
                    format!("{}/", entry.name)
                } else {
                    entry.name.clone()
                })
                .on_click(cx.listener(move |this, event: &gpui::ClickEvent, window, cx| {
                    if let Some(field) = this
                        .plan_menus
                        .prompt
                        .as_mut()
                        .and_then(|prompt| prompt.field.as_mut())
                    {
                        field.set(answer.clone());
                    }
                    this.plan_prompt_focus.focus(window, cx);
                    if !is_folder && event.click_count() >= 2 {
                        submit_prompt(this, window, cx);
                    }
                    cx.notify();
                }))
                .into_any_element()
        })
        .collect();
    div()
        .flex()
        .flex_col()
        .gap_1()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(format!("in {}", folder.display())),
        )
        .child(
            div()
                .id("plan-file-list")
                .flex()
                .flex_col()
                .max_h(px(220.0))
                .overflow_y_scroll()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .rounded_sm()
                .children(rows),
        )
        .into_any_element()
}

/// The menu and the dialog, for the planning screen to draw over itself. `fence_mode` is
/// whether the geofence is being drawn, which decides the polygon icon menu's two fence entries.
pub fn overlays(
    menus: &PlanMenus,
    fence_mode: bool,
    focus: &gpui::FocusHandle,
    window: &gpui::Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    map_menu(menus, window, cx)
        .into_iter()
        .chain(zoom_menu(menus, cx))
        .chain(poly_menu(menus, fence_mode, cx))
        .chain(elevation_form(menus, window, cx))
        .chain(prompt_dialog(menus, focus, window, cx))
        .collect()
}

/// The chart's box inside `zg1`, left, top, right and bottom margins: room for the pane title and
/// legend above it, the Y axis's labels and title left of it, the X axis's below it.
const CHART_MARGINS: (f32, f32, f32, f32) = (70.0, 58.0, 24.0, 48.0);
/// `zg1`'s size, and where it sits in the form: `(12, 23)`, 810 by 427.
/// `// C#: Controls/ElevationProfile.Designer.cs:36-49`
const ZG1: (f32, f32, f32, f32) = (12.0, 23.0, 810.0, 427.0);
/// The form's `ClientSize`, 834 by 462.
/// `// C#: Controls/ElevationProfile.Designer.cs:66`
const ELEVATION_CLIENT: (f32, f32) = (834.0, 462.0);

/// Map Tool > Elevation Graph's form, `ElevationProfile`, drawn over the planning screen as the
/// C#'s `ShowDialog` holds it over the planner: its caption and close box, `label1`'s note, and
/// `zg1` - the pane's title and legend, the X axis's major grid and the Y axis's zero line, the
/// "Planned Path" in red over the "DEM" in blue, each planned point's tag in white, and the axes
/// with their titles, the Y axis's in red as `CreateChart` colours it.
///
/// Two differences, both gpui's: a tag is written level rather than turned 90 degrees, and so is
/// the Y axis title, above the axis, as gpui draws no turned text.
/// `// C#: Controls/ElevationProfile.Designer.cs; Controls/ElevationProfile.cs:278-340`
fn elevation_form(
    menus: &PlanMenus,
    window: &gpui::Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    use elevation::{BLUE, RED};
    let profile = menus.elevation.clone()?;
    let size = window.viewport_size();
    let (left, top, right, bottom) = CHART_MARGINS;
    let (zg_left, zg_top, zg_width, zg_height) = ZG1;
    let chart_width = zg_width - left - right;
    let chart_height = zg_height - top - bottom;
    // Fractions of the chart box, narrowed once for the layout.
    #[allow(clippy::cast_possible_truncation)]
    let across = |value: f64| profile.x_axis.fraction(value) as f32;
    #[allow(clippy::cast_possible_truncation)]
    let up = |value: f64| profile.y_axis.fraction(value) as f32;

    let text = |content: String, colour: u32| {
        div()
            .absolute()
            .text_xs()
            .text_color(rgb(colour))
            .child(content)
    };
    let mut zg1 = crate::probe::measured("plan-elevation-chart", div())
        .absolute()
        .left(px(zg_left))
        .top(px(zg_top))
        .w(px(zg_width))
        .h(px(zg_height))
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        // The pane's title, centred at the top.
        .child(
            div()
                .absolute()
                .top(px(4.0))
                .left_0()
                .right_0()
                .flex()
                .justify_center()
                .text_base()
                .font_weight(gpui::FontWeight::BOLD)
                .text_color(rgb(theme::TEXT))
                .child(elevation::TITLE),
        )
        // The legend, under the title: each curve's line and its label.
        .child(
            div()
                .absolute()
                .top(px(28.0))
                .left_0()
                .right_0()
                .flex()
                .justify_center()
                .gap_4()
                .children(
                    [(elevation::PLANNED_PATH, RED), (elevation::DEM, BLUE)].map(
                        |(label, colour)| {
                            div()
                                .flex()
                                .items_center()
                                .gap_1()
                                .child(div().w(px(24.0)).h(px(2.0)).bg(rgb(colour)))
                                .child(div().text_xs().text_color(rgb(theme::TEXT)).child(label))
                        },
                    ),
                ),
        );

    // The Y axis title, red, above the axis; the X axis title under its labels.
    zg1 = zg1
        .child(
            text(profile.y_axis.title(elevation::Y_TITLE), RED)
                .left(px(4.0))
                .top(px(top - 16.0)),
        )
        .child(
            div()
                .absolute()
                .left(px(left))
                .w(px(chart_width))
                .top(px(top + chart_height + 22.0))
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(profile.x_axis.title(elevation::X_TITLE)),
        );
    // The scales' labels: the X axis's centred under each tic, the Y axis's in red, flush to the
    // axis (`AlignP.Inside`).
    for tic in profile.x_axis.tics() {
        zg1 = zg1.child(
            div()
                .absolute()
                .left(px(left + across(tic) * chart_width - 30.0))
                .w(px(60.0))
                .top(px(top + chart_height + 4.0))
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(profile.x_axis.label(tic)),
        );
    }
    for tic in profile.y_axis.tics() {
        zg1 = zg1.child(
            div()
                .absolute()
                .left_0()
                .w(px(left - 6.0))
                .top(px(top + (1.0 - up(tic)) * chart_height - 7.0))
                .flex()
                .justify_end()
                .text_xs()
                .text_color(rgb(RED))
                .child(profile.y_axis.label(tic)),
        );
    }

    // The lines: grid, zero line, border, then the curves - the DEM first, as ZedGraph draws the
    // last curve added first - clipped to the chart.
    let lines = profile.clone();
    zg1 = zg1.child(
        gpui::canvas(
            |_bounds, _window, _cx| (),
            move |bounds, (), window, _cx| {
                paint_elevation(&lines, bounds, window);
            },
        )
        .absolute()
        .left(px(left))
        .top(px(top))
        .w(px(chart_width))
        .h(px(chart_height)),
    );
    // Each planned point's tag, white, its end at the point (`AlignH.Right`, `AlignV.Center`,
    // turned 90 degrees in the C#: the text runs down from the point).
    for point in &profile.planned {
        let Some(tag) = &point.tag else {
            continue;
        };
        zg1 = zg1.child(
            div()
                .absolute()
                .left(px(left + across(point.x) * chart_width - 20.0))
                .w(px(40.0))
                .top(px(top + (1.0 - up(point.y)) * chart_height + 2.0))
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(0xff_ff_ff))
                .child(tag.clone()),
        );
    }

    let (client_width, client_height) = ELEVATION_CLIENT;
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(elevation::FORM_TEXT),
        )
        .child(action(
            "plan-elevation-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.plan_menus.elevation = None;
                cx.notify();
            }),
        ));
    let client = div()
        .relative()
        .w(px(client_width))
        .h(px(client_height))
        .child(
            text(elevation::NOTE.to_owned(), theme::TEXT)
                .left(px(162.0))
                .top(px(7.0)),
        )
        .child(zg1);
    let form = crate::probe::measured("plan-elevation", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);

    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(
                    div()
                        .id("plan-elevation-backdrop")
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
        .into_any_element(),
    )
}

/// The Elevation Graph's lines, in the chart's box: the X axis's major grid (ZedGraph's dotted
/// grey), the Y axis's zero line when 0 is inside it, the chart's border, and the two curves,
/// clipped to the box as ZedGraph clips them.
/// `// C#: Controls/ElevationProfile.cs:289-322; ExtLibs/ZedGraph/ZedGraph/Scale.cs:2262-2270`
fn paint_elevation(
    profile: &elevation::ElevationProfile,
    bounds: gpui::Bounds<gpui::Pixels>,
    window: &mut gpui::Window,
) {
    let width = f32::from(bounds.size.width);
    let height = f32::from(bounds.size.height);
    #[allow(clippy::cast_possible_truncation)]
    let at = |x: f64, y: f64| {
        gpui::point(
            bounds.origin.x + px(profile.x_axis.fraction(x) as f32 * width),
            bounds.origin.y + px((1.0 - profile.y_axis.fraction(y) as f32) * height),
        )
    };
    let stroke = |window: &mut gpui::Window,
                  points: &[gpui::Point<gpui::Pixels>],
                  colour: u32,
                  dashed: bool| {
        let mut builder = gpui::PathBuilder::stroke(px(1.0));
        if dashed {
            builder = builder.dash_array(&[px(1.0), px(5.0)]);
        }
        let mut points = points.iter();
        let Some(first) = points.next() else {
            return;
        };
        builder.move_to(*first);
        for point in points {
            builder.line_to(*point);
        }
        if let Ok(path) = builder.build() {
            window.paint_path(path, gpui::Hsla::from(rgb(colour)));
        }
    };
    let (y_min, y_max) = (profile.y_axis.min, profile.y_axis.max);
    for tic in profile.x_axis.tics() {
        stroke(window, &[at(tic, y_min), at(tic, y_max)], theme::DIM, true);
    }
    if y_min < 0.0 && y_max > 0.0 {
        let (x_min, x_max) = (profile.x_axis.min, profile.x_axis.max);
        stroke(window, &[at(x_min, 0.0), at(x_max, 0.0)], theme::DIM, false);
    }
    let corner = |x: f32, y: f32| {
        gpui::point(
            bounds.origin.x + px(x * width),
            bounds.origin.y + px(y * height),
        )
    };
    stroke(
        window,
        &[
            corner(0.0, 0.0),
            corner(1.0, 0.0),
            corner(1.0, 1.0),
            corner(0.0, 1.0),
            corner(0.0, 0.0),
        ],
        theme::BORDER,
        false,
    );
    window.with_content_mask(Some(gpui::ContentMask { bounds }), |window| {
        for (curve, colour) in [
            (&profile.dem, elevation::BLUE),
            (&profile.planned, elevation::RED),
        ] {
            let points: Vec<_> = curve.iter().map(|point| at(point.x, point.y)).collect();
            stroke(window, &points, colour, false);
        }
    });
}

/// What a `.waypoints` text holds, for the facts: every record's `seq:frame:command`, and record
/// 0's latitude, longitude and altitude as the file has them.
#[must_use]
pub fn written_facts(file: &str) -> (String, String) {
    let records: Vec<Vec<&str>> = file
        .lines()
        .skip(1)
        .map(|line| line.split('\t').collect())
        .collect();
    fn field<'a>(fields: &[&'a str], index: usize) -> &'a str {
        fields.get(index).copied().unwrap_or("")
    }
    let summary = records
        .iter()
        .map(|fields| {
            format!(
                "{}:{}:{}",
                field(fields, 0),
                field(fields, 2),
                field(fields, 3)
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    let home = records
        .first()
        .and_then(|fields| fields.get(8..11))
        .map_or_else(String::new, |position| position.join(","));
    (summary, home)
}

/// Facts a UI test asserts on for this screen: what the mission holds, item by item, and what the
/// menu and its dialogs are doing.
pub fn record_facts(plan: &Plan, menus: &PlanMenus) {
    use crate::facts::record;
    let items = plan.items();
    record(
        "mission.commands",
        items
            .iter()
            .map(|item| item.command.to_string())
            .collect::<Vec<_>>()
            .join(","),
    );
    // Home: whether the Home Location boxes hold one a write would take, and what they hold.
    record("mission.home", plan.home().is_some());
    record(
        "plan.home",
        HomeBox::ALL
            .iter()
            .map(|which| plan.home_text(*which))
            .collect::<Vec<_>>()
            .join(","),
    );
    // What Save File would write, read from the text it would write, so a test proves home is
    // item 0 and the first row item 1 on the path the product takes.
    let (written, written_home) = written_facts(&plan.waypoints_file());
    record("mission.written", written);
    record("mission.written.home", written_home);
    record(
        "mission.selected",
        plan.selected()
            .map_or_else(|| "none".to_owned(), |seq| seq.to_string()),
    );
    let last = items.last();
    record(
        "mission.last.command",
        last.map_or_else(|| "none".to_owned(), |item| item.command.to_string()),
    );
    record("mission.last.p1", last.map_or(0.0, |item| item.param1));
    record("mission.last.p2", last.map_or(0.0, |item| item.param2));
    record("mission.last.alt", last.map_or(0.0, |item| item.z));
    record(
        "mission.last.position",
        last.and_then(|item| item.position().ok().flatten())
            .is_some(),
    );
    record("survey.points", plan.polygon().len());
    // KML Overlay: what is on the planner's map, and whether the flight screen's has it too.
    let kml = plan.kml_overlay();
    record("plan.kml.polygons", kml.map_or(0, |kml| kml.polygons.len()));
    record("plan.kml.routes", kml.map_or(0, |kml| kml.routes.len()));
    record("plan.kml.labels", kml.map_or(0, |kml| kml.labels.len()));
    record("plan.kml.ground", kml.map_or(0, |kml| kml.ground_overlays));
    record("plan.kml.flight", plan.kml_on_flight());
    record("plan.fence.exclusions", plan.fence_exclusions().len());
    record("plan.grid", plan.grid());
    record("plan.mavftp", plan.use_mavftp());
    record(
        "plan.mavftp.state",
        plan.mission_ftp().map_or("idle", MissionFtp::state_name),
    );
    record(
        "plan.mavftp.text",
        plan.mission_ftp()
            .map_or_else(String::new, MissionFtp::label),
    );
    record("plan.commands.minimised", plan.commands_minimised());
    record("plan.write.home_requests", plan.home_requests);
    record("plan.coords.system", plan.coords().system.name());
    record("plan.coords.lines", plan.coords().lines().join("|"));
    record("plan.coords.source", plan.coords().alt_source);
    record(
        "plan.fence.exclusion.points",
        plan.fence_exclusions()
            .iter()
            .map(|polygon| polygon.len().to_string())
            .collect::<Vec<_>>()
            .join(","),
    );
    // The corners the map shows, which Geo-Fence > Clear takes off it while keeping them.
    record("survey.shown", plan.shown_polygon().len());
    // The drawn polygon's corners, `lat,lng` each and `;` between: what Offset Polygon, Load
    // Polygon and From SHP left.
    record(
        "plan.polygon",
        plan.polygon()
            .iter()
            .map(|corner| format!("{},{}", corner.latitude(), corner.longitude()))
            .collect::<Vec<_>>()
            .join(";"),
    );
    // The rally markers, `lat,lng,alt` each; `MAV.rallypoints`' count, what Download fills; and
    // Upload - `busy` while its calls are out, then what each did.
    record(
        "plan.rally.markers",
        plan.rally_markers()
            .iter()
            .map(|(at, alt)| format!("{},{},{alt}", at.latitude(), at.longitude()))
            .collect::<Vec<_>>()
            .join(";"),
    );
    record("plan.rally.vehicle", plan.vehicle_rally().len());
    record("plan.rally.downloading", plan.rally_downloading());
    record(
        "plan.rally.upload",
        if plan.rally_upload().is_some() {
            "busy"
        } else if plan.rally_upload_results.is_empty() {
            "none"
        } else {
            plan.rally_upload_results.as_str()
        },
    );
    // The panel boxes as they read, whether Loiter Radius takes typing and Spline is ticked, and
    // what the last row of parameter sets did - `NAME=value` for each the vehicle echoed,
    // `unknown` for one it does not have - with whether one is still going.
    record("plan.wprad", plan.panel_text(PanelBox::WpRadius));
    record("plan.loiterrad", plan.panel_text(PanelBox::LoiterRadius));
    record("plan.defaultalt", plan.panel_text(PanelBox::DefaultAlt));
    record("plan.loiterrad.enabled", plan.loiter_enabled());
    record("plan.spline", plan.spline());
    record(
        "plan.params",
        if plan.write_results().is_empty() {
            "none"
        } else {
            plan.write_results()
        },
    );
    record(
        "plan.params.busy",
        plan.writes().is_some() || plan.pending_write.is_some(),
    );
    // Geo-Fence > Upload and Download: what each last said - the progress reporter's text while
    // an upload runs, "busy" while a download does - the upload's calls, and the geofence and its
    // return location as `lat,lng`, `;` between corners.
    record("plan.fence.upload", plan.fence_upload_text());
    record(
        "plan.fence.upload.calls",
        if plan.fence_upload_results.is_empty() {
            "none"
        } else {
            plan.fence_upload_results.as_str()
        },
    );
    record("plan.fence.download", plan.fence_download_text());
    record(
        "plan.fence.points",
        plan.fence()
            .iter()
            .map(|corner| format!("{},{}", corner.latitude(), corner.longitude()))
            .collect::<Vec<_>>()
            .join(";"),
    );
    record(
        "plan.fence.return",
        plan.fence_return().map_or_else(
            || "none".to_owned(),
            |at| format!("{},{}", at.latitude(), at.longitude()),
        ),
    );
    // The geofence's return location, as `latitude,longitude`.
    record(
        "fence.return",
        plan.fence_return().map_or_else(
            || "none".to_owned(),
            |at| format!("{},{}", at.latitude(), at.longitude()),
        ),
    );
    record("plan.draw", plan.draw_mode().id());
    record(
        "plan.menu",
        if menus.open.is_some() {
            "open"
        } else {
            "closed"
        },
    );
    record(
        "plan.menu.submenu",
        menus
            .open
            .and_then(|menu| menu.submenu)
            .and_then(|index| MAP_MENU.get(index))
            .map_or("none", |entry| entry.id),
    );
    record("plan.menu.delete", menus.delete_enabled());
    record(
        "plan.menu.hidden",
        match menus.open {
            Some(menu) if menu.mission_fence => HIDDEN_ON_MISSION_FENCE.join(","),
            _ => "none".to_owned(),
        },
    );
    record(
        "plan.prompt",
        menus.prompt.as_ref().map_or("none", |prompt| {
            if prompt.title.is_empty() {
                "message"
            } else {
                prompt.title
            }
        }),
    );
    record(
        "plan.prompt.text",
        menus
            .prompt
            .as_ref()
            .map_or("", |prompt| prompt.text.as_str()),
    );
    record(
        "plan.prompt.value",
        menus.prompt.as_ref().map_or("", Prompt::value),
    );
    // A file dialog's list: its folder and how many rows it shows.
    let listing = menus
        .prompt
        .as_ref()
        .filter(|prompt| prompt.is_file_dialog())
        .map(|prompt| {
            dialog_listing(prompt.value(), &menus.dialog_directory, prompt.file_types())
        });
    record(
        "plan.prompt.folder",
        listing
            .as_ref()
            .map_or_else(|| "none".to_owned(), |(folder, _)| folder.display().to_string()),
    );
    record(
        "plan.prompt.files",
        listing.as_ref().map_or_else(
            || "none".to_owned(),
            |(_, entries)| {
                entries
                    .iter()
                    .map(|entry| entry.name.as_str())
                    .collect::<Vec<_>>()
                    .join(",")
            },
        ),
    );
    record(
        "plan.prompt.showagain",
        menus
            .prompt
            .as_ref()
            .and_then(|prompt| prompt.show_again)
            .map_or(
                "none",
                |again| if again.ticked { "ticked" } else { "unticked" },
            ),
    );
    record("plan.measure", menus.measure_from.is_some());
    // Terrain: the lookup's access mode, Verify Height, `sethome`, every row's altitude, and the
    // Elevation Graph's form.
    record("plan.srtm.cacheonly", crate::srtm::cache_only());
    record("plan.verifyheight", plan.verify_height());
    record("plan.sethome", plan.sethome());
    record(
        "mission.alts",
        items
            .iter()
            .map(|item| item.z.to_string())
            .collect::<Vec<_>>()
            .join(","),
    );
    for (key, value) in elevation::facts(menus.elevation.as_ref()) {
        record(key, value);
    }
}

// ---------------------------------------------------------------------------------------------
// Map Tool > Elevation Graph: `Controls/ElevationProfile.cs`, the form the C# shows as a dialog,
// drawn here over the planning screen, with the part of ZedGraph its chart uses.
// ---------------------------------------------------------------------------------------------

/// The Elevation Graph: `ElevationProfile`'s sampling of the terrain along the planned route, its
/// two curves, and the axes ZedGraph's `AxisChange` picks for them.
pub mod elevation {
    use super::{AltitudeFrame, MULTIPLIER_ALT, Terrain, c_sharp_int};
    use crate::mapview::PlanPoint;

    /// `CurrentState.multiplierdist`: 1, metres.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:27`
    const MULTIPLIER_DIST: f32 = crate::fly::MULTIPLIER_DIST;

    /// What the form says with one point or none, and does not open.
    /// `// C#: Controls/ElevationProfile.cs:44-48, 80-84`
    pub const PLAN_SOMETHING: &str = "Please plan something first";
    /// The form's `Text`, its caption.
    /// `// C#: Controls/ElevationProfile.Designer.cs:69`
    pub const FORM_TEXT: &str = "ElevationProfile";
    /// `label1`, above the chart.
    /// `// C#: Controls/ElevationProfile.Designer.cs:55-58`
    pub const NOTE: &str = "NOTE: The ground height data is pulled from Google Earth at 100m intervals. You use this at your own risk";
    /// The pane's title.
    /// `// C#: Controls/ElevationProfile.cs:283`
    pub const TITLE: &str = "Elevation above ground";
    /// The X axis title, `"Distance (" + CurrentState.DistanceUnit + ")"`, in metres.
    /// `// C#: Controls/ElevationProfile.cs:284`
    pub const X_TITLE: &str = "Distance (m)";
    /// The Y axis title, `"Elevation (" + CurrentState.AltUnit + ")"`, in metres.
    /// `// C#: Controls/ElevationProfile.cs:285`
    pub const Y_TITLE: &str = "Elevation (m)";
    /// `list1`'s curve.
    pub const PLANNED_PATH: &str = "Planned Path";
    /// `list3`'s curve.
    pub const DEM: &str = "DEM";
    /// `Color.Red`: the planned path, the Y axis's scale and title.
    pub const RED: u32 = 0xff_00_00;
    /// `Color.Blue`: the DEM.
    pub const BLUE: u32 = 0x00_00_ff;

    /// One `PointPair`: where it is on the chart, and the `Tag` a planned point is labelled with.
    #[derive(Debug, Clone, PartialEq)]
    pub struct ChartPoint {
        /// Distance along the route, metres.
        pub x: f64,
        /// Height, metres.
        pub y: f64,
        /// The waypoint's tag - "H", "1", ... - on the planned path in the Relative and Absolute
        /// frames; none on the terrain-following curve or the DEM.
        pub tag: Option<String>,
    }

    /// The form's content once it has loaded.
    #[derive(Debug, Clone, PartialEq)]
    pub struct ElevationProfile {
        /// `list1`, "Planned Path", red.
        pub planned: Vec<ChartPoint>,
        /// `list3`, "DEM", blue: the ground every 10 m or so along the route.
        pub dem: Vec<ChartPoint>,
        /// `distance`: the route's length, each leg cut to whole metres before it is added -
        /// the X axis's maximum.
        pub distance: i32,
        /// The X axis after `AxisChange`.
        pub x_axis: Scale,
        /// The Y axis after `AxisChange`.
        pub y_axis: Scale,
    }

    /// `PointLatLngAlt.GetDistance`: a haversine on a 6371 km sphere, in the C#'s order.
    /// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
    fn get_distance(from: (f64, f64), to: (f64, f64)) -> f64 {
        let d = from.0 * 0.017_453_292_519_943_295;
        let num2 = from.1 * 0.017_453_292_519_943_295;
        let num3 = to.0 * 0.017_453_292_519_943_295;
        let num4 = to.1 * 0.017_453_292_519_943_295;
        let num5 = num4 - num2;
        let num6 = num3 - d;
        let num7 =
            (num6 / 2.0).sin().powi(2) + ((d.cos() * num3.cos()) * (num5 / 2.0).sin().powi(2));
        let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
        (6371.0 * num8) * 1000.0
    }

    /// `Convert.ToInt32(double)`: the nearest whole number, a half to the even one. (A value past
    /// an `int`, which throws in the C#, saturates here; no ground is that high.)
    #[must_use]
    pub fn convert_to_int32(value: f64) -> f64 {
        value
            .round_ties_even()
            .clamp(f64::from(i32::MIN), f64::from(i32::MAX))
    }

    impl ElevationProfile {
        /// `new ElevationProfile(pointlist, homealt, altmode)` and its `Load`: the planned points
        /// with the nulls and the ROIs taken out - "Please plan something first" unless two are
        /// left - then the terrain sampled along each leg as `getSRTMAltPath` samples it, the
        /// planned path, and the chart's axes.
        ///
        /// `getSRTMAltPath` cuts each leg into `(int)(dist / 10) + 1` steps, straight in latitude
        /// and longitude, and asks `srtm` at every step, both ends included, so each leg's first
        /// sample is the last leg's last again. Each sample's distance is added up from the one
        /// before, `disttotal`; "DEM" is the ground there and the terrain-following curve the
        /// ground plus the leg's altitude at that step, each `Convert.ToInt32`. In the Terrain
        /// frame every point's altitude first has the ground under it taken off - the planned
        /// altitudes above sea level made heights above the ground again - and the planned path
        /// is that terrain-following curve, untagged; otherwise it is the points themselves at
        /// their altitudes, which "already include the home alt", each tagged.
        ///
        /// (The C# takes the points off `FlightPlanner.pointlist` itself, and `writeKML` makes a
        /// new one on the next redraw; here the form has its own copy. Its `LoadingBox`, shown and
        /// closed while it samples, is not drawn: the sampling is done within the frame that
        /// chose the entry.)
        /// `// C#: Controls/ElevationProfile.cs:27-76, 78-139, 141-206`
        pub fn new(
            locs: Vec<Option<PlanPoint>>,
            altmode: AltitudeFrame,
            terrain: Terrain,
        ) -> Result<Self, &'static str> {
            let mut planlocs: Vec<PlanPoint> = locs
                .into_iter()
                .flatten()
                .filter(|loc| !loc.tag.contains("ROI"))
                .collect();
            if planlocs.len() <= 1 {
                return Err(PLAN_SOMETHING);
            }

            // get total distance
            let mut distance: i32 = 0;
            for (lastloc, loc) in planlocs.iter().zip(planlocs.iter().skip(1)) {
                distance = distance.saturating_add(c_sharp_int(get_distance(
                    (loc.lat, loc.lng),
                    (lastloc.lat, lastloc.lng),
                )));
            }

            // getSRTMAltPath
            let multiplierdist = f64::from(MULTIPLIER_DIST);
            let multiplieralt = f64::from(MULTIPLIER_ALT);
            let mut dem = Vec::new();
            let mut list4terrain = Vec::new();
            let mut disttotal = 0.0;
            let mut last: Option<PlanPoint> = None;
            for loc in &mut planlocs {
                let Some(prev) = last.take() else {
                    if altmode == AltitudeFrame::Terrain {
                        loc.alt -= terrain.at(loc.lat, loc.lng).alt;
                    }
                    last = Some(loc.clone());
                    continue;
                };
                let dist = get_distance((prev.lat, prev.lng), (loc.lat, loc.lng));
                if altmode == AltitudeFrame::Terrain {
                    loc.alt -= terrain.at(loc.lat, loc.lng).alt;
                }
                let points = c_sharp_int(dist / 10.0).saturating_add(1);
                let steplat = (prev.lat - loc.lat) / f64::from(points);
                let steplng = (prev.lng - loc.lng) / f64::from(points);
                let stepalt = (prev.alt - loc.alt) / f64::from(points);
                let mut lastpnt = (prev.lat, prev.lng);
                for a in 0..=points {
                    let a = f64::from(a);
                    let lat = prev.lat - steplat * a;
                    let lng = prev.lng - steplng * a;
                    let alt = prev.alt - stepalt * a;
                    let ground = terrain.at(lat, lng).alt;
                    disttotal += get_distance(lastpnt, (lat, lng));
                    // srtm alts
                    dem.push(ChartPoint {
                        x: disttotal * multiplierdist,
                        y: convert_to_int32(ground * multiplieralt),
                        tag: None,
                    });
                    // terrain alt
                    list4terrain.push(ChartPoint {
                        x: disttotal * multiplierdist,
                        y: convert_to_int32((ground + alt) * multiplieralt),
                        tag: None,
                    });
                    lastpnt = (lat, lng);
                }
                // `answer.Add(... srtm.getAltitude(loc) ...)`: the form keeps it as `srtmlocs`
                // and never reads it, but the lookup is made.
                let _ = terrain.at(loc.lat, loc.lng);
                last = Some(loc.clone());
            }

            // Load: the planner plot.
            let planned = if altmode == AltitudeFrame::Terrain {
                list4terrain
            } else {
                let mut a = 0.0;
                let mut lastloc: Option<&PlanPoint> = None;
                let mut planned = Vec::with_capacity(planlocs.len());
                for planloc in &planlocs {
                    if let Some(lastloc) = lastloc {
                        a += get_distance((planloc.lat, planloc.lng), (lastloc.lat, lastloc.lng));
                    }
                    planned.push(ChartPoint {
                        x: a * multiplierdist,
                        y: planloc.alt * multiplieralt,
                        tag: Some(planloc.tag.clone()),
                    });
                    lastloc = Some(planloc);
                }
                planned
            };

            // CreateChart: X from 0 to the distance; Y from the two curves.
            let x_axis = Scale::pick(0.0, 0.0, Some((0.0, f64::from(distance) * multiplierdist)));
            let (range_min, range_max) = planned
                .iter()
                .chain(dem.iter())
                .fold((f64::MAX, f64::MIN), |(low, high), point| {
                    (low.min(point.y), high.max(point.y))
                });
            let y_axis = Scale::pick(range_min, range_max, None);
            Ok(Self {
                planned,
                dem,
                distance,
                x_axis,
                y_axis,
            })
        }
    }

    /// A ZedGraph `LinearScale` after `AxisChange`: its range, its steps, and the power of ten
    /// and the decimals its labels are written with.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct Scale {
        /// `_min`.
        pub min: f64,
        /// `_max`.
        pub max: f64,
        /// `_majorStep`.
        pub major_step: f64,
        /// `_minorStep`.
        pub minor_step: f64,
        /// `_mag`: labels are the value over ten to this.
        pub mag: i32,
        /// The `"f"` format's decimals.
        pub decimals: usize,
    }

    /// `Scale.Default.MinGrace` and `MaxGrace`.
    const GRACE: f64 = 0.1;
    /// `Scale.Default.ZeroLever`.
    const ZERO_LEVER: f64 = 0.25;
    /// `Scale.Default.TargetXSteps` and `TargetYSteps`.
    const TARGET_STEPS: f64 = 7.0;
    /// `Scale.Default.TargetMinorXSteps` and `TargetMinorYSteps`.
    const TARGET_MINOR_STEPS: f64 = 5.0;

    /// `Scale.CalcStepSize`: the range over the target steps, rounded to 1, 2 or 5 times a power
    /// of ten.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2577-2598`
    fn calc_step_size(range: f64, target_steps: f64) -> f64 {
        let temp_step = range / target_steps;
        let mag = temp_step.log10().floor();
        let mag_pow = 10f64.powf(mag);
        let mut mag_msd = f64::from(c_sharp_int(temp_step / mag_pow + 0.5));
        if mag_msd > 5.0 {
            mag_msd = 10.0;
        } else if mag_msd > 2.0 {
            mag_msd = 5.0;
        } else if mag_msd > 1.0 {
            mag_msd = 2.0;
        }
        mag_msd * mag_pow
    }

    /// `Scale.MyMod`: the remainder, 0 for a divisor of 0.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2667-2676`
    fn my_mod(x: f64, y: f64) -> f64 {
        if y == 0.0 {
            return 0.0;
        }
        let temp = x / y;
        y * (temp - temp.floor())
    }

    impl Scale {
        /// `Scale.PickScale` then `LinearScale.PickScale` for an axis whose data run from
        /// `range_min` to `range_max`, or whose `Min` and `Max` were set (`fixed`), as
        /// `CreateChart` sets the X axis's: the grace either side, the zero lever, a major step
        /// of about seven to the range and a minor step of about five to that, the ends taken
        /// out to whole steps, and the magnitude and decimals of the labels.
        ///
        /// (`IsPreventLabelOverlap` would then widen the step until the labels' measured widths
        /// fit the chart. Not ported: it needs GDI+'s font metrics, and the chart here is wide
        /// enough for seven labels either way.)
        /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2394-2449, 2527-2560; ExtLibs/ZedGraph/ZedGraph/LinearScale.cs:133-188`
        #[must_use]
        pub fn pick(range_min: f64, range_max: f64, fixed: Option<(f64, f64)>) -> Self {
            let auto = fixed.is_none();
            let (mut min, mut max) = fixed.unwrap_or((0.0, 0.0));
            // Scale.PickScale
            let legitimate = |value: f64| {
                if value.is_infinite() || value.is_nan() || value == f64::MAX {
                    0.0
                } else {
                    value
                }
            };
            let min_val = legitimate(range_min);
            let max_val = legitimate(range_max);
            let range = max_val - min_val;
            if auto {
                min = min_val;
                if min < 0.0 || min_val - GRACE * range >= 0.0 {
                    min = min_val - GRACE * range;
                }
                max = max_val;
                if max > 0.0 || max_val + GRACE * range <= 0.0 {
                    max = max_val + GRACE * range;
                }
                if max == min {
                    if max.abs() > 1e-100 {
                        max *= if min < 0.0 { 0.95 } else { 1.05 };
                        min *= if min < 0.0 { 1.05 } else { 0.95 };
                    } else {
                        max = 1.0;
                        min = -1.0;
                    }
                }
                if max <= min {
                    max = min + 1.0;
                }
            }
            // LinearScale.PickScale
            if auto && max - min < 1.0e-30 {
                max += 0.2 * if max == 0.0 { 1.0 } else { max.abs() };
                min -= 0.2 * if min == 0.0 { 1.0 } else { min.abs() };
            }
            if auto && min > 0.0 && min / (max - min) < ZERO_LEVER {
                min = 0.0;
            }
            if auto && max < 0.0 && (max / (max - min)).abs() < ZERO_LEVER {
                max = 0.0;
            }
            let major_step = calc_step_size(max - min, TARGET_STEPS);
            let minor_step = calc_step_size(major_step, TARGET_MINOR_STEPS);
            if auto {
                min -= my_mod(min, major_step);
                let rest = my_mod(max, major_step);
                if rest != 0.0 {
                    max = max + major_step - rest;
                }
            }
            // SetScaleMag
            let mut mag = -100.0_f64;
            let mut mag2 = -100.0_f64;
            if min.abs() > 1.0e-30 {
                mag = min.abs().log10().floor();
            }
            if max.abs() > 1.0e-30 {
                mag2 = max.abs().log10().floor();
            }
            let mut mag = mag2.max(mag);
            if mag == -100.0 || mag.abs() <= 3.0 {
                mag = 0.0;
            }
            let mag = c_sharp_int((mag / 3.0).floor() * 3.0);
            let num_dec = 0 - c_sharp_int(major_step.log10().floor() - f64::from(mag));
            Self {
                min,
                max,
                major_step,
                minor_step,
                mag,
                decimals: usize::try_from(num_dec.max(0)).unwrap_or(0),
            }
        }

        /// The major tics `DrawLabels` draws: from the first whole step at or above the
        /// minimum (`CalcBaseTic`) to the maximum, allowing a thousandth of the range past it.
        /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:1941-1957, 2006-2040, 2629-2642`
        #[must_use]
        pub fn tics(&self) -> Vec<f64> {
            let step = self.major_step;
            if !step.is_finite() || step <= 0.0 || self.min >= self.max {
                return Vec::new();
            }
            let base = (self.min / step - 0.000_000_01).ceil() * step;
            let n_tics = c_sharp_int((self.max - self.min) / step + 0.01)
                .saturating_add(1)
                .clamp(1, 1000);
            let range_tol = (self.max - self.min) * 0.001;
            let first = c_sharp_int((self.min - base) / step + 0.99).max(0);
            let mut tics = Vec::new();
            for i in first..n_tics.saturating_add(first) {
                let value = base + step * f64::from(i);
                if value < self.min {
                    continue;
                }
                if value > self.max + range_tol {
                    break;
                }
                tics.push(value);
            }
            tics
        }

        /// A tic's label: `(dVal / Math.Pow(10, _mag)).ToString("f" + decimals)`.
        /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:1779-1789`
        #[must_use]
        pub fn label(&self, value: f64) -> String {
            // `+ 0.0` so that a tic at -0 is written "0", as .NET writes it.
            let shown = value / 10f64.powi(self.mag) + 0.0;
            format!("{shown:.*}", self.decimals)
        }

        /// The axis title with ZedGraph's magnitude, `" (10^3)"`, when the labels are scaled.
        /// `// C#: ExtLibs/ZedGraph/ZedGraph/Axis.cs:1371-1393`
        #[must_use]
        pub fn title(&self, text: &str) -> String {
            if self.mag == 0 {
                text.to_owned()
            } else {
                format!("{text} (10^{})", self.mag)
            }
        }

        /// Where a value sits across the axis, 0 at `min` and 1 at `max`.
        #[must_use]
        pub fn fraction(&self, value: f64) -> f64 {
            let span = self.max - self.min;
            if span <= 0.0 || !span.is_finite() {
                return 0.0;
            }
            (value - self.min) / span
        }
    }

    /// The facts a UI test reads of the form: whether it is showing, its curves, and its axes.
    #[must_use]
    pub fn facts(profile: Option<&ElevationProfile>) -> Vec<(&'static str, String)> {
        let Some(profile) = profile else {
            return vec![("plan.elevation", "closed".to_owned())];
        };
        let ys = |points: &[ChartPoint]| {
            points
                .iter()
                .map(|point| point.y.to_string())
                .collect::<Vec<_>>()
                .join(",")
        };
        let low = profile
            .dem
            .iter()
            .map(|point| point.y)
            .fold(f64::MAX, f64::min);
        let high = profile
            .dem
            .iter()
            .map(|point| point.y)
            .fold(f64::MIN, f64::max);
        let labels = |scale: &Scale| {
            scale
                .tics()
                .iter()
                .map(|tic| scale.label(*tic))
                .collect::<Vec<_>>()
                .join(",")
        };
        vec![
            ("plan.elevation", "open".to_owned()),
            (
                "plan.elevation.titles",
                format!(
                    "{TITLE}|{}|{}",
                    profile.x_axis.title(X_TITLE),
                    profile.y_axis.title(Y_TITLE)
                ),
            ),
            ("plan.elevation.legend", format!("{PLANNED_PATH},{DEM}")),
            ("plan.elevation.distance", profile.distance.to_string()),
            ("plan.elevation.planned", profile.planned.len().to_string()),
            ("plan.elevation.planned.y", ys(&profile.planned)),
            (
                "plan.elevation.planned.tags",
                profile
                    .planned
                    .iter()
                    .filter_map(|point| point.tag.clone())
                    .collect::<Vec<_>>()
                    .join(","),
            ),
            ("plan.elevation.dem", profile.dem.len().to_string()),
            ("plan.elevation.dem.range", format!("{low}..{high}")),
            ("plan.elevation.xlabels", labels(&profile.x_axis)),
            ("plan.elevation.ylabels", labels(&profile.y_axis)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three frames are the three MAV_FRAME values the C# enum names.
    #[test]
    fn the_altitude_frames_are_the_mav_frame_values_they_claim() {
        // C#: Relative = GLOBAL_RELATIVE_ALT (3), Absolute = GLOBAL (0),
        //     Terrain = GLOBAL_TERRAIN_ALT (10).
        assert_eq!(AltitudeFrame::Relative.mav_frame(), 3);
        assert_eq!(AltitudeFrame::Absolute.mav_frame(), 0);
        assert_eq!(AltitudeFrame::Terrain.mav_frame(), 10);
    }

    /// Every frame survives the trip out to a number and back.
    #[test]
    fn a_frame_round_trips_through_its_mav_frame_value() {
        for frame in AltitudeFrame::all() {
            assert_eq!(
                AltitudeFrame::from_mav_frame(frame.mav_frame()),
                Some(frame),
                "{} did not round trip",
                frame.label()
            );
        }
    }

    /// And through the settings file, which stores a name rather than a number.
    #[test]
    fn a_frame_round_trips_through_the_settings_key() {
        for frame in AltitudeFrame::all() {
            assert_eq!(AltitudeFrame::from_key(frame.key()), frame);
        }
    }

    /// A settings file hand-edited into nonsense gives the frame every mission uses.
    ///
    /// Not terrain: a vehicle without terrain data refuses a terrain mission at upload, long
    /// after the choice was made, and nobody would connect the refusal to a typo in a config file.
    #[test]
    fn an_unrecognised_frame_name_falls_back_to_relative() {
        for nonsense in ["", "TERRAIN", "3", "above ground", "🙂"] {
            assert_eq!(AltitudeFrame::from_key(nonsense), AltitudeFrame::Relative);
        }
    }

    /// A waypoint is created in the frame the screen is set to.
    #[test]
    fn a_waypoint_takes_the_chosen_frame() {
        for frame in AltitudeFrame::all() {
            let mut plan = Plan::default();
            plan.add_waypoint_in(
                LatLon::new(-35.363_262, 149.165_237).expect("valid"),
                50.0,
                frame,
            );
            assert_eq!(
                plan.items().first().map(|item| item.frame),
                Some(frame.mav_frame()),
                "a waypoint added in {} did not get that frame",
                frame.label()
            );
        }
    }

    /// A terrain mission survives being written to a file and read back.
    ///
    /// The frame is column three of the QGC WPL format, and losing it turns a mission that clears
    /// a hill into one that flies at the same number above home.
    #[test]
    fn a_terrain_frame_survives_a_waypoints_file() {
        let mut plan = Plan::default();
        for (latitude, longitude) in [(-35.36, 149.16), (-35.35, 149.17)] {
            plan.add_waypoint_in(
                LatLon::new(latitude, longitude).expect("valid"),
                80.0,
                AltitudeFrame::Terrain,
            );
        }

        let text = plan.waypoints_file();
        assert!(
            text.lines()
                .skip(2)
                .all(|line| line.split('\t').nth(2) == Some("10")),
            "the terrain frame should be column three of every row after home:\n{text}"
        );

        let read = mp_mission::read_waypoints(&text).expect("our own output must parse");
        let (_, rows) = mp_mission::rows::split_home(&read);
        assert_eq!(rows.len(), plan.items().len());
        assert!(
            rows.iter().all(|item| item.frame == FRAME_TERRAIN),
            "the frame did not survive the round trip"
        );
    }

    /// An item in a frame the enum does not name is shown as its number, not as relative.
    ///
    /// A mission read back from a vehicle can hold anything; showing nothing for an unknown frame
    /// tells the operator it is relative when it is not.
    #[test]
    fn an_unknown_frame_is_shown_rather_than_hidden() {
        assert_eq!(frame_label(FRAME_RELATIVE), "relative");
        assert_eq!(frame_label(FRAME_ABSOLUTE), "absolute");
        assert_eq!(frame_label(FRAME_TERRAIN), "terrain");
        assert_eq!(frame_label(1), "frame 1");
        assert_eq!(frame_label(255), "frame 255");
    }

    /// A still click on empty map adds a waypoint. This is the whole feature.
    #[test]
    fn a_still_click_on_empty_map_adds_a_waypoint() {
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (100.0, 100.0)),
            MapRelease::AddWaypoint
        );
    }

    /// A drag does not. Moving the map is the commonest thing anybody does to it, and a drag that
    /// dropped a waypoint every time would make the map unusable.
    #[test]
    fn a_drag_does_not_add_a_waypoint() {
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (400.0, 260.0)),
            MapRelease::Nothing
        );
        // Either axis is enough.
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (100.0, 260.0)),
            MapRelease::Nothing
        );
        assert_eq!(
            map_release(true, None, Some((100.0, 100.0)), (400.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// Releasing over a waypoint that was grabbed adds nothing - "cant add WP in existing rect".
    #[test]
    fn releasing_a_grabbed_waypoint_adds_nothing() {
        assert_eq!(
            map_release(true, Some(3), Some((100.0, 100.0)), (100.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// The flight screen's map is for watching, not editing.
    #[test]
    fn a_click_outside_the_planning_screen_adds_nothing() {
        assert_eq!(
            map_release(false, None, Some((100.0, 100.0)), (100.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// A release with no press behind it is a button that went down somewhere else.
    ///
    /// Dragging in from off the map, or a press the window never saw. Adding a waypoint for it
    /// puts one wherever a stray release landed.
    #[test]
    fn a_release_with_no_press_adds_nothing() {
        assert_eq!(
            map_release(true, None, None, (100.0, 100.0)),
            MapRelease::Nothing
        );
    }

    /// The slop is a tolerance, not a licence: inside it is a click, outside it is a drag.
    #[test]
    fn the_boundary_of_the_click_tolerance_is_where_it_says_it_is() {
        let press = Some((100.0, 100.0));
        // Exactly at the tolerance is still a click - the test is `>`, not `>=`.
        let at = 100.0 + CLICK_SLOP;
        assert_eq!(
            map_release(true, None, press, (at, 100.0)),
            MapRelease::AddWaypoint
        );
        // A hair past it is a drag.
        let past = 100.0 + CLICK_SLOP + 0.5;
        assert_eq!(
            map_release(true, None, press, (past, 100.0)),
            MapRelease::Nothing
        );
        // And it works in the negative direction too, which an `abs()` that went missing would
        // break silently in one direction only.
        let back = 100.0 - CLICK_SLOP - 0.5;
        assert_eq!(
            map_release(true, None, press, (back, 100.0)),
            MapRelease::Nothing
        );
    }

    /// Three clicks make three waypoints, in the order they were clicked.
    ///
    /// The decision and the effect together, because "the click was recognised" and "the mission
    /// grew by one item at that position" are different claims.
    #[test]
    fn clicking_three_times_builds_a_three_item_mission() {
        let mut plan = Plan::default();
        let points = [
            (-35.363_262, 149.165_237),
            (-35.362_000, 149.166_000),
            (-35.361_000, 149.167_000),
        ];
        for (latitude, longitude) in points {
            let press = Some((10.0, 10.0));
            assert_eq!(
                map_release(true, None, press, (10.0, 10.0)),
                MapRelease::AddWaypoint
            );
            plan.add_waypoint(
                LatLon::new(latitude, longitude).expect("a valid position"),
                50.0,
            );
        }

        assert_eq!(plan.items().len(), 3);
        for (index, (latitude, longitude)) in points.iter().enumerate() {
            let item = &plan.items()[index];
            // Waypoint 1 is the first click: the grid numbers from 1, because home is 0.
            assert_eq!(u16::try_from(index + 1).unwrap_or(0), item.seq, "sequence");
            assert!((item.x - latitude).abs() < 1e-9, "latitude of item {index}");
            assert!(
                (item.y - longitude).abs() < 1e-9,
                "longitude of item {index}"
            );
            assert_eq!(item.frame, FRAME_RELATIVE);
            assert_eq!(item.command, CMD_WAYPOINT);
        }
    }

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

    /// The rows are numbered as the grid's headers read, from 1: home is 0 and is not a row.
    #[test]
    fn waypoints_are_numbered_from_one_without_gaps() {
        let mut plan = Plan::default();
        for n in 0..4 {
            plan.add_waypoint(at(-35.36 + f64::from(n) * 0.001, 149.16), 50.0);
        }
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![1, 2, 3, 4]);
    }

    #[test]
    fn deleting_renumbers_so_the_vehicle_will_accept_the_mission() {
        // A mission with a gap in its sequence is rejected at upload time, minutes after the edit
        // that caused it and with no indication of which one it was.
        let mut plan = Plan::default();
        for n in 0..4 {
            plan.add_waypoint(at(-35.36 + f64::from(n) * 0.001, 149.16), 50.0);
        }
        plan.remove(2);
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![1, 2, 3]);
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

        plan.move_item(3, -1);

        let latitudes: Vec<f64> = plan.items().iter().map(|i| i.x).collect();
        assert!((latitudes[1] - -35.362).abs() < 1e-9, "{latitudes:?}");
        assert!((latitudes[2] - -35.361).abs() < 1e-9, "{latitudes:?}");
        let seqs: Vec<u16> = plan.items().iter().map(|i| i.seq).collect();
        assert_eq!(seqs, vec![1, 2, 3]);
        assert_eq!(plan.selected(), Some(2), "the moved row stays selected");
    }

    #[test]
    fn moving_past_either_end_does_nothing() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.add_waypoint(at(-35.37, 149.16), 50.0);

        plan.move_item(1, -1);
        plan.move_item(2, 1);

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
        let _ = plan.adopt_from_vehicle(&[]);
        assert_eq!(*plan.origin(), Origin::Vehicle);
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn a_loaded_plan_names_its_file() {
        let mut plan = Plan::default();
        let _ = plan.adopt_from_file("survey.waypoints", &[]);
        assert_eq!(plan.origin().label(), "loaded from survey.waypoints");
    }

    #[test]
    fn adopting_clears_a_stale_selection() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.select(Some(1));
        let _ = plan.adopt_from_vehicle(&[]);
        assert_eq!(plan.selected(), None);
    }

    #[test]
    fn map_clicks_produce_items_a_vehicle_will_fly() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.363, 149.165), 50.0);
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

        plan.nudge_altitude(2, 10.0);

        assert!((plan.items()[0].z - 50.0).abs() < 1e-9);
        assert!((plan.items()[1].z - 60.0).abs() < 1e-9);
    }

    #[test]
    fn a_held_stepper_cannot_walk_a_waypoint_into_the_stratosphere() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for _ in 0..500 {
            plan.nudge_altitude(1, 10.0);
        }
        assert!((plan.items()[0].z - MAX_STEPPED_ALTITUDE).abs() < 1e-9);
    }

    #[test]
    fn a_waypoint_below_home_is_allowed() {
        // Meaningful on a vehicle launched from a cliff or a rooftop. Refusing it would be the
        // ground station deciding it knows the terrain better than the operator.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 5.0);
        plan.nudge_altitude(1, -10.0);
        assert!(plan.items()[0].z < 0.0);
    }

    #[test]
    fn changing_to_a_positionless_command_clears_the_coordinates() {
        // Leaving stale coordinates on a command that ignores them is how a mission looks right
        // on the map and flies somewhere else: the map would keep drawing a waypoint the vehicle
        // has no intention of visiting.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(1, CMD_RTL);
        assert!((plan.items()[0].x).abs() < f64::EPSILON);
        assert!((plan.items()[0].y).abs() < f64::EPSILON);
        assert_eq!(plan.items()[0].command, CMD_RTL);
    }

    #[test]
    fn changing_to_a_positioned_command_keeps_the_coordinates() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(1, 22);
        assert!((plan.items()[0].x - -35.36).abs() < 1e-9);
        assert_eq!(plan.items()[0].command, 22);
    }

    #[test]
    fn editing_an_item_marks_the_plan_as_no_longer_the_vehicles() {
        let mut plan = Plan::default();
        let _ = plan.adopt_from_vehicle(&downloaded(&[MissionItem {
            seq: 1,
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
        }]));
        assert_eq!(*plan.origin(), Origin::Vehicle);
        plan.nudge_altitude(1, 10.0);
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
    fn dragging_a_waypoint_moves_only_its_position() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 75.0);
        plan.move_to(1, at(-35.37, 149.17));

        let item = &plan.items()[0];
        assert!((item.x - -35.37).abs() < 1e-9);
        assert!((item.y - 149.17).abs() < 1e-9);
        // Altitude and command survive the move: dragging on a map says where, not what or how
        // high.
        assert!((item.z - 75.0).abs() < 1e-9);
        assert_eq!(item.command, CMD_WAYPOINT);
    }

    #[test]
    fn a_command_with_no_position_cannot_be_dragged_somewhere() {
        // Writing coordinates into a return-to-launch would make the map draw a waypoint the
        // vehicle has no intention of visiting.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_command(1, CMD_RTL);
        plan.move_to(1, at(-35.37, 149.17));

        assert!((plan.items()[0].x).abs() < f64::EPSILON);
        assert!((plan.items()[0].y).abs() < f64::EPSILON);
    }

    #[test]
    fn dragging_marks_the_plan_as_no_longer_the_vehicles() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        let _ = plan.adopt_from_vehicle(&downloaded(plan.items()));
        assert_eq!(*plan.origin(), Origin::Vehicle);
        plan.move_to(1, at(-35.37, 149.17));
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn dragging_an_item_that_is_not_there_is_ignored() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.move_to(99, at(0.0, 0.0));
        assert!((plan.items()[0].x - -35.36).abs() < 1e-9);
    }

    #[test]
    fn a_fence_needs_three_corners_before_it_can_be_sent() {
        // Fewer encloses nothing, and both the protocol and the vehicle refuse it. Saying why is
        // better than a write button that fails silently.
        let mut plan = Plan::default();
        plan.add_fence_vertex(at(-35.364, 149.164));
        plan.add_fence_vertex(at(-35.364, 149.168));
        assert!(plan.fence_items().is_none());
        let error = plan.fence_error().expect("an explanation");
        assert!(error.contains('3'), "{error}");

        plan.add_fence_vertex(at(-35.361, 149.168));
        let items = plan.fence_items().expect("three corners should be enough");
        assert_eq!(items.len(), 3);
        assert!(plan.fence_error().is_none());
    }

    #[test]
    fn a_drawn_fence_becomes_inclusion_polygon_items() {
        // Inclusion: the vehicle must stay inside. An exclusion fence drawn by mistake would do
        // nothing useful, or trigger a return-to-launch in flight.
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_fence_vertex(vertex);
        }
        let items = plan.fence_items().expect("a valid fence");
        assert_eq!(items.len(), 4);
        for item in &items {
            assert_eq!(item.command, mp_mission::fence::CMD_FENCE_POLYGON_INCLUSION);
            // Every vertex declares the total, which is how the run is delimited on the wire.
            assert!(
                (item.param1 - 4.0).abs() < f64::EPSILON,
                "{:?}",
                item.param1
            );
        }
    }

    #[test]
    fn a_fence_round_trips_through_the_items_the_protocol_carries() {
        let mut plan = Plan::default();
        for vertex in area() {
            plan.add_fence_vertex(vertex);
        }
        let items = plan.fence_items().expect("a valid fence");

        let mut received = Plan::default();
        received.adopt_fence(&items);
        assert_eq!(received.fence().len(), 4);
        for (sent, got) in area().iter().zip(received.fence()) {
            assert!((sent.latitude() - got.latitude()).abs() < 1e-9);
            assert!((sent.longitude() - got.longitude()).abs() < 1e-9);
        }
    }

    #[test]
    fn undoing_and_clearing_the_fence_leave_the_mission_alone() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for vertex in area() {
            plan.add_fence_vertex(vertex);
        }

        plan.undo_fence_vertex();
        assert_eq!(plan.fence().len(), 3);
        plan.clear_fence();
        assert!(plan.fence().is_empty());
        assert_eq!(plan.items().len(), 1, "the mission should be untouched");
    }

    #[test]
    fn the_fence_and_the_survey_area_are_separate_shapes() {
        // They are both polygons drawn by clicking, and confusing them would send a survey
        // boundary to the vehicle as a fence.
        let mut plan = Plan::default();
        plan.set_draw_mode(DrawMode::Area);
        plan.add_area_vertex(at(-35.360, 149.160));
        plan.set_draw_mode(DrawMode::Fence);
        plan.add_fence_vertex(at(-35.370, 149.170));

        assert_eq!(plan.polygon().len(), 1);
        assert_eq!(plan.fence().len(), 1);
        assert!((plan.polygon()[0].latitude() - -35.360).abs() < 1e-9);
        assert!((plan.fence()[0].latitude() - -35.370).abs() < 1e-9);
    }

    #[test]
    fn adopting_a_vehicle_with_no_polygon_fence_says_so() {
        // Rather than showing an empty fence that looks like one was read successfully.
        let mut plan = Plan::default();
        plan.adopt_fence(&[]);
        assert!(plan.fence().is_empty());
        assert!(plan.fence_error().is_some());
    }

    #[test]
    fn a_rally_point_takes_the_rally_altitude() {
        // The survey panel's altitude, which rally points took, is gone to the Survey (Grid)
        // dialog; they keep the 50 m it started at.
        let mut plan = Plan::default();
        let expected = RALLY_ALTITUDE;
        plan.add_rally_point(at(-35.3625, 149.1655));

        assert_eq!(plan.rally().len(), 1);
        assert!((plan.rally()[0].altitude - expected).abs() < 1e-9);
    }

    #[test]
    fn rally_points_round_trip_through_the_items_the_protocol_carries() {
        let mut plan = Plan::default();
        plan.add_rally_point(at(-35.3625, 149.1655));
        plan.add_rally_point(at(-35.3630, 149.1660));
        let items = plan.rally_items().expect("two points should send");
        assert_eq!(items.len(), 2);
        for (index, item) in items.iter().enumerate() {
            assert_eq!(item.command, mp_mission::fence::CMD_RALLY_POINT);
            assert_eq!(usize::from(item.seq), index);
        }

        let mut received = Plan::default();
        received.adopt_rally(&items);
        assert_eq!(received.rally().len(), 2);
        assert!((received.rally()[0].position.latitude() - -35.3625).abs() < 1e-9);
    }

    #[test]
    fn sending_no_rally_points_says_why_rather_than_sending_nothing() {
        // An empty upload would clear the vehicle's rally points, which is a different intent from
        // not having drawn any yet.
        let mut plan = Plan::default();
        assert!(plan.rally_items().is_none());
        assert!(plan.rally_error().is_some());
    }

    #[test]
    fn adopting_a_vehicle_with_no_rally_points_says_so() {
        let mut plan = Plan::default();
        plan.adopt_rally(&[]);
        assert!(plan.rally().is_empty());
        assert!(plan.rally_error().is_some());
    }

    #[test]
    fn the_four_drawing_modes_are_distinct_and_named() {
        // The ids address controls a test script clicks; two sharing one would make a click land
        // on the wrong mode.
        let mut ids: Vec<&str> = DrawMode::ALL.iter().map(|m| m.id()).collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count);

        for mode in DrawMode::ALL {
            assert!(!mode.label().is_empty());
            // A click places things, as AddWPToMap does; the right button is the menu's.
            assert!(mode.hint().starts_with("click the map"), "{}", mode.hint());
        }
    }

    #[test]
    fn the_shapes_do_not_share_storage() {
        // Four things are placed by clicking on the same map. Mixing any two would send one
        // to the vehicle as another.
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.300, 149.100), 50.0);
        plan.add_area_vertex(at(-35.310, 149.110));
        plan.add_fence_vertex(at(-35.320, 149.120));
        plan.add_rally_point(at(-35.330, 149.130));

        assert_eq!(plan.items().len(), 1);
        assert_eq!(plan.polygon().len(), 1);
        assert_eq!(plan.fence().len(), 1);
        assert_eq!(plan.rally().len(), 1);
        assert!((plan.items()[0].x - -35.300).abs() < 1e-9);
        assert!((plan.polygon()[0].latitude() - -35.310).abs() < 1e-9);
        assert!((plan.fence()[0].latitude() - -35.320).abs() < 1e-9);
        assert!((plan.rally()[0].position.latitude() - -35.330).abs() < 1e-9);
    }

    #[test]
    fn command_parameters_are_labelled_from_the_definitions() {
        // Not from a table here. A mission editor that shows "param1" is one the operator has to
        // look the meaning up for, and the definitions already carry it.
        let waypoint = MavCmd(u32::from(CMD_WAYPOINT)).parameters();
        assert_eq!(waypoint[0].map(|(label, _)| label), Some("Hold"));
        assert_eq!(waypoint[0].map(|(_, units)| units), Some("s"));

        let loiter_time = MavCmd(19).parameters();
        assert_eq!(loiter_time[0].map(|(label, _)| label), Some("Time"));
        assert_eq!(loiter_time[0].map(|(_, units)| units), Some("s"));
    }

    #[test]
    fn a_command_that_uses_no_parameters_offers_none() {
        // Return-to-launch takes nothing. Four controls doing nothing would imply otherwise.
        let rtl = MavCmd(u32::from(CMD_RTL)).parameters();
        assert!(rtl.iter().all(Option::is_none), "{rtl:?}");
    }

    #[test]
    fn setting_a_parameter_changes_only_that_one() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.set_param(1, 0, 12.0);

        assert!((plan.items()[0].param1 - 12.0).abs() < f64::EPSILON);
        assert!(plan.items()[0].param2.abs() < f64::EPSILON);
        assert!(plan.items()[0].param3.abs() < f64::EPSILON);
        assert!(plan.items()[0].param4.abs() < f64::EPSILON);
        // And nothing about where or how high it is.
        assert!((plan.items()[0].x - -35.36).abs() < 1e-9);
        assert!((plan.items()[0].z - 50.0).abs() < f64::EPSILON);
    }

    #[test]
    fn every_parameter_slot_is_reachable_and_reads_back() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        for index in 0..4 {
            #[allow(clippy::cast_precision_loss)]
            let value = (index + 1) as f64;
            plan.set_param(1, index, value);
            assert!((plan.param(1, index).expect("a value") - value).abs() < f64::EPSILON);
        }
        // Out of range is ignored rather than wrapping onto another parameter.
        plan.set_param(1, 9, 99.0);
        assert!(plan.param(1, 9).is_none());
    }

    #[test]
    fn editing_a_parameter_marks_the_plan_as_no_longer_the_vehicles() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        let _ = plan.adopt_from_vehicle(&downloaded(plan.items()));
        plan.set_param(1, 0, 5.0);
        assert_eq!(*plan.origin(), Origin::Edited);
    }

    #[test]
    fn clearing_resets_everything() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.36, 149.16), 50.0);
        plan.select(Some(1));
        plan.set_home_text(HomeBox::Lat, "-35.36".to_owned());
        plan.clear();
        assert!(plan.is_empty());
        assert_eq!(plan.selected(), None);
        assert_eq!(*plan.origin(), Origin::Empty);
        assert_eq!(
            plan.home_text(HomeBox::Lat),
            "-35.36",
            "home is the boxes', not the mission's"
        );
    }

    // ---- The map menu: each entry against its C# handler ----

    use mp_mission::commands as cmd;

    const CLICK: (f32, f32) = (400.0, 300.0);

    fn canberra() -> LatLon {
        at(-35.363_262, 149.165_237)
    }

    fn context() -> MenuContext {
        MenuContext {
            frame: AltitudeFrame::Relative,
            vehicle: None,
            takeoff_pitch: false,
            copter: false,
            tracker_alt: 0.0,
        }
    }

    /// The home the vehicle in these tests has: 580 m above sea level.
    const VEHICLE_HOME: Home = Home {
        lat: -35.36,
        lng: 149.16,
        alt: 580.0,
    };

    /// Rows as a vehicle sends them back: its home at item 0, then the rows from 1.
    fn downloaded(rows: &[MissionItem]) -> Vec<MissionItem> {
        mp_mission::rows::upload_list(Some(VEHICLE_HOME), rows)
    }

    /// A plan as it arrives from a vehicle, the operator having said Yes to its home: home at
    /// 580 m in the boxes, and two waypoints at 100 in the grid.
    fn from_vehicle() -> Plan {
        let mut plan = Plan::default();
        let offer = plan.adopt_from_vehicle(&downloaded(&[
            cmd::waypoint(at(-35.361, 149.161), 100.0, FRAME_RELATIVE),
            cmd::waypoint(at(-35.362, 149.162), 100.0, FRAME_RELATIVE),
        ]));
        plan.reset_home_to(offer.expect("empty boxes are offered the vehicle's home"));
        plan
    }

    /// Opens the menu over `marker` at Canberra and chooses `action`.
    fn choose(plan: &mut Plan, menus: &mut PlanMenus, action: MenuAction, marker: Option<u16>) {
        menus.open_at(CLICK, canberra(), marker);
        menus.choose(plan, action, &context());
    }

    /// Types `value` into the open prompt, replacing what it offered, and presses OK.
    fn answer(plan: &mut Plan, menus: &mut PlanMenus, value: &str) {
        menus
            .prompt
            .as_mut()
            .and_then(|prompt| prompt.field.as_mut())
            .expect("a prompt with a field")
            .set(value);
        menus.submit(plan, &context());
    }

    fn commands(plan: &Plan) -> Vec<u16> {
        plan.items().iter().map(|item| item.command).collect()
    }

    fn last(plan: &Plan) -> MissionItem {
        *plan.items().last().expect("an item")
    }

    /// The constructors' command ids are the MAVLink ones the C#'s `MAV_CMD` names resolve to.
    #[test]
    fn the_menu_commands_are_the_mavlink_commands_the_c_sharp_names() {
        for (id, name) in [
            (cmd::WAYPOINT, "MAV_CMD_NAV_WAYPOINT"),
            (cmd::LOITER_UNLIM, "MAV_CMD_NAV_LOITER_UNLIM"),
            (cmd::LOITER_TURNS, "MAV_CMD_NAV_LOITER_TURNS"),
            (cmd::LOITER_TIME, "MAV_CMD_NAV_LOITER_TIME"),
            (cmd::RETURN_TO_LAUNCH, "MAV_CMD_NAV_RETURN_TO_LAUNCH"),
            (cmd::LAND, "MAV_CMD_NAV_LAND"),
            (cmd::TAKEOFF, "MAV_CMD_NAV_TAKEOFF"),
            (cmd::SPLINE_WAYPOINT, "MAV_CMD_NAV_SPLINE_WAYPOINT"),
            (cmd::DO_JUMP, "MAV_CMD_DO_JUMP"),
            (cmd::DO_SET_ROI, "MAV_CMD_DO_SET_ROI"),
        ] {
            assert_eq!(MavCmd(u32::from(id)).name(), Some(name), "command {id}");
        }
    }

    /// Delete WP is enabled over a marker and nowhere else, as `contextMenuStrip1_Opening` has it.
    #[test]
    fn delete_wp_is_enabled_only_over_a_marker() {
        let mut menus = PlanMenus::default();
        assert!(!menus.delete_enabled(), "nothing is open");
        menus.open_at(CLICK, canberra(), None);
        assert!(!menus.delete_enabled());
        menus.open_at(CLICK, canberra(), Some(2));
        assert!(menus.delete_enabled());
    }

    /// Delete WP removes the waypoint under the cursor and renumbers the rest.
    #[test]
    fn delete_wp_removes_the_waypoint_under_the_cursor() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::DeleteWp, Some(1));
        assert_eq!(plan.items().len(), 1);
        assert!(
            (plan.items()[0].x - -35.362).abs() < 1e-9,
            "the second waypoint moved up"
        );
        assert_eq!(plan.items()[0].seq, 1, "and is waypoint 1 now");
        assert!(menus.open.is_none(), "choosing closes the menu");
    }

    /// Home is not a row, so Delete WP never reaches it: nothing is numbered 0, and the boxes
    /// keep home.
    #[test]
    fn delete_wp_never_reaches_home() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::DeleteWp, Some(0));
        assert_eq!(plan.items().len(), 2);
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
    }

    /// Insert Wp offers the number after the selected waypoint, and accepting it puts a WAYPOINT
    /// there at the menu's position, in the screen's frame, at the default altitude.
    #[test]
    fn insert_wp_puts_a_waypoint_after_the_number_given() {
        let mut plan = from_vehicle();
        plan.select(Some(1));
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertWp, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(prompt.title, "Insert WP");
        assert_eq!(prompt.text, "Insert WP after wp#");
        assert_eq!(prompt.value(), "1", "(selectedrow + 1): after waypoint 1");
        menus.submit(&mut plan, &context());
        assert_eq!(commands(&plan), vec![16, 16, 16]);
        let inserted = plan.items()[1];
        assert_eq!(inserted.position(), Ok(Some(canberra())));
        assert!((inserted.z - 100.0).abs() < 1e-9, "the default altitude");
        assert_eq!(inserted.frame, FRAME_RELATIVE);
        assert_eq!(plan.selected(), Some(2), "the new row is selected");
    }

    /// With nothing selected the offer is after the last waypoint.
    #[test]
    fn insert_wp_offers_the_end_with_nothing_selected() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertWp, None);
        assert_eq!(menus.prompt.as_ref().map(Prompt::value), Some("2"));
        answer(&mut plan, &mut menus, "0");
        assert_eq!(plan.items().len(), 3);
        assert_eq!(
            plan.items()[0].position(),
            Ok(Some(canberra())),
            "0 is after home: waypoint 1"
        );
        assert_eq!(plan.items()[0].seq, 1);
    }

    /// A number `Rows.Insert` would throw on is refused in the C#'s words, and nothing changes.
    #[test]
    fn insert_wp_refuses_a_position_that_is_not_there() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertWp, None);
        answer(&mut plan, &mut menus, "9");
        assert_eq!(plan.items().len(), 2);
        let message = menus.prompt.as_ref().expect("a message");
        assert_eq!(message.title, "Error");
        assert_eq!(message.text, "Invalid insert position");
        assert_eq!(message.kind, PromptKind::Message);
    }

    /// Insert Spline WP puts a SPLINE_WAYPOINT, and refuses with Strings.InvalidNumberEntered.
    #[test]
    fn insert_spline_wp_puts_a_spline_waypoint() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::InsertSplineWp, None);
        answer(&mut plan, &mut menus, "1");
        assert_eq!(commands(&plan), vec![16, 82, 16]);

        choose(&mut plan, &mut menus, MenuAction::InsertSplineWp, None);
        answer(&mut plan, &mut menus, "x");
        assert_eq!(plan.items().len(), 3);
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some("Invalid number entered")
        );
    }

    /// Loiter > Forever appends LOITER_UNLIM at the click, no prompt.
    #[test]
    fn loiter_forever_appends_an_unlimited_loiter() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterForever, None);
        assert!(menus.prompt.is_none());
        let item = last(&plan);
        assert_eq!(item.command, 17);
        assert_eq!(item.position(), Ok(Some(canberra())));
        assert!((item.z - 100.0).abs() < 1e-9);
    }

    /// Loiter > Time asks "Loiter Time", offering 5, and puts the answer in Param1.
    #[test]
    fn loiter_time_asks_for_a_time_and_puts_it_in_param1() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterTime, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str()),
            ("Loiter Time", "Loiter Time")
        );
        assert_eq!(prompt.value(), "5");
        answer(&mut plan, &mut menus, "12");
        let item = last(&plan);
        assert_eq!(item.command, 19);
        assert!((item.param1 - 12.0).abs() < 1e-9);
        assert_eq!(item.position(), Ok(Some(canberra())));
    }

    /// Loiter > Circles asks "Loiter Turns", offering 3.
    #[test]
    fn loiter_circles_asks_for_turns_and_puts_them_in_param1() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterCircles, None);
        assert_eq!(menus.prompt.as_ref().map(|p| p.title), Some("Loiter Turns"));
        assert_eq!(menus.prompt.as_ref().map(Prompt::value), Some("3"));
        menus.submit(&mut plan, &context());
        let item = last(&plan);
        assert_eq!(item.command, 18);
        assert!((item.param1 - 3.0).abs() < 1e-9);
    }

    /// Cancel is the handler's `return`: nothing is added.
    #[test]
    fn cancelling_a_prompt_adds_nothing() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::LoiterTime, None);
        menus.cancel(&mut plan);
        assert!(menus.prompt.is_none());
        assert_eq!(plan.items().len(), 2);
    }

    /// Jump > Start jumps to item 1 the number of times asked, offering 5.
    #[test]
    fn jump_start_jumps_to_item_one() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::JumpStart, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("Jump repeat", "Number of times to Repeat", "5")
        );
        menus.submit(&mut plan, &context());
        let item = last(&plan);
        assert_eq!(item.command, 177);
        assert!((item.param1 - 1.0).abs() < 1e-9);
        assert!((item.param2 - 5.0).abs() < 1e-9);
        assert_eq!(item.position(), Ok(None));
    }

    /// Jump > WP # asks for the target, offering 1, then the repeat count.
    #[test]
    fn jump_wp_asks_for_the_target_then_the_repeat() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::JumpWp, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("WP No", "Jump to WP no?", "1")
        );
        answer(&mut plan, &mut menus, "2");
        assert_eq!(menus.prompt.as_ref().map(|p| p.title), Some("Jump repeat"));
        answer(&mut plan, &mut menus, "3");
        let item = last(&plan);
        assert_eq!(item.command, 177);
        assert!((item.param1 - 2.0).abs() < 1e-9);
        assert!((item.param2 - 3.0).abs() < 1e-9);
    }

    /// Text that is not a number is refused with Strings.InvalidNumberEntered.
    #[test]
    fn a_jump_that_is_not_a_number_is_refused() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::JumpStart, None);
        answer(&mut plan, &mut menus, "lots");
        assert_eq!(plan.items().len(), 2);
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some("Invalid number entered")
        );
    }

    /// RTL appends RETURN_TO_LAUNCH and nothing else.
    #[test]
    fn rtl_appends_a_return_to_launch() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Rtl, None);
        let item = last(&plan);
        assert_eq!(item.command, 20);
        assert_eq!((item.x, item.y, item.z), (0.0, 0.0, 0.0));
        assert_eq!(item.frame, FRAME_RELATIVE);
    }

    /// Land appends LAND at the click, one metre up.
    #[test]
    fn land_appends_a_land_at_the_click() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Land, None);
        let item = last(&plan);
        assert_eq!(item.command, 21);
        assert_eq!(item.position(), Ok(Some(canberra())));
        assert!((item.z - 1.0).abs() < 1e-9);
    }

    /// Takeoff asks for the altitude, offering 10 in metres, and writes it with no position.
    #[test]
    fn takeoff_asks_for_an_altitude() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Takeoff, None);
        let prompt = menus.prompt.as_ref().expect("an InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("Altitude", "Please enter your takeoff altitude in m", "10")
        );
        answer(&mut plan, &mut menus, "25");
        assert!(menus.prompt.is_none(), "not a plane: no pitch");
        let item = last(&plan);
        assert_eq!(item.command, 22);
        assert!((item.z - 25.0).abs() < 1e-9);
        assert!((item.param1 - 0.0).abs() < 1e-9);
        assert_eq!(item.position(), Ok(None));
    }

    /// On a plane it asks for the pitch too, offering 15, and puts it in Param1.
    #[test]
    fn takeoff_on_a_plane_asks_for_the_pitch() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        let plane = MenuContext {
            takeoff_pitch: true,
            ..context()
        };
        menus.open_at(CLICK, canberra(), None);
        menus.choose(&mut plan, MenuAction::Takeoff, &plane);
        menus.submit(&mut plan, &plane);
        let prompt = menus.prompt.as_ref().expect("the pitch");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            ("Takeoff Pitch", "Please enter your takeoff pitch", "15")
        );
        menus.submit(&mut plan, &plane);
        let item = last(&plan);
        assert_eq!(item.command, 22);
        assert!((item.z - 10.0).abs() < 1e-9);
        assert!((item.param1 - 15.0).abs() < 1e-9);
    }

    /// `int.TryParse` refuses a fractional altitude: "Bad Alt".
    #[test]
    fn a_takeoff_altitude_int_parse_refuses_is_a_bad_alt() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::Takeoff, None);
        answer(&mut plan, &mut menus, "10.5");
        assert_eq!(plan.items().len(), 2);
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some("Bad Alt")
        );
    }

    /// Which vehicles are asked for a pitch: ArduPlane's types, and a quadplane only with bit 1.
    #[test]
    fn only_a_plane_is_asked_for_a_takeoff_pitch() {
        let none: Vec<(String, f64)> = Vec::new();
        assert!(
            MenuContext::asks_takeoff_pitch(Some(1), &none),
            "fixed wing"
        );
        assert!(
            MenuContext::asks_takeoff_pitch(Some(16), &none),
            "flapping wing"
        );
        assert!(
            MenuContext::asks_takeoff_pitch(Some(20), &none),
            "VTOL quad"
        );
        assert!(
            !MenuContext::asks_takeoff_pitch(Some(2), &none),
            "quadrotor"
        );
        assert!(!MenuContext::asks_takeoff_pitch(Some(10), &none), "rover");
        assert!(!MenuContext::asks_takeoff_pitch(None, &none), "no vehicle");
        let without = vec![("Q_OPTIONS".to_owned(), 1.0)];
        let with = vec![("Q_OPTIONS".to_owned(), 2.0)];
        assert!(!MenuContext::asks_takeoff_pitch(Some(1), &without));
        assert!(MenuContext::asks_takeoff_pitch(Some(1), &with));
    }

    /// DO_SET_ROI appends a region of interest at the click, which is a position.
    #[test]
    fn set_roi_appends_a_region_of_interest() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::SetRoi, None);
        let item = last(&plan);
        assert_eq!(item.command, 201);
        assert_eq!(item.position(), Ok(Some(canberra())));
        assert!((item.z - 100.0).abs() < 1e-9);
    }

    /// Clear Mission empties the rows and keeps home, which is not one of them.
    #[test]
    fn clear_mission_keeps_home() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::ClearMission, None);
        assert!(plan.is_empty());
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
        // A plan drawn here has rows only, and they all go.
        let mut drawn = Plan::default();
        drawn.add_waypoint(canberra(), 50.0);
        drawn.add_waypoint(canberra(), 50.0);
        choose(&mut drawn, &mut menus, MenuAction::ClearMission, None);
        assert!(drawn.is_empty());
    }

    /// Reverse WPs turns the rows round; home is still what is written first.
    #[test]
    fn reverse_wps_turns_the_rows_round_and_home_stays_first() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::ReverseWps, None);
        let latitudes: Vec<f64> = plan.items().iter().map(|item| item.x).collect();
        assert_eq!(latitudes, vec![-35.362, -35.361]);
        let seqs: Vec<u16> = plan.items().iter().map(|item| item.seq).collect();
        assert_eq!(seqs, vec![1, 2]);
        let written = plan.vehicle_mission(true).expect("a home");
        let latitudes: Vec<f64> = written.iter().map(|item| item.x).collect();
        assert_eq!(latitudes, vec![-35.36, -35.362, -35.361]);
    }

    /// Draw a Polygon starts drawing the first time and adds the corner under the menu after.
    #[test]
    fn draw_a_polygon_starts_drawing_then_adds_corners() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::DrawPolygon, None);
        assert_eq!(plan.draw_mode(), DrawMode::Area);
        assert!(
            plan.polygon().is_empty(),
            "the first choice only starts drawing"
        );
        choose(&mut plan, &mut menus, MenuAction::DrawPolygon, None);
        assert_eq!(plan.polygon(), &[canberra()]);
        // And a click on the map adds a corner while drawing: AddWPToMap's polygongridmode.
        plan.add_wp_to_map(at(-35.37, 149.17), 50.0, AltitudeFrame::Relative);
        assert_eq!(plan.polygon().len(), 2);
        assert!(plan.is_empty(), "no waypoint was added");
    }

    /// Clear Polygon removes the corners and stops drawing.
    #[test]
    fn clear_polygon_removes_the_corners_and_stops_drawing() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        plan.set_draw_mode(DrawMode::Area);
        plan.add_area_vertex(canberra());
        choose(&mut plan, &mut menus, MenuAction::ClearPolygon, None);
        assert!(plan.polygon().is_empty());
        assert_eq!(plan.draw_mode(), DrawMode::Waypoints);
    }

    /// From Current Waypoints makes the polygon from the WAYPOINT rows and asks about clearing.
    #[test]
    fn from_current_waypoints_makes_the_polygon_and_asks() {
        let mut plan = from_vehicle();
        plan.append(cmd::loiter_time(
            at(-35.9, 149.9),
            50.0,
            FRAME_RELATIVE,
            5.0,
        ));
        let mut menus = PlanMenus::default();
        choose(
            &mut plan,
            &mut menus,
            MenuAction::PolygonFromWaypoints,
            None,
        );
        assert_eq!(
            plan.polygon(),
            &[at(-35.361, 149.161), at(-35.362, 149.162)],
            "the two waypoints, not home and not the loiter"
        );
        let question = menus.prompt.as_ref().expect("the question");
        assert_eq!(
            (question.title, question.text.as_str()),
            ("Confirm", "Clear current waypoints?")
        );
        assert!(question.is_question());
        // No keeps the mission.
        menus.cancel(&mut plan);
        assert_eq!(plan.items().len(), 3);
        // Yes clears it; home, not a row, stays.
        choose(
            &mut plan,
            &mut menus,
            MenuAction::PolygonFromWaypoints,
            None,
        );
        menus.submit(&mut plan, &context());
        assert!(plan.is_empty());
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
    }

    /// With no rows it does nothing and asks nothing.
    #[test]
    fn from_current_waypoints_with_no_rows_does_nothing() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(
            &mut plan,
            &mut menus,
            MenuAction::PolygonFromWaypoints,
            None,
        );
        assert!(menus.prompt.is_none());
        assert!(plan.polygon().is_empty());
    }

    /// Measure Distance: the first choice remembers the point, the second reports distance and
    /// bearing on GMap's 6378137 m sphere, in the C#'s words and formats.
    #[test]
    fn measure_distance_takes_two_choices() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        menus.open_at(CLICK, at(0.0, 0.0), None);
        menus.choose(&mut plan, MenuAction::MeasureDistance, &context());
        assert!(menus.measure_from.is_some());
        assert_eq!(menus.prompt.as_ref().map(|p| p.title), Some("Measure Dist"));
        menus.cancel(&mut plan);
        menus.open_at(CLICK, at(0.0, 1.0), None);
        menus.choose(&mut plan, MenuAction::MeasureDistance, &context());
        assert!(menus.measure_from.is_none(), "startmeasure is reset");
        // One degree of the equator on a 6378137 m sphere is 111319.490793 m.
        assert_eq!(
            menus.prompt.as_ref().map(|p| p.text.as_str()),
            Some("Distance: 111319.49 m AZ: 90")
        );
    }

    /// Measure Dist's first box is `MessageShowAgain`'s: "Show me again?" ticked under the
    /// text, OK alone, the tick's click the setting's new value, and a cleared tick - the
    /// setting false - the box not shown at all on the next choice.
    /// `// C#: GCSViews/FlightPlanner.cs:2632-2633; Common.cs:260-270, 446-449`
    #[test]
    fn measure_dist_is_a_show_again_box() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        menus.open_at(CLICK, at(0.0, 0.0), None);
        menus.choose(&mut plan, MenuAction::MeasureDistance, &context());
        let prompt = menus.prompt.as_ref().expect("the box");
        assert_eq!(prompt.kind, PromptKind::Message);
        assert!(!prompt.is_question());
        assert_eq!(
            prompt.show_again,
            Some(ShowAgain {
                key: MEASURE_DIST_KEY,
                ticked: true
            })
        );
        assert_eq!(MEASURE_DIST_KEY, "SHOWAGAIN_Measure_Dist");
        assert_eq!(menus.toggle_show_again(), Some((MEASURE_DIST_KEY, false)));
        assert_eq!(menus.toggle_show_again(), Some((MEASURE_DIST_KEY, true)));
        assert_eq!(menus.toggle_show_again(), Some((MEASURE_DIST_KEY, false)));
        // Another key cleared: this box stays.
        menus.drop_suppressed_prompt(|key| key == FENCE_KEY);
        assert!(menus.prompt.is_some());
        // Its own key cleared by its own tick, just now: the box stays until OK, as the C#'s
        // form does (a box that vanished under the pointer was a defect, 2026-09-25).
        menus.drop_suppressed_prompt(|key| key == MEASURE_DIST_KEY);
        assert!(menus.prompt.is_some());
        // Ticked, with its key already off - a box made after an earlier clearing - it goes,
        // as `MessageShowAgain` returns OK before the form.
        assert_eq!(menus.toggle_show_again(), Some((MEASURE_DIST_KEY, true)));
        menus.drop_suppressed_prompt(|key| key == MEASURE_DIST_KEY);
        assert!(menus.prompt.is_none());
        assert_eq!(menus.toggle_show_again(), None);
        // A plain message has no tick and is never dropped.
        menus.tell("", "plain");
        menus.drop_suppressed_prompt(|_| true);
        assert!(menus.prompt.is_some());
        assert_eq!(menus.toggle_show_again(), None);
    }

    /// A show-again question - the zero altitude warning's, with Cancel - is checked at its
    /// site, not dropped: only a message goes.
    #[test]
    fn a_show_again_question_is_not_dropped_and_keeps_its_cancel() {
        let mut menus = PlanMenus::default();
        menus.ask(
            Prompt::question(ZERO_ALT_TITLE, "zero", PromptKind::WriteZeroAlt)
                .with_show_again("SHOWAGAIN_Zero_Altitude_Warning"),
        );
        menus.drop_suppressed_prompt(|_| true);
        let prompt = menus.prompt.as_ref().expect("kept");
        assert!(prompt.is_question());
        assert_eq!(
            prompt.show_again.map(|again| again.key),
            Some("SHOWAGAIN_Zero_Altitude_Warning")
        );
        // The settings' reading: false, or a word `bool.TryParse` refuses, suppresses; true
        // and nothing do not.
        assert!(ShowAgain::suppressed(Some("False")));
        assert!(!ShowAgain::suppressed(Some("True")));
        assert!(!ShowAgain::suppressed(None));
    }

    /// The three boxes and their keys are the C#'s calls.
    #[test]
    fn the_show_again_boxes_are_the_csharps() {
        let Some(source) = crate::config_coverage::source::csharp("GCSViews/FlightPlanner.cs")
        else {
            return;
        };
        assert!(source.contains("Common.MessageShowAgain(\"Measure Dist\","));
        assert!(source.contains("Common.MessageShowAgain(\"FlightPlan Fence\", \"Please use the Polygon drawing tool to draw \" +"));
        assert!(
            source.contains("Strings.ZeroAltWarningTitle + (is_arduplane ? \" Plane\" : \"\")")
        );
        assert_eq!(FENCE_KEY, "SHOWAGAIN_FlightPlan_Fence");
        assert!(FENCE_TEXT.starts_with(
            "Please use the Polygon drawing tool to draw Inclusion and Exclusion areas"
        ));
        assert!(FENCE_TEXT.ends_with("convert it to a inclusion or exclusion fence"));
    }

    /// Modify Alt adds, or multiplies with a star, every row's altitude and leaves home alone.
    #[test]
    fn modify_alt_changes_every_row_and_not_home() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::ModifyAlt, None);
        assert_eq!(menus.prompt.as_ref().map(Prompt::value), Some("0"));
        answer(&mut plan, &mut menus, "20");
        let altitudes: Vec<f64> = plan.items().iter().map(|item| item.z).collect();
        assert_eq!(altitudes, vec![120.0, 120.0]);
        choose(&mut plan, &mut menus, MenuAction::ModifyAlt, None);
        answer(&mut plan, &mut menus, "*2");
        let altitudes: Vec<f64> = plan.items().iter().map(|item| item.z).collect();
        assert_eq!(altitudes, vec![240.0, 240.0]);
        assert_eq!(plan.home_text(HomeBox::Alt), "580.00", "home is left alone");
    }

    /// Insert Wp > At Current Position adds a waypoint where the vehicle is, at its altitude.
    #[test]
    fn at_current_position_adds_a_waypoint_at_the_vehicle() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        let flying = MenuContext {
            vehicle: Some((at(-35.4, 149.2), 42.7)),
            ..context()
        };
        menus.open_at(CLICK, canberra(), None);
        menus.choose(&mut plan, MenuAction::InsertAtCurrentPosition, &flying);
        let item = last(&plan);
        assert_eq!(item.command, 16);
        assert_eq!(item.position(), Ok(Some(at(-35.4, 149.2))));
        assert!((item.z - 42.0).abs() < 1e-9, "(int) cs.alt");
        // Without a vehicle it says so and adds nothing.
        choose(
            &mut plan,
            &mut menus,
            MenuAction::InsertAtCurrentPosition,
            None,
        );
        assert_eq!(plan.items().len(), 3);
        assert_eq!(
            menus.prompt.as_ref().map(|p| p.text.as_str()),
            Some("Invalid Location")
        );
    }

    /// A click adds what is being drawn: waypoint, fence corner or rally point.
    #[test]
    fn a_click_adds_what_the_draw_mode_says() {
        let mut plan = Plan::default();
        plan.add_wp_to_map(canberra(), 60.0, AltitudeFrame::Terrain);
        assert_eq!(plan.items().len(), 1);
        assert_eq!(plan.items()[0].frame, FRAME_TERRAIN);
        plan.set_draw_mode(DrawMode::Fence);
        plan.add_wp_to_map(canberra(), 60.0, AltitudeFrame::Relative);
        assert_eq!(plan.fence().len(), 1);
        plan.set_draw_mode(DrawMode::Rally);
        plan.add_wp_to_map(canberra(), 60.0, AltitudeFrame::Relative);
        assert_eq!(plan.rally().len(), 1);
        assert_eq!(plan.items().len(), 1, "only the first click was a waypoint");
    }

    // ---- The panel boxes: TXT_WPRad, TXT_loiterrad, TXT_DefaultAlt, CHK_splinedefault ----

    fn key(key: &str, key_char: Option<&str>) -> gpui::KeyDownEvent {
        gpui::KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: key.to_owned(),
                key_char: key_char.map(ToOwned::to_owned),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    /// Types `text` into a panel box a character at a time, as the keyboard would.
    fn type_into(plan: &mut Plan, which: PanelBox, text: &str) {
        for character in text.chars() {
            let character = character.to_string();
            plan.panel_key(which, &key(&character, Some(&character)));
        }
    }

    /// The boxes start as the `.resx` has them, left to right under their labels.
    #[test]
    fn the_panel_boxes_start_as_the_resx_has_them() {
        let plan = Plan::default();
        let texts: Vec<(&str, &str)> = PanelBox::ALL
            .iter()
            .map(|which| (which.label(), plan.panel_text(*which)))
            .collect();
        assert_eq!(
            texts,
            vec![
                ("WP Radius", "30"),
                ("Loiter Radius", "45"),
                ("Default Alt", "100"),
                ("Alt Warn", "0")
            ]
        );
        assert!(plan.loiter_enabled());
        assert!(!plan.spline());
    }

    /// KeyPress: digits in every box, a point only in WP Radius, a minus only in Loiter Radius,
    /// and nothing else; backspace always.
    #[test]
    fn each_box_takes_the_characters_its_key_press_lets_through() {
        let mut plan = Plan::default();
        for which in PanelBox::ALL {
            plan.set_panel_text(which, "");
            type_into(&mut plan, which, "1.-2a 3");
        }
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "1.23");
        assert_eq!(plan.panel_text(PanelBox::LoiterRadius), "1-23");
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "123");
        plan.panel_key(PanelBox::DefaultAlt, &key("backspace", None));
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "12");
    }

    /// Leave: what does not parse goes back to 100 and 45; WP Radius only when empty, to the
    /// startup value.
    #[test]
    fn leaving_a_box_that_does_not_parse_puts_a_number_back() {
        let mut plan = Plan::default();
        for which in PanelBox::ALL {
            plan.set_panel_text(which, "");
            plan.panel_leave(which);
        }
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "5.0");
        assert_eq!(plan.panel_text(PanelBox::LoiterRadius), "45");
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "100");

        // Not empty and not a number: WP Radius keeps it, the other two do not.
        plan.set_panel_text(PanelBox::WpRadius, "1.2.3");
        plan.set_panel_text(PanelBox::LoiterRadius, "-");
        plan.set_panel_text(PanelBox::DefaultAlt, "7");
        for which in PanelBox::ALL {
            plan.panel_leave(which);
        }
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "1.2.3");
        assert_eq!(plan.panel_text(PanelBox::LoiterRadius), "45");
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "7", "a number stays");
    }

    /// config(false): the saved texts, and WP Radius's becomes the one an emptied box gets back.
    #[test]
    fn the_saved_boxes_come_back_from_config_xml() {
        let mut config = mp_settings::Config::default();
        config.set("TXT_WPRad", "12");
        config.set("TXT_loiterrad", "80");
        config.set("TXT_DefaultAlt", "120");
        let mut plan = Plan::default();
        plan.apply_panel_config(Some(&config));
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "12");
        assert_eq!(plan.panel_text(PanelBox::LoiterRadius), "80");
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "120");
        plan.set_panel_text(PanelBox::WpRadius, "");
        plan.panel_leave(PanelBox::WpRadius);
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "12");
        // No config.xml: the .resx texts stay.
        let mut plan = Plan::default();
        plan.apply_panel_config(None);
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "100");
    }

    fn params(list: &[(&str, f64)]) -> Vec<(String, f64)> {
        list.iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// setWPParams: the last radius name the vehicle has wins, `{0:N2}`; WPNAV_RADIUS is
    /// centimetres; Loiter Radius from LOITER_RADIUS or WP_LOITER_RAD, disabled without either.
    #[test]
    fn the_vehicles_radii_fill_the_boxes() {
        let mut plan = Plan::default();
        // A copter of 4.8: WP_RADIUS_M, and no loiter radius at all.
        plan.set_wp_params(&params(&[("WP_RADIUS_M", 2.0), ("ANGLE_MAX", 3000.0)]));
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "2.00");
        assert!(!plan.loiter_enabled());
        assert_eq!(
            plan.panel_text(PanelBox::LoiterRadius),
            "45",
            "left as it was"
        );

        // An older copter: centimetres.
        plan.set_wp_params(&params(&[("WPNAV_RADIUS", 250.0)]));
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "2.50");

        // A plane: WP_RADIUS and WP_LOITER_RAD.
        plan.set_wp_params(&params(&[("WP_RADIUS", 90.0), ("WP_LOITER_RAD", 60.0)]));
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "90.00");
        assert_eq!(plan.panel_text(PanelBox::LoiterRadius), "60");
        assert!(plan.loiter_enabled());

        // WP_RADIUS_M is read last, so it wins over WP_RADIUS.
        plan.set_wp_params(&params(&[("WP_RADIUS", 90.0), ("WP_RADIUS_M", 3.0)]));
        assert_eq!(plan.panel_text(PanelBox::WpRadius), "3.00");

        // No vehicle: nothing to read, and Loiter Radius disabled.
        plan.set_wp_params(&[]);
        assert!(!plan.loiter_enabled());
    }

    /// A disabled Loiter Radius takes no typing.
    #[test]
    fn a_disabled_loiter_radius_takes_nothing() {
        let mut plan = Plan::default();
        plan.set_wp_params(&[]);
        type_into(&mut plan, PanelBox::LoiterRadius, "9");
        assert_eq!(plan.panel_text(PanelBox::LoiterRadius), "45");
    }

    /// `.fen` files and `.param` files are written with the same `double.ToString()`, ported twice
    /// because `mp-mission` does not depend on `mp-params`; the two must agree.
    #[test]
    fn the_two_ports_of_double_to_string_agree() {
        for value in [
            0.0,
            -0.0,
            1.0,
            0.3,
            f64::from(0.3_f32),
            -31.25,
            -35.363_262_345_678_91,
            149.165_237_4,
            0.0001,
            0.000_089,
            0.000_01,
            1e-7,
            -0.000_000_15,
            110_000.0,
            123_456_789_012_345.0,
            1_234_567_890_123_456.0,
        ] {
            assert_eq!(
                mp_mission::fence_file::invariant_double(value),
                mp_params::param_file::invariant_double(value),
                "{value}"
            );
        }
    }

    /// `{0:N2}`: two decimals, halves away from zero, thousands grouped.
    #[test]
    fn n2_is_two_decimals_with_thousands_grouped() {
        assert_eq!(number_n2(2.0), "2.00");
        assert_eq!(number_n2(2.345), "2.35");
        assert_eq!(number_n2(0.125), "0.13");
        assert_eq!(number_n2(1234.5), "1,234.50");
        assert_eq!(number_n2(1_234_567.0), "1,234,567.00");
        assert_eq!(number_n2(-3.0), "-3.00");
        assert_eq!(number_n2(-0.001), "0.00");
        assert_eq!(number_n2(0.005), "0.01");
        assert_eq!(number_n2(0.0049), "0.00");
        assert_eq!(number_n2(f64::from(0.3_f32)), "0.30");
        assert_eq!(
            number_n2(1.005),
            "1.01",
            "the fifteen-digit decimal, not the binary value"
        );
    }

    /// setfromMap's altitude: Default Alt, unless the handler passed one; 0 in the box is 50, or
    /// 15 on a copter, whatever was passed; and a box that is not a whole number refuses.
    #[test]
    fn a_new_row_takes_default_alt_as_set_from_map_does() {
        let mut plan = Plan::default();
        assert_eq!(plan.new_row_altitude(0.0, false), Ok(100.0));
        assert_eq!(
            plan.new_row_altitude(37.0, false),
            Ok(37.0),
            "the passed one wins"
        );
        plan.set_panel_text(PanelBox::DefaultAlt, "0");
        assert_eq!(plan.new_row_altitude(0.0, false), Ok(50.0));
        assert_eq!(plan.new_row_altitude(37.0, true), Ok(15.0));
        plan.set_panel_text(PanelBox::DefaultAlt, "");
        assert_eq!(plan.new_row_altitude(0.0, false), Err(DEFAULT_ALT_INVALID));
        plan.set_panel_text(PanelBox::DefaultAlt, "12.5");
        assert_eq!(plan.new_row_altitude(0.0, false), Err(DEFAULT_ALT_INVALID));
    }

    /// Activate: a Default Alt `int.Parse` refuses becomes 50, and it is said.
    #[test]
    fn activating_fixes_a_default_alt_that_is_not_a_whole_number() {
        let mut plan = Plan::default();
        assert_eq!(plan.check_default_alt(), None);
        plan.set_panel_text(PanelBox::DefaultAlt, "abc");
        assert_eq!(plan.check_default_alt(), Some(DEFAULT_ALT_FIX));
        assert_eq!(plan.panel_text(PanelBox::DefaultAlt), "50");
    }

    /// The menu's rows take Default Alt, and refuse with the C#'s words when it will not parse.
    #[test]
    fn the_menus_rows_take_default_alt() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        plan.set_panel_text(PanelBox::DefaultAlt, "120");
        choose(&mut plan, &mut menus, MenuAction::LoiterForever, None);
        assert!((last(&plan).z - 120.0).abs() < 1e-9);
        choose(&mut plan, &mut menus, MenuAction::LoiterTime, None);
        answer(&mut plan, &mut menus, "5");
        assert!((last(&plan).z - 120.0).abs() < 1e-9);

        plan.set_panel_text(PanelBox::DefaultAlt, "");
        choose(&mut plan, &mut menus, MenuAction::SetRoi, None);
        assert_eq!(plan.items().len(), 2, "nothing added");
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some(DEFAULT_ALT_INVALID)
        );
    }

    /// Spline: a click adds a SPLINE_WAYPOINT; a fence corner is still a fence corner.
    #[test]
    fn spline_makes_a_click_a_spline_waypoint() {
        let mut plan = Plan::default();
        plan.set_spline(true);
        plan.add_wp_to_map(canberra(), 100.0, AltitudeFrame::Relative);
        assert_eq!(commands(&plan), vec![cmd::SPLINE_WAYPOINT]);
        plan.set_spline(false);
        plan.add_wp_to_map(canberra(), 100.0, AltitudeFrame::Relative);
        assert_eq!(commands(&plan), vec![cmd::SPLINE_WAYPOINT, cmd::WAYPOINT]);
        plan.set_spline(true);
        plan.set_draw_mode(DrawMode::Fence);
        plan.add_wp_to_map(canberra(), 100.0, AltitudeFrame::Relative);
        assert_eq!(plan.items().len(), 2);
    }

    /// saveWPs' "Setting params": the radius into all three names, WPNAV_RADIUS in centimetres,
    /// then the loiter radius into the first of LOITER_RAD and WP_LOITER_RAD.
    #[test]
    fn a_write_sets_the_radii_as_save_wps_does() {
        let mut plan = Plan::default();
        plan.set_panel_text(PanelBox::WpRadius, "3.5");
        plan.set_panel_text(PanelBox::LoiterRadius, "-60");
        let steps = plan.wp_param_steps().expect("the radius parses");
        let names: Vec<(Vec<&str>, f64)> = steps
            .iter()
            .map(|step| (step.names.clone(), step.value))
            .collect();
        assert_eq!(
            names,
            vec![
                (vec!["WP_RADIUS"], 3.5),
                (vec!["WP_RADIUS_M"], 3.5),
                (vec!["WPNAV_RADIUS"], 350.0),
                (vec!["LOITER_RAD", "WP_LOITER_RAD"], -60.0),
            ]
        );
        assert!(matches!(steps[0].on_timeout, OnTimeout::Stop { .. }));
        assert_eq!(
            steps[3].on_timeout,
            OnTimeout::CarryOn,
            "the loiter set is caught"
        );

        plan.set_panel_text(PanelBox::WpRadius, ".");
        assert_eq!(plan.wp_param_steps(), Err(FORMAT_EXCEPTION));
        // A loiter radius that will not parse is passed over, as the C# catches it.
        plan.set_panel_text(PanelBox::WpRadius, "2");
        plan.set_panel_text(PanelBox::LoiterRadius, "-");
        assert_eq!(plan.wp_param_steps().map(|steps| steps.len()), Ok(3));
    }

    // ---- A row of parameter sets ----

    use mp_link::requests::RequestOutcome;

    /// Runs a row to its end, answering each set from `answers` by name; a name not listed is
    /// one that could not be sent.
    fn run(writes: &mut ParamWrites, answers: &[(&str, RequestOutcome)]) -> Vec<String> {
        let mut sent = Vec::new();
        while let Some((name, value)) = writes.due() {
            sent.push(format!("{name}={value}"));
            let outcome = answers
                .iter()
                .find(|(answered, _)| *answered == name)
                .map(|(_, outcome)| *outcome);
            writes.answer(outcome);
        }
        sent
    }

    fn accepted(value: f32) -> RequestOutcome {
        RequestOutcome::Accepted {
            value: Some(mp_params::ParamValue::from_ardupilot(
                value,
                mp_params::ParamType::Real32,
            )),
        }
    }

    /// Each set goes out only once the last is answered; a name the vehicle does not have is
    /// passed over for the next name in its step, and then for the next step.
    #[test]
    fn a_row_of_sets_goes_one_at_a_time_as_the_c_sharp_makes_them() {
        let mut plan = Plan::default();
        plan.set_panel_text(PanelBox::WpRadius, "3");
        plan.set_panel_text(PanelBox::LoiterRadius, "70");
        let mut writes =
            ParamWrites::new(plan.wp_param_steps().expect("steps"), AfterWrites::Nothing);
        // A copter of 4.8: WP_RADIUS_M only.
        let sent = run(
            &mut writes,
            &[
                ("WP_RADIUS", RequestOutcome::UnknownParameter),
                ("WP_RADIUS_M", accepted(3.0)),
                ("WPNAV_RADIUS", RequestOutcome::UnknownParameter),
                ("LOITER_RAD", RequestOutcome::UnknownParameter),
                ("WP_LOITER_RAD", RequestOutcome::UnknownParameter),
            ],
        );
        assert_eq!(
            sent,
            vec![
                "WP_RADIUS=3",
                "WP_RADIUS_M=3",
                "WPNAV_RADIUS=300",
                "LOITER_RAD=70",
                "WP_LOITER_RAD=70"
            ]
        );
        assert_eq!(writes.end(), Some(&WritesEnd::Done));
        assert_eq!(
            writes.results(),
            "WP_RADIUS=unknown,WP_RADIUS_M=3,WPNAV_RADIUS=unknown,LOITER_RAD=unknown,WP_LOITER_RAD=unknown"
        );
    }

    /// `setParam(string[])` stops at the first name the vehicle has.
    #[test]
    fn the_first_name_the_vehicle_has_takes_the_value() {
        let mut writes = ParamWrites::new(
            vec![ParamStep {
                names: vec!["LOITER_RAD", "WP_LOITER_RAD"],
                value: 80.0,
                on_timeout: OnTimeout::CarryOn,
            }],
            AfterWrites::Nothing,
        );
        let sent = run(&mut writes, &[("LOITER_RAD", RequestOutcome::Unchanged)]);
        assert_eq!(sent, vec!["LOITER_RAD=80"], "WP_LOITER_RAD is never tried");
        assert_eq!(writes.results(), "LOITER_RAD=80 unchanged");
    }

    /// A timeout where the C# stops ends the row with its message; where it catches, the row goes
    /// on.
    #[test]
    fn a_timeout_stops_the_row_only_where_the_c_sharp_does() {
        let mut writes = ParamWrites::new(
            vec![
                ParamStep {
                    names: vec!["LOITER_RAD"],
                    value: 1.0,
                    on_timeout: OnTimeout::CarryOn,
                },
                ParamStep::one(
                    "FENCE_ACTION",
                    0.0,
                    OnTimeout::Stop {
                        title: "",
                        text: "Failed to set FENCE_ACTION".to_owned(),
                    },
                ),
                ParamStep::one("FENCE_TOTAL", 0.0, OnTimeout::CarryOn),
            ],
            AfterWrites::ClearFence,
        );
        let sent = run(
            &mut writes,
            &[
                ("LOITER_RAD", RequestOutcome::TimedOut),
                ("FENCE_ACTION", RequestOutcome::TimedOut),
            ],
        );
        assert_eq!(
            sent,
            vec!["LOITER_RAD=1", "FENCE_ACTION=0"],
            "FENCE_TOTAL never goes"
        );
        assert_eq!(
            writes.end(),
            Some(&WritesEnd::Stopped {
                title: "",
                text: "Failed to set FENCE_ACTION".to_owned()
            })
        );
        assert_eq!(writes.after(), AfterWrites::ClearFence);
    }

    /// With no vehicle every set is one that could not be sent, and the row still ends - as the
    /// C#'s `setParam` returns false against an empty parameter list.
    #[test]
    fn with_no_vehicle_the_row_ends_having_sent_nothing() {
        let mut writes = ParamWrites::new(
            vec![ParamStep::one("FENCE_ENABLE", 0.0, OnTimeout::CarryOn)],
            AfterWrites::ClearFence,
        );
        run(&mut writes, &[]);
        assert_eq!(writes.end(), Some(&WritesEnd::Done));
        assert_eq!(writes.results(), "FENCE_ENABLE=unknown");
        // An empty row is done at once.
        assert_eq!(
            ParamWrites::new(Vec::new(), AfterWrites::Nothing).end(),
            Some(&WritesEnd::Done)
        );
    }

    fn transfer(finished: bool, failed: bool) -> crate::telemetry::TransferStatus {
        crate::telemetry::TransferStatus {
            label: String::new(),
            fraction: 0.0,
            finished,
            failed,
        }
    }

    /// The sets follow the upload they belong to: not the transfer that was already finished when
    /// Write was pressed, unless it holds exactly what was sent.
    #[test]
    fn the_sets_wait_for_this_writes_upload() {
        let items = vec![cmd::waypoint(canberra(), 100.0, FRAME_RELATIVE)];
        let earlier = vec![cmd::waypoint(at(-35.0, 149.0), 50.0, FRAME_RELATIVE)];
        let mut pending = PendingWrite::new(items.clone(), Ok(Vec::new()));
        assert_eq!(pending.upload_ended(None, &[]), None, "nothing has started");
        assert_eq!(
            pending.upload_ended(Some(&transfer(true, false)), &earlier),
            None,
            "an earlier transfer's end is not this one's"
        );
        assert_eq!(
            pending.upload_ended(Some(&transfer(false, false)), &items),
            None
        );
        assert_eq!(
            pending.upload_ended(Some(&transfer(true, false)), &items),
            Some(true)
        );

        // Too quick to be seen running, but holding what was sent.
        let mut pending = PendingWrite::new(items.clone(), Ok(Vec::new()));
        assert_eq!(
            pending.upload_ended(Some(&transfer(true, false)), &items),
            Some(true)
        );
        // A failed upload sets nothing.
        let mut pending = PendingWrite::new(items.clone(), Ok(Vec::new()));
        pending.upload_ended(Some(&transfer(false, false)), &items);
        assert_eq!(
            pending.upload_ended(Some(&transfer(true, true)), &items),
            Some(false)
        );
    }

    // ---- Home on the map ----

    /// The planner draws home at the boxes once all three parse, and not before.
    #[test]
    fn the_planning_map_draws_home_at_the_boxes() {
        let mut plan = Plan::default();
        assert_eq!(planner_map_home(&plan), None);
        type_home(&mut plan, "-35.36", "149.16", "");
        assert_eq!(planner_map_home(&plan), None, "an empty ASL box is no home");
        type_home(&mut plan, "-35.36", "149.16", "584");
        assert_eq!(planner_map_home(&plan), Some(at(-35.36, 149.16)));
        type_home(&mut plan, "-35.36", "x", "584");
        assert_eq!(planner_map_home(&plan), None);
        // `Tag = "H"` makes even 0,0 a home to draw.
        type_home(&mut plan, "0", "0", "0");
        assert_eq!(planner_map_home(&plan), Some(at(0.0, 0.0)));
    }

    /// The flight screen draws the vehicle's home once its mission is held, falling back on the
    /// planned home while the vehicle's is 0,0.
    #[test]
    fn the_flight_map_draws_the_vehicles_home_with_its_mission() {
        let vehicle = Some(at(-35.3632, 149.1652));
        let planned = Home {
            lat: -27.5,
            lng: 153.0,
            alt: 8.0,
        };
        assert_eq!(
            flight_map_home(vehicle, false, planned),
            None,
            "no mission, no home"
        );
        assert_eq!(flight_map_home(vehicle, true, planned), vehicle);
        assert_eq!(
            flight_map_home(None, true, planned),
            Some(at(-27.5, 153.0)),
            "HOME_POSITION not yet sent"
        );
        assert_eq!(
            flight_map_home(Some(at(0.0, 0.0)), true, planned),
            Some(at(-27.5, 153.0))
        );
        assert_eq!(flight_map_home(None, true, Home::default()), None);
    }

    // ---- The Geo-Fence drop-down ----

    /// Set Return Location puts the marker where the menu was opened.
    #[test]
    fn set_return_location_puts_the_marker_at_the_click() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        assert_eq!(plan.fence_return(), None);
        choose(&mut plan, &mut menus, MenuAction::SetReturnLocation, None);
        assert_eq!(plan.fence_return(), Some(canberra()));
        assert!(menus.prompt.is_none());
        assert!(menus.open.is_none(), "choosing closes the menu");
    }

    /// Save to File without a return location says so before any file is asked for; with one,
    /// it asks for a `.fen`.
    #[test]
    fn save_to_file_needs_a_return_location_first() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::FenceSaveToFile, None);
        assert_eq!(
            menus.prompt.as_ref().map(|prompt| prompt.text.as_str()),
            Some(SET_RETURN_LOCATION)
        );
        menus.cancel(&mut plan);
        choose(&mut plan, &mut menus, MenuAction::SetReturnLocation, None);
        choose(&mut plan, &mut menus, MenuAction::FenceSaveToFile, None);
        let prompt = menus.prompt.as_ref().expect("the file dialog");
        assert_eq!(
            (prompt.title, prompt.text.as_str()),
            (SAVE_FILE, FENCE_FILTER)
        );
        menus
            .prompt
            .as_mut()
            .and_then(|prompt| prompt.field.as_mut())
            .expect("a name field")
            .set("square");
        assert_eq!(
            menus.submit(&mut plan, &context()),
            Some(FileRequest::SaveFence("square".to_owned()))
        );
    }

    /// Load from File asks for a `.fen`, and hands the name back to be read.
    #[test]
    fn load_from_file_asks_for_a_fence_file() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::FenceLoadFromFile, None);
        let prompt = menus.prompt.as_ref().expect("the file dialog");
        assert_eq!(
            (prompt.title, prompt.text.as_str()),
            (OPEN_FILE, FENCE_FILTER)
        );
        menus
            .prompt
            .as_mut()
            .and_then(|prompt| prompt.field.as_mut())
            .expect("a name field")
            .set("square.fen");
        assert_eq!(
            menus.submit(&mut plan, &context()),
            Some(FileRequest::LoadFence("square.fen".to_owned()))
        );
    }

    /// The file is the return location, then the drawn polygon closed - or the geofence when
    /// nothing is drawn - and it reads back into the drawn polygon and the marker.
    #[test]
    fn a_fence_file_round_trips_through_the_drawn_polygon() {
        let square = [
            at(-35.364, 149.164),
            at(-35.364, 149.168),
            at(-35.361, 149.168),
            at(-35.361, 149.164),
        ];
        let mut plan = Plan::default();
        plan.set_fence_return(at(-35.3625, 149.166));
        assert_eq!(
            plan.fence_file(),
            Err(FENCE_FILE_FAILED),
            "nothing drawn, no geofence"
        );

        // The geofence, when nothing is drawn.
        for corner in &square {
            plan.add_fence_vertex(*corner);
        }
        let file = plan.fence_file().expect("a file");
        assert_eq!(file.vertices, square.to_vec());
        // The drawn polygon, when there is one.
        plan.add_area_vertex(square[0]);
        plan.add_area_vertex(square[1]);
        plan.add_area_vertex(square[2]);
        let file = plan.fence_file().expect("a file");
        assert_eq!(file.vertices.len(), 3);

        let text = mp_mission::fence_file::write_fence(&file);
        // Return, three corners, the first again.
        assert_eq!(
            text.lines().filter(|line| !line.starts_with('#')).count(),
            5
        );

        let mut other = Plan::default();
        other.adopt_fence_file(mp_mission::fence_file::read_fence(&text));
        assert_eq!(other.polygon(), &square[..3]);
        assert_eq!(other.fence_return(), Some(at(-35.3625, 149.166)));
        // A file with no lines leaves the marker where it was, and empties the polygon.
        other.adopt_fence_file(mp_mission::fence_file::read_fence(
            "#nothing
",
        ));
        assert!(other.polygon().is_empty());
        assert_eq!(other.fence_return(), Some(at(-35.3625, 149.166)));
    }

    /// A typed answer is a path as a file dialog takes one (the owner, 2026-10-04: Load and
    /// Save File are to reach any file): a name or a relative path from the folder the dialog
    /// opened in, an absolute path as it is, `~/` from home; the filter's extension added when
    /// the file has none; nothing when empty or a folder.
    #[test]
    fn a_dialog_answer_is_a_path_as_a_file_dialog_takes_it() {
        let folder = mp_os::temp_dir().join(format!("mp-dialog-path-{}", mp_os::process_id()));
        std::fs::create_dir_all(folder.join("sub")).unwrap();
        assert_eq!(
            dialog_path("square", "fen", &folder),
            Some(folder.join("square.fen"))
        );
        assert_eq!(
            dialog_path(" square.fen ", "fen", &folder),
            Some(folder.join("square.fen"))
        );
        assert_eq!(
            dialog_path("sub/square", "fen", &folder),
            Some(folder.join("sub").join("square.fen"))
        );
        let elsewhere = mp_os::temp_dir().join("elsewhere.waypoints");
        assert_eq!(
            dialog_path(&elsewhere.display().to_string(), "waypoints", &folder),
            Some(elsewhere)
        );
        if let Some(home) = home_directory() {
            assert_eq!(
                dialog_path("~/flight.txt", "waypoints", &folder),
                Some(home.join("flight.txt"))
            );
        }
        assert_eq!(dialog_path("  ", "fen", &folder), None);
        assert_eq!(dialog_path("sub", "fen", &folder), None, "a folder is not a file");
        assert_eq!(dialog_path("sub/", "fen", &folder), None);
        std::fs::remove_dir_all(&folder).unwrap();
    }

    /// The dialog's list: `..`, the folders, then the files of its filter, each by name and
    /// hidden ones left out; of the folder the typed answer names or sits in, else its own; and
    /// a row puts a file of its own folder in the box by name, anything else by path.
    #[test]
    fn a_file_dialog_lists_its_folder() {
        let folder = mp_os::temp_dir().join(format!("mp-dialog-list-{}", mp_os::process_id()));
        for sub in ["b-sub", ".hidden"] {
            std::fs::create_dir_all(folder.join(sub)).unwrap();
        }
        for file in ["z.waypoints", "a.TXT", "notes.md", ".x.waypoints"] {
            std::fs::write(folder.join(file), "").unwrap();
        }
        std::fs::write(folder.join("b-sub").join("deep.waypoints"), "").unwrap();
        let names = |entries: &[DialogEntry]| -> Vec<String> {
            entries.iter().map(|entry| entry.name.clone()).collect()
        };
        let (listed, entries) = dialog_listing("", &folder, &["waypoints", "txt"]);
        assert_eq!(listed, folder);
        assert_eq!(names(&entries), ["..", "b-sub", "a.TXT", "z.waypoints"]);
        assert_eq!(
            entries.iter().map(|entry| entry.folder).collect::<Vec<_>>(),
            [true, true, false, false]
        );
        assert_eq!(dialog_answer(&entries[3], &folder), "z.waypoints");
        assert!(dialog_answer(&entries[1], &folder).starts_with(&folder.join("b-sub").display().to_string()));
        let (listed, entries) = dialog_listing("b-sub/", &folder, &["waypoints"]);
        assert_eq!(listed, folder.join("b-sub"));
        assert_eq!(names(&entries), ["..", "deep.waypoints"]);
        assert_eq!(
            dialog_answer(&entries[1], &folder),
            folder.join("b-sub").join("deep.waypoints").display().to_string()
        );
        let typed = folder.join("b-sub").join("new.waypoints");
        let (listed, _) = dialog_listing(&typed.display().to_string(), &folder, &["waypoints"]);
        assert_eq!(listed, folder.join("b-sub"), "a file's own folder");
        let (_, entries) = dialog_listing("", &folder, &[]);
        assert_eq!(names(&entries), ["..", "b-sub"], "a folder dialog lists folders");
        std::fs::remove_dir_all(&folder).unwrap();
    }

    /// Clear's ending: the geofence goes, the return marker stays, the drawn corners stay but go
    /// off the map until the polygon is next drawn.
    #[test]
    fn clearing_the_geofence_keeps_the_return_marker_and_the_drawn_corners() {
        let mut plan = Plan::default();
        plan.set_fence_return(canberra());
        plan.add_fence_vertex(at(-35.364, 149.164));
        plan.add_area_vertex(at(-35.364, 149.164));
        plan.clear_geofence();
        assert!(plan.fence().is_empty());
        assert_eq!(plan.fence_return(), Some(canberra()));
        assert_eq!(plan.polygon().len(), 1);
        assert!(plan.shown_polygon().is_empty(), "off the map");
        plan.add_area_vertex(at(-35.365, 149.165));
        assert_eq!(
            plan.shown_polygon().len(),
            2,
            "both corners, back with the next"
        );
    }

    // ---- Home is item 0: the Home Location boxes against BUT_write_Click, saveWPs,
    // ---- savewaypoints, processToScreen and updateHomeText ----

    /// Types a home into the three boxes, as an operator does.
    fn type_home(plan: &mut Plan, lat: &str, lng: &str, alt: &str) {
        plan.set_home_text(HomeBox::Lat, lat.to_owned());
        plan.set_home_text(HomeBox::Lng, lng.to_owned());
        plan.set_home_text(HomeBox::Alt, alt.to_owned());
    }

    /// The defect this rule exists for: a mission drawn on an empty map, with no home anywhere,
    /// starts at waypoint 1. The first click is never item 0.
    #[test]
    fn the_first_click_on_an_empty_map_is_waypoint_one_not_home() {
        let mut plan = Plan::default();
        plan.add_wp_to_map(canberra(), 50.0, AltitudeFrame::Relative);
        plan.add_wp_to_map(at(-35.364, 149.166), 50.0, AltitudeFrame::Relative);
        assert_eq!(plan.items()[0].seq, 1);
        assert!(plan.items().iter().all(|item| !item.is_home()));
        assert_eq!(plan.home(), None, "nothing has set a home");

        // The file still starts with a home record - savewaypoints' blank one - so reading it
        // back gives both clicks back as rows.
        let file = plan.waypoints_file();
        let mut lines = file.lines();
        assert_eq!(lines.next(), Some("QGC WPL 110"));
        assert_eq!(lines.next(), Some(mp_mission::waypoints::BLANK_HOME_RECORD));
        assert!(
            lines
                .next()
                .is_some_and(|line| line.starts_with("1\t0\t3\t16\t")),
            "{file}"
        );
        let read = mp_mission::read_waypoints(&file).expect("parses");
        let mut again = Plan::default();
        assert_eq!(again.adopt_from_file("drawn.waypoints", &read), None);
        assert_eq!(again.items(), plan.items());
    }

    /// Write refuses without a home, in the C#'s words, whatever the autopilot.
    #[test]
    fn writing_without_a_home_is_refused() {
        let mut plan = Plan::default();
        plan.add_waypoint(canberra(), 50.0);
        assert_eq!(plan.vehicle_mission(true), Err(HOME_INVALID));
        assert_eq!(plan.vehicle_mission(false), Err(HOME_INVALID));
        assert_eq!(HOME_INVALID, "Your home location is invalid");
        // Any one box that does not parse is no home.
        type_home(&mut plan, "-35.363262", "149.165237", "high");
        assert_eq!(plan.vehicle_mission(true), Err(HOME_INVALID));
    }

    /// With a home in the boxes, what an ArduPilot is sent has it at item 0 - a GLOBAL waypoint
    /// at the boxes' position and altitude - and the rows after it from 1.
    #[test]
    fn a_written_mission_has_the_boxes_home_at_zero() {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.361, 149.161), 50.0);
        plan.add_waypoint(at(-35.362, 149.162), 60.0);
        type_home(&mut plan, "-35.363262", "149.165237", "584.00");

        let sent = plan.vehicle_mission(true).expect("a home");
        assert_eq!(sent.len(), 3);
        let home = sent[0];
        assert_eq!((home.seq, home.frame, home.command), (0, 0, 16));
        assert_eq!((home.x, home.y, home.z), (-35.363_262, 149.165_237, 584.0));
        assert!(
            (sent[1].x - -35.361).abs() < 1e-12,
            "the first row is item 1"
        );
        assert_eq!(
            sent.iter().map(|item| item.seq).collect::<Vec<_>>(),
            vec![0, 1, 2]
        );

        // Any other autopilot is sent the rows alone.
        let sent = plan.vehicle_mission(false).expect("a home");
        assert_eq!(sent.len(), 2);
        assert!((sent[0].x - -35.361).abs() < 1e-12);

        // And the file has it as record 0, in savewaypoints' format.
        let file = plan.waypoints_file();
        assert_eq!(
            file.lines().nth(1),
            Some("0\t1\t0\t16\t0\t0\t0\t0\t-35.3632620\t149.1652370\t584.000000\t1")
        );
        assert_eq!(
            written_facts(&file),
            (
                "0:0:16,1:3:16,2:3:16".to_owned(),
                "-35.3632620,149.1652370,584.000000".to_owned()
            )
        );
    }

    /// What the facts say is written comes from the text Save File writes: with nothing planned,
    /// that is still a home record, the blank one.
    #[test]
    fn the_written_facts_read_the_file() {
        let plan = Plan::default();
        assert_eq!(
            written_facts(&plan.waypoints_file()),
            ("0:0:0".to_owned(), "0,0,0".to_owned())
        );
    }

    /// The boxes show the vehicle's home when it has sent one, else the planned home, else stay
    /// as they are - and whatever they are set to becomes the planned home.
    #[test]
    fn activating_shows_the_vehicles_home_else_the_planned_one() {
        let mut plan = Plan::default();
        plan.update_home_text(None);
        assert_eq!(plan.home(), None, "neither: the boxes stay empty");

        let planned = Home {
            lat: -27.509_708_302_151_7,
            lng: 153.015_389_442_444,
            alt: 8.1,
        };
        plan.set_planned_home(planned);
        plan.update_home_text(None);
        assert_eq!(plan.home_text(HomeBox::Lat), "-27.5097083021517");
        assert_eq!(plan.home_text(HomeBox::Lng), "153.015389442444");
        assert_eq!(plan.home_text(HomeBox::Alt), "8.10");

        // A vehicle's home at 0,0 is no home.
        plan.update_home_text(Some(Home::default()));
        assert_eq!(plan.home_text(HomeBox::Lat), "-27.5097083021517");

        plan.update_home_text(Some(VEHICLE_HOME));
        assert_eq!(plan.home(), Some(VEHICLE_HOME));
        assert_eq!(plan.home_text(HomeBox::Alt), "580.00");
        assert_eq!(
            plan.planned_home(),
            VEHICLE_HOME,
            "TextChanged made it the planned home"
        );
    }

    /// Typing moves the planned home when the text parses and leaves it when it does not.
    #[test]
    fn a_box_that_parses_sets_the_planned_home() {
        let mut plan = Plan::default();
        type_home(&mut plan, "-35.5", "149.5", "600");
        assert_eq!(
            plan.planned_home(),
            Home {
                lat: -35.5,
                lng: 149.5,
                alt: 600.0
            }
        );
        plan.set_home_text(HomeBox::Lat, "-35.5x".to_owned());
        assert!((plan.planned_home().lat - -35.5).abs() < f64::EPSILON);
        assert_eq!(
            plan.home(),
            None,
            "but the write takes the box, which does not parse"
        );
    }

    /// Reading takes item 0 as home and never as a row, and offers it when the boxes differ.
    #[test]
    fn reading_a_mission_takes_item_zero_as_home() {
        let rows = [
            cmd::waypoint(at(-35.361, 149.161), 100.0, FRAME_RELATIVE),
            cmd::waypoint(at(-35.362, 149.162), 100.0, FRAME_RELATIVE),
        ];
        let mut plan = Plan::default();
        let offer = plan.adopt_from_file("x.waypoints", &downloaded(&rows));
        assert_eq!(offer, Some(VEHICLE_HOME), "empty boxes: asked");
        assert_eq!(plan.items().len(), 2);
        assert_eq!(plan.items()[0].seq, 1);
        assert_eq!(plan.home(), None, "not taken until the operator says Yes");

        // The boxes already say that latitude: not asked.
        plan.reset_home_to(VEHICLE_HOME);
        assert_eq!(plan.adopt_from_vehicle(&downloaded(&rows)), None);

        // A blank home - latitude 0 - is never offered.
        let mut blank = downloaded(&rows);
        blank[0] = MissionItem::default();
        assert_eq!(Plan::default().adopt_from_file("x", &blank), None);
    }

    /// Yes to "Reset Home to loaded coords" puts the loaded home in the boxes; No leaves them.
    #[test]
    fn yes_to_reset_home_takes_the_loaded_home() {
        let mut plan = Plan::default();
        type_home(&mut plan, "-35.5", "149.5", "600");
        let mut menus = PlanMenus::default();
        let offer = plan
            .adopt_from_vehicle(&downloaded(&[cmd::waypoint(canberra(), 50.0, 3)]))
            .expect("the boxes say otherwise");
        menus.offer_home_reset(offer);
        let question = menus.prompt.as_ref().expect("asked");
        assert_eq!(
            (question.title, question.text.as_str()),
            ("Reset Home Coords", "Reset Home to loaded coords")
        );
        assert!(question.is_question());
        menus.cancel(&mut plan);
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.5", "No");

        menus.offer_home_reset(offer);
        menus.submit(&mut plan, &context());
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.36");
        assert_eq!(plan.home_text(HomeBox::Lng), "149.16");
        assert_eq!(plan.home_text(HomeBox::Alt), "580.00");
    }

    /// Saving and reading back gives the same rows and the same home, and asks nothing.
    #[test]
    fn a_saved_mission_reads_back_whole() {
        let mut plan = Plan::default();
        plan.add_waypoint_in(at(-35.361, 149.161), 50.0, AltitudeFrame::Terrain);
        plan.append(cmd::return_to_launch(FRAME_RELATIVE));
        type_home(&mut plan, "-35.363262", "149.165237", "584.25");
        let read = mp_mission::read_waypoints(&plan.waypoints_file()).expect("parses");
        let mut again = Plan::default();
        type_home(&mut again, "-35.363262", "149.165237", "584.25");
        assert_eq!(again.adopt_from_file("x", &read), None);
        assert_eq!(again.items(), plan.items());
        assert_eq!(again.home(), plan.home());
    }

    /// The Home Location link takes the vehicle's position and altitude above sea level, or says
    /// how to get one.
    #[test]
    fn the_home_location_link_takes_the_vehicles_position() {
        let mut plan = Plan::default();
        assert_eq!(plan.home_from_vehicle(None), Err(HOME_NEEDS_A_FIX));
        assert_eq!(plan.home(), None);
        plan.home_from_vehicle(Some((canberra(), 612.3)))
            .expect("a position");
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.363262");
        assert_eq!(plan.home_text(HomeBox::Lng), "149.165237");
        assert_eq!(plan.home_text(HomeBox::Alt), "612.30");
    }

    /// The planned home `MainV2` starts with comes from the saved boxes, and none off the globe.
    #[test]
    fn the_planned_home_comes_from_the_saved_boxes() {
        assert_eq!(planned_home_from_config(None), Home::default());
        let mut config = mp_settings::Config::default();
        config.set("TXT_homelat", "-27.5097083021517");
        config.set("TXT_homelng", "153.015389442444");
        config.set("TXT_homealt", "8.10");
        assert_eq!(
            planned_home_from_config(Some(&config)),
            Home {
                lat: -27.509_708_302_151_7,
                lng: 153.015_389_442_444,
                alt: 8.1
            }
        );
        config.set("TXT_homealt", "");
        assert!(
            planned_home_from_config(Some(&config)).alt.abs() < f64::EPSILON,
            "GetDouble's default"
        );
        config.set("TXT_homelng", "200");
        assert_eq!(planned_home_from_config(Some(&config)), Home::default());
    }

    /// The boxes carry the resx labels and distinct ids.
    #[test]
    fn the_home_boxes_are_lat_long_and_asl() {
        let labels: Vec<&str> = HomeBox::ALL.iter().map(|which| which.label()).collect();
        assert_eq!(labels, vec!["Lat", "Long", "ASL"]);
        let ids: Vec<&str> = HomeBox::ALL.iter().map(|which| which.id()).collect();
        assert_eq!(ids, vec!["plan-home-lat", "plan-home-lng", "plan-home-alt"]);
    }

    /// A press that closed the menu is swallowed once, at that point only.
    #[test]
    fn the_press_that_closes_the_menu_adds_nothing() {
        let mut menus = PlanMenus::default();
        menus.open_at(CLICK, canberra(), None);
        menus.dismiss((10.0, 20.0));
        assert!(menus.open.is_none());
        assert!(!menus.swallows_press((11.0, 20.0)), "another press");
        menus.open_at(CLICK, canberra(), None);
        menus.dismiss((10.0, 20.0));
        assert!(menus.swallows_press((10.0, 20.0)));
        assert!(!menus.swallows_press((10.0, 20.0)), "only once");
        // A dismissal with no menu open records nothing.
        menus.dismiss((5.0, 5.0));
        assert!(!menus.swallows_press((5.0, 5.0)));
    }

    /// Choosing with no menu open does nothing: a stale click after the menu went away.
    #[test]
    fn choosing_with_no_menu_open_does_nothing() {
        let mut plan = from_vehicle();
        let mut menus = PlanMenus::default();
        menus.choose(&mut plan, MenuAction::Rtl, &context());
        assert_eq!(plan.items().len(), 2);
    }

    /// Every id is distinct, so a script never clicks the wrong entry, and every drop-down lines
    /// up with its entry.
    #[test]
    fn the_menu_ids_are_distinct_and_drop_downs_line_up() {
        let mut ids: Vec<&str> = menu_entries()
            .filter(|entry| !entry.is_separator())
            .map(|entry| entry.id)
            .collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count);
        assert!((menu_offset(0, false) - MENU_PADDING).abs() < f32::EPSILON);
        // After ten entries and the separator.
        let polygon = MAP_MENU
            .iter()
            .position(|entry| entry.control == "polygonToolStripMenuItem")
            .expect("Polygon");
        assert!(
            (menu_offset(polygon, false) - (MENU_PADDING + 10.0 * MENU_ROW + MENU_SEPARATOR)).abs()
                < f32::EPSILON
        );
    }

    /// A drop-down lines up with its entry when it fits, moves up just enough when it would run
    /// off the bottom, and never above the window; and none of that moves the menu, whose top is
    /// an input here rather than an output.
    #[test]
    fn a_drop_down_moves_up_to_fit_and_the_menu_does_not_move() {
        // Room below: aligned with the entry (its padding removed so the rows line up).
        assert!((dropdown_top(100.0, 50.0, 200.0, 1200.0) - 46.0).abs() < f32::EPSILON);
        // The menu opened at 837 in a 1200-tall window and Map Tool is 12 rows down: a drop-down
        // of 7 rows would end below the window, so it starts higher, ending exactly at the bottom.
        let map_tool = MENU_PADDING + 12.0 * MENU_ROW;
        let height = column_height(&[]) + 7.0 * MENU_ROW;
        let top = dropdown_top(837.0, map_tool, height, 1200.0);
        assert!(top < map_tool - MENU_PADDING);
        assert!((837.0 + top + height - 1200.0).abs() < f32::EPSILON);
        // Taller than the window: pinned to the window's top, not above it.
        assert!((dropdown_top(500.0, 50.0, 5000.0, 1200.0) + 500.0).abs() < f32::EPSILON);
        // The menu's own height is what the window snaps, and a drop-down is not part of it.
        let menu = column_height(MAP_MENU);
        assert!(menu > 10.0 * MENU_ROW);
        assert!(
            menu < 1200.0,
            "the menu itself fits a 1200-tall window: {menu}"
        );
    }

    /// Every entry the plan's list says the tests prove is live, and the rest are dimmed.
    #[test]
    fn the_ported_entries_are_live() {
        let live: Vec<&str> = menu_entries()
            .filter(|entry| entry.action.is_some())
            .map(|entry| entry.id)
            .collect();
        assert_eq!(
            live,
            vec![
                "menu-deleteWP",
                "menu-insertWp",
                "menu-currentPosition",
                "menu-insertSplineWP",
                "menu-loiterForever",
                "menu-loitertime",
                "menu-loitercircles",
                "menu-jumpstart",
                "menu-jumpwP",
                "menu-rTL",
                "menu-land",
                "menu-takeoff",
                "menu-setROI",
                "menu-clearMission",
                "menu-addPolygonPoint2",
                "menu-clearPolygon2",
                "menu-savePolygon2",
                "menu-loadPolygon2",
                "menu-fromSHP2",
                "menu-fromCurrentWaypoints",
                "menu-offsetPolygon2",
                "menu-area2",
                "menu-GeoFenceupload",
                "menu-GeoFencedownload",
                "menu-setReturnLocation",
                "menu-loadFromFile",
                "menu-saveToFile",
                "menu-clear",
                "menu-setRallyPoint",
                "menu-getRallyPoints",
                "menu-saveRallyPoints",
                "menu-clearRallyPoints",
                "menu-saveToFile1",
                "menu-loadFromFile1",
                "menu-createWpCircle",
                "menu-createSplineCircle",
                "menu-area1",
                "menu-text",
                "menu-createCircleSurvey",
                "menu-surveyGrid",
                "menu-ContextMeasure",
                "menu-zoomTo",
                "menu-prefetch",
                "menu-prefetchWPPath",
                "menu-kMLOverlay",
                "menu-elevationGraph",
                "menu-reverseWPs",
                "menu-loadWPFile",
                "menu-loadAndAppend",
                "menu-saveWPFile",
                "menu-loadKMLFile",
                "menu-loadSHPFile",
                "menu-poiadd",
                "menu-poidelete",
                "menu-poiedit",
                "menu-trackerHome",
                "menu-modifyAlt",
                "menu-enterUTMCoord",
                "menu-switchDocking",
                "menu-setHomeHere",
                "menu-zoomToVehicle",
                "menu-zoomToMission",
                "menu-zoomToHome",
                "menu-poly-addPolygonPoint",
                "menu-poly-clearPolygon",
                "menu-poly-savePolygon",
                "menu-poly-loadPolygon",
                "menu-poly-fromSHP",
                "menu-poly-convertWPToPolygon",
                "menu-poly-offsetPolygon",
                "menu-poly-area",
                "menu-fenceInclusion",
                "menu-fenceExclusion",
            ]
        );
    }

    // ---- The map's zoom: the zoom icon's menu, Zoom To, the Zoom box and bar ----

    fn close(a: LatLon, b: LatLon) -> bool {
        (a.latitude() - b.latitude()).abs() < 1e-9 && (a.longitude() - b.longitude()).abs() < 1e-9
    }

    fn zoom_of(map: &MapViewport) -> f64 {
        map.zoom_level().expect("a view")
    }

    /// `zoomToVehicleToolStripMenuItem_Click`: no position is "Invalid Location" and moves
    /// nothing; a position is centred, and a view wider than 17 brought in to it, a closer one
    /// left.
    #[test]
    fn zoom_to_vehicle_centres_on_it_at_17_or_closer() {
        let mut map = MapViewport::new(0, 0);
        assert_eq!(zoom_to_vehicle(&mut map, None), Err("Invalid Location"));
        assert_eq!(
            zoom_to_vehicle(&mut map, Some(at(0.0, 0.0))),
            Err("Invalid Location")
        );
        assert_eq!(map.centre(), None, "refused: nothing moved");
        assert_eq!(zoom_to_vehicle(&mut map, Some(canberra())), Ok(()));
        assert!(close(map.centre().expect("centre"), canberra()));
        assert!((zoom_of(&map) - 17.0).abs() < 1e-9);
        map.set_zoom(19.5);
        let elsewhere = at(-35.0, 149.0);
        assert_eq!(zoom_to_vehicle(&mut map, Some(elsewhere)), Ok(()));
        assert!(close(map.centre().expect("centre"), elsewhere));
        assert!((zoom_of(&map) - 19.5).abs() < 1e-9);
    }

    /// `zoomToHomeToolStripMenuItem_Click`: the vehicle's home, else the planned home when its
    /// latitude is not zero, and 17 either way.
    #[test]
    fn zoom_to_home_prefers_the_vehicles_home_then_the_planned_one() {
        let planned = Home {
            lat: -27.5,
            lng: 153.0,
            alt: 8.0,
        };
        let mut map = MapViewport::new(0, 0);
        zoom_to_home(&mut map, Some(canberra()), planned);
        assert!(close(map.centre().expect("centre"), canberra()));
        assert!((zoom_of(&map) - 17.0).abs() < 1e-9);
        zoom_to_home(&mut map, Some(at(0.0, 0.0)), planned);
        assert!(close(map.centre().expect("centre"), at(-27.5, 153.0)));
        // The C# tests the planned latitude twice and never the longitude.
        let on_the_meridian = Home {
            lat: 51.5,
            lng: 0.0,
            alt: 0.0,
        };
        zoom_to_home(&mut map, None, on_the_meridian);
        assert!(close(map.centre().expect("centre"), at(51.5, 0.0)));
        // Neither: the view stays, and is still brought in to 17.
        map.set_zoom(10.0);
        zoom_to_home(&mut map, None, Home::default());
        assert!(close(map.centre().expect("centre"), at(51.5, 0.0)));
        assert!((zoom_of(&map) - 17.0).abs() < 1e-9);
    }

    /// Map Tool > Zoom To asks for a place, Perth Airport offered.
    #[test]
    fn zoom_to_asks_where() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::ZoomTo, None);
        let prompt = menus.prompt.as_ref().expect("the InputBox");
        assert_eq!(
            (prompt.title, prompt.text.as_str(), prompt.value()),
            (
                "Location",
                "Enter your location",
                "Perth Airport, Australia"
            )
        );
        menus.cancel(&mut plan);
        assert!(!menus.geocoding(), "Cancel asks the geocoder nothing");
        assert!(plan.items().is_empty());
    }

    /// Waits for the geocoder thread's answer.
    fn geocoded(menus: &mut PlanMenus) -> (String, (mapview::GeocoderStatus, Option<LatLon>)) {
        for _ in 0..500 {
            if let Some(answer) = menus.take_geocode() {
                return answer;
            }
            wasm_thread::sleep(std::time::Duration::from_millis(10));
        }
        panic!("the geocoder never answered");
    }

    const CANBERRA_PAGE: &str = "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n<searchresults><place place_rank=\"16\" lat=\"-35.2975906\" lon=\"149.1012676\" display_name=\"Canberra\"/></searchresults>";

    /// The whole of Zoom To, the geocoder faked: OK sends the place, and the answer centres the
    /// view on the first place found at zoom 15.
    #[test]
    fn zoom_to_goes_to_the_place_the_geocoder_finds() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus {
            fake_geocoder: Some(|url| {
                if url == "https://nominatim.openstreetmap.org/search?q=Canberra+ACT&format=xml" {
                    Ok(CANBERRA_PAGE.to_owned())
                } else {
                    Err(format!("asked for {url}"))
                }
            }),
            ..PlanMenus::default()
        };
        choose(&mut plan, &mut menus, MenuAction::ZoomTo, None);
        answer(&mut plan, &mut menus, "Canberra ACT");
        assert!(menus.geocoding());
        let (place, outcome) = geocoded(&mut menus);
        assert_eq!(place, "Canberra ACT");
        assert!(!menus.geocoding());
        let mut map = MapViewport::new(0, 0);
        assert_eq!(zoom_to_answer(&mut map, &place, outcome), None);
        assert!(close(
            map.centre().expect("centre"),
            at(-35.297_590_6, 149.101_267_6)
        ));
        assert!((zoom_of(&map) - ZOOM_TO_PLACE).abs() < 1e-9);
    }

    /// A geocoder that cannot be reached is `ExceptionInCode`, said in the C#'s words; a page of
    /// no places is a success that leaves the view where it was, at 15.
    #[test]
    fn zoom_to_says_what_the_geocoder_could_not_find() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus {
            fake_geocoder: Some(|_| Err("offline".to_owned())),
            ..PlanMenus::default()
        };
        choose(&mut plan, &mut menus, MenuAction::ZoomTo, None);
        answer(&mut plan, &mut menus, "Atlantis");
        let (place, outcome) = geocoded(&mut menus);
        let mut map = MapViewport::new(0, 0);
        map.centre_on(canberra());
        let before = zoom_of(&map);
        assert_eq!(
            zoom_to_answer(&mut map, &place, outcome),
            Some("Google Maps Geocoder can't find: 'Atlantis', reason: ExceptionInCode".to_owned())
        );
        assert!(
            (zoom_of(&map) - before).abs() < 1e-9,
            "a failure moves nothing"
        );
        let none = (mapview::GeocoderStatus::Success, None);
        assert_eq!(zoom_to_answer(&mut map, "Nowhere", none), None);
        assert!(close(map.centre().expect("centre"), canberra()));
        assert!((zoom_of(&map) - 15.0).abs() < 1e-9);
    }

    /// `Zoomlevel`: one decimal place, half a zoom an arrow, 0 to 24.
    #[test]
    fn the_zoom_box_steps_by_a_half_within_0_to_24() {
        assert_eq!(zoomlevel_text(16.0), "16.0");
        assert_eq!(zoomlevel_text(16.25), "16.3");
        assert!((zoomlevel_step(16.0, true) - 16.5).abs() < f64::EPSILON);
        assert!((zoomlevel_step(16.0, false) - 15.5).abs() < f64::EPSILON);
        assert!((zoomlevel_step(23.8, true) - 24.0).abs() < f64::EPSILON);
        assert!((zoomlevel_step(0.2, false) - 0.0).abs() < f64::EPSILON);
    }

    /// `TRK_zoom`: 24 at the top of its travel, 0 at the bottom, whole thousandths between.
    #[test]
    fn the_zoom_bar_runs_from_24_at_the_top_to_0_at_the_bottom() {
        let (top, height) = (100.0, 416.0);
        assert!((track_value(top, top, height) - 24.0).abs() < f32::EPSILON);
        assert!((track_value(top + height, top, height) - 0.0).abs() < f32::EPSILON);
        assert!((track_value(top + height / 2.0, top, height) - 12.0).abs() < f32::EPSILON);
        let value = track_value(top + 101.0, top, height);
        assert!(
            ((value * 1000.0).round() - value * 1000.0).abs() < 1e-3,
            "{value}"
        );
        assert!(value < 24.0 && value > 12.0);
    }

    /// The radii the planning map is handed are the boxes', with `writeKML`'s defaults for an
    /// empty box, and none when a box does not parse.
    #[test]
    fn the_planning_map_takes_its_radii_from_the_boxes() {
        let mut plan = Plan::default();
        let overlay = |wp: f64, loiter: f64| mapview::Overlay {
            wp_radius: wp,
            loiter_radius: loiter,
            planner: true,
        };
        assert_eq!(map_overlay(&plan), Some(overlay(30.0, 45.0)));
        plan.set_panel_text(PanelBox::WpRadius, "12.5");
        plan.set_panel_text(PanelBox::LoiterRadius, "-80");
        assert_eq!(map_overlay(&plan), Some(overlay(12.5, -80.0)));
        plan.set_panel_text(PanelBox::WpRadius, "");
        plan.set_panel_text(PanelBox::LoiterRadius, "");
        assert_eq!(map_overlay(&plan), Some(overlay(5.0, 30.0)));
        plan.set_panel_text(PanelBox::WpRadius, "1.2.3");
        assert_eq!(map_overlay(&plan), None);
    }

    /// The flight screen's Guided Mode marker: in Guided, once somewhere has been sent, at
    /// `GuidedMode`'s position and whole height, circled at the WP Radius box.
    #[test]
    fn the_guided_mode_marker_is_where_guided_mode_went() {
        let mut plan = Plan::default();
        let sent = crate::fly::GuidedMode {
            x: -353_625_000,
            y: 1_491_657_000,
            z: 20.7,
            frame: 3,
        };
        assert_eq!(guided_marker(None, sent, &plan), None);
        assert_eq!(guided_marker(Some("Loiter"), sent, &plan), None);
        let nowhere = crate::fly::GuidedMode { x: 0, ..sent };
        assert_eq!(guided_marker(Some("Guided"), nowhere, &plan), None);
        let marker = guided_marker(Some("GUIDED"), sent, &plan).expect("a marker");
        assert!(close(marker.position, at(-35.3625, 149.1657)));
        assert_eq!(marker.alt, 20);
        assert!((marker.wp_radius - 30.0).abs() < f64::EPSILON);
        plan.set_panel_text(PanelBox::WpRadius, "12.5");
        let marker = guided_marker(Some("Guided"), sent, &plan).expect("a marker");
        assert!((marker.wp_radius - 12.5).abs() < f64::EPSILON);
        // `GetFloat` of what is not a number is 0: no circle.
        plan.set_panel_text(PanelBox::WpRadius, "");
        let marker = guided_marker(Some("Guided"), sent, &plan).expect("a marker");
        assert!(marker.wp_radius.abs() < f64::EPSILON);
    }

    /// `MainMap_OnMarkerEnter` leaves the grid on the last rect entered that is a row.
    #[test]
    fn the_row_entered_last_is_selected() {
        use mapview::MarkerTag::{Guided, Home as H, Item};
        assert_eq!(entered_row(&[]), None);
        assert_eq!(entered_row(&[H]), None);
        assert_eq!(entered_row(&[Item(4), Guided]), Some(4));
        assert_eq!(entered_row(&[Item(2), Item(3)]), Some(3));
        assert_eq!(entered_row(&[Item(2), H]), Some(2));
    }

    /// The zoom icon's menu opens where it was clicked, closes the map's, and a press elsewhere
    /// closes it without the map taking the press.
    #[test]
    fn the_zoom_menu_opens_and_closes() {
        let mut menus = PlanMenus::default();
        menus.open_at(CLICK, canberra(), None);
        menus.open_zoom_menu((20.0, 150.0));
        assert_eq!(menus.zoom_menu, Some((20.0, 150.0)));
        assert!(menus.open.is_none());
        menus.dismiss_zoom_menu((500.0, 500.0));
        assert_eq!(menus.zoom_menu, None);
        assert!(menus.swallows_press((500.0, 500.0)));
        menus.open_zoom_menu((20.0, 150.0));
        menus.open_at(CLICK, canberra(), None);
        assert_eq!(
            menus.zoom_menu, None,
            "the map's menu closes the zoom icon's"
        );
        let facts = zoom_facts(&menus, Some(16.04));
        assert!(facts.contains(&("plan.zoommenu", "closed".to_owned())));
        assert!(facts.contains(&("plan.zoomlevel", "16.0".to_owned())));
        assert!(facts.contains(&("plan.geocoding", "false".to_owned())));
    }
}

/// Rally Points, the polygon's files and tools, and Auto WP's circles: the map menu's entries
/// from `FlightPlanner.cs`, held to the C#'s messages and, where the geometry is the C#'s, to its
/// numbers under mono (`testdata/planner/golden`).
#[cfg(test)]
mod menu_batch_tests {
    use super::*;
    use mp_link::requests::{RallyPointSet, RequestOutcome};

    const CLICK: (f32, f32) = (400.0, 300.0);

    /// SITL's home at CMAC, which the goldens draw about.
    fn cmac() -> LatLon {
        LatLon::new(-35.363_262_1, 149.165_237_4).expect("a position")
    }

    fn at(lat: f64, lng: f64) -> LatLon {
        LatLon::new(lat, lng).expect("a position")
    }

    fn context() -> MenuContext {
        MenuContext {
            frame: AltitudeFrame::Relative,
            vehicle: None,
            takeoff_pitch: false,
            copter: false,
            tracker_alt: 0.0,
        }
    }

    fn choose(plan: &mut Plan, menus: &mut PlanMenus, action: MenuAction) {
        menus.open_at(CLICK, cmac(), None);
        menus.choose(plan, action, &context());
    }

    fn answer(plan: &mut Plan, menus: &mut PlanMenus, value: &str) -> Option<FileRequest> {
        menus
            .prompt
            .as_mut()
            .and_then(|prompt| prompt.field.as_mut())
            .expect("a prompt with a field")
            .set(value);
        menus.submit(plan, &context())
    }

    /// The dialog showing: its caption, its text and what its field offers.
    fn showing(menus: &PlanMenus) -> (&'static str, String, String) {
        let prompt = menus.prompt.as_ref().expect("a dialog");
        (prompt.title, prompt.text.clone(), prompt.value().to_owned())
    }

    /// The golden's blocks: each `case,<name>...` line and the lines under it.
    fn golden_cases(golden: &str) -> Vec<(String, Vec<Vec<String>>)> {
        let mut out: Vec<(String, Vec<Vec<String>>)> = Vec::new();
        for line in golden.lines() {
            let fields: Vec<String> = line.split(',').map(ToOwned::to_owned).collect();
            if fields[0] == "case" {
                out.push((fields[1].clone(), Vec::new()));
            } else if let Some((_, rows)) = out.last_mut() {
                rows.push(fields);
            }
        }
        out
    }

    fn golden_case(golden: &str, name: &str) -> Vec<Vec<String>> {
        golden_cases(golden)
            .into_iter()
            .find(|(case, _)| case == name)
            .unwrap_or_else(|| panic!("no case {name}"))
            .1
    }

    fn number(field: &str) -> f64 {
        field.parse().expect("a number")
    }

    // ---- Rally Points ----

    /// Set Rally Point asks "Altitude", offering Default Alt, and puts a marker at the menu's
    /// position at the whole number given; one `int.TryParse` refuses is "Invalid Alt".
    #[test]
    fn set_rally_point_asks_an_altitude_and_places_a_marker() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::SetRallyPoint);
        assert_eq!(
            showing(&menus),
            ("Altitude", "Altitude".to_owned(), "100".to_owned())
        );
        answer(&mut plan, &mut menus, "60");
        assert_eq!(plan.rally_markers(), vec![(cmac(), 60)]);

        choose(&mut plan, &mut menus, MenuAction::SetRallyPoint);
        answer(&mut plan, &mut menus, "12.5");
        assert_eq!(
            showing(&menus),
            (ERROR, INVALID_ALT.to_owned(), String::new())
        );
        assert_eq!(plan.rally().len(), 1, "a refused altitude places nothing");

        menus.submit(&mut plan, &context());
        choose(&mut plan, &mut menus, MenuAction::SetRallyPoint);
        menus.cancel(&mut plan);
        assert_eq!(plan.rally().len(), 1, "Cancel places nothing");
    }

    /// Save Rally to File says "Please set some rally points" with none, and otherwise asks for a
    /// file filtered to `.ral`; Load Rally from File asks for one.
    #[test]
    fn the_rally_file_entries_ask_for_their_files() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::SaveRallyToFile);
        assert_eq!(
            showing(&menus),
            ("", SET_SOME_RALLY_POINTS.to_owned(), String::new())
        );
        menus.submit(&mut plan, &context());

        plan.add_rally_marker(cmac(), 60);
        choose(&mut plan, &mut menus, MenuAction::SaveRallyToFile);
        assert_eq!(showing(&menus).0, SAVE_FILE);
        assert_eq!(showing(&menus).1, RALLY_FILTER);
        assert_eq!(
            answer(&mut plan, &mut menus, "home"),
            Some(FileRequest::SaveRally("home".to_owned()))
        );

        choose(&mut plan, &mut menus, MenuAction::LoadRallyFromFile);
        assert_eq!(showing(&menus).0, OPEN_FILE);
        assert_eq!(
            answer(&mut plan, &mut menus, "home"),
            Some(FileRequest::LoadRally("home".to_owned()))
        );
        assert_eq!(
            dialog_path("home", "ral", Path::new("/missions")),
            Some(Path::new("/missions").join("home.ral"))
        );
    }

    /// Load File and Save File each open their dialog (the owner, 2026-10-04: they acted on the
    /// name field at once): Open on nothing, filtered to `All Supported Types`, and Save As on the
    /// mission's file name, filtered to `Mission`; the answer is the file to read or write.
    /// `// C#: GCSViews/FlightPlanner.cs:1817-1823, 6069-6077`
    #[test]
    fn load_and_save_file_ask_for_the_file() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        menus.ask_mission_load();
        assert_eq!(
            showing(&menus),
            (OPEN_FILE, MISSION_LOAD_FILTER.to_owned(), String::new())
        );
        assert!(menus.prompt.as_ref().is_some_and(Prompt::is_file_dialog));
        assert_eq!(
            answer(&mut plan, &mut menus, "/elsewhere/flight.waypoints"),
            Some(FileRequest::LoadMission(
                "/elsewhere/flight.waypoints".to_owned()
            ))
        );
        menus.ask_mission_save("survey.waypoints");
        assert_eq!(
            showing(&menus),
            (
                SAVE_FILE,
                MISSION_SAVE_FILTER.to_owned(),
                "survey.waypoints".to_owned()
            )
        );
        assert_eq!(
            menus.prompt.as_ref().map(Prompt::file_types),
            Some(&["waypoints", "txt", "mission"][..])
        );
        assert_eq!(
            answer(&mut plan, &mut menus, "survey.waypoints"),
            Some(FileRequest::SaveMission("survey.waypoints".to_owned()))
        );
    }

    /// What Save Rally to File writes from the markers is the C#'s file; what Load Rally from
    /// File reads from it replaces the markers - with the C#'s float and truncation - and a file
    /// with no row it reads leaves them.
    #[test]
    fn a_rally_file_saves_and_loads_the_markers() {
        let mut plan = Plan::default();
        plan.add_rally_marker(at(-35.363_262_1, 149.165_237_4), 100);
        plan.add_rally_marker(at(-35.363_262_345_678_91, 149.165_237_412_345_67), 60);
        plan.add_rally_marker(at(-0.000_01, 0.000_015), 0);
        let written = mp_mission::fence_file::write_rally(&plan.rally_markers());
        let golden = include_str!("../../../testdata/planner/golden/rally.ral");
        assert_eq!(
            written.split_once('\n').map(|s| s.1),
            golden.split_once('\n').map(|s| s.1)
        );

        let mut other = Plan::default();
        other.add_rally_marker(at(-30.0, 150.0), 5);
        other.load_rally_file(&mp_mission::fence_file::read_rally("#nothing\n"));
        assert_eq!(other.rally().len(), 1, "no row read, nothing cleared");
        other.load_rally_file(&mp_mission::fence_file::read_rally(&written));
        let markers = other.rally_markers();
        assert_eq!(markers.len(), 3);
        // `(int)(float.Parse("-35.3632623456789") * 1e7) / 1e7`, as the C# read it back.
        assert_eq!(markers[1].0.latitude(), -35.363_262_1);
        assert_eq!(markers[1].1, 60);
        assert_eq!(markers[2].0.latitude(), -9.9e-6);
    }

    fn upload_of(count: usize) -> RallyUpload {
        RallyUpload::new(
            (0..count)
                .map(|index| {
                    let index = i32::try_from(index).expect("a few");
                    (at(-35.36 - f64::from(index) * 0.001, 149.16), 50 + index)
                })
                .collect(),
        )
    }

    /// The waits the scripted vehicle is driven with: the C#'s counts, a twentieth of its waits.
    fn fast() -> mp_link::ProtocolTimeouts {
        mp_link::ProtocolTimeouts::default().faster(20)
    }

    /// Upload sets `RALLY_TOTAL` to the markers' count, then sends each marker as `setRallyPoint`
    /// builds it - index from 0, the vehicle's `RALLY_TOTAL` as the count, degrees times 1e7
    /// truncated - one at a time; a point sent and never read back the same is passed over.
    #[test]
    fn upload_sets_the_total_then_each_point_in_turn() {
        let mut upload = upload_of(2);
        assert_eq!(upload.due(None), Some(RallyDue::Total(2.0)));
        upload.answer(Some(RequestOutcome::Accepted { value: None }));
        let Some(RallyDue::Point(first)) = upload.due(Some(2.0)) else {
            panic!("the first point");
        };
        assert_eq!(
            first,
            RallyPointSet {
                idx: 0,
                count: 2,
                lat: -353_600_000,
                lng: 1_491_600_000,
                alt: 50,
                break_alt: 0,
                land_dir: 0,
                flags: 0,
            }
        );
        upload.answer(Some(RequestOutcome::Accepted { value: None }));
        let Some(RallyDue::Point(second)) = upload.due(Some(2.0)) else {
            panic!("the second point");
        };
        assert_eq!((second.idx, second.alt, second.lat), (1, 51, -353_610_000));
        upload.answer(Some(RequestOutcome::Sent));
        assert_eq!(upload.due(Some(2.0)), None);
        assert_eq!(upload.end(), Some(&WritesEnd::Done));
        assert_eq!(upload.results(), "RALLY_TOTAL=2,0=set,1=sent");
    }

    /// A vehicle that has not listed `RALLY_TOTAL` takes no set, and the first point's count
    /// throws: "Failed to save rally point", nothing sent. A point never read back is the same
    /// message; a `RALLY_TOTAL` never echoed is the exception's own.
    #[test]
    fn upload_stops_where_the_c_sharp_says_it_failed() {
        let failed = WritesEnd::Stopped {
            title: ERROR,
            text: RALLY_SAVE_FAILED.to_owned(),
        };
        let mut upload = upload_of(1);
        let _ = upload.due(None);
        upload.answer(Some(RequestOutcome::UnknownParameter));
        assert_eq!(upload.due(None), None);
        assert_eq!(upload.end(), Some(&failed));

        let mut upload = upload_of(2);
        let _ = upload.due(Some(1.0));
        upload.answer(Some(RequestOutcome::Unchanged));
        let _ = upload.due(Some(1.0));
        upload.answer(Some(RequestOutcome::TimedOut));
        assert_eq!(upload.end(), Some(&failed));
        assert_eq!(upload.results(), "RALLY_TOTAL=2 unchanged,0=timeout");

        let mut upload = upload_of(1);
        let _ = upload.due(Some(1.0));
        upload.answer(Some(RequestOutcome::TimedOut));
        assert_eq!(
            upload.end(),
            Some(&WritesEnd::Stopped {
                title: ERROR,
                text: "Timeout on read - setParam RALLY_TOTAL".to_owned()
            })
        );

        // No markers: the total is set to 0 and nothing else is said.
        let mut upload = upload_of(0);
        assert_eq!(upload.due(None), Some(RallyDue::Total(0.0)));
        upload.answer(None);
        assert_eq!(upload.due(None), None);
        assert_eq!(upload.end(), Some(&WritesEnd::Done));
    }

    /// Upload through the real link, to a scripted ArduPilot that answers the legacy protocol:
    /// `PARAM_SET RALLY_TOTAL`, then each `RALLY_POINT` and the `RALLY_FETCH_POINT` that reads it
    /// back, in that order - and a vehicle that never reads a point back gives "Failed to save
    /// rally point" after the fetch has gone four times.
    #[test]
    fn upload_goes_through_the_link_as_the_c_sharp_sends_it() {
        use crate::telemetry::scripted::{INT32, VEHICLE, Vehicle, param, until};
        use mp_mavlink_dialects::all::{MavMessage, RallyPoint};

        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("RALLY_TOTAL", 0.0, INT32));
        until("RALLY_TOTAL listed", || {
            telemetry.holds_parameter("RALLY_TOTAL")
        });

        let mut upload = upload_of(2);
        let mut rally_total = None;
        until("the upload to end", || {
            for message in vehicle.read() {
                match message {
                    MavMessage::ParamSet(set) => {
                        rally_total = Some(f64::from(set.param_value));
                        vehicle.send(&param("RALLY_TOTAL", set.param_value, INT32));
                    }
                    MavMessage::RallyFetchPoint(fetch) => {
                        let sent = vehicle.heard.iter().rev().find_map(|heard| match heard {
                            MavMessage::RallyPoint(point) if point.idx == fetch.idx => Some(*point),
                            _ => None,
                        });
                        if let Some(point) = sent {
                            vehicle.send(&MavMessage::RallyPoint(RallyPoint {
                                target_system: 255,
                                target_component: 190,
                                ..point
                            }));
                        }
                    }
                    _ => {}
                }
            }
            step_rally_upload(&telemetry, &mut upload, rally_total);
            upload.end().is_some()
        });
        assert_eq!(upload.end(), Some(&WritesEnd::Done));
        assert_eq!(upload.results(), "RALLY_TOTAL=2,0=set,1=set");
        let order: Vec<String> = vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::ParamSet(_) => Some("PARAM_SET".to_owned()),
                MavMessage::RallyPoint(point) => Some(format!("RALLY_POINT {}", point.idx)),
                MavMessage::RallyFetchPoint(fetch) => Some(format!("FETCH {}", fetch.idx)),
                _ => None,
            })
            .collect();
        assert_eq!(
            order,
            [
                "PARAM_SET",
                "RALLY_POINT 0",
                "FETCH 0",
                "RALLY_POINT 1",
                "FETCH 1"
            ]
        );
        let first = vehicle
            .heard
            .iter()
            .find_map(|message| match message {
                MavMessage::RallyPoint(point) => Some(*point),
                _ => None,
            })
            .expect("a point");
        assert_eq!(
            (
                first.count,
                first.lat,
                first.lng,
                first.alt,
                first.target_system
            ),
            (2, -353_600_000, 1_491_600_000, 50, VEHICLE.sysid)
        );

        // A vehicle that never reads a point back.
        let mut upload = upload_of(1);
        until("the upload to fail", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    vehicle.send(&param("RALLY_TOTAL", set.param_value, INT32));
                }
            }
            step_rally_upload(&telemetry, &mut upload, Some(1.0));
            upload.end().is_some()
        });
        assert_eq!(
            upload.end(),
            Some(&WritesEnd::Stopped {
                title: ERROR,
                text: RALLY_SAVE_FAILED.to_owned()
            })
        );
        let _ = vehicle.read();
        let fetches = vehicle.count(
            |message| matches!(message, MavMessage::RallyFetchPoint(fetch) if fetch.idx == 0),
        );
        // One from the first upload's point 0, then this upload's four - all RALLY_FETCH_POINT,
        // where the C#'s retries would be FENCE_FETCH_POINT.
        assert_eq!(fetches, 1 + 4);
    }

    /// Download's list: nothing while the transfer is queued or running, then the vehicle's
    /// rally items - here none - through `mav_mission.download`'s messages.
    #[test]
    fn download_reads_the_vehicles_rally_list() {
        use crate::telemetry::scripted::{Vehicle, until};
        use mp_mavlink_dialects::all::{MavMessage, MissionCount};

        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        assert!(telemetry.download_rally());
        assert_eq!(telemetry.rally_list(), None, "queued, not answered");
        until("the rally list", || {
            for message in vehicle.read() {
                if let MavMessage::MissionRequestList(request) = message {
                    assert_eq!(request.mission_type, 2, "the rally list");
                    vehicle.send(&MavMessage::MissionCount(MissionCount {
                        count: 0,
                        target_system: 255,
                        target_component: 190,
                        mission_type: 2,
                    }));
                }
            }
            telemetry.rally_list().is_some()
        });
        assert_eq!(telemetry.rally_list(), Some(Ok(Vec::new())));
    }

    /// Clear Rally Points' `RALLY_TOTAL = 0` carries on whatever it gets, and then the markers
    /// and `MAV.rallypoints` go.
    #[test]
    fn clear_rally_points_clears_after_its_set_whatever_it_gets() {
        let mut writes = ParamWrites::new(
            vec![ParamStep::one("RALLY_TOTAL", 0.0, OnTimeout::CarryOn)],
            AfterWrites::ClearRally,
        );
        assert_eq!(writes.due(), Some(("RALLY_TOTAL", 0.0)));
        writes.answer(Some(RequestOutcome::TimedOut));
        assert_eq!(writes.end(), Some(&WritesEnd::Done));
        assert_eq!(writes.after(), AfterWrites::ClearRally);

        let mut plan = Plan::default();
        plan.add_rally_marker(cmac(), 60);
        plan.vehicle_rally = vec![
            RallyPoint {
                position: cmac(),
                altitude: 60.0,
                break_altitude: None,
            }
            .to_item(0),
        ];
        plan.clear_rally_points();
        assert!(plan.rally().is_empty());
        assert!(plan.vehicle_rally().is_empty());
    }

    // ---- Polygon ----

    fn square() -> Vec<LatLon> {
        vec![
            at(-35.3627, 149.1646),
            at(-35.3627, 149.1658),
            at(-35.3637, 149.1658),
            at(-35.3637, 149.1646),
        ]
    }

    fn corners(rows: &[Vec<String>]) -> Vec<LatLon> {
        rows.iter()
            .filter(|row| row[0] == "corner")
            .map(|row| at(number(&row[1]), number(&row[2])))
            .collect()
    }

    /// Save Polygon with nothing drawn does nothing; with a polygon it asks for a `.poly`. Load
    /// Polygon asks for one, and its corners replace the polygon's.
    #[test]
    fn the_polygon_file_entries_ask_for_their_files() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::SavePolygon);
        assert!(menus.prompt.is_none(), "nothing drawn, nothing asked");
        plan.load_polygon(square());
        choose(&mut plan, &mut menus, MenuAction::SavePolygon);
        assert_eq!(
            (showing(&menus).0, showing(&menus).1),
            (SAVE_FILE, POLYGON_FILTER.to_owned())
        );
        assert_eq!(
            answer(&mut plan, &mut menus, "field"),
            Some(FileRequest::SavePolygon("field".to_owned()))
        );
        choose(&mut plan, &mut menus, MenuAction::LoadPolygon);
        assert_eq!(showing(&menus).0, OPEN_FILE);
        assert_eq!(
            answer(&mut plan, &mut menus, "field"),
            Some(FileRequest::LoadPolygon("field".to_owned()))
        );
        let text = mp_mission::fence_file::write_polygon(plan.polygon());
        let mut other = Plan::default();
        other.load_polygon(mp_mission::fence_file::read_polygon(&text));
        assert_eq!(other.polygon(), square().as_slice());
    }

    /// Offset Polygon asks for metres, offering 0, and the polygon becomes the C#'s offset of it
    /// under mono; Cancel offsets by 0 all the same, as the C# goes on with its 0; an answer
    /// `double.Parse` refuses is the exception's message and the polygon stays.
    #[test]
    fn offset_polygon_is_the_c_sharps() {
        let golden = include_str!("../../../testdata/planner/golden/offset.csv");
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::OffsetPolygon);
        assert!(menus.prompt.is_none(), "no polygon, no question");

        plan.load_polygon(square());
        choose(&mut plan, &mut menus, MenuAction::OffsetPolygon);
        assert_eq!(
            showing(&menus),
            (
                "Offset in Meters",
                "Please enter the offset in meters. Enter a negative value to make the polygon smaller"
                    .to_owned(),
                "0".to_owned()
            )
        );
        answer(&mut plan, &mut menus, "10");
        assert_eq!(
            plan.polygon(),
            corners(&golden_case(golden, "square_out")).as_slice()
        );

        plan.load_polygon(square());
        choose(&mut plan, &mut menus, MenuAction::OffsetPolygon);
        menus.cancel(&mut plan);
        assert_eq!(
            plan.polygon(),
            corners(&golden_case(golden, "square_zero")).as_slice()
        );

        plan.load_polygon(square());
        choose(&mut plan, &mut menus, MenuAction::OffsetPolygon);
        answer(&mut plan, &mut menus, "ten");
        assert_eq!(
            showing(&menus),
            (ERROR, FORMAT_EXCEPTION.to_owned(), String::new())
        );
        assert_eq!(plan.polygon(), square().as_slice());
    }

    /// Area says the polygon's area in the C#'s words and figures under mono; with no polygon it
    /// says "Please define a polygon!" and then the area of nothing, however that is closed. Auto
    /// WP > Area is the same entry.
    #[test]
    fn area_is_said_as_the_c_sharp_says_it() {
        let golden = include_str!("../../../testdata/planner/golden/area.csv");
        let text = |name: &str| {
            golden_case(golden, name)
                .iter()
                .filter(|row| row[0] == "text")
                .map(|row| row[1..].join(","))
                .collect::<Vec<_>>()
                .join("\n")
        };
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        plan.load_polygon(square());
        choose(&mut plan, &mut menus, MenuAction::Area);
        assert_eq!(showing(&menus), ("Area", text("square"), String::new()));
        assert_eq!(plan.polygon(), square().as_slice(), "measured, not changed");
        menus.submit(&mut plan, &context());

        for (name, vertices) in [
            (
                "ell",
                vec![
                    at(-35.362, 149.164),
                    at(-35.362, 149.165),
                    at(-35.363, 149.165),
                    at(-35.363, 149.167),
                    at(-35.364, 149.167),
                    at(-35.364, 149.164),
                ],
            ),
            (
                "small",
                vec![
                    at(-35.36326, 149.16523),
                    at(-35.36326, 149.16526),
                    at(-35.36329, 149.16526),
                ],
            ),
        ] {
            plan.load_polygon(vertices);
            let aream2 = plan.polygon_area().expect("a polygon");
            assert_eq!(area_text(aream2), text(name), "{name}");
        }

        plan.load_polygon(Vec::new());
        choose(&mut plan, &mut menus, MenuAction::Area);
        assert_eq!(
            showing(&menus),
            ("", DEFINE_POLYGON.to_owned(), String::new())
        );
        menus.submit(&mut plan, &context());
        assert_eq!(showing(&menus), ("Area", text("empty"), String::new()));
        menus.submit(&mut plan, &context());
        choose(&mut plan, &mut menus, MenuAction::Area);
        menus.cancel(&mut plan);
        assert_eq!(
            showing(&menus).0,
            "Area",
            "Escape closes it and the area follows"
        );

        let area1 = MAP_MENU
            .iter()
            .flat_map(|entry| entry.children.iter())
            .find(|entry| entry.id == "menu-area1")
            .expect("Auto WP > Area");
        assert_eq!(area1.action, Some(MenuAction::Area));
    }

    /// From SHP asks for a shape file; cancelling it still clears the polygon, as the C# clears
    /// it before looking at what its dialog returned. The committed field - one ring in UTM 55S
    /// with its `.prj` - lands where DotSpatial puts it, its closing corner dropped.
    #[test]
    fn from_shp_reads_the_field_where_dotspatial_puts_it() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        plan.load_polygon(square());
        choose(&mut plan, &mut menus, MenuAction::FromShp);
        assert_eq!(
            (showing(&menus).0, showing(&menus).1),
            (OPEN_FILE, SHP_FILTER.to_owned())
        );
        assert_eq!(
            menus.cancel(&mut plan),
            Some(FileRequest::LoadShp(String::new()))
        );

        let shp = include_bytes!("../../../testdata/planner/field.shp");
        let prj = include_str!("../../../testdata/planner/field.prj");
        let projection =
            mp_mission::shapefile::Projection::from_esri(prj.lines().next().unwrap_or(""))
                .expect("UTM 55S");
        let features = mp_mission::shapefile::features(shp).expect("a shape file");
        plan.load_polygon(Vec::new());
        plan.add_shp_features(&features, Some(projection));
        assert_eq!(
            plan.polygon().len(),
            4,
            "five points, the closing one dropped"
        );
        // DotSpatial under mono: (695400, 6084100) in UTM 55S (golden/reproject.csv).
        let first = plan.polygon()[0];
        assert!((first.latitude() - -35.367_300_938_561_82).abs() < 1e-11);
        assert!((first.longitude() - 149.150_818_946_920_45).abs() < 1e-11);
    }

    /// Two features accumulate, and "remove loop close" compares each feature's last corner with
    /// the polygon's first - so only the first ring's close is dropped.
    #[test]
    fn from_shp_accumulates_features_as_the_c_sharp_does() {
        let mut plan = Plan::default();
        let ring = |x: f64| vec![(x, -35.0), (x + 0.1, -35.0), (x + 0.1, -35.1), (x, -35.0)];
        plan.add_shp_features(&[ring(149.0), ring(150.0)], None);
        assert_eq!(plan.polygon().len(), 3 + 4);
        assert_eq!(plan.polygon()[0], at(-35.0, 149.0));
        assert_eq!(plan.polygon()[6], at(-35.0, 150.0));
    }

    // ---- File Load/Save's loads and Map Tool's KML Overlay ----

    fn two_rows() -> Plan {
        let mut plan = Plan::default();
        plan.add_waypoint(at(-35.0, 149.0), 100.0);
        plan.add_waypoint(at(-35.1, 149.1), 100.0);
        plan
    }

    /// Load and Append keeps the rows, passes over the file's home, and adds the rest.
    #[test]
    fn load_and_append_keeps_the_rows_and_passes_over_the_files_home() {
        let mut plan = two_rows();
        let items =
            mp_mission::read_waypoints(include_str!("../../../testdata/planner/append.waypoints"))
                .expect("a mission file");
        assert_eq!(plan.append_from_file(&items), 2);
        assert_eq!(plan.items().len(), 4);
        assert_eq!(plan.items()[2].z, 80.0);
        assert_eq!(plan.items()[3].z, 90.0);
        assert_eq!(plan.items()[3].seq, 4, "renumbered after the rows kept");
        // An item of command 0 past the first row ends the list: the two after it are not added.
        let mut plan = two_rows();
        let mut items = items;
        items[2].command = 0;
        assert_eq!(plan.append_from_file(&items), 1);
        assert_eq!(plan.items().len(), 3);
        // With no rows the file's home is still passed over: `i` goes back to -1.
        let mut plan = Plan::default();
        let items =
            mp_mission::read_waypoints(include_str!("../../../testdata/planner/append.waypoints"))
                .expect("a mission file");
        assert_eq!(plan.append_from_file(&items), 2);
        assert_eq!(plan.items()[0].z, 80.0);
    }

    /// Load KML File: a row per line-string coordinate with `(int)` of its altitude, and a POI
    /// per point placemark with its name; the polygon adds nothing.
    #[test]
    fn load_kml_file_adds_rows_for_line_strings_and_pois_for_points() {
        let mut plan = Plan::default();
        let mut pois = crate::poi::Pois::kept_in(None);
        let text =
            mp_kml::read::without_snippets(include_str!("../../../testdata/planner/route.kml"));
        let rows = kml_into_mission(&mut plan, &mut pois, &text, &context()).expect("a KML file");
        assert_eq!(rows, 3);
        assert_eq!(plan.items().len(), 3);
        assert_eq!(plan.items()[0].x, -35.36);
        assert_eq!(plan.items()[0].y, 149.16);
        assert_eq!(plan.items()[0].z, 50.0);
        assert_eq!(plan.items()[1].z, 60.0, "(int) 60.9");
        assert_eq!(plan.items()[2].z, 70.0);
        assert!(plan.items().iter().all(|item| item.command == 16));
        assert_eq!(pois.points().len(), 1);
        assert_eq!(pois.points()[0].id(), "Gate");
        assert_eq!(pois.points()[0].lat, -35.362);
    }

    /// A coordinate without an altitude is `(int) loc.Altitude` on nothing: "Bad KML File :" with
    /// the exception, the rows before it kept.
    #[test]
    fn a_kml_path_without_altitudes_fails_after_the_rows_before_it() {
        let mut plan = Plan::default();
        let mut pois = crate::poi::Pois::kept_in(None);
        let kml = "<kml><Placemark><LineString><coordinates>149.1,-35.3,40 149.2,-35.3</coordinates></LineString></Placemark></kml>";
        let why = kml_into_mission(&mut plan, &mut pois, kml, &context()).expect_err("no altitude");
        assert!(why.contains("Nullable object must have a value"), "{why}");
        assert_eq!(plan.items().len(), 1);
        assert_eq!(plan.items()[0].z, 40.0);
        // Not XML at all.
        let why = kml_into_mission(&mut plan, &mut pois, "<kml>", &context()).expect_err("not XML");
        assert!(why.starts_with("System.Xml.XmlException"), "{why}");
    }

    /// A .kmz gives its first top-level .kml; one without any is nothing, without a word.
    #[test]
    fn a_kmz_gives_its_first_kml_at_the_top() {
        let text = kml_text(
            "route.kmz",
            include_bytes!("../../../testdata/planner/route.kmz"),
        )
        .expect("a zip")
        .expect("a kml inside");
        assert!(text.contains("<name>route</name>"));
        assert!(!text.contains("<Snippet/>"), "snippets dropped");
        let entries = [mp_log::zip::Entry {
            name: "inner/only.kml".to_owned(),
            data: b"<kml/>".to_vec(),
        }];
        let stamp = mp_log::zip::DosTime {
            year: 2026,
            month: 9,
            day: 24,
            hour: 0,
            minute: 0,
            second: 0,
        };
        let zip = mp_log::zip::write(&entries, stamp).expect("a zip");
        assert_eq!(
            kml_text("x.KMZ", &zip),
            Ok(None),
            "a .kml in a folder is not at the top"
        );
        assert!(kml_text("x.kmz", b"not a zip").is_err());
        assert_eq!(
            kml_text("x.kml", b"<kml><Snippet/></kml>"),
            Ok(Some("<kml></kml>".to_owned()))
        );
    }

    /// Load SHP File on a point file: a row per record at the record's vertex, the altitude from
    /// the ELEVATION column (or -1, which `setfromMap` leaves as 0), sorted by the wp column.
    #[test]
    fn load_shp_file_reads_points_with_their_elevation_sorted_by_wp() {
        let mut plan = Plan::default();
        let rows = shp_into_mission(
            &mut plan,
            include_bytes!("../../../testdata/planner/points.shp"),
            include_bytes!("../../../testdata/planner/points.dbf"),
            Some(include_str!("../../../testdata/planner/points.prj")),
            &context(),
        )
        .expect("a point shapefile");
        assert_eq!(rows, 3);
        let items = plan.items();
        // wp 1: (149.16, -35.36) at 610; wp 2: 620.50 -> 620; wp 3: no elevation -> -1 -> 0.
        assert_eq!(
            (items[0].x, items[0].y, items[0].z),
            (-35.36, 149.16, 610.0)
        );
        assert_eq!(
            (items[1].x, items[1].y, items[1].z),
            (-35.363, 149.165, 620.0)
        );
        assert_eq!((items[2].x, items[2].y, items[2].z), (-35.366, 149.17, 0.0));
        assert!(items.iter().all(|item| item.command == 16));
    }

    /// Without ELEVATION or alt columns the shape's own Z is the altitude, `(int)` of it, and
    /// without a wp column the records keep their order.
    #[test]
    fn load_shp_file_takes_z_from_the_shape_when_the_table_has_no_column() {
        let mut plan = Plan::default();
        let rows = shp_into_mission(
            &mut plan,
            include_bytes!("../../../testdata/planner/pointz.shp"),
            include_bytes!("../../../testdata/planner/pointz.dbf"),
            None,
            &context(),
        )
        .expect("a PointZ shapefile");
        assert_eq!(rows, 2);
        assert_eq!(plan.items()[0].z, 601.0);
        assert_eq!(plan.items()[1].z, 602.0);
        assert_eq!(plan.items()[0].x, -35.36);
    }

    /// `fs.Vertex[row * 2]` is the row-th vertex of the whole file: a one-record polygon gives one
    /// row at its first corner, and a table with more rows than the file has vertices is the
    /// handler's "Error opening File".
    #[test]
    fn load_shp_file_indexes_vertices_by_table_row() {
        let mut plan = Plan::default();
        let rows = shp_into_mission(
            &mut plan,
            include_bytes!("../../../testdata/planner/field.shp"),
            include_bytes!("../../../testdata/planner/field.dbf"),
            Some(include_str!("../../../testdata/planner/field.prj")),
            &context(),
        )
        .expect("the field");
        assert_eq!(rows, 1);
        assert!((plan.items()[0].x - -35.367_300_938_561_82).abs() < 1e-9);
        // Two records' worth of table over one point.
        let mut plan = Plan::default();
        let why = shp_into_mission(
            &mut plan,
            include_bytes!("../../../testdata/planner/pointz.shp"),
            include_bytes!("../../../testdata/planner/points.dbf"),
            None,
            &context(),
        )
        .expect_err("three rows, two vertices");
        assert!(why.contains("IndexOutOfRange"), "{why}");
        assert_eq!(plan.items().len(), 0, "the rows are added after the loop");
    }

    /// Each of the four entries opens its dialog with the C#'s filter and returns its request.
    #[test]
    fn the_file_entries_ask_for_a_file_and_return_the_request() {
        for (action, filter, make) in [
            (
                MenuAction::LoadAndAppend,
                MISSION_FILTER,
                FileRequest::LoadAndAppend as fn(String) -> FileRequest,
            ),
            (MenuAction::LoadKmlFile, KML_FILTER, FileRequest::LoadKml),
            (
                MenuAction::LoadShpFile,
                SHP_FILTER,
                FileRequest::LoadShpMission,
            ),
            (
                MenuAction::KmlOverlay,
                KML_OVERLAY_FILTER,
                FileRequest::KmlOverlay,
            ),
        ] {
            let mut plan = Plan::default();
            let mut menus = PlanMenus::default();
            choose(&mut plan, &mut menus, action);
            let (title, text, value) = showing(&menus);
            assert_eq!(
                (title, text.as_str(), value.as_str()),
                (OPEN_FILE, filter, "")
            );
            assert_eq!(
                answer(&mut plan, &mut menus, "a-file"),
                Some(make("a-file".to_owned()))
            );
            choose(&mut plan, &mut menus, action);
            assert_eq!(menus.cancel(&mut plan), None, "cancelled: nothing");
        }
    }

    /// KML Overlay's two questions: Yes to the first puts the shapes on the flight screen too, No
    /// does not, and either is followed by the zoom question, whose Yes the screen picks up.
    #[test]
    fn kml_overlays_two_questions() {
        let overlay = mp_kml::read::overlay(&mp_kml::read::without_snippets(include_str!(
            "../../../testdata/planner/route.kml"
        )))
        .expect("an overlay");
        let mut plan = Plan::default();
        plan.set_kml_overlay(Some(overlay));
        assert_eq!(plan.kml_points().len(), 5 + 3 + 1);
        let mut menus = PlanMenus::default();
        menus.ask_kml_to_flight_screen();
        let question = menus.prompt.as_ref().expect("a question");
        assert!(question.is_question());
        assert_eq!(
            (question.title, question.text.as_str()),
            (LOAD_DATA, LOAD_INTO_FLIGHT_SCREEN)
        );
        // Yes.
        assert_eq!(menus.submit(&mut plan, &context()), None);
        assert!(plan.kml_on_flight());
        let question = menus.prompt.as_ref().expect("the zoom question");
        assert_eq!(
            (question.title, question.text.as_str()),
            (ZOOM_TO_TITLE, ZOOM_TO_LOADED)
        );
        assert!(!menus.take_zoom_to_kml());
        assert_eq!(menus.submit(&mut plan, &context()), None);
        assert!(menus.take_zoom_to_kml());
        assert!(!menus.take_zoom_to_kml(), "taken once");
        assert!(menus.prompt.is_none());
        // No, then No.
        plan.set_kml_overlay(plan.kml_overlay().cloned());
        assert!(
            !plan.kml_on_flight(),
            "a new file clears the flight screen's copy"
        );
        menus.ask_kml_to_flight_screen();
        assert_eq!(menus.cancel(&mut plan), None);
        assert!(!plan.kml_on_flight());
        assert_eq!(showing(&menus).0, ZOOM_TO_TITLE);
        assert_eq!(menus.cancel(&mut plan), None);
        assert!(!menus.take_zoom_to_kml());
    }

    /// Create Circle Survey: six boxes in the C#'s order with its defaults, a cancelled box keeping
    /// what it offered, then the ROI and the rings through AddCommand.
    #[test]
    fn create_circle_survey_asks_six_boxes_and_adds_the_rows() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::CreateCircleSurvey);
        let expected = [
            ("startalt", "10"),
            ("endalt", "20"),
            ("seperation", "2"),
            ("radius", "5"),
            ("photos", "50"),
            ("start heading", "0"),
        ];
        let typed = ["12", "14", "2", "5", "4", "0"];
        for (step, ((text, offered), answer_text)) in expected.iter().zip(typed).enumerate() {
            let (title, shown, value) = showing(&menus);
            assert_eq!(
                (title, shown.as_str(), value.as_str()),
                ("", *text, *offered),
                "box {step}"
            );
            assert_eq!(answer(&mut plan, &mut menus, answer_text), None);
        }
        assert!(menus.prompt.is_none());
        // Two rings of five (90 degree steps, 0 to 360), each point a WAYPOINT then a DIGICAM.
        assert_eq!(plan.items().len(), 1 + 2 * 5 * 2);
        assert_eq!(plan.items()[0].command, 201);
        assert_eq!(plan.items()[1].command, 16);
        assert_eq!(plan.items()[1].param1, 2.0, "a two second delay");
        assert_eq!(plan.items()[1].z, 12.0);
        assert_eq!(plan.items()[2].command, 203);
        assert_eq!(
            (plan.items()[2].x, plan.items()[2].y),
            (1.0, 0.0),
            "lat 1, lng 0"
        );
        assert_eq!(plan.items()[20].z, 0.0);
        assert_eq!(plan.items()[19].z, 14.0);

        // Cancelling the sixth box keeps its 0 and leaves the survey to the screen.
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::CreateCircleSurvey);
        for answer_text in ["12", "14", "2", "5", "4"] {
            answer(&mut plan, &mut menus, answer_text);
        }
        assert_eq!(menus.cancel(&mut plan), None);
        assert!(menus.prompt.is_none());
        assert_eq!(plan.items().len(), 0, "not until the screen finishes it");
        let centre = menus.take_survey_pending().expect("a centre to finish at");
        menus.finish_circle_survey(&mut plan, centre, &context());
        assert_eq!(plan.items().len(), 21);
        assert!(menus.take_survey_pending().is_none());

        // A word: int.Parse's FormatException, nothing added.
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::CreateCircleSurvey);
        answer(&mut plan, &mut menus, "ten");
        for _ in 0..5 {
            answer(&mut plan, &mut menus, "");
        }
        assert_eq!(showing(&menus).0, ERROR);
        assert!(showing(&menus).1.contains("System.FormatException"));
        assert_eq!(plan.items().len(), 0);
    }

    /// Enter UTM Coord: the zone typed is kept for the next time, the hemisphere is always the
    /// south, and the row lands where GeoUtility puts it.
    #[test]
    fn enter_utm_coord_adds_a_row_and_keeps_the_zone() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        assert_eq!(menus.utm_zone(), "50s");
        choose(&mut plan, &mut menus, MenuAction::EnterUtmCoord);
        assert_eq!(
            showing(&menus),
            (
                "Zone",
                "Enter Zone. (eg 50S, 11N)".to_owned(),
                "50s".to_owned()
            )
        );
        answer(&mut plan, &mut menus, "55S");
        assert_eq!(
            showing(&menus),
            ("Easting", "Easting".to_owned(), "578994".to_owned())
        );
        answer(&mut plan, &mut menus, "695400");
        assert_eq!(
            showing(&menus),
            ("Northing", "Northing".to_owned(), "6126244".to_owned())
        );
        answer(&mut plan, &mut menus, "6084100");
        assert!(menus.prompt.is_none());
        assert_eq!(plan.items().len(), 1);
        assert!((plan.items()[0].x - -35.367_300_939_431_25).abs() < 1e-9);
        assert!((plan.items()[0].y - 149.150_818_946_518_8).abs() < 1e-9);
        assert_eq!(menus.utm_zone(), "55S");
        // Cancel anywhere adds nothing.
        choose(&mut plan, &mut menus, MenuAction::EnterUtmCoord);
        assert_eq!(showing(&menus).2, "55S");
        answer(&mut plan, &mut menus, "55S");
        assert_eq!(menus.cancel(&mut plan), None);
        assert_eq!(plan.items().len(), 1);
        // A zone that is not a number is the exception, after all three boxes.
        choose(&mut plan, &mut menus, MenuAction::EnterUtmCoord);
        answer(&mut plan, &mut menus, "zone-x");
        answer(&mut plan, &mut menus, "1");
        answer(&mut plan, &mut menus, "2");
        assert_eq!(showing(&menus).0, ERROR);
        assert!(showing(&menus).1.contains("FormatException"));
    }

    /// POI > Add asks for the ID and hands the screen the point; Edit hands it the index.
    #[test]
    fn poi_add_and_edit_hand_the_screen_their_requests() {
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::PoiAdd);
        assert_eq!(
            showing(&menus),
            ("POI", "Enter ID".to_owned(), String::new())
        );
        answer(&mut plan, &mut menus, "Gate");
        assert_eq!(
            menus.take_poi_request(),
            Some(PoiRequest::Add {
                lat: cmac().latitude(),
                lng: cmac().longitude(),
                id: "Gate".to_owned()
            })
        );
        assert_eq!(menus.take_poi_request(), None);
        menus.ask_poi_edit(3);
        answer(&mut plan, &mut menus, "Tower");
        assert_eq!(
            menus.take_poi_request(),
            Some(PoiRequest::Rename {
                index: 3,
                id: "Tower".to_owned()
            })
        );
        choose(&mut plan, &mut menus, MenuAction::PoiAdd);
        assert_eq!(menus.cancel(&mut plan), None);
        assert_eq!(menus.take_poi_request(), None);
    }

    /// Fence Inclusion and Exclusion take the drawn polygon and clear it; the fence's items carry
    /// the inclusion first, then each exclusion.
    #[test]
    fn fence_inclusion_and_exclusion_take_the_drawn_polygon() {
        let mut plan = Plan::default();
        plan.load_polygon(square());
        plan.fence_exclusion_from_polygon();
        assert_eq!(plan.polygon().len(), 0);
        assert_eq!(plan.fence_exclusions().len(), 1);
        assert_eq!(plan.fence_exclusions()[0].len(), 4);
        // Exclusions alone are a fence.
        let items = plan.fence_items().expect("a fence of one exclusion");
        assert_eq!(items.len(), 4);
        assert!(items.iter().all(|item| item.command == 5002));
        let mut ring = square();
        ring.truncate(3);
        plan.load_polygon(ring);
        plan.fence_inclusion_from_polygon();
        assert_eq!(plan.fence().len(), 3);
        assert_eq!(plan.polygon().len(), 0);
        let items = plan.fence_items().expect("inclusion then exclusion");
        assert_eq!(items.len(), 3 + 4);
        assert!(items[..3].iter().all(|item| item.command == 5001));
        assert!(items[3..].iter().all(|item| item.command == 5002));
        assert_eq!(items[3].seq, 3, "numbered after the inclusion");
        plan.clear_fence();
        assert!(plan.fence_exclusions().is_empty());
        // An empty polygon makes no exclusion.
        plan.fence_exclusion_from_polygon();
        assert!(plan.fence_exclusions().is_empty());
    }

    // ---- Auto WP's circles ----

    /// Answers every question a circle asks, in turn, returning what each offered.
    fn answer_all(
        plan: &mut Plan,
        menus: &mut PlanMenus,
        answers: &[&str],
    ) -> Vec<(String, String, String)> {
        let mut asked = Vec::new();
        for value in answers {
            let (title, text, offered) = showing(menus);
            asked.push((title.to_owned(), text, offered));
            answer(plan, menus, value);
        }
        asked
    }

    /// Create Wp Circle asks its four questions with the C#'s captions and offers, then adds a
    /// `WAYPOINT` row per point of the C#'s circle under mono, at Default Alt, in the screen's
    /// frame.
    #[test]
    fn a_wp_circle_asks_four_questions_and_adds_the_c_sharps_points() {
        let golden = include_str!("../../../testdata/planner/golden/wp_circle.csv");
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::CreateWpCircle);
        let asked = answer_all(&mut plan, &mut menus, &["50", "20", "1", "0"]);
        let asked: Vec<(&str, &str, &str)> = asked
            .iter()
            .map(|(a, b, c)| (a.as_str(), b.as_str(), c.as_str()))
            .collect();
        assert_eq!(
            asked,
            [
                ("Radius", "Radius", "50"),
                ("Points", "Number of points to generate Circle", "20"),
                ("Points", "Direction of circle (-1 or 1)", "1"),
                ("angle", "Angle of first point (whole degrees)", "0"),
            ]
        );
        assert!(menus.prompt.is_none());
        let expected: Vec<Vec<String>> = golden_case(golden, "default");
        assert_eq!(plan.items().len(), expected.len());
        for (item, row) in plan.items().iter().zip(&expected) {
            assert_eq!(item.command, mp_mission::commands::WAYPOINT);
            // To the bit where the golden was made (glibc under mono), within the documented
            // last-bits allowance elsewhere: the Windows runner's libm put one coordinate a bit
            // off (2026-10-04). `mp_units::golden_match` says how far.
            assert!(
                mp_units::golden_match(item.x, number(&row[1])),
                "x {} where the C# has {}",
                item.x,
                row[1]
            );
            assert!(
                mp_units::golden_match(item.y, number(&row[2])),
                "y {} where the C# has {}",
                item.y,
                row[2]
            );
            assert_eq!(item.z, 100.0, "Default Alt");
            assert_eq!(item.frame, FRAME_RELATIVE);
        }
        assert_eq!(plan.items()[20].seq, 21, "numbered from 1");
    }

    /// Every answer is asked for before any is read, and the first that `int.TryParse` refuses
    /// is said in the C#'s words; Cancel at any question adds nothing.
    #[test]
    fn a_wp_circle_refuses_as_the_c_sharp_does() {
        for (answers, message) in [
            (["fifty", "20", "1", "0"], "Bad Radius"),
            (["50", "x", "y", "0"], "Bad Point value"),
            (["50", "20", "up", "0"], "Bad Direction value"),
            (["50", "20", "1", "north"], "Bad start angle value"),
        ] {
            let mut plan = Plan::default();
            let mut menus = PlanMenus::default();
            choose(&mut plan, &mut menus, MenuAction::CreateWpCircle);
            answer_all(&mut plan, &mut menus, &answers);
            assert_eq!(showing(&menus), ("", message.to_owned(), String::new()));
            assert!(plan.items().is_empty());
        }
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::CreateWpCircle);
        answer_all(&mut plan, &mut menus, &["50", "20"]);
        menus.cancel(&mut plan);
        assert!(menus.prompt.is_none() && plan.items().is_empty());
        // A Default Alt `float.Parse` refuses throws in the C#: its message, and nothing added.
        plan.set_panel_text(PanelBox::DefaultAlt, "high");
        choose(&mut plan, &mut menus, MenuAction::CreateWpCircle);
        answer_all(&mut plan, &mut menus, &["50", "20", "1", "0"]);
        assert_eq!(
            showing(&menus),
            (ERROR, FORMAT_EXCEPTION.to_owned(), String::new())
        );
        assert!(plan.items().is_empty());
    }

    /// Create Spline Circle asks five questions, then adds a `DO_SET_ROI` at the menu's position
    /// at 0 and the C#'s laps of `SPLINE_WAYPOINT`s at its altitudes; a step under 4 is refused
    /// where the C# would lap forever.
    #[test]
    fn a_spline_circle_adds_the_roi_and_the_c_sharps_laps() {
        let golden = include_str!("../../../testdata/planner/golden/spline_circle.csv");
        let mut plan = Plan::default();
        let mut menus = PlanMenus::default();
        choose(&mut plan, &mut menus, MenuAction::CreateSplineCircle);
        let asked = answer_all(&mut plan, &mut menus, &["50", "5", "20", "5", "0"]);
        let titles: Vec<&str> = asked.iter().map(|(title, ..)| title.as_str()).collect();
        assert_eq!(
            titles,
            ["Radius", "min alt", "max alt", "alt step", "angle"]
        );
        let expected = golden_case(golden, "default");
        let items = plan.items();
        assert_eq!(items.len(), expected.len());
        assert_eq!(items[0].command, mp_mission::commands::DO_SET_ROI);
        assert_eq!(
            (items[0].x, items[0].y, items[0].z),
            (cmac().latitude(), cmac().longitude(), 0.0)
        );
        for (item, row) in items.iter().zip(&expected).skip(1) {
            assert_eq!(item.command, mp_mission::commands::SPLINE_WAYPOINT);
            assert_eq!(item.x.to_bits(), number(&row[1]).to_bits());
            assert_eq!(item.y.to_bits(), number(&row[2]).to_bits());
            assert_eq!(item.z, number(&row[3]));
        }

        let mut plan = Plan::default();
        choose(&mut plan, &mut menus, MenuAction::CreateSplineCircle);
        answer_all(&mut plan, &mut menus, &["50", "5", "20", "3", "0"]);
        assert_eq!(
            showing(&menus),
            ("", "Bad alt step".to_owned(), String::new())
        );
        assert!(plan.items().is_empty());
    }

    // ---- Add Below ----

    /// Add Below appends while there is at most one row or the current row is the last, and
    /// otherwise puts the row after the current one - after the first when none is current - and
    /// selects it: a `WAYPOINT` of zeroes in the screen's frame.
    #[test]
    fn add_below_puts_a_blank_row_where_the_c_sharp_does() {
        let mut plan = Plan::default();
        plan.add_below(AltitudeFrame::Absolute);
        assert_eq!(plan.items().len(), 1);
        let row = plan.items()[0];
        assert_eq!(
            (row.command, row.frame, row.x, row.y, row.z),
            (
                mp_mission::commands::WAYPOINT,
                FRAME_ABSOLUTE,
                0.0,
                0.0,
                0.0
            )
        );
        assert_eq!(plan.selected(), Some(1));
        plan.add_waypoint_in(cmac(), 50.0, AltitudeFrame::Relative);
        plan.add_waypoint_in(cmac(), 60.0, AltitudeFrame::Relative);
        // Row 1 current, three rows: the new one goes second.
        plan.select(Some(1));
        plan.add_below(AltitudeFrame::Relative);
        let altitudes: Vec<f64> = plan.items().iter().map(|item| item.z).collect();
        assert_eq!(altitudes, [0.0, 0.0, 50.0, 60.0]);
        assert_eq!(plan.selected(), Some(2));
        // The last current: appended.
        plan.select(Some(4));
        plan.add_below(AltitudeFrame::Relative);
        assert_eq!(plan.items().len(), 5);
        assert_eq!(plan.selected(), Some(5));
        // None current: after row 0, the first.
        plan.select(None);
        plan.add_below(AltitudeFrame::Relative);
        assert_eq!(plan.selected(), Some(2));
        assert_eq!(plan.items().len(), 6);
    }

    // ---- The map ----

    /// The rally pin's outline starts at its point and goes round its head.
    #[test]
    fn the_rally_pin_is_a_pin_about_its_head() {
        let outline = mapview::pin_outline_at(
            mapview::RALLY_PIN_HEAD,
            mapview::RALLY_PIN_RADIUS,
            mapview::RALLY_PIN_TIP,
        );
        assert_eq!(outline.first(), Some(&mapview::RALLY_PIN_TIP));
        assert_eq!(outline.len(), 26);
        for (x, y) in &outline[1..] {
            let (hx, hy) = mapview::RALLY_PIN_HEAD;
            assert!(((x - hx).hypot(y - hy) - mapview::RALLY_PIN_RADIUS).abs() < 1e-3);
        }
        // Over the top: the arc reaches the head's highest point.
        assert!(outline.iter().any(|(_, y)| *y < -37.0));
    }
}

/// Terrain on the planning screen (PLAN.md §13.4 row 35): Verify Height's altitudes, a drag's,
/// home from the map, and the Elevation Graph - each held to the C#'s arithmetic, worked by hand
/// over terrain whose heights are known.
#[cfg(test)]
mod terrain_tests {
    use super::*;
    use crate::srtm::{AltResponse, TileType};
    use elevation::{ElevationProfile, Scale};
    use mp_mission::commands as cmd;

    fn at(lat: f64, lng: f64) -> LatLon {
        LatLon::new(lat, lng).expect("a valid position")
    }

    const fn valid(alt: f64) -> AltResponse {
        AltResponse {
            current_type: TileType::Valid,
            alt,
            alt_source: "SRTM",
        }
    }

    /// The synthetic tile's flat 584 m.
    fn flat(_: f64, _: f64) -> AltResponse {
        valid(584.0)
    }

    /// Two plateaus: 584.7 m at home's latitude and north of -35.37, 612.9 m south of it.
    fn plateaus(lat: f64, _: f64) -> AltResponse {
        if lat < -35.37 {
            valid(612.9)
        } else {
            valid(584.7)
        }
    }

    /// Below the sea south of -35.37, 584.7 m north of it.
    fn sea(lat: f64, _: f64) -> AltResponse {
        if lat < -35.37 {
            valid(-3.7)
        } else {
            valid(584.7)
        }
    }

    /// No tile anywhere.
    fn nothing(_: f64, _: f64) -> AltResponse {
        AltResponse::INVALID
    }

    /// Home in the boxes at -35.36, 149.16 (584.7 m of ground under it), Default Alt 100, Verify
    /// Height ticked, over `terrain`.
    fn plan_over(terrain: fn(f64, f64) -> AltResponse) -> Plan {
        let mut plan = Plan::default();
        plan.set_terrain(Terrain(terrain));
        plan.set_home_text(HomeBox::Lat, "-35.36".to_owned());
        plan.set_home_text(HomeBox::Lng, "149.16".to_owned());
        plan.set_home_text(HomeBox::Alt, "584.70".to_owned());
        plan.set_verify_height(true);
        plan
    }

    // ---- Verify Height, `setfromMap` ----

    /// The box is clear as the Designer leaves it, and while clear the altitude stands.
    #[test]
    fn verify_height_starts_clear_and_clear_changes_nothing() {
        let mut plan = plan_over(plateaus);
        assert!(!Plan::default().verify_height());
        plan.set_verify_height(false);
        for frame in AltitudeFrame::all() {
            assert_eq!(plan.verified_altitude(-35.38, 149.16, 37.0, frame), 37.0);
        }
    }

    /// Absolute: the ground at the row, 612.9, plus Default Alt, 100, in double - 712.9.
    #[test]
    fn verify_height_in_absolute_is_the_ground_plus_default_alt() {
        let plan = plan_over(plateaus);
        let altitude = plan.verified_altitude(-35.38, 149.16, 100.0, AltitudeFrame::Absolute);
        assert!((altitude - (612.9 + 100.0)).abs() < 1e-9, "{altitude}");
        // No tile: `alt` 0, which the C# does not look past - Default Alt alone.
        let plan = plan_over(nothing);
        assert_eq!(
            plan.verified_altitude(-35.38, 149.16, 100.0, AltitudeFrame::Absolute),
            100.0
        );
    }

    /// Terrain: Default Alt, whatever was passed.
    #[test]
    fn verify_height_in_terrain_is_default_alt() {
        let plan = plan_over(plateaus);
        assert_eq!(
            plan.verified_altitude(-35.38, 149.16, 37.0, AltitudeFrame::Terrain),
            100.0
        );
    }

    /// Relative: `(int)612.9 + 100 - (int)584.7` = 612 + 100 - 584 = 128, the planned home's
    /// ground subtracted; a row on home's plateau gets Default Alt.
    #[test]
    fn verify_height_in_relative_keeps_the_row_default_alt_above_the_ground_under_it() {
        let plan = plan_over(plateaus);
        assert_eq!(
            plan.verified_altitude(-35.38, 149.16, 100.0, AltitudeFrame::Relative),
            128.0
        );
        assert_eq!(
            plan.verified_altitude(-35.365, 149.16, 100.0, AltitudeFrame::Relative),
            100.0
        );
        // `(int)` cuts toward zero: -3.7 is -3, so -3 + 100 - 584 = -487.
        let plan = plan_over(sea);
        assert_eq!(
            plan.verified_altitude(-35.38, 149.16, 100.0, AltitudeFrame::Relative),
            -487.0
        );
    }

    /// The altitude passed is not what Verify Height uses: Default Alt is, even 0, where the lines
    /// before it gave 50.
    #[test]
    fn verify_height_takes_default_alt_not_the_altitude_handed_in() {
        let mut plan = plan_over(plateaus);
        plan.set_panel_text(PanelBox::DefaultAlt, "0");
        let given = plan.new_row_altitude(0.0, false).expect("0 is a number");
        assert_eq!(given, 50.0);
        assert_eq!(
            plan.verified_altitude(-35.365, 149.16, given, AltitudeFrame::Relative),
            0.0
        );
    }

    /// The menu's rows go through `setfromMap` too: Loiter > Forever in Absolute over the high
    /// plateau is at 712.9, and Land - handed 1 - at the ground plus Default Alt as well.
    #[test]
    fn the_menu_rows_are_verified_where_the_menu_was_opened() {
        let mut plan = plan_over(plateaus);
        let mut menus = PlanMenus::default();
        let context = MenuContext {
            frame: AltitudeFrame::Absolute,
            vehicle: None,
            takeoff_pitch: false,
            copter: false,
            tracker_alt: 0.0,
        };
        let south = at(-35.38, 149.16);
        menus.open_at((10.0, 10.0), south, None);
        menus.choose(&mut plan, MenuAction::LoiterForever, &context);
        menus.open_at((10.0, 10.0), south, None);
        menus.choose(&mut plan, MenuAction::Land, &context);
        let alts: Vec<f64> = plan.items().iter().map(|item| item.z).collect();
        assert_eq!(alts.len(), 2);
        assert!(
            alts.iter().all(|alt| (alt - 712.9).abs() < 1e-9),
            "{alts:?}"
        );
        // Box clear: the land keeps the 1 it was handed.
        plan.set_verify_height(false);
        menus.open_at((10.0, 10.0), south, None);
        menus.choose(&mut plan, MenuAction::Land, &context);
        assert_eq!(plan.items().last().map(|item| item.z), Some(1.0));
    }

    /// Insert Wp's row, placed once its number is answered, is verified at the menu's position.
    #[test]
    fn an_inserted_waypoint_is_verified_at_the_menu_position() {
        let mut plan = plan_over(plateaus);
        let mut menus = PlanMenus::default();
        let context = MenuContext {
            frame: AltitudeFrame::Relative,
            vehicle: None,
            takeoff_pitch: false,
            copter: false,
            tracker_alt: 0.0,
        };
        menus.open_at((10.0, 10.0), at(-35.38, 149.16), None);
        menus.choose(&mut plan, MenuAction::InsertWp, &context);
        menus.submit(&mut plan, &context);
        assert_eq!(plan.items().first().map(|item| item.z), Some(128.0));
    }

    /// An `InputBox`'s OK hands its caption, question and text over to be kept as `InputBox`
    /// keeps every titled answer - Insert WP's even when the handler then refuses the number -
    /// once; Cancel, a file dialog's stand-in and an untitled box keep nothing.
    /// `// C#: ExtLibs/Controls/InputBox.cs:73-84, 178-184; GCSViews/FlightPlanner.cs:4073, 4779`
    #[test]
    fn an_input_box_ok_keeps_its_answer_under_the_input_box_key() {
        use crate::config::optional::{answers_key, remember_answer};
        let insert = answers_key("Insert WP", "Insert WP after wp#");
        let loiter = answers_key("Loiter Time", "Loiter Time");
        assert_eq!(insert, "InputBoxInsertWPInsertWPafterwp");
        assert_eq!(loiter, "InputBoxLoiterTimeLoiterTime");
        for key in [&insert, &loiter] {
            assert!(crate::settings::PUBLISHED.contains(&key.as_str()), "{key}");
        }
        let mut plan = plan_over(plateaus);
        let mut menus = PlanMenus::default();
        let context = MenuContext {
            frame: AltitudeFrame::Relative,
            vehicle: None,
            takeoff_pitch: false,
            copter: false,
            tracker_alt: 0.0,
        };
        let mut settings = crate::settings::Persisted::at(None);
        let mut keep = |menus: &mut PlanMenus| {
            let (title, question, answer) = menus.take_answered()?;
            remember_answer(&mut settings, title, &question, &answer);
            Some(answer)
        };
        menus.open_at((10.0, 10.0), at(-35.38, 149.16), None);
        menus.choose(&mut plan, MenuAction::LoiterTime, &context);
        menus.cancel(&mut plan);
        assert_eq!(keep(&mut menus), None, "Cancel keeps nothing");
        menus.open_at((10.0, 10.0), at(-35.38, 149.16), None);
        menus.choose(&mut plan, MenuAction::LoiterTime, &context);
        menus.submit(&mut plan, &context);
        assert_eq!(keep(&mut menus).as_deref(), Some("5"));
        assert_eq!(keep(&mut menus), None, "kept once");
        menus.open_at((10.0, 10.0), at(-35.38, 149.16), None);
        menus.choose(&mut plan, MenuAction::InsertWp, &context);
        if let Some(field) = menus.prompt.as_mut().and_then(|p| p.field.as_mut()) {
            field.set("9");
        }
        menus.submit(&mut plan, &context);
        assert_eq!(
            keep(&mut menus).as_deref(),
            Some("9"),
            "kept before the refusal"
        );
        menus.prompt = None;
        menus.open_at((10.0, 10.0), at(-35.38, 149.16), None);
        menus.choose(&mut plan, MenuAction::LoadPolygon, &context);
        assert!(menus.prompt.as_ref().is_some_and(Prompt::is_file_dialog));
        menus.submit(&mut plan, &context);
        assert_eq!(keep(&mut menus), None, "a file dialog keeps nothing");
        assert_eq!(settings.get(&insert), Some("9"));
        assert_eq!(settings.get(&loiter), Some("5"));
        // An untitled box - Create Circle Survey's - has no list to keep.
        remember_answer(&mut settings, "", "startalt", "10");
        assert_eq!(settings.get(&answers_key("", "startalt")), None);
    }

    // ---- A drag, `callMeDrag(..., -2)` ----

    /// A waypoint dragged from home's plateau (584.7, cut to 584) to the high one (612.9, cut to
    /// 612) climbs by the difference, its own 100.9 cut to 100 first: 128.
    #[test]
    fn a_dragged_waypoint_keeps_its_height_above_the_ground() {
        let mut plan = plan_over(plateaus);
        plan.append(cmd::waypoint(at(-35.365, 149.16), 100.9, FRAME_RELATIVE));
        plan.begin_drag(1);
        plan.move_to(1, at(-35.38, 149.16));
        plan.end_drag(1, AltitudeFrame::Relative, None);
        assert_eq!(plan.items().first().map(|item| item.z), Some(128.0));
        // And back down again: 128 + 584 - 612.
        plan.begin_drag(1);
        plan.move_to(1, at(-35.365, 149.16));
        plan.end_drag(1, AltitudeFrame::Absolute, None);
        assert_eq!(plan.items().first().map(|item| item.z), Some(100.0));
    }

    /// Nothing changes with the box clear, in the Terrain frame, for a row not moved, or for a
    /// command whose Z column is not headed "Alt" in the vehicle's `mavcmd.xml` section.
    #[test]
    fn a_drag_leaves_the_altitude_where_the_c_sharp_does() {
        let dragged = |plan: &mut Plan, command: u16, frame, family| {
            plan.clear_mission();
            let mut row = cmd::waypoint(at(-35.365, 149.16), 100.9, FRAME_RELATIVE);
            row.command = command;
            plan.append(row);
            plan.begin_drag(1);
            plan.move_to(1, at(-35.38, 149.16));
            plan.end_drag(1, frame, family);
            plan.items().first().map(|item| item.z)
        };
        let mut plan = plan_over(plateaus);
        assert_eq!(
            dragged(&mut plan, 16, AltitudeFrame::Terrain, None),
            Some(100.9)
        );
        // VTOL_LAND's Z is "Alt" for a plane (APM) and not in the copter's section (AC2).
        let plane = Some(mp_vehicle::VehicleFamily::Plane);
        assert_eq!(
            dragged(&mut plan, 85, AltitudeFrame::Relative, None),
            Some(100.9)
        );
        assert_eq!(
            dragged(&mut plan, 85, AltitudeFrame::Relative, plane),
            Some(128.0)
        );
        plan.set_verify_height(false);
        assert_eq!(
            dragged(&mut plan, 16, AltitudeFrame::Relative, None),
            Some(100.9)
        );
        // Grabbed and let go where it was: never dragged, and its 100.9 is not cut to 100.
        plan.set_verify_height(true);
        plan.begin_drag(1);
        plan.end_drag(1, AltitudeFrame::Relative, None);
        assert_eq!(plan.items().first().map(|item| item.z), Some(100.9));
    }

    /// `alt_column` is `mavcmd.xml`'s: every command whose `Z` heading starts "Alt", section by
    /// section, read from the C# tree when it is present.
    #[test]
    fn the_alt_column_is_the_one_mavcmd_xml_heads_alt() {
                let Some(xml) = crate::config_coverage::source::csharp("mavcmd.xml") else {
            println!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let number = |name: &str| -> u16 {
            match name {
                "WAYPOINT" => 16,
                "LOITER_UNLIM" => 17,
                "LOITER_TURNS" => 18,
                "LOITER_TIME" => 19,
                "LAND" => 21,
                "TAKEOFF" => 22,
                "CONTINUE_AND_CHANGE_ALT" => 30,
                "LOITER_TO_ALT" => 31,
                "ARC_WAYPOINT" => 36,
                "SPLINE_WAYPOINT" => 82,
                "VTOL_TAKEOFF" => 84,
                "VTOL_LAND" => 85,
                "PAYLOAD_PLACE" => 94,
                "DO_SET_HOME" => 179,
                "DO_SET_ROI_LOCATION" => 195,
                "DO_SET_ROI" => 201,
                other => panic!("{other} is headed Alt and has no number here"),
            }
        };
        for (section, family) in [
            ("AC2", None),
            ("APM", Some(mp_vehicle::VehicleFamily::Plane)),
            ("APRover", Some(mp_vehicle::VehicleFamily::Rover)),
        ] {
            let open = format!("<{section}>");
            let close = format!("</{section}>");
            let body = xml
                .split_once(open.as_str())
                .and_then(|(_, rest)| rest.split_once(close.as_str()))
                .map(|(body, _)| body)
                .expect("the section");
            let mut headed_alt = Vec::new();
            let mut command = "";
            for line in body.lines().map(str::trim) {
                // A command opens on a line of its own; its seven cells close on theirs.
                if line.starts_with('<') && !line.contains("</") && !line.starts_with("<!--") {
                    command = line.trim_start_matches('<').trim_end_matches('>');
                }
                if line.starts_with("<Z")
                    && line
                        .split_once('>')
                        .is_some_and(|(_, text)| text.starts_with("Alt"))
                {
                    headed_alt.push(number(command));
                }
            }
            assert!(!headed_alt.is_empty(), "{section}");
            for command in 0..=u16::MAX {
                assert_eq!(
                    alt_column(command, family),
                    headed_alt.contains(&command),
                    "{section} {command}"
                );
            }
        }
    }

    // ---- Home from the map ----

    /// Set Home Here: the ASL box takes the ground there `ToString("0.00")`, then Lat and Long,
    /// and the planned home follows.
    #[test]
    fn set_home_here_puts_home_at_the_ground_there() {
        let mut plan = plan_over(plateaus);
        let mut menus = PlanMenus::default();
        menus.open_at((10.0, 10.0), at(-35.38, 149.17), None);
        menus.choose(
            &mut plan,
            MenuAction::SetHomeHere,
            &MenuContext {
                frame: AltitudeFrame::Relative,
                vehicle: None,
                takeoff_pitch: false,
                copter: false,
                tracker_alt: 0.0,
            },
        );
        assert_eq!(plan.home_text(HomeBox::Alt), "612.90");
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.38");
        assert_eq!(plan.home_text(HomeBox::Lng), "149.17");
        assert!((plan.planned_home().alt - 612.9).abs() < 1e-9);
        assert!(plan.items().is_empty());
        // No tile: 0, as the C# writes `alt` whatever kind of answer it is.
        let mut plan = plan_over(nothing);
        plan.set_home_at(at(-35.38, 149.17));
        assert_eq!(plan.home_text(HomeBox::Alt), "0.00");
    }

    /// Entering the Lat box says "Click on the Map to set Home " once; the next click on the map
    /// moves home there instead of adding a row; any change to a box forgets the request.
    #[test]
    fn entering_the_lat_box_makes_the_next_click_set_home() {
        let mut plan = plan_over(flat);
        assert_eq!(plan.home_lat_enter(), Some(CLICK_TO_SET_HOME));
        assert!(plan.sethome());
        assert_eq!(plan.home_lat_enter(), None);
        assert!(plan.click_sets_home(at(-35.37, 149.18)));
        assert!(!plan.sethome());
        assert_eq!(plan.home_text(HomeBox::Alt), "584.00");
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.37");
        assert!(plan.items().is_empty());
        // Once done, a click adds rows again.
        assert!(!plan.click_sets_home(at(-35.37, 149.18)));

        // A typed change clears it.
        assert_eq!(plan.home_lat_enter(), Some(CLICK_TO_SET_HOME));
        plan.set_home_text(HomeBox::Lng, "149.2".to_owned());
        assert!(!plan.sethome());
        assert!(!plan.click_sets_home(at(-35.37, 149.18)));

        // Drawing a polygon, the click is a corner first.
        plan.home_lat_enter();
        plan.set_draw_mode(DrawMode::Area);
        assert!(!plan.click_sets_home(at(-35.37, 149.18)));
        assert!(plan.sethome());
    }

    /// Insert Wp > At Current Position with `sethome` set moves home to the vehicle.
    #[test]
    fn at_current_position_moves_home_to_the_vehicle_when_asked_to() {
        let mut plan = plan_over(flat);
        let mut menus = PlanMenus::default();
        plan.home_lat_enter();
        menus.open_at((10.0, 10.0), at(-35.36, 149.16), None);
        menus.choose(
            &mut plan,
            MenuAction::InsertAtCurrentPosition,
            &MenuContext {
                frame: AltitudeFrame::Relative,
                vehicle: Some((at(-35.35, 149.15), 30.0)),
                takeoff_pitch: false,
                copter: false,
                tracker_alt: 0.0,
            },
        );
        assert!(plan.items().is_empty());
        assert_eq!(plan.home_text(HomeBox::Lat), "-35.35");
        assert_eq!(plan.home_text(HomeBox::Alt), "584.00");
    }

    // ---- The Elevation Graph ----

    /// Home at 584.1 and two waypoints 0.0005 degrees apart due south, 100 m in `frame`.
    fn route(frame: u8) -> (Option<Home>, Vec<MissionItem>) {
        let home = Home {
            lat: -35.0,
            lng: 149.0,
            alt: 584.1,
        };
        let mut plan = Plan::default();
        plan.append(cmd::waypoint(at(-35.0005, 149.0), 100.0, frame));
        plan.append(cmd::waypoint(at(-35.001, 149.0), 100.0, frame));
        (Some(home), plan.items().to_vec())
    }

    /// Each leg is 55.597463 m (`GetDistance` on its 6371 km sphere), so 6 steps, 7 samples each
    /// end included: 14 DEM points, the flat 584; the distance 55 + 55 = 110. In the Relative
    /// frame the planned path is the points themselves - home at 584.1, the rows at 100 + 584.1 -
    /// tagged H, 1, 2, at 0, 55.597 and 111.195 m along.
    #[test]
    fn the_relative_graph_plots_the_points_over_the_ground() {
        let (home, items) = route(FRAME_RELATIVE);
        let points = mapview::point_list(home, &items, &flat);
        let profile = ElevationProfile::new(points, AltitudeFrame::Relative, Terrain(flat))
            .expect("three points");
        assert_eq!(profile.distance, 110);
        assert_eq!(profile.dem.len(), 14);
        assert!(profile.dem.iter().all(|point| point.y == 584.0));
        let xs = [
            0.0,
            9.266_243_887_376_07,
            18.532_487_774_044_817,
            27.798_731_661_420_888,
            37.064_975_548_796_96,
            46.331_219_435_465_705,
            55.597_463_322_841_776,
            55.597_463_322_841_776,
            64.863_707_210_217_85,
            74.129_951_096_886_6,
            83.396_194_983_555_35,
            92.662_438_870_931_42,
            101.928_682_758_307_5,
            111.194_926_644_268_93,
        ];
        for (point, x) in profile.dem.iter().zip(xs) {
            assert!((point.x - x).abs() < 1e-9, "{} against {x}", point.x);
        }
        let planned: Vec<(f64, f64, Option<&str>)> = profile
            .planned
            .iter()
            .map(|point| (point.x, point.y, point.tag.as_deref()))
            .collect();
        assert_eq!(planned.len(), 3);
        for ((x, y, tag), (want_x, want_y, want_tag)) in planned.iter().zip([
            (0.0, 584.1, "H"),
            (55.597_463_322_841_755, 684.1, "1"),
            (111.194_926_644_268_86, 684.1, "2"),
        ]) {
            assert!((x - want_x).abs() < 1e-9, "{x}");
            assert!((y - want_y).abs() < 1e-9, "{y}");
            assert_eq!(*tag, Some(want_tag));
        }
        // X from 0 to 110 in steps of 20 (110 / 7 = 15.7, promoted to 2 x 10); Y from the data's
        // 584..684.1 with a tenth either side, 573.99..694.11, out to whole steps of 20.
        assert_eq!(
            (
                profile.x_axis.min,
                profile.x_axis.max,
                profile.x_axis.major_step
            ),
            (0.0, 110.0, 20.0)
        );
        let labels = |scale: &Scale| -> Vec<String> {
            scale.tics().iter().map(|tic| scale.label(*tic)).collect()
        };
        assert_eq!(
            labels(&profile.x_axis),
            ["0", "20", "40", "60", "80", "100"]
        );
        assert_eq!(profile.y_axis.major_step, 20.0);
        assert!((profile.y_axis.min - 560.0).abs() < 1e-9);
        assert!((profile.y_axis.max - 700.0).abs() < 1e-9);
        assert_eq!(
            labels(&profile.y_axis),
            ["560", "580", "600", "620", "640", "660", "680", "700"]
        );
    }

    /// In the Terrain frame the rows sit 100 above the ground (584 + 100 = 684 on the list), and
    /// the ground under each point comes off again: home 0.1 above it, the rows 100. The planned
    /// path is the ground plus the leg's altitude at each sample, `Convert.ToInt32`: 584.1 is
    /// 584, 600.75 is 601, 617.4 617, 634.05 634, 650.7 651, 667.35 667, then 684 - untagged.
    #[test]
    fn the_terrain_graph_follows_the_ground() {
        let (home, items) = route(FRAME_TERRAIN);
        let points = mapview::point_list(home, &items, &flat);
        let alts: Vec<f64> = points.iter().flatten().map(|point| point.alt).collect();
        assert_eq!(alts, [584.1, 684.0, 684.0]);
        let profile = ElevationProfile::new(points, AltitudeFrame::Terrain, Terrain(flat))
            .expect("three points");
        let ys: Vec<f64> = profile.planned.iter().map(|point| point.y).collect();
        assert_eq!(
            ys,
            [
                584.0, 601.0, 617.0, 634.0, 651.0, 667.0, 684.0, 684.0, 684.0, 684.0, 684.0, 684.0,
                684.0, 684.0
            ]
        );
        assert!(profile.planned.iter().all(|point| point.tag.is_none()));
        assert_eq!(profile.dem.len(), 14);
        // 584..684 and a tenth of 100 either side: 574..694, out to 560..700 in 20s.
        let labels: Vec<String> = profile
            .y_axis
            .tics()
            .iter()
            .map(|tic| profile.y_axis.label(*tic))
            .collect();
        assert_eq!(
            labels,
            ["560", "580", "600", "620", "640", "660", "680", "700"]
        );
    }

    /// One point, or none, is "Please plan something first"; a DO_SET_ROI and a DO_JUMP do not
    /// count.
    #[test]
    fn too_little_planned_is_refused() {
        let empty = ElevationProfile::new(Vec::new(), AltitudeFrame::Relative, Terrain(flat));
        assert_eq!(empty, Err(elevation::PLAN_SOMETHING));
        let (home, _) = route(FRAME_RELATIVE);
        let rows = [
            cmd::set_roi(at(-35.001, 149.0), 0.0, FRAME_RELATIVE),
            cmd::do_jump(1.0, 2.0, FRAME_RELATIVE),
        ];
        let points = mapview::point_list(home, &rows, &flat);
        assert_eq!(points.len(), 3);
        assert_eq!(
            ElevationProfile::new(points, AltitudeFrame::Relative, Terrain(flat)),
            Err(elevation::PLAN_SOMETHING)
        );
    }

    /// Map Tool > Elevation Graph opens the form from the plan as it stands, or says why not.
    #[test]
    fn the_menu_opens_the_form_or_says_plan_something_first() {
        let mut plan = Plan::default();
        plan.set_terrain(Terrain(flat));
        let mut menus = PlanMenus::default();
        let context = MenuContext {
            frame: AltitudeFrame::Relative,
            vehicle: None,
            takeoff_pitch: false,
            copter: false,
            tracker_alt: 0.0,
        };
        menus.open_at((10.0, 10.0), at(-35.0, 149.0), None);
        menus.choose(&mut plan, MenuAction::ElevationGraph, &context);
        assert!(menus.elevation.is_none());
        let prompt = menus.prompt.take().expect("a message");
        assert_eq!(
            (prompt.title, prompt.text.as_str()),
            (ERROR, elevation::PLAN_SOMETHING)
        );

        let (home, items) = route(FRAME_RELATIVE);
        let home = home.expect("a home");
        plan.set_home_text(HomeBox::Lat, double_text(home.lat));
        plan.set_home_text(HomeBox::Lng, double_text(home.lng));
        plan.set_home_text(HomeBox::Alt, double_text(home.alt));
        for item in items {
            plan.append(item);
        }
        menus.open_at((10.0, 10.0), at(-35.0, 149.0), None);
        menus.choose(&mut plan, MenuAction::ElevationGraph, &context);
        assert!(menus.prompt.is_none());
        let facts = elevation::facts(menus.elevation.as_ref());
        let fact = |key: &str| {
            facts
                .iter()
                .find(|(name, _)| *name == key)
                .map(|(_, value)| value.clone())
        };
        assert_eq!(fact("plan.elevation").as_deref(), Some("open"));
        assert_eq!(
            fact("plan.elevation.planned.tags").as_deref(),
            Some("H,1,2")
        );
        assert_eq!(
            fact("plan.elevation.planned.y").as_deref(),
            Some("584.1,684.1,684.1")
        );
        assert_eq!(fact("plan.elevation.dem").as_deref(), Some("14"));
        assert_eq!(
            fact("plan.elevation.dem.range").as_deref(),
            Some("584..584")
        );
        assert_eq!(fact("plan.elevation.distance").as_deref(), Some("110"));
        assert_eq!(
            fact("plan.elevation.titles").as_deref(),
            Some("Elevation above ground|Distance (m)|Elevation (m)")
        );
        assert_eq!(
            fact("plan.elevation.legend").as_deref(),
            Some("Planned Path,DEM")
        );
        assert_eq!(
            fact("plan.elevation.xlabels").as_deref(),
            Some("0,20,40,60,80,100")
        );
    }

    /// ZedGraph's scale by hand: the zero lever pulls a minimum within a quarter of the range to
    /// 0; a flat range is widened by a fifth; a range past 10^3 is labelled in thousands with
    /// the magnitude on the title.
    #[test]
    fn the_scale_is_zedgraphs() {
        // 10..100: grace 9 either side, -> 1..109; 1 / 108 < 0.25, so 0; step 109 / 7 = 15.6
        // -> 20; max out to 120.
        let scale = Scale::pick(10.0, 100.0, None);
        assert_eq!((scale.min, scale.major_step), (0.0, 20.0));
        assert!((scale.max - 120.0).abs() < 1e-9, "{}", scale.max);
        assert_eq!(scale.minor_step, 5.0);
        // 584..584: range 0, grace nothing; equal ends, so 584 * 0.95 .. 584 * 1.05 = 554.8..613.2;
        // step 58.4 / 7 = 8.3 -> 10; out to 550..620.
        let scale = Scale::pick(584.0, 584.0, None);
        assert!((scale.min - 550.0).abs() < 1e-9 && (scale.max - 620.0).abs() < 1e-9);
        assert_eq!(scale.major_step, 10.0);
        // A fixed 0..12345: step 1763.6 -> 2000; magnitude 4, a multiple of 3 -> 3; labels in
        // thousands, no decimals.
        let scale = Scale::pick(0.0, 0.0, Some((0.0, 12345.0)));
        assert_eq!(
            (scale.min, scale.max, scale.major_step),
            (0.0, 12345.0, 2000.0)
        );
        assert_eq!(scale.mag, 3);
        let labels: Vec<String> = scale.tics().iter().map(|tic| scale.label(*tic)).collect();
        assert_eq!(labels, ["0", "2", "4", "6", "8", "10", "12"]);
        assert_eq!(scale.title("Distance (m)"), "Distance (m) (10^3)");
        // A step of 0.5 is written to one decimal.
        let scale = Scale::pick(0.0, 0.0, Some((0.0, 3.0)));
        assert_eq!(scale.major_step, 0.5);
        assert_eq!(scale.label(1.5), "1.5");
    }

    /// `Convert.ToInt32`: a half goes to the even neighbour.
    #[test]
    fn convert_to_int32_rounds_a_half_to_even() {
        assert_eq!(elevation::convert_to_int32(600.5), 600.0);
        assert_eq!(elevation::convert_to_int32(601.5), 602.0);
        assert_eq!(elevation::convert_to_int32(-2.5), -2.0);
        assert_eq!(elevation::convert_to_int32(634.05), 634.0);
    }

    /// The facts say closed while no form shows.
    #[test]
    fn with_no_form_the_facts_say_closed() {
        assert_eq!(
            elevation::facts(None),
            vec![("plan.elevation", "closed".to_owned())]
        );
    }
}
#[cfg(test)]
mod geofence_tests {
    use super::*;
    use mp_link::requests::{FencePointRead, FencePointSet, RequestOutcome};

    fn at(lat: f64, lng: f64) -> LatLon {
        LatLon::new(lat, lng).expect("a position")
    }

    /// Three corners about SITL's home, and a return location inside them.
    fn triangle() -> Vec<LatLon> {
        vec![
            at(-35.364, 149.164),
            at(-35.364, 149.168),
            at(-35.361, 149.166),
        ]
    }

    fn inside() -> LatLon {
        at(-35.3632, 149.1660)
    }

    fn params(list: &[(&str, f64)]) -> Vec<(String, f64)> {
        list.iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// ArduCopter's: `FENCE_ENABLE`, `FENCE_ACTION` 1, `FENCE_TOTAL`, no altitude box.
    fn copter() -> Vec<(String, f64)> {
        params(&[
            ("FENCE_ENABLE", 0.0),
            ("FENCE_ACTION", 1.0),
            ("FENCE_TOTAL", 0.0),
        ])
    }

    fn drawn(polygon: &[LatLon], return_point: Option<LatLon>) -> Plan {
        let mut plan = Plan::default();
        for corner in polygon {
            plan.add_area_vertex(*corner);
        }
        if let Some(at) = return_point {
            plan.set_fence_return(at);
        }
        plan
    }

    fn context() -> MenuContext {
        MenuContext {
            frame: AltitudeFrame::Relative,
            vehicle: None,
            takeoff_pitch: false,
            copter: false,
            tracker_alt: 0.0,
        }
    }

    fn answer(plan: &mut Plan, menus: &mut PlanMenus, value: &str) {
        menus
            .prompt
            .as_mut()
            .and_then(|prompt| prompt.field.as_mut())
            .expect("a prompt with a field")
            .set(value);
        menus.submit(plan, &context());
    }

    /// Every call an upload makes, answered `outcome` each, until it ends.
    fn run(upload: &mut FenceUpload, mut outcome: impl FnMut(&FenceDue) -> RequestOutcome) {
        let mut calls = 0;
        while let Some(due) = upload.due() {
            upload.answer(Some(outcome(&due)));
            calls += 1;
            assert!(calls < 100, "the upload never ended");
        }
    }

    const ACCEPTED: RequestOutcome = RequestOutcome::Accepted { value: None };

    /// `pnpoly`: the even-odd rule over latitude and longitude, whether or not the polygon is
    /// closed by its first corner again.
    #[test]
    fn pnpoly_is_the_even_odd_rule() {
        let square = [
            at(-35.0, 149.0),
            at(-35.0, 150.0),
            at(-34.0, 150.0),
            at(-34.0, 149.0),
        ];
        let mut closed = square.to_vec();
        closed.push(square[0]);
        for polygon in [&square[..], &closed[..]] {
            assert!(pnpoly(polygon, -34.5, 149.5));
            assert!(!pnpoly(polygon, -33.5, 149.5));
            assert!(!pnpoly(polygon, -34.5, 150.5));
        }
        assert!(!pnpoly(&[], -34.5, 149.5));
        let triangle = triangle();
        assert!(pnpoly(&triangle, inside().latitude(), inside().longitude()));
    }

    /// The handler's checks in its order: the fence parameters, a return location, a drawn
    /// polygon, the return location inside it.
    #[test]
    fn upload_refuses_in_the_c_sharp_order() {
        let none = params(&[]);
        let plan = drawn(&[], None);
        assert_eq!(
            fence_upload_checks(&plan, &none).err(),
            Some(FENCE_NOT_SUPPORTED)
        );
        assert_eq!(
            fence_upload_checks(&plan, &copter()).err(),
            Some(NO_RETURN_LOCATION)
        );
        let plan = drawn(&[], Some(inside()));
        assert_eq!(
            fence_upload_checks(&plan, &copter()).err(),
            Some(NO_POLYGON_DRAWN)
        );
        let plan = drawn(&triangle(), Some(at(-35.30, 149.20)));
        assert_eq!(
            fence_upload_checks(&plan, &copter()).err(),
            Some(RETURN_OUTSIDE)
        );
        let plan = drawn(&triangle(), Some(inside()));
        let ask = fence_upload_checks(&plan, &copter()).expect("an upload");
        // ArduCopter lists neither altitude: nothing is asked.
        assert_eq!(ask.next_question(), None);
        // FENCE_ACTION alone (ArduPlane's) is enough too; the corners kept off the map by
        // Geo-Fence > Clear are still `drawnpolygon.Points`.
        let mut plan = drawn(&triangle(), Some(inside()));
        plan.hide_polygon();
        assert!(fence_upload_checks(&plan, &params(&[("FENCE_ACTION", 0.0)])).is_ok());
    }

    /// The sequence the C# makes: `FENCE_ACTION` 0, `FENCE_TOTAL` the corners plus two, the return
    /// location, each corner, the first corner again - each with the count, and the progress
    /// reporter's words - and `FENCE_ACTION` back; then done, the drawn polygon the geofence.
    #[test]
    fn upload_sets_the_parameters_then_every_point_then_the_action_back() {
        let plan = drawn(&triangle(), Some(inside()));
        let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
        assert_eq!(upload.status(), SENDING_FENCE_POINTS);
        let mut seen = Vec::new();
        let mut statuses = Vec::new();
        while let Some(due) = upload.due() {
            if matches!(due, FenceDue::Point(_)) {
                statuses.push(upload.status());
            }
            seen.push(due);
            upload.answer(Some(ACCEPTED));
        }
        let corners = triangle();
        let point = |idx: u8, at: LatLon| {
            FenceDue::Point(FencePointSet {
                idx,
                count: 5,
                lat: at.latitude(),
                lng: at.longitude(),
            })
        };
        assert_eq!(
            seen,
            vec![
                FenceDue::Param("FENCE_ACTION", 0.0),
                FenceDue::Param("FENCE_TOTAL", 5.0),
                point(0, inside()),
                point(1, corners[0]),
                point(2, corners[1]),
                point(3, corners[2]),
                point(4, corners[0]),
                FenceDue::Param("FENCE_ACTION", 1.0),
            ]
        );
        assert_eq!(
            statuses,
            [
                SENDING_RETURN,
                SENDING_POLYGON,
                SENDING_POLYGON,
                SENDING_POLYGON,
                SENDING_CLOSE
            ]
        );
        assert_eq!(upload.end(), Some(&FenceUploadEnd::Done { error: None }));
        assert_eq!(
            upload.results(),
            "FENCE_ACTION=0,FENCE_TOTAL=5,0=set,1=set,2=set,3=set,4=set,FENCE_ACTION=1"
        );
    }

    /// A point that times out, or never reads back the same, ends the points - the exception
    /// leaves `DoGeofencePointsUpload` - and the upload goes on to put `FENCE_ACTION` back and
    /// redraw, the failure said.
    #[test]
    fn a_failed_point_ends_the_points_and_the_action_is_still_put_back() {
        let plan = drawn(&triangle(), Some(inside()));
        for (answer, said, word) in [
            (RequestOutcome::TimedOut, FENCE_POINT_TIMEOUT, "timeout"),
            (RequestOutcome::Sent, FENCE_POINT_UNVERIFIED, "unverified"),
        ] {
            let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
            run(&mut upload, |due| match due {
                FenceDue::Point(set) if set.idx == 1 => answer,
                _ => ACCEPTED,
            });
            assert_eq!(
                upload.end(),
                Some(&FenceUploadEnd::Done { error: Some(said) })
            );
            assert_eq!(
                upload.results(),
                format!("FENCE_ACTION=0,FENCE_TOTAL=5,0=set,1={word},FENCE_ACTION=1")
            );
        }
    }

    /// A set that goes unanswered stops the upload with the handler's words, nothing redrawn; one
    /// the vehicle does not list is passed over, as `setParam` returns false for it.
    #[test]
    fn an_unanswered_set_stops_the_upload_with_the_handlers_words() {
        let plan = drawn(&triangle(), Some(inside()));
        for (name, said) in [
            ("FENCE_ACTION", FENCE_ACTION_FAILED),
            ("FENCE_TOTAL", FENCE_TOTAL_FAILED),
        ] {
            let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
            run(&mut upload, |due| match due {
                FenceDue::Param(set, _) if *set == name => RequestOutcome::TimedOut,
                _ => ACCEPTED,
            });
            assert_eq!(upload.end(), Some(&FenceUploadEnd::Stopped(said)));
        }
        // The restore: every point sent, then "Failed to restore FENCE_ACTION".
        let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
        let mut restoring = false;
        run(&mut upload, |due| match due {
            FenceDue::Param("FENCE_ACTION", value) if *value == 1.0 => {
                restoring = true;
                RequestOutcome::TimedOut
            }
            _ => ACCEPTED,
        });
        assert!(restoring);
        assert_eq!(
            upload.end(),
            Some(&FenceUploadEnd::Stopped(FENCE_RESTORE_FAILED))
        );
        // FENCE_TOTAL not listed: passed over.
        let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
        run(&mut upload, |due| match due {
            FenceDue::Param("FENCE_TOTAL", _) => RequestOutcome::UnknownParameter,
            _ => ACCEPTED,
        });
        assert_eq!(upload.end(), Some(&FenceUploadEnd::Done { error: None }));
        assert!(upload.results().contains("FENCE_TOTAL=unknown,0=set"));
    }

    /// A vehicle with `FENCE_ENABLE` and no `FENCE_ACTION`: `(float) MAV.param["FENCE_ACTION"]`
    /// throws before anything is sent.
    #[test]
    fn no_fence_action_is_the_null_reference_before_any_send() {
        let plan = drawn(&triangle(), Some(inside()));
        let ask = fence_upload_checks(&plan, &params(&[("FENCE_ENABLE", 1.0)])).expect("ok");
        let mut upload = FenceUpload::new(ask);
        assert_eq!(upload.due(), None);
        assert_eq!(upload.end(), Some(&FenceUploadEnd::Stopped(NULL_REFERENCE)));
    }

    /// ArduPlane's `FENCE_MINALT` and `FENCE_MAXALT`: "Min Alt" and "Max Alt" asked in turn, each
    /// offering the parameter; a word that is not a whole number is "Bad Min Alt"; Cancel stops
    /// the upload; the answers are set before `FENCE_ACTION`.
    #[test]
    fn a_plane_is_asked_its_box_altitudes_and_they_are_set_first() {
        let plane = params(&[
            ("FENCE_ACTION", 1.0),
            ("FENCE_TOTAL", 0.0),
            ("FENCE_MINALT", 10.0),
            ("FENCE_MAXALT", 100.0),
        ]);
        let mut plan = drawn(&triangle(), Some(inside()));
        let mut menus = PlanMenus::default();
        plan.fence_ask = Some(fence_upload_checks(&plan, &plane).expect("ok"));
        menus.continue_fence_upload(&mut plan);
        let prompt = menus.prompt.as_ref().expect("Min Alt");
        assert_eq!(
            (prompt.title, prompt.text.as_str()),
            ("Min Alt", "Box Minimum Altitude?")
        );
        assert_eq!(prompt.value(), "10");
        answer(&mut plan, &mut menus, "15");
        let prompt = menus.prompt.as_ref().expect("Max Alt");
        assert_eq!((prompt.title, prompt.value()), ("Max Alt", "100"));
        answer(&mut plan, &mut menus, "120");
        assert!(menus.prompt.is_none());
        assert!(plan.fence_ask.is_none());
        let upload = plan.fence_upload.as_mut().expect("the upload");
        assert_eq!(upload.due(), Some(FenceDue::Param("FENCE_MINALT", 15.0)));
        upload.answer(Some(ACCEPTED));
        assert_eq!(upload.due(), Some(FenceDue::Param("FENCE_MAXALT", 120.0)));
        upload.answer(Some(ACCEPTED));
        assert_eq!(upload.due(), Some(FenceDue::Param("FENCE_ACTION", 0.0)));

        // "Bad Min Alt": said, and nothing sent.
        let mut plan = drawn(&triangle(), Some(inside()));
        let mut menus = PlanMenus::default();
        plan.fence_ask = Some(fence_upload_checks(&plan, &plane).expect("ok"));
        menus.continue_fence_upload(&mut plan);
        answer(&mut plan, &mut menus, "ten");
        assert!(menus.prompt.is_none());
        assert!(!plan.fence_busy());
        assert_eq!(plan.fence_upload_text(), BAD_MIN_ALT);
        assert_eq!(plan.fence_say.as_deref(), Some(BAD_MIN_ALT));

        // Cancel on the second box: `return`, nothing sent.
        let mut plan = drawn(&triangle(), Some(inside()));
        let mut menus = PlanMenus::default();
        plan.fence_ask = Some(fence_upload_checks(&plan, &plane).expect("ok"));
        menus.continue_fence_upload(&mut plan);
        answer(&mut plan, &mut menus, "15");
        let _ = menus.cancel(&mut plan);
        assert!(!plan.fence_busy());
    }

    /// A parameter holding a fraction cannot be offered: `int.Parse("10.5")` throws.
    #[test]
    fn a_fractional_altitude_parameter_is_the_format_exception() {
        let plan = drawn(&triangle(), Some(inside()));
        let ask = fence_upload_checks(
            &plan,
            &params(&[("FENCE_ACTION", 1.0), ("FENCE_MINALT", 10.5)]),
        )
        .expect("ok");
        assert_eq!(ask.next_question(), Some(Err(FORMAT_EXCEPTION)));
    }

    /// The end of the upload: the drawn polygon is the geofence and is gone from the map, the
    /// return marker stays.
    #[test]
    fn an_uploaded_polygon_becomes_the_geofence() {
        let mut plan = drawn(&triangle(), Some(inside()));
        plan.fence_uploaded(triangle());
        assert_eq!(plan.fence(), triangle().as_slice());
        assert!(plan.polygon().is_empty());
        assert_eq!(plan.fence_return(), Some(inside()));
    }

    fn read(lat: f32, lng: f32, count: u8) -> Option<FencePointRead> {
        Some(FencePointRead { lat, lng, count })
    }

    /// Download point by point: the first read's count says how many; the first point is the
    /// return location and the rest - the closing point with them - the geofence. A point that
    /// does not come is "Failed to get fence point".
    #[test]
    fn download_reads_while_below_the_count_the_points_give() {
        let mut download = FenceDownload::new();
        let points = [
            (-35.3632_f32, 149.166_f32),
            (-35.364, 149.164),
            (-35.364, 149.168),
            (-35.361, 149.166),
            (-35.364, 149.164),
        ];
        for (index, (lat, lng)) in points.iter().enumerate() {
            assert_eq!(download.due(), Some(u8::try_from(index).expect("few")));
            download.answer(Some(ACCEPTED), read(*lat, *lng, 5));
        }
        assert_eq!(download.due(), None);
        assert_eq!(download.end(), Some(Ok(())));
        let mut plan = Plan::default();
        plan.fence_downloaded(download.points());
        assert_eq!(plan.fence().len(), 4);
        let back = plan.fence_return().expect("the return");
        assert!((back.latitude() - -35.3632).abs() < 1e-5);

        let mut download = FenceDownload::new();
        let _ = download.due();
        download.answer(Some(RequestOutcome::TimedOut), None);
        assert_eq!(download.end(), Some(Err(FENCE_POINT_FAILED)));
    }

    /// The facts' words: the progress reporter's text while an upload runs, "busy" while a
    /// download does, and what each last said.
    #[test]
    fn the_facts_say_what_each_last_said() {
        let mut plan = drawn(&triangle(), Some(inside()));
        assert_eq!(plan.fence_upload_text(), "none");
        assert_eq!(plan.fence_download_text(), "none");
        plan.fence_upload = Some(FenceUpload::new(
            fence_upload_checks(&plan, &copter()).expect("ok"),
        ));
        assert_eq!(plan.fence_upload_text(), SENDING_FENCE_POINTS);
        plan.fence_upload = None;
        plan.fence_upload_said("done");
        assert_eq!(plan.fence_upload_text(), "done");
        plan.fence_download = Some(FenceDownloading::Mission);
        assert_eq!(plan.fence_download_text(), "busy");
        plan.fence_download = None;
        plan.fence_download_said(NOTHING_TO_DOWNLOAD);
        assert_eq!(plan.fence_download_text(), NOTHING_TO_DOWNLOAD);
    }

    /// Both entries do something now, where they were drawn dimmed.
    #[test]
    fn the_geofence_upload_and_download_entries_are_wired() {
        let geofence = MAP_MENU
            .iter()
            .find(|entry| entry.id == "menu-geoFence")
            .expect("Geo-Fence");
        let action = |id: &str| {
            geofence
                .children
                .iter()
                .find(|entry| entry.id == id)
                .and_then(|entry| entry.action)
        };
        assert_eq!(
            action("menu-GeoFenceupload"),
            Some(MenuAction::GeoFenceUpload)
        );
        assert_eq!(
            action("menu-GeoFencedownload"),
            Some(MenuAction::GeoFenceDownload)
        );
    }

    /// `MAV_PROTOCOL_CAPABILITY_MISSION_FENCE` is 16384, the dialect's and the C#'s
    /// (`Mavlink.cs:7103`), and the SITL's 0xfbef has it; 16 is `PARAM_ENCODE_BYTEWISE`.
    #[test]
    fn the_mission_fence_bit_is_16384_and_the_sitl_has_it() {
        assert_eq!(CAPABILITY_MISSION_FENCE, 16384);
        assert!(has_mission_fence(0xfbef));
        assert!(!has_mission_fence(0));
        assert!(!has_mission_fence(16));
    }

    /// `contextMenuStrip1_Opening`: over a vehicle with `MISSION_FENCE` the menu has no Geo-Fence
    /// and no Rally Points, two rows shorter, and the entries below them move up; over one
    /// without, or with nothing connected, both show.
    #[test]
    fn a_mission_fence_vehicle_hides_geofence_and_rally_points() {
        let mut menus = PlanMenus::default();
        menus.open_for((100.0, 100.0), inside(), None, 0xfbef);
        let menu = menus.open.expect("open");
        assert!(menu.mission_fence);
        let entry = |id: &str| MAP_MENU.iter().position(|e| e.id == id).expect(id);
        for id in HIDDEN_ON_MISSION_FENCE {
            assert!(!shown(&MAP_MENU[entry(id)], true), "{id} hidden");
            assert!(shown(&MAP_MENU[entry(id)], false), "{id} shown");
        }
        assert_eq!(MAP_MENU.iter().filter(|e| !shown(e, true)).count(), 2);
        assert!(
            (map_menu_height(false) - map_menu_height(true) - 2.0 * MENU_ROW).abs() < f32::EPSILON
        );
        // An entry below both moves up two rows; one above neither does not move.
        let below = entry("menu-rallyPoints").max(entry("menu-geoFence")) + 1;
        let above = entry("menu-rallyPoints").min(entry("menu-geoFence")) - 1;
        assert!(
            (menu_offset(below, false) - menu_offset(below, true) - 2.0 * MENU_ROW).abs()
                < f32::EPSILON
        );
        assert!((menu_offset(above, false) - menu_offset(above, true)).abs() < f32::EPSILON);
        menus.open_for((100.0, 100.0), inside(), None, 0);
        assert!(!menus.open.expect("open").mission_fence);
        menus.open_at((100.0, 100.0), inside(), None);
        assert!(!menus.open.expect("open").mission_fence);
    }

    /// A set whose request the link cannot carry - the link gone - stops the upload as its
    /// timeout does, where a parameter the vehicle does not list is passed over.
    #[test]
    fn a_set_the_link_cannot_carry_stops_as_a_timeout() {
        let plan = drawn(&triangle(), Some(inside()));
        let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
        assert!(matches!(
            upload.due(),
            Some(FenceDue::Param("FENCE_ACTION", _))
        ));
        upload.answer(Some(ACCEPTED));
        assert!(matches!(
            upload.due(),
            Some(FenceDue::Param("FENCE_TOTAL", _))
        ));
        upload.answer(None);
        assert_eq!(
            upload.end(),
            Some(&FenceUploadEnd::Stopped(FENCE_TOTAL_FAILED))
        );
        assert_eq!(upload.results(), "FENCE_ACTION=0,FENCE_TOTAL=timeout");
        assert_eq!(upload.due(), None);
    }

    /// Upload, Download and Clear wait for one another, and for the draw panel's fence read, as
    /// the C#'s modal calls keep them apart; and a Min/Max Alt box another box replaces is its
    /// Cancel.
    #[test]
    fn upload_download_and_clear_wait_for_one_another() {
        let telemetry = crate::telemetry::Telemetry::idle();
        let mut menus = PlanMenus::default();
        let mut plan = drawn(&triangle(), Some(inside()));
        begin_fence_clear(&mut plan);
        assert!(plan.writes.is_some());
        begin_fence_upload(&mut plan, &mut menus, &copter(), false);
        assert!(!plan.fence_busy());
        assert_eq!(plan.fence_upload_text(), "none");
        assert!(!begin_fence_download(
            &mut plan,
            &telemetry,
            true,
            0,
            &copter(),
            false
        ));
        assert!(plan.fence_download.is_none());

        let mut plan = drawn(&triangle(), Some(inside()));
        plan.fence_download = Some(FenceDownloading::Points(FenceDownload::new()));
        begin_fence_clear(&mut plan);
        assert!(plan.writes.is_none());

        let mut plan = drawn(&triangle(), Some(inside()));
        begin_fence_upload(&mut plan, &mut menus, &copter(), true);
        assert!(!plan.fence_busy());
        let with_total = params(&[("FENCE_ACTION", 1.0), ("FENCE_TOTAL", 5.0)]);
        assert!(!begin_fence_download(
            &mut plan,
            &telemetry,
            true,
            0,
            &with_total,
            true
        ));
        assert!(plan.fence_download.is_none());

        // A plane's Min Alt box, then another flow's message over it.
        let plane = params(&[
            ("FENCE_ACTION", 1.0),
            ("FENCE_TOTAL", 0.0),
            ("FENCE_MINALT", 10.0),
            ("FENCE_MAXALT", 100.0),
        ]);
        let mut plan = drawn(&triangle(), Some(inside()));
        begin_fence_upload(&mut plan, &mut menus, &plane, false);
        assert!(plan.fence_busy());
        let mut status = None;
        let _ = fence_frame(&mut plan, true, &telemetry, false, &mut status);
        assert!(plan.fence_busy(), "the box still up");
        menus.say("", "another flow's message");
        let _ = fence_frame(&mut plan, false, &telemetry, false, &mut status);
        assert!(!plan.fence_busy());
    }

    /// Download on a vehicle without `MISSION_FENCE`: "Not Supported" without both parameters,
    /// "Nothing to download" for a `FENCE_TOTAL` of 1, and nothing sent for either; with
    /// `MISSION_FENCE` and no link, "Please connect first".
    #[test]
    fn download_refuses_as_the_c_sharp_does() {
        let telemetry = crate::telemetry::Telemetry::idle();
        let mut plan = Plan::default();
        let no_action = params(&[("FENCE_ENABLE", 0.0), ("FENCE_TOTAL", 5.0)]);
        assert!(!begin_fence_download(
            &mut plan, &telemetry, true, 0, &no_action, false
        ));
        assert_eq!(plan.fence_download_text(), FENCE_NOT_SUPPORTED);
        let one = params(&[("FENCE_ACTION", 1.0), ("FENCE_TOTAL", 1.0)]);
        assert!(!begin_fence_download(
            &mut plan, &telemetry, true, 0, &one, false
        ));
        assert_eq!(plan.fence_download_text(), NOTHING_TO_DOWNLOAD);
        assert!(plan.fence_download.is_none());
        assert!(!begin_fence_download(
            &mut plan,
            &telemetry,
            false,
            0x4000,
            &[],
            false
        ));
        assert_eq!(plan.fence_download_text(), PLEASE_CONNECT);
        assert!(plan.fence_download.is_none());
    }

    /// A scripted vehicle that answers the parameters and the legacy fence protocol, and the
    /// mission protocol's fence list when it holds one.
    struct FenceVehicle {
        vehicle: crate::telemetry::scripted::Vehicle,
        points: Vec<mp_mavlink_dialects::all::FencePoint>,
        list: Option<Vec<(i32, i32)>>,
        /// Whether it keeps the points it is sent, so a fetch reads them back.
        keeps: bool,
    }

    impl FenceVehicle {
        fn serve(&mut self) {
            use crate::telemetry::scripted::{INT32, param};
            use mp_mavlink_dialects::all::{FencePoint, MavMessage, MissionCount, MissionItemInt};
            let fence = mp_mission::fence::MISSION_TYPE_FENCE;
            for message in self.vehicle.read() {
                match message {
                    MavMessage::ParamSet(set) => {
                        let name = mp_params::decode_param_id(&set.param_id);
                        self.vehicle.send(&param(&name, set.param_value, INT32));
                    }
                    MavMessage::FencePoint(point) if self.keeps => {
                        self.points.retain(|kept| kept.idx != point.idx);
                        self.points.push(point);
                    }
                    MavMessage::FenceFetchPoint(fetch) => {
                        if let Some(point) = self.points.iter().find(|kept| kept.idx == fetch.idx) {
                            let point = FencePoint {
                                target_system: 255,
                                target_component: 190,
                                ..*point
                            };
                            self.vehicle.send(&MavMessage::FencePoint(point));
                        }
                    }
                    MavMessage::MissionRequestList(request) if request.mission_type == fence => {
                        if let Some(list) = &self.list {
                            self.vehicle.send(&MavMessage::MissionCount(MissionCount {
                                count: u16::try_from(list.len()).expect("few"),
                                target_system: 255,
                                target_component: 190,
                                mission_type: fence,
                            }));
                        }
                    }
                    MavMessage::MissionRequestInt(request) if request.mission_type == fence => {
                        let item = self
                            .list
                            .as_ref()
                            .and_then(|list| list.get(usize::from(request.seq)).copied());
                        if let Some((x, y)) = item {
                            self.vehicle
                                .send(&MavMessage::MissionItemInt(MissionItemInt {
                                    param1: 3.0,
                                    param2: 0.0,
                                    param3: 0.0,
                                    param4: 0.0,
                                    x,
                                    y,
                                    z: 0.0,
                                    seq: request.seq,
                                    command: 5001,
                                    target_system: 255,
                                    target_component: 190,
                                    frame: 3,
                                    current: 0,
                                    autocontinue: 1,
                                    mission_type: fence,
                                }));
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    /// The link to a [`FenceVehicle`] that lists `FENCE_ACTION` and `FENCE_TOTAL`.
    fn fence_vehicle(
        list: Option<Vec<(i32, i32)>>,
        keeps: bool,
    ) -> (crate::telemetry::Telemetry, FenceVehicle) {
        use crate::telemetry::scripted::{INT32, Vehicle, param, until};
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("FENCE_ACTION", 1.0, INT32));
        vehicle.send(&param("FENCE_TOTAL", 0.0, INT32));
        until("the fence parameters listed", || {
            telemetry.holds_parameter("FENCE_ACTION") && telemetry.holds_parameter("FENCE_TOTAL")
        });
        let vehicle = FenceVehicle {
            vehicle,
            points: Vec::new(),
            list,
            keeps,
        };
        (telemetry, vehicle)
    }

    /// Frames of [`fence_frame`], the vehicle answering between them, until neither flow runs:
    /// the status line as each frame left it, and what the last frame changed.
    fn frames(
        plan: &mut Plan,
        telemetry: &crate::telemetry::Telemetry,
        vehicle: &mut FenceVehicle,
    ) -> (Vec<Option<String>>, FenceChanged) {
        let mut status = None;
        let mut seen = Vec::new();
        let mut changed = FenceChanged::default();
        crate::telemetry::scripted::until("the fence flows to end", || {
            vehicle.serve();
            changed = fence_frame(plan, false, telemetry, true, &mut status);
            seen.push(status.clone());
            plan.fence_upload.is_none() && plan.fence_download.is_none()
        });
        (seen, changed)
    }

    /// Through the real link, the menu's way: Upload shows the progress window's words while
    /// points go and nothing before or after; Download point by point then makes the geofence and
    /// its return location, the map told, and says nothing on the status line.
    #[test]
    fn the_menus_upload_and_download_run_through_the_link_and_say_only_what_the_c_sharp_shows() {
        let (telemetry, mut vehicle) = fence_vehicle(None, true);
        let mut menus = PlanMenus::default();
        let mut plan = drawn(&triangle(), Some(inside()));
        begin_fence_upload(&mut plan, &mut menus, &copter(), false);
        let (seen, changed) = frames(&mut plan, &telemetry, &mut vehicle);
        assert!(changed.fence && changed.polygon);
        assert_eq!(plan.fence_upload_text(), "done");
        assert_eq!(seen.last(), Some(&None), "nothing left on the status line");
        let shown: Vec<&str> = seen.iter().flatten().map(String::as_str).collect();
        assert!(shown.contains(&SENDING_RETURN), "{shown:?}");
        assert!(!shown.contains(&SENDING_FENCE_POINTS), "{shown:?}");
        assert!(!shown.contains(&"done"), "{shown:?}");
        assert_eq!(plan.fence().len(), 3);

        // The vehicle now holds five points, as FENCE_TOTAL says.
        let with_total = params(&[("FENCE_ACTION", 1.0), ("FENCE_TOTAL", 5.0)]);
        let mut back = Plan::default();
        assert!(begin_fence_download(
            &mut back,
            &telemetry,
            true,
            0,
            &with_total,
            false
        ));
        assert!(back.fence().is_empty() && back.fence_return().is_none());
        let (seen, changed) = frames(&mut back, &telemetry, &mut vehicle);
        assert!(
            changed.fence,
            "the map shows the geofence and its return location"
        );
        assert_eq!(back.fence().len(), 4, "the closing point with them");
        let returned = back.fence_return().expect("the return location");
        assert!((returned.latitude() - inside().latitude()).abs() < 1e-5);
        assert_eq!(back.fence_download_text(), "5");
        assert!(seen.iter().all(Option::is_none), "{seen:?}");
    }

    /// A point that is never read back: the progress reporter's "There was an unexpected error
    /// (...)" on the status line, `FENCE_ACTION` put back all the same.
    #[test]
    fn a_point_never_read_back_is_the_progress_reporters_error() {
        let (telemetry, mut vehicle) = fence_vehicle(None, false);
        let mut menus = PlanMenus::default();
        let mut plan = drawn(&triangle(), Some(inside()));
        begin_fence_upload(&mut plan, &mut menus, &copter(), false);
        let (seen, changed) = frames(&mut plan, &telemetry, &mut vehicle);
        assert!(changed.fence, "the C# redraws after the progress reporter");
        let said = unexpected_error(FENCE_POINT_TIMEOUT);
        assert_eq!(
            said,
            "There was an unexpected error (Timeout on read - getFencePoint)"
        );
        assert_eq!(plan.fence_upload_text(), said);
        assert_eq!(seen.last(), Some(&Some(said)));
        assert!(plan.fence_upload_results.ends_with(",FENCE_ACTION=1"));
    }

    /// Download on a vehicle with `MISSION_FENCE`: the fence list through the mission protocol,
    /// thrown away as the C# throws it - the geofence as it was, nothing said - its items in the
    /// link's fence points; "Failed to get fence point" when the list does not come or the link
    /// goes.
    #[test]
    fn a_mission_fence_download_leaves_the_geofence_and_fails_when_the_link_goes() {
        let corners = vec![
            (-353_600_000, 1_491_600_000),
            (-353_700_000, 1_491_600_000),
            (-353_650_000, 1_491_700_000),
        ];
        let (telemetry, mut vehicle) = fence_vehicle(Some(corners), true);
        let mut plan = Plan {
            fence: triangle(),
            fence_return: Some(inside()),
            ..Plan::default()
        };
        assert!(!begin_fence_download(
            &mut plan,
            &telemetry,
            true,
            0xfbef,
            &[],
            false
        ));
        assert!(matches!(
            plan.fence_download,
            Some(FenceDownloading::Mission)
        ));
        let (seen, changed) = frames(&mut plan, &telemetry, &mut vehicle);
        assert_eq!(changed, FenceChanged::default());
        assert_eq!(plan.fence(), &triangle()[..]);
        assert_eq!(plan.fence_return(), Some(inside()));
        assert_eq!(plan.fence_download_text(), "3");
        assert!(seen.iter().all(Option::is_none), "{seen:?}");
        assert_eq!(telemetry.fence_points().len(), 3);

        // No answer: the link's list request gives up, and so does Download.
        let (telemetry, mut vehicle) = fence_vehicle(None, true);
        let mut plan = Plan::default();
        assert!(!begin_fence_download(
            &mut plan,
            &telemetry,
            true,
            0xfbef,
            &[],
            false
        ));
        let (seen, _) = frames(&mut plan, &telemetry, &mut vehicle);
        assert_eq!(plan.fence_download_text(), FENCE_POINT_FAILED);
        assert_eq!(seen.last(), Some(&Some(FENCE_POINT_FAILED.to_owned())));

        // Disconnect mid-download: the window's telemetry idle and the link down.
        let (telemetry, _vehicle) = fence_vehicle(None, true);
        let mut plan = Plan::default();
        assert!(!begin_fence_download(
            &mut plan,
            &telemetry,
            true,
            0xfbef,
            &[],
            false
        ));
        let idle = crate::telemetry::Telemetry::idle();
        let mut status = None;
        let _ = fence_frame(&mut plan, false, &idle, false, &mut status);
        assert!(plan.fence_download.is_none() && !plan.fence_busy());
        assert_eq!(status.as_deref(), Some(FENCE_POINT_FAILED));
    }

    /// The waits the scripted vehicle is driven with: the C#'s counts, a twentieth of its waits.
    fn fast() -> mp_link::ProtocolTimeouts {
        mp_link::ProtocolTimeouts::default().faster(20)
    }

    /// Upload and then Download through the real link, to a scripted ArduPilot that answers the
    /// legacy protocol: `PARAM_SET` `FENCE_ACTION` and `FENCE_TOTAL`, each `FENCE_POINT` and the
    /// `FENCE_FETCH_POINT` that reads it back, `FENCE_ACTION` again - in that order on the wire -
    /// and the points read back one by one make the geofence and its return location.
    #[test]
    fn upload_and_download_go_through_the_link_as_the_c_sharp_sends_them() {
        use crate::telemetry::scripted::{INT32, Vehicle, param, until};
        use mp_mavlink_dialects::all::{FencePoint, MavMessage};

        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("FENCE_ACTION", 1.0, INT32));
        vehicle.send(&param("FENCE_TOTAL", 0.0, INT32));
        until("the fence parameters listed", || {
            telemetry.holds_parameter("FENCE_ACTION") && telemetry.holds_parameter("FENCE_TOTAL")
        });
        let mut held: Vec<FencePoint> = Vec::new();
        let mut serve = |vehicle: &mut Vehicle| {
            for message in vehicle.read() {
                match message {
                    MavMessage::ParamSet(set) => {
                        let name = mp_params::decode_param_id(&set.param_id);
                        vehicle.send(&param(&name, set.param_value, INT32));
                    }
                    MavMessage::FencePoint(point) => {
                        held.retain(|kept| kept.idx != point.idx);
                        held.push(point);
                    }
                    MavMessage::FenceFetchPoint(fetch) => {
                        if let Some(point) = held.iter().find(|kept| kept.idx == fetch.idx) {
                            vehicle.send(&MavMessage::FencePoint(FencePoint {
                                target_system: 255,
                                target_component: 190,
                                ..*point
                            }));
                        }
                    }
                    _ => {}
                }
            }
        };

        let plan = drawn(&triangle(), Some(inside()));
        let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
        until("the upload to end", || {
            serve(&mut vehicle);
            step_fence_upload(&telemetry, &mut upload);
            upload.end().is_some()
        });
        assert_eq!(upload.end(), Some(&FenceUploadEnd::Done { error: None }));
        assert_eq!(
            upload.results(),
            "FENCE_ACTION=0,FENCE_TOTAL=5,0=set,1=set,2=set,3=set,4=set,FENCE_ACTION=1"
        );
        let order: Vec<String> = vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::ParamSet(set) => Some(format!(
                    "{}={}",
                    mp_params::decode_param_id(&set.param_id),
                    set.param_value
                )),
                MavMessage::FencePoint(point) => {
                    Some(format!("point{}/{}", point.idx, point.count))
                }
                MavMessage::FenceFetchPoint(fetch) => Some(format!("fetch{}", fetch.idx)),
                _ => None,
            })
            .collect();
        assert_eq!(
            order,
            [
                "FENCE_ACTION=0",
                "FENCE_TOTAL=5",
                "point0/5",
                "fetch0",
                "point1/5",
                "fetch1",
                "point2/5",
                "fetch2",
                "point3/5",
                "fetch3",
                "point4/5",
                "fetch4",
                "FENCE_ACTION=1",
            ]
        );

        let mut download = FenceDownload::new();
        until("the download to end", || {
            serve(&mut vehicle);
            step_fence_download(&telemetry, &mut download);
            download.end().is_some()
        });
        assert_eq!(download.end(), Some(Ok(())));
        let mut back = Plan::default();
        back.fence_downloaded(download.points());
        assert_eq!(back.fence().len(), 4);
        for (got, sent) in back
            .fence()
            .iter()
            .zip(triangle().iter().chain(&triangle()[..1]))
        {
            assert!((got.latitude() - sent.latitude()).abs() < 1e-5, "{got:?}");
            assert!((got.longitude() - sent.longitude()).abs() < 1e-5, "{got:?}");
        }
        let home = back.fence_return().expect("the return location");
        assert!((home.latitude() - inside().latitude()).abs() < 1e-5);
    }

    /// A vehicle that never answers `FENCE_FETCH_POINT` - ArduCopter 4.8's SITL, built without the
    /// legacy protocol: the first point's fetch goes four times, "Timeout on read -
    /// getFencePoint", and `FENCE_ACTION` is put back all the same.
    #[test]
    fn a_vehicle_without_the_legacy_protocol_times_out_on_the_return_point() {
        use crate::telemetry::scripted::{INT32, Vehicle, param, until};
        use mp_mavlink_dialects::all::MavMessage;

        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("FENCE_ACTION", 1.0, INT32));
        vehicle.send(&param("FENCE_TOTAL", 0.0, INT32));
        until("the fence parameters listed", || {
            telemetry.holds_parameter("FENCE_ACTION") && telemetry.holds_parameter("FENCE_TOTAL")
        });
        let plan = drawn(&triangle(), Some(inside()));
        let mut upload = FenceUpload::new(fence_upload_checks(&plan, &copter()).expect("ok"));
        until("the upload to end", || {
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    let name = mp_params::decode_param_id(&set.param_id);
                    vehicle.send(&param(&name, set.param_value, INT32));
                }
            }
            step_fence_upload(&telemetry, &mut upload);
            upload.end().is_some()
        });
        assert_eq!(
            upload.end(),
            Some(&FenceUploadEnd::Done {
                error: Some(FENCE_POINT_TIMEOUT)
            })
        );
        assert_eq!(
            upload.results(),
            "FENCE_ACTION=0,FENCE_TOTAL=5,0=timeout,FENCE_ACTION=1"
        );
        let _ = vehicle.read();
        assert_eq!(
            vehicle.count(|message| matches!(message, MavMessage::FenceFetchPoint(_))),
            4
        );

        // And Download: point 0 never comes.
        let mut download = FenceDownload::new();
        until("the download to end", || {
            step_fence_download(&telemetry, &mut download);
            download.end().is_some()
        });
        assert_eq!(download.end(), Some(Err(FENCE_POINT_FAILED)));
    }
}
