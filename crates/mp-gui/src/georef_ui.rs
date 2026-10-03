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

//! Geo Reference Images: the form `GeoRef/georefimage.cs` is, which the DataFlash Logs page's
//! button opens (`new Georefimage().Show()`, `// C#: GCSViews/FlightData.cs:5933-5936`).
//!
//! What the form's handlers do with `GeoRefImageBase` - Estimate Offset, Pre-process, GeoTag
//! Images, what choosing a folder reads, what a change to the log box or to the GPS2 and CAM boxes
//! forgets - is `mp_georef`, held to the C# under mono by `crates/mp-georef/tests/oracle.rs`. This
//! module is the form over it:
//!
//! - **The layout is `georefimage.resx`'s.** Every control at its location and size in the form's
//!   1085 by 595 client area ([`CONTROLS`]), drawn over the flight screen as `Show()` puts a
//!   window of its own over it, with its title and a close box. The three panels have
//!   `BorderStyle.FixedSingle`, so what is in them sits one pixel in from their edge.
//! - **The handlers are the C#'s**, each at its site: the text boxes' `TextChanged`, the check
//!   boxes' `CheckedChanged`, the radio buttons' `ProcessType_CheckedChanged` enabling the two
//!   panels, `num_minshutter`'s `ValueChanged`, and the buttons. Estimate Offset, Pre-process and
//!   GeoTag Images run on a thread of their own, as the DataFlash page's conversions do, the text
//!   box filling as the lines arrive; the C# runs them on the window's thread, so while one runs
//!   the form takes nothing - no click, no key - as the C#'s busy form takes nothing.
//! - **The map is `myGMAP1` as Pre-process leaves it:** the route through `vehicleLocations`, a
//!   camera marker (`GMapMarkerPhoto`) per `camLocations` entry and a green pin
//!   (`GMarkerGoogle`) per matched photo, each marker's tooltip while the pointer is over it and a
//!   camera marker's footprint with it, and the view zoomed to the route (`ZoomAndCenterRoutes`).
//!   The markers are painted over [`crate::mapview`]'s map through its public
//!   [`MapViewport::screen_of`], as the log browser's pin is; the map draws the imagery.
//!
//! Location Kml (`BUT_networklinkgeoref`) opens `m3u/GeoRefnetworklink.kml`, a network link to the
//! built-in web server (`crate::http_server`), which serves the KML `httpGeoRefKML` hands it and
//! the photos from the folder; the file is Mission Planner's, beside the program, or written under
//! the data directory when it is not there.
//!
//! Not ported, each for its reason: the photo `pictureBox1` shows when its pin is clicked (drawn
//! dimmed; the click is taken, and the photo's path is what it would show); and the "Report this
//! Error???" question of the box an uncaught exception shows, which sends a report to Mission
//! Planner's server.
//! `// C#: GeoRef/georefimage.cs, GeoRef/Georefimage.Designer.cs, GeoRef/georefimage.resx`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, mpsc};

use gpui::{
    AnyElement, App, BorderStyle, Bounds, Context, Corners, Div, Edges, FocusHandle, Hsla,
    MouseButton, PathBuilder, Pixels, Point, ScrollHandle, SharedString, TextAlign, TextRun,
    Window, canvas, div, point, prelude::*, px, quad, rgb, size,
};
use mp_georef::georef::{GeorefError, ProcessingMode};
use mp_georef::time::{DateTime, Kind as TimeKind, TICKS_PER_DAY, UNIX_EPOCH_TICKS};
use mp_georef::{FormSettings, GeoRefImageBase, ReportFiles, Terrain};
use mp_mission::dotnet::{Decimal, bool_text};
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::mapview::MapViewport;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// WinForms' 8.25 pt Microsoft Sans Serif is eleven pixels; the `.resx` sizes are laid out for it.
const FONT: f32 = 11.0;

// ---------------------------------------------------------------------------------------------
// The layout.
// ---------------------------------------------------------------------------------------------

/// `$this.ClientSize`. `// C#: GeoRef/georefimage.resx ($this.ClientSize)`
pub const CLIENT_SIZE: (f32, f32) = (1085.0, 595.0);

/// `$this.Text`, the window's title. `// C#: GeoRef/georefimage.resx ($this.Text)`
pub const TITLE: &str = "Geo Ref Images";

/// Where the window is put over the flight screen: `Show()` leaves the place to the system.
const WINDOW_AT: (f32, f32) = (24.0, 64.0);

/// A control's `Location` and `Size`.
pub type Place = (f32, f32, f32, f32);

/// A control of the form: its Designer name, the panel it is in, its place there, and its `Text`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Control {
    /// The Designer's name.
    pub name: &'static str,
    /// The panel it is added to; "" for the form.
    pub parent: &'static str,
    /// `Location` and `Size`.
    pub place: Place,
    /// `Text`, "" for none.
    pub text: &'static str,
}

const fn control(
    name: &'static str,
    parent: &'static str,
    place: Place,
    text: &'static str,
) -> Control {
    Control {
        name,
        parent,
        place,
        text,
    }
}

/// Every control with a place, as the `.resx` places it, in the Designer's order.
/// `// C#: GeoRef/Georefimage.Designer.cs:20-462; GeoRef/georefimage.resx`
#[rustfmt::skip]
pub const CONTROLS: &[Control] = &[
    control("TXT_logfile", "", (12.0, 12.0, 434.0, 20.0), ""),
    control("TXT_jpgdir", "", (12.0, 47.0, 434.0, 20.0), ""),
    control("BUT_browselog", "", (507.0, 7.0, 75.0, 29.0), "Browse Log"),
    control("BUT_browsedir", "", (507.0, 42.0, 75.0, 28.0), "Browse Pictures"),
    control("RDIO_TimeOffset", "", (87.0, 90.0, 77.0, 17.0), "Time offset"),
    control("RDIO_CAMMsgSynchro", "", (261.0, 90.0, 94.0, 17.0), "CAM Message"),
    control("RDIO_trigmsg", "", (487.0, 90.0, 104.0, 17.0), "Trigger Message"),
    control("chk_cammsg", "", (87.0, 108.0, 120.0, 17.0), "Use Cam Messages"),
    control("chk_camusegpsalt", "", (261.0, 108.0, 82.0, 17.0), "Use GPSAlt"),
    control("chk_trigusergpsalt", "", (487.0, 108.0, 82.0, 17.0), "Use GPSAlt"),
    control("PANEL_TIME_OFFSET", "", (16.0, 128.0, 375.0, 55.0), ""),
    control("label1", "PANEL_TIME_OFFSET", (31.0, 24.0, 78.0, 13.0), "Seconds offset"),
    control("TXT_offsetseconds", "PANEL_TIME_OFFSET", (115.0, 21.0, 100.0, 20.0), "0"),
    control("BUT_estoffset", "PANEL_TIME_OFFSET", (240.0, 18.0, 75.0, 23.0), "Estimate Offset"),
    control("PANEL_SHUTTER_LAG", "", (390.0, 128.0, 223.0, 55.0), ""),
    control("label27", "PANEL_SHUTTER_LAG", (40.0, 22.0, 80.0, 13.0), "Shutter lag (ms)"),
    control("TXT_shutterLag", "PANEL_SHUTTER_LAG", (128.0, 19.0, 68.0, 20.0), "0"),
    control("panel3", "", (16.0, 189.0, 597.0, 96.0), ""),
    control("lbldrpstart", "panel3", (31.0, 10.0, 96.0, 13.0), "Drop image at start"),
    control("num_dropfromstart", "panel3", (34.0, 25.0, 53.0, 20.0), ""),
    control("label2", "panel3", (31.0, 48.0, 94.0, 13.0), "Drop image at end"),
    control("num_dropend", "panel3", (34.0, 64.0, 53.0, 20.0), ""),
    control("label3", "panel3", (140.0, 10.0, 107.0, 13.0), "Min Shutter (s) (CAM)"),
    control("num_minshutter", "panel3", (143.0, 25.0, 53.0, 20.0), ""),
    control("CHECK_AMSLAlt_Use", "panel3", (149.0, 63.0, 92.0, 17.0), "Use AMSL Alt"),
    control("label28", "panel3", (266.0, 63.0, 64.0, 13.0), "Rel Alt base"),
    control("txt_basealt", "panel3", (336.0, 61.0, 100.0, 20.0), "0"),
    control("label7", "panel3", (318.0, 10.0, 38.0, 13.0), "Dir fov"),
    control("num_vfov", "panel3", (321.0, 26.0, 42.0, 20.0), ""),
    control("label9", "panel3", (374.0, 9.0, 47.0, 13.0), "Rotation"),
    control("num_camerarotation", "panel3", (377.0, 25.0, 42.0, 20.0), ""),
    control("label8", "panel3", (427.0, 9.0, 51.0, 13.0), "Cross fov"),
    control("num_hfov", "panel3", (430.0, 25.0, 42.0, 20.0), ""),
    control("chk_usegps2", "panel3", (442.0, 63.0, 76.0, 17.0), "Use GPS2"),
    control("BUT_doit", "", (123.0, 305.0, 84.0, 40.0), "Pre-process"),
    control("label11", "", (216.0, 319.0, 26.0, 17.0), ">>"),
    control("BUT_networklinkgeoref", "", (254.0, 305.0, 88.0, 40.0), "Location Kml"),
    control("label12", "", (354.0, 317.0, 26.0, 17.0), ">>"),
    control("BUT_Geotagimages", "", (390.0, 305.0, 83.0, 40.0), "GeoTag Images"),
    control("TXT_outputlog", "", (12.0, 351.0, 601.0, 230.0), ""),
    control("myGMAP1", "", (636.0, 9.0, 437.0, 262.0), ""),
    control("pictureBox1", "", (636.0, 277.0, 437.0, 306.0), ""),
];

/// The panels, each `BorderStyle.FixedSingle`: a one-pixel border their controls sit inside.
/// `// C#: GeoRef/Georefimage.Designer.cs:190, 262, 368`
const BORDERED: [&str; 3] = ["PANEL_TIME_OFFSET", "panel3", "PANEL_SHUTTER_LAG"];

/// A control by name.
#[must_use]
pub fn control_named(name: &str) -> Option<&'static Control> {
    CONTROLS.iter().find(|control| control.name == name)
}

/// Where a control is in the client area: its place, moved by its panel's place and border.
#[must_use]
pub fn absolute(name: &str) -> Place {
    let Some(control) = control_named(name) else {
        return (0.0, 0.0, 0.0, 0.0);
    };
    let (mut x, mut y, width, height) = control.place;
    if let Some(panel) = control_named(control.parent) {
        let border = if BORDERED.contains(&panel.name) {
            1.0
        } else {
            0.0
        };
        x += panel.place.0 + border;
        y += panel.place.1 + border;
    }
    (x, y, width, height)
}

/// A control's `Text`.
fn text_of(name: &str) -> &'static str {
    control_named(name).map_or("", |control| control.text)
}

/// The Designer's event wirings, each with the function here that is its handler.
/// `// C#: GeoRef/Georefimage.Designer.cs:85, 110, 118, 126, 134, 142, 150, 172, 181, 298, 334, 356, 377, 390, 397`
#[cfg(test)]
pub const WIRINGS: &[(&str, &str, &str, &str)] = &[
    (
        "TXT_logfile",
        "TextChanged",
        "TXT_logfile_TextChanged",
        "text_changed",
    ),
    (
        "BUT_Geotagimages",
        "Click",
        "BUT_Geotagimages_Click",
        "geotag_click",
    ),
    (
        "BUT_estoffset",
        "Click",
        "BUT_estoffset_Click",
        "estimate_click",
    ),
    ("BUT_doit", "Click", "BUT_doit_Click", "doit_click"),
    (
        "BUT_browsedir",
        "Click",
        "BUT_browsedir_Click",
        "browse_dir",
    ),
    (
        "BUT_browselog",
        "Click",
        "BUT_browselog_Click",
        "browse_log",
    ),
    (
        "BUT_networklinkgeoref",
        "Click",
        "BUT_networklinkgeoref_Click",
        "georef_network_link",
    ),
    (
        "RDIO_TimeOffset",
        "CheckedChanged",
        "ProcessType_CheckedChanged",
        "process_type_changed",
    ),
    (
        "RDIO_CAMMsgSynchro",
        "CheckedChanged",
        "ProcessType_CheckedChanged",
        "process_type_changed",
    ),
    (
        "num_minshutter",
        "ValueChanged",
        "num_minshutter_ValueChanged",
        "set_num",
    ),
    (
        "chk_usegps2",
        "CheckedChanged",
        "chk_usegps2_CheckedChanged",
        "click_check",
    ),
    (
        "CHECK_AMSLAlt_Use",
        "CheckedChanged",
        "CHECK_AMSLAlt_Use_CheckedChanged",
        "click_check",
    ),
    (
        "TXT_shutterLag",
        "TextChanged",
        "TXT_shutterLag_TextChanged",
        "text_changed",
    ),
    (
        "chk_cammsg",
        "CheckedChanged",
        "chk_cammsg_CheckedChanged",
        "click_check",
    ),
    (
        "RDIO_trigmsg",
        "CheckedChanged",
        "ProcessType_CheckedChanged",
        "process_type_changed",
    ),
];

// ---------------------------------------------------------------------------------------------
// The controls.
// ---------------------------------------------------------------------------------------------

/// The form's `TextBox`es that take typing (`TXT_outputlog` is `ReadOnly`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// `TXT_logfile`.
    LogFile,
    /// `TXT_jpgdir`.
    JpgDir,
    /// `TXT_offsetseconds`, in `PANEL_TIME_OFFSET`.
    OffsetSeconds,
    /// `TXT_shutterLag`, in `PANEL_SHUTTER_LAG`.
    ShutterLag,
    /// `txt_basealt`, in `panel3`.
    BaseAlt,
}

impl Field {
    /// Every box.
    pub const ALL: [Self; 5] = [
        Self::LogFile,
        Self::JpgDir,
        Self::OffsetSeconds,
        Self::ShutterLag,
        Self::BaseAlt,
    ];

    /// The Designer's name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::LogFile => "TXT_logfile",
            Self::JpgDir => "TXT_jpgdir",
            Self::OffsetSeconds => "TXT_offsetseconds",
            Self::ShutterLag => "TXT_shutterLag",
            Self::BaseAlt => "txt_basealt",
        }
    }

    const fn index(self) -> usize {
        self as usize
    }
}

/// The form's `NumericUpDown`s.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Num {
    /// `num_dropfromstart`.
    DropFromStart,
    /// `num_dropend`.
    DropEnd,
    /// `num_minshutter`.
    MinShutter,
    /// `num_vfov`, "Dir fov".
    Vfov,
    /// `num_camerarotation`.
    CameraRotation,
    /// `num_hfov`, "Cross fov".
    Hfov,
}

/// A `NumericUpDown` as the Designer sets it up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NumSpec {
    /// `Minimum`.
    pub minimum: Decimal,
    /// `Maximum`.
    pub maximum: Decimal,
    /// `Increment`.
    pub increment: Decimal,
    /// `DecimalPlaces`.
    pub places: u32,
    /// `Value`.
    pub initial: Decimal,
}

impl Num {
    /// Every number.
    pub const ALL: [Self; 6] = [
        Self::DropFromStart,
        Self::DropEnd,
        Self::MinShutter,
        Self::Vfov,
        Self::CameraRotation,
        Self::Hfov,
    ];

    /// The Designer's name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::DropFromStart => "num_dropfromstart",
            Self::DropEnd => "num_dropend",
            Self::MinShutter => "num_minshutter",
            Self::Vfov => "num_vfov",
            Self::CameraRotation => "num_camerarotation",
            Self::Hfov => "num_hfov",
        }
    }

    const fn index(self) -> usize {
        self as usize
    }

    /// `Minimum`, `Maximum`, `Increment`, `DecimalPlaces` and `Value`, the defaults (0, 100, 1,
    /// 0, 0) where the Designer sets none.
    /// `// C#: GeoRef/Georefimage.Designer.cs:198-251, 282-331`
    #[must_use]
    pub const fn spec(self) -> NumSpec {
        const fn whole(minimum: i128, maximum: i128, initial: i128) -> NumSpec {
            NumSpec {
                minimum: Decimal::new(minimum, 0),
                maximum: Decimal::new(maximum, 0),
                increment: Decimal::new(1, 0),
                places: 0,
                initial: Decimal::new(initial, 0),
            }
        }
        match self {
            Self::DropFromStart | Self::DropEnd => whole(0, 900, 0),
            Self::Vfov => whole(0, 900, 130),
            Self::Hfov => whole(0, 900, 200),
            Self::CameraRotation => whole(-180, 180, 90),
            // `Increment` 1 at scale 2 (flags 131072), `Value` 5 at scale 1 (flags 65536).
            Self::MinShutter => NumSpec {
                minimum: Decimal::new(0, 0),
                maximum: Decimal::new(99, 0),
                increment: Decimal::new(1, 2),
                places: 2,
                initial: Decimal::new(5, 1),
            },
        }
    }
}

/// The form's `CheckBox`es.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Check {
    /// `chk_cammsg`, "Use Cam Messages": in time-offset mode, the `CAM` messages as positions.
    CamMsg,
    /// `chk_camusegpsalt`: in CAM mode, the GPS altitude.
    CamUseGpsAlt,
    /// `chk_trigusergpsalt`: in TRIG mode, the GPS altitude.
    TrigUseGpsAlt,
    /// `CHECK_AMSLAlt_Use`, checked in the Designer.
    AmslAltUse,
    /// `chk_usegps2`.
    UseGps2,
}

impl Check {
    /// Every box.
    pub const ALL: [Self; 5] = [
        Self::CamMsg,
        Self::CamUseGpsAlt,
        Self::TrigUseGpsAlt,
        Self::AmslAltUse,
        Self::UseGps2,
    ];

    /// The Designer's name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::CamMsg => "chk_cammsg",
            Self::CamUseGpsAlt => "chk_camusegpsalt",
            Self::TrigUseGpsAlt => "chk_trigusergpsalt",
            Self::AmslAltUse => "CHECK_AMSLAlt_Use",
            Self::UseGps2 => "chk_usegps2",
        }
    }

    const fn index(self) -> usize {
        self as usize
    }
}

/// The three `RadioButton`s, which pick `selectedProcessingMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Radio {
    /// `RDIO_TimeOffset`.
    TimeOffset,
    /// `RDIO_CAMMsgSynchro`, checked in the Designer.
    CamMsg,
    /// `RDIO_trigmsg`.
    Trig,
}

impl Radio {
    /// Every button.
    pub const ALL: [Self; 3] = [Self::TimeOffset, Self::CamMsg, Self::Trig];

    /// The Designer's name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::TimeOffset => "RDIO_TimeOffset",
            Self::CamMsg => "RDIO_CAMMsgSynchro",
            Self::Trig => "RDIO_trigmsg",
        }
    }
}

/// `PROCESSING_MODE`'s names, as `log.Info("process " + selectedProcessingMode)` writes them.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:27-32`
const fn mode_name(mode: ProcessingMode) -> &'static str {
    match mode {
        ProcessingMode::TimeOffset => "TIME_OFFSET",
        ProcessingMode::CamMsg => "CAM_MSG",
        ProcessingMode::Trig => "TRIG",
    }
}

/// The box a key goes to.
#[derive(Debug)]
enum Editing {
    /// A `TextBox`: each keystroke is its `TextChanged`.
    Text(Field),
    /// A `NumericUpDown`: what is typed, read into `Value` when the box is left or on Enter.
    Num(Num, TextField),
}

/// What a click on a box asks to type into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target {
    /// A `TextBox`.
    Text(Field),
    /// A `NumericUpDown`.
    Num(Num),
}

/// A dialog or message box over the form.
#[derive(Debug)]
pub enum Prompt {
    /// `openFileDialog1`: a log's path, typed after the folder the dialog opens in.
    Log(TextField),
    /// `folderBrowserDialog1`: a folder's path, starting at `SelectedPath`.
    Folder(TextField),
    /// `Program.handleException`'s box: its caption and text.
    Message(&'static str, String),
}

impl Prompt {
    /// What the facts call it.
    const fn name(&self) -> &'static str {
        match self {
            Self::Log(_) => "Browse Log",
            Self::Folder(_) => "Browse Pictures",
            Self::Message(caption, _) => caption,
        }
    }

    /// Its box's text, or its message.
    fn text(&self) -> &str {
        match self {
            Self::Log(field) | Self::Folder(field) => field.value(),
            Self::Message(_, text) => text,
        }
    }
}

/// `Program.handleException`'s box, which an exception out of a click handler reaches: its
/// caption, and the text before the exception. `// C#: Program.cs:791-793`
const UNHANDLED: (&str, &str) = ("Send Error", "An error has occurred\n");

/// `openFileDialog1.Filter`'s one description. `// C#: GeoRef/georefimage.cs:89`
pub const LOG_FILTER: &str = "Logs|*.log;*.tlog;*.bin;*.BIN";

// ---------------------------------------------------------------------------------------------
// The runs.
// ---------------------------------------------------------------------------------------------

/// Which button's run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Estimate Offset.
    Estimate,
    /// Pre-process.
    Process,
    /// GeoTag Images.
    Geotag,
}

impl Kind {
    /// What the facts call it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Estimate => "estimate",
            Self::Process => "process",
            Self::Geotag => "geotag",
        }
    }
}

/// A button's call into `GeoRefImageBase`, with the arguments its handler reads from the form.
#[derive(Debug, Clone, PartialEq)]
pub enum Job {
    /// `georef.EstimateOffset(TXT_logfile.Text, TXT_jpgdir.Text, UseGpsorGPS2(),
    /// chk_cammsg.Checked, AppendText)`. `// C#: GeoRef/georefimage.cs:262-263`
    Estimate {
        /// `TXT_logfile.Text`.
        log: String,
        /// `TXT_jpgdir.Text`.
        dir: String,
        /// `UseGpsorGPS2()`.
        gps: &'static str,
        /// `chk_cammsg.Checked`.
        usecam: bool,
    },
    /// The mode's `dowork` and `CreateReportFiles`. `// C#: GeoRef/georefimage.cs:167-200`
    Process {
        /// `TXT_logfile.Text`.
        log: String,
        /// `TXT_jpgdir.Text`.
        dir: String,
        /// The controls it reads.
        settings: FormSettings,
    },
    /// `WriteCoordinatesToImage` for each matched photo. `// C#: GeoRef/georefimage.cs:268-319`
    Geotag {
        /// `TXT_jpgdir.Text`.
        dir: String,
        /// The controls it reads.
        settings: FormSettings,
    },
}

impl Job {
    /// Which button's.
    #[must_use]
    pub const fn kind(&self) -> Kind {
        match self {
            Self::Estimate { .. } => Kind::Estimate,
            Self::Process { .. } => Kind::Process,
            Self::Geotag { .. } => Kind::Geotag,
        }
    }
}

/// What a run returns to the form.
#[derive(Debug)]
pub enum Outcome {
    /// The offset, or what `EstimateOffset` threw.
    Estimate(Result<f64, GeorefError>),
    /// The report files written (none where the matching found nothing or threw), and what the
    /// map then draws.
    Process {
        /// The files, with the KML `httpGeoRefKML` is handed.
        files: Option<ReportFiles>,
        /// Where they were written.
        dir: String,
        /// The route and the markers.
        contents: MapContents,
    },
    /// What `BUT_Geotagimages_Click` threw, if anything, and the folder it tagged in.
    Geotag(Result<(), GeorefError>, String),
}

/// One run on its thread: its lines, then the `GeoRefImageBase` it had and what it returned.
enum Event {
    Line(String),
    Done(Box<GeoRefImageBase>, Outcome),
}

/// A run under way.
struct Running {
    kind: Kind,
    receiver: mpsc::Receiver<Event>,
}

/// `GMapMarkerPhoto.hfov` and `vfov`, the footprint's field of view: the statics' 63 by 43. The
/// flight screen's map loop sets them from `camera_fovh`/`camera_fovv` when it draws the
/// vehicle's own camera markers, which this application's flight map does not draw.
/// `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs:20-21; GCSViews/FlightData.cs:4015-4019`
pub const PHOTO_FOV: (f64, f64) = (63.0, 43.0);

/// Runs one button's call as its handler does, every line it appends going to `out`. Pre-process
/// then works out what the map draws, as the handler's end does.
/// `// C#: GeoRef/georefimage.cs:167-246, 258-266, 268-319`
pub fn run_job(
    georef: &mut GeoRefImageBase,
    job: &Job,
    terrain: &dyn Terrain,
    out: &mut dyn FnMut(&str),
) -> Outcome {
    match job {
        Job::Estimate {
            log,
            dir,
            gps,
            usecam,
        } => Outcome::Estimate(georef.estimate_offset(log, dir, gps, *usecam, &mut *out)),
        Job::Process { log, dir, settings } => {
            let files = georef.process(log, dir, settings, &mut *out, terrain);
            Outcome::Process {
                files,
                dir: dir.clone(),
                contents: map_contents(georef, terrain, PHOTO_FOV),
            }
        }
        Job::Geotag { dir, settings } => {
            Outcome::Geotag(georef.geotag_images(dir, settings, &mut *out), dir.clone())
        }
    }
}

/// `srtm.getAltitude`: the planner's own terrain lookup (`crate::srtm`), which the footprints
/// in the KML and on the map are projected onto.
pub(crate) struct PlannerTerrain;

impl Terrain for PlannerTerrain {
    fn altitude(&self, lat: f64, lng: f64) -> f64 {
        crate::srtm::altitude(lat, lng).alt
    }
}

/// `DateTime.Today`: the date now, at midnight - the machine's time taken as UTC, as the port
/// takes local time everywhere (`mp_georef::time`). `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1197`
fn today() -> DateTime {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let days = i64::try_from(seconds / 86_400).unwrap_or(0);
    DateTime::from_ticks(UNIX_EPOCH_TICKS + days * TICKS_PER_DAY, TimeKind::Local)
}

// ---------------------------------------------------------------------------------------------
// The map.
// ---------------------------------------------------------------------------------------------

/// A `GMapMarkerPhoto` for a `CAM` message.
#[derive(Debug, Clone, PartialEq)]
pub struct CamMarker {
    /// `new PointLatLng(mark.lat / 1e7, mark.lng / 1e7)`.
    pub position: LatLon,
    /// Its `ToolTipText`.
    pub tooltip: String,
    /// `footprintpoly`: `ImageProjection.calc` at the marker, level, the camera unturned.
    pub footprint: Vec<LatLon>,
}

/// A green `GMarkerGoogle` for a matched photo.
#[derive(Debug, Clone, PartialEq)]
pub struct PhotoMarker {
    /// The photo's position.
    pub position: LatLon,
    /// `Path.GetFileName(pictureLocation.Value.Path)`, its tooltip - by which a click finds the
    /// photo to show.
    pub tooltip: String,
}

/// What `myGMAP1.Overlays[0]` holds after Pre-process.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MapContents {
    /// The `GMapRoute` "vehicle": every `vehicleLocations` entry.
    pub route: Vec<LatLon>,
    /// The camera markers, first.
    pub cams: Vec<CamMarker>,
    /// The photo pins, after them.
    pub photos: Vec<PhotoMarker>,
}

/// What Pre-process draws, from what `GeoRefImageBase` holds after it: the route, a
/// `GMapMarkerPhoto` of a `mavlink_camera_feedback_t` made from each `CAM` - its position through
/// `(int)(Lat * 1e7)`, its altitude `(float)AltAMSL`, roll, pitch and yaw 0, `img_idx` counting
/// from 0 - and a green pin at each matched photo.
/// `// C#: GeoRef/georefimage.cs:205-241; ExtLibs/Maps/GMapMarkerPhoto.cs:38-63`
#[must_use]
pub fn map_contents(
    georef: &GeoRefImageBase,
    terrain: &dyn Terrain,
    fov: (f64, f64),
) -> MapContents {
    let route = georef
        .vehicle_locations
        .values()
        .filter_map(|location| LatLon::new(location.lat, location.lon).ok())
        .collect();
    let mut cams = Vec::new();
    for (index, location) in georef.cam_locations.values().enumerate() {
        // `(int)(Lat * 1e7)`, then `mark.lat / 1e7`.
        #[allow(clippy::cast_possible_truncation)]
        let lat = f64::from((location.lat * 1e7) as i32) / 1e7;
        #[allow(clippy::cast_possible_truncation)]
        let lng = f64::from((location.lon * 1e7) as i32) / 1e7;
        let Ok(position) = LatLon::new(lat, lng) else {
            continue;
        };
        #[allow(clippy::cast_possible_truncation)]
        let alt = location.alt_amsl as f32;
        // `ushort a = 0; ... a++`.
        let img_idx = index % 65_536;
        // `Roll = local.roll - rolltrim`: 0 - 0.
        let tooltip = format!(
            "Photo\nAlt: {}\nNo: {img_idx}\nRoll: 0.00",
            mp_log::netfmt::single(alt)
        );
        let corners = mp_georef::projection::calc_quick(
            mp_georef::projection::Point {
                lat,
                lng,
                alt: f64::from(alt),
            },
            0.0,
            fov.0,
            fov.1,
            terrain,
        );
        let footprint = corners
            .iter()
            .filter_map(|corner| LatLon::new(corner.lat, corner.lng).ok())
            .collect();
        cams.push(CamMarker {
            position,
            tooltip,
            footprint,
        });
    }
    let photos = georef
        .pictures_info
        .iter()
        .flat_map(|pictures| pictures.values())
        .filter_map(|picture| {
            let position = LatLon::new(picture.location.lat, picture.location.lon).ok()?;
            Some(PhotoMarker {
                position,
                tooltip: mp_georef::photos::file_name(&picture.path),
            })
        })
        .collect();
    MapContents {
        route,
        cams,
        photos,
    }
}

/// A marker of [`MapContents`]: the n-th camera marker or the n-th photo pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Marker {
    /// A `GMapMarkerPhoto`.
    Cam(usize),
    /// A green `GMarkerGoogle`.
    Photo(usize),
}

/// A marker's `LocalArea` from its point: `GMapMarkerPhoto`'s 20 by 20 icon at `Offset`
/// (-10, -10), and `GMarkerGoogle`'s 32 by 32 pin at (-15, -31). `(left, top, width, height)`.
/// `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs:15, 41-42; ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/Markers/GMarkerGoogle.cs:112-127`
#[must_use]
pub const fn marker_area(marker: Marker) -> (f32, f32, f32, f32) {
    match marker {
        Marker::Cam(_) => (-10.0, -10.0, 20.0, 20.0),
        Marker::Photo(_) => (-15.0, -31.0, 32.0, 32.0),
    }
}

/// The markers whose `LocalArea` holds the point, in the overlay's order: GMap's
/// `IsMouseOver`, for every marker the pointer is over, and the first of them is the one a click
/// goes to. `screen` says where a position was drawn. `Rectangle.Contains` takes the left and
/// top edges and not the right and bottom.
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapControl.cs:1912-1936, 2134-2187`
pub fn markers_at(
    contents: &MapContents,
    (x, y): (f32, f32),
    screen: impl Fn(LatLon) -> Option<(f32, f32)>,
) -> Vec<Marker> {
    let cams = contents
        .cams
        .iter()
        .enumerate()
        .map(|(index, cam)| (Marker::Cam(index), cam.position));
    let photos = contents
        .photos
        .iter()
        .enumerate()
        .map(|(index, photo)| (Marker::Photo(index), photo.position));
    cams.chain(photos)
        .filter(|(marker, position)| {
            let Some((px_, py_)) = screen(*position) else {
                return false;
            };
            let (left, top, width, height) = marker_area(*marker);
            let (left, top) = (px_ + left, py_ + top);
            (left..left + width).contains(&x) && (top..top + height).contains(&y)
        })
        .map(|(marker, _)| marker)
        .collect()
}

/// A marker's tooltip.
#[must_use]
pub fn tooltip(contents: &MapContents, marker: Marker) -> Option<&str> {
    match marker {
        Marker::Cam(index) => contents.cams.get(index).map(|cam| cam.tooltip.as_str()),
        Marker::Photo(index) => contents
            .photos
            .get(index)
            .map(|photo| photo.tooltip.as_str()),
    }
}

/// `myGMAP1.MapProvider = MainV2.instance.FlightData.gMapControl1.MapProvider`: a tile store of
/// its own for the flight map's source. `// C#: GeoRef/georefimage.cs:39`
fn use_imagery(map: &Rc<RefCell<MapViewport>>, source: Option<&str>) {
    let Some(source) = source.and_then(mp_tiles::source::source_by_id) else {
        return;
    };
    let cache = mp_tiles::cache::TileCache::new(mp_tiles::cache::TileCache::default_root());
    let store = if std::env::var("MP_OFFLINE").is_ok() {
        mp_tiles::TileStore::offline(source, cache)
    } else {
        mp_tiles::TileStore::new(source, cache)
    };
    map.borrow_mut().set_tiles(Arc::new(store));
}

// ---------------------------------------------------------------------------------------------
// The form.
// ---------------------------------------------------------------------------------------------

/// `Path.GetDirectoryName`: the path up to its last separator, "" with none, `None` for a root
/// (the C#'s `null`); an empty path throws, `Err`.
/// `// C#: referencesource mscorlib/system/io/path.cs:120-150`
fn get_directory_name(path: &str) -> Result<Option<String>, ()> {
    if path.trim().is_empty() {
        return Err(());
    }
    let is_separator = |c: char| c == '/' || c == std::path::MAIN_SEPARATOR;
    let root = usize::from(path.starts_with(is_separator));
    let chars: Vec<char> = path.chars().collect();
    let mut i = chars.len();
    if i <= root {
        return Ok(None);
    }
    loop {
        i -= 1;
        if i <= root || chars.get(i).copied().is_some_and(is_separator) {
            break;
        }
    }
    let i = i.max(root);
    Ok(Some(chars.get(..i).unwrap_or_default().iter().collect()))
}

/// `Settings.Instance.LogDir`, where the log dialog opens here: `openFileDialog1` has no
/// `InitialDirectory`, so the system's dialog opens where it was last - here the log directory, as
/// the DataFlash page's conversions' dialogs do.
/// `// C#: GeoRef/georefimage.cs:87-97; ExtLibs/Utilities/Settings.cs:127-140`
fn log_directory() -> Option<std::path::PathBuf> {
    std::env::var_os("MP_LOG_DIR")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            mp_settings::Config::default_path()
                .and_then(|path| mp_settings::Config::load(&path).ok())
                .and_then(|config| config.log_directory())
        })
        .or_else(mp_settings::default_log_directory)
}

/// One showing of the form: `new Georefimage()`, its controls and its `GeoRefImageBase`.
pub struct Form {
    /// `georef`: `None` while a run has it on its thread.
    georef: Option<GeoRefImageBase>,
    /// The `TextBox`es, by [`Field`].
    texts: [TextField; 5],
    /// The `NumericUpDown`s' `Value`s, by [`Num`].
    nums: [Decimal; 6],
    /// The `CheckBox`es, by [`Check`].
    checks: [bool; 5],
    /// The radio button checked.
    radio: Radio,
    /// `selectedProcessingMode`.
    mode: ProcessingMode,
    /// `PANEL_TIME_OFFSET.Enabled`.
    time_offset_enabled: bool,
    /// `PANEL_SHUTTER_LAG.Enabled`.
    shutter_lag_enabled: bool,
    /// `txt_basealt.Enabled`.
    base_alt_enabled: bool,
    /// `BUT_doit.Enabled`.
    doit_enabled: bool,
    /// `BUT_Geotagimages.Enabled`.
    geotag_enabled: bool,
    /// `TXT_outputlog.Text`.
    output: String,
    /// The box being typed into.
    editing: Option<Editing>,
    /// A dialog or message box over the form.
    prompt: Option<Prompt>,
    /// `openFileDialog1.FileName`, kept from one showing of the dialog to the next.
    file_name: String,
    /// `folderBrowserDialog1.SelectedPath`, likewise.
    selected_path: String,
    /// The run under way.
    running: Option<Running>,
    /// The last run to finish.
    last: Option<Kind>,
    /// The last Estimate Offset's answer.
    offset: Option<f64>,
    /// The last Pre-process's report files, by name, with their sizes on the disk.
    files: Option<Vec<(String, u64)>>,
    /// `httpserver.georefkml`: the KML `httpGeoRefKML` is handed.
    kml: Option<String>,
    /// The network link file Location Kml last opened.
    network_link: Option<String>,
    /// The files in the `geotagged` folder after the last GeoTag Images.
    geotagged: Option<usize>,
    /// `myGMAP1`.
    map: Rc<RefCell<MapViewport>>,
    /// What it draws over the imagery.
    contents: Rc<MapContents>,
    /// `ZoomAndCenterRoutes`' points, until the map has been painted and can fit them.
    fit: Option<Vec<LatLon>>,
    /// The markers the pointer is over.
    hovered: Vec<Marker>,
    /// Where the left button went down on the map, and whether it has moved since.
    press: Option<(f32, f32, bool)>,
    /// `pictureBox1.ImageLocation`.
    image_location: Option<String>,
    /// `DateTime.Today` when the form was made.
    today: DateTime,
    /// `srtm`, for the footprints.
    terrain: Arc<dyn Terrain + Send + Sync>,
    /// A line arrived: the text box scrolls to its end, as `AppendText` leaves it.
    scroll_to_end: bool,
}

impl Form {
    /// `Georefimage()`: the Designer's controls, `CHECK_AMSLAlt_Use` checked and `useAMSLAlt`
    /// with it, `PANEL_TIME_OFFSET` disabled and CAM the mode; `myGMAP1` empty.
    /// `// C#: GeoRef/georefimage.cs:20, 26-47`
    #[must_use]
    pub fn new(today: DateTime, terrain: Arc<dyn Terrain + Send + Sync>) -> Self {
        let mut texts = Field::ALL.map(|_| TextField::new(""));
        for field in Field::ALL {
            if let Some(text) = texts.get_mut(field.index()) {
                text.set(text_of(field.name()));
            }
        }
        let mut georef = GeoRefImageBase::new(today);
        georef.use_amsl_alt = true;
        Self {
            georef: Some(georef),
            texts,
            nums: Num::ALL.map(|num| num.spec().initial),
            checks: Check::ALL.map(|check| check == Check::AmslAltUse),
            radio: Radio::CamMsg,
            mode: ProcessingMode::CamMsg,
            time_offset_enabled: false,
            shutter_lag_enabled: true,
            // `txt_basealt.Enabled` False in the `.resx`.
            base_alt_enabled: false,
            doit_enabled: true,
            // `BUT_Geotagimages.Enabled` False in the `.resx`.
            geotag_enabled: false,
            output: String::new(),
            editing: None,
            prompt: None,
            // `this.openFileDialog1.FileName = "openFileDialog1"`.
            file_name: "openFileDialog1".to_owned(),
            selected_path: String::new(),
            running: None,
            last: None,
            offset: None,
            files: None,
            kml: None,
            network_link: None,
            geotagged: None,
            map: Rc::new(RefCell::new(MapViewport::new(0, 0))),
            contents: Rc::new(MapContents::default()),
            fit: None,
            hovered: Vec::new(),
            press: None,
            image_location: None,
            today,
            terrain,
            scroll_to_end: false,
        }
    }

    // ---- What the controls hold ----

    /// A `TextBox`'s `Text`.
    #[must_use]
    pub fn text(&self, field: Field) -> &str {
        self.texts.get(field.index()).map_or("", TextField::value)
    }

    /// A `NumericUpDown`'s `Value`.
    #[must_use]
    pub fn num(&self, num: Num) -> Decimal {
        self.nums.get(num.index()).copied().unwrap_or(Decimal::ZERO)
    }

    /// What a `NumericUpDown` shows: what is being typed, or `Value` at `DecimalPlaces`.
    #[must_use]
    pub fn num_text(&self, num: Num) -> String {
        match &self.editing {
            Some(Editing::Num(held, field)) if *held == num => field.value().to_owned(),
            _ => self.num(num).to_fixed(num.spec().places),
        }
    }

    /// A `CheckBox`'s `Checked`.
    #[must_use]
    pub fn check(&self, check: Check) -> bool {
        self.checks.get(check.index()).copied().unwrap_or(false)
    }

    /// Whether a `RadioButton` is checked.
    #[must_use]
    pub fn radio(&self, radio: Radio) -> bool {
        self.radio == radio
    }

    /// `selectedProcessingMode`.
    #[must_use]
    pub const fn mode(&self) -> ProcessingMode {
        self.mode
    }

    /// `TXT_outputlog.Text`.
    #[must_use]
    pub fn output(&self) -> &str {
        &self.output
    }

    /// The `GeoRefImageBase`, unless a run has it.
    #[must_use]
    pub const fn georef(&self) -> Option<&GeoRefImageBase> {
        self.georef.as_ref()
    }

    /// The run under way.
    #[must_use]
    pub fn running(&self) -> Option<Kind> {
        self.running.as_ref().map(|running| running.kind)
    }

    /// What the map draws.
    #[must_use]
    pub fn contents(&self) -> &MapContents {
        &self.contents
    }

    /// Whether a control takes clicks and keys as the C#'s `Enabled` has it: the panels' controls
    /// with their panel, `txt_basealt`, the two buttons the handlers switch, and Location Kml and
    /// the picture box, which are not ported.
    #[must_use]
    pub fn enabled(&self, name: &str) -> bool {
        match name {
            "PANEL_TIME_OFFSET" | "label1" | "TXT_offsetseconds" | "BUT_estoffset" => {
                self.time_offset_enabled
            }
            "PANEL_SHUTTER_LAG" | "label27" | "TXT_shutterLag" => self.shutter_lag_enabled,
            "txt_basealt" => self.base_alt_enabled,
            "BUT_doit" => self.doit_enabled,
            "BUT_Geotagimages" => self.geotag_enabled,
            "pictureBox1" => false,
            _ => true,
        }
    }

    fn target_enabled(&self, target: Target) -> bool {
        match target {
            Target::Text(field) => self.enabled(field.name()),
            Target::Num(num) => self.enabled(num.name()),
        }
    }

    /// What a run reads from the controls.
    #[must_use]
    pub fn settings(&self) -> FormSettings {
        let int = |num: Num| self.num(num).to_i32().unwrap_or(0);
        FormSettings {
            mode: self.mode,
            offset_text: self.text(Field::OffsetSeconds).to_owned(),
            use_gps2: self.check(Check::UseGps2),
            use_cam_messages: self.check(Check::CamMsg),
            drop_start: int(Num::DropFromStart),
            drop_end: int(Num::DropEnd),
            camera_rotation: self.num(Num::CameraRotation).to_f64(),
            hfov: self.num(Num::Hfov).to_f64(),
            vfov: self.num(Num::Vfov).to_f64(),
            cam_use_gps_alt: self.check(Check::CamUseGpsAlt),
            trig_use_gps_alt: self.check(Check::TrigUseGpsAlt),
            base_alt_text: self.text(Field::BaseAlt).to_owned(),
        }
    }

    /// While a run holds `georef` the form takes nothing: the C#'s is in its handler.
    fn inert(&self) -> bool {
        self.running.is_some()
    }

    /// `AppendText`: a run's text at the end of the box.
    /// `// C#: GeoRef/georefimage.cs:73-85`
    fn append(&mut self, text: &str) {
        self.output.push_str(text);
        self.scroll_to_end = true;
    }

    // ---- Typing ----

    /// A click on a box: whatever was being typed is taken, and this one takes the keyboard.
    pub fn start_edit(&mut self, target: Target) {
        if self.inert() || self.prompt.is_some() || !self.target_enabled(target) {
            return;
        }
        let same = match (&self.editing, target) {
            (Some(Editing::Text(held)), Target::Text(field)) => *held == field,
            (Some(Editing::Num(held, _)), Target::Num(num)) => *held == num,
            _ => false,
        };
        if same {
            return;
        }
        self.commit_edit();
        self.editing = Some(match target {
            Target::Text(field) => Editing::Text(field),
            Target::Num(num) => {
                let mut field = TextField::new("");
                field.set(self.num_text(num));
                Editing::Num(num, field)
            }
        });
    }

    /// Leaving a box: a `NumericUpDown`'s `ValidateEditText`. A `TextBox` has had each keystroke.
    pub fn commit_edit(&mut self) {
        if let Some(Editing::Num(num, field)) = self.editing.take() {
            self.type_num(num, field.value());
        }
    }

    /// The box being typed into.
    #[must_use]
    pub fn editing(&self) -> Option<Target> {
        match &self.editing {
            Some(Editing::Text(field)) => Some(Target::Text(*field)),
            Some(Editing::Num(num, _)) => Some(Target::Num(*num)),
            None => None,
        }
    }

    /// `TextBox.Text = value`: `TextChanged` when it changes.
    pub fn set_text(&mut self, field: Field, value: &str) {
        let Some(text) = self.texts.get_mut(field.index()) else {
            return;
        };
        if text.value() == value {
            return;
        }
        text.set(value);
        self.text_changed(field);
    }

    /// A box's `TextChanged`. The log box's forgets the positions and the pictures and dims
    /// GeoTag Images; the shutter lag's reads the number, and puts "0" back for anything that is
    /// not one - which is a change of its own, read again.
    /// `// C#: GeoRef/georefimage.cs:327-335, 360-367`
    pub fn text_changed(&mut self, field: Field) {
        match field {
            Field::LogFile => {
                if let Some(georef) = self.georef.as_mut() {
                    georef.forget_positions(true);
                }
                self.geotag_enabled = false;
            }
            Field::ShutterLag => {
                // `int.TryParse(..., out georef.millisShutterLag)`: 0 when it fails.
                let parsed = mp_log::netfmt::parse_i32(self.text(Field::ShutterLag));
                if let Some(georef) = self.georef.as_mut() {
                    georef.millis_shutter_lag = parsed.unwrap_or(0);
                }
                if parsed.is_none() {
                    self.set_text(Field::ShutterLag, "0");
                }
            }
            Field::JpgDir | Field::OffsetSeconds | Field::BaseAlt => {}
        }
    }

    /// A key in the box being typed into, or in the dialog over the form. Returns whether
    /// anything changed.
    pub fn key(&mut self, event: &gpui::KeyDownEvent) -> bool {
        if self.inert() {
            return false;
        }
        if let Some(prompt) = self.prompt.as_mut() {
            let outcome = match prompt {
                Prompt::Log(field) | Prompt::Folder(field) => field.key(event),
                Prompt::Message(..) => match event.keystroke.key.as_str() {
                    "enter" => KeyOutcome::Submitted,
                    "escape" => KeyOutcome::Cancelled,
                    _ => KeyOutcome::Ignored,
                },
            };
            return match outcome {
                KeyOutcome::Submitted => {
                    self.answer_prompt(true);
                    true
                }
                KeyOutcome::Cancelled => {
                    self.answer_prompt(false);
                    true
                }
                KeyOutcome::Changed => true,
                KeyOutcome::Ignored => false,
            };
        }
        match self.editing.as_mut() {
            Some(Editing::Text(field)) => {
                let field = *field;
                let Some(text) = self.texts.get_mut(field.index()) else {
                    return false;
                };
                match text.key(event) {
                    KeyOutcome::Changed => {
                        self.text_changed(field);
                        true
                    }
                    KeyOutcome::Cancelled => {
                        self.editing = None;
                        true
                    }
                    KeyOutcome::Submitted | KeyOutcome::Ignored => false,
                }
            }
            Some(Editing::Num(_, field)) => {
                // `NumericUpDown`'s key filter: a printable character that is not part of a
                // number is refused. `// C#: referencesource System.Windows.Forms/winforms/Managed/System/WinForms/NumericUpDown.cs:560-600`
                if let Some(typed) = event.keystroke.key_char.as_deref()
                    && !event.keystroke.modifiers.control
                    && typed.chars().any(|c| {
                        !c.is_control() && !c.is_ascii_digit() && !matches!(c, '.' | ',' | '-')
                    })
                {
                    return false;
                }
                match field.key(event) {
                    KeyOutcome::Changed => true,
                    KeyOutcome::Submitted => {
                        self.commit_edit();
                        true
                    }
                    KeyOutcome::Cancelled => {
                        self.editing = None;
                        true
                    }
                    KeyOutcome::Ignored => false,
                }
            }
            None => false,
        }
    }

    // ---- The check boxes, radio buttons and numbers ----

    /// A click on a `CheckBox`: it toggles, and its `CheckedChanged` runs. GPS2 and Use Cam
    /// Messages forget the positions; Use AMSL Alt sets `useAMSLAlt` and enables the base
    /// altitude's box only when it is off.
    /// `// C#: GeoRef/georefimage.cs:369-386`
    pub fn click_check(&mut self, check: Check) {
        if self.inert() || self.prompt.is_some() {
            return;
        }
        self.commit_edit();
        let Some(checked) = self.checks.get_mut(check.index()) else {
            return;
        };
        *checked = !*checked;
        let checked = *checked;
        match check {
            Check::CamMsg | Check::UseGps2 => {
                if let Some(georef) = self.georef.as_mut() {
                    georef.forget_positions(false);
                }
            }
            Check::AmslAltUse => {
                if let Some(georef) = self.georef.as_mut() {
                    georef.use_amsl_alt = checked;
                }
                self.base_alt_enabled = !checked;
            }
            Check::CamUseGpsAlt | Check::TrigUseGpsAlt => {}
        }
    }

    /// A click on a `RadioButton`: checked, the others not, and `ProcessType_CheckedChanged`. A
    /// click on the one already checked changes nothing and raises nothing.
    pub fn click_radio(&mut self, radio: Radio) {
        if self.inert() || self.prompt.is_some() {
            return;
        }
        self.commit_edit();
        if self.radio == radio {
            return;
        }
        self.radio = radio;
        self.process_type_changed();
    }

    /// `ProcessType_CheckedChanged`: the mode, the time-offset panel for time offset only, the
    /// shutter lag's for CAM only. A box in a panel disabled under the caret loses it.
    /// `// C#: GeoRef/georefimage.cs:337-357`
    fn process_type_changed(&mut self) {
        let (mode, time_offset, shutter_lag) = match self.radio {
            Radio::CamMsg => (ProcessingMode::CamMsg, false, true),
            Radio::Trig => (ProcessingMode::Trig, false, false),
            Radio::TimeOffset => (ProcessingMode::TimeOffset, true, false),
        };
        self.mode = mode;
        self.time_offset_enabled = time_offset;
        self.shutter_lag_enabled = shutter_lag;
        if let Some(target) = self.editing()
            && !self.target_enabled(target)
        {
            self.editing = None;
        }
    }

    /// `NumericUpDown.Value = value`: `ValueChanged` when it changes, whose one handler is
    /// `num_minshutter`'s - `georef.minshutter = (double)num_minshutter.Value`.
    /// `// C#: GeoRef/georefimage.cs:388-391`
    pub fn set_num(&mut self, num: Num, value: Decimal) {
        let Some(held) = self.nums.get_mut(num.index()) else {
            return;
        };
        if *held == value {
            return;
        }
        *held = value;
        if num == Num::MinShutter
            && let Some(georef) = self.georef.as_mut()
        {
            georef.minshutter = value.to_f64();
        }
    }

    /// `ParseEditText`: what `decimal.Parse` reads, held to the range; nothing when it does not
    /// parse. `// C#: referencesource System.Windows.Forms/winforms/Managed/System/WinForms/NumericUpDown.cs:615-650`
    pub fn type_num(&mut self, num: Num, text: &str) {
        let Some(value) = Decimal::parse(text) else {
            return;
        };
        let spec = num.spec();
        let value = if value < spec.minimum {
            spec.minimum
        } else if value > spec.maximum {
            spec.maximum
        } else {
            value
        };
        self.set_num(num, value);
    }

    /// The up arrow: one `Increment` more, no more than `Maximum`.
    pub fn up(&mut self, num: Num) {
        if self.inert() || self.prompt.is_some() {
            return;
        }
        self.commit_edit();
        let spec = num.spec();
        let value = match self.num(num).checked_add(spec.increment) {
            Some(value) if value <= spec.maximum => value,
            _ => spec.maximum,
        };
        self.set_num(num, value);
    }

    /// The down arrow: one `Increment` less, no less than `Minimum`.
    pub fn down(&mut self, num: Num) {
        if self.inert() || self.prompt.is_some() {
            return;
        }
        self.commit_edit();
        let spec = num.spec();
        let value = match self.num(num).checked_sub(spec.increment) {
            Some(value) if value >= spec.minimum => value,
            _ => spec.minimum,
        };
        self.set_num(num, value);
    }

    // ---- The dialogs ----

    /// Browse Log: `openFileDialog1`, here a box starting in `start`.
    /// `// C#: GeoRef/georefimage.cs:87-91`
    pub fn browse_log(&mut self, start: &str) {
        if self.inert() || self.prompt.is_some() {
            return;
        }
        self.commit_edit();
        self.editing = None;
        let mut field = TextField::new("");
        field.set(start);
        self.prompt = Some(Prompt::Log(field));
    }

    /// Browse Pictures: `folderBrowserDialog1`, starting at the log's folder when
    /// `GetDirectoryName` has one - it throws for an empty box, and the `catch` leaves the last
    /// folder chosen.
    /// `// C#: GeoRef/georefimage.cs:99-109`
    pub fn browse_dir(&mut self) {
        if self.inert() || self.prompt.is_some() {
            return;
        }
        self.commit_edit();
        self.editing = None;
        if let Ok(folder) = get_directory_name(self.text(Field::LogFile)) {
            self.selected_path = folder.unwrap_or_default();
        }
        let mut field = TextField::new("");
        field.set(self.selected_path.clone());
        self.prompt = Some(Prompt::Folder(field));
    }

    /// OK or Cancel on the box over the form. A dialog's OK takes only a file (a folder) that is
    /// there, as the system's dialog does; the handler then goes on whatever the dialog returned,
    /// as the C#'s does, which reads `FileName` and `SelectedPath` without asking - so Cancel
    /// takes the last file chosen, and the folder the dialog was opened at.
    /// `// C#: GeoRef/georefimage.cs:92-137`
    pub fn answer_prompt(&mut self, ok: bool) {
        match self.prompt.take() {
            Some(Prompt::Log(field)) => {
                let path = field.value().trim().to_owned();
                if ok {
                    if !std::path::Path::new(&path).is_file() {
                        self.prompt = Some(Prompt::Log(field));
                        return;
                    }
                    self.file_name = path;
                }
                // `if (File.Exists(openFileDialog1.FileName))`.
                if std::path::Path::new(&self.file_name).is_file() {
                    let file = self.file_name.clone();
                    self.set_text(Field::LogFile, &file);
                    let folder = get_directory_name(&file).ok().flatten().unwrap_or_default();
                    self.set_text(Field::JpgDir, &folder);
                }
            }
            Some(Prompt::Folder(field)) => {
                if ok {
                    let typed = field.value().trim();
                    let path = if typed.len() > 1 {
                        typed.trim_end_matches(['/', std::path::MAIN_SEPARATOR])
                    } else {
                        typed
                    };
                    if !std::path::Path::new(path).is_dir() {
                        self.prompt = Some(Prompt::Folder(field));
                        return;
                    }
                    self.selected_path = path.to_owned();
                }
                if !self.selected_path.is_empty() {
                    let folder = self.selected_path.clone();
                    self.set_text(Field::JpgDir, &folder);
                    // `location.txt`'s `seconds_offset: ` digits, if the file is there.
                    if let Some(digits) = mp_georef::run::offset_from_location_txt(&folder) {
                        self.set_text(Field::OffsetSeconds, &digits);
                    }
                }
            }
            Some(Prompt::Message(..)) | None => {}
        }
    }

    /// The box over the form.
    #[must_use]
    pub const fn prompt(&self) -> Option<&Prompt> {
        self.prompt.as_ref()
    }

    // ---- The three runs ----

    /// Estimate Offset: the box emptied, then `EstimateOffset` over the log and folder.
    /// `// C#: GeoRef/georefimage.cs:258-266`
    pub fn estimate_click(&mut self) -> Option<Job> {
        if self.inert() || self.prompt.is_some() || !self.enabled("BUT_estoffset") {
            return None;
        }
        self.commit_edit();
        self.output.clear();
        let settings = self.settings();
        Some(Job::Estimate {
            log: self.text(Field::LogFile).to_owned(),
            dir: self.text(Field::JpgDir).to_owned(),
            gps: settings.gps(),
            usecam: settings.use_cam_messages,
        })
    }

    /// Pre-process: nothing without the log and the folder, the offset's message for an offset
    /// that is not a number, and otherwise the button disabled, the box emptied and the markers
    /// taken off the map (the route stays until the run ends) before the run.
    /// `// C#: GeoRef/georefimage.cs:140-165`
    pub fn doit_click(&mut self) -> Option<Job> {
        if self.inert() || self.prompt.is_some() || !self.doit_enabled {
            return None;
        }
        self.commit_edit();
        let log = self.text(Field::LogFile).to_owned();
        let dir = self.text(Field::JpgDir).to_owned();
        if !std::path::Path::new(&log).is_file() || !std::path::Path::new(&dir).is_dir() {
            return None;
        }
        let settings = self.settings();
        if self.mode == ProcessingMode::TimeOffset
            && mp_georef::run::parse_offset(&settings.offset_text).is_none()
        {
            self.append("Offset number not in correct format. Use . as decimal separator\n");
            return None;
        }
        self.doit_enabled = false;
        self.output.clear();
        self.contents = Rc::new(MapContents {
            route: self.contents.route.clone(),
            ..MapContents::default()
        });
        self.hovered.clear();
        Some(Job::Process { log, dir, settings })
    }

    /// GeoTag Images, once Pre-process has enabled it.
    /// `// C#: GeoRef/georefimage.cs:268-272`
    pub fn geotag_click(&mut self) -> Option<Job> {
        if self.inert() || self.prompt.is_some() || !self.geotag_enabled {
            return None;
        }
        self.commit_edit();
        Some(Job::Geotag {
            dir: self.text(Field::JpgDir).to_owned(),
            settings: self.settings(),
        })
    }

    /// A button's call on a thread of its own, `georef` with it; the form takes nothing until
    /// it is back. False, and nothing started, with a run already under way.
    pub fn start(&mut self, job: Job) -> bool {
        if self.running.is_some() {
            return false;
        }
        let Some(mut georef) = self.georef.take() else {
            return false;
        };
        let (sender, receiver) = mpsc::channel();
        let terrain = Arc::clone(&self.terrain);
        let kind = job.kind();
        let spawned = std::thread::Builder::new()
            .name(format!("mp-georef-{}", kind.name()))
            .spawn(move || {
                let lines = sender.clone();
                let mut out = |text: &str| {
                    let _ = lines.send(Event::Line(text.to_owned()));
                };
                let outcome = run_job(&mut georef, &job, terrain.as_ref(), &mut out);
                let _ = sender.send(Event::Done(Box::new(georef), outcome));
            });
        if spawned.is_err() {
            // The thread and `georef` with it are gone: a fresh one with the controls' settings.
            self.georef = Some(self.fresh_georef());
            self.doit_enabled = true;
            return false;
        }
        self.running = Some(Running { kind, receiver });
        true
    }

    /// A `GeoRefImageBase` as the form's controls have set it up.
    fn fresh_georef(&self) -> GeoRefImageBase {
        let mut georef = GeoRefImageBase::new(self.today);
        georef.use_amsl_alt = self.check(Check::AmslAltUse);
        georef.millis_shutter_lag =
            mp_log::netfmt::parse_i32(self.text(Field::ShutterLag)).unwrap_or(0);
        georef.minshutter = self.num(Num::MinShutter).to_f64();
        georef
    }

    /// Once a frame: the lines that have arrived into the box, and a run that has finished
    /// taken back. Whether anything changed.
    pub fn poll(&mut self) -> bool {
        let Some(running) = self.running.as_ref() else {
            return false;
        };
        let kind = running.kind;
        let mut lines = Vec::new();
        let mut done = None;
        loop {
            match running.receiver.try_recv() {
                Ok(Event::Line(text)) => lines.push(text),
                Ok(Event::Done(georef, outcome)) => {
                    done = Some((*georef, Some(outcome)));
                    break;
                }
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => {
                    done = Some((self.fresh_georef(), None));
                    break;
                }
            }
        }
        let changed = !lines.is_empty() || done.is_some();
        for line in &lines {
            self.append(line);
        }
        if let Some((georef, outcome)) = done {
            self.running = None;
            self.georef = Some(georef);
            self.finish(kind, outcome);
        }
        changed
    }

    /// What the handler does after its call returns: the offset's line, the map drawn and the
    /// two buttons enabled after Pre-process, and the box an exception out of the handler shows.
    /// `// C#: GeoRef/georefimage.cs:202-249, 265, 283-318; Program.cs:791-793`
    fn finish(&mut self, kind: Kind, outcome: Option<Outcome>) {
        self.last = Some(kind);
        match outcome {
            Some(Outcome::Estimate(Ok(offset))) => {
                self.offset = Some(offset);
                self.append(&format!(
                    "Offset around :  {}\n\n",
                    mp_log::netfmt::double(offset)
                ));
            }
            Some(Outcome::Estimate(Err(error)) | Outcome::Geotag(Err(error), _)) => {
                self.prompt = Some(Prompt::Message(
                    UNHANDLED.0,
                    format!("{}{error}", UNHANDLED.1),
                ));
                if kind == Kind::Geotag {
                    self.geotagged = None;
                }
            }
            Some(Outcome::Process {
                files,
                dir,
                contents,
            }) => {
                self.kml = files.as_ref().map(|files| files.kml.clone());
                self.files = files.map(|files| {
                    files
                        .files
                        .iter()
                        .map(|(name, _)| {
                            let path = format!("{dir}{}{name}", mp_georef::photos::SEPARATOR);
                            let size = std::fs::metadata(path).map_or(0, |meta| meta.len());
                            (name.clone(), size)
                        })
                        .collect()
                });
                // `ZoomAndCenterRoutes`: nothing to fit with no route.
                self.fit = (!contents.route.is_empty()).then(|| contents.route.clone());
                self.contents = Rc::new(contents);
                self.hovered.clear();
                self.doit_enabled = true;
                self.geotag_enabled = true;
            }
            Some(Outcome::Geotag(Ok(()), dir)) => {
                let folder = mp_georef::photos::geotag_folder(&dir);
                self.geotagged = std::fs::read_dir(folder)
                    .ok()
                    .map(|entries| entries.filter_map(Result::ok).count());
            }
            None => {
                // The thread ended without an answer.
                self.doit_enabled = true;
            }
        }
    }

    // ---- The map ----

    /// `ZoomAndCenterRoutes`, once the map has been painted at its size: the whole zoom that
    /// fits the route, about its middle. `// C#: GeoRef/georefimage.cs:246`
    pub fn fit_map(&mut self) {
        let Some(points) = self.fit.as_ref() else {
            return;
        };
        if self.map.borrow_mut().zoom_to_fit(points) {
            self.fit = None;
        }
    }

    /// The pointer moved over the map with no button down: the markers it is over.
    pub fn hover(&mut self, pointer: Option<(f32, f32)>) -> bool {
        let hovered = pointer.map_or_else(Vec::new, |at| {
            let map = self.map.borrow();
            markers_at(&self.contents, at, |position| map.screen_of(position))
        });
        let changed = hovered != self.hovered;
        self.hovered = hovered;
        changed
    }

    /// A click on the map: `MyGMAP1_OnMarkerClick` for the first marker under it, which shows the
    /// photo whose file name is the marker's tooltip.
    /// `// C#: GeoRef/georefimage.cs:49-62`
    pub fn map_click(&mut self, at: (f32, f32)) {
        let first = {
            let map = self.map.borrow();
            markers_at(&self.contents, at, |position| map.screen_of(position))
                .into_iter()
                .next()
        };
        let Some(marker) = first else {
            return;
        };
        let Some(text) = tooltip(&self.contents, marker) else {
            return;
        };
        if let Some(pictures) = self.georef.as_ref().and_then(|g| g.pictures_info.as_ref())
            && let Some(picture) = pictures
                .values()
                .find(|picture| mp_georef::photos::file_name(&picture.path) == text)
        {
            self.image_location = Some(picture.path.clone());
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The form's place in the application.
// ---------------------------------------------------------------------------------------------

/// The form, while it shows, and what the window keeps for it.
pub struct GeorefUi {
    form: Option<Form>,
    focus: FocusHandle,
    output_scroll: ScrollHandle,
}

impl GeorefUi {
    /// `httpserver.georefkml` and `georefimagepath`, for the server: the KML the last Pre-process
    /// handed `httpGeoRefKML`, and the photo folder with its separator.
    /// `// C#: GeoRef/georefimage.cs:252-256`
    #[must_use]
    pub fn http_kml(&self) -> Option<(&str, String)> {
        let form = self.form.as_ref()?;
        let kml = form.kml.as_deref()?;
        Some((
            kml,
            format!("{}{}", form.text(Field::JpgDir), std::path::MAIN_SEPARATOR),
        ))
    }

    /// Closed.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            form: None,
            focus: cx.focus_handle(),
            output_scroll: ScrollHandle::new(),
        }
    }

    /// Once a frame: a run's lines and its end, and the map's fit once it can be made. A dialog
    /// or message box over the form has the keyboard while it shows: `ShowDialog` is modal.
    pub fn tick(&mut self, window: &mut Window, cx: &mut Context<MissionPlanner>) {
        let Some(form) = self.form.as_mut() else {
            return;
        };
        form.poll();
        form.fit_map();
        if form.scroll_to_end {
            form.scroll_to_end = false;
            self.output_scroll.scroll_to_bottom();
        }
        if form.prompt.is_some() && !self.focus.is_focused(window) {
            self.focus.focus(window, cx);
        }
    }
}

/// `new Georefimage().Show()`: the form, over the flight screen. This window shows one: a second
/// click brings back the one showing rather than a second window beside it.
/// `// C#: GCSViews/FlightData.cs:5933-5936; GeoRef/georefimage.cs:26-47`
pub fn open(this: &mut MissionPlanner) {
    if this.georef.form.is_some() {
        return;
    }
    let form = Form::new(today(), Arc::new(PlannerTerrain));
    let source = this.map.borrow().source_id();
    use_imagery(&form.map, source);
    this.georef.form = Some(form);
}

/// The close box. The C#'s form cannot be closed while it is busy in a handler, nor can this.
fn close(this: &mut MissionPlanner) {
    if this.georef.form.as_ref().is_some_and(Form::inert) {
        return;
    }
    this.georef.form = None;
}

/// A button's run started, and the window redrawn.
fn run(this: &mut MissionPlanner, job: Option<Job>) {
    if let (Some(form), Some(job)) = (this.georef.form.as_mut(), job) {
        form.start(job);
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing the form.
// ---------------------------------------------------------------------------------------------

fn control_id(name: &str) -> SharedString {
    SharedString::from(format!("georef-{name}"))
}

/// A control's box at its place in the client area.
fn placed(name: &str) -> Div {
    let (x, y, w, h) = absolute(name);
    div().absolute().left(px(x)).top(px(y)).w(px(w)).h(px(h))
}

/// A `Label`, dimmed with its panel. `label11` and `label12` are 10 pt bold.
fn label(form: &Form, name: &str) -> AnyElement {
    let bold = matches!(name, "label11" | "label12");
    let mut body = placed(name)
        .whitespace_nowrap()
        .text_size(px(if bold { 13.0 } else { FONT }))
        .text_color(rgb(if form.enabled(name) {
            theme::TEXT
        } else {
            theme::DIM
        }))
        .child(text_of(name));
    if bold {
        body = body.font_weight(gpui::FontWeight::BOLD);
    }
    body.into_any_element()
}

/// A bordered `Panel`.
fn panel(name: &str) -> AnyElement {
    placed(name)
        .border_1()
        .border_color(rgb(theme::BORDER))
        .into_any_element()
}

/// A text box's element: what it holds, the caret while it takes the keyboard.
fn text_box(
    form: &Form,
    field: Field,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let name = field.name();
    let enabled = form.enabled(name);
    let editing = form.editing() == Some(Target::Text(field));
    let focused = editing && focus.is_focused(window);
    let shown = form.text(field).to_owned();
    let mut body = crate::probe::measured(format!("georef-{name}"), placed(name))
        .id(control_id(name))
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(if enabled { theme::BG } else { theme::PANEL }))
        .text_size(px(FONT))
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(if shown.is_empty() {
            " ".to_owned()
        } else {
            shown
        })
        .children(focused.then(|| div().w(px(1.0)).h(px(11.0)).bg(rgb(theme::ACCENT))));
    if enabled {
        body = body.cursor_text().on_click(cx.listener(
            move |this, _event: &gpui::ClickEvent, window, cx| {
                if let Some(form) = this.georef.form.as_mut() {
                    form.start_edit(Target::Text(field));
                }
                this.georef.focus.focus(window, cx);
                cx.notify();
            },
        ));
    }
    if editing {
        body = body.track_focus(focus).on_key_down(cx.listener(key));
    }
    body.into_any_element()
}

/// A key while the form has the keyboard.
fn key(
    this: &mut MissionPlanner,
    event: &gpui::KeyDownEvent,
    _window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    if let Some(form) = this.georef.form.as_mut()
        && form.key(event)
    {
        cx.notify();
    }
}

/// A `NumericUpDown`: its value, or what is being typed, and its two arrows.
fn numeric(
    form: &Form,
    num: Num,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let name = num.name();
    let editing = form.editing() == Some(Target::Num(num));
    let focused = editing && focus.is_focused(window);
    let arrow = |up: bool, cx: &mut Context<MissionPlanner>| {
        let id = format!("{name}-{}", if up { "up" } else { "down" });
        crate::probe::measured(format!("georef-{id}"), div())
            .id(control_id(&id))
            .h(px(9.0))
            .w(px(14.0))
            .flex()
            .items_center()
            .justify_center()
            .bg(rgb(theme::ACTION))
            .text_size(px(7.0))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .child(if up { "\u{25b2}" } else { "\u{25bc}" })
            .on_click(
                cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                    if let Some(form) = this.georef.form.as_mut() {
                        if up {
                            form.up(num);
                        } else {
                            form.down(num);
                        }
                    }
                    cx.notify();
                }),
            )
    };
    let text = crate::probe::measured(format!("georef-{name}"), div())
        .id(control_id(name))
        .flex_1()
        .h_full()
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .cursor_text()
        .child(form.num_text(num))
        .children(focused.then(|| div().w(px(1.0)).h(px(11.0)).bg(rgb(theme::ACCENT))))
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, window, cx| {
                if let Some(form) = this.georef.form.as_mut() {
                    form.start_edit(Target::Num(num));
                }
                this.georef.focus.focus(window, cx);
                cx.notify();
            }),
        );
    let mut body = placed(name)
        .flex()
        .border_1()
        .border_color(rgb(if focused {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::BG))
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .child(text)
        .child(
            div()
                .flex()
                .flex_col()
                .justify_between()
                .child(arrow(true, cx))
                .child(arrow(false, cx)),
        );
    if editing {
        body = body.track_focus(focus).on_key_down(cx.listener(key));
    }
    body.into_any_element()
}

/// A box and its text, for a `CheckBox` or, round, a `RadioButton`.
fn tick(name: &str, checked: bool, round: bool) -> gpui::Stateful<Div> {
    let mark = div()
        .w(px(11.0))
        .h(px(11.0))
        .flex_shrink_0()
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(theme::TEXT))
        .bg(rgb(theme::BG))
        .text_size(px(9.0))
        .text_color(rgb(theme::ACCENT))
        .children(checked.then_some(if round { "\u{25cf}" } else { "\u{2713}" }));
    let mark = if round { mark.rounded_full() } else { mark };
    crate::probe::measured(format!("georef-{name}"), placed(name))
        .id(control_id(name))
        .flex()
        .items_center()
        .gap_1()
        .whitespace_nowrap()
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .child(mark)
        .child(text_of(name))
}

/// A `CheckBox`.
fn checkbox(form: &Form, check: Check, cx: &mut Context<MissionPlanner>) -> AnyElement {
    tick(check.name(), form.check(check), false)
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                if let Some(form) = this.georef.form.as_mut() {
                    form.click_check(check);
                }
                cx.notify();
            }),
        )
        .into_any_element()
}

/// A `RadioButton`.
fn radio(form: &Form, radio: Radio, cx: &mut Context<MissionPlanner>) -> AnyElement {
    tick(radio.name(), form.radio(radio), true)
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                if let Some(form) = this.georef.form.as_mut() {
                    form.click_radio(radio);
                }
                cx.notify();
            }),
        )
        .into_any_element()
}

/// A `MyButton`, dimmed while it is not `Enabled`.
fn button(
    form: &Form,
    name: &str,
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let body = crate::probe::measured(format!("georef-{name}"), placed(name))
        .id(control_id(name))
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .rounded_sm()
        .text_center()
        .text_size(px(FONT))
        .child(text_of(name));
    if !form.enabled(name) {
        return body
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::PANEL))
            .text_color(rgb(theme::DIM))
            .into_any_element();
    }
    body.border_color(rgb(theme::ACCENT))
        .bg(rgb(theme::ACTION))
        .text_color(rgb(theme::ACCENT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                on_click(this);
                cx.notify();
            }),
        )
        .into_any_element()
}

/// `TXT_outputlog`: `ReadOnly`, multi-line, word-wrapped, scrolled to its end as lines arrive.
fn output_box(form: &Form, scroll: &ScrollHandle) -> AnyElement {
    let lines = form.output().split('\n').map(|line| {
        div().child(if line.is_empty() {
            " ".to_owned()
        } else {
            line.to_owned()
        })
    });
    crate::probe::measured("georef-TXT_outputlog", placed("TXT_outputlog"))
        .id(control_id("TXT_outputlog"))
        .overflow_y_scroll()
        .track_scroll(scroll)
        .p_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .children(lines)
        .into_any_element()
}

/// `pictureBox1`, dimmed: the photo a pin's click would show is not drawn.
fn picture_box(form: &Form) -> AnyElement {
    let note = form.image_location.as_ref().map_or_else(
        || "pictureBox1: the photo a pin's click shows is not ported".to_owned(),
        |path| format!("pictureBox1: {path}"),
    );
    crate::probe::measured("georef-pictureBox1", placed("pictureBox1"))
        .p_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .text_size(px(FONT))
        .text_color(rgb(theme::DIM))
        .child(note)
        .into_any_element()
}

/// `myGMAP1`: the imagery, and over it the route, the markers, and the tooltips and footprints
/// of the markers under the pointer. Drag pans, the wheel zooms, a click is a marker's.
fn map_view(form: &Form, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let map = Rc::clone(&form.map);
    let wheel_map = Rc::clone(&form.map);
    let overlay_map = Rc::clone(&form.map);
    let contents = Rc::clone(&form.contents);
    let hovered = form.hovered.clone();
    crate::probe::measured("georef-myGMAP1", placed("myGMAP1"))
        .id(control_id("myGMAP1"))
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                let Some(form) = this.georef.form.as_mut() else {
                    return;
                };
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                form.commit_edit();
                form.press = Some((x, y, false));
                form.map.borrow_mut().begin_drag(x, y);
                cx.notify();
            }),
        )
        .on_mouse_move(
            cx.listener(|this, event: &gpui::MouseMoveEvent, window, cx| {
                let Some(form) = this.georef.form.as_mut() else {
                    return;
                };
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                if event.pressed_button == Some(MouseButton::Left) {
                    if let Some((from_x, from_y, moved)) = form.press.as_mut()
                        && ((x - *from_x).abs() > 2.0 || (y - *from_y).abs() > 2.0)
                    {
                        *moved = true;
                    }
                    form.map.borrow_mut().drag_to(x, y);
                    window.refresh();
                    return;
                }
                if form.hover(Some((x, y))) {
                    cx.notify();
                }
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseUpEvent, window, cx| {
                let Some(form) = this.georef.form.as_mut() else {
                    return;
                };
                form.map.borrow_mut().end_drag();
                let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
                if let Some((_, _, false)) = form.press.take() {
                    form.map_click((x, y));
                }
                window.refresh();
                cx.notify();
            }),
        )
        .on_scroll_wheel(move |event, window, _cx| {
            let delta = event.delta.pixel_delta(px(20.0));
            wheel_map.borrow_mut().zoom(
                f32::from(event.position.x),
                f32::from(event.position.y),
                f32::from(delta.y) / 20.0,
            );
            window.refresh();
        })
        .child(crate::mapview::map_element(map))
        .child(
            canvas(
                |_bounds, _window, _cx| (),
                move |_bounds, (), window, cx| {
                    let map = overlay_map.borrow();
                    paint_overlay(&map, &contents, &hovered, window, cx);
                },
            )
            .absolute()
            .size_full(),
        )
        .into_any_element()
}

/// `GMapRoute.DefaultStroke`: `Color.FromArgb(144, Color.MidnightBlue)`, five pixels.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapRoute.cs:249-267`
const ROUTE: (u32, f32) = (0x19_19_70_90, 5.0);
/// `camera_icon_G`'s green disc, and the white camera on it.
/// `// C#: Resources/camera-icon-G.png`
const CAMERA_GREEN: u32 = 0x5d_db_38;
/// `Pens.Crimson`, the footprint's outline. `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs:61-62`
const CRIMSON: u32 = 0xdc_14_3c;
/// The pins' dark edge, as `green.png` has it.
/// `// C#: ExtLibs/GMap.NET.Drawing/Resources/green.png`
const PIN_EDGE: u32 = 0x04_00_01;
/// `GMapToolTip`'s font size, stroke, fill and text colour.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapToolTip.cs:44-111`
const TOOLTIP: (f32, u32, u32, u32) = (14.0, 0x19_19_70_8c, 0xf0_f8_ff_de, 0x00_00_80);

fn solid(colour: u32) -> Hsla {
    Hsla::from(rgb(colour))
}

/// A closed polygon filled, from points relative to `at`.
fn fill_polygon(window: &mut Window, at: Point<Pixels>, points: &[(f32, f32)], colour: u32) {
    let mut builder = PathBuilder::fill();
    let mut points = points.iter();
    let Some((x, y)) = points.next() else {
        return;
    };
    builder.move_to(point(at.x + px(*x), at.y + px(*y)));
    for (x, y) in points {
        builder.line_to(point(at.x + px(*x), at.y + px(*y)));
    }
    builder.close();
    if let Ok(path) = builder.build() {
        window.paint_path(path, solid(colour));
    }
}

/// A filled rectangle or disc.
fn fill_box(window: &mut Window, origin: (f32, f32), extent: (f32, f32), round: f32, colour: u32) {
    window.paint_quad(quad(
        Bounds {
            origin: point(px(origin.0), px(origin.1)),
            size: size(px(extent.0), px(extent.1)),
        },
        Corners::all(px(round)),
        rgb(colour),
        Edges::default(),
        rgb(colour),
        BorderStyle::default(),
    ));
}

/// Paints what `myGMAP1.Overlays[0]` holds over the map, in GMap's order: the route, the
/// markers (a camera marker's footprint over it while the pointer is), then the tooltips.
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/GMapOverlay.cs (OnRender); ExtLibs/Maps/GMapMarkerPhoto.cs:66-78`
fn paint_overlay(
    map: &MapViewport,
    contents: &MapContents,
    hovered: &[Marker],
    window: &mut Window,
    cx: &mut App,
) {
    let screen: Vec<(f32, f32)> = contents
        .route
        .iter()
        .filter_map(|position| map.screen_of(*position))
        .collect();
    if screen.len() > 1 {
        let mut builder = PathBuilder::stroke(px(ROUTE.1));
        for (index, (x, y)) in screen.iter().enumerate() {
            if index == 0 {
                builder.move_to(point(px(*x), px(*y)));
            } else {
                builder.line_to(point(px(*x), px(*y)));
            }
        }
        if let Ok(path) = builder.build() {
            window.paint_path(path, Hsla::from(gpui::rgba(ROUTE.0)));
        }
    }
    for (index, cam) in contents.cams.iter().enumerate() {
        let Some((x, y)) = map.screen_of(cam.position) else {
            continue;
        };
        paint_camera(window, x, y);
        if hovered.contains(&Marker::Cam(index)) {
            let corners: Vec<(f32, f32)> = cam
                .footprint
                .iter()
                .filter_map(|corner| map.screen_of(*corner))
                .collect();
            if corners.len() > 1 {
                let mut builder = PathBuilder::stroke(px(1.0));
                for (at, (cx_, cy_)) in corners.iter().enumerate() {
                    if at == 0 {
                        builder.move_to(point(px(*cx_), px(*cy_)));
                    } else {
                        builder.line_to(point(px(*cx_), px(*cy_)));
                    }
                }
                builder.close();
                if let Ok(path) = builder.build() {
                    window.paint_path(path, solid(CRIMSON));
                }
            }
        }
    }
    for photo in &contents.photos {
        if let Some((x, y)) = map.screen_of(photo.position) {
            paint_green_pin(window, point(px(x), px(y)));
        }
    }
    for marker in hovered {
        let position = match marker {
            Marker::Cam(index) => contents.cams.get(*index).map(|cam| cam.position),
            Marker::Photo(index) => contents.photos.get(*index).map(|photo| photo.position),
        };
        if let (Some(position), Some(text)) = (position, tooltip(contents, *marker))
            && let Some((x, y)) = map.screen_of(position)
        {
            paint_tooltip(window, cx, point(px(x), px(y)), text);
        }
    }
}

/// `camera_icon_G` at 20 by 20, centred on its point: a green disc with a white camera on it.
/// `// C#: ExtLibs/Maps/GMapMarkerPhoto.cs:15, 41, 66-72; Resources/camera-icon-G.png`
fn paint_camera(window: &mut Window, x: f32, y: f32) {
    let (left, top) = (x - 10.0, y - 10.0);
    fill_box(window, (left, top), (20.0, 20.0), 10.0, CAMERA_GREEN);
    fill_box(window, (left + 7.0, top + 5.0), (6.0, 2.0), 0.5, 0xff_ff_ff);
    fill_box(
        window,
        (left + 5.0, top + 7.0),
        (10.0, 8.0),
        1.0,
        0xff_ff_ff,
    );
    fill_box(
        window,
        (left + 7.0, top + 8.0),
        (6.0, 6.0),
        3.0,
        CAMERA_GREEN,
    );
    fill_box(window, (left + 8.5, top + 9.5), (3.0, 3.0), 1.5, 0xff_ff_ff);
}

/// `GMarkerGoogleType.green`: the pin with its point on the position, as the flight map draws
/// its green pins. `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/Markers/GMarkerGoogle.cs:111-127`
fn paint_green_pin(window: &mut Window, at: Point<Pixels>) {
    use crate::mapview::{PIN_GREEN, PIN_RADIUS, PIN_STALK, pin_outline};
    fill_polygon(
        window,
        at,
        &pin_outline(PIN_RADIUS, PIN_STALK + 1.0),
        PIN_EDGE,
    );
    fill_box(
        window,
        (f32::from(at.x) - 1.5, f32::from(at.y) + PIN_STALK),
        (3.0, -PIN_STALK),
        0.0,
        PIN_EDGE,
    );
    fill_polygon(
        window,
        at,
        &pin_outline(PIN_RADIUS - 1.5, PIN_STALK - 0.5),
        PIN_GREEN,
    );
}

/// A marker's tooltip as `GMapRoundedToolTip` draws it: its lines centred in a rounded box
/// padded by ten, 14 right of and 44 above the point with its bottom there, and a line to it.
/// `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/ToolTips/GMapRoundedToolTip.cs:16-64; ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapToolTip.cs:84-98`
fn paint_tooltip(window: &mut Window, cx: &mut App, at: Point<Pixels>, text: &str) {
    const RADIUS: f32 = 10.0;
    const OFFSET: (f32, f32) = (14.0, -44.0);
    let (font_size, stroke, fill, colour) = TOOLTIP;
    let mut font = window.text_style().font();
    font.weight = gpui::FontWeight::BOLD;
    let lines: Vec<gpui::ShapedLine> = text
        .split('\n')
        .map(|line| {
            let run = TextRun {
                len: line.len(),
                font: font.clone(),
                color: solid(colour),
                background_color: None,
                underline: None,
                strikethrough: None,
            };
            window.text_system().shape_line(
                SharedString::from(line.to_owned()),
                px(font_size),
                &[run],
                None,
            )
        })
        .collect();
    let line_height = (font_size * 1.2).floor();
    let text_width = lines
        .iter()
        .map(|line| f32::from(line.width).ceil())
        .fold(0.0, f32::max);
    #[allow(clippy::cast_precision_loss)] // a handful of lines
    let text_height = line_height * lines.len() as f32;
    let width = text_width + RADIUS * 2.0;
    let height = text_height + RADIUS;
    let left = at.x + px(OFFSET.0);
    let top = at.y - px(text_height) + px(OFFSET.1);
    let mut leader = PathBuilder::stroke(px(2.0));
    leader.move_to(at);
    leader.line_to(point(
        left + px(RADIUS / 2.0),
        top + px(height - RADIUS / 2.0),
    ));
    if let Ok(path) = leader.build() {
        window.paint_path(path, Hsla::from(gpui::rgba(stroke)));
    }
    window.paint_quad(quad(
        Bounds {
            origin: point(left, top),
            size: size(px(width), px(height)),
        },
        Corners::all(px(RADIUS)),
        gpui::rgba(fill),
        Edges::all(px(2.0)),
        gpui::rgba(stroke),
        BorderStyle::default(),
    ));
    for (index, line) in lines.iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let row = index as f32;
        let origin = point(
            left + px((width - f32::from(line.width).ceil()) / 2.0),
            top + px((height - text_height) / 2.0 + row * line_height),
        );
        let _ = line.paint(origin, px(line_height), TextAlign::Left, None, window, cx);
    }
}

/// The dialog or message box over the form.
fn prompt_box(
    form: &Form,
    window: &Window,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let prompt = form.prompt.as_ref()?;
    let (title, text, field) = match prompt {
        Prompt::Log(field) => ("Open", LOG_FILTER.to_owned(), Some(field)),
        Prompt::Folder(field) => ("Browse For Folder", String::new(), Some(field)),
        Prompt::Message(caption, text) => (*caption, text.clone(), None),
    };
    let ok = crate::ui::action(
        "georef-prompt-ok",
        "OK",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            if let Some(form) = this.georef.form.as_mut() {
                form.answer_prompt(true);
            }
            cx.notify();
        }),
    );
    let mut buttons = div().flex().justify_end().gap_2().child(ok);
    if field.is_some() {
        buttons = buttons.child(crate::ui::action(
            "georef-prompt-cancel",
            "Cancel",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                if let Some(form) = this.georef.form.as_mut() {
                    form.answer_prompt(false);
                }
                cx.notify();
            }),
        ));
    }
    let mut body = crate::probe::measured("georef-prompt", div())
        .id("georef-prompt")
        .flex()
        .flex_col()
        .gap_2()
        .w(px(460.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .occlude()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(title.to_owned()),
        );
    if !text.is_empty() {
        body = body.child(div().text_sm().text_color(rgb(theme::TEXT)).child(text));
    }
    let listen = field.is_none();
    if let Some(field) = field {
        let focused = focus.is_focused(window);
        body = body.child(
            crate::probe::measured("georef-prompt-field", div())
                .id("georef-prompt-field")
                .track_focus(focus)
                .on_key_down(cx.listener(key))
                .px_2()
                .py_1()
                .border_1()
                .border_color(rgb(if focused {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(theme::ACTION))
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(field.value().to_owned()),
        );
    }
    let mut body = body.child(buttons);
    if listen {
        body = body.track_focus(focus).on_key_down(cx.listener(key));
    }
    Some(
        div()
            .absolute()
            .left(px(0.0))
            .top(px(0.0))
            .size_full()
            .flex()
            .items_center()
            .justify_center()
            .child(body)
            .into_any_element(),
    )
}

/// The form over the flight screen, while it shows: its title and close box, and its client
/// area at the `.resx`'s size with every control at its place.
pub fn window(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = this.georef.form.as_ref()?;
    let focus = &this.georef.focus;
    let mut client = div()
        .relative()
        .w(px(CLIENT_SIZE.0))
        .h(px(CLIENT_SIZE.1))
        .bg(rgb(theme::PANEL));
    for name in BORDERED {
        client = client.child(panel(name));
    }
    for name in [
        "label1",
        "label27",
        "lbldrpstart",
        "label2",
        "label3",
        "label28",
        "label7",
        "label9",
        "label8",
        "label11",
        "label12",
    ] {
        client = client.child(label(form, name));
    }
    for field in Field::ALL {
        client = client.child(text_box(form, field, window, focus, cx));
    }
    for num in Num::ALL {
        client = client.child(numeric(form, num, window, focus, cx));
    }
    for check in Check::ALL {
        client = client.child(checkbox(form, check, cx));
    }
    for which in Radio::ALL {
        client = client.child(radio(form, which, cx));
    }
    client = client
        .child(button(
            form,
            "BUT_browselog",
            |this| {
                let start = log_directory().map_or_else(String::new, |dir| {
                    format!("{}{}", dir.display(), std::path::MAIN_SEPARATOR)
                });
                if let Some(form) = this.georef.form.as_mut() {
                    form.browse_log(&start);
                }
            },
            cx,
        ))
        .child(button(
            form,
            "BUT_browsedir",
            |this| {
                if let Some(form) = this.georef.form.as_mut() {
                    form.browse_dir();
                }
            },
            cx,
        ))
        .child(button(
            form,
            "BUT_estoffset",
            |this| {
                let job = this.georef.form.as_mut().and_then(Form::estimate_click);
                run(this, job);
            },
            cx,
        ))
        .child(button(
            form,
            "BUT_doit",
            |this| {
                let job = this.georef.form.as_mut().and_then(Form::doit_click);
                run(this, job);
            },
            cx,
        ))
        // Location Kml: `Process.Start` of `m3u/GeoRefnetworklink.kml`, a network link to the
        // built-in web server. `// C#: GeoRef/georefimage.cs:321-325`
        .child(button(
            form,
            "BUT_networklinkgeoref",
            MissionPlanner::georef_network_link,
            cx,
        ))
        .child(button(
            form,
            "BUT_Geotagimages",
            |this| {
                let job = this.georef.form.as_mut().and_then(Form::geotag_click);
                run(this, job);
            },
            cx,
        ))
        .child(output_box(form, &this.georef.output_scroll))
        .child(map_view(form, cx))
        .child(picture_box(form))
        .children(prompt_box(form, window, focus, cx));

    let title = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_sm().text_color(rgb(theme::TEXT)).child(TITLE))
        .child(
            crate::probe::measured("georef-close", div())
                .id("georef-close")
                .px_2()
                .cursor_pointer()
                .text_color(rgb(theme::DIM))
                .hover(|style| style.text_color(rgb(theme::ALERT)))
                .child("\u{2715}")
                .on_click(cx.listener(|this, _event: &gpui::ClickEvent, _window, cx| {
                    close(this);
                    cx.notify();
                })),
        );
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(point(px(WINDOW_AT.0), px(WINDOW_AT.1)))
                .child(
                    crate::probe::measured("georef", div())
                        .id("georef")
                        .flex()
                        .flex_col()
                        .bg(rgb(theme::PANEL))
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .rounded_md()
                        .occlude()
                        .child(title)
                        .child(client),
                ),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

// ---------------------------------------------------------------------------------------------
// The facts.
// ---------------------------------------------------------------------------------------------

/// The controls whose `Enabled` the handlers change, or which are drawn dimmed, as facts.
const ENABLED_FACTS: [&str; 7] = [
    "PANEL_TIME_OFFSET",
    "PANEL_SHUTTER_LAG",
    "txt_basealt",
    "BUT_estoffset",
    "BUT_doit",
    "BUT_Geotagimages",
    "BUT_networklinkgeoref",
];

impl MissionPlanner {
    /// `BUT_networklinkgeoref_Click`: `Process.Start` of `m3u/GeoRefnetworklink.kml` beside the
    /// program - Mission Planner's network link to `/georefnetwork.kml` - or, when it is not
    /// beside this program, the same file written under the data directory; what the desktop
    /// could not open goes on the status line.
    /// `// C#: GeoRef/georefimage.cs:321-325; m3u/GeoRefnetworklink.kml`
    pub(crate) fn georef_network_link(&mut self) {
        let beside = crate::help::install_dir()
            .join("m3u")
            .join("GeoRefnetworklink.kml");
        let path = if beside.is_file() {
            beside
        } else {
            let Some(data) = mp_settings::data_directory() else {
                self.file_status =
                    Some("no data directory to write the network link to".to_owned());
                return;
            };
            let written = data.join("m3u").join("GeoRefnetworklink.kml");
            if let Some(dir) = written.parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            if let Err(why) = std::fs::write(&written, crate::http_server::GEOREF_NETWORK_LINK_KML)
            {
                self.file_status = Some(format!("could not write {}: {why}", written.display()));
                return;
            }
            written
        };
        match crate::scripts_tab::open_with_shell(&path) {
            Ok(()) => {
                if let Some(form) = self.georef.form.as_mut() {
                    form.network_link = Some(path.display().to_string());
                }
            }
            Err(why) => {
                self.file_status = Some(format!("could not open {}: {why}", path.display()));
            }
        }
    }
}

/// Facts a UI test asserts on: whether the form shows, every control as it reads and whether it
/// is enabled, the text box and its lines, the run under way and the last, the offset, the
/// report files by name and size, the geotagged copies, what `GeoRefImageBase` holds, what the
/// map draws and what is under the pointer, the picture box's photo and the box over the form.
pub fn record_facts(georef: &GeorefUi) {
    use crate::facts::record;
    let Some(form) = georef.form.as_ref() else {
        record("fly.georef", "closed");
        return;
    };
    record("fly.georef", "open");
    for field in Field::ALL {
        record(format!("fly.georef.{}", field.name()), form.text(field));
    }
    for num in Num::ALL {
        record(format!("fly.georef.{}", num.name()), form.num_text(num));
    }
    for check in Check::ALL {
        record(
            format!("fly.georef.{}", check.name()),
            bool_text(form.check(check)),
        );
    }
    for which in Radio::ALL {
        record(
            format!("fly.georef.{}", which.name()),
            bool_text(form.radio(which)),
        );
    }
    for name in ENABLED_FACTS {
        record(
            format!("fly.georef.{name}.enabled"),
            bool_text(form.enabled(name)),
        );
    }
    record("fly.georef.mode", mode_name(form.mode()));
    record("fly.georef.TXT_outputlog", form.output());
    record("fly.georef.lines", form.output().lines().count());
    record(
        "fly.georef.running",
        form.running().map_or("none", Kind::name),
    );
    record("fly.georef.last", form.last.map_or("none", Kind::name));
    record(
        "fly.georef.offset",
        form.offset
            .map_or_else(|| "none".to_owned(), mp_log::netfmt::double),
    );
    record(
        "fly.georef.files",
        form.files.as_ref().map_or_else(
            || "none".to_owned(),
            |files| {
                files
                    .iter()
                    .map(|(name, size)| format!("{name}:{size}"))
                    .collect::<Vec<_>>()
                    .join(",")
            },
        ),
    );
    record("fly.georef.kml", form.kml.as_ref().map_or(0, String::len));
    record(
        "fly.georef.networklink",
        form.network_link.as_deref().unwrap_or("none"),
    );
    record(
        "fly.georef.geotagged",
        form.geotagged
            .map_or_else(|| "none".to_owned(), |count| count.to_string()),
    );
    if let Some(state) = form.georef() {
        record("fly.georef.positions", state.vehicle_locations.len());
        record("fly.georef.cams", state.cam_locations.len());
        record(
            "fly.georef.pictures",
            state
                .pictures_info
                .as_ref()
                .map_or_else(|| "null".to_owned(), |pictures| pictures.len().to_string()),
        );
        record("fly.georef.useAMSLAlt", bool_text(state.use_amsl_alt));
        record("fly.georef.millisShutterLag", state.millis_shutter_lag);
        record(
            "fly.georef.minshutter",
            mp_log::netfmt::double(state.minshutter),
        );
    }
    let contents = form.contents();
    record("fly.georef.map.route", contents.route.len());
    record("fly.georef.map.cams", contents.cams.len());
    record("fly.georef.map.photos", contents.photos.len());
    record(
        "fly.georef.map.hover",
        form.hovered
            .iter()
            .filter_map(|marker| tooltip(contents, *marker))
            .collect::<Vec<_>>()
            .join(" | "),
    );
    record(
        "fly.georef.pictureBox1",
        form.image_location.as_deref().unwrap_or("none"),
    );
    record(
        "fly.georef.prompt",
        form.prompt().map_or("none", Prompt::name),
    );
    record(
        "fly.georef.prompt.text",
        form.prompt().map_or("", Prompt::text),
    );
    record(
        "fly.georef.editing",
        match form.editing() {
            Some(Target::Text(field)) => field.name(),
            Some(Target::Num(num)) => num.name(),
            None => "none",
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_georef::{Location, OrderedMap, PictureInformation};
    use std::path::{Path, PathBuf};
    use std::time::{Duration, Instant};

    fn testdata(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    fn csharp(path: &str) -> Option<String> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../references/missionplanner")
            .join(path);
        std::fs::read_to_string(path).ok()
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "headless-planner-georef-ui-{name}-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    struct Offline;

    impl mp_terrain::Http for Offline {
        fn get(&self, _url: &str) -> Result<Vec<u8>, mp_terrain::HttpError> {
            Err(mp_terrain::HttpError("offline".to_owned()))
        }
    }

    /// The recorded flight's log and 25 photos in one fresh folder, as Browse Log leaves the two
    /// boxes, and the S28E153 tile the oracle's footprints were projected onto.
    fn setup(name: &str) -> (String, String, Arc<dyn Terrain + Send + Sync>) {
        let dir = scratch(name);
        let photos = dir.join("logs");
        std::fs::create_dir_all(&photos).unwrap();
        for entry in std::fs::read_dir(testdata("georef/photos")).unwrap() {
            let path = entry.unwrap().path();
            std::fs::copy(&path, photos.join(path.file_name().unwrap())).unwrap();
        }
        let log = photos.join("camera.bin");
        std::fs::copy(testdata("georef/camera.bin"), &log).unwrap();
        let srtm = dir.join("srtm");
        std::fs::create_dir_all(&srtm).unwrap();
        let zip = std::fs::read(testdata("srtm/S28E153.hgt.zip")).unwrap();
        mp_log::zip::extract(&zip, &srtm).unwrap();
        let terrain = mp_terrain::Srtm::without_thread(&srtm, Arc::new(Offline));
        (
            photos.to_str().unwrap().to_owned(),
            log.to_str().unwrap().to_owned(),
            Arc::new(terrain),
        )
    }

    fn the_day() -> DateTime {
        DateTime::from_parts(2026, 9, 24, 0, 0, 0, TimeKind::Local).unwrap()
    }

    fn flat() -> Form {
        Form::new(the_day(), Arc::new(mp_georef::Flat))
    }

    /// A button's run as the form starts it, on its thread, polled until it is back.
    fn finish(form: &mut Form, job: Job) {
        assert!(form.start(job));
        assert!(form.running().is_some());
        let deadline = Instant::now() + Duration::from_secs(300);
        while form.running().is_some() {
            form.poll();
            assert!(Instant::now() < deadline, "the run never finished");
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    /// `<data name="...">` and its `<value>`, unescaped.
    fn resx_data(resx: &str) -> Vec<(String, String)> {
        let unescape = |text: &str| {
            text.replace("&gt;", ">")
                .replace("&lt;", "<")
                .replace("&amp;", "&")
        };
        let mut out = Vec::new();
        let mut rest = resx;
        while let Some(at) = rest.find("<data name=\"") {
            rest = &rest[at + 12..];
            let name_end = rest.find('"').unwrap();
            let name = unescape(&rest[..name_end]);
            let end = rest.find("</data>").unwrap_or(rest.len());
            let body = &rest[..end];
            if let (Some(start), Some(stop)) = (body.find("<value>"), body.find("</value>")) {
                out.push((name, unescape(&body[start + 7..stop])));
            }
            rest = &rest[end..];
        }
        out
    }

    fn pair(text: &str) -> (f32, f32) {
        let (a, b) = text.split_once(',').unwrap();
        (a.trim().parse().unwrap(), b.trim().parse().unwrap())
    }

    /// Every control the `.resx` places is drawn at that place, in that panel, with that text;
    /// the form is the `.resx`'s size and title; the two controls it disables start disabled.
    #[test]
    fn the_controls_are_where_the_resx_puts_them() {
        let Some(resx) = csharp("GeoRef/georefimage.resx") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let data = resx_data(&resx);
        let get = |key: &str| data.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str());
        let mut located = 0;
        for (key, value) in &data {
            let Some(name) = key.strip_suffix(".Location") else {
                continue;
            };
            located += 1;
            let control = control_named(name).unwrap_or_else(|| panic!("{name} is not drawn"));
            let (x, y) = pair(value);
            let (w, h) = pair(get(&format!("{name}.Size")).unwrap());
            assert_eq!(control.place, (x, y, w, h), "{name}");
            let parent = get(&format!(">>{name}.Parent")).unwrap();
            let parent = if parent == "$this" { "" } else { parent };
            assert_eq!(control.parent, parent, "{name}");
            assert_eq!(
                control.text,
                get(&format!("{name}.Text")).unwrap_or(""),
                "{name}"
            );
        }
        assert_eq!(located, CONTROLS.len());
        assert_eq!(pair(get("$this.ClientSize").unwrap()), CLIENT_SIZE);
        assert_eq!(get("$this.Text"), Some(TITLE));
        assert_eq!(get("BUT_Geotagimages.Enabled"), Some("False"));
        assert_eq!(get("txt_basealt.Enabled"), Some("False"));
        let form = flat();
        assert!(!form.enabled("BUT_Geotagimages"));
        assert!(!form.enabled("txt_basealt"));
        for field in Field::ALL {
            assert_eq!(
                form.text(field),
                get(&format!("{}.Text", field.name())).unwrap_or(""),
                "{}",
                field.name()
            );
        }
        // A control in a bordered panel sits a pixel in from the panel's edge.
        assert_eq!(absolute("BUT_estoffset"), (257.0, 147.0, 75.0, 23.0));
        assert_eq!(absolute("chk_usegps2"), (459.0, 253.0, 76.0, 17.0));
        assert_eq!(absolute("BUT_doit"), (123.0, 305.0, 84.0, 40.0));
    }

    /// `this.X.Prop = new decimal(new int[] { lo, mid, hi, flags })`.
    fn designer_decimal(designer: &str, name: &str, prop: &str) -> Option<Decimal> {
        let key = format!("this.{name}.{prop} = new decimal(new int[] {{");
        let at = designer.find(&key)? + key.len();
        let rest = &designer[at..];
        let end = rest.find('}')?;
        let ints: Vec<i64> = rest[..end]
            .split(',')
            .map(|word| word.trim().parse().unwrap())
            .collect();
        let (low, flags) = (ints[0], ints[3]);
        let scale = u32::try_from((flags >> 16) & 0xff).unwrap();
        let low = i128::from(low);
        Some(Decimal::new(if flags < 0 { -low } else { low }, scale))
    }

    /// The numbers' ranges, steps, places and values, the check box and radio button the
    /// Designer checks, and its event wirings each with a handler here.
    #[test]
    fn the_numbers_boxes_and_wirings_are_the_designers() {
        let Some(designer) = csharp("GeoRef/Georefimage.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        for num in Num::ALL {
            let name = num.name();
            let spec = num.spec();
            let or = |prop: &str, default: i128| {
                designer_decimal(&designer, name, prop).unwrap_or(Decimal::new(default, 0))
            };
            assert_eq!(spec.minimum, or("Minimum", 0), "{name}");
            assert_eq!(spec.maximum, or("Maximum", 100), "{name}");
            assert_eq!(spec.increment, or("Increment", 1), "{name}");
            assert_eq!(spec.initial, or("Value", 0), "{name}");
            let places = designer
                .split(&format!("this.{name}.DecimalPlaces = "))
                .nth(1)
                .map_or(0, |rest| rest.split(';').next().unwrap().parse().unwrap());
            assert_eq!(spec.places, places, "{name}");
        }
        let form = flat();
        assert_eq!(form.num_text(Num::MinShutter), "0.50");
        assert_eq!(form.num_text(Num::CameraRotation), "90");
        for check in Check::ALL {
            let checked = designer.contains(&format!("this.{}.Checked = true;", check.name()));
            assert_eq!(form.check(check), checked, "{}", check.name());
        }
        for radio in Radio::ALL {
            let checked = designer.contains(&format!("this.{}.Checked = true;", radio.name()));
            assert_eq!(form.radio(radio), checked, "{}", radio.name());
        }
        assert!(designer.contains("this.TXT_outputlog.ReadOnly = true;"));

        let mut wired: Vec<(String, String, String)> = designer
            .lines()
            .filter_map(|line| {
                let (left, right) = line
                    .trim()
                    .split_once(" += new System.EventHandler(this.")?;
                let (control, event) = left.strip_prefix("this.")?.split_once('.')?;
                let handler = right.strip_suffix(");")?;
                Some((control.to_owned(), event.to_owned(), handler.to_owned()))
            })
            .collect();
        wired.sort();
        let mut ours: Vec<(String, String, String)> = WIRINGS
            .iter()
            .map(|(c, e, h, _)| ((*c).to_owned(), (*e).to_owned(), (*h).to_owned()))
            .collect();
        ours.sort();
        assert_eq!(wired, ours);
        let source = include_str!("georef_ui.rs");
        for (_, _, _, handler) in WIRINGS {
            assert!(source.contains(&format!("fn {handler}(")), "{handler}");
        }
    }

    /// Each button calls the crate with what its handler reads from the form, and only when the
    /// handler would: Estimate Offset in the time offset panel, Pre-process with a log and a
    /// folder that are there and an offset that is a number, GeoTag Images once enabled.
    #[test]
    fn each_button_calls_the_crate_as_its_handler_does() {
        let (dir, log, terrain) = setup("buttons");
        let mut form = Form::new(the_day(), terrain);
        assert_eq!(
            form.estimate_click(),
            None,
            "the time offset panel is disabled"
        );
        assert_eq!(form.doit_click(), None, "no log");
        assert_eq!(form.geotag_click(), None, "not until Pre-process");
        form.set_text(Field::LogFile, &log);
        form.set_text(Field::JpgDir, &dir);

        form.click_radio(Radio::TimeOffset);
        form.click_check(Check::CamMsg);
        form.output = "old".to_owned();
        assert_eq!(
            form.estimate_click(),
            Some(Job::Estimate {
                log: log.clone(),
                dir: dir.clone(),
                gps: "GPS",
                usecam: true,
            })
        );
        assert_eq!(form.output(), "", "the box is emptied first");
        form.click_check(Check::UseGps2);
        assert!(matches!(
            form.estimate_click(),
            Some(Job::Estimate { gps: "GPS2", .. })
        ));

        // A comma is no decimal point: the handler's line, and no call.
        form.set_text(Field::OffsetSeconds, "36003,3");
        assert_eq!(form.doit_click(), None);
        assert_eq!(
            form.output(),
            "Offset number not in correct format. Use . as decimal separator\n"
        );
        form.set_text(Field::OffsetSeconds, "36003.3");
        form.type_num(Num::DropFromStart, "2");
        form.up(Num::DropEnd);
        form.type_num(Num::Hfov, "1000");
        form.down(Num::CameraRotation);
        form.click_check(Check::CamUseGpsAlt);
        form.click_check(Check::AmslAltUse);
        form.set_text(Field::BaseAlt, "12.5");
        let job = form.doit_click();
        assert_eq!(
            job,
            Some(Job::Process {
                log: log.clone(),
                dir: dir.clone(),
                settings: FormSettings {
                    mode: ProcessingMode::TimeOffset,
                    offset_text: "36003.3".to_owned(),
                    use_gps2: true,
                    use_cam_messages: true,
                    drop_start: 2,
                    drop_end: 1,
                    camera_rotation: 89.0,
                    hfov: 900.0,
                    vfov: 130.0,
                    cam_use_gps_alt: true,
                    trig_use_gps_alt: false,
                    base_alt_text: "12.5".to_owned(),
                },
            })
        );
        assert_eq!(form.output(), "", "the box is emptied first");
        assert!(!form.enabled("BUT_doit"), "disabled until the run ends");
        assert_eq!(form.doit_click(), None);
        assert!(!form.georef().unwrap().use_amsl_alt);

        // A folder that is not there: nothing, the box as it was.
        let mut form = flat();
        form.set_text(Field::LogFile, &log);
        form.set_text(Field::JpgDir, &format!("{dir}/missing"));
        form.output = "kept".to_owned();
        assert_eq!(form.doit_click(), None);
        assert_eq!(form.output(), "kept");

        // GeoTag Images once Pre-process has enabled it, with the folder and the controls.
        form.geotag_enabled = true;
        form.click_radio(Radio::Trig);
        form.click_check(Check::TrigUseGpsAlt);
        let Some(Job::Geotag {
            dir: tagged,
            settings,
        }) = form.geotag_click()
        else {
            panic!("no call");
        };
        assert_eq!(tagged, format!("{dir}/missing"));
        assert_eq!(settings.mode, ProcessingMode::Trig);
        assert!(settings.trig_use_gps_alt);
    }

    /// The recorded flight through the form as a pilot drives it: Browse Log, Estimate Offset in
    /// the time offset panel, Pre-process in CAM mode, GeoTag Images - each on its thread - and the
    /// text box ends up holding `mp-georef`'s golden lines, the files Mission Planner writes.
    #[test]
    fn the_text_box_holds_the_recorded_flights_golden_lines() {
        let (dir, log, terrain) = setup("golden");
        let mut form = Form::new(the_day(), terrain);
        form.browse_log(&format!("{dir}/"));
        let Some(Prompt::Log(field)) = form.prompt.as_mut() else {
            panic!("no dialog");
        };
        field.set(format!("{dir}/camera.bin"));
        form.answer_prompt(true);
        assert!(form.prompt().is_none());
        assert_eq!(form.text(Field::LogFile), log);
        assert_eq!(form.text(Field::JpgDir), dir);

        form.click_radio(Radio::TimeOffset);
        let job = form.estimate_click().unwrap();
        finish(&mut form, job);
        let golden = testdata("georef/golden");
        assert_eq!(
            form.output(),
            std::fs::read_to_string(golden.join("estimate.txt")).unwrap()
        );
        // `offset.ToString(CultureInfo.InvariantCulture)`: fifteen digits of 36066.10310937499.
        assert_eq!(
            form.offset.map(mp_log::netfmt::double).as_deref(),
            Some("36066.103109375")
        );
        assert_eq!(form.last, Some(Kind::Estimate));

        form.click_radio(Radio::CamMsg);
        let job = form.doit_click().unwrap();
        finish(&mut form, job);
        let want = std::fs::read_to_string(golden.join("cam-amsl/messages.txt")).unwrap();
        let tagging = want.find("GeoTagging").unwrap();
        assert_eq!(form.output().replace(&dir, "{dir}"), want[..tagging]);
        let mut sizes = Vec::new();
        for name in [
            "loglocation.csv",
            "location.csv",
            "location.kml",
            "location.txt",
            "location.geo",
            "location.tel",
            "location.jxl",
            "location.gpx",
        ] {
            let bytes = std::fs::read(Path::new(&dir).join(name)).unwrap();
            assert_eq!(
                bytes,
                std::fs::read(golden.join("cam-amsl").join(name)).unwrap(),
                "{name}"
            );
            sizes.push((name.to_owned(), bytes.len() as u64));
        }
        assert_eq!(form.files, Some(sizes));
        assert_eq!(
            form.kml.as_deref().map(str::len),
            Some(
                std::fs::read(golden.join("cam-amsl/location.kml"))
                    .unwrap()
                    .len()
            )
        );
        assert!(form.enabled("BUT_doit") && form.enabled("BUT_Geotagimages"));
        let contents = form.contents();
        assert_eq!(
            (
                contents.route.len(),
                contents.cams.len(),
                contents.photos.len()
            ),
            (617, 24, 24)
        );
        assert_eq!(contents.photos[0].tooltip, "IMG_0001.jpg");
        assert!(contents.cams[0].tooltip.starts_with("Photo\nAlt: "));
        assert!(contents.cams[0].tooltip.ends_with("\nNo: 0\nRoll: 0.00"));
        assert_eq!(contents.cams[0].footprint.len(), 4);
        assert_eq!(form.fit.as_ref().map(Vec::len), Some(617));

        let job = form.geotag_click().unwrap();
        finish(&mut form, job);
        assert_eq!(form.output().replace(&dir, "{dir}"), want);
        assert_eq!(form.geotagged, Some(24));
        for n in [1, 13, 24] {
            let name = format!("IMG_{n:04}_geotag.jpg");
            assert_eq!(
                std::fs::read(Path::new(&dir).join("geotagged").join(&name)).unwrap(),
                std::fs::read(golden.join("cam-amsl/geotagged").join(&name)).unwrap(),
                "{name}"
            );
        }
        assert!(form.prompt().is_none());
    }

    /// While a run has `georef`, the form takes nothing, as the C#'s busy form takes nothing.
    #[test]
    fn the_form_takes_nothing_while_a_run_is_under_way() {
        let (dir, log, terrain) = setup("busy");
        let mut form = Form::new(the_day(), terrain);
        form.set_text(Field::LogFile, &log);
        form.set_text(Field::JpgDir, &dir);
        let job = form.doit_click().unwrap();
        assert!(form.start(job.clone()));
        assert!(!form.start(job), "one run at a time");
        form.click_check(Check::UseGps2);
        form.click_radio(Radio::Trig);
        form.up(Num::Hfov);
        form.browse_dir();
        assert!(!form.check(Check::UseGps2));
        assert!(form.radio(Radio::CamMsg));
        assert_eq!(form.num_text(Num::Hfov), "200");
        assert!(form.prompt().is_none());
        assert_eq!(form.geotag_click(), None);
        let deadline = Instant::now() + Duration::from_secs(300);
        while form.running().is_some() {
            form.poll();
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(form.georef().is_some());
        assert_eq!(form.last, Some(Kind::Process));
    }

    fn remembered(form: &mut Form) {
        let georef = form.georef.as_mut().unwrap();
        georef.vehicle_locations.set(1, Location::default());
        let mut pictures = OrderedMap::new();
        pictures.set("a.jpg".to_owned(), PictureInformation::default());
        georef.pictures_info = Some(pictures);
    }

    fn held(form: &Form) -> (usize, Option<usize>) {
        let georef = form.georef().unwrap();
        (
            georef.vehicle_locations.len(),
            georef.pictures_info.as_ref().map(OrderedMap::len),
        )
    }

    /// `forget_positions` on the changes the C# wires to it and on no other: the log box forgets
    /// the positions and the pictures and dims GeoTag Images, Use GPS2 and Use Cam Messages the
    /// positions only.
    #[test]
    fn forget_positions_on_the_changes_the_csharp_wires() {
        let mut form = flat();
        remembered(&mut form);
        form.geotag_enabled = true;
        form.set_text(Field::JpgDir, "elsewhere");
        form.set_text(Field::OffsetSeconds, "12");
        form.set_text(Field::BaseAlt, "3");
        form.set_text(Field::ShutterLag, "150");
        form.click_check(Check::CamUseGpsAlt);
        form.click_check(Check::TrigUseGpsAlt);
        form.click_check(Check::AmslAltUse);
        form.click_radio(Radio::TimeOffset);
        form.type_num(Num::MinShutter, "0.3");
        assert_eq!(held(&form), (1, Some(1)), "nothing else forgets");
        assert!(form.enabled("BUT_Geotagimages"));

        form.click_check(Check::UseGps2);
        assert_eq!(held(&form), (0, Some(1)));
        remembered(&mut form);
        form.click_check(Check::CamMsg);
        assert_eq!(held(&form), (0, Some(1)));

        remembered(&mut form);
        form.set_text(Field::LogFile, "");
        assert_eq!(held(&form), (1, Some(1)), "the same text is no change");
        form.set_text(Field::LogFile, "a.bin");
        assert_eq!(held(&form), (0, Some(0)));
        assert!(!form.enabled("BUT_Geotagimages"));
    }

    /// Browse Pictures starts at the log's folder, and a folder with a `location.txt` saying
    /// `seconds_offset: ` puts its digits in the offset box. Cancel takes the folder the dialog
    /// opened at, as the C# reads `SelectedPath` whatever the dialog returned.
    #[test]
    fn choosing_a_folder_fills_the_offset_from_location_txt() {
        let dir = scratch("folder");
        let with = dir.join("with");
        let without = dir.join("without");
        std::fs::create_dir_all(&with).unwrap();
        std::fs::create_dir_all(&without).unwrap();
        std::fs::write(with.join("location.txt"), "x seconds_offset: 42 y\n").unwrap();
        let with = with.to_str().unwrap().to_owned();
        let without = without.to_str().unwrap().to_owned();

        let mut form = flat();
        // An empty log box: `GetDirectoryName` throws, and the dialog starts where it was.
        form.browse_dir();
        assert_eq!(form.prompt().map(Prompt::text), Some(""));
        form.answer_prompt(false);
        assert_eq!(form.text(Field::JpgDir), "");

        form.set_text(Field::LogFile, &format!("{with}/camera.bin"));
        form.browse_dir();
        assert_eq!(form.prompt().map(Prompt::text), Some(with.as_str()));
        form.answer_prompt(true);
        assert_eq!(form.text(Field::JpgDir), with);
        assert_eq!(form.text(Field::OffsetSeconds), "42");

        form.set_text(Field::OffsetSeconds, "7");
        form.browse_dir();
        let Some(Prompt::Folder(field)) = form.prompt.as_mut() else {
            panic!("no dialog");
        };
        field.set(format!("{without}/"));
        form.answer_prompt(true);
        assert_eq!(form.text(Field::JpgDir), without);
        assert_eq!(form.text(Field::OffsetSeconds), "7");

        // A folder that is not there: the dialog stays.
        form.browse_dir();
        let Some(Prompt::Folder(field)) = form.prompt.as_mut() else {
            panic!("no dialog");
        };
        field.set(format!("{without}/nothing"));
        form.answer_prompt(true);
        assert!(form.prompt().is_some());
        // Cancel: the folder the dialog opened at, the log's.
        form.answer_prompt(false);
        assert_eq!(form.text(Field::JpgDir), with);
        assert_eq!(form.text(Field::OffsetSeconds), "42");
    }

    /// Browse Log takes a file that is there and puts its folder in the photo box; Cancel reads
    /// `FileName` all the same, so after a first choice it puts the photo box back.
    #[test]
    fn browse_log_fills_both_boxes() {
        let (dir, log, _) = setup("browselog");
        let mut form = flat();
        form.browse_log(&format!("{dir}/"));
        form.answer_prompt(true);
        assert!(form.prompt().is_some(), "a folder is not a file");
        form.answer_prompt(false);
        assert_eq!(form.text(Field::LogFile), "");

        form.browse_log(&format!("{dir}/"));
        let Some(Prompt::Log(field)) = form.prompt.as_mut() else {
            panic!("no dialog");
        };
        field.set(log.clone());
        form.answer_prompt(true);
        assert_eq!(form.text(Field::LogFile), log);
        assert_eq!(form.text(Field::JpgDir), dir);

        form.set_text(Field::JpgDir, "typed");
        form.browse_log("");
        form.answer_prompt(false);
        assert_eq!(form.text(Field::JpgDir), dir);
    }

    /// The numbers: typed text read on leaving, held to the range, the arrows by the increment;
    /// `num_minshutter`'s value is `minshutter`. The shutter lag's box reads a whole number and
    /// puts "0" back for anything else. Use AMSL Alt sets `useAMSLAlt` and enables the base
    /// altitude only when it is off.
    #[test]
    fn the_numbers_and_the_boxes_with_handlers() {
        let mut form = flat();
        form.start_edit(Target::Num(Num::MinShutter));
        if let Some(Editing::Num(_, field)) = form.editing.as_mut() {
            field.set("0.25");
        }
        assert_eq!(form.num_text(Num::MinShutter), "0.25");
        form.commit_edit();
        assert_eq!(form.georef().unwrap().minshutter, 0.25);
        form.up(Num::MinShutter);
        assert_eq!(form.num_text(Num::MinShutter), "0.26");
        assert_eq!(form.georef().unwrap().minshutter, 0.26);
        form.type_num(Num::MinShutter, "abc");
        assert_eq!(form.num_text(Num::MinShutter), "0.26");
        form.type_num(Num::MinShutter, "200");
        assert_eq!(form.num_text(Num::MinShutter), "99.00");
        form.type_num(Num::CameraRotation, "-500");
        assert_eq!(form.num_text(Num::CameraRotation), "-180");
        form.down(Num::CameraRotation);
        assert_eq!(form.num_text(Num::CameraRotation), "-180");
        form.up(Num::Vfov);
        assert_eq!(form.settings().vfov, 131.0);
        assert_eq!(form.georef().unwrap().minshutter, 99.0);

        form.set_text(Field::ShutterLag, "150");
        assert_eq!(form.georef().unwrap().millis_shutter_lag, 150);
        form.set_text(Field::ShutterLag, "15x");
        assert_eq!(form.text(Field::ShutterLag), "0");
        assert_eq!(form.georef().unwrap().millis_shutter_lag, 0);

        assert!(form.georef().unwrap().use_amsl_alt);
        form.click_check(Check::AmslAltUse);
        assert!(!form.georef().unwrap().use_amsl_alt);
        assert!(form.enabled("txt_basealt"));
        form.click_check(Check::AmslAltUse);
        assert!(form.georef().unwrap().use_amsl_alt);
        assert!(!form.enabled("txt_basealt"));
    }

    /// `ProcessType_CheckedChanged`: the mode, and the panels each mode uses; a box in a panel
    /// disabled while it is being typed into is left.
    #[test]
    fn the_radio_buttons_pick_the_mode_and_the_panels() {
        let mut form = flat();
        assert_eq!(form.mode(), ProcessingMode::CamMsg);
        assert!(!form.enabled("PANEL_TIME_OFFSET") && form.enabled("PANEL_SHUTTER_LAG"));
        form.start_edit(Target::Text(Field::OffsetSeconds));
        assert_eq!(form.editing(), None, "a disabled box takes no typing");
        form.click_radio(Radio::TimeOffset);
        assert_eq!(form.mode(), ProcessingMode::TimeOffset);
        assert!(form.enabled("BUT_estoffset") && !form.enabled("TXT_shutterLag"));
        form.start_edit(Target::Text(Field::OffsetSeconds));
        assert_eq!(form.editing(), Some(Target::Text(Field::OffsetSeconds)));
        form.click_radio(Radio::Trig);
        assert_eq!(form.mode(), ProcessingMode::Trig);
        assert!(!form.enabled("PANEL_TIME_OFFSET") && !form.enabled("PANEL_SHUTTER_LAG"));
        assert_eq!(form.editing(), None);
        assert!(form.radio(Radio::Trig) && !form.radio(Radio::CamMsg));
        form.click_radio(Radio::CamMsg);
        assert!(form.enabled("PANEL_SHUTTER_LAG"));
    }

    /// The markers under a point, by GMap's `LocalArea`: the camera icon's 20 by 20 square about
    /// its point, the pin's 32 by 32 above it; every one under the point, cameras first.
    #[test]
    fn markers_under_the_pointer_are_gmaps() {
        let at = |lat: f64| LatLon::new(lat, 153.0).unwrap();
        let contents = MapContents {
            route: Vec::new(),
            cams: vec![CamMarker {
                position: at(-27.0),
                tooltip: "Photo\nAlt: 25\nNo: 0\nRoll: 0.00".to_owned(),
                footprint: Vec::new(),
            }],
            photos: vec![PhotoMarker {
                position: at(-27.0),
                tooltip: "IMG_0001.jpg".to_owned(),
            }],
        };
        let screen = |_: LatLon| Some((100.0, 100.0));
        let both = vec![Marker::Cam(0), Marker::Photo(0)];
        assert_eq!(markers_at(&contents, (100.0, 100.0), screen), both);
        assert_eq!(markers_at(&contents, (90.0, 90.0), screen), both);
        assert_eq!(
            markers_at(&contents, (110.0, 100.0), screen),
            vec![Marker::Photo(0)]
        );
        assert_eq!(
            markers_at(&contents, (100.0, 69.0), screen),
            vec![Marker::Photo(0)]
        );
        assert_eq!(
            markers_at(&contents, (100.0, 101.0), screen),
            vec![Marker::Cam(0)]
        );
        assert!(markers_at(&contents, (117.0, 80.0), screen).is_empty());
        assert!(markers_at(&contents, (100.0, 100.0), |_| None).is_empty());
        assert_eq!(tooltip(&contents, Marker::Photo(0)), Some("IMG_0001.jpg"));
    }

    /// A camera marker is a `camera_feedback_t` of the `CAM` position: `(int)(lat * 1e7)`, the
    /// AMSL altitude as a float in its tooltip, its number from 0.
    #[test]
    fn a_camera_marker_is_the_csharps_feedback_message() {
        let mut georef = GeoRefImageBase::new(the_day());
        for (key, lat) in [(1, -27.469_799_99), (2, -27.47)] {
            georef.cam_locations.set(
                key,
                Location {
                    lat,
                    lon: 153.025_099_99,
                    alt_amsl: 25.1,
                    ..Location::default()
                },
            );
        }
        let contents = map_contents(&georef, &mp_georef::Flat, PHOTO_FOV);
        assert_eq!(contents.cams.len(), 2);
        assert_eq!(contents.cams[0].position.latitude(), -27.469_799_9);
        assert_eq!(contents.cams[0].position.longitude(), 153.025_099_9);
        assert_eq!(
            contents.cams[0].tooltip,
            "Photo\nAlt: 25.1\nNo: 0\nRoll: 0.00"
        );
        assert_eq!(
            contents.cams[1].tooltip,
            "Photo\nAlt: 25.1\nNo: 1\nRoll: 0.00"
        );
        assert!(contents.route.is_empty() && contents.photos.is_empty());
    }

    /// `Path.GetDirectoryName`.
    #[test]
    fn get_directory_name_is_dotnets() {
        assert_eq!(get_directory_name(""), Err(()));
        assert_eq!(get_directory_name("camera.bin"), Ok(Some(String::new())));
        assert_eq!(get_directory_name("/"), Ok(None));
        assert_eq!(get_directory_name("/x"), Ok(Some("/".to_owned())));
        assert_eq!(get_directory_name("/a/b.bin"), Ok(Some("/a".to_owned())));
        assert_eq!(get_directory_name("/a/"), Ok(Some("/a".to_owned())));
        assert_eq!(get_directory_name("a/b/c"), Ok(Some("a/b".to_owned())));
    }

    /// Every fact `tests/gui/fly-georef.gui` asserts on is one [`record_facts`] records, and
    /// every control it clicks is one the form draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_the_form_has() {
        let script = include_str!("../../../tests/gui/fly-georef.gui");
        let source = include_str!("georef_ui.rs");
        let mut names: Vec<String> = [
            "mode",
            "TXT_outputlog",
            "lines",
            "running",
            "last",
            "offset",
            "files",
            "kml",
            "geotagged",
            "positions",
            "cams",
            "pictures",
            "useAMSLAlt",
            "millisShutterLag",
            "minshutter",
            "map.route",
            "map.cams",
            "map.photos",
            "map.hover",
            "pictureBox1",
            "prompt",
            "prompt.text",
            "editing",
            "networklink",
        ]
        .map(ToOwned::to_owned)
        .to_vec();
        for name in names.clone() {
            assert!(
                source.contains(&format!("\"fly.georef.{name}\"")),
                "{name} is not recorded"
            );
        }
        names.extend(Field::ALL.map(|field| field.name().to_owned()));
        names.extend(Num::ALL.map(|num| num.name().to_owned()));
        names.extend(Check::ALL.map(|check| check.name().to_owned()));
        names.extend(Radio::ALL.map(|radio| radio.name().to_owned()));
        names.extend(ENABLED_FACTS.map(|control| format!("{control}.enabled")));
        let (mut facts, mut clicks) = (0, 0);
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("fly.georef") => {
                    let name = key.strip_prefix("fly.georef.").unwrap_or("");
                    let known = key == "fly.georef" || names.iter().any(|known| known == name);
                    assert!(known, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("georef") => {
                    let name = id.strip_prefix("georef-").unwrap_or("");
                    let name = name.split('@').next().unwrap_or(name);
                    let base = name
                        .strip_suffix("-up")
                        .or_else(|| name.strip_suffix("-down"))
                        .unwrap_or(name);
                    let drawn = control_named(base).is_some()
                        || source.contains(&format!("\"georef-{base}\""));
                    assert!(drawn, "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 40 && clicks > 8, "{facts} facts, {clicks} clicks");
    }
}
