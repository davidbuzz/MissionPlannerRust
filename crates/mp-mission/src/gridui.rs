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

//! The Survey (Grid) dialog without its form: `Grid/GridUI.cs`.
//!
//! Everything the dialog decides is here, and the screen in `crates/mp-gui/src/survey_ui.rs` only
//! draws it and forwards the operator's clicks: the controls' values as the C# holds them (each
//! `NumericUpDown`'s `decimal`, with the Designer's range, step and places), the events each
//! control raises and what they run (`GridUI.Designer.cs`'s wiring), `doCalc`'s camera arithmetic,
//! the grid regenerated on every change (`domainUpDown1_ValueChanged`) through
//! [`crate::grid::create_grid`], [`crate::corridor::create_corridor`] and
//! [`crate::rotary::create_rotary`], the Stats labels as that handler writes them, the settings
//! `loadsettings` reads and `savesettings` writes, and the mission items `BUT_Accept_Click` hands
//! the planner, call by call.
//!
//! `tests/gridui_vectors.rs` holds all of it to the dialog's own code run under mono (the grid
//! oracle's `accept` verb; its file, GridUI.cs's code re-hosted, was deleted on 2026-10-03 and
//! `testdata/grid/golden/accept` stands as it wrote it): every control, the calculated text
//! boxes, the thirteen Stats labels, the grid's shape and every call to the bit.
//!
//! Metric only, as the rest of this application: `CurrentState.multiplierdist` and
//! `multiplierspeed` are 1 and `distunits` is not "Feet", so the dialog's feet branch
//! (`GridUI.cs:774-812`) and its `inchpixel` / `feet_fov` strings never show.

use mp_units::LatLon;

use crate::cameras::{CameraInfo, Cameras};
use crate::corridor::{CorridorArgs, create_corridor};
use crate::dotnet::{
    Decimal, bool_text, format_f32, format_f64, general_f32, general_f64, parse_bool, parse_f32,
    parse_f64, parse_i32, to_int,
};
use crate::grid::{GridArgs, GridPoint, GridTag, StartPosition, create_grid};
use crate::rotary::{RotaryArgs, create_rotary};
use crate::utm::{pow2, to_utm};

/// `GridUI.cs:36`.
const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
/// `GridUI.cs:37`.
const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// The `MAV_CMD`s Accept adds.
pub mod cmd {
    /// `MAV_CMD.WAYPOINT`.
    pub const WAYPOINT: u16 = 16;
    /// `MAV_CMD.RETURN_TO_LAUNCH`.
    pub const RETURN_TO_LAUNCH: u16 = 20;
    /// `MAV_CMD.LAND`.
    pub const LAND: u16 = 21;
    /// `MAV_CMD.TAKEOFF`.
    pub const TAKEOFF: u16 = 22;
    /// `MAV_CMD.SPLINE_WAYPOINT`.
    pub const SPLINE_WAYPOINT: u16 = 82;
    /// `MAV_CMD.CONDITION_YAW`.
    pub const CONDITION_YAW: u16 = 115;
    /// `MAV_CMD.DO_JUMP`.
    pub const DO_JUMP: u16 = 177;
    /// `MAV_CMD.DO_CHANGE_SPEED`.
    pub const DO_CHANGE_SPEED: u16 = 178;
    /// `MAV_CMD.DO_SET_SERVO`.
    pub const DO_SET_SERVO: u16 = 183;
    /// `MAV_CMD.DO_REPEAT_SERVO`.
    pub const DO_REPEAT_SERVO: u16 = 184;
    /// `MAV_CMD.DO_DIGICAM_CONTROL`.
    pub const DO_DIGICAM_CONTROL: u16 = 203;
    /// `MAV_CMD.DO_SET_CAM_TRIGG_DIST`.
    pub const DO_SET_CAM_TRIGG_DIST: u16 = 206;
}

/// What a control's event runs, as `GridUI.Designer.cs` wires it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Handler {
    /// Nothing is wired.
    None,
    /// `domainUpDown1_ValueChanged`, directly or through `NUM_Lane_Dist_ValueChanged`,
    /// `NUM_ValueChanged`, `TXT_TextChanged` or `CHK_camdirection_CheckedChanged`.
    Redraw,
    /// The same, for `NUM_spacing` and `NUM_Distance`, whose handlers `doCalc` takes off while it
    /// sets them (`GridUI.cs:1096-1112`).
    RedrawUnlessDetached,
    /// `CHK_advanced_CheckedChanged`: the Grid Options and Camera Config tabs.
    Advanced,
    /// `CHK_copter_headinghold_CheckedChanged`.
    HeadingHold,
    /// `CHK_copter_headingholdlock_CheckedChanged`.
    HeadingHoldLock,
}

// ---------------------------------------------------------------------------------------------
// The NumericUpDowns.
// ---------------------------------------------------------------------------------------------

/// One of the dialog's `NumericUpDown`s.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Num {
    /// `NUM_altitude`, "Altitude (m)".
    Altitude,
    /// `NUM_angle`, "Angle [deg]".
    Angle,
    /// `NUM_UpDownFlySpeed`, "Flying Speed (est) (m/s)".
    FlySpeed,
    /// `NUM_split`, "Split into x segments".
    Split,
    /// `NUM_Distance`, "Distance between lines [m]".
    Distance,
    /// `NUM_spacing`, "Take a picture every [m]" - hidden (`Visible` False in the `.resx`).
    Spacing,
    /// `NUM_overshoot`, the first of "OverShoot [m]".
    Overshoot,
    /// `NUM_overshoot2`, the second.
    Overshoot2,
    /// `NUM_leadin`, the first of "LeadIn [m]".
    Leadin,
    /// `NUM_leadin2`, the second.
    Leadin2,
    /// `num_overlap`, "Overlap [%]".
    Overlap,
    /// `num_sidelap`, "Sidelap [%]".
    Sidelap,
    /// `num_corridorwidth`, "Corridor Width [m]".
    CorridorWidth,
    /// `NUM_copter_delay`, "Delay at WP (sec)".
    CopterDelay,
    /// `NUM_Lane_Dist`, "Min Lane separation".
    LaneDist,
    /// `NUM_laps`, "Number of Laps".
    Laps,
    /// `NUM_clockwise_laps`, "Number of Clockwise Laps".
    ClockwiseLaps,
    /// `NUM_focallength`, "Focal Length [mm]".
    FocalLength,
    /// `NUM_reptservo`, DO_REPEAT_SERVO's "Servo".
    ReptServo,
    /// `num_reptpwm`, its "PWM".
    ReptPwm,
    /// `NUM_repttime`, its "Cycle Time [s]".
    ReptTime,
    /// `num_setservono`, DO_SET_SERVO's "Servo".
    SetServoNo,
    /// `num_setservolow`, its "PWM L".
    SetServoLow,
    /// `num_setservohigh`, its "PWM H".
    SetServoHigh,
}

/// A `NumericUpDown` as the Designer sets it up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NumSpec {
    /// The control's name.
    pub name: &'static str,
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
    handler: Handler,
}

const fn spec(
    name: &'static str,
    range: (i128, i128),
    increment: (i128, u32),
    places: u32,
    initial: i128,
    handler: Handler,
) -> NumSpec {
    NumSpec {
        name,
        minimum: Decimal::new(range.0, 0),
        maximum: Decimal::new(range.1, 0),
        increment: Decimal::new(increment.0, increment.1),
        places,
        initial: Decimal::new(initial, 0),
        handler,
    }
}

impl Num {
    /// Every one, in the order the golden files list them.
    pub const ALL: [Self; 24] = [
        Self::Altitude,
        Self::Angle,
        Self::FlySpeed,
        Self::Split,
        Self::Distance,
        Self::Spacing,
        Self::Overshoot,
        Self::Overshoot2,
        Self::Leadin,
        Self::Leadin2,
        Self::Overlap,
        Self::Sidelap,
        Self::CorridorWidth,
        Self::CopterDelay,
        Self::LaneDist,
        Self::Laps,
        Self::ClockwiseLaps,
        Self::FocalLength,
        Self::ReptServo,
        Self::ReptPwm,
        Self::ReptTime,
        Self::SetServoNo,
        Self::SetServoLow,
        Self::SetServoHigh,
    ];

    /// The Designer's range, step, places and value, and what `ValueChanged` runs.
    /// `// C#: Grid/GridUI.Designer.cs:413-1333`
    #[must_use]
    pub const fn spec(self) -> NumSpec {
        use Handler::{None, Redraw, RedrawUnlessDetached};
        match self {
            // :1316-1333, Increment 10
            Self::Altitude => spec("NUM_altitude", (1, 99_999), (10, 0), 0, 100, Redraw),
            // :1290, no Value: 0 until the constructor sets it
            Self::Angle => spec("NUM_angle", (0, 360), (1, 0), 0, 0, Redraw),
            // :1257-1270
            Self::FlySpeed => spec("NUM_UpDownFlySpeed", (0, 360), (1, 0), 1, 5, Redraw),
            // :1215-1226
            Self::Split => spec("NUM_split", (1, 300), (1, 0), 0, 1, Redraw),
            // :1136-1149, Minimum 0.3
            Self::Distance => NumSpec {
                minimum: Decimal::new(3, 1),
                ..spec(
                    "NUM_Distance",
                    (0, 9999),
                    (1, 0),
                    2,
                    50,
                    RedrawUnlessDetached,
                )
            },
            // :1164
            Self::Spacing => spec("NUM_spacing", (0, 5000), (1, 0), 0, 0, RedrawUnlessDetached),
            // :1116-1121
            Self::Overshoot => spec("NUM_overshoot", (-999, 9999), (1, 0), 0, 0, Redraw),
            // :1049-1054
            Self::Overshoot2 => spec("NUM_overshoot2", (-999, 9999), (1, 0), 0, 0, Redraw),
            // :1033-1038
            Self::Leadin => spec("NUM_leadin", (-999, 9999), (1, 0), 0, 0, Redraw),
            // :964-969
            Self::Leadin2 => spec("NUM_leadin2", (-999, 9999), (1, 0), 0, 0, Redraw),
            // :1084-1089, the default range
            Self::Overlap => spec("num_overlap", (0, 100), (1, 0), 1, 50, Redraw),
            // :1096-1101
            Self::Sidelap => spec("num_sidelap", (0, 100), (1, 0), 1, 60, Redraw),
            // :997-1011
            Self::CorridorWidth => spec("num_corridorwidth", (1, 5000), (1, 0), 1, 100, Redraw),
            // :918-929, Increment 0.1, not wired
            Self::CopterDelay => spec("NUM_copter_delay", (0, 9999), (1, 1), 1, 0, None),
            // :848, NUM_Lane_Dist_ValueChanged
            Self::LaneDist => spec("NUM_Lane_Dist", (0, 9999), (1, 0), 0, 0, Redraw),
            // :761-772
            Self::Laps => spec("NUM_laps", (1, 9999), (1, 0), 0, 200, Redraw),
            // :799-804
            Self::ClockwiseLaps => spec("NUM_clockwise_laps", (-1, 9999), (1, 0), 0, 0, Redraw),
            // :666-689, NUM_ValueChanged
            Self::FocalLength => spec("NUM_focallength", (1, 500), (1, 1), 1, 5, Redraw),
            // :538-549, not wired
            Self::ReptServo => spec("NUM_reptservo", (5, 12), (1, 0), 0, 5, None),
            // :523-529
            Self::ReptPwm => spec("num_reptpwm", (0, 5000), (1, 0), 0, 1100, None),
            // :503-514
            Self::ReptTime => spec("NUM_repttime", (1, 5000), (1, 0), 0, 2, None),
            // :413-424
            Self::SetServoNo => spec("num_setservono", (5, 12), (1, 0), 0, 5, None),
            // :443-449
            Self::SetServoLow => spec("num_setservolow", (0, 5000), (1, 0), 0, 1100, None),
            // :458-464
            Self::SetServoHigh => spec("num_setservohigh", (0, 5000), (1, 0), 0, 1900, None),
        }
    }

    /// The control's name in the Designer.
    #[must_use]
    pub const fn name(self) -> &'static str {
        self.spec().name
    }

    /// The control with this Designer name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|num| num.name() == name)
    }

    const fn index(self) -> usize {
        self as usize
    }
}

// ---------------------------------------------------------------------------------------------
// The CheckBoxes, the RadioButtons and the TextBoxes.
// ---------------------------------------------------------------------------------------------

/// One of the dialog's `CheckBox`es.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Check {
    /// `CHK_camdirection`.
    CamDirection,
    /// `CHK_usespeed`.
    UseSpeed,
    /// `CHK_toandland`.
    ToAndLand,
    /// `CHK_toandland_RTL`.
    ToAndLandRtl,
    /// `CHK_internals`.
    Internals,
    /// `CHK_footprints`.
    Footprints,
    /// `CHK_advanced`.
    Advanced,
    /// `CHK_boundary`.
    Boundary,
    /// `CHK_markers`.
    Markers,
    /// `CHK_grid`.
    Grid,
    /// `chk_crossgrid`.
    CrossGrid,
    /// `chk_Corridor`.
    Corridor,
    /// `chk_spiral`.
    Spiral,
    /// `CHK_copter_headinghold`.
    HeadingHold,
    /// `CHK_copter_headingholdlock`.
    HeadingHoldLock,
    /// `chk_spline`.
    Spline,
    /// `chk_optimize_for_distance`.
    OptimizeForDistance,
    /// `CHK_match_spiral_perimeter`.
    MatchSpiralPerimeter,
    /// `chk_stopstart`.
    StopStart,
}

/// Which event a `CheckBox`'s handler is wired to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Event {
    /// `CheckedChanged`: any change, the operator's or the code's.
    CheckedChanged,
    /// `Click`: the operator's click only.
    Click,
}

impl Check {
    /// Every one, in the order the golden files list them.
    pub const ALL: [Self; 19] = [
        Self::CamDirection,
        Self::UseSpeed,
        Self::ToAndLand,
        Self::ToAndLandRtl,
        Self::Internals,
        Self::Footprints,
        Self::Advanced,
        Self::Boundary,
        Self::Markers,
        Self::Grid,
        Self::CrossGrid,
        Self::Corridor,
        Self::Spiral,
        Self::HeadingHold,
        Self::HeadingHoldLock,
        Self::Spline,
        Self::OptimizeForDistance,
        Self::MatchSpiralPerimeter,
        Self::StopStart,
    ];

    /// The control's name, its `.resx` text, whether the Designer checks it, and its wiring.
    /// `// C#: Grid/GridUI.Designer.cs, Grid/GridUI.resx`
    const fn spec(self) -> (&'static str, &'static str, bool, Event, Handler) {
        use Event::{CheckedChanged, Click};
        use Handler::{Advanced, HeadingHold, HeadingHoldLock, None, Redraw};
        match self {
            Self::CamDirection => (
                "CHK_camdirection",
                "Camera top facing forward",
                true,
                CheckedChanged,
                Redraw,
            ),
            Self::UseSpeed => (
                "CHK_usespeed",
                "Use speed for this mission",
                false,
                CheckedChanged,
                None,
            ),
            Self::ToAndLand => (
                "CHK_toandland",
                "Add Takeoff and Land WP's",
                true,
                CheckedChanged,
                None,
            ),
            Self::ToAndLandRtl => ("CHK_toandland_RTL", "Use RTL", true, CheckedChanged, None),
            Self::Internals => ("CHK_internals", "Internals", false, CheckedChanged, Redraw),
            Self::Footprints => (
                "CHK_footprints",
                "Footprints",
                false,
                CheckedChanged,
                Redraw,
            ),
            Self::Advanced => (
                "CHK_advanced",
                "Advanced Options",
                false,
                CheckedChanged,
                Advanced,
            ),
            Self::Boundary => ("CHK_boundary", "Boundary", true, CheckedChanged, Redraw),
            Self::Markers => ("CHK_markers", "Markers", true, CheckedChanged, Redraw),
            Self::Grid => ("CHK_grid", "Grid", true, CheckedChanged, Redraw),
            Self::CrossGrid => ("chk_crossgrid", "Cross Grid", false, Click, Redraw),
            Self::Corridor => ("chk_Corridor", "Corridor", false, Click, Redraw),
            Self::Spiral => ("chk_spiral", "Spiral", false, Click, Redraw),
            Self::HeadingHold => (
                "CHK_copter_headinghold",
                "Heading Hold",
                false,
                CheckedChanged,
                HeadingHold,
            ),
            Self::HeadingHoldLock => (
                "CHK_copter_headingholdlock",
                "Unlock from grid",
                false,
                CheckedChanged,
                HeadingHoldLock,
            ),
            Self::Spline => (
                "chk_spline",
                "Spline Exit/Entrys",
                false,
                CheckedChanged,
                None,
            ),
            Self::OptimizeForDistance => (
                "chk_optimize_for_distance",
                "Optimise for Distance",
                false,
                CheckedChanged,
                Redraw,
            ),
            Self::MatchSpiralPerimeter => (
                "CHK_match_spiral_perimeter",
                "Match Perimeter to Polygon",
                false,
                CheckedChanged,
                Redraw,
            ),
            Self::StopStart => (
                "chk_stopstart",
                "Breakup starts",
                true,
                CheckedChanged,
                None,
            ),
        }
    }

    /// The control's name in the Designer.
    #[must_use]
    pub const fn name(self) -> &'static str {
        self.spec().0
    }

    /// Its text in the `.resx`.
    #[must_use]
    pub const fn text(self) -> &'static str {
        self.spec().1
    }

    /// The control with this Designer name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|check| check.name() == name)
    }

    const fn index(self) -> usize {
        self as usize
    }
}

/// The trigger method, `groupBox3`'s four `RadioButton`s.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trigger {
    /// `rad_trigdist`, "CAM_TRIGG_DIST", checked in the Designer.
    Distance,
    /// `rad_digicam`, "DO_DIGICAM_CONTROL".
    Digicam,
    /// `rad_repeatservo`, "DO_REPEAT_SERVO".
    RepeatServo,
    /// `rad_do_set_servo`, "DO_SET_SERVO".
    SetServo,
}

impl Trigger {
    /// All four, in the order the golden files list them.
    pub const ALL: [Self; 4] = [
        Self::Distance,
        Self::Digicam,
        Self::RepeatServo,
        Self::SetServo,
    ];

    /// The control's name in the Designer.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Distance => "rad_trigdist",
            Self::Digicam => "rad_digicam",
            Self::RepeatServo => "rad_repeatservo",
            Self::SetServo => "rad_do_set_servo",
        }
    }

    /// Its text in the `.resx`.
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Distance => "CAM_TRIGG_DIST",
            Self::Digicam => "DO_DIGICAM_CONTROL",
            Self::RepeatServo => "DO_REPEAT_SERVO",
            Self::SetServo => "DO_SET_SERVO",
        }
    }

    /// The radio button with this Designer name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|trigger| trigger.name() == name)
    }

    const fn index(self) -> usize {
        self as usize
    }
}

/// One of the dialog's `TextBox`es.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Text {
    /// `TXT_headinghold`: the heading to hold, disabled and read only in the Designer.
    HeadingHold,
    /// `TXT_imgwidth`, "Image Width [Pixels]".
    ImgWidth,
    /// `TXT_imgheight`, "Image Height [Pixels]".
    ImgHeight,
    /// `TXT_senswidth`, "Sensor Width [mm]".
    SensWidth,
    /// `TXT_sensheight`, "Sensor Height [mm]".
    SensHeight,
    /// `TXT_cmpixel`, "cm/pixel", which `doCalc` writes.
    CmPixel,
    /// `TXT_fovH`, "Field of View Horizontal [m]", which `doCalc` writes.
    FovH,
    /// `TXT_fovV`, "Field of View Vertical [m]", which `doCalc` writes.
    FovV,
}

impl Text {
    /// All of them, in the order the golden files list them.
    pub const ALL: [Self; 8] = [
        Self::HeadingHold,
        Self::ImgWidth,
        Self::ImgHeight,
        Self::SensWidth,
        Self::SensHeight,
        Self::CmPixel,
        Self::FovH,
        Self::FovV,
    ];

    /// The control's name, its `.resx` text, and whether `TextChanged` redraws.
    const fn spec(self) -> (&'static str, &'static str, bool) {
        match self {
            Self::HeadingHold => ("TXT_headinghold", "0", false),
            Self::ImgWidth => ("TXT_imgwidth", "4608", true),
            Self::ImgHeight => ("TXT_imgheight", "3456", true),
            Self::SensWidth => ("TXT_senswidth", "6.16", true),
            Self::SensHeight => ("TXT_sensheight", "4.62", true),
            Self::CmPixel => ("TXT_cmpixel", "", false),
            Self::FovH => ("TXT_fovH", "", false),
            Self::FovV => ("TXT_fovV", "", false),
        }
    }

    /// The control's name in the Designer.
    #[must_use]
    pub const fn name(self) -> &'static str {
        self.spec().0
    }

    /// The control with this Designer name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|text| text.name() == name)
    }

    const fn index(self) -> usize {
        self as usize
    }
}

/// One of `groupBox5`'s value labels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stat {
    /// `lbl_area`, captioned "Area: ".
    Area,
    /// `lbl_distance`, "Distance: ".
    Distance,
    /// `lbl_spacing`, "Dist between images: ".
    Spacing,
    /// `lbl_grndres`, "Ground Resolution: ".
    GroundResolution,
    /// `lbl_pictures`, "Pictures:".
    Pictures,
    /// `lbl_strips`, "No of Strips:".
    Strips,
    /// `lbl_footprint`, "Footprint:".
    Footprint,
    /// `lbl_distbetweenlines`, "Dist between lines:".
    DistBetweenLines,
    /// `lbl_flighttime`, "Flight Time (est):".
    FlightTime,
    /// `lbl_photoevery`, "Photo every (est):".
    PhotoEvery,
    /// `lbl_turnrad`, "Turn Dia (at 45d):".
    TurnRadius,
    /// `lbl_gndelev`, "Ground Elevation:".
    GroundElevation,
    /// `lbl_minshutter`, "Min Shutter Speed:".
    MinShutter,
}

impl Stat {
    /// All thirteen, in the order the golden files list them.
    pub const ALL: [Self; 13] = [
        Self::Area,
        Self::Distance,
        Self::Spacing,
        Self::GroundResolution,
        Self::Pictures,
        Self::Strips,
        Self::Footprint,
        Self::DistBetweenLines,
        Self::FlightTime,
        Self::PhotoEvery,
        Self::TurnRadius,
        Self::GroundElevation,
        Self::MinShutter,
    ];

    /// The label's name in the Designer.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Area => "lbl_area",
            Self::Distance => "lbl_distance",
            Self::Spacing => "lbl_spacing",
            Self::GroundResolution => "lbl_grndres",
            Self::Pictures => "lbl_pictures",
            Self::Strips => "lbl_strips",
            Self::Footprint => "lbl_footprint",
            Self::DistBetweenLines => "lbl_distbetweenlines",
            Self::FlightTime => "lbl_flighttime",
            Self::PhotoEvery => "lbl_photoevery",
            Self::TurnRadius => "lbl_turnrad",
            Self::GroundElevation => "lbl_gndelev",
            Self::MinShutter => "lbl_minshutter",
        }
    }

    const fn index(self) -> usize {
        self as usize
    }
}

// ---------------------------------------------------------------------------------------------
// The dialog.
// ---------------------------------------------------------------------------------------------

/// What the dialog is opened with from the planner: `MainV2.comPort.MAV.cs`'s planned home and
/// firmware, and `Grid.StartPointLatLngAlt`, a static that outlives any one dialog.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Context {
    /// `cs.PlannedHomeLocation`: (0, 0) until a home is planned.
    pub home: (f64, f64, f64),
    /// `cs.firmware == Firmwares.ArduPlane`: the flying speed starts at 12 rather than 5.
    pub plane: bool,
    /// `Grid.StartPointLatLngAlt`, as the last dialog left it.
    pub start_point: LatLon,
}

impl Default for Context {
    fn default() -> Self {
        Self {
            home: (0.0, 0.0, 0.0),
            plane: false,
            start_point: LatLon::default(),
        }
    }
}

/// What Accept needs from the planner beyond the dialog.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Planner {
    /// The rows the `Commands` grid holds before Accept adds any: `AddWPtoList` returns row
    /// numbers, and the jumps `NUM_split` adds point at them.
    pub rows: usize,
    /// The vehicle's `WPNAV_SPEED`, if it has one.
    pub wpnav_speed: Option<f64>,
    /// The vehicle's `WP_SPD`, if it has one.
    pub wp_spd: Option<f64>,
}

/// One `plugin.Host.AddWPtoList` or `InsertWP`: the command and its seven numbers, `x` the
/// longitude and `y` the latitude as the C# passes them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Call {
    /// `MAV_CMD`.
    pub command: u16,
    /// `p1` to `p4`.
    pub params: [f64; 4],
    /// `x`: longitude, for a command with a position.
    pub x: f64,
    /// `y`: latitude.
    pub y: f64,
    /// `z`: altitude.
    pub z: f64,
}

/// A row Accept puts in the planner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Step {
    /// `AddWPtoList`: a row at the end.
    Add(Call),
    /// `InsertWP(index, ...)`: a row at a row index, or at the end past the last row.
    Insert(usize, Call),
}

/// What Accept did.
#[derive(Debug, Clone, PartialEq)]
pub struct Accepted {
    /// The rows, in the order the C# adds them.
    pub steps: Vec<Step>,
    /// The settings `savesettings` writes and the camera's field of view for the footprints,
    /// `camera_fovh` and `camera_fovv`, in the order the C# writes them.
    pub settings: Vec<(&'static str, String)>,
    /// The polygon to redraw on the planner: `plugin.Host.RedrawFPPolygon(list)`.
    pub polygon: Vec<LatLon>,
}

/// Why Accept did nothing.
#[derive(Debug, Clone, PartialEq)]
pub enum Refused {
    /// A message box: its text and caption.
    Message(&'static str, &'static str),
    /// An exception the C# does not catch, part way through the rows: the rows added before it
    /// stay, and the dialog does not close.
    Exception(Vec<Step>, &'static str),
}

/// `BUT_save_Click`'s box for a Camera Config box that is not a number.
/// `// C#: Grid/GridUI.cs:1597`
pub const NOT_A_NUMBER: &str = "One of your entries is not a valid number";

/// `BUT_save_Click`'s `InputBox`: its title, its prompt (the C#'s words, typo and all) and the
/// name it offers.
/// `// C#: Grid/GridUI.cs:1571-1573`
pub const CAMERA_NAME_TITLE: &str = "Camera Name";
/// See [`CAMERA_NAME_TITLE`].
pub const CAMERA_NAME_PROMPT: &str = "Please and a camera name";
/// See [`CAMERA_NAME_TITLE`].
pub const CAMERA_NAME_DEFAULT: &str = "Default";

/// `FormatException`'s message, for a heading `Convert.ToInt32` cannot read.
pub const FORMAT_EXCEPTION: &str = "Input string was not in a correct format.";

/// The dialog's state: every control, the grid it last drew, and its Stats labels.
#[derive(Debug, Clone)]
pub struct Dialog {
    /// `list`: the polygon's vertices. `calcpolygonarea` takes a closing vertex off it.
    list: Vec<LatLon>,
    nums: [Decimal; 24],
    checks: [bool; 19],
    triggers: [bool; 4],
    texts: [String; 8],
    /// `CMB_camera.Text`.
    camera: String,
    /// `CMB_startfrom.Text`.
    startfrom: String,
    cameras: Cameras,
    /// `grid`: the last pattern generated, null until the first.
    grid: Option<Vec<GridPoint>>,
    stats: [String; 13],
    loading: bool,
    /// Whether `NUM_spacing` and `NUM_Distance` redraw: `doCalc` detaches them while it sets them.
    spacing_handlers: bool,
    /// `Grid.StartPointLatLngAlt`.
    start_point: LatLon,
    /// `cs.PlannedHomeLocation`.
    home: (f64, f64, f64),
    /// How many times the grid has been generated: a check that the events ran as the C#'s do.
    recomputes: u32,
    /// `TXT_headinghold.Enabled`, `CHK_copter_headingholdlock.Enabled` and the two buttons'.
    heading_enabled: bool,
    /// `TXT_headinghold.ReadOnly`.
    heading_read_only: bool,
}

impl Dialog {
    /// `new GridUI(plugin)` and `GridUI_Load`: the controls at the Designer's values, the angle of
    /// the polygon's longest side, the camera list, the settings the last Accept saved (when
    /// `grid_camera` is among them), and the grid drawn once.
    /// `// C#: Grid/GridUI.cs:66-149`
    #[must_use]
    pub fn open(
        polygon: &[LatLon],
        cameras: Cameras,
        context: &Context,
        saved: &dyn Fn(&str) -> Option<String>,
    ) -> Self {
        let mut dialog = Self {
            list: polygon.to_vec(),
            nums: Num::ALL.map(|num| num.spec().initial),
            checks: Check::ALL.map(|check| check.spec().2),
            triggers: Trigger::ALL.map(|trigger| trigger == Trigger::Distance),
            texts: Text::ALL.map(|text| text.spec().1.to_owned()),
            camera: String::new(),
            startfrom: String::new(),
            cameras,
            grid: None,
            stats: Stat::ALL.map(|_| "0.00".to_owned()),
            loading: true,
            spacing_handlers: true,
            start_point: context.start_point,
            home: context.home,
            recomputes: 0,
            heading_enabled: false,
            heading_read_only: true,
        };

        // CMB_startfrom.DataSource = Enum.GetNames(...); SelectedIndex = 0
        dialog.startfrom = StartPosition::Home.name().to_owned();

        // set and angle that is good
        if let Some(angle) =
            Decimal::from_f64((angle_of_longest_side(&dialog.list) + 360.0) % 360.0)
        {
            let _ = dialog.set_num(Num::Angle, angle);
        }
        let heading = dialog.num(Num::Angle).round_even().to_text();
        dialog.put_text(Text::HeadingHold, heading);

        if context.plane {
            // (decimal)(12 * CurrentState.multiplierspeed): a float.
            if let Some(speed) = Decimal::from_f32(12.0 * 1.0) {
                let _ = dialog.set_num(Num::FlySpeed, speed);
            }
        }
        dialog.loading = false;

        // GridUI_Load
        dialog.loading = true;
        if saved("grid_camera").is_some() {
            dialog.load_settings(saved);
        }
        dialog.loading = false;
        dialog.recompute();
        dialog
    }

    // ---- What the controls hold ----

    /// A `NumericUpDown`'s `Value`.
    #[must_use]
    pub fn num(&self, num: Num) -> Decimal {
        self.nums.get(num.index()).copied().unwrap_or(Decimal::ZERO)
    }

    /// What a `NumericUpDown` shows: `Value` at `DecimalPlaces`.
    #[must_use]
    pub fn num_text(&self, num: Num) -> String {
        self.num(num).to_fixed(num.spec().places)
    }

    /// A `CheckBox`'s `Checked`.
    #[must_use]
    pub fn check(&self, check: Check) -> bool {
        self.checks.get(check.index()).copied().unwrap_or(false)
    }

    /// Whether a trigger method's `RadioButton` is checked.
    #[must_use]
    pub fn trigger(&self, trigger: Trigger) -> bool {
        self.triggers.get(trigger.index()).copied().unwrap_or(false)
    }

    /// A `TextBox`'s `Text`.
    #[must_use]
    pub fn text(&self, text: Text) -> &str {
        self.texts.get(text.index()).map_or("", String::as_str)
    }

    /// `CMB_camera.Text`.
    #[must_use]
    pub fn camera(&self) -> &str {
        &self.camera
    }

    /// `CMB_camera.Items`.
    #[must_use]
    pub fn camera_items(&self) -> &[String] {
        self.cameras.items()
    }

    /// `CMB_startfrom.Text`.
    #[must_use]
    pub fn startfrom(&self) -> &str {
        &self.startfrom
    }

    /// A Stats label's `Text`.
    #[must_use]
    pub fn stat(&self, stat: Stat) -> &str {
        self.stats.get(stat.index()).map_or("", String::as_str)
    }

    /// The pattern last generated.
    #[must_use]
    pub fn grid(&self) -> &[GridPoint] {
        self.grid.as_deref().unwrap_or(&[])
    }

    /// The polygon, as the dialog holds it.
    #[must_use]
    pub fn polygon(&self) -> &[LatLon] {
        &self.list
    }

    /// `Grid.StartPointLatLngAlt`, for the next dialog.
    #[must_use]
    pub const fn start_point(&self) -> LatLon {
        self.start_point
    }

    /// How many times the grid has been generated.
    #[must_use]
    pub const fn recomputes(&self) -> u32 {
        self.recomputes
    }

    /// Whether `doCalc` left `NUM_spacing` and `NUM_Distance` wired.
    #[must_use]
    pub const fn spacing_handlers(&self) -> bool {
        self.spacing_handlers
    }

    /// Whether the heading hold's box, lock and buttons are enabled.
    #[must_use]
    pub const fn heading_enabled(&self) -> bool {
        self.heading_enabled
    }

    /// Whether the heading hold's box is read only.
    #[must_use]
    pub const fn heading_read_only(&self) -> bool {
        self.heading_read_only
    }

    /// The tab pages `tabControl1` shows: Simple, and Grid Options and Camera Config while
    /// Advanced Options is checked (`CHK_advanced_CheckedChanged`, `GridUI.cs:1350-1363`).
    #[must_use]
    pub fn tabs(&self) -> &'static [Tab] {
        if self.check(Check::Advanced) {
            &[Tab::Simple, Tab::Grid, Tab::Camera]
        } else {
            &[Tab::Simple]
        }
    }

    // ---- The operator ----

    /// Typing into a `NumericUpDown` and leaving it: `ParseEditText`, which sets `Value` to what
    /// `decimal.Parse` reads, held to the range, and leaves it alone when nothing parses.
    pub fn type_num(&mut self, num: Num, text: &str) {
        let Some(value) = Decimal::parse(text) else {
            return;
        };
        // Constrain: a value equal to a bound is kept as typed, scale and all.
        let spec = num.spec();
        let value = if value < spec.minimum {
            spec.minimum
        } else if value > spec.maximum {
            spec.maximum
        } else {
            value
        };
        let _ = self.set_num(num, value);
    }

    /// The up arrow: `UpButton`, one `Increment` more, no more than `Maximum`.
    pub fn up(&mut self, num: Num) {
        let spec = num.spec();
        let value = match self.num(num).checked_add(spec.increment) {
            Some(value) if value > spec.maximum => spec.maximum,
            Some(value) => value,
            None => spec.maximum,
        };
        let _ = self.set_num(num, value);
    }

    /// The down arrow: `DownButton`, one `Increment` less, no less than `Minimum`.
    pub fn down(&mut self, num: Num) {
        let spec = num.spec();
        let value = match self.num(num).checked_sub(spec.increment) {
            Some(value) if value < spec.minimum => spec.minimum,
            Some(value) => value,
            None => spec.minimum,
        };
        let _ = self.set_num(num, value);
    }

    /// A click on a `CheckBox`: it toggles, then `CheckedChanged` and `Click` run.
    pub fn click_check(&mut self, check: Check) {
        let value = !self.check(check);
        put(&mut self.checks, check.index(), value);
        let (_, _, _, _, handler) = check.spec();
        self.run(handler);
    }

    /// A click on a trigger method: it is checked and the others in `groupBox3` are not.
    pub fn click_trigger(&mut self, trigger: Trigger) {
        for other in Trigger::ALL {
            put(&mut self.triggers, other.index(), other == trigger);
        }
    }

    /// Typing into a `TextBox`: `TextChanged`, which redraws for the camera's four.
    pub fn set_text(&mut self, text: Text, value: &str) {
        if self.text(text) == value {
            return;
        }
        self.put_text(text, value.to_owned());
        if text.spec().2 {
            self.recompute();
        }
    }

    /// Picking a camera in `CMB_camera`: `CMB_camera_SelectedIndexChanged`, when the pick changes.
    /// `// C#: Grid/GridUI.cs:1316-1333`
    pub fn select_camera(&mut self, name: &str) {
        if self.camera == name {
            return;
        }
        name.clone_into(&mut self.camera);
        self.camera_selected();
    }

    /// `BUT_save_Click` after its `InputBox` ("Camera Name", default "Default") has said OK with
    /// `name`: `CMB_camera.Text = name`, then the camera of that name - the one held, or a new
    /// one added - takes the name, `NUM_focallength` and the four boxes in that order, each box
    /// through `float.Parse`. A box that will not parse stops it there with the message box;
    /// what was set before it stays set, in memory, and nothing is written. Ok is the dictionary
    /// to write to `cameras.xml` ([`Cameras::write`]). `CMB_camera`'s list is not changed.
    /// `// C#: Grid/GridUI.cs:1567-1602`
    ///
    /// # Errors
    /// A box that is not a number: "One of your entries is not a valid number".
    pub fn save_camera(&mut self, name: &str) -> Result<&Cameras, Refused> {
        self.set_camera_text(name);
        let focallen = self.num(Num::FocalLength).to_f32();
        let boxes = [
            parse_f32(self.text(Text::ImgHeight)),
            parse_f32(self.text(Text::ImgWidth)),
            parse_f32(self.text(Text::SensHeight)),
            parse_f32(self.text(Text::SensWidth)),
        ];
        let Some(camera) = self.cameras.entry(&self.camera) else {
            return Ok(&self.cameras);
        };
        camera.name.clone_from(&self.camera);
        camera.focallen = focallen;
        let fields = [
            &mut camera.imageheight,
            &mut camera.imagewidth,
            &mut camera.sensorheight,
            &mut camera.sensorwidth,
        ];
        for (field, parsed) in fields.into_iter().zip(boxes) {
            let Some(value) = parsed else {
                return Err(Refused::Message(NOT_A_NUMBER, ""));
            };
            *field = value;
        }
        Ok(&self.cameras)
    }

    /// `CMB_camera.Text = value` on a `DropDown` combo box: the text is set; then, unless it is
    /// the selected item's text already, the item it names ignoring case is selected - and a
    /// selection that changes raises `SelectedIndexChanged`, whose handler loads that camera over
    /// the boxes, with the text becoming the item's. Here the selected item is the one the text
    /// names exactly.
    /// `// C#: Grid/GridUI.cs:1576, 1344-1362`
    fn set_camera_text(&mut self, value: &str) {
        let selected = self
            .cameras
            .items()
            .iter()
            .find(|item| **item == self.camera)
            .cloned();
        value.clone_into(&mut self.camera);
        if selected.as_deref() == Some(value) {
            return;
        }
        let lower = value.to_lowercase();
        let found = self
            .cameras
            .items()
            .iter()
            .find(|item| item.to_lowercase() == lower)
            .cloned();
        if let Some(item) = found
            && selected.as_ref() != Some(&item)
        {
            self.camera = item;
            self.camera_selected();
        }
    }

    /// `CMB_camera_SelectedIndexChanged`: the camera's lens and sensor into the Camera Config
    /// boxes, each raising its own event, then the grid redrawn.
    fn camera_selected(&mut self) {
        if let Some(camera) = self.cameras.get(&self.camera).cloned()
            && !self.load_camera(&camera)
        {
            return;
        }
        self.recompute();
    }

    /// The camera's five values, in the handler's order. False where `NUM_focallength` throws.
    fn load_camera(&mut self, camera: &CameraInfo) -> bool {
        let Some(focal) = Decimal::from_f32(camera.focallen) else {
            return false;
        };
        if self.set_num(Num::FocalLength, focal).is_err() {
            return false;
        }
        self.set_text(Text::ImgHeight, &general_f32(camera.imageheight));
        self.set_text(Text::ImgWidth, &general_f32(camera.imagewidth));
        self.set_text(Text::SensHeight, &general_f32(camera.sensorheight));
        self.set_text(Text::SensWidth, &general_f32(camera.sensorwidth));
        true
    }

    /// Picking a start in `CMB_startfrom`. True when it is Point, whose handler asks "Enter point
    /// #" before it goes on; [`Self::answer_point`] is the rest of it.
    /// `// C#: Grid/GridUI.cs:1917-1933`
    pub fn select_startfrom(&mut self, start: StartPosition) -> bool {
        if self.startfrom == start.name() {
            return false;
        }
        start.name().clone_into(&mut self.startfrom);
        if self.loading {
            return false;
        }
        if start == StartPosition::Point {
            return true;
        }
        self.recompute();
        false
    }

    /// The rest of `CMB_startfrom_SelectedIndexChanged` for Point: `InputBox.Show("Enter point
    /// #", ...)` answered - `None` for Cancel, which leaves the 1 it offered - then the boundary
    /// point it names as `Grid.StartPointLatLngAlt`, if the polygon has more points than that, and
    /// the grid redrawn. An answer `int.Parse` refuses throws, and nothing more happens.
    pub fn answer_point(&mut self, answer: Option<&str>) {
        let pnt = match answer {
            None => 1,
            Some(text) => match parse_i32(text) {
                Some(pnt) => pnt,
                None => return,
            },
        };
        if usize::try_from(pnt).is_ok_and(|pnt| self.list.len() > pnt) {
            // list[pnt - 1]; a pnt of 0 or less throws before this.
            let Some(point) = usize::try_from(pnt - 1)
                .ok()
                .and_then(|index| self.list.get(index))
            else {
                return;
            };
            self.start_point = *point;
        } else if pnt < 1 && !self.list.is_empty() {
            // `list.Count > pnt` holds for any pnt below the count, and list[pnt - 1] throws.
            return;
        }
        self.recompute();
    }

    /// `BUT_headingholdplus_Click`: 180 degrees round, or 1 once unlocked from the grid.
    /// `// C#: Grid/GridUI.cs:1397-1423`
    pub fn heading_plus(&mut self) {
        let Some(previous) = parse_i32(self.text(Text::HeadingHold)) else {
            return;
        };
        let value = if self.check(Check::HeadingHoldLock) {
            if previous + 1 > 359 {
                previous - 359
            } else {
                previous + 1
            }
        } else if previous + 180 > 359 {
            previous - 180
        } else {
            previous + 180
        };
        self.put_text(Text::HeadingHold, value.to_string());
    }

    /// `BUT_headingholdminus_Click`.
    /// `// C#: Grid/GridUI.cs:1425-1452`
    pub fn heading_minus(&mut self) {
        let Some(previous) = parse_i32(self.text(Text::HeadingHold)) else {
            return;
        };
        let value = if self.check(Check::HeadingHoldLock) {
            if previous - 1 < 0 {
                previous + 359
            } else {
                previous - 1
            }
        } else if previous - 180 < 0 {
            previous + 180
        } else {
            previous - 180
        };
        self.put_text(Text::HeadingHold, value.to_string());
    }

    // ---- The events ----

    /// The `Value` setter: nothing if it does not move, `ArgumentOutOfRangeException` outside the
    /// range, otherwise the new value and `ValueChanged`.
    fn set_num(&mut self, num: Num, value: Decimal) -> Result<(), ()> {
        if value == self.num(num) {
            return Ok(());
        }
        let spec = num.spec();
        if value < spec.minimum || value > spec.maximum {
            return Err(());
        }
        put(&mut self.nums, num.index(), value);
        self.run(spec.handler);
        Ok(())
    }

    /// `Checked` set by the code: `CheckedChanged` if it moved, never `Click`.
    fn set_check(&mut self, check: Check, value: bool) {
        if self.check(check) == value {
            return;
        }
        put(&mut self.checks, check.index(), value);
        let (_, _, _, event, handler) = check.spec();
        if event == Event::CheckedChanged {
            self.run(handler);
        }
    }

    /// A trigger method's `Checked` set by the code: checking one clears the others, clearing one
    /// checks nothing.
    fn set_trigger(&mut self, trigger: Trigger, value: bool) {
        if value {
            self.click_trigger(trigger);
        } else {
            put(&mut self.triggers, trigger.index(), false);
        }
    }

    fn run(&mut self, handler: Handler) {
        match handler {
            Handler::None | Handler::Advanced => {}
            Handler::Redraw => self.recompute(),
            Handler::RedrawUnlessDetached => {
                if self.spacing_handlers {
                    self.recompute();
                }
            }
            Handler::HeadingHold => self.heading_hold_changed(),
            Handler::HeadingHoldLock => self.heading_hold_lock_changed(),
        }
    }

    /// `CHK_copter_headinghold_CheckedChanged`: checking it enables the heading box, its lock
    /// (unchecked) and the two buttons.
    /// `// C#: Grid/GridUI.cs:1365-1382`
    fn heading_hold_changed(&mut self) {
        if self.check(Check::HeadingHold) {
            self.heading_enabled = true;
            self.set_check(Check::HeadingHoldLock, false);
        } else {
            self.heading_enabled = false;
        }
    }

    /// `CHK_copter_headingholdlock_CheckedChanged`: unlocked the box takes typing; locked again it
    /// goes back to the grid's angle.
    /// `// C#: Grid/GridUI.cs:1384-1395`
    fn heading_hold_lock_changed(&mut self) {
        if self.check(Check::HeadingHoldLock) {
            self.heading_read_only = false;
        } else {
            self.heading_read_only = true;
            let heading = self.num(Num::Angle).round_even().to_text();
            self.put_text(Text::HeadingHold, heading);
        }
    }

    // ---- loadsettings and savesettings ----

    /// `loadsettings`: each saved value into its control, as the code sets it - a value that does
    /// not parse, or is out of a `NumericUpDown`'s range, is passed over - with the camera late
    /// "so it invokes a reload". The angle is saved but not loaded, and the repeat-servo numbers
    /// are loaded but never saved.
    /// `// C#: Grid/GridUI.cs:360-448`
    pub fn load_settings(&mut self, saved: &dyn Fn(&str) -> Option<String>) {
        enum Control {
            Num(Num),
            Check(Check),
            Trigger(Trigger),
            Camera,
            StartFrom,
        }
        use Control as C;
        let order = [
            ("grid_alt", C::Num(Num::Altitude)),
            ("grid_camdir", C::Check(Check::CamDirection)),
            ("grid_usespeed", C::Check(Check::UseSpeed)),
            ("grid_speed", C::Num(Num::FlySpeed)),
            ("grid_autotakeoff", C::Check(Check::ToAndLand)),
            ("grid_autotakeoff_RTL", C::Check(Check::ToAndLandRtl)),
            ("grid_dist", C::Num(Num::Distance)),
            ("grid_overshoot1", C::Num(Num::Overshoot)),
            ("grid_overshoot2", C::Num(Num::Overshoot2)),
            ("grid_leadin1", C::Num(Num::Leadin)),
            ("grid_leadin2", C::Num(Num::Leadin2)),
            ("grid_startfrom", C::StartFrom),
            ("grid_overlap", C::Num(Num::Overlap)),
            ("grid_sidelap", C::Num(Num::Sidelap)),
            ("grid_spacing", C::Num(Num::Spacing)),
            ("grid_crossgrid", C::Check(Check::CrossGrid)),
            ("grid_spiral", C::Check(Check::Spiral)),
            ("grid_trigdist", C::Trigger(Trigger::Distance)),
            ("grid_digicam", C::Trigger(Trigger::Digicam)),
            ("grid_repeatservo", C::Trigger(Trigger::RepeatServo)),
            ("grid_breakstopstart", C::Check(Check::StopStart)),
            ("grid_repeatservo_no", C::Num(Num::ReptServo)),
            ("grid_repeatservo_pwm", C::Num(Num::ReptPwm)),
            ("grid_repeatservo_cycle", C::Num(Num::ReptTime)),
            ("grid_camera", C::Camera),
            ("grid_copter_spline", C::Check(Check::Spline)),
            ("grid_copter_delay", C::Num(Num::CopterDelay)),
            ("grid_min_lane_separation", C::Num(Num::LaneDist)),
            ("grid_clockwise_laps", C::Num(Num::ClockwiseLaps)),
            ("grid_laps", C::Num(Num::Laps)),
            (
                "grid_match_spiral_perimeter",
                C::Check(Check::MatchSpiralPerimeter),
            ),
            ("grid_internals", C::Check(Check::Internals)),
            ("grid_footprints", C::Check(Check::Footprints)),
            ("grid_advanced", C::Check(Check::Advanced)),
        ];
        for (key, control) in order {
            let Some(value) = saved(key) else {
                continue;
            };
            match control {
                C::Num(num) => {
                    if let Some(value) = Decimal::parse(&value) {
                        let _ = self.set_num(num, value);
                    }
                }
                C::Check(check) => {
                    if let Some(value) = parse_bool(&value) {
                        self.set_check(check, value);
                    }
                }
                C::Trigger(trigger) => {
                    if let Some(value) = parse_bool(&value) {
                        self.set_trigger(trigger, value);
                    }
                }
                C::Camera => {
                    // Text set to an item's name picks it, which raises SelectedIndexChanged.
                    let picks = self.cameras.items().contains(&value);
                    if self.camera != value {
                        self.camera = value;
                        if picks {
                            self.camera_selected();
                        }
                    }
                }
                C::StartFrom => {
                    if self.startfrom != value {
                        self.startfrom = value;
                    }
                }
            }
        }
    }

    /// `savesettings`, in its order.
    /// `// C#: Grid/GridUI.cs:450-510`
    #[must_use]
    pub fn save_settings(&self) -> Vec<(&'static str, String)> {
        let num = |num: Num| self.num(num).to_text();
        let check = |check: Check| bool_text(self.check(check)).to_owned();
        let trigger = |trigger: Trigger| bool_text(self.trigger(trigger)).to_owned();
        vec![
            ("grid_camera", self.camera.clone()),
            ("grid_alt", num(Num::Altitude)),
            ("grid_angle", num(Num::Angle)),
            ("grid_camdir", check(Check::CamDirection)),
            ("grid_usespeed", check(Check::UseSpeed)),
            ("grid_speed", num(Num::FlySpeed)),
            ("grid_dist", num(Num::Distance)),
            ("grid_overshoot1", num(Num::Overshoot)),
            ("grid_overshoot2", num(Num::Overshoot2)),
            ("grid_leadin1", num(Num::Leadin)),
            ("grid_leadin2", num(Num::Leadin2)),
            ("grid_overlap", num(Num::Overlap)),
            ("grid_sidelap", num(Num::Sidelap)),
            ("grid_spacing", num(Num::Spacing)),
            ("grid_crossgrid", check(Check::CrossGrid)),
            ("grid_spiral", check(Check::Spiral)),
            ("grid_startfrom", self.startfrom.clone()),
            ("grid_autotakeoff", check(Check::ToAndLand)),
            ("grid_autotakeoff_RTL", check(Check::ToAndLandRtl)),
            ("grid_internals", check(Check::Internals)),
            ("grid_footprints", check(Check::Footprints)),
            ("grid_advanced", check(Check::Advanced)),
            ("grid_trigdist", trigger(Trigger::Distance)),
            ("grid_digicam", trigger(Trigger::Digicam)),
            ("grid_repeatservo", trigger(Trigger::RepeatServo)),
            ("grid_breakstopstart", check(Check::StopStart)),
            ("grid_copter_spline", check(Check::Spline)),
            ("grid_copter_delay", num(Num::CopterDelay)),
            ("grid_copter_headinghold_chk", check(Check::HeadingHold)),
            ("grid_min_lane_separation", num(Num::LaneDist)),
            ("grid_clockwise_laps", num(Num::ClockwiseLaps)),
            ("grid_laps", num(Num::Laps)),
            (
                "grid_match_spiral_perimeter",
                check(Check::MatchSpiralPerimeter),
            ),
        ]
    }

    // ---- domainUpDown1_ValueChanged ----

    /// `domainUpDown1_ValueChanged`: the camera's numbers, then the pattern, then the Stats.
    /// `// C#: Grid/GridUI.cs:586-872`
    pub fn recompute(&mut self) {
        if self.loading {
            return;
        }

        if !self.camera.is_empty() {
            self.do_calc();
        }
        self.recomputes += 1;

        // Enum.Parse(typeof(StartPosition), CMB_startfrom.Text) throws on anything else.
        let Some(startpos) = StartPosition::from_name(&self.startfrom) else {
            return;
        };
        let altitude = self.num(Num::Altitude).to_f64() / 1.0; // fromDistDisplayUnit
        let distance = self.num(Num::Distance).to_f64();
        let spacing = self.num(Num::Spacing).to_f64();
        let angle = self.num(Num::Angle).to_f64();
        let home = LatLon::new(self.home.0, self.home.1).unwrap_or_default();
        let grid_args = GridArgs {
            altitude,
            distance,
            spacing,
            angle,
            overshoot1: self.num(Num::Overshoot).to_f64(),
            overshoot2: self.num(Num::Overshoot2).to_f64(),
            startpos,
            min_lane_separation: self.num(Num::LaneDist).to_f32(),
            leadin1: self.num(Num::Leadin).to_f32(),
            leadin2: self.num(Num::Leadin2).to_f32(),
            home,
            start_point: self.start_point,
            use_extended_endpoint: self.check(Check::OptimizeForDistance),
        };

        let generated = if self.check(Check::Corridor) {
            create_corridor(
                &self.list,
                &CorridorArgs {
                    altitude,
                    distance,
                    spacing,
                    startpos,
                    width: f64::from(self.num(Num::CorridorWidth).to_f32()),
                },
            )
            .ok()
        } else if self.check(Check::Spiral) {
            let (Some(clockwise_laps), Some(laps)) = (
                self.num(Num::ClockwiseLaps).to_i32(),
                self.num(Num::Laps).to_i32(),
            ) else {
                return;
            };
            create_rotary(
                &self.list,
                &RotaryArgs {
                    altitude,
                    distance,
                    startpos,
                    home,
                    start_point: self.start_point,
                    clockwise_laps,
                    match_spiral_perimeter: self.check(Check::MatchSpiralPerimeter),
                    laps,
                },
            )
            .ok()
        } else {
            create_grid(&self.list, &grid_args).ok()
        };
        // A generator that throws faults the awaited task, and the handler ends there.
        let Some(mut grid) = generated else {
            return;
        };

        if grid.is_empty() {
            self.grid = Some(grid);
            return;
        }

        if self.check(Check::CrossGrid) {
            // add crossover
            if let Some(last) = grid.last()
                && let Ok(last) = LatLon::new(last.lat, last.lng)
            {
                self.start_point = last;
            }
            let cross = GridArgs {
                angle: angle + 90.0,
                startpos: StartPosition::Point,
                start_point: self.start_point,
                ..grid_args
            };
            match create_grid(&self.list, &cross) {
                Ok(more) => grid.extend(more),
                Err(_) => {
                    self.grid = Some(grid);
                    return;
                }
            }
        }
        self.stats_from(&grid);
        self.grid = Some(grid);
        self.calc_heading_hold();
    }

    /// The Stats half of `domainUpDown1_ValueChanged`: the route's length from home and back, the
    /// strips and pictures, the turn and the timings. There is no terrain data, so every point's
    /// ground is `srtm`'s invalid answer, 0.
    /// `// C#: Grid/GridUI.cs:640-866`
    fn stats_from(&mut self, grid: &[GridPoint]) {
        let (Some(first), Some(last)) = (grid.first(), grid.last()) else {
            return;
        };
        let mut strips = 0_i32;
        let mut images = 0_i32;
        let mut prevpoint = *first;
        // distance to/from home
        let (home_lat, home_lng, _) = self.home;
        let mut routetotal = get_distance(first.lat, first.lng, home_lat, home_lng) / 1000.0
            + get_distance(last.lat, last.lng, home_lat, home_lng) / 1000.0;
        let mut maxgroundelevation = f64::MIN;
        let mut mingroundelevation = f64::MAX;

        for item in grid {
            let currentalt = 0.0; // srtm.getAltitude: altresponce.Invalid
            mingroundelevation = mingroundelevation.min(currentalt);
            maxgroundelevation = maxgroundelevation.max(currentalt);

            let mut segment = None;
            if item.tag == GridTag::Middle {
                images += 1;
                if self.check(Check::Internals) {
                    segment = Some((prevpoint, *item));
                    prevpoint = *item;
                }
            } else {
                if item.tag != GridTag::StartMiddle && item.tag != GridTag::MiddleEnd {
                    strips += 1;
                }
                segment = Some((prevpoint, *item));
                prevpoint = *item;
            }
            // routetotal = routetotal + (float)seg.Distance
            let seg = segment.map_or(0.0, |(a, b)| route_distance(a, b));
            #[allow(clippy::cast_possible_truncation)]
            let seg = seg as f32;
            routetotal += f64::from(seg);
        }

        // turn radrad = tas^2 / (tan(angle) * G)
        let speed = self.num(Num::FlySpeed).to_f32() / 1.0;
        let v_sq: f32 = speed * speed;
        #[allow(clippy::cast_possible_truncation)]
        let g = (f64::from(9.808_f32) * (45.0 * DEG2RAD).tan()) as f32;
        let turnrad: f32 = v_sq / g;

        let area = calc_polygon_area(&mut self.list);
        let fov_h = self.text(Text::FovH).to_owned();
        let fov_v = self.text(Text::FovV).to_owned();
        self.set_stat(Stat::Area, format!("{} m^2", format_f64(area, "#")));
        self.set_stat(
            Stat::Distance,
            format!("{} km", format_f64(routetotal, "0.##")),
        );
        self.set_stat(
            Stat::Spacing,
            format!("{} m", self.num(Num::Spacing).format("0.#")),
        );
        self.set_stat(Stat::GroundResolution, self.text(Text::CmPixel).to_owned());
        self.set_stat(
            Stat::DistBetweenLines,
            format!("{} m", self.num(Num::Distance).format("0.##")),
        );
        self.set_stat(Stat::Footprint, format!("{fov_h} x {fov_v} m"));
        self.set_stat(
            Stat::TurnRadius,
            format!("{} m", format_f32(turnrad * 2.0, "0")),
        );
        self.set_stat(
            Stat::GroundElevation,
            format!(
                "{}-{} m",
                format_f64(mingroundelevation, "0"),
                format_f64(maxgroundelevation, "0")
            ),
        );

        if !self.text(Text::CmPixel).is_empty() {
            // speed m/s, cm/pixel, m/pixel, gsd / 2.0, min shutter speed
            let cmpix = parse_f32(self.text(Text::CmPixel).trim_end_matches(['c', 'm', ' ']));
            if let Some(cmpix) = cmpix {
                let mpix = f64::from(cmpix) * 0.01;
                let minmpix = mpix / 2.0;
                let minshutter = f64::from(speed) / minmpix;
                self.set_stat(
                    Stat::MinShutter,
                    format!("1/{}", general_f64(minshutter - minshutter % 1.0)),
                );
            }
        }

        let flyspeedms = self.num(Num::FlySpeed).to_f64() / 1.0; // fromSpeedDisplayUnit
        self.set_stat(Stat::Pictures, images.to_string());
        self.set_stat(Stat::Strips, (strips / 2).to_string());
        // reduce flying speed by 20 %
        let seconds = (routetotal * 1000.0) / (flyspeedms * 0.8);
        self.set_stat(Stat::FlightTime, seconds_to_nice(seconds));
        self.set_stat(
            Stat::PhotoEvery,
            seconds_to_nice(self.num(Num::Spacing).to_f64() / flyspeedms),
        );
    }

    fn set_stat(&mut self, stat: Stat, text: String) {
        put(&mut self.stats, stat.index(), text);
    }

    /// A `TextBox`'s text set by the code, which raises nothing the dialog wires.
    fn put_text(&mut self, text: Text, value: String) {
        put(&mut self.texts, text.index(), value);
    }

    /// `doCalc`: the footprint at this altitude, the ground resolution, and the trigger and lane
    /// distances from the overlap and sidelap - set with their handlers off, so they do not
    /// redraw. Anything that throws ends it where it is.
    /// `// C#: Grid/GridUI.cs:1066-1117`
    fn do_calc(&mut self) {
        // entered values
        #[allow(clippy::cast_possible_truncation)]
        let flyalt = (f64::from(self.num(Num::Altitude).to_f32()) / 1.0) as f32;
        let Some(_imagewidth) = parse_i32(self.text(Text::ImgWidth)) else {
            return;
        };
        let Some(imageheight) = parse_i32(self.text(Text::ImgHeight)) else {
            return;
        };
        let (Some(overlap), Some(sidelap)) = (
            self.num(Num::Overlap).to_i32(),
            self.num(Num::Sidelap).to_i32(),
        ) else {
            return;
        };
        let Some((viewwidth, viewheight)) = self.get_fov(f64::from(flyalt)) else {
            return;
        };

        self.set_text(Text::FovH, &format_f64(viewwidth, "#.#"));
        self.set_text(Text::FovV, &format_f64(viewheight, "#.#"));
        //    mm  / pixels * 100
        self.set_text(
            Text::CmPixel,
            &format_f64((viewheight / f64::from(imageheight)) * 100.0, "0.00 cm"),
        );

        self.spacing_handlers = false;
        #[allow(clippy::cast_precision_loss)]
        let (overlap, sidelap) = (
            1.0_f32 - (overlap as f32 / 100.0_f32),
            1.0_f32 - (sidelap as f32 / 100.0_f32),
        );
        let (spacing, distance) = if self.check(Check::CamDirection) {
            (
                f64::from(overlap) * viewheight,
                f64::from(sidelap) * viewwidth,
            )
        } else {
            (
                f64::from(overlap) * viewwidth,
                f64::from(sidelap) * viewheight,
            )
        };
        let Some(spacing) = Decimal::from_f64(spacing) else {
            return;
        };
        if self.set_num(Num::Spacing, spacing).is_err() {
            return;
        }
        let Some(distance) = Decimal::from_f64(distance) else {
            return;
        };
        if self.set_num(Num::Distance, distance).is_err() {
            return;
        }
        self.spacing_handlers = true;
    }

    /// `getFOV`: the ground a picture covers from `flyalt`, width and height in metres. `None`
    /// where `double.Parse` throws on a sensor box.
    /// `// C#: Grid/GridUI.cs:1035-1054`
    fn get_fov(&self, flyalt: f64) -> Option<(f64, f64)> {
        let focallen = self.num(Num::FocalLength).to_f64();
        let sensorwidth = parse_f64(self.text(Text::SensWidth))?;
        let sensorheight = parse_f64(self.text(Text::SensHeight))?;

        // scale      mm / mm
        let flscale = (1000.0 * flyalt) / focallen;

        //   mm * mm / 1000
        let viewwidth = sensorwidth * flscale / 1000.0;
        let viewheight = sensorheight * flscale / 1000.0;
        Some((viewwidth, viewheight))
    }

    /// `getFOVangle`: the lens's angles of view, each through a `float`.
    /// `// C#: Grid/GridUI.cs:1056-1064`
    fn get_fov_angle(&self) -> Option<(f64, f64)> {
        let focallen = self.num(Num::FocalLength).to_f64();
        let sensorwidth = parse_f64(self.text(Text::SensWidth))?;
        let sensorheight = parse_f64(self.text(Text::SensHeight))?;
        let angle = |sensor: f64| {
            #[allow(clippy::cast_possible_truncation)]
            let angle = ((sensor / (2.0 * focallen)).atan() * RAD2DEG * 2.0) as f32;
            f64::from(angle)
        };
        Some((angle(sensorwidth), angle(sensorheight)))
    }

    /// `CalcHeadingHold`: the heading moved by as much as the angle's whole degrees moved. By the
    /// time the awaited grid comes back `NUM_angle`'s text shows its new value (`UpdateEditText`
    /// runs as the `ValueChanged` handler returns at its first `await`), so the change is the
    /// difference between that text rounded half away from zero and the value rounded half to
    /// even - nothing, unless the angle ends in exactly .5.
    /// `// C#: Grid/GridUI.cs:1119-1146`
    fn calc_heading_hold(&mut self) {
        let angle = self.num(Num::Angle);
        let Some(previous) = Decimal::parse(&angle.to_fixed(0))
            .map(Decimal::round_even)
            .and_then(Decimal::to_i32)
        else {
            return;
        };
        let Some(current) = angle.round_even().to_i32() else {
            return;
        };
        let change = current - previous;
        if change == 0 {
            return;
        }
        let Some(held) = parse_i32(self.text(Text::HeadingHold)) else {
            return;
        };
        let mut val = held + change;
        if change > 0 && val > 359 {
            val -= 360;
        }
        if change < 0 && val < 0 {
            val += 360;
        }
        self.put_text(Text::HeadingHold, val.to_string());
    }

    // ---- BUT_Accept_Click ----

    /// `BUT_Accept_Click`: the grid as mission rows, split into `NUM_split` missions each with its
    /// own takeoff and landing and a `DO_JUMP` to each at the top, then the camera's angles of view
    /// and the settings saved.
    /// `// C#: Grid/GridUI.cs:1595-1880`
    ///
    /// # Errors
    ///
    /// "Bad Grid" with no grid, "You must use Land/RTL to split a mission" when splitting without
    /// the takeoff and landing, and the exception a heading `Convert.ToInt32` cannot read throws,
    /// with the rows added before it.
    pub fn accept(&self, planner: &Planner) -> Result<Accepted, Refused> {
        let Some(grid) = self.grid.as_deref().filter(|grid| !grid.is_empty()) else {
            return Err(Refused::Message("Bad Grid", "Error"));
        };
        let split = self.num(Num::Split);
        if split > Decimal::from_int(1) && !self.check(Check::ToAndLand) {
            return Err(Refused::Message(
                "You must use Land/RTL to split a mission",
                "Error",
            ));
        }

        let mut rows = Rows {
            count: planner.rows,
            steps: Vec::new(),
        };
        // Convert.ToInt32(TXT_headinghold.Text), read by every AddWP while holding a heading.
        let heading = parse_i32(self.text(Text::HeadingHold)).map(f64::from);
        let add_wp = |rows: &mut Rows, point: &GridPoint| -> Result<(), ()> {
            self.add_wp(rows, point, heading)
        };

        let wpsplit = wpsplit(grid.len(), split);
        let mut wpsplitstart: Vec<usize> = Vec::new();

        let mut splitno = 0_i64;
        while Decimal::from_int(splitno) < split {
            let mut wpstart = wpsplit * splitno;
            let mut wpend = wpsplit * (splitno + 1);
            let count = i64::try_from(grid.len()).unwrap_or(i64::MAX);
            let tag = |index: i64| {
                usize::try_from(index)
                    .ok()
                    .and_then(|index| grid.get(index))
                    .map(|point| point.tag)
            };
            while wpstart != 0 && wpstart < count && tag(wpstart) != Some(GridTag::End) {
                wpstart -= 1;
            }
            while wpend > 0 && wpend < count && tag(wpend) != Some(GridTag::Start) {
                wpend -= 1;
            }

            if self.check(Check::ToAndLand) {
                // Copter or not, the same takeoff: (int)(30 * CurrentState.multiplierdist).
                let wpno = rows.add(call(cmd::TAKEOFF, [20.0, 0.0, 0.0, 0.0], 0.0, 0.0, 30.0));
                wpsplitstart.push(wpno);
            }

            if self.check(Check::UseSpeed) {
                let speed = f64::from(self.num(Num::FlySpeed).to_f32() / 1.0);
                rows.add(call(
                    cmd::DO_CHANGE_SPEED,
                    [0.0, speed, 0.0, 0.0],
                    0.0,
                    0.0,
                    0.0,
                ));
            }

            let spacing = f64::from(self.num(Num::Spacing).to_f32());
            let repeat = || {
                (
                    f64::from(self.num(Num::ReptServo).to_f32()),
                    f64::from(self.num(Num::ReptPwm).to_f32()),
                    f64::from(self.num(Num::ReptTime).to_f32()),
                )
            };
            let trigdist = self.trigger(Trigger::Distance);
            let digicam = self.trigger(Trigger::Digicam);
            let repeatservo = self.trigger(Trigger::RepeatServo);
            let setservo = self.trigger(Trigger::SetServo);
            let stopstart = self.check(Check::StopStart);

            let mut startedtrigdist = false;
            let mut lastplla: Option<GridPoint> = None;
            for (i, plla) in grid.iter().enumerate() {
                let i = i64::try_from(i).unwrap_or(i64::MAX);
                // skip before start point
                if i < wpstart {
                    continue;
                }
                // skip after endpoint
                if i >= wpend {
                    break;
                }
                // `plla.Lat != lastplla.Lat || ...`, lastplla starting at PointLatLngAlt.Zero.
                let moved = lastplla.map_or(
                    plla.lat != 0.0 || plla.lng != 0.0 || plla.alt != 0.0,
                    |last| plla.lat != last.lat || plla.lng != last.lng || plla.alt != last.alt,
                );
                let result = if i > wpstart {
                    self.accept_point(
                        &mut rows,
                        plla,
                        moved,
                        &mut startedtrigdist,
                        Triggers {
                            trigdist,
                            digicam,
                            repeatservo,
                            setservo,
                            stopstart,
                            spacing,
                            repeat: repeat(),
                        },
                        &add_wp,
                    )
                } else {
                    add_wp(&mut rows, plla)
                };
                if result.is_err() {
                    return Err(Refused::Exception(rows.steps, FORMAT_EXCEPTION));
                }
                lastplla = Some(*plla);
            }

            // end
            if trigdist {
                rows.add(call(
                    cmd::DO_SET_CAM_TRIGG_DIST,
                    [0.0, 0.0, 1.0, 0.0],
                    0.0,
                    0.0,
                    0.0,
                ));
            }

            if self.check(Check::UseSpeed) {
                let speed = planner
                    .wpnav_speed
                    .map(|speed| speed / 100.0)
                    .or(planner.wp_spd)
                    .unwrap_or(0.0);
                if speed > 0.0 {
                    rows.add(call(
                        cmd::DO_CHANGE_SPEED,
                        [0.0, speed, 0.0, 0.0],
                        0.0,
                        0.0,
                        0.0,
                    ));
                }
            }

            if self.check(Check::ToAndLand) {
                if self.check(Check::ToAndLandRtl) {
                    rows.add(call(cmd::RETURN_TO_LAUNCH, [0.0; 4], 0.0, 0.0, 0.0));
                } else {
                    rows.add(call(cmd::LAND, [0.0; 4], self.home.1, self.home.0, 0.0));
                }
            }
            splitno += 1;
        }

        if split > Decimal::from_int(1) {
            let jumps = wpsplitstart.len();
            for (index, i) in wpsplitstart.into_iter().enumerate() {
                // add do jump
                #[allow(clippy::cast_precision_loss)]
                let target = (i + jumps + 1) as f64;
                rows.steps.push(Step::Insert(
                    index,
                    call(cmd::DO_JUMP, [target, 1.0, 0.0, 0.0], 0.0, 0.0, 0.0),
                ));
                rows.count += 1;
            }
        }

        // save camera fov's for use with footprints
        let mut settings: Vec<(&'static str, String)> = Vec::new();
        if let Some((fovha, fovva)) = self.get_fov_angle() {
            let (h, v) = if self.check(Check::CamDirection) {
                (fovha, fovva)
            } else {
                (fovva, fovha)
            };
            settings.push(("camera_fovh", general_f64(h)));
            settings.push(("camera_fovv", general_f64(v)));
        }
        settings.extend(self.save_settings());

        Ok(Accepted {
            steps: rows.steps,
            settings,
            polygon: self.list.clone(),
        })
    }

    /// One grid point after the first of a segment: a trigger point's waypoint and camera command,
    /// or a lane end's waypoint and the trigger method's start or stop.
    #[allow(clippy::too_many_arguments)]
    fn accept_point(
        &self,
        rows: &mut Rows,
        plla: &GridPoint,
        moved: bool,
        startedtrigdist: &mut bool,
        triggers: Triggers,
        add_wp: &dyn Fn(&mut Rows, &GridPoint) -> Result<(), ()>,
    ) -> Result<(), ()> {
        let Triggers {
            trigdist,
            digicam,
            repeatservo,
            setservo,
            stopstart,
            spacing,
            repeat: (servo, pwm, cycle),
        } = triggers;
        // internal point check
        if plla.tag == GridTag::Middle {
            if repeatservo && !stopstart {
                add_wp(rows, plla)?;
                rows.add(call(
                    cmd::DO_REPEAT_SERVO,
                    [servo, pwm, 1.0, cycle],
                    0.0,
                    0.0,
                    0.0,
                ));
            }
            if digicam {
                add_wp(rows, plla)?;
                rows.add(call(
                    cmd::DO_DIGICAM_CONTROL,
                    [1.0, 0.0, 0.0, 0.0],
                    0.0,
                    1.0,
                    0.0,
                ));
            }
            return Ok(());
        }
        // only add points that are ends
        if (plla.tag == GridTag::Start || plla.tag == GridTag::End) && moved {
            add_wp(rows, plla)?;
        }

        // check trigger method
        if trigdist {
            // if stopstart enabled, add wp and trigger start/stop
            if stopstart {
                if plla.tag == GridTag::StartMiddle {
                    //  s > sm, need to dup check
                    if moved {
                        add_wp(rows, plla)?;
                    }
                    rows.add(call(
                        cmd::DO_SET_CAM_TRIGG_DIST,
                        [spacing, 0.0, 1.0, 0.0],
                        0.0,
                        0.0,
                        0.0,
                    ));
                } else if plla.tag == GridTag::MiddleEnd {
                    add_wp(rows, plla)?;
                    rows.add(call(
                        cmd::DO_SET_CAM_TRIGG_DIST,
                        [0.0, 0.0, 1.0, 0.0],
                        0.0,
                        0.0,
                        0.0,
                    ));
                }
            } else if !*startedtrigdist {
                // add single start trigger
                rows.add(call(
                    cmd::DO_SET_CAM_TRIGG_DIST,
                    [spacing, 0.0, 1.0, 0.0],
                    0.0,
                    0.0,
                    0.0,
                ));
                *startedtrigdist = true;
            } else if plla.tag == GridTag::MiddleEnd {
                add_wp(rows, plla)?;
            }
        } else if repeatservo {
            if stopstart {
                if plla.tag == GridTag::StartMiddle {
                    if moved {
                        add_wp(rows, plla)?;
                    }
                    rows.add(call(
                        cmd::DO_REPEAT_SERVO,
                        [servo, pwm, 999.0, cycle],
                        0.0,
                        0.0,
                        0.0,
                    ));
                } else if plla.tag == GridTag::MiddleEnd {
                    add_wp(rows, plla)?;
                    rows.add(call(
                        cmd::DO_REPEAT_SERVO,
                        [servo, pwm, 0.0, cycle],
                        0.0,
                        0.0,
                        0.0,
                    ));
                }
            }
        } else if setservo {
            let number = f64::from(self.num(Num::SetServoNo).to_f32());
            if plla.tag == GridTag::StartMiddle {
                if moved {
                    add_wp(rows, plla)?;
                }
                let low = f64::from(self.num(Num::SetServoLow).to_f32());
                rows.add(call(
                    cmd::DO_SET_SERVO,
                    [number, low, 0.0, 0.0],
                    0.0,
                    0.0,
                    0.0,
                ));
            } else if plla.tag == GridTag::MiddleEnd {
                add_wp(rows, plla)?;
                let high = f64::from(self.num(Num::SetServoHigh).to_f32());
                rows.add(call(
                    cmd::DO_SET_SERVO,
                    [number, high, 0.0, 0.0],
                    0.0,
                    0.0,
                    0.0,
                ));
            }
        }
        Ok(())
    }

    /// `AddWP`: a `CONDITION_YAW` first when holding a heading, then the waypoint - with the delay
    /// at it, or as a spline at a lane's start. `Err` where `Convert.ToInt32` throws on the
    /// heading.
    /// `// C#: Grid/GridUI.cs:874-897`
    fn add_wp(&self, rows: &mut Rows, point: &GridPoint, heading: Option<f64>) -> Result<(), ()> {
        if self.check(Check::HeadingHold) {
            let heading = heading.ok_or(())?;
            rows.add(call(
                cmd::CONDITION_YAW,
                [heading, 0.0, 0.0, 0.0],
                0.0,
                0.0,
                0.0,
            ));
        }
        let (lng, lat, alt) = (point.lng, point.lat, point.alt);
        let delay = self.num(Num::CopterDelay);
        if delay > Decimal::ZERO {
            // Alt * multiplierdist, not truncated here.
            rows.add(call(
                cmd::WAYPOINT,
                [delay.to_f64(), 0.0, 0.0, 0.0],
                lng,
                lat,
                alt * 1.0,
            ));
        } else {
            let altitude = f64::from(to_int(alt * 1.0));
            let spline = matches!(point.tag, GridTag::Start | GridTag::StartMiddle)
                && self.check(Check::Spline);
            let command = if spline {
                cmd::SPLINE_WAYPOINT
            } else {
                cmd::WAYPOINT
            };
            rows.add(call(command, [0.0; 4], lng, lat, altitude));
        }
        Ok(())
    }
}

/// The trigger settings `accept_point` reads, gathered once per segment.
#[derive(Debug, Clone, Copy)]
struct Triggers {
    trigdist: bool,
    digicam: bool,
    repeatservo: bool,
    setservo: bool,
    stopstart: bool,
    spacing: f64,
    repeat: (f64, f64, f64),
}

/// The `Commands` grid as Accept sees it: how many rows, and what it has added.
struct Rows {
    count: usize,
    steps: Vec<Step>,
}

impl Rows {
    /// `AddWPtoList`: the row's index, `Commands.Rows.Add()`'s answer.
    fn add(&mut self, call: Call) -> usize {
        self.steps.push(Step::Add(call));
        self.count += 1;
        self.count - 1
    }
}

/// `array[index] = value`, for an index an enum's discriminant makes valid.
fn put<T, const N: usize>(array: &mut [T; N], index: usize, value: T) {
    if let Some(slot) = array.get_mut(index) {
        *slot = value;
    }
}

const fn call(command: u16, params: [f64; 4], x: f64, y: f64, z: f64) -> Call {
    Call {
        command,
        params,
        x,
        y,
        z,
    }
}

/// `(int)Math.Round(grid.Count / NUM_split.Value, MidpointRounding.AwayFromZero)`: a decimal
/// division, rounded half away from zero.
fn wpsplit(count: usize, split: Decimal) -> i64 {
    let (Ok(count), Ok(divisor)) = (i128::try_from(count), u128::try_from(split.mantissa())) else {
        return 0;
    };
    if divisor == 0 {
        return 0;
    }
    let numerator = count.unsigned_abs() * 10_u128.pow(split.scale());
    let mut quotient = numerator / divisor;
    if (numerator % divisor) * 2 >= divisor {
        quotient += 1;
    }
    i64::try_from(quotient).unwrap_or(i64::MAX)
}

/// `secondsToNice`.
/// `// C#: Grid/GridUI.cs:899-920`
#[must_use]
pub fn seconds_to_nice(seconds: f64) -> String {
    if seconds < 0.0 {
        return "Infinity Seconds".to_owned();
    }
    let secs = seconds % 60.0;
    let mins = to_int(seconds / 60.0) % 60;
    let hours = to_int(seconds / 3600.0); // % 24;
    if hours > 0 {
        format!(
            "{hours}:{}:{} Hours",
            Decimal::from_int(i64::from(mins)).format("00"),
            format_f64(secs, "00")
        )
    } else if mins > 0 {
        format!("{mins}:{} Minutes", format_f64(secs, "00"))
    } else {
        format!("{} Seconds", format_f64(secs, "0.00"))
    }
}

/// `getAngleOfLongestSide`: the bearing along the polygon's longest side, from each vertex back
/// to the one before it, the first of equal lengths winning.
/// `// C#: Grid/GridUI.cs:1013-1033`
#[must_use]
pub fn angle_of_longest_side(list: &[LatLon]) -> f64 {
    let Some(mut last) = list.last().copied() else {
        return 0.0;
    };
    let mut angle = 0.0;
    let mut maxdist = 0.0;
    for item in list {
        let distance = item.distance_to(last).0;
        if distance > maxdist {
            angle = item.bearing_to(last).0.0;
            maxdist = distance;
        }
        last = *item;
    }
    (angle + 360.0) % 360.0
}

/// `calcpolygonarea`: the shoelace formula in the first vertex's UTM zone. The C# closes the
/// polygon to do it and then takes the last vertex off if it equals the first - so a polygon that
/// arrived closed leaves open.
/// `// C#: Grid/GridUI.cs:957-1011`
#[must_use]
pub fn calc_polygon_area(polygon: &mut Vec<LatLon>) -> f64 {
    let (Some(first), Some(last)) = (polygon.first().copied(), polygon.last().copied()) else {
        // "Please define a polygon!"
        return 0.0;
    };
    // close the polygon
    if first != last {
        polygon.push(first); // make a full loop
    }
    let utmzone = to_int((first.longitude() - -186.0) / 6.0);
    let mut prod1 = 0.0;
    let mut prod2 = 0.0;
    for pair in polygon.windows(2) {
        let [a, b] = pair else {
            continue;
        };
        let p1 = to_utm(utmzone, first.latitude(), a.latitude(), a.longitude());
        let p2 = to_utm(utmzone, first.latitude(), b.latitude(), b.longitude());
        prod1 += p1.0 * p2.1;
        prod2 += p1.1 * p2.0;
    }
    let answer = (prod1 - prod2) / 2.0;
    if polygon.first() == polygon.last() {
        polygon.pop(); // unmake a full loop
    }
    answer.abs()
}

/// `PointLatLngAlt.GetDistance`: a haversine on a 6371 km sphere, in metres, on raw degrees.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
fn get_distance(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f64 {
    let d = lat1 * 0.017_453_292_519_943_295;
    let num2 = lng1 * 0.017_453_292_519_943_295;
    let num3 = lat2 * 0.017_453_292_519_943_295;
    let num4 = lng2 * 0.017_453_292_519_943_295;
    let num5 = num4 - num2;
    let num6 = num3 - d;
    let num7 = pow2((num6 / 2.0).sin()) + ((d.cos() * num3.cos()) * pow2((num5 / 2.0).sin()));
    let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
    (6371.0 * num8) * 1000.0
}

/// `MapRoute.Distance` over one pair: GMap's `PureProjection.GetDistance` on the Mercator axis,
/// 6378137 m, in kilometres.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET/MapRoute.cs:80-96, PureProjection.cs:436-448`
fn route_distance(p1: GridPoint, p2: GridPoint) -> f64 {
    let rad = std::f64::consts::PI / 180.0;
    let lat1 = p1.lat * rad;
    let lng1 = p1.lng * rad;
    let lat2 = p2.lat * rad;
    let lng2 = p2.lng * rad;
    let longitude = lng2 - lng1;
    let latitude = lat2 - lat1;
    let a = pow2((latitude / 2.0).sin()) + lat1.cos() * lat2.cos() * pow2((longitude / 2.0).sin());
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    (6_378_137.0 / 1000.0) * c
}

/// `tabControl1`'s pages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    /// `tabSimple`, "Simple".
    Simple,
    /// `tabGrid`, "Grid Options".
    Grid,
    /// `tabCamera`, "Camera Config".
    Camera,
}

impl Tab {
    /// The page's name in the Designer.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Simple => "tabSimple",
            Self::Grid => "tabGrid",
            Self::Camera => "tabCamera",
        }
    }

    /// Its text in the `.resx`.
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Simple => "Simple",
            Self::Grid => "Grid Options",
            Self::Camera => "Camera Config",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Vec<LatLon> {
        [
            (-35.3600, 149.1600),
            (-35.3600, 149.1710),
            (-35.3690, 149.1710),
            (-35.3690, 149.1600),
        ]
        .into_iter()
        .map(|(lat, lng)| LatLon::new(lat, lng).expect("a vertex"))
        .collect()
    }

    fn open(context: &Context) -> Dialog {
        Dialog::open(&square(), Cameras::builtin(), context, &|_| None)
    }

    #[test]
    fn the_defaults_are_the_designers() {
        let dialog = open(&Context::default());
        assert_eq!(dialog.num_text(Num::Altitude), "100");
        assert_eq!(dialog.num_text(Num::FlySpeed), "5.0");
        assert_eq!(dialog.num_text(Num::Distance), "50.00");
        assert_eq!(dialog.num_text(Num::Overlap), "50.0");
        assert_eq!(dialog.num_text(Num::Sidelap), "60.0");
        assert_eq!(dialog.num_text(Num::Angle), "180");
        assert_eq!(dialog.text(Text::HeadingHold), "180");
        assert!(dialog.check(Check::ToAndLand) && dialog.check(Check::StopStart));
        assert!(dialog.trigger(Trigger::Distance));
        assert_eq!(dialog.tabs(), [Tab::Simple]);
        assert_eq!(dialog.recomputes(), 1);
        assert_eq!(dialog.grid().len(), 80);
        assert_eq!(dialog.stat(Stat::Strips), "20");
        assert_eq!(dialog.camera_items().len(), 31);
    }

    /// Save under a new name: the camera added at the end of the dictionary with the boxes'
    /// values, `CMB_camera` showing the name, its list unchanged, and the file to write holding
    /// it - which the next dialog reads back.
    /// `// C#: Grid/GridUI.cs:1567-1602`
    #[test]
    fn save_adds_a_camera_under_a_new_name() {
        let mut dialog = open(&Context::default());
        dialog.type_num(Num::FocalLength, "8.8");
        dialog.set_text(Text::ImgWidth, "5472");
        dialog.set_text(Text::ImgHeight, "3648");
        dialog.set_text(Text::SensWidth, "13.2");
        dialog.set_text(Text::SensHeight, "8.8");
        let held = dialog.save_camera("Mine").expect("numbers").clone();
        assert_eq!(dialog.camera(), "Mine");
        assert_eq!(
            dialog.camera_items().len(),
            31,
            "the list is filled only on reading"
        );
        assert_eq!(
            held.get("Mine"),
            Some(&CameraInfo {
                name: "Mine".to_owned(),
                focallen: 8.8,
                sensorwidth: 13.2,
                sensorheight: 8.8,
                imagewidth: 5472.0,
                imageheight: 3648.0,
            })
        );
        let mut reread = Cameras::builtin();
        reread.read(&String::from_utf8(held.to_xml()).unwrap());
        assert_eq!(reread.items().last().map(String::as_str), Some("Mine"));
        assert_eq!(reread.get("Mine"), held.get("Mine"));
    }

    /// A box that is not a number: the message box, and nothing to write - though the camera
    /// was added and the values before the bad box were set, as the C#'s are.
    #[test]
    fn save_with_a_bad_box_says_so() {
        let mut dialog = open(&Context::default());
        dialog.type_num(Num::FocalLength, "8.8");
        dialog.set_text(Text::ImgHeight, "3648");
        dialog.set_text(Text::ImgWidth, "wide");
        assert!(matches!(
            dialog.save_camera("Mine"),
            Err(Refused::Message(NOT_A_NUMBER, ""))
        ));
        let held = dialog.cameras.get("Mine").expect("added before the parse");
        assert_eq!(
            (held.focallen, held.imageheight, held.imagewidth),
            (8.8, 3648.0, 0.0)
        );
    }

    /// Save under a listed camera's name, in another case: `CMB_camera.Text = name` selects that
    /// camera, whose `SelectedIndexChanged` loads it over the boxes - so what is saved is the
    /// listed camera as it was, under its own name.
    /// `// C#: Grid/GridUI.cs:1576-1582, 1344-1362`
    #[test]
    fn save_under_a_listed_name_reloads_that_camera_first() {
        let mut dialog = open(&Context::default());
        dialog.type_num(Num::FocalLength, "8.8");
        let held = dialog
            .save_camera("canon sx230 hs")
            .expect("numbers")
            .clone();
        assert_eq!(dialog.camera(), "Canon SX230 HS");
        assert_eq!(
            held.get("Canon SX230 HS"),
            Cameras::builtin().get("Canon SX230 HS")
        );
        assert_eq!(dialog.text(Text::ImgWidth), "4000");
        assert!(held.get("canon sx230 hs").is_none());
    }

    #[test]
    fn a_plane_starts_at_twelve() {
        let dialog = open(&Context {
            plane: true,
            ..Context::default()
        });
        assert_eq!(dialog.num_text(Num::FlySpeed), "12.0");
    }

    #[test]
    fn advanced_options_shows_the_other_tabs() {
        let mut dialog = open(&Context::default());
        dialog.click_check(Check::Advanced);
        assert_eq!(dialog.tabs(), [Tab::Simple, Tab::Grid, Tab::Camera]);
        dialog.click_check(Check::Advanced);
        assert_eq!(dialog.tabs(), [Tab::Simple]);
    }

    #[test]
    fn the_arrows_step_and_stop_at_the_range() {
        let mut dialog = open(&Context::default());
        dialog.up(Num::Altitude);
        assert_eq!(dialog.num_text(Num::Altitude), "110");
        dialog.type_num(Num::Altitude, "5");
        dialog.down(Num::Altitude);
        assert_eq!(dialog.num_text(Num::Altitude), "1");
        dialog.type_num(Num::Altitude, "123456");
        assert_eq!(dialog.num_text(Num::Altitude), "99999");
        dialog.type_num(Num::Altitude, "not a number");
        assert_eq!(dialog.num_text(Num::Altitude), "99999");
        dialog.up(Num::CopterDelay);
        assert_eq!(dialog.num(Num::CopterDelay).to_text(), "0.1");
    }

    #[test]
    fn heading_hold_enables_and_steps() {
        let mut dialog = open(&Context::default());
        assert!(!dialog.heading_enabled());
        dialog.click_check(Check::HeadingHold);
        assert!(dialog.heading_enabled());
        dialog.heading_plus();
        assert_eq!(dialog.text(Text::HeadingHold), "0");
        dialog.click_check(Check::HeadingHoldLock);
        assert!(!dialog.heading_read_only());
        dialog.heading_minus();
        assert_eq!(dialog.text(Text::HeadingHold), "359");
        dialog.click_check(Check::HeadingHoldLock);
        assert_eq!(dialog.text(Text::HeadingHold), "180");
    }

    #[test]
    fn saved_settings_come_back_when_a_camera_was_saved() {
        let dialog = open(&Context::default());
        let mut saved: std::collections::BTreeMap<&str, String> =
            dialog.save_settings().into_iter().collect();
        saved.insert("grid_alt", "120".to_owned());
        saved.insert("grid_camera", "Canon SX230 HS".to_owned());
        saved.insert("grid_advanced", "True".to_owned());
        saved.insert("grid_digicam", "True".to_owned());
        saved.insert("grid_trigdist", "False".to_owned());
        let reopened = Dialog::open(&square(), Cameras::builtin(), &Context::default(), &|key| {
            saved.get(key).cloned()
        });
        assert_eq!(reopened.num_text(Num::Altitude), "120");
        assert_eq!(reopened.camera(), "Canon SX230 HS");
        assert_eq!(reopened.text(Text::ImgWidth), "4000");
        assert!(reopened.trigger(Trigger::Digicam));
        assert!(!reopened.trigger(Trigger::Distance));
        assert_eq!(reopened.tabs().len(), 3);
        // Loading does not redraw; the one redraw is Load's own.
        assert_eq!(reopened.recomputes(), 1);
        // Without grid_camera nothing is loaded.
        saved.remove("grid_camera");
        let fresh = Dialog::open(&square(), Cameras::builtin(), &Context::default(), &|key| {
            saved.get(key).cloned()
        });
        assert_eq!(fresh.num_text(Num::Altitude), "100");
    }

    #[test]
    fn splitting_rounds_half_away() {
        assert_eq!(wpsplit(81, Decimal::from_int(2)), 41);
        assert_eq!(wpsplit(80, Decimal::from_int(3)), 27);
        assert_eq!(wpsplit(80, Decimal::new(25, 1)), 32);
    }

    #[test]
    fn seconds_read_as_the_dialog_writes_them() {
        assert_eq!(seconds_to_nice(5550.0), "1:32:30 Hours");
        assert_eq!(seconds_to_nice(125.4), "2:05 Minutes");
        assert_eq!(seconds_to_nice(9.256), "9.26 Seconds");
        assert_eq!(seconds_to_nice(-1.0), "Infinity Seconds");
        assert_eq!(seconds_to_nice(f64::NAN), "NaN Seconds");
        assert_eq!(seconds_to_nice(f64::INFINITY), "NaN Seconds");
    }
}
