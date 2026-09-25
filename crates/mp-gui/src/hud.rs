//! Primary flight display (DELIVERABLES.md D9).
//!
//! Ported from `ExtLibs/Controls/HUD.cs` @ efb0801 (GPL-3.0-or-later): the instrument geometry of
//! `doPaint()` (lines 1954-3333), drawn with gpui paths and text instead of GDI+ and OpenGL. The
//! ~2,400 lines of 2D graphics abstraction around it - the dual GL/GDI backends, the per-glyph
//! texture atlas, the stencil fills - are not ported; gpui owns the atlas and the GPU (PLAN.md
//! §9.2), and the 30 ms self-throttle at `HUD.cs:1284` is gone with them.
//!
//! # Shape
//!
//! [`scene`] turns [`HudInputs`] into a [`Scene`]: an ordered list of fills, strokes and labels in
//! canvas coordinates, plus the list of [`Element`]s it drew. It is pure, so the geometry is
//! tested without a window - where the horizon sits at a roll, where the target bug lands on the
//! heading tape, that ARMED goes away after eight seconds. [`paint`] then puts a scene on a gpui
//! canvas. The split is also what D9's golden-frame tests will drive.
//!
//! [`ELEMENTS`] is the coverage table: every element `doPaint()` draws, with its C# lines and
//! whether this file draws it. A test holds the table to what [`scene`] produces, so the table
//! cannot claim more than the code does, and what is missing is listed rather than forgotten.
//! An element whose drawing is ported but whose value `mp_vehicle` does not carry is
//! [`Status::Blocked`], with the `CurrentState` property it waits on named in the table. None
//! is: all 24 draw from a live vehicle.
//!
//! # Conventions, stated because getting them wrong is invisible in code review
//!
//! * Roll is positive with the right wing down. On screen the horizon then tilts so its **right
//!   end rises**, because the display shows the world as seen from the aircraft, not the aircraft
//!   as seen from the world. An artificial horizon that rolls the wrong way looks plausible in a
//!   screenshot and is lethal in cloud.
//! * Pitch is positive nose-up, and the horizon moves **down** the screen as the nose rises.
//! * Screen y grows downward, so every "up" in this file is negative y.
//! * Sizes follow the C#'s: the font is `height / 30`, the pitch scale `height / 65` pixels per
//!   degree, the heading tape `height / 14` tall, the scrollers `width / 10` wide. The display
//!   scales with its box rather than with a constant, as the original does.
//! * [`HudInputs`] holds the vehicle's values in SI, as `mp_vehicle` keeps them, and the user's
//!   [`DisplayUnits`] beside them; [`scene`] multiplies each by its multiplier where the C#'s
//!   `CurrentState` getter does before the binding hands it to the HUD, and writes the unit
//!   names `FlightData.Activate` gives the HUD (`altunit`, `speedunit`, `distunit`).
//!
//! # The pictures
//!
//! With `displayicons` on, the battery, the GPS fix, Vibe, EKF and the pre-arm line are drawn
//! as `HUDT`'s bitmaps (`ExtLibs/Controls/Resources/*.png`) at the rectangles `doPaint()` gives
//! `DrawImage`. The scene records which picture goes where ([`Item::Icon`]); the bitmaps are
//! Mission Planner's artwork and not files this application ships (as `config/frame_type.rs`
//! does not ship its frame pictures), so [`paint`] draws each as what it shows - a coloured
//! badge with its words, or a battery with its bars - in the bitmap's colours, in its rectangle.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::time::{Duration, Instant};

use gpui::{
    Bounds, Hsla, PathBuilder, Pixels, Point, SharedString, TextAlign, TextRun, Window, point, px,
    rgb,
};
use mp_vehicle::VehicleState;
use mp_vehicle::units::DisplayUnits;

/// Golden frames: the scenes of recorded flights and of the hard cases, drawn by [`raster`] and
/// held to the images committed under `testdata/hud/`.
#[cfg(test)]
mod golden;
/// A software rasteriser for a [`Scene`], so a frame can be drawn and compared without a window.
#[cfg(test)]
mod raster;

mod colour {
    /// Sky above the horizon.
    pub const SKY: u32 = 0x2f_6d_9e;
    /// Ground below it.
    pub const GROUND: u32 = 0x6b_4f_2a;
    /// Ladder, reticle, scales and most text: the C#'s white pen and brush.
    pub const INK: u32 = 0xff_ff_ff;
    /// Roll pointer, DISARMED, FAILSAFE, a lost GPS: the C#'s red.
    pub const ALERT: u32 = 0xf8_51_49;
    /// Target bugs and the 0° rung: the C#'s green pen.
    pub const TARGET: u32 = 0x3c_c8_5a;
    /// A low battery, and Vibe or EKF past its first threshold: the C#'s orange brush.
    pub const WARN: u32 = 0xff_a5_00;
    /// The VSI's climb polygon and the AOA scale's lowest band: `Brushes.Blue`.
    pub const VSI: u32 = 0x3b_7d_ff;
    /// The AOA scale's caution band: `Brushes.Yellow`.
    pub const CAUTION: u32 = 0xff_ff_00;
    /// Ground under the altitude tape: `AltGroundBrush`, burlywood at alpha 100.
    pub const GROUND_TAPE: u32 = 0xde_b8_87;
    /// The ground-course mark and the scroller arrows: black.
    pub const BLACK: u32 = 0x00_00_00;
    /// The heading tape's centre box: `SlightlyTransparentWhiteBrush`.
    pub const READOUT: u32 = 0xff_ff_ff;
    // The pictures' colours, read from the bitmaps' pixels.
    // `// C#: ExtLibs/Controls/Resources/*.png`
    /// The green badges - 3D FIX, 3D DGPS, Unknown, VIBE, EKF, Ready to Arm - at alpha 179.
    pub const ICON_GREEN: u32 = 0x04_a2_13;
    /// The amber badges: 2D FIX, VIBE, EKF.
    pub const ICON_AMBER: u32 = 0xff_b3_21;
    /// The red badges: NO GPS, NO FIX, VIBE, EKF, Not Ready to Arm.
    pub const ICON_RED: u32 = 0xc2_05_05;
    /// The violet badges: RTKFloat, RTKFixed.
    pub const ICON_VIOLET: u32 = 0x53_3c_ff;
    /// The battery's outline.
    pub const BATTERY_OUTLINE: u32 = 0xff_ff_ff;
    /// `batt_red`'s outline.
    pub const BATTERY_RED: u32 = 0xff_00_2a;
    /// The battery's green bars.
    pub const BATTERY_GREEN: u32 = 0x0b_ff_05;
    /// `batt_yellow`'s bar.
    pub const BATTERY_YELLOW: u32 = 0xff_cc_00;
}

/// One of `HUDT`'s pictures, named as its file in `ExtLibs/Controls/Resources`.
/// `// C#: ExtLibs/Controls/HUDT.resx batt_1 .. _3dfix_wide`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Icon {
    /// `batt_1`: one green bar.
    Batt1,
    /// `batt_2`.
    Batt2,
    /// `batt_3`.
    Batt3,
    /// `batt_4`: four green bars.
    Batt4,
    /// `batt_red`: a red outline, empty.
    BattRed,
    /// `batt_yellow`: one yellow bar.
    BattYellow,
    /// `nogps_wide`.
    NoGps,
    /// `nofix_wide`.
    NoFix,
    /// `_2dfix_wide`.
    Fix2d,
    /// `_3dfix_wide`.
    Fix3d,
    /// `_3ddgps_wide`.
    Dgps3d,
    /// `rtkfloat_wide`.
    RtkFloat,
    /// `rtkfixed_wide`.
    RtkFixed,
    /// `unknown`: a fix type past 6.
    Unknown,
    /// `vibe_green`.
    VibeGreen,
    /// `vibe_yellow`.
    VibeYellow,
    /// `vibe_red`.
    VibeRed,
    /// `ekf_green`.
    EkfGreen,
    /// `ekf_yellow`.
    EkfYellow,
    /// `ekf_red`.
    EkfRed,
    /// `prearm_green`.
    PrearmGreen,
    /// `prearm_red`.
    PrearmRed,
}

/// What a picture shows, for the stand-in [`paint`] draws.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Look {
    /// A badge of one colour at alpha 179 with white words across it.
    Badge { colour: u32, text: &'static str },
    /// A battery: an outline with a cap, and `bars` bars filled from the bottom.
    Battery { outline: u32, bars: u8, bar: u32 },
}

impl Icon {
    /// The file's name, for the facts.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Batt1 => "batt_1",
            Self::Batt2 => "batt_2",
            Self::Batt3 => "batt_3",
            Self::Batt4 => "batt_4",
            Self::BattRed => "batt_red",
            Self::BattYellow => "batt_yellow",
            Self::NoGps => "nogps_wide",
            Self::NoFix => "nofix_wide",
            Self::Fix2d => "2dfix_wide",
            Self::Fix3d => "3dfix_wide",
            Self::Dgps3d => "3ddgps_wide",
            Self::RtkFloat => "rtkfloat_wide",
            Self::RtkFixed => "rtkfixed_wide",
            Self::Unknown => "unknown",
            Self::VibeGreen => "vibe_green",
            Self::VibeYellow => "vibe_yellow",
            Self::VibeRed => "vibe_red",
            Self::EkfGreen => "ekf_green",
            Self::EkfYellow => "ekf_yellow",
            Self::EkfRed => "ekf_red",
            Self::PrearmGreen => "prearm_green",
            Self::PrearmRed => "prearm_red",
        }
    }

    /// What the bitmap shows: its colour and its words, or its battery. Read from the files.
    fn look(self) -> Look {
        use colour::{
            BATTERY_GREEN, BATTERY_OUTLINE, BATTERY_RED, BATTERY_YELLOW, ICON_AMBER, ICON_GREEN,
            ICON_RED, ICON_VIOLET,
        };
        let badge = |colour, text| Look::Badge { colour, text };
        let battery = |outline, bars, bar| Look::Battery { outline, bars, bar };
        match self {
            Self::Batt1 => battery(BATTERY_OUTLINE, 1, BATTERY_GREEN),
            Self::Batt2 => battery(BATTERY_OUTLINE, 2, BATTERY_GREEN),
            Self::Batt3 => battery(BATTERY_OUTLINE, 3, BATTERY_GREEN),
            Self::Batt4 => battery(BATTERY_OUTLINE, 4, BATTERY_GREEN),
            Self::BattRed => battery(BATTERY_RED, 0, BATTERY_RED),
            Self::BattYellow => battery(BATTERY_OUTLINE, 1, BATTERY_YELLOW),
            Self::NoGps => badge(ICON_RED, "NO GPS"),
            Self::NoFix => badge(ICON_RED, "NO FIX"),
            Self::Fix2d => badge(ICON_AMBER, "2D FIX"),
            Self::Fix3d => badge(ICON_GREEN, "3D FIX"),
            Self::Dgps3d => badge(ICON_GREEN, "3D DGPS"),
            Self::RtkFloat => badge(ICON_VIOLET, "RTKFloat"),
            Self::RtkFixed => badge(ICON_VIOLET, "RTKFixed"),
            Self::Unknown => badge(ICON_GREEN, "Unknown"),
            Self::VibeGreen => badge(ICON_GREEN, "VIBE"),
            Self::VibeYellow => badge(ICON_AMBER, "VIBE"),
            Self::VibeRed => badge(ICON_RED, "VIBE"),
            Self::EkfGreen => badge(ICON_GREEN, "EKF"),
            Self::EkfYellow => badge(ICON_AMBER, "EKF"),
            Self::EkfRed => badge(ICON_RED, "EKF"),
            Self::PrearmGreen => badge(ICON_GREEN, "Ready to Arm"),
            Self::PrearmRed => badge(ICON_RED, "Not Ready to Arm"),
        }
    }

    /// The GPS fix's picture. `// C#: ExtLibs/Controls/HUD.cs:2936-2986`
    #[must_use]
    pub const fn gps(fix_type: u8) -> Self {
        match fix_type {
            0 => Self::NoGps,
            1 => Self::NoFix,
            2 => Self::Fix2d,
            3 => Self::Fix3d,
            4 => Self::Dgps3d,
            5 => Self::RtkFloat,
            6 => Self::RtkFixed,
            _ => Self::Unknown,
        }
    }
}

/// One thing `doPaint()` draws.
///
/// Named as a pilot would name them, in the order the C# paints them; `csharp_lines` says where.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Element {
    /// The sky and ground polygons.
    SkyGround,
    /// The pitch ladder, with the 0° rung as the horizon line.
    PitchLadder,
    /// The roll scale and its pointer.
    RollIndicator,
    /// The fixed aircraft symbol.
    Reticle,
    /// The flight path vector (AOA/SSA).
    FlightPathVector,
    /// The heading tape along the top.
    HeadingTape,
    /// The target-heading and ground-course marks on it.
    HeadingBugs,
    /// The cross-track error bar.
    XtrackBar,
    /// The rate-of-turn marks under it.
    RateOfTurn,
    /// The speed scroller on the left, with its airspeed and groundspeed lines.
    SpeedTape,
    /// The altitude scroller on the right, with the ground fill.
    AltitudeTape,
    /// The vertical speed indicator beside the altitude scroller.
    Vsi,
    /// Flight mode, distance and number of the current waypoint.
    ModeAndWaypoint,
    /// Link quality bars and the clock.
    LinkInfo,
    /// The angle-of-attack scale.
    Aoa,
    /// Battery voltage, current and remaining.
    Battery,
    /// GPS fix.
    Gps,
    /// User-configured extra fields.
    CustomItems,
    /// ARMED / DISARMED / SAFE.
    ArmedBanner,
    /// The FAILSAFE banner.
    Failsafe,
    /// The high-priority message line.
    Message,
    /// The vibration indicator, and the CPU warning drawn beside it.
    Vibe,
    /// The EKF indicator.
    Ekf,
    /// The pre-arm indicator.
    Prearm,
}

impl Element {
    /// The name a fact or a test uses.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::SkyGround => "sky-ground",
            Self::PitchLadder => "pitch-ladder",
            Self::RollIndicator => "roll-indicator",
            Self::Reticle => "reticle",
            Self::FlightPathVector => "flight-path-vector",
            Self::HeadingTape => "heading-tape",
            Self::HeadingBugs => "heading-bugs",
            Self::XtrackBar => "xtrack-bar",
            Self::RateOfTurn => "rate-of-turn",
            Self::SpeedTape => "speed-tape",
            Self::AltitudeTape => "altitude-tape",
            Self::Vsi => "vsi",
            Self::ModeAndWaypoint => "mode-and-waypoint",
            Self::LinkInfo => "link-info",
            Self::Aoa => "aoa",
            Self::Battery => "battery",
            Self::Gps => "gps",
            Self::CustomItems => "custom-items",
            Self::ArmedBanner => "armed-banner",
            Self::Failsafe => "failsafe",
            Self::Message => "message",
            Self::Vibe => "vibe",
            Self::Ekf => "ekf",
            Self::Prearm => "prearm",
        }
    }

    /// Where the C# draws it: `ExtLibs/Controls/HUD.cs` lines within `doPaint()`.
    #[must_use]
    pub const fn csharp_lines(self) -> (u32, u32) {
        match self {
            Self::SkyGround => (2031, 2101),
            Self::PitchLadder => (2103, 2151),
            Self::RollIndicator => (2153, 2208),
            Self::Reticle => (2209, 2231),
            Self::FlightPathVector => (2232, 2252),
            Self::HeadingTape => (2253, 2396),
            Self::HeadingBugs => (2270, 2302),
            Self::XtrackBar => (2397, 2438),
            Self::RateOfTurn => (2439, 2483),
            Self::SpeedTape => (2484, 2582),
            Self::AltitudeTape => (2583, 2659),
            Self::Vsi => (2660, 2736),
            Self::ModeAndWaypoint => (2737, 2772),
            Self::LinkInfo => (2773, 2803),
            Self::Aoa => (2804, 2845),
            Self::Battery => (2855, 2925),
            Self::Gps => (2926, 3028),
            Self::CustomItems => (3029, 3083),
            Self::ArmedBanner => (3084, 3119),
            Self::Failsafe => (3120, 3126),
            Self::Message => (3128, 3145),
            Self::Vibe => (3148, 3208),
            Self::Ekf => (3209, 3263),
            Self::Prearm => (3264, 3313),
        }
    }
}

/// Whether this file draws an element.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Drawn by [`scene`] from what the vehicle reports.
    Drawn,
    /// Drawn by [`scene`] when [`HudInputs`] carries its value, but `mp_vehicle` does not keep
    /// that value, so [`HudInputs::from_vehicle`] cannot supply it and a live display never
    /// shows the element. The text names the `CurrentState` property and why it is absent.
    ///
    /// No element is blocked now - the last two, the flight path vector and the AOA scale, draw
    /// from `AOA_SSA` and the `AOA_CRIT` parameter - so nothing constructs this. It stays so that
    /// an element whose value is lost has a row to fall back to, and `hud.missing` a count.
    #[allow(dead_code)]
    Blocked(&'static str),
}

/// The coverage table: every element `doPaint()` draws.
pub const ELEMENTS: &[(Element, Status)] = &[
    (Element::SkyGround, Status::Drawn),
    (Element::PitchLadder, Status::Drawn),
    (Element::RollIndicator, Status::Drawn),
    (Element::Reticle, Status::Drawn),
    // From `AOA` and `SSA`, once the vehicle has sent a non-zero one (`displayAOASSA`).
    // C#: ExtLibs/ArduPilot/CurrentState.cs:3903-3911, ExtLibs/Controls/HUD.cs:889-930
    (Element::FlightPathVector, Status::Drawn),
    (Element::HeadingTape, Status::Drawn),
    (Element::HeadingBugs, Status::Drawn),
    (Element::XtrackBar, Status::Drawn),
    (Element::RateOfTurn, Status::Drawn),
    (Element::SpeedTape, Status::Drawn),
    (Element::AltitudeTape, Status::Drawn),
    (Element::Vsi, Status::Drawn),
    (Element::ModeAndWaypoint, Status::Drawn),
    (Element::LinkInfo, Status::Drawn),
    // From `AOA` against `crit_AOA`, the `AOA_CRIT` parameter, shown with the flight path
    // vector. C#: ExtLibs/ArduPilot/CurrentState.cs:1017-1036, 3903-3911
    (Element::Aoa, Status::Drawn),
    (Element::Battery, Status::Drawn),
    (Element::Gps, Status::Drawn),
    // Drawn from a given list. There is no editor for the list and no stored one yet: the C#'s
    // checkboxes and `hud1_useritem_` settings are not ported (GCSViews/FlightData.cs:336-348,
    // 2436-2472), so on a live vehicle the list is empty.
    (Element::CustomItems, Status::Drawn),
    (Element::ArmedBanner, Status::Drawn),
    (Element::Failsafe, Status::Drawn),
    (Element::Message, Status::Drawn),
    // Vibe, and the "CPU" beside it (HUD.cs:3206-3207) from `load`, `SYS_STATUS.load` / 10.
    // C#: ExtLibs/ArduPilot/CurrentState.cs:2949
    (Element::Vibe, Status::Drawn),
    (Element::Ekf, Status::Drawn),
    (Element::Prearm, Status::Drawn),
];

/// The elements a live display cannot show, with their C# lines and what each waits on, one
/// string, for the facts and the plan. "none" when there are none, as the other facts say it: a
/// `.gui` script cannot expect an empty value.
#[must_use]
pub fn missing_report() -> String {
    let report = ELEMENTS
        .iter()
        .filter_map(|(element, status)| match status {
            Status::Blocked(why) => {
                let (from, to) = element.csharp_lines();
                Some(format!("{} (HUD.cs:{from}-{to}) {why}", element.name()))
            }
            Status::Drawn => None,
        })
        .collect::<Vec<_>>()
        .join("; ");
    if report.is_empty() {
        "none".to_owned()
    } else {
        report
    }
}

/// The elements a live display cannot show: those the table marks [`Status::Blocked`].
#[must_use]
pub fn missing() -> Vec<Element> {
    ELEMENTS
        .iter()
        .filter(|(_, status)| matches!(status, Status::Blocked(_)))
        .map(|(element, _)| *element)
        .collect()
}

/// How long ARMED stays up after arming.
/// `// C#: ExtLibs/Controls/HUD.cs:3106`
pub const ARMED_BANNER: Duration = Duration::from_secs(8);
/// How long the mode name shows red after a mode change.
/// `// C#: ExtLibs/Controls/HUD.cs:2738`
pub const MODE_CHANGE_FLASH: Duration = Duration::from_secs(2);
/// How long a high-priority message stays on the display.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1271`
pub const MESSAGE_LIFETIME: Duration = Duration::from_secs(10);

/// The clocks the display keeps between frames: when arming happened and when the mode changed.
///
/// The vehicle state says *whether* it is armed; the C# HUD shows ARMED for eight seconds after
/// the transition and then nothing, which needs somebody to remember the transition. This is that
/// somebody, fed once a frame.
#[derive(Debug, Default)]
pub struct Timing {
    armed: Option<bool>,
    armed_at: Option<Instant>,
    mode: Option<u32>,
    mode_changed_at: Option<Instant>,
    message: Option<(String, Instant)>,
    display_aoa_ssa: bool,
}

impl Timing {
    /// Notes this frame's armed state and mode; returns how long ago each last changed.
    ///
    /// `None` for "never seen a transition", which the banner treats as "not recently".
    pub fn observe(
        &mut self,
        armed: bool,
        mode: u32,
        now: Instant,
    ) -> (Option<Duration>, Option<Duration>) {
        if self.armed != Some(armed) {
            if self.armed.is_some() {
                self.armed_at = Some(now);
            }
            self.armed = Some(armed);
        }
        if self.mode != Some(mode) {
            if self.mode.is_some() {
                self.mode_changed_at = Some(now);
            }
            self.mode = Some(mode);
        }
        (
            self.armed_at.map(|at| now.saturating_duration_since(at)),
            self.mode_changed_at
                .map(|at| now.saturating_duration_since(at)),
        )
    }

    /// Notes this frame's angle of attack and sideslip; returns whether the display shows them:
    /// the C#'s `displayAOASSA`.
    ///
    /// Off when the display is made, and turned on by the `AOA` or `SSA` setter when the bound
    /// value differs from the one held, which starts at 0 - so the first time either angle is
    /// not 0. Nothing turns it off again, not a later 0 and not a new vehicle. `!=` is the C#'s
    /// float comparison: -0 is 0, and a NaN turns it on.
    /// `// C#: ExtLibs/Controls/HUD.cs:276, 352-353, 889-930; GCSViews/FlightData.Designer.cs:400`
    pub fn display_aoa_ssa(&mut self, aoa: f32, ssa: f32) -> bool {
        if aoa != 0.0 || ssa != 0.0 {
            self.display_aoa_ssa = true;
        }
        self.display_aoa_ssa
    }
}

/// GPS fix as the C# names it.
/// `// C#: ExtLibs/Controls/HUDT.resx GPS0-GPS6`
#[must_use]
pub fn gps_fix_text(fix_type: u8) -> String {
    match fix_type {
        0 => "GPS: No GPS".to_owned(),
        1 => "GPS: No Fix".to_owned(),
        2 => "GPS: 2D Fix".to_owned(),
        3 => "GPS: 3D Fix".to_owned(),
        4 => "GPS: 3D dgps".to_owned(),
        5 => "GPS: rtk Float".to_owned(),
        6 => "GPS: rtk Fixed".to_owned(),
        other => other.to_string(),
    }
}

/// The distance to the waypoint as the HUD writes it, given in the user's distance unit: whole
/// units under 1000, cut toward zero; from 1000, kilometres to one place ("k") where the unit
/// is "m", and otherwise - feet, or no unit set - miles of 5280 ("mi"). The place is
/// `Math.Round(double, 1)`'s, halves to even, and the number is written as a `float` writes.
/// `// C#: ExtLibs/Controls/HUD.cs:2749-2769`
#[must_use]
pub fn wp_distance_text(distance: f32, unit: &str) -> String {
    if distance >= 1000.0 {
        let (per, shown) = if unit == "m" {
            (1000.0, "k")
        } else {
            (5280.0, "mi")
        };
        // C#'s `(float)Math.Round(newdist / 1000.0, 1)`: the double scaled, rounded, unscaled;
        // then `newdist + newdistunit`, the float's own `ToString()`.
        #[allow(clippy::cast_possible_truncation)] // the C#'s (float)
        let rounded = ((f64::from(distance) / per * 10.0).round_ties_even() / 10.0) as f32;
        format!(
            "{}{shown}",
            crate::config::battery_monitor::float_text(rounded)
        )
    } else {
        #[allow(clippy::cast_possible_truncation)] // the C#'s (int)
        let whole = distance as i32;
        format!("{whole}{unit}")
    }
}

/// The angles the flight path vector and the AOA scale are drawn from.
/// `// C#: ExtLibs/Controls/HUD.cs:352-358, 889-945`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AoaSsa {
    /// Angle of attack, degrees: `AOA`.
    pub aoa: f32,
    /// Sideslip angle, degrees: `SSA`.
    pub ssa: f32,
    /// The critical angle of attack, degrees: `critAOA`, bound to `CurrentState.crit_AOA` - the
    /// `AOA_CRIT` parameter cut to a whole number, or 25 without one. See [`crit_aoa`].
    pub crit_aoa: f32,
}

/// `CurrentState.crit_AOA`: the vehicle's `AOA_CRIT` parameter, cast to `int` - so cut toward
/// zero, 15.9 reading as 15 - or 25 when the vehicle has no such parameter, the name matched
/// exactly as `MAVLinkParamList.ContainsKey` matches it.
///
/// The C#'s `catch` returning 0 guards a vehicle object that is not there; here the table
/// always is, empty until the parameters are read. A NaN or a value past `int`'s range is
/// unspecified for the C#'s unchecked cast; Rust's `as` saturates, and NaN reads 0.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1017-1036, ExtLibs/Mavlink/MAVLinkParamList.cs:114-122`
#[must_use]
#[allow(clippy::cast_possible_truncation)] // the C#'s `(int)`, truncation is the point
pub fn crit_aoa(parameters: &[(String, f64)]) -> f32 {
    parameters
        .iter()
        .find(|(name, _)| name == "AOA_CRIT")
        .map_or(25.0, |(_, value)| *value as i32 as f32)
}

/// One of the user's extra fields: `HUD.Custom`.
///
/// Mission Planner keeps them in a `Hashtable` keyed by the name of the `CurrentState` property
/// shown, and reads each value by reflection as it paints. There is no reflection here, so the
/// caller reads the value and hands it over. The C# draws them in the `Hashtable`'s enumeration
/// order, which nothing defines; these are drawn in the list's order.
/// `// C#: ExtLibs/Controls/HUD.cs:949-968, GCSViews/FlightData.cs:948-955`
#[derive(Debug, Clone, PartialEq)]
pub struct CustomItem {
    /// The prefix the user typed: `Header`.
    pub header: String,
    /// The `CurrentState` property shown: `Item.Name`. It also picks the number format.
    pub name: String,
    /// The property's value: `GetValue`. `None` when it cannot be read - no such property, or
    /// one that is not a number - which the C# skips without leaving a gap.
    pub value: Option<f64>,
}

/// Everything the display draws from: the vehicle's values in SI, and [`HudInputs::units`] to
/// show them in, which [`scene`] applies where the C#'s `CurrentState` getters apply them.
#[derive(Debug, Clone, PartialEq)]
pub struct HudInputs {
    /// Whether there is a vehicle at all. Without one the display shows the horizon at rest and
    /// nothing else.
    pub has_vehicle: bool,
    /// Roll, degrees, right wing down positive.
    pub roll: f32,
    /// Pitch, degrees, nose up positive.
    pub pitch: f32,
    /// Heading, degrees.
    pub heading: f32,
    /// The controller's desired heading, degrees: the green mark.
    pub target_heading: f32,
    /// Ground course, degrees: the black mark.
    pub ground_course: f32,
    /// Cross-track error, metres.
    pub xtrack_error: f32,
    /// Rate of turn, degrees per second.
    pub turn_rate: f32,
    /// Airspeed, m/s. Zero means "not measured", and the tape shows ground speed instead.
    pub airspeed: f32,
    /// Ground speed, m/s.
    pub ground_speed: f32,
    /// The controller's target airspeed, m/s; zero draws no mark.
    pub target_speed: f32,
    /// Altitude, metres, relative to home.
    pub altitude: f32,
    /// The controller's target altitude, metres; zero draws no mark.
    pub target_altitude: f32,
    /// Ground level in the altitude tape's frame; zero draws no ground.
    pub ground_altitude: f32,
    /// Vertical speed, m/s, up positive.
    pub vertical_speed: f32,
    /// Flight mode name.
    pub mode: String,
    /// How long ago the mode changed, if it has.
    pub mode_changed_for: Option<Duration>,
    /// Distance to the current waypoint, metres.
    pub wp_distance: f32,
    /// The current waypoint's number.
    pub wp_number: u16,
    /// Link quality, percent.
    pub link_quality: u8,
    /// The clock, `HH:MM:SS`.
    pub clock: String,
    /// Battery voltage.
    pub battery_voltage: f32,
    /// Battery current, amps.
    pub battery_current: f32,
    /// Battery remaining, percent.
    pub battery_remaining: i8,
    /// Whether the battery is below the low-voltage alert.
    pub battery_low: bool,
    /// Whether it is below the critical alert.
    pub battery_critical: bool,
    /// GPS fix type.
    pub gps_fix: u8,
    /// Whether the vehicle is armed.
    pub armed: bool,
    /// How long ago the armed state last changed, if it has since the display started.
    pub armed_for: Option<Duration>,
    /// Whether the vehicle reports a failsafe (`MAV_STATE_CRITICAL`).
    pub failsafe: bool,
    /// Whether the safety switch is holding the motors.
    pub safety: bool,
    /// The high-priority message, with its colour.
    pub message: Option<(String, u32)>,
    /// The angle of attack and sideslip. `None` is the C#'s `displayAOASSA == false`, which
    /// hides the flight path vector and the AOA scale; the C# turns it on the first time either
    /// angle changes from zero and never turns it off (HUD.cs:889-930), which
    /// [`Timing::display_aoa_ssa`] keeps.
    pub aoa_ssa: Option<AoaSsa>,
    /// The user's extra fields, drawn bottom up.
    pub custom_items: Vec<CustomItem>,
    /// Vibration on x, y and z, m/s²: `vibex`, `vibey`, `vibez`.
    pub vibe: [f32; 3],
    /// The autopilot's main-loop load, percent: `load`, `SYS_STATUS.load` / 10. 0 until the
    /// vehicle reports one, as the C#'s `_load` starts (HUD.cs:3388).
    pub cpu_load: f32,
    /// The worst EKF variance, or 1 when the flags say the filter has no answer: `ekfstatus`.
    pub ekf_status: f32,
    /// Whether the vehicle reports its pre-arm checks passing: `prearmstatus`.
    pub prearm_ready: bool,
    /// `Russian`, which the HUD's menu toggles: the sky and ground held level while the pitch
    /// ladder, the aircraft symbol and the flight path vector turn with the roll instead, and the
    /// roll pointer reads the other way. `false` in the Designer.
    /// `// C#: ExtLibs/Controls/HUD.cs:163, 2029-2036, GCSViews/FlightData.Designer.cs:421`
    pub russian: bool,
    /// `CurrentState`'s multipliers, which its getters apply to `alt`, `airspeed`,
    /// `groundspeed`, `wp_dist` and the rest before the bindings hand them over, and the unit
    /// names `FlightData.Activate` sets on the HUD. Metres and metres per second until the
    /// caller says otherwise: `MainV2` runs `ChangeUnits` before the flight screen first shows.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:23-38, GCSViews/FlightData.cs:442-444, MainV2.cs:836`
    pub units: DisplayUnits,
    /// `displayCellVoltage`, which Battery Cell Voltage turns on: the HUD then shows the pack's
    /// voltage over `batterycellcount`. Off in the Designer and in `CheckBatteryShow`'s default.
    /// `// C#: ExtLibs/Controls/HUD.cs:245, GCSViews/FlightData.cs:597-602, 6115-6140`
    pub display_cell_voltage: bool,
    /// `batterycellcount`: the count Battery Cell Voltage's "Cell Count" prompt sets. The
    /// Designer's 4 is replaced at load by `CheckBatteryShow`, whose default is 0 - and at 0 no
    /// cell voltage is drawn, whatever `display_cell_voltage` says. The prompt takes any `int`.
    /// `// C#: GCSViews/FlightData.Designer.cs:338, GCSViews/FlightData.cs:517, 601, 6130-6139`
    pub battery_cell_count: i32,
    /// `displayicons`, which Show icons turns over: pictures in place of the battery, GPS, Vibe,
    /// EKF and pre-arm text. Off in the Designer and in `HUD_showicons`'s default.
    /// `// C#: GCSViews/FlightData.Designer.cs:402, GCSViews/FlightData.cs:427, 6484-6496`
    pub display_icons: bool,
    /// The second battery's voltage, current and remaining: `battery_voltage2`, `current2`,
    /// `battery_remaining2`. Drawn on the lower line while the voltage is above 0 and no cell
    /// voltage is shown (`batteryon2` is true from the constructor and nothing turns it off).
    /// `// C#: GCSViews/FlightData.Designer.cs:341, 359-361, ExtLibs/Controls/HUD.cs:2906-2912`
    pub battery_voltage2: f32,
    /// See [`HudInputs::battery_voltage2`].
    pub battery_current2: f32,
    /// See [`HudInputs::battery_voltage2`].
    pub battery_remaining2: i8,
    /// The second GPS's fix: `gpsstatus2`. 0 draws nothing for it.
    /// `// C#: GCSViews/FlightData.Designer.cs:367, ExtLibs/Controls/HUD.cs:2935-3027`
    pub gps_fix2: u8,
}

impl Default for HudInputs {
    fn default() -> Self {
        Self {
            has_vehicle: false,
            roll: 0.0,
            pitch: 0.0,
            heading: 0.0,
            target_heading: 0.0,
            ground_course: 0.0,
            xtrack_error: 0.0,
            turn_rate: 0.0,
            airspeed: 0.0,
            ground_speed: 0.0,
            target_speed: 0.0,
            altitude: 0.0,
            target_altitude: 0.0,
            ground_altitude: 0.0,
            vertical_speed: 0.0,
            mode: String::new(),
            mode_changed_for: None,
            wp_distance: 0.0,
            wp_number: 0,
            link_quality: 100,
            clock: String::new(),
            battery_voltage: 0.0,
            battery_current: 0.0,
            battery_remaining: 0,
            battery_low: false,
            battery_critical: false,
            gps_fix: 0,
            armed: false,
            armed_for: None,
            failsafe: false,
            safety: false,
            message: None,
            aoa_ssa: None,
            custom_items: Vec::new(),
            vibe: [0.0; 3],
            cpu_load: 0.0,
            ekf_status: 0.0,
            prearm_ready: false,
            russian: false,
            units: DisplayUnits::default().change_units(None, None, None),
            display_cell_voltage: false,
            battery_cell_count: 0,
            display_icons: false,
            battery_voltage2: 0.0,
            battery_current2: 0.0,
            battery_remaining2: 0,
            gps_fix2: 0,
        }
    }
}

impl HudInputs {
    /// The display's inputs from a vehicle's state, as `FlightData` binds them to `hud1`.
    ///
    /// `// C#: GCSViews/FlightData.Designer.cs:352-397` for the bindings, and
    /// `ExtLibs/ArduPilot/CurrentState.cs` for the derived ones: `turnrate` (1203), `targetalt`
    /// (1107), `targetairspeed` (1130), `failsafe` from `MAV_STATE_CRITICAL` (2895), the safety
    /// message when armed with motor control disabled (2887), `linkqualitygcs` (4595),
    /// `ekfstatus` (2649-2741), `prearmstatus` (179-182), `vibex`..`vibez` (2592-2606), `load`
    /// (2949), `AOA` and `SSA` (3903-3911), and `crit_AOA` from the parameters (1017-1036).
    ///
    /// `display_aoa_ssa` is the display's own `displayAOASSA`, which [`Timing::display_aoa_ssa`]
    /// keeps; `parameters` is the vehicle's parameter table, which `crit_AOA` reads.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // display precision
    #[allow(clippy::too_many_arguments)] // the bindings' sources: the state, the display's clocks
    pub fn from_vehicle(
        state: &VehicleState,
        mode: String,
        armed_for: Option<Duration>,
        mode_changed_for: Option<Duration>,
        clock: String,
        message: Option<(String, u32)>,
        display_aoa_ssa: bool,
        parameters: &[(String, f64)],
    ) -> Self {
        const MAV_STATE_CRITICAL: u8 = 5;
        Self {
            has_vehicle: true,
            roll: state.attitude.roll.0.to_degrees() as f32,
            pitch: state.attitude.pitch.0.to_degrees() as f32,
            heading: state.heading.degrees() as f32,
            target_heading: state.nav.bearing,
            ground_course: state.heading.degrees() as f32,
            xtrack_error: state.nav.xtrack_error,
            turn_rate: state.turn_rate(),
            airspeed: state.air_speed.0 as f32,
            ground_speed: state.ground_speed.0 as f32,
            target_speed: state.target_airspeed() as f32,
            altitude: state.altitude_relative.0 as f32,
            target_altitude: state.target_altitude() as f32,
            ground_altitude: 0.0,
            vertical_speed: state.climb_rate.0 as f32,
            mode,
            mode_changed_for,
            wp_distance: state.nav.wp_distance,
            wp_number: state.mission_current,
            link_quality: state.link.quality_percent(),
            clock,
            battery_voltage: state.battery.voltage,
            battery_current: state.battery.current,
            battery_remaining: state.battery.remaining_percent,
            battery_low: false,
            battery_critical: false,
            gps_fix: state.gps.fix_type,
            armed: state.armed,
            armed_for,
            failsafe: state.system_status == MAV_STATE_CRITICAL,
            safety: state.armed && state.sensors.reported && !state.sensors.motor_outputs_enabled(),
            message,
            // The angles as `AOA_SSA` sent them, in degrees, and the critical angle, drawn only
            // while `displayAOASSA` is on. C#: GCSViews/FlightData.Designer.cs:395-397,
            // ExtLibs/Controls/HUD.cs:2233, 2805
            aoa_ssa: display_aoa_ssa.then(|| AoaSsa {
                aoa: state.aoa,
                ssa: state.ssa,
                crit_aoa: crit_aoa(parameters),
            }),
            // No editor and no stored list yet. C#: GCSViews/FlightData.cs:336-348, 2436-2472
            custom_items: Vec::new(),
            vibe: [state.vibration.x, state.vibration.y, state.vibration.z],
            // C#: GCSViews/FlightData.Designer.cs:354, ExtLibs/ArduPilot/CurrentState.cs:2949
            cpu_load: state.load,
            ekf_status: ekf_status(state),
            prearm_ready: prearm_ready(state),
            // The HUD's own setting, not the vehicle's: the flight screen sets it.
            russian: false,
            // The user's, not the vehicle's: the flight screen hands over the Planner page's.
            units: DisplayUnits::default().change_units(None, None, None),
            // The HUD menu's settings, which the flight screen sets.
            display_cell_voltage: false,
            battery_cell_count: 0,
            display_icons: false,
            // C#: GCSViews/FlightData.Designer.cs:359-361, 367
            battery_voltage2: state.batteries[0].voltage,
            battery_current2: state.batteries[0].current,
            battery_remaining2: state.batteries[0].remaining_percent,
            gps_fix2: state.gps2.fix_type,
        }
    }
}

/// What the flight screen hands the display each frame: a vehicle's state through the display's
/// own clocks - how long ago it armed and changed mode ([`Timing::observe`]), the high-priority
/// message ([`Timing::message`]), `displayAOASSA` ([`Timing::display_aoa_ssa`]) - with the mode
/// by name and `cs.alt` moved by Set Home Alt. `clock` is the time the display shows.
///
/// `main.rs` calls this once a frame, and the golden frames (`hud/golden.rs`) call it on a
/// recorded flight, so a golden image is drawn by the path the screen takes.
/// `// C#: GCSViews/FlightData.Designer.cs:352-397, ExtLibs/Controls/HUD.cs:889-930,
/// ExtLibs/ArduPilot/CurrentState.cs:325-328`
#[must_use]
pub fn live_inputs(
    state: &VehicleState,
    timing: &mut Timing,
    now: Instant,
    clock: String,
    parameters: &[(String, f64)],
) -> HudInputs {
    let (armed_for, mode_changed_for) = timing.observe(state.armed, state.custom_mode, now);
    let mode = mp_vehicle::flight_mode_name(state.vehicle_type, state.custom_mode)
        .map_or_else(|| format!("mode {}", state.custom_mode), ToOwned::to_owned);
    let message = timing.message(high_priority_message(state), now);
    // C#: ExtLibs/Controls/HUD.cs:889-930
    let display_aoa_ssa = timing.display_aoa_ssa(state.aoa, state.ssa);
    // The HUD's altitude is `cs.alt`, which Set Home Alt moves to above sea level.
    // `// C#: ExtLibs/ArduPilot/CurrentState.cs:325-328`
    let mut shown = *state;
    shown.altitude_relative = mp_units::Metres(crate::fly::displayed_altitude(
        state.altitude_relative.0,
        state.alt_offset_home,
    ));
    HudInputs::from_vehicle(
        &shown,
        mode,
        armed_for,
        mode_changed_for,
        clock,
        message,
        display_aoa_ssa,
        parameters,
    )
}

/// `EKF_STATUS_FLAGS`: the three bits `ekfstatus` looks at.
const EKF_ATTITUDE: u16 = 1;
const EKF_VELOCITY_HORIZ: u16 = 2;
const EKF_UNINITIALIZED: u16 = 1024;

/// The number the HUD colours "EKF" by: `CurrentState.ekfstatus`.
///
/// The largest of the five variances - terrain included, which the vehicle crate's own verdict
/// leaves out - and 1 whatever they say when the filter has no attitude, has no horizontal
/// velocity while there is a GPS fix, or says it is uninitialised. Zero until the first
/// `EKF_STATUS_REPORT`, as the C#'s property starts. `Math.Max` is NaN if either side is, so a
/// NaN variance is carried through, and a NaN draws white as it does in the C#.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2649-2741`
#[must_use]
pub fn ekf_status(state: &VehicleState) -> f32 {
    let ekf = &state.ekf;
    if !ekf.seen {
        return 0.0;
    }
    let max = |a: f32, b: f32| {
        if a.is_nan() || b.is_nan() {
            f32::NAN
        } else {
            a.max(b)
        }
    };
    // C#: CurrentState.cs:2665-2667
    let worst = max(
        ekf.velocity_variance,
        max(
            ekf.compass_variance,
            max(
                ekf.position_horizontal_variance,
                max(
                    ekf.position_vertical_variance,
                    ekf.terrain_altitude_variance,
                ),
            ),
        ),
    );
    // The flag loop, reduced to the three cases in it that set anything. C#: :2694-2740
    let no_attitude = ekf.flags & EKF_ATTITUDE == 0;
    let no_velocity_with_gps = ekf.flags & EKF_VELOCITY_HORIZ == 0 && state.gps.fix_type > 0;
    let uninitialised = ekf.flags & EKF_UNINITIALIZED != 0;
    if no_attitude || no_velocity_with_gps || uninitialised {
        1.0
    } else {
        worst
    }
}

/// `MAV_SYS_STATUS_PREARM_CHECK`.
const PREARM_CHECK: u32 = 0x1000_0000;

/// Whether the pre-arm checks pass, as `CurrentState.prearmstatus` decides: the pre-arm bit
/// healthy, or not enabled at all - so a vehicle that has not yet sent `SYS_STATUS` reads as
/// ready, as it does in the C#. The C#'s `connected &&` is the caller's: this is asked only of
/// a vehicle on the link.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:179-182, 4935-4942`
#[must_use]
pub const fn prearm_ready(state: &VehicleState) -> bool {
    state.sensors.health & PREARM_CHECK != 0 || state.sensors.enabled & PREARM_CHECK == 0
}

/// What the C# writes for one extra field, or `None` for one it skips.
///
/// The property's name picks the format: seven optional decimals for a latitude or longitude,
/// none for the mAh used, `hh:mm:ss` for the time in the air, the header alone for `string`,
/// and two optional decimals for everything else.
/// `// C#: ExtLibs/Controls/HUD.cs:3036-3068`
#[must_use]
pub fn custom_item_text(item: &CustomItem) -> Option<String> {
    let value = item.value?;
    let header = &item.header;
    let name = item.name.as_str();
    Some(if name.contains("lat") || name.contains("lng") {
        format!("{header}{}", format_hash(value, 7))
    } else if name == "battery_usedmah" {
        format!("{header}{}", format_hash(value, 0))
    } else if name == "timeInAir" {
        // `(int)` truncates toward zero, as `as` does; `%` on a double keeps the dividend's sign
        // in both languages.
        #[allow(clippy::cast_possible_truncation)] // the C#'s own (int) casts
        let (hrs, mins, secs) = (
            (value / 3600.0) as i64,
            (value / 60.0) as i64 % 60,
            (value % 60.0) as i64,
        );
        format!(
            "{header}{}:{}:{}",
            two_digits(hrs),
            two_digits(mins),
            two_digits(secs)
        )
    } else if name == "string" {
        header.clone()
    } else {
        format!("{header}{}", format_hash(value, 2))
    })
}

/// An integer as .NET's `"00"` writes it: at least two digits, the sign outside them.
fn two_digits(value: i64) -> String {
    if value < 0 {
        format!("-{:02}", value.unsigned_abs())
    } else {
        format!("{value:02}")
    }
}

/// `value.ToString("0.##…")` with `decimals` optional places, as .NET Framework writes it.
///
/// The number is first taken to fifteen significant digits and then rounded half away from
/// zero, which is not what `format!("{:.2}")` does: .NET writes 0.125 as "0.13" and 1.005 as
/// "1.01" where Rust writes "0.12" and "1.00". Trailing zeros and a bare point are dropped, and
/// a value that rounds to zero loses its sign.
#[must_use]
pub fn format_hash(value: f64, decimals: u8) -> String {
    format_digits(value, decimals, 15)
}

/// A `float`'s `ToString("0.##…")`: as [`format_hash`], but taken first to seven significant
/// digits, which is what .NET Framework's `Number.FormatSingle` takes a `float` to for a custom
/// format - so 2.675f, stored as 2.67499995, writes "2.68" where the same double writes "2.67".
#[must_use]
pub fn format_single_hash(value: f32, decimals: u8) -> String {
    format_digits(f64::from(value), decimals, 7)
}

/// A `float`'s `ToString("0.00")`: [`format_single_hash`] with every place written, so 12.6
/// is "12.60" and 3 is "3.0" at one place. NaN and the infinities are written as they are.
#[must_use]
pub fn format_single_fixed(value: f32, decimals: u8) -> String {
    let mut text = format_single_hash(value, decimals);
    if decimals == 0 || !value.is_finite() {
        return text;
    }
    let written = text
        .split_once('.')
        .map_or(0, |(_, fraction)| fraction.len());
    if written == 0 {
        text.push('.');
    }
    text.push_str(&"0".repeat(usize::from(decimals).saturating_sub(written)));
    text
}

/// [`format_hash`]'s rounding on `significant` significant digits.
fn format_digits(value: f64, decimals: u8, significant: usize) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned();
    }
    // d.ddd…e<x>: `significant` significant digits.
    let scientific = format!("{:.*e}", significant.saturating_sub(1), value.abs());
    let Some((mantissa, exponent)) = scientific.split_once('e') else {
        return "0".to_owned();
    };
    let Ok(exponent) = exponent.parse::<i64>() else {
        return "0".to_owned();
    };
    let mut digits: Vec<u8> = mantissa
        .bytes()
        .filter(u8::is_ascii_digit)
        .map(|b| b - b'0')
        .collect();
    // The value is 0.d1d2d3... times ten to the `point`.
    let mut point = exponent + 1;
    let Ok(kept) = usize::try_from(point + i64::from(decimals)) else {
        return "0".to_owned();
    };
    if kept < digits.len() {
        let round_up = digits.get(kept).is_some_and(|digit| *digit >= 5);
        digits.truncate(kept);
        if round_up {
            let mut carry = true;
            for digit in digits.iter_mut().rev() {
                if *digit == 9 {
                    *digit = 0;
                } else {
                    *digit += 1;
                    carry = false;
                    break;
                }
            }
            if carry {
                digits.insert(0, 1);
                point += 1;
            }
        }
    }
    if digits.iter().all(|digit| *digit == 0) {
        return "0".to_owned();
    }
    let text: String = digits
        .iter()
        .map(|digit| char::from(b'0' + digit))
        .collect();
    let (whole, fraction) = match usize::try_from(point) {
        Ok(0) | Err(_) => {
            let zeros = usize::try_from(-point).unwrap_or(0);
            ("0".to_owned(), format!("{}{text}", "0".repeat(zeros)))
        }
        Ok(width) if width >= text.len() => (
            format!("{text}{}", "0".repeat(width - text.len())),
            String::new(),
        ),
        Ok(width) => {
            let (whole, fraction) = text.split_at(width);
            (whole.to_owned(), fraction.to_owned())
        }
    };
    let fraction = fraction.trim_end_matches('0');
    let sign = if value < 0.0 { "-" } else { "" };
    if fraction.is_empty() {
        format!("{sign}{whole}")
    } else {
        format!("{sign}{whole}.{fraction}")
    }
}

/// A colour by the name of the C#'s brush, for the facts.
#[must_use]
pub fn colour_name(colour: u32) -> String {
    match colour {
        colour::INK => "white".to_owned(),
        colour::WARN => "orange".to_owned(),
        colour::ALERT => "red".to_owned(),
        other => format!("#{other:06x}"),
    }
}

/// What the health readouts and the angle-of-attack elements drew, as facts a `.gui` test
/// asserts on: the colours of "Vibe" and "EKF" by name, whether "CPU" is up, the pre-arm line
/// and its colour, how many extra fields were drawn, and whether the flight path vector and the
/// AOA scale showed. Read from the scene, so what is asserted is what was painted.
#[must_use]
pub fn health_facts(scene: &Scene) -> Vec<(&'static str, String)> {
    let colour_of = |element: Element, text: &str| {
        scene
            .labels_of(element)
            .into_iter()
            .find(|(drawn, _)| *drawn == text)
            .map_or_else(|| "none".to_owned(), |(_, colour)| colour_name(colour))
    };
    let prearm = scene.labels_of(Element::Prearm).into_iter().next();
    vec![
        ("hud.vibe.colour", colour_of(Element::Vibe, "Vibe")),
        (
            "hud.cpu",
            scene
                .labels_of(Element::Vibe)
                .iter()
                .any(|(drawn, _)| *drawn == "CPU")
                .to_string(),
        ),
        ("hud.ekf.colour", colour_of(Element::Ekf, "EKF")),
        (
            "hud.prearm",
            prearm.map_or_else(|| "none".to_owned(), |(drawn, _)| drawn.to_owned()),
        ),
        (
            "hud.prearm.colour",
            prearm.map_or_else(|| "none".to_owned(), |(_, colour)| colour_name(colour)),
        ),
        (
            "hud.custom.count",
            scene.labels_of(Element::CustomItems).len().to_string(),
        ),
        (
            "hud.fpv.shown",
            scene.drawn.contains(&Element::FlightPathVector).to_string(),
        ),
        (
            "hud.aoa.shown",
            scene.drawn.contains(&Element::Aoa).to_string(),
        ),
    ]
}

/// What the tapes, the waypoint line, the battery and GPS lines and the pictures drew, as facts
/// a `.gui` test asserts on - read from the scene, so what is asserted is what was painted:
///
/// * `hud.speed`, `hud.airspeed`, `hud.groundspeed`: the speed tape's number and its AS and GS
///   lines, in the user's speed unit;
/// * `hud.alt`: the altitude tape's number, in the altitude unit;
/// * `hud.wpdist`: the distance and number of the waypoint, in the distance unit;
/// * `hud.battery.upper`, `hud.battery.lower`: the battery text on each bottom line;
/// * `hud.gps`: the GPS lines, top first, comma separated;
/// * `hud.icons`: the pictures drawn, by file name, in the order drawn;
/// * `hud.zone.vibe`, `hud.zone.ekf`: the click rectangles, `left,top,width,height` in whole
///   pixels - at the text, or at the pictures with the icons on.
///
/// "none" for anything not drawn.
#[must_use]
pub fn readout_facts(scene: &Scene) -> Vec<(&'static str, String)> {
    let none = || "none".to_owned();
    let speed = scene.placed_labels_of(Element::SpeedTape);
    let speed_line = |prefix: &str| {
        speed
            .iter()
            .find(|(text, _)| text.starts_with(prefix))
            .map_or_else(none, |(text, _)| (*text).to_owned())
    };
    let first = |element: Element| {
        scene
            .placed_labels_of(element)
            .first()
            .map_or_else(none, |(text, _)| (*text).to_owned())
    };
    // The battery's lines by height: the lower is the one furthest down.
    let mut battery = scene.placed_labels_of(Element::Battery);
    battery.sort_by(|a, b| a.1.1.total_cmp(&b.1.1));
    let (upper, lower) = match battery.as_slice() {
        [] => (none(), none()),
        [only] => (none(), only.0.to_owned()),
        [top, .., bottom] => (top.0.to_owned(), bottom.0.to_owned()),
    };
    let gps: Vec<&str> = {
        let mut lines = scene.placed_labels_of(Element::Gps);
        lines.sort_by(|a, b| a.1.1.total_cmp(&b.1.1));
        lines.into_iter().map(|(text, _)| text).collect()
    };
    let icons: Vec<&str> = scene.icons().into_iter().map(Icon::name).collect();
    let joined = |list: &[&str]| {
        if list.is_empty() {
            none()
        } else {
            list.join(",")
        }
    };
    let zone = |element: Element| {
        scene.zone(element).map_or_else(none, |(x, y, w, h)| {
            format!("{},{},{},{}", x.round(), y.round(), w.round(), h.round())
        })
    };
    vec![
        (
            "hud.speed",
            speed
                .iter()
                .find(|(text, _)| !text.starts_with("AS ") && !text.starts_with("GS "))
                .map_or_else(none, |(text, _)| (*text).to_owned()),
        ),
        ("hud.airspeed", speed_line("AS ")),
        ("hud.groundspeed", speed_line("GS ")),
        ("hud.alt", first(Element::AltitudeTape)),
        ("hud.wpdist", first(Element::ModeAndWaypoint)),
        ("hud.battery.upper", upper),
        ("hud.battery.lower", lower),
        ("hud.gps", joined(&gps)),
        ("hud.icons", joined(&icons)),
        ("hud.zone.vibe", zone(Element::Vibe)),
        ("hud.zone.ekf", zone(Element::Ekf)),
    ]
}

/// The message the C# raises to the HUD from the vehicle's state, if any.
///
/// `messageHigh` is not the last thing the vehicle said - `STATUSTEXT` never reaches it - but a
/// short list of conditions the C# checks itself: an EKF variance at or past 1, in the order the
/// C# checks them so the last one wins as it does there, and the safety switch holding an armed
/// vehicle. Fence breaches and peripheral over-current are the other two sources and are not
/// ingested yet. The colour is always red: the C# sets the severity to EMERGENCY.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2665-2691, 2887-2890, 1282`
#[must_use]
pub fn high_priority_message(state: &VehicleState) -> Option<String> {
    let mut message = None;
    if state.ekf.seen {
        for (name, variance) in state.ekf.variances() {
            if variance >= 1.0 {
                let word = match name {
                    "velocity" => "velocity variance",
                    "position (horizontal)" => "pos horiz variance",
                    "position (vertical)" => "pos vert variance",
                    "compass" => "compass variance",
                    _ => "terrain alt variance",
                };
                message = Some(format!("Error {word}"));
            }
        }
    }
    if state.armed && state.sensors.reported && !state.sensors.motor_outputs_enabled() {
        message = Some("(SAFETY)".to_owned());
    }
    message
}

impl Timing {
    /// Keeps a high-priority message on the display for [`MESSAGE_LIFETIME`] after it was last
    /// raised, as `CurrentState.messageHigh` does.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1269-1287`
    pub fn message(&mut self, fresh: Option<String>, now: Instant) -> Option<(String, u32)> {
        if let Some(text) = fresh {
            self.message = Some((text, now));
        }
        match &self.message {
            Some((text, at)) if now.saturating_duration_since(*at) < MESSAGE_LIFETIME => {
                Some((text.clone(), colour::ALERT))
            }
            _ => None,
        }
    }
}

/// Where the horizon sits on screen for a given attitude.
///
/// Pure and separate from painting so the sign conventions can be tested. An inverted roll is
/// invisible in review and indistinguishable from a correct display in a screenshot of a banked
/// aircraft - the only way to know is to assert it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HorizonGeometry {
    /// Left-hand end of the visible horizon.
    pub left: (f32, f32),
    /// Right-hand end.
    pub right: (f32, f32),
    /// Unit vector along the horizon, pointing right.
    pub along: (f32, f32),
    /// Unit vector perpendicular to it, pointing at the sky.
    pub up: (f32, f32),
}

/// Screen pixels per degree of pitch: `every5deg = -Height / 65`.
/// `// C#: ExtLibs/Controls/HUD.cs:2043`
#[must_use]
pub fn pixels_per_degree(height: f32) -> f32 {
    height / 65.0
}

/// Computes the horizon for a roll and pitch in radians, within a viewport of `w` x `h` pixels
/// centred at `centre`.
#[must_use]
pub fn horizon_geometry(
    roll: f32,
    pitch: f32,
    centre: (f32, f32),
    w: f32,
    h: f32,
) -> HorizonGeometry {
    // The world rotates opposite to the aircraft, so the screen rotation is -roll.
    let (sin, cos) = (-roll).sin_cos();
    let along = (cos, sin);
    // Perpendicular, pointing at the sky: rotate `along` by -90 degrees in screen space, where
    // -y is up.
    let up = (sin, -cos);
    let shift = pitch.to_degrees() * pixels_per_degree(h);
    let horizon = (
        up.0.mul_add(-shift, centre.0),
        up.1.mul_add(-shift, centre.1),
    );
    let reach = w.hypot(h);
    HorizonGeometry {
        left: offset(horizon, along, -reach),
        right: offset(horizon, along, reach),
        along,
        up,
    }
}

/// Where text sits relative to its anchor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Align {
    /// The anchor is the left edge.
    Left,
    /// The anchor is the middle.
    Centre,
}

/// One drawing operation, in canvas coordinates.
#[derive(Debug, Clone, PartialEq)]
pub enum Item {
    /// A filled polygon.
    Fill {
        /// Its corners.
        points: Vec<(f32, f32)>,
        /// Its colour.
        colour: u32,
        /// Its opacity.
        alpha: f32,
    },
    /// A polyline.
    Stroke {
        /// Its vertices.
        points: Vec<(f32, f32)>,
        /// Its width.
        width: f32,
        /// Its colour.
        colour: u32,
        /// Its opacity.
        alpha: f32,
    },
    /// A line of text.
    Label {
        /// The text.
        text: String,
        /// The anchor point: the top of the text and the edge `align` names.
        at: (f32, f32),
        /// Font size in pixels.
        size: f32,
        /// Its colour.
        colour: u32,
        /// Which edge the anchor is.
        align: Align,
    },
    /// One of `HUDT`'s pictures, stretched into a rectangle as `DrawImage` stretches it.
    Icon {
        /// Which.
        icon: Icon,
        /// Where: left, top, width, height.
        rect: (f32, f32, f32, f32),
    },
}

/// A frame of the display, ready to paint.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Scene {
    /// What to draw, in order.
    pub items: Vec<Item>,
    /// Which elements were drawn.
    pub drawn: Vec<Element>,
    /// Which element drew which label or picture, as an index into `items`, for the facts.
    pub owners: Vec<(Element, usize)>,
    /// The rectangles `doPaint()` keeps for a click - `vibehitzone`, `ekfhitzone` - where it
    /// sets them: at the text, or at the picture with the icons on. Left, top, width, height.
    /// `// C#: ExtLibs/Controls/HUD.cs:3150-3158, 3211-3219`
    pub zones: Vec<(Element, (f32, f32, f32, f32))>,
}

impl Scene {
    fn fill(&mut self, points: Vec<(f32, f32)>, colour: u32, alpha: f32) {
        self.items.push(Item::Fill {
            points,
            colour,
            alpha,
        });
    }

    fn stroke(&mut self, points: Vec<(f32, f32)>, width: f32, colour: u32, alpha: f32) {
        self.items.push(Item::Stroke {
            points,
            width,
            colour,
            alpha,
        });
    }

    fn line(&mut self, from: (f32, f32), to: (f32, f32), width: f32, colour: u32) {
        self.stroke(vec![from, to], width, colour, 1.0);
    }

    fn label(
        &mut self,
        text: impl Into<String>,
        at: (f32, f32),
        size: f32,
        colour: u32,
        align: Align,
    ) {
        self.items.push(Item::Label {
            text: text.into(),
            at,
            size,
            colour,
            align,
        });
    }

    /// A label remembered as its element's, so [`Scene::labels_of`] can find it.
    fn owned_label(
        &mut self,
        element: Element,
        text: impl Into<String>,
        at: (f32, f32),
        size: f32,
        colour: u32,
        align: Align,
    ) {
        self.owners.push((element, self.items.len()));
        self.label(text, at, size, colour, align);
    }

    /// A picture, remembered as its element's.
    fn icon(&mut self, element: Element, icon: Icon, rect: (f32, f32, f32, f32)) {
        self.owners.push((element, self.items.len()));
        self.items.push(Item::Icon { icon, rect });
    }

    fn drew(&mut self, element: Element) {
        if !self.drawn.contains(&element) {
            self.drawn.push(element);
        }
    }

    /// The click rectangle `doPaint()` set for an element this frame, if it set one.
    #[must_use]
    pub fn zone(&self, element: Element) -> Option<(f32, f32, f32, f32)> {
        self.zones
            .iter()
            .find(|(owner, _)| *owner == element)
            .map(|(_, rect)| *rect)
    }

    /// The labels an element drew through [`Scene::owned_label`], with where each is anchored.
    #[must_use]
    pub fn placed_labels_of(&self, element: Element) -> Vec<(&str, (f32, f32))> {
        self.owners
            .iter()
            .filter(|(owner, _)| *owner == element)
            .filter_map(|(_, index)| match self.items.get(*index) {
                Some(Item::Label { text, at, .. }) => Some((text.as_str(), *at)),
                _ => None,
            })
            .collect()
    }

    /// The pictures drawn, in order.
    #[must_use]
    pub fn icons(&self) -> Vec<Icon> {
        self.items
            .iter()
            .filter_map(|item| match item {
                Item::Icon { icon, .. } => Some(*icon),
                _ => None,
            })
            .collect()
    }

    /// The labels an element drew through [`Scene::owned_label`], with their colours.
    #[must_use]
    pub fn labels_of(&self, element: Element) -> Vec<(&str, u32)> {
        self.owners
            .iter()
            .filter(|(owner, _)| *owner == element)
            .filter_map(|(_, index)| match self.items.get(*index) {
                Some(Item::Label { text, colour, .. }) => Some((text.as_str(), *colour)),
                _ => None,
            })
            .collect()
    }

    /// The labels, for tests.
    #[cfg(test)]
    #[must_use]
    pub fn labels(&self) -> Vec<&str> {
        self.items
            .iter()
            .filter_map(|item| match item {
                Item::Label { text, .. } => Some(text.as_str()),
                _ => None,
            })
            .collect()
    }

    /// The names of the drawn elements, comma separated, for the facts.
    #[must_use]
    pub fn drawn_names(&self) -> String {
        self.drawn
            .iter()
            .map(|element| element.name())
            .collect::<Vec<_>>()
            .join(",")
    }
}

/// The frame for a set of inputs in a `w` x `h` box. Canvas coordinates: (0, 0) is top left.
#[must_use]
#[allow(clippy::too_many_lines)] // one function per doPaint(), in its order, is the point
pub fn scene(inputs: &HudInputs, w: f32, h: f32) -> Scene {
    let mut scene = Scene::default();
    // C#: HUD.cs:2040-2048
    let fontsize = (h / 30.0).max(9.0);
    let fontoffset = (fontsize - 10.0).max(0.0);
    let ppd = pixels_per_degree(h);
    let (halfwidth, halfheight) = (w / 2.0, h / 2.0);
    let centre = (halfwidth, halfheight);

    // `Russian`: the sky and ground are not turned, and the roll is negated for everything drawn
    // after them - the ladder, the roll pointer - while the aircraft symbol and the flight path
    // vector, which are otherwise fixed, are turned by it. C#: HUD.cs:2029-2036, 2107-2110,
    // 2176-2182, 2210-2211
    let roll = if inputs.russian {
        -inputs.roll
    } else {
        inputs.roll
    };
    // Sky and ground, then the horizon geometry everything attitude-relative hangs off.
    let geometry = horizon_geometry(
        if inputs.russian {
            0.0
        } else {
            roll.to_radians()
        },
        inputs.pitch.to_radians(),
        centre,
        w,
        h,
    );
    // The ladder is turned by the roll as it stands after the negation.
    let ladder = if inputs.russian {
        horizon_geometry(roll.to_radians(), inputs.pitch.to_radians(), centre, w, h)
    } else {
        geometry
    };
    // Where the aircraft symbol and the flight path vector are turned to: `RotateTransform(-_roll)`
    // with `_roll` negated, so by the roll itself, clockwise for a right wing down.
    let symbol_turn = if inputs.russian { -roll } else { 0.0 };
    let turned = |p: (f32, f32)| turn(p, centre, symbol_turn);
    let (along, up) = (geometry.along, geometry.up);
    let horizon = (
        (geometry.left.0 + geometry.right.0) / 2.0,
        (geometry.left.1 + geometry.right.1) / 2.0,
    );
    let reach = w.hypot(h);
    scene.fill(half_plane(horizon, along, up, reach), colour::SKY, 1.0);
    scene.fill(
        half_plane(horizon, along, (-up.0, -up.1), reach),
        colour::GROUND,
        1.0,
    );
    scene.drew(Element::SkyGround);

    // Pitch ladder: rungs every 5°, the long ones every 10° with a number, shown within
    // [pitch - 29, pitch + 20] and below the heading tape. The 0° rung is the horizon line, in
    // green. C#: HUD.cs:2103-2151
    let (long, short) = (w / 10.0, w / 14.0);
    let tape_bottom = h / 14.0;
    for degrees in (-90..=90).step_by(5) {
        #[allow(clippy::cast_precision_loss)]
        let a = degrees as f32;
        if a < inputs.pitch - 29.0 || a > inputs.pitch + 20.0 {
            continue;
        }
        let rung = offset(centre, ladder.up, (a - inputs.pitch) * ppd);
        if rung.1 < tape_bottom {
            continue;
        }
        let half = if degrees % 10 == 0 { long } else { short };
        let ink = if degrees == 0 {
            colour::TARGET
        } else {
            colour::INK
        };
        scene.line(
            offset(rung, ladder.along, -half),
            offset(rung, ladder.along, half),
            if degrees == 0 { 2.0 } else { 1.5 },
            ink,
        );
        if degrees % 10 == 0 {
            let at = offset(rung, ladder.along, -half - 30.0 - fontoffset * 1.7);
            scene.label(
                degrees.to_string(),
                (at.0, at.1 - 8.0 - fontoffset),
                fontsize + 2.0,
                colour::INK,
                Align::Left,
            );
        }
    }
    scene.drew(Element::PitchLadder);

    // Roll scale: fixed ticks around the top, with a pointer that rolls with the aircraft.
    // C#: HUD.cs:2153-2208
    let radius = (w.min(h) * 0.42).max(10.0);
    for tick in [
        -60.0_f32, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0,
    ] {
        let angle = tick.to_radians();
        let len = if (tick.abs() % 30.0) < 0.01 {
            12.0
        } else {
            7.0
        };
        scene.line(
            polar(centre, angle, radius),
            polar(centre, angle, radius - len),
            1.5,
            colour::INK,
        );
    }
    let pointer = (-roll).to_radians();
    scene.fill(
        vec![
            polar(centre, pointer, radius - 14.0),
            polar(centre, pointer - 0.06, radius - 28.0),
            polar(centre, pointer + 0.06, radius - 28.0),
        ],
        colour::ALERT,
        1.0,
    );
    scene.drew(Element::RollIndicator);

    // The aircraft symbol: the one thing on the display that does not move - but on the Russian
    // display, where it turns with the roll. C#: HUD.cs:2209-2231
    let wing = w * 0.13;
    let (cx, cy) = centre;
    scene.line(
        turned((cx - wing, cy)),
        turned((cx - wing * 0.35, cy)),
        3.0,
        colour::ALERT,
    );
    scene.line(
        turned((cx + wing * 0.35, cy)),
        turned((cx + wing, cy)),
        3.0,
        colour::ALERT,
    );
    scene.line(
        turned((cx, cy - 4.0)),
        turned((cx, cy + 4.0)),
        3.0,
        colour::ALERT,
    );
    scene.drew(Element::Reticle);

    if !inputs.has_vehicle {
        return scene;
    }

    // The flight path vector: a red circle with two wings and a fin, moved from the centre by
    // the sideslip to the right and the angle of attack down, at the ladder's scale. Shown once
    // the vehicle has sent an angle (`displayAOASSA`). The C# draws it inside the clip that
    // keeps the ladder off the heading tape (HUD.cs:2104-2105); the scene has no clip, so a
    // vector pushed up into the tape - an angle of attack near -28° - is drawn over the tape's
    // translucent fill rather than cut off. C#: HUD.cs:2232-2245
    if let Some(angles) = inputs.aoa_ssa {
        let fpv = (angles.ssa.mul_add(ppd, cx), angles.aoa.mul_add(ppd, cy));
        let (outer, inner) = (halfwidth / 20.0, halfwidth / 40.0);
        scene.stroke(
            circle(fpv, inner).into_iter().map(turned).collect(),
            2.0,
            colour::ALERT,
            1.0,
        );
        scene.line(
            turned((fpv.0 - outer, fpv.1)),
            turned((fpv.0 - inner, fpv.1)),
            2.0,
            colour::ALERT,
        );
        scene.line(
            turned((fpv.0 + outer, fpv.1)),
            turned((fpv.0 + inner, fpv.1)),
            2.0,
            colour::ALERT,
        );
        scene.line(
            turned((fpv.0, fpv.1 - outer)),
            turned((fpv.0, fpv.1 - inner)),
            2.0,
            colour::ALERT,
        );
        scene.drew(Element::FlightPathVector);
    }

    // Heading tape across the top: ±60° around the heading, a tick every 5°, a label every 15°,
    // the target heading in green and the ground course in black. C#: HUD.cs:2248-2396
    let tape = Rect {
        left: 0.0,
        top: 0.0,
        width: w,
        height: h / 14.0,
    };
    scene.fill(tape.corners(), colour::READOUT, 0.33);
    scene.line(
        (tape.left + 5.0, tape.bottom() - 5.0),
        (tape.width - 5.0, tape.bottom() - 5.0),
        1.5,
        colour::INK,
    );
    let space = (tape.width - 10.0) / 120.0;
    let start = (inputs.heading - 60.0).round();
    let end = inputs.heading + 60.0;
    let target = inputs.target_heading.rem_euclid(360.0).trunc();
    let course = inputs.ground_course.rem_euclid(360.0).trunc();
    // A target outside the tape sits at the edge nearest to it. The C# draws the right-hand
    // case at `space * 60`, which is the middle of the tape, not its edge - a slip of the pen
    // that puts the bug where the current heading is; the edge is what was meant.
    // C#: HUD.cs:2270-2285
    if target < start.rem_euclid(360.0) && inputs.target_heading < inputs.heading - 60.0 {
        let x = tape.left + 5.0;
        scene.line((x, tape.bottom()), (x, tape.top), 6.0, colour::TARGET);
        scene.drew(Element::HeadingBugs);
    }
    if inputs.target_heading > inputs.heading + 60.0 {
        let x = tape.left + 5.0 + space * 120.0;
        scene.line((x, tape.bottom()), (x, tape.top), 6.0, colour::TARGET);
        scene.drew(Element::HeadingBugs);
    }
    let mut a = start;
    while a <= end {
        let x = tape.left + 5.0 + space * (a - start);
        let shown = (a + 360.0).rem_euclid(360.0).trunc();
        if (shown - target).abs() < 0.5 {
            scene.line((x, tape.bottom()), (x, tape.top), 6.0, colour::TARGET);
            scene.drew(Element::HeadingBugs);
        }
        if (shown - course).abs() < 0.5 {
            scene.line((x, tape.bottom()), (x, tape.top), 6.0, colour::BLACK);
            scene.drew(Element::HeadingBugs);
        }
        #[allow(clippy::cast_possible_truncation)]
        let degrees = a.round() as i32;
        if degrees % 15 == 0 {
            scene.line(
                (x, tape.bottom() - 5.0),
                (x, tape.bottom() - 10.0),
                1.5,
                colour::INK,
            );
            let disp = degrees.rem_euclid(360);
            let (text, size) = match disp {
                0 => (" N".to_owned(), fontsize + 4.0),
                45 => ("NE".to_owned(), fontsize + 4.0),
                90 => (" E".to_owned(), fontsize + 4.0),
                135 => ("SE".to_owned(), fontsize + 4.0),
                180 => (" S".to_owned(), fontsize + 4.0),
                225 => ("SW".to_owned(), fontsize + 4.0),
                270 => (" W".to_owned(), fontsize + 4.0),
                315 => ("NW".to_owned(), fontsize + 4.0),
                other => (format!("{other:3}"), fontsize),
            };
            scene.label(
                text,
                (
                    tape.left - 5.0 + space * (a - start) - fontoffset,
                    tape.bottom() - 24.0 - fontoffset * 1.7,
                ),
                size,
                colour::INK,
                Align::Left,
            );
        } else if degrees % 5 == 0 {
            scene.line(
                (x, tape.bottom() - 5.0),
                (x, tape.bottom() - 10.0),
                1.5,
                colour::INK,
            );
        }
        a += 1.0;
    }
    // The centre box with the heading in it.
    let box_w = fontsize * 2.4;
    scene.fill(
        Rect {
            left: tape.width / 2.0 - box_w / 2.0,
            top: 0.0,
            width: box_w,
            height: tape.height,
        }
        .corners(),
        colour::READOUT,
        0.5,
    );
    scene.label(
        format!("{:3}", inputs.heading.rem_euclid(360.0).trunc()),
        (
            tape.width / 2.0 - fontsize,
            tape.bottom() - 24.0 - fontoffset * 1.7,
        ),
        fontsize,
        colour::INK,
        Align::Left,
    );
    scene.drew(Element::HeadingTape);

    // Cross-track error under the tape: five marks, the aircraft's position in green, clamped
    // at ±40 m and faded when clamped. C#: HUD.cs:2397-2438
    let xtspace = w / 10.0 / 3.0;
    let pad = 10.0;
    let xt_centre = w / 10.0;
    let (xt_top, xt_bottom) = (tape.bottom() + 5.0, tape.bottom() + h / 10.0);
    let xtrack = inputs.xtrack_error.clamp(-40.0, 40.0);
    let loc = xtrack / 20.0 * xtspace;
    let clamped = xtrack.abs() >= 40.0;
    scene.stroke(
        vec![(xt_centre + loc, xt_top), (xt_centre + loc, xt_bottom)],
        2.0,
        colour::TARGET,
        if clamped { 0.5 } else { 1.0 },
    );
    scene.line(
        (xt_centre, xt_top),
        (xt_centre, xt_bottom),
        2.0,
        colour::INK,
    );
    for step in [-2.0_f32, -1.0, 1.0, 2.0] {
        let x = xt_centre + xtspace * step;
        scene.line((x, xt_top + pad), (x, xt_bottom - pad), 2.0, colour::INK);
    }
    scene.drew(Element::XtrackBar);

    // Rate of turn below it: three marks and a green pointer, ±6°/s across the width.
    // C#: HUD.cs:2439-2483
    let rot_y = xt_bottom + 10.0;
    for step in [-2.0_f32, 0.0, 2.0] {
        let x0 = xt_centre + xtspace * step - xtspace / 2.0;
        scene.line((x0, rot_y), (x0 + xtspace, rot_y), 4.0, colour::INK);
    }
    let trwidth = xtspace * 4.0;
    let range = 12.0;
    let turn = inputs.turn_rate.clamp(-range / 2.0, range / 2.0);
    let loc = turn / range * trwidth;
    let alpha = if turn.abs() >= range / 2.0 { 0.5 } else { 1.0 };
    scene.stroke(
        vec![
            (xt_centre + loc - xtspace / 2.0, rot_y + 3.0),
            (xt_centre + loc + xtspace / 2.0, rot_y + 3.0),
        ],
        4.0,
        colour::TARGET,
        alpha,
    );
    scene.stroke(
        vec![
            (xt_centre + loc, rot_y + 3.0),
            (xt_centre + loc, rot_y + 10.0),
        ],
        4.0,
        colour::TARGET,
        alpha,
    );
    scene.drew(Element::RateOfTurn);

    // Speed scroller on the left: 26 units tall, a tick and a number every 5, the target speed
    // in green, an arrow with the value at the middle. Airspeed if there is one, else ground
    // speed. C#: HUD.cs:2484-2582
    let left_box = Rect {
        left: 0.0,
        top: halfheight - halfheight / 2.0,
        width: w / 10.0,
        height: h / 2.0,
    };
    let viewrange = 26.0;
    let unit_space = left_box.height / viewrange;
    // In the user's speed unit: `airspeed` and `groundspeed` are `_airspeed * multiplierspeed`
    // and `_groundspeed * multiplierspeed`, and `targetairspeed` is filtered from `airspeed` and
    // `aspd_error`, both multiplied. The tape's 26 units and its ticks every 5 are then that
    // many of the user's unit. C#: ExtLibs/ArduPilot/CurrentState.cs:496, 529, 1125-1130
    let units = &inputs.units;
    let airspeed = inputs.airspeed * units.speed;
    let ground_speed = inputs.ground_speed * units.speed;
    let speed = if airspeed == 0.0 {
        ground_speed
    } else {
        airspeed
    };
    scroller(
        &mut scene,
        &left_box,
        Side::Left,
        speed,
        inputs.target_speed * units.speed,
        None,
        unit_space,
        viewrange,
        fontsize,
        fontoffset,
        halfheight,
    );
    // `speed.ToString("0") + speedunit`, then `HUDT.AS` ("AS ") and `HUDT.GS` ("GS ") with each
    // to one place: .NET's rounding of a float, halves away from zero. C#: HUD.cs:2552-2577
    scene.owned_label(
        Element::SpeedTape,
        format!("{}{}", format_single_hash(speed, 0), units.speed_unit),
        (0.0, halfheight - 9.0),
        10.0,
        colour::INK,
        Align::Left,
    );
    scene.owned_label(
        Element::SpeedTape,
        format!(
            "AS {}{}",
            format_single_fixed(airspeed, 1),
            units.speed_unit
        ),
        (1.0, left_box.bottom() + 5.0),
        fontsize,
        colour::INK,
        Align::Left,
    );
    scene.owned_label(
        Element::SpeedTape,
        format!(
            "GS {}{}",
            format_single_fixed(ground_speed, 1),
            units.speed_unit
        ),
        (1.0, left_box.bottom() + fontsize + 2.0 + 10.0),
        fontsize,
        colour::INK,
        Align::Left,
    );
    scene.drew(Element::SpeedTape);

    // Altitude scroller on the right, with the ground filled in below ground level.
    // C#: HUD.cs:2583-2659
    let right_box = Rect {
        left: w - w / 10.0,
        top: halfheight - halfheight / 2.0,
        width: w / 10.0,
        height: h / 2.0,
    };
    // In the user's altitude unit: `alt` is `(_alt - altoffsethome) * multiplieralt`, and
    // `targetalt` is filtered from `alt` and `alt_error`, both multiplied. `groundalt` is bound
    // to `HomeAlt`, which no multiplier touches. C#: ExtLibs/ArduPilot/CurrentState.cs:327,
    // 1102-1107
    let altitude = inputs.altitude * units.alt;
    scroller(
        &mut scene,
        &right_box,
        Side::Right,
        altitude,
        inputs.target_altitude * units.alt,
        (inputs.ground_altitude != 0.0).then_some(inputs.ground_altitude),
        unit_space,
        viewrange,
        fontsize,
        fontoffset,
        halfheight,
    );
    // `((int) _alt).ToString("0 ") + altunit`: cut toward zero, not rounded. C#: HUD.cs:2733
    #[allow(clippy::cast_possible_truncation)] // the C#'s (int)
    let whole = altitude as i32;
    scene.owned_label(
        Element::AltitudeTape,
        format!("{whole} {}", units.alt_unit),
        (right_box.left + 10.0, halfheight - 9.0),
        10.0,
        colour::INK,
        Align::Left,
    );
    scene.drew(Element::AltitudeTape);

    // Vertical speed beside it: a tapered box, a blue bar from the middle for the climb rate,
    // clamped at ±6 of the user's speed unit. C#: HUD.cs:2660-2711
    //
    // The C# binds `verticalspeed`, its own derivative of `alt` - already in the altitude unit -
    // multiplied again by `multiplierspeed`, so with feet and knots the bar reads feet per second
    // times 1.94. This draws the climb rate in the speed unit alone, as `climbrate` is:
    // `_climbrate * multiplierspeed`. C#: ExtLibs/ArduPilot/CurrentState.cs:340, 1043-1050, 1154
    let vertical_speed = inputs.vertical_speed * units.speed;
    let quarter = right_box.width / 4.0;
    let vsi_left = right_box.left - quarter;
    scene.stroke(
        vec![
            (right_box.left, right_box.top),
            (vsi_left, right_box.top + quarter),
            (vsi_left, right_box.bottom() - quarter),
            (right_box.left, right_box.bottom()),
            (right_box.left, right_box.top),
        ],
        1.5,
        colour::INK,
        1.0,
    );
    let vs = vertical_speed.clamp(-6.0, 6.0);
    let mid = right_box.top + right_box.height / 2.0;
    let scaled = vs / -12.0 * right_box.height;
    let mut peak = 0.0;
    if scaled > 0.0 {
        peak = -quarter;
        if mid + scaled + peak < mid {
            peak = -scaled;
        }
    } else if scaled < 0.0 {
        peak = quarter;
        if mid + scaled + peak > mid {
            peak = -scaled;
        }
    }
    scene.fill(
        vec![
            (right_box.left, mid),
            (vsi_left, mid),
            (vsi_left, mid + scaled + peak),
            (right_box.left, mid + scaled),
        ],
        colour::VSI,
        1.0,
    );
    let linespace = right_box.height / 12.0;
    for step in 1..12 {
        #[allow(clippy::cast_precision_loss)]
        let y = right_box.top + linespace * step as f32;
        scene.line(
            (vsi_left, y),
            (right_box.left - quarter / 2.0, y),
            1.0,
            colour::INK,
        );
    }
    scene.drew(Element::Vsi);

    // Mode, and distance to the waypoint with its number, under the altitude scroller. The
    // mode is red for two seconds after it changes. C#: HUD.cs:2737-2772
    let mode_colour = if inputs
        .mode_changed_for
        .is_some_and(|since| since < MODE_CHANGE_FLASH)
    {
        colour::ALERT
    } else {
        colour::INK
    };
    scene.label(
        inputs.mode.clone(),
        (right_box.left - 30.0, right_box.bottom() + 5.0),
        fontsize,
        mode_colour,
        Align::Left,
    );
    scene.owned_label(
        Element::ModeAndWaypoint,
        format!(
            "{}>{}",
            wp_distance_text(inputs.wp_distance * units.dist, units.dist_unit),
            inputs.wp_number
        ),
        (
            right_box.left - 30.0,
            right_box.bottom() + fontsize + 2.0 + 10.0,
        ),
        fontsize,
        colour::INK,
        Align::Left,
    );
    scene.drew(Element::ModeAndWaypoint);

    // Link quality as three bars and a percentage, with the clock, above the altitude
    // scroller; a red cross when the link is dead. C#: HUD.cs:2773-2803
    let base = right_box.top - fontsize * 2.2 - 2.0;
    let bar_bottom = right_box.top - fontsize - 2.0 - 20.0;
    for (threshold, dx, dy) in [
        (80u8, 5.0_f32, 20.0_f32),
        (50, 10.0, 15.0),
        (20, 15.0, 10.0),
    ] {
        if inputs.link_quality > threshold {
            let x = right_box.left - dx;
            scene.line((x, base - dy), (x, bar_bottom), 2.0, colour::TARGET);
        }
    }
    scene.label(
        format!("{}%", inputs.link_quality),
        (right_box.left, base - 20.0),
        fontsize,
        colour::INK,
        Align::Left,
    );
    if inputs.link_quality == 0 {
        scene.line(
            (right_box.left, base - 20.0),
            (right_box.left + 50.0, base),
            2.0,
            colour::ALERT,
        );
        scene.line(
            (right_box.left, base),
            (right_box.left + 50.0, base - 20.0),
            2.0,
            colour::ALERT,
        );
    }
    scene.label(
        inputs.clock.clone(),
        (right_box.left - 30.0, right_box.top - fontsize - 2.0 - 20.0),
        fontsize,
        colour::INK,
        Align::Left,
    );
    scene.drew(Element::LinkInfo);

    // The AOA scale, right of centre and below it: bands of red, yellow, green and blue from
    // the top, split at the C#'s 90, 60 and 10 percent, and a black arrow at the angle of attack
    // as a fraction of the critical one - on the green band's bottom edge at zero, on the red
    // band's bottom edge at the critical angle, held to the bar's ends beyond them. Shown with
    // the flight path vector. C#: HUD.cs:2804-2843, the percentages at 356-358
    if let Some(angles) = inputs.aoa_ssa {
        const RED: f32 = 90.0;
        const YELLOW: f32 = 60.0;
        const GREEN: f32 = 10.0;
        let bar = Rect {
            left: w - w / 6.0,
            top: halfheight + halfheight / 10.0,
            width: w / 25.0,
            height: h / 5.0,
        };
        let band = |from: f32, height: f32| {
            Rect {
                top: bar.top + bar.height * from / 100.0,
                height: bar.height * height / 100.0,
                ..bar
            }
            .corners()
        };
        scene.fill(band(0.0, 100.0 - RED), colour::ALERT, 1.0);
        scene.fill(band(100.0 - RED, RED - YELLOW), colour::CAUTION, 1.0);
        scene.fill(band(100.0 - YELLOW, YELLOW - GREEN), colour::TARGET, 1.0);
        scene.fill(band(100.0 - GREEN, GREEN), colour::VSI, 1.0);
        scene.stroke(bar.outline(), 2.0, colour::INK, 1.0);
        // Two `if`s, as the C# has, rather than `clamp`: a NaN - an angle of zero over a
        // critical angle of zero - passes through both unchanged here as it does there.
        let mut indicator = bar.height * (100.0 - GREEN) / 100.0
            - (angles.aoa / angles.crit_aoa) * (bar.height * (RED - GREEN) / 100.0);
        if indicator < 0.0 {
            indicator = 0.0;
        }
        if indicator > bar.height {
            indicator = bar.height;
        }
        let tip = (bar.left + bar.width / 5.0, bar.top + indicator);
        let back = bar.left - bar.width / 2.0 + bar.width / 5.0;
        let arrow = vec![
            tip,
            (back, tip.1 + bar.width / 2.0),
            (back, tip.1 - bar.width / 2.0),
        ];
        scene.fill(arrow.clone(), colour::BLACK, 1.0);
        let mut outline = arrow;
        outline.push(tip);
        scene.stroke(outline, 2.0, colour::INK, 1.0);
        scene.drew(Element::Aoa);
    }

    // The text lines along the bottom. C#: HUD.cs:2846-2854
    let y_bot_offset = if fontsize >= 8.0 { fontsize / 3.0 } else { 2.0 };
    let y_text_offset = fontsize + y_bot_offset + 2.0;
    let x_pos = fontsize;
    let y_upper = h - 2.0 * y_text_offset - y_bot_offset - 4.0;
    let y_lower = h - y_text_offset - y_bot_offset - 4.0;
    // The pictures' strip: `(fontsize + 8) * 3` wide and `fontsize + 8` high, `fontsize + 13`
    // up from the bottom. C#: HUD.cs:3002-3008, 3152-3153, 3213-3214, 3268-3269
    let icons = inputs.display_icons;
    let wide = (fontsize + 8.0) * 3.0;
    let strip_top = h - (fontsize + 13.0);

    // Battery: voltage, current and remaining, coloured by the alert level. C#: HUD.cs:2855-2925
    let battery_colour = if inputs.battery_critical {
        colour::ALERT
    } else if inputs.battery_low {
        colour::WARN
    } else {
        colour::INK
    };
    // `(_batterylevel / _batterycellcount).ToString("0.00v")`, when Battery Cell Voltage is on
    // with a count that is not 0. C#: HUD.cs:2897-2898, 2904-2905
    #[allow(clippy::cast_precision_loss)] // the C#'s float divided by an int
    let cell = (inputs.display_cell_voltage && inputs.battery_cell_count != 0)
        .then(|| format_single_fixed(inputs.battery_voltage / inputs.battery_cell_count as f32, 2));
    // `ToString("0.00v")`, `ToString("0.0 A")` and the remaining as a float writes it.
    let pack = format!(
        "{}v {} A {}%",
        format_single_fixed(inputs.battery_voltage, 2),
        format_single_fixed(inputs.battery_current, 1),
        inputs.battery_remaining
    );
    if icons {
        // The battery's picture at the left edge, half as wide as it is high, by the alert
        // level and then by the charge left; the numbers beside it on the lower line, and the
        // cell voltage alone above them. C#: HUD.cs:2861-2899
        let icon = if inputs.battery_critical {
            Icon::BattRed
        } else if inputs.battery_low {
            Icon::BattYellow
        } else if inputs.battery_remaining > 75 {
            Icon::Batt4
        } else if inputs.battery_remaining > 50 {
            Icon::Batt3
        } else if inputs.battery_remaining > 25 {
            Icon::Batt2
        } else {
            Icon::Batt1
        };
        let bottomsize = ((fontsize + 2.0) * 3.0) + fontoffset - 2.0;
        scene.icon(
            Element::Battery,
            icon,
            (3.0, h - bottomsize, bottomsize / 2.0, bottomsize),
        );
        let beside = bottomsize / 2.0 + 6.0;
        scene.owned_label(
            Element::Battery,
            pack,
            (beside, y_lower),
            fontsize + 1.0,
            battery_colour,
            Align::Left,
        );
        if let Some(cell) = cell {
            scene.owned_label(
                Element::Battery,
                format!("{cell}v"),
                (beside, y_upper),
                fontsize,
                battery_colour,
                Align::Left,
            );
        }
    } else {
        // The lower line is the cell voltage's, or failing it the second battery's while its
        // voltage is above 0; either puts the first battery's a line up. C#: HUD.cs:2901-2923
        let mut pack_line = y_upper;
        if let Some(cell) = cell {
            scene.owned_label(
                Element::Battery,
                format!("Cell {cell}v"),
                (x_pos, y_lower),
                fontsize + 2.0,
                battery_colour,
                Align::Left,
            );
        } else if inputs.battery_voltage2 > 0.0 {
            scene.owned_label(
                Element::Battery,
                format!(
                    "Bat2 {}v {} A {}%",
                    format_single_fixed(inputs.battery_voltage2, 2),
                    format_single_fixed(inputs.battery_current2, 1),
                    inputs.battery_remaining2
                ),
                (x_pos, y_lower),
                fontsize,
                battery_colour,
                Align::Left,
            );
        } else {
            pack_line = y_lower;
        }
        scene.owned_label(
            Element::Battery,
            format!("Bat1 {pack}"),
            (x_pos, pack_line),
            fontsize,
            battery_colour,
            Align::Left,
        );
    }
    scene.drew(Element::Battery);

    // GPS fix, each receiver's: in red when there is none, and the second's - skipped while it
    // has no GPS at all - under the first's, which then goes up a line. `col` is not reset
    // between the two, so a second receiver after a red first is red whatever its fix; that is
    // the C#'s and is kept. With the icons on, each fix's picture at the right of the strip.
    // C#: HUD.cs:2926-3027
    let mut gps_colour = colour::INK;
    for (index, fix) in [inputs.gps_fix, inputs.gps_fix2].into_iter().enumerate() {
        if fix <= 1 {
            gps_colour = colour::ALERT;
        }
        let mut text = gps_fix_text(fix);
        if index == 1 {
            text = text.replace("GPS:", "GPS2:");
        }
        if index >= 1 && fix == 0 {
            continue;
        }
        let line = if index == 0 && inputs.gps_fix2 > 0 {
            y_upper
        } else {
            y_lower
        };
        if icons {
            let x = if index == 0 && inputs.gps_fix2 > 0 {
                w - wide * 2.0 - 5.0
            } else {
                w - wide - 3.0
            };
            scene.icon(
                Element::Gps,
                Icon::gps(fix),
                (x, strip_top, wide, fontsize + 8.0),
            );
        } else {
            scene.owned_label(
                Element::Gps,
                text,
                (w - 13.0 * fontsize, line),
                fontsize,
                gps_colour,
                Align::Left,
            );
        }
    }
    scene.drew(Element::Gps);

    // The user's extra fields, header then value, from above the battery line upward, an
    // eighth of the way across; a field whose value cannot be read leaves no gap.
    // C#: HUD.cs:3029-3077
    let mut custom_y = h - (fontsize + 2.0) * 3.0 - fontoffset - fontsize - 8.0;
    for item in &inputs.custom_items {
        // `GetValue` reads the property at paint time, through its getter - so `alt`, `wp_dist`
        // and the rest arrive multiplied, as the quick view shows them. C#: HUD.cs:949-968
        let shown = CustomItem {
            value: item
                .value
                .map(|value| crate::quick::to_display(&item.name, value, units)),
            ..item.clone()
        };
        let Some(text) = custom_item_text(&shown) else {
            continue;
        };
        scene.owned_label(
            Element::CustomItems,
            text,
            (w / 8.0, custom_y),
            fontsize + 2.0,
            colour::INK,
            Align::Left,
        );
        custom_y -= fontsize + 5.0;
    }
    scene.drew(Element::CustomItems);

    // ARMED for eight seconds after arming, DISARMED whenever disarmed, SAFE while the safety
    // switch holds the motors: red, above the centre. C#: HUD.cs:3084-3119
    let banner_y = halfheight - halfheight / 3.0;
    if inputs.armed {
        if inputs.armed_for.is_some_and(|since| since < ARMED_BANNER) {
            scene.label(
                "ARMED",
                (halfwidth, banner_y),
                fontsize + 20.0,
                colour::ALERT,
                Align::Centre,
            );
        }
    } else {
        scene.label(
            "DISARMED",
            (halfwidth, banner_y),
            fontsize + 10.0,
            colour::ALERT,
            Align::Centre,
        );
    }
    if inputs.safety {
        scene.label(
            "SAFE",
            (halfwidth, halfheight - halfheight / 6.0),
            fontsize + 10.0,
            colour::ALERT,
            Align::Centre,
        );
    }
    scene.drew(Element::ArmedBanner);

    // FAILSAFE, when the vehicle reports MAV_STATE_CRITICAL. C#: HUD.cs:3120-3126, FailsafeH = 5
    if inputs.failsafe {
        scene.label(
            "FAILSAFE",
            (halfwidth - 85.0, halfheight - halfheight / 5.0),
            fontsize + 20.0,
            colour::ALERT,
            Align::Left,
        );
    }
    scene.drew(Element::Failsafe);

    // The high-priority message, below the centre, in its severity's colour.
    // C#: HUD.cs:3128-3145
    if let Some((text, message_colour)) = &inputs.message {
        scene.label(
            text.clone(),
            (halfwidth, halfheight + halfheight / 3.0),
            fontsize + 10.0,
            *message_colour,
            Align::Centre,
        );
    }
    scene.drew(Element::Message);

    // "Vibe" on the lower text line: white, orange once any axis is past 30 m/s², red past 60;
    // with the icons on, the green, amber or red VIBE picture in the strip, four pictures in
    // from the right. `vibehitzone` is set to where either goes: a 40-pixel box at the text, or
    // the picture's rectangle, which is drawn two pixels below it. Clipping does not enter into
    // it. C#: HUD.cs:3148-3204
    let vibe_zone = if icons {
        (
            w - wide * 4.0 + wide / 2.0 - 5.0,
            strip_top,
            wide,
            fontsize + 8.0,
        )
    } else {
        (w - 18.0 * fontsize, y_lower, 40.0, fontsize * 2.0)
    };
    scene.zones.push((Element::Vibe, vibe_zone));
    let [vibe_x_axis, vibe_y_axis, vibe_z_axis] = inputs.vibe;
    let over = |limit: f32| vibe_x_axis > limit || vibe_y_axis > limit || vibe_z_axis > limit;
    let (vibe_colour, vibe_icon) = if over(60.0) {
        (colour::ALERT, Icon::VibeRed)
    } else if over(30.0) {
        (colour::WARN, Icon::VibeYellow)
    } else {
        (colour::INK, Icon::VibeGreen)
    };
    if icons {
        scene.icon(Element::Vibe, vibe_icon, picture_at(vibe_zone));
    } else {
        scene.owned_label(
            Element::Vibe,
            "Vibe",
            (vibe_zone.0, vibe_zone.1),
            fontsize + 2.0,
            vibe_colour,
            Align::Left,
        );
    }
    // "CPU" in red at the right edge of Vibe's box when the autopilot reports a full load -
    // exactly 100, as the C# compares it. C#: HUD.cs:3206-3207
    if inputs.cpu_load == 100.0 {
        scene.owned_label(
            Element::Vibe,
            "CPU",
            (vibe_zone.0 + vibe_zone.2, vibe_zone.1),
            fontsize + 2.0,
            colour::ALERT,
            Align::Left,
        );
    }
    scene.drew(Element::Vibe);

    // "EKF" left of it: white, orange past 0.5, red past 0.8; with the icons on, its picture
    // five in from the right. `ekfhitzone` as Vibe's. C#: HUD.cs:3209-3262
    let ekf_zone = if icons {
        (
            w - wide * 5.0 + wide / 2.0 - 10.0,
            strip_top,
            wide,
            fontsize + 8.0,
        )
    } else {
        (w - 23.0 * fontsize, y_lower, 40.0, fontsize * 2.0)
    };
    scene.zones.push((Element::Ekf, ekf_zone));
    let (ekf_colour, ekf_icon) = if inputs.ekf_status > 0.8 {
        (colour::ALERT, Icon::EkfRed)
    } else if inputs.ekf_status > 0.5 {
        (colour::WARN, Icon::EkfYellow)
    } else {
        (colour::INK, Icon::EkfGreen)
    };
    if icons {
        scene.icon(Element::Ekf, ekf_icon, picture_at(ekf_zone));
    } else {
        scene.owned_label(
            Element::Ekf,
            "EKF",
            (ekf_zone.0, ekf_zone.1),
            fontsize + 2.0,
            ekf_colour,
            Align::Left,
        );
    }
    scene.drew(Element::Ekf);

    // While disarmed, the pre-arm state on the upper text line: "Ready to Arm" in white, or
    // "Not Ready to Arm" in red starting two characters further left; with the icons on, the
    // green or red picture, two wide, above EKF's. C#: HUD.cs:3264-3301, HUDT.resx
    // NotReadyToArm and ReadyToArm
    if !inputs.armed {
        if icons {
            let zone = (
                w - wide * 5.0 + wide / 2.0 - 7.0,
                h - (fontsize * 2.0 + 25.0),
                wide * 2.0,
                fontsize + 8.0,
            );
            let icon = if inputs.prearm_ready {
                Icon::PrearmGreen
            } else {
                Icon::PrearmRed
            };
            scene.icon(Element::Prearm, icon, picture_at(zone));
        } else {
            let (text, x, prearm_colour) = if inputs.prearm_ready {
                ("Ready to Arm", w - 24.0 * fontsize, colour::INK)
            } else {
                ("Not Ready to Arm", w - 26.0 * fontsize, colour::ALERT)
            };
            scene.owned_label(
                Element::Prearm,
                text,
                (x, y_upper - 4.0),
                fontsize + 2.0,
                prearm_colour,
                Align::Left,
            );
        }
    }
    scene.drew(Element::Prearm);

    scene
}

/// Which side of the display a scroller is on: where its ticks, numbers and arrow point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Side {
    Left,
    Right,
}

/// A scroller: the vertical tape the speed and altitude are read from.
///
/// 26 units visible, a tick and a number every five, the target as a green line across it (or
/// a faded line at the edge it is beyond), and for the altitude tape the ground filled in below
/// ground level. C#: HUD.cs:2484-2559 and 2583-2659, which are the same code twice.
#[allow(clippy::too_many_arguments)] // the C#'s own variables, named the same
fn scroller(
    scene: &mut Scene,
    bx: &Rect,
    side: Side,
    value: f32,
    target: f32,
    ground: Option<f32>,
    space: f32,
    viewrange: f32,
    fontsize: f32,
    fontoffset: f32,
    halfheight: f32,
) {
    scene.fill(bx.corners(), colour::READOUT, 0.33);
    scene.stroke(bx.outline(), 1.5, colour::INK, 1.0);
    let start = (value - viewrange / 2.0).floor();
    // The tape's y for a value: the box's top is `start + viewrange`, its bottom `start`, and
    // the C# draws it translated to the middle of the display.
    let y_for = |a: f32| halfheight + bx.top - space * (a - start);
    if start > target {
        scene.stroke(
            vec![
                (bx.left, halfheight + bx.top),
                (bx.right(), halfheight + bx.top),
            ],
            6.0,
            colour::TARGET,
            0.5,
        );
    }
    if value + viewrange / 2.0 < target {
        let y = halfheight + bx.top - space * viewrange;
        scene.stroke(
            vec![(bx.left, y), (bx.right(), y)],
            6.0,
            colour::TARGET,
            0.5,
        );
    }
    let mut ground_drawn = false;
    let mut a = start;
    while a <= value + viewrange / 2.0 {
        if (a - target.round()).abs() < 0.5 && target != 0.0 {
            scene.line(
                (bx.left, y_for(a)),
                (bx.right(), y_for(a)),
                6.0,
                colour::TARGET,
            );
        }
        if let Some(ground_level) = ground
            && (a - ground_level.round()).abs() < 0.5
            && !ground_drawn
        {
            // From ground level down to the bottom of the tape.
            scene.fill(
                vec![
                    (bx.left, y_for(a)),
                    (bx.right(), y_for(a)),
                    (bx.right(), y_for(start)),
                    (bx.left, y_for(start)),
                ],
                colour::GROUND_TAPE,
                0.4,
            );
            ground_drawn = true;
        }
        if a.rem_euclid(5.0) < 0.5 {
            let (tick_from, tick_to, label_x) = match side {
                Side::Left => (bx.right(), bx.right() - 10.0, 0.0),
                Side::Right => (bx.left, bx.left + 10.0, bx.left),
            };
            scene.line((tick_from, y_for(a)), (tick_to, y_for(a)), 1.5, colour::INK);
            scene.label(
                format!("{:5}", a.trunc()),
                (label_x, y_for(a) - 6.0 - fontoffset),
                fontsize,
                colour::INK,
                Align::Left,
            );
        }
        a += 1.0;
    }
    // The arrow at the middle, pointing into the display, with the value drawn over it by the
    // caller. C#: HUD.cs:2491-2496 (left), 2590-2595 and 2719-2722 (right, rotated 180°)
    let arrow = match side {
        Side::Left => vec![
            (0.0, halfheight - 10.0),
            (bx.width - 10.0, halfheight - 10.0),
            (bx.width - 5.0, halfheight),
            (bx.width - 10.0, halfheight + 10.0),
            (0.0, halfheight + 10.0),
        ],
        Side::Right => vec![
            (bx.right(), halfheight + 10.0),
            (bx.left + 10.0, halfheight + 10.0),
            (bx.left + 5.0, halfheight),
            (bx.left + 10.0, halfheight - 10.0),
            (bx.right(), halfheight - 10.0),
        ],
    };
    scene.fill(arrow, colour::BLACK, 1.0);
}

/// An axis-aligned rectangle, as the C# uses `Rectangle`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Rect {
    left: f32,
    top: f32,
    width: f32,
    height: f32,
}

impl Rect {
    const fn right(&self) -> f32 {
        self.left + self.width
    }

    const fn bottom(&self) -> f32 {
        self.top + self.height
    }

    fn corners(&self) -> Vec<(f32, f32)> {
        vec![
            (self.left, self.top),
            (self.right(), self.top),
            (self.right(), self.bottom()),
            (self.left, self.bottom()),
        ]
    }

    fn outline(&self) -> Vec<(f32, f32)> {
        let mut points = self.corners();
        points.push((self.left, self.top));
        points
    }
}

/// Where `DrawImage` puts a picture for a click rectangle: the same box, two pixels lower.
/// `// C#: ExtLibs/Controls/HUD.cs:3173, 3184, 3197, 3232, 3243, 3255, 3284, 3295`
const fn picture_at(zone: (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
    (zone.0, zone.1 + 2.0, zone.2, zone.3)
}

/// The stand-in [`paint`] draws for a picture in its rectangle.
///
/// A badge is its colour at the bitmaps' alpha of 179, with its words in white across the
/// middle, as large as the box's height allows and small enough to fit its width. A battery is
/// its outline and cap, and its bars from the bottom, at the places the bitmaps have them:
/// measured from `batt_4.png`, 180 by 360 pixels, as fractions of the box.
/// `// C#: ExtLibs/Controls/Resources/*.png`
#[must_use]
pub fn icon_items(icon: Icon, rect: (f32, f32, f32, f32)) -> Vec<Item> {
    let (left, top, width, height) = rect;
    let at = |fx: f32, fy: f32| (fx.mul_add(width, left), fy.mul_add(height, top));
    let block =
        |x0: f32, y0: f32, x1: f32, y1: f32| vec![at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)];
    match icon.look() {
        Look::Badge { colour, text } => {
            // Seven tenths of the height, or what fits the width at about six tenths of the
            // size a character.
            #[allow(clippy::cast_precision_loss)] // a few characters
            let characters = text.chars().count().max(1) as f32;
            let size = (height * 0.7).min(width * 0.9 / (characters * 0.6));
            vec![
                Item::Fill {
                    points: block(0.0, 0.0, 1.0, 1.0),
                    colour,
                    alpha: 179.0 / 255.0,
                },
                Item::Label {
                    text: text.to_owned(),
                    at: (left + width / 2.0, top + (height - size * 1.2) / 2.0),
                    size,
                    colour: colour::INK,
                    align: Align::Centre,
                },
            ]
        }
        Look::Battery { outline, bars, bar } => {
            // The bars from the bottom: y from 264-313, 198-246, 131-180 and 65-114 of 360,
            // x from 33 to 144 of 180.
            const BARS: [(f32, f32); 4] = [
                (264.0 / 360.0, 314.0 / 360.0),
                (198.0 / 360.0, 247.0 / 360.0),
                (131.0 / 360.0, 181.0 / 360.0),
                (65.0 / 360.0, 115.0 / 360.0),
            ];
            let mut items = vec![
                // The cap: x 57-120, y 16-33.
                Item::Fill {
                    points: block(57.0 / 180.0, 16.0 / 360.0, 121.0 / 180.0, 33.0 / 360.0),
                    colour: outline,
                    alpha: 1.0,
                },
                // The case, 13 pixels of 180 thick, its middle from x 10 to 167 and y 39 to 339.
                Item::Stroke {
                    points: {
                        let mut points =
                            block(10.0 / 180.0, 39.0 / 360.0, 167.0 / 180.0, 339.0 / 360.0);
                        points.push(at(10.0 / 180.0, 39.0 / 360.0));
                        points
                    },
                    width: width * 13.0 / 180.0,
                    colour: outline,
                    alpha: 1.0,
                },
            ];
            for (from, to) in BARS.iter().take(usize::from(bars)) {
                items.push(Item::Fill {
                    points: block(33.0 / 180.0, *from, 145.0 / 180.0, *to),
                    colour: bar,
                    alpha: 1.0,
                });
            }
            items
        }
    }
}

/// A half-plane bounded by a line through `origin` along `along`, extending `reach` toward
/// `toward`.
fn half_plane(
    origin: (f32, f32),
    along: (f32, f32),
    toward: (f32, f32),
    reach: f32,
) -> Vec<(f32, f32)> {
    let a = offset(origin, along, -reach);
    let b = offset(origin, along, reach);
    let c = offset(b, toward, reach);
    let d = offset(a, toward, reach);
    vec![a, b, c, d]
}

fn offset(from: (f32, f32), direction: (f32, f32), distance: f32) -> (f32, f32) {
    (
        direction.0.mul_add(distance, from.0),
        direction.1.mul_add(distance, from.1),
    )
}

/// A closed polyline round `centre`, standing in for GDI+'s `DrawEllipse` of a circle.
fn circle(centre: (f32, f32), radius: f32) -> Vec<(f32, f32)> {
    const SEGMENTS: u16 = 32;
    (0..=SEGMENTS)
        .map(|step| {
            let angle = f32::from(step) / f32::from(SEGMENTS) * std::f32::consts::TAU;
            polar(centre, angle, radius)
        })
        .collect()
}

/// `p` turned `degrees` clockwise about `centre`, as `RotateTransform` turns what is drawn after
/// it on a screen whose y grows downward. Zero leaves it where it is.
fn turn(p: (f32, f32), centre: (f32, f32), degrees: f32) -> (f32, f32) {
    if degrees == 0.0 {
        return p;
    }
    let (sin, cos) = degrees.to_radians().sin_cos();
    let (x, y) = (p.0 - centre.0, p.1 - centre.1);
    (
        x.mul_add(cos, -y * sin) + centre.0,
        x.mul_add(sin, y * cos) + centre.1,
    )
}

/// A point at `angle` radians clockwise from straight up, `radius` from `centre`.
fn polar(centre: (f32, f32), angle: f32, radius: f32) -> (f32, f32) {
    (
        angle.sin().mul_add(radius, centre.0),
        angle.cos().mul_add(-radius, centre.1),
    )
}

fn to_screen(origin: Point<Pixels>, p: (f32, f32)) -> Point<Pixels> {
    point(origin.x + px(p.0), origin.y + px(p.1))
}

fn tinted(colour: u32, alpha: f32) -> Hsla {
    let mut hsla = Hsla::from(rgb(colour));
    hsla.a = alpha;
    hsla
}

/// Paints a scene into a canvas whose top-left is `bounds.origin`.
pub fn paint(scene: &Scene, bounds: Bounds<Pixels>, window: &mut Window, cx: &mut gpui::App) {
    let origin = bounds.origin;
    for item in &scene.items {
        paint_item(item, origin, window, cx);
    }
}

/// Paints one item: a picture as its stand-in, [`icon_items`].
fn paint_item(item: &Item, origin: Point<Pixels>, window: &mut Window, cx: &mut gpui::App) {
    match item {
        Item::Fill {
            points,
            colour,
            alpha,
        } => fill(window, origin, points, tinted(*colour, *alpha)),
        Item::Stroke {
            points,
            width,
            colour,
            alpha,
        } => stroke(window, origin, points, *width, tinted(*colour, *alpha)),
        Item::Label {
            text,
            at,
            size,
            colour,
            align,
        } => label(window, cx, origin, text, *at, *size, *colour, *align),
        Item::Icon { icon, rect } => {
            for part in icon_items(*icon, *rect) {
                paint_item(&part, origin, window, cx);
            }
        }
    }
}

/// Strokes a polyline.
fn stroke(
    window: &mut Window,
    origin: Point<Pixels>,
    points: &[(f32, f32)],
    width: f32,
    colour: Hsla,
) {
    if points.len() < 2 {
        return;
    }
    let mut builder = PathBuilder::stroke(px(width));
    let mut iter = points.iter();
    if let Some(first) = iter.next() {
        builder.move_to(to_screen(origin, *first));
    }
    for p in iter {
        builder.line_to(to_screen(origin, *p));
    }
    // A failed tessellation must not silently erase part of the display: ADR 0001 records how a
    // swallowed `Err` made a whole flight track vanish.
    match builder.build() {
        Ok(path) => window.paint_path(path, colour),
        Err(err) => {
            debug_assert!(false, "HUD stroke failed to tessellate: {err:?}");
        }
    }
}

/// Fills a closed polygon.
fn fill(window: &mut Window, origin: Point<Pixels>, points: &[(f32, f32)], colour: Hsla) {
    if points.len() < 3 {
        return;
    }
    let mut builder = PathBuilder::fill();
    let mut iter = points.iter();
    if let Some(first) = iter.next() {
        builder.move_to(to_screen(origin, *first));
    }
    for p in iter {
        builder.line_to(to_screen(origin, *p));
    }
    if let Some(first) = points.first() {
        builder.line_to(to_screen(origin, *first));
    }
    match builder.build() {
        Ok(path) => window.paint_path(path, colour),
        Err(err) => {
            debug_assert!(false, "HUD fill failed to tessellate: {err:?}");
        }
    }
}

/// Draws one line of text through gpui's text system, aligned about its anchor.
#[allow(clippy::too_many_arguments)]
fn label(
    window: &mut Window,
    cx: &mut gpui::App,
    origin: Point<Pixels>,
    text: &str,
    at: (f32, f32),
    size: f32,
    colour: u32,
    align: Align,
) {
    if text.is_empty() {
        return;
    }
    let run = TextRun {
        len: text.len(),
        font: window.text_style().font(),
        color: Hsla::from(rgb(colour)),
        background_color: None,
        underline: None,
        strikethrough: None,
    };
    let line = window.text_system().shape_line(
        SharedString::from(text.to_owned()),
        px(size),
        &[run],
        None,
    );
    let width = f32::from(line.width());
    let x = match align {
        Align::Left => at.0,
        Align::Centre => at.0 - width / 2.0,
    };
    if let Err(err) = line.paint(
        to_screen(origin, (x, at.1)),
        px(size * 1.2),
        TextAlign::Left,
        None,
        window,
        cx,
    ) {
        debug_assert!(false, "HUD label failed to paint: {err:?}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink_dialects::all::{AoaSsa as AoaSsaMessage, MavMessage};

    const CENTRE: (f32, f32) = (200.0, 150.0);
    const W: f32 = 400.0;
    const H: f32 = 300.0;

    #[test]
    fn level_flight_puts_a_flat_horizon_through_the_centre() {
        let g = horizon_geometry(0.0, 0.0, CENTRE, W, H);
        assert!(
            (g.left.1 - CENTRE.1).abs() < 1e-3,
            "left end at {}",
            g.left.1
        );
        assert!(
            (g.right.1 - CENTRE.1).abs() < 1e-3,
            "right end at {}",
            g.right.1
        );
        assert!(g.left.0 < g.right.0, "left must be left of right");
        // Sky is up, which is negative y.
        assert!(g.up.1 < 0.0, "up vector was {:?}", g.up);
    }

    #[test]
    fn rolling_right_raises_the_right_hand_horizon() {
        // The convention this whole display depends on. A right bank shows the horizon with its
        // right end high, because the world appears to rotate opposite to the aircraft.
        let g = horizon_geometry(20.0_f32.to_radians(), 0.0, CENTRE, W, H);
        assert!(
            g.right.1 < g.left.1,
            "rolling right must raise the right end: left y {}, right y {}",
            g.left.1,
            g.right.1
        );
    }

    #[test]
    fn rolling_left_raises_the_left_hand_horizon() {
        let g = horizon_geometry(-20.0_f32.to_radians(), 0.0, CENTRE, W, H);
        assert!(
            g.left.1 < g.right.1,
            "rolling left must raise the left end: left y {}, right y {}",
            g.left.1,
            g.right.1
        );
    }

    #[test]
    fn pitching_up_moves_the_horizon_down_the_screen() {
        let level = horizon_geometry(0.0, 0.0, CENTRE, W, H);
        let nose_up = horizon_geometry(0.0, 10.0_f32.to_radians(), CENTRE, W, H);
        assert!(
            nose_up.left.1 > level.left.1,
            "nose up should push the horizon down: {} then {}",
            level.left.1,
            nose_up.left.1
        );

        // And the displacement is the C#'s scale: Height / 65 pixels per degree.
        let expected = 10.0 * pixels_per_degree(H);
        assert!(
            (nose_up.left.1 - level.left.1 - expected).abs() < 0.01,
            "expected {expected} px of movement, got {}",
            nose_up.left.1 - level.left.1
        );
    }

    #[test]
    fn the_up_vector_stays_perpendicular_at_every_roll_angle() {
        for degrees in [-180, -90, -45, -1, 0, 1, 45, 90, 180] {
            let g = horizon_geometry((degrees as f32).to_radians(), 0.0, CENTRE, W, H);
            let dot = g.along.0.mul_add(g.up.0, g.along.1 * g.up.1);
            assert!(
                dot.abs() < 1e-5,
                "at {degrees} degrees the dot product was {dot}"
            );
            let len = g.up.0.hypot(g.up.1);
            assert!((len - 1.0).abs() < 1e-5, "up vector length was {len}");
        }
    }

    #[test]
    fn inverted_flight_puts_the_sky_below() {
        // Rolled past 90 degrees the sky is underneath the aircraft, and the display must show it.
        let g = horizon_geometry(180.0_f32.to_radians(), 0.0, CENTRE, W, H);
        assert!(
            g.up.1 > 0.0,
            "inverted, the sky direction should point down the screen: {:?}",
            g.up
        );
    }

    fn flying() -> HudInputs {
        HudInputs {
            has_vehicle: true,
            roll: 5.0,
            pitch: 2.0,
            heading: 90.0,
            target_heading: 110.0,
            ground_course: 92.0,
            xtrack_error: 3.0,
            turn_rate: 2.0,
            airspeed: 18.0,
            ground_speed: 17.0,
            target_speed: 20.0,
            altitude: 100.0,
            target_altitude: 105.0,
            ground_altitude: 0.0,
            vertical_speed: 1.5,
            mode: "Auto".to_owned(),
            mode_changed_for: Some(Duration::from_secs(30)),
            wp_distance: 250.0,
            wp_number: 3,
            link_quality: 100,
            clock: "12:34:56".to_owned(),
            battery_voltage: 12.6,
            battery_current: 3.2,
            battery_remaining: 85,
            battery_low: false,
            battery_critical: false,
            gps_fix: 3,
            armed: true,
            armed_for: Some(Duration::from_secs(30)),
            failsafe: false,
            safety: false,
            message: None,
            aoa_ssa: None,
            custom_items: Vec::new(),
            vibe: [3.0, 4.0, 5.0],
            cpu_load: 12.0,
            ekf_status: 0.1,
            prearm_ready: true,
            russian: false,
            units: DisplayUnits::default().change_units(None, None, None),
            display_cell_voltage: false,
            battery_cell_count: 0,
            display_icons: false,
            battery_voltage2: 0.0,
            battery_current2: 0.0,
            battery_remaining2: 0,
            gps_fix2: 0,
        }
    }

    /// `flying()` in feet and knots, as the Planner page sets them.
    fn in_feet_and_knots() -> HudInputs {
        HudInputs {
            units: DisplayUnits::default().change_units(Some("Feet"), Some("Feet"), Some("knots")),
            ..flying()
        }
    }

    /// A fact by its key.
    fn fact(facts: &[(&'static str, String)], key: &str) -> String {
        facts
            .iter()
            .find(|(name, _)| *name == key)
            .map(|(_, value)| value.clone())
            .unwrap_or_else(|| panic!("no {key} in {facts:?}"))
    }

    /// `flying()` with an angle of attack and sideslip, as a vehicle sending AOA_SSA would give.
    fn with_angles(aoa: f32, ssa: f32) -> HudInputs {
        HudInputs {
            aoa_ssa: Some(AoaSsa {
                aoa,
                ssa,
                crit_aoa: 25.0,
            }),
            ..flying()
        }
    }

    /// The fontsize and bottom text lines `scene` uses, for asserting positions.
    fn text_lines(h: f32) -> (f32, f32, f32) {
        let fontsize = (h / 30.0).max(9.0);
        let bot = fontsize / 3.0;
        let text_offset = fontsize + bot + 2.0;
        (
            fontsize,
            h - 2.0 * text_offset - bot - 4.0,
            h - text_offset - bot - 4.0,
        )
    }

    /// The one label with this text: where it is, its size and its colour.
    fn label_at(scene: &Scene, wanted: &str) -> Option<((f32, f32), f32, u32)> {
        let mut found = scene.items.iter().filter_map(|item| match item {
            Item::Label {
                text,
                at,
                size,
                colour,
                ..
            } if text == wanted => Some((*at, *size, *colour)),
            _ => None,
        });
        let first = found.next();
        assert!(found.next().is_none(), "more than one {wanted:?}");
        first
    }

    fn close(a: (f32, f32), b: (f32, f32)) -> bool {
        (a.0 - b.0).abs() < 0.01 && (a.1 - b.1).abs() < 0.01
    }

    /// What `main.rs`'s `hud_inputs` makes of a vehicle: the angles through the display's
    /// `displayAOASSA`, then `from_vehicle` with the parameter table.
    fn live(state: &VehicleState, timing: &mut Timing, parameters: &[(String, f64)]) -> HudInputs {
        let display_aoa_ssa = timing.display_aoa_ssa(state.aoa, state.ssa);
        HudInputs::from_vehicle(
            state,
            "Stabilize".to_owned(),
            None,
            None,
            String::new(),
            None,
            display_aoa_ssa,
            parameters,
        )
    }

    /// An `AOA_SSA` message with these angles, in degrees as the vehicle sends them.
    fn aoa_ssa(aoa: f32, ssa: f32) -> MavMessage {
        MavMessage::AoaSsa(AoaSsaMessage {
            time_usec: 0,
            aoa,
            ssa,
        })
    }

    /// The coverage table is held to the code: every element it calls drawn is in the scene
    /// given every value, and in the scene `from_vehicle` makes of a vehicle that has sent them -
    /// the path the display takes; every element it calls blocked is drawn when given its value
    /// and never from a live vehicle.
    #[test]
    fn the_coverage_table_matches_what_the_scene_draws() {
        let given = scene(&with_angles(4.0, -2.0), W, H);
        let mut state = VehicleState::default();
        state.ekf.seen = true;
        state.vibration.seen = true;
        state.apply(&aoa_ssa(4.0, -2.0));
        let live = scene(&live(&state, &mut Timing::default(), &[]), W, H);
        for (element, status) in ELEMENTS {
            match status {
                Status::Drawn => {
                    for (which, drawn) in [("given", &given), ("live", &live)] {
                        assert!(
                            drawn.drawn.contains(element),
                            "{} is listed as drawn and the {which} scene did not draw it",
                            element.name()
                        );
                    }
                }
                Status::Blocked(why) => {
                    assert!(
                        given.drawn.contains(element),
                        "{} is blocked, not unported: given its value it must draw",
                        element.name()
                    );
                    assert!(
                        !live.drawn.contains(element),
                        "{} is drawn from a live vehicle; update ELEMENTS",
                        element.name()
                    );
                    assert!(why.contains("CurrentState."), "{why}");
                }
            }
        }
        // Every element of doPaint() has a row, once.
        let mut names: Vec<&str> = ELEMENTS.iter().map(|(e, _)| e.name()).collect();
        let count = names.len();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), count);
        assert_eq!(ELEMENTS.len(), 24, "doPaint() has 24 elements");
        // Nothing a live vehicle cannot show, and the facts say so in a word a script can match.
        let report = missing_report();
        assert!(missing().is_empty(), "{report}");
        assert_eq!(report, "none");
    }

    /// `displayAOASSA`: off until an angle is not 0 - by the C#'s `!=`, so -0 leaves it off and
    /// a NaN turns it on - and never off again. C#: ExtLibs/Controls/HUD.cs:276, 889-930
    #[test]
    fn display_aoa_ssa_turns_on_at_the_first_non_zero_angle_and_stays_on() {
        let mut timing = Timing::default();
        assert!(!timing.display_aoa_ssa(0.0, 0.0), "off when made");
        assert!(!timing.display_aoa_ssa(-0.0, -0.0), "-0 is 0");
        assert!(
            timing.display_aoa_ssa(0.0, 0.5),
            "a sideslip alone turns it on"
        );
        assert!(
            timing.display_aoa_ssa(0.0, 0.0),
            "and a later 0 does not turn it off"
        );
        let mut timing = Timing::default();
        assert!(
            timing.display_aoa_ssa(-3.0, 0.0),
            "an angle of attack alone"
        );
        let mut timing = Timing::default();
        assert!(timing.display_aoa_ssa(f32::NAN, 0.0), "NaN != 0");
    }

    /// `crit_AOA`: the `AOA_CRIT` parameter cast to `int`, or 25 without it.
    /// C#: ExtLibs/ArduPilot/CurrentState.cs:1017-1036
    #[test]
    fn crit_aoa_is_the_parameter_cut_to_a_whole_number_or_25() {
        let with = |value: f64| {
            crit_aoa(&[
                ("AHRS_ORIENT".to_owned(), 0.0),
                ("AOA_CRIT".to_owned(), value),
                ("ARSPD_USE".to_owned(), 1.0),
            ])
        };
        assert_eq!(crit_aoa(&[]), 25.0, "no parameters read yet");
        assert_eq!(
            crit_aoa(&[("aoa_crit".to_owned(), 12.0)]),
            25.0,
            "ContainsKey matches the name exactly"
        );
        assert_eq!(with(20.0), 20.0);
        assert_eq!(with(15.9), 15.0, "(int) cuts, it does not round");
        assert_eq!(with(-3.7), -3.0, "toward zero");
        assert_eq!(with(0.0), 0.0, "a 0 is taken, not replaced by 25");
    }

    /// The angles reach the display from an `AOA_SSA` message through `displayAOASSA`: nothing
    /// until the vehicle sends an angle that is not 0; then the flight path vector at the
    /// message's degrees and the AOA scale against `AOA_CRIT`; and both stay up when the angles
    /// go back to 0. C#: ExtLibs/ArduPilot/CurrentState.cs:1017-1036, 3903-3911,
    /// ExtLibs/Controls/HUD.cs:889-930, 2232-2245, 2804-2843
    #[test]
    fn the_angles_come_from_aoa_ssa_once_one_is_not_zero() {
        let mut state = VehicleState::default();
        let mut timing = Timing::default();
        let parameters = vec![("AOA_CRIT".to_owned(), 15.9)];
        let shown = |inputs: &HudInputs, want: bool| {
            let facts = health_facts(&scene(inputs, W, H));
            for key in ["hud.fpv.shown", "hud.aoa.shown"] {
                assert!(
                    facts.contains(&(key, want.to_string())),
                    "{key} not {want} in {facts:?}"
                );
            }
        };

        let before = live(&state, &mut timing, &parameters);
        assert_eq!(before.aoa_ssa, None, "nothing sent");
        shown(&before, false);
        state.apply(&aoa_ssa(0.0, 0.0));
        let zeroes = live(&state, &mut timing, &parameters);
        assert_eq!(zeroes.aoa_ssa, None, "zeroes change nothing");
        shown(&zeroes, false);

        state.apply(&aoa_ssa(15.0, -2.5));
        let flying = live(&state, &mut timing, &parameters);
        assert_eq!(
            flying.aoa_ssa,
            Some(AoaSsa {
                aoa: 15.0,
                ssa: -2.5,
                crit_aoa: 15.0
            })
        );
        shown(&flying, true);
        let drawn = scene(&flying, W, H);
        // The ring's centre: the sideslip to the left, the angle of attack down.
        let ppd = H / 65.0;
        let centre = (W / 2.0 - 2.5 * ppd, H / 2.0 + 15.0 * ppd);
        let ring = drawn
            .items
            .iter()
            .find_map(|item| match item {
                Item::Stroke { points, colour, .. }
                    if points.len() > 8 && *colour == colour::ALERT =>
                {
                    Some(points.clone())
                }
                _ => None,
            })
            .expect("a red ring");
        for p in &ring {
            let r = (p.0 - centre.0).hypot(p.1 - centre.1);
            assert!(
                (r - W / 2.0 / 40.0).abs() < 0.01,
                "{p:?} is {r} from {centre:?}"
            );
        }
        // At the critical angle the arrow's tip sits on the red band's bottom edge, a tenth of
        // the way down the bar; with the default 25 it would be at 0.42.
        let tip = drawn
            .items
            .iter()
            .find_map(|item| match item {
                Item::Fill { points, colour, .. }
                    if *colour == colour::BLACK && points.len() == 3 =>
                {
                    Some(points[0])
                }
                _ => None,
            })
            .expect("an arrow");
        let (left, top, width, height) = (W - W / 6.0, H / 2.0 + H / 20.0, W / 25.0, H / 5.0);
        assert!(
            close(tip, (left + width / 5.0, top + 0.1 * height)),
            "{tip:?}"
        );

        state.apply(&aoa_ssa(0.0, 0.0));
        let level = live(&state, &mut timing, &parameters);
        assert_eq!(
            level.aoa_ssa,
            Some(AoaSsa {
                aoa: 0.0,
                ssa: 0.0,
                crit_aoa: 15.0
            }),
            "back at 0 and still shown"
        );
        shown(&level, true);
    }

    /// The flight path vector sits at the sideslip across and the angle of attack down from the
    /// centre, a red circle of half-width/40 with wings and a fin, and is absent until the
    /// vehicle has sent an angle. C#: HUD.cs:2232-2245
    #[test]
    fn the_flight_path_vector_follows_the_angles_and_hides_without_them() {
        assert!(
            !scene(&flying(), W, H)
                .drawn
                .contains(&Element::FlightPathVector)
        );
        let scene = scene(&with_angles(4.0, -2.0), W, H);
        assert!(scene.drawn.contains(&Element::FlightPathVector));
        let ppd = H / 65.0;
        let centre = (W / 2.0 - 2.0 * ppd, H / 2.0 + 4.0 * ppd);
        let radius = W / 2.0 / 40.0;
        let ring = scene
            .items
            .iter()
            .find_map(|item| match item {
                Item::Stroke {
                    points,
                    colour,
                    width,
                    ..
                } if points.len() > 8 && *colour == colour::ALERT => Some((points.clone(), *width)),
                _ => None,
            })
            .expect("a red ring");
        assert_eq!(ring.1, 2.0, "the C#'s red pen is 2 wide");
        for p in &ring.0 {
            let r = (p.0 - centre.0).hypot(p.1 - centre.1);
            assert!(
                (r - radius).abs() < 0.01,
                "ring point {p:?} is {r} from {centre:?}"
            );
        }
        let (outer, inner) = (W / 2.0 / 20.0, W / 2.0 / 40.0);
        for (from, to) in [
            ((centre.0 - outer, centre.1), (centre.0 - inner, centre.1)),
            ((centre.0 + outer, centre.1), (centre.0 + inner, centre.1)),
            ((centre.0, centre.1 - outer), (centre.0, centre.1 - inner)),
        ] {
            let found = scene.items.iter().any(|item| {
                matches!(item, Item::Stroke { points, colour, .. }
                    if *colour == colour::ALERT && points.len() == 2
                        && close(points[0], from) && close(points[1], to))
            });
            assert!(found, "no red stroke from {from:?} to {to:?}");
        }
    }

    /// The AOA scale's bands and arrow: 10/30/50/10 percent of the bar in red, yellow, green
    /// and blue from the top; the arrow's tip at the green band's bottom at zero, at the red
    /// band's bottom at the critical angle, and held to the bar beyond. C#: HUD.cs:2804-2843
    #[test]
    fn the_aoa_scale_puts_the_arrow_by_the_critical_angle() {
        assert!(!scene(&flying(), W, H).drawn.contains(&Element::Aoa));
        let (left, top, width, height) = (W - W / 6.0, H / 2.0 + H / 20.0, W / 25.0, H / 5.0);
        let scene_at = |aoa: f32| scene(&with_angles(aoa, 0.0), W, H);
        let level = scene_at(0.0);
        assert!(level.drawn.contains(&Element::Aoa));
        let bands: Vec<(u32, f32, f32)> = level
            .items
            .iter()
            .filter_map(|item| match item {
                Item::Fill { points, colour, .. }
                    if points.len() == 4 && (points[0].0 - left).abs() < 0.01 =>
                {
                    Some((*colour, points[0].1, points[2].1))
                }
                _ => None,
            })
            .collect();
        let expected = [
            (colour::ALERT, top, top + 0.1 * height),
            (colour::CAUTION, top + 0.1 * height, top + 0.4 * height),
            (colour::TARGET, top + 0.4 * height, top + 0.9 * height),
            (colour::VSI, top + 0.9 * height, top + height),
        ];
        assert_eq!(bands.len(), 4, "{bands:?}");
        for ((colour, from, to), (want_colour, want_from, want_to)) in bands.iter().zip(expected) {
            assert_eq!(*colour, want_colour);
            assert!(
                (from - want_from).abs() < 0.01 && (to - want_to).abs() < 0.01,
                "{bands:?}"
            );
        }
        let tip = |scene: &Scene| {
            scene
                .items
                .iter()
                .find_map(|item| match item {
                    Item::Fill { points, colour, .. }
                        if *colour == colour::BLACK && points.len() == 3 =>
                    {
                        Some(points[0])
                    }
                    _ => None,
                })
                .expect("an arrow")
        };
        let x = left + width / 5.0;
        assert!(
            close(tip(&level), (x, top + 0.9 * height)),
            "{:?}",
            tip(&level)
        );
        assert!(close(tip(&scene_at(25.0)), (x, top + 0.1 * height)));
        assert!(close(tip(&scene_at(60.0)), (x, top)), "held at the top");
        assert!(
            close(tip(&scene_at(-60.0)), (x, top + height)),
            "held at the bottom"
        );
    }

    /// Extra fields go up from above the battery line at an eighth of the width, each in the
    /// format its property's name picks, and one that cannot be read leaves no gap.
    /// C#: HUD.cs:3029-3077
    #[test]
    fn custom_items_stack_upward_in_the_csharps_formats() {
        let item = |header: &str, name: &str, value: Option<f64>| CustomItem {
            header: header.to_owned(),
            name: name.to_owned(),
            value,
        };
        let mut inputs = flying();
        inputs.custom_items = vec![
            item("Lat: ", "lat", Some(-35.363_262_18)),
            item("Gone: ", "nothing", None),
            item("mAh: ", "battery_usedmah", Some(1234.5)),
            item("Air: ", "timeInAir", Some(3725.9)),
            item("Hello", "string", Some(0.0)),
            item("Dist: ", "wp_dist", Some(12.345)),
        ];
        let scene = scene(&inputs, W, H);
        let texts: Vec<&str> = scene
            .labels_of(Element::CustomItems)
            .into_iter()
            .map(|(text, _)| text)
            .collect();
        assert_eq!(
            texts,
            [
                "Lat: -35.3632622",
                "mAh: 1235",
                "Air: 01:02:05",
                "Hello",
                "Dist: 12.35"
            ]
        );
        let fontsize = (H / 30.0).max(9.0);
        let fontoffset = (fontsize - 10.0).max(0.0);
        let first = H - (fontsize + 2.0) * 3.0 - fontoffset - fontsize - 8.0;
        for (row, text) in texts.iter().enumerate() {
            let (at, size, colour) = label_at(&scene, text).expect("drawn");
            let y = first - (fontsize + 5.0) * row as f32;
            assert!(close(at, (W / 8.0, y)), "{text} at {at:?}, wanted y {y}");
            assert_eq!(size, fontsize + 2.0);
            assert_eq!(colour, colour::INK);
        }
        assert!(!scene.labels().iter().any(|l| l.starts_with("Gone")));
        let facts = health_facts(&scene);
        assert!(
            facts.contains(&("hud.custom.count", "5".to_owned())),
            "{facts:?}"
        );
    }

    /// .NET's "0.##" rounds half away from zero on fifteen significant digits and drops
    /// trailing zeros; Rust's `{:.2}` does neither.
    #[test]
    fn format_hash_writes_numbers_as_dotnet_does() {
        assert_eq!(format_hash(7.4321, 2), "7.43");
        assert_eq!(format_hash(2.0, 2), "2");
        assert_eq!(format_hash(0.125, 2), "0.13");
        assert_eq!(format_hash(1.005, 2), "1.01");
        assert_eq!(format_hash(99.995, 2), "100");
        assert_eq!(format_hash(0.006, 2), "0.01");
        assert_eq!(format_hash(-0.001, 2), "0");
        assert_eq!(format_hash(1e-10, 2), "0");
        assert_eq!(format_hash(2.5, 0), "3");
        assert_eq!(format_hash(-2.5, 0), "-3");
        assert_eq!(format_hash(1.5e20, 2), "150000000000000000000");
        assert_eq!(format_hash(-35.363_262_18, 7), "-35.3632622");
        assert_eq!(format_hash(f64::NAN, 2), "NaN");
        assert_eq!(format_hash(f64::NEG_INFINITY, 2), "-Infinity");
        assert_eq!(two_digits(-5), "-05");
    }

    /// "Vibe": white up to 30 on every axis, orange past 30 on any, red past 60, at
    /// width - 18 characters on the lower text line. C#: HUD.cs:3148-3204
    #[test]
    fn vibe_is_coloured_by_the_worst_axis() {
        let (fontsize, _, lower) = text_lines(H);
        let colour_at = |vibe: [f32; 3]| {
            let inputs = HudInputs { vibe, ..flying() };
            let scene = scene(&inputs, W, H);
            let (at, size, colour) = label_at(&scene, "Vibe").expect("Vibe is always drawn");
            assert!(close(at, (W - 18.0 * fontsize, lower)), "{at:?}");
            assert_eq!(size, fontsize + 2.0);
            colour
        };
        assert_eq!(colour_at([0.0, 0.0, 0.0]), colour::INK);
        assert_eq!(
            colour_at([30.0, 30.0, 30.0]),
            colour::INK,
            "30 is not past 30"
        );
        assert_eq!(colour_at([0.0, 30.5, 0.0]), colour::WARN);
        assert_eq!(
            colour_at([60.0, 0.0, 0.0]),
            colour::WARN,
            "60 is not past 60"
        );
        assert_eq!(colour_at([0.0, 0.0, 61.0]), colour::ALERT);
    }

    /// "CPU" appears in red at the right of Vibe's box only at a load of exactly 100: a
    /// `SYS_STATUS.load` of 1000, the message counting in tenths of a percent.
    /// C#: HUD.cs:3206-3207, ExtLibs/ArduPilot/CurrentState.cs:2949
    #[test]
    fn cpu_shows_only_at_full_load() {
        let (fontsize, _, lower) = text_lines(H);
        let mut state = VehicleState::default();
        assert_eq!(live(&state, &mut Timing::default(), &[]).cpu_load, 0.0);
        for (raw, shown) in [(0, false), (999, false), (1000, true), (1001, false)] {
            let Some(MavMessage::SysStatus(mut sys_status)) = MavMessage::decode(1, &[]) else {
                panic!("no SYS_STATUS")
            };
            sys_status.load = raw;
            state.apply(&MavMessage::SysStatus(sys_status));
            let inputs = live(&state, &mut Timing::default(), &[]);
            assert_eq!(inputs.cpu_load, f32::from(raw) / 10.0);
            let scene = scene(&inputs, W, H);
            let cpu = label_at(&scene, "CPU");
            assert_eq!(cpu.is_some(), shown, "{raw}");
            if let Some((at, _, colour)) = cpu {
                assert!(close(at, (W - 18.0 * fontsize + 40.0, lower)), "{at:?}");
                assert_eq!(colour, colour::ALERT);
            }
            let facts = health_facts(&scene);
            assert!(facts.contains(&("hud.cpu", shown.to_string())), "{facts:?}");
        }
    }

    /// "EKF": white up to 0.5, orange past it, red past 0.8, at width - 23 characters on the
    /// lower text line. C#: HUD.cs:3209-3262
    #[test]
    fn ekf_is_coloured_by_the_status() {
        let (fontsize, _, lower) = text_lines(H);
        let colour_at = |ekf_status: f32| {
            let inputs = HudInputs {
                ekf_status,
                ..flying()
            };
            let scene = scene(&inputs, W, H);
            let (at, size, colour) = label_at(&scene, "EKF").expect("EKF is always drawn");
            assert!(close(at, (W - 23.0 * fontsize, lower)), "{at:?}");
            assert_eq!(size, fontsize + 2.0);
            colour
        };
        assert_eq!(colour_at(0.0), colour::INK);
        assert_eq!(colour_at(0.5), colour::INK, "0.5 is not past 0.5");
        assert_eq!(colour_at(0.51), colour::WARN);
        assert_eq!(colour_at(0.8), colour::WARN, "0.8 is not past 0.8");
        assert_eq!(colour_at(0.81), colour::ALERT);
        assert_eq!(colour_at(f32::NAN), colour::INK);
    }

    /// `ekfstatus` is the worst of the five variances, terrain included, and 1 when the flags
    /// say there is no attitude, no horizontal velocity with a GPS fix, or no initialisation.
    /// C#: ExtLibs/ArduPilot/CurrentState.cs:2649-2741
    #[test]
    fn ekf_status_follows_current_state() {
        let mut state = VehicleState::default();
        state.ekf.terrain_altitude_variance = 0.9;
        assert_eq!(ekf_status(&state), 0.0, "nothing heard yet");
        state.ekf.seen = true;
        state.ekf.flags = EKF_ATTITUDE | EKF_VELOCITY_HORIZ;
        state.ekf.velocity_variance = 0.2;
        assert_eq!(ekf_status(&state), 0.9, "terrain counts here");
        state.ekf.terrain_altitude_variance = 0.0;
        state.ekf.compass_variance = 0.6;
        assert_eq!(ekf_status(&state), 0.6);
        state.ekf.flags = EKF_VELOCITY_HORIZ;
        assert_eq!(ekf_status(&state), 1.0, "no attitude");
        state.ekf.flags = EKF_ATTITUDE;
        assert_eq!(ekf_status(&state), 0.6, "no velocity but no GPS either");
        state.gps.fix_type = 3;
        assert_eq!(ekf_status(&state), 1.0, "no velocity with a fix");
        state.ekf.flags = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_UNINITIALIZED;
        assert_eq!(ekf_status(&state), 1.0, "uninitialised");
        state.ekf.flags = EKF_ATTITUDE | EKF_VELOCITY_HORIZ;
        state.ekf.position_vertical_variance = f32::NAN;
        assert!(ekf_status(&state).is_nan(), "Math.Max carries a NaN");
    }

    /// While disarmed, "Ready to Arm" in white at width - 24 characters or "Not Ready to Arm"
    /// in red at width - 26, on the upper text line less four; armed, neither.
    /// C#: HUD.cs:3264-3301
    #[test]
    fn prearm_shows_only_while_disarmed() {
        let (fontsize, upper, _) = text_lines(H);
        let armed = scene(&flying(), W, H);
        assert!(armed.labels_of(Element::Prearm).is_empty());
        assert!(health_facts(&armed).contains(&("hud.prearm", "none".to_owned())));
        let mut inputs = flying();
        inputs.armed = false;
        let ready = scene(&inputs, W, H);
        let (at, size, colour) = label_at(&ready, "Ready to Arm").expect("shown");
        assert!(close(at, (W - 24.0 * fontsize, upper - 4.0)), "{at:?}");
        assert_eq!((size, colour), (fontsize + 2.0, colour::INK));
        inputs.prearm_ready = false;
        let not_ready = scene(&inputs, W, H);
        let (at, _, colour) = label_at(&not_ready, "Not Ready to Arm").expect("shown");
        assert!(close(at, (W - 26.0 * fontsize, upper - 4.0)), "{at:?}");
        assert_eq!(colour, colour::ALERT);
        let facts = health_facts(&not_ready);
        assert!(facts.contains(&("hud.prearm", "Not Ready to Arm".to_owned())));
        assert!(facts.contains(&("hud.prearm.colour", "red".to_owned())));
    }

    /// `prearmstatus`: the pre-arm bit healthy, or not enabled - so a vehicle that has sent no
    /// SYS_STATUS reads as ready. C#: ExtLibs/ArduPilot/CurrentState.cs:179-182
    #[test]
    fn prearm_ready_follows_the_prearm_sensor_bit() {
        let mut state = VehicleState::default();
        assert!(prearm_ready(&state), "not enabled");
        state.sensors.enabled = PREARM_CHECK;
        assert!(!prearm_ready(&state), "enabled and failing");
        state.sensors.health = PREARM_CHECK;
        assert!(prearm_ready(&state), "enabled and passing");
    }

    /// `from_vehicle` feeds the health readouts from the vehicle, and the facts read the scene
    /// it makes. This vehicle is SITL's copter: a part load, no `AOA_SSA`, so the angles stay
    /// off whatever the parameters say.
    #[test]
    fn from_vehicle_feeds_the_health_readouts() {
        let mut state = VehicleState::default();
        state.vibration.x = 45.0;
        state.vibration.seen = true;
        state.ekf.seen = true;
        state.ekf.flags = EKF_ATTITUDE | EKF_VELOCITY_HORIZ;
        state.ekf.compass_variance = 0.9;
        state.sensors.enabled = PREARM_CHECK;
        state.sensors.reported = true;
        state.load = 35.0;
        let inputs = live(
            &state,
            &mut Timing::default(),
            &[("AOA_CRIT".to_owned(), 18.0)],
        );
        assert_eq!(inputs.vibe, [45.0, 0.0, 0.0]);
        assert_eq!(inputs.ekf_status, 0.9);
        assert!(!inputs.prearm_ready);
        assert_eq!(inputs.aoa_ssa, None);
        assert_eq!(inputs.cpu_load, 35.0);
        assert!(inputs.custom_items.is_empty());
        let facts = health_facts(&scene(&inputs, 800.0, 260.0));
        for expected in [
            ("hud.vibe.colour", "orange"),
            ("hud.ekf.colour", "red"),
            ("hud.prearm", "Not Ready to Arm"),
            ("hud.prearm.colour", "red"),
            ("hud.cpu", "false"),
            ("hud.custom.count", "0"),
            ("hud.fpv.shown", "false"),
            ("hud.aoa.shown", "false"),
        ] {
            assert!(
                facts.contains(&(expected.0, expected.1.to_owned())),
                "{expected:?} not in {facts:?}"
            );
        }
    }

    /// Without a vehicle the display is the horizon at rest and nothing else.
    #[test]
    fn no_vehicle_draws_the_horizon_and_no_readouts() {
        let scene = scene(&HudInputs::default(), W, H);
        assert!(scene.drawn.contains(&Element::SkyGround));
        assert!(!scene.drawn.contains(&Element::HeadingTape));
        assert!(
            scene
                .labels()
                .iter()
                .all(|l| l.trim().parse::<i32>().is_ok()),
            "{:?}",
            scene.labels()
        );
    }

    /// The ladder shows rungs from 29° below the pitch to 20° above it, every 5°.
    /// C#: HUD.cs:2119-2120
    #[test]
    fn the_pitch_ladder_is_clipped_around_the_pitch() {
        let mut inputs = flying();
        inputs.pitch = 0.0;
        let scene = scene(&inputs, W, H);
        let numbers: Vec<i32> = scene
            .labels()
            .iter()
            .filter_map(|l| l.parse::<i32>().ok())
            .filter(|n| n.abs() <= 90 && n % 10 == 0)
            .collect();
        assert!(numbers.contains(&-20), "{numbers:?}");
        assert!(numbers.contains(&20), "{numbers:?}");
        assert!(
            !numbers.contains(&-30),
            "-30 is 30 below, outside -29: {numbers:?}"
        );
        assert!(!numbers.contains(&30), "30 is above +20: {numbers:?}");
    }

    /// The target-heading mark sits at the target's place on the tape, 1/120 of the width
    /// per degree from 60° left of the heading.
    #[test]
    fn the_target_heading_bug_is_where_the_target_is() {
        let inputs = flying(); // heading 90, target 110
        let scene = scene(&inputs, W, H);
        let space = (W - 10.0) / 120.0;
        let expected_x = 5.0 + space * (110.0 - 30.0);
        let bug = scene.items.iter().find(|item| {
            matches!(item, Item::Stroke { points, width, colour, .. }
                if *colour == colour::TARGET && *width == 6.0 && (points[0].0 - expected_x).abs() < 0.01)
        });
        assert!(bug.is_some(), "no green 6px mark at x={expected_x}");
        assert!(scene.drawn.contains(&Element::HeadingBugs));
    }

    /// A target beyond the tape's edge is shown at that edge - both edges, not the middle.
    #[test]
    fn a_target_beyond_the_tape_is_pinned_to_its_nearest_edge() {
        let mut inputs = flying();
        inputs.heading = 90.0;
        inputs.target_heading = 200.0;
        let scene = scene(&inputs, W, H);
        let right_edge = 5.0 + (W - 10.0);
        let at_edge = scene.items.iter().any(|item| {
            matches!(item, Item::Stroke { points, width, colour, .. }
                if *colour == colour::TARGET && *width == 6.0 && (points[0].0 - right_edge).abs() < 0.01)
        });
        assert!(
            at_edge,
            "the bug should sit at the right edge, not the C#'s middle"
        );
    }

    /// Cross-track error clamps at ±40 m and fades when clamped.
    #[test]
    fn the_xtrack_bar_clamps_and_fades() {
        let mut inputs = flying();
        inputs.xtrack_error = 400.0;
        let scene = scene(&inputs, W, H);
        let xtspace = W / 10.0 / 3.0;
        let expected_x = W / 10.0 + 40.0 / 20.0 * xtspace;
        let marker = scene.items.iter().find_map(|item| match item {
            Item::Stroke {
                points,
                colour,
                alpha,
                width,
                ..
            } if *colour == colour::TARGET
                && *width == 2.0
                && (points[0].0 - expected_x).abs() < 0.01 =>
            {
                Some(*alpha)
            }
            _ => None,
        });
        assert_eq!(marker, Some(0.5), "clamped marker should be faded");
    }

    /// ARMED shows for eight seconds after arming and then goes; DISARMED shows whenever disarmed.
    #[test]
    fn the_armed_banner_times_out_and_disarmed_does_not() {
        let mut inputs = flying();
        inputs.armed_for = Some(Duration::from_secs(3));
        assert!(scene(&inputs, W, H).labels().contains(&"ARMED"));
        inputs.armed_for = Some(Duration::from_secs(9));
        assert!(!scene(&inputs, W, H).labels().contains(&"ARMED"));
        inputs.armed_for = None;
        assert!(!scene(&inputs, W, H).labels().contains(&"ARMED"));
        inputs.armed = false;
        for since in [
            None,
            Some(Duration::from_secs(1)),
            Some(Duration::from_secs(600)),
        ] {
            inputs.armed_for = since;
            assert!(
                scene(&inputs, W, H).labels().contains(&"DISARMED"),
                "{since:?}"
            );
        }
    }

    /// Failsafe, safety and the message each show only when they apply.
    #[test]
    fn failsafe_safety_and_message_are_conditional() {
        let mut inputs = flying();
        let quiet = scene(&inputs, W, H);
        assert!(!quiet.labels().contains(&"FAILSAFE"));
        assert!(!quiet.labels().contains(&"SAFE"));
        inputs.failsafe = true;
        inputs.safety = true;
        inputs.message = Some(("ERROR velocity variance".to_owned(), colour::ALERT));
        let loud = scene(&inputs, W, H);
        assert!(loud.labels().contains(&"FAILSAFE"));
        assert!(loud.labels().contains(&"SAFE"));
        assert!(loud.labels().contains(&"ERROR velocity variance"));
    }

    /// The vertical speed bar clamps at ±6 m/s and points the right way.
    #[test]
    fn the_vsi_bar_points_up_for_a_climb_and_clamps() {
        let mut inputs = flying();
        inputs.vertical_speed = 3.0;
        let climbing = scene(&inputs, W, H);
        let bar = |s: &Scene| {
            s.items.iter().find_map(|item| match item {
                Item::Fill { points, colour, .. } if *colour == colour::VSI => Some(points.clone()),
                _ => None,
            })
        };
        let up = bar(&climbing).expect("a vsi bar");
        assert!(
            up[3].1 < up[0].1,
            "a climb draws the bar upward (smaller y): {up:?}"
        );
        inputs.vertical_speed = 60.0;
        let clamped = bar(&scene(&inputs, W, H)).expect("a vsi bar");
        let box_top = H / 2.0 - H / 4.0;
        assert!(
            clamped[3].1 >= box_top - 0.01,
            "clamped at the top of the box: {clamped:?}"
        );
    }

    /// The GPS line names the fix as HUDT does, red without one.
    #[test]
    fn the_gps_line_names_the_fix() {
        assert_eq!(gps_fix_text(3), "GPS: 3D Fix");
        assert_eq!(gps_fix_text(6), "GPS: rtk Fixed");
        assert_eq!(gps_fix_text(9), "9");
        let mut inputs = flying();
        inputs.gps_fix = 0;
        let scene = scene(&inputs, W, H);
        let red = scene.items.iter().any(|item| {
            matches!(item, Item::Label { text, colour, .. } if text == "GPS: No GPS" && *colour == colour::ALERT)
        });
        assert!(red);
    }

    /// The timing keeper reports how long since arming and since the mode changed.
    #[test]
    fn timing_reports_time_since_each_transition() {
        let mut timing = Timing::default();
        let t0 = Instant::now();
        assert_eq!(timing.observe(false, 0, t0), (None, None));
        let (armed, mode) = timing.observe(true, 0, t0 + Duration::from_secs(1));
        assert_eq!(armed, Some(Duration::ZERO));
        assert_eq!(mode, None);
        let (armed, mode) = timing.observe(true, 3, t0 + Duration::from_secs(5));
        assert_eq!(armed, Some(Duration::from_secs(4)));
        assert_eq!(mode, Some(Duration::ZERO));
        let (armed, _) = timing.observe(false, 3, t0 + Duration::from_secs(6));
        assert_eq!(armed, Some(Duration::ZERO), "disarming is a transition too");
    }

    /// A message stays up ten seconds after it was last raised, then goes.
    #[test]
    fn a_high_priority_message_lives_ten_seconds() {
        let mut timing = Timing::default();
        let t0 = Instant::now();
        assert_eq!(timing.message(None, t0), None);
        let shown = timing.message(Some("Error compass variance".to_owned()), t0);
        assert_eq!(
            shown.as_ref().map(|m| m.0.as_str()),
            Some("Error compass variance")
        );
        assert!(timing.message(None, t0 + Duration::from_secs(9)).is_some());
        assert!(timing.message(None, t0 + Duration::from_secs(11)).is_none());
        // Raised again, it starts over.
        assert!(
            timing
                .message(Some("(SAFETY)".to_owned()), t0 + Duration::from_secs(12))
                .is_some()
        );
    }

    /// The EKF message follows the C#'s order, so the last variance past 1 is the one shown.
    #[test]
    fn the_ekf_message_names_the_last_variance_over_one() {
        let mut state = VehicleState::default();
        assert_eq!(high_priority_message(&state), None);
        state.ekf.seen = true;
        state.ekf.velocity_variance = 1.2;
        assert_eq!(
            high_priority_message(&state).as_deref(),
            Some("Error velocity variance")
        );
        state.ekf.compass_variance = 1.5;
        assert_eq!(
            high_priority_message(&state).as_deref(),
            Some("Error compass variance")
        );
        state.armed = true;
        state.sensors.reported = true;
        state.sensors.enabled = 0;
        assert_eq!(high_priority_message(&state).as_deref(), Some("(SAFETY)"));
    }

    /// The scroller's numbers are every five units around the value.
    #[test]
    fn the_speed_tape_labels_every_five_units_around_the_speed() {
        let mut inputs = flying();
        inputs.airspeed = 23.0;
        let scene = scene(&inputs, W, H);
        let labels = scene.labels();
        for expected in ["   10", "   15", "   20", "   25", "   30", "   35"] {
            assert!(
                labels.contains(&expected),
                "missing {expected:?} in {labels:?}"
            );
        }
        assert!(
            !labels.contains(&"   40"),
            "40 is beyond 23 + 13: {labels:?}"
        );
        assert!(labels.contains(&"23m/s"), "{labels:?}");
        assert!(labels.contains(&"AS 23.0m/s"), "{labels:?}");
    }

    /// `Russian`: the sky and ground stay level; the ladder, the aircraft symbol and the flight
    /// path vector turn with the roll - a right wing down turns them clockwise, where the ordinary
    /// display turns the ladder the other way and holds the symbol still - and the roll pointer
    /// moves to the other side. `// C#: ExtLibs/Controls/HUD.cs:2029-2036, 2107-2110, 2176-2182,
    /// 2210-2245`
    #[test]
    fn a_russian_hud_holds_the_horizon_level_and_turns_the_symbol() {
        let rolled = HudInputs {
            roll: 20.0,
            pitch: 0.0,
            ..flying()
        };
        let russian = HudInputs {
            russian: true,
            ..rolled.clone()
        };
        let level = HudInputs {
            roll: 0.0,
            pitch: 0.0,
            ..flying()
        };
        let (normal, turned, flat) = (
            scene(&rolled, W, H),
            scene(&russian, W, H),
            scene(&level, W, H),
        );
        let (cx, cy) = (W / 2.0, H / 2.0);

        // The sky and the ground are the level display's.
        assert_eq!(turned.items[..2], flat.items[..2]);
        assert_ne!(normal.items[..2], flat.items[..2]);

        // The 0° rung, the first green line: its right end rises on the ordinary display, and
        // falls on the Russian one.
        let rung = |scene: &Scene| {
            scene
                .items
                .iter()
                .find_map(|item| match item {
                    Item::Stroke { points, colour, .. }
                        if *colour == colour::TARGET && points.len() == 2 =>
                    {
                        Some((points[0], points[1]))
                    }
                    _ => None,
                })
                .expect("the 0° rung")
        };
        let (left, right) = rung(&normal);
        assert!(right.1 < left.1, "ordinary: {left:?} {right:?}");
        let (left, right) = rung(&turned);
        assert!(right.1 > left.1, "Russian: {left:?} {right:?}");

        // The wings, the first two red lines three wide: level on the ordinary display, the
        // right one down on the Russian one.
        let wings = |scene: &Scene| -> Vec<((f32, f32), (f32, f32))> {
            scene
                .items
                .iter()
                .filter_map(|item| match item {
                    Item::Stroke {
                        points,
                        colour,
                        width,
                        ..
                    } if *colour == colour::ALERT
                        && (*width - 3.0).abs() < 0.01
                        && points.len() == 2 =>
                    {
                        Some((points[0], points[1]))
                    }
                    _ => None,
                })
                .take(2)
                .collect()
        };
        for (from, to) in wings(&normal) {
            assert!((from.1 - cy).abs() < 0.01 && (to.1 - cy).abs() < 0.01);
        }
        let turned_wings = wings(&turned);
        assert!(turned_wings[0].0.1 < cy, "the left wing tip rises");
        assert!(turned_wings[1].1.1 > cy, "the right wing tip falls");

        // The roll pointer, the red triangle: left of centre on the ordinary display, right of it
        // on the Russian one.
        let pointer = |scene: &Scene| {
            scene
                .items
                .iter()
                .find_map(|item| match item {
                    Item::Fill { points, colour, .. }
                        if *colour == colour::ALERT && points.len() == 3 =>
                    {
                        Some(points[0])
                    }
                    _ => None,
                })
                .expect("the pointer")
        };
        assert!(pointer(&normal).0 < cx);
        assert!(pointer(&turned).0 > cx);

        // The flight path vector turns with the symbol: its fin, straight up from the circle on
        // the ordinary display, leans right on the Russian one.
        let fin = |inputs: &HudInputs| {
            let scene = scene(inputs, W, H);
            let ring = scene
                .items
                .iter()
                .position(|item| {
                    matches!(item, Item::Stroke { points, colour, .. }
                        if points.len() > 8 && *colour == colour::ALERT)
                })
                .expect("the ring");
            // The ring, its two wings, then its fin.
            scene.items[ring + 1..]
                .iter()
                .filter_map(|item| match item {
                    Item::Stroke { points, colour, .. }
                        if *colour == colour::ALERT && points.len() == 2 =>
                    {
                        Some((points[0], points[1]))
                    }
                    _ => None,
                })
                .nth(2)
                .expect("the fin")
        };
        let angled = |inputs: HudInputs| HudInputs {
            aoa_ssa: Some(AoaSsa {
                aoa: 0.0,
                ssa: 0.0,
                crit_aoa: 25.0,
            }),
            ..inputs
        };
        let (top, bottom) = fin(&angled(rolled));
        assert!(
            (top.0 - bottom.0).abs() < 0.01,
            "upright: {top:?} {bottom:?}"
        );
        let (top, bottom) = fin(&angled(russian));
        assert!(top.0 > bottom.0, "leaning right: {top:?} {bottom:?}");
    }

    /// Off by default, and a vehicle's inputs leave it off: it is the display's setting.
    #[test]
    fn russian_is_off_until_the_menu_turns_it_on() {
        assert!(!HudInputs::default().russian);
        let inputs = live(&VehicleState::default(), &mut Timing::default(), &[]);
        assert!(!inputs.russian);
    }

    /// The Designer's and the settings' defaults: no cell voltage, a count of 0, no icons, and
    /// metres and metres per second. A vehicle's inputs leave the menu's settings alone.
    /// `// C#: GCSViews/FlightData.Designer.cs:401-402, GCSViews/FlightData.cs:427, 597-602`
    #[test]
    fn the_menus_settings_start_off() {
        let inputs = HudInputs::default();
        assert!(!inputs.display_cell_voltage);
        assert_eq!(inputs.battery_cell_count, 0);
        assert!(!inputs.display_icons);
        assert_eq!(
            (inputs.units.alt_unit, inputs.units.speed_unit),
            ("m", "m/s")
        );
        let live = live(&VehicleState::default(), &mut Timing::default(), &[]);
        assert!(!live.display_cell_voltage && !live.display_icons);
        assert_eq!(live.battery_cell_count, 0);
    }

    /// The tapes and the waypoint line in the user's units: each number is the SI value times
    /// the C#'s single-precision multiplier, written as .NET writes a float, with the unit's
    /// name after it. `// C#: ExtLibs/Controls/HUD.cs:2552-2577, 2733, 2749-2769,
    /// ExtLibs/ArduPilot/CurrentState.cs:327, 496, 529, 1093, MainV2.cs:4262, 4317`
    #[test]
    fn the_tapes_read_in_the_users_units() {
        let metric = readout_facts(&scene(&flying(), W, H));
        assert_eq!(fact(&metric, "hud.speed"), "18m/s");
        assert_eq!(fact(&metric, "hud.airspeed"), "AS 18.0m/s");
        assert_eq!(fact(&metric, "hud.groundspeed"), "GS 17.0m/s");
        assert_eq!(fact(&metric, "hud.alt"), "100 m");
        assert_eq!(fact(&metric, "hud.wpdist"), "250m>3");

        let converted = in_feet_and_knots();
        let drawn = scene(&converted, W, H);
        let facts = readout_facts(&drawn);
        // 18 m/s is 34.989 kts, 17 is 33.045; 100 m is 328.08 ft, 250 m is 820.21.
        assert_eq!(fact(&facts, "hud.speed"), "35kts");
        assert_eq!(fact(&facts, "hud.airspeed"), "AS 35.0kts");
        assert_eq!(fact(&facts, "hud.groundspeed"), "GS 33.0kts");
        assert_eq!(fact(&facts, "hud.alt"), "328 ft");
        assert_eq!(fact(&facts, "hud.wpdist"), "820ft>3");
        // The tapes are numbered in the unit: every five knots from 21 to 48 around 35, every
        // five feet from 315 to 341 around 328.
        let labels = drawn.labels();
        for number in ["   25", "   45", "  315", "  340"] {
            assert!(labels.contains(&number), "{number:?} not in {labels:?}");
        }
        assert!(!scene(&flying(), W, H).labels().contains(&"   45"));

        // The climb rate in knots: 1.5 m/s is 2.916 kts of the bar's ±6.
        let bar_end = |inputs: &HudInputs| {
            scene(inputs, W, H)
                .items
                .iter()
                .find_map(|item| match item {
                    Item::Fill { points, colour, .. } if *colour == colour::VSI => {
                        Some(points[3].1)
                    }
                    _ => None,
                })
                .expect("a vsi bar")
        };
        let knots: f32 = "1.94384449".parse().expect("a float");
        let (mid, height) = (H / 2.0, H / 2.0);
        assert!((bar_end(&flying()) - (mid - 1.5 / 12.0 * height)).abs() < 0.01);
        assert!((bar_end(&converted) - (mid - 1.5 * knots / 12.0 * height)).abs() < 0.01);
    }

    /// From 1000 of the unit the distance is kilometres where the unit is metres, and miles of
    /// 5280 otherwise, to a place rounded halves to even; under 1000, whole units cut toward
    /// zero. `// C#: ExtLibs/Controls/HUD.cs:2749-2769`
    #[test]
    fn the_waypoint_distance_goes_to_kilometres_or_miles_past_1000() {
        assert_eq!(wp_distance_text(999.9, "m"), "999m");
        assert_eq!(wp_distance_text(250.7, "ft"), "250ft");
        assert_eq!(wp_distance_text(1234.0, "m"), "1.2k");
        // `Math.Round(12.5)` is 12: halves go to even.
        assert_eq!(wp_distance_text(1250.0, "m"), "1.2k");
        assert_eq!(wp_distance_text(1260.0, "m"), "1.3k");
        assert_eq!(wp_distance_text(2000.0, "m"), "2k");
        assert_eq!(wp_distance_text(3280.84, "ft"), "0.6mi");
        assert_eq!(wp_distance_text(5280.0, "ft"), "1mi");
        // No unit set is not "m": miles.
        assert_eq!(wp_distance_text(1000.0, ""), "0.2mi");
        let mut inputs = in_feet_and_knots();
        inputs.wp_distance = 1609.344;
        let facts = readout_facts(&scene(&inputs, W, H));
        assert_eq!(fact(&facts, "hud.wpdist"), "1mi>3");
    }

    /// A float is written from seven significant digits, a double from fifteen; "0.00" keeps
    /// its places. `// C#: ExtLibs/Controls/HUD.cs:2560, 2893, 2898, 2918`
    #[test]
    fn a_float_is_written_as_dotnet_writes_a_float() {
        assert_eq!(format_single_fixed(12.6, 2), "12.60");
        assert_eq!(format_single_fixed(3.0, 1), "3.0");
        assert_eq!(format_single_fixed(1.25, 1), "1.3", "halves away from zero");
        assert_eq!(format_single_fixed(-0.04, 1), "0.0");
        assert_eq!(format_single_fixed(f32::NAN, 2), "NaN");
        assert_eq!(format_single_hash(34.989_2, 0), "35");
        assert_eq!(format_single_hash(2.5, 0), "3");
        assert_eq!(format_single_hash(2.675, 2), "2.68");
        assert_eq!(format_hash(f64::from(2.675_f32), 2), "2.67");
    }

    /// Battery Cell Voltage: with a count that is not 0, "Cell" and the pack's voltage over it
    /// on the lower line at two sizes up, and the pack's line moved to the upper; a count of 0,
    /// or the setting off, leaves the pack's line where it was. The count is any `int`.
    /// `// C#: ExtLibs/Controls/HUD.cs:2896-2923, GCSViews/FlightData.cs:6115-6140`
    #[test]
    fn the_cell_voltage_takes_the_lower_line() {
        let (fontsize, upper, lower) = text_lines(H);
        let pack = "Bat1 12.60v 3.2 A 85%";
        let with = |on: bool, count: i32| HudInputs {
            display_cell_voltage: on,
            battery_cell_count: count,
            ..flying()
        };
        for (on, count) in [(false, 0), (false, 4), (true, 0)] {
            let drawn = scene(&with(on, count), W, H);
            let facts = readout_facts(&drawn);
            assert_eq!(fact(&facts, "hud.battery.lower"), pack, "{on} {count}");
            assert_eq!(fact(&facts, "hud.battery.upper"), "none");
            let (at, size, _) = label_at(&drawn, pack).expect("drawn");
            assert!(close(at, (fontsize, lower)) && size == fontsize, "{at:?}");
        }
        let drawn = scene(&with(true, 4), W, H);
        let facts = readout_facts(&drawn);
        assert_eq!(fact(&facts, "hud.battery.lower"), "Cell 3.15v");
        assert_eq!(fact(&facts, "hud.battery.upper"), pack);
        let (at, size, colour) = label_at(&drawn, "Cell 3.15v").expect("drawn");
        assert!(close(at, (fontsize, lower)), "{at:?}");
        assert_eq!((size, colour), (fontsize + 2.0, colour::INK));
        let (at, _, _) = label_at(&drawn, pack).expect("drawn");
        assert!(close(at, (fontsize, upper)), "{at:?}");
        for (count, cell) in [(3, "Cell 4.20v"), (-3, "Cell -4.20v"), (6, "Cell 2.10v")] {
            let facts = readout_facts(&scene(&with(true, count), W, H));
            assert_eq!(fact(&facts, "hud.battery.lower"), cell);
        }
        // The alert colours it with the pack.
        let critical = HudInputs {
            battery_critical: true,
            ..with(true, 4)
        };
        let (_, _, colour) = label_at(&scene(&critical, W, H), "Cell 3.15v").expect("drawn");
        assert_eq!(colour, colour::ALERT);
    }

    /// The second battery: its line on the lower line while its voltage is above 0 and no cell
    /// voltage is shown, the first battery's then up a line; its values from `BATTERY_STATUS`
    /// for battery 2. `// C#: ExtLibs/Controls/HUD.cs:2906-2923, GCSViews/FlightData.Designer.cs:359-361`
    #[test]
    fn the_second_battery_takes_the_lower_line() {
        let two = HudInputs {
            battery_voltage2: 11.1,
            battery_current2: 1.25,
            battery_remaining2: 50,
            ..flying()
        };
        let facts = readout_facts(&scene(&two, W, H));
        // 1.25 A is "1.3 A": .NET rounds halves away from zero.
        assert_eq!(fact(&facts, "hud.battery.lower"), "Bat2 11.10v 1.3 A 50%");
        assert_eq!(fact(&facts, "hud.battery.upper"), "Bat1 12.60v 3.2 A 85%");
        let cells = HudInputs {
            display_cell_voltage: true,
            battery_cell_count: 3,
            ..two
        };
        let facts = readout_facts(&scene(&cells, W, H));
        assert_eq!(fact(&facts, "hud.battery.lower"), "Cell 4.20v");
        assert!(
            !scene(&cells, W, H)
                .labels()
                .iter()
                .any(|l| l.starts_with("Bat2"))
        );

        let mut state = VehicleState::default();
        state.batteries[0].voltage = 11.1;
        state.batteries[0].current = 2.0;
        state.batteries[0].remaining_percent = 40;
        let inputs = live(&state, &mut Timing::default(), &[]);
        assert_eq!(
            (
                inputs.battery_voltage2,
                inputs.battery_current2,
                inputs.battery_remaining2
            ),
            (11.1, 2.0, 40)
        );
    }

    /// The second GPS: nothing while it has no GPS; otherwise "GPS2:" on the lower line and the
    /// first up a line. The colour carries from the first to the second, as the C#'s `col` does.
    /// `// C#: ExtLibs/Controls/HUD.cs:2926-3027`
    #[test]
    fn the_second_gps_goes_under_the_first() {
        let (fontsize, upper, lower) = text_lines(H);
        let gps = |fix: u8, fix2: u8| {
            let drawn = scene(
                &HudInputs {
                    gps_fix: fix,
                    gps_fix2: fix2,
                    ..flying()
                },
                W,
                H,
            );
            (fact(&readout_facts(&drawn), "hud.gps"), drawn)
        };
        let (one, drawn) = gps(3, 0);
        assert_eq!(one, "GPS: 3D Fix");
        let (at, _, _) = label_at(&drawn, "GPS: 3D Fix").expect("drawn");
        assert!(close(at, (W - 13.0 * fontsize, lower)), "{at:?}");

        let (two, drawn) = gps(3, 6);
        assert_eq!(two, "GPS: 3D Fix,GPS2: rtk Fixed");
        let (at, _, colour) = label_at(&drawn, "GPS: 3D Fix").expect("drawn");
        assert!(close(at, (W - 13.0 * fontsize, upper)), "{at:?}");
        assert_eq!(colour, colour::INK);
        let (at, _, colour) = label_at(&drawn, "GPS2: rtk Fixed").expect("drawn");
        assert!(close(at, (W - 13.0 * fontsize, lower)), "{at:?}");
        assert_eq!(colour, colour::INK);

        let (_, drawn) = gps(1, 3);
        let (_, _, colour) = label_at(&drawn, "GPS2: 3D Fix").expect("drawn");
        assert_eq!(colour, colour::ALERT, "red carried from the first");
        let (_, drawn) = gps(3, 1);
        let (_, _, colour) = label_at(&drawn, "GPS: 3D Fix").expect("drawn");
        assert_eq!(colour, colour::INK);
        let (_, _, colour) = label_at(&drawn, "GPS2: No Fix").expect("drawn");
        assert_eq!(colour, colour::ALERT);

        let mut state = VehicleState::default();
        state.gps2.fix_type = 5;
        assert_eq!(live(&state, &mut Timing::default(), &[]).gps_fix2, 5);
    }

    /// Show icons: each readout's picture where `doPaint()` puts it - the battery at the left
    /// edge with its numbers beside it, the GPS fixes at the right of the strip, Vibe four and
    /// EKF five pictures in, the pre-arm picture above EKF's while disarmed - and no text for
    /// them. The click rectangles follow the pictures. `// C#: ExtLibs/Controls/HUD.cs:2861-2899,
    /// 2997-3009, 3150-3207, 3211-3301`
    #[test]
    fn show_icons_draws_the_pictures_where_the_csharp_puts_them() {
        let (fontsize, upper, lower) = text_lines(H);
        let fontoffset = (fontsize - 10.0).max(0.0);
        let icons = HudInputs {
            display_icons: true,
            ..flying()
        };
        let drawn = scene(&icons, W, H);
        let rect_of = |scene: &Scene, wanted: Icon| {
            scene.items.iter().find_map(|item| match item {
                Item::Icon { icon, rect } if *icon == wanted => Some(*rect),
                _ => None,
            })
        };
        let same = |a: (f32, f32, f32, f32), b: (f32, f32, f32, f32)| {
            close((a.0, a.1), (b.0, b.1)) && close((a.2, a.3), (b.2, b.3))
        };
        assert_eq!(
            fact(&readout_facts(&drawn), "hud.icons"),
            "batt_4,3dfix_wide,vibe_green,ekf_green"
        );
        let bottomsize = (fontsize + 2.0) * 3.0 + fontoffset - 2.0;
        let battery = rect_of(&drawn, Icon::Batt4).expect("the battery");
        assert!(
            same(battery, (3.0, H - bottomsize, bottomsize / 2.0, bottomsize)),
            "{battery:?}"
        );
        let (at, size, _) = label_at(&drawn, "12.60v 3.2 A 85%").expect("the numbers");
        assert!(close(at, (bottomsize / 2.0 + 6.0, lower)), "{at:?}");
        assert_eq!(size, fontsize + 1.0);
        let wide = (fontsize + 8.0) * 3.0;
        let strip = H - (fontsize + 13.0);
        let gps = rect_of(&drawn, Icon::Fix3d).expect("the fix");
        assert!(
            same(gps, (W - wide - 3.0, strip, wide, fontsize + 8.0)),
            "{gps:?}"
        );
        let vibe = (
            W - wide * 4.0 + wide / 2.0 - 5.0,
            strip,
            wide,
            fontsize + 8.0,
        );
        let ekf = (
            W - wide * 5.0 + wide / 2.0 - 10.0,
            strip,
            wide,
            fontsize + 8.0,
        );
        let picture = |zone: (f32, f32, f32, f32)| (zone.0, zone.1 + 2.0, zone.2, zone.3);
        assert!(same(
            rect_of(&drawn, Icon::VibeGreen).expect("vibe"),
            picture(vibe)
        ));
        assert!(same(
            rect_of(&drawn, Icon::EkfGreen).expect("ekf"),
            picture(ekf)
        ));
        assert!(same(drawn.zone(Element::Vibe).expect("a zone"), vibe));
        assert!(same(drawn.zone(Element::Ekf).expect("a zone"), ekf));
        // No text for what the pictures show.
        for text in ["Vibe", "EKF", "GPS: 3D Fix", "Bat1 12.60v 3.2 A 85%"] {
            assert!(label_at(&drawn, text).is_none(), "{text} drawn as text");
        }
        assert_eq!(fact(&health_facts(&drawn), "hud.vibe.colour"), "none");

        // Disarmed: the pre-arm picture, two wide, above EKF's.
        let disarmed = scene(
            &HudInputs {
                armed: false,
                ..icons.clone()
            },
            W,
            H,
        );
        let prearm = (
            W - wide * 5.0 + wide / 2.0 - 7.0,
            H - (fontsize * 2.0 + 25.0),
            wide * 2.0,
            fontsize + 8.0,
        );
        assert!(same(
            rect_of(&disarmed, Icon::PrearmGreen).expect("ready"),
            picture(prearm)
        ));
        assert!(label_at(&disarmed, "Ready to Arm").is_none());
        let not_ready = scene(
            &HudInputs {
                armed: false,
                prearm_ready: false,
                ..icons.clone()
            },
            W,
            H,
        );
        assert!(rect_of(&not_ready, Icon::PrearmRed).is_some());

        // CPU at the right of Vibe's picture's rectangle.
        let busy = scene(
            &HudInputs {
                cpu_load: 100.0,
                ..icons.clone()
            },
            W,
            H,
        );
        let (at, _, _) = label_at(&busy, "CPU").expect("CPU");
        assert!(close(at, (vibe.0 + wide, vibe.1)), "{at:?}");

        // The cell voltage alone above the numbers.
        let cells = scene(
            &HudInputs {
                display_cell_voltage: true,
                battery_cell_count: 4,
                ..icons.clone()
            },
            W,
            H,
        );
        let (at, size, _) = label_at(&cells, "3.15v").expect("the cell");
        assert!(close(at, (bottomsize / 2.0 + 6.0, upper)), "{at:?}");
        assert_eq!(size, fontsize);

        // Two receivers: the first one picture further in.
        let two = scene(
            &HudInputs {
                gps_fix2: 5,
                ..icons.clone()
            },
            W,
            H,
        );
        assert!(same(
            rect_of(&two, Icon::Fix3d).expect("first"),
            (W - wide * 2.0 - 5.0, strip, wide, fontsize + 8.0)
        ));
        assert!(same(
            rect_of(&two, Icon::RtkFloat).expect("second"),
            (W - wide - 3.0, strip, wide, fontsize + 8.0)
        ));
        // And with the icons off, the zones are at the text, 40 wide and two fonts high.
        let text = scene(&flying(), W, H);
        assert!(same(
            text.zone(Element::Vibe).expect("a zone"),
            (W - 18.0 * fontsize, lower, 40.0, fontsize * 2.0)
        ));
        assert!(same(
            text.zone(Element::Ekf).expect("a zone"),
            (W - 23.0 * fontsize, lower, 40.0, fontsize * 2.0)
        ));
        assert_eq!(fact(&readout_facts(&text), "hud.icons"), "none");
    }

    /// The click rectangles as facts, at the size the facts' scene is drawn (`main.rs`, 800 by
    /// 260): what `tests/gui/hud-icons.gui` expects, text then pictures.
    #[test]
    fn the_zone_facts_are_the_ones_the_icon_script_expects() {
        let text = readout_facts(&scene(&flying(), 800.0, 260.0));
        assert_eq!(fact(&text, "hud.zone.vibe"), "638,239,40,18");
        assert_eq!(fact(&text, "hud.zone.ekf"), "593,239,40,18");
        let icons = HudInputs {
            display_icons: true,
            ..flying()
        };
        let pictures = readout_facts(&scene(&icons, 800.0, 260.0));
        assert_eq!(fact(&pictures, "hud.zone.vibe"), "617,238,51,17");
        assert_eq!(fact(&pictures, "hud.zone.ekf"), "561,238,51,17");
    }

    /// Each readout's picture follows its level as its text's colour does.
    /// `// C#: ExtLibs/Controls/HUD.cs:2861-2885, 2936-2986, 3166-3200, 3226-3259, 3280-3300`
    #[test]
    fn the_pictures_follow_the_levels() {
        let icons_of = |inputs: HudInputs| {
            scene(
                &HudInputs {
                    display_icons: true,
                    ..inputs
                },
                W,
                H,
            )
            .icons()
        };
        for (remaining, low, critical, icon) in [
            (76, false, false, Icon::Batt4),
            (75, false, false, Icon::Batt3),
            (51, false, false, Icon::Batt3),
            (50, false, false, Icon::Batt2),
            (26, false, false, Icon::Batt2),
            (25, false, false, Icon::Batt1),
            (0, false, false, Icon::Batt1),
            (90, true, false, Icon::BattYellow),
            (90, true, true, Icon::BattRed),
        ] {
            let drawn = icons_of(HudInputs {
                battery_remaining: remaining,
                battery_low: low,
                battery_critical: critical,
                ..flying()
            });
            assert_eq!(drawn.first(), Some(&icon), "{remaining} {low} {critical}");
        }
        for (fix, icon) in [
            (0, Icon::NoGps),
            (1, Icon::NoFix),
            (2, Icon::Fix2d),
            (3, Icon::Fix3d),
            (4, Icon::Dgps3d),
            (5, Icon::RtkFloat),
            (6, Icon::RtkFixed),
            (8, Icon::Unknown),
        ] {
            let drawn = icons_of(HudInputs {
                gps_fix: fix,
                ..flying()
            });
            assert_eq!(drawn.get(1), Some(&icon), "fix {fix}");
        }
        for (vibe, icon) in [
            ([30.0, 0.0, 0.0], Icon::VibeGreen),
            ([0.0, 31.0, 0.0], Icon::VibeYellow),
            ([0.0, 0.0, 61.0], Icon::VibeRed),
        ] {
            assert!(icons_of(HudInputs { vibe, ..flying() }).contains(&icon));
        }
        for (ekf_status, icon) in [
            (0.5, Icon::EkfGreen),
            (0.6, Icon::EkfYellow),
            (0.9, Icon::EkfRed),
        ] {
            assert!(
                icons_of(HudInputs {
                    ekf_status,
                    ..flying()
                })
                .contains(&icon)
            );
        }
    }

    /// The stand-ins: a badge is its colour at the bitmaps' alpha with its words in white in
    /// the middle, inside its rectangle; a battery is its outline and cap and as many bars as
    /// its file has, from the bottom. `// C#: ExtLibs/Controls/Resources/*.png`
    #[test]
    fn a_picture_is_drawn_as_what_it_shows() {
        let rect = (100.0, 200.0, 54.0, 18.0);
        let inside = |points: &[(f32, f32)]| {
            points.iter().all(|(x, y)| {
                *x >= rect.0 - 0.01
                    && *x <= rect.0 + rect.2 + 0.01
                    && *y >= rect.1 - 0.01
                    && *y <= rect.1 + rect.3 + 0.01
            })
        };
        let badge = icon_items(Icon::EkfRed, rect);
        match badge.as_slice() {
            [
                Item::Fill {
                    points,
                    colour,
                    alpha,
                },
                Item::Label {
                    text,
                    at,
                    colour: ink,
                    align,
                    size,
                },
            ] => {
                assert!(inside(points) && points.len() == 4, "{points:?}");
                assert_eq!(*colour, colour::ICON_RED);
                assert!((alpha - 179.0 / 255.0).abs() < 1e-6);
                assert_eq!(text, "EKF");
                assert_eq!((*ink, *align), (colour::INK, Align::Centre));
                assert!(close(*at, (127.0, 200.0 + (18.0 - size * 1.2) / 2.0)));
                assert!(*size <= 18.0 * 0.7);
            }
            other => panic!("{other:?}"),
        }
        let bars = |icon: Icon| {
            icon_items(icon, rect)
                .iter()
                .filter(|item| {
                    matches!(item, Item::Fill { colour, .. }
                        if *colour == colour::BATTERY_GREEN || *colour == colour::BATTERY_YELLOW)
                })
                .count()
        };
        assert_eq!(
            [
                Icon::Batt1,
                Icon::Batt2,
                Icon::Batt3,
                Icon::Batt4,
                Icon::BattYellow,
                Icon::BattRed
            ]
            .map(bars),
            [1, 2, 3, 4, 1, 0]
        );
        for icon in [Icon::Batt4, Icon::BattRed] {
            for item in icon_items(icon, rect) {
                match item {
                    Item::Fill { points, .. } | Item::Stroke { points, .. } => {
                        assert!(inside(&points), "{icon:?}: {points:?}");
                    }
                    other => panic!("{other:?}"),
                }
            }
        }
        let outline = |icon: Icon| {
            icon_items(icon, rect).iter().find_map(|item| match item {
                Item::Stroke { colour, .. } => Some(*colour),
                _ => None,
            })
        };
        assert_eq!(outline(Icon::BattRed), Some(colour::BATTERY_RED));
        assert_eq!(outline(Icon::Batt2), Some(colour::BATTERY_OUTLINE));
        // Every picture has words or bars, and a name that is its file's.
        for icon in [Icon::Fix2d, Icon::RtkFixed, Icon::PrearmRed, Icon::Unknown] {
            assert!(!icon_items(icon, rect).is_empty());
        }
        assert_eq!(Icon::Fix2d.name(), "2dfix_wide");
        assert_eq!(Icon::PrearmRed.name(), "prearm_red");
    }

    /// The user's extra fields are read through the getters, so a distance is in feet where the
    /// units say so and a heading is as it was. `// C#: ExtLibs/Controls/HUD.cs:949-968, 3029-3077`
    #[test]
    fn custom_items_are_shown_in_the_users_units() {
        let mut inputs = in_feet_and_knots();
        inputs.custom_items = vec![
            CustomItem {
                header: "Dist: ".to_owned(),
                name: "wp_dist".to_owned(),
                value: Some(250.0),
            },
            CustomItem {
                header: "Yaw: ".to_owned(),
                name: "yaw".to_owned(),
                value: Some(90.5),
            },
        ];
        let drawn = scene(&inputs, W, H);
        let texts: Vec<&str> = drawn
            .labels_of(Element::CustomItems)
            .into_iter()
            .map(|(text, _)| text)
            .collect();
        assert_eq!(texts, ["Dist: 820.21", "Yaw: 90.5"]);
    }
}
