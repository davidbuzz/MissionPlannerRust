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

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::time::{Duration, Instant};

use gpui::{
    Bounds, Hsla, PathBuilder, Pixels, Point, SharedString, TextAlign, TextRun, Window, point, px,
    rgb,
};
use mp_vehicle::VehicleState;

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
    /// A low battery: the C#'s orange brush.
    pub const WARN: u32 = 0xff_a5_00;
    /// The VSI's climb polygon: `Brushes.Blue`.
    pub const VSI: u32 = 0x3b_7d_ff;
    /// Ground under the altitude tape: `AltGroundBrush`, burlywood at alpha 100.
    pub const GROUND_TAPE: u32 = 0xde_b8_87;
    /// The ground-course mark and the scroller arrows: black.
    pub const BLACK: u32 = 0x00_00_00;
    /// The heading tape's centre box: `SlightlyTransparentWhiteBrush`.
    pub const READOUT: u32 = 0xff_ff_ff;
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
    /// The vibration indicator.
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
    /// Drawn by [`scene`].
    Drawn,
    /// Not yet. The coverage test lists these; PLAN.md §13.3 item 4 tracks them.
    Missing,
}

/// The coverage table: every element `doPaint()` draws.
pub const ELEMENTS: &[(Element, Status)] = &[
    (Element::SkyGround, Status::Drawn),
    (Element::PitchLadder, Status::Drawn),
    (Element::RollIndicator, Status::Drawn),
    (Element::Reticle, Status::Drawn),
    (Element::FlightPathVector, Status::Missing),
    (Element::HeadingTape, Status::Drawn),
    (Element::HeadingBugs, Status::Drawn),
    (Element::XtrackBar, Status::Drawn),
    (Element::RateOfTurn, Status::Drawn),
    (Element::SpeedTape, Status::Drawn),
    (Element::AltitudeTape, Status::Drawn),
    (Element::Vsi, Status::Drawn),
    (Element::ModeAndWaypoint, Status::Drawn),
    (Element::LinkInfo, Status::Drawn),
    (Element::Aoa, Status::Missing),
    (Element::Battery, Status::Drawn),
    (Element::Gps, Status::Drawn),
    (Element::CustomItems, Status::Missing),
    (Element::ArmedBanner, Status::Drawn),
    (Element::Failsafe, Status::Drawn),
    (Element::Message, Status::Drawn),
    (Element::Vibe, Status::Missing),
    (Element::Ekf, Status::Missing),
    (Element::Prearm, Status::Missing),
];

/// The missing elements with their C# lines, one string, for the facts and the plan.
#[must_use]
pub fn missing_report() -> String {
    missing()
        .iter()
        .map(|element| {
            let (from, to) = element.csharp_lines();
            format!("{} (HUD.cs:{from}-{to})", element.name())
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The elements the table says are not drawn.
#[must_use]
pub fn missing() -> Vec<Element> {
    ELEMENTS
        .iter()
        .filter(|(_, status)| *status == Status::Missing)
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

/// Everything the display draws from, in the units the C# draws in.
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
        }
    }
}

impl HudInputs {
    /// The display's inputs from a vehicle's state, as `FlightData` binds them to `hud1`.
    ///
    /// `// C#: GCSViews/FlightData.Designer.cs:352-391` for the bindings, and
    /// `ExtLibs/ArduPilot/CurrentState.cs` for the derived ones: `turnrate` (1203), `targetalt`
    /// (1107), `targetairspeed` (1130), `failsafe` from `MAV_STATE_CRITICAL` (2895), the safety
    /// message when armed with motor control disabled (2887), `linkqualitygcs` (4595).
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // display precision
    pub fn from_vehicle(
        state: &VehicleState,
        mode: String,
        armed_for: Option<Duration>,
        mode_changed_for: Option<Duration>,
        clock: String,
        message: Option<(String, u32)>,
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
        }
    }
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
}

/// A frame of the display, ready to paint.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Scene {
    /// What to draw, in order.
    pub items: Vec<Item>,
    /// Which elements were drawn.
    pub drawn: Vec<Element>,
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

    fn drew(&mut self, element: Element) {
        if !self.drawn.contains(&element) {
            self.drawn.push(element);
        }
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

    // Sky and ground, then the horizon geometry everything attitude-relative hangs off.
    let geometry = horizon_geometry(
        inputs.roll.to_radians(),
        inputs.pitch.to_radians(),
        centre,
        w,
        h,
    );
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
        let rung = offset(centre, up, (a - inputs.pitch) * ppd);
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
            offset(rung, along, -half),
            offset(rung, along, half),
            if degrees == 0 { 2.0 } else { 1.5 },
            ink,
        );
        if degrees % 10 == 0 {
            let at = offset(rung, along, -half - 30.0 - fontoffset * 1.7);
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
    let pointer = (-inputs.roll).to_radians();
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

    // The aircraft symbol: the one thing on the display that does not move. C#: HUD.cs:2209-2231
    let wing = w * 0.13;
    let (cx, cy) = centre;
    scene.line((cx - wing, cy), (cx - wing * 0.35, cy), 3.0, colour::ALERT);
    scene.line((cx + wing * 0.35, cy), (cx + wing, cy), 3.0, colour::ALERT);
    scene.line((cx, cy - 4.0), (cx, cy + 4.0), 3.0, colour::ALERT);
    scene.drew(Element::Reticle);

    if !inputs.has_vehicle {
        return scene;
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
    let speed = if inputs.airspeed == 0.0 {
        inputs.ground_speed
    } else {
        inputs.airspeed
    };
    scroller(
        &mut scene,
        &left_box,
        Side::Left,
        speed,
        inputs.target_speed,
        None,
        unit_space,
        viewrange,
        fontsize,
        fontoffset,
        halfheight,
    );
    scene.label(
        format!("{speed:.0}m/s"),
        (0.0, halfheight - 9.0),
        10.0,
        colour::INK,
        Align::Left,
    );
    scene.label(
        format!("AS {:.1}m/s", inputs.airspeed),
        (1.0, left_box.bottom() + 5.0),
        fontsize,
        colour::INK,
        Align::Left,
    );
    scene.label(
        format!("GS {:.1}m/s", inputs.ground_speed),
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
    scroller(
        &mut scene,
        &right_box,
        Side::Right,
        inputs.altitude,
        inputs.target_altitude,
        (inputs.ground_altitude != 0.0).then_some(inputs.ground_altitude),
        unit_space,
        viewrange,
        fontsize,
        fontoffset,
        halfheight,
    );
    scene.label(
        format!("{:.0} m", inputs.altitude),
        (right_box.left + 10.0, halfheight - 9.0),
        10.0,
        colour::INK,
        Align::Left,
    );
    scene.drew(Element::AltitudeTape);

    // Vertical speed beside it: a tapered box, a blue bar from the middle for the climb rate,
    // clamped at ±6 m/s. C#: HUD.cs:2660-2711
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
    let vs = inputs.vertical_speed.clamp(-6.0, 6.0);
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
    let distance = if inputs.wp_distance >= 1000.0 {
        format!("{:.1}k", inputs.wp_distance / 1000.0)
    } else {
        format!("{}m", inputs.wp_distance.trunc())
    };
    scene.label(
        format!("{distance}>{}", inputs.wp_number),
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

    // The text lines along the bottom. C#: HUD.cs:2846-2854
    let y_bot_offset = if fontsize >= 8.0 { fontsize / 3.0 } else { 2.0 };
    let y_text_offset = fontsize + y_bot_offset + 2.0;
    let x_pos = fontsize;
    let y_lower = h - y_text_offset - y_bot_offset - 4.0;

    // Battery: voltage, current and remaining, coloured by the alert level. C#: HUD.cs:2855-2925
    let battery_colour = if inputs.battery_critical {
        colour::ALERT
    } else if inputs.battery_low {
        colour::WARN
    } else {
        colour::INK
    };
    scene.label(
        format!(
            "Bat1 {:.2}v {:.1} A {}%",
            inputs.battery_voltage, inputs.battery_current, inputs.battery_remaining
        ),
        (x_pos, y_lower),
        fontsize,
        battery_colour,
        Align::Left,
    );
    scene.drew(Element::Battery);

    // GPS fix, in red when there is none. C#: HUD.cs:2926-2986
    let gps_colour = if inputs.gps_fix <= 1 {
        colour::ALERT
    } else {
        colour::INK
    };
    scene.label(
        gps_fix_text(inputs.gps_fix),
        (w - 13.0 * fontsize, y_lower),
        fontsize,
        gps_colour,
        Align::Left,
    );
    scene.drew(Element::Gps);

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
        }
    }

    /// The coverage table is held to the code: every element it calls drawn is in the scene,
    /// and nothing the scene draws is listed as missing.
    #[test]
    fn the_coverage_table_matches_what_the_scene_draws() {
        let scene = scene(&flying(), W, H);
        for (element, status) in ELEMENTS {
            let drawn = scene.drawn.contains(element);
            match status {
                Status::Drawn => {
                    assert!(drawn, "{} is listed as drawn and was not", element.name())
                }
                Status::Missing => {
                    assert!(!drawn, "{} is drawn; update ELEMENTS", element.name());
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
        // The list of what is missing, printed so it is read.
        let missing = missing();
        eprintln!(
            "HUD elements not yet drawn ({}): {}",
            missing.len(),
            missing
                .iter()
                .map(|e| format!(
                    "{} (HUD.cs:{}-{})",
                    e.name(),
                    e.csharp_lines().0,
                    e.csharp_lines().1
                ))
                .collect::<Vec<_>>()
                .join(", ")
        );
        assert_eq!(missing.len(), 6);
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
}
