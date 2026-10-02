//! The proximity window: `Controls/ProximityControl.cs`, which SETUP's Advanced page's
//! "Proximity" opens (`ConfigAdvanced.but_proximity_Click`,
//! `GCSViews/ConfigurationView/ConfigAdvanced.cs:42-45`: `new ProximityControl(MainV2.comPort.MAV)
//! .Show()`).
//!
//! What it shows, as `Temp_Paint` draws it in a 430 x 391 form (`ProximityControl.cs:91-208,
//! 233-245`; the `.resx` holds only the timer's tray place), centred in the client:
//!
//! * the quad picture (`Resources.quadicon`) `mavsize` centimetres across, for a copter
//!   (`cs.firmware == ArduCopter2`) only;
//! * for a vehicle that has proximity data (`MAVState.Proximity`, made for every vehicle heard -
//!   [`mp_vehicle::proximity`]), a dim grey circle every half metre out to `screenradius`, each
//!   labelled in green "0.5m", "1.0m" at its top; eight dim grey spokes every 45 degrees, each
//!   labelled with its angle; and each reading not yet expired: a `DISTANCE_SENSOR` facing one
//!   of the eight yaw orientations as a red 45-degree arc at its distance, labelled with it, and
//!   an `OBSTACLE_DISTANCE` angle as a yellow arc as wide as its increment, unlabelled. Nothing
//!   is drawn past `screenradius`, the arcs of readings beyond it drawn on it;
//! * in the caption, `"Radius(+/-): 5m MAV size([/]): 0.8m"`.
//!
//! What it does: `+` and `-` take 50 cm off or put 50 cm on `screenradius`, `]` and `[` a
//! centimetre on or off `mavsize`, neither going below its floor (`Temp_KeyPress`, `:56-85`); the
//! picture is drawn again every 100 ms (`timer1`, `:40-43`) and on a resize; closing stops the
//! timer (`:46-49`).
//!
//! Where this is not the C#, each written at its site:
//!
//! * the form is drawn over SETUP, modal, where the C#'s is a free form; a second click replaces
//!   it with a fresh one, as the MAVLink Inspector's is (`config/mavlink_inspector.rs`). It has
//!   the keyboard focus when it opens, as a new form is activated, and again when clicked;
//! * the form does not resize; its scale is worked from its size as drawn here - the client and
//!   the caption row above it - where the C# takes the outer `Height` and `Width` the window
//!   manager gives it;
//! * it is drawn every frame the application draws, which is at least as often as the C#'s
//!   timer, and the readings are expired against the vehicle's clock in a log being played back -
//!   the readings carry the log's time there ([`mp_vehicle::proximity`]) - and against the wall
//!   clock otherwise, as the C# does;
//! * `quadicon` is drawn through `pictures.rs`, which carries the C#'s images - `Resources/quad2.png`
//!   under [`QUAD_ICON`] - and the fact `config.proximity.quad` says where;
//! * an arc of no radius - a reading of the near end of an `OBSTACLE_DISTANCE` range that is 0 -
//!   is not drawn, where GDI+ is handed a rectangle of no size;
//! * the colours on the application's dark ground: the background is the window's, where the
//!   C#'s is `SystemColors.Control`; the pens and brushes are the C#'s.
//!
//! The vehicle is the one shown when the form opens, `MainV2.comPort.MAV`, held by the form for
//! good; with none shown the form has no proximity data and draws the caption alone.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::Arc;

use gpui::{
    AnyElement, Bounds, Context, FocusHandle, Hsla, KeyDownEvent, PathBuilder, Pixels,
    SharedString, TextAlign, TextRun, Window, canvas, div, point, prelude::*, px, rgb, size,
};
use mp_mission::dotnet::{format_f32, format_f64, general_f64};
use mp_vehicle::proximity::{ROTATION_CUSTOM, Reading};
use mp_vehicle::{DateTime, VehicleId, VehicleState};

use crate::MissionPlanner;
use crate::ui::theme;

/// `ClientSize`. `// C#: Controls/ProximityControl.cs:241`
pub const CLIENT: (f32, f32) = (430.0, 391.0);

/// The caption row above the client, as the application draws a form's caption.
pub const CAPTION_HEIGHT: f32 = 26.0;

/// `public float screenradius = 500`: the radius drawn, centimetres.
/// `// C#: Controls/ProximityControl.cs:88`
pub const SCREEN_RADIUS: f32 = 500.0;

/// `public float mavsize = 80`: the vehicle's picture across, centimetres.
/// `// C#: Controls/ProximityControl.cs:89`
pub const MAV_SIZE: f32 = 80.0;

/// `Resources.quadicon`, as `pictures.rs` carries it.
/// `// C#: Controls/ProximityControl.cs:111`
pub const QUAD_ICON: &str = "quadicon";

/// `float move = 5`: how far a label sits off its point.
/// `// C#: Controls/ProximityControl.cs:122`
const MOVE: f32 = 5.0;

/// `Pens.DimGray`, the circles and spokes, and `Brushes.DimGray`, the spokes' labels.
const DIM_GRAY: u32 = 0x69_69_69;
/// `Brushes.Green`: the circles' and readings' labels.
const GREEN: u32 = 0x00_80_00;
/// `new Pen(Color.Red, 3)`: a `DISTANCE_SENSOR`'s arc.
const RED: u32 = 0xff_00_00;
/// `new Pen(Color.Yellow, 3)`: an `OBSTACLE_DISTANCE` angle's arc.
const YELLOW: u32 = 0xff_ff_00;
/// The two pens' width. `// C#: Controls/ProximityControl.cs:118-119`
const PEN: f32 = 3.0;

/// `new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 2,
/// FontStyle.Bold)`: Microsoft Sans Serif 8.25 pt and two, bold, in pixels at 96 dpi.
/// `// C#: Controls/ProximityControl.cs:120`
const FONT_PX: f32 = (8.25 + 2.0) * 96.0 / 72.0;

/// `HALF_SQRT_2`, `0.70710678118654757`: the double `FRAC_1_SQRT_2` is.
/// `// C#: ExtLibs/Utilities/Vector3.cs:285-288`
const HALF_SQRT_2: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// The form's state: `screenradius` and `mavsize`, and the vehicle it was made for.
#[derive(Debug, Clone, PartialEq)]
pub struct Proximity {
    /// `screenradius`, centimetres.
    pub screenradius: f32,
    /// `mavsize`, centimetres.
    pub mavsize: f32,
    /// `_parent`: the vehicle shown when the form opened, once the next frame has looked; `None`
    /// inside for none shown.
    pub vehicle: Option<Option<VehicleId>>,
    /// That vehicle's state, as of this frame.
    pub state: Option<Arc<VehicleState>>,
    /// Whether it is a copter: `cs.firmware == Firmwares.ArduCopter2`.
    pub copter: bool,
    /// The time the readings are expired against this frame.
    pub now: DateTime,
}

impl Default for Proximity {
    fn default() -> Self {
        Self {
            screenradius: SCREEN_RADIUS,
            mavsize: MAV_SIZE,
            vehicle: None,
            state: None,
            copter: false,
            now: DateTime::MIN,
        }
    }
}

impl Proximity {
    /// `Temp_KeyPress`: `+` closer in, `-` further out, `]` a bigger picture, `[` a smaller one;
    /// then `screenradius` under 1 back to 50 and `mavsize` under 1 to 1. True for the four keys,
    /// `e.Handled`. Every key redraws.
    /// `// C#: Controls/ProximityControl.cs:56-85`
    pub fn key_press(&mut self, key: char) -> bool {
        let handled = match key {
            '+' => {
                self.screenradius -= 50.0;
                true
            }
            '-' => {
                self.screenradius += 50.0;
                true
            }
            ']' => {
                self.mavsize += 1.0;
                true
            }
            '[' => {
                self.mavsize -= 1.0;
                true
            }
            _ => false,
        };
        // "prevent 0's"
        if self.screenradius < 1.0 {
            self.screenradius = 50.0;
        }
        if self.mavsize < 1.0 {
            self.mavsize = 1.0;
        }
        handled
    }

    /// The caption `Temp_Paint` sets: `"Radius(+/-): " + (screenradius / 100.0) + "m MAV
    /// size([/]): " + (mavsize / 100.0) + "m"`, each a double as .NET writes one.
    /// `// C#: Controls/ProximityControl.cs:100`
    #[must_use]
    pub fn caption(&self) -> String {
        format!(
            "Radius(+/-): {}m MAV size([/]): {}m",
            general_f64(f64::from(self.screenradius) / 100.0),
            general_f64(f64::from(self.mavsize) / 100.0)
        )
    }

    /// The readings drawn: none without a vehicle (`_dS == null`), else `GetRaw()` at `now`.
    #[must_use]
    pub fn readings(&self) -> Option<Vec<Reading>> {
        let state = self.state.as_ref()?;
        Some(state.proximity.raw(self.now).copied().collect())
    }

    /// Everything `Temp_Paint` draws, in client coordinates.
    #[must_use]
    pub fn picture(&self) -> Picture {
        picture(self, CLIENT, self.copter, self.readings().as_deref())
    }
}

/// A circle or an arc: GDI+'s `DrawArc` of the square round `centre` `radius` across each way,
/// from `start` degrees clockwise from three o'clock through `sweep`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DrawnArc {
    /// The centre.
    pub centre: (f32, f32),
    /// The radius.
    pub radius: f32,
    /// Where it starts, degrees clockwise from three o'clock.
    pub start: f32,
    /// How far it goes, degrees.
    pub sweep: f32,
    /// The pen's colour.
    pub colour: u32,
    /// The pen's width.
    pub width: f32,
}

/// A `DrawString`: its text, its top left corner and its brush.
#[derive(Debug, Clone, PartialEq)]
pub struct Text {
    /// The words.
    pub text: String,
    /// The top left corner.
    pub at: (f32, f32),
    /// The brush.
    pub colour: u32,
}

/// What `Temp_Paint` draws.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Picture {
    /// The quad picture's box, `x, y, size`, for a copter.
    pub quad: Option<(f32, f32, f32)>,
    /// The circles, the spokes' lines - `from`, `to` - and the readings' arcs.
    pub arcs: Vec<DrawnArc>,
    /// The spokes: from the centre to the outer circle.
    pub spokes: Vec<((f32, f32), (f32, f32))>,
    /// The circles' labels, then the spokes', then the readings'.
    pub texts: Vec<Text>,
    /// How many readings were drawn.
    pub readings: usize,
}

/// `new Vector3(0, length, 0).rotate(Rotation.ROTATION_YAW_*)` for an orientation of the eight,
/// or `None` for any other.
/// `// C#: ExtLibs/Utilities/Vector3.cs:302-357`
fn rotated(orientation: u8, length: f64) -> Option<(f64, f64)> {
    let (x, y) = (0.0_f64, length);
    Some(match orientation {
        0 => (x, y),
        1 => (HALF_SQRT_2 * (x - y), HALF_SQRT_2 * (x + y)),
        2 => (-y, x),
        3 => (-HALF_SQRT_2 * (x + y), HALF_SQRT_2 * (x - y)),
        4 => (-x, -y),
        5 => (HALF_SQRT_2 * (y - x), -HALF_SQRT_2 * (x + y)),
        6 => (y, -x),
        7 => (HALF_SQRT_2 * (x + y), HALF_SQRT_2 * (y - x)),
        _ => return None,
    })
}

/// `new Vector3(0, length, 0).rotate(degrees)`: turned `degrees` about z.
/// `// C#: ExtLibs/Utilities/Vector3.cs:290-300`
fn turned(degrees: f64, length: f64) -> (f64, f64) {
    let rotation = degrees * (std::f64::consts::PI / 180.0);
    let (x, y) = (0.0_f64, length);
    (
        x * rotation.cos() - y * rotation.sin(),
        x * rotation.sin() + y * rotation.cos(),
    )
}

/// `Temp_Paint`'s drawing for a client of `client` size: the quad for a copter, and - with
/// `readings`, `_dS` not null - the circles, the spokes and the readings.
/// `// C#: Controls/ProximityControl.cs:91-208`
#[must_use]
#[allow(clippy::cast_possible_truncation)] // the C#'s `(float)` casts of its doubles
pub fn picture(
    form: &Proximity,
    client: (f32, f32),
    copter: bool,
    readings: Option<&[Reading]>,
) -> Picture {
    let mut picture = Picture::default();
    let midx = client.0 / 2.0;
    let midy = client.1 / 2.0;
    // "11m radius = 22 m coverage": `Math.Min(Height, Width)` of the form as drawn here.
    let (width, height) = (client.0, client.1 + CAPTION_HEIGHT);
    let scale = ((form.screenradius + 50.0) * 2.0) / height.min(width);
    // "80cm quad / scale"
    let size = form.mavsize / scale;
    if copter {
        let imw = size / 2.0;
        picture.quad = Some((midx - imw, midy - imw, size));
    }
    let Some(readings) = readings else {
        return picture;
    };
    // The circles, every 50 cm out to `screenradius`.
    let mut x = 50.0_f32;
    while x <= form.screenradius {
        let length = f64::from(x / scale);
        picture.arcs.push(DrawnArc {
            centre: (midx, midy),
            radius: length as f32,
            start: 0.0,
            sweep: 360.0,
            colour: DIM_GRAY,
            width: 1.0,
        });
        picture.texts.push(Text {
            text: format_f32(x / 100.0, "0.0m"),
            at: (midx + MOVE, midy - length as f32),
            colour: GREEN,
        });
        x += 50.0;
    }
    // The spokes, every 45 degrees.
    let mut x = 0.0_f32;
    while x < 360.0 {
        let (lx, ly) = turned(f64::from(x), f64::from(form.screenradius / scale));
        let (lx, ly) = (lx as f32, ly as f32);
        picture.texts.push(Text {
            text: format_f32(x, "0"),
            at: (midx - lx - MOVE * 2.0, midy - ly + MOVE),
            colour: DIM_GRAY,
        });
        picture.spokes.push(((midx, midy), (midx - lx, midy - ly)));
        x += 45.0;
    }
    // The readings.
    for reading in readings {
        let length = (f64::from(reading.distance) / f64::from(scale))
            .min(f64::from(form.screenradius) / f64::from(scale));
        let arc = |start: f32, sweep: f32, colour: u32| DrawnArc {
            centre: (midx, midy),
            radius: length as f32,
            start,
            sweep,
            colour,
            width: PEN,
        };
        if reading.orientation == ROTATION_CUSTOM {
            // `location.rotate(temp.Angle)`, unlabelled: `DrawString` is commented out.
            picture.arcs.push(arc(
                reading.angle - reading.size / 2.0 - 90.0,
                reading.size,
                YELLOW,
            ));
            picture.readings += 1;
            continue;
        }
        let Some((lx, ly)) = rotated(reading.orientation, length) else {
            continue;
        };
        let (lx, ly) = (lx as f32, ly as f32);
        // Each orientation's label offset, as its case writes it.
        let (dx, dy) = match reading.orientation {
            0 => (-MOVE * 2.0, MOVE),
            1 => (-MOVE * 8.0, MOVE),
            2 => (-MOVE * 8.0, 0.0),
            3 => (-MOVE * 8.0, -MOVE),
            4 => (-MOVE * 2.0, -MOVE * 3.0),
            5 => (MOVE, -MOVE),
            _ => (MOVE, 0.0),
        };
        picture.texts.push(Text {
            text: format_f64(f64::from(reading.distance) / 100.0, "0.0m"),
            at: (midx - lx + dx, midy - ly + dy),
            colour: GREEN,
        });
        // `-22.5f - 90f` and on by 45 for each orientation.
        let start = -22.5 + 45.0 * f32::from(reading.orientation) - 90.0;
        picture.arcs.push(arc(start, 45.0, RED));
        picture.readings += 1;
    }
    picture
}

/// The window and how often it has been opened, held with the Advanced page.
#[derive(Debug, Default)]
pub struct ProximityWindow {
    /// The form, while it is open.
    pub window: Option<Proximity>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl ProximityWindow {
    /// `new ProximityControl(MainV2.comPort.MAV).Show()`: a fresh form, the vehicle the one shown
    /// at the next frame, its timer started.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:42-45; Controls/ProximityControl.cs:27-44`
    pub fn show(&mut self) {
        self.opened += 1;
        self.window = Some(Proximity::default());
    }

    /// The close box: `ProximityControl_FormClosing`, the timer stopped.
    /// `// C#: Controls/ProximityControl.cs:46-49`
    pub fn close(&mut self) {
        self.window = None;
    }

    /// Once a frame: the vehicle fixed at the first, its state and kind read, and the clock the
    /// readings are expired against - the vehicle's in a log being played back, now otherwise.
    pub fn tick(
        &mut self,
        shown: Option<VehicleId>,
        state_of: impl Fn(VehicleId) -> Option<Arc<VehicleState>>,
        copter_of: impl Fn(&VehicleState) -> bool,
        replaying: bool,
    ) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let vehicle = *form.vehicle.get_or_insert(shown);
        form.state = vehicle.and_then(state_of);
        form.copter = form.state.as_deref().is_some_and(copter_of);
        form.now = match form.state.as_deref() {
            Some(state) if replaying => state.datetime,
            _ => DateTime::now(),
        };
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(holder: &ProximityWindow) {
    use crate::facts::record;
    record("config.proximity.window", holder.window.is_some());
    record("config.proximity.opened", holder.opened);
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record("config.proximity.caption", form.caption());
    record("config.proximity.screenradius", form.screenradius);
    record("config.proximity.mavsize", form.mavsize);
    record(
        "config.proximity.vehicle",
        form.vehicle.flatten().map_or_else(
            || "none".to_owned(),
            |id| format!("{}.{}", id.sysid, id.compid),
        ),
    );
    let picture = form.picture();
    record(
        "config.proximity.quad",
        picture.quad.map_or_else(
            || "none".to_owned(),
            |(x, y, side)| format!("{x:.1},{y:.1},{side:.1}"),
        ),
    );
    let circles: Vec<String> = picture
        .arcs
        .iter()
        .filter(|arc| arc.sweep >= 360.0)
        .map(|arc| format!("{:.1}", arc.radius))
        .collect();
    record("config.proximity.circles", circles.join(","));
    let labels = |colour: u32| {
        picture
            .texts
            .iter()
            .filter(|text| text.colour == colour)
            .map(|text| text.text.as_str())
            .collect::<Vec<_>>()
            .join(",")
    };
    record("config.proximity.spokes", labels(DIM_GRAY));
    record("config.proximity.labels", labels(GREEN));
    record("config.proximity.readings", picture.readings);
    let arcs: Vec<String> = picture
        .arcs
        .iter()
        .filter(|arc| arc.sweep < 360.0)
        .map(|arc| format!("{:.1}@{:.1}+{:.1}", arc.radius, arc.start, arc.sweep))
        .collect();
    record("config.proximity.arcs", arcs.join(","));
}

/// The form's state from the application.
fn access(this: &mut MissionPlanner) -> Option<&mut Proximity> {
    this.extra.proximity.window.as_mut()
}

/// Points round an arc, GDI+'s way: degrees clockwise from three o'clock, y down.
fn arc_points(arc: &DrawnArc) -> Vec<(f32, f32)> {
    let steps = (arc.sweep.abs() / 3.0).ceil().max(2.0);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a few hundred at most
    let count = steps as usize;
    (0..=count)
        .map(|index| {
            #[allow(clippy::cast_precision_loss)]
            let degrees = arc.start + arc.sweep * index as f32 / steps;
            let radians = degrees.to_radians();
            (
                arc.centre.0 + arc.radius * radians.cos(),
                arc.centre.1 + arc.radius * radians.sin(),
            )
        })
        .collect()
}

/// A line of `points` from `origin`.
fn stroke(points: &[(f32, f32)], colour: u32, width: f32, origin: (f32, f32), window: &mut Window) {
    let mut builder = PathBuilder::stroke(px(width));
    let mut first = true;
    for (x, y) in points {
        let at = point(px(origin.0 + x), px(origin.1 + y));
        if first {
            builder.move_to(at);
            first = false;
        } else {
            builder.line_to(at);
        }
    }
    if let Ok(path) = builder.build() {
        window.paint_path(path, Hsla::from(rgb(colour)));
    }
}

/// The picture painted into `bounds`.
fn paint(picture: &Picture, bounds: Bounds<Pixels>, window: &mut Window, cx: &mut gpui::App) {
    let origin = (f32::from(bounds.origin.x), f32::from(bounds.origin.y));
    window.paint_quad(gpui::fill(bounds, rgb(theme::BG)));
    if let Some((x, y, side)) = picture.quad {
        let target = Bounds {
            origin: point(px(origin.0 + x), px(origin.1 + y)),
            size: size(px(side), px(side)),
        };
        crate::pictures::paint_stretched(QUAD_ICON, bounds, target, window);
    }
    for arc in &picture.arcs {
        if arc.radius > 0.0 {
            stroke(&arc_points(arc), arc.colour, arc.width, origin, window);
        }
    }
    for (from, to) in &picture.spokes {
        stroke(&[*from, *to], DIM_GRAY, 1.0, origin, window);
    }
    let mut font = window.text_style().font();
    font.weight = gpui::FontWeight::BOLD;
    for text in &picture.texts {
        let run = TextRun {
            len: text.text.len(),
            font: font.clone(),
            color: Hsla::from(rgb(text.colour)),
            background_color: None,
            underline: None,
            strikethrough: None,
        };
        let line = window.text_system().shape_line(
            SharedString::from(text.text.clone()),
            px(FONT_PX),
            &[run],
            None,
        );
        let _ = line.paint(
            point(px(origin.0 + text.at.0), px(origin.1 + text.at.1)),
            px(FONT_PX * 1.2),
            TextAlign::Left,
            None,
            window,
            cx,
        );
    }
}

/// The form over the window: its caption and close box, and the picture, which takes the keys.
pub fn overlay(
    holder: &ProximityWindow,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let picture = form.picture();
    let viewport = window.viewport_size();
    let focus_on_click = focus.clone();
    let client = crate::probe::measured("proximity-client", div())
        .id("proximity-client")
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .track_focus(focus)
        .on_click(cx.listener(move |_this, _event, window, cx| {
            focus_on_click.focus(window, cx);
        }))
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            let typed = event
                .keystroke
                .key_char
                .as_deref()
                .and_then(|text| text.chars().next());
            if let (Some(form), Some(key)) = (access(this), typed) {
                // `Invalidate()` whatever the key.
                form.key_press(key);
                cx.notify();
            }
        }))
        .child(
            canvas(
                |_bounds, _window, _cx| (),
                move |bounds, (), window, cx| paint(&picture, bounds, window, cx),
            )
            .size_full(),
        );
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .h(px(CAPTION_HEIGHT))
        .px_2()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(
            crate::probe::measured("proximity-caption", div())
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(form.caption()),
        )
        .child(crate::ui::action(
            "proximity-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.proximity.close();
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("proximity", div())
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
            gpui::anchored().position(point(px(0.0), px(0.0))).child(
                div()
                    .id("proximity-backdrop")
                    .w(viewport.width)
                    .h(viewport.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(body),
            ),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_coverage::source::csharp;
    use mp_mavlink_dialects::all::{DistanceSensor, MavMessage, ObstacleDistance};

    /// A clock for the readings.
    fn at(seconds: f64) -> DateTime {
        #[allow(clippy::cast_possible_truncation)]
        DateTime::from_ticks(638_000_000_000_000_000 + (seconds * 1e7) as i64)
    }

    fn sensor(orientation: u8, distance: u16) -> MavMessage {
        MavMessage::DistanceSensor(DistanceSensor {
            time_boot_ms: 0,
            min_distance: 20,
            max_distance: 4000,
            current_distance: distance,
            r#type: 0,
            id: 0,
            orientation,
            covariance: 0,
            horizontal_fov: 0.0,
            vertical_fov: 0.0,
            quaternion: [0.0; 4],
            signal_quality: 0,
        })
    }

    /// A vehicle's state with `messages` applied at `at(0)`.
    fn vehicle(messages: &[MavMessage]) -> Arc<VehicleState> {
        let mut state = VehicleState::new(1, 1);
        state.datetime = at(0.0);
        for message in messages {
            state.apply(message);
        }
        Arc::new(state)
    }

    /// A form showing `state` at `now`.
    fn form(state: Option<Arc<VehicleState>>, now: f64) -> Proximity {
        Proximity {
            vehicle: Some(state.as_ref().map(|_| VehicleId::new(1, 1))),
            state,
            copter: true,
            now: at(now),
            ..Proximity::default()
        }
    }

    fn close(a: (f32, f32), b: (f32, f32)) -> bool {
        (a.0 - b.0).abs() < 0.01 && (a.1 - b.1).abs() < 0.01
    }

    /// The keys, the floors, and the caption as `Temp_Paint` writes it.
    /// `// C#: Controls/ProximityControl.cs:56-85, 100`
    #[test]
    fn the_keys_move_the_radius_and_the_size() {
        let mut form = Proximity::default();
        assert_eq!(form.caption(), "Radius(+/-): 5m MAV size([/]): 0.8m");
        assert!(form.key_press('+'));
        assert_eq!(form.caption(), "Radius(+/-): 4.5m MAV size([/]): 0.8m");
        assert!(form.key_press('-'));
        assert!(form.key_press('-'));
        assert!(form.key_press(']'));
        assert_eq!(form.caption(), "Radius(+/-): 5.5m MAV size([/]): 0.81m");
        assert!(!form.key_press('x'));
        for _ in 0..20 {
            form.key_press('+');
        }
        assert_eq!(form.screenradius, 50.0, "under 1 back to 50");
        for _ in 0..200 {
            form.key_press('[');
        }
        assert_eq!(form.mavsize, 1.0);
    }

    /// With no vehicle the form has no proximity data: the caption, and the quad only.
    /// `// C#: Controls/ProximityControl.cs:107-116`
    #[test]
    fn with_no_vehicle_only_the_quad_is_drawn() {
        let empty = picture(&Proximity::default(), CLIENT, true, None);
        assert!(empty.arcs.is_empty() && empty.texts.is_empty() && empty.spokes.is_empty());
        // scale = (500 + 50) * 2 / min(417, 430); 80 cm across.
        let scale = 1100.0 / 417.0;
        let (x, y, side) = empty.quad.unwrap();
        assert!((side - 80.0 / scale).abs() < 1e-4);
        assert!(close((x + side / 2.0, y + side / 2.0), (215.0, 195.5)));
        assert_eq!(
            picture(&Proximity::default(), CLIENT, false, None).quad,
            None
        );
    }

    /// The circles every half metre to the radius, each labelled at its top; the spokes every
    /// 45 degrees, the angle written beside each end - 0 at the top, 90 at the right.
    /// `// C#: Controls/ProximityControl.cs:124-146`
    #[test]
    fn the_circles_and_spokes_are_where_the_csharp_draws_them() {
        let drawn = picture(&Proximity::default(), CLIENT, false, Some(&[]));
        let scale = 1100.0_f32 / 417.0;
        let circles: Vec<f32> = drawn.arcs.iter().map(|arc| arc.radius).collect();
        assert_eq!(circles.len(), 10);
        assert!((circles[0] - 50.0 / scale).abs() < 1e-4);
        assert!((circles[9] - 500.0 / scale).abs() < 1e-4);
        let labels: Vec<&str> = drawn.texts.iter().map(|text| text.text.as_str()).collect();
        assert_eq!(
            labels,
            [
                "0.5m", "1.0m", "1.5m", "2.0m", "2.5m", "3.0m", "3.5m", "4.0m", "4.5m", "5.0m",
                "0", "45", "90", "135", "180", "225", "270", "315"
            ]
        );
        assert!(close(drawn.texts[0].at, (220.0, 195.5 - 50.0 / scale)));
        let r = 500.0 / scale;
        assert!(close(drawn.spokes[0].1, (215.0, 195.5 - r)), "0 at the top");
        assert!(
            close(drawn.spokes[2].1, (215.0 + r, 195.5)),
            "90 at the right"
        );
        assert!(
            close(drawn.spokes[4].1, (215.0, 195.5 + r)),
            "180 at the bottom"
        );
        assert!(close(drawn.texts[12].at, (215.0 + r - 10.0, 195.5 + 5.0)));
        assert_eq!(drawn.readings, 0);
    }

    /// Scripted sensors through the vehicle state: a `DISTANCE_SENSOR` facing each of the eight
    /// ways is a red 45-degree arc at its distance centred on its direction - front at the top,
    /// clockwise - labelled where its case puts the label; one further out than the radius is
    /// drawn on it; a downward one is held but not drawn; an `OBSTACLE_DISTANCE` angle is a
    /// yellow arc as wide as its increment, with no label; and three seconds on they are gone.
    /// `// C#: Controls/ProximityControl.cs:148-207; ExtLibs/ArduPilot/Proximity.cs:48-106`
    #[test]
    fn the_readings_are_drawn_from_the_sensors_messages() {
        let mut obstacle = ObstacleDistance {
            time_usec: 0,
            distances: [u16::MAX; 72],
            min_distance: 10,
            max_distance: 2000,
            sensor_type: 9,
            increment: 10,
            increment_f: 0.0,
            angle_offset: 0.0,
            frame: 12,
        };
        obstacle.distances[3] = 200;
        let state = vehicle(&[
            sensor(0, 100),
            sensor(2, 300),
            sensor(4, 3000),
            sensor(25, 150),
            MavMessage::ObstacleDistance(obstacle),
        ]);
        assert_eq!(state.proximity.held().len(), 5);
        let shown = form(Some(state), 0.1);
        let drawn = shown.picture();
        assert_eq!(drawn.readings, 4, "the downward one held, not drawn");
        let scale = 1100.0_f32 / 417.0;
        let arcs: Vec<&DrawnArc> = drawn.arcs.iter().filter(|arc| arc.sweep < 360.0).collect();
        // Front, 1 m: centred on the top.
        assert!((arcs[0].radius - 100.0 / scale).abs() < 1e-4);
        assert_eq!(
            (arcs[0].start, arcs[0].sweep, arcs[0].colour),
            (-112.5, 45.0, RED)
        );
        // Right, 3 m.
        assert_eq!(arcs[1].start, -22.5);
        // Behind, 30 m: on the 5 m circle.
        assert!((arcs[2].radius - 500.0 / scale).abs() < 1e-4);
        assert_eq!(arcs[2].start, 67.5);
        // The obstacle at 30 degrees, 2 m, 10 degrees wide.
        assert_eq!(
            (arcs[3].start, arcs[3].sweep, arcs[3].colour),
            (-65.0, 10.0, YELLOW)
        );
        assert_eq!(arcs[3].width, PEN);
        let labels: Vec<(&str, (f32, f32))> = drawn
            .texts
            .iter()
            .skip(18)
            .map(|text| (text.text.as_str(), text.at))
            .collect();
        assert_eq!(labels.len(), 3, "the obstacle has none");
        assert_eq!(labels[0].0, "1.0m");
        assert!(close(
            labels[0].1,
            (215.0 - 10.0, 195.5 - 100.0 / scale + 5.0)
        ));
        assert_eq!(labels[1].0, "3.0m");
        assert!(close(labels[1].1, (215.0 + 300.0 / scale - 40.0, 195.5)));
        assert_eq!(labels[2].0, "30.0m");
        assert!(close(
            labels[2].1,
            (215.0 - 10.0, 195.5 + 500.0 / scale - 15.0)
        ));
        // The obstacle's fifth of a second is over, the sensors' three seconds not yet.
        assert_eq!(
            form(Some(vehicle(&[sensor(0, 100)])), 3.0)
                .picture()
                .readings,
            1
        );
        assert_eq!(
            form(Some(vehicle(&[sensor(0, 100)])), 3.001)
                .picture()
                .readings,
            0
        );
        let mut later = shown;
        later.now = at(0.3);
        assert_eq!(later.picture().readings, 3);
    }

    /// The window: opened fresh by each click, its vehicle the one shown at the first frame
    /// after and kept, a copter's quad drawn, and closed by its box.
    #[test]
    fn the_window_holds_the_vehicle_it_opened_on() {
        let mut holder = ProximityWindow::default();
        holder.show();
        assert_eq!(holder.opened, 1);
        let state = vehicle(&[sensor(0, 100)]);
        let shown = VehicleId::new(1, 1);
        holder.tick(Some(shown), |_| Some(Arc::clone(&state)), |_| true, true);
        let form = holder.window.as_ref().unwrap();
        assert_eq!(form.vehicle, Some(Some(shown)));
        assert!(form.copter);
        assert_eq!(form.now, at(0.0), "a replay's clock");
        // Another vehicle shown later: the form keeps its own.
        holder.tick(
            Some(VehicleId::new(2, 1)),
            |_| Some(Arc::clone(&state)),
            |_| false,
            false,
        );
        let form = holder.window.as_ref().unwrap();
        assert_eq!(form.vehicle, Some(Some(shown)));
        assert!(!form.copter);
        assert!(form.now > at(0.0), "the wall clock");
        holder.window.as_mut().unwrap().key_press('+');
        holder.show();
        assert_eq!(holder.window.as_ref().unwrap().screenradius, SCREEN_RADIUS);
        // Opened with none shown: none for good.
        holder.tick(None, |_| Some(Arc::clone(&state)), |_| true, false);
        holder.tick(Some(shown), |_| Some(Arc::clone(&state)), |_| true, false);
        assert!(holder.window.as_ref().unwrap().readings().is_none());
        holder.close();
        assert!(holder.window.is_none());
    }

    /// The numbers here are the C#'s: the form's size, the radius and size, the timer, and the
    /// four keys.
    #[test]
    fn the_numbers_are_the_csharps() {
        let Some(source) = csharp("Controls/ProximityControl.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        assert!(source.contains("this.ClientSize = new System.Drawing.Size(430, 391);"));
        assert!(source.contains("public float screenradius = 500;"));
        assert!(source.contains("public float mavsize = 80;"));
        assert!(source.contains("timer1.Interval = 100;"));
        assert!(source.contains("float move = 5;"));
        for key in ["'+'", "'-'", "']'", "'['"] {
            assert!(source.contains(&format!("case {key}:")), "{key}");
        }
        assert!(source.contains("Resources.quadicon"));
        let vector = csharp("ExtLibs/Utilities/Vector3.cs").unwrap_or_default();
        assert!(vector.contains("get { return 0.70710678118654757; }"));
        assert_eq!("0.70710678118654757".parse::<f64>(), Ok(HALF_SQRT_2));
    }

    /// Every fact the GUI script asserts on is recorded here, and every control it clicks is
    /// drawn here or on the Advanced page.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-proximity.gui");
        let source = include_str!("proximity.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.proximity.") => {
                    assert!(
                        source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("proximity-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id} is not drawn");
                }
                (Some("click"), Some(id)) if id.starts_with("advanced-") => {
                    assert_eq!(id, "advanced-but_proximity");
                }
                _ => {}
            }
        }
        assert!(facts >= 10, "{facts} facts");
    }
}
