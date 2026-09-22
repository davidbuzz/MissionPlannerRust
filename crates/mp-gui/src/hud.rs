//! Primary flight display (DELIVERABLES.md D9).
//!
//! Replaces `Controls/HUD.cs` (3,856 LOC of GDI+ drawing) with GPU-composited paths and quads.
//!
//! # Conventions, stated because getting them wrong is invisible in code review
//!
//! * Roll is positive with the right wing down. On screen the horizon then tilts so its **right
//!   end rises**, because the display shows the world as seen from the aircraft, not the aircraft
//!   as seen from the world. An artificial horizon that rolls the wrong way looks plausible in a
//!   screenshot and is lethal in cloud.
//! * Pitch is positive nose-up, and the horizon moves **down** the screen as the nose rises.
//! * Screen y grows downward, so every "up" in this file is negative y.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{Bounds, Hsla, PathBuilder, Pixels, Point, Window, point, px, quad, rgb, size};
use mp_vehicle::VehicleState;

/// Screen pixels per degree of pitch. Sets how much of the pitch ladder is visible at once;
/// Mission Planner's HUD is comparable at a default window size.
const PIXELS_PER_DEGREE: f32 = 5.5;

/// Pitch ladder rungs, in degrees above and below the horizon.
const LADDER_STEPS: &[i32] = &[-30, -20, -10, 10, 20, 30];

mod colour {
    /// Sky above the horizon.
    pub const SKY: u32 = 0x2f_6d_9e;
    /// Ground below it.
    pub const GROUND: u32 = 0x6b_4f_2a;
    /// Ladder, reticle and scales.
    pub const INK: u32 = 0xff_ff_ff;
    /// Roll pointer and warnings.
    pub const ALERT: u32 = 0xf8_51_49;
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
    // Positive roll is right-wing-down. The display shows the world from the aircraft, so the
    // horizon rotates the opposite way: its right end rises, which is negative y on screen.
    let along = (roll.cos(), -roll.sin());
    // Rotate `along` a quarter turn toward the sky. At zero roll this is (0, -1), straight up.
    let up = (along.1, -along.0);

    // Nose up moves the horizon down the screen.
    let pitch_offset = pitch.to_degrees() * PIXELS_PER_DEGREE;
    let anchor = (centre.0, centre.1 + pitch_offset);
    let reach = w.hypot(h);

    HorizonGeometry {
        left: offset(anchor, along, -reach),
        right: offset(anchor, along, reach),
        along,
        up,
    }
}

/// Paints the primary flight display.
pub fn paint_hud(state: Option<&VehicleState>, bounds: Bounds<Pixels>, window: &mut Window) {
    let origin = bounds.origin;
    let w = f32::from(bounds.size.width);
    let h = f32::from(bounds.size.height);
    let centre = point(origin.x + px(w / 2.0), origin.y + px(h / 2.0));

    #[allow(clippy::cast_possible_truncation)]
    let (roll, pitch) = state.map_or((0.0_f32, 0.0_f32), |s| {
        (s.attitude.roll.0 as f32, s.attitude.pitch.0 as f32)
    });

    let geometry = horizon_geometry(
        roll,
        pitch,
        (f32::from(centre.x), f32::from(centre.y)),
        w,
        h,
    );
    let (along, up) = (geometry.along, geometry.up);
    let horizon = (
        (geometry.left.0 + geometry.right.0) / 2.0,
        (geometry.left.1 + geometry.right.1) / 2.0,
    );
    // Long enough that a rotated half-plane still covers the corners at any roll angle.
    let reach = w.hypot(h);

    paint_half_plane(window, horizon, along, up, reach, colour::SKY);
    paint_half_plane(
        window,
        horizon,
        along,
        (-up.0, -up.1),
        reach,
        colour::GROUND,
    );

    stroke(window, &[geometry.left, geometry.right], 2.0, colour::INK);

    // Pitch ladder. Rungs are parallel to the horizon and shorten away from it, which is how a
    // pilot reads magnitude at a glance without labels.
    for degrees in LADDER_STEPS {
        let distance = *degrees as f32 * PIXELS_PER_DEGREE;
        let rung_centre = offset(horizon, up, distance);
        let half = if degrees.abs() >= 20 {
            w * 0.10
        } else {
            w * 0.16
        };
        stroke(
            window,
            &[
                offset(rung_centre, along, -half),
                offset(rung_centre, along, half),
            ],
            1.5,
            colour::INK,
        );
    }

    // Fixed aircraft reticle: the one thing on the display that does not move.
    let wing = w * 0.13;
    let cx = f32::from(centre.x);
    let cy = f32::from(centre.y);
    stroke(
        window,
        &[(cx - wing, cy), (cx - wing * 0.35, cy)],
        3.0,
        colour::ALERT,
    );
    stroke(
        window,
        &[(cx + wing * 0.35, cy), (cx + wing, cy)],
        3.0,
        colour::ALERT,
    );
    stroke(
        window,
        &[(cx, cy - 4.0), (cx, cy + 4.0)],
        3.0,
        colour::ALERT,
    );

    // Roll scale: fixed ticks around the top, with a pointer that rolls with the aircraft.
    let radius = (w.min(h) * 0.42).max(10.0);
    for tick in [
        -60.0_f32, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0,
    ] {
        let a = tick.to_radians();
        let outer = (cx + a.sin() * radius, cy - a.cos() * radius);
        let len = if (tick.abs() % 30.0) < 0.01 {
            12.0
        } else {
            7.0
        };
        let inner = (cx + a.sin() * (radius - len), cy - a.cos() * (radius - len));
        stroke(window, &[outer, inner], 1.5, colour::INK);
    }
    let a = (-roll).to_radians();
    let tip = (
        cx + a.sin() * (radius - 14.0),
        cy - a.cos() * (radius - 14.0),
    );
    let base_l = (
        cx + (a - 0.06).sin() * (radius - 28.0),
        cy - (a - 0.06).cos() * (radius - 28.0),
    );
    let base_r = (
        cx + (a + 0.06).sin() * (radius - 28.0),
        cy - (a + 0.06).cos() * (radius - 28.0),
    );
    fill(window, &[tip, base_l, base_r], colour::ALERT);

    // Side scales, drawn as plain backing quads; the numbers are overlaid as text by the caller,
    // because shaping text inside a canvas is not worth it for four readouts.
    let scale_w = w * 0.16;
    window.paint_quad(quad(
        Bounds {
            origin,
            size: size(px(scale_w), px(h)),
        },
        gpui::Corners::default(),
        Hsla {
            a: 0.28,
            ..Hsla::black()
        },
        gpui::Edges::default(),
        rgb(0x00_00_00),
        gpui::BorderStyle::default(),
    ));
    window.paint_quad(quad(
        Bounds {
            origin: point(origin.x + px(w - scale_w), origin.y),
            size: size(px(scale_w), px(h)),
        },
        gpui::Corners::default(),
        Hsla {
            a: 0.28,
            ..Hsla::black()
        },
        gpui::Edges::default(),
        rgb(0x00_00_00),
        gpui::BorderStyle::default(),
    ));
}

/// A half-plane bounded by the horizon, extending `reach` in every direction.
fn paint_half_plane(
    window: &mut Window,
    origin: (f32, f32),
    along: (f32, f32),
    toward: (f32, f32),
    reach: f32,
    colour: u32,
) {
    let a = offset(origin, along, -reach);
    let b = offset(origin, along, reach);
    let c = offset(b, toward, reach);
    let d = offset(a, toward, reach);
    fill(window, &[a, b, c, d], colour);
}

fn offset(from: (f32, f32), direction: (f32, f32), distance: f32) -> (f32, f32) {
    (
        direction.0.mul_add(distance, from.0),
        direction.1.mul_add(distance, from.1),
    )
}

fn to_screen(p: (f32, f32)) -> Point<Pixels> {
    point(px(p.0), px(p.1))
}

/// Strokes a polyline.
fn stroke(window: &mut Window, points: &[(f32, f32)], width: f32, colour: u32) {
    if points.len() < 2 {
        return;
    }
    let mut builder = PathBuilder::stroke(px(width));
    let mut iter = points.iter();
    if let Some(first) = iter.next() {
        builder.move_to(to_screen(*first));
    }
    for p in iter {
        builder.line_to(to_screen(*p));
    }
    // A failed tessellation must not silently erase part of the display: ADR 0001 records how a
    // swallowed `Err` made a whole flight track vanish.
    match builder.build() {
        Ok(path) => window.paint_path(path, Hsla::from(rgb(colour))),
        Err(err) => {
            debug_assert!(false, "HUD stroke failed to tessellate: {err:?}");
        }
    }
}

/// Fills a closed polygon.
fn fill(window: &mut Window, points: &[(f32, f32)], colour: u32) {
    if points.len() < 3 {
        return;
    }
    let mut builder = PathBuilder::fill();
    let mut iter = points.iter();
    if let Some(first) = iter.next() {
        builder.move_to(to_screen(*first));
    }
    for p in iter {
        builder.line_to(to_screen(*p));
    }
    if let Some(first) = points.first() {
        builder.line_to(to_screen(*first));
    }
    match builder.build() {
        Ok(path) => window.paint_path(path, Hsla::from(rgb(colour))),
        Err(err) => {
            debug_assert!(false, "HUD fill failed to tessellate: {err:?}");
        }
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

        // And the displacement should match the configured scale.
        let expected = 10.0 * PIXELS_PER_DEGREE;
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
}
