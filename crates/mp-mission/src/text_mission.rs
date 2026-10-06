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

//! Text: the planner's `textToolStripMenuItem_Click`, ported from `GCSViews/FlightPlanner.cs:
//! 6839-6883` @ efb0801 (GPL-3.0-only): a string, a size and a rotation are asked for, the
//! string is drawn into a `GraphicsPath` with `AddString` in the `1CamBam_Stick_3` font at
//! `size * 1.35`, the path is rotated, and **every point of the path** - GDI+ keeps a TrueType
//! outline as lines and cubic Béziers, so the Bézier control points are points too - is added
//! as a waypoint at `utmpos(MouseDownStart) + (x, -y)` metres.
//!
//! This module is the geometry after the font: [`path_points`] turns an outline's drawing
//! commands into the points GDI+ would list, and [`place`] rotates them and lays them on the
//! ground. Reading the font is the screen's business (`crates/mp-gui/src/glyph_text.rs`).

use crate::utm::UtmPos;

/// One drawing command of a glyph outline, in the path's units (y down, as GDI+ has it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Segment {
    /// A figure starts here.
    MoveTo(f64, f64),
    /// A straight edge to here.
    LineTo(f64, f64),
    /// A quadratic Bézier through the control point to the end, as TrueType glyphs have them.
    QuadTo {
        /// The control point.
        control: (f64, f64),
        /// The end.
        end: (f64, f64),
    },
    /// A cubic Bézier, as CFF glyphs have them.
    CurveTo {
        /// The first control point.
        control1: (f64, f64),
        /// The second.
        control2: (f64, f64),
        /// The end.
        end: (f64, f64),
    },
    /// The figure closes; GDI+ marks the last point rather than adding one.
    Close,
}

/// `GraphicsPath.PathPoints` for the outline: the start of each figure, the end of each line,
/// and for each curve its two cubic control points then its end - a TrueType quadratic raised to
/// a cubic as GDI+ raises it, its control points a third and two thirds of the way. A close adds
/// nothing.
#[must_use]
pub fn path_points(segments: &[Segment]) -> Vec<(f64, f64)> {
    let mut points = Vec::new();
    let mut current = (0.0, 0.0);
    for segment in segments {
        match *segment {
            Segment::MoveTo(x, y) | Segment::LineTo(x, y) => {
                points.push((x, y));
                current = (x, y);
            }
            Segment::QuadTo { control, end } => {
                let c1 = (
                    current.0 + 2.0 / 3.0 * (control.0 - current.0),
                    current.1 + 2.0 / 3.0 * (control.1 - current.1),
                );
                let c2 = (
                    end.0 + 2.0 / 3.0 * (control.0 - end.0),
                    end.1 + 2.0 / 3.0 * (control.1 - end.1),
                );
                points.push(c1);
                points.push(c2);
                points.push(end);
                current = end;
            }
            Segment::CurveTo {
                control1,
                control2,
                end,
            } => {
                points.push(control1);
                points.push(control2);
                points.push(end);
                current = end;
            }
            Segment::Close => {}
        }
    }
    points
}

/// `Matrix.Rotate(rotation)` applied to the path, then each point laid at `utmpos(centre) + (x,
/// -y)` and brought back to latitude and longitude: the positions of the waypoints, in the path's
/// order. GDI+'s rotation is about the origin, clockwise on the screen for a positive angle
/// (`x' = x cos - y sin`, `y' = x sin + y cos` with y down); the path's y is then negated, so
/// text reads upright with north up.
/// `// C#: GCSViews/FlightPlanner.cs:6854-6868`
#[must_use]
pub fn place(
    points: &[(f64, f64)],
    rotation_degrees: f64,
    centre_lat: f64,
    centre_lng: f64,
) -> Vec<(f64, f64)> {
    let (sin, cos) = rotation_degrees.to_radians().sin_cos();
    let base = UtmPos::from_lat_lng(centre_lat, centre_lng);
    points
        .iter()
        .filter_map(|&(x, y)| {
            let (rx, ry) = (x * cos - y * sin, x * sin + y * cos);
            UtmPos::new(base.x + rx, base.y - ry, base.zone).to_lla()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_quadratic_is_listed_as_a_cubics_three_points() {
        let segments = [
            Segment::MoveTo(0.0, 0.0),
            Segment::LineTo(3.0, 0.0),
            Segment::QuadTo {
                control: (3.0, 3.0),
                end: (0.0, 3.0),
            },
            Segment::Close,
        ];
        let points = path_points(&segments);
        assert_eq!(points.len(), 5);
        assert_eq!(points[0], (0.0, 0.0));
        assert_eq!(points[1], (3.0, 0.0));
        // Two thirds of the way from each end towards the control point.
        assert_eq!(points[2], (3.0, 2.0));
        assert_eq!(points[3], (2.0, 3.0));
        assert_eq!(points[4], (0.0, 3.0));
        let cubic = [
            Segment::MoveTo(1.0, 1.0),
            Segment::CurveTo {
                control1: (2.0, 1.0),
                control2: (2.0, 2.0),
                end: (1.0, 2.0),
            },
        ];
        assert_eq!(
            path_points(&cubic),
            vec![(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)]
        );
    }

    #[test]
    fn points_are_laid_east_and_north_of_the_click_and_turn_with_the_rotation() {
        let centre = (-35.363, 149.165);
        // 100 m along x is 100 m east; `newpos.y += -Y`, so 100 m down the path is 100 m south.
        // Grid east is a degree or so off true east here (UTM 55's convergence), so a few metres
        // of latitude come with the hundred of longitude.
        let east = place(&[(100.0, 0.0)], 0.0, centre.0, centre.1)[0];
        assert!(
            east.1 > centre.1 && (east.0 - centre.0).abs() < 5e-5,
            "{east:?}"
        );
        let south = place(&[(0.0, 100.0)], 0.0, centre.0, centre.1)[0];
        assert!(
            south.0 < centre.0 && (south.1 - centre.1).abs() < 5e-5,
            "{south:?}"
        );
        // Rotated 90 degrees, the eastward point goes to the path's +y: south.
        let turned = place(&[(100.0, 0.0)], 90.0, centre.0, centre.1)[0];
        assert!((turned.0 - south.0).abs() < 1e-9 && (turned.1 - south.1).abs() < 1e-9);
        // The distance is kept: 100 m is about 0.0011 degrees of longitude at this latitude.
        assert!(
            ((east.1 - centre.1) * 111_320.0 * centre.0.to_radians().cos() - 100.0).abs() < 1.0
        );
    }
}
