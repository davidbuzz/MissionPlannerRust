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

//! The planner map's UTM grid: `chk_grid` ticked, `MainMap_Paint` draws grid lines over the map
//! at zoom 10 and closer. Ported from `GCSViews/FlightPlanner.cs:4809-4903` @ efb0801
//! (GPL-3.0-only).
//!
//! The view's corners go to UTM in the zone of the view's centre (ProjNet, as `ToUTM(zone)`),
//! the grid step is the smallest of 100 km, 10 km, 1 km, 100 m, 10 m and 1 m that puts fewer than
//! forty lines across the view (the C#'s six `if`s in turn, the last true one winning), the corners are rounded down to the step, and each grid line
//! runs between two UTM points brought back with GeoUtility (`FromUTM`) - a straight line on
//! the screen, as the C# draws it. The zone's two bounding meridians are drawn too, twice as
//! wide.
//!
//! The arithmetic only: the planner's `chk_grid` box and the paint over the map are PLAN.md §13.5
//! row 58's, still owed, and so is a run of the C# itself over these views.

use crate::geoutility::utm_to_wgs84;
use crate::utm::{to_utm, utm_zone};

/// A line of the grid, from one corner to the other, in latitude and longitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridLine {
    /// One end.
    pub from: (f64, f64),
    /// The other.
    pub to: (f64, f64),
    /// A zone boundary, drawn with the two-pixel pen.
    pub boundary: bool,
}

/// `PointLatLngAlt.GetLngStartFromZone` and `GetLngEndFromZone`: the zone's western and eastern
/// meridians, the zone's sign dropped.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:200-212`
fn zone_meridians(zone: i32) -> (f64, f64) {
    let zone = zone.abs();
    let start = f64::from(zone - 1) * 6.0 - 180.0;
    (start, start + 6.0)
}

/// `PointLatLngAlt.FromUTM(zone, x, y)`: GeoUtility's inverse, the hemisphere from the zone's
/// sign.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:246-252`
fn from_utm(zone: i32, x: f64, y: f64) -> (f64, f64) {
    utm_to_wgs84(zone.abs(), zone < 0, x, y)
}

/// The grid for a view from its top-left corner to its bottom-right, at `zoom`: nothing under
/// zoom 10.
/// `// C#: GCSViews/FlightPlanner.cs:4811-4903`
#[must_use]
pub fn grid_lines(top_left: (f64, f64), bottom_right: (f64, f64), zoom: f64) -> Vec<GridLine> {
    if zoom < 10.0 {
        return Vec::new();
    }
    let (plla1, plla2) = (top_left, bottom_right);
    let centre = (
        f64::midpoint(plla1.0, plla2.0),
        f64::midpoint(plla1.1, plla2.1),
    );
    // `center.GetUTMZone()`: negative in the south.
    let zone = utm_zone(centre.0, centre.1);
    let (lngstart, lngend) = zone_meridians(zone);
    // `plla.ToUTM(zone)`: in the centre's zone, the hemisphere the point's own.
    let utm1 = to_utm(zone, plla1.0, plla1.0, plla1.1);
    let utm2 = to_utm(zone, plla2.0, plla2.0, plla2.1);
    let deltax = utm1.0 - utm2.0;
    let mut gridsize = 1000.0;
    for step in [100_000.0, 10_000.0, 1000.0, 100.0, 10.0, 1.0] {
        if deltax.abs() / step < 40.0 {
            gridsize = step;
        }
    }
    // `utm1[0] - utm1[0] % gridsize`, `utm2[1] - utm2[1] % gridsize`.
    let x0 = utm1.0 - utm1.0 % gridsize;
    let y0 = utm2.1 - utm2.1 % gridsize;
    let mut lines = Vec::new();
    let mut x = x0;
    while x < utm2.0 {
        lines.push(GridLine {
            from: from_utm(zone, x, utm1.1),
            to: from_utm(zone, x, utm2.1),
            boundary: false,
        });
        x += gridsize;
    }
    let mut y = y0;
    while y < utm1.1 {
        lines.push(GridLine {
            from: from_utm(zone, utm1.0, y),
            to: from_utm(zone, utm2.0, y),
            boundary: false,
        });
        y += gridsize;
    }
    for meridian in [lngstart, lngend] {
        lines.push(GridLine {
            from: (plla1.0, meridian),
            to: (plla2.0, meridian),
            boundary: true,
        });
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_under_zoom_ten() {
        assert!(grid_lines((-35.36, 149.16), (-35.37, 149.17), 9.9).is_empty());
    }

    #[test]
    fn a_kilometre_wide_view_gets_hundred_metre_lines_and_the_zone_meridians() {
        // About 1.1 km across at this latitude: 100 m lines, a dozen of them each way.
        let lines = grid_lines((-35.360, 149.160), (-35.370, 149.172), 15.0);
        let verticals: Vec<&GridLine> = lines.iter().filter(|l| !l.boundary).collect();
        assert!(
            verticals.len() > 15 && verticals.len() < 40,
            "{}",
            verticals.len()
        );
        let boundaries: Vec<&GridLine> = lines.iter().filter(|l| l.boundary).collect();
        assert_eq!(boundaries.len(), 2);
        // Zone 55: 144 E to 150 E.
        assert_eq!(boundaries[0].from.1, 144.0);
        assert_eq!(boundaries[1].from.1, 150.0);
        // A vertical grid line's ends sit on the same easting: near-identical longitude across
        // a one-degree-of-latitude-free view.
        let first = verticals[0];
        assert!((first.from.1 - first.to.1).abs() < 1e-3);
    }
}
