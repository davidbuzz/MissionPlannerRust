//! Rotary (spiral) surveys as Mission Planner lays them out: `Grid.CreateRotary`, transliterated.
//!
//! Replaces `ExtLibs/Utilities/Grid.cs:190-310`. The pattern is the polygon's outline, then the
//! outline inset by `distance`, then by twice that, lap after lap until the inset polygon vanishes
//! or `laps` run out. The insets are ClipperLib's (`ClipperOffset` with mitred joins, on the UTM
//! coordinates in whole millimetres, `Grid.cs:248-257`), ported in [`crate::clipper`]; as the area
//! shrinks, a concave outline splits into several, each flown in turn. Every lap is rotated to
//! start at its vertex nearest where the last one started, and the first `clockwise_laps` are
//! flown the other way round.
//!
//! Like [`crate::grid`] this follows the C# statement for statement. `CreateRotary` also takes a
//! trigger spacing (which it overwrites with 0, `Grid.cs:198`), an angle, overshoots, a shutter
//! flag, a lane separation and a lead-in, and reads none of them; [`RotaryArgs`] has no field for
//! them. Every point is tagged `S` (`Grid.cs:309`).
//!
//! The result is compared bit for bit against `CreateRotary` run under mono, by
//! `tests/rotary_vectors.rs` over `testdata/grid`.

use mp_units::LatLon;

use crate::clipper::{ClipperError, ClipperOffset, IntPoint};
use crate::grid::{
    GridPoint, GridTag, StartPosition, cs_long, find_closest_point, get_poly_min_max,
};
use crate::utm::{UtmPos, to_utm, utm_zone};

/// Why a rotary survey could not be generated: where `CreateRotary` throws.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum RotaryError {
    /// A point could not be converted back from UTM, where ProjNet throws for want of
    /// convergence (`MapProjection.cs:792`). Positions a [`LatLon`] accepts always convert.
    #[error("a survey point could not be converted from UTM")]
    Projection,
    /// The polygon offset failed where ClipperLib throws (`clipper.cs:4924`), or a lap had no
    /// vertex to start from, where `Grid.cs:277` throws. Neither happens for a polygon a
    /// [`LatLon`] can hold.
    #[error("the polygon could not be inset: {0}")]
    Offset(&'static str),
}

impl From<ClipperError> for RotaryError {
    fn from(error: ClipperError) -> Self {
        Self::Offset(error.0)
    }
}

/// `CreateRotary`'s arguments (`Grid.cs:196`) that reach the pattern, named as the C# names them.
///
/// [`Default`] gives what `GridUI` passes on a fresh install (`GridUI.cs:600-605`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotaryArgs {
    /// Altitude given to every point, metres. `NUM_altitude`, default 100.
    pub altitude: f64,
    /// Distance between laps, metres. `NUM_Distance`, default 50; below 0.1 it is 0.1. The inset
    /// is whole millimetres (`Grid.cs:257`).
    pub distance: f64,
    /// Which corner or vertex the first lap starts nearest. `CMB_startfrom`, default Home.
    pub startpos: StartPosition,
    /// The planned home, `MAV.cs.PlannedHomeLocation`, for [`StartPosition::Home`]. It is (0, 0)
    /// until a home is planned (`CurrentState.cs:41`), and `CreateRotary` uses whatever it is
    /// given.
    pub home: LatLon,
    /// `Grid.StartPointLatLngAlt` (`Grid.cs:34`), a static the C# reads for
    /// [`StartPosition::Point`].
    pub start_point: LatLon,
    /// How many laps, from the outermost, are flown in the reverse direction; negative for all
    /// of them. The last of them is closed on its first vertex. `NUM_clockwise_laps`, default 0.
    pub clockwise_laps: i32,
    /// Close the first lap on itself, unless exactly one lap is reversed.
    /// `CHK_match_spiral_perimeter`, unchecked by default.
    pub match_spiral_perimeter: bool,
    /// The most laps to fly. `NUM_laps`, default 200.
    pub laps: i32,
}

impl Default for RotaryArgs {
    fn default() -> Self {
        Self {
            altitude: 100.0,
            distance: 50.0,
            startpos: StartPosition::Home,
            home: LatLon::default(),
            start_point: LatLon::default(),
            clockwise_laps: 0,
            match_spiral_perimeter: false,
            laps: 200,
        }
    }
}

/// `Grid.CreateRotary`, `Grid.cs:196-310`: the laps over `polygon`, in flight order, every point
/// tagged [`GridTag::Start`].
///
/// An empty polygon, one of fewer than three distinct millimetre points, or `laps` of 0 or less
/// gives no points, as the C# does.
///
/// # Errors
///
/// As [`RotaryError`], where the C# throws.
pub fn create_rotary(polygon: &[LatLon], args: &RotaryArgs) -> Result<Vec<GridPoint>, RotaryError> {
    // spacing = 0 (Grid.cs:198): it is not read again.
    let mut distance = args.distance;

    if distance < 0.1 {
        distance = 0.1;
    }

    let Some(first) = polygon.first() else {
        return Ok(Vec::new());
    };

    let mut ans: Vec<UtmPos> = Vec::new();

    // utm zone distance calcs will be done in
    let utmzone = utm_zone(first.latitude(), first.longitude());

    // utm position list: every vertex in the first vertex's zone and hemisphere
    // C#: PointLatLngAlt.cs:294-302
    let utmpositions: Vec<UtmPos> = polygon
        .iter()
        .map(|p| {
            let (x, y) = to_utm(utmzone, first.latitude(), p.latitude(), p.longitude());
            UtmPos::new(x, y, utmzone)
        })
        .collect();

    // get mins/maxs of coverage area
    let area = get_poly_min_max(&utmpositions);

    let maxlane = args.laps; // (Centroid(utmpositions).GetDistance(utmpositions[0]) / distance);

    // pick start positon based on initial point rectangle
    let startposutm = match args.startpos {
        // C#: utmpos.cs:39 - projected in the home's own zone, not the polygon's
        StartPosition::Home => UtmPos::from_lat_lng(args.home.latitude(), args.home.longitude()),
        StartPosition::BottomLeft => UtmPos::new(area.left, area.bottom, utmzone),
        StartPosition::BottomRight => UtmPos::new(area.right, area.bottom, utmzone),
        StartPosition::TopLeft => UtmPos::new(area.left, area.top, utmzone),
        StartPosition::TopRight => UtmPos::new(area.right, area.top, utmzone),
        StartPosition::Point => {
            UtmPos::from_lat_lng(args.start_point.latitude(), args.start_point.longitude())
        }
    };

    // find the closes polygon point based from our startpos selection
    let mut startposutm = find_closest_point(startposutm, &utmpositions);

    let mut clipper_offset = ClipperOffset::new();

    // C#: Grid.cs:250, `new IntPoint(a.x * 1000.0, a.y * 1000.0)`: millimetres, truncated.
    let path: Vec<IntPoint> = utmpositions
        .iter()
        .map(|a| IntPoint::from_f64(a.x * 1000.0, a.y * 1000.0))
        .collect();
    clipper_offset.add_path(&path);

    let mut lane: i32 = 0;
    while lane < maxlane {
        // C#: Grid.cs:257, `(Int64)(distance * 1000.0 * -lane)`, widened back to Execute's double.
        #[allow(clippy::cast_precision_loss)] // (double)long, as the C# converts it
        let delta = cs_long(distance * 1000.0 * f64::from(lane.wrapping_neg())) as f64;
        let mut tree = clipper_offset.execute(delta)?;

        if tree.child_count() == 0 {
            break;
        }

        if lane < args.clockwise_laps || args.clockwise_laps < 0 {
            tree.reverse_paths();
        }

        for contour in tree.child_contours() {
            // C#: Grid.cs:269, `a.X / 1000.0`: the long widened, then divided.
            #[allow(clippy::cast_precision_loss)] // (double)long, as the C# converts it
            let mut ans1: Vec<UtmPos> = contour
                .iter()
                .map(|a| UtmPos::new(a.x as f64 / 1000.0, a.y as f64 / 1000.0, utmzone))
                .collect();
            // rotate points so the start point is close to the previous
            {
                startposutm = find_closest_point(startposutm, &ans1);

                // `ans1.IndexOf(startposutm)`: the first equal by utmpos.Equals, x and y only.
                let Some(startidx) = ans1.iter().position(|p| p.equals(startposutm)) else {
                    return Err(RotaryError::Offset("no vertex nearest the start"));
                };

                // firsthalf = ans1[startidx..], secondhalf = ans1[..startidx]
                ans1.rotate_left(startidx);
            }
            if lane == 0 && args.clockwise_laps != 1 && args.match_spiral_perimeter {
                // start at the last point of the first calculated lap
                // to make a closed polygon on the first trip around
                if let Some(&last) = ans1.last() {
                    ans1.insert(0, last);
                }
            }

            if lane == args.clockwise_laps.wrapping_sub(1) {
                // revisit the first waypoint on this lap to cleanly exit the CW pattern
                if let Some(&first) = ans1.first() {
                    ans1.push(first);
                }
            }

            // The C# then reads the last two points of `ans` and the ends of `ans1` into locals
            // it never uses (Grid.cs:295-302).

            ans.extend(ans1);
        }

        lane += 1;
    }

    // set the altitude on all points
    ans.into_iter()
        .map(|pos| {
            let (lat, lng) = pos.to_lla().ok_or(RotaryError::Projection)?;
            Ok(GridPoint {
                lat,
                lng,
                alt: args.altitude,
                tag: GridTag::Start,
            })
        })
        .collect()
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
        .map(|(lat, lng)| LatLon::new(lat, lng).unwrap())
        .collect()
    }

    #[test]
    fn nothing_in_nothing_out() {
        let args = RotaryArgs::default();
        assert_eq!(create_rotary(&[], &args), Ok(Vec::new()));
        let none = RotaryArgs { laps: 0, ..args };
        assert_eq!(create_rotary(&square(), &none), Ok(Vec::new()));
    }

    #[test]
    fn each_lap_of_a_square_is_four_corners_until_it_vanishes() {
        // About 1000 m by 1000 m, 100 m laps: five laps before the inset passes the middle.
        let args = RotaryArgs {
            distance: 100.0,
            ..RotaryArgs::default()
        };
        let points = create_rotary(&square(), &args).unwrap();
        assert_eq!(points.len(), 5 * 4);
        assert!(
            points
                .iter()
                .all(|p| p.tag == GridTag::Start && p.alt == 100.0)
        );
    }

    #[test]
    fn the_last_reversed_lap_is_closed_and_a_matched_perimeter_starts_where_it_ends() {
        let base = RotaryArgs {
            distance: 100.0,
            laps: 2,
            ..RotaryArgs::default()
        };
        let plain = create_rotary(&square(), &base).unwrap();
        assert_eq!(plain.len(), 8);

        let reversed = create_rotary(
            &square(),
            &RotaryArgs {
                clockwise_laps: 1,
                ..base
            },
        )
        .unwrap();
        assert_eq!(reversed.len(), 9);
        assert_eq!(reversed[0], reversed[4]);

        let matched = create_rotary(
            &square(),
            &RotaryArgs {
                match_spiral_perimeter: true,
                ..base
            },
        )
        .unwrap();
        assert_eq!(matched.len(), 9);
        assert_eq!(matched[0], matched[4]);
    }
}
