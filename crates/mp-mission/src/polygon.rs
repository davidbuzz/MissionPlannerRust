//! Polygon > Offset Polygon: the drawn polygon grown or shrunk by a distance.
//!
//! Ported from `GCSViews/FlightPlanner.cs` @ efb0801 (GPL-3.0-or-later),
//! `offsetPolygonToolStripMenuItem_Click` (3639-3686), from the parse of its answer on. The corners
//! go into the first corner's UTM zone, to millimetres, through ClipperLib's mitred offset, and
//! back - with the same `utmpos`, `PointLatLngAlt.ToUTM` and `ClipperOffset` the survey grid uses
//! (`utm.rs`, `clipper.rs`), each held to the C# bit for bit by its own goldens.
//! `testdata/planner/golden/offset.csv` is the handler's lines run under mono
//! (`tools/csharp-reference/PlannerOracle.cs`).

use mp_units::LatLon;

use crate::clipper::{ClipperOffset, IntPoint};
use crate::grid::cs_long;
use crate::utm::{UtmPos, to_utm, utm_zone};

/// The polygon `metres` further out, or in where it is negative: every outline ClipperLib's
/// offset leaves, one after another, as `redrawPolygonSurvey` is handed them.
///
/// `None` where the C# returns with the polygon as it was: no corners (it returns before asking),
/// an offset that leaves nothing (`tree.ChildCount == 0`), or where it would throw - ClipperLib's
/// range check, or a point ProjNet cannot bring back.
/// `// C#: GCSViews/FlightPlanner.cs:3654-3685`
#[must_use]
pub fn offset_polygon(vertices: &[LatLon], metres: f64) -> Option<Vec<LatLon>> {
    let first = vertices.first()?;

    // utm zone distance calcs will be done in
    let utmzone = utm_zone(first.latitude(), first.longitude());

    // utm position list: `ToUTM(utmzone, list)` projects every corner in the first's hemisphere.
    // C#: ExtLibs/Utilities/PointLatLngAlt.cs:294-302
    let mut utmpositions: Vec<UtmPos> = vertices
        .iter()
        .map(|p| {
            let (x, y) = to_utm(utmzone, first.latitude(), p.latitude(), p.longitude());
            UtmPos::new(x, y, utmzone)
        })
        .collect();

    // close the loop if its not already: `utmpos !=`, position and zone.
    if let (Some(&head), Some(&tail)) = (utmpositions.first(), utmpositions.last())
        && !head.op_eq(tail)
    {
        utmpositions.push(head); // make a full loop
    }

    let mut clipper_offset = ClipperOffset::new();
    // `new ClipperLib.IntPoint(a.x * 1000.0, a.y * 1000.0)`: millimetres, truncated.
    let path: Vec<IntPoint> = utmpositions
        .iter()
        .map(|a| IntPoint::from_f64(a.x * 1000.0, a.y * 1000.0))
        .collect();
    clipper_offset.add_path(&path);

    // `Execute(ref tree, (Int64)(intmeter * 1000.0))`: truncated to a long, widened back to the
    // double Execute takes.
    #[allow(clippy::cast_precision_loss)] // (double)long, as the C# converts it
    let delta = cs_long(metres * 1000.0) as f64;
    let tree = clipper_offset.execute(delta).ok()?;

    if tree.child_count() == 0 {
        return None;
    }

    let mut ans = Vec::new();
    for contour in tree.child_contours() {
        for a in contour {
            // `new utmpos(a.X / 1000.0, a.Y / 1000.0, utmzone).ToLLA()`
            #[allow(clippy::cast_precision_loss)] // (double)long, as the C# converts it
            let (lat, lng) =
                UtmPos::new(a.x as f64 / 1000.0, a.y as f64 / 1000.0, utmzone).to_lla()?;
            ans.push(LatLon::new(lat, lng).ok()?);
        }
    }
    Some(ans)
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN: &str = include_str!("../../../testdata/planner/golden/offset.csv");

    fn number(field: &str) -> f64 {
        field.parse().expect("a number in the golden")
    }

    /// Every case of `offset.csv`: the same corners in the same order, each the same doubles, or
    /// nothing where the C# returns with the polygon as it was.
    #[test]
    fn an_offset_is_the_c_sharps_to_the_bit() {
        let mut cases = 0;
        for block in GOLDEN.split("case,").skip(1) {
            cases += 1;
            let mut lines = block.lines();
            let header: Vec<&str> = lines.next().expect("a header").split(',').collect();
            let (name, metres) = (header[0], number(header[1]));
            let mut vertices = Vec::new();
            let mut corners = Vec::new();
            let mut empty = false;
            for line in lines {
                let fields: Vec<&str> = line.split(',').collect();
                match fields[0] {
                    "vertex" => vertices.push(
                        LatLon::new(number(fields[1]), number(fields[2])).expect("a position"),
                    ),
                    "corner" => corners.push((number(fields[1]), number(fields[2]))),
                    "empty" => empty = true,
                    other => panic!("{other} in {name}"),
                }
            }
            let got = offset_polygon(&vertices, metres);
            if empty {
                assert_eq!(got, None, "{name}");
                continue;
            }
            let got = got.expect(name);
            assert_eq!(got.len(), corners.len(), "{name}");
            for (index, (point, (lat, lng))) in got.iter().zip(&corners).enumerate() {
                assert_eq!(point.latitude().to_bits(), lat.to_bits(), "{name} {index}");
                assert_eq!(point.longitude().to_bits(), lng.to_bits(), "{name} {index}");
            }
        }
        assert_eq!(cases, 6);
    }

    /// No corners, nothing done: the C# returns before its question.
    #[test]
    fn no_polygon_is_left_alone() {
        assert_eq!(offset_polygon(&[], 10.0), None);
    }
}
