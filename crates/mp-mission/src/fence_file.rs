//! `.fen`, `.ral` and `.poly` files: the formats Mission Planner saves fences, rally points and
//! the drawn polygon in.
//!
//! Ported from `GCSViews/FlightPlanner.cs` @ efb0801 (GPL-3.0-or-later) - the writers at 5970,
//! 6038 and 5892, the readers at 4349, 4419 and 4528.
//!
//! The `.poly` is the `.fen` without its return point: `#saved by Mission Planner <version>`, then
//! `lat lng` per corner and the first corner again. The other two are not alike:
//!
//! ```text
//! # .fen - space separated, and the first data line is the RETURN POINT, not a vertex
//! #saved by APM Planner 1.3.80
//! -35.362938 149.165085     <- return point
//! -35.363000 149.165000     <- vertex 1
//! -35.363000 149.166000     <- vertex 2
//! -35.362000 149.166000     <- vertex 3
//! -35.363000 149.165000     <- vertex 1 again, closing the ring
//!
//! # .ral - tab separated, one keyword-led row per point
//! #saved by Mission Planner 1.3.80
//! RALLY <tab> -35.363000 <tab> 149.165000 <tab> 100 <tab> 0 <tab> 0 <tab> 0
//! ```
//!
//! The `.fen` shape is the part to get right. The first line being the return point rather than a
//! vertex is not signposted anywhere in the file, and reading it as a vertex produces a fence with
//! one extra corner - at the return point, which is usually inside the fence, so the polygon
//! folds in on itself. The repeated last line is the same first vertex closing the ring, and
//! keeping it produces a duplicate corner instead.

use mp_units::LatLon;

/// A geofence as a file holds it.
#[derive(Debug, Clone, PartialEq)]
pub struct FenceFile {
    /// Where the vehicle goes when it breaches. The first data line.
    pub return_point: Option<LatLon>,
    /// The polygon, without the repeated closing point.
    pub vertices: Vec<LatLon>,
}

/// A rally point as `loadFromFileToolStripMenuItem1_Click` reads a row into a
/// `mavlink_rally_point_t`: the position through `(int)(float.Parse(..) * 1e7)` and back over 1e7,
/// the altitudes as `(short)`, the landing direction as `(ushort)`, the flags as `byte.Parse`.
/// The marker the C# then makes keeps only the position and the altitude.
/// `// C#: GCSViews/FlightPlanner.cs:4437-4458`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RallyPointFile {
    /// Where it is: `rally.lat / 1e7`, `rally.lng / 1e7`.
    pub position: LatLon,
    /// `rally.alt`, a `short`, in metres.
    pub altitude: f64,
    /// `rally.break_alt`, a `short`.
    pub break_altitude: f64,
    /// `rally.land_dir`, a `ushort`.
    pub land_direction: f64,
    /// `rally.flags`.
    pub flags: u8,
}

/// The header a `.fen` starts with, `"#saved by APM Planner " + Application.ProductVersion`, and
/// which a reader skips. This application's version stands in for Mission Planner's.
/// `// C#: GCSViews/FlightPlanner.cs:5978`
const FENCE_HEADER: &str = concat!("#saved by APM Planner ", env!("CARGO_PKG_VERSION"));

/// The header a `.ral` starts with, `"#saved by Mission Planner " + Application.ProductVersion`.
/// `// C#: GCSViews/FlightPlanner.cs:6046`
const RALLY_HEADER: &str = concat!("#saved by Mission Planner ", env!("CARGO_PKG_VERSION"));

/// The header a `.poly` starts with: the same words as a `.ral`'s.
/// `// C#: GCSViews/FlightPlanner.cs:5909`
const POLYGON_HEADER: &str = concat!("#saved by Mission Planner ", env!("CARGO_PKG_VERSION"));

/// `double.ToString(CultureInfo.InvariantCulture)` as Mission Planner's .NET Framework runtime
/// writes it, which is the general format at fifteen significant digits: trailing zeros dropped,
/// and scientific notation - `1E-05`, `1.5E+15` - once the exponent is below -4 or above 14.
/// (.NET Core writes the shortest text that reads back instead; Mission Planner targets
/// `net472`, and Mono writes it as the Framework does.) `mp_params::param_file::invariant_double`
/// is the same function for `.param` files; this crate does not depend on that one, and a test in
/// `mp-gui`, which has both, holds them to the same answers.
#[must_use]
pub fn invariant_double(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned();
    }
    if value == 0.0 {
        // The Framework writes negative zero as "0".
        return "0".to_owned();
    }
    // Fifteen significant digits, correctly rounded.
    let scientific = format!("{:.14e}", value.abs());
    let Some((mantissa, exponent)) = scientific.split_once('e') else {
        return scientific;
    };
    let Ok(exponent) = exponent.parse::<i32>() else {
        return scientific;
    };
    let digits: String = mantissa.chars().filter(char::is_ascii_digit).collect();
    let digits = digits.trim_end_matches('0');
    let digits = if digits.is_empty() { "0" } else { digits };
    let sign = if value < 0.0 { "-" } else { "" };

    if (-5 < exponent) && (exponent < 15) {
        let body = if exponent >= 0 {
            let whole_len = usize::try_from(exponent).unwrap_or(0) + 1;
            if digits.len() <= whole_len {
                format!("{digits}{}", "0".repeat(whole_len - digits.len()))
            } else {
                let (whole, fraction) = digits.split_at(whole_len);
                format!("{whole}.{fraction}")
            }
        } else {
            let zeros = usize::try_from(-exponent - 1).unwrap_or(0);
            format!("0.{}{digits}", "0".repeat(zeros))
        };
        return format!("{sign}{body}");
    }
    let (first, rest) = digits.split_at(1);
    let mantissa = if rest.is_empty() {
        first.to_owned()
    } else {
        format!("{first}.{rest}")
    };
    let exponent_sign = if exponent < 0 { '-' } else { '+' };
    format!(
        "{sign}{mantissa}E{exponent_sign}{:02}",
        exponent.unsigned_abs()
    )
}

/// Writes a `.fen`.
///
/// The return point first, then every vertex, then the first vertex again to close the ring -
/// which is what `saveToFileToolStripMenuItem_Click` does and what ArduPilot's own tools expect to
/// read back. Each line is `lat.ToString(InvariantCulture) + " " + lng.ToString(...)`.
/// `// C#: GCSViews/FlightPlanner.cs:5976-6008`
#[must_use]
pub fn write_fence(fence: &FenceFile) -> String {
    let line = |position: LatLon| {
        format!(
            "{} {}\n",
            invariant_double(position.latitude()),
            invariant_double(position.longitude())
        )
    };
    let mut out = format!("{FENCE_HEADER}\n");
    if let Some(point) = fence.return_point {
        out.push_str(&line(point));
    }
    for vertex in &fence.vertices {
        out.push_str(&line(*vertex));
    }
    // The ring is closed by repeating the first vertex. Not a formatting nicety: a reader that
    // takes the points as given draws an open shape, and an open geofence is not a fence.
    if let Some(first) = fence.vertices.first() {
        out.push_str(&line(*first));
    }
    out
}

/// Reads a `.fen`.
///
/// Lines beginning `#` are comments. The first data line is the return point; the rest are
/// vertices, and a final line equal to the first vertex is the closing repeat and is dropped.
/// `// C#: GCSViews/FlightPlanner.cs:4360-4395`
#[must_use]
pub fn read_fence(text: &str) -> FenceFile {
    let mut return_point = None;
    let mut vertices = Vec::new();
    for line in text.lines() {
        if line.starts_with('#') {
            continue;
        }
        let mut fields = line.split([' ', '\t']).filter(|field| !field.is_empty());
        let (Some(latitude), Some(longitude)) = (fields.next(), fields.next()) else {
            continue;
        };
        let (Ok(latitude), Ok(longitude)) = (latitude.parse::<f64>(), longitude.parse::<f64>())
        else {
            continue;
        };
        let Ok(position) = LatLon::new(latitude, longitude) else {
            continue;
        };
        if return_point.is_none() {
            return_point = Some(position);
        } else {
            vertices.push(position);
        }
    }

    // Drop the closing repeat. Compared by value rather than by counting, because a file written
    // by something other than Mission Planner may not have one - and dropping the last vertex of
    // a fence that was not closed removes a real corner. "remove loop close": more than one
    // point, and the last the same as the first.
    // `// C#: GCSViews/FlightPlanner.cs:4398-4403`
    if vertices.len() > 1 && vertices.first() == vertices.last() {
        vertices.pop();
    }

    FenceFile {
        return_point,
        vertices,
    }
}

/// Writes a `.ral`: `saveToFileToolStripMenuItem1_Click`.
///
/// Tab separated, one `RALLY` row per marker: its latitude and longitude as
/// `ToString(CultureInfo.InvariantCulture)` writes a `double`, its altitude -
/// `GMapMarkerRallyPt.Alt`, an `int` - and then `0, 0, 0` whatever the point was loaded with: the
/// marker keeps no break altitude, landing direction or flags, so the C# writes the literals.
/// `// C#: GCSViews/FlightPlanner.cs:6044-6055; ExtLibs/Maps/GMapMarkerRallyPt.cs:23`
#[must_use]
pub fn write_rally(markers: &[(LatLon, i32)]) -> String {
    let mut out = format!("{RALLY_HEADER}\n");
    for (position, altitude) in markers {
        out.push_str(&format!(
            "RALLY\t{}\t{}\t{altitude}\t0\t0\t0\n",
            invariant_double(position.latitude()),
            invariant_double(position.longitude()),
        ));
    }
    out
}

/// One row as the C# reads it, or nothing where it would throw: `float.Parse` and `byte.Parse`
/// refusing their text, or an index past the row's end. `(int)`, `(short)` and `(ushort)` of a
/// floating value truncate toward zero in the C#'s unchecked context, as `as` does inside the
/// range, which is all a rally point uses.
/// `// C#: GCSViews/FlightPlanner.cs:4437-4444`
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // the C#'s casts, on purpose
fn rally_row(items: &[&str]) -> Option<RallyPointFile> {
    let float = |index: usize| {
        items
            .get(index)
            .and_then(|text| crate::dotnet::parse_f32(text))
    };
    // `rally.lat = (int) (float.Parse(items[1]) * 1e7)`: the float widened, times the double 1e7,
    // truncated; the marker is at `rally.lat / 1e7`.
    let latitude = (f64::from(float(1)?) * 1e7) as i32;
    let longitude = (f64::from(float(2)?) * 1e7) as i32;
    let altitude = float(3)? as i16;
    let break_altitude = float(4)? as i16;
    let land_direction = float(5)? as u16;
    let flags =
        crate::dotnet::parse_i32(items.get(6)?).and_then(|flags| u8::try_from(flags).ok())?;
    let position = LatLon::new(f64::from(latitude) / 1e7, f64::from(longitude) / 1e7).ok()?;
    Some(RallyPointFile {
        position,
        altitude: f64::from(altitude),
        break_altitude: f64::from(break_altitude),
        land_direction: f64::from(land_direction),
        flags,
    })
}

/// Reads a `.ral`: `loadFromFileToolStripMenuItem1_Click`.
///
/// A line starting `#` is passed over; any other is split on spaces and tabs and read from
/// `items[1]` to `items[6]`. The first word is not looked at - the C# reads a row whatever it
/// begins with.
///
/// **A divergence, kept from the first port of this reader.** The C# indexes `items[1]` to
/// `items[6]` and parses them unchecked, so a row that is short or does not parse throws, and the
/// handler is abandoned there with the points before it on the map. Here that row is passed over
/// and the rest of the file is read.
/// `// C#: GCSViews/FlightPlanner.cs:4426-4461`
#[must_use]
pub fn read_rally(text: &str) -> Vec<RallyPointFile> {
    let mut points = Vec::new();
    for line in text.lines() {
        if line.starts_with('#') {
            continue;
        }
        let items: Vec<&str> = line
            .split([' ', '\t'])
            .filter(|field| !field.is_empty())
            .collect();
        if let Some(point) = rally_row(&items) {
            points.push(point);
        }
    }
    points
}

/// Writes a `.poly`: `savePolygonToolStripMenuItem_Click`.
///
/// The header, every corner as `lat.ToString(InvariantCulture) + " " + lng.ToString(...)`, then
/// the first corner again. The C# returns before its dialog when there are no corners, so it
/// never writes an empty polygon; here that is the header alone.
/// `// C#: GCSViews/FlightPlanner.cs:5892-5933`
#[must_use]
pub fn write_polygon(vertices: &[LatLon]) -> String {
    let line = |position: &LatLon| {
        format!(
            "{} {}\n",
            invariant_double(position.latitude()),
            invariant_double(position.longitude())
        )
    };
    let mut out = format!("{POLYGON_HEADER}\n");
    for vertex in vertices {
        out.push_str(&line(vertex));
    }
    if let Some(first) = vertices.first() {
        out.push_str(&line(first));
    }
    out
}

/// Reads a `.poly`: `loadPolygonToolStripMenuItem_Click`.
///
/// `#` lines are passed over, and so is a line of fewer than two words; the rest are
/// `double.Parse(items[0])` for latitude and `items[1]` for longitude. Then "remove loop close":
/// more than one corner, and the last equal to the first, loses the last.
///
/// A line whose numbers `double.Parse` refuses throws in the C# and abandons the load with the
/// corners before it drawn; here that line is passed over, as [`read_rally`] passes one over.
/// `// C#: GCSViews/FlightPlanner.cs:4540-4576`
#[must_use]
pub fn read_polygon(text: &str) -> Vec<LatLon> {
    let mut vertices = Vec::new();
    for line in text.lines() {
        if line.starts_with('#') {
            continue;
        }
        let items: Vec<&str> = line
            .split([' ', '\t'])
            .filter(|field| !field.is_empty())
            .collect();
        let (Some(latitude), Some(longitude)) = (items.first(), items.get(1)) else {
            continue;
        };
        let (Some(latitude), Some(longitude)) = (
            crate::dotnet::parse_f64(latitude),
            crate::dotnet::parse_f64(longitude),
        ) else {
            continue;
        };
        if let Ok(position) = LatLon::new(latitude, longitude) {
            vertices.push(position);
        }
    }
    if vertices.len() > 1 && vertices.first() == vertices.last() {
        vertices.pop();
    }
    vertices
}

/// How a file differs from what a vehicle holds.
///
/// The question a read-back exists to answer. Uploading a fence and reading it back tells you the
/// transfer worked; comparing it against the file that produced it tells you the *fence* is the
/// one you drew - which is a different claim, and the one a pilot cares about before flying
/// inside it.
#[derive(Debug, Clone, PartialEq)]
pub enum FenceDifference {
    /// The vehicle has a different number of vertices.
    VertexCount {
        /// How many the file has.
        file: usize,
        /// How many the vehicle reported.
        vehicle: usize,
    },
    /// A vertex is in a different place.
    Vertex {
        /// Which one, counting from zero.
        index: usize,
        /// How far apart they are, in metres.
        metres: f64,
    },
    /// The return point differs, or one side has none.
    ReturnPoint {
        /// How far apart they are, in metres, where both have one.
        metres: Option<f64>,
    },
}

/// How far apart two positions may be and still count as the same point.
///
/// A fence travels as `int32` degrees times 1e7, so a round trip loses about a centimetre. Ten
/// centimetres is far inside that tolerance and far outside any difference that matters to a
/// fence, whose corners are tens of metres apart.
pub const SAME_POINT_METRES: f64 = 0.1;

impl FenceFile {
    /// Compares this file against what a vehicle reported.
    ///
    /// Empty means they agree. Ordered so the first difference is the most structural: a vertex
    /// count that disagrees makes every per-vertex comparison meaningless, so it is reported alone.
    #[must_use]
    pub fn compare(&self, vehicle: &Self) -> Vec<FenceDifference> {
        let mut differences = Vec::new();

        match (self.return_point, vehicle.return_point) {
            (Some(file), Some(reported)) => {
                let metres = file.distance_to(reported).0;
                if metres > SAME_POINT_METRES {
                    differences.push(FenceDifference::ReturnPoint {
                        metres: Some(metres),
                    });
                }
            }
            (None, None) => {}
            _ => differences.push(FenceDifference::ReturnPoint { metres: None }),
        }

        if self.vertices.len() != vehicle.vertices.len() {
            differences.push(FenceDifference::VertexCount {
                file: self.vertices.len(),
                vehicle: vehicle.vertices.len(),
            });
            return differences;
        }

        for (index, (file, reported)) in self
            .vertices
            .iter()
            .zip(vehicle.vertices.iter())
            .enumerate()
        {
            let metres = file.distance_to(*reported).0;
            if metres > SAME_POINT_METRES {
                differences.push(FenceDifference::Vertex { index, metres });
            }
        }

        differences
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(latitude: f64, longitude: f64) -> LatLon {
        LatLon::new(latitude, longitude).expect("a valid position")
    }

    /// The first data line is the return point, not a vertex.
    ///
    /// Nothing in the file says so. Reading it as a vertex gives a fence with an extra corner at
    /// the return point - which is usually inside the fence, so the polygon folds in on itself.
    #[test]
    fn the_first_line_of_a_fence_is_the_return_point() {
        let text = "#saved by APM Planner 1.3.80\n\
                    -35.362938 149.165085\n\
                    -35.363000 149.165000\n\
                    -35.363000 149.166000\n\
                    -35.362000 149.166000\n\
                    -35.363000 149.165000\n";
        let fence = read_fence(text);
        assert_eq!(fence.return_point, Some(at(-35.362_938, 149.165_085)));
        assert_eq!(fence.vertices.len(), 3, "{:?}", fence.vertices);
        assert_eq!(fence.vertices[0], at(-35.363_000, 149.165_000));
    }

    /// The repeated last line closes the ring and is not a fourth corner.
    #[test]
    fn the_closing_repeat_is_not_a_vertex() {
        let fence = read_fence("-35.0 149.0\n-35.1 149.1\n-35.1 149.2\n-35.2 149.2\n-35.1 149.1\n");
        assert_eq!(fence.vertices.len(), 3);
        assert_ne!(
            fence.vertices.first(),
            fence.vertices.last(),
            "the ring should not still be closed after reading"
        );
    }

    /// A fence that was never closed keeps all its corners.
    ///
    /// Counting instead of comparing would drop a real vertex from a file written by something
    /// other than Mission Planner.
    #[test]
    fn an_unclosed_fence_keeps_every_vertex() {
        let fence = read_fence("-35.0 149.0\n-35.1 149.1\n-35.1 149.2\n-35.2 149.3\n");
        assert_eq!(fence.vertices.len(), 3);
        assert_eq!(fence.vertices.last(), Some(&at(-35.2, 149.3)));
    }

    /// What is written reads back as what was written.
    #[test]
    fn a_fence_round_trips() {
        let fence = FenceFile {
            return_point: Some(at(-35.362_938, 149.165_085)),
            vertices: vec![
                at(-35.363, 149.165),
                at(-35.363, 149.166),
                at(-35.362, 149.166),
            ],
        };
        let text = write_fence(&fence);
        assert!(text.starts_with('#'), "the header should be a comment");
        // Five data lines: the return point, three vertices, and the closing repeat.
        assert_eq!(text.lines().filter(|l| !l.starts_with('#')).count(), 5);
        assert_eq!(read_fence(&text), fence);
    }

    /// Numbers are written as the .NET Framework writes a `double`: fifteen significant digits,
    /// trailing zeros dropped, scientific past the exponents where the general format switches.
    #[test]
    fn numbers_are_written_as_the_framework_writes_a_double() {
        assert_eq!(invariant_double(-35.362_938), "-35.362938");
        assert_eq!(invariant_double(149.165_085), "149.165085");
        assert_eq!(invariant_double(100.0), "100");
        assert_eq!(invariant_double(0.0), "0");
        assert_eq!(invariant_double(-0.0), "0");
        // A clicked position carries more digits than fifteen, and the sixteenth goes.
        assert_eq!(
            invariant_double(-35.363_262_345_678_91),
            "-35.3632623456789"
        );
        // A float parameter widened to a double shows its binary noise to fifteen digits.
        assert_eq!(invariant_double(f64::from(0.3_f32)), "0.300000011920929");
        assert_eq!(invariant_double(0.0001), "0.0001");
        assert_eq!(invariant_double(0.000_01), "1E-05");
        assert_eq!(invariant_double(-0.000_000_15), "-1.5E-07");
        assert_eq!(invariant_double(123_456_789_012_345.0), "123456789012345");
        assert_eq!(
            invariant_double(1_234_567_890_123_456.0),
            "1.23456789012346E+15"
        );
    }

    /// The header is the C#'s for a fence, and a reader skips it.
    #[test]
    fn a_fence_file_starts_with_the_c_sharp_header() {
        let text = write_fence(&FenceFile {
            return_point: Some(at(-35.0, 149.0)),
            vertices: Vec::new(),
        });
        let first = text.lines().next().expect("a header");
        assert!(first.starts_with("#saved by APM Planner "), "{first}");
        assert_eq!(read_fence(&text).return_point, Some(at(-35.0, 149.0)));
    }

    /// Two points the same are a closed loop of one, as the C#'s `Count > 1` has it.
    #[test]
    fn two_equal_points_close_on_one() {
        let fence = read_fence("-35.0 149.0\n-35.1 149.1\n-35.1 149.1\n");
        assert_eq!(fence.vertices, vec![at(-35.1, 149.1)]);
    }

    /// A fence with no vertices writes no closing line and reads back empty.
    #[test]
    fn an_empty_fence_does_not_write_a_closing_line() {
        let fence = FenceFile {
            return_point: Some(at(-35.0, 149.0)),
            vertices: Vec::new(),
        };
        let text = write_fence(&fence);
        assert_eq!(text.lines().filter(|l| !l.starts_with('#')).count(), 1);
        assert_eq!(read_fence(&text), fence);
    }

    /// The rally markers the oracle wrote (`tools/csharp-reference/PlannerOracle.cs`, `RallyMarkers`).
    fn oracle_markers() -> Vec<(LatLon, i32)> {
        vec![
            (at(-35.363_262_1, 149.165_237_4), 100),
            (at(-35.363_262_345_678_91, 149.165_237_412_345_67), 60),
            (at(-0.000_01, 0.000_015), 0),
        ]
    }

    /// The `.ral` `saveToFileToolStripMenuItem1_Click` writes, under mono: after the header's
    /// version, the same bytes - fifteen significant digits, `1E-05` where the Framework writes
    /// it, the altitude as an integer and three literal zeros.
    #[test]
    fn a_rally_file_is_the_c_sharps_byte_for_byte() {
        let golden = include_str!("../../../testdata/planner/golden/rally.ral");
        let ours = write_rally(&oracle_markers());
        let (golden_header, golden_body) = golden.split_once('\n').expect("a header");
        let (our_header, our_body) = ours.split_once('\n').expect("a header");
        assert_eq!(our_body, golden_body);
        assert!(golden_header.starts_with("#saved by Mission Planner "));
        assert!(our_header.starts_with("#saved by Mission Planner "));
    }

    /// What `loadFromFileToolStripMenuItem1_Click` makes of each row, under mono: the position
    /// through a `float` and a truncation to 1e-7, the altitude a `short`, the rest parsed and -
    /// for the marker - dropped. Whatever the first word says.
    #[test]
    fn a_rally_file_reads_as_the_c_sharp_reads_it() {
        let golden = include_str!("../../../testdata/planner/golden/rally_read.csv");
        let text = format!(
            "{}RALLY\t-35.12345678\t149.98765432\t123.9\t-4.5\t270.7\t1\nANYTHING 51.5 -0.125 7 0 0 0\n",
            include_str!("../../../testdata/planner/golden/rally.ral")
        );
        let points = read_rally(&text);
        let expected: Vec<Vec<&str>> = golden
            .lines()
            .map(|line| line.split(',').collect())
            .collect();
        assert_eq!(points.len(), expected.len());
        for (point, row) in points.iter().zip(&expected) {
            let number = |index: usize| row[index].parse::<f64>().expect("a number");
            assert_eq!(
                point.position.latitude().to_bits(),
                number(1).to_bits(),
                "{row:?}"
            );
            assert_eq!(
                point.position.longitude().to_bits(),
                number(2).to_bits(),
                "{row:?}"
            );
            assert_eq!(point.altitude, number(3), "{row:?}");
            assert_eq!(point.break_altitude, number(4), "{row:?}");
            assert_eq!(point.land_direction, number(5), "{row:?}");
            assert_eq!(f64::from(point.flags), number(6), "{row:?}");
        }
    }

    /// Rows the C# would throw on - short, or `byte.Parse` refusing its flags - are passed over.
    #[test]
    fn a_line_the_c_sharp_throws_on_is_skipped() {
        let points = read_rally(
            "#comment\n\
             RALLY\t-35.0\t149.0\t100\t0\t0\t0\n\
             GARBAGE\t1\t2\t3\n\
             \n\
             RALLY\t-35.2\t149.2\t100\t0\t0\t1.5\n\
             RALLY\t-35.1\t149.1\t120\t0\t0\t0\n",
        );
        assert_eq!(points.len(), 2);
        assert_eq!(points[1].altitude, 120.0);
    }

    /// The `.poly` `savePolygonToolStripMenuItem_Click` writes, under mono, after the header's
    /// version: every corner, then the first again.
    #[test]
    fn a_polygon_file_is_the_c_sharps_byte_for_byte() {
        let golden = include_str!("../../../testdata/planner/golden/polygon.poly");
        let corners = vec![
            at(-35.362, 149.164),
            at(-35.362, 149.165),
            at(-35.363, 149.165),
            at(-35.363, 149.167),
            at(-35.364, 149.167),
            at(-35.364, 149.164),
            at(-35.363_262_345_678_91, 149.165_237_412_345_67),
        ];
        let ours = write_polygon(&corners);
        let (_, golden_body) = golden.split_once('\n').expect("a header");
        let (our_header, our_body) = ours.split_once('\n').expect("a header");
        assert_eq!(our_body, golden_body);
        assert!(our_header.starts_with("#saved by Mission Planner "));
        // Read back: the closing repeat goes, the fifteen digits stay as written.
        let read = read_polygon(&ours);
        assert_eq!(read.len(), corners.len());
        assert_eq!(read[..6], corners[..6]);
        assert_eq!(read[6], at(-35.363_262_345_678_9, 149.165_237_412_346));
    }

    /// A short line, and a line `double.Parse` refuses, are passed over; an open polygon keeps its
    /// last corner.
    #[test]
    fn a_polygon_file_skips_what_it_cannot_read() {
        let read = read_polygon("#x\n-35.0 149.0\n-35.1\nfoo bar\n-35.1 149.1\n-35.2 149.0\n");
        assert_eq!(
            read,
            vec![at(-35.0, 149.0), at(-35.1, 149.1), at(-35.2, 149.0)]
        );
        assert!(read_polygon("").is_empty());
    }

    /// A short row loses that point, not the rest of the file.
    ///
    /// The C# indexes items[1] through items[6] unchecked, so a truncated line throws and the
    /// load is abandoned there - the rows after a bad one are never read.
    #[test]
    fn a_truncated_row_does_not_lose_the_whole_file() {
        let points = read_rally(
            "RALLY\t-35.0\t149.0\t100\t0\t0\t0\n\
             RALLY\t-35.1\n\
             RALLY\t-35.2\t149.2\t140\t0\t0\t0\n",
        );
        assert_eq!(points.len(), 2, "the good rows should survive the bad one");
    }

    /// A fence that matches reports nothing. Silence is the answer a pilot wants.
    #[test]
    fn a_fence_that_matches_reports_no_differences() {
        let fence = FenceFile {
            return_point: Some(at(-35.362_938, 149.165_085)),
            vertices: vec![
                at(-35.363, 149.165),
                at(-35.363, 149.166),
                at(-35.362, 149.166),
            ],
        };
        assert!(fence.compare(&fence).is_empty());
    }

    /// A round trip through the wire's int32-times-1e7 is not a difference.
    ///
    /// Without a tolerance every read-back reports every corner as moved, by a centimetre, and a
    /// comparison that always disagrees is one nobody reads.
    #[test]
    fn a_centimetre_of_wire_rounding_is_not_a_difference() {
        let fence = FenceFile {
            return_point: None,
            vertices: vec![at(-35.363_262, 149.165_237)],
        };
        // What survives a trip through degrees * 1e7 as an int32.
        let rounded = at(
            (-35.363_262_f64 * 1e7).round() / 1e7,
            (149.165_237_f64 * 1e7).round() / 1e7,
        );
        let reported = FenceFile {
            return_point: None,
            vertices: vec![rounded],
        };
        assert!(
            fence.compare(&reported).is_empty(),
            "{:?}",
            fence.compare(&reported)
        );
    }

    /// A corner that actually moved is reported, with how far.
    #[test]
    fn a_moved_vertex_is_reported_with_its_distance() {
        let fence = FenceFile {
            return_point: None,
            vertices: vec![at(-35.363, 149.165), at(-35.363, 149.166)],
        };
        let reported = FenceFile {
            return_point: None,
            // About 110 m south.
            vertices: vec![at(-35.364, 149.165), at(-35.363, 149.166)],
        };
        let differences = fence.compare(&reported);
        assert_eq!(differences.len(), 1);
        match differences[0] {
            FenceDifference::Vertex { index, metres } => {
                assert_eq!(index, 0);
                assert!((100.0..120.0).contains(&metres), "{metres} m");
            }
            ref other => panic!("expected a moved vertex, got {other:?}"),
        }
    }

    /// A different number of corners is reported on its own.
    ///
    /// Comparing vertex 3 of a four-corner fence against vertex 3 of a three-corner one is
    /// comparing different corners, so every per-vertex difference after a count mismatch is
    /// noise.
    #[test]
    fn a_vertex_count_mismatch_is_reported_alone() {
        let fence = FenceFile {
            return_point: None,
            vertices: vec![
                at(-35.363, 149.165),
                at(-35.363, 149.166),
                at(-35.362, 149.166),
            ],
        };
        let reported = FenceFile {
            return_point: None,
            vertices: vec![at(-40.0, 140.0)],
        };
        let differences = fence.compare(&reported);
        assert_eq!(differences.len(), 1);
        assert_eq!(
            differences[0],
            FenceDifference::VertexCount {
                file: 3,
                vehicle: 1
            }
        );
    }

    /// A return point on one side and not the other is a difference.
    #[test]
    fn a_missing_return_point_is_a_difference() {
        let with = FenceFile {
            return_point: Some(at(-35.0, 149.0)),
            vertices: Vec::new(),
        };
        let without = FenceFile {
            return_point: None,
            vertices: Vec::new(),
        };
        assert_eq!(
            with.compare(&without),
            vec![FenceDifference::ReturnPoint { metres: None }]
        );
        assert_eq!(
            without.compare(&with),
            vec![FenceDifference::ReturnPoint { metres: None }]
        );
        assert!(without.compare(&without).is_empty());
    }

    /// A rally row with only position and altitude is not a rally point: the C# reads
    /// `items[4]` to `items[6]` unconditionally and throws on it. (An earlier port defaulted the
    /// missing columns; nothing in the C# does.)
    #[test]
    fn a_row_without_its_last_three_columns_is_not_read() {
        assert!(read_rally("RALLY\t-35.0\t149.0\t100\n").is_empty());
    }
}
