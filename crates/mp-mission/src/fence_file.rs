//! `.fen` and `.ral` files: the formats Mission Planner saves fences and rally points in.
//!
//! Ported from `GCSViews/FlightPlanner.cs` @ efb0801 (GPL-3.0-or-later) - the writers at 5970 and
//! 6038, the readers at 4349 and 4419.
//!
//! Two formats, and they are not alike:
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

/// A rally point as a file holds it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RallyPointFile {
    /// Where it is.
    pub position: LatLon,
    /// Altitude, in metres.
    pub altitude: f64,
    /// Break altitude - the height to leave a loiter at, for a plane.
    pub break_altitude: f64,
    /// Landing direction in degrees, for a plane.
    pub land_direction: f64,
    /// `RALLY_FLAGS`.
    pub flags: u8,
}

/// The header Mission Planner writes, and skips on read.
const HEADER: &str = "#saved by Mission Planner";

/// Writes a `.fen`.
///
/// The return point first, then every vertex, then the first vertex again to close the ring -
/// which is what `savefence` does and what ArduPilot's own tools expect to read back.
/// `// C#: GCSViews/FlightPlanner.cs:5976-6008`
#[must_use]
pub fn write_fence(fence: &FenceFile) -> String {
    let mut out = format!("{HEADER}\n");
    if let Some(point) = fence.return_point {
        out.push_str(&format!("{} {}\n", point.latitude(), point.longitude()));
    }
    for vertex in &fence.vertices {
        out.push_str(&format!("{} {}\n", vertex.latitude(), vertex.longitude()));
    }
    // The ring is closed by repeating the first vertex. Not a formatting nicety: a reader that
    // takes the points as given draws an open shape, and an open geofence is not a fence.
    if let Some(first) = fence.vertices.first() {
        out.push_str(&format!("{} {}\n", first.latitude(), first.longitude()));
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
    // a fence that was not closed removes a real corner.
    if vertices.len() > 2 && vertices.first() == vertices.last() {
        vertices.pop();
    }

    FenceFile {
        return_point,
        vertices,
    }
}

/// Writes a `.ral`.
///
/// Tab separated, one `RALLY` row per point. `// C#: GCSViews/FlightPlanner.cs:6049-6054`
#[must_use]
pub fn write_rally(points: &[RallyPointFile]) -> String {
    let mut out = format!("{HEADER}\n");
    for point in points {
        out.push_str(&format!(
            "RALLY\t{}\t{}\t{}\t{}\t{}\t{}\n",
            point.position.latitude(),
            point.position.longitude(),
            point.altitude,
            point.break_altitude,
            point.land_direction,
            point.flags
        ));
    }
    out
}

/// Reads a `.ral`.
///
/// `// C#: GCSViews/FlightPlanner.cs:4430-4445`
#[must_use]
pub fn read_rally(text: &str) -> Vec<RallyPointFile> {
    let mut points = Vec::new();
    for line in text.lines() {
        if line.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = line
            .split([' ', '\t'])
            .filter(|field| !field.is_empty())
            .collect();
        // The keyword, then latitude, longitude, altitude at minimum. The C# indexes items[1]
        // through items[6] without checking, which throws on a short line and abandons the file;
        // a short line here is skipped and the rest of the points are kept.
        let number = |index: usize| -> f64 {
            fields
                .get(index)
                .and_then(|field| field.parse::<f64>().ok())
                .unwrap_or(0.0)
        };
        let is_rally = fields
            .first()
            .is_some_and(|keyword| keyword.eq_ignore_ascii_case("RALLY"));
        if fields.len() < 4 || !is_rally {
            continue;
        }
        let (Some(Ok(latitude)), Some(Ok(longitude))) = (
            fields.get(1).map(|f| f.parse::<f64>()),
            fields.get(2).map(|f| f.parse::<f64>()),
        ) else {
            continue;
        };
        let Ok(position) = LatLon::new(latitude, longitude) else {
            continue;
        };
        points.push(RallyPointFile {
            position,
            altitude: number(3),
            break_altitude: number(4),
            land_direction: number(5),
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            flags: number(6).clamp(0.0, 255.0) as u8,
        });
    }
    points
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

    /// Rally points are tab separated and keyword led.
    #[test]
    fn a_rally_file_round_trips() {
        let points = vec![
            RallyPointFile {
                position: at(-35.363, 149.165),
                altitude: 100.0,
                break_altitude: 0.0,
                land_direction: 0.0,
                flags: 0,
            },
            RallyPointFile {
                position: at(-35.361, 149.167),
                altitude: 120.0,
                break_altitude: 45.0,
                land_direction: 270.0,
                flags: 1,
            },
        ];
        let text = write_rally(&points);
        assert!(text.contains("RALLY\t-35.363\t149.165\t100"));
        assert_eq!(read_rally(&text), points);
    }

    /// A row that is not a rally point is skipped, not guessed at.
    #[test]
    fn a_line_that_is_not_a_rally_point_is_skipped() {
        let points = read_rally(
            "#comment\n\
             RALLY\t-35.0\t149.0\t100\t0\t0\t0\n\
             GARBAGE\t1\t2\t3\n\
             \n\
             RALLY\t-35.1\t149.1\t120\t0\t0\t0\n",
        );
        assert_eq!(points.len(), 2);
        assert_eq!(points[1].altitude, 120.0);
    }

    /// A short row loses that point, not the rest of the file.
    ///
    /// The C# indexes items[1] through items[6] unchecked, so a truncated line throws and the
    /// whole load is abandoned - a file with one bad row loads as no rally points at all.
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

    /// A rally row with only position and altitude is still a rally point.
    #[test]
    fn the_optional_columns_default_rather_than_refusing() {
        let points = read_rally("RALLY\t-35.0\t149.0\t100\n");
        assert_eq!(points.len(), 1);
        assert_eq!(points[0].altitude, 100.0);
        assert_eq!(points[0].break_altitude, 0.0);
        assert_eq!(points[0].flags, 0);
    }
}
