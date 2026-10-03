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

//! The part of an ESRI shapefile Polygon > From SHP reads: every feature's coordinates, and the
//! `.prj` beside it.
//!
//! `fromSHPToolStripMenuItem_Click` (`GCSViews/FlightPlanner.cs:3533-3616` @ efb0801) opens the
//! file with DotSpatial's `FeatureSet.Open` and walks `fs.Features`, taking every coordinate of
//! each feature's geometry in turn; where a `.prj` of the same name sits beside it, it parses the
//! first line with `ProjectionInfo.ParseEsriString` and reprojects each coordinate to WGS 1984
//! with `Reproject.ReprojectPoints`. This module is those two reads:
//!
//! * [`features`] reads the `.shp` itself, from the layout in ESRI's *Shapefile Technical
//!   Description* (July 1998): a 100-byte header whose file code is 9994, then records of a
//!   big-endian number and length and a little-endian shape. Every shape type with coordinates is
//!   read - points, multipoints, polylines and polygons, with or without Z and M - and a feature's
//!   coordinates are its points in the file's order, all parts one after another.
//! * [`Projection::from_esri`] and [`Projection::to_wgs84`] handle the `.prj`: WGS 1984 itself, or
//!   WGS 1984 in a UTM zone - DotSpatial's transverse Mercator inverted with ProjNet's, which the
//!   survey grid already carries (`utm.rs`).
//!
//! **Where this differs from DotSpatial, and why.** The `.shx` index and the `.dbf` table beside a
//! shapefile are not read: the handler reads no attribute (it fills the table and never looks at
//! it), and the records follow one another in the `.shp`, so a file without its index loads here
//! where DotSpatial would throw. A polygon record of several rings gives its rings in the file's
//! order, where DotSpatial orders each shell before its holes - the same for every file whose
//! shells come first, which is how shapefiles are written. A `.prj` of any other coordinate system
//! is refused by name, where DotSpatial carries some four thousand of them; a null shape is passed
//! over; a multipatch is refused.

use mp_units::LatLon;

use crate::utm::UtmPos;

/// Why a `.shp` or `.prj` could not be read.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ShapefileError {
    /// Shorter than the 100-byte header, or a record runs past the end.
    #[error("the shape file ends early")]
    Truncated,
    /// The first word is not the shapefile code, 9994.
    #[error("not a shape file (file code {0})")]
    NotAShapefile(i32),
    /// A shape type this reader does not take.
    #[error("shape type {0} is not supported")]
    ShapeType(i32),
    /// A `.prj` in a coordinate system other than WGS 1984 or one of its UTM zones.
    #[error("the projection {0} is not supported")]
    Projection(String),
}

/// The `N` bytes at `at`, or the file ends early.
fn word<const N: usize>(bytes: &[u8], at: usize) -> Result<[u8; N], ShapefileError> {
    bytes
        .get(at..at.saturating_add(N))
        .and_then(|slice| slice.try_into().ok())
        .ok_or(ShapefileError::Truncated)
}

/// A big-endian `int` at `at`.
fn be_i32(bytes: &[u8], at: usize) -> Result<i32, ShapefileError> {
    Ok(i32::from_be_bytes(word(bytes, at)?))
}

/// A little-endian `int` at `at`.
fn le_i32(bytes: &[u8], at: usize) -> Result<i32, ShapefileError> {
    Ok(i32::from_le_bytes(word(bytes, at)?))
}

/// A little-endian `double` at `at`.
fn le_f64(bytes: &[u8], at: usize) -> Result<f64, ShapefileError> {
    Ok(f64::from_le_bytes(word(bytes, at)?))
}

/// A count read from the file, as a length.
fn count(value: i32) -> Result<usize, ShapefileError> {
    usize::try_from(value).map_err(|_| ShapefileError::Truncated)
}

/// `count` X, Y pairs from `at`.
fn points(bytes: &[u8], at: usize, count: usize) -> Result<Vec<(f64, f64)>, ShapefileError> {
    (0..count)
        .map(|index| {
            let at = at + index * 16;
            Ok((le_f64(bytes, at)?, le_f64(bytes, at + 8)?))
        })
        .collect()
}

/// The coordinates of one record's shape, `content` starting at its shape type.
fn shape(content: &[u8]) -> Result<Option<Vec<(f64, f64)>>, ShapefileError> {
    let kind = le_i32(content, 0)?;
    match kind {
        // Null: no geometry, passed over.
        0 => Ok(None),
        // Point, PointZ, PointM: X, Y, then Z and M which are not read.
        1 | 11 | 21 => Ok(Some(points(content, 4, 1)?)),
        // MultiPoint and its Z and M kin: box, count, points.
        8 | 18 | 28 => {
            let n = count(le_i32(content, 36)?)?;
            Ok(Some(points(content, 40, n)?))
        }
        // PolyLine and Polygon and their Z and M kin: box, parts, points, part starts, points.
        3 | 5 | 13 | 15 | 23 | 25 => {
            let parts = count(le_i32(content, 36)?)?;
            let n = count(le_i32(content, 40)?)?;
            Ok(Some(points(content, 44 + parts * 4, n)?))
        }
        other => Err(ShapefileError::ShapeType(other)),
    }
}

/// Every feature's coordinates as (X, Y) - longitude and latitude in a geographic file - in the
/// order the records hold them.
/// `// C#: GCSViews/FlightPlanner.cs:3564-3567, 3577-3588`
pub fn features(shp: &[u8]) -> Result<Vec<Vec<(f64, f64)>>, ShapefileError> {
    if shp.len() < 100 {
        return Err(ShapefileError::Truncated);
    }
    let code = be_i32(shp, 0)?;
    if code != 9994 {
        return Err(ShapefileError::NotAShapefile(code));
    }
    // The file's length in 16-bit words, header included; a file longer than it says is read to
    // what it says.
    let length = count(be_i32(shp, 24)?)?.saturating_mul(2).min(shp.len());
    let mut out = Vec::new();
    let mut at = 100;
    while at + 8 <= length {
        let content_words = count(be_i32(shp, at + 4)?)?;
        let start = at + 8;
        let end = start + content_words * 2;
        let content = shp.get(start..end).ok_or(ShapefileError::Truncated)?;
        if let Some(coordinates) = shape(content)? {
            out.push(coordinates);
        }
        at = end;
    }
    Ok(out)
}

/// Every vertex of a `.shp` in one list, as DotSpatial's `FeatureSet.Vertex` holds them - X and Y
/// pairs, record after record, part after part - with the Z of each where the file's shape type
/// carries one (`FeatureSet.Z`, null for a file without Z).
#[derive(Debug, Clone, PartialEq)]
pub struct Vertices {
    /// `Vertex`: (X, Y) per vertex, so `Vertex[row * 2]` is the `row`th vertex's X.
    pub xy: Vec<(f64, f64)>,
    /// `Z`: one per vertex for PointZ, MultiPointZ, PolyLineZ and PolygonZ files; `None` otherwise.
    pub z: Option<Vec<f64>>,
}

/// The Z values of one record's shape, `content` starting at its shape type; `None` for a shape
/// type without them.
fn shape_z(content: &[u8]) -> Result<Option<Vec<f64>>, ShapefileError> {
    let kind = le_i32(content, 0)?;
    let doubles = |at: usize, count: usize| -> Result<Vec<f64>, ShapefileError> {
        (0..count)
            .map(|index| le_f64(content, at + index * 8))
            .collect()
    };
    match kind {
        // PointZ: X, Y, Z, M.
        11 => Ok(Some(doubles(20, 1)?)),
        // MultiPointZ: box, count, points, Z range, Z array.
        18 => {
            let n = count(le_i32(content, 36)?)?;
            Ok(Some(doubles(40 + n * 16 + 16, n)?))
        }
        // PolyLineZ and PolygonZ: box, parts, count, part starts, points, Z range, Z array.
        13 | 15 => {
            let parts = count(le_i32(content, 36)?)?;
            let n = count(le_i32(content, 40)?)?;
            Ok(Some(doubles(44 + parts * 4 + n * 16 + 16, n)?))
        }
        _ => Ok(None),
    }
}

/// Every record's vertices in one list, with their Z where the file has one.
/// `// C#: GCSViews/FlightPlanner.cs:4602-4604, 4628-4629` (`fs.Vertex[row * 2]`, `fs.Z[row]`)
///
/// # Errors
///
/// As [`features`].
pub fn vertices(shp: &[u8]) -> Result<Vertices, ShapefileError> {
    if shp.len() < 100 {
        return Err(ShapefileError::Truncated);
    }
    let code = be_i32(shp, 0)?;
    if code != 9994 {
        return Err(ShapefileError::NotAShapefile(code));
    }
    // The header's shape type says whether the file carries Z at all.
    let with_z = matches!(le_i32(shp, 32)?, 11 | 13 | 15 | 18);
    let length = count(be_i32(shp, 24)?)?.saturating_mul(2).min(shp.len());
    let mut xy = Vec::new();
    let mut z = Vec::new();
    let mut at = 100;
    while at + 8 <= length {
        let content_words = count(be_i32(shp, at + 4)?)?;
        let start = at + 8;
        let end = start + content_words * 2;
        let content = shp.get(start..end).ok_or(ShapefileError::Truncated)?;
        if let Some(coordinates) = shape(content)? {
            xy.extend(coordinates);
            if with_z {
                z.extend(shape_z(content)?.unwrap_or_default());
            }
        }
        at = end;
    }
    Ok(Vertices {
        xy,
        z: with_z.then_some(z),
    })
}

/// A `.prj` this reader can reproject from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Projection {
    /// Latitude and longitude on WGS 1984: already what the map takes.
    Wgs84,
    /// WGS 1984 in a UTM zone: easting and northing in metres.
    Utm {
        /// The zone, 1 to 60.
        zone: u8,
        /// Northern hemisphere: no false northing.
        north: bool,
    },
}

/// A WKT node: its keyword, its quoted and numeric values in order, and the nodes inside it.
#[derive(Debug, Default)]
struct Node {
    keyword: String,
    values: Vec<String>,
    children: Vec<Node>,
}

impl Node {
    fn child(&self, keyword: &str) -> Option<&Self> {
        self.children
            .iter()
            .find(|child| child.keyword.eq_ignore_ascii_case(keyword))
    }

    fn name(&self) -> &str {
        self.values.first().map_or("", String::as_str)
    }

    fn number(&self, index: usize) -> Option<f64> {
        self.values.get(index)?.parse().ok()
    }

    /// `PARAMETER["name", value]` among this node's children.
    fn parameter(&self, name: &str) -> Option<f64> {
        self.children
            .iter()
            .filter(|child| child.keyword.eq_ignore_ascii_case("PARAMETER"))
            .find(|child| child.name().eq_ignore_ascii_case(name))
            .and_then(|child| child.number(1))
    }
}

/// Parses one WKT node from `text`, returning it and what follows it.
fn parse_node(text: &str) -> Option<(Node, &str)> {
    let text = text.trim_start();
    let open = text.find(['[', '('])?;
    let mut node = Node {
        keyword: text.get(..open)?.trim().to_owned(),
        ..Node::default()
    };
    let mut rest = text.get(open + 1..)?;
    loop {
        rest = rest.trim_start();
        let first = rest.chars().next()?;
        match first {
            ']' | ')' => return Some((node, rest.get(1..)?)),
            ',' => rest = rest.get(1..)?,
            '"' => {
                let inner = rest.get(1..)?;
                let close = inner.find('"')?;
                node.values.push(inner.get(..close)?.to_owned());
                rest = inner.get(close + 1..)?;
            }
            _ => {
                let end = rest.find([',', ']', ')', '[', '('])?;
                let value = rest.get(..end)?.trim();
                let after = rest.get(end..)?;
                if after.starts_with(['[', '(']) {
                    let (child, after) = parse_node(rest)?;
                    node.children.push(child);
                    rest = after;
                } else {
                    node.values.push(value.to_owned());
                    rest = after;
                }
            }
        }
    }
}

/// Whether a `GEOGCS` node is WGS 1984 in degrees from Greenwich.
fn is_wgs84(geogcs: &Node) -> bool {
    let datum = geogcs
        .child("DATUM")
        .map_or("", Node::name)
        .to_ascii_uppercase();
    let wgs84 = datum.contains("WGS_1984") || datum.contains("WGS 84") || datum.contains("WGS84");
    let greenwich = geogcs
        .child("PRIMEM")
        .and_then(|primem| primem.number(1))
        .is_some_and(|longitude| longitude == 0.0);
    let degrees = geogcs
        .child("UNIT")
        .and_then(|unit| unit.number(1))
        .is_some_and(|radians| (radians - std::f64::consts::PI / 180.0).abs() < 1e-12);
    wgs84 && greenwich && degrees
}

impl Projection {
    /// The coordinate system a `.prj`'s first line names, where it is one this reader takes:
    /// `pStart.ParseEsriString(re.ReadLine())`.
    /// `// C#: GCSViews/FlightPlanner.cs:3547-3560`
    pub fn from_esri(line: &str) -> Result<Self, ShapefileError> {
        let refuse = || {
            let name = parse_node(line).map_or_else(
                || line.trim().to_owned(),
                |(node, _)| node.name().to_owned(),
            );
            ShapefileError::Projection(name)
        };
        let (root, _) = parse_node(line).ok_or_else(refuse)?;
        if root.keyword.eq_ignore_ascii_case("GEOGCS") {
            return if is_wgs84(&root) {
                Ok(Self::Wgs84)
            } else {
                Err(refuse())
            };
        }
        if !root.keyword.eq_ignore_ascii_case("PROJCS") {
            return Err(refuse());
        }
        let transverse_mercator = root.child("PROJECTION").is_some_and(|projection| {
            projection
                .name()
                .eq_ignore_ascii_case("Transverse_Mercator")
        });
        let metres = root
            .child("UNIT")
            .and_then(|unit| unit.number(1))
            .is_some_and(|scale| scale == 1.0);
        if !transverse_mercator || !metres || !root.child("GEOGCS").is_some_and(is_wgs84) {
            return Err(refuse());
        }
        let parameter = |name: &str| root.parameter(name);
        let (Some(false_easting), Some(false_northing), Some(meridian), Some(scale), Some(origin)) = (
            parameter("False_Easting"),
            parameter("False_Northing"),
            parameter("Central_Meridian"),
            parameter("Scale_Factor"),
            parameter("Latitude_Of_Origin"),
        ) else {
            return Err(refuse());
        };
        // A UTM zone's central meridian is 6 * zone - 183.
        let zone = (meridian + 183.0) / 6.0;
        let north = false_northing == 0.0;
        let utm = false_easting == 500_000.0
            && (north || false_northing == 10_000_000.0)
            && scale == 0.9996
            && origin == 0.0
            && zone.fract() == 0.0
            && (1.0..=60.0).contains(&zone);
        if !utm {
            return Err(refuse());
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 1 to 60, checked
        let zone = zone as u8;
        Ok(Self::Utm { zone, north })
    }

    /// A coordinate of the file as latitude and longitude on WGS 1984: `Reproject.ReprojectPoints`
    /// from this system to `KnownCoordinateSystems.Geographic.World.WGS1984`.
    /// `// C#: GCSViews/FlightPlanner.cs:3570-3584`
    #[must_use]
    pub fn to_wgs84(self, x: f64, y: f64) -> Option<(f64, f64)> {
        match self {
            Self::Wgs84 => Some((y, x)),
            Self::Utm { zone, north } => {
                let zone = i32::from(zone);
                UtmPos::new(x, y, if north { zone } else { -zone }).to_lla()
            }
        }
    }
}

/// A feature's coordinates as the map takes them: `new PointLatLng(point.Y, point.X)` after the
/// reprojection, where there is a `.prj`. `None` for a coordinate that is not a position.
#[must_use]
pub fn position(projection: Option<Projection>, x: f64, y: f64) -> Option<LatLon> {
    let (lat, lng) = match projection {
        Some(projection) => projection.to_wgs84(x, y)?,
        None => (y, x),
    };
    LatLon::new(lat, lng).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN: &str = include_str!("../../../testdata/planner/golden/reproject.csv");

    /// A header and records built as ESRI's description lays them out.
    fn shp(records: &[Vec<u8>]) -> Vec<u8> {
        let mut body = Vec::new();
        for (index, content) in records.iter().enumerate() {
            let number = i32::try_from(index + 1).unwrap();
            body.extend_from_slice(&number.to_be_bytes());
            body.extend_from_slice(&i32::try_from(content.len() / 2).unwrap().to_be_bytes());
            body.extend_from_slice(content);
        }
        let mut out = vec![0u8; 100];
        out[0..4].copy_from_slice(&9994_i32.to_be_bytes());
        let words = i32::try_from((100 + body.len()) / 2).unwrap();
        out[24..28].copy_from_slice(&words.to_be_bytes());
        out[28..32].copy_from_slice(&1000_i32.to_le_bytes());
        out[32..36].copy_from_slice(&5_i32.to_le_bytes());
        out.extend(body);
        out
    }

    fn polygon(kind: i32, rings: &[&[(f64, f64)]]) -> Vec<u8> {
        let mut out = kind.to_le_bytes().to_vec();
        out.extend([0u8; 32]);
        let total: usize = rings.iter().map(|ring| ring.len()).sum();
        out.extend(i32::try_from(rings.len()).unwrap().to_le_bytes());
        out.extend(i32::try_from(total).unwrap().to_le_bytes());
        let mut start = 0;
        for ring in rings {
            out.extend(i32::try_from(start).unwrap().to_le_bytes());
            start += ring.len();
        }
        for ring in rings {
            for (x, y) in *ring {
                out.extend(x.to_le_bytes());
                out.extend(y.to_le_bytes());
            }
        }
        if kind == 15 {
            // Z range and values, which are not read.
            out.extend([0u8; 16]);
            out.extend(std::iter::repeat_n(0u8, 8 * total));
        }
        out
    }

    #[test]
    fn a_polygon_record_gives_its_ring_in_order() {
        let ring = [
            (149.0, -35.0),
            (149.1, -35.0),
            (149.1, -35.1),
            (149.0, -35.0),
        ];
        let file = shp(&[polygon(5, &[&ring])]);
        assert_eq!(features(&file).unwrap(), vec![ring.to_vec()]);
    }

    #[test]
    fn rings_and_records_follow_the_file_and_z_is_passed_over() {
        let a = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (1.0, 2.0)];
        let b = [(7.0, 8.0), (9.0, 10.0), (7.0, 8.0)];
        let file = shp(&[polygon(15, &[&a, &b]), vec![0, 0, 0, 0], polygon(5, &[&b])]);
        let got = features(&file).unwrap();
        assert_eq!(got.len(), 2, "the null record is passed over");
        assert_eq!(got[0], [a.as_slice(), b.as_slice()].concat());
        assert_eq!(got[1], b.to_vec());
    }

    #[test]
    fn a_file_that_is_not_a_shapefile_or_ends_early_is_refused() {
        assert_eq!(features(&[0u8; 50]), Err(ShapefileError::Truncated));
        let mut file = shp(&[]);
        file[3] = 1;
        assert!(matches!(
            features(&file),
            Err(ShapefileError::NotAShapefile(_))
        ));
        let mut cut = shp(&[polygon(5, &[&[(1.0, 2.0), (3.0, 4.0)]])]);
        cut.truncate(cut.len() - 4);
        let words = i32::try_from(cut.len() / 2 + 2).unwrap();
        cut[24..28].copy_from_slice(&words.to_be_bytes());
        assert_eq!(features(&cut), Err(ShapefileError::Truncated));
    }

    /// Every point of `reproject.csv`: DotSpatial's `ReprojectPoints` under mono, from the `.prj`
    /// the case names, to within a micrometre of arc - DotSpatial's transverse Mercator and
    /// ProjNet's are different series.
    #[test]
    fn a_projected_file_lands_where_dotspatial_puts_it() {
        let prj = |name: &str| match name {
            "utm55s" => include_str!("../../../testdata/planner/utm55s.prj"),
            "utm30n" => include_str!("../../../testdata/planner/utm30n.prj"),
            "wgs84" => include_str!("../../../testdata/planner/wgs84.prj"),
            other => panic!("no .prj for {other}"),
        };
        let mut checked = 0;
        for line in GOLDEN.lines() {
            let fields: Vec<&str> = line.split(',').collect();
            let number = |index: usize| fields[index].parse::<f64>().unwrap();
            let projection = Projection::from_esri(prj(fields[1]).lines().next().unwrap())
                .unwrap_or_else(|why| panic!("{}: {why}", fields[1]));
            let (lat, lng) = projection.to_wgs84(number(2), number(3)).unwrap();
            assert!((lat - number(4)).abs() < 1e-11, "{line}: {lat}");
            assert!((lng - number(5)).abs() < 1e-11, "{line}: {lng}");
            checked += 1;
        }
        assert_eq!(checked, 6);
    }

    #[test]
    fn a_projection_is_named_when_it_is_refused() {
        let mercator = "PROJCS[\"WGS_1984_Web_Mercator_Auxiliary_Sphere\",GEOGCS[\"GCS_WGS_1984\",DATUM[\"D_WGS_1984\",SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.0174532925199433]],PROJECTION[\"Mercator_Auxiliary_Sphere\"],PARAMETER[\"False_Easting\",0.0],PARAMETER[\"False_Northing\",0.0],PARAMETER[\"Central_Meridian\",0.0],PARAMETER[\"Standard_Parallel_1\",0.0],PARAMETER[\"Auxiliary_Sphere_Type\",0.0],UNIT[\"Meter\",1.0]]";
        assert_eq!(
            Projection::from_esri(mercator),
            Err(ShapefileError::Projection(
                "WGS_1984_Web_Mercator_Auxiliary_Sphere".to_owned()
            ))
        );
        let gda = "GEOGCS[\"GCS_GDA_1994\",DATUM[\"D_GDA_1994\",SPHEROID[\"GRS_1980\",6378137.0,298.257222101]],PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.0174532925199433]]";
        assert_eq!(
            Projection::from_esri(gda),
            Err(ShapefileError::Projection("GCS_GDA_1994".to_owned()))
        );
    }

    /// The committed field - its `.shp`, `.shx`, `.dbf` and `.prj` laid out as ESRI describes -
    /// reads as one ring of five points, the last closing it, in UTM 55S.
    #[test]
    fn the_committed_field_reads_as_one_closed_ring() {
        let shp = include_bytes!("../../../testdata/planner/field.shp");
        let got = features(shp).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].len(), 5);
        assert_eq!(got[0].first(), got[0].last());
        assert_eq!(got[0][0], (695_400.0, 6_084_100.0));
        let prj = include_str!("../../../testdata/planner/field.prj");
        assert_eq!(
            Projection::from_esri(prj.lines().next().unwrap()),
            Ok(Projection::Utm {
                zone: 55,
                north: false
            })
        );
    }

    #[test]
    fn vertices_are_one_flat_list_with_z_only_where_the_file_has_it() {
        // Three points in wp order 2, 1, 3; no Z in a Point file.
        let points =
            vertices(include_bytes!("../../../testdata/planner/points.shp")).expect("points");
        assert_eq!(points.xy.len(), 3);
        assert_eq!(points.xy[1], (149.16, -35.36));
        assert_eq!(points.z, None);
        // PointZ carries a Z per vertex.
        let pointz =
            vertices(include_bytes!("../../../testdata/planner/pointz.shp")).expect("pointz");
        assert_eq!(pointz.xy.len(), 2);
        assert_eq!(pointz.z, Some(vec![601.25, 602.75]));
        // A polygon's ring is its vertices in order: `Vertex[row * 2]` of row 1 is its second corner.
        let field = vertices(include_bytes!("../../../testdata/planner/field.shp")).expect("field");
        assert_eq!(field.xy.len(), 5);
        assert_eq!(field.z, None);
        assert_eq!(vertices(&[0u8; 50]), Err(ShapefileError::Truncated));
    }

    #[test]
    fn without_a_prj_the_coordinates_are_longitude_and_latitude() {
        assert_eq!(
            position(None, 149.165, -35.363),
            LatLon::new(-35.363, 149.165).ok()
        );
    }
}
