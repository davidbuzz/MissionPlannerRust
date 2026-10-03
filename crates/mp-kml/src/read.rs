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

//! Reading KML the way the planner's two loaders read it.
//!
//! Ported from `GCSViews/FlightPlanner.cs` @ efb0801 (GPL-3.0-only): `processKMLMission`
//! (Load KML File, `:4470-4525`) and `processKML` with `GetKMLLineColor` (KML Overlay,
//! `:4131-4290`). Both run over SharpKml's object tree; this module walks the XML with
//! `roxmltree` instead and hands back what each handler acted on, in the order it acted.
//!
//! **Coordinates are `longitude,latitude[,altitude]`**, as everywhere in KML, and each tuple's
//! altitude is optional (`CoordinateCollection.Parse`, `ExtLibs/SharpKml/Dom/Fields/
//! CoordinateCollection.cs:196-222`): a two-part tuple has none, which matters to Load KML File,
//! whose `(int) loc.Altitude` throws on it.
//!
//! Where the C# fails, this fails with the .NET exception's name and message, which is what
//! `CustomMessageBox.Show(Strings.Bad_KML_File + ex)` shows before the stack trace this port does
//! not have.

use std::fmt;

/// One tuple of a `<coordinates>` element.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coord {
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lon: f64,
    /// Altitude, metres, when the tuple had a third part.
    pub alt: Option<f64>,
}

/// What Load KML File acts on, in the order SharpKml's parser raised `ElementAdded` - an element
/// is added to its parent once it is parsed whole, so children come before their parents and the
/// leaf geometries come in document order.
/// `// C#: GCSViews/FlightPlanner.cs:4527-4577 processKMLMission`
#[derive(Debug, Clone, PartialEq)]
pub enum MissionEvent {
    /// A `Placemark` whose geometry is a `Point`: `POI.POIAdd(point, pm.Name)`. The name is
    /// `None` when the placemark has no `<name>`.
    Poi {
        /// The point.
        at: Coord,
        /// `pm.Name`.
        name: Option<String>,
    },
    /// A `LineString` anywhere in the file: a row per coordinate through `setfromMap`.
    Path(Vec<Coord>),
}

/// A polygon or route the KML Overlay draws: its points and the pen `GetKMLLineColor` chose.
#[derive(Debug, Clone, PartialEq)]
pub struct Shape {
    /// The outer boundary's ring, or the line's points.
    pub points: Vec<Coord>,
    /// The pen's colour as .NET's ARGB.
    pub argb: u32,
    /// The pen's width in pixels.
    pub width: i32,
}

/// A `Placemark` with a `Point`: `GMapMarkerKMLLabel` at the placemark's look-at with its name.
#[derive(Debug, Clone, PartialEq)]
pub struct Label {
    /// Where the label is.
    pub at: Coord,
    /// The text.
    pub text: String,
}

/// What `processKML` put on `kmlpolygonsoverlay`.
/// `// C#: GCSViews/FlightPlanner.cs:4290-4400 processKML`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Overlay {
    /// `GMapPolygon`s, transparent fill.
    pub polygons: Vec<Shape>,
    /// `GMapRoute`s.
    pub routes: Vec<Shape>,
    /// `GMapMarkerKMLLabel`s.
    pub labels: Vec<Label>,
    /// `GroundOverlay`s met, which are not drawn here (the C# drapes their image over the map;
    /// this port has no image draping yet).
    pub ground_overlays: usize,
}

/// Why a file could not be read as the C# reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadError {
    /// Not well-formed XML: SharpKml's parser throws `XmlException`.
    Xml(String),
    /// The root element is not `<kml>`: `parser.Root as Kml` is null and `rootnode.Feature`
    /// dereferences it.
    NoKmlRoot,
    /// A coordinate without an altitude where `(int) loc.Altitude` needs one.
    NoAltitude,
    /// A `styleUrl` with no matching style in the document (`.First()` on nothing), or a styled
    /// placemark with no `Document` to look in.
    NoStyle(String),
    /// A style map's target is itself not a `Style`: the cast fails.
    NotAStyle(String),
    /// A `Point` placemark without a name: `fontBitmaps[null]`.
    UnnamedPoint,
}

impl fmt::Display for ReadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Xml(why) => write!(f, "System.Xml.XmlException: {why}"),
            Self::NoKmlRoot => f.write_str(
                "System.NullReferenceException: Object reference not set to an instance of an object.",
            ),
            Self::NoAltitude => {
                f.write_str("System.InvalidOperationException: Nullable object must have a value.")
            }
            Self::NoStyle(url) => write!(
                f,
                "System.InvalidOperationException: Sequence contains no matching element (styleUrl {url})"
            ),
            Self::NotAStyle(url) => write!(
                f,
                "System.InvalidCastException: Unable to cast object of type 'SharpKml.Dom.StyleMapCollection' to type 'SharpKml.Dom.Style'. (styleUrl {url})"
            ),
            Self::UnnamedPoint => f.write_str(
                "System.ArgumentNullException: Value cannot be null.\nParameter name: key",
            ),
        }
    }
}

impl std::error::Error for ReadError {}

/// `kml.Replace("<Snippet/>", "")`, which both loaders do before parsing.
/// `// C#: GCSViews/FlightPlanner.cs:4237, 4514`
#[must_use]
pub fn without_snippets(kml: &str) -> String {
    kml.replace("<Snippet/>", "")
}

/// `CoordinateCollection.Parse`: every `lon,lat[,alt]` the regular expression finds, its parts
/// numbers separated by commas with whitespace allowed around them, tuples separated by
/// whitespace; a third part that is not a number leaves the tuple without an altitude, and text
/// that is not a number is passed over.
/// `// C#: ExtLibs/SharpKml/Dom/Fields/CoordinateCollection.cs:186-222`
#[must_use]
pub fn coordinates(text: &str) -> Vec<Coord> {
    #[derive(PartialEq)]
    enum Token {
        Number(f64),
        Comma,
        Other,
    }
    let tokens: Vec<Token> = text
        .split_whitespace()
        .flat_map(|word| {
            // "149.2," is a number then a comma; ",-35.3" a comma then a number.
            let mut out = Vec::new();
            for (index, part) in word.split(',').enumerate() {
                if index > 0 {
                    out.push(Token::Comma);
                }
                if !part.is_empty() {
                    out.push(part.parse::<f64>().map_or(Token::Other, Token::Number));
                }
            }
            out
        })
        .collect();
    let mut coords = Vec::new();
    let mut at = 0;
    while at < tokens.len() {
        let tuple = match (tokens.get(at), tokens.get(at + 1), tokens.get(at + 2)) {
            (Some(Token::Number(lon)), Some(Token::Comma), Some(Token::Number(lat))) => {
                let alt = match (tokens.get(at + 3), tokens.get(at + 4)) {
                    (Some(Token::Comma), Some(Token::Number(alt))) => Some(*alt),
                    _ => None,
                };
                Some(Coord {
                    lat: *lat,
                    lon: *lon,
                    alt,
                })
            }
            _ => None,
        };
        match tuple {
            Some(coord) => {
                at += if coord.alt.is_some() { 5 } else { 3 };
                coords.push(coord);
            }
            None => at += 1,
        }
    }
    coords
}

type Node<'a, 'i> = roxmltree::Node<'a, 'i>;

/// The element's KML name, namespace dropped.
fn name<'a>(node: Node<'a, '_>) -> &'a str {
    node.tag_name().name()
}

/// The element's child elements.
fn elements<'a, 'i>(node: Node<'a, 'i>) -> impl Iterator<Item = Node<'a, 'i>> {
    node.children().filter(roxmltree::Node::is_element)
}

/// The first child element called `tag`.
fn child<'a, 'i>(node: Node<'a, 'i>, tag: &str) -> Option<Node<'a, 'i>> {
    elements(node).find(|child| name(*child) == tag)
}

/// The text of the first child element called `tag`.
fn child_text<'a>(node: Node<'a, '_>, tag: &str) -> Option<&'a str> {
    child(node, tag).and_then(|child| child.text())
}

/// The geometry element of a placemark: its first child that is one.
fn geometry<'a, 'i>(placemark: Node<'a, 'i>) -> Option<Node<'a, 'i>> {
    elements(placemark).find(|child| {
        matches!(
            name(*child),
            "Point" | "LineString" | "Polygon" | "MultiGeometry" | "LinearRing" | "Model"
        )
    })
}

/// The `<coordinates>` of a geometry.
fn geometry_coordinates(node: Node<'_, '_>) -> Vec<Coord> {
    child_text(node, "coordinates").map_or_else(Vec::new, coordinates)
}

fn parse(kml: &str) -> Result<roxmltree::Document<'_>, ReadError> {
    roxmltree::Document::parse(kml).map_err(|err| ReadError::Xml(err.to_string()))
}

/// Load KML File's walk: every `LineString`'s coordinates as a path and every `Point` placemark
/// as a POI, in the order the C#'s `ElementAdded` handler saw them.
///
/// # Errors
///
/// [`ReadError::Xml`] when the text is not XML. Nothing else fails here: a path's coordinates
/// without altitudes are handed back as they are, and the caller fails on them as
/// `(int) loc.Altitude` does, after the rows before them are in.
/// `// C#: GCSViews/FlightPlanner.cs:4527-4577`
pub fn mission_events(kml: &str) -> Result<Vec<MissionEvent>, ReadError> {
    let document = parse(kml)?;
    let mut events = Vec::new();
    collect_mission_events(document.root_element(), &mut events);
    Ok(events)
}

/// Post-order: children before their parent, as SharpKml adds a parsed child to its parent.
fn collect_mission_events(node: Node<'_, '_>, events: &mut Vec<MissionEvent>) {
    for child in elements(node) {
        collect_mission_events(child, events);
    }
    match name(node) {
        "LineString" => events.push(MissionEvent::Path(geometry_coordinates(node))),
        "Placemark" => {
            if let Some(point) = geometry(node).filter(|geometry| name(*geometry) == "Point")
                && let Some(at) = geometry_coordinates(point).first().copied()
            {
                events.push(MissionEvent::Poi {
                    at,
                    name: child_text(node, "name").map(str::to_owned),
                });
            }
        }
        _ => {}
    }
}

/// `GetKMLLineColor(styleurl, root)`: white and 2 for no style URL; else the style the URL names
/// **must** exist, and only a `StyleMap` whose first pair names a `Style` with a `LineStyle` gives
/// a colour and width - a plain `Style`, or a mapped one without a line, is white and 2 again.
/// The colour is KML's `aabbggrr` read as ABGR and swapped to ARGB; the width is `(int)` of the
/// style's.
/// `// C#: GCSViews/FlightPlanner.cs:4487-4525`
fn line_colour(
    style_url: Option<&str>,
    root: Option<Node<'_, '_>>,
) -> Result<(u32, i32), ReadError> {
    const WHITE: (u32, i32) = (0xFF_FF_FF_FF, 2);
    let Some(url) = style_url.filter(|url| !url.is_empty()) else {
        return Ok(WHITE);
    };
    // `root.Styles`: a null root dereferenced.
    let root = root.ok_or_else(|| ReadError::NoStyle(url.to_owned()))?;
    let id = url.trim_start_matches('#');
    let style = find_style(root, id).ok_or_else(|| ReadError::NoStyle(url.to_owned()))?;
    if name(style) != "StyleMap" {
        return Ok(WHITE);
    }
    let Some(target_url) = child(style, "Pair").and_then(|pair| child_text(pair, "styleUrl"))
    else {
        // `((StyleMapCollection) style2).First().StyleUrl` on a map without pairs throws on
        // `First()`; one whose pair has no URL dereferences null at `.OriginalString`.
        return Err(ReadError::NoStyle(url.to_owned()));
    };
    let target_id = target_url.trim_start_matches('#');
    let target = find_style(root, target_id).ok_or_else(|| ReadError::NoStyle(url.to_owned()))?;
    if name(target) != "Style" {
        return Err(ReadError::NotAStyle(url.to_owned()));
    }
    let Some(line) = child(target, "LineStyle") else {
        return Ok(WHITE);
    };
    let argb = child_text(line, "color")
        .and_then(|text| u32::from_str_radix(text.trim(), 16).ok())
        .map_or(WHITE.0, |abgr| {
            (abgr & 0xFF00_FF00) | ((abgr & 0x00FF_0000) >> 16) | ((abgr & 0x0000_00FF) << 16)
        });
    #[allow(clippy::cast_possible_truncation)] // `(int) Line.Width.Value`
    let width = child_text(line, "width")
        .and_then(|text| text.trim().parse::<f64>().ok())
        .map_or(WHITE.1, |width| width as i32);
    Ok((argb, width))
}

/// `root.Styles.Where(a => a.Id == id).First()`: the document's own `Style` and `StyleMap`
/// children with that id, the first of them.
fn find_style<'a, 'i>(root: Node<'a, 'i>, id: &str) -> Option<Node<'a, 'i>> {
    elements(root)
        .filter(|node| matches!(name(*node), "Style" | "StyleMap"))
        .find(|node| node.attribute("id") == Some(id))
}

/// KML Overlay's walk over `rootnode.Feature`: polygons and line strings with their pens, point
/// placemarks as labels, ground overlays counted.
///
/// # Errors
///
/// As the C#'s `catch` is reached: not XML, no `<kml>` root, a style URL naming nothing (or with
/// no `Document` to look in), a style map whose target is not a style, a point placemark with
/// no name.
/// `// C#: GCSViews/FlightPlanner.cs:4131-4290, 4290-4400`
pub fn overlay(kml: &str) -> Result<Overlay, ReadError> {
    let document = parse(kml)?;
    let root = document.root_element();
    if name(root) != "kml" {
        return Err(ReadError::NoKmlRoot);
    }
    let mut out = Overlay::default();
    // `rootnode.Feature`: the one feature under `<kml>`; none is a null passed to `processKML`,
    // which matches no branch and does nothing.
    let Some(feature) = elements(root).find(|node| {
        matches!(
            name(*node),
            "Document"
                | "Folder"
                | "Placemark"
                | "GroundOverlay"
                | "NetworkLink"
                | "PhotoOverlay"
                | "ScreenOverlay"
        )
    }) else {
        return Ok(out);
    };
    process(feature, None, None, &mut out)?;
    Ok(out)
}

/// `processKML(Element, root)`. `style_url` carries a `MultiGeometry` placemark's URL down to
/// each of its geometries, as the C# wraps each in a new placemark with the same `StyleUrl`.
fn process(
    node: Node<'_, '_>,
    root: Option<Node<'_, '_>>,
    style_url: Option<&str>,
    out: &mut Overlay,
) -> Result<(), ReadError> {
    match name(node) {
        // A document becomes the root its features' styles are looked up in.
        "Document" => {
            for feature in elements(node) {
                process(feature, Some(node), None, out)?;
            }
        }
        "Folder" => {
            for feature in elements(node) {
                process(feature, root, None, out)?;
            }
        }
        "Placemark" => {
            let url = child_text(node, "styleUrl").map(str::trim);
            if let Some(geometry) = geometry(node) {
                process_geometry(geometry, node, root, url, out)?;
            }
        }
        // A geometry handed down from a `MultiGeometry`, standing in for its wrapping placemark.
        "Polygon" | "LineString" | "MultiGeometry" | "Point" => {
            process_geometry(node, node, root, style_url, out)?;
        }
        "GroundOverlay" => out.ground_overlays += 1,
        _ => {}
    }
    Ok(())
}

fn process_geometry(
    geometry: Node<'_, '_>,
    placemark: Node<'_, '_>,
    root: Option<Node<'_, '_>>,
    style_url: Option<&str>,
    out: &mut Overlay,
) -> Result<(), ReadError> {
    match name(geometry) {
        "Polygon" => {
            let (argb, width) = line_colour(style_url, root)?;
            let ring = child(geometry, "outerBoundaryIs")
                .and_then(|outer| child(outer, "LinearRing"))
                .map_or_else(Vec::new, geometry_coordinates);
            out.polygons.push(Shape {
                points: ring,
                argb,
                width,
            });
        }
        "LineString" => {
            let (argb, width) = line_colour(style_url, root)?;
            out.routes.push(Shape {
                points: geometry_coordinates(geometry),
                argb,
                width,
            });
        }
        "MultiGeometry" => {
            for part in elements(geometry) {
                process(part, root, style_url, out)?;
            }
        }
        "Point" => {
            // "its a label": `placemark.CalculateLookAt()` of a point is the point.
            let Some(at) = geometry_coordinates(geometry).first().copied() else {
                return Ok(());
            };
            let text = child_text(placemark, "name").ok_or(ReadError::UnnamedPoint)?;
            out.labels.push(Label {
                at,
                text: text.to_owned(),
            });
        }
        _ => {}
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROUTE: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <name>route</name>
  <Snippet/>
  <Style id="line-red">
    <LineStyle><color>ff0000ff</color><width>3.7</width></LineStyle>
  </Style>
  <StyleMap id="map-red">
    <Pair><key>normal</key><styleUrl>#line-red</styleUrl></Pair>
    <Pair><key>highlight</key><styleUrl>#line-red</styleUrl></Pair>
  </StyleMap>
  <Style id="plain"><LineStyle><color>ff00ff00</color></LineStyle></Style>
  <Folder>
    <Placemark>
      <name>Field</name>
      <styleUrl>#map-red</styleUrl>
      <Polygon><outerBoundaryIs><LinearRing><coordinates>
        149.1,-35.3,0 149.2,-35.3,0 149.2,-35.4,0 149.1,-35.3,0
      </coordinates></LinearRing></outerBoundaryIs></Polygon>
    </Placemark>
    <Placemark>
      <name>Leg</name>
      <styleUrl>#plain</styleUrl>
      <LineString><coordinates>149.10,-35.30,50 149.15,-35.35,60.9</coordinates></LineString>
    </Placemark>
    <Placemark>
      <name>Gate</name>
      <Point><coordinates>149.12,-35.32</coordinates></Point>
    </Placemark>
  </Folder>
</Document>
</kml>"#;

    #[test]
    fn coordinates_are_lon_lat_and_an_optional_alt() {
        let parsed = coordinates(" 149.1,-35.3,0\n149.2, -35.3 149.3,-35.3,abc bad,tuple ");
        assert_eq!(
            parsed,
            vec![
                Coord {
                    lat: -35.3,
                    lon: 149.1,
                    alt: Some(0.0)
                },
                Coord {
                    lat: -35.3,
                    lon: 149.2,
                    alt: None
                },
                Coord {
                    lat: -35.3,
                    lon: 149.3,
                    alt: None
                },
            ]
        );
    }

    #[test]
    fn load_kml_file_sees_paths_and_point_placemarks_in_document_order() {
        let events = mission_events(&without_snippets(ROUTE)).expect("XML");
        assert_eq!(events.len(), 2, "a polygon adds nothing: {events:?}");
        assert_eq!(
            events[0],
            MissionEvent::Path(vec![
                Coord {
                    lat: -35.30,
                    lon: 149.10,
                    alt: Some(50.0)
                },
                Coord {
                    lat: -35.35,
                    lon: 149.15,
                    alt: Some(60.9)
                },
            ])
        );
        assert_eq!(
            events[1],
            MissionEvent::Poi {
                at: Coord {
                    lat: -35.32,
                    lon: 149.12,
                    alt: None
                },
                name: Some("Gate".to_owned()),
            }
        );
    }

    #[test]
    fn a_placemark_without_a_name_is_a_poi_with_none() {
        let kml = "<kml><Placemark><Point><coordinates>1,2</coordinates></Point></Placemark></kml>";
        assert_eq!(
            mission_events(kml).expect("XML"),
            vec![MissionEvent::Poi {
                at: Coord {
                    lat: 2.0,
                    lon: 1.0,
                    alt: None
                },
                name: None,
            }]
        );
    }

    #[test]
    fn the_overlay_takes_its_pens_only_through_a_style_map() {
        let overlay = overlay(&without_snippets(ROUTE)).expect("a KML overlay");
        assert_eq!(overlay.polygons.len(), 1);
        // ff0000ff is ABGR: alpha ff, blue 00, green 00, red ff - ARGB red, width (int) 3.7.
        assert_eq!(overlay.polygons[0].argb, 0xFF_FF_00_00);
        assert_eq!(overlay.polygons[0].width, 3);
        assert_eq!(overlay.polygons[0].points.len(), 4);
        // A plain Style, even with a LineStyle, is white and 2: only a StyleMap is followed.
        assert_eq!(overlay.routes.len(), 1);
        assert_eq!(overlay.routes[0].argb, 0xFF_FF_FF_FF);
        assert_eq!(overlay.routes[0].width, 2);
        assert_eq!(
            overlay.labels,
            vec![Label {
                at: Coord {
                    lat: -35.32,
                    lon: 149.12,
                    alt: None
                },
                text: "Gate".to_owned()
            }]
        );
        assert_eq!(overlay.ground_overlays, 0);
    }

    #[test]
    fn a_style_url_naming_nothing_is_a_bad_kml_file() {
        let kml = r"<kml><Document><Placemark><styleUrl>#nope</styleUrl>
            <LineString><coordinates>1,2 3,4</coordinates></LineString></Placemark></Document></kml>";
        assert_eq!(overlay(kml), Err(ReadError::NoStyle("#nope".to_owned())));
        // With no Document there is no root to look in either.
        let kml = r"<kml><Folder><Placemark><styleUrl>#x</styleUrl>
            <LineString><coordinates>1,2 3,4</coordinates></LineString></Placemark></Folder></kml>";
        assert_eq!(overlay(kml), Err(ReadError::NoStyle("#x".to_owned())));
        // No style URL at all needs no Document.
        let kml = r"<kml><Folder><Placemark>
            <LineString><coordinates>1,2 3,4</coordinates></LineString></Placemark></Folder></kml>";
        assert_eq!(overlay(kml).map(|o| o.routes.len()), Ok(1));
    }

    #[test]
    fn a_multi_geometry_hands_its_style_to_each_part_and_a_nameless_point_fails() {
        let kml = r#"<kml><Document><StyleMap id="m"><Pair><styleUrl>#s</styleUrl></Pair></StyleMap>
            <Style id="s"><LineStyle><color>80ff8000</color><width>1</width></LineStyle></Style>
            <Placemark><styleUrl>#m</styleUrl><MultiGeometry>
              <LineString><coordinates>1,2 3,4</coordinates></LineString>
              <Polygon><outerBoundaryIs><LinearRing><coordinates>1,2 3,4 5,6 1,2</coordinates></LinearRing></outerBoundaryIs></Polygon>
            </MultiGeometry></Placemark>
            <GroundOverlay><Icon><href>x.png</href></Icon></GroundOverlay>
            </Document></kml>"#;
        let overlay = super::overlay(kml).expect("an overlay");
        // 80ff8000: alpha 80, blue ff, green 80, red 00 -> ARGB 0x80_00_80_ff.
        assert_eq!(overlay.routes[0].argb, 0x80_00_80_FF);
        assert_eq!(overlay.polygons[0].argb, 0x80_00_80_FF);
        assert_eq!(overlay.routes[0].width, 1);
        assert_eq!(overlay.ground_overlays, 1);

        let kml = "<kml><Document><Placemark><Point><coordinates>1,2</coordinates></Point></Placemark></Document></kml>";
        assert_eq!(super::overlay(kml), Err(ReadError::UnnamedPoint));
    }

    #[test]
    fn a_root_that_is_not_kml_or_not_xml_is_a_bad_kml_file() {
        assert_eq!(overlay("<Document/>"), Err(ReadError::NoKmlRoot));
        assert!(matches!(overlay("<kml>"), Err(ReadError::Xml(_))));
        assert!(matches!(mission_events("nope"), Err(ReadError::Xml(_))));
    }
}
