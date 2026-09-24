//! `location.kml`: the SharpKml document `CreateReportFiles` builds - a "Pins" folder with the
//! vehicle's path and a placemark per photo, an "Overlay" folder with a ground overlay per photo -
//! as SharpKml's `Serializer.Serialize` writes it.
//!
//! SharpKml writes an element's fields in the order its `KmlElement` attributes give and skips
//! those not set; the orders below are those, for the fields `CreateReportFiles` sets. Text with
//! any of `&'<>"` goes in a CDATA section, anything else is escaped as XML text (`Serializer.cs:
//! 170-190`). Numbers are `KmlFormatter`'s `#0.##############` and times its
//! `yyyy-MM-ddTHH:mm:sszzzzzz`.
//! `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1239-1246, 1414-1503, 1534-1536;
//! ExtLibs/SharpKml/Base/Serializer.cs`

use crate::numfmt::kml_double;
use crate::time::DateTime;
use crate::xml::IndentWriter;

/// `http://www.opengis.net/kml/2.2`. `// C#: ExtLibs/SharpKml/Base/KmlNamespaces.cs`
pub const KML22: &str = "http://www.opengis.net/kml/2.2";

/// A photo's placemark and ground overlay.
#[derive(Debug, Clone, PartialEq)]
pub struct Photo {
    /// `Path.GetFileNameWithoutExtension`: the placemark's and overlay's name.
    pub name: String,
    /// `Path.GetFileName(...).ToLower()`: what the description's image and the overlay's icon
    /// point at.
    pub file_lower: String,
    /// `picInfo.Time`.
    pub time: DateTime,
    /// The placemark's position: longitude, latitude, altitude.
    pub lon: f64,
    /// Latitude.
    pub lat: f64,
    /// `getAltitude(true, usegpsalt)`.
    pub alt: f64,
    /// The overlay's box: north, south, east, west (each a `float` widened) and rotation.
    pub north: f64,
    /// South.
    pub south: f64,
    /// East.
    pub east: f64,
    /// West.
    pub west: f64,
    /// `-alpha % 360`.
    pub rotation: f64,
}

/// `Serializer.WriteData`: nothing for empty text, CDATA for text holding markup characters that
/// is not already CDATA, escaped text otherwise. `// C#: ExtLibs/SharpKml/Base/Serializer.cs:170-190`
fn data(w: &mut IndentWriter, text: &str) {
    if text.is_empty() {
        return;
    }
    if !text.contains("<![CDATA[") && text.contains(['&', '\'', '<', '>', '"']) {
        w.cdata(text);
    } else {
        w.string(text);
    }
}

fn leaf(w: &mut IndentWriter, name: &str, text: &str) {
    w.start_element(name, None);
    data(w, text);
    w.end_element();
}

/// `CoordinateCollection`'s inner text: `lon,lat,alt` and a `\n` per point.
/// `// C#: ExtLibs/SharpKml/Dom/Fields/CoordinateCollection.cs:133-161`
fn coordinates(w: &mut IndentWriter, points: &[(f64, f64, f64)]) {
    let mut text = String::new();
    for &(lon, lat, alt) in points {
        text.push_str(&format!(
            "{},{},{}\n",
            kml_double(lon),
            kml_double(lat),
            kml_double(alt)
        ));
    }
    leaf(w, "coordinates", &text);
}

/// The document: `path` is the vehicle's positions as longitude, latitude, altitude.
#[must_use]
pub fn document(path: &[(f64, f64, f64)], photos: &[Photo]) -> String {
    let mut w = IndentWriter::new();
    w.start_element("Document", Some(KML22));

    w.start_element("Folder", None);
    leaf(&mut w, "name", "Pins");
    // The path placemark: name, then its LineString.
    w.start_element("Placemark", None);
    leaf(&mut w, "name", "path");
    w.start_element("LineString", None);
    leaf(&mut w, "altitudeMode", "absolute");
    coordinates(&mut w, path);
    w.end_element();
    w.end_element();
    for photo in photos {
        w.start_element("Placemark", None);
        leaf(&mut w, "name", &photo.name);
        leaf(&mut w, "visibility", "true");
        leaf(
            &mut w,
            "description",
            &format!(
                "<table><tr><td><img src=\"{}\" width=500 /></td></tr></table>",
                photo.file_lower
            ),
        );
        w.start_element("TimeStamp", None);
        leaf(&mut w, "when", &photo.time.format_kml());
        w.end_element();
        w.start_element("Style", None);
        w.start_element("BalloonStyle", None);
        leaf(&mut w, "text", "$[name]<br>$[description]");
        w.end_element();
        w.end_element();
        w.start_element("Point", None);
        leaf(&mut w, "altitudeMode", "absolute");
        coordinates(&mut w, &[(photo.lon, photo.lat, photo.alt)]);
        w.end_element();
        w.end_element();
    }
    w.end_element();

    w.start_element("Folder", None);
    leaf(&mut w, "name", "Overlay");
    for photo in photos {
        w.start_element("GroundOverlay", None);
        leaf(&mut w, "name", &photo.name);
        leaf(&mut w, "visibility", "false");
        w.start_element("TimeStamp", None);
        leaf(&mut w, "when", &photo.time.format_kml());
        w.end_element();
        w.start_element("Icon", None);
        leaf(&mut w, "href", &photo.file_lower);
        w.end_element();
        leaf(&mut w, "altitudeMode", "clampToGround");
        w.start_element("LatLonBox", None);
        leaf(&mut w, "north", &kml_double(photo.north));
        leaf(&mut w, "south", &kml_double(photo.south));
        leaf(&mut w, "east", &kml_double(photo.east));
        leaf(&mut w, "west", &kml_double(photo.west));
        leaf(&mut w, "rotation", &kml_double(photo.rotation));
        w.end_element();
        w.end_element();
    }
    w.end_element();

    w.end_element();
    w.finish()
}
