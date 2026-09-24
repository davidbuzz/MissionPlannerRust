//! Writing missions and flown paths as KML.
//!
//! Ported from `Log/MavlinkLogBase.cs:132-330` and `GCSViews/FlightPlanner.cs` @ efb0801
//! (GPL-3.0-or-later).
//!
//! This is how a flight is handed to somebody without a ground station: a `.kml` opens in Google
//! Earth, in QGIS, and in every GIS tool anyone is likely to have. Mission Planner writes one
//! beside every log it converts.
//!
//! Written by hand rather than through a KML crate. The document this produces is a few element
//! types deep and the escaping rules are XML's, so a writer is shorter than the integration would
//! be - and the one thing that has to be right is not something a crate would save us from.
//!
//! **Coordinates are `longitude,latitude,altitude`.** Every other interface in this application
//! takes latitude first, every person says "lat, long", and KML reverses it. Getting it backwards
//! produces a file that opens without complaint and puts an Australian flight in Kazakhstan. It is
//! the only thing about this format worth being careful about, and there is a test named after it.

use mp_mission::MissionItem;
use mp_units::LatLon;

pub mod dflog;
pub mod read;

/// The colours Mission Planner cycles flight-path segments through.
///
/// `// C#: Log/MavlinkLogBase.cs:136-140` - red, orange, yellow, green, blue, indigo, violet, pink.
/// KML colours are `aabbggrr`, which is ABGR and not the RGBA anybody expects.
const SEGMENT_COLOURS: &[&str] = &[
    "ff0000ff", // red
    "ff00a5ff", // orange
    "ff00ffff", // yellow
    "ff008000", // green
    "ffff0000", // blue
    "ff82004b", // indigo
    "ffee82ee", // violet
    "ffcbc0ff", // pink
];

/// How many colours the C# actually reaches.
///
/// **Seven, not eight.** The index is `c % (colours.Length - 1)`, so pink is in the array and
/// never selected. Reproduced so a flight exported by either program is coloured the same way; it
/// is the sort of thing that looks like a bug in the port when it is a bug in the original.
/// `// C#: Log/MavlinkLogBase.cs:239`
const COLOURS_USED: usize = SEGMENT_COLOURS.len() - 1;

/// One sample of a flown path.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackPoint {
    /// Where the vehicle was.
    pub position: LatLon,
    /// Altitude above mean sea level, in metres. `// C#: uses cs.altasl`
    pub altitude_msl: f64,
    /// Seconds since the start of the recording, for the segment time spans.
    pub seconds: f64,
    /// The flight mode at this sample. A change starts a new coloured segment.
    pub mode: String,
}

/// Escapes text for an XML body.
///
/// Mission and mode names reach this from files and from firmware, so neither is trusted to be
/// free of `&` or `<`. An unescaped one produces a `.kml` that will not open at all, which at
/// least fails loudly - but it fails after the flight, when the file is being handed over.
fn escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for character in text.chars() {
        match character {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            // A raw control character is not valid XML at all, and a NUL from a fixed-length
            // firmware string is the likely source.
            c if (c as u32) < 0x20 && c != '\n' && c != '\t' => out.push(' '),
            c => out.push(c),
        }
    }
    out
}

/// One `lon,lat,alt` triple, in KML's order.
fn coordinate(position: LatLon, altitude: f64) -> String {
    // Six decimal places of degrees is about 0.1 m, which is finer than any of this is known to.
    format!(
        "{:.7},{:.7},{:.2}",
        position.longitude(),
        position.latitude(),
        altitude
    )
}

/// Writes a flown path: one coloured `LineString` per flight mode.
///
/// `// C#: Log/MavlinkLogBase.cs:171-250`
#[must_use]
pub fn flight_path(name: &str, track: &[TrackPoint]) -> String {
    let mut out = String::new();
    out.push_str(&header(name));
    out.push_str(LINE_STYLE);

    // Consecutive samples in the same mode are one segment, and every sample belongs to exactly
    // one of them.
    //
    // The C# condition is `mode != cs.mode || flightdata.Count == a`, and it adds the current
    // point to the *next* segment's coordinates after emitting - so when the final sample is also
    // a mode change, the segment it starts is built and never written. The last mode of a flight
    // is usually RTL or LAND, which is the part somebody reviewing a crash wants. Not reproduced:
    // a file that silently omits the end of the flight is worse than one that differs from the
    // original by including it.
    let mut segment = 0usize;
    let mut start = 0usize;
    while start < track.len() {
        let mode = track
            .get(start)
            .map(|point| point.mode.as_str())
            .unwrap_or("");
        let mut end = start + 1;
        while track
            .get(end)
            .is_some_and(|point| point.mode.as_str() == mode)
        {
            end += 1;
        }
        if let Some(points) = track.get(start..end) {
            out.push_str(&path_segment(segment, points));
            segment += 1;
        }
        start = end;
    }

    out.push_str(FOOTER);
    out
}

/// One coloured segment of the path.
fn path_segment(index: usize, points: &[TrackPoint]) -> String {
    let mode = points.first().map_or("", |p| p.mode.as_str());
    let colour = SEGMENT_COLOURS
        .get(index % COLOURS_USED)
        .copied()
        .unwrap_or("ffffffff");
    let coordinates: Vec<String> = points
        .iter()
        .map(|point| coordinate(point.position, point.altitude_msl))
        .collect();
    let begin = points.first().map_or(0.0, |p| p.seconds);
    let end = points.last().map_or(0.0, |p| p.seconds);

    format!(
        "    <Placemark>\n\
         \x20     <name>{index} Flight Path {mode}</name>\n\
         \x20     <description>{begin:.1}s to {end:.1}s</description>\n\
         \x20     <Style><LineStyle><color>{colour}</color><width>4</width></LineStyle></Style>\n\
         \x20     <LineString>\n\
         \x20       <extrude>1</extrude>\n\
         \x20       <altitudeMode>absolute</altitudeMode>\n\
         \x20       <coordinates>{}</coordinates>\n\
         \x20     </LineString>\n\
         \x20   </Placemark>\n",
        coordinates.join(" "),
        mode = escape(mode),
    )
}

/// Writes a mission: a line through the waypoints, and a placemark for each.
#[must_use]
pub fn mission(name: &str, items: &[MissionItem]) -> String {
    let mut out = String::new();
    out.push_str(&header(name));
    out.push_str(LINE_STYLE);

    // Only the items that are somewhere. A DO_ command has no position and drawing it at (0,0)
    // puts a leg through the Gulf of Guinea, which is how a mission ends up looking wrong in a
    // file somebody else opened.
    let located: Vec<&MissionItem> = items
        .iter()
        .filter(|item| item.x != 0.0 || item.y != 0.0)
        .collect();

    for (index, item) in located.iter().enumerate() {
        let position = LatLon::new(item.x, item.y).unwrap_or_default();
        out.push_str(&format!(
            "    <Placemark>\n\
             \x20     <name>{}</name>\n\
             \x20     <description>{}</description>\n\
             \x20     <Point>\n\
             \x20       <altitudeMode>relativeToGround</altitudeMode>\n\
             \x20       <coordinates>{}</coordinates>\n\
             \x20     </Point>\n\
             \x20   </Placemark>\n",
            index + 1,
            escape(&format!("command {} at {} m", item.command, item.z)),
            coordinate(position, item.z),
        ));
    }

    if located.len() > 1 {
        let coordinates: Vec<String> = located
            .iter()
            .map(|item| coordinate(LatLon::new(item.x, item.y).unwrap_or_default(), item.z))
            .collect();
        out.push_str(&format!(
            "    <Placemark>\n\
             \x20     <name>Mission</name>\n\
             \x20     <styleUrl>#missionLine</styleUrl>\n\
             \x20     <LineString>\n\
             \x20       <tessellate>1</tessellate>\n\
             \x20       <altitudeMode>relativeToGround</altitudeMode>\n\
             \x20       <coordinates>{}</coordinates>\n\
             \x20     </LineString>\n\
             \x20   </Placemark>\n",
            coordinates.join(" ")
        ));
    }

    out.push_str(FOOTER);
    out
}

/// The document opening, with the name the file will show in the places list.
fn header(name: &str) -> String {
    format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <kml xmlns=\"http://www.opengis.net/kml/2.2\">\n\
         \x20 <Document>\n\
         \x20   <name>{}</name>\n",
        escape(name)
    )
}

/// The shared line style, as the C# defines it. `// C#: Log/MavlinkLogBase.cs:146-155`
const LINE_STYLE: &str = "    <Style id=\"missionLine\">\n\
                          \x20     <LineStyle><color>7f00ffff</color><width>4</width></LineStyle>\n\
                          \x20     <PolyStyle><color>7f00ff00</color></PolyStyle>\n\
                          \x20   </Style>\n";

/// The document closing.
const FOOTER: &str = "  </Document>\n</kml>\n";

#[cfg(test)]
mod tests {
    use super::*;

    fn point(lat: f64, lon: f64, seconds: f64, mode: &str) -> TrackPoint {
        TrackPoint {
            position: LatLon::new(lat, lon).expect("a valid position"),
            altitude_msl: 100.0,
            seconds,
            mode: mode.to_owned(),
        }
    }

    /// The one thing about this format that is worth being careful about.
    ///
    /// KML is `longitude,latitude,altitude`. Every other interface here is latitude first, and
    /// reversing them produces a file that opens without complaint and puts a Canberra flight in
    /// Kazakhstan.
    #[test]
    fn coordinates_are_longitude_first_as_kml_requires_and_nothing_else_does() {
        let canberra = LatLon::new(-35.363262, 149.165237).expect("valid");
        let text = coordinate(canberra, 584.0);
        assert_eq!(text, "149.1652370,-35.3632620,584.00");
        // Spelled out: the first number is the one near 149, which is the longitude.
        let first: f64 = text
            .split(',')
            .next()
            .and_then(|s| s.parse().ok())
            .expect("a number");
        assert!(
            (first - 149.165_237).abs() < 1e-6,
            "longitude must come first, got {first}"
        );
    }

    /// A new segment starts at every mode change, and the last one is closed.
    ///
    /// Including when the last sample is itself the change: the C# builds that segment and never
    /// writes it, which drops the end of the flight - usually the RTL or the landing, which is the
    /// part somebody reviewing a crash is looking for.
    #[test]
    fn the_path_is_split_at_every_mode_change() {
        let track = vec![
            point(-35.0, 149.0, 0.0, "STABILIZE"),
            point(-35.1, 149.1, 1.0, "STABILIZE"),
            point(-35.2, 149.2, 2.0, "AUTO"),
            point(-35.3, 149.3, 3.0, "AUTO"),
            point(-35.4, 149.4, 4.0, "RTL"),
        ];
        let kml = flight_path("flight", &track);
        assert_eq!(kml.matches("<LineString>").count(), 3);
        assert!(kml.contains("0 Flight Path STABILIZE"));
        assert!(kml.contains("1 Flight Path AUTO"));
        assert!(kml.contains("2 Flight Path RTL"));
    }

    /// A single-mode flight is one segment, not none.
    #[test]
    fn a_flight_that_never_changes_mode_still_produces_a_path() {
        let track = vec![
            point(-35.0, 149.0, 0.0, "LOITER"),
            point(-35.1, 149.1, 1.0, "LOITER"),
        ];
        let kml = flight_path("flight", &track);
        assert_eq!(kml.matches("<LineString>").count(), 1);
        assert_eq!(kml.matches(',').count() / 2, 2, "both points are in it");
    }

    /// An empty track must produce a valid document rather than a panic or half a file.
    #[test]
    fn an_empty_track_is_still_a_valid_document() {
        let kml = flight_path("nothing", &[]);
        assert!(kml.starts_with("<?xml"));
        assert!(kml.trim_end().ends_with("</kml>"));
        assert!(!kml.contains("<LineString>"));
    }

    /// Pink is in the table and the C# never reaches it.
    #[test]
    fn the_colour_cycle_matches_the_c_sharps_off_by_one() {
        assert_eq!(SEGMENT_COLOURS.len(), 8);
        assert_eq!(COLOURS_USED, 7);
        // Eight segments, and the eighth wraps to the first colour rather than reaching pink.
        let used: Vec<&str> = (0..8).map(|c| SEGMENT_COLOURS[c % COLOURS_USED]).collect();
        assert_eq!(used[0], used[7], "the cycle is seven long");
        assert!(
            !used.contains(&"ffcbc0ff"),
            "pink is in the array and the C# never selects it"
        );
    }

    /// Altitudes on a flight path are above sea level and said to be.
    #[test]
    fn the_flight_path_altitude_mode_is_absolute() {
        let kml = flight_path("f", &[point(-35.0, 149.0, 0.0, "AUTO")]);
        assert!(kml.contains("<altitudeMode>absolute</altitudeMode>"));
        assert!(!kml.contains("clampToGround"));
    }

    /// Names come from files and firmware and are not trusted.
    #[test]
    fn markup_in_a_name_cannot_break_the_document() {
        let kml = flight_path("a & b <script>", &[]);
        assert!(kml.contains("a &amp; b &lt;script&gt;"));
        assert!(!kml.contains("<script>"));
    }

    /// A control character from a fixed-length firmware string is not valid XML.
    #[test]
    fn a_control_character_is_replaced_rather_than_written() {
        assert_eq!(escape("bad\u{0}name"), "bad name");
    }
}
