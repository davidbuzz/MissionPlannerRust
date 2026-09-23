//! Exporting a real recorded flight, and checking where it lands.
//!
//! The unit tests check the coordinate order against a constant. This one checks it against a
//! flight that actually happened: `testdata/mavlink/autotest.tlog` was recorded at ArduPilot's
//! default SITL home outside Canberra, so a file whose longitude and latitude are the wrong way
//! round puts it in Kazakhstan - and that is a bounds check a test can make.
//!
//! It is the same property the unit test asserts, arrived at from the other end. A transposition
//! that survived both would have to be wrong in a way that is also self-consistent.

use mp_kml::{TrackPoint, flight_path};
use mp_units::LatLon;

/// Canberra, give or take. ArduPilot's SITL default home is -35.363262, 149.165237.
const AUSTRALIA_LON: std::ops::Range<f64> = 112.0..154.0;
const AUSTRALIA_LAT: std::ops::Range<f64> = -44.0..-10.0;

fn recorded_track() -> Vec<TrackPoint> {
    use mp_log::TlogReader;
    use mp_mavlink_dialects::all::{DIALECT, MavMessage};

    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/mavlink/autotest.tlog");
    let data =
        std::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()));

    let mut track = Vec::new();
    let mut reader = TlogReader::new(&data);
    while let Some(record) = reader.next_record(&DIALECT) {
        let Ok((frame, _)) = mp_mavlink::parse(record.frame, &DIALECT) else {
            continue;
        };
        if let Some(MavMessage::GlobalPositionInt(m)) =
            MavMessage::decode(frame.msgid, frame.payload)
            && !(m.lat == 0 && m.lon == 0)
            && let Ok(position) = LatLon::from_mavlink_e7(m.lat, m.lon)
        {
            track.push(TrackPoint {
                position,
                altitude_msl: f64::from(m.alt) / 1000.0,
                seconds: track.len() as f64 * 0.1,
                mode: "AUTO".to_owned(),
            });
        }
    }
    track
}

#[test]
fn a_recorded_flight_exports_to_where_it_was_flown() {
    let track = recorded_track();
    assert!(
        track.len() > 100,
        "the fixture should hold a real flight, found {} positions",
        track.len()
    );

    let kml = flight_path("autotest", &track);

    // Pull every coordinate triple back out and check both fields land in Australia. A
    // transposition puts the longitude at -35 and the latitude at 149, and 149 is not a latitude
    // at all - but asserting the range catches the subtler case where both are plausible numbers.
    let body = kml
        .split("<coordinates>")
        .nth(1)
        .and_then(|rest| rest.split("</coordinates>").next())
        .expect("a coordinates element");
    let mut checked = 0usize;
    for triple in body.split_whitespace() {
        let parts: Vec<f64> = triple.split(',').filter_map(|p| p.parse().ok()).collect();
        assert_eq!(parts.len(), 3, "{triple:?} is not lon,lat,alt");
        assert!(
            AUSTRALIA_LON.contains(&parts[0]),
            "{} is not an Australian longitude - the fields are probably transposed",
            parts[0]
        );
        assert!(
            AUSTRALIA_LAT.contains(&parts[1]),
            "{} is not an Australian latitude - the fields are probably transposed",
            parts[1]
        );
        checked += 1;
    }
    assert!(checked > 10, "only {checked} coordinates were checked");
}

/// The document a real flight produces has to be parseable, not merely printable.
#[test]
fn a_real_export_is_balanced_xml() {
    let kml = flight_path("autotest", &recorded_track());

    // A crude balance check rather than a parser: every element this writer emits is opened and
    // closed exactly once per use, so the counts must agree.
    for element in [
        "Document",
        "Placemark",
        "LineString",
        "coordinates",
        "Style",
        "name",
    ] {
        let open = kml.matches(&format!("<{element}>")).count()
            + kml.matches(&format!("<{element} ")).count();
        let close = kml.matches(&format!("</{element}>")).count();
        assert_eq!(open, close, "<{element}> is not balanced");
    }
    assert!(kml.starts_with("<?xml"));
    assert!(kml.trim_end().ends_with("</kml>"));
}
