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

//! Create KML + GPX on the flight screen's "Tlog > Kml or Graph" form: a telemetry log's positions
//! ([`mp_log::mavlinklog::flight_data`]) as `MavlinkLog.BUT_redokml_Click` hands them to
//! `MavlinkLogBase`, which writes, for the log `<log>`:
//!
//! - `writeGPX(logfile + "-" + sysid)` for each system in turn: a `.gpx` named by
//!   `GetFileNameWithoutExtension` of `<log>-<sysid>`, which cuts at the log's own extension, so
//!   every system writes the one `<name>.gpx` and the last system's is the one left;
//! - `writeKML(logfile + ".kml")`: the document zipped with the plane model into `<log>.kmz` -
//!   every `.kml` in the path made `.kmz`, as `Replace` makes it - and the `.kml` itself deleted, so
//!   not left beside it.
//!
//! The document is the flown path `headless-planner kml` writes ([`crate::flight_paths`]): a
//! coloured `LineString` for each flight mode, each system's in turn, as `writeKML`'s are. Its
//! plane models and points folders, its "First Person View" tour and SharpKml's serialisation are
//! not ported; the archive holds the model all the same, as the C#'s does.
//! `// C#: Log/MavlinkLog.cs:162-191; Log/MavlinkLogBase.cs:31-96, 132-435`

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use mp_log::mavlinklog::{FlightData, Sample, Zone};
use mp_log::netfmt;
use mp_log::zip;
use mp_units::LatLon;

use crate::TrackPoint;
use crate::dflog::{PLANE_MODEL, PLANE_MODEL_NAME, dos_now, gpx_time};

/// `new DateTime(1970, 1, 1).Ticks`.
const UNIX_EPOCH_TICKS: i64 = 621_355_968_000_000_000;
/// `TimeSpan.TicksPerMillisecond`.
const TICKS_PER_MS: i64 = 10_000;

/// `cs.datetime` as [`gpx_time`] takes it: Unix milliseconds, or `None` for `DateTime.MinValue`.
fn unix_ms(sample: &Sample) -> Option<i64> {
    let ticks = sample.datetime.ticks();
    (ticks != 0).then(|| (ticks - UNIX_EPOCH_TICKS).div_euclid(TICKS_PER_MS))
}

/// What `XmlTextWriter` escapes in an attribute's value: `&`, `<`, `>` and `"`.
fn attribute(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// `WriteElementString(name, value)`: the element, or `<name />` for an empty value; the text's
/// `&`, `<` and `>` escaped.
fn element(out: &mut String, name: &str, value: &str) {
    if value.is_empty() {
        let _ = write!(out, "<{name} />");
    } else {
        let text = value
            .replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;");
        let _ = write!(out, "<{name}>{text}</{name}>");
    }
}

/// `writeGPX`: a system's samples as a track, then again as numbered waypoints, by
/// `XmlTextWriter` in ASCII - no declaration, no indentation, a character past ASCII `?` - each
/// number as `ToString(new CultureInfo("en-US"))` writes it (a `double` to fifteen digits, a
/// `float` to seven) and each time as `yyyy-MM-ddTHH:mm:sszzzzzz`, local. `creator` is
/// `MainV2.instance.Text`, the main window's title.
/// `// C#: Log/MavlinkLogBase.cs:31-96`
#[must_use]
pub fn gpx(creator: &str, samples: &[Sample], zone: Zone<'_>) -> String {
    let mut out = format!(
        "<gpx creator=\"{}\" xmlns=\"http://www.topografix.com/GPX/1/1\"><trk>",
        attribute(creator)
    );
    if samples.is_empty() {
        out.push_str("<trkseg />");
    } else {
        out.push_str("<trkseg>");
        for cs in samples {
            let _ = write!(
                out,
                "<trkpt lat=\"{}\" lon=\"{}\">",
                netfmt::double(cs.lat),
                netfmt::double(cs.lng)
            );
            element(&mut out, "ele", &netfmt::single(cs.altasl));
            element(&mut out, "ele2", &netfmt::single(cs.alt));
            element(&mut out, "time", &gpx_time(unix_ms(cs), zone));
            element(&mut out, "course", &netfmt::single(cs.yaw));
            element(&mut out, "roll", &netfmt::single(cs.roll));
            element(&mut out, "pitch", &netfmt::single(cs.pitch));
            element(&mut out, "mode", &cs.mode);
            out.push_str("</trkpt>");
        }
        out.push_str("</trkseg>");
    }
    out.push_str("</trk>");
    for (a, cs) in samples.iter().enumerate() {
        let _ = write!(
            out,
            "<wpt lat=\"{}\" lon=\"{}\">",
            netfmt::double(cs.lat),
            netfmt::double(cs.lng)
        );
        element(&mut out, "name", &a.to_string());
        element(&mut out, "time", &gpx_time(unix_ms(cs), zone));
        element(&mut out, "ele", &netfmt::single(cs.altasl));
        out.push_str("</wpt>");
    }
    out.push_str("</gpx>");
    out.chars()
        .map(|c| if c.is_ascii() { c } else { '?' })
        .collect()
}

/// `Path.GetFileNameWithoutExtension` of a file name: up to its last `.`, or all of it.
fn without_extension(name: &str) -> &str {
    name.rfind('.')
        .map_or(name, |at| name.get(..at).unwrap_or(name))
}

/// The path `writeKML` draws for one system: each sample where it was, at its `altasl` - or at
/// `alt + basealt` when the home altitude was given, `basealt != 0` - its seconds from the
/// system's first sample, in its mode.
/// `// C#: Log/MavlinkLogBase.cs:250-260`
#[must_use]
pub fn track(samples: &[Sample], base_alt: f64) -> Vec<TrackPoint> {
    let start = samples.first().map_or(0, |first| first.datetime.ticks());
    samples
        .iter()
        .map(|cs| {
            #[allow(clippy::cast_precision_loss)] // ticks of a flight
            let seconds = (cs.datetime.ticks() - start) as f64 / 10_000_000.0;
            TrackPoint {
                position: LatLon::new(cs.lat, cs.lng).unwrap_or_default(),
                altitude_msl: if base_alt == 0.0 {
                    f64::from(cs.altasl)
                } else {
                    f64::from(cs.alt) + base_alt
                },
                seconds,
                mode: cs.mode.clone(),
            }
        })
        .collect()
}

/// Every file Create KML + GPX leaves for `log`, with its bytes, in the order the C# writes them:
/// each system's `.gpx` (one name for all, see the module), then the `.kmz` - the document, named
/// as `headless-planner kml` names it, as `<log name>.kml` with the plane model, its entries
/// stamped `modified`. `base_alt` is `basealt`: the home altitude, or 0 for altitudes above sea
/// level.
///
/// # Errors
///
/// The archive cannot be made.
/// `// C#: Log/MavlinkLog.cs:179-189; Log/MavlinkLogBase.cs:376-435`
pub fn kml_files(
    log: &Path,
    flight: &FlightData,
    base_alt: f64,
    creator: &str,
    zone: Zone<'_>,
    modified: zip::DosTime,
) -> Result<Vec<(PathBuf, Vec<u8>)>, zip::ZipError> {
    let dir = log.parent().unwrap_or_else(|| Path::new(""));
    let name = log
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    let mut files = Vec::new();
    for (sysid, samples) in &flight.vehicles {
        let gpx_name = format!("{}.gpx", without_extension(&format!("{name}-{sysid}")));
        files.push((dir.join(gpx_name), gpx(creator, samples, zone).into_bytes()));
    }
    let title = log
        .file_stem()
        .map_or_else(|| name.clone(), |stem| stem.to_string_lossy().into_owned());
    let tracks: Vec<Vec<TrackPoint>> = flight
        .vehicles
        .iter()
        .map(|(_, samples)| track(samples, base_alt))
        .collect();
    let tracks: Vec<&[TrackPoint]> = tracks.iter().map(Vec::as_slice).collect();
    let kml = crate::flight_paths(&title, &tracks).into_bytes();
    let entries = [
        zip::Entry {
            name: format!("{name}.kml"),
            data: kml,
        },
        zip::Entry {
            name: PLANE_MODEL_NAME.to_owned(),
            data: PLANE_MODEL.to_vec(),
        },
    ];
    let kmz = zip::write(&entries, modified)?;
    // `filename.Replace(Path.GetExtension(filename), ".kmz")`, `filename` the log's and ".kml".
    let kmz_path = PathBuf::from(format!("{}.kml", log.to_string_lossy()).replace(".kml", ".kmz"));
    files.push((kmz_path, kmz));
    Ok(files)
}

/// Create KML + GPX for one log: its files ([`kml_files`]) written, stamped now in `zone`,
/// returning each written, once.
///
/// # Errors
///
/// A file cannot be written, or the archive made.
pub fn create_kml(
    log: &Path,
    flight: &FlightData,
    base_alt: f64,
    creator: &str,
    zone: Zone<'_>,
) -> std::io::Result<Vec<PathBuf>> {
    let files =
        kml_files(log, flight, base_alt, creator, zone, dos_now(zone)).map_err(
            |err| match err {
                zip::ZipError::Io(err) => err,
                other => std::io::Error::other(other.to_string()),
            },
        )?;
    let mut written: Vec<PathBuf> = Vec::new();
    for (path, data) in files {
        mp_os::fs::write(&path, &data)?;
        if !written.contains(&path) {
            written.push(path);
        }
    }
    Ok(written)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use super::*;
    use crate::dflog::utc;
    use mp_log::mavlinklog::DateTime;

    /// 2026-10-16 00:00:02 UTC, in Unix microseconds.
    const AT: u64 = 1_792_108_802_000_000;

    fn sample(mode: &str) -> Sample {
        Sample {
            datetime: DateTime::from_tlog_micros(AT).unwrap(),
            lat: -27.5,
            lng: 153.25,
            altasl: 584.5,
            alt: 10.5,
            roll: 1.5,
            pitch: -2.25,
            yaw: 90.0,
            mode: mode.to_owned(),
        }
    }

    /// `writeGPX` as `XmlTextWriter` writes it: no declaration, no indentation, the track then the
    /// waypoints, each number as .NET writes it, each time with its offset.
    #[test]
    fn a_gpx_is_written_as_the_xml_writer_writes_it() {
        assert_eq!(
            gpx("MissionPlannerRust", &[sample("Loiter")], &utc),
            "<gpx creator=\"MissionPlannerRust\" xmlns=\"http://www.topografix.com/GPX/1/1\"><trk>\
             <trkseg><trkpt lat=\"-27.5\" lon=\"153.25\"><ele>584.5</ele><ele2>10.5</ele2>\
             <time>2026-10-16T00:00:02+00:00</time><course>90</course><roll>1.5</roll>\
             <pitch>-2.25</pitch><mode>Loiter</mode></trkpt></trkseg></trk>\
             <wpt lat=\"-27.5\" lon=\"153.25\"><name>0</name><time>2026-10-16T00:00:02+00:00</time>\
             <ele>584.5</ele></wpt></gpx>"
        );
        // Ten hours east, local.
        assert!(gpx("x", &[sample("Loiter")], &|_| 36_000).contains("2026-10-16T10:00:02+10:00"));
        // No samples: an empty segment, written as the writer closes an empty element.
        assert_eq!(
            gpx("x", &[], &utc),
            "<gpx creator=\"x\" xmlns=\"http://www.topografix.com/GPX/1/1\"><trk><trkseg /></trk></gpx>"
        );
        // An empty mode is an empty element; a time never stamped is `MinValue`'s.
        let mut never = sample("");
        never.datetime = DateTime::MIN;
        let text = gpx("x", &[never], &utc);
        assert!(text.contains("<mode />"));
        assert!(text.contains("<time>0001-01-01T00:00:00+00:00</time>"));
    }

    /// Each system's `.gpx` is `<name>.gpx` - `GetFileNameWithoutExtension` of `<log>-<sysid>` -
    /// written once for each, then `<log>.kmz` holding `<log>.kml` and the plane model.
    #[test]
    fn the_files_are_the_csharps_names_in_its_order() {
        let flight = FlightData {
            vehicles: vec![(1, vec![sample("Auto")]), (2, vec![sample("RTL")])],
            home_z: None,
        };
        let modified = zip::DosTime {
            year: 2026,
            month: 10,
            day: 16,
            hour: 0,
            minute: 0,
            second: 0,
        };
        let log = Path::new("/logs/flight.tlog");
        let files = kml_files(log, &flight, 0.0, "x", &utc, modified).unwrap();
        let names: Vec<PathBuf> = files.iter().map(|(path, _)| path.clone()).collect();
        assert_eq!(
            names,
            [
                PathBuf::from("/logs/flight.gpx"),
                PathBuf::from("/logs/flight.gpx"),
                PathBuf::from("/logs/flight.tlog.kmz"),
            ]
        );
        // The last system's is the one left.
        assert!(String::from_utf8_lossy(&files[1].1).contains("<mode>RTL</mode>"));
        let entries = zip::read(&files[2].1).unwrap();
        let entry_names: Vec<&str> = entries.iter().map(|entry| entry.name.as_str()).collect();
        assert_eq!(entry_names, ["flight.tlog.kml", "block_plane_0.dae"]);
        let kml = String::from_utf8(entries[0].data.clone()).unwrap();
        assert!(kml.contains("<name>flight</name>"));
        // Each system's segments numbered from the first again.
        assert!(kml.contains("0 Flight Path Auto"));
        assert!(kml.contains("0 Flight Path RTL"));
        // A log with no extension: its own name, and the `-sysid` kept.
        let files =
            kml_files(Path::new("/logs/flight"), &flight, 0.0, "x", &utc, modified).unwrap();
        assert_eq!(files[0].0, PathBuf::from("/logs/flight-1.gpx"));
        assert_eq!(files[2].0, PathBuf::from("/logs/flight.kmz"));
    }

    /// The path's altitude: above sea level, or above home plus the home altitude when one is
    /// given; its seconds from the system's first sample.
    #[test]
    fn the_path_is_at_altasl_or_home_altitude_plus_alt() {
        let mut later = sample("Auto");
        later.datetime = DateTime::from_tlog_micros(AT + 1_500_000).unwrap();
        let samples = [sample("Auto"), later];
        let absolute = track(&samples, 0.0);
        assert_eq!(absolute[0].altitude_msl, 584.5);
        assert_eq!(absolute[1].seconds, 1.5);
        let relative = track(&samples, 100.0);
        assert_eq!(relative[0].altitude_msl, 110.5);
        assert_eq!(relative[0].position.latitude(), -27.5);
    }
}
