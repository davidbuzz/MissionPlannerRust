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

//! Create KML + gpx: a dataflash log as Mission Planner's `LogOutput` writes it - a `.kmz` holding
//! the flown path, the `POS` path, a model of the aircraft per attitude sample and the mission, and
//! beside it a `.gpx` track, the mission and rally points as waypoint files, the parameters as a
//! `.param` file and the raw GNSS observations as RINEX.
//!
//! `but_dflogtokml_Click` (`GCSViews/FlightData.cs:1135-1197`) feeds every line of the log to
//! `LogOutput.processLine` - a `.bin` through `DFLogBuffer` ([`mp_log::dflogbuffer`]), a `.log`
//! line by line - and then calls `writeKML(<log>.kml)`. Everything is decided by the text of the
//! lines, split on `,` and `:`, and by the formats read from the `FMT` lines among them, so this
//! is a port of `ExtLibs/Utilities/LogOutput.cs` over that text, with its rules kept:
//!
//! - a flight path is cut at every `MODE` line and named after the mode; a `GPS` point needs a
//!   3D fix, an altitude below 40 km and a place on the globe; after 199 mode changes no point is
//!   kept;
//! - an aircraft is placed at each `ATT` line that follows a `GPS` point which moved in both
//!   latitude and longitude, carrying that line's attitude and the time of that `GPS` line;
//! - the waypoint and rally files start a new file whenever the total changes or the numbering goes
//!   back; the parameters come out in the invariant culture's order;
//! - the `.kml` is XmlSerializer's rendering of KMLib's classes (`ExtLibs/KMLib`), indented by two,
//!   and it is zipped with the plane model into `<log>.kmz`, not left beside it.
//!
//! **One line differs from mono's output on purpose.** The root element's namespace declarations
//! come in .NET Framework's order, `xmlns:xsi` then `xmlns:xsd`, as every `XmlSerializer` document
//! Mission Planner ships has them (`checklistDefault.xml`); mono writes them the other way round.
//! `tests/dflog.rs` holds everything else to the oracle's output byte for byte.
//!
//! Text files end their lines with the platform's newline, as `StreamWriter.WriteLine` and the
//! indenting `XmlWriter` do. The `.kmz` goes to the log's directory with its file name in lower
//! case; the C# lower-cases the whole path (`LogOutput.cs:1105`), which on Windows is the same
//! place and on Linux fails for any directory with a capital in it.
//! `// C#: ExtLibs/Utilities/LogOutput.cs; GCSViews/FlightData.cs:1135-1197`

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use mp_log::convert::ModeName;
use mp_log::dflogbuffer::{DfLog, DfLogBuffer, civil_from_days};
use mp_log::netfmt;
use mp_log::zip;

/// `Environment.NewLine`.
pub const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// The aircraft model `writeKML` zips beside the `.kml`, from Mission Planner's install directory.
/// `// C#: ExtLibs/Utilities/LogOutput.cs:1130-1152`
pub const PLANE_MODEL: &[u8] = include_bytes!("../assets/block_plane_0.dae");

/// Its name in the `.kmz`, which each aircraft's `Link` refers to.
const PLANE_MODEL_NAME: &str = "block_plane_0.dae";

/// `new List<Point3D>[200]`: how many flight path segments there can be.
const SEGMENTS: usize = 200;

/// `(int)MAVLink.MAV_CMD.LAST`: the highest command number that is a place.
/// `// C#: ExtLibs/Mavlink/Mavlink.cs:921`
const MAV_CMD_LAST: i32 = 95;

/// `DateTime.MinValue` in Unix milliseconds: what a `GPS` line with no time gives.
const MIN_VALUE_MILLIS: i64 = -62_135_596_800_000;

/// A local time zone: the offset from UTC, in seconds, at a UTC instant in Unix milliseconds.
/// Mission Planner writes GPX times in local time (`DFLog.gpsTimeToTime` ends in `ToLocalTime`).
pub type Zone<'a> = &'a dyn Fn(i64) -> i32;

/// UTC as the zone: what the tests use, and what the oracle ran under.
#[must_use]
pub fn utc(_: i64) -> i32 {
    0
}

/// This machine's time zone, `TimeZoneInfo.Local`: what the DataFlash Logs page passes.
#[must_use]
pub fn local_zone(utc_millis: i64) -> i32 {
    use chrono::{Offset, TimeZone};
    chrono::Local
        .timestamp_millis_opt(utc_millis)
        .single()
        .map_or(0, |time| time.offset().fix().local_minus_utc())
}

/// `Core.Geometry.Point3D`: longitude, latitude, altitude.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Point3D {
    x: f64,
    y: f64,
    z: f64,
}

impl Point3D {
    /// `Point3D.Serialize`: `x,y,z` in the en-US culture.
    /// `// C#: ExtLibs/Core/Geometry/Point3D.cs:80-83`
    fn serialize(self) -> String {
        format!(
            "{},{},{}",
            netfmt::double(self.x),
            netfmt::double(self.y),
            netfmt::double(self.z)
        )
    }
}

/// `KMLib.Geometry.Model`'s location and orientation. `// C#: ExtLibs/KMLib/Geometry/Model.cs`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Model {
    latitude: f64,
    longitude: f64,
    altitude: f64,
    heading: f64,
    tilt: f64,
    roll: f64,
}

impl Model {
    /// `ALocation.Equals`: the three doubles, bit for bit, as a struct of primitives compares.
    fn same_place(&self, other: &Self) -> bool {
        self.latitude.to_bits() == other.latitude.to_bits()
            && self.longitude.to_bits() == other.longitude.to_bits()
            && self.altitude.to_bits() == other.altitude.to_bits()
    }
}

/// `LogOutput.Data`: one attitude sample. `// C#: ExtLibs/Utilities/LogOutput.cs:47-54`
#[derive(Debug, Clone)]
struct Data {
    model: Model,
    ntun: Vec<String>,
    /// UTC milliseconds, or `None` for `DateTime.MinValue`.
    datetime: Option<i64>,
}

/// Mission Planner's `LogOutput`: what the lines of a log have said so far.
/// `// C#: ExtLibs/Utilities/LogOutput.cs:20-56`
#[derive(Debug, Clone)]
pub struct LogOutput {
    lastline: String,
    ntunlast: Vec<String>,
    cmdraw: Vec<String>,
    ralyraw: Vec<String>,
    oldlastpos: Point3D,
    lastpos: Point3D,
    flightdata: Vec<Data>,
    gpsrawdata: Vec<String>,
    modelist: Vec<String>,
    paramlist: HashMap<String, String>,
    position: Vec<Option<Vec<Point3D>>>,
    positionindex: usize,
    /// `PosLatLngAlts`: latitude, longitude, altitude.
    positions: Vec<(f64, f64, f64)>,
    dflog: DfLog,
}

impl Default for LogOutput {
    fn default() -> Self {
        Self::new()
    }
}

/// `line.Split(',', ':')`.
fn split(line: &str) -> Vec<&str> {
    line.split([',', ':']).collect()
}

impl LogOutput {
    /// A `LogOutput` that has read nothing.
    #[must_use]
    pub fn new() -> Self {
        Self {
            lastline: String::new(),
            ntunlast: vec![String::new(); 14],
            cmdraw: Vec::new(),
            ralyraw: Vec::new(),
            oldlastpos: Point3D::default(),
            lastpos: Point3D::default(),
            flightdata: Vec::new(),
            gpsrawdata: Vec::new(),
            modelist: Vec::new(),
            paramlist: HashMap::new(),
            position: vec![None; SEGMENTS],
            positionindex: 0,
            positions: Vec::new(),
            dflog: DfLog::new(),
        }
    }

    /// `double.Parse(items[FindMessageOffset(type, field)], InvariantCulture)`, or `None` where
    /// that throws.
    fn number(&mut self, items: &[&str], linetype: &str, field: &str) -> Option<f64> {
        let index = self.dflog.find_message_offset(linetype, field)?;
        netfmt::parse_double(items.get(index)?)
    }

    /// `processLine`: one line of the log. Whatever throws in the C# ends the line's processing
    /// there, as its outer `catch` does, keeping what was done before it.
    /// `// C#: ExtLibs/Utilities/LogOutput.cs:58-242`
    pub fn process_line(&mut self, line: &str) {
        if line.is_empty() {
            return;
        }
        let _ = self.process(line);
    }

    fn process(&mut self, line: &str) -> Option<()> {
        let items = split(line);
        let kind = *items.first()?;
        if kind.contains("FMT") {
            self.dflog.fmt_line(line);
        }
        if kind.contains("PARM") {
            let name = self.dflog.find_message_offset("PARM", "Name");
            let value = self.dflog.find_message_offset("PARM", "Value");
            if let (Some(name), Some(value)) = (
                name.and_then(|i| items.get(i)),
                value.and_then(|i| items.get(i)),
            ) {
                self.paramlist.insert(
                    netfmt::trim(name).to_owned(),
                    netfmt::trim(value).to_owned(),
                );
            }
        } else if kind.contains("CMD") {
            self.cmdraw.push(line.to_owned());
        } else if kind.contains("RALY") {
            self.ralyraw.push(line.to_owned());
        } else if kind.contains("MOD") {
            self.positionindex += 1;
            while self.modelist.len() < self.positionindex + 1 {
                self.modelist.push(String::new());
            }
            let mode = match self.dflog.find_message_offset("MODE", "Mode") {
                Some(index) => items.get(index)?,
                None if items.len() == 4 => items.get(2)?,
                None => items.get(1)?,
            };
            if let Some(slot) = self.modelist.get_mut(self.positionindex) {
                *slot = (*mode).to_owned();
            }
        } else if kind.contains("GPS") && self.dflog.contains("GPS") {
            if kind.contains("GPS2") {
                return None;
            }
            let status = self.dflog.find_message_offset("GPS", "Status")?;
            if netfmt::parse_i32(items.get(status)?)? < 3 {
                return None;
            }
            // `position[positionindex]` past the array throws.
            let slot = self.position.get_mut(self.positionindex)?;
            slot.get_or_insert_with(Vec::new);
            let alt = self.number(&items, "GPS", "Alt")?;
            if alt > 40_000.0 {
                return None;
            }
            let lng = self.number(&items, "GPS", "Lng")?;
            let lat = self.number(&items, "GPS", "Lat")?;
            if !(-90.0..=90.0).contains(&lat) && !lat.is_nan() {
                return None;
            }
            if !(-180.0..=180.0).contains(&lng) && !lng.is_nan() {
                return None;
            }
            let point = Point3D {
                x: lng,
                y: lat,
                z: alt,
            };
            self.position
                .get_mut(self.positionindex)?
                .as_mut()?
                .push(point);
            self.oldlastpos = self.lastpos;
            self.lastpos = point;
            line.clone_into(&mut self.lastline);
        } else if kind.contains("POS") {
            if self.dflog.contains("POS") {
                let lat = self.number(&items, "POS", "Lat")?;
                let lng = self.number(&items, "POS", "Lng")?;
                let alt = self.number(&items, "POS", "Alt")?;
                self.positions.push((lat, lng, alt));
            }
        } else if kind.contains("GRAW") || kind.contains("GRXH") || kind.contains("GRXS") {
            self.gpsrawdata.push(line.to_owned());
        } else if kind.contains("CTUN") {
            // `ctunlast` is kept but nothing reads it.
        } else if kind.contains("NTUN") {
            self.ntunlast = items.iter().map(|s| (*s).to_owned()).collect();
        } else if kind.contains("ATT") {
            let (lastpos, oldlastpos) = (self.lastpos, self.oldlastpos);
            if lastpos.x != 0.0 && oldlastpos.x != lastpos.x && oldlastpos.y != lastpos.y {
                let lastline = self.lastline.clone();
                let datetime = self.dflog.gps_time(&lastline);
                self.oldlastpos = lastpos;
                let roll = self.number(&items, "ATT", "Roll")? / -1.0;
                let tilt = self.number(&items, "ATT", "Pitch")? / -1.0;
                let heading = self.number(&items, "ATT", "Yaw")? / 1.0;
                self.flightdata.push(Data {
                    model: Model {
                        latitude: lastpos.y,
                        longitude: lastpos.x,
                        altitude: lastpos.z,
                        heading,
                        tilt,
                        roll,
                    },
                    ntun: self.ntunlast.clone(),
                    datetime,
                });
            }
        }
        Some(())
    }
}

/// `XmlConvert.ToString(double)`: `-0` for negative zero, `INF`/`-INF`, else the round-trip form.
fn xml_double(value: f64) -> String {
    if value == 0.0 && value.is_sign_negative() {
        "-0".to_owned()
    } else if value.is_infinite() {
        if value > 0.0 { "INF" } else { "-INF" }.to_owned()
    } else {
        netfmt::double_roundtrip(value)
    }
}

/// `XmlConvert.ToString(float)`.
fn xml_single(value: f32) -> String {
    if value == 0.0 && value.is_sign_negative() {
        "-0".to_owned()
    } else if value.is_infinite() {
        if value > 0.0 { "INF" } else { "-INF" }.to_owned()
    } else {
        netfmt::single_roundtrip(value)
    }
}

/// Text as `XmlTextWriter` writes it: `&`, `<`, `>` escaped; in an attribute `"` too, and line
/// breaks and tabs as character entities; in an element, line breaks and tabs as they are - the
/// plane's description keeps the `\r\n` of the C# source it was written in - and any other control
/// character as an entity.
fn escape(text: &str, attribute: bool) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' if attribute => out.push_str("&quot;"),
            '\t' | '\n' | '\r' if !attribute => out.push(c),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "&#x{:X};", c as u32);
            }
            c => out.push(c),
        }
    }
    out
}

/// An indenting XML writer shaped like `XmlTextWriter` with `Formatting.Indented` and two spaces:
/// every element on its own line, text kept on its element's line, an empty element as `<a />`.
struct Xml {
    out: String,
    depth: usize,
    newline: &'static str,
}

impl Xml {
    fn new(newline: &'static str) -> Self {
        Self {
            out: "<?xml version=\"1.0\"?>".to_owned(),
            depth: 0,
            newline,
        }
    }

    fn indent(&mut self) {
        self.out.push_str(self.newline);
        for _ in 0..self.depth {
            self.out.push_str("  ");
        }
    }

    fn start(&mut self, name: &str, attributes: &[(&str, &str)]) {
        self.indent();
        self.out.push('<');
        self.out.push_str(name);
        for (key, value) in attributes {
            let _ = write!(self.out, " {key}=\"{}\"", escape(value, true));
        }
        self.out.push('>');
        self.depth += 1;
    }

    fn end(&mut self, name: &str) {
        self.depth = self.depth.saturating_sub(1);
        self.indent();
        let _ = write!(self.out, "</{name}>");
    }

    fn leaf(&mut self, name: &str, text: &str) {
        self.indent();
        if text.is_empty() {
            let _ = write!(self.out, "<{name} />");
        } else {
            let _ = write!(self.out, "<{name}>{}</{name}>", escape(text, false));
        }
    }
}

/// One part of a KMLib `Style`.
#[derive(Debug, Clone, Copy)]
enum StylePart {
    /// `LineStyle(color, width)`.
    Line { color: &'static str, width: f32 },
    /// `PolyStyle { Color = color }`.
    Poly { color: &'static str },
}

/// A KMLib `Style` as XmlSerializer writes it: its `id` attribute if it has one, then each
/// `LineStyle`/`PolyStyle` - `color` (`ColorKML`, `AABBGGRR` in capitals), `colorMode` (an enum
/// with no `Specified` the serializer honours, so always `normal`) and a line's `width`.
/// `// C#: ExtLibs/KMLib/Style.cs; ExtLibs/KMLib/Support/Wrappers.cs:73-108`
fn style(xml: &mut Xml, id: Option<&str>, parts: &[StylePart]) {
    match id {
        Some(id) => xml.start("Style", &[("id", id)]),
        None => xml.start("Style", &[]),
    }
    for part in parts {
        match *part {
            StylePart::Line { color, width } => {
                xml.start("LineStyle", &[]);
                xml.leaf("color", color);
                xml.leaf("colorMode", "normal");
                xml.leaf("width", &xml_single(width));
                xml.end("LineStyle");
            }
            StylePart::Poly { color } => {
                xml.start("PolyStyle", &[]);
                xml.leaf("color", color);
                xml.leaf("colorMode", "normal");
                xml.end("PolyStyle");
            }
        }
    }
    xml.end("Style");
}

/// `style`, the path style: `HexStringToColor("7f00ffff")`, width 4, and a green polygon.
/// `// C#: ExtLibs/Utilities/LogOutput.cs:820-831`
const PATH_STYLE: [StylePart; 2] = [
    StylePart::Line {
        color: "7F00FFFF",
        width: 4.0,
    },
    StylePart::Poly { color: "7F00FF00" },
];

/// `style1`, "spray", which nothing uses. `// C#: ExtLibs/Utilities/LogOutput.cs:824-827`
const SPRAY_STYLE: [StylePart; 2] = [
    StylePart::Line {
        color: "4C0000FF",
        width: 0.0,
    },
    StylePart::Poly { color: "4C0000FF" },
];

/// `colours`: red, orange, yellow, green, blue, indigo, violet, pink, as `ColorKML` writes them.
/// A segment takes `colours[g % (colours.Length - 1)]`, so pink is never used.
/// `// C#: ExtLibs/Utilities/LogOutput.cs:809-813, 909-915`
const COLOURS: [&str; 8] = [
    "FF0000FF", "FF00A5FF", "FF00FFFF", "FF008000", "FFFF0000", "FF82004B", "FFEE82EE", "FFCBC0FF",
];

/// A placemark holding a `LineString`, as XmlSerializer writes `Placemark`: `name`, `visibility`,
/// `description`, `styleUrl`, the `Style` list, then the geometry.
struct LinePlacemark<'a> {
    name: &'a str,
    style_url: Option<&'a str>,
    style: Option<(Option<&'a str>, &'a [StylePart])>,
    /// `Extrude = true` and an `AltitudeMode`, or neither.
    extrude_mode: Option<&'a str>,
    coordinates: &'a [Point3D],
}

fn line_placemark(xml: &mut Xml, placemark: &LinePlacemark<'_>) {
    xml.start("Placemark", &[]);
    xml.leaf("name", placemark.name);
    if let Some(url) = placemark.style_url {
        xml.leaf("styleUrl", url);
    }
    if let Some((id, parts)) = placemark.style {
        style(xml, id, parts);
    }
    xml.start("LineString", &[]);
    if let Some(mode) = placemark.extrude_mode {
        xml.leaf("extrude", "1");
        xml.leaf("altitudeMode", mode);
    }
    let coordinates: Vec<String> = placemark
        .coordinates
        .iter()
        .map(|p| p.serialize())
        .collect();
    xml.leaf("coordinates", &coordinates.join(" "));
    xml.end("LineString");
    xml.end("Placemark");
}

/// `GetDistance`: haversine metres on a 6371 km sphere.
/// `// C#: ExtLibs/Utilities/PointLatLngAlt.cs:382-393`
fn distance(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f64 {
    let d = lat1 * 0.017_453_292_519_943_295;
    let num2 = lng1 * 0.017_453_292_519_943_295;
    let num3 = lat2 * 0.017_453_292_519_943_295;
    let num4 = lng2 * 0.017_453_292_519_943_295;
    let num5 = num4 - num2;
    let num6 = num3 - d;
    let num7 = (num6 / 2.0).sin().powf(2.0) + d.cos() * num3.cos() * (num5 / 2.0).sin().powf(2.0);
    let num8 = 2.0 * num7.sqrt().atan2((1.0 - num7).sqrt());
    6371.0 * num8 * 1000.0
}

/// A local time as `DateTime.ToString("yyyy-MM-ddTHH:mm:sszzzzzz")` writes it: `zzzzzz` is `zzz`,
/// the offset as `+hh:mm`.
fn gpx_time(datetime: Option<i64>, zone: Zone<'_>) -> String {
    let utc = datetime.unwrap_or(MIN_VALUE_MILLIS);
    let offset = zone(utc);
    let local = if datetime.is_some() {
        utc + i64::from(offset) * 1000
    } else {
        MIN_VALUE_MILLIS
    };
    let seconds = local.div_euclid(1000);
    let (year, month, day) = civil_from_days(seconds.div_euclid(86_400));
    let of_day = seconds.rem_euclid(86_400);
    let sign = if offset < 0 { '-' } else { '+' };
    let magnitude = offset.unsigned_abs();
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}{sign}{:02}:{:02}",
        of_day / 3600,
        of_day / 60 % 60,
        of_day % 60,
        magnitude / 3600,
        magnitude / 60 % 60
    )
}

/// .NET's custom numeric format `0.000`-style for a double: the value first rounded to 15
/// significant digits, then to `decimals` places, half away from zero; a result of zero loses its
/// sign.
fn fixed(value: f64, decimals: usize) -> String {
    if !value.is_finite() {
        return netfmt::double(value);
    }
    // 15 significant digits, as the general format gives them.
    let digits = netfmt::general(value.abs(), 15);
    let (mantissa, exponent) = match digits.split_once('E') {
        Some((m, e)) => (m.to_owned(), e.parse::<i32>().unwrap_or(0)),
        None => (digits.clone(), 0),
    };
    let (whole, fraction) = mantissa.split_once('.').unwrap_or((&mantissa, ""));
    let mut all: Vec<u8> = whole
        .bytes()
        .chain(fraction.bytes())
        .map(|b| b - b'0')
        .collect();
    let mut point = i32::try_from(whole.len()).unwrap_or(0) + exponent;
    // Pad so the point sits inside the digits and `decimals` places follow it.
    while point < 1 {
        all.insert(0, 0);
        point += 1;
    }
    let point = usize::try_from(point).unwrap_or(0);
    all.resize(all.len().max(point + decimals + 1), 0);
    let round_up = all.get(point + decimals).is_some_and(|&d| d >= 5);
    all.truncate(point + decimals);
    let mut at = all.len();
    let mut carried = round_up;
    while carried && at > 0 {
        at -= 1;
        if let Some(d) = all.get_mut(at) {
            if *d == 9 {
                *d = 0;
            } else {
                *d += 1;
                carried = false;
            }
        }
    }
    let mut point = point;
    if carried {
        all.insert(0, 1);
        point += 1;
    }
    let zero = all.iter().all(|&d| d == 0);
    let mut out = String::new();
    if value < 0.0 && !zero {
        out.push('-');
    }
    let integer: String = all
        .iter()
        .take(point)
        .map(|&d| char::from(b'0' + d))
        .collect();
    let integer = integer.trim_start_matches('0');
    out.push_str(if integer.is_empty() { "0" } else { integer });
    if decimals > 0 {
        out.push('.');
        out.extend(all.iter().skip(point).map(|&d| char::from(b'0' + d)));
    }
    out
}

/// .NET's custom format `00`: rounded to a whole number (via 15 significant digits), at least two
/// digits.
fn two_digits(value: f64) -> String {
    let text = fixed(value, 0);
    let (sign, digits) = text
        .strip_prefix('-')
        .map_or(("", text.as_str()), |d| ("-", d));
    format!("{sign}{digits:0>2}")
}

/// Composite formatting's `{n,w}`: right-aligned in `w` columns, never cut.
fn right(text: &str, width: usize) -> String {
    format!("{text:>width$}")
}

/// What `writeKML` leaves on disk for one log: each file and what goes in it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct KmlFiles {
    /// Every file written, in the order the C# writes them: the `.gpx`, the `.obs`, the waypoint
    /// files, the rally files, the `.param`, and last the `.kmz`.
    pub files: Vec<(PathBuf, Vec<u8>)>,
    /// The `.kml` inside the `.kmz`, for reading back.
    pub kml: Vec<u8>,
}

/// `GetFromGps`: 1980-01-06 UTC plus the weeks and seconds, the seconds rounded to the millisecond
/// as `AddSeconds` rounds them. In Unix milliseconds, or `None` where `DateTime` throws for a time
/// outside years 1 to 9999. `// C#: ExtLibs/Utilities/LogOutput.cs:435-441`
fn from_gps(week: i32, seconds: f64) -> Option<i64> {
    const GPS_EPOCH: i64 = 315_964_800_000;
    const MAX_VALUE_MILLIS: i64 = 253_402_300_799_999;
    let rounded = seconds * 1000.0 + if seconds >= 0.0 { 0.5 } else { -0.5 };
    if !rounded.is_finite() || rounded.abs() >= 3.2e17 {
        return None;
    }
    #[allow(clippy::cast_possible_truncation)]
    let millis = rounded as i64;
    let weeks = GPS_EPOCH + i64::from(week) * 7 * 86_400_000;
    let time = weeks + millis;
    ((MIN_VALUE_MILLIS..=MAX_VALUE_MILLIS).contains(&weeks)
        && (MIN_VALUE_MILLIS..=MAX_VALUE_MILLIS).contains(&time))
    .then_some(time)
}

impl LogOutput {
    /// `writeKML`: every file "Create KML + gpx" leaves for the log whose `.kml` would be
    /// `filename` - `<log>.kml` - with text lines ended by `newline`. `zone` is the local time zone
    /// the GPX times are written in, and `modified` stamps the `.kmz`'s entries.
    ///
    /// # Errors
    ///
    /// A `CMD` line without a `CId` or `CNum` its format declares: the C# throws out of `writeKML`
    /// there, after the side files and before the `.kml`, so only those are returned.
    /// `// C#: ExtLibs/Utilities/LogOutput.cs:776-1163`
    pub fn write_kml(
        &mut self,
        filename: &Path,
        zone: Zone<'_>,
        newline: &'static str,
        modified: zip::DosTime,
    ) -> Result<KmlFiles, KmlFiles> {
        let dir = filename.parent().unwrap_or_else(|| Path::new(""));
        let stem = filename
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let beside = |suffix: &str| dir.join(format!("{stem}{suffix}"));
        let mut files = Vec::new();
        files.push((beside(".gpx"), self.gpx(zone).into_bytes()));
        if let Some(obs) = self.rinex(newline) {
            files.push((beside(".obs"), obs.into_bytes()));
        }
        for (index, text) in self.mission_files("CMD", newline).into_iter().enumerate() {
            files.push((beside(&format!("{index}wp.txt")), text.into_bytes()));
        }
        for (index, text) in self.mission_files("RALY", newline).into_iter().enumerate() {
            files.push((beside(&format!("{index}rally.txt")), text.into_bytes()));
        }
        if !self.paramlist.is_empty() {
            let mut names: Vec<(&String, &String)> = self.paramlist.iter().collect();
            names.sort_by(|a, b| netfmt::culture_compare(a.0, b.0));
            let mut text = String::new();
            for (name, value) in names {
                let _ = write!(text, "{name},{value}{newline}");
            }
            files.push((beside(".param"), text.into_bytes()));
        }
        let Some(kml) = self.kml(newline) else {
            return Err(KmlFiles {
                files,
                kml: Vec::new(),
            });
        };
        let kml = kml.into_bytes();
        let name = filename
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let entries = [
            zip::Entry {
                name: name.clone(),
                data: kml.clone(),
            },
            zip::Entry {
                name: PLANE_MODEL_NAME.to_owned(),
                data: PLANE_MODEL.to_vec(),
            },
        ];
        let kmz_name = name
            .to_lowercase()
            .replace(".log.kml", ".kmz")
            .replace(".bin.kml", ".kmz");
        match zip::write(&entries, modified) {
            Ok(kmz) => files.push((dir.join(kmz_name), kmz)),
            Err(_) => {
                return Err(KmlFiles {
                    files,
                    kml: Vec::new(),
                });
            }
        }
        Ok(KmlFiles { files, kml })
    }

    /// `writeGPX`: the attitude samples as a track and again as waypoints, ASCII, no indentation,
    /// no declaration. `<mode>` is always empty: nothing sets `Data.mode`.
    /// `// C#: ExtLibs/Utilities/LogOutput.cs:371-433`
    fn gpx(&self, zone: Zone<'_>) -> String {
        let mut out = String::from(
            "<gpx creator=\"Mission Planner\" xmlns=\"http://www.topografix.com/GPX/1/1\"><trk>",
        );
        if self.flightdata.is_empty() {
            out.push_str("<trkseg />");
        } else {
            out.push_str("<trkseg>");
            for data in &self.flightdata {
                let model = &data.model;
                let _ = write!(
                    out,
                    "<trkpt lat=\"{}\" lon=\"{}\"><ele>{}</ele><time>{}</time><course>{}</course>\
                     <roll>{}</roll><pitch>{}</pitch><mode /></trkpt>",
                    netfmt::double(model.latitude),
                    netfmt::double(model.longitude),
                    netfmt::double(model.altitude),
                    gpx_time(data.datetime, zone),
                    netfmt::double(model.heading),
                    netfmt::double(model.roll),
                    netfmt::double(model.tilt),
                );
            }
            out.push_str("</trkseg>");
        }
        out.push_str("</trk>");
        for (index, data) in self.flightdata.iter().enumerate() {
            let _ = write!(
                out,
                "<wpt lat=\"{}\" lon=\"{}\"><name>{index}</name></wpt>",
                netfmt::double(data.model.latitude),
                netfmt::double(data.model.longitude),
            );
        }
        out.push_str("</gpx>");
        out.chars()
            .map(|c| if c.is_ascii() { c } else { '?' })
            .collect()
    }

    /// `writeWPFile` (`CMD`) and `writeRallyFile` (`RALY`): a `QGC WPL 110` file per upload - a new
    /// one whenever the total changes or the number goes back - each line in the invariant culture.
    /// Whatever throws ends the writing there, keeping the files so far. (The C# leaves its last
    /// `StreamWriter` unclosed when that happens, so what it had buffered may not reach the disk.)
    /// `// C#: ExtLibs/Utilities/LogOutput.cs:443-541`
    fn mission_files(&mut self, kind: &str, newline: &str) -> Vec<String> {
        let (lines, total, number) = if kind == "CMD" {
            (self.cmdraw.clone(), "CTot", "CNum")
        } else {
            (self.ralyraw.clone(), "Tot", "Seq")
        };
        let mut files: Vec<String> = Vec::new();
        let mut current_total = -1.0;
        let mut last_seen = -1.0;
        for line in lines {
            let items = split(&line);
            let (Some(ctot), Some(cnum)) = (
                self.number(&items, kind, total),
                self.number(&items, kind, number),
            ) else {
                break;
            };
            #[allow(clippy::float_cmp)]
            if ctot != current_total || cnum < last_seen {
                current_total = ctot;
                files.push(format!("QGC WPL 110{newline}"));
            }
            last_seen = cnum;
            let Some(text) = self.mission_line(kind, &items, cnum) else {
                break;
            };
            // Before any file, `sw` is null and the write throws.
            let Some(file) = files.last_mut() else {
                break;
            };
            file.push_str(&text);
            file.push_str(newline);
        }
        files
    }

    /// The rest of one waypoint or rally line, as its tab-separated text.
    fn mission_line(&mut self, kind: &str, items: &[&str], cnum: f64) -> Option<String> {
        let d = netfmt::double;
        if kind == "CMD" {
            let id = self.number(items, kind, "CId")?;
            let p1 = self.number(items, kind, "Prm1")?;
            let p2 = self.number(items, kind, "Prm2")?;
            let p3 = self.number(items, kind, "Prm3")?;
            let p4 = self.number(items, kind, "Prm4")?;
            let lat = self.number(items, kind, "Lat")?;
            let lng = self.number(items, kind, "Lng")?;
            let alt = self.number(items, kind, "Alt")?;
            let frame = if self.dflog.find_message_offset("CMD", "Frame").is_some() {
                self.number(items, kind, "Frame")?
            } else {
                3.0
            };
            Some(
                [
                    d(cnum),
                    "0".to_owned(),
                    d(frame),
                    d(id),
                    d(p1),
                    d(p2),
                    d(p3),
                    d(p4),
                    d(lat),
                    d(lng),
                    d(alt),
                    "1".to_owned(),
                ]
                .join("\t"),
            )
        } else {
            let lat = self.number(items, kind, "Lat")?;
            let lng = self.number(items, kind, "Lng")?;
            let alt = self.number(items, kind, "Alt")?;
            Some(
                [
                    d(cnum),
                    "0".to_owned(),
                    d(3.0),
                    "5100".to_owned(),
                    "0".to_owned(),
                    "0".to_owned(),
                    "0".to_owned(),
                    "0".to_owned(),
                    d(lat),
                    d(lng),
                    d(alt),
                    "1".to_owned(),
                ]
                .join("\t"),
            )
        }
    }

    /// `writeRinex`: the raw GNSS records as RINEX 3.02 observations, or nothing when there are
    /// none. Whatever throws ends the file there.
    /// `// C#: ExtLibs/Utilities/LogOutput.cs:543-722`
    fn rinex(&mut self, newline: &str) -> Option<String> {
        if self.gpsrawdata.is_empty() {
            return None;
        }
        let mut out = String::from(RINEX_HEADER);
        out.push_str(newline);
        let mut last_time: Option<i64> = None;
        let mut weekms = 0.0f64;
        let mut satellites = 0.0f64;
        let mut week = 0i32;
        for line in self.gpsrawdata.clone() {
            let items = split(&line);
            let kind = items.first().copied().unwrap_or("");
            let Some(()) = self.rinex_line(
                kind,
                &items,
                &mut weekms,
                &mut week,
                &mut satellites,
                &mut last_time,
                &mut out,
                newline,
            ) else {
                break;
            };
        }
        Some(out)
    }

    /// One raw GNSS line; `None` where the C# throws. `continue` is `Some`.
    #[allow(clippy::too_many_arguments)]
    fn rinex_line(
        &mut self,
        kind: &str,
        items: &[&str],
        weekms: &mut f64,
        week: &mut i32,
        satellites: &mut f64,
        last_time: &mut Option<i64>,
        out: &mut String,
        newline: &str,
    ) -> Option<()> {
        let mut sv = -1.0;
        let mut cp_mes = -1.0;
        let mut pr_mes = -1.0;
        let mut do_mes = -1.0;
        let mut cno = -1.0;
        let mut lli = -1.0;
        let mut gnss = 0;
        let int = |this: &mut Self, name: &str, field: &str| -> Option<i32> {
            let index = this.dflog.find_message_offset(name, field)?;
            netfmt::parse_i32(items.get(index)?)
        };
        if kind.starts_with("GRAW") {
            *weekms = self.number(items, "GRAW", "WkMS")?;
            *week = int(self, "GRAW", "Week")?;
            *satellites = self.number(items, "GRAW", "numSV")?;
            sv = self.number(items, "GRAW", "sv")?;
            cp_mes = self.number(items, "GRAW", "cpMes")?;
            pr_mes = self.number(items, "GRAW", "prMes")?;
            do_mes = self.number(items, "GRAW", "doMes")?;
            self.number(items, "GRAW", "mesQI")?;
            cno = self.number(items, "GRAW", "cno")?;
            lli = self.number(items, "GRAW", "lli")?;
            if sv > 32.0 {
                gnss = 1;
            }
        } else if kind.starts_with("GRXH") {
            *weekms = self.number(items, "GRXH", "rcvTime")? * 1000.0;
            *week = int(self, "GRXH", "week")?;
            *satellites = self.number(items, "GRXH", "numMeas")?;
            return Some(());
        } else if kind.starts_with("GRXS") {
            sv = self.number(items, "GRXS", "sv")?;
            cp_mes = self.number(items, "GRXS", "cpMes")?;
            pr_mes = self.number(items, "GRXS", "prMes")?;
            do_mes = self.number(items, "GRXS", "doMes")?;
            gnss = int(self, "GRXS", "gnss")?;
            cno = self.number(items, "GRXS", "cno")?;
            self.number(items, "GRXS", "lock")?;
            lli = 0.0;
        }
        let time = from_gps(*week, *weekms / 1000.0)?;
        if *week == 0 {
            return Some(());
        }
        if *last_time != Some(time) {
            let seconds = time.div_euclid(1000);
            let millis = time.rem_euclid(1000);
            let (year, month, day) = civil_from_days(seconds.div_euclid(86_400));
            let of_day = seconds.rem_euclid(86_400);
            #[allow(clippy::cast_precision_loss)]
            let second = (of_day % 60) as f64 + millis as f64 / 1000.0;
            let _ = write!(
                out,
                "> {} {} {} {} {}{}  {}{}{newline}",
                right(&year.to_string(), 4),
                right(&month.to_string(), 2),
                right(&day.to_string(), 2),
                right(&(of_day / 3600).to_string(), 2),
                right(&(of_day / 60 % 60).to_string(), 2),
                right(&fixed(second, 7), 11),
                right("0", 1),
                right(&netfmt::double(*satellites), 3),
            );
            *last_time = Some(time);
        }
        let system = match gnss {
            1 => {
                sv -= 100.0;
                "S"
            }
            2 => "E",
            3 => "C",
            4 => "I",
            5 => {
                sv -= 192.0;
                "J"
            }
            6 => "R",
            _ => "G",
        };
        if sv <= 0.0 {
            return Some(());
        }
        // `(int)(cno / 6)`: out of range, x86 gives int.MinValue, which the Max makes 1.
        let sixth = cno / 6.0;
        #[allow(clippy::cast_possible_truncation)]
        let truncated = if sixth.is_nan() || !(-2_147_483_648.0..2_147_483_648.0).contains(&sixth) {
            i32::MIN
        } else {
            sixth as i32
        };
        let strength = truncated.clamp(1, 9);
        let _ = write!(
            out,
            "{system}{}{}{}{}{}{}{}{}{}{}{}{}{}{newline}",
            right(&two_digits(sv), 2),
            right(&fixed(pr_mes, 3), 14),
            right(&netfmt::double(lli), 1),
            right(&strength.to_string(), 1),
            right(&fixed(cp_mes, 3), 14),
            right(" ", 1),
            right(" ", 1),
            right(&netfmt::double(do_mes), 14),
            right(" ", 1),
            right(" ", 1),
            right(&fixed(cno, 3), 14),
            right(" ", 1),
            right(" ", 1),
        );
        Some(())
    }

    /// The `.kml`: KMLib's `KMLRoot` as XmlSerializer writes it, or `None` where `writeKML` throws
    /// on a `CMD` line. `// C#: ExtLibs/Utilities/LogOutput.cs:809-1101`
    fn kml(&mut self, newline: &'static str) -> Option<String> {
        let mut xml = Xml::new(newline);
        xml.start(
            "kml",
            &[
                ("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance"),
                ("xmlns:xsd", "http://www.w3.org/2001/XMLSchema"),
            ],
        );
        xml.start("Document", &[]);
        style(&mut xml, Some("yellowLineGreenPoly"), &PATH_STYLE);
        style(&mut xml, Some("spray"), &SPRAY_STYLE);
        xml.start("Folder", &[]);
        xml.leaf("name", "Log");

        // One placemark per flight path segment, coloured by its index.
        for (g, points) in self.position.iter().enumerate() {
            let Some(points) = points else { continue };
            let mode = self.modelist.get(g).map_or("", String::as_str);
            let name = format!("{g} Flight Path {mode}");
            let colour = COLOURS
                .get(g % (COLOURS.len() - 1))
                .copied()
                .unwrap_or("FF0000FF");
            let parts = [StylePart::Line {
                color: colour,
                width: 4.0,
            }];
            line_placemark(
                &mut xml,
                &LinePlacemark {
                    name: &name,
                    style_url: Some("#yellowLineGreenPoly"),
                    style: Some((None, &parts)),
                    extrude_mode: Some("absolute"),
                    coordinates: points,
                },
            );
        }

        // The POS path, thinned, in placemarks of at most 20001 points.
        let mut placemark_name = "POS Message";
        let mut coordinates: Vec<Point3D> = Vec::new();
        let mut last_point = Point3D::default();
        let mut last = (0.0, 0.0);
        for &(lat, lng, alt) in &self.positions {
            let point = Point3D {
                x: lng,
                y: lat,
                z: alt,
            };
            if distance(lat, lng, last.0, last.1) < 0.1
                && last_point.z >= point.z - 0.3
                && last_point.z <= point.z + 0.3
            {
                continue;
            }
            coordinates.push(point);
            last_point = point;
            last = (lat, lng);
            if coordinates.len() > 20_000 {
                line_placemark(
                    &mut xml,
                    &LinePlacemark {
                        name: placemark_name,
                        style_url: None,
                        style: Some((Some("yellowLineGreenPoly"), &PATH_STYLE)),
                        extrude_mode: None,
                        coordinates: &coordinates,
                    },
                );
                placemark_name = "POS Message - extra";
                coordinates.clear();
                last_point = Point3D::default();
                last = (0.0, 0.0);
            }
        }
        line_placemark(
            &mut xml,
            &LinePlacemark {
                name: placemark_name,
                style_url: None,
                style: Some((Some("yellowLineGreenPoly"), &PATH_STYLE)),
                extrude_mode: None,
                coordinates: &coordinates,
            },
        );

        // The aircraft, one per sample that moved; the Waypoints folder is filled before either
        // is written, so a CMD line that throws leaves no .kml at all.
        let waypoints = self.waypoint_paths()?;
        xml.start("Folder", &[]);
        xml.leaf("name", "Planes");
        let mut a = 0usize;
        let mut last_model: Option<Model> = None;
        for data in &self.flightdata {
            let model = data.model;
            if model.latitude == 0.0 {
                continue;
            }
            if last_model.is_some_and(|last| last.same_place(&model)) {
                continue;
            }
            // `new KmlPoint((float)lng, (float)lat, (float)alt)` refuses a place off the globe.
            #[allow(clippy::cast_possible_truncation)]
            let (lng, lat) = (model.longitude as f32, model.latitude as f32);
            if !(-180.0..=180.0).contains(&lng) || !(-90.0..=90.0).contains(&lat) {
                last_model = Some(model);
                a += 1;
                continue;
            }
            xml.start("Placemark", &[]);
            xml.leaf("name", &format!("Plane {a}"));
            xml.leaf("visibility", "0");
            if let (Some(n2), Some(n3), Some(n4), Some(n5)) = (
                data.ntun.get(2),
                data.ntun.get(3),
                data.ntun.get(4),
                data.ntun.get(5),
            ) {
                let description = format!(
                    "<![CDATA[\r\n              <table>\r\n                \
                     <tr><td>Roll: {} </td></tr>\r\n                \
                     <tr><td>Pitch: {} </td></tr>\r\n                \
                     <tr><td>Yaw: {} </td></tr>\r\n                \
                     <tr><td>WP dist {n2} </td></tr>\r\n\t\t\t\t\
                     <tr><td>tar bear {n3} </td></tr>\r\n\t\t\t\t\
                     <tr><td>nav bear {n4} </td></tr>\r\n\t\t\t\t\
                     <tr><td>alt error {n5} </td></tr>\r\n              \
                     </table>\r\n            ]]>",
                    netfmt::double(model.roll),
                    netfmt::double(model.tilt),
                    netfmt::double(model.heading),
                );
                xml.leaf("description", &description);
            }
            xml.start("Model", &[]);
            xml.leaf("altitudeMode", "absolute");
            xml.start("Location", &[]);
            xml.leaf("latitude", &xml_double(model.latitude));
            xml.leaf("longitude", &xml_double(model.longitude));
            xml.leaf("altitude", &xml_double(model.altitude));
            xml.end("Location");
            xml.start("Orientation", &[]);
            xml.leaf("heading", &xml_double(model.heading));
            xml.leaf("tilt", &xml_double(model.tilt));
            xml.leaf("roll", &xml_double(model.roll));
            xml.end("Orientation");
            xml.start("Scale", &[]);
            xml.leaf("x", &xml_single(2.0));
            xml.leaf("y", &xml_single(2.0));
            xml.leaf("z", &xml_single(2.0));
            xml.end("Scale");
            xml.start("Link", &[]);
            xml.leaf("href", PLANE_MODEL_NAME);
            xml.end("Link");
            xml.end("Model");
            xml.end("Placemark");
            last_model = Some(model);
            a += 1;
        }
        xml.end("Folder");

        xml.start("Folder", &[]);
        xml.leaf("name", "Waypoints");
        for (name, points) in &waypoints {
            line_placemark(
                &mut xml,
                &LinePlacemark {
                    name,
                    style_url: None,
                    style: None,
                    extrude_mode: Some("relativeToGround"),
                    coordinates: points,
                },
            );
        }
        xml.end("Folder");

        xml.end("Folder");
        xml.end("Document");
        xml.end("kml");
        Some(xml.out)
    }

    /// The mission as flown paths: a placemark per upload - a new one whenever the number goes
    /// back - of every `CMD` up to `MAV_CMD.LAST` with a place, the first at zero altitude. `None`
    /// where the C# throws: a `CMD` without the `CId` or `CNum` its format declares.
    /// `// C#: ExtLibs/Utilities/LogOutput.cs:968-1026`
    fn waypoint_paths(&mut self) -> Option<Vec<(&'static str, Vec<Point3D>)>> {
        let mut paths = Vec::new();
        let mut coordinates: Vec<Point3D> = Vec::new();
        let mut last = 0;
        for line in self.cmdraw.clone() {
            let item = self.dflog.item_from_line(&line);
            let id = netfmt::parse_i32(item.get(&mut self.dflog, "CId")?)?;
            if id > MAV_CMD_LAST {
                continue;
            }
            let number = netfmt::parse_i32(item.get(&mut self.dflog, "CNum")?)?;
            if number < last {
                if !coordinates.is_empty() {
                    paths.push(("Waypoints ", std::mem::take(&mut coordinates)));
                }
                coordinates.clear();
            }
            last = number;
            let lng = netfmt::parse_double(item.get(&mut self.dflog, "Lng")?)?;
            let lat = netfmt::parse_double(item.get(&mut self.dflog, "Lat")?)?;
            let mut alt = netfmt::parse_double(item.get(&mut self.dflog, "Alt")?)?;
            if number == 0 {
                alt = 0.0;
            }
            if lat == 0.0 && lng == 0.0 {
                continue;
            }
            coordinates.push(Point3D {
                x: lng,
                y: lat,
                z: alt,
            });
        }
        if !coordinates.is_empty() {
            paths.push(("Waypoints", coordinates));
        }
        Some(paths)
    }
}

/// The RINEX header, line for line as the C# source holds it - with `\r\n` between its lines on
/// every platform, the source file's own line endings inside a verbatim string.
/// `// C#: ExtLibs/Utilities/LogOutput.cs:555-569`
const RINEX_HEADER: &str = concat!(
    "     3.02           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE\r\n",
    "                                                            MARKER NAME         \r\n",
    "                                                            MARKER NUMBER       \r\n",
    "                                                            MARKER TYPE         \r\n",
    "                                                            OBSERVER / AGENCY   \r\n",
    "                                                            REC # / TYPE / VERS \r\n",
    "                                                            ANT # / TYPE        \r\n",
    "        0.0000        0.0000        0.0000                  APPROX POSITION XYZ \r\n",
    "        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N\r\n",
    "G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \r\n",
    "S    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \r\n",
    "R    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \r\n",
    "E    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \r\n",
    "G                                                           SYS / PHASE SHIFT   \r\n",
    "                                                            END OF HEADER       ",
);

/// Why "Create KML + gpx" wrote nothing, or less than everything.
#[derive(Debug, thiserror::Error)]
pub enum DflogKmlError {
    /// The log could not be read, or a file could not be written.
    #[error("{0}")]
    Io(#[from] std::io::Error),
    /// `writeKML` threw on a `CMD` line: the side files are written, the `.kmz` is not.
    #[error("the log's mission could not be read; {} files written, no .kmz", .0.len())]
    Mission(Vec<PathBuf>),
}

/// Every line of a log as `but_dflogtokml_Click` feeds them to `processLine`: a `.bin` (by its
/// name) through `DFLogBuffer`, anything else as text, `StreamReader.ReadLine` by
/// `StreamReader.ReadLine`. `// C#: GCSViews/FlightData.cs:1154-1186`
pub fn process_log(log: &Path, data: &[u8], mode_name: ModeName<'_>, output: &mut LogOutput) {
    if log.to_string_lossy().to_lowercase().ends_with(".bin") {
        let mut buffer = DfLogBuffer::new(data, mode_name);
        for index in 0..buffer.count() {
            let line = buffer.line(index);
            output.process_line(&line);
        }
    } else {
        let text = String::from_utf8_lossy(data);
        let text = text.strip_prefix('\u{feff}').unwrap_or(&text);
        let mut rest = text;
        while !rest.is_empty() {
            let end = rest.find(['\r', '\n']).unwrap_or(rest.len());
            output.process_line(rest.get(..end).unwrap_or_default());
            let after = rest.get(end..).unwrap_or_default();
            rest = after
                .strip_prefix("\r\n")
                .or_else(|| after.strip_prefix('\r'))
                .or_else(|| after.strip_prefix('\n'))
                .unwrap_or(after);
        }
    }
}

/// The local time now as a zip entry's stamp, `DateTime.Now`.
fn dos_now(zone: Zone<'_>) -> zip::DosTime {
    let now = i64::try_from(mp_log::now_micros() / 1000).unwrap_or(0);
    let local = now + i64::from(zone(now)) * 1000;
    let seconds = local.div_euclid(1000);
    let (year, month, day) = civil_from_days(seconds.div_euclid(86_400));
    let of_day = seconds.rem_euclid(86_400);
    let small = |v: i64| u16::try_from(v).unwrap_or(0);
    zip::DosTime {
        year: small(year),
        month: u16::try_from(month).unwrap_or(1),
        day: u16::try_from(day).unwrap_or(1),
        hour: small(of_day / 3600),
        minute: small(of_day / 60 % 60),
        second: small(of_day % 60),
    }
}

/// Create KML + gpx (`but_dflogtokml`) for one log: reads it, and writes beside it everything
/// `writeKML(<log>.kml)` writes - `<log>.gpx`, `<log>.obs` if it has raw GNSS, `<log><n>wp.txt`,
/// `<log><n>rally.txt`, `<log>.param`, and `<name>.kmz` - returning where. `mode_name` names
/// flight modes as for "Convert .Bin to .Log" ([`mp_log::convert::flight_mode_name`] is the
/// application's), and `zone` is the local time zone for the GPX times.
///
/// # Errors
///
/// The log cannot be read or a file cannot be written; or [`DflogKmlError::Mission`] when the
/// C# would throw out of `writeKML`, after the side files.
/// `// C#: GCSViews/FlightData.cs:1135-1197; ExtLibs/Utilities/LogOutput.cs:776-1163`
pub fn dflog_to_kml(
    log: &Path,
    mode_name: ModeName<'_>,
    zone: Zone<'_>,
) -> Result<Vec<PathBuf>, DflogKmlError> {
    let mut kml = log.as_os_str().to_owned();
    kml.push(".kml");
    dflog_to_kml_at(log, Path::new(&kml), mode_name, zone)
}

/// [`dflog_to_kml`] with the `.kml` - and so every file named after it - at `kml` rather than
/// `<log>.kml`: the same files, somewhere else.
///
/// # Errors
///
/// As [`dflog_to_kml`].
pub fn dflog_to_kml_at(
    log: &Path,
    kml: &Path,
    mode_name: ModeName<'_>,
    zone: Zone<'_>,
) -> Result<Vec<PathBuf>, DflogKmlError> {
    let data = std::fs::read(log)?;
    let mut output = LogOutput::new();
    process_log(log, &data, mode_name, &mut output);
    let written = output.write_kml(kml, zone, NEWLINE, dos_now(zone));
    let (files, complete) = match written {
        Ok(files) => (files.files, true),
        Err(files) => (files.files, false),
    };
    let mut paths = Vec::with_capacity(files.len());
    for (path, contents) in files {
        std::fs::write(&path, contents)?;
        paths.push(path);
    }
    if complete {
        Ok(paths)
    } else {
        Err(DflogKmlError::Mission(paths))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_formats_round_through_fifteen_digits() {
        assert_eq!(fixed(1.0005, 3), "1.001");
        assert_eq!(fixed(123.4565, 3), "123.457");
        assert_eq!(fixed(-0.0001, 3), "0.000");
        assert_eq!(fixed(9.99999, 3), "10.000");
        assert_eq!(fixed(-2.5, 3), "-2.500");
        assert_eq!(fixed(20_000_000.123_456_789, 7), "20000000.1234568");
        assert_eq!(fixed(1e20, 3), "100000000000000000000.000");
        assert_eq!(two_digits(2.5), "03");
        assert_eq!(two_digits(-1.0005), "-01");
        assert_eq!(two_digits(132.0), "132");
        assert_eq!(two_digits(0.0), "00");
    }

    #[test]
    fn gpx_times_carry_the_zone() {
        let utc = 1_786_858_871_000; // 2026-08-16T05:41:11Z
        assert_eq!(gpx_time(Some(utc), &|_| 0), "2026-08-16T05:41:11+00:00");
        assert_eq!(
            gpx_time(Some(utc), &|_| 36_000),
            "2026-08-16T15:41:11+10:00"
        );
        assert_eq!(
            gpx_time(Some(utc), &|_| -14_400),
            "2026-08-16T01:41:11-04:00"
        );
        assert_eq!(gpx_time(None, &|_| 0), "0001-01-01T00:00:00+00:00");
    }

    #[test]
    fn negative_zero_is_written_as_xml_convert_writes_it() {
        assert_eq!(xml_double(0.0 / -1.0), "-0");
        assert_eq!(netfmt::double(0.0 / -1.0), "0");
        assert_eq!(xml_double(f64::INFINITY), "INF");
        assert_eq!(xml_single(4.0), "4");
    }
}
