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

//! Where a log says the vehicle went: the routes Mission Planner's log browser puts on its map.
//!
//! `DrawMap` walks the log's position messages and builds one route per source, each its own
//! colour: the first GPS in blue, the second in green, a `GPSB` blend in yellow, the EKF's `POS`
//! in red, and the mission the vehicle logged (`CMD`) in indigo with a marker per waypoint, and a
//! photo marker where each `CAM` record says a picture was taken. This is the reading half -
//! which records count as a point of which route - with no map in it.
//! `// C#: Log/LogBrowse.cs:2191-2540, 2541-2784`

use crate::dataflash::{DataflashReader, LogMessage, Value, decode_record};

/// One point of a route.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoutePoint {
    /// The record's line: its place among the log's records, which is the grid's row and what
    /// `DrawMap` keeps in each route's `samples`.
    pub line: usize,
    /// The record's `TimeUS`, when it has one.
    pub time_us: Option<f64>,
    /// Degrees north.
    pub latitude: f64,
    /// Degrees east.
    pub longitude: f64,
    /// Course over ground in degrees, for a source that logs one.
    pub course: Option<f64>,
}

/// One mission item as the vehicle logged it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoggedCommand {
    /// The record's line.
    pub line: usize,
    /// Its place in the mission: `CNum`.
    pub seq: u16,
    /// How many items the mission had when it was logged: `CTot`, which older logs lack.
    pub total: Option<u16>,
    /// The MAVLink command: `CId`.
    pub command: u16,
    /// `Prm1` to `Prm4`.
    pub params: [f64; 4],
    /// Degrees north.
    pub latitude: f64,
    /// Degrees east.
    pub longitude: f64,
    /// Altitude, in the command's frame.
    pub altitude: f64,
    /// The MAVLink frame, which older logs do not record.
    pub frame: Option<u8>,
}

/// Which route a record's position belongs to.
///
/// The five `GMapRoute`s `DrawMap` adds, in its order, with the pen each is stroked with:
/// `Color.FromArgb(127, colour)`, two pixels wide.
/// `// C#: Log/LogBrowse.cs:2437-2505`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteKind {
    /// `route_`: the first GPS, `Color.Blue`.
    Gps,
    /// `routegps2_`: the second GPS, `Color.Green`.
    Gps2,
    /// `routegpsb_`: the blended GPS, `Color.Yellow`.
    Gpsb,
    /// `routepos_`: the EKF's position, `Color.Red`.
    Pos,
    /// `routecmd_`: the logged mission, `Color.Indigo`.
    Cmd,
}

impl RouteKind {
    /// Every route, in the order `DrawMap` adds them to the overlay - and so the order GMap
    /// draws them in, the last on top.
    pub const ALL: [Self; 5] = [Self::Gps, Self::Gps2, Self::Gpsb, Self::Pos, Self::Cmd];

    /// The pen's alpha: `Color.FromArgb(127, ...)`.
    pub const ALPHA: u8 = 127;

    /// The `System.Drawing.Color` the route's pen is made from, as `0xRRGGBB`.
    #[must_use]
    pub const fn colour(self) -> u32 {
        match self {
            Self::Gps => 0x00_00_ff,
            Self::Gps2 => 0x00_80_00,
            Self::Gpsb => 0xff_ff_00,
            Self::Pos => 0xff_00_00,
            Self::Cmd => 0x4b_00_82,
        }
    }
}

/// Every route a log holds, as `DrawMap` sorts its records into them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Routes {
    /// The first GPS: `GPS` records of instance 0, or with no instance, that have a 3D fix.
    pub gps: Vec<RoutePoint>,
    /// The second GPS: `GPS2` records, or `GPS` records of instance 1, that have a 3D fix.
    pub gps2: Vec<RoutePoint>,
    /// The blended GPS: `GPSB` records, or `GPS` records of instance 2, that have a 3D fix.
    pub gpsb: Vec<RoutePoint>,
    /// The EKF's position estimate: `POS` records.
    pub pos: Vec<RoutePoint>,
    /// The logged mission, one entry per waypoint number, in the order first logged: the indigo
    /// route runs through these, and each has a waypoint marker.
    pub commands: Vec<LoggedCommand>,
    /// `CMD` records for a waypoint number already seen: `DrawMap` gives each a marker of its
    /// own and leaves it out of the route.
    pub repeats: Vec<LoggedCommand>,
    /// Where each `CAM` record says a photo was taken: a `GMapMarkerPhoto` each.
    pub cameras: Vec<RoutePoint>,
}

impl Routes {
    /// The points of one route.
    #[must_use]
    pub fn route(&self, kind: RouteKind) -> Vec<RoutePoint> {
        match kind {
            RouteKind::Gps => self.gps.clone(),
            RouteKind::Gps2 => self.gps2.clone(),
            RouteKind::Gpsb => self.gpsb.clone(),
            RouteKind::Pos => self.pos.clone(),
            RouteKind::Cmd => self
                .commands
                .iter()
                .map(|command| RoutePoint {
                    line: command.line,
                    time_us: None,
                    latitude: command.latitude,
                    longitude: command.longitude,
                    course: None,
                })
                .collect(),
        }
    }

    /// Only what lies between two lines, both included: `DrawMap(startline, endline)`, which the
    /// chart's zoom asks for so the map shows the stretch of flight the chart does.
    ///
    /// A mission item is kept or dropped with its record, as the C# walks the records in that
    /// range and nothing else: one whose first record falls outside the range is not a route
    /// point there, and its first repeat inside the range is - it is the first `DrawMap` sees.
    /// `// C#: Log/LogBrowse.cs:2234-2240`
    #[must_use]
    pub fn between(&self, start: usize, end: usize) -> Self {
        let inside = |line: usize| (start..=end).contains(&line);
        let points = |list: &[RoutePoint]| -> Vec<RoutePoint> {
            list.iter()
                .copied()
                .filter(|point| inside(point.line))
                .collect()
        };
        let mut every: Vec<LoggedCommand> = self
            .commands
            .iter()
            .chain(&self.repeats)
            .copied()
            .filter(|command| inside(command.line))
            .collect();
        every.sort_by_key(|command| command.line);
        let mut commands: Vec<LoggedCommand> = Vec::new();
        let mut repeats = Vec::new();
        for command in every {
            if commands.iter().any(|seen| seen.seq == command.seq) {
                repeats.push(command);
            } else {
                commands.push(command);
            }
        }
        Self {
            gps: points(&self.gps),
            gps2: points(&self.gps2),
            gpsb: points(&self.gpsb),
            pos: points(&self.pos),
            commands,
            repeats,
            cameras: points(&self.cameras),
        }
    }

    /// Every point of every route: what `ZoomAndCenterRoutes` fits the map to.
    /// `// C#: Log/LogBrowse.cs:2515`
    pub fn places(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.gps
            .iter()
            .chain(&self.gps2)
            .chain(&self.gpsb)
            .chain(&self.pos)
            .map(|point| (point.latitude, point.longitude))
            .chain(
                self.commands
                    .iter()
                    .map(|command| (command.latitude, command.longitude)),
            )
    }
}

/// The message types `DrawMap` reads into `gpscache` - the last three of which it then does
/// nothing with.
/// `// C#: Log/LogBrowse.cs:2201-2203`
pub const DRAWMAP_TYPES: [&str; 9] = [
    "GPS", "POS", "GPS2", "GPSB", "CMD", "CAM", "TRIG", "SIM", "RALY",
];

/// Sorts a log's position records into routes.
///
/// The rules are `DrawMap`'s and `getPointLatLng`'s:
///
/// - a `GPS` point needs `Lat`, `Lng` and a `Status` of 3 or better - a GPS without a 3D fix
///   reports a position of nothing in particular. Instance 0, or none, is the first GPS's route,
///   instance 1 the second's and instance 2 the blend's; an older log's `GPS2` and `GPSB` types
///   are the second and the blend, by the same rule over their own fields;
/// - a `POS` point needs a latitude and longitude that are on the planet;
/// - a `CMD` needs a position that is not 0,0 - which is how a command with no position logs -
///   and joins the route once per waypoint number, since a mission re-sent to the vehicle logs
///   again; a repeat is a marker only;
/// - a `CAM` needs a `Lat` and `Lng` that are not 0.
///
/// **One divergence.** The C# looks for a repeated waypoint number with
/// `foreach (GMapMarkerWP m in mapoverlay.Markers)`, and once a `CAM` has put a
/// `GMapMarkerPhoto` in that list the cast throws, the catch around the walk swallows it, and the
/// map keeps whatever it showed before - no routes at all for a log that triggers a camera
/// before a `CMD` is logged. The markers are kept apart here, and such a log is drawn.
/// `// C#: Log/LogBrowse.cs:2234-2433, 2541-2784`
#[must_use]
pub fn routes(data: &[u8]) -> Routes {
    let gps_instance = crate::plot::instance_fields(data).get("GPS").cloned();
    let mut routes = Routes::default();
    let mut reader = DataflashReader::new(data);
    let mut line = 0usize;
    while let Some(record) = reader.next_record() {
        let this_line = line;
        line += 1;
        let wanted = reader
            .formats()
            .get(&record.msg_type)
            .is_some_and(|format| DRAWMAP_TYPES.contains(&format.name.as_str()));
        if !wanted || record.msg_type == crate::dataflash::FMT_TYPE {
            continue;
        }
        let Some(message) = data
            .get(record.offset..)
            .and_then(|bytes| decode_record(reader.formats(), bytes))
        else {
            continue;
        };
        route_record(&mut routes, &message, this_line, gps_instance.as_deref());
    }
    routes
}

/// Puts one record where `DrawMap` puts it.
///
/// `gps_instance` is the label of the `GPS` format's instance field, from its `FMTU`.
/// `// C#: Log/LogBrowse.cs:2242-2433`
pub fn route_record(
    routes: &mut Routes,
    message: &LogMessage,
    line: usize,
    gps_instance: Option<&str>,
) {
    match message.name.as_str() {
        "GPS" => {
            // `item.instance`: the instance field's value, or "" for a type without one.
            #[allow(clippy::cast_possible_truncation)] // an instance number is a small integer
            let instance = gps_instance
                .and_then(|label| message.field(label))
                .and_then(Value::as_f64)
                .map(|value| value as i64);
            let route = match instance {
                None | Some(0) => &mut routes.gps,
                Some(1) => &mut routes.gps2,
                Some(2) => &mut routes.gpsb,
                Some(_) => return,
            };
            if let Some(point) = gps_point(message, line) {
                route.push(point);
            }
        }
        "GPS2" => {
            if let Some(point) = gps_point(message, line) {
                routes.gps2.push(point);
            }
        }
        "GPSB" => {
            if let Some(point) = gps_point(message, line) {
                routes.gpsb.push(point);
            }
        }
        "POS" => {
            if let Some(point) = position(message, line, None) {
                routes.pos.push(point);
            }
        }
        "CMD" => {
            if let Some(command) = command(message, line) {
                if routes.commands.iter().any(|seen| seen.seq == command.seq) {
                    routes.repeats.push(command);
                } else {
                    routes.commands.push(command);
                }
            }
        }
        "CAM" => {
            // The last branch of `getPointLatLng`: no range check, and a coordinate that is
            // exactly 0 is no place.
            let latitude = message.field("Lat").and_then(Value::as_f64);
            let longitude = message.field("Lng").and_then(Value::as_f64);
            if let (Some(latitude), Some(longitude)) = (latitude, longitude)
                && latitude != 0.0
                && longitude != 0.0
            {
                routes.cameras.push(RoutePoint {
                    line,
                    time_us: message.field("TimeUS").and_then(Value::as_f64),
                    latitude,
                    longitude,
                    course: None,
                });
            }
        }
        // `TRIG`, `SIM` and `RALY` are read and fall through every branch.
        _ => {}
    }
}

/// A point of a GPS's route, if the record has a fix.
fn gps_point(message: &LogMessage, line: usize) -> Option<RoutePoint> {
    let status = message.field("Status").and_then(Value::as_f64)?;
    if status < 3.0 {
        return None;
    }
    position(message, line, message.field("GCrs").and_then(Value::as_f64))
}

/// A record's `Lat` and `Lng` as a point, if they are a place on the planet.
fn position(message: &LogMessage, line: usize, course: Option<f64>) -> Option<RoutePoint> {
    let latitude = message.field("Lat").and_then(Value::as_f64)?;
    let longitude = message.field("Lng").and_then(Value::as_f64)?;
    if !(latitude.abs() <= 90.0 && longitude.abs() <= 180.0) {
        return None;
    }
    Some(RoutePoint {
        line,
        time_us: message.field("TimeUS").and_then(Value::as_f64),
        latitude,
        longitude,
        course,
    })
}

/// A logged mission item, if it has a position.
fn command(message: &LogMessage, line: usize) -> Option<LoggedCommand> {
    let at = position(message, line, None)?;
    if at.latitude == 0.0 || at.longitude == 0.0 {
        return None;
    }
    let number = |label: &str| message.field(label).and_then(Value::as_f64);
    let whole = |label: &str| number(label).and_then(small_whole);
    Some(LoggedCommand {
        line,
        seq: whole("CNum")?,
        total: whole("CTot"),
        command: whole("CId")?,
        params: [
            number("Prm1").unwrap_or(0.0),
            number("Prm2").unwrap_or(0.0),
            number("Prm3").unwrap_or(0.0),
            number("Prm4").unwrap_or(0.0),
        ],
        latitude: at.latitude,
        longitude: at.longitude,
        altitude: number("Alt").unwrap_or(0.0),
        frame: whole("Frame").and_then(|frame| u8::try_from(frame).ok()),
    })
}

/// A field that holds a small whole number - a waypoint number, a command id - as one.
fn small_whole(value: f64) -> Option<u16> {
    if !(0.0..=f64::from(u16::MAX)).contains(&value) {
        return None;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // range-checked above
    Some(value as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testlog::{fixed, fmt, record};

    fn testdata(name: &str) -> Vec<u8> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name);
        mp_os::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
    }

    /// The healthy fixture was logged on a bench: its GPS never had a fix, so there is no route.
    ///
    /// Worth pinning, because the obvious test - "the fixture has a flight path" - is false for
    /// this file, and a reader that ignored `Status` would pass it with ninety-one points at 0,0
    /// off the coast of Africa.
    #[test]
    fn a_gps_without_a_fix_makes_no_route() {
        let routes = routes(&testdata("dataflash.bin"));
        assert!(routes.gps.is_empty(), "{} points", routes.gps.len());
        assert!(routes.pos.is_empty(), "the fixture logs no POS");
    }

    /// The same log does carry the mission it was given, and that is what its map shows.
    #[test]
    fn the_logged_mission_is_read_once_per_waypoint() {
        let routes = routes(&testdata("dataflash.bin"));
        let numbers: Vec<u16> = routes.commands.iter().map(|command| command.seq).collect();
        assert_eq!(
            numbers,
            vec![1, 2, 3, 4, 5, 6],
            "item 0, home, is logged at 0,0 and is left out"
        );
        for command in &routes.commands {
            assert!(
                (command.latitude + 27.51).abs() < 0.05
                    && (command.longitude - 153.01).abs() < 0.05,
                "{command:?} is not where the rest of the mission is"
            );
        }
        assert_eq!(
            routes.commands[0].command, 22,
            "the mission starts with a takeoff"
        );
        assert_eq!(routes.commands[0].frame, Some(3));
    }

    /// The damaged fixture came off a vehicle with a fix, and its route is a real one: SITL's,
    /// at Canberra Model Aircraft Club.
    ///
    /// It holds two boots of the same vehicle, one after the other: `TimeUS` climbs through the
    /// first, goes back to the start of the second, and climbs again. The route keeps the log's
    /// order, as `DrawMap` does, so time runs forward everywhere but that one seam. (408 fixes
    /// and 815 positions, as a header walk in Python counts them, `tools/sitl/make-damaged-log.py`
    /// having made the log.)
    #[test]
    fn a_gps_with_a_fix_makes_a_route_in_log_order() {
        let routes = routes(&testdata("dataflash_damaged.bin"));
        assert_eq!(routes.gps.len(), 408);
        assert_eq!(routes.pos.len(), 815);
        for point in routes.gps.iter().chain(&routes.pos) {
            assert!(
                (point.latitude + 35.3633).abs() < 0.01
                    && (point.longitude - 149.1652).abs() < 0.01,
                "{point:?} is not where this vehicle was"
            );
        }
        let times: Vec<f64> = routes
            .gps
            .iter()
            .filter_map(|point| point.time_us)
            .collect();
        assert_eq!(times.len(), routes.gps.len(), "every GPS point has a time");
        let backwards = times.windows(2).filter(|pair| pair[1] < pair[0]).count();
        assert_eq!(
            backwards, 1,
            "time runs forward within each of the two boots"
        );
    }

    const GPS: u8 = 140;
    const FMTU: u8 = 141;

    fn gps(time_us: u64, instance: u8, status: u8, lat: f64, lng: f64, course: f32) -> Vec<u8> {
        let mut payload = time_us.to_le_bytes().to_vec();
        payload.push(instance);
        payload.push(status);
        payload.extend(0u32.to_le_bytes()); // GMS
        payload.extend(0u16.to_le_bytes()); // GWk
        payload.push(12); // NSats
        payload.extend(80i16.to_le_bytes()); // HDop
        #[allow(clippy::cast_possible_truncation)]
        payload.extend(((lat * 1e7).round() as i32).to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        payload.extend(((lng * 1e7).round() as i32).to_le_bytes());
        payload.extend(58_400i32.to_le_bytes()); // Alt
        payload.extend(3f32.to_le_bytes()); // Spd
        payload.extend(course.to_le_bytes()); // GCrs
        payload.extend(0f32.to_le_bytes()); // VZ
        payload.extend(0f32.to_le_bytes()); // Yaw
        payload.push(1); // U
        record(GPS, &payload)
    }

    /// A small flight at Canberra, where SITL starts, with two GPSs and a moment without a fix.
    fn canberra() -> Vec<u8> {
        let mut log = fmt(
            GPS,
            51,
            "GPS",
            "QBBIHBcLLeffffB",
            "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U",
        );
        log.extend(fmt(
            FMTU,
            44,
            "FMTU",
            "QBNN",
            "TimeUS,FmtType,UnitIds,MultIds",
        ));
        let mut fmtu = 0u64.to_le_bytes().to_vec();
        fmtu.push(GPS);
        fmtu.extend(fixed("s#-------------", 16));
        fmtu.extend(fixed("F--------------", 16));
        log.extend(record(FMTU, &fmtu));

        log.extend(gps(1_000_000, 0, 1, 0.0, 0.0, 0.0)); // no fix yet
        log.extend(gps(1_200_000, 0, 3, -35.363_26, 149.165_24, 90.0));
        log.extend(gps(1_200_000, 1, 3, -35.0, 149.0, 0.0)); // the second GPS
        log.extend(gps(1_400_000, 0, 6, -35.363_26, 149.165_30, 91.5));
        log.extend(gps(1_600_000, 0, 4, -35.363_20, 149.165_36, 45.0));
        log
    }

    /// Only the first GPS with a fix is a point of the route, in order, with its course.
    #[test]
    fn the_first_gps_route_skips_no_fix_and_the_second_gps() {
        let routes = routes(&canberra());
        assert_eq!(routes.gps.len(), 3);
        for point in &routes.gps {
            assert!(
                (point.latitude + 35.363).abs() < 0.001
                    && (point.longitude - 149.165).abs() < 0.001,
                "{point:?} is not in Canberra"
            );
        }
        let times: Vec<Option<f64>> = routes.gps.iter().map(|point| point.time_us).collect();
        assert_eq!(
            times,
            vec![Some(1_200_000.0), Some(1_400_000.0), Some(1_600_000.0)]
        );
        assert_eq!(routes.gps[1].course, Some(91.5));
        // Each point keeps its record's line: FMT, FMT, FMTU, the fixless GPS, then these.
        let lines: Vec<usize> = routes.gps.iter().map(|point| point.line).collect();
        assert_eq!(lines, vec![4, 6, 7]);
    }

    /// `GPS` instance 1 is the green route, as `DrawMap`'s second branch has it, and instance 2
    /// the yellow one; an instance past that is on no route.
    #[test]
    fn the_second_gps_and_the_blend_are_routes_of_their_own() {
        let mut log = canberra();
        log.extend(gps(1_800_000, 1, 3, -35.1, 149.1, 0.0));
        log.extend(gps(1_800_000, 2, 3, -35.2, 149.2, 0.0));
        log.extend(gps(1_800_000, 2, 1, -35.2, 149.2, 0.0)); // the blend without a fix
        log.extend(gps(1_800_000, 3, 3, -35.3, 149.3, 0.0)); // no route of its own
        let routes = routes(&log);
        assert_eq!(routes.gps.len(), 3);
        let second: Vec<(f64, usize)> = routes
            .gps2
            .iter()
            .map(|point| (point.latitude, point.line))
            .collect();
        assert_eq!(second, vec![(-35.0, 5), (-35.1, 8)]);
        let blend: Vec<(f64, usize)> = routes
            .gpsb
            .iter()
            .map(|point| (point.latitude, point.line))
            .collect();
        assert_eq!(blend, vec![(-35.2, 9)]);
    }

    const GPS2: u8 = 142;
    const CMD: u8 = 143;
    const CAM: u8 = 144;

    /// An older log's `GPS2` type, a mission logged twice and a camera: every branch of `DrawMap`
    /// that puts something on the map.
    fn older() -> Vec<u8> {
        let mut log = fmt(GPS2, 24, "GPS2", "QBLLf", "TimeUS,Status,Lat,Lng,GCrs");
        log.extend(fmt(
            CMD,
            29,
            "CMD",
            "QHHHLLf",
            "TimeUS,CTot,CNum,CId,Lat,Lng,Alt",
        ));
        log.extend(fmt(CAM, 19, "CAM", "QLL", "TimeUS,Lat,Lng"));
        let gps2 = |time: u64, status: u8, lat: f64, lng: f64| {
            let mut payload = time.to_le_bytes().to_vec();
            payload.push(status);
            #[allow(clippy::cast_possible_truncation)]
            payload.extend(((lat * 1e7).round() as i32).to_le_bytes());
            #[allow(clippy::cast_possible_truncation)]
            payload.extend(((lng * 1e7).round() as i32).to_le_bytes());
            payload.extend(0f32.to_le_bytes());
            record(GPS2, &payload)
        };
        let cmd = |total: u16, number: u16, lat: f64, lng: f64| {
            let mut payload = 5u64.to_le_bytes().to_vec();
            payload.extend(total.to_le_bytes());
            payload.extend(number.to_le_bytes());
            payload.extend(16u16.to_le_bytes());
            #[allow(clippy::cast_possible_truncation)]
            payload.extend(((lat * 1e7).round() as i32).to_le_bytes());
            #[allow(clippy::cast_possible_truncation)]
            payload.extend(((lng * 1e7).round() as i32).to_le_bytes());
            payload.extend(20f32.to_le_bytes());
            record(CMD, &payload)
        };
        let cam = |lat: f64, lng: f64| {
            let mut payload = 9u64.to_le_bytes().to_vec();
            #[allow(clippy::cast_possible_truncation)]
            payload.extend(((lat * 1e7).round() as i32).to_le_bytes());
            #[allow(clippy::cast_possible_truncation)]
            payload.extend(((lng * 1e7).round() as i32).to_le_bytes());
            record(CAM, &payload)
        };
        log.extend(cmd(3, 0, 0.0, 0.0)); // line 3: home, at 0,0: no place
        log.extend(cmd(3, 1, -35.1, 149.1)); // line 4
        log.extend(cmd(3, 2, -35.2, 149.2)); // line 5
        log.extend(gps2(1_000, 2, -35.0, 149.0)); // line 6: no fix
        log.extend(gps2(2_000, 3, -35.05, 149.05)); // line 7
        log.extend(cam(-35.06, 149.06)); // line 8
        log.extend(cam(0.0, 149.06)); // line 9: no place
        log.extend(cmd(3, 1, -35.1, 149.1)); // line 10: the mission again
        log.extend(cmd(3, 2, -35.25, 149.25)); // line 11
        log
    }

    /// An older log's `GPS2` is the green route; a waypoint number joins the mission's route once
    /// and every later record of it is a marker; a camera is a photo marker.
    #[test]
    fn gps2_the_mission_and_the_camera_as_drawmap_sorts_them() {
        let routes = routes(&older());
        let green: Vec<usize> = routes.gps2.iter().map(|point| point.line).collect();
        assert_eq!(green, vec![7]);
        assert!(routes.gps.is_empty() && routes.gpsb.is_empty() && routes.pos.is_empty());
        let mission: Vec<(u16, usize)> = routes
            .commands
            .iter()
            .map(|command| (command.seq, command.line))
            .collect();
        assert_eq!(mission, vec![(1, 4), (2, 5)]);
        assert_eq!(routes.commands[0].total, Some(3));
        let repeats: Vec<(u16, usize)> = routes
            .repeats
            .iter()
            .map(|command| (command.seq, command.line))
            .collect();
        assert_eq!(repeats, vec![(1, 10), (2, 11)]);
        let photos: Vec<usize> = routes.cameras.iter().map(|point| point.line).collect();
        assert_eq!(photos, vec![8]);
        assert_eq!(routes.route(RouteKind::Cmd).len(), 2);
        assert_eq!(
            routes.places().count(),
            3,
            "the green point and two waypoints"
        );
    }

    /// `DrawMap(startline, endline)` sees only the records in its range: a waypoint first
    /// logged before the range joins the route from its first repeat inside it.
    #[test]
    fn a_range_keeps_what_its_records_put_on_the_map() {
        let routes = routes(&older()).between(6, 11);
        assert_eq!(routes.gps2.len(), 1);
        assert_eq!(routes.cameras.len(), 1);
        let mission: Vec<(u16, usize)> = routes
            .commands
            .iter()
            .map(|command| (command.seq, command.line))
            .collect();
        assert_eq!(mission, vec![(1, 10), (2, 11)]);
        assert!(routes.repeats.is_empty());
        let none = super::routes(&older()).between(0, 2);
        assert_eq!(none.places().count(), 0);
    }

    /// The pens, `Color.FromArgb(127, ...)` of the named colours.
    #[test]
    fn each_route_has_its_pen() {
        let colours: Vec<u32> = RouteKind::ALL.iter().map(|kind| kind.colour()).collect();
        assert_eq!(
            colours,
            vec![0x00_00_ff, 0x00_80_00, 0xff_ff_00, 0xff_00_00, 0x4b_00_82]
        );
        assert_eq!(RouteKind::ALPHA, 127);
    }

    /// A position off the planet is not a point.
    #[test]
    fn a_position_off_the_planet_is_refused() {
        let message = |lat: f64, lng: f64| LogMessage {
            name: "POS".to_owned(),
            fields: vec![
                ("Lat".to_owned(), Value::Float(lat)),
                ("Lng".to_owned(), Value::Float(lng)),
            ],
        };
        assert!(position(&message(-35.0, 149.0), 0, None).is_some());
        assert!(position(&message(-95.0, 149.0), 0, None).is_none());
        assert!(position(&message(-35.0, 190.0), 0, None).is_none());
        assert!(position(&message(f64::NAN, 149.0), 0, None).is_none());
    }
}
