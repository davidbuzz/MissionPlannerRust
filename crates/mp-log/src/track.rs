//! Where a log says the vehicle went: the routes Mission Planner's log browser puts on its map.
//!
//! `DrawMap` walks the log's position messages and builds one route per source, each its own
//! colour: the first GPS in blue, the second in green, a `GPSB` blend in yellow, the EKF's `POS`
//! in red, and the mission the vehicle logged (`CMD`) in indigo with a marker per waypoint. This
//! is the reading half - which records count as a point of which route - with no map in it.
//! `// C#: Log/LogBrowse.cs:2191-2540, 2541-2700`

use crate::dataflash::{DataflashReader, LogMessage, Value};

/// One point of a route.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoutePoint {
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
    /// Its place in the mission: `CNum`.
    pub seq: u16,
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

/// Every route a log holds, as `DrawMap` sorts its records into them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Routes {
    /// The first GPS: `GPS` records of instance 0, or with no instance, that have a 3D fix.
    pub gps: Vec<RoutePoint>,
    /// The EKF's position estimate: `POS` records.
    pub pos: Vec<RoutePoint>,
    /// The logged mission, one entry per waypoint number, in the order first logged.
    pub commands: Vec<LoggedCommand>,
}

/// Sorts a log's position records into routes.
///
/// The rules are `DrawMap`'s and `getPointLatLng`'s:
///
/// - a `GPS` point needs `Lat`, `Lng` and a `Status` of 3 or better - a GPS without a 3D fix
///   reports a position of nothing in particular - and belongs to the first GPS only if its
///   instance is 0 or it has none; the second GPS is a route of its own, not a jump in this one;
/// - a `POS` point needs a latitude and longitude that are on the planet;
/// - a `CMD` needs a position that is not 0,0 - which is how a command with no position logs -
///   and is taken once per waypoint number, since a mission re-sent to the vehicle logs again.
///
/// `// C#: Log/LogBrowse.cs:2234-2262, 2342-2433, 2541-2600, 2676-2720`
#[must_use]
pub fn routes(data: &[u8]) -> Routes {
    let gps_instance = crate::plot::instance_fields(data).get("GPS").cloned();
    let mut routes = Routes::default();
    let mut reader = DataflashReader::new(data);
    while let Some(message) = reader.next_message() {
        match message.name.as_str() {
            "GPS" => {
                if let Some(label) = gps_instance.as_deref()
                    && message.field(label).and_then(Value::as_f64) != Some(0.0)
                {
                    continue;
                }
                if let Some(point) = gps_point(&message) {
                    routes.gps.push(point);
                }
            }
            "POS" => {
                if let Some(point) = position(&message, None) {
                    routes.pos.push(point);
                }
            }
            "CMD" => {
                if let Some(command) = command(&message)
                    && !routes.commands.iter().any(|seen| seen.seq == command.seq)
                {
                    routes.commands.push(command);
                }
            }
            _ => {}
        }
    }
    routes
}

/// A point of the first GPS's route, if the record has a fix.
fn gps_point(message: &LogMessage) -> Option<RoutePoint> {
    let status = message.field("Status").and_then(Value::as_f64)?;
    if status < 3.0 {
        return None;
    }
    position(message, message.field("GCrs").and_then(Value::as_f64))
}

/// A record's `Lat` and `Lng` as a point, if they are a place on the planet.
fn position(message: &LogMessage, course: Option<f64>) -> Option<RoutePoint> {
    let latitude = message.field("Lat").and_then(Value::as_f64)?;
    let longitude = message.field("Lng").and_then(Value::as_f64)?;
    if !(latitude.abs() <= 90.0 && longitude.abs() <= 180.0) {
        return None;
    }
    Some(RoutePoint {
        time_us: message.field("TimeUS").and_then(Value::as_f64),
        latitude,
        longitude,
        course,
    })
}

/// A logged mission item, if it has a position.
fn command(message: &LogMessage) -> Option<LoggedCommand> {
    let at = position(message, None)?;
    if at.latitude == 0.0 || at.longitude == 0.0 {
        return None;
    }
    let number = |label: &str| message.field(label).and_then(Value::as_f64);
    let whole = |label: &str| number(label).and_then(small_whole);
    Some(LoggedCommand {
        seq: whole("CNum")?,
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
        std::fs::read(&path).unwrap_or_else(|err| panic!("reading {}: {err}", path.display()))
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

    /// The damaged fixture came off a vehicle with a fix, and its route is a real one.
    ///
    /// It holds two boots of the same vehicle, one after the other: `TimeUS` climbs through the
    /// first, goes back to the start of the second, and climbs again. The route keeps the log's
    /// order, as `DrawMap` does, so time runs forward everywhere but that one seam.
    #[test]
    fn a_gps_with_a_fix_makes_a_route_in_log_order() {
        let routes = routes(&testdata("dataflash_damaged.bin"));
        assert_eq!(routes.gps.len(), 63);
        assert_eq!(routes.pos.len(), 119);
        for point in routes.gps.iter().chain(&routes.pos) {
            assert!(
                (point.latitude + 27.5134).abs() < 0.01
                    && (point.longitude - 153.0094).abs() < 0.01,
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
        assert!(position(&message(-35.0, 149.0), None).is_some());
        assert!(position(&message(-95.0, 149.0), None).is_none());
        assert!(position(&message(-35.0, 190.0), None).is_none());
        assert!(position(&message(f64::NAN, 149.0), None).is_none());
    }
}
