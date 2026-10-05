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

//! `MagCalib.ProcessLog`'s pass over a log, its fit, and `doDXF`'s drawing of what it fitted - the
//! planner's EXPERIMENTAL "mag calb log" and `headless-planner magcal` both read logs through here.
//!
//! A file whose name ends in `tlog` is read as `getOffsets` reads it: every `RAW_IMU` taken while
//! the throttle is at least the threshold, with the last `SENSOR_OFFSETS` taken back off. Anything
//! else is a dataflash log read as `getOffsetsLog` reads it, through `DFLogBuffer`: every `MAG`,
//! `MAG2` and `MAG3` line. The fit is `mp_calibration::magcalib`'s; this is here, with the logs,
//! because reading them is this crate's (mp-calibration sits a layer below it).
//! `// C#: MagCalib.cs:93-133, 813-1137`

use mp_calibration::CalibrationError;
use mp_calibration::magcalib::{
    DataflashSamples, LogFit, Sample, TlogSamples, fit_dataflash, fit_tlog,
};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};

use crate::TlogReader;
use crate::convert::flight_mode_name;
use crate::dflogbuffer::DfLogBuffer;

/// What a pass over a log gathered.
#[derive(Debug, Clone)]
pub enum Gathered {
    /// `getOffsets`' pass over a telemetry log.
    Tlog(TlogSamples),
    /// `getOffsetsLog`'s pass over a dataflash log.
    Dataflash(DataflashSamples),
}

/// `ProcessLog`'s choice: a name ending in `tlog`, whatever its case, is a telemetry log.
/// `// C#: MagCalib.cs:115`
#[must_use]
pub fn is_tlog(path: &str) -> bool {
    path.to_lowercase().ends_with("tlog")
}

/// Reads a log's samples the way the C# path for its name does.
///
/// # Errors
///
/// A dataflash line the C# would throw on - a `MagY` or `OfsY` column missing beside a `MagX` and
/// `OfsX`, or a value `float.Parse` refuses - which `ProcessLog` catches and shows nothing for.
pub fn gather(path: &str, data: &[u8], throttle: i32) -> Result<Gathered, String> {
    if is_tlog(path) {
        Ok(Gathered::Tlog(gather_tlog(data, throttle)))
    } else {
        gather_dataflash(data).map(Gathered::Dataflash)
    }
}

/// `getOffsets`' read loop: every frame the log holds, decoded, into [`TlogSamples`].
/// `// C#: MagCalib.cs:988-1057`
fn gather_tlog(data: &[u8], throttle: i32) -> TlogSamples {
    let mut samples = TlogSamples::new(throttle);
    let mut reader = TlogReader::new(data);
    while let Some(record) = reader.next_record(&DIALECT) {
        let Ok((frame, _)) = mp_mavlink::parse(record.frame, &DIALECT) else {
            continue;
        };
        // `DebugPacket` returning null: a message this build does not know.
        let Some(message) = MavMessage::decode(frame.msgid, frame.payload) else {
            continue;
        };
        samples.message(&message);
    }
    samples
}

/// `getOffsetsLog`'s read loop: `GetEnumeratorType` over `MAG`, `MAG2` and `MAG3`, each line with
/// a `MagX` and an `OfsX` column parsed into [`DataflashSamples`]. `// C#: MagCalib.cs:829-899`
fn gather_dataflash(data: &[u8]) -> Result<DataflashSamples, String> {
    let mut buffer = DfLogBuffer::new(data, &flight_mode_name);
    let mut samples = DataflashSamples::new();
    for (number, line) in buffer.items_of(&["MAG", "MAG2", "MAG3"]) {
        let msgtype = line.msgtype().to_owned();
        let mut column = |name: &str| buffer.dflog.find_message_offset(&msgtype, name);
        let (Some(magx), Some(ofsx)) = (column("MagX"), column("OfsX")) else {
            continue;
        };
        let rest = [
            column("MagY"),
            column("MagZ"),
            column("OfsY"),
            column("OfsZ"),
        ];
        let [Some(magy), Some(magz), Some(ofsy), Some(ofsz)] = rest else {
            return Err(format!(
                "line {number}: {msgtype} has MagX and OfsX but not the rest"
            ));
        };
        let value = |index: usize| -> Result<f32, String> {
            let text = line.items.get(index).and_then(Option::as_deref);
            text.and_then(|text| text.trim().parse::<f32>().ok())
                .ok_or_else(|| format!("line {number}: {msgtype} value {text:?} is not a number"))
        };
        let mag = [value(magx)?, value(magy)?, value(magz)?];
        let ofs = [value(ofsx)?, value(ofsy)?, value(ofsz)?];
        samples.line(&msgtype, mag, ofs);
    }
    Ok(samples)
}

/// Compass 1's samples, which are what `ProcessLog` fits and shows.
#[must_use]
pub fn compass_one(gathered: &Gathered) -> &[Sample] {
    match gathered {
        Gathered::Tlog(samples) => &samples.data,
        Gathered::Dataflash(samples) => &samples.data[0],
    }
}

/// `getOffsets` or `getOffsetsLog` after the pass, as the log's path chose: compass 1's fit.
///
/// # Errors
///
/// The fit's: too few telemetry samples, where the C# shows
/// [`NOT_ENOUGH_DATA`](mp_calibration::magcalib::NOT_ENOUGH_DATA) and throws; a dataflash log with no
/// `MAG` line, where alglib throws and `ProcessLog` swallows it.
/// `// C#: MagCalib.cs:901-932, 1070-1117`
pub fn fit(gathered: &Gathered) -> Result<LogFit, CalibrationError> {
    match gathered {
        Gathered::Tlog(samples) => fit_tlog(samples.data.clone()),
        Gathered::Dataflash(_) => fit_dataflash(compass_one(gathered)),
    }
}

/// The points `doDXF` draws: every sample of compass 1 taken - on the telemetry path before its
/// filter, on the dataflash path every `MAG` line's. `// C#: MagCalib.cs:868-873, 1021-1027`
#[must_use]
pub fn drawn(gathered: &Gathered) -> &[Sample] {
    match gathered {
        Gathered::Tlog(samples) => &samples.vertexes,
        Gathered::Dataflash(samples) => &samples.data[0],
    }
}

/// `doDXF`'s file, in `Settings.GetUserDataDirectory()`. `// C#: MagCalib.cs:1134`
pub const DXF_NAME: &str = "magoffset.dxf";

/// `doDXF`: "a dxf for those who want to 'see' the calibration" - the samples as one closed 3D
/// polyline on the layer "polyline" (colour 24), and the new offsets' centre, minus each offset as
/// a float, as a point on the layer "new offset" (colour 21). The C#'s netDxf writes AutoCAD 2000's
/// version by default, as this does.
///
/// # Errors
///
/// The writer's, as text: netDxf's `Save` throwing, which `ProcessLog`'s `catch` swallows - and
/// with it `SaveOffsets`, which comes after.
/// `// C#: MagCalib.cs:1119-1137`
pub fn dxf(points: &[Sample], offsets: [f64; 3]) -> Result<Vec<u8>, String> {
    use dxf::entities::{Entity, EntityType, ModelPoint, Polyline, Vertex};
    use dxf::enums::AcadVersion;
    use dxf::tables::Layer;
    use dxf::{Color, Drawing, Point};

    const POLYLINE: &str = "polyline";
    const NEW_OFFSET: &str = "new offset";

    let mut drawing = Drawing::new();
    drawing.header.version = AcadVersion::R2000;
    for (name, colour) in [(POLYLINE, 24), (NEW_OFFSET, 21)] {
        drawing.add_layer(Layer {
            name: name.to_owned(),
            color: Color::from_index(colour),
            ..Layer::default()
        });
    }
    // `new Polyline(vertexes, true)`: closed, and netDxf's Polyline is a 3D one.
    let mut polyline = Polyline::default();
    polyline.set_is_closed(true);
    polyline.set_is_3d_polyline(true);
    for [x, y, z] in points {
        let vertex = Vertex {
            location: Point::new(f64::from(*x), f64::from(*y), f64::from(*z)),
            ..Vertex::default()
        };
        polyline.add_vertex(&mut drawing, vertex);
    }
    let mut line = Entity::new(EntityType::Polyline(polyline));
    line.common.layer = POLYLINE.to_owned();
    drawing.add_entity(line);
    // `new Point(new Vector3(-(float)x[0], -(float)x[1], -(float)x[2]))`.
    #[allow(clippy::cast_possible_truncation)] // `(float)x[i]`
    let [x, y, z] = offsets.map(|value| f64::from(-(value as f32)));
    let mut point = Entity::new(EntityType::ModelPoint(ModelPoint {
        location: Point::new(x, y, z),
        ..ModelPoint::default()
    }));
    point.common.layer = NEW_OFFSET.to_owned();
    drawing.add_entity(point);
    let mut out = Vec::new();
    drawing.save(&mut out).map_err(|error| error.to_string())?;
    Ok(out)
}

/// `getOffsets`' box for a telemetry log it cannot open. `// C#: MagCalib.cs:981`
pub const CANNOT_OPEN: &str = "Log Can not be opened. Are you still connected?";

/// What `ProcessLog` comes to.
#[derive(Debug, Clone, PartialEq)]
pub enum Processed {
    /// The offsets it hands `SaveOffsets`, once the drawing is written.
    Offsets {
        /// `ans`.
        offsets: [f64; 3],
        /// How many bytes the drawing came to.
        drawing: usize,
    },
    /// A box the C# shows, and nothing after it.
    Said(&'static str),
    /// Nothing shown: the `catch`'s `log.Debug`, with what it would have logged.
    Quiet(String),
}

/// `ProcessLog(throttle)` once its dialog has chosen `path`: the log read and fitted as its name
/// says, the drawing written into `data_directory` as [`DXF_NAME`], and the offsets for
/// `SaveOffsets`. Anything that throws on the way ends it quietly, the drawing's write among them.
/// The C#'s data directory always exists by then - `MainV2` creates the log directory inside it -
/// so it is made here if it is not there.
/// `// C#: MagCalib.cs:109-131, 940-1117`
#[must_use]
pub fn process_log(
    path: &std::path::Path,
    throttle: i32,
    data_directory: Option<&std::path::Path>,
) -> Processed {
    let name = path.to_string_lossy();
    let data = match mp_os::fs::read(path) {
        Ok(data) => data,
        // `getOffsets` says so; `getOffsetsLog`'s `File.OpenRead` throws to the `catch`.
        Err(_) if is_tlog(&name) => return Processed::Said(CANNOT_OPEN),
        Err(error) => return Processed::Quiet(format!("{name}: {error}")),
    };
    let gathered = match gather(&name, &data, throttle) {
        Ok(gathered) => gathered,
        Err(error) => return Processed::Quiet(format!("{name}: {error}")),
    };
    let fitted = match fit(&gathered) {
        Ok(fitted) => fitted,
        Err(CalibrationError::NotEnoughData { .. }) => {
            return Processed::Said(mp_calibration::magcalib::NOT_ENOUGH_DATA);
        }
        Err(error) => return Processed::Quiet(error.to_string()),
    };
    let Some(directory) = data_directory else {
        return Processed::Quiet("no data directory for the drawing".to_owned());
    };
    let written = dxf(drawn(&gathered), fitted.offsets).and_then(|bytes| {
        let drawing = bytes.len();
        mp_os::fs::create_dir_all(directory)
            .and_then(|()| mp_os::fs::write(directory.join(DXF_NAME), bytes))
            .map(|()| drawing)
            .map_err(|error| error.to_string())
    });
    match written {
        Ok(drawing) => Processed::Offsets {
            offsets: fitted.offsets,
            drawing,
        },
        Err(error) => Processed::Quiet(format!("{DXF_NAME}: {error}")),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    /// The drawing reads back as `doDXF` made it: its two layers with their colours, the samples
    /// as a closed 3D polyline on the first, and the offsets' centre on the second.
    #[test]
    fn the_drawing_is_the_samples_and_the_centre() {
        let points = [[1.0, 2.0, 3.0], [-4.5, 5.0, 6.25], [7.0, -8.0, 9.0]];
        let bytes = dxf(&points, [10.0, -20.5, 30.25]).unwrap();
        let drawing = dxf::Drawing::load(&mut bytes.as_slice()).unwrap();
        assert_eq!(drawing.header.version, dxf::enums::AcadVersion::R2000);
        let layers: Vec<(String, Option<u8>)> = drawing
            .layers()
            .map(|layer| (layer.name.clone(), layer.color.index()))
            .collect();
        assert!(
            layers.contains(&("polyline".to_owned(), Some(24))),
            "{layers:?}"
        );
        assert!(
            layers.contains(&("new offset".to_owned(), Some(21))),
            "{layers:?}"
        );
        let entities: Vec<&dxf::entities::Entity> = drawing.entities().collect();
        assert_eq!(entities.len(), 2);
        let dxf::entities::EntityType::Polyline(polyline) = &entities[0].specific else {
            panic!("{:?}", entities[0].specific);
        };
        assert_eq!(entities[0].common.layer, "polyline");
        assert!(polyline.is_closed());
        assert!(polyline.is_3d_polyline());
        let read: Vec<[f64; 3]> = polyline
            .vertices()
            .map(|vertex| [vertex.location.x, vertex.location.y, vertex.location.z])
            .collect();
        assert_eq!(read, [[1.0, 2.0, 3.0], [-4.5, 5.0, 6.25], [7.0, -8.0, 9.0]]);
        let dxf::entities::EntityType::ModelPoint(point) = &entities[1].specific else {
            panic!("{:?}", entities[1].specific);
        };
        assert_eq!(entities[1].common.layer, "new offset");
        assert_eq!(
            [point.location.x, point.location.y, point.location.z],
            [-10.0, 20.5, -30.25]
        );
    }

    fn testdata(name: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    /// A directory of this test's own, empty.
    fn scratch(name: &str) -> std::path::PathBuf {
        let directory = mp_os::temp_dir().join(format!("mp-magcal-{}-{name}", std::process::id()));
        let _ = mp_os::fs::remove_dir_all(&directory);
        directory
    }

    /// A dataflash log: the fit's offsets for `SaveOffsets`, and the drawing of every `MAG` line
    /// written into the data directory, made if it was not there.
    #[test]
    fn a_dataflash_log_gives_offsets_and_its_drawing() {
        let directory = scratch("dataflash").join("Mission Planner");
        let log = testdata("dataflash.bin");
        let processed = process_log(&log, 0, Some(&directory));
        let data = mp_os::fs::read(&log).unwrap();
        let gathered = gather("dataflash.bin", &data, 0).unwrap();
        let expected = fit(&gathered).unwrap().offsets;
        let written = mp_os::fs::read(directory.join(DXF_NAME)).unwrap();
        assert_eq!(
            processed,
            Processed::Offsets {
                offsets: expected,
                drawing: written.len()
            }
        );
        let drawing = dxf::Drawing::load(&mut written.as_slice()).unwrap();
        let dxf::entities::EntityType::Polyline(polyline) =
            &drawing.entities().next().unwrap().specific
        else {
            panic!("no polyline");
        };
        assert_eq!(polyline.vertices().count(), drawn(&gathered).len());
        assert!(!drawn(&gathered).is_empty());
        let _ = mp_os::fs::remove_dir_all(directory.parent().unwrap());
    }

    /// A telemetry log with too few samples says so, as the C#'s box does, and draws nothing.
    #[test]
    fn a_still_telemetry_log_is_not_enough_data() {
        let directory = scratch("still");
        assert_eq!(
            process_log(&testdata("mavlink/autotest.tlog"), 0, Some(&directory)),
            Processed::Said(mp_calibration::magcalib::NOT_ENOUGH_DATA)
        );
        assert!(!directory.join(DXF_NAME).exists());
    }

    /// A telemetry log that cannot be read says the C#'s words; a dataflash one says nothing.
    #[test]
    fn a_missing_log_is_said_only_on_the_telemetry_path() {
        let directory = scratch("missing");
        assert_eq!(
            process_log(&directory.join("gone.tlog"), 0, Some(&directory)),
            Processed::Said(CANNOT_OPEN)
        );
        assert!(matches!(
            process_log(&directory.join("gone.bin"), 0, Some(&directory)),
            Processed::Quiet(_)
        ));
    }

    /// With nowhere to draw, `doDXF` throws, and `SaveOffsets` after it is never reached.
    #[test]
    fn no_drawing_no_offsets() {
        assert!(matches!(
            process_log(&testdata("dataflash.bin"), 0, None),
            Processed::Quiet(_)
        ));
    }

    /// A name ending in `tlog`, whatever its case, is a telemetry log; anything else dataflash.
    #[test]
    fn the_name_decides_the_path() {
        assert!(is_tlog("flight.TLOG"));
        assert!(is_tlog("oddtlog"));
        assert!(!is_tlog("00000001.BIN"));
        assert!(!is_tlog("flight.log"));
    }

    /// The telemetry path draws every sample taken, not only those the filter keeps.
    #[test]
    fn the_telemetry_path_draws_what_the_filter_drops() {
        let mut samples = TlogSamples::new(0);
        let imu = MavMessage::RawImu(mp_mavlink_dialects::all::RawImu {
            time_usec: 0,
            xacc: 0,
            yacc: 0,
            zacc: 0,
            xgyro: 0,
            ygyro: 0,
            zgyro: 0,
            xmag: 100,
            ymag: 100,
            zmag: -300,
            id: 0,
            temperature: 0,
        });
        // The same sample five times: the filter keeps three.
        for _ in 0..5 {
            samples.message(&imu);
        }
        let gathered = Gathered::Tlog(samples);
        assert_eq!(compass_one(&gathered).len(), 3);
        assert_eq!(drawn(&gathered).len(), 5);
    }
}
