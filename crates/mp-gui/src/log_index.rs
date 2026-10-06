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

//! `LogIndex`, the temp form's logindex ("tlog browser"): every `.tlog`, `.bin` and `.log` under a
//! folder, each with its LogMap picture (log_map.rs) and what the C# reads out of it - a row each.
//!
//! What a row holds is `processbg`'s: a telemetry log of 4 KiB or more replayed as
//! `MAVLinkInterface` replays one - the first vehicle whose heartbeat arrives is the one read, as
//! the C#'s reader attaches to the first device it sees; `getHeartBeat`'s time is the date, the
//! last packet's less it the duration; the vehicle's state brought up to date every 200 packets and
//! whenever its GPS clock moves a second, as `UpdateCurrentSettings` counts its time in air and
//! distance; its position at the end is "Home"; the camera's `CAMERA_FEEDBACK`s counted. A
//! dataflash log's GPS lines with a 3D fix give the date (the first), the duration, home (the
//! first fix), the distance (once a second) and the time in air (a second while faster than 0.2
//! m/s or 2 m over home); its `CAM` lines are counted, and its aircraft and frame are what the C#
//! writes, 0 and "DFLog Unknown". A telemetry log under 4 KiB, or one the C# throws on, has no row.
//! `// C#: Log/LogIndex.cs:1-447`

#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};

use gpui::{
    AnyElement, Context, InteractiveElement, IntoElement, ParentElement, SharedString,
    StatefulInteractiveElement, Styled, Window, div, px, rgb,
};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_os::fs::FsExt as _;
use mp_vehicle::clock::DateTime;

use crate::MissionPlanner;
use crate::ui::theme;

/// A telemetry log smaller than this is skipped: "file is to small". `// C#: LogIndex.cs:150-152`
const SMALLEST_TLOG: u64 = 1024 * 4;
/// "abandon last 100 bytes". `// C#: LogIndex.cs:179-180`
const ABANDONED: usize = 100;
/// `getHeartBeat` gives up after this many packets. `// C#: MAVLinkInterface.cs:1201`
const HEARTBEAT_READS: usize = 200;
/// The state is brought up to date this often, in packets. `// C#: LogIndex.cs:191`
const UPDATE_EVERY: usize = 200;
const HEARTBEAT: u32 = 0;
const HIGH_LATENCY2: u32 = 235;
const CAMERA_FEEDBACK: u32 = 180;
/// `MAV_TYPE.GCS`, which `getHeartBeat` passes over.
const GCS: u8 = 6;
/// The ground station's own packets, which the replay loop does not count.
const GCS_SYSID: u8 = 255;
/// What the C# writes for a dataflash log's frame. `// C#: LogIndex.cs:275`
const DFLOG_FRAME: &str = "DFLog Unknown";
/// `DateTime` ticks at the Unix epoch, and ticks in a millisecond.
const UNIX_EPOCH_TICKS: i64 = 621_355_968_000_000_000;
const TICKS_PER_MS: i64 = 10_000;

/// `loginfo`: one row. `// C#: LogIndex.cs:286-320`
#[derive(Debug, Clone, PartialEq)]
pub struct LogInfo {
    /// `fullname`.
    pub fullname: PathBuf,
    /// `imgfile`: the LogMap picture, when there is one.
    pub picture: Option<PathBuf>,
    /// `Duration`: a `TimeSpan`'s text, or empty.
    pub duration: String,
    /// `Date`, as `DateTime` ticks - 0 for `DateTime.MinValue`.
    pub date_ticks: i64,
    /// `Aircraft`.
    pub aircraft: i32,
    /// `Size`, bytes.
    pub size: u64,
    /// `Home`: latitude, longitude and altitude, or none.
    pub home: Option<(f64, f64, f64)>,
    /// `Frame`.
    pub frame: String,
    /// `TimeInAir`, seconds.
    pub time_in_air: f32,
    /// `DistTraveled`, metres.
    pub dist_traveled: f32,
    /// `CamMSG`.
    pub cam_msg: i32,
}

impl LogInfo {
    fn new(fullname: &Path) -> Self {
        Self {
            fullname: fullname.to_path_buf(),
            picture: None,
            duration: String::new(),
            date_ticks: 0,
            aircraft: 0,
            size: 0,
            home: None,
            frame: String::new(),
            time_in_air: 0.0,
            dist_traveled: 0.0,
            cam_msg: 0,
        }
    }

    /// `Name`: the file's name.
    #[must_use]
    pub fn name(&self) -> String {
        self.fullname
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_default()
    }

    /// `Directory`: the folder it is in.
    #[must_use]
    pub fn directory(&self) -> String {
        self.fullname
            .parent()
            .map(|folder| folder.display().to_string())
            .unwrap_or_default()
    }

    /// Each column's text, as the list writes its values (`ToString()`, English): Date, Directory,
    /// Frame, Aircraft, Duration, FileName, Size, Home, TimeInAir, DistTraveled, CamMSG - the
    /// picture column has none. `// C#: Log/LogIndex.Designer.cs:68-80`
    #[must_use]
    pub fn cells(&self) -> [String; 11] {
        [
            date_text(self.date_ticks),
            self.directory(),
            self.frame.clone(),
            self.aircraft.to_string(),
            self.duration.clone(),
            self.name(),
            self.size.to_string(),
            self.home.map_or_else(String::new, |(lat, lng, alt)| {
                // `PointLatLngAlt.ToString()`: `Lat + "," + Lng + "," + Alt + "," + Tag`.
                format!(
                    "{},{},{},",
                    double_text(lat),
                    double_text(lng),
                    double_text(alt)
                )
            }),
            single_text(self.time_in_air),
            single_text(self.dist_traveled),
            self.cam_msg.to_string(),
        ]
    }
}

/// A double as .NET writes it.
fn double_text(value: f64) -> String {
    mp_params::param_file::invariant_double(value)
}

/// A float as .NET Framework writes it: seven significant digits, the `G` rules.
#[must_use]
pub fn single_text(value: f32) -> String {
    let value = f64::from(value);
    if value == 0.0 || !value.is_finite() {
        return double_text(value);
    }
    let rounded: f64 = format!("{value:.6e}").parse().unwrap_or(value);
    #[allow(clippy::cast_possible_truncation)] // a float's exponent
    let exponent = rounded.abs().log10().floor() as i32;
    if (-5..7).contains(&exponent) {
        double_text(rounded)
    } else {
        // `1.234568E+07`: the mantissa's trailing zeros dropped, the exponent signed and two digits.
        let text = format!("{rounded:.6e}");
        let (mantissa, power) = text.split_once('e').unwrap_or((&text, "0"));
        let mantissa = mantissa.trim_end_matches('0').trim_end_matches('.');
        let power: i32 = power.parse().unwrap_or(0);
        let sign = if power < 0 { '-' } else { '+' };
        format!("{mantissa}E{sign}{:02}", power.abs())
    }
}

/// A UTC time in `DateTime` ticks as the C# shows it - local, as its readers' `ToLocalTime()` make
/// it, English, `M/d/yyyy h:mm:ss tt`; `DateTime.MinValue` as it is.
#[must_use]
pub fn date_text(ticks: i64) -> String {
    const FORMAT: &str = "%-m/%-d/%Y %-I:%M:%S %p";
    if ticks == 0 {
        return "1/1/0001 12:00:00 AM".to_owned();
    }
    let unix_ms = (ticks - UNIX_EPOCH_TICKS) / TICKS_PER_MS;
    chrono::DateTime::from_timestamp_millis(unix_ms).map_or_else(String::new, |utc| {
        utc.with_timezone(&chrono::Local).format(FORMAT).to_string()
    })
}

/// `TimeSpan.ToString()`: `[-][d.]hh:mm:ss[.fffffff]`.
#[must_use]
pub fn duration_text(ticks: i64) -> String {
    let sign = if ticks < 0 { "-" } else { "" };
    let ticks = ticks.unsigned_abs();
    let fraction = ticks % 10_000_000;
    let seconds = ticks / 10_000_000;
    let (days, hours, minutes, seconds) = (
        seconds / 86_400,
        seconds / 3600 % 24,
        seconds / 60 % 60,
        seconds % 60,
    );
    let mut text = String::from(sign);
    if days > 0 {
        text.push_str(&format!("{days}."));
    }
    text.push_str(&format!("{hours:02}:{minutes:02}:{seconds:02}"));
    if fraction > 0 {
        text.push_str(&format!(".{fraction:07}"));
    }
    text
}

/// `ToHours`: seconds as `hh:mm:ss`, the hours two digits or more. `// C#: LogIndex.cs:431-438`
#[must_use]
pub fn to_hours(seconds: f32) -> String {
    #[allow(clippy::cast_possible_truncation)] // `(int)Math.Floor(...)`, `(int)(seconds % 60)`
    let (hours, minutes, sec) = (
        (seconds / 3600.0).floor() as i32,
        ((seconds % 3600.0) / 60.0).floor() as i32,
        (seconds % 60.0) as i32,
    );
    format!(
        "{}{hours}:{minutes:02}:{sec:02}",
        if hours < 10 { "0" } else { "" }
    )
}

/// `lbStats`: the rows chosen, their time in air and their distance, whole metres.
/// `// C#: LogIndex.cs:412-429`
#[must_use]
pub fn stats_text(chosen: &[&LogInfo]) -> String {
    let time_in_air: f32 = chosen.iter().map(|info| info.time_in_air).sum();
    let dist_traveled: f32 = chosen.iter().map(|info| info.dist_traveled).sum();
    #[allow(clippy::cast_possible_truncation)] // `(int)distTraveled`
    let metres = dist_traveled as i32;
    format!(
        "Selected: {}; TimeInAir: {}; DistTraveled: {metres}m",
        chosen.len(),
        to_hours(time_in_air)
    )
}

/// `createFileList`: every `.tlog`, then `.bin`, then `.log`, in and under the folder.
/// `// C#: LogIndex.cs:58-74`
#[must_use]
pub fn files(folder: &Path) -> Vec<PathBuf> {
    ["tlog", "bin", "log"]
        .into_iter()
        .flat_map(|extension| crate::log_map::files_under(folder, extension))
        .collect()
}

/// `processbg`: the log's picture made if it has none, then its row - none for a telemetry log too
/// small to read or one the C# throws on. `// C#: LogIndex.cs:102-282`
pub fn process(
    file: &Path,
    tile: &mut dyn FnMut(mp_units::tiles::TileId) -> Option<image::RgbaImage>,
) -> Option<LogInfo> {
    if !file.os_exists() {
        return None;
    }
    let picture = crate::log_map::picture_of(file);
    if !picture.os_exists() {
        crate::log_map::process_file(file, tile);
    }
    let mut info = LogInfo::new(file);
    info.size = mp_os::fs::metadata(file).map_or(0, |metadata| metadata.len());
    if picture.os_exists() {
        info.picture = Some(picture);
    }
    let name = file.to_string_lossy().to_lowercase();
    if name.ends_with(".tlog") {
        let data = mp_os::fs::read(file).ok()?;
        read_tlog(&data, &mut info)?;
    } else if name.ends_with(".bin") || name.ends_with(".log") {
        let data = mp_os::fs::read(file).ok()?;
        read_dataflash(&data, &mut info).ok()?;
    }
    Some(info)
}

/// One record of a telemetry log.
struct Packet {
    time: Option<DateTime>,
    sysid: u8,
    compid: u8,
    seq: u8,
    msgid: u32,
    message: Option<MavMessage>,
    /// Where the record ends in the file.
    end: usize,
}

/// Every record of a telemetry log, in order, each with the time the C#'s reader gives it: the
/// eight bytes after the frame before it, read as a stamp unless they start a frame themselves -
/// so a frame found past lines of text, as MAVProxy's logs open, has the stamp the text had.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6488-6521`
fn packets(data: &[u8]) -> Vec<Packet> {
    let mut reader = mp_log::TlogReader::new(data);
    let mut out = Vec::new();
    let mut previous_end = 0;
    while let Some(record) = reader.next_record(&DIALECT) {
        let Ok((frame, _)) = mp_mavlink::parse(record.frame, &DIALECT) else {
            continue;
        };
        let stamp = data
            .get(previous_end..previous_end + 8)
            .and_then(|bytes| <[u8; 8]>::try_from(bytes).ok())
            .filter(|bytes| bytes[0] != mp_mavlink::STX_V1 && bytes[0] != mp_mavlink::STX_V2)
            .map(u64::from_be_bytes);
        previous_end = record.offset + record.frame.len();
        out.push(Packet {
            time: stamp.and_then(DateTime::from_tlog_micros),
            sysid: frame.sysid,
            compid: frame.compid,
            seq: frame.seq,
            msgid: frame.msgid,
            message: MavMessage::decode(frame.msgid, frame.payload),
            end: record.offset + record.frame.len(),
        });
    }
    out
}

/// The replay `processbg` makes of a telemetry log; `None` for one too small to read.
/// `// C#: LogIndex.cs:131-213`
fn read_tlog(data: &[u8], info: &mut LogInfo) -> Option<()> {
    if (data.len() as u64) < SMALLEST_TLOG {
        return None;
    }
    let packets = packets(data);
    let mut registry = mp_vehicle::VehicleRegistry::new();
    // The reader's clock, `lastlogread`: the last packet's time.
    let mut last_read = DateTime::MIN;
    // `sysidcurrent`: the first vehicle a heartbeat comes from, any type.
    let mut current: Option<mp_vehicle::VehicleId> = None;
    let apply = |registry: &mut mp_vehicle::VehicleRegistry,
                 current: &mut Option<mp_vehicle::VehicleId>,
                 last_read: &mut DateTime,
                 packet: &Packet| {
        if let Some(time) = packet.time {
            *last_read = time;
        }
        let Some(message) = &packet.message else {
            return;
        };
        let id = registry.apply_at(packet.sysid, packet.compid, packet.seq, message, *last_read);
        if current.is_none() && (packet.msgid == HEARTBEAT || packet.msgid == HIGH_LATENCY2) {
            *current = Some(id);
        }
    };
    // `getHeartBeat`: packets until a heartbeat from anything but a ground station, two hundred at
    // most.
    for packet in packets.iter().take(HEARTBEAT_READS) {
        apply(&mut registry, &mut current, &mut last_read, packet);
        let not_gcs = match &packet.message {
            Some(MavMessage::Heartbeat(heartbeat)) => heartbeat.r#type != GCS,
            Some(MavMessage::HighLatency2(latency)) => latency.r#type != GCS,
            _ => false,
        };
        if not_gcs {
            break;
        }
    }
    let start = last_read;
    info.date_ticks = start.ticks();
    info.aircraft = current.map_or(0, |id| i32::from(id.sysid));
    info.frame = mp_log::log_sort::type_name(
        current
            .and_then(|id| registry.working(id))
            .map_or(0, |state| state.vehicle_type),
    );
    // The same log again from its start, but its last hundred bytes.
    let mut end = start;
    let mut count = 0usize;
    let mut gps_second = 0i64;
    let stop = data.len().saturating_sub(ABANDONED);
    for packet in &packets {
        if packet.end > stop {
            break;
        }
        apply(&mut registry, &mut current, &mut last_read, packet);
        if packet.sysid == GCS_SYSID {
            continue;
        }
        if packet.msgid == CAMERA_FEEDBACK {
            info.cam_msg += 1;
        }
        let second = current
            .and_then(|id| registry.working(id))
            .map_or(0, |state| {
                i64::try_from(state.gps_time_unix_ms / 1000 % 60).unwrap_or(0)
            });
        if count.is_multiple_of(UPDATE_EVERY) || second != gps_second {
            gps_second = second;
            if let Some(state) = current.and_then(|id| registry.working_mut(id)) {
                state.update_current_settings(false);
            }
        }
        count += 1;
        if last_read.ticks() > end.ticks() {
            end = last_read;
        }
    }
    if let Some(state) = current.and_then(|id| registry.working(id)) {
        // `cs.Location`: where it is at the end, `(lat, lng, altasl)`.
        let (lat, lng) = state
            .position
            .map_or((0.0, 0.0), |at| (at.latitude(), at.longitude()));
        // `altasl` is a float in the C#.
        #[allow(clippy::cast_possible_truncation)]
        let altitude = f64::from(state.altitude_msl.0 as f32);
        info.home = Some((lat, lng, altitude));
        info.time_in_air = state.time_in_air;
        info.dist_traveled = state.dist_traveled;
    }
    info.duration = duration_text(end.ticks() - start.ticks());
    Some(())
}

/// A dataflash log's GPS lines with a 3D fix, as `processbg` reads them.
///
/// # Errors
///
/// A GPS line with a fix and no `Lat`, `Lng`, `Alt` or `Spd`, or one that is no number, where
/// `double.Parse` throws and the C# lists nothing. `// C#: LogIndex.cs:215-277`
fn read_dataflash(data: &[u8], info: &mut LogInfo) -> Result<(), String> {
    let mut buffer =
        mp_log::dflogbuffer::DfLogBuffer::new(data, &mp_log::convert::flight_mode_name);
    let lines = buffer.items_of(&["GPS"]);
    let mut clock = GpsClock::default();
    let mut last: Option<(f64, f64, f64)> = None;
    let mut start: Option<i64> = None;
    let mut end = 0i64;
    let mut in_air_until = 0i64;
    for (line, item) in &lines {
        let text = buffer.line(*line);
        clock.see(&mut buffer.dflog, item, &text);
        let dflog = &mut buffer.dflog;
        let Some(status) = item.get(dflog, "Status") else {
            continue;
        };
        // `int.Parse(dfItem["Status"])`
        let status: i32 = status
            .trim()
            .parse()
            .map_err(|_| format!("line {line}: Status {status}"))?;
        if status < 3 {
            continue;
        }
        // `double.Parse(..., CultureInfo.InvariantCulture)`
        let number = |text: Option<&str>, name: &str| -> Result<f64, String> {
            text.and_then(|text| text.trim().parse::<f64>().ok())
                .ok_or_else(|| format!("line {line}: {name} {text:?}"))
        };
        let position = (
            number(item.get(dflog, "Lat"), "Lat")?,
            number(item.get(dflog, "Lng"), "Lng")?,
            number(item.get(dflog, "Alt"), "Alt")?,
        );
        let time = clock.time_ticks(dflog, item);
        let previous = *last.get_or_insert(position);
        if start.is_none() {
            info.date_ticks = time;
            start = Some(time);
        }
        end = time;
        let home = *info.home.get_or_insert(position);
        // "add distance" once a second: `dfItem.time > tia.AddSeconds(1)`.
        if time > in_air_until + 1000 * TICKS_PER_MS {
            let distance = mp_units::LatLon::new(previous.0, previous.1)
                .ok()
                .zip(mp_units::LatLon::new(position.0, position.1).ok())
                .map_or(0.0, |(from, to)| from.distance_to(to).0);
            #[allow(clippy::cast_possible_truncation)] // `(float)`
            {
                info.dist_traveled += distance as f32;
            }
            last = Some(position);
            // "ground speed > 0.2 or alt > homelat+2"
            if number(item.get(dflog, "Spd"), "Spd")? > 0.2 || position.2 > home.2 + 2.0 {
                info.time_in_air += 1.0;
            }
            in_air_until = time;
        }
    }
    info.duration = duration_text(end - start.unwrap_or(0));
    info.cam_msg = i32::try_from(buffer.items_of(&["CAM"]).len()).unwrap_or(i32::MAX);
    info.aircraft = 0;
    DFLOG_FRAME.clone_into(&mut info.frame);
    Ok(())
}

/// `DFItem.time`: the GPS time of the log's first fix, plus how long after that fix a line was
/// logged; before a fix, from `DateTime.MinValue`. `// C#: ExtLibs/Utilities/DFLog.cs:42-51,
/// 100-130, 163-208`
#[derive(Debug, Default)]
struct GpsClock {
    /// `gpsstarttime`, Unix milliseconds, once a fix gives one.
    start_unix_ms: Option<i64>,
    /// `msoffset`.
    offset_ms: f64,
}

impl GpsClock {
    /// A GPS line read: the first with a fix and a time sets the clock.
    fn see(
        &mut self,
        dflog: &mut mp_log::dflogbuffer::DfLog,
        item: &mp_log::dflogbuffer::DfItem,
        text: &str,
    ) {
        if self.start_unix_ms.is_some() || !item.msgtype().starts_with("GPS") {
            return;
        }
        let Some(start) = dflog.gps_time(text) else {
            return;
        };
        self.start_unix_ms = Some(start);
        // `msoffset = int.Parse(T)`, then `long.Parse(TimeUS) / 1000`.
        if let Some(t) = item
            .get(dflog, "T")
            .and_then(|t| t.trim().parse::<i64>().ok())
        {
            #[allow(clippy::cast_precision_loss)] // milliseconds since boot
            {
                self.offset_ms = t as f64;
            }
        }
        if let Some(us) = item
            .get(dflog, "TimeUS")
            .and_then(|t| t.trim().parse::<i64>().ok())
        {
            #[allow(clippy::cast_precision_loss)] // milliseconds since boot
            {
                self.offset_ms = (us / 1000) as f64;
            }
        }
    }

    /// `time`, as ticks.
    fn time_ticks(
        &self,
        dflog: &mut mp_log::dflogbuffer::DfLog,
        item: &mp_log::dflogbuffer::DfItem,
    ) -> i64 {
        // `timems`: TimeMS, else TimeUS / 1000.0, else T.
        let mut parse = |name: &str| -> Option<f64> {
            item.get(dflog, name)
                .and_then(|text| text.trim().parse::<i64>().ok())
                .map(|value| {
                    #[allow(clippy::cast_precision_loss)] // a log's clock
                    let value = value as f64;
                    value
                })
        };
        let time_ms = parse("TimeMS")
            .or_else(|| parse("TimeUS").map(|us| us / 1000.0))
            .or_else(|| parse("T"))
            .unwrap_or(0.0);
        let start_ticks = self
            .start_unix_ms
            .map_or(0, |ms| UNIX_EPOCH_TICKS + ms * TICKS_PER_MS);
        #[allow(clippy::cast_possible_truncation)] // `(Int64)(... * 10000)`
        let since = ((time_ms - self.offset_ms) * 10_000.0) as i64;
        start_ticks + since
    }
}

// ---- The form ----------------------------------------------------------------------------------

/// `ClientSize`, and the list's place in it. `// C#: Log/LogIndex.Designer.cs:83-87, 222`
const FORM_WIDTH: f32 = 1177.0;
const FORM_HEIGHT: f32 = 514.0;
const LIST_TOP: f32 = 42.0;
const LIST_LEFT: f32 = 12.0;
const LIST_WIDTH: f32 = 1153.0;
const LIST_HEIGHT: f32 = 460.0;
/// `RowHeight`, and the picture's size in its cell: `new Bitmap(info.img, 150, 150)`.
const ROW_HEIGHT: f32 = 150.0;
const PICTURE: u32 = 150;
/// The columns, in order: their headers and widths - 60 where the designer gives none, as
/// ObjectListView's columns default. `// C#: Log/LogIndex.Designer.cs:96-174`
const COLUMNS: [(&str, f32); 12] = [
    ("", 150.0),
    ("Date", 100.0),
    ("Directory", 258.0),
    ("Frame", 60.0),
    ("Aircraft", 60.0),
    ("Duration", 100.0),
    ("FileName", 178.0),
    ("Size", 60.0),
    ("Home", 60.0),
    ("TimeInAir, sec", 60.0),
    ("DistTraveled, m", 60.0),
    ("CamMSG", 60.0),
];
/// `Loading.ShowLoading`'s words. `// C#: LogIndex.cs:60, 84`
const SCANNING: &str = "Scanning for files";
const POPULATING: &str = "Populating Data";
/// What a deleted row reads. `// C#: LogIndex.cs:381, 401`
const DELETED: &str = "--- DELETED ---";

/// The form's work on its thread: what `Loading` says, then the rows.
enum Progress {
    Loading(String),
    Done(Vec<(LogInfo, Option<image::RgbaImage>)>),
}

/// What the form is asking.
enum Asking {
    /// Custom Directory's `FolderBrowserDialog`, the folder typed.
    Folder(crate::config::optional::InputBox),
    /// "Do you really want to delete N logs?"
    Delete,
    /// A delete that failed, in a box as the C# shows it.
    Failed(String),
}

/// A log read, and its picture as drawn.
type Read = (LogInfo, Option<std::sync::Arc<gpui::RenderImage>>);

/// The form: its rows, which are selected, what `Loading` says while it reads, and the question
/// it is asking.
pub struct LogIndexForm {
    /// `logs`: every log read since the form opened, with its picture. The C# never clears it.
    read: Vec<Read>,
    /// The list's rows, in order: each one of `read`, as the list holds the same objects as
    /// `logs` - so a second scan's `AddObjects(logs)` shows the first's rows again, and a row
    /// deleted reads so wherever it shows.
    rows: Vec<usize>,
    /// The selected rows.
    pub selected: std::collections::BTreeSet<usize>,
    /// Where a shift-click's range starts.
    anchor: Option<usize>,
    /// `PrimarySortColumn` and whether `PrimarySortOrder` is descending: none until a column's
    /// header is clicked.
    pub sort: Option<(usize, bool)>,
    /// `Loading`'s words while a scan runs.
    pub loading: Option<String>,
    job: Option<std::sync::mpsc::Receiver<Progress>>,
    asking: Option<Asking>,
    /// The folder box's keyboard, made the first time it shows.
    focus: Option<gpui::FocusHandle>,
    scroll: gpui::ScrollHandle,
}

impl Default for LogIndexForm {
    fn default() -> Self {
        Self::new()
    }
}

impl LogIndexForm {
    /// `new LogIndex()`: empty until a folder is chosen; `LogIndex_Load` does nothing.
    /// `// C#: LogIndex.cs:23-32`
    pub fn new() -> Self {
        Self {
            read: Vec::new(),
            rows: Vec::new(),
            selected: std::collections::BTreeSet::new(),
            anchor: None,
            sort: None,
            loading: None,
            job: None,
            asking: None,
            focus: None,
            scroll: gpui::ScrollHandle::new(),
        }
    }

    /// The row at `index`, with its picture.
    fn row(&self, index: usize) -> Option<&Read> {
        self.rows.get(index).and_then(|&read| self.read.get(read))
    }

    /// The rows, in order.
    #[cfg(test)]
    pub fn rows(&self) -> impl Iterator<Item = &LogInfo> {
        (0..self.rows.len()).filter_map(|index| self.row(index).map(|(info, _)| info))
    }

    /// The selected rows, in order.
    fn chosen(&self) -> Vec<&LogInfo> {
        self.selected
            .iter()
            .filter_map(|&index| self.row(index).map(|(info, _)| info))
            .collect()
    }

    /// A scan's logs added to `logs`, then `AddObjects(logs)`: every log read so far added to the
    /// list again. `// C#: LogIndex.cs:84-91, 279-280`
    ///
    /// `AddObjects` then `BuildList`s, which sorts again by the column last sorted by and puts the
    /// selection back as `DoSort` does. `// C#: ExtLibs/ObjectListView/VirtualObjectListView.cs:472-492, 799-807`
    fn adopt(&mut self, logs: Vec<Read>) {
        let chosen = self.chosen_logs();
        self.read.extend(logs);
        self.rows.extend(0..self.read.len());
        self.sort_rows();
        self.reselect(&chosen);
    }

    /// The logs the selected rows show (`SelectedObjects`).
    fn chosen_logs(&self) -> Vec<usize> {
        self.selected
            .iter()
            .filter_map(|&index| self.rows.get(index).copied())
            .collect()
    }

    /// `SelectedObjects = selection`: each log selected where the list finds it - its last row, as
    /// `FastObjectListDataSource`'s index map holds a log shown twice.
    /// `// C#: ExtLibs/ObjectListView/VirtualObjectListView.cs:677-692; FastObjectListView.cs:198-205, 377-381`
    fn reselect(&mut self, logs: &[usize]) {
        self.selected = logs
            .iter()
            .filter_map(|log| self.rows.iter().rposition(|row| row == log))
            .collect();
    }

    /// The rows in the order of the column last sorted by, if any: `ModelObjectComparer`'s, with no
    /// second column (the list's `SecondarySortColumn` is none). A tie is left where it was, where
    /// the C#'s `List.Sort` - an introsort - may swap two. `// C#: ExtLibs/ObjectListView/FastObjectListView.cs:238-245`
    fn sort_rows(&mut self) {
        let Some((column, descending)) = self.sort else {
            return;
        };
        let read = &self.read;
        self.rows
            .sort_by(|&a, &b| match (read.get(a), read.get(b)) {
                (Some((a, _)), Some((b, _))) => {
                    let order = compare(a, b, column);
                    if descending { order.reverse() } else { order }
                }
                _ => std::cmp::Ordering::Equal,
            });
    }
}

/// A column's header clicked: sorted by it, ascending - or the other way, when it is the column
/// last sorted by - and the selection put back on the same logs.
/// `// C#: ExtLibs/ObjectListView/ObjectListView.cs:5587-5603, 8241-8300`
fn sort_by(form: &mut LogIndexForm, column: usize) {
    let descending = match form.sort {
        Some((last, descending)) if last == column => !descending,
        _ => false,
    };
    let chosen = form.chosen_logs();
    form.sort = Some((column, descending));
    form.sort_rows();
    form.reselect(&chosen);
}

/// `ModelObjectComparer.Compare` on one column's aspect, ascending: strings by the culture with
/// case ignored, numbers and dates by value, a home - `PointLatLngAlt.CompareTo`, by a `Tag` these
/// have none of - equal to another, and a missing one first. The picture's column has no aspect:
/// every row equal. `// C#: ExtLibs/ObjectListView/Implementation/Comparers.cs:265-317;
/// ExtLibs/Utilities/PointLatLngAlt.cs:441-467`
fn compare(a: &LogInfo, b: &LogInfo, column: usize) -> std::cmp::Ordering {
    use mp_log::netfmt::culture_compare_ignore_case as text;
    use std::cmp::Ordering::{Equal, Greater, Less};
    // `float.CompareTo`: NaN before every number and equal to itself.
    let single = |a: f32, b: f32| {
        a.partial_cmp(&b)
            .unwrap_or_else(|| b.is_nan().cmp(&a.is_nan()))
    };
    match column {
        1 => a.date_ticks.cmp(&b.date_ticks),
        2 => text(&a.directory(), &b.directory()),
        3 => text(&a.frame, &b.frame),
        4 => a.aircraft.cmp(&b.aircraft),
        5 => text(&a.duration, &b.duration),
        6 => text(&a.name(), &b.name()),
        7 => a.size.cmp(&b.size),
        8 => match (a.home.is_some(), b.home.is_some()) {
            (false, true) => Less,
            (true, false) => Greater,
            _ => Equal,
        },
        9 => single(a.time_in_air, b.time_in_air),
        10 => single(a.dist_traveled, b.dist_traveled),
        11 => a.cam_msg.cmp(&b.cam_msg),
        _ => Equal,
    }
}

/// `butlogindex_Click`: the form shown, or brought back as it was.
pub fn open(this: &mut MissionPlanner) {
    if this.experimental.log_index.is_none() {
        this.experimental.log_index = Some(LogIndexForm::new());
    }
}

/// `createFileList` and `queueRunner` on a thread of their own: "Scanning for files", each log in
/// turn as `a/count file`, "Populating Data", and the rows. The pictures are LogMap's, over the
/// tile cache - and the server too unless the map is to fetch nothing.
/// `// C#: LogIndex.cs:58-92, 102-106`
fn scan(this: &mut MissionPlanner, folder: PathBuf) {
    let fetch = std::env::var("MP_OFFLINE").is_err()
        && !crate::config::planner::cache_only(&this.persisted);
    let Some(form) = this.experimental.log_index.as_mut() else {
        return;
    };
    if form.job.is_some() {
        return;
    }
    let (sender, receiver) = std::sync::mpsc::channel();
    let spawned = wasm_thread::Builder::new()
        .name("mp-log-index".to_owned())
        .spawn(move || {
            let _ = sender.send(Progress::Loading(SCANNING.to_owned()));
            let logs = files(&folder);
            let cache = mp_tiles::TileCache::new(mp_tiles::TileCache::default_root());
            let fetcher = fetch.then(mp_tiles::TileFetcher::new);
            let mut tile = |id| crate::log_map::cached_or_fetched(&cache, fetcher.as_ref(), id);
            let mut rows = Vec::new();
            for (index, log) in logs.iter().enumerate() {
                let _ = sender.send(Progress::Loading(format!(
                    "{}/{} {}",
                    index + 1,
                    logs.len(),
                    log.display()
                )));
                if let Some(info) = process(log, &mut tile) {
                    let picture = info.picture.as_deref().and_then(thumbnail);
                    rows.push((info, picture));
                }
            }
            let _ = sender.send(Progress::Loading(POPULATING.to_owned()));
            let _ = sender.send(Progress::Done(rows));
        });
    match spawned {
        Ok(_) => form.job = Some(receiver),
        Err(error) => log::debug!("LogIndex: {error}"),
    }
}

/// `new Bitmap(info.img, 150, 150)`: the picture stretched to 150 by 150, as BGRA for gpui.
fn thumbnail(picture: &Path) -> Option<image::RgbaImage> {
    let bytes = mp_os::fs::read(picture).ok()?;
    let image = image::load_from_memory(&bytes).ok()?.to_rgba8();
    let mut small = image::imageops::resize(
        &image,
        PICTURE,
        PICTURE,
        image::imageops::FilterType::Triangle,
    );
    for pixel in small.pixels_mut() {
        pixel.0.swap(0, 2);
    }
    Some(small)
}

/// Once a frame: what the scan says, and its rows when it is done - `AddObjects(logs)`, every row
/// read so far added again. `// C#: LogIndex.cs:84-91`
pub fn tick(this: &mut MissionPlanner) {
    let Some(form) = this.experimental.log_index.as_mut() else {
        return;
    };
    let Some(receiver) = form.job.as_ref() else {
        return;
    };
    crate::repaint::in_flight();
    loop {
        match receiver.try_recv() {
            Ok(Progress::Loading(text)) => form.loading = Some(text),
            Ok(Progress::Done(rows)) => {
                form.adopt(
                    rows.into_iter()
                        .map(|(info, picture)| {
                            let picture = picture.map(|picture| {
                                std::sync::Arc::new(gpui::RenderImage::new(vec![
                                    image::Frame::new(picture),
                                ]))
                            });
                            (info, picture)
                        })
                        .collect(),
                );
                form.loading = None;
                form.job = None;
                return;
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => return,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                form.loading = None;
                form.job = None;
                return;
            }
        }
    }
}

/// A row clicked: chosen alone; with Control, added or taken out; with Shift, the rows from the
/// last chosen to it. `// C#: ObjectListView's MultiSelect, FullRowSelect`
fn click_row(form: &mut LogIndexForm, index: usize, control: bool, shift: bool) {
    if shift && let Some(anchor) = form.anchor {
        form.selected = (anchor.min(index)..=anchor.max(index)).collect();
        return;
    }
    if control {
        if !form.selected.remove(&index) {
            form.selected.insert(index);
        }
    } else {
        form.selected = std::iter::once(index).collect();
    }
    form.anchor = Some(index);
}

/// `btnDeleteLog_Click`'s Yes: each selected row's files - `<name>.*` beside it, as Windows
/// matches the mask, the log itself among them, and a telemetry log's `.rlog` - deleted, and the
/// row marked deleted with its times gone. Its picture file is forgotten, but the picture shown
/// stays: the C#'s `img` keeps the bitmap it loaded for the row's cell (`LogIndex.cs:300-302`). A
/// file that will not go is said, as the C#'s box says it, and the rest go on.
/// `// C#: LogIndex.cs:372-410`
fn delete_selected(form: &mut LogIndexForm) -> Vec<String> {
    let mut failures = Vec::new();
    let selected: Vec<usize> = form.selected.iter().copied().collect();
    for index in selected {
        let Some((row, _)) = form
            .rows
            .get(index)
            .copied()
            .and_then(|read| form.read.get_mut(read))
        else {
            continue;
        };
        if row.fullname.as_os_str() == DELETED {
            continue;
        }
        for file in files_of(&row.fullname) {
            if file.os_exists()
                && let Err(error) = mp_os::fs::remove_file(&file)
            {
                failures.push(format!(
                    "Error has been occured when trying to delete {}\r\n{error}",
                    row.fullname.display()
                ));
            }
        }
        row.fullname = PathBuf::from(DELETED);
        row.picture = None;
        row.time_in_air = 0.0;
        row.dist_traveled = 0.0;
        row.duration = String::new();
    }
    failures
}

/// `Directory.GetFiles(folder, name + ".*")` - the log and every file named after it with a
/// further extension - and, for a telemetry log, its `.rlog`. `// C#: LogIndex.cs:384-386`
#[must_use]
pub fn files_of(log: &Path) -> Vec<PathBuf> {
    let Some(name) = log
        .file_name()
        .map(|name| name.to_string_lossy().to_lowercase())
    else {
        return Vec::new();
    };
    let folder = log.parent().unwrap_or_else(|| Path::new("."));
    let mut files: Vec<PathBuf> = mp_os::fs::read_dir(folder)
        .map(|entries| {
            entries
                .filter_map(Result::ok)
                .map(|entry| entry.path())
                .filter(|path| {
                    path.file_name().is_some_and(|other| {
                        let other = other.to_string_lossy().to_lowercase();
                        other == name || other.starts_with(&format!("{name}."))
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    files.sort();
    if log
        .extension()
        .is_some_and(|ext| ext.to_string_lossy().eq_ignore_ascii_case("tlog"))
    {
        files.push(log.with_extension("rlog"));
    }
    files
}

/// The ids of the form's boxes.
const IDS: crate::config::firmware::BoxIds = crate::config::firmware::BoxIds {
    question: "experimental-logindex-question",
    yes: "experimental-logindex-question-yes",
    no: "experimental-logindex-question-no",
    message: "experimental-logindex-message",
    ok: "experimental-logindex-message-ok",
    path: "experimental-logindex-path",
    path_value: "experimental-logindex-path-value",
    path_ok: "experimental-logindex-path-ok",
    path_cancel: "experimental-logindex-path-cancel",
};

/// `but_defaultlogdir_Click`: the log directory read. `// C#: LogIndex.cs:440-445`
fn default_directory(this: &mut MissionPlanner) {
    if let Some(folder) = crate::fly::log_directory() {
        scan(this, folder);
    }
}

/// `BUT_changedir_Click`: the folder asked for - typed, as every folder dialog here - with
/// nothing offered, as the C#'s dialog sets no path. `// C#: LogIndex.cs:359-370`
fn custom_directory(
    this: &mut MissionPlanner,
    window: &mut Window,
    cx: &mut Context<MissionPlanner>,
) {
    let Some(form) = this.experimental.log_index.as_mut() else {
        return;
    };
    form.asking = Some(Asking::Folder(crate::config::optional::InputBox::new(
        crate::inject_map::FOLDER_TITLE,
        "",
        "",
    )));
    let focus = form.focus.get_or_insert_with(|| cx.focus_handle()).clone();
    focus.focus(window, cx);
}

/// The folder's answer: OK reads it - the rows cleared of nothing, as `files.Clear()` clears only
/// the list of files; Cancel does nothing.
fn folder_answered(this: &mut MissionPlanner, ok: bool) {
    let Some(form) = this.experimental.log_index.as_mut() else {
        return;
    };
    let Some(Asking::Folder(input)) = form.asking.take() else {
        return;
    };
    if ok {
        scan(this, PathBuf::from(input.field.value().trim()));
    }
}

/// "Do you really want to delete N logs?" answered. `// C#: LogIndex.cs:374-409`
fn delete_answered(this: &mut MissionPlanner, yes: bool) {
    let Some(form) = this.experimental.log_index.as_mut() else {
        return;
    };
    form.asking = None;
    if !yes {
        return;
    }
    let failures = delete_selected(form);
    if !failures.is_empty() {
        form.asking = Some(Asking::Failed(failures.join("\n")));
    }
}

/// The form, over the tab, as `form.Show()` leaves the tab beside it: the two folder buttons, the
/// selection's totals and Delete, the list - a picture and eleven columns a row - and what
/// `Loading` says while it reads. `// C#: Log/LogIndex.Designer.cs:29-232`
pub fn window(
    this: &MissionPlanner,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = this.experimental.log_index.as_ref()?;
    let chosen = form.chosen();
    let header = div()
        .flex()
        .items_center()
        .justify_between()
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child("LogIndex"),
        )
        .child(crate::ui::action(
            "experimental-logindex-close",
            "\u{2715}",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.experimental.log_index = None;
                cx.notify();
            }),
        ));
    let buttons = div()
        .relative()
        .h(px(30.0))
        .child(
            div()
                .absolute()
                .left(px(LIST_LEFT))
                .top(px(4.0))
                .child(crate::ui::action(
                    "experimental-logindex-default",
                    "Default Log Dir",
                    theme::ACCENT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        default_directory(this);
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .absolute()
                .left(px(117.0))
                .top(px(4.0))
                .child(crate::ui::action(
                    "experimental-logindex-custom",
                    "Custom Directory",
                    theme::ACCENT,
                    true,
                    cx.listener(|this, _event: &(), window, cx| {
                        custom_directory(this, window, cx);
                        cx.notify();
                    }),
                )),
        )
        .child(
            crate::probe::measured("experimental-logindex-stats", div())
                .absolute()
                .left(px(683.0))
                .top(px(6.0))
                .w(px(372.0))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(stats_text(&chosen)),
        )
        .child(
            div()
                .absolute()
                .left(px(1061.0))
                .top(px(4.0))
                .child(crate::ui::action(
                    "experimental-logindex-delete",
                    "Delete selected",
                    theme::ACCENT,
                    !form.selected.is_empty(),
                    cx.listener(|this, _event: &(), _window, cx| {
                        if let Some(form) = this.experimental.log_index.as_mut()
                            && !form.selected.is_empty()
                        {
                            form.asking = Some(Asking::Delete);
                        }
                        cx.notify();
                    }),
                )),
        );
    let mut heading = div()
        .flex()
        .h(px(22.0))
        .border_b_1()
        .border_color(rgb(theme::BORDER));
    // A header clicked sorts by its column; the column sorted by shows which way
    // (`ShowSortIndicators`, on by default).
    for (column, (title, width)) in COLUMNS.into_iter().enumerate() {
        let indicator = match form.sort {
            Some((sorted, false)) if sorted == column => " \u{25b4}",
            Some((sorted, true)) if sorted == column => " \u{25be}",
            _ => "",
        };
        let id = format!("experimental-logindex-header-{column}");
        heading = heading.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .w(px(width))
                .flex_shrink_0()
                .px_1()
                .text_xs()
                .text_color(rgb(theme::DIM))
                // One line, cut short with an ellipsis, as a ListView's header is.
                .truncate()
                .cursor_pointer()
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    if let Some(form) = this.experimental.log_index.as_mut() {
                        sort_by(form, column);
                    }
                    cx.notify();
                }))
                .child(format!("{title}{indicator}")),
        );
    }
    let mut list = div().flex().flex_col().child(heading);
    for (index, (row, picture)) in
        (0..form.rows.len()).filter_map(|index| Some((index, form.row(index)?)))
    {
        let selected = form.selected.contains(&index);
        let picture = picture.clone();
        let mut line = crate::probe::measured(format!("experimental-logindex-row-{index}"), div())
            .id(SharedString::from(format!(
                "experimental-logindex-row-{index}"
            )))
            .flex()
            .h(px(ROW_HEIGHT))
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, window, cx| {
                let modifiers = window.modifiers();
                if let Some(form) = this.experimental.log_index.as_mut() {
                    click_row(form, index, modifiers.control, modifiers.shift);
                }
                cx.notify();
            }));
        // The picture's column is never drawn selected (`NonSelectableRenderer`).
        line = line.child(div().w(px(COLUMNS[0].1)).h_full().flex_shrink_0().children(
            picture.map(|picture| {
                gpui::canvas(
                    |_, _, _| {},
                    move |bounds, (), window, _| {
                        let _ = window.paint_image(
                            bounds,
                            bounds,
                            gpui::Corners::default(),
                            std::sync::Arc::clone(&picture),
                            0,
                            false,
                        );
                    },
                )
                .size(px(f32::from(u16::try_from(PICTURE).unwrap_or(150))))
            }),
        ));
        for (text, (_, width)) in row.cells().into_iter().zip(COLUMNS.iter().skip(1)) {
            line = line.child(
                div()
                    .w(px(*width))
                    .h_full()
                    .flex_shrink_0()
                    .px_1()
                    .flex()
                    .items_center()
                    .overflow_hidden()
                    .text_xs()
                    .bg(rgb(if selected {
                        theme::ACTION
                    } else {
                        theme::PANEL
                    }))
                    .text_color(rgb(theme::TEXT))
                    .child(text),
            );
        }
        list = list.child(line);
    }
    let body = crate::probe::measured("experimental-logindex", div())
        .id("experimental-logindex")
        .relative()
        .flex()
        .flex_col()
        .gap_1()
        .w(px(FORM_WIDTH))
        .h(px(FORM_HEIGHT + 30.0))
        .p_2()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .occlude()
        .child(header)
        .child(buttons)
        .child(
            div()
                .id("experimental-logindex-list")
                .ml(px(LIST_LEFT - 8.0))
                .w(px(LIST_WIDTH))
                .h(px(LIST_HEIGHT))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .overflow_scroll()
                .track_scroll(&form.scroll)
                .child(list),
        )
        .children(form.loading.as_ref().map(|text| {
            crate::probe::measured("experimental-logindex-loading", div())
                .absolute()
                .top(px(LIST_TOP + 60.0))
                .left(px(FORM_WIDTH / 2.0 - 200.0))
                .w(px(400.0))
                .p_3()
                .bg(rgb(theme::PANEL))
                .border_1()
                .border_color(rgb(theme::ACCENT))
                .rounded_md()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(text.clone())
        }));
    let asking = asking_box(form, window, cx);
    let size = window.viewport_size();
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(
                    ((size.width - px(FORM_WIDTH)) / 2.0).max(px(0.0)),
                    px(80.0),
                ))
                .child(div().relative().child(body).children(asking)),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

/// The question the form is asking, over it.
fn asking_box(
    form: &LogIndexForm,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    match form.asking.as_ref()? {
        Asking::Folder(input) => Some(crate::config::optional::input_box(
            "experimental-input-box",
            input,
            form.focus.as_ref()?,
            window,
            |this, event| {
                let outcome =
                    match this.experimental.log_index.as_mut().and_then(|form| {
                        match form.asking.as_mut() {
                            Some(Asking::Folder(input)) => Some(input.field.key(event)),
                            _ => None,
                        }
                    }) {
                        Some(outcome) => outcome,
                        None => return false,
                    };
                match outcome {
                    crate::textfield::KeyOutcome::Submitted => folder_answered(this, true),
                    crate::textfield::KeyOutcome::Cancelled => folder_answered(this, false),
                    crate::textfield::KeyOutcome::Changed => {}
                    crate::textfield::KeyOutcome::Ignored => return false,
                }
                true
            },
            |this| folder_answered(this, true),
            |this| folder_answered(this, false),
            cx,
        )),
        // `CustomMessageBox.Show(string.Format("Do you really want to delete {0} logs?", n), YesNo)`
        Asking::Delete => Some(crate::config::firmware::question_box(
            IDS,
            "",
            &format!("Do you really want to delete {} logs?", form.selected.len()),
            mp_firmware::flow::Buttons::YesNo,
            window,
            delete_answered,
            cx,
        )),
        Asking::Failed(text) => Some(crate::config::firmware::message_box(
            IDS,
            &crate::config::firmware::Waiting {
                text: text.clone(),
                caption: String::new(),
                buttons: None,
            },
            window,
            |this| {
                if let Some(form) = this.experimental.log_index.as_mut() {
                    form.asking = None;
                }
            },
            cx,
        )),
    }
}

/// `experimental.logindex.*`: whether the form shows, its rows - each's columns, a picture or
/// not - the selection and its totals, what `Loading` says and the question showing.
pub fn record_facts(form: Option<&LogIndexForm>) {
    use crate::facts::record;
    record("experimental.logindex.open", form.is_some());
    let Some(form) = form else {
        return;
    };
    record("experimental.logindex.rows", form.rows.len());
    record(
        "experimental.logindex.loading",
        form.loading.as_deref().unwrap_or("none"),
    );
    record("experimental.logindex.stats", stats_text(&form.chosen()));
    record(
        "experimental.logindex.selected",
        form.selected
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "experimental.logindex.sort",
        form.sort.map_or_else(
            || "none".to_owned(),
            |(column, descending)| {
                format!(
                    "{} {}",
                    COLUMNS.get(column).map_or("", |(title, _)| *title),
                    if descending {
                        "descending"
                    } else {
                        "ascending"
                    }
                )
            },
        ),
    );
    record(
        "experimental.logindex.delete.enabled",
        !form.selected.is_empty(),
    );
    record(
        "experimental.logindex.asking",
        match form.asking.as_ref() {
            None => "none".to_owned(),
            Some(Asking::Folder(input)) => format!("folder: {}", input.field.value()),
            Some(Asking::Delete) => {
                format!("Do you really want to delete {} logs?", form.selected.len())
            }
            Some(Asking::Failed(text)) => text.clone(),
        },
    );
    for (index, (row, picture)) in (0..form.rows.len())
        .filter_map(|index| Some((index, form.row(index)?)))
        .take(20)
    {
        record(
            format!("experimental.logindex.row.{index}"),
            row.cells().join(" | "),
        );
        record(
            format!("experimental.logindex.row.{index}.picture"),
            picture.is_some(),
        );
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    fn testdata(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    fn scratch(name: &str) -> PathBuf {
        let folder = mp_os::temp_dir().join(format!("mp-logindex-{}-{name}", std::process::id()));
        let _ = mp_os::fs::remove_dir_all(&folder);
        mp_os::fs::create_dir_all(&folder).unwrap();
        folder
    }

    /// A copy of a test log in `folder`, its row as `processbg` makes it, no tiles to be had.
    fn row_of(folder: &Path, log: &str, as_name: &str) -> Option<LogInfo> {
        let file = folder.join(as_name);
        mp_os::fs::write(&file, mp_os::fs::read(testdata(log)).unwrap()).unwrap();
        process(&file, &mut |_| None)
    }

    /// The values as the list writes them: a float's seven digits, a `TimeSpan`, `ToHours`, the
    /// date `DateTime.MinValue` shows.
    #[test]
    fn values_are_written_as_dotnet_writes_them() {
        assert_eq!(single_text(0.0), "0");
        assert_eq!(single_text(451.454_5), "451.4545");
        assert_eq!(single_text(0.5), "0.5");
        assert_eq!(single_text(12_345_678.0), "1.234568E+07");
        assert_eq!(duration_text(0), "00:00:00");
        assert_eq!(duration_text(15_000_000), "00:00:01.5000000");
        assert_eq!(duration_text((86_400 + 7_200) * 10_000_000), "1.02:00:00");
        assert_eq!(duration_text(-10_000_000), "-00:00:01");
        assert_eq!(to_hours(3_725.0), "01:02:05");
        assert_eq!(to_hours(36_000.0), "10:00:00");
        assert_eq!(date_text(0), "1/1/0001 12:00:00 AM");
        let one = LogInfo {
            time_in_air: 61.0,
            dist_traveled: 100.7,
            ..LogInfo::new(Path::new("a.tlog"))
        };
        let two = LogInfo {
            time_in_air: 3_600.0,
            dist_traveled: 1.0,
            ..LogInfo::new(Path::new("b.tlog"))
        };
        assert_eq!(
            stats_text(&[&one, &two]),
            "Selected: 2; TimeInAir: 01:01:01; DistTraveled: 101m"
        );
    }

    /// A simulated copter's telemetry log, replayed: the vehicle the first heartbeat names, its
    /// type, the time from its heartbeat to its last packet, its time in air and distance as
    /// `UpdateCurrentSettings` counts them, where it is at the end, the camera's feedbacks.
    #[test]
    fn a_telemetry_log_is_replayed() {
        let folder = scratch("tlog");
        let row = row_of(&folder, "georef/camera.tlog", "camera.tlog").unwrap();
        assert_eq!(row.aircraft, 1);
        assert_eq!(row.frame, "QUADROTOR");
        assert_eq!(row.duration, "00:02:35.7960000");
        assert_eq!(row.size, 422_834);
        assert_eq!(single_text(row.time_in_air), "46");
        assert_eq!(single_text(row.dist_traveled), "451.4545");
        assert_eq!(row.cam_msg, 25);
        assert_ne!(row.date_ticks, 0);
        assert_eq!(row.cells()[7], "-27.4691978,153.0272559,25.0900001525879,");
        assert!(row.picture.is_some(), "LogMap's picture made first");
        // Under 4 KiB: no row.
        mp_os::fs::write(folder.join("small.tlog"), [0u8; 100]).unwrap();
        assert!(process(&folder.join("small.tlog"), &mut |_| None).is_none());
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// A dataflash log's GPS lines with a fix; one with none has no date, duration or home.
    #[test]
    fn a_dataflash_log_is_read_from_its_gps() {
        let folder = scratch("bin");
        let row = row_of(&folder, "georef/camera.bin", "camera.bin").unwrap();
        assert_eq!(row.frame, "DFLog Unknown");
        assert_eq!(row.aircraft, 0);
        assert_eq!(row.duration, "00:02:03.1998670");
        assert_eq!(single_text(row.time_in_air), "91");
        assert_eq!(single_text(row.dist_traveled), "451.9137");
        assert_eq!(row.cam_msg, 25);
        assert_eq!(row.home, Some((-27.4698, 153.025_099_9, 25.1)));
        let none = row_of(&folder, "dataflash.bin", "bench.bin").unwrap();
        assert_eq!(none.date_ticks, 0);
        assert_eq!(none.duration, "00:00:00");
        assert_eq!(none.home, None);
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// The logs in and under a folder, telemetry logs first; a log's files are its own name and
    /// that name with more after a dot, and a telemetry log's `.rlog`.
    #[test]
    fn the_files_are_found_and_named() {
        let folder = scratch("files");
        mp_os::fs::create_dir_all(folder.join("deep")).unwrap();
        for name in [
            "b.bin",
            "a.tlog",
            "deep/c.tlog",
            "a.tlog.jpg",
            "a.tlog2",
            "c.log",
        ] {
            mp_os::fs::write(folder.join(name), b"x").unwrap();
        }
        let found: Vec<String> = files(&folder)
            .iter()
            .map(|path| path.strip_prefix(&folder).unwrap().display().to_string())
            .collect();
        assert_eq!(found, ["a.tlog", "deep/c.tlog", "b.bin", "c.log"]);
        let names: Vec<String> = files_of(&folder.join("a.tlog"))
            .iter()
            .map(|path| path.file_name().unwrap().to_string_lossy().into_owned())
            .collect();
        assert_eq!(names, ["a.tlog", "a.tlog.jpg", "a.rlog"]);
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    fn form() -> LogIndexForm {
        LogIndexForm::new()
    }

    /// A click chooses one row; Control adds or takes one out; Shift takes the range.
    #[test]
    fn rows_are_chosen_as_a_list_view_chooses_them() {
        let mut form = form();
        click_row(&mut form, 2, false, false);
        click_row(&mut form, 4, true, false);
        assert_eq!(form.selected.iter().copied().collect::<Vec<_>>(), [2, 4]);
        click_row(&mut form, 2, true, false);
        assert_eq!(form.selected.iter().copied().collect::<Vec<_>>(), [4]);
        // The anchor is the row last clicked, a Control-click's too.
        click_row(&mut form, 1, false, true);
        assert_eq!(form.selected.iter().copied().collect::<Vec<_>>(), [1, 2]);
        click_row(&mut form, 3, false, false);
        click_row(&mut form, 0, false, true);
        assert_eq!(
            form.selected.iter().copied().collect::<Vec<_>>(),
            [0, 1, 2, 3]
        );
    }

    fn log(name: &str, size: u64, frame: &str, home: bool) -> Read {
        let info = LogInfo {
            size,
            frame: frame.to_owned(),
            home: home.then_some((1.0, 2.0, 3.0)),
            ..LogInfo::new(Path::new(name))
        };
        (info, None)
    }

    fn names(form: &LogIndexForm) -> Vec<String> {
        form.rows().map(LogInfo::name).collect()
    }

    /// A header clicked sorts by its column, ascending, and again the other way; the selection
    /// stays on the same logs. Text ignores case, a missing home comes first, a picture's column
    /// leaves the order as it is.
    #[test]
    fn a_header_sorts_by_its_column() {
        let mut form = form();
        form.adopt(vec![
            log("b.tlog", 30, "quadrotor", true),
            log("a.tlog", 10, "QUADROTOR", false),
            log("c.tlog", 20, "FIXED_WING", true),
        ]);
        click_row(&mut form, 0, false, false);
        sort_by(&mut form, 7);
        assert_eq!(names(&form), ["a.tlog", "c.tlog", "b.tlog"]);
        assert_eq!(form.sort, Some((7, false)));
        assert_eq!(
            form.chosen()[0].name(),
            "b.tlog",
            "the selection follows its log"
        );
        sort_by(&mut form, 7);
        assert_eq!(names(&form), ["b.tlog", "c.tlog", "a.tlog"]);
        assert_eq!(form.sort, Some((7, true)));
        // Another column: ascending again. Case aside, the two QUADROTORs tie and keep their order.
        sort_by(&mut form, 3);
        assert_eq!(names(&form), ["c.tlog", "b.tlog", "a.tlog"]);
        sort_by(&mut form, 8);
        assert_eq!(names(&form), ["a.tlog", "c.tlog", "b.tlog"]);
        sort_by(&mut form, 0);
        assert_eq!(names(&form), ["a.tlog", "c.tlog", "b.tlog"]);
        let nan = LogInfo {
            time_in_air: f32::NAN,
            ..LogInfo::new(Path::new("x"))
        };
        let one = LogInfo {
            time_in_air: 1.0,
            ..LogInfo::new(Path::new("y"))
        };
        assert_eq!(compare(&nan, &one, 9), std::cmp::Ordering::Less);
        assert_eq!(compare(&nan, &nan, 9), std::cmp::Ordering::Equal);
    }

    /// Logs added sort into the order of the column last sorted by, and a log selected is selected
    /// at its last row once it shows twice.
    #[test]
    fn logs_added_are_sorted_and_the_selection_moves_to_the_last_row() {
        let mut form = form();
        form.adopt(vec![
            log("b.tlog", 30, "", true),
            log("a.tlog", 10, "", true),
        ]);
        click_row(&mut form, 1, false, false);
        assert_eq!(form.chosen()[0].name(), "a.tlog");
        form.adopt(vec![log("c.tlog", 20, "", true)]);
        assert_eq!(
            names(&form),
            ["b.tlog", "a.tlog", "b.tlog", "a.tlog", "c.tlog"]
        );
        assert_eq!(form.selected.iter().copied().collect::<Vec<_>>(), [3]);
        sort_by(&mut form, 7);
        form.adopt(vec![log("d.tlog", 15, "", true)]);
        assert_eq!(
            names(&form),
            [
                "a.tlog", "a.tlog", "a.tlog", "d.tlog", "c.tlog", "c.tlog", "b.tlog", "b.tlog",
                "b.tlog"
            ]
        );
        assert_eq!(form.selected.iter().copied().collect::<Vec<_>>(), [2]);
    }

    /// A second scan adds every log read so far again, the first's among them, as the C#'s
    /// `AddObjects(logs)` does; a deleted log reads so wherever it shows, its files gone.
    #[test]
    fn rescans_and_deletes_as_the_csharp() {
        let folder = scratch("delete");
        let log = folder.join("a.tlog");
        mp_os::fs::write(&log, b"x").unwrap();
        mp_os::fs::write(folder.join("a.tlog.jpg"), b"x").unwrap();
        let mut form = form();
        form.adopt(vec![(LogInfo::new(&log), None)]);
        form.adopt(vec![(LogInfo::new(&folder.join("b.bin")), None)]);
        let names: Vec<String> = form.rows().map(LogInfo::name).collect();
        assert_eq!(names, ["a.tlog", "a.tlog", "b.bin"]);
        click_row(&mut form, 0, false, false);
        assert!(delete_selected(&mut form).is_empty());
        assert!(!log.exists());
        assert!(!folder.join("a.tlog.jpg").exists());
        let names: Vec<String> = form.rows().map(LogInfo::name).collect();
        assert_eq!(names, [DELETED, DELETED, "b.bin"]);
        let _ = mp_os::fs::remove_dir_all(&folder);
    }
}
