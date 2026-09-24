//! `GeoRefImageBase`: reading the log into positions, and matching photos to them in the three
//! modes - by a time offset, by `CAM` message, by `TRIG` message.
//! `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:34-1054`

use mp_log::netfmt;
use mp_mavlink::Message;
use mp_mavlink_dialects::all::{
    Attitude, CameraFeedback, GlobalPositionInt, GpsRawInt, Rangefinder,
};

use crate::dataflash::{DfBuffer, Item, parse_single, yielded};
use crate::location::{Location, OrderedMap, PictureInformation};
use crate::photos::{self, PhotoTimes};
use crate::time::{DateTime, GPS_EPOCH_TICKS, Kind, OutOfRange, TICKS_PER_MILLISECOND};
use crate::tlog;

/// `MathHelper.rad2deg`. `// C#: ExtLibs/Utilities/Math.cs:10`
pub const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;
/// `MathHelper.deg2rad`. `// C#: ExtLibs/Utilities/Math.cs:11`
pub const DEG2RAD: f64 = 1.0 / RAD2DEG;

/// The three ways of matching photos. `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:27-32`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessingMode {
    /// `TIME_OFFSET`: each photo's EXIF time less an offset, looked up among the GPS positions.
    TimeOffset,
    /// `CAM_MSG`: the n-th photo by time is the n-th `CAM` message.
    CamMsg,
    /// `TRIG`: the n-th photo by time is the n-th `TRIG` message.
    Trig,
}

/// What stops a run where the C# throws out of it. Each displays as the first line of the C#'s
/// `ex.ToString()` - the exception's type and message, as mono words it - which is what the
/// form prints after "Error " (the stack trace that follows is the runtime's own).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum GeorefError {
    /// The log or the folder could not be read at all.
    #[error("System.IO.IOException: {path}: {message}")]
    Io {
        /// The file.
        path: String,
        /// The system's reason.
        message: String,
    },
    /// A value the C# parses without a `try` would not parse, or an index it reads is not
    /// there: `FormatException`, `IndexOutOfRangeException`, `OverflowException`.
    #[error("{0}")]
    BadLog(String),
    /// A time outside `DateTime`'s range: `ArgumentOutOfRangeException`.
    #[error("System.ArgumentOutOfRangeException: {0}")]
    Time(#[from] OutOfRange),
    /// `readCAMMsgInLog` found no `CAM` message: `PrevNowNext` over nothing yields one triple
    /// whose middle is `null`, and `.Value.Time` of it throws (`GeoRefImageBase.cs:781-784`).
    #[error("System.NullReferenceException: Object reference not set to an instance of an object")]
    NoCamMessages,
    /// What `Parallel.ForEach` wraps an exception in: `doworkGPSOFFSET`'s matching threw.
    #[error("System.AggregateException: One or more errors occurred. ({0})")]
    Aggregate(String),
    /// `EstimateOffset` indexed a photo or a log time that is not there.
    #[error("System.IndexOutOfRangeException: Index was outside the bounds of the array.")]
    TooFew,
}

/// `AppendText`: the lines the form's output box shows.
pub type Output<'a> = &'a mut dyn FnMut(&str);

/// `GeoRefImageBase`'s state, kept across runs as the form keeps its one instance.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:34-47`
#[derive(Debug, Clone)]
pub struct GeoRefImageBase {
    /// `picturesInfo`: `None` is the C#'s `null`, what a run that matched nothing leaves.
    pub pictures_info: Option<OrderedMap<String, PictureInformation>>,
    /// `vehicleLocations`: the GPS (or `CAM`, or `TRIG`) positions, by Unix milliseconds. Read
    /// only when empty - the form clears it when the log or the GPS choice changes.
    pub vehicle_locations: OrderedMap<i64, Location>,
    /// `camLocations`: the `CAM` messages of the last run.
    pub cam_locations: OrderedMap<i64, Location>,
    /// `useAMSLAlt`.
    pub use_amsl_alt: bool,
    /// `millisShutterLag`.
    pub millis_shutter_lag: i32,
    /// `minshutter`: two `CAM`s closer than this many seconds are one too many.
    pub minshutter: f64,
    /// `filedatecache` and what "today" is for a date nothing parses.
    pub photo_times: PhotoTimes,
    /// `JXL_StationIDs`.
    pub jxl_station_ids: Vec<i32>,
}

impl GeoRefImageBase {
    /// `new GeoRefImageBase()`: `minshutter` 0.5, no shutter lag, `useAMSLAlt` false (the form
    /// sets it true, `georefimage.cs:30-33`). `today` is `DateTime.Today`.
    #[must_use]
    pub fn new(today: DateTime) -> Self {
        Self {
            pictures_info: Some(OrderedMap::new()),
            vehicle_locations: OrderedMap::new(),
            cam_locations: OrderedMap::new(),
            use_amsl_alt: false,
            millis_shutter_lag: 0,
            minshutter: 0.5,
            photo_times: PhotoTimes::new(today),
            jxl_station_ids: Vec::new(),
        }
    }
}

/// `ToMilliseconds`. `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:501-505`
#[must_use]
pub fn to_milliseconds(time: DateTime) -> i64 {
    time.to_milliseconds()
}

/// `GetTimeFromGps`: the GPS epoch plus the weeks and milliseconds, less 17 leap seconds -
/// seventeen, though GPS has been 18 ahead of UTC since 2017, and `DFLog.gpsTimeToTime` (which
/// times the GPS messages these are compared with) takes today's count. So a `CAM` time is one
/// second later than the GPS time of the same instant.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:490-499`
///
/// # Errors
///
/// Where `AddDays` or `AddMilliseconds` throws.
pub fn get_time_from_gps(weeknumber: i32, milliseconds: i32) -> Result<DateTime, OutOfRange> {
    const LEAP_SECONDS: f64 = 17.0;
    let datum = DateTime::from_ticks(GPS_EPOCH_TICKS, Kind::Utc);
    // `weeknumber * 7` is int arithmetic.
    let week = datum.add_days(f64::from(weeknumber.wrapping_mul(7)))?;
    let time = week.add_milliseconds(f64::from(milliseconds))?;
    time.add_seconds(-LEAP_SECONDS)
}

/// `FromUTCTimeMilliseconds`. `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:484-488`
///
/// # Errors
///
/// Where `AddMilliseconds` throws.
pub fn from_utc_time_milliseconds(milliseconds: i64) -> Result<DateTime, OutOfRange> {
    DateTime::from_unix_millis(milliseconds)
}

/// `double.Parse`, `float.Parse` or `int.Parse` of a field that is not a number, or not there.
fn bad(what: &str) -> GeorefError {
    GeorefError::BadLog(format!(
        "System.FormatException: Input string was not in a correct format. ({what})"
    ))
}

fn parse_double(item: &Item, index: Option<usize>, what: &str) -> Result<f64, GeorefError> {
    item.field(index)
        .and_then(netfmt::parse_double)
        .ok_or_else(|| bad(what))
}

fn parse_float(item: &Item, index: Option<usize>, what: &str) -> Result<f32, GeorefError> {
    item.field(index)
        .and_then(parse_single)
        .ok_or_else(|| bad(what))
}

fn parse_int(item: &Item, index: Option<usize>, what: &str) -> Result<i32, GeorefError> {
    item.field(index)
        .and_then(netfmt::parse_i32)
        .ok_or_else(|| bad(what))
}

fn read_file(path: &str) -> Result<Vec<u8>, GeorefError> {
    std::fs::read(path).map_err(|e| GeorefError::Io {
        path: path.to_owned(),
        message: e.to_string(),
    })
}

fn is_tlog(path: &str) -> bool {
    path.to_lowercase().ends_with("tlog")
}

/// `readGPSMsgInLog`: every GPS position of the log with a 3D fix, by Unix milliseconds, each
/// with the attitude and sonar altitude last seen before it.
///
/// From a tlog: `GLOBAL_POSITION_INT` at its receive time, `ATTITUDE` in degrees, `RANGEFINDER`,
/// and `GPS_RAW_INT` only to skip - a `GPS_RAW_INT` without a fix is skipped and nothing else; the
/// positions are taken whatever the fix. The relative and AMSL altitudes are divided by `1000.0f`,
/// a `float`, and the GPS altitude is never set.
///
/// From a dataflash log: `GPS` (or `GPS2`) lines with `Status` 3 or more, the first of a
/// millisecond kept, with `ATT`'s `Roll`, `Pitch`, `Yaw` and the sonar altitude from `CTUN`'s
/// `SAlt` or `RFND`'s `Dist1`. A bad GPS line is skipped; a bad `ATT`, `CTUN` or `RFND` value ends
/// the read, as it throws out of the C#'s loop.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:141-301`
///
/// # Errors
///
/// Where the C# throws out of it.
pub fn read_gps_msg_in_log(
    path: &str,
    gpstouse: &str,
) -> Result<OrderedMap<i64, Location>, GeorefError> {
    let data = read_file(path)?;
    let mut list = OrderedMap::new();
    if is_tlog(path) {
        let packets = tlog::messages_of_type(
            &data,
            &[
                GlobalPositionInt::ID,
                GpsRawInt::ID,
                Rangefinder::ID,
                Attitude::ID,
            ],
        )
        .map_err(|e| GeorefError::BadLog(e.to_string()))?;
        let mut location = Location::default();
        for packet in packets {
            match packet.msgid {
                GpsRawInt::ID if GpsRawInt::decode(&packet.payload).fix_type <= 2 => continue,
                Rangefinder::ID => {
                    location.s_alt = f64::from(Rangefinder::decode(&packet.payload).distance);
                }
                Attitude::ID => {
                    let att = Attitude::decode(&packet.payload);
                    location.roll = degrees_single(att.roll);
                    location.pitch = degrees_single(att.pitch);
                    location.yaw = degrees_single(att.yaw);
                }
                GlobalPositionInt::ID => {
                    let pos = GlobalPositionInt::decode(&packet.payload);
                    location.time = packet.rxtime;
                    location.lat = f64::from(pos.lat) / 1e7;
                    location.lon = f64::from(pos.lon) / 1e7;
                    location.rel_alt = f64::from(single_of_int(pos.relative_alt) / 1000.0_f32);
                    location.alt_amsl = f64::from(single_of_int(pos.alt) / 1000.0_f32);
                    list.set(to_milliseconds(location.time), location);
                }
                _ => {}
            }
        }
        return Ok(list);
    }

    let mut sr = DfBuffer::new(&data).map_err(|e| GeorefError::BadLog(e.to_string()))?;
    let types = ["GPS", "GPS2", "ATT", "CTUN", "RFND"];
    let mut current_yaw = 0f32;
    let mut current_roll = 0f32;
    let mut current_pitch = 0f32;
    let mut current_salt = 0f32;
    for line in sr.lines_of(&types) {
        let item = sr.item(line);
        let msgtype = item.msgtype().to_owned();
        if !yielded(&types, &msgtype) {
            continue;
        }
        if msgtype == gpstouse {
            if !sr.dflog.contains(gpstouse) {
                continue;
            }
            let latindex = sr.dflog.find_message_offset(gpstouse, "Lat");
            let lngindex = sr.dflog.find_message_offset(gpstouse, "Lng");
            let altindex = sr.dflog.find_message_offset(gpstouse, "Alt");
            let statusindex = sr.dflog.find_message_offset(gpstouse, "Status");
            let raltindex = sr
                .dflog
                .find_message_offset(gpstouse, "RAlt")
                .or_else(|| sr.dflog.find_message_offset(gpstouse, "RelAlt"));
            // Everything in the C#'s try: any failure is "Bad GPS Line" and the line is skipped.
            let parsed = (|| -> Result<Option<Location>, GeorefError> {
                let mut location = Location {
                    time: sr.time(&item)?,
                    ..Location::default()
                };
                if statusindex.is_some() && parse_double(&item, statusindex, "Status")? < 3.0 {
                    return Ok(None);
                }
                if latindex.is_some() {
                    location.lat = parse_double(&item, latindex, "Lat")?;
                }
                if lngindex.is_some() {
                    location.lon = parse_double(&item, lngindex, "Lng")?;
                }
                if raltindex.is_some() {
                    location.rel_alt = parse_double(&item, raltindex, "RelAlt")?;
                }
                if altindex.is_some() {
                    location.alt_amsl = parse_double(&item, altindex, "Alt")?;
                    location.gps_alt = parse_double(&item, altindex, "Alt")?;
                }
                location.roll = current_roll;
                location.pitch = current_pitch;
                location.yaw = current_yaw;
                location.s_alt = f64::from(current_salt);
                Ok(Some(location))
            })();
            if let Ok(Some(location)) = parsed {
                let millis = to_milliseconds(location.time);
                if !list.contains_key(&millis) && location.time != DateTime::MIN {
                    list.set(millis, location);
                }
            }
        } else if msgtype == "ATT" {
            let r = sr.dflog.find_message_offset("ATT", "Roll");
            let p = sr.dflog.find_message_offset("ATT", "Pitch");
            let y = sr.dflog.find_message_offset("ATT", "Yaw");
            if r.is_some() {
                current_roll = parse_float(&item, r, "ATT Roll")?;
            }
            if p.is_some() {
                current_pitch = parse_float(&item, p, "ATT Pitch")?;
            }
            if y.is_some() {
                current_yaw = parse_float(&item, y, "ATT Yaw")?;
            }
        } else if msgtype == "CTUN" {
            let s = sr.dflog.find_message_offset("CTUN", "SAlt");
            if s.is_some() {
                current_salt = parse_float(&item, s, "CTUN SAlt")?;
            }
        } else if msgtype == "RFND" {
            let s = sr.dflog.find_message_offset("RFND", "Dist1");
            if s.is_some() {
                current_salt = parse_float(&item, s, "RFND Dist1")?;
            }
        }
    }
    Ok(list)
}

/// `(float)degrees(radians)`: widened, multiplied by `rad2deg`, narrowed.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:176-178, 1061-1064`
#[allow(clippy::cast_possible_truncation)]
fn degrees_single(radians: f32) -> f32 {
    (f64::from(radians) * RAD2DEG) as f32
}

/// An `int` in `float` arithmetic, as `pos.Value.relative_alt / 1000.0f` converts it.
#[allow(clippy::cast_precision_loss)]
const fn single_of_int(value: i32) -> f32 {
    value as f32
}

/// `readCAMMsgInLog`: every camera trigger of the log, by Unix milliseconds - a later one of the
/// same millisecond replacing the earlier in its place.
///
/// From a tlog, `CAMERA_FEEDBACK` at its own `time_usec`, with the last `RANGEFINDER` distance;
/// from a dataflash log, `CAM` at its GPS week and time (less 17 s, see [`get_time_from_gps`]),
/// with the roll, pitch and yaw under either of their names and the last `RFND`'s `Dist1`. Nothing
/// is caught: one bad `CAM` line ends the read.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:308-419`
///
/// # Errors
///
/// Where the C# throws out of it.
pub fn read_cam_msg_in_log(path: &str) -> Result<OrderedMap<i64, Location>, GeorefError> {
    let data = read_file(path)?;
    let mut list = OrderedMap::new();
    if is_tlog(path) {
        let packets = tlog::messages_of_type(&data, &[Rangefinder::ID, CameraFeedback::ID])
            .map_err(|e| GeorefError::BadLog(e.to_string()))?;
        let mut location = Location::default();
        for packet in packets {
            if packet.msgid == Rangefinder::ID {
                location.s_alt = f64::from(Rangefinder::decode(&packet.payload).distance);
            }
            if packet.msgid == CameraFeedback::ID {
                let msg = CameraFeedback::decode(&packet.payload);
                // `(long)(msg.time_usec / 1000)`: a ulong division, then a reinterpretation.
                #[allow(clippy::cast_possible_wrap)]
                let millis = (msg.time_usec / 1000) as i64;
                location.time = from_utc_time_milliseconds(millis)?;
                location.lat = f64::from(msg.lat) / 1e7;
                location.lon = f64::from(msg.lng) / 1e7;
                location.rel_alt = f64::from(msg.alt_rel);
                location.alt_amsl = f64::from(msg.alt_msl);
                location.roll = msg.roll;
                location.pitch = msg.pitch;
                location.yaw = msg.yaw;
                list.set(to_milliseconds(location.time), location);
            }
        }
        return Ok(list);
    }
    read_trigger_lines(&data, "CAM", &["Roll", "R"], &["Pitch", "P"], &["Yaw", "Y"])
}

/// `readTRIGMsgInLog`: `TRIG` lines, as `readCAMMsgInLog` reads `CAM` ones but preferring the
/// short names `R`, `P`, `Y`, and always through `DFLogBuffer` - a tlog is read as a text log and
/// has none.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:421-482`
///
/// # Errors
///
/// Where the C# throws out of it.
pub fn read_trig_msg_in_log(path: &str) -> Result<OrderedMap<i64, Location>, GeorefError> {
    let data = read_file(path)?;
    read_trigger_lines(
        &data,
        "TRIG",
        &["R", "Roll"],
        &["P", "Pitch"],
        &["Y", "Yaw"],
    )
}

/// The body both dataflash readers share: each `name` line through `GetTimeFromGps` and
/// `double.Parse`, the attitude by the first of each pair of column names the format has.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:361-415, 428-478`
fn read_trigger_lines(
    data: &[u8],
    name: &str,
    roll: &[&str; 2],
    pitch: &[&str; 2],
    yaw: &[&str; 2],
) -> Result<OrderedMap<i64, Location>, GeorefError> {
    let mut list = OrderedMap::new();
    let mut sr = DfBuffer::new(data).map_err(|e| GeorefError::BadLog(e.to_string()))?;
    let types = [name, "RFND"];
    let mut current_salt = 0f32;
    for line in sr.lines_of(&types) {
        let item = sr.item(line);
        let msgtype = item.msgtype().to_owned();
        if !yielded(&types, &msgtype) {
            continue;
        }
        if msgtype == name {
            let mut find = |column: &str| sr.dflog.find_message_offset(name, column);
            let latindex = find("Lat");
            let lngindex = find("Lng");
            let altindex = find("Alt");
            let raltindex = find("RelAlt");
            let galtindex = find("GPSAlt");
            let rindex = find(roll[0]).or_else(|| find(roll[1]));
            let pindex = find(pitch[0]).or_else(|| find(pitch[1]));
            let yindex = find(yaw[0]).or_else(|| find(yaw[1]));
            let gtimeindex = find("GPSTime");
            let gweekindex = find("GPSWeek");

            let mut p = Location {
                time: get_time_from_gps(
                    parse_int(&item, gweekindex, "GPSWeek")?,
                    parse_int(&item, gtimeindex, "GPSTime")?,
                )?,
                ..Location::default()
            };
            p.lat = parse_double(&item, latindex, "Lat")?;
            p.lon = parse_double(&item, lngindex, "Lng")?;
            p.alt_amsl = parse_double(&item, altindex, "Alt")?;
            if raltindex.is_some() {
                p.rel_alt = parse_double(&item, raltindex, "RelAlt")?;
            }
            if galtindex.is_some() {
                p.gps_alt = parse_double(&item, galtindex, "GPSAlt")?;
            }
            p.pitch = parse_float(&item, pindex, "Pitch")?;
            p.roll = parse_float(&item, rindex, "Roll")?;
            p.yaw = parse_float(&item, yindex, "Yaw")?;
            p.s_alt = f64::from(current_salt);
            list.set(to_milliseconds(p.time), p);
        } else if msgtype == "RFND" {
            let s = sr.dflog.find_message_offset("RFND", "Dist1");
            if s.is_some() {
                current_salt = parse_float(&item, s, "RFND Dist1")?;
            }
        }
    }
    Ok(list)
}

/// `LookForLocation`: the entry at `t`'s millisecond, else the nearest within
/// `offsettime / 2 - 1` milliseconds either way - later before earlier at the same distance.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:576-612`
#[must_use]
pub fn look_for_location(
    t: DateTime,
    list: &OrderedMap<i64, Location>,
    offsettime: i32,
) -> Option<Location> {
    let time = to_milliseconds(t);
    let max_iteration = i64::from(offsettime / 2);
    let mut iteration = 0i64;
    while iteration < max_iteration {
        if let Some(found) = list.get(&(time + iteration)) {
            return Some(*found);
        }
        if let Some(found) = list.get(&(time - iteration)) {
            return Some(*found);
        }
        iteration += 1;
    }
    None
}

/// .NET's default `ToString()` of a `double`, as string concatenation writes it.
fn text(value: f64) -> String {
    netfmt::double(value)
}

/// `bool.ToString()`.
const fn text_bool(value: bool) -> &'static str {
    if value { "True" } else { "False" }
}

/// `Take(count - dropend).Skip(dropstart)`, then `ToDictionary`.
fn take_skip(
    map: &OrderedMap<i64, Location>,
    dropstart: i32,
    dropend: i32,
) -> OrderedMap<i64, Location> {
    let count = i64::try_from(map.len()).unwrap_or(i64::MAX);
    let take = usize::try_from((count - i64::from(dropend)).max(0)).unwrap_or(0);
    let skip = usize::try_from(dropstart.max(0)).unwrap_or(0);
    map.iter()
        .take(take)
        .skip(skip)
        .map(|(k, v)| (*k, *v))
        .collect()
}

impl GeoRefImageBase {
    /// The photos `doworkGPSOFFSET` and `EstimateOffset` list, sorted by time.
    fn sorted_photos(&mut self, dir: &str) -> Result<Vec<String>, GeorefError> {
        let mut files = photos::list_photos(dir).map_err(|e| GeorefError::Io {
            path: dir.to_owned(),
            message: e.to_string(),
        })?;
        self.photo_times.sort(&mut files);
        Ok(files)
    }

    /// The positions `doworkGPSOFFSET` and `EstimateOffset` read when they have none: the `CAM`
    /// messages always, into `camLocations`, and then those or the GPS positions.
    fn read_locations_if_empty(
        &mut self,
        log_file: &str,
        gps: &str,
        usecam: bool,
        mut out: Option<&mut dyn FnMut(&str)>,
    ) -> Result<(), GeorefError> {
        if !self.vehicle_locations.is_empty() {
            return Ok(());
        }
        if let Some(out) = out.as_mut() {
            out("Reading log for CAM Messages\n");
        }
        self.cam_locations = read_cam_msg_in_log(log_file)?;
        if usecam {
            self.vehicle_locations = self.cam_locations.clone();
        } else {
            if let Some(out) = out.as_mut() {
                out("Reading log for GPS-ATT Messages\n");
            }
            self.vehicle_locations = read_gps_msg_in_log(log_file, gps)?;
        }
        Ok(())
    }

    /// `EstimateOffset`: the photo times less the log times of photos and log entries 1-4 and the
    /// last three by index - photo *n* against log entry *n* - averaged by halving; `-1` with no
    /// positions or no photos.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:507-574`
    ///
    /// # Errors
    ///
    /// Where the C# throws: fewer than four photos, or fewer log entries than photos, index past an
    /// array - an exception the form's button does not catch.
    pub fn estimate_offset(
        &mut self,
        log_file: &str,
        dir_with_images: &str,
        use_gps_or_gps2: &str,
        usecam: bool,
        append_text: Output<'_>,
    ) -> Result<f64, GeorefError> {
        self.read_locations_if_empty(log_file, use_gps_or_gps2, usecam, None)?;
        if self.vehicle_locations.is_empty() {
            return Ok(-1.0);
        }
        let files = self.sorted_photos(dir_with_images)?;
        if files.is_empty() {
            return Ok(-1.0);
        }
        let n = i64::try_from(files.len()).unwrap_or(i64::MAX);
        let mut ans = 0.0f64;
        let mut times: Vec<i64> = self.vehicle_locations.keys().copied().collect();
        times.sort_unstable();
        for a in [0, 1, 2, 3, n - 3, n - 2, n - 1] {
            let index = usize::try_from(a).map_err(|_| GeorefError::TooFew)?;
            let first_photo = files.get(index).ok_or(GeorefError::TooFew)?.clone();
            let photo_time = self.photo_times.get(&first_photo);
            append_text(&format!(
                "{} Picture {} with DateTime: {}\n",
                a + 1,
                photos::file_stem(&first_photo),
                photo_time.format_exif()
            ));
            let first_time = *times.get(index).ok_or(GeorefError::TooFew)?;
            let log_time = from_utc_time_milliseconds(first_time)?;
            append_text(&format!(
                "{} GPS Log Msg: {}\n",
                a + 1,
                log_time.format_exif()
            ));
            let est = photo_time.minus(log_time).total_seconds();
            append_text(&format!("{} Est: {}\n", a + 1, text(est)));
            if ans == 0.0 {
                ans = est;
            } else {
                ans = ans * 0.5 + est * 0.5;
            }
        }
        Ok(ans)
    }

    /// `doworkGPSOFFSET`: each photo's EXIF time less `offset` seconds, looked up among the
    /// positions within 2.5 s. Also writes the positions beside the log as `<log>.xml`.
    ///
    /// The C# matches the photos in a `Parallel.ForEach`, so the order of the result - and of the
    /// lines it prints - is whatever the threads make it; here it is the photos' sorted order,
    /// which is one of those. The GPS altitude is not copied to the photo.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:614-729`
    ///
    /// # Errors
    ///
    /// Where the C# throws: an unreadable log, or a photo time the offset takes out of range.
    /// `Ok(None)` is its `null`: no photos.
    pub fn dowork_gps_offset(
        &mut self,
        log_file: &str,
        dir_with_images: &str,
        offset: f32,
        use_gps_or_gps2: &str,
        usecam: bool,
        append_text: Output<'_>,
    ) -> Result<Option<OrderedMap<String, PictureInformation>>, GeorefError> {
        let mut temp = OrderedMap::new();
        self.read_locations_if_empty(log_file, use_gps_or_gps2, usecam, Some(&mut *append_text))?;

        // The XmlSerializer's try swallows any failure (GeoRefImageBase.cs:644-653).
        let _ =
            crate::report::write_locations_xml(&format!("{log_file}.xml"), &self.vehicle_locations);

        append_text(&format!(
            "Log locations : {}\n",
            self.vehicle_locations.len()
        ));
        append_text("Read images\n");
        let files = self.sorted_photos(dir_with_images)?;
        append_text(&format!("Images read : {}\n", files.len()));
        if files.is_empty() {
            append_text("Not enought files found.  Aborting..... \n");
            return Ok(None);
        }
        for filename in &files {
            let shot = self.photo_times.get(filename);
            // Inside the C#'s Parallel.ForEach, which rethrows it wrapped.
            let corrected = shot
                .add_seconds(f64::from(-offset))
                .map_err(|e| GeorefError::Aggregate(e.to_string()))?;
            let found = look_for_location(corrected, &self.vehicle_locations, 5000);
            let Some(shot_location) = found else {
                append_text(&format!(
                    "Photo {} NOT PROCESSED. No GPS match in the log file. Please take care\n",
                    photos::file_stem(filename)
                ));
                continue;
            };
            let p = PictureInformation {
                location: Location {
                    time: shot_location.time,
                    lat: shot_location.lat,
                    lon: shot_location.lon,
                    alt_amsl: shot_location.alt_amsl,
                    rel_alt: shot_location.rel_alt,
                    gps_alt: 0.0,
                    s_alt: shot_location.s_alt,
                    roll: shot_location.roll,
                    pitch: shot_location.pitch,
                    yaw: shot_location.yaw,
                },
                path: filename.clone(),
                shot_time_reported_by_camera: shot,
                ..PictureInformation::default()
            };
            temp.set(filename.clone(), p);
            append_text(&format!(
                "Photo {} PROCESSED with GPS position found {} ms away\n",
                photos::file_stem(filename),
                text(shot_location.time.minus(corrected).total_milliseconds())
            ));
        }
        Ok(Some(temp))
    }

    /// `doworkCAM`: the n-th photo by time is the n-th `CAM` message left after dropping
    /// `dropstart` from the front and `dropend` from the back and every one taken less than
    /// `minshutter` seconds before the next. With a shutter lag the position is the GPS one
    /// `millisShutterLag` after the `CAM` - unless that is more than twice the lag away; with
    /// `useAMSLAlt` the altitude is the GPS one nearest the `CAM`.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:739-966`
    ///
    /// # Errors
    ///
    /// Where the C# throws. `Ok(None)` is its `null`: AMSL wanted and no GPS positions.
    #[allow(clippy::too_many_lines)]
    pub fn dowork_cam(
        &mut self,
        log_file: &str,
        dir_with_images: &str,
        use_gps_or_gps2: &str,
        append_text: Output<'_>,
        dropstart: i32,
        dropend: i32,
    ) -> Result<Option<OrderedMap<String, PictureInformation>>, GeorefError> {
        let mut temp = OrderedMap::new();
        append_text(&format!(
            "Using AMSL Altitude {}\n",
            text_bool(self.use_amsl_alt)
        ));
        if self.use_amsl_alt || self.millis_shutter_lag > 0 {
            append_text("Reading log for GPS Messages in order to get AMSL Altitude\n");
            if self.vehicle_locations.is_empty() {
                self.vehicle_locations = read_gps_msg_in_log(log_file, use_gps_or_gps2)?;
                if self.vehicle_locations.is_empty() {
                    append_text("Log file problem. Aborting....\n");
                    return Ok(None);
                }
            }
            append_text("Log Read for GPS Messages\n");
            append_text(&format!(
                "Log locations : {}\n",
                self.vehicle_locations.len()
            ));
        }
        append_text("Reading log for CAM Messages\n");
        self.cam_locations = read_cam_msg_in_log(log_file)?;
        append_text(&format!(
            "Log Read with - {} - CAM Messages found\n",
            self.cam_locations.len()
        ));
        self.cam_locations = take_skip(&self.cam_locations, dropstart, dropend);

        // `PrevNowNext` then (next.Time - now.Time, now): each CAM against the one after it, the
        // last against `DateTime.MinValue`.
        if self.cam_locations.is_empty() {
            return Err(GeorefError::NoCamMessages);
        }
        let entries: Vec<(i64, DateTime)> = self
            .cam_locations
            .iter()
            .map(|(k, v)| (*k, v.time))
            .collect();
        let mut deltas = Vec::new();
        for (at, (key, time)) in entries.iter().enumerate() {
            let next = entries.get(at + 1).map_or(DateTime::MIN, |(_, t)| *t);
            deltas.push((next.minus(*time).total_seconds(), *key));
        }
        for (seconds, key) in deltas {
            if seconds > 0.0 && seconds < self.minshutter {
                append_text(&format!(
                    "Possible Shutter speed issue - {}s\n",
                    text(seconds)
                ));
                self.cam_locations.remove(&key);
            }
        }
        append_text(&format!(
            "Filtered - {} - CAM Messages found\n",
            self.cam_locations.len()
        ));
        append_text("Read images\n");
        let mut files = photos::list_photos(dir_with_images).map_err(|e| GeorefError::Io {
            path: dir_with_images.to_owned(),
            message: e.to_string(),
        })?;
        append_text(&format!("Images read : {}\n", files.len()));
        if files.len() != self.cam_locations.len() {
            append_text(&format!(
                "CAM Msgs and Files discrepancy. Check it! files: {} vs CAM msg: {}\n",
                files.len(),
                self.cam_locations.len()
            ));
        }
        self.photo_times.sort(&mut files);

        let lag = self.millis_shutter_lag;
        let cams: Vec<Location> = self.cam_locations.values().copied().collect();
        for (i, current_cam) in cams.iter().enumerate() {
            let Some(file) = files.get(i).cloned() else {
                continue;
            };
            let shot = self.photo_times.get(&file);
            let cam_time = current_cam.time;
            let name = photos::file_stem(&file);
            let mut p = PictureInformation {
                shot_time_reported_by_camera: shot,
                path: file.clone(),
                ..PictureInformation::default()
            };
            if lag == 0 {
                p.location = Location {
                    time: cam_time,
                    lat: current_cam.lat,
                    lon: current_cam.lon,
                    alt_amsl: current_cam.alt_amsl,
                    rel_alt: current_cam.rel_alt,
                    gps_alt: current_cam.gps_alt,
                    ..Location::default()
                };
                let mut log_alt_msg = "RelAlt".to_owned();
                if self.use_amsl_alt {
                    match look_for_location(p.location.time, &self.vehicle_locations, 2000) {
                        Some(gps) => {
                            log_alt_msg = format!(
                                "AMSL Alt {} ms away offset: {}",
                                text(gps.time.minus(p.location.time).total_milliseconds()),
                                text(shot.minus(cam_time).total_seconds())
                            );
                            p.location.alt_amsl = gps.alt_amsl;
                        }
                        None => log_alt_msg = "AMSL Alt NOT found".to_owned(),
                    }
                }
                p.location.pitch = current_cam.pitch;
                p.location.roll = current_cam.roll;
                p.location.yaw = current_cam.yaw;
                p.location.s_alt = current_cam.s_alt;
                temp.set(file.clone(), p);
                append_text(&format!(
                    "Photo {name} processed from CAM Msg with {lag} ms shutter lag. {log_alt_msg}\n"
                ));
            } else {
                let corrected = cam_time.add_milliseconds(f64::from(lag))?;
                let Some(gps) = look_for_location(corrected, &self.vehicle_locations, 2000) else {
                    append_text(&format!(
                        "Photo {name} NOT Processed. Time not found in log. Too large Shutter Lag? Try setting it to 0\n"
                    ));
                    continue;
                };
                let diff = gps.time.minus(cam_time);
                if diff.total_milliseconds() > 2.0 * f64::from(lag) {
                    p.location = Location {
                        time: cam_time,
                        lat: current_cam.lat,
                        lon: current_cam.lon,
                        alt_amsl: current_cam.alt_amsl,
                        rel_alt: current_cam.rel_alt,
                        gps_alt: current_cam.gps_alt,
                        ..Location::default()
                    };
                    let mut log_alt_msg = "RelAlt".to_owned();
                    if self.use_amsl_alt {
                        match look_for_location(p.location.time, &self.vehicle_locations, 2000) {
                            Some(near) => {
                                log_alt_msg = format!(
                                    "AMSL Alt {} ms away",
                                    text(near.time.minus(p.location.time).total_milliseconds())
                                );
                                p.location.alt_amsl = near.alt_amsl;
                            }
                            None => log_alt_msg = "AMSL Alt NOT found".to_owned(),
                        }
                    }
                    append_text(&format!(
                        "Photo {name} processed with CAM Msg. Shutter lag too small. {log_alt_msg}\n"
                    ));
                } else {
                    p.location = Location {
                        time: gps.time,
                        lat: gps.lat,
                        lon: gps.lon,
                        alt_amsl: gps.alt_amsl,
                        rel_alt: gps.rel_alt,
                        gps_alt: gps.gps_alt,
                        ..Location::default()
                    };
                    let log_alt_msg = if self.use_amsl_alt {
                        "AMSL Alt"
                    } else {
                        "RelAlt"
                    };
                    append_text(&format!(
                        "Photo {name} processed with GPS Msg : {} ms ahead of CAM Msg. {log_alt_msg}\n",
                        text(diff.total_milliseconds())
                    ));
                }
                p.location.pitch = current_cam.pitch;
                p.location.roll = current_cam.roll;
                p.location.yaw = current_cam.yaw;
                p.location.s_alt = current_cam.s_alt;
                temp.set(file.clone(), p);
            }
        }
        Ok(Some(temp))
    }

    /// `doworkTRIG`: the n-th photo by time is the n-th `TRIG` message, which also become the
    /// vehicle's positions; nothing at all unless there are as many photos as messages.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:968-1054`
    ///
    /// # Errors
    ///
    /// Where the C# throws. `Ok(None)` is its `null`: a count mismatch.
    pub fn dowork_trig(
        &mut self,
        log_file: &str,
        dir_with_images: &str,
        _use_gps_or_gps2: &str,
        append_text: Output<'_>,
        dropstart: i32,
        dropend: i32,
    ) -> Result<Option<OrderedMap<String, PictureInformation>>, GeorefError> {
        let mut temp = OrderedMap::new();
        append_text(&format!(
            "Using AMSL Altitude {}\n",
            text_bool(self.use_amsl_alt)
        ));
        append_text("Reading log for TRIG Messages\n");
        self.vehicle_locations = read_trig_msg_in_log(log_file)?;
        append_text(&format!(
            "Log Read with - {} - TRIG Messages found\n",
            self.vehicle_locations.len()
        ));
        self.vehicle_locations = take_skip(&self.vehicle_locations, dropstart, dropend);
        append_text(&format!(
            "Filtered - {} - TRIG Messages found\n",
            self.vehicle_locations.len()
        ));
        append_text("Read images\n");
        let mut files = photos::list_photos(dir_with_images).map_err(|e| GeorefError::Io {
            path: dir_with_images.to_owned(),
            message: e.to_string(),
        })?;
        append_text(&format!("Images read : {}\n", files.len()));
        if files.len() != self.vehicle_locations.len() {
            append_text(&format!(
                "TRIG Msgs and Files discrepancy. Check it! files: {} vs TRIG msg: {}\n",
                files.len(),
                self.vehicle_locations.len()
            ));
            return Ok(None);
        }
        self.photo_times.sort(&mut files);
        let trigs: Vec<Location> = self.vehicle_locations.values().copied().collect();
        for (i, current) in trigs.iter().enumerate() {
            let Some(file) = files.get(i).cloned() else {
                continue;
            };
            let shot = self.photo_times.get(&file);
            let p = PictureInformation {
                location: Location {
                    time: current.time,
                    lat: current.lat,
                    lon: current.lon,
                    alt_amsl: current.alt_amsl,
                    rel_alt: current.rel_alt,
                    gps_alt: current.gps_alt,
                    s_alt: current.s_alt,
                    roll: current.roll,
                    pitch: current.pitch,
                    yaw: current.yaw,
                },
                path: file.clone(),
                shot_time_reported_by_camera: shot,
                ..PictureInformation::default()
            };
            temp.set(file.clone(), p);
            append_text(&format!(
                "Photo {} processed from CAM Msg with {} ms shutter lag. \n",
                photos::file_stem(&file),
                self.millis_shutter_lag
            ));
        }
        Ok(Some(temp))
    }
}

/// The ticks of a whole millisecond: what `ToMilliseconds` of a time built from one gives back.
#[must_use]
pub const fn millis_to_ticks(millis: i64) -> i64 {
    millis * TICKS_PER_MILLISECOND
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cam_times_take_seventeen_leap_seconds() {
        // CAM 1 of testdata/georef/camera.bin: week 2437, 351022600 ms.
        let t = get_time_from_gps(2437, 351_022_600).unwrap();
        assert_eq!(t.format_exif(), "2026:09:24 01:30:05");
        assert_eq!(t.to_milliseconds() % 1000, 600);
    }

    #[test]
    fn look_for_location_prefers_later_at_equal_distance() {
        let mut list = OrderedMap::new();
        let at = |ms: i64| Location {
            lat: ms as f64,
            ..Location::default()
        };
        list.set(1_000_100, at(1_000_100));
        list.set(999_900, at(999_900));
        let t = DateTime::from_unix_millis(1_000_000).unwrap();
        assert_eq!(look_for_location(t, &list, 2000).unwrap().lat, 1_000_100.0);
        // 2000 looks 999 ms either way, 200 only 99.
        assert!(look_for_location(t, &list, 200).is_none());
        assert!(look_for_location(t, &list, 202).is_some());
    }

    #[test]
    fn take_then_skip() {
        let map: OrderedMap<i64, Location> = (0..5).map(|k| (k, Location::default())).collect();
        let kept = take_skip(&map, 1, 2);
        assert_eq!(kept.keys().copied().collect::<Vec<_>>(), [1, 2]);
        assert!(take_skip(&map, 0, 9).is_empty());
    }
}
