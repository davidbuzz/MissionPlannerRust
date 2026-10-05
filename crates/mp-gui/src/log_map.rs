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

//! `LogMap`: a picture of where a log flew, `<log>.jpg` beside it - the temp form's map logs, and
//! the pictures LogIndex shows.
//!
//! A telemetry log's `GLOBAL_POSITION_INT`s, a track for each vehicle, or a dataflash log's `GPS`
//! lines with a 3D fix; more than ten points on the first track, and the tracks are drawn over
//! Google's imagery of the area (`GoogleSatelliteMap`, at zoom 16 or out until the picture is no
//! wider than 2000 pixels), each in a colour of its own, "SITL" written in the corner when a
//! simulator flew it; fewer, and the picture is 100 by 100 and says "No gps data". A log that
//! already has its picture is left alone.
//!
//! As the C# draws it: the tracks are placed by a straight scale from the area's corners to the
//! picture's, padding included, not by the projection the tiles are drawn with; the eighth colour,
//! pink, is never reached (`colours[a % (colours.Length - 1)]`); a point at the picture's corner
//! starts a new line (`PointF.IsEmpty`); lines are one pixel and not smoothed, as GDI+ draws them
//! by default; the picture is a JPEG at quality 100.
//!
//! Where this is not the C#: the logs one after another where it runs `Parallel.ForEach`; the
//! text in the font the system gives for Microsoft Sans Serif (`SystemFonts.DefaultFont`), drawn
//! from its outlines at its top left without GDI+'s padding - in a web page, IBM Plex Sans, the
//! page's font; a log of "Mavlink 0.9", which the C#'s reader throws for and its `catch` turns into
//! a picture saying "Old log", is one this reader skips, so never said; in a web page the picture
//! keeps its own time, where the C# gives it the log's (the page's store has none to set).
//! `// C#: ExtLibs/Utilities/LogMap.cs:1-280`

#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};

use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_os::fs::FsExt as _;
use mp_units::tiles::TileId;
use tiny_skia::{Color, Paint, PathBuilder, Pixmap, PixmapPaint, Stroke, Transform};

/// `<log>.jpg`.
#[must_use]
pub fn picture_of(log: &Path) -> PathBuf {
    let mut name = log.as_os_str().to_owned();
    name.push(".jpg");
    PathBuf::from(name)
}

/// `colours`: `Color.Red`, `Orange`, `Yellow`, `Green`, `Blue`, `Indigo`, `Violet` and `Pink`.
/// `// C#: LogMap.cs:136-140`
const COLOURS: [(u8, u8, u8); 8] = [
    (255, 0, 0),
    (255, 165, 0),
    (255, 255, 0),
    (0, 128, 0),
    (0, 0, 255),
    (75, 0, 130),
    (238, 130, 238),
    (255, 192, 203),
];
/// The messages that mark a simulator: `SIM_STATE` and `SIMSTATE`. `// C#: LogMap.cs:56-60`
const SIM_STATE: u32 = 108;
const SIMSTATE: u32 = 164;
const GLOBAL_POSITION_INT: u32 = 33;
/// A track drawn needs more points than this. `// C#: LogMap.cs:124`
const FEWEST_POINTS: usize = 10;
/// The edge added around the tracks, degrees. `// C#: LogMap.cs:127`
const MARGIN: f64 = 0.001;
/// `GetMap`'s first zoom, and the widest picture it allows before zooming out.
const ZOOM: i32 = 16;
const WIDEST: i64 = 2000;
/// The pixels around the tiles, and their size.
const PADDING: i64 = 10;
const TILE: i64 = 256;
/// `DoTextMap`'s picture. `// C#: LogMap.cs:182`
const TEXT_PICTURE: u32 = 100;
const NO_GPS_DATA: &str = "No gps data";
const SITL: &str = "SITL";
/// `SystemFonts.DefaultFont`: Microsoft Sans Serif, 8.25 points - 11 pixels at 96 to the inch.
const FONT_FAMILY: &str = "Microsoft Sans Serif";
const FONT_PIXELS: f64 = 11.0;
/// `Encode(format, 100)`. `// C#: ExtLibs/MissionPlanner.Drawing/Image.cs:150-154`
const JPEG_QUALITY: u8 = 100;

/// A log's tracks: each vehicle's points (latitude, longitude) in the order its first was heard -
/// a dataflash log's one track - whether a simulator flew it, and the points' extremes.
#[derive(Debug, Clone, PartialEq)]
pub struct Tracks {
    /// `sitl`.
    pub sitl: bool,
    /// `loc_list`: `sysid * 256 + compid`, or 0, and its points.
    pub tracks: Vec<(u32, Vec<(f64, f64)>)>,
    /// `minx`, `maxx` (longitude), `miny`, `maxy` (latitude).
    bounds: [f64; 4],
}

impl Default for Tracks {
    fn default() -> Self {
        Self {
            sitl: false,
            tracks: Vec::new(),
            // `minx = 99999, maxx = -99999, ...`
            bounds: [99_999.0, -99_999.0, 99_999.0, -99_999.0],
        }
    }
}

impl Tracks {
    /// A point of track `id`, made if it is new.
    fn add(&mut self, id: u32, lat: f64, lng: f64) {
        match self.tracks.iter_mut().find(|(key, _)| *key == id) {
            Some((_, points)) => points.push((lat, lng)),
            None => self.tracks.push((id, vec![(lat, lng)])),
        }
        let [min_x, max_x, min_y, max_y] = &mut self.bounds;
        *min_x = min_x.min(lng);
        *max_x = max_x.max(lng);
        *min_y = min_y.min(lat);
        *max_y = max_y.max(lat);
    }

    /// `RectLatLng.FromLTRB(minx - 0.001, maxy + 0.001, maxx + 0.001, miny - 0.001)`.
    fn area(&self) -> mp_tiles::prefetch::Area {
        let [min_x, max_x, min_y, max_y] = self.bounds;
        mp_tiles::prefetch::Area {
            top: max_y + MARGIN,
            left: min_x - MARGIN,
            bottom: min_y - MARGIN,
            right: max_x + MARGIN,
        }
    }
}

/// A telemetry log's tracks: every `GLOBAL_POSITION_INT` away from 0, by the vehicle that sent it,
/// each degree a float as `loc.lat / 10000000.0f` makes it. `// C#: LogMap.cs:40-85`
#[must_use]
pub fn read_tlog(data: &[u8]) -> Tracks {
    let mut tracks = Tracks::default();
    let mut reader = mp_log::TlogReader::new(data);
    while let Some(record) = reader.next_record(&DIALECT) {
        let Ok((frame, _)) = mp_mavlink::parse(record.frame, &DIALECT) else {
            continue;
        };
        if frame.msgid == SIM_STATE || frame.msgid == SIMSTATE {
            tracks.sitl = true;
        }
        if frame.msgid != GLOBAL_POSITION_INT {
            continue;
        }
        let Some(MavMessage::GlobalPositionInt(position)) =
            MavMessage::decode(frame.msgid, frame.payload)
        else {
            continue;
        };
        if position.lat == 0 || position.lon == 0 {
            continue;
        }
        let id = u32::from(frame.sysid) * 256 + u32::from(frame.compid);
        #[allow(clippy::cast_precision_loss)] // `loc.lat / 10000000.0f`: in float, as the C#
        let degrees = |value: i32| f64::from(value as f32 / 10_000_000.0_f32);
        tracks.add(id, degrees(position.lat), degrees(position.lon));
    }
    tracks
}

/// A dataflash log's track: its `GPS` lines with a 3D fix or better away from 0.
///
/// # Errors
///
/// A `GPS` line without a `Status`, `Lat` or `Lng` column, or a value that is no number - where
/// the C# throws and its `catch` writes no picture. `// C#: LogMap.cs:88-122`
pub fn read_dataflash(data: &[u8]) -> Result<Tracks, String> {
    let mut buffer =
        mp_log::dflogbuffer::DfLogBuffer::new(data, &mp_log::convert::flight_mode_name);
    let mut tracks = Tracks::default();
    // `loc_list[0] = new List<PointLatLngAlt>()`
    tracks.tracks.push((0, Vec::new()));
    for (line, item) in buffer.items_of(&["GPS"]) {
        let msgtype = item.msgtype().to_owned();
        if !msgtype.starts_with("GPS") || buffer.dflog.label("GPS").is_none() {
            continue;
        }
        let mut value = |name: &str| -> Result<f64, String> {
            buffer
                .dflog
                .find_message_offset(&msgtype, name)
                .and_then(|index| item.items.get(index))
                .and_then(Option::as_deref)
                .and_then(|text| text.trim().parse::<f64>().ok())
                .ok_or_else(|| format!("line {line}: {msgtype} {name}"))
        };
        let status = value("Status")?;
        let lat = value("Lat")?;
        let lng = value("Lng")?;
        #[allow(clippy::float_cmp)] // `lat == 0`, as the C#
        if lat == 0.0 || lng == 0.0 || status < 3.0 {
            continue;
        }
        tracks.add(0, lat, lng);
    }
    Ok(tracks)
}

/// What one log came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Made {
    /// Its picture was there already.
    Kept,
    /// The tracks over the map.
    Map,
    /// "No gps data".
    NoGps,
    /// Nothing: what the C#'s `catch` swallows.
    Nothing,
}

/// `ProcessFile(logfile)`, with `tile` giving each map tile Google's imagery has for the area.
/// `// C#: LogMap.cs:23-178`
pub fn process_file(log: &Path, tile: &mut dyn FnMut(TileId) -> Option<image::RgbaImage>) -> Made {
    let picture = picture_of(log);
    if picture.os_exists() {
        return Made::Kept;
    }
    let Ok(data) = mp_os::fs::read(log) else {
        return Made::Nothing;
    };
    let name = log.to_string_lossy().to_lowercase();
    let tracks = if name.ends_with(".tlog") {
        read_tlog(&data)
    } else if name.ends_with(".bin") || name.ends_with(".log") {
        match read_dataflash(&data) {
            Ok(tracks) => tracks,
            Err(why) => {
                log::debug!("LogMap {}: {why}", log.display());
                return Made::Nothing;
            }
        }
    } else {
        Tracks::default()
    };
    let drawn = tracks
        .tracks
        .first()
        .is_some_and(|(_, points)| points.len() > FEWEST_POINTS);
    let (pixmap, made) = if drawn {
        (map_picture(&tracks, tile), Made::Map)
    } else {
        (text_picture(NO_GPS_DATA), Made::NoGps)
    };
    let Some(pixmap) = pixmap else {
        return Made::Nothing;
    };
    if let Err(why) = save_jpeg(&pixmap, &picture) {
        log::debug!("LogMap {}: {why}", picture.display());
        return Made::Nothing;
    }
    stamp(&picture, log);
    made
}

/// `File.SetLastWriteTime(jpg, new FileInfo(logfile).LastWriteTime)`. `// C#: LogMap.cs:163, 170`
#[cfg(not(target_family = "wasm"))]
fn stamp(picture: &Path, log: &Path) {
    let Ok(time) = std::fs::metadata(log).and_then(|metadata| metadata.modified()) else {
        return;
    };
    if let Ok(file) = std::fs::File::options().write(true).open(picture) {
        let _ = file.set_modified(time);
    }
}

/// A page's store keeps the time of the write, and has none other to set.
#[cfg(target_family = "wasm")]
const fn stamp(_picture: &Path, _log: &Path) {}

/// `MercatorProjection.FromLatLngToPixel`, as `GetMap` asks it.
fn pixel(lat: f64, lng: f64, zoom: i32) -> (i64, i64) {
    crate::mapview::mercator_pixel(lat, lng, zoom)
}

/// `GetMap(area)` with the tracks drawn on it, and "SITL" when a simulator flew them.
/// `// C#: LogMap.cs:127-161, 200-279`
fn map_picture(
    tracks: &Tracks,
    tile: &mut dyn FnMut(TileId) -> Option<image::RgbaImage>,
) -> Option<Pixmap> {
    let area = tracks.area();
    // "zoom based on pixel density"
    let mut zoom = ZOOM;
    let (mut top_left, mut bottom_right) = (
        pixel(area.top, area.left, zoom),
        pixel(area.bottom, area.right, zoom),
    );
    while bottom_right.0 - top_left.0 > WIDEST {
        zoom -= 1;
        top_left = pixel(area.top, area.left, zoom);
        bottom_right = pixel(area.bottom, area.right, zoom);
    }
    let delta = (bottom_right.0 - top_left.0, bottom_right.1 - top_left.1);
    let width = u32::try_from(delta.0 + PADDING * 2).ok()?;
    let height = u32::try_from(delta.1 + PADDING * 2).ok()?;
    let mut map = Pixmap::new(width, height)?;
    // "get tiles & combine into one"
    for id in mp_tiles::prefetch::area_tiles(&area, u8::try_from(zoom).ok()?) {
        let Some(image) = tile(id) else {
            continue;
        };
        let Some(drawn) = rgba_pixmap(&image) else {
            continue;
        };
        let x = i64::from(id.x) * TILE - top_left.0 + PADDING;
        let y = i64::from(id.y) * TILE - top_left.1 + PADDING;
        let (Ok(x), Ok(y)) = (i32::try_from(x), i32::try_from(y)) else {
            continue;
        };
        map.draw_pixmap(
            x,
            y,
            drawn.as_ref(),
            &PixmapPaint::default(),
            Transform::identity(),
            None,
        );
    }
    if tracks.sitl {
        draw_text(&mut map, SITL);
    }
    #[allow(clippy::cast_precision_loss)] // a picture's pixels
    let size = (map.width() as f64, map.height() as f64);
    for (index, (_, points)) in tracks.tracks.iter().enumerate() {
        let Some(&(r, g, b)) = COLOURS.get(index % (COLOURS.len() - 1)) else {
            continue;
        };
        let mut paint = Paint::default();
        paint.set_color_rgba8(r, g, b, 255);
        paint.anti_alias = false;
        let mut last: Option<(f32, f32)> = None;
        for &(lat, lng) in points {
            let next = place(&area, lat, lng, size);
            if let Some(last) = last.filter(|&(x, y)| x != 0.0 || y != 0.0) {
                let mut line = PathBuilder::new();
                line.move_to(last.0, last.1);
                line.line_to(next.0, next.1);
                if let Some(line) = line.finish() {
                    map.stroke_path(
                        &line,
                        &paint,
                        &Stroke {
                            width: 1.0,
                            ..Stroke::default()
                        },
                        Transform::identity(),
                        None,
                    );
                }
            }
            last = Some(next);
        }
    }
    Some(map)
}

/// `GetPixel`: a point placed by a straight scale from the area's corners to the picture's.
/// `// C#: LogMap.cs:200-210`
#[allow(clippy::cast_possible_truncation)] // `(float)`
fn place(area: &mp_tiles::prefetch::Area, lat: f64, lng: f64, size: (f64, f64)) -> (f32, f32) {
    let x = (lng - area.left) * size.0 / (area.right - area.left);
    let y = (lat - area.top) * size.1 / (area.bottom - area.top);
    (x as f32, y as f32)
}

/// A decoded tile as a pixmap to draw.
fn rgba_pixmap(image: &image::RgbaImage) -> Option<Pixmap> {
    let mut pixmap = Pixmap::new(image.width(), image.height())?;
    for (out, pixel) in pixmap.pixels_mut().iter_mut().zip(image.pixels()) {
        let [r, g, b, a] = pixel.0;
        *out = tiny_skia::ColorU8::from_rgba(r, g, b, a).premultiply();
    }
    Some(pixmap)
}

/// `DoTextMap`: a 100 by 100 picture with the words at its top left. `// C#: LogMap.cs:180-193`
fn text_picture(text: &str) -> Option<Pixmap> {
    let mut map = Pixmap::new(TEXT_PICTURE, TEXT_PICTURE)?;
    draw_text(&mut map, text);
    Some(map)
}

/// `AddTextToMap`: the words in red, `SystemFonts.DefaultFont`, at the top left.
/// `// C#: LogMap.cs:195-198`
fn draw_text(map: &mut Pixmap, text: &str) {
    let Some(font) = default_font() else {
        return;
    };
    let Ok(segments) = crate::glyph_text::outline(&font, text, FONT_PIXELS) else {
        return;
    };
    let mut path = PathBuilder::new();
    #[allow(clippy::cast_possible_truncation)] // pixels
    let at = |value: f64| value as f32;
    for segment in segments {
        match segment {
            mp_mission::text_mission::Segment::MoveTo(x, y) => path.move_to(at(x), at(y)),
            mp_mission::text_mission::Segment::LineTo(x, y) => path.line_to(at(x), at(y)),
            mp_mission::text_mission::Segment::QuadTo { control, end } => {
                path.quad_to(at(control.0), at(control.1), at(end.0), at(end.1));
            }
            mp_mission::text_mission::Segment::CurveTo {
                control1,
                control2,
                end,
            } => path.cubic_to(
                at(control1.0),
                at(control1.1),
                at(control2.0),
                at(control2.1),
                at(end.0),
                at(end.1),
            ),
            mp_mission::text_mission::Segment::Close => path.close(),
        }
    }
    let Some(path) = path.finish() else {
        return;
    };
    let mut paint = Paint::default();
    paint.set_color(Color::from_rgba8(255, 0, 0, 255));
    map.fill_path(
        &path,
        &paint,
        tiny_skia::FillRule::Winding,
        Transform::identity(),
        None,
    );
}

/// The bytes of `SystemFonts.DefaultFont`: Microsoft Sans Serif where it is installed, fontconfig's
/// answer for it elsewhere; in a web page, the page's own font.
fn default_font() -> Option<Vec<u8>> {
    #[cfg(target_family = "wasm")]
    {
        Some(crate::page_fonts::FONTS[0].to_vec())
    }
    #[cfg(not(target_family = "wasm"))]
    {
        let file = if cfg!(windows) {
            PathBuf::from(std::env::var_os("WINDIR")?)
                .join("Fonts")
                .join("micross.ttf")
        } else {
            let output = std::process::Command::new("fc-match")
                .args(["-f", "%{file}", FONT_FAMILY])
                .output()
                .ok()?;
            PathBuf::from(String::from_utf8_lossy(&output.stdout).trim())
        };
        mp_os::fs::read(file).ok()
    }
}

/// `map.Save(jpg, SKEncodedImageFormat.Jpeg)`: what has no colour is black.
fn save_jpeg(pixmap: &Pixmap, path: &Path) -> Result<(), String> {
    let mut rgb = Vec::with_capacity(pixmap.pixels().len() * 3);
    for pixel in pixmap.pixels() {
        let colour = pixel.demultiply();
        rgb.extend_from_slice(&[colour.red(), colour.green(), colour.blue()]);
    }
    let mut bytes = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut bytes, JPEG_QUALITY)
        .encode(
            &rgb,
            pixmap.width(),
            pixmap.height(),
            image::ExtendedColorType::Rgb8,
        )
        .map_err(|error| error.to_string())?;
    mp_os::fs::write(path, bytes).map_err(|error| error.to_string())
}

/// The temp form's GetFiles: every file of the extension in `folder` and every folder under it -
/// in any case, as Windows matches the mask - a folder it may not read passed over.
/// `// C#: temp.cs:507-527`
#[must_use]
pub fn files_under(folder: &Path, extension: &str) -> Vec<PathBuf> {
    let Ok(entries) = mp_os::fs::read_dir(folder) else {
        return Vec::new();
    };
    let mut paths: Vec<PathBuf> = entries.filter_map(Result::ok).map(|e| e.path()).collect();
    paths.sort();
    let mut found = Vec::new();
    let mut folders = Vec::new();
    for path in paths {
        if path.os_is_dir() {
            folders.push(path);
        } else if path
            .extension()
            .is_some_and(|ext| ext.to_string_lossy().eq_ignore_ascii_case(extension))
        {
            found.push(path);
        }
    }
    for folder in folders {
        found.extend(files_under(&folder, extension));
    }
    found
}

/// Google's imagery for one tile: the cache's, else - unless the map is to fetch nothing - the
/// server's, kept in the cache, as `GMaps.Instance.GetImageFrom` gets it.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET/GMaps.cs (GetImageFrom)`
pub fn cached_or_fetched(
    cache: &mp_tiles::TileCache,
    fetcher: Option<&mp_tiles::TileFetcher>,
    id: TileId,
) -> Option<image::RgbaImage> {
    let source = &mp_tiles::source::GOOGLE_SATELLITE_MAP;
    let bytes = match cache.read(source.cache_name, id) {
        Some(tile) => tile.bytes,
        None => {
            let bytes = fetcher?.fetch(source, id).ok()?;
            let _ = cache.write(source.cache_name, id, &bytes);
            bytes
        }
    };
    image::load_from_memory(&bytes)
        .ok()
        .map(|image| image.to_rgba8())
}

/// `but_maplogs_Click` once its folder is chosen: `MapLogs` of every `.tlog`, then `.bin`, then
/// `.log` in and under it. Returns how many pictures were made.
///
/// # Errors
///
/// A folder that is not there, which `Directory.GetFiles` throws for. `// C#: temp.cs:529-540`
pub fn map_logs(
    folder: &Path,
    tile: &mut dyn FnMut(TileId) -> Option<image::RgbaImage>,
) -> std::io::Result<usize> {
    if !folder.os_is_dir() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            format!("Could not find a part of the path '{}'.", folder.display()),
        ));
    }
    let mut made = 0;
    for extension in ["tlog", "bin", "log"] {
        for log in files_under(folder, extension) {
            if matches!(process_file(&log, tile), Made::Map | Made::NoGps) {
                made += 1;
            }
        }
    }
    Ok(made)
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
        let folder = mp_os::temp_dir().join(format!("mp-logmap-{}-{name}", std::process::id()));
        let _ = mp_os::fs::remove_dir_all(&folder);
        mp_os::fs::create_dir_all(&folder).unwrap();
        folder
    }

    /// A tile of one colour, as a server might send.
    fn grey(_: TileId) -> Option<image::RgbaImage> {
        Some(image::RgbaImage::from_pixel(
            256,
            256,
            image::Rgba([90, 90, 90, 255]),
        ))
    }

    /// A dataflash log with a GPS track: a picture of it, the tiles under the track and the track
    /// in red over them; then left alone, its picture there.
    #[test]
    fn a_dataflash_log_is_drawn_over_the_map() {
        let folder = scratch("bin");
        let log = folder.join("flight.bin");
        mp_os::fs::write(
            &log,
            mp_os::fs::read(testdata("georef/camera.bin")).unwrap(),
        )
        .unwrap();
        let data = mp_os::fs::read(&log).unwrap();
        let tracks = read_dataflash(&data).unwrap();
        assert_eq!(tracks.tracks.len(), 1);
        assert!(tracks.tracks[0].1.len() > FEWEST_POINTS);
        assert_eq!(process_file(&log, &mut grey), Made::Map);
        let picture = image::open(picture_of(&log)).unwrap().to_rgb8();
        assert!(picture.width() > 2 * 10 && picture.width() <= 2000 + 20);
        let mut red = 0;
        let mut tiled = 0;
        for pixel in picture.pixels() {
            let [r, g, b] = pixel.0;
            if r > 200 && g < 60 && b < 60 {
                red += 1;
            }
            if (80..100).contains(&r) && (80..100).contains(&g) && (80..100).contains(&b) {
                tiled += 1;
            }
        }
        assert!(red > 10, "the track is drawn: {red}");
        assert!(tiled > picture.pixels().len() / 2, "the tiles are under it");
        assert_eq!(process_file(&log, &mut grey), Made::Kept);
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// A dataflash log with no GPS fix says "No gps data" in a 100 by 100 picture - red words on
    /// black where there is a font to write them in; a log that is not there says nothing.
    #[test]
    fn no_fix_says_so() {
        let folder = scratch("nofix");
        let log = folder.join("bench.bin");
        mp_os::fs::write(&log, mp_os::fs::read(testdata("dataflash.bin")).unwrap()).unwrap();
        let tracks = read_dataflash(&mp_os::fs::read(&log).unwrap()).unwrap();
        assert_eq!(tracks.tracks, vec![(0, Vec::new())]);
        assert_eq!(process_file(&log, &mut grey), Made::NoGps);
        let picture = image::open(picture_of(&log)).unwrap().to_rgb8();
        assert_eq!((picture.width(), picture.height()), (100, 100));
        let red = picture
            .pixels()
            .filter(|p| p.0[0] > 150 && p.0[1] < 80)
            .count();
        assert_eq!(red > 0, default_font().is_some(), "{red} red pixels");
        assert_eq!(
            process_file(&folder.join("gone.tlog"), &mut grey),
            Made::Nothing
        );
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// A simulator's telemetry log: its vehicle's track over the map, "SITL" in the top left where
    /// there is a font.
    #[test]
    fn a_simulators_telemetry_log_says_sitl() {
        let folder = scratch("sitl");
        let log = folder.join("sim.tlog");
        mp_os::fs::write(
            &log,
            mp_os::fs::read(testdata("georef/camera.tlog")).unwrap(),
        )
        .unwrap();
        let tracks = read_tlog(&mp_os::fs::read(&log).unwrap());
        assert!(tracks.sitl);
        assert_eq!(tracks.tracks.len(), 1);
        assert_eq!(tracks.tracks[0].0, 257);
        assert_eq!(process_file(&log, &mut grey), Made::Map);
        let picture = image::open(picture_of(&log)).unwrap().to_rgb8();
        let corner_red = (0..12)
            .flat_map(|y| (0..30).map(move |x| (x, y)))
            .filter(|&(x, y)| {
                let [r, g, b] = picture.get_pixel(x, y).0;
                r > 150 && g < 80 && b < 80
            })
            .count();
        assert_eq!(corner_red > 0, default_font().is_some(), "{corner_red}");
        let _ = mp_os::fs::remove_dir_all(&folder);
    }

    /// The tracks are placed by a straight scale from the area to the picture, corner to corner.
    #[test]
    fn points_are_placed_by_a_straight_scale() {
        let area = mp_tiles::prefetch::Area {
            top: 1.0,
            left: 10.0,
            bottom: 0.0,
            right: 12.0,
        };
        assert_eq!(place(&area, 1.0, 10.0, (200.0, 100.0)), (0.0, 0.0));
        assert_eq!(place(&area, 0.0, 12.0, (200.0, 100.0)), (200.0, 100.0));
        assert_eq!(place(&area, 0.5, 11.0, (200.0, 100.0)), (100.0, 50.0));
    }

    /// The colours go round seven, pink never reached, as `a % (colours.Length - 1)` has it.
    #[test]
    fn seven_colours_go_round() {
        let picked: Vec<_> = (0..9).map(|a| COLOURS[a % (COLOURS.len() - 1)]).collect();
        assert_eq!(picked[7], COLOURS[0]);
        assert!(!picked.contains(&(255, 192, 203)));
    }

    /// The folders are walked for each kind, in and under the one chosen.
    #[test]
    fn logs_are_found_under_the_folder() {
        let folder = scratch("walk");
        mp_os::fs::create_dir_all(folder.join("deep")).unwrap();
        mp_os::fs::write(folder.join("a.TLOG"), b"x").unwrap();
        mp_os::fs::write(folder.join("deep").join("b.tlog"), b"x").unwrap();
        mp_os::fs::write(folder.join("c.bin"), b"x").unwrap();
        assert_eq!(files_under(&folder, "tlog").len(), 2);
        assert_eq!(files_under(&folder, "bin").len(), 1);
        assert!(files_under(&folder.join("gone"), "tlog").is_empty());
        let _ = mp_os::fs::remove_dir_all(&folder);
    }
}
