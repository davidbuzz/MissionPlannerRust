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

//! Points of interest on the flight map: `Utilities/POI.cs`.
//!
//! A point of interest is a position with an ID the user typed. They are kept in one list for the
//! whole application, written to `poi.txt` in the user data directory whenever the list changes
//! and read back from it when the flight screen starts, and drawn on the flight map as red markers
//! (`GMapMarkerPOI`, a `GMarkerGoogle` red dot). The map's context menu adds one where the map was
//! pressed (Add Poi), adds one at typed coordinates (Coords), and deletes the one under the
//! pointer (Delete).
//! `// C#: Utilities/POI.cs, ExtLibs/Maps/GMapMarkerPOI.cs, GCSViews/FlightData.cs:1007-1010,
//! 2632-2638, 6011-6038`
//!
//! Save File and Load File, the menu's other two entries, ask for a file with the system's file
//! dialogs, which this application does not have; they are not here. The file the C# keeps by
//! itself is.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};

use gpui::{Bounds, Window, point, px, rgb, size};
use mp_params::param_file::invariant_double;
use mp_units::LatLon;

/// The C#'s file name, under `Settings.GetUserDataDirectory()`. `// C#: Utilities/POI.cs:39`
pub const FILE_NAME: &str = "poi.txt";

/// `InputBox.Show("POI", "Enter ID", ...)`. `// C#: Utilities/POI.cs:73`
pub const ID_TITLE: &str = "POI";
/// See [`ID_TITLE`].
pub const ID_TEXT: &str = "Enter ID";
/// `InputBox.Show("Enter POI Coords", ...)`. `// C#: GCSViews/FlightData.cs:6014`
pub const COORDS_TITLE: &str = "Enter POI Coords";
/// See [`COORDS_TITLE`].
pub const COORDS_TEXT: &str = "Please enter the coords 'lat;long;alt' or 'lat;long'";

/// `GMarkerGoogleType.red_dot`'s picture: 32 pixels square, its bottom middle on the point.
/// `// C#: ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms.Markers/GMarkerGoogle.cs`
pub const MARKER: f32 = 32.0;

/// One point of interest: a `PointLatLngAlt` whose `Tag` is the ID, a newline, and the point as
/// it was when added - `Lat,Lng,Alt,` and its old tag, which is empty.
/// `// C#: Utilities/POI.cs:57-67, ExtLibs/Utilities/PointLatLngAlt.cs:186-189`
#[derive(Debug, Clone, PartialEq)]
pub struct Poi {
    /// Latitude, degrees.
    pub lat: f64,
    /// Longitude, degrees.
    pub lng: f64,
    /// Altitude, metres.
    pub alt: f64,
    /// `Tag`: the ID, a newline, and the point written out - the marker's tooltip.
    pub tag: String,
}

impl Poi {
    /// `POIAdd(Point, tag)`: the point with its tag made up.
    #[must_use]
    pub fn new(lat: f64, lng: f64, alt: f64, id: &str) -> Self {
        let written = format!(
            "{},{},{},",
            invariant_double(lat),
            invariant_double(lng),
            invariant_double(alt)
        );
        Self {
            lat,
            lng,
            alt,
            tag: format!("{id}\n{written}"),
        }
    }

    /// The ID: the tag up to its newline.
    #[must_use]
    pub fn id(&self) -> &str {
        self.tag.split('\n').next().unwrap_or_default()
    }

    /// Where it is, if that is a position at all.
    #[must_use]
    pub fn position(&self) -> Option<LatLon> {
        LatLon::new(self.lat, self.lng).ok()
    }
}

/// The list, and where it is kept.
#[derive(Debug, Default)]
pub struct Pois {
    points: Vec<Poi>,
    /// `POI.filename`, or `None` where there is no user data directory.
    file: Option<PathBuf>,
    /// A point waiting for its ID to be typed: `POIAdd(Point)` between asking and adding.
    pub pending: Option<(f64, f64, f64)>,
    /// What the last save said, if it failed.
    pub error: Option<String>,
}

impl Pois {
    /// The list kept in `file`, read from it if it is there: the C# reads the file when the
    /// flight screen first subscribes to `POIModified`. `// C#: Utilities/POI.cs:23-37`
    #[must_use]
    pub fn kept_in(file: Option<PathBuf>) -> Self {
        let points = file
            .as_deref()
            .and_then(|path| std::fs::read(path).ok())
            .map(|bytes| parse(&String::from_utf8_lossy(&bytes)))
            .unwrap_or_default();
        Self {
            points,
            file,
            pending: None,
            error: None,
        }
    }

    /// The list kept where the C# keeps it: `poi.txt` in the user data directory.
    #[must_use]
    pub fn load_default() -> Self {
        Self::kept_in(mp_settings::user_data_directory().map(|dir| dir.join(FILE_NAME)))
    }

    /// The points, in the order they were added.
    #[must_use]
    pub fn points(&self) -> &[Poi] {
        &self.points
    }

    /// Where the list is written.
    #[must_use]
    pub fn file(&self) -> Option<&Path> {
        self.file.as_deref()
    }

    /// `POIAdd(Point, tag)`, then the save `CollectionChanged` makes.
    pub fn add(&mut self, lat: f64, lng: f64, alt: f64, id: &str) {
        self.points.push(Poi::new(lat, lng, alt, id));
        self.save();
    }

    /// `POIEdit`: the point's tag rewritten with a new ID and the point written out again, then
    /// the save `POIModified` makes. Returns whether there was a point at `index`.
    /// `// C#: Utilities/POI.cs:104-124`
    pub fn rename(&mut self, index: usize, id: &str) -> bool {
        let Some(poi) = self.points.get_mut(index) else {
            return false;
        };
        *poi = Poi::new(poi.lat, poi.lng, poi.alt, id);
        self.save();
        true
    }

    /// `POIDelete`: the first point at the marker's position, removed and the list saved.
    /// Returns whether one was.
    pub fn delete(&mut self, index: usize) -> bool {
        if index >= self.points.len() {
            return false;
        }
        self.points.remove(index);
        self.save();
        true
    }

    /// `SaveFile(filename)`, errors swallowed as the C# swallows them - but kept, so the screen
    /// can say so. `// C#: Utilities/POI.cs:44-55`
    fn save(&mut self) {
        let Some(path) = self.file.clone() else {
            return;
        };
        self.error = write(&path, &self.points)
            .err()
            .map(|err| format!("{}: {err}", path.display()));
    }

    /// Publishes what a UI test asserts on: how many there are, and each one's ID and position.
    pub fn record_facts(&self) {
        crate::facts::record("fly.poi.count", self.points.len());
        crate::facts::record(
            "fly.poi.list",
            self.points
                .iter()
                .map(|poi| {
                    format!(
                        "{} {};{}",
                        poi.id(),
                        invariant_double(poi.lat),
                        invariant_double(poi.lng)
                    )
                })
                .collect::<Vec<_>>()
                .join(" | "),
        );
        crate::facts::record(
            "fly.poi.file",
            self.file()
                .map_or_else(|| "none".to_owned(), |path| path.display().to_string()),
        );
        crate::facts::record("fly.poi.error", self.error.as_deref().unwrap_or("none"));
    }
}

/// The file as `SaveFile` writes it: `lat<TAB>lng<TAB>ID`, each number as `ToString` writes it
/// and each line ended `\r\n`, in ASCII. `// C#: Utilities/POI.cs:144-157`
#[must_use]
pub fn render(points: &[Poi]) -> String {
    let mut text = String::new();
    for poi in points {
        text.push_str(&invariant_double(poi.lat));
        text.push('\t');
        text.push_str(&invariant_double(poi.lng));
        text.push('\t');
        text.push_str(poi.id());
        text.push_str("\r\n");
    }
    // `ASCIIEncoding` writes anything outside ASCII as '?'.
    text.chars()
        .map(|c| if c.is_ascii() { c } else { '?' })
        .collect()
}

/// Writes the list, making the directory the C# application makes when it starts.
fn write(path: &Path, points: &[Poi]) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(path, render(points))
}

/// The file as `LoadFile` reads it: each line split on tabs, a line with fewer than three parts
/// skipped, the first two parsed as the latitude and longitude, the third the ID, and the
/// altitude 0. A number that does not parse throws in the C#, ending the load; the points read
/// before it are kept, and so they are here. `// C#: Utilities/POI.cs:172-193`
#[must_use]
pub fn parse(text: &str) -> Vec<Poi> {
    let mut points = Vec::new();
    for line in text.lines() {
        let items: Vec<&str> = line.split('\t').collect();
        let [lat, lng, id, ..] = items.as_slice() else {
            continue;
        };
        let (Ok(lat), Ok(lng)) = (lat.trim().parse::<f64>(), lng.trim().parse::<f64>()) else {
            break;
        };
        points.push(Poi::new(lat, lng, 0.0, id));
    }
    points
}

/// What Coords reads: `lat;long;alt` or `lat;long`, each parsed as a `float` - so the degrees
/// are a single's, widened. Anything else is `Strings.InvalidField`. With no altitude the C#
/// asks the terrain database at the point last pressed; with no terrain data that is
/// `altresponce.Invalid`, whose altitude is 0, and this application has no terrain data.
/// `// C#: GCSViews/FlightData.cs:6011-6038, ExtLibs/Utilities/srtm.cs:32`
pub fn parse_coords(text: &str) -> Result<(f64, f64, f64), &'static str> {
    let parts: Vec<&str> = text.split(';').collect();
    let number = |part: &str| {
        part.trim()
            .parse::<f32>()
            .map(f64::from)
            .map_err(|_| crate::fly::strings::INVALID_FIELD)
    };
    match parts.as_slice() {
        [lat, lng, alt] => Ok((number(lat)?, number(lng)?, number(alt)?)),
        [lat, lng] => Ok((number(lat)?, number(lng)?, 0.0)),
        _ => Err(crate::fly::strings::INVALID_FIELD),
    }
}

/// Which point's marker is under a press at window position `at`, given where each marker's
/// point was drawn: the red dot's picture, 32 pixels square with its bottom middle on the point.
/// The newest is on top, so it is found first, as the C#'s hover finds the marker drawn last.
/// GMap places a marker on whole pixels; the half pixel either side is that rounding, so a press
/// on the very point a marker was added at finds it however the projection rounds back.
#[must_use]
pub fn under(drawn: &[Option<(f32, f32)>], at: (f32, f32)) -> Option<usize> {
    const ROUNDING: f32 = 0.5;
    drawn.iter().enumerate().rev().find_map(|(index, spot)| {
        let (x, y) = (*spot)?;
        let inside = at.0 >= x - MARKER / 2.0 - ROUNDING
            && at.0 <= x + MARKER / 2.0 + ROUNDING
            && at.1 >= y - MARKER - ROUNDING
            && at.1 <= y + ROUNDING;
        inside.then_some(index)
    })
}

/// The markers over the flight map, `UpdateOverlay`'s: painted after the map, each where the map
/// drew its point in the same frame, and kept inside the map.
/// `// C#: Utilities/POI.cs:195-211`
pub fn layer(
    pois: &Pois,
    map: std::rc::Rc<std::cell::RefCell<crate::mapview::MapViewport>>,
) -> impl gpui::IntoElement {
    use gpui::{ParentElement as _, Styled as _};
    let points: Vec<LatLon> = pois.points().iter().filter_map(Poi::position).collect();
    // Each marker's rectangle, `fly-poi-<index>`, measured for the harness: the pointer over it is
    // the pointer over the C#'s marker (`OnMarkerEnter`'s `CurrentPOIMarker`), which the map's
    // menu acts on. Where the map drew the point last frame, in the layer's own coordinates;
    // nothing listens on them.
    let boxes: Vec<(usize, f32, f32)> = {
        let map = map.borrow();
        points
            .iter()
            .enumerate()
            .filter_map(|(index, at)| {
                let (x, y) = map.screen_of(*at)?;
                let (x, y) = map.to_viewport(x, y);
                Some((index, x, y))
            })
            .collect()
    };
    let painter = gpui::canvas(
        |_bounds, _window, _cx| (),
        move |bounds, (), window, _cx| {
            let spots: Vec<(f32, f32)> = {
                let map = map.borrow();
                points.iter().filter_map(|at| map.screen_of(*at)).collect()
            };
            window.with_content_mask(Some(gpui::ContentMask { bounds }), |window| {
                paint(&spots, window);
            });
        },
    )
    .absolute()
    .size_full();
    gpui::div()
        .absolute()
        .size_full()
        .child(painter)
        .children(boxes.into_iter().map(|(index, x, y)| {
            crate::probe::measured(format!("fly-poi-{index}"), gpui::div())
                .absolute()
                .left(px(x - MARKER / 2.0))
                .top(px(y - MARKER))
                .w(px(MARKER))
                .h(px(MARKER))
                .child(gpui::div().size_full())
        }))
}

/// Paints the markers: a red dot on a stem whose foot is the point, as `red_dot` pictures it.
fn paint(spots: &[(f32, f32)], window: &mut Window) {
    for &(x, y) in spots {
        let radius = 7.0;
        let centre_y = y - MARKER + radius + 3.0;
        // The stem, from the dot down to the point.
        window.paint_quad(gpui::fill(
            Bounds {
                origin: point(px(x - 1.0), px(centre_y)),
                size: size(px(2.0), px(y - centre_y)),
            },
            rgb(0x8b_00_00),
        ));
        window.paint_quad(
            gpui::fill(
                Bounds {
                    origin: point(px(x - radius), px(centre_y - radius)),
                    size: size(px(radius * 2.0), px(radius * 2.0)),
                },
                rgb(0xe5_39_35),
            )
            .corner_radii(px(radius))
            .border_widths(px(1.0))
            .border_color(rgb(0x8b_00_00)),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        // Nanoseconds since the epoch, not `SystemTime`'s Debug text: that carries braces and a
        // colon, which Windows refuses in a folder name (os error 123, the hosted runner,
        // 2026-10-04).
        let stamp = web_time::SystemTime::now()
            .duration_since(web_time::UNIX_EPOCH)
            .map_or(0, |since| since.as_nanos());
        let dir = mp_os::temp_dir().join(format!(
            "headless-planner-poi-{}-{name}-{stamp}",
            mp_os::process_id()
        ));
        dir.join("MissionPlannerRust").join(FILE_NAME)
    }

    #[test]
    fn a_point_is_tagged_with_its_id_and_itself() {
        let poi = Poi::new(-35.36, 149.16, 0.0, "tower");
        assert_eq!(poi.tag, "tower\n-35.36,149.16,0,");
        assert_eq!(poi.id(), "tower");
    }

    #[test]
    fn the_file_is_the_csharps_tab_separated_lines() {
        let points = vec![
            Poi::new(-35.3632621, 149.1652374, 0.0, "one"),
            Poi::new(0.000_01, 12.5, 0.0, "two"),
        ];
        assert_eq!(
            render(&points),
            "-35.3632621\t149.1652374\tone\r\n1E-05\t12.5\ttwo\r\n"
        );
        // Read back: the same IDs and places, at altitude 0.
        let read = parse(&render(&points));
        assert_eq!(read.len(), 2);
        assert_eq!(read[0].id(), "one");
        assert_eq!(read[1].lat, 0.000_01);
        assert_eq!(read[1].alt, 0.0);
    }

    #[test]
    fn a_short_line_is_skipped_and_a_bad_number_ends_the_load() {
        let read = parse("1\t2\n3\t4\tkept\nx\t5\tlost\n6\t7\tafter\n");
        assert_eq!(read.len(), 1);
        assert_eq!(read[0].id(), "kept");
    }

    #[test]
    fn every_change_is_saved_and_the_file_is_read_at_start() {
        let path = scratch("saved");
        let mut pois = Pois::kept_in(Some(path.clone()));
        assert!(pois.points().is_empty());
        pois.add(-35.0, 149.0, 0.0, "a");
        pois.add(-35.5, 149.5, 10.0, "b");
        assert_eq!(pois.error, None);
        let written = std::fs::read_to_string(&path).unwrap();
        assert_eq!(written, "-35\t149\ta\r\n-35.5\t149.5\tb\r\n");
        assert!(pois.delete(0));
        assert!(!pois.delete(5));
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            "-35.5\t149.5\tb\r\n"
        );
        // A new screen reads what the last one left.
        let again = Pois::kept_in(Some(path.clone()));
        assert_eq!(again.points().len(), 1);
        assert_eq!(again.points()[0].id(), "b");
        let _ = std::fs::remove_dir_all(path.parent().unwrap().parent().unwrap());
    }

    #[test]
    fn coords_are_two_or_three_floats() {
        let (lat, lng, alt) = parse_coords("-35.3625;149.1657;20").unwrap();
        // A single's degrees, widened: not the typed decimal.
        assert_eq!(lat, f64::from(-35.3625f32));
        assert_eq!(lng, f64::from(149.1657f32));
        assert_eq!(alt, 20.0);
        assert_eq!(parse_coords("1;2").unwrap(), (1.0, 2.0, 0.0));
        assert_eq!(parse_coords(""), Err("Invalid Field"));
        assert_eq!(parse_coords("1;2;3;4"), Err("Invalid Field"));
        assert_eq!(parse_coords("a;2"), Err("Invalid Field"));
    }

    #[test]
    fn a_press_finds_the_marker_it_lands_on_newest_first() {
        let drawn = [Some((100.0, 100.0)), None, Some((110.0, 100.0))];
        // On the dot, above the point: both markers' pictures cover it; the newer wins.
        assert_eq!(under(&drawn, (105.0, 80.0)), Some(2));
        // Only the first's.
        assert_eq!(under(&drawn, (86.0, 90.0)), Some(0));
        // On the point itself, however the projection rounded it back.
        assert_eq!(under(&drawn, (86.0, 100.3)), Some(0));
        // Below the point: nothing.
        assert_eq!(under(&drawn, (100.0, 101.0)), None);
    }
}
