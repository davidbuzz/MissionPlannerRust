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

//! The flight map's camera shots: `photosoverlay`'s photo markers and, with Camera Overlap
//! checked, `kmlpolygons`' overlap count - the camera half of `FlightData.mainloop`'s map update
//! and the toggle's handler.
//!
//! Each `CAMERA_FEEDBACK` the vehicle has sent (`MAV.camerapoints`, which the link keeps) is a
//! `GMapMarkerPhoto` at its position: the camera icon, red when the shot came sooner after the
//! one before than `CAM_MIN_INTERVAL` allows, and its footprint - `ImageProjection.calc` with
//! the shot's altitude, roll, pitch and yaw over the survey grid's `camera_fovh`/`camera_fovv`
//! (`GMapMarkerPhoto.hfov`/`vfov`, 63 by 43 until the grid has saved them) - drawn for the last
//! four shots and under the pointer. With Camera Overlap on, every footprint whose roll is under
//! 25 degrees goes into a `GMapMarkerOverlapCount`: each 0.0001-degree cell of their area counted
//! for the footprints that hold it, drawn as a disc in the count's colour with a legend down the
//! map's left. Camera Overlap is `CheckOnClick`; unchecking it clears the photo markers, which
//! the next update puts back, and takes the count off the map.
//!
//! The C# builds the markers and the count afresh on every pass of the loop; here they are built
//! again only when a shot, the toggle, the interval or the fields of view have changed, which
//! draws the same thing. A shot whose position is not a place (a latitude past 90) is left out,
//! where the C# would put a marker nowhere.
//! `// C#: GCSViews/FlightData.cs:4115-4196, 4571-4585; ExtLibs/Maps/GMapMarkerPhoto.cs;
//! ExtLibs/Maps/GMapMarkerOverlapCount.cs`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;

use mp_georef::projection::{Point, Terrain, calc};
use mp_mavlink_dialects::all::CameraFeedback;
use mp_units::LatLon;

use crate::mapview::PhotoMarker;

/// `GMapMarkerPhoto.hfov` and `vfov` as the statics start: 63 by 43.
pub const DEFAULT_FOV: (f64, f64) = crate::georef_ui::PHOTO_FOV;
/// `rolltrim`, `pitchtrim` and `yawtrim`: statics nothing sets.
const TRIM: (f64, f64, f64) = (0.0, 0.0, 0.0);
/// "abandon roll higher than 25 degrees".
const OVERLAP_ROLL_LIMIT: f64 = 25.0;
/// `drawfootprint` on the last four markers.
const FOOTPRINTS_DRAWN: usize = 4;
/// The count's grid, `0.0001` degrees a cell, in whole cells.
const CELLS_PER_DEGREE: f64 = 10_000.0;

/// `Settings.Instance["camera_fovh"] != null`: both fields of view from the settings while the
/// horizontal one is saved - `GetDouble`, which is 0 for a value that is not a number - else the
/// statics. `// C#: GCSViews/FlightData.cs:4128-4133; ExtLibs/Utilities/Settings.cs:245-254`
pub fn fov(get: impl Fn(&str) -> Option<String>) -> (f64, f64) {
    let get_double = |key: &str| {
        get(key)
            .and_then(|text| text.trim().parse::<f64>().ok())
            .unwrap_or(0.0)
    };
    if get("camera_fovh").is_some() {
        (get_double("camera_fovh"), get_double("camera_fovv"))
    } else {
        DEFAULT_FOV
    }
}

/// `srtm.getAltitude`: the planner's own terrain, which the footprints are projected onto.
#[derive(Debug, Clone, Copy, Default)]
pub struct PlannerTerrain;

impl Terrain for PlannerTerrain {
    fn altitude(&self, lat: f64, lng: f64) -> f64 {
        crate::srtm::altitude(lat, lng).alt
    }
}

/// One shot's marker, with what the overlap count needs of it.
#[derive(Debug, Clone, PartialEq)]
pub struct Photo {
    /// The marker.
    pub marker: PhotoMarker,
    /// `Roll`, degrees.
    pub roll: f64,
    /// `Tag`: `time_usec`, which names the footprint too (`"FP" + time_usec`).
    pub time_usec: u64,
}

/// `new GMapMarkerPhoto(mark, timesincelastshot < min_interval)` for every shot in order, the
/// last four with their footprints drawn. `timesincelastshot` is the shot's seconds less the one
/// before's, the first's measured from `double.MinValue`.
/// `// C#: GCSViews/FlightData.cs:4135-4152, 4157-4176; ExtLibs/Maps/GMapMarkerPhoto.cs:38-63`
pub fn photo_markers(
    points: &[CameraFeedback],
    min_interval: f64,
    fov: (f64, f64),
    terrain: &dyn Terrain,
) -> Vec<Photo> {
    let count = points.len();
    let mut old_time = f64::MIN;
    points
        .iter()
        .enumerate()
        .filter_map(|(index, mark)| {
            #[allow(clippy::cast_precision_loss)] // `mark.time_usec / 1000.0`, a ulong in a double
            let seconds = (mark.time_usec as f64 / 1000.0) / 1000.0;
            let since_last = seconds - old_time;
            old_time = seconds;
            let position = LatLon::from_mavlink_e7(mark.lat, mark.lng).ok()?;
            let roll = f64::from(mark.roll) - TRIM.0;
            let pitch = f64::from(mark.pitch) - TRIM.1;
            let yaw = f64::from(mark.yaw) - TRIM.2;
            let plla = Point {
                lat: position.latitude(),
                lng: position.longitude(),
                alt: f64::from(mark.alt_msl),
            };
            let footprint = calc(plla, roll, pitch, yaw, fov.0, fov.1, terrain)
                .iter()
                .filter_map(|corner| LatLon::new(corner.lat, corner.lng).ok())
                .collect();
            Some(Photo {
                marker: PhotoMarker {
                    position,
                    below_min_interval: since_last < min_interval,
                    footprint,
                    draw_footprint: index + FOOTPRINTS_DRAWN >= count,
                    tooltip: format!(
                        "Photo\nAlt: {}\nNo: {}\nRoll: {roll:.2}",
                        mark.alt_msl, mark.img_idx
                    ),
                },
                roll,
                time_usec: mark.time_usec,
            })
        })
        .collect()
}

/// `GMapPolygon.IsInside`: the even-odd rule over the polygon's edges, latitude as y and
/// longitude as x. `// C#: ExtLibs/GMap.NET.Drawing/GMap.NET.WindowsForms/GMapPolygon.cs:266-295`
#[must_use]
pub fn is_inside(points: &[(f64, f64)], (lat, lng): (f64, f64)) -> bool {
    let count = points.len();
    if count < 3 {
        return false;
    }
    let mut result = false;
    let mut j = count - 1;
    for i in 0..count {
        let (Some(p1), Some(p2)) = (points.get(i), points.get(j)) else {
            break;
        };
        if (p1.0 < lat && p2.0 >= lat || p2.0 < lat && p1.0 >= lat)
            && p1.1 + (lat - p1.0) / (p2.0 - p1.0) * (p2.1 - p1.1) < lng
        {
            result = !result;
        }
        j = i;
    }
    result
}

/// `GMapMarkerOverlapCount`: the footprints added, each once by its name, the area they cover
/// and the count at each 0.0001-degree cell inside any of them.
/// `// C#: ExtLibs/Maps/GMapMarkerOverlapCount.cs:13-26, 99-135, 210-241`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct OverlapCount {
    /// `footprintpolys`' names.
    names: Vec<u64>,
    /// `area`: top latitude, left longitude, bottom latitude, right longitude.
    area: Option<(f64, f64, f64, f64)>,
    /// `overlapCount`, keyed by the cell in whole cells.
    counts: BTreeMap<(i64, i64), u32>,
}

impl OverlapCount {
    /// `Add`: nothing for a footprint already added by name; the area reset to the first point
    /// of the first footprint and enlarged round every point, then the footprint's cells counted
    /// (`generateCoverageFP`). A footprint with no points is left out, where the C# would throw.
    pub fn add(&mut self, name: u64, footprint: &[LatLon]) {
        if self.names.contains(&name) {
            return;
        }
        let Some(first) = footprint.first() else {
            return;
        };
        if self.names.is_empty() {
            self.area = Some((
                first.latitude(),
                first.longitude(),
                first.latitude(),
                first.longitude(),
            ));
        }
        self.names.push(name);
        if let Some(area) = self.area.as_mut() {
            for point in footprint {
                let (lat, lng) = (point.latitude(), point.longitude());
                let inside = lat <= area.0 && lat >= area.2 && lng >= area.1 && lng <= area.3;
                if !inside {
                    *area = (
                        area.0.max(lat),
                        area.1.min(lng),
                        area.2.min(lat),
                        area.3.max(lng),
                    );
                }
            }
        }
        self.generate_coverage(footprint);
    }

    /// `generateCoverageFP`: every cell of the area, from the rounded top-left down and across,
    /// counted once more where the footprint holds it; nothing for an area over a degree either
    /// way.
    fn generate_coverage(&mut self, footprint: &[LatLon]) {
        let Some((top, left, bottom, right)) = self.area else {
            return;
        };
        if right - left > 1.0 || top - bottom > 1.0 {
            return;
        }
        let polygon: Vec<(f64, f64)> = footprint
            .iter()
            .map(|point| (point.latitude(), point.longitude()))
            .collect();
        let mut lat = round_cell(top);
        while cell_degrees(lat) >= bottom {
            let mut lng = round_cell(left);
            while cell_degrees(lng) <= right {
                if is_inside(&polygon, (cell_degrees(lat), cell_degrees(lng))) {
                    *self.counts.entry((lat, lng)).or_insert(0) += 1;
                }
                lng += 1;
            }
            lat -= 1;
        }
    }

    /// The cells and their counts, for the map.
    #[must_use]
    pub fn cells(&self) -> Vec<(LatLon, u32)> {
        self.counts
            .iter()
            .filter_map(|((lat, lng), count)| {
                LatLon::new(cell_degrees(*lat), cell_degrees(*lng))
                    .ok()
                    .map(|at| (at, *count))
            })
            .collect()
    }
}

/// `Math.Round(x, 4)` as a whole number of cells.
#[allow(clippy::cast_possible_truncation)] // degrees times ten thousand
fn round_cell(degrees: f64) -> i64 {
    (degrees * CELLS_PER_DEGREE).round() as i64
}

/// A whole number of cells back in degrees.
#[allow(clippy::cast_precision_loss)]
fn cell_degrees(cells: i64) -> f64 {
    cells as f64 / CELLS_PER_DEGREE
}

/// What the markers and the count were last built from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Key {
    count: usize,
    last_time: u64,
    overlap: bool,
    fov: (u64, u64),
    min_interval: u64,
}

/// `photosoverlay`'s markers and `kmlpolygons`' overlap count as the map loop keeps them.
#[derive(Debug, Default)]
pub struct PhotoLayer {
    built_from: Option<Key>,
    photos: Vec<PhotoMarker>,
    coverage: Option<Vec<(LatLon, u32)>>,
}

impl PhotoLayer {
    /// The map loop's camera half: the markers for every shot, and with Camera Overlap on and a
    /// shot to count, the overlap count of the footprints whose roll is under 25. Built again
    /// only when what it is built from has changed.
    /// `// C#: GCSViews/FlightData.cs:4115-4196`
    pub fn refresh(
        &mut self,
        points: &[CameraFeedback],
        min_interval: f64,
        fov: (f64, f64),
        overlap: bool,
        terrain: &dyn Terrain,
    ) {
        let key = Key {
            count: points.len(),
            last_time: points.last().map_or(0, |point| point.time_usec),
            overlap,
            fov: (fov.0.to_bits(), fov.1.to_bits()),
            min_interval: min_interval.to_bits(),
        };
        if self.built_from == Some(key) {
            return;
        }
        self.built_from = Some(key);
        let photos = photo_markers(points, min_interval, fov, terrain);
        // `if (CameraOverlap) { if (... camcount > 0) { kmlpolygons.Markers.Clear(); Add(...) } }`
        self.coverage = (overlap && !photos.is_empty()).then(|| {
            let mut count = OverlapCount::default();
            for photo in &photos {
                if photo.roll.abs() < OVERLAP_ROLL_LIMIT {
                    count.add(photo.time_usec, &photo.marker.footprint);
                }
            }
            count.cells()
        });
        self.photos = photos.into_iter().map(|photo| photo.marker).collect();
    }

    /// `onOffCameraOverlapToolStripMenuItem_Click` with the box unchecked: every photo marker
    /// removed, and put back by the next update, which finds none on the overlay.
    /// `// C#: GCSViews/FlightData.cs:4571-4585`
    pub fn clear(&mut self) {
        self.photos.clear();
        self.coverage = None;
        self.built_from = None;
    }

    /// `photosoverlay.Markers`.
    #[must_use]
    pub fn photos(&self) -> &[PhotoMarker] {
        &self.photos
    }

    /// The overlap count's cells, while it is on the map.
    #[must_use]
    pub fn coverage(&self) -> Option<&[(LatLon, u32)]> {
        self.coverage.as_deref()
    }

    /// What a script can see: the markers, how many draw their footprint, the count's cells and
    /// its greatest count.
    pub fn record_facts(&self) {
        crate::facts::record("fly.photos", self.photos.len());
        crate::facts::record(
            "fly.photos.footprints",
            self.photos
                .iter()
                .filter(|photo| photo.draw_footprint)
                .count(),
        );
        crate::facts::record(
            "fly.coverage.cells",
            self.coverage.as_ref().map_or(0, Vec::len),
        );
        crate::facts::record(
            "fly.coverage.most",
            self.coverage
                .as_ref()
                .and_then(|cells| cells.iter().map(|(_, count)| *count).max())
                .unwrap_or(0),
        );
    }
}

#[cfg(test)]
mod tests {
    use mp_georef::projection::Flat;

    use super::*;

    fn shot(time_usec: u64, img_idx: u16, lat: f64, roll: f32, pitch: f32) -> CameraFeedback {
        #[allow(clippy::cast_possible_truncation)]
        let lat_e7 = (lat * 1e7) as i32;
        CameraFeedback {
            time_usec,
            lat: lat_e7,
            lng: 1_530_251_000,
            alt_msl: 65.2,
            alt_rel: 40.1,
            roll,
            pitch,
            yaw: 0.0,
            foc_len: 0.0,
            img_idx,
            target_system: 1,
            cam_idx: 0,
            flags: 0,
            completed_captures: 0,
        }
    }

    fn square(lat: f64, lng: f64, half: f64) -> Vec<LatLon> {
        [
            (lat + half, lng - half),
            (lat + half, lng + half),
            (lat - half, lng + half),
            (lat - half, lng - half),
        ]
        .into_iter()
        .filter_map(|(a, b)| LatLon::new(a, b).ok())
        .collect()
    }

    /// `IsInside`: a point in a square is inside, one beside it is not, and a line is nothing.
    #[test]
    fn is_inside_is_the_even_odd_rule() {
        let poly = [(1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 1.0)];
        assert!(is_inside(&poly, (1.5, 1.5)));
        assert!(!is_inside(&poly, (1.5, 2.5)));
        assert!(!is_inside(&poly, (2.5, 1.5)));
        assert!(!is_inside(&poly[..2], (1.5, 1.5)));
    }

    /// Two squares that overlap: the cells both hold count 2, the rest 1; a footprint added
    /// again by name counts nothing more.
    #[test]
    fn the_overlap_count_counts_each_cell_for_the_footprints_holding_it() {
        let mut count = OverlapCount::default();
        count.add(1, &square(-27.4700, 153.0250, 0.0003));
        count.add(2, &square(-27.4700, 153.0254, 0.0003));
        count.add(2, &square(-27.4700, 153.0254, 0.0003));
        let cells = count.cells();
        assert!(!cells.is_empty());
        let most = cells.iter().map(|(_, n)| *n).max().unwrap();
        assert_eq!(most, 2);
        assert!(cells.iter().any(|(_, n)| *n == 1));
        // A cell well inside the overlap: both squares hold (-27.4700, 153.0252).
        let shared = cells
            .iter()
            .find(|(at, _)| {
                (at.latitude() + 27.4700).abs() < 1e-9 && (at.longitude() - 153.0252).abs() < 1e-9
            })
            .map(|(_, n)| *n);
        assert_eq!(shared, Some(2));
    }

    /// The markers: one a shot, the icon red for a shot sooner than the interval after the one
    /// before, the footprint drawn for the last four only.
    #[test]
    fn the_markers_follow_the_shots() {
        let points: Vec<CameraFeedback> = (0..6)
            .map(|index| {
                let time =
                    1_000_000 + u64::from(index) * if index == 3 { 500_000 } else { 2_000_000 };
                shot(
                    time,
                    index + 1,
                    -27.47 + f64::from(index) * 0.0001,
                    0.0,
                    0.0,
                )
            })
            .collect();
        let photos = photo_markers(&points, 1.0, DEFAULT_FOV, &Flat);
        assert_eq!(photos.len(), 6);
        assert!(
            !photos[0].marker.below_min_interval,
            "the first is measured from MinValue"
        );
        // Shot 4's time is 2.5 s, before shot 3's 5 s: less than a second on, so below the
        // interval; shot 5's 9 s is 6.5 s on.
        assert!(photos[3].marker.below_min_interval);
        assert!(!photos[4].marker.below_min_interval);
        let drawn: Vec<bool> = photos.iter().map(|p| p.marker.draw_footprint).collect();
        assert_eq!(drawn, [false, false, true, true, true, true]);
        assert_eq!(photos[0].marker.footprint.len(), 4);
        assert!(
            photos[0]
                .marker
                .tooltip
                .starts_with("Photo\nAlt: 65.2\nNo: 1\nRoll: 0.00")
        );
    }

    /// The layer: built once for the same shots, again for a new one; the count only with the
    /// box checked, and only of the footprints rolled under 25; cleared on request.
    #[test]
    fn the_layer_rebuilds_only_when_something_changed() {
        let mut layer = PhotoLayer::default();
        let mut points = vec![
            shot(1_000_000, 1, -27.4700, 0.0, 0.0),
            shot(3_000_000, 2, -27.4702, 30.0, 0.0),
        ];
        layer.refresh(&points, 0.0, DEFAULT_FOV, false, &Flat);
        assert_eq!(layer.photos().len(), 2);
        assert!(layer.coverage().is_none());
        layer.refresh(&points, 0.0, DEFAULT_FOV, true, &Flat);
        let cells = layer.coverage().expect("the count");
        assert!(!cells.is_empty());
        assert!(
            cells.iter().all(|(_, n)| *n == 1),
            "the rolled shot is left out"
        );
        points.push(shot(5_000_000, 3, -27.4700, 0.0, 0.0));
        layer.refresh(&points, 0.0, DEFAULT_FOV, true, &Flat);
        assert_eq!(layer.photos().len(), 3);
        assert!(layer.coverage().unwrap().iter().any(|(_, n)| *n == 2));
        layer.clear();
        assert!(layer.photos().is_empty() && layer.coverage().is_none());
    }

    /// `camera_fovh` saved: both read, a bad number 0; not saved: the statics.
    #[test]
    fn the_fields_of_view_come_from_the_settings() {
        let saved = |key: &str| match key {
            "camera_fovh" => Some("70.5".to_owned()),
            "camera_fovv" => Some("not a number".to_owned()),
            _ => None,
        };
        assert_eq!(fov(saved), (70.5, 0.0));
        assert_eq!(fov(|_| None), (63.0, 43.0));
    }
}
