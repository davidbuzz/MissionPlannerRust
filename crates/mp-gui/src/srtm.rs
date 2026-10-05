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

//! Terrain heights from SRTM tiles: `srtm.getAltitude`, which the flight map's Set Home Here, Set
//! EKF Origin Here and Point Camera Coords ask for the ground's height at a point, and the
//! planning screen's Verify Height, home from the map and Elevation Graph.
//!
//! The lookup itself is `mp_terrain` - `ExtLibs/Utilities/srtm.cs` whole, held to Mission
//! Planner's own DLL under mono - over Mission Planner's `srtm` folder, with the download thread
//! running as the C#'s does, so a tile that is not on disk answers `Invalid` until it has been
//! fetched - unless the map is cache-only, when it is never fetched. One lookup for the process,
//! made on first use after MainV2's start-up sweep of the folder.
//! `// C#: MainV2.cs:737-750; ExtLibs/Utilities/srtm.cs:116, 385`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, Ordering};

/// `srtm.tiletype`. `// C#: ExtLibs/Utilities/srtm.cs:23-28`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TileType {
    /// A height from a tile.
    Valid,
    /// No height: no tile, a tile being fetched, or a hole in the data.
    Invalid,
    /// The server has no tile here because it is sea; Set Home Here still takes it, as the C#
    /// does.
    Ocean,
}

impl From<mp_terrain::TileType> for TileType {
    fn from(kind: mp_terrain::TileType) -> Self {
        match kind {
            mp_terrain::TileType::Valid => Self::Valid,
            mp_terrain::TileType::Invalid => Self::Invalid,
            mp_terrain::TileType::Ocean => Self::Ocean,
        }
    }
}

/// `srtm.altresponce`: what kind of answer, and the height, metres above sea level - 0 unless
/// valid. `// C#: ExtLibs/Utilities/srtm.cs:30-38`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltResponse {
    /// `currenttype`.
    pub current_type: TileType,
    /// `alt`.
    pub alt: f64,
    /// `altsource`: "SRTM", "ASCII", "Invalid", "Ocean", or "" - what `coords1` shows under
    /// the height.
    pub alt_source: &'static str,
}

impl AltResponse {
    /// `altresponce.Invalid`.
    pub const INVALID: Self = Self {
        current_type: TileType::Invalid,
        alt: 0.0,
        alt_source: "Invalid",
    };
}

impl From<mp_terrain::AltResponse> for AltResponse {
    fn from(answer: mp_terrain::AltResponse) -> Self {
        Self {
            current_type: answer.current_type.into(),
            alt: answer.alt,
            alt_source: answer.alt_source,
        }
    }
}

/// `getAltitude`'s default zoom, at which a missing tile is queued for download.
/// `// C#: ExtLibs/Utilities/srtm.cs:116, 380`
const ZOOM: f64 = 16.0;

/// The process's lookup over Mission Planner's `srtm` folder, or `None` with no data directory.
fn lookup() -> Option<&'static mp_terrain::Srtm> {
    static LOOKUP: OnceLock<Option<mp_terrain::Srtm>> = OnceLock::new();
    LOOKUP
        .get_or_init(|| {
            let dir = mp_terrain::srtm_directory()?;
            // MainV2's sweep of empty and stale files, once, before the first lookup.
            mp_terrain::clean_cache_directory(&dir);
            Some(mp_terrain::Srtm::new(dir))
        })
        .as_ref()
}

/// `GMaps.Instance.Mode == AccessMode.CacheOnly`: the map's access mode, which Mission Planner
/// sets from the Planner page's Map Access Mode (`mapCache`) at start-up and whenever the page
/// changes it, and which `srtm.getAltitude` reads at every lookup to decide whether a missing
/// tile is queued for download.
/// `// C#: Program.cs:321-325; GCSViews/ConfigurationView/ConfigPlanner.cs:1161-1167; ExtLibs/Utilities/srtm.cs:385`
static CACHE_ONLY: AtomicBool = AtomicBool::new(false);

/// Sets the map's access mode as the terrain lookup sees it: `true` for `CacheOnly`, when a tile
/// that is not on disk is never fetched.
pub fn set_cache_only(cache_only: bool) {
    CACHE_ONLY.store(cache_only, Ordering::Release);
}

/// Whether the terrain lookup is cache-only, as [`set_cache_only`] last left it.
#[must_use]
pub fn cache_only() -> bool {
    CACHE_ONLY.load(Ordering::Acquire)
}

/// One `getAltitude` on a lookup with the access mode as it is now: the C# reads
/// `GMaps.Instance.Mode` inside the lookup, so a mode changed between two lookups governs the
/// second. `// C#: ExtLibs/Utilities/srtm.cs:116, 378-398`
fn ask(srtm: &mp_terrain::Srtm, lat: f64, lng: f64) -> AltResponse {
    srtm.set_cache_only(cache_only());
    srtm.get_altitude(lat, lng, ZOOM).into()
}

/// `srtm.getAltitude(lat, lng)`: the height at a point from the tiles in Mission Planner's `srtm`
/// folder, `Invalid` with no data directory or no tile yet.
pub fn altitude(lat: f64, lng: f64) -> AltResponse {
    lookup().map_or(AltResponse::INVALID, |srtm| ask(srtm, lat, lng))
}

/// The same lookup over a given folder, with nothing downloaded: what a test's own tile answers.
#[cfg(test)]
pub fn altitude_in(directory: &std::path::Path, lat: f64, lng: f64) -> AltResponse {
    let srtm = mp_terrain::Srtm::new(directory);
    srtm.set_cache_only(true);
    srtm.get_altitude(lat, lng, ZOOM).into()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tile written as a function of its column and row, in a scratch folder.
    fn tile(
        test: &str,
        name: &str,
        size: usize,
        height: impl Fn(usize, usize) -> i16,
    ) -> std::path::PathBuf {
        let dir = mp_os::temp_dir().join(format!(
            "headless-planner-srtm-{test}-{}",
            mp_os::process_id()
        ));
        std::fs::create_dir_all(&dir).expect("scratch folder");
        let mut bytes = Vec::with_capacity(size * size * 2);
        for row in 0..size {
            for column in 0..size {
                bytes.extend_from_slice(&height(column, row).to_be_bytes());
            }
        }
        std::fs::write(dir.join(name), bytes).expect("write the tile");
        dir
    }

    /// The map's access mode reaches the lookup at the lookup, as `GMaps.Instance.Mode` does: a
    /// missing tile is queued for the download thread while the mode is not `CacheOnly` and not
    /// while it is, and a tile on disk answers either way. (No thread runs, so nothing is
    /// fetched.)
    #[test]
    fn the_access_mode_decides_whether_a_missing_tile_is_queued() {
        let dir = tile("cacheonly", "S36E149.hgt", 1201, |_, _| 584);
        let srtm = mp_terrain::Srtm::without_thread(
            &dir,
            std::sync::Arc::new(mp_terrain::UreqHttp::new()),
        );

        set_cache_only(true);
        assert!(cache_only());
        assert_eq!(ask(&srtm, 51.5, -0.12), AltResponse::INVALID);
        assert!(srtm.queued().is_empty(), "{:?}", srtm.queued());
        let on_disk = ask(&srtm, -35.363_262_1, 149.165_237_4);
        assert_eq!(on_disk.current_type, TileType::Valid);
        assert!((on_disk.alt - 584.0).abs() < 1e-9, "{on_disk:?}");

        set_cache_only(false);
        assert!(!cache_only());
        assert_eq!(ask(&srtm, 51.5, -0.12), AltResponse::INVALID);
        assert_eq!(srtm.queued(), vec!["N51W001.hgt".to_owned()]);
    }

    /// The wrapper hands back what `mp_terrain` answers, in this module's types: a flat tile's
    /// height inside it, Invalid outside any tile.
    #[test]
    fn a_folder_lookup_answers_from_its_tile_and_invalid_elsewhere() {
        let dir = tile("wrapper", "S36E149.hgt", 1201, |_, _| 584);
        let inside = altitude_in(&dir, -35.363_262_1, 149.165_237_4);
        assert_eq!(inside.current_type, TileType::Valid);
        assert!((inside.alt - 584.0).abs() < 1e-9, "{inside:?}");
        let outside = altitude_in(&dir, 51.5, -0.12);
        assert_eq!(outside, AltResponse::INVALID);
    }
}
