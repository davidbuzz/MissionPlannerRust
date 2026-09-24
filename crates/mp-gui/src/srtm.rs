//! Terrain heights from SRTM tiles: `srtm.getAltitude`, which the flight map's Set Home Here, Set
//! EKF Origin Here and Point Camera Coords ask for the ground's height at a point.
//!
//! The lookup itself is `mp_terrain` - `ExtLibs/Utilities/srtm.cs` whole, held to Mission
//! Planner's own DLL under mono - over Mission Planner's `srtm` folder, with the download thread
//! running as the C#'s does, so a tile that is not on disk answers `Invalid` until it has been
//! fetched. One lookup for the process, made on first use after MainV2's start-up sweep of the
//! folder. `// C#: MainV2.cs:737-750; ExtLibs/Utilities/srtm.cs:116`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::OnceLock;

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
}

impl AltResponse {
    /// `altresponce.Invalid`.
    pub const INVALID: Self = Self {
        current_type: TileType::Invalid,
        alt: 0.0,
    };
}

impl From<mp_terrain::AltResponse> for AltResponse {
    fn from(answer: mp_terrain::AltResponse) -> Self {
        Self {
            current_type: answer.current_type.into(),
            alt: answer.alt,
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

/// `srtm.getAltitude(lat, lng)`: the height at a point from the tiles in Mission Planner's `srtm`
/// folder, `Invalid` with no data directory or no tile yet.
pub fn altitude(lat: f64, lng: f64) -> AltResponse {
    lookup().map_or(AltResponse::INVALID, |srtm| {
        srtm.get_altitude(lat, lng, ZOOM).into()
    })
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
        let dir = std::env::temp_dir().join(format!("mpr-srtm-{test}-{}", std::process::id()));
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
