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

//! The on-disk tile cache, in Mission Planner's own layout.
//!
//! Tiles are the one thing a ground station downloads that it will want again tomorrow, in a field
//! with no signal. The cache is therefore the primary source and the network is what fills it, not
//! the other way round.
//!
//! The layout is the C# application's, byte for byte, because operators carry multi-gigabyte
//! caches into the field and a cache that had to be migrated is one that would not be:
//!
//! ```text
//! <data directory>/gmapcache/TileDBv3/<language>/<provider>/<zoom>/<y>/<x>.jpg
//! ```
//!
//! `// C#: ExtLibs/Maps/MyImageCache.cs:27-42` (the root) and `:72-74` (the file).
//!
//! Three things about it are not what the file name suggests, and each is deliberate here because
//! each is what makes a tile written by one application readable by the other. The extension is
//! always `.jpg` whatever the bytes are: OpenStreetMap serves PNG and the C# writes it under
//! `.jpg` regardless, then sniffs the format on the way back - `GetImageFromCache` hands the bytes
//! to the image decoder and never looks at the name. The path is `zoom/y/x`, row before column,
//! where every slippy-map tool writes `z/x/y`. And the provider directory is the C# provider's
//! `Name` - `OpenStreetMap`, `GoogleSatelliteMap` - not any identifier of ours.

use mp_os::fs::FsExt as _;
use std::path::{Path, PathBuf};

use mp_units::TileId;

/// `GMapProvider.LanguageStr`, the directory between `TileDBv3` and the provider.
///
/// A constant rather than a setting: it starts as `"en"` and nothing in Mission Planner ever
/// assigns `GMapProvider.Language`, so every cache the C# has written is under `en`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:350-357`
pub const LANGUAGE: &str = "en";

/// The directory under the cache root that holds the tile tree.
/// `// C#: ExtLibs/Maps/MyImageCache.cs:40`
pub const TILE_DB: &str = "TileDBv3";

/// Why a cache operation failed.
#[derive(Debug, thiserror::Error)]
pub enum CacheError {
    /// The filesystem said no.
    #[error("tile cache I/O at {path}: {source}")]
    Io {
        /// What was being touched.
        path: PathBuf,
        /// The underlying error.
        source: std::io::Error,
    },
    /// The bytes offered were not an image.
    #[error("refusing to cache {bytes} bytes that are not a PNG or JPEG")]
    NotAnImage {
        /// How many bytes were offered.
        bytes: usize,
    },
}

/// The image formats a tile may be in.
///
/// Checked on the way in and on the way out, from the bytes and never from the file name - the
/// name is always `.jpg`. A tile server having a bad day answers with an HTML error page and HTTP
/// 200, and a cache that stores it will serve that page as a tile forever.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageFormat {
    /// PNG, what most raster providers serve.
    Png,
    /// JPEG, what aerial imagery is usually in.
    Jpeg,
}

impl ImageFormat {
    /// Identifies an image by its magic bytes.
    #[must_use]
    pub fn sniff(bytes: &[u8]) -> Option<Self> {
        const PNG: &[u8] = &[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
        const JPEG: &[u8] = &[0xFF, 0xD8, 0xFF];
        if bytes.starts_with(PNG) {
            return Some(Self::Png);
        }
        if bytes.starts_with(JPEG) {
            return Some(Self::Jpeg);
        }
        None
    }

    /// The extension this format is conventionally given.
    ///
    /// Not the one used on disk: every cached tile is `.jpg`, see the module documentation. This
    /// is for anything that exports a tile under its own name.
    #[must_use]
    pub const fn extension(self) -> &'static str {
        match self {
            Self::Png => "png",
            Self::Jpeg => "jpg",
        }
    }
}

/// The extension every cached tile has, whatever its bytes are.
/// `// C#: ExtLibs/Maps/MyImageCache.cs:74`
const CACHED_EXTENSION: &str = "jpg";

/// The smallest a real tile can be.
///
/// A 256x256 PNG of one flat colour is a few hundred bytes; anything under this is a truncated
/// download or an error page.
const MINIMUM_TILE_BYTES: usize = 64;

/// A directory of cached tiles.
///
/// Rooted at the `gmapcache` directory - what the C# calls `CacheLocation` before it appends the
/// tile database and language to it.
#[derive(Debug, Clone)]
pub struct TileCache {
    root: PathBuf,
}

impl TileCache {
    /// A cache rooted at a `gmapcache` directory. Created when something is first written.
    #[must_use]
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self { root: root.into() }
    }

    /// Where the C#'s rules put the cache, under this application's own directory name (PLAN.md
    /// section 12, D11): the layout is the C#'s, so a copy of its cache reads here, but the
    /// directory is not the C#'s and the one-shot import does not copy it.
    ///
    /// `%ProgramData%\MissionPlannerRust\gmapcache` on Windows and, on Linux,
    /// `$XDG_DATA_HOME/MissionPlannerRust/gmapcache` (by default under `~/.local/share`), never
    /// `~/MissionPlannerRust`. `MP_TILE_CACHE` overrides it, for tests and for a screenshot that
    /// must not depend on what is cached here.
    #[must_use]
    pub fn default_root() -> PathBuf {
        if let Some(explicit) = std::env::var_os("MP_TILE_CACHE") {
            return PathBuf::from(explicit);
        }
        mp_settings::map_cache_directory()
            // No home directory at all is a strange environment, not a reason to have no cache;
            // the temp directory keeps tiles for the length of the session.
            .unwrap_or_else(|| {
                mp_os::temp_dir()
                    .join("mission-planner-rust")
                    .join("gmapcache")
            })
    }

    /// The `gmapcache` directory this cache lives in.
    #[must_use]
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// The directory the tile tree starts at: `<root>/TileDBv3/en`.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:40-42`
    #[must_use]
    pub fn tile_root(&self) -> PathBuf {
        self.root.join(TILE_DB).join(LANGUAGE)
    }

    /// The file one tile is stored in: `<root>/TileDBv3/en/<provider>/<z>/<y>/<x>.jpg`.
    ///
    /// `provider` is the C# provider's `Name`, which is [`crate::TileSource::cache_name`]. Public
    /// because a test that wants to prove compatibility lays the file out by hand and then checks
    /// this agrees, rather than trusting one function to be consistent with itself.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:72-74`
    #[must_use]
    pub fn path_for(&self, provider: &str, tile: TileId) -> PathBuf {
        self.tile_root()
            .join(provider)
            .join(tile.z.to_string())
            .join(tile.y.to_string())
            .join(format!("{}.{CACHED_EXTENSION}", tile.x))
    }

    /// Reads a tile, or `None` if it is not cached or is not usable.
    ///
    /// A corrupt entry is deleted rather than returned. Recovering silently is right here: the
    /// tile will simply be fetched again, and an operator has no use for being told that one of
    /// several thousand cached images had a bad byte. The C# returns null and leaves the file,
    /// so it fails to decode the same tile on every visit for the rest of time; that is the one
    /// place this diverges from it.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:94-130`
    #[must_use]
    pub fn read(&self, provider: &str, tile: TileId) -> Option<CachedTile> {
        let path = self.path_for(provider, tile);
        let bytes = mp_os::fs::read(&path).ok()?;
        match ImageFormat::sniff(&bytes) {
            Some(format) if bytes.len() >= MINIMUM_TILE_BYTES => Some(CachedTile { bytes, format }),
            _ => {
                // Truncated, empty, or an error page saved as a tile. Remove it so the next
                // request fetches rather than finding the same rubbish again.
                let _ = mp_os::fs::remove_file(&path);
                None
            }
        }
    }

    /// Stores a tile.
    ///
    /// Written to a temporary file and renamed, so a process killed mid-write leaves either the
    /// old tile or the new one, never half of one. A half-written tile is exactly the corrupt
    /// entry `read` then has to clean up. The temporary name ends in `.part`, which neither this
    /// crate nor the C# ever looks for, so a leftover one is invisible rather than served.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:65-92`
    pub fn write(
        &self,
        provider: &str,
        tile: TileId,
        bytes: &[u8],
    ) -> Result<ImageFormat, CacheError> {
        let Some(format) = ImageFormat::sniff(bytes) else {
            return Err(CacheError::NotAnImage { bytes: bytes.len() });
        };
        if bytes.len() < MINIMUM_TILE_BYTES {
            return Err(CacheError::NotAnImage { bytes: bytes.len() });
        }

        let final_path = self.path_for(provider, tile);
        let directory = final_path
            .parent()
            .map_or_else(|| self.tile_root(), Path::to_path_buf);
        mp_os::fs::create_dir_all(&directory).map_err(|source| CacheError::Io {
            path: directory.clone(),
            source,
        })?;

        let temporary = directory.join(format!("{}.{CACHED_EXTENSION}.part", tile.x));
        mp_os::fs::write(&temporary, bytes).map_err(|source| CacheError::Io {
            path: temporary.clone(),
            source,
        })?;
        mp_os::fs::rename(&temporary, &final_path).map_err(|source| {
            let _ = mp_os::fs::remove_file(&temporary);
            CacheError::Io {
                path: final_path.clone(),
                source,
            }
        })?;
        Ok(format)
    }

    /// Whether a tile is cached, without reading it.
    /// `// C#: ExtLibs/Maps/MyImageCache.cs:186-215`
    #[must_use]
    pub fn contains(&self, provider: &str, tile: TileId) -> bool {
        self.path_for(provider, tile).os_is_file()
    }

    /// Total bytes held, and how many tiles that is, across every provider - including ones the
    /// C# application cached and this one has no source for.
    ///
    /// Walks the tree, so it is for a settings screen rather than for every frame.
    #[must_use]
    pub fn usage(&self) -> CacheUsage {
        let mut usage = CacheUsage::default();
        walk(&self.tile_root(), &mut |entry| {
            if let Ok(metadata) = entry.metadata() {
                usage.tiles += 1;
                usage.bytes += metadata.len();
            }
        });
        usage
    }

    /// Deletes every cached tile for one provider.
    pub fn clear(&self, provider: &str) -> Result<(), CacheError> {
        let directory = self.tile_root().join(provider);
        match mp_os::fs::remove_dir_all(&directory) {
            Ok(()) => Ok(()),
            // Nothing cached is not a failure to clear it.
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(source) => Err(CacheError::Io {
                path: directory,
                source,
            }),
        }
    }
}

/// A tile as it was stored.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CachedTile {
    /// The encoded image.
    pub bytes: Vec<u8>,
    /// What it is encoded as - from the bytes, since the name always says JPEG.
    pub format: ImageFormat,
}

/// How much the cache is holding.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CacheUsage {
    /// Number of tiles.
    pub tiles: u64,
    /// Total size on disk.
    pub bytes: u64,
}

/// Visits every file under a directory.
fn walk(directory: &Path, visit: &mut impl FnMut(&mp_os::fs::DirEntry)) {
    let Ok(entries) = mp_os::fs::read_dir(directory) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.os_is_dir() {
            walk(&path, visit);
        } else {
            visit(&entry);
        }
    }
}

#[cfg(test)]
mod tests {
    use mp_os::fs::FsExt as _;
    use super::*;

    /// The C# provider name for OpenStreetMap, as the tests file tiles under it.
    const OSM: &str = "OpenStreetMap";

    /// A temporary directory that removes itself, so tests leave nothing behind.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = mp_os::temp_dir().join(format!(
                "mp-tiles-{name}-{}-{:?}",
                mp_os::process_id(),
                wasm_thread::current().id()
            ));
            let _ = mp_os::fs::remove_dir_all(&path);
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = mp_os::fs::remove_dir_all(&self.0);
        }
    }

    fn png(size: usize) -> Vec<u8> {
        let mut bytes = vec![0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
        bytes.resize(size.max(8), 0x42);
        bytes
    }

    fn jpeg(size: usize) -> Vec<u8> {
        let mut bytes = vec![0xFF, 0xD8, 0xFF];
        bytes.resize(size.max(3), 0x42);
        bytes
    }

    fn tile() -> TileId {
        TileId::new(14, 15_089, 9_814).expect("a valid tile")
    }

    #[test]
    fn the_layout_is_mission_planners_zoom_then_row_then_column_under_jpg() {
        // `CacheLocation + sep + Name + sep + zoom + sep + pos.Y + sep + pos.X + ".jpg"`, with
        // CacheLocation already `gmapcache/TileDBv3/en/`.
        // C#: ExtLibs/Maps/MyImageCache.cs:72-74
        let cache = TileCache::new("/tmp/gmapcache");
        assert_eq!(
            cache.path_for(OSM, tile()),
            PathBuf::from("/tmp/gmapcache/TileDBv3/en/OpenStreetMap/14/9814/15089.jpg")
        );
    }

    #[test]
    fn a_png_is_filed_under_jpg_because_that_is_what_the_csharp_reads() {
        // OpenStreetMap serves PNG. The C# writes every tile as `<x>.jpg` and sniffs the bytes on
        // the way back, so a PNG written under `.png` would be invisible to it.
        let scratch = Scratch::new("png-as-jpg");
        let cache = TileCache::new(&scratch.0);
        let format = cache.write(OSM, tile(), &png(512)).expect("should write");
        assert_eq!(
            format,
            ImageFormat::Png,
            "the format is reported from the bytes"
        );

        let on_disk = scratch
            .0
            .join("TileDBv3/en/OpenStreetMap/14/9814/15089.jpg");
        assert!(on_disk.os_is_file(), "{}", on_disk.display());
        assert!(
            !scratch
                .0
                .join("TileDBv3/en/OpenStreetMap/14/9814/15089.png")
                .exists()
        );

        let read = cache.read(OSM, tile()).expect("should read back");
        assert_eq!(read.format, ImageFormat::Png);
        assert_eq!(read.bytes, png(512));
    }

    #[test]
    fn a_written_tile_reads_back_byte_for_byte() {
        let scratch = Scratch::new("round-trip");
        let cache = TileCache::new(&scratch.0);
        let bytes = jpeg(512);
        assert_eq!(
            cache.write(OSM, tile(), &bytes).expect("write"),
            ImageFormat::Jpeg
        );
        let read = cache.read(OSM, tile()).expect("should read back");
        assert_eq!(read.bytes, bytes);
        assert_eq!(read.format, ImageFormat::Jpeg);
    }

    #[test]
    fn a_tile_that_was_never_written_is_a_miss() {
        let scratch = Scratch::new("miss");
        let cache = TileCache::new(&scratch.0);
        assert!(cache.read(OSM, tile()).is_none());
        assert!(!cache.contains(OSM, tile()));
    }

    #[test]
    fn providers_do_not_share_a_tile() {
        // Same coordinates, different imagery. Mixing them would put street tiles in a satellite
        // view and be nearly impossible to diagnose.
        let scratch = Scratch::new("providers");
        let cache = TileCache::new(&scratch.0);
        cache.write(OSM, tile(), &png(512)).expect("write osm");
        assert!(cache.contains(OSM, tile()));
        assert!(!cache.contains("GoogleSatelliteMap", tile()));
    }

    #[test]
    fn an_error_page_is_refused_rather_than_cached_as_a_tile() {
        // A tile server having a bad day answers with HTML and HTTP 200. Cached, it would be
        // served as that tile forever.
        let scratch = Scratch::new("error-page");
        let cache = TileCache::new(&scratch.0);
        let html = b"<!DOCTYPE html><html><body>503 Service Unavailable</body></html>";
        let refused = cache.write(OSM, tile(), html);
        assert!(matches!(refused, Err(CacheError::NotAnImage { .. })));
        assert!(!cache.contains(OSM, tile()));
    }

    #[test]
    fn a_truncated_download_is_refused() {
        let scratch = Scratch::new("truncated");
        let cache = TileCache::new(&scratch.0);
        // Correct magic bytes, far too short to be an image.
        assert!(matches!(
            cache.write(OSM, tile(), &png(8)),
            Err(CacheError::NotAnImage { .. })
        ));
    }

    #[test]
    fn a_corrupt_entry_is_removed_and_reported_as_a_miss() {
        // Recovery rather than an error: the tile is simply fetched again. The C# would return
        // null and leave the file, and fail on it again next time.
        let scratch = Scratch::new("corrupt");
        let cache = TileCache::new(&scratch.0);
        cache.write(OSM, tile(), &png(512)).expect("write");

        // Corrupt it behind the cache's back, as a half-finished write or a bad disk would.
        let path = cache.path_for(OSM, tile());
        mp_os::fs::write(&path, b"nonsense").expect("overwrite");

        assert!(cache.read(OSM, tile()).is_none());
        assert!(!path.os_exists(), "the corrupt entry should have been removed");
    }

    #[test]
    fn an_empty_file_is_treated_as_corrupt() {
        let scratch = Scratch::new("empty");
        let cache = TileCache::new(&scratch.0);
        let path = cache.path_for(OSM, tile());
        mp_os::fs::create_dir_all(path.parent().expect("a parent")).expect("mkdir");
        mp_os::fs::write(&path, b"").expect("write empty");

        assert!(cache.read(OSM, tile()).is_none());
        assert!(!path.os_exists());
    }

    #[test]
    fn writing_leaves_no_partial_files_behind() {
        // The temporary file is renamed into place; a .part left in the tree would be counted by
        // usage() and copied into the field with everything else.
        let scratch = Scratch::new("atomic");
        let cache = TileCache::new(&scratch.0);
        cache.write(OSM, tile(), &png(512)).expect("write");

        let mut names = Vec::new();
        walk(&scratch.0, &mut |entry| {
            names.push(entry.file_name().to_string_lossy().into_owned());
        });
        assert_eq!(names, vec!["15089.jpg".to_owned()], "{names:?}");
    }

    #[test]
    fn rewriting_a_tile_replaces_it() {
        let scratch = Scratch::new("replace");
        let cache = TileCache::new(&scratch.0);
        cache.write(OSM, tile(), &png(512)).expect("first");
        let updated = png(1024);
        cache.write(OSM, tile(), &updated).expect("second");

        assert_eq!(cache.read(OSM, tile()).expect("read").bytes, updated);
        assert_eq!(cache.usage().tiles, 1, "the old tile should not linger");
    }

    #[test]
    fn usage_counts_what_is_there() {
        let scratch = Scratch::new("usage");
        let cache = TileCache::new(&scratch.0);
        assert_eq!(cache.usage(), CacheUsage::default());

        for x in 0..3 {
            let id = TileId::new(10, x, 5).expect("a valid tile");
            cache.write(OSM, id, &png(512)).expect("write");
        }
        let usage = cache.usage();
        assert_eq!(usage.tiles, 3);
        assert_eq!(usage.bytes, 3 * 512);
    }

    #[test]
    fn usage_includes_what_the_other_application_cached() {
        // A provider this crate has no source for is still in the tree the operator carries.
        let scratch = Scratch::new("usage-foreign");
        let cache = TileCache::new(&scratch.0);
        cache
            .write("GoogleSatelliteMap", tile(), &jpeg(700))
            .expect("write");
        assert_eq!(cache.usage().tiles, 1);
        assert_eq!(cache.usage().bytes, 700);
    }

    #[test]
    fn clearing_one_provider_leaves_the_others() {
        let scratch = Scratch::new("clear");
        let cache = TileCache::new(&scratch.0);
        cache.write(OSM, tile(), &png(512)).expect("osm");
        cache
            .write("GoogleSatelliteMap", tile(), &jpeg(512))
            .expect("google");

        cache.clear(OSM).expect("clear");
        assert!(!cache.contains(OSM, tile()));
        assert!(cache.contains("GoogleSatelliteMap", tile()));
    }

    #[test]
    fn clearing_what_was_never_cached_is_not_an_error() {
        let scratch = Scratch::new("clear-empty");
        let cache = TileCache::new(&scratch.0);
        assert!(cache.clear(OSM).is_ok());
    }

    #[test]
    fn magic_bytes_identify_the_format_and_reject_anything_else() {
        assert_eq!(ImageFormat::sniff(&png(64)), Some(ImageFormat::Png));
        assert_eq!(ImageFormat::sniff(&jpeg(64)), Some(ImageFormat::Jpeg));
        assert_eq!(ImageFormat::sniff(b"GIF89a"), None);
        assert_eq!(ImageFormat::sniff(b"<!DOCTYPE html>"), None);
        assert_eq!(ImageFormat::sniff(b""), None);
    }

    #[test]
    fn the_default_root_is_the_csharp_applications_gmapcache() {
        // Unless a test or a screenshot points it elsewhere, which this test cannot tell from
        // here; either way it ends in the directory name the C# uses.
        let root = TileCache::default_root();
        let shown = root.display().to_string();
        assert!(
            std::env::var_os("MP_TILE_CACHE").is_some() || shown.ends_with("gmapcache"),
            "{shown}"
        );
    }

    #[test]
    fn the_language_directory_is_the_one_the_csharp_never_changes() {
        assert_eq!(LANGUAGE, "en");
        assert_eq!(TILE_DB, "TileDBv3");
        assert!(TileCache::new("/x").tile_root().ends_with("TileDBv3/en"));
    }
}
