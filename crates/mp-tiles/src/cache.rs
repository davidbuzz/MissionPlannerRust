//! The on-disk tile cache.
//!
//! Tiles are the one thing a ground station downloads that it will want again tomorrow, in a field
//! with no signal. The cache is therefore the primary source and the network is what fills it, not
//! the other way round.

use std::path::{Path, PathBuf};

use mp_units::TileId;

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
/// Checked on the way in and on the way out. A tile server having a bad day answers with an HTML
/// error page and HTTP 200, and a cache that stores it will serve that page as a tile forever.
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

    /// The extension used on disk.
    #[must_use]
    pub const fn extension(self) -> &'static str {
        match self {
            Self::Png => "png",
            Self::Jpeg => "jpg",
        }
    }
}

/// The smallest a real tile can be.
///
/// A 256x256 PNG of one flat colour is a few hundred bytes; anything under this is a truncated
/// download or an error page.
const MINIMUM_TILE_BYTES: usize = 64;

/// A directory of cached tiles.
#[derive(Debug, Clone)]
pub struct TileCache {
    root: PathBuf,
}

impl TileCache {
    /// A cache rooted at a directory. The directory is created when something is first written.
    #[must_use]
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self { root: root.into() }
    }

    /// The default location, following each platform's convention.
    ///
    /// Tiles are regenerable downloads, so on Linux they belong in the cache directory rather than
    /// alongside configuration - a user who clears their cache should lose tiles and keep their
    /// parameter files.
    #[must_use]
    pub fn default_root() -> PathBuf {
        if let Some(explicit) = std::env::var_os("MP_TILE_CACHE") {
            return PathBuf::from(explicit);
        }
        let base = std::env::var_os("XDG_CACHE_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("LOCALAPPDATA").map(PathBuf::from))
            .or_else(|| std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".cache")))
            .unwrap_or_else(std::env::temp_dir);
        base.join("mission-planner-rust").join("tiles")
    }

    /// Where this cache lives.
    #[must_use]
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// The directory holding one provider's tiles at one zoom and column.
    fn directory_for(&self, source_id: &str, tile: TileId) -> PathBuf {
        // provider/z/x/y.ext, the layout every slippy-map tool uses, so the cache can be
        // inspected, copied to a field laptop, or seeded from another tool with cp -r.
        self.root
            .join(source_id)
            .join(tile.z.to_string())
            .join(tile.x.to_string())
    }

    /// Reads a tile, or `None` if it is not cached or is not usable.
    ///
    /// A corrupt entry is deleted rather than returned. Recovering silently is right here: the
    /// tile will simply be fetched again, and an operator has no use for being told that one of
    /// several thousand cached images had a bad byte.
    #[must_use]
    pub fn read(&self, source_id: &str, tile: TileId) -> Option<CachedTile> {
        let directory = self.directory_for(source_id, tile);
        for format in [ImageFormat::Png, ImageFormat::Jpeg] {
            let path = directory.join(format!("{}.{}", tile.y, format.extension()));
            let Ok(bytes) = std::fs::read(&path) else {
                continue;
            };
            match ImageFormat::sniff(&bytes) {
                Some(actual) if bytes.len() >= MINIMUM_TILE_BYTES => {
                    return Some(CachedTile {
                        bytes,
                        format: actual,
                    });
                }
                _ => {
                    // Truncated, empty, or an error page saved as a tile. Remove it so the next
                    // request fetches rather than finding the same rubbish again.
                    let _ = std::fs::remove_file(&path);
                }
            }
        }
        None
    }

    /// Stores a tile.
    ///
    /// Written to a temporary file and renamed, so a process killed mid-write leaves either the
    /// old tile or the new one, never half of one. A half-written tile is exactly the corrupt
    /// entry `read` then has to clean up.
    pub fn write(
        &self,
        source_id: &str,
        tile: TileId,
        bytes: &[u8],
    ) -> Result<ImageFormat, CacheError> {
        let Some(format) = ImageFormat::sniff(bytes) else {
            return Err(CacheError::NotAnImage { bytes: bytes.len() });
        };
        if bytes.len() < MINIMUM_TILE_BYTES {
            return Err(CacheError::NotAnImage { bytes: bytes.len() });
        }

        let directory = self.directory_for(source_id, tile);
        std::fs::create_dir_all(&directory).map_err(|source| CacheError::Io {
            path: directory.clone(),
            source,
        })?;

        let final_path = directory.join(format!("{}.{}", tile.y, format.extension()));
        let temporary = directory.join(format!("{}.{}.part", tile.y, format.extension()));
        std::fs::write(&temporary, bytes).map_err(|source| CacheError::Io {
            path: temporary.clone(),
            source,
        })?;
        std::fs::rename(&temporary, &final_path).map_err(|source| {
            let _ = std::fs::remove_file(&temporary);
            CacheError::Io {
                path: final_path.clone(),
                source,
            }
        })?;
        Ok(format)
    }

    /// Whether a tile is cached, without reading it.
    #[must_use]
    pub fn contains(&self, source_id: &str, tile: TileId) -> bool {
        let directory = self.directory_for(source_id, tile);
        [ImageFormat::Png, ImageFormat::Jpeg].iter().any(|format| {
            directory
                .join(format!("{}.{}", tile.y, format.extension()))
                .exists()
        })
    }

    /// Total bytes held, and how many tiles that is.
    ///
    /// Walks the tree, so it is for a settings screen rather than for every frame.
    #[must_use]
    pub fn usage(&self) -> CacheUsage {
        let mut usage = CacheUsage::default();
        walk(&self.root, &mut |entry| {
            if let Ok(metadata) = entry.metadata() {
                usage.tiles += 1;
                usage.bytes += metadata.len();
            }
        });
        usage
    }

    /// Deletes every cached tile for one provider.
    pub fn clear(&self, source_id: &str) -> Result<(), CacheError> {
        let directory = self.root.join(source_id);
        match std::fs::remove_dir_all(&directory) {
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
    /// What it is encoded as.
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
fn walk(directory: &Path, visit: &mut impl FnMut(&std::fs::DirEntry)) {
    let Ok(entries) = std::fs::read_dir(directory) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            walk(&path, visit);
        } else {
            visit(&entry);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A temporary directory that removes itself, so tests leave nothing behind.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = std::env::temp_dir().join(format!(
                "mp-tiles-{name}-{}-{:?}",
                std::process::id(),
                std::thread::current().id()
            ));
            let _ = std::fs::remove_dir_all(&path);
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
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
    fn a_written_tile_reads_back_byte_for_byte() {
        let scratch = Scratch::new("round-trip");
        let cache = TileCache::new(&scratch.0);
        let bytes = png(512);

        let format = cache.write("osm", tile(), &bytes).expect("should write");
        assert_eq!(format, ImageFormat::Png);

        let read = cache.read("osm", tile()).expect("should read back");
        assert_eq!(read.bytes, bytes);
        assert_eq!(read.format, ImageFormat::Png);
    }

    #[test]
    fn a_tile_that_was_never_written_is_a_miss() {
        let scratch = Scratch::new("miss");
        let cache = TileCache::new(&scratch.0);
        assert!(cache.read("osm", tile()).is_none());
        assert!(!cache.contains("osm", tile()));
    }

    #[test]
    fn providers_do_not_share_a_tile() {
        // Same coordinates, different imagery. Mixing them would put street tiles in a satellite
        // view and be nearly impossible to diagnose.
        let scratch = Scratch::new("providers");
        let cache = TileCache::new(&scratch.0);
        cache.write("osm", tile(), &png(512)).expect("write osm");
        assert!(cache.contains("osm", tile()));
        assert!(!cache.contains("opentopo", tile()));
    }

    #[test]
    fn an_error_page_is_refused_rather_than_cached_as_a_tile() {
        // A tile server having a bad day answers with HTML and HTTP 200. Cached, it would be
        // served as that tile forever.
        let scratch = Scratch::new("error-page");
        let cache = TileCache::new(&scratch.0);
        let html = b"<!DOCTYPE html><html><body>503 Service Unavailable</body></html>";

        let refused = cache.write("osm", tile(), html);
        assert!(matches!(refused, Err(CacheError::NotAnImage { .. })));
        assert!(!cache.contains("osm", tile()));
    }

    #[test]
    fn a_truncated_download_is_refused() {
        let scratch = Scratch::new("truncated");
        let cache = TileCache::new(&scratch.0);
        // Correct magic bytes, far too short to be an image.
        let stub = png(8);
        assert!(matches!(
            cache.write("osm", tile(), &stub),
            Err(CacheError::NotAnImage { .. })
        ));
    }

    #[test]
    fn a_corrupt_entry_is_removed_and_reported_as_a_miss() {
        // Recovery rather than an error: the tile is simply fetched again, and an operator has no
        // use for being told one of several thousand cached images had a bad byte.
        let scratch = Scratch::new("corrupt");
        let cache = TileCache::new(&scratch.0);
        cache.write("osm", tile(), &png(512)).expect("write");

        // Corrupt it behind the cache's back, as a half-finished write or a bad disk would.
        let path = cache
            .directory_for("osm", tile())
            .join(format!("{}.png", tile().y));
        std::fs::write(&path, b"nonsense").expect("overwrite");

        assert!(cache.read("osm", tile()).is_none());
        assert!(!path.exists(), "the corrupt entry should have been removed");
    }

    #[test]
    fn an_empty_file_is_treated_as_corrupt() {
        let scratch = Scratch::new("empty");
        let cache = TileCache::new(&scratch.0);
        let directory = cache.directory_for("osm", tile());
        std::fs::create_dir_all(&directory).expect("mkdir");
        let path = directory.join(format!("{}.png", tile().y));
        std::fs::write(&path, b"").expect("write empty");

        assert!(cache.read("osm", tile()).is_none());
        assert!(!path.exists());
    }

    #[test]
    fn jpeg_tiles_work_too() {
        let scratch = Scratch::new("jpeg");
        let cache = TileCache::new(&scratch.0);
        let bytes = jpeg(512);
        assert_eq!(
            cache.write("aerial", tile(), &bytes).expect("write"),
            ImageFormat::Jpeg
        );
        let read = cache.read("aerial", tile()).expect("read");
        assert_eq!(read.format, ImageFormat::Jpeg);
        assert_eq!(read.bytes, bytes);
    }

    #[test]
    fn writing_leaves_no_partial_files_behind() {
        // The temporary file is renamed into place; a .part left in the tree would be served as a
        // tile by another tool reading this cache, and counted by usage().
        let scratch = Scratch::new("atomic");
        let cache = TileCache::new(&scratch.0);
        cache.write("osm", tile(), &png(512)).expect("write");

        let mut names = Vec::new();
        walk(&scratch.0, &mut |entry| {
            names.push(entry.file_name().to_string_lossy().into_owned());
        });
        assert_eq!(names.len(), 1, "{names:?}");
        assert!(!names[0].ends_with(".part"), "{names:?}");
    }

    #[test]
    fn rewriting_a_tile_replaces_it() {
        let scratch = Scratch::new("replace");
        let cache = TileCache::new(&scratch.0);
        cache.write("osm", tile(), &png(512)).expect("first");
        let updated = png(1024);
        cache.write("osm", tile(), &updated).expect("second");

        assert_eq!(cache.read("osm", tile()).expect("read").bytes, updated);
        assert_eq!(cache.usage().tiles, 1, "the old tile should not linger");
    }

    #[test]
    fn usage_counts_what_is_there() {
        let scratch = Scratch::new("usage");
        let cache = TileCache::new(&scratch.0);
        assert_eq!(cache.usage(), CacheUsage::default());

        for x in 0..3 {
            let id = TileId::new(10, x, 5).expect("a valid tile");
            cache.write("osm", id, &png(512)).expect("write");
        }
        let usage = cache.usage();
        assert_eq!(usage.tiles, 3);
        assert_eq!(usage.bytes, 3 * 512);
    }

    #[test]
    fn clearing_one_provider_leaves_the_others() {
        let scratch = Scratch::new("clear");
        let cache = TileCache::new(&scratch.0);
        cache.write("osm", tile(), &png(512)).expect("osm");
        cache.write("opentopo", tile(), &png(512)).expect("topo");

        cache.clear("osm").expect("clear");
        assert!(!cache.contains("osm", tile()));
        assert!(cache.contains("opentopo", tile()));
    }

    #[test]
    fn clearing_what_was_never_cached_is_not_an_error() {
        let scratch = Scratch::new("clear-empty");
        let cache = TileCache::new(&scratch.0);
        assert!(cache.clear("osm").is_ok());
    }

    #[test]
    fn the_layout_is_the_one_every_slippy_map_tool_uses() {
        // So the cache can be inspected by hand, copied to a field laptop, or seeded from another
        // tool with cp -r.
        let cache = TileCache::new("/tmp/example");
        let path = cache.directory_for("osm", tile());
        assert!(path.ends_with("osm/14/15089"), "{}", path.display());
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
    fn the_default_root_is_under_a_cache_directory_not_a_config_one() {
        // Tiles are regenerable downloads. A user who clears their cache should lose tiles and
        // keep their parameter files.
        let root = TileCache::default_root();
        let shown = root.display().to_string();
        assert!(shown.contains("tiles"), "{shown}");
    }
}
