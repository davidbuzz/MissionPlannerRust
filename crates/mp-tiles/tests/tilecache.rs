//! Compatibility with the tile cache the C# application writes (DELIVERABLES.md D8).
//!
//! The claim is specific: a tile Mission Planner cached is read here with the network off, and a
//! tile cached here is found by Mission Planner. Both directions are proved against the layout
//! written out by hand from `ExtLibs/Maps/MyImageCache.cs:72-74`, not through `path_for`, so a
//! change that broke the layout while keeping the crate consistent with itself would fail here.
//!
//! The last test goes further when it can: it reads a tile the real C# application wrote, from
//! the real cache on the machine running the test, and skips with a message when there is none.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use mp_tiles::cache::{ImageFormat, TileCache};
use mp_tiles::source::{GOOGLE_SATELLITE_MAP, OPENSTREETMAP};
use mp_tiles::store::{DecodedTile, TileAnswer, TileStore};
use mp_units::TileId;

/// A temporary directory that removes itself.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let path = std::env::temp_dir().join(format!(
            "mp-tiles-compat-{name}-{}-{:?}",
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

/// A real 256x256 PNG, the size a tile is, so the decoder is exercised on the shape it will see.
fn tile_png() -> Vec<u8> {
    let mut buffer = std::io::Cursor::new(Vec::new());
    let image = image::RgbaImage::from_pixel(256, 256, image::Rgba([0x7a, 0x9e, 0x5c, 255]));
    image
        .write_to(&mut buffer, image::ImageFormat::Png)
        .expect("encoding a png should work");
    buffer.into_inner()
}

/// The file the C# writes for a tile, spelled out from the source rather than computed.
///
/// `CacheLocation + sep + Name + sep + zoom + sep + pos.Y + sep + pos.X + ".jpg"`, where
/// `CacheLocation` is `gmapcache/TileDBv3/en/`. Note `.jpg` regardless of the bytes, and Y before X.
/// C#: ExtLibs/Maps/MyImageCache.cs:40-42, 72-74
fn csharp_path(gmapcache: &Path, provider: &str, z: u8, x: u32, y: u32) -> PathBuf {
    gmapcache.join(format!("TileDBv3/en/{provider}/{z}/{y}/{x}.jpg"))
}

/// Waits for the store's worker to have read something, bounded so a broken worker is a
/// failure and not a hang.
fn wait_for_a_tile(store: &TileStore) {
    let deadline = Instant::now() + Duration::from_secs(2);
    while store.generation() == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn a_tile_the_csharp_application_wrote_is_read_with_the_network_off() {
    let scratch = Scratch::new("csharp-wrote");
    // OpenStreetMap tiles are PNG; the C# files them under .jpg. The SITL home field at zoom 16.
    let path = csharp_path(&scratch.0, "OpenStreetMap", 16, 59_922, 39_658);
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(&path, tile_png()).unwrap();

    let cache = TileCache::new(&scratch.0);
    let subject = TileId::new(16, 59_922, 39_658).unwrap();
    let found = cache
        .read(OPENSTREETMAP.cache_name, subject)
        .expect("the tile the C# wrote should be found");
    assert_eq!(
        found.format,
        ImageFormat::Png,
        "the format comes from the bytes, not the name"
    );

    // And through the store, on the path the map takes, with nothing to fetch from.
    let store = TileStore::offline(&OPENSTREETMAP, cache);
    assert!(
        matches!(store.get(subject), TileAnswer::Missing),
        "first ask queues it"
    );
    wait_for_a_tile(&store);
    match store.get(subject) {
        TileAnswer::Exact(tile) => {
            assert_eq!((tile.width, tile.height), (256, 256));
        }
        other => panic!(
            "expected the cached tile, got {other:?} with {:?}",
            store.stats()
        ),
    }
    assert_eq!(store.stats().fetched, 0, "the network was never consulted");
}

#[test]
fn a_tile_cached_here_is_where_the_csharp_application_looks_for_it() {
    let scratch = Scratch::new("we-wrote");
    let cache = TileCache::new(&scratch.0);
    let subject = TileId::new(16, 59_922, 39_658).unwrap();
    cache
        .write(OPENSTREETMAP.cache_name, subject, &tile_png())
        .expect("write");

    let expected = csharp_path(&scratch.0, "OpenStreetMap", 16, 59_922, 39_658);
    assert!(expected.is_file(), "not at {}", expected.display());
    assert_eq!(
        std::fs::read(&expected).unwrap(),
        tile_png(),
        "byte for byte"
    );
    // Nothing else was written that the C# would not expect to find.
    let mut files = Vec::new();
    collect_files(&scratch.0, &mut files);
    assert_eq!(files, vec![expected]);
}

#[test]
fn a_tile_the_csharp_did_not_cache_is_a_miss_not_a_guess() {
    // The wrong branch of the tree - x/y rather than y/x, or .png - must not be found. Finding it
    // would mean this crate reads a layout the C# does not write, and a cache built here would
    // silently not be shared.
    let scratch = Scratch::new("wrong-layout");
    for wrong in [
        "TileDBv3/en/OpenStreetMap/16/59922/39658.jpg", // x before y
        "TileDBv3/en/OpenStreetMap/16/39658/59922.png", // right place, .png
        "TileDBv3/OpenStreetMap/16/39658/59922.jpg",    // no language directory
        "OpenStreetMap/16/39658/59922.jpg",             // no TileDBv3
    ] {
        let path = scratch.0.join(wrong);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, tile_png()).unwrap();
    }
    let cache = TileCache::new(&scratch.0);
    let subject = TileId::new(16, 59_922, 39_658).unwrap();
    assert!(cache.read(OPENSTREETMAP.cache_name, subject).is_none());
    assert!(!cache.contains(OPENSTREETMAP.cache_name, subject));
}

#[test]
fn the_real_mission_planner_cache_on_this_machine_reads_back() {
    // Where the C# keeps it, by the same rules it uses. The first tile found under any provider
    // is copied into a scratch cache at the same relative path and read from there: the point is
    // to read bytes the C# wrote through this crate's layout, without touching the real cache -
    // `read` deletes what it cannot decode, and a test has no business deleting an operator's
    // tiles even when they are corrupt.
    let Some(real) =
        mp_settings::Folders::from_environment().map(|f| f.csharp_map_cache_directory())
    else {
        eprintln!("skipped: no home directory");
        return;
    };
    let tile_root = real.join("TileDBv3").join("en");
    if !tile_root.is_dir() {
        eprintln!("skipped: no Mission Planner cache at {}", real.display());
        return;
    }
    let Some((provider, id, source_path)) = first_tile(&tile_root) else {
        eprintln!("skipped: {} holds no tiles", tile_root.display());
        return;
    };
    eprintln!("reading {} from the real cache", source_path.display());

    let scratch = Scratch::new("real");
    let copy = csharp_path(&scratch.0, &provider, id.z, id.x, id.y);
    std::fs::create_dir_all(copy.parent().unwrap()).unwrap();
    std::fs::copy(&source_path, &copy).unwrap();

    let cache = TileCache::new(&scratch.0);
    let found = cache
        .read(&provider, id)
        .expect("a tile the C# application wrote should be readable");
    assert_eq!(
        found.bytes,
        std::fs::read(&source_path).unwrap(),
        "byte for byte"
    );
    let decoded = DecodedTile::decode(&found.bytes).expect("and it decodes");
    assert_eq!((decoded.width, decoded.height), (256, 256), "a map tile");
}

#[test]
fn google_imagery_the_csharp_cached_on_this_machine_is_shown_with_the_network_off() {
    // Mission Planner's default map, from its own cache, through the store the map draws from.
    // As above, the tile is copied out first: the store deletes what it cannot decode.
    let Some(real) =
        mp_settings::Folders::from_environment().map(|f| f.csharp_map_cache_directory())
    else {
        eprintln!("skipped: no home directory");
        return;
    };
    let tile_root = real.join("TileDBv3").join("en");
    let provider = GOOGLE_SATELLITE_MAP.cache_name;
    let Some((_, id, source_path)) = first_tile_of(&tile_root, provider) else {
        eprintln!("skipped: no {provider} tiles under {}", tile_root.display());
        return;
    };
    eprintln!("showing {} from the real cache", source_path.display());

    let scratch = Scratch::new("real-google");
    let copy = csharp_path(&scratch.0, provider, id.z, id.x, id.y);
    std::fs::create_dir_all(copy.parent().unwrap()).unwrap();
    std::fs::copy(&source_path, &copy).unwrap();

    let store = TileStore::offline(&GOOGLE_SATELLITE_MAP, TileCache::new(&scratch.0));
    assert!(
        matches!(store.get(id), TileAnswer::Missing),
        "first ask queues it"
    );
    wait_for_a_tile(&store);
    match store.get(id) {
        TileAnswer::Exact(tile) => assert_eq!((tile.width, tile.height), (256, 256)),
        other => panic!(
            "expected the cached tile, got {other:?} with {:?}",
            store.stats()
        ),
    }
    assert_eq!(store.stats().fetched, 0, "the network was never consulted");
    assert_eq!(
        ImageFormat::sniff(&std::fs::read(&copy).unwrap()),
        Some(ImageFormat::Jpeg),
        "Google's imagery is JPEG, filed under .jpg like everything else"
    );
}

/// The first `<provider>/<z>/<y>/<x>.jpg` under a tile root, parsed back into its parts.
fn first_tile(tile_root: &Path) -> Option<(String, TileId, PathBuf)> {
    first_tile_under(tile_root, tile_root)
}

/// The same, for one provider's directory only.
fn first_tile_of(tile_root: &Path, provider: &str) -> Option<(String, TileId, PathBuf)> {
    first_tile_under(tile_root, &tile_root.join(provider))
}

/// The first tile file under `directory`, parsed relative to `tile_root`.
fn first_tile_under(tile_root: &Path, directory: &Path) -> Option<(String, TileId, PathBuf)> {
    let mut files = Vec::new();
    collect_files(directory, &mut files);
    files.sort();
    files.into_iter().find_map(|path| {
        let relative = path.strip_prefix(tile_root).ok()?;
        let parts: Vec<&str> = relative.iter().filter_map(|p| p.to_str()).collect();
        let [provider, z, y, file] = parts[..] else {
            return None;
        };
        let x = file.strip_suffix(".jpg")?;
        let id = TileId::new(z.parse().ok()?, x.parse().ok()?, y.parse().ok()?)?;
        Some((provider.to_owned(), id, path))
    })
}

/// Every file under a directory, in no particular order.
fn collect_files(directory: &Path, into: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(directory) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            collect_files(&path, into);
        } else {
            into.push(path);
        }
    }
}
