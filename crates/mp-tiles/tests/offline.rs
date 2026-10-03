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

//! Full function with the network disabled (DELIVERABLES.md D8).
//!
//! The requirement is not "degrades gracefully". A survey flown from a cache filled at home has to
//! work in a paddock with no signal, and nothing here may block, retry, or wait on a network that
//! is not there.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::PathBuf;
use std::time::{Duration, Instant};

use mp_tiles::cache::TileCache;
use mp_tiles::source::OPENSTREETMAP;
use mp_tiles::store::{DecodedTile, TileAnswer, TileStore};
use mp_units::TileId;

/// A temporary directory that removes itself.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let path = std::env::temp_dir().join(format!(
            "mp-tiles-offline-{name}-{}-{:?}",
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

/// A real, small PNG. Built with the encoder rather than pasted as fixture bytes, so a change in
/// what the decoder accepts cannot silently diverge from what the tests feed it.
fn small_png() -> Vec<u8> {
    let mut buffer = std::io::Cursor::new(Vec::new());
    let image = image::RgbaImage::from_pixel(2, 2, image::Rgba([10, 20, 30, 255]));
    image
        .write_to(&mut buffer, image::ImageFormat::Png)
        .expect("encoding a 2x2 png should work");
    buffer.into_inner()
}

fn tile(z: u8, x: u32, y: u32) -> TileId {
    TileId::new(z, i64::from(x), i64::from(y)).expect("a valid tile")
}

#[test]
fn an_offline_store_serves_everything_that_was_cached() {
    let scratch = Scratch::new("serves-cache");
    let cache = TileCache::new(&scratch.0);
    let subject = tile(14, 15_089, 9_814);
    cache
        .write(OPENSTREETMAP.cache_name, subject, &small_png())
        .expect("seeding the cache");

    let store = TileStore::offline(&OPENSTREETMAP, cache);
    let loaded = store
        .load_from_cache(subject)
        .expect("a cached tile should load with no network");
    assert_eq!(loaded.width, 2);
    assert_eq!(loaded.height, 2);
    assert_eq!(loaded.rgba.len(), 2 * 2 * 4);

    // And it is then served from memory, which is the path a pan takes.
    assert!(matches!(store.get(subject), TileAnswer::Exact(_)));
    assert_eq!(store.stats().memory_hits, 1);
}

#[test]
fn an_offline_store_reads_the_disk_without_being_asked_to() {
    // The path the map takes: `get` misses memory, queues the tile, and the worker reads it from
    // disk. `load_from_cache` above is the pre-loading shortcut; this is the one that has to work
    // in a paddock, and for a while it did not - an offline store had no worker, so nothing was
    // ever read unless a test read it by hand.
    let scratch = Scratch::new("reads-disk");
    let cache = TileCache::new(&scratch.0);
    let subject = tile(14, 15_089, 9_814);
    cache
        .write(OPENSTREETMAP.cache_name, subject, &small_png())
        .expect("seeding the cache");

    let store = TileStore::offline(&OPENSTREETMAP, cache);
    assert!(
        matches!(store.get(subject), TileAnswer::Missing),
        "the first ask is a miss: nothing blocks on the disk in a render pass"
    );

    // Bounded: the worker has a whole second to read one file, and a test that waits for ever
    // when it does not is a hang rather than a failure.
    let deadline = Instant::now() + Duration::from_secs(2);
    while store.generation() == 0 && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        matches!(store.get(subject), TileAnswer::Exact(_)),
        "the worker should have read the tile from disk: {:?}",
        store.stats()
    );
    assert_eq!(store.stats().disk_hits, 1);
    assert_eq!(store.stats().fetched, 0, "offline means offline");
}

#[test]
fn an_offline_store_never_reaches_for_the_network() {
    let scratch = Scratch::new("no-network");
    let store = TileStore::offline(&OPENSTREETMAP, TileCache::new(&scratch.0));
    assert!(store.is_offline());

    let answer = store.get(tile(14, 1, 1));
    assert!(matches!(answer, TileAnswer::Missing));
    assert_eq!(store.stats().fetched, 0);
    assert_eq!(store.stats().failed, 0);
}

#[test]
fn asking_for_a_missing_tile_returns_immediately() {
    // The whole design rests on this: a render pass never waits. If a miss ever blocked, the map
    // would stutter for every tile not yet loaded, which is most of them while panning.
    let scratch = Scratch::new("immediate");
    let store = TileStore::offline(&OPENSTREETMAP, TileCache::new(&scratch.0));

    let started = Instant::now();
    for x in 0..500 {
        let _ = store.get(tile(14, x, 9_814));
    }
    let elapsed = started.elapsed();
    assert!(
        elapsed < Duration::from_millis(100),
        "500 misses took {elapsed:?}; a render pass must never wait"
    );
}

#[test]
fn a_missing_tile_is_answered_with_an_ancestor_to_scale_up() {
    // This is what makes a map fill in progressively from coarse to fine instead of appearing
    // blank and then snapping into place.
    let scratch = Scratch::new("ancestor");
    let cache = TileCache::new(&scratch.0);

    let child = tile(14, 15_089, 9_814);
    let grandparent = child
        .parent()
        .expect("a parent")
        .parent()
        .expect("a grandparent");
    cache
        .write(OPENSTREETMAP.cache_name, grandparent, &small_png())
        .expect("seeding");

    let store = TileStore::offline(&OPENSTREETMAP, cache);
    store.load_from_cache(grandparent).expect("load ancestor");

    match store.get(child) {
        TileAnswer::Ancestor { id, .. } => assert_eq!(id, grandparent),
        other => panic!("expected an ancestor, got {other:?}"),
    }
    assert_eq!(store.stats().ancestor_hits, 1);
}

#[test]
fn an_ancestor_search_gives_up_rather_than_walking_to_the_whole_world() {
    // A zoom-0 stand-in for a zoom-18 tile is one pixel of the planet stretched across the screen.
    // It would be worse than nothing, because it looks like data.
    let scratch = Scratch::new("ancestor-depth");
    let cache = TileCache::new(&scratch.0);
    cache
        .write(OPENSTREETMAP.cache_name, tile(0, 0, 0), &small_png())
        .expect("seeding");

    let store = TileStore::offline(&OPENSTREETMAP, cache);
    store.load_from_cache(tile(0, 0, 0)).expect("load world");

    let deep = tile(18, 241_430, 157_035);
    assert!(
        matches!(store.get(deep), TileAnswer::Missing),
        "a zoom-0 tile is not a usable stand-in for zoom 18"
    );
}

#[test]
fn memory_use_is_bounded_however_far_you_pan() {
    // An hour of panning must not consume the machine.
    let scratch = Scratch::new("bounded");
    let cache = TileCache::new(&scratch.0);
    let png = small_png();

    let count = mp_tiles::store::MEMORY_TILES + 50;
    for x in 0..count {
        let id = tile(14, u32::try_from(x).expect("small"), 9_814);
        cache
            .write(OPENSTREETMAP.cache_name, id, &png)
            .expect("seeding");
    }

    let store = TileStore::offline(&OPENSTREETMAP, cache);
    for x in 0..count {
        let id = tile(14, u32::try_from(x).expect("small"), 9_814);
        store.load_from_cache(id).expect("load");
    }

    assert_eq!(
        store.memory_tiles(),
        mp_tiles::store::MEMORY_TILES,
        "the memory cache should have evicted down to its limit"
    );
}

#[test]
fn the_tile_most_recently_used_survives_eviction() {
    // Panning back and forth over the same area must stay instant; evicting what is on screen
    // would make it reload constantly.
    let scratch = Scratch::new("lru");
    let cache = TileCache::new(&scratch.0);
    let png = small_png();

    let keeper = tile(14, 0, 9_814);
    let count = mp_tiles::store::MEMORY_TILES + 10;
    for x in 0..count {
        let id = tile(14, u32::try_from(x).expect("small"), 9_814);
        cache
            .write(OPENSTREETMAP.cache_name, id, &png)
            .expect("seeding");
    }

    let store = TileStore::offline(&OPENSTREETMAP, cache);
    store.load_from_cache(keeper).expect("load keeper");

    for x in 1..count {
        let id = tile(14, u32::try_from(x).expect("small"), 9_814);
        store.load_from_cache(id).expect("load");
        // Keep touching the keeper, so it stays the most recently used.
        assert!(matches!(store.get(keeper), TileAnswer::Exact(_)), "x={x}");
    }
}

#[test]
fn a_corrupt_cached_tile_does_not_take_the_store_down_with_it() {
    let scratch = Scratch::new("corrupt");
    let cache = TileCache::new(&scratch.0);
    let subject = tile(14, 15_089, 9_814);
    cache
        .write(OPENSTREETMAP.cache_name, subject, &small_png())
        .expect("seeding");

    // Corrupt it behind the cache's back.
    let path = cache.path_for(OPENSTREETMAP.cache_name, subject);
    std::fs::write(&path, b"not a png at all").expect("corrupting");

    let store = TileStore::offline(&OPENSTREETMAP, cache);
    assert!(store.load_from_cache(subject).is_none());
    assert!(matches!(store.get(subject), TileAnswer::Missing));
}

#[test]
fn decoding_rejects_what_is_not_an_image_rather_than_panicking() {
    assert!(DecodedTile::decode(b"").is_none());
    assert!(DecodedTile::decode(b"<!DOCTYPE html>").is_none());
    // Correct PNG magic, truncated body: the shape a half-finished download takes.
    let mut truncated = small_png();
    truncated.truncate(20);
    assert!(DecodedTile::decode(&truncated).is_none());
}

#[test]
fn a_real_png_decodes_to_straight_rgba() {
    let decoded = DecodedTile::decode(&small_png()).expect("should decode");
    assert_eq!(decoded.width, 2);
    assert_eq!(decoded.height, 2);
    assert_eq!(decoded.rgba.len(), 16);
    // The colour written above: straight alpha, not premultiplied and not swizzled.
    assert_eq!(&decoded.rgba[..4], &[10, 20, 30, 255]);
}

#[test]
fn dropping_a_store_stops_its_thread() {
    // A ground station that leaked a thread per provider switch would accumulate them all session.
    // The drop joins the thread, so completing at all is the assertion.
    let scratch = Scratch::new("shutdown");
    let store = TileStore::new(&OPENSTREETMAP, TileCache::new(&scratch.0));
    assert!(!store.is_offline());
    drop(store);
}
