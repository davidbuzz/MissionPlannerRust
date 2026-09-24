//! A cached tile is on screen whatever the network is doing.
//!
//! The first view after start-up is often somewhere the cache has never seen - the saved map
//! position, the vehicle's position before the operator pans home - and every tile of it goes to
//! the network. An earlier store did disk and network on one thread, so the cached tiles asked
//! for next waited behind those fetches, up to the ten-second timeout each, and a full cache took
//! five to ten seconds to appear. GMap.NET reads the cache on the same threads that fetch, but it
//! has five of them (`Core.cs:62`), so a stalled fetch costs it one thread, not the cache.
//!
//! Here the proxy is a socket that accepts and never answers, so every fetch hangs for the whole
//! timeout; with all the fetch threads hung on uncached tiles, a cached one must still arrive.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use mp_tiles::cache::TileCache;
use mp_tiles::fetch::TileFetcher;
use mp_tiles::source::BING_MAP;
use mp_tiles::store::{FETCH_THREADS, TileAnswer, TileStore};
use mp_units::TileId;

/// A temporary directory that removes itself.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let path =
            std::env::temp_dir().join(format!("mp-tiles-startup-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&path);
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// A proxy that accepts every connection and answers none of them, holding each open until told
/// to stop - what a tile server behind a slow or absent network looks like to the fetcher.
struct Silent {
    proxy: String,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Silent {
    fn start() -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        listener.set_nonblocking(true).unwrap();
        let proxy = format!("http://{}", listener.local_addr().unwrap());
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let thread = std::thread::spawn(move || {
            let mut held: Vec<TcpStream> = Vec::new();
            while !flag.load(Ordering::Acquire) {
                match listener.accept() {
                    Ok((stream, _)) => held.push(stream),
                    Err(_) => std::thread::sleep(Duration::from_millis(10)),
                }
            }
            // Dropping the sockets ends every hung fetch with an error, so the store's threads
            // finish now rather than at the timeout.
            drop(held);
        });
        Self {
            proxy,
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for Silent {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// A 256x256 PNG, which is what a tile server sends.
fn png() -> Vec<u8> {
    let mut buffer = std::io::Cursor::new(Vec::new());
    image::RgbaImage::from_pixel(256, 256, image::Rgba([0x30, 0x60, 0x90, 255]))
        .write_to(&mut buffer, image::ImageFormat::Png)
        .unwrap();
    buffer.into_inner()
}

#[test]
fn cached_tiles_arrive_while_every_fetch_thread_is_stuck() {
    let scratch = Scratch::new("stuck");
    let cache = TileCache::new(&scratch.0);
    let cached = TileId::new(12, 3000, 2400).unwrap();
    cache.write(BING_MAP.cache_name, cached, &png()).unwrap();

    let silent = Silent::start();
    let store = TileStore::with_fetcher(
        &BING_MAP,
        cache,
        TileFetcher::through_proxy(&silent.proxy).unwrap(),
    );

    // More uncached tiles than there are fetch threads, so every one of them is hung on the
    // proxy - the version check first, then a tile each - and the rest wait in the network queue.
    let uncached: Vec<TileId> = (0..FETCH_THREADS as i64 + 3)
        .map(|i| TileId::new(12, 100 + i, 100).unwrap())
        .collect();
    for tile in &uncached {
        assert!(matches!(store.get(*tile), TileAnswer::Missing));
    }
    // Long enough for the reader to have handed all of them to the fetch threads.
    std::thread::sleep(Duration::from_millis(200));

    let asked = Instant::now();
    assert!(matches!(store.get(cached), TileAnswer::Missing));
    let deadline = asked + Duration::from_secs(1);
    let found = loop {
        if let TileAnswer::Exact(tile) = store.get(cached) {
            break tile;
        }
        assert!(
            Instant::now() < deadline,
            "the cached tile did not arrive within a second of being asked for, with the fetch \
             threads stuck; the stats say {:?}",
            store.stats()
        );
        std::thread::sleep(Duration::from_millis(5));
    };
    let took = asked.elapsed();
    assert_eq!((found.width, found.height), (256, 256));
    let stats = store.stats();
    // At least one: a second `get` before the reader publishes queues the tile again, and the
    // reader reads it again, which is a wasted read and not a wait.
    assert!(stats.disk_hits >= 1, "{stats:?}");
    assert_eq!(
        stats.fetched, 0,
        "nothing came from the silent proxy: {stats:?}"
    );
    // The point of the test, stated as a number: a disk read and a decode, not a network wait.
    assert!(took < Duration::from_millis(500), "took {took:?}");
    drop(silent);
    drop(store);
}

#[test]
fn an_offline_store_starts_no_fetch_threads_and_still_reads_the_disk() {
    let scratch = Scratch::new("offline");
    let cache = TileCache::new(&scratch.0);
    let cached = TileId::new(10, 500, 600).unwrap();
    cache.write(BING_MAP.cache_name, cached, &png()).unwrap();
    let store = TileStore::offline(&BING_MAP, cache);
    let asked = Instant::now();
    let deadline = asked + Duration::from_secs(1);
    loop {
        if let TileAnswer::Exact(_) = store.get(cached) {
            break;
        }
        assert!(Instant::now() < deadline, "{:?}", store.stats());
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(store.is_offline());
    assert_eq!(store.stats().disk_hits, 1);
}
