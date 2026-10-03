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

//! The store the map talks to.
//!
//! One rule governs the design: **a render pass never waits**. It asks for a tile and gets an
//! answer immediately - the tile, an ancestor to scale up in its place, or nothing. Everything
//! slow happens on a thread of its own.
//!
//! That is why tiles are decoded here rather than by the caller. Decoding a 256x256 PNG takes
//! around a millisecond, which is most of a frame at 120 Hz; doing it in the painter would make
//! the map stutter every time a tile arrived, which is exactly when it must not.
//!
//! The slow work is split the way GMap.NET splits it. One thread reads the disk: a cached tile is
//! decoded and published from there, and only a tile the cache does not hold is handed on to a
//! pool of fetch threads, five of them as `Core.cs`'s `GThreadPoolSize` has it, each of which
//! goes to the network and waits there. An earlier version did both on one thread, so a single
//! tile that was not cached - the first view after start-up is often somewhere the cache has
//! never seen - held every cached tile behind it for the length of a request, up to the ten
//! second timeout, and a full cache took five to ten seconds to appear.
//! `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:62, 791-1030, 1150-1165`

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use mp_units::TileId;

use crate::cache::TileCache;
use crate::fetch::TileFetcher;
use crate::policy::{Decision, FetchPolicy};
use crate::source::TileSource;

/// How long a fetch may be outstanding before its slot is reclaimed.
const FETCH_TIMEOUT: Duration = Duration::from_secs(20);

/// How many threads fetch from the network at once: GMap.NET's `GThreadPoolSize`, the number
/// of `ProcessLoadTask` threads its map core starts.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:62`
pub const FETCH_THREADS: usize = 5;

/// How many decoded tiles to keep in memory.
///
/// A 256x256 RGBA tile is 256 KiB, so this is about 64 MiB. A 1600x1200 window shows around forty
/// tiles, so this holds several screenfuls - enough that panning back and forth is instant, and
/// bounded so that an hour of panning does not consume the machine.
pub const MEMORY_TILES: usize = 256;

/// How far up the pyramid to look for a stand-in while a tile loads.
///
/// Four levels means a sixteenth-scale ancestor at worst, which is blurry but recognisable. Going
/// further gives a stand-in so coarse it misleads more than it helps.
pub const MAX_ANCESTOR_DEPTH: u8 = 4;

/// A decoded tile, ready to upload to the GPU.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedTile {
    /// Width in pixels, normally 256.
    pub width: u32,
    /// Height in pixels, normally 256.
    pub height: u32,
    /// Straight RGBA, eight bits per channel, not premultiplied.
    pub rgba: Vec<u8>,
}

impl DecodedTile {
    /// Decodes an encoded tile.
    ///
    /// Only PNG and JPEG are enabled in the decoder. A ground station is pointed at whatever URL
    /// the operator configures, and every other format is attack surface for no benefit.
    #[must_use]
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let image = image::load_from_memory(bytes).ok()?;
        let rgba = image.to_rgba8();
        Some(Self {
            width: rgba.width(),
            height: rgba.height(),
            rgba: rgba.into_raw(),
        })
    }
}

/// What the map got when it asked for a tile.
#[derive(Debug, Clone)]
pub enum TileAnswer {
    /// The tile itself.
    Exact(Arc<DecodedTile>),
    /// An ancestor, to be drawn scaled up until the real one arrives.
    ///
    /// Carries which ancestor it is so the caller can work out which part of it to draw: the tile
    /// asked for occupies one 2^n-th of it.
    Ancestor {
        /// The ancestor that was found.
        id: TileId,
        /// Its pixels.
        tile: Arc<DecodedTile>,
    },
    /// Nothing yet.
    Missing,
}

/// Counters, for the status line and for tests.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StoreStats {
    /// Answered from memory.
    pub memory_hits: u64,
    /// Answered from disk.
    pub disk_hits: u64,
    /// Answered with an ancestor.
    pub ancestor_hits: u64,
    /// Not answered at all.
    pub misses: u64,
    /// Tiles fetched over the network.
    pub fetched: u64,
    /// Fetches that failed.
    pub failed: u64,
}

/// Shared between the map and the fetch thread.
#[derive(Debug)]
struct Shared {
    memory: Mutex<Memory>,
    policy: Mutex<FetchPolicy>,
    /// What the map asked for and the reader has not looked at yet. Newest last, taken from the
    /// end.
    queue: Mutex<Vec<TileId>>,
    wake: Condvar,
    /// What the cache did not hold and the policy allows a fetch for, waiting for a fetch
    /// thread. Newest last, taken from the end, like the C#'s `tileLoadQueue` stack.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:59`
    network: Mutex<Vec<TileId>>,
    network_wake: Condvar,
    /// Whether this store has run the provider's `OnInitialized` - the version check Google's
    /// and Bing's providers make the first time they are shown. Here, the first time one is
    /// about to fetch: a store that only ever reads the disk never goes to the network for a
    /// version it has no use for, which is what "offline" has to mean. Held for the length of
    /// the check, so the other fetch threads wait for it rather than fetching with the
    /// hard-coded version while it runs.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:204-208`
    initialized: Mutex<bool>,
    running: AtomicBool,
    stats: Mutex<StoreStats>,
    /// Bumped whenever a tile arrives, so the UI knows to repaint without polling every tile.
    generation: AtomicU64,
    /// Holds the reader after it has taken a tile and before it looks at it, so a test can ask
    /// again while the first ask is being answered.
    #[cfg(test)]
    reader_gate: Mutex<Option<ReaderGate>>,
}

/// The reader's gate, for tests: it says which tile it took, then waits to be let go.
#[cfg(test)]
#[derive(Debug)]
struct ReaderGate {
    taken: std::sync::mpsc::Sender<TileId>,
    go: std::sync::mpsc::Receiver<()>,
}

/// The in-memory tiles, with a least-recently-used order.
#[derive(Debug, Default)]
struct Memory {
    tiles: HashMap<TileId, Arc<DecodedTile>>,
    /// Least recently used first. Small enough that a Vec beats a linked structure.
    order: Vec<TileId>,
}

impl Memory {
    fn get(&mut self, tile: TileId) -> Option<Arc<DecodedTile>> {
        let found = self.tiles.get(&tile).map(Arc::clone)?;
        if let Some(position) = self.order.iter().position(|held| *held == tile) {
            let id = self.order.remove(position);
            self.order.push(id);
        }
        Some(found)
    }

    /// Looks without disturbing the order, for the ancestor search.
    ///
    /// An ancestor consulted as a stand-in is not the tile the operator wants; promoting it would
    /// evict the tiles they do want.
    fn peek(&self, tile: TileId) -> Option<Arc<DecodedTile>> {
        self.tiles.get(&tile).map(Arc::clone)
    }

    fn insert(&mut self, tile: TileId, decoded: Arc<DecodedTile>) {
        if self.tiles.insert(tile, decoded).is_none() {
            self.order.push(tile);
        }
        while self.order.len() > MEMORY_TILES {
            if self.order.is_empty() {
                break;
            }
            let evicted = self.order.remove(0);
            self.tiles.remove(&evicted);
        }
    }
}

/// Serves tiles to the map.
#[derive(Debug)]
pub struct TileStore {
    shared: Arc<Shared>,
    cache: TileCache,
    source: &'static TileSource,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl TileStore {
    /// Starts a store with a fetch thread.
    #[must_use]
    pub fn new(source: &'static TileSource, cache: TileCache) -> Self {
        Self::with_fetcher(source, cache, TileFetcher::new())
    }

    /// Starts a store with a fetch thread that fetches with the fetcher given - one that goes
    /// through a particular proxy, say.
    #[must_use]
    pub fn with_fetcher(
        source: &'static TileSource,
        cache: TileCache,
        fetcher: TileFetcher,
    ) -> Self {
        Self::start(source, cache, FetchPolicy::new(), fetcher, true)
    }

    /// A store that never fetches, for offline use and for tests.
    ///
    /// Still serves everything already cached, which is what "full function with the network
    /// disabled" has to mean: a survey flown from a cache filled at home works in a paddock.
    ///
    /// The worker thread runs, because it is the thread that reads the disk - the policy only
    /// stops it going to the network afterwards. An earlier version started no thread here and
    /// so served nothing from disk unless a caller pre-loaded it by hand; every test passed,
    /// because every test pre-loaded by hand, and the map showed a graticule over a full cache.
    #[must_use]
    pub fn offline(source: &'static TileSource, cache: TileCache) -> Self {
        Self::start(
            source,
            cache,
            FetchPolicy::offline(),
            TileFetcher::new(),
            true,
        )
    }

    fn start(
        source: &'static TileSource,
        cache: TileCache,
        policy: FetchPolicy,
        fetcher: TileFetcher,
        spawn: bool,
    ) -> Self {
        let offline = policy.is_offline();
        let shared = Arc::new(Shared {
            memory: Mutex::new(Memory::default()),
            policy: Mutex::new(policy),
            queue: Mutex::new(Vec::new()),
            wake: Condvar::new(),
            network: Mutex::new(Vec::new()),
            network_wake: Condvar::new(),
            initialized: Mutex::new(false),
            running: AtomicBool::new(true),
            stats: Mutex::new(StoreStats::default()),
            generation: AtomicU64::new(0),
            #[cfg(test)]
            reader_gate: Mutex::new(None),
        });

        let mut threads = Vec::new();
        if spawn {
            let thread_shared = Arc::clone(&shared);
            let thread_cache = cache.clone();
            let reader = std::thread::Builder::new()
                .name("mp-tiles".to_owned())
                .spawn(move || run_reader(source, &thread_cache, &thread_shared));
            threads.extend(reader.ok());
            // An offline store's policy never lets a tile reach the network queue, so its fetch
            // threads would only ever wait; none are started.
            let fetchers = if offline { 0 } else { FETCH_THREADS };
            for index in 0..fetchers {
                let thread_shared = Arc::clone(&shared);
                let thread_cache = cache.clone();
                let thread_fetcher = fetcher.clone();
                let handle = std::thread::Builder::new()
                    .name(format!("mp-tiles-fetch-{index}"))
                    .spawn(move || {
                        run_fetcher(source, &thread_cache, &thread_shared, &thread_fetcher);
                    });
                threads.extend(handle.ok());
            }
        }

        Self {
            shared,
            cache,
            source,
            threads,
        }
    }

    /// The provider being shown.
    #[must_use]
    pub const fn source(&self) -> &'static TileSource {
        self.source
    }

    /// Bumped whenever a tile arrives. A UI can repaint when it changes rather than polling.
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.shared.generation.load(Ordering::Acquire)
    }

    /// Counters.
    #[must_use]
    pub fn stats(&self) -> StoreStats {
        self.shared.stats.lock().map(|s| *s).unwrap_or_default()
    }

    /// Whether fetching is switched off.
    #[must_use]
    pub fn is_offline(&self) -> bool {
        self.shared
            .policy
            .lock()
            .map(|policy| policy.is_offline())
            .unwrap_or(true)
    }

    /// Asks for a tile. Never blocks on I/O.
    ///
    /// A miss queues a fetch and answers with the best ancestor already held, so the map fills in
    /// progressively from coarse to fine rather than appearing blank and then snapping.
    pub fn get(&self, tile: TileId) -> TileAnswer {
        // Memory first: the common case, and the only one on the hot path of a pan.
        if let Ok(mut memory) = self.shared.memory.lock()
            && let Some(found) = memory.get(tile)
        {
            self.bump(|stats| stats.memory_hits += 1);
            return TileAnswer::Exact(found);
        }

        // Then disk. Reading and decoding here would block the render thread, so it is queued for
        // the fetch thread, which checks the cache before the network.
        self.request(tile);

        match self.best_ancestor(tile) {
            Some((id, found)) => {
                self.bump(|stats| stats.ancestor_hits += 1);
                TileAnswer::Ancestor { id, tile: found }
            }
            None => {
                self.bump(|stats| stats.misses += 1);
                TileAnswer::Missing
            }
        }
    }

    /// The nearest ancestor already in memory.
    fn best_ancestor(&self, tile: TileId) -> Option<(TileId, Arc<DecodedTile>)> {
        let Ok(memory) = self.shared.memory.lock() else {
            return None;
        };
        let mut ancestor = tile;
        for _ in 0..MAX_ANCESTOR_DEPTH {
            ancestor = ancestor.parent()?;
            if let Some(found) = memory.peek(ancestor) {
                return Some((ancestor, found));
            }
        }
        None
    }

    /// Queues a tile, if the policy allows and it is not already queued.
    fn request(&self, tile: TileId) {
        if self.source.url_for(tile).is_none() {
            return;
        }
        let Ok(mut queue) = self.shared.queue.lock() else {
            return;
        };
        if queue.contains(&tile) {
            return;
        }
        // Newest first: the operator has moved, and the tiles they asked for most recently are the
        // ones under their eyes. A stale request at the back is dropped rather than served late.
        queue.push(tile);
        if queue.len() > MEMORY_TILES {
            queue.remove(0);
        }
        drop(queue);
        self.shared.wake.notify_one();
    }

    /// Reads a tile straight from the cache, decoding on this thread.
    ///
    /// For tests and for pre-loading; the map uses [`TileStore::get`], which never blocks.
    pub fn load_from_cache(&self, tile: TileId) -> Option<Arc<DecodedTile>> {
        let cached = self.cache.read(self.source.cache_name, tile)?;
        let decoded = Arc::new(DecodedTile::decode(&cached.bytes)?);
        if let Ok(mut memory) = self.shared.memory.lock() {
            memory.insert(tile, Arc::clone(&decoded));
        }
        self.bump(|stats| stats.disk_hits += 1);
        Some(decoded)
    }

    /// How many tiles are held in memory.
    #[must_use]
    pub fn memory_tiles(&self) -> usize {
        self.shared
            .memory
            .lock()
            .map(|memory| memory.tiles.len())
            .unwrap_or(0)
    }

    fn bump(&self, change: impl FnOnce(&mut StoreStats)) {
        if let Ok(mut stats) = self.shared.stats.lock() {
            change(&mut stats);
        }
    }
}

impl Drop for TileStore {
    fn drop(&mut self) {
        self.shared.running.store(false, Ordering::Release);
        self.shared.wake.notify_all();
        self.shared.network_wake.notify_all();
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

/// The reader thread: the cache first, and what it does not hold to the fetch threads.
///
/// Nothing here waits on the network, so a cached tile is on screen a decode after it was asked
/// for whatever the network is doing.
fn run_reader(source: &'static TileSource, cache: &TileCache, shared: &Arc<Shared>) {
    while shared.running.load(Ordering::Acquire) {
        let Some(tile) = next_from(shared, &shared.queue, &shared.wake) else {
            continue;
        };

        #[cfg(test)]
        if let Ok(gate) = shared.reader_gate.lock()
            && let Some(gate) = gate.as_ref()
        {
            let _ = gate.taken.send(tile);
            let _ = gate.go.recv();
        }

        // A tile already in memory is not loaded again. The map asks every frame, so a tile it
        // asks for a second time while the first ask is still being read is queued again - the
        // queue only refuses a tile it still holds - and by the time this thread takes the second
        // ask, the first has been published. GMap.NET's load task makes the same check before it
        // loads anything: `Matrix.GetTileWithReadLock`, and nothing unless `!m.NotEmpty`.
        // `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:881-882, 1139-1142`
        if shared
            .memory
            .lock()
            .is_ok_and(|memory| memory.tiles.contains_key(&tile))
        {
            continue;
        }

        // The cache is the primary source. Checked here rather than on the render thread because
        // reading and decoding a tile is far too slow to do in a painter.
        if let Some(cached) = cache.read(source.cache_name, tile)
            && let Some(decoded) = DecodedTile::decode(&cached.bytes)
        {
            publish(shared, tile, decoded, |stats| stats.disk_hits += 1);
            continue;
        }

        let now = Instant::now();
        {
            let Ok(mut policy) = shared.policy.lock() else {
                continue;
            };
            policy.expire(now, FETCH_TIMEOUT);
            if policy.decide(tile, now) != Decision::Fetch {
                continue;
            }
            policy.begin(tile, now);
        }

        let Ok(mut network) = shared.network.lock() else {
            continue;
        };
        if !network.contains(&tile) {
            network.push(tile);
            // Bounded like the request queue: a tile nobody is looking at any more is dropped
            // from the front, and the policy reclaims its slot when it expires.
            if network.len() > MEMORY_TILES {
                network.remove(0);
            }
        }
        drop(network);
        shared.network_wake.notify_one();
    }
}

/// A fetch thread: the network, then decode. One of [`FETCH_THREADS`].
fn run_fetcher(
    source: &'static TileSource,
    cache: &TileCache,
    shared: &Arc<Shared>,
    fetcher: &TileFetcher,
) {
    while shared.running.load(Ordering::Acquire) {
        let Some(tile) = next_from(shared, &shared.network, &shared.network_wake) else {
            continue;
        };

        {
            let Ok(mut initialized) = shared.initialized.lock() else {
                continue;
            };
            if !*initialized {
                *initialized = true;
                crate::versions::initialize(source, cache.root(), fetcher);
            }
        }

        match fetcher.fetch(source, tile) {
            Ok(bytes) => {
                // Written before decoding, so a tile survives even if this build cannot decode it.
                let _ = cache.write(source.cache_name, tile, &bytes);
                if let Ok(mut policy) = shared.policy.lock() {
                    policy.succeeded(tile);
                }
                if let Some(decoded) = DecodedTile::decode(&bytes) {
                    publish(shared, tile, decoded, |stats| stats.fetched += 1);
                }
            }
            Err(_) => {
                if let Ok(mut policy) = shared.policy.lock() {
                    policy.failed(tile, Instant::now());
                }
                if let Ok(mut stats) = shared.stats.lock() {
                    stats.failed += 1;
                }
            }
        }
    }
}

/// Takes the next tile from a queue, newest first, waiting if there is nothing to do.
fn next_from(shared: &Arc<Shared>, queue: &Mutex<Vec<TileId>>, wake: &Condvar) -> Option<TileId> {
    let Ok(mut queue) = queue.lock() else {
        return None;
    };
    while queue.is_empty() {
        if !shared.running.load(Ordering::Acquire) {
            return None;
        }
        // A timeout rather than a bare wait, so shutdown is never missed if the notify raced.
        let Ok((held, _)) = wake.wait_timeout(queue, Duration::from_millis(200)) else {
            return None;
        };
        queue = held;
    }
    queue.pop()
}

/// Publishes a decoded tile and tells anyone watching that something changed.
fn publish(
    shared: &Arc<Shared>,
    tile: TileId,
    decoded: DecodedTile,
    count: impl FnOnce(&mut StoreStats),
) {
    // Counted with the memory still held, so whoever finds the tile in memory also finds it
    // counted: a caller that sees the tile and then reads the stats never sees it uncounted.
    if let Ok(mut memory) = shared.memory.lock() {
        memory.insert(tile, Arc::new(decoded));
        if let Ok(mut stats) = shared.stats.lock() {
            count(&mut stats);
        }
    }
    shared.generation.fetch_add(1, Ordering::AcqRel);
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use crate::source::BING_MAP;
    use std::sync::mpsc;

    /// A 256x256 PNG, which is what a tile server sends.
    fn png() -> Vec<u8> {
        let mut buffer = std::io::Cursor::new(Vec::new());
        image::RgbaImage::from_pixel(256, 256, image::Rgba([0x30, 0x60, 0x90, 255]))
            .write_to(&mut buffer, image::ImageFormat::Png)
            .unwrap();
        buffer.into_inner()
    }

    /// Row 85: the map asks again while the reader is still reading the first ask. The second
    /// ask is queued - the first has left the queue - and must not be read from disk a second
    /// time. The reader is held on a gate after taking the first ask until the second has been
    /// made, so the race happens every run rather than one run in a hundred under load.
    #[test]
    fn a_second_ask_while_the_first_is_being_read_reads_the_disk_once() {
        let root =
            std::env::temp_dir().join(format!("mp-tiles-store-second-ask-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let cache = TileCache::new(&root);
        let tile = TileId::new(10, 500, 600).unwrap();
        cache.write(BING_MAP.cache_name, tile, &png()).unwrap();

        let store = TileStore::offline(&BING_MAP, cache);
        let (taken_tx, taken) = mpsc::channel();
        let (go, go_rx) = mpsc::channel();
        *store.shared.reader_gate.lock().unwrap() = Some(ReaderGate {
            taken: taken_tx,
            go: go_rx,
        });
        let wait = Duration::from_secs(5);

        assert!(matches!(store.get(tile), TileAnswer::Missing));
        assert_eq!(
            taken.recv_timeout(wait).unwrap(),
            tile,
            "the reader took the first ask"
        );

        // The first ask is in the reader's hands and out of the queue; this one is queued.
        assert!(matches!(store.get(tile), TileAnswer::Missing));
        assert!(
            store.shared.queue.lock().unwrap().contains(&tile),
            "the second ask was queued, which is the race"
        );

        go.send(()).unwrap();
        assert_eq!(
            taken.recv_timeout(wait).unwrap(),
            tile,
            "the reader took the second ask"
        );
        go.send(()).unwrap();

        // Dropping the store joins the reader, which finishes the second ask first.
        let shared = Arc::clone(&store.shared);
        drop(store);
        let stats = *shared.stats.lock().unwrap();
        assert_eq!(stats.disk_hits, 1, "{stats:?}");
        assert_eq!(stats.misses, 2, "{stats:?}");
        assert!(shared.memory.lock().unwrap().tiles.contains_key(&tile));
        let _ = std::fs::remove_dir_all(&root);
    }
}
