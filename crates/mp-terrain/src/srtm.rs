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

//! `srtm.getAltitude` and everything it reads: tile names, `.hgt` tiles, the ASCII-grid fallback,
//! and the state the download thread shares with it.

use mp_os::fs::FsExt as _;
use mp_os::Lock as _;
use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use web_time::{Duration, Instant};

use crate::fetch::{Http, UreqHttp};

/// `srtm.tiletype`: what kind of answer an [`AltResponse`] is.
/// `// C#: ExtLibs/Utilities/srtm.cs:23-28`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TileType {
    /// `valid`: `alt` is the ground.
    Valid,
    /// `invalid`: no answer - no tile, a tile being fetched, a void, a damaged file.
    Invalid,
    /// `ocean`: the tile is on no server, so it is sea; `alt` is 0.
    Ocean,
}

impl TileType {
    /// The member's name, as the C# prints the enum.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Valid => "valid",
            Self::Invalid => "invalid",
            Self::Ocean => "ocean",
        }
    }
}

/// `srtm.altresponce`: an altitude, where it came from, and whether to believe it.
/// `// C#: ExtLibs/Utilities/srtm.cs:30-38`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltResponse {
    /// `currenttype`.
    pub current_type: TileType,
    /// `alt`: metres above the SRTM datum.
    pub alt: f64,
    /// `altsource`: `"SRTM"`, `"ASCII"`, `"Invalid"`, `"Ocean"`, or `""` for a cell found in an
    /// ASCII grid (srtm.cs:358-362 never sets it).
    pub alt_source: &'static str,
}

impl AltResponse {
    /// `altresponce.Invalid`.
    /// `// C#: ExtLibs/Utilities/srtm.cs:32`
    pub const INVALID: Self = Self {
        current_type: TileType::Invalid,
        alt: 0.0,
        alt_source: "Invalid",
    };

    /// `altresponce.Ocean`.
    /// `// C#: ExtLibs/Utilities/srtm.cs:33`
    pub const OCEAN: Self = Self {
        current_type: TileType::Ocean,
        alt: 0.0,
        alt_source: "Ocean",
    };
}

/// `getAltitude`'s default zoom.
/// `// C#: ExtLibs/Utilities/srtm.cs:116`
pub const DEFAULT_ZOOM: f64 = 16.0;

/// The zoom from which a missing tile is queued for download. The only thing `zoom` does.
/// `// C#: ExtLibs/Utilities/srtm.cs:380`
pub const DOWNLOAD_ZOOM: f64 = 7.0;

/// `srtm.baseurl1sec`: the 1-arc-second tiles, searched first.
/// `// C#: ExtLibs/Utilities/srtm.cs:628`
pub const BASEURL1SEC: &str = "https://terrain.ardupilot.org/SRTM1/";

/// `srtm.baseurl`: the index of the 3-arc-second regions, searched after.
/// `// C#: ExtLibs/Utilities/srtm.cs:630`
pub const BASEURL: &str = "https://terrain.ardupilot.org/SRTM3/";

/// Samples along each side of a 3-arc-second tile.
const SIZE_3SEC: usize = 1201;

/// Samples along each side of a 1-arc-second tile.
const SIZE_1SEC: usize = 3601;

/// What went wrong inside `getAltitude`'s `try`: whatever it is, the answer is
/// [`AltResponse::INVALID`] (srtm.cs:400-404).
#[derive(Debug)]
pub(crate) struct Fault;

impl From<std::io::Error> for Fault {
    fn from(_: std::io::Error) -> Self {
        Self
    }
}

/// A lock that a panic elsewhere does not poison for good: every value behind these is left
/// consistent between statements.
pub(crate) fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.os_lock().unwrap_or_else(PoisonError::into_inner)
}

/// C#'s `(int)` of a double, as mono does it on x86-64 (`cvttsd2si`): truncation toward zero,
/// and `int.MinValue` for NaN, the infinities and anything out of range. Measured, not assumed:
/// the `cast` records of `testdata/srtm/oracle.txt`.
#[allow(clippy::cast_possible_truncation)] // range-checked just above the cast
pub(crate) fn cs_int(value: f64) -> i32 {
    let truncated = value.trunc();
    if (-2_147_483_648.0..=2_147_483_647.0).contains(&truncated) {
        truncated as i32
    } else {
        i32::MIN
    }
}

/// `(int)Math.Floor(value)`.
/// `// C#: ExtLibs/Utilities/srtm.cs:106-107, 244-245`
fn floor_int(value: f64) -> i32 {
    cs_int(value.floor())
}

/// `GetFilename` and the `filenameDictionary` it asks: the tile holding a point, `N35E149.hgt`,
/// or `None` where the C# returns `""`.
///
/// Both coordinates are floored, so the tile is named for its south-west corner and a point on a
/// whole degree belongs to the tile north and east of it.
///
/// The C# memoises names in a dictionary keyed `y * 1000 + x`, which is not one-to-one over the
/// integers it is handed: once `N01E005.hgt` has been named, `(0, 1005)` finds that entry before
/// the range check and answers with it, and `getAltitude` then reads that tile at a longitude it
/// does not hold. Diverged: names are not memoised - formatting one is cheaper than the lock the C#
/// takes to look it up - so a coordinate out of range is always out of range.
/// `// C#: ExtLibs/Utilities/srtm.cs:68-92, 104-114`
#[must_use]
pub fn tile_name(lat: f64, lng: f64) -> Option<String> {
    let x = floor_int(lng);
    let y = floor_int(lat);
    if !(-90..=90).contains(&y) || !(-180..=180).contains(&x) {
        return None;
    }
    Some(format!(
        "{}{:02}{}{:03}.hgt",
        if y >= 0 { 'N' } else { 'S' },
        y.unsigned_abs(),
        if x >= 0 { 'E' } else { 'W' },
        x.unsigned_abs()
    ))
}

/// `Math.Round(value, 0)` as mono 6 computes it.
///
/// `Round(double, int)` rounds only below 1e16 and hands the rest to the runtime's
/// `Math.Round(double)` icall, which is not IEEE round-half-even: it floors `value + 0.5` and steps
/// back to even only when `value` is exactly halfway, so `0.49999999999999994` - whose `+ 0.5`
/// rounds up to 1 - rounds to 1. The `format` records of `testdata/srtm/oracle.txt` hold this to
/// what mono printed; no coordinate in range reaches the difference, but nothing has to prove that
/// if the arithmetic is the same.
fn math_round(value: f64) -> f64 {
    // C#: Abs(value) < doubleRoundLimit, branched on unordered too (`bge.un`), so NaN returns.
    if value.is_nan() || value.abs() >= 1e16 {
        return value;
    }
    // "If the number has no fractional part do nothing" - which keeps -0 as -0.
    #[allow(clippy::cast_possible_truncation)] // |value| < 1e16 fits an i64
    let whole = value as i64;
    #[allow(clippy::cast_precision_loss)] // the C#'s own round trip through a long
    let whole = whole as f64;
    if value == whole {
        return value;
    }
    let mut floor = (value + 0.5).floor();
    if value == value.floor() + 0.5 && floor % 2.0 != 0.0 {
        floor -= 1.0;
    }
    floor.copysign(value)
}

/// `.ToString("00")` of a whole number: at least two digits, and a zero never signed.
fn format_00(value: f64) -> String {
    #[allow(clippy::cast_possible_truncation)] // a whole number well inside an i64 (see callers)
    let whole = value as i64;
    if whole < 0 {
        format!("-{:02}", whole.unsigned_abs())
    } else {
        format!("{whole:02}")
    }
}

/// The CGIAR ASCII grid `getAltitude` falls back to when there is no `.hgt`:
/// `srtm_<column>_<row>.asc`, each a 5-degree square counted from 180 W and 60 N.
///
/// Rounded with `Math.Round` - half to even - so `lng` 20 is column 40 but 20.1 is 41. Only ever
/// asked for a point that has a tile name, so both numbers are small.
/// `// C#: ExtLibs/Utilities/srtm.cs:283-284`
#[must_use]
pub fn ascii_name(lat: f64, lng: f64) -> String {
    format!(
        "srtm_{}_{}.asc",
        format_00(math_round((lng + 2.5 + 180.0) / 5.0)),
        format_00(math_round((60.0 - lat + 2.5) / 5.0))
    )
}

/// One tile as `getAltitude` caches it: heights in file order - rows north to south, each west to
/// east - so sample `(x, y)` of the C#'s `short[x, y]` is `data[y * size + x]`.
/// `// C#: ExtLibs/Utilities/srtm.cs:209-226`
#[derive(Debug)]
struct Tile {
    size: usize,
    data: Vec<i16>,
}

impl Tile {
    /// `GetAlt`: one sample, or an error where the C#'s two-dimensional array throws. Each index
    /// is checked on its own, as a `short[,]` checks them - a flat index would read the next row
    /// instead of failing.
    /// `// C#: ExtLibs/Utilities/srtm.cs:414-417`
    fn get(&self, x: i32, y: i32) -> Result<f64, Fault> {
        let x = usize::try_from(x).map_err(|_| Fault)?;
        let y = usize::try_from(y).map_err(|_| Fault)?;
        if x >= self.size || y >= self.size {
            return Err(Fault);
        }
        self.data
            .get(y * self.size + x)
            .map(|&sample| f64::from(sample))
            .ok_or(Fault)
    }
}

/// `avg`: `v1` and `v2` blended, `weight` of the way to `v2`. Written as the C# writes it; the
/// order of the operations is the answer's last bit.
/// `// C#: ExtLibs/Utilities/srtm.cs:419-422`
fn avg(v1: f64, v2: f64, weight: f64) -> f64 {
    v2 * weight + v1 * (1.0 - weight)
}

/// Reads a `.hgt`: `None` for a length that is neither size, which the C# answers
/// [`AltResponse::INVALID`] without caching.
/// `// C#: ExtLibs/Utilities/srtm.cs:193-230`
fn read_hgt(path: &Path) -> Result<Option<Tile>, Fault> {
    let mut file = mp_os::fs::File::open(path)?;
    let length = file.metadata()?.len();
    let size = if length == (SIZE_3SEC * SIZE_3SEC * 2) as u64 {
        SIZE_3SEC
    } else if length == (SIZE_1SEC * SIZE_1SEC * 2) as u64 {
        SIZE_1SEC
    } else {
        return Ok(None);
    };
    let mut bytes = Vec::with_capacity(size * size * 2);
    file.read_to_end(&mut bytes)?;
    // C#: a file that grew since its length was read overruns the array and throws; one that
    // shrank leaves the rest of the array 0.
    if bytes.len() > size * size * 2 {
        return Err(Fault);
    }
    let mut data = vec![0i16; size * size];
    for (sample, pair) in data.iter_mut().zip(bytes.chunks_exact(2)) {
        // C#: (short)((altbytes[0] << 8) + altbytes[1]) - big-endian.
        if let [high, low] = *pair {
            *sample = i16::from_be_bytes([high, low]);
        }
    }
    Ok(Some(Tile { size, data }))
}

/// `SemaphoreSlim`: a count, a release that adds one, and a wait that takes one or times out.
/// No maximum, as the C#'s has none, so releases the queue thread does not consume pile up and
/// each later wait returns at once.
#[derive(Debug)]
pub(crate) struct Semaphore {
    count: Mutex<u64>,
    wake: Condvar,
}

impl Semaphore {
    fn new(initial: u64) -> Self {
        Self {
            count: Mutex::new(initial),
            wake: Condvar::new(),
        }
    }

    /// `Release()`.
    pub(crate) fn release(&self) {
        *lock(&self.count) += 1;
        self.wake.notify_one();
    }

    /// Wakes every wait, so each looks at its `stop` flag.
    fn wake_all(&self) {
        let _count = lock(&self.count);
        self.wake.notify_all();
    }

    /// `WaitAsync(timeout)`: whether a count was taken. Also gives up when `stop` is set, which
    /// the C# does not need (see [`Srtm`]'s `Drop`).
    pub(crate) fn wait(&self, timeout: Duration, stop: &AtomicBool) -> bool {
        let deadline = Instant::now() + timeout;
        let mut count = lock(&self.count);
        loop {
            if *count > 0 {
                *count -= 1;
                return true;
            }
            let now = Instant::now();
            if stop.load(Ordering::Acquire) || now >= deadline {
                return false;
            }
            count = match self.wake.wait_timeout(count, deadline - now) {
                Ok((guard, _)) => guard,
                Err(poisoned) => poisoned.into_inner().0,
            };
        }
    }
}

/// The two servers, `srtm.baseurl1sec` and `srtm.baseurl`: public settable statics in the C#.
#[derive(Debug, Clone)]
pub(crate) struct Servers {
    pub(crate) baseurl1sec: String,
    pub(crate) baseurl: String,
}

/// The class's static state. One per [`Srtm`], shared with its download thread.
pub(crate) struct Inner {
    /// `srtm.datadirectory`.
    pub(crate) datadirectory: PathBuf,
    /// `HttpClient client`, injectable.
    pub(crate) http: Arc<dyn Http>,
    pub(crate) servers: Mutex<Servers>,
    /// `queue`, under `objlock`.
    pub(crate) queue: Mutex<Vec<String>>,
    /// `oceantile`.
    pub(crate) oceantile: Mutex<Vec<String>>,
    /// `cache`: the tiles read so far, never evicted.
    cache: Mutex<HashMap<String, Arc<Tile>>>,
    /// `filecache`: the ASCII grids read so far.
    filecache: Mutex<HashMap<PathBuf, Arc<Vec<u8>>>>,
    /// `extract`: held while a download is unzipped, so a lookup does not read half a tile.
    pub(crate) extract: Mutex<()>,
    /// `requestSemaphore`.
    pub(crate) semaphore: Semaphore,
    /// `requestThreadrun`.
    pub(crate) run: AtomicBool,
    /// Set when the [`Srtm`] is dropped, so a wait on the semaphore gives up.
    pub(crate) stopping: AtomicBool,
    /// `GMaps.Instance.Mode == AccessMode.CacheOnly`.
    cache_only: AtomicBool,
}

/// Mission Planner's terrain lookup: `srtm`'s static state and methods, with its download thread.
///
/// The C# class is all statics - one cache directory, one queue, one thread per process. Here
/// that is one value, which the application makes once and shares; a test makes as many as it
/// likes, each with its own directory and its own [`Http`]. Dropping it stops its thread, as
/// `Dispose` does (srtm.cs:757-760).
pub struct Srtm {
    inner: Arc<Inner>,
}

impl std::fmt::Debug for Srtm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Srtm")
            .field("datadirectory", &self.inner.datadirectory)
            .field("queue", &*lock(&self.inner.queue))
            .finish_non_exhaustive()
    }
}

impl Srtm {
    /// A lookup over `datadirectory` that downloads from `terrain.ardupilot.org`, with its
    /// download thread running.
    ///
    /// Mission Planner's `datadirectory` is [`crate::srtm_directory`] (MainV2.cs:737).
    #[must_use]
    pub fn new(datadirectory: impl Into<PathBuf>) -> Self {
        Self::with_http(datadirectory, Arc::new(UreqHttp::new()))
    }

    /// The same, fetching through `http` - a test's server, or a real client through a proxy.
    ///
    /// The thread starts here, as the C#'s starts in the static constructor (srtm.cs:94-102).
    #[must_use]
    pub fn with_http(datadirectory: impl Into<PathBuf>, http: Arc<dyn Http>) -> Self {
        let srtm = Self::without_thread(datadirectory, http);
        let inner = Arc::clone(&srtm.inner);
        // C#: requestThreadrun = true, at the top of requestRunner (srtm.cs:530). Set before the
        // thread starts rather than in it, so a drop that comes first is not undone.
        inner.run.store(true, Ordering::Release);
        let spawned = wasm_thread::Builder::new()
            .name("mp-terrain".to_owned())
            .spawn(move || crate::fetch::request_runner(&inner));
        // C#: StartQueueProcess's task. A thread that cannot be started leaves the queue unread;
        // every lookup still answers from the tiles already on disk.
        drop(spawned);
        srtm
    }

    /// A lookup with no download thread: the queue fills and nothing takes from it until
    /// [`Srtm::run_queue_once`] is called. For tests, and for a caller that schedules the work
    /// itself.
    #[must_use]
    pub fn without_thread(datadirectory: impl Into<PathBuf>, http: Arc<dyn Http>) -> Self {
        Self {
            inner: Arc::new(Inner {
                datadirectory: datadirectory.into(),
                http,
                servers: Mutex::new(Servers {
                    baseurl1sec: BASEURL1SEC.to_owned(),
                    baseurl: BASEURL.to_owned(),
                }),
                queue: Mutex::new(Vec::new()),
                oceantile: Mutex::new(Vec::new()),
                cache: Mutex::new(HashMap::new()),
                filecache: Mutex::new(HashMap::new()),
                extract: Mutex::new(()),
                // C#: new SemaphoreSlim(1) - one count before anything is queued.
                semaphore: Semaphore::new(1),
                run: AtomicBool::new(false),
                stopping: AtomicBool::new(false),
                cache_only: AtomicBool::new(false),
            }),
        }
    }

    /// `srtm.datadirectory`.
    #[must_use]
    pub fn datadirectory(&self) -> &Path {
        &self.inner.datadirectory
    }

    /// Whether the map is offline: GMap.NET's `AccessMode.CacheOnly`, which Mission Planner sets
    /// from the "mapCache" setting (Program.cs:323, ConfigPlanner.cs:1166). A missing tile is
    /// then never queued.
    /// `// C#: ExtLibs/Utilities/srtm.cs:385`
    pub fn set_cache_only(&self, cache_only: bool) {
        self.inner.cache_only.store(cache_only, Ordering::Release);
    }

    /// See [`Srtm::set_cache_only`].
    #[must_use]
    pub fn cache_only(&self) -> bool {
        self.inner.cache_only.load(Ordering::Acquire)
    }

    /// `srtm.baseurl1sec =`.
    pub fn set_baseurl1sec(&self, url: impl Into<String>) {
        lock(&self.inner.servers).baseurl1sec = url.into();
    }

    /// `srtm.baseurl =`.
    pub fn set_baseurl(&self, url: impl Into<String>) {
        lock(&self.inner.servers).baseurl = url.into();
    }

    /// `srtm.baseurl1sec`.
    #[must_use]
    pub fn baseurl1sec(&self) -> String {
        lock(&self.inner.servers).baseurl1sec.clone()
    }

    /// `srtm.baseurl`.
    #[must_use]
    pub fn baseurl(&self) -> String {
        lock(&self.inner.servers).baseurl.clone()
    }

    /// The tiles waiting for the download thread, the one at the head being fetched. Private in
    /// the C#; what a caller that wants to wait for a download watches.
    #[must_use]
    pub fn queued(&self) -> Vec<String> {
        lock(&self.inner.queue).clone()
    }

    /// One pass of the download thread's loop, without its one-second pause: wait for the
    /// semaphore (up to 30 seconds), fetch the tile at the head of the queue, and take it off the
    /// queue - unless fetching it failed, when it stays at the head.
    /// `// C#: ExtLibs/Utilities/srtm.cs:534-566`
    pub fn run_queue_once(&self) {
        crate::fetch::request_step(&self.inner);
    }

    /// `srtm.getAltitude`: the ground at a point.
    ///
    /// Never waits on the network. `zoom` is the map's zoom; below [`DOWNLOAD_ZOOM`] a missing tile
    /// is not queued, and it does nothing else. [`DEFAULT_ZOOM`] is the C#'s default.
    ///
    /// The C# first asks `GeoTiff.getAltitude` and `DTED.getAltitude` (srtm.cs:120-150), which
    /// answer from `*.tif` and `*.dt0`-`*.dt2` files a user has put in the same directory and are
    /// otherwise `Invalid`. Not ported here: `GeoTiff.cs` and `DTED.cs` are their own ports (a
    /// TIFF reader and a DTED reader, 1,100 lines between them), and with neither kind of file in
    /// the directory - which is every installation that has not added one by hand - the C# goes
    /// on to exactly what follows.
    /// `// C#: ExtLibs/Utilities/srtm.cs:116-407`
    #[must_use]
    pub fn get_altitude(&self, lat: f64, lng: f64, zoom: f64) -> AltResponse {
        // C#: ExtLibs/Utilities/srtm.cs:159-162
        let Some(filename) = tile_name(lat, lng) else {
            return AltResponse::INVALID;
        };
        // C#: the try at srtm.cs:164, and its catch at 400-404.
        self.inner
            .lookup(lat, lng, zoom, &filename)
            .unwrap_or(AltResponse::INVALID)
    }
}

impl Drop for Srtm {
    /// `Dispose`: `requestThreadrun = false`. The C#'s task notices on its next pass, up to 31
    /// seconds and possibly one more download later; here the semaphore's wait is woken and gives
    /// up, so the thread ends within its one-second pause, or when a fetch in progress returns.
    /// `// C#: ExtLibs/Utilities/srtm.cs:757-760`
    fn drop(&mut self) {
        self.inner.run.store(false, Ordering::Release);
        self.inner.stopping.store(true, Ordering::Release);
        self.inner.semaphore.wake_all();
    }
}

impl Inner {
    /// The body of `getAltitude`'s `try`, after the tile has a name.
    /// `// C#: ExtLibs/Utilities/srtm.cs:164-398`
    fn lookup(&self, lat: f64, lng: f64, zoom: f64, filename: &str) -> Result<AltResponse, Fault> {
        // C#: ExtLibs/Utilities/srtm.cs:166-168 - marked as an ocean tile.
        if lock(&self.oceantile).iter().any(|tile| tile == filename) {
            return Ok(AltResponse::OCEAN);
        }
        // C#: ExtLibs/Utilities/srtm.cs:170-177 - in the download queue: no answer until fetched.
        if lock(&self.queue).iter().any(|tile| tile == filename) {
            return Ok(AltResponse::INVALID);
        }

        let path = self.datadirectory.join(filename);
        let cached = lock(&self.cache).get(filename).cloned();
        // C#: File.Exists is false for a directory, and for anything it cannot look at.
        if cached.is_some() || path.os_is_file() {
            let tile = if let Some(tile) = cached {
                tile
            } else {
                // C#: lock (extract) { Thread.Sleep(0); } - wait out an unzip in progress.
                drop(lock(&self.extract));
                let Some(tile) = read_hgt(&path)? else {
                    return Ok(AltResponse::INVALID);
                };
                let tile = Arc::new(tile);
                lock(&self.cache).insert(filename.to_owned(), Arc::clone(&tile));
                tile
            };
            return interpolate(&tile, lat, lng);
        }

        let ascii = self.datadirectory.join(ascii_name(lat, lng));
        if ascii.os_is_file() {
            return self.ascii(&ascii, lat, lng);
        }

        // C#: ExtLibs/Utilities/srtm.cs:378-398 - "get something".
        if zoom >= DOWNLOAD_ZOOM {
            if !self.datadirectory.os_is_dir() {
                mp_os::fs::create_dir_all(&self.datadirectory)?;
            }
            if !self.cache_only.load(Ordering::Acquire) {
                let mut queue = lock(&self.queue);
                if !queue.iter().any(|tile| tile == filename) {
                    queue.push(filename.to_owned());
                    self.semaphore.release();
                }
            }
        }
        Ok(AltResponse::INVALID)
    }

    /// `readFile`: an ASCII grid's bytes, read once and kept.
    ///
    /// The C# keeps a `MemoryStream`, and the `StreamReader` that reads it disposes it
    /// (srtm.cs:288-290, 501-522): the second lookup in the same grid gets a closed stream, the
    /// `StreamReader` constructor throws, and the answer is `Invalid` - every lookup after the
    /// first, for the life of the process, which `testdata/srtm/oracle.txt` shows. Diverged: the
    /// bytes are kept and read from the start each time, which is what the cache is evidently for.
    fn read_file(&self, path: &Path) -> Result<Arc<Vec<u8>>, Fault> {
        if let Some(bytes) = lock(&self.filecache).get(path) {
            return Ok(Arc::clone(bytes));
        }
        let bytes = Arc::new(mp_os::fs::read(path)?);
        lock(&self.filecache).insert(path.to_owned(), Arc::clone(&bytes));
        Ok(bytes)
    }

    /// The CGIAR ASCII grid, read as the C# reads it.
    ///
    /// Not what the format means: `yllcorner` is the grid's bottom but the row counted from the
    /// top is `nrows - cells above the bottom`, one row south of the point, and a point in the
    /// bottom row matches no row at all - which answers a valid altitude of 0 sourced "ASCII".
    /// A cell that is found answers with an empty `altsource`, and `NODATA_value` is parsed and
    /// then ignored, so a void answers -9999 as valid ground.
    ///
    /// The header's numbers are parsed as the invariant culture reads them. The C# uses the
    /// machine's culture, so on a machine whose decimal separator is a comma it misreads
    /// `cellsize 0.000833333` - diverged, because no grid is written in a comma culture.
    /// `// C#: ExtLibs/Utilities/srtm.cs:286-377`
    fn ascii(&self, path: &Path, lat: f64, lng: f64) -> Result<AltResponse, Fault> {
        let bytes = self.read_file(path)?;
        let text = decode_utf8(&bytes);
        let mut nox: i32 = 0;
        let mut noy: i32 = 0;
        let mut left: f32 = 0.0;
        let mut top: f32 = 0.0;
        let mut cellsize: f32 = 0.0;
        let mut rowcounter: i32 = 0;

        for line in read_lines(&text) {
            if line.starts_with("ncols") {
                nox = cs_int_parse(after_space(line)?)?;
            } else if line.starts_with("nrows") {
                noy = cs_int_parse(after_space(line)?)?;
            } else if line.starts_with("xllcorner") {
                left = cs_float_parse(after_space(line)?)?;
            } else if line.starts_with("yllcorner") {
                top = cs_float_parse(after_space(line)?)?;
            } else if line.starts_with("cellsize") {
                cellsize = cs_float_parse(after_space(line)?)?;
            } else if line.starts_with("NODATA_value") {
                // C#: nodata is parsed - a bad one still throws - and never used.
                cs_int_parse(after_space(line)?)?;
            } else {
                let data: Vec<&str> = line.split(' ').collect();
                // C#: data.Length == (nox + 1), in int arithmetic.
                if i32::try_from(data.len()).ok() == Some(nox.wrapping_add(1)) {
                    // C#: (float)(lng - Math.Round(left, 0)), then float arithmetic - mono does
                    // float / float in single precision (the `fdiv` records of oracle.txt).
                    #[allow(clippy::cast_possible_truncation)] // the C#'s (float) casts
                    let wantcol = (lng - math_round(f64::from(left))) as f32;
                    #[allow(clippy::cast_possible_truncation)]
                    let wantrow = (lat - math_round(f64::from(top))) as f32;
                    #[allow(clippy::cast_precision_loss)] // int to float, as the C# assigns it
                    let wantrow = cs_int(f64::from(wantrow / cellsize)) as f32;
                    #[allow(clippy::cast_precision_loss)]
                    let wantcol = cs_int(f64::from(wantcol / cellsize)) as f32;
                    #[allow(clippy::cast_precision_loss)]
                    let wantrow = noy as f32 - wantrow;

                    #[allow(clippy::cast_precision_loss, clippy::float_cmp)]
                    if rowcounter as f32 == wantrow {
                        // C#: Console.WriteLine of the lookup - a debug line on a console a
                        // WinForms application does not have. Not ported.
                        let column =
                            usize::try_from(cs_int(f64::from(wantcol))).map_err(|_| Fault)?;
                        let value = data.get(column).ok_or(Fault)?;
                        return Ok(AltResponse {
                            current_type: TileType::Valid,
                            alt: f64::from(cs_int_parse(value)?),
                            alt_source: "",
                        });
                    }
                    rowcounter = rowcounter.wrapping_add(1);
                }
            }
        }

        // C#: ExtLibs/Utilities/srtm.cs:371-376 - alt is the method's `short alt = 0`.
        Ok(AltResponse {
            current_type: TileType::Valid,
            alt: 0.0,
            alt_source: "ASCII",
        })
    }
}

/// The bilinear lookup in a cached tile.
///
/// The fraction of the degree east picks the column, the fraction north the row counted up from
/// the south edge, flipped because the file's rows run north to south; the four samples around the
/// point are blended along the row, then between the rows. Any blend below -1000 - a void of
/// -32768 with any weight at all, near enough - is no answer. A point so close below a whole
/// degree that its fraction rounds to 1 reads one sample past the edge, which the C#'s array
/// refuses, and so does this.
/// `// C#: ExtLibs/Utilities/srtm.cs:233-280`
fn interpolate(tile: &Tile, mut lat: f64, mut lng: f64) -> Result<AltResponse, Fault> {
    // C#: ExtLibs/Utilities/srtm.cs:233-242 - the size again, from the cached array's length.
    let size: i32 = match tile.data.len() {
        n if n == SIZE_3SEC * SIZE_3SEC => 1201,
        n if n == SIZE_1SEC * SIZE_1SEC => 3601,
        _ => return Ok(AltResponse::INVALID),
    };

    let x = floor_int(lng);
    let y = floor_int(lat);

    // remove the base lat long
    lat -= f64::from(y);
    lng -= f64::from(x);

    // values should be 0-1199, 1200 is for interpolation
    let xf = lng * f64::from(size - 1);
    let yf = lat * f64::from(size - 1);

    let x_int = cs_int(xf);
    let x_frac = xf - f64::from(x_int);

    let y_int = cs_int(yf);
    let y_frac = yf - f64::from(y_int);

    let y_int = (size - 2).wrapping_sub(y_int);

    let alt00 = tile.get(x_int, y_int)?;
    let alt10 = tile.get(x_int.wrapping_add(1), y_int)?;
    let alt01 = tile.get(x_int, y_int.wrapping_add(1))?;
    let alt11 = tile.get(x_int.wrapping_add(1), y_int.wrapping_add(1))?;

    let v1 = avg(alt00, alt10, x_frac);
    let v2 = avg(alt01, alt11, x_frac);
    let v = avg(v1, v2, 1.0 - y_frac);

    if v < -1000.0 {
        return Ok(AltResponse::INVALID);
    }

    Ok(AltResponse {
        current_type: TileType::Valid,
        alt: v,
        alt_source: "SRTM",
    })
}

/// `StreamReader`'s decoding: UTF-8, a byte-order mark dropped, bad bytes replaced.
pub(crate) fn decode_utf8(bytes: &[u8]) -> String {
    let bytes = bytes.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(bytes);
    String::from_utf8_lossy(bytes).into_owned()
}

/// `StreamReader.ReadLine` until `EndOfStream`: lines end at `\r\n`, `\n` or `\r`, and a final
/// line without an ending is still a line - but the end of the text is not an empty one.
pub(crate) fn read_lines(text: &str) -> Vec<&str> {
    let mut lines = Vec::new();
    let mut rest = text;
    while !rest.is_empty() {
        match rest.find(['\r', '\n']) {
            Some(at) => {
                lines.push(rest.get(..at).unwrap_or_default());
                let after = rest.get(at..).unwrap_or_default();
                rest = after
                    .strip_prefix("\r\n")
                    .or_else(|| after.get(1..))
                    .unwrap_or_default();
            }
            None => {
                lines.push(rest);
                rest = "";
            }
        }
    }
    lines
}

/// `line.Substring(line.IndexOf(' '))`: from the first space on, which throws when there is none.
fn after_space(line: &str) -> Result<&str, Fault> {
    line.find(' ').and_then(|at| line.get(at..)).ok_or(Fault)
}

/// .NET's white space for `NumberStyles.AllowLeadingWhite` and `AllowTrailingWhite`: tab, line
/// feed, vertical tab, form feed, carriage return and space.
fn trim_number(text: &str) -> &str {
    text.trim_matches(|c| matches!(c, '\t' | '\n' | '\u{b}' | '\u{c}' | '\r' | ' '))
}

/// `int.Parse`: optional white space, an optional sign, digits.
fn cs_int_parse(text: &str) -> Result<i32, Fault> {
    trim_number(text).parse().map_err(|_| Fault)
}

/// `float.Parse`, as the invariant culture reads a plain decimal: parsed as a double and narrowed,
/// as mono's `Single.Parse` does.
fn cs_float_parse(text: &str) -> Result<f32, Fault> {
    let value: f64 = trim_number(text).parse().map_err(|_| Fault)?;
    #[allow(clippy::cast_possible_truncation)] // the C#'s float
    Ok(value as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The records of `testdata/srtm/oracle.txt` whose first word is `kind`, split on spaces.
    fn records(kind: &str) -> Vec<Vec<String>> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/srtm/oracle.txt");
        let text = mp_os::fs::read_to_string(path).unwrap();
        text.lines()
            .map(|line| line.split(' ').map(str::to_owned).collect::<Vec<_>>())
            .filter(|fields| fields.first().map(String::as_str) == Some(kind))
            .collect()
    }

    #[test]
    fn math_round_and_format_are_monos() {
        let records = records("format");
        assert!(records.len() >= 20, "{records:?}");
        for record in records {
            let value: f64 = record[1].parse().unwrap();
            assert_eq!(
                format_00(math_round(value)),
                record[2],
                "Math.Round({value}, 0)"
            );
        }
    }

    #[test]
    fn the_int_conversion_is_monos() {
        let records = records("cast");
        assert!(records.len() >= 14, "{records:?}");
        for record in records {
            let value: f64 = record[1].parse().unwrap();
            let expected: i32 = record[2].parse().unwrap();
            assert_eq!(floor_int(value), expected, "(int)Math.Floor({value})");
        }
    }

    #[test]
    fn float_division_is_single_precision_as_monos() {
        let records = records("fdiv");
        assert!(records.len() >= 7, "{records:?}");
        for record in records {
            let a = cs_float_parse(&record[1]).unwrap();
            let b = cs_float_parse(&record[2]).unwrap();
            let expected: i32 = record[3].parse().unwrap();
            assert_eq!(cs_int(f64::from(a / b)), expected, "(int)({a}f / {b}f)");
        }
    }

    #[test]
    fn tile_names_floor_both_coordinates() {
        assert_eq!(tile_name(-35.1, 149.2).as_deref(), Some("S36E149.hgt"));
        assert_eq!(tile_name(35.0, -0.5).as_deref(), Some("N35W001.hgt"));
        assert_eq!(tile_name(-0.0, -0.0).as_deref(), Some("N00E000.hgt"));
        assert_eq!(tile_name(90.9, 180.9).as_deref(), Some("N90E180.hgt"));
        assert_eq!(tile_name(-90.0, -180.0).as_deref(), Some("S90W180.hgt"));
        assert_eq!(tile_name(-90.1, 0.0), None);
        assert_eq!(tile_name(0.0, 181.0), None);
        assert_eq!(tile_name(f64::NAN, 0.0), None);
    }

    #[test]
    fn the_ascii_name_rounds_half_to_even() {
        // (20 + 182.5) / 5 = 40.5 rounds to 40; 20.1 is past it.
        assert_eq!(ascii_name(10.0, 20.0), "srtm_40_10.asc");
        assert_eq!(ascii_name(10.0, 20.1), "srtm_41_10.asc");
        // Past 62.5 north the row is negative, and a rounded -0 prints unsigned.
        assert_eq!(ascii_name(64.0, 0.0), "srtm_36_00.asc");
        assert_eq!(ascii_name(90.0, 0.0), "srtm_36_-06.asc");
    }

    #[test]
    fn lines_end_as_stream_reader_ends_them() {
        assert_eq!(read_lines("a\r\nb\nc\rd"), vec!["a", "b", "c", "d"]);
        assert_eq!(read_lines("a\n\nb\n"), vec!["a", "", "b"]);
        assert_eq!(read_lines(""), Vec::<&str>::new());
        assert_eq!(read_lines(" "), vec![" "]);
        assert_eq!(decode_utf8(b"\xEF\xBB\xBFx"), "x");
    }

    #[test]
    fn numbers_parse_as_dotnet_parses_them() {
        assert_eq!(cs_int_parse("         10").unwrap(), 10);
        assert_eq!(cs_int_parse(" -9999\t").unwrap(), -9999);
        assert_eq!(cs_int_parse("+7").unwrap(), 7);
        assert!(cs_int_parse("").is_err());
        assert!(cs_int_parse("1.5").is_err());
        assert!(cs_int_parse("2147483648").is_err());
        assert_eq!(cs_float_parse("   0.5").unwrap(), 0.5);
        assert!(after_space("ncols").is_err());
        assert_eq!(after_space("ncols 5").unwrap(), " 5");
    }

    #[test]
    fn a_sample_past_the_edge_is_an_error_not_the_next_row() {
        let tile = Tile {
            size: 3,
            data: (0..9).collect(),
        };
        assert_eq!(tile.get(2, 0).unwrap(), 2.0);
        assert_eq!(tile.get(0, 1).unwrap(), 3.0);
        assert!(tile.get(3, 0).is_err());
        assert!(tile.get(0, 3).is_err());
        assert!(tile.get(-1, 0).is_err());
    }
}
