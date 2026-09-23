//! Terrain altitude: Mission Planner's `srtm` class (`ExtLibs/Utilities/srtm.cs`,
//! GPL-3.0-or-later).
//!
//! What the planner's status line, the terrain-following checks and the elevation graph ask: "how
//! high is the ground here?" The answer comes from SRTM tiles - one-degree squares of big-endian
//! 16-bit heights, `N35E149.hgt` and the like - kept in `<data directory>/srtm` and downloaded from
//! `terrain.ardupilot.org` on first use. [`Srtm::get_altitude`] never waits for a download: a tile
//! it does not have is queued for the download thread and the answer is
//! [`AltResponse::INVALID`] until the thread has it, exactly as in the C#, where the next mouse
//! move asks again.
//!
//! Held to the C# itself: `tools/csharp-reference/SrtmOracle.cs` drives `srtm.getAltitude` under
//! mono - its download queue included, against a local server - and `tests/oracle.rs` replays
//! every question and every answer, to the bit.
//!
//! Some of what the C# does is not what its names suggest, and all of it is kept unless noted at
//! the site:
//!
//! - the tile name floors both coordinates, so `-27.5, 153.5` is `S28E153`, and a value .NET cannot
//!   convert to an `int` (NaN, the infinities, anything past +-2^31) becomes `int.MinValue` and so
//!   no tile at all;
//! - `zoom` does nothing but decide whether a missing tile is queued: below 7 it is not;
//! - "ocean" is not a list of known sea tiles: a tile becomes ocean only after the download thread
//!   has walked every server listing (at least nine, more than 38000 names) without finding it, and
//!   only for the life of the process;
//! - a server that fails leaves its tile at the head of the queue, retried every 31 seconds, and
//!   every tile queued behind it waits;
//! - the ASCII-grid fallback (`srtm_XX_YY.asc`) answers a found cell with an empty `altsource` and
//!   a missing one with a valid altitude of 0 sourced "ASCII".

mod fetch;
mod srtm;

pub use fetch::{Http, HttpError, UreqHttp};
pub use srtm::{
    AltResponse, BASEURL, BASEURL1SEC, DEFAULT_ZOOM, DOWNLOAD_ZOOM, Srtm, TileType, ascii_name,
    tile_name,
};

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

/// How we identify ourselves to the terrain server: the one identity the whole application sends.
///
/// `srtm`'s `HttpClient` sends `Settings.Instance.UserAgent` (srtm.cs:98-99), which Mission
/// Planner sets at startup to its product name, version and operating system - the same string it
/// gives GMap.NET for the map tiles (`Program.cs:373-375`). So this is `mp-tiles`' `USER_AGENT`,
/// and a test holds the two together.
pub const USER_AGENT: &str = concat!(
    "MissionPlannerRust/",
    env!("CARGO_PKG_VERSION"),
    " (+https://github.com/davidbuzz/MissionPlannerRust)"
);

/// The terrain cache: `Settings.GetDataDirectory()` + `srtm`.
/// `// C#: MainV2.cs:737`
#[must_use]
pub fn srtm_directory_in(folders: &mp_settings::Folders) -> PathBuf {
    folders.data_directory().join("srtm")
}

/// The terrain cache for this process, if there is a home directory to derive it from.
/// `// C#: MainV2.cs:737`
#[must_use]
pub fn srtm_directory() -> Option<PathBuf> {
    mp_settings::Folders::from_environment().map(|folders| srtm_directory_in(&folders))
}

/// The files older than this are what `MainV2` deletes from the terrain cache at startup:
/// 2026-03-01T00:00:00Z, the fix for a bad SRTM3 set the server once served.
/// `// C#: MainV2.cs:745-747`
pub const STALE_BEFORE_UNIX_SECONDS: u64 = 1_772_323_200;

/// `MainV2`'s startup sweep of the terrain cache: every file directly in it that is empty, or
/// was last written before [`STALE_BEFORE_UNIX_SECONDS`], is deleted.
///
/// Mission Planner runs it once, right after pointing `srtm` at the directory, so an empty
/// listing or a truncated tile left by a failed download is fetched again rather than trusted
/// (`get3secfile` skips any tile file that is not empty, srtm.cs:583-588). As in the C#, the first
/// thing that goes wrong - a directory that is not there, a file that cannot be deleted - ends the
/// sweep quietly.
/// `// C#: MainV2.cs:739-750`
pub fn clean_cache_directory(dir: &Path) {
    let stale = SystemTime::UNIX_EPOCH + Duration::from_secs(STALE_BEFORE_UNIX_SECONDS);
    let sweep = || -> std::io::Result<()> {
        // C#: Directory.GetFiles(dir) - the files directly in it, not what is under them.
        let mut files = Vec::new();
        for entry in std::fs::read_dir(dir)? {
            let path = entry?.path();
            if path.is_file() {
                files.push(path);
            }
        }
        for path in files {
            // C#: FileInfo reads the length and the time once; both tests use that one reading.
            let meta = std::fs::metadata(&path)?;
            if meta.len() == 0 {
                delete(&path)?;
            }
            if meta.modified()? < stale {
                delete(&path)?;
            }
        }
        Ok(())
    };
    // C#: catch { } around the whole sweep.
    let _ = sweep();
}

/// `File.Delete`: a file that is already gone is not an error.
fn delete(path: &Path) -> std::io::Result<()> {
    match std::fs::remove_file(path) {
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        other => other,
    }
}
