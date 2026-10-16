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

//! GMap.NET's page cache, `Cache`'s "etc cache": the URL cache, pages the providers download to
//! learn their current tile version, and the geocoder cache, OpenStreetMap's answers to Map Tool >
//! Zoom To. (The module is named for the first of the two it held.)
//!
//! Ported from `ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:177-266` (`SaveContent`,
//! `GetContent` and `CacheType`). It lives beside the tiles, in `gmapcache/UrlCache/` and
//! `gmapcache/GeocoderCache/`, because the directory GMap.NET falls back to is
//! `CacheLocator.Location`, which Mission Planner sets to the tile cache when the planning screen
//! is built (`GCSViews/FlightPlanner.cs:151-152`, via
//! `GMap.NET.WindowsForms/GMapControl.cs:2785-2790`). So a real installation has, for example,
//! `~/.local/share/Mission Planner/gmapcache/UrlCache/2E-E7-82-...-F9.txt` holding the Google Maps
//! loader script the C# last downloaded - and this module names, reads and writes that same file.
//! The C# keeps its tiles in SQLite but these pages as a file each, so a file each is its layout
//! here too, with no database.
//!
//! A version page is trusted for eight hours, the time its callers name; a geocoder page for 88
//! days, `GetContent`'s own default, which the geocoder takes (`Cache.cs:251-254`;
//! `GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:293`). A read never refreshes
//! either: the time runs from the download.
//!
//! **A read before the first write of a process always misses, as it does in the C#.** `Cache`'s
//! constructor returns before it sets its location (`Cache.cs:119`), nothing else in Mission Planner
//! sets `Cache.Instance.CacheLocation`, and `GetContent` answers `null` while the location is unset
//! (`Cache.cs:220-221`). Only `SaveContent` sets it (`Cache.cs:188-189`). The effect is that the
//! first version check of a run always goes to the network, and a later check in the same run - Bing
//! after Google, say - can be answered from a file written within the last eight hours. That is
//! modelled here with one process-wide flag rather than "fixed", because what the C# sends to the
//! network, and when, is the behaviour being ported. The flag is one for both kinds of page, as
//! `cache` is one field of the one `Cache`: a version page saved at start makes the geocoder's
//! earlier answers readable, and the first Zoom To of a run in which nothing has been saved - a map
//! with no version check, or no network for one - goes to the server whatever is on disk.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use web_time::{Duration, SystemTime};

/// The directory under the cache root, `CacheType.UrlCache.ToString()`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:191, 259-266`
pub const DIRECTORY: &str = "UrlCache";

/// The geocoder's directory under the cache root, `CacheType.GeocoderCache.ToString()`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:191, 259-266`
pub const GEOCODER_DIRECTORY: &str = "GeocoderCache";

/// How long a page is trusted when its caller names no time, `GetContent(url, type)`'s
/// `TimeSpan.FromDays(88)`: the geocoder's.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:251-254;
/// GMap.NET.MapProviders/OpenStreetMap/OpenStreetMapProvider.cs:293`
pub const DEFAULT_STAY_IN_CACHE: Duration = Duration::from_secs(88 * 24 * 60 * 60);

/// How long a version page is trusted, `TimeSpan.FromHours(8)` at both call sites.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:104;
/// Bing/BingMapProvider.cs:159`
pub const STAY_IN_CACHE: Duration = Duration::from_secs(8 * 60 * 60);

/// The UTF-8 byte-order mark `StreamWriter(file, false, Encoding.UTF8)` writes first.
const BOM: &str = "\u{feff}";

/// Whether this process has saved anything yet, of either kind - the C#'s `Cache.cache` being
/// non-null.
static LOCATION_KNOWN: AtomicBool = AtomicBool::new(false);

/// `CacheType`, as far as Mission Planner keeps pages: the version pages and the geocoder's
/// answers. `PlacemarkCache`, `RouteCache` and `DirectionsCache` are GMap.NET's reverse geocoding
/// and routing, which nothing in Mission Planner calls.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:259-266`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CacheType {
    /// `UrlCache`: Google's and Bing's version pages.
    UrlCache,
    /// `GeocoderCache`: OpenStreetMap's geocoder pages, Zoom To's.
    GeocoderCache,
}

impl CacheType {
    /// The directory under the cache root: the member's name, as `type.ToString()` gives it.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:191, 223`
    #[must_use]
    pub const fn directory(self) -> &'static str {
        match self {
            Self::UrlCache => DIRECTORY,
            Self::GeocoderCache => GEOCODER_DIRECTORY,
        }
    }
}

/// The file name a URL is cached under.
///
/// `BitConverter.ToString(SHA1(Encoding.Unicode.GetBytes(url)))` plus `.txt`: the SHA-1 of the URL
/// as UTF-16LE, written as upper-case hex pairs joined by `-`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:175-180, 199`
#[must_use]
pub fn file_name(url: &str) -> String {
    let utf16: Vec<u8> = url.encode_utf16().flat_map(u16::to_le_bytes).collect();
    let digest = sha1_smol::Sha1::from(utf16).digest().bytes();
    let pairs: Vec<String> = digest.iter().map(|byte| format!("{byte:02X}")).collect();
    format!("{}.txt", pairs.join("-"))
}

/// Where a URL's page is cached under a `gmapcache` root, in the URL cache.
#[must_use]
pub fn path(root: &Path, url: &str) -> PathBuf {
    content_path(root, CacheType::UrlCache, url)
}

/// Where a URL's page of one kind is cached under a `gmapcache` root: `<root>/<type>/<SHA-1>.txt`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:191, 199, 223-224`
#[must_use]
pub fn content_path(root: &Path, kind: CacheType, url: &str) -> PathBuf {
    root.join(kind.directory()).join(file_name(url))
}

/// `Cache.GetContent` on the URL cache.
#[must_use]
pub fn get(root: &Path, url: &str, stay: Duration) -> Option<String> {
    get_content(root, CacheType::UrlCache, url, stay)
}

/// `Cache.GetContent`: a page saved less than `stay` ago, if this process has saved anything yet.
///
/// A page older than `stay` is deleted, as the C# deletes it, and answers nothing. The byte-order
/// mark the C# writes is not part of the page: `StreamReader(file, Encoding.UTF8)` strips it.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:212-249`
#[must_use]
pub fn get_content(root: &Path, kind: CacheType, url: &str, stay: Duration) -> Option<String> {
    if !LOCATION_KNOWN.load(Ordering::Acquire) {
        return None;
    }
    let file = content_path(root, kind, url);
    let written = mp_os::fs::metadata(&file).ok()?.modified().ok()?;
    let age = SystemTime::now()
        .duration_since(written)
        .unwrap_or(Duration::ZERO);
    if age >= stay {
        let _ = mp_os::fs::remove_file(&file);
        return None;
    }
    let text = mp_os::fs::read_to_string(&file).ok()?;
    Some(text.strip_prefix(BOM).unwrap_or(&text).to_owned())
}

/// `Cache.SaveContent` on the URL cache.
pub fn save(root: &Path, url: &str, content: &str) {
    save_content(root, CacheType::UrlCache, url, content);
}

/// `Cache.SaveContent`: writes a page, as UTF-8 with a byte-order mark, creating the directory.
///
/// Failure is swallowed, as the C# swallows it: a page that could not be cached is fetched again
/// next time, which is no worse than having no cache. Either way this process now knows where the
/// cache is, which is what lets a later [`get_content`] find anything, of either kind.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Cache.cs:182-210`
pub fn save_content(root: &Path, kind: CacheType, url: &str, content: &str) {
    LOCATION_KNOWN.store(true, Ordering::Release);
    let file = content_path(root, kind, url);
    if let Some(directory) = file.parent() {
        let _ = mp_os::fs::create_dir_all(directory);
    }
    let Ok(mut out) = mp_os::fs::File::create(&file) else {
        return;
    };
    let _ = out
        .write_all(BOM.as_bytes())
        .and_then(|()| out.write_all(content.as_bytes()));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The loader script's cache file on a real installation of the C# application on Linux is
    /// `2E-E7-82-36-A4-AA-AE-AE-3F-46-79-C8-08-CA-12-13-20-7F-22-F9.txt` - read off this machine's
    /// `~/.local/share/Mission Planner/gmapcache/UrlCache/`, and equal to
    /// `printf '%s' "$url" | iconv -t UTF-16LE | sha1sum`.
    #[test]
    fn a_url_is_named_as_the_csharp_names_it() {
        assert_eq!(
            file_name("http://maps.google.com/maps/api/js?v=3.2&sensor=false"),
            "2E-E7-82-36-A4-AA-AE-AE-3F-46-79-C8-08-CA-12-13-20-7F-22-F9.txt"
        );
    }

    #[test]
    fn the_name_is_of_the_utf16_bytes_not_the_utf8_ones() {
        // SHA-1 of the UTF-8 bytes of "a" is 86F7E437...; of the UTF-16LE bytes 61 00 it is not.
        assert!(
            !file_name("a").starts_with("86-F7-E4-37"),
            "{}",
            file_name("a")
        );
        assert_eq!(file_name("a").len(), 20 * 3 - 1 + ".txt".len());
    }

    /// Each kind in its own directory, named as its `CacheType` member; the geocoder's pages for
    /// 88 days, `GetContent`'s default.
    #[test]
    fn each_kind_is_kept_in_its_members_directory() {
        let root = Path::new("/tmp/gmapcache");
        assert_eq!(
            content_path(root, CacheType::GeocoderCache, "u"),
            root.join("GeocoderCache").join(file_name("u"))
        );
        assert_eq!(
            content_path(root, CacheType::UrlCache, "u"),
            root.join("UrlCache").join(file_name("u"))
        );
        assert_eq!(path(root, "u"), content_path(root, CacheType::UrlCache, "u"));
        assert_eq!(DEFAULT_STAY_IN_CACHE, Duration::from_secs(7_603_200));
    }
}
