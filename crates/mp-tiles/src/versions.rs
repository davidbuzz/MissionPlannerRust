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

//! The version in Google's and Bing's tile URLs, and how GMap.NET keeps it current.
//!
//! Both providers put a version into every tile URL - `v=955` for Google's imagery, `g=4810` for
//! Bing's - and both ship a hard-coded one that was current when GMap.NET was last edited. The first
//! time a provider of either family is shown, its `OnInitialized` downloads a page the provider
//! publishes, finds the current version in it with a regular expression, and overwrites the
//! hard-coded one for every provider in the family. If the page cannot be had, the hard-coded one
//! stays, and the tiles are asked for with it.
//!
//! Ported as the C# does it: the page is looked for in the URL cache first ([`crate::urlcache`]),
//! fetched if it is not there, saved if it was fetched, and matched with the C#'s own patterns. Each
//! family tries once per process until it succeeds; a failed attempt is tried again the next time a
//! provider of that family that has not been shown before is chosen.
//!
//! Where it runs is the one deliberate difference. The C# runs Google's check on a task of its own
//! (`Task.Run`) and Bing's on the UI thread, so choosing a Bing map freezes the window for the
//! length of a request. Here both run off the render thread: Google's on a thread of its own, as in
//! the C#, and Bing's on the tile thread before its first fetch - so, as in the C#, no Bing tile is
//! requested before the check has finished, and the window does not freeze while it runs.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::fetch::{FetchError, TileFetcher};
use crate::source::{
    BING_HYBRID_MAP, BING_MAP, BING_SATELLITE_MAP, GOOGLE_MAP, GOOGLE_SATELLITE_MAP,
    GOOGLE_TERRAIN_MAP, TileSource,
};
use crate::urlcache;

/// Which `OnInitialized` a provider runs the first time it is shown.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Correction {
    /// The base class's, which does nothing.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:282-285`
    None,
    /// `GoogleMapProviderBase.OnInitialized`.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:92-238`
    Google,
    /// `BingMapProviderBase.OnInitialized`.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:149-250`
    Bing,
}

/// The page Google's version is read from.
///
/// Line 99 builds `http://maps.google.com` from the decoded server name and line 100 immediately
/// replaces it with this.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:99-100`
pub const GOOGLE_PAGE: &str = "http://maps.google.com/maps/api/js?v=3.2&sensor=false";

/// The page Bing's version is read from.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:158`
pub const BING_PAGE: &str = "http://www.bing.com/maps";

/// Versions found by a check, by the C# provider's `Name` - the `GMapProviders.X.Version = ver`
/// assignments. A provider not in here uses its hard-coded version.
static CORRECTED: Mutex<BTreeMap<&'static str, String>> = Mutex::new(BTreeMap::new());

/// Providers whose `OnInitialized` has run: `GMapProvider.IsInitialized`, set by the map core the
/// first time a provider is shown.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.Internals/Core.cs:204-208`
static INITIALIZED: Mutex<BTreeSet<&'static str>> = Mutex::new(BTreeSet::new());

/// `GoogleMapProviderBase.init`: the Google check has succeeded in this process.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:90, 228`
static GOOGLE_DONE: AtomicBool = AtomicBool::new(false);

/// `BingMapProviderBase.init`: the Bing check has succeeded in this process.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:147, 211`
static BING_DONE: AtomicBool = AtomicBool::new(false);

/// The version a provider's URLs carry now: the one a check found, or its hard-coded one.
#[must_use]
pub fn current(source: &TileSource) -> Cow<'static, str> {
    CORRECTED
        .lock()
        .ok()
        .and_then(|corrected| corrected.get(source.cache_name).cloned())
        .map_or(Cow::Borrowed(source.version), Cow::Owned)
}

/// Records a version found for a provider.
fn set(source: &TileSource, version: String) {
    if let Ok(mut corrected) = CORRECTED.lock() {
        corrected.insert(source.cache_name, version);
    }
}

/// What Google's loader script says the current versions are.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct GoogleVersions {
    /// `m@...`, for `GoogleMap`.
    pub map: Option<String>,
    /// `h@...`, for `GoogleHybridMap` - found, as the C# finds it, but there is no hybrid provider
    /// here to give it to.
    pub hybrid: Option<String>,
    /// The bare number, for `GoogleSatelliteMap`.
    pub satellite: Option<String>,
    /// `t@...,r@...`, for `GoogleTerrainMap`.
    pub terrain: Option<String>,
}

/// The first capture groups of a .NET `Regex(pattern, RegexOptions.IgnoreCase).Match(html)`.
///
/// The patterns are the C#'s, character for character. The `regex` crate's leftmost-first matching
/// gives the same match a backtracking engine does for patterns like these, and its `\d`, `\D` and
/// `.` mean what .NET's do: Unicode digits, their complement, and anything but a newline.
fn first_match(pattern: &str, html: &str) -> Option<Vec<String>> {
    let regex = regex::Regex::new(&format!("(?i){pattern}")).ok()?;
    let captures = regex.captures(html)?;
    Some(
        captures
            .iter()
            .skip(1)
            .map(|group| group.map_or_else(String::new, |found| found.as_str().to_owned()))
            .collect(),
    )
}

/// The versions `GoogleMapProviderBase.OnInitialized` finds in the loader script.
///
/// Four patterns, each tried independently. Three of them look for `mt` hosts serving `vt` tiles,
/// which today's script no longer names - so today only the satellite version is found, and the
/// road and terrain maps keep their hard-coded versions in the C# as they do here.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:121-225`
#[must_use]
pub fn google_versions(html: &str) -> GoogleVersions {
    // The first argument of `string.Format` on lines 123, 148 and 200 is Google's server name, but
    // those patterns have no `{0}` for it to go into. Line 173's does, and it is "googleapis.com" -
    // whose dot is a regex wildcard, kept as the C# has it.
    let one =
        |pattern: &str| first_match(pattern, html).and_then(|groups| groups.into_iter().next());
    GoogleVersions {
        // :123, :132
        map: one(r#""*https?://mt\D?\d..*/vt\?lyrs=m@(\d*)"#).map(|found| format!("m@{found}")),
        // :148, :157
        hybrid: one(r#""*https?://mt\D?\d..*/vt\?lyrs=h@(\d*)"#).map(|found| format!("h@{found}")),
        // :173, :182
        satellite: one(r#""*https?://khm\D?\d.googleapis.com/kh\?v=(\d*)"#),
        // :199-209
        terrain: first_match(r#""*https?://mt\D?\d..*/vt\?lyrs=t@(\d*),r@(\d*)"#, html).and_then(
            |groups| match groups.as_slice() {
                [terrain, roads, ..] => Some(format!("t@{terrain},r@{roads}")),
                _ => None,
            },
        ),
    }
}

/// The version `BingMapProviderBase.OnInitialized` finds on Bing's maps page.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:177-185`
#[must_use]
pub fn bing_version(html: &str) -> Option<String> {
    first_match(r"tilegeneration:(\d*)", html).and_then(|groups| groups.into_iter().next())
}

/// A provider's `OnInitialized`, the first time it is about to fetch in this process.
///
/// Called by the tile thread, never the render thread. A provider that has already been through
/// here does nothing, whatever the outcome was - the C#'s `IsInitialized` is set before the check
/// runs, not after it succeeds.
pub fn initialize(source: &'static TileSource, root: &Path, fetcher: &TileFetcher) {
    if source.correction == Correction::None {
        return;
    }
    let first_time = INITIALIZED
        .lock()
        .map(|mut shown| shown.insert(source.cache_name))
        .unwrap_or(false);
    if !first_time {
        return;
    }
    match source.correction {
        Correction::None => {}
        Correction::Google => {
            // `if (!init && TryCorrectVersion) Task.Run(...)` - checked before the task starts.
            if GOOGLE_DONE.load(Ordering::Acquire) {
                return;
            }
            let root = root.to_path_buf();
            let fetcher = fetcher.clone();
            let _ = std::thread::Builder::new()
                .name("mp-tiles-version".to_owned())
                .spawn(move || {
                    correct_google(&root, |url| fetcher.fetch_text(url, source.referer));
                });
        }
        Correction::Bing => {
            correct_bing(root, |url| fetcher.fetch_text(url, source.referer));
        }
    }
}

/// The body of Google's check, with the download supplied by the caller.
///
/// The page from the URL cache if it is there and fresh, otherwise fetched and saved; then the four
/// patterns, each overwriting its provider's version if it matched. A failed download leaves the
/// check undone, to be tried again; an empty page completes it with nothing learned, as in the C#.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:92-238`
pub fn correct_google(root: &Path, fetch: impl FnOnce(&str) -> Result<String, FetchError>) {
    if GOOGLE_DONE.load(Ordering::Acquire) {
        return;
    }
    let Some(html) = page(root, GOOGLE_PAGE, fetch) else {
        return;
    };
    let found = google_versions(&html);
    if let Some(version) = found.map {
        set(&GOOGLE_MAP, version);
    }
    // `found.hybrid` would go to GoogleHybridMap, which is not ported: it draws the satellite tile
    // with a label layer over it, and the map here draws one layer.
    if let Some(version) = found.satellite {
        set(&GOOGLE_SATELLITE_MAP, version);
    }
    if let Some(version) = found.terrain {
        set(&GOOGLE_TERRAIN_MAP, version);
    }
    GOOGLE_DONE.store(true, Ordering::Release);
}

/// The body of Bing's check, with the download supplied by the caller.
///
/// The version part only. The C# goes on to ask Microsoft's logging service for a session key
/// (`BingMapProvider.cs:213-242`), presenting a Bing Maps key that is baked into GMap.NET, and adds
/// `&key=<session>` to tile URLs when it gets one. That is not ported: it is a credential exchange
/// with a key that belongs to GMap.NET, and the tile URLs work without it - the C# appends the key
/// only when it has one (`BingMapProvider.cs:623`), so a URL without it is one the C# sends too.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:149-211`
pub fn correct_bing(root: &Path, fetch: impl FnOnce(&str) -> Result<String, FetchError>) {
    if BING_DONE.load(Ordering::Acquire) {
        return;
    }
    let Some(html) = page(root, BING_PAGE, fetch) else {
        return;
    };
    if let Some(version) = bing_version(&html) {
        for source in [&BING_MAP, &BING_SATELLITE_MAP, &BING_HYBRID_MAP] {
            set(source, version.clone());
        }
    }
    BING_DONE.store(true, Ordering::Release);
}

/// A version page: from the URL cache, or downloaded and then cached. `None` if the download
/// failed, which is the C#'s exception path - the check is left undone.
fn page(
    root: &Path,
    url: &str,
    fetch: impl FnOnce(&str) -> Result<String, FetchError>,
) -> Option<String> {
    let cached = urlcache::get(root, url, urlcache::STAY_IN_CACHE).unwrap_or_default();
    if !cached.is_empty() {
        return Some(cached);
    }
    let fetched = fetch(url).ok()?;
    if !fetched.is_empty() {
        urlcache::save(root, url, &fetched);
    }
    Some(fetched)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Nothing here calls `correct_google`, `correct_bing` or `initialize`: they change what every
    // URL in the process carries, and the URL tests in this crate run in the same process. Those
    // are exercised in `tests/versions.rs`, which is a process of its own.

    #[test]
    fn the_satellite_version_is_the_number_after_kh_v() {
        let html = r#"apiLoad([0.01,[null,[["http://khm0.googleapis.com/kh?v=1015&hl=en-US"]]]])"#;
        let found = google_versions(html);
        assert_eq!(found.satellite.as_deref(), Some("1015"));
        assert_eq!(found.map, None);
        assert_eq!(found.hybrid, None);
        assert_eq!(found.terrain, None);
    }

    #[test]
    fn the_satellite_pattern_is_case_insensitive_and_takes_the_first() {
        let html = r#""HTTPS://KHM1.GOOGLEAPIS.COM/KH?V=2000 "http://khm0.googleapis.com/kh?v=169"#;
        assert_eq!(google_versions(html).satellite.as_deref(), Some("2000"));
    }

    #[test]
    fn the_road_hybrid_and_terrain_versions_carry_their_prefixes() {
        // Script lines of the shape the C#'s patterns were written against.
        let html = concat!(
            "\"https://mt0.googleapis.com/vt?lyrs=m@354000123&hl=en\"\n",
            "\"https://mt1.googleapis.com/vt?lyrs=h@333000456&hl=en\"\n",
            "\"https://mt0.googleapis.com/vt?lyrs=t@355,r@354000789&hl=en\"\n",
        );
        let found = google_versions(html);
        assert_eq!(found.map.as_deref(), Some("m@354000123"));
        assert_eq!(found.hybrid.as_deref(), Some("h@333000456"));
        assert_eq!(found.terrain.as_deref(), Some("t@355,r@354000789"));
        assert_eq!(found.satellite, None);
    }

    #[test]
    fn an_empty_number_is_taken_as_the_csharp_takes_it() {
        // `(\d*)` matches nothing as happily as something, and the C# writes what it matched.
        assert_eq!(
            google_versions("\"http://khm0.googleapis.com/kh?v=&hl")
                .satellite
                .as_deref(),
            Some("")
        );
    }

    #[test]
    fn the_bing_version_follows_tilegeneration() {
        assert_eq!(
            bing_version("var x = {TileGeneration:15512,other:1}").as_deref(),
            Some("15512")
        );
        assert_eq!(bing_version("nothing to see"), None);
    }

    #[test]
    fn a_provider_nobody_corrected_uses_its_hard_coded_version() {
        // GoogleSatelliteMapProvider.cs:22 and BingMapProvider.cs:26.
        assert_eq!(current(&GOOGLE_SATELLITE_MAP), "955");
        assert_eq!(current(&BING_SATELLITE_MAP), "4810");
        assert_eq!(current(&crate::source::OPENSTREETMAP), "");
    }
}
