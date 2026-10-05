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

//! Google's and Bing's version checks, end to end short of the network.
//!
//! A process of its own, because a check that succeeds changes the URL every tile of that family
//! is asked for, for the rest of the process - which is the point, and which would upset the URL
//! tests if they shared a process with it. The download is supplied by each test: fetching from
//! Google or Bing in a test is not allowed. The request itself, and the check running on the tile
//! thread before the first tile, are exercised for Bing in `tests/wire.rs`, through a proxy that
//! never leaves the machine.
//!
//! The page is real where it can be: a fixture excerpt of the loader script the C# application
//! downloaded on this machine, and - when it is there - the C#'s own cached copy, read in place.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::PathBuf;

use mp_tiles::FetchError;
use mp_tiles::source::{
    BING_HYBRID_MAP, BING_MAP, BING_SATELLITE_MAP, GOOGLE_MAP, GOOGLE_SATELLITE_MAP,
    GOOGLE_TERRAIN_MAP,
};
use mp_tiles::urlcache;
use mp_tiles::versions::{
    BING_PAGE, GOOGLE_PAGE, correct_bing, correct_google, current, google_versions,
};
use mp_units::TileId;

/// The opening of `http://maps.google.com/maps/api/js?v=3.2&sensor=false` as Google served it to
/// the C# application on this machine on 2026-09-20, cut after the first imagery URLs.
const LOADER: &str = include_str!("fixtures/google-maps-api-js.txt");

/// A temporary directory that removes itself.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let path = mp_os::temp_dir().join(format!(
            "mp-tiles-versions-{name}-{}-{:?}",
            mp_os::process_id(),
            wasm_thread::current().id()
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

fn offline() -> FetchError {
    FetchError::Request {
        url: GOOGLE_PAGE.to_owned(),
        message: "no network in a test".to_owned(),
    }
}

#[test]
fn google_imagery_is_asked_for_by_the_version_the_loader_script_names() {
    let scratch = Scratch::new("google");
    let tile = TileId::new(16, 59_922, 39_658).unwrap();
    assert_eq!(current(&GOOGLE_SATELLITE_MAP), "955");

    // A failed download is the C#'s exception path: nothing learned, nothing cached, and the check
    // left undone so that it is tried again.
    correct_google(&scratch.0, |_| Err(offline()));
    assert_eq!(current(&GOOGLE_SATELLITE_MAP), "955");
    assert!(!urlcache::path(&scratch.0, GOOGLE_PAGE).exists());

    // Tried again, and this time the page arrives.
    let mut asked = Vec::new();
    correct_google(&scratch.0, |url| {
        asked.push(url.to_owned());
        Ok(LOADER.to_owned())
    });
    assert_eq!(
        asked,
        vec![GOOGLE_PAGE.to_owned()],
        "one request, for the loader script"
    );
    assert_eq!(current(&GOOGLE_SATELLITE_MAP), "1015");
    assert_eq!(
        GOOGLE_SATELLITE_MAP.url_for(tile).as_deref(),
        Some("https://khms2.google.com/kh/v=1015&hl=en&x=59922&s=&y=39658&z=16&s=")
    );
    // Today's script names no road or terrain version, so those keep their hard-coded ones - as
    // they do in the C#.
    assert_eq!(current(&GOOGLE_MAP), "m@354000000");
    assert_eq!(current(&GOOGLE_TERRAIN_MAP), "t@354,r@354000000");

    // Saved where the C# saves it, as the C# writes it: UTF-8 with a byte-order mark.
    let saved = urlcache::path(&scratch.0, GOOGLE_PAGE);
    assert_eq!(
        saved,
        scratch
            .0
            .join("UrlCache/2E-E7-82-36-A4-AA-AE-AE-3F-46-79-C8-08-CA-12-13-20-7F-22-F9.txt")
    );
    let bytes = std::fs::read(&saved).unwrap();
    assert_eq!(&bytes[..3], b"\xEF\xBB\xBF");
    assert_eq!(&bytes[3..], LOADER.as_bytes());

    // Once it has succeeded it is not tried again in this process.
    correct_google(&scratch.0, |_| panic!("the check ran twice"));
}

#[test]
fn bing_tiles_are_asked_for_by_the_generation_bings_page_names() {
    let scratch = Scratch::new("bing");
    let tile = TileId::new(2, 3, 1).unwrap();
    assert_eq!(current(&BING_SATELLITE_MAP), "4810");

    correct_bing(&scratch.0, |url| {
        assert_eq!(url, BING_PAGE);
        Ok("<script>var cfg={tileGeneration:15512,mkt:'en-us'};</script>".to_owned())
    });
    // All three, from one page: BingMapProvider.cs:189-191.
    for source in [&BING_MAP, &BING_SATELLITE_MAP, &BING_HYBRID_MAP] {
        assert_eq!(current(source), "15512", "{}", source.id);
    }
    assert_eq!(
        BING_SATELLITE_MAP.url_for(tile).as_deref(),
        Some("http://ecn.t1.tiles.virtualearth.net/tiles/a13.jpeg?g=15512&mkt=en&n=z")
    );
    assert!(urlcache::path(&scratch.0, BING_PAGE).is_file());
    correct_bing(&scratch.0, |_| panic!("the check ran twice"));
}

#[test]
fn the_fixture_is_what_the_csharp_found() {
    let found = google_versions(LOADER);
    assert_eq!(found.satellite.as_deref(), Some("1015"));
    assert_eq!(found.map, None);
    assert_eq!(found.hybrid, None);
    assert_eq!(found.terrain, None);
}

#[test]
fn the_page_the_csharp_cached_on_this_machine_yields_a_version() {
    // Read in place and never through `urlcache::get`, which deletes a page it finds too old - a
    // test has no business deleting the C# application's files.
    let Some(root) =
        mp_settings::Folders::from_environment().map(|f| f.csharp_map_cache_directory())
    else {
        eprintln!("skipped: no home directory");
        return;
    };
    let cached = urlcache::path(&root, GOOGLE_PAGE);
    let Ok(text) = std::fs::read_to_string(&cached) else {
        eprintln!(
            "skipped: the C# application has not cached Google's loader script at {}",
            cached.display()
        );
        return;
    };
    // Found at the path computed from the URL, which is the naming rule proved on real data.
    let text = text.strip_prefix('\u{feff}').unwrap_or(&text);
    let found = google_versions(text);
    eprintln!("{}: {found:?}", cached.display());
    let satellite = found
        .satellite
        .expect("the loader script names the imagery version");
    assert!(
        !satellite.is_empty() && satellite.chars().all(|c| c.is_ascii_digit()),
        "{satellite:?}"
    );
}
