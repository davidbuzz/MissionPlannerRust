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

//! GMap.NET's URL cache, including the part of its behaviour that depends on the whole process.
//!
//! One test, in a process of its own, because what it checks is a sequence: in the C#, nothing is
//! read from the URL cache until something has been written to it in the same run
//! (`Cache.cs:119, 188-189, 220-221`), so the first version check of a run always goes to the
//! network. A test that ran after another had written would see a different answer. The geocoder's
//! pages are in the same `Cache`, so the same holds for them, and a version page's write is what
//! makes them readable.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::PathBuf;
use web_time::{Duration, SystemTime};

use mp_tiles::urlcache::{self, CacheType};

const PAGE: &str = "http://www.bing.com/maps";
const OTHER: &str = "http://maps.google.com/maps/api/js?v=3.2&sensor=false";
/// Zoom To's search for "Canberra ACT".
const SEARCH: &str = "https://nominatim.openstreetmap.org/search?q=Canberra+ACT&format=xml";

#[test]
fn nothing_is_read_before_something_is_written_and_old_pages_are_deleted() {
    let root = mp_os::temp_dir().join(format!("mp-tiles-urlcache-{}", mp_os::process_id()));
    let _ = std::fs::remove_dir_all(&root);
    let cleanup = Cleanup(root.clone());

    // A fresh page the C# left behind, exactly where it leaves it.
    let page = urlcache::path(&root, PAGE);
    std::fs::create_dir_all(page.parent().unwrap()).unwrap();
    std::fs::write(&page, "\u{feff}tilegeneration:15512").unwrap();
    // And a geocoder page, in its own directory under the SHA-1 of its URL's UTF-16 bytes.
    let found = urlcache::content_path(&root, CacheType::GeocoderCache, SEARCH);
    assert_eq!(
        found,
        root.join("GeocoderCache")
            .join("78-FE-77-2E-4E-4A-20-59-5D-6B-3D-ED-A5-60-4A-A5-36-14-17-A7.txt")
    );
    std::fs::create_dir_all(found.parent().unwrap()).unwrap();
    std::fs::write(&found, "\u{feff}<?xml?><searchresults><place/>").unwrap();
    let geocoded = || {
        urlcache::get_content(
            &root,
            CacheType::GeocoderCache,
            SEARCH,
            urlcache::DEFAULT_STAY_IN_CACHE,
        )
    };

    // Not read: this process has not written to the cache yet, and neither would the C# have.
    assert_eq!(urlcache::get(&root, PAGE, urlcache::STAY_IN_CACHE), None);
    assert!(page.is_file(), "a miss for that reason deletes nothing");
    assert_eq!(geocoded(), None, "nor the geocoder's");
    assert!(found.is_file());

    // The first write - of another page - is what makes the cache readable.
    urlcache::save(&root, OTHER, "loader");
    let written = std::fs::read(urlcache::path(&root, OTHER)).unwrap();
    assert_eq!(
        written,
        "\u{feff}loader".as_bytes(),
        "UTF-8 with a byte-order mark"
    );

    // Now the fresh page is read, without the byte-order mark.
    assert_eq!(
        urlcache::get(&root, PAGE, urlcache::STAY_IN_CACHE).as_deref(),
        Some("tilegeneration:15512")
    );

    // An old page is deleted rather than read.
    let nine_hours_ago = SystemTime::now() - Duration::from_secs(9 * 60 * 60);
    std::fs::File::options()
        .write(true)
        .open(&page)
        .unwrap()
        .set_modified(nine_hours_ago)
        .unwrap();
    assert_eq!(urlcache::get(&root, PAGE, urlcache::STAY_IN_CACHE), None);
    assert!(!page.exists(), "a page past its eight hours is deleted");

    // The version page's write made the geocoder's readable too: one `Cache`, one location.
    assert_eq!(geocoded().as_deref(), Some("<?xml?><searchresults><place/>"));
    // Its time is 88 days, not eight hours; past them it is deleted.
    let age = |days: u64| {
        std::fs::File::options()
            .write(true)
            .open(&found)
            .unwrap()
            .set_modified(SystemTime::now() - Duration::from_secs(days * 24 * 60 * 60))
            .unwrap();
    };
    age(87);
    assert!(geocoded().is_some(), "87 days is within the 88");
    age(89);
    assert_eq!(geocoded(), None);
    assert!(!found.exists(), "a geocoder page past its 88 days is deleted");

    // And a page never saved is simply absent.
    assert_eq!(
        urlcache::get(&root, "http://example.invalid/", urlcache::STAY_IN_CACHE),
        None
    );
    drop(cleanup);
}

/// Removes a directory when dropped, so a failed assertion does not leave it behind.
struct Cleanup(PathBuf);

impl Drop for Cleanup {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
