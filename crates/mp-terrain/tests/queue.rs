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

//! The download queue and the cache directory, beyond what the oracle's one sequence reaches: the
//! real thread, the week a listing is trusted, a tile that arrives while queued, an error page
//! where a zip should be, and `MainV2`'s startup sweep.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_os::Lock as _;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant, SystemTime};

use mp_terrain::{AltResponse, Http, HttpError, Srtm, TileType};

const BASE: &str = "http://terrain.test";

/// A fresh directory for one test, removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(test: &str) -> Self {
        let dir = mp_os::temp_dir().join(format!("mp-terrain-q-{test}-{}", mp_os::process_id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        Self(dir)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// A 3-arc-second tile of one height, zipped as the server zips one.
fn tile_zip(name: &str, height: i16) -> Vec<u8> {
    let data = height.to_be_bytes().repeat(1201 * 1201);
    let entry = mp_log::zip::Entry {
        name: name.to_owned(),
        data,
    };
    let when = mp_log::zip::DosTime {
        year: 2022,
        month: 6,
        day: 20,
        hour: 8,
        minute: 47,
        second: 0,
    };
    mp_log::zip::write(&[entry], when).unwrap()
}

fn page(links: &[&str]) -> Vec<u8> {
    let mut page = String::new();
    for link in links {
        page.push_str(&format!("<a href=\"{link}\">{link}</a>\n"));
    }
    page.into_bytes()
}

/// A server of fixed pages, logging every path asked for.
struct Server {
    pages: Vec<(String, Vec<u8>)>,
    requests: Mutex<Vec<String>>,
}

impl Server {
    fn new(pages: Vec<(&str, Vec<u8>)>) -> Arc<Self> {
        Arc::new(Self {
            pages: pages
                .into_iter()
                .map(|(path, body)| (path.to_owned(), body))
                .collect(),
            requests: Mutex::new(Vec::new()),
        })
    }

    fn take_requests(&self) -> Vec<String> {
        std::mem::take(&mut *self.requests.os_lock().unwrap())
    }
}

impl Http for Server {
    fn get(&self, url: &str) -> Result<Vec<u8>, HttpError> {
        let path = url.strip_prefix(BASE).unwrap_or(url).to_owned();
        self.requests.os_lock().unwrap().push(path.clone());
        Ok(self.pages.iter().find(|(p, _)| *p == path).map_or_else(
            || b"<html>404 Not Found</html>".to_vec(),
            |(_, b)| b.clone(),
        ))
    }
}

/// The server a tile is found on: nothing in the 3-arc-second index, N00W001 in the 1-arc-second
/// listing.
fn one_tile_server(height: i16) -> Arc<Server> {
    Server::new(vec![
        ("/SRTM3/", page(&[])),
        ("/SRTM1/", page(&["N00W001.hgt.zip"])),
        ("/SRTM1/N00W001.hgt.zip", tile_zip("N00W001.hgt", height)),
    ])
}

fn point_to(srtm: &Srtm) {
    srtm.set_baseurl1sec(format!("{BASE}/SRTM1/"));
    srtm.set_baseurl(format!("{BASE}/SRTM3/"));
}

fn names(dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    names.sort();
    names
}

#[test]
fn the_thread_downloads_a_queued_tile_and_the_next_lookup_has_it() {
    let scratch = Scratch::new("thread");
    // Not there yet: a lookup that queues makes it (srtm.cs:382-383).
    let dir = scratch.0.join("srtm");
    let server = one_tile_server(123);
    let srtm = Srtm::with_http(&dir, Arc::clone(&server) as Arc<dyn Http>);
    point_to(&srtm);

    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    assert!(dir.is_dir());

    let deadline = Instant::now() + Duration::from_secs(30);
    let answer = loop {
        let answer = srtm.get_altitude(0.5, -0.5, 16.0);
        if answer.current_type == TileType::Valid || Instant::now() > deadline {
            break answer;
        }
        wasm_thread::sleep(Duration::from_millis(20));
    };
    assert_eq!(answer.current_type, TileType::Valid, "{answer:?}");
    assert_eq!(answer.alt, 123.0);
    assert_eq!(answer.alt_source, "SRTM");
    assert!(srtm.queued().is_empty());
    // The index first, then the 1-arc-second listing, where it is found (srtm.cs:593-616).
    assert_eq!(
        server.take_requests(),
        vec!["/SRTM3/", "/SRTM1/", "/SRTM1/N00W001.hgt.zip"]
    );
    // The zip is left beside the tile, and each listing is cached under its last path segment.
    assert_eq!(
        names(&dir),
        vec!["N00W001.hgt", "N00W001.hgt.zip", "SRTM1", "SRTM3"]
    );
    // A line at a time with Environment.NewLine, as the C# writes it: "\r\n" on Windows.
    let newline = if cfg!(windows) { "\r\n" } else { "\n" };
    assert_eq!(
        std::fs::read_to_string(dir.join("SRTM1")).unwrap(),
        format!("{BASE}/SRTM1/N00W001.hgt.zip{newline}")
    );
}

#[test]
fn dropping_the_lookup_ends_its_thread() {
    let scratch = Scratch::new("drop");
    let server = one_tile_server(1);
    let srtm = Srtm::with_http(&scratch.0, Arc::clone(&server) as Arc<dyn Http>);
    // The lookup and its thread each hold the server.
    assert_eq!(Arc::strong_count(&server), 2);
    drop(srtm);
    let deadline = Instant::now() + Duration::from_secs(10);
    while Arc::strong_count(&server) > 1 && Instant::now() < deadline {
        wasm_thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(Arc::strong_count(&server), 1, "the thread is still running");
}

#[test]
fn zoom_decides_only_whether_a_missing_tile_is_queued() {
    let scratch = Scratch::new("zoom");
    let dir = scratch.0.join("srtm");
    let srtm = Srtm::without_thread(&dir, one_tile_server(1) as Arc<dyn Http>);

    assert_eq!(srtm.get_altitude(0.5, -0.5, 6.99), AltResponse::INVALID);
    assert!(srtm.queued().is_empty());
    assert!(!dir.exists(), "below zoom 7 nothing is touched");

    // Offline: the directory is made, and nothing is queued (srtm.cs:380-396).
    srtm.set_cache_only(true);
    assert_eq!(srtm.get_altitude(0.5, -0.5, 7.0), AltResponse::INVALID);
    assert!(dir.is_dir());
    assert!(srtm.queued().is_empty());

    srtm.set_cache_only(false);
    assert_eq!(srtm.get_altitude(0.5, -0.5, 7.0), AltResponse::INVALID);
    assert_eq!(srtm.queued(), vec!["N00W001.hgt"]);
    // Queued once, however often it is asked for.
    assert_eq!(srtm.get_altitude(0.7, -0.1, 16.0), AltResponse::INVALID);
    assert_eq!(srtm.queued(), vec!["N00W001.hgt"]);

    // A tile on disk answers at any zoom.
    std::fs::write(
        dir.join("N00W001.hgt"),
        7i16.to_be_bytes().repeat(1201 * 1201),
    )
    .unwrap();
    srtm.run_queue_once();
    let answer = srtm.get_altitude(0.5, -0.5, 0.0);
    assert_eq!((answer.current_type, answer.alt), (TileType::Valid, 7.0));
}

#[test]
fn a_tile_that_arrives_while_queued_is_not_fetched() {
    let scratch = Scratch::new("arrives");
    let server = one_tile_server(1);
    let srtm = Srtm::without_thread(&scratch.0, Arc::clone(&server) as Arc<dyn Http>);
    point_to(&srtm);
    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    std::fs::write(
        scratch.0.join("N00W001.hgt"),
        9i16.to_be_bytes().repeat(1201 * 1201),
    )
    .unwrap();

    srtm.run_queue_once();
    // get3secfile finds it there and returns (srtm.cs:582-588).
    assert!(server.take_requests().is_empty());
    assert!(srtm.queued().is_empty());
    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0).alt, 9.0);
}

/// Sets a file's modification time `age` ago.
fn age(path: &Path, age: Duration) {
    let file = std::fs::File::options().write(true).open(path).unwrap();
    file.set_modified(SystemTime::now() - age).unwrap();
}

#[test]
fn a_cached_listing_is_trusted_for_a_week() {
    let scratch = Scratch::new("week");
    let server = one_tile_server(5);
    let srtm = Srtm::without_thread(&scratch.0, Arc::clone(&server) as Arc<dyn Http>);
    point_to(&srtm);
    let listing = format!("{BASE}/SRTM1/N00W001.hgt.zip\n");
    std::fs::write(scratch.0.join("SRTM3"), "").unwrap();
    std::fs::write(scratch.0.join("SRTM1"), &listing).unwrap();
    age(&scratch.0.join("SRTM1"), Duration::from_secs(6 * 24 * 3600));

    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    srtm.run_queue_once();
    // An empty listing is not a cached one; a six-day-old one is.
    assert_eq!(
        server.take_requests(),
        vec!["/SRTM3/", "/SRTM1/N00W001.hgt.zip"]
    );
    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0).alt, 5.0);

    // Eight days old: asked for again.
    std::fs::remove_file(scratch.0.join("N00W001.hgt")).unwrap();
    age(&scratch.0.join("SRTM1"), Duration::from_secs(8 * 24 * 3600));
    let again = Srtm::without_thread(&scratch.0, Arc::clone(&server) as Arc<dyn Http>);
    point_to(&again);
    assert_eq!(again.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    again.run_queue_once();
    // The index again too: the empty one written last time is still not a cached one.
    assert_eq!(
        server.take_requests(),
        vec!["/SRTM3/", "/SRTM1/", "/SRTM1/N00W001.hgt.zip"]
    );
}

#[test]
fn an_error_page_where_the_zip_should_be_is_saved_and_the_tile_asked_for_again() {
    let scratch = Scratch::new("404");
    // Listed, but the zip is not there: the server answers 404 with a page.
    let server = Server::new(vec![
        ("/SRTM3/", page(&[])),
        ("/SRTM1/", page(&["N00W001.hgt.zip"])),
    ]);
    let srtm = Srtm::without_thread(&scratch.0, Arc::clone(&server) as Arc<dyn Http>);
    point_to(&srtm);
    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    srtm.run_queue_once();

    // GetAsync does not throw on a 404, so the page is written as the zip, the unzip fails and
    // is logged, and the tile is off the queue as if it had been fetched (srtm.cs:640-675).
    assert_eq!(
        std::fs::read(scratch.0.join("N00W001.hgt.zip")).unwrap(),
        b"<html>404 Not Found</html>"
    );
    assert!(!scratch.0.join("N00W001.hgt").exists());
    assert!(srtm.queued().is_empty());
    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    assert_eq!(srtm.queued(), vec!["N00W001.hgt"]);
}

#[test]
fn an_empty_tile_is_never_fetched_again_until_the_startup_sweep() {
    let scratch = Scratch::new("empty");
    let srtm = Srtm::without_thread(&scratch.0, one_tile_server(1) as Arc<dyn Http>);
    std::fs::write(scratch.0.join("N00W001.hgt"), b"").unwrap();

    // There is a file, so it is read, and a file of the wrong length is no answer - and it is
    // not queued, because it is there.
    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    assert!(srtm.queued().is_empty());

    // MainV2 deletes it at the next start, and then it is fetched.
    mp_terrain::clean_cache_directory(&scratch.0);
    assert!(!scratch.0.join("N00W001.hgt").exists());
    assert_eq!(srtm.get_altitude(0.5, -0.5, 16.0), AltResponse::INVALID);
    assert_eq!(srtm.queued(), vec!["N00W001.hgt"]);
}

#[test]
fn the_startup_sweep_deletes_empty_and_stale_files_and_nothing_else() {
    let scratch = Scratch::new("sweep");
    let dir = &scratch.0;
    std::fs::write(dir.join("empty.hgt"), b"").unwrap();
    std::fs::write(dir.join("stale.hgt"), b"x").unwrap();
    std::fs::write(dir.join("fresh.hgt"), b"x").unwrap();
    std::fs::write(dir.join("just-after.hgt"), b"x").unwrap();
    std::fs::create_dir(dir.join("sub")).unwrap();
    std::fs::write(dir.join("sub").join("empty"), b"").unwrap();
    let cutoff =
        SystemTime::UNIX_EPOCH + Duration::from_secs(mp_terrain::STALE_BEFORE_UNIX_SECONDS);
    let set = |name: &str, when: SystemTime| {
        let file = std::fs::File::options()
            .write(true)
            .open(dir.join(name))
            .unwrap();
        file.set_modified(when).unwrap();
    };
    set("stale.hgt", cutoff - Duration::from_secs(1));
    set("just-after.hgt", cutoff);

    mp_terrain::clean_cache_directory(dir);

    assert_eq!(names(dir), vec!["fresh.hgt", "just-after.hgt", "sub"]);
    // Only the files directly in it.
    assert!(dir.join("sub").join("empty").exists());
    // A directory that is not there is not an error.
    mp_terrain::clean_cache_directory(&dir.join("missing"));
}

#[test]
fn the_cache_is_the_data_directorys_srtm() {
    let unix = mp_settings::Folders {
        my_documents: PathBuf::from("/nonexistent-home"),
        local_application_data: PathBuf::from("/nonexistent-home/.local/share"),
        common_application_data: PathBuf::from("/usr/share"),
        unix: true,
    };
    assert_eq!(
        mp_terrain::srtm_directory_in(&unix),
        PathBuf::from("/nonexistent-home/.local/share/MissionPlannerRust/srtm")
    );
    let windows = mp_settings::Folders {
        my_documents: PathBuf::from("C:/Users/u/Documents"),
        local_application_data: PathBuf::from("C:/Users/u/AppData/Local"),
        common_application_data: PathBuf::from("C:/ProgramData"),
        unix: false,
    };
    assert_eq!(
        mp_terrain::srtm_directory_in(&windows),
        PathBuf::from("C:/ProgramData/MissionPlannerRust/srtm")
    );
}

#[test]
fn the_terrain_server_sees_the_identity_the_map_servers_see() {
    assert_eq!(mp_terrain::USER_AGENT, mp_tiles::fetch::USER_AGENT);
}

#[test]
fn the_servers_are_the_csharps_until_changed() {
    let srtm = Srtm::without_thread("unused", one_tile_server(1) as Arc<dyn Http>);
    assert_eq!(srtm.baseurl1sec(), "https://terrain.ardupilot.org/SRTM1/");
    assert_eq!(srtm.baseurl(), "https://terrain.ardupilot.org/SRTM3/");
    assert!(!srtm.cache_only());
}
