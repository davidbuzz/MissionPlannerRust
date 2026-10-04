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

//! `Srtm` held to Mission Planner's own `srtm.getAltitude`, run under mono by
//! `tools/csharp-reference/SrtmOracle.cs` (`regen-srtm.sh`).
//!
//! The oracle asked the C# a sequence of questions against `testdata/srtm/` and a local server,
//! and wrote down every answer, the download queue after each step, every request the server saw
//! and every file the cache directory held. This replays the same sequence, against the same
//! files and a server that answers the same pages, through the same path the application takes -
//! `get_altitude` and the queue - and holds every record to the C#'s: altitudes to the bit.
//!
//! One kind of answer differs on purpose. The C#'s ASCII-grid reader works once per file per
//! process (it caches a stream it then disposes, srtm.cs:288-290, 501-522); every later lookup in
//! that grid answers `Invalid`. The port reads the grid again. For those lines the C#'s `Invalid`
//! is checked, and the port's answer is held to the one a fresh lookup gives.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_os::Lock as _;
use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use mp_terrain::{AltResponse, Http, HttpError, Srtm, TileType, ascii_name, tile_name};

/// The oracle's server, as far as anything the C# wrote down can tell: its port was five digits,
/// so a listing file that holds these URLs has the length the oracle measured.
const BASE: &str = "http://127.0.0.1:54321";

/// The oracle's second server, which resets every connection.
const BROKEN: &str = "http://127.0.0.1:54322";

fn testdata(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/srtm")
        .join(name)
}

/// A fresh directory for one test, removed afterwards.
struct Scratch(PathBuf);

impl Scratch {
    fn new(test: &str) -> Self {
        let dir = mp_os::temp_dir().join(format!("mp-terrain-{test}-{}", mp_os::process_id()));
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

// ------------------------------------------------------------------ the oracle's server

/// `SrtmOracle.Srtm3Index`, byte for byte.
const SRTM3_INDEX: &str = "<html><body><h1>Index of /SRTM3/</h1>\n\
<a href=\"../\">Parent Directory</a>\n\
<a href=\"Region1/\">Region1/</a>\n\
<a href=\"Region2/\">Region2/</a>\n\
<A HREF=\"Region3/\">Region3/</A>\n\
<a href=\"Region4/\">Region4/</a>\n\
<a href=\"Region5/\">Region5/</a>\n\
<a href=\"Region6/\">Region6/</a>\n\
<a href=\"http://elsewhere.invalid/Region7/\">elsewhere</a>\n\
<a href=\"/srtm/version2_1/\">version 2.1</a>\n\
<a href=\"README.txt\">README.txt</a>\n\
<a href=\"bios\">bios</a>\n\
<a href='single/'>single quotes</a>\n\
<a href=\"\">empty</a>\n\
</body></html>\n";

/// `SrtmOracle.Readme`.
const README: &str = "Nothing in here is a link.\n";

/// `SrtmOracle.Listing`.
fn listing(names: impl IntoIterator<Item = String>) -> Vec<u8> {
    let mut page = String::from("<html><body>\n");
    for name in names {
        page.push_str(&format!("<a href=\"{name}\">{name}</a>\n"));
    }
    page.push_str("</body></html>\n");
    page.into_bytes()
}

/// `SrtmOracle.Srtm1Names`: 20000 names that are no tile, and S28E153 among them.
fn srtm1_names() -> Vec<String> {
    let mut names = Vec::new();
    for i in 0..20000 {
        names.push(format!("X{i:05}.hgt.zip"));
        if i == 9999 {
            names.push("S28E153.hgt.zip".to_owned());
        }
    }
    names
}

/// `SrtmOracle.RegionNames`.
fn region_names(region: u32) -> Vec<String> {
    (0..3000)
        .map(|i| format!("Y{region}_{i:04}.hgt.zip"))
        .collect()
}

/// `SrtmOracle.Route`, `Serve` and `StartBrokenServer`: what the C# was served, and the log of
/// what it asked for.
struct Server {
    zip: Vec<u8>,
    requests: Mutex<Vec<String>>,
}

impl Server {
    fn new() -> Self {
        Self {
            zip: std::fs::read(testdata("S28E153.hgt.zip")).unwrap(),
            requests: Mutex::new(Vec::new()),
        }
    }

    fn take_requests(&self) -> Vec<String> {
        std::mem::take(&mut *self.requests.os_lock().unwrap())
    }
}

impl Http for Server {
    fn get(&self, url: &str) -> Result<Vec<u8>, HttpError> {
        if let Some(path) = url.strip_prefix(BROKEN) {
            self.requests.os_lock().unwrap().push(path.to_owned());
            return Err(HttpError("connection reset".to_owned()));
        }
        let Some(path) = url.strip_prefix(BASE) else {
            panic!("the oracle's C# never asked for {url}");
        };
        self.requests.os_lock().unwrap().push(path.to_owned());
        let region = path
            .strip_prefix("/SRTM3/Region")
            .and_then(|rest| rest.strip_suffix('/'))
            .and_then(|n| n.parse::<u32>().ok())
            .filter(|n| (1..=6).contains(n));
        Ok(match (path, region) {
            (_, Some(region)) => listing(region_names(region)),
            ("/SRTM3/", _) => SRTM3_INDEX.as_bytes().to_vec(),
            ("/SRTM3/README.txt", _) => README.as_bytes().to_vec(),
            ("/SRTM1/", _) => listing(srtm1_names()),
            ("/SRTM1/S28E153.hgt.zip", _) => self.zip.clone(),
            ("/EMPTY/", _) => Vec::new(),
            _ => b"<html><body>not found</body></html>\n".to_vec(),
        })
    }
}

// ------------------------------------------------------------------ the records

/// A record's fields: space-separated, except a trailing `"..."` pair, which may hold spaces.
fn fields(line: &str) -> Vec<String> {
    match line.find(" \"") {
        Some(at) => {
            let mut fields: Vec<String> = line[..at].split(' ').map(str::to_owned).collect();
            let mut rest = &line[at + 1..];
            while let Some(quoted) = rest.strip_prefix('"') {
                let end = quoted.find('"').unwrap();
                fields.push(quoted[..end].to_owned());
                rest = quoted[end + 1..].trim_start_matches(' ');
            }
            fields
        }
        None => line.split(' ').map(str::to_owned).collect(),
    }
}

fn number(field: &str) -> f64 {
    field
        .parse()
        .unwrap_or_else(|_| panic!("not a number: {field}"))
}

/// `File.ReadAllLines`.
fn read_all_lines(path: &Path) -> Vec<String> {
    let text = String::from_utf8_lossy(&std::fs::read(path).unwrap()).into_owned();
    let mut lines = Vec::new();
    let mut rest = text.as_str();
    while !rest.is_empty() {
        match rest.find(['\r', '\n']) {
            Some(at) => {
                lines.push(rest[..at].to_owned());
                let after = &rest[at..];
                rest = after.strip_prefix("\r\n").unwrap_or(&after[1..]);
            }
            None => {
                lines.push(rest.to_owned());
                rest = "";
            }
        }
    }
    lines
}

/// The cache directory as the oracle prints it: `file` and `listing` records, in ordinal order.
fn files(dir: &Path) -> Vec<Vec<String>> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|entry| entry.unwrap().file_name().into_string().unwrap())
        .collect();
    names.sort();
    names
        .into_iter()
        .map(|name| {
            let path = dir.join(&name);
            let bytes = std::fs::read(&path).unwrap();
            if name.ends_with(".hgt") || name.ends_with(".zip") || name.ends_with(".asc") {
                return vec!["file".to_owned(), name, bytes.len().to_string()];
            }
            // A listing is written a line at a time with Environment.NewLine, as the C# writes it
            // - "\r\n" on Windows - and the oracle measured it where that is "\n": its length is
            // compared as the oracle's, a byte less for each line's "\r" on Windows.
            let carriage_returns = if cfg!(windows) {
                bytes.windows(2).filter(|pair| pair == b"\r\n").count()
            } else {
                0
            };
            let length = (bytes.len() - carriage_returns).to_string();
            let lines = read_all_lines(&path);
            let norm = |s: &String| s.replace(BASE, "{base}");
            vec![
                "listing".to_owned(),
                name,
                length,
                lines.len().to_string(),
                lines.first().map(norm).unwrap_or_default(),
                lines.last().map(norm).unwrap_or_default(),
            ]
        })
        .collect()
}

/// The cache directory the oracle started from: the synthetic tile unzipped as the C# unzips a
/// download (`FastZip.ExtractZip`, which `mp_log::zip::extract` ports), and the rest copied.
fn fill(dir: &Path) {
    let zip = std::fs::read(testdata("N00W001.hgt.zip")).unwrap();
    mp_log::zip::extract(&zip, dir).unwrap();
    for name in [
        "N01W001.hgt",
        "N02W001.hgt",
        "srtm_41_10.asc",
        "srtm_41_11.asc",
        "srtm_42_10.asc",
        "srtm_42_11.asc",
    ] {
        std::fs::copy(testdata(name), dir.join(name)).unwrap();
    }
}

#[test]
fn every_answer_is_mission_planners() {
    let text = std::fs::read_to_string(testdata("oracle.txt")).unwrap();
    let scratch = Scratch::new("oracle");
    let dir = scratch.0.join("srtm");
    std::fs::create_dir_all(&dir).unwrap();
    fill(&dir);

    let server = Arc::new(Server::new());
    let srtm = Srtm::without_thread(&dir, Arc::clone(&server) as Arc<dyn Http>);
    srtm.set_baseurl1sec(format!("{BASE}/SRTM1/"));
    srtm.set_baseurl(format!("{BASE}/SRTM3/"));

    let mut answers = 0;
    let mut kinds = HashSet::new();
    let mut ascii_read = HashSet::new();
    let mut ascii_repeats = Vec::new();
    let mut records = text.lines().enumerate().peekable();
    while let Some((at, line)) = records.next() {
        let where_ = format!("oracle.txt:{}: {line}", at + 1);
        let record = fields(line);
        match record[0].as_str() {
            // Runtime arithmetic, held to the oracle by the crate's unit tests.
            "format" | "cast" | "fdiv" => {}
            "mode" => srtm.set_cache_only(record[1] == "cacheonly"),
            "baseurl" => {
                srtm.set_baseurl(
                    record[1]
                        .replace("{base}", BASE)
                        .replace("{broken}", BROKEN),
                );
            }
            "alt" => {
                let (lat, lng, zoom) = (number(&record[1]), number(&record[2]), number(&record[3]));
                let got = srtm.get_altitude(lat, lng, zoom);
                answers += 1;
                kinds.insert(record[4].clone());

                let ascii = ascii_name(lat, lng);
                let is_ascii = tile_name(lat, lng)
                    .is_some_and(|tile| !dir.join(tile).is_file() && dir.join(&ascii).is_file());
                if is_ascii && !ascii_read.insert(ascii.clone()) {
                    // The C#'s one-read ASCII grid: see the module comment.
                    assert_eq!(
                        (record[4].as_str(), record[6].as_str()),
                        ("invalid", "Invalid"),
                        "{where_}: the C# reads an ASCII grid once"
                    );
                    let fresh = Srtm::without_thread(&dir, Arc::clone(&server) as Arc<dyn Http>);
                    fresh.set_cache_only(true);
                    let expected = fresh.get_altitude(lat, lng, zoom);
                    assert_eq!(got, expected, "{where_}: a second read of the grid");
                    assert_eq!(got.current_type, TileType::Valid, "{where_}");
                    ascii_repeats.push((lat, lng, got));
                    continue;
                }

                assert_eq!(got.current_type.name(), record[4], "{where_}: currenttype");
                assert_eq!(
                    got.alt.to_bits(),
                    number(&record[5]).to_bits(),
                    "{where_}: alt {}",
                    got.alt
                );
                assert_eq!(got.alt_source, record[6], "{where_}: altsource");
            }
            "queue" => assert_eq!(srtm.queued(), record[1..].to_vec(), "{where_}"),
            "run" => srtm.run_queue_once(),
            "requests" => assert_eq!(server.take_requests(), record[1..].to_vec(), "{where_}"),
            "file" | "listing" => {
                // A block of them: the whole directory at this point.
                let mut expected = vec![record];
                while let Some((_, next)) =
                    records.next_if(|(_, l)| l.starts_with("file ") || l.starts_with("listing "))
                {
                    expected.push(fields(next));
                }
                assert_eq!(files(&dir), expected, "{where_}: the cache directory");
            }
            other => panic!("{where_}: unknown record {other}"),
        }
    }

    // The oracle is what it says it is: every kind of answer, and more than a thousand of them.
    assert!(answers > 1200, "{answers} answers");
    assert_eq!(
        kinds,
        HashSet::from(["valid".to_owned(), "invalid".to_owned(), "ocean".to_owned()])
    );
    // The two lookups the C# gets wrong, answered from the grid: row 6 and row 4 counted from the
    // top of srtm_41_10.asc, whose cells are row * 100 + column.
    let alts: Vec<f64> = ascii_repeats.iter().map(|(_, _, got)| got.alt).collect();
    assert_eq!(alts, vec![602.0, 401.0], "{ascii_repeats:?}");
    assert!(
        ascii_repeats
            .iter()
            .all(|(_, _, got)| got.alt_source.is_empty())
    );
}

/// `SrtmOracle.Synthetic`.
fn synthetic(col: usize, row: usize) -> i16 {
    if (600..=602).contains(&col) && (600..=602).contains(&row) {
        return -32768;
    }
    match (col, row) {
        (100, 1100) => -32768,
        (900, 300) | (901, 301) => -1000,
        (901, 300) => -1001,
        (900, 301) => -999,
        (50, 50) => 32767,
        (51, 50) => -32767,
        _ => i16::try_from((col * 3 + row * 7) % 2000).unwrap() - 200,
    }
}

#[test]
fn the_synthetic_tile_is_what_the_oracle_says_it_is() {
    let zip = std::fs::read(testdata("N00W001.hgt.zip")).unwrap();
    let entries = mp_log::zip::read(&zip).unwrap();
    assert_eq!(entries.len(), 1);
    assert_eq!(entries[0].name, "N00W001.hgt");
    let data = &entries[0].data;
    assert_eq!(data.len(), 1201 * 1201 * 2);
    for row in 0..1201 {
        for col in 0..1201 {
            let at = (row * 1201 + col) * 2;
            let sample = i16::from_be_bytes([data[at], data[at + 1]]);
            assert_eq!(sample, synthetic(col, row), "col {col} row {row}");
        }
    }
}

#[test]
fn the_real_tile_is_the_one_mission_planner_downloaded() {
    // One tile from a real Mission Planner cache: S28E153, 1 arc-second, as terrain.ardupilot.org
    // serves it zipped.
    let zip = std::fs::read(testdata("S28E153.hgt.zip")).unwrap();
    let entries = mp_log::zip::read(&zip).unwrap();
    assert_eq!(entries.len(), 1);
    assert_eq!(entries[0].name, "S28E153.hgt");
    assert_eq!(entries[0].data.len(), 3601 * 3601 * 2);
}

#[test]
fn an_answer_is_the_same_value_the_constants_hold() {
    // The two canned answers are the C#'s static instances, field for field.
    assert_eq!(
        AltResponse::INVALID,
        AltResponse {
            current_type: TileType::Invalid,
            alt: 0.0,
            alt_source: "Invalid"
        }
    );
    assert_eq!(AltResponse::OCEAN.current_type.name(), "ocean");
    assert_eq!(AltResponse::OCEAN.alt_source, "Ocean");
}
