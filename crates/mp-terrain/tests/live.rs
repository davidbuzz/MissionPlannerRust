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

//! The real servers, once, by hand: `cargo test -p mp-terrain --test live -- --ignored`.
//!
//! Every other test serves the pages itself. This one lets the queue thread do what it does for
//! a user - read `terrain.ardupilot.org/SRTM3/`, then the 1-arc-second listing at
//! `terrain.ardupilot.org/SRTM1/`, the first server `get3secfile` searches, and fetch
//! `S28E153.hgt.zip` from it - and holds the result to the copy of that tile a real Mission
//! Planner downloaded, which `testdata/srtm/` keeps.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::Path;
use std::time::{Duration, Instant};

use mp_terrain::{AltResponse, Srtm, TileType};

#[test]
#[ignore = "downloads about 4 MB from terrain.ardupilot.org"]
fn a_tile_comes_from_the_first_server_as_mission_planner_got_it() {
    let dir = std::env::temp_dir().join(format!("mp-terrain-live-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);

    let srtm = Srtm::new(&dir);
    assert_eq!(srtm.get_altitude(-27.5, 153.5, 16.0), AltResponse::INVALID);
    assert_eq!(srtm.queued(), vec!["S28E153.hgt"]);

    // HttpClient's 100 seconds for each of the three requests, and the one-second pause.
    let deadline = Instant::now() + Duration::from_secs(330);
    while !srtm.queued().is_empty() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(200));
    }
    let answer = srtm.get_altitude(-27.5, 153.5, 16.0);

    let listing = std::fs::read_to_string(dir.join("SRTM1")).unwrap_or_default();
    let index = std::fs::read_to_string(dir.join("SRTM3")).unwrap_or_default();
    let downloaded = std::fs::read(dir.join("S28E153.hgt.zip")).unwrap_or_default();
    let tile = std::fs::read(dir.join("S28E153.hgt")).unwrap_or_default();
    let _ = std::fs::remove_dir_all(&dir);

    assert!(
        listing
            .lines()
            .any(|l| l == "https://terrain.ardupilot.org/SRTM1/S28E153.hgt.zip"),
        "the SRTM1 listing, {} bytes",
        listing.len()
    );
    assert!(
        index
            .lines()
            .any(|l| l.starts_with("https://terrain.ardupilot.org/SRTM3/")),
        "{index}"
    );
    assert!(downloaded.starts_with(b"PK"), "{} bytes", downloaded.len());
    let expected = std::fs::read(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/srtm/S28E153.hgt.zip"),
    )
    .unwrap();
    let expected = mp_log::zip::read(&expected).unwrap();
    assert_eq!(tile.len(), 3601 * 3601 * 2);
    assert!(
        tile == expected[0].data,
        "the server's S28E153 is not the one in testdata"
    );
    // What the oracle's C# answered from the same tile.
    assert_eq!(
        (answer.current_type, answer.alt, answer.alt_source),
        (TileType::Valid, 58.0, "SRTM")
    );
}
