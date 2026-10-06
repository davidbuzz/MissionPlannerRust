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

//! The providers against GMap.NET itself (DELIVERABLES.md Deliverable 8: the identical provider list).
//!
//! `fixtures/gmap-oracle.tsv` is what a Mission Planner `GMap.NET.Core.dll` said when asked by
//! `fixtures/GMapOracle.cs`: the order and names of `GMapProviders.List` - the list the map-type box
//! is filled from - and, for the Google and Bing providers, the URL each one's own
//! `MakeTileImageUrl` builds for a spread of tiles. Every URL here is compared with that, character
//! for character; the unit tests in `source.rs` check a few of the same URLs against strings worked
//! out by hand from the C#'s format strings, so the two sources of truth check each other.
//!
//! The DLL is the one shipped with Mission Planner (the copy in `~/Downloads` on the machine that
//! made the fixture), not one built from https://github.com/ArduPilot/MissionPlanner. Its versions, referers and
//! list agree with the tree's source at the lines `source.rs` cites.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_tiles::source::{CSHARP_LIST, SOURCES, source_by_name};
use mp_units::TileId;

const ORACLE: &str = include_str!("fixtures/gmap-oracle.tsv");

/// The oracle's records of one kind, split on tabs.
fn records(kind: &str) -> Vec<Vec<&'static str>> {
    ORACLE
        .lines()
        .map(|line| line.split('\t').collect::<Vec<_>>())
        .filter(|fields| fields[0] == kind)
        .collect()
}

#[test]
fn the_list_is_the_csharps_list_in_the_csharps_order() {
    let listed: Vec<&str> = records("list").iter().map(|fields| fields[2]).collect();
    assert_eq!(listed.len(), 66, "GMapProviders' own fields");
    for (index, fields) in records("list").iter().enumerate() {
        assert_eq!(fields[1], index.to_string());
    }
    // CSHARP_LIST is the oracle's list, then the providers Program.cs appends - which the oracle
    // cannot see, as it runs GMap.NET without Mission Planner.
    assert_eq!(&CSHARP_LIST[..listed.len()], listed.as_slice());
    assert_eq!(CSHARP_LIST.len(), listed.len() + 24, "Program.cs:330-353");
}

#[test]
fn every_ported_provider_is_the_csharps_by_name_referer_and_zoom() {
    let listed = records("list");
    let mut ported = 0;
    for source in SOURCES {
        let Some(fields) = listed.iter().find(|fields| fields[2] == source.cache_name) else {
            continue;
        };
        ported += 1;
        let (max_zoom, referer, overlays) = (fields[3], fields[4], fields[5]);
        assert_eq!(source.referer, referer, "{}", source.id);
        // One layer: the provider draws itself and nothing under it. A provider whose Overlays
        // are two - GoogleHybridMap's are GoogleSatelliteMap then itself - needs the map to draw a
        // second store under the first, which it does not.
        assert_eq!(
            overlays, source.cache_name,
            "{} is not one layer",
            source.id
        );
        // The C# sets no limit on OpenStreetMap; 19 is kept here, see the note on it.
        if source.cache_name != "OpenStreetMap" {
            let expected = match max_zoom {
                "null" => mp_units::tiles::MAX_ZOOM,
                limit => limit.parse().unwrap(),
            };
            assert_eq!(source.max_zoom, expected, "{}", source.id);
        }
    }
    assert_eq!(
        ported, 7,
        "OpenStreetMap, three Bing and three Google providers"
    );
}

#[test]
fn every_url_is_the_one_gmap_net_builds() {
    let mut compared = 0;
    for fields in records("url") {
        let [_, name, x, y, zoom, expected] = fields[..] else {
            panic!("a malformed record: {fields:?}");
        };
        let Some(source) = source_by_name(name) else {
            // GoogleHybridMap: in the oracle, not ported.
            assert_eq!(name, "GoogleHybridMap");
            continue;
        };
        let tile = TileId::new(
            zoom.parse().unwrap(),
            x.parse().unwrap(),
            y.parse().unwrap(),
        )
        .expect("the oracle's tiles are on the grid");
        assert_eq!(
            source.url_for(tile).as_deref(),
            Some(expected),
            "{name} at {x}, {y}, zoom {zoom}"
        );
        compared += 1;
    }
    assert_eq!(compared, 6 * 14, "six providers, fourteen tiles each");
}

#[test]
fn every_version_and_copyright_is_the_csharps() {
    let year = records("year")[0][1];
    let mut compared = 0;
    for fields in records("provider") {
        let [_, name, version, copyright] = fields[..] else {
            panic!("a malformed record: {fields:?}");
        };
        let Some(source) = source_by_name(name) else {
            continue;
        };
        assert_eq!(source.version, version, "{name}");
        // The oracle formatted its copyright with the year it ran in; so does the provider, with
        // this year. Compared on the format, so the fixture does not expire on 1 January.
        assert_eq!(source.attribution.replace("{0}", year), copyright, "{name}");
        compared += 1;
    }
    assert_eq!(compared, 6);
}
