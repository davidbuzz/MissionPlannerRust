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

//! The projection proof: PLAN.md §13.3 item 8, DELIVERABLES.md D8.
//!
//! Every file in `testdata/projection/golden` is what Mission Planner's own code returned under
//! mono for the coordinates of `testdata/projection/points.txt` (`tools/csharp-reference/
//! regen-projection.sh`, and `testdata/projection/README.md` for how): GMap.NET's
//! `MercatorProjection` at zooms 1, 10, 16, 20 and 30, and `PointLatLngAlt`'s `GetDistance`,
//! `GetBearing` and `newpos`. Each is re-run through `mp_units` and compared.
//!
//! What each comparison can prove, and so what it asserts:
//! - **Forward projection.** GMap never exposes its continuous projection: `FromLatLngToPixel`
//!   rounds to a whole pixel (`MercatorProjection.cs:67-68`). So a 1e-6-pixel comparison of the
//!   unrounded value is not possible; what is, is rounding ours the way GMap rounds and requiring
//!   the same integer at every zoom. At zoom 30 - the deepest at which GMap's `1 << zoom` is still
//!   exact - a pixel is 2^-38 of the world, 1/1024 of a zoom-20 pixel and 0.15 mm on the ground at
//!   the equator, so equality there bounds our continuous value to GMap's within that.
//! - **Inverse projection.** `FromPixelToLatLng` takes the pixel, and `pixel / map size` is exact
//!   for GMap's power-of-two sizes, so ours is held to it to the bit.
//! - **Round trip.** GMap's own round trip goes through the whole pixel, so it is only as good as
//!   half a pixel - several centimetres at zoom 20, measured below - and cannot meet D8's < 1 mm.
//!   Ours stays in `f64` (PLAN.md §9.1), and is held to 1e-8 degrees and 1 mm on the ground.
//! - **Distance, bearing, offset.** Transliterated from `PointLatLngAlt.cs` operation for
//!   operation (PLAN.md §1.3 ports `GetDistance` literally), and held to the bit, except where
//!   `newpos` returns a longitude past ±180, which a `LatLon` cannot hold and wraps.
//!
//! `utm.csv` is generated beside these for `new utmpos(PointLatLngAlt)` and `ToLLA`; mp-units has
//! no UTM, and the port that does (`mp_mission`'s `utm` module) is private to its crate, so it is
//! not compared here.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use std::path::Path;

use mp_units::geodesy::WEB_MERCATOR_MAX_LATITUDE;
use mp_units::tiles::{MAX_ZOOM, TILE_SIZE_PX, tiles_across};
use mp_units::{Bearing, Degrees, LatLon, Metres, TileId, WebMercator};

/// D8's round-trip bound on the ground.
const MILLIMETRE: f64 = 0.001;

/// The same bound in degrees, as PLAN.md §13.3 item 8 words it: 1e-8 degrees of latitude is
/// 1.1 mm.
const ROUND_TRIP_DEGREES: f64 = 1e-8;

fn golden(name: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/projection/golden")
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} (regenerate with regen-projection.sh)",
            path.display()
        )
    })
}

/// The `key,value...` rows of a golden with the given key.
fn rows<'a>(text: &'a str, key: &str) -> Vec<Vec<&'a str>> {
    text.lines()
        .filter(|line| !line.starts_with('#'))
        .map(|line| line.split(',').collect::<Vec<_>>())
        .filter(|fields| fields[0] == key)
        .map(|fields| fields[1..].to_vec())
        .collect()
}

fn num(text: &str) -> f64 {
    text.parse()
        .unwrap_or_else(|_| panic!("{text} is not a number"))
}

fn int(text: &str) -> i64 {
    text.parse()
        .unwrap_or_else(|_| panic!("{text} is not an integer"))
}

fn position(lat: &str, lng: &str) -> LatLon {
    LatLon::new(num(lat), num(lng)).unwrap_or_else(|e| panic!("{lat},{lng}: {e}"))
}

/// Bit equality that reads as the C# does: -0.0 is not 0.0, and the test says which differed.
fn same_bits(ours: f64, theirs: f64) -> bool {
    ours.to_bits() == theirs.to_bits()
}

/// GMap's whole pixel for a Web Mercator coordinate at a map size, `MercatorProjection.cs:67-68`:
/// `(long)Clip(x * mapSizeX + 0.5, 0, mapSizeX - 1)`, where Clip is Min(Max(..)) and the cast
/// truncates toward zero, as `as` does.
fn gmap_pixel(unit: f64, map_size: i64) -> i64 {
    let size = map_size as f64;
    (unit * size + 0.5).max(0.0).min((map_size - 1) as f64) as i64
}

/// One zoom's worth of a `mercator.csv` point row.
struct Readout {
    zoom: u8,
    map_size: i64,
    pixel: (i64, i64),
    back: (f64, f64),
}

/// One `mercator.csv` point: the input exactly as GMap received it, and what it returned per zoom.
struct MercatorPoint {
    lat: f64,
    lng: f64,
    readouts: Vec<Readout>,
}

fn mercator_points() -> Vec<MercatorPoint> {
    let text = golden("mercator.csv");
    let sizes: Vec<(u8, i64)> = rows(&text, "size")
        .iter()
        .map(|f| {
            assert_eq!(int(f[1]), int(f[2]), "GMap's maps are square");
            (u8::try_from(int(f[0])).unwrap(), int(f[1]))
        })
        .collect();
    assert_eq!(
        sizes.iter().map(|(z, _)| *z).collect::<Vec<_>>(),
        [1, 10, 16, 20, 30],
        "the zooms of PLAN.md §13.3 item 8, and the finest GMap can read out"
    );
    let points: Vec<MercatorPoint> = rows(&text, "point")
        .iter()
        .map(|f| {
            assert_eq!(f.len(), 2 + 4 * sizes.len(), "{f:?}");
            let readouts = sizes
                .iter()
                .enumerate()
                .map(|(i, &(zoom, map_size))| {
                    let at = 2 + 4 * i;
                    Readout {
                        zoom,
                        map_size,
                        pixel: (int(f[at]), int(f[at + 1])),
                        back: (num(f[at + 2]), num(f[at + 3])),
                    }
                })
                .collect();
            MercatorPoint {
                lat: num(f[0]),
                lng: num(f[1]),
                readouts,
            }
        })
        .collect();
    assert!(
        points.len() > 600,
        "only {} points in mercator.csv",
        points.len()
    );
    points
}

#[test]
fn the_map_size_at_each_zoom_is_gmaps() {
    // GetTileMatrixSizePixel: 2^zoom tiles of 256 pixels (PureProjection.cs:204-208).
    for size in rows(&golden("mercator.csv"), "size") {
        let zoom = u8::try_from(int(size[0])).unwrap();
        let ours = i64::from(tiles_across(zoom)) * i64::from(TILE_SIZE_PX);
        assert_eq!(ours, int(size[1]), "zoom {zoom}");
    }
}

#[test]
fn the_forward_projection_rounds_to_gmaps_pixel_at_every_zoom() {
    let mut compared = 0;
    let mut worst_unrounded: f64 = 0.0;
    for point in mercator_points() {
        let ours = LatLon::new(point.lat, point.lng).unwrap().to_web_mercator();
        for readout in &point.readouts {
            let pixel = (
                gmap_pixel(ours.x, readout.map_size),
                gmap_pixel(ours.y, readout.map_size),
            );
            assert_eq!(
                pixel, readout.pixel,
                "{},{} at zoom {}: ours {:?} rounds to {pixel:?}, GMap's is {:?}",
                point.lat, point.lng, readout.zoom, ours, readout.pixel
            );
            compared += 1;

            // Where GMap did not clip, its integer is ours rounded, so ours is within half a
            // pixel of it - the whole of what an integer readout can say.
            let size = readout.map_size as f64;
            for (unit, whole) in [(ours.x, readout.pixel.0), (ours.y, readout.pixel.1)] {
                let unrounded = unit * size;
                if (0.0..size - 1.0).contains(&unrounded) {
                    worst_unrounded = worst_unrounded.max((unrounded - whole as f64).abs());
                }
            }
        }
    }
    assert!(compared > 3000, "only {compared} pixels compared");
    assert!(worst_unrounded <= 0.5, "{worst_unrounded}");
}

#[test]
fn the_inverse_projection_is_gmaps_to_the_bit() {
    // FromPixelToLatLng on GMap's own pixel. pixel / map size is exact - the sizes are powers of
    // two - so the input is the one GMap divided out, and the answer must be identical.
    let mut compared = 0;
    for point in mercator_points() {
        for readout in &point.readouts {
            let size = readout.map_size as f64;
            let ours = LatLon::from_web_mercator(WebMercator {
                x: readout.pixel.0 as f64 / size,
                y: readout.pixel.1 as f64 / size,
            })
            .unwrap();
            assert!(
                same_bits(ours.latitude(), readout.back.0)
                    && same_bits(ours.longitude(), readout.back.1),
                "pixel {:?} at zoom {}: ours {},{}, GMap's {},{}",
                readout.pixel,
                readout.zoom,
                ours.latitude(),
                ours.longitude(),
                readout.back.0,
                readout.back.1
            );
            compared += 1;
        }
    }
    assert!(compared > 3000, "only {compared} pixels compared");
}

#[test]
fn our_round_trip_is_within_a_millimetre_where_gmaps_is_not() {
    let mut ours_worst: (f64, f64) = (0.0, 0.0); // (degrees, metres)
    let mut gmap_worst_z20: f64 = 0.0;
    for point in mercator_points() {
        let original = LatLon::new(point.lat, point.lng).unwrap();
        let back = LatLon::from_web_mercator(original.to_web_mercator()).unwrap();

        // Beyond the clip latitude the projection is flat, so the round trip lands on the clip.
        let expected_lat = point
            .lat
            .clamp(-WEB_MERCATOR_MAX_LATITUDE, WEB_MERCATOR_MAX_LATITUDE);
        let expected = LatLon::new(expected_lat, point.lng).unwrap();
        let degrees = (back.latitude() - expected_lat)
            .abs()
            .max((back.longitude() - point.lng).abs());
        let metres = back.distance_to(expected).0;
        assert!(
            degrees < ROUND_TRIP_DEGREES && metres < MILLIMETRE,
            "{},{} came back as {},{}: {degrees:e} degrees, {metres:e} m",
            point.lat,
            point.lng,
            back.latitude(),
            back.longitude()
        );
        ours_worst = (ours_worst.0.max(degrees), ours_worst.1.max(metres));

        // GMap's round trip at zoom 20, for comparison: it goes through the whole pixel, and the
        // edge pixels are clipped, so it is measured only well inside the map.
        let z20 = point.readouts.iter().find(|r| r.zoom == 20).unwrap();
        let size = z20.map_size as f64;
        let inside = |p: i64| (1..z20.map_size - 1).contains(&p);
        if point.lat.abs() < WEB_MERCATOR_MAX_LATITUDE && inside(z20.pixel.0) && inside(z20.pixel.1)
        {
            let gmap = LatLon::new(z20.back.0, z20.back.1).unwrap();
            gmap_worst_z20 = gmap_worst_z20.max(gmap.distance_to(original).0);
            // Half a pixel's diagonal, in metres on the 6371 km sphere GetDistance measures on.
            let pixel_metres =
                2.0 * std::f64::consts::PI * 6_371_000.0 / size * point.lat.to_radians().cos();
            assert!(
                gmap.distance_to(original).0
                    <= pixel_metres * 0.5 * std::f64::consts::SQRT_2 * 1.001,
                "{},{}: GMap's own round trip is off by more than half a pixel",
                point.lat,
                point.lng
            );
        }
    }
    // Ours is 6 nm at worst over these points; GMap's is 7.8 cm at zoom 20. The second is why the
    // map does not round to whole pixels as GMap does.
    assert!(ours_worst.1 < 1e-6, "ours: {ours_worst:?}");
    assert!(
        gmap_worst_z20 > 0.01,
        "GMap's zoom-20 round trip was expected to lose centimetres, lost {gmap_worst_z20} m"
    );
    println!(
        "round trip: ours worst {:e} degrees, {:e} m; GMap at zoom 20 worst {gmap_worst_z20} m",
        ours_worst.0, ours_worst.1
    );
}

#[test]
fn a_tile_is_gmaps_except_where_gmap_rounds_or_clips_first() {
    // GMap's tile for a position is the tile of its rounded, clipped pixel (PureProjection.cs:
    // 145-148 on MercatorProjection.cs:67-68); ours is the tile the position lies in. Two places
    // differ, and both are asserted rather than skipped:
    // - `x + 0.5` truncated carries a position into the next tile from the half pixel just short
    //   of a tile edge (on or past the edge it rounds down to it), except in the last tile, where
    //   the clip holds it back;
    // - GMap clips to the map, ours does not: a pole projects a hair above row 0, which has no
    //   tile, and +180 is column 0 again for ours but the last column for GMap.
    let tile = f64::from(TILE_SIZE_PX);
    let (mut compared, mut carried, mut clipped) = (0, 0, 0);
    for point in mercator_points() {
        let ours = LatLon::new(point.lat, point.lng).unwrap().to_web_mercator();
        for readout in point.readouts.iter().filter(|r| r.zoom <= MAX_ZOOM) {
            let size = readout.map_size as f64;
            let last = readout.map_size - 1;
            let (x, y) = (ours.x * size, ours.y * size);
            let context = || format!("{},{} at zoom {}", point.lat, point.lng, readout.zoom);

            let containing = TileId::containing(ours, readout.zoom);
            if !(0.0..size).contains(&y) {
                assert_eq!(
                    readout.pixel.1,
                    if y < 0.0 { 0 } else { last },
                    "{}",
                    context()
                );
                assert_eq!(containing, None, "{}", context());
                clipped += 1;
                continue;
            }
            if !(0.0..size).contains(&x) {
                // Longitude is clipped to +-180 first, so x is never past the other side.
                assert!(x >= size, "{}", context());
                assert_eq!(readout.pixel.0, last, "{}", context());
                assert_eq!(containing.map(|t| t.x), Some(0), "{}", context());
                clipped += 1;
                continue;
            }

            let containing = containing.unwrap_or_else(|| panic!("{}", context()));
            let tiles = i64::from(tiles_across(readout.zoom));
            let carry = |unrounded: f64| i64::from(unrounded.rem_euclid(tile) >= tile - 0.5);
            let expected = (
                (i64::from(containing.x) + carry(x)).min(tiles - 1),
                (i64::from(containing.y) + carry(y)).min(tiles - 1),
            );
            let gmaps = (
                readout.pixel.0 / i64::from(TILE_SIZE_PX),
                readout.pixel.1 / i64::from(TILE_SIZE_PX),
            );
            assert_eq!(gmaps, expected, "{}", context());
            compared += 1;
            carried += carry(x).max(carry(y));
        }
    }
    // Both departures must actually occur, or the assertions above prove nothing about them; the
    // lattices put whole rows at the poles and a column on +180 for exactly this.
    assert!(compared > 2000, "only {compared} tiles compared");
    assert!(
        carried > 0 && clipped > 0,
        "carried {carried}, clipped {clipped}"
    );
    println!(
        "tiles: {compared} compared, {carried} of them carried by rounding, {clipped} clipped"
    );
}

/// One `geodesy.csv` pair: the two points, GetDistance, GetBearing, and newpos from the first
/// with that bearing and distance.
struct Pair {
    a: LatLon,
    b: LatLon,
    distance: f64,
    bearing: f64,
    there: (f64, f64),
}

fn pairs() -> Vec<Pair> {
    let pairs: Vec<Pair> = rows(&golden("geodesy.csv"), "pair")
        .iter()
        .map(|f| {
            assert_eq!(f.len(), 8, "{f:?}");
            Pair {
                a: position(f[0], f[1]),
                b: position(f[2], f[3]),
                distance: num(f[4]),
                bearing: num(f[5]),
                there: (num(f[6]), num(f[7])),
            }
        })
        .collect();
    assert!(pairs.len() > 600, "only {} pairs", pairs.len());
    pairs
}

#[test]
fn distance_is_get_distance_to_the_bit() {
    for pair in pairs() {
        let ours = pair.a.distance_to(pair.b).0;
        assert!(
            same_bits(ours, pair.distance),
            "{:?} to {:?}: ours {ours} m, GetDistance {} m",
            pair.a,
            pair.b,
            pair.distance
        );
    }
}

#[test]
fn bearing_is_get_bearing_to_the_bit() {
    for pair in pairs() {
        let ours = pair.a.bearing_to(pair.b).0.0;
        assert!(
            same_bits(ours, pair.bearing),
            "{:?} to {:?}: ours {ours}, GetBearing {}",
            pair.a,
            pair.b,
            pair.bearing
        );
    }
}

/// Holds an offset to newpos: to the bit where newpos's longitude is in range, and to the same
/// meridian where it is not (`LatLon::offset` wraps what the C# leaves past ±180).
fn assert_newpos(start: LatLon, bearing: f64, distance: f64, theirs: (f64, f64)) -> bool {
    let ours = start.offset(Bearing(Degrees(bearing)), Metres(distance));
    let context = || {
        format!(
            "{start:?} {bearing} degrees {distance} m: ours {},{}, newpos {},{}",
            ours.latitude(),
            ours.longitude(),
            theirs.0,
            theirs.1
        )
    };
    assert!(same_bits(ours.latitude(), theirs.0), "{}", context());
    if (-180.0..=180.0).contains(&theirs.1) {
        assert!(same_bits(ours.longitude(), theirs.1), "{}", context());
        false
    } else {
        // Wrapping adds 360 and takes it away again, so the bits below 360's precision go.
        let apart = (ours.longitude() - theirs.1).rem_euclid(360.0);
        assert!(apart.min(360.0 - apart) < 1e-12, "{}", context());
        true
    }
}

#[test]
fn offset_is_newpos_to_the_bit() {
    let mut wrapped = 0;
    for pair in pairs() {
        wrapped += usize::from(assert_newpos(
            pair.a,
            pair.bearing,
            pair.distance,
            pair.there,
        ));
    }
    let geodesy = golden("geodesy.csv");
    let directives = rows(&geodesy, "newpos");
    assert!(
        directives.len() >= 15,
        "only {} newpos lines",
        directives.len()
    );
    for f in directives {
        assert_eq!(f.len(), 6, "{f:?}");
        let start = position(f[0], f[1]);
        wrapped += usize::from(assert_newpos(
            start,
            num(f[2]),
            num(f[3]),
            (num(f[4]), num(f[5])),
        ));
    }
    // The antimeridian cases in points.txt must actually exercise the wrap.
    assert!(
        wrapped >= 2,
        "only {wrapped} offsets crossed the antimeridian"
    );
}

#[test]
fn the_utm_golden_is_well_formed() {
    // Not compared - see the module comment - but kept honest, so the file a later test in
    // mp-mission reads is complete: one line per mercator point, zone signed by hemisphere
    // (PLAN.md §1.3), no refusals.
    let text = golden("utm.csv");
    let utm = rows(&text, "utm");
    assert_eq!(utm.len(), mercator_points().len());
    for f in utm {
        assert_eq!(f.len(), 7, "{f:?}");
        let (lat, zone) = (num(f[0]), int(f[2]));
        assert!((1..=61).contains(&zone.abs()), "{f:?}");
        assert_eq!(zone < 0, lat < 0.0, "{f:?}");
    }
}
