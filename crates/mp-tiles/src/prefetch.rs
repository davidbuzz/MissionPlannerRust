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

//! Tile prefetching, as GMap.NET's `TilePrefetcher` and `TilePrefetcherMenu` do it for the
//! planner's Prefetch and Prefetch WP Path (`GCSViews/FlightPlanner.cs:3305-3365, 5030-5080`
//! @ efb0801; `ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/TilePrefetcher.cs`,
//! `TilePrefetcherMenu.cs`; GPL-3.0-only).
//!
//! The menu counts the tiles an area needs at each zoom (`GetAreaTileNumber`) and estimates
//! their size at 15,000 bytes a tile. The prefetcher then walks `GetAreaTileList` for one zoom,
//! shuffled, skipping tiles the cache already holds and fetching the rest into it, reporting
//! "Fetching tile at zoom (z): i of n, complete: p%" as it goes, until cancelled.

use std::sync::atomic::{AtomicBool, Ordering};

use mp_units::{LatLon, TileId, WebMercator, tiles};

use crate::cache::TileCache;

/// `TilePrefetcherMenu`'s `averageTileSize`.
/// `// C#: TilePrefetcherMenu.cs:44`
pub const AVERAGE_TILE_BYTES: u64 = 15_000;

/// A `RectLatLng`: the area to fetch, by its top-left and bottom-right corners.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Area {
    /// The northern edge, degrees.
    pub top: f64,
    /// The western edge, degrees.
    pub left: f64,
    /// The southern edge.
    pub bottom: f64,
    /// The eastern edge.
    pub right: f64,
}

impl Area {
    /// The rectangle between two corners, whichever way round they come: `FetchPath`'s
    /// `Math.Max`/`Math.Min` over a segment's ends.
    /// `// C#: GCSViews/FlightPlanner.cs:3338-3345`
    #[must_use]
    pub fn between(a: LatLon, b: LatLon) -> Self {
        Self {
            top: a.latitude().max(b.latitude()),
            left: a.longitude().min(b.longitude()),
            bottom: a.latitude().min(b.latitude()),
            right: a.longitude().max(b.longitude()),
        }
    }

    /// `RectLatLng.IsEmpty`: no size at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.top <= self.bottom && self.left >= self.right
    }

    fn corners(&self) -> Option<(WebMercator, WebMercator)> {
        let north_west = LatLon::new(self.top, self.left).ok()?.to_web_mercator();
        let south_east = LatLon::new(self.bottom, self.right).ok()?.to_web_mercator();
        Some((north_west, south_east))
    }
}

/// `GetAreaTileList(area, zoom, 0)`: every tile from the top-left corner's to the bottom-right's,
/// in row order. Empty for an area that cannot be projected.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET/PureProjection.cs:213-236`
#[must_use]
pub fn area_tiles(area: &Area, zoom: u8) -> Vec<TileId> {
    let Some((north_west, south_east)) = area.corners() else {
        return Vec::new();
    };
    let n = f64::from(tiles::tiles_across(zoom));
    #[allow(clippy::cast_possible_truncation)] // clamped to the tile row, at most 2^24 - 1
    let index = |value: f64| -> Option<i64> {
        let index = (value * n).floor();
        (index.is_finite()).then(|| index.clamp(0.0, n - 1.0) as i64)
    };
    let (Some(x0), Some(x1), Some(y0), Some(y1)) = (
        index(north_west.x),
        index(south_east.x),
        index(north_west.y),
        index(south_east.y),
    ) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for x in x0..=x1 {
        for y in y0..=y1 {
            if let Some(tile) = TileId::new(zoom, x, y) {
                out.push(tile);
            }
        }
    }
    out
}

/// `GetAreaTileNumber(area, zoom, 0)`: how many tiles [`area_tiles`] would list.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET/PureProjection.cs:238-244`
#[must_use]
pub fn area_tile_count(area: &Area, zoom: u8) -> u64 {
    area_tiles(area, zoom).len() as u64
}

/// `#,0` in a culture whose group separator is a space: thousands grouped with spaces.
/// `// C#: TilePrefetcherMenu.cs:34-35`
#[must_use]
pub fn grouped(count: u64) -> String {
    let digits = count.to_string();
    let mut out = String::new();
    for (i, ch) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push(' ');
        }
        out.push(ch);
    }
    out
}

/// `sizeConverter`: bytes as `N2` gigabytes, megabytes, kilobytes or bytes, each with its unit
/// and a trailing space, as the label shows it.
/// `// C#: TilePrefetcherMenu.cs:48-85`
#[must_use]
pub fn size_text(bytes: f32) -> String {
    const GB: f32 = 1024.0 * 1024.0 * 1024.0;
    const MB: f32 = 1024.0 * 1024.0;
    const KB: f32 = 1024.0;
    let (value, unit) = if bytes >= GB {
        (bytes / GB, " GB ")
    } else if bytes >= MB {
        (bytes / MB, " MB ")
    } else if bytes >= KB {
        (bytes / KB, " KB ")
    } else {
        (bytes, " B ")
    };
    // `N2`: two decimals, thousands grouped with commas.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a byte count, whole
    let whole = value.trunc() as u64;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 0 to 100
    let hundredths = ((value - value.trunc()) * 100.0).round() as u64;
    let (whole, hundredths) = if hundredths >= 100 {
        (whole + 1, 0)
    } else {
        (whole, hundredths)
    };
    format!("{}.{hundredths:02}{unit}", grouped(whole).replace(' ', ","))
}

/// `UpdateTilesCount`: the "Zoom i : n" line per zoom from `min` to `max`, and the total's line.
/// `// C#: TilePrefetcherMenu.cs:27-46`
#[must_use]
pub fn menu_text(area: &Area, min: u8, max: u8) -> (Vec<String>, String) {
    let mut lines = Vec::new();
    let mut total = 0u64;
    for zoom in min..=max {
        let count = area_tile_count(area, zoom);
        lines.push(format!("Zoom {zoom} : {}", grouped(count)));
        total += count;
    }
    let plural = if total > 1 { "s" } else { "" };
    let estimate = format!(
        "Estimated: {} tile{plural} for {}",
        grouped(total),
        size_text((total * AVERAGE_TILE_BYTES) as f32)
    );
    (lines, estimate)
}

/// Where one zoom's fetch stands: `worker_ProgressChanged`'s numbers.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Progress {
    /// The zoom being fetched.
    pub zoom: u8,
    /// Tiles handled so far, `i + 1`.
    pub done: usize,
    /// `all`: tiles in the list.
    pub all: usize,
    /// Tiles now in the cache, `countOk`.
    pub ok: usize,
    /// `e.ProgressPercentage`.
    pub percent: i32,
}

impl Progress {
    /// `label1.Text`.
    /// `// C#: TilePrefetcher.cs:200-201`
    #[must_use]
    pub fn label(&self) -> String {
        format!(
            "Fetching tile at zoom ({}): {} of {}, complete: {}%",
            self.zoom, self.done, self.all, self.percent
        )
    }
}

/// A source of tile bytes for the prefetcher: the network, or a test's stand-in.
pub trait FetchTile {
    /// The tile's image bytes, or `None` where the server has none.
    fn fetch(&self, tile: TileId) -> Option<Vec<u8>>;
}

impl<F: Fn(TileId) -> Option<Vec<u8>>> FetchTile for F {
    fn fetch(&self, tile: TileId) -> Option<Vec<u8>> {
        self(tile)
    }
}

/// `Stuff.Shuffle`: the list in a random order, here from a small generator seeded by the
/// caller so a test's order is its own.
fn shuffle(list: &mut [TileId], mut seed: u64) {
    for i in (1..list.len()).rev() {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let j = (seed >> 33) as usize % (i + 1);
        list.swap(i, j);
    }
}

/// `worker_DoWork` for one zoom: the area's tiles, shuffled, each skipped when the cache has
/// it (`CheckImageExist`) and fetched into the cache otherwise (`GetImageFrom`); a fetch that
/// fails is retried `retry` times after 1111 ms (the planner passes 0) and then left. Progress
/// is reported after every tile, and `cancel` ends the walk between two tiles. Returns
/// `countOk`, the tiles the cache holds at the end of the walk.
/// `// C#: TilePrefetcher.cs:113-141, 178-247`
#[allow(clippy::too_many_arguments)]
pub fn run(
    area: &Area,
    zoom: u8,
    provider: &str,
    cache: &TileCache,
    fetch: &dyn FetchTile,
    retry: u8,
    cancel: &AtomicBool,
    seed: u64,
    report: &mut dyn FnMut(&Progress),
) -> usize {
    let mut list = area_tiles(area, zoom);
    shuffle(&mut list, seed);
    let all = list.len();
    let mut ok = 0;
    report(&Progress {
        zoom,
        done: 0,
        all,
        ok,
        percent: 0,
    });
    for (i, tile) in list.iter().enumerate() {
        if cancel.load(Ordering::Acquire) {
            break;
        }
        let mut attempts = 0u8;
        loop {
            let cached = cache.contains(provider, *tile)
                || fetch
                    .fetch(*tile)
                    .is_some_and(|bytes| cache.write(provider, *tile, &bytes).is_ok());
            if cached {
                ok += 1;
                break;
            }
            if attempts < retry {
                attempts += 1;
                wasm_thread::sleep(std::time::Duration::from_millis(1111));
                continue;
            }
            break;
        }
        report(&Progress {
            zoom,
            done: i + 1,
            all,
            ok,
            percent: i32::try_from((i + 1) * 100 / all.max(1)).unwrap_or(100),
        });
    }
    ok
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmac() -> Area {
        Area::between(
            LatLon::new(-35.36, 149.16).expect("a"),
            LatLon::new(-35.37, 149.17).expect("b"),
        )
    }

    #[test]
    fn an_area_lists_every_tile_between_its_corners_and_counts_them() {
        let area = cmac();
        assert!(!area.is_empty());
        // A kilometre-wide area is one or two tiles at zoom 14, a few dozen at zoom 18.
        let z14 = area_tiles(&area, 14);
        assert!((1..=4).contains(&z14.len()), "{}", z14.len());
        assert!(z14.iter().all(|tile| tile.z == 14));
        // Nine tiles across and nine down at zoom 18.
        let z18 = area_tile_count(&area, 18);
        assert_eq!(z18, 81);
        assert_eq!(area_tile_count(&area, 1), 1);
    }

    #[test]
    fn the_menu_says_a_line_a_zoom_and_an_estimate_at_fifteen_kilobytes_a_tile() {
        let area = cmac();
        let (lines, estimate) = menu_text(&area, 1, 3);
        assert_eq!(lines, vec!["Zoom 1 : 1", "Zoom 2 : 1", "Zoom 3 : 1"]);
        // 3 tiles × 15,000 bytes = 45,000 bytes = 43.95 KB.
        assert_eq!(estimate, "Estimated: 3 tiles for 43.95 KB ");
        let (_, one) = menu_text(&area, 1, 1);
        assert_eq!(one, "Estimated: 1 tile for 14.65 KB ");
    }

    #[test]
    fn counts_group_thousands_with_spaces_and_sizes_read_as_dotnet_n2() {
        assert_eq!(grouped(0), "0");
        assert_eq!(grouped(999), "999");
        assert_eq!(grouped(1_000), "1 000");
        assert_eq!(grouped(1_234_567), "1 234 567");
        assert_eq!(size_text(512.0), "512.00 B ");
        assert_eq!(size_text(1536.0), "1.50 KB ");
        assert_eq!(size_text(15_000.0 * 100.0), "1.43 MB ");
        assert_eq!(size_text(3.0 * 1024.0 * 1024.0 * 1024.0), "3.00 GB ");
    }

    /// A temporary directory that removes itself.
    struct Scratch(std::path::PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path = mp_os::temp_dir().join(format!(
                "mp-tiles-prefetch-{name}-{}-{:?}",
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

    #[test]
    fn a_run_fetches_what_the_cache_lacks_skips_what_it_has_and_reports_each_tile() {
        let dir = Scratch::new("run");
        let cache = TileCache::new(&dir.0);
        let area = cmac();
        let tiles = area_tiles(&area, 18);
        // One tile already cached: a real PNG, as the cache refuses anything else.
        let png = {
            let mut bytes = Vec::new();
            image::RgbaImage::new(4, 4)
                .write_to(
                    &mut std::io::Cursor::new(&mut bytes),
                    image::ImageFormat::Png,
                )
                .expect("png");
            bytes
        };
        cache.write("Test", tiles[0], &png).expect("cached");
        let fetched = std::cell::Cell::new(0);
        let fetch = |tile: TileId| {
            fetched.set(fetched.get() + 1);
            // Every other tile is missing on the server.
            tile.x.is_multiple_of(2).then(|| png.clone())
        };
        let mut reports = Vec::new();
        let ok = run(
            &area,
            18,
            "Test",
            &cache,
            &fetch,
            0,
            &AtomicBool::new(false),
            7,
            &mut |progress: &Progress| reports.push(progress.clone()),
        );
        assert_eq!(
            fetched.get(),
            tiles.len() - 1,
            "the cached one is not fetched"
        );
        assert_eq!(reports.len(), tiles.len() + 1);
        assert_eq!(reports.last().map(|p| p.percent), Some(100));
        assert_eq!(reports.last().map(|p| p.done), Some(tiles.len()));
        assert_eq!(reports.last().map(|p| p.ok), Some(ok));
        assert!(ok >= 1 && ok < tiles.len());
        let held = tiles
            .iter()
            .filter(|tile| cache.contains("Test", **tile))
            .count();
        assert_eq!(held, ok);
        assert_eq!(
            reports[3].label(),
            format!(
                "Fetching tile at zoom (18): 3 of {}, complete: {}%",
                tiles.len(),
                300 / tiles.len()
            )
        );
    }

    #[test]
    fn a_cancelled_run_stops_between_tiles() {
        let dir = Scratch::new("cancel");
        let cache = TileCache::new(&dir.0);
        let cancel = AtomicBool::new(true);
        let mut reports = 0;
        let ok = run(
            &cmac(),
            18,
            "Test",
            &cache,
            &|_tile: TileId| None,
            0,
            &cancel,
            1,
            &mut |_: &Progress| reports += 1,
        );
        assert_eq!(ok, 0);
        assert_eq!(reports, 1);
    }
}
