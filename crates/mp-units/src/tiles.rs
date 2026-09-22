//! Slippy-map tile arithmetic.
//!
//! Every raster tile server - OpenStreetMap, Google, Bing, and Mission Planner's own cache -
//! addresses tiles the same way: at zoom `z` the Web Mercator unit square is divided into
//! `2^z * 2^z` tiles, numbered from the north-west corner. This module is that arithmetic and
//! nothing else, so it stays pure, dependency-free and exhaustively testable.
//!
//! The one thing worth stating twice: **y grows southward**, matching [`WebMercator`] and screen
//! coordinates, not latitude. A map that renders upside down is usually this.

use crate::WebMercator;

/// Standard tile edge in pixels. Every provider Mission Planner uses serves 256-pixel tiles;
/// "retina" 512-pixel variants exist but are addressed by a different URL, not a different grid.
pub const TILE_SIZE_PX: u32 = 256;

/// The deepest zoom any provider in use offers. Beyond this, tile ids stop being meaningful and
/// `2^z` starts threatening `u32`.
pub const MAX_ZOOM: u8 = 22;

/// One tile in the slippy-map grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TileId {
    /// Zoom level; the world is `2^z` tiles across.
    pub z: u8,
    /// Column, counting east from 180°W.
    pub x: u32,
    /// Row, counting south from the projection's north edge.
    pub y: u32,
}

/// Converts a floored or ceiled tile coordinate to an integer.
///
/// Tile grids are at most `2^22` across and the inputs are already floored or ceiled, so this is
/// exact for every value the map can produce. Anything else - a NaN from a degenerate viewport,
/// an infinity from a zero span - is rejected rather than silently wrapped, because a wrapped
/// tile index fetches a tile from the wrong side of the planet.
fn to_tile_index(value: f64) -> Option<i64> {
    if !value.is_finite() {
        return None;
    }
    const LIMIT: f64 = (1i64 << 32) as f64;
    if value.abs() > LIMIT {
        return None;
    }
    // Guarded above: finite, and within a range i64 represents exactly.
    #[allow(clippy::cast_possible_truncation)]
    Some(value as i64)
}

/// Tiles across the world at a zoom level.
#[must_use]
pub const fn tiles_across(z: u8) -> u32 {
    1u32 << z
}

impl TileId {
    /// Creates a tile id, wrapping `x` around the date line and clamping `y` at the poles.
    ///
    /// Wrapping x is correct: longitude is cyclic, and a viewport straddling the date line asks
    /// for tiles either side of it. Clamping y is also correct: there is no tile above the north
    /// edge, and a request for one should be dropped rather than wrapped to the south pole.
    #[must_use]
    pub fn new(z: u8, x: i64, y: i64) -> Option<Self> {
        if z > MAX_ZOOM {
            return None;
        }
        let n = i64::from(tiles_across(z));
        if y < 0 || y >= n {
            return None;
        }
        let wrapped = x.rem_euclid(n);
        Some(Self {
            z,
            x: u32::try_from(wrapped).ok()?,
            y: u32::try_from(y).ok()?,
        })
    }

    /// The tile containing a projected position at this zoom.
    #[must_use]
    pub fn containing(position: WebMercator, z: u8) -> Option<Self> {
        let n = f64::from(tiles_across(z));
        // floor, not truncate: a position in the western hemisphere has x below 0.5, and
        // truncation toward zero would be wrong for any negative value that slipped through.
        let x = to_tile_index((position.x * n).floor())?;
        let y = to_tile_index((position.y * n).floor())?;
        Self::new(z, x, y)
    }

    /// The projected rectangle this tile covers, as (north-west, south-east).
    #[must_use]
    pub fn bounds(self) -> (WebMercator, WebMercator) {
        let n = f64::from(tiles_across(self.z));
        let nw = WebMercator {
            x: f64::from(self.x) / n,
            y: f64::from(self.y) / n,
        };
        let se = WebMercator {
            x: (f64::from(self.x) + 1.0) / n,
            y: (f64::from(self.y) + 1.0) / n,
        };
        (nw, se)
    }

    /// The tile one zoom level out that contains this one.
    ///
    /// This is what a map draws while a tile is still downloading: the parent, scaled up. It is
    /// blurry and it is instantly available, which is the correct trade while panning.
    #[must_use]
    pub fn parent(self) -> Option<Self> {
        if self.z == 0 {
            return None;
        }
        Some(Self {
            z: self.z - 1,
            x: self.x / 2,
            y: self.y / 2,
        })
    }

    /// The four tiles one zoom level in.
    #[must_use]
    pub fn children(self) -> Option<[Self; 4]> {
        if self.z >= MAX_ZOOM {
            return None;
        }
        let (z, x, y) = (self.z + 1, self.x * 2, self.y * 2);
        Some([
            Self { z, x, y },
            Self { z, x: x + 1, y },
            Self { z, x, y: y + 1 },
            Self {
                z,
                x: x + 1,
                y: y + 1,
            },
        ])
    }

    /// Which quadrant of its parent this tile occupies, as (east, south).
    ///
    /// Needed to pick the right quarter of a parent tile's texture when standing in for a tile
    /// that has not arrived.
    #[must_use]
    pub const fn quadrant_in_parent(self) -> (bool, bool) {
        (self.x % 2 == 1, self.y % 2 == 1)
    }
}

/// The zoom at which a projected span fills a viewport at native tile resolution.
///
/// Choosing a zoom that is too low gives a blurry map stretched from too few tiles; too high
/// wastes bandwidth drawing detail below one screen pixel.
#[must_use]
pub fn zoom_for_span(span: f64, viewport_px: f32, tile_px: u32) -> u8 {
    if span <= 0.0 || viewport_px <= 0.0 || tile_px == 0 {
        return 0;
    }
    // Tiles needed to cover the span: span * 2^z. We want that many tiles to be about
    // viewport_px / tile_px, so 2^z = viewport_px / (tile_px * span).
    let ideal = f64::from(viewport_px) / (f64::from(tile_px) * span);
    if ideal <= 1.0 {
        return 0;
    }
    let Some(z) = to_tile_index(ideal.log2().round()) else {
        return 0;
    };
    u8::try_from(z.clamp(0, i64::from(MAX_ZOOM))).unwrap_or(0)
}

/// Every tile needed to cover a projected rectangle at a zoom level, ordered centre-outwards.
///
/// Centre-first matters on a slow link: the tiles a pilot is looking at arrive before the corners.
#[must_use]
pub fn tiles_for_view(north_west: WebMercator, south_east: WebMercator, z: u8) -> Vec<TileId> {
    let n = f64::from(tiles_across(z));
    let (Some(x0), Some(x1), Some(y0), Some(y1)) = (
        to_tile_index((north_west.x * n).floor()),
        to_tile_index((south_east.x * n).ceil()),
        to_tile_index((north_west.y * n).floor()),
        to_tile_index((south_east.y * n).ceil()),
    ) else {
        return Vec::new();
    };
    let (x1, y1) = (x1 - 1, y1 - 1);

    let centre_x = (x0 + x1) as f64 / 2.0;
    let centre_y = (y0 + y1) as f64 / 2.0;

    let mut out = Vec::new();
    for y in y0..=y1 {
        for x in x0..=x1 {
            if let Some(tile) = TileId::new(z, x, y) {
                out.push(tile);
            }
        }
    }
    out.sort_by(|a, b| {
        // Compare by squared distance from the view centre. Tiles wrap in x, so compare on the
        // pre-wrap column where possible by using the tile's own coordinates.
        let da = (f64::from(a.x) - centre_x).powi(2) + (f64::from(a.y) - centre_y).powi(2);
        let db = (f64::from(b.x) - centre_x).powi(2) + (f64::from(b.y) - centre_y).powi(2);
        da.partial_cmp(&db)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.cmp(b))
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LatLon;

    #[test]
    fn the_world_is_one_tile_at_zoom_zero() {
        assert_eq!(tiles_across(0), 1);
        assert_eq!(tiles_across(1), 4 / 2);
        assert_eq!(tiles_across(10), 1024);

        let tile = TileId::containing(WebMercator { x: 0.5, y: 0.5 }, 0).expect("valid");
        assert_eq!(tile, TileId { z: 0, x: 0, y: 0 });
        let (nw, se) = tile.bounds();
        assert!((nw.x - 0.0).abs() < 1e-12 && (nw.y - 0.0).abs() < 1e-12);
        assert!((se.x - 1.0).abs() < 1e-12 && (se.y - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_known_position_lands_in_a_known_tile() {
        // ArduPilot's default SITL location, CMAC near Canberra.
        let cmac = LatLon::new(-35.363_262, 149.165_237)
            .expect("valid")
            .to_web_mercator();

        // At zoom 1 the world is 2x2: CMAC is east of Greenwich and south of the equator.
        let z1 = TileId::containing(cmac, 1).expect("valid");
        assert_eq!(z1, TileId { z: 1, x: 1, y: 1 }, "south-east quadrant");

        // Deeper zooms must stay inside their parent.
        let z14 = TileId::containing(cmac, 14).expect("valid");
        let mut walk = z14;
        for _ in 0..13 {
            walk = walk.parent().expect("has a parent");
        }
        assert_eq!(walk.z, 1);
        assert_eq!(
            (walk.x, walk.y),
            (z1.x, z1.y),
            "zoom 14 tile must sit inside the zoom 1 tile"
        );
    }

    #[test]
    fn tile_bounds_contain_the_position_that_selected_them() {
        for (lat, lon) in [
            (0.0, 0.0),
            (-35.36, 149.16),
            (51.48, -0.0015),
            (60.0, -170.0),
        ] {
            let position = LatLon::new(lat, lon).expect("valid").to_web_mercator();
            for z in [0u8, 1, 5, 12, 18] {
                let tile = TileId::containing(position, z).expect("valid");
                let (nw, se) = tile.bounds();
                assert!(
                    position.x >= nw.x && position.x <= se.x,
                    "z{z}: x {} outside [{}, {}]",
                    position.x,
                    nw.x,
                    se.x
                );
                assert!(
                    position.y >= nw.y && position.y <= se.y,
                    "z{z}: y {} outside [{}, {}]",
                    position.y,
                    nw.y,
                    se.y
                );
            }
        }
    }

    #[test]
    fn parents_and_children_are_inverses() {
        let tile = TileId {
            z: 10,
            x: 512,
            y: 300,
        };
        for child in tile.children().expect("has children") {
            assert_eq!(child.parent(), Some(tile));
        }
        assert_eq!(
            TileId { z: 0, x: 0, y: 0 }.parent(),
            None,
            "zoom 0 has no parent"
        );
    }

    #[test]
    fn quadrants_identify_the_right_quarter_of_the_parent() {
        let parent = TileId { z: 5, x: 10, y: 20 };
        let [nw, ne, sw, se] = parent.children().expect("children");
        assert_eq!(nw.quadrant_in_parent(), (false, false));
        assert_eq!(ne.quadrant_in_parent(), (true, false));
        assert_eq!(sw.quadrant_in_parent(), (false, true));
        assert_eq!(se.quadrant_in_parent(), (true, true));
    }

    #[test]
    fn longitude_wraps_and_latitude_does_not() {
        // One tile east of the last column at zoom 3 is the first column again.
        assert_eq!(TileId::new(3, 8, 4), Some(TileId { z: 3, x: 0, y: 4 }));
        assert_eq!(TileId::new(3, -1, 4), Some(TileId { z: 3, x: 7, y: 4 }));

        // There is no tile above the north edge or below the south edge.
        assert_eq!(TileId::new(3, 0, -1), None);
        assert_eq!(TileId::new(3, 0, 8), None);
    }

    #[test]
    fn a_full_world_view_asks_for_every_tile() {
        let nw = WebMercator { x: 0.0, y: 0.0 };
        let se = WebMercator { x: 1.0, y: 1.0 };
        assert_eq!(tiles_for_view(nw, se, 0).len(), 1);
        assert_eq!(tiles_for_view(nw, se, 1).len(), 4);
        assert_eq!(tiles_for_view(nw, se, 2).len(), 16);
    }

    #[test]
    fn tiles_arrive_centre_first() {
        let nw = WebMercator { x: 0.0, y: 0.0 };
        let se = WebMercator { x: 1.0, y: 1.0 };
        let tiles = tiles_for_view(nw, se, 3);
        assert_eq!(tiles.len(), 64);

        // The first tiles fetched must be nearer the middle than the last ones.
        let centre = 3.5f64;
        let distance = |t: &TileId| (f64::from(t.x) - centre).hypot(f64::from(t.y) - centre);
        assert!(
            distance(&tiles[0]) < distance(&tiles[63]),
            "first tile {:?} should be nearer the centre than last {:?}",
            tiles[0],
            tiles[63]
        );
    }

    #[test]
    fn a_4k_viewport_needs_a_bounded_number_of_tiles() {
        // A 3840x2160 viewport at native resolution: the zoom is chosen so tiles are 1:1, so the
        // count must be about (3840/256) * (2160/256) = 15 * 8.4, plus edges.
        let span = 0.001_f64;
        let z = zoom_for_span(span, 3840.0, TILE_SIZE_PX);
        let n = f64::from(tiles_across(z));
        let centre = WebMercator { x: 0.5, y: 0.5 };
        let half_x = span / 2.0;
        let half_y = span * (2160.0 / 3840.0) / 2.0;
        let tiles = tiles_for_view(
            WebMercator {
                x: centre.x - half_x,
                y: centre.y - half_y,
            },
            WebMercator {
                x: centre.x + half_x,
                y: centre.y + half_y,
            },
            z,
        );
        assert!(
            (100..=220).contains(&tiles.len()),
            "a 4K viewport needed {} tiles at zoom {z} (world is {n} tiles across)",
            tiles.len()
        );
    }

    #[test]
    fn zoom_selection_is_monotonic_and_bounded() {
        // Halving the span should ask for one zoom level deeper.
        let a = zoom_for_span(0.01, 1000.0, TILE_SIZE_PX);
        let b = zoom_for_span(0.005, 1000.0, TILE_SIZE_PX);
        assert_eq!(b, a + 1, "halving the span should step one zoom level");

        assert_eq!(
            zoom_for_span(1.0, 256.0, TILE_SIZE_PX),
            0,
            "whole world in one tile"
        );
        assert!(
            zoom_for_span(1e-12, 4000.0, TILE_SIZE_PX) <= MAX_ZOOM,
            "zoom must stay bounded"
        );
        assert_eq!(
            zoom_for_span(0.0, 1000.0, TILE_SIZE_PX),
            0,
            "degenerate span is zoom 0"
        );
    }
}
