//! Tile providers: where tiles come from and what must be said about them.

use mp_units::TileId;

/// A raster tile provider.
///
/// Deliberately data rather than a trait. Every provider worth having is "substitute z, x and y
/// into a URL", and a trait would invite an implementation that does something else - which is how
/// a map ends up with one provider that ignores the cache or blocks the render thread.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TileSource {
    /// Short identifier, used as the cache directory name. Stable: changing it orphans the cache.
    pub id: &'static str,
    /// What to call it on screen.
    pub label: &'static str,
    /// URL with `{z}`, `{x}`, `{y}` and optionally `{s}` for a subdomain.
    pub url: &'static str,
    /// Subdomains to rotate through, or empty if the provider has none.
    ///
    /// Rotating spreads a screenful of tiles over several hostnames, which is what the providers
    /// that offer subdomains expect. A provider that does not offer them must not be given any:
    /// requesting `a.tile.example.com` when only `tile.example.com` exists fails every tile.
    pub subdomains: &'static [&'static str],
    /// Deepest zoom the provider serves. Asking for more returns errors or blank tiles.
    pub max_zoom: u8,
    /// Attribution text. Not optional: every provider here requires it, and a map that shows
    /// someone else's data without saying whose is not one worth shipping.
    pub attribution: &'static str,
}

impl TileSource {
    /// The URL for one tile, or `None` if the provider does not serve that zoom.
    #[must_use]
    pub fn url_for(&self, tile: TileId) -> Option<String> {
        if tile.z > self.max_zoom {
            return None;
        }
        let mut url = self.url.to_owned();
        if let Some(subdomain) = self.subdomain_for(tile) {
            url = url.replace("{s}", subdomain);
        }
        Some(
            url.replace("{z}", &tile.z.to_string())
                .replace("{x}", &tile.x.to_string())
                .replace("{y}", &tile.y.to_string()),
        )
    }

    /// Which subdomain a tile should come from.
    ///
    /// Chosen from the tile's own coordinates rather than a counter, so the same tile always goes
    /// to the same host. A counter would send a tile to a different host on every pan, defeating
    /// every HTTP cache between here and the provider.
    fn subdomain_for(&self, tile: TileId) -> Option<&'static str> {
        if self.subdomains.is_empty() {
            return None;
        }
        // Summed as u64 and reduced with a u64 modulus, so nothing truncates on a 32-bit target.
        let index = (u64::from(tile.x) + u64::from(tile.y)) % self.subdomains.len() as u64;
        self.subdomains.get(usize::try_from(index).ok()?).copied()
    }
}

/// OpenStreetMap's standard raster tiles.
///
/// Their usage policy requires a real identifying User-Agent, forbids bulk downloading, and asks
/// that clients not send more than a couple of requests at a time. All three are honoured in
/// `fetch.rs`; this is a ground station showing the operator their own surroundings, which is the
/// use the policy contemplates.
pub const OPENSTREETMAP: TileSource = TileSource {
    id: "osm",
    label: "OpenStreetMap",
    url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    // OSM retired its a/b/c subdomains; using them now is a redirect at best.
    subdomains: &[],
    max_zoom: 19,
    attribution: "© OpenStreetMap contributors",
};

/// OpenTopoMap: contour lines and hillshading, which is what a pilot wants over terrain.
pub const OPENTOPOMAP: TileSource = TileSource {
    id: "opentopo",
    label: "OpenTopoMap",
    url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
    subdomains: &["a", "b", "c"],
    max_zoom: 17,
    attribution: "© OpenStreetMap contributors, SRTM | © OpenTopoMap (CC-BY-SA)",
};

/// Esri's World Imagery: aerial and satellite photography.
///
/// Planning over a paddock needs imagery, not a street map - a survey grid over farmland is drawn
/// against field boundaries and tree lines that a road map does not have, and a landing site is
/// chosen by looking at the ground.
///
/// **The URL is `{z}/{y}/{x}`, not `{z}/{x}/{y}`.** ArcGIS orders it row-then-column where every
/// other provider here orders it column-then-row, and the C# says so plainly:
/// `string.Format(UrlFormat, zoom, pos.Y, pos.X)`. Getting it backwards does not fail - it
/// returns a real tile, of somewhere else, and the map looks like imagery that does not match the
/// roads under it.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/ArcGIS/ArcGIS_Imagery_World_2D_MapProvider.cs:55`
///
/// **The endpoint is not the one Mission Planner ships.** Its provider points at
/// `ESRI_Imagery_World_2D`, which Esri retired - it answers 301 today. `World_Imagery` is the
/// current service and returns tiles. PLAN.md §9.1 predicted this ("audit the dead endpoints");
/// this is one of them, found by asking the server rather than by reading the C#.
pub const ESRI_WORLD_IMAGERY: TileSource = TileSource {
    id: "esri-imagery",
    label: "Satellite (Esri)",
    url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    subdomains: &[],
    // Esri serves 23 levels, but coverage above 17 is patchy outside cities and a missing tile
    // costs a request and a blank square. 19 is where the imagery generally stops being useful.
    max_zoom: 19,
    attribution: "Imagery © Esri, Maxar, Earthstar Geographics, and the GIS User Community",
};

/// Every provider we ship.
pub const SOURCES: &[TileSource] = &[OPENSTREETMAP, OPENTOPOMAP, ESRI_WORLD_IMAGERY];

/// Finds a provider by its identifier.
#[must_use]
pub fn source_by_id(id: &str) -> Option<&'static TileSource> {
    SOURCES.iter().find(|source| source.id == id)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ArcGIS orders its path row-then-column, and every other provider here does the opposite.
    ///
    /// Getting it backwards does not fail. It returns a real tile of somewhere else, so the map
    /// shows imagery that quietly does not match the roads drawn over it - which is worse than a
    /// blank square, because a blank square is obviously wrong.
    #[test]
    fn the_imagery_url_is_zoom_then_row_then_column() {
        // The SITL home field outside Canberra at zoom 16.
        let tile = TileId {
            z: 16,
            x: 59_922,
            y: 39_658,
        };
        let url = ESRI_WORLD_IMAGERY.url_for(tile).expect("zoom 16 is served");
        assert!(
            url.ends_with("/tile/16/39658/59922"),
            "the path must be zoom/y/x, got {url}"
        );
        assert!(
            !url.ends_with("/tile/16/59922/39658"),
            "x and y are transposed: {url}"
        );
    }

    /// The endpoint is the live one, not the retired one the C# still points at.
    #[test]
    fn the_imagery_endpoint_is_not_the_retired_one() {
        assert!(
            ESRI_WORLD_IMAGERY.url.contains("World_Imagery"),
            "{}",
            ESRI_WORLD_IMAGERY.url
        );
        assert!(
            !ESRI_WORLD_IMAGERY.url.contains("ESRI_Imagery_World_2D"),
            "that service answers 301 - see the note on this provider"
        );
        assert!(ESRI_WORLD_IMAGERY.url.starts_with("https://"));
    }

    /// Esri's terms require attribution by name, not merely some attribution - which
    /// `every_provider_carries_attribution` already covers for all of them.
    #[test]
    fn the_imagery_attribution_names_esri() {
        assert!(ESRI_WORLD_IMAGERY.attribution.contains("Esri"));
    }

    /// Identifiers are the cache directory names, so a collision mixes two providers' tiles.
    #[test]
    fn provider_identifiers_are_unique() {
        let mut ids: Vec<&str> = SOURCES.iter().map(|source| source.id).collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count, "two providers share a cache directory");
    }

    /// There is a satellite option at all, which is the point of the item.
    #[test]
    fn imagery_is_among_the_providers_offered() {
        assert!(source_by_id("esri-imagery").is_some());
        assert!(SOURCES.len() >= 3);
    }

    fn tile(z: u8, x: i64, y: i64) -> TileId {
        TileId::new(z, x, y).expect("a valid tile")
    }

    #[test]
    fn a_url_substitutes_the_coordinates() {
        let url = OPENSTREETMAP
            .url_for(tile(14, 15_089, 9_814))
            .expect("a url");
        assert_eq!(url, "https://tile.openstreetmap.org/14/15089/9814.png");
    }

    #[test]
    fn a_zoom_the_provider_does_not_serve_has_no_url() {
        // Better than requesting it and getting a blank tile that then gets cached as if it were
        // real data.
        assert!(OPENSTREETMAP.url_for(tile(20, 0, 0)).is_none());
        assert!(OPENSTREETMAP.url_for(tile(19, 0, 0)).is_some());
    }

    #[test]
    fn a_subdomain_is_chosen_from_the_tile_not_from_a_counter() {
        // The same tile must always go to the same host, or every pan sends it somewhere new and
        // defeats every HTTP cache in between.
        let subject = tile(10, 511, 340);
        let first = OPENTOPOMAP.url_for(subject).expect("a url");
        let second = OPENTOPOMAP.url_for(subject).expect("a url");
        assert_eq!(first, second);
        assert!(first.starts_with("https://"), "{first}");
        assert!(
            !first.contains("{s}"),
            "the subdomain was not substituted: {first}"
        );
    }

    #[test]
    fn subdomains_are_actually_spread_across_hosts() {
        let hosts: std::collections::BTreeSet<String> = (0..12)
            .map(|x| {
                let url = OPENTOPOMAP.url_for(tile(10, x, 0)).expect("a url");
                url.split('/').nth(2).unwrap_or_default().to_owned()
            })
            .collect();
        assert_eq!(hosts.len(), OPENTOPOMAP.subdomains.len(), "{hosts:?}");
    }

    #[test]
    fn a_provider_without_subdomains_gets_none() {
        // Requesting a.tile.openstreetmap.org when only tile.openstreetmap.org exists would fail
        // every tile.
        let url = OPENSTREETMAP.url_for(tile(5, 1, 2)).expect("a url");
        assert!(url.starts_with("https://tile.openstreetmap.org/"), "{url}");
    }

    #[test]
    fn every_provider_carries_attribution() {
        // A map that shows someone else's data without saying whose is not one worth shipping,
        // and for these providers it is also a licence breach.
        for source in SOURCES {
            assert!(
                !source.attribution.is_empty(),
                "{} has no attribution",
                source.id
            );
            assert!(!source.id.is_empty());
            assert!(source.url.contains("{z}"), "{} has no zoom", source.id);
            assert!(source.url.contains("{x}"), "{} has no x", source.id);
            assert!(source.url.contains("{y}"), "{} has no y", source.id);
        }
    }

    #[test]
    fn provider_ids_are_unique_because_the_cache_is_keyed_on_them() {
        let mut ids: Vec<&str> = SOURCES.iter().map(|source| source.id).collect();
        let count = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), count, "duplicate provider ids");
    }

    #[test]
    fn a_provider_can_be_found_by_id_and_an_unknown_one_cannot() {
        assert_eq!(source_by_id("osm"), Some(&OPENSTREETMAP));
        assert!(source_by_id("nothing-here").is_none());
    }

    #[test]
    fn declaring_subdomains_requires_the_url_to_use_them() {
        // A provider with subdomains and no {s} would send every tile to one host while looking
        // like it was spreading them.
        for source in SOURCES {
            assert_eq!(
                source.subdomains.is_empty(),
                !source.url.contains("{s}"),
                "{} disagrees with itself about subdomains",
                source.id
            );
        }
    }
}
