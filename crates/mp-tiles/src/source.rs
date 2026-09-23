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

/// Every provider we ship.
pub const SOURCES: &[TileSource] = &[OPENSTREETMAP, OPENTOPOMAP];

/// Finds a provider by its identifier.
#[must_use]
pub fn source_by_id(id: &str) -> Option<&'static TileSource> {
    SOURCES.iter().find(|source| source.id == id)
}

#[cfg(test)]
mod tests {
    use super::*;

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
