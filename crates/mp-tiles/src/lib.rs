//! Map tiles: providers, an on-disk cache, and the policy that keeps us a good citizen.
//!
//! Deliverable D8. The map is the screen a ground station is mostly looking at, and until this
//! exists it draws a checkerboard.
//!
//! The shape of the thing: a render pass asks the store for a tile and gets an answer immediately,
//! or does not get one. It never waits. A tile that is not in memory is looked for on disk, and a
//! tile that is not on disk is queued for fetching, and in the meantime the map draws the
//! corresponding piece of the tile one zoom level up, scaled - which is why the map fills in
//! progressively rather than appearing blank.

pub mod cache;
pub mod fetch;
pub mod gmap;
pub mod policy;
pub mod prefetch;
pub mod source;
pub mod store;
pub mod urlcache;
pub mod versions;

pub use cache::{CacheError, CacheUsage, CachedTile, ImageFormat, TileCache};
pub use fetch::{FetchError, TileFetcher};
pub use policy::{Decision, FetchPolicy};
pub use source::{
    CSHARP_LIST, CUSTOM, DEFAULT_PROVIDER, GOOGLE_SATELLITE_MAP, OPENSTREETMAP, OPENTOPOMAP,
    SOURCES,
    TileSource, default_source, source_by_id, source_by_name,
};
pub use store::{DecodedTile, StoreStats, TileAnswer, TileStore};
pub use versions::Correction;
