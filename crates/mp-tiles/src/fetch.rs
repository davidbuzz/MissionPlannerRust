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

//! Fetching tiles over HTTP.
//!
//! Blocking, on a thread of its own. The link's I/O thread works the same way and for the same
//! reason: an async runtime would be machinery with no user, and the render thread must never wait
//! on this regardless of how it is written.

use std::time::Duration;

use mp_units::TileId;

use crate::source::TileSource;

/// How we identify ourselves to tile servers.
///
/// Not optional and not a lie. OpenStreetMap's tile usage policy requires a User-Agent that
/// identifies the application, and servers block clients that send a generic one or none. A
/// ground station pretending to be a browser is both rude and, when it gets the address blocked,
/// self-defeating.
///
/// The C# is the same shape. GMap.NET's default is a browser string (`GMapProvider.cs:326-328`),
/// but Mission Planner replaces it at startup with its product name, version and operating system,
/// and that is what every provider, Google's and Bing's included, receives from it.
/// `// C#: Program.cs:373-375`
pub const USER_AGENT: &str = concat!(
    "MissionPlannerRust/",
    env!("CARGO_PKG_VERSION"),
    " (+https://github.com/davidbuzz/MissionPlannerRust)"
);

/// How long to wait for a tile before giving up on it.
///
/// Short on purpose. A tile that has not arrived in ten seconds is not going to be useful for the
/// view the operator is looking at now, and the slot it holds is better spent on a tile that is.
pub const TIMEOUT: Duration = Duration::from_secs(10);

/// What every request accepts, `GMapProvider.requestAccept`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:386, 404, 450-451`
pub const ACCEPT: &str = "*/*";

/// The largest tile we will accept.
///
/// A 256x256 tile is a few tens of kilobytes. Anything approaching this is not a tile, and reading
/// it into memory unbounded is how a hostile or broken server exhausts a ground station's RAM.
pub const MAX_TILE_BYTES: usize = 4 * 1024 * 1024;

/// Why a tile could not be fetched.
#[derive(Debug, thiserror::Error)]
pub enum FetchError {
    /// The provider does not serve that zoom.
    #[error("{provider} does not serve zoom {zoom}")]
    UnsupportedZoom {
        /// Provider name.
        provider: &'static str,
        /// Zoom asked for.
        zoom: u8,
    },
    /// The request failed or the server refused.
    #[error("fetching {url}: {message}")]
    Request {
        /// What was asked for.
        url: String,
        /// What went wrong.
        message: String,
    },
    /// The server answered with something that is not a tile.
    #[error("{url} returned {bytes} bytes that are not an image")]
    NotAnImage {
        /// What was asked for.
        url: String,
        /// How much came back.
        bytes: usize,
    },
    /// The response was too large to be a tile.
    #[error("{url} returned more than {limit} bytes")]
    TooLarge {
        /// What was asked for.
        url: String,
        /// The limit that was exceeded.
        limit: usize,
    },
}

/// Fetches tiles over HTTP.
///
/// Cheap to clone: the agent inside is shared, connection pool and all.
#[derive(Debug, Clone)]
pub struct TileFetcher {
    // In a web page the browser asks (mp_os::http), not the agent.
    #[cfg_attr(target_family = "wasm", allow(dead_code))]
    agent: ureq::Agent,
}

impl Default for TileFetcher {
    fn default() -> Self {
        Self::new()
    }
}

impl TileFetcher {
    /// A fetcher with the timeouts and identity the providers expect.
    ///
    /// It goes through the proxy the environment names (`HTTP_PROXY` and its relatives), if any.
    #[must_use]
    pub fn new() -> Self {
        let config = ureq::Agent::config_builder()
            .timeout_global(Some(TIMEOUT))
            .user_agent(USER_AGENT)
            .build();
        Self {
            agent: config.into(),
        }
    }

    /// The same fetcher, sending every request through the given proxy, e.g.
    /// `http://127.0.0.1:3128` - what [`TileFetcher::new`] does when the environment names one,
    /// without needing the environment to. How a test watches what goes over the wire without
    /// anything leaving the machine.
    pub fn through_proxy(proxy: &str) -> Result<Self, FetchError> {
        let proxy = ureq::Proxy::new(proxy).map_err(|error| FetchError::Request {
            url: proxy.to_owned(),
            message: error.to_string(),
        })?;
        let config = ureq::Agent::config_builder()
            .timeout_global(Some(TIMEOUT))
            .user_agent(USER_AGENT)
            .proxy(Some(proxy))
            .build();
        Ok(Self {
            agent: config.into(),
        })
    }

    /// A GET carrying the headers `GetTileImageUsingHttp` and `GetContentUsingHttp` send: the
    /// User-Agent, `Accept: */*`, and the provider's `Referer` when it has one.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:401-406, 447-453`
    #[cfg(not(target_family = "wasm"))]
    fn get(
        &self,
        url: &str,
        referer: &str,
    ) -> Result<ureq::http::Response<ureq::Body>, FetchError> {
        // Over https first, then the address as written (mp_os::https_first).
        let call = |address: &str| {
            let mut request = self.agent.get(address).header("Accept", ACCEPT);
            if !referer.is_empty() {
                request = request.header("Referer", referer);
            }
            request.call()
        };
        mp_os::ask_https_first(url, call, |error| {
            matches!(error, ureq::Error::StatusCode(_))
        })
        .map_err(|error| FetchError::Request {
            url: url.to_owned(),
            message: error.to_string(),
        })
    }

    /// Fetches one tile, blocking until it arrives or fails.
    ///
    /// Returns the encoded bytes; decoding happens elsewhere, because this runs on the fetch
    /// thread and decoding is the caller's problem to schedule.
    pub fn fetch(&self, source: &TileSource, tile: TileId) -> Result<Vec<u8>, FetchError> {
        let url = source.url_for(tile).ok_or(FetchError::UnsupportedZoom {
            provider: source.id,
            zoom: tile.z,
        })?;

        #[cfg(not(target_family = "wasm"))]
        let mut response = self.get(&url, source.referer)?;

        // Bounded read. A server that streams forever, or lies about its content length, must not
        // be able to exhaust memory.
        #[cfg(not(target_family = "wasm"))]
        let bytes = response
            .body_mut()
            .with_config()
            .limit(MAX_TILE_BYTES as u64)
            .read_to_vec()
            .map_err(|error| FetchError::Request {
                url: url.clone(),
                message: error.to_string(),
            })?;
        // In a web page, through the browser (mp_os::http); the size is checked below as here.
        #[cfg(target_family = "wasm")]
        let bytes = page_get(&url)?;

        if bytes.len() >= MAX_TILE_BYTES {
            return Err(FetchError::TooLarge {
                url,
                limit: MAX_TILE_BYTES,
            });
        }
        // Checked here as well as in the cache, so a bad response is reported as a fetch failure -
        // which the policy then backs off - rather than as a cache error that looks like a disk
        // problem.
        if crate::cache::ImageFormat::sniff(&bytes).is_none() {
            return Err(FetchError::NotAnImage {
                url,
                bytes: bytes.len(),
            });
        }
        Ok(bytes)
    }

    /// Fetches a page as text: `GetContentUsingHttp`, which the version checks use.
    ///
    /// Bounded like a tile, because the pages are a few hundred kilobytes and a server that sends
    /// more is not sending the page.
    /// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:443-461`
    #[cfg(not(target_family = "wasm"))]
    pub fn fetch_text(&self, url: &str, referer: &str) -> Result<String, FetchError> {
        let mut response = self.get(url, referer)?;
        response
            .body_mut()
            .with_config()
            .limit(MAX_TILE_BYTES as u64)
            .read_to_string()
            .map_err(|error| FetchError::Request {
                url: url.to_owned(),
                message: error.to_string(),
            })
    }

    /// In a web page, through the browser (the `Referer` is the browser's own).
    #[cfg(target_family = "wasm")]
    pub fn fetch_text(&self, url: &str, _referer: &str) -> Result<String, FetchError> {
        String::from_utf8(page_get(url)?).map_err(|error| FetchError::Request {
            url: url.to_owned(),
            message: error.to_string(),
        })
    }
}

/// A GET through the browser (mp_os::http), failing on a status that is not a success as ureq's
/// does here.
#[cfg(target_family = "wasm")]
fn page_get(url: &str) -> Result<Vec<u8>, FetchError> {
    let request = |message: String| FetchError::Request {
        url: url.to_owned(),
        message,
    };
    let (status, bytes) = mp_os::http("GET", url, None).map_err(request)?;
    if !(200..300).contains(&status) {
        return Err(request(format!("http status: {status}")));
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::OPENSTREETMAP;

    fn tile(z: u8) -> TileId {
        TileId::new(z, 0, 0).expect("a valid tile")
    }

    #[test]
    fn we_identify_ourselves_honestly() {
        // OSM's usage policy requires a User-Agent that identifies the application, and servers
        // block clients that send a generic one. Pretending to be a browser is both rude and,
        // when it gets the address blocked, self-defeating.
        assert!(
            USER_AGENT.starts_with("MissionPlannerRust/"),
            "{USER_AGENT}"
        );
        assert!(USER_AGENT.contains("github.com"), "{USER_AGENT}");
        assert!(
            !USER_AGENT.to_lowercase().contains("mozilla"),
            "{USER_AGENT}"
        );
    }

    #[test]
    fn a_zoom_the_provider_does_not_serve_fails_without_a_request() {
        // No point asking: the answer is an error page, and an error page that arrives with HTTP
        // 200 is the thing most likely to end up cached as a tile.
        let fetcher = TileFetcher::new();
        let error = fetcher
            .fetch(&OPENSTREETMAP, tile(OPENSTREETMAP.max_zoom + 1))
            .expect_err("should refuse");
        assert!(
            matches!(error, FetchError::UnsupportedZoom { .. }),
            "{error}"
        );
    }

    #[test]
    fn every_provider_has_a_zoom_it_will_refuse() {
        // A provider claiming the maximum the tile grid allows would never refuse anything, and
        // the refusal is what stops us caching error pages as tiles.
        //
        // Except where the C# refuses nothing either. Google's and Bing's providers set
        // `MaxZoom = null` (GoogleMapProvider.cs:24, BingMapProvider.cs:21), leaving the limit to
        // the map control's 24 (FlightPlanner.cs:188); the grid here stops at 22, so they get all
        // of it. A missing tile from them is a 404, which the policy backs off like any failure.
        //
        // Custom sets `MaxZoom = 22` (Custom.cs:22), the grid's top, and fetches nothing: its
        // tiles are what Inject Custom Map put in the cache.
        use crate::source::{
            BING_HYBRID_MAP, BING_MAP, BING_SATELLITE_MAP, CUSTOM, GOOGLE_MAP,
            GOOGLE_SATELLITE_MAP, GOOGLE_TERRAIN_MAP,
        };
        let unlimited_in_the_csharp = [
            &BING_MAP,
            &BING_SATELLITE_MAP,
            &BING_HYBRID_MAP,
            &GOOGLE_MAP,
            &GOOGLE_SATELLITE_MAP,
            &GOOGLE_TERRAIN_MAP,
            &CUSTOM,
        ];
        for source in crate::source::SOURCES {
            if unlimited_in_the_csharp.contains(&source) {
                assert_eq!(source.max_zoom, mp_units::tiles::MAX_ZOOM, "{}", source.id);
            } else {
                assert!(
                    source.max_zoom < mp_units::tiles::MAX_ZOOM,
                    "{} claims every zoom the grid allows",
                    source.id
                );
            }
        }
    }

    #[test]
    fn the_size_limit_is_far_above_a_real_tile_and_far_below_trouble() {
        // A 256x256 tile is tens of kilobytes; the limit exists for a hostile or broken server.
        // Checked at compile time, because both sides are constants and a runtime assertion on
        // constants is a test that cannot fail after it has compiled.
        const _: () = assert!(MAX_TILE_BYTES > 100 * 1024);
        const _: () = assert!(MAX_TILE_BYTES <= 8 * 1024 * 1024);
        const _: () = assert!(TIMEOUT.as_secs() > 0 && TIMEOUT.as_secs() <= 30);
    }

    #[test]
    #[ignore = "requires network access to a live tile server"]
    fn a_real_tile_can_be_fetched_and_is_an_image() {
        // Run with: cargo test -p mp-tiles -- --ignored
        // One tile, at zoom zero, which is the whole world and the cheapest thing to ask for.
        let fetcher = TileFetcher::new();
        let bytes = fetcher
            .fetch(&OPENSTREETMAP, tile(0))
            .expect("zoom 0 should be fetchable");
        assert!(bytes.len() > 1024, "{} bytes is too small", bytes.len());
        assert_eq!(
            crate::cache::ImageFormat::sniff(&bytes),
            Some(crate::cache::ImageFormat::Png)
        );
    }

    #[test]
    #[ignore = "requires network access"]
    fn a_host_that_does_not_exist_fails_as_a_request_error() {
        let fetcher = TileFetcher::new();
        let source = TileSource {
            id: "broken",
            cache_name: "Test",
            label: "Broken",
            url: "https://tiles.invalid.example/{z}/{x}/{y}.png",
            ..crate::source::OPENSTREETMAP
        };
        let error = fetcher.fetch(&source, tile(0)).expect_err("should fail");
        assert!(matches!(error, FetchError::Request { .. }), "{error}");
    }
}
