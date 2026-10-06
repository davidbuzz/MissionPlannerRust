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

//! What goes over the wire for a Bing map, on the path the map takes, with nothing leaving the
//! machine.
//!
//! The store is given a fetcher that sends everything through a proxy, and the proxy is this test:
//! it accepts the `CONNECT` ureq opens for each host, answers the request that comes down the
//! tunnel as Bing would, and writes down what it was asked. So the fetch thread, the version check
//! it runs before its first tile, the URL cache, the tile cache and the headers are all the
//! product's, and only the far end is not Microsoft. Bing rather than Google because its URLs are
//! plain `http`; Google's are `https`, and a tunnel carrying TLS would need a certificate the
//! fetcher trusts.
//!
//! A process of its own, because the version check changes what every Bing URL carries for the
//! rest of the process.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use mp_os::Lock as _;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use mp_tiles::cache::TileCache;
use mp_tiles::fetch::{TileFetcher, USER_AGENT};
use mp_tiles::source::{BING_MAP, BING_SATELLITE_MAP};
use mp_tiles::store::{TileAnswer, TileStore};
use mp_tiles::urlcache;
use mp_tiles::versions::BING_PAGE;
use mp_units::TileId;

/// One request as the proxy saw it: the host the tunnel was opened to, the request line, and the
/// headers with their names in lower case.
#[derive(Debug, Clone)]
struct Seen {
    tunnel: String,
    line: String,
    headers: Vec<(String, String)>,
}

impl Seen {
    fn header(&self, name: &str) -> Vec<&str> {
        self.headers
            .iter()
            .filter(|(key, _)| key == name)
            .map(|(_, value)| value.as_str())
            .collect()
    }
}

/// Reads a request head, up to the blank line.
fn read_head(reader: &mut BufReader<TcpStream>) -> Option<(String, Vec<(String, String)>)> {
    let mut line = String::new();
    reader.read_line(&mut line).ok()?;
    let first = line.trim_end().to_owned();
    if first.is_empty() {
        return None;
    }
    let mut headers = Vec::new();
    loop {
        let mut header = String::new();
        reader.read_line(&mut header).ok()?;
        let header = header.trim_end();
        if header.is_empty() {
            return Some((first, headers));
        }
        let (name, value) = header.split_once(':')?;
        headers.push((name.trim().to_lowercase(), value.trim().to_owned()));
    }
}

/// A 256x256 PNG, which is what a tile server sends.
fn png() -> Vec<u8> {
    let mut buffer = std::io::Cursor::new(Vec::new());
    image::RgbaImage::from_pixel(256, 256, image::Rgba([0x30, 0x60, 0x90, 255]))
        .write_to(&mut buffer, image::ImageFormat::Png)
        .unwrap();
    buffer.into_inner()
}

/// One proxied connection: the `CONNECT`, then one request down the tunnel.
fn serve(stream: TcpStream, seen: &Mutex<Vec<Seen>>) {
    let mut writer = stream.try_clone().unwrap();
    let mut reader = BufReader::new(stream);
    let Some((connect, _)) = read_head(&mut reader) else {
        return;
    };
    let tunnel = connect
        .strip_prefix("CONNECT ")
        .and_then(|rest| rest.split_whitespace().next())
        .unwrap_or_default()
        .to_owned();
    writer
        .write_all(b"HTTP/1.1 200 Connection established\r\n\r\n")
        .unwrap();
    let Some((line, headers)) = read_head(&mut reader) else {
        return;
    };
    seen.os_lock().unwrap().push(Seen {
        tunnel: tunnel.clone(),
        line: line.clone(),
        headers,
    });
    let (content_type, body) = if tunnel == "www.bing.com:80" {
        (
            "text/html",
            b"<script>var cfg={tileGeneration:15512,mkt:'en-us'};</script>".to_vec(),
        )
    } else {
        ("image/png", png())
    };
    let head = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    );
    let _ = writer.write_all(head.as_bytes());
    let _ = writer.write_all(&body);
}

/// Waits until the store has a tile, bounded so a broken fetch is a failure and not a hang.
fn wait_for(store: &TileStore, tile: TileId) -> TileAnswer {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let answer = store.get(tile);
        if matches!(answer, TileAnswer::Exact(_)) || Instant::now() > deadline {
            return answer;
        }
        wasm_thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn a_bing_map_checks_its_version_then_asks_for_tiles_as_the_csharp_does() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let proxy = format!("http://{}", listener.local_addr().unwrap());
    let seen = Arc::new(Mutex::new(Vec::<Seen>::new()));
    {
        let seen = Arc::clone(&seen);
        wasm_thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let seen = Arc::clone(&seen);
                wasm_thread::spawn(move || serve(stream, &seen));
            }
        });
    }

    let root = mp_os::temp_dir().join(format!("mp-tiles-wire-{}", mp_os::process_id()));
    let _ = std::fs::remove_dir_all(&root);
    let _cleanup = Cleanup(root.clone());
    let tile = TileId::new(2, 3, 1).unwrap();

    let store = TileStore::with_fetcher(
        &BING_SATELLITE_MAP,
        TileCache::new(&root),
        TileFetcher::through_proxy(&proxy).unwrap(),
    );
    let answer = wait_for(&store, tile);
    assert!(
        matches!(answer, TileAnswer::Exact(_)),
        "{answer:?}, {:?}, seen {:?}",
        store.stats(),
        seen.os_lock().unwrap()
    );
    assert_eq!(store.stats().fetched, 1);

    let requests = seen.os_lock().unwrap().clone();
    assert_eq!(requests.len(), 2, "{requests:#?}");

    // First Bing's maps page, for the version: BingMapProvider.cs:158, before any tile.
    let page = &requests[0];
    assert_eq!(page.tunnel, "www.bing.com:80");
    assert_eq!(page.line, "GET /maps HTTP/1.1");
    // Then the tile, carrying the version the page named: BingSatelliteMapProvider.cs:59, with
    // (3 + 2 * 1) % 4 = 1 for the server and "13" for the quadkey.
    let tile_request = &requests[1];
    assert_eq!(tile_request.tunnel, "ecn.t1.tiles.virtualearth.net:80");
    assert_eq!(
        tile_request.line,
        "GET /tiles/a13.jpeg?g=15512&mkt=en&n=z HTTP/1.1"
    );
    // Both with the C#'s headers: its Accept, the provider's Referer (BingMapProvider.cs:22), and
    // a User-Agent that names the application (Program.cs:375-377).
    for request in &requests {
        assert_eq!(request.header("accept"), vec!["*/*"], "{request:?}");
        assert_eq!(
            request.header("referer"),
            vec!["http://www.bing.com/maps/"],
            "{request:?}"
        );
        assert_eq!(
            request.header("user-agent"),
            vec![USER_AGENT],
            "{request:?}"
        );
    }

    // The page is in the URL cache and the tile in the tile cache, where the C# keeps them.
    assert_eq!(
        std::fs::read_to_string(urlcache::path(&root, BING_PAGE)).unwrap(),
        "\u{feff}<script>var cfg={tileGeneration:15512,mkt:'en-us'};</script>"
    );
    assert!(
        root.join("TileDBv3/en/BingSatelliteMap/2/1/3.jpg")
            .is_file()
    );
    drop(store);

    // Another Bing provider in the same run does not check again - the C#'s `init` is shared by
    // the family - and asks with the version already found.
    let store = TileStore::with_fetcher(
        &BING_MAP,
        TileCache::new(&root),
        TileFetcher::through_proxy(&proxy).unwrap(),
    );
    assert!(matches!(wait_for(&store, tile), TileAnswer::Exact(_)));
    let requests = seen.os_lock().unwrap().clone();
    assert_eq!(requests.len(), 3, "{requests:#?}");
    assert_eq!(
        requests[2].line,
        "GET /tiles/r13?g=15512&mkt=en&lbl=l1&stl=h&shading=hill&n=z HTTP/1.1"
    );
}

/// Removes a directory when dropped.
struct Cleanup(PathBuf);

impl Drop for Cleanup {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
