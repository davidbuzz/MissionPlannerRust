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

//! The download queue: `requestRunner`, `get3secfile`, `getListing` and `gethgt`.
//!
//! One tile at a time, at most one a second. A tile is looked for in the 1-arc-second listing
//! first and then in each 3-arc-second region's, every listing cached on disk for a week; the
//! first listed URL that contains the tile's name is fetched as `<tile>.zip` and unzipped beside
//! it. A tile in no listing, once every listing has been read, is ocean.

use std::sync::Arc;
use std::sync::OnceLock;
use std::sync::atomic::Ordering;
// A file's time is std's: this file compares the clock only with files' times. port_clock: keep
use std::time::SystemTime; // port_clock: keep
use web_time::Duration;

use crate::srtm::{Fault, Inner, decode_utf8, lock, read_lines};

/// How long the queue thread waits for something to be queued before looking anyway.
/// `// C#: ExtLibs/Utilities/srtm.cs:536`
pub(crate) const REQUEST_WAIT: Duration = Duration::from_secs(30);

/// The pause after every pass: "never more than 1/s".
/// `// C#: ExtLibs/Utilities/srtm.cs:568-571`
pub(crate) const REQUEST_PAUSE: Duration = Duration::from_secs(1);

/// How long a cached listing is trusted.
/// `// C#: ExtLibs/Utilities/srtm.cs:692`
const LISTING_MAX_AGE: Duration = Duration::from_secs(7 * 24 * 60 * 60);

/// Fewer listings than this and a tile found in none of them is not taken for ocean.
/// `// C#: ExtLibs/Utilities/srtm.cs:621`
const OCEAN_MIN_LISTINGS: usize = 9;

/// Names that must have been looked through, and not matched, before a tile is ocean: "38581 is
/// all srtm3 and srtm1".
/// `// C#: ExtLibs/Utilities/srtm.cs:620-621`
const OCEAN_MIN_NAMES: usize = 38000;

/// `HttpClient.Timeout`'s default: 100 seconds for a whole response, body included, because
/// `GetAsync` reads the body before it returns.
const HTTP_TIMEOUT: Duration = Duration::from_secs(100);

/// `HttpClient.MaxResponseContentBufferSize`'s default, `int.MaxValue`.
const HTTP_MAX_BODY: u64 = 2_147_483_647;

/// `Environment.NewLine`, which `StreamWriter.WriteLine` ends a listing's lines with.
const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// A GET that went wrong on the way: no connection, a reset, a timeout.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HttpError(pub String);

impl std::fmt::Display for HttpError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for HttpError {}

impl From<HttpError> for Fault {
    fn from(_: HttpError) -> Self {
        Self
    }
}

/// `HttpClient.GetAsync(url)` and its body: what the queue thread fetches with.
///
/// Whatever the status, the body is the answer. `GetAsync` does not throw on a 404, so the C#
/// parses an error page for links like any other page, and writes one to `<tile>.zip` - which then
/// fails to unzip. Only a failure to get any response at all is an error.
pub trait Http: Send + Sync {
    /// The body `url` answers with.
    ///
    /// # Errors
    ///
    /// No response: the connection failed, was reset, or timed out.
    fn get(&self, url: &str) -> Result<Vec<u8>, HttpError>;

    /// The status and the body `url` answers with, for a caller that tells a 404 from a body:
    /// `get` with 200, unless the client knows better.
    ///
    /// # Errors
    ///
    /// As `get`.
    fn get_status(&self, url: &str) -> Result<(u16, Vec<u8>), HttpError> {
        self.get(url).map(|body| (200, body))
    }
}

/// The real client: `ureq`, blocking, on the queue thread.
#[derive(Debug, Clone)]
pub struct UreqHttp {
    agent: ureq::Agent,
}

impl Default for UreqHttp {
    fn default() -> Self {
        Self::new()
    }
}

impl UreqHttp {
    /// A client as `srtm`'s `HttpClient` is set up: its User-Agent (srtm.cs:98-99), .NET's
    /// 100-second timeout, and every status an answer. Through the environment's proxy, if any.
    #[must_use]
    pub fn new() -> Self {
        let config = ureq::Agent::config_builder()
            .timeout_global(Some(HTTP_TIMEOUT))
            .user_agent(crate::USER_AGENT)
            .http_status_as_error(false)
            .build();
        Self {
            agent: config.into(),
        }
    }
}

impl Http for UreqHttp {
    fn get(&self, url: &str) -> Result<Vec<u8>, HttpError> {
        self.get_status(url).map(|(_, body)| body)
    }

    fn get_status(&self, url: &str) -> Result<(u16, Vec<u8>), HttpError> {
        let mut response = self
            .agent
            .get(url)
            .call()
            .map_err(|error| HttpError(format!("{url}: {error}")))?;
        let status = response.status().as_u16();
        response
            .body_mut()
            .with_config()
            .limit(HTTP_MAX_BODY)
            .read_to_vec()
            .map(|body| (status, body))
            .map_err(|error| HttpError(format!("{url}: {error}")))
    }
}

/// `requestRunner`: the queue thread, until the [`crate::Srtm`] is dropped.
/// `// C#: ExtLibs/Utilities/srtm.cs:526-578`
pub(crate) fn request_runner(inner: &Arc<Inner>) {
    while inner.run.load(Ordering::Acquire) {
        request_step(inner);
        // C#: await Task.Delay(1000) - "never more than 1/s".
        wasm_thread::sleep(REQUEST_PAUSE);
    }
}

/// One pass of `requestRunner`'s loop, without the pause.
/// `// C#: ExtLibs/Utilities/srtm.cs:534-566`
pub(crate) fn request_step(inner: &Inner) {
    // C#: await requestSemaphore.WaitAsync(30000) - and whether it timed out is not looked at.
    inner.semaphore.wait(REQUEST_WAIT, &inner.stopping);
    if inner.stopping.load(Ordering::Acquire) {
        return;
    }

    let item = lock(&inner.queue).first().cloned();
    let Some(item) = item else {
        return;
    };
    // C#: an exception from get3secfile lands in the catch at srtm.cs:563-566 and skips the
    // removal below: the tile stays at the head of the queue and is tried again on the next pass,
    // and every tile behind it waits.
    if get3secfile(inner, &item).is_err() {
        return;
    }
    let mut queue = lock(&inner.queue);
    if let Some(at) = queue.iter().position(|tile| *tile == item) {
        queue.remove(at);
    }
    // continue without delay
    if !queue.is_empty() {
        inner.semaphore.release();
    }
}

/// `get3secfile`: find the tile in the listings and fetch it, or decide it is ocean.
/// `// C#: ExtLibs/Utilities/srtm.cs:580-626`
fn get3secfile(inner: &Inner, name: &str) -> Result<(), Fault> {
    // check file doesnt already exist
    let path = inner.datadirectory.join(name);
    if path.is_file() && std::fs::metadata(&path)?.len() != 0 {
        return Ok(());
    }

    let servers = lock(&inner.servers).clone();
    let mut checkednames: usize = 0;
    // load 1 arc seconds first, then 3 arc second
    let mut list = vec![servers.baseurl1sec];
    list.extend(get_listing(inner, &servers.baseurl)?);

    for item in &list {
        for hgt in get_listing(inner, item)? {
            checkednames += 1;
            if hgt.contains(name) {
                gethgt(inner, &hgt, name);
                return Ok(());
            }
        }
    }

    // if there are no http exceptions, and the list is >= 9, then everything above is valid
    if list.len() >= OCEAN_MIN_LISTINGS && checkednames > OCEAN_MIN_NAMES {
        let mut ocean = lock(&inner.oceantile);
        if !ocean.iter().any(|tile| tile == name) {
            // we must be an ocean tile - no matchs
            ocean.push(name.to_owned());
        }
    }
    Ok(())
}

/// `gethgt`: fetch `url` to `<datadirectory>/<filename>.zip` and unzip it into `datadirectory`.
/// Every failure is logged and swallowed; the tile is then simply not there, and the next lookup
/// queues it again. The `.zip` is left beside the tile, as the C# leaves it.
/// `// C#: ExtLibs/Utilities/srtm.cs:634-676`
fn gethgt(inner: &Inner, url: &str, filename: &str) {
    let fetch = || -> Result<(), Fault> {
        let body = inner.http.get(url)?;
        let zip = inner.datadirectory.join(format!("{filename}.zip"));
        std::fs::write(&zip, &body)?;
        // C#: lock(extract) fzip.ExtractZip(zip, datadirectory, "") - every entry, overwriting.
        let _extracting = lock(&inner.extract);
        mp_log::zip::extract(&body, &inner.datadirectory).map_err(|_| Fault)?;
        Ok(())
    };
    // C#: catch (Exception ex) { log.Error(ex); }
    let _ = fetch();
}

/// `getListing`: the links on a directory page, as absolute URLs, cached in
/// `<datadirectory>/<last path segment>` for a week.
///
/// A link containing `..` or `http`, or ending `/srtm/version2_1/`, is skipped. An empty cached
/// listing is not a cached one, so a server that lists nothing is asked again every time. Any
/// failure is thrown on, to `get3secfile` and from there to the queue thread.
/// `// C#: ExtLibs/Utilities/srtm.cs:678-755`
fn get_listing(inner: &Inner, url: &str) -> Result<Vec<String>, Fault> {
    if url.ends_with("bios") {
        return Ok(Vec::new());
    }

    let name = listing_name(url)?;
    let path = inner.datadirectory.join(&name);

    if path.is_file() {
        let meta = std::fs::metadata(&path)?;
        // C#: fi.LastWriteTime.AddDays(7) > DateTime.Now
        let fresh = meta
            .modified()
            .ok()
            .and_then(|modified| modified.checked_add(LISTING_MAX_AGE))
            .is_some_and(|expires| expires > SystemTime::now());
        if meta.len() > 0 && fresh {
            let text = decode_utf8(&std::fs::read(&path)?);
            return Ok(read_lines(&text).into_iter().map(str::to_owned).collect());
        }
    }

    let body = inner.http.get(url)?;
    let data = decode_utf8(&body);
    let base = url.trim_end_matches(['/', '\\']);
    let mut list = Vec::new();
    for link in href().captures_iter(&data) {
        let Some(link) = link.get(1).map(|m| m.as_str()) else {
            continue;
        };
        if link.contains("..") || link.contains("http") || link.ends_with("/srtm/version2_1/") {
            continue;
        }
        list.push(format!("{base}/{link}"));
    }

    let mut text = String::new();
    for line in &list {
        text.push_str(line);
        text.push_str(NEWLINE);
    }
    if name == "README.txt" || name == "Region_definition.jpg" {
        text.push(' ');
    }
    std::fs::write(&path, text)?;

    Ok(list)
}

/// `href="([^"]+)"`, case-insensitive.
/// `// C#: ExtLibs/Utilities/srtm.cs:717`
fn href() -> &'static regex::Regex {
    static HREF: OnceLock<regex::Regex> = OnceLock::new();
    HREF.get_or_init(|| {
        // A constant pattern: it compiles, or no test passes.
        regex::Regex::new(r#"(?i)href="([^"]+)""#).unwrap_or_else(|_| unreachable!())
    })
}

/// `Path.GetFileName(new Uri(url).AbsolutePath.TrimEnd('/'))`: the last segment of the URL's path,
/// which names the listing's cache file - `SRTM1`, `Africa`.
///
/// A URL with no scheme is an error, as `new Uri` throws on one. The C# takes the path escaped
/// (`%20` for a space) and this takes it as written; the listing URLs are plain.
fn listing_name(url: &str) -> Result<String, Fault> {
    let after_scheme = url.split_once("://").map(|(_, rest)| rest).ok_or(Fault)?;
    let path = after_scheme
        .find('/')
        .and_then(|at| after_scheme.get(at..))
        .unwrap_or("/");
    let path = path.split(['?', '#']).next().unwrap_or_default();
    let path = path.trim_end_matches('/');
    let separators: &[char] = if cfg!(windows) { &['/', '\\'] } else { &['/'] };
    Ok(path
        .rsplit(separators)
        .next()
        .unwrap_or_default()
        .to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_listing_is_named_for_the_last_segment_of_its_path() {
        assert_eq!(listing_name(crate::BASEURL1SEC).unwrap(), "SRTM1");
        assert_eq!(listing_name(crate::BASEURL).unwrap(), "SRTM3");
        assert_eq!(
            listing_name("https://terrain.ardupilot.org/SRTM3/Africa/").unwrap(),
            "Africa"
        );
        assert_eq!(
            listing_name("https://terrain.ardupilot.org/SRTM3/filelist_python").unwrap(),
            "filelist_python"
        );
        assert_eq!(listing_name("http://host:8080/a/b/?q=1").unwrap(), "b");
        assert_eq!(listing_name("http://host").unwrap(), "");
        assert!(listing_name("SRTM3/").is_err());
    }

    #[test]
    fn the_href_pattern_is_the_csharps() {
        let page = "<a href=\"a/\">x</a><A HREF=\"B.zip\"></A><a href=''>\
                    <a href=\"\">e</a><a href=\"c\nd\">";
        let links: Vec<&str> = href()
            .captures_iter(page)
            .filter_map(|c| c.get(1).map(|m| m.as_str()))
            .collect();
        assert_eq!(links, vec!["a/", "B.zip", "c\nd"]);
    }
}
