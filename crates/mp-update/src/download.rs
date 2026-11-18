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

//! Fetching an update: the manifest whole, and a release's download to a file as it comes, its
//! SHA-256 taken on the way and checked against the manifest's before anything reads the file.
//! `Update.cs`'s `GetNewFile` in its shape - the progress said once a second there, a percent
//! here, and Cancel seen between reads - with SHA-256 for its MD5. updates.md, section 4.6.
//! `// C#: Utilities/Update.cs:578-682`

use std::io::{Read, Write};
use std::path::Path;

use sha2::{Digest, Sha256};

use crate::manifest::{Asset, allowed_url};

/// What a cancelled update fails with. `// C#: Utilities/Update.cs:313`
pub const USER_REQUEST: &str = "User Request";
/// "Getting " and the file. `// C#: Strings.Getting`
pub const GETTING: &str = "Getting ";

/// A response's body, and its length where the server gave one.
pub struct Body {
    /// `Content-Length`.
    pub length: Option<u64>,
    /// The bytes.
    pub reader: Box<dyn Read + Send>,
}

/// The network, as the updater asks it.
pub trait Source: Send + Sync {
    /// A GET's body, for a file to be read as it comes.
    ///
    /// # Errors
    ///
    /// The request failed, or the status was not a success.
    fn open(&self, url: &str) -> Result<Body, String>;
}

/// A small file whole - the manifest - and no more than `limit` bytes of it.
///
/// # Errors
///
/// An address that may not be fetched, the request's failure, or a body past the limit.
pub fn get(source: &dyn Source, url: &str, limit: u64) -> Result<Vec<u8>, String> {
    if !allowed_url(url) {
        return Err(format!("not fetched, the address is not https: {url}"));
    }
    let mut bytes = Vec::new();
    source
        .open(url)?
        .reader
        .take(limit + 1)
        .read_to_end(&mut bytes)
        .map_err(|why| format!("{url}: {why}"))?;
    if bytes.len() as u64 > limit {
        return Err(format!("{url}: more than {limit} bytes"));
    }
    Ok(bytes)
}

/// The network itself: ureq, following GitHub's redirect to where a release's file is kept.
#[cfg(not(target_family = "wasm"))]
#[derive(Debug, Clone, Copy, Default)]
pub struct Http;

#[cfg(not(target_family = "wasm"))]
impl Source for Http {
    fn open(&self, url: &str) -> Result<Body, String> {
        use std::time::Duration;
        // No limit on the whole exchange: a release is tens of megabytes, and a slow link is
        // slow. The connection and the first answer are what must not hang.
        let response = ureq::get(url)
            .header("User-Agent", mp_firmware::manifest::USER_AGENT)
            .config()
            .timeout_connect(Some(Duration::from_secs(30)))
            .timeout_recv_response(Some(Duration::from_secs(30)))
            .build()
            .call()
            .map_err(|why| format!("{url}: {why}"))?;
        let length = response
            .headers()
            .get("content-length")
            .and_then(|value| value.to_str().ok())
            .and_then(|text| text.parse().ok());
        let reader = response
            .into_body()
            .into_with_config()
            .limit(MAX_ASSET)
            .reader();
        Ok(Body {
            length,
            reader: Box::new(reader),
        })
    }
}

/// The most a release's download may be: four times the largest so far.
pub const MAX_ASSET: u64 = 400 << 20;

/// A hash as the manifest writes it.
#[must_use]
pub fn hex(digest: &[u8]) -> String {
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// The SHA-256 of those bytes, as the manifest writes it.
#[must_use]
pub fn sha256_hex(bytes: &[u8]) -> String {
    hex(&Sha256::digest(bytes))
}

/// The download fetched to `target`, which is the manifest's file or nothing: its length and its
/// SHA-256 are the manifest's, or `target` is removed and the error says which was not.
/// `progress` is the dialog's `UpdateProgressAndStatus(percent, text)`; `cancel_requested` its
/// Cancel, seen between reads.
///
/// # Errors
///
/// An address that may not be fetched; the request's or the file's failure; [`USER_REQUEST`]
/// when cancelled; "File downloaded does not match hash: <file>".
/// `// C#: Utilities/Update.cs:311-345, 578-682`
pub fn fetch_file(
    source: &dyn Source,
    asset: &Asset,
    target: &Path,
    progress: &mut dyn FnMut(i32, &str),
    cancel_requested: &dyn Fn() -> bool,
) -> Result<(), String> {
    let outcome = fetch_unchecked(source, asset, target, progress, cancel_requested);
    if outcome.is_err() {
        let _ = mp_os::fs::remove_file(target);
    }
    outcome
}

fn fetch_unchecked(
    source: &dyn Source,
    asset: &Asset,
    target: &Path,
    progress: &mut dyn FnMut(i32, &str),
    cancel_requested: &dyn Fn() -> bool,
) -> Result<(), String> {
    if !allowed_url(&asset.url) {
        return Err(format!(
            "not fetched, the address is not https: {}",
            asset.url
        ));
    }
    progress(0, &format!("{GETTING}{}", asset.name));
    let mut body = source.open(&asset.url)?;
    let mut file =
        mp_os::fs::File::create(target).map_err(|why| format!("{}: {why}", target.display()))?;
    let mut hash = Sha256::new();
    let mut buffer = vec![0_u8; 64 * 1024];
    let mut done: u64 = 0;
    let mut said = 0;
    loop {
        if cancel_requested() {
            return Err(USER_REQUEST.to_owned());
        }
        let read = body
            .reader
            .read(&mut buffer)
            .map_err(|why| format!("{}: {why}", asset.url))?;
        if read == 0 {
            break;
        }
        let chunk = buffer.get(..read).unwrap_or_default();
        hash.update(chunk);
        file.write_all(chunk)
            .map_err(|why| format!("{}: {why}", target.display()))?;
        done += read as u64;
        // More than the manifest's size is not the manifest's file: stop before the disk fills.
        if done > asset.size {
            return Err(mismatch(asset));
        }
        let percent = i32::try_from(done.saturating_mul(100) / asset.size).unwrap_or(100);
        if percent != said {
            said = percent;
            let megabytes = |bytes: u64| bytes as f64 / f64::from(1 << 20);
            progress(
                percent,
                &format!(
                    "{GETTING}{}: {:.1} of {:.1} MB",
                    asset.name,
                    megabytes(done),
                    megabytes(asset.size)
                ),
            );
        }
    }
    file.sync_all()
        .map_err(|why| format!("{}: {why}", target.display()))?;
    drop(file);
    if done != asset.size || hex(&hash.finalize()) != asset.sha256 {
        return Err(mismatch(asset));
    }
    Ok(())
}

/// `// C#: Utilities/Update.cs:345`
fn mismatch(asset: &Asset) -> String {
    format!("File downloaded does not match hash: {}", asset.name)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use mp_os::Lock as _;
    use mp_os::fs::FsExt as _;
    use std::collections::BTreeMap;
    use std::sync::Mutex;

    /// A network of files held in memory, and the addresses asked of it.
    #[derive(Default)]
    pub(crate) struct Server {
        pub(crate) files: BTreeMap<String, Vec<u8>>,
        pub(crate) asked: Mutex<Vec<String>>,
    }

    impl Server {
        pub(crate) fn with(files: &[(&str, &[u8])]) -> Self {
            Self {
                files: files
                    .iter()
                    .map(|(url, bytes)| ((*url).to_owned(), bytes.to_vec()))
                    .collect(),
                asked: Mutex::new(Vec::new()),
            }
        }
    }

    impl Source for Server {
        fn open(&self, url: &str) -> Result<Body, String> {
            self.asked.os_lock().unwrap().push(url.to_owned());
            let bytes = self
                .files
                .get(url)
                .ok_or_else(|| format!("{url}: http status: 404"))?
                .clone();
            Ok(Body {
                length: Some(bytes.len() as u64),
                reader: Box::new(std::io::Cursor::new(bytes)),
            })
        }
    }

    pub(crate) fn scratch(test: &str) -> std::path::PathBuf {
        let dir = mp_os::temp_dir().join(format!("mp-update-{test}-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&dir);
        mp_os::fs::create_dir_all(&dir).unwrap();
        dir
    }

    pub(crate) fn asset_of(name: &str, url: &str, bytes: &[u8]) -> Asset {
        Asset {
            name: name.to_owned(),
            url: url.to_owned(),
            size: bytes.len() as u64,
            sha256: hex(&Sha256::digest(bytes)),
        }
    }

    #[test]
    fn a_hash_is_written_as_the_manifest_writes_it() {
        assert_eq!(
            hex(&Sha256::digest(b"abc")),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn a_download_is_the_manifests_file_or_nothing() {
        let dir = scratch("download");
        let bytes: Vec<u8> = (0..200_000_u32).map(|n| (n % 251) as u8).collect();
        let url = "https://x/planner.tar.gz";
        let server = Server::with(&[(url, &bytes)]);
        let target = dir.join("planner.tar.gz.part");
        let mut said = Vec::new();

        // The file, whole, with its progress to 100.
        let good = asset_of("planner.tar.gz", url, &bytes);
        fetch_file(
            &server,
            &good,
            &target,
            &mut |percent, text| said.push((percent, text.to_owned())),
            &|| false,
        )
        .unwrap();
        assert_eq!(mp_os::fs::read(&target).unwrap(), bytes);
        assert_eq!(
            said.first().unwrap(),
            &(0, "Getting planner.tar.gz".to_owned())
        );
        assert_eq!(said.last().unwrap().0, 100);
        assert!(
            said.last()
                .unwrap()
                .1
                .starts_with("Getting planner.tar.gz: 0.2 of 0.2 MB")
        );

        // Another hash, a shorter file and a longer one: refused, and nothing left behind.
        let mut wrong = good.clone();
        wrong.sha256 = hex(&Sha256::digest(b"something else"));
        for (bad, why) in [
            (wrong, "another hash"),
            (
                Asset {
                    size: good.size + 1,
                    ..good.clone()
                },
                "shorter than said",
            ),
            (
                Asset {
                    size: good.size - 1,
                    ..good.clone()
                },
                "longer than said",
            ),
        ] {
            let error = fetch_file(&server, &bad, &target, &mut |_, _| {}, &|| false).unwrap_err();
            assert_eq!(
                error, "File downloaded does not match hash: planner.tar.gz",
                "{why}"
            );
            assert!(!target.os_exists(), "{why}");
        }

        // Cancel, a file the server has not got, and an address that is not https.
        let error = fetch_file(&server, &good, &target, &mut |_, _| {}, &|| true).unwrap_err();
        assert_eq!(error, USER_REQUEST);
        assert!(!target.os_exists());
        let missing = asset_of("planner.tar.gz", "https://x/none", &bytes);
        assert!(
            fetch_file(&server, &missing, &target, &mut |_, _| {}, &|| false)
                .unwrap_err()
                .contains("404")
        );
        let plain = asset_of(
            "planner.tar.gz",
            "http://example.com/planner.tar.gz",
            &bytes,
        );
        assert!(
            fetch_file(&server, &plain, &target, &mut |_, _| {}, &|| false)
                .unwrap_err()
                .starts_with("not fetched, the address is not https")
        );
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_manifest_is_fetched_whole_and_no_larger_than_allowed() {
        let server = Server::with(&[("https://x/update.json", b"{\"schema\": 1}")]);
        assert_eq!(
            get(&server, "https://x/update.json", 64).unwrap(),
            b"{\"schema\": 1}"
        );
        assert_eq!(
            get(&server, "https://x/update.json", 5).unwrap_err(),
            "https://x/update.json: more than 5 bytes"
        );
        assert!(
            get(&server, "http://example.com/update.json", 64)
                .unwrap_err()
                .contains("not https")
        );
    }
}
