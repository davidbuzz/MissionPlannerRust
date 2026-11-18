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

//! A release's manifest, `update.json`: one small file the release workflow writes beside the
//! downloads, saying which release it is and, for each platform, the file to fetch, its size and
//! its SHA-256 (`tools/release/manifest.py` writes it; this reads it). Stable's is at
//! `releases/latest/download/update.json`, which GitHub redirects to the newest full release;
//! beta's at `releases/download/beta/update.json`, the one rolling pre-release. Not in the C#,
//! which reads `version.txt` and `checksums.txt`: updates.md, sections 4.1, 4.2 and 4.4.

use std::collections::BTreeMap;

use crate::identity::{Channel, Identity};
use crate::release::ReleaseVersion;

/// The manifest's form, `schema`: a manifest of another is refused, not guessed at.
pub const SCHEMA: u64 = 1;
/// The repository whose releases these are. `Cargo.toml`'s `repository`.
pub const REPOSITORY: &str = env!("CARGO_PKG_REPOSITORY");
/// The manifest's file name among a release's downloads.
pub const FILE: &str = "update.json";
/// The settings that name another manifest than the repository's: a fork's, or a test's.
pub const STABLE_KEY: &str = "UpdateManifest";
/// The beta channel's.
pub const BETA_KEY: &str = "BetaUpdateManifest";
/// The Debian package's place in `assets`.
pub const DEB: &str = "linux-x86_64-deb";

/// One download of a release.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Asset {
    /// Its file name, `missionplanner-rust_0_0_2_linux-x86_64.tar.gz`.
    pub name: String,
    /// Where it is fetched from.
    pub url: String,
    /// Its length in bytes.
    pub size: u64,
    /// Its SHA-256, sixty-four lower-case hexadecimal digits.
    pub sha256: String,
}

/// A release, as its manifest describes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Manifest {
    /// Its channel.
    pub channel: Channel,
    /// Its version; for the beta, that of the last release tag before its commit.
    pub version: ReleaseVersion,
    /// The tag its downloads are under: `v0_0_2`, or `beta`.
    pub tag: String,
    /// The commit it was built from.
    pub commit: String,
    /// When it was built, `2026-11-20T03:10:00Z`.
    pub built: String,
    /// The release's page, where "ChangeLog" goes.
    pub notes_url: String,
    /// Its downloads by platform: `linux-x86_64`, `linux-x86_64-deb`, `windows-x86_64`,
    /// `macos-universal`.
    pub assets: BTreeMap<String, Asset>,
}

/// Whether an address may be fetched: https, or - for the tests' server - plain http on this
/// machine alone.
#[must_use]
pub fn allowed_url(url: &str) -> bool {
    url.starts_with("https://")
        || url.starts_with("http://127.0.0.1:")
        || url.starts_with("http://127.0.0.1/")
        || url.starts_with("http://localhost:")
}

/// Where a channel's manifest is: the setting's address when one is set, else the repository's.
#[must_use]
pub fn manifest_url(channel: Channel, setting: &dyn Fn(&str) -> Option<String>) -> String {
    let (key, path) = match channel {
        Channel::Stable => (STABLE_KEY, "releases/latest/download"),
        Channel::Beta => (BETA_KEY, "releases/download/beta"),
    };
    match setting(key).filter(|url| !url.trim().is_empty()) {
        Some(url) => url.trim().to_owned(),
        None => format!("{REPOSITORY}/{path}/{FILE}"),
    }
}

/// This build's platform, as `assets` names it: the archive a person would download for it.
#[must_use]
pub const fn platform() -> Option<&'static str> {
    if cfg!(all(target_os = "linux", target_arch = "x86_64")) {
        Some("linux-x86_64")
    } else if cfg!(all(target_os = "windows", target_arch = "x86_64")) {
        Some("windows-x86_64")
    } else if cfg!(target_os = "macos") {
        Some("macos-universal")
    } else {
        None
    }
}

impl Manifest {
    /// The manifest those bytes are.
    ///
    /// # Errors
    ///
    /// What is wrong with it, by the field: not JSON, another schema, a field missing or of the
    /// wrong kind, an address that is neither https nor this machine, a file name with a path in
    /// it, a hash that is not sixty-four hexadecimal digits.
    pub fn parse(bytes: &[u8]) -> Result<Self, String> {
        let value: serde_json::Value =
            serde_json::from_slice(bytes).map_err(|why| format!("the update manifest is not JSON: {why}"))?;
        let text = |name: &str| -> Result<String, String> {
            value
                .get(name)
                .and_then(serde_json::Value::as_str)
                .filter(|text| !text.is_empty())
                .map(str::to_owned)
                .ok_or_else(|| format!("the update manifest has no {name}"))
        };
        match value.get("schema").and_then(serde_json::Value::as_u64) {
            Some(SCHEMA) => {}
            Some(other) => {
                return Err(format!(
                    "the update manifest is of schema {other}, which this planner does not read"
                ));
            }
            None => return Err("the update manifest has no schema".to_owned()),
        }
        let channel = text("channel")?;
        let channel = Channel::parse(&channel)
            .ok_or_else(|| format!("the update manifest names no channel this planner has: {channel:?}"))?;
        let notes_url = text("notes_url")?;
        if !allowed_url(&notes_url) {
            return Err(format!("the update manifest's notes_url is not https: {notes_url}"));
        }
        let listed = value
            .get("assets")
            .and_then(serde_json::Value::as_object)
            .ok_or_else(|| "the update manifest has no assets".to_owned())?;
        let mut assets = BTreeMap::new();
        for (platform, entry) in listed {
            assets.insert(platform.clone(), asset(platform, entry)?);
        }
        Ok(Self {
            channel,
            version: ReleaseVersion::parse(&text("version")?)?,
            tag: text("tag")?,
            commit: text("commit")?,
            built: text("built")?,
            notes_url,
            assets,
        })
    }

    /// The download for a platform.
    ///
    /// # Errors
    ///
    /// The release has none for it.
    pub fn asset(&self, platform: &str) -> Result<&Asset, String> {
        self.assets
            .get(platform)
            .ok_or_else(|| format!("release {} has no download for {platform}", self.tag))
    }

    /// Whether this release is one to offer a build, asked on the manifest's own channel.
    ///
    /// Stable is offered when its version is above the build's, whatever the build's channel - a
    /// beta carries the version of the last release before it, so the next release is above it.
    /// Beta is offered when it is another commit than the build's and was built later, so a
    /// release newer than the beta is never taken back to it.
    #[must_use]
    pub fn offers(&self, build: &Identity) -> bool {
        match self.channel {
            Channel::Stable => self.version > build.version,
            Channel::Beta => self.commit != build.commit && self.built > build.built,
        }
    }

    /// What the question says of it after "Update Found": the versions for a release, the day
    /// and the commit for a beta.
    #[must_use]
    pub fn described_for(&self, build: &Identity) -> String {
        match self.channel {
            Channel::Stable => format!("{} to {}", build.version, self.version),
            Channel::Beta => format!(
                "{} built {} to {} built {}",
                build.short_commit(),
                day(&build.built),
                crate::identity::short(&self.commit),
                day(&self.built)
            ),
        }
    }
}

/// The day of a build time, `2026-11-20`; "unknown" for a build that did not say.
fn day(built: &str) -> &str {
    if built.is_empty() {
        return "unknown";
    }
    built.get(..10).unwrap_or(built)
}

/// One entry of `assets`.
fn asset(platform: &str, entry: &serde_json::Value) -> Result<Asset, String> {
    let text = |name: &str| -> Result<String, String> {
        entry
            .get(name)
            .and_then(serde_json::Value::as_str)
            .filter(|text| !text.is_empty())
            .map(str::to_owned)
            .ok_or_else(|| format!("the update manifest's {platform} has no {name}"))
    };
    let name = text("name")?;
    if name.contains(['/', '\\']) || name.starts_with('.') {
        return Err(format!("the update manifest's {platform} names a path, not a file: {name}"));
    }
    let url = text("url")?;
    if !allowed_url(&url) {
        return Err(format!("the update manifest's {platform} is not fetched over https: {url}"));
    }
    let size = entry
        .get("size")
        .and_then(serde_json::Value::as_u64)
        .filter(|size| *size > 0)
        .ok_or_else(|| format!("the update manifest's {platform} has no size"))?;
    let sha256 = text("sha256")?;
    if sha256.len() != 64 || !sha256.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f')) {
        return Err(format!("the update manifest's {platform} has no SHA-256: {sha256}"));
    }
    Ok(Asset {
        name,
        url,
        size,
        sha256,
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) const COMMIT: &str = "42992d086a789143682f2997adc737545e3d8c2e";
    pub(crate) const OTHER: &str = "0570c61aa1b2c3d4e5f60718293a4b5c6d7e8f90";
    const HASH: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

    pub(crate) fn text(channel: &str, version: &str, commit: &str, built: &str) -> String {
        format!(
            r#"{{
  "schema": 1,
  "channel": "{channel}",
  "version": "{version}",
  "tag": "v0_0_2",
  "commit": "{commit}",
  "built": "{built}",
  "notes_url": "https://github.com/davidbuzz/MissionPlannerRust/releases/tag/v0_0_2",
  "assets": {{
    "linux-x86_64": {{
      "name": "missionplanner-rust_0_0_2_linux-x86_64.tar.gz",
      "url": "https://github.com/davidbuzz/MissionPlannerRust/releases/download/v0_0_2/missionplanner-rust_0_0_2_linux-x86_64.tar.gz",
      "size": 58623651,
      "sha256": "{HASH}"
    }},
    "linux-x86_64-deb": {{
      "name": "missionplanner-rust_0.0.2_amd64.deb",
      "url": "https://github.com/davidbuzz/MissionPlannerRust/releases/download/v0_0_2/missionplanner-rust_0.0.2_amd64.deb",
      "size": 44743122,
      "sha256": "{HASH}"
    }}
  }}
}}"#
        )
    }

    fn build(version: &str, channel: &str, commit: &str, built: &str) -> Identity {
        Identity::from_values(Some(version), Some(commit), Some(channel), Some(built)).unwrap()
    }

    #[test]
    fn a_manifest_reads_field_by_field() {
        let manifest =
            Manifest::parse(text("stable", "0.0.2", COMMIT, "2026-11-20T03:10:00Z").as_bytes()).unwrap();
        assert_eq!(manifest.channel, Channel::Stable);
        assert_eq!(manifest.version.to_string(), "0.0.2");
        assert_eq!(manifest.tag, "v0_0_2");
        assert_eq!(manifest.commit, COMMIT);
        let linux = manifest.asset("linux-x86_64").unwrap();
        assert_eq!(linux.name, "missionplanner-rust_0_0_2_linux-x86_64.tar.gz");
        assert_eq!(linux.size, 58_623_651);
        assert_eq!(linux.sha256, HASH);
        assert_eq!(manifest.asset(DEB).unwrap().name, "missionplanner-rust_0.0.2_amd64.deb");
        assert_eq!(
            manifest.asset("windows-x86_64").unwrap_err(),
            "release v0_0_2 has no download for windows-x86_64"
        );
    }

    #[test]
    fn what_is_wrong_with_a_manifest_is_said_by_the_field() {
        let good = text("stable", "0.0.2", COMMIT, "2026-11-20T03:10:00Z");
        let broken = |from: &str, to: &str| Manifest::parse(good.replacen(from, to, 1).as_bytes()).unwrap_err();
        assert!(Manifest::parse(b"<html>").unwrap_err().starts_with("the update manifest is not JSON"));
        assert_eq!(
            broken("\"schema\": 1", "\"schema\": 2"),
            "the update manifest is of schema 2, which this planner does not read"
        );
        assert_eq!(broken("\"schema\": 1,", ""), "the update manifest has no schema");
        assert!(broken("\"stable\"", "\"master\"").contains("no channel this planner has"));
        assert!(broken("\"0.0.2\"", "\"next\"").starts_with("not a release version"));
        assert_eq!(broken("\"commit\"", "\"comit\""), "the update manifest has no commit");
        // Never over plain http from another machine, and never to a path.
        assert!(
            broken("\"url\": \"https://github.com", "\"url\": \"http://example.com")
                .contains("is not fetched over https")
        );
        assert!(
            broken("\"name\": \"missionplanner", "\"name\": \"../missionplanner")
                .contains("names a path, not a file")
        );
        assert!(broken("58623651", "0").ends_with("has no size"));
        assert!(broken(HASH, "BA7816BF").contains("has no SHA-256"));
        assert!(allowed_url("http://127.0.0.1:5793/update.json"));
        assert!(!allowed_url("http://127.0.0.1.example.com/update.json"));
        assert!(!allowed_url("ftp://github.com/update.json"));
    }

    #[test]
    fn stable_is_offered_by_its_version_and_beta_by_its_commit_and_day() {
        let stable =
            Manifest::parse(text("stable", "0.0.2", COMMIT, "2026-11-20T03:10:00Z").as_bytes()).unwrap();
        assert!(stable.offers(&build("0.0.1", "stable", OTHER, "2026-10-06T11:00:00Z")));
        assert!(!stable.offers(&build("0.0.2", "stable", COMMIT, "2026-11-20T03:10:00Z")));
        assert!(!stable.offers(&build("0.0.3", "stable", OTHER, "2026-12-01T00:00:00Z")));
        // A beta made after 0.0.1 carries 0.0.1, so the release after it is offered; one made
        // after 0.0.2 is not taken back to 0.0.2.
        assert!(stable.offers(&build("0.0.1", "beta", OTHER, "2026-11-25T00:00:00Z")));
        assert!(!stable.offers(&build("0.0.2", "beta", OTHER, "2026-11-25T00:00:00Z")));
        assert_eq!(
            stable.described_for(&build("0.0.1", "stable", OTHER, "2026-10-06T11:00:00Z")),
            "0.0.1 to 0.0.2"
        );

        let beta =
            Manifest::parse(text("beta", "0.0.2", COMMIT, "2026-11-25T02:00:00Z").as_bytes()).unwrap();
        assert!(beta.offers(&build("0.0.2", "beta", OTHER, "2026-11-24T02:00:00Z")));
        assert!(beta.offers(&build("0.0.2", "stable", OTHER, "2026-11-20T03:10:00Z")));
        // The same commit, or a build newer than the beta: nothing.
        assert!(!beta.offers(&build("0.0.2", "beta", COMMIT, "2026-11-25T02:00:00Z")));
        assert!(!beta.offers(&build("0.0.3", "stable", OTHER, "2026-12-01T00:00:00Z")));
        assert_eq!(
            beta.described_for(&build("0.0.2", "beta", OTHER, "2026-11-24T02:00:00Z")),
            "0570c61aa1b2 built 2026-11-24 to 42992d086a78 built 2026-11-25"
        );
    }

    #[test]
    fn a_manifest_is_the_repositorys_unless_a_setting_names_another() {
        let none = |_: &str| None;
        assert_eq!(
            manifest_url(Channel::Stable, &none),
            "https://github.com/davidbuzz/MissionPlannerRust/releases/latest/download/update.json"
        );
        assert_eq!(
            manifest_url(Channel::Beta, &none),
            "https://github.com/davidbuzz/MissionPlannerRust/releases/download/beta/update.json"
        );
        let set = |key: &str| Some(format!("http://127.0.0.1:5793/{key}.json"));
        assert_eq!(manifest_url(Channel::Beta, &set), "http://127.0.0.1:5793/BetaUpdateManifest.json");
        let blank = |_: &str| Some("  ".to_owned());
        assert!(manifest_url(Channel::Stable, &blank).starts_with("https://github.com/"));
    }
}
