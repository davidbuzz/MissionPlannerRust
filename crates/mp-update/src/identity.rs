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

//! What this build is: the release it was built as, from which commit, on which channel. The
//! release workflow compiles the values in (`MP_RELEASE_VERSION`, `MP_RELEASE_COMMIT`,
//! `MP_RELEASE_CHANNEL`, `MP_RELEASE_BUILT`); a build without them, `cargo build` on a
//! developer's machine, is a development build and is never updated from a release. Not in the
//! C#, which reads `version.txt` beside the program: updates.md, section 4.3.

use crate::release::ReleaseVersion;

/// The harness's door: `version,channel,commit,built`, read in place of the compiled values, so a
/// test's planner - a development build - can be a release for its length.
pub const DOOR: &str = "MP_UPDATE_IDENTITY";

/// Which run of releases a build belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// The tagged releases.
    Stable,
    /// The rolling pre-release built from main.
    Beta,
}

impl Channel {
    /// The name the manifest and the build values write.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Stable => "stable",
            Self::Beta => "beta",
        }
    }

    /// The channel of that name.
    #[must_use]
    pub fn parse(text: &str) -> Option<Self> {
        match text.trim() {
            "stable" => Some(Self::Stable),
            "beta" => Some(Self::Beta),
            _ => None,
        }
    }
}

/// A release build's own description of itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Identity {
    /// The release's version; for a beta, that of the last release tag before its commit.
    pub version: ReleaseVersion,
    /// The commit it was built from, in full.
    pub commit: String,
    /// Its channel.
    pub channel: Channel,
    /// When it was built, `2026-11-20T03:10:00Z`; empty when the build did not say.
    pub built: String,
}

impl Identity {
    /// The identity those values give: a version, a commit and a channel, each present and
    /// readable, or `None` - a development build.
    #[must_use]
    pub fn from_values(
        version: Option<&str>,
        commit: Option<&str>,
        channel: Option<&str>,
        built: Option<&str>,
    ) -> Option<Self> {
        let commit = commit.map(str::trim).filter(|c| !c.is_empty())?;
        Some(Self {
            version: ReleaseVersion::parse(version?).ok()?,
            commit: commit.to_owned(),
            channel: Channel::parse(channel?)?,
            built: built.map(str::trim).unwrap_or_default().to_owned(),
        })
    }

    /// The identity the door's text names: `version,channel,commit,built`.
    #[must_use]
    pub fn from_door(text: &str) -> Option<Self> {
        let mut parts = text.split(',');
        let version = parts.next();
        let channel = parts.next();
        let commit = parts.next();
        let built = parts.next();
        Self::from_values(version, commit, channel, built)
    }

    /// The commit's first twelve characters, as `--version` and the question show it.
    #[must_use]
    pub fn short_commit(&self) -> &str {
        short(&self.commit)
    }
}

/// A commit's first twelve characters.
#[must_use]
pub fn short(commit: &str) -> &str {
    commit.get(..12).unwrap_or(commit)
}

/// This build: the door's when it is set, else what the release workflow compiled in.
#[must_use]
pub fn this_build() -> Option<Identity> {
    if let Ok(door) = std::env::var(DOOR) {
        return Identity::from_door(&door);
    }
    Identity::from_values(
        option_env!("MP_RELEASE_VERSION"),
        option_env!("MP_RELEASE_COMMIT"),
        option_env!("MP_RELEASE_CHANNEL"),
        option_env!("MP_RELEASE_BUILT"),
    )
}

/// What `--version` prints: the program and its version, and what kind of build it is.
#[must_use]
pub fn version_line(program: &str, package_version: &str, identity: Option<&Identity>) -> String {
    match identity {
        Some(identity) => {
            let built = if identity.built.is_empty() {
                String::new()
            } else {
                format!(", built {}", identity.built)
            };
            format!(
                "{program} {} ({}, commit {}{built})",
                identity.version,
                identity.channel.name(),
                identity.short_commit()
            )
        }
        None => format!("{program} {package_version} (development build)"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const COMMIT: &str = "42992d086a789143682f2997adc737545e3d8c2e";

    #[test]
    fn a_build_with_its_three_values_is_a_release_and_any_other_a_development_build() {
        let identity =
            Identity::from_values(Some("0.0.2"), Some(COMMIT), Some("stable"), Some("2026-11-20T03:10:00Z"))
                .unwrap();
        assert_eq!(identity.version.to_string(), "0.0.2");
        assert_eq!(identity.channel, Channel::Stable);
        assert_eq!(identity.short_commit(), "42992d086a78");
        // The tag's own form reads too, and a build that does not say when is still a release.
        let beta = Identity::from_values(Some("0_0_2"), Some(COMMIT), Some("beta"), None).unwrap();
        assert_eq!((beta.channel, beta.built.as_str()), (Channel::Beta, ""));
        // Nothing compiled in, or any one of the three missing, empty or unreadable.
        assert_eq!(Identity::from_values(None, None, None, None), None);
        assert_eq!(Identity::from_values(None, Some(COMMIT), Some("stable"), None), None);
        assert_eq!(Identity::from_values(Some("0.0.2"), None, Some("stable"), None), None);
        assert_eq!(Identity::from_values(Some("0.0.2"), Some(" "), Some("stable"), None), None);
        assert_eq!(Identity::from_values(Some("0.0.2"), Some(COMMIT), None, None), None);
        assert_eq!(Identity::from_values(Some("0.0.2"), Some(COMMIT), Some("master"), None), None);
        assert_eq!(Identity::from_values(Some("next"), Some(COMMIT), Some("stable"), None), None);
    }

    #[test]
    fn the_door_names_an_identity_in_one_line() {
        let identity = Identity::from_door(&format!("0.0.1,beta,{COMMIT},2026-11-01T00:00:00Z")).unwrap();
        assert_eq!(identity.version.to_string(), "0.0.1");
        assert_eq!(identity.channel, Channel::Beta);
        assert_eq!(identity.built, "2026-11-01T00:00:00Z");
        assert_eq!(Identity::from_door("0.0.1,beta"), None);
        assert_eq!(Identity::from_door(""), None);
    }

    #[test]
    fn version_says_what_kind_of_build_this_is() {
        let identity =
            Identity::from_values(Some("v0_0_2"), Some(COMMIT), Some("stable"), Some("2026-11-20T03:10:00Z"));
        assert_eq!(
            version_line("planner", "0.0.0", identity.as_ref()),
            "planner 0.0.2 (stable, commit 42992d086a78, built 2026-11-20T03:10:00Z)"
        );
        assert_eq!(
            version_line("planner", "0.0.0", None),
            "planner 0.0.0 (development build)"
        );
    }
}
