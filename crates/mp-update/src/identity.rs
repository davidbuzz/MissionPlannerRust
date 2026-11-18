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
//! `MP_RELEASE_CHANNEL`, `MP_RELEASE_BUILT`). A build without them, `cargo build` on a
//! developer's machine - `target/debug/planner`, `target/release/planner` - is a development
//! build: it updates as any planner does when its buttons are pressed, offered whatever the
//! channel holds, since it cannot say which release it is (the owner, 2026-11-18: his local
//! working copy is that binary, and its update is to work); only the check at start is left to
//! the releases. Not in the C#, which reads `version.txt` beside the program: updates.md,
//! section 4.3.

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
    /// Whether the build was told what it is - the release workflow's, or the harness's door's -
    /// and not a development build, which only knows its package's version.
    pub stamped: bool,
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
            stamped: true,
        })
    }

    /// A development build's identity: its package's version, no commit and no time, on the
    /// stable channel.
    #[must_use]
    pub fn development(package_version: &str) -> Self {
        Self {
            version: ReleaseVersion::parse(package_version)
                .unwrap_or_else(|_| ReleaseVersion::zero()),
            commit: "development".to_owned(),
            channel: Channel::Stable,
            built: String::new(),
            stamped: false,
        }
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

/// This build: the door's when it is set and reads, else what the release workflow compiled in,
/// else a development build of this package's version.
#[must_use]
pub fn this_build() -> Identity {
    let told = match std::env::var(DOOR) {
        Ok(door) => Identity::from_door(&door),
        Err(_) => Identity::from_values(
            option_env!("MP_RELEASE_VERSION"),
            option_env!("MP_RELEASE_COMMIT"),
            option_env!("MP_RELEASE_CHANNEL"),
            option_env!("MP_RELEASE_BUILT"),
        ),
    };
    told.unwrap_or_else(|| Identity::development(env!("CARGO_PKG_VERSION")))
}

/// What `--version` prints: the program and its version, and what kind of build it is.
#[must_use]
pub fn version_line(program: &str, identity: &Identity) -> String {
    if !identity.stamped {
        return format!("{program} {} (development build)", identity.version);
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    const COMMIT: &str = "42992d086a789143682f2997adc737545e3d8c2e";

    #[test]
    fn a_build_with_its_three_values_is_a_release_and_any_other_a_development_build() {
        let identity = Identity::from_values(
            Some("0.0.2"),
            Some(COMMIT),
            Some("stable"),
            Some("2026-11-20T03:10:00Z"),
        )
        .unwrap();
        assert_eq!(identity.version.to_string(), "0.0.2");
        assert_eq!(identity.channel, Channel::Stable);
        assert_eq!(identity.short_commit(), "42992d086a78");
        // The tag's own form reads too, and a build that does not say when is still a release.
        let beta = Identity::from_values(Some("0_0_2"), Some(COMMIT), Some("beta"), None).unwrap();
        assert_eq!((beta.channel, beta.built.as_str()), (Channel::Beta, ""));
        // Nothing compiled in, or any one of the three missing, empty or unreadable.
        assert_eq!(Identity::from_values(None, None, None, None), None);
        assert_eq!(
            Identity::from_values(None, Some(COMMIT), Some("stable"), None),
            None
        );
        assert_eq!(
            Identity::from_values(Some("0.0.2"), None, Some("stable"), None),
            None
        );
        assert_eq!(
            Identity::from_values(Some("0.0.2"), Some(" "), Some("stable"), None),
            None
        );
        assert_eq!(
            Identity::from_values(Some("0.0.2"), Some(COMMIT), None, None),
            None
        );
        assert_eq!(
            Identity::from_values(Some("0.0.2"), Some(COMMIT), Some("master"), None),
            None
        );
        assert_eq!(
            Identity::from_values(Some("next"), Some(COMMIT), Some("stable"), None),
            None
        );
    }

    #[test]
    fn the_door_names_an_identity_in_one_line() {
        let identity =
            Identity::from_door(&format!("0.0.1,beta,{COMMIT},2026-11-01T00:00:00Z")).unwrap();
        assert_eq!(identity.version.to_string(), "0.0.1");
        assert_eq!(identity.channel, Channel::Beta);
        assert_eq!(identity.built, "2026-11-01T00:00:00Z");
        assert_eq!(Identity::from_door("0.0.1,beta"), None);
        assert_eq!(Identity::from_door(""), None);
    }

    #[test]
    fn version_says_what_kind_of_build_this_is() {
        let identity = Identity::from_values(
            Some("v0_0_2"),
            Some(COMMIT),
            Some("stable"),
            Some("2026-11-20T03:10:00Z"),
        );
        assert_eq!(
            version_line("planner", &identity.unwrap()),
            "planner 0.0.2 (stable, commit 42992d086a78, built 2026-11-20T03:10:00Z)"
        );
        // A build nobody told what it is: its package's version, and that it is a developer's.
        let development = Identity::development("0.0.0");
        assert!(!development.stamped);
        assert_eq!(
            (development.channel, development.built.as_str()),
            (Channel::Stable, "")
        );
        assert_eq!(
            version_line("planner", &development),
            "planner 0.0.0 (development build)"
        );
        assert_eq!(
            Identity::development("not a version").version.to_string(),
            "0"
        );
    }
}
