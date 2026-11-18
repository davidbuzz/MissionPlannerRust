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

//! `Update.CheckForUpdate`, over a release's manifest: the channel's `update.json` fetched and
//! read, and whether it is a release to offer this build. Where the C# compares `version.txt`
//! beside the program with the channel's (`Update.cs:118-178`), the build knows what it is
//! (`identity.rs`) and the manifest says what the release is (`manifest.rs`). updates.md,
//! section 4.4.

use crate::download::{Source, get};
use crate::identity::{Channel, Identity};
use crate::manifest::Manifest;

/// The most a manifest may be; one is about a kilobyte.
pub const MAX_MANIFEST: u64 = 1 << 20;

/// What a check found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Check {
    /// A release to offer this build.
    UpdateFound(Box<Manifest>),
    /// The channel has nothing newer.
    UpToDate,
}

/// The channel's manifest at `url`, against this build.
///
/// # Errors
///
/// The fetch's failure, a manifest that does not read ([`Manifest::parse`]), or one of another
/// channel than the one asked for.
pub fn check_for_update(
    source: &dyn Source,
    url: &str,
    channel: Channel,
    build: &Identity,
) -> Result<Check, String> {
    let manifest = Manifest::parse(&get(source, url, MAX_MANIFEST)?)?;
    if manifest.channel != channel {
        return Err(format!(
            "the manifest at {url} is the {} channel's, not the {} channel's",
            manifest.channel.name(),
            channel.name()
        ));
    }
    Ok(if manifest.offers(build) {
        Check::UpdateFound(Box::new(manifest))
    } else {
        Check::UpToDate
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::download::tests::Server;
    use crate::manifest::tests::{COMMIT, OTHER, text};

    #[test]
    fn a_check_reads_the_channels_manifest_and_says_whether_it_is_newer() {
        let url = "https://x/update.json";
        let stable = text("stable", "0.0.2", COMMIT, "2026-11-20T03:10:00Z");
        let server = Server::with(&[(url, stable.as_bytes())]);
        let build = |version: &str| {
            Identity::from_values(Some(version), Some(OTHER), Some("stable"), Some("2026-10-06T11:00:00Z"))
                .unwrap()
        };
        match check_for_update(&server, url, Channel::Stable, &build("0.0.1")).unwrap() {
            Check::UpdateFound(manifest) => assert_eq!(manifest.tag, "v0_0_2"),
            Check::UpToDate => panic!("0.0.2 is above 0.0.1"),
        }
        assert_eq!(check_for_update(&server, url, Channel::Stable, &build("0.0.2")).unwrap(), Check::UpToDate);
        // The beta button is never answered with the stable channel's release.
        assert_eq!(
            check_for_update(&server, url, Channel::Beta, &build("0.0.1")).unwrap_err(),
            "the manifest at https://x/update.json is the stable channel's, not the beta channel's"
        );
        // No answer, and an answer that is no manifest.
        assert!(check_for_update(&server, "https://x/none", Channel::Stable, &build("0.0.1")).unwrap_err().contains("404"));
        let page = Server::with(&[(url, b"<html>Not Found</html>")]);
        assert!(
            check_for_update(&page, url, Channel::Stable, &build("0.0.1"))
                .unwrap_err()
                .starts_with("the update manifest is not JSON")
        );
    }
}
