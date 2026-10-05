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

//! The probe for ArduPilot's WebAssembly SITL, and the note the page shows until there is one.
//!
//! Not in the C#: the owner's ruling D14 (PLAN.md §12, §13.6 row 78) makes a WebAssembly SITL,
//! run under a bundled runtime, the engine on the desktops where the C#'s Cygwin build cannot
//! run - once ArduPilot publishes one. Until then the page says so, and a SITL already running is
//! flown through the connection controls as any TCP link is.
//!
//! **Where it would be published.** Row 78 names no address, and ArduPilot has announced none.
//! Every build ArduPilot's build server publishes is listed in its firmware manifest,
//! [`PROBE_URL`] - the SITL builds `CheckandGetSITLImage` runs on Linux among them, as the
//! platforms `SITL_x86_64_linux_gnu` and `SITL_arm_linux_gnueabihf` (`SITL.cs:302-375`). So the
//! probe reads the manifest the application already fetches, through the same client.
//!
//! **The check.** A record whose `platform` names WebAssembly (`wasm` in any case, as a
//! `SITL_wasm32` would) or whose `format` is `wasm`. The platform's exact name is ArduPilot's to
//! choose; this is the owner's to confirm when one appears.
//!
//! **What it would download.** That record's `url`: the module the runtime would load. Nothing
//! is downloaded here, and there is no runtime to run it yet.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_firmware::manifest::{self, Fetch, FirmwareInfo, Manifest, MavType, ReleaseType};

/// Where the probe looks: ArduPilot's firmware manifest, as `APFirmware.GetList()` fetches it.
pub const PROBE_URL: &str = manifest::MANIFEST_URL;

/// What the probe found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Probe {
    /// Not looked yet.
    NotYetAsked,
    /// The manifest lists one.
    Published {
        /// The record's `url`: what would be downloaded.
        url: String,
        /// The record's `platform`.
        platform: String,
    },
    /// The manifest lists none.
    NotPublished,
    /// The manifest could not be had: why.
    Unreachable(String),
}

impl Probe {
    /// From a record the manifest has, or has not.
    #[must_use]
    pub fn from_record(record: Option<&FirmwareInfo>) -> Self {
        record.map_or(Self::NotPublished, |fw| Self::Published {
            url: fw.url.clone().unwrap_or_default(),
            platform: fw.platform.clone().unwrap_or_default(),
        })
    }
}

/// Whether a record is a WebAssembly build.
#[must_use]
pub fn is_wasm(fw: &FirmwareInfo) -> bool {
    fw.platform
        .as_deref()
        .is_some_and(|platform| platform.to_ascii_lowercase().contains("wasm"))
        || fw
            .format
            .as_deref()
            .is_some_and(|format| format.eq_ignore_ascii_case("wasm"))
}

/// The first WebAssembly build in the manifest, for a vehicle and a release when they are given.
#[must_use]
pub fn published(
    catalogue: &Manifest,
    mav_type: Option<MavType>,
    release: Option<ReleaseType>,
) -> Option<&FirmwareInfo> {
    catalogue.firmware.iter().find(|fw| {
        is_wasm(fw)
            && mav_type.is_none_or(|m| fw.mav_type.as_deref() == Some(m.name()))
            && release.is_none_or(|r| fw.mav_firmware_version_type.as_deref() == Some(r.name()))
    })
}

/// Reads the manifest through `fetch` and looks.
pub fn probe(fetch: &dyn Fetch) -> Probe {
    let mut held = None;
    let errors = manifest::get_list(&mut held, PROBE_URL, false, fetch);
    match held {
        Some(catalogue) => Probe::from_record(published(&catalogue, None, None)),
        None => Probe::Unreachable(
            errors
                .first()
                .map_or_else(|| "no manifest".to_owned(), ToString::to_string),
        ),
    }
}

/// Why the C#'s way does not apply here.
const WHY: &str = "Mission Planner starts SITL through Cygwin, which only Windows has.";
/// What can be done today, which the connection controls already do.
const MEANWHILE: &str = "A SITL already running can be flown now: choose TCP in the connection controls and connect to 127.0.0.1, port 5760.";

/// The page's note for what the probe found.
#[must_use]
pub fn note(probe: &Probe) -> String {
    let middle = match probe {
        Probe::NotYetAsked => "On this desktop the application will run ArduPilot's WebAssembly SITL under a bundled runtime once ArduPilot publishes one.".to_owned(),
        Probe::NotPublished => "ArduPilot's firmware manifest lists no WebAssembly SITL yet; once it does, the application will run it under a bundled runtime.".to_owned(),
        Probe::Unreachable(reason) => format!(
            "ArduPilot's firmware manifest, where a WebAssembly SITL would be listed, could not be read ({reason})."
        ),
        Probe::Published { url, .. } => format!(
            "ArduPilot publishes a WebAssembly SITL ({url}), but the runtime to run it is not yet bundled with this application."
        ),
    };
    format!("{WHY} {middle} {MEANWHILE}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl::launcher::tests::StubWeb;

    /// A manifest of the given records, gzipped as the server sends it.
    fn gzipped(records: &str) -> Vec<u8> {
        use std::io::Write as _;
        let json = format!("{{\"format-version\": \"1.0.0\", \"firmware\": [{records}]}}");
        let mut encoder = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
        encoder.write_all(json.as_bytes()).expect("gzip");
        encoder.finish().expect("gzip")
    }

    const SITL_LINUX: &str = r#"{"board_id": 0, "mav-type": "Copter", "mav-firmware-version-type": "OFFICIAL", "platform": "SITL_x86_64_linux_gnu", "format": "ELF", "url": "https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter"}"#;
    const SITL_WASM: &str = r#"{"board_id": 0, "mav-type": "Copter", "mav-firmware-version-type": "DEV", "platform": "SITL_wasm32", "format": "wasm", "url": "https://firmware.ardupilot.org/Copter/latest/SITL_wasm32/arducopter.wasm"}"#;

    /// The trimmed real manifest has none: "not yet".
    #[test]
    fn the_real_manifest_excerpt_lists_no_wasm_sitl() {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/firmware/manifest.json.gz");
        let fetch = manifest::FromFile { path: fixture };
        assert_eq!(probe(&fetch), Probe::NotPublished);
        assert!(note(&Probe::NotPublished).contains("lists no WebAssembly SITL yet"));
    }

    /// A manifest with one: its URL is what would be downloaded, and the page says so; the one
    /// URL asked for is the manifest's.
    #[test]
    fn a_published_wasm_sitl_is_found_with_its_url() {
        let web =
            StubWeb::default().serve(PROBE_URL, &gzipped(&format!("{SITL_LINUX}, {SITL_WASM}")));
        let found = probe(&web);
        assert_eq!(
            found,
            Probe::Published {
                url: "https://firmware.ardupilot.org/Copter/latest/SITL_wasm32/arducopter.wasm"
                    .to_owned(),
                platform: "SITL_wasm32".to_owned()
            }
        );
        assert_eq!(
            web.asked().first().map(String::as_str),
            Some("https://firmware.ardupilot.org/manifest.json.gz")
        );
        assert!(note(&found).contains("SITL_wasm32/arducopter.wasm"));
        let catalogue =
            Manifest::decode(&gzipped(&format!("{SITL_LINUX}, {SITL_WASM}")), true).expect("ok");
        assert!(published(&catalogue, Some(MavType::Copter), Some(ReleaseType::Dev)).is_some());
        assert!(
            published(
                &catalogue,
                Some(MavType::Copter),
                Some(ReleaseType::Official)
            )
            .is_none()
        );
        assert!(published(&catalogue, Some(MavType::FixedWing), None).is_none());
    }

    /// No manifest at all: the reason, and still the way to fly one already running.
    #[test]
    fn an_unreachable_manifest_says_why() {
        let found = probe(&StubWeb::default());
        let Probe::Unreachable(reason) = &found else {
            panic!("{found:?}");
        };
        assert!(reason.contains("404"), "{reason}");
        let text = note(&found);
        assert!(text.contains("could not be read"));
        assert!(text.contains("127.0.0.1, port 5760"));
    }

    /// The Linux launcher over a manifest without its platform: the note, not a download.
    #[test]
    fn the_linux_launcher_without_a_record_gives_the_note() {
        use crate::sitl::launcher::{Image, Launcher as _, ManifestSitl};
        let dir = crate::sitl::launcher::tests::scratch("manifest-none");
        let web = StubWeb::default().serve(PROBE_URL, &gzipped(SITL_WASM));
        let image = ManifestSitl::new("SITL_x86_64_linux_gnu").image(
            "ArduCopter.elf",
            Some(ReleaseType::Official),
            &dir,
            &web,
            &|_| {},
        );
        let Image::NotAvailable(text) = image else {
            panic!("{image:?}");
        };
        // The DEV WebAssembly build is not the stable release asked for.
        assert!(text.contains("lists no WebAssembly SITL yet"), "{text}");
        let _ = mp_os::fs::remove_dir_all(dir);
    }

    /// The Linux launcher says "Downloading sitl software" before it asks the web for anything -
    /// the manifest a session's first click fetches included - with a release to download and
    /// with "Skip Download" alike (the owner's bug of 2026-10-06: three or four seconds after the
    /// click before the page said anything).
    #[test]
    fn the_linux_launcher_says_downloading_before_the_manifest() {
        use crate::sitl::launcher::{Launcher as _, ManifestSitl};
        for release in [Some(ReleaseType::Official), None] {
            let dir = crate::sitl::launcher::tests::scratch("manifest-say");
            let web = StubWeb::default()
                .serve(PROBE_URL, &gzipped(&format!("{SITL_WASM}, {SITL_LINUX}")))
                .serve(
                    "https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter",
                    b"\x7fELF",
                );
            let said = std::cell::RefCell::new(Vec::new());
            let _ = ManifestSitl::new("SITL_x86_64_linux_gnu").image(
                "ArduCopter.elf",
                release,
                &dir,
                &web,
                &|text| said.borrow_mut().push((text.to_owned(), web.asked().len())),
            );
            assert_eq!(
                said.into_inner(),
                vec![(crate::sitl::model::DOWNLOADING.to_owned(), 0)],
                "{release:?}"
            );
            assert!(!web.asked().is_empty(), "the manifest is fetched after");
            let _ = mp_os::fs::remove_dir_all(dir);
        }
    }

    /// The Linux launcher with its record: downloaded to the name without `.elf`, executable.
    #[test]
    fn the_linux_launcher_downloads_its_platforms_record() {
        use crate::sitl::launcher::{Image, Launcher as _, ManifestSitl};
        let dir = crate::sitl::launcher::tests::scratch("manifest-some");
        let web = StubWeb::default()
            .serve(PROBE_URL, &gzipped(&format!("{SITL_WASM}, {SITL_LINUX}")))
            .serve(
                "https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter",
                b"\x7fELF",
            );
        let image = ManifestSitl::new("SITL_x86_64_linux_gnu").image(
            "ArduCopter.elf",
            Some(ReleaseType::Official),
            &dir,
            &web,
            &|_| {},
        );
        assert_eq!(image, Image::Found(dir.join("ArduCopter")));
        assert_eq!(
            mp_os::fs::read(dir.join("ArduCopter")).ok(),
            Some(b"\x7fELF".to_vec())
        );
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt as _;
            let mode = mp_os::fs::metadata(dir.join("ArduCopter"))
                .map(|m| m.permissions().mode() & 0o777)
                .unwrap_or_default();
            assert_eq!(mode, 0o755);
        }
        let _ = mp_os::fs::remove_dir_all(dir);
    }
}
