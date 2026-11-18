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

//! The release workflow's manifest tool, `tools/release/manifest.py`, held to the planner's
//! reader: what the tool writes from a release's four downloads is what `Manifest::parse` reads,
//! field by field, and what it checks of a published release is each download's size and hash.
//! The tool runs under Python, as it does in the workflow. updates.md, section 5.

#![cfg(unix)]
// A test file: its helpers fail a test by panicking, as its tests do.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use mp_update::download::sha256_hex;
use mp_update::identity::Channel;
use mp_update::manifest::{DEB, Manifest};

const REPOSITORY: &str = "https://github.com/davidbuzz/MissionPlannerRust";
const COMMIT: &str = "42992d086a789143682f2997adc737545e3d8c2e";
const BUILT: &str = "2026-11-20T03:10:00Z";
/// A release's downloads as `release.yml` names them, each with contents of its own.
const DOWNLOADS: [(&str, &str, &[u8]); 4] = [
    (
        "linux-x86_64",
        "missionplanner-rust_0_0_2_linux-x86_64.tar.gz",
        b"the linux archive",
    ),
    (
        DEB,
        "missionplanner-rust_0.0.2_amd64.deb",
        b"the debian package",
    ),
    (
        "windows-x86_64",
        "missionplanner-rust_0_0_2_windows-x86_64.zip",
        b"the windows zip",
    ),
    (
        "macos-universal",
        "missionplanner-rust_0_0_2_macos-universal.tar.gz",
        b"the macos archive",
    ),
];

fn scratch(test: &str) -> PathBuf {
    let dir = mp_os::temp_dir().join(format!("mp-update-tool-{test}-{}", mp_os::process_id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn downloads_in(dir: &Path) {
    std::fs::create_dir_all(dir).unwrap();
    for (_, name, bytes) in DOWNLOADS {
        std::fs::write(dir.join(name), bytes).unwrap();
    }
    // What else a release holds is not a download of the planner.
    std::fs::write(dir.join("SHA256SUMS.txt"), b"sums").unwrap();
}

fn tool(arguments: &[&str]) -> Output {
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/release/manifest.py");
    Command::new("python3")
        .arg(script)
        .args(arguments)
        .output()
        .expect("python3, which the release workflow runs the tool under")
}

fn write(dir: &Path, channel: &str, tag: &str, more: &[&str]) -> Output {
    let dir = dir.to_string_lossy();
    let mut arguments = vec![
        "write",
        "--channel",
        channel,
        "--tag",
        tag,
        "--commit",
        COMMIT,
        "--built",
        BUILT,
        "--repository",
        REPOSITORY,
        "--dir",
        &dir,
    ];
    arguments.extend_from_slice(more);
    tool(&arguments)
}

#[test]
fn the_release_job_writes_a_manifest_the_planner_reads() {
    let dir = scratch("write");
    downloads_in(&dir);
    let output = write(&dir, "stable", "v0_0_2", &[]);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let bytes = std::fs::read(dir.join("update.json")).unwrap();
    assert_eq!(
        output.stdout, bytes,
        "the manifest is printed as it is written"
    );
    let manifest = Manifest::parse(&bytes).unwrap();
    assert_eq!(manifest.channel, Channel::Stable);
    assert_eq!(
        manifest.version.to_string(),
        "0.0.2",
        "the tag's own numbers"
    );
    assert_eq!(manifest.tag, "v0_0_2");
    assert_eq!(manifest.commit, COMMIT);
    assert_eq!(manifest.built, BUILT);
    assert_eq!(
        manifest.notes_url,
        format!("{REPOSITORY}/releases/tag/v0_0_2")
    );
    assert_eq!(manifest.assets.len(), 4);
    for (platform, name, contents) in DOWNLOADS {
        let asset = manifest.asset(platform).unwrap();
        assert_eq!(asset.name, name);
        assert_eq!(
            asset.url,
            format!("{REPOSITORY}/releases/download/v0_0_2/{name}")
        );
        assert_eq!(asset.size, contents.len() as u64);
        assert_eq!(asset.sha256, sha256_hex(contents));
    }

    // The beta: its tag is no version, so the workflow says which release it follows.
    let out = dir.join("beta.json");
    let output = write(
        &dir,
        "beta",
        "beta",
        &["--version", "0.0.2", "--out", &out.to_string_lossy()],
    );
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let beta = Manifest::parse(&std::fs::read(&out).unwrap()).unwrap();
    assert_eq!((beta.channel, beta.tag.as_str()), (Channel::Beta, "beta"));
    assert_eq!(beta.version.to_string(), "0.0.2");
    assert!(
        beta.asset(DEB)
            .unwrap()
            .url
            .starts_with(&format!("{REPOSITORY}/releases/download/beta/"))
    );
    let output = write(&dir, "beta", "beta", &[]);
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("is not a release tag"));

    // A release short of a download, or with two for one platform, gets no manifest.
    std::fs::remove_file(dir.join("update.json")).unwrap();
    std::fs::rename(dir.join(DOWNLOADS[2].1), dir.join("elsewhere.bin")).unwrap();
    let output = write(&dir, "stable", "v0_0_2", &[]);
    assert!(!output.status.success());
    let said = String::from_utf8_lossy(&output.stderr).into_owned();
    assert!(
        said.contains("holds 0 downloads for windows-x86_64"),
        "{said}"
    );
    assert!(!dir.join("update.json").exists());
    std::fs::rename(dir.join("elsewhere.bin"), dir.join(DOWNLOADS[2].1)).unwrap();
    std::fs::write(
        dir.join("missionplanner-rust_0_0_1_windows-x86_64.zip"),
        b"an older one",
    )
    .unwrap();
    let output = write(&dir, "stable", "v0_0_2", &[]);
    assert!(!output.status.success());
    assert!(
        String::from_utf8_lossy(&output.stderr).contains("holds 2 downloads for windows-x86_64")
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn the_release_job_checks_what_it_published() {
    // A release as its addresses lay it out, on this machine's disk for GitHub.
    let root = scratch("verify");
    let release = root.join("releases/download/v0_0_2");
    downloads_in(&release);
    let repository = format!("file://{}", root.display());
    let manifest = root.join("update.json");
    let release_text = release.to_string_lossy();
    let manifest_text = manifest.to_string_lossy();
    let output = tool(&[
        "write",
        "--channel",
        "stable",
        "--tag",
        "v0_0_2",
        "--commit",
        COMMIT,
        "--built",
        BUILT,
        "--repository",
        &repository,
        "--dir",
        &release_text,
        "--out",
        &manifest_text,
    ]);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );

    let output = tool(&["verify", &manifest_text]);
    let said = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(
        output.status.success(),
        "{said}{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        said.starts_with("stable 0.0.2 (v0_0_2, 42992d086a78, built 2026-11-20T03:10:00Z)"),
        "{said}"
    );
    assert_eq!(said.matches("  ok   ").count(), 4, "{said}");

    // One download changed after the manifest was written, and one gone.
    std::fs::write(release.join(DOWNLOADS[0].1), b"not the linux archive").unwrap();
    std::fs::remove_file(release.join(DOWNLOADS[3].1)).unwrap();
    let output = tool(&["verify", &manifest_text]);
    let said = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(!output.status.success(), "{said}");
    assert!(
        said.contains(
            "  FAIL linux-x86_64: missionplanner-rust_0_0_2_linux-x86_64.tar.gz is 21 bytes"
        ),
        "{said}"
    );
    assert!(said.contains("  FAIL macos-universal: "), "{said}");
    assert_eq!(said.matches("  ok   ").count(), 2, "{said}");
    let _ = std::fs::remove_dir_all(&root);
}

/// What a run of the release workflow builds, as `tools/release/what.sh` decides it from how the
/// run was started - the two cases that ask GitHub nothing.
#[test]
fn a_tag_is_a_stable_release_and_a_run_by_hand_is_no_release() {
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/release/what.sh");
    let run = |environment: &[(&str, &str)]| {
        let output = Command::new("bash")
            .arg(&script)
            .env_clear()
            .env("PATH", std::env::var_os("PATH").unwrap_or_default())
            .envs(environment.iter().copied())
            .output()
            .expect("bash");
        (
            output.status.success(),
            String::from_utf8_lossy(&output.stdout).into_owned(),
        )
    };
    let answer = |said: &str, name: &str| {
        said.lines()
            .find_map(|line| line.strip_prefix(&format!("{name}=")))
            .unwrap_or_else(|| panic!("{name} in {said}"))
            .to_owned()
    };

    // A release tag: the stable channel, its numbers with points for the version and the Debian
    // package, its own form for the downloads' names.
    let (ok, said) = run(&[
        ("GITHUB_REF_TYPE", "tag"),
        ("GITHUB_REF_NAME", "v0_0_2"),
        ("GITHUB_SHA", COMMIT),
    ]);
    assert!(ok, "{said}");
    for (name, value) in [
        ("build", "true"),
        ("channel", "stable"),
        ("version", "0.0.2"),
        ("label", "0_0_2"),
        ("deb", "0.0.2"),
        ("tag", "v0_0_2"),
    ] {
        assert_eq!(answer(&said, name), value, "{name}");
    }
    assert_eq!(answer(&said, "built").len(), "2026-11-20T03:10:00Z".len());
    // What the planner makes of those values: release 0.0.2.
    let identity = mp_update::identity::Identity::from_values(
        Some(&answer(&said, "version")),
        Some(COMMIT),
        Some(&answer(&said, "channel")),
        Some(&answer(&said, "built")),
    )
    .unwrap();
    assert_eq!(identity.version.to_string(), "0.0.2");

    // A run by hand with no channel: built, and no release - a development build.
    let (ok, said) = run(&[
        ("GITHUB_REF_TYPE", "branch"),
        ("GITHUB_EVENT_NAME", "workflow_dispatch"),
        ("WANTED", "none"),
        ("GITHUB_SHA", COMMIT),
    ]);
    assert!(ok, "{said}");
    assert_eq!(answer(&said, "build"), "true");
    assert_eq!(answer(&said, "channel"), "");
    assert_eq!(
        mp_update::identity::Identity::from_values(
            Some(&answer(&said, "version")),
            Some(COMMIT),
            Some(&answer(&said, "channel")),
            None
        ),
        None
    );

    // A tag that is no release.
    let (ok, _) = run(&[
        ("GITHUB_REF_TYPE", "tag"),
        ("GITHUB_REF_NAME", "vnext"),
        ("GITHUB_SHA", COMMIT),
    ]);
    assert!(!ok);
}
