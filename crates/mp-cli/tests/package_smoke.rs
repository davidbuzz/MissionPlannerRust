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

//! Deliverable 20's package smoke test, for the Debian package `tools/package.sh deb` writes: installed into
//! a clean container, both programs run, the package removed and nothing left behind.
//!
//! Needs Docker and the package, which a plain `cargo test` has neither of, so it is ignored
//! until asked: `MP_PACKAGE_DEB=dist/missionplanner-rust_0.1.0_amd64.deb cargo test -p mp-cli
//! --test package_smoke -- --ignored`. The container is `ubuntu:noble`, the release the package
//! is built on (its glibc is the floor the control file states).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

fn testdata() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata")
}

#[test]
#[ignore = "needs Docker and MP_PACKAGE_DEB"]
fn the_deb_installs_runs_and_removes_cleanly() {
    let deb = PathBuf::from(std::env::var("MP_PACKAGE_DEB").expect("MP_PACKAGE_DEB"));
    // A relative path is from the repository, where `tools/package.sh` writes `dist/`; cargo runs
    // this from the crate.
    let deb = if deb.is_absolute() {
        deb
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(deb)
    };
    let deb = deb.canonicalize().expect("the package exists");
    let testdata = testdata().canonicalize().unwrap();
    let script = "set -e\n\
        export DEBIAN_FRONTEND=noninteractive\n\
        apt-get update -qq >/dev/null\n\
        apt-get install -y -qq /pkg.deb >/dev/null\n\
        planner --version\n\
        headless-planner log loganalysis /testdata/dataflash/synthetic.log /tmp/report.xml | head -5\n\
        test -s /tmp/report.xml\n\
        test -f /usr/share/applications/planner.desktop\n\
        test -f /usr/share/icons/hicolor/128x128/apps/planner.png\n\
        apt-get remove -y -qq missionplanner-rust >/dev/null\n\
        test ! -e /usr/bin/planner\n\
        test ! -e /usr/bin/headless-planner\n\
        test ! -e /usr/share/applications/planner.desktop\n\
        test ! -e /usr/share/doc/missionplanner-rust\n\
        echo SMOKE-OK";
    let output = std::process::Command::new("docker")
        .args(["run", "--rm"])
        .arg("-v")
        .arg(format!("{}:/pkg.deb:ro", deb.display()))
        .arg("-v")
        .arg(format!("{}:/testdata:ro", testdata.display()))
        .args(["ubuntu:noble", "bash", "-c", script])
        .output()
        .expect("docker runs");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "stdout:\n{stdout}\nstderr:\n{stderr}"
    );
    assert!(stdout.contains("planner 0.1.0\n"), "{stdout}");
    assert!(
        stdout.contains("Log File /testdata/dataflash/synthetic.log\n"),
        "{stdout}"
    );
    assert!(stdout.trim_end().ends_with("SMOKE-OK"), "{stdout}");
}
