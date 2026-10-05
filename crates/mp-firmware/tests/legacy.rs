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

//! The legacy catalogue and the firmware pages' flows against fixtures.
//!
//! `testdata/legacy/firmware2.xml` is ArduPilot's own `firmware2.xml`, whole, as fetched from
//! `firmware.ardupilot.org/Tools/MissionPlanner/Firmware/` on 2026-09-24; `git-version.txt` is
//! the one beside its Copter `fmuv3` build that day. `arducopter.apj` is not a firmware: it is a
//! synthetic container in the `.apj` shape, a 3,000-byte made-up image for board 140, written by
//! `mkapj.py` beside it. The network is a [`Mirror`] of a scratch directory holding those files
//! at the paths their URLs name.
//!
//! The expected labels were worked out from the fixture independently of this code, with the
//! C#'s `getAPMVersion` and `updateDisplayName` applied by hand in Python over the same XML and
//! the same mirror.

#![allow(clippy::indexing_slicing, clippy::expect_used, clippy::unwrap_used)]

use std::path::PathBuf;

use mp_firmware::detect::{Boards, DeviceInfo};
use mp_firmware::flow::{
    self, Buttons, Cx, Dialogue, Reached, Stop, custom_legacy, custom_manifest, download_and_flash,
    find_firmware,
};
use mp_firmware::legacy::{
    FIRMWARE_URLS, Picture, Software, display_name, get_fw_list, nice_names, parse_list,
};
use mp_firmware::manifest::{Fetch, Mirror};

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("testdata/legacy")
        .join(name)
}

/// A scratch directory, removed when the test ends.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Self {
        let dir =
            mp_os::temp_dir().join(format!("mp-firmware-legacy-{name}-{}", mp_os::process_id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        Self(dir)
    }

    /// A file at the path its URL names, copied from a fixture.
    fn serve(&self, url_path: &str, fixture_name: &str) {
        let path = self.0.join("web").join(url_path);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::copy(fixture(fixture_name), path).unwrap();
    }

    fn mirror(&self) -> Mirror {
        Mirror {
            root: self.0.join("web"),
        }
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// The web the tests run against: the list at its first source, and the Copter builds.
fn web(name: &str) -> Scratch {
    let scratch = Scratch::new(name);
    scratch.serve(
        "github.com/ArduPilot/binary/raw/master/Firmware/firmware2.xml",
        "firmware2.xml",
    );
    scratch.serve(
        "firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj",
        "arducopter.apj",
    );
    scratch.serve(
        "firmware.ardupilot.org/Copter/stable/fmuv3/git-version.txt",
        "git-version.txt",
    );
    scratch.serve(
        "firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj",
        "arducopter.apj",
    );
    scratch
}

/// A person who answers from a script, and remembers what they were asked and shown.
#[derive(Default)]
struct Person {
    answers: Vec<bool>,
    asked: Vec<(String, String, Buttons)>,
    shown: Vec<(String, String)>,
    progress: Vec<(i32, String)>,
}

impl Person {
    fn answering(answers: &[bool]) -> Self {
        Self {
            answers: answers.to_vec(),
            ..Self::default()
        }
    }
}

impl Dialogue for Person {
    fn ask(&mut self, text: &str, caption: &str, buttons: Buttons) -> bool {
        self.asked
            .push((text.to_owned(), caption.to_owned(), buttons));
        if self.answers.is_empty() {
            false
        } else {
            self.answers.remove(0)
        }
    }

    fn show(&mut self, text: &str, caption: &str) {
        self.shown.push((text.to_owned(), caption.to_owned()));
    }

    fn progress(&mut self, percent: i32, status: &str) {
        self.progress.push((percent, status.to_owned()));
    }
}

fn cx<'a>(person: &'a mut Person, fetch: &'a dyn Fetch, scratch: &Scratch) -> Cx<'a> {
    Cx {
        dialogue: person,
        fetch,
        user_data: scratch.0.join("MissionPlannerRust"),
        temp_dir: scratch.0.join("tmp"),
    }
}

fn list(fetch: &dyn Fetch) -> Vec<Software> {
    get_fw_list("", fetch, &mut |_, _| {}).expect("the list")
}

// --- the list ---------------------------------------------------------------------------------

#[test]
fn the_real_list_parses_into_twelve_entries() {
    let text = std::fs::read_to_string(fixture("firmware2.xml")).unwrap();
    let entries = parse_list(&text).expect("parses");
    assert_eq!(entries.len(), 12);
    let quad = &entries[3];
    assert_eq!(quad.name, "ArduCopter Quad Stable");
    assert_eq!(
        quad.url2560,
        "http://firmware.ardupilot.org/Copter/stable/apm1-quad/ArduCopter.hex"
    );
    assert_eq!(
        quad.url2560_2,
        "http://firmware.ardupilot.org/Copter/stable/apm2-quad/ArduCopter.hex"
    );
    assert_eq!(
        quad.urlfmuv3,
        "http://firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj"
    );
    assert_eq!(quad.urlpx4v1, "", "<urlpx4> is no field of software");
    assert_eq!(entries[11].name, "PX4 Stable");
}

#[test]
fn get_fw_list_reads_each_version_beside_its_fmuv3_build() {
    let scratch = web("list");
    let mirror = scratch.mirror();
    let mut said = Vec::new();
    let entries = get_fw_list("", &mirror, &mut |percent, status| {
        said.push((percent, status.to_owned()));
    })
    .expect("the list");
    assert_eq!(entries.len(), 12);
    // Copter's fmuv3 build is there, so its git-version.txt names the version.
    assert_eq!(entries[3].name, "ArduCopter V4.7.1");
    assert_eq!(entries[3].desc, "ArduCopter Quad Stable");
    // Plane's is not, nor its px4v2 one's version file: the entry is left as it was.
    assert_eq!(entries[0].name, "ArduPlane Stable");
    assert_eq!(entries[0].desc, "");
    assert_eq!(
        said.first().map(|(_, s)| s.as_str()),
        Some("Getting FW List")
    );
    assert_eq!(said.last().map(|(_, s)| s.as_str()), Some("Received List"));
    assert!(said.iter().any(|(_, s)| s == "Getting FW Version"));

    // The first source missing, the second is asked.
    let other = Scratch::new("second");
    other.serve(
        "firmware.ardupilot.org/Tools/MissionPlanner/Firmware/firmware2.xml",
        "firmware2.xml",
    );
    assert_eq!(list(&other.mirror()).len(), 12);
    // Neither: the last error.
    let none = Scratch::new("none");
    let err = get_fw_list(FIRMWARE_URLS, &none.mirror(), &mut |_, _| {}).expect_err("no list");
    assert!(err.contains("firmware.ardupilot.org"), "{err}");
}

#[test]
fn each_entry_labels_the_pictures_update_display_name_gives_it() {
    let scratch = web("labels");
    let mut labels = std::collections::HashMap::new();
    let mut homeless = Vec::new();
    for mut entry in list(&scratch.mirror()) {
        let name = entry.name.clone();
        let given = display_name(&mut entry);
        if given.is_empty() {
            homeless.push(name);
        }
        for (picture, text) in given {
            labels.insert(picture, text);
        }
    }
    let expected = [
        (Picture::Apm, "ArduPlane Stable"),
        (Picture::Sub, "ArduSub Stable"),
        (Picture::Rover, "ArduRover Stable"),
        (Picture::Quad, "ArduCopter V4.7.1 Quad"),
        (Picture::Tri, "ArduCopter V4.7.1 Tri"),
        (Picture::Hexa, "ArduCopter V4.7.1 Hexa"),
        (Picture::Y6, "ArduCopter V4.7.1 Y6"),
        (Picture::OctaQuad, "ArduCopter V4.7.1 Octa Quad"),
        (Picture::Octa, "ArduCopter V4.7.1 Octa"),
        (Picture::Heli, "ArduCopter Heli Stable heli"),
        (Picture::AntennaTracker, "Antenna Tracker"),
    ];
    assert_eq!(labels.len(), expected.len());
    for (picture, text) in expected {
        assert_eq!(
            labels.get(&picture).map(String::as_str),
            Some(text),
            "{picture:?}"
        );
    }
    assert_eq!(homeless, ["PX4 Stable"], "No Home");
}

#[test]
fn a_copter_entry_with_no_frame_labels_every_copter_picture() {
    let mut entry = Software {
        name: "ArduCopter V4.0".to_owned(),
        urlfmuv3: "http://x/Copter/stable/fmuv3/arducopter.apj".to_owned(),
        ..Software::default()
    };
    let given = display_name(&mut entry);
    assert_eq!(given.len(), 7);
    assert_eq!(given[0], (Picture::Octa, "ArduCopter V4.0 Octa".to_owned()));
    assert_eq!(given[6], (Picture::Quad, "ArduCopter V4.0 Quad".to_owned()));
    assert_eq!(
        entry.name, "ArduCopter V4.0",
        "the catch-all renames nothing"
    );
}

#[test]
fn a_history_entry_rewrites_every_url_through_get_url() {
    let (history, text) = nice_names().into_iter().next().unwrap();
    assert_eq!(text, "AC 4.0.3 AP 4.0.5 AS 4.0.0");
    let entry = Software {
        urlfmuv2: "http://firmware.ardupilot.org/Copter/stable/fmuv2/arducopter.apj".to_owned(),
        ..Software::default()
    };
    let rewritten = entry.for_history(&history);
    // An absolute URL against the history's is itself; an empty one is left alone.
    assert_eq!(rewritten.urlfmuv2, entry.urlfmuv2);
    assert_eq!(rewritten.url, "");
}

// --- a vehicle clicked on the legacy page ---------------------------------------------------------

fn copter_quad(fetch: &dyn Fetch) -> Software {
    let mut entry = list(fetch).remove(3);
    let _ = display_name(&mut entry);
    entry
}

/// A CubeOrange in its bootloader: the device rules name it, the URL is the entry's `fmuv2` one
/// with the board put in, the file is downloaded to `firmware.hex` and read - and the flow stops
/// where the px4 upload would reboot the board.
#[test]
fn a_cube_orange_downloads_its_build_and_stops_at_the_reboot() {
    let scratch = web("cube");
    let mirror = scratch.mirror();
    let entry = copter_quad(&mirror);
    let mut person = Person::answering(&[true]);
    let cube = DeviceInfo::new("CubeOrange-BL", r"USB\VID_2DAE&PID_1016");
    let reached = find_firmware(
        &mut cx(&mut person, &mirror, &scratch),
        "",
        &entry,
        "",
        &[cube],
        &[],
    );
    assert_eq!(
        person.asked[0].0,
        "Are you sure you want to upload ArduCopter V4.7.1 Quad?"
    );
    assert_eq!(person.asked[0].1, "Continue");
    assert_eq!(reached.board.as_deref(), Some("chbootloader CubeOrange"));
    assert_eq!(
        reached.url.as_deref(),
        Some("http://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj")
    );
    let saved = scratch.0.join("MissionPlannerRust").join("firmware.hex");
    assert_eq!(reached.file.as_deref(), Some(saved.as_path()));
    let size = std::fs::metadata(fixture("arducopter.apj")).unwrap().len();
    assert_eq!(reached.size, Some(size));
    let firmware = reached.firmware.as_ref().expect("read");
    assert_eq!(firmware.board_id, 140);
    assert_eq!(firmware.image_len, 3000);
    assert_eq!(reached.stop, Some(Stop::RebootToBootloader));
    assert!(person.shown.is_empty(), "{:?}", person.shown);
    let statuses: Vec<&str> = person.progress.iter().map(|(_, s)| s.as_str()).collect();
    assert_eq!(statuses.first(), Some(&"Detecting Board Version"));
    assert!(statuses.contains(&"Detected a chbootloader"));
    assert!(statuses.contains(&"Downloaded from internet"));
    assert_eq!(statuses.last(), Some(&"Reading Hex File"));
}

/// No: nothing else happens.
#[test]
fn no_to_are_you_sure_does_nothing() {
    let scratch = web("no");
    let mirror = scratch.mirror();
    let entry = copter_quad(&mirror);
    let mut person = Person::answering(&[false]);
    let reached = find_firmware(
        &mut cx(&mut person, &mirror, &scratch),
        "",
        &entry,
        "",
        &[],
        &[],
    );
    assert_eq!(reached, Reached::default());
    assert_eq!(person.asked.len(), 1);
    assert!(person.progress.is_empty());
}

/// An fmuv3 in its bootloader: the CubeBlack question, then `px4v3`'s URL - the `px4v3` build is
/// not there, so the `px4v2` one, and the ChibiOS build offered over it.
#[test]
fn an_fmuv3_asks_cubeblack_then_chibios() {
    let scratch = web("fmuv3");
    let mirror = scratch.mirror();
    let entry = copter_quad(&mirror);
    let mut person = Person::answering(&[true, false, true]);
    let board = DeviceInfo::new("fmuv3-BL", r"USB\VID_1209&PID_5740");
    let reached = find_firmware(
        &mut cx(&mut person, &mirror, &scratch),
        "",
        &entry,
        "",
        &[board],
        &[],
    );
    let asked: Vec<&str> = person.asked.iter().map(|(t, _, _)| t.as_str()).collect();
    assert_eq!(
        asked,
        [
            "Are you sure you want to upload ArduCopter V4.7.1 Quad?",
            "Is this a CubeBlack?",
            "Upload ChibiOS",
        ]
    );
    assert_eq!(reached.board.as_deref(), Some("px4v3"));
    assert_eq!(
        reached.url.as_deref(),
        Some("http://firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj")
    );
    assert_eq!(reached.stop, Some(Stop::RebootToBootloader));
}

/// A board the rules cannot name from the device list: "Is this a Linux board?", No, and the
/// STK500 probe on the port is where it stops.
#[test]
fn an_unknown_board_stops_at_the_stk500_probe() {
    let scratch = web("unknown");
    let mirror = scratch.mirror();
    let entry = copter_quad(&mirror);
    let mut person = Person::answering(&[true, false]);
    let reached = find_firmware(
        &mut cx(&mut person, &mirror, &scratch),
        "/dev/ttyACM0",
        &entry,
        "",
        &[],
        &[],
    );
    assert_eq!(person.asked[1].0, "Is this a Linux board?");
    assert_eq!(
        reached.stop,
        Some(Stop::OpenPort("/dev/ttyACM0".to_owned()))
    );
    assert!(reached.url.is_none());
}

/// A ChibiOS board with no build of that name: "No firmware available for this board!", then
/// the click's "Error uploading firmware".
#[test]
fn a_board_with_no_build_says_so_twice() {
    let scratch = web("nobuild");
    let mirror = scratch.mirror();
    let entry = copter_quad(&mirror);
    let mut person = Person::answering(&[true]);
    let board = DeviceInfo::new("MatekH743-BL", r"USB\VID_1209&PID_5740");
    let reached = find_firmware(
        &mut cx(&mut person, &mirror, &scratch),
        "",
        &entry,
        "",
        &[board],
        &[],
    );
    let shown: Vec<&str> = person.shown.iter().map(|(t, _)| t.as_str()).collect();
    assert_eq!(
        shown,
        [
            "No firmware available for this board!",
            "Error uploading firmware"
        ]
    );
    assert!(reached.stop.is_none());
}

// --- Load custom firmware -------------------------------------------------------------------------

#[test]
fn a_custom_apj_is_read_and_stops_at_the_reboot() {
    let scratch = web("custom");
    let mirror = scratch.mirror();
    let file = fixture("arducopter.apj");
    let mut person = Person::default();
    let legacy = custom_legacy(&mut cx(&mut person, &mirror, &scratch), "", &file, &[], &[]);
    assert_eq!(legacy.board.as_deref(), Some("px4v3"));
    assert_eq!(legacy.firmware.as_ref().map(|f| f.board_id), Some(140));
    assert_eq!(legacy.stop, Some(Stop::RebootToBootloader));
    let manifest = custom_manifest(&mut cx(&mut person, &mirror, &scratch), "", &file, &[], &[]);
    assert_eq!(manifest.board.as_deref(), Some("px4v2"));
    assert_eq!(manifest.stop, Some(Stop::RebootToBootloader));
    assert!(person.asked.is_empty());
}

#[test]
fn a_custom_file_that_is_not_firmware_says_so() {
    let scratch = web("bad");
    let mirror = scratch.mirror();
    let file = scratch.0.join("broken.apj");
    std::fs::write(&file, "{ not json").unwrap();
    let mut person = Person::default();
    let reached = custom_legacy(&mut cx(&mut person, &mirror, &scratch), "", &file, &[], &[]);
    assert!(reached.stop.is_none());
    assert!(
        person.shown[0]
            .0
            .starts_with("Error loading firmware file\n\n")
    );
    assert_eq!(person.shown[0].1, "Error");
}

#[test]
fn the_manifest_page_asks_about_dfu_for_hex_and_bin() {
    let scratch = web("dfu");
    let mirror = scratch.mirror();
    let hex = scratch.0.join("image.hex");
    std::fs::write(&hex, ":00000001FF\n").unwrap();
    // OK: through DFU, which is where it stops.
    let mut person = Person::answering(&[true]);
    let reached = custom_manifest(&mut cx(&mut person, &mirror, &scratch), "", &hex, &[], &[]);
    assert_eq!(reached.stop, Some(Stop::Dfu));
    assert_eq!(person.asked[0].0, "Do you want to upload this via DFU?");
    assert_eq!(person.asked[0].2, Buttons::OkCancel);
    // Cancel: no board, and "Cant detect your Board version".
    let mut person = Person::answering(&[false]);
    let reached = custom_manifest(&mut cx(&mut person, &mirror, &scratch), "", &hex, &[], &[]);
    assert!(reached.stop.is_none());
    assert_eq!(person.shown[0].0, flow::CANT_DETECT_BOARD_VERSION);
    // A .bin: the question, and nothing after it either way.
    let bin = scratch.0.join("image.bin");
    std::fs::write(&bin, [0u8; 4]).unwrap();
    let mut person = Person::answering(&[false]);
    let reached = custom_manifest(&mut cx(&mut person, &mirror, &scratch), "", &bin, &[], &[]);
    assert!(reached.stop.is_none());
    assert_eq!(person.asked[0].0, "Flashing image to 0x08000000");
    assert!(person.shown.is_empty());
    // A .dfu: straight to DFU.
    let dfu = scratch.0.join("image.dfu");
    std::fs::write(&dfu, [0u8; 4]).unwrap();
    let mut person = Person::default();
    let reached = custom_manifest(&mut cx(&mut person, &mirror, &scratch), "", &dfu, &[], &[]);
    assert_eq!(reached.stop, Some(Stop::Dfu));
}

/// An APM's HEX file is read on the legacy page's path; the port is where it stops.
#[test]
fn a_custom_hex_for_an_apm_is_read_then_stops_at_the_port() {
    let scratch = web("apm");
    let mirror = scratch.mirror();
    let hex = scratch.0.join("ArduCopter.hex");
    std::fs::write(&hex, ":0400000001020304F2\n:00000001FF\n").unwrap();
    // No device: "Is this a Linux board?" No, then the STK500 sync opens the port.
    let mut person = Person::default();
    let reached = custom_legacy(
        &mut cx(&mut person, &mirror, &scratch),
        "COM4",
        &hex,
        &[],
        &[],
    );
    assert_eq!(reached.stop, Some(Stop::OpenPort("COM4".to_owned())));
    // The upload itself, for a board already known.
    let mut person = Person::default();
    let mut reached = Reached::default();
    let answer = flow::upload_flash(
        &mut cx(&mut person, &mirror, &scratch),
        "COM4",
        &hex,
        Boards::B2560v2,
        &mut reached,
    );
    assert_eq!(answer, Err(Stop::ArduinoUpload("COM4".to_owned())));
    assert_eq!(reached.hex_len, Some(4));
}

// --- the manifest page's download ----------------------------------------------------------------

#[test]
fn look_for_port_downloads_to_a_temporary_file_then_stops_at_the_reboot() {
    let scratch = web("download");
    let mirror = scratch.mirror();
    let mut person = Person::default();
    let reached = download_and_flash(
        &mut cx(&mut person, &mirror, &scratch),
        "https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj",
        "/dev/ttyACM0",
    );
    let file = reached.file.clone().expect("downloaded");
    assert!(file.starts_with(scratch.0.join("tmp")));
    assert_eq!(
        std::fs::read(&file).unwrap(),
        std::fs::read(fixture("arducopter.apj")).unwrap()
    );
    assert_eq!(reached.firmware.as_ref().map(|f| f.board_id), Some(140));
    assert_eq!(reached.stop, Some(Stop::RebootToBootloader));
    let statuses: Vec<&(i32, String)> = person.progress.iter().collect();
    assert_eq!(statuses[0], &(0, "Downloading from Internet".to_owned()));
    assert!(statuses.contains(&&(100, "Downloaded from internet".to_owned())));

    // Nothing there: "Failed download", and nothing read.
    let mut person = Person::default();
    let reached = download_and_flash(
        &mut cx(&mut person, &mirror, &scratch),
        "https://firmware.ardupilot.org/Copter/stable/Nowhere/arducopter.apj",
        "",
    );
    assert!(reached.file.is_none());
    assert_eq!(
        person.shown,
        [("Failed download".to_owned(), "Error".to_owned())]
    );
}

#[test]
fn the_mirror_serves_only_what_it_holds() {
    let scratch = web("mirror");
    let mirror = scratch.mirror();
    let url = "https://firmware.ardupilot.org/Copter/stable/fmuv3/git-version.txt";
    assert_eq!(mirror.exists(url), Ok(true));
    assert_eq!(
        mirror.exists("http://firmware.ardupilot.org/Copter/stable/fmuv3/git-version.txt"),
        Ok(true),
        "http and https alike"
    );
    assert_eq!(
        mirror.exists("https://firmware.ardupilot.org/none"),
        Ok(false)
    );
    assert!(mirror.path_of("https://host/../etc/passwd").is_none());
    assert!(mirror.get("ftp://host/a").is_err());
}
