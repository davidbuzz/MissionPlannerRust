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

//! The firmware catalogue against an excerpt of the real manifest.
//!
//! `testdata/firmware/manifest.json.gz` is 240 records cut from ArduPilot's own
//! `manifest.json.gz` by `testdata/firmware/trim.py` - gzipped, as it is downloaded, and in the
//! manifest's order; `periph-manifest.json` is CubePilot's peripheral manifest whole. Nothing here
//! touches the network: the fetch is a table of URLs to fixture bytes, so the order in which
//! `GetList` asks for things is itself under test.
//!
//! The expected answers were worked out from the fixture independently of this code, with the
//! C#'s rules applied by hand in Python over the same JSON.

#![allow(clippy::indexing_slicing, clippy::expect_used, clippy::unwrap_used)]

use std::cell::RefCell;
use std::collections::HashMap;
use std::path::PathBuf;

use mp_firmware::detect::DeviceInfo;
use mp_firmware::manifest::{
    Fetch, MANIFEST_URL, MIRROR_URL, Manifest, ManifestError, MavType, NO_FIRMWARE, OVERRIDE_ENV,
    Outcome, PERIPH_URL, ReleaseType, Version, get_list, icon_name, load,
};
use mp_transport::PortInfo;

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/firmware")
        .join(name)
}

fn fixture_bytes(name: &str) -> Vec<u8> {
    let path = fixture_path(name);
    std::fs::read(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
}

fn fixture() -> Manifest {
    Manifest::decode(&fixture_bytes("manifest.json.gz"), true).expect("the fixture parses")
}

/// A device as Mission Planner lists one: a product string and a hardware id.
fn device(board: &str, hardwareid: &str) -> DeviceInfo {
    DeviceInfo::new(board, hardwareid)
}

/// A network made of fixtures, which remembers what it was asked for.
struct Table {
    answers: HashMap<&'static str, Vec<u8>>,
    asked: RefCell<Vec<String>>,
}

impl Table {
    fn new(answers: &[(&'static str, Vec<u8>)]) -> Self {
        Self {
            answers: answers.iter().cloned().collect(),
            asked: RefCell::new(Vec::new()),
        }
    }
}

impl Fetch for Table {
    fn get(&self, url: &str) -> Result<Vec<u8>, String> {
        self.asked.borrow_mut().push(url.to_owned());
        self.answers
            .get(url)
            .cloned()
            .ok_or_else(|| "404 Not Found".to_owned())
    }
}

// --- reading it ---------------------------------------------------------------------------------

#[test]
fn the_fixture_parses_into_the_csharps_records() {
    let manifest = fixture();
    assert_eq!(manifest.firmware.len(), 240);
    assert_eq!(manifest.format_version, Version::parse("1.0.0"));

    // The first record, field by field: a CubeOrange's AntennaTracker, the manifest's order.
    let first = &manifest.firmware[0];
    assert_eq!(first.mav_type.as_deref(), Some("ANTENNA_TRACKER"));
    assert_eq!(first.vehicle_type.as_deref(), Some("AntennaTracker"));
    assert_eq!(first.mav_firmware_version_type.as_deref(), Some("OFFICIAL"));
    assert_eq!(first.mav_firmware_version, Version::parse("4.7.1"));
    assert_eq!(first.mav_firmware_version_str.as_deref(), Some("V4.7.1"));
    assert_eq!(
        (
            first.mav_firmware_version_major,
            first.mav_firmware_version_minor,
            first.mav_firmware_version_patch
        ),
        (4, 7, 1),
        "the manifest writes these as strings; Newtonsoft reads them as longs"
    );
    assert_eq!(first.mav_autopilot.as_deref(), Some("ARDUPILOTMEGA"));

    // A record with a board id carries its bootloader strings and USB ids; one without has none.
    let cube = manifest
        .firmware
        .iter()
        .find(|a| a.platform.as_deref() == Some("CubeOrange") && a.board_id == 140)
        .expect("a CubeOrange apj");
    assert_eq!(cube.bootloader_str, ["CubeOrange-BL"]);
    assert_eq!(cube.usbid, ["0x2dae/0x1016"]);
    assert_eq!(cube.format.as_deref(), Some("apj"));
    let navio = manifest
        .firmware
        .iter()
        .find(|a| a.platform.as_deref() == Some("navio2"))
        .expect("navio2");
    assert_eq!(navio.board_id, 0, "absent board_id is the long's default");
    assert!(navio.usbid.is_empty() && navio.bootloader_str.is_empty());
}

#[test]
fn the_manifest_is_gzip_and_the_peripheral_manifest_is_not() {
    assert!(Manifest::decode(&fixture_bytes("manifest.json.gz"), false).is_err());
    let periph = Manifest::decode(&fixture_bytes("periph-manifest.json"), false).expect("plain");
    assert_eq!(periph.firmware.len(), 2);
    assert_eq!(
        periph.firmware[0].firmware_name.as_deref(),
        Some("Here4_FW.bin")
    );
    assert!(matches!(
        Manifest::decode(&fixture_bytes("periph-manifest.json"), true),
        Err(ManifestError::Gzip(_))
    ));
}

// --- fetching it --------------------------------------------------------------------------------

#[test]
fn get_list_fetches_once_and_appends_the_peripheral_manifest() {
    let table = Table::new(&[
        (MANIFEST_URL, fixture_bytes("manifest.json.gz")),
        (PERIPH_URL, fixture_bytes("periph-manifest.json")),
    ]);
    let mut manifest = None;
    assert!(get_list(&mut manifest, MANIFEST_URL, false, &table).is_empty());
    let held = manifest.as_ref().expect("fetched");
    assert_eq!(
        held.firmware.len(),
        242,
        "240 records and CubePilot's 2 after them"
    );
    assert_eq!(
        held.firmware[240].platform.as_deref(),
        Some("com.cubepilot.here4")
    );
    assert_eq!(*table.asked.borrow(), [MANIFEST_URL, PERIPH_URL]);

    // Held for the life of the process: asking again asks nothing.
    assert!(get_list(&mut manifest, MANIFEST_URL, false, &table).is_empty());
    assert_eq!(table.asked.borrow().len(), 2);
    // Unless forced, which nothing in Mission Planner does.
    assert!(get_list(&mut manifest, MANIFEST_URL, true, &table).is_empty());
    assert_eq!(table.asked.borrow().len(), 4);
    assert_eq!(manifest.expect("still there").firmware.len(), 242);
}

#[test]
fn the_page_asks_the_mirror_first_and_ardupilot_org_only_when_it_fails() {
    let bytes = fixture_bytes("manifest.json.gz");

    let mirror_up = Table::new(&[(MIRROR_URL, bytes.clone())]);
    let mut manifest = None;
    let errors = load(&mut manifest, &mirror_up);
    assert_eq!(*mirror_up.asked.borrow(), [MIRROR_URL, PERIPH_URL]);
    assert_eq!(
        errors.len(),
        1,
        "the peripheral manifest was not there: {errors:?}"
    );
    assert_eq!(manifest.expect("from the mirror").firmware.len(), 240);

    let mirror_down = Table::new(&[(MANIFEST_URL, bytes)]);
    let mut manifest = None;
    let errors = load(&mut manifest, &mirror_down);
    assert_eq!(
        *mirror_down.asked.borrow(),
        [MIRROR_URL, MANIFEST_URL, PERIPH_URL]
    );
    assert_eq!(errors.len(), 2, "{errors:?}");
    assert_eq!(manifest.expect("from ardupilot.org").firmware.len(), 240);

    let offline = Table::new(&[]);
    let mut manifest = None;
    assert_eq!(load(&mut manifest, &offline).len(), 2);
    assert!(manifest.is_none(), "nothing on disk to fall back on");
}

#[test]
fn a_failed_download_keeps_what_was_held() {
    let table = Table::new(&[(MANIFEST_URL, b"not gzip".to_vec())]);
    let mut manifest = Some(fixture());
    let errors = get_list(&mut manifest, MANIFEST_URL, true, &table);
    assert!(
        matches!(errors.as_slice(), [ManifestError::Gzip(_)]),
        "{errors:?}"
    );
    assert_eq!(manifest.expect("kept").firmware.len(), 240);
}

// --- what the page labels ----------------------------------------------------------------------

#[test]
fn each_vehicle_is_labelled_with_its_newest_version_of_the_release() {
    let manifest = fixture();
    let label = |release, mav_type| {
        manifest
            .newest(release, mav_type)
            .map(icon_name)
            .unwrap_or_default()
    };
    assert_eq!(
        label(ReleaseType::Official, MavType::AntennaTracker),
        "AntennaTracker V4.7.1 OFFICIAL",
        "not the apm2's 0.7.2, which is also OFFICIAL"
    );
    assert_eq!(
        label(ReleaseType::Official, MavType::Copter),
        "Copter V4.7.1 OFFICIAL",
        "not the apm2-quad's 3.2.1"
    );
    assert_eq!(
        label(ReleaseType::Official, MavType::GroundRover),
        "Rover V4.7.1 OFFICIAL"
    );
    // The manifest calls a helicopter's vehicle "Copter", and so does the label.
    assert_eq!(
        label(ReleaseType::Official, MavType::Helicopter),
        "Copter V4.7.1 OFFICIAL"
    );
    assert_eq!(
        label(ReleaseType::Beta, MavType::Submarine),
        "Sub V4.7.1 BETA"
    );
    assert_eq!(
        label(ReleaseType::Dev, MavType::FixedWing),
        "Plane V4.8.0-dev DEV"
    );

    // The version string wins; without one it is the version.
    let mut record = manifest
        .newest(ReleaseType::Dev, MavType::Copter)
        .unwrap()
        .clone();
    record.mav_firmware_version_str = None;
    assert_eq!(icon_name(&record), "Copter 4.8.0 DEV");
}

#[test]
fn get_release_keeps_the_newest_records_ordered_by_format() {
    let manifest = fixture();
    let release = manifest.release(ReleaseType::Official);
    // The vehicles in the order they first appear in the manifest.
    let mut order: Vec<&str> = Vec::new();
    for record in &release {
        let mav_type = record.mav_type.as_deref().unwrap();
        if !order.contains(&mav_type) {
            order.push(mav_type);
        }
    }
    assert_eq!(
        order,
        [
            "ANTENNA_TRACKER",
            "Copter",
            "HELICOPTER",
            "FIXED_WING",
            "GROUND_ROVER",
            "SUBMARINE",
            "CAN_PERIPHERAL"
        ]
    );
    let copter: Vec<&str> = release
        .iter()
        .filter(|a| a.mav_type.as_deref() == Some("Copter"))
        .map(|a| a.format.as_deref().unwrap())
        .collect();
    let expected: Vec<&str> = [("abin", 4), ("apj", 9), ("elf", 9), ("ELF", 1), ("hex", 8)]
        .iter()
        .flat_map(|(format, count)| std::iter::repeat_n(*format, *count))
        .collect();
    assert_eq!(
        copter, expected,
        "31 of the 32: the 3.2.1 is not the newest"
    );
    assert!(
        release
            .iter()
            .all(|a| a.mav_firmware_version_type.as_deref() == Some("OFFICIAL"))
    );
    // STABLE-4.7.1 is a release type no RELEASE_TYPES value names.
    assert!(
        ReleaseType::ALL
            .iter()
            .flat_map(|r| manifest.release(*r))
            .all(|a| a.mav_firmware_version_type.as_deref() != Some("STABLE-4.7.1"))
    );

    let newest = manifest.release_newest(ReleaseType::Official);
    assert_eq!(newest.len(), 7, "one per vehicle");
    assert_eq!(
        newest[1].url.as_deref(),
        Some("https://firmware.ardupilot.org/Copter/stable/MatekH743/arducopter.abin"),
        "the first of the Copter records by format"
    );
}

// --- which board --------------------------------------------------------------------------------

#[test]
fn board_ids_come_from_the_product_string_first() {
    let manifest = fixture();
    let ids = |board: &str| manifest.board_ids(&device(board, ""), true);
    // A bootloader string, a platform, and a platform in another case.
    assert_eq!(ids("CubeOrange-BL"), Some(vec![140]));
    assert_eq!(ids("CubeOrange"), Some(vec![140]));
    assert_eq!(ids("cubeorange"), Some(vec![140]));
    assert_eq!(ids("fmuv3-BL"), Some(vec![9]));
    // The old px4 bootloader's string, shared by every board 9.
    assert_eq!(ids("PX4 BL FMU v2.x"), Some(vec![9]));
    // A CubeRed is two processors: "CubeRed" is both, and so is either one's name with -BL
    // read as primary or secondary, and the secondary's own name.
    assert_eq!(ids("CubeRed"), Some(vec![1069, 1070]));
    assert_eq!(ids("CubeRed-BL"), Some(vec![1069, 1070]));
    assert_eq!(ids("CubeRedSecondary"), Some(vec![1069, 1070]));
    // A Linux board has no board id to flash to.
    assert_eq!(ids("navio2"), None);
    assert_eq!(
        manifest.board_ids(&device("navio2", ""), false),
        Some(vec![0])
    );
    assert_eq!(ids("FT232R USB UART"), None);
}

#[test]
fn board_ids_fall_back_to_the_vid_and_pid_only_with_a_trailing_ampersand() {
    let manifest = fixture();
    // Windows' id, with its &REV_: the pattern matches, and every board 9 lists 0x26AC/0x0011.
    let windows = device("unknown", r"USB\VID_26AC&PID_0011&REV_0200");
    assert_eq!(manifest.board_ids(&windows, true), Some(vec![9]));
    // The id Mission Planner builds on Linux has no trailing & and never matches.
    let linux = device("unknown", r"USB\VID_26ac&PID_0011");
    assert_eq!(manifest.board_ids(&linux, true), None);
    // No hardware id at all stops before the pattern.
    let none = DeviceInfo {
        board: Some("unknown".to_owned()),
        ..DeviceInfo::default()
    };
    assert_eq!(manifest.board_ids(&none, true), None);
}

#[test]
fn a_port_as_it_is_enumerated_names_its_board() {
    // The path the page takes: the port list, through detect.rs's DeviceInfo.
    let port = PortInfo {
        name: "/dev/ttyACM0".to_owned(),
        vid: Some(0x2dae),
        pid: Some(0x1016),
        serial_number: None,
        manufacturer: Some("CubePilot".to_owned()),
        product: Some("CubeOrange-BL".to_owned()),
        hardware_id: None,
        description: None,
    };
    let device = DeviceInfo::from_port(&port).expect("a USB port");
    assert_eq!(fixture().board_ids(&device, true), Some(vec![140]));
}

/// The same board on Windows, as the port list gives it with Windows' own record over the
/// crate's (mp-transport's win32.rs, `Win32DeviceMgmt`'s fields): named by its bus-reported
/// "CubeOrange"; and, where Windows reported no name and the product is the crate's friendly
/// name, found by `SPDRP_HARDWAREID` - whose `&REV_0200&MI_00` after the PID is what the C#'s
/// `VID_..&PID_..&` pattern needs, and what the id built from the VID and PID alone lacks. That
/// lack was the owner's report of 2026-09-26: "none of 2 usb devices names a board" on Windows.
/// `// C#: Utilities/Win32DeviceMgnt.cs:462-549; ExtLibs/ArduPilot/APFirmware.cs (GetBoardID)`
#[test]
fn a_windows_port_names_its_board_by_its_bus_name_or_its_hardware_id() {
    let windows = |product: &str| PortInfo {
        name: "COM4".to_owned(),
        vid: Some(0x2dae),
        pid: Some(0x1016),
        serial_number: Some("19002E000F51303339323537".to_owned()),
        manufacturer: Some("Microsoft".to_owned()),
        product: Some(product.to_owned()),
        hardware_id: Some(r"USB\VID_2DAE&PID_1016&REV_0200&MI_00".to_owned()),
        description: Some("USB Serial Device".to_owned()),
    };
    let manifest = fixture();
    let named = DeviceInfo::from_port(&windows("CubeOrange")).expect("a USB port");
    assert_eq!(named.board.as_deref(), Some("CubeOrange"));
    assert_eq!(named.description.as_deref(), Some("USB Serial Device"));
    assert_eq!(named.hardwareid.as_deref(), Some(r"USB\VID_2DAE&PID_1016&REV_0200&MI_00"));
    assert_eq!(manifest.board_ids(&named, true), Some(vec![140]));

    let unnamed = DeviceInfo::from_port(&windows("USB Serial Device (COM4)")).expect("a USB port");
    let by_id = manifest.board_ids(&unnamed, true).expect("found by the hardware id");
    assert!(by_id.contains(&140), "{by_id:?}");

    // Without Windows' record - the id built from the VID and PID - the pattern finds nothing,
    // which is what the page said on Windows before.
    let crate_only = PortInfo {
        hardware_id: None,
        description: None,
        ..windows("USB Serial Device (COM4)")
    };
    let before = DeviceInfo::from_port(&crate_only).expect("a USB port");
    assert_eq!(manifest.board_ids(&before, true), None);
}

// --- which firmware -----------------------------------------------------------------------------

#[test]
fn a_cube_orange_gets_its_own_platform_of_three() {
    let manifest = fixture();
    let devices = [
        device("FT232R USB UART", r"USB\VID_0403&PID_6001"),
        device("CubeOrange-BL", r"USB\VID_2DAE&PID_1016"),
    ];
    let lookup = manifest
        .look_for_port(
            &devices,
            None,
            MavType::Copter,
            ReleaseType::Official,
            false,
        )
        .expect("the second device has a board id");
    assert_eq!(lookup.device.board.as_deref(), Some("CubeOrange-BL"));
    assert_eq!(lookup.board_ids, [140]);
    let platforms: Vec<&str> = lookup
        .items
        .iter()
        .map(|a| a.platform.as_deref().unwrap())
        .collect();
    assert_eq!(
        platforms,
        [
            "CubeOrange",
            "CubeOrange-SimOnHardWare",
            "CubeOrange-bdshot"
        ]
    );
    // Three records open FirmwareSelection, whose Platform picker takes "CubeOrange".
    let Outcome::Choose(selection) = lookup.outcome() else {
        panic!("three records are a choice");
    };
    assert_eq!(selection.platform.as_deref(), Some("CubeOrange"));
    assert_eq!(
        selection.results(),
        ["https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj"]
    );
    let chosen = lookup.chosen().expect("one left, selected");
    assert_eq!(chosen.mav_firmware_version, Version::parse("4.7.1"));
    assert_eq!(
        chosen.url.as_deref(),
        Some("https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj")
    );
}

#[test]
fn the_release_and_the_vehicle_choose_the_url() {
    let manifest = fixture();
    let devices = [device("CubeOrange-BL", r"USB\VID_2DAE&PID_1016")];
    let url = |mav_type, release| {
        let lookup = manifest
            .look_for_port(&devices, None, mav_type, release, false)
            .expect("a board");
        lookup
            .chosen()
            .and_then(|a| a.url.clone())
            .unwrap_or_default()
    };
    // One record each, taken without a dialog.
    assert_eq!(
        url(MavType::Copter, ReleaseType::Beta),
        "https://firmware.ardupilot.org/Copter/beta/CubeOrange/arducopter.apj"
    );
    assert_eq!(
        url(MavType::Copter, ReleaseType::Dev),
        "https://firmware.ardupilot.org/Copter/latest/CubeOrange/arducopter.apj"
    );
    assert_eq!(
        url(MavType::Helicopter, ReleaseType::Official),
        "https://firmware.ardupilot.org/Copter/stable/CubeOrange-heli/arducopter-heli.apj"
    );
    assert_eq!(
        url(MavType::Submarine, ReleaseType::Official),
        "https://firmware.ardupilot.org/Sub/stable/CubeOrange/ardusub.apj"
    );
    let lookup = manifest
        .look_for_port(&devices, None, MavType::Copter, ReleaseType::Dev, false)
        .unwrap();
    assert!(matches!(lookup.outcome(), Outcome::One(_)));
    assert_eq!(
        lookup.chosen().unwrap().mav_firmware_version_str.as_deref(),
        Some("V4.8.0-dev")
    );
}

#[test]
fn a_shared_bootloader_string_leaves_the_choice_to_the_operator() {
    let manifest = fixture();
    let devices = [device("PX4 BL FMU v2.x", r"USB\VID_26AC&PID_0011")];
    let lookup = manifest
        .look_for_port(
            &devices,
            None,
            MavType::Copter,
            ReleaseType::Official,
            false,
        )
        .unwrap();
    let Outcome::Choose(selection) = lookup.outcome() else {
        panic!("three boards share board id 9");
    };
    assert_eq!(selection.platform, None, "no platform is called that");
    assert_eq!(
        selection.results(),
        [
            "https://firmware.ardupilot.org/Copter/stable/Pixhawk1/arducopter.apj",
            "https://firmware.ardupilot.org/Copter/stable/CubeBlack/arducopter.apj",
            "https://firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj",
        ]
    );
    assert_eq!(lookup.chosen(), None);

    // The same board with its own bootloader's name is no choice at all.
    let fmuv3 = [device("fmuv3-BL", r"USB\VID_1209&PID_5741")];
    let lookup = manifest
        .look_for_port(&fmuv3, None, MavType::Copter, ReleaseType::Official, false)
        .unwrap();
    assert_eq!(
        lookup.chosen().and_then(|a| a.url.as_deref()),
        Some("https://firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj")
    );
}

#[test]
fn no_board_and_no_firmware_are_the_csharps_two_messages() {
    let manifest = fixture();
    let nothing = [device("FT232R USB UART", r"USB\VID_0403&PID_6001")];
    assert_eq!(
        manifest.look_for_port(
            &nothing,
            None,
            MavType::Copter,
            ReleaseType::Official,
            false
        ),
        None,
        "Failed to detect port to upload to"
    );
    // A CAN node has a board id and no copter firmware.
    let periph = [device("f103-GPS-BL", r"USB\VID_1209&PID_5741")];
    let lookup = manifest
        .look_for_port(&periph, None, MavType::Copter, ReleaseType::Official, false)
        .unwrap();
    assert_eq!(lookup.board_ids, [1000]);
    assert_eq!(lookup.outcome(), Outcome::NoFirmware);
    assert_eq!(NO_FIRMWARE, "No firmware available for this board!");
}

#[test]
fn a_board_id_read_from_the_bootloader_replaces_the_lookup() {
    let manifest = fixture();
    let blank = [DeviceInfo::default()];
    let lookup = manifest
        .look_for_port(
            &blank,
            Some(140),
            MavType::Copter,
            ReleaseType::Official,
            false,
        )
        .unwrap();
    assert_eq!(lookup.board_ids, [140]);
    assert_eq!(lookup.items.len(), 3);
    assert_eq!(
        lookup.chosen(),
        None,
        "no product string to pick a platform by"
    );
    // Zero is no board.
    assert_eq!(
        manifest.look_for_port(
            &blank,
            Some(0),
            MavType::Copter,
            ReleaseType::Official,
            false
        ),
        None
    );
}

#[test]
fn all_options_offers_the_whole_manifest() {
    let manifest = fixture();
    let lookup = manifest
        .look_for_port(&[], None, MavType::Copter, ReleaseType::Official, true)
        .expect("the blank device All Options adds");
    assert_eq!(lookup.items.len(), 240);
    let Outcome::Choose(selection) = lookup.outcome() else {
        panic!("a choice");
    };
    assert_eq!(
        selection.results(),
        ["To many options - apply more filters - 240"],
        "the C#'s spelling"
    );
    assert_eq!(lookup.chosen(), None);
}

#[test]
fn get_options_narrows_by_usb_id_when_it_can() {
    let manifest = fixture();
    let all = manifest
        .options(
            &device("", ""),
            Some(ReleaseType::Official),
            Some(MavType::Copter),
        )
        .unwrap();
    assert_eq!(all.len(), 32);
    let cube = manifest
        .options(
            &device("anything", r"USB\VID_2DAE&PID_1016&REV_0200"),
            Some(ReleaseType::Official),
            Some(MavType::Copter),
        )
        .unwrap();
    assert_eq!(cube.len(), 3, "the three platforms of board 140");
    // An id no record carries narrows nothing.
    let other = manifest
        .options(&device("", r"USB\VID_0403&PID_6001&REV_0600"), None, None)
        .unwrap();
    assert_eq!(other.len(), 240);
    // A null hardware id throws in the C#.
    assert_eq!(manifest.options(&DeviceInfo::default(), None, None), None);
}

// --- the real thing -----------------------------------------------------------------------------

/// The whole manifest, when one is given: `MP_FIRMWARE_MANIFEST=<manifest.json.gz>`.
///
/// Mission Planner keeps the manifest in memory only, so there is no copy of it under
/// `~/.local/share/Mission Planner` to read; a downloaded one has to be named. Skipped, saying so,
/// when none is.
#[test]
fn the_real_manifest_parses_when_one_is_named() {
    let Some(path) = std::env::var_os(OVERRIDE_ENV) else {
        eprintln!("skipped: {OVERRIDE_ENV} names no manifest.json.gz to read");
        return;
    };
    let bytes = std::fs::read(&path).expect("readable");
    let manifest = Manifest::decode(&bytes, true).expect("the real manifest parses");
    assert!(
        manifest.firmware.len() > 10_000,
        "{}",
        manifest.firmware.len()
    );
    for release in ReleaseType::ALL {
        for mav_type in MavType::ALL {
            assert!(
                manifest.newest(release, mav_type).is_some(),
                "{release} has no {mav_type}"
            );
        }
    }
    let cube = [device("CubeOrange-BL", r"USB\VID_2DAE&PID_1016")];
    let lookup = manifest
        .look_for_port(&cube, None, MavType::Copter, ReleaseType::Official, false)
        .expect("a CubeOrange");
    assert_eq!(
        lookup.chosen().and_then(|a| a.url.as_deref()),
        Some("https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj")
    );
    eprintln!(
        "{}: {} records, OFFICIAL Copter {}",
        PathBuf::from(path).display(),
        manifest.firmware.len(),
        manifest
            .newest(ReleaseType::Official, MavType::Copter)
            .map(icon_name)
            .unwrap_or_default()
    );
}
