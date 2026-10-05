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

//! The C#'s four real plugins - AnonymizeBinlog, TerrainMaker, OpenDroneID, Dowding - ported to
//! the world, each driven through the host against a scripted application.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

mod common;

use common::load;
use mp_plugin_host::{Area, CsValue, DialogResult, MapMenu, OpenedFile};

// ---- AnonymizeBinlog ----------------------------------------------------------------------

/// A `FMT` message: type, length, name, format, columns.
fn fmt(type_id: u8, length: u8, name: &str, format: &str, columns: &str) -> Vec<u8> {
    let mut out = vec![0xA3, 0x95, 128, type_id, length];
    let field = |text: &str, size: usize| {
        let mut bytes = text.as_bytes().to_vec();
        bytes.resize(size, 0);
        bytes
    };
    out.extend(field(name, 4));
    out.extend(field(format, 16));
    out.extend(field(columns, 64));
    assert_eq!(out.len(), 89);
    out
}

/// A `GPS` message of type 130: `QLLf` - time, lat, lng, alt.
fn gps(lat: i32, lng: i32, alt: f32) -> Vec<u8> {
    let mut out = vec![0xA3, 0x95, 130];
    out.extend(1_000_000u64.to_le_bytes());
    out.extend(lat.to_le_bytes());
    out.extend(lng.to_le_bytes());
    out.extend(alt.to_le_bytes());
    out
}

fn read_i32(data: &[u8], at: usize) -> i32 {
    i32::from_le_bytes(data[at..at + 4].try_into().unwrap())
}

/// A log of a `FMT` for `GPS` and two fixes; the second fix starts at 89 + 23.
fn log() -> Vec<u8> {
    let mut data = fmt(130, 23, "GPS", "QLLf", "TimeUS,Lat,Lng,Alt");
    data.extend(gps(-353_632_000, 1_491_652_000, 584.0));
    data.extend(gps(-353_633_000, 1_491_653_000, 585.0));
    data
}

/// The whole flow: the log opened, the two offsets asked, every fix moved by them (degrees times
/// 1e7), the altitude untouched, the copy saved as `<name>_anon.bin`, and the C#'s summary.
#[test]
fn anonymizebinlog_moves_every_fix() {
    let Some((mut plugin, script)) = load("anonymizebinlog") else {
        return;
    };
    assert_eq!(plugin.info().name, "Anonymize Binlog");
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    assert_eq!(
        script.record().menu,
        [(MapMenu::FlightData, None, "Anonymize Bin Log...".to_owned())]
    );
    let data = log();
    script.with(|r| {
        r.open_answers.push_back(Some(OpenedFile {
            name: "flight.bin".to_owned(),
            data: data.clone(),
        }));
        r.input_answers.push_back(Some("0.5".to_owned()));
        r.input_answers.push_back(Some("-0.25".to_owned()));
    });
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    let r = script.record();
    assert_eq!(r.inputs[0].1, "Latitude Offset (degrees, blank = random)");
    let (path, out) = &r.saved[0];
    assert_eq!(path, "/saved/flight_anon.bin");
    assert_eq!(out.len(), data.len());
    let first = 89 + 3 + 8;
    assert_eq!(read_i32(out, first), -353_632_000 + 5_000_000);
    assert_eq!(read_i32(out, first + 4), 1_491_652_000 - 2_500_000);
    assert_eq!(&out[first + 8..first + 12], &584.0f32.to_le_bytes());
    let second = first + 23;
    assert_eq!(read_i32(out, second), -353_633_000 + 5_000_000);
    // The format message itself is untouched.
    assert_eq!(&out[..89], &data[..89]);
    let (text, caption, _) = r.messages.last().unwrap();
    assert_eq!(caption, "Anonymize Bin Log");
    assert_eq!(
        text,
        "Anonymization complete.\n\nOutput: /saved/flight_anon.bin\n\nLat offset: +0.500000\u{b0}\nLon offset: -0.250000\u{b0}"
    );
}

/// With `FMTU`, the units say which fields are coordinates, whatever their names; blank offsets
/// are random, 0.5 to 2 degrees either way, and the same log gives the same ones.
#[test]
fn anonymizebinlog_reads_fmtu_and_randomises_blank_offsets() {
    let Some((mut plugin, script)) = load("anonymizebinlog") else {
        return;
    };
    plugin.loaded().unwrap();
    // Type 131, "POS" with columns that no name rule knows, and an FMTU (type 132) giving their
    // units: '#' instance, 'D' latitude, 'U' longitude.
    let mut data = fmt(131, 11, "POS", "ii", "Xa,Xb");
    data.extend(fmt(
        132,
        44,
        "FMTU",
        "QBNN",
        "TimeUS,FmtType,UnitIds,MultIds",
    ));
    let mut fmtu = vec![0xA3, 0x95, 132];
    fmtu.extend(0u64.to_le_bytes());
    fmtu.push(131);
    let mut units = b"DU".to_vec();
    units.resize(16, 0);
    fmtu.extend(&units);
    fmtu.extend([0u8; 16]);
    data.extend(&fmtu);
    let pos = data.len();
    data.extend([0xA3, 0x95, 131]);
    data.extend(100_000_000i32.to_le_bytes());
    data.extend(200_000_000i32.to_le_bytes());
    for _ in 0..2 {
        script.with(|r| {
            r.open_answers.push_back(Some(OpenedFile {
                name: "a.bin".to_owned(),
                data: data.clone(),
            }));
            r.input_answers.push_back(Some(String::new()));
            r.input_answers.push_back(Some("  ".to_owned()));
        });
        plugin.menu_click(0, 0.0, 0.0).unwrap();
    }
    let r = script.record();
    let lat = |out: &[u8]| f64::from(read_i32(out, pos + 3) - 100_000_000) / 1e7;
    let lng = |out: &[u8]| f64::from(read_i32(out, pos + 7) - 200_000_000) / 1e7;
    let (one, two) = (&r.saved[0].1, &r.saved[1].1);
    for offset in [lat(one), lng(one)] {
        assert!((0.5..=2.0).contains(&offset.abs()), "{offset}");
    }
    assert_eq!((lat(one), lng(one)), (lat(two), lng(two)));
}

/// A log without coordinates is the C#'s error, and nothing is saved.
#[test]
fn anonymizebinlog_refuses_a_log_without_coordinates() {
    let Some((mut plugin, script)) = load("anonymizebinlog") else {
        return;
    };
    plugin.loaded().unwrap();
    script.with(|r| {
        r.open_answers.push_back(Some(OpenedFile {
            name: "empty.bin".to_owned(),
            data: fmt(140, 7, "BAT", "f", "Volt"),
        }));
    });
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    let r = script.record();
    assert!(r.saved.is_empty());
    assert_eq!(
        r.messages.last().unwrap().0,
        "Error: No coordinate fields found in log file."
    );
}

// ---- TerrainMaker -------------------------------------------------------------------------

/// `crc16` as the C#'s table gives it.
fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        let mut x = (crc >> 8) ^ u16::from(b);
        let mut t: u16 = x << 8;
        for _ in 0..8 {
            t = if t & 0x8000 != 0 {
                (t << 1) ^ 0x1021
            } else {
                t << 1
            };
        }
        x = t;
        crc = (crc << 8) ^ x;
    }
    crc
}

/// No selected area, Yes to the view, spacing 100: one file for the one degree the view covers,
/// every block's heights the terrain's, every 4 x 4 grid marked valid, and each block's CRC
/// right.
#[test]
fn terrainmaker_writes_a_dat_file() {
    // The check value of CRC-16/XMODEM, which the C#'s table and zero start are.
    assert_eq!(crc16(b"123456789"), 0x31C3);
    let Some((mut plugin, script)) = load("terrainmaker") else {
        return;
    };
    assert_eq!(plugin.info().version, "2.0");
    plugin.loaded().unwrap();
    assert_eq!(
        script.record().menu,
        [(MapMenu::FlightPlanner, None, "Make Terrain DAT".to_owned())]
    );
    script.with(|r| {
        r.view_area = Some(Area {
            top: -35.2,
            bottom: -35.8,
            left: 149.1,
            right: 149.6,
        });
        r.message_answers.push_back(DialogResult::Yes);
        r.input_answers.push_back(Some("100".to_owned()));
        r.terrain = Some(577.4);
    });
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    let r = script.record();
    assert_eq!(r.inputs[0].1, "Enter the grid spacing in meters (5-100).");
    assert_eq!(r.written.len(), 1, "{:?}", r.messages);
    let (path, data) = &r.written[0];
    assert_eq!(path, "/user/TerrainData/S36E149.DAT");
    // The C# packs 2047 bytes a block.
    assert_eq!(data.len() % 2047, 0);
    let blocks = data.len() / 2047;
    assert!(blocks > 100, "{blocks}");
    assert_eq!(r.terrain_reads, blocks * 28 * 32);
    for block in data.chunks(2047).take(3) {
        let bitmap = u64::from_le_bytes(block[0..8].try_into().unwrap());
        assert_eq!(bitmap, (1u64 << 56) - 1);
        let crc = u16::from_le_bytes(block[16..18].try_into().unwrap());
        let mut zeroed = block.to_vec();
        zeroed[16] = 0;
        zeroed[17] = 0;
        assert_eq!(crc, crc16(&zeroed[..1821]));
        assert_eq!(u16::from_le_bytes(block[18..20].try_into().unwrap()), 1);
        assert_eq!(u16::from_le_bytes(block[20..22].try_into().unwrap()), 100);
        // Heights rounded to the metre.
        assert_eq!(i16::from_le_bytes(block[22..24].try_into().unwrap()), 577);
        // LonDegrees, LatDegrees.
        assert_eq!(
            i16::from_le_bytes(block[1818..1820].try_into().unwrap()),
            149
        );
        assert_eq!(block[1820] as i8, -36);
    }
    // The first block is the degree's south-west corner.
    assert_eq!(read_i32(data, 8), -360_000_000);
    assert_eq!(read_i32(data, 12), 1_490_000_000);
    assert_eq!(
        r.messages.last().unwrap().0,
        "Terrain DAT created in Documents/Mission Planner/TerrainDat folder"
    );
    assert!(r.status.last().unwrap().starts_with("Making block "));
}

/// A spacing that is not a number, or out of range, is the C#'s error, and nothing is made.
#[test]
fn terrainmaker_checks_the_spacing() {
    let Some((mut plugin, script)) = load("terrainmaker") else {
        return;
    };
    plugin.loaded().unwrap();
    for (answer, error) in [
        ("abc", "Invalid Number"),
        ("3", "Spacing must be between 5 and 100 meters"),
    ] {
        script.with(|r| {
            r.view_area = Some(Area {
                top: 1.0,
                bottom: 0.5,
                left: 0.5,
                right: 1.0,
            });
            r.message_answers.push_back(DialogResult::Yes);
            r.input_answers.push_back(Some(answer.to_owned()));
        });
        plugin.menu_click(0, 0.0, 0.0).unwrap();
        let r = script.record();
        assert_eq!(r.messages.last().unwrap().0, error);
        assert!(r.written.is_empty());
    }
}

// ---- OpenDroneID --------------------------------------------------------------------------

/// Loaded without the flight screen's saved tabs, it asks for a restart; its tab is a form of
/// the C#'s fields; the UAS ID is saved as typed; and with no transmitter heard, nothing is sent.
#[test]
fn opendroneid_shows_its_tab_and_keeps_the_uas_id() {
    let Some((mut plugin, script)) = load("opendroneid") else {
        return;
    };
    assert_eq!(plugin.info().name, "Open Drone ID");
    // `_update_rate_hz_1`, read at load.
    assert_eq!(plugin.loop_rate_hz(), 10.0);
    assert!(plugin.init().unwrap());
    plugin.loaded().unwrap();
    assert!(
        script.record().messages[0]
            .0
            .starts_with("Restart Mission Planner")
    );
    // Its page: `tabDroneID`, "Drone ID", inserted sixth.
    assert_eq!(
        script.record().flight_tabs,
        [("tabDroneID".to_owned(), "Drone ID".to_owned(), 5)]
    );
    let (title, controls) = script.record().form.clone().unwrap();
    assert_eq!(title, "Drone ID");
    let labels: Vec<&str> = controls.iter().map(|c| c.label.as_str()).collect();
    assert!(labels.contains(&"UAS ID"));
    assert!(labels.contains(&"Oper. ID Type"));
    assert_eq!(script.control("ua_type").unwrap().options.len(), 16);
    plugin.form_event("uas_id", "ABC123").unwrap();
    assert_eq!(script.record().config["ODID_UAS_ID"], "ABC123");
    for _ in 0..5 {
        plugin.run_loop().unwrap();
    }
    assert!(script.record().packets.is_empty());
}

/// With the flight screen's tabs saved, no restart is asked.
#[test]
fn opendroneid_needs_no_restart_with_tabs_saved() {
    let Some((mut plugin, script)) = load("opendroneid") else {
        return;
    };
    script.with(|r| {
        r.config
            .insert("tabcontrolactions".to_owned(), "tabQuick".to_owned());
    });
    plugin.loaded().unwrap();
    assert!(script.record().messages.is_empty());
}

// ---- Dowding ------------------------------------------------------------------------------

/// Enabled at start without credentials: the C#'s two messages, since its web socket cannot
/// open; its two flight map entries; its page's check box kept as `True`/`False`; the tracker
/// home checked, stored, and its send failing as the C#'s does without a tracker.
#[test]
fn dowding_without_a_network() {
    let Some((mut plugin, script)) = load("dowding") else {
        return;
    };
    script.with(|r| {
        r.config
            .insert("Dowding_enabled".to_owned(), "True".to_owned());
    });
    assert!(plugin.init().unwrap());
    assert_eq!(plugin.loop_rate_hz(), 1.0);
    let texts: Vec<String> = script
        .record()
        .messages
        .iter()
        .map(|m| m.0.clone())
        .collect();
    assert_eq!(
        texts,
        ["Dowding invalid settings", "Failed to start Dowding"]
    );
    plugin.loaded().unwrap();
    let menu: Vec<String> = script.record().menu.iter().map(|m| m.2.clone()).collect();
    assert_eq!(menu, ["Dowding", "Dowding Point At"]);
    plugin.menu_click(0, 0.0, 0.0).unwrap();
    assert_eq!(script.record().form.as_ref().unwrap().0, "Dowding");
    assert_eq!(script.control("chk_enable").unwrap().value, "true");
    plugin.form_event("chk_enable", "false").unwrap();
    assert_eq!(script.record().config["Dowding_enabled"], "False");
    plugin.form_event("txt_trackerlat", "north").unwrap();
    plugin.form_event("but_setathome", "").unwrap();
    assert_eq!(
        script.record().messages.last().unwrap().0,
        "Invalid Location lat"
    );
    plugin.form_event("txt_trackerlat", "-35.5").unwrap();
    plugin.form_event("txt_trackerlong", "149").unwrap();
    plugin.form_event("txt_trackerhae", "600").unwrap();
    plugin.form_event("but_setathome", "").unwrap();
    let r = script.record();
    assert_eq!(r.config["Dowding_trackerlat"], "-35.5");
    assert_eq!(r.config["Dowding_trackerlng"], "149");
    assert_eq!(r.messages.last().unwrap().0, "Failed to send home location");
}

/// With a token and a server the settings are valid, and only the start's failure is said.
#[test]
fn dowding_with_a_token() {
    let Some((mut plugin, script)) = load("dowding") else {
        return;
    };
    script.with(|r| {
        for (key, value) in [
            ("Dowding_enabled", "true"),
            ("Dowding_token", "t"),
            ("Dowding_server", "s"),
        ] {
            r.config.insert(key.to_owned(), value.to_owned());
        }
        r.cs.insert("lat".to_owned(), CsValue::Number(0.0));
    });
    plugin.init().unwrap();
    let texts: Vec<String> = script
        .record()
        .messages
        .iter()
        .map(|m| m.0.clone())
        .collect();
    assert_eq!(texts, ["Failed to start Dowding"]);
}
