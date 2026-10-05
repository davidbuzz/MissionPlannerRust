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

//! ADS-B reports arriving over a link and becoming traffic.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use web_time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_mavlink::{Message as _, encode_v2};
use mp_mavlink_dialects::all::{AdsbVehicle, Heartbeat};
use mp_transport::Transport;
use mp_transport::testing::Loopback;

/// `ADSB_FLAGS_VALID_COORDS | VALID_ALTITUDE | VALID_HEADING | VALID_VELOCITY`.
const ALL_VALID: u16 = 1 | 2 | 4 | 8;

fn heartbeat(seq: u8) -> Vec<u8> {
    let message = Heartbeat {
        custom_mode: 0,
        r#type: 2,
        autopilot: 3,
        base_mode: 81,
        system_status: 3,
        mavlink_version: 3,
    };
    let mut payload = [0u8; Heartbeat::LEN];
    message.encode(&mut payload);
    let mut frame = [0u8; 64];
    let n = encode_v2(
        &mut frame,
        seq,
        1,
        1,
        Heartbeat::ID,
        &payload,
        Heartbeat::CRC_EXTRA,
        0,
    )
    .unwrap();
    frame[..n].to_vec()
}

fn adsb(seq: u8, icao: u32, callsign: &str, flags: u16) -> Vec<u8> {
    let mut name = [0u8; 9];
    let bytes = callsign.as_bytes();
    let take = bytes.len().min(9);
    name[..take].copy_from_slice(&bytes[..take]);

    let message = AdsbVehicle {
        icao_address: icao,
        lat: -353_632_620,
        lon: 1_491_652_370,
        altitude: 1_500_000,
        heading: 9_000,
        hor_velocity: 5_000,
        ver_velocity: 0,
        flags,
        squawk: 1200,
        altitude_type: 0,
        callsign: name,
        emitter_type: 0,
        tslc: 1,
    };
    let mut payload = [0u8; AdsbVehicle::LEN];
    message.encode(&mut payload);
    let mut frame = [0u8; 128];
    let n = encode_v2(
        &mut frame,
        seq,
        1,
        1,
        AdsbVehicle::ID,
        &payload,
        AdsbVehicle::CRC_EXTRA,
        0,
    )
    .unwrap();
    frame[..n].to_vec()
}

fn wait_for(what: &str, mut check: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        if check() {
            return;
        }
        wasm_thread::sleep(Duration::from_millis(5));
    }
    panic!("timed out waiting for {what}");
}

#[test]
fn an_adsb_report_becomes_traffic_with_its_position_and_callsign() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat(0)).unwrap();
    vehicle_side
        .write_all(&adsb(1, 0x7C_1A2B, "QFA123", ALL_VALID))
        .unwrap();

    wait_for("the aircraft", || !link.traffic().is_empty());

    let traffic = link.traffic();
    assert_eq!(traffic.len(), 1);
    let aircraft = &traffic[0];
    assert_eq!(aircraft.icao, 0x7C_1A2B);
    assert_eq!(aircraft.callsign.as_deref(), Some("QFA123"));
    assert_eq!(aircraft.label(), "QFA123");
    assert!((aircraft.position.latitude() - -35.363_262).abs() < 1e-6);
    // Altitude is millimetres on the wire, the same unit GLOBAL_POSITION_INT uses and easy to
    // read as metres by mistake.
    assert!(
        (aircraft.altitude - 1500.0).abs() < 0.001,
        "{}",
        aircraft.altitude
    );
    // Heading is centi-degrees.
    assert!((aircraft.heading.expect("a heading") - 90.0).abs() < 0.01);
    assert!((aircraft.speed.expect("a speed") - 50.0).abs() < 0.01);
}

#[test]
fn a_report_without_valid_heading_or_velocity_reports_neither() {
    // Zero is a legitimate heading, so absence has to come from the flags. Reading the field
    // regardless would draw every aircraft pointing due north at a standstill.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat(0)).unwrap();
    // Coordinates valid, heading and velocity not.
    vehicle_side
        .write_all(&adsb(1, 0x40_0001, "", 1 | 2))
        .unwrap();

    wait_for("the aircraft", || !link.traffic().is_empty());

    let traffic = link.traffic();
    assert_eq!(traffic[0].heading, None);
    assert_eq!(traffic[0].speed, None);
    // And with no callsign it is named by its address, as every other tool does.
    assert_eq!(traffic[0].callsign, None);
    assert_eq!(traffic[0].label(), "400001");
}

#[test]
fn a_moving_aircraft_stays_one_symbol() {
    // Keyed on identity, not position. Otherwise a moving aircraft becomes a trail.
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat(0)).unwrap();
    for seq in 1..5 {
        vehicle_side
            .write_all(&adsb(seq, 0x7C_1A2B, "QFA123", ALL_VALID))
            .unwrap();
    }

    wait_for("the aircraft", || !link.traffic().is_empty());
    wasm_thread::sleep(Duration::from_millis(200));
    assert_eq!(
        link.traffic().len(),
        1,
        "four reports should be one aircraft"
    );
}

#[test]
fn two_aircraft_are_two_symbols() {
    let (mut vehicle_side, gcs_side) = Loopback::pair();
    let link = Link::from_transport(Box::new(gcs_side), LinkConfig::default());

    vehicle_side.write_all(&heartbeat(0)).unwrap();
    vehicle_side
        .write_all(&adsb(1, 0x7C_1A2B, "QFA123", ALL_VALID))
        .unwrap();
    vehicle_side
        .write_all(&adsb(2, 0x40_0001, "VH-ABC", ALL_VALID))
        .unwrap();

    wait_for("both aircraft", || link.traffic().len() >= 2);
    assert_eq!(link.traffic().len(), 2);
}
