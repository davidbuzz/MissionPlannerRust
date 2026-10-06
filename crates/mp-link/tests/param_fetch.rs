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

//! The parameter fetch through the real link thread: `@PARAM/param.pck?withdefaults=1` over
//! MAVFTP first, the `PARAM_REQUEST_LIST` stream when the vehicle has no such file.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1810-1928`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation
)]

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use web_time::{Duration, Instant};

use mp_ftp::mavftp::testing::FakeVehicle;
use mp_ftp::mavftp::wire::Header;
use mp_link::ftp::ftp_message;
use mp_link::param_fetch::{FetchVia, PARAM_FILE, ParamFetchState};
use mp_link::{Link, LinkConfig, ProtocolTimeouts};
use mp_mavlink::{FrameDecoder, MavVersion, encode_v2};
use mp_mavlink_dialects::all::{DIALECT, Heartbeat, MavMessage, ParamValue};
use mp_params::parampck::{PackedParam, pack, unpack};
use mp_params::{ParamType, decode_param_id, encode_param_id};
use mp_transport::Transport;
use mp_transport::testing::{Loopback, LoopbackEnd};
use mp_vehicle::VehicleId;

const VEHICLE: VehicleId = VehicleId::new(1, 1);
const GCS: VehicleId = VehicleId::new(255, 190);

fn frame(seq: u8, message: &MavMessage) -> Vec<u8> {
    let mut payload = [0u8; 255];
    let len = message.encode(&mut payload);
    let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
    let n = encode_v2(
        &mut out,
        seq,
        VEHICLE.sysid,
        VEHICLE.compid,
        message.id(),
        &payload[..len],
        message.crc_extra(),
        0,
    )
    .unwrap();
    out[..n].to_vec()
}

/// A copter-like table: names sharing prefixes, every AP_Param type, two off their defaults.
fn table() -> Vec<PackedParam> {
    let entry = |name: &str, value: f64, kind: ParamType, default: f64| PackedParam {
        name: name.to_owned(),
        value: mp_params::ParamValue::from_ardupilot(value as f32, kind),
        default: Some(default),
    };
    vec![
        entry("ACRO_BAL_PITCH", 1.0, ParamType::Real32, 1.0),
        entry("ACRO_BAL_ROLL", 1.0, ParamType::Real32, 1.0),
        entry("ACRO_OPTIONS", 0.0, ParamType::Int8, 0.0),
        entry("ATC_ACCEL_P_MAX", 110_000.0, ParamType::Real32, 110_000.0),
        entry("BATT_CAPACITY", 3300.0, ParamType::Int32, 3300.0),
        entry("INS_GYRO_FILTER", 20.0, ParamType::Int16, 20.0),
        entry("MOT_THST_EXPO", 0.65, ParamType::Real32, 0.5),
        entry("SERVO1_FUNCTION", 33.0, ParamType::Int16, 0.0),
    ]
}

/// The vehicle on a thread of its own: it announces itself, answers every
/// `FILE_TRANSFER_PROTOCOL` from `files`, and streams `stream` as `PARAM_VALUE`s when asked for
/// the list - the classic path - counting how many list requests it heard.
fn run_vehicle(
    mut end: LoopbackEnd,
    stop: Arc<AtomicBool>,
    mut files: FakeVehicle,
    stream: Vec<PackedParam>,
) -> wasm_thread::JoinHandle<(usize, usize)> {
    wasm_thread::spawn(move || {
        let mut seq = 0u8;
        let heartbeat = MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 2,
            autopilot: 3,
            base_mode: 81,
            system_status: 3,
            mavlink_version: 3,
        });
        end.write_all(&frame(seq, &heartbeat)).unwrap();
        let mut decoder = FrameDecoder::new();
        let mut buf = [0u8; 4096];
        let mut list_requests = 0;
        let mut ftp_requests = 0;
        let mut last_beat = Instant::now();
        while !stop.load(Ordering::Acquire) {
            if last_beat.elapsed() > Duration::from_millis(500) {
                seq = seq.wrapping_add(1);
                end.write_all(&frame(seq, &heartbeat)).unwrap();
                last_beat = Instant::now();
            }
            let n = end.read(&mut buf).unwrap_or(0);
            if n == 0 {
                wasm_thread::sleep(Duration::from_millis(1));
                continue;
            }
            let mut heard = Vec::new();
            decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                    heard.push(message);
                }
            });
            for message in heard {
                match message {
                    MavMessage::FileTransferProtocol(ftp) => {
                        ftp_requests += 1;
                        let head = Header::decode(&ftp.payload);
                        for reply in files.answer(&head) {
                            seq = seq.wrapping_add(1);
                            end.write_all(&frame(seq, &ftp_message(GCS, &reply)))
                                .unwrap();
                        }
                    }
                    MavMessage::ParamRequestList(_) => {
                        list_requests += 1;
                        let count = u16::try_from(stream.len()).unwrap();
                        for (index, entry) in stream.iter().enumerate() {
                            seq = seq.wrapping_add(1);
                            let value = MavMessage::ParamValue(ParamValue {
                                param_value: entry.value.to_param_value_field(),
                                param_count: count,
                                param_index: u16::try_from(index).unwrap(),
                                param_id: encode_param_id(&entry.name),
                                param_type: entry.value.param_type().to_wire(),
                            });
                            end.write_all(&frame(seq, &value)).unwrap();
                        }
                    }
                    _ => {}
                }
            }
        }
        (ftp_requests, list_requests)
    })
}

fn link(end: LoopbackEnd) -> Link {
    recording_link(end, None)
}

fn recording_link(end: LoopbackEnd, record_path: Option<PathBuf>) -> Link {
    let config = LinkConfig {
        send_heartbeat: false,
        stream_rate_hz: 0,
        timeouts: ProtocolTimeouts::default().faster(20),
        record_path,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(end), config);
    let deadline = Instant::now() + Duration::from_secs(5);
    while link.vehicle(VEHICLE).is_none() {
        assert!(Instant::now() < deadline, "the vehicle was never heard");
        wasm_thread::sleep(Duration::from_millis(1));
    }
    link
}

fn wait_until_finished(link: &Link) -> ParamFetchState {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let fetch = link.param_fetch(VEHICLE).expect("a fetch was started");
        if fetch.is_finished() {
            return fetch.state;
        }
        assert!(
            Instant::now() < deadline,
            "the fetch never finished: {fetch:?}"
        );
        wasm_thread::sleep(Duration::from_millis(2));
    }
}

/// A vehicle serving `@PARAM/param.pck?withdefaults=1`: the table comes as one file, every
/// parameter at its place, the defaults kept, and the stream is never asked for.
#[test]
fn the_table_comes_over_mavftp_with_its_defaults() {
    let (vehicle_side, gcs_side) = Loopback::pair();
    let stop = Arc::new(AtomicBool::new(false));
    let entries = table();
    let file = pack(&entries, true, None);
    let files = FakeVehicle::new().with_file(PARAM_FILE, &file);
    let vehicle = run_vehicle(vehicle_side, Arc::clone(&stop), files, Vec::new());
    let link = link(gcs_side);

    assert!(link.fetch_params(VEHICLE));
    let state = wait_until_finished(&link);
    assert_eq!(
        state,
        ParamFetchState::Complete {
            via: FetchVia::MavFtp,
            count: entries.len()
        }
    );
    let params = link.params(VEHICLE).expect("a table");
    assert!(params.is_complete(), "{params:?}");
    assert_eq!(params.len(), entries.len());
    assert_eq!(params.expected(), Some(8));
    assert!((params.get("MOT_THST_EXPO").unwrap().as_f64() - 0.65).abs() < 1e-6);
    assert_eq!(params.get("BATT_CAPACITY").unwrap().as_f64(), 3300.0);
    assert_eq!(
        params.get("INS_GYRO_FILTER").unwrap().param_type(),
        ParamType::Int16
    );
    let fetch = link.param_fetch(VEHICLE).unwrap();
    assert_eq!(fetch.defaults.get("MOT_THST_EXPO"), Some(&0.5));
    assert_eq!(fetch.defaults.get("SERVO1_FUNCTION"), Some(&0.0));
    assert_eq!(fetch.defaults.len(), entries.len());

    stop.store(true, Ordering::Release);
    let (ftp_requests, list_requests) = vehicle.join().unwrap();
    assert!(ftp_requests > 0);
    assert_eq!(list_requests, 0, "the stream was never asked for");
}

/// A vehicle with no such file: the MAVFTP open is refused, and the fetch falls back to the
/// stream, which fills the table.
#[test]
fn without_the_file_the_stream_fills_the_table() {
    let (vehicle_side, gcs_side) = Loopback::pair();
    let stop = Arc::new(AtomicBool::new(false));
    let entries = table();
    let files = FakeVehicle::new();
    let vehicle = run_vehicle(vehicle_side, Arc::clone(&stop), files, entries.clone());
    let link = link(gcs_side);

    assert!(link.fetch_params(VEHICLE));
    let state = wait_until_finished(&link);
    assert_eq!(
        state,
        ParamFetchState::Complete {
            via: FetchVia::Stream,
            count: entries.len()
        }
    );
    let params = link.params(VEHICLE).expect("a table");
    assert!(params.is_complete(), "{params:?}");
    assert_eq!(params.len(), entries.len());
    assert!((params.get("MOT_THST_EXPO").unwrap().as_f64() - 0.65).abs() < 1e-6);
    let fetch = link.param_fetch(VEHICLE).unwrap();
    assert!(fetch.defaults.is_empty(), "the stream carries no defaults");

    stop.store(true, Ordering::Release);
    let (ftp_requests, list_requests) = vehicle.join().unwrap();
    assert!(ftp_requests > 0, "MAVFTP was tried first");
    assert!(list_requests >= 1, "then the list was asked for");
}

/// A MAVFTP fetch on a recording link: the recording holds each parameter of the file as the
/// `PARAM_VALUE` the C#'s `SaveToTlog` writes for it - MAVLink 1 from the vehicle, numbered from
/// 0, index 0, the count, `(float)a.Value` and the type - in the file's order.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1903-1912`
#[test]
fn a_mavftp_fetch_records_each_parameter_as_a_param_value() {
    let (vehicle_side, gcs_side) = Loopback::pair();
    let stop = Arc::new(AtomicBool::new(false));
    let entries = table();
    let file = pack(&entries, true, None);
    let files = FakeVehicle::new().with_file(PARAM_FILE, &file);
    let vehicle = run_vehicle(vehicle_side, Arc::clone(&stop), files, Vec::new());
    // `TlogWriter` refuses to overwrite.
    let recording = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("param_fetch-{}-recorded.tlog", mp_os::process_id()));
    let _ = std::fs::remove_file(&recording);
    let link = recording_link(gcs_side, Some(recording.clone()));

    assert!(link.fetch_params(VEHICLE));
    assert!(matches!(
        wait_until_finished(&link),
        ParamFetchState::Complete {
            via: FetchVia::MavFtp,
            ..
        }
    ));
    // Closed, so the recording is flushed.
    drop(link);
    stop.store(true, Ordering::Release);
    vehicle.join().unwrap();

    // The recording, a timestamp and a frame at a time.
    let bytes = std::fs::read(&recording).unwrap();
    let _ = std::fs::remove_file(&recording);
    let mut recorded = Vec::new();
    let mut at = 0;
    while at < bytes.len() {
        at += 8;
        let (frame, used) = mp_mavlink::parse(&bytes[at..], &DIALECT).unwrap();
        if let Some(MavMessage::ParamValue(value)) = MavMessage::decode(frame.msgid, frame.payload)
        {
            recorded.push((frame.version, frame.seq, frame.sysid, frame.compid, value));
        }
        at += used;
    }
    let in_the_file = unpack(&file).unwrap();
    assert_eq!(recorded.len(), in_the_file.len());
    for (n, ((version, seq, sysid, compid, value), entry)) in
        recorded.iter().zip(&in_the_file).enumerate()
    {
        assert_eq!(*version, MavVersion::V1);
        assert_eq!(usize::from(*seq), n);
        assert_eq!((*sysid, *compid), (VEHICLE.sysid, VEHICLE.compid));
        assert_eq!(decode_param_id(&value.param_id), entry.name);
        assert_eq!((value.param_count, value.param_index), (8, 0));
        assert_eq!(value.param_type, entry.value.param_type().to_wire());
    }
    let value_of = |name: &str| {
        recorded
            .iter()
            .find(|(.., value)| decode_param_id(&value.param_id) == name)
            .map(|(.., value)| value.param_value)
            .unwrap()
    };
    // A float through `GetValue`'s seven significant digits, an integer exactly.
    assert_eq!(value_of("MOT_THST_EXPO"), 0.65);
    assert_eq!(value_of("BATT_CAPACITY"), 3300.0);
    assert_eq!(value_of("ACRO_OPTIONS"), 0.0);
}
