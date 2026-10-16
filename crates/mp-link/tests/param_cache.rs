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

//! `param.json` through the real link thread: written `MAVState`'s two seconds - here a twentieth
//! of that - after the parameters change, in the shape Json.NET gives it, never when nothing
//! changed, and read back in place of the fetch only when `UseCachedParams` is set.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:27-46, 63-95; MainV2.cs:1659-1667`

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation
)]

use mp_os::Lock as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use mp_ftp::mavftp::testing::FakeVehicle;
use mp_ftp::mavftp::wire::Header;
use mp_link::ftp::ftp_message;
use mp_link::param_cache::{self, NEWLINE};
use mp_link::param_download::ParamDownloadState;
use mp_link::param_fetch::{CAPABILITY_FTP, PARAM_FILE, ParamFetchState};
use mp_link::{Link, LinkConfig, ProtocolTimeouts};
use mp_mavlink::{FrameDecoder, encode_v2};
use mp_mavlink_dialects::all::{
    AutopilotVersion, DIALECT, Heartbeat, MavMessage, ParamValue as ParamValueMessage,
};
use mp_params::parampck::{PackedParam, pack};
use mp_params::{ParamType, ParamValue, decode_param_id, encode_param_id};
use mp_transport::Transport;
use mp_transport::testing::{Loopback, LoopbackEnd};
use mp_vehicle::VehicleId;

const VEHICLE: VehicleId = VehicleId::new(1, 1);
const GCS: VehicleId = VehicleId::new(255, 190);

/// The board's `uid2`, as its `AUTOPILOT_VERSION` gives it.
const UID2: [u8; 18] = [
    0x00, 0x1f, 0x00, 0x24, 0x31, 0x32, 0x51, 0x02, 0x33, 0x34, 0x35, 0x38, 0, 0, 0, 0, 0, 0,
];

/// `cs.uid2`: `ToHexString`, two lowercase digits a byte.
const UID2_HEX: &str = "001f00243132510233343538000000000000";

/// The stream's six parameters as `param.ToJSON()` writes them after the fetch: every one an
/// ArduPilot `PARAM_VALUE`'s, so `Type` 9 with the declared type as `TypeAP` and no default, in
/// the order the vehicle listed them; floats through seven significant digits, `1E-05` as .NET
/// writes it, `.0` on whole numbers.
const STREAMED: &str = r#"[
  {
    "default_value": null,
    "Name": "ACRO_OPTIONS",
    "Type": 9,
    "Value": 0.0,
    "TypeAP": 2
  },
  {
    "default_value": null,
    "Name": "ATC_ACCEL_P_MAX",
    "Type": 9,
    "Value": 110000.0,
    "TypeAP": 9
  },
  {
    "default_value": null,
    "Name": "BATT_CAPACITY",
    "Type": 9,
    "Value": 3300.0,
    "TypeAP": 6
  },
  {
    "default_value": null,
    "Name": "COMPASS_DEC",
    "Type": 9,
    "Value": -0.2,
    "TypeAP": 9
  },
  {
    "default_value": null,
    "Name": "INS_ACCOFFS_X",
    "Type": 9,
    "Value": 1E-05,
    "TypeAP": 9
  },
  {
    "default_value": null,
    "Name": "MOT_THST_EXPO",
    "Type": 9,
    "Value": 0.65,
    "TypeAP": 9
  }
]"#;

/// The MAVFTP file's four as the C# writes them after `getParamListMavftp`: `Type` the file's
/// own type (`parampck.unpack`), the file's default, and a float default widened from the
/// file's float by `Convert.ToDouble`, 17 digits being what reads back.
const OVER_MAVFTP: &str = r#"[
  {
    "default_value": 0.0,
    "Name": "ACRO_OPTIONS",
    "Type": 2,
    "Value": 0.0,
    "TypeAP": 2
  },
  {
    "default_value": 3300.0,
    "Name": "BATT_CAPACITY",
    "Type": 6,
    "Value": 3300.0,
    "TypeAP": 6
  },
  {
    "default_value": 20.0,
    "Name": "INS_GYRO_FILTER",
    "Type": 4,
    "Value": 20.0,
    "TypeAP": 4
  },
  {
    "default_value": 0.44999998807907104,
    "Name": "MOT_THST_EXPO",
    "Type": 9,
    "Value": 0.65,
    "TypeAP": 9
  }
]"#;

/// A file the C# wrote on Windows: lines ended `\r\n`, a MAVFTP entry, values the vehicle no
/// longer has.
const WRITTEN_BY_THE_CSHARP: &str = r#"[
  {
    "default_value": 0.0,
    "Name": "ACRO_OPTIONS",
    "Type": 2,
    "Value": 1.0,
    "TypeAP": 2
  },
  {
    "default_value": null,
    "Name": "BATT_CAPACITY",
    "Type": 9,
    "Value": 5000.0,
    "TypeAP": 6
  },
  {
    "default_value": 0.44999998807907104,
    "Name": "MOT_THST_EXPO",
    "Type": 9,
    "Value": 0.5,
    "TypeAP": 9
  }
]"#;

/// A fixture written with `\n`, with this platform's `Environment.NewLine`.
fn native(fixture: &str) -> String {
    fixture.replace('\n', NEWLINE)
}

/// The stream's parameters: name, value, declared type, in the vehicle's order.
fn stream() -> Vec<(&'static str, f32, ParamType)> {
    vec![
        ("ACRO_OPTIONS", 0.0, ParamType::Int8),
        ("ATC_ACCEL_P_MAX", 110_000.0, ParamType::Real32),
        ("BATT_CAPACITY", 3300.0, ParamType::Int32),
        ("COMPASS_DEC", -0.2, ParamType::Real32),
        ("INS_ACCOFFS_X", 1e-5, ParamType::Real32),
        ("MOT_THST_EXPO", 0.65, ParamType::Real32),
    ]
}

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

fn param_value(name: &str, value: f32, kind: ParamType, index: u16, count: u16) -> MavMessage {
    MavMessage::ParamValue(ParamValueMessage {
        param_value: value,
        param_count: count,
        param_index: index,
        param_id: encode_param_id(name),
        param_type: kind.to_wire(),
    })
}

/// An ArduPilot quadcopter on a board whose `uid2` is [`UID2`].
struct Board {
    /// Its parameters: name, value, declared type, in its order.
    params: Mutex<Vec<(String, f32, ParamType)>>,
    /// Whether it sends its `AUTOPILOT_VERSION`, with each heartbeat.
    announce: AtomicBool,
    /// Its `MAV_PROTOCOL_CAPABILITY` bits.
    capabilities: u64,
    /// How many `PARAM_REQUEST_LIST` it heard.
    list_requests: AtomicUsize,
    stop: AtomicBool,
}

impl Board {
    fn new(params: Vec<(&str, f32, ParamType)>, announce: bool, capabilities: u64) -> Arc<Self> {
        Arc::new(Self {
            params: Mutex::new(
                params
                    .into_iter()
                    .map(|(name, value, kind)| (name.to_owned(), value, kind))
                    .collect(),
            ),
            announce: AtomicBool::new(announce),
            capabilities,
            list_requests: AtomicUsize::new(0),
            stop: AtomicBool::new(false),
        })
    }

    /// Its answer to a set or a read of `name`: the parameter, at its index.
    fn answer(&self, name: &str, set: Option<f32>) -> Option<MavMessage> {
        let mut params = self.params.os_lock().unwrap();
        let count = u16::try_from(params.len()).unwrap();
        let index = params.iter().position(|(held, ..)| held == name)?;
        if let Some(value) = set {
            params[index].1 = value;
        }
        let (_, value, kind) = params[index].clone();
        Some(param_value(
            name,
            value,
            kind,
            u16::try_from(index).unwrap(),
            count,
        ))
    }
}

/// The vehicle on a thread of its own: a heartbeat every 50 ms, its `AUTOPILOT_VERSION` after
/// each while it announces itself; its parameters streamed on `PARAM_REQUEST_LIST`, a set
/// echoed, a read by name answered, and MAVFTP answered from `files`.
fn run_vehicle(
    mut end: LoopbackEnd,
    board: Arc<Board>,
    mut files: FakeVehicle,
) -> wasm_thread::JoinHandle<()> {
    wasm_thread::spawn(move || {
        let mut seq = 0u8;
        let mut send = |end: &mut LoopbackEnd, message: &MavMessage| {
            seq = seq.wrapping_add(1);
            end.write_all(&frame(seq, message)).unwrap();
        };
        let heartbeat = MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 2,
            autopilot: 3,
            base_mode: 81,
            system_status: 3,
            mavlink_version: 3,
        });
        let version = MavMessage::AutopilotVersion(AutopilotVersion {
            capabilities: board.capabilities,
            uid: 0,
            flight_sw_version: 0x0406_00ff,
            middleware_sw_version: 0,
            os_sw_version: 0,
            board_version: 0,
            vendor_id: 0,
            product_id: 0,
            flight_custom_version: [0; 8],
            middleware_custom_version: [0; 8],
            os_custom_version: [0; 8],
            uid2: UID2,
        });
        let mut decoder = FrameDecoder::new();
        let mut buf = [0u8; 4096];
        let mut last_beat = Instant::now() - Duration::from_secs(1);
        while !board.stop.load(Ordering::Acquire) {
            if last_beat.elapsed() > Duration::from_millis(50) {
                send(&mut end, &heartbeat);
                if board.announce.load(Ordering::Acquire) {
                    send(&mut end, &version);
                }
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
                        let head = Header::decode(&ftp.payload);
                        for reply in files.answer(&head) {
                            send(&mut end, &ftp_message(GCS, &reply));
                        }
                    }
                    MavMessage::ParamRequestList(_) => {
                        board.list_requests.fetch_add(1, Ordering::AcqRel);
                        let params = board.params.os_lock().unwrap().clone();
                        let count = u16::try_from(params.len()).unwrap();
                        for (index, (name, value, kind)) in params.iter().enumerate() {
                            let index = u16::try_from(index).unwrap();
                            send(&mut end, &param_value(name, *value, *kind, index, count));
                        }
                    }
                    MavMessage::ParamSet(set) => {
                        let name = decode_param_id(&set.param_id);
                        if let Some(echo) = board.answer(&name, Some(set.param_value)) {
                            send(&mut end, &echo);
                        }
                    }
                    MavMessage::ParamRequestRead(read) => {
                        let name = decode_param_id(&read.param_id);
                        if let Some(answer) = board.answer(&name, None) {
                            send(&mut end, &answer);
                        }
                    }
                    _ => {}
                }
            }
        }
    })
}

/// The C#'s timer, a twentieth of its two seconds: 100 ms.
fn fast() -> ProtocolTimeouts {
    ProtocolTimeouts::default().faster(20)
}

/// A link keeping its cache under `directory`, the vehicle heard on it.
fn start(
    directory: &Path,
    use_cached_params: bool,
    timeouts: ProtocolTimeouts,
    board: &Arc<Board>,
    files: FakeVehicle,
) -> (Link, wasm_thread::JoinHandle<()>) {
    let (vehicle_side, gcs_side) = Loopback::pair();
    let vehicle = run_vehicle(vehicle_side, Arc::clone(board), files);
    let config = LinkConfig {
        send_heartbeat: false,
        stream_rate_hz: 0,
        timeouts,
        data_directory: Some(directory.to_owned()),
        use_cached_params,
        ..LinkConfig::default()
    };
    let link = Link::from_transport(Box::new(gcs_side), config);
    wait_for("the vehicle to be heard", || link.vehicle(VEHICLE).is_some());
    (link, vehicle)
}

fn stop(board: &Board, vehicle: wasm_thread::JoinHandle<()>) {
    board.stop.store(true, Ordering::Release);
    vehicle.join().unwrap();
}

fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        wasm_thread::sleep(Duration::from_millis(2));
    }
}

/// Whether the vehicle's `AUTOPILOT_VERSION` has reached the state the screens read.
fn board_known(link: &Link) -> bool {
    link.vehicle(VEHICLE)
        .is_some_and(|handle| handle.load().autopilot_info.uid2 == UID2)
}

fn download(link: &Link) {
    assert!(link.download_params(VEHICLE));
    wait_for("the download", || {
        link.param_download(VEHICLE)
            .is_some_and(|download| download.state() == ParamDownloadState::Complete)
    });
}

/// A fresh data directory for one test.
fn directory(test: &str) -> PathBuf {
    let directory = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!(
        "param_cache-{}-{test}",
        mp_os::process_id()
    ));
    let _ = std::fs::remove_dir_all(&directory);
    directory
}

/// Where the C# keeps this vehicle's file: `ParamCachePath`, written out by hand.
fn cache_file(directory: &Path) -> PathBuf {
    directory
        .join("paramcache")
        .join("QUADROTOR")
        .join(UID2_HEX)
        .join("1")
        .join("1")
        .join("param.json")
}

/// The file once it is all there, which `File.WriteAllText` leaves ending with its `]`.
fn written(file: &Path) -> Option<String> {
    std::fs::read_to_string(file)
        .ok()
        .filter(|text| text.ends_with(']'))
}

/// Waits for the file to read `expected`, and says what it read instead if it never does.
fn wait_for_text(file: &Path, expected: &str) {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let text = written(file);
        if text.as_deref() == Some(expected) {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "param.json never read\n{expected}\nit reads\n{}",
            text.unwrap_or_default()
        );
        wasm_thread::sleep(Duration::from_millis(5));
    }
}

/// The stream fetched: the C#'s `AddRange` raises the list's event, and two seconds later the
/// whole list is written where `ParamCachePath` says, as Json.NET writes it, without a
/// byte-order mark.
#[test]
fn a_fetched_table_is_written_where_and_as_the_csharp_writes_it() {
    let directory = directory("fetched");
    let file = cache_file(&directory);
    assert_eq!(param_cache::path(&directory, 2, &UID2, VEHICLE), file);
    let board = Board::new(stream(), true, 0);
    let (link, vehicle) = start(&directory, false, fast(), &board, FakeVehicle::new());
    wait_for("AUTOPILOT_VERSION", || board_known(&link));

    download(&link);
    wait_for_text(&file, &native(STREAMED));
    assert_eq!(std::fs::read(&file).unwrap()[0], b'[', "no byte-order mark");

    drop(link);
    stop(&board, vehicle);
}

/// Nothing changes - heartbeats and `AUTOPILOT_VERSION`s go on - and nothing is written; a set,
/// whose echo replaces a parameter the list holds, writes the list again with its new value.
#[test]
fn nothing_changed_is_nothing_written_and_a_set_writes_it_again() {
    let directory = directory("set");
    let file = cache_file(&directory);
    let board = Board::new(stream(), true, 0);
    let (link, vehicle) = start(&directory, false, fast(), &board, FakeVehicle::new());
    wait_for("AUTOPILOT_VERSION", || board_known(&link));
    download(&link);
    wait_for_text(&file, &native(STREAMED));

    std::fs::remove_file(&file).unwrap();
    wasm_thread::sleep(fast().param_cache_write * 10);
    assert!(!file.exists(), "nothing changed, and nothing was written");

    let set = link.set_param(VEHICLE, "MOT_THST_EXPO", 0.7, false);
    wait_for("the set", || {
        link.request(set).is_some_and(|request| request.is_finished())
    });
    let expected = native(STREAMED).replace("\"Value\": 0.65", "\"Value\": 0.7");
    wait_for_text(&file, &expected);

    drop(link);
    stop(&board, vehicle);
}

/// No `uid2` yet, no file - and the C#'s `return` before `queuewrite = false` means a change
/// heard before the board's `AUTOPILOT_VERSION` stops the cache for the session: the board
/// known, a set still writes nothing, nor does the link stopping.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:79-80, 92`
#[test]
fn a_change_before_the_boards_id_stops_the_cache() {
    let directory = directory("stopped");
    let cache = directory.join("paramcache");
    let board = Board::new(stream(), false, 0);
    let (link, vehicle) = start(&directory, false, fast(), &board, FakeVehicle::new());

    download(&link);
    wasm_thread::sleep(fast().param_cache_write * 10);
    assert!(!cache.exists(), "no uid2, and no file");

    board.announce.store(true, Ordering::Release);
    wait_for("AUTOPILOT_VERSION", || board_known(&link));
    let set = link.set_param(VEHICLE, "MOT_THST_EXPO", 0.7, false);
    wait_for("the set", || {
        link.request(set).is_some_and(|request| request.is_finished())
    });
    wasm_thread::sleep(fast().param_cache_write * 10);
    assert!(!cache.exists(), "queuewrite was left set");

    drop(link);
    assert!(!cache.exists());
    stop(&board, vehicle);
}

/// A write still waiting when the link stops is written as it stops, where the C#'s timer fires
/// after the port has closed.
#[test]
fn a_write_still_waiting_is_made_as_the_link_stops() {
    let directory = directory("closing");
    let file = cache_file(&directory);
    let board = Board::new(stream(), true, 0);
    let timeouts = ProtocolTimeouts {
        param_cache_write: Duration::from_secs(3600),
        ..fast()
    };
    let (link, vehicle) = start(&directory, false, timeouts, &board, FakeVehicle::new());
    wait_for("AUTOPILOT_VERSION", || board_known(&link));
    download(&link);
    wasm_thread::sleep(Duration::from_millis(300));
    assert!(!file.exists(), "the timer has an hour to go");

    drop(link);
    assert_eq!(written(&file), Some(native(STREAMED)));
    stop(&board, vehicle);
}

/// Over MAVFTP each parameter is `Type` the file's own and has the file's default. An ArduPilot
/// set's echo keeps the default, `setParam` passing it on, but is a `REAL32` `PARAM_VALUE`'s; a
/// read's answer has no default.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1703-1713, 1899-1902, 2372-2386`
#[test]
fn a_mavftp_fetch_writes_the_files_types_and_defaults() {
    let directory = directory("mavftp");
    let file = cache_file(&directory);
    // Name, value, type and default, the default a float as the file holds it.
    let entries: [(&str, f32, ParamType, f32); 4] = [
        ("ACRO_OPTIONS", 0.0, ParamType::Int8, 0.0),
        ("BATT_CAPACITY", 3300.0, ParamType::Int32, 3300.0),
        ("INS_GYRO_FILTER", 20.0, ParamType::Int16, 20.0),
        ("MOT_THST_EXPO", 0.65, ParamType::Real32, 0.45),
    ];
    let packed: Vec<PackedParam> = entries
        .iter()
        .map(|&(name, value, kind, default)| PackedParam {
            name: name.to_owned(),
            value: ParamValue::from_ardupilot(value, kind),
            default: Some(f64::from(default)),
        })
        .collect();
    let files = FakeVehicle::new().with_file(PARAM_FILE, &pack(&packed, true, None));
    let board = Board::new(
        entries
            .iter()
            .map(|&(name, value, kind, _)| (name, value, kind))
            .collect(),
        true,
        u64::from(CAPABILITY_FTP),
    );
    let (link, vehicle) = start(&directory, false, fast(), &board, files);
    wait_for("AUTOPILOT_VERSION", || board_known(&link));

    assert!(link.fetch_params(VEHICLE));
    wait_for("the fetch", || {
        link.param_fetch(VEHICLE)
            .is_some_and(|fetch| matches!(fetch.state, ParamFetchState::Complete { .. }))
    });
    wait_for_text(&file, &native(OVER_MAVFTP));
    assert_eq!(
        board.list_requests.load(Ordering::Acquire),
        0,
        "the file, not the stream"
    );

    let set = link.set_param(VEHICLE, "ACRO_OPTIONS", 1.0, false);
    wait_for("the set", || {
        link.request(set).is_some_and(|request| request.is_finished())
    });
    let read = link.read_param(VEHICLE, "INS_GYRO_FILTER");
    wait_for("the read", || {
        link.request(read).is_some_and(|request| request.is_finished())
    });
    // ACRO_OPTIONS: the set's echo, `REAL32`, its default kept. INS_GYRO_FILTER: the read's
    // answer, `REAL32`, no default.
    let gyro = "\"Name\": \"INS_GYRO_FILTER\",\n    \"Type\"";
    let expected = native(OVER_MAVFTP)
        .replace(
            &native("\"Type\": 2,\n    \"Value\": 0.0,"),
            &native("\"Type\": 9,\n    \"Value\": 1.0,"),
        )
        .replace(
            &native(&format!("\"default_value\": 20.0,\n    {gyro}: 4,")),
            &native(&format!("\"default_value\": null,\n    {gyro}: 9,")),
        );
    wait_for_text(&file, &expected);

    drop(link);
    stop(&board, vehicle);
}

/// With `UseCachedParams` set and the file fresh, the C#'s file is read in place of the fetch:
/// the table whole at the file's count, each entry as the file has it - an integer `Type` its
/// own bytes - and nothing asked of the vehicle. Every name was new to the table, so nothing
/// changed and the file is left as the C# wrote it.
/// `// C#: MainV2.cs:1659-1667`
#[test]
fn with_use_cached_params_the_file_is_read_in_place_of_the_fetch() {
    let directory = directory("read");
    let file = cache_file(&directory);
    std::fs::create_dir_all(file.parent().unwrap()).unwrap();
    let by_the_csharp = WRITTEN_BY_THE_CSHARP.replace('\n', "\r\n");
    std::fs::write(&file, &by_the_csharp).unwrap();
    let board = Board::new(stream(), true, 0);
    let (link, vehicle) = start(&directory, true, fast(), &board, FakeVehicle::new());
    wait_for("AUTOPILOT_VERSION", || board_known(&link));

    assert_eq!(link.read_cached_params(VEHICLE), Ok(Some(3)));
    let params = link.params(VEHICLE).expect("a table");
    assert!(params.is_complete());
    assert_eq!((params.len(), params.expected()), (3, Some(3)));
    let battery = params.get("BATT_CAPACITY").unwrap();
    assert_eq!(battery.as_f64(), 5000.0);
    assert_eq!(
        (battery.wire_type(), battery.param_type()),
        (ParamType::Real32, ParamType::Int32)
    );
    let acro = params.get("ACRO_OPTIONS").unwrap();
    assert_eq!(acro.as_f64(), 1.0);
    assert_eq!(
        (acro.wire_type(), acro.param_type()),
        (ParamType::Int8, ParamType::Int8)
    );
    assert!((params.get("MOT_THST_EXPO").unwrap().as_f64() - 0.5).abs() < 1e-9);

    wasm_thread::sleep(fast().param_cache_write * 10);
    assert_eq!(
        board.list_requests.load(Ordering::Acquire),
        0,
        "nothing asked of the vehicle"
    );
    assert_eq!(std::fs::read_to_string(&file).unwrap(), by_the_csharp);

    drop(link);
    stop(&board, vehicle);
}

/// The file is read only with the setting set - the C#'s default is off, and nothing sets it -
/// only once the board is known, and only while it is under an hour old (or dated after now, as
/// the C#'s comparison has it); a file that is not a `MAVLinkParamList` is the C#'s exception.
#[test]
fn the_file_is_read_only_with_the_setting_and_only_while_fresh() {
    let directory = directory("gates");
    let file = cache_file(&directory);
    std::fs::create_dir_all(file.parent().unwrap()).unwrap();
    std::fs::write(&file, WRITTEN_BY_THE_CSHARP).unwrap();
    let dated = |age_seconds: i64| {
        let now = std::time::SystemTime::now();
        let when = if age_seconds >= 0 {
            now - Duration::from_secs(age_seconds.unsigned_abs())
        } else {
            now + Duration::from_secs(age_seconds.unsigned_abs())
        };
        std::fs::File::options()
            .write(true)
            .open(&file)
            .unwrap()
            .set_modified(when)
            .unwrap();
    };

    // Without the setting: not read, the table untouched.
    let board = Board::new(stream(), false, 0);
    let (link, vehicle) = start(&directory, false, fast(), &board, FakeVehicle::new());
    board.announce.store(true, Ordering::Release);
    wait_for("AUTOPILOT_VERSION", || board_known(&link));
    assert_eq!(link.read_cached_params(VEHICLE), Ok(None));
    assert!(link.params(VEHICLE).is_none());
    drop(link);
    stop(&board, vehicle);

    // With it: not before the board is known; then by the file's age. A second read replaces
    // names the table holds, which queues a write as the C#'s `Add` does; the timer is an hour
    // here, so that write cannot land among the files this test puts there.
    let board = Board::new(stream(), false, 0);
    let timeouts = ProtocolTimeouts {
        param_cache_write: Duration::from_secs(3600),
        ..fast()
    };
    let (link, vehicle) = start(&directory, true, timeouts, &board, FakeVehicle::new());
    assert_eq!(link.read_cached_params(VEHICLE), Ok(None), "no uid2, no path");
    board.announce.store(true, Ordering::Release);
    wait_for("AUTOPILOT_VERSION", || board_known(&link));
    dated(2 * 3600);
    assert_eq!(link.read_cached_params(VEHICLE), Ok(None), "two hours old");
    dated(3600 + 1);
    assert_eq!(link.read_cached_params(VEHICLE), Ok(None), "an hour old");
    assert!(link.params(VEHICLE).is_none());
    dated(-3600);
    assert_eq!(link.read_cached_params(VEHICLE), Ok(Some(3)), "dated ahead");
    dated(59 * 60);
    assert_eq!(link.read_cached_params(VEHICLE), Ok(Some(3)), "59 minutes");

    std::fs::write(&file, "{}").unwrap();
    assert!(link.read_cached_params(VEHICLE).is_err(), "not a list");
    std::fs::write(&file, "[]").unwrap();
    assert_eq!(link.read_cached_params(VEHICLE), Ok(Some(0)));
    std::fs::remove_file(&file).unwrap();
    assert_eq!(link.read_cached_params(VEHICLE), Ok(None), "no file");

    drop(link);
    stop(&board, vehicle);
}
