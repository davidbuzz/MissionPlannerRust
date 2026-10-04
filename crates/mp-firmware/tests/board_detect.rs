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

//! Board detection: `BoardDetect.DetectBoard` against the C#'s own test cases, USB descriptor
//! fixtures for every rule it has, and its live probes against the px4 mock.
//!
//! Deliverable 13's definition of done names this file. The C# test cases are
//! `testdata/boards/detect_board_tests.json`, their inputs exactly as `BoardDetectTests.cs` writes
//! them; the rule fixtures are `testdata/boards/usb_descriptors.json`, built from the constants in
//! `BoardDetect.cs`. Every case asserts what `BoardDetect.cs` does, and the C# test cases also
//! record whether the C# test's own assertion holds - five of them do not, against the C# itself,
//! and this file checks that the reasons recorded for that follow from the rules.

#![allow(clippy::indexing_slicing, clippy::expect_used, clippy::unwrap_used)]

mod common;

use common::MockBootloader;
use mp_firmware::detect::{
    Boards, DetectHost, Detected, DeviceInfo, FLASH_SIZE_2MB, PLEASE_UNPLUG_THE_BOARD_AND, Probe,
    Runtime, STK500V1_GET_SYNC, STK500V1_IN_SYNC_OK, STK500V2_LOAD_ADDRESS, STK500V2_OK,
    TransportPort, Verdict, Win32SerialPort, bootloader_board, detect_board, match_devices,
    match_ports, match_serial_ports, read_stk500v2_packet, stk500v2_board, stk500v2_packet,
};
use mp_firmware::uploader::Board;
use mp_transport::{PortInfo, Transport};
use serde_json::Value;
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

// --- fixtures -------------------------------------------------------------------------------

fn fixture(name: &str) -> Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/boards")
        .join(name);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    serde_json::from_str(&text).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
}

/// A C# string field: JSON null is the C# null.
fn text(value: &Value) -> Option<String> {
    value.as_str().map(str::to_owned)
}

fn device(value: &Value) -> DeviceInfo {
    DeviceInfo {
        name: text(&value["name"]),
        description: text(&value["description"]),
        board: text(&value["board"]),
        hardwareid: text(&value["hardwareid"]),
    }
}

fn devices(value: &Value) -> Vec<DeviceInfo> {
    value
        .as_array()
        .expect("a device list")
        .iter()
        .map(device)
        .collect()
}

fn rows(value: &Value) -> Vec<Win32SerialPort> {
    value
        .as_array()
        .expect("a Win32_SerialPort table")
        .iter()
        .map(|row| {
            Win32SerialPort::new(
                row["pnp_device_id"].as_str().expect("PNPDeviceID"),
                row["name"].as_str().expect("Name"),
            )
        })
        .collect()
}

fn board(name: &str) -> Boards {
    Boards::from_name(name).unwrap_or_else(|| panic!("{name} is not a C# board name"))
}

fn verdict(value: &Value) -> Verdict {
    if value.as_str() == Some("no_match") {
        return Verdict::NoMatch;
    }
    if let Some(probe) = value["probe"].as_str() {
        return Verdict::Probe(match probe {
            "fmu_v2_or_v3" => Probe::FmuV2OrV3,
            "chibios_or_px4" => Probe::ChibiosOrPx4,
            other => panic!("unknown probe {other}"),
        });
    }
    Verdict::Board(Detected {
        board: board(value["board"].as_str().expect("a board")),
        chbootloader: text(&value["chbootloader"]),
    })
}

/// Whether a board satisfies what a C# test asserts about it.
fn satisfies(assertion: &Value, result: Boards) -> bool {
    if let Some(expected) = assertion["equals"].as_str() {
        return result == board(expected);
    }
    if let Some(unwanted) = assertion["not"].as_str() {
        return result != board(unwanted);
    }
    panic!("unknown assertion {assertion}")
}

/// Everything a replug-and-probe can return: `BoardDetect.cs:169-189` and `:265-290`.
fn probe_outcomes(probe: Probe) -> &'static [Boards] {
    match probe {
        Probe::FmuV2OrV3 => &[Boards::Px4v3, Boards::Px4v2, Boards::None],
        Probe::ChibiosOrPx4 => &[Boards::Px4v3, Boards::Fmuv5, Boards::Px4v2, Boards::None],
    }
}

// --- the C# test cases ------------------------------------------------------------------------

#[test]
fn every_detect_board_test_case_decides_as_board_detect_cs_does() {
    let file = fixture("detect_board_tests.json");
    let cases = file["cases"].as_array().expect("cases");
    // DetectBoardTest, DetectBoardTestMany's six calls, DetectBoardTest1 to 9.
    assert_eq!(cases.len(), 16);

    let mut tally = std::collections::BTreeMap::<String, usize>::new();
    for case in cases {
        let name = case["test"].as_str().expect("test name");
        let port = case["port"].as_str().expect("port");
        assert_eq!(port, "com1", "{name}: every C# case asks about com1");

        let from_list = match_devices(&devices(&case["devices"]));
        assert_eq!(
            from_list,
            verdict(&case["device_rules"]),
            "{name}: the device list"
        );
        if from_list == Verdict::NoMatch {
            let from_wmi = match_serial_ports(port, &rows(&case["serial_ports"]));
            assert_eq!(
                from_wmi,
                verdict(&case["serial_port_rules"]),
                "{name}: the Win32_SerialPort row for the same device"
            );
        } else {
            assert!(
                case.get("serial_ports").is_none(),
                "{name}: the list decided, so WMI is never read"
            );
        }

        // The C# result recorded for the case has to follow from the rules.
        let assertion = &case["csharp_asserts"];
        let recorded = case["csharp_result"].as_str().expect("csharp_result");
        match recorded {
            "passes" => match &from_list {
                Verdict::Board(found) => assert!(
                    satisfies(assertion, found.board),
                    "{name}: recorded as passing, but {} does not satisfy {assertion}",
                    found.board
                ),
                other => panic!("{name}: recorded as passing, but the list gives {other:?}"),
            },
            "fails" => match &from_list {
                Verdict::Board(found) => assert!(
                    !satisfies(assertion, found.board),
                    "{name}: recorded as failing, but {} satisfies {assertion}",
                    found.board
                ),
                // The prompt throws in the test process. In the application the probe can only
                // return these, and none of them is what the test expects.
                Verdict::Probe(probe) => assert!(
                    probe_outcomes(*probe)
                        .iter()
                        .all(|outcome| !satisfies(assertion, *outcome)),
                    "{name}: recorded as failing, but a probe could satisfy {assertion}"
                ),
                Verdict::NoMatch => panic!("{name}: a list that decides nothing reads live WMI"),
            },
            "machine-dependent" => assert_eq!(
                from_list,
                Verdict::NoMatch,
                "{name}: only a case the list leaves to live WMI depends on the machine"
            ),
            "unchecked" => assert!(assertion.is_null(), "{name}: has an assertion"),
            other => panic!("{name}: unknown csharp_result {other}"),
        }
        *tally.entry(recorded.to_owned()).or_default() += 1;
    }

    // The summary the report gives, pinned: five pass, five fail against BoardDetect.cs itself,
    // five depend on what is plugged into the machine running them, one asserts nothing.
    let tally: Vec<(&str, usize)> = tally.iter().map(|(k, v)| (k.as_str(), *v)).collect();
    assert_eq!(
        tally,
        [
            ("fails", 5),
            ("machine-dependent", 5),
            ("passes", 5),
            ("unchecked", 1)
        ]
    );
}

// --- the rule fixtures ------------------------------------------------------------------------

#[test]
fn every_device_list_rule_decides_as_the_c_sharp_does() {
    let file = fixture("usb_descriptors.json");
    let entries = file["devices"].as_array().expect("devices");
    assert!(entries.len() >= 30);
    for entry in entries {
        let about = entry["about"].as_str().expect("about");
        assert_eq!(
            match_devices(&devices(&entry["devices"])),
            verdict(&entry["expect"]),
            "BoardDetect.cs:{} - {about}",
            entry["line"].as_str().unwrap_or("?")
        );
    }
}

#[test]
fn every_win32_serial_port_rule_decides_as_the_c_sharp_does() {
    let file = fixture("usb_descriptors.json");
    let entries = file["serial_ports"].as_array().expect("serial_ports");
    for entry in entries {
        let about = entry["about"].as_str().expect("about");
        assert_eq!(
            match_serial_ports(entry["port"].as_str().expect("port"), &rows(&entry["rows"])),
            verdict(&entry["expect"]),
            "BoardDetect.cs:{} - {about}",
            entry["line"].as_str().unwrap_or("?")
        );
    }
}

/// Every id in the WMI table has a fixture, so a rule dropped from the table fails a case.
#[test]
fn the_rule_fixtures_cover_every_id_in_the_table() {
    let file = fixture("usb_descriptors.json");
    let fixture_text = file["serial_ports"].to_string();
    for rule in mp_firmware::detect::SERIAL_PORT_RULES {
        for id in rule.ids {
            let escaped = id.replace('\\', "\\\\");
            assert!(
                fixture_text.contains(&escaped),
                "no Win32_SerialPort fixture for {id}"
            );
        }
    }
    // And every board the device list names outright.
    let devices_text = file["devices"].to_string();
    for name in ["px4v2", "px4v3", "px4v4", "fmuv5", "px4", "chbootloader"] {
        assert!(
            devices_text.contains(&format!("\"board\":\"{name}\"")),
            "{name}"
        );
    }
}

// --- from an enumeration ----------------------------------------------------------------------

fn usb(name: &str, vid: u16, pid: u16, product: &str) -> PortInfo {
    PortInfo {
        name: name.to_owned(),
        vid: Some(vid),
        pid: Some(pid),
        serial_number: Some("0123456789".to_owned()),
        manufacturer: None,
        product: Some(product.to_owned()),
        hardware_id: None,
        description: None,
    }
}

#[test]
fn an_enumerated_port_becomes_the_record_windows_gives_mission_planner() {
    let cube = usb("/dev/ttyACM0", 0x2dae, 0x1016, "CubeOrange");
    let record = DeviceInfo::from_port(&cube).expect("a USB device is listed");
    assert_eq!(record.hardwareid.as_deref(), Some(r"USB\VID_2DAE&PID_1016"));
    assert_eq!(record.board.as_deref(), Some("CubeOrange"));
    assert_eq!(record.name.as_deref(), Some("/dev/ttyACM0"));

    let row = Win32SerialPort::from_port(&cube);
    assert_eq!(row.pnp_device_id, r"USB\VID_2DAE&PID_1016\0123456789");
    assert_eq!(row.name, "CubeOrange (/dev/ttyACM0)");

    // A port with no USB ids is in neither of Mission Planner's device lists.
    assert_eq!(
        DeviceInfo::from_port(&PortInfo::bare("/dev/ttyS0".to_owned())),
        None
    );
}

#[test]
fn an_enumeration_is_decided_by_the_list_then_by_wmi() {
    // A ChibiOS bootloader names itself from its product string.
    assert_eq!(
        match_ports(
            "/dev/ttyACM0",
            &[usb("/dev/ttyACM0", 0x2dae, 0x1016, "CubeOrange-BL")]
        ),
        Verdict::Board(Detected::chbootloader("CubeOrange"))
    );
    // A px4 bootloader's product string is unknown to the list, and its id decides in WMI.
    assert_eq!(
        match_ports(
            "/dev/ttyACM0",
            &[usb("/dev/ttyACM0", 0x26ac, 0x0012, "PX4 BL FMU v4.x")]
        ),
        Verdict::Board(Detected::board(Boards::Px4v4))
    );
    // A port that is not USB is not in the device list, so it cannot hide the board behind it
    // the way a null hardware id would (BoardDetect.cs:82, :194).
    assert_eq!(
        match_ports(
            "/dev/ttyACM0",
            &[
                PortInfo::bare("/dev/ttyS0".to_owned()),
                usb("/dev/ttyACM0", 0x2dae, 0x1016, "CubeOrange")
            ]
        ),
        Verdict::Board(Detected::chbootloader("CubeOrange"))
    );
    assert_eq!(match_ports("/dev/ttyACM0", &[]), Verdict::NoMatch);
}

// --- the bootloader's answer ------------------------------------------------------------------

fn answered(board_id: u32, flash_size: usize, bootloader_revision: u32) -> Board {
    Board {
        bootloader_revision,
        board_id,
        board_revision: 0,
        flash_size,
    }
}

#[test]
fn the_bootloader_answer_decides_px4v3_fmuv5_or_px4v2() {
    use Probe::{ChibiosOrPx4, FmuV2OrV3};
    let cases = [
        // BoardDetect.cs:169-178
        (FmuV2OrV3, answered(9, FLASH_SIZE_2MB, 5), Boards::Px4v3),
        (FmuV2OrV3, answered(9, FLASH_SIZE_2MB, 4), Boards::Px4v2),
        (FmuV2OrV3, answered(9, 1_032_192, 5), Boards::Px4v2),
        (FmuV2OrV3, answered(50, FLASH_SIZE_2MB, 5), Boards::Px4v2),
        // BoardDetect.cs:265-279
        (ChibiosOrPx4, answered(9, FLASH_SIZE_2MB, 5), Boards::Px4v3),
        (ChibiosOrPx4, answered(50, FLASH_SIZE_2MB, 5), Boards::Fmuv5),
        (ChibiosOrPx4, answered(50, FLASH_SIZE_2MB, 4), Boards::Px4v2),
        (
            ChibiosOrPx4,
            answered(140, FLASH_SIZE_2MB, 5),
            Boards::Px4v2,
        ),
    ];
    for (probe, answer, expected) in cases {
        assert_eq!(
            bootloader_board(probe, &answer),
            expected,
            "{probe:?} {answer:?}"
        );
    }
}

#[test]
fn an_stk500v2_answer_is_an_apm_2_only_where_wmi_names_the_port() {
    let mega = |port: &str| {
        Win32SerialPort::new(
            r"USB\VID_2341&PID_0010\640333439373519060F0",
            &format!("USB Serial Port ({port})"),
        )
    };
    assert_eq!(stk500v2_board("COM3", &[mega("COM3")]), Boards::B2560v2);
    assert_eq!(stk500v2_board("COM3", &[mega("COM4")]), Boards::B2560);
    assert_eq!(stk500v2_board("COM3", &[]), Boards::B2560);
}

// --- STK500 framing ---------------------------------------------------------------------------

#[test]
fn an_stk500v2_reply_is_read_past_leading_noise() {
    let mut wire: &[u8] = &[0x00, 0x55, 0x1b, 0x01, 0x00, 0x02, 0x0e, 0x06, 0x00, 0x13];
    assert_eq!(read_stk500v2_packet(&mut wire), STK500V2_OK);
    assert!(wire.is_empty(), "the checksum is consumed");

    // A reply built by the same framing reads back to its message.
    let framed = stk500v2_packet(&[0x01, 0x02, 0x03]);
    assert_eq!(read_stk500v2_packet(&mut framed.as_slice()), [1, 2, 3]);
}

#[test]
fn a_silent_or_short_stk500v2_reply_is_the_c_sharp_default() {
    let read = |bytes: &[u8]| read_stk500v2_packet(&mut &bytes[..]);
    // Nothing: the C#'s initial `{ 0x0, 0xC0 }` (BoardDetect.cs:615).
    assert_eq!(read(&[]), [0x00, 0xC0]);
    // Cut off before the length: still the default.
    assert_eq!(read(&[0x1b, 0x01, 0x00]), [0x00, 0xC0]);
    // Cut off after the length: the message is sized, zero-filled and only partly read.
    assert_eq!(read(&[0x1b, 0x01, 0x00, 0x02, 0x0e, 0x07]), [0x07, 0x00]);
    // Which means a reply cut off after its echoed command byte reads as STATUS_CMD_OK, and the
    // C# takes it for a 2560 (BoardDetect.cs:483). Kept: the rule has to match.
    assert_eq!(read(&[0x1b, 0x01, 0x00, 0x02, 0x0e, 0x06]), STK500V2_OK);
}

// --- the whole of DetectBoard over a scripted host ---------------------------------------------

/// What answers on a port.
enum Device {
    /// The px4 bootloader mock.
    Px4(MockBootloader),
    /// Answers every STK500v1 sync.
    Stk500v1,
    /// Answers every STK500v2 load-address.
    Stk500v2,
    /// Says nothing.
    Silent,
}

/// A device on the end of an `mp-transport` link, with everything written to it recorded.
struct MockLink {
    device: Device,
    outgoing: VecDeque<u8>,
    sent: Arc<Mutex<Vec<u8>>>,
}

impl Transport for MockLink {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if let Device::Px4(mock) = &mut self.device {
            return match mock.read(buf) {
                Err(error) if error.kind() == io::ErrorKind::TimedOut => Ok(0),
                other => other,
            };
        }
        let count = buf.len().min(self.outgoing.len());
        for (slot, byte) in buf.iter_mut().zip(self.outgoing.drain(..count)) {
            *slot = byte;
        }
        Ok(count)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.sent.lock().unwrap().extend_from_slice(buf);
        match &mut self.device {
            Device::Px4(mock) => mock.write_all(buf)?,
            Device::Stk500v1 => {
                if buf == STK500V1_GET_SYNC {
                    self.outgoing.extend(STK500V1_IN_SYNC_OK);
                }
            }
            Device::Stk500v2 => {
                if buf == stk500v2_packet(&STK500V2_LOAD_ADDRESS) {
                    self.outgoing.extend(stk500v2_packet(&STK500V2_OK));
                }
            }
            Device::Silent => {}
        }
        Ok(())
    }

    fn description(&self) -> &str {
        "mock"
    }

    fn is_open(&self) -> bool {
        true
    }

    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        Ok(())
    }
}

/// One port opened during a run.
#[derive(Debug)]
struct Opened {
    port: String,
    baud: u32,
    dtr: bool,
    sent: Arc<Mutex<Vec<u8>>>,
}

impl Opened {
    fn sent(&self) -> Vec<u8> {
        self.sent.lock().unwrap().clone()
    }
}

type Wiring = Box<dyn FnMut(&str, u32) -> Option<Device>>;

/// A person answering from a script, a WMI table, a port list, ports and a clock that only moves
/// when told to.
struct ScriptedHost {
    runtime: Runtime,
    answers: VecDeque<bool>,
    asked: Vec<String>,
    shown: Vec<String>,
    rows: Vec<Win32SerialPort>,
    wmi_reads: usize,
    /// `GetPortNames()` answers in turn; the last one repeats.
    port_lists: VecDeque<Vec<String>>,
    wiring: Wiring,
    opened: Vec<Opened>,
    start: Instant,
    elapsed: Duration,
}

impl ScriptedHost {
    fn new() -> Self {
        Self {
            runtime: Runtime::DotNet,
            answers: VecDeque::new(),
            asked: Vec::new(),
            shown: Vec::new(),
            rows: Vec::new(),
            wmi_reads: 0,
            port_lists: VecDeque::from([Vec::new()]),
            wiring: Box::new(|_, _| None),
            opened: Vec::new(),
            start: Instant::now(),
            elapsed: Duration::ZERO,
        }
    }

    fn answering(mut self, answers: &[bool]) -> Self {
        self.answers = answers.iter().copied().collect();
        self
    }

    fn with_rows(mut self, rows: Vec<Win32SerialPort>) -> Self {
        self.rows = rows;
        self
    }

    fn with_ports(mut self, lists: &[&[&str]]) -> Self {
        self.port_lists = lists
            .iter()
            .map(|list| list.iter().map(|name| (*name).to_owned()).collect())
            .collect();
        self
    }

    fn wired(mut self, wiring: impl FnMut(&str, u32) -> Option<Device> + 'static) -> Self {
        self.wiring = Box::new(wiring);
        self
    }
}

impl DetectHost for ScriptedHost {
    type Port = TransportPort<MockLink>;

    fn runtime(&self) -> Runtime {
        self.runtime
    }

    fn show(&mut self, text: &str) {
        self.shown.push(text.to_owned());
    }

    fn ask(&mut self, text: &str, caption: &str) -> bool {
        self.asked.push(format!("{caption}: {text}"));
        self.answers
            .pop_front()
            .unwrap_or_else(|| panic!("asked more than the script answers: {text}"))
    }

    fn win32_serial_ports(&mut self) -> Vec<Win32SerialPort> {
        self.wmi_reads += 1;
        self.rows.clone()
    }

    fn port_names(&mut self) -> Vec<String> {
        if self.port_lists.len() > 1 {
            self.port_lists.pop_front().unwrap_or_default()
        } else {
            self.port_lists.front().cloned().unwrap_or_default()
        }
    }

    fn open(&mut self, port: &str, baud: u32, dtr: bool) -> io::Result<Self::Port> {
        let Some(device) = (self.wiring)(port, baud) else {
            return Err(io::Error::new(io::ErrorKind::NotFound, "no such port"));
        };
        let sent = Arc::new(Mutex::new(Vec::new()));
        self.opened.push(Opened {
            port: port.to_owned(),
            baud,
            dtr,
            sent: Arc::clone(&sent),
        });
        Ok(TransportPort::new(MockLink {
            device,
            outgoing: VecDeque::new(),
            sent,
        }))
    }

    fn now(&mut self) -> Instant {
        // Time passes while the C# spins through its port list.
        self.elapsed += Duration::from_millis(250);
        self.start + self.elapsed
    }

    fn sleep(&mut self, duration: Duration) {
        self.elapsed += duration;
    }
}

fn case(name: &str) -> Value {
    let file = fixture("detect_board_tests.json");
    file["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .find(|case| case["test"] == name)
        .unwrap_or_else(|| panic!("no case {name}"))
        .clone()
}

fn px4(board_id: u32, flash_size: usize) -> Device {
    Device::Px4(MockBootloader::new(board_id, flash_size))
}

/// DetectBoardTest3, 4, 6 and 8 reach the replug prompt. Here the prompt is answered, and the
/// board is read from the bootloader as the application would read it.
#[test]
fn the_c_sharp_replug_cases_read_the_board_from_the_bootloader() {
    for name in [
        "DetectBoardTest3",
        "DetectBoardTest4",
        "DetectBoardTest6",
        "DetectBoardTest8",
    ] {
        let devices = devices(&case(name)["devices"]);
        for (answer, expected) in [
            (px4(9, FLASH_SIZE_2MB), Boards::Px4v3),
            (px4(9, 1_032_192), Boards::Px4v2),
            // This copy of the probe does not know fmuv5 (BoardDetect.cs:169-178).
            (px4(50, FLASH_SIZE_2MB), Boards::Px4v2),
        ] {
            let mut answer = Some(answer);
            let mut host = ScriptedHost::new()
                .with_ports(&[&["COM1", "COM7"]])
                .wired(move |port, _| if port == "COM7" { answer.take() } else { None });
            let found = detect_board("com1", &devices, &mut host).expect("no error escapes");

            assert_eq!(found, Detected::board(expected), "{name}");
            assert_eq!(host.shown, [PLEASE_UNPLUG_THE_BOARD_AND], "{name}");
            assert!(host.asked.is_empty(), "{name}: {:?}", host.asked);
            assert_eq!(host.wmi_reads, 0, "{name}: the list decided");
            // COM1 does not open; COM7 opens at the bootloader's rate with .NET's default DTR.
            assert_eq!(host.opened.len(), 1, "{name}");
            let opened = &host.opened[0];
            assert_eq!(
                (opened.port.as_str(), opened.baud, opened.dtr),
                ("COM7", 115_200, false)
            );
            // `identify()`, and nothing else: GET_SYNC, then BL_REV, BOARD_ID, BOARD_REV,
            // FLASH_SIZE; and, as the mock is a revision 5 bootloader, GET_CHIP, GET_CHIP_DES,
            // GET_SN at 0, 4 and 8, and EXTF_SIZE.
            // `// C#: ExtLibs/px4uploader/Uploader.cs:867-921`
            assert_eq!(
                opened.sent(),
                [
                    0x21, 0x20, 0x22, 1, 0x20, 0x22, 2, 0x20, 0x22, 3, 0x20, 0x22, 4, 0x20, 0x2c,
                    0x20, 0x2e, 0x20, 0x2b, 0, 0, 0, 0, 0x20, 0x2b, 4, 0, 0, 0, 0x20, 0x2b, 8, 0,
                    0, 0, 0x20, 0x22, 6, 0x20
                ],
                "{name}"
            );
        }
    }
}

#[test]
fn nothing_answering_the_replug_probe_is_none_after_thirty_seconds() {
    let devices = devices(&case("DetectBoardTest3")["devices"]);
    let mut host = ScriptedHost::new()
        .with_ports(&[&["COM7"]])
        .wired(|_, _| Some(Device::Silent));
    let found = detect_board("com1", &devices, &mut host).expect("no error escapes");

    assert_eq!(found, Detected::board(Boards::None));
    assert!(
        host.elapsed >= Duration::from_secs(30),
        "{:?}",
        host.elapsed
    );
    assert!(
        host.opened.len() > 10,
        "it keeps trying for the whole window, not once: {}",
        host.opened.len()
    );
    assert!(
        host.asked.is_empty(),
        "none is not followed by questions here"
    );
}

/// The list is fetched afresh each pass, because a replugged board can come back as a new port.
#[test]
fn a_board_that_comes_back_on_a_new_port_is_found_there() {
    let devices = devices(&case("DetectBoardTest6")["devices"]);
    let mut answer = Some(px4(9, FLASH_SIZE_2MB));
    let mut host = ScriptedHost::new()
        .with_ports(&[&["COM3"], &["COM3"], &["COM3", "COM12"]])
        .wired(move |port, _| match port {
            "COM3" => Some(Device::Silent),
            "COM12" => answer.take(),
            _ => None,
        });
    let found = detect_board("com1", &devices, &mut host).expect("no error escapes");
    assert_eq!(found, Detected::board(Boards::Px4v3));
    assert_eq!(host.opened.last().map(|o| o.port.as_str()), Some("COM12"));
}

#[test]
fn detect_board_test5_is_fmuv5_without_opening_or_asking_anything() {
    let devices = devices(&case("DetectBoardTest5")["devices"]);
    let mut host = ScriptedHost::new();
    let found = detect_board("com1", &devices, &mut host).expect("no error escapes");
    assert_eq!(found, Detected::board(Boards::Fmuv5));
    assert!(host.opened.is_empty() && host.asked.is_empty() && host.shown.is_empty());
    assert_eq!(host.wmi_reads, 0);
}

/// DetectBoardTestMany's px4 bootloader strings, with the device on the machine: WMI decides.
#[test]
fn detect_board_test_many_is_decided_by_wmi_when_the_device_is_there() {
    for (name, expected) in [
        ("DetectBoardTestMany[2]", Detected::board(Boards::Px4v4)),
        ("DetectBoardTestMany[3]", Detected::board(Boards::Px4v4pro)),
        ("DetectBoardTestMany[4]", Detected::chbootloader("CUAVv5")),
    ] {
        let case = case(name);
        let mut host = ScriptedHost::new().with_rows(rows(&case["serial_ports"]));
        let found =
            detect_board("com1", &devices(&case["devices"]), &mut host).expect("no error escapes");
        assert_eq!(found, expected, "{name}");
        assert!(
            satisfies(&case["csharp_asserts"], found.board),
            "{name}: the C# assertion holds"
        );
        assert_eq!(host.wmi_reads, 1);
        assert!(host.opened.is_empty() && host.asked.is_empty() && host.shown.is_empty());
    }
}

/// DetectBoardTestMany[0]: the WMI copy of the probe does know fmuv5.
#[test]
fn the_wmi_probe_reads_px4v3_fmuv5_or_px4v2_from_the_bootloader() {
    let case = case("DetectBoardTestMany[0]");
    for (answer, expected) in [
        (px4(9, FLASH_SIZE_2MB), Boards::Px4v3),
        (px4(50, FLASH_SIZE_2MB), Boards::Fmuv5),
        (px4(50, 1_032_192), Boards::Px4v2),
    ] {
        let mut answer = Some(answer);
        let mut host = ScriptedHost::new()
            .with_rows(rows(&case["serial_ports"]))
            .with_ports(&[&["COM1"]])
            .wired(move |_, _| answer.take());
        let found =
            detect_board("com1", &devices(&case["devices"]), &mut host).expect("no error escapes");
        assert_eq!(found, Detected::board(expected));
        assert!(satisfies(&case["csharp_asserts"], found.board));
        assert_eq!(host.shown, [PLEASE_UNPLUG_THE_BOARD_AND]);
    }
}

/// Under Mono the C# reads nothing, not even a device list that would decide on Windows.
#[test]
fn under_mono_only_questions_are_asked() {
    let decisive = devices(&case("DetectBoardTest1")["devices"]);
    let apm2 = "APM 2+: Is this a APM 2+?";
    let px4 = "PX4/PIXHAWK: Is this a CUBE/PX4/PIXHAWK/PIXRACER?";
    let pixracer = "PIXRACER: Is this a PIXRACER?";
    let cube = "CUBE: Is this a CUBE?";
    let pixhawk = "PIXHAWK: Is this a PIXHAWK?";
    let cases: [(&[bool], Boards, &[&str]); 6] = [
        (&[true], Boards::B2560v2, &[apm2]),
        (&[false, false], Boards::B2560, &[apm2, px4]),
        (&[false, true, true], Boards::Px4v4, &[apm2, px4, pixracer]),
        (
            &[false, true, false, true],
            Boards::Px4v3,
            &[apm2, px4, pixracer, cube],
        ),
        (
            &[false, true, false, false, true],
            Boards::Px4v2,
            &[apm2, px4, pixracer, cube, pixhawk],
        ),
        (
            &[false, true, false, false, false],
            Boards::Px4,
            &[apm2, px4, pixracer, cube, pixhawk],
        ),
    ];
    for (answers, expected, questions) in cases {
        let mut host = ScriptedHost::new().answering(answers);
        host.runtime = Runtime::Mono;
        let found = detect_board("com1", &decisive, &mut host).expect("no error escapes");
        assert_eq!(found, Detected::board(expected), "{answers:?}");
        assert_eq!(host.asked, questions, "{answers:?}");
        assert!(host.opened.is_empty());
        assert_eq!(host.wmi_reads, 0);
    }
}

#[test]
fn a_linux_board_is_named_by_the_operator() {
    let linux = "Linux: Is this a Linux board?";
    let bebop = "Bebop2: Is this Bebop2?";
    let cases: [(&[bool], Boards, &[&str]); 2] = [
        (&[true, true], Boards::Bebop2, &[linux, bebop]),
        (
            &[true, false, true],
            Boards::Disco,
            &[linux, bebop, "Disco: Is this Disco?"],
        ),
    ];
    for (answers, expected, questions) in cases {
        let mut host = ScriptedHost::new().answering(answers);
        let found = detect_board("COM3", &[], &mut host).expect("no error escapes");
        assert_eq!(found, Detected::board(expected));
        assert_eq!(host.asked, questions);
        assert_eq!(host.wmi_reads, 1, "WMI is read before the question");
        assert!(host.opened.is_empty(), "the port is not touched");
    }
}

#[test]
fn an_apm1_1280_answers_the_stk500v1_sync_at_57600() {
    let mut host = ScriptedHost::new()
        .answering(&[false])
        .wired(|port, baud| (port == "COM3" && baud == 57_600).then_some(Device::Stk500v1));
    let found = detect_board("COM3", &[], &mut host).expect("no error escapes");

    assert_eq!(found, Detected::board(Boards::B1280));
    assert_eq!(host.asked, ["Linux: Is this a Linux board?"]);
    assert_eq!(host.opened.len(), 1);
    let opened = &host.opened[0];
    assert_eq!((opened.baud, opened.dtr), (57_600, true));
    assert_eq!(opened.sent(), b"0 ", "one sync, answered");
}

#[test]
fn a_2560_answers_the_stk500v2_load_address_at_115200() {
    let mut host = ScriptedHost::new()
        .answering(&[false])
        .wired(|port, baud| match (port, baud) {
            ("COM3", 57_600) => Some(Device::Silent),
            ("COM3", 115_200) => Some(Device::Stk500v2),
            _ => None,
        });
    let found = detect_board("COM3", &[], &mut host).expect("no error escapes");

    // WMI is read again after the answer (BoardDetect.cs:489), and it was read once already and
    // found nothing for this port, so this path can only say b2560.
    assert_eq!(found, Detected::board(Boards::B2560));
    assert_eq!(host.wmi_reads, 2);
    assert_eq!(host.opened.len(), 2);

    let v1 = &host.opened[0];
    assert_eq!((v1.baud, v1.dtr), (57_600, true));
    assert_eq!(v1.sent(), b"0 ".repeat(20), "twenty syncs, none answered");

    let v2 = &host.opened[1];
    assert_eq!((v2.baud, v2.dtr), (115_200, true));
    assert_eq!(
        v2.sent(),
        [
            0x1b, 0x01, 0x00, 0x05, 0x0e, 0x06, 0x00, 0x00, 0x00, 0x00, 0x17
        ],
        "one load-address, answered"
    );
}

#[test]
fn when_nothing_answers_the_operator_is_asked_last() {
    let apm2 = "APM 2+: Is this a APM 2+?";
    let px4 = "PX4/PIXHAWK: Is this a PX4/PIXHAWK?";
    let pixhawk = "PIXHAWK: Is this a PIXHAWK?";
    let linux = "Linux: Is this a Linux board?";
    let cases: [(&[bool], Boards, &[&str]); 4] = [
        (&[false, true], Boards::B2560v2, &[linux, apm2]),
        (&[false, false, false], Boards::B2560, &[linux, apm2, px4]),
        (
            &[false, false, true, true],
            Boards::Px4v2,
            &[linux, apm2, px4, pixhawk],
        ),
        (
            &[false, false, true, false],
            Boards::Px4,
            &[linux, apm2, px4, pixhawk],
        ),
    ];
    for (answers, expected, questions) in cases {
        let mut host = ScriptedHost::new()
            .answering(answers)
            .wired(|_, _| Some(Device::Silent));
        let found = detect_board("COM3", &[], &mut host).expect("no error escapes");
        assert_eq!(found, Detected::board(expected), "{answers:?}");
        assert_eq!(host.asked, questions, "{answers:?}");
        // Twenty syncs at 57600 and four load-addresses at 115200 went unanswered first.
        assert_eq!(host.opened.len(), 2);
        assert_eq!(host.opened[1].sent().len(), 4 * 11);
    }
}

/// Opening the port for the STK500 probes is outside any `try` in the C#: the failure escapes.
#[test]
fn a_port_that_will_not_open_for_the_stk500_probe_is_an_error() {
    let mut host = ScriptedHost::new().answering(&[false]);
    let outcome = detect_board("COM3", &[], &mut host);
    assert_eq!(
        outcome.map_err(|error| error.kind()),
        Err(io::ErrorKind::NotFound)
    );
}

// --- the probe over a real serial port --------------------------------------------------------

/// The bootloader probe over `mp-transport`'s `SerialTransport` on a pseudo-terminal, with the px4
/// mock on the other side: the path a board plugged into this machine would take.
// Linux only: macOS's pseudo-terminals refuse the slave's second open by path with ENOTTY ("Not a
// typewriter", the hosted runner, 2026-10-03), so the mock on the master never meets the detector;
// that is the harness's limit, not the detector's, whose serial path the Linux run holds.
#[cfg(target_os = "linux")]
mod pty {
    use super::*;
    use mp_firmware::detect::{ProbePort, open_serial};
    use serialport::{SerialPort, TTYPort};
    use std::sync::atomic::{AtomicBool, Ordering};

    struct SerialHost {
        path: String,
        shown: Vec<String>,
    }

    impl DetectHost for SerialHost {
        type Port = TransportPort<mp_transport::SerialTransport>;

        fn runtime(&self) -> Runtime {
            Runtime::DotNet
        }
        fn show(&mut self, text: &str) {
            self.shown.push(text.to_owned());
        }
        fn ask(&mut self, text: &str, _caption: &str) -> bool {
            panic!("asked {text}")
        }
        fn win32_serial_ports(&mut self) -> Vec<Win32SerialPort> {
            Vec::new()
        }
        fn port_names(&mut self) -> Vec<String> {
            vec![self.path.clone()]
        }
        fn open(&mut self, port: &str, baud: u32, dtr: bool) -> io::Result<Self::Port> {
            open_serial(port, baud, dtr)
        }
        fn now(&mut self) -> Instant {
            Instant::now()
        }
        fn sleep(&mut self, duration: Duration) {
            wasm_thread::sleep(duration);
        }
    }

    /// Runs the mock on the master side until told to stop, across however many times the probe
    /// opens and closes the slave.
    fn serve(
        mut master: TTYPort,
        stop: Arc<AtomicBool>,
    ) -> wasm_thread::JoinHandle<MockBootloader> {
        wasm_thread::spawn(move || {
            let mut mock = MockBootloader::new(9, FLASH_SIZE_2MB);
            master
                .set_timeout(Duration::from_millis(10))
                .expect("a timeout");
            let mut buffer = [0u8; 64];
            while !stop.load(Ordering::Acquire) {
                match master.read(&mut buffer) {
                    Ok(count) if count > 0 => {
                        mock.write_all(&buffer[..count])
                            .expect("the mock accepts it");
                        let reply: Vec<u8> = mock.outgoing.drain(..).collect();
                        master.write_all(&reply).expect("the reply goes back");
                    }
                    Ok(_) => {}
                    Err(error) if error.kind() == io::ErrorKind::TimedOut => {}
                    // Between probe attempts nothing holds the slave open, and the master reads
                    // EIO until the next attempt opens it: a board waiting to be spoken to.
                    Err(_) => wasm_thread::sleep(Duration::from_millis(1)),
                }
            }
            mock
        })
    }

    #[test]
    fn the_replug_probe_identifies_a_board_over_a_real_serial_port() {
        let (master, slave) = match TTYPort::pair() {
            Ok(pair) => pair,
            Err(error) => {
                println!("no pseudo-terminals here ({error}); skipped");
                return;
            }
        };
        let path = slave.name().expect("a slave path");
        drop(slave);

        let stop = Arc::new(AtomicBool::new(false));
        let mock = serve(master, Arc::clone(&stop));

        let mut host = SerialHost {
            path,
            shown: Vec::new(),
        };
        let devices = devices(&case("DetectBoardTest8")["devices"]);
        let found = detect_board("com1", &devices, &mut host).expect("no error escapes");

        stop.store(true, Ordering::Release);
        let mock = mock.join().expect("the mock thread");
        assert_eq!(found, Detected::board(Boards::Px4v3));
        assert_eq!(host.shown, [PLEASE_UNPLUG_THE_BOARD_AND]);
        assert!(!mock.erased, "detection never erases");
        assert!(mock.flash.is_empty(), "detection never writes flash");
    }

    /// The look-ahead reads of `BytesToRead` hand their bytes on to the next read.
    #[test]
    fn bytes_seen_by_a_look_ahead_are_still_read() {
        let Ok((mut master, slave)) = TTYPort::pair() else {
            println!("no pseudo-terminals here; skipped");
            return;
        };
        let path = slave.name().expect("a slave path");
        drop(slave);
        let mut port = open_serial(&path, 115_200, false).expect("the pty opens");
        port.set_read_timeout(Duration::from_millis(500))
            .expect("timeout");

        master.write_all(&[1, 2, 3]).expect("written");
        let deadline = Instant::now() + Duration::from_secs(5);
        while port.bytes_to_read().expect("look-ahead") < 3 && Instant::now() < deadline {}
        let mut got = [0u8; 3];
        port.read_exact(&mut got).expect("read back");
        assert_eq!(got, [1, 2, 3]);

        master.write_all(&[4, 5]).expect("written");
        let deadline = Instant::now() + Duration::from_secs(5);
        while port.bytes_to_read().expect("look-ahead") < 2 && Instant::now() < deadline {}
        port.discard_in_buffer().expect("discard");
        assert_eq!(port.bytes_to_read().expect("look-ahead"), 0, "discarded");
        let error = port.read(&mut got).expect_err("nothing left to read");
        assert_eq!(error.kind(), io::ErrorKind::TimedOut);
    }
}
