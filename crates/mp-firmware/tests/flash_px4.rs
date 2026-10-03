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

//! `UploadPX4` to a board: the reboot attempt, the port scan and the upload, against the mock
//! bootloader on a bench of pretend serial ports.
//!
//! This is the gate before the first real flash (PLAN.md §13.6 row 79): every word the operator
//! is shown and every byte the board is sent, with the clock and the ports under the test's
//! control.
// The mock indexes into buffers it has just length-checked; `expect` in a test is the report.
#![allow(clippy::indexing_slicing, clippy::expect_used)]

use std::collections::BTreeMap;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mp_firmware::Firmware;
use mp_firmware::detect::{PLEASE_UNPLUG_THE_BOARD_AND, ProbePort};
use mp_firmware::flow::{
    self, Buttons, Cx, Dialogue, FlashHost, LOST_COMMUNICATION, LinkReboot, NO_NEED_TO_UPLOAD,
    NO_RESPONSE_FROM_BOARD, Reached, SAME_FIRMWARE_QUESTION,
};
use mp_firmware::manifest::Fetch;

mod common;
use common::MockBootloader;

/// A pretend serial port: a board's mock behind a lock, so the same board answers every time the
/// scan opens and closes it, or a port with nothing behind it, which swallows every byte and
/// never answers.
enum BenchPort {
    Board(Arc<Mutex<MockBootloader>>),
    Silent,
    /// A board whose port fails after the identify: the write after [`IDENTIFY_WRITES`] and every
    /// read after it are an I/O error.
    Flaky(Arc<Mutex<MockBootloader>>, Arc<Mutex<usize>>),
}

/// The writes of a revision 5 bootloader's `identify()`: the sync, the four infos, `GET_CHIP`,
/// `GET_CHIP_DES`, `GET_SN` at 0, 4 and 8, and `EXTF_SIZE`.
/// `// C#: ExtLibs/px4uploader/Uploader.cs:867-921`
const IDENTIFY_WRITES: usize = 11;

impl Read for BenchPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            Self::Flaky(mock, writes) => {
                if *writes.lock().expect("count") > IDENTIFY_WRITES {
                    return Err(io::Error::other("the cable came out"));
                }
                let mut mock = mock.lock().expect("the bench");
                if mock.outgoing.is_empty() {
                    return Err(io::Error::new(io::ErrorKind::TimedOut, "nothing to read"));
                }
                mock.read(buf)
            }
            Self::Board(mock) => {
                let mut mock = mock.lock().expect("the bench");
                if mock.outgoing.is_empty() {
                    return Err(io::Error::new(io::ErrorKind::TimedOut, "nothing to read"));
                }
                mock.read(buf)
            }
            Self::Silent => Err(io::Error::new(io::ErrorKind::TimedOut, "nothing there")),
        }
    }
}

impl Write for BenchPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            Self::Flaky(mock, writes) => {
                let mut count = writes.lock().expect("count");
                *count += 1;
                if *count > IDENTIFY_WRITES {
                    return Err(io::Error::other("the cable came out"));
                }
                mock.lock().expect("the bench").write(buf)
            }
            Self::Board(mock) => mock.lock().expect("the bench").write(buf),
            Self::Silent => Ok(buf.len()),
        }
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl ProbePort for BenchPort {
    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        Ok(())
    }
    fn discard_in_buffer(&mut self) -> io::Result<()> {
        if let Self::Board(mock) | Self::Flaky(mock, _) = self {
            mock.lock().expect("the bench").outgoing.clear();
        }
        Ok(())
    }
    fn bytes_to_read(&mut self) -> io::Result<usize> {
        Ok(match self {
            Self::Board(mock) | Self::Flaky(mock, _) => {
                mock.lock().expect("the bench").outgoing.len()
            }
            Self::Silent => 0,
        })
    }
}

/// The bench: named ports, some with a board behind them - from the start, or only once the
/// link has been asked to reboot, as a board running firmware appears in its bootloader after
/// `doReboot` - a link that answers as scripted, and a clock that moves a second each time it is
/// read.
struct Bench {
    ports: BTreeMap<String, Option<Arc<Mutex<MockBootloader>>>>,
    /// Boards that answer only after the reboot attempt.
    later: BTreeMap<String, Arc<Mutex<MockBootloader>>>,
    /// Boards whose port breaks after the identify.
    flaky: BTreeMap<String, Arc<Mutex<MockBootloader>>>,
    link: LinkReboot,
    reboots: usize,
    clock: Instant,
    ticks: u32,
    opened: Vec<String>,
}

impl Bench {
    fn new(link: LinkReboot) -> Self {
        Self {
            ports: BTreeMap::new(),
            later: BTreeMap::new(),
            flaky: BTreeMap::new(),
            link,
            reboots: 0,
            clock: Instant::now(),
            ticks: 0,
            opened: Vec::new(),
        }
    }

    fn empty_port(mut self, name: &str) -> Self {
        self.ports.insert(name.to_owned(), None);
        self
    }

    fn board(mut self, name: &str, mock: MockBootloader) -> Self {
        self.ports
            .insert(name.to_owned(), Some(Arc::new(Mutex::new(mock))));
        self
    }

    /// A board running its firmware: silent until the link has been asked to reboot it.
    fn board_after_reboot(mut self, name: &str, mock: MockBootloader) -> Self {
        self.ports.insert(name.to_owned(), None);
        self.later
            .insert(name.to_owned(), Arc::new(Mutex::new(mock)));
        self
    }

    /// A board whose port breaks once the bootloader has been identified.
    fn flaky_board(mut self, name: &str, mock: MockBootloader) -> Self {
        self.ports.insert(name.to_owned(), None);
        self.flaky
            .insert(name.to_owned(), Arc::new(Mutex::new(mock)));
        self
    }

    fn board_at(&self, name: &str) -> Arc<Mutex<MockBootloader>> {
        self.ports
            .get(name)
            .and_then(Option::as_ref)
            .or_else(|| self.later.get(name))
            .map(Arc::clone)
            .expect("a board")
    }
}

impl FlashHost for Bench {
    type Port = BenchPort;

    fn port_names(&mut self) -> Vec<String> {
        self.ports.keys().cloned().collect()
    }
    fn open(&mut self, port: &str, baud: u32) -> io::Result<Self::Port> {
        assert_eq!(baud, 115_200, "the bootloader's baud");
        self.opened.push(port.to_owned());
        if let Some(mock) = self.flaky.get(port) {
            return Ok(BenchPort::Flaky(Arc::clone(mock), Arc::new(Mutex::new(0))));
        }
        match self.ports.get(port) {
            Some(Some(mock)) => Ok(BenchPort::Board(Arc::clone(mock))),
            Some(None) => match self.later.get(port) {
                // Rebooted: its bootloader answers now.
                Some(mock) if self.reboots > 0 => Ok(BenchPort::Board(Arc::clone(mock))),
                // A port with nothing behind it opens and never answers.
                _ => Ok(BenchPort::Silent),
            },
            None => Err(io::Error::new(io::ErrorKind::NotFound, "no such port")),
        }
    }
    fn reboot_link(&mut self) -> LinkReboot {
        self.reboots += 1;
        self.link
    }
    fn now(&mut self) -> Instant {
        self.ticks += 1;
        self.clock + Duration::from_secs(u64::from(self.ticks))
    }
    fn sleep(&mut self, _duration: Duration) {}
}

/// The operator: answers from a script, remembers everything.
#[derive(Default)]
struct Person {
    answers: Vec<bool>,
    asked: Vec<(String, String)>,
    shown: Vec<(String, String)>,
    progress: Vec<(i32, String)>,
}

impl Dialogue for Person {
    fn ask(&mut self, text: &str, caption: &str, _buttons: Buttons) -> bool {
        self.asked.push((text.to_owned(), caption.to_owned()));
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

struct NoNetwork;

impl Fetch for NoNetwork {
    fn get(&self, _url: &str) -> Result<Vec<u8>, String> {
        Err("no network on the bench".to_owned())
    }
}

fn cx<'a>(person: &'a mut Person) -> Cx<'a> {
    Cx {
        dialogue: person,
        fetch: &NoNetwork,
        user_data: std::env::temp_dir().join("mp-flash-test"),
        temp_dir: std::env::temp_dir(),
    }
}

/// A firmware file for board 140 (CubeOrange), as the tools write one.
fn firmware(board_id: u32, image: &[u8]) -> Firmware {
    use base64::Engine as _;
    let mut encoder = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    encoder.write_all(image).expect("compressing");
    let compressed = encoder.finish().expect("finishing");
    let encoded = base64::engine::general_purpose::STANDARD.encode(&compressed);
    let json = format!(
        r#"{{"board_id":{board_id},"image_size":{},"image":"{encoded}","description":"bench","board_revision":0}}"#,
        image.len()
    );
    Firmware::parse(json.as_bytes()).expect("a firmware file")
}

fn statuses(person: &Person) -> Vec<&str> {
    person
        .progress
        .iter()
        .map(|(_, status)| status.as_str())
        .collect()
}

#[test]
fn a_board_on_the_second_port_is_found_after_the_reboot_and_flashed() {
    let image: Vec<u8> = (0..4096u32).map(|i| (i % 253) as u8).collect();
    let fw = firmware(140, &image);
    let mut bench = Bench::new(LinkReboot::Rebooted)
        .empty_port("/dev/ttyUSB0")
        .board_after_reboot("/dev/ttyACM0", MockBootloader::new(140, 2_080_768));
    let mut person = Person::default();
    let mut reached = Reached::default();
    let ok = flow::upload_px4(&mut cx(&mut person), &mut bench, &fw, &mut reached);
    assert!(ok, "{:?} {:?}", person.progress, person.shown);
    // The reboot went out once no bootloader answered the first pass.
    assert_eq!(bench.reboots, 1);
    let seen = statuses(&person);
    assert_eq!(
        &seen[..3],
        &[
            "Look for HeartBeat",
            "Reboot to Bootloader",
            "Scanning comports"
        ]
    );
    assert!(seen.contains(&"/dev/ttyACM0 Identify"), "{seen:?}");
    assert!(seen.contains(&"Connecting"), "{seen:?}");
    assert!(seen.contains(&"Upload"), "{seen:?}");
    assert_eq!(seen.last(), Some(&"Upload Done"));
    assert_eq!(person.progress.last().map(|(p, _)| *p), Some(100));
    assert!(
        person.asked.is_empty(),
        "nothing to ask: the board held other firmware"
    );
    assert!(person.shown.is_empty());
    let flashed = reached.flashed.expect("the board");
    assert_eq!(flashed.board_id, 140);
    // The board holds the image now.
    let mock = bench.board_at("/dev/ttyACM0");
    let mock = mock.lock().expect("bench");
    assert!(mock.erased);
    assert_eq!(&mock.flash[..image.len()], &image[..]);
}

#[test]
fn a_board_already_holding_the_firmware_asks_and_no_is_no_need_to_upload() {
    let image = vec![0x5a; 2048];
    let fw = firmware(140, &image);
    let mut mock = MockBootloader::new(140, 2_080_768);
    // The board's flash already reads back as this firmware.
    mock.crc_override = Some(fw.crc(2_080_768));
    let mut bench = Bench::new(LinkReboot::NotSerial).board("/dev/ttyACM0", mock);
    let mut person = Person::default();
    let mut reached = Reached::default();
    let ok = flow::upload_px4(&mut cx(&mut person), &mut bench, &fw, &mut reached);
    assert!(ok);
    assert_eq!(
        person.asked,
        vec![(
            SAME_FIRMWARE_QUESTION.to_owned(),
            "Same Firmware".to_owned()
        )]
    );
    assert_eq!(
        person.shown,
        vec![(NO_NEED_TO_UPLOAD.to_owned(), String::new())]
    );
    assert!(reached.flashed.is_none());
    let mock = bench.board_at("/dev/ttyACM0");
    assert!(!mock.lock().expect("bench").erased, "nothing was erased");
    // Yes uploads anyway.
    let mut mock = MockBootloader::new(140, 2_080_768);
    mock.crc_override = Some(fw.crc(2_080_768));
    let mut bench = Bench::new(LinkReboot::NotSerial).board("/dev/ttyACM0", mock);
    let mut person = Person {
        answers: vec![true],
        ..Person::default()
    };
    let mut reached = Reached::default();
    assert!(flow::upload_px4(
        &mut cx(&mut person),
        &mut bench,
        &fw,
        &mut reached
    ));
    assert!(reached.flashed.is_some());
    assert_eq!(statuses(&person).last(), Some(&"Upload Done"));
}

#[test]
fn another_boards_bootloader_is_passed_over_and_no_answer_in_thirty_seconds_is_an_error() {
    let fw = firmware(140, &[0u8; 1024]);
    // Nothing answers the first pass; the other board's bootloader is there for the scan.
    let mut bench = Bench::new(LinkReboot::NoHeartbeat)
        .board_after_reboot("/dev/ttyACM1", MockBootloader::new(9, 2_080_768))
        .empty_port("/dev/ttyUSB0");
    let mut person = Person::default();
    let mut reached = Reached::default();
    let ok = flow::upload_px4(&mut cx(&mut person), &mut bench, &fw, &mut reached);
    assert!(!ok);
    // No heartbeat: the replug prompt, as the C# shows it.
    assert_eq!(
        person.shown,
        vec![(PLEASE_UNPLUG_THE_BOARD_AND.to_owned(), String::new())]
    );
    let seen = statuses(&person);
    assert!(seen.contains(&"/dev/ttyACM1 Identify"), "{seen:?}");
    assert!(
        !seen.contains(&"Connecting"),
        "a board of another type is never connected to"
    );
    assert_eq!(seen.last(), Some(&NO_RESPONSE_FROM_BOARD));
    assert_eq!(person.progress.last().map(|(p, _)| *p), Some(0));
    assert!(reached.flashed.is_none());
    // The clock ran the thirty seconds out.
    assert!(bench.ticks > 30);
}

/// The owner's ruling of 2026-09-25: a port that fails once the board is found says so on the
/// status line and never in a box.
#[test]
fn a_port_that_breaks_after_the_identify_is_a_status_line_not_a_box() {
    let fw = firmware(140, &[7u8; 1024]);
    let mut bench = Bench::new(LinkReboot::NotSerial)
        .flaky_board("/dev/ttyACM0", MockBootloader::new(140, 2_080_768));
    let mut person = Person::default();
    let mut reached = Reached::default();
    let ok = flow::upload_px4(&mut cx(&mut person), &mut bench, &fw, &mut reached);
    assert!(!ok);
    assert!(person.shown.is_empty(), "no box: {:?}", person.shown);
    assert_eq!(
        person.progress.last().map(|(_, text)| text.as_str()),
        Some(LOST_COMMUNICATION)
    );
    assert!(reached.flashed.is_none());
}

#[test]
fn a_bootloader_already_answering_is_not_rebooted() {
    let fw = firmware(140, &[1u8; 1024]);
    let mut bench =
        Bench::new(LinkReboot::Rebooted).board("/dev/ttyACM0", MockBootloader::new(140, 2_080_768));
    let mut person = Person::default();
    let mut reached = Reached::default();
    assert!(flow::upload_px4(
        &mut cx(&mut person),
        &mut bench,
        &fw,
        &mut reached
    ));
    assert_eq!(bench.reboots, 0, "a board in its bootloader is left there");
    assert_eq!(statuses(&person).first(), Some(&"Scanning comports"));
}

/// `Instance_DeviceChanged` over the bench: every port tried at once - one with nothing behind
/// it, one that is not there any more, one with a CubeOrange's bootloader - and the bootloader
/// found with its chip, said on the status line as the C# formats it.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:142-179`
#[test]
fn a_device_arriving_finds_the_bootloader_with_its_chip_on_every_port_at_once() {
    let bench = Mutex::new(
        Bench::new(LinkReboot::NotSerial)
            .empty_port("/dev/ttyS0")
            .board("/dev/ttyACM0", MockBootloader::new(140, 2_080_768)),
    );
    let names = ["/dev/ttyS0", "/dev/ttyACM0", "/dev/gone"].map(str::to_owned);
    let found = Mutex::new(Vec::new());
    let started = Instant::now();
    flow::probe_arrival(
        &names,
        |port, baud| bench.lock().expect("the bench").open(port, baud),
        |board| found.lock().expect("found").push(board),
    );
    // Twenty milliseconds for each port to appear - at once, not one after another.
    assert!(started.elapsed() >= flow::ARRIVAL_SETTLE);
    let found = found.into_inner().expect("found");
    let [board] = found.as_slice() else {
        panic!("one bootloader: {found:?}");
    };
    assert_eq!(board.port, "/dev/ttyACM0");
    assert_eq!(
        (board.board.board_id, board.board.bootloader_revision),
        (140, 5)
    );
    assert_eq!(board.chip.chip, 0x1003_6450);
    assert_eq!(board.chip.chip_desc, "STM32H7[4|5]x,rev:V");
    assert_eq!(
        board.chip.sn,
        b"\x00\x1d\x00\x2a\x32\x30\x51\x0b\x35\x38\x39\x38"
    );
    assert_eq!(
        board.status(),
        "Found board type 140 brdrev 0 blrev 5 fwmax 2080768 chip 10036450 chipdes \
         STM32H7[4|5]x,rev:V on /dev/ttyACM0"
    );
    let bench = bench.into_inner().expect("the bench");
    let mut opened = bench.opened.clone();
    opened.sort();
    assert_eq!(
        opened,
        ["/dev/gone", "/dev/ttyACM0", "/dev/ttyS0"],
        "every port tried"
    );
    let mock = bench.board_at("/dev/ttyACM0");
    let mock = mock.lock().expect("the board");
    assert!(
        mock.received.is_empty(),
        "whole commands: {:?}",
        mock.received
    );
    assert!(mock.outgoing.is_empty(), "every answer read");
    assert!(!mock.erased, "a probe writes nothing");
}

/// A bootloader that refuses `GET_CHIP` - `INSYNC INVALID` - is synced again and read on, as
/// the C#'s `catch { __sync(); }` does: no chip, the serial number and the rest still read. One
/// before revision 5 is not asked for a chip at all.
/// `// C#: ExtLibs/px4uploader/Uploader.cs:436-452, 895-918`
#[test]
fn a_bootloader_without_the_chip_is_found_without_it() {
    let mut refusing = MockBootloader::new(140, 2_080_768);
    refusing.refuse_chip = true;
    let mut old = MockBootloader::new(9, 1_032_192);
    old.info
        .insert(mp_firmware::Info::BootloaderRevision.byte(), 4);
    let bench = Mutex::new(
        Bench::new(LinkReboot::NotSerial)
            .board("/dev/ttyACM0", refusing)
            .board("/dev/ttyACM1", old),
    );
    let names = ["/dev/ttyACM0", "/dev/ttyACM1"].map(str::to_owned);
    let found = Mutex::new(Vec::new());
    flow::probe_arrival(
        &names,
        |port, baud| bench.lock().expect("the bench").open(port, baud),
        |board| found.lock().expect("found").push(board),
    );
    let mut found = found.into_inner().expect("found");
    found.sort_by(|a, b| a.port.cmp(&b.port));
    let statuses: Vec<String> = found.iter().map(flow::FoundBoard::status).collect();
    assert_eq!(
        statuses,
        [
            "Found board type 140 brdrev 0 blrev 5 fwmax 2080768 chip 0 chipdes  on /dev/ttyACM0",
            "Found board type 9 brdrev 0 blrev 4 fwmax 1032192 chip 0 chipdes  on /dev/ttyACM1",
        ]
    );
    assert_eq!(found[0].chip.sn.len(), 12, "read on after the sync");
    assert!(
        found[1].chip.sn.is_empty(),
        "revision 4 has no serial number to ask"
    );
    let bench = bench.into_inner().expect("the bench");
    for port in ["/dev/ttyACM0", "/dev/ttyACM1"] {
        let mock = bench.board_at(port);
        let mock = mock.lock().expect("the board");
        assert!(mock.received.is_empty(), "{port}: {:?}", mock.received);
        assert!(mock.outgoing.is_empty(), "{port}: every answer read");
    }
}
