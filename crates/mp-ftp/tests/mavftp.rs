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

//! The MAVFTP client against a scripted vehicle, on a clock the test moves.
//!
//! Each test hands the client's requests to [`FakeVehicle`], which answers as ArduPilot's
//! `GCS_FTP.cpp` does, and hands the answers back - through the payload's bytes both ways, as the
//! link does - losing the ones the test says are lost. Time only passes when nothing is in
//! flight, ten milliseconds at a time, so a test that runs the C#'s thirty one-second retries
//! takes no time at all and every wait is measured exactly.
//!
//! C# lines are in `ExtLibs/ArduPilot/Mavlink/MAVFtp.cs` unless named.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use mp_ftp::FtpError;
use mp_ftp::mavftp::testing::FakeVehicle;
use mp_ftp::mavftp::wire::{Errno, ErrorCode, Header, Opcode};
use mp_ftp::mavftp::{FtpOutcome, FtpRequest, FtpTimeouts, MavFtp, RW_SIZE, crc_crc32};
use mp_vehicle::VehicleId;

const VEHICLE: VehicleId = VehicleId::new(1, 1);

/// The client, the vehicle, and the clock.
struct Rig {
    ftp: MavFtp,
    vehicle: FakeVehicle,
    start: Instant,
    now: Instant,
    /// Every request the client sent, with when.
    sent: Vec<(Duration, Header)>,
}

impl Rig {
    fn new(vehicle: FakeVehicle) -> Self {
        let now = Instant::now();
        Self {
            ftp: MavFtp::new(VEHICLE, FtpTimeouts::default()),
            vehicle,
            start: now,
            now,
            sent: Vec::new(),
        }
    }

    /// Runs one request to its end. `lost` says which of the vehicle's replies never arrive.
    fn run(
        &mut self,
        request: FtpRequest,
        mut lost: impl FnMut(&Header) -> bool,
    ) -> Result<FtpOutcome, FtpError> {
        let mut out = Vec::new();
        assert!(self.ftp.start(request, self.now, &mut out));
        let mut pending: VecDeque<Header> = out.drain(..).collect();
        loop {
            while let Some(request) = pending.pop_front() {
                // Through the bytes, as the link sends them.
                let heard = Header::decode(&request.encode());
                self.sent.push((self.now - self.start, heard.clone()));
                for reply in self.vehicle.answer(&heard) {
                    if lost(&reply) {
                        continue;
                    }
                    self.ftp.on_message(&reply.encode(), self.now, &mut out);
                    pending.extend(out.drain(..));
                }
            }
            if !self.ftp.is_busy() {
                break;
            }
            self.now += Duration::from_millis(10);
            self.ftp.on_tick(self.now, &mut out);
            pending.extend(out.drain(..));
            assert!(
                self.now - self.start < Duration::from_secs(3600),
                "the client never finished"
            );
        }
        self.ftp
            .take_outcome()
            .expect("a finished request has an outcome")
    }

    /// Every request's opcode, in order.
    fn opcodes(&self) -> Vec<Opcode> {
        self.sent.iter().map(|(_, h)| h.opcode).collect()
    }

    /// The requests with this opcode, with when they went.
    fn sent_as(&self, opcode: Opcode) -> Vec<(Duration, Header)> {
        self.sent
            .iter()
            .filter(|(_, h)| h.opcode == opcode)
            .cloned()
            .collect()
    }

    fn elapsed(&self) -> Duration {
        self.now - self.start
    }
}

fn nothing_lost(_: &Header) -> bool {
    false
}

/// Bytes that say where they are, so a chunk in the wrong place shows.
fn numbered(len: usize) -> Vec<u8> {
    (0..len).map(|i| u8::try_from(i % 251).unwrap()).collect()
}

fn get(path: &str, burst: bool) -> FtpRequest {
    FtpRequest::Get {
        path: path.to_owned(),
        burst,
        readsize: RW_SIZE,
    }
}

fn file_data(outcome: Result<FtpOutcome, FtpError>) -> (Vec<u8>, bool) {
    match outcome {
        Ok(FtpOutcome::File {
            data: Some(data),
            ended,
        }) => (data, ended),
        other => panic!("expected a file, got {other:?}"),
    }
}

// --- listings ---------------------------------------------------------------------------------

/// Forty files are more than one acknowledgement holds, so the listing asks again at the count of
/// entries so far until the vehicle says end of file (MAVFtp.cs:1339-1340, 1419-1424). The timed
/// listing goes first and ArduPilot does not know it (kErrUnknownCommand), so the plain one
/// follows (:1252-1261, 1306-1317).
#[test]
fn a_listing_runs_across_offsets_until_the_vehicle_says_end_of_file() {
    let mut vehicle = FakeVehicle::new().with_dir("/APM/LOGS");
    for i in 0..40 {
        vehicle = vehicle.with_file(&format!("/APM/file_{i:02}.bin"), &numbered(i * 10));
    }
    let mut rig = Rig::new(vehicle);

    let outcome = rig.run(
        FtpRequest::List {
            path: "/APM/".to_owned(),
        },
        nothing_lost,
    );
    let Ok(FtpOutcome::Listing { entries, ended }) = outcome else {
        panic!("{outcome:?}");
    };
    assert!(ended, "ended by end of file, not by giving up");
    assert_eq!(entries.len(), 41);
    assert_eq!(entries[0].to_string(), "Directory: LOGS");
    assert_eq!(entries[1].to_string(), "File: file_00.bin 0");
    assert_eq!(entries[40].to_string(), "File: file_39.bin 390");
    assert_eq!(
        entries[40].full_path, "/APM/file_39.bin",
        "the trailing / was trimmed"
    );

    let timed = rig.sent_as(Opcode::LIST_DIRECTORY_WITH_TIME);
    assert_eq!(timed.len(), 1, "refused once, not retried");
    let offsets: Vec<u32> = rig
        .sent_as(Opcode::LIST_DIRECTORY)
        .iter()
        .map(|(_, h)| h.offset)
        .collect();
    assert!(offsets.len() >= 4, "{offsets:?}");
    assert_eq!(offsets[0], 0);
    assert!(offsets.windows(2).all(|w| w[1] > w[0]), "{offsets:?}");
    assert_eq!(*offsets.last().unwrap(), 41, "the last asks past the end");
    // Every request is the directory without its slash, and a new sequence number.
    assert!(rig.sent.iter().all(|(_, h)| h.payload() == b"/APM"));
    let seqs: Vec<u16> = rig.sent.iter().map(|(_, h)| h.seq_number).collect();
    assert_eq!(
        seqs,
        (0..u16::try_from(seqs.len()).unwrap()).collect::<Vec<_>>()
    );

    // The refusal is remembered: the next listing is plain from the start.
    rig.sent.clear();
    let again = rig.run(
        FtpRequest::List {
            path: "/APM/LOGS".to_owned(),
        },
        nothing_lost,
    );
    assert!(
        matches!(again, Ok(FtpOutcome::Listing { ref entries, ended: true }) if entries.is_empty())
    );
    assert_eq!(rig.opcodes(), [Opcode::LIST_DIRECTORY]);
    assert!(rig.ftp.list_directory_with_time_unsupported());
}

/// A vehicle that knows the timed listing gets asked it, and its times come through
/// (MAVFtp.cs:1367-1389, 1240-1246).
#[test]
fn a_vehicle_that_knows_timed_listings_gives_times() {
    let mut vehicle = FakeVehicle::new()
        .with_file("/logs/00000001.BIN", b"abc")
        .with_dir("/logs/old");
    vehicle.knows_list_with_time = true;
    vehicle
        .times
        .insert("/logs/00000001.BIN".to_owned(), 1_758_000_000);
    let mut rig = Rig::new(vehicle);

    let Ok(FtpOutcome::Listing { entries, .. }) = rig.run(
        FtpRequest::List {
            path: "/logs".to_owned(),
        },
        nothing_lost,
    ) else {
        panic!();
    };
    assert_eq!(entries[0].name, "old");
    assert_eq!(entries[0].modified_utc, None, "zero is no time");
    assert_eq!(entries[1].name, "00000001.BIN");
    assert_eq!(entries[1].size, 3);
    assert_eq!(entries[1].modified_utc, Some(1_758_000_000));
    assert_eq!(
        rig.opcodes(),
        [
            Opcode::LIST_DIRECTORY_WITH_TIME,
            Opcode::LIST_DIRECTORY_WITH_TIME
        ]
    );
}

/// A lost acknowledgement part way through: the listing waits its second and asks for the same
/// offset again, five sends in all (MAVFtp.cs:1287, RetryTimeout.cs:63-76).
#[test]
fn a_lost_listing_reply_is_asked_for_again_after_a_second() {
    let mut vehicle = FakeVehicle::new();
    for i in 0..30 {
        vehicle = vehicle.with_file(&format!("/d/f{i:02}"), b"x");
    }
    let mut rig = Rig::new(vehicle);
    let mut lost_one = false;
    let outcome = rig.run(
        FtpRequest::List {
            path: "/d".to_owned(),
        },
        |reply| {
            if reply.req_opcode == Opcode::LIST_DIRECTORY && reply.offset > 0 && !lost_one {
                lost_one = true;
                return true;
            }
            false
        },
    );
    let Ok(FtpOutcome::Listing { entries, ended }) = outcome else {
        panic!();
    };
    assert!(ended);
    assert_eq!(entries.len(), 30, "nothing twice, nothing missing");
    let plain = rig.sent_as(Opcode::LIST_DIRECTORY);
    let again = plain
        .windows(2)
        .find(|w| w[0].1.offset == w[1].1.offset)
        .expect("the lost offset was asked again");
    assert_eq!(again[1].0 - again[0].0, Duration::from_secs(1));
    assert_eq!(
        again[0].1.seq_number, again[1].1.seq_number,
        "a resend, not a new request"
    );
}

// --- reads ------------------------------------------------------------------------------------

/// `GetFile` with a burst: reset, open, burst. The chunk at 240 is lost; the burst runs on to the
/// end, sees the hole, and switches to `kCmdReadFile` at 240 (MAVFtp.cs:826-840, 895-912).
///
/// It asks twice. The last chunk shows the hole and asks for it (:826-840); the end-of-file NAK
/// ArduPilot sends behind that chunk arrives before the answer and asks again (:746-769). The C#
/// does the same; the second answer arrives after the file is whole and nobody reads it.
#[test]
fn a_burst_read_with_a_lost_chunk_reads_the_hole_after_the_burst() {
    let content = numbered(1000);
    let mut rig = Rig::new(FakeVehicle::new().with_file("/APM/test.bin", &content));
    let mut lost = false;
    let outcome = rig.run(get("/APM/test.bin", true), |reply| {
        if reply.req_opcode == Opcode::BURST_READ_FILE && reply.offset == 240 && !lost {
            lost = true;
            return true;
        }
        false
    });
    let (data, ended) = file_data(outcome);
    assert!(ended);
    assert_eq!(data, content);
    assert_eq!(
        rig.opcodes(),
        [
            Opcode::RESET_SESSIONS,
            Opcode::OPEN_FILE_RO,
            Opcode::BURST_READ_FILE,
            Opcode::READ_FILE,
            Opcode::READ_FILE
        ]
    );
    for (_, read) in rig.sent_as(Opcode::READ_FILE) {
        assert_eq!(read.offset, 240);
        assert_eq!(read.size, RW_SIZE, "the burst's read size, kept");
    }
    assert_eq!(
        rig.elapsed(),
        Duration::ZERO,
        "no wait: the hole is asked for at once"
    );
}

/// A burst the vehicle marks complete before the end is asked to continue from where it got to
/// (MAVFtp.cs:813-816, 843-848).
#[test]
fn a_burst_that_stops_short_is_asked_to_continue() {
    let content = numbered(1000);
    let mut vehicle = FakeVehicle::new().with_file("/f", &content);
    vehicle.burst_chunks = 4;
    let mut rig = Rig::new(vehicle);
    let (data, _) = file_data(rig.run(get("/f", true), nothing_lost));
    assert_eq!(data, content);
    let offsets: Vec<u32> = rig
        .sent_as(Opcode::BURST_READ_FILE)
        .iter()
        .map(|(_, h)| h.offset)
        .collect();
    assert_eq!(offsets, [0, 320, 640, 960]);
}

/// The chunk that says "burst complete" is lost, so nobody asks for the next burst. After the
/// second `RetryTimeout()` waits (MAVFtp.cs:696), the request goes again - from where the data got
/// to, which the handler kept moving (:813-816).
#[test]
fn a_lost_end_of_burst_is_recovered_by_the_timeout() {
    let content = numbered(1000);
    let mut vehicle = FakeVehicle::new().with_file("/f", &content);
    vehicle.burst_chunks = 4;
    let mut rig = Rig::new(vehicle);
    let mut lost = false;
    let (data, ended) = file_data(rig.run(get("/f", true), |reply| {
        if reply.burst_complete == 1 && !lost {
            lost = true;
            return true;
        }
        false
    }));
    assert!(ended);
    assert_eq!(data, content);
    let bursts = rig.sent_as(Opcode::BURST_READ_FILE);
    assert_eq!(
        bursts[1].1.offset, 240,
        "from the end of the last chunk that came"
    );
    assert_eq!(bursts[1].0 - bursts[0].0, Duration::from_secs(1));
}

/// ArduPilot's `@SYS` files open claiming a size they do not have. A plain read asks chunk after
/// chunk until the vehicle says end of file, which is what completes it (MAVFtp.cs:1599-1600).
#[test]
fn a_plain_read_ends_on_end_of_file() {
    let content = b"SERIAL0 OTG1 TX=0 RX=0\nSERIAL1 UART4 TX=0 RX=0\n".repeat(4);
    let mut vehicle = FakeVehicle::new().with_file("@SYS/uarts.txt", &content);
    vehicle
        .reported_sizes
        .insert("@SYS/uarts.txt".to_owned(), 100_000);
    let mut rig = Rig::new(vehicle);
    let (data, ended) = file_data(rig.run(get("@SYS/uarts.txt", false), nothing_lost));
    assert!(ended);
    assert_eq!(data, content);
    let reads = rig.sent_as(Opcode::READ_FILE);
    assert_eq!(
        reads.last().unwrap().1.offset,
        u32::try_from(content.len()).unwrap(),
        "the last read asks past the end and is told end of file"
    );
}

/// The same file by burst: the short last chunk cuts the size (MAVFtp.cs:785-789) and the file is
/// whole without waiting for anything else.
#[test]
fn a_burst_read_ends_at_its_short_chunk() {
    let content = numbered(300);
    let mut vehicle = FakeVehicle::new().with_file("@SYS/threads.txt", &content);
    vehicle
        .reported_sizes
        .insert("@SYS/threads.txt".to_owned(), 100_000);
    let mut rig = Rig::new(vehicle);
    let (data, ended) = file_data(rig.run(get("@SYS/threads.txt", true), nothing_lost));
    assert!(ended);
    assert_eq!(data, content);
    assert_eq!(rig.sent_as(Opcode::READ_FILE).len(), 0);
}

/// A plain read whose chunk is lost waits its second and asks for the same offset again
/// (MAVFtp.cs:1549, 1612-1617).
#[test]
fn a_plain_read_asks_again_for_a_lost_chunk() {
    let content = numbered(500);
    let mut rig = Rig::new(FakeVehicle::new().with_file("/f", &content));
    let mut lost = false;
    let (data, _) = file_data(rig.run(get("/f", false), |reply| {
        if reply.req_opcode == Opcode::READ_FILE && reply.offset == 160 && !lost {
            lost = true;
            return true;
        }
        false
    }));
    assert_eq!(data, content);
    let reads = rig.sent_as(Opcode::READ_FILE);
    let at_160: Vec<&(Duration, Header)> = reads.iter().filter(|(_, h)| h.offset == 160).collect();
    assert_eq!(at_160.len(), 2);
    assert_eq!(at_160[1].0 - at_160[0].0, Duration::from_secs(1));
}

/// A file that is not there: the vehicle says so at once, and the C# says so two seconds later,
/// when `kCmdOpenFileRO`'s wait runs out (MAVFtp.cs:608, 646-652, RetryTimeout.cs:63-76).
#[test]
fn a_file_that_is_not_there_is_reported_when_the_open_wait_runs_out() {
    let mut rig = Rig::new(FakeVehicle::new());
    let outcome = rig.run(get("/nope.txt", true), nothing_lost);
    let Err(error) = outcome else {
        panic!("{outcome:?}")
    };
    assert_eq!(
        error,
        FtpError::FileNotFound {
            path: "/nope.txt".to_owned()
        }
    );
    assert_eq!(error.to_string(), "File Not Found");
    assert_eq!(rig.sent_as(Opcode::OPEN_FILE_RO).len(), 1, "not retried");
    assert_eq!(rig.elapsed(), Duration::from_secs(2));
}

// --- refusals ---------------------------------------------------------------------------------

/// `kErrFailErrno`: stop retrying, and throw with the errno named, in the words of the command's
/// handler (MAVFtp.cs:626-634).
#[test]
fn a_refusal_with_an_errno_is_reported_in_the_csharps_words() {
    let mut vehicle = FakeVehicle::new().with_file("/secret", b"x");
    vehicle.refuse = Some((Opcode::OPEN_FILE_RO, ErrorCode::FAIL_ERRNO, Errno::EACCES));
    let mut rig = Rig::new(vehicle);
    let Err(error) = rig.run(get("/secret", true), nothing_lost) else {
        panic!();
    };
    assert_eq!(
        error.to_string(),
        "Mavftp responded - kCmdOpenFileRO kErrFailErrno EACCES"
    );
    assert_eq!(rig.sent_as(Opcode::OPEN_FILE_RO).len(), 1);
    assert_eq!(rig.elapsed(), Duration::from_secs(2));

    // kCmdRemoveFile words the same refusal the other way (MAVFtp.cs:1778-1785).
    let mut vehicle = FakeVehicle::new().with_file("/busy", b"x");
    vehicle.refuse = Some((Opcode::REMOVE_FILE, ErrorCode::FAIL_ERRNO, Errno(16)));
    let mut rig = Rig::new(vehicle);
    let Err(error) = rig.run(
        FtpRequest::RemoveFile {
            path: "/busy".to_owned(),
        },
        nothing_lost,
    ) else {
        panic!();
    };
    assert_eq!(
        error.to_string(),
        "Failed to OpenFile - kCmdRemoveFile kErrFailErrno EBUSY"
    );
    assert_eq!(rig.elapsed(), Duration::from_secs(1));
}

/// `kErrFail` stops `kCmdOpenFileRO` (MAVFtp.cs:639-645) but not `kCmdCalcFileCRC32`, which has
/// no branch for it and so sends all three times, thirty seconds apart, and answers "no CRC"
/// (:928, 941-970).
#[test]
fn err_fail_stops_some_commands_and_not_others() {
    let mut vehicle = FakeVehicle::new().with_file("/f", b"x");
    vehicle.refuse = Some((Opcode::OPEN_FILE_RO, ErrorCode::FAIL, Errno(0)));
    let mut rig = Rig::new(vehicle);
    let Err(error) = rig.run(get("/f", true), nothing_lost) else {
        panic!();
    };
    assert_eq!(error, FtpError::ErrFail);
    assert_eq!(error.to_string(), "Mavftp responded - Err Fail");

    let mut vehicle = FakeVehicle::new().with_file("/f", b"x");
    vehicle.refuse = Some((Opcode::CALC_FILE_CRC32, ErrorCode::FAIL, Errno(0)));
    let mut rig = Rig::new(vehicle);
    let outcome = rig.run(
        FtpRequest::Crc32 {
            path: "/f".to_owned(),
        },
        nothing_lost,
    );
    assert_eq!(
        outcome,
        Ok(FtpOutcome::Crc32 {
            crc: u32::MAX,
            answered: false
        })
    );
    let sends: Vec<Duration> = rig
        .sent_as(Opcode::CALC_FILE_CRC32)
        .iter()
        .map(|(at, _)| *at)
        .collect();
    assert_eq!(
        sends,
        [
            Duration::ZERO,
            Duration::from_secs(30),
            Duration::from_secs(60)
        ]
    );
    assert_eq!(rig.elapsed(), Duration::from_secs(90));
}

/// `kErrNoSessionsAvailable` on the open: the sessions are reset, and the open goes again - its
/// own request, not the reset's - when its wait runs out (MAVFtp.cs:654-658). See the module notes
/// in `mavftp/mod.rs` for why this is not what the C# manages to do.
#[test]
fn no_sessions_available_resets_the_sessions_and_opens_again() {
    let content = numbered(200);
    let mut vehicle = FakeVehicle::new().with_file("/f", &content);
    vehicle.refuse_once = Some((
        Opcode::OPEN_FILE_RO,
        ErrorCode::NO_SESSIONS_AVAILABLE,
        Errno(0),
    ));
    let mut rig = Rig::new(vehicle);
    let (data, _) = file_data(rig.run(get("/f", true), nothing_lost));
    assert_eq!(data, content);
    assert_eq!(
        rig.opcodes(),
        [
            Opcode::RESET_SESSIONS,
            Opcode::OPEN_FILE_RO,
            Opcode::RESET_SESSIONS,
            Opcode::OPEN_FILE_RO,
            Opcode::BURST_READ_FILE
        ]
    );
    let resets = rig.sent_as(Opcode::RESET_SESSIONS);
    let opens = rig.sent_as(Opcode::OPEN_FILE_RO);
    // The nested reset goes at once, with the next sequence number.
    assert_eq!(resets[1].0, Duration::ZERO);
    assert_eq!(resets[1].1.seq_number, opens[0].1.seq_number + 1);
    // The open goes again with its own payload - a resend, same sequence number - when its two
    // second wait runs out.
    assert_eq!(opens[1].1.payload(), b"/f");
    assert_eq!(opens[1].1.seq_number, opens[0].1.seq_number);
    assert_eq!(opens[1].0, Duration::from_secs(2));
}

// --- sessions ---------------------------------------------------------------------------------

/// `kCmdTerminateSession` is acknowledged; unanswered, it is sent thirty times a second apart
/// and answers false (MAVFtp.cs:1963-2016, RetryTimeout.cs:37).
#[test]
fn terminating_the_session() {
    let mut rig = Rig::new(FakeVehicle::new());
    assert_eq!(
        rig.run(FtpRequest::TerminateSession, nothing_lost),
        Ok(FtpOutcome::Done(true))
    );
    assert_eq!(rig.opcodes(), [Opcode::TERMINATE_SESSION]);
    assert_eq!(rig.sent[0].1.size, 0, "it carries nothing");

    let mut rig = Rig::new(FakeVehicle::new());
    assert_eq!(
        rig.run(FtpRequest::TerminateSession, |_| true),
        Ok(FtpOutcome::Done(false))
    );
    let sends = rig.sent_as(Opcode::TERMINATE_SESSION);
    assert_eq!(sends.len(), 30);
    assert!(
        sends
            .windows(2)
            .all(|w| w[1].0 - w[0].0 == Duration::from_secs(1))
    );
    assert!(
        sends
            .iter()
            .all(|(_, h)| h.seq_number == sends[0].1.seq_number)
    );
    assert_eq!(rig.elapsed(), Duration::from_secs(30));
}

/// `kCmdResetSessions`: five sends a second apart (MAVFtp.cs:1919).
#[test]
fn resetting_the_sessions() {
    let mut rig = Rig::new(FakeVehicle::new());
    assert_eq!(
        rig.run(FtpRequest::ResetSessions, nothing_lost),
        Ok(FtpOutcome::Done(true))
    );
    let mut rig = Rig::new(FakeVehicle::new());
    assert_eq!(
        rig.run(FtpRequest::ResetSessions, |_| true),
        Ok(FtpOutcome::Done(false))
    );
    assert_eq!(rig.sent.len(), 5);
    assert_eq!(rig.elapsed(), Duration::from_secs(5));
}

/// Cancelling: the command notices at the next event and ends when its wait runs out
/// (`RetriesCurrent = 999`, e.g. MAVFtp.cs:853-859).
#[test]
fn a_cancelled_read_ends_when_its_wait_runs_out() {
    let mut rig = Rig::new(FakeVehicle::new().with_file("/f", &numbered(500)));
    let mut out = Vec::new();
    assert!(rig.ftp.start(get("/f", false), rig.now, &mut out));
    // Answer the reset and the open, then lose everything and cancel.
    for _ in 0..2 {
        let request = out.remove(0);
        for reply in rig.vehicle.answer(&request) {
            rig.ftp.on_message(&reply.encode(), rig.now, &mut out);
        }
    }
    assert_eq!(out[0].opcode, Opcode::READ_FILE);
    out.clear();
    rig.ftp.cancel();
    let mut sent_after = 0;
    while rig.ftp.is_busy() {
        rig.now += Duration::from_millis(10);
        rig.ftp.on_tick(rig.now, &mut out);
        sent_after += out.len();
        out.clear();
    }
    assert_eq!(sent_after, 0, "nothing more is sent");
    assert_eq!(
        rig.elapsed(),
        Duration::from_secs(2),
        "one wait, then one more"
    );
    assert_eq!(
        rig.ftp.take_outcome(),
        Some(Ok(FtpOutcome::File {
            data: None,
            ended: false
        }))
    );
}

// --- CRC --------------------------------------------------------------------------------------

/// The vehicle's CRC of a file is `crc_crc32(0, file)`, which for the catalogue's check string is
/// 0x2DFD2D88; and the CRC of what a read brought back matches it (Controls/MavFTPUI.cs:437-442).
#[test]
fn the_vehicles_crc_matches_the_crc_of_what_was_read() {
    let mut rig = Rig::new(
        FakeVehicle::new()
            .with_file("/check.txt", b"123456789")
            .with_file("/big.bin", &numbered(5000)),
    );
    assert_eq!(
        rig.run(
            FtpRequest::Crc32 {
                path: "/check.txt".to_owned()
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Crc32 {
            crc: 0x2DFD_2D88,
            answered: true
        })
    );
    let (data, _) = file_data(rig.run(get("/big.bin", true), nothing_lost));
    let Ok(FtpOutcome::Crc32 { crc, .. }) = rig.run(
        FtpRequest::Crc32 {
            path: "/big.bin".to_owned(),
        },
        nothing_lost,
    ) else {
        panic!();
    };
    assert_eq!(crc_crc32(0, &data), crc);
}

// --- writes -----------------------------------------------------------------------------------

/// `UploadFile`: reset, create, write, reset (MAVFtp.cs:576-583). The first chunk goes alone and
/// each answer to the latest send releases the next five (:2309-2330).
#[test]
fn an_upload_writes_every_chunk_between_two_resets() {
    let content = numbered(1000);
    let mut rig = Rig::new(FakeVehicle::new());
    assert_eq!(
        rig.run(
            FtpRequest::Put {
                path: "/APM/up.bin".to_owned(),
                data: content.clone(),
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Uploaded)
    );
    assert_eq!(rig.vehicle.files["/APM/up.bin"], content);
    let opcodes = rig.opcodes();
    assert_eq!(opcodes.first(), Some(&Opcode::RESET_SESSIONS));
    assert_eq!(opcodes.get(1), Some(&Opcode::CREATE_FILE));
    assert_eq!(opcodes.last(), Some(&Opcode::RESET_SESSIONS));
    let writes: Vec<(u32, u8)> = rig
        .sent_as(Opcode::WRITE_FILE)
        .iter()
        .map(|(_, h)| (h.offset, h.size))
        .collect();
    assert_eq!(writes.len(), 13);
    assert_eq!(writes[0], (0, 80));
    assert_eq!(writes[12], (960, 40), "the short last chunk");
}

/// An empty file is created and nothing is written: `kCmdWriteFile` returns false before it sends
/// anything (MAVFtp.cs:2230-2231), and `UploadFile` does not look.
#[test]
fn an_empty_upload_creates_the_file_and_writes_nothing() {
    let mut rig = Rig::new(FakeVehicle::new());
    assert_eq!(
        rig.run(
            FtpRequest::Put {
                path: "/empty".to_owned(),
                data: Vec::new(),
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Uploaded)
    );
    assert_eq!(rig.vehicle.files["/empty"], b"");
    assert_eq!(
        rig.opcodes(),
        [
            Opcode::RESET_SESSIONS,
            Opcode::CREATE_FILE,
            Opcode::RESET_SESSIONS
        ]
    );
    // The write still took its sequence number (MAVFtp.cs:2218-2224).
    assert_eq!(rig.sent[2].1.seq_number, rig.sent[1].1.seq_number + 2);
}

/// A lost write acknowledgement: the chunk stays on the list and goes again in the next batch.
#[test]
fn a_write_whose_answer_is_lost_is_sent_again() {
    let content = numbered(1000);
    let mut rig = Rig::new(FakeVehicle::new());
    let mut lost = false;
    let outcome = rig.run(
        FtpRequest::Put {
            path: "/up".to_owned(),
            data: content.clone(),
        },
        |reply| {
            if reply.req_opcode == Opcode::WRITE_FILE && reply.offset == 80 && !lost {
                lost = true;
                return true;
            }
            false
        },
    );
    assert_eq!(outcome, Ok(FtpOutcome::Uploaded));
    assert_eq!(rig.vehicle.files["/up"], content);
    let at_80 = rig
        .sent_as(Opcode::WRITE_FILE)
        .iter()
        .filter(|(_, h)| h.offset == 80)
        .count();
    assert_eq!(at_80, 2);
}

// --- the rest ---------------------------------------------------------------------------------

/// Remove, rename and the directories, each one command.
#[test]
fn remove_rename_and_directories() {
    let mut rig = Rig::new(FakeVehicle::new().with_file("/a/one", b"1"));
    assert_eq!(
        rig.run(
            FtpRequest::Rename {
                from: "/a/one".to_owned(),
                to: "/a/two".to_owned()
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Done(true))
    );
    // C#: MAVFtp.cs:1839, "src\0dest".
    assert_eq!(rig.sent[0].1.payload(), b"/a/one\0/a/two");
    assert!(rig.vehicle.files.contains_key("/a/two"));

    // C#: MAVFtp.cs:1748, "//" becomes "/".
    assert_eq!(
        rig.run(
            FtpRequest::RemoveFile {
                path: "/a//two".to_owned()
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Done(true))
    );
    assert_eq!(rig.sent.last().unwrap().1.payload(), b"/a/two");
    assert!(rig.vehicle.files.is_empty());

    assert_eq!(
        rig.run(
            FtpRequest::CreateDirectory {
                path: "/a/b".to_owned()
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Done(true))
    );
    // kErrFailFileExists completes it (MAVFtp.cs:1094-1097).
    assert_eq!(
        rig.run(
            FtpRequest::CreateDirectory {
                path: "/a/b".to_owned()
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Done(true))
    );
    assert_eq!(
        rig.run(
            FtpRequest::RemoveDirectory {
                path: "/a/b".to_owned()
            },
            nothing_lost
        ),
        Ok(FtpOutcome::Done(true))
    );
    let Err(error) = rig.run(
        FtpRequest::RemoveFile {
            path: "/a/gone".to_owned(),
        },
        nothing_lost,
    ) else {
        panic!();
    };
    assert_eq!(
        error,
        FtpError::FileNotFound {
            path: "/a/gone".to_owned()
        }
    );
}

/// `kCmdCreateDirectory` on `EEXIST` sets `Complete` - and then throws anyway, because `ex` was
/// set on the line after (MAVFtp.cs:1076-1087, 1134-1135).
#[test]
fn a_directory_that_exists_by_errno_is_still_an_error() {
    let mut vehicle = FakeVehicle::new();
    vehicle.refuse = Some((
        Opcode::CREATE_DIRECTORY,
        ErrorCode::FAIL_ERRNO,
        Errno::EEXIST,
    ));
    let mut rig = Rig::new(vehicle);
    let outcome = rig.run(
        FtpRequest::CreateDirectory {
            path: "/x".to_owned(),
        },
        nothing_lost,
    );
    assert_eq!(
        outcome.map_err(|e| e.to_string()),
        Err("Mavftp responded - kCmdCreateDirectory kErrFailErrno EEXIST".to_owned())
    );
    assert_eq!(rig.elapsed(), Duration::ZERO, "complete, so no wait");
}

/// A second request while one is in flight is refused and sends nothing.
#[test]
fn one_request_at_a_time() {
    let mut rig = Rig::new(FakeVehicle::new());
    let mut out = Vec::new();
    assert!(rig.ftp.start(FtpRequest::ResetSessions, rig.now, &mut out));
    assert_eq!(out.len(), 1);
    assert!(
        !rig.ftp
            .start(FtpRequest::TerminateSession, rig.now, &mut out)
    );
    assert_eq!(out.len(), 1);
}
