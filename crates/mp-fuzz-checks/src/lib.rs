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

//! The properties the fuzz targets check, in one place.
//!
//! Each function takes arbitrary bytes and either returns or panics. A panic is a finding.
//!
//! They live here rather than inside the `fuzz` crate because that crate needs a nightly
//! toolchain and cargo-fuzz, which an ordinary `cargo build --workspace` must not require. The
//! result was four fuzz targets that had never once been compiled - they happened to still match
//! the APIs they call, but nothing was checking, and a target that does not build is a target that
//! is not running. With the properties here, `cargo test --workspace` compiles and exercises them
//! on stable, and the libfuzzer entry points in `fuzz/fuzz_targets` are one-line wrappers that
//! cannot drift from what is actually checked.
//!
//! A bounded pass on stable is not a substitute for a real fuzzing run - it has no coverage
//! feedback, so it will not find what libFuzzer finds. It is the thing that keeps the targets
//! honest between runs.

// A failed expectation here *is* the finding: these run under a fuzzer, where a panic is the
// reporting mechanism and there is nobody to hand an error to.
#![allow(clippy::expect_used, clippy::missing_panics_doc)]

use mp_dronecan::slcan::{Line, read_line};
use mp_dronecan::transfer::Reassembler;
use mp_firmware::flow::{Buttons, Dialogue, read_intel_hex};
use mp_firmware::{github, legacy};
use mp_log::TlogReader;
use mp_log::analysis::{analyse_text, results};
use mp_log::convert::{convert_bin, no_mode_names};
use mp_log::dataflash::DataflashReader;
use mp_log::zip;
use mp_mavlink::{FrameDecoder, MAX_FRAME_LEN, parse};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_mission::fence_file::{read_fence, read_polygon, read_rally};
use mp_mission::{dbf, read_waypoints, write_waypoints};

/// Single-frame parsing.
///
/// `parse` must never panic, never loop, and never report consuming more bytes than it was given.
/// Every byte on a telemetry link is attacker-influenced in the sense that matters - radio noise,
/// a misconfigured peer, a corrupted log - and a parser that panics takes the ground station down
/// mid-flight.
pub fn frame_parse(data: &[u8]) {
    if let Ok((frame, used)) = parse(data, &DIALECT) {
        assert!(
            used <= data.len(),
            "claimed to consume {used} of {} bytes",
            data.len()
        );
        assert_eq!(
            frame.raw.len(),
            used,
            "raw slice must match the consumed length"
        );
        assert!(frame.payload.len() <= 255);

        // Zero-extension must never read out of bounds, whatever the payload length claimed.
        let mut buffer = [0u8; 255];
        frame.payload_into(&mut buffer);
        let _ = frame.payload_byte(254);
        let _ = frame.signable_bytes();
    }
}

/// The streaming decoder.
///
/// Beyond not panicking, the decoder must not wedge: a stream of arbitrary bytes has to keep
/// making progress rather than filling its buffer and stalling. That is a liveness property a
/// single-frame fuzzer cannot see, and a wedged decoder looks exactly like a dead link.
pub fn frame_stream(data: &[u8]) {
    let mut decoder = FrameDecoder::new();
    let mut frames = 0u64;

    // Fed in irregular chunks, which is what a real transport does.
    let mut offset = 0usize;
    let mut step = 1usize;
    while offset < data.len() {
        let end = (offset + step).min(data.len());
        let chunk = data.get(offset..end).expect("a slice inside the input");
        let consumed = decoder.push_and_drain(chunk, &DIALECT, |_| frames += 1);
        assert_eq!(
            consumed,
            end - offset,
            "push_and_drain must consume everything it is given"
        );
        offset = end;
        step = (step * 3 % 97) + 1;
    }
    decoder.flush(&DIALECT, |_| frames += 1);

    // After a flush the decoder must not be holding a frame's worth of undecidable bytes.
    assert!(
        decoder.buffered() < MAX_FRAME_LEN,
        "decoder wedged with {} bytes",
        decoder.buffered()
    );
}

/// The waypoint file reader.
///
/// Mission files come from other tools, other people and other decades. The reader must reject or
/// accept, never panic - and anything it accepts must survive a write-read round trip, or the file
/// a user saves will not match the one they loaded.
pub fn waypoint_file(data: &[u8]) {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let Ok(items) = read_waypoints(text) else {
        return;
    };

    let written = write_waypoints(&items);
    let reread = read_waypoints(&written).expect("our own output must parse");
    assert_eq!(
        items.len(),
        reread.len(),
        "item count changed on round trip"
    );

    // Writing must be a fixed point: the second write cannot differ from the first.
    assert_eq!(
        written,
        write_waypoints(&reread),
        "writing is not idempotent"
    );
}

/// The telemetry log reader.
///
/// Logs get truncated by crashes, concatenated by accident and copied off failing SD cards. The
/// reader must terminate on any input and never hand back a record that points outside the buffer.
pub fn tlog_reader(data: &[u8]) {
    let mut reader = TlogReader::new(data);
    let mut records = 0usize;

    while let Some(record) = reader.next_record(&DIALECT) {
        assert!(
            record.offset < data.len(),
            "record offset outside the buffer"
        );
        assert!(!record.frame.is_empty(), "an empty frame is not a record");
        assert!(
            record.offset + record.frame.len() <= data.len(),
            "frame extends past the end of the log"
        );
        records += 1;
        // A log cannot contain more records than it has bytes; this catches a reader that fails
        // to advance.
        assert!(records <= data.len(), "reader is not making progress");
    }
}

/// Decoding a frame into a message.
///
/// The gap `frame_parse` leaves. That target stops at the framing layer: it checks the header, the
/// length and the checksum, and never calls a per-message decoder. Those decoders are where a
/// payload is read at fixed offsets - `get_f32(payload, 22)` on a payload the sender said was
/// twenty-two bytes long - and they are reachable from the network by anything that can put a
/// message id on the wire.
///
/// Zero-extension is the specific thing being checked. MAVLink v2 trims trailing zero bytes, so a
/// decoder must treat a short payload as zero-padded rather than reading past it; a decoder that
/// indexes straight into the slice works perfectly against every real sender and panics on the
/// first truncated frame.
pub fn message_decode(data: &[u8]) {
    // Both a decode of the raw bytes as a payload, and a decode of whatever a real frame carries.
    // The first reaches every message id cheaply; the second gets there through the parser, which
    // is how it happens in production.
    if let Some((first, rest)) = data.split_first() {
        let msgid = u32::from(*first);
        if let Some(message) = MavMessage::decode(msgid, rest) {
            // Every accessor a consumer would use, because a decoder that panics in `fields` is
            // as broken as one that panics in `decode`.
            let _ = message.name();
            let _ = message.fields();
        }
        // And the same id with nothing at all, which is the trimmed-to-death case.
        if let Some(message) = MavMessage::decode(msgid, &[]) {
            let _ = message.fields();
        }
    }
    if let Ok((frame, _)) = parse(data, &DIALECT)
        && let Some(message) = MavMessage::decode(frame.msgid, frame.payload)
    {
        let _ = message.fields();
    }
}

/// A dataflash `.bin`.
///
/// What a failing card or a download cut short leaves: the reader must terminate, and never hand
/// back more messages than there are bytes; the text conversion, which runs over every message a
/// `.bin` has, must come out the other side.
pub fn dataflash_bin(data: &[u8]) {
    let mut reader = DataflashReader::new(data);
    let mut messages = 0usize;
    while reader.next_message().is_some() {
        messages += 1;
        assert!(messages <= data.len(), "reader is not making progress");
    }
    let _ = convert_bin(data, &no_mode_names);
}

/// The log analyzer over a `.log`'s text.
///
/// Its Python skips every line it cannot read and runs seventeen checks over what is left, each
/// of which it lets raise; the port must not panic on any of it, and the XML it writes must be
/// readable by the C#'s reader, or fail to be, without a panic either.
pub fn loganalyzer_text(data: &[u8]) {
    let text = String::from_utf8_lossy(data);
    if let Ok(xml) = analyse_text(&text, "fuzz.log") {
        let _ = results(&xml);
    }
}

/// A zip archive.
///
/// KMZ files, SRTM tiles, firmware and updates all arrive as zips. The reader must terminate and
/// read nothing outside the buffer; what it reads, written back, must read again the same.
pub fn zip_archive(data: &[u8]) {
    let Ok(entries) = zip::read(data) else {
        return;
    };
    let stamp = zip::DosTime {
        year: 2026,
        month: 1,
        day: 1,
        hour: 0,
        minute: 0,
        second: 0,
    };
    let Ok(written) = zip::write(&entries, stamp) else {
        return;
    };
    let reread = zip::read(&written).expect("our own archive must read");
    assert_eq!(
        reread.len(),
        entries.len(),
        "entry count changed on round trip"
    );
    for (before, after) in entries.iter().zip(reread.iter()) {
        assert_eq!(
            before.name, after.name,
            "an entry's name changed on round trip"
        );
        assert_eq!(
            before.data, after.data,
            "an entry's data changed on round trip"
        );
    }
}

/// Fence, rally and polygon files.
///
/// Readers that never fail - each returns what it could read - must read anything without a
/// panic.
pub fn fence_file(data: &[u8]) {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let _ = read_fence(text);
    let _ = read_rally(text);
    let _ = read_polygon(text);
}

/// A shapefile's DBF table: the reader must terminate and index nothing outside the buffer.
pub fn dbf_table(data: &[u8]) {
    let _ = dbf::read(data);
}

/// A dialogue that answers every question Yes and shows nothing: what the Intel HEX reader's
/// boxes meet under the fuzzer.
struct Mute;

impl Dialogue for Mute {
    fn ask(&mut self, _text: &str, _caption: &str, _buttons: Buttons) -> bool {
        true
    }

    fn show(&mut self, _text: &str, _caption: &str) {}

    fn progress(&mut self, _percent: i32, _status: &str) {}
}

/// An Intel HEX firmware image: records, extended addresses and checksums from a file somebody
/// downloaded. The reader must terminate, and write nowhere outside its image.
pub fn intel_hex(data: &[u8]) {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let _ = read_intel_hex(text, &mut Mute);
}

/// The firmware catalogues: the legacy XML list, GitHub's directory JSON and its file JSON,
/// each from a server that may answer anything.
pub fn catalogue_text(data: &[u8]) {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let _ = legacy::parse_list(text);
    let _ = github::parse_dir(text, "");
    let _ = github::parse_file(text);
}

/// SLCAN lines into the DroneCAN transfer reassembler, and through it the DSDL decoders: every
/// frame a bus delivers goes this way, and a line from an adapter is anything.
pub fn dronecan_slcan(data: &[u8]) {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let mut reassembler = Reassembler::new();
    for line in text.split(['\r', '\n']) {
        if let Line::Frame(frame, id, payload) = read_line(line) {
            let _ = reassembler.push(frame, id, &payload);
        }
    }
}

/// A named property: arbitrary bytes in, a panic if the property does not hold.
pub type Target = (&'static str, fn(&[u8]));

/// Every target, by name, so a harness can run the lot without listing them.
///
/// A list that has to be edited when a target is added is a list that will not be. This one is
/// next to the functions it names.
pub const TARGETS: &[Target] = &[
    ("frame_parse", frame_parse),
    ("frame_stream", frame_stream),
    ("waypoint_file", waypoint_file),
    ("tlog_reader", tlog_reader),
    ("message_decode", message_decode),
    ("dataflash_bin", dataflash_bin),
    ("loganalyzer_text", loganalyzer_text),
    ("zip_archive", zip_archive),
    ("fence_file", fence_file),
    ("dbf_table", dbf_table),
    ("intel_hex", intel_hex),
    ("catalogue_text", catalogue_text),
    ("dronecan_slcan", dronecan_slcan),
];
