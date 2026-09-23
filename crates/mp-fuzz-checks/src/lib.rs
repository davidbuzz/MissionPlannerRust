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

use mp_log::TlogReader;
use mp_mavlink::{FrameDecoder, MAX_FRAME_LEN, parse};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_mission::{read_waypoints, write_waypoints};

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
];
