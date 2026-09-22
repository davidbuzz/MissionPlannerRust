//! End-to-end: a recorded flight, through a transport, a frame decoder and into typed messages.
//!
//! This is the first vertical slice of the port - `mp-transport` -> `mp-mavlink` ->
//! `mp-mavlink-dialects` - exercised on real data rather than fixtures, in small reads that force
//! the decoder through its partial-frame paths.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::BTreeMap;

use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::{DIALECT, Heartbeat, MavMessage};
use mp_transport::{ReplayTransport, Transport};

fn tlog_path() -> String {
    concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    )
    .to_owned()
}

#[test]
fn a_recorded_flight_decodes_into_typed_messages() {
    // 64-byte reads: small enough that most frames straddle a read boundary.
    let mut transport = ReplayTransport::open(tlog_path())
        .unwrap()
        .with_chunk_size(64);
    let mut decoder = FrameDecoder::new();

    let mut counts: BTreeMap<&'static str, u64> = BTreeMap::new();
    let mut heartbeats = 0u64;
    let mut buf = [0u8; 256];

    loop {
        let n = transport.read(&mut buf).unwrap();
        if n == 0 {
            break;
        }
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            if let Some(msg) = MavMessage::decode(frame.msgid, frame.payload) {
                *counts.entry(msg.name()).or_default() += 1;
                if let MavMessage::Heartbeat(hb) = msg {
                    heartbeats += 1;
                    assert_eq!(hb.mavlink_version, 3, "ArduPilot reports MAVLink version 3");
                }
            }
        });
    }
    decoder.flush(&DIALECT, |_| {});

    let total: u64 = counts.values().sum();
    let stats = decoder.stats();

    println!("decoded {total} messages of {} types", counts.len());
    println!("top messages:");
    let mut ranked: Vec<_> = counts.iter().collect();
    ranked.sort_by(|a, b| b.1.cmp(a.1));
    for (name, count) in ranked.iter().take(8) {
        println!("  {count:>6}  {name}");
    }
    println!("stats: {stats:?}");

    assert!(total > 35_000, "expected the whole flight, decoded {total}");
    assert!(
        heartbeats > 100,
        "a flight must contain many heartbeats, saw {heartbeats}"
    );
    assert!(
        counts.len() > 20,
        "expected many message types, saw {}",
        counts.len()
    );
    assert_eq!(
        stats.overflow_bytes, 0,
        "the decoder must keep up with a 64-byte-read stream"
    );

    // The interleaved tlog timestamps are not frames; the decoder must skip them without ever
    // mistaking one for a valid frame. CRC makes false positives vanishingly unlikely.
    assert!(
        stats.resync_bytes > 0,
        "timestamps should have been skipped as non-frame bytes"
    );
}

#[test]
fn decoding_is_identical_at_every_read_size() {
    // A transport that returns 1 byte at a time must produce exactly the same messages as one
    // that returns 4 KiB. Read-size sensitivity is a classic source of field-only bugs.
    let mut baseline = Vec::new();
    for chunk in [1usize, 7, 64, 512, 4096] {
        let mut transport = ReplayTransport::open(tlog_path())
            .unwrap()
            .with_chunk_size(chunk);
        let mut decoder = FrameDecoder::new();
        let mut seen: Vec<(u32, u8)> = Vec::new();
        let mut buf = [0u8; 4096];

        loop {
            let n = transport.read(&mut buf).unwrap();
            if n == 0 {
                break;
            }
            decoder.push_and_drain(&buf[..n], &DIALECT, |f| seen.push((f.msgid, f.seq)));
        }
        decoder.flush(&DIALECT, |f| seen.push((f.msgid, f.seq)));

        if baseline.is_empty() {
            baseline = seen;
            assert!(!baseline.is_empty());
        } else {
            assert_eq!(
                seen.len(),
                baseline.len(),
                "frame count differs at chunk size {chunk}"
            );
            assert_eq!(
                seen, baseline,
                "frame sequence differs at chunk size {chunk}"
            );
        }
    }
}

#[test]
fn heartbeat_round_trips_through_the_link() {
    use mp_mavlink::{Message as _, encode_v2};
    use mp_transport::testing::Loopback;

    let (mut gcs, mut vehicle) = Loopback::pair();

    let sent = Heartbeat {
        custom_mode: 4,
        r#type: 2,
        autopilot: 3,
        base_mode: 81,
        system_status: 4,
        mavlink_version: 3,
    };
    let mut payload = [0u8; Heartbeat::LEN];
    sent.encode(&mut payload);

    let mut frame = [0u8; 64];
    let n = encode_v2(
        &mut frame,
        0,
        1,
        1,
        Heartbeat::ID,
        &payload,
        Heartbeat::CRC_EXTRA,
        0,
    )
    .unwrap();
    gcs.write_all(&frame[..n]).unwrap();

    let mut decoder = FrameDecoder::new();
    let mut buf = [0u8; 128];
    let got = vehicle.read(&mut buf).unwrap();

    let mut received = None;
    decoder.push_and_drain(&buf[..got], &DIALECT, |f| {
        received = MavMessage::decode(f.msgid, f.payload);
    });

    assert_eq!(received, Some(MavMessage::Heartbeat(sent)));
}
