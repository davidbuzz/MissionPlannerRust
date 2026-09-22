//! Dataflash parsing, against a real ArduPilot log.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::BTreeMap;

use mp_log::dataflash::{DataflashReader, FieldType, Value};

/// A healthy log from a real vehicle.
fn log_bytes() -> Vec<u8> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/dataflash.bin");
    std::fs::read(path).expect("dataflash corpus")
}

/// A genuinely damaged log, kept deliberately.
///
/// It came off a development board and roughly 80% of its bytes are not message-framed at all.
/// Two independent implementations agree on what it contains, so it is not a parser bug - it is
/// what a broken log looks like, and a ground station has to survive one.
fn damaged_log_bytes() -> Vec<u8> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/dataflash_damaged.bin"
    );
    std::fs::read(path).expect("damaged dataflash corpus")
}

#[test]
fn a_real_log_decodes_into_named_messages() {
    let data = log_bytes();
    let mut reader = DataflashReader::new(&data);

    let mut counts: BTreeMap<String, u64> = BTreeMap::new();
    while let Some(message) = reader.next_message() {
        *counts.entry(message.name.clone()).or_default() += 1;
    }

    let stats = *reader.stats();
    println!(
        "decoded {} messages of {} types",
        stats.messages,
        counts.len()
    );
    println!(
        "formats {} unknown {} inconsistent {}",
        stats.formats, stats.unknown_types, stats.inconsistent_formats
    );
    let mut ranked: Vec<_> = counts.iter().collect();
    ranked.sort_by(|a, b| b.1.cmp(a.1));
    for (name, count) in ranked.iter().take(8) {
        println!("  {count:>6}  {name}");
    }

    assert!(
        stats.formats > 20,
        "a real log defines many message types, saw {}",
        stats.formats
    );
    assert!(
        stats.messages > 10_000,
        "expected a substantial log, decoded {}",
        stats.messages
    );
    assert!(
        counts.len() > 10,
        "expected many message types, saw {}",
        counts.len()
    );

    // A healthy log is perfectly framed end to end. Any resynchronisation at all means the parser
    // lost its place, because a well-formed log never requires a byte to be skipped.
    assert_eq!(
        stats.resync_bytes, 0,
        "a healthy log should need no resynchronisation"
    );
    assert_eq!(
        stats.unknown_types, 0,
        "every message type should be defined before use"
    );
    assert_eq!(
        stats.inconsistent_formats, 0,
        "format strings should match declared lengths"
    );
}

#[test]
fn a_damaged_log_yields_what_it_can_without_hanging() {
    // Roughly 80% of this log is not message-framed. The requirement is not that it parses
    // cleanly - it cannot - but that the parser terminates, decodes the intact regions, and does
    // not mistake noise for data.
    let data = damaged_log_bytes();
    let mut reader = DataflashReader::new(&data);

    let mut messages = 0u64;
    while reader.next_message().is_some() {
        messages += 1;
        assert!(messages < 1_000_000, "parser is not making progress");
    }

    let stats = *reader.stats();
    println!(
        "damaged log: {} messages recovered, {} bytes skipped, {} unknown types",
        stats.messages, stats.resync_bytes, stats.unknown_types
    );
    assert!(
        messages > 1_000,
        "should still recover the intact regions, got {messages}"
    );
    assert!(
        stats.resync_bytes > 0,
        "this corpus is damaged; some bytes must be skipped"
    );
}

#[test]
fn scaled_field_types_are_scaled() {
    // The trap: L is degrees in 1e7 fixed point, c and C are hundredths. Reading an L as a plain
    // integer gives 350123456 instead of 35.0123456 - a number that looks like data.
    assert_eq!(
        FieldType(b'L').decode(&350_123_456i32.to_le_bytes()),
        Some(Value::Float(35.012_345_6))
    );
    assert_eq!(
        FieldType(b'c').decode(&1234i16.to_le_bytes()),
        Some(Value::Float(12.34))
    );
    assert_eq!(
        FieldType(b'C').decode(&5678u16.to_le_bytes()),
        Some(Value::Float(56.78))
    );
    assert_eq!(
        FieldType(b'e').decode(&123_456i32.to_le_bytes()),
        Some(Value::Float(1234.56))
    );
    assert_eq!(
        FieldType(b'E').decode(&123_456u32.to_le_bytes()),
        Some(Value::Float(1234.56))
    );

    // And the unscaled ones must stay unscaled.
    assert_eq!(
        FieldType(b'i').decode(&42i32.to_le_bytes()),
        Some(Value::Int(42))
    );
    assert_eq!(
        FieldType(b'I').decode(&42u32.to_le_bytes()),
        Some(Value::Uint(42))
    );
}

#[test]
fn every_documented_field_type_has_a_size() {
    // The full set from BinaryLog.cs. A missing size means messages containing that type cannot
    // be decoded at all, and silently so.
    for code in b"bBhHiIqQgfdcCeELnNMZa" {
        assert!(
            FieldType(*code).size().is_some(),
            "no size for format character {}",
            *code as char
        );
    }
    assert_eq!(
        FieldType(b'?').size(),
        None,
        "unknown characters must be rejected"
    );
}

#[test]
fn half_precision_floats_decode() {
    // 0x3C00 is 1.0, 0xC000 is -2.0, 0x0000 is zero.
    assert_eq!(
        FieldType(b'g').decode(&0x3C00u16.to_le_bytes()),
        Some(Value::Float(1.0))
    );
    assert_eq!(
        FieldType(b'g').decode(&0xC000u16.to_le_bytes()),
        Some(Value::Float(-2.0))
    );
    assert_eq!(
        FieldType(b'g').decode(&0x0000u16.to_le_bytes()),
        Some(Value::Float(0.0))
    );
}

#[test]
fn the_log_contains_plausible_flight_data() {
    // Decoding correctly is not the same as decoding sensibly: check the values are physical.
    let data = log_bytes();
    let mut reader = DataflashReader::new(&data);

    let mut attitude_samples = 0u32;
    let mut max_abs_roll = 0.0_f64;
    let mut voltage_samples = 0u32;

    while let Some(message) = reader.next_message() {
        match message.name.as_str() {
            "ATT" => {
                if let Some(roll) = message.field("Roll").and_then(Value::as_f64) {
                    // ArduPilot logs attitude in degrees here, not radians.
                    assert!(
                        roll.abs() <= 180.0,
                        "roll of {roll} degrees is not physical"
                    );
                    max_abs_roll = max_abs_roll.max(roll.abs());
                    attitude_samples += 1;
                }
            }
            "BAT" => {
                if let Some(volts) = message.field("Volt").and_then(Value::as_f64) {
                    assert!((0.0..=100.0).contains(&volts), "battery at {volts} V");
                    voltage_samples += 1;
                }
            }
            _ => {}
        }
    }

    assert!(
        attitude_samples > 100,
        "expected attitude data, saw {attitude_samples} samples"
    );
    println!(
        "{attitude_samples} attitude samples, max |roll| {max_abs_roll:.2} deg, {voltage_samples} battery samples"
    );
}
