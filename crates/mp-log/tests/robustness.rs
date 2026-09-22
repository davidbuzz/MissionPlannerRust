//! Deterministic robustness testing for the log reader.
//!
//! Logs get truncated by crashes, concatenated by accident, and copied off failing SD cards. The
//! reader must terminate on any input and never hand back a record pointing outside the buffer.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
// The generator narrows u64 to smaller types deliberately; it is producing test input, not
// interpreting vehicle data, so truncation is the intent rather than a hazard.
#![allow(clippy::cast_possible_truncation)]

use mp_log::TlogReader;
use mp_log::dataflash::DataflashReader;
use mp_mavlink_dialects::all::DIALECT;

struct Rng(u64);

impl Rng {
    const fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn byte(&mut self) -> u8 {
        (self.next_u64() >> 24) as u8
    }

    fn below(&mut self, limit: usize) -> usize {
        if limit == 0 {
            0
        } else {
            (self.next_u64() % limit as u64) as usize
        }
    }
}

fn check(data: &[u8]) {
    let mut reader = TlogReader::new(data);
    let mut records = 0usize;
    while let Some(record) = reader.next_record(&DIALECT) {
        assert!(
            record.offset < data.len(),
            "record offset {} outside buffer",
            record.offset
        );
        assert!(!record.frame.is_empty());
        assert!(
            record.offset + record.frame.len() <= data.len(),
            "frame at {} extends past the end",
            record.offset
        );
        records += 1;
        // A reader that fails to advance would loop forever; bound it by the input size.
        assert!(records <= data.len(), "reader is not making progress");
    }
}

#[test]
fn arbitrary_bytes_terminate_and_stay_in_bounds() {
    let mut rng = Rng::new(0xBEEF_0001);
    for _ in 0..2_000 {
        let len = rng.below(2_000);
        let mut data = Vec::with_capacity(len);
        for _ in 0..len {
            data.push(rng.byte());
        }
        check(&data);
    }
}

#[test]
fn a_truncated_real_log_still_terminates() {
    // The common corruption: a recording cut off mid-frame by a crash or a full disk.
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    let full = std::fs::read(path).expect("corpus");
    let mut rng = Rng::new(0xBEEF_0002);

    for _ in 0..200 {
        let cut = rng.below(full.len());
        check(&full[..cut]);
    }
}

#[test]
fn a_log_with_corruption_in_the_middle_keeps_reading_afterwards() {
    // Corruption must cost the frames it touches, not the rest of the flight.
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/mavlink/autotest.tlog"
    );
    let mut data = std::fs::read(path).expect("corpus");
    let clean = TlogReader::new(&data).records(&DIALECT).len();
    assert!(clean > 1_000);

    let mut rng = Rng::new(0xBEEF_0003);
    for _ in 0..500 {
        let at = rng.below(data.len());
        data[at] ^= 0xFF;
    }

    let damaged = TlogReader::new(&data).records(&DIALECT).len();
    check(&data);
    // 500 smashed bytes in a 1.5 MB log should cost a small fraction of the records.
    assert!(
        damaged * 100 >= clean * 90,
        "500 corrupt bytes cost {} of {clean} records",
        clean - damaged
    );
}

/// Runs the dataflash parser to exhaustion, asserting it terminates and stays in bounds.
fn check_dataflash(data: &[u8]) {
    let mut reader = DataflashReader::new(data);
    let mut messages = 0usize;
    while reader.next_message().is_some() {
        messages += 1;
        // A log cannot hold more messages than it has bytes; this catches a parser that fails to
        // advance, which would otherwise hang rather than fail.
        assert!(
            messages <= data.len(),
            "dataflash reader is not making progress"
        );
    }
}

#[test]
fn arbitrary_bytes_never_panic_the_dataflash_parser() {
    let mut rng = Rng::new(0xDF10_0001);
    for _ in 0..2_000 {
        let len = rng.below(3_000);
        let mut data = Vec::with_capacity(len);
        for _ in 0..len {
            data.push(rng.byte());
        }
        check_dataflash(&data);
    }
}

#[test]
fn a_dataflash_log_full_of_headers_does_not_hang() {
    // The pathological input for this format: every byte looks like the start of a message,
    // including format definitions, so a parser that trusts declared lengths can be walked
    // anywhere.
    for pattern in [
        vec![0xA3u8; 4096],
        [0xA3u8, 0x95, 0x80].repeat(1024),
        [0xA3u8, 0x95, 0xFF].repeat(1024),
    ] {
        check_dataflash(&pattern);
    }
}

#[test]
fn a_truncated_dataflash_log_terminates() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/dataflash.bin");
    let full = std::fs::read(path).expect("corpus");
    let mut rng = Rng::new(0xDF10_0002);
    for _ in 0..100 {
        let cut = rng.below(full.len());
        check_dataflash(&full[..cut]);
    }
}

#[test]
fn corrupting_a_dataflash_log_costs_only_the_damaged_region() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/dataflash.bin");
    let mut data = std::fs::read(path).expect("corpus");

    let mut clean_reader = DataflashReader::new(&data);
    let mut clean = 0usize;
    while clean_reader.next_message().is_some() {
        clean += 1;
    }
    assert!(clean > 10_000);

    let mut rng = Rng::new(0xDF10_0003);
    for _ in 0..200 {
        let at = rng.below(data.len());
        data[at] ^= 0xFF;
    }

    let mut damaged_reader = DataflashReader::new(&data);
    let mut damaged = 0usize;
    while damaged_reader.next_message().is_some() {
        damaged += 1;
    }
    check_dataflash(&data);

    // Unlike the tlog format, dataflash has no per-message checksum, so a corrupted length field
    // can desynchronise the stream until the next recognisable header. Losing some records is
    // expected; losing the log is not.
    assert!(
        damaged * 100 >= clean * 50,
        "200 corrupt bytes cost {} of {clean} messages",
        clean.saturating_sub(damaged)
    );
}
