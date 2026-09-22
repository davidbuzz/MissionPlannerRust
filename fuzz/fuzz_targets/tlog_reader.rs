//! Fuzzes the telemetry log reader.
//!
//! Logs get truncated by crashes, concatenated by accident and copied off failing SD cards. The
//! reader must terminate on any input and never hand back a record that points outside the buffer.

#![no_main]

use libfuzzer_sys::fuzz_target;
use mp_log::TlogReader;
use mp_mavlink_dialects::all::DIALECT;

fuzz_target!(|data: &[u8]| {
    let mut reader = TlogReader::new(data);
    let mut records = 0u64;

    while let Some(record) = reader.next_record(&DIALECT) {
        assert!(record.offset < data.len(), "record offset outside the buffer");
        assert!(!record.frame.is_empty(), "an empty frame is not a record");
        assert!(
            record.offset + record.frame.len() <= data.len(),
            "frame extends past the end of the log"
        );
        records += 1;
        // A log cannot contain more records than it has bytes; this catches a reader that fails
        // to advance.
        assert!(records as usize <= data.len(), "reader is not making progress");
    }
});
