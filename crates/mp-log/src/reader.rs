//! Reading telemetry logs.

use mp_mavlink::{Dialect, STX_V1, STX_V2, parse};

use crate::TIMESTAMP_LEN;

/// One record from a log.
#[derive(Debug, Clone, Copy)]
pub struct TlogRecord<'a> {
    /// Unix epoch microseconds, as recorded.
    pub timestamp_micros: u64,
    /// The raw MAVLink frame.
    pub frame: &'a [u8],
    /// Byte offset of the frame within the log, for diagnostics and seeking.
    pub offset: usize,
}

/// Reads records from an in-memory log.
///
/// # Why it resynchronises
///
/// The format has no framing beyond "timestamp, frame", so any corruption, any writer that
/// flushed mid-frame, and any console text the autopilot emitted before MAVLink started will
/// desynchronise a reader that trusts the structure. Real logs contain all three: an ArduPilot
/// autotest log opens with several hundred bytes of boot text wrapped in timestamps.
///
/// So the reader scans for a start byte and validates every frame's checksum, and treats the
/// timestamp as a hint rather than a guarantee.
#[derive(Debug)]
pub struct TlogReader<'a> {
    data: &'a [u8],
    pos: usize,
}

/// How far to scan for a start byte before giving up on a record.
const MAX_SCAN: usize = 280;

impl<'a> TlogReader<'a> {
    /// Wraps a log already in memory.
    #[must_use]
    pub const fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    /// Reads the next valid record, or `None` at the end of the log.
    pub fn next_record<D: Dialect + ?Sized>(&mut self, dialect: &D) -> Option<TlogRecord<'a>> {
        while self.pos + TIMESTAMP_LEN <= self.data.len() {
            let stamp_at = self.pos;
            self.pos += TIMESTAMP_LEN;

            let mut scan = self.pos;
            let limit = (self.pos + MAX_SCAN).min(self.data.len());
            while scan < limit
                && self.data.get(scan) != Some(&STX_V2)
                && self.data.get(scan) != Some(&STX_V1)
            {
                scan += 1;
            }
            if scan >= limit {
                if limit <= self.pos {
                    break;
                }
                self.pos = limit;
                continue;
            }

            let Some(window) = self.data.get(scan..) else {
                break;
            };
            match parse(window, dialect) {
                Ok((frame, used)) => {
                    let stamp = self
                        .data
                        .get(stamp_at..stamp_at + TIMESTAMP_LEN)
                        .and_then(|b| <[u8; 8]>::try_from(b).ok())
                        .map_or(0, u64::from_be_bytes);
                    self.pos = scan + used;
                    return Some(TlogRecord {
                        timestamp_micros: stamp,
                        frame: frame.raw,
                        offset: scan,
                    });
                }
                Err(_) => self.pos = scan + 1,
            }
        }
        None
    }

    /// Collects every record. Convenient for tests; a large log should be streamed instead.
    pub fn records<D: Dialect + ?Sized>(mut self, dialect: &D) -> Vec<TlogRecord<'a>> {
        let mut out = Vec::new();
        while let Some(record) = self.next_record(dialect) {
            out.push(record);
        }
        out
    }
}
