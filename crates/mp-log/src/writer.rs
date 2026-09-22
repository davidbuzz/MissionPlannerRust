//! Writing telemetry logs.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::{LogError, TIMESTAMP_LEN, now_micros};

/// Writes frames to a `.tlog` in Mission Planner's format.
///
/// Buffered, because a telemetry link produces small writes at high rate and an unbuffered
/// `write` per frame would put a syscall in the link thread's hot path.
#[derive(Debug)]
pub struct TlogWriter {
    out: BufWriter<File>,
    frames: u64,
    bytes: u64,
}

impl TlogWriter {
    /// Creates a log, failing if it already exists so a recording cannot silently overwrite one.
    pub fn create(path: impl AsRef<Path>) -> Result<Self, LogError> {
        let path = path.as_ref();
        let file = File::create_new(path)
            .map_err(|e| LogError::io(format!("creating {}", path.display()), e))?;
        Ok(Self {
            out: BufWriter::with_capacity(64 * 1024, file),
            frames: 0,
            bytes: 0,
        })
    }

    /// Appends a frame stamped with the current time.
    pub fn write_frame(&mut self, frame: &[u8]) -> Result<(), LogError> {
        self.write_frame_at(frame, now_micros())
    }

    /// Appends a frame with an explicit timestamp, used by tests and by log conversion.
    pub fn write_frame_at(&mut self, frame: &[u8], timestamp_micros: u64) -> Result<(), LogError> {
        // Big-endian: the format predates any thought of endianness portability, and Mission
        // Planner reverses the bytes on read.
        self.out
            .write_all(&timestamp_micros.to_be_bytes())
            .map_err(|e| LogError::io("writing timestamp", e))?;
        self.out
            .write_all(frame)
            .map_err(|e| LogError::io("writing frame", e))?;
        self.frames += 1;
        self.bytes += (TIMESTAMP_LEN + frame.len()) as u64;
        Ok(())
    }

    /// Frames written so far.
    #[must_use]
    pub const fn frames(&self) -> u64 {
        self.frames
    }

    /// Bytes written so far, including timestamps.
    #[must_use]
    pub const fn bytes(&self) -> u64 {
        self.bytes
    }

    /// Flushes buffered data. Called on drop, but a recorder should call it periodically so a
    /// crash loses seconds rather than minutes.
    pub fn flush(&mut self) -> Result<(), LogError> {
        self.out
            .flush()
            .map_err(|e| LogError::io("flushing log", e))
    }
}

impl Drop for TlogWriter {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}
