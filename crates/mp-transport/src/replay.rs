//! Replaying recorded logs as if they were a live link.

use std::io;
use std::path::Path;
use std::time::Duration;

use crate::{OpenError, Transport};

/// Replays a recorded byte stream.
///
/// Telemetry logs (`.tlog`) interleave an 8-byte big-endian microsecond timestamp before each
/// frame. Those bytes are left in the stream: the frame decoder resynchronises past them, and
/// keeping them means the replay is byte-exact with what was recorded. Callers that want the
/// timestamps can use [`ReplayTransport::with_timestamps`].
#[derive(Debug)]
pub struct ReplayTransport {
    data: Vec<u8>,
    pos: usize,
    chunk: usize,
    /// `file:<name> (<n> bytes)`, kept ready for [`Transport::description`]. The log is loaded
    /// whole and never changes, so neither does this.
    description: String,
    loop_forever: bool,
}

/// What a replay is called: the log's name and its size.
fn describe(name: impl std::fmt::Display, len: usize) -> String {
    format!("file:{name} ({len} bytes)")
}

impl ReplayTransport {
    /// Default read size, chosen to resemble a serial driver's returns rather than to be fast.
    pub const DEFAULT_CHUNK: usize = 512;

    /// Loads a log file into memory.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, OpenError> {
        let path = path.as_ref();
        let data = std::fs::read(path)
            .map_err(|e| OpenError::io(format!("reading {}", path.display()), e))?;
        Ok(Self {
            description: describe(path.display(), data.len()),
            data,
            pos: 0,
            chunk: Self::DEFAULT_CHUNK,
            loop_forever: false,
        })
    }

    /// Builds a replay from bytes already in memory, for tests and benchmarks.
    #[must_use]
    pub fn from_bytes(name: impl Into<String>, data: Vec<u8>) -> Self {
        Self {
            description: describe(name.into(), data.len()),
            data,
            pos: 0,
            chunk: Self::DEFAULT_CHUNK,
            loop_forever: false,
        }
    }

    /// Sets how many bytes a single `read` returns at most. Small values exercise the decoder's
    /// partial-frame handling, which is where stream bugs hide.
    #[must_use]
    pub const fn with_chunk_size(mut self, chunk: usize) -> Self {
        self.chunk = if chunk == 0 { 1 } else { chunk };
        self
    }

    /// Restarts from the beginning when the log is exhausted, for soak tests.
    #[must_use]
    pub const fn looping(mut self, loop_forever: bool) -> Self {
        self.loop_forever = loop_forever;
        self
    }

    /// Whether the log has been fully replayed.
    #[must_use]
    pub const fn is_exhausted(&self) -> bool {
        self.pos >= self.data.len()
    }

    /// Bytes replayed so far.
    #[must_use]
    pub const fn position(&self) -> usize {
        self.pos
    }

    /// Total bytes in the log.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.data.len()
    }

    /// Whether the log is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Iterates `(timestamp_micros, frame_offset)` pairs for a `.tlog`, without consuming the
    /// replay. Timestamps are big-endian microseconds since the Unix epoch.
    #[must_use]
    pub fn with_timestamps(&self) -> Vec<(u64, usize)> {
        let mut out = Vec::new();
        let mut pos = 0usize;
        while pos + 8 <= self.data.len() {
            let Some(stamp) = self.data.get(pos..pos + 8) else {
                break;
            };
            let mut bytes = [0u8; 8];
            bytes.copy_from_slice(stamp);
            out.push((u64::from_be_bytes(bytes), pos + 8));
            // Advance past the timestamp; the caller decides how long the frame is.
            pos += 8;
            // Skip to the next plausible start byte so the next timestamp aligns.
            let mut scan = pos;
            while scan < self.data.len()
                && self.data.get(scan) != Some(&0xFD)
                && self.data.get(scan) != Some(&0xFE)
            {
                scan += 1;
            }
            if scan >= self.data.len() {
                break;
            }
            pos = scan + 1;
        }
        out
    }
}

impl Transport for ReplayTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.is_exhausted() {
            if self.loop_forever && !self.data.is_empty() {
                self.pos = 0;
            } else {
                return Ok(0);
            }
        }
        let remaining = self.data.len() - self.pos;
        let n = remaining.min(buf.len()).min(self.chunk);
        let (Some(dst), Some(src)) = (buf.get_mut(..n), self.data.get(self.pos..self.pos + n))
        else {
            return Ok(0);
        };
        dst.copy_from_slice(src);
        self.pos += n;
        Ok(n)
    }

    fn write_all(&mut self, _buf: &[u8]) -> io::Result<()> {
        // A recording cannot answer. Silently discarding is right: the link engine should behave
        // identically whether it is talking to a vehicle or to history.
        Ok(())
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn is_open(&self) -> bool {
        self.loop_forever || !self.is_exhausted()
    }

    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        Ok(())
    }
}
