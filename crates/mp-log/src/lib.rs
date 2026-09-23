//! Telemetry log (`.tlog`) reading and writing.
//!
//! # Format
//!
//! A tlog is a sequence of records: an 8-byte **big-endian** microsecond timestamp followed by one
//! raw MAVLink frame. There is no header, no index and no framing beyond that, which is why a
//! reader must resynchronise rather than trust the structure.
//!
//! Byte compatibility with Mission Planner is a requirement, not a nicety: users have years of
//! recordings, and the two applications must be able to read each other's logs
//! (`DELIVERABLES.md` D17).

#![forbid(unsafe_code)]

pub mod dataflash;
pub mod index;
pub mod plot;
pub mod reader;
#[cfg(test)]
mod testlog;
pub mod track;
pub mod writer;

pub use dataflash::{DataflashReader, DataflashStats, LogMessage, MessageFormat, Value};
pub use reader::{TlogReader, TlogRecord};
pub use writer::TlogWriter;

/// Size of the timestamp that precedes each frame.
pub const TIMESTAMP_LEN: usize = 8;

/// Errors from log I/O.
#[derive(Debug, thiserror::Error)]
pub enum LogError {
    /// Underlying I/O failure.
    #[error("{context}: {source}")]
    Io {
        /// What was being attempted.
        context: String,
        /// The underlying error.
        #[source]
        source: std::io::Error,
    },
}

impl LogError {
    /// Attaches context to an I/O error.
    pub fn io(context: impl Into<String>, source: std::io::Error) -> Self {
        Self::Io {
            context: context.into(),
            source,
        }
    }
}

/// Unix epoch microseconds, the unit tlog timestamps use.
#[must_use]
pub fn now_micros() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_micros()).unwrap_or(u64::MAX))
}
