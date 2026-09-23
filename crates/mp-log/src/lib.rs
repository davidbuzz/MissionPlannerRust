//! Telemetry log (`.tlog`) reading.
//!
//! Writing is the link's (`mp_link::tlog`): the link records itself, as `MAVLinkInterface` does,
//! and it sits below this crate in PLAN.md §5.1's layers. `tests/tlog.rs` reads what it writes.
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
//!
//! # The DataFlash Logs page's conversions
//!
//! Three of the flight screen's dataflash buttons live here, each a function the page calls with
//! [`convert::flight_mode_name`] for its flight modes: "Convert .Bin to .Log"
//! ([`convert::convert_bin_file`]), "Create Matlab file" ([`matlab::process_log_file`]) and
//! "Auto Analysis" ([`analysis::analyse`], which runs ArduPilot's own analyzer). The fourth,
//! "Create KML + gpx", is `mp_kml::dflog::dflog_to_kml`, reading through [`dflogbuffer`]. All of
//! them are held to Mission Planner's own output under `testdata/dataflash/golden`.

#![forbid(unsafe_code)]

pub mod analysis;
pub mod convert;
pub mod dataflash;
pub mod dflogbuffer;
pub mod index;
pub mod matlab;
pub mod netfmt;
pub mod overlay;
pub mod plot;
pub mod reader;
#[cfg(test)]
mod testlog;
pub mod track;
pub mod zip;

pub use dataflash::{DataflashReader, DataflashStats, LogMessage, MessageFormat, Value};
pub use reader::{TlogReader, TlogRecord};

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
