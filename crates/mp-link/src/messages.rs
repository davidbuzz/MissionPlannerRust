//! The message log: what the vehicle says, and what it says about our commands.
//!
//! Mission Planner devotes a permanent pane to this on the flight screen, and with good reason.
//! `STATUSTEXT` is how ArduPilot explains a refusal - "PreArm: Compass not calibrated" is the
//! difference between a pilot who fixes the problem and one who keeps pressing Arm. `COMMAND_ACK`
//! is how it answers a command at all: without it the UI can only show that a button was pressed,
//! never whether it worked.
//!
//! The log is link-wide rather than part of a vehicle snapshot. Messages are events, not state:
//! copying a growing history into every `VehicleState` publish would make each snapshot cost more
//! as the flight went on, which is exactly what the Arc-pooled snapshot bus exists to avoid.

use std::collections::VecDeque;
use std::time::{SystemTime, UNIX_EPOCH};

use mp_vehicle::VehicleId;

/// How many messages to retain.
///
/// A long flight can emit thousands; a ground station that keeps all of them grows without bound,
/// and nobody scrolls back that far in flight. The full record belongs in the telemetry log, which
/// is written regardless.
pub const CAPACITY: usize = 500;

/// MAVLink severity, from RFC 5424 as `MAV_SEVERITY` uses it.
///
/// Lower is worse, which reads backwards until you remember it comes from syslog.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// System unusable.
    Emergency,
    /// Action must be taken immediately.
    Alert,
    /// Critical condition.
    Critical,
    /// Error.
    Error,
    /// Warning.
    Warning,
    /// Normal but significant.
    Notice,
    /// Informational.
    Info,
    /// Debug.
    Debug,
}

impl Severity {
    /// Maps a wire value, clamping anything unknown to `Debug` rather than dropping the message.
    ///
    /// A message with a severity we cannot name is still a message the pilot should see.
    #[must_use]
    pub const fn from_wire(value: u8) -> Self {
        match value {
            0 => Self::Emergency,
            1 => Self::Alert,
            2 => Self::Critical,
            3 => Self::Error,
            4 => Self::Warning,
            5 => Self::Notice,
            6 => Self::Info,
            _ => Self::Debug,
        }
    }

    /// Whether this warrants the operator's attention now.
    #[must_use]
    pub const fn is_urgent(self) -> bool {
        matches!(
            self,
            Self::Emergency | Self::Alert | Self::Critical | Self::Error | Self::Warning
        )
    }

    /// A short lowercase label.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Emergency => "emergency",
            Self::Alert => "alert",
            Self::Critical => "critical",
            Self::Error => "error",
            Self::Warning => "warning",
            Self::Notice => "notice",
            Self::Info => "info",
            Self::Debug => "debug",
        }
    }
}

/// One line in the log.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogMessage {
    /// Which vehicle said it.
    pub from: VehicleId,
    /// How bad it is.
    pub severity: Severity,
    /// The text, already trimmed of its padding.
    pub text: String,
    /// Monotonic sequence number, so a UI can tell new lines from redrawn ones.
    pub seq: u64,
    /// When it arrived, as seconds since the Unix epoch.
    ///
    /// Arrival rather than origin: `STATUSTEXT` carries no timestamp of its own, so anything else
    /// would be invented. The difference is the link's latency, which is milliseconds.
    pub received: u64,
}

/// Formats a Unix timestamp as `HH:MM:SS` UTC.
///
/// UTC rather than local time, and labelled as such wherever it is shown. Converting to local
/// needs a timezone database, which is a dependency and a portability problem; and the logs this
/// will be read alongside - dataflash and telemetry - are in UTC anyway, so a local clock here
/// would be the odd one out during the one task this exists for, which is lining up what the
/// vehicle said with what the log recorded.
#[must_use]
pub fn time_of_day(seconds_since_epoch: u64) -> String {
    let seconds_today = seconds_since_epoch % 86_400;
    let hours = seconds_today / 3_600;
    let minutes = (seconds_today % 3_600) / 60;
    let seconds = seconds_today % 60;
    format!("{hours:02}:{minutes:02}:{seconds:02}")
}

/// Now, as seconds since the Unix epoch.
///
/// A clock set before 1970 yields zero rather than an error: a wrong timestamp on a status message
/// is not worth failing over, and the alternative is an `Option` every caller must handle.
fn now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since| since.as_secs())
}

/// A bounded, newest-last ring of messages.
#[derive(Debug, Default)]
pub struct MessageLog {
    entries: VecDeque<LogMessage>,
    next_seq: u64,
    /// Messages dropped because the ring was full, so the UI can say so rather than imply the
    /// flight was quiet.
    dropped: u64,
}

impl MessageLog {
    /// An empty log.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends a message, evicting the oldest if the ring is full.
    pub fn push(&mut self, from: VehicleId, severity: Severity, text: impl Into<String>) {
        let seq = self.next_seq;
        self.next_seq += 1;
        if self.entries.len() >= CAPACITY {
            self.entries.pop_front();
            self.dropped += 1;
        }
        self.entries.push_back(LogMessage {
            from,
            severity,
            text: text.into(),
            seq,
            received: now(),
        });
    }

    /// The most recent messages, newest last, at most `count` of them.
    #[must_use]
    pub fn recent(&self, count: usize) -> Vec<LogMessage> {
        let skip = self.entries.len().saturating_sub(count);
        self.entries.iter().skip(skip).cloned().collect()
    }

    /// Every message held, newest last.
    #[must_use]
    pub fn all(&self) -> Vec<LogMessage> {
        self.entries.iter().cloned().collect()
    }

    /// How many messages are held.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether nothing has been logged.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// How many messages were evicted.
    #[must_use]
    pub const fn dropped(&self) -> u64 {
        self.dropped
    }
}

/// Decodes a `STATUSTEXT` payload into a string.
///
/// The field is a fixed 50 bytes padded with NULs, and ArduPilot does not always pad cleanly: text
/// that exactly fills the field has no terminator at all. Splitting at the first NUL and then
/// trimming handles both, and lossy UTF-8 conversion means a corrupt byte costs one character
/// rather than the whole message.
#[must_use]
pub fn decode_status_text(raw: &[u8]) -> String {
    let end = raw.iter().position(|b| *b == 0).unwrap_or(raw.len());
    let text = raw.get(..end).unwrap_or(raw);
    String::from_utf8_lossy(text).trim().to_owned()
}

/// A human-readable form of `MAV_RESULT`.
#[must_use]
pub const fn command_result_name(result: u8) -> &'static str {
    match result {
        0 => "accepted",
        1 => "temporarily rejected",
        2 => "denied",
        3 => "unsupported",
        4 => "failed",
        5 => "in progress",
        6 => "cancelled",
        7 => "command long only",
        8 => "command int only",
        9 => "unsupported MAV_FRAME",
        _ => "unknown result",
    }
}

/// Whether a `MAV_RESULT` means the command will not happen.
///
/// `IN_PROGRESS` is deliberately not a failure: a long-running command such as a calibration acks
/// repeatedly while it works, and treating that as an error would make every calibration look
/// broken.
#[must_use]
pub const fn command_failed(result: u8) -> bool {
    matches!(result, 1 | 2 | 3 | 4 | 6 | 7 | 8 | 9)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id() -> VehicleId {
        VehicleId {
            sysid: 1,
            compid: 1,
        }
    }

    #[test]
    fn status_text_stops_at_the_first_nul() {
        let mut raw = [0_u8; 50];
        raw[..8].copy_from_slice(b"PreArm: ");
        assert_eq!(decode_status_text(&raw), "PreArm:");
    }

    #[test]
    fn status_text_without_a_terminator_keeps_every_byte() {
        let raw = [b'x'; 50];
        assert_eq!(decode_status_text(&raw).len(), 50);
    }

    #[test]
    fn invalid_utf8_costs_one_character_not_the_message() {
        let mut raw = [0_u8; 50];
        raw[..5].copy_from_slice(&[b'A', 0xff, b'B', b'C', b'D']);
        let text = decode_status_text(&raw);
        assert!(text.starts_with('A'), "{text}");
        assert!(text.ends_with("BCD"), "{text}");
    }

    #[test]
    fn the_ring_evicts_oldest_and_counts_what_it_lost() {
        let mut log = MessageLog::new();
        for n in 0..(CAPACITY + 10) {
            log.push(id(), Severity::Info, format!("line {n}"));
        }
        assert_eq!(log.len(), CAPACITY);
        assert_eq!(log.dropped(), 10);
        let recent = log.recent(1);
        assert_eq!(
            recent.first().map(|m| m.text.as_str()),
            Some(format!("line {}", CAPACITY + 9).as_str())
        );
    }

    #[test]
    fn sequence_numbers_survive_eviction() {
        // A UI that tracks "have I seen this line" by sequence number must not see a number reused
        // after the ring wraps, or an old message would suppress a new one.
        let mut log = MessageLog::new();
        for n in 0..(CAPACITY + 5) {
            log.push(id(), Severity::Info, format!("{n}"));
        }
        let all = log.all();
        let seqs: Vec<u64> = all.iter().map(|m| m.seq).collect();
        assert!(seqs.windows(2).all(|w| w[1] == w[0] + 1));
        #[allow(clippy::cast_possible_truncation)]
        let expected_first = 5_u64;
        assert_eq!(seqs.first().copied(), Some(expected_first));
    }

    #[test]
    fn recent_asks_for_more_than_exists_without_panicking() {
        let mut log = MessageLog::new();
        log.push(id(), Severity::Warning, "only one");
        assert_eq!(log.recent(100).len(), 1);
    }

    #[test]
    fn messages_are_stamped_with_when_they_arrived() {
        let mut log = MessageLog::new();
        log.push(id(), Severity::Warning, "PreArm: Compass not calibrated");
        let message = log.recent(1).pop().expect("a message");
        // Any plausible present-day time. The point is that it is stamped at all, not what the
        // clock says.
        assert!(message.received > 1_700_000_000, "{}", message.received);
    }

    #[test]
    fn the_time_of_day_is_readable_and_fixed_width() {
        // Fixed width matters: the timestamps form a column, and a column that jitters is harder
        // to scan than no column at all.
        assert_eq!(time_of_day(0), "00:00:00");
        assert_eq!(time_of_day(3_661), "01:01:01");
        assert_eq!(time_of_day(86_399), "23:59:59");
        // Rolls over at midnight rather than running to 24 and beyond.
        assert_eq!(time_of_day(86_400), "00:00:00");
        assert_eq!(time_of_day(1_758_000_000).len(), 8);
    }

    #[test]
    fn unknown_severities_are_shown_not_dropped() {
        assert_eq!(Severity::from_wire(200), Severity::Debug);
        assert!(!Severity::from_wire(200).is_urgent());
        assert!(Severity::from_wire(4).is_urgent());
    }

    #[test]
    fn in_progress_is_not_a_failure() {
        // A gyro calibration acks IN_PROGRESS repeatedly; calling that a failure would make every
        // calibration in the setup screens look broken.
        assert!(!command_failed(5));
        assert!(!command_failed(0));
        assert!(command_failed(2));
        assert!(command_failed(4));
    }
}
