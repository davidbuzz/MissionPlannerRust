// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! Replaying recorded logs as if they were a live link.
//!
//! By default a replay is as fast as the reader: tests, benchmarks and the differential harness
//! want the whole flight now. [`ReplayTransport::paced`] makes one play the way Mission
//! Planner's Telemetry Logs page plays a `.tlog`: in the time it was recorded in, scaled by a
//! speed, paused and resumed, and moved to a place in the file - through a [`Playback`] shared
//! with whoever holds the controls.

use std::io;
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use web_time::{Duration, Instant};

use crate::{OpenError, Transport};

/// Replays a recorded byte stream.
///
/// Telemetry logs (`.tlog`) interleave an 8-byte big-endian microsecond timestamp before each
/// frame. Those bytes are left in the stream: the frame decoder resynchronises past them, and
/// keeping them means the replay is byte-exact with what was recorded. Callers that want the
/// timestamps can use [`ReplayTransport::with_timestamps`].
///
/// A read never runs past the end of a record, as `readlogPacketMavlink` reads one record at a
/// time, so every frame decoded from what one read returned was recorded at the time
/// [`Transport::read_time`] then reports: the newest usable timestamp read so far, which the link
/// stamps each packet's `datetime` with.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6511-6650`
#[derive(Debug)]
pub struct ReplayTransport {
    data: Vec<u8>,
    pos: usize,
    chunk: usize,
    /// Where the record being read ends; a read at or past it starts the next record.
    record_end: usize,
    /// `lastlogread`: the newest usable timestamp read, in microseconds, `None` while the C#'s is
    /// still `DateTime.MinValue`. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:482, 6553-6557`
    last_log_read: Option<u64>,
    /// `file:<name> (<n> bytes)`, kept ready for [`Transport::description`]. The log is loaded
    /// whole and never changes, so neither does this.
    description: String,
    loop_forever: bool,
    /// The clock and the controls, when the replay is [`ReplayTransport::paced`].
    paced: Option<Paced>,
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
        let data = mp_os::fs::read(path)
            .map_err(|e| OpenError::io(format!("reading {}", path.display()), e))?;
        Ok(Self {
            description: describe(path.display(), data.len()),
            data,
            pos: 0,
            chunk: Self::DEFAULT_CHUNK,
            record_end: 0,
            last_log_read: None,
            loop_forever: false,
            paced: None,
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
            record_end: 0,
            last_log_read: None,
            loop_forever: false,
            paced: None,
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

    /// Plays the log in the time it was recorded in, under the controls the returned
    /// [`Playback`] holds: paused or playing, a speed, and a place to move to.
    ///
    /// Mission Planner's `mainloop` reads one packet, then sleeps for the time between its
    /// timestamp and the last one's divided by the playback speed; this does the same, waiting
    /// before each record instead of after it. While paced the replay never reports itself
    /// closed: reaching the end is where the C# stops playing, not where it forgets the file,
    /// and the track bar can still take it back.
    /// `// C#: GCSViews/FlightData.cs:3474-3528`
    #[must_use]
    pub fn paced(mut self) -> (Self, Arc<Playback>) {
        let control = Arc::new(Playback::new(self.data.len()));
        control.position.store(self.pos, Ordering::Release);
        self.paced = Some(Paced {
            records: records(&self.data),
            control: Arc::clone(&control),
            last_stamp: None,
            due: None,
            timeout: crate::DEFAULT_READ_TIMEOUT,
        });
        (self, control)
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
        if self.paced.is_some() {
            return Ok(self.read_paced(buf));
        }
        if self.is_exhausted() {
            if self.loop_forever && !self.data.is_empty() {
                self.pos = 0;
                self.record_end = 0;
            } else {
                return Ok(0);
            }
        }
        // A new record: its timestamp, if usable, is the clock from here on.
        // C#: MAVLinkInterface.cs:6535-6558
        if self.pos >= self.record_end {
            let record = record_at(&self.data, self.pos);
            self.record_end = record.end;
            if record.stamp.is_some() {
                self.last_log_read = record.stamp;
            }
        }
        let remaining = self.record_end.min(self.data.len()) - self.pos;
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
        self.loop_forever || self.paced.is_some() || !self.is_exhausted()
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        if let Some(paced) = self.paced.as_mut() {
            paced.timeout = timeout;
        }
        Ok(())
    }

    /// The recording's clock, `lastlogread`, which the link stamps each packet with as
    /// `readlogPacketMavlink` stamps `cs.datetime`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6649`
    fn read_time(&self) -> crate::ReadTime {
        crate::ReadTime::Recorded(self.last_log_read)
    }
}

// --- Playing at the recorded pace -----------------------------------------------------------------

/// The controls of a paced replay, shared between the replay and whoever drives it - Mission
/// Planner's Telemetry Logs page, `tabTLogs`.
///
/// Atomics rather than a lock: the replay reads them on the link thread every pass, and a
/// screen that holds a lock across a frame must not be able to stall a link.
#[derive(Debug)]
pub struct Playback {
    /// `!MainV2.comPort.logreadmode`: Play/Pause toggles it.
    paused: AtomicBool,
    /// `LogPlayBackSpeed`, as the bits of an `f64`.
    speed: AtomicU64,
    /// A place asked for and not yet moved to, as a byte offset; [`NO_SEEK`] when there is none.
    seek: AtomicU64,
    /// Where the replay is, in bytes: `logplaybackfile.BaseStream.Position`.
    position: AtomicUsize,
    /// The file's length: `logplaybackfile.BaseStream.Length`.
    len: usize,
}

/// No seek pending.
const NO_SEEK: u64 = u64::MAX;

impl Playback {
    /// Playing, at `LogPlayBackSpeed`'s start, 1.0. `// C#: GCSViews/FlightData.cs:137`
    fn new(len: usize) -> Self {
        Self {
            paused: AtomicBool::new(false),
            speed: AtomicU64::new(1.0f64.to_bits()),
            seek: AtomicU64::new(NO_SEEK),
            position: AtomicUsize::new(0),
            len,
        }
    }

    /// Stops or restarts the replay where it is.
    pub fn set_paused(&self, paused: bool) {
        self.paused.store(paused, Ordering::Release);
    }

    /// Whether it is stopped.
    #[must_use]
    pub fn is_paused(&self) -> bool {
        self.paused.load(Ordering::Acquire)
    }

    /// Whether records are going out: not paused, and not at the end - where the C#'s main loop
    /// sets `logreadmode` false itself. `// C#: GCSViews/FlightData.cs:3556-3568`
    #[must_use]
    pub fn is_playing(&self) -> bool {
        !self.is_paused() && self.position() < self.len
    }

    /// Sets the speed, recorded time over replayed time. The C# replaces a speed of 0 with 0.01
    /// before dividing by it; so does this. `// C#: GCSViews/FlightData.cs:3497-3498`
    pub fn set_speed(&self, speed: f64) {
        let speed = if speed == 0.0 { 0.01 } else { speed };
        self.speed.store(speed.to_bits(), Ordering::Release);
    }

    /// The speed.
    #[must_use]
    pub fn speed(&self) -> f64 {
        f64::from_bits(self.speed.load(Ordering::Acquire))
    }

    /// Moves to `fraction` of the way through the file, as `tracklog_Scroll` sets
    /// `BaseStream.Position = Length * (tracklog.Value / 100.0)`.
    /// `// C#: GCSViews/FlightData.cs:5361-5378`
    pub fn seek_fraction(&self, fraction: f64) {
        // A file's length in bytes, times a fraction clamped to 0..1.
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let offset = (self.len as f64 * fraction.clamp(0.0, 1.0)) as u64;
        self.seek.store(offset, Ordering::Release);
        // Said at once, as the C#'s stream position is, rather than when the link thread next
        // reads: the page reads it back straight after the scroll.
        self.position.store(
            usize::try_from(offset).unwrap_or(self.len).min(self.len),
            Ordering::Release,
        );
    }

    /// Bytes replayed so far.
    #[must_use]
    pub fn position(&self) -> usize {
        self.position.load(Ordering::Acquire)
    }

    /// The file's length in bytes.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether the file is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// What a paced replay keeps between reads.
#[derive(Debug)]
struct Paced {
    /// Every record in the file, in order and without gaps.
    records: Vec<Record>,
    /// The controls.
    control: Arc<Playback>,
    /// `MainV2.comPort.lastlogread`: the newest timestamp replayed, in microseconds.
    last_stamp: Option<u64>,
    /// When the last record went out, on the schedule the waits keep - so time spent outside
    /// the replay is taken off the next wait, as the C#'s `timeerror` takes it off.
    due: Option<Instant>,
    /// The link's read timeout: the longest one read waits.
    timeout: Duration,
}

/// One record of a `.tlog`: an eight-byte big-endian timestamp in microseconds, then one
/// MAVLink frame. `start..end` runs from the first byte of the timestamp to the last of the
/// frame, with any bytes skipped to find the frame's start byte inside it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Record {
    start: usize,
    end: usize,
    /// The timestamp, when there was a believable one.
    stamp: Option<u64>,
}

/// MAVLink 1's start byte.
const STX_V1: u8 = 0xFE;
/// MAVLink 2's start byte.
const STX_V2: u8 = 0xFD;
/// The signed flag in a MAVLink 2 frame's `incompat_flags`.
const IFLAG_SIGNED: u8 = 0x01;
/// The signature block a signed MAVLink 2 frame carries.
const SIGNATURE_LEN: usize = 13;

/// Splits a `.tlog` into records the way `readlogPacketMavlink` reads them.
///
/// Eight bytes of timestamp - unless the first of them is a start byte, in which case the
/// record has none and the eight are rewound; a timestamp is believed when it is under
/// 9 999 999 hours. Then the frame: its start byte, scanned forward to if it is not the next
/// byte, and a length from its header - the payload, the header, the start byte, two bytes of
/// checksum, and thirteen of signature on a signed MAVLink 2 frame.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6511-6650`
fn records(data: &[u8]) -> Vec<Record> {
    let mut out = Vec::new();
    let mut pos = 0;
    while pos < data.len() {
        let record = record_at(data, pos);
        out.push(record);
        pos = record.end;
    }
    out
}

/// The record that starts at `start`, as [`records`] splits them: never empty, and running to
/// the end of the data when its timestamp, start byte or length is cut off.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6511-6650`
fn record_at(data: &[u8], start: usize) -> Record {
    let mut pos = start;
    let mut stamp = None;
    if !matches!(data.get(pos), Some(&STX_V1 | &STX_V2)) {
        let Some(bytes) = data.get(pos..pos + 8) else {
            // A truncated timestamp: the C#'s read comes back short and it finds no frame.
            return Record {
                start,
                end: data.len(),
                stamp: None,
            };
        };
        let mut array = [0u8; 8];
        array.copy_from_slice(bytes);
        let micros = u64::from_be_bytes(array);
        if micros / 1000 / 1000 / 60 / 60 < 9_999_999 {
            stamp = Some(micros);
        }
        pos += 8;
    }
    // "lost sync byte": on to the next start byte.
    while pos < data.len() && !matches!(data.get(pos), Some(&STX_V1 | &STX_V2)) {
        pos += 1;
    }
    let (Some(&stx), Some(&payload)) = (data.get(pos), data.get(pos + 1)) else {
        return Record {
            start,
            end: data.len(),
            stamp,
        };
    };
    let header = if stx == STX_V2 { 9 } else { 5 };
    let mut length = usize::from(payload) + header + 1 + 2;
    if stx == STX_V2
        && data
            .get(pos + 2)
            .is_some_and(|flags| flags & IFLAG_SIGNED != 0)
    {
        length += SIGNATURE_LEN;
    }
    let end = (pos + length).min(data.len());
    Record { start, end, stamp }
}

/// How long to wait before a record stamped `stamp`, after one stamped `last`, at `speed`.
///
/// The C#'s `act` is the milliseconds between the two, taken as 0 when it is over 9999 or
/// negative; the sleep is `act / LogPlayBackSpeed`, at most 1000 ms. A record with no
/// timestamp leaves `lastlogread` where it was, so it goes without waiting.
/// `// C#: GCSViews/FlightData.cs:3474-3528`
fn wait_before(last: Option<u64>, stamp: Option<u64>, speed: f64) -> Duration {
    let (Some(last), Some(stamp)) = (last, stamp) else {
        return Duration::ZERO;
    };
    #[allow(clippy::cast_precision_loss)] // microseconds between two packets
    let act = (i128::from(stamp) - i128::from(last)) as f64 / 1000.0;
    let act = if (0.0..=9999.0).contains(&act) {
        act
    } else {
        0.0
    };
    let speed = if speed == 0.0 { 0.01 } else { speed };
    let ts = (act / speed).min(1000.0);
    if ts > 0.0 && ts.is_finite() {
        Duration::from_secs_f64(ts / 1000.0)
    } else {
        Duration::ZERO
    }
}

impl ReplayTransport {
    /// A read while paced: the controls first, then the next record once its time has come.
    fn read_paced(&mut self, buf: &mut [u8]) -> usize {
        let Some(paced) = self.paced.as_mut() else {
            return 0;
        };
        let control = Arc::clone(&paced.control);
        let seek = control.seek.swap(NO_SEEK, Ordering::AcqRel);
        if seek != NO_SEEK {
            // To the first record at or after the byte asked for. The C# lands on the byte
            // itself, reads whatever eight bytes are there as a timestamp and resyncs on the
            // next start byte; starting on a record keeps that stray timestamp out of the
            // waits. It also forgets `lastlogread`, as `tracklog_Scroll` sets it to MinValue.
            let target = usize::try_from(seek).unwrap_or(self.data.len());
            let index = paced
                .records
                .partition_point(|record| record.start < target);
            self.pos = paced
                .records
                .get(index)
                .map_or(self.data.len(), |record| record.start);
            paced.last_stamp = None;
            paced.due = None;
            // C#: GCSViews/FlightData.cs:5367
            self.last_log_read = None;
            control.position.store(self.pos, Ordering::Release);
        }
        if control.is_paused() || self.pos >= self.data.len() {
            // Resuming starts the clock again rather than rushing out what the pause held up.
            paced.due = None;
            return 0;
        }
        let index = paced
            .records
            .partition_point(|record| record.end <= self.pos);
        let Some(record) = paced.records.get(index).copied() else {
            return 0;
        };
        if self.pos == record.start {
            let now = Instant::now();
            let wait = wait_before(paced.last_stamp, record.stamp, control.speed());
            let due = paced.due.map_or(now, |last| {
                let due = last + wait;
                // Behind by more than the C#'s one-second cap: start the schedule again here
                // rather than racing to catch up.
                if now.saturating_duration_since(due) > Duration::from_secs(1) {
                    now
                } else {
                    due
                }
            });
            if due > now {
                let remaining = due - now;
                wasm_thread::sleep(remaining.min(paced.timeout));
                if remaining > paced.timeout {
                    return 0;
                }
            }
            paced.due = Some(due);
            if record.stamp.is_some() {
                paced.last_stamp = record.stamp;
                self.last_log_read = record.stamp;
            }
        }
        let n = (record.end - self.pos).min(buf.len()).min(self.chunk);
        let (Some(dst), Some(src)) = (buf.get_mut(..n), self.data.get(self.pos..self.pos + n))
        else {
            return 0;
        };
        dst.copy_from_slice(src);
        self.pos += n;
        control.position.store(self.pos, Ordering::Release);
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A MAVLink 1 frame with `payload` bytes of payload: start byte, length, then filler.
    fn v1(payload: u8) -> Vec<u8> {
        let mut frame = vec![STX_V1, payload];
        frame.resize(usize::from(payload) + 8, 0x11);
        frame
    }

    /// A MAVLink 2 frame, signed or not.
    fn v2(payload: u8, signed: bool) -> Vec<u8> {
        let mut frame = vec![STX_V2, payload, u8::from(signed)];
        let signature = if signed { SIGNATURE_LEN } else { 0 };
        frame.resize(usize::from(payload) + 12 + signature, 0x22);
        frame
    }

    /// A tlog of `(timestamp in ms, frame)` records.
    fn tlog(records: &[(u64, Vec<u8>)]) -> Vec<u8> {
        let mut out = Vec::new();
        for (millis, frame) in records {
            out.extend_from_slice(&(millis * 1000).to_be_bytes());
            out.extend_from_slice(frame);
        }
        out
    }

    /// Reads until the replay has handed over `bytes` more, or gives up after `limit` reads.
    fn read_bytes(replay: &mut ReplayTransport, bytes: usize, limit: usize) -> usize {
        let mut buf = [0u8; 4096];
        let mut got = 0;
        for _ in 0..limit {
            if got >= bytes {
                break;
            }
            got += replay.read(&mut buf).unwrap();
        }
        got
    }

    #[test]
    fn a_tlog_splits_into_the_records_readlogpacketmavlink_reads() {
        let data = tlog(&[
            (1_000, v1(9)),
            (1_100, v2(20, false)),
            (1_200, v2(20, true)),
        ]);
        let found = records(&data);
        assert_eq!(found.len(), 3);
        assert_eq!(
            found[0],
            Record {
                start: 0,
                end: 8 + 17,
                stamp: Some(1_000_000)
            }
        );
        assert_eq!(found[1].start, 25);
        assert_eq!(found[1].end, 25 + 8 + 32);
        // Signed: thirteen more.
        assert_eq!(found[2].end - found[2].start, 8 + 32 + 13);
        assert_eq!(found[2].end, data.len());
    }

    #[test]
    fn a_record_without_a_timestamp_or_with_junk_before_its_frame_is_still_one_record() {
        // A frame straight after a frame: the first "timestamp" byte is a start byte, so the
        // C# rewinds and reads no timestamp.
        let mut data = tlog(&[(5, v1(3))]);
        data.extend_from_slice(&v1(3));
        // A timestamp, three bytes of junk, then the frame: the junk is skipped to the start
        // byte and belongs to that record.
        data.extend_from_slice(&7000u64.to_be_bytes());
        data.extend_from_slice(&[1, 2, 3]);
        data.extend_from_slice(&v1(3));
        let found = records(&data);
        assert_eq!(found.len(), 3);
        assert_eq!(found[1].stamp, None);
        assert_eq!(found[1].end - found[1].start, 11);
        assert_eq!(found[2].stamp, Some(7000));
        assert_eq!(found[2].end - found[2].start, 8 + 3 + 11);
        // Contiguous: every byte is in exactly one record.
        for pair in found.windows(2) {
            assert_eq!(pair[0].end, pair[1].start);
        }
        // An unbelievable timestamp, over 9 999 999 hours, is no timestamp.
        let huge = tlog(&[(u64::MAX / 1000, v1(1))]);
        assert_eq!(records(&huge)[0].stamp, None);
    }

    /// The recorded copter flight: every record is a believable timestamp and a frame, the
    /// records cover the file, and the timestamps never run backwards - so the waits are the
    /// flight's own.
    ///
    /// Not every record is a timestamp straight followed by a frame. The recording also holds
    /// the SITL console's text ("Init ArduCopter...") under timestamps of its own, and the C#
    /// reads such a stretch as it reads junk: one timestamp, then on to the next start byte.
    /// That is what is asserted - a few records skip bytes, and those that do are the text.
    #[test]
    fn a_recorded_flight_splits_into_timestamped_frames() {
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/mavlink/autotest.tlog");
        let Ok(data) = mp_os::fs::read(&path) else {
            eprintln!("skipped: {} is not here", path.display());
            return;
        };
        let found = records(&data);
        assert!(found.len() > 1000, "{} records", found.len());
        assert_eq!(found.first().map(|r| r.start), Some(0));
        assert_eq!(found.last().map(|r| r.end), Some(data.len()));
        let mut last = 0;
        let mut skipping = 0;
        for record in &found {
            let stamp = record.stamp.expect("every record is stamped");
            assert!(
                stamp >= last,
                "a timestamp ran backwards at {}",
                record.start
            );
            last = stamp;
            if !matches!(data[record.start + 8], STX_V1 | STX_V2) {
                skipping += 1;
                // "Init ...", "*" and the like: text after the timestamp.
                let first = data[record.start + 8];
                assert!(
                    first.is_ascii_graphic() || first.is_ascii_whitespace(),
                    "junk that is not text at {}: {first:#04x}",
                    record.start
                );
            }
        }
        assert!(
            skipping * 100 < found.len(),
            "{skipping} of {}",
            found.len()
        );
        // An hour of recording or less: it is a short autotest flight, not a clock gone wrong.
        let span = last - found[0].stamp.unwrap_or(last);
        assert!(span < 3_600_000_000, "{span} us");
    }

    #[test]
    fn the_wait_is_the_csharps_act_over_the_speed_capped_at_a_second() {
        let ms = |millis: u64| Duration::from_millis(millis);
        // 100 ms apart at 1x, 2x, 0.25x.
        assert_eq!(wait_before(Some(0), Some(100_000), 1.0), ms(100));
        assert_eq!(wait_before(Some(0), Some(100_000), 2.0), ms(50));
        assert_eq!(wait_before(Some(0), Some(100_000), 0.25), ms(400));
        // At most a second, however slow.
        assert_eq!(wait_before(Some(0), Some(900_000), 0.1), ms(1000));
        // Over 9999 ms apart, or backwards, is no wait at all: act = 0.
        assert_eq!(wait_before(Some(0), Some(10_000_000), 1.0), Duration::ZERO);
        assert_eq!(wait_before(Some(500_000), Some(0), 1.0), Duration::ZERO);
        // Nothing before it, or no timestamp on it: lastlogread does not move.
        assert_eq!(wait_before(None, Some(100_000), 1.0), Duration::ZERO);
        assert_eq!(wait_before(Some(0), None, 1.0), Duration::ZERO);
        // A speed of 0 is 0.01, as the C# makes it.
        assert_eq!(wait_before(Some(0), Some(1_000), 0.0), ms(100));
    }

    #[test]
    fn a_paced_replay_takes_the_recorded_time_over_the_speed() {
        // Five records 100 ms apart: 400 ms recorded, 40 ms at 10x.
        let frames: Vec<(u64, Vec<u8>)> = (0..5).map(|i| (i * 100, v1(4))).collect();
        let data = tlog(&frames);
        let len = data.len();
        let (mut replay, control) = ReplayTransport::from_bytes("paced", data).paced();
        control.set_speed(10.0);
        assert!(control.is_playing());
        let started = Instant::now();
        assert_eq!(read_bytes(&mut replay, len, 1000), len);
        // Sleeping never ends early, so this side cannot flake.
        assert!(
            started.elapsed() >= Duration::from_millis(40),
            "{:?}",
            started.elapsed()
        );
        assert_eq!(control.position(), len);
        // At the end it stops playing but stays open, so the track bar can take it back.
        assert!(!control.is_playing());
        assert!(replay.is_open());
        assert_eq!(read_bytes(&mut replay, 1, 3), 0);
    }

    #[test]
    fn a_paused_replay_sends_nothing_until_it_is_resumed() {
        let data = tlog(&[(0, v1(4)), (1, v1(4)), (2, v1(4))]);
        let len = data.len();
        let (mut replay, control) = ReplayTransport::from_bytes("paused", data).paced();
        control.set_paused(true);
        assert!(!control.is_playing());
        assert_eq!(read_bytes(&mut replay, 1, 5), 0);
        assert_eq!(control.position(), 0);
        control.set_paused(false);
        assert_eq!(read_bytes(&mut replay, len, 1000), len);
    }

    #[test]
    fn a_seek_moves_to_the_first_record_at_or_after_the_place() {
        let frames: Vec<(u64, Vec<u8>)> = (0..10).map(|i| (i, v1(4))).collect();
        let data = tlog(&frames);
        let record = 8 + 12;
        let (mut replay, control) = ReplayTransport::from_bytes("seek", data).paced();
        // Half of 200 bytes is 100, which is the start of the sixth record.
        control.seek_fraction(0.5);
        assert_eq!(control.position(), 100);
        let mut buf = [0u8; 4096];
        let n = replay.read(&mut buf).unwrap();
        assert_eq!(n, record);
        assert_eq!(buf[8], STX_V1);
        assert_eq!(control.position(), 100 + record);
        // A place inside a record moves on to the next one's start.
        control.seek_fraction(0.51);
        assert_eq!(replay.read(&mut buf).unwrap(), record);
        assert_eq!(control.position(), 120 + record);
    }

    #[test]
    fn an_unpaced_replay_is_as_fast_as_it_ever_was() {
        // Records a second apart: paced at 1x that would be a second each.
        let data = tlog(&[(0, v1(4)), (1_000, v1(4)), (2_000, v1(4))]);
        let len = data.len();
        let mut replay = ReplayTransport::from_bytes("fast", data);
        let started = Instant::now();
        let mut buf = [0u8; 4096];
        let mut got = 0;
        while !replay.is_exhausted() {
            got += replay.read(&mut buf).unwrap();
        }
        assert_eq!(got, len);
        assert!(started.elapsed() < Duration::from_millis(500));
        assert!(!replay.is_open());
    }

    /// Reads one read's worth and returns its length with the clock the replay then reports.
    fn read_with_time(replay: &mut ReplayTransport, buf: &mut [u8]) -> (usize, crate::ReadTime) {
        let n = replay.read(buf).unwrap();
        (n, replay.read_time())
    }

    #[test]
    fn a_read_ends_with_its_record_and_the_clock_is_the_newest_usable_stamp() {
        use crate::ReadTime::Recorded;
        // Two records, then one without a timestamp - a frame straight after a frame - then one
        // with an unbelievable timestamp: the last two leave the clock where the second set it,
        // as `lastlogread` is set only by a usable stamp (MAVLinkInterface.cs:6545-6557).
        let mut data = tlog(&[(1_000, v1(9)), (1_100, v2(20, false))]);
        data.extend_from_slice(&v1(3));
        data.extend_from_slice(&(u64::MAX / 2).to_be_bytes());
        data.extend_from_slice(&v1(3));
        let mut replay = ReplayTransport::from_bytes("clock", data);
        // Before anything is read, `lastlogread` is still MinValue.
        assert_eq!(replay.read_time(), Recorded(None));
        let mut buf = [0u8; 4096];
        assert_eq!(
            read_with_time(&mut replay, &mut buf),
            (8 + 17, Recorded(Some(1_000_000)))
        );
        assert_eq!(
            read_with_time(&mut replay, &mut buf),
            (8 + 32, Recorded(Some(1_100_000)))
        );
        assert_eq!(
            read_with_time(&mut replay, &mut buf),
            (11, Recorded(Some(1_100_000)))
        );
        assert_eq!(
            read_with_time(&mut replay, &mut buf),
            (8 + 11, Recorded(Some(1_100_000)))
        );
        assert_eq!(read_with_time(&mut replay, &mut buf).0, 0);

        // A read smaller than a record keeps the record's time for every piece of it, and never
        // takes bytes of the next record with the last piece.
        let data = tlog(&[(5, v1(9)), (6, v1(9))]);
        let mut replay = ReplayTransport::from_bytes("pieces", data).with_chunk_size(10);
        let pieces: Vec<_> = (0..6)
            .map(|_| read_with_time(&mut replay, &mut buf))
            .collect();
        assert_eq!(
            pieces,
            [
                (10, Recorded(Some(5_000))),
                (10, Recorded(Some(5_000))),
                (5, Recorded(Some(5_000))),
                (10, Recorded(Some(6_000))),
                (10, Recorded(Some(6_000))),
                (5, Recorded(Some(6_000))),
            ]
        );
    }

    #[test]
    fn a_paced_replay_reports_each_records_stamp_and_a_seek_forgets_it() {
        use crate::ReadTime::Recorded;
        let frames: Vec<(u64, Vec<u8>)> = (0..10).map(|i| (i, v1(4))).collect();
        let (mut replay, control) = ReplayTransport::from_bytes("paced", tlog(&frames)).paced();
        control.set_speed(1000.0);
        let mut buf = [0u8; 4096];
        assert_eq!(
            read_with_time(&mut replay, &mut buf),
            (20, Recorded(Some(0)))
        );
        assert_eq!(
            read_with_time(&mut replay, &mut buf),
            (20, Recorded(Some(1_000)))
        );
        // `tracklog_Scroll` sets `lastlogread` to MinValue (FlightData.cs:5367); the record
        // landed on sets it again.
        control.seek_fraction(0.5);
        assert_eq!(
            read_with_time(&mut replay, &mut buf),
            (20, Recorded(Some(5_000)))
        );
    }

    #[test]
    fn a_live_transport_is_read_at_the_time_it_is_read() {
        let (mut a, _b) = crate::testing::Loopback::pair();
        let mut buf = [0u8; 16];
        let _ = a.read(&mut buf);
        assert_eq!(a.read_time(), crate::ReadTime::Live);
    }
}
