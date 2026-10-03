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

//! One MAVFTP command each: what it sends, what it makes of each reply, and when it is done.
//!
//! Every `kCmd*` method in `MAVFtp.cs` has the same shape: build a payload, subscribe a handler to
//! `FILE_TRANSFER_PROTOCOL` from the vehicle, and run `RetryTimeout.DoWork` with "send the payload"
//! as the work. Here that is a [`Step`]: [`Step::begin`] is the first pass of `DoWork`,
//! [`Step::on_reply`] is the handler and [`Step::on_tick`] is `DoWork`'s wait running out. What
//! the method returns, or throws, is [`StepResult`].
//!
//! Two things every handler shares, and both are kept:
//!
//! * A NAK is taken without looking at its sequence number or which request it answers. A stale
//!   NAK - the vehicle's answer to a request an earlier command gave up on - lands on whichever
//!   command is running now.
//! * An ACK is taken only if it answers this command's opcode with the sequence number after the
//!   request's, compared as `payload.seq_number + 1 != ftphead.seq_number` in `int` arithmetic
//!   (e.g. MAVFtp.cs:664). At sequence number 65535 the request's `+ 1` is 65536 and the reply's
//!   wrapped 0 can never match it, so every 65,536th command times out; that is kept too.

use std::time::Instant;

use super::Progress;
use super::listing::{FtpFileInfo, parse_entries};
use super::retry::{Expiry, Patience, RetryTimeout};
use super::wire::{Errno, ErrorCode, Header, Opcode};
use crate::FtpError;

/// `rwSize`: the largest read or write Mission Planner uses, "low to keep some radios happy".
///
/// C#: MAVFtp.cs:32.
pub const RW_SIZE: u8 = 80;

/// What a step touches that outlives it.
pub(crate) struct Ctx<'a> {
    /// The client's `seq_no`, shared by every command it sends.
    pub seq_no: &'a mut u16,
    /// Payloads to put on the wire, in order.
    pub out: &'a mut Vec<Header>,
    /// `cancel.IsCancellationRequested`.
    pub cancelled: bool,
    /// The last `Progress` event.
    pub progress: &'a mut Progress,
}

impl Ctx<'_> {
    /// `seq_no++`.
    pub(crate) fn next_seq(&mut self) -> u16 {
        let seq = *self.seq_no;
        *self.seq_no = seq.wrapping_add(1);
        seq
    }

    fn send(&mut self, request: &Header) {
        self.out.push(request.clone());
    }

    /// `Progress?.Invoke(message, percent)`.
    fn progress(&mut self, message: String, percent: i32) {
        *self.progress = Progress { message, percent };
    }
}

/// Whether an ACK answers `request`: its opcode, and the sequence number one past the request's.
///
/// C#: e.g. MAVFtp.cs:664, `payload.opcode != ftphead.req_opcode || payload.seq_number + 1 !=
/// ftphead.seq_number` - `int` arithmetic, so 65535 + 1 is 65536 and never matches.
fn answers(request: &Header, reply: &Header) -> bool {
    request.opcode == reply.req_opcode
        && u32::from(request.seq_number) + 1 == u32::from(reply.seq_number)
}

/// `(int)((float)part / whole * 100.0)`, the C#'s progress arithmetic.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
fn percent(part: i64, whole: i64) -> i32 {
    (f64::from(part as f32 / whole as f32) * 100.0) as i32
}

/// Where a step is after an event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Status {
    /// Still waiting.
    Running,
    /// `DoWork` has returned.
    Finished,
    /// The handler called `kCmdResetSessions()` and is waiting for it (see [`Simple::after_reset`]).
    ResetSessions,
}

/// The two wordings the C# gives a `kErrFailErrno` exception.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Wording {
    /// `"Mavftp responded - "`.
    Responded,
    /// `"Failed to OpenFile - "`, which several commands that open no file use too.
    FailedToOpenFile,
}

impl Wording {
    fn error(self, head: &Header) -> FtpError {
        let req_opcode = head.req_opcode;
        let error = ErrorCode(head.data_byte(0));
        let errno = Errno(head.data_byte(1));
        match self {
            Self::Responded => FtpError::Responded {
                req_opcode,
                error,
                errno,
            },
            Self::FailedToOpenFile => FtpError::FailedToOpenFile {
                req_opcode,
                error,
                errno,
            },
        }
    }
}

/// What a command does with `kErrNoSessionsAvailable`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum OnNoSessions {
    /// Nothing.
    Ignore,
    /// `kCmdResetSessions();`.
    Reset,
    /// `kCmdResetSessions(); timeout.RetriesCurrent = 0;`.
    ResetAndRetry,
}

/// How one of the one-request commands treats a NAK. Each field is a branch some commands have
/// and others do not; the table is in [`Simple::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Rules {
    /// The handler and `WorkToDo` look at the cancellation token.
    pub cancellable: bool,
    /// How a `kErrFailErrno` exception is worded.
    pub wording: Wording,
    /// `kErrFail`: `timeout.Retries = 0; ex = new Exception("Mavftp responded - Err Fail")`.
    pub fail_stops: bool,
    /// `kErrFileNotFound`: `timeout.Retries = 0; ex = new FileNotFoundException(...)`.
    pub not_found_stops: bool,
    /// `kErrFailFileExists`, or `kErrFailErrno` with `EEXIST`: `timeout.Complete = true`.
    pub exists_completes: bool,
    /// `kErrNoSessionsAvailable`.
    pub no_sessions: OnNoSessions,
}

/// A command that is one request and one answer.
#[derive(Debug, Clone)]
pub(crate) struct Simple {
    request: Header,
    /// The path, for a `FileNotFoundException`.
    path: String,
    rules: Rules,
    retry: RetryTimeout,
    /// `ex`: what the method throws once `DoWork` returns. The last NAK's, if several.
    ex: Option<FtpError>,
    /// `localsize` / `localcrc32`: the ACK's first four bytes, once it came.
    value: Option<u32>,
}

impl Simple {
    /// The command `opcode` on `path`, with its NAK rules and patience from the C#.
    ///
    /// | command | C# | cancel | errno wording | Fail | FileNotFound | exists | NoSessions |
    /// |---|---|---|---|---|---|---|---|
    /// | `kCmdResetSessions` | :1908-1961 | no | responded | - | - | - | - |
    /// | `kCmdTerminateSession` | :1963-2016 | no | responded | - | - | - | - |
    /// | `kCmdOpenFileRO` | :594-692 | yes | responded | stops | stops | - | reset, retry |
    /// | `kCmdCreateFile` | :1139-1236 | yes | failed to open | stops | stops | - | reset, retry |
    /// | `kCmdCalcFileCRC32` | :914-1000 | yes | failed to open | - | stops | - | reset |
    /// | `kCmdCreateDirectory` | :1046-1137 | yes | responded | stops | - | completes | reset |
    /// | `kCmdRemoveDirectory` | :1661-1744 | yes | failed to open | - | stops | - | reset |
    /// | `kCmdRemoveFile` | :1746-1829 | yes | failed to open | - | stops | - | reset |
    /// | `kCmdRename` | :1831-1906 | yes | responded | - | - | - | reset |
    pub(crate) fn new(
        opcode: Opcode,
        data: &[u8],
        path: &str,
        timeouts: &super::FtpTimeouts,
        ctx: &mut Ctx<'_>,
        now: Instant,
    ) -> Self {
        let (rules, patience) = rules_for(opcode, timeouts);
        let mut request = Header {
            opcode,
            seq_number: ctx.next_seq(),
            session: 0,
            ..Header::default()
        };
        // Reset and terminate carry no data; the others carry the path, which sets the size.
        if !matches!(opcode, Opcode::RESET_SESSIONS | Opcode::TERMINATE_SESSION) {
            request.set_data(data);
        }
        Self {
            request,
            path: path.to_owned(),
            rules,
            retry: RetryTimeout::new(patience, now),
            ex: None,
            value: None,
        }
    }

    /// `timeout.WorkToDo`.
    fn work(&mut self, ctx: &mut Ctx<'_>) {
        if self.rules.cancellable && ctx.cancelled {
            self.retry.cancel();
            return;
        }
        ctx.send(&self.request);
    }

    fn begin(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if !self.retry.begin() {
            return Status::Finished;
        }
        self.work(ctx);
        self.retry.arm(now);
        Status::Running
    }

    fn on_reply(&mut self, head: &Header, ctx: &mut Ctx<'_>) -> Status {
        if self.rules.cancellable && ctx.cancelled {
            self.retry.cancel();
            return Status::Running;
        }
        if head.opcode == Opcode::NAK {
            let error = ErrorCode(head.data_byte(0));
            if error == ErrorCode::FAIL_ERRNO {
                if self.rules.exists_completes && Errno(head.data_byte(1)) == Errno::EEXIST {
                    // C#: MAVFtp.cs:1082-1083. Complete, and then the exception below is thrown
                    // anyway: a directory that already exists is an error after all.
                    self.retry.complete = true;
                }
                self.retry.retries = 0;
                self.ex = Some(self.rules.wording.error(head));
            }
            if self.rules.exists_completes && error == ErrorCode::FAIL_FILE_EXISTS {
                self.retry.complete = true;
            }
            if self.rules.fail_stops && error == ErrorCode::FAIL {
                self.retry.retries = 0;
                self.ex = Some(FtpError::ErrFail);
            }
            if self.rules.not_found_stops && error == ErrorCode::FILE_NOT_FOUND {
                self.retry.retries = 0;
                self.ex = Some(FtpError::FileNotFound {
                    path: self.path.clone(),
                });
            }
            if error == ErrorCode::NO_SESSIONS_AVAILABLE
                && self.rules.no_sessions != OnNoSessions::Ignore
            {
                return Status::ResetSessions;
            }
            return self.status();
        }
        if !answers(&self.request, head) || head.opcode != Opcode::ACK {
            return Status::Running;
        }
        self.value = Some(head.data_u32());
        self.retry.complete = true;
        Status::Finished
    }

    /// After the nested `kCmdResetSessions()` returns: `timeout.RetriesCurrent = 0` for the
    /// commands that have it. Not if the reset threw, because the exception leaves the handler
    /// before that line (and `PacketReceived` swallows it, MAVLinkInterface.cs:5543-5550).
    fn after_reset(&mut self, threw: bool) {
        if self.rules.no_sessions == OnNoSessions::ResetAndRetry && !threw {
            self.retry.retries_current = 0;
        }
    }

    fn on_tick(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if self.retry.complete {
            return Status::Finished;
        }
        match self.retry.expire(now) {
            Expiry::Waiting => Status::Running,
            Expiry::Again => {
                self.work(ctx);
                self.retry.arm(now);
                Status::Running
            }
            Expiry::GaveUp => Status::Finished,
        }
    }

    fn status(&self) -> Status {
        if self.retry.complete {
            Status::Finished
        } else {
            Status::Running
        }
    }
}

/// C#: the `new RetryTimeout(...)` and the NAK branches of each one-request command; see the
/// table on [`Simple::new`].
fn rules_for(opcode: Opcode, t: &super::FtpTimeouts) -> (Rules, Patience) {
    let rules = |cancellable, wording, fail_stops, not_found_stops, exists, no_sessions| Rules {
        cancellable,
        wording,
        fail_stops,
        not_found_stops,
        exists_completes: exists,
        no_sessions,
    };
    use OnNoSessions::{Ignore, Reset, ResetAndRetry};
    use Wording::{FailedToOpenFile, Responded};
    match opcode {
        Opcode::RESET_SESSIONS => (
            rules(false, Responded, false, false, false, Ignore),
            t.reset_sessions,
        ),
        Opcode::TERMINATE_SESSION => (
            rules(false, Responded, false, false, false, Ignore),
            t.other,
        ),
        Opcode::OPEN_FILE_RO => (
            rules(true, Responded, true, true, false, ResetAndRetry),
            t.open_file_ro,
        ),
        Opcode::CREATE_FILE => (
            rules(true, FailedToOpenFile, true, true, false, ResetAndRetry),
            t.other,
        ),
        Opcode::CALC_FILE_CRC32 => (
            rules(true, FailedToOpenFile, false, true, false, Reset),
            t.calc_file_crc32,
        ),
        Opcode::CREATE_DIRECTORY => (rules(true, Responded, true, false, true, Reset), t.other),
        Opcode::REMOVE_DIRECTORY | Opcode::REMOVE_FILE => (
            rules(true, FailedToOpenFile, false, true, false, Reset),
            t.other,
        ),
        // kCmdRename, and anything else sent this way.
        _ => (rules(true, Responded, false, false, false, Reset), t.other),
    }
}

/// `kCmdListDirectory`, with or without times.
///
/// C#: MAVFtp.cs:1264-1446.
#[derive(Debug, Clone)]
pub(crate) struct List {
    request: Header,
    /// The directory as asked for, trailing `/` trimmed.
    dir: String,
    with_time: bool,
    retry: RetryTimeout,
    answer: Vec<FtpFileInfo>,
    /// `notsupported`: the vehicle does not know `kCmdListDirectoryWithTime`.
    not_supported: bool,
    ex: Option<FtpError>,
}

impl List {
    pub(crate) fn new(
        dir: &str,
        with_time: bool,
        timeouts: &super::FtpTimeouts,
        ctx: &mut Ctx<'_>,
        now: Instant,
    ) -> Self {
        // C#: MAVFtp.cs:1269-1270, `if (dir.Length > 1) dir = dir.TrimEnd('/');`. Length in
        // UTF-16 units, but only "/" itself is short enough to matter.
        let dir = if dir.chars().count() > 1 {
            dir.trim_end_matches('/')
        } else {
            dir
        };
        let mut request = Header {
            opcode: if with_time {
                Opcode::LIST_DIRECTORY_WITH_TIME
            } else {
                Opcode::LIST_DIRECTORY
            },
            seq_number: ctx.next_seq(),
            offset: 0,
            ..Header::default()
        };
        request.set_data(dir.as_bytes());
        // C#: MAVFtp.cs:1285.
        ctx.progress(format!("{dir} Listing"), 0);
        Self {
            request,
            dir: dir.to_owned(),
            with_time,
            retry: RetryTimeout::new(timeouts.list_directory, now),
            answer: Vec::new(),
            not_supported: false,
            ex: None,
        }
    }

    fn work(&mut self, ctx: &mut Ctx<'_>) {
        if ctx.cancelled {
            self.retry.cancel();
            return;
        }
        ctx.send(&self.request);
    }

    fn begin(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if !self.retry.begin() {
            return Status::Finished;
        }
        self.work(ctx);
        self.retry.arm(now);
        Status::Running
    }

    fn on_reply(&mut self, head: &Header, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if ctx.cancelled {
            self.retry.cancel();
            return Status::Running;
        }
        if head.opcode == Opcode::NAK {
            let error = ErrorCode(head.data_byte(0));
            // C#: MAVFtp.cs:1306-1317. Only the very first reply can mean "unknown opcode"; a NAK
            // part way through a listing is a real error.
            if self.with_time
                && head.req_opcode == Opcode::LIST_DIRECTORY_WITH_TIME
                && self.answer.is_empty()
                && (error == ErrorCode::UNKNOWN_COMMAND || error == ErrorCode::FAIL)
            {
                self.not_supported = true;
                self.retry.complete = true;
                return Status::Finished;
            }
            if error == ErrorCode::FAIL_ERRNO {
                self.retry.retries = 0;
                self.ex = Some(Wording::FailedToOpenFile.error(head));
            }
            if error == ErrorCode::FILE_NOT_FOUND {
                self.retry.retries = 0;
                self.ex = Some(FtpError::FileNotFound {
                    path: self.dir.clone(),
                });
            }
            // C#: MAVFtp.cs:1339-1340. The listing's normal end: the offset past the last entry.
            if error == ErrorCode::EOF {
                self.retry.complete = true;
                return Status::Finished;
            }
            return Status::Running;
        }
        if !answers(&self.request, head) || head.opcode != Opcode::ACK {
            return Status::Running;
        }
        if parse_entries(head, &self.dir, self.with_time, &mut self.answer).is_err() {
            return Status::Running;
        }
        // C#: MAVFtp.cs:1413-1415, "0 records". Complete - and then the next request goes out
        // below anyway, as it does in the C#.
        if self.answer.is_empty() {
            self.retry.complete = true;
        }
        let count = self.answer.len();
        let count_i32 = i32::try_from(count).unwrap_or(i32::MAX);
        ctx.progress(format!("{} {count}", self.dir), count_i32 % 100);
        // C#: MAVFtp.cs:1419-1424. The next offset is how many entries have arrived, skips
        // included.
        self.request.offset = u32::try_from(count).unwrap_or(u32::MAX);
        self.request.seq_number = ctx.next_seq();
        ctx.send(&self.request);
        self.retry.arm(now);
        self.retry.retries_current = 0;
        if self.retry.complete {
            Status::Finished
        } else {
            Status::Running
        }
    }

    fn on_tick(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if self.retry.complete {
            return Status::Finished;
        }
        match self.retry.expire(now) {
            Expiry::Waiting => Status::Running,
            Expiry::Again => {
                self.work(ctx);
                self.retry.arm(now);
                Status::Running
            }
            Expiry::GaveUp => Status::Finished,
        }
    }
}

/// `kCmdBurstReadFile`: the vehicle streams the file, and holes are asked for one read at a time.
///
/// C#: MAVFtp.cs:694-870.
///
/// # How it decides the file is whole
///
/// Chunks are written at their offset, and their extents kept in a sorted list that merges
/// neighbours (`SimplifyChunkList`, :872-893). The file is whole when the extents add up to the
/// size - and "the size" is not fixed: it starts as `kCmdOpenFileRO`'s answer and is cut to the
/// end of any chunk shorter than `readsize` (:785-789), which is how a virtual file such as
/// `@SYS/threads.txt`, that opens claiming whatever size, ends where its data does.
///
/// A burst that reaches the end with holes switches to `kCmdReadFile` for the first hole
/// (`FindMissing`, :895-912), one request per hole, each answer triggering the next. A NAK saying
/// end of file does the same, or finishes if nothing is missing.
///
/// Once switched to reads, a burst chunk still in flight is ignored: its `req_opcode` is no
/// longer the one being asked (:777).
///
/// `DoWork` giving up does not make this fail: the C# returns whatever has arrived (:862-869),
/// holes zero-filled, and so does this, with [`BurstResult::ended`] false to say so.
#[derive(Debug, Clone)]
pub(crate) struct Burst {
    request: Header,
    file: String,
    readsize: u8,
    /// `size`, which a short chunk cuts.
    size: i64,
    /// `chunkSortedList`: `(offset, end)` by offset.
    chunks: Vec<(u32, u32)>,
    /// `answer`.
    answer: Vec<u8>,
    retry: RetryTimeout,
    ex: Option<FtpError>,
}

impl Burst {
    pub(crate) fn new(
        file: &str,
        size: i32,
        readsize: u8,
        timeouts: &super::FtpTimeouts,
        ctx: &mut Ctx<'_>,
        now: Instant,
    ) -> Result<Self, FtpError> {
        let request = Header {
            opcode: Opcode::BURST_READ_FILE,
            seq_number: ctx.next_seq(),
            session: 0,
            offset: 0,
            size: readsize,
            ..Header::default()
        };
        ctx.progress(file.to_owned(), 0);
        // C#: MAVFtp.cs:713, `new MemoryStream(size)`, which throws for a negative size.
        if size < 0 {
            return Err(FtpError::NegativeSize(size));
        }
        Ok(Self {
            request,
            file: file.to_owned(),
            readsize,
            size: i64::from(size),
            chunks: Vec::new(),
            answer: Vec::new(),
            retry: RetryTimeout::new(timeouts.other, now),
            ex: None,
        })
    }

    fn work(&mut self, ctx: &mut Ctx<'_>) {
        if ctx.cancelled {
            self.retry.cancel();
            return;
        }
        ctx.send(&self.request);
    }

    fn begin(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if !self.retry.begin() {
            return Status::Finished;
        }
        self.work(ctx);
        self.retry.arm(now);
        Status::Running
    }

    /// `chunkSortedList.Sum(a => a.Value - a.Key)`.
    fn received(&self) -> i64 {
        self.chunks
            .iter()
            .map(|(start, end)| i64::from(end.wrapping_sub(*start)))
            .sum()
    }

    /// Switches to `kCmdReadFile` for the first hole and asks for it.
    ///
    /// C#: MAVFtp.cs:760-769 and :829-839.
    fn read_missing(&mut self, missing: u32, reply_seq: u16, ctx: &mut Ctx<'_>) {
        self.request.opcode = Opcode::READ_FILE;
        self.request.offset = missing;
        *ctx.seq_no = reply_seq.wrapping_add(1);
        self.request.seq_number = *ctx.seq_no;
        self.retry.retries_current = 0;
        ctx.send(&self.request);
    }

    fn on_reply(&mut self, head: &Header, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if ctx.cancelled {
            self.retry.cancel();
            return Status::Running;
        }
        if head.opcode == Opcode::NAK {
            // C#: MAVFtp.cs:729-772.
            *ctx.seq_no = head.seq_number.wrapping_add(1);
            let error = ErrorCode(head.data_byte(0));
            if error == ErrorCode::FAIL_ERRNO {
                self.retry.retries = 0;
                self.ex = Some(Wording::Responded.error(head));
            }
            if error == ErrorCode::EOF {
                if self.received() >= self.size {
                    self.retry.complete = true;
                    return Status::Finished;
                }
                let missing = find_missing(&self.chunks);
                if missing == u32::MAX {
                    self.retry.complete = true;
                    return Status::Finished;
                }
                self.read_missing(missing, head.seq_number, ctx);
            }
            return Status::Running;
        }
        // C#: MAVFtp.cs:776-780. Many replies answer one request, so the sequence number is not
        // checked here.
        if self.request.opcode != head.req_opcode || head.opcode != Opcode::ACK {
            return Status::Running;
        }
        let offset = i64::from(head.offset);
        let len = i64::from(head.size);
        let size = self.size;
        // C#: MAVFtp.cs:781-784, reject bad packets.
        if offset > size
            || len > size
            || offset + len > size
            || (self.answer.is_empty() && offset > 0 && size < i64::from(self.readsize))
        {
            return Status::Running;
        }
        // C#: MAVFtp.cs:785-789, a short chunk is the end of the file.
        if head.size < self.readsize {
            self.size = offset + len;
        }
        // C#: :790-794 only logs a chunk arriving somewhere other than the stream's position.
        self.retry.retries_current = 0;
        self.retry.arm(now);

        let end = head.offset.wrapping_add(u32::from(head.size));
        match self
            .chunks
            .binary_search_by_key(&head.offset, |(start, _)| *start)
        {
            Ok(at) => {
                if let Some(chunk) = self.chunks.get_mut(at) {
                    chunk.1 = end;
                }
            }
            Err(at) => self.chunks.insert(at, (head.offset, end)),
        }
        simplify_chunk_list(&mut self.chunks);
        let current = self.received();

        // C#: MAVFtp.cs:808-809. A size past the 239 data bytes makes `Write` throw here, after
        // the chunk was recorded; PacketReceived swallows it.
        let Some(data) = head.data.get(..usize::from(head.size)) else {
            return Status::Running;
        };
        let Ok(start) = usize::try_from(head.offset) else {
            return Status::Running;
        };
        let stop = start + data.len();
        if self.answer.len() < stop {
            self.answer.resize(stop, 0);
        }
        if let Some(slot) = self.answer.get_mut(start..stop) {
            slot.copy_from_slice(data);
        }
        self.retry.arm(now);

        *ctx.seq_no = head.seq_number.wrapping_add(1);
        // C#: MAVFtp.cs:813-816, "dont move backwards".
        self.request.offset = end.max(self.request.offset);
        self.request.seq_number = *ctx.seq_no;
        if head.size > 0 {
            ctx.progress(self.file.clone(), percent(current, self.size));
        }
        if current >= self.size {
            self.retry.complete = true;
            return Status::Finished;
        }
        // C#: MAVFtp.cs:826-840. The end is in sight with holes behind it, or already reading
        // holes: ask for the next one.
        if offset + len >= self.size || self.request.opcode == Opcode::READ_FILE {
            let missing = find_missing(&self.chunks);
            self.read_missing(missing, head.seq_number, ctx);
            return Status::Running;
        }
        // C#: MAVFtp.cs:843-848. The vehicle has sent the burst it was asked for; ask for the next.
        if head.burst_complete == 1 {
            ctx.send(&self.request);
        }
        Status::Running
    }

    fn on_tick(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if self.retry.complete {
            return Status::Finished;
        }
        match self.retry.expire(now) {
            Expiry::Waiting => Status::Running,
            Expiry::Again => {
                self.work(ctx);
                self.retry.arm(now);
                Status::Running
            }
            Expiry::GaveUp => Status::Finished,
        }
    }
}

/// `SimplifyChunkList`: join chunks that touch or overlap.
///
/// C#: MAVFtp.cs:872-893, index for index. Joining takes the *next* chunk's end, even when the
/// next chunk ends inside this one - so a chunk read again after a merge can shorten the extent
/// that covers it. The data is still in the buffer; the file then looks short, the read asks for
/// what it thinks is missing, and the vehicle's answer (or its end of file) puts it right.
pub(crate) fn simplify_chunk_list(chunks: &mut Vec<(u32, u32)>) {
    let mut i = 0usize;
    while i + 1 < chunks.len() {
        let (Some(&(start, end)), Some(&(next_start, next_end))) =
            (chunks.get(i), chunks.get(i + 1))
        else {
            break;
        };
        if end >= next_start {
            if let Some(chunk) = chunks.get_mut(i) {
                *chunk = (start, next_end);
            }
            chunks.remove(i + 1);
            // i--, then the loop's i++: look at this chunk again.
            continue;
        } else if next_start >= start && next_end <= end {
            chunks.remove(i + 1);
            continue;
        }
        i += 1;
    }
}

/// `FindMissing`: where the first hole starts, or `u32::MAX` if the chunks run unbroken from zero.
///
/// C#: MAVFtp.cs:895-912. Unbroken from zero is not the same as whole - a file missing only its
/// tail has no hole by this measure - so the callers check the size first.
pub(crate) fn find_missing(chunks: &[(u32, u32)]) -> u32 {
    let mut current = 0u32;
    for (start, end) in chunks {
        if *start == current {
            current = *end;
        } else {
            return current;
        }
    }
    u32::MAX
}

/// `kCmdReadFile`: one request, one chunk, in order.
///
/// C#: MAVFtp.cs:1547-1659. What Mission Planner's plain Download and its serial-port names use
/// (Controls/MavFTPUI.cs:353, GCSViews/ConfigurationView/ConfigSerial.cs:105).
#[derive(Debug, Clone)]
pub(crate) struct Read {
    request: Header,
    file: String,
    size: i64,
    answer: Vec<u8>,
    /// `answer.Position`.
    position: u64,
    retry: RetryTimeout,
    ex: Option<FtpError>,
}

impl Read {
    pub(crate) fn new(
        file: &str,
        size: i32,
        readsize: u8,
        timeouts: &super::FtpTimeouts,
        ctx: &mut Ctx<'_>,
        now: Instant,
    ) -> Result<Self, FtpError> {
        let request = Header {
            opcode: Opcode::READ_FILE,
            seq_number: ctx.next_seq(),
            offset: 0,
            session: 0,
            size: readsize,
            ..Header::default()
        };
        ctx.progress(file.to_owned(), 0);
        // C#: MAVFtp.cs:1562, `new MemoryStream(size)`.
        if size < 0 {
            return Err(FtpError::NegativeSize(size));
        }
        Ok(Self {
            request,
            file: file.to_owned(),
            size: i64::from(size),
            answer: Vec::new(),
            position: 0,
            retry: RetryTimeout::new(timeouts.other, now),
            ex: None,
        })
    }

    fn work(&mut self, ctx: &mut Ctx<'_>) {
        if ctx.cancelled {
            self.retry.cancel();
            return;
        }
        ctx.send(&self.request);
    }

    fn begin(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if !self.retry.begin() {
            return Status::Finished;
        }
        self.work(ctx);
        self.retry.arm(now);
        Status::Running
    }

    fn on_reply(&mut self, head: &Header, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if ctx.cancelled {
            self.retry.cancel();
            return Status::Running;
        }
        if head.opcode == Opcode::NAK {
            // C#: MAVFtp.cs:1576-1603.
            let error = ErrorCode(head.data_byte(0));
            if error == ErrorCode::FAIL_ERRNO {
                self.retry.retries = 0;
                self.ex = Some(Wording::Responded.error(head));
            }
            if error == ErrorCode::FAIL {
                self.retry.retries = 0;
                self.ex = Some(FtpError::ErrFail);
            }
            if error == ErrorCode::EOF {
                self.retry.complete = true;
                return Status::Finished;
            }
            return Status::Running;
        }
        if !answers(&self.request, head) || head.opcode != Opcode::ACK {
            return Status::Running;
        }
        // C#: MAVFtp.cs:1612-1617, "we have lost data - use retry after timeout".
        if self.position != u64::from(head.offset) {
            self.retry.retries_current = 0;
            return Status::Running;
        }
        self.retry.retries_current = 0;
        self.retry.arm(now);
        let Some(data) = head.data.get(..usize::from(head.size)) else {
            // `Write` throws for a size past the 239 data bytes; PacketReceived swallows it.
            return Status::Running;
        };
        let Ok(start) = usize::try_from(head.offset) else {
            return Status::Running;
        };
        let stop = start + data.len();
        if self.answer.len() < stop {
            self.answer.resize(stop, 0);
        }
        if let Some(slot) = self.answer.get_mut(start..stop) {
            slot.copy_from_slice(data);
        }
        self.position = u64::try_from(stop).unwrap_or(u64::MAX);
        // C#: MAVFtp.cs:1625, the percentage of the request's offset, before it moves on.
        ctx.progress(
            self.file.clone(),
            percent(i64::from(self.request.offset), self.size),
        );
        let end = i64::from(head.offset) + i64::from(head.size);
        if end >= self.size {
            self.retry.complete = true;
            return Status::Finished;
        }
        self.request.offset = head.offset.wrapping_add(u32::from(head.size));
        self.request.seq_number = ctx.next_seq();
        ctx.send(&self.request);
        Status::Running
    }

    fn on_tick(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if self.retry.complete {
            return Status::Finished;
        }
        match self.retry.expire(now) {
            Expiry::Waiting => Status::Running,
            Expiry::Again => {
                self.work(ctx);
                self.retry.arm(now);
                Status::Running
            }
            Expiry::GaveUp => Status::Finished,
        }
    }
}

/// `kCmdWriteFile(Stream, ...)`: the upload half of `UploadFile`.
///
/// C#: MAVFtp.cs:2213-2361. The first chunk goes alone; each ACK that answers the latest send
/// releases the next five chunks not yet acknowledged, lowest offset first; ACKs for anything
/// else only cross their chunk off. The upload is done when the latest send's ACK is for the
/// last chunk - which does not check that every earlier chunk was acknowledged. Mission Planner's
/// upload page finds out afterwards, by comparing CRCs (Controls/MavFTPUI.cs:437-442).
#[derive(Debug, Clone)]
pub(crate) struct Write {
    request: Header,
    data: Vec<u8>,
    /// `sendlist`: offsets not yet acknowledged, keyed as `(int)offset`.
    unacknowledged: std::collections::BTreeSet<i64>,
    /// `payload.data.Length`: how much the next `stream.Read` asks for. It starts at `rwSize`
    /// and shrinks to whatever the last read returned (`Array.Resize`, :2320, :2338).
    read_len: usize,
    retry: RetryTimeout,
    ex: Option<FtpError>,
    friendly: String,
}

impl Write {
    pub(crate) fn new(
        friendly: &str,
        data: Vec<u8>,
        timeouts: &super::FtpTimeouts,
        ctx: &mut Ctx<'_>,
        now: Instant,
    ) -> Self {
        let request = Header {
            opcode: Opcode::WRITE_FILE,
            seq_number: ctx.next_seq(),
            offset: 0,
            session: 0,
            ..Header::default()
        };
        // C#: MAVFtp.cs:2234-2236.
        let step = usize::from(RW_SIZE);
        let unacknowledged = (0..data.len())
            .step_by(step)
            .filter_map(|offset| i64::try_from(offset).ok())
            .collect();
        Self {
            request,
            data,
            unacknowledged,
            read_len: step,
            retry: RetryTimeout::new(timeouts.other, now),
            ex: None,
            friendly: friendly.to_owned(),
        }
    }

    /// `stream.Position = offset; bytes_read = stream.Read(payload.data, 0, payload.data.Length);
    /// Array.Resize(ref payload.data, bytes_read);`, and the payload's size with it.
    fn read_chunk(&mut self, offset: usize) -> usize {
        let available = self.data.len().saturating_sub(offset);
        let n = self.read_len.min(available);
        if n == 0 {
            return 0;
        }
        self.read_len = n;
        let chunk = self.data.get(offset..offset + n).unwrap_or(&[]).to_vec();
        self.request.set_data(&chunk);
        self.request.offset = u32::try_from(offset).unwrap_or(u32::MAX);
        n
    }

    fn work(&mut self, ctx: &mut Ctx<'_>) {
        if ctx.cancelled {
            self.retry.cancel();
            return;
        }
        ctx.send(&self.request);
    }

    fn begin(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        // C#: MAVFtp.cs:2230-2231, an empty file writes nothing and returns false.
        if self.data.is_empty() {
            return Status::Finished;
        }
        // C#: MAVFtp.cs:2335-2339, fill the first buffer.
        self.read_chunk(0);
        if !self.retry.begin() {
            return Status::Finished;
        }
        self.work(ctx);
        self.retry.arm(now);
        Status::Running
    }

    fn on_reply(&mut self, head: &Header, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if ctx.cancelled {
            self.retry.cancel();
            return Status::Running;
        }
        if head.opcode == Opcode::NAK {
            // C#: MAVFtp.cs:2259-2284.
            let error = ErrorCode(head.data_byte(0));
            if error == ErrorCode::FAIL_ERRNO {
                self.retry.retries = 0;
                self.ex = Some(Wording::Responded.error(head));
            }
            if error == ErrorCode::FAIL {
                self.retry.retries = 0;
                self.ex = Some(FtpError::ErrFail);
            }
            return Status::Running;
        }
        if self.request.opcode != head.req_opcode || head.opcode != Opcode::ACK {
            return Status::Running;
        }
        // C#: MAVFtp.cs:2296, `sendlist.Remove((int)ftphead.offset)`; the cast wraps.
        let key = i64::from(i32::from_ne_bytes(head.offset.to_ne_bytes()));
        self.unacknowledged.remove(&key);
        // C#: MAVFtp.cs:2298-2300, not the answer to the latest send.
        if u32::from(self.request.seq_number) + 1 != u32::from(head.seq_number) {
            return Status::Running;
        }
        // C#: MAVFtp.cs:2302-2307, "confirm this is an ack for the last chunk".
        let size = i64::try_from(self.data.len()).unwrap_or(i64::MAX);
        if size - i64::from(head.offset) <= i64::from(RW_SIZE) {
            self.retry.complete = true;
            return Status::Finished;
        }
        // C#: MAVFtp.cs:2309-2330, "batch 5 at a time".
        let batch: Vec<i64> = self.unacknowledged.iter().take(5).copied().collect();
        for key in batch {
            let Ok(offset) = usize::try_from(key) else {
                continue;
            };
            if self.read_chunk(offset) == 0 {
                continue;
            }
            self.request.seq_number = ctx.next_seq();
            ctx.send(&self.request);
            ctx.progress(
                self.friendly.clone(),
                percent(i64::from(self.request.offset), size),
            );
            self.retry.arm(now);
        }
        Status::Running
    }

    fn on_tick(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        if self.retry.complete {
            return Status::Finished;
        }
        match self.retry.expire(now) {
            Expiry::Waiting => Status::Running,
            Expiry::Again => {
                self.work(ctx);
                self.retry.arm(now);
                Status::Running
            }
            Expiry::GaveUp => Status::Finished,
        }
    }
}

/// A command in flight.
#[derive(Debug, Clone)]
pub(crate) enum Step {
    Simple(Simple),
    List(List),
    Burst(Burst),
    Read(Read),
    Write(Write),
}

/// What a command's method returns, or throws, once `DoWork` has returned.
#[derive(Debug, Clone)]
pub(crate) enum StepResult {
    /// A one-request command: `DoWork`'s answer, the ACK's value if one came, and `ex`.
    Simple {
        opcode: Opcode,
        ans: bool,
        value: Option<u32>,
        ex: Option<FtpError>,
    },
    /// `kCmdListDirectory`.
    List {
        answer: Vec<FtpFileInfo>,
        ended: bool,
        not_supported: bool,
        ex: Option<FtpError>,
    },
    /// `kCmdBurstReadFile`.
    Burst(BurstResult),
    /// `kCmdReadFile`.
    Read {
        answer: Vec<u8>,
        complete: bool,
        ex: Option<FtpError>,
    },
    /// `kCmdWriteFile`. Its `bool` is not kept: `UploadFile`, the one caller, ignores it
    /// (MAVFtp.cs:581).
    Write { ex: Option<FtpError> },
}

/// What `kCmdBurstReadFile` hands back.
#[derive(Debug, Clone)]
pub(crate) struct BurstResult {
    /// The bytes, holes zero-filled.
    pub answer: Vec<u8>,
    /// Whether `DoWork` returned true: the file was whole, rather than the sends running out.
    pub ended: bool,
    pub ex: Option<FtpError>,
}

impl Step {
    pub(crate) fn begin(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        match self {
            Self::Simple(step) => step.begin(ctx, now),
            Self::List(step) => step.begin(ctx, now),
            Self::Burst(step) => step.begin(ctx, now),
            Self::Read(step) => step.begin(ctx, now),
            Self::Write(step) => step.begin(ctx, now),
        }
    }

    pub(crate) fn on_reply(&mut self, head: &Header, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        match self {
            Self::Simple(step) => step.on_reply(head, ctx),
            Self::List(step) => step.on_reply(head, ctx, now),
            Self::Burst(step) => step.on_reply(head, ctx, now),
            Self::Read(step) => step.on_reply(head, ctx, now),
            Self::Write(step) => step.on_reply(head, ctx, now),
        }
    }

    pub(crate) fn on_tick(&mut self, ctx: &mut Ctx<'_>, now: Instant) -> Status {
        match self {
            Self::Simple(step) => step.on_tick(ctx, now),
            Self::List(step) => step.on_tick(ctx, now),
            Self::Burst(step) => step.on_tick(ctx, now),
            Self::Read(step) => step.on_tick(ctx, now),
            Self::Write(step) => step.on_tick(ctx, now),
        }
    }

    /// Back from the nested `kCmdResetSessions()`.
    pub(crate) fn after_reset(&mut self, threw: bool) {
        if let Self::Simple(step) = self {
            step.after_reset(threw);
        }
    }

    /// What the method returns once `DoWork` has. Takes what it returns out of the step, which
    /// is spent afterwards.
    pub(crate) fn finish(&mut self, ctx: &mut Ctx<'_>) -> StepResult {
        match self {
            Self::Simple(step) => StepResult::Simple {
                opcode: step.request.opcode,
                ans: step.retry.complete,
                value: step.value.take(),
                ex: step.ex.take(),
            },
            Self::List(step) => {
                // C#: MAVFtp.cs:1441.
                ctx.progress(format!("{} Ready", step.dir), 100);
                StepResult::List {
                    answer: std::mem::take(&mut step.answer),
                    ended: step.retry.complete,
                    not_supported: step.not_supported,
                    ex: step.ex.take(),
                }
            }
            Self::Burst(step) => {
                // C#: MAVFtp.cs:864.
                ctx.progress(step.file.clone(), 100);
                StepResult::Burst(BurstResult {
                    answer: std::mem::take(&mut step.answer),
                    ended: step.retry.complete,
                    ex: step.ex.take(),
                })
            }
            Self::Read(step) => {
                // C#: MAVFtp.cs:1651.
                ctx.progress(step.file.clone(), 100);
                StepResult::Read {
                    answer: std::mem::take(&mut step.answer),
                    complete: step.retry.complete,
                    ex: step.ex.take(),
                }
            }
            Self::Write(step) => {
                // C#: MAVFtp.cs:2355; an empty file returned before it (:2230-2231).
                if !step.data.is_empty() {
                    ctx.progress(step.friendly.clone(), 100);
                }
                StepResult::Write { ex: step.ex.take() }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn neighbours_merge_and_a_gap_is_found() {
        let mut chunks = vec![(0, 80), (80, 160), (240, 320)];
        simplify_chunk_list(&mut chunks);
        assert_eq!(chunks, [(0, 160), (240, 320)]);
        assert_eq!(find_missing(&chunks), 160);
        chunks.insert(1, (160, 240));
        simplify_chunk_list(&mut chunks);
        assert_eq!(chunks, [(0, 320)]);
        assert_eq!(find_missing(&chunks), u32::MAX);
    }

    #[test]
    fn a_hole_at_the_start_is_found_at_zero() {
        assert_eq!(find_missing(&[(80, 160)]), 0);
        assert_eq!(find_missing(&[]), u32::MAX, "no chunks is no hole");
    }

    #[test]
    fn a_chunk_read_again_inside_a_merged_extent_shortens_it_as_the_csharp_does() {
        // SimplifyChunkList takes the next chunk's end on a merge, even a shorter one.
        let mut chunks = vec![(0, 240), (80, 160)];
        simplify_chunk_list(&mut chunks);
        assert_eq!(chunks, [(0, 160)]);
    }

    #[test]
    fn the_sequence_check_is_int_arithmetic() {
        let request = Header {
            opcode: Opcode::OPEN_FILE_RO,
            seq_number: 7,
            ..Header::default()
        };
        let mut reply = Header {
            req_opcode: Opcode::OPEN_FILE_RO,
            seq_number: 8,
            ..Header::default()
        };
        assert!(answers(&request, &reply));
        reply.seq_number = 7;
        assert!(!answers(&request, &reply));
        let request = Header {
            opcode: Opcode::OPEN_FILE_RO,
            seq_number: u16::MAX,
            ..Header::default()
        };
        reply.seq_number = 0;
        assert!(!answers(&request, &reply), "65535 + 1 is 65536, never 0");
    }
}
