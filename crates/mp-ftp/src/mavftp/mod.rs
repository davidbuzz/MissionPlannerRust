//! MAVFTP: the vehicle's file system over `FILE_TRANSFER_PROTOCOL`.
//!
//! Replaces the client in `ExtLibs/ArduPilot/Mavlink/MAVFtp.cs`. [`MavFtp`] is one instance of the
//! C#'s `MAVFtp` class - one vehicle, one sequence counter, one command at a time - driven by
//! whoever owns the link: [`MavFtp::start`] to begin a request, [`MavFtp::on_message`] with each
//! `FILE_TRANSFER_PROTOCOL` payload from that vehicle, and [`MavFtp::on_tick`] as time passes.
//! Each returns the payloads to send. Nothing here touches a socket, so every path through it is
//! tested with scripted replies.
//!
//! # What a request is
//!
//! A request is one of the C#'s public entry points, which are sequences of commands:
//!
//! * [`FtpRequest::List`] - `kCmdListDirectory` (MAVFtp.cs:1248-1262): with modification times
//!   first, and once the vehicle has refused that opcode, plain listings from then on.
//! * [`FtpRequest::Get`] - `GetFile` (MAVFtp.cs:560-574): reset sessions, open the file, then
//!   a burst read or a plain one. The burst read (`kCmdBurstReadFile`, :694-870) is what the
//!   parameter download, the mission download over FTP and the upload page's "Download Burst"
//!   use; the plain one (`kCmdReadFile`, :1547-1659) is the page's "Download" and the serial
//!   ports page's `uarts.txt`.
//! * [`FtpRequest::Put`] - `UploadFile` (MAVFtp.cs:576-592): reset, create, write, reset.
//! * The one-command requests: `kCmdCalcFileCRC32`, `kCmdRemoveFile`, `kCmdRemoveDirectory`,
//!   `kCmdCreateDirectory`, `kCmdRename`, `kCmdResetSessions`, `kCmdTerminateSession`.
//!
//! What each returns, or throws, is [`FtpOutcome`] or [`FtpError`], as the C# returns or throws
//! it: a listing that times out is still a listing, and a burst read whose retries run out still
//! hands back the bytes it has. [`FtpOutcome`] also carries whether `DoWork` succeeded, which the
//! C# computes and throws away, so a caller can tell.
//!
//! # Where this differs from the C#, and why
//!
//! * **`kErrNoSessionsAvailable`.** Several handlers call `kCmdResetSessions()` from inside
//!   themselves, then carry on (e.g. MAVFtp.cs:654-658). In the C# that cannot work: the handler
//!   runs on the thread that reads the link (MAVLinkInterface.cs:5367, 5528-5553), so the nested
//!   call waits five seconds for an answer nobody can read; and it writes its own payload into
//!   the shared `fileTransferProtocol` (:39, :1916), so every later retry of the outer command
//!   sends a reset instead of the command. The outer command then times out. This port does what
//!   the code says it means: the reset runs as a command of its own, the outer command waits for
//!   it, and then resumes, resending *its own* request.
//! * **Addressing.** The C# addresses every message from the shared `fileTransferProtocol`, whose
//!   target is only set by the commands that set it (e.g. :596-598). A fresh instance's first
//!   `kCmdResetSessions` - the first thing `GetFile` and `UploadFile` do - therefore goes to system
//!   0, component 0: every vehicle on the link. Here every message goes to the vehicle this
//!   client is for.
//! * **Polling.** `RetryTimeout.DoWork` notices completion on a 100 ms poll (RetryTimeout.cs:69-75),
//!   so the C# starts each command of a sequence up to 100 ms after the last ended, and handles
//!   replies that arrive in between with a handler that has already finished. Here a command ends
//!   when it ends; see [`retry`].
//! * **Progress.** Kept as the C# reports it ([`Progress`]), except the upload's bytes-per-second,
//!   which the C# measures from the link's send counter (:2238-2246); that belongs to the link.

pub mod crc;
pub mod listing;
pub mod retry;
mod steps;
pub mod testing;
pub mod wire;

use std::time::Instant;

pub use crc::crc_crc32;
pub use listing::FtpFileInfo;
pub use retry::{FtpTimeouts, Patience};
pub use steps::RW_SIZE;
use steps::{Burst, Ctx, List, Read, Simple, Status, Step, StepResult, Write};
use wire::{Header, Opcode};

use crate::FtpError;
use mp_vehicle::VehicleId;

/// A request, as one of the C#'s public entry points makes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FtpRequest {
    /// `kCmdListDirectory(dir, cancel)`.
    List {
        /// The directory. A trailing `/` is dropped unless it is all there is.
        path: String,
    },
    /// `GetFile(file, cancel, burst, readsize)`.
    Get {
        /// The file.
        path: String,
        /// `kCmdBurstReadFile` rather than `kCmdReadFile`. `GetFile`'s default is true.
        burst: bool,
        /// Bytes per read. `GetFile`'s default is [`RW_SIZE`], 80; the parameter and mission
        /// downloads ask for 110 (MAVLinkInterface.cs:1877, GCSViews/FlightPlanner.cs:3995).
        readsize: u8,
    },
    /// `UploadFile(file, srcfile, cancel)`.
    Put {
        /// Where the file goes on the vehicle.
        path: String,
        /// What goes in it.
        data: Vec<u8>,
    },
    /// `kCmdCalcFileCRC32(file, ref crc32, cancel)`.
    Crc32 {
        /// The file.
        path: String,
    },
    /// `kCmdRemoveFile(file, cancel)`.
    RemoveFile {
        /// The file.
        path: String,
    },
    /// `kCmdRemoveDirectory(file, cancel)`.
    RemoveDirectory {
        /// The directory, which must be empty.
        path: String,
    },
    /// `kCmdCreateDirectory(file, cancel)`.
    CreateDirectory {
        /// The directory.
        path: String,
    },
    /// `kCmdRename(src, dest, cancel)`.
    Rename {
        /// The file now.
        from: String,
        /// The file after.
        to: String,
    },
    /// `kCmdResetSessions()`.
    ResetSessions,
    /// `kCmdTerminateSession()`.
    TerminateSession,
}

/// What a request returned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FtpOutcome {
    /// `kCmdListDirectory`'s list.
    Listing {
        /// The entries, in the order the vehicle gave them.
        entries: Vec<FtpFileInfo>,
        /// Whether the listing reached its end, rather than its retries running out. The C#
        /// returns the entries either way.
        ended: bool,
    },
    /// `GetFile`'s stream.
    File {
        /// The file, or `None` where `GetFile` returns null: the file never opened (no size,
        /// MAVFtp.cs:566-567), or a plain read did not finish (:1654-1655).
        data: Option<Vec<u8>>,
        /// Whether the read finished, rather than its retries running out. A burst read that
        /// ran out still returns what it has, holes zero-filled, as the C# does (:862-869).
        ended: bool,
    },
    /// `UploadFile` returned.
    Uploaded,
    /// `kCmdCalcFileCRC32`.
    Crc32 {
        /// The vehicle's CRC, or `u32::MAX` if it never answered (MAVFtp.cs:929, 996).
        crc: u32,
        /// What the method returns: whether the vehicle answered.
        answered: bool,
    },
    /// A command that returns `bool`: whether the vehicle acknowledged it.
    Done(bool),
}

/// The last `Progress` event: a line of text and a percentage, or -1 for "no percentage".
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Progress {
    /// What is happening.
    pub message: String,
    /// How far through, 0 to 100, or -1.
    pub percent: i32,
}

/// Where a multi-command request is.
#[derive(Debug, Clone)]
enum Job {
    /// Listing, `with_time` or plain.
    List { path: String, with_time: bool },
    /// `GetFile`.
    Get {
        path: String,
        burst: bool,
        readsize: u8,
    },
    /// `UploadFile`, and which of its four commands is running.
    Put {
        path: String,
        data: Option<Vec<u8>>,
        phase: PutPhase,
    },
    /// A request that is one command.
    One,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PutPhase {
    Reset,
    Create,
    Write,
    ResetAgain,
}

/// A request in flight.
#[derive(Debug, Clone)]
struct Running {
    job: Job,
    step: Step,
    /// The `kCmdResetSessions()` a handler called, while the command it interrupted waits.
    nested_reset: Option<Step>,
    cancelled: bool,
}

/// One vehicle's MAVFTP client: the C#'s `MAVFtp` instance.
#[derive(Debug, Clone)]
pub struct MavFtp {
    target: VehicleId,
    timeouts: FtpTimeouts,
    /// `seq_no`: "incremented anytime its not a retransmit" (MAVFtp.cs:42-43).
    seq_no: u16,
    /// `listDirectoryWithTimeUnsupported` (MAVFtp.cs:47-48).
    list_directory_with_time_unsupported: bool,
    running: Option<Running>,
    outcome: Option<Result<FtpOutcome, FtpError>>,
    progress: Progress,
}

/// What [`MavFtp::step_finished`] decides.
enum Next {
    Step(Box<Step>),
    Done(Result<FtpOutcome, FtpError>),
}

impl MavFtp {
    /// A client for one vehicle: `new MAVFtp(mavint, sysid, compid)`.
    #[must_use]
    pub const fn new(target: VehicleId, timeouts: FtpTimeouts) -> Self {
        Self {
            target,
            timeouts,
            seq_no: 0,
            list_directory_with_time_unsupported: false,
            running: None,
            outcome: None,
            progress: Progress {
                message: String::new(),
                percent: 0,
            },
        }
    }

    /// The vehicle this client talks to.
    #[must_use]
    pub const fn target(&self) -> VehicleId {
        self.target
    }

    /// Whether a request is in flight.
    #[must_use]
    pub const fn is_busy(&self) -> bool {
        self.running.is_some()
    }

    /// The last progress report.
    #[must_use]
    pub const fn progress(&self) -> &Progress {
        &self.progress
    }

    /// The last request's outcome, once it has one.
    #[must_use]
    pub const fn outcome(&self) -> Option<&Result<FtpOutcome, FtpError>> {
        self.outcome.as_ref()
    }

    /// Takes the last request's outcome.
    pub const fn take_outcome(&mut self) -> Option<Result<FtpOutcome, FtpError>> {
        self.outcome.take()
    }

    /// Whether the vehicle has refused `kCmdListDirectoryWithTime`, so listings are plain.
    #[must_use]
    pub const fn list_directory_with_time_unsupported(&self) -> bool {
        self.list_directory_with_time_unsupported
    }

    /// Asks the running request to stop: the caller's `CancellationTokenSource.Cancel()`.
    ///
    /// As in the C#, the command notices at its next reply or the end of its current wait, and
    /// ends when that wait runs out (`timeout.RetriesCurrent = 999`); a reset or terminate in
    /// progress does not look (MAVFtp.cs:1908-1961).
    pub const fn cancel(&mut self) {
        if let Some(running) = self.running.as_mut() {
            running.cancelled = true;
        }
    }

    /// Starts a request. Returns false, and sends nothing, if one is already in flight.
    pub fn start(&mut self, request: FtpRequest, now: Instant, out: &mut Vec<Header>) -> bool {
        if self.running.is_some() {
            return false;
        }
        self.outcome = None;
        let mut progress = std::mem::take(&mut self.progress);
        let mut ctx = Ctx {
            seq_no: &mut self.seq_no,
            out,
            cancelled: false,
            progress: &mut progress,
        };
        let (job, step) = first_step(
            request,
            self.list_directory_with_time_unsupported,
            &self.timeouts,
            &mut ctx,
            now,
        );
        self.progress = progress;
        self.running = Some(Running {
            job,
            step,
            nested_reset: None,
            cancelled: false,
        });
        self.drive(out, now, |step, ctx| step.begin(ctx, now));
        true
    }

    /// One `FILE_TRANSFER_PROTOCOL` payload from this client's vehicle.
    pub fn on_message(&mut self, payload: &[u8], now: Instant, out: &mut Vec<Header>) {
        let head = Header::decode(payload);
        self.drive(out, now, |step, ctx| step.on_reply(&head, ctx, now));
    }

    /// Time has passed: a wait may have run out.
    pub fn on_tick(&mut self, now: Instant, out: &mut Vec<Header>) {
        self.drive(out, now, |step, ctx| step.on_tick(ctx, now));
    }

    /// Hands an event to whichever command is current - a nested reset if one is running - and
    /// follows the request on to its next command, or its end, for as long as commands finish.
    fn drive(
        &mut self,
        out: &mut Vec<Header>,
        now: Instant,
        event: impl FnOnce(&mut Step, &mut Ctx<'_>) -> Status,
    ) {
        let Some(mut running) = self.running.take() else {
            return;
        };
        let mut progress = std::mem::take(&mut self.progress);
        let mut ctx = Ctx {
            seq_no: &mut self.seq_no,
            out,
            cancelled: running.cancelled,
            progress: &mut progress,
        };

        let mut status = match running.nested_reset.as_mut() {
            Some(reset) => event(reset, &mut ctx),
            None => event(&mut running.step, &mut ctx),
        };
        let result = loop {
            match status {
                Status::Running => break None,
                Status::ResetSessions => {
                    // A handler asked for kCmdResetSessions(); see the module's notes.
                    let mut reset = Step::Simple(Simple::new(
                        Opcode::RESET_SESSIONS,
                        &[],
                        "",
                        &self.timeouts,
                        &mut ctx,
                        now,
                    ));
                    status = reset.begin(&mut ctx, now);
                    running.nested_reset = Some(reset);
                }
                Status::Finished => {
                    if let Some(mut reset) = running.nested_reset.take() {
                        let threw = matches!(
                            reset.finish(&mut ctx),
                            StepResult::Simple { ex: Some(_), .. }
                        );
                        running.step.after_reset(threw);
                        // The interrupted command carries on: if its wait ran out meanwhile, it
                        // sends again now.
                        status = running.step.on_tick(&mut ctx, now);
                        continue;
                    }
                    let finished = running.step.finish(&mut ctx);
                    match step_finished(
                        &mut running.job,
                        finished,
                        &mut self.list_directory_with_time_unsupported,
                        &self.timeouts,
                        &mut ctx,
                        now,
                    ) {
                        Next::Step(next) => {
                            running.step = *next;
                            status = running.step.begin(&mut ctx, now);
                        }
                        Next::Done(result) => break Some(result),
                    }
                }
            }
        };
        self.progress = progress;
        match result {
            Some(result) => self.outcome = Some(result),
            None => self.running = Some(running),
        }
    }
}

/// The first command of a request.
fn first_step(
    request: FtpRequest,
    with_time_unsupported: bool,
    timeouts: &FtpTimeouts,
    ctx: &mut Ctx<'_>,
    now: Instant,
) -> (Job, Step) {
    let simple = |opcode, data: &[u8], path: &str, ctx: &mut Ctx<'_>| {
        Step::Simple(Simple::new(opcode, data, path, timeouts, ctx, now))
    };
    match request {
        FtpRequest::List { path } => {
            // C#: MAVFtp.cs:1252-1261.
            let with_time = !with_time_unsupported;
            let step = Step::List(List::new(&path, with_time, timeouts, ctx, now));
            (Job::List { path, with_time }, step)
        }
        FtpRequest::Get {
            path,
            burst,
            readsize,
        } => {
            // C#: MAVFtp.cs:562-564.
            *ctx.progress = Progress {
                message: format!("Opening file {path}"),
                percent: -1,
            };
            let step = simple(Opcode::RESET_SESSIONS, &[], "", ctx);
            (
                Job::Get {
                    path,
                    burst,
                    readsize,
                },
                step,
            )
        }
        FtpRequest::Put { path, data } => {
            // C#: MAVFtp.cs:576-583.
            let step = simple(Opcode::RESET_SESSIONS, &[], "", ctx);
            (
                Job::Put {
                    path,
                    data: Some(data),
                    phase: PutPhase::Reset,
                },
                step,
            )
        }
        FtpRequest::Crc32 { path } => (
            Job::One,
            simple(Opcode::CALC_FILE_CRC32, path.as_bytes(), &path, ctx),
        ),
        FtpRequest::RemoveFile { path } => {
            // C#: MAVFtp.cs:1748.
            let path = path.replace("//", "/");
            (
                Job::One,
                simple(Opcode::REMOVE_FILE, path.as_bytes(), &path, ctx),
            )
        }
        FtpRequest::RemoveDirectory { path } => {
            // C#: MAVFtp.cs:1663.
            let path = path.replace("//", "/");
            (
                Job::One,
                simple(Opcode::REMOVE_DIRECTORY, path.as_bytes(), &path, ctx),
            )
        }
        FtpRequest::CreateDirectory { path } => (
            Job::One,
            simple(Opcode::CREATE_DIRECTORY, path.as_bytes(), &path, ctx),
        ),
        FtpRequest::Rename { from, to } => {
            // C#: MAVFtp.cs:1839, `src + "\0" + dest`.
            let data = format!("{from}\0{to}");
            (
                Job::One,
                simple(Opcode::RENAME, data.as_bytes(), &from, ctx),
            )
        }
        FtpRequest::ResetSessions => (Job::One, simple(Opcode::RESET_SESSIONS, &[], "", ctx)),
        FtpRequest::TerminateSession => (Job::One, simple(Opcode::TERMINATE_SESSION, &[], "", ctx)),
    }
}

/// A command has returned: what the request does next.
fn step_finished(
    job: &mut Job,
    finished: StepResult,
    with_time_unsupported: &mut bool,
    timeouts: &FtpTimeouts,
    ctx: &mut Ctx<'_>,
    now: Instant,
) -> Next {
    match (job, finished) {
        (
            Job::List { path, with_time },
            StepResult::List {
                answer,
                ended,
                not_supported,
                ex,
            },
        ) => {
            // C#: MAVFtp.cs:1441-1445, then :1254-1260.
            if let Some(ex) = ex {
                return Next::Done(Err(ex));
            }
            if *with_time && not_supported {
                // "ListDirectoryWithTime unsupported, falling back to ListDirectory".
                *with_time_unsupported = true;
                *with_time = false;
                return Next::Step(Box::new(Step::List(List::new(
                    path, false, timeouts, ctx, now,
                ))));
            }
            Next::Done(Ok(FtpOutcome::Listing {
                entries: answer,
                ended,
            }))
        }
        (
            Job::Get {
                path,
                burst,
                readsize,
            },
            StepResult::Simple {
                opcode,
                ans,
                value,
                ex,
            },
        ) => {
            if let Some(ex) = ex {
                return Next::Done(Err(ex));
            }
            if opcode == Opcode::RESET_SESSIONS {
                // C#: MAVFtp.cs:565, whatever the reset returned.
                return Next::Step(Box::new(Step::Simple(Simple::new(
                    Opcode::OPEN_FILE_RO,
                    path.as_bytes(),
                    path,
                    timeouts,
                    ctx,
                    now,
                ))));
            }
            // C#: MAVFtp.cs:566-567 and :687, `size = localsize`: -1 unless the ACK came, and
            // the ACK's four bytes as an int if it did.
            let size = value.map_or(-1, |v| i32::from_ne_bytes(v.to_ne_bytes()));
            // C#: MAVFtp.cs:690, "Opened " + file + " " + ans.
            *ctx.progress = Progress {
                message: format!("Opened {path} {}", if ans { "True" } else { "False" }),
                percent: -1,
            };
            if size == -1 {
                return Next::Done(Ok(FtpOutcome::File {
                    data: None,
                    ended: false,
                }));
            }
            let step = if *burst {
                Burst::new(path, size, *readsize, timeouts, ctx, now).map(Step::Burst)
            } else {
                Read::new(path, size, *readsize, timeouts, ctx, now).map(Step::Read)
            };
            match step {
                Ok(step) => Next::Step(Box::new(step)),
                Err(error) => Next::Done(Err(error)),
            }
        }
        (Job::Get { .. }, StepResult::Burst(result)) => {
            // C#: MAVFtp.cs:866-869.
            Next::Done(match result.ex {
                Some(ex) => Err(ex),
                None => Ok(FtpOutcome::File {
                    data: Some(result.answer),
                    ended: result.ended,
                }),
            })
        }
        (
            Job::Get { .. },
            StepResult::Read {
                answer,
                complete,
                ex,
            },
        ) => {
            // C#: MAVFtp.cs:1654-1658: not complete is null, whatever went wrong.
            if !complete {
                return Next::Done(Ok(FtpOutcome::File {
                    data: None,
                    ended: false,
                }));
            }
            Next::Done(match ex {
                Some(ex) => Err(ex),
                None => Ok(FtpOutcome::File {
                    data: Some(answer),
                    ended: true,
                }),
            })
        }
        (Job::Put { path, data, phase }, result) => {
            // C#: MAVFtp.cs:576-583. Each command's answer is ignored; only a throw stops it.
            let ex = match result {
                StepResult::Simple { ex, .. } | StepResult::Write { ex, .. } => ex,
                _ => None,
            };
            if let Some(ex) = ex {
                return Next::Done(Err(ex));
            }
            match phase {
                PutPhase::Reset => {
                    *phase = PutPhase::Create;
                    Next::Step(Box::new(Step::Simple(Simple::new(
                        Opcode::CREATE_FILE,
                        path.as_bytes(),
                        path,
                        timeouts,
                        ctx,
                        now,
                    ))))
                }
                PutPhase::Create => {
                    *phase = PutPhase::Write;
                    // The progress name is the file's, as UploadFile(file, Stream) gives it
                    // (MAVFtp.cs:590).
                    let name = path.rsplit('/').next().unwrap_or(path.as_str()).to_owned();
                    Next::Step(Box::new(Step::Write(Write::new(
                        &name,
                        data.take().unwrap_or_default(),
                        timeouts,
                        ctx,
                        now,
                    ))))
                }
                PutPhase::Write => {
                    *phase = PutPhase::ResetAgain;
                    Next::Step(Box::new(Step::Simple(Simple::new(
                        Opcode::RESET_SESSIONS,
                        &[],
                        "",
                        timeouts,
                        ctx,
                        now,
                    ))))
                }
                PutPhase::ResetAgain => Next::Done(Ok(FtpOutcome::Uploaded)),
            }
        }
        (
            Job::One,
            StepResult::Simple {
                opcode,
                ans,
                value,
                ex,
            },
        ) => Next::Done(match ex {
            Some(ex) => Err(ex),
            None if opcode == Opcode::CALC_FILE_CRC32 => Ok(FtpOutcome::Crc32 {
                crc: value.unwrap_or(u32::MAX),
                answered: ans,
            }),
            None => Ok(FtpOutcome::Done(ans)),
        }),
        // A command finished that its request never starts; nothing sensible to continue with.
        (_, _) => Next::Done(Ok(FtpOutcome::Done(false))),
    }
}
