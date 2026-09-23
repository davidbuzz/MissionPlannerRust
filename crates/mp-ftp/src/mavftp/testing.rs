//! A vehicle's side of MAVFTP, for tests: an in-memory file system that answers requests the way
//! ArduPilot's `GCS_FTP.cpp` does, closely enough to drive the client through every path.
//!
//! It answers every request and loses nothing; a test that wants a reply lost drops it on the
//! way. Normal operation uses nothing here.

use std::collections::{BTreeMap, BTreeSet};

use super::crc::crc_crc32;
use super::wire::{DATA_LEN, Errno, ErrorCode, Header, Opcode};

/// How many chunks ArduPilot sends for one `kCmdBurstReadFile` before it marks the burst complete
/// and waits to be asked again: `transfer_size`, "enough for a full parameter file with max
/// parameters" (libraries/GCS_MAVLink/GCS_FTP.cpp:609 in ArduPilot master).
pub const ARDUPILOT_BURST_CHUNKS: usize = 2000;

/// A file system and the one session ArduPilot keeps.
#[derive(Debug, Clone)]
pub struct FakeVehicle {
    /// Files, by full path.
    pub files: BTreeMap<String, Vec<u8>>,
    /// Directories, by full path. `/` always exists.
    pub dirs: BTreeSet<String>,
    /// Sizes `kCmdOpenFileRO` reports in place of a file's real one, as ArduPilot's virtual
    /// `@SYS` files report a size they do not have.
    pub reported_sizes: BTreeMap<String, u32>,
    /// Modification times for the timed listing, by full path; zero if absent.
    pub times: BTreeMap<String, u32>,
    /// Chunks per burst before `burst_complete`.
    pub burst_chunks: usize,
    /// Whether `kCmdListDirectoryWithTime` is known. ArduPilot's answer to an opcode it does not
    /// know is `kErrUnknownCommand`.
    pub knows_list_with_time: bool,
    /// Another ground station holds the session: opens are refused with
    /// `kErrNoSessionsAvailable` until a `kCmdResetSessions` frees it.
    pub session_held_elsewhere: bool,
    /// Refuse this opcode with this NAK, every time.
    pub refuse: Option<(Opcode, ErrorCode, Errno)>,
    /// Refuse the next request with this opcode with this NAK, once.
    pub refuse_once: Option<(Opcode, ErrorCode, Errno)>,
    open: Option<(String, bool)>,
    /// Every request heard, in order.
    pub heard: Vec<Header>,
}

impl Default for FakeVehicle {
    fn default() -> Self {
        Self::new()
    }
}

impl FakeVehicle {
    /// An empty file system with ArduPilot's burst length.
    #[must_use]
    pub fn new() -> Self {
        let mut dirs = BTreeSet::new();
        dirs.insert("/".to_owned());
        Self {
            files: BTreeMap::new(),
            dirs,
            reported_sizes: BTreeMap::new(),
            times: BTreeMap::new(),
            burst_chunks: ARDUPILOT_BURST_CHUNKS,
            knows_list_with_time: false,
            session_held_elsewhere: false,
            refuse: None,
            refuse_once: None,
            open: None,
            heard: Vec::new(),
        }
    }

    /// Adds a file, and the directories above it.
    #[must_use]
    pub fn with_file(mut self, path: &str, data: &[u8]) -> Self {
        self.add_parents(path);
        self.files.insert(path.to_owned(), data.to_vec());
        self
    }

    /// Adds a directory, and the directories above it.
    #[must_use]
    pub fn with_dir(mut self, path: &str) -> Self {
        self.add_parents(path);
        self.dirs.insert(path.to_owned());
        self
    }

    fn add_parents(&mut self, path: &str) {
        let mut parent = parent_of(path);
        while let Some(dir) = parent {
            self.dirs.insert(dir.clone());
            parent = parent_of(&dir);
        }
    }

    /// The replies to one request, in the order the vehicle sends them.
    pub fn answer(&mut self, request: &Header) -> Vec<Header> {
        self.heard.push(request.clone());
        let once = self
            .refuse_once
            .filter(|(opcode, ..)| *opcode == request.opcode);
        if once.is_some() {
            self.refuse_once = None;
        }
        if let Some((opcode, error, errno)) = once.or(self.refuse)
            && opcode == request.opcode
        {
            let mut nak = reply(request, Opcode::NAK);
            nak.set_data(&[error.0, errno.0]);
            return vec![nak];
        }
        let path = text(request.payload());
        match request.opcode {
            Opcode::RESET_SESSIONS => {
                self.open = None;
                self.session_held_elsewhere = false;
                vec![reply(request, Opcode::ACK)]
            }
            Opcode::TERMINATE_SESSION => {
                self.open = None;
                vec![reply(request, Opcode::ACK)]
            }
            Opcode::LIST_DIRECTORY => vec![self.list(request, &path, false)],
            Opcode::LIST_DIRECTORY_WITH_TIME if self.knows_list_with_time => {
                vec![self.list(request, &path, true)]
            }
            Opcode::OPEN_FILE_RO => vec![self.open_read(request, &path)],
            Opcode::READ_FILE => vec![self.read(request)],
            Opcode::BURST_READ_FILE => self.burst(request),
            Opcode::CREATE_FILE => {
                if self.session_held_elsewhere {
                    return vec![nak(request, ErrorCode::NO_SESSIONS_AVAILABLE)];
                }
                self.add_parents(&path);
                self.files.insert(path.clone(), Vec::new());
                self.open = Some((path, true));
                vec![reply(request, Opcode::ACK)]
            }
            Opcode::WRITE_FILE => vec![self.write(request)],
            Opcode::REMOVE_FILE => {
                if self.files.remove(&path).is_some() {
                    vec![reply(request, Opcode::ACK)]
                } else {
                    vec![nak(request, ErrorCode::FILE_NOT_FOUND)]
                }
            }
            Opcode::CREATE_DIRECTORY => {
                if self.dirs.contains(&path) || self.files.contains_key(&path) {
                    // ArduPilot turns EEXIST into its own code (GCS_FTP.cpp, ftp_error).
                    vec![nak(request, ErrorCode::FAIL_FILE_EXISTS)]
                } else {
                    self.add_parents(&path);
                    self.dirs.insert(path);
                    vec![reply(request, Opcode::ACK)]
                }
            }
            Opcode::REMOVE_DIRECTORY => {
                if !self.dirs.contains(&path) {
                    vec![nak(request, ErrorCode::FILE_NOT_FOUND)]
                } else if self.children(&path).is_empty() {
                    self.dirs.remove(&path);
                    vec![reply(request, Opcode::ACK)]
                } else {
                    vec![nak_errno(request, Errno::ENOTEMPTY)]
                }
            }
            Opcode::RENAME => {
                let (from, to) = path.split_once('\0').unwrap_or((path.as_str(), ""));
                match self.files.remove(from) {
                    Some(data) => {
                        self.files.insert(to.to_owned(), data);
                        vec![reply(request, Opcode::ACK)]
                    }
                    None => vec![nak(request, ErrorCode::FILE_NOT_FOUND)],
                }
            }
            Opcode::CALC_FILE_CRC32 => match self.files.get(&path) {
                Some(data) => {
                    let mut ack = reply(request, Opcode::ACK);
                    ack.set_data(&crc_crc32(0, data).to_le_bytes());
                    vec![ack]
                }
                None => vec![nak(request, ErrorCode::FILE_NOT_FOUND)],
            },
            _ => vec![nak(request, ErrorCode::UNKNOWN_COMMAND)],
        }
    }

    /// Everything directly inside `dir`, as listing entries, directories first then files, each
    /// by name.
    fn children(&self, dir: &str) -> Vec<(String, bool)> {
        let inside = |path: &String| parent_of(path).as_deref() == Some(dir);
        let mut out: Vec<(String, bool)> = self
            .dirs
            .iter()
            .filter(|path| inside(path))
            .map(|path| (path.clone(), true))
            .collect();
        out.extend(
            self.files
                .keys()
                .filter(|path| inside(path))
                .map(|path| (path.clone(), false)),
        );
        out
    }

    fn list(&self, request: &Header, path: &str, with_time: bool) -> Header {
        let dir = if path.is_empty() { "/" } else { path };
        if !self.dirs.contains(dir) {
            return nak(request, ErrorCode::FILE_NOT_FOUND);
        }
        let mut ack = reply(request, Opcode::ACK);
        ack.offset = request.offset;
        let mut entries = Vec::new();
        let start = usize::try_from(request.offset).unwrap_or(usize::MAX);
        for (full, is_dir) in self.children(dir).into_iter().skip(start) {
            let name = full.rsplit('/').next().unwrap_or(&full);
            let time = self.times.get(&full).copied().unwrap_or(0);
            let size = self.files.get(&full).map_or(0, Vec::len);
            let entry = match (is_dir, with_time) {
                (true, false) => format!("D{name}\0"),
                (true, true) => format!("D{name}\t0\t{time}\0"),
                (false, false) => format!("F{name}\t{size}\0"),
                (false, true) => format!("F{name}\t{size}\t{time}\0"),
            };
            if entries.len() + entry.len() > DATA_LEN {
                break;
            }
            entries.extend_from_slice(entry.as_bytes());
        }
        if entries.is_empty() {
            return nak(request, ErrorCode::EOF);
        }
        ack.set_data(&entries);
        ack
    }

    fn open_read(&mut self, request: &Header, path: &str) -> Header {
        if self.session_held_elsewhere {
            return nak(request, ErrorCode::NO_SESSIONS_AVAILABLE);
        }
        let Some(data) = self.files.get(path) else {
            return nak(request, ErrorCode::FILE_NOT_FOUND);
        };
        let size = self
            .reported_sizes
            .get(path)
            .copied()
            .unwrap_or_else(|| u32::try_from(data.len()).unwrap_or(u32::MAX));
        self.open = Some((path.to_owned(), false));
        let mut ack = reply(request, Opcode::ACK);
        ack.set_data(&size.to_le_bytes());
        ack
    }

    fn open_data(&self, write: bool) -> Option<&Vec<u8>> {
        match &self.open {
            Some((path, mode)) if *mode == write => self.files.get(path),
            _ => None,
        }
    }

    /// One chunk at `offset` of at most `max` bytes, or `None` at the end of the file.
    fn chunk(&self, request: &Header, offset: u32, max: usize) -> Option<Header> {
        let data = self.open_data(false)?;
        let start = usize::try_from(offset).ok()?;
        if start >= data.len() {
            return None;
        }
        let n = max.min(DATA_LEN).min(data.len() - start);
        let mut ack = reply(request, Opcode::ACK);
        ack.offset = offset;
        ack.set_data(data.get(start..start + n)?);
        Some(ack)
    }

    fn read(&self, request: &Header) -> Header {
        if self.open_data(false).is_none() {
            return nak(request, ErrorCode::FAIL);
        }
        let max = usize::from(request.size);
        self.chunk(request, request.offset, max)
            .unwrap_or_else(|| nak(request, ErrorCode::EOF))
    }

    fn burst(&self, request: &Header) -> Vec<Header> {
        if self.open_data(false).is_none() {
            return vec![nak(request, ErrorCode::FAIL)];
        }
        let max = if request.size == 0 {
            DATA_LEN
        } else {
            usize::from(request.size)
        };
        let mut out = Vec::new();
        let mut offset = request.offset;
        let mut seq = request.seq_number.wrapping_add(1);
        for i in 0..self.burst_chunks {
            let Some(mut ack) = self.chunk(request, offset, max) else {
                let mut eof = nak(request, ErrorCode::EOF);
                eof.seq_number = seq;
                eof.offset = offset;
                // GCS_FTP.cpp:626, `reply.burst_complete = true;` before the end-of-file NAK.
                eof.burst_complete = 1;
                out.push(eof);
                break;
            };
            ack.seq_number = seq;
            ack.burst_complete = u8::from(i + 1 == self.burst_chunks);
            let short = usize::from(ack.size) < max;
            offset = offset.wrapping_add(u32::from(ack.size));
            out.push(ack);
            seq = seq.wrapping_add(1);
            if short {
                let mut eof = nak(request, ErrorCode::EOF);
                eof.seq_number = seq;
                eof.offset = offset;
                // GCS_FTP.cpp:626, `reply.burst_complete = true;` before the end-of-file NAK.
                eof.burst_complete = 1;
                out.push(eof);
                break;
            }
        }
        out
    }

    fn write(&mut self, request: &Header) -> Header {
        let Some((path, true)) = self.open.clone() else {
            return nak(request, ErrorCode::FAIL);
        };
        let Some(file) = self.files.get_mut(&path) else {
            return nak(request, ErrorCode::FAIL);
        };
        let start = usize::try_from(request.offset).unwrap_or(usize::MAX);
        let chunk = request.payload();
        let stop = start.saturating_add(chunk.len());
        if file.len() < stop {
            file.resize(stop, 0);
        }
        if let Some(slot) = file.get_mut(start..stop) {
            slot.copy_from_slice(chunk);
        }
        let mut ack = reply(request, Opcode::ACK);
        ack.offset = request.offset;
        ack
    }
}

/// The directory a path is in, or `None` for `/`.
fn parent_of(path: &str) -> Option<String> {
    if path == "/" || path.is_empty() {
        return None;
    }
    let trimmed = path.trim_end_matches('/');
    match trimmed.rfind('/') {
        Some(0) => Some("/".to_owned()),
        Some(at) => trimmed.get(..at).map(str::to_owned),
        // "@SYS" and the like hang off the root.
        None => Some("/".to_owned()),
    }
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

/// A reply to `request`: its sequence number plus one and its opcode echoed.
fn reply(request: &Header, opcode: Opcode) -> Header {
    Header {
        seq_number: request.seq_number.wrapping_add(1),
        session: request.session,
        opcode,
        size: 0,
        req_opcode: request.opcode,
        burst_complete: 0,
        padding: 0,
        offset: request.offset,
        ..Header::default()
    }
}

/// A NAK carrying `error`.
fn nak(request: &Header, error: ErrorCode) -> Header {
    let mut out = reply(request, Opcode::NAK);
    out.set_data(&[error.0]);
    out
}

/// A `kErrFailErrno` NAK carrying `errno`.
fn nak_errno(request: &Header, errno: Errno) -> Header {
    let mut out = reply(request, Opcode::NAK);
    out.set_data(&[ErrorCode::FAIL_ERRNO.0, errno.0]);
    out
}
