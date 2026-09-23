//! What goes inside a `FILE_TRANSFER_PROTOCOL` message: the 251-byte payload, its opcodes and its
//! error codes.
//!
//! C#: ExtLibs/ArduPilot/Mavlink/MAVFtp.cs:60-463 (`errno`), :466-497 (`FTPErrorCode`), :500-558
//! (`FTPOpcode`) and :2363-2418 (`FTPPayloadHeader`).

use std::fmt;

/// Bytes in a `FILE_TRANSFER_PROTOCOL` payload.
///
/// C#: MAVFtp.cs:2363 (`[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 251)]`).
pub const PAYLOAD_LEN: usize = 251;

/// Bytes of header before the data.
pub const HEADER_LEN: usize = 12;

/// Bytes of data after the header.
///
/// C#: MAVFtp.cs:2391 (`SizeConst = 251 - 12`).
pub const DATA_LEN: usize = PAYLOAD_LEN - HEADER_LEN;

/// A command or response opcode: `FTPOpcode`.
///
/// A number rather than a closed enum, because the C# casts whatever byte arrives to the enum
/// without checking it, and prints an unknown one as its number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Opcode(pub u8);

impl Opcode {
    /// `kCmdNone`: ignored, always acked.
    pub const NONE: Self = Self(0);
    /// `kCmdTerminateSession`: terminates open read session.
    pub const TERMINATE_SESSION: Self = Self(1);
    /// `kCmdResetSessions`: terminates all open read sessions.
    pub const RESET_SESSIONS: Self = Self(2);
    /// `kCmdListDirectory`: list files in `<path>` from offset.
    pub const LIST_DIRECTORY: Self = Self(3);
    /// `kCmdOpenFileRO`: opens file at `<path>` for reading, returns `<session>`.
    pub const OPEN_FILE_RO: Self = Self(4);
    /// `kCmdReadFile`: reads `<size>` bytes from `<offset>` in `<session>`.
    pub const READ_FILE: Self = Self(5);
    /// `kCmdCreateFile`: creates file at `<path>` for writing, returns `<session>`.
    pub const CREATE_FILE: Self = Self(6);
    /// `kCmdWriteFile`: writes `<size>` bytes to `<offset>` in `<session>`.
    pub const WRITE_FILE: Self = Self(7);
    /// `kCmdRemoveFile`: remove file at `<path>`.
    pub const REMOVE_FILE: Self = Self(8);
    /// `kCmdCreateDirectory`: creates directory at `<path>`.
    pub const CREATE_DIRECTORY: Self = Self(9);
    /// `kCmdRemoveDirectory`: removes directory at `<path>`, must be empty.
    pub const REMOVE_DIRECTORY: Self = Self(10);
    /// `kCmdOpenFileWO`: opens file at `<path>` for writing, returns `<session>`.
    pub const OPEN_FILE_WO: Self = Self(11);
    /// `kCmdTruncateFile`: truncate file at `<path>` to `<offset>` length.
    pub const TRUNCATE_FILE: Self = Self(12);
    /// `kCmdRename`: rename path1 to path2.
    pub const RENAME: Self = Self(13);
    /// `kCmdCalcFileCRC32`: calculate CRC32 for file at `<path>`.
    pub const CALC_FILE_CRC32: Self = Self(14);
    /// `kCmdBurstReadFile`: burst download session file.
    pub const BURST_READ_FILE: Self = Self(15);
    /// `kCmdListDirectoryWithTime`: list files and directories in `<path>` from offset, each with
    /// its modification time.
    pub const LIST_DIRECTORY_WITH_TIME: Self = Self(16);
    /// `kRspAck`.
    pub const ACK: Self = Self(128);
    /// `kRspNak`.
    pub const NAK: Self = Self(129);

    /// The C# enum member's name, or `None` for a number the enum does not define.
    ///
    /// C#: MAVFtp.cs:500-558.
    #[must_use]
    pub const fn name(self) -> Option<&'static str> {
        Some(match self.0 {
            0 => "kCmdNone",
            1 => "kCmdTerminateSession",
            2 => "kCmdResetSessions",
            3 => "kCmdListDirectory",
            4 => "kCmdOpenFileRO",
            5 => "kCmdReadFile",
            6 => "kCmdCreateFile",
            7 => "kCmdWriteFile",
            8 => "kCmdRemoveFile",
            9 => "kCmdCreateDirectory",
            10 => "kCmdRemoveDirectory",
            11 => "kCmdOpenFileWO",
            12 => "kCmdTruncateFile",
            13 => "kCmdRename",
            14 => "kCmdCalcFileCRC32",
            15 => "kCmdBurstReadFile",
            16 => "kCmdListDirectoryWithTime",
            128 => "kRspAck",
            129 => "kRspNak",
            _ => return None,
        })
    }
}

impl fmt::Display for Opcode {
    /// As the C# enum's `ToString` prints it: the member's name, or the number if none.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "{}", self.0),
        }
    }
}

/// The error code in a NAK's first data byte: `FTPErrorCode`.
///
/// C#: MAVFtp.cs:466-497.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ErrorCode(pub u8);

impl ErrorCode {
    /// `kErrNone`.
    pub const NONE: Self = Self(0);
    /// `kErrFail`: unknown failure.
    pub const FAIL: Self = Self(1);
    /// `kErrFailErrno`: command failed, errno sent back in the second data byte.
    pub const FAIL_ERRNO: Self = Self(2);
    /// `kErrInvalidDataSize`: the header's size is invalid.
    pub const INVALID_DATA_SIZE: Self = Self(3);
    /// `kErrInvalidSession`: session is not currently open.
    pub const INVALID_SESSION: Self = Self(4);
    /// `kErrNoSessionsAvailable`: all available sessions in use.
    pub const NO_SESSIONS_AVAILABLE: Self = Self(5);
    /// `kErrEOF`: offset past end of file for list and read commands.
    pub const EOF: Self = Self(6);
    /// `kErrUnknownCommand`: unknown command opcode.
    pub const UNKNOWN_COMMAND: Self = Self(7);
    /// `kErrFailFileExists`: file exists already.
    pub const FAIL_FILE_EXISTS: Self = Self(8);
    /// `kErrFailFileProtected`: file is write protected.
    pub const FAIL_FILE_PROTECTED: Self = Self(9);
    /// `kErrFileNotFound`: file not found.
    pub const FILE_NOT_FOUND: Self = Self(10);

    /// The C# enum member's name, or `None` for a number the enum does not define.
    #[must_use]
    pub const fn name(self) -> Option<&'static str> {
        Some(match self.0 {
            0 => "kErrNone",
            1 => "kErrFail",
            2 => "kErrFailErrno",
            3 => "kErrInvalidDataSize",
            4 => "kErrInvalidSession",
            5 => "kErrNoSessionsAvailable",
            6 => "kErrEOF",
            7 => "kErrUnknownCommand",
            8 => "kErrFailFileExists",
            9 => "kErrFailFileProtected",
            10 => "kErrFileNotFound",
            _ => return None,
        })
    }
}

impl fmt::Display for ErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "{}", self.0),
        }
    }
}

/// The POSIX errno a `kErrFailErrno` NAK carries in its second data byte: the C#'s `errno`.
///
/// C#: MAVFtp.cs:60-463.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Errno(pub u8);

impl Errno {
    /// `ENOENT`: no such file or directory.
    pub const ENOENT: Self = Self(2);
    /// `EACCES`: permission denied.
    pub const EACCES: Self = Self(13);
    /// `EEXIST`: file exists.
    pub const EEXIST: Self = Self(17);
    /// `ENOTEMPTY`: directory not empty.
    pub const ENOTEMPTY: Self = Self(39);

    /// The C# enum member's name, or `None` for a number the enum does not define (41 and 58,
    /// which Linux leaves unused, and everything above 133).
    ///
    /// Three numbers have two members in the C# (`EWOULDBLOCK = 11`, `EDEADLOCK = 35`,
    /// `ENOTSUP = 95`, MAVFtp.cs:183, 234, 462). Which of the two `ToString` prints is the
    /// runtime's choice; this names the one mono prints for the C# build (`MpFtp.exe errno`).
    #[must_use]
    pub const fn name(self) -> Option<&'static str> {
        Some(match self.0 {
            1 => "EPERM",
            2 => "ENOENT",
            3 => "ESRCH",
            4 => "EINTR",
            5 => "EIO",
            6 => "ENXIO",
            7 => "E2BIG",
            8 => "ENOEXEC",
            9 => "EBADF",
            10 => "ECHILD",
            11 => ERRNO_11,
            12 => "ENOMEM",
            13 => "EACCES",
            14 => "EFAULT",
            15 => "ENOTBLK",
            16 => "EBUSY",
            17 => "EEXIST",
            18 => "EXDEV",
            19 => "ENODEV",
            20 => "ENOTDIR",
            21 => "EISDIR",
            22 => "EINVAL",
            23 => "ENFILE",
            24 => "EMFILE",
            25 => "ENOTTY",
            26 => "ETXTBSY",
            27 => "EFBIG",
            28 => "ENOSPC",
            29 => "ESPIPE",
            30 => "EROFS",
            31 => "EMLINK",
            32 => "EPIPE",
            33 => "EDOM",
            34 => "ERANGE",
            35 => ERRNO_35,
            36 => "ENAMETOOLONG",
            37 => "ENOLCK",
            38 => "ENOSYS",
            39 => "ENOTEMPTY",
            40 => "ELOOP",
            42 => "ENOMSG",
            43 => "EIDRM",
            44 => "ECHRNG",
            45 => "EL2NSYNC",
            46 => "EL3HLT",
            47 => "EL3RST",
            48 => "ELNRNG",
            49 => "EUNATCH",
            50 => "ENOCSI",
            51 => "EL2HLT",
            52 => "EBADE",
            53 => "EBADR",
            54 => "EXFULL",
            55 => "ENOANO",
            56 => "EBADRQC",
            57 => "EBADSLT",
            59 => "EBFONT",
            60 => "ENOSTR",
            61 => "ENODATA",
            62 => "ETIME",
            63 => "ENOSR",
            64 => "ENONET",
            65 => "ENOPKG",
            66 => "EREMOTE",
            67 => "ENOLINK",
            68 => "EADV",
            69 => "ESRMNT",
            70 => "ECOMM",
            71 => "EPROTO",
            72 => "EMULTIHOP",
            73 => "EDOTDOT",
            74 => "EBADMSG",
            75 => "EOVERFLOW",
            76 => "ENOTUNIQ",
            77 => "EBADFD",
            78 => "EREMCHG",
            79 => "ELIBACC",
            80 => "ELIBBAD",
            81 => "ELIBSCN",
            82 => "ELIBMAX",
            83 => "ELIBEXEC",
            84 => "EILSEQ",
            85 => "ERESTART",
            86 => "ESTRPIPE",
            87 => "EUSERS",
            88 => "ENOTSOCK",
            89 => "EDESTADDRREQ",
            90 => "EMSGSIZE",
            91 => "EPROTOTYPE",
            92 => "ENOPROTOOPT",
            93 => "EPROTONOSUPPORT",
            94 => "ESOCKTNOSUPPORT",
            95 => ERRNO_95,
            96 => "EPFNOSUPPORT",
            97 => "EAFNOSUPPORT",
            98 => "EADDRINUSE",
            99 => "EADDRNOTAVAIL",
            100 => "ENETDOWN",
            101 => "ENETUNREACH",
            102 => "ENETRESET",
            103 => "ECONNABORTED",
            104 => "ECONNRESET",
            105 => "ENOBUFS",
            106 => "EISCONN",
            107 => "ENOTCONN",
            108 => "ESHUTDOWN",
            109 => "ETOOMANYREFS",
            110 => "ETIMEDOUT",
            111 => "ECONNREFUSED",
            112 => "EHOSTDOWN",
            113 => "EHOSTUNREACH",
            114 => "EALREADY",
            115 => "EINPROGRESS",
            116 => "ESTALE",
            117 => "EUCLEAN",
            118 => "ENOTNAM",
            119 => "ENAVAIL",
            120 => "EISNAM",
            121 => "EREMOTEIO",
            122 => "EDQUOT",
            123 => "ENOMEDIUM",
            124 => "EMEDIUMTYPE",
            125 => "ECANCELED",
            126 => "ENOKEY",
            127 => "EKEYEXPIRED",
            128 => "EKEYREVOKED",
            129 => "EKEYREJECTED",
            130 => "EOWNERDEAD",
            131 => "ENOTRECOVERABLE",
            132 => "ERFKILL",
            133 => "EHWPOISON",
            _ => return None,
        })
    }
}

/// What mono's `Enum.ToString` prints for 11, which both `EAGAIN` and `EWOULDBLOCK` are.
const ERRNO_11: &str = "EAGAIN";
/// What mono's `Enum.ToString` prints for 35, which both `EDEADLK` and `EDEADLOCK` are.
const ERRNO_35: &str = "EDEADLOCK";
/// What mono's `Enum.ToString` prints for 95, which both `EOPNOTSUPP` and `ENOTSUP` are.
const ERRNO_95: &str = "EOPNOTSUPP";

impl fmt::Display for Errno {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "{}", self.0),
        }
    }
}

/// One `FILE_TRANSFER_PROTOCOL` payload, laid out as the C#'s `FTPPayloadHeader`.
///
/// C#: MAVFtp.cs:2363-2392. Little-endian and packed, as `MavlinkUtil.StructureToByteArray`
/// writes it on the little-endian machines Mission Planner runs on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    /// Sequence number for message.
    pub seq_number: u16,
    /// Session id for read and write commands.
    pub session: u8,
    /// Command opcode.
    pub opcode: Opcode,
    /// Size of data.
    pub size: u8,
    /// Request opcode returned in `kRspAck`, `kRspNak` message.
    pub req_opcode: Opcode,
    /// Only used if `req_opcode` is `kCmdBurstReadFile`: 1, set of burst packets complete; 0,
    /// more burst packets coming.
    pub burst_complete: u8,
    /// 32 bit alignment padding.
    pub padding: u8,
    /// Offsets for list and read commands.
    pub offset: u32,
    /// Command data, varies by opcode.
    pub data: [u8; DATA_LEN],
}

impl Default for Header {
    fn default() -> Self {
        Self {
            seq_number: 0,
            session: 0,
            opcode: Opcode::NONE,
            size: 0,
            req_opcode: Opcode::NONE,
            burst_complete: 0,
            padding: 0,
            offset: 0,
            data: [0; DATA_LEN],
        }
    }
}

impl Header {
    /// Sets `data` the way the C#'s conversion to bytes does when a request carries data: `size`
    /// becomes the data's length as a byte, and the data is cut or zero-padded to fit.
    ///
    /// C#: MAVFtp.cs:2394-2407 (`value.size = (byte)(value.data.Length);`,
    /// `value.data.MakeSize(251 - 12)`). The cast wraps, so a path of 256 bytes goes out saying
    /// it has none; nothing here stops it, because nothing in the C# does.
    pub fn set_data(&mut self, bytes: &[u8]) {
        #[allow(clippy::cast_possible_truncation)] // C#: (byte)(value.data.Length), which wraps
        {
            self.size = bytes.len() as u8;
        }
        self.data = [0; DATA_LEN];
        let n = bytes.len().min(DATA_LEN);
        if let (Some(to), Some(from)) = (self.data.get_mut(..n), bytes.get(..n)) {
            to.copy_from_slice(from);
        }
    }

    /// The first `size` bytes of `data`: what a reply says it carries.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        let n = usize::from(self.size).min(DATA_LEN);
        self.data.get(..n).unwrap_or(&[])
    }

    /// Data byte `index`, or zero past the end.
    #[must_use]
    pub fn data_byte(&self, index: usize) -> u8 {
        self.data.get(index).copied().unwrap_or(0)
    }

    /// The first four data bytes as a little-endian `u32`: `BitConverter.ToUInt32(data, 0)`.
    #[must_use]
    pub fn data_u32(&self) -> u32 {
        u32::from_le_bytes([
            self.data_byte(0),
            self.data_byte(1),
            self.data_byte(2),
            self.data_byte(3),
        ])
    }

    /// The payload's bytes.
    #[must_use]
    pub fn encode(&self) -> [u8; PAYLOAD_LEN] {
        let mut out = [0u8; PAYLOAD_LEN];
        let [s0, s1] = self.seq_number.to_le_bytes();
        let [o0, o1, o2, o3] = self.offset.to_le_bytes();
        let head = [
            s0,
            s1,
            self.session,
            self.opcode.0,
            self.size,
            self.req_opcode.0,
            self.burst_complete,
            self.padding,
            o0,
            o1,
            o2,
            o3,
        ];
        if let Some(to) = out.get_mut(..HEADER_LEN) {
            to.copy_from_slice(&head);
        }
        if let Some(to) = out.get_mut(HEADER_LEN..) {
            to.copy_from_slice(&self.data);
        }
        out
    }

    /// Reads a payload. Short input is zero-filled, as a MAVLink 2 frame's trailing zeros are.
    #[must_use]
    pub fn decode(bytes: &[u8]) -> Self {
        let at = |i: usize| bytes.get(i).copied().unwrap_or(0);
        let mut data = [0u8; DATA_LEN];
        for (i, slot) in data.iter_mut().enumerate() {
            *slot = at(HEADER_LEN + i);
        }
        Self {
            seq_number: u16::from_le_bytes([at(0), at(1)]),
            session: at(2),
            opcode: Opcode(at(3)),
            size: at(4),
            req_opcode: Opcode(at(5)),
            burst_complete: at(6),
            padding: at(7),
            offset: u32::from_le_bytes([at(8), at(9), at(10), at(11)]),
            data,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_header_is_packed_little_endian_in_the_structs_order() {
        let mut header = Header {
            seq_number: 0x0201,
            session: 3,
            opcode: Opcode::READ_FILE,
            size: 80,
            req_opcode: Opcode::OPEN_FILE_RO,
            burst_complete: 1,
            padding: 0,
            offset: 0x0807_0605,
            ..Header::default()
        };
        header.data[0] = 0xAA;
        header.data[DATA_LEN - 1] = 0xBB;
        let bytes = header.encode();
        assert_eq!(bytes[..12], [1, 2, 3, 5, 80, 4, 1, 0, 5, 6, 7, 8]);
        assert_eq!(bytes[12], 0xAA);
        assert_eq!(bytes[250], 0xBB);
        assert_eq!(Header::decode(&bytes), header);
    }

    #[test]
    fn data_sets_the_size_and_is_padded_or_cut_to_fit() {
        let mut header = Header::default();
        header.set_data(b"@SYS/uarts.txt");
        assert_eq!(header.size, 14);
        assert_eq!(header.payload(), b"@SYS/uarts.txt");
        assert!(header.data[14..].iter().all(|b| *b == 0));

        // Longer than the payload holds: cut to 239 bytes, and the size is the length as a byte.
        header.set_data(&[b'a'; 300]);
        assert_eq!(header.size, 44, "(byte)300 wraps, as the C#'s cast does");
        assert!(header.data.iter().all(|b| *b == b'a'));
    }

    #[test]
    fn a_truncated_payload_decodes_with_zeros() {
        // MAVLink 2 drops trailing zero bytes; the dialect zero-fills, and so does this.
        let header = Header::decode(&[7, 0, 0, 128, 4, 4, 0, 0]);
        assert_eq!(header.seq_number, 7);
        assert_eq!(header.opcode, Opcode::ACK);
        assert_eq!(header.req_opcode, Opcode::OPEN_FILE_RO);
        assert_eq!(header.offset, 0);
        assert_eq!(header.data_u32(), 0);
    }

    #[test]
    fn names_print_as_the_csharp_enums_do() {
        assert_eq!(Opcode::BURST_READ_FILE.to_string(), "kCmdBurstReadFile");
        assert_eq!(Opcode(17).to_string(), "17");
        assert_eq!(ErrorCode::FAIL_ERRNO.to_string(), "kErrFailErrno");
        assert_eq!(ErrorCode(11).to_string(), "11");
        assert_eq!(Errno::ENOENT.to_string(), "ENOENT");
        assert_eq!(Errno(41).to_string(), "41", "41 has no member");
        assert_eq!(Errno(58).to_string(), "58", "nor has 58");
        assert_eq!(Errno(133).to_string(), "EHWPOISON");
        assert_eq!(Errno(134).to_string(), "134");
    }
}
