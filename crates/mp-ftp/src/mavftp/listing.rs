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

//! A directory listing: what one `kCmdListDirectory` acknowledgement says, entry by entry.
//!
//! C#: ExtLibs/ArduPilot/Mavlink/MAVFtp.cs:1349-1411 (the ACK half of the listing handler),
//! :1238-1246 (`ParseListingTime`) and :2420-2452 (`FtpFileInfo`).
//!
//! Each entry is a type byte - `F`ile, `D`irectory or `S`kipped - and a NUL-terminated name. A file
//! is `name<TAB>size`, and in the timed listing both carry a third field, the modification time
//! in seconds since the Unix epoch. Names are read a byte to a character, as the C#'s
//! `(char) b` does: Latin-1, not UTF-8, so a name the vehicle stores in UTF-8 comes back as the
//! C# shows it.

use super::wire::Header;

/// `kDirentDir`. C#: MAVFtp.cs:23.
const DIRENT_DIR: u8 = b'D';
/// `kDirentFile`. C#: MAVFtp.cs:26.
const DIRENT_FILE: u8 = b'F';
/// `kDirentSkip`. C#: MAVFtp.cs:29.
const DIRENT_SKIP: u8 = b'S';

/// One entry of a listing: the C#'s `FtpFileInfo`.
///
/// C#: MAVFtp.cs:2420-2452.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FtpFileInfo {
    /// `Name`. Empty for an entry the vehicle skipped.
    pub name: String,
    /// `Parent`: the directory that was listed, as it was asked for.
    pub parent: String,
    /// `isDirectory`. A skipped entry counts as one (MAVFtp.cs:1396).
    pub is_directory: bool,
    /// `Size`, in bytes; zero for a directory.
    pub size: u64,
    /// `ModifiedUtc`, as seconds since the Unix epoch, or `None` if the vehicle did not say.
    pub modified_utc: Option<u32>,
    /// `FullPath`: the parent, a `/` unless it already ends in one, and the name.
    pub full_path: String,
}

impl FtpFileInfo {
    /// C#: MAVFtp.cs:2422-2431.
    #[must_use]
    pub fn new(
        name: String,
        parent: &str,
        is_directory: bool,
        size: u64,
        modified_utc: Option<u32>,
    ) -> Self {
        let full_path = if parent.ends_with('/') {
            format!("{parent}{name}")
        } else {
            format!("{parent}/{name}")
        };
        Self {
            name,
            parent: parent.to_owned(),
            is_directory,
            size,
            modified_utc,
            full_path,
        }
    }
}

impl std::fmt::Display for FtpFileInfo {
    /// C#: MAVFtp.cs:2446-2451 (`ToString`).
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.is_directory {
            write!(f, "Directory: {}", self.name)
        } else {
            write!(f, "File: {} {}", self.name, self.size)
        }
    }
}

/// Where the C#'s parse of an acknowledgement throws part way through it.
///
/// An entry with no size, a size that is not a number, or a name that runs off the end of the
/// payload without its NUL: `ulong.Parse` or an array index throws, `PacketReceived` swallows it
/// (C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5543-5550), and the handler has stopped
/// with whatever entries it had already added still added - and without asking for the next
/// offset, so the listing waits for its timeout and asks for the same one again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Malformed;

/// Appends the entries in one acknowledgement to `answer`.
///
/// C#: MAVFtp.cs:1349-1411. On [`Malformed`], the entries before the fault stay appended, as
/// they do in the C#.
pub(crate) fn parse_entries(
    head: &Header,
    dir: &str,
    with_time: bool,
    answer: &mut Vec<FtpFileInfo>,
) -> Result<(), Malformed> {
    let size = usize::from(head.size);
    // The C# indexes the 239-byte array, not the first `size` bytes of it, once inside a name.
    let byte = |at: usize| head.data.get(at).copied().ok_or(Malformed);
    // C#: while (b != 0x0) { b = ftphead.data[offset++]; if (b != 0x0) name.Append((char) b); }
    let read_name = |offset: &mut usize| -> Result<String, Malformed> {
        let mut name = String::new();
        loop {
            let b = byte(*offset)?;
            *offset += 1;
            if b == 0 {
                return Ok(name);
            }
            name.push(char::from(b));
        }
    };

    let mut offset = 0usize;
    while offset < size {
        let b = byte(offset)?;
        offset += 1;
        match b {
            DIRENT_FILE => {
                // C#: MAVFtp.cs:1358-1371.
                let name = read_name(&mut offset)?;
                let items: Vec<&str> = name.split('\t').collect();
                let size = items
                    .get(1)
                    .and_then(|field| parse_ulong(field))
                    .ok_or(Malformed)?;
                let modified = if with_time {
                    items.get(2).and_then(|field| parse_listing_time(field))
                } else {
                    None
                };
                let file = items.first().copied().unwrap_or_default().to_owned();
                answer.push(FtpFileInfo::new(file, dir, false, size, modified));
            }
            DIRENT_DIR => {
                // C#: MAVFtp.cs:1372-1389. A timed listing gives a directory the same trailing
                // fields as a file; a plain one is just the name, where a tab would be part of it.
                let name = read_name(&mut offset)?;
                if with_time {
                    let fields: Vec<&str> = name.split('\t').collect();
                    let modified = fields.get(2).and_then(|field| parse_listing_time(field));
                    let dirname = fields.first().copied().unwrap_or_default().to_owned();
                    answer.push(FtpFileInfo::new(dirname, dir, true, 0, modified));
                } else {
                    answer.push(FtpFileInfo::new(name, dir, true, 0, None));
                }
            }
            DIRENT_SKIP => {
                // C#: MAVFtp.cs:1390-1397. Kept, nameless, because the next request's offset is
                // the count of entries, and the vehicle counted this one.
                read_name(&mut offset)?;
                answer.push(FtpFileInfo::new(String::new(), dir, true, 0, None));
            }
            0 => {
                // C#: MAVFtp.cs:1398-1410 with `b` already zero: the loop does not run, the name
                // is empty, and nothing is added.
            }
            _ => {
                // C#: MAVFtp.cs:1398-1410. An unknown type byte: the name after it is a file,
                // the type byte itself not part of it.
                let name = read_name(&mut offset)?;
                if !name.is_empty() {
                    answer.push(FtpFileInfo::new(name, dir, false, 0, None));
                }
            }
        }
    }
    Ok(())
}

/// A listing's time field: seconds since the Unix epoch, or `None` for zero (the vehicle does not
/// know), for anything that is not a number, and for anything past a `u32`, which is a corrupt
/// entry.
///
/// C#: MAVFtp.cs:1240-1246.
pub(crate) fn parse_listing_time(seconds: &str) -> Option<u32> {
    let secs = parse_ulong(seconds)?;
    if secs == 0 {
        return None;
    }
    u32::try_from(secs).ok()
}

/// `ulong.Parse` and `ulong.TryParse` with .NET's default `NumberStyles.Integer`: white space
/// either side, an optional sign, and digits. A minus sign parses only zero.
fn parse_ulong(text: &str) -> Option<u64> {
    // .NET's white space for number parsing: U+0009 to U+000D and U+0020.
    let trimmed = text.trim_matches(|c: char| matches!(c, '\u{9}'..='\u{d}' | ' '));
    let (negative, digits) = match trimmed.as_bytes().first() {
        Some(b'+') => (false, trimmed.get(1..)?),
        Some(b'-') => (true, trimmed.get(1..)?),
        _ => (false, trimmed),
    };
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let value: u64 = digits.parse().ok()?;
    (!negative || value == 0).then_some(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ack(entries: &[u8]) -> Header {
        let mut head = Header::default();
        head.set_data(entries);
        head
    }

    #[test]
    fn files_directories_and_skips_parse_as_the_csharp_reads_them() {
        let head = ack(b"Dsubdir\0Fparam.pck\t1234\0S\0Fempty\t0\0");
        let mut answer = Vec::new();
        parse_entries(&head, "/APM", false, &mut answer).unwrap();
        assert_eq!(answer.len(), 4);
        assert_eq!(answer[0].to_string(), "Directory: subdir");
        assert_eq!(answer[0].full_path, "/APM/subdir");
        assert_eq!(answer[1].to_string(), "File: param.pck 1234");
        assert_eq!(answer[1].full_path, "/APM/param.pck");
        assert_eq!(answer[2].name, "");
        assert!(
            answer[2].is_directory,
            "a skip counts as a nameless directory"
        );
        assert_eq!(answer[3].size, 0);
    }

    #[test]
    fn a_plain_listing_keeps_a_tab_in_a_directory_name_and_a_timed_one_splits_it() {
        let head = ack(b"Dlogs\t0\t1700000000\0Fa.bin\t10\t1700000001\0");
        let mut plain = Vec::new();
        parse_entries(&head, "/", false, &mut plain).unwrap();
        assert_eq!(plain[0].name, "logs\t0\t1700000000");
        assert_eq!(plain[0].full_path, "/logs\t0\t1700000000");
        assert_eq!(plain[1].modified_utc, None, "a plain listing has no times");

        let mut timed = Vec::new();
        parse_entries(&head, "/", true, &mut timed).unwrap();
        assert_eq!(timed[0].name, "logs");
        assert_eq!(timed[0].modified_utc, Some(1_700_000_000));
        assert_eq!(timed[1].modified_utc, Some(1_700_000_001));
    }

    #[test]
    fn a_time_of_zero_or_past_a_u32_or_not_a_number_is_no_time() {
        assert_eq!(parse_listing_time("0"), None);
        assert_eq!(parse_listing_time("4294967296"), None);
        assert_eq!(parse_listing_time("4294967295"), Some(u32::MAX));
        assert_eq!(parse_listing_time("soon"), None);
        assert_eq!(
            parse_listing_time(" 12 "),
            Some(12),
            "NumberStyles.Integer trims"
        );
        assert_eq!(parse_listing_time("+7"), Some(7));
    }

    #[test]
    fn an_unknown_type_byte_names_a_file_without_itself() {
        // C#'s default branch reads the name after the byte it switched on.
        let head = ack(b"Xabc\0");
        let mut answer = Vec::new();
        parse_entries(&head, "/", false, &mut answer).unwrap();
        assert_eq!(answer.len(), 1);
        assert_eq!(answer[0].name, "abc");
        assert!(!answer[0].is_directory);
    }

    #[test]
    fn a_file_without_a_size_stops_the_parse_with_what_came_before_kept() {
        // ulong.Parse(items[1]) throws; PacketReceived swallows it; the directory before it is
        // already in the answer.
        let head = ack(b"Dkept\0Fnosize\0Flater\t1\0");
        let mut answer = Vec::new();
        assert_eq!(
            parse_entries(&head, "/", false, &mut answer),
            Err(Malformed)
        );
        assert_eq!(answer.len(), 1);
        assert_eq!(answer[0].name, "kept");
    }

    #[test]
    fn names_are_latin1_bytes_as_the_csharp_casts_them() {
        // "é" in UTF-8 is C3 A9; (char) b makes two characters of it.
        let head = ack(b"F\xc3\xa9\t1\0");
        let mut answer = Vec::new();
        parse_entries(&head, "/", false, &mut answer).unwrap();
        assert_eq!(answer[0].name, "\u{c3}\u{a9}");
    }

    #[test]
    fn a_name_that_runs_off_the_payload_is_malformed() {
        let mut head = Header {
            data: [b'a'; super::super::wire::DATA_LEN],
            size: 10,
            ..Header::default()
        };
        head.data[0] = DIRENT_FILE;
        let mut answer = Vec::new();
        assert_eq!(
            parse_entries(&head, "/", false, &mut answer),
            Err(Malformed)
        );
        assert!(answer.is_empty());
    }

    #[test]
    fn a_root_parent_is_not_doubled_in_the_full_path() {
        let info = FtpFileInfo::new("APM".to_owned(), "/", true, 0, None);
        assert_eq!(info.full_path, "/APM");
        let info = FtpFileInfo::new("threads.txt".to_owned(), "@SYS", false, 1, None);
        assert_eq!(info.full_path, "@SYS/threads.txt");
    }
}
