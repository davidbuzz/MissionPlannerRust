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

//! `ZipFile.ExtractToDirectory`, for the GStreamer runtime [`crate::gstreamer::download_gstreamer`]
//! downloads: a zip archive's stored and deflated entries written under a directory.
//!
//! The container is read here; the inflating is the caller's (`inflate`), so this crate needs no
//! compression library of its own. An archive that needs Zip64, or an encrypted entry, is
//! refused, and an entry whose name would land outside the directory is refused as .NET refuses
//! it.

use std::path::{Component, Path, PathBuf};

/// The end of central directory record's signature.
const END_OF_CENTRAL_DIRECTORY: u32 = 0x0605_4b50;
/// A central directory entry's signature.
const CENTRAL_ENTRY: u32 = 0x0201_4b50;
/// A local file header's signature.
const LOCAL_HEADER: u32 = 0x0403_4b50;

/// Inflates raw deflate data: the data, and how long the result will be.
pub type Inflate<'a> = &'a dyn Fn(&[u8], usize) -> Result<Vec<u8>, String>;

/// One entry of the central directory.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    name: String,
    method: u16,
    flags: u16,
    compressed: usize,
    size: usize,
    local: usize,
}

fn u16_at(bytes: &[u8], at: usize) -> Result<u16, String> {
    bytes
        .get(at..at + 2)
        .and_then(|slice| slice.try_into().ok())
        .map(u16::from_le_bytes)
        .ok_or_else(|| "the archive is cut short".to_owned())
}

fn u32_at(bytes: &[u8], at: usize) -> Result<u32, String> {
    bytes
        .get(at..at + 4)
        .and_then(|slice| slice.try_into().ok())
        .map(u32::from_le_bytes)
        .ok_or_else(|| "the archive is cut short".to_owned())
}

/// A 32-bit field as a length or an offset; all ones means the value is in a Zip64 record.
fn size_at(bytes: &[u8], at: usize) -> Result<usize, String> {
    let value = u32_at(bytes, at)?;
    if value == u32::MAX {
        return Err("a Zip64 archive, which is not read".to_owned());
    }
    usize::try_from(value).map_err(|why| why.to_string())
}

/// The central directory's entries.
fn entries(bytes: &[u8]) -> Result<Vec<Entry>, String> {
    // The record is 22 bytes and a comment of up to 65,535; search back from the end.
    let earliest = bytes.len().saturating_sub(22 + usize::from(u16::MAX));
    let end = (earliest..=bytes.len().saturating_sub(22))
        .rev()
        .find(|at| u32_at(bytes, *at).is_ok_and(|sig| sig == END_OF_CENTRAL_DIRECTORY))
        .ok_or("not a zip archive")?;
    let count = u16_at(bytes, end + 10)?;
    let mut at = size_at(bytes, end + 16)?;
    let mut found = Vec::with_capacity(usize::from(count));
    for _ in 0..count {
        if u32_at(bytes, at)? != CENTRAL_ENTRY {
            return Err("a central directory entry is damaged".to_owned());
        }
        let name_length = usize::from(u16_at(bytes, at + 28)?);
        let extra_length = usize::from(u16_at(bytes, at + 30)?);
        let comment_length = usize::from(u16_at(bytes, at + 32)?);
        let name = bytes
            .get(at + 46..at + 46 + name_length)
            .ok_or("the archive is cut short")?;
        found.push(Entry {
            name: String::from_utf8_lossy(name).into_owned(),
            flags: u16_at(bytes, at + 8)?,
            method: u16_at(bytes, at + 10)?,
            compressed: size_at(bytes, at + 20)?,
            size: size_at(bytes, at + 24)?,
            local: size_at(bytes, at + 42)?,
        });
        at += 46 + name_length + extra_length + comment_length;
    }
    Ok(found)
}

/// Where an entry goes under `to`, or an error for a name that leaves it.
fn destination(to: &Path, name: &str) -> Result<PathBuf, String> {
    let relative = PathBuf::from(name.replace('\\', "/"));
    let mut out = to.to_path_buf();
    for component in relative.components() {
        match component {
            Component::Normal(part) => out.push(part),
            Component::CurDir => {}
            _ => return Err(format!("{name}: outside the directory being extracted to")),
        }
    }
    Ok(out)
}

/// `ZipFile.ExtractToDirectory(archive, to)`: every entry written under `to`, directories made
/// as needed. A file already there is replaced (.NET Framework's throws), so an extraction cut
/// short can be run again.
/// `// C#: ExtLibs/Utilities/GStreamer.cs:1524`
///
/// # Errors
///
/// A damaged or unsupported archive, an entry that will not inflate, or a write that fails.
pub fn extract(archive: &[u8], to: &Path, inflate: Inflate<'_>) -> Result<usize, String> {
    let mut written = 0;
    for entry in entries(archive)? {
        let path = destination(to, &entry.name)?;
        if entry.name.ends_with('/') || entry.name.ends_with('\\') {
            std::fs::create_dir_all(&path).map_err(|why| why.to_string())?;
            continue;
        }
        if entry.flags & 1 != 0 {
            return Err(format!("{}: encrypted", entry.name));
        }
        if u32_at(archive, entry.local)? != LOCAL_HEADER {
            return Err(format!("{}: its local header is damaged", entry.name));
        }
        let start = entry.local
            + 30
            + usize::from(u16_at(archive, entry.local + 26)?)
            + usize::from(u16_at(archive, entry.local + 28)?);
        let data = archive
            .get(start..start + entry.compressed)
            .ok_or("the archive is cut short")?;
        let contents = match entry.method {
            0 => data.to_vec(),
            8 => inflate(data, entry.size)?,
            other => return Err(format!("{}: compression method {other}", entry.name)),
        };
        if contents.len() != entry.size {
            return Err(format!("{}: the wrong length once inflated", entry.name));
        }
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|why| why.to_string())?;
        }
        std::fs::write(&path, contents).map_err(|why| format!("{}: {why}", path.display()))?;
        written += 1;
    }
    Ok(written)
}

#[cfg(test)]
pub(crate) mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::cast_possible_truncation
    )]

    use super::*;

    /// A zip archive of `files` (name, method, stored bytes, length once inflated), as a zip
    /// writer lays one out.
    pub(crate) fn archive(files: &[(&str, u16, &[u8], usize)]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut central = Vec::new();
        for (name, method, data, size) in files {
            let local = out.len() as u32;
            let mut header = Vec::new();
            header.extend_from_slice(&LOCAL_HEADER.to_le_bytes());
            header.extend_from_slice(&[20, 0, 0, 0]);
            header.extend_from_slice(&method.to_le_bytes());
            header.extend_from_slice(&[0; 8]); // time, date, crc
            header.extend_from_slice(&(data.len() as u32).to_le_bytes());
            header.extend_from_slice(&(*size as u32).to_le_bytes());
            header.extend_from_slice(&(name.len() as u16).to_le_bytes());
            header.extend_from_slice(&0u16.to_le_bytes());
            header.extend_from_slice(name.as_bytes());
            out.extend_from_slice(&header);
            out.extend_from_slice(data);
            central.extend_from_slice(&CENTRAL_ENTRY.to_le_bytes());
            central.extend_from_slice(&[20, 0, 20, 0, 0, 0]);
            central.extend_from_slice(&method.to_le_bytes());
            central.extend_from_slice(&[0; 8]);
            central.extend_from_slice(&(data.len() as u32).to_le_bytes());
            central.extend_from_slice(&(*size as u32).to_le_bytes());
            central.extend_from_slice(&(name.len() as u16).to_le_bytes());
            central.extend_from_slice(&[0; 12]); // extra, comment, disk, attributes
            central.extend_from_slice(&local.to_le_bytes());
            central.extend_from_slice(name.as_bytes());
        }
        let at = out.len() as u32;
        out.extend_from_slice(&central);
        out.extend_from_slice(&END_OF_CENTRAL_DIRECTORY.to_le_bytes());
        out.extend_from_slice(&[0; 4]);
        out.extend_from_slice(&(files.len() as u16).to_le_bytes());
        out.extend_from_slice(&(files.len() as u16).to_le_bytes());
        out.extend_from_slice(&(central.len() as u32).to_le_bytes());
        out.extend_from_slice(&at.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("mp-video-zip-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    /// A stand-in for inflating: the "compressed" bytes are the text reversed.
    fn reversing(data: &[u8], _size: usize) -> Result<Vec<u8>, String> {
        Ok(data.iter().rev().copied().collect())
    }

    #[test]
    fn stored_and_deflated_entries_are_written_under_the_directory() {
        let zip = archive(&[
            ("gstreamer/", 0, b"", 0),
            (
                "gstreamer/1.0/x86_64/bin/gst-launch-1.0.exe",
                0,
                b"MZ launcher",
                11,
            ),
            ("gstreamer\\1.0\\x86_64\\lib\\note.txt", 8, b"olleh", 5),
        ]);
        let to = scratch("ok");
        assert_eq!(extract(&zip, &to, &reversing).unwrap(), 2);
        assert_eq!(
            std::fs::read(to.join("gstreamer/1.0/x86_64/bin/gst-launch-1.0.exe")).unwrap(),
            b"MZ launcher"
        );
        assert_eq!(
            std::fs::read(to.join("gstreamer/1.0/x86_64/lib/note.txt")).unwrap(),
            b"hello"
        );
        // Again over the same files: replaced, not refused.
        assert_eq!(extract(&zip, &to, &reversing).unwrap(), 2);
        let _ = std::fs::remove_dir_all(&to);
    }

    #[test]
    fn a_name_outside_the_directory_is_refused() {
        let zip = archive(&[("../evil", 0, b"x", 1)]);
        let to = scratch("evil");
        let refused = extract(&zip, &to, &reversing).unwrap_err();
        assert!(refused.contains("outside"), "{refused}");
    }

    #[test]
    fn what_is_not_a_zip_is_said() {
        assert_eq!(
            extract(
                b"not a zip at all, not by a long way",
                Path::new("/nonexistent"),
                &reversing
            )
            .unwrap_err(),
            "not a zip archive"
        );
        let zip = archive(&[("a", 12, b"x", 1)]);
        assert!(
            extract(&zip, &scratch("bzip"), &reversing)
                .unwrap_err()
                .contains("compression method 12")
        );
    }
}
