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

//! Just enough of the zip format for the DataFlash Logs page: writing the `.kmz` "Create KML +
//! gpx" leaves (SharpZipLib's `ZipOutputStream`, `LogOutput.cs:1103-1155`), and unpacking the
//! analyzer "Auto Analysis" downloads (SharpZipLib's `FastZip.ExtractZip`,
//! `Utilities/LogAnalyzer.cs:58-59`).
//!
//! Entries are stored or deflated; there is no zip64, no encryption and no spanning, none of which
//! either file uses. A name that would land outside the directory it is extracted to is refused.

use std::io::{Read, Write};
use std::path::{Component, Path};

/// One file in an archive.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entry {
    /// Its name, `/`-separated.
    pub name: String,
    /// Its contents.
    pub data: Vec<u8>,
}

/// What is wrong with an archive.
#[derive(Debug, thiserror::Error)]
pub enum ZipError {
    /// Not a zip, or one cut short.
    #[error("not a zip archive, or a damaged one")]
    Malformed,
    /// A compression method other than store and deflate.
    #[error("entry {0} uses a compression method this reader does not have")]
    Method(String),
    /// A name with `..` or a root in it.
    #[error("entry {0} would be written outside the directory")]
    Unsafe(String),
    /// Reading or writing failed.
    #[error("{0}")]
    Io(#[from] std::io::Error),
}

/// The most deflate can expand its input, from RFC 1951's smallest encoding of a stored block:
/// 1032 bytes out of one byte in. A declared uncompressed size above this many times the
/// compressed bytes is a lie, whatever else it is.
const DEFLATE_MAX_EXPANSION: usize = 1032;

fn u16_at(data: &[u8], at: usize) -> Result<u16, ZipError> {
    data.get(at..at + 2)
        .and_then(|b| b.try_into().ok())
        .map(u16::from_le_bytes)
        .ok_or(ZipError::Malformed)
}

fn u32_at(data: &[u8], at: usize) -> Result<u32, ZipError> {
    data.get(at..at + 4)
        .and_then(|b| b.try_into().ok())
        .map(u32::from_le_bytes)
        .ok_or(ZipError::Malformed)
}

fn usize_at(data: &[u8], at: usize) -> Result<usize, ZipError> {
    usize::try_from(u32_at(data, at)?).map_err(|_| ZipError::Malformed)
}

/// Every entry of an archive, in central-directory order, decompressed.
///
/// # Errors
///
/// The archive is damaged or uses something this reader does not have.
pub fn read(data: &[u8]) -> Result<Vec<Entry>, ZipError> {
    // The end-of-central-directory record: at the end, before a comment of at most 64 KiB.
    let lowest = data.len().saturating_sub(22 + 0xFFFF);
    let end = (lowest..=data.len().saturating_sub(22))
        .rev()
        .find(|&at| u32_at(data, at).ok() == Some(0x0605_4b50))
        .ok_or(ZipError::Malformed)?;
    let count = usize::from(u16_at(data, end + 10)?);
    let mut at = usize_at(data, end + 16)?;
    let mut entries = Vec::with_capacity(count);
    for _ in 0..count {
        if u32_at(data, at)? != 0x0201_4b50 {
            return Err(ZipError::Malformed);
        }
        let method = u16_at(data, at + 10)?;
        let compressed = usize_at(data, at + 20)?;
        let size = usize_at(data, at + 24)?;
        let name_len = usize::from(u16_at(data, at + 28)?);
        let extra_len = usize::from(u16_at(data, at + 30)?);
        let comment_len = usize::from(u16_at(data, at + 32)?);
        let local = usize_at(data, at + 42)?;
        let name_bytes = data
            .get(at + 46..at + 46 + name_len)
            .ok_or(ZipError::Malformed)?;
        let name = String::from_utf8_lossy(name_bytes).into_owned();
        at += 46 + name_len + extra_len + comment_len;

        if u32_at(data, local)? != 0x0403_4b50 {
            return Err(ZipError::Malformed);
        }
        let start = local
            + 30
            + usize::from(u16_at(data, local + 26)?)
            + usize::from(u16_at(data, local + 28)?);
        let raw = data
            .get(start..start + compressed)
            .ok_or(ZipError::Malformed)?;
        let contents = match method {
            0 => raw.to_vec(),
            8 => {
                // `size` is the archive's claim about the uncompressed length, not a measurement,
                // and reserving it trusts a stranger with this process's memory: a 696-byte
                // archive declaring 3 GiB for an entry with no compressed bytes at all made this
                // reserve 3 GiB, which is how the fuzzer ran out of memory (its `zip_archive`
                // target, CI run 37116316968, 2026-10-03). Deflate cannot expand by more than
                // 1032:1, so the claim is believed only as far as these compressed bytes could
                // stretch; `read_to_end` grows the buffer if the truth is larger.
                let ceiling = compressed.saturating_mul(DEFLATE_MAX_EXPANSION);
                let mut out = Vec::with_capacity(size.min(ceiling));
                flate2::read::DeflateDecoder::new(raw).read_to_end(&mut out)?;
                out
            }
            _ => return Err(ZipError::Method(name)),
        };
        entries.push(Entry {
            name,
            data: contents,
        });
    }
    Ok(entries)
}

/// `FastZip.ExtractZip(zip, dir, "")`: every entry written under `dir`, directories made as
/// needed, existing files overwritten.
///
/// # Errors
///
/// The archive is damaged, an entry's name leaves `dir`, or a file cannot be written.
pub fn extract(data: &[u8], dir: &Path) -> Result<(), ZipError> {
    for entry in read(data)? {
        let relative = Path::new(&entry.name);
        if relative
            .components()
            .any(|c| !matches!(c, Component::Normal(_) | Component::CurDir))
        {
            return Err(ZipError::Unsafe(entry.name));
        }
        let target = dir.join(relative);
        if entry.name.ends_with('/') {
            std::fs::create_dir_all(&target)?;
            continue;
        }
        if let Some(parent) = target.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(target, entry.data)?;
    }
    Ok(())
}

/// A local time as MS-DOS packs it: seconds halved, years from 1980.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DosTime {
    /// Year, 1980 to 2107.
    pub year: u16,
    /// Month, 1 to 12.
    pub month: u16,
    /// Day, 1 to 31.
    pub day: u16,
    /// Hour.
    pub hour: u16,
    /// Minute.
    pub minute: u16,
    /// Second.
    pub second: u16,
}

impl DosTime {
    fn packed(self) -> (u16, u16) {
        let time = (self.hour << 11) | (self.minute << 5) | (self.second / 2);
        let date = (self.year.saturating_sub(1980) << 9) | (self.month << 5) | self.day;
        (time, date)
    }
}

/// A zip of `entries`, each deflated at the best compression (`SetLevel(9)`), every one stamped
/// `modified`; no zip64 (`UseZip64.Off`).
///
/// # Errors
///
/// Deflating failed, or an entry is too large for a zip without zip64.
pub fn write(entries: &[Entry], modified: DosTime) -> Result<Vec<u8>, ZipError> {
    let (time, date) = modified.packed();
    let mut out = Vec::new();
    let mut central = Vec::new();
    for entry in entries {
        let mut encoder =
            flate2::write::DeflateEncoder::new(Vec::new(), flate2::Compression::best());
        encoder.write_all(&entry.data)?;
        let compressed = encoder.finish()?;
        let mut crc = flate2::Crc::new();
        crc.update(&entry.data);
        let too_large = || ZipError::Io(std::io::Error::other("entry too large for a zip"));
        let size = u32::try_from(entry.data.len()).map_err(|_| too_large())?;
        let packed = u32::try_from(compressed.len()).map_err(|_| too_large())?;
        let name_len = u16::try_from(entry.name.len()).map_err(|_| too_large())?;
        let offset = u32::try_from(out.len()).map_err(|_| too_large())?;
        let common = |header: &mut Vec<u8>| {
            header.extend(20u16.to_le_bytes()); // version needed: 2.0
            header.extend(0u16.to_le_bytes()); // flags
            header.extend(8u16.to_le_bytes()); // deflate
            header.extend(time.to_le_bytes());
            header.extend(date.to_le_bytes());
            header.extend(crc.sum().to_le_bytes());
            header.extend(packed.to_le_bytes());
            header.extend(size.to_le_bytes());
            header.extend(name_len.to_le_bytes());
            header.extend(0u16.to_le_bytes()); // extra
        };
        out.extend(0x0403_4b50u32.to_le_bytes());
        common(&mut out);
        out.extend(entry.name.as_bytes());
        out.extend(&compressed);

        central.extend(0x0201_4b50u32.to_le_bytes());
        central.extend(20u16.to_le_bytes()); // version made by
        common(&mut central);
        central.extend(0u16.to_le_bytes()); // comment
        central.extend(0u16.to_le_bytes()); // disk
        central.extend(0u16.to_le_bytes()); // internal attributes
        central.extend(0u32.to_le_bytes()); // external attributes
        central.extend(offset.to_le_bytes());
        central.extend(entry.name.as_bytes());
    }
    let count = u16::try_from(entries.len()).unwrap_or(u16::MAX);
    let directory_at = u32::try_from(out.len()).unwrap_or(u32::MAX);
    let directory_len = u32::try_from(central.len()).unwrap_or(u32::MAX);
    out.extend(central);
    out.extend(0x0605_4b50u32.to_le_bytes());
    out.extend(0u16.to_le_bytes());
    out.extend(0u16.to_le_bytes());
    out.extend(count.to_le_bytes());
    out.extend(count.to_le_bytes());
    out.extend(directory_len.to_le_bytes());
    out.extend(directory_at.to_le_bytes());
    out.extend(0u16.to_le_bytes());
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn what_is_written_reads_back() {
        let entries = vec![
            Entry {
                name: "a.kml".to_owned(),
                data: b"<kml/>".repeat(100),
            },
            Entry {
                name: "b.dae".to_owned(),
                data: Vec::new(),
            },
        ];
        let when = DosTime {
            year: 2026,
            month: 9,
            day: 24,
            hour: 5,
            minute: 30,
            second: 12,
        };
        let zip = write(&entries, when).unwrap();
        assert_eq!(read(&zip).unwrap(), entries);
    }

    /// An archive's declared uncompressed size is not a reason to reserve that much memory.
    ///
    /// The fuzzer's `zip_archive` target found a 696-byte archive declaring 3 GiB for an entry
    /// with no compressed bytes, and this reader reserved it (CI run 37116316968, 2026-10-03).
    /// The entry still reads as what its bytes say, and the buffer it comes in stays small.
    #[test]
    fn a_declared_size_the_compressed_bytes_cannot_hold_is_not_reserved() {
        let entries = vec![Entry {
            name: "doc.kml".to_owned(),
            data: b"<kml>a</kml>".to_vec(),
        }];
        let when = DosTime {
            year: 2026,
            month: 10,
            day: 3,
            hour: 0,
            minute: 0,
            second: 0,
        };
        let mut zip = write(&entries, when).unwrap();
        // The uncompressed size, in the central directory (at + 24) and the local header (+ 22),
        // set to 3 GiB - the fuzzer's claim - leaving every length that bounds a slice alone.
        let huge = 3_225_747_584u32.to_le_bytes();
        let end = (0..=zip.len() - 22)
            .rev()
            .find(|&at| zip[at..at + 4] == [0x50, 0x4b, 0x05, 0x06])
            .expect("the end record");
        let central = usize_at(&zip, end + 16).unwrap();
        let local = usize_at(&zip, central + 42).unwrap();
        zip[central + 24..central + 28].copy_from_slice(&huge);
        zip[local + 22..local + 26].copy_from_slice(&huge);

        let read_back = read(&zip).unwrap();
        assert_eq!(read_back, entries, "the entry is what its bytes say");
        assert!(
            read_back[0].data.capacity() < 1 << 20,
            "reserved {} bytes for a 12-byte entry",
            read_back[0].data.capacity()
        );
    }

    /// A stored entry and a directory, as `zip -0` writes them.
    #[test]
    fn stored_entries_and_directories_extract() {
        let mut zip = Vec::new();
        let mut central = Vec::new();
        for (name, data) in [("dir/", &b""[..]), ("dir/runner.exe", &b"MZ"[..])] {
            let mut crc = flate2::Crc::new();
            crc.update(data);
            let offset = u32::try_from(zip.len()).unwrap();
            let header = |out: &mut Vec<u8>| {
                out.extend(10u16.to_le_bytes());
                out.extend(0u16.to_le_bytes());
                out.extend(0u16.to_le_bytes());
                out.extend([0u8; 4]);
                out.extend(crc.sum().to_le_bytes());
                let size = u32::try_from(data.len()).unwrap();
                out.extend(size.to_le_bytes());
                out.extend(size.to_le_bytes());
                out.extend(u16::try_from(name.len()).unwrap().to_le_bytes());
                out.extend(0u16.to_le_bytes());
            };
            zip.extend(0x0403_4b50u32.to_le_bytes());
            header(&mut zip);
            zip.extend(name.as_bytes());
            zip.extend(data);
            central.extend(0x0201_4b50u32.to_le_bytes());
            central.extend(10u16.to_le_bytes());
            header(&mut central);
            central.extend([0u8; 10]);
            central.extend(offset.to_le_bytes());
            central.extend(name.as_bytes());
        }
        let at = u32::try_from(zip.len()).unwrap();
        let len = u32::try_from(central.len()).unwrap();
        zip.extend(central);
        zip.extend(0x0605_4b50u32.to_le_bytes());
        zip.extend([0u8; 4]);
        zip.extend(2u16.to_le_bytes());
        zip.extend(2u16.to_le_bytes());
        zip.extend(len.to_le_bytes());
        zip.extend(at.to_le_bytes());
        zip.extend(0u16.to_le_bytes());

        let dir = std::env::temp_dir().join(format!("mp-log-zip-{}", std::process::id()));
        extract(&zip, &dir).unwrap();
        assert_eq!(std::fs::read(dir.join("dir/runner.exe")).unwrap(), b"MZ");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_name_that_climbs_out_is_refused() {
        let entries = vec![Entry {
            name: "../evil".to_owned(),
            data: b"x".to_vec(),
        }];
        let zip = write(
            &entries,
            DosTime {
                year: 2026,
                month: 1,
                day: 1,
                hour: 0,
                minute: 0,
                second: 0,
            },
        )
        .unwrap();
        assert!(matches!(
            extract(&zip, Path::new("/nonexistent")),
            Err(ZipError::Unsafe(_))
        ));
    }
}
