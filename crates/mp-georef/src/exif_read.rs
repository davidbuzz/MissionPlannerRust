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

//! A photo's shutter time as `getPhotoTime` reads it: Drew Noakes' metadata-extractor 2.4.0 as
//! ported to C# (`ExtLibs/MetaDataExtractorCSharp240d`), reading the first APP1 segment of a JPEG
//! or the whole of a TIFF as EXIF, and the first directory holding `DateTimeOriginal` (0x9003) or
//! `DateTimeDigitized` (0x9004) answering.
//!
//! Only what decides that answer is kept: which directories exist and in what order, which tags
//! each holds and whether the date tags hold text. That still means the reader's structure whole -
//! the IFD walk with its bounds checks and its 32-bit arithmetic, the maker-note dispatch (a maker
//! note can hold tags of any number, and an exception in it ends the walk), `ReadCommentString`'s
//! habit of writing into the segment it reads - because every one of them changes which tags are
//! read before an exception stops the reader.
//!
//! The date text itself goes through `AbstractDirectory.GetDate`: `DateTime.TryParse` in the
//! current culture, then seven exact formats, then *today* - so a photo whose date nothing parses is
//! dated today at midnight.

use std::collections::HashMap;

use crate::time::{DateTime, Kind};

/// The tags `getPhotoTime` asks for. `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:92-110`
pub const TAG_DATETIME_ORIGINAL: i32 = 0x9003;
/// `DateTimeDigitized`.
pub const TAG_DATETIME_DIGITIZED: i32 = 0x9004;

const TAG_EXIF_OFFSET: i32 = 0x8769;
const TAG_INTEROP_OFFSET: i32 = 0xA005;
const TAG_GPS_INFO_OFFSET: i32 = 0x8825;
const TAG_MAKER_NOTE: i32 = 0x927C;
const TAG_USER_COMMENT: i32 = 0x9286;
const TAG_MAKE: i32 = 0x010F;

/// `BYTES_PER_FORMAT`. `// C#: com/drew/metadata/exif/ExifReader.cs:39`
const BYTES_PER_FORMAT: [i32; 13] = [0, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8];

/// The directory classes, each `Metadata.GetDirectory` creates on first use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Directory {
    /// `ExifDirectory`: IFD0, the Exif IFD and every IFD chained after them, merged.
    Exif,
    /// `ExifInteropDirectory`.
    Interop,
    /// `GpsDirectory`.
    Gps,
    /// A maker note's directory, by its class name.
    MakerNote(&'static str),
}

/// A tag's value as far as `GetDate` cares: text, or anything else.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TagValue {
    /// A `string`.
    Text(String),
    /// Bytes, numbers, rationals.
    Other,
}

/// `Metadata`: its directories in the order they were created, each with its tags.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Metadata {
    /// The directories and their tags.
    pub directories: Vec<(Directory, HashMap<i32, TagValue>)>,
}

impl Metadata {
    /// `GetDirectory`: the directory, created at the end if new.
    fn directory(&mut self, which: Directory) -> usize {
        if let Some(at) = self.directories.iter().position(|(d, _)| *d == which) {
            at
        } else {
            self.directories.push((which, HashMap::new()));
            self.directories.len() - 1
        }
    }

    fn set(&mut self, directory: usize, tag: i32, value: TagValue) {
        if let Some((_, tags)) = self.directories.get_mut(directory) {
            tags.insert(tag, value);
        }
    }

    /// `getPhotoTime`'s search: the first directory holding `DateTimeOriginal`, else in the same
    /// directory `DateTimeDigitized`, and that tag's value. `None` when no directory has either.
    /// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:90-111`
    #[must_use]
    pub fn date_tag(&self) -> Option<&TagValue> {
        for (_, tags) in &self.directories {
            if let Some(value) = tags.get(&TAG_DATETIME_ORIGINAL) {
                return Some(value);
            }
            if let Some(value) = tags.get(&TAG_DATETIME_DIGITIZED) {
                return Some(value);
            }
        }
        None
    }
}

/// An exception inside the reader: a read past the segment's end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct OutOfSegment;

/// Why a file gives no metadata at all.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ReadError {
    /// `JpegProcessingException`: not a JPEG, or its segments do not fit the file.
    #[error("{0}")]
    Jpeg(&'static str),
    /// `TiffProcessingException`: the walk threw.
    #[error("tiff")]
    Tiff,
}

/// `ExifReader` over one segment.
struct ExifReader {
    data: Vec<u8>,
    motorola: bool,
}

impl ExifReader {
    fn len(&self) -> i32 {
        i32::try_from(self.data.len()).unwrap_or(i32::MAX)
    }

    fn byte(&self, at: i32) -> Result<u8, OutOfSegment> {
        usize::try_from(at)
            .ok()
            .and_then(|at| self.data.get(at))
            .copied()
            .ok_or(OutOfSegment)
    }

    /// `Get16Bits`. `// C#: com/drew/metadata/exif/ExifReader.cs:876-895`
    fn get16(&self, at: i32) -> Result<i32, OutOfSegment> {
        let (a, b) = (self.byte(at)?, self.byte(at.wrapping_add(1))?);
        Ok(if self.motorola {
            i32::from(a) << 8 | i32::from(b)
        } else {
            i32::from(b) << 8 | i32::from(a)
        })
    }

    /// `Get32Bits`, as a signed `int`. `// C#: com/drew/metadata/exif/ExifReader.cs:897-926`
    fn get32(&self, at: i32) -> Result<i32, OutOfSegment> {
        let b = [
            self.byte(at)?,
            self.byte(at.wrapping_add(1))?,
            self.byte(at.wrapping_add(2))?,
            self.byte(at.wrapping_add(3))?,
        ];
        Ok(if self.motorola {
            i32::from_be_bytes(b)
        } else {
            i32::from_le_bytes(b)
        })
    }

    /// `Utils.Decode(data, offset, length, false)`: each byte a character, `\r` a space.
    fn decode(&self, offset: i32, length: i32) -> Result<String, OutOfSegment> {
        let mut out = String::new();
        let mut i = offset;
        while i < offset.wrapping_add(length) {
            let b = self.byte(i)?;
            out.push(if b == b'\r' { ' ' } else { char::from(b) });
            i += 1;
        }
        Ok(out)
    }

    fn set_byte_order(&mut self, identifier: &str) -> bool {
        match identifier {
            "MM" => {
                self.motorola = true;
                true
            }
            "II" => {
                self.motorola = false;
                true
            }
            _ => false,
        }
    }

    /// `Extract`: an APP1 segment. Exceptions leave what was read.
    /// `// C#: com/drew/metadata/exif/ExifReader.cs:111-166`
    fn extract(&mut self, metadata: &mut Metadata) -> Result<(), OutOfSegment> {
        let exif = metadata.directory(Directory::Exif);
        if self.data.len() <= 14 {
            return Ok(());
        }
        if self.decode(0, 6)? != "Exif\0\0" {
            return Ok(());
        }
        let order = self.decode(6, 2)?;
        if !self.set_byte_order(&order) {
            return Ok(());
        }
        if self.get16(8)? != 0x2a {
            return Ok(());
        }
        let mut first = self.get32(10)?.wrapping_add(6);
        if first >= self.len() - 1 {
            first = 14;
        }
        let mut processed = Vec::new();
        self.process_directory(metadata, exif, &mut processed, first, 6)
    }

    /// `ExtractTiff`: a whole TIFF file, the header at 0 and its magic number not checked.
    /// `// C#: com/drew/metadata/exif/ExifReader.cs:60-110`
    fn extract_tiff(&mut self, metadata: &mut Metadata) -> Result<(), OutOfSegment> {
        let exif = metadata.directory(Directory::Exif);
        let order = self.decode(0, 2)?;
        if !self.set_byte_order(&order) {
            return Ok(());
        }
        self.get16(2)?;
        let mut first = self.get32(4)?;
        if first >= self.len() - 1 {
            first = 14;
        }
        let mut processed = Vec::new();
        self.process_directory(metadata, exif, &mut processed, first, 0)
    }

    /// `ProcessDirectory`. `// C#: com/drew/metadata/exif/ExifReader.cs:241-343`
    fn process_directory(
        &mut self,
        metadata: &mut Metadata,
        directory: usize,
        processed: &mut Vec<i32>,
        start: i32,
        tiff: i32,
    ) -> Result<(), OutOfSegment> {
        if processed.contains(&start) {
            return Ok(());
        }
        processed.push(start);
        if start >= self.len() || start < 0 {
            return Ok(());
        }
        // `IsDirectoryLengthValid`, which counts the TIFF header's offset twice.
        let count = self.get16(start)?;
        let dir_length = 2 + 12 * count + 4;
        if dir_length.wrapping_add(start).wrapping_add(tiff) >= self.len() {
            return Ok(());
        }
        for number in 0..count {
            let entry = start + 2 + 12 * number;
            let tag = self.get16(entry)?;
            let format = self.get16(entry + 2)?;
            if !(1..=12).contains(&format) {
                continue;
            }
            let components = self.get32(entry + 4)?;
            if components < 0 {
                continue;
            }
            let per = BYTES_PER_FORMAT
                .get(usize::try_from(format).unwrap_or(0))
                .copied()
                .unwrap_or(0);
            let byte_count = components.wrapping_mul(per);
            // `CalculateTagValueOffset`.
            let value_at = if byte_count > 4 {
                let offset = self.get32(entry + 8)?;
                if offset.wrapping_add(byte_count) > self.len() {
                    -1
                } else {
                    tiff.wrapping_add(offset)
                }
            } else {
                entry + 8
            };
            if value_at < 0 || value_at > self.len() {
                continue;
            }
            if byte_count < 0 || value_at.wrapping_add(byte_count) > self.len() {
                continue;
            }
            let subdir = tiff.wrapping_add(self.get32(value_at)?);
            match tag {
                TAG_EXIF_OFFSET => {
                    let exif = metadata.directory(Directory::Exif);
                    self.process_directory(metadata, exif, processed, subdir, tiff)?;
                }
                TAG_INTEROP_OFFSET => {
                    let interop = metadata.directory(Directory::Interop);
                    self.process_directory(metadata, interop, processed, subdir, tiff)?;
                }
                TAG_GPS_INFO_OFFSET => {
                    let gps = metadata.directory(Directory::Gps);
                    self.process_directory(metadata, gps, processed, subdir, tiff)?;
                }
                TAG_MAKER_NOTE => self.process_maker_note(metadata, value_at, processed, tiff)?,
                _ => {
                    let value = self.process_tag(tag, value_at, components, format)?;
                    metadata.set(directory, tag, value);
                }
            }
        }
        let next_at = start + 2 + 12 * count;
        let mut next = self.get32(next_at)?;
        if next != 0 {
            next = next.wrapping_add(tiff);
            if next >= self.len() || next < start {
                return Ok(());
            }
            self.process_directory(metadata, directory, processed, next, tiff)?;
        }
        Ok(())
    }

    /// `ProcessMakerNote`: which vendor's directory a maker note is, and where it starts - and
    /// the `NullReferenceException` of a camera with no `Make` whose note is none of the first
    /// three kinds tested.
    /// `// C#: com/drew/metadata/exif/ExifReader.cs:344-506`
    fn process_maker_note(
        &mut self,
        metadata: &mut Metadata,
        at: i32,
        processed: &mut Vec<i32>,
        tiff: i32,
    ) -> Result<(), OutOfSegment> {
        let exif = metadata.directory(Directory::Exif);
        let model = match metadata
            .directories
            .get(exif)
            .and_then(|(_, tags)| tags.get(&TAG_MAKE))
        {
            Some(TagValue::Text(text)) => Some(text.clone()),
            // `GetString` of anything else is its `ToString()`, which is never one of the names
            // tested for.
            Some(TagValue::Other) => Some(String::new()),
            None => None,
        };
        let two = self.decode(at, 2)?;
        let three = self.decode(at, 3)?;
        let four = self.decode(at, 4)?;
        let five = self.decode(at, 5)?;
        let six = self.decode(at, 6)?;
        let seven = self.decode(at, 7)?;
        let eight = self.decode(at, 8)?;
        let upper = model.as_deref().map(str::to_uppercase);
        let (which, start, base) = if five == "OLYMP" || five == "EPSON" || four == "AGFA" {
            ("OlympusDirectory", at + 8, tiff)
        } else if model
            .as_deref()
            .is_some_and(|m| m.trim().to_uppercase().starts_with("NIKON"))
        {
            if self.decode(at, 5)? == "Nikon" {
                match self.byte(at + 6)? {
                    1 => ("NikonType1Directory", at + 8, tiff),
                    2 => ("NikonType2Directory", at + 18, at + 10),
                    _ => return Ok(()),
                }
            } else {
                ("NikonType2Directory", at, tiff)
            }
        } else if eight == "SONY CAM" || eight == "SONY DSC" {
            ("SonyDirectory", at + 12, tiff)
        } else if three == "KDK" {
            ("KodakDirectory", at + 20, tiff)
        } else {
            // `"Canon".ToUpper().Equals(cameraModel.ToUpper())` throws with no model.
            let Some(upper) = upper else {
                return Err(OutOfSegment);
            };
            if upper == "CANON" {
                ("CanonDirectory", at, tiff)
            } else if upper.starts_with("CASIO") {
                if six == "QVC\0\0\0" {
                    ("CasioType2Directory", at + 6, tiff)
                } else {
                    ("CasioType1Directory", at, tiff)
                }
            } else if eight == "FUJIFILM" || upper == "FUJIFILM" {
                // The Fujifilm note is little-endian whatever the file, and its offset is from
                // the note's start.
                let before = self.motorola;
                self.motorola = false;
                let start = at.wrapping_add(self.get32(at + 8)?);
                let dir = metadata.directory(Directory::MakerNote("FujifilmDirectory"));
                let result = self.process_directory(metadata, dir, processed, start, tiff);
                self.motorola = before;
                return result;
            } else if upper.starts_with("MINOLTA") {
                ("OlympusDirectory", at, tiff)
            } else if two == "KC" || five == "MINOL" || three == "MLY" || eight == "+M+M+M+M" {
                return Ok(());
            } else if seven == "KYOCERA" {
                ("KyoceraDirectory", at + 22, tiff)
            } else if self.decode(at, 12)? == "Panasonic\0\0\0" {
                ("PanasonicDirectory", at + 12, tiff)
            } else if four == "AOC\0" {
                ("CasioType2Directory", at + 6, at)
            } else if upper.starts_with("PENTAX") || upper.starts_with("ASAHI") {
                ("PentaxDirectory", at, at)
            } else {
                return Ok(());
            }
        };
        let dir = metadata.directory(Directory::MakerNote(which));
        self.process_directory(metadata, dir, processed, start, base)
    }

    /// `ProcessTag`: the value as the directory keeps it - here only whether it is text.
    /// `// C#: com/drew/metadata/exif/ExifReader.cs:507-646`
    fn process_tag(
        &mut self,
        tag: i32,
        at: i32,
        components: i32,
        format: i32,
    ) -> Result<TagValue, OutOfSegment> {
        Ok(match format {
            2 => TagValue::Text(if tag == TAG_USER_COMMENT {
                self.read_comment_string(at, components, format)?
            } else {
                self.read_string(at, components)
            }),
            // The rest read inside the bounds already checked, or one byte of them.
            1 | 6 | 11 | 12 if components == 1 => {
                self.byte(at)?;
                TagValue::Other
            }
            _ => TagValue::Other,
        })
    }

    /// `ReadString`: up to the first NUL, the count, or the end of the segment.
    /// `// C#: com/drew/metadata/exif/ExifReader.cs:647-657`
    fn read_string(&self, at: i32, max: i32) -> String {
        let mut length = 0;
        while at.wrapping_add(length) < self.len()
            && self.byte(at.wrapping_add(length)).is_ok_and(|b| b != 0)
            && length < max
        {
            length += 1;
        }
        self.decode(at, length).unwrap_or_default()
    }

    /// `ReadCommentString`, which first overwrites the comment's trailing spaces with NULs in the
    /// segment itself. `// C#: com/drew/metadata/exif/ExifReader.cs:658-710`
    fn read_comment_string(
        &mut self,
        at: i32,
        components: i32,
        format: i32,
    ) -> Result<String, OutOfSegment> {
        let per = BYTES_PER_FORMAT
            .get(usize::try_from(format).unwrap_or(0))
            .copied()
            .unwrap_or(0);
        let byte_count = components.wrapping_mul(per);
        let mut i = byte_count - 1;
        while i >= 0 {
            let index = at.wrapping_add(i);
            if self.byte(index)? == b' ' {
                if let Some(slot) = usize::try_from(index)
                    .ok()
                    .and_then(|index| self.data.get_mut(index))
                {
                    *slot = 0;
                }
            } else {
                break;
            }
            i -= 1;
        }
        if self.decode(at, 5)? == "ASCII" {
            for i in 5..10 {
                let b = self.byte(at + i)?;
                if b != 0 && b != b' ' {
                    return Ok(self.read_string(at + i, 1999));
                }
            }
        } else if self.decode(at, 7)? == "UNICODE" {
            let start = at + 7;
            let mut from = start;
            for i in start..start + 10 {
                let b = self.byte(i)?;
                if b == 0 || b == b' ' {
                    continue;
                }
                from = i;
                break;
            }
            // `Utils.Decode(data, start, end - start, true)`: NULs dropped.
            let mut out = String::new();
            let mut j = from;
            while j < self.len() {
                let b = self.byte(j)?;
                if b != 0 {
                    out.push(if b == b'\r' { ' ' } else { char::from(b) });
                }
                j += 1;
            }
            return Ok(out);
        }
        Ok(self.read_string(at, 1999))
    }
}

/// `JpegMetadataReader.ReadMetadata(FileInfo)`: the segments up to the first SOS, then the first
/// APP1 through the EXIF reader. What the EXIF reader throws is swallowed and what it read before
/// kept (`JpegMetadataReader.cs:81-91`).
///
/// # Errors
///
/// [`ReadError::Jpeg`] where `JpegSegmentReader` throws `JpegProcessingException`.
/// `// C#: com/drew/imaging/jpg/JpegMetadataReader.cs:52-127, JpegSegmentReader.cs:97-166`
pub fn read_jpeg(file: &[u8]) -> Result<Metadata, ReadError> {
    if file.first() != Some(&0xFF) || file.get(1) != Some(&0xD8) {
        return Err(ReadError::Jpeg("not a jpeg file"));
    }
    let mut pos = 2usize;
    let mut app1: Option<&[u8]> = None;
    // `ReadByte` past the end is -1, which `& 0xFF` makes 0xFF; `Read` past it reads nothing and
    // leaves the length bytes zero.
    let next = |pos: &mut usize| -> u8 {
        let b = file.get(*pos).copied().unwrap_or(0xFF);
        *pos += 1;
        b
    };
    loop {
        let identifier = next(&mut pos);
        if identifier != 0xFF {
            return Err(ReadError::Jpeg(
                "expected jpeg segment start identifier 0xFF",
            ));
        }
        let marker = next(&mut pos);
        let high = file.get(pos).copied().unwrap_or(0);
        let low = if pos < file.len() {
            file.get(pos + 1).copied().unwrap_or(0)
        } else {
            0
        };
        pos = (pos + 2).min(file.len().max(pos));
        let length = (i64::from(high) << 8 | i64::from(low)) - 2;
        let remaining = i64::try_from(file.len().saturating_sub(pos)).unwrap_or(0);
        if length > remaining {
            return Err(ReadError::Jpeg(
                "segment size would extend beyond file stream length",
            ));
        }
        if length < 0 {
            return Err(ReadError::Jpeg("segment size would be less than zero"));
        }
        let length = usize::try_from(length).unwrap_or(0);
        let segment = file.get(pos..pos + length).unwrap_or(&[]);
        pos += length;
        match marker {
            0xDA | 0xD9 => break,
            0xE1 if app1.is_none() => app1 = Some(segment),
            _ => {}
        }
    }
    let mut metadata = Metadata::default();
    if let Some(segment) = app1 {
        let mut reader = ExifReader {
            data: segment.to_vec(),
            motorola: false,
        };
        let _ = reader.extract(&mut metadata);
    }
    Ok(metadata)
}

/// `TiffMetadataReader.ReadMetadata(FileInfo)`: the whole file as a TIFF.
///
/// # Errors
///
/// [`ReadError::Tiff`] where the walk throws, which the reader turns into
/// `TiffProcessingException` and so no metadata at all.
/// `// C#: com/drew/imaging/tiff/TiffMetadataReader.cs:17-53, ExifReader.cs:60-110`
pub fn read_tiff(file: &[u8]) -> Result<Metadata, ReadError> {
    let mut metadata = Metadata::default();
    let mut reader = ExifReader {
        data: file.to_vec(),
        motorola: false,
    };
    reader
        .extract_tiff(&mut metadata)
        .map_err(|OutOfSegment| ReadError::Tiff)?;
    Ok(metadata)
}

/// `AbstractDirectory.GetDate` of a text value: `DateTime.TryParse` in the current culture -
/// taken to be the invariant one - then `TryParseExact` with each of `DATE_FORMATS`, then
/// `DateTime.Today`.
///
/// Of `DateTime.TryParse`'s grammar only the forms a date tag carries are recognised: a date
/// `yyyy-M-d` or `yyyy/M/d` or `M/d/yyyy`, optionally followed by a space or `T` and `H:m` or
/// `H:m:s`, with white space around. An EXIF date, `yyyy:MM:dd HH:mm:ss`, is not among them -
/// `TryParse` rejects it (the oracle asked mono) - and is read by the second exact format.
/// `// C#: com/drew/metadata/AbstractDirectory.cs:587-657`
#[must_use]
pub fn get_date(text: &str, today: DateTime) -> DateTime {
    if let Some(date) = try_parse(text) {
        return date;
    }
    parse_date(text, today)
}

/// `ParseDate`: the exact formats in order, then today.
/// `// C#: com/drew/metadata/AbstractDirectory.cs:64, 630-657`
fn parse_date(text: &str, today: DateTime) -> DateTime {
    if text.trim().is_empty() {
        return today;
    }
    const FORMATS: [&str; 7] = [
        "dd/MM/yyyy HH:mm:ss",
        "yyyy:MM:dd HH:mm:ss",
        "yyyy-MM-dd_HH-mm-ss",
        "yyyy/MM/dd HH:mm:ss",
        "dd/MM/yyyy",
        "yyyy/MM/dd",
        "yyyy-MM-dd",
    ];
    for format in FORMATS {
        if let Some(date) = parse_exact(text, format) {
            return date;
        }
    }
    today
}

/// `DateTime.TryParseExact(s, format, null, DateTimeStyles.None)` for the formats above: each
/// `yyyy` four digits, each two-letter field two, every other character itself (`/` and `:` being
/// the invariant culture's separators), nothing else allowed.
fn parse_exact(text: &str, format: &str) -> Option<DateTime> {
    let text = text.as_bytes();
    let format = format.as_bytes();
    let (mut y, mut mo, mut d, mut h, mut mi, mut s) = (1, 1, 1, 0, 0, 0);
    let mut t = 0usize;
    let mut f = 0usize;
    while f < format.len() {
        let c = *format.get(f)?;
        let run = format.get(f..)?.iter().take_while(|&&x| x == c).count();
        let field = match c {
            b'y' | b'M' | b'd' | b'H' | b'm' | b's' => Some(c),
            _ => None,
        };
        if let Some(field) = field {
            let digits = text.get(t..t + run)?;
            if !digits.iter().all(u8::is_ascii_digit) {
                return None;
            }
            let value: i32 = std::str::from_utf8(digits).ok()?.parse().ok()?;
            match field {
                b'y' => y = value,
                b'M' => mo = value,
                b'd' => d = value,
                b'H' => h = value,
                b'm' => mi = value,
                _ => s = value,
            }
            t += run;
            f += run;
        } else {
            if text.get(t) != Some(&c) {
                return None;
            }
            t += 1;
            f += 1;
        }
    }
    if t != text.len() {
        return None;
    }
    DateTime::from_parts(y, mo, d, h, mi, s, Kind::Unspecified)
}

/// The part of `DateTime.TryParse` a date tag can reach (see [`get_date`]).
fn try_parse(text: &str) -> Option<DateTime> {
    let trimmed = text.trim_matches(|c: char| c.is_whitespace() || c == '\0');
    let (date, time) = match trimmed.find([' ', 'T']) {
        Some(at) => (trimmed.get(..at)?, Some(trimmed.get(at + 1..)?.trim())),
        None => (trimmed, None),
    };
    let numbers = |s: &str, sep: char| -> Option<Vec<i32>> {
        s.split(sep)
            .map(|p| {
                (!p.is_empty() && p.len() <= 4 && p.bytes().all(|b| b.is_ascii_digit()))
                    .then(|| p.parse().ok())
                    .flatten()
            })
            .collect()
    };
    let (y, mo, d) = if let Some(parts) = numbers(date, '-') {
        match parts.as_slice() {
            [y, mo, d] if date.len() >= 8 && date.find('-') == Some(4) => (*y, *mo, *d),
            _ => return None,
        }
    } else if let Some(parts) = numbers(date, '/') {
        match parts.as_slice() {
            [y, mo, d] if date.find('/') == Some(4) => (*y, *mo, *d),
            [mo, d, y] if *y >= 1000 => (*y, *mo, *d),
            _ => return None,
        }
    } else {
        return None;
    };
    let (h, mi, s) = match time {
        None => (0, 0, 0),
        Some(time) => match numbers(time, ':')?.as_slice() {
            [h, mi] => (*h, *mi, 0),
            [h, mi, s] => (*h, *mi, *s),
            _ => return None,
        },
    };
    DateTime::from_parts(y, mo, d, h, mi, s, Kind::Unspecified)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn today() -> DateTime {
        DateTime::from_parts(2030, 1, 2, 0, 0, 0, Kind::Local).unwrap()
    }

    #[test]
    fn exif_dates_go_to_the_second_exact_format() {
        let d = get_date("2026:09:24 11:30:08", today());
        assert_eq!(d.format_exif(), "2026:09:24 11:30:08");
        assert_eq!(d.kind, Kind::Unspecified);
        // TryParse takes the ISO form.
        assert_eq!(
            get_date("2026-09-24 11:30:08", today()).format_exif(),
            "2026:09:24 11:30:08"
        );
        // Blank, zero and trailing-space dates are today.
        assert_eq!(get_date("    :  :     :  :  ", today()), today());
        assert_eq!(get_date("0000:00:00 00:00:00", today()), today());
        assert_eq!(get_date("2026:09:24 11:30:08 ", today()), today());
        assert_eq!(get_date("", today()), today());
    }

    fn app1(tiff: &[u8]) -> Vec<u8> {
        let mut segment = b"Exif\0\0".to_vec();
        segment.extend_from_slice(tiff);
        let mut jpeg = vec![0xFF, 0xD8, 0xFF, 0xE1];
        let len = u16::try_from(segment.len() + 2).unwrap();
        jpeg.extend_from_slice(&len.to_be_bytes());
        jpeg.extend_from_slice(&segment);
        jpeg.extend_from_slice(&[0xFF, 0xDA, 0x00, 0x02, 0xFF, 0xD9]);
        jpeg
    }

    #[test]
    fn digitized_answers_only_when_original_is_absent_from_that_directory() {
        // II, IFD0 at 8 with one entry: 0x9004 ASCII count 20 at offset 26.
        let mut tiff = b"II\x2a\x00\x08\x00\x00\x00".to_vec();
        tiff.extend_from_slice(&[1, 0]);
        tiff.extend_from_slice(&[0x04, 0x90, 2, 0, 20, 0, 0, 0, 26, 0, 0, 0]);
        tiff.extend_from_slice(&[0, 0, 0, 0]);
        tiff.extend_from_slice(b"2026:09:24 11:30:08\0");
        tiff.extend_from_slice(&[0; 8]);
        let metadata = read_jpeg(&app1(&tiff)).unwrap();
        assert_eq!(
            metadata.date_tag(),
            Some(&TagValue::Text("2026:09:24 11:30:08".to_owned()))
        );
        assert!(read_jpeg(b"GIF89a").is_err());
        // A JPEG with no APP1 has no directories at all.
        let bare = read_jpeg(&[0xFF, 0xD8, 0xFF, 0xDA, 0x00, 0x02]).unwrap();
        assert!(bare.directories.is_empty());
    }
}
