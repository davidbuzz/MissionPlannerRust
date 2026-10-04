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

//! `AviWriter`: the MJPEG AVI the flight screen's Record Hud to AVI writes - a RIFF file whose
//! headers are rewritten after every frame "so even partial files will play", the frames `00dc`
//! chunks of JPEG, each padded to an even length, and an `idx1` index at the close.
//!
//! Byte for byte the C#'s layout, its arithmetic included: the headers are reserved 8,204 bytes
//! with a JUNK chunk filling what they do not use, the `movi` list header sits at 8,192, the
//! index offsets count from the `movi` list's data, and the RIFF size is the file's length less
//! twelve (a RIFF size proper is the length less eight; the C#'s constants fall four short). A frame added when the recording is behind its target
//! rate is added again until the count catches up ("Extra frame"), the target being 10 a second
//! until the first `end` says 25. `// C#: ExtLibs/Utilities/AviWriter.cs`

use std::fs::File;
use std::io::{self, BufWriter, Seek, SeekFrom, Write};
use std::path::Path;
use std::time::Duration;

/// `fd.Seek(8204)`: where the frames begin.
const DATA_AT: u64 = 8204;
/// Where the `movi` list's header is written.
const MOVI_AT: u64 = 8192;
/// `dwChunkOffset = offset - 8212 + 4`.
const INDEX_BASE: u64 = 8208;
/// The frame header and the index entry, as the C# sizes them.
const DB_HEAD: u32 = 8;
const INDEX_ENTRY: u32 = 16;

/// One `AVIINDEXENTRY`. `// C#: AviWriter.cs:38-48`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct IndexEntry {
    offset: u32,
    length: u32,
}

/// The writer: `avi_start`, `avi_add`, `avi_end`, `avi_close`.
#[derive(Debug)]
pub struct AviWriter {
    file: Option<BufWriter<File>>,
    indexes: Vec<IndexEntry>,
    nframes: u32,
    totalsize: u32,
    /// `targetfps`: 10 until `end` says otherwise.
    target_fps: u32,
    width: i32,
    height: i32,
}

/// A little-endian `u32`.
fn u32le(value: u32) -> [u8; 4] {
    value.to_le_bytes()
}

impl Default for AviWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl AviWriter {
    /// A writer with no file open, its target rate the C#'s starting 10 a second.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            file: None,
            indexes: Vec::new(),
            nframes: 0,
            totalsize: 0,
            target_fps: 10,
            width: 0,
            height: 0,
        }
    }

    /// Whether a file is open.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.file.is_some()
    }

    /// Frames added so far.
    #[must_use]
    pub const fn frames(&self) -> u32 {
        self.nframes
    }

    /// `avi_start(filename)`: any file open closed first, the new one created and its first
    /// 8,204 bytes left for the headers.
    ///
    /// # Errors
    ///
    /// The file could not be created, or the one before it closed. `// C#: AviWriter.cs:206-221`
    pub fn start(&mut self, path: &Path) -> io::Result<()> {
        self.close()?;
        let mut file = BufWriter::new(File::create(path)?);
        file.seek(SeekFrom::Start(DATA_AT))?;
        self.file = Some(file);
        self.indexes.clear();
        self.nframes = 0;
        self.totalsize = 0;
        Ok(())
    }

    /// `avi_add(buf, size)`: one JPEG frame as a `00dc` chunk, padded to an even length and
    /// indexed; and again while `elapsed` since the start at the target rate is ahead of the
    /// frames written, the C#'s "Extra frame".
    ///
    /// # Errors
    ///
    /// No file is open, or the write failed. `// C#: AviWriter.cs:225-257`
    pub fn add(&mut self, jpeg: &[u8], elapsed: Duration) -> io::Result<()> {
        loop {
            self.add_once(jpeg)?;
            // `if (((DateTime.Now - start).TotalSeconds * targetfps) > nframes)`
            let due = elapsed.as_secs_f64() * f64::from(self.target_fps);
            if due <= f64::from(self.nframes) {
                return Ok(());
            }
        }
    }

    fn add_once(&mut self, jpeg: &[u8]) -> io::Result<()> {
        let file = self
            .file
            .as_mut()
            .ok_or_else(|| io::Error::other("no AVI file is open"))?;
        let mut size =
            u32::try_from(jpeg.len()).map_err(|_| io::Error::other("a frame of more than 4 GB"))?;
        file.write_all(b"00dc")?;
        file.write_all(&u32le(size))?;
        let offset = file.stream_position()?;
        file.write_all(jpeg)?;
        self.indexes.push(IndexEntry {
            offset: u32::try_from(offset.saturating_sub(INDEX_BASE)).unwrap_or(u32::MAX),
            length: size,
        });
        while file.stream_position()? % 2 != 0 {
            size += 1;
            file.write_all(&[0])?;
        }
        self.nframes += 1;
        self.totalsize = self.totalsize.wrapping_add(size);
        Ok(())
    }

    /// `avi_end(width, height, fps)`: the headers written at the front for the frames so far,
    /// the file left where it was. Called after every frame, so a file cut short still plays.
    ///
    /// # Errors
    ///
    /// No file is open, or the write failed. `// C#: AviWriter.cs:265-345`
    pub fn end(&mut self, width: i32, height: i32, fps: u32) -> io::Result<()> {
        self.width = width;
        self.height = height;
        self.target_fps = fps;
        let file = self
            .file
            .as_mut()
            .ok_or_else(|| io::Error::other("no AVI file is open"))?;
        let nframes = self.nframes;
        let index_length = u32::try_from(self.indexes.len())
            .unwrap_or(u32::MAX)
            .wrapping_mul(INDEX_ENTRY)
            .wrapping_add(8);
        let frame_bytes = nframes.wrapping_mul(DB_HEAD).wrapping_add(self.totalsize);
        // The six headers after the RIFF's: 12 + 64 + 12 + 64 + 48 + 12.
        let riff_size = 212u32
            .wrapping_add(frame_bytes)
            .wrapping_add(index_length)
            .wrapping_add(7980);
        let lh1_size: u32 = 4 + 64 + 12 + 64 + 48;
        let lh2_size: u32 = 4 + 64 + 48;
        let lh3_size = 4u32.wrapping_add(frame_bytes);
        let junk_size = 8204 - lh1_size - 12 - 12 - 12 - 4;
        let fps_i = fps.max(1);
        let unpacked = 3i64 * i64::from(width) * i64::from(height);
        let per_second = u32::try_from(unpacked * i64::from(fps_i)).unwrap_or(u32::MAX);
        let unpacked = u32::try_from(unpacked).unwrap_or(u32::MAX);

        let mut head = Vec::with_capacity(224);
        // riff_head
        head.extend_from_slice(b"RIFF");
        head.extend_from_slice(&u32le(riff_size));
        head.extend_from_slice(b"AVI ");
        // list_head hdrl
        head.extend_from_slice(b"LIST");
        head.extend_from_slice(&u32le(lh1_size));
        head.extend_from_slice(b"hdrl");
        // avi_head
        head.extend_from_slice(b"avih");
        head.extend_from_slice(&u32le(64 - 8));
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 1e6 / fps fits
        let micros = (1e6 / f64::from(fps_i)) as u32;
        head.extend_from_slice(&u32le(micros)); // time
        head.extend_from_slice(&u32le(per_second)); // maxbytespersec
        head.extend_from_slice(&u32le(0)); // reserved1
        head.extend_from_slice(&u32le(0x10)); // flags
        head.extend_from_slice(&u32le(nframes)); // nframes
        head.extend_from_slice(&u32le(0)); // initialframes
        head.extend_from_slice(&u32le(1)); // numstreams
        head.extend_from_slice(&u32le(0)); // suggested_bufsize
        head.extend_from_slice(&u32le(u32::try_from(width).unwrap_or(0)));
        head.extend_from_slice(&u32le(u32::try_from(height).unwrap_or(0)));
        head.extend_from_slice(&u32le(0)); // scale
        head.extend_from_slice(&u32le(0)); // rate
        head.extend_from_slice(&u32le(0)); // start
        head.extend_from_slice(&u32le(0)); // length
        // list_head strl
        head.extend_from_slice(b"LIST");
        head.extend_from_slice(&u32le(lh2_size));
        head.extend_from_slice(b"strl");
        // stream_head
        head.extend_from_slice(b"strh");
        head.extend_from_slice(&u32le(64 - 8));
        head.extend_from_slice(b"vids");
        head.extend_from_slice(b"MJPG");
        head.extend_from_slice(&u32le(0)); // flags
        head.extend_from_slice(&u32le(0)); // reserved1
        head.extend_from_slice(&u32le(0)); // initialframes
        head.extend_from_slice(&u32le(1)); // scale
        head.extend_from_slice(&u32le(fps_i)); // rate
        head.extend_from_slice(&u32le(0)); // start
        head.extend_from_slice(&u32le(nframes)); // length
        head.extend_from_slice(&u32le(per_second)); // suggested_bufsize
        head.extend_from_slice(&u32le(u32::MAX)); // quality: (uint)-1
        head.extend_from_slice(&u32le(0)); // samplesize
        head.extend_from_slice(&0i16.to_le_bytes()); // l
        head.extend_from_slice(&0i16.to_le_bytes()); // t
        head.extend_from_slice(&i16::try_from(width).unwrap_or(i16::MAX).to_le_bytes()); // r
        head.extend_from_slice(&i16::try_from(height).unwrap_or(i16::MAX).to_le_bytes()); // b
        // frame_head
        head.extend_from_slice(b"strf");
        head.extend_from_slice(&u32le(48 - 8));
        head.extend_from_slice(&u32le(48 - 8)); // size2
        head.extend_from_slice(&width.to_le_bytes());
        head.extend_from_slice(&height.to_le_bytes());
        head.extend_from_slice(&1i16.to_le_bytes()); // planes
        head.extend_from_slice(&24i16.to_le_bytes()); // bitcount
        head.extend_from_slice(b"MJPG");
        head.extend_from_slice(&u32le(unpacked));
        head.extend_from_slice(&u32le(0)); // r1
        head.extend_from_slice(&u32le(0)); // r2
        head.extend_from_slice(&u32le(0)); // clr_used
        head.extend_from_slice(&u32le(0)); // clr_important
        // JUNK, its type the C#'s null array: zeros.
        head.extend_from_slice(b"JUNK");
        head.extend_from_slice(&u32le(junk_size));
        head.extend_from_slice(&[0, 0, 0, 0]);

        let position = file.stream_position()?;
        file.seek(SeekFrom::Start(0))?;
        file.write_all(&head)?;
        file.seek(SeekFrom::Start(MOVI_AT))?;
        file.write_all(b"LIST")?;
        file.write_all(&u32le(lh3_size))?;
        file.write_all(b"movi")?;
        file.seek(SeekFrom::Start(position))?;
        Ok(())
    }

    /// `avi_close`: the headers for what was written, the `idx1` index at the end, the file
    /// closed. Nothing when no file is open.
    ///
    /// # Errors
    ///
    /// A write failed. `// C#: AviWriter.cs:191-203, 347-360`
    pub fn close(&mut self) -> io::Result<()> {
        if self.file.is_none() {
            return Ok(());
        }
        let (width, height, fps) = (self.width, self.height, self.target_fps);
        self.end(width, height, fps)?;
        let Some(mut file) = self.file.take() else {
            return Ok(());
        };
        file.seek(SeekFrom::End(0))?;
        file.write_all(b"idx1")?;
        let count = u32::try_from(self.indexes.len()).unwrap_or(u32::MAX);
        file.write_all(&u32le(count.wrapping_mul(INDEX_ENTRY)))?;
        for entry in &self.indexes {
            file.write_all(b"00dc")?;
            file.write_all(&u32le(0x10))?;
            file.write_all(&u32le(entry.offset))?;
            file.write_all(&u32le(entry.length))?;
        }
        file.flush()?;
        Ok(())
    }
}

impl Drop for AviWriter {
    /// `Dispose`: `avi_close`.
    fn drop(&mut self) {
        let _ = self.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn u32_at(bytes: &[u8], at: usize) -> u32 {
        let mut word = [0u8; 4];
        word.copy_from_slice(&bytes[at..at + 4]);
        u32::from_le_bytes(word)
    }

    #[test]
    #[allow(clippy::indexing_slicing)]
    fn the_file_is_the_csharps_layout() {
        let dir = mp_os::temp_dir().join(format!("mp-video-avi-{}", mp_os::process_id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("hud.avi");
        let mut writer = AviWriter::new();
        writer.start(&path).unwrap();
        // Three frames, the second of an odd length; no time has passed, so no extra frames.
        writer
            .add(&[0xFF, 0xD8, 1, 2, 3, 4], Duration::ZERO)
            .unwrap();
        writer.end(398, 258, 25).unwrap();
        writer.add(&[0xFF, 0xD8, 9, 8, 7], Duration::ZERO).unwrap();
        writer.end(398, 258, 25).unwrap();
        writer
            .add(&[0xFF, 0xD8, 5, 5, 5, 5], Duration::ZERO)
            .unwrap();
        writer.end(398, 258, 25).unwrap();
        assert_eq!(writer.frames(), 3);
        writer.close().unwrap();
        assert!(!writer.is_open());

        let bytes = std::fs::read(&path).unwrap();
        // 8204 of headers, then 8 + 6, 8 + 5 + 1 of padding, 8 + 6, then idx1 of 8 + 3 * 16.
        assert_eq!(bytes.len(), 8204 + 14 + 14 + 14 + 8 + 48);
        assert_eq!(&bytes[0..4], b"RIFF");
        // The C#'s size: 212 + the frames + the index + 7980, the file's length less twelve.
        assert_eq!(u32_at(&bytes, 4) as usize, bytes.len() - 12);
        assert_eq!(&bytes[8..12], b"AVI ");
        assert_eq!(&bytes[12..16], b"LIST");
        assert_eq!(u32_at(&bytes, 16), 192);
        assert_eq!(&bytes[20..24], b"hdrl");
        assert_eq!(&bytes[24..28], b"avih");
        assert_eq!(u32_at(&bytes, 28), 56);
        assert_eq!(u32_at(&bytes, 32), 40_000); // microseconds a frame at 25
        assert_eq!(u32_at(&bytes, 36), 3 * 398 * 258 * 25);
        assert_eq!(u32_at(&bytes, 44), 0x10);
        assert_eq!(u32_at(&bytes, 48), 3); // nframes
        assert_eq!(u32_at(&bytes, 56), 1); // streams
        assert_eq!(u32_at(&bytes, 64), 398);
        assert_eq!(u32_at(&bytes, 68), 258);
        assert_eq!(&bytes[88..92], b"LIST");
        assert_eq!(u32_at(&bytes, 92), 116);
        assert_eq!(&bytes[96..100], b"strl");
        assert_eq!(&bytes[100..104], b"strh");
        assert_eq!(&bytes[108..112], b"vids");
        assert_eq!(&bytes[112..116], b"MJPG");
        assert_eq!(u32_at(&bytes, 128), 1); // scale
        assert_eq!(u32_at(&bytes, 132), 25); // rate
        assert_eq!(u32_at(&bytes, 140), 3); // length
        assert_eq!(u32_at(&bytes, 148), u32::MAX); // quality
        assert_eq!(&bytes[156..160], &[0, 0, 0, 0]); // l, t
        assert_eq!(&bytes[160..164], &[142, 1, 2, 1]); // r, b: 398, 258
        assert_eq!(&bytes[164..168], b"strf");
        assert_eq!(u32_at(&bytes, 168), 40);
        assert_eq!(u32_at(&bytes, 176), 398);
        assert_eq!(u32_at(&bytes, 180), 258);
        assert_eq!(&bytes[188..192], b"MJPG");
        assert_eq!(u32_at(&bytes, 192), 3 * 398 * 258);
        assert_eq!(&bytes[212..216], b"JUNK");
        assert_eq!(u32_at(&bytes, 216), 7972);
        assert_eq!(&bytes[8192..8196], b"LIST");
        assert_eq!(u32_at(&bytes, 8196), 4 + 3 * 8 + 6 + 6 + 6);
        assert_eq!(&bytes[8200..8204], b"movi");
        assert_eq!(&bytes[8204..8208], b"00dc");
        assert_eq!(u32_at(&bytes, 8208), 6);
        assert_eq!(&bytes[8212..8218], &[0xFF, 0xD8, 1, 2, 3, 4]);
        assert_eq!(&bytes[8218..8222], b"00dc");
        assert_eq!(u32_at(&bytes, 8222), 5);
        assert_eq!(bytes[8231], 0, "padded to even");
        let idx = 8204 + 14 + 14 + 14;
        assert_eq!(&bytes[idx..idx + 4], b"idx1");
        assert_eq!(u32_at(&bytes, idx + 4), 48);
        assert_eq!(&bytes[idx + 8..idx + 12], b"00dc");
        assert_eq!(u32_at(&bytes, idx + 12), 0x10);
        assert_eq!(u32_at(&bytes, idx + 16), 4); // the first frame's data, from the movi list
        assert_eq!(u32_at(&bytes, idx + 20), 6);
        assert_eq!(u32_at(&bytes, idx + 32), 4 + 6 + 8); // the second's: 8212 + 6 + 8 - 8208
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_frame_behind_the_clock_is_added_again() {
        let dir = mp_os::temp_dir().join(format!("mp-video-avi-extra-{}", mp_os::process_id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("extra.avi");
        let mut writer = AviWriter::new();
        writer.start(&path).unwrap();
        // Half a second in at the starting rate of 10 a second: five frames due, so the one
        // frame is written five times.
        writer.add(&[1, 2], Duration::from_millis(500)).unwrap();
        assert_eq!(writer.frames(), 5);
        writer.end(10, 10, 25).unwrap();
        // At 25 a second, 0.52 s is thirteen frames: eight more.
        writer.add(&[1, 2], Duration::from_millis(520)).unwrap();
        assert_eq!(writer.frames(), 13);
        writer.close().unwrap();
        assert!(writer.add(&[1], Duration::ZERO).is_err(), "closed");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
