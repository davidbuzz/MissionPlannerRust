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

//! Recording the link: writing telemetry logs.
//!
//! The link records itself, as `MAVLinkInterface.SaveToTlog` does in Mission Planner
//! (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1467`): an 8-byte big-endian microsecond
//! timestamp, then the frame exactly as it crossed the wire. This lives with the link rather than
//! beside the reader in `mp-log` because PLAN.md §5.1 puts the log crates at L4, above the link at
//! L3, and a recorder the link cannot reach records nothing. `mp-log`'s `tests/tlog.rs` reads back
//! what this writes, so the two halves of the format cannot drift apart.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::LinkError;

/// Size of the timestamp that precedes each frame. `mp_log::TIMESTAMP_LEN` is the same eight
/// bytes, read.
const TIMESTAMP_LEN: usize = 8;

/// Unix epoch microseconds, the unit tlog timestamps use.
fn now_micros() -> u64 {
    web_time::SystemTime::now()
        .duration_since(web_time::UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_micros()).unwrap_or(u64::MAX))
}

/// A recording that could not be written, and what was being attempted.
fn io(context: impl Into<String>, source: std::io::Error) -> LinkError {
    LinkError::Record {
        context: context.into(),
        source,
    }
}

/// Writes frames to a `.tlog` in Mission Planner's format.
///
/// Buffered, because a telemetry link produces small writes at high rate and an unbuffered
/// `write` per frame would put a syscall in the link thread's hot path.
#[derive(Debug)]
pub struct TlogWriter {
    out: BufWriter<File>,
    frames: u64,
    bytes: u64,
}

impl TlogWriter {
    /// Creates a log, failing if it already exists so a recording cannot silently overwrite one.
    pub fn create(path: impl AsRef<Path>) -> Result<Self, LinkError> {
        let path = path.as_ref();
        let file =
            File::create_new(path).map_err(|e| io(format!("creating {}", path.display()), e))?;
        Ok(Self {
            out: BufWriter::with_capacity(64 * 1024, file),
            frames: 0,
            bytes: 0,
        })
    }

    /// Appends a frame stamped with the current time.
    pub fn write_frame(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        self.write_frame_at(frame, now_micros())
    }

    /// Appends a frame with an explicit timestamp, used by tests and by log conversion.
    pub fn write_frame_at(&mut self, frame: &[u8], timestamp_micros: u64) -> Result<(), LinkError> {
        // Big-endian: the format predates any thought of endianness portability, and Mission
        // Planner reverses the bytes on read.
        self.out
            .write_all(&timestamp_micros.to_be_bytes())
            .map_err(|e| io("writing timestamp", e))?;
        self.out
            .write_all(frame)
            .map_err(|e| io("writing frame", e))?;
        self.frames += 1;
        self.bytes += (TIMESTAMP_LEN + frame.len()) as u64;
        Ok(())
    }

    /// Frames written so far.
    #[must_use]
    pub const fn frames(&self) -> u64 {
        self.frames
    }

    /// Bytes written so far, including timestamps.
    #[must_use]
    pub const fn bytes(&self) -> u64 {
        self.bytes
    }

    /// Flushes buffered data. Called on drop, but a recorder should call it periodically so a
    /// crash loses seconds rather than minutes.
    pub fn flush(&mut self) -> Result<(), LinkError> {
        self.out.flush().map_err(|e| io("flushing log", e))
    }
}

impl Drop for TlogWriter {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}
