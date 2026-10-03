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

//! Incremental frame decoder for byte streams.
//!
//! A link hands us whatever the OS read returned: partial frames, several frames at once, or
//! noise from a half-open serial port. This decoder owns a **fixed-size** buffer, so a hostile or
//! broken peer cannot make it allocate, and resynchronises one byte at a time after corruption.

use crate::dialect::Dialect;
use crate::frame::{Frame, MAX_FRAME_LEN, ParseError, parse};

/// Decoder buffer capacity: two maximal frames, so a full frame always fits even when a partial
/// one precedes it, without ever compacting mid-frame.
pub const CAPACITY: usize = MAX_FRAME_LEN * 2;

/// Counters describing link health. Cheap enough to update per frame and surfaced in the UI as
/// the equivalent of Mission Planner's packet-loss indicator.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DecodeStats {
    /// Frames parsed successfully.
    pub frames: u64,
    /// Frames that carried a valid signature block.
    pub signed_frames: u64,
    /// Frames rejected by the checksum.
    pub crc_errors: u64,
    /// Frames whose message id the dialect does not know.
    pub unknown_msgids: u64,
    /// Frames rejected for setting an incompatibility flag we do not implement.
    pub unsupported_flags: u64,
    /// Bytes skipped while resynchronising after corruption.
    pub resync_bytes: u64,
    /// Bytes dropped because the caller pushed faster than it drained.
    pub overflow_bytes: u64,
}

/// Streaming MAVLink decoder.
#[derive(Debug)]
pub struct FrameDecoder {
    buf: [u8; CAPACITY],
    head: usize,
    tail: usize,
    stats: DecodeStats,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    /// Creates an empty decoder. The buffer is inline, so this does not allocate.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0; CAPACITY],
            head: 0,
            tail: 0,
            stats: DecodeStats::new_const(),
        }
    }

    /// Link statistics accumulated so far.
    #[must_use]
    pub const fn stats(&self) -> &DecodeStats {
        &self.stats
    }

    /// Bytes currently buffered.
    #[must_use]
    pub const fn buffered(&self) -> usize {
        self.tail - self.head
    }

    /// Discards buffered bytes, e.g. after a reconnect.
    pub fn reset(&mut self) {
        self.head = 0;
        self.tail = 0;
    }

    /// Copies `data` into the buffer, returning how many bytes were accepted.
    ///
    /// A short return means the caller is not draining; the shortfall is counted in
    /// [`DecodeStats::overflow_bytes`] rather than silently ignored.
    pub fn push(&mut self, data: &[u8]) -> usize {
        let n = self.push_inner(data);
        let dropped = data.len() - n;
        if dropped > 0 {
            self.stats.overflow_bytes += dropped as u64;
        }
        n
    }

    /// Copies what fits without touching the overflow counter.
    ///
    /// [`push`](Self::push) treats a short accept as data loss, because for a caller that pushes
    /// once and moves on it is. [`push_and_drain`](Self::push_and_drain) retries the remainder,
    /// so for it a short accept is just backpressure and counting it would be a lie.
    fn push_inner(&mut self, data: &[u8]) -> usize {
        self.compact();
        let free = CAPACITY - self.tail;
        let n = data.len().min(free);
        if let (Some(dst), Some(src)) = (self.buf.get_mut(self.tail..self.tail + n), data.get(..n))
        {
            dst.copy_from_slice(src);
        }
        self.tail += n;
        n
    }

    /// Parses every complete frame currently buffered, invoking `on_frame` for each.
    ///
    /// Frames are delivered **in order**, and only after their checksum passes. A corrupt byte
    /// that happens to look like a header with a large length field therefore stalls the stream
    /// until enough bytes arrive to prove it wrong - up to [`MAX_FRAME_LEN`] bytes. That is the
    /// correct trade for a live link, because the alternative is emitting a later frame before an
    /// earlier one that turns out to be valid. Use [`flush`](Self::flush) at end-of-stream or on
    /// an idle timeout to force resynchronisation instead of waiting.
    ///
    /// The callback receives a borrow of the internal buffer, which is why this is a callback
    /// rather than an iterator: it keeps the zero-copy guarantee without handing out a lifetime
    /// that outlives the next `push`.
    pub fn drain<D, F>(&mut self, dialect: &D, mut on_frame: F)
    where
        D: Dialect + ?Sized,
        F: FnMut(&Frame<'_>),
    {
        loop {
            // Disjoint field borrows: the frame borrows `buf` while `head`/`stats` are updated.
            let Self {
                buf,
                head,
                tail,
                stats,
            } = &mut *self;
            let Some(window) = buf.get(*head..*tail) else {
                break;
            };
            if window.is_empty() {
                break;
            }

            match parse(window, dialect) {
                Ok((frame, used)) => {
                    stats.frames += 1;
                    if frame.is_signed() {
                        stats.signed_frames += 1;
                    }
                    on_frame(&frame);
                    *head += used;
                }
                Err(ParseError::Incomplete { .. }) => {
                    // A window larger than any legal frame that still will not parse is garbage
                    // shaped like a header. Skip a byte so the stream cannot wedge.
                    if window.len() >= MAX_FRAME_LEN {
                        *head += 1;
                        stats.resync_bytes += 1;
                        continue;
                    }
                    break;
                }
                Err(err) => {
                    match err {
                        ParseError::Crc { .. } => stats.crc_errors += 1,
                        ParseError::UnknownMessage { .. } => stats.unknown_msgids += 1,
                        ParseError::UnsupportedIncompatFlags(_) => stats.unsupported_flags += 1,
                        _ => {}
                    }
                    *head += 1;
                    stats.resync_bytes += 1;
                }
            }
        }
        self.compact();
    }

    /// Forces resynchronisation, giving up on any pending partial frame.
    ///
    /// Unlike [`drain`](Self::drain), an incomplete candidate at the head of the buffer is
    /// skipped a byte at a time rather than waited on. Call this when no more bytes are coming
    /// (end of a log file) or when a live link has gone idle long enough that a partial frame is
    /// certainly lost. Any genuinely in-flight frame still buffered is discarded, so do not call
    /// it on every read.
    pub fn flush<D, F>(&mut self, dialect: &D, mut on_frame: F)
    where
        D: Dialect + ?Sized,
        F: FnMut(&Frame<'_>),
    {
        loop {
            let Self {
                buf,
                head,
                tail,
                stats,
            } = &mut *self;
            let Some(window) = buf.get(*head..*tail) else {
                break;
            };
            if window.is_empty() {
                break;
            }
            match parse(window, dialect) {
                Ok((frame, used)) => {
                    stats.frames += 1;
                    if frame.is_signed() {
                        stats.signed_frames += 1;
                    }
                    on_frame(&frame);
                    *head += used;
                }
                Err(err) => {
                    match err {
                        ParseError::Crc { .. } => stats.crc_errors += 1,
                        ParseError::UnknownMessage { .. } => stats.unknown_msgids += 1,
                        ParseError::UnsupportedIncompatFlags(_) => stats.unsupported_flags += 1,
                        _ => {}
                    }
                    *head += 1;
                    stats.resync_bytes += 1;
                }
            }
        }
        self.compact();
    }

    /// Feeds an arbitrarily large buffer through the decoder.
    ///
    /// The internal buffer is deliberately small and fixed, so a caller handing over more than
    /// [`CAPACITY`] bytes at once - a 4 KiB socket read, a whole log file - must be consumed in
    /// several passes. Doing that here rather than in every caller removes an easy way to lose
    /// data silently: a plain `push` would accept only what fits and count the rest as overflow.
    ///
    /// Returns the number of bytes consumed, which is `data.len()` unless the link is wedged.
    pub fn push_and_drain<D, F>(&mut self, data: &[u8], dialect: &D, mut on_frame: F) -> usize
    where
        D: Dialect + ?Sized,
        F: FnMut(&Frame<'_>),
    {
        let mut consumed = 0;
        while consumed < data.len() {
            let Some(rest) = data.get(consumed..) else {
                break;
            };
            let n = self.push_inner(rest);
            if n == 0 {
                // Buffer full and nothing drainable: a single frame cannot exceed CAPACITY, so
                // this means the buffer is wedged with garbage. Let drain resynchronise.
                self.drain(dialect, &mut on_frame);
                if self.buffered() >= CAPACITY {
                    break;
                }
                continue;
            }
            consumed += n;
            self.drain(dialect, &mut on_frame);
        }
        consumed
    }

    fn compact(&mut self) {
        if self.head == 0 {
            return;
        }
        if self.head == self.tail {
            self.head = 0;
            self.tail = 0;
            return;
        }
        self.buf.copy_within(self.head..self.tail, 0);
        self.tail -= self.head;
        self.head = 0;
    }
}

impl DecodeStats {
    const fn new_const() -> Self {
        Self {
            frames: 0,
            signed_frames: 0,
            crc_errors: 0,
            unknown_msgids: 0,
            unsupported_flags: 0,
            resync_bytes: 0,
            overflow_bytes: 0,
        }
    }
}
