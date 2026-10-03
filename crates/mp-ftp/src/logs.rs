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

//! Downloading dataflash logs from the vehicle.
//!
//! The vehicle lists what it holds, and then hands over one log in 90-byte chunks, each stamped
//! with its offset. The offsets are the whole problem: chunks arrive out of order, they are
//! re-sent when a request is repeated, and the last one is short. A downloader that appends what
//! arrives produces a file that is the right length and wrong everywhere, which a log parser then
//! reports as corruption in the vehicle rather than in the transfer.
//!
//! So chunks are written at their stated offset into a sparse buffer, and the transfer is finished
//! when every byte has been filled - not when the expected number of bytes has arrived.

use std::collections::BTreeMap;

use mp_vehicle::VehicleId;

/// How much of a log to ask for at a time.
///
/// Large on purpose. ArduPilot answers a request by streaming `LOG_DATA` until the window is
/// satisfied, so the window is the unit of work, not the unit of transfer - and a small window
/// means the transfer runs at one window per re-request rather than at the link's speed. Measured
/// against SITL: a 9 KB window nudged every 400 ms moved 22 KB/s and would have taken eleven
/// minutes for a 15 MB log. The window is what a re-request costs when a gap appears, which is
/// rare, so it should be big.
pub const WINDOW_BYTES: u32 = 1_024 * 1_024;

/// Bytes in one `LOG_DATA` message.
pub const CHUNK_BYTES: usize = 90;

/// One log the vehicle holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogListing {
    /// Log number, as the vehicle indexes them.
    pub id: u16,
    /// Size in bytes.
    pub size: u32,
    /// Unix time the log was started, or zero if the vehicle had no clock.
    pub time_utc: u32,
}

impl LogListing {
    /// Whether the vehicle knew what time it was when this log began.
    ///
    /// A flight controller with no GPS fix and no RTC reports zero, and a ground station that
    /// shows "1 January 1970" for every log is less useful than one that says it does not know.
    #[must_use]
    pub const fn has_timestamp(&self) -> bool {
        self.time_utc > 0
    }
}

/// A log download in progress.
#[derive(Debug)]
pub struct LogDownload {
    /// Which vehicle.
    pub target: VehicleId,
    /// Which log.
    pub id: u16,
    /// How big the vehicle said it is.
    pub size: u32,
    /// Chunks received, keyed by offset. Sparse, because they arrive out of order.
    chunks: BTreeMap<u32, Vec<u8>>,
    /// Bytes filled so far, counted rather than summed on demand.
    filled: u32,
}

impl LogDownload {
    /// Starts a download.
    #[must_use]
    pub fn new(target: VehicleId, id: u16, size: u32) -> Self {
        Self {
            target,
            id,
            size,
            chunks: BTreeMap::new(),
            filled: 0,
        }
    }

    /// Records a chunk. Returns false if it was already held, which happens on a re-request.
    pub fn receive(&mut self, offset: u32, data: &[u8]) -> bool {
        if data.is_empty() || offset >= self.size {
            return false;
        }
        // Trim a chunk that runs past the end. The last one is short and ArduPilot pads it, so
        // taking all ninety bytes would append padding to the file.
        let remaining = self.size.saturating_sub(offset);
        let take = usize::try_from(remaining)
            .unwrap_or(data.len())
            .min(data.len());
        let Some(slice) = data.get(..take) else {
            return false;
        };

        if self.chunks.contains_key(&offset) {
            return false;
        }
        self.filled = self
            .filled
            .saturating_add(u32::try_from(slice.len()).unwrap_or(0));
        self.chunks.insert(offset, slice.to_vec());
        true
    }

    /// How many bytes have arrived.
    #[must_use]
    pub const fn filled(&self) -> u32 {
        self.filled
    }

    /// How far through, from zero to one.
    #[must_use]
    pub fn fraction(&self) -> f32 {
        if self.size == 0 {
            return 1.0;
        }
        #[allow(clippy::cast_precision_loss)] // log sizes are megabytes at most
        {
            (self.filled as f32 / self.size as f32).clamp(0.0, 1.0)
        }
    }

    /// Whether every byte has arrived.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        self.filled >= self.size
    }

    /// The first gap in what has been received, as an offset.
    ///
    /// Used to ask again for what is missing rather than starting over. A transfer that restarts
    /// on any loss never finishes over a lossy link, which is the only kind of link a log download
    /// happens over - they are megabytes long and telemetry radios are slow.
    #[must_use]
    pub fn first_gap(&self) -> Option<u32> {
        let mut expected = 0u32;
        for (offset, data) in &self.chunks {
            if *offset > expected {
                return Some(expected);
            }
            expected = offset.saturating_add(u32::try_from(data.len()).unwrap_or(0));
        }
        (expected < self.size).then_some(expected)
    }

    /// Assembles what has been received into one buffer.
    ///
    /// Gaps are zero-filled, so an incomplete download produces a file of the right length with
    /// holes rather than a shorter file whose contents are silently shifted. A log parser can
    /// resynchronise past a hole; it cannot detect a shift.
    #[must_use]
    pub fn assemble(&self) -> Vec<u8> {
        let mut out = vec![0u8; usize::try_from(self.size).unwrap_or(0)];
        for (offset, data) in &self.chunks {
            let Ok(start) = usize::try_from(*offset) else {
                continue;
            };
            let end = start.saturating_add(data.len()).min(out.len());
            if let Some(slice) = out.get_mut(start..end)
                && let Some(source) = data.get(..end.saturating_sub(start))
            {
                slice.copy_from_slice(source);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    fn download(size: u32) -> LogDownload {
        LogDownload::new(target(), 3, size)
    }

    #[test]
    fn chunks_are_placed_at_their_offset_not_appended() {
        // Out-of-order arrival is normal. Appending produces a file of the right length that is
        // wrong everywhere, which a parser then reports as corruption in the vehicle.
        let mut log = download(270);
        assert!(log.receive(180, &[3u8; 90]));
        assert!(log.receive(0, &[1u8; 90]));
        assert!(log.receive(90, &[2u8; 90]));

        let assembled = log.assemble();
        assert_eq!(assembled[0], 1);
        assert_eq!(assembled[90], 2);
        assert_eq!(assembled[180], 3);
        assert!(log.is_complete());
    }

    #[test]
    fn a_repeated_chunk_is_not_counted_twice() {
        // Re-requesting a gap re-sends chunks either side of it. Counting them again would report
        // more bytes received than the log has.
        let mut log = download(180);
        assert!(log.receive(0, &[1u8; 90]));
        assert!(!log.receive(0, &[1u8; 90]), "the second copy is not new");
        assert_eq!(log.filled(), 90);
        assert!(!log.is_complete());
    }

    #[test]
    fn the_last_chunk_is_trimmed_rather_than_padded() {
        // ArduPilot pads the final LOG_DATA to ninety bytes. Taking all of it appends padding to
        // the file, which a dataflash parser reads as a truncated record.
        let mut log = download(100);
        assert!(log.receive(0, &[1u8; 90]));
        assert!(log.receive(90, &[2u8; 90]));
        assert_eq!(log.filled(), 100);
        assert_eq!(log.assemble().len(), 100);
        assert!(log.is_complete());
    }

    #[test]
    fn a_chunk_beyond_the_end_is_refused() {
        let mut log = download(90);
        assert!(!log.receive(90, &[1u8; 90]));
        assert!(!log.receive(1000, &[1u8; 90]));
        assert_eq!(log.filled(), 0);
    }

    #[test]
    fn the_first_gap_is_where_to_ask_again() {
        // Restarting on any loss never finishes over a telemetry radio, which is the only kind of
        // link a megabyte log download happens over.
        let mut log = download(270);
        log.receive(0, &[1u8; 90]);
        log.receive(180, &[3u8; 90]);
        assert_eq!(log.first_gap(), Some(90));

        log.receive(90, &[2u8; 90]);
        assert_eq!(log.first_gap(), None);
    }

    #[test]
    fn a_gap_at_the_end_is_found_too() {
        let mut log = download(270);
        log.receive(0, &[1u8; 90]);
        log.receive(90, &[2u8; 90]);
        assert_eq!(log.first_gap(), Some(180));
    }

    #[test]
    fn an_incomplete_download_assembles_to_the_right_length_with_holes() {
        // Not a shorter file whose contents are shifted: a parser can resynchronise past a hole
        // and cannot detect a shift.
        let mut log = download(270);
        log.receive(180, &[3u8; 90]);
        let assembled = log.assemble();
        assert_eq!(assembled.len(), 270);
        assert_eq!(assembled[0], 0, "the hole is zero-filled");
        assert_eq!(assembled[180], 3, "the real data is where it belongs");
    }

    #[test]
    fn progress_is_a_fraction_of_the_declared_size() {
        let mut log = download(180);
        assert!(log.fraction().abs() < f32::EPSILON);
        log.receive(0, &[1u8; 90]);
        assert!((log.fraction() - 0.5).abs() < 0.01);
    }

    #[test]
    fn an_empty_log_is_complete_rather_than_dividing_by_zero() {
        let log = download(0);
        assert!(log.is_complete());
        assert!((log.fraction() - 1.0).abs() < f32::EPSILON);
        assert!(log.assemble().is_empty());
    }

    #[test]
    fn a_log_without_a_clock_says_so_rather_than_claiming_1970() {
        let dated = LogListing {
            id: 1,
            size: 100,
            time_utc: 1_758_000_000,
        };
        let undated = LogListing {
            id: 2,
            size: 100,
            time_utc: 0,
        };
        assert!(dated.has_timestamp());
        assert!(!undated.has_timestamp());
    }
}
