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

//! Packet loss and link health.

/// What a SiK telemetry radio says about the link, from `RADIO` or `RADIO_STATUS`.
///
/// The radio is its own MAVLink system, so its report would land on a vehicle nobody is looking
/// at; the C# hands it to every vehicle on the link instead, and so does
/// [`crate::VehicleRegistry::apply`]. Signal and noise are the radio's own 0 to 255 scale, as
/// sent - the C# keeps them unconverted too.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:2280-2282, 3389-3395`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Radio {
    /// Local signal strength (`rssi`).
    pub rssi: u8,
    /// Remote signal strength (`remrssi`).
    pub remrssi: u8,
    /// How full the radio's transmit buffer is, percent (`txbuffer`).
    pub txbuf: u8,
    /// Receive errors (`rxerrors`).
    pub rxerrors: u16,
    /// Local background noise (`noise`).
    pub noise: u8,
    /// Remote background noise (`remnoise`).
    pub remnoise: u8,
    /// Packets corrected by error correction (`fixedp`).
    pub fixed: u16,
}

/// Link health derived from MAVLink sequence numbers.
///
/// MAVLink stamps every frame with an 8-bit per-sender sequence number, so gaps reveal loss
/// without any cooperation from the vehicle. The subtlety is the wrap: a gap of 250 is far more
/// likely to be a reordered or duplicated packet than 250 genuinely lost frames, so gaps beyond
/// half the sequence space are treated as reordering rather than loss.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkQuality {
    /// Frames received from this sender.
    pub received: u64,
    /// Frames inferred lost from sequence gaps.
    pub lost: u64,
    /// Frames that arrived out of order or duplicated.
    pub out_of_order: u64,
    last_seq: Option<u8>,
}

/// Gaps larger than this are treated as reordering, not loss.
const REORDER_THRESHOLD: u8 = 128;

impl LinkQuality {
    /// Records a received frame's sequence number.
    pub fn record(&mut self, seq: u8) {
        self.received += 1;
        let Some(last) = self.last_seq else {
            self.last_seq = Some(seq);
            return;
        };
        let gap = seq.wrapping_sub(last).wrapping_sub(1);
        // A repeat of the last sequence number, or a gap past half the sequence space, is far
        // more likely to be duplication or reordering than a burst of real loss.
        if seq == last || gap >= REORDER_THRESHOLD {
            self.out_of_order += 1;
        } else if gap > 0 {
            self.lost += u64::from(gap);
        }
        self.last_seq = Some(seq);
    }

    /// Loss as a percentage of frames sent, 0 when nothing has been received.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // counts stay far below f64's exact integer range
    pub fn loss_percent(&self) -> f64 {
        let sent = self.received + self.lost;
        if sent == 0 {
            return 0.0;
        }
        (self.lost as f64 / sent as f64) * 100.0
    }

    /// Forgets history, e.g. after a reconnect where the sequence restarts.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Link quality as Mission Planner's HUD shows it: frames that arrived as a percentage of
    /// frames sent, capped at 100, and 100 before anything has been counted.
    ///
    /// The C# also zeroes it when nothing valid has arrived for ten seconds; that clock is not
    /// kept here, and the HUD's "no vehicle" state covers the same case.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4590-4599`
    #[must_use]
    pub fn quality_percent(&self) -> u8 {
        let sent = self.received + self.lost;
        if sent == 0 {
            return 100;
        }
        let quality = self.received.saturating_mul(100) / sent;
        u8::try_from(quality.min(100)).unwrap_or(100)
    }
}

#[cfg(test)]
mod quality_tests {
    use super::*;

    #[test]
    fn quality_is_the_share_of_frames_that_arrived() {
        let mut link = LinkQuality::default();
        for seq in [0u8, 1, 2, 3, 6, 7, 8, 9] {
            link.record(seq);
        }
        // Eight received, two lost (4 and 5): 8 / 10.
        assert_eq!(link.received, 8);
        assert_eq!(link.lost, 2);
        assert_eq!(link.quality_percent(), 80);
    }

    #[test]
    fn a_perfect_link_and_an_empty_one_both_read_full() {
        let mut link = LinkQuality::default();
        assert_eq!(link.quality_percent(), 100);
        for seq in 0..50u8 {
            link.record(seq);
        }
        assert_eq!(link.quality_percent(), 100);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clean_stream_reports_no_loss() {
        let mut q = LinkQuality::default();
        for seq in 0..=255u8 {
            q.record(seq);
        }
        assert_eq!(q.received, 256);
        assert_eq!(q.lost, 0);
        assert!(q.loss_percent() < f64::EPSILON);
    }

    #[test]
    fn gaps_are_counted_as_loss() {
        let mut q = LinkQuality::default();
        q.record(0);
        q.record(3); // 1 and 2 missing
        assert_eq!(q.lost, 2);
        assert_eq!(q.received, 2);
        assert!((q.loss_percent() - 50.0).abs() < 1e-9);
    }

    #[test]
    fn the_sequence_wrap_is_not_a_burst_of_loss() {
        let mut q = LinkQuality::default();
        q.record(254);
        q.record(255);
        q.record(0);
        q.record(1);
        assert_eq!(q.lost, 0, "wrapping from 255 to 0 is normal");
    }

    #[test]
    fn reordering_is_distinguished_from_loss() {
        let mut q = LinkQuality::default();
        q.record(10);
        q.record(9); // arrived late
        assert_eq!(q.lost, 0);
        assert_eq!(q.out_of_order, 1);

        q.record(9); // duplicate
        assert_eq!(q.out_of_order, 2);
    }

    #[test]
    fn loss_across_a_wrap_boundary_is_still_loss() {
        let mut q = LinkQuality::default();
        q.record(253);
        q.record(2); // 254, 255, 0, 1 missing
        assert_eq!(q.lost, 4);
    }
}
