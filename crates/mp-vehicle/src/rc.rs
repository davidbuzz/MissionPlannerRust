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

//! Radio control channel values.
//!
//! What the receiver is reporting, in microseconds of pulse width. A radio calibration is
//! essentially watching these while the sticks are moved to their limits, so the channels are kept
//! as raw values rather than as normalised positions: the whole point is to learn what the limits
//! are, and normalising needs the answer first.

/// How many channels to carry.
///
/// ArduPilot calibrates sixteen. `RC_CHANNELS` carries eighteen, but the last two are beyond what
/// the firmware maps to functions, and a calibration screen that offered them would invite setting
/// limits on channels nothing reads.
pub const CHANNELS: usize = 16;

/// A channel value the receiver is not reporting.
///
/// MAVLink uses `UINT16_MAX` for "this channel is not present", which is not the same as a pulse
/// width of zero. Treating it as a number would draw a bar at the far end of the scale for every
/// channel the radio does not have.
pub const UNAVAILABLE: u16 = u16::MAX;

/// Channel values as last reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcChannels {
    /// Pulse widths in microseconds, or [`UNAVAILABLE`].
    pub values: [u16; CHANNELS],
    /// How many channels the receiver says it has.
    pub count: u8,
    /// Receiver signal strength, 0 to 254, or 255 for unknown.
    pub rssi: u8,
    /// Whether anything has been reported at all.
    ///
    /// Distinguishes "no radio connected" from "not heard from yet"; both look like zeros.
    pub reported: bool,
}

impl Default for RcChannels {
    fn default() -> Self {
        Self {
            values: [UNAVAILABLE; CHANNELS],
            count: 0,
            rssi: 255,
            reported: false,
        }
    }
}

impl RcChannels {
    /// One channel's value, or `None` if the receiver is not reporting it.
    ///
    /// Channels are numbered from one, as they are on every transmitter and in every parameter
    /// name. Taking a zero-based index here would mean `RC1_MIN` is written from `values[0]` in
    /// one place and `channel(1)` in another, which is how an off-by-one gets into a calibration.
    #[must_use]
    pub fn channel(&self, number: usize) -> Option<u16> {
        if number == 0 || number > CHANNELS {
            return None;
        }
        let value = *self.values.get(number - 1)?;
        (value != UNAVAILABLE && value != 0).then_some(value)
    }

    /// How many channels are actually reporting a value.
    #[must_use]
    pub fn live(&self) -> usize {
        (1..=CHANNELS)
            .filter(|n| self.channel(*n).is_some())
            .count()
    }

    /// Signal strength as a percentage, or `None` if the receiver does not report it.
    #[must_use]
    pub fn rssi_percent(&self) -> Option<u8> {
        if self.rssi == 255 {
            return None;
        }
        // The field is 0 to 254, not 0 to 100.
        Some(u8::try_from(u32::from(self.rssi) * 100 / 254).unwrap_or(100))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn channels(values: &[u16]) -> RcChannels {
        let mut rc = RcChannels::default();
        for (index, value) in values.iter().enumerate() {
            if let Some(slot) = rc.values.get_mut(index) {
                *slot = *value;
            }
        }
        rc.count = u8::try_from(values.len()).unwrap_or(u8::MAX);
        rc.reported = true;
        rc
    }

    #[test]
    fn channels_are_numbered_from_one() {
        // As they are on every transmitter and in every parameter name. A zero-based index here
        // would mean RC1_MIN is written from values[0] in one place and channel(1) in another.
        let rc = channels(&[1100, 1500, 1900]);
        assert_eq!(rc.channel(1), Some(1100));
        assert_eq!(rc.channel(3), Some(1900));
        assert_eq!(rc.channel(0), None);
        assert_eq!(rc.channel(CHANNELS + 1), None);
    }

    #[test]
    fn a_channel_the_receiver_does_not_have_is_not_a_value() {
        // MAVLink uses UINT16_MAX for absent. Treated as a number it would draw a bar at the far
        // end of the scale for every channel the radio does not have.
        let rc = channels(&[1500, UNAVAILABLE, 0]);
        assert_eq!(rc.channel(1), Some(1500));
        assert_eq!(rc.channel(2), None);
        assert_eq!(rc.channel(3), None, "zero is not a pulse width either");
        assert_eq!(rc.live(), 1);
    }

    #[test]
    fn a_silent_receiver_is_distinguishable_from_one_reporting_zeros() {
        let quiet = RcChannels::default();
        assert!(!quiet.reported);
        assert_eq!(quiet.live(), 0);
    }

    #[test]
    fn rssi_is_a_percentage_of_254_not_of_100() {
        let mut rc = channels(&[1500]);
        rc.rssi = 254;
        assert_eq!(rc.rssi_percent(), Some(100));
        rc.rssi = 127;
        assert_eq!(rc.rssi_percent(), Some(50));
        rc.rssi = 255;
        assert_eq!(rc.rssi_percent(), None, "255 means unknown, not 100%");
    }

    #[test]
    fn sixteen_channels_are_carried_because_that_is_what_the_firmware_maps() {
        // RC_CHANNELS carries eighteen. The last two are beyond what ArduPilot reads, and a
        // calibration screen offering limits on channels nothing looks at is an invitation to set
        // something pointless.
        const _: () = assert!(CHANNELS == 16);
        let mut rc = RcChannels::default();
        for (index, slot) in rc.values.iter_mut().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let value = 1000 + index as u16;
            *slot = value;
        }
        rc.reported = true;
        assert_eq!(rc.channel(16), Some(1015));
        assert_eq!(rc.channel(17), None);
        assert_eq!(rc.live(), 16);
    }
}
