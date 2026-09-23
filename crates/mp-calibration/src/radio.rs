//! Radio calibration: the extremes each channel passed through while the operator swept the
//! sticks and switches.
//!
//! `BUT_Calibrateradio_Click` in `GCSViews/ConfigurationView/ConfigRadioInput.cs:197`, which keeps
//! `rcmin` and `rcmax` per channel and folds each new reading into them (from line 246). The live
//! values it folds in are vehicle state and stay in `mp_vehicle::rc`; this is the recording made
//! from them, which is the calibration.

use mp_vehicle::rc::{CHANNELS, RcChannels, UNAVAILABLE};

/// The minimum and maximum seen on each channel during a calibration.
///
/// Separate from the live values because a calibration is a recording, not a reading: the operator
/// moves every stick and switch to its limits, and what matters afterwards is the extremes that
/// were passed through, not where anything happens to be sitting now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcRange {
    /// Lowest seen, per channel.
    pub minimum: [u16; CHANNELS],
    /// Highest seen, per channel.
    pub maximum: [u16; CHANNELS],
    /// Whether anything has been recorded.
    pub recording: bool,
}

impl Default for RcRange {
    fn default() -> Self {
        Self {
            minimum: [UNAVAILABLE; CHANNELS],
            maximum: [0; CHANNELS],
            recording: false,
        }
    }
}

impl RcRange {
    /// A range that has recorded nothing.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Folds a set of channel values into the range.
    pub fn observe(&mut self, channels: &RcChannels) {
        self.recording = true;
        for number in 1..=CHANNELS {
            let Some(value) = channels.channel(number) else {
                continue;
            };
            let Some(index) = number.checked_sub(1) else {
                continue;
            };
            if let Some(minimum) = self.minimum.get_mut(index) {
                *minimum = (*minimum).min(value);
            }
            if let Some(maximum) = self.maximum.get_mut(index) {
                *maximum = (*maximum).max(value);
            }
        }
    }

    /// The range recorded for one channel, if it moved enough to be worth writing.
    ///
    /// A channel that never moved has a range of nothing, and writing its limits would set a
    /// minimum equal to its maximum - which on a control channel means a stick with no travel and
    /// on a switch means a switch with one position.
    #[must_use]
    pub fn channel(&self, number: usize) -> Option<(u16, u16)> {
        /// Below this the channel did not really move. A radio's noise is a few microseconds; a
        /// real stick travels hundreds.
        const MEANINGFUL_TRAVEL: u16 = 50;

        if number == 0 || number > CHANNELS {
            return None;
        }
        let index = number.checked_sub(1)?;
        let minimum = *self.minimum.get(index)?;
        let maximum = *self.maximum.get(index)?;
        if minimum == UNAVAILABLE || maximum == 0 || maximum <= minimum {
            return None;
        }
        (maximum - minimum >= MEANINGFUL_TRAVEL).then_some((minimum, maximum))
    }

    /// How many channels moved enough to have a range worth writing.
    #[must_use]
    pub fn usable(&self) -> usize {
        (1..=CHANNELS)
            .filter(|n| self.channel(*n).is_some())
            .count()
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
    fn a_range_records_the_extremes_passed_through() {
        // Not where the stick is now. The operator sweeps each control and what matters is the
        // limits it reached.
        let mut range = RcRange::new();
        range.observe(&channels(&[1500]));
        range.observe(&channels(&[1100]));
        range.observe(&channels(&[1900]));
        range.observe(&channels(&[1500]));
        assert_eq!(range.channel(1), Some((1100, 1900)));
    }

    #[test]
    fn a_channel_that_never_moved_has_no_usable_range() {
        // Writing its limits would set a minimum equal to its maximum: a stick with no travel, or
        // a switch with one position.
        let mut range = RcRange::new();
        for _ in 0..10 {
            range.observe(&channels(&[1500, 1100]));
        }
        assert_eq!(range.channel(1), None);
        assert_eq!(range.channel(2), None);
        assert_eq!(range.usable(), 0);
    }

    #[test]
    fn a_small_wobble_is_not_travel() {
        // A radio's noise is a few microseconds; a real stick travels hundreds.
        let mut range = RcRange::new();
        range.observe(&channels(&[1500]));
        range.observe(&channels(&[1508]));
        assert_eq!(range.channel(1), None);
    }

    #[test]
    fn channels_that_are_absent_are_not_recorded() {
        let mut range = RcRange::new();
        range.observe(&channels(&[1100, UNAVAILABLE]));
        range.observe(&channels(&[1900, UNAVAILABLE]));
        assert_eq!(range.channel(1), Some((1100, 1900)));
        assert_eq!(range.channel(2), None);
        assert_eq!(range.usable(), 1);
    }

    #[test]
    fn an_empty_range_is_not_mistaken_for_a_recorded_one() {
        let range = RcRange::new();
        assert!(!range.recording);
        assert_eq!(range.usable(), 0);
        assert_eq!(range.channel(1), None);
    }
}
