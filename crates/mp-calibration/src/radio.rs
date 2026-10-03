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

//! Radio calibration: `BUT_Calibrateradio_Click`'s arithmetic, and the two messages the Radio
//! Calibration page sends besides its parameter writes.
//!
//! `GCSViews/ConfigurationView/ConfigRadioInput.cs:197-406` keeps three arrays of sixteen floats -
//! `rcmin`, `rcmax` and `rctrim` - folds each reading of `ch1in` to `ch16in` into the first two
//! while the operator sweeps the sticks, constrains the readings taken with the sticks centred into
//! them for the third, and writes `RCn_MIN`, `RCn_MAX` and `RCn_TRIM` for every channel whose three
//! numbers pass one test. [`RadioCalibration`] is those arrays and that arithmetic, exactly: no
//! threshold of travel, no skipping of a channel the receiver does not have - a channel reading
//! zero folds its zero in like any other, and the write test is what leaves it out.
//!
//! The live values it folds in are vehicle state and stay in `mp_vehicle::rc`; [`ch_in`] reads them
//! as `CurrentState`'s `chNin` properties hold them. The form - its message boxes, its button text,
//! its bars - is the GUI's.

use mp_mavlink_dialects::all::{MavMessage, RequestDataStream};
use mp_vehicle::VehicleId;
use mp_vehicle::rc::{CHANNELS, RcChannels, UNAVAILABLE};

/// `rcmin`'s value before anything is folded in.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:30-35`
pub const MIN_START: f32 = 3000.0;
/// `rcmax`'s.
pub const MAX_START: f32 = 0.0;
/// `rctrim`'s.
pub const TRIM_START: f32 = 1500.0;

/// The pulse widths channel 1 must lie strictly between for a reading to be folded in, and for
/// its minimum to be taken as a real radio afterwards.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:244, 322`
const PULSE_FLOOR: f32 = 800.0;
/// The upper bound of that.
const PULSE_CEILING: f32 = 2200.0;

/// `ch1in` to `ch16in` as `CurrentState` holds them: the pulse widths `RC_CHANNELS` sent, as sent -
/// a channel the receiver lacks is whatever the vehicle put there, zero from ArduPilot - and zero
/// for every channel before one has arrived.
///
/// `RC_CHANNELS_RAW` alone fills the first eight and leaves the rest at zero, as the C#'s
/// `ch9in`-`ch16in` stay at their initial zero; its channels are taken as sent except `UINT16_MAX`,
/// which is also the marker `mp_vehicle` leaves in a slot nothing has written, and reads as zero.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3474-3526`
#[must_use]
pub fn ch_in(rc: &RcChannels) -> [f32; CHANNELS] {
    std::array::from_fn(|index| {
        let raw = rc.values.get(index).copied().unwrap_or(UNAVAILABLE);
        if rc.reported || (index < 8 && raw != UNAVAILABLE) {
            f32::from(raw)
        } else {
            0.0
        }
    })
}

/// One channel's three numbers, as the save writes them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelLimits {
    /// The channel, from one: the `n` of `RCn_MIN`.
    pub number: usize,
    /// `rcmin`.
    pub min: f32,
    /// `rcmax`.
    pub max: f32,
    /// `rctrim`.
    pub trim: f32,
}

impl ChannelLimits {
    /// The three writes, in the C#'s order: `RCn_MIN`, `RCn_MAX`, `RCn_TRIM`.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:367-372`
    #[must_use]
    pub fn params(&self) -> [(String, f64); 3] {
        let n = self.number;
        [
            (format!("RC{n}_MIN"), f64::from(self.min)),
            (format!("RC{n}_MAX"), f64::from(self.max)),
            (format!("RC{n}_TRIM"), f64::from(self.trim)),
        ]
    }
}

/// `rcmin`, `rcmax` and `rctrim`: what a calibration recorded.
///
/// Fields of the page in the C#, set once when it is made and never reset, so a second
/// calibration on the same page folds into what the first recorded.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadioCalibration {
    /// `rcmin`, per channel.
    pub min: [f32; CHANNELS],
    /// `rcmax`, per channel.
    pub max: [f32; CHANNELS],
    /// `rctrim`, per channel.
    pub trim: [f32; CHANNELS],
}

impl Default for RadioCalibration {
    /// `rcmin` 3000, `rcmax` 0, `rctrim` 1500, as the constructor sets them.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:29-35`
    fn default() -> Self {
        Self {
            min: [MIN_START; CHANNELS],
            max: [MAX_START; CHANNELS],
            trim: [TRIM_START; CHANNELS],
        }
    }
}

impl RadioCalibration {
    /// Nothing recorded.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// One pass of the loop: when channel 1 reads a pulse strictly between 800 and 2200, every
    /// channel's reading is folded into its minimum and maximum; otherwise nothing is. Whether it
    /// was folded, which is when the C# moves the bars' red lines.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:243-293`
    pub fn observe(&mut self, ch_in: &[f32; CHANNELS]) -> bool {
        let ch1 = ch_in.first().copied().unwrap_or(0.0);
        if !(ch1 > PULSE_FLOOR && ch1 < PULSE_CEILING) {
            return false;
        }
        for ((min, max), reading) in self.min.iter_mut().zip(self.max.iter_mut()).zip(ch_in) {
            *min = min.min(*reading);
            *max = max.max(*reading);
        }
        true
    }

    /// The check once the loop ends: channel 1's minimum is a real pulse. When it is not the C#
    /// says "Bad channel 1 input, canceling" and writes nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:322-329`
    #[must_use]
    pub fn channel_one_good(&self) -> bool {
        let min = self.min.first().copied().unwrap_or(MIN_START);
        min > PULSE_FLOOR && min < PULSE_CEILING
    }

    /// The trims, from a reading taken with the sticks centred and the throttle down: each
    /// channel's reading constrained into its minimum and maximum (`Constrain`, which is
    /// `Math.Min(Math.Max(chin, rcmin), rcmax)`).
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:333-351, 408-411`
    pub fn take_trims(&mut self, ch_in: &[f32; CHANNELS]) {
        for (((trim, min), max), reading) in self
            .trim
            .iter_mut()
            .zip(&self.min)
            .zip(&self.max)
            .zip(ch_in)
        {
            *trim = reading.max(*min).min(*max);
        }
    }

    /// A channel's three numbers if the save writes it: minimum below maximum, neither zero, and
    /// the trim between them and not zero.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:361-377`
    #[must_use]
    pub fn channel(&self, number: usize) -> Option<ChannelLimits> {
        let index = number.checked_sub(1)?;
        let min = *self.min.get(index)?;
        let max = *self.max.get(index)?;
        let trim = *self.trim.get(index)?;
        // The C#'s test, term for term: exact comparisons of the same readings, including the
        // `min != max` its `min < max` already implies.
        #[allow(clippy::float_cmp)]
        let writes = min < max
            && min != 0.0
            && max != 0.0
            && trim <= max
            && trim >= min
            && trim != 0.0
            && min != max;
        writes.then_some(ChannelLimits {
            number,
            min,
            max,
            trim,
        })
    }

    /// Every channel the save writes, channel 1 first.
    #[must_use]
    pub fn writes(&self) -> Vec<ChannelLimits> {
        (1..=CHANNELS)
            .filter_map(|number| self.channel(number))
            .collect()
    }

    /// The summary's `data`: a rule, then `CHn min | max` for each channel the save writes -
    /// including one whose write failed, since the `catch` falls through to the line and only the
    /// test's `continue` skips it.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:353, 374-385`
    #[must_use]
    pub fn data(&self) -> String {
        let mut data = String::from("---------------\n");
        for channel in self.writes() {
            // `"CH" + (a + 1) + " " + rcmin[a] + " | " + rcmax[a]`: a float's `ToString()`, which
            // for the whole pulse widths a radio reports has no decimals, as Rust's has none.
            data.push_str(&format!(
                "CH{} {} | {}\n",
                channel.number, channel.min, channel.max
            ));
        }
        data
    }
}

/// `MAV_CMD_START_RX_PAIR`.
pub const CMD_START_RX_PAIR: u16 = 500;

/// `MAV_DATA_STREAM_RC_CHANNELS`.
pub const DATA_STREAM_RC_CHANNELS: u8 = 3;

/// The three Spektrum bind buttons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Spektrum {
    /// Bind DSM2.
    Dsm2,
    /// Bind DSMX.
    DsmX,
    /// Bind DSM8.
    Dsm8,
}

impl Spektrum {
    /// What each button puts in `START_RX_PAIR`'s second parameter.
    /// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:475, 488, 501`
    #[must_use]
    pub const fn param2(self) -> f32 {
        match self {
            Self::Dsm2 => 0.0,
            Self::DsmX => 1.0,
            Self::Dsm8 => 2.0,
        }
    }
}

/// `doCommand(START_RX_PAIR, 0, dsm, 0, 0, 0, 0, 0)`: puts the receiver in bind mode.
/// `// C#: GCSViews/ConfigurationView/ConfigRadioInput.cs:471-508`
#[must_use]
pub fn start_rx_pair(target: VehicleId, spektrum: Spektrum) -> MavMessage {
    crate::command(
        target,
        CMD_START_RX_PAIR,
        [0.0, spektrum.param2(), 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// `requestDatastream(MAV_DATA_STREAM.RC_CHANNELS, hz)`'s `REQUEST_DATA_STREAM`: the stream
/// started, at `hz`, rate as a byte.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3061-3260`
#[must_use]
pub fn request_rc_channels(target: VehicleId, hz: u8) -> MavMessage {
    MavMessage::RequestDataStream(RequestDataStream {
        req_message_rate: u16::from(hz),
        target_system: target.sysid,
        target_component: target.compid,
        req_stream_id: DATA_STREAM_RC_CHANNELS,
        start_stop: 1,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink_dialects::all::{MavCmd, MavDataStream};

    /// `chNin` from a list of pulse widths, channel 1 first; the rest zero.
    fn inputs(values: &[f32]) -> [f32; CHANNELS] {
        std::array::from_fn(|index| values.get(index).copied().unwrap_or(0.0))
    }

    #[test]
    fn a_calibration_starts_where_the_constructor_puts_it() {
        let calibration = RadioCalibration::new();
        assert_eq!(calibration.min, [3000.0; CHANNELS]);
        assert_eq!(calibration.max, [0.0; CHANNELS]);
        assert_eq!(calibration.trim, [1500.0; CHANNELS]);
        // Nothing folded in: channel 1's minimum is the 3000 it started at.
        assert!(!calibration.channel_one_good());
        assert!(calibration.writes().is_empty());
    }

    #[test]
    fn a_range_records_the_extremes_passed_through() {
        // Not where the stick is now. The operator sweeps each control and what matters is the
        // limits it reached.
        let mut calibration = RadioCalibration::new();
        for ch1 in [1500.0, 1100.0, 1900.0, 1500.0] {
            assert!(calibration.observe(&inputs(&[ch1])));
        }
        assert_eq!(calibration.min[0], 1100.0);
        assert_eq!(calibration.max[0], 1900.0);
        assert!(calibration.channel_one_good());
    }

    #[test]
    fn a_reading_without_a_real_channel_one_is_not_folded_at_all() {
        // Every channel is gated on channel 1, not each on its own.
        let mut calibration = RadioCalibration::new();
        assert!(!calibration.observe(&inputs(&[800.0, 1100.0])));
        assert!(!calibration.observe(&inputs(&[2200.0, 1100.0])));
        assert!(!calibration.observe(&inputs(&[0.0, 1100.0])));
        assert_eq!(calibration, RadioCalibration::new());
        assert!(calibration.observe(&inputs(&[801.0, 1100.0])));
        assert_eq!(calibration.min[1], 1100.0);
    }

    #[test]
    fn a_channel_the_receiver_lacks_folds_its_zero_and_is_not_written() {
        // ArduPilot reports a channel it has no input for as zero, and the C# folds that in like
        // any reading: the minimum goes to zero, and the write test's `rcmin != 0` leaves it out.
        let mut calibration = RadioCalibration::new();
        calibration.observe(&inputs(&[1100.0]));
        calibration.observe(&inputs(&[1900.0]));
        assert_eq!(calibration.min[8], 0.0);
        assert_eq!(calibration.max[8], 0.0);
        calibration.take_trims(&inputs(&[1500.0]));
        assert_eq!(calibration.trim[8], 0.0);
        assert_eq!(calibration.channel(9), None);
        assert_eq!(
            calibration.writes(),
            [ChannelLimits {
                number: 1,
                min: 1100.0,
                max: 1900.0,
                trim: 1500.0
            }]
        );
    }

    #[test]
    fn a_channel_that_never_moved_is_not_written() {
        // Its minimum equals its maximum; the test wants `rcmin < rcmax`.
        let mut calibration = RadioCalibration::new();
        for _ in 0..10 {
            calibration.observe(&inputs(&[1500.0, 1100.0]));
        }
        calibration.take_trims(&inputs(&[1500.0, 1100.0]));
        assert_eq!(calibration.channel(1), None);
        assert_eq!(calibration.channel(2), None);
        assert!(calibration.writes().is_empty());
    }

    #[test]
    fn any_travel_at_all_is_written() {
        // The C# has no threshold: eight microseconds of wobble is a range of eight.
        let mut calibration = RadioCalibration::new();
        calibration.observe(&inputs(&[1500.0]));
        calibration.observe(&inputs(&[1508.0]));
        calibration.take_trims(&inputs(&[1504.0]));
        assert_eq!(
            calibration.channel(1),
            Some(ChannelLimits {
                number: 1,
                min: 1500.0,
                max: 1508.0,
                trim: 1504.0
            })
        );
    }

    #[test]
    fn the_trim_is_constrained_into_the_range() {
        let mut calibration = RadioCalibration::new();
        calibration.observe(&inputs(&[1100.0, 1100.0, 1100.0]));
        calibration.observe(&inputs(&[1900.0, 1900.0, 1900.0]));
        calibration.take_trims(&inputs(&[1000.0, 2000.0, 1520.0]));
        assert_eq!(&calibration.trim[..3], &[1100.0, 1900.0, 1520.0]);
        // A channel never folded (min 3000 above max 0) constrains to its max, zero.
        let mut untouched = RadioCalibration::new();
        untouched.take_trims(&inputs(&[1500.0]));
        assert_eq!(untouched.trim[0], 0.0);
    }

    #[test]
    fn the_writes_are_min_max_and_trim_in_that_order() {
        let limits = ChannelLimits {
            number: 12,
            min: 1100.0,
            max: 1900.0,
            trim: 1500.0,
        };
        assert_eq!(
            limits.params(),
            [
                ("RC12_MIN".to_owned(), 1100.0),
                ("RC12_MAX".to_owned(), 1900.0),
                ("RC12_TRIM".to_owned(), 1500.0),
            ]
        );
    }

    #[test]
    fn the_summary_lists_the_channels_written() {
        let mut calibration = RadioCalibration::new();
        calibration.observe(&inputs(&[1100.0, 1500.0, 1000.0]));
        calibration.observe(&inputs(&[1900.0, 1500.0, 2000.0]));
        calibration.take_trims(&inputs(&[1500.0, 1500.0, 1000.0]));
        assert_eq!(
            calibration.data(),
            "---------------\nCH1 1100 | 1900\nCH3 1000 | 2000\n"
        );
    }

    #[test]
    fn sitls_static_sticks_write_nothing() {
        // SITL's receiver holds ch1-8 at 1500 1500 1000 1500 1800 1000 1000 1800 and reports
        // nothing above eight (ArduPilot's AP_RCProtocol_UDP::set_default_pwm_input_values): no
        // channel moves, so every minimum equals its maximum and the save writes nothing.
        let sitl = inputs(&[
            1500.0, 1500.0, 1000.0, 1500.0, 1800.0, 1000.0, 1000.0, 1800.0,
        ]);
        let mut calibration = RadioCalibration::new();
        for _ in 0..5 {
            calibration.observe(&sitl);
        }
        assert!(calibration.channel_one_good());
        calibration.take_trims(&sitl);
        assert_eq!(calibration.min, sitl);
        assert_eq!(calibration.max, sitl);
        assert_eq!(calibration.trim, sitl);
        assert!(calibration.writes().is_empty());
        assert_eq!(calibration.data(), "---------------\n");
    }

    #[test]
    fn a_second_calibration_folds_into_the_first() {
        // The arrays are the page's fields, set once in its constructor.
        let mut calibration = RadioCalibration::new();
        calibration.observe(&inputs(&[1100.0]));
        calibration.observe(&inputs(&[1500.0]));
        assert_eq!((calibration.min[0], calibration.max[0]), (1100.0, 1500.0));
    }

    #[test]
    fn chnin_is_the_raw_value_the_vehicle_sent() {
        let mut rc = RcChannels::default();
        // Before anything arrives, zero: `chNin`'s initial value, not the "absent" marker.
        assert_eq!(ch_in(&rc), [0.0; CHANNELS]);
        // RC_CHANNELS: every channel as sent, zero included.
        rc.values = [1500; CHANNELS];
        rc.values[3] = 0;
        rc.values[9] = UNAVAILABLE;
        rc.reported = true;
        let read = ch_in(&rc);
        assert_eq!(read[0], 1500.0);
        assert_eq!(read[3], 0.0);
        assert_eq!(read[9], 65535.0);
        // RC_CHANNELS_RAW only: eight channels, the rest zero.
        let mut raw = RcChannels::default();
        raw.values[0] = 1234;
        raw.values[7] = 1800;
        assert_eq!(ch_in(&raw)[0], 1234.0);
        assert_eq!(ch_in(&raw)[7], 1800.0);
        assert_eq!(ch_in(&raw)[1], 0.0);
        assert_eq!(ch_in(&raw)[8], 0.0);
    }

    #[test]
    fn the_bind_buttons_differ_only_in_the_second_parameter() {
        let target = VehicleId::new(1, 1);
        for (spektrum, dsm) in [
            (Spektrum::Dsm2, 0.0),
            (Spektrum::DsmX, 1.0),
            (Spektrum::Dsm8, 2.0),
        ] {
            let MavMessage::CommandLong(long) = start_rx_pair(target, spektrum) else {
                panic!("not a COMMAND_LONG");
            };
            assert_eq!(u32::from(long.command), MavCmd::MAV_CMD_START_RX_PAIR.0);
            assert_eq!(
                [long.param1, long.param2, long.param3, long.param7],
                [0.0, dsm, 0.0, 0.0]
            );
            assert_eq!((long.target_system, long.target_component), (1, 1));
        }
    }

    #[test]
    fn the_stream_request_is_rc_channels_started_at_the_rate() {
        let MavMessage::RequestDataStream(request) = request_rc_channels(VehicleId::new(1, 1), 10)
        else {
            panic!("not a REQUEST_DATA_STREAM");
        };
        assert_eq!(
            u32::from(request.req_stream_id),
            MavDataStream::MAV_DATA_STREAM_RC_CHANNELS.0
        );
        assert_eq!(request.req_message_rate, 10);
        assert_eq!(request.start_stop, 1);
    }
}
