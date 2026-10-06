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

//! Turning stick positions into RC channel values, as Mission Planner does it.
//!
//! `JoystickBase.pickchannel` makes one channel's value: the axis the channel is given, read as
//! 0 to 65535, mapped onto -500 to 500, reversed if asked, put through `Expo` onto the channel's
//! `RCn_MIN`, `RCn_TRIM` and `RCn_MAX` - or onto -1000, 0 and 1000 for `MANUAL_CONTROL` - and held
//! inside them. `mainloop` does that for every channel given an axis, mixing channels 1 and 2 when
//! Elevons is ticked, and `MainV2.joysticksend` puts the values in `RC_CHANNELS_OVERRIDE`, a channel
//! with no axis sent as "ignore this field", or in `MANUAL_CONTROL`. That is [`Mapping`],
//! [`Mapping::overrides`], [`Overrides::channels`] and [`Overrides::manual`] here, with the C#'s
//! arithmetic - `double` where it has `double`, truncating casts where it casts - so a stick
//! position gives the value Mission Planner would send.
//!
//! One thing is added, and it is not the C#'s: a position is never sent as one of the message's
//! control values ([`SAFE_MIN_US`], [`SAFE_MAX_US`]).
//! `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:729-996, 1030-1147; MainV2.cs:2279-2449`

use crate::Reading;
use crate::config::{ButtonFunction, JoyButton, JoystickAxis, JoystickConfig};

/// How many channels `RC_CHANNELS_OVERRIDE` carries.
pub const CHANNELS: usize = 18;

/// A channel value meaning "ignore this field", for channels 1 to 8.
const IGNORE_LOW: u16 = u16::MAX;
/// A channel value meaning "release this channel to the transmitter", for channels 1 to 8.
const RELEASE_LOW: u16 = 0;
/// A channel value meaning "ignore this field", for channels 9 to 18.
const IGNORE_HIGH: u16 = 0;
/// A channel value meaning "release this channel to the transmitter", for channels 9 to 18.
///
/// The high channels were added as MAVLink v2 extensions and use the opposite convention to the
/// low ones: 0 means ignore rather than release, and release is `UINT16_MAX - 1`. Getting this
/// backwards produces a failsafe that sends what it believes is a release, the vehicle reads as
/// "ignore", and the channel keeps whatever it was overridden to - a failsafe that does nothing
/// and reports success.
const RELEASE_HIGH: u16 = u16::MAX - 1;

/// The lowest value a mapped channel may carry.
///
/// Not zero. Zero means "release this channel back to the transmitter" on channels 1 to 8, so a
/// stick position of zero is a failsafe, not a stick position. One microsecond is far outside any
/// real RC range and is only reached by a vehicle whose `RCn_MIN` is nonsense.
pub const SAFE_MIN_US: u16 = 1;
/// The highest value a mapped channel may carry.
///
/// `UINT16_MAX - 2`, because `UINT16_MAX` means "ignore this field" and `UINT16_MAX - 1` means
/// "release" on the high channels.
pub const SAFE_MAX_US: u16 = u16::MAX - 2;

/// The centre of an RC channel's normal range, in microseconds: `pickchannel`'s trim when the
/// vehicle has not listed `RCn_TRIM`.
pub const CENTRE_US: u16 = 1500;
/// The bottom of an RC channel's normal range: `pickchannel`'s default `min`.
pub const MIN_US: u16 = 1000;
/// The top of an RC channel's normal range: `pickchannel`'s default `max`.
pub const MAX_US: u16 = 2000;

/// `ushort.MaxValue / 2`: an axis at rest, an axis the device does not have, and where the hats
/// and the button axes start.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:33-36, 784; MyJoystickState.cs:24-28`
pub const AXIS_CENTRE: i32 = 32_767;

/// Eighteen channel values, ready to send.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channels(pub [u16; CHANNELS]);

impl Channels {
    /// Every channel ignored, which is what a channel with no axis is sent as: `ushort.MaxValue`
    /// on channels 1 to 8 and 0 on 9 to 18.
    /// `// C#: MainV2.cs:2289-2324`
    #[must_use]
    pub const fn ignored() -> Self {
        // Written out rather than looped, so the split between the two conventions is visible on
        // the page. It is the one thing about this message that is easy to get wrong.
        Self([
            IGNORE_LOW,
            IGNORE_LOW,
            IGNORE_LOW,
            IGNORE_LOW,
            IGNORE_LOW,
            IGNORE_LOW,
            IGNORE_LOW,
            IGNORE_LOW,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
            IGNORE_HIGH,
        ])
    }

    /// Every channel centred. Used by the tests; a real mapping produces positions.
    #[must_use]
    pub const fn centred() -> Self {
        Self([CENTRE_US; CHANNELS])
    }

    /// Every channel handed back to the transmitter.
    ///
    /// What a failsafe sends. Both conventions, each on its own half. Mission Planner's
    /// `clearRCOverride` sends zero on all eighteen, which on channels 9 to 18 is "ignore" and
    /// leaves them overridden; the release here is the protocol's.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:294-364`
    #[must_use]
    pub const fn release() -> Self {
        Self([
            RELEASE_LOW,
            RELEASE_LOW,
            RELEASE_LOW,
            RELEASE_LOW,
            RELEASE_LOW,
            RELEASE_LOW,
            RELEASE_LOW,
            RELEASE_LOW,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
            RELEASE_HIGH,
        ])
    }

    /// Sets one channel, numbered from 1 as a person and the protocol both number them.
    ///
    /// Out of range does nothing.
    pub fn set(&mut self, channel: usize, value: u16) {
        if let Some(slot) = channel
            .checked_sub(1)
            .and_then(|index| self.0.get_mut(index))
        {
            *slot = value;
        }
    }

    /// Reads one channel, numbered from 1.
    #[must_use]
    pub fn get(&self, channel: usize) -> Option<u16> {
        channel
            .checked_sub(1)
            .and_then(|index| self.0.get(index))
            .copied()
    }
}

/// `RCn_MIN`, `RCn_MAX` and `RCn_TRIM`, as `pickchannel` reads them from the vehicle's parameters.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RcRanges {
    /// `Interface.MAV.param.Count > 0`: the vehicle has listed any parameter at all.
    pub listed: bool,
    /// By channel number: `(min, max, trim)` for a channel whose three the vehicle has, `None`
    /// for one it lacks `RCn_MIN` for - or has `RCn_MIN` and lacks another, where the C#'s
    /// indexer throws and its `catch` takes the defaults.
    pub channels: Vec<Option<(i32, i32, i32)>>,
}

impl RcRanges {
    /// The ranges from a vehicle's parameters, `lookup` giving a parameter's value by name.
    /// `(int)(float)` of each, as the C# casts them.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:733-756`
    #[must_use]
    pub fn from_lookup(listed: bool, lookup: impl Fn(&str) -> Option<f64>) -> Self {
        // `(int)(float)`, as the C# casts: out of `int`'s range that is `int.MinValue`, not the
        // saturated value `as` would give.
        #[allow(clippy::cast_possible_truncation)] // to single precision first, as the C# does
        let cast = |value: f64| csharp_int(f64::from(value as f32));
        let channels = (0..=CHANNELS)
            .map(|channel| {
                let min = lookup(&format!("RC{channel}_MIN"))?;
                let max = lookup(&format!("RC{channel}_MAX"))?;
                let trim = lookup(&format!("RC{channel}_TRIM"))?;
                Some((cast(min), cast(max), cast(trim)))
            })
            .collect();
        Self { listed, channels }
    }

    /// A channel's `(min, max, trim)`: the vehicle's, or 1000, 2000 and 1500.
    #[must_use]
    pub fn of(&self, channel: usize) -> (i32, i32, i32) {
        let default = (i32::from(MIN_US), i32::from(MAX_US), i32::from(CENTRE_US));
        if !self.listed {
            return default;
        }
        self.channels
            .get(channel)
            .copied()
            .flatten()
            .unwrap_or(default)
    }
}

/// What the joystick object changes as it is flown, which `pickchannel` reads: the hat switch's
/// two axes and the two button axes.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:32-36`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Runtime {
    /// `hat1`, which the hat's up and down move - on Windows. Mission Planner's Linux joystick
    /// has no hat (`getNumberPOV` is 0), so here it stays where it starts.
    pub hat1: i32,
    /// `hat2`, the hat's left and right.
    pub hat2: i32,
    /// `custom0`: the `Custom1` axis, set by the `Button_axis0` buttons.
    pub custom0: i32,
    /// `custom1`: the `Custom2` axis, set by the `Button_axis1` buttons.
    pub custom1: i32,
}

impl Default for Runtime {
    /// "set to default midpoint": all four at `65535/2` - which the button axes then read as a
    /// PWM, and hold at the top of their channel until a button is pressed.
    fn default() -> Self {
        Self {
            hat1: AXIS_CENTRE,
            hat2: AXIS_CENTRE,
            custom0: AXIS_CENTRE,
            custom1: AXIS_CENTRE,
        }
    }
}

/// A `double` cast to `int` as .NET does it on x86 and x64: truncated toward zero, and
/// `int.MinValue` for anything the `int` cannot hold, NaN included.
#[allow(clippy::cast_possible_truncation)]
fn csharp_int(value: f64) -> i32 {
    if value.is_finite() && value > f64::from(i32::MIN) - 1.0 && value < f64::from(i32::MAX) + 1.0 {
        value as i32
    } else {
        i32::MIN
    }
}

/// `map`: `x` taken from one range onto another, in `double`.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:984-987`
#[must_use]
pub fn map(x: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> f64 {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

/// `Expo`: an input of -500 to 500 onto `min`, `mid` and `max`, softened by `expo` percent.
///
/// Not a cubic: the linear value less `expo`% of a tent that rises to 250 at half stick and
/// falls back to zero at full stick, so the ends stay the ends and the middle moves less.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:935-982`
#[must_use]
pub fn expo(input: f64, expo: f64, min: f64, max: f64, mid: f64) -> f64 {
    let expomult = expo / 100.0;
    if input >= 0.0 {
        let linearpwm = map(input, 0.0, 500.0, mid, max);
        // over half way though input
        let factor = if input > 250.0 {
            250.0 - (input - 250.0)
        } else {
            input
        };
        linearpwm - factor * expomult
    } else {
        let linearpwm = map(input, -500.0, 0.0, min, mid);
        let factor = if input < -250.0 {
            -250.0 - (input + 250.0)
        } else {
            input
        };
        linearpwm - factor * expomult
    }
}

/// A `js` axis as Mission Planner's Linux joystick holds it: the kernel's -32767 to 32767 moved
/// up by `65535/2`, so 0 to 65534, centre 32767; and 32767 for an axis the device has not
/// reported.
///
/// Two differences from the C#, both in what the reading holds rather than here. Its `(ushort)`
/// cast turns the kernel's -32768 into 65535, full deflection the other way; the reading clamps it
/// to -32767, the bottom. And it ignores the position an axis's `JS_EVENT_INIT` carries, so every
/// axis starts centred until it moves; the reading takes it (`crate::event`), so a throttle held
/// up when the device opens reads up.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickLinux.cs:121-123, 187-223; MyJoystickState.cs:13-29`
#[must_use]
pub fn js_axis(reading: &Reading, number: usize) -> i32 {
    reading.axes.get(number).map_or(AXIS_CENTRE, |value| {
        #[allow(clippy::cast_possible_truncation)] // a value of -1.0 to 1.0, times 32767
        let raw = (f64::from(*value) * 32_767.0).round() as i32;
        raw + AXIS_CENTRE
    })
}

/// What a named axis reads, 0 to 65535, on Mission Planner's Linux joystick: `X`, `Y`, `Z`, `Rx`,
/// `Ry` and `Rz` are `js` axes 0 to 5, `Slider1` is 6, `AZ`, `AY` and `AX` are 7, 8 and 9, and
/// `Slider2` is always centred. The rest are properties it never sets, so they read 0 - the bottom
/// of their channel. `None` for the axes that are not the device's.
/// `// C#: ExtLibs/ArduPilot/Joystick/MyJoystickState.cs:31-100`
#[must_use]
pub fn axis_value(axis: JoystickAxis, reading: &Reading) -> Option<i32> {
    use JoystickAxis as A;
    let number = match axis {
        A::X => 0,
        A::Y => 1,
        A::Z => 2,
        A::Rx => 3,
        A::Ry => 4,
        A::Rz => 5,
        A::Slider1 => 6,
        A::AZ => 7,
        A::AY => 8,
        A::AX => 9,
        A::Slider2 => return Some(AXIS_CENTRE),
        A::ARx
        | A::ARy
        | A::ARz
        | A::FRx
        | A::FRy
        | A::FRz
        | A::FX
        | A::FY
        | A::FZ
        | A::VRx
        | A::VRy
        | A::VRz
        | A::VX
        | A::VY
        | A::VZ => return Some(0),
        A::None | A::Pass | A::Hatud1 | A::Hatlr2 | A::Custom1 | A::Custom2 | A::Uint16Max => {
            return None;
        }
    };
    Some(js_axis(reading, number))
}

/// Everything `pickchannel` reads besides the channel's own settings.
#[derive(Debug, Clone, Copy)]
pub struct Inputs<'a> {
    /// The device's axes and buttons.
    pub reading: &'a Reading,
    /// The vehicle's `RCn_*`.
    pub ranges: &'a RcRanges,
    /// `manual_control`: -1000 to 1000 rather than the channel's range.
    pub manual_control: bool,
    /// The hats and the button axes.
    pub runtime: Runtime,
}

/// `pickchannel`: one channel's value from its axis, reverse and expo.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:729-933`
#[must_use]
pub fn pick_channel(
    channel: usize,
    axis: JoystickAxis,
    reverse: bool,
    expo_percent: i32,
    inputs: Inputs<'_>,
) -> i16 {
    let (mut min, mut max, mut trim) = inputs.ranges.of(channel);
    if inputs.manual_control {
        min = -1000;
        max = 1000;
        trim = 0;
    }
    // The C#'s `int` arithmetic is unchecked: a vehicle's nonsense `RC3_MAX` wraps there, where
    // plain `+` and `-` would panic a debug build's send thread - and with it the release.
    if channel == 3 {
        trim = min.wrapping_add(max) / 2;
    }
    let range = max.wrapping_sub(min).wrapping_abs();
    // `(float)(x - min) / range * ushort.MaxValue`, in single precision as the C# has it.
    let scaled = |value: i32| {
        #[allow(clippy::cast_precision_loss)] // PWM values, far inside f32's exact integers
        let fraction = value.wrapping_sub(min) as f32 / range as f32;
        csharp_int(f64::from(fraction * f32::from(u16::MAX)))
    };
    let working = match axis {
        JoystickAxis::None => AXIS_CENTRE,
        JoystickAxis::Pass => scaled(trim),
        JoystickAxis::Hatud1 => inputs.runtime.hat1.clamp(0, 65_535),
        JoystickAxis::Hatlr2 => inputs.runtime.hat2.clamp(0, 65_535),
        JoystickAxis::Custom1 => scaled(inputs.runtime.custom0).clamp(0, 65_535),
        JoystickAxis::Custom2 => scaled(inputs.runtime.custom1).clamp(0, 65_535),
        JoystickAxis::Uint16Max => return -1,
        named => axis_value(named, inputs.reading).unwrap_or(0),
    };
    // between 0 and 65535 - convert to int -500 to 500
    let mut working = csharp_int(map(f64::from(working), 0.0, 65_535.0, -500.0, 500.0));
    if reverse {
        working = working.wrapping_neg();
    }
    working = csharp_int(expo(
        f64::from(working),
        f64::from(expo_percent),
        f64::from(min),
        f64::from(max),
        f64::from(trim),
    ));
    // `Math.Max(min, working)` then `Math.Min(max, working)`: not a clamp, which would refuse a
    // vehicle whose `RCn_MIN` is above its `RCn_MAX`.
    working = working.max(min).min(max);
    #[allow(clippy::cast_possible_truncation)] // `(short)`, unchecked, as the C# casts
    let value = working as i16;
    value
}

/// `BOOL_TO_SIGN`: -1 for true.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:282-292`
const fn bool_to_sign(input: bool) -> i32 {
    if input { -1 } else { 1 }
}

/// What the sticks fly with: the configuration, Elevons, Manual Control and the vehicle's ranges.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Mapping {
    /// `JoyChannels` and `JoyButtons`.
    pub config: JoystickConfig,
    /// `elevons`: channels 1 and 2 mixed.
    pub elevons: bool,
    /// `manual_control`: `MANUAL_CONTROL`, -1000 to 1000, rather than `RC_CHANNELS_OVERRIDE`.
    pub manual_control: bool,
    /// The vehicle's `RCn_*`.
    pub ranges: RcRanges,
}

/// The values `mainloop` puts in `cs.rcoverridech1` to `18`, and which of them it sets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Overrides {
    /// By channel, 1 first.
    pub values: [i16; CHANNELS],
    /// Whether the channel has an axis, and so is sent.
    pub mapped: [bool; CHANNELS],
}

/// `MANUAL_CONTROL`'s four axes: channels 1 to 4's values, 0 for a channel with no axis.
/// `// C#: MainV2.cs:2416-2427`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ManualControl {
    /// `x`, from channel 1.
    pub x: i16,
    /// `y`, from channel 2.
    pub y: i16,
    /// `z`, from channel 3.
    pub z: i16,
    /// `r`, from channel 4.
    pub r: i16,
}

impl Overrides {
    /// A channel's value, if it is sent.
    #[must_use]
    pub fn get(&self, channel: usize) -> Option<i16> {
        let index = channel.checked_sub(1)?;
        if *self.mapped.get(index)? {
            self.values.get(index).copied()
        } else {
            None
        }
    }

    /// `RC_CHANNELS_OVERRIDE`'s eighteen fields: `(ushort)` each channel's value, and "ignore"
    /// for a channel with no axis.
    ///
    /// A value is held inside [`SAFE_MIN_US`] to [`SAFE_MAX_US`] - except 65535, which is the
    /// `UINT16_MAX` axis asking for "ignore" by name. The C# sends whatever `(ushort)` makes, so a
    /// vehicle listing `RCn_MIN` as 0 would have a stick at the bottom of its travel sent as 0,
    /// "give this channel back to the transmitter": a release nobody asked for.
    /// `// C#: MainV2.cs:2283-2361`
    #[must_use]
    pub fn channels(&self) -> Channels {
        let mut channels = Channels::ignored();
        for (index, (value, mapped)) in self.values.iter().zip(self.mapped).enumerate() {
            if !mapped {
                continue;
            }
            #[allow(clippy::cast_sign_loss)] // `(ushort)`, as the C# casts
            let raw = *value as u16;
            let sent = if raw == u16::MAX {
                raw
            } else {
                raw.clamp(SAFE_MIN_US, SAFE_MAX_US)
            };
            channels.set(index + 1, sent);
        }
        channels
    }

    /// `MANUAL_CONTROL`'s axes.
    /// `// C#: MainV2.cs:2416-2427`
    #[must_use]
    pub fn manual(&self) -> ManualControl {
        let axis = |channel| self.get(channel).unwrap_or(0);
        ManualControl {
            x: axis(1),
            y: axis(2),
            z: axis(3),
            r: axis(4),
        }
    }
}

impl Mapping {
    fn inputs<'a>(&'a self, reading: &'a Reading, runtime: Runtime) -> Inputs<'a> {
        Inputs {
            reading,
            ranges: &self.ranges,
            manual_control: self.manual_control,
            runtime,
        }
    }

    /// `pickchannel` with a channel's own settings.
    fn pick(&self, channel: usize, reverse: Option<bool>, inputs: Inputs<'_>) -> i16 {
        let config = self.config.channel(channel);
        pick_channel(
            channel,
            config.axis,
            reverse.unwrap_or(config.reverse),
            config.expo,
            inputs,
        )
    }

    /// `getValueForChannel`: one channel's value with its settings, not mixed - what the page's
    /// bars show while the sticks are not flying.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1149-1160`
    #[must_use]
    pub fn value_for_channel(&self, channel: usize, reading: &Reading, runtime: Runtime) -> i16 {
        self.pick(channel, None, self.inputs(reading, runtime))
    }

    /// `mainloop`'s values: every channel with an axis, channels 1 and 2 mixed as elevons when
    /// Elevons is ticked.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1068-1126`
    #[must_use]
    pub fn overrides(&self, reading: &Reading, runtime: Runtime) -> Overrides {
        let inputs = self.inputs(reading, runtime);
        let mut values = [0i16; CHANNELS];
        let mut mapped = [false; CHANNELS];
        for channel in 1..=CHANNELS {
            if self.config.channel(channel).axis == JoystickAxis::None {
                continue;
            }
            let (Some(value), Some(sent)) =
                (values.get_mut(channel - 1), mapped.get_mut(channel - 1))
            else {
                continue;
            };
            *sent = true;
            *value = if self.elevons && channel <= 2 {
                let roll = i32::from(self.pick(1, Some(false), inputs));
                let pitch = i32::from(self.pick(2, Some(false), inputs));
                let reverse = bool_to_sign(self.config.channel(channel).reverse);
                let mixed = if channel == 1 {
                    reverse * ((pitch - 1500) - (roll - 1500)) / 2 + 1500
                } else {
                    reverse * ((pitch - 1500) + (roll - 1500)) / 2 + 1500
                };
                #[allow(clippy::cast_possible_truncation)] // `(short)`, as the C# casts
                let mixed = mixed as i16;
                mixed
            } else {
                self.pick(channel, None, inputs)
            };
        }
        Overrides { values, mapped }
    }
}

/// A button function pressed or let go, for the application to do: every function but the two
/// button axes, which [`ButtonTracker`] does itself because they only move a channel.
#[derive(Debug, Clone, PartialEq)]
pub struct ButtonEvent {
    /// The function's slot in `JoyButtons`.
    pub slot: usize,
    /// The function's settings when it fired.
    pub button: JoyButton,
    /// Pressed, or let go.
    pub down: bool,
}

/// `getButtonState`'s memory: `buttonpressed`, one per device button.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:21, 673-690`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ButtonTracker {
    pressed: Vec<bool>,
}

impl Default for ButtonTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl ButtonTracker {
    /// Nothing pressed, as a new joystick object starts.
    #[must_use]
    pub fn new() -> Self {
        Self {
            pressed: vec![false; crate::config::BUTTON_SLOTS],
        }
    }

    /// `DoJoystickButtonFunction`: each function with a button, in order, checked for a press
    /// or a let-go of its button, and `ProcessButtonEvent` for each - the button axes set here,
    /// everything else returned.
    ///
    /// The memory is the device button's, not the function's, as the C#'s is: two functions on
    /// one button see one press between them, the first in the list.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:366-624, 673-700`
    pub fn process(
        &mut self,
        config: &JoystickConfig,
        buttons: &[bool],
        runtime: &mut Runtime,
    ) -> Vec<ButtonEvent> {
        let mut events = Vec::new();
        for (slot, button) in config.buttons.iter().enumerate() {
            if button.buttonno == -1 {
                continue;
            }
            // `buts[buttonno]` beyond the 128 the state holds throws, and the loop's `catch` ends
            // this pass.
            let Some(pressed) = usize::try_from(button.buttonno)
                .ok()
                .and_then(|number| self.pressed.get_mut(number))
            else {
                break;
            };
            let now = usize::try_from(button.buttonno)
                .ok()
                .and_then(|number| buttons.get(number))
                .copied()
                .unwrap_or(false);
            let down = now && !*pressed;
            let up = !now && *pressed;
            *pressed = now;
            if (down || up)
                && let Some(event) = process_button_event(slot, button, down, runtime)
            {
                events.push(event);
            }
        }
        events
    }
}

/// `ProcessButtonEvent`: a let-go matters only to `Do_Set_Relay` and the button axes; the button
/// axes set their axis to `p2` while held and `p1` when let go.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:377-392, 584-621`
fn process_button_event(
    slot: usize,
    button: &JoyButton,
    down: bool,
    runtime: &mut Runtime,
) -> Option<ButtonEvent> {
    if !down
        && !matches!(
            button.function,
            ButtonFunction::DoSetRelay | ButtonFunction::ButtonAxis0 | ButtonFunction::ButtonAxis1
        )
    {
        return None;
    }
    #[allow(clippy::cast_possible_truncation)] // `(int) but.p1`, as the C# casts
    let (pwmmin, pwmmax) = (button.p1 as i32, button.p2 as i32);
    match button.function {
        ButtonFunction::ButtonAxis0 => {
            runtime.custom0 = if down { pwmmax } else { pwmmin };
            None
        }
        ButtonFunction::ButtonAxis1 => {
            runtime.custom1 = if down { pwmmax } else { pwmmin };
            None
        }
        _ => Some(ButtonEvent {
            slot,
            button: button.clone(),
            down,
        }),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    /// A reading with axis `number` at a `js` value, everything else unreported.
    fn stick(number: usize, value: i16) -> Reading {
        let mut axes = vec![0.0; number + 1];
        axes[number] = f32::from(value) / 32_767.0;
        Reading {
            axes,
            buttons: Vec::new(),
        }
    }

    fn inputs<'a>(reading: &'a Reading, ranges: &'a RcRanges) -> Inputs<'a> {
        Inputs {
            reading,
            ranges,
            manual_control: false,
            runtime: Runtime::default(),
        }
    }

    fn pick(axis: JoystickAxis, value: i16, reverse: bool, expo_percent: i32) -> i16 {
        let reading = stick(0, value);
        let ranges = RcRanges::default();
        pick_channel(1, axis, reverse, expo_percent, inputs(&reading, &ranges))
    }

    /// The two halves of the message use opposite conventions, and a failsafe that gets it
    /// backwards sends a release the vehicle reads as "ignore".
    #[test]
    fn release_uses_the_right_convention_on_each_half() {
        let release = Channels::release();
        for channel in 1..=8 {
            assert_eq!(release.get(channel), Some(0), "channel {channel}");
        }
        for channel in 9..=18 {
            assert_eq!(
                release.get(channel),
                Some(u16::MAX - 1),
                "channel {channel}"
            );
        }
    }

    /// And so does "ignore", the other way round: `joysticksend`'s `ushort.MaxValue` and 0.
    #[test]
    fn ignore_uses_the_right_convention_on_each_half() {
        let ignored = Channels::ignored();
        for channel in 1..=8 {
            assert_eq!(ignored.get(channel), Some(u16::MAX), "channel {channel}");
        }
        for channel in 9..=18 {
            assert_eq!(ignored.get(channel), Some(0), "channel {channel}");
        }
    }

    /// A centred stick is the trim: 32767 maps to -0.0076, which the `(int)` truncates to 0.
    #[test]
    fn a_centred_axis_is_the_trim() {
        assert_eq!(pick(JoystickAxis::X, 0, false, 0), 1500);
    }

    /// The ends as the C#'s arithmetic has them: full forward is 65534 of 65535, so 499 of 500 -
    /// 1999, not 2000 - and full back is 0, so 1000. Half stick is 16384 + 32767 = 49151, which
    /// `map` makes 249.996 and the `(int)` 249: 1749, not 1750.
    #[test]
    fn the_extremes_are_the_csharps() {
        assert_eq!(pick(JoystickAxis::X, 32_767, false, 0), 1999);
        assert_eq!(pick(JoystickAxis::X, -32_767, false, 0), 1000);
        assert_eq!(pick(JoystickAxis::X, 16_384, false, 0), 1749);
    }

    /// Reverse is the -500 to 500 value negated before the expo: half stick's 249 is -249, 1251.
    #[test]
    fn reversing_an_axis_swaps_its_ends() {
        assert_eq!(pick(JoystickAxis::X, 32_767, true, 0), 1001);
        assert_eq!(pick(JoystickAxis::X, -32_767, true, 0), 2000);
        assert_eq!(pick(JoystickAxis::X, 16_384, true, 0), 1251);
    }

    /// `Expo`: half stick at 50% is 249 PWM out less 124.5 of tent, either way - 1624.5 and
    /// 1375.5, which the `(int)` truncates; the ends stay put.
    #[test]
    fn expo_takes_a_tent_off_the_middle_and_leaves_the_ends() {
        assert_eq!(pick(JoystickAxis::X, 16_384, false, 50), 1624);
        assert_eq!(pick(JoystickAxis::X, 16_384, true, 50), 1375);
        assert_eq!(pick(JoystickAxis::X, 0, false, 50), 1500);
        assert_eq!(pick(JoystickAxis::X, -32_767, false, 100), 1000);
        assert!((expo(500.0, 100.0, 1000.0, 2000.0, 1500.0) - 2000.0).abs() < 1e-9);
        assert!((expo(250.0, 100.0, 1000.0, 2000.0, 1500.0) - 1500.0).abs() < 1e-9);
        assert!((expo(-100.0, 30.0, 1000.0, 2000.0, 1500.0) - 1430.0).abs() < 1e-9);
    }

    /// The vehicle's `RCn_*` are the range; a channel without all three takes the defaults, and
    /// throttle's trim is the middle of its range whatever `RC3_TRIM` says.
    #[test]
    fn the_vehicles_ranges_are_used_and_throttle_is_centred() {
        let params = [
            ("RC1_MIN", 1100.0),
            ("RC1_MAX", 1900.0),
            ("RC1_TRIM", 1520.0),
            ("RC2_MIN", 1100.0),
            ("RC3_MIN", 1000.0),
            ("RC3_MAX", 2000.0),
            ("RC3_TRIM", 1000.0),
        ];
        let ranges = RcRanges::from_lookup(true, |name| {
            params.iter().find(|(n, _)| *n == name).map(|(_, v)| *v)
        });
        assert_eq!(ranges.of(1), (1100, 1900, 1520));
        assert_eq!(ranges.of(2), (1000, 2000, 1500), "RC2_MAX is missing");
        let reading = stick(0, 0);
        let centre = |channel| {
            pick_channel(
                channel,
                JoystickAxis::X,
                false,
                0,
                inputs(&reading, &ranges),
            )
        };
        assert_eq!(centre(1), 1520);
        assert_eq!(centre(3), 1500, "trim = (min + max) / 2 on channel 3");
        let unlisted = RcRanges {
            listed: false,
            ..ranges
        };
        assert_eq!(
            unlisted.of(1),
            (1000, 2000, 1500),
            "no parameters, no ranges"
        );
    }

    /// Manual Control's range is -1000 to 1000 around 0: half stick's 249 of 500 is 498.
    #[test]
    fn manual_control_is_minus_a_thousand_to_a_thousand() {
        let ranges = RcRanges::default();
        for (value, expected) in [(0, 0), (32_767, 998), (-32_767, -1000), (16_384, 498)] {
            let reading = stick(0, value);
            let mut inputs = inputs(&reading, &ranges);
            inputs.manual_control = true;
            assert_eq!(
                pick_channel(1, JoystickAxis::X, false, 0, inputs),
                expected,
                "{value}"
            );
        }
    }

    /// The named axes where the Linux joystick puts them, the rest at zero, and the special ones.
    #[test]
    fn each_axis_reads_where_the_linux_joystick_puts_it() {
        let mut reading = Reading {
            axes: vec![0.0; 10],
            buttons: Vec::new(),
        };
        for (number, axis) in [
            (0, JoystickAxis::X),
            (1, JoystickAxis::Y),
            (2, JoystickAxis::Z),
            (3, JoystickAxis::Rx),
            (4, JoystickAxis::Ry),
            (5, JoystickAxis::Rz),
            (6, JoystickAxis::Slider1),
            (7, JoystickAxis::AZ),
            (8, JoystickAxis::AY),
            (9, JoystickAxis::AX),
        ] {
            reading.axes[number] = 1.0;
            assert_eq!(axis_value(axis, &reading), Some(65_534), "{}", axis.name());
            reading.axes[number] = 0.0;
        }
        assert_eq!(axis_value(JoystickAxis::Slider2, &reading), Some(32_767));
        assert_eq!(axis_value(JoystickAxis::FRx, &reading), Some(0));
        assert_eq!(
            js_axis(&Reading::default(), 3),
            32_767,
            "an axis not reported"
        );
        let ranges = RcRanges::default();
        let at_rest = inputs(&reading, &ranges);
        assert_eq!(pick_channel(1, JoystickAxis::VX, false, 0, at_rest), 1000);
        assert_eq!(pick_channel(1, JoystickAxis::Pass, false, 0, at_rest), 1500);
        assert_eq!(pick_channel(1, JoystickAxis::None, false, 0, at_rest), 1500);
        assert_eq!(
            pick_channel(1, JoystickAxis::Hatud1, false, 0, at_rest),
            1500
        );
        assert_eq!(
            pick_channel(1, JoystickAxis::Uint16Max, false, 0, at_rest),
            -1
        );
        // The button axes start at 65535/2 read as a PWM: the top of the channel.
        assert_eq!(
            pick_channel(1, JoystickAxis::Custom1, false, 0, at_rest),
            2000
        );
        let mut pressed = at_rest;
        pressed.runtime.custom1 = 1200;
        assert_eq!(
            pick_channel(1, JoystickAxis::Custom2, false, 0, pressed),
            1200
        );
    }

    /// Only channels given an axis are sent; the rest are ignored, each half its own way, and
    /// `UINT16_MAX` asks for "ignore" by name.
    #[test]
    fn unmapped_channels_are_ignored_and_mapped_ones_sent() {
        let mut mapping = Mapping::default();
        mapping.config.set_axis(1, JoystickAxis::X);
        mapping.config.set_axis(9, JoystickAxis::Y);
        mapping.config.set_axis(4, JoystickAxis::Uint16Max);
        let reading = stick(0, 16_384);
        let overrides = mapping.overrides(&reading, Runtime::default());
        let channels = overrides.channels();
        assert_eq!(channels.get(1), Some(1749));
        assert_eq!(channels.get(9), Some(1500));
        assert_eq!(channels.get(4), Some(u16::MAX));
        assert_eq!(channels.get(2), Some(u16::MAX), "channel 2 has no axis");
        assert_eq!(channels.get(10), Some(0), "channel 10 has no axis");
        assert_eq!(overrides.get(2), None);
        assert_eq!(
            Mapping::default()
                .overrides(&reading, Runtime::default())
                .channels(),
            Channels::ignored(),
            "a new configuration overrides nothing"
        );
    }

    /// Elevons: channel 1 is (pitch - roll) / 2 and channel 2 (pitch + roll) / 2 about 1500,
    /// each negated by its own reverse.
    #[test]
    fn elevons_mix_channels_one_and_two() {
        let mut mapping = Mapping {
            elevons: true,
            ..Mapping::default()
        };
        mapping.config.set_axis(1, JoystickAxis::X);
        mapping.config.set_axis(2, JoystickAxis::Y);
        // Roll full right (1999), pitch centred (1500).
        let mut reading = stick(0, 32_767);
        reading.axes.push(0.0);
        let overrides = mapping.overrides(&reading, Runtime::default());
        assert_eq!(overrides.get(1), Some(1500 + (0 - 499) / 2));
        assert_eq!(overrides.get(2), Some(1500 + 499 / 2));
        mapping.config.set_reverse(1, true);
        let reversed = mapping.overrides(&reading, Runtime::default());
        assert_eq!(
            reversed.get(1),
            Some(1500 + 499 / 2),
            "sign * (pitch - roll)"
        );
        assert_eq!(
            mapping.value_for_channel(1, &reading, Runtime::default()),
            1001,
            "getValueForChannel is not mixed"
        );
    }

    /// `MANUAL_CONTROL` takes channels 1 to 4; one with no axis is 0.
    #[test]
    fn manual_control_takes_channels_one_to_four() {
        let mut mapping = Mapping {
            manual_control: true,
            ..Mapping::default()
        };
        mapping.config.set_axis(1, JoystickAxis::X);
        mapping.config.set_axis(3, JoystickAxis::X);
        mapping.config.set_reverse(3, true);
        let reading = stick(0, 16_384);
        let manual = mapping.overrides(&reading, Runtime::default()).manual();
        assert_eq!(
            manual,
            ManualControl {
                x: 498,
                y: 0,
                z: -498,
                r: 0
            }
        );
    }

    /// A stick position must never collide with the protocol's control values: 0 is "release" on
    /// channels 1-8 and 65534 on 9-18. A vehicle whose `RC1_MIN` is 0 does not get its channel
    /// released by a stick at the bottom.
    #[test]
    fn a_stick_position_can_never_be_read_as_release() {
        let params = [("RC1_MIN", 0.0), ("RC1_MAX", 65_534.0), ("RC1_TRIM", 0.0)];
        let mut mapping = Mapping {
            ranges: RcRanges::from_lookup(true, |name| {
                params.iter().find(|(n, _)| *n == name).map(|(_, v)| *v)
            }),
            ..Mapping::default()
        };
        mapping.config.set_axis(1, JoystickAxis::X);
        for step in -10..=10 {
            let reading = stick(0, step * 3_276);
            let sent = mapping
                .overrides(&reading, Runtime::default())
                .channels()
                .get(1)
                .unwrap();
            assert_ne!(sent, 0, "0 is 'release' on channels 1-8");
            assert_ne!(sent, u16::MAX - 1, "65534 is 'release' on channels 9-18");
        }
    }

    /// A vehicle's nonsense ranges wrap as the C#'s unchecked `int` does rather than panicking the
    /// send thread: an `RC3_MAX` of three billion is `(int)(float)`'s `int.MinValue`, and the
    /// throttle's trim and range are worked out from it all the same.
    #[test]
    fn nonsense_ranges_wrap_rather_than_panic() {
        let params = [
            ("RC3_MIN", 1_000.0),
            ("RC3_MAX", 3.0e9),
            ("RC3_TRIM", 1_500.0),
            ("RC1_MIN", -2.0e9),
            ("RC1_MAX", 2.0e9),
            ("RC1_TRIM", 0.0),
        ];
        let ranges = RcRanges::from_lookup(true, |name| {
            params.iter().find(|(n, _)| *n == name).map(|(_, v)| *v)
        });
        assert_eq!(ranges.of(3).1, i32::MIN, "(int)(float) of 3e9");
        let mut mapping = Mapping {
            ranges,
            ..Mapping::default()
        };
        mapping.config.set_axis(1, JoystickAxis::X);
        mapping.config.set_axis(3, JoystickAxis::Y);
        for step in [-10, 0, 10] {
            let reading = stick(0, step * 3_276);
            let _ = mapping.overrides(&reading, Runtime::default()).channels();
        }
    }

    /// A press fires on the way down, a let-go only for `Do_Set_Relay`; the button axes move
    /// their axis rather than firing; two functions on one button share its memory.
    #[test]
    fn buttons_fire_on_their_edges() {
        let mut config = JoystickConfig::new();
        let with = |number, function, p1, p2| JoyButton {
            buttonno: number,
            function,
            p1,
            p2,
            ..JoyButton::unassigned()
        };
        config.set_button(0, with(2, ButtonFunction::Arm, 0.0, 0.0));
        config.set_button(1, with(3, ButtonFunction::DoSetRelay, 1.0, 0.0));
        config.set_button(2, with(4, ButtonFunction::ButtonAxis0, 1100.0, 1900.0));
        config.set_button(3, with(2, ButtonFunction::Disarm, 0.0, 0.0));
        let mut tracker = ButtonTracker::new();
        let mut runtime = Runtime::default();
        let mut buttons = vec![false; 6];
        assert!(tracker.process(&config, &buttons, &mut runtime).is_empty());

        buttons[2] = true;
        buttons[3] = true;
        buttons[4] = true;
        let events = tracker.process(&config, &buttons, &mut runtime);
        let fired: Vec<(usize, bool)> = events.iter().map(|e| (e.slot, e.down)).collect();
        assert_eq!(
            fired,
            [(0, true), (1, true)],
            "Disarm shares button 2 and sees nothing"
        );
        assert_eq!(runtime.custom0, 1900);
        assert!(
            tracker.process(&config, &buttons, &mut runtime).is_empty(),
            "held"
        );

        buttons.fill(false);
        let events = tracker.process(&config, &buttons, &mut runtime);
        let fired: Vec<(usize, bool)> = events.iter().map(|e| (e.slot, e.down)).collect();
        assert_eq!(fired, [(1, false)], "only the relay hears the let-go");
        assert_eq!(runtime.custom0, 1100);
        assert_eq!(events[0].button.function, ButtonFunction::DoSetRelay);
    }
}
