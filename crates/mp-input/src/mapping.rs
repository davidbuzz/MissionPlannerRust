//! Turning stick positions into RC channel values.

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
/// real RC range and is only ever reached by a binding configured with a nonsensical `min`.
pub const SAFE_MIN_US: u16 = 1;
/// The highest value a mapped channel may carry.
///
/// `UINT16_MAX - 2`, because `UINT16_MAX` means "ignore this field" and `UINT16_MAX - 1` means
/// "release" on the high channels.
pub const SAFE_MAX_US: u16 = u16::MAX - 2;

/// The centre of an RC channel's normal range, in microseconds.
pub const CENTRE_US: u16 = 1500;
/// The bottom of an RC channel's normal range, in microseconds.
pub const MIN_US: u16 = 1000;
/// The top of an RC channel's normal range, in microseconds.
pub const MAX_US: u16 = 2000;

/// Eighteen channel values, ready to send.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channels(pub [u16; CHANNELS]);

impl Channels {
    /// Every channel ignored, which is what an unmapped channel should be.
    ///
    /// Ignored rather than centred. A ground station that sends 1500 on a channel nobody mapped is
    /// a ground station that centres a flight mode switch the moment overrides are enabled.
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
    /// What a failsafe sends. Both conventions, each on its own half.
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
    /// Out of range does nothing. A mapping naming channel 19 is a misconfiguration, and refusing
    /// to fly over it would be worse than ignoring it.
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

/// Where a channel's value comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// An analogue axis, by index.
    Axis(usize),
    /// A button, by index: released is the bottom of the range, pressed the top.
    Button(usize),
}

/// A named axis, for describing a device to a person.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    /// Roll, usually the right stick left and right.
    Roll,
    /// Pitch, usually the right stick forward and back.
    Pitch,
    /// Throttle, usually the left stick forward and back.
    Throttle,
    /// Yaw, usually the left stick left and right.
    Yaw,
}

impl Axis {
    /// The channel ArduPilot expects this on by default.
    ///
    /// `RCMAP_*` can move them, and a vehicle that has been remapped needs its mapping changed
    /// here too. This is the default, not an assumption baked in.
    #[must_use]
    pub const fn default_channel(self) -> usize {
        match self {
            Self::Roll => 1,
            Self::Pitch => 2,
            Self::Throttle => 3,
            Self::Yaw => 4,
        }
    }

    /// What to call it on screen.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Roll => "roll",
            Self::Pitch => "pitch",
            Self::Throttle => "throttle",
            Self::Yaw => "yaw",
        }
    }
}

/// One channel driven by one control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Binding {
    /// The channel to drive, numbered from 1.
    pub channel: usize,
    /// What drives it.
    pub source: Source,
    /// Whether to invert it.
    ///
    /// Needed rather than optional: a joystick's pitch axis reads positive when pushed forward on
    /// some devices and negative on others, and an inverted pitch axis on a copter is a crash.
    pub reversed: bool,
    /// Microseconds at the bottom of the axis.
    pub min: u16,
    /// Microseconds at the top.
    pub max: u16,
    /// Exponential softening around centre, 0.0 for none, 1.0 for as much as is useful.
    ///
    /// Not a nicety. A gamepad stick is around 20 mm of travel where a transmitter gives 60, so a
    /// linear mapping makes the middle of the range - where all the flying happens - three times
    /// as sensitive as a pilot expects.
    pub expo: f32,
}

impl Binding {
    /// A binding with the usual range and no expo.
    #[must_use]
    pub const fn new(channel: usize, source: Source) -> Self {
        Self {
            channel,
            source,
            reversed: false,
            min: MIN_US,
            max: MAX_US,
            expo: 0.0,
        }
    }

    /// The microsecond value this binding produces for a reading.
    #[must_use]
    pub fn value(&self, reading: &crate::Reading) -> u16 {
        let raw = match self.source {
            Source::Axis(index) => reading.axis(index),
            // A button is a two-position switch: released is the bottom of the range, pressed the
            // top. Flight mode switches are done this way on a gamepad.
            Source::Button(index) => {
                if reading.button(index) {
                    1.0
                } else {
                    -1.0
                }
            }
        };
        let raw = if self.reversed { -raw } else { raw };
        let shaped = apply_expo(raw.clamp(-1.0, 1.0), self.expo);
        // Mapped from -1..1 onto min..max. Done in f32 and rounded rather than integer arithmetic,
        // because a range of 1000 microseconds over 65536 axis steps is not an integer ratio and
        // truncating it puts a centred stick at 1499.
        let span = f32::from(self.max) - f32::from(self.min);
        let value = f32::from(self.min) + (shaped + 1.0) * 0.5 * span;
        // Clamped into the band of values that mean "a stick position", not the full u16 range.
        // 0, 65534 and 65535 are not positions in this message - they are "release this channel",
        // "release" again on the high half, and "ignore this field". A misconfigured binding with
        // min = 0 would emit 0 for a stick at the bottom of its travel, and the vehicle would read
        // that as "the ground station has given this channel back", which is the failsafe firing
        // because somebody typed a zero.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            value
                .round()
                .clamp(f32::from(SAFE_MIN_US), f32::from(SAFE_MAX_US)) as u16
        }
    }
}

/// Softens the middle of an axis.
///
/// The cubic blend every transmitter uses: `(1-e)x + e·x³`. At `e = 0` it is linear; at `e = 1` it
/// is pure cube. The ends stay at the ends, which is the property that matters - expo must never
/// cost a pilot full travel.
#[must_use]
pub fn apply_expo(value: f32, expo: f32) -> f32 {
    let expo = expo.clamp(0.0, 1.0);
    (1.0 - expo).mul_add(value, expo * value * value * value)
}

/// Every binding for a device.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Mapping {
    /// The bindings, in no particular order.
    pub bindings: Vec<Binding>,
}

impl Mapping {
    /// The mapping a two-stick gamepad gets before anybody configures one.
    ///
    /// Axes **0, 1, 3, 4** — not 0 to 3. On Linux's `js` API an Xbox-layout pad reports
    /// `0 = left X`, `1 = left Y`, `2 = left trigger`, `3 = right X`, `4 = right Y`,
    /// `5 = right trigger`, and the same is true of a DualShock under `hid-sony` and of anything
    /// else presenting the XInput layout — which is almost everything sold. An earlier version of
    /// this used 0,1,2,3 and therefore bound **roll to the left trigger**, an axis that rests at
    /// one end of its travel rather than centred: enabling overrides would have commanded full
    /// roll before the pilot touched anything.
    ///
    /// Pitch and throttle are reversed because both sticks report negative when pushed forward and
    /// both mean "more" when pushed forward.
    ///
    /// A default, not a detection. A device that lays its axes out differently needs configuring,
    /// and the screen shows live axis values so a person can push a stick and see which number
    /// moves — which is the only reliable way, because the `js` API carries no axis names.
    #[must_use]
    pub fn gamepad() -> Self {
        let mut yaw = Binding::new(Axis::Yaw.default_channel(), Source::Axis(0));
        yaw.expo = 0.3;
        let mut throttle = Binding::new(Axis::Throttle.default_channel(), Source::Axis(1));
        throttle.reversed = true;
        let mut roll = Binding::new(Axis::Roll.default_channel(), Source::Axis(3));
        roll.expo = 0.3;
        let mut pitch = Binding::new(Axis::Pitch.default_channel(), Source::Axis(4));
        pitch.reversed = true;
        pitch.expo = 0.3;
        Self {
            bindings: vec![roll, pitch, throttle, yaw],
        }
    }

    /// The channels this mapping produces for a reading.
    ///
    /// Channels with no binding are left as "ignore", so enabling overrides cannot disturb a
    /// switch nobody mapped.
    #[must_use]
    pub fn channels(&self, reading: &crate::Reading) -> Channels {
        let mut channels = Channels::ignored();
        for binding in &self.bindings {
            channels.set(binding.channel, binding.value(reading));
        }
        channels
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Reading;

    fn reading(axes: &[f32]) -> Reading {
        Reading {
            axes: axes.to_vec(),
            buttons: Vec::new(),
        }
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

    /// And so does "ignore", the other way round.
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

    /// A centred stick is 1500, not 1499. Integer arithmetic on a non-integer ratio gets this
    /// wrong, and a throttle that sits one microsecond low is a vehicle that creeps down.
    #[test]
    fn a_centred_axis_is_exactly_centre() {
        let binding = Binding::new(1, Source::Axis(0));
        assert_eq!(binding.value(&reading(&[0.0])), CENTRE_US);
    }

    /// The ends are the ends.
    #[test]
    fn the_extremes_reach_the_configured_limits() {
        let binding = Binding::new(1, Source::Axis(0));
        assert_eq!(binding.value(&reading(&[-1.0])), MIN_US);
        assert_eq!(binding.value(&reading(&[1.0])), MAX_US);
        // And beyond them, because a device can report slightly over full scale.
        assert_eq!(binding.value(&reading(&[-1.5])), MIN_US);
        assert_eq!(binding.value(&reading(&[1.5])), MAX_US);
    }

    /// Reversal is needed, not optional: an inverted pitch axis on a copter is a crash.
    #[test]
    fn reversing_an_axis_swaps_its_ends() {
        let mut binding = Binding::new(2, Source::Axis(0));
        binding.reversed = true;
        assert_eq!(binding.value(&reading(&[-1.0])), MAX_US);
        assert_eq!(binding.value(&reading(&[1.0])), MIN_US);
        assert_eq!(binding.value(&reading(&[0.0])), CENTRE_US);
    }

    /// Expo softens the middle and must never cost full travel.
    #[test]
    fn expo_softens_the_middle_without_shortening_the_ends() {
        assert!((apply_expo(1.0, 1.0) - 1.0).abs() < f32::EPSILON);
        assert!((apply_expo(-1.0, 1.0) + 1.0).abs() < f32::EPSILON);
        assert!(apply_expo(0.0, 1.0).abs() < f32::EPSILON);
        // Half stick with full expo moves an eighth, not a half.
        assert!((apply_expo(0.5, 1.0) - 0.125).abs() < 1e-6);
        // Linear when off.
        assert!((apply_expo(0.5, 0.0) - 0.5).abs() < f32::EPSILON);
        // And the shaped value is always inside the raw one, never outside.
        for step in 0..=100 {
            #[allow(clippy::cast_precision_loss)]
            let x = step as f32 / 100.0;
            assert!(apply_expo(x, 0.5) <= x + 1e-6, "expo overshot at {x}");
        }
    }

    /// A channel nobody mapped must not be disturbed by enabling overrides.
    #[test]
    fn unmapped_channels_are_ignored_rather_than_centred() {
        let mapping = Mapping::gamepad();
        let channels = mapping.channels(&reading(&[0.0, 0.0, 0.0, 0.0, 0.0]));
        // The four sticks are driven.
        for channel in 1..=4 {
            assert_eq!(channels.get(channel), Some(CENTRE_US), "channel {channel}");
        }
        // Channel 5 is usually the flight mode switch. Centring it would change the mode.
        assert_eq!(channels.get(5), Some(u16::MAX));
        assert_eq!(channels.get(9), Some(0));
    }

    /// Both sticks mean "more" when pushed forward, and both report negative doing it.
    #[test]
    fn the_default_gamepad_mapping_has_forward_meaning_more() {
        let mapping = Mapping::gamepad();
        // Axis 1 is the left stick's vertical, mapped to throttle: forward is -1.0.
        let forward = mapping.channels(&reading(&[0.0, -1.0, 0.0, 0.0, 0.0]));
        assert_eq!(forward.get(Axis::Throttle.default_channel()), Some(MAX_US));
        let back = mapping.channels(&reading(&[0.0, 1.0, 0.0, 0.0, 0.0]));
        assert_eq!(back.get(Axis::Throttle.default_channel()), Some(MIN_US));
        // Axis 4 is the right stick's vertical, mapped to pitch: forward is nose down, which is
        // the high end of the channel on ArduPilot.
        let nose_down = mapping.channels(&reading(&[0.0, 0.0, 0.0, 0.0, -1.0]));
        assert_eq!(nose_down.get(Axis::Pitch.default_channel()), Some(MAX_US));
    }

    /// The triggers must not be bound to anything, because they do not rest centred.
    ///
    /// An Xbox-layout pad reports the triggers on axes 2 and 5, resting at one end of their travel.
    /// The first version of this mapping used axes 0-3 and bound roll to axis 2, so enabling
    /// overrides commanded full roll before the pilot touched a thing.
    #[test]
    fn no_default_binding_lands_on_a_trigger_axis() {
        const TRIGGERS: [usize; 2] = [2, 5];
        for binding in &Mapping::gamepad().bindings {
            if let Source::Axis(index) = binding.source {
                assert!(
                    !TRIGGERS.contains(&index),
                    "channel {} is bound to axis {index}, which is a trigger on an XInput pad",
                    binding.channel
                );
            }
        }
    }

    /// A trigger at rest must not move a stick channel, whichever end it rests at.
    #[test]
    fn a_resting_trigger_does_not_disturb_the_sticks() {
        let mapping = Mapping::gamepad();
        // Sticks centred, both triggers hard at -1.0, which is where they sit unpressed.
        let resting = mapping.channels(&reading(&[0.0, 0.0, -1.0, 0.0, 0.0, -1.0]));
        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            assert_eq!(
                resting.get(axis.default_channel()),
                Some(CENTRE_US),
                "{} moved with the sticks centred",
                axis.label()
            );
        }
    }

    /// A button is a two-position switch.
    #[test]
    fn a_button_drives_a_channel_to_one_end_or_the_other() {
        let binding = Binding::new(7, Source::Button(2));
        let pressed = Reading {
            buttons: vec![false, false, true],
            ..Reading::default()
        };
        assert_eq!(binding.value(&pressed), MAX_US);
        let released = Reading {
            buttons: vec![false, false, false],
            ..Reading::default()
        };
        assert_eq!(binding.value(&released), MIN_US);
    }

    /// A stick position must never collide with the protocol's control values.
    ///
    /// 0 means "release" on channels 1-8 and 65535 means "ignore"; a binding configured with
    /// `min = 0` would otherwise turn a stick at the bottom of its travel into a failsafe.
    #[test]
    fn a_stick_position_can_never_be_read_as_release_or_ignore() {
        let mut binding = Binding::new(1, Source::Axis(0));
        binding.min = 0;
        binding.max = u16::MAX;
        for step in -20..=20 {
            #[allow(clippy::cast_precision_loss)]
            let value = binding.value(&reading(&[step as f32 / 10.0]));
            assert_ne!(value, 0, "0 is 'release' on channels 1-8");
            assert_ne!(value, u16::MAX, "65535 is 'ignore'");
            assert_ne!(value, u16::MAX - 1, "65534 is 'release' on channels 9-18");
        }
    }

    /// A mapping naming a channel that does not exist must not panic or write out of bounds.
    #[test]
    fn a_channel_outside_the_message_is_ignored() {
        let mut channels = Channels::ignored();
        channels.set(0, 1500);
        channels.set(19, 1500);
        channels.set(usize::MAX, 1500);
        assert_eq!(channels, Channels::ignored());
        assert_eq!(channels.get(0), None);
        assert_eq!(channels.get(19), None);
    }
}
