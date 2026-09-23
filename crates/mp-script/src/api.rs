//! The `Script` object a script is handed.
//!
//! `// C#: Script.cs:125-215`
//!
//! Three of these behave in ways worth knowing before relying on them, and all three are
//! transliterated rather than corrected, because the scripts in the corpus were written against
//! them. Each is flagged on the method.

/// The comparison operators `Script.Conditional` offers.
///
/// Declared by the C# and, as far as the shipped corpus goes, never used by anything. Carried so
/// that a script referring to it finds it rather than raising.
/// `// C#: Script.cs:125-134`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Conditional {
    /// No comparison.
    None = 0,
    /// Less than.
    LessThan = 1,
    /// Less than or equal.
    LessOrEqual = 2,
    /// Equal.
    Equal = 3,
    /// Greater than.
    GreaterThan = 4,
    /// Greater than or equal.
    GreaterOrEqual = 5,
    /// Not equal.
    NotEqual = 6,
}

/// What a script host has to be able to do.
///
/// A trait so the corpus can be exercised against a simulated vehicle - which is what D16's
/// `tests/stock_scripts.rs` requires - without a link, a port or an aircraft.
pub trait ScriptHost {
    /// Reads a parameter the ground station already holds.
    ///
    /// **Returns 0.0 for a parameter that is not there.** Not an error, not a `None` - zero. A
    /// script testing `GetParam("FENCE_ENABLE") == 0` cannot tell a disabled fence from a
    /// parameter set that has not finished downloading. Reproduced because the corpus was written
    /// against it; `get_parameter` is the honest form for new code.
    /// `// C#: Script.cs:141-147`
    fn get_param(&self, name: &str) -> f32 {
        self.get_parameter(name).unwrap_or(0.0)
    }

    /// Reads a parameter, distinguishing absent from zero.
    fn get_parameter(&self, name: &str) -> Option<f32>;

    /// Writes a parameter, returning whether the vehicle confirmed it.
    /// `// C#: Script.cs:136-139`
    fn change_param(&mut self, name: &str, value: f32) -> bool;

    /// Changes flight mode.
    ///
    /// **Always returns true.** The C# calls `setMode` and returns a literal `true` without
    /// looking at whether the vehicle accepted, so a script that checks the result learns nothing.
    /// `// C#: Script.cs:149-153`
    fn change_mode(&mut self, mode: &str) -> bool;

    /// Waits for a status message containing `text`, up to `timeout_ms`.
    ///
    /// A **substring** match over every message received so far, not a match on new ones - so a
    /// message that arrived before the call returns immediately, and `WaitFor("Disarm")` is
    /// satisfied by "Disarming motors" from ten minutes ago. `// C#: Script.cs:155-167`
    fn wait_for(&mut self, text: &str, timeout_ms: u32) -> bool;

    /// Sets one RC override channel, and optionally sends immediately.
    ///
    /// Channels 1 to 8 only; the C# `switch` has no other arms, so channel 9 silently does
    /// nothing. When `send_now`, the packet goes out **twice** with 20 ms between - RC overrides
    /// are unacknowledged and this is the C#'s answer to losing one.
    /// `// C#: Script.cs:169-215`
    fn send_rc(&mut self, channel: u8, pwm: u16, send_now: bool) -> bool;

    /// Sleeps. `// C#: Script.cs:102-105`
    fn sleep(&mut self, milliseconds: u32);
}

/// The highest RC channel `Script.SendRC` can address.
///
/// Eight, because the C# `switch` stops there. A script asking for channel 9 gets no error and no
/// effect.
pub const MAX_SCRIPT_RC_CHANNEL: u8 = 8;

/// How long the C# waits between the two copies of an RC override.
pub const RC_RESEND_GAP_MS: u32 = 20;

/// How often `WaitFor` re-checks. `// C#: Script.cs:161`
pub const WAIT_FOR_POLL_MS: u32 = 5;

/// The RC override state a script builds up across calls.
///
/// Held between `SendRC` calls, exactly as the C# holds one `mavlink_rc_channels_override_t` for
/// the lifetime of the `Script` object: setting channel 3 and then channel 1 sends both, because
/// the previous value is still in the struct.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ScriptApi {
    channels: [u16; MAX_SCRIPT_RC_CHANNEL as usize],
}

impl ScriptApi {
    /// No channels set.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            channels: [0; MAX_SCRIPT_RC_CHANNEL as usize],
        }
    }

    /// Records a channel, returning whether the channel exists.
    ///
    /// `false` for a channel outside 1..=8, which the C# swallows. Returned here so a host can
    /// tell a script author, rather than leaving them to work out why nothing moved.
    pub fn set_channel(&mut self, channel: u8, pwm: u16) -> bool {
        let Some(index) = channel.checked_sub(1).map(usize::from) else {
            return false;
        };
        match self.channels.get_mut(index) {
            Some(slot) => {
                *slot = pwm;
                true
            }
            None => false,
        }
    }

    /// The eighteen-channel array to send, with everything the script has not set left at zero.
    ///
    /// Zero is "release this channel" on channels 1-8 and "ignore" on 9-18, which is what the C#
    /// transmits: its struct is zero-initialised and only the channels a script touched are
    /// filled in. So a script that sets only throttle releases the other three sticks, and that is
    /// the existing behaviour rather than a choice made here.
    #[must_use]
    pub fn overrides(&self) -> [u16; 18] {
        let mut out = [0u16; 18];
        for (index, value) in self.channels.iter().enumerate() {
            if let Some(slot) = out.get_mut(index) {
                *slot = *value;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The channel range is the C#'s, and going outside it is not an error there.
    #[test]
    fn only_the_first_eight_channels_exist() {
        let mut api = ScriptApi::new();
        for channel in 1..=8 {
            assert!(api.set_channel(channel, 1500), "channel {channel} exists");
        }
        for channel in [0, 9, 18, 255] {
            assert!(
                !api.set_channel(channel, 1500),
                "channel {channel} does not"
            );
        }
    }

    /// State carries between calls, which is why setting one channel sends all of them.
    #[test]
    fn channels_persist_across_calls_as_the_c_sharp_struct_does() {
        let mut api = ScriptApi::new();
        api.set_channel(3, 1100);
        api.set_channel(1, 1600);
        let out = api.overrides();
        assert_eq!(out[0], 1600);
        assert_eq!(out[2], 1100);
        // Untouched channels stay zero, which on channels 1-8 means "release".
        assert_eq!(out[1], 0);
        assert_eq!(out[3], 0);
    }

    /// Eighteen channels out, whatever the script set.
    #[test]
    fn the_override_is_the_full_message_width() {
        assert_eq!(ScriptApi::new().overrides().len(), 18);
    }

    /// The constants are the C#'s, and each is load-bearing for a script that works today.
    #[test]
    fn the_timings_are_the_ones_the_scripts_were_written_against() {
        assert_eq!(MAX_SCRIPT_RC_CHANNEL, 8);
        assert_eq!(RC_RESEND_GAP_MS, 20);
        assert_eq!(WAIT_FOR_POLL_MS, 5);
    }
}
