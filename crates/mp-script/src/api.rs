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

    /// Whether any status message received so far contains `text`: `cs.messages.Any(a =>
    /// a.message.Contains(message))`, the test `WaitFor` polls.
    /// `// C#: Script.cs:158`
    fn has_message(&self, text: &str) -> bool;
    /// `cs.messages.Clear()`: the list of status messages emptied, which the corpus does before a
    /// `WaitFor` so an old line cannot satisfy it.
    fn clear_messages(&mut self);
    /// `cs.<field>`: a `CurrentState` member by its C# name - `lat`, `alt`, `mode`, `satcount` -
    /// or `None` for a name the state does not have, which a script sees as an `AttributeError`.
    fn cs_field(&self, name: &str) -> Option<CsValue>;
    /// Waits for a status message containing `text`, up to `timeout_ms`.
    ///
    /// A **substring** match over every message received so far, not a match on new ones - so a
    /// message that arrived before the call returns immediately, and `WaitFor("Disarm")` is
    /// satisfied by "Disarming motors" from ten minutes ago. Polled every 5 ms, as the C# polls;
    /// the engine's own loop adds the abort check. `// C#: Script.cs:155-167`
    fn wait_for(&mut self, text: &str, timeout_ms: u32) -> bool {
        let mut waited = 0;
        while !self.has_message(text) {
            self.sleep(WAIT_FOR_POLL_MS);
            waited += WAIT_FOR_POLL_MS;
            if waited > timeout_ms {
                return false;
            }
        }
        true
    }

    /// Sets one RC override channel, and optionally sends immediately.
    ///
    /// Channels 1 to 8 only; the C# `switch` has no other arms, so channel 9 silently does
    /// nothing. When `send_now`, the packet goes out **twice** with 20 ms between - RC overrides
    /// are unacknowledged and this is the C#'s answer to losing one.
    /// `// C#: Script.cs:169-215`
    fn send_rc(&mut self, channel: u8, pwm: u16, send_now: bool) -> bool;

    /// Sleeps. `// C#: Script.cs:102-105`
    fn sleep(&mut self, milliseconds: u32);

    /// `MAVlist[sysid, compid].cs.<field>`: a named vehicle's `CurrentState`, as
    /// `setGuidedModeWP(sysid, compid, ...)` reads its `mode` and `firmware`. This default is
    /// [`ScriptHost::cs_field`], for a host with one vehicle.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4423, 4430`
    fn cs_field_of(&self, _target: (u32, u8), name: &str) -> Option<CsValue> {
        self.cs_field(name)
    }

    /// `setMode(sysid, compid, mode)`: the named vehicle put in a mode, not the one flown. This
    /// default is [`ScriptHost::change_mode`], for a host with one vehicle.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4614-4642`
    fn set_mode_of(&mut self, _target: (u32, u8), mode: &str) {
        let _ = self.change_mode(mode);
    }

    // `MAV`, `MainV2.comPort`: the `MAVLinkInterface` members the shipped scripts reach
    // (`crates/mp-script/src/clr/shim.py` holds the rest of each member - its checks, its
    // overloads, its conversions - and calls these for what goes over the link). Each default is
    // what the C# does with no port open (`BaseStream == null || !BaseStream.IsOpen`), so a host
    // without a link needs to say nothing more.

    /// `MAV.sysid`, `MAV.compid` - here also `sysidcurrent`, `compidcurrent` - the vehicle the
    /// link is talking to, (0, 0) before one is heard.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:289-315`
    fn link_target(&self) -> (u32, u8) {
        (0, 0)
    }

    /// `MAV.BaseStream.IsOpen`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:32-66`
    fn is_open(&self) -> bool {
        false
    }

    /// `MAV.setParam(name, value, force)`: false without sending for a parameter the vehicle
    /// has not listed, true without sending for a value it already holds unless `force`, else
    /// `PARAM_SET` until the vehicle echoes it, three more times 700 ms apart, and then the
    /// `TimeoutException`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1619-1767`
    fn set_param(
        &mut self,
        _target: (u32, u8),
        _name: &str,
        _value: f64,
        _force: bool,
    ) -> Result<bool, Timeout> {
        Ok(false)
    }

    /// `MAV.doCommand(...)`: `COMMAND_LONG` and, when `require_ack`, its `COMMAND_ACK` waited
    /// for - accepted true, any other result false, nothing heard the `TimeoutException`; a
    /// reboot and the rest the C# does not wait for are true at once. False with no port.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2671-2833`
    fn command(
        &mut self,
        _target: (u32, u8),
        _command: u16,
        _params: [f32; 7],
        _require_ack: bool,
    ) -> Result<bool, Timeout> {
        Ok(false)
    }

    /// `MAV.doReboot(bootloadermode, true)`: `PREFLIGHT_REBOOT_SHUTDOWN` with param1 1 (3 into
    /// the bootloader) through `doCommand`, which does not wait for it, and param1 1 again if
    /// that was refused. A host on a serial port also reopens the port afterwards if the reboot
    /// took it away, as the C# does (`:2573-2583`); this default has no port to reopen.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2550-2586`
    fn reboot(&mut self, bootloader: bool) -> Result<bool, Timeout> {
        let target = self.link_target();
        let param1 = if bootloader { 3.0 } else { 1.0 };
        let first = self.command(
            target,
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [param1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            true,
        )?;
        if first {
            return Ok(true);
        }
        self.command(
            target,
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            true,
        )
    }

    /// `MAV.setWPTotal(total, type)`: `MISSION_COUNT`, and the vehicle's first request for an
    /// item (0 or 1) waited for, three more times 700 ms apart, then the `TimeoutException`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3746-3870`
    fn set_wp_total(
        &mut self,
        _target: (u32, u8),
        _total: u16,
        _mission_type: u8,
    ) -> Result<(), Timeout> {
        Err(Timeout::on("setWPTotal"))
    }

    /// `MAV.setWP(loc, index, frame, current, autocontinue)` with `use_int` false: the
    /// `MISSION_ITEM` until the vehicle acknowledges it - its `MAV_MISSION_RESULT` - or asks for
    /// the item after it - `MAV_MISSION_ACCEPTED` - ten more times 450 ms apart, then the
    /// `TimeoutException`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3965-4224`
    fn set_wp(&mut self, _target: (u32, u8), _item: &WpItem) -> Result<u8, Timeout> {
        Err(Timeout::on("setWP"))
    }

    /// `MAV.setWPACK(type)`: a `MISSION_ACK` of `MAV_MISSION_ACCEPTED`, not waited on.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2431-2447`
    fn set_wp_ack(&mut self, _target: (u32, u8), _mission_type: u8) {}

    /// `MAV.setWPCurrent(sysid, compid, index)`: `MISSION_SET_CURRENT` until a
    /// `MISSION_CURRENT` arrives, five more times 2 s apart, then the `TimeoutException`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2452-2501`
    fn set_wp_current(&mut self, _target: (u32, u8), _seq: u16) -> Result<bool, Timeout> {
        Err(Timeout::on("setWPCurrent"))
    }

    /// `MAV.getWP(index, type)`: the item as the vehicle holds it, or the `TimeoutException`
    /// after five more asks 2.5 s apart.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3393-3557`
    fn get_wp(
        &mut self,
        _target: (u32, u8),
        _index: u16,
        _mission_type: u8,
    ) -> Result<Locationwp, Timeout> {
        Err(Timeout::on("getWP"))
    }

    /// `setPositionTargetGlobalInt` as `setGuidedModeWP` calls it for everything but ArduPlane:
    /// a `SET_POSITION_TARGET_GLOBAL_INT` with only the position enabled, sent and not waited
    /// on. Whether it was queued.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4439-4443, 4500-4555`
    fn set_position_target(&mut self, _target: (u32, u8), _position: &PositionTarget) -> bool {
        false
    }

    /// `MAV.BaseStream.Write(buffer, offset, count)`: the bytes put on the port as they are,
    /// not framed. Whether they went; false with no port, where the C#'s port throws.
    /// `// C#: ExtLibs/Interfaces/ICommsSerial.cs:68`
    fn write_raw(&mut self, _bytes: &[u8]) -> bool {
        false
    }

    /// `MainV2.speechEngine.SpeakAsync(text)` once `SpeakAsync`'s own checks have passed: the
    /// text to be spoken. Speech itself is DELIVERABLES Deliverable 15; a host records what it was asked
    /// to say.
    /// `// C#: Utilities/Speech.cs:65-80`
    fn speak(&mut self, _text: &str) {}

    /// `MainV2.speechEnable` and `MainV2.speech_armed_only` as a script starts: the user's
    /// "speechenable" and "speech_armed_only" settings, which `MainV2` reads them from, off
    /// when absent - as they are for this default, a host with no settings.
    /// `// C#: MainV2.cs:458-468, 658, 1007-1008; Utilities/Speech.cs:15`
    fn speech_settings(&self) -> (bool, bool) {
        (false, false)
    }
}

/// `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`.
pub const MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: u16 = 246;

/// The C#'s `TimeoutException` from a `MAVLinkInterface` member that heard nothing back, with
/// its message: "Timeout on read - setWP". A script sees it as `System.TimeoutException`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Timeout(pub String);

impl Timeout {
    /// "Timeout on read - `member`", the message every one of these members throws.
    #[must_use]
    pub fn on(member: &str) -> Self {
        Self(format!("Timeout on read - {member}"))
    }
}

/// `mavlink_mission_item_t` as `setWPAsync` fills it from a `Locationwp` with `use_int`
/// false: the position as floats, the rest as the script gave it.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4020-4039`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WpItem {
    /// `seq`, the index the script asked for.
    pub seq: u16,
    /// `frame`: the `MAV_FRAME` argument, not the `Locationwp`'s own.
    pub frame: u8,
    /// `command`: `loc.id`.
    pub command: u16,
    /// `current`: 0, or 2 for a guided target, 3 for an altitude change.
    pub current: u8,
    /// `autocontinue`.
    pub autocontinue: u8,
    /// `param1` to `param4`: `loc.p1` to `loc.p4`.
    pub params: [f32; 4],
    /// `x`: `(float)loc.lat`.
    pub x: f32,
    /// `y`: `(float)loc.lng`.
    pub y: f32,
    /// `z`: `(float)loc.alt`.
    pub z: f32,
    /// `mission_type`.
    pub mission_type: u8,
}

/// `MissionPlanner.Utilities.Locationwp`'s fields, as `getWP` fills them from the item the
/// vehicle sent.
/// `// C#: ExtLibs/Utilities/locationwp.cs:199-210; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:3494-3540`
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Locationwp {
    /// `id`, the `MAV_CMD`.
    pub id: u16,
    /// `p1`.
    pub p1: f32,
    /// `p2`.
    pub p2: f32,
    /// `p3`.
    pub p3: f32,
    /// `p4`.
    pub p4: f32,
    /// `lat`, degrees.
    pub lat: f64,
    /// `lng`, degrees.
    pub lng: f64,
    /// `alt`, metres in the item's frame.
    pub alt: f32,
    /// `frame`, the `MAV_FRAME`.
    pub frame: u8,
}

/// Where `setGuidedModeWP` sends a vehicle that is not a plane: the `Locationwp`'s own frame,
/// latitude, longitude and altitude.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4439-4443`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionTarget {
    /// `(MAV_FRAME)gotohere.frame`.
    pub frame: u8,
    /// Degrees.
    pub lat: f64,
    /// Degrees.
    pub lng: f64,
    /// Metres.
    pub alt: f64,
}

/// A `CurrentState` member's value as a script sees it: the C#'s doubles and floats as a number,
/// its strings (`mode`, `firmware`) as text, its booleans (`armed`) as a flag.
#[derive(Debug, Clone, PartialEq)]
pub enum CsValue {
    /// A numeric field.
    Number(f64),
    /// A string field.
    Text(String),
    /// A boolean field.
    Flag(bool),
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
