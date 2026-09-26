//! Joystick and gamepad input, mapped to RC channels.
//!
//! Flying from a ground station with no transmitter, which is what D15 asks for. It is also the
//! most dangerous thing in this application: a stick position sent over a telemetry link is a
//! stick position that can stop arriving, and a vehicle holding the last one it received is a
//! vehicle flying itself into the ground.
//!
//! So the failsafe is not a feature of this module, it is the shape of it. Nothing here holds a
//! value; every channel comes with a deadline, and a mapping that has not been fed produces a
//! release rather than a repeat. The interesting code is `Failsafe` and the tests around it,
//! and the mapping is the easy part: Mission Planner's own (`JoystickBase.pickchannel`), with its
//! settings kept in its own files ([`config`]).
//!
//! Getting a stick onto the wire fast is [`StickReader`]: a thread that blocks on the device and a
//! thread that sends, so that neither the latency nor the failsafe depends on how quickly a user
//! interface gets round to it. Its module documentation is the wiring guide.

pub mod config;
pub mod event;
pub mod latency;
pub mod mapping;
pub mod reader;
pub mod scripted;

#[cfg(target_os = "linux")]
pub mod linux;

pub use config::{ButtonFunction, ConfigFiles, JoyButton, JoyChannel, JoystickAxis, JoystickConfig};
pub use latency::LatencyHistogram;
pub use mapping::{
    ButtonEvent, ButtonTracker, Channels, ManualControl, Mapping, Overrides, RcRanges, Runtime,
};
pub use reader::{Cause, Frame, MIN_INTERVAL, RESEND, StickReader};

use std::time::{Duration, Instant};

/// How long a mapping may go unfed before its channels are released.
///
/// A fifth of a second. Long enough that an ordinary scheduling hiccup on the machine does not
/// hand control back mid-manoeuvre, short enough that a stuck or unplugged stick does not fly the
/// aircraft for a second and a half. Mission Planner sends overrides at around 10 Hz, so this is
/// two missed sends.
pub const STICK_TIMEOUT: Duration = Duration::from_millis(200);

/// Whether a device is still there.
///
/// Deliberately not a `bool`. The question a failsafe must ask is "is this device still there",
/// and the question a `bool` invites is "did something happen" - which on an edge-triggered API
/// are different questions with the same answer most of the time and opposite answers at the worst
/// possible moment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Poll {
    /// The device answered. The reading is current, whether or not it changed.
    Alive,
    /// The device has gone away.
    Gone,
}

/// One connected input device.
#[derive(Debug, Clone, PartialEq)]
pub struct Device {
    /// How the operating system identifies it, for reconnecting to the same one.
    pub id: String,
    /// What to show a person.
    pub name: String,
    /// How many axes it reports.
    pub axes: usize,
    /// How many buttons it reports.
    pub buttons: usize,
}

/// The live position of every axis and button on a device.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Reading {
    /// Axis positions, -1.0 to 1.0.
    pub axes: Vec<f32>,
    /// Button states, one per button.
    pub buttons: Vec<bool>,
}

impl Reading {
    /// An axis position, or centre if the device has no such axis.
    ///
    /// Centre rather than an error: a mapping that names an axis a device does not have is a
    /// misconfiguration, and a misconfigured stick should sit still rather than refuse to fly.
    #[must_use]
    pub fn axis(&self, index: usize) -> f32 {
        self.axes.get(index).copied().unwrap_or(0.0)
    }

    /// A button state, or released if the device has no such button.
    #[must_use]
    pub fn button(&self, index: usize) -> bool {
        self.buttons.get(index).copied().unwrap_or(false)
    }
}

/// Decides whether the sticks are still being believed.
///
/// Separate from the device and from the mapping, and tested on its own, because it is the part
/// that has to be right. It is fed the time an update arrived rather than reading the clock
/// itself, so the tests can describe a link that stalls without waiting for one.
#[derive(Debug, Clone, Copy)]
pub struct Failsafe {
    last_update: Option<Instant>,
    timeout: Duration,
    /// Whether the operator has switched overrides on at all.
    enabled: bool,
    /// How many more release frames to send for the current lapse.
    ///
    /// A count rather than a flag. The flag latched the moment the release was *produced*, and
    /// nothing downstream could un-latch it: `Link::send` returns a bool that was discarded, the
    /// transport is unacknowledged, and the condition that triggers a release in the first place is
    /// usually a link that is dropping frames. So the one frame that hands an aircraft back to its
    /// pilot was sent exactly once, unacknowledged, over a link known to be lossy - while every
    /// ordinary frame in this system is sent at 20 Hz precisely because loss is expected.
    releases_left: u8,
}

/// How many times a release is repeated before going quiet.
///
/// Five, which at [`RESEND`] is a quarter of a second. Enough that a 20% loss rate has a
/// one-in-3,000 chance of losing all of them, and still short enough to leave the vehicle's own
/// failsafe timer to do its job - sending releases forever would stop that timer ever expiring,
/// which is the opposite of handing control back.
pub(crate) const RELEASE_REPEATS: u8 = 5;

impl Default for Failsafe {
    fn default() -> Self {
        Self::new(STICK_TIMEOUT)
    }
}

impl Failsafe {
    /// A failsafe with a given timeout, switched off.
    #[must_use]
    pub const fn new(timeout: Duration) -> Self {
        Self {
            last_update: None,
            timeout,
            enabled: false,
            releases_left: 0,
        }
    }

    /// Turns overrides on or off.
    ///
    /// Switching off arms a release, because a vehicle must not keep flying the last stick
    /// position after the operator has said stop. That is the same requirement as a disconnect,
    /// so it takes the same path rather than a second one that could be got wrong separately.
    pub fn set_enabled(&mut self, enabled: bool) {
        if self.enabled && !enabled {
            self.releases_left = RELEASE_REPEATS;
        }
        self.enabled = enabled;
        if !enabled {
            self.last_update = None;
        }
    }

    /// Hands control back now, without waiting for the timeout.
    ///
    /// For a lapse that has been *detected* rather than inferred - an unplugged device, a link that
    /// reported itself dead. The timeout exists for lapses that cannot be detected; continuing to
    /// send a stale stick position for another 200 ms after the code already knows the device is
    /// gone is not a failsafe, it is a delay.
    pub fn release_now(&mut self) {
        self.set_enabled(false);
    }

    /// Whether overrides are switched on.
    #[must_use]
    pub const fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Records that a reading arrived.
    pub fn fed(&mut self, at: Instant) {
        if self.enabled {
            self.last_update = Some(at);
            self.releases_left = 0;
        }
    }

    /// Whether the sticks are currently trusted.
    #[must_use]
    pub fn is_live(&self, now: Instant) -> bool {
        self.enabled
            && self
                .last_update
                .is_some_and(|last| now.duration_since(last) < self.timeout)
    }

    /// What to send now, if anything.
    ///
    /// `Some(channels)` while the sticks are live. One `Some(Channels::release())` when they stop
    /// being live, and `None` after that.
    ///
    /// The single release is the point. Sending releases forever would keep a vehicle's failsafe
    /// timer from ever expiring, which is the opposite of the intent: once control has been handed
    /// back to the transmitter, the ground station should be silent and let the vehicle's own
    /// failsafe handle a transmitter that is also gone.
    pub fn outgoing(&mut self, now: Instant, channels: Channels) -> Option<Channels> {
        if self.is_live(now) {
            return Some(channels);
        }
        // A lapse that arrived by timeout rather than by `release_now` still has to be announced.
        if self.enabled && self.last_update.is_some() {
            self.enabled = false;
            self.last_update = None;
            self.releases_left = RELEASE_REPEATS;
        }
        if self.releases_left == 0 {
            return None;
        }
        self.releases_left -= 1;
        Some(Channels::release())
    }

    /// How many release frames are still to be sent.
    #[must_use]
    pub const fn releases_pending(&self) -> u8 {
        self.releases_left
    }

    /// Puts back a release frame that the link refused to send.
    ///
    /// Saturating rather than wrapping, and capped at the original budget: a link that is refusing
    /// everything must not turn the release into an unbounded retry loop that keeps the vehicle's
    /// own failsafe timer from ever expiring.
    pub fn retry_release(&mut self) {
        if self.releases_left < RELEASE_REPEATS {
            self.releases_left = self.releases_left.saturating_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(millis: u64) -> Instant {
        // A fixed origin so the arithmetic in these tests is readable. Instant has no public
        // constructor, so it is built by adding to one taken once.
        static ORIGIN: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
        *ORIGIN.get_or_init(Instant::now) + Duration::from_millis(millis)
    }

    /// Nothing is sent until the operator says so. An application that starts overriding a
    /// vehicle's transmitter because a gamepad is plugged in is an application nobody should run.
    #[test]
    fn overrides_are_off_until_switched_on() {
        let mut failsafe = Failsafe::default();
        assert!(!failsafe.is_enabled());
        failsafe.fed(at(0));
        assert!(!failsafe.is_live(at(0)));
        assert_eq!(failsafe.outgoing(at(0), Channels::centred()), None);
    }

    /// The whole reason this module exists.
    #[test]
    fn a_stick_that_stops_reporting_releases_the_channels() {
        let mut failsafe = Failsafe::default();
        failsafe.set_enabled(true);
        failsafe.fed(at(0));

        assert!(failsafe.is_live(at(100)));
        assert_eq!(
            failsafe.outgoing(at(100), Channels::centred()),
            Some(Channels::centred())
        );

        // Past the timeout: releases, repeated, then silence.
        assert!(!failsafe.is_live(at(250)));
        for tick in 0..RELEASE_REPEATS {
            let at_ms = 250 + u64::from(tick) * 50;
            assert_eq!(
                failsafe.outgoing(at(at_ms), Channels::centred()),
                Some(Channels::release()),
                "release {tick} should still be going out"
            );
        }
        assert_eq!(failsafe.outgoing(at(600), Channels::centred()), None);
        assert_eq!(failsafe.outgoing(at(5_000), Channels::centred()), None);
    }

    /// The release is repeated, because the link that caused the lapse is usually the link that
    /// will drop it. One unacknowledged frame is not a handover.
    #[test]
    fn the_release_is_repeated_rather_than_sent_once() {
        let mut failsafe = Failsafe::default();
        failsafe.set_enabled(true);
        failsafe.fed(at(0));
        let _ = failsafe.outgoing(at(250), Channels::centred());
        assert_eq!(failsafe.releases_pending(), RELEASE_REPEATS - 1);
        const _: () = assert!(
            RELEASE_REPEATS > 1,
            "one frame over a lossy link is not a handover"
        );
    }

    /// A detected loss releases now, not after the timeout that exists for undetectable ones.
    #[test]
    fn a_known_disconnect_releases_on_the_same_tick() {
        let mut failsafe = Failsafe::default();
        failsafe.set_enabled(true);
        failsafe.fed(at(0));
        // The device is unplugged at t=10, well inside the 200 ms timeout.
        failsafe.release_now();
        assert_eq!(
            failsafe.outgoing(at(10), Channels::centred()),
            Some(Channels::release()),
            "a release must not wait 200 ms after the unplug is already known"
        );
        assert!(!failsafe.is_enabled());
    }

    /// Switching off must hand control back, not merely stop sending.
    ///
    /// Stopping quietly leaves the vehicle holding the last stick position until its own failsafe
    /// notices, which on a copter is long enough to hit something.
    #[test]
    fn switching_off_releases_rather_than_going_quiet() {
        let mut failsafe = Failsafe::default();
        failsafe.set_enabled(true);
        failsafe.fed(at(0));
        failsafe.set_enabled(false);
        for tick in 0..RELEASE_REPEATS {
            assert_eq!(
                failsafe.outgoing(at(10 + u64::from(tick)), Channels::centred()),
                Some(Channels::release())
            );
        }
        assert_eq!(failsafe.outgoing(at(20), Channels::centred()), None);
    }

    /// A stick that comes back does **not** silently take control again.
    ///
    /// The opposite of what this originally did, and the change is deliberate. Once control has
    /// been handed to the transmitter the vehicle may already be in RC failsafe, flying an RTL.
    /// Grabbing it back because a stick twitched produces exactly the oscillation between
    /// ground-station control and vehicle failsafe that makes an aircraft unflyable - and it
    /// happens without the operator doing anything. Coming back is a decision, so it needs a press.
    #[test]
    fn a_reconnected_stick_does_not_take_control_back_by_itself() {
        let mut failsafe = Failsafe::default();
        failsafe.set_enabled(true);
        failsafe.fed(at(0));
        for tick in 0..RELEASE_REPEATS {
            let _ = failsafe.outgoing(at(300 + u64::from(tick) * 50), Channels::centred());
        }
        assert_eq!(failsafe.outgoing(at(600), Channels::centred()), None);
        assert!(!failsafe.is_enabled(), "a lapse switches overrides off");

        // Feeding it now must do nothing at all: it is disabled.
        failsafe.fed(at(700));
        assert!(!failsafe.is_live(at(710)));
        assert_eq!(failsafe.outgoing(at(710), Channels::centred()), None);

        // Only a deliberate re-enable brings it back.
        failsafe.set_enabled(true);
        failsafe.fed(at(800));
        assert!(failsafe.is_live(at(810)));
        assert_eq!(
            failsafe.outgoing(at(810), Channels::centred()),
            Some(Channels::centred())
        );
    }

    /// Exactly at the timeout is already too late. A boundary that is inclusive on the wrong side
    /// means one more frame of a stick that is not there.
    #[test]
    fn the_timeout_boundary_favours_releasing() {
        let mut failsafe = Failsafe::default();
        failsafe.set_enabled(true);
        failsafe.fed(at(0));
        assert!(failsafe.is_live(at(199)));
        assert!(!failsafe.is_live(at(200)));
    }

    /// Feeding a failsafe that is switched off must not arm it.
    #[test]
    fn a_disabled_failsafe_cannot_be_fed_into_life() {
        let mut failsafe = Failsafe::default();
        failsafe.fed(at(0));
        failsafe.fed(at(10));
        assert!(!failsafe.is_live(at(20)));
    }

    /// An axis or button a device does not have reads as centred, not as an error.
    #[test]
    fn a_missing_axis_reads_as_centred() {
        let reading = Reading {
            axes: vec![0.5],
            buttons: vec![true],
        };
        assert!((reading.axis(0) - 0.5).abs() < f32::EPSILON);
        assert!(reading.axis(7).abs() < f32::EPSILON);
        assert!(reading.button(0));
        assert!(!reading.button(7));
    }
}
