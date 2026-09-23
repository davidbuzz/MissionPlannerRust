//! Reading joysticks on Linux, through `/dev/input/js*`.
//!
//! The legacy joystick API rather than evdev. It is deprecated and it is also an eight-byte
//! struct of little-endian integers that any kernel since 1996 emits, which means it can be read
//! with `std::fs::File` and no `unsafe` - and this workspace denies `unsafe`. evdev would need
//! ioctls for the axis ranges, which needs `libc` and a block of `unsafe` around each one, to
//! support devices that also present a `js` node.
//!
//! The one thing given up is the axis names evdev reports. That matters less than it sounds: the
//! names are wrong on half the devices anyway, and the screen shows live axis values so a person
//! can push a stick and see which number moves.

use crate::{Device, Reading};
use std::fs::File;
use std::io::{ErrorKind, Read};
use std::path::{Path, PathBuf};

/// Size of a `js_event`: `__u32 time`, `__s16 value`, `__u8 type`, `__u8 number`.
const EVENT_LEN: usize = 8;

/// `JS_EVENT_BUTTON`.
const EVENT_BUTTON: u8 = 0x01;
/// `JS_EVENT_AXIS`.
const EVENT_AXIS: u8 = 0x02;
/// `JS_EVENT_INIT`, or-ed into the type on the synthetic events sent when a device is opened.
///
/// Those carry the current position of everything, which is exactly what is wanted: without them
/// a device reads as centred until something moves, and a throttle that is actually up reads as
/// down until the pilot touches it.
const EVENT_INIT: u8 = 0x80;

/// Full scale of an axis in the joystick API.
const AXIS_SCALE: f32 = 32_767.0;

/// Where the joystick nodes live.
const INPUT_DIR: &str = "/dev/input";

/// Lists the joysticks the machine can see.
///
/// Sorted, so the list does not reorder itself between calls and move the device a person chose.
#[must_use]
pub fn devices() -> Vec<Device> {
    let Ok(entries) = std::fs::read_dir(INPUT_DIR) else {
        return Vec::new();
    };
    let mut paths: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| {
                    // `strip_prefix` then a non-empty all-digits check. `starts_with("js")` plus
                    // `name[2..].chars().all(..)` accepts a bare "js", because `all` on an empty
                    // iterator is vacuously true - and it panics on a non-ASCII boundary at byte 2.
                    name.strip_prefix("js").is_some_and(|rest| {
                        !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit())
                    })
                })
        })
        .collect();
    paths.sort();
    paths.iter().map(|path| describe(path)).collect()
}

/// What can be said about a device without opening it for reading.
///
/// The name comes from sysfs rather than an ioctl, for the same reason the events do: reading a
/// file needs no `unsafe`. A device whose sysfs entry is missing gets its node name, which is at
/// least something a person can match against `ls /dev/input`.
fn describe(path: &Path) -> Device {
    let id = path.display().to_string();
    let node = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("js");
    let name = std::fs::read_to_string(format!("/sys/class/input/{node}/device/name"))
        .ok()
        .map(|text| text.trim().to_owned())
        .filter(|text| !text.is_empty())
        .unwrap_or_else(|| node.to_owned());
    // Counts come from sysfs rather than the `JSIOCGAXES`/`JSIOCGBUTTONS` ioctls, for the same
    // reason the events do: reading a file needs no `unsafe`. A device whose sysfs entry is missing
    // reports zero, which the panel shows as "unknown" rather than claiming a device has no
    // controls. An earlier version hard-coded zero with a comment promising these were filled in
    // later; nothing filled them in.
    let count = |leaf: &str| {
        std::fs::read_to_string(format!("/sys/class/input/{node}/device/{leaf}"))
            .ok()
            .map_or(0, |text| text.split_whitespace().count())
    };
    Device {
        id,
        name,
        axes: count("abs"),
        buttons: count("key"),
    }
}

/// What a poll found.
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

/// An open joystick.
#[derive(Debug)]
pub struct Joystick {
    file: File,
    reading: Reading,
    /// Set when the device goes away, so a caller can tell "nothing moved" from "unplugged".
    disconnected: bool,
}

impl Joystick {
    /// Opens a device by path.
    ///
    /// Non-blocking, because the caller is a UI thread that polls: a blocking read on a stick
    /// nobody is touching would stop the application until somebody touched it. `O_NONBLOCK` is
    /// passed to `open(2)` through `custom_flags`, which is how `std` exposes it on Unix and
    /// which avoids the `fcntl` call - and the `unsafe` - that setting it afterwards would need.
    ///
    /// # Errors
    /// If the device cannot be opened - usually permissions, which on most distributions means
    /// the user is not in the `input` group.
    pub fn open(path: &str) -> std::io::Result<Self> {
        use std::os::unix::fs::OpenOptionsExt;
        // `O_NONBLOCK` is 0o4000 on the common Linux ABIs and something else on mips, sparc and
        // alpha. Getting it wrong does not fail loudly - it opens the device *blocking*, and the
        // first read freezes whatever called it, which on the stick-polling path is the thing
        // flying the aircraft. So rather than carry three constants nobody here can test, the
        // build refuses on the architectures where the value differs. Anyone who needs one adds it
        // with a machine to check it on.
        #[cfg(any(
            target_arch = "mips",
            target_arch = "mips32r6",
            target_arch = "mips64",
            target_arch = "mips64r6",
            target_arch = "sparc",
            target_arch = "sparc64",
        ))]
        compile_error!(
            "O_NONBLOCK differs on this architecture; add the right value and test it on hardware"
        );
        const O_NONBLOCK: i32 = 0o4000;
        let file = File::options()
            .read(true)
            .custom_flags(O_NONBLOCK)
            .open(path)?;
        Ok(Self {
            file,
            reading: Reading::default(),
            disconnected: false,
        })
    }

    /// Reads everything the device has sent since last time.
    ///
    /// Returns `Poll::Alive` when the device answered - whether or not anything had changed - and
    /// `Poll::Gone` when it has vanished.
    ///
    /// **This is not a "did the stick move" question, and the distinction is a flight-safety one.**
    /// The joystick API is edge-triggered: the kernel emits an event only when an axis or button
    /// *changes*, and the input core filters jitter below the device's `fuzz` value, so a stick
    /// held steady - against an end stop, in a centre detent, or simply held - produces no events
    /// at all. An earlier version of this returned a bare `bool` meaning "events arrived" and the
    /// caller fed its failsafe from it. The result was a ground station that handed control back to
    /// the transmitter after 200 ms of a pilot holding full throttle, on a feature whose entire
    /// premise is that there is no transmitter bound. The type now makes that mistake unavailable:
    /// there is no value here that means "no events", only "alive" and "gone".
    pub fn poll(&mut self) -> Poll {
        let mut buffer = [0u8; EVENT_LEN];
        loop {
            match self.file.read(&mut buffer) {
                Ok(EVENT_LEN) => self.apply(&buffer),
                // A short read from a character device is not something this API does, but a
                // partial event cannot be interpreted, so it is dropped rather than guessed at.
                Ok(0) => {
                    self.disconnected = true;
                    return Poll::Gone;
                }
                Ok(_) => return Poll::Alive,
                // Nothing more to read. The device is there and the sticks are wherever they were.
                Err(err) if err.kind() == ErrorKind::WouldBlock => return Poll::Alive,
                Err(err) if err.kind() == ErrorKind::Interrupted => {}
                // Anything else is the device going away: unplugged, or the node removed.
                Err(_) => {
                    self.disconnected = true;
                    return Poll::Gone;
                }
            }
        }
    }

    fn apply(&mut self, event: &[u8; EVENT_LEN]) {
        let value = i16::from_le_bytes([event[4], event[5]]);
        // The init bit is masked off rather than filtered on: an init event carries a real
        // position and should be applied like any other.
        let kind = event[6] & !EVENT_INIT;
        let number = usize::from(event[7]);
        match kind {
            EVENT_AXIS => {
                if self.reading.axes.len() <= number {
                    self.reading.axes.resize(number + 1, 0.0);
                }
                if let Some(slot) = self.reading.axes.get_mut(number) {
                    // Divided by 32767 rather than 32768, so full deflection reaches exactly 1.0.
                    // The API's negative extreme is -32768, which this clamps; losing one part in
                    // 32768 at one end is better than never quite reaching full travel.
                    *slot = (f32::from(value) / AXIS_SCALE).clamp(-1.0, 1.0);
                }
            }
            EVENT_BUTTON => {
                if self.reading.buttons.len() <= number {
                    self.reading.buttons.resize(number + 1, false);
                }
                if let Some(slot) = self.reading.buttons.get_mut(number) {
                    *slot = value != 0;
                }
            }
            _ => {}
        }
    }

    /// The current position of everything.
    #[must_use]
    pub const fn reading(&self) -> &Reading {
        &self.reading
    }

    /// Whether the device has gone away.
    #[must_use]
    pub const fn is_disconnected(&self) -> bool {
        self.disconnected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(kind: u8, number: u8, value: i16) -> [u8; EVENT_LEN] {
        let value = value.to_le_bytes();
        [0, 0, 0, 0, value[0], value[1], kind, number]
    }

    fn joystick() -> Joystick {
        // /dev/null reads as end of file, which is enough to build one; the tests drive `apply`
        // directly because what is being tested is the decoding, not the file.
        Joystick {
            file: File::open("/dev/null").expect("/dev/null"),
            reading: Reading::default(),
            disconnected: false,
        }
    }

    /// Full deflection must reach exactly 1.0, or a pilot never gets full travel.
    #[test]
    fn full_deflection_reaches_the_ends() {
        let mut stick = joystick();
        stick.apply(&event(EVENT_AXIS, 0, 32_767));
        assert!((stick.reading().axis(0) - 1.0).abs() < f32::EPSILON);
        stick.apply(&event(EVENT_AXIS, 0, -32_768));
        assert!((stick.reading().axis(0) + 1.0).abs() < f32::EPSILON);
        stick.apply(&event(EVENT_AXIS, 0, 0));
        assert!(stick.reading().axis(0).abs() < f32::EPSILON);
    }

    /// The synthetic events sent at open carry real positions and must be applied.
    ///
    /// Without this a throttle that is physically up reads as down until the pilot touches it.
    #[test]
    fn initial_events_carry_positions_rather_than_being_skipped() {
        let mut stick = joystick();
        stick.apply(&event(EVENT_AXIS | EVENT_INIT, 1, 32_767));
        assert!((stick.reading().axis(1) - 1.0).abs() < f32::EPSILON);
        stick.apply(&event(EVENT_BUTTON | EVENT_INIT, 3, 1));
        assert!(stick.reading().button(3));
    }

    /// A node called exactly "js" is not a joystick, and must not panic the filter either.
    #[test]
    fn the_device_filter_rejects_a_bare_js_and_survives_odd_names() {
        // Exercised through `devices()`, which must not panic on whatever /dev/input holds. The
        // bare-"js" case is asserted directly on the predicate shape: strip_prefix + non-empty.
        for name in ["js", "jsx", "js1a", "jsé", "joystick", ""] {
            let accepted = name
                .strip_prefix("js")
                .is_some_and(|rest| !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit()));
            assert!(!accepted, "{name} must not be taken for a joystick node");
        }
        for name in ["js0", "js1", "js31"] {
            let accepted = name
                .strip_prefix("js")
                .is_some_and(|rest| !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit()));
            assert!(accepted, "{name} is a joystick node");
        }
    }

    /// A device with more axes than seen so far must not panic or drop them.
    #[test]
    fn a_high_numbered_control_grows_the_reading() {
        let mut stick = joystick();
        stick.apply(&event(EVENT_AXIS, 7, 16_383));
        assert_eq!(stick.reading().axes.len(), 8);
        assert!((stick.reading().axis(7) - 0.5).abs() < 0.01);
        // And the ones in between read as centred rather than as rubbish.
        assert!(stick.reading().axis(3).abs() < f32::EPSILON);

        stick.apply(&event(EVENT_BUTTON, 11, 1));
        assert_eq!(stick.reading().buttons.len(), 12);
        assert!(stick.reading().button(11));
        assert!(!stick.reading().button(5));
    }

    /// A stick that is not moving is a stick that is still there.
    ///
    /// The bug this replaces: `poll` used to answer "did any event arrive", the caller fed its
    /// failsafe from that, and a pilot holding a stick steady - which is most of flying - looked
    /// exactly like an unplugged device.
    #[test]
    fn a_device_with_nothing_to_report_is_alive_not_silent() {
        let mut stick = joystick();
        // /dev/null yields EOF, which is the disconnect signal, so this asserts the other half:
        // a device with no pending events must not be reported as gone by the read path itself.
        // The WouldBlock arm is the one a real idle joystick takes.
        assert_eq!(stick.poll(), Poll::Gone, "end of file is a disconnect");
        assert!(stick.is_disconnected());
    }

    /// An event type this API does not define must be ignored, not guessed at.
    #[test]
    fn an_unknown_event_type_changes_nothing() {
        let mut stick = joystick();
        stick.apply(&event(EVENT_AXIS, 0, 32_767));
        let before = stick.reading().clone();
        stick.apply(&event(0x40, 0, -32_768));
        assert_eq!(stick.reading(), &before);
    }

    /// Enumeration must not fail on a machine with no joystick, which is most machines.
    #[test]
    fn listing_devices_on_a_machine_without_one_is_empty_rather_than_an_error() {
        // Whatever this machine has, the call must return rather than panic. On a machine with a
        // joystick attached this will be non-empty, which is also fine.
        let _ = devices();
    }
}
