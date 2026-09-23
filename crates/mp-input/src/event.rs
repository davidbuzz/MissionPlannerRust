//! The `js_event` record the Linux joystick API emits, and what it does to a [`Reading`].
//!
//! Separate from `linux` because two readers share it: [`crate::linux::Joystick`], which polls a
//! non-blocking device from whatever calls it, and [`crate::StickReader`], which blocks on a device
//! from a thread of its own. The decoding is the part that has to agree between them, so there is
//! one copy of it. It is also not Linux-only in the sense that matters: it is eight bytes and some
//! arithmetic, so the reader can be tested on any Unix against a fake device fed these records.

use crate::Reading;

/// Size of a `js_event`: `__u32 time`, `__s16 value`, `__u8 type`, `__u8 number`.
pub const LEN: usize = 8;

/// `JS_EVENT_BUTTON`.
pub const BUTTON: u8 = 0x01;
/// `JS_EVENT_AXIS`.
pub const AXIS: u8 = 0x02;
/// `JS_EVENT_INIT`, or-ed into the type on the synthetic events sent when a device is opened.
///
/// Those carry the current position of everything, which is exactly what is wanted: without them
/// a device reads as centred until something moves, and a throttle that is actually up reads as
/// down until the pilot touches it.
pub const INIT: u8 = 0x80;

/// Full scale of an axis in the joystick API.
const AXIS_SCALE: f32 = 32_767.0;

/// Builds a `js_event` as the kernel lays it out.
///
/// For fake devices. Nothing in this crate reads `time`, which is the kernel's jiffies in
/// milliseconds and not on any clock this process can compare against; it is carried so a fake
/// can produce exactly what a real device would.
#[must_use]
pub const fn encode(time_ms: u32, kind: u8, number: u8, value: i16) -> [u8; LEN] {
    let time = time_ms.to_le_bytes();
    let value = value.to_le_bytes();
    [
        time[0], time[1], time[2], time[3], value[0], value[1], kind, number,
    ]
}

/// Applies one event to a reading.
pub(crate) fn apply(reading: &mut Reading, event: &[u8; LEN]) {
    let value = i16::from_le_bytes([event[4], event[5]]);
    // The init bit is masked off rather than filtered on: an init event carries a real position
    // and should be applied like any other.
    let kind = event[6] & !INIT;
    let number = usize::from(event[7]);
    match kind {
        AXIS => {
            if reading.axes.len() <= number {
                reading.axes.resize(number + 1, 0.0);
            }
            if let Some(slot) = reading.axes.get_mut(number) {
                // Divided by 32767 rather than 32768, so full deflection reaches exactly 1.0. The
                // API's negative extreme is -32768, which this clamps; losing one part in 32768 at
                // one end is better than never quite reaching full travel.
                *slot = (f32::from(value) / AXIS_SCALE).clamp(-1.0, 1.0);
            }
        }
        BUTTON => {
            if reading.buttons.len() <= number {
                reading.buttons.resize(number + 1, false);
            }
            if let Some(slot) = reading.buttons.get_mut(number) {
                *slot = value != 0;
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The layout is the kernel's, little-endian, time first. A fake that gets this wrong tests a
    /// device that does not exist.
    #[test]
    fn encoding_matches_the_kernel_layout() {
        let event = encode(0x0403_0201, AXIS | INIT, 7, -2);
        assert_eq!(event, [0x01, 0x02, 0x03, 0x04, 0xFE, 0xFF, 0x82, 7]);
    }

    /// What is encoded is what is decoded, through the same function both readers use.
    #[test]
    fn an_encoded_event_applies_as_itself() {
        let mut reading = Reading::default();
        apply(&mut reading, &encode(123, AXIS, 2, 32_767));
        apply(&mut reading, &encode(124, BUTTON, 1, 1));
        assert!((reading.axis(2) - 1.0).abs() < f32::EPSILON);
        assert!(reading.button(1));
        assert!(!reading.button(0));
    }
}
