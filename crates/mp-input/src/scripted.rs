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

//! Test scaffolding: a joystick that plays a script, standing in for `/dev/input/js*` on a
//! machine with no joystick (this one has none, and `/dev/uinput` is root's).
//!
//! A script is `name;step;step;...`. A step is `<ms>:a<number>=<value>` - an axis moved to a `js`
//! value, -32767 to 32767 - or `<ms>:b<number>=<0|1>` - a button let go or pressed - that many
//! milliseconds after the device is opened, or `<ms>:gone`, the device unplugged. The steps at
//! 0 ms are the `JS_EVENT_INIT` burst a real device sends as it is opened, which says how many
//! axes and buttons it has. After its last step the device holds still, as a real one does when
//! nobody touches it: a read blocks until the reader lets go.
//!
//! It reads like the kernel's device - eight-byte `js_event`s from a blocking `read` - so
//! [`crate::StickReader::spawn`] takes it exactly as it takes the real node.

use std::io::Read;
use std::time::{Duration, Instant};

use crate::event::{self, AXIS, BUTTON, INIT, LEN};

/// How long a device with nothing more to say sleeps between looks: it has nothing to look at,
/// so as long as a sleep can be.
const HOLD: Duration = Duration::from_secs(3_600);

/// One step of a script.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// A `js_event` at a time.
    Event {
        at: Duration,
        kind: u8,
        number: u8,
        value: i16,
    },
    /// The device unplugged: the next read returns end of file.
    Gone { at: Duration },
}

impl Step {
    const fn at(&self) -> Duration {
        match self {
            Self::Event { at, .. } | Self::Gone { at } => *at,
        }
    }
}

/// A parsed script.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Script {
    /// What the device calls itself.
    pub name: String,
    steps: Vec<Step>,
}

impl Script {
    /// Reads a script.
    ///
    /// # Errors
    /// A step that is not one of the three forms, or a script with no name.
    pub fn parse(spec: &str) -> Result<Self, String> {
        let mut parts = spec.split(';');
        let name = parts.next().unwrap_or("").trim().to_owned();
        if name.is_empty() {
            return Err("a scripted joystick needs a name".to_owned());
        }
        let mut steps = Vec::new();
        for part in parts.map(str::trim).filter(|part| !part.is_empty()) {
            let (at, what) = part
                .split_once(':')
                .ok_or_else(|| format!("{part:?}: expected <ms>:<step>"))?;
            let at = Duration::from_millis(
                at.trim()
                    .parse()
                    .map_err(|_| format!("{part:?}: {at:?} is not milliseconds"))?,
            );
            let what = what.trim();
            if what == "gone" {
                steps.push(Step::Gone { at });
                continue;
            }
            let (control, value) = what
                .split_once('=')
                .ok_or_else(|| format!("{part:?}: expected a<n>=<value> or b<n>=<0|1>"))?;
            let (kind, number) = match control.split_at_checked(1) {
                Some(("a", number)) => (AXIS, number),
                Some(("b", number)) => (BUTTON, number),
                _ => return Err(format!("{part:?}: {control:?} is not a<n> or b<n>")),
            };
            let number = number
                .parse()
                .map_err(|_| format!("{part:?}: {number:?} is not a control number"))?;
            let value = value
                .trim()
                .parse()
                .map_err(|_| format!("{part:?}: {value:?} is not a value"))?;
            let kind = if at.is_zero() { kind | INIT } else { kind };
            steps.push(Step::Event {
                at,
                kind,
                number,
                value,
            });
        }
        steps.sort_by_key(Step::at);
        Ok(Self { name, steps })
    }

    /// How many controls of a kind the opening burst reports: one more than the highest number.
    fn count(&self, of: u8) -> usize {
        self.steps
            .iter()
            .filter_map(|step| match step {
                Step::Event {
                    at, kind, number, ..
                } if at.is_zero() && kind & !INIT == of => Some(usize::from(*number) + 1),
                _ => None,
            })
            .max()
            .unwrap_or(0)
    }

    /// How many axes the device has.
    #[must_use]
    pub fn axes(&self) -> usize {
        self.count(AXIS)
    }

    /// How many buttons the device has.
    #[must_use]
    pub fn buttons(&self) -> usize {
        self.count(BUTTON)
    }

    /// The device, opened now: its clock starts here.
    #[must_use]
    pub fn open(&self) -> ScriptedDevice {
        ScriptedDevice {
            steps: self.steps.clone(),
            opened: Instant::now(),
            next: 0,
        }
    }
}

/// A script being played.
#[derive(Debug)]
pub struct ScriptedDevice {
    steps: Vec<Step>,
    opened: Instant,
    next: usize,
}

impl Read for ScriptedDevice {
    /// Blocks until the next step is due, then returns every event due by then that fits, as the
    /// kernel returns everything queued; end of file at a `gone`.
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        loop {
            let Some(step) = self.steps.get(self.next).copied() else {
                std::thread::sleep(HOLD);
                continue;
            };
            let due = self.opened + step.at();
            let now = Instant::now();
            if now < due {
                std::thread::sleep(due - now);
                continue;
            }
            if matches!(step, Step::Gone { .. }) {
                return Ok(0);
            }
            if buf.len() < LEN {
                return Err(std::io::ErrorKind::InvalidInput.into());
            }
            let mut written = 0;
            while let Some(Step::Event {
                at,
                kind,
                number,
                value,
            }) = self.steps.get(self.next).copied()
            {
                let Some(slot) = buf.get_mut(written..written + LEN) else {
                    break;
                };
                if self.opened + at > now {
                    break;
                }
                let time = u32::try_from(at.as_millis()).unwrap_or(u32::MAX);
                slot.copy_from_slice(&event::encode(time, kind, number, value));
                written += LEN;
                self.next += 1;
            }
            return Ok(written);
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    /// The opening burst says what the device has; later steps are ordinary events.
    #[test]
    fn a_script_parses_and_counts_its_controls() {
        let script = Script::parse("Scripted Stick;0:a0=0;0:a1=0;0:b0=0;0:b2=0;50:a0=16384;90:gone")
            .unwrap();
        assert_eq!(script.name, "Scripted Stick");
        assert_eq!(script.axes(), 2);
        assert_eq!(script.buttons(), 3);
        assert!(Script::parse(";0:a0=0").is_err());
        assert!(Script::parse("x;0:c0=0").is_err());
        assert!(Script::parse("x;soon:a0=0").is_err());
    }

    /// Played back in time, as `js_event`s, the opening burst flagged as such, then end of file.
    #[test]
    fn a_script_plays_its_events_in_time_and_then_goes() {
        let script = Script::parse("s;0:a0=-5;0:b1=1;40:a0=300;80:gone").unwrap();
        let mut device = script.open();
        let mut buf = [0u8; 64];
        let first = device.read(&mut buf).unwrap();
        assert_eq!(first, 16, "the opening burst in one read");
        assert_eq!(buf[..8], event::encode(0, AXIS | INIT, 0, -5));
        assert_eq!(buf[8..16], event::encode(0, BUTTON | INIT, 1, 1));
        let before = Instant::now();
        assert_eq!(device.read(&mut buf).unwrap(), 8);
        assert_eq!(buf[..8], event::encode(40, AXIS, 0, 300));
        assert!(device.opened.elapsed() >= Duration::from_millis(40));
        assert_eq!(device.read(&mut buf).unwrap(), 0, "unplugged");
        assert!(before.elapsed() >= Duration::from_millis(30));
    }
}
