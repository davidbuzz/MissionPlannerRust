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

//! Deterministic test doubles.
//!
//! Link bugs are timing bugs, and timing bugs found in production are found in the air. These
//! doubles make the nasty cases - partial writes, dropped bytes, duplicated bytes, disconnects
//! mid-frame - reproducible in a unit test.

use mp_os::Lock as _;
use std::collections::VecDeque;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::Transport;

/// A pair of in-memory endpoints wired to each other.
#[derive(Debug, Default, Clone, Copy)]
pub struct Loopback;

impl Loopback {
    /// Creates a connected pair.
    #[must_use]
    pub fn pair() -> (LoopbackEnd, LoopbackEnd) {
        let a_to_b = Arc::new(Mutex::new(Wire::default()));
        let b_to_a = Arc::new(Mutex::new(Wire::default()));
        let plug = Plug(Arc::new(AtomicBool::new(true)));
        (
            LoopbackEnd {
                name: "loopback:a".to_owned(),
                tx: Arc::clone(&a_to_b),
                rx: Arc::clone(&b_to_a),
                plug: plug.clone(),
                open: true,
            },
            LoopbackEnd {
                name: "loopback:b".to_owned(),
                tx: b_to_a,
                rx: a_to_b,
                plug,
                open: true,
            },
        )
    }
}

/// How a link should misbehave, for fault injection.
///
/// A fault describes the path *into* the end it is set on: byte faults act as that end reads,
/// write faults as its peer writes. Counters run over the whole stream rather than per call, so
/// where a fault lands depends on the bytes and not on how a reader happens to chunk them - which
/// is what lets a test work out exactly which frames a fault must have cost.
#[derive(Debug, Clone, Copy, Default)]
pub struct Fault {
    /// Drop every Nth byte read (0 = never).
    pub drop_every: usize,
    /// Duplicate every Nth byte read (0 = never).
    pub duplicate_every: usize,
    /// Cap each read at this many bytes (0 = uncapped).
    pub max_read: usize,
    /// Deliver every Nth write twice (0 = never): a radio's retransmission, a repeated datagram.
    pub repeat_write_every: usize,
    /// Hold every Nth write back until the one after it has been delivered (0 = never): datagrams
    /// arriving out of order. A held write is not also repeated.
    pub reorder_write_every: usize,
    /// Pull the plug once this many bytes have been read (0 = never). The read that reaches the
    /// count returns the bytes before the cut; the read after it fails.
    pub unplug_after: usize,
}

/// The cable between the two ends of a [`Loopback`].
///
/// Pulling it is a surprise unplug: whatever was still in flight is lost, and each end fails the
/// next read or write it attempts, the way a USB serial port reports a hang-up. It can be pulled
/// from another thread while a reader owns its end.
#[derive(Debug, Clone)]
pub struct Plug(Arc<AtomicBool>);

impl Plug {
    /// Pulls the cable out.
    pub fn pull(&self) {
        self.0.store(false, Ordering::Release);
    }

    /// Whether the cable is still in.
    #[must_use]
    pub fn is_connected(&self) -> bool {
        self.0.load(Ordering::Acquire)
    }
}

/// One direction of a [`Loopback`], with the state its faults need.
#[derive(Debug, Default)]
struct Wire {
    bytes: VecDeque<u8>,
    fault: Fault,
    /// Bytes taken off the wire so far, for the every-Nth byte faults.
    taken: usize,
    /// Bytes handed to the reader so far, for `unplug_after`.
    delivered: usize,
    /// The second copy of a duplicated byte that did not fit in the read that produced it.
    duplicate: Option<u8>,
    /// Writes put on the wire so far, for the every-Nth write faults.
    writes: usize,
    /// A write held back to arrive after the next one.
    held: Option<Vec<u8>>,
}

/// One end of a [`Loopback`].
#[derive(Debug)]
pub struct LoopbackEnd {
    name: String,
    tx: Arc<Mutex<Wire>>,
    rx: Arc<Mutex<Wire>>,
    plug: Plug,
    open: bool,
}

impl LoopbackEnd {
    /// Injects faults into the path towards this end.
    #[must_use]
    pub fn with_fault(self, fault: Fault) -> Self {
        if let Ok(mut wire) = self.rx.os_lock() {
            wire.fault = fault;
        }
        self
    }

    /// Bytes waiting to be read.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.rx.os_lock().map(|wire| wire.bytes.len()).unwrap_or(0)
    }

    /// The cable this end hangs off, for pulling it while something else owns the end.
    #[must_use]
    pub fn plug(&self) -> Plug {
        self.plug.clone()
    }

    /// Simulates the peer vanishing: pulls the plug under both ends.
    pub fn disconnect(&mut self) {
        self.plug.pull();
    }

    /// The error both ends report once the plug is out, noting that this end now knows.
    fn unplugged(&mut self) -> io::Error {
        self.open = false;
        // BrokenPipe because that is what the serialport crate reports when poll() sees a
        // hang-up, which is what a USB serial device does as it disappears.
        io::Error::new(io::ErrorKind::BrokenPipe, "loopback unplugged")
    }
}

impl Transport for LoopbackEnd {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        // The unplug first: it is what happened, even once a failed call has closed this end.
        if !self.plug.is_connected() {
            return Err(self.unplugged());
        }
        if !self.open {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "loopback closed",
            ));
        }
        let Ok(mut wire) = self.rx.os_lock() else {
            return Err(io::Error::other("loopback queue poisoned"));
        };
        let fault = wire.fault;

        let mut limit = buf.len();
        if fault.max_read > 0 {
            limit = limit.min(fault.max_read);
        }
        if fault.unplug_after > 0 {
            limit = limit.min(fault.unplug_after.saturating_sub(wire.delivered));
        }

        let mut written = 0usize;
        while written < limit {
            let Some(slot) = buf.get_mut(written) else {
                break;
            };
            if let Some(byte) = wire.duplicate.take() {
                *slot = byte;
                written += 1;
                continue;
            }
            let Some(byte) = wire.bytes.pop_front() else {
                break;
            };
            wire.taken += 1;
            if fault.drop_every > 0 && wire.taken.is_multiple_of(fault.drop_every) {
                continue;
            }
            *slot = byte;
            written += 1;
            if fault.duplicate_every > 0 && wire.taken.is_multiple_of(fault.duplicate_every) {
                // Delivered next, in this read if it fits and at the start of the next if not.
                wire.duplicate = Some(byte);
            }
        }

        wire.delivered += written;
        if fault.unplug_after > 0 && wire.delivered >= fault.unplug_after {
            self.plug.pull();
        }
        Ok(written)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        // The unplug first: it is what happened, even once a failed call has closed this end.
        if !self.plug.is_connected() {
            return Err(self.unplugged());
        }
        if !self.open {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "loopback closed",
            ));
        }
        let Ok(mut wire) = self.tx.os_lock() else {
            return Err(io::Error::other("loopback queue poisoned"));
        };
        let fault = wire.fault;
        wire.writes += 1;
        let nth = wire.writes;

        if fault.reorder_write_every > 0
            && nth.is_multiple_of(fault.reorder_write_every)
            && wire.held.is_none()
        {
            wire.held = Some(buf.to_vec());
            return Ok(());
        }
        wire.bytes.extend(buf.iter().copied());
        if fault.repeat_write_every > 0 && nth.is_multiple_of(fault.repeat_write_every) {
            wire.bytes.extend(buf.iter().copied());
        }
        if let Some(held) = wire.held.take() {
            wire.bytes.extend(held);
        }
        Ok(())
    }

    fn description(&self) -> &str {
        &self.name
    }

    fn is_open(&self) -> bool {
        // What this end knows, like a real port: an unplug is learned from the next read or
        // write, not the moment it happens.
        self.open
    }

    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        Ok(())
    }

    fn close(&mut self) {
        self.open = false;
    }
}
