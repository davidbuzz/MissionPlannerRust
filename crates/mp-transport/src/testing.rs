//! Deterministic test doubles.
//!
//! Link bugs are timing bugs, and timing bugs found in production are found in the air. These
//! doubles make the nasty cases - partial writes, dropped bytes, duplicated bytes, disconnects
//! mid-frame - reproducible in a unit test.

use std::collections::VecDeque;
use std::io;
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
        let a_to_b = Arc::new(Mutex::new(VecDeque::new()));
        let b_to_a = Arc::new(Mutex::new(VecDeque::new()));
        (
            LoopbackEnd {
                name: "loopback:a".to_owned(),
                tx: Arc::clone(&a_to_b),
                rx: Arc::clone(&b_to_a),
                open: true,
                fault: Fault::default(),
            },
            LoopbackEnd {
                name: "loopback:b".to_owned(),
                tx: b_to_a,
                rx: a_to_b,
                open: true,
                fault: Fault::default(),
            },
        )
    }
}

/// How a link should misbehave, for fault injection.
#[derive(Debug, Clone, Copy, Default)]
pub struct Fault {
    /// Drop every Nth byte read (0 = never).
    pub drop_every: usize,
    /// Duplicate every Nth byte read (0 = never).
    pub duplicate_every: usize,
    /// Cap each read at this many bytes (0 = uncapped).
    pub max_read: usize,
}

/// One end of a [`Loopback`].
#[derive(Debug)]
pub struct LoopbackEnd {
    name: String,
    tx: Arc<Mutex<VecDeque<u8>>>,
    rx: Arc<Mutex<VecDeque<u8>>>,
    open: bool,
    fault: Fault,
}

impl LoopbackEnd {
    /// Injects faults into subsequent reads.
    #[must_use]
    pub const fn with_fault(mut self, fault: Fault) -> Self {
        self.fault = fault;
        self
    }

    /// Bytes waiting to be read.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.rx.lock().map(|q| q.len()).unwrap_or(0)
    }

    /// Simulates the peer vanishing.
    pub fn disconnect(&mut self) {
        self.open = false;
    }
}

impl Transport for LoopbackEnd {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if !self.open {
            return Ok(0);
        }
        let Ok(mut queue) = self.rx.lock() else {
            return Err(io::Error::other("loopback queue poisoned"));
        };

        let mut limit = buf.len();
        if self.fault.max_read > 0 {
            limit = limit.min(self.fault.max_read);
        }

        let mut written = 0usize;
        let mut taken = 0usize;
        while written < limit {
            let Some(byte) = queue.pop_front() else { break };
            taken += 1;
            if self.fault.drop_every > 0 && taken.is_multiple_of(self.fault.drop_every) {
                continue;
            }
            if let Some(slot) = buf.get_mut(written) {
                *slot = byte;
                written += 1;
            }
            if self.fault.duplicate_every > 0
                && taken.is_multiple_of(self.fault.duplicate_every)
                && written < limit
                && let Some(slot) = buf.get_mut(written)
            {
                *slot = byte;
                written += 1;
            }
        }
        Ok(written)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        if !self.open {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "loopback closed",
            ));
        }
        let Ok(mut queue) = self.tx.lock() else {
            return Err(io::Error::other("loopback queue poisoned"));
        };
        queue.extend(buf.iter().copied());
        Ok(())
    }

    fn description(&self) -> String {
        self.name.clone()
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        Ok(())
    }

    fn close(&mut self) {
        self.open = false;
    }
}
