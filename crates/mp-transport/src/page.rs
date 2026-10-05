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

//! The browser build's links: a web page has no sockets, so a `tcp:`, `udpcl:` or `ws://` link is
//! handed to the page (web/www/link.js), which reaches the vehicle its own
//! way - ArduPilot's WebAssembly SITL running in the same page for this machine's 127.0.0.1, the
//! page's Tailscale node for any other address (www/tailscale.js), or the browser's WebSocket - and
//! the bytes cross between the page and the link's thread here.
//!
//! The link's thread is a Web Worker and may wait. The page's script runs on the browser's main
//! thread, which may never wait (`Atomics.wait` throws there), so its three calls only
//! `try_lock`: what one cannot hand over now, the page keeps for its next turn.

use mp_os::Lock as _;
use std::collections::VecDeque;
use std::io;
use std::sync::{Condvar, Mutex, MutexGuard};
use std::time::Duration;

use wasm_bindgen::prelude::wasm_bindgen;

use crate::{OpenError, Transport};

/// What the page and the link's thread share.
struct Shared {
    /// What the planner asked of the page, oldest first, until the page takes it: a link's URL,
    /// `close`, or a SITL to start or stop ([`start_sitl`], [`stop_sitl`]).
    requests: VecDeque<String>,
    /// Bytes from the vehicle, until the link reads them.
    inbox: VecDeque<u8>,
    /// Bytes for the vehicle, until the page takes them.
    outbox: Vec<u8>,
}

static SHARED: Mutex<Shared> = Mutex::new(Shared {
    requests: VecDeque::new(),
    inbox: VecDeque::new(),
    outbox: Vec::new(),
});
/// Rung when the page hands bytes over.
static ARRIVED: Condvar = Condvar::new();

fn shared() -> MutexGuard<'static, Shared> {
    SHARED
        .os_lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// The page: the next thing the planner asked of it, if any - a link's URL as typed, `close`,
/// or a SITL's start (`sitl`, then the module and each argument on a line of its own) or stop
/// (`sitl-stop`).
#[wasm_bindgen]
#[must_use]
pub fn page_link_requested() -> Option<String> {
    SHARED
        .try_lock()
        .ok()
        .and_then(|mut shared| shared.requests.pop_front())
}

/// Asks the page to start ArduPilot's WebAssembly SITL `module` (a file of its `sitl/` folder,
/// such as `arducopter.js`) with `arguments`, in place of any it runs: the browser build's
/// "try local wasm", which the desktop starts under Node (mp-gui's `sitl::launcher`). Its SERIAL0
/// is then what a `tcp:` link reaches. `folder` is the vehicle's SITL folder, where the page keeps
/// its eeprom.bin - its parameters - as the desktop's bridge keeps it in the same folder on disk.
pub fn start_sitl(module: &str, folder: &str, arguments: &[String]) {
    let mut request = format!("sitl\n{module}\n{folder}");
    for argument in arguments {
        request.push('\n');
        request.push_str(argument);
    }
    shared().requests.push_back(request);
}

/// Asks the page to stop the SITL it runs, if any.
pub fn stop_sitl() {
    shared().requests.push_back("sitl-stop".to_owned());
}

/// The page: bytes from the vehicle. False when the link's thread holds the buffer this moment;
/// the page keeps them and tries again.
#[wasm_bindgen]
#[must_use]
pub fn page_link_push(bytes: &[u8]) -> bool {
    let Ok(mut shared) = SHARED.try_lock() else {
        return false;
    };
    shared.inbox.extend(bytes);
    drop(shared);
    ARRIVED.notify_all();
    true
}

/// The page: the bytes the planner wrote for the vehicle since the last call (none when the
/// link's thread holds the buffer this moment).
#[wasm_bindgen]
#[must_use]
pub fn page_link_take() -> Vec<u8> {
    SHARED
        .try_lock()
        .map(|mut shared| std::mem::take(&mut shared.outbox))
        .unwrap_or_default()
}

/// A link the page carries.
#[derive(Debug)]
pub struct PageTransport {
    description: String,
    timeout: Duration,
    open: bool,
}

impl PageTransport {
    /// Asks the page for the link at `url`; the bytes of any link before it are dropped.
    ///
    /// # Errors
    /// None today: the page answers by sending bytes or not.
    pub fn open(url: &str) -> Result<Self, OpenError> {
        let mut shared = shared();
        shared.requests.push_back(url.to_owned());
        shared.inbox.clear();
        shared.outbox.clear();
        Ok(Self {
            description: format!("{url} (through the page)"),
            timeout: Duration::from_millis(100),
            open: true,
        })
    }
}

impl Transport for PageTransport {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut shared = shared();
        if shared.inbox.is_empty() {
            // A Condvar's timed wait is a timed `memory.atomic.wait32` here: no clock is read.
            shared = ARRIVED
                .wait_timeout(shared, self.timeout)
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .0;
        }
        let count = buf.len().min(shared.inbox.len());
        for (slot, byte) in buf.iter_mut().zip(shared.inbox.drain(..count)) {
            *slot = byte;
        }
        Ok(count)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        shared().outbox.extend_from_slice(buf);
        Ok(())
    }

    fn description(&self) -> &str {
        &self.description
    }

    fn is_open(&self) -> bool {
        self.open
    }

    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        self.timeout = timeout;
        Ok(())
    }

    fn close(&mut self) {
        if self.open {
            self.open = false;
            shared().requests.push_back("close".to_owned());
        }
    }
}
