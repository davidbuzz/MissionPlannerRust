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

//! The std calls that panic in a web page rather than fail, each std's own on the desktop.
//!
//! On `wasm32-unknown-unknown` most of std's operating system calls return an error - a file
//! does not open, a socket does not connect - and the planner already handles that. Three panic
//! instead: `std::env::temp_dir` ("no filesystem on this platform"), `std::process::id` ("no
//! pids on this platform") and `std::env::split_paths`. Here a web page gets an answer that leads
//! to the ordinary error further on: a temporary folder no file can be made in, an id of 0, and
//! no folders on the path. (experiments/web-experiment/tools/port_os.py puts these in place.)
//!
//! And one the desktop has another way, [`http`]: a web page has no sockets, so the crates that
//! fetch over ureq on the desktop fetch through the browser in a page.

use std::ffi::OsStr;
use std::path::PathBuf;
#[cfg(target_family = "wasm")]
use std::sync::TryLockError;
use std::sync::{LockResult, Mutex, MutexGuard};

/// `std::env::temp_dir`; in a web page `/tmp`, where nothing can be written.
#[must_use]
pub fn temp_dir() -> PathBuf {
    #[cfg(not(target_family = "wasm"))]
    {
        std::env::temp_dir()
    }
    #[cfg(target_family = "wasm")]
    {
        PathBuf::from("/tmp")
    }
}

/// `std::process::id`; 0 in a web page, which has no processes.
#[must_use]
pub fn process_id() -> u32 {
    #[cfg(not(target_family = "wasm"))]
    {
        std::process::id()
    }
    #[cfg(target_family = "wasm")]
    {
        0
    }
}

/// `std::env::split_paths`, collected; nothing in a web page, which has no programs to find.
#[must_use]
pub fn split_paths(paths: &(impl AsRef<OsStr> + ?Sized)) -> std::vec::IntoIter<PathBuf> {
    #[cfg(not(target_family = "wasm"))]
    {
        std::env::split_paths(paths).collect::<Vec<_>>().into_iter()
    }
    #[cfg(target_family = "wasm")]
    {
        let _ = paths;
        Vec::new().into_iter()
    }
}

/// `Mutex::lock`, except on a web page's main thread, which a browser never lets wait
/// (`Atomics.wait` throws there, and the throw leaves whatever gpui had borrowed borrowed): there
/// it spins until the lock is free, as Emscripten's main thread does. The holder is a Web Worker
/// on a core of its own, so the spin lasts the holder's critical section.
///
/// For the locks the screen's thread shares with the planner's threads: on the desktop, and on a
/// worker, it is `lock` itself.
///
/// # Errors
/// A poisoned lock, as `lock`'s.
pub fn lock<T: ?Sized>(mutex: &Mutex<T>) -> LockResult<MutexGuard<'_, T>> {
    #[cfg(target_family = "wasm")]
    if on_main_thread() {
        loop {
            match mutex.try_lock() {
                Ok(guard) => return Ok(guard),
                Err(TryLockError::Poisoned(poisoned)) => return Err(poisoned),
                Err(TryLockError::WouldBlock) => std::hint::spin_loop(),
            }
        }
    }
    mutex.lock()
}

/// [`lock`] as a method, so a call reads as `mutex.os_lock()` where it read `mutex.lock()`
/// (experiments/web-experiment/tools/port_locks.py puts it in place).
pub trait Lock<T: ?Sized> {
    /// [`lock`].
    ///
    /// # Errors
    /// A poisoned lock, as `lock`'s.
    fn os_lock(&self) -> LockResult<MutexGuard<'_, T>>;
}

impl<T: ?Sized> Lock<T> for Mutex<T> {
    fn os_lock(&self) -> LockResult<MutexGuard<'_, T>> {
        lock(self)
    }
}

/// Whether this is the page's main thread rather than a Web Worker, asked of the browser once a
/// thread.
#[cfg(target_family = "wasm")]
fn on_main_thread() -> bool {
    thread_local! {
        static MAIN: bool = !wasm_thread::is_web_worker_thread();
    }
    MAIN.with(|main| *main)
}

/// An HTTP request in a web page, blocking: a synchronous `XMLHttpRequest`, which a browser
/// allows in a Web Worker - the threads the planner fetches tiles, terrain and catalogues on (on
/// the page's main thread it refuses a binary answer). The status and the body; a status that is
/// not a success is the caller's to judge, as ureq's `http_status_as_error(false)` leaves it.
///
/// The browser's rules apply: the server must let the page's origin read the answer (CORS), and
/// `User-Agent` and `Referer` are the browser's own, so a caller's are not sent.
///
/// # Errors
/// The browser's refusal - a network failure, a server that does not allow the origin - in its
/// own words.
#[cfg(target_family = "wasm")]
pub fn http(method: &str, url: &str, body: Option<&str>) -> Result<(u16, Vec<u8>), String> {
    let text = |err: wasm_bindgen::JsValue| format!("{url}: {err:?}");
    let request = web_sys::XmlHttpRequest::new().map_err(text)?;
    request.open_with_async(method, url, false).map_err(text)?;
    request.set_response_type(web_sys::XmlHttpRequestResponseType::Arraybuffer);
    match body {
        Some(body) => request.send_with_opt_str(Some(body)).map_err(text)?,
        None => request.send().map_err(text)?,
    }
    let status = request.status().map_err(text)?;
    if status == 0 {
        // What a browser reports for a request it would not let the page see (CORS) or that
        // never reached the server.
        return Err(format!("{url}: no answer the page may read"));
    }
    let answer = request.response().map_err(text)?;
    Ok((status, js_sys::Uint8Array::new(&answer).to_vec()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn on_the_desktop_lock_is_mutex_lock() {
        let mutex = Mutex::new(1);
        *lock(&mutex).expect("not poisoned") += 1;
        assert_eq!(*mutex.lock().expect("not poisoned"), 2);
        let poisoned = std::sync::Arc::new(Mutex::new(0));
        let held = std::sync::Arc::clone(&poisoned);
        let _ = wasm_thread::spawn(move || {
            let _guard = held.lock();
            panic!("poison it");
        })
        .join();
        assert!(lock(&poisoned).is_err());
    }

    #[test]
    fn on_the_desktop_each_is_std_s_own() {
        assert_eq!(temp_dir(), std::env::temp_dir());
        assert_eq!(process_id(), std::process::id());
        let joined = std::env::join_paths(["/a", "/b/c"]).expect("joinable");
        assert_eq!(
            split_paths(&joined).collect::<Vec<_>>(),
            std::env::split_paths(&joined).collect::<Vec<_>>()
        );
    }
}
