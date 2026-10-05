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

//! The browser build's plugins: wasmtime's runtime in a web page, running plugins the desktop
//! compiled to Pulley bytecode ([`crate::precompile_for_web`], carried by mp-gui's build script),
//! on wasmtime's interpreter with [`crate::web_config`]'s settings.
//!
//! The one file of this crate with `unsafe`, by the owner's ruling of 2026-10-05 (browser plugins,
//! this file only): wasmtime's own platform layer, which a page has none of, needs functions it
//! calls by name - its thread's pointers and its locks - and loading compiled bytecode is unsafe in
//! wasmtime because it cannot check where the bytes came from.

#![allow(unsafe_code)]

use std::cell::Cell;
use std::ptr;
use std::sync::atomic::{AtomicUsize, Ordering};

use wasmtime::Engine;
use wasmtime::component::Component;

use crate::Fault;

thread_local! {
    /// wasmtime's thread-local pointers: slot 0 the runtime's, slot 1 component-model-async's.
    static TLS: [Cell<*mut u8>; 2] = const { [Cell::new(ptr::null_mut()), Cell::new(ptr::null_mut())] };
}

/// wasmtime's custom platform (wasmtime 48, `runtime/vm/sys/custom/capi.rs`): the thread's
/// pointer in `slot`, null until set. wasmtime passes only 0 or 1; anything else is null.
// Called by name from wasmtime's runtime, which `unreachable_pub` cannot see.
#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_tls_get(slot: usize) -> *mut u8 {
    TLS.with(|tls| tls.get(slot).map_or(ptr::null_mut(), Cell::get))
}

/// wasmtime's custom platform: sets the thread's pointer in `slot` (0 or 1; anything else is
/// ignored).
// Called by name from wasmtime's runtime, which `unreachable_pub` cannot see.
#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_tls_set(slot: usize, value: *mut u8) {
    TLS.with(|tls| {
        if let Some(cell) = tls.get(slot) {
            cell.set(value);
        }
    });
}

// wasmtime's custom platform's locks ("custom-sync-primitives"): a word wasmtime keeps for each,
// zero when free, which these take and give back by spinning on it. Its critical sections are
// short, and a page's main thread could not sleep in one anyway.

/// The word wasmtime keeps for a lock.
///
/// # Safety
/// `lock` is a live, aligned word wasmtime owns and hands only to these functions.
unsafe fn word<'a>(lock: *mut usize) -> &'a AtomicUsize {
    // SAFETY: as the caller promises, and AtomicUsize has usize's size and alignment.
    unsafe { AtomicUsize::from_ptr(lock) }
}

/// A writer holds the read-write lock: no reader may.
const WRITER: usize = usize::MAX;

// Called by name from wasmtime's runtime, which `unreachable_pub` cannot see.
#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_lock_free(_lock: *mut usize) {}

#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_lock_acquire(lock: *mut usize) {
    // SAFETY: wasmtime's word for this lock (`word`'s contract).
    let word = unsafe { word(lock) };
    while word
        .compare_exchange_weak(0, 1, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        std::hint::spin_loop();
    }
}

#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_lock_release(lock: *mut usize) {
    // SAFETY: wasmtime's word for this lock.
    unsafe { word(lock) }.store(0, Ordering::Release);
}

#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_rwlock_read(lock: *mut usize) {
    // SAFETY: wasmtime's word for this lock.
    let word = unsafe { word(lock) };
    loop {
        let readers = word.load(Ordering::Relaxed);
        if readers != WRITER
            && word
                .compare_exchange_weak(readers, readers + 1, Ordering::Acquire, Ordering::Relaxed)
                .is_ok()
        {
            return;
        }
        std::hint::spin_loop();
    }
}

#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_rwlock_read_release(lock: *mut usize) {
    // SAFETY: wasmtime's word for this lock.
    unsafe { word(lock) }.fetch_sub(1, Ordering::Release);
}

#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_rwlock_write(lock: *mut usize) {
    // SAFETY: wasmtime's word for this lock.
    let word = unsafe { word(lock) };
    while word
        .compare_exchange_weak(0, WRITER, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        std::hint::spin_loop();
    }
}

#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_rwlock_write_release(lock: *mut usize) {
    // SAFETY: wasmtime's word for this lock.
    unsafe { word(lock) }.store(0, Ordering::Release);
}

#[allow(unreachable_pub)]
#[unsafe(no_mangle)]
pub extern "C" fn wasmtime_sync_rwlock_free(_lock: *mut usize) {}

/// The plugin in `bytes`, Pulley bytecode [`crate::precompile_for_web`] made.
///
/// # Errors
/// [`Fault::Load`] when wasmtime refuses them: another wasmtime's, other settings', or not
/// bytecode at all.
pub(crate) fn deserialize(engine: &Engine, bytes: &[u8]) -> Result<Component, Fault> {
    // SAFETY: wasmtime's contract is that the bytes are its own compiled output, which it checks
    // the version and settings of but cannot prove the origin of. A web page has no files, so the
    // only plugins it loads are the built-in ones the planner carries (mp-gui's build script runs
    // precompile_for_web over them with this crate's web_config, at the version Cargo.lock pins
    // for both), compiled in with the planner itself.
    unsafe { Component::deserialize(engine, bytes) }.map_err(|err| Fault::Load(format!("{err:#}")))
}
