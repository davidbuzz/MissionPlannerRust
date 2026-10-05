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

//! The page's side of the planner's files (the owner's request of 2026-10-05: settings, missions
//! and logs to survive a reload). In a web page the planner's files are held in memory
//! (`mp_os::fs::mem`); web/www/storage.js keeps them in the browser's own storage, asking every
//! half second for what changed. The files the last visit kept are taken in at the top of `main`
//! (`mp_os::fs::preload_from_page`).

/// The page: what the planner's files changed since the page last asked, packed as storage.js
/// reads it (`mp_os::fs::mem::encode`); empty when nothing has.
#[wasm_bindgen::prelude::wasm_bindgen]
#[must_use]
// Exported to the page by wasm-bindgen, not to another crate.
#[allow(unreachable_pub)]
pub fn planner_storage_take() -> Vec<u8> {
    mp_os::fs::mem::encode(&mp_os::fs::mem::STORE.take_changes())
}
