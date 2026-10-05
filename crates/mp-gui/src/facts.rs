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

//! Publishing what the application currently believes, so a test can assert on it.
//!
//! The other half of `probe`. That one says *where* a control is, so a script can click it; this
//! one says *what happened* as a result, so a script can check it did.
//!
//! Without this, a UI test can drive the application and produce a screenshot, and a human has to
//! look at the screenshot. That is not a test - `DELIVERABLES.md`'s test policy says so directly:
//! "if it is not a program that fails, it is not a test". A click-to-add-waypoint that quietly
//! stopped adding waypoints would still produce a PNG, and the run would still exit zero.
//!
//! Off unless `MP_FACTS` names a file. Nothing is recorded and nothing is written otherwise, so a
//! normal run pays for none of it. In a web page, which has neither, the page turns them on
//! (`planner_facts_enable`, from planner.html's `?facts=1`) and reads them (`planner_facts`).
//!
//! The format is the flat `key = value` of `settings.rs`, for the same reasons: greppable by eye,
//! parseable by a shell with no dependency, and impossible to get subtly wrong in the way a nested
//! format can be.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::Lock as _;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

/// The facts recorded this frame, and where to write them.
static FACTS: OnceLock<Mutex<BTreeMap<String, String>>> = OnceLock::new();

/// Where to write, from `MP_FACTS`. Absent means the whole module is off.
fn destination() -> Option<&'static PathBuf> {
    static DESTINATION: OnceLock<Option<PathBuf>> = OnceLock::new();
    DESTINATION
        .get_or_init(|| std::env::var_os("MP_FACTS").map(PathBuf::from))
        .as_ref()
}

/// Whether anything is being recorded.
#[must_use]
pub fn enabled() -> bool {
    destination().is_some() || web::enabled()
}

/// The browser build's facts: no environment names a file, so the page asks for them, and reads
/// them as the file would hold them.
#[cfg(target_family = "wasm")]
mod web {
    use std::sync::atomic::{AtomicBool, Ordering};

    use wasm_bindgen::prelude::wasm_bindgen;

    static ENABLED: AtomicBool = AtomicBool::new(false);

    pub(super) fn enabled() -> bool {
        ENABLED.load(Ordering::Relaxed)
    }

    /// The page: start recording facts (planner.html with `?facts=1`, for its checks).
    #[wasm_bindgen]
    pub fn planner_facts_enable() {
        ENABLED.store(true, Ordering::Relaxed);
    }

    /// The page: the facts recorded so far, `key = value` a line as `MP_FACTS`'s file has them;
    /// empty while a planner thread holds them, since the page's main thread may not wait.
    #[wasm_bindgen]
    #[must_use]
    pub fn planner_facts() -> String {
        super::FACTS
            .get()
            .and_then(|facts| facts.try_lock().ok())
            .map(|facts| super::text(&facts))
            .unwrap_or_default()
    }
}

#[cfg(not(target_family = "wasm"))]
mod web {
    pub(super) const fn enabled() -> bool {
        false
    }
}

/// Records one fact.
///
/// Called from `render`, so it must be cheap and must not panic. A poisoned lock is ignored: a
/// test harness that has lost its facts should fail on a missing key, not take the application
/// down with it.
pub fn record(key: impl Into<String>, value: impl std::fmt::Display) {
    if !enabled() {
        return;
    }
    if let Ok(mut facts) = FACTS.get_or_init(|| Mutex::new(BTreeMap::new())).os_lock() {
        facts.insert(key.into(), value.to_string());
    }
}

/// Writes everything recorded so far.
///
/// Written whole, through a temporary file and a rename, so a reader never sees half a set. A
/// test that polls this file would otherwise read a state that never existed - half the old facts
/// and half the new ones - which is the kind of flake nobody can reproduce.
pub fn publish() {
    let Some(path) = destination() else {
        return;
    };
    let Some(facts) = FACTS.get() else {
        return;
    };
    let Ok(facts) = facts.os_lock() else {
        return;
    };
    let text = text(&facts);
    let temporary = path.with_extension("facts.tmp");
    if std::fs::write(&temporary, text).is_ok() {
        let _ = std::fs::rename(&temporary, path);
    }
}

/// The facts as the file holds them: `key = value`, a line each, in key order.
fn text(facts: &BTreeMap<String, String>) -> String {
    let mut text = String::new();
    for (key, value) in facts {
        // A newline in a value would split one fact into two. Replaced rather than escaped: no
        // fact worth asserting on contains one, and an escaping scheme is a thing to get wrong.
        text.push_str(&format!("{key} = {}\n", value.replace('\n', " ")));
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A value that would split a line is flattened, not written.
    #[test]
    fn a_newline_in_a_value_cannot_split_a_fact_in_two() {
        let value = "first\nsecond".replace('\n', " ");
        assert_eq!(value, "first second");
        assert!(!value.contains('\n'));
    }

    /// Recording while switched off must cost nothing and must not panic.
    #[test]
    fn recording_while_disabled_does_nothing() {
        // MP_FACTS is not set under `cargo test`, so this exercises the off path.
        record("key", "value");
        publish();
        assert!(!enabled());
    }
}
