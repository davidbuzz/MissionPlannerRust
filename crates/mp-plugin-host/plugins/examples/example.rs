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

//! `plugins/example.cs`: the smallest plugin - no name, no version, no author, and `Init`,
//! `Loaded` and `Exit` all false, so the loader never keeps it.
//! `// C#: plugins/example.cs:15-34`

use mp_plugins::Guest;

struct Urlmod;

impl Guest for Urlmod {
    fn name() -> String {
        String::new()
    }

    fn version() -> String {
        String::new()
    }

    fn author() -> String {
        String::new()
    }

    fn loop_rate_hz() -> f32 {
        0.0
    }

    // `// C#: plugins/example.cs:21-34`
    fn init() -> bool {
        false
    }

    fn loaded() -> bool {
        false
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        false
    }

    fn menu_click(_id: u32, _lat: f64, _lng: f64) {}

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(Urlmod);
