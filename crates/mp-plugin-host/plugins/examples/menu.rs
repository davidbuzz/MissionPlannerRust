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

//! `plugins/example2-menu.cs`, "Small stuff": "Fix mission top/bottom" on the planning screen's
//! map menu, which asks for a heading and puts two `DO_SET_SERVO`s at the top of the mission and
//! two at the bottom.
//!
//! Left out: `commands.Rows.RemoveAt(1)`, the click's last line, which reaches into the planning
//! screen's grid through `Host.MainForm.FlightPlanner.Controls.Find("Commands")`; the world
//! reaches the mission through `AddWPtoList`, `InsertWP` and `GetWPs` only.

use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, MapMenu, Waypoint};
use mp_plugins::{Guest, show};

/// `MAV_CMD_DO_SET_SERVO`.
const DO_SET_SERVO: u16 = 183;

/// The menu entry's id.
static ENTRY: Mutex<Option<u32>> = Mutex::new(None);

/// `MAV_CMD.DO_SET_SERVO` with its servo and PWM, the rest zero.
const fn servo(servo: f64, pwm: f64) -> Waypoint {
    Waypoint {
        command: DO_SET_SERVO,
        p1: servo,
        p2: pwm,
        p3: 0.0,
        p4: 0.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

struct SmallStuff;

impl Guest for SmallStuff {
    fn name() -> String {
        "Small stuff".to_owned()
    }

    fn version() -> String {
        "0.10".to_owned()
    }

    fn author() -> String {
        "EOSBandi".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        0.0
    }

    fn init() -> bool {
        true
    }

    // `// C#: plugins/example2-menu.cs:43-53`
    fn loaded() -> bool {
        let id = host::menu_add(MapMenu::FlightPlanner, None, "Fix mission top/bottom");
        *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) = Some(id);
        true
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    // `// C#: plugins/example2-menu.cs:65-79`
    fn menu_click(id: u32, _lat: f64, _lng: f64) {
        if *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) != Some(id) {
            return;
        }
        show("This is a sample plugin\nSee the source in the plugins folder");
        // `InputBox.Show(..., ref angle)`: the result is not looked at, so Cancel leaves "0".
        let angle = host::input_box("Enter Angle", "This will be the heading", "0")
            .unwrap_or_else(|| "0".to_owned());
        // `Int32.Parse` throws on anything else, and the handler goes no further.
        let Ok(angle) = angle.trim().parse::<i32>() else {
            return;
        };
        host::insert_wp(0, servo(9.0, f64::from(angle)));
        host::insert_wp(1, servo(10.0, 1000.0));
        let _ = host::add_wp(servo(9.0, 1000.0));
        let _ = host::add_wp(servo(10.0, 1000.0));
    }

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(SmallStuff);
