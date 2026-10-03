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

//! `plugins/example6-mapicondesc.cs`, "MapIconDesc": "Change icon Description" on the flight
//! screen's map menu, which asks for the vehicle icon's description template and keeps it in the
//! settings as `mapicondesc`.

use std::sync::{Mutex, PoisonError};

use mp_plugins::Guest;
use mp_plugins::host::{self, MapMenu};

/// The menu entry's id.
static ENTRY: Mutex<Option<u32>> = Mutex::new(None);

/// The template the C# starts from when the setting is empty.
const DEFAULT: &str = "{alt}{altunit} {airspeed}{speedunit} id:{sysid} Sats:{satcount} HDOP:{gpshdop} Volts:{battery_voltage}";

struct MapIconDesc;

impl Guest for MapIconDesc {
    fn name() -> String {
        "MapIconDesc".to_owned()
    }

    fn version() -> String {
        "0.10".to_owned()
    }

    fn author() -> String {
        "Michael Oborne".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        0.0
    }

    // The entry is added in `Init` here, not `Loaded`. `// C#: plugins/example6-mapicondesc.cs:30-38`
    fn init() -> bool {
        let id = host::menu_add(MapMenu::FlightData, None, "Change icon Description");
        *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) = Some(id);
        true
    }

    fn loaded() -> bool {
        true
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    // `// C#: plugins/example6-mapicondesc.cs:40-54`
    fn menu_click(id: u32, _lat: f64, _lng: f64) {
        if *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) != Some(id) {
            return;
        }
        let description = host::config_get("mapicondesc")
            .filter(|value| !value.is_empty())
            .unwrap_or_else(|| DEFAULT.to_owned());
        let Some(description) =
            host::input_box("Description", "What do you want it to show?", &description)
        else {
            return;
        };
        host::config_set("mapicondesc", &description);
    }

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(MapIconDesc);
