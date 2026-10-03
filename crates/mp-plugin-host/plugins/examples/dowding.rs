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

//! `plugins/Dowding/`, "Dowding", one of the C#'s four real plugins: a client of the Dowding
//! counter-drone service. Its web socket brings the tracked aircraft, drawn as markers on the
//! flight map; "Dowding" on the flight map's menu opens its settings page, "Dowding Point At"
//! points an antenna tracker at the spot clicked.
//!
//! Unimplementable in this world, recorded: everything on the network - the service's web socket
//! (`Start`, `WebAPIs.Dowding`), its login (Verify), the ONVIF camera and the CoT output - and the
//! antenna tracker on a serial or TCP port; a plugin here has no sockets and no serial ports. So
//! the vehicles never arrive and `Loop`'s markers never draw (the world has no marker surface
//! either). What remains is the C#'s own behaviour without a connection: the start at load
//! failing as its `catch` says, the settings the page keeps, and the tracker home it validates
//! and stores before its send fails.

use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, ControlKind, MapMenu, MessageButtons};
use mp_plugins::{Guest, control, show};

/// The two menu entries' ids: "Dowding", "Dowding Point At".
static ENTRIES: Mutex<Option<(u32, u32)>> = Mutex::new(None);

/// The tracker home's three boxes, as typed.
static HOME: Mutex<(String, String, String)> =
    Mutex::new((String::new(), String::new(), String::new()));

/// `loopratehz`.
static RATE: Mutex<f32> = Mutex::new(0.0);

/// `Settings.GetBoolean`: "True" or "true" is true, anything else false.
fn setting_true(key: &str) -> bool {
    host::config_get(key).is_some_and(|value| value.trim().eq_ignore_ascii_case("true"))
}

fn has(key: &str) -> bool {
    host::config_get(key).is_some()
}

/// `Start`: the credentials checked, then the web socket, which cannot open here and ends in the
/// C#'s `catch`. `// C#: plugins/Dowding/DowdingPlugin.cs:68-100`
fn start() {
    let login = has("Dowding_username") && has("Dowding_password") && has("Dowding_server");
    let token = has("Dowding_token") && has("Dowding_server");
    if !login && !token {
        show("Dowding invalid settings");
    }
    show("Failed to start Dowding");
}

/// The settings page, as the form shows it. `// C#: plugins/Dowding/DowdingUI.cs:39-66`
fn page() {
    let home = HOME.lock().unwrap_or_else(PoisonError::into_inner).clone();
    let enabled = if setting_true("Dowding_enabled") {
        "true"
    } else {
        "false"
    };
    host::form_show(
        "Dowding",
        &[
            control(
                "chk_enable",
                ControlKind::CheckBox,
                "Enable at Start",
                enabled,
                &[],
            ),
            control("but_token", ControlKind::Button, "", "Enter Token", &[]),
            control("txt_trackerlat", ControlKind::TextBox, "Lat", &home.0, &[]),
            control("txt_trackerlong", ControlKind::TextBox, "Lng", &home.1, &[]),
            control("txt_trackerhae", ControlKind::TextBox, "HAE", &home.2, &[]),
            control(
                "but_setathome",
                ControlKind::Button,
                "",
                "Set Tracker Home",
                &[],
            ),
        ],
    );
}

/// `but_setathome_Click`: each box a number, stored; then the send to the tracker, which is not
/// connected. `// C#: plugins/Dowding/DowdingUI.cs:401-441`
fn set_tracker_home() {
    let home = HOME.lock().unwrap_or_else(PoisonError::into_inner).clone();
    let mut values = [0.0; 3];
    for ((text, what), value) in [(&home.0, "lat"), (&home.1, "lng"), (&home.2, "hae")]
        .into_iter()
        .zip(&mut values)
    {
        let Ok(parsed) = text.trim().parse::<f64>() else {
            let _ = host::message_box(
                &format!("Invalid Location {what}"),
                "Error",
                MessageButtons::Ok,
            );
            return;
        };
        *value = parsed;
    }
    for (key, value) in [
        "Dowding_trackerlat",
        "Dowding_trackerlng",
        "Dowding_trackerhae",
    ]
    .into_iter()
    .zip(values)
    {
        host::config_set(key, &value.to_string());
    }
    let _ = host::message_box("Failed to send home location", "Error", MessageButtons::Ok);
}

struct Dowding;

impl Guest for Dowding {
    fn name() -> String {
        "Dowding".to_owned()
    }

    fn version() -> String {
        "0.1".to_owned()
    }

    fn author() -> String {
        "Michael Oborne".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        *RATE.lock().unwrap_or_else(PoisonError::into_inner)
    }

    // `// C#: plugins/Dowding/DowdingPlugin.cs:52-66`
    fn init() -> bool {
        if setting_true("Dowding_enabled") {
            start();
        }
        *RATE.lock().unwrap_or_else(PoisonError::into_inner) = 1.0;
        true
    }

    // `// C#: plugins/Dowding/DowdingPlugin.cs:102-117`
    fn loaded() -> bool {
        let main = host::menu_add(MapMenu::FlightData, None, "Dowding");
        let point = host::menu_add(MapMenu::FlightData, None, "Dowding Point At");
        *ENTRIES.lock().unwrap_or_else(PoisonError::into_inner) = Some((main, point));
        let tracker = [
            "Dowding_trackerlat",
            "Dowding_trackerlng",
            "Dowding_trackerhae",
        ]
        .map(|key| host::config_get(key).unwrap_or_default());
        let [lat, lng, hae] = tracker;
        *HOME.lock().unwrap_or_else(PoisonError::into_inner) = (lat, lng, hae);
        true
    }

    // The markers from `Dowding.Vehicles`, which stays empty without the web socket.
    // `// C#: plugins/Dowding/DowdingPlugin.cs:126-236`
    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    // `men_Click` opens the page; `men2_Click` asks the altitude for `UpdateOutput`, whose only
    // listener is the tracker page's connected tracker. `// C#: plugins/Dowding/DowdingPlugin.cs:119-124, 245-248`
    fn menu_click(id: u32, lat: f64, lng: f64) {
        let Some((main, point)) = *ENTRIES.lock().unwrap_or_else(PoisonError::into_inner) else {
            return;
        };
        if id == main {
            page();
        } else if id == point {
            let Some(alt) = host::input_box("Altitude", "Enter HAE altitude", "0") else {
                return;
            };
            let alt = alt.trim().parse::<f64>().unwrap_or(0.0);
            host::log(&format!(
                "Dowding Point At {lat} {lng} {alt}: no tracker connected"
            ));
        }
    }

    fn form_event(id: String, value: String) {
        match id.as_str() {
            // `chk_enable_CheckedChanged`: `Checked.ToString()`. `// C#: plugins/Dowding/DowdingUI.cs:122-125`
            "chk_enable" => {
                let checked = if value == "true" { "True" } else { "False" };
                host::config_set("Dowding_enabled", checked);
            }
            // `but_token_Click`. `// C#: plugins/Dowding/DowdingUI.cs:127-134`
            "but_token" => {
                if let Some(token) = host::input_box("Token", "Enter your token", "") {
                    host::config_set("Dowding_token", &token);
                }
            }
            "txt_trackerlat" => HOME.lock().unwrap_or_else(PoisonError::into_inner).0 = value,
            "txt_trackerlong" => HOME.lock().unwrap_or_else(PoisonError::into_inner).1 = value,
            "txt_trackerhae" => HOME.lock().unwrap_or_else(PoisonError::into_inner).2 = value,
            "but_setathome" => set_tracker_home(),
            _ => {}
        }
    }
}

mp_plugins::export_plugin!(Dowding);
