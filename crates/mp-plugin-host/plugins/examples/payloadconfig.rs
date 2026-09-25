//! `Plugins/example22-payloadconfig.cs`, "Payload Select Page": a page of check boxes, one per
//! payload; checking one writes its parameters, unchecking reverts them to their defaults, and
//! nothing may change while the vehicle is armed.
//!
//! The C# ships its payload table empty, with a gimbal and a mapping camera commented out as the
//! way to fill it, and an empty table does not load (`Init` is `payloadParameters.Count > 0`).
//! The port fills the table with those two, as the file tells its user to.
//!
//! Left out: the revert to defaults - the world has no parameter defaults, so an unchecked
//! payload's parameters take the C#'s "No default value found" path - and `CheckParamCountChanged`
//! (`MAV.param.TotalReceived` against `TotalReported`, and the list read again). The page is the
//! plugin's form, drawn by the host, where the C# adds a page to the Config tab.

use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, ControlKind};
use mp_plugins::{Guest, control, cs_number};

/// The payloads and their parameters. `// C#: Plugins/example22-payloadconfig.cs:41-56`
const PAYLOADS: &[(&str, &[(&str, f64)])] = &[
    ("Gimbal", &[("MNT1_TYPE", 9.0)]),
    (
        "Mapping Camera",
        &[("CAM_TRIGG_TYPE", 1.0), ("SERVO9_FUNCTION", 10.0)],
    ),
];

/// Each payload's box, checked or not.
static CHECKED: Mutex<Vec<bool>> = Mutex::new(Vec::new());

/// `ParamEquality`: equal to a millionth. `// C#: Plugins/example22-payloadconfig.cs:260-263`
fn equal(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6_f64.max(a.abs().max(b.abs()) * 1e-6)
}

/// `IsPayloadActive`: every parameter the payload names is at its value.
/// `// C#: Plugins/example22-payloadconfig.cs:150-168`
fn active(parameters: &[(&str, f64)]) -> bool {
    parameters
        .iter()
        .all(|(name, value)| match host::get_param(name) {
            Some(current) => equal(current, *value),
            None => {
                host::log(&format!("Parameter {name} not found for comparison."));
                false
            }
        })
}

/// `CheckArmed`: an armed vehicle's page is disabled, and the user told.
/// `// C#: Plugins/example22-payloadconfig.cs:120-131`
fn armed() -> bool {
    let armed = cs_number("armed").is_some_and(|armed| armed != 0.0);
    if armed {
        let _ = host::message_box(
            "The vehicle is armed. Payload selection is disabled.",
            "Payload Selection",
            host::MessageButtons::Ok,
        );
    }
    armed
}

/// The page, as the form shows it.
fn show(checked: &[bool]) {
    let controls: Vec<host::Control> = PAYLOADS
        .iter()
        .zip(checked)
        .map(|((name, _), on)| {
            control(
                name,
                ControlKind::CheckBox,
                name,
                if *on { "true" } else { "false" },
                &[],
            )
        })
        .collect();
    host::form_show("Payload Selection", &controls);
}

struct PayloadConfig;

impl Guest for PayloadConfig {
    fn name() -> String {
        "Payload Select Page".to_owned()
    }

    fn version() -> String {
        "0.1".to_owned()
    }

    fn author() -> String {
        "Bob Long".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        0.0
    }

    // `// C#: Plugins/example22-payloadconfig.cs:60-64`
    fn init() -> bool {
        !PAYLOADS.is_empty()
    }

    // The page registered, and `Activate`: the boxes from the parameters, then the armed check.
    // `// C#: Plugins/example22-payloadconfig.cs:66-72, 111-117, 133-146`
    fn loaded() -> bool {
        let checked: Vec<bool> = PAYLOADS
            .iter()
            .map(|(_, parameters)| active(parameters))
            .collect();
        show(&checked);
        *CHECKED.lock().unwrap_or_else(PoisonError::into_inner) = checked;
        let _ = armed();
        true
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    fn menu_click(_id: u32, _lat: f64, _lng: f64) {}

    // `PayloadCheckboxList_ItemCheck`. `// C#: Plugins/example22-payloadconfig.cs:170-189`
    fn form_event(id: String, value: String) {
        let Some(index) = PAYLOADS.iter().position(|(name, _)| *name == id) else {
            return;
        };
        let mut checked = CHECKED
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone();
        // The disabled list cannot be changed: the box goes back.
        if armed() {
            show(&checked);
            return;
        }
        let on = value == "true";
        let Some((_, parameters)) = PAYLOADS.get(index) else {
            return;
        };
        if on {
            // `ApplyPayloadParameters`. `// C#: Plugins/example22-payloadconfig.cs:191-210`
            for (name, value) in *parameters {
                let _ = host::set_param(name, *value);
            }
        } else {
            // `RevertParametersToDefault`, which finds no default here.
            // `// C#: Plugins/example22-payloadconfig.cs:235-258`
            for (name, _) in *parameters {
                if host::get_param(name).is_none() {
                    host::log(&format!(
                        "Parameter {name} missing during revert to default."
                    ));
                } else {
                    host::log(&format!("No default value found for parameter {name}"));
                }
            }
        }
        if let Some(slot) = checked.get_mut(index) {
            *slot = on;
        }
        show(&checked);
        *CHECKED.lock().unwrap_or_else(PoisonError::into_inner) = checked;
    }
}

mp_plugins::export_plugin!(PayloadConfig);
