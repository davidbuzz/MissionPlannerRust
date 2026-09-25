//! `Plugins/example21-persistentsimple.cs`, "Persistent Simple Actions": three mode buttons -
//! Auto, Loiter, RTL - in the flight screen's persistent panel; each sets its mode.
//!
//! As the C# ships it, `Init` returns false ("CHANGE THIS TO TRUE TO USE THIS PLUGIN"); the port
//! keeps that. The buttons are the plugin's form, drawn by the host, since a plugin cannot add a
//! control to the flight screen's panel.

use mp_plugins::host::{self, ControlKind, MessageButtons};
use mp_plugins::{Guest, control};

/// The buttons' texts, each a mode's name. `// C#: Plugins/example21-persistentsimple.cs:29-32`
const MODES: [&str; 3] = ["Auto", "Loiter", "RTL"];

struct PersistentSimpleActions;

impl Guest for PersistentSimpleActions {
    fn name() -> String {
        "Persistent Simple Actions".to_owned()
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

    // `// C#: Plugins/example21-persistentsimple.cs:19-20`
    fn init() -> bool {
        false
    }

    // `// C#: Plugins/example21-persistentsimple.cs:22-66`
    fn loaded() -> bool {
        let buttons: Vec<host::Control> = MODES
            .iter()
            .map(|mode| {
                control(
                    mode,
                    ControlKind::Button,
                    &format!("Change mode to {mode}"),
                    mode,
                    &[],
                )
            })
            .collect();
        host::form_show("Persistent Simple Actions", &buttons);
        true
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    fn menu_click(_id: u32, _lat: f64, _lng: f64) {}

    // `but_mode_Click`: the button's text is the mode. `// C#: Plugins/example21-persistentsimple.cs:70-82`
    fn form_event(id: String, _value: String) {
        if !MODES.contains(&id.as_str()) {
            return;
        }
        if !host::set_mode(&id) {
            let _ = host::message_box(
                "The Command failed to execute\n",
                "Error",
                MessageButtons::Ok,
            );
        }
    }
}

mp_plugins::export_plugin!(PersistentSimpleActions);
