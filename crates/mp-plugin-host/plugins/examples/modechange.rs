//! `plugins/example8-modechange.cs`, "Mode Change Widget": a combo box of the vehicle's flight
//! modes on the main menu, kept to the vehicle's mode, and choosing one sets it.
//!
//! As the C# ships it, `Init` returns false, so the loader never keeps it, and `loopratehz`
//! starts at zero and is only set inside `Loop`, so its `Loop` would not run either; the port
//! keeps both. The combo box is the plugin's form, drawn by the host, since a plugin cannot add
//! a control to the main menu.

use std::sync::{Mutex, PoisonError};

use mp_plugins::host::{self, ControlKind, MessageButtons};
use mp_plugins::{Guest, control, cs_text};

/// The plugin's fields. `// C#: plugins/example8-modechange.cs:17-21`
struct State {
    /// `hashcode`: the connection the modes were read for; here the firmware's name.
    connection: Option<String>,
    /// `modecmb.Items`.
    modes: Vec<String>,
    /// `currentmode`.
    current: String,
    /// `modecmb.Enabled`.
    enabled: bool,
    /// `loopratehz`.
    rate: f32,
}

static STATE: Mutex<State> = Mutex::new(State {
    connection: None,
    modes: Vec::new(),
    current: String::new(),
    enabled: false,
    rate: 0.0,
});

/// The combo box, as the form shows it.
fn show(state: &State) {
    let options: Vec<&str> = state.modes.iter().map(String::as_str).collect();
    let label = if state.enabled {
        "Mode"
    } else {
        "Mode (not connected)"
    };
    host::form_show(
        "Mode Change Widget",
        &[control(
            "mode",
            ControlKind::ComboBox,
            label,
            &state.current,
            &options,
        )],
    );
}

struct ModeChange;

impl Guest for ModeChange {
    fn name() -> String {
        "Mode Change Widget".to_owned()
    }

    fn version() -> String {
        "0.1".to_owned()
    }

    fn author() -> String {
        "Michael Oborne".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        STATE.lock().unwrap_or_else(PoisonError::into_inner).rate
    }

    // `// C#: plugins/example8-modechange.cs:38-41`
    fn init() -> bool {
        false
    }

    // `// C#: plugins/example8-modechange.cs:43-50`
    fn loaded() -> bool {
        show(&STATE.lock().unwrap_or_else(PoisonError::into_inner));
        true
    }

    // `// C#: plugins/example8-modechange.cs:75-117, 119-130`
    fn run_loop() -> bool {
        let mut state = STATE.lock().unwrap_or_else(PoisonError::into_inner);
        let connected = matches!(host::cs("connected"), Some(host::CsValue::Flag(true)));
        if connected {
            // A new connection: `ComPort_MavChanged`, the modes FLTMODE1's documentation lists
            // for the firmware.
            let firmware = cs_text("firmware");
            if state.connection != firmware {
                state.connection = firmware;
                state.modes = host::param_options("FLTMODE1")
                    .into_iter()
                    .map(|(_, name)| name)
                    .collect();
                state.enabled = true;
            }
            // Kept to the vehicle's mode, without sending it back (`setwithnosend`).
            let mode = cs_text("mode").unwrap_or_default();
            if mode != state.current {
                state.current = mode;
                state.enabled = true;
            }
        } else if state.enabled {
            state.enabled = false;
        }
        show(&state);
        state.rate = 0.3;
        true
    }

    fn exit() -> bool {
        true
    }

    fn menu_click(_id: u32, _lat: f64, _lng: f64) {}

    // `Modecmb_SelectedValueChanged`: the mode chosen is set; a refusal is the C#'s box.
    // `// C#: plugins/example8-modechange.cs:52-73`
    fn form_event(id: String, value: String) {
        if id != "mode" {
            return;
        }
        {
            let mut state = STATE.lock().unwrap_or_else(PoisonError::into_inner);
            // A disabled combo box cannot be changed.
            if !state.enabled {
                return;
            }
            state.current.clone_from(&value);
        }
        if !host::set_mode(&value) {
            let _ = host::message_box("Error: no response from MAV", "Error", MessageButtons::Ok);
        }
    }
}

mp_plugins::export_plugin!(ModeChange);
