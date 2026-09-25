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
