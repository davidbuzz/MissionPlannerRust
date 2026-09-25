//! Not a port: the host's own test of the two faults the C# cannot survive, carried over from the
//! experiment's `explode` and `spin` (`experiments/wasm-plugin-host`). "Panic" on the flight
//! screen's map menu panics; "Spin" never returns; with the setting `misbehave` at `loop`, `Loop`
//! panics on its third run, and at `idle`, `Loaded` says no.

use std::sync::atomic::{AtomicU32, Ordering};

use mp_plugins::Guest;
use mp_plugins::host::{self, MapMenu};

/// `Loop`'s runs so far.
static LOOPS: AtomicU32 = AtomicU32::new(0);
/// Whether `Loop` panics on its third run.
static LOOP_PANICS: AtomicU32 = AtomicU32::new(0);

struct Misbehave;

impl Guest for Misbehave {
    fn name() -> String {
        "Misbehave".to_owned()
    }

    fn version() -> String {
        "1.0".to_owned()
    }

    fn author() -> String {
        "the host's tests".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        50.0
    }

    fn init() -> bool {
        true
    }

    fn loaded() -> bool {
        let _ = host::menu_add(MapMenu::FlightData, None, "Panic");
        let _ = host::menu_add(MapMenu::FlightData, None, "Spin");
        match host::config_get("misbehave").as_deref() {
            Some("loop") => LOOP_PANICS.store(1, Ordering::Relaxed),
            // `Loaded` false: kept, but never looped.
            Some("idle") => return false,
            _ => {}
        }
        true
    }

    fn run_loop() -> bool {
        let loops = LOOPS.fetch_add(1, Ordering::Relaxed) + 1;
        host::status(&format!("loop {loops}"));
        assert!(
            !(LOOP_PANICS.load(Ordering::Relaxed) == 1 && loops == 3),
            "the third loop"
        );
        true
    }

    fn exit() -> bool {
        host::log("Misbehave exit");
        true
    }

    fn menu_click(id: u32, _lat: f64, _lng: f64) {
        match id {
            0 => panic!("asked to"),
            1 => {
                let mut count: u64 = 0;
                loop {
                    count = std::hint::black_box(count.wrapping_add(1));
                }
            }
            _ => {}
        }
    }

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(Misbehave);
