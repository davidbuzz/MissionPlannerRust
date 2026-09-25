//! The measurements: how long a plugin takes to load, what a `Loop` call costs, and how much
//! memory the instance holds. `run.sh` prints them after the tests.

use std::path::PathBuf;
use std::time::Instant;

use wasm_plugin_host::{Host, Plugin};

fn plugin_path() -> PathBuf {
    std::env::args().nth(1).map_or_else(
        || {
            let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..");
            std::env::var_os("CARGO_TARGET_DIR")
                .map_or_else(|| root.join("target"), PathBuf::from)
                .join("wasm32-unknown-unknown/release/fencedist.wasm")
        },
        PathBuf::from,
    )
}

fn host() -> Host {
    Host {
        cs: vec![("lat".to_owned(), -35.3632), ("lng".to_owned(), 149.1652)],
        fence: vec![
            (-35.360, 149.160),
            (-35.360, 149.170),
            (-35.366, 149.170),
            (-35.366, 149.160),
        ],
        ..Host::default()
    }
}

fn main() -> anyhow::Result<()> {
    let path = plugin_path();
    let bytes = std::fs::metadata(&path)?.len();
    println!("plugin: {} ({bytes} bytes)", path.display());

    // Load: compile and instantiate, five times, the first cold.
    let mut loads = Vec::new();
    let mut plugin = None;
    for _ in 0..5 {
        let loaded = Plugin::load(&path, host())?;
        loads.push(loaded.load_time);
        plugin = Some(loaded);
    }
    let mut plugin = plugin.expect("loaded");
    println!(
        "load (compile + instantiate): first {:.1} ms, then {:?}",
        loads[0].as_secs_f64() * 1000.0,
        loads[1..]
            .iter()
            .map(|d| format!("{:.1} ms", d.as_secs_f64() * 1000.0))
            .collect::<Vec<_>>()
    );
    println!(
        "plugin: {} {} by {}, loopratehz {}",
        plugin.name, plugin.version, plugin.author, plugin.loop_rate_hz
    );
    assert!(plugin.init()?);
    assert!(plugin.loaded()?);
    println!("memory: {} bytes", plugin.memory_bytes());

    // Loop: two telemetry reads, eight fence reads and a status line per call.
    let calls = 100_000;
    let started = Instant::now();
    for _ in 0..calls {
        plugin.run_loop()?;
    }
    let per_call = started.elapsed() / calls;
    println!(
        "loop: {calls} calls in {:.1} ms, {per_call:?} per call, status {:?}",
        started.elapsed().as_secs_f64() * 1000.0,
        plugin.host().status
    );

    // A host call alone: the cheapest export, a hundred thousand times.
    let started = Instant::now();
    for _ in 0..calls {
        plugin.call_u64("plugin_loops")?;
    }
    println!("bare call: {:?} per call", started.elapsed() / calls);

    plugin.exit()?;
    Ok(())
}
