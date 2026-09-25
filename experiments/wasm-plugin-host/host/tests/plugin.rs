//! The experiment's claims, each a test against the built plugin.

use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant};

use wasm_plugin_host::{Host, Plugin};

/// The plugin, built for wasm32-unknown-unknown by `run.sh`, or here when it is missing; `None`,
/// and the test says so and passes, on a machine without that target.
fn plugin_path() -> Option<PathBuf> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..");
    // Where cargo put it: `CARGO_TARGET_DIR` when set, else the workspace's `target`.
    let target =
        std::env::var_os("CARGO_TARGET_DIR").map_or_else(|| root.join("target"), PathBuf::from);
    let path = target.join("wasm32-unknown-unknown/release/fencedist.wasm");
    if path.exists() {
        return Some(path);
    }
    let installed = Command::new("rustup")
        .args(["target", "list", "--installed"])
        .output()
        .ok()
        .is_some_and(|out| String::from_utf8_lossy(&out.stdout).contains("wasm32-unknown-unknown"));
    if !installed {
        eprintln!("SKIP: rustup target add wasm32-unknown-unknown");
        return None;
    }
    let status = Command::new("cargo")
        .args([
            "build",
            "--release",
            "-p",
            "fencedist",
            "--target",
            "wasm32-unknown-unknown",
        ])
        .current_dir(&root)
        .status()
        .expect("cargo runs");
    assert!(status.success(), "the plugin builds");
    Some(path)
}

fn canberra() -> Host {
    Host {
        cs: vec![("lat".to_owned(), -35.3632), ("lng".to_owned(), 149.1652)],
        // A rectangle around the vehicle: the nearest edge is the southern one, 0.0028 degrees
        // of latitude away - about 311 m; the northern is 356 m, the sides about 435 m.
        fence: vec![
            (-35.360, 149.160),
            (-35.360, 149.170),
            (-35.366, 149.170),
            (-35.366, 149.160),
        ],
        ..Host::default()
    }
}

/// `PluginLoader.Load` then `Init` and `Loaded`: the plugin says who it is, `Loaded` adds its
/// menu entry to the flight screen's map menu, and the log carries its lines.
#[test]
fn a_plugin_loads_and_adds_its_menu_entry() {
    let Some(path) = plugin_path() else { return };
    let mut plugin = Plugin::load(&path, canberra()).expect("loads");
    assert_eq!(plugin.name, "FenceDist");
    assert_eq!(plugin.version, "0.10");
    assert_eq!(plugin.author, "Michael Oborne");
    assert_eq!(plugin.loop_rate_hz, 2.0);
    assert!(plugin.init().expect("init"));
    assert!(plugin.host().menu.is_empty());
    assert!(plugin.loaded().expect("loaded"));
    assert_eq!(plugin.host().menu, ["Draw Fence Dist"]);
    assert_eq!(plugin.host().log, ["FenceDist init", "FenceDist loaded"]);
    assert!(plugin.exit().expect("exit"));
    assert_eq!(
        plugin.host().log.last().map(String::as_str),
        Some("FenceDist exit")
    );
}

/// `Loop` reads `Host.cs` and the fence and puts the distance on the status line: the vehicle
/// 311 m north of the southern edge.
#[test]
fn loop_measures_the_distance_to_the_fence() {
    let Some(path) = plugin_path() else { return };
    let mut plugin = Plugin::load(&path, canberra()).expect("loads");
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    assert!(plugin.run_loop().expect("loop"));
    let distance = plugin.call_f64("plugin_last_distance").unwrap();
    assert!((distance - 311.3).abs() < 2.0, "{distance}");
    assert_eq!(plugin.host().status, format!("fence {distance:.1} m"));
    // The vehicle moved north: the next Loop sees the new position through the same host, and
    // the northern edge is now 56 m away.
    plugin.host_mut().cs[0].1 = -35.3605;
    plugin.run_loop().unwrap();
    let nearer = plugin.call_f64("plugin_last_distance").unwrap();
    assert!((nearer - 55.6).abs() < 2.0, "{nearer}");
    // No fence: the example's 99999.
    plugin.host_mut().fence.clear();
    plugin.run_loop().unwrap();
    assert_eq!(plugin.call_f64("plugin_last_distance").unwrap(), 99999.0);
    // No position: NaN, and the status line left alone.
    plugin.host_mut().cs.clear();
    plugin.run_loop().unwrap();
    assert!(plugin.call_f64("plugin_last_distance").unwrap().is_nan());
}

/// The plugin thread's rule: `Loop` runs at `loopratehz`, not every tick.
/// `// C#: MainV2.cs:2524-2540`
#[test]
fn loop_runs_at_its_rate() {
    let Some(path) = plugin_path() else { return };
    let mut plugin = Plugin::load(&path, canberra()).expect("loads");
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    let start = Instant::now() + Duration::from_millis(1);
    assert!(plugin.tick(start).unwrap());
    assert!(!plugin.tick(start + Duration::from_millis(100)).unwrap());
    assert!(!plugin.tick(start + Duration::from_millis(499)).unwrap());
    assert!(plugin.tick(start + Duration::from_millis(501)).unwrap());
    assert_eq!(plugin.call_u64("plugin_loops").unwrap(), 2);
}

/// The menu entry chosen: the example's message box, then the measurement; another plugin's
/// entry is not this plugin's.
#[test]
fn the_menu_entry_shows_the_message_and_measures() {
    let Some(path) = plugin_path() else { return };
    let mut plugin = Plugin::load(&path, canberra()).expect("loads");
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    assert!(!plugin.menu_click(7).unwrap());
    assert!(plugin.host().messages.is_empty());
    assert!(plugin.menu_click(0).unwrap());
    assert_eq!(
        plugin.host().messages,
        ["This is a sample plugin\nSee the source in the plugins folder"]
    );
    assert!(
        plugin.host().status.starts_with("fence 31"),
        "{}",
        plugin.host().status
    );
}

/// A bug in a plugin - a panic - is an error the host reports; the host and the plugin's state
/// survive, and the next call works.
#[test]
fn a_panicking_plugin_is_an_error_not_a_crash() {
    let Some(path) = plugin_path() else { return };
    let mut plugin = Plugin::load(&path, canberra()).expect("loads");
    plugin.init().unwrap();
    let err = plugin
        .call_i32("plugin_explode")
        .expect_err("the panic is reported");
    assert_eq!(err.to_string(), "the plugin panicked");
    assert!(plugin.loaded().expect("the host goes on"));
    assert_eq!(plugin.host().menu, ["Draw Fence Dist"]);
}

/// A plugin that never returns is stopped by its fuel, and the host goes on.
#[test]
fn a_runaway_plugin_is_stopped() {
    let Some(path) = plugin_path() else { return };
    let mut plugin = Plugin::load(&path, canberra()).expect("loads");
    let started = Instant::now();
    let err = plugin.call_i32("plugin_spin").expect_err("stopped");
    assert!(
        err.to_string().starts_with("the plugin did not return"),
        "{err}"
    );
    assert!(
        started.elapsed() < Duration::from_secs(5),
        "{:?}",
        started.elapsed()
    );
    assert!(plugin.init().expect("the host goes on"));
}

/// A module that is not a plugin - no exports - is refused at load, with a reason.
#[test]
fn a_module_without_the_exports_is_refused() {
    let dir = std::env::temp_dir().join(format!("wasm-plugin-host-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let empty = dir.join("empty.wasm");
    // The smallest module: the magic and the version.
    std::fs::write(&empty, [0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00]).unwrap();
    let Err(err) = Plugin::load(&empty, Host::default()) else {
        panic!("an empty module loaded as a plugin");
    };
    assert!(err.to_string().contains("memory"), "{err}");
    let _ = std::fs::remove_dir_all(dir);
}
