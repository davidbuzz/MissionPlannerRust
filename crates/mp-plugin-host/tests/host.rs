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

//! The host itself: loading a folder as `PluginLoader.LoadAll` does, each plugin's thread and
//! rate, the window's side of the channel, and the faults the C# cannot survive.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

mod common;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use common::{Scripted, load, load_with, path};
use mp_plugin_host::{
    CsValue, DialogResult, Fault, Limits, MapMenu, PluginHost, PluginState, Request, RequestBody,
    Snapshot,
};

/// A folder of its own for one test.
fn folder(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("host-{name}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Drains the host until `done` says so or five seconds pass, answering every question as the
/// window would: message boxes OK, input boxes with their value, settings from nothing.
fn pump(host: &mut PluginHost, mut done: impl FnMut(&[Request]) -> bool) -> Vec<Request> {
    let mut seen = Vec::new();
    let started = Instant::now();
    while started.elapsed() < Duration::from_secs(5) {
        for request in host.drain() {
            match request.body {
                RequestBody::MessageBox { reply, .. } => reply.send(DialogResult::Ok),
                RequestBody::ConfigGet { reply, .. } => reply.send(None),
                RequestBody::InputBox { value, reply, .. } => reply.send(Some(value)),
                RequestBody::SetMode { reply, .. } | RequestBody::SetParam { reply, .. } => {
                    reply.send(true);
                }
                body => seen.push(Request {
                    plugin: request.plugin,
                    body,
                }),
            }
        }
        if done(&seen) {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    seen
}

/// `LoadAll` over a folder: every `*.wasm` in name order, a file that is not a plugin refused
/// with its reason, a disabled one skipped, `Init` false not kept, the rest started and saying
/// who they are.
#[test]
fn load_all_loads_the_folder() {
    let (Some(fencedist), Some(example)) = (path("fencedist"), path("example")) else {
        return;
    };
    let dir = folder("load-all");
    std::fs::copy(&fencedist, dir.join("b-fencedist.wasm")).unwrap();
    std::fs::copy(&example, dir.join("a-example.WASM")).unwrap();
    std::fs::copy(&fencedist, dir.join("c-disabled.wasm")).unwrap();
    std::fs::write(dir.join("d-garbage.wasm"), b"not a plugin").unwrap();
    std::fs::write(dir.join("readme.txt"), b"not even wasm").unwrap();
    let mut host = PluginHost::load_all(&dir, &["C-Disabled.wasm".to_owned()], Limits::default());
    let files: Vec<&str> = host.plugins().iter().map(|p| p.file.as_str()).collect();
    assert_eq!(
        files,
        ["a-example.WASM", "b-fencedist.wasm", "d-garbage.wasm"]
    );
    let seen = pump(&mut host, |_| false);
    let _ = seen;
    let states: Vec<&str> = host.plugins().iter().map(|p| p.state.word()).collect();
    assert_eq!(states, ["not-loaded", "running", "not-loaded"]);
    assert_eq!(
        host.plugins()[0].state,
        PluginState::NotLoaded("Init returned false".to_owned())
    );
    let PluginState::NotLoaded(reason) = &host.plugins()[2].state else {
        panic!("{:?}", host.plugins()[2]);
    };
    assert!(reason.starts_with("not loaded:"), "{reason}");
    let info = host.plugins()[1].info.clone().unwrap();
    assert_eq!(
        (
            info.name.as_str(),
            info.version.as_str(),
            info.author.as_str()
        ),
        ("FenceDist", "0.10", "Michael Oborne")
    );
    assert_eq!(host.plugins()[1].name(), "FenceDist");
}

/// A built plugin's bytes, for the life of the test binary, as the application carries its own.
fn builtin(name: &str) -> Option<&'static [u8]> {
    Some(Box::leak(
        std::fs::read(path(name)?).ok()?.into_boxed_slice(),
    ))
}

/// The plugins the application ships load with no folder at all: the planner started anywhere,
/// with nothing beside it, still runs them (the owner's bug of 2026-10-03: a plain start loaded
/// none, because they were only ever in a test's folder).
#[test]
fn builtins_load_with_no_folder() {
    let (Some(example), Some(fencedist)) = (builtin("example"), builtin("fencedist")) else {
        return;
    };
    let builtins = [("example.wasm", example), ("fencedist.wasm", fencedist)];
    let mut host = PluginHost::load_with_builtins(&builtins, None, &[], Limits::default());
    let files: Vec<&str> = host.plugins().iter().map(|p| p.file.as_str()).collect();
    assert_eq!(files, ["example.wasm", "fencedist.wasm"]);
    let _ = pump(&mut host, |_| false);
    let states: Vec<&str> = host.plugins().iter().map(|p| p.state.word()).collect();
    // example.cs's Init returns false in the C# as here; FenceDist runs.
    assert_eq!(states, ["not-loaded", "running"]);
    assert_eq!(host.plugins()[1].name(), "FenceDist");
}

/// A file in the plugins folder with a built-in's name is loaded in its place - as replacing a
/// file in the C#'s folder does - any other file is loaded as well, and the disable list applies
/// to both, without regard to case.
#[test]
fn a_file_replaces_the_builtin_of_its_name_and_the_disable_list_applies_to_both() {
    let (Some(example), Some(fencedist), Some(menu)) =
        (builtin("example"), builtin("fencedist"), builtin("menu"))
    else {
        return;
    };
    let dir = folder("builtins");
    std::fs::copy(path("fencedist").unwrap(), dir.join("FenceDist.wasm")).unwrap();
    std::fs::copy(path("fencedist").unwrap(), dir.join("extra.wasm")).unwrap();
    std::fs::copy(path("fencedist").unwrap(), dir.join("off.wasm")).unwrap();
    let builtins = [
        ("example.wasm", example),
        ("fencedist.wasm", fencedist),
        ("menu.wasm", menu),
    ];
    let mut host = PluginHost::load_with_builtins(
        &builtins,
        Some(&dir),
        &["MENU.wasm".to_owned(), "Off.WASM".to_owned()],
        Limits::default(),
    );
    let files: Vec<&str> = host.plugins().iter().map(|p| p.file.as_str()).collect();
    // The file's FenceDist.wasm stands in for the built-in fencedist.wasm: three, not four.
    assert_eq!(files, ["example.wasm", "extra.wasm", "FenceDist.wasm"]);
    let _ = pump(&mut host, |_| false);
    let states: Vec<&str> = host.plugins().iter().map(|p| p.state.word()).collect();
    assert_eq!(states, ["not-loaded", "running", "running"]);
}

/// No folder, no plugins.
#[test]
fn no_folder_no_plugins() {
    let host = PluginHost::load_all(
        &PathBuf::from("/nonexistent/plugins"),
        &[],
        Limits::default(),
    );
    assert!(host.plugins().is_empty());
}

/// The window's side: the plugin's menu entry arrives as a request; the click goes to the
/// plugin's thread, which reads the snapshot the window set and answers with a status line.
#[test]
fn a_click_reaches_the_plugins_thread_and_the_snapshot() {
    let Some(fencedist) = path("fencedist") else {
        return;
    };
    let mut host = PluginHost::load_files(&[fencedist], Limits::default());
    let seen = pump(&mut host, |seen| {
        seen.iter()
            .any(|r| matches!(r.body, RequestBody::MenuAdd { .. }))
    });
    let entry = seen.iter().find_map(|r| match &r.body {
        RequestBody::MenuAdd { id, menu, text, .. } => Some((r.plugin, *id, *menu, text.clone())),
        _ => None,
    });
    assert_eq!(
        entry,
        Some((0, 0, MapMenu::FlightData, "Draw Fence Dist".to_owned()))
    );
    host.set_snapshot(Snapshot {
        cs: std::sync::Arc::new(|name: &str| match name {
            "lat" => Some(CsValue::Number(-35.3632)),
            "lng" => Some(CsValue::Number(149.1652)),
            _ => None,
        }),
        ..Snapshot::default()
    });
    host.menu_click(0, 0, -35.0, 149.0);
    let seen = pump(&mut host, |seen| {
        seen.iter()
            .any(|r| matches!(r.body, RequestBody::Status(_)))
    });
    let status = seen.iter().find_map(|r| match &r.body {
        RequestBody::Status(text) => Some(text.clone()),
        _ => None,
    });
    assert_eq!(status.as_deref(), Some("Fence distance 99999.0 m"));
    host.shutdown();
    assert_eq!(host.plugins()[0].state, PluginState::Running);
    let _ = host.drain();
    assert_eq!(host.plugins()[0].state, PluginState::Exited);
}

/// `Loaded` false: the plugin stays, idle - its `Loop` never runs, and closing does not call its
/// `Exit` - but its menu entries still work. `// C#: Plugin/PluginLoader.cs:323-336`
#[test]
fn loaded_false_is_idle() {
    let Some(misbehave) = path("misbehave") else {
        return;
    };
    let mut host = PluginHost::load_files(&[misbehave], Limits::default());
    let started = Instant::now();
    let mut statuses = 0;
    while started.elapsed() < Duration::from_millis(300) {
        for request in host.drain() {
            match request.body {
                RequestBody::ConfigGet { reply, .. } => reply.send(Some("idle".to_owned())),
                RequestBody::Status(_) => statuses += 1,
                _ => {}
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(host.plugins()[0].state, PluginState::Idle);
    assert_eq!(statuses, 0);
    host.menu_click(0, 0, 0.0, 0.0);
    pump(&mut host, |seen| {
        seen.iter()
            .any(|r| matches!(r.body, RequestBody::Unloaded(_)))
    });
    assert_eq!(
        host.plugins()[0].state,
        PluginState::Unloaded("the plugin panicked".to_owned())
    );
}

/// A plugin that panics in a click is unloaded, with the reason, and the host goes on.
#[test]
fn a_panic_unloads_the_plugin() {
    let Some(misbehave) = path("misbehave") else {
        return;
    };
    let mut host = PluginHost::load_files(&[misbehave], Limits::default());
    pump(&mut host, |seen| {
        seen.iter()
            .filter(|r| matches!(r.body, RequestBody::MenuAdd { .. }))
            .count()
            == 2
    });
    host.menu_click(0, 0, 0.0, 0.0);
    pump(&mut host, |seen| {
        seen.iter()
            .any(|r| matches!(r.body, RequestBody::Unloaded(_)))
    });
    assert_eq!(
        host.plugins()[0].state,
        PluginState::Unloaded("the plugin panicked".to_owned())
    );
}

/// A plugin that never returns is stopped by its fuel, within a bounded time, and unloaded.
#[test]
fn a_spin_is_stopped_by_fuel() {
    let Some(misbehave) = path("misbehave") else {
        return;
    };
    let limits = Limits {
        fuel_per_event: 200_000_000,
        ..Limits::default()
    };
    let mut host = PluginHost::load_files(&[misbehave], limits);
    pump(&mut host, |seen| {
        seen.iter()
            .filter(|r| matches!(r.body, RequestBody::MenuAdd { .. }))
            .count()
            == 2
    });
    let started = Instant::now();
    host.menu_click(0, 1, 0.0, 0.0);
    pump(&mut host, |seen| {
        seen.iter()
            .any(|r| matches!(r.body, RequestBody::Unloaded(_)))
    });
    assert!(
        started.elapsed() < Duration::from_secs(5),
        "{:?}",
        started.elapsed()
    );
    assert_eq!(
        host.plugins()[0].state,
        PluginState::Unloaded(Fault::OutOfFuel(200_000_000).to_string())
    );
}

/// A panic in `Loop` unloads it too: its third run, at its 50 Hz.
#[test]
fn a_panic_in_loop_unloads_the_plugin() {
    let Some((mut plugin, script)) = load("misbehave") else {
        return;
    };
    script.with(|r| {
        r.config.insert("misbehave".to_owned(), "loop".to_owned());
    });
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    assert!(plugin.run_loop().unwrap());
    assert!(plugin.run_loop().unwrap());
    assert_eq!(plugin.run_loop(), Err(Fault::Panicked));
    assert_eq!(script.record().status, ["loop 1", "loop 2", "loop 3"]);
}

/// The plugin thread's rule: `Loop` when `NextRun` has passed and then `1000 / loopratehz` ms
/// on - 20 ms at 50 Hz - not every tick. `// C#: MainV2.cs:2524-2540`
#[test]
fn loop_runs_at_its_rate() {
    let Some((mut plugin, script)) = load("misbehave") else {
        return;
    };
    plugin.init().unwrap();
    plugin.loaded().unwrap();
    assert_eq!(plugin.loop_rate_hz(), 50.0);
    let start = Instant::now();
    assert!(plugin.tick(start).unwrap());
    assert!(!plugin.tick(start + Duration::from_millis(10)).unwrap());
    assert!(!plugin.tick(start + Duration::from_millis(20)).unwrap());
    assert!(plugin.tick(start + Duration::from_millis(21)).unwrap());
    assert_eq!(script.record().status.len(), 2);
}

/// On the host's thread the rate is kept by the clock: a quarter of a second at 50 Hz is about
/// a dozen loops, and closing runs `Exit`.
///
/// The quarter second is counted from the first loop, not from the load: the thread compiles
/// the component first, which took over a quarter of a second on a loaded machine and left this
/// test with no loops at all (the flake of 2026-09-26). The rate is the thing under test; the
/// compile is not.
#[test]
fn the_thread_loops_at_the_rate() {
    let Some(misbehave) = path("misbehave") else {
        return;
    };
    let mut host = PluginHost::load_files(&[misbehave], Limits::default());
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut started = None;
    let mut loops = 0;
    loop {
        for request in host.drain() {
            match request.body {
                RequestBody::Status(_) => {
                    if started.is_none() {
                        started = Some(Instant::now());
                    } else {
                        loops += 1;
                    }
                }
                RequestBody::ConfigGet { reply, .. } => reply.send(None),
                _ => {}
            }
        }
        match started {
            Some(at) if at.elapsed() >= Duration::from_millis(250) => break,
            None => assert!(Instant::now() < deadline, "the plugin never looped"),
            Some(_) => {}
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    // Never faster than its rate: 16 at most. As fast as the rate where the machine wakes a
    // sleeping thread on time: 8 at least; on one that does not, it still loops, and the rate is
    // not judged - the hosted macOS runner managed 3 in the quarter second (2026-10-04, run
    // 37167018667), and 1 when its timed waits woke 50 ms late (run 37172083775).
    let lateness = (0..3)
        .map(|_| {
            let asked = Duration::from_millis(5);
            let slept = Instant::now();
            std::thread::sleep(asked);
            slept.elapsed().saturating_sub(asked)
        })
        .max()
        .unwrap_or(Duration::ZERO);
    let least = if lateness <= Duration::from_millis(2) { 8 } else { 1 };
    assert!(
        (least..=16).contains(&loops),
        "{loops} loops in 250 ms, a timed wait here waking {lateness:?} late"
    );
    host.shutdown();
    let logs: Vec<String> = host
        .drain()
        .into_iter()
        .filter_map(|r| match r.body {
            RequestBody::Log(line) => Some(line),
            _ => None,
        })
        .collect();
    assert_eq!(logs, ["Misbehave exit"]);
    assert_eq!(host.plugins()[0].state, PluginState::Exited);
}

/// A plugin that grows its memory past the limit traps rather than taking the machine's.
#[test]
fn memory_is_limited() {
    let Some(misbehave) = path("misbehave") else {
        return;
    };
    let bytes = std::fs::read(misbehave).unwrap();
    let engine = mp_plugin_host::engine().unwrap();
    // Too small even to instantiate: refused at load.
    let refused = mp_plugin_host::Plugin::load(
        &engine,
        &bytes,
        "misbehave.wasm",
        Box::new(Scripted::new()),
        Limits {
            memory: 64 * 1024,
            ..Limits::default()
        },
    );
    assert!(matches!(refused, Err(Fault::Load(_))), "{refused:?}");
    // And the default is enough.
    assert!(load_with("misbehave", Scripted::new(), Limits::default()).is_some());
}

/// The plugins folder is `plugins` beside the executable.
#[test]
fn the_plugins_folder_is_beside_the_executable() {
    let exe = std::env::current_exe().unwrap();
    assert_eq!(
        mp_plugin_host::plugins_dir(),
        Some(exe.parent().unwrap().join("plugins"))
    );
}
