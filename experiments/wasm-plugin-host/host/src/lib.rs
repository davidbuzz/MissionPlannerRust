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

//! The application's side of a WebAssembly plugin: what `PluginLoader` and `MainV2`'s plugin
//! thread do for a `.dll`, done for a `.wasm` under wasmtime.
//!
//! * `Plugin::load` is `PluginLoader.Load`: the module compiled and instantiated with the host's
//!   imports bound (`Plugin/PluginLoader.cs:100-190`);
//! * `init`, `loaded`, `exit` are the lifecycle the loader and `MainV2` call
//!   (`PluginLoader.cs:175, 327; MainV2.cs:2561`);
//! * [`Plugin::tick`] is the plugin thread's rule: `Loop` runs when `NextRun` has passed and
//!   `loopratehz > 0`, and `NextRun` moves on by `1000 / loopratehz` ms (`MainV2.cs:2524-2540`);
//! * [`Host`] is `PluginHost` (`Plugin/Plugin.cs:44-180`), the small part of it this experiment
//!   needs: `cs`, a map menu, `CustomMessageBox.Show`, the fence points.
//!
//! What the C# cannot do and this can: a plugin that panics is an error the host reports, not a
//! crash of the application; a plugin that never returns is stopped by fuel; a plugin cannot
//! open a file, a socket or a clock the host did not hand it.

use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow};
use wasmtime::{Caller, Config, Engine, Instance, Linker, Memory, Module, Store, Trap, TypedFunc};

/// `PluginHost`, as far as the experiment's plugin reaches into it.
#[derive(Debug, Default)]
pub struct Host {
    /// `Host.cs`: telemetry by field name.
    pub cs: Vec<(String, f64)>,
    /// `MAV.fencepoints`, less the return point: vertices in degrees.
    pub fence: Vec<(f64, f64)>,
    /// `Host.FDMenuMap.Items`: the entries plugins added, by id.
    pub menu: Vec<String>,
    /// `CustomMessageBox.Show`'s texts, in order.
    pub messages: Vec<String>,
    /// The status line.
    pub status: String,
    /// `log.Info`.
    pub log: Vec<String>,
}

impl Host {
    fn cs_get(&self, name: &str) -> f64 {
        self.cs
            .iter()
            .find(|(field, _)| field == name)
            .map_or(f64::NAN, |(_, value)| *value)
    }
}

/// How much a plugin may compute in one call before it is stopped: about a hundred million
/// instructions, well over any `Loop` and well under forever.
pub const FUEL_PER_CALL: u64 = 100_000_000;

/// A loaded plugin: `Plugin` with its `Host`.
pub struct Plugin {
    store: Store<Host>,
    memory: Memory,
    init: TypedFunc<(), i32>,
    loaded: TypedFunc<(), i32>,
    loop_: TypedFunc<(), i32>,
    exit: TypedFunc<(), i32>,
    menu_click: TypedFunc<i32, i32>,
    /// `Name`, `Version`, `Author`, read once.
    pub name: String,
    pub version: String,
    pub author: String,
    /// `loopratehz`.
    pub loop_rate_hz: f32,
    /// `NextRun`.
    next_run: Instant,
    instance: Instance,
    /// How long compiling and instantiating took.
    pub load_time: Duration,
}

fn read_str(memory: &Memory, store: &impl wasmtime::AsContext, ptr: i32, len: i32) -> String {
    let data = memory.data(store);
    let start = usize::try_from(ptr).unwrap_or(0);
    let end = start.saturating_add(usize::try_from(len).unwrap_or(0));
    data.get(start..end)
        .map(|bytes| String::from_utf8_lossy(bytes).into_owned())
        .unwrap_or_default()
}

fn memory_of(caller: &mut Caller<'_, Host>) -> Option<Memory> {
    caller
        .get_export("memory")
        .and_then(|export| export.into_memory())
}

fn string_arg(caller: &mut Caller<'_, Host>, ptr: i32, len: i32) -> String {
    match memory_of(caller) {
        Some(memory) => read_str(&memory, caller, ptr, len),
        None => String::new(),
    }
}

impl Plugin {
    /// `PluginLoader.Load`: the module at `path` compiled, its imports bound to `host`, its
    /// `Name`, `Version` and `Author` read.
    pub fn load(path: &Path, host: Host) -> Result<Self> {
        let started = Instant::now();
        let mut config = Config::new();
        config.consume_fuel(true);
        let engine = Engine::new(&config)?;
        let module = Module::from_file(&engine, path).map_err(|err| {
            anyhow::Error::from(err).context(format!("compiling {}", path.display()))
        })?;
        let mut linker = Linker::new(&engine);
        linker.func_wrap(
            "env",
            "host_log",
            |mut caller: Caller<'_, Host>, ptr: i32, len: i32| {
                let text = string_arg(&mut caller, ptr, len);
                caller.data_mut().log.push(text);
            },
        )?;
        linker.func_wrap(
            "env",
            "host_cs_get",
            |mut caller: Caller<'_, Host>, ptr: i32, len: i32| -> f64 {
                let name = string_arg(&mut caller, ptr, len);
                caller.data().cs_get(&name)
            },
        )?;
        linker.func_wrap(
            "env",
            "host_menu_add",
            |mut caller: Caller<'_, Host>, ptr: i32, len: i32| -> i32 {
                let text = string_arg(&mut caller, ptr, len);
                let menu = &mut caller.data_mut().menu;
                menu.push(text);
                i32::try_from(menu.len()).unwrap_or(i32::MAX) - 1
            },
        )?;
        linker.func_wrap(
            "env",
            "host_message",
            |mut caller: Caller<'_, Host>, ptr: i32, len: i32| {
                let text = string_arg(&mut caller, ptr, len);
                caller.data_mut().messages.push(text);
            },
        )?;
        linker.func_wrap(
            "env",
            "host_status",
            |mut caller: Caller<'_, Host>, ptr: i32, len: i32| {
                let text = string_arg(&mut caller, ptr, len);
                caller.data_mut().status = text;
            },
        )?;
        linker.func_wrap(
            "env",
            "host_fence_count",
            |caller: Caller<'_, Host>| -> i32 {
                i32::try_from(caller.data().fence.len()).unwrap_or(i32::MAX)
            },
        )?;
        linker.func_wrap(
            "env",
            "host_fence_lat",
            |caller: Caller<'_, Host>, index: i32| -> f64 {
                usize::try_from(index)
                    .ok()
                    .and_then(|index| caller.data().fence.get(index))
                    .map_or(f64::NAN, |vertex| vertex.0)
            },
        )?;
        linker.func_wrap(
            "env",
            "host_fence_lng",
            |caller: Caller<'_, Host>, index: i32| -> f64 {
                usize::try_from(index)
                    .ok()
                    .and_then(|index| caller.data().fence.get(index))
                    .map_or(f64::NAN, |vertex| vertex.1)
            },
        )?;
        let mut store = Store::new(&engine, host);
        store.set_fuel(FUEL_PER_CALL)?;
        let instance = linker.instantiate(&mut store, &module)?;
        let memory = instance
            .get_memory(&mut store, "memory")
            .ok_or_else(|| anyhow!("the plugin exports no memory"))?;
        let text = |store: &mut Store<Host>, what: &str| -> Result<String> {
            let ptr =
                instance.get_typed_func::<(), i32>(&mut *store, &format!("plugin_{what}_ptr"))?;
            let len =
                instance.get_typed_func::<(), i32>(&mut *store, &format!("plugin_{what}_len"))?;
            let ptr = ptr.call(&mut *store, ())?;
            let len = len.call(&mut *store, ())?;
            Ok(read_str(&memory, store, ptr, len))
        };
        let name = text(&mut store, "name")?;
        let version = text(&mut store, "version")?;
        let author = text(&mut store, "author")?;
        let loop_rate_hz = instance
            .get_typed_func::<(), f32>(&mut store, "plugin_loop_rate_hz")?
            .call(&mut store, ())?;
        let init = instance.get_typed_func::<(), i32>(&mut store, "plugin_init")?;
        let loaded = instance.get_typed_func::<(), i32>(&mut store, "plugin_loaded")?;
        let loop_ = instance.get_typed_func::<(), i32>(&mut store, "plugin_loop")?;
        let exit = instance.get_typed_func::<(), i32>(&mut store, "plugin_exit")?;
        let menu_click = instance.get_typed_func::<i32, i32>(&mut store, "plugin_menu_click")?;
        Ok(Self {
            store,
            memory,
            init,
            loaded,
            loop_,
            exit,
            menu_click,
            name,
            version,
            author,
            loop_rate_hz,
            next_run: Instant::now(),
            instance,
            load_time: started.elapsed(),
        })
    }

    /// The host, as the plugin has changed it.
    pub fn host(&self) -> &Host {
        self.store.data()
    }

    /// The host, to change what the plugin will see.
    pub fn host_mut(&mut self) -> &mut Host {
        self.store.data_mut()
    }

    /// One call into the plugin, with a fresh tank of fuel: a plugin that panics or runs out is
    /// an error here, and the host goes on.
    fn call<R: wasmtime::WasmResults>(&mut self, function: &TypedFunc<(), R>) -> Result<R> {
        self.store.set_fuel(FUEL_PER_CALL)?;
        function.call(&mut self.store, ()).map_err(describe)
    }

    /// `Init`: true keeps the plugin.
    pub fn init(&mut self) -> Result<bool> {
        let init = self.init.clone();
        Ok(self.call(&init)? != 0)
    }

    /// `Loaded`.
    pub fn loaded(&mut self) -> Result<bool> {
        let loaded = self.loaded.clone();
        Ok(self.call(&loaded)? != 0)
    }

    /// `Loop`, now, whatever the clock says.
    pub fn run_loop(&mut self) -> Result<bool> {
        let loop_ = self.loop_.clone();
        Ok(self.call(&loop_)? != 0)
    }

    /// The plugin thread's rule at `now`: `Loop` when `NextRun` has passed and the rate is
    /// above zero, `NextRun` then `1000 / loopratehz` ms on. Returns whether `Loop` ran.
    /// `// C#: MainV2.cs:2524-2540`
    pub fn tick(&mut self, now: Instant) -> Result<bool> {
        if now <= self.next_run || self.loop_rate_hz <= 0.0 {
            return Ok(false);
        }
        let ms = (1000.0 / self.loop_rate_hz) as u64;
        self.next_run = now + Duration::from_millis(ms);
        self.run_loop()?;
        Ok(true)
    }

    /// `Exit`.
    pub fn exit(&mut self) -> Result<bool> {
        let exit = self.exit.clone();
        Ok(self.call(&exit)? != 0)
    }

    /// A menu entry the plugin added, chosen.
    pub fn menu_click(&mut self, id: i32) -> Result<bool> {
        self.store.set_fuel(FUEL_PER_CALL)?;
        Ok(self
            .menu_click
            .call(&mut self.store, id)
            .map_err(describe)?
            != 0)
    }

    /// Any other export taking nothing and returning an `i32`: the experiment's `explode` and
    /// `spin`, and the tests' readings.
    pub fn call_i32(&mut self, name: &str) -> Result<i32> {
        let function = self
            .instance
            .get_typed_func::<(), i32>(&mut self.store, name)?;
        self.call(&function)
    }

    /// An export returning an `f64`.
    pub fn call_f64(&mut self, name: &str) -> Result<f64> {
        let function = self
            .instance
            .get_typed_func::<(), f64>(&mut self.store, name)?;
        self.call(&function)
    }

    /// An export returning a `u64`.
    pub fn call_u64(&mut self, name: &str) -> Result<u64> {
        let function = self
            .instance
            .get_typed_func::<(), u64>(&mut self.store, name)?;
        self.call(&function)
    }

    /// The plugin's linear memory, in bytes.
    pub fn memory_bytes(&self) -> usize {
        self.memory.data_size(&self.store)
    }
}

/// What went wrong in a plugin call, in words the log can carry: a trap - a panic's
/// `unreachable`, the fuel running out - or anything else. Read off wasmtime's own error
/// before it becomes an `anyhow` one, which would hide the trap from a downcast.
fn describe(err: wasmtime::Error) -> anyhow::Error {
    match err.downcast_ref::<Trap>() {
        Some(Trap::OutOfFuel) => {
            anyhow!("the plugin did not return: stopped after {FUEL_PER_CALL} units of fuel")
        }
        Some(Trap::UnreachableCodeReached) => anyhow!("the plugin panicked"),
        Some(trap) => anyhow!("the plugin trapped: {trap}"),
        None => err.into(),
    }
}
