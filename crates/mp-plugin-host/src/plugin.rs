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

//! One plugin: a component instantiated with the host's imports, and the `Plugin` lifecycle
//! called on it with fuel.
//!
//! `Plugin::load` is `PluginLoader.Load` and `InitPlugin` less the `Init` call
//! (`// C#: Plugin/PluginLoader.cs:99-201`); `init`, `loaded`, `run_loop` and `exit` are the
//! calls the loader and `MainV2`'s plugin thread make; [`Plugin::tick`] is that thread's rate
//! rule (`// C#: MainV2.cs:2514-2540`).

use std::fmt;
use web_time::{Duration, Instant};

use wasmtime::component::{Component, HasSelf, Linker};
use wasmtime::{Config, Engine, Store, StoreLimits, StoreLimitsBuilder, Trap};

use crate::Surface;
use crate::wit;

/// How much a plugin may do in one call before it is stopped.
///
/// Fuel is wasmtime's count of the plugin's own instructions - time it spends waiting in the host
/// (for a message box's answer, say) costs nothing - so a budget is a bound on computation, not
/// on the clock. The C# runs `Loop` on its plugin thread and a menu click on the window's thread
/// for as long as either takes; here each call has a tank, and a plugin that runs it dry is
/// stopped and unloaded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limits {
    /// For `init`, `loaded`, `loop`, `exit` and the reads of `name`, `version`, `author` and
    /// `loopratehz`: about a second of computation.
    pub fuel_per_call: u64,
    /// For a menu entry's click and a form's event: the C#'s handlers run long work there (a log
    /// anonymised, a terrain file made), so these get about half a minute.
    pub fuel_per_event: u64,
    /// The most linear memory a plugin may grow to, in bytes.
    pub memory: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            fuel_per_call: 1_000_000_000,
            fuel_per_event: 30_000_000_000,
            memory: 1 << 30,
        }
    }
}

/// Why a call into a plugin failed. Every one of them unloads the plugin in [`crate::PluginHost`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Fault {
    /// The file is not a plugin: not WebAssembly, not of this world, or it failed to
    /// instantiate.
    Load(String),
    /// The plugin panicked: a Rust plugin's panic is the `unreachable` instruction.
    Panicked,
    /// The plugin ran its tank dry: it did not return.
    OutOfFuel(u64),
    /// Any other trap: an out-of-bounds access, a division by zero, memory refused.
    Trapped(String),
}

impl fmt::Display for Fault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Load(reason) => write!(f, "not loaded: {reason}"),
            Self::Panicked => f.write_str("the plugin panicked"),
            Self::OutOfFuel(fuel) => {
                write!(
                    f,
                    "the plugin did not return: stopped after {fuel} units of fuel"
                )
            }
            Self::Trapped(trap) => write!(f, "the plugin trapped: {trap}"),
        }
    }
}

impl std::error::Error for Fault {}

/// What went wrong in a call, read off wasmtime's error: its trap, if it is one.
fn describe(err: &wasmtime::Error, fuel: u64) -> Fault {
    match err.downcast_ref::<Trap>() {
        Some(Trap::OutOfFuel) => Fault::OutOfFuel(fuel),
        Some(Trap::UnreachableCodeReached) => Fault::Panicked,
        Some(trap) => Fault::Trapped(trap.to_string()),
        None => Fault::Trapped(format!("{err:#}")),
    }
}

/// `Name`, `Version` and `Author`, read once at load, and the file the plugin came from
/// (`Plugin.FileName`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Info {
    /// `Name`.
    pub name: String,
    /// `Version`.
    pub version: String,
    /// `Author`.
    pub author: String,
    /// `FileName`: the file's name, without its folder.
    pub file: String,
}

/// The engine every plugin is compiled with: Cranelift, fuel on; in a web page, [`web_config`]'s.
///
/// # Errors
/// When wasmtime cannot make an engine for this machine.
pub fn engine() -> Result<Engine, Fault> {
    #[cfg(not(target_family = "wasm"))]
    let config = {
        let mut config = Config::new();
        config.consume_fuel(true);
        config
    };
    #[cfg(target_family = "wasm")]
    let config = web_config()?;
    Engine::new(&config).map_err(|err| Fault::Load(format!("{err:#}")))
}

/// The settings a web page's engine runs plugins with (the browser build): Pulley's 32-bit
/// bytecode on wasmtime's interpreter, since a page cannot run machine code wasmtime makes; no
/// signal handlers, no virtual memory reserved, guarded or copied on write, which a page has none
/// of; and fuel on, as on the desktop. [`precompile_for_web`] compiles with the same, and wasmtime
/// refuses bytecode compiled under any other.
///
/// # Errors
/// When this wasmtime was built without Pulley.
pub fn web_config() -> Result<Config, Fault> {
    let mut config = Config::new();
    config
        .target("pulley32")
        .map_err(|err| Fault::Load(format!("{err:#}")))?;
    config.signals_based_traps(false);
    config.memory_reservation(0);
    config.memory_guard_size(0);
    config.memory_init_cow(false);
    config.consume_fuel(true);
    Ok(config)
}

/// A plugin - a component, or a core module carrying the world - compiled to the Pulley bytecode a
/// web page's engine runs ([`web_config`]): what mp-gui's build script carries into the browser
/// build for each built-in plugin.
///
/// # Errors
/// [`Fault::Load`] when the bytes are not a plugin of this world.
#[cfg(not(target_family = "wasm"))]
pub fn precompile_for_web(bytes: &[u8]) -> Result<Vec<u8>, Fault> {
    let engine = Engine::new(&web_config()?).map_err(|err| Fault::Load(format!("{err:#}")))?;
    let component = componentized(bytes)?;
    engine
        .precompile_component(&component)
        .map_err(|err| Fault::Load(format!("{err:#}")))
}

/// What a plugin's store holds: its surface, and the limits its memory is held to.
pub(crate) struct State {
    pub(crate) surface: Box<dyn Surface>,
    limits: StoreLimits,
}

/// A loaded plugin: `Plugin` with its `Host`.
pub struct Plugin {
    store: Store<State>,
    bindings: wit::Plugin,
    info: Info,
    limits: Limits,
    /// `loopratehz`, read again after every call.
    loop_rate_hz: f32,
    /// `NextRun`: `None` until the first tick, which runs `Loop` at once.
    next_run: Option<Instant>,
}

impl fmt::Debug for Plugin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Plugin")
            .field("info", &self.info)
            .field("loop_rate_hz", &self.loop_rate_hz)
            .finish_non_exhaustive()
    }
}

/// The component in `bytes`: as it is when it is one, made one when it is a core module that
/// carries the world (a plugin built for `wasm32-unknown-unknown` with wit-bindgen). In a web page,
/// the Pulley bytecode the desktop compiled it to ([`precompile_for_web`]).
#[cfg(not(target_family = "wasm"))]
fn component(engine: &Engine, bytes: &[u8]) -> Result<Component, Fault> {
    let bytes = componentized(bytes)?;
    Component::new(engine, &bytes).map_err(|err| Fault::Load(format!("{err:#}")))
}

#[cfg(target_family = "wasm")]
fn component(engine: &Engine, bytes: &[u8]) -> Result<Component, Fault> {
    crate::web::deserialize(engine, bytes)
}

/// `bytes` as a component: as they are when they are one, encoded when they are a core module.
#[cfg(not(target_family = "wasm"))]
fn componentized(bytes: &[u8]) -> Result<std::borrow::Cow<'_, [u8]>, Fault> {
    // The preamble's version and layer: `01 00 00 00` is a core module; a component's layer is 1.
    if bytes.get(4..8) != Some(&[1, 0, 0, 0][..]) {
        return Ok(std::borrow::Cow::Borrowed(bytes));
    }
    wit_component::ComponentEncoder::default()
        .module(bytes)
        .and_then(|encoder| encoder.validate(true).encode())
        .map(std::borrow::Cow::Owned)
        .map_err(|err| Fault::Load(format!("{err:#}")))
}

impl Plugin {
    /// `PluginLoader.Load`: `bytes` compiled, instantiated with `surface` behind its imports,
    /// and its `Name`, `Version`, `Author` and `loopratehz` read. `file` is `FileName`.
    /// `// C#: Plugin/PluginLoader.cs:99-201`
    ///
    /// # Errors
    /// [`Fault::Load`] when the bytes are not a plugin of this world; any other fault when
    /// reading who it is fails.
    pub fn load(
        engine: &Engine,
        bytes: &[u8],
        file: &str,
        surface: Box<dyn Surface>,
        limits: Limits,
    ) -> Result<Self, Fault> {
        let component = component(engine, bytes)?;
        let mut linker = Linker::<State>::new(engine);
        wit::missionplanner::plugin::host::add_to_linker::<State, HasSelf<State>>(
            &mut linker,
            |state| state,
        )
        .map_err(|err| Fault::Load(format!("{err:#}")))?;
        let state = State {
            surface,
            limits: StoreLimitsBuilder::new().memory_size(limits.memory).build(),
        };
        let mut store = Store::new(engine, state);
        store.limiter(|state| &mut state.limits);
        store
            .set_fuel(limits.fuel_per_call)
            .map_err(|err| Fault::Load(format!("{err:#}")))?;
        let bindings = wit::Plugin::instantiate(&mut store, &component, &linker)
            .map_err(|err| Fault::Load(format!("{err:#}")))?;
        let mut plugin = Self {
            store,
            bindings,
            info: Info {
                file: file.to_owned(),
                ..Info::default()
            },
            limits,
            loop_rate_hz: 0.0,
            next_run: None,
        };
        let fuel = limits.fuel_per_call;
        plugin.info.name = plugin.call(fuel, |b, s| b.call_name(s))?;
        plugin.info.version = plugin.call(fuel, |b, s| b.call_version(s))?;
        plugin.info.author = plugin.call(fuel, |b, s| b.call_author(s))?;
        Ok(plugin)
    }

    /// Who the plugin says it is.
    #[must_use]
    pub const fn info(&self) -> &Info {
        &self.info
    }

    /// `loopratehz` as the plugin last said it.
    #[must_use]
    pub const fn loop_rate_hz(&self) -> f32 {
        self.loop_rate_hz
    }

    /// The surface the plugin's imports reach.
    pub fn surface(&mut self) -> &mut dyn Surface {
        self.store.data_mut().surface.as_mut()
    }

    /// One call into the plugin with a fresh tank of `fuel`, then `loopratehz` read again: a
    /// trap, a panic or the tank run dry is a [`Fault`], and the host goes on.
    fn call<R>(
        &mut self,
        fuel: u64,
        call: impl FnOnce(&wit::Plugin, &mut Store<State>) -> wasmtime::Result<R>,
    ) -> Result<R, Fault> {
        self.store
            .set_fuel(fuel)
            .map_err(|err| describe(&err, fuel))?;
        let result = call(&self.bindings, &mut self.store).map_err(|err| describe(&err, fuel))?;
        self.store
            .set_fuel(self.limits.fuel_per_call)
            .map_err(|err| describe(&err, fuel))?;
        self.loop_rate_hz = self
            .bindings
            .call_loop_rate_hz(&mut self.store)
            .map_err(|err| describe(&err, self.limits.fuel_per_call))?;
        Ok(result)
    }

    /// `Init`: false and the loader drops the plugin.
    /// `// C#: Plugin/PluginLoader.cs:175-182`
    ///
    /// # Errors
    /// The plugin's fault.
    pub fn init(&mut self) -> Result<bool, Fault> {
        self.call(self.limits.fuel_per_call, |b, s| b.call_init(s))
    }

    /// `Loaded`: false and the plugin is not added to the running ones, so `Loop` never runs.
    /// `// C#: Plugin/PluginLoader.cs:313-331`
    ///
    /// # Errors
    /// The plugin's fault.
    pub fn loaded(&mut self) -> Result<bool, Fault> {
        self.call(self.limits.fuel_per_call, |b, s| b.call_loaded(s))
    }

    /// `Loop`, now, whatever the clock says.
    ///
    /// # Errors
    /// The plugin's fault.
    pub fn run_loop(&mut self) -> Result<bool, Fault> {
        self.call(self.limits.fuel_per_call, |b, s| b.call_run_loop(s))
    }

    /// The plugin thread's rule at `now`: `Loop` when `NextRun` has passed and `loopratehz` is
    /// above zero, `NextRun` moved on by `1000 / loopratehz` ms before the call, so `Loop` may
    /// change the rate for the next. Returns whether `Loop` ran.
    /// `// C#: MainV2.cs:2524-2540`
    ///
    /// # Errors
    /// The plugin's fault.
    pub fn tick(&mut self, now: Instant) -> Result<bool, Fault> {
        if self.next_run.is_some_and(|next| now <= next) || self.loop_rate_hz <= 0.0 {
            return Ok(false);
        }
        self.next_run = Some(now + Self::period(self.loop_rate_hz));
        self.run_loop()?;
        Ok(true)
    }

    /// When `Loop` runs next: `None` while the rate is zero, which is never.
    #[must_use]
    pub fn next_run(&self) -> Option<Instant> {
        if self.loop_rate_hz <= 0.0 {
            return None;
        }
        Some(self.next_run.unwrap_or_else(Instant::now))
    }

    /// `(int)(1000 / loopratehz)` ms.
    fn period(rate_hz: f32) -> Duration {
        let ms = (1000.0 / f64::from(rate_hz)).floor();
        // The rate is above zero here, so the period is finite and not negative.
        Duration::from_millis(if ms.is_finite() && ms > 0.0 {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let ms = ms as u64;
            ms
        } else {
            0
        })
    }

    /// `Exit`: the plugin is being unloaded.
    /// `// C#: MainV2.cs:2556-2569`
    ///
    /// # Errors
    /// The plugin's fault.
    pub fn exit(&mut self) -> Result<bool, Fault> {
        self.call(self.limits.fuel_per_call, |b, s| b.call_exit(s))
    }

    /// A menu entry the plugin added was chosen, the menu opened at `lat`, `lng`.
    ///
    /// # Errors
    /// The plugin's fault.
    pub fn menu_click(&mut self, id: u32, lat: f64, lng: f64) -> Result<(), Fault> {
        self.call(self.limits.fuel_per_event, |b, s| {
            b.call_menu_click(s, id, lat, lng)
        })
    }

    /// A control of the plugin's form changed or was clicked.
    ///
    /// # Errors
    /// The plugin's fault.
    pub fn form_event(&mut self, id: &str, value: &str) -> Result<(), Fault> {
        self.call(self.limits.fuel_per_event, |b, s| {
            b.call_form_event(s, id, value)
        })
    }
}
