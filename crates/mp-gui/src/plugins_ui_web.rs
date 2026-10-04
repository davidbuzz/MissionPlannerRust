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

//! The browser build's Plugins: none, under the names the rest of the planner uses.
//!
//! The desktop's plugins run on wasmtime (mp-plugin-host, beside plugins_ui.rs), which does not
//! build for a web page; there a plugin would run on the browser's own WebAssembly engine
//! instead. Until it does, the Plugins screen says so, and nothing adds a menu entry or a page.

use gpui::{AnyElement, Context, Window};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;

/// A map menu entry a plugin added: there are none.
#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    pub plugin: usize,
    pub id: u32,
    pub parent: Option<u32>,
    pub text: String,
}

impl Entry {
    #[must_use]
    pub fn probe_id(&self) -> &'static str {
        "plugin-menu"
    }

    #[must_use]
    pub fn label(&self) -> String {
        self.text.clone()
    }
}

/// The plugins: none started.
#[derive(Default)]
pub struct Plugins {
    pub manager: crate::plugin_manager::PluginManager,
}

impl Plugins {
    pub fn start(_persisted: &crate::settings::Persisted, _cx: &mut gpui::App) -> Self {
        Self::default()
    }

    pub fn open_manager(&mut self, _persisted: &crate::settings::Persisted) {}

    #[must_use]
    pub fn planner_entries(&self) -> Vec<Entry> {
        Vec::new()
    }
}

/// Nothing a plugin draws over the screens.
pub fn overlay(
    _this: &MissionPlanner,
    _fly_screen: bool,
    _window: &Window,
    _cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    Vec::new()
}

impl MissionPlanner {
    pub(crate) fn plugin_manager_save(&mut self) {}

    pub(crate) fn plugins_tick(
        &mut self,
        _view: &TelemetryView,
        _window: &mut Window,
        _cx: &mut Context<Self>,
    ) {
    }
}
