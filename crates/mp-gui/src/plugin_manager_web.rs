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

//! The browser build's Plugin Manager: the screen says plugins are not in this build yet (see
//! plugins_ui_web.rs).

use gpui::{AnyElement, Context, div, prelude::*};

use crate::MissionPlanner;

#[derive(Debug, Default)]
pub(crate) struct PluginManager;

impl PluginManager {
    #[must_use]
    pub(crate) fn is_open(&self) -> bool {
        false
    }

    pub(crate) fn close(&mut self) {}
}

pub(crate) fn screen(_this: &MissionPlanner, _cx: &mut Context<MissionPlanner>) -> AnyElement {
    div()
        .p_4()
        .child("Plugins are not in the browser build yet.")
        .into_any_element()
}
