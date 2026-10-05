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

//! The rest of row 71 (PLAN §13.6), ported together: CONFIG's Standard Params and Advanced
//! Params (`ConfigFriendlyParams.cs`, `GCSViews/SoftwareConfig.cs:198, 203`) and MAVFtp
//! (`Controls/MavFTPUI.cs`, `:215`), and SETUP's Heli Setup (`ConfigTradHeli4.cs`,
//! `GCSViews/InitialSetup.cs:187`). Each page is its own module; this one holds their page
//! objects, their keyboard focus, and their part in the application: the arms of `setup.rs` and
//! the lines of `main.rs` call into `impl MissionPlanner` here.
//!
//! The C#'s boxes for a failure of the link's work - a write's "Set X Failed", the fetch's "Error
//! receiving list", the MAVFtp page's errors - go on the status line, as the owner ruled on
//! 2026-09-25 (`extra_setup::link_error`).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use web_time::Instant;

use gpui::{AnyElement, Context, FocusHandle, Window, div, prelude::*};

use super::adsb::{self, Access, Adsb};
use super::extra_setup::{link_error, status_words};
use super::{friendly_params, mavftp, onboard_osd, onboard_osd_ui, trad_heli};
use crate::MissionPlanner;
use crate::setup::Key;
use crate::telemetry::TelemetryView;

/// The four page objects.
#[derive(Debug)]
pub struct SoftwarePages2 {
    /// Standard Params.
    pub standard: Adsb,
    /// Advanced Params.
    pub advanced: Adsb,
    /// MAVFtp.
    pub mavftp: mavftp::MavFtp,
    /// Heli Setup.
    pub heli: trad_heli::TradHeli,
    /// Onboard OSD.
    pub onboard_osd: onboard_osd::OnboardOsd,
}

impl Default for SoftwarePages2 {
    fn default() -> Self {
        Self {
            standard: friendly_params::standard_page(),
            advanced: friendly_params::advanced_page(),
            mavftp: mavftp::MavFtp::default(),
            heli: trad_heli::TradHeli::default(),
            onboard_osd: onboard_osd::OnboardOsd::default(),
        }
    }
}

/// The keyboard focus of the pages' boxes.
pub struct Focus {
    /// The number being typed into: a `RangeControl`'s, or a `MavlinkNumericUpDown`'s.
    pub number: FocusHandle,
    /// An `InputBox`'s text box: Find's, New Folder's, a path's.
    pub prompt: FocusHandle,
    /// A MAVFtp row's name being edited.
    pub rename: FocusHandle,
    /// A params page itself, for `ProcessCmdKey`'s Ctrl+S.
    pub page: FocusHandle,
    /// A `RangeControl`'s track bar, for its keys.
    pub track: FocusHandle,
    /// An Onboard OSD text box being typed into.
    pub osd_text: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
            prompt: cx.focus_handle(),
            rename: cx.focus_handle(),
            page: cx.focus_handle(),
            track: cx.focus_handle(),
            osd_text: cx.focus_handle(),
        }
    }
}

/// How Standard Params' drawing reaches its page object.
fn standard(this: &mut MissionPlanner) -> &mut Adsb {
    &mut this.software_pages2.standard
}

/// How Advanced Params' drawing reaches its page object.
fn advanced(this: &mut MissionPlanner) -> &mut Adsb {
    &mut this.software_pages2.advanced
}

/// Facts a UI test asserts on, for the four pages.
pub fn record_facts(pages: &SoftwarePages2, view: &TelemetryView) {
    adsb::record_facts(&pages.standard, view);
    adsb::record_facts(&pages.advanced, view);
    mavftp::record_facts(&pages.mavftp);
    trad_heli::record_facts(&pages.heli, view);
    onboard_osd::record_facts(&pages.onboard_osd, view);
    crate::display_view::record_facts();
}

impl MissionPlanner {
    /// `Activate`, when one of the four is chosen: the two parameter pages and Heli Setup are
    /// `IActivate`, called every time; MAVFtp is made and loaded once per screen.
    pub(crate) fn software2_activate(&mut self, class: &str) {
        let view = self.telemetry.view();
        let key = Key::of(&view);
        let lookup = crate::metadata::lookup;
        let pages = &mut self.software_pages2;
        match class {
            // C#: GCSViews/ConfigurationView/ConfigFriendlyParams.cs:253-260
            friendly_params::STANDARD_CLASS | friendly_params::ADVANCED_CLASS => {
                let list = if class == friendly_params::STANDARD_CLASS {
                    &mut pages.standard
                } else {
                    &mut pages.advanced
                };
                let favourites = adsb::favourites(friendly_params::FAVOURITES);
                let jobs = list.activate(
                    &view.parameters,
                    &|name| view.parameter_type(name),
                    key,
                    lookup,
                    &favourites,
                );
                list.push(jobs);
            }
            // C#: Controls/MavFTPUI.cs:29-71, 665-668
            mavftp::CLASS => pages.mavftp.activate(view.vehicle, key),
            // C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:28-172
            trad_heli::CLASS => {
                let jobs = pages.heli.activate(&view.parameters, key, lookup);
                pages.heli.push(jobs);
            }
            // C#: GCSViews/ConfigurationView/ConfigOSD.cs:219-231
            onboard_osd::CLASS => pages.onboard_osd.activate(&view.parameters),
            _ => {}
        }
    }

    /// The page hidden when another is chosen: `Deactivate` for Heli Setup, which empties its
    /// tables; the others hidden, a number or a name being typed into read.
    pub(crate) fn software2_deactivate(&mut self, class: &str) {
        let pages = &mut self.software_pages2;
        match class {
            friendly_params::STANDARD_CLASS => pages.standard.deactivate(),
            friendly_params::ADVANCED_CLASS => pages.advanced.deactivate(),
            mavftp::CLASS => pages.mavftp.hide(),
            // C#: GCSViews/ConfigurationView/ConfigTradHeli4.cs:187-193
            trad_heli::CLASS => {
                let jobs = pages.heli.deactivate(Instant::now());
                pages.heli.push(jobs);
            }
            // C#: GCSViews/ConfigurationView/ConfigOSD.cs:233-237
            onboard_osd::CLASS => {
                let view = self.telemetry.view();
                let connected = view.connected && view.vehicle.is_some();
                let telemetry = &self.telemetry;
                self.software_pages2
                    .onboard_osd
                    .deactivate(telemetry, connected);
            }
            _ => {}
        }
    }

    /// The page, when one of the four is showing.
    pub(crate) fn software2_page(
        &self,
        class: &str,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        let pages = &self.software_pages2;
        let focus = &self.software2_focus;
        match class {
            friendly_params::STANDARD_CLASS => {
                let access: Access = standard;
                friendly_params::page(
                    &pages.standard,
                    access,
                    &focus.number,
                    &focus.prompt,
                    &focus.page,
                    &focus.track,
                    window,
                    cx,
                )
            }
            friendly_params::ADVANCED_CLASS => {
                let access: Access = advanced;
                friendly_params::page(
                    &pages.advanced,
                    access,
                    &focus.number,
                    &focus.prompt,
                    &focus.page,
                    &focus.track,
                    window,
                    cx,
                )
            }
            mavftp::CLASS => mavftp::page(&pages.mavftp, &focus.rename, window, cx),
            trad_heli::CLASS => trad_heli::page(&pages.heli, &focus.number, window, cx),
            onboard_osd::CLASS => onboard_osd_ui::page(
                &pages.onboard_osd,
                &focus.number,
                &focus.osd_text,
                window,
                cx,
            ),
            _ => div().into_any_element(),
        }
    }

    /// Once a frame: each page object disposed with its screen, the timers, a number the focus
    /// has left read, the writes and the MAVFtp commands moved on - and the link's failures the
    /// C# boxes put on the status line.
    pub(crate) fn software2_tick(&mut self, view: &TelemetryView, window: &Window) {
        let on_config = self.screen == crate::Screen::Config;
        let on_setup = self.screen == crate::Screen::Setup;
        let now = Instant::now();
        let number_focused = self.software2_focus.number.is_focused(window);
        let telemetry = &self.telemetry;
        let pages = &mut self.software_pages2;
        pages.standard.tick(telemetry, view, on_config, now);
        pages.advanced.tick(telemetry, view, on_config, now);
        pages.mavftp.tick(telemetry, view, on_config, now);
        pages
            .heli
            .tick(telemetry, view, on_setup, number_focused, now);
        pages.onboard_osd.tick(telemetry, view, now);
        // The two parameter pages move their link failures out of their boxes themselves, in
        // their tick (`adsb.rs`); the words are taken here.
        let mut status = pages.standard.take_status();
        if let Some(words) = pages.advanced.take_status() {
            status = Some(words);
        }
        while let Some(message) = pages.heli.message().filter(|m| link_error(m)) {
            status = Some(status_words(message));
            pages.heli.dismiss_message();
        }
        while let Some(line) = pages.mavftp.take_status_line() {
            status = Some(line.replace("\r\n", " ").replace('\n', " "));
        }
        if let Some(words) = pages.onboard_osd.take_status() {
            status = Some(words);
        }
        if status.is_some() {
            self.file_status = status;
        }
    }

    /// The box, question or window one of the four is showing, over the whole window.
    pub(crate) fn software2_overlay(
        &self,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> Option<AnyElement> {
        let pages = &self.software_pages2;
        let prompt = &self.software2_focus.prompt;
        adsb::list_overlay(&pages.standard, standard, prompt, window, cx)
            .or_else(|| adsb::list_overlay(&pages.advanced, advanced, prompt, window, cx))
            .or_else(|| mavftp::overlay(&pages.mavftp, prompt, window, cx))
            .or_else(|| trad_heli::overlay(&pages.heli, window, cx))
            .or_else(|| onboard_osd_ui::overlay(&pages.onboard_osd, &self.software2_focus.osd_text, window, cx))
    }
}
