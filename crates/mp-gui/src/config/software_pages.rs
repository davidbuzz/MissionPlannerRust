//! Three of CONFIG's pages, ported together (PLAN §13.6 row 71): GeoFence (`ConfigAC_Fence.cs`,
//! `GCSViews/SoftwareConfig.cs:156`), the rover's Basic Tuning (`ConfigArdurover.cs`, `:188`) and
//! User Params (`ConfigUserDefined.cs`, `:221`). Each page is its own module; this one holds
//! their page objects, their keyboard focus, and their part in the application: the arms of
//! `setup.rs` and the lines of `main.rs` call into `impl MissionPlanner` here.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::time::Instant;

use gpui::{AnyElement, Context, FocusHandle, KeyDownEvent, Window, div, prelude::*};

use super::{geofence, rover_tuning, user_params};
use crate::MissionPlanner;
use crate::setup::Key;
use crate::telemetry::TelemetryView;

/// The three page objects.
#[derive(Debug, Default)]
pub struct SoftwarePages {
    /// GeoFence.
    pub geofence: geofence::GeoFence,
    /// The rover's Basic Tuning.
    pub rover: rover_tuning::RoverTuning,
    /// User Params.
    pub user: user_params::UserParams,
}

/// The keyboard focus of the pages' boxes: the number being typed into, and the `InputBox`'s
/// text box.
pub struct Focus {
    /// The `NumericUpDown` being typed into.
    pub number: FocusHandle,
    /// User Params' `InputBox`.
    pub prompt: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
            prompt: cx.focus_handle(),
        }
    }
}

/// Facts a UI test asserts on, for the three pages.
pub fn record_facts(pages: &SoftwarePages, rover_listed: bool, view: &TelemetryView) {
    geofence::record_facts(&pages.geofence, view);
    rover_tuning::record_facts(&pages.rover, rover_listed, view);
    user_params::record_facts(&pages.user, view);
}

impl MissionPlanner {
    /// `Activate`, when one of the three is chosen: each is `IActivate`, called every time.
    pub(crate) fn software_activate(&mut self, class: &str) {
        let view = self.telemetry.view();
        let key = Key::of(&view);
        let lookup = crate::metadata::lookup;
        match class {
            geofence::CLASS => {
                let units = self.planner.units();
                let jobs =
                    self.software_pages
                        .geofence
                        .activate(&view.parameters, key, units, lookup);
                self.software_pages.geofence.push(jobs);
            }
            rover_tuning::CLASS => {
                let vehicle = crate::setup::Vehicle::of(&view, self.telemetry.firmware_banner());
                let jobs = self.software_pages.rover.activate(
                    &view.parameters,
                    key,
                    vehicle.connected,
                    vehicle.firmware,
                    lookup,
                );
                self.software_pages.rover.push(jobs);
            }
            user_params::CLASS => {
                let setting = self.persisted.get(user_params::SETTING).map(str::to_owned);
                self.software_pages.user.activate(
                    &view.parameters,
                    key,
                    setting.as_deref(),
                    lookup,
                );
            }
            _ => {}
        }
    }

    /// The page hidden when another is chosen: `Deactivate` for User Params, which does nothing,
    /// and the others' number being typed into read.
    pub(crate) fn software_deactivate(&mut self, class: &str) {
        let now = Instant::now();
        match class {
            geofence::CLASS => self.software_pages.geofence.hide(now),
            rover_tuning::CLASS => self.software_pages.rover.hide(now),
            user_params::CLASS => self.software_pages.user.deactivate(),
            _ => {}
        }
    }

    /// The page, when one of the three is showing.
    pub(crate) fn software_page(
        &self,
        class: &str,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        let pages = &self.software_pages;
        let focus = &self.software_focus;
        match class {
            geofence::CLASS => geofence::page(&pages.geofence, &focus.number, window, cx),
            rover_tuning::CLASS => rover_tuning::page(&pages.rover, &focus.number, window, cx),
            user_params::CLASS => user_params::page(&pages.user, cx),
            _ => div().into_any_element(),
        }
    }

    /// Once a frame: each page object disposed with its screen, a number the focus has left
    /// read, the timers, and the writes.
    pub(crate) fn software_tick(&mut self, view: &TelemetryView, window: &Window) {
        let on_config = self.screen == crate::Screen::Config;
        let now = Instant::now();
        let focused = self.software_focus.number.is_focused(window);
        let telemetry = &self.telemetry;
        let pages = &mut self.software_pages;
        pages
            .geofence
            .tick(telemetry, view, on_config, focused, now);
        pages.rover.tick(telemetry, view, on_config, focused, now);
        pages.user.tick(telemetry, view, on_config);
        // What the pages' controls box when the link fails - "Set X Failed", "Set X Failed!" -
        // goes on the status line instead: the owner's ruling of 2026-09-25. Each page moves
        // those out of its boxes as its writes move on.
        let failures = [
            pages.geofence.take_status(),
            pages.rover.take_status(),
            pages.user.take_status(),
        ];
        for status in failures.into_iter().flatten() {
            self.file_status = Some(status);
        }
    }

    /// The box or question one of the three is showing, over the whole window.
    pub(crate) fn software_overlay(
        &self,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> Option<AnyElement> {
        let pages = &self.software_pages;
        geofence::overlay(&pages.geofence, window, cx)
            .or_else(|| rover_tuning::overlay(&pages.rover, window, cx))
            .or_else(|| user_params::overlay(&pages.user, &self.software_focus.prompt, window, cx))
    }

    /// A key for User Params' `InputBox`; Escape closes it as Cancel does.
    pub(crate) fn user_params_input_key(&mut self, event: &KeyDownEvent) {
        if let Some(ok) = self.software_pages.user.input_key(event) {
            self.user_params_close_input(ok);
        }
    }

    /// User Params' `InputBox` closed: the list it leaves, saved to `Settings.Instance` - in
    /// memory, written to config.xml at the next save, as every page's keys are.
    /// `// C#: GCSViews/ConfigurationView/ConfigUserDefined.cs:56-59`
    pub(crate) fn user_params_close_input(&mut self, ok: bool) {
        let view = self.telemetry.view();
        if let Some(setting) =
            self.software_pages
                .user
                .close_input(ok, &view.parameters, crate::metadata::lookup)
        {
            self.persisted.set(user_params::SETTING, setting);
        }
    }
}
