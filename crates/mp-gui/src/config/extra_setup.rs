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

//! Six of SETUP's small pages, ported together (PLAN §13.6 row 70): Parachute
//! (`ConfigHWParachute.cs`, `GCSViews/InitialSetup.cs:325`), OSD (`ConfigHWOSD.cs`, `:305`), CAN
//! GPS Order (`ConfigGPSOrder.cs`, `:266`), HW ID (`ConfigHWIDs.cs`, `:241`), Compass/Motor Calib
//! (`ConfigCompassMot.cs`, `:285`) and Initial Tune Parameter (`ConfigInitialParams.cs`, `:237`).
//! Each page is its own module; this one holds their page objects, their keyboard focus, and
//! their part in the application: the arms of `setup.rs` and the lines of `main.rs` call into
//! `impl MissionPlanner` here.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use web_time::Instant;

use gpui::{AnyElement, Context, FocusHandle, Window, div, prelude::*};

use super::{
    auth_keys, compass_mot, fft, gps_order, hw_ids, initial_params, mavlink_inspector,
    mavlink_mirror, nmea_output, osd, parachute, param_gen, proximity, sikradio, spectrogram,
    support_proxy, warnings_manager,
};
use crate::MissionPlanner;
use crate::config::servo_output::{ERROR_TITLE, Message};
use crate::setup::Key;
use crate::telemetry::TelemetryView;

/// Whether a message is one of the C#'s boxes for a link that failed - `setParam`'s timeout
/// (`Strings.ERROR`, the unhandled-exception "Send Error", `ConfigGPSOrder`'s "Failed to set
/// param ..." caption) or `ConfigHWOSD`'s "Failed to set OSD rates." - as against a question, a
/// validation of what was typed ("ERROR!") or a report of success, which keep their boxes.
///
/// The owner's ruling of 2026-09-25: an error the window already shows as state - the link's
/// state is always at the top right - never gets a message box. These go on the status line.
#[must_use]
pub fn link_error(message: &Message) -> bool {
    message.title == ERROR_TITLE
        || message.title == "Send Error"
        || message.title.starts_with("Failed to set param")
        || message.text == osd::FAILED
}

/// The status line's words for a link error: the caption when it carries the reason (the GPS
/// order page puts the exception there), else the text.
#[must_use]
pub fn status_words(message: &Message) -> String {
    if message.title.starts_with("Failed to set param") {
        message.title.to_owned()
    } else {
        message.text.replace('\n', " ")
    }
}

/// Moves the link errors - those `is_link` picks out - from a page's queue of boxes, for the
/// status line: the words of the last of them, on one trimmed line, or `None` when there were
/// none. The boxes left
/// keep their order. A page calls this after its writes move on, so a failure never reaches its
/// `message()` and never draws; the holder's tick hands the words to the status line.
pub fn take_link_errors(
    messages: &mut VecDeque<Message>,
    is_link: impl Fn(&Message) -> bool,
) -> Option<String> {
    let mut last = None;
    messages.retain(|message| {
        let link = is_link(message);
        if link {
            last = Some(status_words(message).trim().to_owned());
        }
        !link
    });
    last
}

/// The six page objects.
#[derive(Debug, Default)]
pub struct ExtraSetup {
    /// Parachute.
    pub parachute: parachute::Parachute,
    /// OSD.
    pub osd: osd::Osd,
    /// CAN GPS Order.
    pub gps_order: gps_order::GpsOrder,
    /// HW ID.
    pub hw_ids: hw_ids::HwIds,
    /// Compass/Motor Calib.
    pub compass_mot: compass_mot::CompassMot,
    /// Initial Tune Parameter.
    pub initial_params: initial_params::InitialParams,
    /// FFT Setup, and the FFT window it opens.
    pub fft: fft::Fft,
    /// The MAVLink Inspector the Advanced page opens (`config/mavlink_inspector.rs`).
    pub inspector: mavlink_inspector::InspectorWindow,
    /// The Warning Manager the Advanced page opens (`config/warnings_manager.rs`).
    pub warnings_manager: warnings_manager::ManagerWindow,
    /// The proximity window the Advanced page opens (`config/proximity.rs`).
    pub proximity: proximity::ProximityWindow,
    /// The signing keys window the Advanced page opens, and the key store (`config/auth_keys.rs`).
    pub auth_keys: auth_keys::AuthKeysWindow,
    /// The spectrogram the Advanced page opens (`config/spectrogram.rs`).
    pub spectrogram: spectrogram::SpectrogramWindow,
    /// The Support Proxy the Advanced page opens, and the mirror it starts
    /// (`config/support_proxy.rs`).
    pub support_proxy: support_proxy::SupportProxy,
    /// DroneCAN/UAVCAN (`config/dronecan.rs`).
    pub dronecan: super::dronecan::DroneCan,
    /// Sik Radio (`config/sikradio.rs`).
    pub sikradio: sikradio::SikRadio,
    /// The Mavlink Mirror the Advanced page opens, and its mirrors (`config/mavlink_mirror.rs`).
    pub mavlink_mirror: mavlink_mirror::MavlinkMirror,
    /// The NMEA output the Advanced page opens, and its thread (`config/nmea_output.rs`).
    pub nmea_output: nmea_output::NmeaOutput,
    /// The Cursor-on-Target output EXPERIMENTAL's CoT opens, and its thread
    /// (`config/cot_output.rs`).
    pub cot_output: super::cot_output::CotOutput,
    /// Param gen's run and its dialogue (`config/param_gen.rs`).
    pub param_gen: param_gen::ParamGen,
}

/// The keyboard focus of the pages' boxes: Parachute's number being typed into, and Initial
/// Tune Parameter's text box.
pub struct Focus {
    /// The `NumericUpDown` being typed into.
    pub number: FocusHandle,
    /// The `TextBox` being typed into.
    pub text: FocusHandle,
    /// The FFT window's Bins or Start Freq box being typed into.
    pub fft_number: FocusHandle,
    /// FFT Setup's `INS_LOG_BAT_CNT` track bar, for its keys.
    pub fft_track: FocusHandle,
    /// The FFT window's file dialog or rate question.
    pub fft_prompt: FocusHandle,
    /// The MAVLink Inspector's "Points of history?".
    pub inspector_prompt: FocusHandle,
    /// The Warning Manager's number box being typed into.
    pub warnings_number: FocusHandle,
    /// The Warning Manager's message box being typed into.
    pub warnings_text: FocusHandle,
    /// The proximity window, which takes its four keys.
    pub proximity: FocusHandle,
    /// The signing keys' grid, which takes Delete.
    pub auth_keys: FocusHandle,
    /// The signing keys' Add questions.
    pub auth_keys_prompt: FocusHandle,
    /// The spectrogram's boxes and Load Log's dialog.
    pub spectrogram: spectrogram::FocusHandles,
    /// The Support Proxy's server and number.
    pub support_proxy: support_proxy::FocusHandles,
    /// Sik Radio's key boxes and its dialogs.
    pub sikradio: FocusHandle,
    /// The DroneCAN Inspector's "Points of history?".
    pub dronecan_prompt: FocusHandle,
    /// The Mavlink Mirror's grid cell being typed into.
    pub mirror_cell: FocusHandle,
    /// The NMEA output's questions.
    pub nmea_prompt: FocusHandle,
    /// The CoT output's boxes and grid cell being typed into, and its questions.
    pub cot_typing: FocusHandle,
    pub cot_prompt: FocusHandle,
}

impl Focus {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
            text: cx.focus_handle(),
            fft_number: cx.focus_handle(),
            fft_track: cx.focus_handle(),
            fft_prompt: cx.focus_handle(),
            inspector_prompt: cx.focus_handle(),
            warnings_number: cx.focus_handle(),
            warnings_text: cx.focus_handle(),
            proximity: cx.focus_handle(),
            auth_keys: cx.focus_handle(),
            auth_keys_prompt: cx.focus_handle(),
            spectrogram: spectrogram::FocusHandles::new(cx),
            support_proxy: support_proxy::FocusHandles::new(cx),
            sikradio: cx.focus_handle(),
            dronecan_prompt: cx.focus_handle(),
            mirror_cell: cx.focus_handle(),
            nmea_prompt: cx.focus_handle(),
            cot_typing: cx.focus_handle(),
            cot_prompt: cx.focus_handle(),
        }
    }
}

/// Facts a UI test asserts on, for the six pages.
pub fn record_facts(pages: &ExtraSetup, view: &TelemetryView) {
    parachute::record_facts(&pages.parachute, view);
    osd::record_facts(&pages.osd, view);
    gps_order::record_facts(&pages.gps_order, view);
    hw_ids::record_facts(&pages.hw_ids);
    compass_mot::record_facts(&pages.compass_mot);
    initial_params::record_facts(&pages.initial_params);
    fft::record_facts(&pages.fft, view);
    mavlink_inspector::record_facts(&pages.inspector);
    warnings_manager::record_facts(&pages.warnings_manager);
    proximity::record_facts(&pages.proximity);
    auth_keys::record_facts(&pages.auth_keys);
    spectrogram::record_facts(&pages.spectrogram);
    support_proxy::record_facts(&pages.support_proxy);
    super::dronecan::record_facts(&pages.dronecan);
    super::dronecan_inspector::record_facts(
        pages.dronecan.inspector.as_ref(),
        pages.dronecan.inspector_opened,
    );
    sikradio::record_facts(&pages.sikradio);
    mavlink_mirror::record_facts(&pages.mavlink_mirror);
    nmea_output::record_facts(&pages.nmea_output);
    super::cot_output::record_facts(&pages.cot_output);
    param_gen::record_facts(&pages.param_gen);
}

impl MissionPlanner {
    /// `Activate`, when one of the six is chosen: each is `IActivate`, called every time.
    pub(crate) fn extra_setup_activate(&mut self, class: &str) {
        let view = self.telemetry.view();
        let key = Key::of(&view);
        match class {
            // C#: GCSViews/ConfigurationView/ConfigHWParachute.cs:17-45
            "ConfigHWParachute" => {
                let jobs =
                    self.extra
                        .parachute
                        .activate(&view.parameters, key, crate::metadata::lookup);
                self.extra.parachute.push(jobs);
            }
            // C#: GCSViews/ConfigurationView/ConfigHWOSD.cs:14-21
            "ConfigHWOSD" => self.extra.osd.activate(key),
            // C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:21-51
            "ConfigGPSOrder" => self.extra.gps_order.activate(&view.parameters, key),
            // C#: GCSViews/ConfigurationView/ConfigHWIDs.cs:16-31
            "ConfigHWIDs" => self.extra.hw_ids.activate(&view.parameters, key),
            // C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:25-30
            "ConfigCompassMot" => self.extra.compass_mot.activate(key, view.vehicle),
            // C#: GCSViews/ConfigurationView/ConfigInitialParams.cs:56-68
            "ConfigInitialParams" => self.extra.initial_params.activate(key),
            // C#: GCSViews/ConfigurationView/ConfigFFT.cs:26-64
            "ConfigFFT" => {
                self.extra
                    .fft
                    .activate(&view.parameters, key, crate::metadata::lookup);
            }
            // C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:39-57
            "ConfigDroneCAN" => self.dronecan_activate(),
            // `Sikradio` is no `IActivate`: shown, made the first time.
            "Sikradio" => self.extra.sikradio.activate(),
            _ => {}
        }
    }

    /// The page hidden when another is chosen: `Deactivate` for Compass/Motor Calib, which
    /// stops a running calibration; the others are `IActivate` only, hidden with a number or
    /// text being typed into read.
    pub(crate) fn extra_setup_deactivate(&mut self, class: &str) {
        match class {
            "ConfigHWParachute" => self.extra.parachute.hide(Instant::now()),
            "ConfigHWOSD" => self.extra.osd.hide(),
            "ConfigGPSOrder" => self.extra.gps_order.hide(),
            "ConfigHWIDs" => self.extra.hw_ids.hide(),
            // C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:32-48
            "ConfigCompassMot" => self.extra.compass_mot.deactivate(&self.telemetry),
            "ConfigInitialParams" => self.extra.initial_params.hide(),
            // C#: GCSViews/ConfigurationView/ConfigFFT.cs:66-69
            "ConfigFFT" => self.extra.fft.hide(),
            // C#: GCSViews/ConfigurationView/ConfigDroneCAN.cs:680-684
            "ConfigDroneCAN" => self.extra.dronecan.deactivate(),
            "Sikradio" => self.extra.sikradio.hide(),
            _ => {}
        }
    }

    /// The page, when one of the six is showing.
    pub(crate) fn extra_setup_page(
        &self,
        class: &str,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        let pages = &self.extra;
        let focus = &self.extra_focus;
        match class {
            "ConfigHWParachute" => parachute::page(&pages.parachute, focus, window, cx),
            "ConfigHWOSD" => osd::page(&pages.osd, cx),
            "ConfigGPSOrder" => gps_order::page(&pages.gps_order, cx),
            "ConfigHWIDs" => hw_ids::page(&pages.hw_ids, cx),
            "ConfigCompassMot" => compass_mot::page(&pages.compass_mot, cx),
            "ConfigInitialParams" => initial_params::page(&pages.initial_params, focus, window, cx),
            "ConfigFFT" => fft::page(&pages.fft, &focus.number, &focus.fft_track, window, cx),
            "ConfigDroneCAN" => super::dronecan::page(&pages.dronecan, cx),
            "Sikradio" => sikradio::page(&pages.sikradio, &focus.sikradio, window, cx),
            _ => div().into_any_element(),
        }
    }

    /// Once a frame: each page object disposed with its screen, a box the focus has left read,
    /// the timers, the calibration's statuses, and the writes.
    pub(crate) fn extra_setup_tick(&mut self, view: &TelemetryView, window: &Window) {
        let on_setup = self.screen == crate::Screen::Setup;
        self.sikradio_tick(on_setup);
        // `logreadmode`, for the proximity window's clock.
        let replaying = self.fly_data.playback.playing();
        let now = Instant::now();
        let number_focused = self.extra_focus.number.is_focused(window);
        let fft_number_focused = self.extra_focus.fft_number.is_focused(window);
        let text_focused = self.extra_focus.text.is_focused(window);
        let spectrogram_focus = &self.extra_focus.spectrogram;
        let spectrogram_focused = (
            spectrogram_focus.number.is_focused(window),
            spectrogram_focus.text.is_focused(window),
        );
        let proxy_focus = &self.extra_focus.support_proxy;
        let proxy_focused = (
            proxy_focus.number.is_focused(window),
            proxy_focus.text.is_focused(window),
        );
        let telemetry = &self.telemetry;
        // The CoT output's vehicles, as `cs` gives them in the user's units.
        let cot_vehicles: Vec<_> = if self.extra.cot_output.running() {
            let units = self.planner.units();
            telemetry
                .vehicles()
                .into_iter()
                .filter_map(|id| {
                    telemetry
                        .vehicle_state(id)
                        .map(|state| (id, super::cot_output::Position::of(&state, &units)))
                })
                .collect()
        } else {
            Vec::new()
        };
        let pages = &mut self.extra;
        pages
            .parachute
            .tick(telemetry, view, on_setup, number_focused, now);
        pages.osd.tick(telemetry, view, on_setup);
        pages.gps_order.tick(telemetry, view, on_setup);
        pages.hw_ids.tick(view, on_setup);
        pages.compass_mot.tick(telemetry, view, on_setup);
        pages
            .initial_params
            .tick(telemetry, view, on_setup, text_focused);
        // The FFT window's run, when it ended in what the C# throws, is a status line too.
        let fft_error = pages.fft.tick(telemetry, view, on_setup, number_focused);
        if let Some(ui) = pages.fft.window.as_mut()
            && ui.editing.is_some()
            && !fft_number_focused
        {
            ui.leave();
        }
        // The MAVLink Inspector's subscriptions and timers, while it is open.
        pages.inspector.tick(telemetry, now);
        // The proximity window's vehicle and clock; the key store, read at the first frame, and
        // the signing window's timer.
        let banner = telemetry.firmware_banner();
        pages.proximity.tick(
            view.vehicle,
            |id| telemetry.vehicle_state(id),
            |state| {
                super::flight_modes::firmware_of(state.autopilot, state.vehicle_type, banner)
                    == super::flight_modes::Firmware::ArduCopter2
            },
            replaying,
        );
        let link = pages
            .auth_keys
            .window
            .is_some()
            .then(|| telemetry.send_handle())
            .flatten();
        pages.auth_keys.tick(link.as_ref(), now);
        // The spectrogram's reading and drawing, and the Support Proxy's stream and mirror: what
        // the C# throws or boxes, on the status line.
        let spectrogram_error = pages
            .spectrogram
            .tick(spectrogram_focused.0, spectrogram_focused.1);
        let proxy_status = pages.support_proxy.tick(
            telemetry,
            &mut self.persisted,
            proxy_focused.0,
            proxy_focused.1,
        );
        // The Mavlink Mirror's streams and mirrors, and the NMEA output's stream and thread.
        let mirror_status = pages.mavlink_mirror.tick(telemetry);
        let nmea_status = pages.nmea_output.tick(view.state.as_deref());
        let cot_status = pages.cot_output.tick(cot_vehicles);
        let paramgen_status = pages.param_gen.tick();
        // The link errors the C# boxes go on the status line instead (the owner's ruling); the
        // page never draws them, since they leave its queue in the tick before the frame.
        let mut status = None;
        while let Some(message) = pages.osd.message().filter(|m| link_error(m)) {
            status = Some(status_words(message));
            pages.osd.dismiss_message();
        }
        while let Some(message) = pages.gps_order.message().filter(|m| link_error(m)) {
            status = Some(status_words(message));
            pages.gps_order.dismiss_message();
        }
        while let Some(message) = pages.compass_mot.message().filter(|m| link_error(m)) {
            status = Some(status_words(message));
            pages.compass_mot.dismiss_message();
        }
        while let Some(message) = pages.parachute.message().filter(|m| link_error(m)) {
            status = Some(status_words(message));
            pages.parachute.dismiss_message();
        }
        while let Some(message) = pages.initial_params.message().filter(|m| link_error(m)) {
            status = Some(status_words(message));
            pages.initial_params.dismiss_message();
        }
        while let Some(message) = pages.fft.message().filter(|m| link_error(m)) {
            status = Some(status_words(message));
            pages.fft.dismiss_message();
        }
        if fft_error.is_some() {
            status = fft_error;
        }
        if spectrogram_error.is_some() {
            status = spectrogram_error;
        }
        if proxy_status.is_some() {
            status = proxy_status;
        }
        if mirror_status.is_some() {
            status = mirror_status;
        }
        if nmea_status.is_some() {
            status = nmea_status;
        }
        if cot_status.is_some() {
            status = cot_status;
        }
        if paramgen_status.is_some() {
            status = paramgen_status;
        }
        if status.is_some() {
            self.file_status = status;
        }
        // DroneCAN/UAVCAN: its bus, its node, and the port SLCAN takes from the link.
        self.dronecan_tick(view);
    }

    /// The box, question or dialog one of the six is showing, over the whole window.
    pub(crate) fn extra_setup_overlay(
        &self,
        window: &Window,
        cx: &mut Context<Self>,
    ) -> Option<AnyElement> {
        let pages = &self.extra;
        let focus = &self.extra_focus;
        fft::overlay(&pages.fft, &focus.fft_number, &focus.fft_prompt, window, cx)
            .or_else(|| {
                mavlink_inspector::overlay(&pages.inspector, &focus.inspector_prompt, window, cx)
            })
            .or_else(|| {
                warnings_manager::overlay(
                    &pages.warnings_manager,
                    &focus.warnings_number,
                    &focus.warnings_text,
                    window,
                    cx,
                )
            })
            .or_else(|| proximity::overlay(&pages.proximity, &focus.proximity, window, cx))
            .or_else(|| {
                auth_keys::overlay(
                    &pages.auth_keys,
                    &focus.auth_keys,
                    &focus.auth_keys_prompt,
                    window,
                    cx,
                )
            })
            .or_else(|| spectrogram::overlay(&pages.spectrogram, &focus.spectrogram, window, cx))
            .or_else(|| {
                support_proxy::overlay(&pages.support_proxy, &focus.support_proxy, window, cx)
            })
            .or_else(|| parachute::overlay(&pages.parachute, window, cx))
            .or_else(|| osd::overlay(&pages.osd, window, cx))
            .or_else(|| gps_order::overlay(&pages.gps_order, window, cx))
            .or_else(|| compass_mot::overlay(&pages.compass_mot, window, cx))
            .or_else(|| initial_params::overlay(&pages.initial_params, window, cx))
            .or_else(|| super::dronecan::overlay(&pages.dronecan, window, cx))
            .or_else(|| {
                super::dronecan_inspector::overlay(
                    pages.dronecan.inspector.as_ref(),
                    &focus.dronecan_prompt,
                    window,
                    cx,
                )
            })
            .or_else(|| sikradio::overlay(&pages.sikradio, &focus.sikradio, window, cx))
            .or_else(|| {
                mavlink_mirror::overlay(&pages.mavlink_mirror, &focus.mirror_cell, window, cx)
            })
            .or_else(|| nmea_output::overlay(&pages.nmea_output, &focus.nmea_prompt, window, cx))
            .or_else(|| {
                super::cot_output::overlay(
                    &pages.cot_output,
                    &focus.cot_typing,
                    &focus.cot_prompt,
                    window,
                    cx,
                )
            })
            .or_else(|| param_gen::overlay(&pages.param_gen, window, cx))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::optional::{error, plain};

    /// The C#'s link-failure boxes are status lines; questions, validations and reports keep
    /// their boxes.
    #[test]
    fn link_failures_are_status_lines_and_the_rest_are_boxes() {
        assert!(link_error(&plain(osd::FAILED)));
        assert!(link_error(&error(compass_mot::NEEDS_AC_3_2)));
        assert!(link_error(&super::super::rangefinder::unhandled(
            "RC10_FUNCTION"
        )));
        assert!(link_error(&Message {
            title: gps_order::FAILED_ACTIVATE,
            text: gps_order::ERROR_TEXT.to_owned(),
        }));
        assert!(!link_error(&Message {
            title: initial_params::REFUSAL_TITLE,
            text: initial_params::PROP_TOO_SMALL.to_owned(),
        }));
        assert!(!link_error(&Message {
            title: initial_params::DONE_TITLE,
            text: initial_params::DONE.to_owned(),
        }));
        assert_eq!(status_words(&plain(osd::FAILED)), osd::FAILED);
        let refused = Message {
            title: initial_params::REFUSAL_TITLE,
            text: initial_params::PROP_TOO_SMALL.to_owned(),
        };
        let mut queue = VecDeque::from([
            error("Set A Failed"),
            refused.clone(),
            error("Set B\nFailed"),
        ]);
        assert_eq!(
            take_link_errors(&mut queue, link_error).as_deref(),
            Some("Set B Failed"),
            "the last failure's words, on one line"
        );
        assert_eq!(queue, [refused]);
        assert_eq!(take_link_errors(&mut queue, link_error), None);
        assert_eq!(
            status_words(&Message {
                title: gps_order::FAILED_ACTIVATE,
                text: gps_order::ERROR_TEXT.to_owned(),
            }),
            gps_order::FAILED_ACTIVATE
        );
    }

    /// The seven pages are SETUP's, so their boxes and the FFT window - which FFT Setup's FFT and
    /// Advanced's FFT open - are drawn over the SETUP screen, and over it only.
    /// `// C#: GCSViews/InitialSetup.cs:237-342`
    #[test]
    fn the_boxes_and_the_fft_window_draw_over_setup() {
        let main = include_str!("../main.rs");
        let setup = main
            .find("Screen::Setup => probe::measured(\"setup-body\"")
            .expect("the SETUP screen");
        let config = main
            .find("Screen::Config => probe::measured(\"config-body\"")
            .expect("the CONFIG screen");
        assert!(setup < config);
        let overlay = ".children(self.extra_setup_overlay(window, cx))";
        assert_eq!(main.matches(overlay).count(), 1);
        let at = main.find(overlay).expect("drawn");
        assert!(setup < at && at < config, "drawn over SETUP");
    }
}
