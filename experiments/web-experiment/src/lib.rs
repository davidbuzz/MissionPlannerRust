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

//! The browser experiment: the planner's own HUD (crates/mp-gui/src/hud.rs, linked in unchanged)
//! in a web page, drawn from a live vehicle.
//!
//! The vehicle's bytes come from `globalThis.mpLink` (www/link.js), the page's stand-in for the
//! desktop's link: ArduPilot's WebAssembly SITL running in the same page, or a WebSocket to a
//! vehicle. They go through the path the desktop's HUD takes, `FrameDecoder`, then
//! `VehicleRegistry`, then `hud::live_inputs`, `hud::scene` and `hud::paint`, as
//! crates/mp-gui/src/hud/golden.rs plays a tlog through it.

use std::borrow::Cow;
use std::cell::RefCell;
use std::time::Duration;

use gpui::{App, Context, Window, WindowOptions, canvas, div, prelude::*, px, rgb};
use mp_mavlink::Dialect;
use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::{DIALECT, Heartbeat, MavMessage, RequestDataStream};
use mp_vehicle::{DateTime, StreamRates, VehicleId, VehicleRegistry};
use wasm_bindgen::prelude::*;
use web_time::Instant;

#[allow(dead_code, unused_imports)]
mod hud;
#[allow(dead_code, unused_imports)]
mod pictures;
#[allow(dead_code)]
mod standins;

use standins::{config, facts, fly, probe, quick, ui};

#[wasm_bindgen]
extern "C" {
    /// www/link.js: the bytes the vehicle sent since the last call (empty when none).
    #[wasm_bindgen(js_namespace = mpLink, js_name = take)]
    fn link_take() -> Vec<u8>;
    /// www/link.js: bytes for the vehicle.
    #[wasm_bindgen(js_namespace = mpLink, js_name = send)]
    fn link_send(bytes: &[u8]);
    /// www/link.js: what the link is doing, in words.
    #[wasm_bindgen(js_namespace = mpLink, js_name = status)]
    fn link_status() -> String;
}

thread_local! {
    /// What the last frame drew and what the link has done, for [`facts`].
    static FACTS: RefCell<Vec<(String, String)>> = const { RefCell::new(Vec::new()) };
}

/// The page's facts as a JSON object, as the desktop writes `MP_FACTS` for its GUI scripts: the
/// HUD's own `readout_facts` and `health_facts` from the scene last painted, and the link's.
/// check/check.js reads them.
#[wasm_bindgen]
pub fn facts() -> String {
    let quote = |text: &str| {
        let mut out = String::from('"');
        for c in text.chars() {
            match c {
                '"' => out.push_str("\\\""),
                '\\' => out.push_str("\\\\"),
                c if u32::from(c) < 0x20 => out.push_str(&format!("\\u{:04x}", u32::from(c))),
                c => out.push(c),
            }
        }
        out.push('"');
        out
    };
    FACTS.with(|facts| {
        let fields: Vec<String> = facts
            .borrow()
            .iter()
            .map(|(key, value)| format!("{}:{}", quote(key), quote(value)))
            .collect();
        format!("{{{}}}", fields.join(","))
    })
}

/// How often the page drains the link: the desktop's link thread reads as bytes arrive; here a
/// timer on gpui's executor stands in for it.
const POLL: Duration = Duration::from_millis(20);
/// Mission Planner's own ids on the link (`MAVLinkInterface.gcssysid`, `MAV_COMP_ID_MISSIONPLANNER`).
const GCS_SYSID: u8 = 255;
const GCS_COMPID: u8 = 190;
/// `MAV_TYPE_GCS`, `MAV_AUTOPILOT_INVALID`.
const MAV_TYPE_GCS: u8 = 6;
const MAV_AUTOPILOT_INVALID: u8 = 8;
/// The streams `UpdateCurrentSettings` asks for, ids as crates/mp-link/src/current_settings.rs
/// names them, and the rates from `StreamRates::default()`.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4638-4652`
fn stream_requests(rates: StreamRates) -> [(u8, i32); 7] {
    [
        (2, rates.status),    // EXTENDED_STATUS
        (6, rates.position),  // POSITION
        (10, rates.attitude), // EXTRA1
        (11, rates.attitude), // EXTRA2
        (12, rates.sensors),  // EXTRA3
        (1, rates.sensors),   // RAW_SENSORS
        (3, rates.rc),        // RC_CHANNELS
    ]
}

struct Planner {
    decoder: FrameDecoder,
    registry: VehicleRegistry,
    /// The vehicle the HUD shows: the first to send ATTITUDE, as golden.rs picks.
    vehicle: Option<VehicleId>,
    timing: hud::Timing,
    frames: u64,
    seq: u8,
    last_heartbeat: Option<Instant>,
}

impl Planner {
    fn new(cx: &mut Context<Self>) -> Self {
        cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor().timer(POLL).await;
                if this.update(cx, |planner, cx| planner.poll(cx)).is_err() {
                    break;
                }
            }
        })
        .detach();
        Self {
            decoder: FrameDecoder::new(),
            registry: VehicleRegistry::new(),
            vehicle: None,
            timing: hud::Timing::default(),
            frames: 0,
            seq: 0,
            last_heartbeat: None,
        }
    }

    fn poll(&mut self, cx: &mut Context<Self>) {
        let bytes = link_take();
        if !bytes.is_empty() {
            let sent = now_datetime();
            let Self { decoder, registry, vehicle, frames, .. } = self;
            decoder.push_and_drain(&bytes, &DIALECT, |frame| {
                *frames += 1;
                let Some(message) = MavMessage::decode(frame.msgid, frame.payload) else {
                    return;
                };
                let id = registry.apply_at(frame.sysid, frame.compid, frame.seq, &message, sent);
                if matches!(message, MavMessage::Attitude(_)) && vehicle.is_none() {
                    *vehicle = Some(id);
                }
            });
            cx.notify();
        }
        // A GCS heartbeat a second, and the streams asked for with it until the vehicle sends
        // attitude: the desktop's link asks again every 30 s (`streams_due`); once is the least
        // a fresh SITL needs.
        let now = Instant::now();
        if self
            .last_heartbeat
            .is_none_or(|last| now.duration_since(last) >= Duration::from_secs(1))
        {
            self.last_heartbeat = Some(now);
            self.send(&MavMessage::Heartbeat(Heartbeat {
                custom_mode: 0,
                r#type: MAV_TYPE_GCS,
                autopilot: MAV_AUTOPILOT_INVALID,
                base_mode: 0,
                system_status: 0,
                mavlink_version: 3,
            }));
            if self.vehicle.is_none() {
                for (stream, hz) in stream_requests(StreamRates::default()) {
                    for target in self.registry.ids() {
                        self.send(&MavMessage::RequestDataStream(RequestDataStream {
                            req_message_rate: u16::try_from(hz).unwrap_or(0),
                            target_system: target.sysid,
                            target_component: target.compid,
                            req_stream_id: stream,
                            start_stop: 1,
                        }));
                    }
                }
            }
        }
    }

    fn send(&mut self, message: &MavMessage) {
        let mut payload = [0u8; 255];
        let length = message.encode(&mut payload);
        let id = message.id();
        let Some(crc_extra) = DIALECT.crc_extra(id) else {
            return;
        };
        let mut frame = [0u8; 280];
        let payload = payload.get(..length).unwrap_or(&[]);
        if let Ok(total) = mp_mavlink::frame::encode_v2(
            &mut frame, self.seq, GCS_SYSID, GCS_COMPID, id, payload, crc_extra, 0,
        ) {
            self.seq = self.seq.wrapping_add(1);
            link_send(frame.get(..total).unwrap_or(&[]));
        }
    }
}

/// Now, as the registry's clock takes it: `DateTime` from the page's wall clock.
fn now_datetime() -> DateTime {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let micros = (js_sys::Date::now() * 1000.0) as u64;
    DateTime::from_tlog_micros(micros).unwrap_or(DateTime::MIN)
}

/// The HUD's clock, the pilot's local time as the desktop shows it.
fn clock_text() -> String {
    let date = js_sys::Date::new_0();
    format!(
        "{:02}:{:02}:{:02}",
        date.get_hours(),
        date.get_minutes(),
        date.get_seconds()
    )
}

impl Render for Planner {
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        let inputs = match self.vehicle.and_then(|id| self.registry.working(id)) {
            Some(state) => {
                let state = *state;
                hud::live_inputs(&state, &mut self.timing, Instant::now(), clock_text(), &[])
            }
            None => hud::HudInputs::default(),
        };
        let status = format!("{} · {} frames", link_status(), self.frames);
        let mut link_facts = vec![
            ("link.status".to_owned(), link_status()),
            ("link.frames".to_owned(), self.frames.to_string()),
        ];
        if let Some(state) = self.vehicle.and_then(|id| self.registry.working(id)) {
            let mode = mp_vehicle::flight_mode_name(state.vehicle_type, state.custom_mode)
                .map_or_else(|| format!("mode {}", state.custom_mode), ToOwned::to_owned);
            link_facts.push(("vehicle.mode".to_owned(), mode));
            link_facts.push(("vehicle.armed".to_owned(), state.armed.to_string()));
        }
        div()
            .size_full()
            .flex()
            .flex_col()
            .bg(rgb(0x000000))
            .child(
                div()
                    .id("status")
                    .h(px(28.))
                    .px_2()
                    .flex()
                    .items_center()
                    .gap_4()
                    .bg(rgb(0x262a30))
                    .text_color(rgb(0xe6edf3))
                    .text_sm()
                    .child("MissionPlannerRust in the browser")
                    .child(div().text_color(rgb(0x8b949e)).child(status)),
            )
            .child(
                // Clipped to its box, as the flight screen's HUD panel is: the sky and ground
                // are drawn past the edges and cut there.
                div().flex_1().relative().overflow_hidden().child(
                    canvas(
                        |_bounds, _window, _cx| (),
                        move |bounds, (), window, cx| {
                            let scene = hud::scene(
                                &inputs,
                                f32::from(bounds.size.width),
                                f32::from(bounds.size.height),
                            );
                            hud::paint(&scene, bounds, window, cx);
                            let drawn = hud::readout_facts(&scene)
                                .into_iter()
                                .chain(hud::health_facts(&scene))
                                .map(|(key, value)| (key.to_owned(), value));
                            let all = link_facts.iter().cloned().chain(drawn).collect();
                            FACTS.with(|facts| *facts.borrow_mut() = all);
                        },
                    )
                    .size_full(),
                ),
            )
    }
}

/// A web page has no system fonts for gpui to find, so the page brings its own, as Zed's web
/// examples do (crates/gpui/examples/example_support/fonts.rs).
fn load_fonts(cx: &App) -> bool {
    let fonts = [
        Cow::Borrowed(include_bytes!("../fonts/IBMPlexSans-Regular.ttf").as_slice()),
        Cow::Borrowed(include_bytes!("../fonts/IBMPlexSans-SemiBold.ttf").as_slice()),
    ];
    if let Err(err) = cx.text_system().add_fonts(fonts.into()) {
        web_sys::console::error_1(&format!("could not load the fonts: {err:#}").into());
        return false;
    }
    true
}

#[wasm_bindgen(start)]
pub fn start() {
    gpui_platform::web_init();
    gpui_platform::application().run(|cx: &mut App| {
        if !load_fonts(cx) {
            return;
        }
        if let Err(err) = cx.open_window(WindowOptions::default(), |_window, cx| cx.new(Planner::new)) {
            web_sys::console::error_1(&format!("could not open the window: {err:?}").into());
        }
        cx.activate(true);
    });
}
