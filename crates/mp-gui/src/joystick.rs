//! Flying from a joystick.
//!
//! The dangerous screen. Every other panel reads the vehicle or asks it to do one thing; this one
//! takes over the pilot's sticks, and the failure that matters is not a wrong number on screen but
//! a stick that stops reporting while the vehicle keeps flying the last position it heard.
//!
//! The safety is in `mp_input`, which is a separate crate with its own tests, precisely so the
//! decision about when to hand control back is not tangled up with how a panel is laid out. This
//! module polls the device, feeds the failsafe, and sends whatever the failsafe says to send.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_input::{Device, Failsafe, Mapping, Reading};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, field, panel, progress, theme};

/// Everything the joystick screen needs to keep between frames.
#[derive(Debug)]
pub struct Sticks {
    /// The devices found the last time anybody looked.
    devices: Vec<Device>,
    /// The device being used, by path.
    chosen: Option<String>,
    /// The open device, if one is open.
    #[cfg(target_os = "linux")]
    open: Option<mp_input::linux::Joystick>,
    /// The last positions read.
    reading: Reading,
    /// How stick positions become channels.
    mapping: Mapping,
    /// When to stop believing the sticks.
    failsafe: Failsafe,
    /// The last thing that happened, shown so a refusal is never silent.
    status: Option<String>,
}

impl Default for Sticks {
    fn default() -> Self {
        Self::new()
    }
}

impl Sticks {
    /// No device, overrides off.
    #[must_use]
    pub fn new() -> Self {
        Self {
            devices: Vec::new(),
            chosen: None,
            #[cfg(target_os = "linux")]
            open: None,
            reading: Reading::default(),
            mapping: Mapping::gamepad(),
            failsafe: Failsafe::default(),
            status: None,
        }
    }

    /// Looks for devices.
    pub fn refresh(&mut self) {
        #[cfg(target_os = "linux")]
        {
            self.devices = mp_input::linux::devices();
        }
        self.status = Some(match self.devices.len() {
            0 => "no joystick found".to_owned(),
            1 => "1 joystick".to_owned(),
            count => format!("{count} joysticks"),
        });
    }

    /// Opens a device, replacing whichever was open.
    pub fn choose(&mut self, id: &str) {
        // Overrides go off across a device change, and off means a release is sent. Carrying an
        // enabled state across is the obvious convenience and exactly wrong: the new device's
        // sticks are wherever they happen to be, and adopting them without the operator saying so
        // hands the aircraft a stick position nobody chose.
        self.failsafe.set_enabled(false);
        self.reading = Reading::default();
        #[cfg(target_os = "linux")]
        {
            match mp_input::linux::Joystick::open(id) {
                Ok(device) => {
                    self.open = Some(device);
                    self.chosen = Some(id.to_owned());
                    self.status = Some(format!("using {id}"));
                }
                Err(err) => {
                    self.open = None;
                    self.chosen = None;
                    // Permissions are the usual cause and the usual thing nobody guesses, so it
                    // is named rather than left as "permission denied".
                    let hint = if err.kind() == std::io::ErrorKind::PermissionDenied {
                        " - add yourself to the 'input' group and log in again"
                    } else {
                        ""
                    };
                    self.status = Some(format!("could not open {id}: {err}{hint}"));
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = id;
            self.status = Some("joysticks are only supported on Linux so far".to_owned());
        }
    }

    /// Turns overrides on or off.
    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.is_open() {
            self.status = Some("choose a joystick first".to_owned());
            return;
        }
        self.failsafe.set_enabled(enabled);
        self.status = Some(if enabled {
            "sticks are flying the vehicle".to_owned()
        } else {
            "control handed back to the transmitter".to_owned()
        });
    }

    /// Whether a device is open.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        #[cfg(target_os = "linux")]
        {
            self.open.is_some()
        }
        #[cfg(not(target_os = "linux"))]
        {
            false
        }
    }

    /// Whether overrides are switched on.
    #[must_use]
    pub const fn is_enabled(&self) -> bool {
        self.failsafe.is_enabled()
    }

    /// The current stick positions.
    #[must_use]
    pub const fn reading(&self) -> &Reading {
        &self.reading
    }

    /// The last thing that happened.
    #[must_use]
    pub fn status(&self) -> Option<&str> {
        self.status.as_deref()
    }

    /// Reads the device and returns what to send, if anything.
    ///
    /// Called once per frame. The return is `None` most of the time - once control has been handed
    /// back there is nothing more to say, and a ground station that keeps sending releases stops
    /// the vehicle's own failsafe timer from ever running.
    pub fn poll(&mut self) -> Option<[u16; 18]> {
        let now = std::time::Instant::now();
        #[cfg(target_os = "linux")]
        {
            if let Some(device) = self.open.as_mut() {
                match device.poll() {
                    // Alive means the device answered, not that anything moved. Feeding the
                    // failsafe on movement was a flight-safety bug: the joystick API is
                    // edge-triggered, so a pilot holding a stick steady emits nothing, and the
                    // failsafe would hand control to a transmitter that this feature assumes is
                    // not there. `STICK_TIMEOUT` now guards what it was written to guard - this
                    // function not running at all.
                    mp_input::linux::Poll::Alive => {
                        self.reading = device.reading().clone();
                        self.failsafe.fed(now);
                    }
                    mp_input::linux::Poll::Gone => {
                        // Released here rather than left to time out. The timeout is for a lapse
                        // that cannot be detected; this one has been, and sending a stale stick
                        // position for another 200 ms after the code already knows the device is
                        // gone is not a failsafe, it is a delay.
                        self.failsafe.release_now();
                        // Dropped rather than kept for a retry. A half-open device that might come
                        // back is a device somebody will believe is working.
                        self.open = None;
                        self.chosen = None;
                        // The positions go with it, so the panel does not keep showing axis values
                        // for a device that is not there.
                        self.reading = Reading::default();
                        self.status =
                            Some("the joystick was unplugged - control handed back".to_owned());
                    }
                }
            }
        }
        let channels = self.mapping.channels(&self.reading);
        self.failsafe.outgoing(now, channels).map(|out| out.0)
    }

    /// Notes that a frame the link refused is not a frame that was delivered.
    ///
    /// The transport is unacknowledged, so "delivered" can never be certain - but a send the link
    /// actively refused certainly was not, and counting it against the release budget is how the
    /// one frame that hands an aircraft back to its pilot gets silently dropped.
    pub fn send_failed(&mut self) {
        self.failsafe.retry_release();
    }
}

/// The joystick panel.
pub fn panel_for(
    view: &TelemetryView,
    sticks: &Sticks,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let has_vehicle = view.vehicle.is_some();
    let enabled = sticks.is_enabled();
    let open = sticks.is_open();

    let mut devices = div().flex().flex_col().gap_1();
    for device in &sticks.devices {
        let id = device.id.clone();
        let chosen = sticks.chosen.as_deref() == Some(device.id.as_str());
        devices = devices.child(
            crate::probe::measured(format!("joystick-{}", device.name), div())
                .id(gpui::SharedString::from(format!("js-{}", device.id)))
                .px_2()
                .py(px(2.0))
                .rounded_sm()
                .border_1()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BORDER }))
                .text_xs()
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::TEXT }))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(format!("{}  ({})", device.name, device.id))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.sticks.choose(&id);
                    cx.notify();
                })),
        );
    }

    // Live axis positions, so a person can push a stick and see which number moves. That is how a
    // device gets mapped in practice, and it is why the numbers are here rather than a picture of
    // a gamepad that will not match theirs.
    let mut axes = div().flex().flex_col().gap_1();
    for (index, value) in sticks.reading().axes.iter().enumerate() {
        let fraction = (value + 1.0) * 0.5;
        axes = axes.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .child(
                    div()
                        .w(px(52.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(format!("axis {index}")),
                )
                .child(
                    div()
                        .flex_1()
                        .min_w(px(0.0))
                        .child(progress(fraction, theme::ACCENT)),
                )
                .child(
                    div()
                        .w(px(44.0))
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(format!("{value:+.2}")),
                ),
        );
    }

    let pressed: Vec<String> = sticks
        .reading()
        .buttons
        .iter()
        .enumerate()
        .filter(|(_, down)| **down)
        .map(|(index, _)| index.to_string())
        .collect();

    panel(
        "joystick",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(
                div()
                    .flex()
                    .items_center()
                    .gap_2()
                    .child(action(
                        "joystick-refresh",
                        "find joysticks",
                        theme::ACCENT,
                        true,
                        cx.listener(|this, _event: &(), _window, cx| {
                            this.sticks.refresh();
                            cx.notify();
                        }),
                    ))
                    .child(action(
                        "joystick-enable",
                        if enabled {
                            "stop flying"
                        } else {
                            "fly with sticks"
                        },
                        if enabled { theme::ALERT } else { theme::WARN },
                        open && has_vehicle,
                        cx.listener(move |this, _event: &(), _window, cx| {
                            let now = this.sticks.is_enabled();
                            this.sticks.set_enabled(!now);
                            cx.notify();
                        }),
                    ))
                    .child(field(
                        "state",
                        if enabled {
                            "sticks have control".to_owned()
                        } else {
                            "transmitter has control".to_owned()
                        },
                        if enabled { theme::ALERT } else { theme::TEXT },
                    )),
            )
            .children(sticks.status().map(|status| {
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(status.to_owned())
            }))
            .child(devices)
            .children((!sticks.reading().axes.is_empty()).then_some(axes))
            .children((!pressed.is_empty()).then(|| {
                div()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(format!("buttons down: {}", pressed.join(", ")))
            })),
    )
    .into_any_element()
}
