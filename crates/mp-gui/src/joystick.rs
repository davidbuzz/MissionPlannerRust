//! Flying from a joystick.
//!
//! The dangerous screen. Every other panel reads the vehicle or asks it to do one thing; this one
//! takes over the pilot's sticks, and the failure that matters is not a wrong number on screen but
//! a stick that stops reporting while the vehicle keeps flying the last position it heard.
//!
//! The safety is in `mp_input`, which is a separate crate with its own tests, precisely so the
//! decision about when to hand control back is not tangled up with how a panel is laid out. So is
//! the speed: the device is read and the frames are sent on threads of that crate's own
//! (`mp_input::reader`), because D15's five milliseconds from stick to wire cannot be met from a
//! screen that sends what it polled once a frame. This module chooses the device, switches control
//! on and off, keeps the reader pointed at the vehicle being flown, and shows what the sticks are
//! doing.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_input::{Device, Mapping, Reading};
use mp_link::LinkSender;
use mp_vehicle::VehicleId;

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, field, panel, progress, theme};

/// Where frames go: a handle that sends on the link, and the vehicle to address them to.
///
/// `None` while there is no vehicle. The reader's send thread reads this on every frame, so a
/// vehicle chosen on screen is the vehicle the next frame goes to, and a link that has closed is
/// a frame that is refused - which for a release means it is retried, not forgotten.
pub type Target = Option<(LinkSender, VehicleId)>;

/// Everything the joystick screen needs to keep between frames.
#[derive(Debug)]
pub struct Sticks {
    /// The devices found the last time anybody looked.
    devices: Vec<Device>,
    /// The device being used, by path.
    chosen: Option<String>,
    /// The open device and its threads, if one is open.
    #[cfg(target_os = "linux")]
    reader: Option<mp_input::StickReader>,
    /// How stick positions become channels.
    mapping: Mapping,
    /// Where the send thread puts its frames. Shared with it; set from the screen every frame.
    target: Arc<Mutex<Target>>,
    /// Frames the link accepted, for the status line and the facts.
    sent: Arc<AtomicU64>,
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
            reader: None,
            mapping: Mapping::gamepad(),
            target: Arc::new(Mutex::new(None)),
            sent: Arc::new(AtomicU64::new(0)),
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
    ///
    /// Overrides do not carry across a device change: the new reader starts switched off, and
    /// the old one, if it was flying, sends its release from its own thread as it is dropped.
    /// Carrying an enabled state across is the obvious convenience and exactly wrong - the new
    /// device's sticks are wherever they happen to be, and adopting them without the operator
    /// saying so hands the aircraft a stick position nobody chose.
    pub fn choose(&mut self, id: &str) {
        #[cfg(target_os = "linux")]
        {
            let target = Arc::clone(&self.target);
            let sent = Arc::clone(&self.sent);
            // Runs on the reader's send thread. It must not block, and it must say whether the
            // link took the frame: for a release that is the difference between an aircraft
            // handed back and one still being flown by a stick nobody is holding.
            let sink = move |frame: &mp_input::Frame| -> bool {
                let guard = target.lock().unwrap_or_else(PoisonError::into_inner);
                let Some((sender, vehicle)) = guard.as_ref() else {
                    // No vehicle to send to. Not delivered, and said so, rather than swallowed.
                    return false;
                };
                let accepted =
                    sender.send(&mp_link::commands::rc_override(*vehicle, frame.channels.0));
                if accepted {
                    sent.fetch_add(1, Ordering::Relaxed);
                }
                accepted
            };
            match mp_input::StickReader::open(id, self.mapping.clone(), sink) {
                Ok(reader) => {
                    self.reader = Some(reader);
                    self.chosen = Some(id.to_owned());
                    self.status = Some(format!("using {id}"));
                }
                Err(err) => {
                    self.reader = None;
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
        #[cfg(target_os = "linux")]
        {
            let Some(reader) = self.reader.as_ref() else {
                self.status = Some("choose a joystick first".to_owned());
                return;
            };
            // Refused for a device that has gone: switching on sticks that are not there would
            // fly the aircraft on the last position they reported.
            self.status = Some(if !reader.set_enabled(enabled) {
                "the joystick is gone - control stays with the transmitter".to_owned()
            } else if enabled {
                "sticks are flying the vehicle".to_owned()
            } else {
                "control handed back to the transmitter".to_owned()
            });
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = enabled;
            self.status = Some("joysticks are only supported on Linux so far".to_owned());
        }
    }

    /// Keeps the reader pointed at the vehicle, and notices a device that has gone.
    ///
    /// Called once a frame. The reader does the flying on its own threads; what it cannot know
    /// is which vehicle the operator has chosen since the last frame, and what the screen cannot
    /// know without asking is that the device was unplugged - by which time the release has
    /// already been sent, from the reader's thread, so all that is left is to say so and let the
    /// reader go. A half-open device that might come back is a device somebody will believe is
    /// working.
    pub fn tick(&mut self, target: Target) {
        *self.target.lock().unwrap_or_else(PoisonError::into_inner) = target;
        #[cfg(target_os = "linux")]
        if self
            .reader
            .as_ref()
            .is_some_and(|reader| reader.liveness() == mp_input::Poll::Gone)
        {
            self.reader = None;
            self.chosen = None;
            self.status = Some("the joystick was unplugged - control handed back".to_owned());
        }
    }

    /// Whether a device is open.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        #[cfg(target_os = "linux")]
        {
            self.reader.is_some()
        }
        #[cfg(not(target_os = "linux"))]
        {
            false
        }
    }

    /// Whether overrides are switched on. Goes false by itself when the device is unplugged.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        #[cfg(target_os = "linux")]
        {
            self.reader
                .as_ref()
                .is_some_and(mp_input::StickReader::is_enabled)
        }
        #[cfg(not(target_os = "linux"))]
        {
            false
        }
    }

    /// The current stick positions, as the reader last saw them.
    #[must_use]
    pub fn reading(&self) -> Reading {
        #[cfg(target_os = "linux")]
        {
            self.reader
                .as_ref()
                .map(mp_input::StickReader::reading)
                .unwrap_or_default()
        }
        #[cfg(not(target_os = "linux"))]
        {
            Reading::default()
        }
    }

    /// How many frames the link has accepted since the application started.
    #[must_use]
    pub fn sent(&self) -> u64 {
        self.sent.load(Ordering::Relaxed)
    }

    /// Stick-to-link latency so far, as (p50, p99), once anything has been measured.
    #[must_use]
    pub fn latency(&self) -> Option<(Duration, Duration)> {
        #[cfg(target_os = "linux")]
        {
            let histogram = self.reader.as_ref()?.latency();
            Some((histogram.p50()?, histogram.p99()?))
        }
        #[cfg(not(target_os = "linux"))]
        {
            None
        }
    }

    /// The last thing that happened.
    #[must_use]
    pub fn status(&self) -> Option<&str> {
        self.status.as_deref()
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
    let reading = sticks.reading();

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
    for (index, value) in reading.axes.iter().enumerate() {
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

    let pressed: Vec<String> = reading
        .buttons
        .iter()
        .enumerate()
        .filter(|(_, down)| **down)
        .map(|(index, _)| index.to_string())
        .collect();

    // What the sticks are costing: frames on the wire and how long a movement took to get
    // there. The number D15 is judged on, shown where the person judging it is looking.
    let wire = match sticks.latency() {
        Some((p50, p99)) => format!(
            "{} frames sent, stick to link p50 {:.1} ms, p99 {:.1} ms",
            sticks.sent(),
            p50.as_secs_f64() * 1000.0,
            p99.as_secs_f64() * 1000.0
        ),
        None => format!("{} frames sent", sticks.sent()),
    };

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
            .children(open.then(|| div().text_xs().text_color(rgb(theme::DIM)).child(wire)))
            .child(devices)
            .children((!reading.axes.is_empty()).then_some(axes))
            .children((!pressed.is_empty()).then(|| {
                div()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(format!("buttons down: {}", pressed.join(", ")))
            })),
    )
    .into_any_element()
}
