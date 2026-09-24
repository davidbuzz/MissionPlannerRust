//! The flight screen's Transponder page, `tabTransponder`: a uAvionix ADS-B transponder driven
//! over MAVLink.
//!
//! STBY, ON and ALT choose which replies the transponder makes - none, Mode A, S and 1090ES, or
//! those and Mode C - and IDENT adds the ident flag; each press, a new squawk and every keystroke
//! in the flight ID send `UAVIONIX_ADSB_OUT_CONTROL` with all of it. Connect to Transponder asks
//! the autopilot to forward `UAVIONIX_ADSB_OUT_STATUS` once a second, and each status then sets
//! the page: the modes and the button in bold, the faults, the flight ID and squawk unless they
//! are being typed in, the position integrity and accuracy. Until a status has arrived every
//! control but Connect is disabled, as the `.resx` starts them.
//! `// C#: GCSViews/FlightData.cs:210-241, 4314-4318, 6182-6482, GCSViews/FlightData.Designer.cs:1609-1741`
//!
//! One divergence, and why. The C# clears `xpdr_status_pending` each time the page has shown a
//! status, so the five-second check can tell a transponder that stopped reporting ("Transponder
//! Status Lost") from one still reporting. `mp_vehicle` keeps the last status and a flag that one
//! has ever arrived, with no count of how many, so a status that repeats unchanged cannot be
//! told from none; the page keeps showing the last status it had rather than calling a
//! transponder lost that may be reporting every second.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::time::{Duration, Instant};

use gpui::{AnyElement, Context, FocusHandle, KeyDownEvent, Window, div, prelude::*, px, rgb};
use mp_link::{RequestId, commands};
use mp_mavlink_dialects::all::{MavCmd, MavMessage, UavionixAdsbOutControl};
use mp_vehicle::VehicleId;
use mp_vehicle::onboard::Transponder as Status;

use crate::MissionPlanner;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// `MAVLINK_MSG_ID.UAVIONIX_ADSB_OUT_STATUS`, the message Connect asks for.
pub const STATUS_MESSAGE: u16 = 10_008;

/// `XPDRConnect_btn.Text` in the `.resx`, before anything has changed it.
pub const CONNECT: &str = "Connect to Transponder";

/// What `updateTransponder` puts on the button with no status and none ever had. The C#'s own
/// capital T, unlike the `.resx`'s.
pub const CONNECT_AGAIN: &str = "Connect To Transponder";

/// ... and with no status after having had one.
pub const STATUS_LOST: &str = "Transponder Status Lost";

/// ... with a status.
pub const CONNECTED: &str = "Transponder Connected!";

/// ... with a status that says the transponder has none of its own.
pub const OFFLINE: &str = "Transponder Offline";

/// Connect's message box when no status comes within three seconds.
/// `// C#: GCSViews/FlightData.cs:6358`
pub const NO_STATUS: &str = "Timeout: Status message not received.";

/// Connect's message box when its command goes unanswered. `// C#: GCSViews/FlightData.cs:6363`
pub const TIMEOUT: &str = "Timeout.";

/// How long Connect waits for a status once its command is answered. `// C#: GCSViews/FlightData.cs:6353`
pub const STATUS_WAIT: Duration = Duration::from_secs(3);

/// How often the main loop shows the transponder again without a new status.
/// `// C#: GCSViews/FlightData.cs:4314`
pub const UPDATE_EVERY: Duration = Duration::from_secs(5);

/// `Squawk_nud.Maximum`, and its `Value` at start. `// C#: GCSViews/FlightData.Designer.cs:1655-1665`
pub const SQUAWK_MAX: u16 = 7777;
/// See [`SQUAWK_MAX`].
pub const SQUAWK_START: u16 = 1200;

/// `Mode_clb`'s items. The list is `Visible = False` in the `.resx`; its checks are what the
/// buttons set and what is sent. `// C#: GCSViews/FlightData.resx (Mode_clb.Items)`
pub const MODES: [&str; 4] = ["Mode A", "Mode C", "Mode S", "1090ES ADS-B OUT"];

/// `fault_clb`'s items. The last is checked by `xpdr_airborne_status`, whatever it says.
/// `// C#: GCSViews/FlightData.resx (fault_clb.Items), GCSViews/FlightData.cs:6434-6438`
pub const FAULTS: [&str; 5] = [
    "Maint. Req.",
    "GPS Unavail.",
    "GPS No Fix",
    "TX Sys. Fail.",
    "On Ground",
];

/// `NIC_table`. `// C#: GCSViews/FlightData.cs:210-224`
pub const NIC_TABLE: [&str; 12] = [
    "UNKNOWN", "<20.0NM", "<8.0NM", "<4.0NM", "<2.0NM", "<1.0NM", "<0.3NM", "<0.2NM", "<0.1NM",
    "<75m", "<25m", "<7.5m",
];

/// `NACp_table`. `// C#: GCSViews/FlightData.cs:225-239`
pub const NACP_TABLE: [&str; 12] = [
    "UNKNOWN", "<10.0NM", "<4.0NM", "<2.0NM", "<1.0NM", "<0.5NM", "<0.3NM", "<0.1NM", "<0.05NM",
    "<30m", "<10m", "<3m",
];

/// `UAVIONIX_ADSB_OUT_CONTROL_STATE`'s ident bit, which IDENT adds. `// C#: GCSViews/FlightData.cs:6188`
pub const IDENT_BIT: u8 = 8;

/// The four mode checks as the state bits: Mode A 16, Mode C 32, Mode S 64, 1090ES 128.
/// `// C#: GCSViews/FlightData.cs:6189-6192`
#[must_use]
pub fn state_bits(modes: [bool; 4]) -> u8 {
    let mut bits = 0;
    for (checked, bit) in modes.into_iter().zip([16u8, 32, 64, 128]) {
        if checked {
            bits |= bit;
        }
    }
    bits
}

/// `uAvionixADSBControl(int.MaxValue, squawk, state, 0, ASCII(flight_id), 0)`: the barometric
/// altitude unknown, no emergency, no X-bit, and the flight ID's ASCII bytes cut or padded with
/// zeros to eight, as `StructureToByteArray` fits an array to its field.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:6854-6864, ExtLibs/Mavlink/MavlinkUtil.cs:270-297`
#[must_use]
pub fn control(squawk: u16, state: u8, flight_id: &str) -> MavMessage {
    let mut id = [0u8; 8];
    // `Encoding.ASCII` writes anything outside ASCII as '?'.
    for (slot, c) in id.iter_mut().zip(flight_id.chars()) {
        *slot = if c.is_ascii() {
            u8::try_from(c).unwrap_or(b'?')
        } else {
            b'?'
        };
    }
    MavMessage::UavionixAdsbOutControl(UavionixAdsbOutControl {
        baroaltmsl: i32::MAX,
        squawk,
        state,
        emergencystatus: 0,
        flight_id: id,
        x_bit: 0,
    })
}

/// `doCommand(SET_MESSAGE_INTERVAL, UAVIONIX_ADSB_OUT_STATUS, 1000000, ...)`: a status a second.
/// `// C#: GCSViews/FlightData.cs:6352, 6403`
#[must_use]
pub fn set_message_interval(target: VehicleId) -> MavMessage {
    let command = u16::try_from(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL.0).unwrap_or(u16::MAX);
    commands::command_long(
        target,
        command,
        [
            f32::from(STATUS_MESSAGE),
            1_000_000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
    )
}

/// `Squawk_nud_ValueChanged`'s digits: a squawk is four octal digits, so a 9 is a 7 and an 8
/// carries into the next digit, up to 7777.
/// `// C#: GCSViews/FlightData.cs:6220-6258`
#[must_use]
pub fn fix_squawk(value: u16) -> u16 {
    let mut ones = value % 10;
    let mut tens = (value / 10) % 10;
    let mut hundreds = (value / 100) % 10;
    let mut thousands = (value / 1000) % 10;
    for digit in [&mut ones, &mut tens, &mut hundreds, &mut thousands] {
        if *digit == 9 {
            *digit = 7;
        }
    }
    if ones > 7 {
        tens += 1;
        ones = 0;
    }
    if tens > 7 {
        hundreds += 1;
        tens = 0;
    }
    if hundreds > 7 {
        hundreds = 0;
        thousands += 1;
    }
    if thousands > 7 {
        thousands = 7;
    }
    thousands * 1000 + hundreds * 100 + tens * 10 + ones
}

/// A button of the page's left column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    /// `STBY_btn`: every mode off.
    Stby,
    /// `ON_btn`: Mode A, Mode S and 1090ES.
    On,
    /// `ALT_btn`: all four.
    Alt,
    /// `IDENT_btn`: the modes as they are, with the ident bit.
    Ident,
}

impl Button {
    /// The id a script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Stby => "fly-xpdr-stby",
            Self::On => "fly-xpdr-on",
            Self::Alt => "fly-xpdr-alt",
            Self::Ident => "fly-xpdr-ident",
        }
    }

    /// Its text in the `.resx`.
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Stby => "STBY",
            Self::On => "ON",
            Self::Alt => "ALT",
            Self::Ident => "IDENT",
        }
    }

    /// `Location` and `Size` in the `.resx`: left, top, width, height.
    #[must_use]
    pub const fn bounds(self) -> (f32, f32, f32, f32) {
        match self {
            Self::Stby => (6.0, 6.0, 55.0, 23.0),
            Self::On => (6.0, 35.0, 55.0, 22.0),
            Self::Alt => (6.0, 63.0, 55.0, 23.0),
            Self::Ident => (6.0, 92.0, 55.0, 45.0),
        }
    }
}

/// Which of STBY, ON and ALT the modes are, for the bold one: none, or the one they match.
/// `// C#: GCSViews/FlightData.cs:6419-6431`
#[must_use]
pub fn bold_for(modes: [bool; 4]) -> Option<Button> {
    match modes {
        [false, false, false, false] => Some(Button::Stby),
        [true, false, true, true] => Some(Button::On),
        [true, true, true, true] => Some(Button::Alt),
        _ => None,
    }
}

/// Connect, pressed: the command it sent, and once that has been answered, since when the page
/// has waited for a status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Connecting {
    /// The request, and when it was made; none where there was no vehicle to send to.
    pub request: Option<(RequestId, Instant)>,
    /// When the command came back, which starts the three seconds.
    pub waiting_since: Option<Instant>,
}

/// The page.
#[derive(Debug)]
pub struct Transponder {
    /// `Squawk_nud.Value`.
    pub squawk: u16,
    /// Its text, as typed.
    pub squawk_field: TextField,
    /// `FlightID_tb.Text`.
    pub flight_id: TextField,
    /// `Mode_clb`'s checks, in [`MODES`]' order.
    pub modes: [bool; 4],
    /// `fault_clb`'s checks, in [`FAULTS`]' order.
    pub faults: [bool; 5],
    /// `NIC_tb.Text`.
    pub nic: &'static str,
    /// `NACp_tb.Text`.
    pub nacp: &'static str,
    /// `XPDRConnect_btn.Text`.
    pub connect_text: &'static str,
    /// `XPDRConnect_btn.Enabled`.
    pub connect_enabled: bool,
    /// `Enabled` of STBY, ON, ALT, IDENT, the flight ID and the squawk, which change together.
    pub enabled: bool,
    /// Which of STBY, ON and ALT is in bold.
    pub bold: Option<Button>,
    /// Whether IDENT is in bold: the transponder says its ident is active.
    pub ident_bold: bool,
    /// `transponderNeverConnected`.
    never_connected: bool,
    /// `transponderUpdate`: when the page was last brought up to date.
    last_update: Option<Instant>,
    /// The status the page last showed.
    shown: Option<Status>,
    /// Connect's wait, while it lasts.
    pub connecting: Option<Connecting>,
}

impl Default for Transponder {
    fn default() -> Self {
        let mut squawk_field = TextField::new("");
        squawk_field.set(SQUAWK_START.to_string());
        Self {
            squawk: SQUAWK_START,
            squawk_field,
            flight_id: TextField::new(""),
            modes: [false; 4],
            faults: [false; 5],
            nic: "",
            nacp: "",
            connect_text: CONNECT,
            connect_enabled: true,
            enabled: false,
            bold: None,
            ident_bold: false,
            never_connected: true,
            last_update: None,
            shown: None,
            connecting: None,
        }
    }
}

impl Transponder {
    /// The control message with the page's squawk, flight ID and modes, and `extra` bits.
    fn message(&self, extra: u8) -> MavMessage {
        control(
            self.squawk,
            extra | state_bits(self.modes),
            self.flight_id.value(),
        )
    }

    /// STBY, ON, ALT or IDENT pressed: the modes set, the right button in bold, and what goes.
    /// `// C#: GCSViews/FlightData.cs:6182-6197, 6273-6338`
    pub fn press(&mut self, button: Button) -> MavMessage {
        let modes = match button {
            Button::Stby => [false; 4],
            Button::On => [true, false, true, true],
            Button::Alt => [true; 4],
            Button::Ident => return self.message(IDENT_BIT),
        };
        self.modes = modes;
        self.bold = Some(button);
        self.message(0)
    }

    /// `Squawk_nud.Value` set to `value`, constrained: `ValueChanged`, if it changed - the digits
    /// made octal and the squawk sent.
    fn set_squawk(&mut self, value: u16) -> Option<MavMessage> {
        let value = value.min(SQUAWK_MAX);
        if value == self.squawk {
            self.squawk_field.set(self.squawk.to_string());
            return None;
        }
        self.squawk = fix_squawk(value);
        self.squawk_field.set(self.squawk.to_string());
        Some(self.message(0))
    }

    /// The wheel over the squawk box, or its up and down buttons: one `Increment` a notch,
    /// within the box's range. `// C#: GCSViews/FlightData.cs:6340-6346`
    pub fn step_squawk(&mut self, up: bool) -> Option<MavMessage> {
        let value = if up {
            self.squawk.saturating_add(1)
        } else {
            self.squawk.saturating_sub(1)
        };
        self.set_squawk(value)
    }

    /// Enter in the squawk box: the typed number validated as a `NumericUpDown` validates it -
    /// constrained to the range, and text that is not a number put back to the value.
    pub fn commit_squawk(&mut self) -> Option<MavMessage> {
        match self.squawk_field.value().trim().parse::<f64>() {
            Ok(typed) if typed.is_finite() => {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                // `(UInt16)` of each digit truncates, as a fractional value's would.
                let value = typed.clamp(0.0, f64::from(SQUAWK_MAX)) as u16;
                self.set_squawk(value)
            }
            _ => {
                self.squawk_field.set(self.squawk.to_string());
                None
            }
        }
    }

    /// `FlightID_tb_TextChanged`: the box is `CharacterCasing.Upper` and cut to eight
    /// characters, and every change is sent. `// C#: GCSViews/FlightData.cs:6199-6218, GCSViews/FlightData.Designer.cs:1671`
    pub fn flight_id_changed(&mut self) -> MavMessage {
        let text: String = self
            .flight_id
            .value()
            .to_uppercase()
            .chars()
            .take(8)
            .collect();
        self.flight_id.set(text);
        self.message(0)
    }

    /// `updateTransponder`: the page from the vehicle's last status, when a port is open. Returns
    /// the subscription the first status makes the C# send.
    /// `// C#: GCSViews/FlightData.cs:6368-6482`
    pub fn update(
        &mut self,
        status: Option<&Status>,
        port_open: bool,
        (flight_id_focused, squawk_focused): (bool, bool),
        now: Instant,
    ) -> bool {
        self.last_update = Some(now);
        if !port_open {
            return false;
        }
        let Some(status) = status.filter(|status| status.status_pending) else {
            self.enabled = false;
            if self.never_connected {
                self.connect_text = CONNECT_AGAIN;
            } else {
                self.connect_text = STATUS_LOST;
                self.never_connected = true;
            }
            self.connect_enabled = true;
            return false;
        };
        self.shown = Some(*status);
        if status.status_unavailable {
            self.enabled = false;
            self.connect_text = OFFLINE;
            self.connect_enabled = false;
            return false;
        }
        let subscribe = self.never_connected;
        self.never_connected = false;
        self.enabled = true;
        // "if (!(STBY_btn.Focused || ON_btn.Focused || ALT_btn.Focused))": the buttons here take
        // no focus, so the status always sets them.
        self.modes = [
            status.mode_a_enabled,
            status.mode_c_enabled,
            status.mode_s_enabled,
            status.es1090_tx_enabled,
        ];
        self.bold = bold_for(self.modes);
        self.faults = [
            status.maintenance_required,
            status.gps_unavailable,
            status.gps_no_fix,
            status.adsb_tx_failure,
            status.airborne,
        ];
        if !flight_id_focused {
            // `Encoding.UTF8.GetString(xpdr_flight_id)`: its trailing zeros are not text.
            let text = String::from_utf8_lossy(&status.flight_id);
            self.flight_id.set(text.trim_end_matches('\0'));
        }
        if !squawk_focused && status.squawk <= SQUAWK_MAX {
            // Set without `ValueChanged`; a value outside the box's range throws and is left.
            self.squawk = status.squawk;
            self.squawk_field.set(status.squawk.to_string());
        }
        // A value past the tables throws in the C#; the text is left as it was here.
        if let Some(text) = NIC_TABLE.get(usize::from(status.nic)) {
            self.nic = text;
        }
        if let Some(text) = NACP_TABLE.get(usize::from(status.nacp)) {
            self.nacp = text;
        }
        self.ident_bold = status.ident_active;
        self.connect_text = CONNECTED;
        self.connect_enabled = false;
        subscribe
    }

    /// Whether the main loop would show the transponder again now: a status the page has not
    /// shown, or five seconds since it last looked - the first five from the first time it is
    /// asked, as `transponderUpdate` starts when the main loop does.
    pub fn due(&mut self, status: Option<&Status>, now: Instant) -> bool {
        let fresh =
            status.is_some_and(|status| status.status_pending && self.shown != Some(*status));
        let last = *self.last_update.get_or_insert(now);
        fresh || now.duration_since(last) >= UPDATE_EVERY
    }

    /// Publishes what a UI test asserts on.
    pub fn record_facts(&self) {
        crate::facts::record("fly.xpdr.connect", self.connect_text);
        crate::facts::record("fly.xpdr.connect.enabled", self.connect_enabled);
        crate::facts::record("fly.xpdr.enabled", self.enabled);
        crate::facts::record("fly.xpdr.squawk", self.squawk);
        crate::facts::record("fly.xpdr.flightid", self.flight_id.value());
        // `Mode_clb`'s checked items, by name.
        let modes: Vec<&str> = MODES
            .iter()
            .zip(self.modes)
            .filter(|(_, on)| *on)
            .map(|(name, _)| *name)
            .collect();
        crate::facts::record(
            "fly.xpdr.modes",
            if modes.is_empty() {
                "none".to_owned()
            } else {
                modes.join(",")
            },
        );
        crate::facts::record(
            "fly.xpdr.faults",
            self.faults.map(|on| if on { "1" } else { "0" }).join(","),
        );
        crate::facts::record("fly.xpdr.bold", self.bold.map_or("none", Button::text));
        crate::facts::record("fly.xpdr.ident", self.ident_bold);
        crate::facts::record("fly.xpdr.nic", self.nic);
        crate::facts::record("fly.xpdr.nacp", self.nacp);
        crate::facts::record(
            "fly.xpdr.waiting",
            match self.connecting {
                None => "no",
                Some(Connecting {
                    waiting_since: None,
                    ..
                }) => "command",
                Some(_) => "status",
            },
        );
    }
}

/// The squawk box's and the flight ID box's focus.
#[derive(Debug)]
pub struct Focus {
    /// `FlightID_tb`.
    pub flight_id: FocusHandle,
    /// `Squawk_nud`.
    pub squawk: FocusHandle,
}

impl Focus {
    /// Two new handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            flight_id: cx.focus_handle(),
            squawk: cx.focus_handle(),
        }
    }
}

/// A control placed where the `.resx` puts it.
fn at(left: f32, top: f32, width: f32, height: f32) -> gpui::Div {
    div()
        .absolute()
        .left(px(left))
        .top(px(top))
        .w(px(width))
        .h(px(height))
}

/// A button: bold where the page says so, dimmed and inert where it is disabled.
fn button(
    id: &'static str,
    label: &'static str,
    bounds: (f32, f32, f32, f32),
    enabled: bool,
    bold: bool,
    on_click: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let (left, top, width, height) = bounds;
    let base = crate::probe::measured(id, at(left, top, width, height))
        .id(id)
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .text_xs()
        .when(bold, |this| this.font_weight(gpui::FontWeight::BOLD))
        .child(label);
    if enabled {
        base.bg(rgb(theme::ACTION))
            .border_color(rgb(theme::ACCENT))
            .text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(on_click)
            .into_any_element()
    } else {
        base.bg(rgb(theme::PANEL))
            .border_color(rgb(theme::BORDER))
            .text_color(rgb(theme::DIM))
            .into_any_element()
    }
}

/// A label, `AutoSize`, at its place.
fn label(text: &'static str, left: f32, top: f32) -> AnyElement {
    div()
        .absolute()
        .left(px(left))
        .top(px(top))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(text)
        .into_any_element()
}

/// A read-only or disabled box showing `text`.
fn shown_box(
    id: &'static str,
    text: String,
    bounds: (f32, f32, f32, f32),
    enabled: bool,
) -> AnyElement {
    let (left, top, width, height) = bounds;
    crate::probe::measured(id, at(left, top, width, height))
        .flex()
        .items_center()
        .px_1()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .text_xs()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .overflow_hidden()
        .child(text)
        .into_any_element()
}

/// A `CheckedListBox`, disabled: its items with their checks.
fn check_list(
    id: &'static str,
    items: &[&'static str],
    checked: &[bool],
    bounds: (f32, f32, f32, f32),
) -> AnyElement {
    let (left, top, width, height) = bounds;
    let mut list = crate::probe::measured(id, at(left, top, width, height))
        .flex()
        .flex_col()
        .p(px(2.0))
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .overflow_hidden();
    for (item, on) in items.iter().zip(checked) {
        list = list.child(
            div()
                .flex()
                .gap_1()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(if *on { "\u{2611}" } else { "\u{2610}" })
                .child(*item),
        );
    }
    list.into_any_element()
}

/// The page, each control where the `.resx` puts it in `tabTransponder`'s 290 by 175. `Mode_clb`
/// is `Visible = False` there and is not drawn.
/// `// C#: GCSViews/FlightData.resx (tabTransponder and its controls)`
pub fn page(
    xpdr: &Transponder,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let enabled = xpdr.enabled;
    let mut page = crate::probe::measured("panel:transponder", div())
        .relative()
        .flex_shrink_0()
        .w(px(290.0))
        .h(px(140.0));
    for which in [Button::Stby, Button::On, Button::Alt, Button::Ident] {
        let bold = match which {
            Button::Ident => xpdr.ident_bold,
            other => xpdr.bold == Some(other),
        };
        page = page.child(button(
            which.id(),
            which.text(),
            which.bounds(),
            enabled,
            bold,
            cx.listener(move |this, _event, _window, cx| {
                this.fly_xpdr_press(which);
                cx.notify();
            }),
        ));
    }
    page = page
        .child(label("FlightID", 64.0, 11.0))
        .child(label("Squawk", 64.0, 37.0))
        .child(label("NIC", 64.0, 63.0))
        .child(label("NACp", 64.0, 89.0));

    // `FlightID_tb`: typed into while enabled.
    page = page.child(if enabled {
        at(113.0, 6.0, 77.0, 20.0)
            .child(crate::textfield::text_field(
                "fly-xpdr-flightid",
                &xpdr.flight_id,
                &focus.flight_id,
                focus.flight_id.is_focused(window),
                px(77.0),
                cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    match this.fly_data.transponder.flight_id.key(event) {
                        KeyOutcome::Changed => this.fly_xpdr_flight_id(),
                        KeyOutcome::Ignored => return,
                        KeyOutcome::Submitted | KeyOutcome::Cancelled => {}
                    }
                    cx.notify();
                }),
            ))
            .into_any_element()
    } else {
        shown_box(
            "fly-xpdr-flightid",
            xpdr.flight_id.value().to_owned(),
            (113.0, 6.0, 77.0, 20.0),
            false,
        )
    });

    // `Squawk_nud`: the number, its up and down buttons, and the wheel.
    let squawk = if enabled {
        at(113.0, 31.0, 58.0, 26.0)
            .id("fly-xpdr-squawk-box")
            // `Squawk_nud_MouseWheel`: a notch is one `Increment`.
            .on_scroll_wheel(
                cx.listener(|this, event: &gpui::ScrollWheelEvent, _window, cx| {
                    let delta = event.delta.pixel_delta(px(20.0));
                    if f32::from(delta.y) != 0.0 {
                        this.fly_xpdr_squawk_step(f32::from(delta.y) > 0.0);
                        cx.notify();
                    }
                }),
            )
            .child(crate::textfield::text_field(
                "fly-xpdr-squawk",
                &xpdr.squawk_field,
                &focus.squawk,
                focus.squawk.is_focused(window),
                px(58.0),
                cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    match this.fly_data.transponder.squawk_field.key(event) {
                        // Enter validates a `NumericUpDown`; its value changing is what sends.
                        KeyOutcome::Submitted => this.fly_xpdr_squawk_commit(),
                        KeyOutcome::Ignored => return,
                        KeyOutcome::Changed | KeyOutcome::Cancelled => {}
                    }
                    cx.notify();
                }),
            ))
            .into_any_element()
    } else {
        shown_box(
            "fly-xpdr-squawk",
            xpdr.squawk.to_string(),
            (113.0, 31.0, 58.0, 26.0),
            false,
        )
    };
    let spin = |id: &'static str, glyph: &'static str, top: f32, up: bool| {
        let base = crate::probe::measured(id, at(171.0, top, 19.0, 13.0))
            .id(id)
            .flex()
            .items_center()
            .justify_center()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .text_size(px(8.0))
            .child(glyph);
        if enabled {
            base.bg(rgb(theme::ACTION))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.fly_xpdr_squawk_step(up);
                    cx.notify();
                }))
                .into_any_element()
        } else {
            base.bg(rgb(theme::PANEL))
                .text_color(rgb(theme::DIM))
                .into_any_element()
        }
    };
    page = page
        .child(squawk)
        .child(spin("fly-xpdr-squawk-up", "\u{25b2}", 31.0, true))
        .child(spin("fly-xpdr-squawk-down", "\u{25bc}", 44.0, false))
        .child(shown_box(
            "fly-xpdr-nic",
            xpdr.nic.to_owned(),
            (113.0, 63.0, 77.0, 20.0),
            true,
        ))
        .child(shown_box(
            "fly-xpdr-nacp",
            xpdr.nacp.to_owned(),
            (113.0, 89.0, 77.0, 20.0),
            true,
        ))
        .child(check_list(
            "fly-xpdr-faults",
            &FAULTS,
            &xpdr.faults,
            (196.0, 6.0, 87.0, 103.0),
        ))
        .child(button(
            "fly-xpdr-connect",
            xpdr.connect_text,
            (67.0, 114.0, 216.0, 23.0),
            xpdr.connect_enabled,
            false,
            cx.listener(|this, _event, _window, cx| {
                this.fly_xpdr_connect();
                cx.notify();
            }),
        ));
    page.into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target() -> VehicleId {
        VehicleId::new(1, 1)
    }

    fn sent(message: &MavMessage) -> (u16, u8, [u8; 8], i32) {
        match message {
            MavMessage::UavionixAdsbOutControl(m) => (m.squawk, m.state, m.flight_id, m.baroaltmsl),
            other => panic!("not a control: {other:?}"),
        }
    }

    /// The page as the `.resx` starts it: Connect alone enabled, 1200 in the squawk box.
    #[test]
    fn the_page_starts_as_the_resx_has_it() {
        let xpdr = Transponder::default();
        assert_eq!(xpdr.connect_text, "Connect to Transponder");
        assert!(xpdr.connect_enabled);
        assert!(!xpdr.enabled);
        assert_eq!(xpdr.squawk, 1200);
        assert_eq!(xpdr.squawk_field.value(), "1200");
        assert_eq!(xpdr.modes, [false; 4]);
    }

    /// The three mode buttons set the modes and go bold; IDENT sends the ident bit with the
    /// modes as they are; every one sends the squawk and the flight ID.
    #[test]
    fn the_buttons_send_the_modes_and_ident_adds_its_bit() {
        let mut xpdr = Transponder::default();
        xpdr.flight_id.set("VH123");
        let (squawk, state, id, baro) = sent(&xpdr.press(Button::On));
        assert_eq!((squawk, state, baro), (1200, 16 | 64 | 128, i32::MAX));
        assert_eq!(&id, b"VH123\0\0\0");
        assert_eq!(xpdr.bold, Some(Button::On));
        assert_eq!(sent(&xpdr.press(Button::Alt)).1, 16 | 32 | 64 | 128);
        assert_eq!(xpdr.bold, Some(Button::Alt));
        assert_eq!(sent(&xpdr.press(Button::Ident)).1, 8 | 16 | 32 | 64 | 128);
        assert_eq!(xpdr.bold, Some(Button::Alt), "IDENT leaves the bold");
        assert_eq!(sent(&xpdr.press(Button::Stby)).1, 0);
        assert_eq!(sent(&xpdr.press(Button::Ident)).1, 8);
        assert_eq!(xpdr.bold, Some(Button::Stby));
    }

    /// A squawk is four octal digits: nines become sevens and eights carry.
    #[test]
    fn the_squawk_is_made_octal_as_the_value_changes() {
        assert_eq!(fix_squawk(1200), 1200);
        assert_eq!(fix_squawk(1208), 1210);
        assert_eq!(fix_squawk(1199), 1177);
        assert_eq!(fix_squawk(1278), 1300);
        // Every digit carries: 7778 is 8000, and the thousands stop at 7 with the rest zero.
        assert_eq!(fix_squawk(7778), 7000);
        assert_eq!(fix_squawk(9999), 7777);
        assert_eq!(fix_squawk(8000), 7000);

        let mut xpdr = Transponder::default();
        // The wheel down from 1200 is 1199, which is 1177.
        let message = xpdr.step_squawk(false).expect("a change is sent");
        assert_eq!(sent(&message).0, 1177);
        assert_eq!(xpdr.squawk_field.value(), "1177");
        // Up from 1177 is 1178, which carries to 1200.
        assert_eq!(xpdr.step_squawk(true).map(|m| sent(&m).0), Some(1200));
        // Typed: 7700, the emergency code, then something that is not a number.
        xpdr.squawk_field.set("7700");
        assert_eq!(xpdr.commit_squawk().map(|m| sent(&m).0), Some(7700));
        xpdr.squawk_field.set("abc");
        assert_eq!(xpdr.commit_squawk(), None);
        assert_eq!(xpdr.squawk_field.value(), "7700");
        // Past the maximum is the maximum; the same value again sends nothing.
        xpdr.squawk_field.set("99999");
        assert_eq!(xpdr.commit_squawk().map(|m| sent(&m).0), Some(7777));
        assert_eq!(xpdr.step_squawk(true), None);
    }

    /// The flight ID is upper case and eight characters at most, and each change is sent.
    #[test]
    fn the_flight_id_is_upper_case_and_eight_long() {
        let mut xpdr = Transponder::default();
        xpdr.flight_id.set("abcdefghij");
        let (_, _, id, _) = sent(&xpdr.flight_id_changed());
        assert_eq!(xpdr.flight_id.value(), "ABCDEFGH");
        assert_eq!(&id, b"ABCDEFGH");
    }

    /// The main loop's first look is five seconds after it starts, then every five seconds; a
    /// status not yet shown is looked at straight away.
    #[test]
    fn the_page_looks_every_five_seconds_and_at_a_new_status() {
        let start = Instant::now();
        let mut xpdr = Transponder::default();
        assert!(!xpdr.due(None, start));
        assert!(!xpdr.due(None, start + Duration::from_secs(4)));
        assert!(xpdr.due(None, start + UPDATE_EVERY));
        let status = Status {
            status_pending: true,
            ..Status::default()
        };
        assert!(xpdr.due(Some(&status), start), "a new status");
    }

    /// Connect's command: a status every second.
    #[test]
    fn connect_asks_for_a_status_a_second() {
        assert_eq!(
            crate::fly::describe(&set_message_interval(target())),
            "COMMAND_LONG MAV_CMD_SET_MESSAGE_INTERVAL 10008,1000000,0,0,0,0,0"
        );
    }

    /// With the port open and no status, the page stays disabled and the button asks again; a
    /// status enables it and fills it in, and the first one subscribes; one that says the
    /// transponder has no status of its own is Offline.
    #[test]
    fn a_status_fills_the_page_and_its_absence_leaves_it_disabled() {
        let now = Instant::now();
        let mut xpdr = Transponder::default();
        // No port: nothing changes.
        assert!(!xpdr.update(None, false, (false, false), now));
        assert_eq!(xpdr.connect_text, CONNECT);
        assert!(!xpdr.update(None, true, (false, false), now));
        assert_eq!(xpdr.connect_text, "Connect To Transponder");
        assert!(xpdr.connect_enabled);
        assert!(!xpdr.enabled);

        let status = Status {
            mode_a_enabled: true,
            mode_s_enabled: true,
            es1090_tx_enabled: true,
            ident_active: true,
            airborne: true,
            gps_no_fix: true,
            squawk: 4321,
            nic: 9,
            nacp: 10,
            status_pending: true,
            flight_id: *b"QFA1\0\0\0\0",
            ..Status::default()
        };
        assert!(xpdr.due(Some(&status), now));
        assert!(
            xpdr.update(Some(&status), true, (false, false), now),
            "subscribes"
        );
        assert!(xpdr.enabled);
        assert_eq!(xpdr.connect_text, "Transponder Connected!");
        assert!(!xpdr.connect_enabled);
        assert_eq!(xpdr.modes, [true, false, true, true]);
        assert_eq!(xpdr.bold, Some(Button::On));
        assert!(xpdr.ident_bold);
        assert_eq!(xpdr.faults, [false, false, true, false, true]);
        assert_eq!(xpdr.squawk, 4321);
        assert_eq!(xpdr.flight_id.value(), "QFA1");
        assert_eq!((xpdr.nic, xpdr.nacp), ("<75m", "<10m"));
        assert!(!xpdr.due(Some(&status), now), "shown already");
        assert!(xpdr.due(Some(&status), now + UPDATE_EVERY));
        // A second status does not subscribe again, and a box being typed in is left alone.
        let typed = Status {
            squawk: 7000,
            ..status
        };
        xpdr.flight_id.set("MINE");
        assert!(!xpdr.update(Some(&typed), true, (true, true), now));
        assert_eq!(xpdr.flight_id.value(), "MINE");
        assert_eq!(xpdr.squawk, 4321);

        let offline = Status {
            status_unavailable: true,
            ..status
        };
        xpdr.update(Some(&offline), true, (false, false), now);
        assert_eq!(xpdr.connect_text, "Transponder Offline");
        assert!(!xpdr.enabled);
        assert!(!xpdr.connect_enabled);
    }
}
