//! FFT Setup: `GCSViews/ConfigurationView/ConfigFFT.cs`, an Optional Hardware page of Initial
//! Setup (`GCSViews/InitialSetup.cs:335-338`, behind `displayFFTSetup`), listed once every
//! parameter is in.
//!
//! What it shows (`ConfigFFT.cs:71-157`, built in code - there is no Designer): a group "FFT
//! Setup" holding `INS_LOG_BAT_CNT`, a `RangeControl` from 32 to 4096 in the documented increment
//! (32 when none is documented), and `INS_LOG_BAT_MASK`, a `MavlinkCheckBoxBitMask`; a group
//! "Please ensure IMU_RAW and IMU_FAST are turned off to use FFT" holding `LOG_BITMASK`; and FFT,
//! which opens the FFT window (`fftui.rs`).
//!
//! What it does: the constructor, run once for the page object, disables the page when the
//! vehicle has no `INS_LOG_BAT_CNT` and sets nothing up; otherwise the range control is set up
//! from the documentation and the value, and each bitmask from its parameter, enabled when the
//! vehicle has it (`:26-50`; `Controls/MavlinkCheckBoxBitMask.cs:74-141`). A change of the number
//! writes it at once (`RangeControl1OnValueChanged`, `:52-55`); a box of a bitmask writes the
//! mask at once, the bitmask having no `ValueChanged` subscriber (`MavlinkCheckBoxBitMask.cs:
//! 143-160`). `Activate` disables the page again without `INS_LOG_BAT_CNT` (`:57-64`).
//! `Deactivate` gives the port back (`giveComport = false`, `:66-69`), which this application's
//! link does not share: nothing to do.
//!
//! The C#'s boxes for a write the link failed - the bitmask's "Set X Failed", the number's
//! unhandled `TimeoutException` - go on the status line (the owner's ruling of 2026-09-25,
//! `extra_setup::link_error`).
//!
//! Dead code (PLAN §12 D16): `ConfigFFT.Log`, the class's logger, is never written to.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, FocusHandle, KeyDownEvent, Window, div, prelude::*, px, rgb};

use super::adsb::{Bitmask, RangeControl, RangeHost, range_number, three_places, trackbar};
use super::fftui::FftUi;
use super::optional::{
    Job, Set, SetQueue, at, button, error, group, label, set_failed, timeout_text,
};
use crate::MissionPlanner;
use crate::config::battery_monitor::float_text;
use crate::config::failsafe::{CheckState, Lookup};
use crate::config::servo_output::{Check, Message, Write, check_box, value_of};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list: the literal "FFT Setup".
/// `// C#: GCSViews/InitialSetup.cs:337`
pub const TITLE: &str = "FFT Setup";

/// `groupBox1.Text`. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:137`
pub const GROUP_SETUP: &str = "FFT Setup";

/// `groupBox2.Text`. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:147`
pub const GROUP_LOG: &str = "Please ensure IMU_RAW and IMU_FAST are turned off to use FFT";

/// `but_fft.Text`. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:112`
pub const BUTTON: &str = "FFT";

/// The number of samples a batch takes.
pub const COUNT: &str = "INS_LOG_BAT_CNT";
/// The IMUs batched.
pub const MASK: &str = "INS_LOG_BAT_MASK";
/// What the log records.
pub const LOG_BITMASK: &str = "LOG_BITMASK";

/// `var inc = 32.0`, kept where the documentation has no increment.
/// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:36`
pub const DEFAULT_INCREMENT: f64 = 32.0;

/// `setup(..., inc, 1, 32, 1024 * 4, value)`: the display scale and the range.
/// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:40-44`
pub const RANGE: (f32, f32) = (32.0, 4096.0);

/// `RangeControl`'s Designer texts, what the range control shows when the constructor returned
/// before setting it up. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:92-102`
pub const RANGE_LABEL: &str = "Label";
/// `DescriptionText`'s.
pub const RANGE_DESCRIPTION: &str = "Description";
/// `MavlinkCheckBoxBitMask`'s Designer texts, `myLabel1` and `label1`.
/// `// C#: Controls/MavlinkCheckBoxBitMask.cs:199, 221`
pub const MASK_LABEL: &str = "myLabel1";
/// `label1.Text`.
pub const MASK_DESCRIPTION: &str = "label1";

/// `this.Size`. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:153`
const PAGE_SIZE: (f32, f32) = (710.0, 455.0);
/// `groupBox1`. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:134-135`
const GROUP_SETUP_AT: (f32, f32, f32, f32) = (7.0, 3.0, 616.0, 264.0);
/// `groupBox2`. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:144-145`
const GROUP_LOG_AT: (f32, f32, f32, f32) = (7.0, 273.0, 616.0, 179.0);
/// `but_fft`. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:109-111`
const BUTTON_AT: (f32, f32, f32, f32) = (629.0, 13.0, 75.0, 23.0);
/// `INS_LOG_BAT_CNT` in its group. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:97-99`
const COUNT_AT: (f32, f32, f32, f32) = (6.0, 19.0, 604.0, 108.0);
/// `INS_LOG_BAT_MASK` in its group. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:84-87`
const MASK_AT: (f32, f32, f32, f32) = (6.0, 138.0, 604.0, 115.0);
/// `LOG_BITMASK` in its group. `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:118-121`
const LOG_AT: (f32, f32, f32, f32) = (6.0, 19.0, 604.0, 154.0);

/// A `MavlinkCheckBoxBitMask` on the page.
#[derive(Debug, Clone)]
pub struct Mask {
    /// `ParamName`.
    pub param: &'static str,
    /// `Enabled`: the constructor's false until `setup` finds the parameter.
    pub enabled: bool,
    /// `myLabel1.Text`: the display name.
    pub label: String,
    /// `label1.Text`: the description.
    pub description: String,
    /// The boxes, once set up.
    pub bits: Option<Bitmask>,
}

impl Mask {
    /// The control as the Designer leaves it.
    fn new(param: &'static str) -> Self {
        Self {
            param,
            enabled: false,
            label: MASK_LABEL.to_owned(),
            description: MASK_DESCRIPTION.to_owned(),
            bits: None,
        }
    }

    /// `setup(paramname, paramlist)`: enabled with the documentation's names and bits when the
    /// vehicle has the parameter, otherwise disabled.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:74-141`
    fn setup(&mut self, parameters: &[(String, f64)], lookup: Lookup) {
        let Some(value) = value_of(parameters, self.param) else {
            self.enabled = false;
            return;
        };
        self.enabled = true;
        let meta = lookup(self.param);
        self.label = meta
            .map(|meta| meta.display_name)
            .unwrap_or_default()
            .to_owned();
        self.description = meta
            .map(|meta| meta.description)
            .unwrap_or_default()
            .to_owned();
        self.bits = Some(Bitmask::new(
            meta.map_or(&[][..], |meta| meta.bitmask),
            value,
        ));
    }
}

/// The page object.
#[derive(Debug)]
pub struct Fft {
    made_for: Option<Key>,
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `INS_LOG_BAT_CNT`, once the constructor set it up.
    pub count: Option<RangeControl>,
    /// Its `LabelText`.
    pub count_label: String,
    /// Its `DescriptionText`.
    pub count_description: String,
    /// `INS_LOG_BAT_MASK`.
    pub mask: Mask,
    /// `LOG_BITMASK`.
    pub log_bitmask: Mask,
    /// Whether the number is being typed into.
    editing: bool,
    /// How many times FFT has opened the window.
    pub opened: usize,
    /// The FFT window, while it is open.
    pub window: Option<FftUi>,
    messages: VecDeque<Message>,
    /// The handlers' writes, before the frame queues them.
    pending: Vec<Job>,
    queue: SetQueue,
}

impl Default for Fft {
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            count: None,
            count_label: RANGE_LABEL.to_owned(),
            count_description: RANGE_DESCRIPTION.to_owned(),
            mask: Mask::new(MASK),
            log_bitmask: Mask::new(LOG_BITMASK),
            editing: false,
            opened: 0,
            window: None,
            messages: VecDeque::new(),
            pending: Vec::new(),
            queue: SetQueue::default(),
        }
    }
}

impl Fft {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The page's `Enabled`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enabled
    }

    /// The message box showing: only link failures, which the status line takes.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Shows the page: a new page object for a new screen - the constructor - then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:26-50, 57-64`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key, lookup: Lookup) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let pending = std::mem::take(&mut self.pending);
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(key),
                messages,
                pending,
                queue,
                opened: self.opened,
                window: self.window.take(),
                ..Self::default()
            };
            self.construct(parameters, lookup);
        }
        self.active = true;
        if value_of(parameters, COUNT).is_none() {
            self.enabled = false;
        }
    }

    /// The constructor's setup.
    /// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:26-50`
    fn construct(&mut self, parameters: &[(String, f64)], lookup: Lookup) {
        let Some(value) = value_of(parameters, COUNT) else {
            self.enabled = false;
            return;
        };
        let meta = lookup(COUNT);
        // `GetParameterIncrement`: the documented increment where there is one.
        let increment = meta
            .and_then(|meta| meta.increment)
            .unwrap_or(DEFAULT_INCREMENT);
        #[allow(clippy::cast_possible_truncation)] // `(float)inc`
        let increment = increment as f32;
        self.count_label = meta
            .map(|meta| meta.display_name)
            .unwrap_or_default()
            .to_owned();
        self.count_description = meta
            .map(|meta| meta.description)
            .unwrap_or_default()
            .to_owned();
        self.count = RangeControl::new(increment, 1.0, RANGE.0, RANGE.1, &three_places(value));
        self.mask.setup(parameters, lookup);
        self.log_bitmask.setup(parameters, lookup);
    }

    /// The page hidden, the number being typed into read: `Deactivate`, whose
    /// `giveComport = false` has nothing to give back here.
    /// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:66-69`
    pub fn hide(&mut self) {
        self.leave();
        self.active = false;
    }

    /// `RangeControl1OnValueChanged`: `setParam(name, double.Parse(value))`, its `bool` ignored,
    /// a timeout unhandled.
    /// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:52-55`
    fn count_changed(&mut self) {
        let Some(range) = &self.count else {
            return;
        };
        let value: f64 = range.value_text().parse().unwrap_or(0.0);
        let set = Set {
            on_throw: Some(error(timeout_text(COUNT))),
            ..Set::plain(COUNT, value)
        };
        self.pending.push(Job::new("count", [set]));
    }

    /// The number typed into leaves the focus: `ValidateEditText`.
    pub fn leave(&mut self) {
        if !self.editing {
            return;
        }
        self.editing = false;
        if self.count.as_mut().is_some_and(RangeControl::commit) {
            self.count_changed();
        }
    }

    /// A box of a bitmask clicked: the mask written at once, `Set X Failed` on a failure.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:143-160`
    pub fn click_bit(&mut self, log: bool, bit: usize) {
        if !self.enabled {
            return;
        }
        self.leave();
        let mask = if log {
            &mut self.log_bitmask
        } else {
            &mut self.mask
        };
        if !mask.enabled {
            return;
        }
        let Some(bits) = mask.bits.as_mut() else {
            return;
        };
        let Some(entry) = bits.bits.get_mut(bit) else {
            return;
        };
        entry.2 = !entry.2;
        let write = Write {
            param: mask.param.to_owned(),
            value: f64::from(bits.value()),
            failure: set_failed(mask.param),
        };
        self.pending.push(Job::control(write));
    }

    /// FFT: `new fftui().Show()`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:162-165`
    pub fn open_window(&mut self) {
        if !self.enabled || !self.active {
            return;
        }
        self.leave();
        self.show_window();
    }

    /// `new fftui().Show()` itself: a fresh window, whichever button asked - this page's FFT, or
    /// the Advanced page's, which asks nothing of the vehicle first. The window lives here, with
    /// the one FFT window this application draws.
    /// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:162-165; ConfigAdvanced.cs:114-117`
    pub fn show_window(&mut self) {
        self.opened += 1;
        self.window = Some(FftUi::new());
    }

    /// The window's close box.
    pub fn close_window(&mut self) {
        self.window = None;
    }

    /// Once a frame: a page object whose screen has gone let go, the number typed into read when
    /// the focus left it, the writes moved on, and the window's run polled - its error returned
    /// for the status line.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        focused: bool,
    ) -> Option<String> {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
        }
        if self.editing && !focused {
            self.leave();
        }
        self.queue.push(std::mem::take(&mut self.pending));
        self.queue.advance(telemetry, &mut self.messages);
        self.window.as_mut().and_then(FftUi::poll)
    }
}

impl RangeHost for Fft {
    fn key(&mut self, event: &KeyDownEvent) -> bool {
        if !self.editing {
            return false;
        }
        let Some((handled, changed)) = self.count.as_mut().map(|range| range.key(event)) else {
            return false;
        };
        if changed {
            self.count_changed();
        }
        handled
    }

    fn begin(&mut self, _index: usize) {
        if self.enabled && self.count.is_some() {
            self.editing = true;
        }
    }

    fn step(&mut self, _index: usize, up: bool) {
        if !self.enabled {
            return;
        }
        self.editing = true;
        if self.count.as_mut().is_some_and(|range| range.step(up)) {
            self.count_changed();
        }
    }

    fn page_trackbar(&mut self, _index: usize, up: bool) {
        if !self.enabled {
            return;
        }
        self.leave();
        if self.count.as_mut().is_some_and(|range| range.page(up)) {
            self.count_changed();
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &Fft, view: &TelemetryView) {
    use crate::facts::record;
    record("config.fft.active", page.is_active());
    record("config.fft.enabled", page.enabled());
    record("config.fft.group", GROUP_SETUP);
    record("config.fft.note", GROUP_LOG);
    record("config.fft.button", BUTTON);
    record("config.fft.count.label", &page.count_label);
    record(
        "config.fft.count",
        page.count.as_ref().map_or("0", RangeControl::shown),
    );
    for (key, mask) in [("mask", &page.mask), ("log_bitmask", &page.log_bitmask)] {
        record(format!("config.fft.{key}.enabled"), mask.enabled);
        record(format!("config.fft.{key}.label"), &mask.label);
        record(
            format!("config.fft.{key}"),
            mask.bits
                .as_ref()
                .map_or_else(|| "none".to_owned(), |bits| float_text(bits.value())),
        );
        record(
            format!("config.fft.{key}.bits"),
            mask.bits.as_ref().map_or(0, |bits| bits.bits.len()),
        );
    }
    record("config.fft.opened", page.opened);
    record("config.fft.write", page.queue.last().unwrap_or("none"));
    record(
        "config.fft.writes.pending",
        page.queue.pending() + page.pending.len(),
    );
    for name in [COUNT, MASK, LOG_BITMASK] {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
    super::fftui::record_facts(page.window.as_ref());
}

/// How the page's drawing reaches its page object.
fn access(this: &mut MissionPlanner) -> &mut Fft {
    &mut this.extra.fft
}

/// A bitmask at its place in its group: its name bold, its description, and a box per bit
/// where `setup` laid them.
fn mask_element(
    mask: &Mask,
    log: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    page_enabled: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let enabled = page_enabled && mask.enabled;
    let colour = if enabled { theme::TEXT } else { theme::DIM };
    let mut item = at(x, y, width, height)
        .overflow_hidden()
        .child(
            div()
                .absolute()
                .left(px(3.0))
                .top(px(3.0))
                .text_sm()
                .font_weight(gpui::FontWeight::BOLD)
                .whitespace_nowrap()
                .text_color(rgb(colour))
                .child(mask.label.clone()),
        )
        .child(
            div()
                .absolute()
                .left(px(3.0))
                .top(px(33.0))
                .w(px(width - 6.0))
                .text_xs()
                .whitespace_nowrap()
                .overflow_hidden()
                .text_color(rgb(theme::DIM))
                .child(mask.description.clone()),
        );
    if let Some(bits) = &mask.bits {
        let (places, _) = bits.layout();
        let key = if log { "log_bitmask" } else { "mask" };
        for (bit, ((_, name, checked), (bx, by))) in bits.bits.iter().zip(places).enumerate() {
            let mut check = Check::default();
            check.enabled = enabled;
            check.state = if *checked {
                CheckState::Checked
            } else {
                CheckState::Unchecked
            };
            item = item.child(check_box(
                format!("fft-{key}-bit{bit}"),
                &check,
                name,
                (bx + 3.0, 51.0 + by),
                move |this| access(this).click_bit(log, bit),
                cx,
            ));
        }
    }
    item.into_any_element()
}

/// The page, laid out as `ConfigFFT.InitializeComponent` lays it out.
/// `// C#: GCSViews/ConfigurationView/ConfigFFT.cs:71-157`
pub fn page(
    fft: &Fft,
    number: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !fft.is_active() {
        return div().into_any_element();
    }
    let enabled = fft.enabled();
    let (cx0, cy0, width, height) = COUNT_AT;
    let colour = if enabled { theme::TEXT } else { theme::DIM };
    // `RangeControl.OnPaint`: the label bold at (3, 0), the description in (3, 15, width, 39).
    let mut count = at(cx0, cy0, width, height)
        .child(
            div()
                .absolute()
                .left(px(3.0))
                .top(px(0.0))
                .text_sm()
                .font_weight(gpui::FontWeight::BOLD)
                .whitespace_nowrap()
                .text_color(rgb(colour))
                .child(fft.count_label.clone()),
        )
        .child(
            div()
                .absolute()
                .left(px(3.0))
                .top(px(15.0))
                .w(px(width - 3.0))
                .h(px(39.0))
                .overflow_hidden()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(fft.count_description.clone()),
        );
    match &fft.count {
        Some(range) if enabled => {
            count = count
                .child(range_number(
                    "fft-INS_LOG_BAT_CNT".to_owned(),
                    range,
                    0,
                    fft.editing,
                    access,
                    number,
                    window,
                    cx,
                ))
                .child(trackbar(
                    "fft-INS_LOG_BAT_CNT",
                    range,
                    0,
                    width - 72.0,
                    access,
                    cx,
                ))
                .child(label(72.0, 90.0, range.lbl_min.clone(), true))
                .child(label(width - 69.0, 90.0, range.lbl_max.clone(), true));
        }
        // The Designer's control, never set up, or a disabled page: the value and the range
        // drawn, taking nothing.
        shown => {
            let text = shown
                .as_ref()
                .map_or_else(|| "0".to_owned(), |range| range.shown().to_owned());
            count = count
                .child(
                    at(6.0, 58.0, 57.0, 20.0)
                        .flex()
                        .items_center()
                        .px_1()
                        .rounded_sm()
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .bg(rgb(theme::PANEL))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(text),
                )
                .child(label(
                    72.0,
                    90.0,
                    shown
                        .as_ref()
                        .map_or_else(|| "0".to_owned(), |range| range.lbl_min.clone()),
                    false,
                ))
                .child(label(
                    width - 69.0,
                    90.0,
                    shown
                        .as_ref()
                        .map_or_else(|| "65535".to_owned(), |range| range.lbl_max.clone()),
                    false,
                ));
        }
    }
    let setup_group = group(GROUP_SETUP_AT, GROUP_SETUP, enabled)
        .child(count)
        .child(mask_element(&fft.mask, false, MASK_AT, enabled, cx));
    let log_group = group(GROUP_LOG_AT, GROUP_LOG, enabled).child(mask_element(
        &fft.log_bitmask,
        true,
        LOG_AT,
        enabled,
        cx,
    ));
    let body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(setup_group)
        .child(log_group)
        .child(button(
            "fft-but_fft",
            BUTTON,
            BUTTON_AT,
            enabled,
            |this, _window, _cx| this.extra.fft.open_window(),
            cx,
        ));
    panel(TITLE, body).into_any_element()
}

/// The FFT window, over the whole window while it is open.
pub fn overlay(
    fft: &Fft,
    number: &FocusHandle,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let ui = fft.window.as_ref()?;
    let access: super::fftui::Access = |this| this.extra.fft.window.as_mut();
    let form = super::fftui::dialog(
        ui,
        access,
        |this| this.extra.fft.close_window(),
        number,
        prompt,
        window,
        cx,
    );
    // Two deferred layers side by side: the form, and over it the dialog it is waiting on.
    Some(
        div()
            .child(form)
            .children(super::fftui::prompt_box(ui, access, prompt, window, cx))
            .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::{Answering, drain};
    use mp_link::requests::RequestOutcome;

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// SITL copter's three, with the bundled documentation.
    fn sitl() -> Vec<(String, f64)> {
        table(&[(COUNT, 1024.0), (MASK, 1.0), (LOG_BITMASK, 176_126.0)])
    }

    /// The constructor's words and places, read from the `.cs` when the tree is here.
    #[test]
    fn the_text_is_the_constructors() {
        let Some(cs) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigFFT.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        assert!(cs.contains(&format!("groupBox1.Text = \"{GROUP_SETUP}\";")));
        assert!(cs.contains(&format!("groupBox2.Text = \"{GROUP_LOG}\";")));
        assert!(cs.contains(&format!("but_fft.Text = \"{BUTTON}\";")));
        assert!(cs.contains("32, 1024 * 4,"));
        assert!(cs.contains("var inc = 32.0;"));
        assert!(cs.contains("new System.Drawing.Point(629, 13)"));
        assert!(cs.contains("new System.Drawing.Size(616, 264)"));
        assert!(cs.contains("new System.Drawing.Size(616, 179)"));
        assert!(cs.contains("new System.Drawing.Size(710, 455)"));
        assert!(cs.contains("new fftui().Show();"));
        let setup =
            crate::config_coverage::source::csharp("GCSViews/InitialSetup.cs").unwrap_or_default();
        assert!(setup.contains(&format!("typeof(ConfigFFT), \"{TITLE}\"")));
    }

    /// Without `INS_LOG_BAT_CNT` the constructor disables the page and sets nothing up: the
    /// Designer's texts stay, and neither the number nor a box nor FFT takes anything.
    #[test]
    fn without_the_batch_count_the_page_is_disabled() {
        let mut page = Fft::default();
        page.activate(
            &table(&[(LOG_BITMASK, 1.0)]),
            key(),
            crate::metadata::lookup,
        );
        assert!(page.is_active() && !page.enabled());
        assert!(page.count.is_none());
        assert_eq!(page.count_label, RANGE_LABEL);
        assert_eq!(page.mask.label, MASK_LABEL);
        assert!(!page.log_bitmask.enabled, "setup never ran");
        page.click_bit(true, 0);
        page.open_window();
        assert!(page.window.is_none());
        assert!(page.pending.is_empty());
    }

    /// The bindings: the count at 32 to 4096 with the documented increment, both masks with
    /// the documentation's bits and the vehicle's value.
    #[test]
    fn the_parameters_are_bound_from_the_documentation() {
        let mut page = Fft::default();
        page.activate(&sitl(), key(), crate::metadata::lookup);
        assert!(page.enabled());
        let range = page.count.as_ref();
        // `DecimalPlaces = increment.ToString().Length - 1`: "32", one place.
        assert_eq!(range.map(RangeControl::shown), Some("1024.0"));
        assert_eq!(range.map(|range| range.lbl_min.as_str()), Some("32"));
        assert_eq!(range.map(|range| range.lbl_max.as_str()), Some("4096"));
        let meta = crate::metadata::lookup(COUNT);
        if let Some(meta) = meta {
            assert_eq!(page.count_label, meta.display_name);
        }
        assert!(page.mask.enabled && page.log_bitmask.enabled);
        assert_eq!(
            page.mask.bits.as_ref().map(|bits| float_text(bits.value())),
            Some("1".to_owned())
        );
        if let Some(meta) = crate::metadata::lookup(MASK) {
            assert_eq!(
                page.mask.bits.as_ref().map(|bits| bits.bits.len()),
                Some(meta.bitmask.len())
            );
        }
    }

    /// A change of the number writes it at once; a box of a bitmask writes the mask at once,
    /// its failure the control's "Set X Failed".
    #[test]
    fn changes_write_at_once() {
        let mut page = Fft::default();
        page.activate(&sitl(), key(), crate::metadata::lookup);
        page.step(0, true);
        let first_bit = page
            .mask
            .bits
            .as_ref()
            .and_then(|bits| bits.bits.get(1))
            .map(|bit| bit.0);
        page.click_bit(false, 1);
        let link = Answering::new(&[(MASK, Progress::Finished(RequestOutcome::TimedOut))]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(std::mem::take(&mut page.pending));
        let messages = drain(&mut queue, &link);
        let taken = link.taken();
        let increment = crate::metadata::lookup(COUNT)
            .and_then(|meta| meta.increment)
            .unwrap_or(DEFAULT_INCREMENT);
        assert_eq!(taken.first(), Some(&(COUNT.to_owned(), 1024.0 + increment)));
        let expected_mask = 1.0 + f64::from(1_u32 << first_bit.unwrap_or(0));
        assert_eq!(taken.get(1), Some(&(MASK.to_owned(), expected_mask)));
        assert_eq!(messages, [error(set_failed(MASK))]);
        assert!(crate::config::extra_setup::link_error(&error(set_failed(
            MASK
        ))));
        assert!(crate::config::extra_setup::link_error(&error(
            timeout_text(COUNT)
        )));
    }

    /// FFT opens a fresh window each time; the close box shuts it.
    #[test]
    fn fft_opens_a_new_window() {
        let mut page = Fft::default();
        page.activate(&sitl(), key(), crate::metadata::lookup);
        page.open_window();
        assert!(page.window.is_some());
        if let Some(window) = page.window.as_mut() {
            window.toggle_magnitude();
        }
        page.open_window();
        assert_eq!(page.opened, 2);
        assert!(page.window.as_ref().is_some_and(|window| !window.magnitude));
        page.close_window();
        assert!(page.window.is_none());
    }

    /// Every fact the GUI script asserts on is one this page or its window records, and every
    /// control it clicks is one they draw.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-fft.gui");
        let source = [include_str!("fft.rs"), include_str!("fftui.rs")].concat();
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.fft.") => {
                    // A key recorded through `format!`: its number or its mask a placeholder.
                    let indexed = key
                        .split('.')
                        .map(|part| {
                            if part.parse::<usize>().is_ok() {
                                "{index}"
                            } else {
                                part
                            }
                        })
                        .collect::<Vec<_>>()
                        .join(".");
                    let masked = key
                        .replacen("config.fft.log_bitmask", "config.fft.{key}", 1)
                        .replacen("config.fft.mask", "config.fft.{key}", 1);
                    assert!(
                        [key, indexed.as_str(), masked.as_str()]
                            .iter()
                            .any(|key| source.contains(&format!("\"{key}\""))),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("fft-") => {
                    let id = id.trim_end_matches("-up").trim_end_matches("-down");
                    assert!(source.contains(&format!("\"{id}\"")), "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 10 && clicks >= 3, "{facts} facts, {clicks} clicks");
    }
}
