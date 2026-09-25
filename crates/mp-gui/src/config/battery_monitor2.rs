//! Battery Monitor 2: `GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs`, an Optional
//! Hardware page of Initial Setup (`GCSViews/InitialSetup.cs:268-272`), listed once every
//! parameter is in.
//!
//! What it shows: the second battery's monitor (`BATT2_MONITOR`), voltage pin (`BATT2_VOLT_PIN`)
//! and current pin (`BATT2_CURR_PIN`) as `MavlinkComboBox`es of their documented values; its
//! capacity (`BATT2_CAPACITY`); "MP Alert on Low Battery"; and the Calibration box - a measured
//! voltage and current typed in, the vehicle's own readings of the second battery beside them,
//! and the divider (`BATT2_VOLT_MULT`) and amps-per-volt (`BATT2_AMP_PERVOL`) they give, the
//! divider as `measured x divider / reading`. A one-second timer refreshes the readings
//! (`ConfigBatteryMonitoring2.cs:17-68, 162-166`).
//!
//! There is no save button: a combo writes when it changes, a text box when it is left
//! (`Validated`), and the measured voltage, divider and amps-per-volt on Enter too
//! (`PreviewKeyDown`). A vehicle without `BATT2_MONITOR` - or no link - disables the page, for the
//! life of the page object: `Activate` never enables it again (`:19-23`). The capacity box asks the
//! vehicle's table, and says the feature is not enabled when `BATT2_CAPACITY` is missing, which it
//! is while `BATT2_MONITOR` is 0 (`:70-89`).
//!
//! The amps-per-volt's name is the C#'s `BATT2_AMP_PERVOL`, which no firmware lists (ArduPilot's
//! is `BATT2_AMP_PERVLT`): the box starts empty and its writes are refused, as in the C#.
//!
//! "MP Alert on Low Battery" reads and writes `Settings.Instance`'s `speechbatteryenabled`,
//! `speechenable` and the three the questions fill - the Planner page's keys, saved with them -
//! and asks the three `InputBox` questions when it is ticked (`:168-204`). Each OK also keeps the
//! answer as `InputBox` does, under `InputBox<caption><question>` (`InputBox.cs:73-84, 178-184`).
//!
//! What a write's failure says - the `catch`es' "Set ... Failed" after a `setParam` timed out, a
//! combo's "Set ... Failed!" - goes on the status line, the owner's ruling of 2026-09-25; the
//! boxes for what was typed - "Invalid number entered", a capacity that does not parse, the
//! feature not enabled - keep their boxes.
//!
//! The layout is `ConfigBatteryMonitoring2.resx`'s, every control at its `Location` in a 521 x 322
//! page. `pictureBox5` shows `Resources.BR_APMPWRDEAN_2`, zoomed ([`crate::pictures`]).

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use gpui::{AnyElement, Context, KeyDownEvent, Window, div, prelude::*, px};
use mp_params::param_file::invariant_double;

use super::battery_monitor::{calibrate, float_text, param_text, parse_float};
use super::optional::{
    FEATURE_NOT_ENABLED, Focus, INVALID_NUMBER, InputBox, Job, Set, SetQueue, error,
    failure_status, group, has, input_box, label, message_box, picture, text_box, value_of,
};
use crate::MissionPlanner;
use crate::config::failsafe::{CheckState, Lookup, options};
use crate::config::servo_output::{Check, Combo, Message, check_box, combo_box, dropdown};
use crate::settings::Persisted;
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::panel;

/// The page's title in Initial Setup's list, `backstageViewPageBatt2.Text`.
/// `// C#: GCSViews/InitialSetup.resx:270-272`
pub const TITLE: &str = "Battery Monitor 2";

/// `timer1.Interval`.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.Designer.cs:95`
const TIMER_INTERVAL: Duration = Duration::from_millis(1000);

/// `TXT_battcapacity.Text` before `Activate`.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.resx TXT_battcapacity.Text`
const DESIGNER_CAPACITY: &str = "2200";

/// The divider's parameter.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:34, 113, 127`
pub const DIVIDER: &str = "BATT2_VOLT_MULT";

/// The amps-per-volt's parameter, as the C# names it.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:37, 141, 246`
pub const AMPS: &str = "BATT2_AMP_PERVOL";

/// The Low Battery alert's three questions: caption, question, the setting it fills, and what it
/// offers when the setting has nothing.
/// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:180-202`
pub const SPEECH_PROMPTS: [(&str, &str, &str, &str); 3] = [
    (
        "Notification",
        "What do you want it to say?",
        "speechbattery",
        "WARNING, Battery at {batv} Volt, {batp} percent",
    ),
    (
        "Battery Level",
        "What Voltage do you want to warn at?",
        "speechbatteryvolt",
        "9.6",
    ),
    (
        "Battery Level",
        "What percentage do you want to warn at?",
        "speechbatterypercent",
        "20",
    ),
];

/// The text boxes that take typing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// `TXT_battcapacity`.
    Capacity,
    /// `TXT_measuredvoltage`.
    Measured,
    /// `TXT_divider`.
    Divider,
    /// `txt_meascurrent`.
    MeasuredCurrent,
    /// `TXT_ampspervolt`.
    AmpsPerVolt,
}

impl Field {
    /// The id a script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Capacity => "battery2-capacity",
            Self::Measured => "battery2-measured",
            Self::Divider => "battery2-divider",
            Self::MeasuredCurrent => "battery2-meascurrent",
            Self::AmpsPerVolt => "battery2-ampspervolt",
        }
    }

    /// Whether Enter validates it: the three with a `PreviewKeyDown` handler.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:206-222`
    const fn validates_on_enter(self) -> bool {
        matches!(self, Self::Measured | Self::Divider | Self::AmpsPerVolt)
    }
}

/// The three combos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Which {
    /// `mavlinkComboBox1`, `BATT2_MONITOR`.
    Monitor,
    /// `mavlinkComboBox2`, `BATT2_VOLT_PIN`.
    VoltPin,
    /// `mavlinkComboBox3`, `BATT2_CURR_PIN`.
    CurrPin,
}

impl Which {
    /// The three, top to bottom.
    pub const ALL: [Self; 3] = [Self::Monitor, Self::VoltPin, Self::CurrPin];

    /// The parameter.
    #[must_use]
    pub const fn param(self) -> &'static str {
        match self {
            Self::Monitor => "BATT2_MONITOR",
            Self::VoltPin => "BATT2_VOLT_PIN",
            Self::CurrPin => "BATT2_CURR_PIN",
        }
    }

    /// Its `Location.Y`; each is at x 177, 165 x 21.
    const fn y(self) -> f32 {
        match self {
            Self::Monitor => 42.0,
            Self::VoltPin => 68.0,
            Self::CurrPin => 95.0,
        }
    }
}

/// `Settings.Instance.GetBoolean(key)`: `bool.TryParse`, false when absent or not a boolean.
/// `// C#: ExtLibs/Utilities/Settings.cs:223-232`
fn get_boolean(settings: &Persisted, key: &str) -> bool {
    settings.get(key).is_some_and(|value| {
        value
            .trim_matches(|c: char| c.is_whitespace() || c == '\0')
            .eq_ignore_ascii_case("true")
    })
}

/// The page object.
#[derive(Debug)]
pub struct BatteryMonitor2 {
    made_for: Option<Key>,
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `_startup`: the handlers write nothing while it is set.
    startup: bool,
    capacity: TextField,
    measured: TextField,
    /// `TXT_voltage.Text`, read-only: the vehicle's second battery voltage.
    voltage: String,
    divider: TextField,
    measured_current: TextField,
    /// `txt_current.Text`, read-only: the vehicle's second battery current.
    current: String,
    amps: TextField,
    /// `CHK_speechbattery.Checked`.
    speech: bool,
    monitor: Combo,
    volt_pin: Combo,
    curr_pin: Combo,
    dropdown: Option<Which>,
    /// The box being typed into.
    editing: Option<Field>,
    /// The Low Battery question being asked, by its number.
    prompt: Option<(usize, InputBox)>,
    timer: Option<Instant>,
    ticks: u32,
    messages: VecDeque<Message>,
    /// The last link failure's words, for the status line.
    status: Option<String>,
    queue: SetQueue,
}

impl Default for BatteryMonitor2 {
    /// `InitializeComponent`: the combos disabled, the capacity "2200", the rest empty.
    fn default() -> Self {
        let mut capacity = TextField::new("");
        capacity.set(DESIGNER_CAPACITY);
        Self {
            made_for: None,
            active: false,
            enabled: true,
            startup: false,
            capacity,
            measured: TextField::new(""),
            voltage: String::new(),
            divider: TextField::new(""),
            measured_current: TextField::new(""),
            current: String::new(),
            amps: TextField::new(""),
            speech: false,
            monitor: Combo::default(),
            volt_pin: Combo::default(),
            curr_pin: Combo::default(),
            dropdown: None,
            editing: None,
            prompt: None,
            timer: None,
            ticks: 0,
            messages: VecDeque::new(),
            status: None,
            queue: SetQueue::default(),
        }
    }
}

/// `cs.battery_voltage2.ToString()` and `cs.current2.ToString()`: doubles.
fn readings(view: &TelemetryView) -> (String, String) {
    view.state.as_deref().map_or_else(
        || ("0".to_owned(), "0".to_owned()),
        |state| {
            let battery = state.batteries.first().copied().unwrap_or_default();
            (
                invariant_double(f64::from(battery.voltage)),
                invariant_double(f64::from(battery.current)),
            )
        },
    )
}

impl BatteryMonitor2 {
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

    /// A text box's text.
    #[must_use]
    pub fn text(&self, field: Field) -> &str {
        self.field(field).value()
    }

    const fn field(&self, field: Field) -> &TextField {
        match field {
            Field::Capacity => &self.capacity,
            Field::Measured => &self.measured,
            Field::Divider => &self.divider,
            Field::MeasuredCurrent => &self.measured_current,
            Field::AmpsPerVolt => &self.amps,
        }
    }

    const fn field_mut(&mut self, field: Field) -> &mut TextField {
        match field {
            Field::Capacity => &mut self.capacity,
            Field::Measured => &mut self.measured,
            Field::Divider => &mut self.divider,
            Field::MeasuredCurrent => &mut self.measured_current,
            Field::AmpsPerVolt => &mut self.amps,
        }
    }

    /// A combo.
    #[must_use]
    pub const fn combo(&self, which: Which) -> &Combo {
        match which {
            Which::Monitor => &self.monitor,
            Which::VoltPin => &self.volt_pin,
            Which::CurrPin => &self.curr_pin,
        }
    }

    const fn combo_mut(&mut self, which: Which) -> &mut Combo {
        match which {
            Which::Monitor => &mut self.monitor,
            Which::VoltPin => &mut self.volt_pin,
            Which::CurrPin => &mut self.curr_pin,
        }
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// The last link failure's words, taken for the status line.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// The question being asked.
    #[must_use]
    pub fn prompt(&self) -> Option<&InputBox> {
        self.prompt.as_ref().map(|(_, input)| input)
    }

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:17-62`
    pub fn activate(
        &mut self,
        view: &TelemetryView,
        key: Key,
        connected: bool,
        lookup: Lookup,
        settings: &Persisted,
    ) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let status = self.status.take();
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(key),
                messages,
                status,
                queue,
                ..Self::default()
            };
        }
        self.active = true;
        self.dropdown = None;
        self.editing = None;
        let parameters = &view.parameters;
        if !connected || !has(parameters, "BATT2_MONITOR") {
            self.enabled = false;
            return;
        }
        self.startup = true;
        if let Some(capacity) = value_of(parameters, "BATT2_CAPACITY") {
            self.capacity.set(param_text(capacity));
        }
        let (voltage, _) = readings(view);
        self.measured.set(voltage.clone());
        self.voltage = voltage;
        if let Some(divider) = value_of(parameters, DIVIDER) {
            self.divider.set(param_text(divider));
        }
        if let Some(amps) = value_of(parameters, AMPS) {
            self.amps.set(param_text(amps));
        }
        // Setting `Checked` raises the handler, which `_startup` stops. `// C#: :40-47`
        self.speech =
            get_boolean(settings, "speechbatteryenabled") && get_boolean(settings, "speechenable");
        for which in Which::ALL {
            let param = which.param();
            self.combo_mut(which)
                .setup(options(param, lookup), param, parameters);
        }
        self.startup = false;
        // `timer1.Start()`: its first tick is an interval away.
        self.timer = Some(Instant::now());
    }

    /// `Deactivate`: the timer stops and `_startup` is set, so a box left as the page goes
    /// writes nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:64-68`
    pub fn deactivate(&mut self) {
        self.active = false;
        self.timer = None;
        self.startup = true;
        self.dropdown = None;
        self.editing = None;
    }

    /// `timer1_Tick`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:162-166`
    fn timer_tick(&mut self, view: &TelemetryView) {
        let (voltage, current) = readings(view);
        self.voltage = voltage;
        self.current = current;
        self.ticks = self.ticks.saturating_add(1);
    }

    /// Whether the handlers act: `_startup` clear and the box enabled - which it is not under a
    /// disabled page.
    const fn live(&self) -> bool {
        !self.startup && self.enabled
    }

    /// `TXT_battcapacity_Validated`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:70-89`
    fn capacity_validated(&self, parameters: &[(String, f64)]) -> Vec<Job> {
        const FAILED: &str = "Set BATT2_CAPACITY Failed";
        if !self.live() {
            return Vec::new();
        }
        if !has(parameters, "BATT2_CAPACITY") {
            return vec![Job::show("capacity", error(FEATURE_NOT_ENABLED))];
        }
        vec![parse_float(self.capacity.value()).map_or_else(
            || Job::show("capacity", error(FAILED)),
            |value| {
                Job::new(
                    "capacity",
                    [Set::caught("BATT2_CAPACITY", f64::from(value), FAILED)],
                )
            },
        )]
    }

    /// `setParam(new[] { name }, float.Parse(text))` in a `try` whose `catch` shows `failed`.
    fn write_text(text: &str, param: &'static str, failed: String) -> Job {
        parse_float(text).map_or_else(
            || Job::show("write", error(failed.clone())),
            |value| {
                Job::new(
                    "write",
                    [Set::caught(param, f64::from(value), failed.clone())],
                )
            },
        )
    }

    /// `TXT_measuredvoltage_Validated`: the divider the measured voltage gives, into its box and
    /// written.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:91-119`
    fn measured_validated(&mut self) -> Vec<Job> {
        if !self.live() {
            return Vec::new();
        }
        let (Some(measured), Some(voltage), Some(divider)) = (
            parse_float(self.measured.value()),
            parse_float(&self.voltage),
            parse_float(self.divider.value()),
        ) else {
            return vec![Job::show("measured", error(INVALID_NUMBER))];
        };
        let Some(divider) = calibrate(measured, divider, voltage) else {
            return Vec::new();
        };
        self.divider.set(float_text(divider));
        vec![Self::write_text(
            self.divider.value(),
            DIVIDER,
            format!("Set {DIVIDER} Failed"),
        )]
    }

    /// `TXT_divider_Validated`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:121-133`
    fn divider_validated(&self) -> Vec<Job> {
        if !self.live() {
            return Vec::new();
        }
        vec![Self::write_text(
            self.divider.value(),
            DIVIDER,
            format!("Set {DIVIDER} Failed"),
        )]
    }

    /// `TXT_ampspervolt_Validated`.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:135-147`
    fn amps_validated(&self) -> Vec<Job> {
        if !self.live() {
            return Vec::new();
        }
        vec![Self::write_text(
            self.amps.value(),
            AMPS,
            format!("Set {AMPS} Failed"),
        )]
    }

    /// `txt_meascurrent_Validated`: the amps-per-volt the measured current gives, into its box
    /// and written.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:224-252`
    fn measured_current_validated(&mut self) -> Vec<Job> {
        if !self.live() {
            return Vec::new();
        }
        let (Some(measured), Some(current), Some(amps)) = (
            parse_float(self.measured_current.value()),
            parse_float(&self.current),
            parse_float(self.amps.value()),
        ) else {
            return vec![Job::show("meascurrent", error(INVALID_NUMBER))];
        };
        let Some(amps) = calibrate(measured, amps, current) else {
            return Vec::new();
        };
        self.amps.set(float_text(amps));
        vec![Self::write_text(
            self.amps.value(),
            AMPS,
            format!("Set {AMPS} Failed"),
        )]
    }

    /// A box's `Validated`.
    fn validated(&mut self, field: Field, parameters: &[(String, f64)]) -> Vec<Job> {
        match field {
            Field::Capacity => self.capacity_validated(parameters),
            Field::Measured => self.measured_validated(),
            Field::Divider => self.divider_validated(),
            Field::MeasuredCurrent => self.measured_current_validated(),
            Field::AmpsPerVolt => self.amps_validated(),
        }
    }

    /// A box clicked into: the one being typed into is left first.
    pub fn begin(&mut self, field: Field, parameters: &[(String, f64)]) -> Vec<Job> {
        if self.editing == Some(field) {
            return Vec::new();
        }
        let jobs = self.leave(parameters);
        self.dropdown = None;
        if self.enabled {
            self.editing = Some(field);
        }
        jobs
    }

    /// The box being typed into loses the focus: its `Validated`.
    pub fn leave(&mut self, parameters: &[(String, f64)]) -> Vec<Job> {
        let Some(field) = self.editing.take() else {
            return Vec::new();
        };
        self.validated(field, parameters)
    }

    /// A key in the box being typed into. Enter validates the three that listen for it.
    pub fn key(&mut self, event: &KeyDownEvent, parameters: &[(String, f64)]) -> (bool, Vec<Job>) {
        let Some(field) = self.editing else {
            return (false, Vec::new());
        };
        match self.field_mut(field).key(event) {
            KeyOutcome::Changed => (true, Vec::new()),
            KeyOutcome::Submitted if field.validates_on_enter() => {
                (true, self.validated(field, parameters))
            }
            KeyOutcome::Submitted | KeyOutcome::Cancelled | KeyOutcome::Ignored => {
                (false, Vec::new())
            }
        }
    }

    /// Replaces a box's text, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, field: Field, text: &str) {
        self.field_mut(field).set(text);
    }

    /// Enter in a box, for a test: what `PreviewKeyDown` does.
    #[cfg(test)]
    pub fn enter(&mut self, field: Field, parameters: &[(String, f64)]) -> Vec<Job> {
        if field.validates_on_enter() {
            self.validated(field, parameters)
        } else {
            Vec::new()
        }
    }

    /// Drops a combo's list down, or back up; a box being typed into is left first.
    pub fn toggle_dropdown(&mut self, which: Which, parameters: &[(String, f64)]) -> Vec<Job> {
        let jobs = self.leave(parameters);
        let enabled = self.enabled && self.combo(which).enabled;
        self.dropdown = if self.dropdown == Some(which) || !enabled {
            None
        } else {
            self.combo_mut(which).open_list();
            Some(which)
        };
        jobs
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, which: Which, lines: i32) {
        if self.dropdown == Some(which) {
            self.combo_mut(which).scroll_list(lines);
        }
    }

    /// A row chosen: the combo's own write.
    pub fn choose(&mut self, which: Which, key: i64) -> Vec<Job> {
        self.dropdown = None;
        if !self.enabled {
            return Vec::new();
        }
        self.combo_mut(which)
            .choose(key)
            .map(Job::control)
            .into_iter()
            .collect()
    }

    /// A click on "MP Alert on Low Battery": the settings changed and, when it ends ticked, the
    /// first of the three questions asked.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:168-204`
    pub fn click_speech(
        &mut self,
        parameters: &[(String, f64)],
        settings: &mut Persisted,
    ) -> Vec<Job> {
        let jobs = self.leave(parameters);
        self.dropdown = None;
        if !self.enabled {
            return jobs;
        }
        self.speech = !self.speech;
        if self.startup {
            return jobs;
        }
        settings.set(
            "speechbatteryenabled",
            if self.speech { "True" } else { "False" },
        );
        settings.set("speechenable", "True");
        self.prompt = if self.speech {
            Self::question(0, settings)
        } else {
            None
        };
        jobs
    }

    /// The `stage`th question, its box holding the setting or the handler's literal.
    fn question(stage: usize, settings: &Persisted) -> Option<(usize, InputBox)> {
        let (title, question, key, default) = SPEECH_PROMPTS.get(stage)?;
        let value = settings.get(key).unwrap_or(default);
        Some((stage, InputBox::new(title, question, value)))
    }

    /// OK on a question: `InputBox` keeping the answer in its list, the page keeping it in its
    /// setting, and the next asked.
    /// `// C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:183-202; ExtLibs/Controls/InputBox.cs:178-184`
    pub fn answer(&mut self, settings: &mut Persisted) {
        let Some((stage, input)) = self.prompt.take() else {
            return;
        };
        input.remember(settings);
        if let Some((_, _, key, _)) = SPEECH_PROMPTS.get(stage) {
            settings.set(key, input.field.value());
        }
        self.prompt = Self::question(stage + 1, settings);
    }

    /// Cancel on a question: `return`, the questions after it unasked.
    pub fn cancel(&mut self) {
        self.prompt = None;
    }

    /// A key in the question's box: Enter is OK, Escape is Cancel.
    pub fn prompt_key(&mut self, event: &KeyDownEvent, settings: &mut Persisted) -> bool {
        let Some((_, input)) = self.prompt.as_mut() else {
            return false;
        };
        match input.field.key(event) {
            KeyOutcome::Submitted => self.answer(settings),
            KeyOutcome::Cancelled => self.cancel(),
            KeyOutcome::Changed => {}
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// Once a frame: a page object whose screen has gone is let go, a box the focus has left is
    /// validated, the timer, the writes.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        focused: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
            self.timer = None;
        }
        if self.editing.is_some() && !focused && self.prompt.is_none() {
            let jobs = self.leave(&view.parameters);
            self.queue.push(jobs);
        }
        if let Some(last) = self.timer
            && now.duration_since(last) >= TIMER_INTERVAL
        {
            self.timer_tick(view);
            self.timer = Some(now);
        }
        // A `catch`'s "Set ... Failed" after a timeout and a combo's "Set ... Failed!" go on the
        // status line (the owner's ruling of 2026-09-25); a handler's own box ahead of its
        // calls - what was typed refused - stays a box.
        // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.cs:85-88, 115-118, 129-132, 143-146, 248-251
        let mut failures = Vec::new();
        self.queue
            .advance_split(telemetry, &mut self.messages, &mut failures);
        if let Some(status) = failure_status(&failures) {
            self.status = Some(status);
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &BatteryMonitor2, view: &TelemetryView) {
    use crate::facts::record;
    record("config.battery2.active", page.is_active());
    record("config.battery2.enabled", page.enabled());
    record("config.battery2.capacity", page.text(Field::Capacity));
    record("config.battery2.measured", page.text(Field::Measured));
    record("config.battery2.divider", page.text(Field::Divider));
    record(
        "config.battery2.meascurrent",
        page.text(Field::MeasuredCurrent),
    );
    record("config.battery2.ampspervolt", page.text(Field::AmpsPerVolt));
    record("config.battery2.voltage", &page.voltage);
    record("config.battery2.current", &page.current);
    record("config.battery2.readouts", page.ticks);
    record("config.battery2.speech", page.speech);
    for (which, key) in [
        (Which::Monitor, "monitor"),
        (Which::VoltPin, "voltpin"),
        (Which::CurrPin, "currpin"),
    ] {
        let combo = page.combo(which);
        record(
            format!("config.battery2.{key}"),
            combo
                .selected
                .map_or_else(|| "none".to_owned(), |value| value.to_string()),
        );
        record(format!("config.battery2.{key}.text"), combo.text());
        record(format!("config.battery2.{key}.enabled"), combo.enabled);
    }
    record(
        "config.battery2.editing",
        page.editing.map_or("none", Field::id),
    );
    record(
        "config.battery2.prompt",
        page.prompt().map_or("none", |input| input.prompt),
    );
    record("config.battery2.write", page.queue.last().unwrap_or("none"));
    record("config.battery2.writes.pending", page.queue.pending());
    record(
        "config.battery2.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    for name in [
        "BATT2_MONITOR",
        "BATT2_CAPACITY",
        "BATT2_VOLT_PIN",
        "BATT2_CURR_PIN",
        DIVIDER,
        AMPS,
    ] {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The page, laid out as `ConfigBatteryMonitoring2.resx` lays it out.
pub fn page(
    battery: &BatteryMonitor2,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !battery.is_active() {
        return div().into_any_element();
    }
    let enabled = battery.enabled;
    let typing = focus.text.is_focused(window);
    let field_box =
        |field: Field, place: (f32, f32, f32, f32), cx: &mut Context<MissionPlanner>| {
            text_box(
                field.id(),
                battery.text(field),
                Some(&focus.text),
                typing && battery.editing == Some(field),
                enabled,
                place,
                move |this| {
                    let view = this.telemetry.view();
                    let jobs = this.optional.battery2.begin(field, &view.parameters);
                    this.optional.battery2.push(jobs);
                },
                |this, event| {
                    let view = this.telemetry.view();
                    let (handled, jobs) = this.optional.battery2.key(event, &view.parameters);
                    this.optional.battery2.push(jobs);
                    handled
                },
                cx,
            )
        };
    let read_only = |id: &'static str, text: &str, y: f32, cx: &mut Context<MissionPlanner>| {
        text_box(
            id,
            text,
            None,
            false,
            enabled,
            (183.0, y, 76.0, 20.0),
            |_| {},
            |_, _| false,
            cx,
        )
    };

    let mut body = div()
        .relative()
        .w(px(521.0))
        .h(px(322.0))
        // `Resources.BR_APMPWRDEAN_2`, zoomed.
        // C#: GCSViews/ConfigurationView/ConfigBatteryMonitoring2.Designer.cs:87; ConfigBatteryMonitoring2.resx:231-232
        .child(picture(
            "battery2",
            "BR_APMPWRDEAN_2",
            (3.0, 41.0, 97.0, 75.0),
            "BR_APMPWRDEAN_2",
            crate::pictures::Layout::Zoom,
        ))
        .child(label(106.0, 45.0, "Monitor", enabled))
        .child(label(106.0, 71.0, "Volt Pin", enabled))
        .child(label(106.0, 98.0, "Current Pin", enabled));
    for which in Which::ALL {
        let mut combo = battery.combo(which).clone();
        combo.enabled &= enabled;
        body = body.child(combo_box(
            format!("battery2-{}", which.param()),
            &combo,
            (177.0, which.y(), 165.0, 21.0),
            move |this| {
                let view = this.telemetry.view();
                let jobs = this
                    .optional
                    .battery2
                    .toggle_dropdown(which, &view.parameters);
                this.optional.battery2.push(jobs);
            },
            cx,
        ));
    }
    let mut speech = Check::default();
    speech.enabled = enabled;
    speech.state = if battery.speech {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    body = body
        .child(label(344.0, 44.0, "Battery Capacity", enabled))
        .child(field_box(Field::Capacity, (434.0, 41.0, 50.0, 20.0), cx))
        .child(label(490.0, 44.0, "mAh", enabled))
        .child(check_box(
            "battery2-speech".to_owned(),
            &speech,
            "MP Alert on Low Battery",
            (347.0, 67.0),
            |this| {
                let view = this.telemetry.view();
                let mut jobs = Vec::new();
                this.battery2_settings(|battery, settings| {
                    jobs = battery.click_speech(&view.parameters, settings);
                });
                this.optional.battery2.push(jobs);
            },
            cx,
        ));

    // groupBox4, "Calibration".
    let row = |x: f32, y: f32, text: &'static str| label(x, y, text, enabled);
    let calibration = group((19.0, 142.0, 276.0, 164.0), "Calibration", enabled)
        .child(row(5.0, 16.0, "1. Measured battery voltage:"))
        .child(field_box(Field::Measured, (183.0, 13.0, 76.0, 20.0), cx))
        .child(row(5.0, 38.0, "2. Battery voltage (Calced):"))
        .child(read_only("battery2-voltage", &battery.voltage, 35.0, cx))
        .child(row(5.0, 59.0, "3. Voltage divider (Calced):"))
        .child(field_box(Field::Divider, (183.0, 56.0, 76.0, 20.0), cx))
        .child(row(6.0, 80.0, "4. Measured current:"))
        .child(field_box(
            Field::MeasuredCurrent,
            (183.0, 77.0, 76.0, 20.0),
            cx,
        ))
        .child(row(6.0, 101.0, "5. Current (Calced)"))
        .child(read_only("battery2-current", &battery.current, 98.0, cx))
        .child(row(6.0, 122.0, "6. Amperes per volt:"))
        .child(field_box(
            Field::AmpsPerVolt,
            (183.0, 119.0, 76.0, 20.0),
            cx,
        ));
    body = body.child(calibration);

    if let Some(which) = battery.dropdown {
        body = body.child(dropdown(
            &format!("battery2-{}", which.param()),
            battery.combo(which),
            (177.0, which.y() + 21.0, 165.0),
            move |this, key| {
                let jobs = this.optional.battery2.choose(which, key);
                this.optional.battery2.push(jobs);
            },
            move |this, lines| this.optional.battery2.scroll_list(which, lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// The message box or the question showing, over the whole window.
pub fn overlay(
    battery: &BatteryMonitor2,
    focus: &Focus,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(message) = battery.message() {
        return Some(message_box(
            "battery2-message",
            "battery2-message-ok",
            message,
            window,
            |this| this.optional.battery2.dismiss_message(),
            cx,
        ));
    }
    let input = battery.prompt()?;
    // The `InputBox` form opens modal with its answer box focused, the one control that takes
    // the focus; so here, whatever the click that asked it left focused.
    // C#: ExtLibs/Controls/InputBox.cs:142-172
    if !focus.prompt.is_focused(window) {
        let handle = focus.prompt.clone();
        cx.defer_in(window, move |_, window, cx| handle.focus(window, cx));
    }
    Some(input_box(
        "battery2-prompt",
        input,
        &focus.prompt,
        window,
        |this, event| {
            let mut handled = false;
            this.battery2_settings(|battery, settings| {
                handled = battery.prompt_key(event, settings);
            });
            handled
        },
        |this| this.battery2_settings(|battery, settings| battery.answer(settings)),
        |this| this.optional.battery2.cancel(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::{Answering, drain};
    use mp_link::requests::RequestOutcome;

    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn view_with(parameters: &[(&str, f64)]) -> TelemetryView {
        let mut view = TelemetryView::disconnected("test");
        view.parameters = parameters
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect::<Vec<_>>()
            .into();
        view
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn run(jobs: Vec<Job>, link: &Answering) -> Vec<Message> {
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        drain(&mut queue, link)
    }

    fn configured() -> TelemetryView {
        let mut view = view_with(&[
            ("BATT2_MONITOR", 4.0),
            ("BATT2_CAPACITY", 5000.0),
            ("BATT2_VOLT_PIN", 13.0),
            ("BATT2_CURR_PIN", 14.0),
            (DIVIDER, 10.1),
            (AMPS, 17.0),
        ]);
        let mut state = mp_vehicle::VehicleState::default();
        state.batteries[0].voltage = 12.5;
        state.batteries[0].current = 2.0;
        view.state = Some(Arc::new(state));
        view
    }

    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigBatteryMonitoring2.resx",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("label30.Text"), Some("Monitor"));
        assert_eq!(get("label47.Text"), Some("Volt Pin"));
        assert_eq!(get("label1.Text"), Some("Current Pin"));
        assert_eq!(get("label29.Text"), Some("Battery Capacity"));
        assert_eq!(get("label2.Text"), Some("mAh"));
        assert_eq!(
            get("CHK_speechbattery.Text"),
            Some("MP Alert on Low Battery")
        );
        assert_eq!(get("TXT_battcapacity.Text"), Some(DESIGNER_CAPACITY));
        assert_eq!(get("TXT_battcapacity.Location"), Some("434, 41"));
        assert_eq!(get("groupBox4.Text"), Some("Calibration"));
        assert_eq!(get("groupBox4.Location"), Some("19, 142"));
        assert_eq!(get("label35.Text"), Some("6. Amperes per volt:"));
        for which in Which::ALL {
            let name = match which {
                Which::Monitor => "mavlinkComboBox1",
                Which::VoltPin => "mavlinkComboBox2",
                Which::CurrPin => "mavlinkComboBox3",
            };
            assert_eq!(
                get(&format!("{name}.Location")),
                Some(format!("177, {}", which.y()).as_str())
            );
        }
    }

    /// SITL's copter: `BATT2_MONITOR` 0 and nothing else of the second battery's.
    #[test]
    fn activate_on_the_sitl_copter() {
        let mut page = BatteryMonitor2::default();
        let view = view_with(&[("BATT2_MONITOR", 0.0)]);
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        assert!(page.enabled());
        assert_eq!(page.text(Field::Capacity), "2200", "the Designer's");
        assert_eq!(page.text(Field::Measured), "0");
        assert_eq!(page.voltage, "0");
        assert_eq!(page.text(Field::Divider), "");
        assert!(page.combo(Which::Monitor).enabled);
        assert_eq!(page.combo(Which::Monitor).text(), "Disabled");
        assert!(!page.combo(Which::VoltPin).enabled && !page.combo(Which::CurrPin).enabled);
        // The capacity asks the table: the feature is not enabled.
        let jobs = page.begin(Field::Capacity, &view.parameters);
        assert!(jobs.is_empty());
        page.type_into(Field::Capacity, "3000");
        let jobs = page.leave(&view.parameters);
        let link = Answering::new(&[]);
        assert_eq!(run(jobs, &link), [error(FEATURE_NOT_ENABLED)]);
        assert!(link.taken().is_empty());
        // The measured voltage against no divider: not a number.
        let jobs = page.enter(Field::Measured, &view.parameters);
        assert_eq!(run(jobs, &link), [error(INVALID_NUMBER)]);
    }

    #[test]
    fn without_batt2_monitor_or_a_link_the_page_is_disabled_for_good() {
        let mut page = BatteryMonitor2::default();
        page.activate(&view_with(&[]), key(), true, bundled, &Persisted::at(None));
        assert!(!page.enabled());
        // `Activate` never enables it again.
        page.activate(&configured(), key(), true, bundled, &Persisted::at(None));
        assert!(!page.enabled());
        assert!(page.choose(Which::Monitor, 3).is_empty());
        let mut page = BatteryMonitor2::default();
        page.activate(&configured(), key(), false, bundled, &Persisted::at(None));
        assert!(!page.enabled());
    }

    #[test]
    fn a_configured_battery_fills_every_box() {
        let mut page = BatteryMonitor2::default();
        page.activate(&configured(), key(), true, bundled, &Persisted::at(None));
        assert_eq!(page.text(Field::Capacity), "5000");
        assert_eq!(page.text(Field::Divider), "10.1");
        assert_eq!(page.text(Field::AmpsPerVolt), "17");
        assert_eq!(page.text(Field::Measured), "12.5");
        assert_eq!(page.combo(Which::VoltPin).text(), "Pixhawk2_PM2");
        assert_eq!(page.combo(Which::CurrPin).text(), "Pixhawk2_PM2");
    }

    /// 12.6 measured against 12.5 read with a divider of 10.1: `12.6 x 10.1 / 12.5` in floats.
    #[test]
    fn a_measured_voltage_sets_the_divider() {
        let view = configured();
        let mut page = BatteryMonitor2::default();
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        page.type_into(Field::Measured, "12.6");
        let jobs = page.enter(Field::Measured, &view.parameters);
        let expected = (12.6_f32 * 10.1_f32) / 12.5_f32;
        assert_eq!(page.text(Field::Divider), float_text(expected));
        let link = Answering::new(&[]);
        assert!(run(jobs, &link).is_empty());
        let written = parse_float(&float_text(expected)).map(f64::from);
        assert_eq!(link.taken(), [(DIVIDER.to_owned(), written.unwrap_or(0.0))]);
    }

    /// The current: the timer's reading, then 2.2 measured against 2 read.
    #[test]
    fn a_measured_current_sets_the_amps_per_volt() {
        let view = configured();
        let mut page = BatteryMonitor2::default();
        let start = Instant::now();
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        page.type_into(Field::MeasuredCurrent, "2.2");
        let jobs = page.validated(Field::MeasuredCurrent, &view.parameters);
        let link = Answering::new(&[]);
        assert_eq!(run(jobs, &link), [error(INVALID_NUMBER)], "no reading yet");
        page.tick(
            &Telemetry::idle(),
            &view,
            true,
            false,
            start + TIMER_INTERVAL * 2,
        );
        assert_eq!(page.current, "2");
        let jobs = page.validated(Field::MeasuredCurrent, &view.parameters);
        assert_eq!(
            page.text(Field::AmpsPerVolt),
            float_text((2.2 * 17.0) / 2.0)
        );
        assert!(run(jobs, &link).is_empty());
        assert_eq!(
            link.taken().last().map(|(name, _)| name.as_str()),
            Some(AMPS)
        );
    }

    /// No firmware has `BATT2_AMP_PERVOL`: a write of it is refused and says nothing; a timeout
    /// says its `catch`.
    #[test]
    fn the_amps_per_volt_write_goes_to_the_csharps_name() {
        let view = configured();
        let mut page = BatteryMonitor2::default();
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        let jobs = page.enter(Field::AmpsPerVolt, &view.parameters);
        let refused =
            Answering::new(&[(AMPS, Progress::Finished(RequestOutcome::UnknownParameter))]);
        assert!(run(jobs, &refused).is_empty());
        let jobs = page.enter(Field::AmpsPerVolt, &view.parameters);
        let silent = Answering::new(&[(AMPS, Progress::Finished(RequestOutcome::TimedOut))]);
        assert_eq!(run(jobs, &silent), [error("Set BATT2_AMP_PERVOL Failed")]);
    }

    /// `Deactivate` sets `_startup`: a box left as the page goes writes nothing.
    #[test]
    fn a_box_left_as_the_page_goes_writes_nothing() {
        let view = configured();
        let mut page = BatteryMonitor2::default();
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        page.begin(Field::Capacity, &view.parameters);
        page.type_into(Field::Capacity, "4000");
        page.deactivate();
        assert!(page.leave(&view.parameters).is_empty());
        assert!(page.validated(Field::Capacity, &view.parameters).is_empty());
    }

    #[test]
    fn the_capacity_is_written_when_its_box_is_left() {
        let view = configured();
        let mut page = BatteryMonitor2::default();
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        page.begin(Field::Capacity, &view.parameters);
        page.type_into(Field::Capacity, "4000");
        let jobs = page.begin(Field::Divider, &view.parameters);
        let link = Answering::new(&[]);
        assert!(run(jobs, &link).is_empty());
        assert_eq!(link.taken(), [("BATT2_CAPACITY".to_owned(), 4000.0)]);
        page.type_into(Field::Capacity, "lots");
        let jobs = page.validated(Field::Capacity, &view.parameters);
        assert_eq!(run(jobs, &link), [error("Set BATT2_CAPACITY Failed")]);
    }

    /// The owner's ruling, through the page's own loop: a write's failure is the status line's,
    /// what was typed refused is still a box.
    #[test]
    fn a_failed_write_is_a_status_line_and_a_refusal_a_box() {
        let telemetry = Telemetry::idle();
        let view = configured();
        let mut page = BatteryMonitor2::default();
        let now = Instant::now();
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        // With no vehicle the combo's `setParam` is false: "Set BATT2_MONITOR Failed!".
        let jobs = page.choose(Which::Monitor, 3);
        page.push(jobs);
        page.tick(&telemetry, &view, true, false, now);
        assert!(page.message().is_none());
        assert_eq!(
            page.take_status().as_deref(),
            Some("Set BATT2_MONITOR Failed!")
        );
        // A capacity that does not parse: the `catch`'s box, for what was typed.
        page.type_into(Field::Capacity, "lots");
        let jobs = page.validated(Field::Capacity, &view.parameters);
        page.push(jobs);
        page.tick(&telemetry, &view, true, false, now);
        assert_eq!(page.message(), Some(&error("Set BATT2_CAPACITY Failed")));
        assert!(page.take_status().is_none());
        // A timeout in the `catch`: sorted out for the status line.
        page.dismiss_message();
        let jobs = page.enter(Field::Divider, &view.parameters);
        let link = Answering::new(&[(DIVIDER, Progress::Finished(RequestOutcome::TimedOut))]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let (mut boxes, mut failures) = (VecDeque::new(), Vec::new());
        while queue.pending() > 0 {
            queue.advance_split(&link, &mut boxes, &mut failures);
        }
        assert!(boxes.is_empty());
        assert_eq!(
            failure_status(&failures).as_deref(),
            Some("Set BATT2_VOLT_MULT Failed")
        );
    }

    #[test]
    fn choosing_a_monitor_writes_it() {
        let view = view_with(&[("BATT2_MONITOR", 0.0)]);
        let mut page = BatteryMonitor2::default();
        page.activate(&view, key(), true, bundled, &Persisted::at(None));
        page.toggle_dropdown(Which::Monitor, &view.parameters);
        let jobs = page.choose(Which::Monitor, 3);
        assert_eq!(page.combo(Which::Monitor).text(), "Analog Voltage Only");
        let link = Answering::new(&[]);
        assert!(run(jobs, &link).is_empty());
        assert_eq!(link.taken(), [("BATT2_MONITOR".to_owned(), 3.0)]);
    }

    /// Ticking the alert writes the settings and asks three questions; Cancel stops them.
    #[test]
    fn the_low_battery_alert_asks_its_three_questions() {
        let view = configured();
        let mut settings = Persisted::at(None);
        let mut page = BatteryMonitor2::default();
        page.activate(&view, key(), true, bundled, &settings);
        assert!(!page.speech);
        page.click_speech(&view.parameters, &mut settings);
        assert_eq!(settings.get("speechbatteryenabled"), Some("True"));
        assert_eq!(settings.get("speechenable"), Some("True"));
        let input = page.prompt().expect("the first question");
        assert_eq!(input.prompt, "What do you want it to say?");
        assert_eq!(
            input.field.value(),
            "WARNING, Battery at {batv} Volt, {batp} percent"
        );
        page.answer(&mut settings);
        assert_eq!(
            settings.get("speechbattery"),
            Some("WARNING, Battery at {batv} Volt, {batp} percent")
        );
        // `InputBox` keeps the answer too, URL-encoded under its caption and question.
        assert_eq!(
            settings.get("InputBoxNotificationWhatdoyouwantittosay"),
            Some("WARNING%2C+Battery+at+%7Bbatv%7D+Volt%2C+%7Bbatp%7D+percent")
        );
        assert_eq!(
            page.prompt().map(|input| input.prompt),
            Some("What Voltage do you want to warn at?")
        );
        page.cancel();
        assert!(page.prompt().is_none());
        assert_eq!(settings.get("speechbatteryvolt"), None);
        // Unticked: no questions, the setting False; and `Activate` reads it back.
        page.click_speech(&view.parameters, &mut settings);
        assert!(page.prompt().is_none());
        assert_eq!(settings.get("speechbatteryenabled"), Some("False"));
        settings.set("speechbatteryenabled", "True");
        page.activate(&view, key(), true, bundled, &settings);
        assert!(page.speech);
    }

    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-battery2.gui");
        let source = include_str!("battery_monitor2.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.battery2.") => {
                    let generic = ["monitor", "voltpin", "currpin"].iter().fold(
                        key.to_owned(),
                        |key, which| {
                            key.replace(
                                &format!("config.battery2.{which}"),
                                "config.battery2.{key}",
                            )
                        },
                    );
                    assert!(
                        source.contains(&format!("\"{key}\""))
                            || source.contains(&format!("\"{generic}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("battery2-") => {
                    let base = id
                        .rsplit_once('-')
                        .filter(|(_, tail)| tail.chars().all(|c| c.is_ascii_digit()))
                        .map_or(id, |(base, _)| base);
                    let param = base.strip_prefix("battery2-").unwrap_or(base);
                    assert!(
                        source.contains(&format!("\"{base}\""))
                            || Which::ALL.iter().any(|which| which.param() == param),
                        "{id} is not drawn"
                    );
                }
                _ => {}
            }
        }
        assert!(facts > 10, "{facts} facts");
    }
}
