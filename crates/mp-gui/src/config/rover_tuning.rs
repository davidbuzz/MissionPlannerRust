//! Basic Tuning for a rover: `GCSViews/ConfigurationView/ConfigArdurover.cs`, the page CONFIG's
//! list adds as "Basic Tuning" - and opens first - when the vehicle is an ArduRover
//! (`GCSViews/SoftwareConfig.cs:186-189`). A plane's Basic Tuning is `ConfigArduplane`
//! (`basic_tuning.rs`), a copter's `ConfigSimplePids`; neither is this.
//!
//! What it shows: six group boxes - Steering Rate, Steering Mode, Speed/Throttle, Throttle and
//! Motors, Navigation and Avoidance - of `MavlinkNumericUpDown`s and two `MavlinkComboBox`es
//! (Brake, Motor Type), each with its label; the RC7 to RC10 options as combos under them; and
//! Write Params and Refresh Screen. The layout is `ConfigArdurover.resx`'s, every control at its
//! `Location` in a 617 x 350 page. Refresh Params is in the Designer and invisible in the `.resx`
//! (`BUT_rerequestparams.Visible = False`), and nothing makes it visible, so it is not drawn.
//!
//! `Activate` (`ConfigArdurover.cs:26-114`) disables the page unless the link is open and the
//! firmware is ArduRover, then binds each control to the first of its names the vehicle has - a
//! number to the first name when the vehicle has none, and disabled; a combo left as it was -
//! hides Avoidance when the vehicle has neither `SONAR_TRIGGER_CM` nor `RNGFND_TRIGGR_CM`, and
//! gives the controls inside the group boxes the parameter's description as their tooltip.
//!
//! Unlike the plane's page, nothing here subscribes to a control's `ValueUpdated`: every control
//! writes its own parameter, a combo when it changes, a number 300 ms after it changes
//! (`Controls/MavlinkNumericUpDown.cs:147-180`, `Controls/MavlinkComboBox.cs:133-200`). So the
//! page's `changes` table is never filled: Write Params - and Ctrl+S, `ProcessCmdKey`'s - goes
//! through an empty copy of it and writes nothing (`ConfigArdurover.cs:116-125, 155-195`).
//! Refresh Screen activates the page again (`:224-238`): its `updateparam` asks the vehicle for
//! the controls whose type is exactly `NumericUpDown` or `ComboBox`, and every control here is a
//! `Mavlink*` one, so it asks for none.
//!
//! Where this differs from the C#, and why:
//!
//! * `setParam` holds the UI thread until the vehicle answers; here each write goes through the
//!   link's retrying set in turn;
//! * the tooltips' delay is gpui's, not `toolTip1`'s 500 ms, and they stay while the pointer does
//!   rather than for `AutoPopDelay`'s 20 s; `AddNewLinesForTooltip` is not called by the C#, and
//!   the text wraps at the tooltip's width.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{
    AnyElement, AnyView, Context, FocusHandle, KeyDownEvent, Render, SharedString, Window, div,
    prelude::*, px, rgb,
};

use super::optional::{Job, SetQueue, button, group, label, message_box};
use crate::MissionPlanner;
use crate::config::extra_setup::{link_error, take_link_errors};
use crate::config::failsafe::{Lookup, options};
use crate::config::flight_modes::{Firmware, ParamWriter};
use crate::config::servo_output::{
    Combo, Designer, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE,
    Question, Setup, Write, combo_box, dropdown, modal, number_box, value_of,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in CONFIG's list, `Strings.BasicTuning`.
/// `// C#: GCSViews/SoftwareConfig.cs:188; ExtLibs/Strings/Strings.resx:241-243`
pub const TITLE: &str = "Basic Tuning";

/// The class, as the lists name it.
pub const CLASS: &str = "ConfigArdurover";

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.resx ($this.Size)`
pub const PAGE_SIZE: (f32, f32) = (617.0, 350.0);

/// Every number's `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.resx (*.Size of each MavlinkNumericUpDown)`
pub const BOX_SIZE: (f32, f32) = (78.0, 20.0);

/// Every combo's `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.resx (*.Size of each MavlinkComboBox)`
pub const COMBO_SIZE: (f32, f32) = (78.0, 21.0);

/// A group box: its Designer name, `Text`, and `Location` and `Size`.
pub type GroupSpec = (&'static str, &'static str, (f32, f32, f32, f32));

/// The six group boxes.
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.resx (groupBox*.Location, .Size, .Text)`
pub const GROUPS: [GroupSpec; 6] = [
    ("groupBox5", "Steering Rate", (12.0, 3.0, 195.0, 156.0)),
    ("groupBox2", "Steering Mode", (12.0, 165.0, 195.0, 37.0)),
    ("groupBox14", "Speed/Throttle", (213.0, 4.0, 195.0, 216.0)),
    (
        "groupBox3",
        "Throttle and Motors",
        (213.0, 226.0, 195.0, 95.0),
    ),
    ("groupBox4", "Navigation", (414.0, 4.0, 195.0, 156.0)),
    ("groupBox1", "Avoidance", (414.0, 167.0, 195.0, 108.0)),
];

/// Group indices, for the tables.
const STEER: usize = 0;
const MODE: usize = 1;
const SPEED: usize = 2;
const THROTTLE: usize = 3;
const NAV: usize = 4;
const AVOID: usize = 5;

/// A control's label: its Designer name, `Text` and `Location` in the control's parent.
pub type LabelSpec = (&'static str, &'static str, (f32, f32));

/// One `MavlinkNumericUpDown`: its Designer name, its group, its `Location` there, its label,
/// and what `Activate` binds it with.
#[derive(Debug, Clone, Copy)]
pub struct Tuned {
    /// The Designer's name.
    pub name: &'static str,
    /// The group box, an index into [`GROUPS`].
    pub group: usize,
    /// Its `Location` in the group.
    pub at: (f32, f32),
    /// Its label.
    pub label: LabelSpec,
    /// `setup`'s `Min`, `Max`, `Scale` and `Increment`.
    pub setup: Setup,
    /// `setup`'s names: the first the vehicle has is bound, else the first.
    pub params: &'static [&'static str],
}

/// A number, as the table below writes one.
const fn tuned(
    name: &'static str,
    group: usize,
    at: (f32, f32),
    label: LabelSpec,
    (minimum, maximum, scale, increment): (f32, f32, f32, f32),
    params: &'static [&'static str],
) -> Tuned {
    Tuned {
        name,
        group,
        at,
        label,
        setup: Setup {
            minimum,
            maximum,
            scale,
            increment,
        },
        params,
    }
}

/// `setup(0, 0, 1, 0.1f, ...)`, the most common arguments.
const TENTHS: (f32, f32, f32, f32) = (0.0, 0.0, 1.0, 0.1);
/// `setup(0, 0, 1, 1, ...)`.
const WHOLE: (f32, f32, f32, f32) = (0.0, 0.0, 1.0, 1.0);

/// Every number, in the order `Activate` sets them up.
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.cs:53-87;
/// ConfigArdurover.resx (*.Location, label*.Text, >>*.Parent)`
#[rustfmt::skip]
pub const NUMBERS: [Tuned; 25] = [
    tuned("STEER2SRV_P", STEER, (111.0, 13.0), ("label17", "P", (6.0, 17.0)), TENTHS, &["STEER2SRV_P", "ATC_STR_RAT_P"]),
    tuned("STEER2SRV_I", STEER, (111.0, 36.0), ("label16", "I", (6.0, 40.0)), TENTHS, &["STEER2SRV_I", "ATC_STR_RAT_I"]),
    tuned("STEER2SRV_D", STEER, (111.0, 59.0), ("label15", "D", (6.0, 63.0)), TENTHS, &["STEER2SRV_D", "ATC_STR_RAT_D"]),
    tuned("STEER2SRV_IMAX", STEER, (111.0, 82.0), ("label14", "IMAX", (6.0, 86.0)), TENTHS, &["STEER2SRV_IMAX", "ATC_STR_RAT_IMAX"]),
    tuned("ATC_STR_RAT_FF", STEER, (111.0, 106.0), ("label24", "FF", (6.0, 110.0)), (0.0, 100.0, 1.0, 0.1), &["ATC_STR_RAT_FF"]),
    tuned("TURN_RADIUS", MODE, (111.0, 12.0), ("label11", "Turn Radius", (6.0, 16.0)), TENTHS, &["TURN_RADIUS"]),
    tuned("SPEED2THR_P", SPEED, (111.0, 13.0), ("label76", "P", (6.0, 17.0)), TENTHS, &["SPEED2THR_P", "ATC_SPEED_P"]),
    tuned("SPEED2THR_I", SPEED, (111.0, 36.0), ("label75", "I", (6.0, 40.0)), TENTHS, &["SPEED2THR_I", "ATC_SPEED_I"]),
    tuned("SPEED2THR_D", SPEED, (111.0, 59.0), ("label74", "D", (6.0, 63.0)), TENTHS, &["SPEED2THR_D", "ATC_SPEED_D"]),
    tuned("SPEED2THR_IMAX", SPEED, (111.0, 82.0), ("label73", "IMAX", (6.0, 86.0)), TENTHS, &["SPEED2THR_IMAX", "ATC_SPEED_IMAX"]),
    tuned("ATC_ACCEL_MAX", SPEED, (111.0, 105.0), ("label20", "Accel Max (m/s/s)", (6.0, 109.0)), TENTHS, &["ATC_ACCEL_MAX"]),
    tuned("WP_SPEED", NAV, (111.0, 14.0), ("label5", "WP Speed", (6.0, 18.0)), (0.0, 100.0, 1.0, 0.1), &["WP_SPEED"]),
    tuned("CRUISE_SPEED", SPEED, (111.0, 157.0), ("label12", "Cruise Speed", (6.0, 161.0)), TENTHS, &["CRUISE_SPEED"]),
    tuned("CRUISE_THROTTLE", SPEED, (111.0, 180.0), ("label8", "Cruise Throttle", (6.0, 184.0)), WHOLE, &["CRUISE_THROTTLE"]),
    tuned("THR_MIN", THROTTLE, (111.0, 41.0), ("label7", "Throttle Min (%)", (6.0, 45.0)), WHOLE, &["THR_MIN", "MOT_THR_MIN"]),
    tuned("THR_MAX", THROTTLE, (111.0, 64.0), ("label6", "Throttle Max (%)", (6.0, 68.0)), WHOLE, &["THR_MAX", "MOT_THR_MAX"]),
    tuned("WP_RADIUS", NAV, (111.0, 38.0), ("label9", "WP Radius", (6.0, 42.0)), TENTHS, &["WP_RADIUS"]),
    tuned("WP_OVERSHOOT", NAV, (111.0, 61.0), ("label19", "WP Overshoot", (6.0, 65.0)), TENTHS, &["WP_OVERSHOOT"]),
    tuned("TURN_G_MAX", NAV, (111.0, 84.0), ("label10", "Turn G Max", (6.0, 88.0)), TENTHS, &["TURN_MAX_G", "ATC_TURN_MAX_G"]),
    tuned("NAVL1_PERIOD", NAV, (111.0, 107.0), ("label18", "Lat Acc Cntl Period", (6.0, 111.0)), WHOLE, &["NAVL1_PERIOD"]),
    tuned("NAVL1_DAMPING", NAV, (111.0, 130.0), ("label13", "Lat Acc Cntl Damp", (6.0, 134.0)), (0.0, 0.0, 1.0, 0.05), &["NAVL1_DAMPING"]),
    tuned("SONAR_TRIGGER_CM", AVOID, (111.0, 13.0), ("label3", "Trigger Dist (cm)", (6.0, 17.0)), WHOLE, &["SONAR_TRIGGER_CM", "RNGFND_TRIGGR_CM"]),
    tuned("SONAR_TURN_ANGLE", AVOID, (111.0, 36.0), ("label2", "Turn Angle", (6.0, 40.0)), WHOLE, &["SONAR_TURN_ANGLE", "RNGFND_TURN_ANGL"]),
    tuned("SONAR_TURN_TIME", AVOID, (111.0, 59.0), ("label1", "Turn Time", (6.0, 63.0)), WHOLE, &["SONAR_TURN_TIME", "RNGFND_TURN_TIME"]),
    tuned("SONAR_DEBOUNCE", AVOID, (111.0, 82.0), ("label4", "Sonar Debounce", (6.0, 86.0)), WHOLE, &["SONAR_DEBOUNCE", "RNGFND_DEBOUNCE"]),
];

/// One `MavlinkComboBox`: its Designer name, its group (`None` for the page), its `Location`
/// there, its label, and `setup`'s names.
#[derive(Debug, Clone, Copy)]
pub struct Chosen {
    /// The Designer's name.
    pub name: &'static str,
    /// The group box, an index into [`GROUPS`], or the page.
    pub group: Option<usize>,
    /// Its `Location` in its parent.
    pub at: (f32, f32),
    /// Its label.
    pub label: LabelSpec,
    /// `setup(new[] { names }, MAV.param)`: the first the vehicle has, or nothing at all.
    pub params: &'static [&'static str],
}

/// Every combo, in the order `Activate` sets them up.
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.cs:45-51;
/// ConfigArdurover.resx (*.Location, label*.Text, >>*.Parent)`
#[rustfmt::skip]
pub const COMBOS: [Chosen; 6] = [
    Chosen { name: "CH7_OPTION", group: None, at: (123.0, 209.0), label: ("label21", "RC7 Opt", (18.0, 212.0)), params: &["CH7_OPTION", "RC7_OPTION"] },
    Chosen { name: "CH8_OPTION", group: None, at: (123.0, 236.0), label: ("label25", "RC8 Opt", (18.0, 239.0)), params: &["CH8_OPTION", "RC8_OPTION"] },
    Chosen { name: "CH9_OPTION", group: None, at: (123.0, 263.0), label: ("label26", "RC9 Opt", (18.0, 266.0)), params: &["CH9_OPTION", "RC9_OPTION"] },
    Chosen { name: "CH10_OPTION", group: None, at: (123.0, 290.0), label: ("label27", "RC10 Opt", (18.0, 293.0)), params: &["CH10_OPTION", "RC10_OPTION"] },
    Chosen { name: "ATC_BRAKE", group: Some(SPEED), at: (111.0, 130.0), label: ("label23", "Brake", (6.0, 133.0)), params: &["ATC_BRAKE"] },
    Chosen { name: "MOT_PWM_TYPE", group: Some(THROTTLE), at: (111.0, 14.0), label: ("label22", "Motor Type", (6.0, 17.0)), params: &["MOT_PWM_TYPE"] },
];

/// A button: its Designer name, `Text`, and `Location` and `Size`.
pub type ButtonSpec = (&'static str, &'static str, (f32, f32, f32, f32));

/// Write Params and Refresh Screen; Refresh Params is invisible (see the module's notes).
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.resx (BUT_*)`
pub const BUTTONS: [ButtonSpec; 2] = [
    ("BUT_writePIDS", "Write Params", (213.0, 328.0, 103.0, 19.0)),
    (
        "BUT_refreshpart",
        "Refresh Screen",
        (431.0, 328.0, 103.0, 19.0),
    ),
];

/// The hidden button, for the Designer test.
#[cfg(test)]
const HIDDEN_BUTTON: &str = "BUT_rerequestparams";

/// The name `setup` binds: the first of `names` the vehicle has, else the first.
/// `// C#: Controls/MavlinkNumericUpDown.cs:54-64`
#[must_use]
pub fn bound_name(names: &[&'static str], parameters: &[(String, f64)]) -> &'static str {
    crate::config::basic_tuning::bound_name(names, parameters)
}

/// The page object.
#[derive(Debug)]
pub struct RoverTuning<H = mp_link::RequestId> {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `groupBox1.Visible`: Avoidance, hidden for good once a vehicle lacks both trigger names.
    avoidance: bool,
    /// The numbers, in [`NUMBERS`]' order.
    numbers: Vec<Number>,
    /// The combos, in [`COMBOS`]' order.
    combos: Vec<Combo>,
    /// The numbers' tooltips, then the combos', in the tables' orders.
    tips: Vec<String>,
    /// The combo whose list is down.
    dropdown: Option<usize>,
    /// The number being typed into.
    editing: Option<usize>,
    /// A number's "Out of range" question.
    question: Option<(usize, Question)>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The last link failure the C# boxes, for the status line (the owner's ruling of
    /// 2026-09-25), until the holder takes it.
    status: Option<String>,
    /// The controls' writes.
    queue: SetQueue<H>,
}

impl<H> Default for RoverTuning<H> {
    /// `InitializeComponent`: every control disabled, as the `.resx` and the `Mavlink*`
    /// constructors leave them, each number at `NumericUpDown`'s defaults.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            avoidance: true,
            numbers: NUMBERS
                .iter()
                .map(|_| Number::new(NUMERIC_DEFAULTS))
                .collect(),
            combos: COMBOS.iter().map(|_| Combo::default()).collect(),
            tips: vec![String::new(); NUMBERS.len() + COMBOS.len()],
            dropdown: None,
            editing: None,
            question: None,
            messages: VecDeque::new(),
            status: None,
            queue: SetQueue::default(),
        }
    }
}

impl<H: Copy> RoverTuning<H> {
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

    /// Whether Avoidance shows.
    #[must_use]
    pub const fn avoidance(&self) -> bool {
        self.avoidance
    }

    /// The numbers, in [`NUMBERS`]' order.
    #[must_use]
    pub fn numbers(&self) -> &[Number] {
        &self.numbers
    }

    /// The combos, in [`COMBOS`]' order.
    #[must_use]
    pub fn combos(&self) -> &[Combo] {
        &self.combos
    }

    /// A number's tooltip.
    #[must_use]
    pub fn number_tip(&self, index: usize) -> &str {
        self.tips.get(index).map_or("", String::as_str)
    }

    /// A combo's tooltip; none for the combos on the page itself.
    #[must_use]
    pub fn combo_tip(&self, index: usize) -> &str {
        self.tips
            .get(NUMBERS.len() + index)
            .map_or("", String::as_str)
    }

    /// The number being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<usize> {
        self.editing
    }

    /// The question showing.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
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

    /// The words of the last link failure since the holder last asked, for the status line.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// How the last write ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.queue.last()
    }

    /// How many writes are queued or on their way.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.pending()
    }

    /// Whether the controls take input: the page enabled, and no question up.
    #[must_use]
    pub const fn live(&self) -> bool {
        self.enabled && self.question.is_none()
    }

    /// The page object let go with its screen, returning what its numbers' timers held.
    fn dispose(&mut self) -> Vec<Write> {
        let pending = self.numbers.iter_mut().filter_map(Number::flush).collect();
        *self = Self {
            messages: std::mem::take(&mut self.messages),
            status: self.status.take(),
            queue: std::mem::take(&mut self.queue),
            ..Self::default()
        };
        pending
    }

    /// Shows the page: a new page object for a new screen, then `Activate`. Returns the writes a
    /// disposed page object's timers held.
    /// `// C#: GCSViews/ConfigurationView/ConfigArdurover.cs:26-114`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        connected: bool,
        firmware: Firmware,
        lookup: Lookup,
    ) -> Vec<Job> {
        let mut jobs = Vec::new();
        if self.made_for != Some(key) {
            jobs.extend(self.dispose().into_iter().map(Job::control));
            self.made_for = Some(key);
        }
        self.active = true;
        self.dropdown = None;
        self.bind(parameters, connected, firmware, lookup);
        jobs
    }

    /// `Activate` itself.
    fn bind(
        &mut self,
        parameters: &[(String, f64)],
        connected: bool,
        firmware: Firmware,
        lookup: Lookup,
    ) {
        // `// C#: :28-41`
        if !connected || firmware != Firmware::ArduRover {
            self.enabled = false;
            return;
        }
        self.enabled = true;
        // `setup(new[] { names }, MAV.param)`: the first name the vehicle has, or nothing - the
        // combo keeps what it had. `// C#: :45-51; Controls/MavlinkComboBox.cs:37-71`
        for (combo, spec) in self.combos.iter_mut().zip(COMBOS) {
            if let Some(name) = spec
                .params
                .iter()
                .find(|name| value_of(parameters, name).is_some())
            {
                combo.setup(options(name, lookup), name, parameters);
            }
        }
        // `// C#: :53-77, 84-87`
        for (number, spec) in self.numbers.iter_mut().zip(NUMBERS) {
            number.setup(
                spec.setup,
                bound_name(spec.params, parameters),
                parameters,
                lookup,
            );
        }
        // `MAV.param[name] == null` for both: the list's indexer gives null for a name it lacks.
        // Nothing shows the group again. `// C#: :79-82; ExtLibs/Mavlink/MAVLinkParamList.cs:20-40`
        if value_of(parameters, "SONAR_TRIGGER_CM").is_none()
            && value_of(parameters, "RNGFND_TRIGGR_CM").is_none()
        {
            self.avoidance = false;
        }
        // The tooltips of the controls inside the group boxes - every number, Brake and Motor
        // Type - the parameter's description, "" for one undocumented. `// C#: :91-111`
        let description =
            |param: &str| lookup(param).map_or("", |meta| meta.description).to_owned();
        for (tip, number) in self.tips.iter_mut().zip(&self.numbers) {
            *tip = description(&number.param);
        }
        for (index, (combo, spec)) in self.combos.iter().zip(COMBOS).enumerate() {
            if spec.group.is_some()
                && let Some(tip) = self.tips.get_mut(NUMBERS.len() + index)
            {
                *tip = description(&combo.param);
            }
        }
    }

    /// The page hidden - it has no `Deactivate` - which takes the focus from a number being typed
    /// into, so its text is read.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = None;
        self.leave(now);
    }

    /// Drops a combo's list down, or back up.
    pub fn toggle_dropdown(&mut self, index: usize, now: Instant) {
        if !self.live() {
            return;
        }
        self.leave(now);
        let enabled = self.combos.get(index).is_some_and(|combo| combo.enabled);
        self.dropdown = if self.dropdown == Some(index) || !enabled {
            None
        } else {
            if let Some(combo) = self.combos.get_mut(index) {
                combo.open_list();
            }
            Some(index)
        };
    }

    /// The wheel over a list.
    pub fn scroll_list(&mut self, index: usize, lines: i32) {
        if self.dropdown == Some(index)
            && let Some(combo) = self.combos.get_mut(index)
        {
            combo.scroll_list(lines);
        }
    }

    /// A row chosen: the control's own write.
    /// `// C#: Controls/MavlinkComboBox.cs:133-200`
    pub fn choose(&mut self, index: usize, key: i64) -> Vec<Job> {
        self.dropdown = None;
        if !self.live() {
            return Vec::new();
        }
        self.combos
            .get_mut(index)
            .and_then(|combo| combo.choose(key))
            .map(Job::control)
            .into_iter()
            .collect()
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) || !self.live() {
            return;
        }
        self.leave(now);
        self.dropdown = None;
        if self.numbers.get(index).is_some_and(|number| number.enabled) {
            self.editing = Some(index);
        }
    }

    /// The number being typed into loses the focus, which reads its text.
    pub fn leave(&mut self, now: Instant) {
        let Some(index) = self.editing.take() else {
            return;
        };
        if let Some(question) = self
            .numbers
            .get_mut(index)
            .and_then(|number| number.commit(now))
        {
            self.question = Some((index, question));
        }
    }

    /// A key for the number being typed into. Ctrl+S is `ProcessCmdKey`'s: Write Params, which
    /// writes nothing (see the module's notes), and the key goes no further.
    /// `// C#: GCSViews/ConfigurationView/ConfigArdurover.cs:116-125`
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
        if !self.live() {
            return false;
        }
        let modifiers = event.keystroke.modifiers;
        if event.keystroke.key == "s"
            && modifiers.control
            && !modifiers.shift
            && !modifiers.alt
            && !modifiers.platform
        {
            return true;
        }
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        let (handled, question) = number.key(event, now);
        if let Some(question) = question {
            self.question = Some((index, question));
        }
        handled
    }

    /// A number's arrow.
    pub fn step(&mut self, index: usize, up: bool, now: Instant) {
        if !self.live() {
            return;
        }
        self.begin(index, now);
        if self.question.is_some() {
            return;
        }
        if let Some(question) = self
            .numbers
            .get_mut(index)
            .and_then(|number| number.step(up, now))
        {
            self.question = Some((index, question));
        }
    }

    /// Types into a number, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, index: usize, text: &str, now: Instant) {
        self.begin(index, now);
        if let Some(number) = self.numbers.get_mut(index) {
            number.type_text(text);
        }
    }

    /// The out-of-range question answered.
    pub fn answer(&mut self, yes: bool, now: Instant) {
        let Some((index, question)) = self.question.take() else {
            return;
        };
        if let Some(number) = self.numbers.get_mut(index) {
            number.answer(&question, yes, now);
        }
    }

    /// Write Params clicked: the focus leaving the number being typed into reads it; then
    /// `BUT_writePIDS_Click` goes through a copy of `changes`, which nothing on this page fills.
    /// `// C#: GCSViews/ConfigurationView/ConfigArdurover.cs:155-195`
    pub fn press_write(&mut self, now: Instant) {
        if self.live() {
            self.leave(now);
        }
    }

    /// Refresh Screen: nothing with no link; otherwise `updateparam`, which asks for nothing (see
    /// the module's notes), then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigArdurover.cs:224-260`
    pub fn refresh_screen(
        &mut self,
        parameters: &[(String, f64)],
        connected: bool,
        firmware: Firmware,
        lookup: Lookup,
        now: Instant,
    ) {
        if !self.live() {
            return;
        }
        self.leave(now);
        if !connected || self.question.is_some() {
            return;
        }
        self.bind(parameters, connected, firmware, lookup);
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// The numbers' timers, then the writes as far as the link's answers allow.
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W, now: Instant) {
        let due: Vec<Job> = self
            .numbers
            .iter_mut()
            .filter_map(|number| number.due(now))
            .map(Job::control)
            .collect();
        self.queue.push(due);
        self.queue.advance(writer, &mut self.messages);
        // The owner's ruling of 2026-09-25 (PLAN.md §12): the controls' boxes for the link
        // failing - "Set X Failed" / "Set X Failed!" in `Strings.ERROR` boxes
        // (`Controls/MavlinkComboBox.cs:182, 197`, `Controls/MavlinkNumericUpDown.cs:171, 175`) -
        // go on the status line. A number's out-of-range question
        // (`Controls/MavlinkNumericUpDown.cs:139`) is a question and keeps its box. The page's
        // own boxes are never reached: Write Params' "Large Value" question, "Your are not
        // connected" and "Set X Failed" (`ConfigArdurover.cs:165, 171, 192`) are inside its loop
        // over `changes`, which nothing fills, and "Error receiving list" (`:215`) is Refresh
        // Params', which is invisible (see the module's notes).
        if let Some(words) = take_link_errors(&mut self.messages, link_error) {
            self.status = Some(words);
        }
    }
}

impl RoverTuning {
    /// Once a frame: a page object whose screen has gone is let go, a number the focus has left
    /// is read, the timers and the writes.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_config: bool,
        focused: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_config || self.made_for != Some(Key::of(view)))
        {
            let pending: Vec<Job> = self.dispose().into_iter().map(Job::control).collect();
            self.queue.push(pending);
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        self.advance(telemetry, now);
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on: whether CONFIG's list holds the page and whether it shows, each
/// control's value, parameter and whether it is enabled, and how the last write went.
pub fn record_facts<H: Copy>(page: &RoverTuning<H>, listed: bool, view: &TelemetryView) {
    use crate::facts::record;
    record("config.rover.listed", listed);
    record("config.rover.active", page.is_active());
    record("config.rover.enabled", page.enabled());
    record("config.rover.avoidance", page.avoidance());
    for (number, spec) in page.numbers().iter().zip(NUMBERS) {
        let key = format!("config.rover.{}", spec.name);
        record(format!("{key}.param"), &number.param);
        record(format!("{key}.enabled"), number.enabled);
        record(key, number.shown());
    }
    for (combo, spec) in page.combos().iter().zip(COMBOS) {
        let key = format!("config.rover.{}", spec.name);
        record(format!("{key}.param"), &combo.param);
        record(format!("{key}.enabled"), combo.enabled);
        record(format!("{key}.text"), combo.text());
        record(
            key,
            combo
                .selected
                .map_or_else(|| "none".to_owned(), |value| value.to_string()),
        );
    }
    record(
        "config.rover.question",
        page.question()
            .map_or_else(|| "none".to_owned(), Question::text),
    );
    record(
        "config.rover.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record("config.rover.write", page.last_write().unwrap_or("none"));
    record("config.rover.writes.pending", page.pending());
    if page.is_active() {
        for name in NUMBERS
            .iter()
            .flat_map(|spec| spec.params.iter())
            .chain(COMBOS.iter().flat_map(|spec| spec.params.iter()))
        {
            if let Some(value) = value_of(&view.parameters, name) {
                record(format!("params.value.{name}"), value);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// A control's tooltip: a `ToolTip`'s text in a box, as gpui shows it. Heli Setup's too.
pub struct Tip(pub SharedString);

impl Render for Tip {
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .max_w(px(360.0))
            .p_1()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_sm()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child(self.0.clone())
    }
}

/// A box at a place, carrying a tooltip when it has one and the page is enabled.
fn tipped(
    id: String,
    (x, y, width, height): (f32, f32, f32, f32),
    tip: &str,
    enabled: bool,
) -> gpui::Stateful<gpui::Div> {
    let wrapper = div()
        .id(SharedString::from(format!("{id}-tip")))
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height));
    if tip.is_empty() || !enabled {
        return wrapper;
    }
    let tip = SharedString::from(tip.to_owned());
    wrapper.tooltip(move |_window, cx| -> AnyView {
        let tip = tip.clone();
        cx.new(|_| Tip(tip)).into()
    })
}

/// A number as a disabled page shows it: its text, inert.
fn inert_number(number: &Number) -> Number {
    let mut copy = Number::new(Designer {
        minimum: number.minimum,
        maximum: number.maximum,
        value: number.shown().parse().unwrap_or(0.0),
        decimals: number.decimals,
    });
    copy.enabled = false;
    copy
}

/// The page, laid out as `ConfigArdurover.resx` lays it out.
/// `// C#: GCSViews/ConfigurationView/ConfigArdurover.Designer.cs:29-723; ConfigArdurover.resx`
pub fn page(
    rover: &RoverTuning,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let enabled = rover.enabled();
    let live = rover.live();
    let mut body = div().relative().w(px(PAGE_SIZE.0)).h(px(PAGE_SIZE.1));
    let combo_at = |index: usize,
                    spec: &Chosen,
                    (ox, oy): (f32, f32),
                    cx: &mut Context<MissionPlanner>|
     -> AnyElement {
        let Some(combo) = rover.combos().get(index) else {
            return div().into_any_element();
        };
        let mut shown = combo.clone();
        shown.enabled &= live;
        let (x, y) = spec.at;
        tipped(
            format!("rover-{}", spec.name),
            (ox + x, oy + y, COMBO_SIZE.0, COMBO_SIZE.1),
            rover.combo_tip(index),
            enabled,
        )
        .child(combo_box(
            format!("rover-{}", spec.name),
            &shown,
            (0.0, 0.0, COMBO_SIZE.0, COMBO_SIZE.1),
            move |this| {
                this.software_pages
                    .rover
                    .toggle_dropdown(index, Instant::now());
            },
            cx,
        ))
        .into_any_element()
    };
    for (group_index, &(_, caption, place)) in GROUPS.iter().enumerate() {
        if group_index == AVOID && !rover.avoidance() {
            continue;
        }
        let mut boxes = group(place, caption, enabled);
        for (index, (number, spec)) in rover.numbers().iter().zip(NUMBERS).enumerate() {
            if spec.group != group_index {
                continue;
            }
            let (_, text, (lx, ly)) = spec.label;
            boxes = boxes.child(label(lx, ly, text, enabled));
            let shown;
            let number = if live {
                number
            } else {
                shown = inert_number(number);
                &shown
            };
            let (x, y) = spec.at;
            boxes = boxes.child(
                tipped(
                    format!("rover-{}", spec.name),
                    (x, y, BOX_SIZE.0, BOX_SIZE.1),
                    rover.number_tip(index),
                    enabled,
                )
                .child(number_box(
                    format!("rover-{}", spec.name),
                    number,
                    rover.editing() == Some(index),
                    handle,
                    (0.0, 0.0, BOX_SIZE.0, BOX_SIZE.1),
                    NumberHandlers {
                        begin: move |this: &mut MissionPlanner| {
                            this.software_pages.rover.begin(index, Instant::now());
                        },
                        key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                            this.software_pages.rover.key(event, Instant::now())
                        },
                        step: move |this: &mut MissionPlanner, up: bool| {
                            this.software_pages.rover.step(index, up, Instant::now());
                        },
                    },
                    window,
                    cx,
                )),
            );
        }
        for (index, spec) in COMBOS.iter().enumerate() {
            if spec.group != Some(group_index) {
                continue;
            }
            let (_, text, (lx, ly)) = spec.label;
            boxes = boxes.child(label(lx, ly, text, enabled)).child(combo_at(
                index,
                spec,
                (0.0, 0.0),
                cx,
            ));
        }
        body = body.child(boxes);
    }
    for (index, spec) in COMBOS.iter().enumerate() {
        if spec.group.is_some() {
            continue;
        }
        let (_, text, (lx, ly)) = spec.label;
        body =
            body.child(label(lx, ly, text, enabled))
                .child(combo_at(index, spec, (0.0, 0.0), cx));
    }
    let [write, screen] = BUTTONS;
    body = body
        .child(button(
            "rover-BUT_writePIDS",
            write.1,
            write.2,
            live,
            |this, _window, _cx| this.software_pages.rover.press_write(Instant::now()),
            cx,
        ))
        .child(button(
            "rover-BUT_refreshpart",
            screen.1,
            screen.2,
            live,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                let vehicle = crate::setup::Vehicle::of(&view, this.telemetry.firmware_banner());
                this.software_pages.rover.refresh_screen(
                    &view.parameters,
                    vehicle.connected,
                    vehicle.firmware,
                    crate::metadata::lookup,
                    Instant::now(),
                );
            },
            cx,
        ));
    if let Some(index) = rover.dropdown
        && let Some((combo, spec)) = rover.combos().get(index).zip(COMBOS.get(index))
    {
        let (ox, oy) = spec
            .group
            .and_then(|group| GROUPS.get(group))
            .map_or((0.0, 0.0), |(_, _, (x, y, _, _))| (*x, *y));
        let (x, y) = spec.at;
        body = body.child(dropdown(
            &format!("rover-{}", spec.name),
            combo,
            (ox + x, oy + y + COMBO_SIZE.1, COMBO_SIZE.0),
            move |this, key| {
                let jobs = this.software_pages.rover.choose(index, key);
                this.software_pages.rover.push(jobs);
            },
            move |this, lines| this.software_pages.rover.scroll_list(index, lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    rover: &RoverTuning,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = rover.question() {
        let buttons = vec![
            action(
                "rover-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.software_pages.rover.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "rover-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.software_pages.rover.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "rover-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = rover.message()?;
    Some(message_box(
        "rover-message",
        "rover-message-ok",
        message,
        window,
        |this| this.software_pages.rover.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::time::Duration;

    use gpui::{Keystroke, Modifiers};
    use mp_link::requests::RequestOutcome;
    use mp_params::{ParamMeta, UserLevel};

    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::Answering;
    use crate::config::servo_output::WRITE_DELAY;
    use crate::config_coverage::source::{csharp, resx};

    /// A parameter's documentation, as the rover's `apm.pdef.xml` gives it.
    const fn meta(
        name: &'static str,
        description: &'static str,
        range: Option<(f64, f64)>,
        increment: Option<f64>,
        values: &'static [(i64, &'static str)],
    ) -> ParamMeta {
        ParamMeta {
            name,
            display_name: name,
            description,
            units: "",
            range,
            increment,
            values,
            bitmask: &[],
            user_level: UserLevel::Standard,
            reboot_required: false,
        }
    }

    /// Part of ArduRover 4.5's documentation: the bundled table is the copter's.
    static ROVER: [ParamMeta; 6] = [
        meta(
            "ATC_STR_RAT_P",
            "Steering control rate P gain.",
            Some((0.0, 2.0)),
            Some(0.01),
            &[],
        ),
        meta(
            "CRUISE_SPEED",
            "The target speed in auto missions.",
            Some((0.0, 100.0)),
            Some(0.1),
            &[],
        ),
        meta(
            "MOT_THR_MAX",
            "The maximum throttle setting as a percentage.",
            Some((30.0, 100.0)),
            Some(1.0),
            &[],
        ),
        meta(
            "ATC_BRAKE",
            "Enable using reverse thrust to slow the vehicle",
            None,
            None,
            &[(0, "Disable"), (1, "Enable")],
        ),
        meta(
            "MOT_PWM_TYPE",
            "This selects the output PWM type.",
            None,
            None,
            &[(0, "Normal"), (1, "OneShot"), (3, "BrushedWithRelay")],
        ),
        meta(
            "RC7_OPTION",
            "Function assigned to this RC channel",
            None,
            None,
            &[(0, "Do Nothing"), (9, "Camera Trigger"), (7, "Save WP")],
        ),
    ];

    fn rover_meta(name: &str) -> Option<&'static ParamMeta> {
        ROVER.iter().find(|meta| meta.name == name)
    }

    /// An ArduRover 4.5's values for the page's names: the new steering and speed names, no
    /// sonar trigger, and the RC options under `RCn_OPTION`.
    fn rover() -> Vec<(String, f64)> {
        [
            ("ATC_STR_RAT_P", 0.2),
            ("ATC_STR_RAT_I", 0.2),
            ("ATC_STR_RAT_D", 0.0),
            ("ATC_STR_RAT_IMAX", 1.0),
            ("ATC_STR_RAT_FF", 0.2),
            ("TURN_RADIUS", 0.9),
            ("ATC_SPEED_P", 0.2),
            ("ATC_SPEED_I", 0.2),
            ("ATC_SPEED_D", 0.0),
            ("ATC_SPEED_IMAX", 1.0),
            ("ATC_ACCEL_MAX", 1.0),
            ("WP_SPEED", 2.0),
            ("CRUISE_SPEED", 2.0),
            ("CRUISE_THROTTLE", 50.0),
            ("MOT_THR_MIN", 0.0),
            ("MOT_THR_MAX", 100.0),
            ("WP_RADIUS", 2.0),
            ("WP_OVERSHOOT", 2.0),
            ("ATC_TURN_MAX_G", 0.6),
            ("NAVL1_PERIOD", 8.0),
            ("NAVL1_DAMPING", 0.75),
            ("ATC_BRAKE", 1.0),
            ("MOT_PWM_TYPE", 0.0),
            ("RC7_OPTION", 7.0),
            ("RC8_OPTION", 0.0),
            ("RC9_OPTION", 0.0),
            ("RC10_OPTION", 0.0),
        ]
        .iter()
        .map(|(name, value)| ((*name).to_owned(), *value))
        .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn index(name: &str) -> usize {
        NUMBERS
            .iter()
            .position(|spec| spec.name == name)
            .unwrap_or_else(|| panic!("{name} is not a number"))
    }

    fn combo_index(name: &str) -> usize {
        COMBOS
            .iter()
            .position(|spec| spec.name == name)
            .unwrap_or_else(|| panic!("{name} is not a combo"))
    }

    fn shown() -> RoverTuning<usize> {
        let mut page = RoverTuning::<usize>::default();
        let jobs = page.activate(&rover(), key(), true, Firmware::ArduRover, rover_meta);
        assert!(jobs.is_empty());
        page
    }

    fn run(page: &mut RoverTuning<usize>, link: &Answering, now: Instant) {
        for _ in 0..100 {
            page.advance(link, now);
            if page.pending() == 0 {
                break;
            }
        }
    }

    fn pair((x, y): (f32, f32)) -> String {
        format!("{x}, {y}")
    }

    /// Every control the Designer makes is drawn - the six groups, 25 numbers, six combos, their
    /// 31 labels and two buttons - but Refresh Params, invisible; and `toolTip1` is the tooltips.
    #[test]
    fn every_designer_control_is_drawn() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigArdurover.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let made: BTreeSet<&str> = designer
            .lines()
            .filter_map(|line| line.trim().strip_prefix("this."))
            .filter(|line| line.contains(" = new ") && !line.starts_with("components"))
            .filter_map(|line| line.split_once(" = new "))
            .filter(|(name, _)| !name.contains('.'))
            .map(|(name, _)| name)
            .collect();
        let ours: BTreeSet<&str> = GROUPS
            .iter()
            .map(|(name, ..)| *name)
            .chain(NUMBERS.iter().map(|spec| spec.name))
            .chain(NUMBERS.iter().map(|spec| spec.label.0))
            .chain(COMBOS.iter().map(|spec| spec.name))
            .chain(COMBOS.iter().map(|spec| spec.label.0))
            .chain(BUTTONS.iter().map(|(name, ..)| *name))
            .chain([HIDDEN_BUTTON, "toolTip1"])
            .collect();
        assert_eq!(made, ours);
        assert_eq!(ours.len(), 6 + 25 + 25 + 6 + 6 + 2 + 2);
    }

    /// Each control where the `.resx` puts it, in the parent it puts it in, with its text; the
    /// hidden button hidden.
    #[test]
    fn every_control_is_where_the_resx_puts_it() {
        let Some(text) = csharp("GCSViews/ConfigurationView/ConfigArdurover.resx") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = resx(&text);
        let get = |key: String| values.get(&key).cloned();
        assert_eq!(get("$this.Size".into()), Some(pair(PAGE_SIZE)));
        let parent = |group: Option<usize>| {
            group
                .and_then(|group| GROUPS.get(group))
                .map_or("$this", |(name, ..)| *name)
                .to_owned()
        };
        for (name, text, (x, y, width, height)) in GROUPS {
            assert_eq!(get(format!("{name}.Text")).as_deref(), Some(text));
            assert_eq!(get(format!("{name}.Location")), Some(pair((x, y))));
            assert_eq!(get(format!("{name}.Size")), Some(pair((width, height))));
        }
        for spec in NUMBERS {
            let name = spec.name;
            assert_eq!(
                get(format!("{name}.Location")),
                Some(pair(spec.at)),
                "{name}"
            );
            assert_eq!(get(format!("{name}.Size")), Some(pair(BOX_SIZE)), "{name}");
            assert_eq!(
                get(format!("&gt;&gt;{name}.Parent")),
                Some(parent(Some(spec.group)))
            );
            let (label, text, place) = spec.label;
            assert_eq!(get(format!("{label}.Text")).as_deref(), Some(text));
            assert_eq!(
                get(format!("{label}.Location")),
                Some(pair(place)),
                "{label}"
            );
            assert_eq!(
                get(format!("&gt;&gt;{label}.Parent")),
                Some(parent(Some(spec.group)))
            );
        }
        for spec in COMBOS {
            let name = spec.name;
            assert_eq!(
                get(format!("{name}.Location")),
                Some(pair(spec.at)),
                "{name}"
            );
            assert_eq!(
                get(format!("{name}.Size")),
                Some(pair(COMBO_SIZE)),
                "{name}"
            );
            assert_eq!(
                get(format!("&gt;&gt;{name}.Parent")),
                Some(parent(spec.group))
            );
            let (label, text, place) = spec.label;
            assert_eq!(get(format!("{label}.Text")).as_deref(), Some(text));
            assert_eq!(
                get(format!("{label}.Location")),
                Some(pair(place)),
                "{label}"
            );
            assert_eq!(
                get(format!("&gt;&gt;{label}.Parent")),
                Some(parent(spec.group))
            );
        }
        for (name, text, (x, y, width, height)) in BUTTONS {
            assert_eq!(get(format!("{name}.Text")).as_deref(), Some(text));
            assert_eq!(get(format!("{name}.Location")), Some(pair((x, y))));
            assert_eq!(get(format!("{name}.Size")), Some(pair((width, height))));
            assert_eq!(get(format!("{name}.Visible")), None);
        }
        assert_eq!(
            get(format!("{HIDDEN_BUTTON}.Visible")).as_deref(),
            Some("False")
        );
    }

    /// `Activate`'s calls are the C#'s: each number's arguments and names, each combo's names.
    #[test]
    fn activate_binds_what_the_csharp_binds() {
        let Some(source) = csharp("GCSViews/ConfigurationView/ConfigArdurover.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let squashed: String = source.split_whitespace().collect();
        let float = |value: f32| {
            let text = value.to_string();
            if text.contains('.') {
                format!("{text}f")
            } else {
                text
            }
        };
        for spec in NUMBERS {
            let Setup {
                minimum,
                maximum,
                scale,
                increment,
            } = spec.setup;
            let names = if spec.params.len() == 1 {
                format!("\"{}\"", spec.params[0])
            } else {
                let quoted: Vec<String> = spec
                    .params
                    .iter()
                    .map(|name| format!("\"{name}\""))
                    .collect();
                format!("new[]{{{}}}", quoted.join(","))
            };
            let call = format!(
                "{}.setup({},{},{},{},{names},MainV2.comPort.MAV.param);",
                spec.name,
                float(minimum),
                float(maximum),
                float(scale),
                float(increment)
            );
            assert!(squashed.contains(&call), "{call}");
        }
        for spec in COMBOS {
            let quoted: Vec<String> = spec
                .params
                .iter()
                .map(|name| format!("\"{name}\""))
                .collect();
            let call = format!(
                "{}.setup(new[]{{{}}},MainV2.comPort.MAV.param);",
                spec.name,
                quoted.join(",")
            );
            assert!(squashed.contains(&call), "{call}");
        }
        // No control's ValueUpdated is wired: `changes` is never filled.
        assert!(!source.contains("ValueUpdated"));
        let designer =
            csharp("GCSViews/ConfigurationView/ConfigArdurover.Designer.cs").unwrap_or_default();
        assert!(!designer.contains("ValueUpdated"));
    }

    /// A rover with the current names: each control bound to the first name the vehicle has.
    #[test]
    fn activate_binds_the_rovers_names() {
        let page = shown();
        assert!(page.enabled());
        let number = |name: &str| &page.numbers()[index(name)];
        assert_eq!(number("STEER2SRV_P").param, "ATC_STR_RAT_P");
        assert_eq!(number("STEER2SRV_P").shown(), "0.20");
        assert!(number("STEER2SRV_P").enabled);
        assert_eq!(number("THR_MAX").param, "MOT_THR_MAX");
        assert_eq!(number("THR_MAX").shown(), "100");
        assert_eq!(number("TURN_G_MAX").param, "ATC_TURN_MAX_G");
        assert_eq!(number("NAVL1_DAMPING").shown(), "0.75");
        assert_eq!(number("CRUISE_THROTTLE").shown(), "50");
        // No sonar: the first name, disabled, and the group hidden.
        assert_eq!(number("SONAR_TRIGGER_CM").param, "SONAR_TRIGGER_CM");
        assert!(!number("SONAR_TRIGGER_CM").enabled);
        assert!(!page.avoidance());
        let combo = |name: &str| &page.combos()[combo_index(name)];
        assert_eq!(combo("CH7_OPTION").param, "RC7_OPTION");
        assert_eq!(combo("CH7_OPTION").text(), "Save WP");
        assert!(combo("CH7_OPTION").enabled);
        assert_eq!(combo("ATC_BRAKE").text(), "Enable");
        assert_eq!(combo("MOT_PWM_TYPE").text(), "Normal");
        // RC8 has no documentation here: an empty list, enabled, nothing selected.
        assert!(combo("CH8_OPTION").enabled);
        assert!(combo("CH8_OPTION").options.is_empty());
    }

    /// The tooltips: the numbers' and the grouped combos', the description; none for the RC
    /// options on the page itself.
    #[test]
    fn the_controls_in_the_groups_have_their_descriptions() {
        let page = shown();
        assert_eq!(
            page.number_tip(index("STEER2SRV_P")),
            "Steering control rate P gain."
        );
        assert_eq!(page.number_tip(index("WP_RADIUS")), "", "undocumented");
        assert_eq!(
            page.combo_tip(combo_index("ATC_BRAKE")),
            "Enable using reverse thrust to slow the vehicle"
        );
        assert_eq!(page.combo_tip(combo_index("CH7_OPTION")), "");
    }

    /// Not a rover, or no link: the page disabled, nothing bound, nothing written.
    #[test]
    fn the_page_is_disabled_without_a_link_or_for_another_vehicle() {
        for (connected, firmware) in [(false, Firmware::ArduRover), (true, Firmware::ArduCopter2)] {
            let mut page = RoverTuning::<usize>::default();
            let _ = page.activate(&rover(), key(), connected, firmware, rover_meta);
            assert!(!page.enabled());
            assert!(page.numbers().iter().all(|number| !number.enabled));
            assert!(page.choose(combo_index("CH7_OPTION"), 9).is_empty());
            let now = Instant::now();
            page.step(index("THR_MAX"), false, now);
            let link = Answering::new(&[]);
            run(&mut page, &link, now + WRITE_DELAY);
            assert!(link.taken().is_empty());
        }
    }

    /// The old names and a sonar: bound to them, and Avoidance shown.
    #[test]
    fn the_old_names_and_a_sonar() {
        let parameters: Vec<(String, f64)> = [
            ("STEER2SRV_P", 1.8),
            ("SPEED2THR_P", 0.3),
            ("THR_MAX", 80.0),
            ("RNGFND_TRIGGR_CM", 100.0),
            ("CH7_OPTION", 1.0),
            ("RC7_OPTION", 7.0),
        ]
        .iter()
        .map(|(name, value)| ((*name).to_owned(), *value))
        .collect();
        let mut page = RoverTuning::<usize>::default();
        let _ = page.activate(&parameters, key(), true, Firmware::ArduRover, rover_meta);
        assert_eq!(page.numbers()[index("STEER2SRV_P")].param, "STEER2SRV_P");
        assert_eq!(page.numbers()[index("THR_MAX")].param, "THR_MAX");
        assert_eq!(
            page.numbers()[index("SONAR_TRIGGER_CM")].param,
            "RNGFND_TRIGGR_CM"
        );
        assert!(page.avoidance());
        assert_eq!(page.combos()[combo_index("CH7_OPTION")].param, "CH7_OPTION");
        // A combo whose names the vehicle lacks is left as the constructor made it.
        assert!(!page.combos()[combo_index("ATC_BRAKE")].enabled);
        assert_eq!(page.combos()[combo_index("ATC_BRAKE")].param, "");
    }

    /// A number writes itself 300 ms after it changes; a combo at once; "Set NAME Failed" and
    /// "Set NAME Failed!" on the status line when they cannot.
    #[test]
    fn every_control_writes_its_own_parameter() {
        let mut page = shown();
        let now = Instant::now();
        page.type_into(index("CRUISE_SPEED"), "2.5", now);
        page.leave(now);
        let link = Answering::new(&[]);
        run(&mut page, &link, now);
        assert!(link.taken().is_empty(), "not before the timer");
        run(
            &mut page,
            &link,
            now + WRITE_DELAY + Duration::from_millis(1),
        );
        assert_eq!(link.taken(), [("CRUISE_SPEED".to_owned(), 2.5)]);

        page.toggle_dropdown(combo_index("CH7_OPTION"), now);
        let jobs = page.choose(combo_index("CH7_OPTION"), 9);
        page.push(jobs);
        run(&mut page, &link, now);
        assert_eq!(link.taken().last(), Some(&("RC7_OPTION".to_owned(), 9.0)));
        assert_eq!(page.last_write(), Some("RC7_OPTION 9 accepted"));

        let timed_out = Progress::Finished(RequestOutcome::TimedOut);
        let failing = Answering::new(&[("MOT_PWM_TYPE", timed_out), ("MOT_THR_MAX", timed_out)]);
        let jobs = page.choose(combo_index("MOT_PWM_TYPE"), 1);
        page.push(jobs);
        page.step(index("THR_MAX"), false, now);
        run(&mut page, &failing, now + WRITE_DELAY);
        // Both on the status line, the last showing, and neither a box (the owner's ruling of
        // 2026-09-25).
        assert!(page.message().is_none(), "no box: {:?}", page.message());
        assert_eq!(
            page.take_status().as_deref(),
            Some("Set MOT_THR_MAX Failed")
        );
        assert_eq!(page.take_status(), None, "taken once");
    }

    /// Write Params, and Ctrl+S, write nothing: nothing fills `changes`. The number being typed
    /// into is read as the focus leaves it, and writes itself.
    #[test]
    fn write_params_and_ctrl_s_write_nothing() {
        let mut page = shown();
        let now = Instant::now();
        page.type_into(index("WP_SPEED"), "3", now);
        let ctrl_s = KeyDownEvent {
            keystroke: Keystroke {
                modifiers: Modifiers {
                    control: true,
                    ..Modifiers::default()
                },
                key: "s".to_owned(),
                key_char: None,
            },
            is_held: false,
            prefer_character_input: false,
        };
        assert!(page.key(&ctrl_s, now), "the key goes no further");
        let link = Answering::new(&[]);
        run(&mut page, &link, now + WRITE_DELAY);
        assert!(link.taken().is_empty());
        page.press_write(now);
        assert_eq!(page.numbers()[index("WP_SPEED")].shown(), "3.0");
        run(&mut page, &link, now);
        assert!(link.taken().is_empty(), "the timer has just started");
        run(&mut page, &link, now + WRITE_DELAY);
        assert_eq!(link.taken(), [("WP_SPEED".to_owned(), 3.0)]);
    }

    /// Refresh Screen activates the page again with the vehicle's table; nothing with no link.
    #[test]
    fn refresh_screen_activates_again() {
        let mut page = shown();
        let mut changed = rover();
        for (name, value) in &mut changed {
            if name == "CRUISE_SPEED" {
                *value = 3.5;
            }
        }
        let now = Instant::now();
        page.refresh_screen(&changed, false, Firmware::ArduRover, rover_meta, now);
        assert_eq!(page.numbers()[index("CRUISE_SPEED")].shown(), "2.0");
        page.refresh_screen(&changed, true, Firmware::ArduRover, rover_meta, now);
        assert_eq!(page.numbers()[index("CRUISE_SPEED")].shown(), "3.5");
    }

    /// Every fact the GUI script asserts on is one this page records.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-rover-tuning.gui");
        let source = include_str!("rover_tuning.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            if let (Some("expect"), Some(key)) = (words.next(), words.next())
                && key.starts_with("config.rover.")
            {
                assert!(
                    source.contains(&format!("\"{key}\"")),
                    "{key} is not recorded"
                );
                facts += 1;
            }
        }
        assert!(facts >= 2, "{facts} facts");
    }
}
