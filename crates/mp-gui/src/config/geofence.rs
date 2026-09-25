//! GeoFence: `GCSViews/ConfigurationView/ConfigAC_Fence.cs`, the page CONFIG's list adds as
//! "GeoFence" for a copter once every parameter is in (`GCSViews/SoftwareConfig.cs:152-158`).
//!
//! What it shows: the heading "Geo Fence" over a rule, and a two-column table of labels and
//! `Mavlink*` controls - Enable (`FENCE_ENABLE`, a check box), Type (`FENCE_TYPE`) and Action
//! (`FENCE_ACTION`), each a combo of the parameter's documented values, Max Alt (`FENCE_ALT_MAX`),
//! Min Alt (`FENCE_ALT_MIN`), Max Radius (`FENCE_RADIUS`) and RTL Altitude (`RTL_ALT_M`, or
//! `RTL_ALT` in centimetres on firmware without it). The constructor puts the display's distance
//! unit after the four distance labels (`ConfigAC_Fence.cs:9-17`); `Activate` sets the seven
//! controls up each time the page is shown, each number scaled from the parameter's metres to
//! that unit (`:19-49`). Every control writes its own parameter: the check box and the combos
//! when they change, a number 300 ms after it changes (`Controls/MavlinkNumericUpDown.cs:154-180`).
//! Enable has a callback the C# runs after its write succeeds: ticked, it downloads the parameter
//! list again (`ConfigAC_Fence.cs:21`, `Controls/MavlinkCheckBox.cs:106-143`).
//!
//! The layout is `ConfigAC_Fence.resx`'s, every control at its `Location` - the table's controls
//! from the table's corner - in a 507 x 231 page.
//!
//! Where this differs from the C#, and why:
//!
//! * `FENCE_TYPE`'s documentation is a bitmask with no values, so its combo is enabled with an
//!   empty list, as the C#'s is: `GetParameterOptionsInt` reads the values only;
//! * `setParam` holds the UI thread until the vehicle answers; here each write goes through the
//!   link's retrying set in turn, and the callback's download starts when the write's answer
//!   comes back;
//! * the `LineSeparator`'s shaded double line is one line in this application's palette.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{AnyElement, Context, KeyDownEvent, Window, div, prelude::*, px, rgb};
use mp_vehicle::units::DisplayUnits;

use super::optional::{Event, Job, Outcome, Set, SetQueue, at, heading, label, message_box};
use crate::MissionPlanner;
use crate::config::failsafe::{CheckState, Lookup, options};
use crate::config::flight_modes::ParamWriter;
use crate::config::servo_output::{
    Check, Combo, Designer, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE,
    Question, Setup, Write, check_box, combo_box, dropdown, modal, number_box,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in CONFIG's list, `Strings.GeoFence`.
/// `// C#: GCSViews/SoftwareConfig.cs:156; ExtLibs/Strings/Strings.resx:259-261`
pub const TITLE: &str = "GeoFence";

/// The class, as the lists name it.
pub const CLASS: &str = "ConfigAC_Fence";

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx ($this.Size)`
pub const PAGE_SIZE: (f32, f32) = (507.0, 231.0);

/// `label1gftitle.Text`, in "Microsoft Sans Serif, 12pt", at its `Location`.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx (label1gftitle.*)`
pub const HEADING: (&str, (f32, f32)) = ("Geo Fence", (19.0, 4.0));

/// `lineSeparator2`'s `Location` and `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx (lineSeparator2.*)`
pub const SEPARATOR: (f32, f32, f32, f32) = (23.0, 27.0, 460.0, 2.0);

/// `tableLayoutPanel1`'s `Location`: the controls in it are placed from here.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx (tableLayoutPanel1.Location)`
pub const TABLE_AT: (f32, f32) = (23.0, 35.0);

/// A label of the table: its Designer name, `Text`, `Location` in the table, and whether the
/// constructor puts the distance unit after it.
pub type LabelSpec = (&'static str, &'static str, (f32, f32), bool);

/// The table's seven labels, in its rows' order.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx (label*.Text, label*.Location);
/// ConfigAC_Fence.cs:13-16`
pub const LABELS: [LabelSpec; 7] = [
    ("label3enable", "Enable", (3.0, 0.0), false),
    ("label4type", "Type", (3.0, 25.0), false),
    ("label5action", "Action", (3.0, 50.0), false),
    ("label6maxalt", "Max Alt", (3.0, 75.0), true),
    ("label8minalt", "Min Alt", (3.0, 100.0), true),
    ("label7maxrad", "Max Radius", (3.0, 125.0), true),
    ("label2rtlalt", "RTL Altitude", (3.0, 145.0), true),
];

/// `mavlinkCheckBox1`: its `Text` and `Location` in the table.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx (mavlinkCheckBox1.*)`
pub const ENABLE: (&str, (f32, f32)) = ("Enable", (136.0, 3.0));

/// The two combos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Which {
    /// `mavlinkComboBox1`, `FENCE_TYPE`.
    Type,
    /// `mavlinkComboBox2`, `FENCE_ACTION`.
    Action,
}

impl Which {
    /// Both, in the table's order.
    pub const ALL: [Self; 2] = [Self::Type, Self::Action];

    /// The Designer's name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Type => "mavlinkComboBox1",
            Self::Action => "mavlinkComboBox2",
        }
    }

    /// The parameter `Activate` binds.
    /// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.cs:23-30`
    #[must_use]
    pub const fn param(self) -> &'static str {
        match self {
            Self::Type => "FENCE_TYPE",
            Self::Action => "FENCE_ACTION",
        }
    }

    /// `Location` in the table and `Size`.
    /// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx (mavlinkComboBox*.Location, .Size)`
    #[must_use]
    pub const fn place(self) -> (f32, f32, f32, f32) {
        match self {
            Self::Type => (136.0, 28.0, 121.0, 20.0),
            Self::Action => (136.0, 53.0, 121.0, 20.0),
        }
    }
}

/// One `MavlinkNumericUpDown`: its Designer name, `Location` in the table, the Designer's
/// `Minimum` and `Value`, and `setup`'s `Min`, `Max` and `Increment`; the scale is the display
/// unit's, from `Activate`.
#[derive(Debug, Clone, Copy)]
pub struct NumberSpec {
    /// The Designer's name.
    pub name: &'static str,
    /// `Location` in the table.
    pub at: (f32, f32),
    /// As the Designer leaves it.
    pub designer: Designer,
    /// `setup`'s `Min`, `Max` and `Increment`.
    pub bounds: (f32, f32, f32),
}

/// A `NumericUpDown` the Designer gives a `Minimum` and a `Value`.
const fn designed(minimum: f64, value: f64) -> Designer {
    Designer {
        minimum,
        maximum: NUMERIC_DEFAULTS.maximum,
        value,
        decimals: 0,
    }
}

/// Every box's `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.resx (mavlinkNumericUpDown*.Size)`
pub const BOX_SIZE: (f32, f32) = (120.0, 19.0);

/// The four numbers, in the table's rows' order.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.cs:34-48; ConfigAC_Fence.Designer.cs:94-192;
/// ConfigAC_Fence.resx (mavlinkNumericUpDown*.Location)`
pub const NUMBERS: [NumberSpec; 4] = [
    NumberSpec {
        name: "mavlinkNumericUpDown1",
        at: (136.0, 78.0),
        designer: NUMERIC_DEFAULTS,
        bounds: (10.0, 1000.0, 1.0),
    },
    NumberSpec {
        name: "mavlinkNumericUpDown4",
        at: (136.0, 103.0),
        designer: designed(-100.0, -10.0),
        bounds: (-100.0, 100.0, 1.0),
    },
    NumberSpec {
        name: "mavlinkNumericUpDown2",
        at: (136.0, 128.0),
        designer: designed(30.0, 30.0),
        bounds: (30.0, 65536.0, 1.0),
    },
    NumberSpec {
        name: "mavlinkNumericUpDown3",
        at: (136.0, 148.0),
        designer: NUMERIC_DEFAULTS,
        bounds: (1.0, 500.0, 1.0),
    },
];

/// `NUMBERS`' indices.
const ALT_MAX: usize = 0;
const ALT_MIN: usize = 1;
const RADIUS: usize = 2;
const RTL: usize = 3;

/// `(float)CurrentState.fromDistDisplayUnit(input)`: `input / multiplierdist` in doubles, as a
/// float.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:4375-4378`
#[must_use]
#[allow(clippy::cast_possible_truncation)] // `(float)`
pub fn from_dist_display_unit(input: f64, units: DisplayUnits) -> f32 {
    (input / f64::from(units.dist)) as f32
}

/// The page object.
#[derive(Debug)]
pub struct GeoFence<H = mp_link::RequestId> {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// `"[" + CurrentState.DistanceUnit + "]"`, as the constructor found it.
    unit: String,
    /// `mavlinkCheckBox1`.
    enable: Check,
    /// `mavlinkComboBox1` and `mavlinkComboBox2`, in [`Which::ALL`] order.
    combos: [Combo; 2],
    /// The numbers, in [`NUMBERS`]' order.
    numbers: [Number; 4],
    /// The combo whose list is down.
    dropdown: Option<Which>,
    /// The number being typed into.
    editing: Option<usize>,
    /// A number's "Out of range" question.
    question: Option<(usize, Question)>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The controls' writes.
    queue: SetQueue<H>,
    /// How many times Enable's callback has downloaded the parameters.
    downloads: usize,
}

impl<H> Default for GeoFence<H> {
    /// `InitializeComponent`: every control disabled, each number as the Designer leaves it.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            unit: "[]".to_owned(),
            enable: Check::default(),
            combos: std::array::from_fn(|_| Combo::default()),
            numbers: std::array::from_fn(|index| {
                Number::new(
                    NUMBERS
                        .get(index)
                        .map_or(NUMERIC_DEFAULTS, |spec| spec.designer),
                )
            }),
            dropdown: None,
            editing: None,
            question: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
            downloads: 0,
        }
    }
}

impl<H: Copy> GeoFence<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// A label's text, the unit after it where the constructor puts one.
    #[must_use]
    pub fn label_text(&self, spec: &LabelSpec) -> String {
        let (_, text, _, unit) = *spec;
        if unit {
            format!("{text}{}", self.unit)
        } else {
            text.to_owned()
        }
    }

    /// Enable.
    #[must_use]
    pub const fn enable(&self) -> &Check {
        &self.enable
    }

    /// A combo.
    #[must_use]
    pub const fn combo(&self, which: Which) -> &Combo {
        let [kind, action] = &self.combos;
        match which {
            Which::Type => kind,
            Which::Action => action,
        }
    }

    const fn combo_mut(&mut self, which: Which) -> &mut Combo {
        let [kind, action] = &mut self.combos;
        match which {
            Which::Type => kind,
            Which::Action => action,
        }
    }

    /// The numbers, in [`NUMBERS`]' order.
    #[must_use]
    pub const fn numbers(&self) -> &[Number; 4] {
        &self.numbers
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

    /// How many times Enable's callback has asked for the parameter list.
    #[must_use]
    pub const fn downloads(&self) -> usize {
        self.downloads
    }

    /// The page object: the constructor adds the distance unit to four labels.
    /// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.cs:9-17`
    fn construct(&mut self, units: DisplayUnits) {
        self.unit = format!("[{}]", units.dist_unit);
    }

    /// The page object let go with its screen, returning what its numbers' timers held: the
    /// C#'s timer is the control's, and writes after the page is gone.
    fn dispose(&mut self) -> Vec<Write> {
        let pending = self.numbers.iter_mut().filter_map(Number::flush).collect();
        *self = Self {
            messages: std::mem::take(&mut self.messages),
            queue: std::mem::take(&mut self.queue),
            downloads: self.downloads,
            ..Self::default()
        };
        pending
    }

    /// Shows the page: a new page object for a new screen, then `Activate`. Returns the writes a
    /// disposed page object's timers held.
    /// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.cs:19-49`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        units: DisplayUnits,
        lookup: Lookup,
    ) -> Vec<Job> {
        let mut jobs = Vec::new();
        if self.made_for != Some(key) {
            jobs.extend(self.dispose().into_iter().map(Job::control));
            self.made_for = Some(key);
            self.construct(units);
        }
        self.active = true;
        self.dropdown = None;
        // `// C#: :21`
        self.enable.setup(1.0, 0.0, "FENCE_ENABLE", parameters);
        // `// C#: :23-30`
        for which in Which::ALL {
            let param = which.param();
            self.combo_mut(which)
                .setup(options(param, lookup), param, parameters);
        }
        // `// C#: :34-41`
        let unit = from_dist_display_unit(1.0, units);
        for (index, param) in [
            (ALT_MAX, "FENCE_ALT_MAX"),
            (ALT_MIN, "FENCE_ALT_MIN"),
            (RADIUS, "FENCE_RADIUS"),
        ] {
            self.setup_number(index, unit, param, parameters, lookup);
        }
        // `RTL_ALT_M` in metres where the vehicle has it, else `RTL_ALT` in centimetres.
        // `// C#: :43-48`
        if super::optional::has(parameters, "RTL_ALT_M") {
            self.setup_number(RTL, unit, "RTL_ALT_M", parameters, lookup);
        } else {
            let centimetres = from_dist_display_unit(100.0, units);
            self.setup_number(RTL, centimetres, "RTL_ALT", parameters, lookup);
        }
        jobs
    }

    /// `setup(Min, Max, scale, Increment, param, MAV.param)` for a number.
    fn setup_number(
        &mut self,
        index: usize,
        scale: f32,
        param: &str,
        parameters: &[(String, f64)],
        lookup: Lookup,
    ) {
        let Some((number, spec)) = self.numbers.get_mut(index).zip(NUMBERS.get(index)) else {
            return;
        };
        let (minimum, maximum, increment) = spec.bounds;
        number.setup(
            Setup {
                minimum,
                maximum,
                scale,
                increment,
            },
            param,
            parameters,
            lookup,
        );
    }

    /// The page hidden: `ConfigAC_Fence` is `IActivate` only, so nothing runs but the number being
    /// typed into losing the focus, which reads its text.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = None;
        self.leave(now);
    }

    /// A click on Enable: the control's own write.
    /// `// C#: Controls/MavlinkCheckBox.cs:106-143`
    pub fn click_enable(&mut self, now: Instant) -> Vec<Job> {
        self.leave(now);
        self.dropdown = None;
        self.enable
            .click()
            .map(|write| Job::new("enable", [Set::control(write)]))
            .into_iter()
            .collect()
    }

    /// Drops a combo's list down, or back up.
    pub fn toggle_dropdown(&mut self, which: Which, now: Instant) {
        self.leave(now);
        self.dropdown = if self.dropdown == Some(which) || !self.combo(which).enabled {
            None
        } else {
            self.combo_mut(which).open_list();
            Some(which)
        };
    }

    /// The wheel over a list.
    pub fn scroll_list(&mut self, which: Which, lines: i32) {
        if self.dropdown == Some(which) {
            self.combo_mut(which).scroll_list(lines);
        }
    }

    /// A row chosen: the control's own write.
    /// `// C#: Controls/MavlinkComboBox.cs:133-200`
    pub fn choose(&mut self, which: Which, key: i64) -> Vec<Job> {
        self.dropdown = None;
        self.combo_mut(which)
            .choose(key)
            .map(Job::control)
            .into_iter()
            .collect()
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) {
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

    /// A key for the number being typed into.
    pub fn key(&mut self, event: &KeyDownEvent, now: Instant) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
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

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// The numbers' timers, then the writes as far as the link's answers allow. Returns whether
    /// Enable's callback asks for the parameter list: its write returned true - accepted, or the
    /// value already held - and the box is ticked.
    /// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.cs:21; Controls/MavlinkCheckBox.cs:111-141`
    pub fn advance<W: ParamWriter<Handle = H>>(&mut self, writer: &W, now: Instant) -> bool {
        let due: Vec<Job> = self
            .numbers
            .iter_mut()
            .filter_map(|number| number.due(now))
            .map(Job::control)
            .collect();
        self.queue.push(due);
        let mut download = false;
        for event in self.queue.advance(writer, &mut self.messages) {
            if let Event::Set { param, outcome, .. } = event
                && param == "FENCE_ENABLE"
                && matches!(outcome, Outcome::Accepted(_) | Outcome::Unchanged)
                && self.enable.state == CheckState::Checked
            {
                download = true;
            }
        }
        if download {
            self.downloads += 1;
        }
        download
    }
}

impl GeoFence {
    /// Once a frame: a page object whose screen has gone is let go, a number the focus has left
    /// is read, the timers, the writes, and the callback's download.
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
        if self.advance(telemetry, now) {
            // `MainV2.comPort.getParamList()`. `// C#: ConfigAC_Fence.cs:21`
            telemetry.download_parameters();
        }
    }
}

/// A combo's selection for the facts, "none" when nothing is selected.
fn selection(combo: &Combo) -> String {
    combo
        .selected
        .map_or_else(|| "none".to_owned(), |value| value.to_string())
}

/// Facts a UI test asserts on.
pub fn record_facts<H: Copy>(page: &GeoFence<H>, view: &TelemetryView) {
    use crate::facts::record;
    record("config.fence.active", page.is_active());
    record("config.fence.heading", HEADING.0);
    for spec in &LABELS {
        record(format!("config.fence.{}", spec.0), page.label_text(spec));
    }
    record("config.fence.enable", page.enable().state.key());
    record("config.fence.enable.enabled", page.enable().enabled);
    for which in Which::ALL {
        let combo = page.combo(which);
        let key = format!("config.fence.{}", which.name());
        record(format!("{key}.text"), combo.text());
        record(format!("{key}.enabled"), combo.enabled);
        record(format!("{key}.param"), &combo.param);
        record(format!("{key}.options"), combo.options.len());
        record(key, selection(combo));
    }
    for (number, spec) in page.numbers().iter().zip(NUMBERS) {
        let key = format!("config.fence.{}", spec.name);
        record(format!("{key}.enabled"), number.enabled);
        record(format!("{key}.param"), &number.param);
        record(key, number.shown());
    }
    record(
        "config.fence.editing",
        page.editing()
            .and_then(|index| NUMBERS.get(index))
            .map_or("none", |spec| spec.name),
    );
    record(
        "config.fence.question",
        page.question()
            .map_or_else(|| "none".to_owned(), Question::text),
    );
    record(
        "config.fence.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    record("config.fence.write", page.last_write().unwrap_or("none"));
    record("config.fence.writes.pending", page.pending());
    record("config.fence.downloads", page.downloads());
    for (name, value) in view.parameters.iter() {
        if name.starts_with("FENCE_") || name.starts_with("RTL_ALT") {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The page, laid out as `ConfigAC_Fence.resx` lays it out.
/// `// C#: GCSViews/ConfigurationView/ConfigAC_Fence.Designer.cs:29-220; ConfigAC_Fence.resx`
pub fn page(
    fence: &GeoFence,
    handle: &gpui::FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (tx, ty) = TABLE_AT;
    let (sx, sy, sw, sh) = SEPARATOR;
    let mut body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(heading(HEADING.1.0, HEADING.1.1, HEADING.0, true))
        .child(at(sx, sy, sw, sh).bg(rgb(theme::BORDER)));
    for spec in &LABELS {
        let (_, _, (x, y), _) = *spec;
        body = body.child(label(tx + x, ty + y, fence.label_text(spec), true));
    }
    let (ex, ey) = ENABLE.1;
    body = body.child(check_box(
        "fence-mavlinkCheckBox1".to_owned(),
        fence.enable(),
        ENABLE.0,
        (tx + ex, ty + ey),
        |this| {
            let jobs = this.software_pages.geofence.click_enable(Instant::now());
            this.software_pages.geofence.push(jobs);
        },
        cx,
    ));
    for which in Which::ALL {
        let (x, y, width, height) = which.place();
        body = body.child(combo_box(
            format!("fence-{}", which.name()),
            fence.combo(which),
            (tx + x, ty + y, width, height),
            move |this| {
                this.software_pages
                    .geofence
                    .toggle_dropdown(which, Instant::now());
            },
            cx,
        ));
    }
    for (index, (number, spec)) in fence.numbers().iter().zip(NUMBERS).enumerate() {
        let (x, y) = spec.at;
        body = body.child(number_box(
            format!("fence-{}", spec.name),
            number,
            fence.editing() == Some(index),
            handle,
            (tx + x, ty + y, BOX_SIZE.0, BOX_SIZE.1),
            NumberHandlers {
                begin: move |this: &mut MissionPlanner| {
                    this.software_pages.geofence.begin(index, Instant::now());
                },
                key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                    this.software_pages.geofence.key(event, Instant::now())
                },
                step: move |this: &mut MissionPlanner, up: bool| {
                    this.software_pages.geofence.step(index, up, Instant::now());
                },
            },
            window,
            cx,
        ));
    }
    if let Some(which) = fence.dropdown {
        let (x, y, width, height) = which.place();
        body = body.child(dropdown(
            &format!("fence-{}", which.name()),
            fence.combo(which),
            (tx + x, ty + y + height, width),
            move |this, key| {
                let jobs = this.software_pages.geofence.choose(which, key);
                this.software_pages.geofence.push(jobs);
            },
            move |this, lines| this.software_pages.geofence.scroll_list(which, lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    fence: &GeoFence,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = fence.question() {
        let buttons = vec![
            action(
                "fence-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.software_pages.geofence.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "fence-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.software_pages.geofence.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "fence-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = fence.message()?;
    Some(message_box(
        "fence-message",
        "fence-message-ok",
        message,
        window,
        |this| this.software_pages.geofence.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::time::Duration;

    use mp_link::requests::RequestOutcome;

    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::error;
    use crate::config::optional::tests::Answering;
    use crate::config::servo_output::WRITE_DELAY;
    use crate::config_coverage::source::{csharp, resx};

    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// SITL's copter as `tools/sitl/params/copter.parm` leaves it, with `RTL_ALT_M`.
    fn sitl() -> Vec<(String, f64)> {
        table(&[
            ("FENCE_ENABLE", 0.0),
            ("FENCE_TYPE", 7.0),
            ("FENCE_ACTION", 1.0),
            ("FENCE_ALT_MAX", 100.0),
            ("FENCE_ALT_MIN", -10.0),
            ("FENCE_RADIUS", 150.0),
            ("RTL_ALT_M", 15.0),
        ])
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// Another screen: a vehicle heard from.
    fn other_key() -> Key {
        let mut view = TelemetryView::disconnected("other");
        view.vehicle = Some(mp_vehicle::VehicleId::new(1, 1));
        Key::of(&view)
    }

    fn metres() -> DisplayUnits {
        DisplayUnits::default().change_units(None, None, None)
    }

    fn feet() -> DisplayUnits {
        DisplayUnits::default().change_units(Some("Feet"), None, None)
    }

    fn shown(parameters: &[(String, f64)], units: DisplayUnits) -> GeoFence<usize> {
        let mut page = GeoFence::<usize>::default();
        let jobs = page.activate(parameters, key(), units, bundled);
        assert!(jobs.is_empty());
        page
    }

    fn text_of(page: &GeoFence<usize>, name: &str) -> String {
        let index = NUMBERS
            .iter()
            .position(|spec| spec.name == name)
            .unwrap_or_else(|| panic!("{name}"));
        page.numbers()[index].shown().to_owned()
    }

    fn param_of(page: &GeoFence<usize>, name: &str) -> String {
        let index = NUMBERS
            .iter()
            .position(|spec| spec.name == name)
            .unwrap_or_else(|| panic!("{name}"));
        page.numbers()[index].param.clone()
    }

    /// Runs the writes until none is left, starting the numbers' timers' writes first.
    fn run(page: &mut GeoFence<usize>, link: &Answering, now: Instant) -> bool {
        let mut download = false;
        for _ in 0..100 {
            download |= page.advance(link, now);
            if page.pending() == 0 {
                break;
            }
        }
        download
    }

    /// Every control the Designer makes is drawn: the heading, the rule, the seven labels, the
    /// check box, the two combos and the four numbers - and the table they sit in.
    #[test]
    fn every_designer_control_is_drawn() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigAC_Fence.Designer.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let made: BTreeSet<&str> = designer
            .lines()
            .filter_map(|line| line.trim().strip_prefix("this."))
            .filter(|line| line.contains(" = new "))
            .filter_map(|line| line.split_once(" = new "))
            .filter(|(name, _)| !name.contains('.'))
            .map(|(name, _)| name)
            .collect();
        let ours: BTreeSet<&str> = LABELS
            .iter()
            .map(|spec| spec.0)
            .chain(Which::ALL.iter().map(|which| which.name()))
            .chain(NUMBERS.iter().map(|spec| spec.name))
            .chain([
                "label1gftitle",
                "lineSeparator2",
                "tableLayoutPanel1",
                "mavlinkCheckBox1",
            ])
            .collect();
        assert_eq!(made, ours);
    }

    /// The `.resx`'s words and places.
    #[test]
    fn the_text_and_places_are_the_resx_ones() {
        let Some(text) = csharp("GCSViews/ConfigurationView/ConfigAC_Fence.resx") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = resx(&text);
        let get = |key: &str| values.get(key).map(String::as_str);
        let pair = |(x, y): (f32, f32)| format!("{x}, {y}");
        assert_eq!(get("$this.Size"), Some(pair(PAGE_SIZE).as_str()));
        assert_eq!(get("label1gftitle.Text"), Some(HEADING.0));
        assert_eq!(
            get("label1gftitle.Location"),
            Some(pair(HEADING.1).as_str())
        );
        assert_eq!(
            get("label1gftitle.Font"),
            Some("Microsoft Sans Serif, 12pt")
        );
        let (sx, sy, sw, sh) = SEPARATOR;
        assert_eq!(
            get("lineSeparator2.Location"),
            Some(pair((sx, sy)).as_str())
        );
        assert_eq!(get("lineSeparator2.Size"), Some(pair((sw, sh)).as_str()));
        assert_eq!(
            get("tableLayoutPanel1.Location"),
            Some(pair(TABLE_AT).as_str())
        );
        for (name, text, place, _) in LABELS {
            assert_eq!(get(&format!("{name}.Text")), Some(text), "{name}");
            assert_eq!(
                get(&format!("{name}.Location")),
                Some(pair(place).as_str()),
                "{name}"
            );
        }
        assert_eq!(get("mavlinkCheckBox1.Text"), Some(ENABLE.0));
        assert_eq!(
            get("mavlinkCheckBox1.Location"),
            Some(pair(ENABLE.1).as_str())
        );
        for which in Which::ALL {
            let (x, y, width, height) = which.place();
            let name = which.name();
            assert_eq!(
                get(&format!("{name}.Location")),
                Some(pair((x, y)).as_str())
            );
            assert_eq!(
                get(&format!("{name}.Size")),
                Some(pair((width, height)).as_str())
            );
        }
        for spec in NUMBERS {
            let name = spec.name;
            assert_eq!(
                get(&format!("{name}.Location")),
                Some(pair(spec.at).as_str())
            );
            assert_eq!(get(&format!("{name}.Size")), Some(pair(BOX_SIZE).as_str()));
        }
    }

    /// `Activate`'s calls, as the C# makes them: the parameter each binds and its arguments.
    #[test]
    fn activate_binds_what_the_csharp_binds() {
        let Some(source) = csharp("GCSViews/ConfigurationView/ConfigAC_Fence.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let squashed: String = source.split_whitespace().collect();
        for (spec, param) in NUMBERS.iter().zip([
            "FENCE_ALT_MAX",
            "FENCE_ALT_MIN",
            "FENCE_RADIUS",
            "RTL_ALT_M",
        ]) {
            let (minimum, maximum, increment) = spec.bounds;
            let designer_number = spec.name;
            let call = format!(
                "{designer_number}.setup({minimum},{maximum},(float)CurrentState.fromDistDisplayUnit(1),{increment},\"{param}\""
            );
            assert!(squashed.contains(&call), "{call}");
        }
        assert!(squashed.contains(
            "mavlinkNumericUpDown3.setup(1,500,(float)CurrentState.fromDistDisplayUnit(100),1,\"RTL_ALT\""
        ));
        assert!(squashed.contains("mavlinkCheckBox1.setup(1,0,\"FENCE_ENABLE\""));
        for which in Which::ALL {
            let param = which.param();
            assert!(squashed.contains(&format!(
                "{}.setup(ParameterMetaDataRepository.GetParameterOptionsInt(\"{param}\"",
                which.name()
            )));
        }
    }

    /// SITL's copter in metres: every control enabled and showing the vehicle's value; the
    /// labels carry the unit; FENCE_TYPE's bitmask has no values, so its combo is empty.
    #[test]
    fn activate_binds_the_sitl_copter() {
        let page = shown(&sitl(), metres());
        assert!(page.is_active());
        assert_eq!(page.enable().state, CheckState::Unchecked);
        assert!(page.enable().enabled);
        let action = page.combo(Which::Action);
        assert!(action.enabled);
        assert_eq!(action.text(), "RTL or Land");
        assert_eq!(action.options.len(), 6);
        let kind = page.combo(Which::Type);
        assert!(kind.enabled, "the vehicle has FENCE_TYPE");
        assert!(kind.options.is_empty(), "a bitmask has no values");
        assert_eq!(kind.text(), "");
        assert_eq!(text_of(&page, "mavlinkNumericUpDown1"), "100");
        assert_eq!(text_of(&page, "mavlinkNumericUpDown4"), "-10");
        assert_eq!(text_of(&page, "mavlinkNumericUpDown2"), "150");
        assert_eq!(text_of(&page, "mavlinkNumericUpDown3"), "15");
        assert_eq!(param_of(&page, "mavlinkNumericUpDown3"), "RTL_ALT_M");
        let texts: Vec<String> = LABELS.iter().map(|spec| page.label_text(spec)).collect();
        assert_eq!(
            texts,
            [
                "Enable",
                "Type",
                "Action",
                "Max Alt[m]",
                "Min Alt[m]",
                "Max Radius[m]",
                "RTL Altitude[m]"
            ]
        );
    }

    /// Firmware before `RTL_ALT_M`: `RTL_ALT`'s centimetres shown as metres; and in feet, every
    /// distance scaled by `fromDistDisplayUnit`.
    #[test]
    fn rtl_alt_in_centimetres_and_the_units_scale() {
        let mut old = sitl();
        old.retain(|(name, _)| name != "RTL_ALT_M");
        old.push(("RTL_ALT".to_owned(), 1500.0));
        let page = shown(&old, metres());
        assert_eq!(param_of(&page, "mavlinkNumericUpDown3"), "RTL_ALT");
        assert_eq!(text_of(&page, "mavlinkNumericUpDown3"), "15");

        let page = shown(&old, feet());
        assert_eq!(page.label_text(&LABELS[3]), "Max Alt[ft]");
        // 100 m / (1 / 3.28084) is 328.084 ft, shown to the places the value has.
        let shown_ft: f64 = text_of(&page, "mavlinkNumericUpDown1")
            .parse()
            .expect("a number");
        assert!((shown_ft - 328.084).abs() < 1e-3, "{shown_ft}");
        let rtl_ft: f64 = text_of(&page, "mavlinkNumericUpDown3")
            .parse()
            .expect("a number");
        assert!((rtl_ft - 49.2126).abs() < 1e-3, "{rtl_ft}");
    }

    /// The constructor's unit is the one the page object was made with; `Activate` does not put
    /// it on again.
    #[test]
    fn the_unit_is_added_once_by_the_constructor() {
        let mut page = shown(&sitl(), metres());
        let _ = page.activate(&sitl(), key(), feet(), bundled);
        assert_eq!(page.label_text(&LABELS[6]), "RTL Altitude[m]");
    }

    /// A vehicle without the fence's parameters: every control disabled.
    #[test]
    fn without_the_parameters_every_control_is_disabled() {
        let page = shown(&[], metres());
        assert!(!page.enable().enabled);
        assert!(!page.combo(Which::Type).enabled);
        assert!(!page.combo(Which::Action).enabled);
        assert!(page.numbers().iter().all(|number| !number.enabled));
        // The Designer's values stand.
        assert_eq!(text_of(&page, "mavlinkNumericUpDown2"), "30");
        assert_eq!(text_of(&page, "mavlinkNumericUpDown4"), "-10");
    }

    /// Enable writes FENCE_ENABLE, and ticked, the callback downloads the parameter list once
    /// the write has returned true; unticked, or refused, it does not.
    #[test]
    fn enable_writes_and_its_callback_downloads_when_ticked() {
        let mut page = shown(&sitl(), metres());
        let now = Instant::now();
        let jobs = page.click_enable(now);
        page.push(jobs);
        let link = Answering::new(&[]);
        assert!(run(&mut page, &link, now), "ticked and accepted");
        assert_eq!(link.taken(), [("FENCE_ENABLE".to_owned(), 1.0)]);
        assert_eq!(page.downloads(), 1);

        let jobs = page.click_enable(now);
        page.push(jobs);
        assert!(!run(&mut page, &link, now), "unticked: no download");
        assert_eq!(link.taken().last(), Some(&("FENCE_ENABLE".to_owned(), 0.0)));

        let refused = Answering::new(&[(
            "FENCE_ENABLE",
            Progress::Finished(RequestOutcome::UnknownParameter),
        )]);
        let jobs = page.click_enable(now);
        page.push(jobs);
        assert!(!run(&mut page, &refused, now), "false: no callback");
        assert_eq!(page.message(), Some(&error("Set FENCE_ENABLE Failed")));
        assert_eq!(page.downloads(), 1);
    }

    /// Choosing an action writes it, as `MavlinkComboBox` does, "Set NAME Failed!" on a timeout.
    #[test]
    fn choosing_an_action_writes_it() {
        let mut page = shown(&sitl(), metres());
        let now = Instant::now();
        page.toggle_dropdown(Which::Action, now);
        let jobs = page.choose(Which::Action, 2);
        page.push(jobs);
        let link = Answering::new(&[]);
        run(&mut page, &link, now);
        assert_eq!(link.taken(), [("FENCE_ACTION".to_owned(), 2.0)]);
        assert_eq!(page.combo(Which::Action).text(), "Always Land");
        assert_eq!(page.last_write(), Some("FENCE_ACTION 2 accepted"));

        let timed_out =
            Answering::new(&[("FENCE_ACTION", Progress::Finished(RequestOutcome::TimedOut))]);
        let jobs = page.choose(Which::Action, 1);
        page.push(jobs);
        run(&mut page, &timed_out, now);
        assert_eq!(page.message(), Some(&error("Set FENCE_ACTION Failed!")));
    }

    /// A radius typed is written 300 ms later, times the scale; in feet, the metres the vehicle
    /// keeps.
    #[test]
    fn a_number_writes_its_value_after_the_timer() {
        let mut page = shown(&sitl(), metres());
        let now = Instant::now();
        page.type_into(RADIUS, "200", now);
        page.leave(now);
        let link = Answering::new(&[]);
        run(&mut page, &link, now);
        assert!(link.taken().is_empty(), "not before the timer");
        let later = now + WRITE_DELAY + Duration::from_millis(1);
        run(&mut page, &link, later);
        assert_eq!(link.taken(), [("FENCE_RADIUS".to_owned(), 200.0)]);
        assert_eq!(page.last_write(), Some("FENCE_RADIUS 200 accepted"));

        let mut page = shown(&sitl(), feet());
        page.type_into(ALT_MAX, "656", now);
        page.leave(now);
        let link = Answering::new(&[]);
        run(&mut page, &link, later);
        let (name, value) = link.taken().first().cloned().expect("a write");
        assert_eq!(name, "FENCE_ALT_MAX");
        assert!((value - 199.949).abs() < 1e-2, "{value}");
    }

    /// The RTL altitude in centimetres: shown in metres, written in centimetres.
    #[test]
    fn rtl_alt_writes_centimetres() {
        let mut old = sitl();
        old.retain(|(name, _)| name != "RTL_ALT_M");
        old.push(("RTL_ALT".to_owned(), 1500.0));
        let mut page = shown(&old, metres());
        let now = Instant::now();
        page.step(RTL, true, now);
        let link = Answering::new(&[]);
        run(&mut page, &link, now + WRITE_DELAY);
        assert_eq!(link.taken(), [("RTL_ALT".to_owned(), 1600.0)]);
    }

    /// A value above the maximum asks first; Yes takes it and writes it.
    #[test]
    fn a_value_above_the_maximum_asks() {
        let mut page = shown(&sitl(), metres());
        let now = Instant::now();
        page.type_into(ALT_MAX, "5000", now);
        page.leave(now);
        assert_eq!(
            page.question().map(Question::text).as_deref(),
            Some("FENCE_ALT_MAX Value out of range\nDo you want to accept the new value?")
        );
        page.answer(true, now);
        let link = Answering::new(&[]);
        run(&mut page, &link, now + WRITE_DELAY);
        assert_eq!(link.taken(), [("FENCE_ALT_MAX".to_owned(), 5000.0)]);
    }

    /// A timer running when the screen is left still writes: the C#'s timer is the control's.
    #[test]
    fn a_timer_running_when_the_page_object_goes_still_writes() {
        let mut page = shown(&sitl(), metres());
        let now = Instant::now();
        page.step(RADIUS, true, now);
        page.hide(now);
        let other = other_key();
        let jobs = page.activate(&sitl(), other, metres(), bundled);
        assert_eq!(jobs.len(), 1);
        page.push(jobs);
        let link = Answering::new(&[]);
        run(&mut page, &link, now);
        assert_eq!(link.taken(), [("FENCE_RADIUS".to_owned(), 151.0)]);
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-geofence.gui");
        let source = include_str!("geofence.rs");
        let names: Vec<&str> = LABELS
            .iter()
            .map(|spec| spec.0)
            .chain(Which::ALL.iter().map(|which| which.name()))
            .chain(NUMBERS.iter().map(|spec| spec.name))
            .collect();
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.fence.") => {
                    // A control's facts are recorded under its Designer name; the rest by name.
                    let rest = key.trim_start_matches("config.fence.");
                    let head = rest.split('.').next().unwrap_or(rest);
                    assert!(
                        names.contains(&head) || source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("fence-") => {
                    // `fence-<control>`, a list's row `fence-<combo>-<value>`, an arrow
                    // `fence-<number>-up`, or a box of the overlay.
                    let id = id.split(':').next().unwrap_or(id);
                    let name = id.trim_start_matches("fence-");
                    let control = name.split('-').next().unwrap_or(name);
                    assert!(
                        names.contains(&control)
                            || control == "mavlinkCheckBox1"
                            || source.contains(&format!("\"{id}\"")),
                        "{id} is not drawn"
                    );
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 10 && clicks >= 3, "{facts} facts, {clicks} clicks");
    }
}
