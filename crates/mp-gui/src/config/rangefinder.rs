//! Range Finder: `GCSViews/ConfigurationView/ConfigHWRangeFinder.cs`, an Optional Hardware page
//! of Initial Setup (`GCSViews/InitialSetup.cs:287-290`), listed once every parameter is in.
//!
//! What it shows: the heading, the sensor's picture, the type (`RNGFND_TYPE`) as a
//! `MavlinkComboBox` of the parameter's documented values, and the vehicle's rangefinder distance
//! and voltage, which a 200 ms timer copies from `cs.sonarrange` and `cs.sonarvoltage` while the
//! page shows (`ConfigHWRangeFinder.cs:17-45`). The combo writes `RNGFND_TYPE` itself; the page's
//! handler, which runs before it, sets `RNGFND_MAX_CM` and `RNGFND_MIN_CM` for a
//! "TeraRangerOne-I2C" (`:47-58`) - a name no current firmware documents, so the branch is
//! reached only with documentation that has it, and its second name carries the C#'s trailing
//! space, which no vehicle lists.
//!
//! The combo's `RNGFND_TYPE` is the name of firmware before 4.0; later firmware numbers its
//! rangefinders (`RNGFND1_TYPE`), and on it the combo stays disabled, as in the C#.
//!
//! The layout is `ConfigHWRangeFinder.resx`'s, every control at its `Location` in a 650 x 116 page.
//!
//! What is not ported, and why:
//!
//! * `pictureBox3`'s image (`Resources.sonar`), a resource this application does not carry;
//! * the report Mission Planner offers to send when the handler's `setParam` times out: its
//!   handler has no `try`, so the `TimeoutException` reaches `Program.handleException`
//!   (`Program.cs:717-800`), whose box is shown here with OK alone - there is no error-report
//!   service for its Yes to send to.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use gpui::{AnyElement, Context, Window, div, prelude::*, px, rgb};

use super::battery_monitor::float_text;
use super::optional::{
    Job, Set, SetQueue, heading, message_box, picture, rule, timeout_text, value_of,
};
use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::config::servo_output::{Combo, Message, combo_box, dropdown};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list, `backstageViewPagesonar.Text`.
/// `// C#: GCSViews/InitialSetup.resx:864-866`
pub const TITLE: &str = "Range Finder";

/// `label1.Text`, the heading.
/// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.resx label1.Text`
pub const HEADING: &str = "RangeFinder";

/// `CMB_sonartype.Items`, which the combo holds until `Activate` binds the documented values.
/// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.resx CMB_sonartype.Items-Items3`
pub const DESIGNER_ITEMS: [&str; 4] = ["XL-EZ0 / XL-EZ4", "LV-EZ0", "XL-EZL0", "HRLV"];

/// The type whose choice sets the range limits.
/// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.cs:52`
pub const TERARANGER: &str = "TeraRangerOne-I2C";

/// `timer1.Interval`.
/// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.Designer.cs:102`
pub const TIMER_INTERVAL: Duration = Duration::from_millis(200);

/// `LBL_dist.Text` and `LBL_volt.Text` before the timer has run.
/// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.resx LBL_dist.Text, LBL_volt.Text`
const DESIGNER_READING: &str = "0.0";

/// `Program.handleException`'s box for an exception nothing caught.
/// `// C#: Program.cs:791-793`
#[must_use]
pub fn unhandled(param: &str) -> Message {
    Message {
        title: "Send Error",
        text: format!(
            "An error has occurred\n{}\n\nReport this Error???",
            timeout_text(param)
        ),
    }
}

/// The page object.
#[derive(Debug)]
pub struct RangeFinder {
    made_for: Option<Key>,
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `startup`.
    startup: bool,
    /// `CMB_sonartype`.
    kind: Combo,
    dropdown: bool,
    /// `LBL_dist.Text`.
    distance: String,
    /// `LBL_volt.Text`.
    voltage: String,
    /// When the timer last ran, while it runs.
    timer: Option<Instant>,
    /// How many times it has run.
    ticks: u32,
    messages: VecDeque<Message>,
    queue: SetQueue,
}

impl Default for RangeFinder {
    /// `InitializeComponent`: the combo disabled, holding the Designer's four items unselected.
    fn default() -> Self {
        let kind = Combo {
            options: DESIGNER_ITEMS
                .iter()
                .zip(0..)
                .map(|(text, key)| (key, (*text).to_owned()))
                .collect(),
            ..Combo::default()
        };
        Self {
            made_for: None,
            active: false,
            enabled: true,
            startup: true,
            kind,
            dropdown: false,
            distance: DESIGNER_READING.to_owned(),
            voltage: DESIGNER_READING.to_owned(),
            timer: None,
            ticks: 0,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

impl RangeFinder {
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

    /// The type combo.
    #[must_use]
    pub const fn kind(&self) -> &Combo {
        &self.kind
    }

    /// `LBL_dist.Text`.
    #[must_use]
    pub fn distance(&self) -> &str {
        &self.distance
    }

    /// `LBL_volt.Text`.
    #[must_use]
    pub fn voltage(&self) -> &str {
        &self.voltage
    }

    /// Whether the timer runs.
    #[must_use]
    pub const fn timer_running(&self) -> bool {
        self.timer.is_some()
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

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.cs:17-34`
    pub fn activate(
        &mut self,
        view: &TelemetryView,
        key: Key,
        connected: bool,
        lookup: Lookup,
        now: Instant,
    ) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(key),
                messages,
                queue,
                ..Self::default()
            };
        }
        self.active = true;
        self.dropdown = false;
        if !connected {
            self.enabled = false;
            return;
        }
        self.enabled = true;
        self.startup = true;
        self.kind.setup(
            options("RNGFND_TYPE", lookup),
            "RNGFND_TYPE",
            &view.parameters,
        );
        // `timer1.Start()`: its first tick is an interval away.
        self.timer = Some(now);
        self.startup = false;
    }

    /// `Deactivate`: the timer stops.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.cs:36-39`
    pub fn deactivate(&mut self) {
        self.active = false;
        self.dropdown = false;
        self.timer = None;
    }

    /// `timer1_Tick`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.cs:41-45`
    fn timer_tick(&mut self, view: &TelemetryView) {
        let (range, voltage) = view.state.as_deref().map_or((0.0, 0.0), |state| {
            (state.rangefinder.range, state.rangefinder.voltage)
        });
        // `cs.sonarrange` is `(float)toAltDisplayUnit(_sonarrange)`.
        // `// C#: ExtLibs/ArduPilot/CurrentState.cs:1856-1863`
        self.distance = float_text(range * crate::fly::MULTIPLIER_ALT);
        self.voltage = float_text(voltage);
        self.ticks = self.ticks.saturating_add(1);
    }

    /// Drops the list down, or back up.
    pub fn toggle_dropdown(&mut self) {
        self.dropdown = !self.dropdown && self.enabled && self.kind.enabled;
        if self.dropdown {
            self.kind.open_list();
        }
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, lines: i32) {
        if self.dropdown {
            self.kind.scroll_list(lines);
        }
    }

    /// A type chosen: the page's handler, then the combo's own write, in one go - the handler has
    /// no `try`, so a throw in it ends the combo's handler too.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWRangeFinder.cs:47-58; Controls/MavlinkComboBox.cs:133-200`
    pub fn choose(&mut self, key: i64) -> Vec<Job> {
        self.dropdown = false;
        if !self.enabled {
            return Vec::new();
        }
        let Some(write) = self.kind.choose(key) else {
            return Vec::new();
        };
        let mut sets = Vec::new();
        if !self.startup && self.kind.text() == TERARANGER {
            for (param, value) in [("RNGFND_MAX_CM", 100.0), ("RNGFND_MIN_CM ", 20.0)] {
                sets.push(Set {
                    on_throw: Some(unhandled(param)),
                    ..Set::plain(param, value)
                });
            }
        }
        sets.push(Set::control(write));
        vec![Job::new("type", sets)]
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// Once a frame: a page object whose screen has gone is let go, the timer, the writes.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
            self.timer = None;
        }
        if let Some(last) = self.timer
            && now.duration_since(last) >= TIMER_INTERVAL
        {
            self.timer_tick(view);
            self.timer = Some(now);
        }
        self.queue.advance(telemetry, &mut self.messages);
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &RangeFinder, view: &TelemetryView) {
    use crate::facts::record;
    record("config.rangefinder.active", page.is_active());
    record("config.rangefinder.enabled", page.enabled());
    record("config.rangefinder.heading", HEADING);
    record(
        "config.rangefinder.type",
        page.kind()
            .selected
            .map_or_else(|| "none".to_owned(), |value| value.to_string()),
    );
    record("config.rangefinder.type.text", page.kind().text());
    record("config.rangefinder.type.enabled", page.kind().enabled);
    record("config.rangefinder.type.options", page.kind().options.len());
    record("config.rangefinder.distance", page.distance());
    record("config.rangefinder.voltage", page.voltage());
    record("config.rangefinder.timer", page.timer_running());
    record("config.rangefinder.readouts", page.ticks);
    record(
        "config.rangefinder.write",
        page.queue.last().unwrap_or("none"),
    );
    record("config.rangefinder.writes.pending", page.queue.pending());
    record(
        "config.rangefinder.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    for name in [
        "RNGFND_TYPE",
        "RNGFND1_TYPE",
        "RNGFND_MAX_CM",
        "RNGFND_MIN_CM",
    ] {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// A reading in "Arial, 12pt", as `label2`, `label4`, `LBL_dist` and `LBL_volt` are.
fn reading(id: &'static str, x: f32, y: f32, text: String, enabled: bool) -> AnyElement {
    crate::probe::measured(id, div())
        .absolute()
        .left(px(x))
        .top(px(y))
        .text_base()
        .whitespace_nowrap()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        .child(text)
        .into_any_element()
}

/// The page, laid out as `ConfigHWRangeFinder.resx` lays it out.
pub fn page(rangefinder: &RangeFinder, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !rangefinder.is_active() {
        return div().into_any_element();
    }
    let enabled = rangefinder.enabled;
    let mut kind = rangefinder.kind.clone();
    kind.enabled &= enabled;
    let mut body = div()
        .relative()
        .w(px(650.0))
        .h(px(116.0))
        .child(heading(7.0, 5.0, HEADING, enabled))
        .child(rule(3.0, 23.0, 644.0))
        .child(picture("sonar", (11.0, 35.0, 75.0, 75.0)))
        .child(combo_box(
            "rangefinder-RNGFND_TYPE".to_owned(),
            &kind,
            (180.0, 62.0, 121.0, 21.0),
            |this| this.optional.rangefinder.toggle_dropdown(),
            cx,
        ))
        .child(reading(
            "rangefinder-distance-label",
            337.0,
            50.0,
            "Distance: ".to_owned(),
            enabled,
        ))
        .child(reading(
            "rangefinder-distance",
            421.0,
            50.0,
            rangefinder.distance.clone(),
            enabled,
        ))
        .child(reading(
            "rangefinder-voltage-label",
            337.0,
            78.0,
            "Voltage: ".to_owned(),
            enabled,
        ))
        .child(reading(
            "rangefinder-voltage",
            421.0,
            78.0,
            rangefinder.voltage.clone(),
            enabled,
        ));
    if rangefinder.dropdown {
        body = body.child(dropdown(
            "rangefinder-RNGFND_TYPE",
            &rangefinder.kind,
            (180.0, 83.0, 121.0),
            |this, key| {
                let jobs = this.optional.rangefinder.choose(key);
                this.optional.rangefinder.push(jobs);
            },
            |this, lines| this.optional.rangefinder.scroll_list(lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// The message box showing, over the whole window.
pub fn overlay(
    rangefinder: &RangeFinder,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let message = rangefinder.message()?;
    Some(message_box(
        "rangefinder-message",
        "rangefinder-message-ok",
        message,
        window,
        |this| this.optional.rangefinder.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::{Answering, drain};
    use crate::config::servo_output::ERROR_TITLE;
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

    /// A lookup that documents `RNGFND_TYPE` with the one value the C# checks for.
    fn with_teraranger(name: &str) -> Option<&'static mp_params::ParamMeta> {
        static META: std::sync::OnceLock<mp_params::ParamMeta> = std::sync::OnceLock::new();
        if name == "RNGFND_TYPE" {
            let base = bundled("RNGFND_TYPE")?;
            return Some(META.get_or_init(|| mp_params::ParamMeta {
                values: &[(0, "None"), (14, TERARANGER)],
                ..*base
            }));
        }
        bundled(name)
    }

    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigHWRangeFinder.resx",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("label1.Text"), Some(HEADING));
        assert_eq!(get("label2.Text"), Some("Distance: "));
        assert_eq!(get("label4.Text"), Some("Voltage: "));
        assert_eq!(get("LBL_dist.Text"), Some(DESIGNER_READING));
        assert_eq!(get("CMB_sonartype.Location"), Some("180, 62"));
        assert_eq!(get("LBL_dist.Location"), Some("421, 50"));
        assert_eq!(get("LBL_volt.Location"), Some("421, 78"));
        for (index, item) in DESIGNER_ITEMS.iter().enumerate() {
            let key = if index == 0 {
                "CMB_sonartype.Items".to_owned()
            } else {
                format!("CMB_sonartype.Items{index}")
            };
            assert_eq!(get(&key), Some(*item));
        }
    }

    /// SITL's copter numbers its rangefinders: `RNGFND_TYPE` is not there and the combo stays
    /// disabled, bound to the documented values.
    #[test]
    fn a_numbered_rangefinder_leaves_the_combo_disabled() {
        let mut page = RangeFinder::default();
        assert_eq!(page.kind().options.len(), 4, "the Designer's items");
        let view = view_with(&[("RNGFND1_TYPE", 0.0)]);
        page.activate(&view, key(), true, bundled, Instant::now());
        assert!(page.enabled() && page.timer_running());
        assert!(!page.kind().enabled);
        assert!(page.kind().options.len() > 4, "the documentation's list");
        assert!(page.choose(0).is_empty(), "a disabled combo writes nothing");
    }

    #[test]
    fn the_timer_shows_the_vehicles_range_and_voltage() {
        let mut page = RangeFinder::default();
        let mut view = view_with(&[("RNGFND_TYPE", 1.0)]);
        let mut state = mp_vehicle::VehicleState::default();
        state.rangefinder.range = 1.25;
        state.rangefinder.voltage = 3.3;
        view.state = Some(Arc::new(state));
        let start = Instant::now();
        page.activate(&view, key(), true, bundled, start);
        assert_eq!(page.distance(), "0.0", "not before the first tick");
        let telemetry = Telemetry::idle();
        page.tick(&telemetry, &view, true, start + TIMER_INTERVAL);
        assert_eq!(page.distance(), "1.25");
        assert_eq!(page.voltage(), "3.3");
        page.deactivate();
        assert!(!page.timer_running());
    }

    #[test]
    fn with_no_link_the_page_is_disabled_and_the_timer_does_not_start() {
        let mut page = RangeFinder::default();
        page.activate(&view_with(&[]), key(), false, bundled, Instant::now());
        assert!(!page.enabled() && !page.timer_running());
    }

    #[test]
    fn choosing_a_type_writes_it() {
        let mut page = RangeFinder::default();
        let view = view_with(&[("RNGFND_TYPE", 0.0)]);
        page.activate(&view, key(), true, bundled, Instant::now());
        assert!(page.kind().enabled);
        page.toggle_dropdown();
        let jobs = page.choose(1);
        let link = Answering::new(&[]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        assert!(drain(&mut queue, &link).is_empty());
        assert_eq!(link.taken(), [("RNGFND_TYPE".to_owned(), 1.0)]);
        assert_eq!(page.kind().text(), "Analog");
    }

    /// The TeraRanger sets the range limits first, the second under a name with a trailing
    /// space, which is refused; then the combo writes the type.
    #[test]
    fn the_teraranger_sets_its_limits_before_the_type() {
        let mut page = RangeFinder::default();
        let view = view_with(&[("RNGFND_TYPE", 0.0), ("RNGFND_MAX_CM", 700.0)]);
        page.activate(&view, key(), true, with_teraranger, Instant::now());
        let jobs = page.choose(14);
        let link = Answering::new(&[(
            "RNGFND_MIN_CM ",
            Progress::Finished(RequestOutcome::UnknownParameter),
        )]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        assert!(drain(&mut queue, &link).is_empty(), "a false says nothing");
        assert_eq!(
            link.taken(),
            [
                ("RNGFND_MAX_CM".to_owned(), 100.0),
                ("RNGFND_MIN_CM ".to_owned(), 20.0),
                ("RNGFND_TYPE".to_owned(), 14.0)
            ]
        );
        // A timeout in the handler escapes it: Mission Planner's own box, and no type written.
        let mut page = RangeFinder::default();
        page.activate(&view, key(), true, with_teraranger, Instant::now());
        let jobs = page.choose(14);
        let link = Answering::new(&[(
            "RNGFND_MAX_CM",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let messages = drain(&mut queue, &link);
        assert_eq!(messages, [unhandled("RNGFND_MAX_CM")]);
        assert_eq!(link.taken().len(), 1);
        assert_ne!(messages[0].title, ERROR_TITLE);
    }

    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-rangefinder.gui");
        let source = include_str!("rangefinder.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.rangefinder.") => {
                    assert!(
                        source.contains(&format!("\"{key}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("rangefinder-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts > 6, "{facts} facts");
    }
}
