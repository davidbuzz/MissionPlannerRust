//! Airspeed: `GCSViews/ConfigurationView/ConfigHWAirspeed.cs`, an Optional Hardware page of
//! Initial Setup (`GCSViews/InitialSetup.cs:291-294`), listed once every parameter is in.
//!
//! What it shows: the heading, the sensor's picture, "Enable" (`ARSPD_ENABLE`) and "Use Airspeed"
//! (`ARSPD_USE`) as `MavlinkCheckBox`es - each hidden when the vehicle lacks its parameter - the
//! pin (`ARSPD_PIN`) from the page's own list of fourteen, and the type (`ARSPD_TYPE`) from the
//! parameter's documented values. `Activate` sets them up each time the page is shown
//! (`ConfigHWAirspeed.cs:18-63`); each control writes its parameter when it changes. "Enable"
//! also has the page's handler, which runs first and writes `ARSPD_ENABLE` itself (`:65-84`).
//!
//! The layout is `ConfigHWAirspeed.resx`'s, every control at its `Location` in a 650 x 120 page.
//!
//! What is not ported, and why: `pictureBox4`'s image (`Resources.airspeed`), a resource this
//! application does not carry - the box is drawn with its name.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, Window, div, prelude::*, px};

use super::optional::{
    FEATURE_NOT_ENABLED, Job, Set, SetQueue, error, has, heading, label, message_box, picture,
    rule, set_failed, value_of,
};
use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::config::servo_output::{Check, Combo, Message, check_box, combo_box, dropdown};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::panel;

/// The page's title in Initial Setup's list, `backstageViewPageairspeed.Text`.
/// `// C#: GCSViews/InitialSetup.resx:189-191`
pub const TITLE: &str = "Airspeed";

/// `label2.Text`, the heading.
/// `// C#: GCSViews/ConfigurationView/ConfigHWAirspeed.resx label2.Text`
pub const HEADING: &str = "Airspeed";

/// The pin combo's list, which `Activate` builds.
/// `// C#: GCSViews/ConfigurationView/ConfigHWAirspeed.cs:41-59`
pub const PINS: [(i64, &str); 14] = [
    (0, "APM 2 analog pin 0"),
    (1, "APM 2 analog pin 1"),
    (2, "APM 2 analog pin 2"),
    (3, "APM 2 analog pin 3"),
    (4, "APM 2 analog pin 4"),
    (5, "APM 2 analog pin 5"),
    (6, "APM 2 analog pin 6"),
    (7, "APM 2 analog pin 7"),
    (8, "APM 2 analog pin 8"),
    (9, "APM 2 analog pin 9"),
    (64, "APM 1 AS Port"),
    (11, "PX4 Analog AS Port"),
    (15, "Pixhawk Analog AS Port"),
    (65, "PX4/Pixhawk EagleTree or MEAS I2C AS Sensor"),
];

/// The two combos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Which {
    /// `mavlinkCheckBoxAirspeed_pin`, `ARSPD_PIN`.
    Pin,
    /// `mavlinkComboBoxARSPD_TYPE`, `ARSPD_TYPE`.
    Type,
}

/// The page object.
#[derive(Debug)]
pub struct Airspeed {
    /// The screen the page object belongs to.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// The page's `Enabled`: false when `Activate` found no link.
    enabled: bool,
    /// `startup`.
    startup: bool,
    /// `CHK_enableairspeed`, and whether it is visible.
    enable: Check,
    enable_visible: bool,
    /// `CHK_airspeeduse`, and whether it is visible.
    use_airspeed: Check,
    use_visible: bool,
    /// `mavlinkCheckBoxAirspeed_pin`.
    pin: Combo,
    /// `mavlinkComboBoxARSPD_TYPE`.
    kind: Combo,
    /// The combo whose list is down.
    dropdown: Option<Which>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// The writes.
    queue: SetQueue,
}

impl Default for Airspeed {
    /// `InitializeComponent`: every control disabled (`.resx` `Enabled = False`) and visible.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            startup: false,
            enable: Check::default(),
            enable_visible: true,
            use_airspeed: Check::default(),
            use_visible: true,
            pin: Combo::default(),
            kind: Combo::default(),
            dropdown: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

impl Airspeed {
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

    /// "Enable".
    #[must_use]
    pub const fn enable(&self) -> &Check {
        &self.enable
    }

    /// "Use Airspeed".
    #[must_use]
    pub const fn use_airspeed(&self) -> &Check {
        &self.use_airspeed
    }

    /// A combo.
    #[must_use]
    pub const fn combo(&self, which: Which) -> &Combo {
        match which {
            Which::Pin => &self.pin,
            Which::Type => &self.kind,
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

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWAirspeed.cs:18-63`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        connected: bool,
        lookup: Lookup,
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
        self.dropdown = None;
        if !connected {
            self.enabled = false;
            return;
        }
        self.enabled = true;
        self.startup = true;
        // `// C#: :29-33`
        if !has(parameters, "ARSPD_USE") {
            self.use_visible = false;
        }
        if !has(parameters, "ARSPD_ENABLE") {
            self.enable_visible = false;
        }
        // `MavlinkCheckBox.setup` shows a box whose parameter it finds. `// C#: Controls/MavlinkCheckBox.cs:71-74`
        self.use_airspeed.setup(1.0, 0.0, "ARSPD_USE", parameters);
        self.use_visible |= has(parameters, "ARSPD_USE");
        self.enable.setup(1.0, 0.0, "ARSPD_ENABLE", parameters);
        self.enable_visible |= has(parameters, "ARSPD_ENABLE");
        self.kind
            .setup(options("ARSPD_TYPE", lookup), "ARSPD_TYPE", parameters);
        let pins = PINS
            .iter()
            .map(|(key, text)| (*key, (*text).to_owned()))
            .collect();
        self.pin.setup(pins, "ARSPD_PIN", parameters);
        self.startup = false;
    }

    /// The page hidden. `ConfigHWAirspeed` is `IActivate` only.
    pub fn hide(&mut self) {
        self.active = false;
        self.dropdown = None;
    }

    /// A click on "Enable": the page's handler, then the control's own write.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWAirspeed.cs:65-84; Controls/MavlinkCheckBox.cs:106-143`
    pub fn click_enable(&mut self, parameters: &[(String, f64)]) -> Vec<Job> {
        self.dropdown = None;
        if !self.enabled {
            return Vec::new();
        }
        let Some(write) = self.enable.click() else {
            return Vec::new();
        };
        let mut jobs = Vec::new();
        if !self.startup {
            if value_of(parameters, "ARSPD_ENABLE").is_none() {
                jobs.push(Job::show("enable", error(FEATURE_NOT_ENABLED)));
            } else {
                let checked = self.enable.state == crate::config::failsafe::CheckState::Checked;
                jobs.push(Job::new(
                    "enable",
                    [Set::caught(
                        "ARSPD_ENABLE",
                        if checked { 1.0 } else { 0.0 },
                        set_failed("ARSPD_ENABLE"),
                    )],
                ));
            }
        }
        jobs.push(Job::control(write));
        jobs
    }

    /// A click on "Use Airspeed": its own write.
    pub fn click_use(&mut self) -> Vec<Job> {
        self.dropdown = None;
        if !self.enabled {
            return Vec::new();
        }
        self.use_airspeed
            .click()
            .map(Job::control)
            .into_iter()
            .collect()
    }

    /// Drops a combo's list down, or back up.
    pub fn toggle_dropdown(&mut self, which: Which) {
        let enabled = self.enabled && self.combo(which).enabled;
        self.dropdown = if self.dropdown == Some(which) || !enabled {
            None
        } else {
            match which {
                Which::Pin => self.pin.open_list(),
                Which::Type => self.kind.open_list(),
            }
            Some(which)
        };
    }

    /// The wheel over the list.
    pub fn scroll_list(&mut self, which: Which, lines: i32) {
        if self.dropdown == Some(which) {
            match which {
                Which::Pin => self.pin.scroll_list(lines),
                Which::Type => self.kind.scroll_list(lines),
            }
        }
    }

    /// A row chosen: the control's own write.
    pub fn choose(&mut self, which: Which, key: i64) -> Vec<Job> {
        self.dropdown = None;
        if !self.enabled {
            return Vec::new();
        }
        let write = match which {
            Which::Pin => self.pin.choose(key),
            Which::Type => self.kind.choose(key),
        };
        write.map(Job::control).into_iter().collect()
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// Once a frame: a page object whose screen has gone is let go, and the writes move on.
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, on_setup: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
        }
        self.queue.advance(telemetry, &mut self.messages);
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &Airspeed, view: &TelemetryView) {
    use crate::facts::record;
    record("config.airspeed.active", page.is_active());
    record("config.airspeed.enabled", page.enabled());
    record("config.airspeed.heading", HEADING);
    record("config.airspeed.enable", page.enable().state.key());
    record("config.airspeed.enable.enabled", page.enable().enabled);
    record("config.airspeed.enable.visible", page.enable_visible);
    record("config.airspeed.use", page.use_airspeed().state.key());
    record("config.airspeed.use.enabled", page.use_airspeed().enabled);
    record("config.airspeed.use.visible", page.use_visible);
    for (which, key) in [(Which::Pin, "pin"), (Which::Type, "type")] {
        let combo = page.combo(which);
        record(
            format!("config.airspeed.{key}"),
            combo
                .selected
                .map_or_else(|| "none".to_owned(), |value| value.to_string()),
        );
        record(format!("config.airspeed.{key}.text"), combo.text());
        record(format!("config.airspeed.{key}.enabled"), combo.enabled);
        record(
            format!("config.airspeed.{key}.options"),
            combo.options.len(),
        );
    }
    record("config.airspeed.write", page.queue.last().unwrap_or("none"));
    record("config.airspeed.writes.pending", page.queue.pending());
    record(
        "config.airspeed.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    for name in ["ARSPD_ENABLE", "ARSPD_USE", "ARSPD_PIN", "ARSPD_TYPE"] {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The page, laid out as `ConfigHWAirspeed.resx` lays it out.
pub fn page(airspeed: &Airspeed, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !airspeed.is_active() {
        return div().into_any_element();
    }
    let enabled = airspeed.enabled;
    let shown = |check: &Check| {
        let mut check = check.clone();
        check.enabled &= enabled;
        check
    };
    let combo = |which: Which| {
        let mut combo = airspeed.combo(which).clone();
        combo.enabled &= enabled;
        combo
    };
    let mut body = div()
        .relative()
        .w(px(650.0))
        .h(px(120.0))
        .child(heading(7.0, 5.0, HEADING, enabled))
        .child(rule(3.0, 23.0, 644.0))
        .child(picture("airspeed", (11.0, 35.0, 75.0, 75.0)));
    if airspeed.enable_visible {
        body = body.child(check_box(
            "airspeed-ARSPD_ENABLE".to_owned(),
            &shown(&airspeed.enable),
            "Enable",
            (92.0, 34.0),
            |this| {
                let view = this.telemetry.view();
                let jobs = this.optional.airspeed.click_enable(&view.parameters);
                this.optional.airspeed.push(jobs);
            },
            cx,
        ));
    }
    if airspeed.use_visible {
        body = body.child(check_box(
            "airspeed-ARSPD_USE".to_owned(),
            &shown(&airspeed.use_airspeed),
            "Use Airspeed",
            (168.0, 35.0),
            |this| {
                let jobs = this.optional.airspeed.click_use();
                this.optional.airspeed.push(jobs);
            },
            cx,
        ));
    }
    body = body
        .child(label(93.0, 61.0, "Pin", enabled))
        .child(combo_box(
            "airspeed-ARSPD_PIN".to_owned(),
            &combo(Which::Pin),
            (165.0, 58.0, 161.0, 21.0),
            |this| this.optional.airspeed.toggle_dropdown(Which::Pin),
            cx,
        ))
        .child(label(92.0, 89.0, "Type", enabled))
        .child(combo_box(
            "airspeed-ARSPD_TYPE".to_owned(),
            &combo(Which::Type),
            (165.0, 86.0, 161.0, 21.0),
            |this| this.optional.airspeed.toggle_dropdown(Which::Type),
            cx,
        ));
    if let Some(which) = airspeed.dropdown {
        // `DropDownWidth = 200` for the pin. `// C#: ConfigHWAirspeed.Designer.cs:86`
        let (id, y, width) = match which {
            Which::Pin => ("airspeed-ARSPD_PIN", 58.0, 200.0),
            Which::Type => ("airspeed-ARSPD_TYPE", 86.0, 161.0),
        };
        body = body.child(dropdown(
            id,
            airspeed.combo(which),
            (165.0, y + 21.0, width),
            move |this, key| {
                let jobs = this.optional.airspeed.choose(which, key);
                this.optional.airspeed.push(jobs);
            },
            move |this, lines| this.optional.airspeed.scroll_list(which, lines),
            cx,
        ));
    }
    panel(TITLE, body).into_any_element()
}

/// The message box showing, over the whole window.
pub fn overlay(
    airspeed: &Airspeed,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let message = airspeed.message()?;
    Some(message_box(
        "airspeed-message",
        "airspeed-message-ok",
        message,
        window,
        |this| this.optional.airspeed.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::failsafe::CheckState;
    use crate::config::optional::tests::{Answering, drain};
    use crate::config::optional::{Event, SetQueue};

    fn bundled(name: &str) -> Option<&'static mp_params::ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn run(jobs: Vec<Job>, link: &Answering) -> (Vec<Message>, SetQueue<usize>) {
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let messages = drain(&mut queue, link);
        (messages, queue)
    }

    /// The `.resx`'s words and places, read from the tree when it is here.
    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigHWAirspeed.resx",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("label2.Text"), Some(HEADING));
        assert_eq!(get("CHK_enableairspeed.Text"), Some("Enable"));
        assert_eq!(get("CHK_enableairspeed.Location"), Some("92, 34"));
        assert_eq!(get("CHK_airspeeduse.Text"), Some("Use Airspeed"));
        assert_eq!(get("CHK_airspeeduse.Location"), Some("168, 35"));
        assert_eq!(get("lbl_airspeed_pin.Text"), Some("Pin"));
        assert_eq!(get("mavlinkCheckBoxAirspeed_pin.Location"), Some("165, 58"));
        assert_eq!(get("label1.Text"), Some("Type"));
        assert_eq!(get("mavlinkComboBoxARSPD_TYPE.Location"), Some("165, 86"));
        assert_eq!(get("pictureBox4.Location"), Some("11, 35"));
        assert_eq!(get("$this.Size"), Some("650, 120"));
    }

    /// SITL's copter: `ARSPD_ENABLE` 0 and nothing else of the sensor's - "Use Airspeed" hidden,
    /// the combos disabled.
    #[test]
    fn activate_on_the_sitl_copter() {
        let mut page = Airspeed::default();
        page.activate(&table(&[("ARSPD_ENABLE", 0.0)]), key(), true, bundled);
        assert!(page.is_active() && page.enabled());
        assert!(page.enable_visible && page.enable().enabled);
        assert_eq!(page.enable().state, CheckState::Unchecked);
        assert!(!page.use_visible, "no ARSPD_USE");
        assert!(!page.use_airspeed().enabled);
        assert!(!page.combo(Which::Pin).enabled);
        assert!(!page.combo(Which::Type).enabled);
        assert_eq!(page.combo(Which::Pin).options.len(), 14);
    }

    #[test]
    fn with_no_link_the_page_is_disabled_and_writes_nothing() {
        let mut page = Airspeed::default();
        page.activate(&table(&[("ARSPD_ENABLE", 0.0)]), key(), false, bundled);
        assert!(!page.enabled());
        assert!(page.click_enable(&[]).is_empty());
        assert!(page.choose(Which::Pin, 15).is_empty());
    }

    #[test]
    fn a_plane_with_every_parameter_binds_every_control() {
        let mut page = Airspeed::default();
        let parameters = table(&[
            ("ARSPD_ENABLE", 1.0),
            ("ARSPD_USE", 1.0),
            ("ARSPD_PIN", 15.0),
            ("ARSPD_TYPE", 1.0),
        ]);
        page.activate(&parameters, key(), true, bundled);
        assert_eq!(page.enable().state, CheckState::Checked);
        assert!(page.use_visible);
        assert_eq!(page.use_airspeed().state, CheckState::Checked);
        assert_eq!(page.combo(Which::Pin).text(), "Pixhawk Analog AS Port");
        assert!(page.combo(Which::Pin).enabled && page.combo(Which::Type).enabled);
    }

    /// "Enable" writes `ARSPD_ENABLE` twice, the page's handler first, then the control; the
    /// second finds the value already held in the C#, which the link here reports the same way.
    #[test]
    fn enable_writes_the_handlers_value_then_the_controls() {
        let mut page = Airspeed::default();
        let parameters = table(&[("ARSPD_ENABLE", 0.0)]);
        page.activate(&parameters, key(), true, bundled);
        let jobs = page.click_enable(&parameters);
        assert_eq!(page.enable().state, CheckState::Checked);
        let link = Answering::new(&[]);
        let (messages, queue) = run(jobs, &link);
        assert!(messages.is_empty());
        assert_eq!(
            link.taken(),
            [
                ("ARSPD_ENABLE".to_owned(), 1.0),
                ("ARSPD_ENABLE".to_owned(), 1.0)
            ]
        );
        assert_eq!(queue.last(), Some("ARSPD_ENABLE 1 accepted"));
        let jobs = page.click_enable(&parameters);
        let (_, _) = run(jobs, &link);
        assert_eq!(link.taken().last(), Some(&("ARSPD_ENABLE".to_owned(), 0.0)));
    }

    /// A timeout: the page's `catch` box, then the control's own.
    #[test]
    fn a_timeout_shows_both_boxes() {
        use crate::config::flight_modes::Progress;
        use mp_link::requests::RequestOutcome;
        let mut page = Airspeed::default();
        let parameters = table(&[("ARSPD_ENABLE", 0.0)]);
        page.activate(&parameters, key(), true, bundled);
        let jobs = page.click_enable(&parameters);
        let link =
            Answering::new(&[("ARSPD_ENABLE", Progress::Finished(RequestOutcome::TimedOut))]);
        let (messages, _) = run(jobs, &link);
        assert_eq!(
            messages,
            [
                error("Set ARSPD_ENABLE Failed"),
                error("Set ARSPD_ENABLE Failed")
            ]
        );
    }

    #[test]
    fn choosing_a_pin_writes_it() {
        let mut page = Airspeed::default();
        let parameters = table(&[("ARSPD_ENABLE", 1.0), ("ARSPD_PIN", 15.0)]);
        page.activate(&parameters, key(), true, bundled);
        page.toggle_dropdown(Which::Pin);
        let jobs = page.choose(Which::Pin, 65);
        let link = Answering::new(&[]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let mut messages = VecDeque::new();
        let events = queue.advance(&link, &mut messages);
        assert!(events.iter().any(|event| matches!(
            event,
            Event::Set { param, value, .. } if param == "ARSPD_PIN" && (*value - 65.0).abs() < 1e-9
        )));
        assert_eq!(
            page.combo(Which::Pin).text(),
            "PX4/Pixhawk EagleTree or MEAS I2C AS Sensor"
        );
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-airspeed.gui");
        let source = include_str!("airspeed.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.airspeed.") => {
                    let generic = ["pin", "type"].iter().fold(key.to_owned(), |key, which| {
                        key.replace(&format!("config.airspeed.{which}"), "config.airspeed.{key}")
                    });
                    assert!(
                        source.contains(&format!("\"{key}\""))
                            || source.contains(&format!("\"{generic}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("airspeed-") => {
                    let id = id.split(':').next().unwrap_or(id);
                    let base = id
                        .rsplit_once('-')
                        .filter(|(_, tail)| tail.chars().all(|c| c.is_ascii_digit()))
                        .map_or(id, |(base, _)| base);
                    assert!(source.contains(&format!("\"{base}")), "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 8 && clicks >= 2, "{facts} facts, {clicks} clicks");
    }
}
