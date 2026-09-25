//! Basic Tuning for a plane: `GCSViews/ConfigurationView/ConfigArduplane.cs`, the page CONFIG's
//! list adds as "Basic Tuning" - and opens first - when the vehicle is an ArduPlane
//! (`GCSViews/SoftwareConfig.cs:173-178`). A copter's Basic Tuning is another class,
//! `ConfigSimplePids` (`:160-165`), and a rover's `ConfigArdurover` (`:186-189`); neither is this.
//!
//! What it shows: twelve group boxes of `MavlinkNumericUpDown`s - the servo roll, pitch and yaw
//! loops, L1, the two navigation-pitch loops, TECS, the mixes, the throttle, the energy loop, the
//! navigation angles and the airspeeds - 44 boxes in all, each with its label, and three buttons
//! under them: Write Params, Refresh Params and Refresh Screen. The layout is
//! `ConfigArduplane.resx`'s, every control at its `Location` in a 621 x 479 page.
//!
//! `Activate` (`ConfigArduplane.cs:26-126`) disables the page unless the link is open and the
//! firmware is ArduPlane, then binds each box with `setup(Min, Max, Scale, Increment, names,
//! MAV.param)` - the first of `names` the vehicle has, else the first - clears the changes, and
//! gives each box the parameter's documented description as its tooltip. A box whose parameter the
//! vehicle lacks is disabled, as `MavlinkNumericUpDown.setup` leaves it; most of the older names
//! here (`ENRGY2THR_*`, `ALT2PTCH_*`, `ARSP2PTCH_*`) are gone from current firmware.
//!
//! Unlike the Setup pages' boxes, these do not write themselves: the page subscribes to each box's
//! `ValueUpdated`, which makes `MavlinkNumericUpDown` hand its value to the page instead of starting
//! its own 300 ms write (`Controls/MavlinkNumericUpDown.cs:147-152`). The page keeps it in
//! `changes` and paints the box green (`ConfigArduplane.cs:177-201`). Write Params - or Ctrl+S -
//! goes through `changes` one parameter at a time (`:203-261`): a value more than twice the
//! vehicle's asks "... has more than doubled the last input. Are you sure?", and No puts the
//! vehicle's value back in the box and stops; with no link it says "Your are not connected" and
//! stops; otherwise `setParam`, which here is the link's retrying set, and the box goes back to its
//! colour and out of `changes` - unless the set threw, which says "Set NAME Failed" and keeps it.
//! Refresh Params downloads the parameter list again and activates the page (`:268-288`); Refresh
//! Screen activates it (`:290-326`).
//!
//! Where this differs from the C#, and why, is said at each site:
//!
//! * `changes` is a `Hashtable`, whose keys come out in the order of its hash buckets; here they
//!   are written in the order the boxes were first changed;
//! * the C# calls `setParam` on the UI thread, which holds the page until each set returns; here
//!   each set goes through the retrying set in turn, and the page's controls do nothing until the
//!   last has returned;
//! * No to "Large Value" puts the vehicle's value back as `Activate` shows it; the C# writes the
//!   parameter's raw value into the box's text, which for the five boxes scaled by 100 is a
//!   hundred times what they show;
//! * Refresh Screen's `updateparam` asks the vehicle for no parameter at all: it matches controls
//!   whose type is exactly `NumericUpDown` or `ComboBox`, and every box here is a
//!   `MavlinkNumericUpDown`. So here, too, it only activates the page;
//! * Refresh Params's `getParamList` throws only when the link fails, which the link here reports
//!   by closing - the screen is then shown again without the page - so "Error: getting param list"
//!   has nothing to say here. The page is activated again once the table is whole with a parameter
//!   of the new download in it, and until then its controls do nothing, as behind the C#'s
//!   progress box;
//! * the green of a changed box is a ring around it, in this application's palette.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Instant;

use gpui::{
    AnyElement, AnyView, Context, FocusHandle, KeyDownEvent, Render, SharedString, Window, div,
    prelude::*, px, rgb,
};

use super::optional::{
    Event, Job, Outcome, Set, SetQueue, button, error, group, label, set_failed,
};
use crate::MissionPlanner;
use crate::config::failsafe::Lookup;
use crate::config::flight_modes::{Firmware, ParamWriter};
use crate::config::servo_output::{
    Designer, ERROR_TITLE, Message, NUMERIC_DEFAULTS, Number, NumberHandlers, OUT_OF_RANGE_TITLE,
    Question, Setup, modal, number_box, value_of,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in CONFIG's list, `Strings.BasicTuning`.
/// `// C#: GCSViews/SoftwareConfig.cs:177; ExtLibs/Strings/Strings.resx:241-243`
pub const TITLE: &str = "Basic Tuning";

/// The class, as the lists name it.
pub const CLASS: &str = "ConfigArduplane";

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.resx ($this.Size)`
pub const PAGE_SIZE: (f32, f32) = (621.0, 479.0);

/// Every box's `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.resx (*.Size of each MavlinkNumericUpDown)`
pub const BOX_SIZE: (f32, f32) = (78.0, 20.0);

/// What `BUT_writePIDS_Click` says with no link, caption `Strings.ERROR`.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:233-237`
pub const NOT_CONNECTED: &str = "Your are not connected";

/// The caption of the question a value more than doubled asks.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:213-214`
pub const LARGE_VALUE_TITLE: &str = "Large Value";

/// That question's text.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:213`
#[must_use]
pub fn large_value_text(param: &str) -> String {
    format!("{param} has more than doubled the last input. Are you sure?")
}

/// A group box: its Designer name, `Text`, and `Location` and `Size`.
pub type GroupSpec = (&'static str, &'static str, (f32, f32, f32, f32));

/// The twelve group boxes.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.resx (groupBox*.Location, .Size, .Text)`
pub const GROUPS: [GroupSpec; 12] = [
    ("groupBox8", "Servo Roll Pid", (12.0, 15.0, 195.0, 108.0)),
    ("groupBox9", "Servo Pitch Pid", (213.0, 15.0, 195.0, 108.0)),
    ("groupBox10", "Servo Yaw", (414.0, 15.0, 195.0, 108.0)),
    (
        "groupBox4",
        "L1 Control - Turn Control",
        (12.0, 129.0, 195.0, 56.0),
    ),
    (
        "groupBox12",
        "Nav Pitch AS Pid",
        (213.0, 123.0, 195.0, 108.0),
    ),
    (
        "groupBox13",
        "Nav Pitch Alt Pid",
        (414.0, 123.0, 195.0, 108.0),
    ),
    ("groupBox5", "TECS", (12.0, 191.0, 195.0, 148.0)),
    ("groupBox16", "Other Mix's", (213.0, 231.0, 195.0, 68.0)),
    ("groupBox3", "Throttle 0-100%", (413.0, 231.0, 195.0, 108.0)),
    ("groupBox14", "Energy/Alt Pid", (12.0, 339.0, 195.0, 108.0)),
    (
        "groupBox2",
        "Navigation Angles",
        (213.0, 339.0, 195.0, 108.0),
    ),
    ("groupBox1", "Airspeed m/s", (414.0, 339.0, 195.0, 108.0)),
];

/// One `MavlinkNumericUpDown`: its Designer name, the group it is in, its `Location` there, its
/// label, and what `Activate` binds it with.
#[derive(Debug, Clone, Copy)]
pub struct Tuned {
    /// The Designer's name, which the box keeps until `setup` renames it to its parameter.
    pub name: &'static str,
    /// The group box, an index into [`GROUPS`].
    pub group: usize,
    /// Its `Location` in the group.
    pub at: (f32, f32),
    /// Its label: the Designer's name, `Text` and `Location` in the group.
    pub label: (&'static str, &'static str, (f32, f32)),
    /// `setup`'s `Min`, `Max`, `Scale` and `Increment`.
    pub setup: Setup,
    /// `setup`'s names: the first the vehicle has is bound, else the first.
    pub params: &'static [&'static str],
}

/// A box, as the table below writes one.
const fn tuned(
    name: &'static str,
    group: usize,
    at: (f32, f32),
    label: (&'static str, &'static str, (f32, f32)),
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

/// Group indices, for the table.
const ROLL: usize = 0;
const PITCH: usize = 1;
const YAW: usize = 2;
const L1: usize = 3;
const AS_PID: usize = 4;
const ALT_PID: usize = 5;
const TECS: usize = 6;
const MIX: usize = 7;
const THROTTLE: usize = 8;
const ENERGY: usize = 9;
const ANGLES: usize = 10;
const AIRSPEED: usize = 11;

/// `setup(0, 0, 1, 0, ...)`, the most common arguments.
const PLAIN: (f32, f32, f32, f32) = (0.0, 0.0, 1.0, 0.0);
/// `setup(0, 0, 100, 0, ...)`: an integrator limit, shown divided by 100.
const IMAX: (f32, f32, f32, f32) = (0.0, 0.0, 100.0, 0.0);
/// `setup(0, 0, 1, 1, ...)`.
const WHOLE: (f32, f32, f32, f32) = (0.0, 0.0, 1.0, 1.0);

/// Every box, in the order `Activate` sets them up.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:45-99;
/// ConfigArduplane.resx (*.Location, *.Text, >>*.Parent)`
#[rustfmt::skip]
pub const BOXES: [Tuned; 44] = [
    tuned("THR_SLEWRATE", THROTTLE, (111.0, 82.0), ("label5", "SlewRate", (6.0, 86.0)), PLAIN, &["THR_SLEWRATE"]),
    tuned("THR_MAX", THROTTLE, (111.0, 59.0), ("label6", "Max", (6.0, 63.0)), PLAIN, &["THR_MAX"]),
    tuned("THR_MIN", THROTTLE, (111.0, 36.0), ("label7", "Min", (6.0, 40.0)), PLAIN, &["THR_MIN"]),
    tuned("TRIM_THROTTLE", THROTTLE, (111.0, 13.0), ("label8", "Cruise", (6.0, 17.0)), PLAIN, &["TRIM_THROTTLE"]),
    tuned("ARSPD_RATIO", AIRSPEED, (111.0, 82.0), ("label1", "Ratio", (6.0, 87.0)), (0.0, 2.5, 1.0, 0.005), &["ARSPD_RATIO"]),
    tuned("ARSPD_FBW_MAX", AIRSPEED, (111.0, 59.0), ("label2", "Max", (6.0, 59.0)), WHOLE, &["AIRSPEED_MAX", "ARSPD_FBW_MAX"]),
    tuned("ARSPD_FBW_MIN", AIRSPEED, (111.0, 36.0), ("label3", "Min", (6.0, 40.0)), WHOLE, &["AIRSPEED_MIN", "ARSPD_FBW_MIN"]),
    tuned("TRIM_ARSPD_CM", AIRSPEED, (111.0, 13.0), ("label4", "Cruise", (6.0, 17.0)), (0.0, 50.0, 1.0, 1.0), &["AIRSPEED_CRUISE"]),
    tuned("LIM_PITCH_MIN", ANGLES, (111.0, 59.0), ("label39", "Pitch Min", (6.0, 63.0)), WHOLE, &["PTCH_LIM_MIN_DEG"]),
    tuned("LIM_PITCH_MAX", ANGLES, (111.0, 36.0), ("label38", "Pitch Max", (6.0, 40.0)), WHOLE, &["PTCH_LIM_MAX_DEG"]),
    tuned("LIM_ROLL_CD", ANGLES, (111.0, 13.0), ("label37", "Bank Max", (6.0, 17.0)), WHOLE, &["ROLL_LIMIT_DEG"]),
    tuned("KFF_PTCH2THR", MIX, (111.0, 13.0), ("label83", "P to T", (6.0, 17.0)), PLAIN, &["KFF_THR2PTCH", "KFF_PTCH2THR"]),
    tuned("KFF_RDDRMIX", MIX, (111.0, 39.0), ("label78", "Rudder Mix", (6.0, 43.0)), PLAIN, &["KFF_RDDRMIX"]),
    tuned("ENRGY2THR_IMAX", ENERGY, (111.0, 82.0), ("label73", "INT_MAX", (6.0, 86.0)), IMAX, &["ENRGY2THR_IMAX"]),
    tuned("ENRGY2THR_D", ENERGY, (111.0, 59.0), ("label74", "D", (6.0, 63.0)), PLAIN, &["ENRGY2THR_D"]),
    tuned("ENRGY2THR_I", ENERGY, (111.0, 36.0), ("label75", "I", (6.0, 40.0)), PLAIN, &["ENRGY2THR_I"]),
    tuned("ENRGY2THR_P", ENERGY, (111.0, 13.0), ("label76", "P", (6.0, 17.0)), PLAIN, &["ENRGY2THR_P"]),
    tuned("ALT2PTCH_IMAX", ALT_PID, (111.0, 82.0), ("label69", "INT_MAX", (6.0, 86.0)), IMAX, &["ALT2PTCH_IMAX"]),
    tuned("ALT2PTCH_D", ALT_PID, (111.0, 59.0), ("label70", "D", (6.0, 63.0)), PLAIN, &["ALT2PTCH_D"]),
    tuned("ALT2PTCH_I", ALT_PID, (111.0, 36.0), ("label71", "I", (6.0, 40.0)), PLAIN, &["ALT2PTCH_I"]),
    tuned("ALT2PTCH_P", ALT_PID, (111.0, 13.0), ("label72", "P", (6.0, 17.0)), PLAIN, &["ALT2PTCH_P"]),
    tuned("ARSP2PTCH_IMAX", AS_PID, (111.0, 82.0), ("label65", "INT_MAX", (6.0, 86.0)), IMAX, &["ARSP2PTCH_IMAX"]),
    tuned("ARSP2PTCH_D", AS_PID, (111.0, 59.0), ("label66", "D", (6.0, 63.0)), PLAIN, &["ARSP2PTCH_D"]),
    tuned("ARSP2PTCH_I", AS_PID, (111.0, 36.0), ("label67", "I", (6.0, 40.0)), PLAIN, &["ARSP2PTCH_I"]),
    tuned("ARSP2PTCH_P", AS_PID, (111.0, 13.0), ("label68", "P", (6.0, 17.0)), PLAIN, &["ARSP2PTCH_P"]),
    tuned("YAW2SRV_IMAX", YAW, (111.0, 82.0), ("label57", "Intergrator Max", (6.0, 86.0)), IMAX, &["YAW2SRV_IMAX"]),
    tuned("YAW2SRV_DAMP", YAW, (111.0, 59.0), ("label58", "Dampening", (6.0, 63.0)), PLAIN, &["YAW2SRV_DAMP"]),
    tuned("YAW2SRV_INT", YAW, (111.0, 36.0), ("label59", "Integral", (6.0, 40.0)), PLAIN, &["YAW2SRV_INT"]),
    tuned("YAW2SRV_RLL", YAW, (111.0, 13.0), ("label60", "Yaw 2 roll", (6.0, 17.0)), PLAIN, &["YAW2SRV_RLL"]),
    tuned("PTCH2SRV_IMAX", PITCH, (111.0, 82.0), ("label53", "INT_MAX", (6.0, 86.0)), IMAX, &["PTCH2SRV_IMAX", "PTCH_RATE_IMAX"]),
    tuned("PTCH2SRV_D", PITCH, (111.0, 59.0), ("label54", "D", (6.0, 63.0)), PLAIN, &["PTCH2SRV_D", "PTCH_RATE_D"]),
    tuned("PTCH2SRV_I", PITCH, (111.0, 36.0), ("label55", "I", (6.0, 40.0)), PLAIN, &["PTCH2SRV_I", "PTCH_RATE_I"]),
    tuned("PTCH2SRV_P", PITCH, (111.0, 13.0), ("label56", "P", (6.0, 17.0)), PLAIN, &["PTCH2SRV_P", "PTCH_RATE_P"]),
    tuned("RLL2SRV_IMAX", ROLL, (111.0, 82.0), ("label49", "INT_MAX", (6.0, 86.0)), IMAX, &["RLL2SRV_IMAX", "RLL_RATE_IMAX"]),
    tuned("RLL2SRV_D", ROLL, (111.0, 59.0), ("label50", "D", (6.0, 63.0)), PLAIN, &["RLL2SRV_D", "RLL_RATE_D"]),
    tuned("RLL2SRV_I", ROLL, (111.0, 36.0), ("label51", "I", (6.0, 40.0)), PLAIN, &["RLL2SRV_I", "RLL_RATE_I"]),
    tuned("RLL2SRV_P", ROLL, (111.0, 13.0), ("label52", "P", (6.0, 17.0)), PLAIN, &["RLL2SRV_P", "RLL_RATE_P"]),
    tuned("NAVL1_DAMPING", L1, (111.0, 36.0), ("label9", "Damping", (6.0, 40.0)), PLAIN, &["NAVL1_DAMPING"]),
    tuned("NAVL1_PERIOD", L1, (111.0, 13.0), ("label10", "Period", (6.0, 17.0)), PLAIN, &["NAVL1_PERIOD"]),
    tuned("TECS_SINK_MAX", TECS, (111.0, 59.0), ("label15", "Sink Max (m/s)", (6.0, 63.0)), PLAIN, &["TECS_SINK_MAX"]),
    tuned("TECS_TIME_CONST", TECS, (111.0, 108.0), ("label14", "Time Const", (6.0, 112.0)), PLAIN, &["TECS_TIME_CONST"]),
    tuned("TECS_PTCH_DAMP", TECS, (111.0, 82.0), ("label13", "Pitch Dampening", (6.0, 86.0)), PLAIN, &["TECS_PTCH_DAMP"]),
    tuned("TECS_SINK_MIN", TECS, (111.0, 36.0), ("label11", "Sink Min (m/s)", (6.0, 40.0)), PLAIN, &["TECS_SINK_MIN"]),
    tuned("TECS_CLMB_MAX", TECS, (111.0, 13.0), ("label12", "Climb Max (m/s)", (6.0, 17.0)), PLAIN, &["TECS_CLMB_MAX"]),
];

/// A button: its Designer name, `Text`, `Location` and `Size`, and its `Click` handler.
pub type ButtonSpec = (
    &'static str,
    &'static str,
    (f32, f32, f32, f32),
    &'static str,
);

/// Write Params, Refresh Params and Refresh Screen.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.resx (BUT_*); ConfigArduplane.Designer.cs (Click)`
pub const BUTTONS: [ButtonSpec; 3] = [
    (
        "BUT_writePIDS",
        "Write Params",
        (204.0, 451.0, 103.0, 19.0),
        "BUT_writePIDS_Click",
    ),
    (
        "BUT_rerequestparams",
        "Refresh Params",
        (313.0, 451.0, 103.0, 19.0),
        "BUT_rerequestparams_Click",
    ),
    (
        "BUT_refreshpart",
        "Refresh Screen",
        (422.0, 451.0, 103.0, 19.0),
        "BUT_refreshpart_Click",
    ),
];

/// A question the page is asking, modal as the C#'s message boxes are.
#[derive(Debug, Clone, PartialEq)]
pub enum Ask {
    /// A box's "Out of range" question: which box, and the question.
    Range(usize, Question),
    /// Write Params's "Large Value" question, for this parameter.
    Large(String),
}

/// Write Params going through `changes`: the keys as the button found them, and whether a set is
/// on its way.
#[derive(Debug, Default)]
struct WriteLoop {
    keys: VecDeque<String>,
}

/// The page object.
#[derive(Debug)]
pub struct BasicTuning<H = mp_link::RequestId> {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// The boxes, in [`BOXES`]' order.
    numbers: Vec<Number>,
    /// Each box's `BackColor` being green: changed and not yet written.
    green: Vec<bool>,
    /// Each box's tooltip.
    tips: Vec<String>,
    /// `changes`: the parameter and the value each box handed over, in the order first changed.
    changes: Vec<(String, f64)>,
    /// The box the keyboard is typing into.
    editing: Option<usize>,
    /// The question showing.
    question: Option<Ask>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// Write Params, while it runs.
    writing: Option<WriteLoop>,
    /// Its sets.
    queue: SetQueue<H>,
    /// Refresh Params, while its download runs: the table it was asked over.
    refreshing: Option<Arc<[(String, f64)]>>,
}

impl<H> Default for BasicTuning<H> {
    /// `InitializeComponent`: every box at `NumericUpDown`'s defaults and disabled, as
    /// `MavlinkNumericUpDown`'s constructor leaves it.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:31-41`
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            numbers: BOXES
                .iter()
                .map(|_| Number::new(NUMERIC_DEFAULTS))
                .collect(),
            green: vec![false; BOXES.len()],
            tips: vec![String::new(); BOXES.len()],
            changes: Vec::new(),
            editing: None,
            question: None,
            messages: VecDeque::new(),
            writing: None,
            queue: SetQueue::default(),
            refreshing: None,
        }
    }
}

/// The name `setup` binds: the first of `names` the vehicle has, else the first.
/// `// C#: Controls/MavlinkNumericUpDown.cs:54-64`
#[must_use]
pub fn bound_name(names: &[&'static str], parameters: &[(String, f64)]) -> &'static str {
    names
        .iter()
        .find(|name| value_of(parameters, name).is_some())
        .or_else(|| names.first())
        .copied()
        .unwrap_or("")
}

impl<H: Copy> BasicTuning<H> {
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

    /// The boxes, in [`BOXES`]' order.
    #[must_use]
    pub fn numbers(&self) -> &[Number] {
        &self.numbers
    }

    /// Whether a box is green.
    #[must_use]
    pub fn is_green(&self, index: usize) -> bool {
        self.green.get(index).copied().unwrap_or(false)
    }

    /// A box's tooltip.
    #[must_use]
    pub fn tip(&self, index: usize) -> &str {
        self.tips.get(index).map_or("", String::as_str)
    }

    /// `changes`, in the order first changed.
    #[must_use]
    pub fn changes(&self) -> &[(String, f64)] {
        &self.changes
    }

    /// The box being typed into.
    #[must_use]
    pub const fn editing(&self) -> Option<usize> {
        self.editing
    }

    /// The question showing.
    #[must_use]
    pub const fn question(&self) -> Option<&Ask> {
        self.question.as_ref()
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Whether Write Params is going through `changes`.
    #[must_use]
    pub const fn writing(&self) -> bool {
        self.writing.is_some()
    }

    /// How many sets are queued or on their way.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.pending()
    }

    /// How the last set ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.queue.last()
    }

    /// Whether Refresh Params is waiting for its download.
    #[must_use]
    pub const fn refreshing(&self) -> bool {
        self.refreshing.is_some()
    }

    /// Whether the page's controls take input: enabled, and not held by Write Params's sets, by
    /// Refresh Params's download or by a question - the C#'s handlers hold the UI thread until
    /// they return, and `getParamList` shows its modal progress box.
    #[must_use]
    pub const fn live(&self) -> bool {
        self.enabled
            && self.writing.is_none()
            && self.refreshing.is_none()
            && self.question.is_none()
    }

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:26-126`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        connected: bool,
        firmware: Firmware,
        lookup: Lookup,
    ) {
        if self.made_for != Some(key) {
            self.dispose();
            self.made_for = Some(key);
        }
        self.active = true;
        self.bind(parameters, connected, firmware, lookup);
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
        if !connected || firmware != Firmware::ArduPlane {
            self.enabled = false;
            return;
        }
        self.enabled = true;
        // `// C#: :45-99`
        for (number, spec) in self.numbers.iter_mut().zip(BOXES) {
            number.setup(
                spec.setup,
                bound_name(spec.params, parameters),
                parameters,
                lookup,
            );
        }
        // `// C#: :101`
        self.changes.clear();
        // Each box's tooltip, the parameter's description; "" for one undocumented, which shows
        // none. `// C#: :103-123`
        for (tip, number) in self.tips.iter_mut().zip(&self.numbers) {
            *tip = lookup(&number.param)
                .map_or("", |meta| meta.description)
                .to_owned();
        }
    }

    /// The page hidden - it has no `Deactivate` - which takes the focus from a box being typed
    /// into, so its text is read.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.leave(now);
    }

    /// The page object let go with its screen. Sets already on their way finish, and what they
    /// say is shown; the rest of a Write Params is not made.
    fn dispose(&mut self) {
        *self = Self {
            messages: std::mem::take(&mut self.messages),
            queue: std::mem::take(&mut self.queue),
            ..Self::default()
        };
    }

    /// A box's `ValueUpdated`: `EEPROM_View_float_TextChanged` keeps the value it hands over in
    /// `changes` under the box's name - its parameter - and paints the box green.
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:177-201, 328-331;
    /// Controls/MavlinkNumericUpDown.cs:147-152`
    fn updated(&mut self, index: usize) {
        // The box's own write is not started: with `ValueUpdated` subscribed it hands the value
        // over and returns. `flush` is that value, `(float)base.Value * (float)_scale`.
        let Some(write) = self.numbers.get_mut(index).and_then(Number::flush) else {
            return;
        };
        match self
            .changes
            .iter_mut()
            .find(|(name, _)| *name == write.param)
        {
            Some((_, value)) => *value = write.value,
            None => self.changes.push((write.param, write.value)),
        }
        if let Some(green) = self.green.get_mut(index) {
            *green = true;
        }
    }

    /// A box's question, or its value handed over.
    fn after(&mut self, index: usize, question: Option<Question>) {
        match question {
            Some(question) => self.question = Some(Ask::Range(index, question)),
            None => self.updated(index),
        }
    }

    /// A box clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) || !self.live() {
            return;
        }
        self.leave(now);
        if self.numbers.get(index).is_some_and(|number| number.enabled) {
            self.editing = Some(index);
        }
    }

    /// The box being typed into loses the focus, which reads its text.
    pub fn leave(&mut self, now: Instant) {
        let Some(index) = self.editing.take() else {
            return;
        };
        let question = self
            .numbers
            .get_mut(index)
            .and_then(|number| number.commit(now));
        self.after(index, question);
    }

    /// A key for the box being typed into. Ctrl+S is `ProcessCmdKey`'s: Write Params, with what
    /// the box holds not yet read, as the key reaches the page before the box.
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:128-137`
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
            self.write_params();
            return true;
        }
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        let (handled, question) = number.key(event, now);
        self.after(index, question);
        handled
    }

    /// A box's up or down arrow.
    pub fn step(&mut self, index: usize, up: bool, now: Instant) {
        if !self.live() {
            return;
        }
        self.begin(index, now);
        if self.question.is_some() {
            return;
        }
        let question = self
            .numbers
            .get_mut(index)
            .and_then(|number| number.step(up, now));
        self.after(index, question);
    }

    /// Types into a box, for a test.
    #[cfg(test)]
    pub fn type_into(&mut self, index: usize, text: &str, now: Instant) {
        self.begin(index, now);
        if let Some(number) = self.numbers.get_mut(index) {
            number.type_text(text);
        }
    }

    /// Write Params clicked: the focus leaving the box being typed into reads it, then
    /// `BUT_writePIDS_Click`. A question the box raises is modal, and takes the click.
    pub fn press_write(&mut self, now: Instant) {
        if !self.live() {
            return;
        }
        self.leave(now);
        if self.question.is_none() {
            self.write_params();
        }
    }

    /// `BUT_writePIDS_Click`: the keys of a copy of `changes`, each in turn - see [`Self::advance`].
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:203-207`
    fn write_params(&mut self) {
        // The C#'s `Hashtable` gives its keys in its buckets' order; here, the order first
        // changed.
        self.writing = Some(WriteLoop {
            keys: self.changes.iter().map(|(name, _)| name.clone()).collect(),
        });
    }

    /// Goes through Write Params's keys as far as the link's answers allow: for each, the value
    /// more than doubled asks first; with no link the loop says so and stops; otherwise `setParam`
    /// through the retrying set, and when it returns the box goes back to its colour and out of
    /// `changes`. A set that throws says "Set NAME Failed" and keeps both.
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:207-260`
    pub fn advance<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        parameters: &[(String, f64)],
        connected: bool,
    ) {
        loop {
            for event in self.queue.advance(writer, &mut self.messages) {
                if let Event::Set { param, outcome, .. } = event
                    && outcome != Outcome::Threw
                {
                    // `changes.Remove(value)` and the colour back. `// C#: :239-254`
                    self.changes.retain(|(name, _)| *name != param);
                    if let Some(index) = self.index_of(&param)
                        && let Some(green) = self.green.get_mut(index)
                    {
                        *green = false;
                    }
                }
            }
            if self.queue.pending() > 0 || self.question.is_some() {
                return;
            }
            let Some(writing) = self.writing.as_mut() else {
                return;
            };
            let Some(key) = writing.keys.pop_front() else {
                self.writing = None;
                return;
            };
            // `(float)changes[value]` and `(float)MAV.param[value]`: either missing is a null
            // cast, which the `catch` reports as the set failing. `// C#: :211, 256-259`
            let (Some(value), Some(held)) = (self.change(&key), value_of(parameters, &key)) else {
                self.messages.push_back(error(set_failed(&key)));
                continue;
            };
            #[allow(clippy::cast_possible_truncation)] // the C# compares floats
            let doubled = value as f32 > held as f32 * 2.0;
            if doubled {
                self.question = Some(Ask::Large(key));
                return;
            }
            self.send(key, value, connected);
        }
    }

    /// The value `changes` holds for a parameter.
    fn change(&self, param: &str) -> Option<f64> {
        self.changes
            .iter()
            .find(|(name, _)| name == param)
            .map(|(_, value)| *value)
    }

    /// The first box bound to a parameter: `Controls.Find(value, true)[0]`.
    fn index_of(&self, param: &str) -> Option<usize> {
        self.numbers.iter().position(|number| number.param == param)
    }

    /// With no link, "Your are not connected" and `return`; otherwise the set, whose `catch` says
    /// "Set NAME Failed".
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:233-239, 256-259`
    fn send(&mut self, key: String, value: f64, connected: bool) {
        if !connected {
            self.messages.push_back(Message {
                title: ERROR_TITLE,
                text: NOT_CONNECTED.to_owned(),
            });
            self.writing = None;
            return;
        }
        let failed = set_failed(&key);
        self.queue
            .push([Job::new("write", [Set::caught(key, value, failed)])]);
    }

    /// A question answered. "Out of range": Yes takes what was typed, and either way the box
    /// hands its value over. "Large Value": Yes goes on to the set; No puts the vehicle's value
    /// back in the box - which the box hands over, as setting its text raises `ValueChanged` -
    /// gives it its colour back, and stops.
    /// `// C#: Controls/MavlinkNumericUpDown.cs:136-152; GCSViews/ConfigurationView/ConfigArduplane.cs:211-231`
    pub fn answer(
        &mut self,
        yes: bool,
        parameters: &[(String, f64)],
        connected: bool,
        lookup: Lookup,
        now: Instant,
    ) {
        match self.question.take() {
            Some(Ask::Range(index, question)) => {
                if let Some(number) = self.numbers.get_mut(index) {
                    number.answer(&question, yes, now);
                }
                self.updated(index);
            }
            Some(Ask::Large(key)) => {
                if yes {
                    if let Some(value) = self.change(&key) {
                        self.send(key, value, connected);
                    }
                    return;
                }
                if let Some(index) = self.index_of(&key) {
                    self.restore(index, parameters, lookup);
                }
                self.writing = None;
            }
            None => {}
        }
    }

    /// `textControls[0].Text = MAV.param[value].Value.ToString()` and the colour back. The text
    /// set is parsed into the box's value, which raises `ValueChanged` and so hands the value to
    /// `changes` again. The value is put back as `Activate` shows it: the C# writes the raw value
    /// into the text, which for a box scaled by 100 is a hundred times what it shows - a value
    /// the box's range clamps, and whose out-of-range question asks about a number the pilot
    /// never typed.
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:216-226`
    fn restore(&mut self, index: usize, parameters: &[(String, f64)], lookup: Lookup) {
        let Some((number, spec)) = self.numbers.get_mut(index).zip(BOXES.get(index)) else {
            return;
        };
        let before = number.written();
        let param = number.param.clone();
        number.setup(spec.setup, &param, parameters, lookup);
        #[allow(clippy::float_cmp)] // `Value`'s setter compares decimals exactly
        if number.written() != before {
            let value = number.written();
            match self.changes.iter_mut().find(|(name, _)| *name == param) {
                Some((_, held)) => *held = value,
                None => self.changes.push((param, value)),
            }
        }
        if let Some(green) = self.green.get_mut(index) {
            *green = false;
        }
    }

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Refresh Screen: `updateparam`, which reads nothing (see the module's notes), then
    /// `Activate`. Nothing with no link.
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:290-326`
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

    /// Refresh Params: `getParamList`, the button disabled until the list is back, then
    /// `Activate`. Nothing with no link.
    /// `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:268-288`
    pub fn refresh_params(&mut self, telemetry: &Telemetry, view: &TelemetryView, now: Instant) {
        if !self.live() {
            return;
        }
        self.leave(now);
        if !view.connected || view.vehicle.is_none() || self.question.is_some() {
            return;
        }
        self.refreshing = Some(Arc::clone(&view.parameters));
        telemetry.download_parameters();
    }

    /// Once a frame: a page object whose screen has gone is let go, a box the focus has left is
    /// read, Refresh Params's download seen back, and Write Params moved on.
    #[allow(clippy::too_many_arguments)]
    pub fn tick<W: ParamWriter<Handle = H>>(
        &mut self,
        writer: &W,
        view: &TelemetryView,
        on_config: bool,
        focused: bool,
        (connected, firmware): (bool, Firmware),
        lookup: Lookup,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_config || self.made_for != Some(Key::of(view)))
        {
            self.dispose();
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        if let Some(before) = &self.refreshing {
            let whole = !view.parameters.is_empty()
                && view.parameters.len() >= usize::from(view.parameters_expected);
            if !connected {
                self.refreshing = None;
            } else if whole && !Arc::ptr_eq(before, &view.parameters) {
                self.refreshing = None;
                self.bind(&view.parameters, connected, firmware, lookup);
            }
        }
        self.advance(writer, &view.parameters, connected);
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on: whether CONFIG's list holds the page and whether it shows, each
/// box - its text, the parameter it is bound to, whether it is enabled and green - `changes`, the
/// question or message showing, and how the last set went.
pub fn record_facts<H: Copy>(tuning: &BasicTuning<H>, listed: bool) {
    use crate::facts::record;
    record("config.tuning.listed", listed);
    record("config.tuning.active", tuning.is_active());
    record("config.tuning.enabled", tuning.enabled());
    for (index, (number, spec)) in tuning.numbers().iter().zip(BOXES).enumerate() {
        let key = format!("config.tuning.{}", spec.name);
        record(format!("{key}.param"), &number.param);
        record(format!("{key}.enabled"), number.enabled);
        record(format!("{key}.changed"), tuning.is_green(index));
        record(key, number.shown());
    }
    let changes: Vec<String> = tuning
        .changes()
        .iter()
        .map(|(name, value)| format!("{name}={value}"))
        .collect();
    record(
        "config.tuning.changes",
        if changes.is_empty() {
            "none".to_owned()
        } else {
            changes.join(",")
        },
    );
    record(
        "config.tuning.question",
        match tuning.question() {
            Some(Ask::Range(_, question)) => question.text(),
            Some(Ask::Large(param)) => large_value_text(param),
            None => "none".to_owned(),
        },
    );
    record(
        "config.tuning.message",
        tuning
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    record("config.tuning.write", tuning.last_write().unwrap_or("none"));
    record("config.tuning.writing", tuning.writing());
    record("config.tuning.writes.pending", tuning.pending());
    record("config.tuning.refreshing", tuning.refreshing());
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// A box's tooltip: a `ToolTip`'s text in a small panel. Also the FailSafe page's.
pub(crate) struct Tip(pub(crate) SharedString);

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

/// A box as a disabled page shows it: its text, inert.
fn inert(number: &Number) -> Number {
    let mut copy = Number::new(Designer {
        minimum: number.minimum,
        maximum: number.maximum,
        value: number.shown().parse().unwrap_or(0.0),
        decimals: number.decimals,
    });
    copy.enabled = false;
    copy
}

/// The page, laid out as `ConfigArduplane.resx` lays it out.
/// `// C#: GCSViews/ConfigurationView/ConfigArduplane.Designer.cs:29-1078; ConfigArduplane.resx`
pub fn page(
    tuning: &BasicTuning,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let enabled = tuning.enabled();
    let live = tuning.live();
    let mut body = div().relative().w(px(PAGE_SIZE.0)).h(px(PAGE_SIZE.1));
    for (group_index, &(_, caption, place)) in GROUPS.iter().enumerate() {
        let mut boxes = group(place, caption, enabled);
        for (index, (number, spec)) in tuning.numbers().iter().zip(BOXES).enumerate() {
            if spec.group != group_index {
                continue;
            }
            let (_, text, (lx, ly)) = spec.label;
            boxes = boxes.child(label(lx, ly, text, enabled));
            let (x, y) = spec.at;
            if tuning.is_green(index) {
                // `BackColor = Color.Green`, as a ring in this application's palette.
                // `// C#: GCSViews/ConfigurationView/ConfigArduplane.cs:195`
                boxes = boxes.child(
                    div()
                        .absolute()
                        .left(px(x - 2.0))
                        .top(px(y - 2.0))
                        .w(px(BOX_SIZE.0 + 4.0))
                        .h(px(BOX_SIZE.1 + 4.0))
                        .rounded_sm()
                        .border_2()
                        .border_color(rgb(theme::OK)),
                );
            }
            let shown;
            let number = if live {
                number
            } else {
                shown = inert(number);
                &shown
            };
            let id = format!("tuning-{}", spec.name);
            let tip = SharedString::from(tuning.tip(index).to_owned());
            let wrapper = div()
                .id(SharedString::from(format!("{id}-tip")))
                .absolute()
                .left(px(x))
                .top(px(y))
                .w(px(BOX_SIZE.0))
                .h(px(BOX_SIZE.1));
            let wrapper = if tip.is_empty() || !enabled {
                wrapper
            } else {
                wrapper.tooltip(move |_window, cx| -> AnyView {
                    let tip = tip.clone();
                    cx.new(|_| Tip(tip)).into()
                })
            };
            boxes = boxes.child(wrapper.child(number_box(
                id,
                number,
                tuning.editing() == Some(index),
                handle,
                (0.0, 0.0, BOX_SIZE.0, BOX_SIZE.1),
                NumberHandlers {
                    begin: move |this: &mut MissionPlanner| {
                        this.basic_tuning.begin(index, Instant::now());
                    },
                    key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                        this.basic_tuning.key(event, Instant::now())
                    },
                    step: move |this: &mut MissionPlanner, up: bool| {
                        this.basic_tuning.step(index, up, Instant::now());
                    },
                },
                window,
                cx,
            )));
        }
        body = body.child(boxes);
    }
    let [write, params, screen] = BUTTONS;
    body = body
        .child(button(
            "tuning-BUT_writePIDS",
            write.1,
            write.2,
            live,
            |this, _window, _cx| this.basic_tuning.press_write(Instant::now()),
            cx,
        ))
        .child(button(
            "tuning-BUT_rerequestparams",
            params.1,
            params.2,
            live,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                this.basic_tuning
                    .refresh_params(&this.telemetry, &view, Instant::now());
            },
            cx,
        ))
        .child(button(
            "tuning-BUT_refreshpart",
            screen.1,
            screen.2,
            live,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                let vehicle = crate::setup::Vehicle::of(&view, this.telemetry.firmware_banner());
                this.basic_tuning.refresh_screen(
                    &view.parameters,
                    vehicle.connected,
                    vehicle.firmware,
                    crate::metadata::lookup,
                    Instant::now(),
                );
            },
            cx,
        ));
    panel(TITLE, body).into_any_element()
}

/// The question or message box showing, drawn over the whole window.
pub fn overlay(
    tuning: &BasicTuning,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(ask) = tuning.question() {
        let (title, text) = match ask {
            Ask::Range(_, question) => (OUT_OF_RANGE_TITLE, question.text()),
            Ask::Large(param) => (LARGE_VALUE_TITLE, large_value_text(param)),
        };
        let answer = |yes: bool| {
            move |this: &mut MissionPlanner,
                  _event: &(),
                  _window: &mut Window,
                  cx: &mut Context<MissionPlanner>| {
                let view = this.telemetry.view();
                let vehicle = crate::setup::Vehicle::of(&view, this.telemetry.firmware_banner());
                this.basic_tuning.answer(
                    yes,
                    &view.parameters,
                    vehicle.connected,
                    crate::metadata::lookup,
                    Instant::now(),
                );
                cx.notify();
            }
        };
        let buttons = vec![
            action(
                "tuning-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(answer(true)),
            ),
            action(
                "tuning-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(answer(false)),
            ),
        ];
        return Some(modal(
            "tuning-question",
            title,
            &text,
            false,
            buttons,
            window,
        ));
    }
    let message = tuning.message()?;
    Some(super::optional::message_box(
        "tuning-message",
        "tuning-message-ok",
        message,
        window,
        |this| this.basic_tuning.dismiss_message(),
        cx,
    ))
}

impl MissionPlanner {
    /// `Activate`, when CONFIG's list shows the page.
    pub(crate) fn basic_tuning_activate(&mut self) {
        let view = self.telemetry.view();
        let vehicle = crate::setup::Vehicle::of(&view, self.telemetry.firmware_banner());
        let (connected, firmware) = (vehicle.connected, vehicle.firmware);
        self.basic_tuning.activate(
            &view.parameters,
            Key::of(&view),
            connected,
            firmware,
            crate::metadata::lookup,
        );
    }

    /// Once a frame: the page object, its box's focus, its refresh and its writes.
    pub(crate) fn basic_tuning_tick(&mut self, view: &TelemetryView, window: &Window) {
        let focused = self.basic_tuning_focus.is_focused(window);
        let vehicle = crate::setup::Vehicle::of(view, self.telemetry.firmware_banner());
        let state = (vehicle.connected, vehicle.firmware);
        let on_config = self.screen == crate::Screen::Config;
        self.basic_tuning.tick(
            &self.telemetry,
            view,
            on_config,
            focused,
            state,
            crate::metadata::lookup,
            Instant::now(),
        );
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;
    use std::time::Duration;

    use gpui::{Keystroke, Modifiers};
    use mp_link::ProtocolTimeouts;
    use mp_link::requests::RequestOutcome;
    use mp_mavlink_dialects::all::{Heartbeat, MavMessage};
    use mp_params::{ParamMeta, UserLevel};

    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::Answering;
    use crate::config_coverage::source::{arguments, csharp, resx};
    use crate::setup::{List, Vehicle as Listed};
    use crate::telemetry::scripted::{Vehicle, param, until};

    /// A parameter's documentation, as the plane's `apm.pdef.xml` gives it.
    const fn meta(
        name: &'static str,
        description: &'static str,
        range: Option<(f64, f64)>,
        increment: Option<f64>,
    ) -> ParamMeta {
        ParamMeta {
            name,
            display_name: name,
            description,
            units: "",
            range,
            range_text: "",
            increment,
            values: &[],
            bitmask: &[],
            user_level: UserLevel::Standard,
            reboot_required: false,
        }
    }

    /// Part of ArduPlane 4.5's documentation: the bundled table is the copter's.
    static PLANE: [ParamMeta; 9] = [
        meta(
            "THR_MAX",
            "Maximum throttle percentage used in all modes except manual, provided \
             THR_PASS_STAGE is not set.",
            Some((0.0, 100.0)),
            Some(1.0),
        ),
        meta(
            "THR_MIN",
            "Minimum throttle percentage used in all modes except manual.",
            Some((-100.0, 100.0)),
            Some(1.0),
        ),
        meta(
            "TRIM_THROTTLE",
            "Target percentage of throttle to apply for flight in automatic throttle modes.",
            Some((0.0, 100.0)),
            Some(1.0),
        ),
        meta(
            "PTCH_LIM_MIN_DEG",
            "Minimum pitch down angle.",
            Some((-90.0, 0.0)),
            Some(1.0),
        ),
        meta(
            "ROLL_LIMIT_DEG",
            "Maximum bank angle commanded in modes with stabilized limits.",
            Some((0.0, 90.0)),
            Some(1.0),
        ),
        meta(
            "YAW2SRV_IMAX",
            "Maximum integrator value for the yaw damper.",
            Some((0.0, 4500.0)),
            Some(1.0),
        ),
        meta(
            "PTCH_RATE_IMAX",
            "Pitch axis rate controller integrator maximum.",
            Some((0.0, 1.0)),
            Some(0.01),
        ),
        meta(
            "NAVL1_PERIOD",
            "Period in seconds of L1 tracking loop.",
            Some((1.0, 60.0)),
            Some(1.0),
        ),
        meta(
            "TECS_CLMB_MAX",
            "Maximum demanded climb rate.",
            Some((0.1, 20.0)),
            Some(0.1),
        ),
    ];

    fn plane_meta(name: &str) -> Option<&'static ParamMeta> {
        PLANE.iter().find(|meta| meta.name == name)
    }

    /// An ArduPlane 4.5's values for the page's names: no `ENRGY2THR_*`, `ALT2PTCH_*` or
    /// `ARSP2PTCH_*`, and the rate loops under their new names.
    const PLANE_PARAMS: [(&str, f64); 32] = [
        ("THR_MAX", 100.0),
        ("THR_MIN", 0.0),
        ("TRIM_THROTTLE", 45.0),
        ("THR_SLEWRATE", 100.0),
        ("ARSPD_RATIO", 1.9936),
        ("AIRSPEED_MAX", 22.0),
        ("AIRSPEED_MIN", 9.0),
        ("AIRSPEED_CRUISE", 15.0),
        ("PTCH_LIM_MIN_DEG", -25.0),
        ("PTCH_LIM_MAX_DEG", 20.0),
        ("ROLL_LIMIT_DEG", 45.0),
        ("KFF_RDDRMIX", 0.5),
        ("KFF_THR2PTCH", 0.0),
        ("YAW2SRV_IMAX", 1500.0),
        ("YAW2SRV_DAMP", 0.0),
        ("YAW2SRV_INT", 0.0),
        ("YAW2SRV_RLL", 1.0),
        ("PTCH_RATE_P", 0.04),
        ("PTCH_RATE_I", 0.15),
        ("PTCH_RATE_D", 0.0),
        ("PTCH_RATE_IMAX", 0.666),
        ("RLL_RATE_P", 0.08),
        ("RLL_RATE_I", 0.15),
        ("RLL_RATE_D", 0.0),
        ("RLL_RATE_IMAX", 0.666),
        ("NAVL1_PERIOD", 17.0),
        ("NAVL1_DAMPING", 0.75),
        ("TECS_CLMB_MAX", 5.0),
        ("TECS_SINK_MIN", 2.0),
        ("TECS_SINK_MAX", 5.0),
        ("TECS_TIME_CONST", 5.0),
        ("TECS_PTCH_DAMP", 0.0),
    ];

    fn plane() -> Vec<(String, f64)> {
        PLANE_PARAMS
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn index(name: &str) -> usize {
        BOXES
            .iter()
            .position(|spec| spec.name == name)
            .unwrap_or_else(|| panic!("{name} is not a box"))
    }

    /// The page shown for a plane with every parameter in, on a link that answers as told.
    fn shown() -> BasicTuning<usize> {
        let mut page = BasicTuning::<usize>::default();
        page.activate(&plane(), key(), true, Firmware::ArduPlane, plane_meta);
        page
    }

    fn shown_text(page: &BasicTuning<usize>, name: &str) -> String {
        page.numbers()[index(name)].shown().to_owned()
    }

    /// Runs Write Params until it has nothing left to do or a question stops it.
    fn run(page: &mut BasicTuning<usize>, link: &Answering, connected: bool) {
        let parameters = plane();
        for _ in 0..100 {
            page.advance(link, &parameters, connected);
            if (!page.writing() && page.pending() == 0) || page.question().is_some() {
                break;
            }
        }
    }

    fn press(key: &str, control: bool) -> KeyDownEvent {
        KeyDownEvent {
            keystroke: Keystroke {
                modifiers: Modifiers {
                    control,
                    ..Modifiers::default()
                },
                key: key.to_owned(),
                key_char: (!control).then(|| key.to_owned()),
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    /// `x, y` as the `.resx` writes a point or a size.
    fn pair((x, y): (f32, f32)) -> String {
        format!("{x}, {y}")
    }

    /// The `>>` of a `.resx` key naming a control's parent, as the XML escapes it.
    const PARENT: &str = "&gt;&gt;";

    /// Every control the Designer makes is drawn here: the twelve group boxes, the 44 boxes and
    /// their 44 labels, the three buttons - and `toolTip1`, a component, is the tooltips.
    #[test]
    fn every_designer_control_is_drawn() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigArduplane.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let made: BTreeSet<&str> = designer
            .lines()
            .filter_map(|line| line.trim().strip_prefix("this."))
            .filter(|line| line.contains(" = new ") && !line.starts_with("components"))
            .filter_map(|line| line.split_once(" = new "))
            .map(|(name, _)| name)
            .collect();
        let ours: BTreeSet<&str> = GROUPS
            .iter()
            .map(|(name, ..)| *name)
            .chain(BOXES.iter().map(|spec| spec.name))
            .chain(BOXES.iter().map(|spec| spec.label.0))
            .chain(BUTTONS.iter().map(|(name, ..)| *name))
            .chain(std::iter::once("toolTip1"))
            .collect();
        assert_eq!(made, ours);
        assert_eq!(ours.len(), 12 + 44 + 44 + 3 + 1);
    }

    /// Each box's `ValueUpdated` is `numeric_ValueUpdated`, each button's `Click` its handler:
    /// the 47 wirings, all handled.
    #[test]
    fn every_wiring_is_handled() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigArduplane.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        for spec in BOXES {
            let line = format!(
                "this.{}.ValueUpdated += new System.EventHandler(this.numeric_ValueUpdated);",
                spec.name
            );
            assert!(designer.contains(&line), "{line}");
        }
        for (name, _, _, handler) in BUTTONS {
            let line = format!("this.{name}.Click += new System.EventHandler(this.{handler});");
            assert!(designer.contains(&line), "{line}");
        }
        assert_eq!(
            designer.matches(" += new ").count(),
            BOXES.len() + BUTTONS.len()
        );
        assert_eq!(BOXES.len() + BUTTONS.len(), 47);
    }

    /// Every place, size, text and parent is the `.resx`'s.
    #[test]
    fn every_place_and_text_is_the_resx() {
        let Some(text) = csharp("GCSViews/ConfigurationView/ConfigArduplane.resx") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = resx(&text);
        let get = |key: String| values.get(&key).cloned().unwrap_or_default();
        assert_eq!(get("$this.Size".to_owned()), pair(PAGE_SIZE));
        for (name, caption, (x, y, w, h)) in GROUPS {
            assert_eq!(get(format!("{name}.Location")), pair((x, y)), "{name}");
            assert_eq!(get(format!("{name}.Size")), pair((w, h)), "{name}");
            assert_eq!(get(format!("{name}.Text")), caption, "{name}");
            assert_eq!(get(format!("{PARENT}{name}.Parent")), "$this", "{name}");
        }
        for spec in BOXES {
            let (group, ..) = GROUPS[spec.group];
            let name = spec.name;
            assert_eq!(get(format!("{name}.Location")), pair(spec.at), "{name}");
            assert_eq!(get(format!("{name}.Size")), pair(BOX_SIZE), "{name}");
            assert_eq!(get(format!("{PARENT}{name}.Parent")), group, "{name}");
            let (label, caption, at) = spec.label;
            assert_eq!(get(format!("{label}.Location")), pair(at), "{label}");
            assert_eq!(get(format!("{label}.Text")), caption, "{label}");
            assert_eq!(get(format!("{PARENT}{label}.Parent")), group, "{label}");
        }
        for (name, caption, (x, y, w, h), _) in BUTTONS {
            assert_eq!(get(format!("{name}.Location")), pair((x, y)), "{name}");
            assert_eq!(get(format!("{name}.Size")), pair((w, h)), "{name}");
            assert_eq!(get(format!("{name}.Text")), caption, "{name}");
        }
    }

    /// `Activate`'s `setup` calls, in order, with their arguments and names.
    #[test]
    fn every_box_is_set_up_as_activate_sets_it_up() {
        let Some(source) = csharp("GCSViews/ConfigurationView/ConfigArduplane.cs") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let body = source
            .split("public void Activate()")
            .nth(1)
            .and_then(|rest| rest.split("changes.Clear();").next())
            .expect("Activate");
        let mut calls = Vec::new();
        for (offset, _) in body.match_indices(".setup(") {
            let start = body[..offset]
                .rfind(|c: char| c.is_whitespace())
                .map_or(0, |at| at + 1);
            let name = &body[start..offset];
            let args = arguments(&body[offset + ".setup".len()..]);
            let number = |at: usize| -> f32 {
                args[at]
                    .trim_end_matches('f')
                    .parse()
                    .unwrap_or_else(|_| panic!("{name}: {}", args[at]))
            };
            // `new string[] { "A", "B" }` spans arguments: `arguments` splits at its comma.
            let names: Vec<String> = args[4..args.len() - 1]
                .join(",")
                .split('"')
                .skip(1)
                .step_by(2)
                .map(str::to_owned)
                .collect();
            calls.push((
                name.to_owned(),
                [number(0), number(1), number(2), number(3)],
                names,
            ));
        }
        let ours: Vec<(String, [f32; 4], Vec<String>)> = BOXES
            .iter()
            .map(|spec| {
                let Setup {
                    minimum,
                    maximum,
                    scale,
                    increment,
                } = spec.setup;
                (
                    spec.name.to_owned(),
                    [minimum, maximum, scale, increment],
                    spec.params.iter().map(|name| (*name).to_owned()).collect(),
                )
            })
            .collect();
        assert_eq!(calls, ours);
    }

    /// The list adds the page for a plane only, and opens on it: a copter's Basic Tuning is
    /// `ConfigSimplePids`. This is what `tests/gui/config-basic-tuning.gui` sees of the page
    /// against SITL's copter.
    #[test]
    fn the_config_list_adds_the_page_for_a_plane_only() {
        let parameters = plane();
        let vehicle = |firmware| Listed {
            connected: true,
            got_all_params: true,
            firmware,
            mav_type: 1,
            version: (4, 5),
            capabilities: 0,
            parameters: &parameters,
        };
        let classes = |firmware| -> (Vec<&'static str>, Option<&'static str>) {
            let built = crate::setup::build(List::Config, &vehicle(firmware));
            let entries = crate::setup::entries(List::Config);
            (
                built
                    .items
                    .iter()
                    .map(|(at, _)| entries[*at].class)
                    .collect(),
                built.start.map(|at| entries[at].class),
            )
        };
        let (pages, start) = classes(Firmware::ArduPlane);
        assert!(pages.contains(&CLASS));
        assert_eq!(start, Some(CLASS));
        let (pages, start) = classes(Firmware::ArduCopter2);
        assert!(!pages.contains(&CLASS));
        assert_eq!(start, Some("ConfigSimplePids"));
        for firmware in [Firmware::ArduRover, Firmware::ArduTracker, Firmware::Ateryx] {
            assert!(!classes(firmware).0.contains(&CLASS), "{firmware:?}");
        }
    }

    /// `Activate` on a copter, or with no link, disables the page and binds nothing.
    #[test]
    fn a_copter_or_no_link_disables_the_page() {
        for (connected, firmware) in [(true, Firmware::ArduCopter2), (false, Firmware::ArduPlane)] {
            let mut page = BasicTuning::<usize>::default();
            page.activate(&plane(), key(), connected, firmware, plane_meta);
            assert!(page.is_active());
            assert!(!page.enabled());
            assert!(!page.live());
            assert!(page.numbers().iter().all(|number| !number.enabled));
            assert_eq!(shown_text(&page, "THR_MAX"), "0", "the Designer's value");
            page.step(index("THR_MAX"), true, Instant::now());
            assert!(page.changes().is_empty());
        }
    }

    /// A plane: each box bound to the first name the vehicle has, shown divided by its scale, to
    /// the places its increment and value have; a box whose names the vehicle lacks disabled; the
    /// tooltips the descriptions.
    #[test]
    fn a_plane_binds_each_box_to_the_name_it_has() {
        let page = shown();
        assert!(page.enabled());
        let bound = |name: &str| page.numbers()[index(name)].param.clone();
        assert_eq!(bound("ARSPD_FBW_MAX"), "AIRSPEED_MAX");
        assert_eq!(bound("ARSPD_FBW_MIN"), "AIRSPEED_MIN");
        assert_eq!(bound("TRIM_ARSPD_CM"), "AIRSPEED_CRUISE");
        assert_eq!(bound("LIM_ROLL_CD"), "ROLL_LIMIT_DEG");
        assert_eq!(bound("KFF_PTCH2THR"), "KFF_THR2PTCH");
        assert_eq!(bound("PTCH2SRV_P"), "PTCH_RATE_P");
        assert_eq!(bound("RLL2SRV_IMAX"), "RLL_RATE_IMAX");
        // Neither name: the first, disabled.
        assert_eq!(bound("ENRGY2THR_P"), "ENRGY2THR_P");
        for name in ["ENRGY2THR_IMAX", "ALT2PTCH_P", "ARSP2PTCH_D"] {
            assert!(!page.numbers()[index(name)].enabled, "{name}");
        }
        let enabled = page
            .numbers()
            .iter()
            .filter(|number| number.enabled)
            .count();
        assert_eq!(enabled, 44 - 12);

        assert_eq!(shown_text(&page, "THR_MAX"), "100");
        assert_eq!(shown_text(&page, "TRIM_THROTTLE"), "45");
        assert_eq!(shown_text(&page, "LIM_PITCH_MIN"), "-25");
        // `setup(0, 2.5, 1, 0.005)`: three places, and the value's four.
        assert_eq!(shown_text(&page, "ARSPD_RATIO"), "1.9936");
        // Scaled by 100: 1500 shows 15, 0.666 shows 0.00666.
        assert_eq!(shown_text(&page, "YAW2SRV_IMAX"), "15");
        assert_eq!(shown_text(&page, "PTCH2SRV_IMAX"), "0.00666");
        assert_eq!(shown_text(&page, "NAVL1_DAMPING"), "0.75");

        assert_eq!(
            page.tip(index("THR_MIN")),
            "Minimum throttle percentage used in all modes except manual."
        );
        assert_eq!(
            page.tip(index("THR_SLEWRATE")),
            "",
            "undocumented: no tooltip"
        );
        assert!(page.changes().is_empty());
    }

    /// A changed box hands its value to `changes`, scaled, and turns green; nothing is written
    /// until Write Params.
    #[test]
    fn a_change_is_held_green_until_write_params() {
        let mut page = shown();
        let now = Instant::now();
        page.step(index("THR_MAX"), false, now);
        page.step(index("YAW2SRV_IMAX"), true, now);
        assert_eq!(
            page.changes(),
            [
                ("THR_MAX".to_owned(), 99.0),
                ("YAW2SRV_IMAX".to_owned(), 1600.0)
            ]
        );
        assert!(page.is_green(index("THR_MAX")));
        assert!(!page.is_green(index("THR_MIN")));
        // Changed again: the same key, the new value, in its place.
        page.step(index("THR_MAX"), false, now);
        assert_eq!(page.changes()[0], ("THR_MAX".to_owned(), 98.0));
        // The box's own 300 ms write never starts.
        assert_eq!(page.pending(), 0);
        let link = Answering::new(&[]);
        page.advance(&link, &plane(), true);
        assert!(link.taken().is_empty(), "nothing without Write Params");
    }

    /// Write Params: each change in the order first made, through the link; each answered set
    /// takes the box out of `changes` and gives it its colour back.
    #[test]
    fn write_params_sets_each_change_in_turn() {
        let mut page = shown();
        let now = Instant::now();
        page.step(index("TRIM_THROTTLE"), true, now);
        page.step(index("THR_MAX"), false, now);
        page.press_write(now);
        assert!(page.writing());
        assert!(!page.live(), "held while it writes");
        let link = Answering::new(&[]);
        run(&mut page, &link, true);
        assert_eq!(
            link.taken(),
            [
                ("TRIM_THROTTLE".to_owned(), 46.0),
                ("THR_MAX".to_owned(), 99.0)
            ]
        );
        assert!(page.changes().is_empty());
        assert!(!page.is_green(index("TRIM_THROTTLE")));
        assert!(!page.is_green(index("THR_MAX")));
        assert!(page.message().is_none());
        assert_eq!(page.last_write(), Some("THR_MAX 99 accepted"));
        assert!(page.live());
    }

    /// A set that throws says "Set NAME Failed" and keeps the change and the green; a false -
    /// a name the vehicle has not listed - says nothing and lets it go; the loop goes on.
    #[test]
    fn a_throw_keeps_the_change_and_a_false_lets_it_go() {
        let mut page = shown();
        let now = Instant::now();
        page.step(index("THR_MAX"), false, now);
        page.step(index("TRIM_THROTTLE"), true, now);
        page.step(index("NAVL1_PERIOD"), true, now);
        page.press_write(now);
        let link = Answering::new(&[
            ("THR_MAX", Progress::Finished(RequestOutcome::TimedOut)),
            (
                "TRIM_THROTTLE",
                Progress::Finished(RequestOutcome::UnknownParameter),
            ),
        ]);
        run(&mut page, &link, true);
        assert_eq!(link.taken().len(), 3, "each set made");
        assert_eq!(page.changes(), [("THR_MAX".to_owned(), 99.0)]);
        assert!(page.is_green(index("THR_MAX")));
        assert!(!page.is_green(index("TRIM_THROTTLE")));
        assert!(!page.is_green(index("NAVL1_PERIOD")));
        let message = page.message().expect("the catch's box");
        assert_eq!(message.title, ERROR_TITLE);
        assert_eq!(message.text, "Set THR_MAX Failed");
        page.dismiss_message();
        assert!(page.message().is_none(), "the false said nothing");
    }

    /// With no link: "Your are not connected", and the loop stops with the changes kept.
    #[test]
    fn no_link_says_so_and_stops() {
        let mut page = shown();
        let now = Instant::now();
        page.step(index("THR_MAX"), false, now);
        page.step(index("THR_MIN"), true, now);
        page.press_write(now);
        let link = Answering::new(&[]);
        run(&mut page, &link, false);
        assert!(link.taken().is_empty());
        assert!(!page.writing());
        assert_eq!(page.changes().len(), 2);
        assert_eq!(
            page.message()
                .map(|message| (message.title, message.text.as_str())),
            Some((ERROR_TITLE, NOT_CONNECTED))
        );
    }

    /// A value more than twice the vehicle's asks first. Yes sets it; No puts the vehicle's
    /// value back in the box - handed to `changes` as the text set raises `ValueChanged` - gives
    /// the box its colour back and stops, leaving the keys after it unwritten.
    #[test]
    fn a_value_more_than_doubled_asks_first() {
        let now = Instant::now();
        let link = Answering::new(&[]);
        let mut page = shown();
        page.type_into(index("TRIM_THROTTLE"), "95", now);
        page.press_write(now);
        run(&mut page, &link, true);
        assert_eq!(
            page.question(),
            Some(&Ask::Large("TRIM_THROTTLE".to_owned()))
        );
        assert_eq!(
            large_value_text("TRIM_THROTTLE"),
            "TRIM_THROTTLE has more than doubled the last input. Are you sure?"
        );
        page.answer(true, &plane(), true, plane_meta, now);
        run(&mut page, &link, true);
        assert_eq!(link.taken(), [("TRIM_THROTTLE".to_owned(), 95.0)]);
        assert!(page.changes().is_empty());

        let link = Answering::new(&[]);
        let mut page = shown();
        page.type_into(index("TRIM_THROTTLE"), "95", now);
        page.leave(now);
        page.step(index("THR_MAX"), false, now);
        page.press_write(now);
        run(&mut page, &link, true);
        assert!(page.question().is_some());
        page.answer(false, &plane(), true, plane_meta, now);
        run(&mut page, &link, true);
        assert!(link.taken().is_empty(), "THR_MAX, after it, is not reached");
        assert!(!page.writing());
        assert_eq!(shown_text(&page, "TRIM_THROTTLE"), "45");
        assert!(!page.is_green(index("TRIM_THROTTLE")));
        assert!(page.is_green(index("THR_MAX")));
        assert_eq!(
            page.changes(),
            [
                ("TRIM_THROTTLE".to_owned(), 45.0),
                ("THR_MAX".to_owned(), 99.0)
            ]
        );
    }

    /// The C# compares `changes > param * 2`, so a negative value asks whenever it is raised,
    /// and a zero whenever it is made positive.
    #[test]
    fn a_negative_or_zero_value_raised_asks_as_the_csharp_compares() {
        let mut page = shown();
        let now = Instant::now();
        page.step(index("THR_MIN"), true, now);
        page.press_write(now);
        run(&mut page, &Answering::new(&[]), true);
        assert_eq!(page.question(), Some(&Ask::Large("THR_MIN".to_owned())));

        let mut page = shown();
        let now = Instant::now();
        page.step(index("LIM_PITCH_MIN"), true, now);
        assert_eq!(page.changes(), [("PTCH_LIM_MIN_DEG".to_owned(), -24.0)]);
        page.press_write(now);
        run(&mut page, &Answering::new(&[]), true);
        assert_eq!(
            page.question(),
            Some(&Ask::Large("PTCH_LIM_MIN_DEG".to_owned()))
        );
    }

    /// A typed value above the maximum asks "Out of range"; either answer hands the box's value
    /// over - the maximum, or with Yes what was typed.
    #[test]
    fn a_typed_value_above_the_maximum_asks_out_of_range() {
        let now = Instant::now();
        let mut page = shown();
        page.type_into(index("LIM_ROLL_CD"), "95", now);
        page.leave(now);
        let Some(Ask::Range(at, question)) = page.question().cloned() else {
            panic!("the question");
        };
        assert_eq!(at, index("LIM_ROLL_CD"));
        assert_eq!(
            question.text(),
            "ROLL_LIMIT_DEG Value out of range\nDo you want to accept the new value?"
        );
        assert!(page.changes().is_empty(), "not before it is answered");
        page.answer(false, &plane(), true, plane_meta, now);
        assert_eq!(page.changes(), [("ROLL_LIMIT_DEG".to_owned(), 90.0)]);

        let mut page = shown();
        page.type_into(index("LIM_ROLL_CD"), "95", now);
        page.leave(now);
        page.answer(true, &plane(), true, plane_meta, now);
        assert_eq!(page.changes(), [("ROLL_LIMIT_DEG".to_owned(), 95.0)]);
        assert!(page.is_green(index("LIM_ROLL_CD")));
    }

    /// Ctrl+S in a box is `ProcessCmdKey`'s Write Params, before the box has read what was typed
    /// into it; other chords are the box's.
    #[test]
    fn ctrl_s_writes_what_changes_holds() {
        let now = Instant::now();
        let mut page = shown();
        page.step(index("TRIM_THROTTLE"), true, now);
        page.begin(index("THR_MAX"), now);
        assert!(
            page.key(&press("u", true), now),
            "the box's own chord: cleared"
        );
        for key in ["9", "0"] {
            page.key(&press(key, false), now);
        }
        assert!(!page.key(&press("q", true), now), "not the page's");
        assert!(!page.writing());
        assert!(page.key(&press("s", true), now));
        assert!(page.writing());
        let link = Answering::new(&[]);
        run(&mut page, &link, true);
        assert_eq!(link.taken(), [("TRIM_THROTTLE".to_owned(), 46.0)]);
        assert_eq!(page.editing(), Some(index("THR_MAX")));
        // The typed text is read when the box is left, and held for the next write.
        page.leave(now);
        assert_eq!(page.changes(), [("THR_MAX".to_owned(), 90.0)]);
    }

    /// Refresh Screen asks the vehicle for nothing and activates the page again: the table's
    /// values back in the boxes, `changes` cleared, the green left as `Activate` leaves it.
    #[test]
    fn refresh_screen_reads_nothing_and_activates_again() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default().faster(20));
        vehicle.send(&param("THR_MAX", 100.0, 9));
        until("the parameter", || telemetry.holds_parameter("THR_MAX"));
        let view = telemetry.view();
        let mut page = BasicTuning::default();
        page.activate(
            &view.parameters,
            Key::of(&view),
            true,
            Firmware::ArduPlane,
            plane_meta,
        );
        let now = Instant::now();
        page.step(index("THR_MAX"), false, now);
        assert_eq!(shown_text_of(&page, "THR_MAX"), "99");
        vehicle.read();
        let requests = |vehicle: &Vehicle| {
            vehicle.count(|message| {
                matches!(
                    message,
                    MavMessage::ParamRequestRead(_) | MavMessage::ParamRequestList(_)
                )
            })
        };
        let before = requests(&vehicle);
        page.refresh_screen(&view.parameters, true, Firmware::ArduPlane, plane_meta, now);
        assert_eq!(shown_text_of(&page, "THR_MAX"), "100");
        assert!(page.changes().is_empty());
        assert!(page.is_green(index("THR_MAX")), "Activate leaves BackColor");
        std::thread::sleep(Duration::from_millis(50));
        vehicle.read();
        assert_eq!(requests(&vehicle), before, "no parameter asked for");
    }

    fn shown_text_of(page: &BasicTuning, name: &str) -> String {
        page.numbers()[index(name)].shown().to_owned()
    }

    /// Refresh Params asks for the whole list, and when it is back activates the page with it.
    #[test]
    fn refresh_params_downloads_and_activates_with_the_new_list() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default().faster(20));
        vehicle.send(&param("THR_MAX", 100.0, 9));
        until("the parameter", || telemetry.holds_parameter("THR_MAX"));
        let view = telemetry.view();
        let mut page = BasicTuning::default();
        page.activate(
            &view.parameters,
            Key::of(&view),
            true,
            Firmware::ArduPlane,
            plane_meta,
        );
        let now = Instant::now();
        page.step(index("THR_MAX"), false, now);
        page.refresh_params(&telemetry, &view, now);
        assert!(page.refreshing());
        assert!(!page.live(), "held while the list comes");
        until("the request", || {
            vehicle.read();
            vehicle.count(|message| matches!(message, MavMessage::ParamRequestList(_))) > 0
        });
        vehicle.send(&param("THR_MAX", 80.0, 9));
        until("the page activated again", || {
            let view = telemetry.view();
            page.tick(
                &telemetry,
                &view,
                true,
                false,
                (true, Firmware::ArduPlane),
                plane_meta,
                Instant::now(),
            );
            !page.refreshing()
        });
        assert_eq!(shown_text_of(&page, "THR_MAX"), "80");
        assert!(page.changes().is_empty());
    }

    /// Against a scripted plane: its heartbeat makes it an ArduPlane, Write Params puts a
    /// `PARAM_SET` on the wire through the retrying set, and the echo clears the change.
    #[test]
    fn write_params_reaches_the_vehicle_through_the_retrying_set() {
        let (telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default().faster(20));
        vehicle.send(&MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 1,
            autopilot: 3,
            base_mode: 81,
            system_status: 3,
            mavlink_version: 3,
        }));
        vehicle.send(&param("THR_MAX", 100.0, 9));
        until("a plane with THR_MAX", || {
            let view = telemetry.view();
            telemetry.holds_parameter("THR_MAX")
                && Listed::of(&view, None).firmware == Firmware::ArduPlane
        });
        let view = telemetry.view();
        let listed = Listed::of(&view, None);
        let mut page = BasicTuning::default();
        page.activate(
            &view.parameters,
            Key::of(&view),
            listed.connected,
            listed.firmware,
            plane_meta,
        );
        assert!(page.enabled());
        let now = Instant::now();
        page.step(index("THR_MAX"), false, now);
        page.step(index("THR_MAX"), false, now);
        page.press_write(now);
        let mut written = None;
        until("the PARAM_SET", || {
            page.tick(
                &telemetry,
                &telemetry.view(),
                true,
                false,
                (true, Firmware::ArduPlane),
                plane_meta,
                Instant::now(),
            );
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    written = Some((mp_params::decode_param_id(&set.param_id), set.param_value));
                    vehicle.send(&param("THR_MAX", set.param_value, 9));
                }
            }
            written.is_some()
        });
        assert_eq!(written, Some(("THR_MAX".to_owned(), 98.0)));
        until("the echo", || {
            page.tick(
                &telemetry,
                &telemetry.view(),
                true,
                false,
                (true, Firmware::ArduPlane),
                plane_meta,
                Instant::now(),
            );
            !page.writing()
        });
        assert_eq!(page.last_write(), Some("THR_MAX 98 accepted"));
        assert!(page.changes().is_empty());
        assert!(!page.is_green(index("THR_MAX")));
        assert!(page.message().is_none());
    }

    /// The page object is the screen's: leaving the screen lets it go, changes and all.
    #[test]
    fn leaving_the_screen_lets_the_page_object_go() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut page = BasicTuning::default();
        page.activate(
            &plane(),
            Key::of(&view),
            true,
            Firmware::ArduPlane,
            plane_meta,
        );
        page.step(index("THR_MAX"), false, Instant::now());
        page.hide(Instant::now());
        let state = (true, Firmware::ArduPlane);
        page.tick(
            &telemetry,
            &view,
            true,
            false,
            state,
            plane_meta,
            Instant::now(),
        );
        assert_eq!(page.changes().len(), 1, "hidden, the object is kept");
        page.tick(
            &telemetry,
            &view,
            false,
            false,
            state,
            plane_meta,
            Instant::now(),
        );
        assert!(page.changes().is_empty());
        assert!(!page.is_green(index("THR_MAX")));
        assert_eq!(shown_text_of(&page, "THR_MAX"), "0");
    }

    /// Every fact the GUI script asserts on is one this page records, and the script clicks
    /// nothing of the page: against SITL's copter the page is not listed.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-basic-tuning.gui");
        let source = include_str!("basic_tuning.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.tuning.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key}");
                    facts += 1;
                }
                (Some("click"), Some(id)) => {
                    assert!(!id.starts_with("tuning-"), "{id}: the page is a plane's");
                }
                _ => {}
            }
        }
        assert!(facts >= 2, "{facts} facts");
        assert!(script.contains("expect config.page ConfigSimplePids"));
    }
}
