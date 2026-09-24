//! Camera Gimbal: `GCSViews/ConfigurationView/ConfigMount.cs`, an Optional Hardware page of Initial
//! Setup (`GCSViews/InitialSetup.cs:307-310`), listed once every parameter is in.
//!
//! What it shows: the mount type (`MNT_TYPE`), and for each of tilt, roll and pan the output its
//! servo is on, that output's limits, its reverse, the angle limits (`MNT_ANGMIN_TIL` and the
//! rest, in centidegrees shown as degrees) and the RC input that drives it (`MNT_RC_IN_TILT` and
//! the rest); stabilise tilt and roll; the neutral and retract angles; and the shutter - an
//! output, the relay or a transistor - with its limits, pushed and not-pushed pulses and duration.
//!
//! The four output boxes are plain combo boxes of channel names. Choosing one assigns the output:
//! the handler clears the function from any other output that has it, sets `MNT_MODE` to 3 where
//! there is one, and then, for the shutter, tilt, roll and pan in turn, writes the chosen output's
//! `_FUNCTION` (10, 7, 8 and 6) - or `CAM_TRIGG_TYPE` for the relay and the transistor - and binds
//! that axis's controls to the output's parameters (`ConfigMount.cs:170-310, 344-371`).
//! `Activate` reads which output has which function and runs the same four, so opening the page
//! writes what the vehicle already holds (`:104-168`). Every other control is a `Mavlink*` control
//! and writes its own parameter.
//!
//! `Activate` needs `CAM_TRIGG_TYPE`, the camera trigger of firmware before 4.3, and disables the
//! page without it - for the life of the page object (`:108-112`). Later firmware (`CAM1_TYPE`,
//! `MNT1_TYPE`) gets the page drawn and disabled, as in the C#.
//!
//! The handler's calls are made one after another against the vehicle's table as each earlier
//! one leaves it - the output the C#'s `ensureDisabled` clears is gone from the table before the
//! next call reads it - and a call that times out ends the handler with the C#'s box, "Failed to
//! set Param", which `Activate` follows by disabling the page. The controls each axis binds are
//! bound as the handler runs, from the table its calls will leave; the C# binds them after the
//! calls return, and so binds none after one that times out.
//!
//! The channel names are `Enum.GetNames` of the C#'s enums - by value, and among equal values in
//! the order they are declared, as .NET Framework's native sort leaves them - with the `RC` names
//! removed when the vehicle has `SERVO1_MIN`, else the `SERVO` names (`:37-98`); both firmware
//! branches build the same lists. `CAM_TRIGG_TYPE`'s name is `Enum.GetName`'s binary search over
//! those values, which finds Relay for 1 and Transistor for 4.
//!
//! The layout is `ConfigMount.resx`'s, every control at its `Location` in a 674 x 620 page.
//!
//! What is not ported, and why: the four pictures (`Resources.cameraGimalPitch1`, `Roll1`,
//! `Yaw` and `Shutter`), resources this application does not carry; and the Wiki link's colour
//! fade, `Transitions` animation, which is decoration.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::time::Instant;

use gpui::{AnyElement, Context, KeyDownEvent, Window, div, prelude::*, px, rgb};

use super::battery_monitor::param_text;
use super::optional::{
    Event, Focus, Job, Set, SetQueue, group, has, heading, label, message_box, picture, plain,
    rule, timeout_text, value_of,
};
use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::config::flight_modes::Firmware;
use crate::config::servo_output::{
    Check, Combo, Designer, Message, Number, NumberHandlers, OUT_OF_RANGE_TITLE, Question, Setup,
    Write, check_box, combo_box, dropdown, modal, number_box,
};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list, `backstageViewPagegimbal.Text`.
/// `// C#: GCSViews/InitialSetup.resx:540-542`
pub const TITLE: &str = "Camera Gimbal";

/// `ParamHead`.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:18`
const HEAD: &str = "MNT_";

/// The wiki page `LNK_wiki` opens.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:337-342`
pub const WIKI: &str = "http://copter.ardupilot.com/wiki/common-optional-hardware/common-cameras-and-gimbals/common-camera-gimbal/";

/// `label42.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.resx label42.Text`
pub const TRIGGER_NOTE: &str = "Please set the Ch7 Option to Camera Trigger";

/// `label44.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.resx label44.Text`
pub const TYPE_NOTE: &str =
    "NOTE: the gimbal type takes effect on the next reboot of the fight controller";

/// `Channelac` (and `Channelap`, the same): `Disable` 0, the rest 1, in declaration order.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:374-431`
const CHANNELS: [&str; 25] = [
    "Disable", "RC5", "RC6", "RC7", "RC8", "RC9", "RC10", "RC11", "RC12", "RC13", "RC14", "SERVO1",
    "SERVO2", "SERVO3", "SERVO4", "SERVO5", "SERVO6", "SERVO7", "SERVO8", "SERVO9", "SERVO10",
    "SERVO11", "SERVO12", "SERVO13", "SERVO14",
];

/// `ChannelCameraShutter`, as `Enum.GetNames` orders it: by value, declaration order among equals.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:433-462`
pub const SHUTTER: [(&str, u32); 27] = [
    ("Disable", 0),
    ("SERVO1", 1),
    ("Relay", 1),
    ("SERVO2", 2),
    ("SERVO3", 3),
    ("SERVO4", 4),
    ("Transistor", 4),
    ("RC5", 5),
    ("SERVO5", 5),
    ("RC6", 6),
    ("SERVO6", 6),
    ("RC7", 7),
    ("SERVO7", 7),
    ("RC8", 8),
    ("SERVO8", 8),
    ("RC9", 9),
    ("SERVO9", 9),
    ("RC10", 10),
    ("SERVO10", 10),
    ("RC11", 11),
    ("SERVO11", 11),
    ("RC12", 12),
    ("SERVO12", 12),
    ("RC13", 13),
    ("SERVO13", 13),
    ("RC14", 14),
    ("SERVO14", 14),
];

/// `Channelinput`, which the three input combos bind by name and write by value.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:464-479`
pub const INPUTS: [(i64, &str); 13] = [
    (0, "Disable"),
    (5, "RC5"),
    (6, "RC6"),
    (7, "RC7"),
    (8, "RC8"),
    (9, "RC9"),
    (10, "RC10"),
    (11, "RC11"),
    (12, "RC12"),
    (13, "RC13"),
    (14, "RC14"),
    (15, "RC15"),
    (16, "RC16"),
];

/// `Enum.GetName(typeof(ChannelCameraShutter), value)`: .NET Framework's binary search over the
/// sorted values, which among equal values lands where its halving takes it.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:116-117`
#[must_use]
pub fn shutter_name(value: i64) -> Option<&'static str> {
    let (mut low, mut high) = (0_i64, i64::try_from(SHUTTER.len()).ok()? - 1);
    while low <= high {
        let middle = low + ((high - low) >> 1);
        let (name, at) = SHUTTER.get(usize::try_from(middle).ok()?)?;
        match i64::from(*at).cmp(&value) {
            std::cmp::Ordering::Equal => return Some(name),
            std::cmp::Ordering::Less => low = middle + 1,
            std::cmp::Ordering::Greater => high = middle - 1,
        }
    }
    None
}

/// The four channel boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// `mavlinkComboBoxTilt`, function 7.
    Tilt,
    /// `mavlinkComboBoxRoll`, function 8.
    Roll,
    /// `mavlinkComboBoxPan`, function 6.
    Pan,
    /// `CMB_shuttertype`, function 10.
    Shutter,
}

impl Channel {
    /// The four.
    pub const ALL: [Self; 4] = [Self::Tilt, Self::Roll, Self::Pan, Self::Shutter];

    /// The id a script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Tilt => "mount-tilt",
            Self::Roll => "mount-roll",
            Self::Pan => "mount-pan",
            Self::Shutter => "mount-shutter",
        }
    }

    /// Its `Location.Y`; each is at x 90, 121 x 21.
    const fn y(self) -> f32 {
        match self {
            Self::Tilt => 34.0,
            Self::Roll => 162.0,
            Self::Pan => 286.0,
            Self::Shutter => 425.0,
        }
    }
}

/// The three `MavlinkComboBox`es of RC inputs, and the mount type's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    /// `CMB_mnt_type`.
    Type,
    /// `CMB_inputch_tilt`.
    Tilt,
    /// `CMB_inputch_roll`.
    Roll,
    /// `CMB_inputch_pan`.
    Pan,
}

impl Input {
    /// The four.
    pub const ALL: [Self; 4] = [Self::Type, Self::Tilt, Self::Roll, Self::Pan];

    /// The id a script clicks it by.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Type => "mount-MNT_TYPE",
            Self::Tilt => "mount-inputch-tilt",
            Self::Roll => "mount-inputch-roll",
            Self::Pan => "mount-inputch-pan",
        }
    }

    /// Its `Location` and width.
    const fn place(self) -> (f32, f32, f32) {
        match self {
            Self::Type => (90.0, 7.0, 121.0),
            Self::Tilt => (450.0, 85.0, 83.0),
            Self::Roll => (450.0, 212.0, 83.0),
            Self::Pan => (450.0, 336.0, 83.0),
        }
    }
}

/// One of the numbers: its name, its Designer state and its `Location`, every one 59 x 20.
#[derive(Debug, Clone, Copy)]
pub struct NumberSpec {
    /// Its C# name, less `mavlinkNumericUpDown`.
    pub name: &'static str,
    /// The Designer's `Minimum`, `Maximum`, `Value`.
    pub designer: Designer,
    /// Its `Location`, in the page or its group box.
    pub at: (f32, f32),
}

const fn designer(minimum: f64, maximum: f64, value: f64) -> Designer {
    Designer {
        minimum,
        maximum,
        value,
        decimals: 0,
    }
}

/// The numbers, from the Designer.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.designer.cs:386-1140; ConfigMount.resx *.Location`
pub const NUMBERS: [NumberSpec; 23] = [
    NumberSpec {
        name: "TSM",
        designer: designer(800.0, 2200.0, 1000.0),
        at: (276.0, 85.0),
    },
    NumberSpec {
        name: "TSMX",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (276.0, 111.0),
    },
    NumberSpec {
        name: "TAM",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (385.0, 85.0),
    },
    NumberSpec {
        name: "TAMX",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (385.0, 111.0),
    },
    NumberSpec {
        name: "RSM",
        designer: designer(800.0, 2200.0, 1000.0),
        at: (276.0, 212.0),
    },
    NumberSpec {
        name: "RSMX",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (276.0, 238.0),
    },
    NumberSpec {
        name: "RAM",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (385.0, 212.0),
    },
    NumberSpec {
        name: "RAMX",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (385.0, 238.0),
    },
    NumberSpec {
        name: "PSM",
        designer: designer(800.0, 2200.0, 1000.0),
        at: (276.0, 336.0),
    },
    NumberSpec {
        name: "PSMX",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (276.0, 362.0),
    },
    NumberSpec {
        name: "PAM",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (385.0, 336.0),
    },
    NumberSpec {
        name: "PAMX",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (385.0, 362.0),
    },
    NumberSpec {
        name: "NEUTRAL_x",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (54.0, 19.0),
    },
    NumberSpec {
        name: "NEUTRAL_y",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (54.0, 45.0),
    },
    NumberSpec {
        name: "NEUTRAL_z",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (54.0, 71.0),
    },
    NumberSpec {
        name: "RETRACT_x",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (54.0, 19.0),
    },
    NumberSpec {
        name: "RETRACT_y",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (54.0, 45.0),
    },
    NumberSpec {
        name: "RETRACT_z",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (54.0, 71.0),
    },
    NumberSpec {
        name: "ShutM",
        designer: designer(800.0, 2200.0, 1000.0),
        at: (276.0, 475.0),
    },
    NumberSpec {
        name: "ShutMX",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (276.0, 501.0),
    },
    NumberSpec {
        name: "shut_pushed",
        designer: designer(0.0, 2200.0, 1000.0),
        at: (423.0, 475.0),
    },
    NumberSpec {
        name: "shut_notpushed",
        designer: designer(800.0, 2200.0, 2000.0),
        at: (423.0, 501.0),
    },
    NumberSpec {
        name: "shut_duration",
        designer: designer(1.0, 100.0, 20.0),
        at: (423.0, 527.0),
    },
];

/// Where the neutral and retract numbers start in [`NUMBERS`].
const NEUTRAL: usize = 12;
/// The retract's.
const RETRACT: usize = 15;
/// The shutter's.
const SHUT: usize = 18;

/// `setup(800, 2200, 1, 1, ...)` for a servo's limits and the shutter's pulses.
const SERVO_LIMITS: Setup = Setup {
    minimum: 800.0,
    maximum: 2200.0,
    scale: 1.0,
    increment: 1.0,
};

/// `setup(-180, 180, 1, 1, ...)` for the neutral and retract angles.
const ANGLES: Setup = Setup {
    minimum: -180.0,
    maximum: 180.0,
    scale: 1.0,
    increment: 1.0,
};

/// An angle limit's `setup`: centidegrees shown as degrees.
const fn angle(minimum: f32, maximum: f32) -> Setup {
    Setup {
        minimum,
        maximum,
        scale: 100.0,
        increment: 1.0,
    }
}

/// The check boxes: the three reverses and the two stabilisations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tick {
    /// `mavlinkCheckBoxTR`.
    TiltReverse,
    /// `mavlinkCheckBoxRR`.
    RollReverse,
    /// `mavlinkCheckBoxPR`.
    PanReverse,
    /// `CHK_stab_tilt`.
    StabTilt,
    /// `CHK_stab_roll`.
    StabRoll,
}

impl Tick {
    /// The five.
    pub const ALL: [Self; 5] = [
        Self::TiltReverse,
        Self::RollReverse,
        Self::PanReverse,
        Self::StabTilt,
        Self::StabRoll,
    ];

    /// The id a script clicks it by, its text and its `Location`.
    const fn spec(self) -> (&'static str, &'static str, (f32, f32)) {
        match self {
            Self::TiltReverse => ("mount-TR", "Reverse", (267.0, 137.0)),
            Self::RollReverse => ("mount-RR", "Reverse", (267.0, 264.0)),
            Self::PanReverse => ("mount-PR", "Reverse", (267.0, 388.0)),
            Self::StabTilt => ("mount-stab-tilt", "Stabalise Tilt", (249.0, 36.0)),
            Self::StabRoll => ("mount-stab-roll", "Stabalise Roll", (249.0, 164.0)),
        }
    }
}

/// A handler's calls, made against the table as each earlier call leaves it.
#[derive(Debug)]
struct Plan {
    table: Vec<(String, f64)>,
    sets: Vec<Set>,
}

impl Plan {
    fn new(parameters: &[(String, f64)]) -> Self {
        Self {
            table: parameters.to_vec(),
            sets: Vec::new(),
        }
    }

    /// `setParam(name, value)`: a name the vehicle has takes the value.
    fn set(&mut self, name: &str, value: f64) {
        self.sets.push(Set::plain(name, value));
        if let Some((_, held)) = self.table.iter_mut().find(|(held, _)| held == name) {
            *held = value;
        }
    }

    /// `ensureDisabled`: each item's `_FUNCTION` that is `number` set to 0, but the one excluded.
    /// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:170-188`
    fn ensure_disabled(&mut self, items: &[String], number: f64, exclude: &str) {
        for item in items {
            let name = format!("{item}_FUNCTION");
            let Some(held) = value_of(&self.table, &name) else {
                continue;
            };
            if item == exclude {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)] // `(float)`
            let held = f64::from(held as f32);
            #[allow(clippy::float_cmp)]
            if held == number {
                self.set(&name, 0.0);
            }
        }
    }
}

/// The page object.
#[derive(Debug)]
pub struct Mount {
    made_for: Option<Key>,
    active: bool,
    /// The page's `Enabled`.
    enabled: bool,
    /// `startup`.
    startup: bool,
    /// The four channel boxes' items and selection, in [`Channel::ALL`] order; keyed by index.
    channels: [Combo; 4],
    /// The mount type and the three inputs, in [`Input::ALL`] order.
    inputs: [Combo; 4],
    /// The numbers, in [`NUMBERS`]' order.
    numbers: [Number; 23],
    /// The check boxes, in [`Tick::ALL`] order.
    ticks: [Check; 5],
    /// The combo whose list is down: a channel box, or an input by `4 + index`.
    dropdown: Option<usize>,
    editing: Option<usize>,
    question: Option<(usize, Question)>,
    messages: VecDeque<Message>,
    queue: SetQueue,
}

impl Default for Mount {
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            enabled: true,
            startup: true,
            channels: std::array::from_fn(|_| Combo::default()),
            inputs: std::array::from_fn(|_| Combo::default()),
            numbers: std::array::from_fn(|index| {
                Number::new(
                    NUMBERS
                        .get(index)
                        .map_or(crate::config::servo_output::NUMERIC_DEFAULTS, |spec| {
                            spec.designer
                        }),
                )
            }),
            ticks: std::array::from_fn(|_| Check::default()),
            dropdown: None,
            editing: None,
            question: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

/// A channel box's items: `names` less those starting with `remove`, keyed by position.
fn items(names: impl Iterator<Item = &'static str>, remove: &str) -> Vec<(i64, String)> {
    names
        .filter(|name| !name.starts_with(remove))
        .zip(0..)
        .map(|(name, key)| (key, name.to_owned()))
        .collect()
}

/// `ComboBox.Text = text` on a drop-down list: the item with that text, ignoring case, selected;
/// no such item, and the selection stays.
fn set_text(combo: &mut Combo, text: &str) {
    if let Some((key, _)) = combo
        .options
        .iter()
        .find(|(_, item)| item.eq_ignore_ascii_case(text))
    {
        combo.selected = Some(*key);
    }
}

impl Mount {
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

    /// A channel box.
    #[must_use]
    pub const fn channel(&self, channel: Channel) -> &Combo {
        let [tilt, roll, pan, shutter] = &self.channels;
        match channel {
            Channel::Tilt => tilt,
            Channel::Roll => roll,
            Channel::Pan => pan,
            Channel::Shutter => shutter,
        }
    }

    const fn channel_mut(&mut self, channel: Channel) -> &mut Combo {
        let [tilt, roll, pan, shutter] = &mut self.channels;
        match channel {
            Channel::Tilt => tilt,
            Channel::Roll => roll,
            Channel::Pan => pan,
            Channel::Shutter => shutter,
        }
    }

    /// A `MavlinkComboBox`.
    #[must_use]
    pub const fn input(&self, input: Input) -> &Combo {
        let [kind, tilt, roll, pan] = &self.inputs;
        match input {
            Input::Type => kind,
            Input::Tilt => tilt,
            Input::Roll => roll,
            Input::Pan => pan,
        }
    }

    const fn input_mut(&mut self, input: Input) -> &mut Combo {
        let [kind, tilt, roll, pan] = &mut self.inputs;
        match input {
            Input::Type => kind,
            Input::Tilt => tilt,
            Input::Roll => roll,
            Input::Pan => pan,
        }
    }

    const fn tick_mut(&mut self, tick: Tick) -> &mut Check {
        let [tilt, roll, pan, stab_tilt, stab_roll] = &mut self.ticks;
        match tick {
            Tick::TiltReverse => tilt,
            Tick::RollReverse => roll,
            Tick::PanReverse => pan,
            Tick::StabTilt => stab_tilt,
            Tick::StabRoll => stab_roll,
        }
    }

    /// `setup` for the number at `index` in [`NUMBERS`].
    fn setup_number(
        &mut self,
        index: usize,
        how: Setup,
        param: &str,
        table: &[(String, f64)],
        lookup: Lookup,
    ) {
        if let Some(number) = self.numbers.get_mut(index) {
            number.setup(how, param, table, lookup);
        }
    }

    /// A number, by its name in [`NUMBERS`].
    #[cfg(test)]
    #[must_use]
    pub fn number(&self, name: &str) -> Option<&Number> {
        NUMBERS
            .iter()
            .position(|spec| spec.name == name)
            .and_then(|index| self.numbers.get(index))
    }

    /// A check box.
    #[must_use]
    pub const fn check(&self, tick: Tick) -> &Check {
        let [tilt, roll, pan, stab_tilt, stab_roll] = &self.ticks;
        match tick {
            Tick::TiltReverse => tilt,
            Tick::RollReverse => roll,
            Tick::PanReverse => pan,
            Tick::StabTilt => stab_tilt,
            Tick::StabRoll => stab_roll,
        }
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// The question waiting.
    #[must_use]
    pub fn question(&self) -> Option<&Question> {
        self.question.as_ref().map(|(_, question)| question)
    }

    /// Dismisses the message box.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    fn channel_items(&self, channel: Channel) -> Vec<String> {
        self.channel(channel)
            .options
            .iter()
            .map(|(_, name)| name.clone())
            .collect()
    }

    /// The page object: its constructor fills the channel boxes and binds the mount type.
    /// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:21-102`
    fn construct(&mut self, parameters: &[(String, f64)], _firmware: Firmware, lookup: Lookup) {
        let remove = if has(parameters, "SERVO1_MIN") {
            "RC"
        } else {
            "SERVO"
        };
        for channel in [Channel::Tilt, Channel::Roll, Channel::Pan] {
            self.channel_mut(channel).options = items(CHANNELS.into_iter(), remove);
        }
        self.channel_mut(Channel::Shutter).options =
            items(SHUTTER.iter().map(|(name, _)| *name), remove);
        for combo in &mut self.channels {
            combo.enabled = true;
        }
        let param = format!("{HEAD}TYPE");
        self.input_mut(Input::Type)
            .setup(options(&param, lookup), &param, parameters);
    }

    /// The page object disposed, returning what its numbers' timers held.
    fn dispose(&mut self) -> Vec<Write> {
        let pending = self.numbers.iter_mut().filter_map(Number::flush).collect();
        let messages = std::mem::take(&mut self.messages);
        let queue = std::mem::take(&mut self.queue);
        *self = Self {
            messages,
            queue,
            ..Self::default()
        };
        pending
    }

    /// Shows the page: a new page object for a new screen, then `Activate`. Returns its writes.
    /// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:104-168`
    pub fn activate(
        &mut self,
        parameters: &[(String, f64)],
        key: Key,
        firmware: Firmware,
        lookup: Lookup,
    ) -> Vec<Job> {
        let mut jobs = Vec::new();
        if self.made_for != Some(key) {
            jobs.extend(self.dispose().into_iter().map(Job::control));
            self.made_for = Some(key);
            self.construct(parameters, firmware, lookup);
        }
        self.active = true;
        self.dropdown = None;
        let Some(trigger) = value_of(parameters, "CAM_TRIGG_TYPE") else {
            self.enabled = false;
            return jobs;
        };
        self.startup = true;
        // `SelectedItem = Enum.GetName(...)`: a name the box lacks leaves it, none clears it.
        #[allow(clippy::cast_possible_truncation)] // `(Int32)`
        match shutter_name(trigger as i64) {
            Some(name) => set_text(self.channel_mut(Channel::Shutter), name),
            None => self.channel_mut(Channel::Shutter).selected = None,
        }
        // `// C#: :119-141`
        for (name, value) in parameters {
            let Some(output) = name.strip_suffix("_FUNCTION") else {
                continue;
            };
            let channel = match param_text(*value).as_str() {
                "6" => Channel::Pan,
                "7" => Channel::Tilt,
                "8" => Channel::Roll,
                "10" => Channel::Shutter,
                _ => continue,
            };
            set_text(self.channel_mut(channel), output);
        }
        self.startup = false;
        let mut plan = Plan::new(parameters);
        self.update_all(&mut plan, lookup);
        let table = plan.table.clone();
        self.tick_mut(Tick::StabTilt)
            .setup(1.0, 0.0, &format!("{HEAD}STAB_TILT"), &table);
        self.tick_mut(Tick::StabRoll)
            .setup(1.0, 0.0, &format!("{HEAD}STAB_ROLL"), &table);
        for (offset, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
            let neutral = format!("{HEAD}NEUTRAL_{axis}");
            self.setup_number(NEUTRAL + offset, ANGLES, &neutral, &table, lookup);
            let retract = format!("{HEAD}RETRACT_{axis}");
            self.setup_number(RETRACT + offset, ANGLES, &retract, &table, lookup);
        }
        let mut job = Job::new("activate", plan.sets);
        job.on_throw = Some(failed_to_set);
        jobs.push(job);
        jobs
    }

    /// `updateShutter`, `updatePitch`, `updateRoll` and `updateYaw`, in that order.
    fn update_all(&mut self, plan: &mut Plan, lookup: Lookup) {
        self.update_shutter(plan, lookup);
        self.update_axis(Channel::Tilt, plan, lookup);
        self.update_axis(Channel::Roll, plan, lookup);
        self.update_axis(Channel::Pan, plan, lookup);
    }

    /// `updateShutter`.
    /// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:190-230`
    fn update_shutter(&mut self, plan: &mut Plan, lookup: Lookup) {
        let text = self.channel(Channel::Shutter).text().to_owned();
        if text.is_empty() {
            return;
        }
        let items = self.channel_items(Channel::Shutter);
        if text == "Disable" {
            plan.set("CAM_TRIGG_TYPE", 0.0);
            plan.ensure_disabled(&items, 10.0, "");
        } else if text == "Relay" {
            plan.ensure_disabled(&items, 10.0, "");
            plan.set("CAM_TRIGG_TYPE", 1.0);
        } else if text == "Transistor" {
            plan.ensure_disabled(&items, 10.0, "");
            plan.set("CAM_TRIGG_TYPE", 4.0);
        } else {
            plan.ensure_disabled(&items, 10.0, "");
            plan.set(&format!("{text}_FUNCTION"), 10.0);
            plan.set("CAM_TRIGG_TYPE", 0.0);
        }
        let binds: [(usize, Setup, String); 5] = [
            (SHUT, SERVO_LIMITS, format!("{text}_MIN")),
            (SHUT + 1, SERVO_LIMITS, format!("{text}_MAX")),
            (SHUT + 2, SERVO_LIMITS, "CAM_SERVO_ON".to_owned()),
            (SHUT + 3, SERVO_LIMITS, "CAM_SERVO_OFF".to_owned()),
            (
                SHUT + 4,
                Setup {
                    minimum: 1.0,
                    maximum: 200.0,
                    scale: 1.0,
                    increment: 1.0,
                },
                "CAM_DURATION".to_owned(),
            ),
        ];
        for (index, setup, param) in binds {
            self.setup_number(index, setup, &param, &plan.table, lookup);
        }
    }

    /// `updatePitch`, `updateRoll` or `updateYaw`.
    /// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:232-310`
    fn update_axis(&mut self, channel: Channel, plan: &mut Plan, lookup: Lookup) {
        let text = self.channel(channel).text().to_owned();
        if text.is_empty() {
            return;
        }
        let (function, first, (min_setup, max_setup), suffix, tick, input) = match channel {
            Channel::Tilt => (
                7.0,
                0,
                (angle(-90.0, 0.0), angle(0.0, 90.0)),
                "TIL",
                Tick::TiltReverse,
                Input::Tilt,
            ),
            Channel::Roll => (
                8.0,
                4,
                (angle(-90.0, 0.0), angle(0.0, 90.0)),
                "ROL",
                Tick::RollReverse,
                Input::Roll,
            ),
            Channel::Pan => (
                6.0,
                8,
                (angle(-180.0, 0.0), angle(0.0, 180.0)),
                "PAN",
                Tick::PanReverse,
                Input::Pan,
            ),
            Channel::Shutter => return,
        };
        if text == "Disable" {
            let items = self.channel_items(channel);
            plan.ensure_disabled(&items, function, "");
        } else {
            plan.set(&format!("{text}_FUNCTION"), function);
        }
        let table = &plan.table;
        self.setup_number(first, SERVO_LIMITS, &format!("{text}_MIN"), table, lookup);
        self.setup_number(
            first + 1,
            SERVO_LIMITS,
            &format!("{text}_MAX"),
            table,
            lookup,
        );
        let angmin = format!("{HEAD}ANGMIN_{suffix}");
        self.setup_number(first + 2, min_setup, &angmin, table, lookup);
        let angmax = format!("{HEAD}ANGMAX_{suffix}");
        self.setup_number(first + 3, max_setup, &angmax, table, lookup);
        // `setup(new double[] { -1, 1 }, new double[] { 1, 0 }, new[] { X_REV, X_REVERSED })`:
        // the first name the vehicle has, with its values; neither, and the box is untouched.
        let reverse = [
            (format!("{text}_REV"), -1.0, 1.0),
            (format!("{text}_REVERSED"), 1.0, 0.0),
        ];
        if let Some((name, on, off)) = reverse.iter().find(|(name, _, _)| has(table, name)) {
            self.tick_mut(tick).setup(*on, *off, name, table);
        }
        let param = match channel {
            Channel::Tilt => format!("{HEAD}RC_IN_TILT"),
            Channel::Roll => format!("{HEAD}RC_IN_ROLL"),
            _ => format!("{HEAD}RC_IN_PAN"),
        };
        let inputs = INPUTS
            .iter()
            .map(|(key, name)| (*key, (*name).to_owned()))
            .collect();
        self.input_mut(input).setup(inputs, &param, table);
    }

    /// The page hidden - it is `IActivate` only - which reads a number being typed into.
    pub fn hide(&mut self, now: Instant) {
        self.active = false;
        self.dropdown = None;
        self.leave(now);
    }

    /// Drops a list down, or back up: a channel box by its index, an input by `4 + index`.
    pub fn toggle_dropdown(&mut self, which: usize, now: Instant) {
        self.leave(now);
        let enabled = self.enabled
            && match which {
                0..4 => self.channels.get(which).is_some_and(|combo| combo.enabled),
                _ => self
                    .inputs
                    .get(which - 4)
                    .is_some_and(|combo| combo.enabled),
            };
        self.dropdown = if self.dropdown == Some(which) || !enabled {
            None
        } else {
            match which {
                0..4 => self.channels.get_mut(which).map(Combo::open_list),
                _ => self.inputs.get_mut(which - 4).map(Combo::open_list),
            };
            Some(which)
        };
    }

    /// The wheel over the open list.
    pub fn scroll_list(&mut self, which: usize, lines: i32) {
        if self.dropdown != Some(which) {
            return;
        }
        match which {
            0..4 => self
                .channels
                .get_mut(which)
                .map(|combo| combo.scroll_list(lines)),
            _ => self
                .inputs
                .get_mut(which - 4)
                .map(|combo| combo.scroll_list(lines)),
        };
    }

    /// A channel chosen: `mavlinkComboBox_SelectedIndexChanged`, when the row changed.
    /// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:344-371`
    pub fn choose_channel(
        &mut self,
        channel: Channel,
        key: i64,
        parameters: &[(String, f64)],
        lookup: Lookup,
    ) -> Vec<Job> {
        self.dropdown = None;
        if !self.enabled || !self.channel_mut(channel).select(key) || self.startup {
            return Vec::new();
        }
        let mut plan = Plan::new(parameters);
        let items = self.channel_items(channel);
        let pan = self.channel(Channel::Pan).text().to_owned();
        let tilt = self.channel(Channel::Tilt).text().to_owned();
        let roll = self.channel(Channel::Roll).text().to_owned();
        plan.ensure_disabled(&items, 6.0, &pan);
        plan.ensure_disabled(&items, 7.0, &tilt);
        plan.ensure_disabled(&items, 8.0, &roll);
        let mode = format!("{HEAD}MODE");
        if has(&plan.table, &mode) {
            plan.set(&mode, 3.0);
        }
        self.update_all(&mut plan, lookup);
        let mut job = Job::new("channel", plan.sets);
        job.on_throw = Some(failed_to_set);
        vec![job]
    }

    /// The mount type or an input chosen: the control's own write. The inputs are bound to an
    /// enum, and fail as `Strings.ErrorSetValueFailed` says; the type to a list, with "!".
    /// `// C#: Controls/MavlinkComboBox.cs:138-199`
    pub fn choose_input(&mut self, input: Input, key: i64) -> Vec<Job> {
        self.dropdown = None;
        let enabled = self.enabled;
        let combo = self.input_mut(input);
        if !enabled || !combo.enabled {
            return Vec::new();
        }
        let write = match input {
            Input::Type => combo.choose(key),
            _ => combo.select(key).then(|| {
                #[allow(clippy::cast_precision_loss)]
                let value = key as f64;
                Write::other(&combo.param, value)
            }),
        };
        write.map(Job::control).into_iter().collect()
    }

    /// A check box clicked: its own write.
    pub fn click_tick(&mut self, tick: Tick, now: Instant) -> Vec<Job> {
        self.leave(now);
        self.dropdown = None;
        if !self.enabled {
            return Vec::new();
        }
        self.tick_mut(tick)
            .click()
            .map(Job::control)
            .into_iter()
            .collect()
    }

    fn number_live(&self, index: usize) -> bool {
        self.enabled && self.numbers.get(index).is_some_and(|number| number.enabled)
    }

    /// A number clicked into.
    pub fn begin(&mut self, index: usize, now: Instant) {
        if self.editing == Some(index) {
            return;
        }
        self.leave(now);
        self.dropdown = None;
        if self.number_live(index) {
            self.editing = Some(index);
        }
    }

    /// The number being typed into loses the focus.
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
        if !self.number_live(index) {
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

    /// Once a frame: a page object whose screen has gone is disposed, a number that lost the
    /// focus is read, the timers, and the writes - `Activate`'s `catch` disabling the page.
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
            let pending: Vec<Job> = self.dispose().into_iter().map(Job::control).collect();
            self.queue.push(pending);
        }
        if self.editing.is_some() && !focused {
            self.leave(now);
        }
        let due: Vec<Job> = self
            .numbers
            .iter_mut()
            .filter_map(|number| number.due(now))
            .map(Job::control)
            .collect();
        self.queue.push(due);
        let events = self.queue.advance(telemetry, &mut self.messages);
        self.absorb(&events);
    }

    /// What the writes did: `Activate`'s `catch` disables the page.
    /// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:163-167`
    fn absorb(&mut self, events: &[Event]) {
        for event in events {
            if matches!(
                event,
                Event::Done {
                    tag: "activate",
                    threw: true
                }
            ) {
                self.enabled = false;
            }
        }
    }
}

/// The handlers' `catch`: "Failed to set Param\n" and the exception.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.cs:163-167, 367-370`
#[must_use]
pub fn failed_to_set(param: &str) -> Message {
    plain(format!("Failed to set Param\n{}", timeout_text(param)))
}

/// A box's text for the facts, "none" when it shows nothing.
fn or_none(text: &str) -> &str {
    if text.is_empty() { "none" } else { text }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &Mount, view: &TelemetryView) {
    use crate::facts::record;
    record("config.mount.active", page.is_active());
    record("config.mount.enabled", page.enabled());
    record("config.mount.note", TYPE_NOTE);
    record("config.mount.trigger", TRIGGER_NOTE);
    for channel in Channel::ALL {
        let key = channel.id().trim_start_matches("mount-");
        let combo = page.channel(channel);
        record(format!("config.mount.{key}"), or_none(combo.text()));
        record(format!("config.mount.{key}.options"), combo.options.len());
    }
    for input in Input::ALL {
        let key = input.id().trim_start_matches("mount-");
        let combo = page.input(input);
        record(format!("config.mount.{key}"), or_none(combo.text()));
        record(format!("config.mount.{key}.enabled"), combo.enabled);
        record(format!("config.mount.{key}.param"), &combo.param);
    }
    for (number, spec) in page.numbers.iter().zip(NUMBERS) {
        record(format!("config.mount.{}", spec.name), number.shown());
        record(
            format!("config.mount.{}.enabled", spec.name),
            number.enabled,
        );
        record(format!("config.mount.{}.param", spec.name), &number.param);
    }
    for tick in Tick::ALL {
        let (id, _, _) = tick.spec();
        let key = id.trim_start_matches("mount-");
        let check = page.check(tick);
        record(format!("config.mount.{key}"), check.state.key());
        record(format!("config.mount.{key}.enabled"), check.enabled);
    }
    record("config.mount.write", page.queue.last().unwrap_or("none"));
    record("config.mount.writes.pending", page.queue.pending());
    record(
        "config.mount.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
    for (name, value) in view.parameters.iter() {
        if name.starts_with("MNT") || name.starts_with("CAM") {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// The labels, at their `Location`s: `(x, y, text, 12-point)`.
/// `// C#: GCSViews/ConfigurationView/ConfigMount.resx label*.Text, label*.Location, label*.Font`
const LABELS: [(f32, f32, &str, bool); 36] = [
    (21.0, 7.0, "Type", true),
    (246.0, 10.0, TYPE_NOTE, false),
    (21.0, 35.0, "Tilt", true),
    (251.0, 62.0, "Servo Limits", true),
    (352.0, 62.0, "Angle Limits", true),
    (453.0, 62.0, "Input Ch", true),
    (246.0, 87.0, "Min", false),
    (246.0, 115.0, "Max", false),
    (355.0, 87.0, "Min", false),
    (355.0, 115.0, "Max", false),
    (20.0, 163.0, "Roll", true),
    (251.0, 189.0, "Servo Limits", true),
    (352.0, 189.0, "Angle Limits", true),
    (453.0, 189.0, "Input Ch", true),
    (246.0, 214.0, "Min", false),
    (246.0, 242.0, "Max", false),
    (355.0, 214.0, "Min", false),
    (355.0, 242.0, "Max", false),
    (20.0, 287.0, "Pan", true),
    (251.0, 313.0, "Servo Limits", true),
    (352.0, 313.0, "Angle Limits", true),
    (453.0, 313.0, "Input Ch", true),
    (246.0, 338.0, "Min", false),
    (246.0, 366.0, "Max", false),
    (355.0, 338.0, "Min", false),
    (355.0, 366.0, "Max", false),
    (20.0, 426.0, "Shutter", true),
    (251.0, 452.0, "Servo Limits", true),
    (402.0, 452.0, "Shutter", true),
    (246.0, 477.0, "Min", false),
    (246.0, 505.0, "Max", false),
    (374.0, 477.0, "Pushed", false),
    (354.0, 505.0, "Not Pushed", false),
    (346.0, 524.0, "Duration", false),
    (346.0, 537.0, "(1/10th sec)", false),
    (180.0, 559.0, TRIGGER_NOTE, true),
];

/// The page, laid out as `ConfigMount.resx` lays it out.
pub fn page(
    mount: &Mount,
    focus: &Focus,
    _view: &TelemetryView,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !mount.is_active() {
        return div().into_any_element();
    }
    let enabled = mount.enabled;
    let mut body = div()
        .relative()
        .w(px(674.0))
        .h(px(620.0))
        .child(picture("cameraGimalPitch1", (33.0, 47.0, 203.0, 112.0)))
        .child(picture("cameraGimalRoll1", (33.0, 172.0, 203.0, 112.0)))
        .child(picture("cameraGimalYaw", (33.0, 296.0, 203.0, 112.0)))
        .child(picture("Shutter", (33.0, 435.0, 203.0, 112.0)))
        .child(rule(17.0, 54.0, 516.0))
        .child(rule(17.0, 181.0, 516.0))
        .child(rule(17.0, 305.0, 516.0))
        .child(rule(17.0, 444.0, 516.0));
    for (x, y, text, large) in LABELS {
        body = body.child(if large {
            heading(x, y, text, enabled)
        } else {
            label(x, y, text, enabled)
        });
    }
    body = body.child(
        crate::probe::measured("mount-wiki", div())
            .id("mount-wiki")
            .absolute()
            .left(px(624.0))
            .top(px(12.0))
            .text_xs()
            .text_color(rgb(if enabled { theme::ACCENT } else { theme::DIM }))
            .cursor_pointer()
            .hover(|style| style.underline())
            .child("Wiki")
            .on_click(move |_event, _window, cx| {
                if enabled {
                    cx.open_url(WIKI);
                }
            }),
    );
    let number_box_at = |index: usize, (x, y): (f32, f32), cx: &mut Context<MissionPlanner>| {
        let spec = NUMBERS.get(index).copied();
        let name = spec.map_or("", |spec| spec.name);
        let Some(number) = mount.numbers.get(index) else {
            return div().into_any_element();
        };
        let mut shown = Number::new(Designer {
            minimum: number.minimum,
            maximum: number.maximum,
            value: number.shown().parse().unwrap_or(0.0),
            decimals: number.decimals,
        });
        let number = if enabled {
            number
        } else {
            shown.enabled = false;
            &shown
        };
        number_box(
            format!("mount-{name}"),
            number,
            mount.editing == Some(index),
            &focus.number,
            (x, y, 59.0, 20.0),
            NumberHandlers {
                begin: move |this: &mut MissionPlanner| {
                    this.optional.mount.begin(index, Instant::now());
                },
                key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                    this.optional.mount.key(event, Instant::now())
                },
                step: move |this: &mut MissionPlanner, up: bool| {
                    this.optional.mount.step(index, up, Instant::now());
                },
            },
            window,
            cx,
        )
    };
    for (index, spec) in NUMBERS.iter().enumerate() {
        if (NEUTRAL..SHUT).contains(&index) {
            continue;
        }
        body = body.child(number_box_at(index, spec.at, cx));
    }
    for (caption, first, y) in [
        ("Retract Angles", RETRACT, 97.0),
        ("Neutral Angles", NEUTRAL, 200.0),
    ] {
        let mut group_box = group((551.0, y, 119.0, 97.0), caption, enabled);
        for (offset, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
            let index = first + offset;
            #[allow(clippy::cast_precision_loss)]
            let row = 21.0 + 26.0 * offset as f32;
            group_box = group_box
                .child(label(7.0, row, axis, enabled))
                .child(number_box_at(
                    index,
                    NUMBERS.get(index).map_or((0.0, 0.0), |spec| spec.at),
                    cx,
                ));
        }
        body = body.child(group_box);
    }
    for tick in Tick::ALL {
        let (id, text, place) = tick.spec();
        let mut check = mount.check(tick).clone();
        check.enabled &= enabled;
        body = body.child(check_box(
            id.to_owned(),
            &check,
            text,
            place,
            move |this| {
                let jobs = this.optional.mount.click_tick(tick, Instant::now());
                this.optional.mount.push(jobs);
            },
            cx,
        ));
    }
    for (index, channel) in Channel::ALL.into_iter().enumerate() {
        let mut combo = mount.channel(channel).clone();
        combo.enabled &= enabled;
        body = body.child(combo_box(
            channel.id().to_owned(),
            &combo,
            (90.0, channel.y(), 121.0, 21.0),
            move |this| this.optional.mount.toggle_dropdown(index, Instant::now()),
            cx,
        ));
    }
    for (index, input) in Input::ALL.into_iter().enumerate() {
        let mut combo = mount.input(input).clone();
        combo.enabled &= enabled;
        let (x, y, width) = input.place();
        body = body.child(combo_box(
            input.id().to_owned(),
            &combo,
            (x, y, width, 21.0),
            move |this| {
                this.optional
                    .mount
                    .toggle_dropdown(4 + index, Instant::now())
            },
            cx,
        ));
    }
    if let Some(which) = mount.dropdown {
        if let Some(channel) = Channel::ALL.get(which).copied() {
            body = body.child(dropdown(
                channel.id(),
                mount.channel(channel),
                (90.0, channel.y() + 21.0, 121.0),
                move |this, key| {
                    let view = this.telemetry.view();
                    let jobs = this.optional.mount.choose_channel(
                        channel,
                        key,
                        &view.parameters,
                        crate::metadata::lookup,
                    );
                    this.optional.mount.push(jobs);
                },
                move |this, lines| this.optional.mount.scroll_list(which, lines),
                cx,
            ));
        } else if let Some(input) = Input::ALL.get(which - 4).copied() {
            let (x, y, width) = input.place();
            body = body.child(dropdown(
                input.id(),
                mount.input(input),
                (x, y + 21.0, width.max(121.0)),
                move |this, key| {
                    let jobs = this.optional.mount.choose_input(input, key);
                    this.optional.mount.push(jobs);
                },
                move |this, lines| this.optional.mount.scroll_list(which, lines),
                cx,
            ));
        }
    }
    panel(TITLE, body).into_any_element()
}

/// The question or message box showing, over the whole window.
pub fn overlay(
    mount: &Mount,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(question) = mount.question() {
        let buttons = vec![
            action(
                "mount-question-yes",
                "Yes",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.optional.mount.answer(true, Instant::now());
                    cx.notify();
                }),
            ),
            action(
                "mount-question-no",
                "No",
                theme::ACCENT,
                true,
                cx.listener(|this, _event: &(), _window, cx| {
                    this.optional.mount.answer(false, Instant::now());
                    cx.notify();
                }),
            ),
        ];
        return Some(modal(
            "mount-question",
            OUT_OF_RANGE_TITLE,
            &question.text(),
            false,
            buttons,
            window,
        ));
    }
    let message = mount.message()?;
    Some(message_box(
        "mount-message",
        "mount-message-ok",
        message,
        window,
        |this| this.optional.mount.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::failsafe::CheckState;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::{Answering, drain};
    use mp_link::requests::RequestOutcome;

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

    fn run(jobs: Vec<Job>, link: &Answering) -> Vec<Message> {
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        drain(&mut queue, link)
    }

    /// A copter of 4.0 to 4.2: `CAM_TRIGG_TYPE`, the old `MNT_` names, a tilt servo on output
    /// 9 and a shutter servo on 10.
    fn gimbal() -> Vec<(String, f64)> {
        table(&[
            ("CAM_TRIGG_TYPE", 0.0),
            ("CAM_SERVO_ON", 1300.0),
            ("CAM_SERVO_OFF", 1100.0),
            ("CAM_DURATION", 10.0),
            ("MNT_TYPE", 1.0),
            ("MNT_MODE", 3.0),
            ("MNT_STAB_TILT", 1.0),
            ("MNT_STAB_ROLL", 0.0),
            ("MNT_ANGMIN_TIL", -4500.0),
            ("MNT_ANGMAX_TIL", 0.0),
            ("MNT_RC_IN_TILT", 6.0),
            ("MNT_NEUTRAL_X", 0.0),
            ("SERVO1_MIN", 1100.0),
            ("SERVO1_FUNCTION", 33.0),
            ("SERVO9_FUNCTION", 7.0),
            ("SERVO9_MIN", 1000.0),
            ("SERVO9_MAX", 2000.0),
            ("SERVO9_REVERSED", 0.0),
            ("SERVO10_FUNCTION", 10.0),
            ("SERVO10_MIN", 1100.0),
            ("SERVO10_MAX", 1900.0),
            ("SERVO11_FUNCTION", 0.0),
        ])
    }

    #[test]
    fn the_text_is_the_resx_text() {
        let Some(resx) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigMount.resx")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let values = crate::config_coverage::source::resx(&resx);
        let get = |key: &str| values.get(key).map(String::as_str);
        assert_eq!(get("label42.Text"), Some(TRIGGER_NOTE));
        assert_eq!(get("label44.Text"), Some(TYPE_NOTE));
        assert_eq!(get("label34.Text"), Some("Duration (1/10th sec)"));
        assert_eq!(get("groupBox4.Text"), Some("Retract Angles"));
        assert_eq!(get("groupBox5.Text"), Some("Neutral Angles"));
        assert_eq!(get("CHK_stab_tilt.Text"), Some("Stabalise Tilt"));
        assert_eq!(get("CMB_mnt_type.Location"), Some("90, 7"));
        for channel in Channel::ALL {
            let name = match channel {
                Channel::Tilt => "mavlinkComboBoxTilt",
                Channel::Roll => "mavlinkComboBoxRoll",
                Channel::Pan => "mavlinkComboBoxPan",
                Channel::Shutter => "CMB_shuttertype",
            };
            assert_eq!(
                get(&format!("{name}.Location")),
                Some(format!("90, {}", channel.y()).as_str())
            );
        }
        let names = [
            "mavlinkNumericUpDownTSM",
            "mavlinkNumericUpDownTSMX",
            "mavlinkNumericUpDownTAM",
            "mavlinkNumericUpDownTAMX",
            "mavlinkNumericUpDownRSM",
            "mavlinkNumericUpDownRSMX",
            "mavlinkNumericUpDownRAM",
            "mavlinkNumericUpDownRAMX",
            "mavlinkNumericUpDownPSM",
            "mavlinkNumericUpDownPSMX",
            "mavlinkNumericUpDownPAM",
            "mavlinkNumericUpDownPAMX",
            "NUD_NEUTRAL_x",
            "NUD_NEUTRAL_y",
            "NUD_NEUTRAL_z",
            "NUD_RETRACT_x",
            "NUD_RETRACT_y",
            "NUD_RETRACT_z",
            "mavlinkNumericUpDownShutM",
            "mavlinkNumericUpDownShutMX",
            "mavlinkNumericUpDownshut_pushed",
            "mavlinkNumericUpDownshut_notpushed",
            "mavlinkNumericUpDownshut_duration",
        ];
        for (name, spec) in names.iter().zip(NUMBERS) {
            let (x, y) = spec.at;
            assert_eq!(
                get(&format!("{name}.Location")),
                Some(format!("{x}, {y}").as_str()),
                "{name}"
            );
        }
        let texts: Vec<&str> = LABELS.iter().map(|(_, _, text, _)| *text).collect();
        for index in [1, 2, 3, 4, 5, 6, 9, 10, 15, 22, 35, 36, 37, 38, 41, 43] {
            let text = get(&format!("label{index}.Text")).expect("a label");
            assert!(texts.contains(&text), "label{index} {text:?}");
        }
    }

    #[test]
    fn the_shutters_names_are_net_frameworks() {
        assert_eq!(shutter_name(0), Some("Disable"));
        assert_eq!(shutter_name(1), Some("Relay"));
        assert_eq!(shutter_name(4), Some("Transistor"));
        assert_eq!(shutter_name(2), Some("SERVO2"));
        assert_eq!(shutter_name(15), None);
    }

    /// SITL's copter of 4.5: no `CAM_TRIGG_TYPE`, so the page is drawn and disabled; the boxes
    /// hold the `SERVO` names, and `MNT_TYPE` is not there (`MNT1_TYPE` is).
    #[test]
    fn a_later_copter_gets_the_page_disabled() {
        let mut page = Mount::default();
        let parameters = table(&[
            ("SERVO1_MIN", 1100.0),
            ("MNT1_TYPE", 0.0),
            ("CAM1_TYPE", 0.0),
        ]);
        let jobs = page.activate(&parameters, key(), Firmware::ArduCopter2, bundled);
        assert!(jobs.is_empty(), "nothing written");
        assert!(!page.enabled());
        assert_eq!(page.channel(Channel::Tilt).options.len(), 15);
        assert_eq!(page.channel(Channel::Tilt).options[1].1, "SERVO1");
        assert_eq!(page.channel(Channel::Shutter).options.len(), 17);
        let shutter: Vec<&str> = page
            .channel(Channel::Shutter)
            .options
            .iter()
            .take(7)
            .map(|(_, name)| name.as_str())
            .collect();
        assert_eq!(
            shutter,
            [
                "Disable",
                "SERVO1",
                "Relay",
                "SERVO2",
                "SERVO3",
                "SERVO4",
                "Transistor"
            ]
        );
        assert!(!page.input(Input::Type).enabled);
        assert!(
            page.choose_channel(Channel::Tilt, 1, &parameters, bundled)
                .is_empty()
        );
        // Old firmware without SERVO1_MIN keeps the RC names.
        let mut old = Mount::default();
        old.activate(
            &table(&[("RC1_MIN", 1100.0)]),
            key(),
            Firmware::ArduCopter2,
            bundled,
        );
        assert_eq!(old.channel(Channel::Pan).options.len(), 11);
        assert_eq!(old.channel(Channel::Pan).options[1].1, "RC5");
    }

    /// `Activate` finds tilt on SERVO9 and the shutter on SERVO10, writes what the vehicle holds -
    /// the shutter's output cleared and set again, as `ensureDisabled` runs over it first - and
    /// binds each axis's controls.
    #[test]
    fn activate_reads_the_outputs_and_writes_them_back() {
        let mut page = Mount::default();
        let jobs = page.activate(&gimbal(), key(), Firmware::ArduCopter2, bundled);
        assert!(page.enabled());
        assert_eq!(page.channel(Channel::Tilt).text(), "SERVO9");
        assert_eq!(page.channel(Channel::Shutter).text(), "SERVO10");
        assert_eq!(page.channel(Channel::Roll).text(), "");
        let link = Answering::new(&[]);
        assert!(run(jobs, &link).is_empty());
        assert_eq!(
            link.taken(),
            [
                ("SERVO10_FUNCTION".to_owned(), 0.0),
                ("SERVO10_FUNCTION".to_owned(), 10.0),
                ("CAM_TRIGG_TYPE".to_owned(), 0.0),
                ("SERVO9_FUNCTION".to_owned(), 7.0),
            ]
        );
        assert_eq!(page.number("TSM").map(Number::shown), Some("1000"));
        assert_eq!(
            page.number("TSMX").map(|n| n.param.as_str()),
            Some("SERVO9_MAX")
        );
        assert_eq!(page.number("TAM").map(Number::shown), Some("-45"));
        assert_eq!(page.number("ShutM").map(Number::shown), Some("1100"));
        assert_eq!(page.number("shut_pushed").map(Number::shown), Some("1300"));
        assert_eq!(page.number("shut_duration").map(Number::shown), Some("10"));
        assert_eq!(page.number("NEUTRAL_x").map(|n| n.enabled), Some(true));
        assert_eq!(page.number("RETRACT_x").map(|n| n.enabled), Some(false));
        assert_eq!(page.check(Tick::TiltReverse).state, CheckState::Unchecked);
        assert!(page.check(Tick::TiltReverse).enabled);
        assert_eq!(page.check(Tick::StabTilt).state, CheckState::Checked);
        assert_eq!(page.input(Input::Tilt).text(), "RC6");
        assert!(page.input(Input::Type).enabled);
        assert_eq!(page.input(Input::Type).text(), "Servo");
    }

    /// Choosing SERVO11 for roll: nothing else has 8, `MNT_MODE` to 3, then the shutter and tilt
    /// as `Activate` wrote them, and SERVO11 made the roll servo.
    #[test]
    fn choosing_an_output_assigns_it() {
        let mut page = Mount::default();
        let parameters = gimbal();
        page.activate(&parameters, key(), Firmware::ArduCopter2, bundled);
        let servo11 = page
            .channel(Channel::Roll)
            .options
            .iter()
            .find(|(_, name)| name == "SERVO11")
            .map(|(key, _)| *key)
            .expect("SERVO11");
        let jobs = page.choose_channel(Channel::Roll, servo11, &parameters, bundled);
        let link = Answering::new(&[]);
        assert!(run(jobs, &link).is_empty());
        let taken = link.taken();
        assert_eq!(taken.first(), Some(&("MNT_MODE".to_owned(), 3.0)));
        assert_eq!(taken.last(), Some(&("SERVO11_FUNCTION".to_owned(), 8.0)));
        assert_eq!(
            page.number("RSM").map(|n| n.param.as_str()),
            Some("SERVO11_MIN")
        );
        // Moving tilt to SERVO11 too clears the roll's function from it first.
        let mut later = parameters.clone();
        for (name, value) in &mut later {
            if name == "SERVO11_FUNCTION" {
                *value = 8.0;
            }
        }
        let jobs = page.choose_channel(Channel::Tilt, servo11, &later, bundled);
        let link = Answering::new(&[]);
        run(jobs, &link);
        let taken = link.taken();
        assert!(
            taken.contains(&("SERVO9_FUNCTION".to_owned(), 0.0)),
            "{taken:?}"
        );
        assert!(
            taken.contains(&("SERVO11_FUNCTION".to_owned(), 7.0)),
            "{taken:?}"
        );
        // Roll is still SERVO11, and its update comes after tilt's: the C#'s last write wins.
        assert_eq!(taken.last(), Some(&("SERVO11_FUNCTION".to_owned(), 8.0)));
    }

    /// The relay: `CAM_TRIGG_TYPE` 1, the shutter's servo output cleared.
    #[test]
    fn the_relay_sets_the_trigger_type() {
        let mut page = Mount::default();
        let parameters = gimbal();
        page.activate(&parameters, key(), Firmware::ArduCopter2, bundled);
        let relay = page
            .channel(Channel::Shutter)
            .options
            .iter()
            .find(|(_, name)| name == "Relay")
            .map(|(key, _)| *key)
            .expect("Relay");
        let jobs = page.choose_channel(Channel::Shutter, relay, &parameters, bundled);
        let link = Answering::new(&[]);
        run(jobs, &link);
        let taken = link.taken();
        assert!(taken.contains(&("SERVO10_FUNCTION".to_owned(), 0.0)));
        assert!(taken.contains(&("CAM_TRIGG_TYPE".to_owned(), 1.0)));
        assert!(
            !page.number("ShutM").is_some_and(|n| n.enabled),
            "Relay_MIN is nothing"
        );
    }

    /// A timeout in `Activate`'s calls: the C#'s box, and the page disabled.
    #[test]
    fn a_timeout_in_activate_disables_the_page() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut page = Mount::default();
        let jobs = page.activate(&gimbal(), Key::of(&view), Firmware::ArduCopter2, bundled);
        let link = Answering::new(&[(
            "SERVO10_FUNCTION",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        let mut queue = SetQueue::<usize>::default();
        queue.push(jobs);
        let mut messages = VecDeque::new();
        let events = queue.advance(&link, &mut messages);
        assert_eq!(link.taken().len(), 1, "the rest are not reached");
        assert_eq!(messages.front(), Some(&failed_to_set("SERVO10_FUNCTION")));
        assert!(events.contains(&Event::Done {
            tag: "activate",
            threw: true
        }));
        assert!(page.enabled());
        page.absorb(&events);
        assert!(!page.enabled(), "the catch disables the page");
        // A channel's handler that throws says the same and leaves the page as it is.
        let mut page = Mount::default();
        page.activate(&gimbal(), Key::of(&view), Firmware::ArduCopter2, bundled);
        page.absorb(&[Event::Done {
            tag: "channel",
            threw: true,
        }]);
        assert!(page.enabled());
        let _ = telemetry;
    }

    #[test]
    fn the_inputs_write_as_an_enum_and_the_type_as_a_list() {
        let mut page = Mount::default();
        page.activate(&gimbal(), key(), Firmware::ArduCopter2, bundled);
        let jobs = page.choose_input(Input::Tilt, 7);
        let link = Answering::new(&[(
            "MNT_RC_IN_TILT",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        assert_eq!(
            run(jobs, &link),
            [crate::config::optional::error("Set MNT_RC_IN_TILT Failed")]
        );
        let jobs = page.choose_input(Input::Type, 2);
        let link = Answering::new(&[("MNT_TYPE", Progress::Finished(RequestOutcome::TimedOut))]);
        assert_eq!(
            run(jobs, &link),
            [crate::config::optional::error("Set MNT_TYPE Failed!")]
        );
    }

    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-mount.gui");
        let source = include_str!("mount.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.mount.") => {
                    let rest = key.trim_start_matches("config.mount.");
                    let base = rest.split('.').next().unwrap_or(rest);
                    let known = source.contains(&format!("\"{key}\""))
                        || NUMBERS.iter().any(|spec| spec.name == base)
                        || Channel::ALL
                            .iter()
                            .any(|c| c.id() == format!("mount-{base}"))
                        || Input::ALL.iter().any(|i| i.id() == format!("mount-{base}"))
                        || Tick::ALL
                            .iter()
                            .any(|t| t.spec().0 == format!("mount-{base}"));
                    assert!(known, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) => {
                    assert_ne!(id, "mount-wiki", "the script must not open a browser");
                }
                _ => {}
            }
        }
        assert!(facts > 8, "{facts} facts");
    }
}
