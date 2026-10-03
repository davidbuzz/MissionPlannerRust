// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! Custom warnings: `ExtLibs/Utilities/Warnings/CustomWarning.cs` and `WarningEngine.cs` - the
//! rules the Warning Manager (`config/warnings_manager.rs`) edits, the file they are kept in, and
//! the loop that checks them.
//!
//! What it does:
//!
//! * a rule, [`CustomWarning`], names a `CurrentState` property, a comparison and a number. It
//!   holds when the property's value - what its getter returns, so in the user's units - compares
//!   so with the number, and then cannot hold again for `RepeatTime` seconds (`CheckValue`,
//!   `CustomWarning.cs:181-225`). A rule's `Child` must hold as well, checked only when the rule
//!   itself does (`checkCond`, `WarningEngine.cs:134-150`);
//! * the rules are `warnings.xml` in the user data directory: read when the application starts,
//!   the whole file or nothing (`LoadConfig`, `:30-43`), and written by the manager's Save as
//!   `XmlSerializer` writes a `List<CustomWarning>` (`SaveConfig`, `:45-59`);
//! * every 250 ms each rule is checked in turn (`MainLoop`, `:85-132`). A SpeakAndText rule that
//!   holds is spoken - when speech was on as the application started (`MainV2.cs:1036`) - and
//!   raised as `WarningMessage`, which `MainV2` makes `cs.messageHigh`, the HUD's red message
//!   (`MainV2.cs:1037`; [`message_high`]). A Coloring rule raises `QuickPanelColoring` with its
//!   colour when it holds and with "NoColor" when it does not, which colours the quick view bound
//!   to its property and takes the colour off again (`MainV2.cs:4786-4811`, `quick.rs`).
//!   Something a rule throws ends that pass: the C#'s `catch` is round the whole loop.
//!
//! Where this is not the C#, each written at its site:
//!
//! * the properties offered ([`options`]) are the numeric ones this application holds, in the
//!   C#'s declaration order (`mp_vehicle::coverage::CURRENTSTATE`). `GetOptions` offers every
//!   property with a value, objects and strings among them, which `GetValue` then cannot convert
//!   to a double - its first is `parent`, a `MAVState`, so the C#'s "+" makes a rule that throws
//!   once given a comparison; here "+" makes one on the first number held;
//! * a rule on a property this application does not hold - from a `warnings.xml` Mission Planner
//!   wrote - never holds here, where the C# reads the value;
//! * speech is not ported (DELIVERABLES D15): what would be spoken is kept as a fact, as the
//!   Scripts tab keeps its `SpeakAsync` (`scripts_tab.rs`);
//! * the loop is not a task of its own but the window's frame (every 100 ms): a pass runs at the
//!   first frame 250 ms or more after the last, where the C# waits 250 ms after each pass. With no
//!   vehicle heard it reads a vehicle state of zeros, as the C#'s reads the `CurrentState` of a
//!   link not yet connected.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::path::PathBuf;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use mp_mission::dotnet::format_f64;
use mp_vehicle::VehicleState;
use mp_vehicle::coverage::CURRENTSTATE;

use crate::telemetry::TelemetryView;

/// `_text`'s initial value, and what `CB_type_CheckedChanged` puts back for a SpeakAndText rule.
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:147; Warnings/WarningControl.cs:321`
pub const DEFAULT_TEXT: &str = "WARNING: {name} is {value}";

/// The constructor's `RepeatTime`, in seconds.
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:49`
pub const REPEAT_TIME: i32 = 10;

/// `warningconfigfile`'s name in the user data directory.
/// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:14`
pub const FILE_NAME: &str = "warnings.xml";

/// `Task.Delay(250)`: the wait between passes.
/// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:130`
pub const PERIOD: Duration = Duration::from_millis(250);

/// The colour name that takes a quick view's colour off.
/// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:121; MainV2.cs:4800`
pub const NO_COLOR: &str = "NoColor";

/// `Console.WriteLine` when `LoadConfig` throws, before the file's path.
/// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:26`
pub const LOAD_FAILED: &str = "Failed to read Warning config file ";

/// `Conditional`: how a rule compares its property with its number.
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:15-24`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Conditional {
    /// `NONE`: never holds.
    #[default]
    None,
    /// `LT`.
    Lt,
    /// `LTEQ`.
    LtEq,
    /// `EQ`.
    Eq,
    /// `GT`.
    Gt,
    /// `GTEQ`.
    GtEq,
    /// `NEQ`.
    Neq,
}

impl Conditional {
    /// `Enum.GetNames(typeof(Conditional))`'s order, which `CMB_condition` lists.
    pub const ALL: [Self; 7] = [
        Self::None,
        Self::Lt,
        Self::LtEq,
        Self::Eq,
        Self::Gt,
        Self::GtEq,
        Self::Neq,
    ];

    /// Its name, as `ToString` and `XmlSerializer` write it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::None => "NONE",
            Self::Lt => "LT",
            Self::LtEq => "LTEQ",
            Self::Eq => "EQ",
            Self::Gt => "GT",
            Self::GtEq => "GTEQ",
            Self::Neq => "NEQ",
        }
    }

    /// The value of a name, exactly as written.
    #[must_use]
    pub fn parse(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|value| value.name() == name)
    }

    /// Whether `value` compares so with `warning`: `CheckValue`'s `switch`, the C#'s double
    /// comparisons - a NaN fails every one but `NEQ`.
    /// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:188-217`
    #[must_use]
    #[allow(clippy::float_cmp)] // `GetValue == Warning`, as the C# compares
    pub fn holds(self, value: f64, warning: f64) -> bool {
        match self {
            Self::None => false,
            Self::Lt => value < warning,
            Self::LtEq => value <= warning,
            Self::Eq => value == warning,
            Self::Gt => value > warning,
            Self::GtEq => value >= warning,
            Self::Neq => value != warning,
        }
    }
}

/// `WarningType`: whether a rule speaks or colours a quick view.
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:40-45`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WarningType {
    /// `SpeakAndText`.
    #[default]
    SpeakAndText,
    /// `Coloring`.
    Coloring,
}

impl WarningType {
    /// Its name, as `XmlSerializer` writes it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::SpeakAndText => "SpeakAndText",
            Self::Coloring => "Coloring",
        }
    }

    /// The value of a name, exactly as written.
    #[must_use]
    pub fn parse(name: &str) -> Option<Self> {
        [Self::SpeakAndText, Self::Coloring]
            .into_iter()
            .find(|value| value.name() == name)
    }
}

/// `WarningColors`, in `Enum.GetNames`' order as `CMB_color` lists them, each with the colour
/// `Color.FromName` makes of it - `System.Drawing`'s known colours - and none for "NoColor".
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:26-38; MainV2.cs:4809`
pub const COLORS: [(&str, Option<u32>); 10] = [
    (NO_COLOR, None),
    ("Red", Some(0xff_00_00)),
    ("OrangeRed", Some(0xff_45_00)),
    ("Maroon", Some(0x80_00_00)),
    ("Yellow", Some(0xff_ff_00)),
    ("Gold", Some(0xff_d7_00)),
    ("Goldenrod", Some(0xda_a5_20)),
    ("LawnGreen", Some(0x7c_fc_00)),
    ("Green", Some(0x00_80_00)),
    ("DarkGreen", Some(0x00_64_00)),
];

/// The colour of a `WarningColors` name; `None` for "NoColor" and for a name the list does not
/// hold.
#[must_use]
pub fn color_rgb(name: &str) -> Option<u32> {
    COLORS
        .iter()
        .find(|(held, _)| *held == name)
        .and_then(|(_, rgb)| *rgb)
}

/// The number's colour on a quick view coloured `back`: black when the mean of its red, green and
/// blue - the C#'s integer division - is over 128, else white.
/// `// C#: MainV2.cs:4810-4813`
#[must_use]
pub const fn readable_on(back: u32) -> u32 {
    let (r, g, b) = ((back >> 16) & 0xff, (back >> 8) & 0xff, back & 0xff);
    if (r + b + g) / 3 > 128 {
        0x00_00_00
    } else {
        0xff_ff_ff
    }
}

/// What the C# throws inside the loop, which its `catch` swallows - ending the pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Thrown(pub &'static str);

/// `GetValue` on a rule whose `Item` was never found: a child the manager added and no property
/// chosen for, given a comparison. `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:170-171`
pub const NO_ITEM: Thrown = Thrown("Value cannot be null. Parameter name: Item");
/// `lastrepeat.AddSeconds(RepeatTime)` on `DateTime.MinValue` with a negative `RepeatTime`.
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:183`
pub const BEFORE_MIN_DATE: Thrown =
    Thrown("The added or subtracted value results in an un-representable DateTime.");

/// One rule: `CustomWarning`'s fields and properties in its declaration order, which is the
/// order `XmlSerializer` writes them in, and the time it last held.
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:7-250`
#[derive(Debug, Clone, PartialEq)]
pub struct CustomWarning {
    /// `Child`: a rule that must hold as well.
    pub child: Option<Box<CustomWarning>>,
    /// `Name`: the property, empty for none.
    pub name: String,
    /// `Warning`: the number compared with.
    pub warning: f64,
    /// `type`.
    pub kind: WarningType,
    /// `color`: a `WarningColors` name; `null` until something sets it.
    pub color: Option<String>,
    /// `RepeatTime`, in seconds.
    pub repeat_time: i32,
    /// `ConditionType`.
    pub condition: Conditional,
    /// `Text`: what is said, `{warning}`, `{value}` and `{name}` replaced.
    pub text: String,
    /// `lastrepeat`: when it last held, `DateTime.MinValue` - never - to start.
    last_repeat: Option<Instant>,
}

impl Default for CustomWarning {
    /// `new CustomWarning()`: no property, 0, `NONE`, ten seconds, the default text.
    /// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:47-52, 106, 147`
    fn default() -> Self {
        Self {
            child: None,
            name: String::new(),
            warning: 0.0,
            kind: WarningType::SpeakAndText,
            color: None,
            repeat_time: REPEAT_TIME,
            condition: Conditional::None,
            text: DEFAULT_TEXT.to_owned(),
            last_repeat: None,
        }
    }
}

/// How a rule reads a property: its value in the user's units, or `None` for one this
/// application does not hold.
pub type Read<'a> = &'a dyn Fn(&str) -> Option<f64>;

impl CustomWarning {
    /// A new rule on a property: `new CustomWarning()` then `SetField(name)`.
    /// `// C#: Warnings/WarningsManager.cs:65-68`
    #[must_use]
    pub fn on(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            ..Self::default()
        }
    }

    /// When it last held, if it has.
    #[cfg(test)]
    #[must_use]
    pub const fn last_repeat(&self) -> Option<Instant> {
        self.last_repeat
    }

    /// `GetValue`: the property's value. Throws for a rule with no property; `None` for one this
    /// application does not hold (see the module's notes).
    /// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:164-175`
    fn get_value(&self, read: Read<'_>) -> Result<Option<f64>, Thrown> {
        if self.name.is_empty() {
            return Err(NO_ITEM);
        }
        Ok(read(&self.name))
    }

    /// `CheckValue()`: false within `RepeatTime` seconds of the last time it held; else whether
    /// the property compares so, the time kept when it does. The value is read only for a
    /// comparison, so `NONE` never reads it. Every caller takes the default `userepeattime`.
    /// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:181-225`
    ///
    /// # Errors
    ///
    /// What the C# throws: [`NO_ITEM`], or [`BEFORE_MIN_DATE`] for a negative repeat before it
    /// has ever held.
    pub fn check_value(&mut self, read: Read<'_>, now: Instant) -> Result<bool, Thrown> {
        match self.last_repeat {
            None if self.repeat_time < 0 => return Err(BEFORE_MIN_DATE),
            Some(last) => {
                let repeat = Duration::from_secs(u64::try_from(self.repeat_time).unwrap_or(0));
                if now < last + repeat {
                    return Ok(false);
                }
            }
            None => {}
        }
        let condition = match self.condition {
            Conditional::None => false,
            condition => self
                .get_value(read)?
                .is_some_and(|value| condition.holds(value, self.warning)),
        };
        if condition {
            self.last_repeat = Some(now);
        }
        Ok(condition)
    }

    /// `SayText()`: `Text` with `{warning}` and `{value}` as `ToString("0.##")` writes them and
    /// `{name}` the property's name, replaced in that order.
    /// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:153-159`
    ///
    /// # Errors
    ///
    /// [`NO_ITEM`] for a rule with no property, or one not held here.
    pub fn say_text(&self, read: Read<'_>) -> Result<String, Thrown> {
        let value = self.get_value(read)?.ok_or(NO_ITEM)?;
        Ok(self
            .text
            .replace("{warning}", &format_f64(self.warning, "0.##"))
            .replace("{value}", &format_f64(value, "0.##"))
            .replace("{name}", &self.name))
    }
}

/// `checkCond`: the rule, and its child when it has one and the rule held.
/// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:134-150`
///
/// # Errors
///
/// What `CheckValue` throws, the rule's or a child's.
pub fn check_cond(item: &mut CustomWarning, read: Read<'_>, now: Instant) -> Result<bool, Thrown> {
    let holds = item.check_value(read, now)?;
    match item.child.as_deref_mut() {
        Some(child) => Ok(holds && check_cond(child, read, now)?),
        None => Ok(holds),
    }
}

/// The C#'s public instance fields of `CurrentState` - `Type.GetProperties` does not return them,
/// so `GetOptions` does not offer them and `SetField` does not find them.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:102-119, 156-177`
pub const FIELDS: [&str; 31] = [
    "firmware",
    "hilch1",
    "hilch2",
    "hilch3",
    "hilch4",
    "hilch5",
    "hilch6",
    "hilch7",
    "hilch8",
    "lastautowp",
    "rcoverridech1",
    "rcoverridech10",
    "rcoverridech11",
    "rcoverridech12",
    "rcoverridech13",
    "rcoverridech14",
    "rcoverridech15",
    "rcoverridech16",
    "rcoverridech17",
    "rcoverridech18",
    "rcoverridech2",
    "rcoverridech3",
    "rcoverridech4",
    "rcoverridech5",
    "rcoverridech6",
    "rcoverridech7",
    "rcoverridech8",
    "rcoverridech9",
    "sensors_enabled",
    "sensors_health",
    "sensors_present",
];

/// Whether `SetField` finds a name among `CurrentState`'s properties: a member of the class that
/// is not static, not an event and not one of the [`FIELDS`]. A property this application does
/// not hold is one too.
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:228-249`
#[must_use]
pub fn is_property(name: &str) -> bool {
    !FIELDS.contains(&name)
        && CURRENTSTATE.iter().any(|field| {
            field.name == name
                && !field.ty.starts_with("static ")
                && !field.ty.starts_with("event ")
        })
}

/// `GetOptions()`: the properties a rule can be on - here the numeric ones this application holds,
/// in `CurrentState`'s declaration order (see the module's notes).
/// `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:57-90`
#[must_use]
pub fn options() -> &'static [&'static str] {
    static OPTIONS: OnceLock<Vec<&'static str>> = OnceLock::new();
    OPTIONS.get_or_init(|| {
        let zeros = VehicleState::default();
        let mut names: Vec<&'static str> = Vec::new();
        for field in CURRENTSTATE {
            if is_property(field.name)
                && crate::quick::value(field.name, &zeros).is_some()
                && !names.contains(&field.name)
            {
                names.push(field.name);
            }
        }
        names
    })
}

// ---------------------------------------------------------------------------------------------
// warnings.xml
// ---------------------------------------------------------------------------------------------

/// `XmlConvert.ToString(double)`: `INF`, `-INF` and `NaN`, else the round-trip text, in
/// exponent form - two digits at least - from 1E+15 and below 1E-05.
#[must_use]
pub fn xml_double(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "INF" } else { "-INF" }.to_owned();
    }
    let scientific = format!("{value:e}");
    let (mantissa, exponent) = scientific.split_once('e').unwrap_or((&scientific, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    if (-5..15).contains(&exponent) || value == 0.0 {
        format!("{value}")
    } else {
        let sign = if exponent < 0 { '-' } else { '+' };
        format!("{mantissa}E{sign}{:02}", exponent.unsigned_abs())
    }
}

/// `XmlConvert.ToDouble`: XML's white space trimmed, `INF`, `-INF` and `NaN` exactly, else a
/// number in the invariant culture.
#[must_use]
pub fn parse_xml_double(text: &str) -> Option<f64> {
    let text = text.trim_matches(|c| matches!(c, ' ' | '\t' | '\n' | '\r'));
    match text {
        "INF" => Some(f64::INFINITY),
        "-INF" => Some(f64::NEG_INFINITY),
        "NaN" => Some(f64::NAN),
        _ if !text.is_empty()
            && text
                .chars()
                .all(|c| c.is_ascii_digit() || matches!(c, '+' | '-' | '.' | 'e' | 'E')) =>
        {
            text.parse().ok()
        }
        _ => None,
    }
}

/// Text as an element's content: `&`, `<` and `>` escaped.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

/// One element, empty as `<Name />`.
fn element(out: &mut String, depth: usize, tag: &str, text: &str) {
    let pad = "  ".repeat(depth);
    if text.is_empty() {
        out.push_str(&format!("\n{pad}<{tag} />"));
    } else {
        out.push_str(&format!("\n{pad}<{tag}>{}</{tag}>", escape(text)));
    }
}

/// A rule as `XmlSerializer` writes it: its members in declaration order, `Child` and `color`
/// left out while null.
fn write_warning(out: &mut String, depth: usize, tag: &str, warning: &CustomWarning) {
    let pad = "  ".repeat(depth);
    out.push_str(&format!("\n{pad}<{tag}>"));
    if let Some(child) = warning.child.as_deref() {
        write_warning(out, depth + 1, "Child", child);
    }
    element(out, depth + 1, "Name", &warning.name);
    element(out, depth + 1, "Warning", &xml_double(warning.warning));
    element(out, depth + 1, "type", warning.kind.name());
    if let Some(color) = warning.color.as_deref() {
        element(out, depth + 1, "color", color);
    }
    element(
        out,
        depth + 1,
        "RepeatTime",
        &warning.repeat_time.to_string(),
    );
    element(out, depth + 1, "ConditionType", warning.condition.name());
    element(out, depth + 1, "Text", &warning.text);
    out.push_str(&format!("\n{pad}</{tag}>"));
}

/// `XmlSerializer(typeof(List<CustomWarning>))`'s document for the rules: indented two spaces,
/// the namespaces it declares, an empty list as an empty element.
/// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:45-59`
#[must_use]
pub fn to_xml(warnings: &[CustomWarning]) -> String {
    let mut out = String::from("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    let root = "<ArrayOfCustomWarning xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
                xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\"";
    out.push_str(root);
    if warnings.is_empty() {
        out.push_str(" />");
        return out;
    }
    out.push('>');
    for warning in warnings {
        write_warning(&mut out, 1, "CustomWarning", warning);
    }
    out.push_str("\n</ArrayOfCustomWarning>");
    out
}

/// One rule from its element, as `XmlSerializer` reads it: the constructor's values, then each
/// element in the document's order - an unknown one passed over - a name `SetField` cannot find,
/// an enum name that is not one, or a number that does not parse throwing.
fn read_warning(node: roxmltree::Node<'_, '_>) -> Result<CustomWarning, String> {
    let mut warning = CustomWarning::default();
    for child in node.children().filter(roxmltree::Node::is_element) {
        let text = child.text().unwrap_or("");
        match child.tag_name().name() {
            "Child" => {
                let nil = child
                    .attribute(("http://www.w3.org/2001/XMLSchema-instance", "nil"))
                    .is_some_and(|nil| nil.trim() == "true");
                warning.child = if nil {
                    None
                } else {
                    Some(Box::new(read_warning(child)?))
                };
            }
            // The `Name` setter's `SetField`: `MissingFieldException("No such name")`.
            // `// C#: ExtLibs/Utilities/Warnings/CustomWarning.cs:92-104, 248`
            "Name" => {
                if !text.is_empty() && !is_property(text) {
                    return Err(format!("No such name: {text}"));
                }
                text.clone_into(&mut warning.name);
            }
            "Warning" => {
                warning.warning =
                    parse_xml_double(text).ok_or_else(|| format!("Warning: {text}"))?;
            }
            "type" => {
                warning.kind = WarningType::parse(text).ok_or_else(|| format!("type: {text}"))?;
            }
            "color" => warning.color = Some(text.to_owned()),
            "RepeatTime" => {
                warning.repeat_time = text
                    .trim_matches(|c| matches!(c, ' ' | '\t' | '\n' | '\r'))
                    .parse()
                    .map_err(|_| format!("RepeatTime: {text}"))?;
            }
            "ConditionType" => {
                warning.condition =
                    Conditional::parse(text).ok_or_else(|| format!("ConditionType: {text}"))?;
            }
            "Text" => text.clone_into(&mut warning.text),
            _ => {}
        }
    }
    Ok(warning)
}

/// The rules from `warnings.xml`: `reader.Deserialize`, all of them or, for anything it throws
/// on, an error.
/// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:30-43`
///
/// # Errors
///
/// A document that does not parse, a root that is not `ArrayOfCustomWarning`, or a rule
/// `read_warning` throws on.
pub fn from_xml(text: &str) -> Result<Vec<CustomWarning>, String> {
    // A byte-order mark, which the C#'s reader reads past.
    let text = text.trim_start_matches('\u{feff}');
    let document = roxmltree::Document::parse(text).map_err(|error| error.to_string())?;
    let root = document.root_element();
    if root.tag_name().name() != "ArrayOfCustomWarning" {
        return Err(format!(
            "<{} xmlns=''> was not expected.",
            root.tag_name().name()
        ));
    }
    root.children()
        .filter(|node| node.is_element() && node.tag_name().name() == "CustomWarning")
        .map(read_warning)
        .collect()
}

// ---------------------------------------------------------------------------------------------
// The engine.
// ---------------------------------------------------------------------------------------------

/// What one pass raised, in the order the C# raises it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Raised {
    /// `WarningMessage`'s texts, each a `SayText`.
    pub messages: Vec<String>,
    /// What `_speech.SpeakAsync` was given: the same texts, when speech is on.
    pub spoken: Vec<String>,
    /// `QuickPanelColoring`'s arguments: the property and the colour name, "NoColor" for a
    /// Coloring rule that does not hold, `null` for one whose colour was never set.
    pub colourings: Vec<(String, Option<String>)>,
    /// What a rule threw, ending the pass.
    pub thrown: Option<Thrown>,
}

/// `WarningEngine`: the rules, the file, whether speech is on, the loop's clock, and what it has
/// raised, for the facts.
#[derive(Debug, Default)]
pub struct WarningEngine {
    /// `warnings`.
    pub warnings: Vec<CustomWarning>,
    /// `warningconfigfile`: `warnings.xml` in the user data directory, if there is one.
    pub file: Option<PathBuf>,
    /// `_speech != null`: `speechEnable` as the application started.
    pub speech: bool,
    /// When the next pass is due; now, at the start.
    next: Option<Instant>,
    /// Passes run.
    pub passes: usize,
    /// What `LoadConfig` threw, when it did.
    pub load_error: Option<String>,
    /// The last message raised, and how many.
    pub last_message: Option<String>,
    /// How many messages have been raised.
    pub messages: usize,
    /// The last thing that would have been spoken.
    pub last_spoken: Option<String>,
    /// The last `QuickPanelColoring`.
    pub last_colouring: Option<(String, Option<String>)>,
    /// Passes a rule's exception ended, and the last exception.
    pub thrown: usize,
    /// The last exception.
    pub last_thrown: Option<Thrown>,
    /// Saves made, and the rules in the file as the last one read it back.
    pub saved: usize,
    /// The rules the file written last holds, read back.
    pub saved_rules: Option<usize>,
}

impl WarningEngine {
    /// The engine as `MainV2` starts it: the static constructor's `LoadConfig` of
    /// `warnings.xml` in the user data directory, and speech from the user's "speechenable".
    /// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:14, 18-28; MainV2.cs:1005-1006, 1035-1036`
    #[must_use]
    pub fn start(settings: &crate::settings::Persisted) -> Self {
        let mut engine = Self {
            file: mp_settings::user_data_directory().map(|dir| dir.join(FILE_NAME)),
            speech: crate::raw_params::get_boolean(settings.get("speechenable")),
            ..Self::default()
        };
        if let Err(error) = engine.load_config() {
            eprintln!(
                "{LOAD_FAILED}{}",
                engine
                    .file
                    .as_ref()
                    .map_or_else(String::new, |file| file.display().to_string())
            );
            engine.load_error = Some(error);
        }
        engine
    }

    /// `LoadConfig`: nothing for a file that is not there; else every rule, or - when the document
    /// throws - none of them, the rules left as they were.
    /// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:30-43`
    ///
    /// # Errors
    ///
    /// The file unreadable, or what [`from_xml`] throws.
    pub fn load_config(&mut self) -> Result<(), String> {
        let Some(file) = self.file.as_ref() else {
            return Ok(());
        };
        if !file.exists() {
            return Ok(());
        }
        let text = std::fs::read_to_string(file).map_err(|error| error.to_string())?;
        self.warnings = from_xml(&text)?;
        Ok(())
    }

    /// `SaveConfig`: the rules written to the file, made where the user data directory is not
    /// there yet; then the file read back for the facts.
    /// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:45-59`
    ///
    /// # Errors
    ///
    /// No user data directory, or the file not written - `new StreamWriter` throwing.
    pub fn save_config(&mut self) -> Result<(), String> {
        let Some(file) = self.file.clone() else {
            return Err("no user data directory to save warnings.xml in".to_owned());
        };
        if let Some(dir) = file.parent() {
            std::fs::create_dir_all(dir).map_err(|error| error.to_string())?;
        }
        std::fs::write(&file, to_xml(&self.warnings)).map_err(|error| error.to_string())?;
        self.saved += 1;
        self.saved_rules = std::fs::read_to_string(&file)
            .ok()
            .and_then(|text| from_xml(&text).ok())
            .map(|rules| rules.len());
        Ok(())
    }

    /// One pass of `MainLoop`: each rule checked, a SpeakAndText one that holds spoken and raised,
    /// a Coloring one raising its colour or "NoColor"; what a rule throws ends the pass.
    /// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:88-128`
    pub fn pass(&mut self, read: Read<'_>, now: Instant) -> Raised {
        let mut raised = Raised::default();
        for item in &mut self.warnings {
            let holds = match check_cond(item, read, now) {
                Ok(holds) => holds,
                Err(thrown) => {
                    raised.thrown = Some(thrown);
                    break;
                }
            };
            match (holds, item.kind) {
                (true, WarningType::SpeakAndText) => match item.say_text(read) {
                    Ok(text) => {
                        if self.speech {
                            raised.spoken.push(text.clone());
                        }
                        raised.messages.push(text);
                    }
                    Err(thrown) => {
                        raised.thrown = Some(thrown);
                        break;
                    }
                },
                (true, WarningType::Coloring) => {
                    raised
                        .colourings
                        .push((item.name.clone(), item.color.clone()));
                }
                (false, WarningType::Coloring) => {
                    raised
                        .colourings
                        .push((item.name.clone(), Some(NO_COLOR.to_owned())));
                }
                (false, WarningType::SpeakAndText) => {}
            }
        }
        self.passes += 1;
        self.messages += raised.messages.len();
        if let Some(text) = raised.messages.last() {
            self.last_message = Some(text.clone());
        }
        if let Some(text) = raised.spoken.last() {
            self.last_spoken = Some(text.clone());
        }
        if let Some(colouring) = raised.colourings.last() {
            self.last_colouring = Some(colouring.clone());
        }
        if raised.thrown.is_some() {
            self.thrown += 1;
            self.last_thrown = raised.thrown;
        }
        raised
    }

    /// The loop's clock: a pass when one is due - at once at the start, then 250 ms after the
    /// last.
    /// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:85-132`
    pub fn tick(&mut self, read: Read<'_>, now: Instant) -> Option<Raised> {
        if self.next.is_some_and(|next| now < next) {
            return None;
        }
        self.next = Some(now + PERIOD);
        Some(self.pass(read, now))
    }
}

/// `cs.messageHigh = s`, the HUD's message: an empty text, or the text the HUD shows now, is not
/// raised again - the setter's "check against get" - so the ten seconds it shows for run from
/// when it was first raised.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:1269-1284; MainV2.cs:1037`
pub fn message_high(timing: &mut crate::hud::Timing, text: String, now: Instant) {
    if text.is_empty() {
        return;
    }
    if timing
        .message(None, now)
        .is_some_and(|(shown, _)| shown == text)
    {
        return;
    }
    timing.message(Some(text), now);
}

/// A rule in a line, for the facts: its property, comparison, number, kind, colour, repeat and
/// text, then its child's after " & ".
#[must_use]
pub fn describe(warning: &CustomWarning) -> String {
    let mut text = format!(
        "{} {} {} {} {} {} {}",
        warning.name,
        warning.condition.name(),
        format_f64(warning.warning, "0.##"),
        warning.kind.name(),
        warning.color.as_deref().unwrap_or("null"),
        warning.repeat_time,
        warning.text
    );
    if let Some(child) = warning.child.as_deref() {
        text.push_str(" & ");
        text.push_str(&describe(child));
    }
    text
}

/// Facts a UI test asserts on: the file and what loading it threw, the rules, the passes, the
/// last message, speech, colouring and exception, the saves - and the HUD's message, which a
/// SpeakAndText rule puts there.
pub fn record_facts(engine: &WarningEngine, hud_message: Option<&(String, u32)>) {
    use crate::facts::record;
    record(
        "warnings.file",
        engine
            .file
            .as_ref()
            .map_or_else(|| "none".to_owned(), |file| file.display().to_string()),
    );
    record(
        "warnings.load",
        engine.load_error.as_deref().unwrap_or("ok"),
    );
    record("warnings.rules", engine.warnings.len());
    for (index, warning) in engine.warnings.iter().enumerate() {
        record(format!("warnings.rule.{index}"), describe(warning));
    }
    record("warnings.passes", engine.passes);
    record("warnings.messages", engine.messages);
    record(
        "warnings.message",
        engine.last_message.as_deref().unwrap_or("none"),
    );
    record("warnings.speech", engine.speech);
    record(
        "warnings.spoken",
        engine.last_spoken.as_deref().unwrap_or("none"),
    );
    record(
        "warnings.colouring",
        engine.last_colouring.as_ref().map_or_else(
            || "none".to_owned(),
            |(name, color)| format!("{name} {}", color.as_deref().unwrap_or("null")),
        ),
    );
    record("warnings.thrown", engine.thrown);
    record("warnings.saved", engine.saved);
    record(
        "warnings.saved.rules",
        engine
            .saved_rules
            .map_or_else(|| "none".to_owned(), |rules| rules.to_string()),
    );
    record(
        "warnings.hud.message",
        hud_message.map_or("none", |(text, _)| text.as_str()),
    );
}

impl crate::MissionPlanner {
    /// Once a frame, before the HUD's inputs are made: the Warning Manager's boxes the keyboard
    /// has left read, then - when one is due - the engine's pass over the shown vehicle's state in
    /// the user's units (a state of zeros with none), its messages made the HUD's and its colours
    /// put on the quick views.
    /// `// C#: ExtLibs/Utilities/Warnings/WarningEngine.cs:85-132; MainV2.cs:1035-1038, 4786-4811`
    pub(crate) fn warnings_tick(&mut self, view: &TelemetryView, window: &gpui::Window) {
        let now = Instant::now();
        crate::config::warnings_manager::focus_left(self, window, now);
        let units = self.planner.units();
        let zeros = VehicleState::default();
        let state = view.state.as_deref().unwrap_or(&zeros);
        let read = |name: &str| crate::quick::display_value(name, state, &units);
        let Some(raised) = self.warnings.tick(&read, now) else {
            return;
        };
        for text in raised.messages {
            message_high(&mut self.hud_timing, text, now);
        }
        for (name, color) in &raised.colourings {
            self.fly_data.quick.warning_colour(name, color.as_deref());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_coverage::source::csharp;
    use std::collections::HashMap;
    use std::sync::Mutex;

    /// A reader over scripted values.
    fn values(pairs: &[(&str, f64)]) -> impl Fn(&str) -> Option<f64> + use<> {
        let map: HashMap<String, f64> = pairs
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect();
        move |name: &str| map.get(name).copied()
    }

    fn rule(name: &str, condition: Conditional, warning: f64) -> CustomWarning {
        CustomWarning {
            condition,
            warning,
            ..CustomWarning::on(name)
        }
    }

    /// The constructor's values, and every comparison as the C#'s `switch` makes it.
    #[test]
    fn a_rule_compares_as_check_value_does() {
        let fresh = CustomWarning::default();
        assert_eq!(fresh.repeat_time, 10);
        assert_eq!(fresh.warning, 0.0);
        assert_eq!(fresh.condition, Conditional::None);
        assert_eq!(fresh.kind, WarningType::SpeakAndText);
        assert_eq!(fresh.text, "WARNING: {name} is {value}");
        assert_eq!(fresh.color, None);
        assert!(fresh.child.is_none() && fresh.name.is_empty());
        let cases = [
            (Conditional::Lt, [true, false, false]),
            (Conditional::LtEq, [true, true, false]),
            (Conditional::Eq, [false, true, false]),
            (Conditional::Gt, [false, false, true]),
            (Conditional::GtEq, [false, true, true]),
            (Conditional::Neq, [true, false, true]),
            (Conditional::None, [false, false, false]),
        ];
        let now = Instant::now();
        for (condition, expected) in cases {
            for (value, holds) in [4.0, 5.0, 6.0].into_iter().zip(expected) {
                let mut warning = rule("satcount", condition, 5.0);
                let read = values(&[("satcount", value)]);
                assert_eq!(
                    warning.check_value(&read, now),
                    Ok(holds),
                    "{} {value}",
                    condition.name()
                );
                assert_eq!(warning.last_repeat().is_some(), holds);
            }
        }
        // NaN fails every comparison but NEQ.
        let read = values(&[("satcount", f64::NAN)]);
        assert_eq!(
            rule("satcount", Conditional::Neq, 5.0).check_value(&read, now),
            Ok(true)
        );
        assert_eq!(
            rule("satcount", Conditional::Eq, 5.0).check_value(&read, now),
            Ok(false)
        );
        // The names, as the C# declares them and XmlSerializer writes them.
        let names: Vec<&str> = Conditional::ALL.iter().map(|c| c.name()).collect();
        assert_eq!(names, ["NONE", "LT", "LTEQ", "EQ", "GT", "GTEQ", "NEQ"]);
        assert_eq!(Conditional::parse("GTEQ"), Some(Conditional::GtEq));
        assert_eq!(Conditional::parse("gt"), None);
    }

    /// A rule that held cannot hold again for `RepeatTime` seconds; one that did not hold keeps
    /// no time; a repeat of 0 holds every time; `NONE` never reads the value, so a rule with no
    /// property throws only once it has a comparison; a negative repeat throws before it has ever
    /// held.
    #[test]
    fn the_repeat_time_stops_a_rule_holding_again() {
        let start = Instant::now();
        let read = values(&[("satcount", 12.0)]);
        let mut warning = rule("satcount", Conditional::Gt, 5.0);
        assert_eq!(warning.check_value(&read, start), Ok(true));
        assert_eq!(
            warning.check_value(&read, start + Duration::from_millis(9_999)),
            Ok(false)
        );
        assert_eq!(
            warning.check_value(&read, start + Duration::from_secs(10)),
            Ok(true)
        );
        let low = values(&[("satcount", 3.0)]);
        let mut never = rule("satcount", Conditional::Gt, 5.0);
        assert_eq!(never.check_value(&low, start), Ok(false));
        assert!(never.last_repeat().is_none());
        let mut every = rule("satcount", Conditional::Gt, 5.0);
        every.repeat_time = 0;
        assert_eq!(every.check_value(&read, start), Ok(true));
        assert_eq!(every.check_value(&read, start), Ok(true));
        let mut unnamed = CustomWarning::default();
        assert_eq!(unnamed.check_value(&read, start), Ok(false));
        unnamed.condition = Conditional::Gt;
        assert_eq!(unnamed.check_value(&read, start), Err(NO_ITEM));
        let mut negative = rule("satcount", Conditional::Gt, 5.0);
        negative.repeat_time = -1;
        assert_eq!(negative.check_value(&read, start), Err(BEFORE_MIN_DATE));
        // A property this application does not hold never holds.
        let mut unheld = rule("parent", Conditional::Neq, 5.0);
        assert_eq!(unheld.check_value(&read, start), Ok(false));
    }

    /// `SayText`: the number and the value as "0.##" writes them, the name, in that order.
    #[test]
    fn say_text_fills_in_the_rule() {
        let mut warning = rule("alt", Conditional::Lt, 100.456);
        warning.text = "{name} {value} under {warning} ({name})".to_owned();
        let read = values(&[("alt", 12.0)]);
        assert_eq!(
            warning.say_text(&read),
            Ok("alt 12 under 100.46 (alt)".to_owned())
        );
        let read = values(&[("alt", 0.125)]);
        let default = rule("alt", Conditional::Lt, 5.0);
        assert_eq!(
            default.say_text(&read),
            Ok("WARNING: alt is 0.13".to_owned())
        );
        assert_eq!(CustomWarning::default().say_text(&read), Err(NO_ITEM));
    }

    /// `checkCond`: a child is checked only when its parent holds, and must hold too; the
    /// parent's repeat is taken even when the child fails.
    #[test]
    fn a_child_must_hold_as_well() {
        let start = Instant::now();
        let mut parent = rule("satcount", Conditional::Gt, 5.0);
        parent.child = Some(Box::new(rule("alt", Conditional::Lt, 100.0)));
        let both = values(&[("satcount", 12.0), ("alt", 50.0)]);
        assert_eq!(check_cond(&mut parent, &both, start), Ok(true));
        let mut parent2 = parent.clone();
        parent2.last_repeat = None;
        if let Some(child) = parent2.child.as_deref_mut() {
            child.last_repeat = None;
        }
        let high = values(&[("satcount", 12.0), ("alt", 150.0)]);
        assert_eq!(check_cond(&mut parent2, &high, start), Ok(false));
        assert!(parent2.last_repeat().is_some(), "the parent held");
        let low = values(&[("satcount", 3.0), ("alt", 50.0)]);
        let mut parent3 = rule("satcount", Conditional::Gt, 5.0);
        parent3.child = Some(Box::new(rule("alt", Conditional::Lt, 100.0)));
        assert_eq!(check_cond(&mut parent3, &low, start), Ok(false));
        assert!(
            parent3
                .child
                .as_ref()
                .is_some_and(|c| c.last_repeat().is_none()),
            "the child is not checked when the parent fails"
        );
        // A new child, with no property: NONE never holds, so neither does its parent.
        let mut parent4 = rule("satcount", Conditional::Gt, 5.0);
        parent4.child = Some(Box::new(CustomWarning::default()));
        assert_eq!(check_cond(&mut parent4, &both, start), Ok(false));
    }

    /// A pass: a SpeakAndText rule that holds is raised, and spoken with speech on; a Coloring
    /// rule raises its colour, or "NoColor" when it does not hold; a rule's exception ends the
    /// pass, the rules after it unchecked.
    #[test]
    fn a_pass_raises_messages_and_colours() {
        let start = Instant::now();
        let mut engine = WarningEngine::default();
        engine.warnings.push(rule("satcount", Conditional::Gt, 5.0));
        let mut colouring = rule("alt", Conditional::Lt, 100.0);
        colouring.kind = WarningType::Coloring;
        colouring.color = Some("Red".to_owned());
        colouring.repeat_time = 0;
        engine.warnings.push(colouring);
        let read = values(&[("satcount", 12.0), ("alt", 50.0)]);
        let raised = engine.pass(&read, start);
        assert_eq!(raised.messages, ["WARNING: satcount is 12"]);
        assert!(raised.spoken.is_empty(), "speech is off");
        assert_eq!(
            raised.colourings,
            [("alt".to_owned(), Some("Red".to_owned()))]
        );
        assert_eq!(raised.thrown, None);
        // 250 ms on: the message waits out its ten seconds, the colour goes when alt rises.
        let read = values(&[("satcount", 12.0), ("alt", 150.0)]);
        let raised = engine.pass(&read, start + PERIOD);
        assert!(raised.messages.is_empty());
        assert_eq!(
            raised.colourings,
            [("alt".to_owned(), Some("NoColor".to_owned()))]
        );
        // Speech on: the same text spoken.
        engine.speech = true;
        let raised = engine.pass(&read, start + Duration::from_secs(10));
        assert_eq!(raised.spoken, ["WARNING: satcount is 12"]);
        assert_eq!(
            engine.last_spoken.as_deref(),
            Some("WARNING: satcount is 12")
        );
        assert_eq!(engine.messages, 2);
        assert_eq!(engine.passes, 3);
        // A rule that throws first: the pass ends there.
        let thrown = CustomWarning {
            condition: Conditional::Gt,
            ..CustomWarning::default()
        };
        engine.warnings.insert(0, thrown);
        let raised = engine.pass(&read, start + Duration::from_secs(30));
        assert_eq!(raised.thrown, Some(NO_ITEM));
        assert!(raised.messages.is_empty() && raised.colourings.is_empty());
        assert_eq!(engine.thrown, 1);
    }

    /// The loop's clock: a pass at once, then none until 250 ms on.
    #[test]
    fn the_engine_passes_every_250_ms() {
        let start = Instant::now();
        let mut engine = WarningEngine::default();
        let read = values(&[]);
        assert!(engine.tick(&read, start).is_some());
        assert!(
            engine
                .tick(&read, start + Duration::from_millis(100))
                .is_none()
        );
        assert!(
            engine
                .tick(&read, start + Duration::from_millis(249))
                .is_none()
        );
        assert!(engine.tick(&read, start + PERIOD).is_some());
        assert_eq!(engine.passes, 2);
    }

    /// `messageHigh`'s setter: an empty text and the text already showing are not raised again,
    /// so the message shows ten seconds from when it was first raised; another text is.
    #[test]
    fn the_hud_message_is_raised_as_message_high_raises_it() {
        let start = Instant::now();
        let mut timing = crate::hud::Timing::default();
        message_high(&mut timing, String::new(), start);
        assert!(timing.message(None, start).is_none());
        message_high(&mut timing, "WARNING: satcount is 12".to_owned(), start);
        let later = start + Duration::from_secs(6);
        message_high(&mut timing, "WARNING: satcount is 12".to_owned(), later);
        // Ten seconds from the first: gone at 10 s, though raised again at 6 s.
        assert!(
            timing
                .message(None, start + Duration::from_secs(9))
                .is_some()
        );
        assert!(
            timing
                .message(None, start + Duration::from_secs(10))
                .is_none()
        );
        message_high(&mut timing, "WARNING: satcount is 13".to_owned(), later);
        assert_eq!(
            timing
                .message(None, later + Duration::from_secs(9))
                .map(|(text, _)| text),
            Some("WARNING: satcount is 13".to_owned())
        );
    }

    /// The colours are `System.Drawing`'s, and the number on each is black or white by the C#'s
    /// brightness test.
    #[test]
    fn colours_and_their_numbers_are_the_csharps() {
        let names: Vec<&str> = COLORS.iter().map(|(name, _)| *name).collect();
        assert_eq!(
            names,
            [
                "NoColor",
                "Red",
                "OrangeRed",
                "Maroon",
                "Yellow",
                "Gold",
                "Goldenrod",
                "LawnGreen",
                "Green",
                "DarkGreen"
            ]
        );
        let black: Vec<&str> = COLORS
            .iter()
            .filter(|(_, rgb)| rgb.is_some_and(|rgb| readable_on(rgb) == 0))
            .map(|(name, _)| *name)
            .collect();
        assert_eq!(black, ["Yellow", "Gold", "Goldenrod"]);
        assert_eq!(color_rgb("Red"), Some(0xff_00_00));
        assert_eq!(color_rgb("NoColor"), None);
        assert_eq!(color_rgb("Blue"), None);
        let Some(source) = csharp("ExtLibs/Utilities/Warnings/CustomWarning.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let start = source.find("enum WarningColors").expect("WarningColors");
        let body = &source[start..];
        let body = &body[body.find('{').expect("{") + 1..body.find('}').expect("}")];
        let declared: Vec<&str> = body
            .split(',')
            .map(|name| name.split('=').next().unwrap_or("").trim())
            .collect();
        assert_eq!(declared, names);
    }

    /// The document `XmlSerializer` writes, read back to the same rules.
    #[test]
    fn the_rules_are_written_and_read_back() {
        let mut first = rule("satcount", Conditional::Gt, 5.5);
        first.child = Some(Box::new(rule("alt", Conditional::Lt, 100.0)));
        first.text = "Sats & <fix>".to_owned();
        let mut second = rule("alt", Conditional::Lt, 100.0);
        second.kind = WarningType::Coloring;
        second.color = Some("Red".to_owned());
        second.repeat_time = 0;
        second.text = String::new();
        let rules = vec![first, second];
        let xml = to_xml(&rules);
        assert_eq!(
            xml,
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\
             <ArrayOfCustomWarning xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
             xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n  \
             <CustomWarning>\n    \
             <Child>\n      \
             <Name>alt</Name>\n      \
             <Warning>100</Warning>\n      \
             <type>SpeakAndText</type>\n      \
             <RepeatTime>10</RepeatTime>\n      \
             <ConditionType>LT</ConditionType>\n      \
             <Text>WARNING: {name} is {value}</Text>\n    \
             </Child>\n    \
             <Name>satcount</Name>\n    \
             <Warning>5.5</Warning>\n    \
             <type>SpeakAndText</type>\n    \
             <RepeatTime>10</RepeatTime>\n    \
             <ConditionType>GT</ConditionType>\n    \
             <Text>Sats &amp; &lt;fix&gt;</Text>\n  \
             </CustomWarning>\n  \
             <CustomWarning>\n    \
             <Name>alt</Name>\n    \
             <Warning>100</Warning>\n    \
             <type>Coloring</type>\n    \
             <color>Red</color>\n    \
             <RepeatTime>0</RepeatTime>\n    \
             <ConditionType>LT</ConditionType>\n    \
             <Text />\n  \
             </CustomWarning>\n\
             </ArrayOfCustomWarning>"
        );
        assert_eq!(from_xml(&xml), Ok(rules));
        assert_eq!(
            to_xml(&[]),
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<ArrayOfCustomWarning \
             xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
             xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" />"
        );
        assert_eq!(from_xml(&to_xml(&[])), Ok(Vec::new()));
    }

    /// A `warnings.xml` as Mission Planner on Windows writes it - CRLF lines, a property this
    /// application does not hold, a number in exponent form - with an element it does not know
    /// and a byte-order mark, which the C#'s reader reads past; one naming no property of
    /// `CurrentState`, or a bad enum or number, is not read at all.
    #[test]
    fn the_csharps_own_file_is_read() {
        let csharp_file = "\u{feff}<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n\
            <ArrayOfCustomWarning xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
            xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\r\n  <CustomWarning>\r\n    \
            <Name>battery_voltage</Name>\r\n    <Warning>10.5</Warning>\r\n    \
            <type>SpeakAndText</type>\r\n    <color>NoColor</color>\r\n    \
            <RepeatTime>30</RepeatTime>\r\n    <ConditionType>LTEQ</ConditionType>\r\n    \
            <Text>Battery {value} volts</Text>\r\n    <Unknown>1</Unknown>\r\n  \
            </CustomWarning>\r\n  <CustomWarning>\r\n    <Name>parent</Name>\r\n    \
            <Warning>1E+20</Warning>\r\n    <type>Coloring</type>\r\n    \
            <color>Gold</color>\r\n    <RepeatTime>0</RepeatTime>\r\n    \
            <ConditionType>NONE</ConditionType>\r\n    <Text />\r\n  </CustomWarning>\r\n\
            </ArrayOfCustomWarning>";
        let rules = from_xml(csharp_file).expect("read");
        let text = csharp_file;
        assert_eq!(rules.len(), 2);
        assert_eq!(rules[0].name, "battery_voltage");
        assert_eq!(rules[0].warning, 10.5);
        assert_eq!(rules[0].condition, Conditional::LtEq);
        assert_eq!(rules[0].repeat_time, 30);
        assert_eq!(rules[0].color.as_deref(), Some("NoColor"));
        assert_eq!(rules[0].text, "Battery {value} volts");
        assert_eq!(rules[1].name, "parent");
        assert_eq!(rules[1].warning, 1e20);
        assert_eq!(rules[1].kind, WarningType::Coloring);
        assert_eq!(rules[1].text, "");
        assert!(from_xml(&text.replace("battery_voltage", "nonsense")).is_err());
        assert!(from_xml(&text.replace("battery_voltage", "hilch1")).is_err());
        assert!(from_xml(&text.replace(">LTEQ<", ">lteq<")).is_err());
        assert!(from_xml(&text.replace(">10.5<", ">ten<")).is_err());
        assert!(from_xml("<ArrayOfPointLatLngAlt />").is_err());
    }

    /// `XmlConvert`'s numbers.
    #[test]
    fn numbers_are_written_as_xmlconvert_writes_them() {
        assert_eq!(xml_double(5.0), "5");
        assert_eq!(xml_double(-0.25), "-0.25");
        assert_eq!(xml_double(99_999.99), "99999.99");
        assert_eq!(xml_double(1e20), "1E+20");
        assert_eq!(xml_double(1.5e-7), "1.5E-07");
        assert_eq!(xml_double(0.0001), "0.0001");
        assert_eq!(xml_double(f64::INFINITY), "INF");
        assert_eq!(xml_double(f64::NEG_INFINITY), "-INF");
        assert_eq!(xml_double(f64::NAN), "NaN");
        assert_eq!(parse_xml_double(" 1E+20\r\n"), Some(1e20));
        assert_eq!(parse_xml_double("INF"), Some(f64::INFINITY));
        assert_eq!(parse_xml_double("inf"), None);
        assert_eq!(parse_xml_double(""), None);
    }

    /// The engine as it starts: `warnings.xml` from the user data directory, all or nothing; and
    /// Save writing it there, the directory made, read back as the next start reads it.
    #[test]
    fn the_file_is_loaded_at_start_and_saved_by_save() {
        static LOCK: Mutex<()> = Mutex::new(());
        let _guard = LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let dir = std::env::temp_dir().join(format!("mp-warnings-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let file = dir.join("data").join(FILE_NAME);
        let mut engine = WarningEngine {
            file: Some(file.clone()),
            ..WarningEngine::default()
        };
        assert_eq!(engine.load_config(), Ok(()), "no file: nothing");
        assert!(engine.warnings.is_empty());
        engine.warnings.push(rule("satcount", Conditional::Gt, 5.0));
        assert_eq!(engine.save_config(), Ok(()));
        assert_eq!(engine.saved, 1);
        assert_eq!(engine.saved_rules, Some(1));
        let mut again = WarningEngine {
            file: Some(file.clone()),
            ..WarningEngine::default()
        };
        assert_eq!(again.load_config(), Ok(()));
        assert_eq!(again.warnings, engine.warnings);
        // A file that throws leaves the rules as they were.
        std::fs::write(&file, "<ArrayOfCustomWarning><CustomWarning><type>x</type>").expect("w");
        assert!(again.load_config().is_err());
        assert_eq!(again.warnings.len(), 1);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The options are the held numeric properties in declaration order: `satcount` and `alt`
    /// among them, no field, nothing static, nothing not held.
    #[test]
    fn the_options_are_the_held_properties_in_declaration_order() {
        let options = options();
        assert!(options.len() > 100, "{}", options.len());
        // The first, which "+" makes a rule on: the C#'s first is `parent`, a `MAVState`.
        assert_eq!(options[0], "customfield0");
        for name in [
            "satcount",
            "alt",
            "groundspeed",
            "battery_voltage",
            "yaw",
            "DistToHome",
        ] {
            assert!(options.contains(&name), "{name}");
        }
        for field in FIELDS {
            assert!(!options.contains(&field), "{field}");
        }
        let order: Vec<usize> = options
            .iter()
            .map(|name| {
                CURRENTSTATE
                    .iter()
                    .position(|field| field.name == *name)
                    .expect("a row")
            })
            .collect();
        assert!(order.windows(2).all(|pair| pair[0] < pair[1]));
        assert!(is_property("parent"));
        assert!(!is_property("multiplierdist"));
        assert!(!is_property("lastautowp"));
        assert!(!is_property("nonsense"));
    }

    /// The fields left out are the C#'s public instance fields, read from `CurrentState.cs`.
    #[test]
    fn the_fields_are_the_ones_currentstate_declares() {
        let Some(source) = csharp("ExtLibs/ArduPilot/CurrentState.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let mut fields: Vec<&str> = Vec::new();
        for line in source.lines() {
            let code = line.split("//").next().unwrap_or("").trim_end();
            let Some(rest) = code.strip_prefix("        public ") else {
                continue;
            };
            if rest.starts_with("static ")
                || rest.starts_with("event ")
                || rest.starts_with("override ")
                || rest.starts_with("const ")
                || rest.contains("=>")
                || rest.contains('{')
                || !rest.ends_with(';')
            {
                continue;
            }
            // Up to an initialiser; a `(` before one is a method's.
            let declaration = rest.split('=').next().unwrap_or(rest).trim_end_matches(';');
            if declaration.contains('(') {
                continue;
            }
            if let Some(name) = declaration.split_whitespace().last() {
                fields.push(name);
            }
        }
        fields.sort_unstable();
        let mut ours = FIELDS.to_vec();
        ours.sort_unstable();
        assert_eq!(fields, ours);
    }

    /// The C#'s words and numbers are these.
    #[test]
    fn the_csharps_words_and_numbers_are_these() {
        let (Some(warning), Some(engine), Some(main)) = (
            csharp("ExtLibs/Utilities/Warnings/CustomWarning.cs"),
            csharp("ExtLibs/Utilities/Warnings/WarningEngine.cs"),
            csharp("MainV2.cs"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        assert!(warning.contains(&format!("string _text = \"{DEFAULT_TEXT}\";")));
        assert!(warning.contains(&format!("RepeatTime = {REPEAT_TIME};")));
        assert!(warning.contains("Warning.ToString(\"0.##\")"));
        assert!(engine.contains(&format!(
            "Settings.GetUserDataDirectory() + \"{FILE_NAME}\""
        )));
        assert!(engine.contains("await Task.Delay(250)"));
        assert!(engine.contains(&format!(
            "QuickPanelColoring?.Invoke(item.Name, \"{NO_COLOR}\")"
        )));
        assert!(engine.contains(&format!("Console.WriteLine(\"{LOAD_FAILED}\"")));
        assert!(main.contains(
            "Warnings.WarningEngine.WarningMessage += (sender, s) => { MainV2.comPort.MAV.cs.messageHigh = s; };"
        ));
        assert!(main.contains("((qv.BackColor.R + qv.BackColor.B + qv.BackColor.G) / 3) > 128"));
    }

    /// The tick is called in `render` before the HUD's inputs are made, so a message raised this
    /// frame shows this frame.
    #[test]
    fn the_engine_runs_before_the_hud_inputs() {
        let main = include_str!("main.rs");
        let render = main.find("fn render(&mut self").expect("render");
        let tick = main
            .find("self.warnings_tick(&view, window);")
            .expect("the tick");
        let hud = main
            .find("self.hud = self.hud_inputs(&view);")
            .expect("the HUD's inputs");
        assert!(render < tick && tick < hud);
        assert_eq!(main.matches("self.warnings_tick(").count(), 1);
    }
}
