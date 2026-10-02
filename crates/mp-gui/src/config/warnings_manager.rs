//! The Warning Manager: `Warnings/WarningsManager.cs` and its rows, `Warnings/WarningControl.cs`
//! - the window SETUP's Advanced page's "Warning Manager" opens (`ConfigAdvanced.cs:22-25`: `new
//! WarningsManager().Show()`), over the warning engine's rules (`warnings.rs`).
//!
//! What it shows, as `WarningsManager.Designer.cs` lays it out in an 872 x 238 form: "+" and Save
//! along the top, the column headings - Item type, Field, Comparison, Trigger, Color, Message,
//! Repeat Interval - and under them `panel1`, a row for each rule. A row (`WarningControl`, 850 x
//! 27, its controls placed by its `InitializeComponent`, `WarningControl.cs:82-217`) holds
//! "QuickPanel Coloring", the property, the comparison, the number (two decimal places, within
//! 99999 either way), the colour, the message, the repeat interval (0 to 100 seconds), and "+"
//! and "-". A rule's child has its row under it, five pixels further in, its message box hidden.
//!
//! What it does:
//!
//! * opening it builds the rows (`reload`, `addwarningcontrol`, `WarningsManager.cs:16-56`). A row
//!   is built by `WarningControl`'s constructor and `updateDisplay` (`WarningControl.cs:15-59`),
//!   whose setting of the boxes goes through their change handlers: a Coloring rule's message is
//!   emptied and its repeat set to 0 as its row is built (`CB_type_CheckedChanged`, `:301-315`);
//! * "+" adds a rule on the first property offered and builds the rows again (`BUT_Add_Click`,
//!   `WarningsManager.cs:63-76`); Save writes `warnings.xml` (`BUT_save_Click`, `:78-81`);
//! * each of a row's controls changes its rule as it changes (`WarningControl.cs:219-255`):
//!   "QuickPanel Coloring" ticked makes a Coloring rule, its message emptied and disabled, its
//!   repeat 0 and disabled, its colour enabled, its "+" disabled; unticked, a SpeakAndText rule
//!   with the default message and a repeat of 10 (`CB_type_CheckedChanged`, `:301-331`);
//! * a row's "+" gives its rule a new child, in place of any it had (`but_addchild_Click`,
//!   `:257-263`); its "-" removes its rule - a rule with its children, a child by joining its
//!   parent to its own child (`but_remove_Click`, `removewarning`, `:265-299`); both build the
//!   rows again.
//!
//! Closing the form leaves the rules as they are, and the engine checking them: only Save writes
//! the file.
//!
//! Where this is not the C#, each written at its site:
//!
//! * the form is drawn over SETUP, modal, where the C#'s is a free form (`Show`); a second click
//!   replaces it with a fresh one, built from the rules again, as the MAVLink Inspector is
//!   replaced (`config/mavlink_inspector.rs`);
//! * the form does not resize, so `panel1` - anchored to the form's four sides, with no scroll
//!   bars - scrolls instead, so a row past its 175 pixels can be reached;
//! * the combo boxes are lists to choose from, their text not typed into: typing into the C#'s
//!   changes nothing, since only their `SelectedIndexChanged` is handled. A box whose rule holds
//!   a text its list has not - a new child's empty property, a property from a `warnings.xml`
//!   that this application does not offer - shows nothing, where the C#'s shows that text;
//! * a number from a hand-edited `warnings.xml` outside its box's bounds is shown held to them -
//!   and a repeat so held is written to its rule, as the box's `ValueChanged` would write it -
//!   where the C#'s `Value` setter throws and the form does not open;
//! * Save's failure goes on the status line (the owner's ruling of 2026-09-25), where the C#'s
//!   `StreamWriter` throws into the unhandled-exception box;
//! * the number boxes are drawn and typed into as `servo_output::Number` is, a plain
//!   `NumericUpDown`'s part of it: a typed value past the bounds is held to them, and nothing is
//!   asked or written to a vehicle.
//!
//! Not drawn: the Designer's `warningControl1` and `label4`, which the first `reload` removes
//! (`panel1.Controls.Clear()`) before the form shows.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::time::Instant;

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};

use super::failsafe::CheckState;
use super::optional::{at, button, label};
use super::servo_output::{
    Check, Combo, Designer, Number, NumberHandlers, check_box, combo_box, dropdown, number_box,
};
use crate::MissionPlanner;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;
use crate::warnings::{
    COLORS, Conditional, CustomWarning, DEFAULT_TEXT, NO_COLOR, WarningType, options,
};

// ---------------------------------------------------------------------------------------------
// The words and places of the two `InitializeComponent`s.
// ---------------------------------------------------------------------------------------------

/// The form's caption, `this.Text`. `// C#: Warnings/WarningsManager.Designer.cs:173`
pub const FORM_TEXT: &str = "Warning Manager";

/// `ClientSize`. `// C#: Warnings/WarningsManager.Designer.cs:161`
pub const CLIENT: (f32, f32) = (872.0, 238.0);

/// `BUT_Add`'s `Text`, `Location` and `Size`. `// C#: Warnings/WarningsManager.Designer.cs:78-82`
pub const ADD: &str = "+";
/// Where `BUT_Add` is.
pub const ADD_AT: (f32, f32, f32, f32) = (13.0, 13.0, 75.0, 23.0);
/// `BUT_save`'s `Text`. `// C#: Warnings/WarningsManager.Designer.cs:88-92`
pub const SAVE: &str = "Save";
/// Where `BUT_save` is.
pub const SAVE_AT: (f32, f32, f32, f32) = (94.0, 13.0, 75.0, 23.0);

/// The column headings: each label's name, `Text` and `Location.X`, left to right; all at
/// [`HEADER_Y`]. `// C#: Warnings/WarningsManager.Designer.cs:96-157`
pub const HEADERS: [(&str, &str, f32); 7] = [
    ("label7", "Item type", 3.0),
    ("label1", "Field", 129.0),
    ("label2", "Comparison", 258.0),
    ("label3", "Trigger", 314.0),
    ("label8", "Color", 398.0),
    ("label6", "Message", 507.0),
    ("label5", "Repeat Interval", 748.0),
];
/// The headings' `Location.Y`.
pub const HEADER_Y: f32 = 39.0;

/// `panel1`'s `Location` and `Size`. `// C#: Warnings/WarningsManager.Designer.cs:54-56`
pub const PANEL_AT: (f32, f32, f32, f32) = (4.0, 53.0, 866.0, 175.0);

/// A row's `Size`: `WarningControl`'s. `// C#: Warnings/WarningControl.cs:211`
pub const ROW_SIZE: (f32, f32) = (850.0, 27.0);
/// A rule's row's `X` in the panel, and how much further in each child's is.
/// `// C#: Warnings/WarningsManager.cs:26, 52`
pub const ROW_X: f32 = 5.0;
/// How much further in a child's row is than its parent's.
pub const CHILD_INDENT: f32 = 5.0;

/// `CB_type`'s `Text` and `Location`. `// C#: Warnings/WarningControl.cs:181-185`
pub const TYPE_TEXT: &str = "QuickPanel Coloring";
/// Where `CB_type` is.
pub const TYPE_AT: (f32, f32) = (3.0, 5.0);
/// `CMB_Source`. `// C#: Warnings/WarningControl.cs:100-102`
pub const SOURCE_AT: (f32, f32, f32, f32) = (131.0, 3.0, 116.0, 21.0);
/// `CMB_condition`. `// C#: Warnings/WarningControl.cs:110-112`
pub const CONDITION_AT: (f32, f32, f32, f32) = (253.0, 3.0, 54.0, 21.0);
/// `NUM_warning`. `// C#: Warnings/WarningControl.cs:120-132`
pub const WARNING_AT: (f32, f32, f32, f32) = (313.0, 4.0, 65.0, 20.0);
/// `CMB_color`. `// C#: Warnings/WarningControl.cs:192-194`
pub const COLOR_AT: (f32, f32, f32, f32) = (384.0, 3.0, 104.0, 21.0);
/// `TXT_warningtext`. `// C#: Warnings/WarningControl.cs:151-153`
pub const TEXT_AT: (f32, f32, f32, f32) = (494.0, 3.0, 236.0, 20.0);
/// `NUM_repeattime`. `// C#: Warnings/WarningControl.cs:138-140`
pub const REPEAT_AT: (f32, f32, f32, f32) = (745.0, 4.0, 39.0, 20.0);
/// `but_addchild`'s `Text`, and where it is. `// C#: Warnings/WarningControl.cs:160-164`
pub const ADD_CHILD: &str = "+";
/// Where `but_addchild` is.
pub const ADD_CHILD_AT: (f32, f32, f32, f32) = (791.0, 4.0, 25.0, 20.0);
/// `but_remove`'s `Text`. `// C#: Warnings/WarningControl.cs:170-174`
pub const REMOVE: &str = "-";
/// Where `but_remove` is.
pub const REMOVE_AT: (f32, f32, f32, f32) = (822.0, 4.0, 25.0, 20.0);

/// `NUM_warning`: two places, within 99999 either way, 0. `// C#: Warnings/WarningControl.cs:119-130`
pub const WARNING_BOX: Designer = Designer {
    minimum: -99_999.0,
    maximum: 99_999.0,
    value: 0.0,
    decimals: 2,
};

/// `NUM_repeattime`: `NumericUpDown`'s 0 to 100, whole seconds, 10.
/// `// C#: Warnings/WarningControl.cs:138-146`
pub const REPEAT_BOX: Designer = Designer {
    minimum: 0.0,
    maximum: 100.0,
    value: 10.0,
    decimals: 0,
};

/// The caption of Save's failure on the status line.
pub const SAVE_FAILED: &str = "Failed to save warnings.xml: ";

// ---------------------------------------------------------------------------------------------
// A row.
// ---------------------------------------------------------------------------------------------

/// One of a row's three combo boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum List {
    /// `CMB_Source`: the property.
    Source,
    /// `CMB_condition`.
    Condition,
    /// `CMB_color`.
    Color,
}

impl List {
    /// The word in its ids.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Source => "source",
            Self::Condition => "condition",
            Self::Color => "color",
        }
    }
}

/// One of a row's two number boxes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Num {
    /// `NUM_warning`.
    Warning,
    /// `NUM_repeattime`.
    Repeat,
}

impl Num {
    /// The word in its ids.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::Warning => "warning",
            Self::Repeat => "repeat",
        }
    }
}

/// A list's rows as a combo box holds them: the index and the text.
fn list_of(names: impl Iterator<Item = &'static str>) -> Combo {
    Combo {
        options: (0_i64..).zip(names.map(str::to_owned)).collect(),
        // `DataSource` set: the first row selected, before `custwarning` is, so no handler acts.
        selected: Some(0),
        enabled: true,
        ..Combo::default()
    }
}

/// The index of the row whose text is `text`, ignoring case as the `Text` setter's
/// `FindStringIgnoreCase` does.
fn find(combo: &Combo, text: &str) -> Option<i64> {
    combo
        .options
        .iter()
        .find(|(_, held)| held.eq_ignore_ascii_case(text))
        .map(|(key, _)| *key)
}

/// A `NumericUpDown` as its Designer leaves it, enabled.
fn number(designer: Designer) -> Number {
    let mut number = Number::new(designer);
    number.enabled = true;
    number
}

/// The rule a row is for: the `depth`-th child down from rule `top`.
fn rule_at(rules: &mut [CustomWarning], top: usize, depth: usize) -> Option<&mut CustomWarning> {
    let mut rule = rules.get_mut(top)?;
    for _ in 0..depth {
        rule = rule.child.as_deref_mut()?;
    }
    Some(rule)
}

/// One `WarningControl`: the rule it is for, by its place, and its controls as they show.
/// `// C#: Warnings/WarningControl.cs:6-217`
#[derive(Debug)]
pub struct Control {
    /// The rule's place in the engine's list.
    pub top: usize,
    /// How many children down from that rule this one is: 0 for the rule itself.
    pub depth: usize,
    /// `CB_type.Checked`: a Coloring rule.
    pub coloring: bool,
    /// `CMB_Source`.
    pub source: Combo,
    /// `CMB_condition`.
    pub condition: Combo,
    /// `CMB_color`.
    pub color: Combo,
    /// `NUM_warning`.
    pub warning: Number,
    /// `TXT_warningtext`.
    pub text: TextField,
    /// `TXT_warningtext.Enabled`.
    pub text_enabled: bool,
    /// `TXT_warningtext.Visible`: hidden on a child's row.
    pub text_visible: bool,
    /// `NUM_repeattime`.
    pub repeat: Number,
    /// `but_addchild.Enabled`.
    pub add_child: bool,
}

impl Control {
    /// `new WarningControl(item)`: the Designer's controls, the lists bound, then
    /// `updateDisplay`. `item.SetField(item.Name)` finds nothing new: a rule's name is one the
    /// load or a list accepted.
    /// `// C#: Warnings/WarningControl.cs:15-31`
    fn new(top: usize, depth: usize, rule: &mut CustomWarning) -> Self {
        let mut text = TextField::new("");
        text.set(DEFAULT_TEXT);
        let mut control = Self {
            top,
            depth,
            coloring: false,
            source: list_of(options().iter().copied()),
            condition: list_of(Conditional::ALL.iter().map(|c| c.name())),
            color: list_of(COLORS.iter().map(|(name, _)| *name)),
            warning: number(WARNING_BOX),
            text,
            text_enabled: true,
            // `if (hideforchild) wrnctl.TXT_warningtext.Visible = false;`
            // `// C#: Warnings/WarningsManager.cs:43-44`
            text_visible: depth == 0,
            repeat: number(REPEAT_BOX),
            add_child: true,
        };
        control.update_display(rule);
        control
    }

    /// `updateDisplay`: each control set from the rule, through its change handler where the
    /// value changes.
    /// `// C#: Warnings/WarningControl.cs:33-59`
    fn update_display(&mut self, rule: &mut CustomWarning) {
        // `CMB_condition.Text`, then `CMB_Source.Text`: their handlers set what is already set.
        if let Some(key) = find(&self.condition, rule.condition.name()) {
            self.condition.select(key);
        }
        match find(&self.source, &rule.name) {
            Some(key) => {
                self.source.select(key);
            }
            // No row has the text: the box shows it - empty, for a new child - and nothing is
            // selected.
            None => self.source.selected = None,
        }
        // `NUM_warning.Value = (decimal)custwarning.Warning`: held to the box's bounds, where the
        // C#'s setter throws for a number past them (see the module's notes).
        self.warning.set_value(rule.warning);
        if rule.kind == WarningType::SpeakAndText {
            // `CB_type.Checked = false` changes nothing; the handler is called by hand.
            self.type_changed(rule);
            self.set_repeat(f64::from(rule.repeat_time), rule);
            let text = rule.text.clone();
            self.set_text(&text, rule);
            self.set_color_text(Some(NO_COLOR), rule);
        } else {
            self.set_checked(true, rule);
            self.type_changed(rule);
            let color = rule.color.clone();
            self.set_color_text(color.as_deref(), rule);
        }
    }

    /// `CB_type.Checked`'s setter: `CheckedChanged` when it changes.
    fn set_checked(&mut self, checked: bool, rule: &mut CustomWarning) {
        if self.coloring != checked {
            self.coloring = checked;
            self.type_changed(rule);
        }
    }

    /// `CB_type_CheckedChanged`: a Coloring rule's message emptied and disabled, its repeat 0 and
    /// disabled, its colour enabled and its "+" disabled; a SpeakAndText rule's the other way,
    /// with the default message and a repeat of 10.
    /// `// C#: Warnings/WarningControl.cs:301-331`
    fn type_changed(&mut self, rule: &mut CustomWarning) {
        if self.coloring {
            self.add_child = false;
            self.set_text("", rule);
            self.text_enabled = false;
            self.set_repeat(0.0, rule);
            self.repeat.enabled = false;
            self.color.enabled = true;
            rule.kind = WarningType::Coloring;
        } else {
            self.add_child = true;
            self.set_text(DEFAULT_TEXT, rule);
            self.text_enabled = true;
            self.set_repeat(10.0, rule);
            self.repeat.enabled = true;
            self.color.enabled = false;
            rule.kind = WarningType::SpeakAndText;
        }
    }

    /// `TXT_warningtext.Text`'s setter: `TextChanged`, `custwarning.Text = ...`, when it changes.
    /// `// C#: Warnings/WarningControl.cs:245-249`
    fn set_text(&mut self, text: &str, rule: &mut CustomWarning) {
        if self.text.value() != text {
            self.text.set(text);
            text.clone_into(&mut rule.text);
        }
    }

    /// `NUM_repeattime.Value`'s setter, held to the box's bounds: `ValueChanged`,
    /// `custwarning.RepeatTime = (int)Value`, when it changes.
    /// `// C#: Warnings/WarningControl.cs:251-255`
    #[allow(clippy::float_cmp)] // `Value`'s setter compares decimals exactly
    fn set_repeat(&mut self, value: f64, rule: &mut CustomWarning) {
        let before = self.repeat.value();
        self.repeat.set_value(value);
        if self.repeat.value() != before {
            rule.repeat_time = whole(self.repeat.value());
        }
    }

    /// `CMB_color.Text`'s setter: a colour the list has selected - `cmbColor_SelectedIndexChanged`,
    /// `custwarning.color = CMB_color.Text`, when the selection changes; `null` selecting nothing,
    /// which changes it, the text then empty.
    /// `// C#: Warnings/WarningControl.cs:232-237`
    fn set_color_text(&mut self, text: Option<&str>, rule: &mut CustomWarning) {
        match text {
            None => {
                if self.color.selected.take().is_some() {
                    rule.color = Some(String::new());
                }
            }
            Some(text) => {
                if let Some(key) = find(&self.color, text)
                    && self.color.select(key)
                {
                    rule.color = Some(self.color.text().to_owned());
                }
            }
        }
    }

    /// One of its combo boxes.
    #[must_use]
    pub const fn list(&self, which: List) -> &Combo {
        match which {
            List::Source => &self.source,
            List::Condition => &self.condition,
            List::Color => &self.color,
        }
    }

    const fn list_mut(&mut self, which: List) -> &mut Combo {
        match which {
            List::Source => &mut self.source,
            List::Condition => &mut self.condition,
            List::Color => &mut self.color,
        }
    }

    /// One of its number boxes.
    #[must_use]
    pub const fn number(&self, which: Num) -> &Number {
        match which {
            Num::Warning => &self.warning,
            Num::Repeat => &self.repeat,
        }
    }

    const fn number_mut(&mut self, which: Num) -> &mut Number {
        match which {
            Num::Warning => &mut self.warning,
            Num::Repeat => &mut self.repeat,
        }
    }
}

/// `(int)` of a box's value: truncated, as a `decimal` converts.
#[allow(clippy::cast_possible_truncation)] // within the box's 0 to 100
fn whole(value: f64) -> i32 {
    value as i32
}

// ---------------------------------------------------------------------------------------------
// The form.
// ---------------------------------------------------------------------------------------------

/// The form: its rows, the list dropped down, and the box being typed into.
#[derive(Debug, Default)]
pub struct Manager {
    /// `panel1.Controls`: a row per rule and per child, in order.
    pub controls: Vec<Control>,
    /// The combo box whose list is dropped down: its row and which.
    pub list: Option<(usize, List)>,
    /// The number box being typed into.
    pub number: Option<(usize, Num)>,
    /// The row whose message is being typed into.
    pub typing: Option<usize>,
    /// How many times the rows have been built.
    pub reloads: usize,
}

impl Manager {
    /// `new WarningsManager()`: `InitializeComponent`, then `reload`.
    /// `// C#: Warnings/WarningsManager.cs:9-14`
    #[must_use]
    pub fn new(rules: &mut [CustomWarning]) -> Self {
        let mut manager = Self::default();
        manager.reload(rules);
        manager
    }

    /// `reload`: the rows built again, each rule's and then its children's, each child five
    /// pixels further in with its message hidden.
    /// `// C#: Warnings/WarningsManager.cs:16-56`
    pub fn reload(&mut self, rules: &mut [CustomWarning]) {
        self.controls.clear();
        self.list = None;
        self.number = None;
        self.typing = None;
        for (top, rule) in rules.iter_mut().enumerate() {
            let mut depth = 0;
            let mut next = Some(rule);
            while let Some(rule) = next {
                self.controls.push(Control::new(top, depth, rule));
                depth += 1;
                next = rule.child.as_deref_mut();
            }
        }
        self.reloads += 1;
    }

    /// The rule row `index` is for.
    fn rule<'a>(
        &self,
        index: usize,
        rules: &'a mut [CustomWarning],
    ) -> Option<&'a mut CustomWarning> {
        let control = self.controls.get(index)?;
        rule_at(rules, control.top, control.depth)
    }

    /// The keyboard leaving the box being typed into, for a control clicked: a number's text read
    /// into its value - `ValueChanged` writing the rule when it changes - and the message's box
    /// let go.
    pub fn leave(&mut self, rules: &mut [CustomWarning], now: Instant) {
        self.typing = None;
        let Some((index, which)) = self.number.take() else {
            return;
        };
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        let before = control.number(which).value();
        let _ = control.number_mut(which).commit(now);
        self.number_changed(index, which, before, rules);
    }

    /// `NUM_warning_ValueChanged` and `NUM_repeattime_ValueChanged`: the rule's number set from
    /// the box's, when it changed.
    /// `// C#: Warnings/WarningControl.cs:239-243, 251-255`
    #[allow(clippy::float_cmp)] // `Value`'s setter compares decimals exactly
    fn number_changed(&self, index: usize, which: Num, before: f64, rules: &mut [CustomWarning]) {
        let Some(control) = self.controls.get(index) else {
            return;
        };
        let after = control.number(which).value();
        if after == before {
            return;
        }
        let Some(rule) = rule_at(rules, control.top, control.depth) else {
            return;
        };
        match which {
            Num::Warning => rule.warning = after,
            Num::Repeat => rule.repeat_time = whole(after),
        }
    }

    /// `BUT_Add_Click`: a rule on the first property offered, added, and the rows built again.
    /// `// C#: Warnings/WarningsManager.cs:63-76`
    pub fn add(&mut self, rules: &mut Vec<CustomWarning>, now: Instant) {
        self.leave(rules, now);
        if let Some(first) = options().first() {
            rules.push(CustomWarning::on(first));
        }
        self.reload(rules);
    }

    /// `CB_type` clicked: `Checked` turned over, and `CB_type_CheckedChanged`.
    /// `// C#: Warnings/WarningControl.cs:301-331`
    pub fn toggle_type(&mut self, index: usize, rules: &mut [CustomWarning], now: Instant) {
        self.leave(rules, now);
        let Some(control) = self.controls.get(index) else {
            return;
        };
        let (top, depth, checked) = (control.top, control.depth, !control.coloring);
        let Some(rule) = rule_at(rules, top, depth) else {
            return;
        };
        if let Some(control) = self.controls.get_mut(index) {
            control.set_checked(checked, rule);
        }
    }

    /// A combo box clicked: its list dropped down, scrolled to the selected row - or, when it is
    /// already down, put away.
    pub fn open_list(
        &mut self,
        index: usize,
        which: List,
        rules: &mut [CustomWarning],
        now: Instant,
    ) {
        self.leave(rules, now);
        if self.list == Some((index, which)) {
            self.list = None;
            return;
        }
        if let Some(control) = self.controls.get_mut(index) {
            control.list_mut(which).open_list();
            self.list = Some((index, which));
        }
    }

    /// The wheel over the list dropped down.
    pub fn scroll_list(&mut self, lines: i32) {
        if let Some((index, which)) = self.list
            && let Some(control) = self.controls.get_mut(index)
        {
            control.list_mut(which).scroll_list(lines);
        }
    }

    /// A row of a list chosen: the list put away and, when the selection changes, its handler -
    /// `CMB_Source_SelectedIndexChanged`'s `SetField`, `CMB_condition_SelectedIndexChanged`'s
    /// `Enum.Parse`, `cmbColor_SelectedIndexChanged`'s colour name.
    /// `// C#: Warnings/WarningControl.cs:219-237`
    pub fn choose(&mut self, index: usize, which: List, key: i64, rules: &mut [CustomWarning]) {
        self.list = None;
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        let (top, depth) = (control.top, control.depth);
        let list = control.list_mut(which);
        if !list.select(key) {
            return;
        }
        let text = list.text().to_owned();
        let Some(rule) = rule_at(rules, top, depth) else {
            return;
        };
        match which {
            List::Source => rule.name = text,
            List::Condition => {
                if let Some(condition) = Conditional::parse(&text) {
                    rule.condition = condition;
                }
            }
            List::Color => rule.color = Some(text),
        }
    }

    /// A number box clicked into: the keyboard its.
    pub fn begin_number(
        &mut self,
        index: usize,
        which: Num,
        rules: &mut [CustomWarning],
        now: Instant,
    ) {
        if self.number == Some((index, which)) {
            return;
        }
        self.leave(rules, now);
        if self
            .controls
            .get(index)
            .is_some_and(|control| control.number(which).enabled)
        {
            self.number = Some((index, which));
        }
    }

    /// A key in the number box being typed into: typing, an arrow stepping, Enter reading the
    /// text - the last two through `ValueChanged`. Whether it was taken.
    pub fn number_key(
        &mut self,
        event: &KeyDownEvent,
        rules: &mut [CustomWarning],
        now: Instant,
    ) -> bool {
        let Some((index, which)) = self.number else {
            return false;
        };
        let Some(control) = self.controls.get_mut(index) else {
            return false;
        };
        let before = control.number(which).value();
        let (handled, _) = control.number_mut(which).key(event, now);
        self.number_changed(index, which, before, rules);
        handled
    }

    /// A number box's arrow: the text read first, then one step, held to the bounds - through
    /// `ValueChanged`.
    pub fn step_number(
        &mut self,
        index: usize,
        which: Num,
        up: bool,
        rules: &mut [CustomWarning],
        now: Instant,
    ) {
        if self.number != Some((index, which)) {
            self.leave(rules, now);
        }
        let Some(control) = self.controls.get_mut(index) else {
            return;
        };
        let before = control.number(which).value();
        let _ = control.number_mut(which).step(up, now);
        self.number_changed(index, which, before, rules);
    }

    /// The message box clicked into, when it is enabled.
    pub fn begin_text(&mut self, index: usize, rules: &mut [CustomWarning], now: Instant) {
        if self.typing == Some(index) {
            return;
        }
        self.leave(rules, now);
        if self
            .controls
            .get(index)
            .is_some_and(|control| control.text_enabled && control.text_visible)
        {
            self.typing = Some(index);
        }
    }

    /// A key in the message box: `TXT_warningtext_TextChanged`, `custwarning.Text = ...`, as
    /// the text changes. Whether it was taken.
    /// `// C#: Warnings/WarningControl.cs:245-249`
    pub fn text_key(&mut self, event: &KeyDownEvent, rules: &mut [CustomWarning]) -> bool {
        let Some(index) = self.typing else {
            return false;
        };
        let Some(control) = self.controls.get_mut(index) else {
            return false;
        };
        match control.text.key(event) {
            KeyOutcome::Changed => {
                let text = control.text.value().to_owned();
                if let Some(rule) = self.rule(index, rules) {
                    rule.text = text;
                }
                true
            }
            KeyOutcome::Submitted | KeyOutcome::Cancelled => true,
            KeyOutcome::Ignored => false,
        }
    }

    /// `but_addchild_Click`: a new child for the row's rule, in place of any it had, and the rows
    /// built again.
    /// `// C#: Warnings/WarningControl.cs:257-263`
    pub fn add_child(&mut self, index: usize, rules: &mut [CustomWarning], now: Instant) {
        self.leave(rules, now);
        if !self
            .controls
            .get(index)
            .is_some_and(|control| control.add_child)
        {
            return;
        }
        if let Some(rule) = self.rule(index, rules) {
            rule.child = Some(Box::new(CustomWarning::default()));
        }
        self.reload(rules);
    }

    /// `but_remove_Click`: a rule taken out of the list with its children; a child taken out of
    /// its chain, its parent given its own child (`removewarning`); the rows built again.
    /// `// C#: Warnings/WarningControl.cs:265-299`
    pub fn remove(&mut self, index: usize, rules: &mut Vec<CustomWarning>, now: Instant) {
        self.leave(rules, now);
        let Some(control) = self.controls.get(index) else {
            return;
        };
        let (top, depth) = (control.top, control.depth);
        if depth == 0 {
            if top < rules.len() {
                rules.remove(top);
            }
        } else if let Some(parent) = rule_at(rules, top, depth - 1) {
            parent.child = parent.child.take().and_then(|removed| removed.child);
        }
        self.reload(rules);
    }
}

/// The window and how often it has been opened, held with the Advanced page.
#[derive(Debug, Default)]
pub struct ManagerWindow {
    /// The form, while it is open.
    pub window: Option<Manager>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl ManagerWindow {
    /// `new WarningsManager().Show()`: a fresh form over the rules.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:22-25`
    pub fn show(&mut self, rules: &mut [CustomWarning]) {
        self.opened += 1;
        self.window = Some(Manager::new(rules));
    }

    /// The form's close box. The rules stay as they are, unsaved unless Save was pressed.
    pub fn close(&mut self) {
        self.window = None;
    }
}

/// Facts a UI test asserts on: whether the form is open, its rows as they show, the list dropped
/// down and the box typed into.
pub fn record_facts(holder: &ManagerWindow) {
    use crate::facts::record;
    record("config.warnings.window", holder.window.is_some());
    record("config.warnings.opened", holder.opened);
    let Some(manager) = holder.window.as_ref() else {
        return;
    };
    record("config.warnings.rows", manager.controls.len());
    record("config.warnings.reloads", manager.reloads);
    record(
        "config.warnings.list",
        manager.list.map_or_else(
            || "none".to_owned(),
            |(index, which)| format!("warnings-{index}-{}", which.key()),
        ),
    );
    record(
        "config.warnings.list.top",
        manager
            .list
            .and_then(|(index, which)| manager.controls.get(index).map(|c| c.list(which).top_index))
            .unwrap_or(0),
    );
    record(
        "config.warnings.editing",
        manager.number.map_or_else(
            || "none".to_owned(),
            |(index, which)| format!("warnings-{index}-{}", which.key()),
        ),
    );
    record(
        "config.warnings.typing",
        manager.typing.map_or_else(
            || "none".to_owned(),
            |index| format!("warnings-{index}-text"),
        ),
    );
    for (index, control) in manager.controls.iter().enumerate() {
        let key = |what: &str| format!("config.warnings.row.{index}.{what}");
        record(key("depth"), control.depth);
        record(key("coloring"), control.coloring);
        record(key("field"), control.source.text());
        record(key("condition"), control.condition.text());
        record(key("trigger"), control.warning.shown());
        record(key("color"), control.color.text());
        record(key("color.enabled"), control.color.enabled);
        record(key("text"), control.text.value());
        record(key("text.enabled"), control.text_enabled);
        record(key("text.visible"), control.text_visible);
        record(key("repeat"), control.repeat.shown());
        record(key("repeat.enabled"), control.repeat.enabled);
        record(key("addchild"), control.add_child);
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// The form and the rules it edits, as the drawing reaches them.
fn access(this: &mut MissionPlanner) -> Option<(&mut Manager, &mut Vec<CustomWarning>)> {
    let manager = this.extra.warnings_manager.window.as_mut()?;
    Some((manager, &mut this.warnings.warnings))
}

/// Once a frame: a number box or the message box the keyboard has left let go, the number's
/// text read into its rule as `ValueChanged` reads it.
pub(crate) fn focus_left(this: &mut MissionPlanner, window: &Window, now: Instant) {
    let number = this.extra_focus.warnings_number.is_focused(window);
    let text = this.extra_focus.warnings_text.is_focused(window);
    let Some((manager, rules)) = access(this) else {
        return;
    };
    if manager.number.is_some() && !number {
        let typing = manager.typing;
        manager.leave(rules, now);
        manager.typing = typing;
    }
    if manager.typing.is_some() && !text {
        manager.typing = None;
    }
}

/// Save: `WarningEngine.SaveConfig()`, after the box being typed into is read; a failure on the
/// status line (the owner's ruling of 2026-09-25).
/// `// C#: Warnings/WarningsManager.cs:78-81`
fn save(this: &mut MissionPlanner) {
    if let Some((manager, rules)) = access(this) {
        manager.leave(rules, Instant::now());
    }
    if let Err(error) = this.warnings.save_config() {
        this.file_status = Some(format!("{SAVE_FAILED}{error}"));
    }
}

/// A row's "+" or "-": a button with an id of its own.
fn row_button(
    id: String,
    text: &'static str,
    (x, y, width, height): (f32, f32, f32, f32),
    enabled: bool,
    on_click: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .child(text);
    let base = if enabled {
        base.bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                window.blur(cx);
                on_click(this);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL)).text_color(rgb(theme::DIM))
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// The handles the row's boxes type through.
struct Handles<'a> {
    number: &'a FocusHandle,
    text: &'a FocusHandle,
}

/// One row at its place in the panel: its controls where `InitializeComponent` puts them, and
/// its list when dropped down.
/// `// C#: Warnings/WarningControl.cs:82-217`
fn row(
    manager: &Manager,
    index: usize,
    control: &Control,
    handles: &Handles<'_>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    #[allow(clippy::cast_precision_loss)] // a handful of rows
    let (x, y) = (
        ROW_X + CHILD_INDENT * control.depth as f32,
        ROW_SIZE.1 * index as f32,
    );
    let mut check = Check::default();
    check.enabled = true;
    check.state = if control.coloring {
        CheckState::Checked
    } else {
        CheckState::Unchecked
    };
    let mut row = crate::probe::measured(format!("warnings-{index}"), div())
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(ROW_SIZE.0))
        .h(px(ROW_SIZE.1))
        .child(check_box(
            format!("warnings-{index}-type"),
            &check,
            TYPE_TEXT,
            TYPE_AT,
            move |this| {
                if let Some((manager, rules)) = access(this) {
                    manager.toggle_type(index, rules, Instant::now());
                }
            },
            cx,
        ));
    for (which, place) in [
        (List::Source, SOURCE_AT),
        (List::Condition, CONDITION_AT),
        (List::Color, COLOR_AT),
    ] {
        row = row.child(combo_box(
            format!("warnings-{index}-{}", which.key()),
            control.list(which),
            place,
            move |this| {
                if let Some((manager, rules)) = access(this) {
                    manager.open_list(index, which, rules, Instant::now());
                }
            },
            cx,
        ));
    }
    for (which, place) in [(Num::Warning, WARNING_AT), (Num::Repeat, REPEAT_AT)] {
        row = row.child(number_box(
            format!("warnings-{index}-{}", which.key()),
            control.number(which),
            manager.number == Some((index, which)),
            handles.number,
            place,
            NumberHandlers {
                begin: move |this: &mut MissionPlanner| {
                    if let Some((manager, rules)) = access(this) {
                        manager.begin_number(index, which, rules, Instant::now());
                    }
                },
                key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                    access(this).is_some_and(|(manager, rules)| {
                        manager.number_key(event, rules, Instant::now())
                    })
                },
                step: move |this: &mut MissionPlanner, up: bool| {
                    if let Some((manager, rules)) = access(this) {
                        manager.step_number(index, which, up, rules, Instant::now());
                    }
                },
            },
            window,
            cx,
        ));
    }
    if control.text_visible {
        row = row.child(text_box(manager, index, control, handles.text, window, cx));
    }
    row = row
        .child(row_button(
            format!("warnings-{index}-addchild"),
            ADD_CHILD,
            ADD_CHILD_AT,
            control.add_child,
            move |this| {
                if let Some((manager, rules)) = access(this) {
                    manager.add_child(index, rules, Instant::now());
                }
            },
            cx,
        ))
        .child(row_button(
            format!("warnings-{index}-remove"),
            REMOVE,
            REMOVE_AT,
            true,
            move |this| {
                if let Some((manager, rules)) = access(this) {
                    manager.remove(index, rules, Instant::now());
                }
            },
            cx,
        ));
    if let Some((open, which)) = manager.list
        && open == index
    {
        let (lx, ly, lw, lh) = match which {
            List::Source => SOURCE_AT,
            List::Condition => CONDITION_AT,
            List::Color => COLOR_AT,
        };
        row = row.child(dropdown(
            &format!("warnings-{index}-{}", which.key()),
            control.list(which),
            (lx, ly + lh, lw),
            move |this, key| {
                if let Some((manager, rules)) = access(this) {
                    manager.choose(index, which, key, rules);
                }
            },
            |this, lines| {
                if let Some((manager, _)) = access(this) {
                    manager.scroll_list(lines);
                }
            },
            cx,
        ));
    }
    row.into_any_element()
}

/// `TXT_warningtext`: typed into while it has the keyboard, else its text, a click taking the
/// keyboard; dimmed and inert while disabled.
fn text_box(
    manager: &Manager,
    index: usize,
    control: &Control,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (x, y, width, height) = TEXT_AT;
    if manager.typing == Some(index) && control.text_enabled {
        return at(x, y, width, height)
            .child(crate::textfield::text_area(
                "warnings-text",
                &control.text,
                handle,
                handle.is_focused(window),
                px(width),
                px(height),
                cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                    if access(this).is_some_and(|(manager, rules)| manager.text_key(event, rules)) {
                        cx.notify();
                    }
                }),
            ))
            .into_any_element();
    }
    let id = format!("warnings-{index}-text");
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .size_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .whitespace_nowrap()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .child(control.text.value().to_owned());
    let base = if control.text_enabled {
        let handle = handle.clone();
        base.bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_text()
            .on_click(cx.listener(move |this, _event, window, cx| {
                if let Some((manager, rules)) = access(this) {
                    manager.begin_text(index, rules, Instant::now());
                }
                handle.focus(window, cx);
                cx.notify();
            }))
    } else {
        base.bg(rgb(theme::PANEL)).text_color(rgb(theme::DIM))
    };
    at(x, y, width, height).child(base).into_any_element()
}

/// The form over the window: its caption with a close box, "+", Save, the headings and the
/// panel of rows, at their places.
/// `// C#: Warnings/WarningsManager.Designer.cs:30-179`
fn form(
    manager: &Manager,
    handles: &Handles<'_>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let mut client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(button(
            "warnings-add",
            ADD,
            ADD_AT,
            true,
            |this, _window, _cx| {
                if let Some((manager, rules)) = access(this) {
                    manager.add(rules, Instant::now());
                }
            },
            cx,
        ))
        .child(button(
            "warnings-save",
            SAVE,
            SAVE_AT,
            true,
            |this, _window, _cx| save(this),
            cx,
        ));
    for (_, text, x) in HEADERS {
        client = client.child(label(x, HEADER_Y, text, true));
    }
    #[allow(clippy::cast_precision_loss)] // a handful of rows
    let height = ROW_SIZE.1 * manager.controls.len() as f32;
    let mut rows = div().relative().w_full().h(px(height));
    for (index, control) in manager.controls.iter().enumerate() {
        rows = rows.child(row(manager, index, control, handles, window, cx));
    }
    let (px_, py_, pw, ph) = PANEL_AT;
    // Scrolled, where the C#'s form is resized to show more rows (see the module's notes).
    client = client.child(
        crate::probe::measured("warnings-panel", at(px_, py_, pw, ph))
            .id("warnings-panel")
            .overflow_y_scroll()
            .child(rows),
    );
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "warnings-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.warnings_manager.close();
                cx.notify();
            }),
        ));
    let form = crate::probe::measured("warnings", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("warnings-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(form);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(over),
    )
    .with_priority(1)
    .into_any_element()
}

/// The form, while it is open.
pub fn overlay(
    holder: &ManagerWindow,
    number: &FocusHandle,
    text: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let manager = holder.window.as_ref()?;
    Some(form(manager, &Handles { number, text }, window, cx))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_coverage::source::csharp;
    use crate::warnings::REPEAT_TIME;

    fn now() -> Instant {
        Instant::now()
    }

    /// A key, as gpui delivers one: `ctrl-` a chord, a single character typed.
    fn key(name: &str) -> KeyDownEvent {
        let control = name.starts_with("ctrl-");
        let name = name.trim_start_matches("ctrl-");
        let typed = (!control && name.chars().count() == 1).then(|| name.to_owned());
        KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers {
                    control,
                    ..gpui::Modifiers::default()
                },
                key: name.to_owned(),
                key_char: typed,
            },
            is_held: false,
            prefer_character_input: false,
        }
    }

    fn index_of(options: &[&str], name: &str) -> i64 {
        i64::try_from(options.iter().position(|n| *n == name).expect(name)).expect("small")
    }

    /// Opening the form builds a row per rule and per child, children further in with their
    /// message hidden; building a Coloring rule's row empties its message and makes its repeat
    /// 0, while a SpeakAndText rule's keeps its own.
    #[test]
    fn opening_builds_a_row_per_rule_and_child() {
        let mut speak = CustomWarning::on("satcount");
        speak.condition = Conditional::Gt;
        speak.warning = 5.0;
        speak.repeat_time = 30;
        speak.text = "Sats {value}".to_owned();
        speak.child = Some(Box::new(CustomWarning::on("alt")));
        let mut colour = CustomWarning::on("alt");
        colour.kind = WarningType::Coloring;
        colour.color = Some("Gold".to_owned());
        colour.repeat_time = 10;
        colour.text = "kept?".to_owned();
        let mut rules = vec![speak, colour];
        let mut holder = ManagerWindow::default();
        holder.show(&mut rules);
        let manager = holder.window.as_ref().expect("open");
        assert_eq!(holder.opened, 1);
        assert_eq!(manager.controls.len(), 3);
        let depths: Vec<usize> = manager.controls.iter().map(|c| c.depth).collect();
        assert_eq!(depths, [0, 1, 0]);
        let first = &manager.controls[0];
        assert_eq!(first.source.text(), "satcount");
        assert_eq!(first.condition.text(), "GT");
        assert_eq!(first.warning.shown(), "5.00");
        assert_eq!(first.repeat.shown(), "30");
        assert_eq!(first.text.value(), "Sats {value}");
        assert!(!first.coloring && first.add_child && first.text_enabled && first.text_visible);
        assert!(!first.color.enabled);
        assert_eq!(first.color.text(), "NoColor");
        let child = &manager.controls[1];
        assert!(!child.text_visible, "a child's message is hidden");
        assert_eq!(child.source.text(), "alt");
        let third = &manager.controls[2];
        assert!(third.coloring && !third.add_child && !third.text_enabled);
        assert!(!third.repeat.enabled && third.color.enabled);
        assert_eq!(third.color.text(), "Gold");
        // The rules as the C#'s handlers leave them: the Coloring one's message and repeat gone.
        assert_eq!(rules[0].text, "Sats {value}");
        assert_eq!(rules[0].repeat_time, 30);
        assert_eq!(rules[0].color, None, "NoColor was already selected");
        assert_eq!(rules[1].text, "");
        assert_eq!(rules[1].repeat_time, 0);
        assert_eq!(rules[1].color.as_deref(), Some("Gold"));
        // A Coloring rule with no colour: nothing selected, the colour then empty.
        let mut uncoloured = CustomWarning::on("alt");
        uncoloured.kind = WarningType::Coloring;
        let mut rules = vec![uncoloured];
        let manager = Manager::new(&mut rules);
        assert_eq!(manager.controls[0].color.text(), "");
        assert_eq!(rules[0].color.as_deref(), Some(""));
    }

    /// "+": a rule on the first property offered, its row built; Save writes the rules (the
    /// engine's, `warnings.rs`).
    #[test]
    fn add_makes_a_rule_on_the_first_property() {
        let mut rules = Vec::new();
        let mut manager = Manager::new(&mut rules);
        manager.add(&mut rules, now());
        assert_eq!(rules.len(), 1);
        assert_eq!(rules[0].name, options()[0]);
        assert_eq!(rules[0].condition, Conditional::None);
        assert_eq!(rules[0].repeat_time, REPEAT_TIME);
        assert_eq!(manager.controls.len(), 1);
        assert_eq!(manager.controls[0].source.text(), options()[0]);
        assert_eq!(manager.reloads, 2);
        manager.add(&mut rules, now());
        assert_eq!(rules.len(), 2);
        assert_eq!(manager.controls.len(), 2);
    }

    /// The row's controls change the rule: the property, the comparison, the number typed and
    /// stepped, the colour, the message as it is typed, the repeat.
    #[test]
    fn a_rows_controls_change_its_rule() {
        let mut rules = vec![CustomWarning::on(options()[0])];
        let mut manager = Manager::new(&mut rules);
        let at = now();
        manager.open_list(0, List::Source, &mut rules, at);
        assert_eq!(manager.list, Some((0, List::Source)));
        manager.choose(0, List::Source, index_of(options(), "satcount"), &mut rules);
        assert_eq!(manager.list, None);
        assert_eq!(rules[0].name, "satcount");
        manager.open_list(0, List::Condition, &mut rules, at);
        manager.choose(0, List::Condition, 4, &mut rules);
        assert_eq!(rules[0].condition, Conditional::Gt);
        // The number: typed, then Enter.
        manager.begin_number(0, Num::Warning, &mut rules, at);
        assert!(manager.number_key(&key("ctrl-u"), &mut rules, at));
        for digit in ["5", ".", "2", "5"] {
            manager.number_key(&key(digit), &mut rules, at);
        }
        assert_eq!(
            rules[0].warning, 0.0,
            "not read until it is left or entered"
        );
        manager.number_key(&key("enter"), &mut rules, at);
        assert_eq!(rules[0].warning, 5.25);
        // Past the bounds: held to them.
        manager.number_key(&key("ctrl-u"), &mut rules, at);
        for digit in ["2", "0", "0", "0", "0", "0"] {
            manager.number_key(&key(digit), &mut rules, at);
        }
        manager.leave(&mut rules, at);
        assert_eq!(rules[0].warning, 99_999.0);
        assert_eq!(manager.controls[0].warning.shown(), "99999.00");
        manager.step_number(0, Num::Warning, false, &mut rules, at);
        assert_eq!(rules[0].warning, 99_998.0);
        // The repeat's arrows.
        manager.step_number(0, Num::Repeat, true, &mut rules, at);
        assert_eq!(rules[0].repeat_time, 11);
        // The message, as it is typed.
        manager.begin_text(0, &mut rules, at);
        assert_eq!(manager.typing, Some(0));
        manager.text_key(&key("!"), &mut rules);
        assert_eq!(rules[0].text, "WARNING: {name} is {value}!");
        // The colour list, while the rule speaks: disabled, but its handler would set it.
        manager.toggle_type(0, &mut rules, at);
        assert_eq!(manager.typing, None, "the click took the keyboard");
        manager.open_list(0, List::Color, &mut rules, at);
        manager.choose(0, List::Color, 1, &mut rules);
        assert_eq!(rules[0].color.as_deref(), Some("Red"));
        // The wheel over a list.
        manager.open_list(0, List::Source, &mut rules, at);
        manager.scroll_list(1_000);
        let source = &manager.controls[0].source;
        assert_eq!(source.top_index, source.max_top_index());
        manager.open_list(0, List::Source, &mut rules, at);
        assert_eq!(manager.list, None, "a second click puts it away");
    }

    /// "QuickPanel Coloring": ticked, a Coloring rule with its message emptied, repeat 0, both
    /// disabled, its colour enabled and its "+" disabled; unticked, the default message and a
    /// repeat of 10 again.
    #[test]
    fn quickpanel_coloring_turns_a_rule_over() {
        let mut speak = CustomWarning::on("alt");
        speak.text = "Mine".to_owned();
        speak.repeat_time = 42;
        let mut rules = vec![speak];
        let mut manager = Manager::new(&mut rules);
        manager.toggle_type(0, &mut rules, now());
        let control = &manager.controls[0];
        assert!(control.coloring && !control.text_enabled && !control.repeat.enabled);
        assert!(control.color.enabled && !control.add_child);
        assert_eq!(rules[0].kind, WarningType::Coloring);
        assert_eq!(rules[0].text, "");
        assert_eq!(rules[0].repeat_time, 0);
        manager.begin_text(0, &mut rules, now());
        assert_eq!(manager.typing, None, "disabled");
        manager.add_child(0, &mut rules, now());
        assert!(rules[0].child.is_none(), "its + is disabled");
        manager.toggle_type(0, &mut rules, now());
        assert_eq!(rules[0].kind, WarningType::SpeakAndText);
        assert_eq!(rules[0].text, DEFAULT_TEXT, "its own message is gone");
        assert_eq!(rules[0].repeat_time, 10);
    }

    /// A row's "+" gives its rule a new child, in place of the one it had; "-" on a child joins
    /// its parent to its own child, and on a rule takes it with its children.
    #[test]
    fn children_are_added_and_spliced_out() {
        let mut rules = vec![CustomWarning::on("alt"), CustomWarning::on("satcount")];
        let mut manager = Manager::new(&mut rules);
        manager.add_child(0, &mut rules, now());
        assert_eq!(manager.controls.len(), 3);
        // The child's row: its "+" makes a grandchild.
        manager.add_child(1, &mut rules, now());
        assert_eq!(manager.controls.len(), 4);
        let depths: Vec<usize> = manager.controls.iter().map(|c| c.depth).collect();
        assert_eq!(depths, [0, 1, 2, 0]);
        // Name the grandchild, then take its parent out: the grandchild moves up.
        manager.choose(2, List::Source, index_of(options(), "satcount"), &mut rules);
        manager.remove(1, &mut rules, now());
        assert_eq!(manager.controls.len(), 3);
        let child = rules[0].child.as_deref().expect("spliced");
        assert_eq!(child.name, "satcount");
        assert!(child.child.is_none());
        // "+" on the rule again replaces its child.
        manager.add_child(0, &mut rules, now());
        assert_eq!(rules[0].child.as_deref().map(|c| c.name.as_str()), Some(""));
        // "-" on a rule: it and its children go.
        manager.remove(0, &mut rules, now());
        assert_eq!(rules.len(), 1);
        assert_eq!(rules[0].name, "satcount");
        assert_eq!(manager.controls.len(), 1);
    }

    /// The Designer's words and places are these.
    #[test]
    fn the_designers_words_and_places_are_these() {
        let (Some(form), Some(control)) = (
            csharp("Warnings/WarningsManager.Designer.cs"),
            csharp("Warnings/WarningControl.cs"),
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let has = |source: &str, line: String| assert!(source.contains(&line), "{line}");
        let place = |(x, y, w, h): (f32, f32, f32, f32)| {
            (
                format!("new System.Drawing.Point({x}, {y});"),
                format!("new System.Drawing.Size({w}, {h});"),
            )
        };
        has(&form, format!("this.Text = \"{FORM_TEXT}\";"));
        has(
            &form,
            format!(
                "this.ClientSize = new System.Drawing.Size({}, {});",
                CLIENT.0, CLIENT.1
            ),
        );
        for (name, text, at) in [("BUT_Add", ADD, ADD_AT), ("BUT_save", SAVE, SAVE_AT)] {
            let (location, size) = place(at);
            has(&form, format!("this.{name}.Text = \"{text}\";"));
            has(&form, format!("this.{name}.Location = {location}"));
            has(&form, format!("this.{name}.Size = {size}"));
        }
        for (name, text, x) in HEADERS {
            has(&form, format!("this.{name}.Text = \"{text}\";"));
            has(
                &form,
                format!("this.{name}.Location = new System.Drawing.Point({x}, {HEADER_Y});"),
            );
        }
        let (location, size) = place(PANEL_AT);
        has(&form, format!("this.panel1.Location = {location}"));
        has(&form, format!("this.panel1.Size = {size}"));
        has(
            &control,
            format!(
                "this.Size = new System.Drawing.Size({}, {});",
                ROW_SIZE.0, ROW_SIZE.1
            ),
        );
        has(&control, format!("this.CB_type.Text = \"{TYPE_TEXT}\";"));
        has(
            &control,
            format!(
                "this.CB_type.Location = new System.Drawing.Point({}, {});",
                TYPE_AT.0, TYPE_AT.1
            ),
        );
        for (name, at) in [
            ("CMB_Source", SOURCE_AT),
            ("CMB_condition", CONDITION_AT),
            ("NUM_warning", WARNING_AT),
            ("CMB_color", COLOR_AT),
            ("TXT_warningtext", TEXT_AT),
            ("NUM_repeattime", REPEAT_AT),
            ("but_addchild", ADD_CHILD_AT),
            ("but_remove", REMOVE_AT),
        ] {
            let (location, size) = place(at);
            has(&control, format!("this.{name}.Location = {location}"));
            has(&control, format!("this.{name}.Size = {size}"));
        }
        has(
            &control,
            format!("this.but_addchild.Text = \"{ADD_CHILD}\";"),
        );
        has(&control, format!("this.but_remove.Text = \"{REMOVE}\";"));
        has(&control, "this.NUM_warning.DecimalPlaces = 2;".to_owned());
        has(&control, "            99999,".to_owned());
        has(
            &control,
            format!("this.TXT_warningtext.Text = \"{DEFAULT_TEXT}\";"),
        );
        assert_eq!(WARNING_BOX.decimals, 2);
        assert_eq!(
            (REPEAT_BOX.minimum, REPEAT_BOX.maximum, REPEAT_BOX.value),
            (0.0, 100.0, 10.0)
        );
        // Every wiring of the form and of a row is a handler here.
        for wiring in [
            "this.BUT_Add.Click += new System.EventHandler(this.BUT_Add_Click);",
            "this.BUT_save.Click += new System.EventHandler(this.BUT_save_Click);",
        ] {
            assert!(form.contains(wiring), "{wiring}");
        }
        assert_eq!(form.matches(" += new ").count(), 2);
        assert_eq!(control.matches(" += new System.EventHandler(").count(), 9);
    }

    /// The Advanced page's button opens the form over the engine's rules; a second click a fresh
    /// one.
    #[test]
    fn the_advanced_page_s_button_opens_the_form() {
        use super::super::advanced::{Windows, open};
        use super::super::extra_setup::ExtraSetup;
        let mut pages = ExtraSetup::default();
        let mut rules = vec![CustomWarning::on("alt")];
        let at = now();
        let windows = |pages: &mut ExtraSetup, rules: &mut Vec<CustomWarning>| {
            open(
                "but_warningmanager",
                Windows {
                    fft: &mut pages.fft,
                    inspector: &mut pages.inspector,
                    warnings: &mut pages.warnings_manager,
                    rules,
                },
                at,
            )
        };
        assert!(windows(&mut pages, &mut rules));
        assert_eq!(pages.warnings_manager.opened, 1);
        assert_eq!(
            pages
                .warnings_manager
                .window
                .as_ref()
                .map(|m| m.controls.len()),
            Some(1)
        );
        assert!(pages.fft.window.is_none() && pages.inspector.window.is_none());
        rules.push(CustomWarning::on("satcount"));
        assert!(windows(&mut pages, &mut rules));
        assert_eq!(pages.warnings_manager.opened, 2);
        assert_eq!(
            pages
                .warnings_manager
                .window
                .as_ref()
                .map(|m| m.controls.len()),
            Some(2)
        );
        pages.warnings_manager.close();
        assert!(pages.warnings_manager.window.is_none());
        assert_eq!(rules.len(), 2, "closing keeps the rules");
    }

    /// Every fact the GUI script asserts on is one recorded here or by the engine or the quick
    /// views, every control it clicks is one drawn here, and each list row it clicks is the
    /// property, comparison or colour its comment names.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-warnings.gui");
        let source = include_str!("warnings_manager.rs");
        let engine = include_str!("../warnings.rs");
        let quick = include_str!("../quick.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let (code, comment) = line.split_once('#').unwrap_or((line, ""));
            let mut words = code.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(fact)) if fact.starts_with("config.warnings.") => {
                    let what = fact.trim_start_matches("config.warnings.");
                    let what = what
                        .strip_prefix("row.")
                        .and_then(|rest| rest.split_once('.'))
                        .map_or(what, |(_, what)| what);
                    let recorded = source.contains(&format!("\"config.warnings.{what}\""))
                        || source.contains(&format!("key(\"{what}\")"));
                    assert!(recorded, "{fact}");
                    facts += 1;
                }
                (Some("expect"), Some(fact)) if fact.starts_with("warnings.") => {
                    let family = fact.split('.').take(2).collect::<Vec<_>>().join(".");
                    assert!(engine.contains(&format!("\"{family}")), "{fact}");
                    facts += 1;
                }
                (Some("expect"), Some(fact)) if fact.starts_with("fly.quick.") => {
                    // `fly.quick.<n>` is the view's binding, recorded under `key` itself.
                    let what = fact.splitn(4, '.').nth(3).unwrap_or("");
                    let recorded = if what.is_empty() {
                        quick.contains("facts.push((\n                key,")
                    } else {
                        quick.contains(&format!("{{key}}.{what}\""))
                    };
                    assert!(recorded, "{fact}");
                    facts += 1;
                }
                (Some("click" | "reveal"), Some(id)) if id.starts_with("warnings-") => {
                    clicks += 1;
                    let parts: Vec<&str> = id.split('-').collect();
                    match parts.as_slice() {
                        ["warnings", "add" | "save" | "close"] => {
                            assert!(source.contains(&format!("\"{id}\"")), "{id}");
                        }
                        ["warnings", row, what, rest @ ..] => {
                            assert!(row.parse::<usize>().is_ok(), "{id}");
                            let control = [
                                "type",
                                "source",
                                "condition",
                                "warning",
                                "color",
                                "text",
                                "repeat",
                                "addchild",
                                "remove",
                            ];
                            assert!(control.contains(what), "{id}");
                            // A list's row: the comment names what it is.
                            if let [key] = rest
                                && let Ok(key) = key.parse::<usize>()
                            {
                                let named = match *what {
                                    "source" => options().get(key).copied(),
                                    "condition" => Conditional::ALL.get(key).map(|c| c.name()),
                                    _ => COLORS.get(key).map(|(name, _)| *name),
                                };
                                let named = named.expect("a row of the list");
                                assert!(comment.contains(named), "{id} is {named}");
                            }
                        }
                        _ => panic!("{id}"),
                    }
                }
                _ => {}
            }
        }
        assert!(facts >= 20, "{facts} facts");
        assert!(clicks >= 10, "{clicks} clicks");
    }
}
