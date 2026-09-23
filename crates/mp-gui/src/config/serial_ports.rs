//! The Serial Ports page of Initial Setup: `GCSViews/ConfigurationView/ConfigSerial.cs`, a
//! Mandatory Hardware entry (`GCSViews/InitialSetup.cs:218-221`), listed once every parameter is
//! in.
//!
//! What it shows: a table with one row per serial port, `SERIAL1` up to the highest-numbered
//! `SERIALn_BAUD` the vehicle has (one digit: `ConfigSerial.cs:163-176`) - the port's name, its
//! speed (`SERIALn_BAUD`) and protocol (`SERIALn_PROTOCOL`) as combo boxes of the values the
//! parameter documentation lists, the names of the `SERIALn_OPTIONS` bits that are set, and a Set
//! Bitmask button that opens a window of those bits as check boxes - and under it the note that
//! the changes take effect when the board is rebooted. `Activate` builds it all afresh each time
//! the page is shown (`ConfigSerial.cs:37-380`).
//!
//! Every change is written at once through the page's own `setParam`, which says "Parameter X not
//! found" for a name the vehicle has not listed and "Unable to set parameter X" when the set fails
//! (`ConfigSerial.cs:467-491`). A protocol change then applies `SerialOptionRules.json`: for
//! MAVLink1 the speed goes to 115200, for either MAVLink the options to 0, and the note becomes the
//! rule's comment - or empty, for a protocol with no rule - with a warning added when four or more
//! ports speak MAVLink (`ConfigSerial.cs:382-430`). The C#'s sets are synchronous, one after
//! another on the UI thread; here they go one at a time in the same order, each after the last is
//! answered.
//!
//! The geometry is the Designer's table: five columns of 152, 114, 132, 307 and the rest of the
//! 789-pixel table, rows of an eleventh of its height, at 9, 10. The colours are this
//! application's.
//!
//! What is not ported, and why:
//!
//! * the port names from `@SYS/uarts.txt`, which `Activate` downloads over MAVLink FTP the first
//!   time, behind a progress window (`ConfigSerial.cs:77-161`): this application has no MAVLink
//!   FTP client, so each port is named "SERIAL PORT n" with only its RTS/CTS note under it - as
//!   the C# names a port when the download fails, which it does silently;
//! * reading `SerialOptionRules.json` from beside the executable: the rules are the file Mission
//!   Planner ships, built in, so its "Error reading SerialOptionRules.json file" box cannot arise;
//! * the bitmask window's conversion of its value to the parameter's integer type
//!   (`MavlinkCheckBoxBitMask.cs:35-48`): the vehicle's table here holds values without their
//!   types. It changes only a value with the type's top bit set, and no documented
//!   `SERIALn_OPTIONS` bit reaches it;
//! * the bitmask windows outliving the screen: in the C# they are separate top-most forms that
//!   stay open until closed; here they close with the page object, when the screen is left;
//! * the crashes: a missing `SERIALn_PROTOCOL` beside a `SERIALn_BAUD`, a rule's speed the list
//!   does not hold, and a protocol change after `Activate` stopped short each throw in the C#.
//!   A missing `SERIALn_OPTIONS` stops `Activate` where it stands, as the C#'s exception does,
//!   and the rest are skipped.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, Div, FontWeight, SharedString, Window, div, prelude::*, px, rgb};
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;

use crate::MissionPlanner;
use crate::config::failsafe::{Lookup, options};
use crate::config::servo_output::{Combo, Message, combo_box, dropdown, modal, value_of};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{action, panel, theme};

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:220`
pub const TITLE: &str = "Serial Ports";

/// The note `Activate` puts under the table.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:361-363`
pub const REBOOT_NOTE: &str =
    "Note: Changes to the serial port settings will not take effect until the board is rebooted.";

/// What a protocol change adds to the note when four or more ports speak MAVLink.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:426-429`
pub const MAVLINK_WARNING: &str =
    "\r\nWarning: Maximum number of Mavlink ports are 5 including the USB port!";

/// The header row: `myLabel1` to `myLabel4`, the third spelt as the Designer spells it.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.Designer.cs:78, 88, 98, 108`
const HEADERS: [&str; 4] = ["Port Name", "Speed", "Protcol", "Options"];

/// The Set Bitmask button's text.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:326`
const SET_BITMASK: &str = "Set Bitmask";

/// The table's `Location` and `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.Designer.cs:51, 67`
const TABLE: (f32, f32, f32, f32) = (9.0, 10.0, 789.0, 497.0);

/// The columns: four absolute widths, and the fifth's ten pixels grown by what the table has
/// left over, as a table whose columns are all absolute gives its last column.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.Designer.cs:41-46`
const COLUMNS: [f32; 5] = [152.0, 114.0, 132.0, 307.0, 84.0];

/// Eleven rows share the table's height, less the last row's 25 pixels.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.Designer.cs:54-66`
const ROW_HEIGHT: f32 = (497.0 - 25.0) / 11.0;

/// A plain `ComboBox`'s default size, which the two combo boxes keep: their `Anchor = None`
/// clears the `Dock = Fill` set before it, and centres them in the cell.
const COMBO: (f32, f32) = (121.0, 21.0);

/// One entry of `SerialOptionRules.json`: for a protocol, the speed and options to preset (-1 for
/// none) and the note's comment.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:499-513`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rule {
    /// `PresetBaudRate`.
    pub baudrate: i64,
    /// `PresetOptionsByte`.
    pub options: i64,
    /// `Comment`.
    pub comment: &'static str,
}

/// `SerialOptionRules.json` as Mission Planner ships it beside the executable, by protocol.
/// `// C#: SerialOptionRules.json:1-12; MissionPlanner.csproj:1035-1037`
pub const OPTION_RULES: [(i64, Rule); 2] = [
    (
        1,
        Rule {
            baudrate: 115,
            options: 0,
            comment: "If connecting a Mavlink sensor, consider setting 'Do not forward Mavlink to/from'",
        },
    ),
    (
        2,
        Rule {
            baudrate: -1,
            options: 0,
            comment: "If connecting a Mavlink sensor, consider setting 'Do not forward Mavlink to/from'",
        },
    ),
];

/// The rules with any speed `SERIAL1_BAUD`'s documentation does not list taken out.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:60-74`
#[must_use]
pub fn sanitised_rules(baud_options: &[(i64, String)]) -> Vec<(i64, Rule)> {
    OPTION_RULES
        .iter()
        .map(|(protocol, rule)| {
            let mut rule = *rule;
            if rule.baudrate > -1 && !baud_options.iter().any(|(key, _)| *key == rule.baudrate) {
                rule.baudrate = -1;
            }
            (*protocol, rule)
        })
        .collect()
}

/// How many ports: the largest digit after `SERIAL` in a `SERIALn_BAUD` name, one digit only -
/// `SERIAL0_BAUD`, the USB port, counts as none.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:163-176`
#[must_use]
pub fn port_count(parameters: &[(String, f64)]) -> usize {
    parameters
        .iter()
        .filter(|(name, _)| name.starts_with("SERIAL") && name.ends_with("_BAUD"))
        .filter_map(|(name, _)| name.get(6..7).and_then(|digit| digit.parse().ok()))
        .max()
        .unwrap_or(0)
}

/// `int.TryParse(value.ToString(), out val)`: a whole number within an `int`.
#[allow(clippy::cast_possible_truncation)] // checked to be a whole number within range
fn whole(value: f64) -> Option<i64> {
    (value.fract() == 0.0 && value >= f64::from(i32::MIN) && value <= f64::from(i32::MAX))
        .then_some(value as i64)
}

/// `setLabelOptions`: the names of the bits set, joined by " / ".
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:460-465`
#[must_use]
pub fn options_text(param: &str, value: f64, lookup: Lookup) -> String {
    // `(uint)param_value & (1 << bin.Key)`, the shift taken modulo 32 as C#'s is.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let value = value as u32;
    lookup(param)
        .map(|meta| {
            meta.bitmask
                .iter()
                .filter(|(bit, _)| value & (1_u32 << (bit & 31)) > 0)
                .map(|(_, name)| name.trim())
                .collect::<Vec<_>>()
                .join(" / ")
        })
        .unwrap_or_default()
}

/// One port's row.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortRow {
    /// The port's number, from 1.
    pub port: usize,
    /// The name label: "SERIAL PORT n", then on its own line the UART's name and its RTS/CTS.
    pub label: String,
    /// The speed's combo box, when the documentation lists speeds.
    pub baud: Option<Combo>,
    /// The protocol's combo box, when it lists protocols.
    pub protocol: Option<Combo>,
    /// The options label, once the row got that far.
    pub options: Option<String>,
    /// Whether it has a Set Bitmask button: the documentation lists the options' bits.
    pub bitmask: bool,
}

/// A combo box as `Activate` makes one: bound to its list, then `SelectedValue` set to the
/// parameter when it is a whole number, `SelectedIndex` -1 when it is not.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:234-253, 265-290`
fn port_combo(param: &str, options: Vec<(i64, String)>, value: f64) -> Combo {
    let mut combo = Combo {
        param: param.to_owned(),
        options,
        selected: None,
        enabled: true,
        top_index: 0,
    };
    if let Some(value) = whole(value)
        && combo.options.iter().any(|(key, _)| *key == value)
    {
        combo.selected = Some(value);
    }
    combo
}

/// What a write leads to once it has been answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Then {
    /// Nothing.
    Nothing,
    /// `doApplyRules(portName, value)`, after a protocol change.
    Rules {
        /// The port.
        port: usize,
        /// The protocol chosen.
        protocol: i64,
    },
    /// `setLabelOptions`, after an options change.
    Label {
        /// The port.
        port: usize,
    },
}

/// One `setParam(name, value)` of the page's.
#[derive(Debug, Clone, PartialEq)]
struct Step {
    param: String,
    value: f64,
    then: Then,
}

/// A `MavlinkCheckBoxBitMask` in its window: the parameter's name and description, and a check
/// box per documented bit.
/// `// C#: Controls/MavlinkCheckBoxBitMask.cs:74-141; GCSViews/ConfigurationView/ConfigSerial.cs:327-353`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BitmaskDialog {
    /// `ParamName`.
    pub param: String,
    /// The port whose options label it updates.
    pub port: usize,
    /// `myLabel1`: the documentation's display name.
    pub title: String,
    /// `label1`: its description.
    pub description: String,
    /// Each bit, its name, and whether it is checked.
    pub bits: Vec<(u32, String, bool)>,
}

impl BitmaskDialog {
    /// `setup`: a box per documented bit, checked for the bits the value has.
    fn setup(param: &str, port: usize, value: f64, lookup: Lookup) -> Self {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let value = value as u32;
        let meta = lookup(param);
        Self {
            param: param.to_owned(),
            port,
            title: meta.map_or_else(String::new, |meta| meta.display_name.to_owned()),
            description: meta.map_or_else(String::new, |meta| meta.description.to_owned()),
            bits: meta.map_or_else(Vec::new, |meta| {
                meta.bitmask
                    .iter()
                    .map(|(bit, name)| (*bit, name.to_string(), value & (1 << (bit & 31)) > 0))
                    .collect()
            }),
        }
    }

    /// `Value`: every checked bit added up.
    /// `// C#: Controls/MavlinkCheckBoxBitMask.cs:24-51`
    #[must_use]
    pub fn value(&self) -> f64 {
        self.bits
            .iter()
            .filter(|(.., checked)| *checked)
            .map(|(bit, ..)| f64::from(1_u32 << (bit & 31)))
            .sum()
    }
}

/// The page object and what it keeps.
#[derive(Debug, Default)]
pub struct SerialPorts {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Whether the page is showing.
    active: bool,
    /// `_gotOptionRules` and `_optionRules`.
    rules: Option<Vec<(i64, Rule)>>,
    /// `serialPorts`.
    ports: usize,
    /// The rows `Activate` built.
    rows: Vec<PortRow>,
    /// `noteLabel.Text`, while there is one.
    note: Option<String>,
    /// The combo box whose list is down: a row, and whether it is the protocol's.
    dropdown: Option<(usize, bool)>,
    /// The bitmask windows open, the last on top.
    dialogs: Vec<BitmaskDialog>,
    /// Sets waiting their turn.
    queue: VecDeque<Step>,
    /// The set the link is carrying.
    in_flight: Option<(RequestId, Step)>,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// How the last set ended.
    last_write: Option<String>,
}

impl SerialPorts {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// `serialPorts`.
    #[must_use]
    pub const fn ports(&self) -> usize {
        self.ports
    }

    /// The rows.
    #[must_use]
    pub fn rows(&self) -> &[PortRow] {
        &self.rows
    }

    /// The note, while there is a note label.
    #[must_use]
    pub fn note(&self) -> Option<&str> {
        self.note.as_deref()
    }

    /// The combo box whose list is down.
    #[must_use]
    pub const fn dropdown(&self) -> Option<(usize, bool)> {
        self.dropdown
    }

    /// The bitmask windows open.
    #[must_use]
    pub fn dialogs(&self) -> &[BitmaskDialog] {
        &self.dialogs
    }

    /// The message box showing, if one is.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// How the last set ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.last_write.as_deref()
    }

    /// Sets waiting or on their way.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.len() + usize::from(self.in_flight.is_some())
    }

    /// `Activate`: the rules read the first time, then the table built afresh from the vehicle's
    /// parameters - a row per port up to the highest, the note under them. A port with no
    /// `SERIALn_BAUD` gets its name only; a port with no `SERIALn_OPTIONS` stops the building
    /// there, with no note, as the C#'s exception does.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:37-380`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key, lookup: Lookup) {
        if self.made_for != Some(key) {
            self.dispose();
            self.made_for = Some(key);
        }
        if self.rules.is_none() {
            self.rules = Some(sanitised_rules(&options("SERIAL1_BAUD", lookup)));
        }
        self.active = true;
        self.dropdown = None;
        self.rows.clear();
        self.note = None;
        self.ports = port_count(parameters);
        if self.ports == 0 {
            return;
        }
        for port in 1..=self.ports {
            let name = format!("SERIAL{port}");
            // No UART names: see the module's notes. `// C#: :188-209`
            let mut uart = String::new();
            match value_of(parameters, &format!("BRD_SER{port}_RTSCTS")) {
                Some(value) if (value - 1.0).abs() < f64::EPSILON => uart.push_str(" (RTS/CTS)"),
                Some(value) if (value - 2.0).abs() < f64::EPSILON => {
                    uart.push_str(" (RTS/CTS Auto)");
                }
                _ => {}
            }
            let mut row = PortRow {
                port,
                label: format!("SERIAL PORT {port}\n{uart}"),
                baud: None,
                protocol: None,
                options: None,
                bitmask: false,
            };
            // `// C#: :224-258`
            let baud_param = format!("{name}_BAUD");
            let Some(baud) = value_of(parameters, &baud_param) else {
                self.rows.push(row);
                continue;
            };
            let baud_options = options(&baud_param, lookup);
            if !baud_options.is_empty() {
                row.baud = Some(port_combo(&baud_param, baud_options, baud));
            }
            // `// C#: :260-297`
            let protocol_param = format!("{name}_PROTOCOL");
            let protocol_options = options(&protocol_param, lookup);
            if !protocol_options.is_empty() {
                let Some(protocol) = value_of(parameters, &protocol_param) else {
                    self.rows.push(row);
                    return;
                };
                row.protocol = Some(port_combo(&protocol_param, protocol_options, protocol));
            }
            // `// C#: :299-358`
            let options_param = format!("{name}_OPTIONS");
            let Some(value) = value_of(parameters, &options_param) else {
                self.rows.push(row);
                return;
            };
            row.options = Some(options_text(&options_param, value, lookup));
            row.bitmask = lookup(&options_param).is_some_and(|meta| !meta.bitmask.is_empty());
            self.rows.push(row);
        }
        self.note = Some(REBOOT_NOTE.to_owned());
    }

    /// The page hidden: its `Deactivate` does nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:493-496`
    pub fn hide(&mut self) {
        self.active = false;
        self.dropdown = None;
    }

    /// The page object disposed with its screen. Sets already asked for still go.
    pub fn dispose(&mut self) {
        self.made_for = None;
        self.active = false;
        self.rules = None;
        self.ports = 0;
        self.rows.clear();
        self.note = None;
        self.dropdown = None;
        self.dialogs.clear();
    }

    /// Drops a row's speed or protocol list down, or back up.
    pub fn toggle_dropdown(&mut self, row: usize, protocol: bool) {
        self.dropdown = if self.dropdown == Some((row, protocol)) {
            None
        } else {
            if let Some(combo) = self.combo_mut(row, protocol) {
                combo.open_list();
            }
            Some((row, protocol))
        };
    }

    /// The wheel over the dropped-down list.
    pub fn scroll_list(&mut self, row: usize, protocol: bool, lines: i32) {
        if self.dropdown == Some((row, protocol))
            && let Some(combo) = self.combo_mut(row, protocol)
        {
            combo.scroll_list(lines);
        }
    }

    fn combo_mut(&mut self, row: usize, protocol: bool) -> Option<&mut Combo> {
        let row = self.rows.get_mut(row)?;
        if protocol {
            row.protocol.as_mut()
        } else {
            row.baud.as_mut()
        }
    }

    /// A speed chosen: `SelectedIndexChanged` sets it.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:254-257`
    pub fn choose_baud(&mut self, row: usize, key: i64) {
        self.dropdown = None;
        let Some(combo) = self.rows.get_mut(row).and_then(|row| row.baud.as_mut()) else {
            return;
        };
        if combo.select(key) {
            #[allow(clippy::cast_precision_loss)] // a speed code
            let value = key as f64;
            self.queue.push_back(Step {
                param: combo.param.clone(),
                value,
                then: Then::Nothing,
            });
        }
    }

    /// A protocol chosen: `SelectedIndexChanged` sets it, then applies the rules.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:291-296`
    pub fn choose_protocol(&mut self, row: usize, key: i64) {
        self.dropdown = None;
        let Some(port_row) = self.rows.get_mut(row) else {
            return;
        };
        let port = port_row.port;
        let Some(combo) = port_row.protocol.as_mut() else {
            return;
        };
        if combo.select(key) {
            #[allow(clippy::cast_precision_loss)] // a protocol number
            let value = key as f64;
            self.queue.push_back(Step {
                param: combo.param.clone(),
                value,
                then: Then::Rules {
                    port,
                    protocol: key,
                },
            });
        }
    }

    /// Set Bitmask: a new window for the port's options, read from the vehicle's table now.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:327-353`
    pub fn open_bitmask(&mut self, row: usize, parameters: &[(String, f64)], lookup: Lookup) {
        self.dropdown = None;
        let Some(port) = self.rows.get(row).map(|row| row.port) else {
            return;
        };
        let param = format!("SERIAL{port}_OPTIONS");
        let value = value_of(parameters, &param).unwrap_or(0.0);
        self.dialogs
            .push(BitmaskDialog::setup(&param, port, value, lookup));
    }

    /// A bit's box clicked in a window: its value set, and the options label after it.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:342-347; Controls/MavlinkCheckBoxBitMask.cs:143-160`
    pub fn click_bit(&mut self, dialog: usize, bit: usize) {
        let Some(window) = self.dialogs.get_mut(dialog) else {
            return;
        };
        let Some((.., checked)) = window.bits.get_mut(bit) else {
            return;
        };
        *checked = !*checked;
        let step = Step {
            param: window.param.clone(),
            value: window.value(),
            then: Then::Label { port: window.port },
        };
        self.queue.push_back(step);
    }

    /// Closes a bitmask window.
    pub fn close_bitmask(&mut self, dialog: usize) {
        if dialog < self.dialogs.len() {
            self.dialogs.remove(dialog);
        }
    }

    /// Dismisses the message box showing.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// A message box with no caption, as the page's own are.
    fn say(&mut self, text: String) {
        self.messages.push_back(Message { title: "", text });
    }

    /// Once a frame: a page object whose screen has gone is disposed, the set on its way is read
    /// back and what follows it done, and the next is sent.
    pub fn tick(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        on_setup: bool,
        lookup: Lookup,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.dispose();
        }
        if let Some((id, step)) = self.in_flight.take() {
            match telemetry.request(id).map(|request| request.outcome()) {
                Some(None) => {
                    self.in_flight = Some((id, step));
                    return;
                }
                Some(Some(outcome)) => self.finish(&step, Some(outcome), view, lookup),
                None => self.finish(&step, None, view, lookup),
            }
        }
        while self.in_flight.is_none() {
            let Some(step) = self.queue.pop_front() else {
                break;
            };
            self.start(step, telemetry, view, lookup);
        }
    }

    /// `setParam`: refused with a message for a name the vehicle has not listed, otherwise sent.
    /// What follows it runs either way, as the C#'s handler carries on after a false.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:467-491`
    fn start(&mut self, step: Step, telemetry: &Telemetry, view: &TelemetryView, lookup: Lookup) {
        if value_of(&view.parameters, &step.param).is_none() {
            self.last_write = Some(format!("{} {} not found", step.param, step.value));
            self.say(format!("Parameter {} not found", step.param));
            self.follow(&step, None, view, lookup);
            return;
        }
        match telemetry.set_parameter_confirmed(&step.param, step.value) {
            Some(id) => self.in_flight = Some((id, step)),
            None => self.finish(&step, Some(RequestOutcome::TimedOut), view, lookup),
        }
    }

    /// A set answered: "Unable to set parameter X" when it failed, then what follows it, with the
    /// value the vehicle's table now holds.
    fn finish(
        &mut self,
        step: &Step,
        outcome: Option<RequestOutcome>,
        view: &TelemetryView,
        lookup: Lookup,
    ) {
        let held = value_of(&view.parameters, &step.param);
        let now = match outcome {
            Some(RequestOutcome::Accepted { value }) => {
                let echoed = value.map_or(step.value, |value| value.as_f64());
                self.last_write = Some(format!("{} {echoed} accepted", step.param));
                Some(echoed)
            }
            Some(RequestOutcome::Unchanged | RequestOutcome::Sent) => {
                self.last_write = Some(format!("{} {} unchanged", step.param, step.value));
                Some(step.value)
            }
            Some(failed) => {
                let why = match failed {
                    RequestOutcome::UnknownParameter => "not on the vehicle".to_owned(),
                    RequestOutcome::Rejected(result) => format!("rejected {result}"),
                    _ => "timed out".to_owned(),
                };
                self.last_write = Some(format!("{} {} failed: {why}", step.param, step.value));
                self.say(format!("Unable to set parameter {}", step.param));
                held
            }
            None => held,
        };
        self.follow(step, now, view, lookup);
    }

    /// What a set leads to: the rules after a protocol, the label after the options.
    fn follow(&mut self, step: &Step, now: Option<f64>, view: &TelemetryView, lookup: Lookup) {
        match step.then {
            Then::Nothing => {}
            Then::Label { port } => {
                if let Some(value) = now
                    && let Some(row) = self.rows.iter_mut().find(|row| row.port == port)
                    && row.options.is_some()
                {
                    row.options = Some(options_text(&step.param, value, lookup));
                }
            }
            Then::Rules { port, protocol } => self.apply_rules(port, protocol, now, view),
        }
    }

    /// `doApplyRules`: the rule for the protocol - its speed selected, which sets it; its options
    /// set; its comment the note - or an empty note, then the MAVLink warning when four or more
    /// ports have protocol 1 or 2. The rule's sets go next, before anything asked for since, as
    /// they are made inside the C#'s handler.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:384-430`
    fn apply_rules(&mut self, port: usize, protocol: i64, now: Option<f64>, view: &TelemetryView) {
        let rule = self
            .rules
            .as_deref()
            .and_then(|rules| rules.iter().find(|(key, _)| *key == protocol))
            .map(|(_, rule)| *rule);
        let mut next = Vec::new();
        let mut note = String::new();
        if let Some(rule) = rule {
            if rule.baudrate > -1
                && let Some(baud) = self
                    .rows
                    .iter_mut()
                    .find(|row| row.port == port)
                    .and_then(|row| row.baud.as_mut())
                && baud.select(rule.baudrate)
            {
                #[allow(clippy::cast_precision_loss)]
                let value = rule.baudrate as f64;
                next.push(Step {
                    param: baud.param.clone(),
                    value,
                    then: Then::Nothing,
                });
            }
            if rule.options > -1 {
                #[allow(clippy::cast_precision_loss)]
                let value = rule.options as f64;
                next.push(Step {
                    param: format!("SERIAL{port}_OPTIONS"),
                    value,
                    then: Then::Label { port },
                });
            }
            note = format!("SERIAL{port} : {}", rule.comment);
        }
        let mavlink = (1..=self.ports)
            .filter(|index| {
                let value = if *index == port {
                    now
                } else {
                    value_of(&view.parameters, &format!("SERIAL{index}_PROTOCOL"))
                };
                value.is_some_and(|value| {
                    (value - 1.0).abs() < f64::EPSILON || (value - 2.0).abs() < f64::EPSILON
                })
            })
            .count();
        if mavlink >= 4 {
            note.push_str(MAVLINK_WARNING);
        }
        if self.note.is_some() {
            self.note = Some(note);
        }
        for step in next.into_iter().rev() {
            self.queue.push_front(step);
        }
    }
}

/// Facts a UI test asserts on: the table, the note, the windows, the sets and what the vehicle
/// holds.
pub fn record_facts(serial: &SerialPorts, view: &TelemetryView) {
    use crate::facts::record;
    record("config.serial.active", serial.is_active());
    record("config.serial.ports", serial.ports());
    record("config.serial.rows", serial.rows().len());
    record(
        "config.serial.note",
        serial
            .note()
            .map_or_else(|| "none".to_owned(), |note| note.replace("\r\n", " ")),
    );
    record(
        "config.serial.message",
        serial
            .message()
            .map_or("none", |message| message.text.as_str()),
    );
    record(
        "config.serial.list.top",
        serial
            .dropdown()
            .and_then(|(index, protocol)| {
                let row = serial.rows().get(index)?;
                if protocol {
                    row.protocol.as_ref()
                } else {
                    row.baud.as_ref()
                }
            })
            .map_or_else(|| "none".to_owned(), |combo| combo.top_index.to_string()),
    );
    record("config.serial.write", serial.last_write().unwrap_or("none"));
    record("config.serial.writes.pending", serial.pending());
    let top = serial.dialogs().last();
    record(
        "config.serial.dialog",
        top.map_or("none", |dialog| dialog.param.as_str()),
    );
    record(
        "config.serial.dialog.title",
        top.map_or("none", |dialog| dialog.title.as_str()),
    );
    // The bits checked in the window on top, "none" for none.
    let checked: Vec<String> = top.map_or_else(Vec::new, |dialog| {
        dialog
            .bits
            .iter()
            .filter(|(.., checked)| *checked)
            .map(|(bit, ..)| bit.to_string())
            .collect()
    });
    record(
        "config.serial.dialog.checked",
        if checked.is_empty() {
            "none".to_owned()
        } else {
            checked.join(",")
        },
    );
    let combo = |combo: Option<&Combo>| {
        combo
            .and_then(|combo| combo.selected)
            .map_or_else(|| "none".to_owned(), |key| key.to_string())
    };
    for row in serial.rows() {
        let n = row.port;
        record(
            format!("config.serial.{n}.label"),
            row.label.replace('\n', " ").trim_end(),
        );
        record(format!("config.serial.{n}.baud"), combo(row.baud.as_ref()));
        record(
            format!("config.serial.{n}.baud.text"),
            row.baud.as_ref().map_or("none", Combo::text),
        );
        record(
            format!("config.serial.{n}.protocol"),
            combo(row.protocol.as_ref()),
        );
        record(
            format!("config.serial.{n}.protocol.text"),
            row.protocol.as_ref().map_or("none", Combo::text),
        );
        record(
            format!("config.serial.{n}.options"),
            row.options.as_deref().unwrap_or("none"),
        );
        record(format!("config.serial.{n}.bitmask"), row.bitmask);
    }
    for (name, value) in view.parameters.iter() {
        if name.starts_with("SERIAL") || name.starts_with("BRD_SER") {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// Where a column starts.
fn column_x(index: usize) -> f32 {
    TABLE.0 + COLUMNS.iter().take(index).sum::<f32>()
}

/// Where a table row starts; row 0 is the header.
#[allow(clippy::cast_precision_loss)] // at most eleven rows
fn row_y(index: usize) -> f32 {
    TABLE.1 + ROW_HEIGHT * index as f32
}

/// A cell's text, centred as `Anchor = None` or `TextAlign = MiddleCenter` centre it.
fn centred(x: f32, y: f32, width: f32, text: impl IntoElement) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(ROW_HEIGHT))
        .flex()
        .flex_col()
        .items_center()
        .justify_center()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(text)
}

/// A combo box centred in its cell, no wider than the cell's margins allow.
fn cell_combo(
    id: String,
    combo: &Combo,
    column: usize,
    row: usize,
    on_open: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let width = COMBO
        .0
        .min(COLUMNS.get(column).copied().unwrap_or(0.0) - 6.0);
    let cell = COLUMNS.get(column).copied().unwrap_or(0.0);
    combo_box(
        id,
        combo,
        (
            column_x(column) + (cell - width) / 2.0,
            row_y(row) + (ROW_HEIGHT - COMBO.1) / 2.0,
            width,
            COMBO.1,
        ),
        on_open,
        cx,
    )
}

/// The page, laid out as `ConfigSerial`'s table is.
pub fn page(serial: &SerialPorts, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    if !serial.is_active() {
        return None;
    }
    let mut body = div()
        .relative()
        .w(px(TABLE.0 + TABLE.2))
        .h(px(TABLE.1 + TABLE.3));
    for (index, header) in HEADERS.iter().enumerate() {
        body = body.child(centred(
            column_x(index),
            row_y(0),
            COLUMNS.get(index).copied().unwrap_or(0.0),
            *header,
        ));
    }
    for (index, row) in serial.rows().iter().enumerate() {
        let table_row = index + 1;
        let mut name = div()
            .flex()
            .flex_col()
            .items_center()
            .font_weight(FontWeight::BOLD);
        for line in row.label.lines() {
            name = name.child(line.to_owned());
        }
        body = body.child(centred(column_x(0), row_y(table_row), COLUMNS[0], name));
        if let Some(baud) = &row.baud {
            body = body.child(cell_combo(
                format!("serial-{}", baud.param),
                baud,
                1,
                table_row,
                move |this| this.serial_ports.toggle_dropdown(index, false),
                cx,
            ));
        }
        if let Some(protocol) = &row.protocol {
            body = body.child(cell_combo(
                format!("serial-{}", protocol.param),
                protocol,
                2,
                table_row,
                move |this| this.serial_ports.toggle_dropdown(index, true),
                cx,
            ));
        }
        if let Some(options) = &row.options {
            body = body.child(
                div()
                    .absolute()
                    .left(px(column_x(3) + 3.0))
                    .top(px(row_y(table_row)))
                    .w(px(300.0))
                    .h(px(ROW_HEIGHT))
                    .flex()
                    .items_center()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(options.clone()),
            );
        }
        if row.bitmask {
            let id = format!("serial-SERIAL{}_OPTIONS-bitmask", row.port);
            body = body.child(
                div()
                    .absolute()
                    .left(px(column_x(4) + 3.0))
                    .top(px(row_y(table_row) + 3.0))
                    .w(px(75.0))
                    .h(px(23.0))
                    .child(
                        crate::probe::measured(id.clone(), div())
                            .id(SharedString::from(id))
                            .size_full()
                            .flex()
                            .items_center()
                            .justify_center()
                            .rounded_sm()
                            .border_1()
                            .border_color(rgb(theme::BORDER))
                            .bg(rgb(theme::ACTION))
                            .text_xs()
                            .text_color(rgb(theme::TEXT))
                            .cursor_pointer()
                            .hover(|style| style.bg(rgb(theme::BORDER)))
                            .child(SET_BITMASK)
                            .on_click(cx.listener(move |this, _event, _window, cx| {
                                let parameters = this.telemetry.view().parameters;
                                this.serial_ports.open_bitmask(
                                    index,
                                    &parameters,
                                    crate::metadata::lookup,
                                );
                                cx.notify();
                            })),
                    ),
            );
        }
    }
    if let Some(note) = serial.note() {
        let mut lines = div()
            .absolute()
            .left(px(TABLE.0 + 3.0))
            .top(px(row_y(serial.ports() + 1)))
            .w(px(600.0))
            .min_h(px(ROW_HEIGHT))
            .flex()
            .flex_col()
            .justify_center()
            .text_xs()
            .text_color(rgb(theme::TEXT));
        for line in note.lines() {
            lines = lines.child(line.to_owned());
        }
        body = body.child(lines);
    }
    // A dropped-down list goes last, over the rows below it.
    if let Some((index, protocol)) = serial.dropdown()
        && let Some(row) = serial.rows().get(index)
    {
        let (combo, column) = if protocol {
            (row.protocol.as_ref(), 2)
        } else {
            (row.baud.as_ref(), 1)
        };
        if let Some(combo) = combo {
            let cell = COLUMNS.get(column).copied().unwrap_or(0.0);
            let width = COMBO.0.min(cell - 6.0);
            body = body.child(dropdown(
                &format!("serial-{}", combo.param),
                combo,
                (
                    column_x(column) + (cell - width) / 2.0,
                    row_y(index + 1) + (ROW_HEIGHT + COMBO.1) / 2.0,
                    width,
                ),
                move |this, key| {
                    if protocol {
                        this.serial_ports.choose_protocol(index, key);
                    } else {
                        this.serial_ports.choose_baud(index, key);
                    }
                },
                move |this, lines| this.serial_ports.scroll_list(index, protocol, lines),
                cx,
            ));
        }
    }
    Some(panel(TITLE, body).into_any_element())
}

/// One bitmask window: `ShowUserControl`'s form, 700 wide, not modal, over the page.
/// `// C#: Controls/MavlinkCheckBoxBitMask.cs:66-141, 162-233; Utilities/ExtensionsMP.cs:110-132`
fn bitmask_window(
    index: usize,
    dialog: &BitmaskDialog,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let size = window.viewport_size();
    let mut boxes = div()
        .flex()
        .flex_wrap()
        .gap_x(px(5.0))
        .gap_y(px(5.0))
        .w(px(520.0));
    for (bit_index, (bit, name, checked)) in dialog.bits.iter().enumerate() {
        let id = format!("serial-bitmask-{}-{bit}", dialog.param);
        boxes = boxes.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .flex()
                .items_center()
                .gap_1()
                .cursor_pointer()
                .child(
                    div()
                        .size(px(13.0))
                        .flex()
                        .items_center()
                        .justify_center()
                        .border_1()
                        .border_color(rgb(theme::DIM))
                        .bg(rgb(theme::BG))
                        .children(checked.then(|| div().size(px(6.0)).bg(rgb(theme::ACCENT)))),
                )
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::TEXT))
                        .child(name.clone()),
                )
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.serial_ports.click_bit(index, bit_index);
                    cx.notify();
                })),
        );
    }
    let close_id = format!("serial-bitmask-{}-close", dialog.param);
    let frame = div()
        .w(px(700.0))
        .flex()
        .flex_col()
        .gap_2()
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .occlude()
        .child(
            div()
                .flex()
                .justify_between()
                .child(
                    div()
                        .text_sm()
                        .font_weight(FontWeight::BOLD)
                        .text_color(rgb(theme::TEXT))
                        .child(dialog.title.clone()),
                )
                .child(
                    crate::probe::measured(close_id.clone(), div())
                        .id(SharedString::from(close_id))
                        .px_1()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .cursor_pointer()
                        .hover(|style| style.text_color(rgb(theme::TEXT)))
                        .child("X")
                        .on_click(cx.listener(move |this, _event, _window, cx| {
                            this.serial_ports.close_bitmask(index);
                            cx.notify();
                        })),
                ),
        )
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(dialog.description.clone()),
        )
        .child(boxes);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(
                (size.width - px(700.0)) / 2.0,
                size.height / 4.0,
            ))
            .child(frame),
    )
    .with_priority(1)
    .into_any_element()
}

/// The bitmask windows open, and over everything the message box showing.
pub fn overlay(
    serial: &SerialPorts,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let mut out: Vec<AnyElement> = serial
        .dialogs()
        .iter()
        .enumerate()
        .map(|(index, dialog)| bitmask_window(index, dialog, window, cx))
        .collect();
    if let Some(message) = serial.message() {
        let ok = action(
            "serial-message-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.serial_ports.dismiss_message();
                cx.notify();
            }),
        );
        out.push(modal(
            "serial-message",
            message.title,
            &message.text,
            true,
            vec![ok],
            window,
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{Vehicle, param, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;
    use mp_params::ParamMeta;

    fn bundled(name: &str) -> Option<&'static ParamMeta> {
        mp_params::param_meta::lookup(name)
    }

    /// Every port documented as `SERIAL1` is: the fetched documentation copies each port's
    /// fields from `SERIAL1`'s, where the bundled table has the speeds for `SERIAL1` only.
    fn documented(name: &str) -> Option<&'static ParamMeta> {
        let bytes = name.as_bytes();
        if name.starts_with("SERIAL")
            && bytes.get(6).is_some_and(u8::is_ascii_digit)
            && bytes.get(7) == Some(&b'_')
        {
            return mp_params::param_meta::lookup(&format!("SERIAL1{}", &name[7..]));
        }
        mp_params::param_meta::lookup(name)
    }

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    fn sitl() -> Vec<(String, f64)> {
        let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/params/sitl-copter.param");
        let file = mp_params::param_file::ParamFile::load(&fixture).expect("the SITL dump");
        file.iter()
            .map(|(name, value)| (name.to_owned(), value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// The built-in rules are the file Mission Planner ships, read from the tree when it is here.
    #[test]
    fn the_rules_are_serial_option_rules_json() {
        let Some(json) = crate::config_coverage::source::csharp("SerialOptionRules.json") else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let mut found = Vec::new();
        for entry in json.split("\"PresetBaudRate\"").skip(1) {
            let number = |field: &str| -> i64 {
                entry
                    .split(field)
                    .nth(1)
                    .and_then(|rest| rest.split(':').nth(1))
                    .and_then(|rest| rest.split(',').next())
                    .map(str::trim)
                    .and_then(|text| text.parse().ok())
                    .expect("a number")
            };
            let baudrate: i64 = entry
                .split(':')
                .nth(1)
                .and_then(|rest| rest.split(',').next())
                .map(str::trim)
                .and_then(|text| text.parse().ok())
                .expect("the speed");
            let options = number("\"PresetOptionsByte\"");
            let comment = entry
                .split("\"Comment\"")
                .nth(1)
                .and_then(|rest| rest.split('"').nth(1))
                .expect("the comment")
                .to_owned();
            found.push((baudrate, options, comment));
        }
        let keys: Vec<i64> = json
            .lines()
            .filter_map(|line| {
                let line = line.trim();
                line.strip_prefix('"')
                    .and_then(|rest| rest.split_once("\": {"))
                    .and_then(|(key, _)| key.parse().ok())
            })
            .collect();
        let ours: Vec<(i64, i64, String)> = OPTION_RULES
            .iter()
            .map(|(_, rule)| (rule.baudrate, rule.options, rule.comment.to_owned()))
            .collect();
        assert_eq!(found, ours);
        assert_eq!(
            keys,
            OPTION_RULES.iter().map(|(key, _)| *key).collect::<Vec<_>>()
        );
    }

    #[test]
    fn a_rules_speed_the_documentation_lacks_is_dropped() {
        let rules = sanitised_rules(&options("SERIAL1_BAUD", bundled));
        assert_eq!(rules[0].1.baudrate, 115);
        let rules = sanitised_rules(&[(57, "57600".to_owned())]);
        assert_eq!(rules[0].1.baudrate, -1);
        assert_eq!(rules[0].1.options, 0);
    }

    #[test]
    fn the_ports_are_counted_by_one_digit() {
        assert_eq!(port_count(&sitl()), 7, "SERIAL0 to SERIAL7");
        assert_eq!(port_count(&table(&[("SERIAL0_BAUD", 115.0)])), 0);
        // `Substring(6, 1)`: SERIAL12 is port 1.
        assert_eq!(port_count(&table(&[("SERIAL12_BAUD", 57.0)])), 1);
        assert_eq!(port_count(&table(&[])), 0);
    }

    #[test]
    fn the_sitl_copters_seven_ports_are_read() {
        let mut serial = SerialPorts::default();
        serial.activate(&sitl(), key(), documented);
        assert!(serial.is_active());
        assert_eq!(serial.ports(), 7);
        assert_eq!(serial.rows().len(), 7);
        assert_eq!(serial.note(), Some(REBOOT_NOTE));
        let first = &serial.rows()[0];
        assert_eq!(first.label, "SERIAL PORT 1\n");
        let baud = first.baud.as_ref().expect("a speed list");
        assert_eq!(baud.selected, Some(57));
        assert_eq!(baud.text(), "57600");
        let protocol = first.protocol.as_ref().expect("a protocol list");
        assert_eq!(protocol.selected, Some(2));
        assert_eq!(protocol.text(), "MAVLink2");
        assert_eq!(first.options.as_deref(), Some(""));
        assert!(first.bitmask);
        let third = &serial.rows()[2];
        assert_eq!(third.protocol.as_ref().map(Combo::text), Some("GPS"));
        assert_eq!(third.baud.as_ref().map(Combo::text), Some("230400"));
        let fifth = &serial.rows()[4];
        assert_eq!(fifth.protocol.as_ref().map(Combo::text), Some("None"));
    }

    /// The bundled table lists speeds and protocols for `SERIAL1` only; a port whose
    /// documentation lists none gets no box for it, as the C# gives it none, and its row goes on.
    #[test]
    fn a_port_with_no_documented_values_has_no_box_for_them() {
        let mut serial = SerialPorts::default();
        serial.activate(&sitl(), key(), bundled);
        assert!(serial.rows()[0].baud.is_some());
        assert!(serial.rows()[0].protocol.is_some());
        assert!(serial.rows()[1].baud.is_none());
        assert!(serial.rows()[1].protocol.is_none());
        assert!(serial.rows()[1].options.is_some());
        assert_eq!(serial.rows().len(), 7);
    }

    #[test]
    fn rts_cts_is_named_under_the_port() {
        let mut serial = SerialPorts::default();
        serial.activate(
            &table(&[
                ("SERIAL1_BAUD", 57.0),
                ("SERIAL1_PROTOCOL", 2.0),
                ("SERIAL1_OPTIONS", 0.0),
                ("BRD_SER1_RTSCTS", 1.0),
                ("SERIAL2_BAUD", 57.0),
                ("SERIAL2_PROTOCOL", 2.0),
                ("SERIAL2_OPTIONS", 0.0),
                ("BRD_SER2_RTSCTS", 2.0),
            ]),
            key(),
            documented,
        );
        assert_eq!(serial.rows()[0].label, "SERIAL PORT 1\n (RTS/CTS)");
        assert_eq!(serial.rows()[1].label, "SERIAL PORT 2\n (RTS/CTS Auto)");
    }

    #[test]
    fn a_gap_in_the_ports_gets_a_name_only_and_missing_options_stop_the_table() {
        let mut serial = SerialPorts::default();
        serial.activate(
            &table(&[
                ("SERIAL1_BAUD", 57.0),
                ("SERIAL1_PROTOCOL", 2.0),
                ("SERIAL1_OPTIONS", 0.0),
                ("SERIAL3_BAUD", 57.0),
                ("SERIAL3_PROTOCOL", 2.0),
                ("SERIAL3_OPTIONS", 0.0),
            ]),
            key(),
            documented,
        );
        assert_eq!(serial.rows().len(), 3);
        assert!(serial.rows()[1].baud.is_none() && serial.rows()[1].options.is_none());
        assert!(serial.note().is_some());

        // No SERIAL2_OPTIONS, as on firmware older than the parameter: the C# throws there.
        serial.activate(
            &table(&[
                ("SERIAL1_BAUD", 57.0),
                ("SERIAL1_PROTOCOL", 2.0),
                ("SERIAL2_BAUD", 57.0),
                ("SERIAL2_PROTOCOL", 2.0),
            ]),
            key(),
            documented,
        );
        assert_eq!(serial.rows().len(), 1);
        assert!(serial.rows()[0].options.is_none());
        assert!(serial.note().is_none(), "no note label");
    }

    #[test]
    fn the_options_label_names_the_bits_set() {
        assert_eq!(options_text("SERIAL1_OPTIONS", 0.0, bundled), "");
        assert_eq!(options_text("SERIAL1_OPTIONS", 1.0, bundled), "InvertRX");
        assert_eq!(
            options_text("SERIAL1_OPTIONS", 1025.0, bundled),
            "InvertRX / Don't forward mavlink to/from"
        );
    }

    #[test]
    fn a_protocol_change_writes_it_then_the_rule() {
        let telemetry = Telemetry::idle();
        let mut view = telemetry.view();
        view.parameters = sitl().into();
        let mut serial = SerialPorts::default();
        serial.activate(&view.parameters, key(), documented);
        // SERIAL5 to MAVLink1: the protocol, then the rule's 115200 and options 0.
        serial.choose_protocol(4, 1);
        assert_eq!(
            serial.rows()[4].protocol.as_ref().map(Combo::text),
            Some("MAVLink1")
        );
        assert_eq!(serial.pending(), 1);
        // With no vehicle the set fails; the C#'s handler says so and applies the rule anyway.
        serial.tick(&telemetry, &view, true, documented);
        assert_eq!(
            serial.message().map(|message| message.text.as_str()),
            Some("Unable to set parameter SERIAL5_PROTOCOL")
        );
        assert_eq!(
            serial.rows()[4].baud.as_ref().map(Combo::text),
            Some("115200"),
            "the rule selects the speed"
        );
        assert_eq!(
            serial.note(),
            Some(
                "SERIAL5 : If connecting a Mavlink sensor, consider setting 'Do not forward \
                 Mavlink to/from'"
            )
        );
        // The rule's two sets followed, each failing in turn.
        let texts: Vec<&str> = serial
            .messages
            .iter()
            .map(|message| message.text.as_str())
            .collect();
        assert_eq!(
            texts,
            [
                "Unable to set parameter SERIAL5_PROTOCOL",
                "Unable to set parameter SERIAL5_BAUD",
                "Unable to set parameter SERIAL5_OPTIONS",
            ]
        );
        // A protocol with no rule empties the note.
        serial.choose_protocol(4, 5);
        serial.tick(&telemetry, &view, true, documented);
        assert_eq!(serial.note(), Some(""));
    }

    #[test]
    fn four_mavlink_ports_add_the_warning() {
        let telemetry = Telemetry::idle();
        let mut view = telemetry.view();
        let mut table = sitl();
        for (name, value) in &mut table {
            if name == "SERIAL3_PROTOCOL" {
                *value = 2.0;
            }
        }
        view.parameters = table.into();
        let mut serial = SerialPorts::default();
        serial.activate(&view.parameters, key(), documented);
        // SERIAL1, 2 and 3 are MAVLink; SERIAL4 makes four.
        serial.choose_protocol(3, 2);
        serial.tick(&telemetry, &view, true, documented);
        // The set failed, so the table still holds GPS for SERIAL4: three ports, no warning.
        assert!(!serial.note().unwrap_or_default().contains("Warning"));
        serial.apply_rules(4, 2, Some(2.0), &view);
        assert!(
            serial
                .note()
                .is_some_and(|note| note.ends_with(MAVLINK_WARNING))
        );
    }

    #[test]
    fn a_name_the_vehicle_lacks_is_not_sent() {
        let telemetry = Telemetry::idle();
        let mut view = telemetry.view();
        view.parameters = table(&[("SERIAL1_BAUD", 57.0), ("SERIAL1_OPTIONS", 0.0)]).into();
        let mut serial = SerialPorts::default();
        serial.activate(
            &table(&[
                ("SERIAL1_BAUD", 57.0),
                ("SERIAL1_PROTOCOL", 2.0),
                ("SERIAL1_OPTIONS", 0.0),
            ]),
            key(),
            documented,
        );
        serial.choose_protocol(0, 5);
        serial.tick(&telemetry, &view, true, documented);
        assert_eq!(
            serial.message().map(|message| message.text.as_str()),
            Some("Parameter SERIAL1_PROTOCOL not found")
        );
    }

    #[test]
    fn a_bitmask_window_adds_up_its_boxes() {
        let mut serial = SerialPorts::default();
        serial.activate(&sitl(), key(), documented);
        serial.open_bitmask(4, &table(&[("SERIAL5_OPTIONS", 1024.0)]), documented);
        let dialog = serial.dialogs().last().expect("a window");
        assert_eq!(dialog.param, "SERIAL5_OPTIONS");
        assert_eq!(dialog.title, "Telem1 options");
        assert!(
            dialog
                .bits
                .iter()
                .any(|(bit, _, checked)| *bit == 10 && *checked)
        );
        assert!((dialog.value() - 1024.0).abs() < f64::EPSILON);
        serial.click_bit(0, 0);
        assert!((serial.dialogs()[0].value() - 1025.0).abs() < f64::EPSILON);
        assert_eq!(serial.queue.back().map(|step| step.value), Some(1025.0));
        assert_eq!(
            serial.queue.back().map(|step| step.then),
            Some(Then::Label { port: 5 })
        );
        serial.close_bitmask(0);
        assert!(serial.dialogs().is_empty());
    }

    /// A speed change reaches the vehicle, and its echo is what the page reports.
    #[test]
    fn a_speed_change_reaches_the_vehicle() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        vehicle.send(&param("SERIAL1_BAUD", 57.0, 6));
        until("the parameter to be held", || {
            telemetry.holds_parameter("SERIAL1_BAUD")
        });
        let parameters = table(&[
            ("SERIAL1_BAUD", 57.0),
            ("SERIAL1_PROTOCOL", 2.0),
            ("SERIAL1_OPTIONS", 0.0),
        ]);
        let mut serial = SerialPorts::default();
        serial.activate(&parameters, key(), documented);
        serial.choose_baud(0, 115);
        let mut written = None;
        until("the PARAM_SET", || {
            serial.tick(&telemetry, &telemetry.view(), true, documented);
            for message in vehicle.read() {
                if let MavMessage::ParamSet(set) = message {
                    written = Some(set.param_value);
                    vehicle.send(&param("SERIAL1_BAUD", set.param_value, 6));
                }
            }
            written.is_some()
        });
        assert_eq!(written, Some(115.0));
        until("the echo", || {
            serial.tick(&telemetry, &telemetry.view(), true, documented);
            serial.last_write() == Some("SERIAL1_BAUD 115 accepted")
        });
        assert!(serial.message().is_none());
        assert_eq!(serial.pending(), 0);
    }

    #[test]
    fn leaving_the_screen_closes_the_windows_with_the_page() {
        let telemetry = Telemetry::idle();
        let view = telemetry.view();
        let mut serial = SerialPorts::default();
        serial.activate(&sitl(), Key::of(&view), documented);
        serial.open_bitmask(0, &sitl(), documented);
        serial.hide();
        serial.tick(&telemetry, &view, true, documented);
        assert_eq!(serial.dialogs().len(), 1, "kept while the screen shows");
        serial.tick(&telemetry, &view, false, documented);
        assert!(serial.dialogs().is_empty());
        assert!(serial.rows().is_empty());
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-serial.gui");
        let source = include_str!("serial_ports.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.serial.") => {
                    let generic = key
                        .split('.')
                        .map(|part| {
                            if part.chars().all(|c| c.is_ascii_digit()) {
                                "{n}"
                            } else {
                                part
                            }
                        })
                        .collect::<Vec<_>>()
                        .join(".");
                    assert!(
                        source.contains(&format!("\"{key}\""))
                            || source.contains(&format!("\"{generic}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("serial-") => {
                    let drawn = source.contains(&format!("\"{id}\""))
                        || id.starts_with("serial-SERIAL")
                        || id.starts_with("serial-bitmask-SERIAL");
                    assert!(drawn, "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 15 && clicks > 5, "{facts} facts, {clicks} clicks");
    }
}
