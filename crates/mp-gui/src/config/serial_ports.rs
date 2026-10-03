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
//! The ports' names come from the vehicle's `@SYS/uarts.txt`, which the page object downloads
//! over MAVLink FTP the first time it is activated - once, whether or not that works: a
//! `_gotUARTNames` set before the read (`ConfigSerial.cs:77-161`). The read is `GetFile` with
//! `burst` false, a plain read of 80-byte chunks, behind a modal `ProgressReporterDialogue` that
//! says "Trying to download uarts.txt / From FC" over a bar of the read's progress, with a
//! Cancel; the table is built when the window closes, as the rest of `Activate` runs then. A
//! line of the file that starts `SERIALn`, n above 0, names port n with its second word; row n
//! takes the name for n, whatever order the file lists them in. Each row's label is "SERIAL PORT
//! n" and on its own line that name - empty when the file did not name the port, or did not
//! come - then its RTS/CTS note. SITL writes no UART name after `SERIALn` (its lines run
//! `SERIAL1 TX=       0 RX= ...`), so there the second word is the first counter's, "TX=", and
//! that is what the C# shows under every port.
//!
//! Where the reading differs from the C#, and why:
//!
//! * a read that returns null - the file never opened, the plain read did not finish, or Cancel
//!   was pressed - throws in the C#: `ms.Length` on the null `GetFile` returned
//!   (`ConfigSerial.cs:105, 123`). `BackstageView` catches it (`BackstageView.cs:495-503`), so the
//!   page comes up empty that time, and names its ports "SERIAL PORT n" when it is next shown.
//!   Here the table is built at once with those names, as the code means to when the download
//!   fails silently;
//! * Cancel's `kCmdResetSessions()` runs on the C#'s UI thread beside the read it cancelled, and
//!   again when the read acknowledges the cancel, since setting `CancelRequested` raises the
//!   event a second time (`ConfigSerial.cs:88-93, 112-117`;
//!   `ExtLibs/Utilities/IProgressReporterDialogue.cs:37-47`). The link runs one request per
//!   vehicle at a time, so here the reset is sent once, when the read has stopped;
//! * the C#'s 200 ms timer writes the last report back over "Cancelling..." at its next tick
//!   (`ProgressReporterDialogue.cs:313-323`); here "Cancelling..." stays until the window closes;
//! * the finished window's 100 ms pause at 100 % before it closes
//!   (`ProgressReporterDialogue.cs:198-215`) is not kept: it closes when the read ends;
//! * each `ConfigSerial` makes a `MAVFtp` of its own, so its read would run beside any other on
//!   the vehicle; the link has one client per vehicle, and a read it refuses because another is
//!   running counts as a failed one. A page object disposed while its read runs - which the
//!   modal window leaves no way to do but losing the vehicle - leaves the read to end on its own;
//! * `StreamReader`'s byte-order-mark detection is kept for UTF-8 only: ArduPilot writes the file
//!   in ASCII.
//!
//! What is not ported, and why:
//!
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

use std::collections::{BTreeMap, VecDeque};
use std::time::Instant;

use gpui::{AnyElement, Context, Div, FontWeight, SharedString, Window, div, prelude::*, px, rgb};
use mp_link::mavftp::{FtpOutcome, FtpRequest, RW_SIZE};
use mp_link::requests::RequestOutcome;
use mp_link::{FtpError, RequestId};
use mp_vehicle::VehicleId;

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

/// The file `Activate` reads the ports' names from.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:82`
pub const UARTS_FILE: &str = "@SYS/uarts.txt";

/// The progress window's text while the file is read: every report the read makes is shown as
/// this.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:95-98`
pub const UARTS_PROGRESS: &str = "Trying to download uarts.txt\r\nFrom FC";

/// The progress window's text once Cancel is pressed.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:254-268`
pub const CANCELLING: &str = "Cancelling...";

/// The progress window's caption.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx:297-298`
const PROGRESS_TITLE: &str = "Progress";

/// The progress window's client size.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx:291-292`
const PROGRESS_WINDOW: (f32, f32) = (306.0, 144.0);

/// `lblProgressMessage`'s `Location` and `Size`.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx:150-154`
const PROGRESS_LABEL: (f32, f32, f32, f32) = (13.0, 13.0, 275.0, 74.0);

/// `progressBar1`'s.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx:125-129`
const PROGRESS_BAR: (f32, f32, f32, f32) = (11.0, 90.0, 277.0, 13.0);

/// `btnCancel`'s.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.resx:177-187`
const PROGRESS_CANCEL: (f32, f32, f32, f32) = (213.0, 109.0, 75.0, 23.0);

/// `MyProgressBar`'s marquee block, `BarSize` pixels wide.
/// `// C#: ExtLibs/Controls/MyProgressBar.cs:55, 113-114`
const MARQUEE_BLOCK: f32 = 40.0;

/// The marquee's `Value` a time after it started from 0: stepped by 2 every 50 ms up to 100,
/// then from -40 again - 71 places.
/// `// C#: ExtLibs/Controls/MyProgressBar.cs:82-95; MyProgressBar.Designer.cs:37`
fn marquee_value(elapsed: std::time::Duration) -> i32 {
    let step = (elapsed.as_millis() / 50) % 71;
    // 0 is place 20.
    let place = i32::try_from((20 + step) % 71).unwrap_or(0);
    -40 + 2 * place
}

/// `Int32.TryParse(s, out n)` as the .NET Framework reads it (`NumberStyles.Integer`): white
/// space either side, a sign, then decimal digits that fit an `int` - and trailing NULs.
fn try_parse_int32(text: &str) -> Option<i32> {
    // `Number.IsWhite`: the space, and tab to carriage return.
    let white = |c: char| c == ' ' || ('\t'..='\r').contains(&c);
    let text = text
        .trim_end_matches('\0')
        .trim_end_matches(white)
        .trim_start_matches(white);
    let (negative, digits) = match text.as_bytes().first() {
        Some(b'-') => (true, &text[1..]),
        Some(b'+') => (false, &text[1..]),
        _ => (false, text),
    };
    if digits.is_empty() || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    let magnitude = digits.bytes().try_fold(0_i64, |n, byte| {
        n.checked_mul(10)?.checked_add(i64::from(byte - b'0'))
    })?;
    i32::try_from(if negative { -magnitude } else { magnitude }).ok()
}

/// `StreamReader.ReadLine` to the end: a line ends at "\r\n", "\r" or "\n", and an end at the
/// very end starts no line of its own.
fn read_lines(text: &str) -> Vec<&str> {
    let mut lines = Vec::new();
    let mut rest = text;
    while !rest.is_empty() {
        let Some(at) = rest.find(['\r', '\n']) else {
            lines.push(rest);
            break;
        };
        lines.push(&rest[..at]);
        let end = if rest[at..].starts_with("\r\n") { 2 } else { 1 };
        rest = &rest[at + end..];
    }
    lines
}

/// `_uartNames` from the file: for each line of two or more words split at single spaces whose
/// first is `SERIAL` and a number above 0, that number and the second word. A number named a
/// second time ends the reading there, keeping what came before, as `Dictionary.Add` throws
/// into the `catch { }` around the loop.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:123-160`
#[must_use]
pub fn uart_names(file: &[u8]) -> BTreeMap<usize, String> {
    // `new StreamReader(ms)`: UTF-8, its byte-order mark skipped.
    let file = file.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(file);
    let text = String::from_utf8_lossy(file);
    let mut names = BTreeMap::new();
    for line in read_lines(&text) {
        let words: Vec<&str> = line.split(' ').collect();
        let [first, second, ..] = words.as_slice() else {
            continue;
        };
        // `s[0].Length >= 7 && s[0].Substring(0, 6) == "SERIAL"`, then its trailing number.
        let Some(number) = first
            .strip_prefix("SERIAL")
            .filter(|number| !number.is_empty())
        else {
            continue;
        };
        let Some(port) = try_parse_int32(number)
            .filter(|n| *n > 0)
            .and_then(|n| usize::try_from(n).ok())
        else {
            continue;
        };
        if names.contains_key(&port) {
            break;
        }
        names.insert(port, (*second).to_owned());
    }
    names
}

/// The progress window's bar, `progressBar1`, as `timer1_Tick` sets it from the last report.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:313-336`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bar {
    /// `ProgressBarStyle.Marquee`: the last report was -1.
    pub marquee: bool,
    /// `Value` otherwise: the last report's percentage. `MyProgressBar` takes any value.
    pub value: i32,
}

impl Default for Bar {
    /// `_progress` starts at -1, so the first tick makes it a marquee.
    fn default() -> Self {
        Self {
            marquee: true,
            value: 0,
        }
    }
}

impl Bar {
    /// A report: -1 the marquee, anything else the bar at that value.
    pub const fn report(&mut self, percent: i32) {
        self.marquee = percent == -1;
        if !self.marquee {
            self.value = percent;
        }
    }
}

/// Where the ports' names are: `_gotUARTNames`, and the read it starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Uarts {
    /// `_gotUARTNames` false: this page object has not asked.
    #[default]
    Unread,
    /// The file is being read, behind the progress window.
    Reading {
        /// The vehicle asked.
        vehicle: VehicleId,
        /// When, for the marquee.
        started: Instant,
        /// The window's bar.
        bar: Bar,
        /// Cancel was pressed.
        cancelling: bool,
    },
    /// The file came, and `_uartNames` holds what it names - nothing, for an empty file.
    Loaded,
    /// No file: the vehicle refused it or never finished sending it, or there was no vehicle to
    /// ask.
    Failed,
    /// Cancel was pressed, and the file did not come.
    Cancelled,
}

impl Uarts {
    /// The word a fact says it with.
    #[must_use]
    pub const fn word(&self) -> &'static str {
        match self {
            Self::Unread => "unread",
            Self::Reading {
                cancelling: false, ..
            } => "reading",
            Self::Reading {
                cancelling: true, ..
            } => "cancelling",
            Self::Loaded => "loaded",
            Self::Failed => "failed",
            Self::Cancelled => "cancelled",
        }
    }
}

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
    /// `_gotUARTNames`, and where the read it starts has got.
    uarts: Uarts,
    /// `_uartNames`: a port's name, by its number.
    uart_names: BTreeMap<usize, String>,
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

    /// Where the ports' names are.
    #[must_use]
    pub const fn uarts(&self) -> Uarts {
        self.uarts
    }

    /// The name `@SYS/uarts.txt` gave port `port`, if it gave one.
    #[must_use]
    pub fn uart_name(&self, port: usize) -> Option<&str> {
        self.uart_names.get(&port).map(String::as_str)
    }

    /// Every name the file gave, by port.
    #[must_use]
    pub const fn uart_names(&self) -> &BTreeMap<usize, String> {
        &self.uart_names
    }

    /// The progress window, while the names are read: its text, its bar, and whether its Cancel
    /// shows.
    #[must_use]
    pub const fn progress(&self) -> Option<(&'static str, Bar, bool)> {
        match self.uarts {
            Uarts::Reading {
                bar, cancelling, ..
            } => Some(if cancelling {
                (CANCELLING, bar, false)
            } else {
                (UARTS_PROGRESS, bar, true)
            }),
            _ => None,
        }
    }

    /// `Activate`: the rules read the first time; the ports' names asked for the first time,
    /// behind the progress window, the rest waiting for it to close; then the table built afresh
    /// from the vehicle's parameters.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:37-380`
    pub fn activate(
        &mut self,
        telemetry: &Telemetry,
        parameters: &[(String, f64)],
        key: Key,
        lookup: Lookup,
    ) {
        if self.made_for != Some(key) {
            self.dispose();
            self.made_for = Some(key);
        }
        if self.rules.is_none() {
            self.rules = Some(sanitised_rules(&options("SERIAL1_BAUD", lookup)));
        }
        self.active = true;
        self.dropdown = None;
        if self.uarts == Uarts::Unread {
            self.read_uarts(telemetry);
        }
        // `prd.RunBackgroundOperationAsync()` is `ShowDialog`: the rest of `Activate` runs when
        // the window closes, which `tick` sees.
        if matches!(self.uarts, Uarts::Reading { .. }) {
            return;
        }
        self.build(parameters, lookup);
    }

    /// `_gotUARTNames = true`, and `GetFile(@"@SYS/uarts.txt", cancel, false)` started on the
    /// vehicle. With no vehicle to ask, or its client busy, it has failed already.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:78-120`
    fn read_uarts(&mut self, telemetry: &Telemetry) {
        let request = FtpRequest::Get {
            path: UARTS_FILE.to_owned(),
            burst: false,
            readsize: RW_SIZE,
        };
        self.uarts = telemetry
            .start_ftp(request)
            .map_or(Uarts::Failed, |vehicle| Uarts::Reading {
                vehicle,
                started: Instant::now(),
                bar: Bar::default(),
                cancelling: false,
            });
    }

    /// The progress window's Cancel: "Cancelling...", the bar a marquee and the button hidden;
    /// then `CancelRequestChanged`'s `cancel.Cancel()`. Its `kCmdResetSessions()` goes when the
    /// read has stopped; see the module's notes.
    /// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:254-268; GCSViews/ConfigurationView/ConfigSerial.cs:88-93`
    pub fn cancel_uarts(&mut self, telemetry: &Telemetry) {
        if let Uarts::Reading {
            vehicle,
            started,
            bar,
            cancelling: false,
        } = self.uarts
        {
            telemetry.cancel_ftp(vehicle);
            self.uarts = Uarts::Reading {
                vehicle,
                started,
                bar: Bar {
                    marquee: true,
                    ..bar
                },
                cancelling: true,
            };
        }
    }

    /// The read, once a frame: its progress onto the bar - none once Cancel is pressed, as
    /// `UpdateProgressAndStatus` ignores them - and when it has ended, the names from what came,
    /// the window closed, and the rest of `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:95-98, 101-161; ExtLibs/Controls/ProgressReporterDialogue.cs:282-296`
    fn poll_uarts(&mut self, telemetry: &Telemetry, view: &TelemetryView, lookup: Lookup) {
        let Uarts::Reading {
            vehicle,
            started,
            mut bar,
            cancelling,
        } = self.uarts
        else {
            return;
        };
        // The outcome is kept as the request stops, under the same lock: once the client is not
        // busy it is there to take, or it never will be - this link has no such client, or the
        // outcome went to someone else.
        if let Some((true, progress)) = telemetry.ftp_progress(vehicle) {
            if !cancelling {
                bar.report(progress.percent);
            }
            self.uarts = Uarts::Reading {
                vehicle,
                started,
                bar,
                cancelling,
            };
            return;
        }
        let outcome = telemetry.take_ftp_outcome(vehicle);
        if cancelling {
            // `// C#: :88-93`
            telemetry.ftp_on(vehicle, FtpRequest::ResetSessions);
        }
        self.land(outcome, cancelling);
        self.build(&view.parameters, lookup);
    }

    /// What `GetFile` left in `ms`: a stream with bytes in it is read for names; an empty one
    /// names nothing; an exception leaves the empty stream `ms` began as. Null - no file, a read
    /// cut short, a cancel - is where the C# throws; here it names nothing, as a failure does.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:86, 103-111, 123-160`
    fn land(&mut self, outcome: Option<Result<FtpOutcome, FtpError>>, cancelled: bool) {
        self.uarts = match outcome {
            Some(Ok(FtpOutcome::File {
                data: Some(data), ..
            })) => {
                if !data.is_empty() {
                    self.uart_names = uart_names(&data);
                }
                Uarts::Loaded
            }
            _ if cancelled => Uarts::Cancelled,
            _ => Uarts::Failed,
        };
    }

    /// The rest of `Activate`: the table built afresh from the vehicle's parameters - a row per
    /// port up to the highest, named from `_uartNames`, the note under them. A port with no
    /// `SERIALn_BAUD` gets its name only; a port with no `SERIALn_OPTIONS` stops the building
    /// there, with no note, as the C#'s exception does.
    /// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:163-380`
    fn build(&mut self, parameters: &[(String, f64)], lookup: Lookup) {
        self.rows.clear();
        self.note = None;
        self.ports = port_count(parameters);
        if self.ports == 0 {
            return;
        }
        for port in 1..=self.ports {
            let name = format!("SERIAL{port}");
            // `// C#: :188-209`
            let mut uart = self.uart_name(port).unwrap_or_default().to_owned();
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

    /// The page object disposed with its screen. Sets already asked for still go; so does a read
    /// of the names, whose end nobody takes. The next page object asks again.
    pub fn dispose(&mut self) {
        self.made_for = None;
        self.active = false;
        self.rules = None;
        self.uarts = Uarts::Unread;
        self.uart_names.clear();
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

    /// Once a frame: a page object whose screen has gone is disposed, the names' read followed,
    /// the set on its way is read back and what follows it done, and the next is sent.
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
        self.poll_uarts(telemetry, view, lookup);
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
    // The names: where the read is, how many the file gave, and the progress window.
    record("config.serial.uarts", serial.uarts().word());
    record("config.serial.uarts.names", serial.uart_names().len());
    let progress = serial.progress();
    record(
        "config.serial.progress",
        progress.map_or_else(|| "none".to_owned(), |(text, ..)| text.replace("\r\n", " ")),
    );
    record(
        "config.serial.progress.bar",
        progress.map_or_else(
            || "none".to_owned(),
            |(_, bar, _)| {
                if bar.marquee {
                    "marquee".to_owned()
                } else {
                    bar.value.to_string()
                }
            },
        ),
    );
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
        record(
            format!("config.serial.{n}.name"),
            serial.uart_name(n).unwrap_or("none"),
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

/// The progress window while the names are read: `ProgressReporterDialogue`, shown with
/// `ShowDialog`, so nothing behind it takes a click.
/// `// C#: GCSViews/ConfigurationView/ConfigSerial.cs:84-120`
fn progress_window(
    serial: &SerialPorts,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let (text, bar, cancel) = serial.progress()?;
    let Uarts::Reading { started, .. } = serial.uarts() else {
        return None;
    };
    let on_cancel = cancel.then_some(|this: &mut MissionPlanner| {
        this.serial_ports.cancel_uarts(&this.telemetry);
    });
    Some(progress_dialog(
        ProgressIds {
            frame: "serial-uarts-progress",
            bar: "serial-uarts-bar",
            cancel: "serial-uarts-cancel",
            backdrop: "serial-uarts-backdrop",
        },
        text,
        bar,
        started,
        on_cancel,
        window,
        cx,
    ))
}

/// The ids of a progress window's parts.
#[derive(Debug, Clone, Copy)]
pub struct ProgressIds {
    /// The window.
    pub frame: &'static str,
    /// Its bar.
    pub bar: &'static str,
    /// Its Cancel.
    pub cancel: &'static str,
    /// What is under it, which takes the clicks meant for the page.
    pub backdrop: &'static str,
}

/// A `ProgressReporterDialogue`, shown with `ShowDialog`, so nothing behind it takes a click. No
/// close box (`ControlBox = false`); its caption, label, bar and - while `on_cancel` is given -
/// Cancel where the `.resx` puts them; centred, as `CenterParent` centres it. The Serial Ports
/// page's, and the MAVFtp page's.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:49-58; ProgressReporterDialogue.designer.cs:34-116; MyProgressBar.cs:98-125`
#[allow(clippy::too_many_arguments)]
pub fn progress_dialog(
    ids: ProgressIds,
    text: &str,
    bar: Bar,
    started: Instant,
    on_cancel: Option<impl Fn(&mut MissionPlanner) + 'static>,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (bar_x, bar_y, bar_w, bar_h) = PROGRESS_BAR;
    // `position = Value / 100 * Width`: the bar done from the left, or the marquee's block there.
    #[allow(clippy::cast_precision_loss)] // a percentage
    let position = |value: i32| value as f32 / 100.0 * bar_w;
    let (fill_x, fill_w) = if bar.marquee {
        (position(marquee_value(started.elapsed())), MARQUEE_BLOCK)
    } else {
        (0.0, position(bar.value).max(0.0))
    };
    let (label_x, label_y, label_w, label_h) = PROGRESS_LABEL;
    let mut label = div()
        .absolute()
        .left(px(label_x))
        .top(px(label_y))
        .w(px(label_w))
        .h(px(label_h))
        .flex()
        .flex_col()
        .text_xs()
        .text_color(rgb(theme::TEXT));
    for line in text.lines() {
        label = label.child(line.to_owned());
    }
    let mut client = div()
        .relative()
        .w(px(PROGRESS_WINDOW.0))
        .h(px(PROGRESS_WINDOW.1))
        .child(label)
        .child(
            crate::probe::measured(ids.bar, div())
                .absolute()
                .left(px(bar_x))
                .top(px(bar_y))
                .w(px(bar_w))
                .h(px(bar_h))
                .overflow_hidden()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::BG))
                .child(
                    div()
                        .absolute()
                        .left(px(fill_x))
                        .top(px(0.0))
                        .w(px(fill_w))
                        .h_full()
                        .bg(rgb(theme::OK)),
                ),
        );
    if let Some(on_cancel) = on_cancel {
        let (x, y, w, h) = PROGRESS_CANCEL;
        client = client.child(
            div()
                .absolute()
                .left(px(x))
                .top(px(y))
                .w(px(w))
                .h(px(h))
                .child(
                    crate::probe::measured(ids.cancel, div())
                        .id(ids.cancel)
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
                        .child("Cancel")
                        .on_click(cx.listener(move |this, _event, _window, cx| {
                            on_cancel(this);
                            cx.notify();
                        })),
                ),
        );
    }
    let frame = crate::probe::measured(ids.frame, div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .child(
            div()
                .px_2()
                .py_1()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(PROGRESS_TITLE),
        )
        .child(client);
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id(ids.backdrop)
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(frame),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// The bitmask windows open, over everything the message box showing, and while the names are
/// read the progress window.
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
    out.extend(progress_window(serial, window, cx));
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
    use mp_link::mavftp::testing::FakeVehicle;
    use mp_link::mavftp::wire::{Errno, ErrorCode, Header, Opcode};
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
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
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
        serial.activate(&Telemetry::idle(), &sitl(), key(), documented);
        assert!(serial.is_active());
        // No vehicle to ask for the names: failed at once, the table built without them.
        assert_eq!(serial.uarts(), Uarts::Failed);
        assert!(serial.progress().is_none());
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
        serial.activate(&Telemetry::idle(), &sitl(), key(), bundled);
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
            &Telemetry::idle(),
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
            &Telemetry::idle(),
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
            &Telemetry::idle(),
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
        serial.activate(&Telemetry::idle(), &view.parameters, key(), documented);
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
        serial.activate(&Telemetry::idle(), &view.parameters, key(), documented);
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
            &Telemetry::idle(),
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
        serial.activate(&Telemetry::idle(), &sitl(), key(), documented);
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
        serial.activate(&Telemetry::idle(), &parameters, key(), documented);
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
        serial.activate(&Telemetry::idle(), &sitl(), Key::of(&view), documented);
        serial.open_bitmask(0, &sitl(), documented);
        serial.hide();
        serial.tick(&telemetry, &view, true, documented);
        assert_eq!(serial.dialogs().len(), 1, "kept while the screen shows");
        serial.tick(&telemetry, &view, false, documented);
        assert!(serial.dialogs().is_empty());
        assert!(serial.rows().is_empty());
    }

    /// `@SYS/uarts.txt` as SITL copter serves it, read with `headless-planner ftp get tcp:127.0.0.1:5763
    /// @SYS/uarts.txt` on 2026-09-24: 712 bytes. SITL's UARTs have no names, so each line's
    /// second word is "TX=".
    const SITL_UARTS: &str = "UARTV1\n\
        SERIAL0 TX=       0 RX=       0 TXBD=     0 RXBD=     0 not connected (tcp:0:wait)\n\
        SERIAL1 TX=       0 RX=       0 TXBD=     0 RXBD=     0 not connected (tcp:2)\n\
        SERIAL2 TX=   65489 RX=     515 TXBD=   244 RXBD=     1 connected     (tcp:3)\n\
        SERIAL3 TX=  498700 RX= 5702436 TXBD=   258 RXBD=   444 connected     (GPS1)\n\
        SERIAL4 TX=       0 RX=       0 TXBD=     0 RXBD=     0 connected     (GPS2)\n\
        SERIAL5 TX=       0 RX=       0 TXBD=     0 RXBD=     0 not connected (tcp:5)\n\
        SERIAL6 TX=       0 RX=       0 TXBD=     0 RXBD=     0 not connected (tcp:6)\n\
        SERIAL7 TX=       0 RX=       0 TXBD=     0 RXBD=     0 not connected (tcp:7)\n\
        SERIAL8 TX=       0 RX=       0 TXBD=     0 RXBD=     0 not connected (tcp:8)\n";

    /// The link's own address, which the vehicle's FTP replies are addressed to.
    const GCS: VehicleId = VehicleId::new(255, 190);

    /// The vehicle's side of MAVFTP: every `FILE_TRANSFER_PROTOCOL` the link has sent answered
    /// from `files`, as ArduPilot's `GCS_FTP.cpp` answers it - but for the answers `lost` picks.
    fn serve(vehicle: &mut Vehicle, files: &mut FakeVehicle, lost: impl Fn(&Header) -> bool) {
        for message in vehicle.read() {
            if let MavMessage::FileTransferProtocol(ftp) = message {
                let request = Header::decode(&ftp.payload);
                for reply in files.answer(&request) {
                    if !lost(&request) {
                        vehicle.send(&mp_link::ftp::ftp_message(GCS, &reply));
                    }
                }
            }
        }
    }

    /// The opcodes the vehicle heard.
    fn opcodes(files: &FakeVehicle) -> Vec<Opcode> {
        files.heard.iter().map(|head| head.opcode).collect()
    }

    /// The page activated on a vehicle `files` answers for, with SITL's parameters, and ticked
    /// until the names' read has ended.
    fn read_through(
        files: &mut FakeVehicle,
        parameters: Vec<(String, f64)>,
    ) -> (SerialPorts, Telemetry, Vehicle) {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut view = telemetry.view();
        view.parameters = parameters.into();
        let mut serial = SerialPorts::default();
        serial.activate(&telemetry, &view.parameters, Key::of(&view), documented);
        until("the read to end", || {
            serve(&mut vehicle, files, |_| false);
            serial.tick(&telemetry, &view, true, documented);
            !matches!(serial.uarts(), Uarts::Reading { .. })
        });
        (serial, telemetry, vehicle)
    }

    #[test]
    fn a_number_is_read_as_int32_try_parse_reads_it() {
        assert_eq!(try_parse_int32("1"), Some(1));
        assert_eq!(try_parse_int32("01"), Some(1));
        assert_eq!(try_parse_int32("+2"), Some(2));
        assert_eq!(try_parse_int32("-3"), Some(-3));
        assert_eq!(try_parse_int32(" \t4\r "), Some(4));
        assert_eq!(try_parse_int32("5 \0\0"), Some(5), "trailing NULs");
        assert_eq!(try_parse_int32("6\0 "), None);
        assert_eq!(try_parse_int32("2147483647"), Some(i32::MAX));
        assert_eq!(try_parse_int32("2147483648"), None);
        assert_eq!(try_parse_int32("-2147483648"), Some(i32::MIN));
        for bad in ["", "+", "- 1", "1a", "0x1", "1.0", "１"] {
            assert_eq!(try_parse_int32(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn lines_end_as_read_line_ends_them() {
        assert_eq!(read_lines("a\r\nb\rc\nd"), ["a", "b", "c", "d"]);
        assert_eq!(read_lines("a\n\nb\n"), ["a", "", "b"]);
        assert!(read_lines("").is_empty());
        assert_eq!(read_lines("\n"), [""]);
    }

    /// SITL names each port "TX=", from SERIAL1 to SERIAL8; SERIAL0 is not a row's.
    #[test]
    fn sitls_file_names_each_port_tx() {
        let names = uart_names(SITL_UARTS.as_bytes());
        assert_eq!(SITL_UARTS.len(), 712);
        assert_eq!(
            names.keys().copied().collect::<Vec<_>>(),
            (1..=8).collect::<Vec<_>>()
        );
        assert!(names.values().all(|name| name == "TX="));
    }

    #[test]
    fn a_name_is_the_second_word_of_a_serial_line() {
        let file = "\u{feff}UARTV1\r\n\
            SERIAL0 OTG1 TX=0\r\n\
            SERIAL3 UART7 TX=0\r\
            SERIAL1 USART2\n\
            SERIAL x\n\
            SERIALx UART1\n\
            SERIAL-2 UART2\n\
            SERIAL+2 UART4 TX=0\n\
            SERIAL5\n\
            serial6 UART6\n\
            SERIAL99999999999 UART9\n\
            SERIAL04  UART8\n\
            SERIAL7\t UART5\n";
        let names = uart_names(file.as_bytes());
        let expected: BTreeMap<usize, String> = [
            (1, "USART2"),
            (2, "UART4"),
            (3, "UART7"),
            // Two spaces: `Split(' ')` makes the second word empty.
            (4, ""),
            // `TryParse` takes the tab as trailing white space.
            (7, "UART5"),
        ]
        .into_iter()
        .map(|(port, name)| (port, name.to_owned()))
        .collect();
        assert_eq!(names, expected);
    }

    /// `Dictionary.Add` throws on a port named twice, and the `catch` is around the loop.
    #[test]
    fn a_port_named_twice_ends_the_reading() {
        let names = uart_names(b"SERIAL1 A\nSERIAL2 B\nSERIAL1 C\nSERIAL3 D\n");
        assert_eq!(names.len(), 2);
        assert_eq!(names.get(&1).map(String::as_str), Some("A"));
        assert!(!names.contains_key(&3));
    }

    #[test]
    fn the_bar_follows_the_reports() {
        let mut bar = Bar::default();
        assert!(bar.marquee, "_progress starts at -1");
        bar.report(0);
        assert_eq!(
            bar,
            Bar {
                marquee: false,
                value: 0
            }
        );
        bar.report(56);
        assert_eq!(
            bar,
            Bar {
                marquee: false,
                value: 56
            }
        );
        bar.report(-1);
        assert_eq!(
            bar,
            Bar {
                marquee: true,
                value: 56
            }
        );
        // The marquee: 0, stepping 2 each 50 ms to 100, then -40.
        let at = |ms| marquee_value(std::time::Duration::from_millis(ms));
        assert_eq!(at(0), 0);
        assert_eq!(at(49), 0);
        assert_eq!(at(50), 2);
        assert_eq!(at(2500), 100);
        assert_eq!(at(2550), -40);
        assert_eq!(at(3550), 0, "71 places round");
    }

    /// The path the product takes: the page's first `Activate` asks the vehicle for
    /// `@SYS/uarts.txt` with a plain read of 80-byte chunks, shows the progress window with no
    /// table behind it, and when the file has come names each row from it - SITL's "TX=".
    #[test]
    fn the_first_activate_reads_the_names_and_then_builds_the_table() {
        let mut files = FakeVehicle::new().with_file(UARTS_FILE, SITL_UARTS.as_bytes());
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut view = telemetry.view();
        let mut parameters = sitl();
        for (name, value) in &mut parameters {
            if name == "BRD_SER2_RTSCTS" {
                *value = 2.0;
            }
        }
        view.parameters = parameters.into();
        let mut serial = SerialPorts::default();
        serial.activate(&telemetry, &view.parameters, Key::of(&view), documented);
        assert_eq!(serial.uarts().word(), "reading");
        assert!(serial.rows().is_empty(), "the table waits for the window");
        assert!(serial.note().is_none());
        assert_eq!(
            serial.progress(),
            Some((UARTS_PROGRESS, Bar::default(), true))
        );
        until("the names", || {
            serve(&mut vehicle, &mut files, |_| false);
            serial.tick(&telemetry, &view, true, documented);
            serial.uarts() == Uarts::Loaded
        });
        assert!(serial.progress().is_none(), "the window closed");
        assert_eq!(serial.uart_names().len(), 8);
        assert_eq!(serial.rows().len(), 7);
        assert_eq!(serial.rows()[0].label, "SERIAL PORT 1\nTX=");
        assert_eq!(serial.rows()[1].label, "SERIAL PORT 2\nTX= (RTS/CTS Auto)");
        assert_eq!(serial.rows()[6].label, "SERIAL PORT 7\nTX=");
        assert_eq!(serial.uart_name(7), Some("TX="));
        assert_eq!(serial.note(), Some(REBOOT_NOTE));
        // `GetFile(filename, cancel, false)`: reset, open, then plain reads of RW_SIZE.
        let heard = opcodes(&files);
        assert_eq!(
            heard[..3],
            [
                Opcode::RESET_SESSIONS,
                Opcode::OPEN_FILE_RO,
                Opcode::READ_FILE
            ]
        );
        assert!(!heard.contains(&Opcode::BURST_READ_FILE));
        let open = &files.heard[1];
        assert_eq!(&open.data[..usize::from(open.size)], UARTS_FILE.as_bytes());
        assert!(
            files
                .heard
                .iter()
                .filter(|head| head.opcode == Opcode::READ_FILE)
                .all(|head| head.size == RW_SIZE)
        );
    }

    /// `_gotUARTNames`: the page object asks once, however often it is shown; a new one - the
    /// screen shown again - asks again.
    #[test]
    fn the_names_are_asked_for_once_per_page_object() {
        let mut files = FakeVehicle::new().with_file(UARTS_FILE, SITL_UARTS.as_bytes());
        let (mut serial, telemetry, mut vehicle) = read_through(&mut files, sitl());
        let opens = |files: &FakeVehicle| {
            opcodes(files)
                .iter()
                .filter(|opcode| **opcode == Opcode::OPEN_FILE_RO)
                .count()
        };
        assert_eq!(opens(&files), 1);
        let mut view = telemetry.view();
        view.parameters = sitl().into();
        serial.hide();
        serial.activate(&telemetry, &view.parameters, Key::of(&view), documented);
        assert_eq!(serial.uarts(), Uarts::Loaded);
        assert_eq!(
            serial.rows()[0].label,
            "SERIAL PORT 1\nTX=",
            "built at once"
        );
        serve(&mut vehicle, &mut files, |_| false);
        assert_eq!(opens(&files), 1);
        // The screen left and shown again: a new page object.
        serial.hide();
        serial.tick(&telemetry, &view, false, documented);
        assert_eq!(serial.uarts(), Uarts::Unread);
        assert!(serial.uart_names().is_empty());
        serial.activate(&telemetry, &view.parameters, Key::of(&view), documented);
        until("the second read", || {
            serve(&mut vehicle, &mut files, |_| false);
            serial.tick(&telemetry, &view, true, documented);
            serial.uarts() == Uarts::Loaded
        });
        assert_eq!(opens(&files), 2);
    }

    /// A file the vehicle does not have: `GetFile` throws, the C# fails silently, and each port
    /// is "SERIAL PORT n" alone.
    #[test]
    fn a_file_the_vehicle_refuses_leaves_the_ports_unnamed() {
        let mut files = FakeVehicle::new();
        let (serial, ..) = read_through(&mut files, sitl());
        assert_eq!(serial.uarts(), Uarts::Failed);
        assert!(serial.uart_names().is_empty());
        assert_eq!(serial.rows().len(), 7);
        assert_eq!(serial.rows()[0].label, "SERIAL PORT 1\n");
        assert_eq!(serial.note(), Some(REBOOT_NOTE));
        assert!(serial.message().is_none(), "silently");
    }

    /// An open that is never answered: `GetFile` returns null, where the C# throws at
    /// `ms.Length`; here the table is built with the ports unnamed.
    #[test]
    fn a_file_that_never_comes_leaves_the_ports_unnamed() {
        let mut files = FakeVehicle::new().with_file(UARTS_FILE, SITL_UARTS.as_bytes());
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut view = telemetry.view();
        view.parameters = sitl().into();
        let mut serial = SerialPorts::default();
        serial.activate(&telemetry, &view.parameters, Key::of(&view), documented);
        until("the read to give up", || {
            serve(&mut vehicle, &mut files, |request| {
                request.opcode == Opcode::OPEN_FILE_RO
            });
            serial.tick(&telemetry, &view, true, documented);
            !matches!(serial.uarts(), Uarts::Reading { .. })
        });
        assert_eq!(serial.uarts(), Uarts::Failed);
        assert_eq!(serial.rows().len(), 7);
        assert_eq!(serial.rows()[3].label, "SERIAL PORT 4\n");
        assert_eq!(serial.note(), Some(REBOOT_NOTE));
    }

    /// An empty file: the read ends on end of file with nothing in the stream, `ms.Length` is
    /// 0, and nothing is read from it.
    #[test]
    fn an_empty_file_names_nothing() {
        let mut files = FakeVehicle::new().with_file(UARTS_FILE, b"");
        let (serial, ..) = read_through(&mut files, sitl());
        assert_eq!(serial.uarts(), Uarts::Loaded);
        assert!(serial.uart_names().is_empty());
        assert_eq!(serial.rows()[0].label, "SERIAL PORT 1\n");
    }

    /// Cancel mid-read: the window says "Cancelling..." with its bar a marquee and its button
    /// gone, the read stops, the sessions are reset, and the table is built unnamed.
    #[test]
    fn cancel_stops_the_read_and_resets_the_sessions() {
        let mut files = FakeVehicle::new().with_file(UARTS_FILE, SITL_UARTS.as_bytes());
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut view = telemetry.view();
        view.parameters = sitl().into();
        let mut serial = SerialPorts::default();
        serial.activate(&telemetry, &view.parameters, Key::of(&view), documented);
        // The first chunk answered, the rest lost: the bar leaves the marquee for 0 %.
        let first_read = std::cell::Cell::new(true);
        until("the first chunk", || {
            serve(&mut vehicle, &mut files, |request| {
                request.opcode == Opcode::READ_FILE && !first_read.replace(false)
            });
            serial.tick(&telemetry, &view, true, documented);
            serial.progress().is_some_and(|(_, bar, _)| !bar.marquee)
        });
        serial.cancel_uarts(&telemetry);
        let Some((text, bar, cancel)) = serial.progress() else {
            panic!("the window closed at once");
        };
        assert_eq!(text, CANCELLING);
        assert!(bar.marquee);
        assert!(!cancel, "the Cancel button is hidden");
        assert_eq!(serial.uarts().word(), "cancelling");
        let before = files.heard.len();
        until("the read to stop", || {
            serve(&mut vehicle, &mut files, |request| {
                request.opcode == Opcode::READ_FILE
            });
            serial.tick(&telemetry, &view, true, documented);
            serial.uarts() == Uarts::Cancelled
        });
        assert!(serial.progress().is_none());
        assert_eq!(serial.rows().len(), 7);
        assert_eq!(serial.rows()[0].label, "SERIAL PORT 1\n");
        until("the sessions reset", || {
            serve(&mut vehicle, &mut files, |_| false);
            files.heard[before..]
                .iter()
                .any(|head| head.opcode == Opcode::RESET_SESSIONS)
        });
    }

    /// A refused NAK other than "not found" fails the same way.
    #[test]
    fn a_refused_open_fails_silently() {
        let mut files = FakeVehicle::new().with_file(UARTS_FILE, SITL_UARTS.as_bytes());
        files.refuse = Some((Opcode::OPEN_FILE_RO, ErrorCode::FAIL, Errno(0)));
        let (serial, ..) = read_through(&mut files, sitl());
        assert_eq!(serial.uarts(), Uarts::Failed);
        assert!(serial.message().is_none());
        assert_eq!(serial.rows()[0].label, "SERIAL PORT 1\n");
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
