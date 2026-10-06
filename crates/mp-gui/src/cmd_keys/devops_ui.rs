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

//! Ctrl+J's DevOps window: `Controls/DevopsUI.cs`, which `MainV2.ProcessCmdKey` opens with
//! `new DevopsUI().ShowUserControl()` (`MainV2.cs:4202-4207`) - the control in a form of its own
//! size, 604 x 265, captioned with its `Text`, which nothing sets. It reads a device's registers
//! over MAVLink: `DEVICE_OP_READ`, and `DEVICE_OP_WRITE` for its test.
//!
//! What it shows, as the Designer places it (`DevopsUI.Designer.cs:29-273`): a flow panel at
//! (3, 3) of sysid and compid (1 each), the bus type (`DomainUpDown`, I2C or SPI, "SPI" to start)
//! and the SPI device's name ("icm20948_ext"), then bus, address, regstart (255, up to 255) and
//! count; Do It at the right; under the panel a text box, many lines, that the answers are added
//! to; test beside it.
//!
//! What it does:
//!
//! * Do It (`but_doit_Click`, `DevopsUI.cs:21-33`): `device_op` with the boxes' numbers, SPI when
//!   the bus type reads "SPI" and I2C otherwise; the bytes read, in hex, two digits each, as a
//!   line - or "No Response - " and the result;
//! * the bus type changed (`dom_bustype_SelectedItemChanged`, `:34-50`): SPI enables the name and
//!   disables the bus and the address; I2C the other way round;
//! * test (`but_test_Click`, `:52-62`): to sysid 1 compid 1 on SPI, the name's device, register
//!   0xff: two bytes written, 0x72 and 0x00, then two read, and those added as a line;
//! * `device_op` (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1014-1113`): `DEVICE_OP_READ`, or
//!   `DEVICE_OP_WRITE` with bytes to write, to the system and component given - the request
//!   numbered by a counter of the link's, the name its UTF-8 bytes padded to 40 - and up to a
//!   second for that component's `DEVICE_OP_READ_REPLY` or `DEVICE_OP_WRITE_REPLY`; the first to
//!   come ends the wait, a read's bringing its bytes (`count` of them) and either its result.
//!
//! Where this is not the C#, each at its site:
//!
//! * the second's wait runs with the window running, the two buttons dimmed meanwhile, where the
//!   C#'s handler holds the window;
//! * what the C# throws - test's bytes joined when none came, `Aggregate`'s "Sequence contains no
//!   elements" - goes on the status line, by the owner's ruling of 2026-09-25;
//! * the bus type's arrows step through its two items as `DomainUpDown` does from no item
//!   chosen: the Designer sets its text, "SPI", and chooses none; typing into it is not ported;
//! * the form is drawn over the window, modal, as the other windows here are; a second Ctrl+J
//!   opens a fresh one in its place, and closing it abandons an answer still awaited;
//! * the colours are this application's.
//!
//! `// C#: Controls/DevopsUI.cs:1-65; Controls/DevopsUI.Designer.cs:29-298`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::{Arc, Mutex};

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::inspector::PacketSubscription;
use mp_mavlink_dialects::all::{DeviceOpRead, DeviceOpWrite, MavMessage};
use mp_os::Lock as _;
use web_time::{Duration, Instant};

use crate::MissionPlanner;
use crate::config::motor_test::NumericUpDown;
use crate::config::optional::{at, button, label};
use crate::telemetry::Telemetry;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// The control's `Size`, which `ShowUserControl` gives its form.
/// `// C#: Controls/DevopsUI.Designer.cs:262; Utilities/ExtensionsMP.cs:110-131`
pub const SIZE: (f32, f32) = (604.0, 265.0);

/// The form's caption: the control's `Text`, which its Designer leaves empty.
/// `// C#: Utilities/ExtensionsMP.cs:114`
pub const FORM_TEXT: &str = "";

/// `flowLayoutPanel1.Location`: the panel's controls are placed from it.
/// `// C#: Controls/DevopsUI.Designer.cs:110`
const FLOW_AT: (f32, f32) = (3.0, 3.0);

/// The panel's labels: their names, `Text`s and `Location`s in it.
/// `// C#: Controls/DevopsUI.Designer.cs:115-226`
pub const LABELS: [(&str, &str, (f32, f32)); 7] = [
    ("label1", "sysid", (3.0, 0.0)),
    ("label2", "compid", (107.0, 0.0)),
    ("label3", "bus type", (222.0, 0.0)),
    ("label4", "bus", (3.0, 26.0)),
    ("label5", "address", (101.0, 26.0)),
    ("label6", "regstart", (219.0, 26.0)),
    ("label7", "count", (335.0, 26.0)),
];

/// `dom_bustype`'s place in the panel, its items, and its `Text`.
/// `// C#: Controls/DevopsUI.Designer.cs:142-151`
const BUSTYPE_AT: (f32, f32, f32, f32) = (275.0, 3.0, 62.0, 20.0);
pub const BUS_TYPES: [&str; 2] = ["I2C", "SPI"];
pub const SPI: &str = "SPI";

/// `txt_spiname`'s place in the panel and its `Text`.
/// `// C#: Controls/DevopsUI.Designer.cs:153-159`
const SPINAME_AT: (f32, f32, f32, f32) = (343.0, 3.0, 100.0, 20.0);
pub const SPINAME: &str = "icm20948_ext";

/// `but_doit`, `textBox1` and `but_test`, on the control.
/// `// C#: Controls/DevopsUI.Designer.cs:59-67, 235-251`
const DOIT_AT: (f32, f32, f32, f32) = (526.0, 6.0, 75.0, 23.0);
pub const DOIT: &str = "Do It";
const OUTPUT_AT: (f32, f32, f32, f32) = (3.0, 67.0, 465.0, 144.0);
const TEST_AT: (f32, f32, f32, f32) = (513.0, 76.0, 75.0, 23.0);
pub const TEST: &str = "test";

/// `device_op`'s wait: a second from sending. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1103-1107`
pub const WAIT: Duration = Duration::from_secs(1);

/// `MAVLink.DEVICE_OP_BUSTYPE`. `// C#: ExtLibs/Mavlink/Mavlink.cs (DEVICE_OP_BUSTYPE)`
pub const I2C_BUS: u8 = 0;
pub const SPI_BUS: u8 = 1;

/// `busname`'s length. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1077, 1095`
const BUSNAME_LENGTH: usize = 40;

/// What `Aggregate` throws over no bytes: test's read that brought none.
/// `// C#: Controls/DevopsUI.cs:62`
pub const NO_ELEMENTS: &str = "Sequence contains no elements";

/// test's write and read: to sysid 1 compid 1, bus 0, address 0, register 0xff, two bytes, and
/// the two written. `// C#: Controls/DevopsUI.cs:57-59`
const TEST_TARGET: (u8, u8) = (1, 1);
const TEST_REGISTER: u8 = 0xff;
const TEST_COUNT: u8 = 2;
const TEST_WRITE: [u8; 2] = [0x72, 0x00];

/// The panel's six numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Num {
    /// `num_sysid`.
    SysId,
    /// `num_compid`.
    CompId,
    /// `num_busno`.
    BusNo,
    /// `num_address`.
    Address,
    /// `num_regstart`.
    RegStart,
    /// `num_count`.
    Count,
}

impl Num {
    /// In the panel's order.
    pub const ALL: [Self; 6] = [
        Self::SysId,
        Self::CompId,
        Self::BusNo,
        Self::Address,
        Self::RegStart,
        Self::Count,
    ];

    /// The probe id: `devops-` and the Designer's name.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::SysId => "devops-num_sysid",
            Self::CompId => "devops-num_compid",
            Self::BusNo => "devops-num_busno",
            Self::Address => "devops-num_address",
            Self::RegStart => "devops-num_regstart",
            Self::Count => "devops-num_count",
        }
    }

    /// The fact's name.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::SysId => "sysid",
            Self::CompId => "compid",
            Self::BusNo => "busno",
            Self::Address => "address",
            Self::RegStart => "regstart",
            Self::Count => "count",
        }
    }

    /// Its place in the panel, 62 by 20.
    /// `// C#: Controls/DevopsUI.Designer.cs:71-91, 170-233`
    const fn place(self) -> (f32, f32) {
        match self {
            Self::SysId => (39.0, 3.0),
            Self::CompId => (154.0, 3.0),
            Self::BusNo => (33.0, 29.0),
            Self::Address => (151.0, 29.0),
            Self::RegStart => (267.0, 29.0),
            Self::Count => (375.0, 29.0),
        }
    }

    /// `Value`, `Minimum` and `Maximum` as the Designer leaves them: `NumericUpDown`'s 0 and 100
    /// but for the two set to 1 and regstart's 255 of 255 - and the system id's maximum
    /// `uint.MaxValue`, as the constructor sets it since the C#'s 32-bit system ids (e6454ccdd).
    /// `// C#: Controls/DevopsUI.Designer.cs:75-91, 205-217; Controls/DevopsUI.cs:18`
    const fn designer(self) -> (f64, f64, f64) {
        match self {
            Self::SysId => (1.0, 0.0, u32::MAX as f64),
            Self::CompId => (1.0, 0.0, 100.0),
            Self::BusNo | Self::Address | Self::Count => (0.0, 0.0, 100.0),
            Self::RegStart => (255.0, 0.0, 255.0),
        }
    }
}

/// Which box has the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    /// One of the numbers.
    Num(Num),
    /// `txt_spiname`.
    SpiName,
}

/// `dom_bustype`: its text, and the item chosen - none until an arrow chooses one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Domain {
    /// `Text`.
    pub text: String,
    /// `SelectedIndex`.
    pub index: Option<usize>,
}

impl Domain {
    /// The up arrow: the item before the one chosen, if there is one (`Wrap` is false); none
    /// chosen, nothing. Whether an item was chosen.
    pub fn up(&mut self) -> bool {
        match self.index {
            Some(index) if index > 0 => self.select(index - 1),
            _ => false,
        }
    }

    /// The down arrow: the item after the one chosen, the first when none is, if there is one.
    pub fn down(&mut self) -> bool {
        let next = self.index.map_or(0, |index| index + 1);
        next < BUS_TYPES.len() && self.select(next)
    }

    fn select(&mut self, index: usize) -> bool {
        let Some(item) = BUS_TYPES.get(index) else {
            return false;
        };
        self.index = Some(index);
        (*item).clone_into(&mut self.text);
        true
    }
}

/// One `device_op`'s arguments.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceOp {
    /// The system and component asked: the system 32-bit, `(uint)num_sysid.Value`.
    pub sysid: u32,
    pub compid: u8,
    /// `DEVICE_OP_BUSTYPE`.
    pub bustype: u8,
    /// The device's name, for SPI.
    pub name: String,
    pub bus: u8,
    pub address: u8,
    pub regstart: u8,
    pub count: u8,
    /// The bytes to write: a `DEVICE_OP_WRITE` rather than a read.
    pub write: Option<Vec<u8>>,
}

/// `name.MakeBytesSize(40)`: the UTF-8 bytes, cut or padded with zeros to 40.
/// `// C#: ExtLibs/Utilities/Extensions.cs:520-528`
#[must_use]
pub fn busname(name: &str) -> [u8; BUSNAME_LENGTH] {
    let mut bytes = [0u8; BUSNAME_LENGTH];
    for (to, from) in bytes.iter_mut().zip(name.bytes()) {
        *to = from;
    }
    bytes
}

/// The message `device_op` sends: a write when there are bytes to write - `count` being how
/// many, the bytes padded to 128 - else a read.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1068-1101`
#[must_use]
pub fn message(op: &DeviceOp, request_id: u32) -> MavMessage {
    match &op.write {
        Some(bytes) => {
            let mut data = [0u8; 128];
            for (to, from) in data.iter_mut().zip(bytes) {
                *to = *from;
            }
            MavMessage::DeviceOpWrite(DeviceOpWrite {
                request_id,
                target_system: mp_vehicle::VehicleId::new(op.sysid, op.compid).payload_target(),
                target_component: op.compid,
                bustype: op.bustype,
                bus: op.bus,
                address: op.address,
                busname: busname(&op.name),
                regstart: op.regstart,
                count: u8::try_from(bytes.len()).unwrap_or(u8::MAX),
                data,
                bank: 0,
            })
        }
        None => MavMessage::DeviceOpRead(DeviceOpRead {
            request_id,
            target_system: mp_vehicle::VehicleId::new(op.sysid, op.compid).payload_target(),
            target_component: op.compid,
            bustype: op.bustype,
            bus: op.bus,
            address: op.address,
            busname: busname(&op.name),
            regstart: op.regstart,
            count: op.count,
            bank: 0,
        }),
    }
}

/// `buffer.Select(a => a.ToString("X2")).Aggregate((a, b) => a + b)`.
/// `// C#: Controls/DevopsUI.cs:30, 62`
#[must_use]
pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02X}")).collect()
}

/// What answered: a read's bytes, and the result.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Answer {
    pub data: Vec<u8>,
    pub result: u8,
}

/// Which press an operation is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Do It's read.
    DoIt,
    /// test's write.
    TestWrite,
    /// test's read.
    TestRead,
}

impl Step {
    /// The word a fact carries.
    #[must_use]
    pub const fn key(self) -> &'static str {
        match self {
            Self::DoIt => "doit",
            Self::TestWrite => "test-write",
            Self::TestRead => "test-read",
        }
    }
}

/// A `device_op` under way: what it is for, when it was sent, the first answer, and the
/// subscriptions that wait for it - dropped with it, `UnSubscribeToPacketType`.
struct Op {
    step: Step,
    started: Instant,
    answer: Arc<Mutex<Option<Answer>>>,
    _subscription: Option<PacketSubscription>,
}

impl std::fmt::Debug for Op {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Op")
            .field("step", &self.step)
            .field("started", &self.started)
            .finish_non_exhaustive()
    }
}

/// `device_op`'s start: the two subscriptions for the component's replies - the first to come
/// is the answer - then the message, numbered `request_id`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1014-1101`
fn start(telemetry: &Telemetry, op: &DeviceOp, request_id: u32, step: Step) -> Op {
    let answer = Arc::new(Mutex::new(None));
    let sink = Arc::clone(&answer);
    let (sysid, compid) = (op.sysid, op.compid);
    let subscription = telemetry.on_packet(move |packet| {
        if packet.sent || packet.sysid != sysid || packet.compid != compid {
            return;
        }
        let got = match &packet.message {
            MavMessage::DeviceOpReadReply(reply) => Answer {
                data: reply
                    .data
                    .iter()
                    .take(usize::from(reply.count))
                    .copied()
                    .collect(),
                result: reply.result,
            },
            MavMessage::DeviceOpWriteReply(reply) => Answer {
                data: Vec::new(),
                result: reply.result,
            },
            _ => return,
        };
        if let Ok(mut held) = sink.os_lock() {
            held.get_or_insert(got);
        }
    });
    // `generatePacket`, which sends nothing without an open port.
    telemetry.send(&message(op, request_id));
    Op {
        step,
        started: Instant::now(),
        answer,
        _subscription: subscription,
    }
}

/// The form, while it is open.
#[derive(Debug)]
pub struct Form {
    /// The six numbers, in [`Num::ALL`]'s order.
    pub numbers: [NumericUpDown; 6],
    /// `dom_bustype`.
    pub bustype: Domain,
    /// `txt_spiname`.
    pub spi_name: TextField,
    /// `textBox1`, where the answers are added.
    pub output: TextField,
    /// `txt_spiname.Enabled`, `num_busno.Enabled` and `num_address.Enabled`.
    pub spi_enabled: bool,
    pub bus_enabled: bool,
    pub address_enabled: bool,
    /// The box with the keyboard.
    pub editing: Option<Field>,
}

impl Form {
    /// `new DevopsUI()`: the Designer's values, every box enabled.
    /// `// C#: Controls/DevopsUI.cs:15-19; Controls/DevopsUI.Designer.cs:29-273`
    #[must_use]
    pub fn new() -> Self {
        let mut spi_name = TextField::new("");
        spi_name.set(SPINAME);
        let mut output = TextField::new("");
        output.set_multiline(true);
        Self {
            numbers: Num::ALL.map(|num| NumericUpDown::new(num.designer())),
            bustype: Domain {
                text: SPI.to_owned(),
                index: None,
            },
            spi_name,
            output,
            spi_enabled: true,
            bus_enabled: true,
            address_enabled: true,
            editing: None,
        }
    }

    /// A number.
    #[must_use]
    pub const fn number(&self, num: Num) -> &NumericUpDown {
        let [sysid, compid, busno, address, regstart, count] = &self.numbers;
        match num {
            Num::SysId => sysid,
            Num::CompId => compid,
            Num::BusNo => busno,
            Num::Address => address,
            Num::RegStart => regstart,
            Num::Count => count,
        }
    }

    fn number_mut(&mut self, num: Num) -> &mut NumericUpDown {
        let [sysid, compid, busno, address, regstart, count] = &mut self.numbers;
        match num {
            Num::SysId => sysid,
            Num::CompId => compid,
            Num::BusNo => busno,
            Num::Address => address,
            Num::RegStart => regstart,
            Num::Count => count,
        }
    }

    /// Whether a number's box takes anything: the bus and the address follow the bus type.
    #[must_use]
    pub const fn enabled(&self, num: Num) -> bool {
        match num {
            Num::BusNo => self.bus_enabled,
            Num::Address => self.address_enabled,
            _ => true,
        }
    }

    /// `Convert.ToByte(num.Text)`: the box validated as the click's change of focus validates it,
    /// and its text read - a whole number within 0 to 255, so a byte.
    fn byte(&mut self, num: Num) -> u8 {
        u8::try_from(self.number_mut(num).int()).unwrap_or(u8::MAX)
    }

    /// `(uint)num_sysid.Value`: the box's value, from 0 to `uint.MaxValue`.
    /// `// C#: Controls/DevopsUI.cs:24`
    fn system_id(&mut self) -> u32 {
        // Constrained to the box's range, 0 to `u32::MAX`: whole and in range.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let id = self.number_mut(Num::SysId).commit() as u32;
        id
    }

    /// `dom_bustype_SelectedItemChanged`: SPI enables the name and disables the bus and the
    /// address; anything else the other way round.
    /// `// C#: Controls/DevopsUI.cs:35-51`
    fn bustype_changed(&mut self) {
        let spi = self.bustype.text == SPI;
        self.spi_enabled = spi;
        self.bus_enabled = !spi;
        self.address_enabled = !spi;
    }

    /// An arrow of the bus type's.
    pub fn step_bustype(&mut self, up: bool) {
        let chosen = if up {
            self.bustype.up()
        } else {
            self.bustype.down()
        };
        if chosen {
            self.bustype_changed();
        }
    }

    /// A box clicked into.
    pub fn begin(&mut self, field: Field) {
        if self.editing != Some(field) {
            self.leave();
        }
        self.editing = Some(field);
    }

    /// The keyboard left the box: a number's text validated.
    pub fn leave(&mut self) {
        if let Some(Field::Num(num)) = self.editing.take() {
            self.number_mut(num).commit();
        }
    }

    /// Do It's arguments, read from the boxes.
    /// `// C#: Controls/DevopsUI.cs:23-27`
    fn do_it(&mut self) -> DeviceOp {
        self.leave();
        DeviceOp {
            sysid: self.system_id(),
            compid: self.byte(Num::CompId),
            bustype: if self.bustype.text == SPI {
                SPI_BUS
            } else {
                I2C_BUS
            },
            name: self.spi_name.value().to_owned(),
            bus: self.byte(Num::BusNo),
            address: self.byte(Num::Address),
            regstart: self.byte(Num::RegStart),
            count: self.byte(Num::Count),
            write: None,
        }
    }

    /// test's two, the name read from its box.
    /// `// C#: Controls/DevopsUI.cs:57-59`
    fn test(&self, write: bool) -> DeviceOp {
        DeviceOp {
            sysid: u32::from(TEST_TARGET.0),
            compid: TEST_TARGET.1,
            bustype: SPI_BUS,
            name: self.spi_name.value().to_owned(),
            bus: 0,
            address: 0,
            regstart: TEST_REGISTER,
            count: TEST_COUNT,
            write: write.then(|| TEST_WRITE.to_vec()),
        }
    }

    /// `textBox1.AppendText(line + "\r\n")`.
    fn append(&mut self, line: &str) {
        let text = format!("{}{line}\r\n", self.output.value());
        self.output.set(text);
    }
}

impl Default for Form {
    fn default() -> Self {
        Self::new()
    }
}

/// The window, held for the application, with `MAVLinkInterface`'s request counter.
#[derive(Debug, Default)]
pub struct Devops {
    /// The form, while it is open.
    pub window: Option<Form>,
    /// How many times it has been opened.
    pub opened: usize,
    /// `request_id`, numbering each `device_op`. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:999`
    request_id: u32,
    /// The operation under way.
    op: Option<Op>,
    /// `DEVICE_OP` messages offered to the link.
    pub sent: usize,
}

impl Devops {
    /// Ctrl+J: a fresh form.
    pub fn show(&mut self) {
        self.op = None;
        self.window = Some(Form::new());
        self.opened += 1;
    }

    /// The close box: the form gone, and an answer still awaited with it.
    pub fn close(&mut self) {
        self.op = None;
        self.window = None;
    }

    /// The operation under way, if one is.
    #[must_use]
    pub fn busy(&self) -> Option<Step> {
        self.op.as_ref().map(|op| op.step)
    }

    fn begin_op(&mut self, telemetry: &Telemetry, op: &DeviceOp, step: Step) {
        let request_id = self.request_id;
        self.request_id = self.request_id.wrapping_add(1);
        self.sent += 1;
        self.op = Some(start(telemetry, op, request_id, step));
    }

    /// Do It. `// C#: Controls/DevopsUI.cs:21-33`
    pub fn press_do_it(&mut self, telemetry: &Telemetry) {
        if self.op.is_some() {
            return;
        }
        let Some(op) = self.window.as_mut().map(Form::do_it) else {
            return;
        };
        self.begin_op(telemetry, &op, Step::DoIt);
    }

    /// test: the write first. `// C#: Controls/DevopsUI.cs:53-63`
    pub fn press_test(&mut self, telemetry: &Telemetry) {
        if self.op.is_some() {
            return;
        }
        let Some(op) = self.window.as_mut().map(|form| {
            form.leave();
            form.test(true)
        }) else {
            return;
        };
        self.begin_op(telemetry, &op, Step::TestWrite);
    }

    /// Once a frame: the operation's answer, or its second gone by with none (nothing read,
    /// result 0), and what follows - Do It's line, test's read, test's line; what test throws
    /// when its read brought nothing is returned for the status line.
    /// `// C#: Controls/DevopsUI.cs:29-32, 59-62; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1103-1112`
    pub fn tick(&mut self, telemetry: &Telemetry, now: Instant) -> Option<String> {
        let op = self.op.as_ref()?;
        let answer = op.answer.os_lock().ok().and_then(|held| held.clone());
        if answer.is_none() && now.saturating_duration_since(op.started) < WAIT {
            return None;
        }
        let step = self.op.take()?.step;
        let answer = answer.unwrap_or_default();
        let form = self.window.as_mut()?;
        match step {
            Step::DoIt => {
                if answer.data.is_empty() {
                    form.append(&format!("No Response - {}", answer.result));
                } else {
                    form.append(&hex(&answer.data));
                }
            }
            Step::TestWrite => {
                let op = form.test(false);
                self.begin_op(telemetry, &op, Step::TestRead);
            }
            Step::TestRead => {
                if answer.data.is_empty() {
                    return Some(NO_ELEMENTS.to_owned());
                }
                form.append(&hex(&answer.data));
            }
        }
        None
    }
}

/// Facts a UI test asserts on, under `devops.`.
pub fn record_facts(holder: &Devops) {
    use crate::facts::record;
    record("devops.window", holder.window.is_some());
    record("devops.opened", holder.opened);
    record("devops.sent", holder.sent);
    record("devops.busy", holder.busy().map_or("none", Step::key));
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    for num in Num::ALL {
        record(
            format!("devops.{}", num.key()),
            form.number(num).field.value(),
        );
    }
    record("devops.bustype", &form.bustype.text);
    record("devops.spiname", form.spi_name.value());
    record("devops.spiname.enabled", form.spi_enabled);
    record("devops.busno.enabled", form.bus_enabled);
    record("devops.address.enabled", form.address_enabled);
    let lines: Vec<&str> = form
        .output
        .value()
        .split("\r\n")
        .filter(|line| !line.is_empty())
        .collect();
    record("devops.lines", lines.len());
    record("devops.last", lines.last().copied().unwrap_or("none"));
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// The keyboard focus of the form's boxes.
pub struct FocusHandles {
    /// The number being typed into.
    pub number: FocusHandle,
    /// `txt_spiname`.
    pub text: FocusHandle,
    /// `textBox1`.
    pub output: FocusHandle,
}

impl FocusHandles {
    /// New handles.
    pub fn new(cx: &mut Context<MissionPlanner>) -> Self {
        Self {
            number: cx.focus_handle(),
            text: cx.focus_handle(),
            output: cx.focus_handle(),
        }
    }

    /// Once a frame: a box the keyboard has left is left.
    pub fn tick(&self, holder: &mut Devops, window: &Window) {
        let Some(form) = holder.window.as_mut() else {
            return;
        };
        let held = match form.editing {
            Some(Field::Num(_)) => self.number.is_focused(window),
            Some(Field::SpiName) => self.text.is_focused(window),
            None => true,
        };
        if !held {
            form.leave();
        }
    }
}

/// How the drawing reaches the holder.
fn access(this: &mut MissionPlanner) -> Option<&mut Form> {
    this.key_forms.devops.window.as_mut()
}

/// A place in the flow panel, as one on the control.
const fn in_flow((x, y, width, height): (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
    (FLOW_AT.0 + x, FLOW_AT.1 + y, width, height)
}

/// `dom_bustype`: its text and its two arrows, `devops-dom_bustype-up` and `-down`.
fn domain_box(form: &Form, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (x, y, width, height) = in_flow(BUSTYPE_AT);
    let text = crate::probe::measured("devops-dom_bustype", div())
        .flex_1()
        .h_full()
        .flex()
        .items_center()
        .px_1()
        .overflow_hidden()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .child(form.bustype.text.clone());
    let mut arrows = div()
        .w(px(14.0))
        .h_full()
        .flex()
        .flex_col()
        .border_l_1()
        .border_color(rgb(theme::BORDER));
    for (suffix, glyph, up) in [("up", "\u{25b2}", true), ("down", "\u{25bc}", false)] {
        let id = format!("devops-dom_bustype-{suffix}");
        arrows = arrows.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .flex_1()
                .flex()
                .items_center()
                .justify_center()
                .text_size(px(7.0))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(glyph)
                .on_click(cx.listener(move |this, _event, window, cx| {
                    window.blur(cx);
                    if let Some(form) = access(this) {
                        form.leave();
                        form.step_bustype(up);
                    }
                    cx.notify();
                })),
        );
    }
    at(x, y, width, height)
        .flex()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::ACTION))
        .child(text)
        .child(arrows)
        .into_any_element()
}

/// One of the numbers.
fn number_box(
    form: &Form,
    num: Num,
    focus: &FocusHandles,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (x, y) = num.place();
    super::form_box(
        num.id(),
        form.number(num).field.value().to_owned(),
        in_flow((x, y, 62.0, 20.0)),
        true,
        form.editing == Some(Field::Num(num)),
        form.enabled(num),
        &focus.number,
        super::BoxHandlers {
            begin: move |this: &mut MissionPlanner| {
                if let Some(form) = access(this) {
                    form.begin(Field::Num(num));
                }
            },
            key: move |this: &mut MissionPlanner, event: &KeyDownEvent| {
                access(this).is_some_and(|form| form.number_mut(num).key(event))
            },
            step: move |this: &mut MissionPlanner, up: bool| {
                if let Some(form) = access(this) {
                    form.leave();
                    form.number_mut(num).step(if up { 1.0 } else { -1.0 });
                }
            },
        },
        window,
        cx,
    )
}

/// `txt_spiname`.
fn spiname_box(
    form: &Form,
    focus: &FocusHandles,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    super::form_box(
        "devops-txt_spiname",
        form.spi_name.value().to_owned(),
        in_flow(SPINAME_AT),
        false,
        form.editing == Some(Field::SpiName),
        form.spi_enabled,
        &focus.text,
        super::BoxHandlers {
            begin: |this: &mut MissionPlanner| {
                if let Some(form) = access(this) {
                    form.begin(Field::SpiName);
                }
            },
            key: |this: &mut MissionPlanner, event: &KeyDownEvent| {
                access(this)
                    .is_some_and(|form| matches!(form.spi_name.key(event), KeyOutcome::Changed))
            },
            step: |_: &mut MissionPlanner, _: bool| {},
        },
        window,
        cx,
    )
}

/// The form over the window: the panel, the two buttons, and the text box.
/// `// C#: Controls/DevopsUI.Designer.cs:29-273`
pub fn overlay(
    holder: &Devops,
    focus: &FocusHandles,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let idle = holder.op.is_none();
    let mut client = div().relative().w(px(SIZE.0)).h(px(SIZE.1));
    for (_, text, (x, y)) in LABELS {
        client = client.child(label(FLOW_AT.0 + x, FLOW_AT.1 + y, text, true));
    }
    for num in Num::ALL {
        client = client.child(number_box(form, num, focus, window, cx));
    }
    client = client
        .child(domain_box(form, cx))
        .child(spiname_box(form, focus, window, cx))
        .child(button(
            "devops-but_doit",
            DOIT,
            DOIT_AT,
            idle,
            |this, window, cx| {
                window.blur(cx);
                this.key_forms.devops.press_do_it(&this.telemetry);
            },
            cx,
        ))
        .child(button(
            "devops-but_test",
            TEST,
            TEST_AT,
            idle,
            |this, window, cx| {
                window.blur(cx);
                this.key_forms.devops.press_test(&this.telemetry);
            },
            cx,
        ))
        .child({
            let (x, y, width, height) = OUTPUT_AT;
            // What the box does not take stays in the form, as `form_box`'s keys do.
            at(x, y, width, height)
                .on_key_down(cx.listener(|_this, _event: &KeyDownEvent, _window, cx| {
                    cx.stop_propagation();
                }))
                .child(crate::textfield::text_area(
                    "devops-textBox1",
                    &form.output,
                    &focus.output,
                    focus.output.is_focused(window),
                    px(width),
                    px(height),
                    cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                        if access(this).is_some_and(|form| {
                            matches!(form.output.key(event), KeyOutcome::Changed)
                        }) {
                            cx.notify();
                        }
                    }),
                ))
        });
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
            "devops-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.key_forms.devops.close();
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("devops", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("devops-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{Vehicle, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::{DeviceOpReadReply, DeviceOpWriteReply};

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// The `DEVICE_OP` messages the vehicle end heard.
    fn device_ops(vehicle: &Vehicle) -> Vec<MavMessage> {
        vehicle
            .heard
            .iter()
            .filter(|message| {
                matches!(
                    message,
                    MavMessage::DeviceOpRead(_) | MavMessage::DeviceOpWrite(_)
                )
            })
            .copied()
            .collect()
    }

    fn read_reply(count: u8, bytes: &[u8], result: u8) -> MavMessage {
        let mut data = [0u8; 128];
        for (to, from) in data.iter_mut().zip(bytes) {
            *to = *from;
        }
        MavMessage::DeviceOpReadReply(DeviceOpReadReply {
            request_id: 0,
            result,
            regstart: 0,
            count,
            data,
            bank: 0,
        })
    }

    /// The Designer's values: 1, 1, 0, 0, 255, 0; "SPI" chosen by its text alone; the name; every
    /// box enabled.
    #[test]
    fn the_form_opens_as_the_designer_leaves_it() {
        let mut holder = Devops::default();
        holder.show();
        let form = holder.window.as_ref().expect("the form");
        let texts: Vec<&str> = Num::ALL
            .iter()
            .map(|num| form.number(*num).field.value())
            .collect();
        assert_eq!(texts, ["1", "1", "0", "0", "255", "0"]);
        assert_eq!(form.bustype.text, "SPI");
        assert_eq!(form.bustype.index, None);
        assert_eq!(form.spi_name.value(), SPINAME);
        assert!(form.spi_enabled && form.bus_enabled && form.address_enabled);
        assert!(form.output.value().is_empty());
        assert_eq!(holder.opened, 1);
    }

    /// The bus type's arrows: down from none chosen to I2C, which enables the bus and the address
    /// and disables the name; down to SPI, the other way; up to I2C; up again, nothing.
    #[test]
    fn the_bus_type_switches_the_boxes() {
        let mut form = Form::new();
        form.step_bustype(true);
        assert_eq!(form.bustype.text, "SPI");
        assert!(form.spi_enabled && form.bus_enabled && form.address_enabled);
        form.step_bustype(false);
        assert_eq!(form.bustype.text, "I2C");
        assert!(!form.spi_enabled && form.bus_enabled && form.address_enabled);
        form.step_bustype(false);
        assert_eq!(form.bustype.text, "SPI");
        assert!(form.spi_enabled && !form.bus_enabled && !form.address_enabled);
        form.step_bustype(false);
        assert_eq!(form.bustype.text, "SPI");
        form.step_bustype(true);
        assert_eq!(form.bustype.text, "I2C");
        form.step_bustype(true);
        assert_eq!(form.bustype.text, "I2C");
        assert!(form.bus_enabled && !form.spi_enabled);
    }

    /// The read and the write as `device_op` fills them: the name's bytes padded to 40, a
    /// write's count its bytes', its data padded to 128.
    #[test]
    fn the_messages_are_device_ops() {
        let mut form = Form::new();
        let op = form.do_it();
        assert_eq!(
            op,
            DeviceOp {
                sysid: 1,
                compid: 1,
                bustype: SPI_BUS,
                name: SPINAME.to_owned(),
                bus: 0,
                address: 0,
                regstart: 255,
                count: 0,
                write: None,
            }
        );
        let MavMessage::DeviceOpRead(read) = message(&op, 7) else {
            panic!("a read");
        };
        assert_eq!(read.request_id, 7);
        assert_eq!(&read.busname[..12], SPINAME.as_bytes());
        assert!(read.busname[12..].iter().all(|byte| *byte == 0));
        assert_eq!((read.target_system, read.target_component), (1, 1));
        assert_eq!((read.regstart, read.count, read.bustype), (255, 0, 1));
        let MavMessage::DeviceOpWrite(write) = message(&form.test(true), 8) else {
            panic!("a write");
        };
        assert_eq!((write.count, write.regstart, write.bustype), (2, 0xff, 1));
        assert_eq!(&write.data[..3], &[0x72, 0x00, 0x00]);
        assert_eq!(busname(&"x".repeat(50)), [b'x'; 40]);
        // I2C: the bus type's text says which.
        form.step_bustype(false);
        assert_eq!(form.do_it().bustype, I2C_BUS);
    }

    /// The numbers are read from their boxes as the click's change of focus validates them: a
    /// typed count past 100 is 100.
    #[test]
    fn the_numbers_are_validated_before_they_are_read() {
        let mut form = Form::new();
        form.begin(Field::Num(Num::Count));
        form.number_mut(Num::Count).field.set("300");
        let event = gpui::KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: "0".to_owned(),
                key_char: Some("0".to_owned()),
            },
            is_held: false,
            prefer_character_input: false,
        };
        assert!(form.number_mut(Num::Count).key(&event));
        assert_eq!(form.do_it().count, 100);
        assert_eq!(form.number(Num::Count).field.value(), "100");
        assert_eq!(form.editing, None);
    }

    /// Do It: the read goes, the component's reply's bytes come back as a line of hex.
    #[test]
    fn do_it_adds_the_bytes_read() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut holder = Devops::default();
        holder.show();
        holder.press_do_it(&telemetry);
        assert_eq!(holder.busy(), Some(Step::DoIt));
        until("the read", || {
            vehicle.read();
            !device_ops(&vehicle).is_empty()
        });
        assert!(matches!(
            device_ops(&vehicle)[0],
            MavMessage::DeviceOpRead(read) if read.request_id == 0 && read.regstart == 255
        ));
        vehicle.send(&read_reply(3, &[0xab, 0x01, 0x0f], 0));
        until("the answer", || {
            holder.tick(&telemetry, Instant::now());
            holder.busy().is_none()
        });
        let form = holder.window.as_ref().expect("the form");
        assert_eq!(form.output.value(), "AB010F\r\n");
        assert_eq!(holder.sent, 1);
    }

    /// No answer within the second: "No Response - 0"; a refusal that reads nothing, its result.
    #[test]
    fn no_answer_is_no_response() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut holder = Devops::default();
        holder.show();
        holder.press_do_it(&telemetry);
        // A second press while it waits does nothing.
        holder.press_do_it(&telemetry);
        assert_eq!(holder.sent, 1);
        assert_eq!(holder.tick(&telemetry, Instant::now()), None);
        assert_eq!(holder.busy(), Some(Step::DoIt));
        assert_eq!(holder.tick(&telemetry, Instant::now() + WAIT), None);
        assert_eq!(holder.busy(), None);
        holder.press_do_it(&telemetry);
        until("the second read", || {
            vehicle.read();
            device_ops(&vehicle).len() == 2
        });
        vehicle.send(&read_reply(0, &[], 5));
        until("the answer", || {
            holder.tick(&telemetry, Instant::now());
            holder.busy().is_none()
        });
        let form = holder.window.as_ref().expect("the form");
        assert_eq!(
            form.output.value(),
            "No Response - 0\r\nNo Response - 5\r\n"
        );
    }

    /// test: the write, its answer, then the read, whose bytes are the line; a read that brings
    /// none is `Aggregate`'s throw, for the status line.
    #[test]
    fn test_writes_then_reads() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut holder = Devops::default();
        holder.show();
        holder.press_test(&telemetry);
        assert_eq!(holder.busy(), Some(Step::TestWrite));
        until("the write", || {
            vehicle.read();
            !device_ops(&vehicle).is_empty()
        });
        vehicle.send(&MavMessage::DeviceOpWriteReply(DeviceOpWriteReply {
            request_id: 0,
            result: 0,
        }));
        until("the read", || {
            holder.tick(&telemetry, Instant::now());
            holder.busy() == Some(Step::TestRead)
        });
        until("the read sent", || {
            vehicle.read();
            device_ops(&vehicle).len() == 2
        });
        let ops = device_ops(&vehicle);
        assert!(matches!(ops[0], MavMessage::DeviceOpWrite(write) if write.data[0] == 0x72));
        assert!(matches!(ops[1], MavMessage::DeviceOpRead(read) if read.count == 2));
        vehicle.send(&read_reply(2, &[0x12, 0x34], 0));
        until("the answer", || {
            holder.tick(&telemetry, Instant::now());
            holder.busy().is_none()
        });
        assert_eq!(
            holder
                .window
                .as_ref()
                .map(|form| form.output.value().to_owned()),
            Some("1234\r\n".to_owned())
        );
        // Again, with no vehicle answering: the write's second, the read's, and the throw.
        holder.press_test(&telemetry);
        assert_eq!(holder.tick(&telemetry, Instant::now() + WAIT), None);
        assert_eq!(holder.busy(), Some(Step::TestRead));
        assert_eq!(
            holder.tick(&telemetry, Instant::now() + WAIT),
            Some(NO_ELEMENTS.to_owned())
        );
        assert_eq!(holder.busy(), None);
        assert_eq!(holder.sent, 4);
    }

    /// Without a link nothing is sent, and the second passes with no answer, as the C#'s closed
    /// port does.
    #[test]
    fn without_a_link_there_is_no_response() {
        let telemetry = Telemetry::idle();
        let mut holder = Devops::default();
        holder.show();
        holder.press_do_it(&telemetry);
        assert_eq!(holder.tick(&telemetry, Instant::now() + WAIT), None);
        let form = holder.window.as_ref().expect("the form");
        assert_eq!(form.output.value(), "No Response - 0\r\n");
        holder.close();
        assert!(holder.window.is_none() && holder.busy().is_none());
    }

    /// The Designer's words, places and values, read from the tree when it is here.
    #[test]
    fn the_words_and_places_are_the_designers() {
        let Some(designer) =
            crate::config_coverage::source::csharp("Controls/DevopsUI.Designer.cs")
        else {
            eprintln!(
                "skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner"
            );
            return;
        };
        for (name, text, (x, y)) in LABELS {
            assert!(
                designer.contains(&format!("this.{name}.Text = \"{text}\";")),
                "{name}"
            );
            assert!(
                designer.contains(&format!(
                    "this.{name}.Location = new System.Drawing.Point({x}, {y});"
                )),
                "{name}"
            );
        }
        for num in Num::ALL {
            let name = num.id().trim_start_matches("devops-");
            let (x, y) = num.place();
            assert!(
                designer.contains(&format!(
                    "this.{name}.Location = new System.Drawing.Point({x}, {y});"
                )),
                "{name}"
            );
        }
        assert!(designer.contains(&format!("this.but_doit.Text = \"{DOIT}\";")));
        assert!(designer.contains(&format!("this.but_test.Text = \"{TEST}\";")));
        assert!(designer.contains(&format!("this.txt_spiname.Text = \"{SPINAME}\";")));
        assert!(designer.contains(&format!("this.dom_bustype.Text = \"{SPI}\";")));
        for item in BUS_TYPES {
            assert!(designer.contains(&format!("this.dom_bustype.Items.Add(\"{item}\");")));
        }
        assert!(designer.contains(&format!(
            "this.Size = new System.Drawing.Size({}, {});",
            SIZE.0, SIZE.1
        )));
    }
}
