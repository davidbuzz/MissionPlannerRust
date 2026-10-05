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

//! Flying from a joystick: Mission Planner's Joystick Setup page, `Joystick/JoystickSetup.cs`.
//!
//! The dangerous screen. Every other panel reads the vehicle or asks it to do one thing; this one
//! takes over the pilot's sticks, and the failure that matters is not a wrong number on screen but
//! a stick that stops reporting while the vehicle keeps flying the last position it heard.
//!
//! The safety is in `mp_input`, which is a separate crate with its own tests, precisely so the
//! decision about when to hand control back is not tangled up with how a panel is laid out. So is
//! the speed: the device is read and the frames are sent on threads of that crate's own
//! (`mp_input::reader`), because Deliverable 15's five milliseconds from stick to wire cannot be met from a
//! screen that sends what it polled once a frame. So is the arithmetic: `mp_input::mapping` is the
//! C#'s `pickchannel`, and `mp_input::config` its `JoyChannel`/`JoyButton` arrays and their files.
//! This module is the page: the device list, the sixteen channel rows (`JoystickAxis`), the button
//! rows `doButtontoUI` makes, Enable, Save, Elevons, Manual Control, Export and Import, the
//! settings forms ([`forms`]), and `MainV2.joystick` - the joystick object the page drives, which
//! is [`Joystick`] here and owns the reader.
//!
//! What the C# does, and what is done here:
//!
//! * `Joystick_Load` (`JoystickSetup.cs:28-129`): [`Sticks::load`], each time the page is shown.
//!   The C#'s page object keeps its controls and runs `Load` when first shown only, so a second
//!   visit within one SETUP screen finds the timer `Deactivate` stopped and nothing restarting it;
//!   here each visit loads, as the first does.
//! * `timer1_Tick` (`:204-312`): [`Sticks::timer_tick`], once a frame while the page shows - the
//!   display joystick made and acquired, the button rows made, the bars and the buttons' bars.
//! * `BUT_enable_Click` (`:142-190`), `BUT_save_Click` (`:192-202`), `CMB_joysticks_Click` and
//!   `SelectedIndexChanged` (`:314-327`, `:489-499`), the button rows' handlers (`:329-487`),
//!   `CHK_elevons_CheckedChanged`, `chk_manualcontrol_CheckedChanged`, `Deactivate` and
//!   `JoystickSetup_FormClosed` (`:501-536`), `but_export_Click`/`but_import_Click` (`:538-567`).
//! * `JoystickBase.ProcessButtonEvent` (`ExtLibs/ArduPilot/Joystick/JoystickBase.cs:377-624`):
//!   [`perform`], for the presses the reader hands over.
//!
//! Divergences, each written at its site too:
//!
//! * the errors the C# puts in a message box - "Please Connect a Joystick", "Please select a
//!   joystick", "Lost Joystick", "No valid option was detected", the button functions' "Failed
//!   to ..." - are said on the status line (the owner's ruling of 2026-09-25); the instructions
//!   (move an axis, press a button, reopen the page), "No settings to set" and Import's question
//!   keep their boxes;
//! * Enable's text says what a click does: the C#'s keeps "Disable" after the joystick is lost;
//! * the devices are the `js` nodes by their sysfs names, as `mp_input::linux` lists them, rather
//!   than `/dev/input/by-id/*joystick`: the C#'s Windows list is product names, which this is;
//! * the file dialogs are typed paths, as every file dialog in this application is.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
mod draw;
pub mod forms;

pub use draw::{overlay, page, page_size};

use mp_os::Lock as _;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use web_time::{Duration, Instant};

use gpui::KeyDownEvent;
use mp_input::mapping::{CHANNELS, axis_value};
use mp_input::{
    ButtonEvent, ButtonFunction, ConfigFiles, Device, JoyButton, JoystickAxis, JoystickConfig,
    Mapping, Poll, RcRanges, Reading, StickReader,
};
use mp_link::LinkSender;
use mp_mavlink_dialects::all::MavCmd;
use mp_vehicle::VehicleId;

use crate::MissionPlanner;
use crate::config::firmware::PathBox;
use crate::config::flight_modes::{Firmware, firmware_of};
use crate::fly::error_box;
use crate::settings::Persisted;
use crate::telemetry::{Report, Telemetry, TelemetryView};
use crate::textfield::KeyOutcome;
use forms::{Form, FormKind};

/// Test scaffolding: the device list is this one scripted device - `mp_input::scripted`'s
/// `name;<ms>:a<n>=<value>;...` - instead of the machine's joysticks, as the firmware page's
/// `MP_FIRMWARE_DEVICE` stands in for its USB ports. This machine has no joystick.
pub const DEVICE_ENV: &str = "MP_JOYSTICK_DEVICE";

/// `maxaxis`: the channel rows.
/// `// C#: Joystick/JoystickSetup.cs:19`
pub const CHANNEL_ROWS: usize = 16;

/// `Math.Min(16, noButtons)`: the button rows at most.
/// `// C#: Joystick/JoystickSetup.cs:230`
pub const BUTTON_ROWS: usize = 16;

/// `getNumButtons` on Mission Planner's Linux joystick when it has no device to ask: the ioctl on
/// a null handle throws, and the `catch` returns the 10 it started with.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickLinux.cs:293-312`
pub const UNKNOWN_BUTTONS: usize = 10;

/// How long `JoystickLinux.Start` waits for the device's first event before giving up.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickLinux.cs:143-150`
const START_WAIT: Duration = Duration::from_millis(1200);

/// The button numbers a row offers: `getButtonNumbers`, -1 then 0 to 126.
/// `// C#: Joystick/JoystickSetup.cs:131-140`
pub const BUTTON_NUMBERS: std::ops::RangeInclusive<i32> = -1..=126;

/// `BUT_enable_Click`'s box: "Please Connect a Joystick", captioned "No Joystick" - on the status
/// line here, an error the page can say.
/// `// C#: Joystick/JoystickSetup.cs:161-166`
pub const NO_JOYSTICK: &str = "Please Connect a Joystick";
/// `BUT_save_Click`'s box.
/// `// C#: Joystick/JoystickSetup.cs:194-198`
pub const SELECT_JOYSTICK: &str = "Please select a joystick";
/// `getMovingAxis`'s instruction.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1202`
pub const MOVE_AXIS: &str =
    "Please move the joystick axis you want assigned to this function after clicking ok";
/// `getPressedButton`'s instruction.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:201-202`
pub const PRESS_BUTTON: &str =
    "Please press the joystick button you want assigned to this function after clicking ok";
/// What either says when ten seconds pass with nothing.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:219, 1267`
pub const NOTHING_DETECTED: &str = "No valid option was detected";
/// `but_settings_Click` for a function with no form.
/// `// C#: Joystick/JoystickSetup.cs:483-485`
pub const NO_SETTINGS: &str = "No settings to set";
/// Its caption.
pub const NO_SETTINGS_TITLE: &str = "No settings";
/// `but_import_Click`'s question.
/// `// C#: Joystick/JoystickSetup.cs:551`
pub const IMPORT_WARNING: &str = "NOTE: this will replace any existing joystick configuration.\nPlease make sure you have saved your current configuration if needed.";
/// Its caption.
pub const IMPORT_TITLE: &str = "Import Joystick Config";
/// What Import says when it has read the archive.
/// `// C#: Joystick/JoystickSetup.cs:559`
pub const REOPEN: &str = "Please reopen joystick for changes to take effect";
/// `LostAction`'s box.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:98-104`
pub const LOST: &str = "Lost Joystick";
/// Export's and Import's filter.
/// `// C#: Joystick/JoystickSetup.cs:541, 554`
pub const JOYCFG_FILTER: &str = "Joystick config files (*.joycfg)|*.joycfg|All files (*.*)|*.*";
/// `SaveFileDialog`'s caption.
pub const SAVE_AS: &str = "Save As";
/// `label14`'s text, before the firmware is added to it.
/// `// C#: Joystick/JoystickSetup.resx (label14.Text); JoystickSetup.cs:69`
pub const LOADED_CONFIG_FOR: &str = "Loaded Config for";
/// `ExportConfig`'s `FileNotFoundException`.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1319-1322`
pub const NO_CONFIG_FILES: &str = "No joystick configuration files found in ";

/// `getMovingAxis(name, 16000)`: how far an axis must move to be the one.
/// `// C#: Joystick/JoystickSetup.cs:91`
const DETECT_THRESHOLD: i32 = 16_000;
/// How long either detection watches.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:206, 1206`
const DETECT_WINDOW: Duration = Duration::from_secs(10);

/// The properties `getMovingAxis` reads, in `LinuxJoystickState`'s declaration order, which is
/// the order `GetProperties` gives and so the order the first mover is found in.
/// `// C#: ExtLibs/ArduPilot/Joystick/MyJoystickState.cs:46-100; JoystickBase.cs:1189-1234`
const MOVING_AXES: [JoystickAxis; 24] = [
    JoystickAxis::AZ,
    JoystickAxis::AY,
    JoystickAxis::AX,
    JoystickAxis::ARz,
    JoystickAxis::ARy,
    JoystickAxis::ARx,
    JoystickAxis::FRx,
    JoystickAxis::FRy,
    JoystickAxis::FRz,
    JoystickAxis::FX,
    JoystickAxis::FY,
    JoystickAxis::FZ,
    JoystickAxis::Rx,
    JoystickAxis::Ry,
    JoystickAxis::Rz,
    JoystickAxis::VRx,
    JoystickAxis::VRy,
    JoystickAxis::VRz,
    JoystickAxis::VX,
    JoystickAxis::VY,
    JoystickAxis::VZ,
    JoystickAxis::X,
    JoystickAxis::Y,
    JoystickAxis::Z,
];

/// Where frames go: a handle that sends on the link, and the vehicle to address them to.
///
/// `None` while there is no vehicle. The reader's send thread reads this on every frame, so a
/// vehicle chosen on screen is the vehicle the next frame goes to, and a link that has closed is
/// a frame that is refused - which for a release means it is retried, not forgotten.
pub type Target = Option<(LinkSender, VehicleId)>;

// ---------------------------------------------------------------------------------------------
// Devices.
// ---------------------------------------------------------------------------------------------

/// Where joysticks come from: `JoystickBase.getDevices` and `getJoyStickByName`.
pub trait DeviceSource: std::fmt::Debug {
    /// The devices there are now.
    fn devices(&self) -> Vec<Device>;
    /// A device opened for reading `js_event`s, blocking.
    ///
    /// # Errors
    /// The device cannot be opened.
    fn open(&self, device: &Device) -> std::io::Result<Box<dyn Read + Send>>;
}

/// The machine's `js` nodes, or the scripted device [`DEVICE_ENV`] names.
#[derive(Debug)]
pub struct Machine;

/// The id a scripted device goes by.
const SCRIPTED: &str = "scripted:";

fn scripted() -> Option<mp_input::scripted::Script> {
    let spec = std::env::var(DEVICE_ENV).ok()?;
    mp_input::scripted::Script::parse(&spec).ok()
}

impl DeviceSource for Machine {
    fn devices(&self) -> Vec<Device> {
        if let Some(script) = scripted() {
            return vec![Device {
                id: format!("{SCRIPTED}{}", script.name),
                name: script.name.clone(),
                axes: script.axes(),
                buttons: script.buttons(),
            }];
        }
        #[cfg(target_os = "linux")]
        {
            mp_input::linux::devices()
        }
        #[cfg(not(target_os = "linux"))]
        {
            Vec::new()
        }
    }

    fn open(&self, device: &Device) -> std::io::Result<Box<dyn Read + Send>> {
        if let Some(script) =
            scripted().filter(|script| device.id == format!("{SCRIPTED}{}", script.name))
        {
            return Ok(Box::new(script.open()));
        }
        #[cfg(target_os = "linux")]
        {
            // Blocking: the reader has a thread whose whole job is to wait on it.
            Ok(Box::new(mp_os::fs::File::open(&device.id)?))
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = device;
            Err(std::io::ErrorKind::Unsupported.into())
        }
    }
}

/// What the reader's send thread shares with the screen.
#[derive(Debug, Clone, Default)]
struct Wire {
    target: Arc<Mutex<Target>>,
    /// Frames the link accepted, for the facts.
    sent: Arc<AtomicU64>,
}

impl Wire {
    /// The reader's sink: runs on its send thread, must not block, and says whether the link took
    /// the frame - for a release that is the difference between an aircraft handed back and one
    /// still being flown by a stick nobody is holding.
    fn sink(&self) -> impl FnMut(&mp_input::Frame) -> bool + Send + 'static {
        let wire = self.clone();
        move |frame| {
            let guard = wire.target.os_lock().unwrap_or_else(PoisonError::into_inner);
            let Some((sender, vehicle)) = guard.as_ref() else {
                // No vehicle to send to. Not delivered, and said so, rather than swallowed.
                return false;
            };
            let message = match frame.manual {
                // `rc.target = comPort.MAV.compid`: the C#'s, which a vehicle whose system id is
                // not its component id does not take as its own.
                // `// C#: MainV2.cs:2409-2435`
                Some(manual) => mp_link::commands::manual_control(
                    vehicle.compid,
                    manual.x,
                    manual.y,
                    manual.z,
                    manual.r,
                ),
                None => mp_link::commands::rc_override(*vehicle, frame.channels.0),
            };
            let accepted = sender.send(&message);
            if accepted {
                wire.sent.fetch_add(1, Ordering::Relaxed);
            }
            accepted
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The joystick object.
// ---------------------------------------------------------------------------------------------

/// A `JoystickBase`: its settings, their files, and - once acquired - the device, read on the
/// reader's threads.
#[derive(Debug)]
pub struct Joystick {
    /// `name`: the device it was acquired by.
    pub name: String,
    /// `JoyChannels`, `JoyButtons`, `elevons`, `manual_control` and the vehicle's ranges.
    mapping: Mapping,
    /// `joystickconfigbutton` and `joystickconfigaxis`: where `loadconfig` found them and
    /// `saveconfig` writes them.
    files: ConfigFiles,
    /// The acquired device; `None` is `IsJoystickValid` false.
    reader: Option<StickReader>,
    /// When it was acquired, for how long `Start` would have waited.
    acquired: Option<Instant>,
    /// Whether it was flying at the last look, so a device lost while flying is said.
    flying: bool,
}

impl Joystick {
    /// `JoystickBase.Create`: the constructor, which loads the files named for the vehicle's
    /// firmware.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:62-105, 1296-1306`
    fn create(firmware: Firmware, data_dir: &Path, ranges: &RcRanges) -> Self {
        let files = ConfigFiles::for_firmware(firmware.label(), data_dir);
        Self {
            name: String::new(),
            mapping: Mapping {
                config: JoystickConfig::load(&files),
                ranges: ranges.clone(),
                ..Mapping::default()
            },
            files,
            reader: None,
            acquired: None,
            flying: false,
        }
    }

    /// `enabled`: flying. Goes false by itself when the device is lost.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.reader.as_ref().is_some_and(StickReader::is_enabled)
    }

    /// `IsJoystickValid`.
    #[must_use]
    pub const fn is_acquired(&self) -> bool {
        self.reader.is_some()
    }

    /// The settings as the joystick holds them, for the tests.
    #[cfg(test)]
    #[must_use]
    pub const fn config(&self) -> &JoystickConfig {
        &self.mapping.config
    }

    /// Hands the reader the settings as they are now.
    fn push(&self) {
        if let Some(reader) = &self.reader {
            reader.set_mapping(self.mapping.clone());
        }
    }

    /// `AcquireJoystick(name)`: the device of that name opened and read. Already acquired is
    /// success.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickLinux.cs:236-256`
    fn acquire(&mut self, source: &dyn DeviceSource, name: &str, wire: &Wire) -> std::io::Result<()> {
        if self.reader.is_some() {
            return Ok(());
        }
        let device = source
            .devices()
            .into_iter()
            .find(|device| device.name == name)
            .ok_or_else(|| std::io::Error::from(std::io::ErrorKind::NotFound))?;
        let reader = StickReader::spawn(source.open(&device)?, self.mapping.clone(), wire.sink())?;
        self.reader = Some(reader);
        self.acquired = Some(Instant::now());
        Ok(())
    }

    /// `UnAcquireJoyStick`. A reader that is flying hands control back as it is dropped.
    fn unacquire(&mut self) {
        self.reader = None;
        self.acquired = None;
    }

    /// `start(name)`: acquired, and flying.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1002-1022`
    fn start(&mut self, source: &dyn DeviceSource, name: &str, wire: &Wire) -> bool {
        name.clone_into(&mut self.name);
        if self.acquire(source, name, wire).is_err() {
            return false;
        }
        self.reader
            .as_ref()
            .is_some_and(|reader| reader.set_enabled(true))
    }

    /// `getValueForChannel`: 0 while not acquired.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1149-1160`
    fn value_for_channel(&self, channel: usize) -> i16 {
        self.reader.as_ref().map_or(0, |reader| {
            self.mapping
                .value_for_channel(channel, &reader.reading(), reader.runtime())
        })
    }

    /// `isButtonPressed(slot)`: the state of the device button the function is given.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:716-727`
    fn is_button_pressed(&self, slot: usize) -> bool {
        let Some(reader) = &self.reader else {
            return false;
        };
        usize::try_from(self.mapping.config.button(slot).buttonno)
            .is_ok_and(|number| reader.reading().button(number))
    }
}

// ---------------------------------------------------------------------------------------------
// The page's state.
// ---------------------------------------------------------------------------------------------

/// Which window the page is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Host {
    /// Initial Setup's list, under Optional Hardware.
    /// `// C#: GCSViews/InitialSetup.cs:278-281`
    Setup,
    /// Planner's Joystick Setup button: `new JoystickSetup().ShowUserControl()`.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:552-555`
    Planner,
}

/// One `JoystickAxis` row: what its three controls hold.
#[derive(Debug)]
pub struct AxisRow {
    /// `CMB_CH.Text`: the axis's name.
    pub axis: String,
    /// `expo_ch`.
    pub expo: crate::textfield::TextField,
    /// `revCH.Checked`.
    pub reverse: bool,
}

/// One row `doButtontoUI` made: what its two combos hold.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ButtonRow {
    /// `butnumberlist`'s text: a button number, "-1" for none.
    pub number: String,
    /// `cmbaction`'s text: the function's name.
    pub action: String,
}

impl ButtonRow {
    /// `doButtontoUI`'s filling from the function's settings: a number the list does not hold
    /// leaves it on its first row, "-1".
    /// `// C#: Joystick/JoystickSetup.cs:376-424`
    fn of(button: &JoyButton) -> Self {
        Self {
            number: if BUTTON_NUMBERS.contains(&button.buttonno) {
                button.buttonno.to_string()
            } else {
                "-1".to_owned()
            },
            action: button.function.name().to_owned(),
        }
    }
}

/// A drop-down list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum List {
    /// `CMB_joysticks`.
    Devices,
    /// A channel row's `CMB_CH`.
    Axis(usize),
    /// A button row's `butnumberlist`.
    Number(usize),
    /// A button row's `cmbaction`.
    Action(usize),
    /// The open form's `comboBox1`.
    Form,
}

/// What has the keyboard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Editing {
    /// A channel row's `expo_ch`.
    Expo(usize),
    /// A box on the open form.
    Form(usize),
    /// The file dialog's path.
    Path,
}

/// What a message box's OK leads to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum After {
    /// Nothing.
    Nothing,
    /// A detection's watch starts.
    Watch,
    /// Import's close of the page and its window.
    CloseAfterImport,
}

/// A message box: its caption, its text, and what its OK does.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    /// Caption.
    pub title: &'static str,
    /// Text.
    pub text: &'static str,
    /// What OK does.
    pub after: After,
}

/// Which file dialog.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathUse {
    /// Export's `SaveFileDialog`.
    Export,
    /// Import's `OpenFileDialog`.
    Import,
}

/// What a detection looks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectKind {
    /// `getMovingAxis`, for a channel row.
    Axis(usize),
    /// `getPressedButton`, for a button row.
    Button(usize),
}

impl DetectKind {
    /// How long the C# sleeps before it takes the "before" state.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:195, 1183`
    const fn settle(self) -> Duration {
        match self {
            Self::Axis(_) => Duration::from_millis(300),
            Self::Button(_) => Duration::from_millis(500),
        }
    }
}

/// A detection under way: the device opened on its own, as `getJoyStickByName` opens it.
#[derive(Debug)]
pub struct Detect {
    /// What it is for.
    pub kind: DetectKind,
    reader: StickReader,
    opened: Instant,
    /// The state before, once taken.
    before: Option<Reading>,
    /// When the watch began: at the instruction's OK.
    since: Option<Instant>,
}

/// The page: `JoystickSetup`'s controls and timer.
#[derive(Debug, Default)]
pub struct Page {
    /// Where it is shown, while it is.
    pub host: Option<Host>,
    /// `timer1` running.
    pub timer: bool,
    /// `startup`, which the button number combos' handler checks.
    startup: bool,
    /// `CMB_joysticks.Items`.
    pub items: Vec<String>,
    /// `CMB_joysticks.SelectedIndex`.
    pub selected: Option<usize>,
    /// `CMB_joysticks.Text`.
    pub text: String,
    /// `CHK_elevons.Checked`.
    pub elevons: bool,
    /// `chk_manualcontrol.Checked`.
    pub manual: bool,
    /// `label14.Text`.
    pub label: String,
    /// The sixteen channel rows.
    pub rows: Vec<AxisRow>,
    /// The button rows, once `doButtontoUI` has made them.
    pub buttons: Vec<ButtonRow>,
    /// The display joystick's button count is not known yet.
    rows_pending: bool,
    /// Each button row's `hbar`: pressed or not.
    pub pressed: Vec<bool>,
    /// `cs.rcoverridech1` to `18`, as the channel rows' bars read them.
    pub rc: [i16; CHANNELS],
    /// The drop-down open, and the first row it shows.
    pub open: Option<List>,
    /// `TopIndex` of the open list.
    pub top: usize,
    /// What has the keyboard.
    pub editing: Option<Editing>,
    /// A message box.
    pub message: Option<Message>,
    /// Import's question.
    pub question: bool,
    /// A file dialog.
    pub path: Option<(PathUse, PathBox)>,
    /// A detection under way.
    pub detect: Option<Detect>,
    /// The settings form open.
    pub form: Option<Form>,
    /// What Save wrote, read back from the files.
    pub saved: Option<JoystickConfig>,
    /// Saves made.
    pub saves: usize,
    /// Export's archive and how many files went in.
    pub exported: Option<(PathBuf, usize)>,
    /// The last thing said.
    pub status: Option<String>,
    /// Import asked for the window the page is in to close.
    close_parent: bool,
}

/// The joystick, its devices, and the page.
#[derive(Debug)]
pub struct Sticks {
    source: Box<dyn DeviceSource>,
    /// `MainV2.joystick`.
    joystick: Option<Joystick>,
    wire: Wire,
    /// `cs.firmware`: the files a new joystick loads are named for it.
    firmware: Firmware,
    /// The vehicle's `RCn_*`.
    ranges: RcRanges,
    /// The parameter list the ranges were read from.
    parameters_seen: Option<Arc<[(String, f64)]>>,
    /// `Settings.GetUserDataDirectory()`.
    data_dir: PathBuf,
    /// `JoystickSetup`.
    pub page: Page,
    /// Words for the status line.
    said: Vec<String>,
}

impl Default for Sticks {
    fn default() -> Self {
        Self::new()
    }
}

impl Sticks {
    /// No joystick, the machine's devices, the user data directory.
    #[must_use]
    pub fn new() -> Self {
        Self::with_source(
            Box::new(Machine),
            mp_settings::user_data_directory().unwrap_or_default(),
        )
    }

    /// With devices from elsewhere, and the files in `data_dir`: for the tests.
    #[must_use]
    pub fn with_source(source: Box<dyn DeviceSource>, data_dir: PathBuf) -> Self {
        Self {
            source,
            joystick: None,
            wire: Wire::default(),
            firmware: Firmware::ArduCopter2,
            ranges: RcRanges::default(),
            parameters_seen: None,
            data_dir,
            page: Page::default(),
            said: Vec::new(),
        }
    }

    /// The joystick object, if there is one, for the tests.
    #[cfg(test)]
    #[must_use]
    pub const fn joystick(&self) -> Option<&Joystick> {
        self.joystick.as_ref()
    }

    /// Whether the sticks are flying.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.joystick.as_ref().is_some_and(Joystick::is_enabled)
    }

    /// How many frames the link has accepted since the application started.
    #[must_use]
    pub fn sent(&self) -> u64 {
        self.wire.sent.load(Ordering::Relaxed)
    }

    /// Stick-to-link latency so far, as (p50, p99), once anything has been measured.
    #[must_use]
    pub fn latency(&self) -> Option<(Duration, Duration)> {
        let histogram = self.joystick.as_ref()?.reader.as_ref()?.latency();
        Some((histogram.p50()?, histogram.p99()?))
    }

    /// Enable's text: "Disable" while flying.
    #[must_use]
    pub fn enable_text(&self) -> &'static str {
        // The C# sets the text as it is clicked, and keeps "Disable" after the joystick is lost;
        // here it says what a click does, on the page that flies the vehicle.
        if self.is_enabled() { "Disable" } else { "Enable" }
    }

    /// Says something on the status line.
    fn say(&mut self, words: impl Into<String>) {
        let words = words.into();
        self.page.status = Some(words.clone());
        self.said.push(words);
    }

    /// What to say on the status line, taken.
    pub fn take_said(&mut self) -> Vec<String> {
        std::mem::take(&mut self.said)
    }

    /// The button functions pressed while flying, for [`perform`].
    pub fn take_button_events(&mut self) -> Vec<ButtonEvent> {
        self.joystick
            .as_ref()
            .and_then(|joystick| joystick.reader.as_ref())
            .map(StickReader::take_button_events)
            .unwrap_or_default()
    }

    /// The firmware a new joystick names its files for.
    #[must_use]
    pub const fn firmware(&self) -> Firmware {
        self.firmware
    }

    // --- Once a frame ------------------------------------------------------------------------

    /// Keeps the reader pointed at the vehicle, the joystick holding the vehicle's ranges, notices
    /// a device lost, and runs the page's timer and a detection.
    pub fn tick(&mut self, target: Target, view: &TelemetryView, banner: Option<&str>) {
        // A joystick in use: its sticks read on their own thread, drawn as the timer drew them.
        if self.joystick.is_some() {
            crate::repaint::in_flight();
        }
        *self.wire.target.os_lock().unwrap_or_else(PoisonError::into_inner) = target;
        // `cs.firmware`, which starts as ArduCopter2 before any vehicle is heard.
        self.firmware = view.state.as_ref().map_or(Firmware::ArduCopter2, |state| {
            firmware_of(state.autopilot, state.vehicle_type, banner)
        });
        if !self
            .parameters_seen
            .as_ref()
            .is_some_and(|seen| Arc::ptr_eq(seen, &view.parameters))
        {
            self.parameters_seen = Some(Arc::clone(&view.parameters));
            let ranges = RcRanges::from_lookup(!view.parameters.is_empty(), |name| {
                parameter(view, name)
            });
            if ranges != self.ranges {
                self.ranges = ranges;
                if let Some(joystick) = &mut self.joystick {
                    joystick.mapping.ranges = self.ranges.clone();
                    joystick.push();
                }
            }
        }
        // A device that has gone: the reader has released control from its own thread already;
        // what is left is to let it go and, if it was flying, say so - the C#'s `LostAction`.
        let mut lost = false;
        if let Some(joystick) = &mut self.joystick {
            if joystick
                .reader
                .as_ref()
                .is_some_and(|reader| reader.liveness() == Poll::Gone)
            {
                lost = joystick.flying;
                joystick.unacquire();
            }
            joystick.flying = joystick.is_enabled();
        }
        if lost {
            self.say(LOST);
        }
        if self.page.timer {
            self.timer_tick();
        }
        self.detect_tick();
    }

    // --- Load, the timer, Deactivate -----------------------------------------------------------

    /// `Joystick_Load`: the device list and the one chosen, Elevons from the settings, the
    /// channel rows from the files a joystick made now loads, Enable's text, and the timer - which
    /// the Designer starts with the page.
    /// `// C#: Joystick/JoystickSetup.cs:28-129; JoystickSetup.Designer.cs:87`
    pub fn load(&mut self, host: Host, settings: &Persisted) {
        self.page = Page {
            host: Some(host),
            timer: true,
            startup: true,
            ..Page::default()
        };
        // `getDevices` never throws on Linux, so "do you have the directx redist installed?"
        // has nothing to say here.
        self.page.items = self
            .source
            .devices()
            .into_iter()
            .map(|device| device.name)
            .collect();
        if !self.page.items.is_empty() && self.page.selected.is_none() {
            self.select_index(Some(0));
        }
        if let Some(name) = settings.get("joystick_name").filter(|name| !name.is_empty()) {
            self.set_device_text(name.to_owned());
        }
        if let Some(elevons) = settings.get("joy_elevons").and_then(parse_bool) {
            self.set_elevons(elevons);
        }
        let temp = Joystick::create(self.firmware, &self.data_dir, &self.ranges);
        self.page.label = format!("{LOADED_CONFIG_FOR} {}", self.firmware.label());
        self.page.rows = (1..=CHANNEL_ROWS)
            .map(|channel| {
                let config = temp.mapping.config.channel(channel);
                let mut expo = crate::textfield::TextField::new("");
                expo.set(config.expo.to_string());
                AxisRow {
                    axis: config.axis.name().to_owned(),
                    expo,
                    reverse: config.reverse,
                }
            })
            .collect();
        self.page.startup = false;
    }

    /// `Deactivate` and `JoystickSetup_FormClosed`: the timer stopped, and a joystick that is not
    /// flying let go.
    /// `// C#: Joystick/JoystickSetup.cs:502-536`
    pub fn close(&mut self) {
        self.page.timer = false;
        self.page.host = None;
        self.page.open = None;
        self.page.editing = None;
        self.page.detect = None;
        self.page.form = None;
        if self.joystick.as_ref().is_some_and(|joystick| !joystick.is_enabled()) {
            self.joystick = None;
        }
    }

    /// `timer1_Tick`: while not flying, a joystick made and acquired if there is none - with its
    /// button rows - and the bars set from it; while flying, the bars show what `mainloop` sets.
    /// Then the button rows' bars.
    /// `// C#: Joystick/JoystickSetup.cs:204-312`
    pub fn timer_tick(&mut self) {
        if !self.is_enabled() {
            if self.joystick.is_none() {
                let mut joystick = Joystick::create(self.firmware, &self.data_dir, &self.ranges);
                // `joy.setChannel(a, config.axis, config.reverse, config.expo)` for the sixteen:
                // each slot replaced with its number filled in, which Save then writes.
                // `// C#: Joystick/JoystickSetup.cs:215-220`
                for channel in 1..=CHANNEL_ROWS {
                    let config = joystick.mapping.config.channel(channel);
                    joystick.mapping.config.set_channel(
                        channel,
                        config.axis,
                        config.reverse,
                        config.expo,
                    );
                }
                joystick.mapping.elevons = self.page.elevons;
                let name = self.page.text.clone();
                // `AcquireJoystick`'s answer is not looked at.
                let _ = joystick.acquire(&*self.source, &name, &self.wire);
                joystick.name = name;
                self.joystick = Some(joystick);
                self.page.buttons.clear();
                self.page.pressed.clear();
                self.page.rows_pending = true;
                // `CMB_joysticks.SelectedIndex = Items.IndexOf(joy.name)`.
                let index = self
                    .page
                    .items
                    .iter()
                    .position(|item| Some(item) == self.joystick.as_ref().map(|j| &j.name));
                self.select_index(index);
            }
            let elevons = self.page.elevons;
            if let Some(joystick) = &mut self.joystick {
                if joystick.mapping.elevons != elevons {
                    joystick.mapping.elevons = elevons;
                    joystick.push();
                }
                for (index, slot) in self.page.rc.iter_mut().enumerate() {
                    *slot = joystick.value_for_channel(index + 1);
                }
            }
        } else if let Some(joystick) = &self.joystick
            && let Some(reader) = &joystick.reader
        {
            let overrides = joystick
                .mapping
                .overrides(&reader.reading(), reader.runtime());
            for (index, slot) in self.page.rc.iter_mut().enumerate() {
                if let Some(value) = overrides.get(index + 1) {
                    *slot = value;
                }
            }
        }
        if self.page.rows_pending {
            self.make_button_rows();
        }
        let pressed: Vec<bool> = (0..self.page.buttons.len())
            .map(|slot| {
                self.joystick
                    .as_ref()
                    .is_some_and(|joystick| joystick.is_button_pressed(slot))
            })
            .collect();
        self.page.pressed = pressed;
    }

    /// `noButtons = Math.Min(16, joy.getNumButtons())` and `doButtontoUI` for each - once the
    /// count can be known: at once for a joystick with no device, whose count is the 10 the
    /// Linux joystick answers then; for an acquired one, once the device has reported, as
    /// `Start` waits for its first event.
    /// `// C#: Joystick/JoystickSetup.cs:228-247`
    fn make_button_rows(&mut self) {
        let Some(joystick) = &self.joystick else {
            return;
        };
        let count = match &joystick.reader {
            None => UNKNOWN_BUTTONS,
            Some(reader) => {
                let reading = reader.reading();
                let waited = joystick
                    .acquired
                    .is_none_or(|at| at.elapsed() >= START_WAIT);
                if reading.axes.is_empty() && reading.buttons.is_empty() && !waited {
                    return;
                }
                reading.buttons.len()
            }
        };
        let rows = count.min(BUTTON_ROWS);
        self.page.buttons = (0..rows)
            .map(|slot| ButtonRow::of(&joystick.mapping.config.button(slot)))
            .collect();
        self.page.pressed = vec![false; rows];
        self.page.rows_pending = false;
    }

    // --- The device list ---------------------------------------------------------------------

    /// `SelectedIndex` set: the text follows, and `SelectedIndexChanged` if it moved.
    fn select_index(&mut self, index: Option<usize>) {
        if index == self.page.selected {
            return;
        }
        self.page.selected = index;
        self.page.text = index
            .and_then(|index| self.page.items.get(index))
            .cloned()
            .unwrap_or_default();
        self.device_changed();
    }

    /// `Text` set: the row of that text - `FindStringExact`, ignoring case - selected, or none
    /// with the text kept, as an editable combo keeps it.
    fn set_device_text(&mut self, text: String) {
        let index = self
            .page
            .items
            .iter()
            .position(|item| item.eq_ignore_ascii_case(&text));
        let moved = index != self.page.selected;
        self.page.selected = index;
        self.page.text = index
            .and_then(|index| self.page.items.get(index))
            .cloned()
            .unwrap_or(text);
        if moved {
            self.device_changed();
        }
    }

    /// `CMB_joysticks_SelectedIndexChanged`: a joystick that is not flying lets its device go.
    /// `// C#: Joystick/JoystickSetup.cs:489-499`
    fn device_changed(&mut self) {
        if let Some(joystick) = &mut self.joystick
            && !joystick.is_enabled()
        {
            joystick.unacquire();
        }
    }

    /// `CMB_joysticks_Click`: the list made again - emptied, which unselects, and its first row
    /// selected - and dropped down.
    /// `// C#: Joystick/JoystickSetup.cs:314-327`
    pub fn devices_click(&mut self) {
        self.page.items = self
            .source
            .devices()
            .into_iter()
            .map(|device| device.name)
            .collect();
        self.select_index(None);
        if !self.page.items.is_empty() {
            self.select_index(Some(0));
        }
        self.toggle_list(List::Devices);
    }

    // --- Enable, Save, Elevons, Manual Control -------------------------------------------------

    /// `BUT_enable_Click`: not flying, a new joystick made from the files - "all config is loaded
    /// from the xmls", so what the rows hold and Save has not written is not what it flies - with
    /// Elevons, and started on the device chosen; flying, control handed back and the joystick
    /// dropped. Manual Control is not carried over, as the C#'s new joystick does not carry it: it
    /// flies `RC_CHANNELS_OVERRIDE` until the box is clicked again (the facts'
    /// `config.joystick.mapping.manual` says which).
    /// `// C#: Joystick/JoystickSetup.cs:142-190`
    pub fn enable_click(&mut self, settings: &mut Persisted) {
        if self.is_enabled() {
            self.stop_flying();
            return;
        }
        if let Some(old) = &mut self.joystick {
            old.unacquire();
        }
        let mut joystick = Joystick::create(self.firmware, &self.data_dir, &self.ranges);
        joystick.mapping.elevons = self.page.elevons;
        let name = self.page.text.clone();
        if !joystick.start(&*self.source, &name, &self.wire) {
            // A box, "No Joystick", in the C#: an error, said on the status line.
            self.say(NO_JOYSTICK);
            return;
        }
        settings.set("joystick_name", name);
        joystick.flying = true;
        self.joystick = Some(joystick);
    }

    /// Enable's other half: `enabled = false`, `clearRCOverride` - the release, which the reader
    /// sends and repeats - and `MainV2.joystick = null`.
    /// `// C#: Joystick/JoystickSetup.cs:177-189; ExtLibs/ArduPilot/Joystick/JoystickBase.cs:294-364`
    pub fn stop_flying(&mut self) {
        if let Some(joystick) = self.joystick.take()
            && let Some(reader) = &joystick.reader
        {
            reader.set_enabled(false);
        }
        self.page.rc = [0; CHANNELS];
    }

    /// The flight screen's `but_disablejoystick_Click`: `enabled = false` and `clearRCOverride` -
    /// the release, which the reader sends and repeats - and, unlike the page's Disable, the
    /// joystick kept, acquired and no longer flying, as `MainV2.joystick` keeps it.
    /// `// C#: GCSViews/FlightData.cs:1211-1221; ExtLibs/ArduPilot/Joystick/JoystickBase.cs:294-364`
    pub fn disable_joystick(&mut self) {
        if !self.is_enabled() {
            return;
        }
        if let Some(reader) = self.joystick.as_ref().and_then(|j| j.reader.as_ref()) {
            reader.set_enabled(false);
        }
        if let Some(joystick) = &mut self.joystick {
            joystick.flying = false;
        }
        self.page.rc = [0; CHANNELS];
    }

    /// `BUT_save_Click`: the joystick's settings to its files, and Elevons to the settings.
    /// `// C#: Joystick/JoystickSetup.cs:192-202`
    pub fn save_click(&mut self, settings: &mut Persisted) {
        let Some(joystick) = &self.joystick else {
            self.say(SELECT_JOYSTICK);
            return;
        };
        if let Some(dir) = joystick
            .files
            .axis
            .parent()
            .filter(|dir| !dir.as_os_str().is_empty())
        {
            let _ = mp_os::fs::create_dir_all(dir);
        }
        match joystick.mapping.config.save(&joystick.files) {
            Ok(()) => {
                self.page.saved = Some(JoystickConfig::load(&joystick.files));
                self.page.saves += 1;
            }
            Err(err) => {
                // The C#'s `StreamWriter` throws to the unhandled-exception box.
                let words = format!("{}: {err}", joystick.files.axis.display());
                self.say(words);
                return;
            }
        }
        settings.set("joy_elevons", if self.page.elevons { "True" } else { "False" });
    }

    fn set_elevons(&mut self, checked: bool) {
        if self.page.elevons == checked {
            return;
        }
        self.page.elevons = checked;
        // `CHK_elevons_CheckedChanged`. `// C#: Joystick/JoystickSetup.cs:513-520`
        if let Some(joystick) = &mut self.joystick {
            joystick.mapping.elevons = checked;
            joystick.push();
        }
    }

    /// `CHK_elevons` clicked.
    pub fn toggle_elevons(&mut self) {
        self.set_elevons(!self.page.elevons);
    }

    /// `chk_manualcontrol` clicked: `MANUAL_CONTROL` rather than `RC_CHANNELS_OVERRIDE`. The C#
    /// throws with no joystick; here there is nothing to set.
    /// `// C#: Joystick/JoystickSetup.cs:522-525`
    pub fn toggle_manual(&mut self) {
        self.page.manual = !self.page.manual;
        let manual = self.page.manual;
        if let Some(joystick) = &mut self.joystick {
            joystick.mapping.manual_control = manual;
            joystick.push();
        }
    }

    // --- The channel rows ----------------------------------------------------------------------

    /// A row's axis chosen: `CMB_CH_SelectedIndexChanged`, then `SetAxis`.
    /// `// C#: Joystick/JoystickAxis.cs:195-198; JoystickSetup.cs:93-94`
    pub fn choose_axis(&mut self, row: usize, axis: JoystickAxis) {
        let Some(slot) = self.page.rows.get_mut(row) else {
            return;
        };
        if slot.axis == axis.name() {
            return;
        }
        axis.name().clone_into(&mut slot.axis);
        if let Some(joystick) = &mut self.joystick {
            joystick.mapping.config.set_axis(row + 1, axis);
            joystick.push();
        }
    }

    /// A row's Reverse clicked: `revCH_CheckedChanged`, then `setReverse`.
    /// `// C#: Joystick/JoystickAxis.cs:210-213; JoystickSetup.cs:92`
    pub fn toggle_reverse(&mut self, row: usize) {
        let Some(slot) = self.page.rows.get_mut(row) else {
            return;
        };
        slot.reverse = !slot.reverse;
        let reverse = slot.reverse;
        if let Some(joystick) = &mut self.joystick {
            joystick.mapping.config.set_reverse(row + 1, reverse);
            joystick.push();
        }
    }

    /// A row's expo typed into: `expo_ch_TextChanged`, then `setExpo` when the text is a whole
    /// number - `int.TryParse` - and nothing otherwise.
    /// `// C#: Joystick/JoystickAxis.cs:205-208; JoystickSetup.cs:100-106`
    fn expo_changed(&mut self, row: usize) {
        let Some(expo) = self
            .page
            .rows
            .get(row)
            .and_then(|slot| slot.expo.value().trim().parse::<i32>().ok())
        else {
            return;
        };
        if let Some(joystick) = &mut self.joystick {
            joystick.mapping.config.set_expo(row + 1, expo);
            joystick.push();
        }
    }

    /// A row's expo box clicked into.
    pub fn begin_expo(&mut self, row: usize) {
        self.commit_editing();
        self.page.open = None;
        self.page.editing = Some(Editing::Expo(row));
    }

    /// A key while something has the keyboard.
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        match self.page.editing {
            Some(Editing::Expo(row)) => {
                let Some(slot) = self.page.rows.get_mut(row) else {
                    return false;
                };
                match slot.expo.key(event) {
                    KeyOutcome::Changed => self.expo_changed(row),
                    KeyOutcome::Submitted | KeyOutcome::Cancelled => self.page.editing = None,
                    KeyOutcome::Ignored => {}
                }
                true
            }
            Some(Editing::Form(index)) => {
                let Some(number) = self
                    .page
                    .form
                    .as_mut()
                    .and_then(|form| form.numbers.get_mut(index))
                else {
                    return false;
                };
                match number.text.key(event) {
                    KeyOutcome::Submitted => self.form_commit(index),
                    KeyOutcome::Cancelled => self.page.editing = None,
                    KeyOutcome::Changed | KeyOutcome::Ignored => {}
                }
                true
            }
            Some(Editing::Path) => {
                let Some((_, path)) = self.page.path.as_mut() else {
                    return false;
                };
                match path.field.key(event) {
                    KeyOutcome::Submitted => self.path_done(true),
                    KeyOutcome::Cancelled => self.path_done(false),
                    KeyOutcome::Changed | KeyOutcome::Ignored => {}
                }
                true
            }
            None => false,
        }
    }

    /// Typed text into whatever has the keyboard, for the tests; on screen each key is a
    /// [`Self::key`].
    #[cfg(test)]
    pub fn type_text(&mut self, text: &str) {
        match self.page.editing {
            Some(Editing::Expo(row)) => {
                if let Some(slot) = self.page.rows.get_mut(row) {
                    slot.expo.insert(text);
                    self.expo_changed(row);
                }
            }
            Some(Editing::Form(index)) => {
                if let Some(number) = self
                    .page
                    .form
                    .as_mut()
                    .and_then(|form| form.numbers.get_mut(index))
                {
                    number.text.insert(text);
                }
            }
            Some(Editing::Path) => {
                if let Some((_, path)) = self.page.path.as_mut() {
                    path.field.insert(text);
                }
            }
            None => {}
        }
    }

    /// A form box being typed into is read when another control is used, as a `NumericUpDown`
    /// validates when it loses the focus.
    fn commit_editing(&mut self) {
        if let Some(Editing::Form(index)) = self.page.editing {
            self.form_commit(index);
        }
        self.page.editing = None;
    }

    // --- Detect -----------------------------------------------------------------------------------

    /// The device chosen, opened on its own for a detection, as `getJoyStickByName` opens it.
    fn open_detector(&self) -> Option<std::io::Result<StickReader>> {
        let device = self
            .source
            .devices()
            .into_iter()
            .find(|device| device.name == self.page.text)?;
        Some(self.source.open(&device).and_then(|stream| {
            // Never switched on, so its sink is never asked.
            StickReader::spawn(stream, Mapping::default(), |_frame| false)
        }))
    }

    fn start_detect(&mut self, kind: DetectKind, text: &'static str) -> bool {
        match self.open_detector() {
            None => false,
            Some(Ok(reader)) => {
                self.page.detect = Some(Detect {
                    kind,
                    reader,
                    opened: Instant::now(),
                    before: None,
                    since: None,
                });
                self.page.message = Some(Message {
                    title: "",
                    text,
                    after: After::Watch,
                });
                true
            }
            Some(Err(err)) => {
                // The C#'s `GetCurrentState` on a device that did not open returns null and the
                // next line throws; the reason is said instead.
                let words = format!("{}: {err}", self.page.text);
                self.say(words);
                true
            }
        }
    }

    /// A channel row's Auto Detect: `getMovingAxis(CMB_joysticks.Text, 16000)`. A device of that
    /// name that is not there is `ARx` at once, as the C# answers it.
    /// `// C#: Joystick/JoystickAxis.cs:200-203; JoystickSetup.cs:91; JoystickBase.cs:1174-1270`
    pub fn detect_axis(&mut self, row: usize) {
        self.commit_editing();
        self.page.open = None;
        if !self.start_detect(DetectKind::Axis(row), MOVE_AXIS) {
            self.choose_axis(row, JoystickAxis::ARx);
        }
    }

    /// A button row's Detect: `getPressedButton(CMB_joysticks.Text)`, -1 at once for a device
    /// that is not there.
    /// `// C#: Joystick/JoystickSetup.cs:339-345; JoystickBase.cs:186-222`
    pub fn detect_button(&mut self, slot: usize) {
        self.commit_editing();
        self.page.open = None;
        if !self.start_detect(DetectKind::Button(slot), PRESS_BUTTON) {
            self.choose_button_number(slot, -1);
        }
    }

    /// The watch: the state before taken once the C#'s sleep has passed, then, from the
    /// instruction's OK, ten seconds for an axis to move 16000 or a button to change.
    fn detect_tick(&mut self) {
        let Some(detect) = self.page.detect.as_mut() else {
            return;
        };
        let reading = detect.reader.reading();
        if detect.before.is_none() && detect.opened.elapsed() >= detect.kind.settle() {
            detect.before = Some(reading.clone());
        }
        let (Some(since), Some(before)) = (detect.since, detect.before.as_ref()) else {
            return;
        };
        let kind = detect.kind;
        let found = match kind {
            DetectKind::Axis(_) => moving_axis(before, &reading).map(|axis| axis.name()),
            DetectKind::Button(_) => None,
        };
        let pressed = match kind {
            DetectKind::Button(_) => pressed_button(before, &reading),
            DetectKind::Axis(_) => None,
        };
        let expired = since.elapsed() > DETECT_WINDOW;
        match kind {
            DetectKind::Axis(row) => {
                if let Some(name) = found {
                    self.page.detect = None;
                    if let Some(axis) = JoystickAxis::from_name(name) {
                        self.choose_axis(row, axis);
                    }
                } else if expired {
                    self.page.detect = None;
                    // A box in the C#: an error, said on the status line. `None` comes back, and
                    // the row is set to it.
                    self.say(NOTHING_DETECTED);
                    self.choose_axis(row, JoystickAxis::None);
                }
            }
            DetectKind::Button(slot) => {
                if let Some(number) = pressed {
                    self.page.detect = None;
                    self.choose_button_number(slot, i32::try_from(number).unwrap_or(-1));
                } else if expired {
                    self.page.detect = None;
                    self.say(NOTHING_DETECTED);
                    self.choose_button_number(slot, -1);
                }
            }
        }
    }

    // --- The button rows ------------------------------------------------------------------------

    /// A row's button number chosen: `cmbbutton_SelectedIndexChanged`, `changeButton`.
    /// `// C#: Joystick/JoystickSetup.cs:329-337`
    pub fn choose_button_number(&mut self, slot: usize, number: i32) {
        let Some(row) = self.page.buttons.get_mut(slot) else {
            return;
        };
        let text = number.to_string();
        if row.number == text || !BUTTON_NUMBERS.contains(&number) {
            return;
        }
        row.number = text;
        if self.page.startup {
            return;
        }
        if let Some(joystick) = &mut self.joystick {
            joystick.mapping.config.change_button(slot, number);
            joystick.push();
        }
    }

    /// A row's function chosen: `cmbaction_SelectedIndexChanged`.
    /// `// C#: Joystick/JoystickSetup.cs:444-451`
    pub fn choose_action(&mut self, slot: usize, function: ButtonFunction) {
        let Some(row) = self.page.buttons.get_mut(slot) else {
            return;
        };
        if row.action == function.name() {
            return;
        }
        function.name().clone_into(&mut row.action);
        if let Some(joystick) = &mut self.joystick {
            let mut button = joystick.mapping.config.button(slot);
            button.function = function;
            joystick.mapping.config.set_button(slot, button);
            joystick.push();
        }
    }

    /// A row's Settings: the form for the function its combo shows, or "No settings to set".
    /// `// C#: Joystick/JoystickSetup.cs:453-487`
    pub fn settings_click(&mut self, slot: usize) {
        self.commit_editing();
        self.page.open = None;
        let Some(function) = self
            .page
            .buttons
            .get(slot)
            .and_then(|row| ButtonFunction::from_name(&row.action))
        else {
            return;
        };
        let Some(kind) = FormKind::of(function) else {
            self.page.message = Some(Message {
                title: NO_SETTINGS_TITLE,
                text: NO_SETTINGS,
                after: After::Nothing,
            });
            return;
        };
        let firmware = self.firmware;
        let Some(joystick) = &mut self.joystick else {
            return;
        };
        let mut button = joystick.mapping.config.button(slot);
        let form = Form::open(
            kind,
            slot,
            &mut button,
            modes_list(firmware),
            mount_mode_options(),
        );
        joystick.mapping.config.set_button(slot, button);
        joystick.push();
        self.page.form = Some(form);
    }

    /// Changes the open form's button through `change`, which says whether it wrote.
    fn with_form_button(&mut self, change: impl FnOnce(&mut Form, &mut JoyButton) -> bool) {
        let Some(form) = self.page.form.as_mut() else {
            return;
        };
        let Some(joystick) = &mut self.joystick else {
            return;
        };
        let mut button = joystick.mapping.config.button(form.slot);
        if change(form, &mut button) {
            joystick.mapping.config.set_button(form.slot, button);
            joystick.push();
        }
    }

    /// The form's combo: a row chosen.
    pub fn form_choose(&mut self, index: usize) {
        self.with_form_button(|form, button| form.choose(index, button));
    }

    /// A form box's arrow.
    pub fn form_step(&mut self, index: usize, up: bool) {
        self.commit_editing();
        self.with_form_button(|form, button| form.step(index, up, button));
    }

    /// A form box clicked into.
    pub fn form_begin(&mut self, index: usize) {
        self.commit_editing();
        self.page.open = None;
        self.page.editing = Some(Editing::Form(index));
    }

    fn form_commit(&mut self, index: usize) {
        self.page.editing = None;
        self.with_form_button(|form, button| form.commit(index, button));
    }

    /// The form's close box: a box being typed into read, as it loses the focus, and the form
    /// gone.
    pub fn form_close(&mut self) {
        self.commit_editing();
        self.page.open = None;
        self.page.form = None;
    }

    // --- The drop-downs ------------------------------------------------------------------------

    /// A combo clicked: its list dropped down on its selected row, or put away.
    pub fn toggle_list(&mut self, list: List) {
        self.commit_editing();
        if self.page.open == Some(list) {
            self.page.open = None;
            return;
        }
        self.page.open = Some(list);
        let (rows, selected) = self.list_rows(list);
        let shown = crate::config::servo_output::LIST_ROWS_SHOWN;
        self.page.top = selected
            .unwrap_or(0)
            .saturating_sub(shown - 1)
            .min(rows.len().saturating_sub(shown));
    }

    /// The wheel over the open list.
    pub fn scroll_list(&mut self, lines: i32) {
        let Some(list) = self.page.open else {
            return;
        };
        let rows = self.list_rows(list).0.len();
        let shown = crate::config::servo_output::LIST_ROWS_SHOWN;
        let step = usize::try_from(lines.unsigned_abs()).unwrap_or(usize::MAX);
        let top = if lines < 0 {
            self.page.top.saturating_sub(step)
        } else {
            self.page.top.saturating_add(step)
        };
        self.page.top = top.min(rows.saturating_sub(shown));
    }

    /// A list's rows, each its id suffix and text, and the row selected.
    #[must_use]
    pub fn list_rows(&self, list: List) -> (Vec<(String, String)>, Option<usize>) {
        match list {
            List::Devices => (
                self.page
                    .items
                    .iter()
                    .enumerate()
                    .map(|(index, name)| (index.to_string(), name.clone()))
                    .collect(),
                self.page.selected,
            ),
            List::Axis(row) => {
                let current = self.page.rows.get(row).map(|slot| slot.axis.as_str());
                (
                    JoystickAxis::ALL
                        .iter()
                        .map(|axis| (axis.name().to_owned(), axis.name().to_owned()))
                        .collect(),
                    JoystickAxis::ALL
                        .iter()
                        .position(|axis| Some(axis.name()) == current),
                )
            }
            List::Number(slot) => {
                let current = self.page.buttons.get(slot).map(|row| row.number.as_str());
                (
                    button_numbers()
                        .map(|number| (number.to_string(), number.to_string()))
                        .collect(),
                    button_numbers().position(|number| Some(number.to_string().as_str()) == current),
                )
            }
            List::Action(slot) => {
                let current = self.page.buttons.get(slot).map(|row| row.action.as_str());
                (
                    ButtonFunction::ALL
                        .iter()
                        .map(|function| (function.name().to_owned(), function.name().to_owned()))
                        .collect(),
                    ButtonFunction::ALL
                        .iter()
                        .position(|function| Some(function.name()) == current),
                )
            }
            List::Form => self
                .page
                .form
                .as_ref()
                .and_then(|form| form.combo.as_ref())
                .map_or((Vec::new(), None), |combo| {
                    (
                        combo
                            .options
                            .iter()
                            .enumerate()
                            .map(|(index, (_, name))| (index.to_string(), name.clone()))
                            .collect(),
                        combo.selected,
                    )
                }),
        }
    }

    /// A row of the open list chosen.
    pub fn choose_from_list(&mut self, index: usize) {
        let Some(list) = self.page.open.take() else {
            return;
        };
        match list {
            List::Devices => self.select_index(Some(index)),
            List::Axis(row) => {
                if let Some(axis) = JoystickAxis::ALL.get(index) {
                    self.choose_axis(row, *axis);
                }
            }
            List::Number(slot) => {
                if let Some(number) = button_numbers().nth(index) {
                    self.choose_button_number(slot, number);
                }
            }
            List::Action(slot) => {
                if let Some(function) = ButtonFunction::ALL.get(index) {
                    self.choose_action(slot, *function);
                }
            }
            List::Form => self.form_choose(index),
        }
    }

    // --- Message boxes and file dialogs --------------------------------------------------------

    /// A message box's OK.
    pub fn message_ok(&mut self) {
        let Some(message) = self.page.message.take() else {
            return;
        };
        match message.after {
            After::Nothing => {}
            After::Watch => {
                if let Some(detect) = self.page.detect.as_mut() {
                    if detect.before.is_none() {
                        detect.before = Some(detect.reader.reading());
                    }
                    detect.since = Some(Instant::now());
                }
            }
            // `this.Close(); ((Form)this.Parent).Close()`: the page's `FormClosed`, and its
            // window - the Planner's - closed. In Initial Setup the parent is not a form, the
            // cast throws, and the page stays in the list.
            // `// C#: Joystick/JoystickSetup.cs:560-564`
            After::CloseAfterImport => {
                let host = self.page.host;
                self.close();
                self.page.close_parent = host == Some(Host::Planner);
            }
        }
    }

    /// Whether Import asked for the Planner's window to close; asked once.
    pub fn take_close_parent(&mut self) -> bool {
        std::mem::take(&mut self.page.close_parent)
    }

    /// `but_export_Click`: the `SaveFileDialog`, opening in the user data directory.
    /// `// C#: Joystick/JoystickSetup.cs:538-547`
    pub fn export_click(&mut self) {
        self.commit_editing();
        self.page.open = None;
        let mut path = PathBox::new(&self.data_dir.display().to_string(), JOYCFG_FILTER);
        path.caption = SAVE_AS;
        self.page.path = Some((PathUse::Export, path));
        self.page.editing = Some(Editing::Path);
    }

    /// `but_import_Click`: the question first.
    /// `// C#: Joystick/JoystickSetup.cs:549-567`
    pub fn import_click(&mut self) {
        self.commit_editing();
        self.page.open = None;
        self.page.question = true;
    }

    /// Import's question answered: OK opens the `OpenFileDialog`.
    pub fn question_answer(&mut self, ok: bool) {
        self.page.question = false;
        if ok {
            let path = PathBox::new(&self.data_dir.display().to_string(), JOYCFG_FILTER);
            self.page.path = Some((PathUse::Import, path));
            self.page.editing = Some(Editing::Path);
        }
    }

    /// The file dialog's OK or Cancel.
    pub fn path_done(&mut self, ok: bool) {
        self.page.editing = None;
        let Some((purpose, path)) = self.page.path.take() else {
            return;
        };
        if !ok {
            return;
        }
        match purpose {
            PathUse::Export => {
                let file = PathBuf::from(path.field.value());
                if path.field.value().trim().is_empty() {
                    return;
                }
                self.export(&file);
            }
            // `OpenFileDialog` takes only a file that exists.
            PathUse::Import => {
                if let Some(file) = path.chosen() {
                    self.import(&file);
                }
            }
        }
    }

    /// Export's OK: `saveconfig`, then `ExportConfig` - every `joystickbutton*.xml` and
    /// `joystickaxis*.xml` in the user data directory into a zip. What throws in the C# - no
    /// files, a write refused - is said on the status line.
    /// `// C#: Joystick/JoystickSetup.cs:542-546; ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1308-1335`
    fn export(&mut self, to: &Path) {
        let Some(joystick) = &self.joystick else {
            return;
        };
        if let Err(err) = joystick.mapping.config.save(&joystick.files) {
            let words = format!("{}: {err}", joystick.files.axis.display());
            self.say(words);
            return;
        }
        match export_config(&self.data_dir, to) {
            Ok(count) => self.page.exported = Some((to.to_path_buf(), count)),
            Err(words) => self.say(words),
        }
    }

    /// Import's file: `ImportConfig`, `loadconfig()` - the default names, not the firmware's - and
    /// "Please reopen joystick for changes to take effect", whose OK closes the page.
    /// `// C#: Joystick/JoystickSetup.cs:555-565; ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1337-1360, 107-158`
    fn import(&mut self, from: &Path) {
        if let Err(words) = import_config(&self.data_dir, from) {
            self.say(words);
            return;
        }
        if let Some(joystick) = &mut self.joystick {
            // `loadconfig()`'s own defaults: joystickbuttons.xml and joystickaxis.xml.
            joystick.files = ConfigFiles::for_firmware("", &self.data_dir);
            joystick.mapping.config = JoystickConfig::load(&joystick.files);
            joystick.push();
        }
        self.page.message = Some(Message {
            title: "",
            text: REOPEN,
            after: After::CloseAfterImport,
        });
    }
}

/// [`BUTTON_NUMBERS`], to iterate.
const fn button_numbers() -> std::ops::RangeInclusive<i32> {
    BUTTON_NUMBERS
}

/// `bool.Parse`: "True" or "False", any case, around white space.
fn parse_bool(text: &str) -> Option<bool> {
    match text.trim().to_ascii_lowercase().as_str() {
        "true" => Some(true),
        "false" => Some(false),
        _ => None,
    }
}

/// A parameter's value, if the vehicle has listed it.
fn parameter(view: &TelemetryView, name: &str) -> Option<f64> {
    view.parameters
        .iter()
        .find(|(held, _)| held == name)
        .map(|(_, value)| *value)
}

/// `getMovingAxis`'s comparison: the first property, in declaration order, more than 16000 from
/// where it was; then `Slider1`, `Slider2` and the hats, which on Linux do not move.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1206-1265`
fn moving_axis(before: &Reading, now: &Reading) -> Option<JoystickAxis> {
    let moved = |axis: JoystickAxis| {
        let (Some(was), Some(is)) = (axis_value(axis, before), axis_value(axis, now)) else {
            return false;
        };
        was > is + DETECT_THRESHOLD || was < is - DETECT_THRESHOLD
    };
    MOVING_AXES
        .into_iter()
        .chain([JoystickAxis::Slider1, JoystickAxis::Slider2])
        .find(|axis| moved(*axis))
}

/// `getPressedButton`'s comparison: the first of the device's buttons that differs.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:206-217`
fn pressed_button(before: &Reading, now: &Reading) -> Option<usize> {
    (0..now.buttons.len()).find(|number| before.button(*number) != now.button(*number))
}

/// `Common.getModesList(cs.firmware)`, which `Joy_ChangeMode` lists.
/// `// C#: ExtLibs/ArduPilot/Common.cs:88-182`
fn modes_list(firmware: Firmware) -> Vec<(i64, String)> {
    if firmware == Firmware::ArduTracker {
        return [
            (0, "MANUAL"),
            (1, "STOP"),
            (2, "SCAN"),
            (3, "SERVO_TEST"),
            (10, "AUTO"),
            (16, "INITIALISING"),
        ]
        .into_iter()
        .map(|(key, name)| (key, name.to_owned()))
        .collect();
    }
    crate::config::flight_modes::modes_list(firmware, |name| {
        crate::config::flight_modes::documented_values(firmware, name)
    })
}

/// `Joy_Mount_Mode`'s list: the first of its three parameters with documented options.
/// `// C#: Joystick/Joy_Mount_Mode.cs:18-29`
fn mount_mode_options() -> Vec<(i64, String)> {
    forms::MOUNT_MODE_PARAMS
        .iter()
        .map(|name| crate::config::failsafe::options(name, crate::metadata::lookup))
        .find(|options| !options.is_empty())
        .unwrap_or_default()
}

/// `ExportConfig`: the joystick files of `dir` into a new zip at `to`, buttons first; how many.
///
/// # Errors
/// No files to export, or the archive not written: what the C# throws, as words.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1308-1335`
pub fn export_config(dir: &Path, to: &Path) -> Result<usize, String> {
    let names = |prefix: &str| {
        let mut names: Vec<String> = mp_os::fs::read_dir(dir)
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .filter(|entry| entry.path().os_is_file())
                    .filter_map(|entry| entry.file_name().to_str().map(str::to_owned))
                    .filter(|name| name.starts_with(prefix) && name.ends_with(".xml"))
                    .collect()
            })
            .unwrap_or_default();
        names.sort();
        names
    };
    let files: Vec<String> = names("joystickbutton")
        .into_iter()
        .chain(names("joystickaxis"))
        .collect();
    if files.is_empty() {
        return Err(format!("{NO_CONFIG_FILES}{}", dir.display()));
    }
    let mut entries = Vec::new();
    for name in &files {
        let data = mp_os::fs::read(dir.join(name)).map_err(|err| format!("{name}: {err}"))?;
        entries.push(mp_log::zip::Entry {
            name: name.clone(),
            data,
        });
    }
    let now = chrono::Local::now();
    let stamp = {
        use chrono::{Datelike, Timelike};
        let part = |value: u32| u16::try_from(value).unwrap_or(0);
        mp_log::zip::DosTime {
            year: u16::try_from(now.year()).unwrap_or(1980),
            month: part(now.month()),
            day: part(now.day()),
            hour: part(now.hour()),
            minute: part(now.minute()),
            second: part(now.second()),
        }
    };
    let zip = mp_log::zip::write(&entries, stamp).map_err(|err| err.to_string())?;
    if to.os_exists() {
        mp_os::fs::remove_file(to).map_err(|err| format!("{}: {err}", to.display()))?;
    }
    mp_os::fs::write(to, zip).map_err(|err| format!("{}: {err}", to.display()))?;
    crate::page_files::saved(to);
    Ok(files.len())
}

/// `ZipArchiveEntry.Name` on Windows: the entry's name after its last `/`, `\` or `:`, so a name
/// can never climb out of the folder it is written to - `joystickaxis\..\..\x` is `x`, which is
/// not a joystick file. Taking it after `/` alone let such an entry pass as a joystick file.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1353-1356`
fn entry_name(full: &str) -> &str {
    full.rsplit(['/', '\\', ':']).next().unwrap_or("")
}

/// `ImportConfig`: the joystick files in the zip at `from` into `dir`, over what is there.
///
/// # Errors
/// Not a zip, or a file not written: what the C# throws, as words.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1337-1360`
pub fn import_config(dir: &Path, from: &Path) -> Result<usize, String> {
    let bytes = mp_os::fs::read(from).map_err(|err| format!("{}: {err}", from.display()))?;
    let entries = mp_log::zip::read(&bytes).map_err(|err| err.to_string())?;
    let mut count = 0;
    for entry in entries {
        let name = entry_name(&entry.name);
        if !mp_input::config::is_config_file(name) {
            continue;
        }
        let _ = mp_os::fs::create_dir_all(dir);
        mp_os::fs::write(dir.join(name), &entry.data)
            .map_err(|err| format!("{name}: {err}"))?;
        count += 1;
    }
    Ok(count)
}

// ---------------------------------------------------------------------------------------------
// The button functions.
// ---------------------------------------------------------------------------------------------

/// A button function the reader handed over: `ProcessButtonEvent`'s switch, each case sending what
/// the C#'s does to the vehicle being flown. A failure the C#'s `catch` puts in a box comes back
/// as words for the status line, now or when the request ends; the button axes never arrive here.
/// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:392-622`
pub fn perform(
    event: &ButtonEvent,
    telemetry: &mut Telemetry,
    view: &TelemetryView,
    firmware: Firmware,
) -> Option<String> {
    let (sender, target) = telemetry.send_handle()?;
    let button = &event.button;
    let family = view
        .state
        .as_ref()
        .and_then(|state| mp_vehicle::VehicleFamily::from_mav_type(state.vehicle_type));
    // `(int) but.p1`, then passed as the command's `float`.
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
    let int = |value: f32| value as i32 as f32;
    let command = |id: MavCmd| u16::try_from(id.0).unwrap_or(u16::MAX);
    let failed = |what: &str| Report::on_timeout(error_box(what));
    match button.function {
        ButtonFunction::ChangeMode => {
            if let Some(mode) = &button.mode {
                for message in crate::fly::set_mode_messages(target, family, mode) {
                    sender.send(&message);
                }
            }
        }
        ButtonFunction::MountMode => {
            telemetry.set_parameter_on(
                target,
                "MNT_MODE",
                f64::from(button.p1),
                false,
                failed("Failed to change mount mode"),
            );
        }
        ButtonFunction::Arm | ButtonFunction::Disarm => {
            let arm = button.function == ButtonFunction::Arm;
            telemetry.command_message(
                &mp_link::commands::arm(target, arm, false),
                failed(if arm { "Failed to Arm" } else { "Failed to Disarm" }),
            );
        }
        ButtonFunction::TakeOff => {
            // `Interface.setMode("Guided")`, then the take-off: 2 m for a copter, 20 otherwise.
            for message in crate::fly::set_mode_messages(target, family, "Guided") {
                sender.send(&message);
            }
            let altitude = if firmware == Firmware::ArduCopter2 {
                2.0
            } else {
                20.0
            };
            telemetry.command(
                target,
                mp_link::commands::CMD_NAV_TAKEOFF,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude],
                failed("Failed to takeoff"),
            );
        }
        ButtonFunction::DoSetRelay => {
            let state = if event.down { 1.0 } else { 0.0 };
            telemetry.command(
                target,
                command(MavCmd::MAV_CMD_DO_SET_RELAY),
                [int(button.p1), state, 0.0, 0.0, 0.0, 0.0, 0.0],
                failed("Failed to DO_SET_RELAY"),
            );
        }
        // `setDigicamControl(true)`, called outside the `try`: a timeout says nothing, and a
        // refusal sends `DIGICAM_CONTROL`. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4558-4570`
        ButtonFunction::DigicamControl => {
            telemetry.command(
                target,
                mp_link::commands::CMD_DO_DIGICAM_CONTROL,
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                Report {
                    fallback: Some(mp_mavlink_dialects::all::MavMessage::DigicamControl(
                        mp_mavlink_dialects::all::DigicamControl {
                            extra_value: 0.0,
                            target_system: target.sysid,
                            target_component: target.compid,
                            session: 0,
                            zoom_pos: 0,
                            zoom_step: 0,
                            focus_lock: 0,
                            shot: 1,
                            command_id: 0,
                            extra_param: 0,
                        },
                    )),
                    ..Report::default()
                },
            );
        }
        ButtonFunction::DoRepeatRelay => {
            telemetry.command(
                target,
                command(MavCmd::MAV_CMD_DO_REPEAT_RELAY),
                [
                    int(button.p1),
                    int(button.p2),
                    int(button.p3),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                failed("Failed to DO_REPEAT_RELAY"),
            );
        }
        ButtonFunction::DoSetServo => {
            telemetry.command(
                target,
                command(MavCmd::MAV_CMD_DO_SET_SERVO),
                [int(button.p1), int(button.p2), 0.0, 0.0, 0.0, 0.0, 0.0],
                failed("Failed to DO_SET_SERVO"),
            );
        }
        ButtonFunction::DoRepeatServo => {
            telemetry.command(
                target,
                command(MavCmd::MAV_CMD_DO_REPEAT_SERVO),
                [
                    int(button.p1),
                    int(button.p2),
                    int(button.p3),
                    int(button.p4),
                    0.0,
                    0.0,
                    0.0,
                ],
                failed("Failed to DO_REPEAT_SERVO"),
            );
        }
        ButtonFunction::TogglePanStab => {
            // `(float) Interface.MAV.param["MNT_STAB_PAN"]` throws for a vehicle without it.
            let Some(current) = parameter(view, "MNT_STAB_PAN") else {
                return Some(error_box("Failed to Toggle_Pan_Stab"));
            };
            let value = if current > 0.0 { 0.0 } else { 1.0 };
            telemetry.set_parameter_on(
                target,
                "MNT_STAB_PAN",
                value,
                false,
                failed("Failed to Toggle_Pan_Stab"),
            );
        }
        ButtonFunction::GimbalPntTrack => {
            // `GimbalPoint.Alt` throws while there is no gimbal point.
            let Some((state, point)) = view
                .state
                .as_ref()
                .and_then(|state| state.gimbal_point.map(|point| (state, point)))
            else {
                return Some(error_box("Failed to Gimbal_pnt_track"));
            };
            #[allow(clippy::cast_possible_truncation)] // `(int)(gimballat * 1e7)`, `(float)Alt`
            let (x, y, z) = (
                (f64::from(state.gimbal_lat()) * 1e7) as i32,
                (f64::from(state.gimbal_lng()) * 1e7) as i32,
                point.alt as f32,
            );
            telemetry.command_int(
                target,
                command(MavCmd::MAV_CMD_DO_SET_ROI),
                mp_link::commands::FRAME_GLOBAL,
                [0.0; 4],
                x,
                y,
                z,
                failed("Failed to Gimbal_pnt_track"),
            );
        }
        // `setMountControl(0, 0, 0, false)`: `DO_MOUNT_CONTROL` in the MAVLink-targeting mode,
        // not waited for. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4592-4606`
        ButtonFunction::MountControl0 => {
            sender.send(&mp_link::commands::command_long(
                target,
                command(MavCmd::MAV_CMD_DO_MOUNT_CONTROL),
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
            ));
        }
        ButtonFunction::ButtonAxis0 | ButtonFunction::ButtonAxis1 => {}
    }
    None
}

impl MissionPlanner {
    /// Once a frame: the sticks kept pointed at the vehicle, the page's timer, the Planner's
    /// window opening and closing the page, the button functions pressed done - "disable button
    /// actions when not connected" - and what was said put on the status line.
    /// `// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1129-1131`
    pub(crate) fn joystick_tick(&mut self, view: &TelemetryView) {
        let banner = self.telemetry.firmware_banner().map(str::to_owned);
        self.sticks
            .tick(self.telemetry.send_handle(), view, banner.as_deref());
        if self.sticks.take_close_parent() {
            self.planner.close_joystick();
        }
        match (self.planner.joystick_open(), self.sticks.page.host) {
            (true, None) => self.sticks.load(Host::Planner, &self.persisted),
            (false, Some(Host::Planner)) => self.sticks.close(),
            _ => {}
        }
        for event in self.sticks.take_button_events() {
            if !view.connected {
                continue;
            }
            let firmware = self.sticks.firmware();
            if let Some(words) = perform(&event, &mut self.telemetry, view, firmware) {
                self.sticks.say(words);
            }
        }
        for said in self.sticks.take_said() {
            self.file_status = Some(said);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// What the page shows and the joystick holds, for a test.
pub fn record_facts(sticks: &Sticks) {
    use crate::facts::record;
    let page = &sticks.page;
    let joystick = sticks.joystick.as_ref();
    record(
        "config.joystick.active",
        page.host.is_some(),
    );
    record(
        "config.joystick.host",
        match page.host {
            Some(Host::Setup) => "setup",
            Some(Host::Planner) => "planner",
            None => "none",
        },
    );
    record("config.joystick.timer", page.timer);
    record("config.joystick.devices", page.items.len());
    record("config.joystick.device", &page.text);
    record("config.joystick.exists", joystick.is_some());
    record(
        "config.joystick.acquired",
        joystick.is_some_and(Joystick::is_acquired),
    );
    record("config.joystick.enabled", sticks.is_enabled());
    record("config.joystick.enable", sticks.enable_text());
    record("config.joystick.label", &page.label);
    record("config.joystick.elevons", page.elevons);
    record("config.joystick.manual", page.manual);
    record(
        "config.joystick.mapping.elevons",
        joystick.is_some_and(|j| j.mapping.elevons),
    );
    record(
        "config.joystick.mapping.manual",
        joystick.is_some_and(|j| j.mapping.manual_control),
    );
    for (index, value) in page.rc.iter().enumerate().take(CHANNEL_ROWS) {
        record(format!("config.joystick.rc{}", index + 1), value);
    }
    for (index, row) in page.rows.iter().enumerate() {
        let channel = index + 1;
        record(format!("config.joystick.rc{channel}.axis"), &row.axis);
        record(format!("config.joystick.rc{channel}.expo"), row.expo.value());
        record(format!("config.joystick.rc{channel}.reverse"), row.reverse);
        if let Some(joystick) = joystick {
            let config = joystick.mapping.config.channel(channel);
            record(
                format!("config.joystick.config.rc{channel}.axis"),
                config.axis.name(),
            );
            record(format!("config.joystick.config.rc{channel}.expo"), config.expo);
            record(
                format!("config.joystick.config.rc{channel}.reverse"),
                config.reverse,
            );
        }
    }
    record("config.joystick.buttons", page.buttons.len());
    for (slot, row) in page.buttons.iter().enumerate() {
        record(format!("config.joystick.but{slot}.buttonno"), &row.number);
        record(format!("config.joystick.but{slot}.function"), &row.action);
        record(
            format!("config.joystick.but{slot}.pressed"),
            page.pressed.get(slot).copied().unwrap_or(false),
        );
        if let Some(joystick) = joystick {
            let button = joystick.mapping.config.button(slot);
            record(
                format!("config.joystick.config.but{slot}.buttonno"),
                button.buttonno,
            );
            record(
                format!("config.joystick.config.but{slot}.function"),
                button.function.name(),
            );
            record(
                format!("config.joystick.config.but{slot}.mode"),
                button.mode.as_deref().unwrap_or("none"),
            );
            for (name, value) in [
                ("p1", button.p1),
                ("p2", button.p2),
                ("p3", button.p3),
                ("p4", button.p4),
            ] {
                record(format!("config.joystick.config.but{slot}.{name}"), value);
            }
        }
    }
    record(
        "config.joystick.form",
        page.form.as_ref().map_or("none", |form| form.kind.class()),
    );
    if let Some(form) = &page.form {
        record("config.joystick.form.slot", form.slot);
        record(
            "config.joystick.form.combo",
            form.combo.as_ref().map_or("none", |combo| combo.text()),
        );
        for (index, number) in form.numbers.iter().enumerate() {
            record(format!("config.joystick.form.n{index}"), number.text.value());
        }
    }
    record(
        "config.joystick.message",
        page.message.as_ref().map_or("none", |message| message.text),
    );
    record("config.joystick.question", page.question);
    record(
        "config.joystick.path",
        page.path.as_ref().map_or("none", |(purpose, _)| match purpose {
            PathUse::Export => "export",
            PathUse::Import => "import",
        }),
    );
    record(
        "config.joystick.detect",
        match page.detect.as_ref().map(|detect| detect.kind) {
            None => "none".to_owned(),
            Some(DetectKind::Axis(row)) => format!("axis {}", row + 1),
            Some(DetectKind::Button(slot)) => format!("button {slot}"),
        },
    );
    record(
        "config.joystick.list",
        match page.open {
            None => "none".to_owned(),
            Some(List::Devices) => "devices".to_owned(),
            Some(List::Axis(row)) => format!("rc{}", row + 1),
            Some(List::Number(slot)) => format!("but{slot}.number"),
            Some(List::Action(slot)) => format!("but{slot}.action"),
            Some(List::Form) => "form".to_owned(),
        },
    );
    record(
        "config.joystick.files.axis",
        joystick.map_or_else(|| "none".to_owned(), |j| j.files.axis.display().to_string()),
    );
    record(
        "config.joystick.files.buttons",
        joystick.map_or_else(
            || "none".to_owned(),
            |j| j.files.buttons.display().to_string(),
        ),
    );
    record("config.joystick.saves", page.saves);
    if let Some(saved) = &page.saved {
        for channel in 1..=CHANNEL_ROWS {
            let config = saved.channel(channel);
            record(
                format!("config.joystick.saved.rc{channel}.axis"),
                config.axis.name(),
            );
            record(format!("config.joystick.saved.rc{channel}.expo"), config.expo);
            record(
                format!("config.joystick.saved.rc{channel}.reverse"),
                config.reverse,
            );
        }
        for slot in 0..BUTTON_ROWS {
            let button = saved.button(slot);
            record(
                format!("config.joystick.saved.but{slot}.function"),
                button.function.name(),
            );
            record(
                format!("config.joystick.saved.but{slot}.buttonno"),
                button.buttonno,
            );
            record(format!("config.joystick.saved.but{slot}.p1"), button.p1);
            record(format!("config.joystick.saved.but{slot}.p2"), button.p2);
        }
    }
    record(
        "config.joystick.exported",
        page.exported
            .as_ref()
            .map_or_else(|| "none".to_owned(), |(path, _)| path.display().to_string()),
    );
    record(
        "config.joystick.exported.files",
        page.exported.as_ref().map_or(0, |(_, count)| *count),
    );
    record(
        "config.joystick.status",
        page.status.as_deref().unwrap_or("none"),
    );
}

#[cfg(test)]
mod tests;
