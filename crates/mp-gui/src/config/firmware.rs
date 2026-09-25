//! Install Firmware: the first page of Initial Setup, without the flashing.
//!
//! Mission Planner lists two pages under that name and shows one: `ConfigFirmwareManifest` while
//! no vehicle is connected, `ConfigFirmwareDisabled` while one is (`GCSViews/InitialSetup.cs:
//! 165-174`, the legacy `ConfigFirmware` beside them being the older `firmware2.xml` catalogue,
//! `firmware_legacy.rs`). Which one is decided when the page opens, as the C# decides it when it
//! builds the list.
//!
//! `ConfigFirmwareManifest` is eleven vehicle pictures - Rover, Plane, Quad, Hexa, Octa Quad and
//! Sub above, Antenna Tracker, Heli, Tri, Y6 and Octa below, and a picture of a Cube that does
//! nothing - with a progress bar and a status line under them, and a row of links: All Options,
//! Bootloader Update, Force Bootloader, Load custom firmware and Beta firmwares. `Activate`
//! disables the pictures and the All Options and Beta links, fetches the firmware catalogue
//! (`mp_firmware::manifest`) on a background task, and labels each picture with the newest
//! firmware of its vehicle in the release being offered - `OFFICIAL` until Beta firmwares is
//! clicked - enabling it as it goes. Clicking a picture asks "Are you sure you want to upload
//! ...?"; Yes runs `LookForPort`, which finds the board among the USB devices and picks its
//! firmware - one file outright, several through `FirmwareSelection`, which starts at the board's
//! own platform and waits for "Upload Firmware" - downloads it to a temporary file with the
//! progress bar and status line following, and hands it to `UploadFlash`. All Options is
//! `LookForPort` over the whole catalogue. Load custom firmware opens a file and hands it to
//! `UploadFlash` by its extension (`mp_firmware::flow::custom_manifest`).
//!
//! Here all of that runs as the C# runs it - on a thread, with its questions and messages as
//! modal boxes over the page - up to the step that would write to a board, and stops there:
//! `UploadFlash` reads the file (`ProcessFirmware`) and stops before `AttemptRebootToBootloader`,
//! and DFU, the STK500 probes and the bootloader probe are stops of their own
//! (`mp_firmware::flow::Stop`). Beneath the page are the board it found, the firmware it chose,
//! the file it downloaded and what the file says of itself, and an Upload button that is disabled
//! and says why. The layout is `ConfigFirmwareManifest.Designer.cs`'s - every control at its
//! `Location` and `Size` in a 946 x 375 page - each picture its `Image`, zoomed as `ImageLabel`'s
//! `PictureBox` zooms it ([`crate::pictures`]), and this application's colours.
//!
//! What is not here, and why:
//!
//! * the upload itself and Bootloader Update write to the board: the upload through
//!   `flow::upload_px4` over this machine's ports (PLAN.md §13.6 row 79), Bootloader Update as
//!   `doCommand(FLASH_BOOTLOADER)` from the window once its two "Are you sure" questions are
//!   answered Yes. Force Bootloader is drawn dimmed;
//! * `Ctrl+Q` for the `DEV` release (`ProcessCmdKey`, `ConfigFirmwareManifest.cs:399-408`): the
//!   C# sees it only while a control of the page has the keyboard, and nothing on this page takes
//!   it; the catalogue answers for `DEV`, and `headless-planner firmware list --release DEV` asks it;
//! * the board id a bootloader reports when a device is plugged in while the page shows
//!   (`Instance_DeviceChanged`, `:134-180`): reading it means opening every port and sending the
//!   bootloader's identify. The board is found from the USB product string and ids alone, as
//!   `LookForPort` does when that probe has found nothing;
//! * `FirmwareSelection`'s filter pickers (`test/FirmwareSelection.xaml.cs:63-152`): the dialog
//!   opens as the C#'s does for a device - its Result list at the board's own platform, the one
//!   file selected when there is one - and a row can be chosen and "Upload Firmware" pressed, but
//!   the Type, Version and Platform pickers that narrow a long list are not drawn;
//! * `Tracking.AddFW` and `AddTiming`, Mission Planner's analytics.
//!
//! The device list is the port enumeration through `mp_firmware::detect::DeviceInfo::from_port`.
//! [`DEVICE_ENV`] replaces it with one named device, so a test does not depend on what is plugged
//! into the machine running it; with [`mp_firmware::manifest::OVERRIDE_ENV`] and
//! [`mp_firmware::manifest::MIRROR_ENV`] a test runs offline.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, TryRecvError, channel};

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px,
    rgb,
};
use mp_firmware::detect::{DeviceInfo, TransportPort, Win32SerialPort, open_serial};
use mp_firmware::flow::{self, Buttons, Dialogue, FlashHost, LinkReboot, Reached};
use mp_firmware::manifest::{
    self, Lookup, Manifest, MavType, NO_FIRMWARE, NO_PORT, Outcome, ReleaseType, Selection,
    icon_name,
};

use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::telemetry::TelemetryView;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{action, panel, theme};

/// Test scaffolding: the device list is this one device - its USB product string, and optionally
/// `|` and a hardware id - instead of the machine's ports.
pub const DEVICE_ENV: &str = "MP_FIRMWARE_DEVICE";

/// Test scaffolding until `MainV2`'s connection box is ported (PLAN.md §13.6 row 80): the serial
/// device `AttemptRebootToBootloader` opens, in place of `MainV2.comPortName` from the settings.
/// The application connects to the settings' `comport` when it starts (`settings.rs`,
/// `with_mission_planner_defaults`), and the Install Firmware page shows only while nothing is
/// connected, so the bench flash names the board's port here and leaves the settings' `comport`
/// unset. Never set in a shipped configuration.
pub const PORT_ENV: &str = "MP_FIRMWARE_PORT";

/// What the Upload button says for itself.
pub const NOT_ENABLED: &str = flow::NOT_ENABLED;

/// `FirmwareSelection`'s heading, shown when several files fit the board.
/// `// C#: test/FirmwareSelection.xaml:7`
const MORE_THAN_ONE: &str = "More than one choice exists. Please filter down to the desired \
                             selection.";

/// `FirmwareSelection`'s label over its Result picker.
/// `// C#: test/FirmwareSelection.xaml:27`
const PICK_A_FILE: &str = "Firmwares - Please pick a file to download and upload (.apj)";

/// `FirmwareSelection`'s button.
/// `// C#: test/FirmwareSelection.xaml:29`
const UPLOAD_FIRMWARE: &str = "Upload Firmware";

/// The caption of the box standing in for `OpenFileDialog`: its own default.
pub const OPEN_FILE: &str = "Open";

/// Load custom firmware's filter on this page.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:416`
pub const MANIFEST_FILTER: &str = "Firmware (*.hex;*.px4;*.vrx;*.apj)|*.hex;*.px4;*.vrx;*.apj|DFU|*.hex;*.bin;*.dfu|All files (*.*)|*.*";

/// The setting both pages keep Load custom firmware's folder in.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:410, 424`
pub const FIRMWARE_FILE_DIRECTORY: &str = "FirmwareFileDirectory";

/// `ConfigFirmwareDisabled.resx`'s `label1.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.resx:134-137`
const DISABLED_TEXT: &str = "You cannot load new firmware while connected via MAVLink. \n\n\
                             Please press the Disconnect button at top right to end the current \
                             MAVLink session and enable the firmware loading screen.";

/// `ConfigFirmwareDisabled.resx`'s `label2.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.resx:187-188`
const DISABLED_BOOTLOADER: &str = "Update the bootloader on your hardware to the newest available.";

/// `but_bootloaderupdate_Click`'s two questions, caption "BL Update".
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:26-30`
pub const BL_QUESTIONS: [&str; 2] = [
    "Are you sure you want to upgrade the bootloader? This can brick your board",
    "Are you sure you want to upgrade the bootloader? This can brick your board, Please allow 5 \
     mins for this process",
];

/// Their caption. `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:27`
const BL_UPDATE: &str = "BL Update";

/// The page's size, `ConfigFirmwareManifest.Designer.cs:296`.
const PAGE: (f32, f32) = (946.0, 375.0);

/// Every picture's size, 150 x 150.
const PICTURE: f32 = 150.0;

/// One of the vehicle pictures: an `ImageLabel` whose `Tag` is the vehicle it flashes.
#[derive(Debug, Clone, Copy)]
pub struct Picture {
    /// The Designer's control name: the citation, which the tests hold to the C#'s order.
    #[cfg_attr(not(test), allow(dead_code))]
    pub control: &'static str,
    /// The id a test clicks, and the last part of its label's fact.
    pub id: &'static str,
    /// What the picture shows, written in its place when its image is not carried.
    pub name: &'static str,
    /// Its `Image`, a `Properties.Resources` name ([`crate::pictures`]).
    pub image: &'static str,
    /// Its `Tag`.
    pub mav_type: MavType,
    /// `Location`.
    pub x: f32,
    /// `Location`.
    pub y: f32,
}

const fn picture(
    control: &'static str,
    id: &'static str,
    (name, image): (&'static str, &'static str),
    mav_type: MavType,
    x: f32,
    y: f32,
) -> Picture {
    Picture {
        control,
        id,
        name,
        image,
        mav_type,
        x,
        y,
    }
}

/// `imageLabel1`'s `Image`: a Cube.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.Designer.cs:118`
pub const CUBE_IMAGE: &str = "pixhawk2cube";

/// The pictures, in the order `Activate` labels them - which matters: it stops at the first
/// vehicle the release lacks (`First` throws), leaving that one and the rest unlabelled.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:32-42, 76-92; ConfigFirmwareManifest.Designer.cs:126-260 (*.Image: 128, 140, 153, 164, 175, 187, 200, 213, 226, 239, 252)`
pub const PICTURES: [Picture; 11] = [
    picture(
        "pictureAntennaTracker",
        "fw-tracker",
        ("Antenna Tracker", "Antenna_Tracker_01"),
        MavType::AntennaTracker,
        3.0,
        159.0,
    ),
    picture(
        "pictureBoxHeli",
        "fw-heli",
        ("Heli", "APM_airframes_08"),
        MavType::Helicopter,
        159.0,
        159.0,
    ),
    picture(
        "pictureBoxSub",
        "fw-sub",
        ("Sub", "sub"),
        MavType::Submarine,
        783.0,
        3.0,
    ),
    picture(
        "pictureBoxRover",
        "fw-rover",
        ("Rover", "rover_11"),
        MavType::GroundRover,
        3.0,
        3.0,
    ),
    picture(
        "pictureBoxOctaQuad",
        "fw-octaquad",
        ("Octa Quad", "x8"),
        MavType::Copter,
        627.0,
        3.0,
    ),
    picture(
        "pictureBoxOcta",
        "fw-octa",
        ("Octa", "FW_icons_2013_logos_12"),
        MavType::Copter,
        627.0,
        159.0,
    ),
    picture(
        "pictureBoxY6",
        "fw-y6",
        ("Y6", "y6a"),
        MavType::Copter,
        471.0,
        159.0,
    ),
    picture(
        "pictureBoxTri",
        "fw-tri",
        ("Tri", "FW_icons_2013_logos_08"),
        MavType::Copter,
        315.0,
        159.0,
    ),
    picture(
        "pictureBoxHexa",
        "fw-hexa",
        ("Hexa", "FW_icons_2013_logos_10"),
        MavType::Copter,
        471.0,
        3.0,
    ),
    picture(
        "pictureBoxQuad",
        "fw-quad",
        ("Quad", "FW_icons_2013_logos_04"),
        MavType::Copter,
        315.0,
        3.0,
    ),
    picture(
        "pictureBoxPlane",
        "fw-plane",
        ("Plane", "APM_airframes_001"),
        MavType::FixedWing,
        159.0,
        3.0,
    ),
];

/// A link along the bottom: its Designer name, the id a test clicks, its `Text`, `Location` and
/// `Size`, and its `Click` handler.
pub type Link = (
    &'static str,
    &'static str,
    &'static str,
    (f32, f32, f32, f32),
    &'static str,
);

/// The five links, left to right.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.Designer.cs:70-114, 262-270`
pub const LINKS: [Link; 5] = [
    (
        "lbl_alloptions",
        "fw-alloptions",
        "All Options",
        (459.0, 341.0, 57.0, 13.0),
        "lbl_alloptions_Click",
    ),
    (
        "lbl_bootloaderupdate",
        "fw-bootloaderupdate",
        "Bootloader Update",
        (522.0, 341.0, 96.0, 13.0),
        "Lbl_bootloaderupdate_Click",
    ),
    (
        "lbl_px4bl",
        "fw-px4bl",
        "Force Bootloader",
        (624.0, 341.0, 88.0, 13.0),
        "Lbl_px4bl_Click",
    ),
    (
        "lbl_Custom_firmware_label",
        "fw-custom",
        "Load custom firmware",
        (736.0, 341.0, 110.0, 13.0),
        "Lbl_Custom_firmware_label_Click",
    ),
    (
        "lbl_devfw",
        "fw-beta",
        "Beta firmwares",
        (867.0, 341.0, 76.0, 13.0),
        "Lbl_devfw_Click",
    ),
];

/// Why Force Bootloader and Bootloader Update do nothing here.
pub const BOOTLOADER_DISABLED: &str = "Force Bootloader reboots the board into its bootloader \
                                       (MAVLink doReboot) and Bootloader Update sends \
                                       MAV_CMD_FLASH_BOOTLOADER: both write to a board, and \
                                       flashing is not enabled in this build";

// ---------------------------------------------------------------------------------------------
// A flow on its thread, shared with the legacy page.
// ---------------------------------------------------------------------------------------------

/// What a flow's thread says to the page.
#[derive(Debug)]
enum Said {
    /// A question, waiting for its answer.
    Ask(Waiting),
    /// A message, waiting for its OK.
    Show(Waiting),
    /// `Progress(percent, status)`.
    Progress(i32, String),
    /// The flow has ended here.
    Done(Box<Reached>),
}

/// A question or message box a flow is waiting on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Waiting {
    /// The text.
    pub text: String,
    /// The caption.
    pub caption: String,
    /// The buttons of a question; `None` for a message with OK.
    pub buttons: Option<Buttons>,
}

/// The flow's end of the conversation: every `CustomMessageBox` blocks it until answered, as the
/// C#'s modal boxes block the handler that shows them.
struct Channel {
    said: Sender<Said>,
    answers: Receiver<bool>,
}

impl Dialogue for Channel {
    /// A page that has gone answers No.
    fn ask(&mut self, text: &str, caption: &str, buttons: Buttons) -> bool {
        let waiting = Waiting {
            text: text.to_owned(),
            caption: caption.to_owned(),
            buttons: Some(buttons),
        };
        self.said.send(Said::Ask(waiting)).is_ok() && self.answers.recv().unwrap_or(false)
    }

    fn show(&mut self, text: &str, caption: &str) {
        let waiting = Waiting {
            text: text.to_owned(),
            caption: caption.to_owned(),
            buttons: None,
        };
        if self.said.send(Said::Show(waiting)).is_ok() {
            let _ = self.answers.recv();
        }
    }

    fn progress(&mut self, percent: i32, status: &str) {
        let _ = self.said.send(Said::Progress(percent, status.to_owned()));
    }
}

/// The page's progress bar and status line: `fw_Progress1`, `-1` leaving the bar where it is.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:367-391; ConfigFirmware.cs:232-256`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Progress {
    /// `progress.Value`, 0 to 100.
    pub value: i32,
    /// `lbl_status.Text`.
    pub status: String,
}

impl Default for Progress {
    /// The Designer's: an empty bar, and "Status".
    fn default() -> Self {
        Self {
            value: 0,
            status: "Status".to_owned(),
        }
    }
}

impl Progress {
    /// `fw_Progress1(progress, status)`.
    pub fn set(&mut self, percent: i32, status: &str) {
        if percent != -1 {
            self.value = percent.clamp(0, 100);
        }
        status.clone_into(&mut self.status);
    }
}

/// This machine's serial ports and its configured comport, for a px4 upload.
///
/// The page shows only while nothing is connected (`InitialSetup.cs:169-173`), so the link the
/// reboot goes over is `MainV2.comPort` as configured - `comport` and its baud from the
/// settings - opened for the purpose and closed again, as `AttemptRebootToBootloader` does.
/// `// C#: Utilities/Firmware.cs:591-750, 797-837`
pub struct SerialHost {
    /// `MainV2.comPortName`.
    comport: String,
    /// `MainV2.comPort.BaseStream.BaudRate` as the settings hold it.
    baud: String,
}

impl SerialHost {
    /// `MainV2.comPortName`: a device path is a serial port; `UDP`, `TCP`, `AUTO` and the like
    /// are not.
    fn is_serial(&self) -> bool {
        self.comport.starts_with('/') || self.comport.starts_with("COM")
    }
}

/// How long `AttemptRebootToBootloader` gives the heartbeat and the reboot: `task.Wait(3 s)`.
/// `// C#: Utilities/Firmware.cs:823`
const REBOOT_WINDOW: std::time::Duration = std::time::Duration::from_secs(3);

/// How long `getHeartBeat` reads for a vehicle's heartbeat before it gives up: 2.2 s (or 200
/// packets read, which a board sending only heartbeats never reaches).
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1197`
const HEARTBEAT_WAIT: std::time::Duration = std::time::Duration::from_millis(2200);

/// How long the link thread may take to write the reboots before the port is closed under them:
/// not the C#'s, whose writes are on the calling thread and done when `doCommand` returns; ours
/// are queued for the link thread, and closing the port before it takes them would lose them.
/// A thread that takes this long is wedged.
const WRITE_WAIT: std::time::Duration = std::time::Duration::from_secs(1);

/// The waits of `AttemptRebootToBootloader`'s task: `task.Wait`'s window, and `getHeartBeat`'s
/// own limit. The tests shorten them.
#[derive(Debug, Clone, Copy)]
struct RebootWaits {
    /// `task.Wait(TimeSpan.FromSeconds(3))`.
    window: std::time::Duration,
    /// `getHeartBeat`'s 2.2 s.
    heartbeat: std::time::Duration,
}

/// Mission Planner's waits.
const REBOOT_WAITS: RebootWaits = RebootWaits {
    window: REBOOT_WINDOW,
    heartbeat: HEARTBEAT_WAIT,
};

/// The task `AttemptRebootToBootloader` runs over the opened link, and what `task.Wait` then
/// makes of it. The link is opened and closed by the caller.
///
/// `getHeartBeat` first: no vehicle heartbeat within its 2.2 s is "No HeartBeat found". Then
/// `doReboot(true, false)` - "scan for hb on unknown mav" - which reads for a heartbeat again,
/// the next one, up to another 2.2 s, and whether one comes or not (the first already set the
/// vehicle's ids) calls `doCommand` with `PREFLIGHT_REBOOT_SHUTDOWN` param1 3 and then with
/// param1 1, both unconditionally. `doCommand` writes a reboot twice back to back and returns
/// without waiting for an answer, so four frames go out, 3, 3, 1, 1, with no gap between any of
/// them: through [`mp_link::Link::command`], acknowledgement required as `doCommand` is called,
/// which the link sends as `Outgoing::Twice` and ends `Sent`. `MainV2.comPort.Close()` follows
/// at once.
///
/// All of it inside `task.Wait`'s 3 s is "Reboot to Bootloader"; a task still running then is
/// "Please unplug the board" - the task goes on, and its reboots still go out, as here.
/// `// C#: Utilities/Firmware.cs:797-837; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1153-1203, 2553-2559, 2591-2618, 2717, 2758-2763`
fn reboot_to_bootloader(
    link: &mp_link::Link,
    started: std::time::Instant,
    waits: RebootWaits,
) -> LinkReboot {
    use std::time::{Duration, Instant};
    let poll = Duration::from_millis(10);
    // `MainV2.comPort.getHeartBeat().Length > 0`, else "No HeartBeat found".
    let first = started + waits.heartbeat;
    let (id, handle) = loop {
        if let Some(vehicle) = link.primary_vehicle() {
            break vehicle;
        }
        if Instant::now() >= first {
            return LinkReboot::NoHeartbeat;
        }
        std::thread::sleep(poll);
    };
    // `doReboot(true, false)`: `getHeartBeat` again, the heartbeat after the one seen.
    let seen = handle.load().heartbeats;
    let again = Instant::now() + waits.heartbeat;
    while handle.load().heartbeats <= seen && Instant::now() < again {
        std::thread::sleep(poll);
    }
    // `if (MAV.sysid != 0 && MAV.compid != 0)`: 3 twice, then 1 twice.
    if id.sysid != 0 && id.compid != 0 {
        let mut last = None;
        for message in [
            mp_link::commands::reboot_to_bootloader(id),
            mp_link::commands::reboot(id),
        ] {
            if let Some((target, command, params)) = crate::telemetry::command_long_parts(&message)
            {
                last = Some(link.command(target, command, params, true));
            }
        }
        // Written before the port closes: the link ends a reboot `Sent` in the pass that
        // writes it.
        let written = Instant::now() + WRITE_WAIT;
        while let Some(request) = last
            && link.is_running()
            && Instant::now() < written
            && link.request(request).is_none_or(|r| r.outcome().is_none())
        {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    if started.elapsed() <= waits.window {
        LinkReboot::Rebooted
    } else {
        LinkReboot::NoHeartbeat
    }
}

impl FlashHost for SerialHost {
    type Port = TransportPort<mp_transport::SerialTransport>;

    fn port_names(&mut self) -> Vec<String> {
        mp_transport::list_ports()
            .into_iter()
            .map(|port| port.name)
            .collect()
    }
    fn open(&mut self, port: &str, baud: u32) -> std::io::Result<Self::Port> {
        open_serial(port, baud, false)
    }
    /// `if (MainV2.comPort.BaseStream is SerialPort)`: the comport opened as a MAVLink link,
    /// [`reboot_to_bootloader`]'s heartbeats and four reboots, and the port closed; a port that
    /// will not open is the task's exception, "Please unplug the board".
    /// `// C#: Utilities/Firmware.cs:797-837`
    fn reboot_link(&mut self) -> LinkReboot {
        if !self.is_serial() {
            return LinkReboot::NotSerial;
        }
        let started = std::time::Instant::now();
        let url = format!("serial:{}:{}", self.comport, self.baud);
        let Ok(link) = mp_link::Link::connect(&url, mp_link::LinkConfig::default()) else {
            return LinkReboot::NoHeartbeat;
        };
        let reached = reboot_to_bootloader(&link, started, REBOOT_WAITS);
        // `MainV2.comPort.Close()`.
        drop(link);
        reached
    }
    fn now(&mut self) -> std::time::Instant {
        std::time::Instant::now()
    }
    fn sleep(&mut self, duration: std::time::Duration) {
        std::thread::sleep(duration);
    }
}

/// A flow running on its thread: what it has said, and the box it is waiting on.
#[derive(Debug)]
pub struct Worker {
    said: Receiver<Said>,
    answers: Sender<bool>,
    /// The question or message showing, until it is answered.
    pub waiting: Option<Waiting>,
}

impl Worker {
    /// Starts a flow on a thread of its own. `None` when no thread could be made.
    pub fn start(
        name: &str,
        work: impl FnOnce(&mut dyn Dialogue) -> Reached + Send + 'static,
    ) -> Option<Self> {
        let (said, heard) = channel();
        let (answer, answers) = channel();
        let done = said.clone();
        std::thread::Builder::new()
            .name(name.to_owned())
            .spawn(move || {
                let mut channel = Channel { said, answers };
                let reached = work(&mut channel);
                let _ = done.send(Said::Done(Box::new(reached)));
            })
            .ok()?;
        Some(Self {
            said: heard,
            answers: answer,
            waiting: None,
        })
    }

    /// What the flow has said since: progress into `progress`, a box to answer into
    /// [`Worker::waiting`], and - once it has ended - where it got to.
    pub fn poll(&mut self, progress: &mut Progress) -> Option<Reached> {
        while self.waiting.is_none() {
            match self.said.try_recv() {
                Ok(Said::Progress(percent, status)) => progress.set(percent, &status),
                Ok(Said::Ask(waiting) | Said::Show(waiting)) => self.waiting = Some(waiting),
                Ok(Said::Done(reached)) => return Some(*reached),
                Err(TryRecvError::Empty) => return None,
                Err(TryRecvError::Disconnected) => return Some(Reached::default()),
            }
        }
        None
    }

    /// The box answered: Yes or OK is true.
    pub fn answer(&mut self, yes: bool) {
        if self.waiting.take().is_some() {
            let _ = self.answers.send(yes);
        }
    }
}

/// What the flows need from the machine: the network, where to save, and the board's ports.
pub struct Machine {
    /// The network, or the fixtures standing in for it.
    pub fetch: Box<dyn manifest::Fetch + Send + Sync>,
    /// `Settings.GetUserDataDirectory()`.
    pub user_data: PathBuf,
    /// `Path.GetTempPath()`.
    pub temp_dir: PathBuf,
    /// `MainV2.comPortName`, as the settings last saved it.
    pub comport: String,
    /// Its baud rate, as the settings hold it.
    pub baud: String,
    /// Whether a px4 upload may go to a board: not while [`DEVICE_ENV`] stands in for the
    /// machine, so no test reaches whatever is plugged in.
    pub flash: bool,
    /// The USB devices.
    pub devices: Vec<DeviceInfo>,
    /// WMI's `Win32_SerialPort` rows, as the ports' USB ids make them.
    pub rows: Vec<Win32SerialPort>,
}

impl Machine {
    /// This machine: the network (or [`manifest::fetcher`]'s fixtures), the data directory, the
    /// temporary directory, the saved port and the ports as enumerated - or [`DEVICE_ENV`]'s one
    /// device, with no ports.
    pub fn here(settings: &Persisted) -> Self {
        let rows = if std::env::var_os(DEVICE_ENV).is_some() {
            Vec::new()
        } else {
            mp_transport::list_ports()
                .iter()
                .map(Win32SerialPort::from_port)
                .collect()
        };
        Self {
            fetch: manifest::fetcher(),
            user_data: mp_settings::user_data_directory()
                .unwrap_or_else(|| std::env::temp_dir().join("MissionPlannerRust")),
            temp_dir: std::env::temp_dir(),
            comport: std::env::var(PORT_ENV)
                .unwrap_or_else(|_| settings.get("comport").unwrap_or_default().to_owned()),
            baud: settings.baud().to_owned(),
            flash: std::env::var_os(DEVICE_ENV).is_none(),
            devices: devices(),
            rows,
        }
    }

    /// The board host for a px4 upload: this machine's ports and its configured comport.
    #[must_use]
    pub fn host(&self) -> SerialHost {
        SerialHost {
            comport: self.comport.clone(),
            baud: self.baud.clone(),
        }
    }

    /// Runs a flow over this machine.
    pub fn run(
        &self,
        dialogue: &mut dyn Dialogue,
        work: impl FnOnce(&mut flow::Cx<'_>, &Self) -> Reached,
    ) -> Reached {
        let mut cx = flow::Cx {
            dialogue,
            fetch: self.fetch.as_ref(),
            user_data: self.user_data.clone(),
            temp_dir: self.temp_dir.clone(),
        };
        work(&mut cx, self)
    }
}

/// Load custom firmware's `OpenFileDialog`, which this application has no platform dialog for:
/// the path is typed, into a box that starts in the dialog's `InitialDirectory` - the folder last
/// used, when it exists - with the dialog's filter under it. OK takes a file that exists; any other
/// path, or Cancel, does nothing, as the C#'s `if (File.Exists(fd.FileName))` does.
#[derive(Debug)]
pub struct PathBox {
    /// The path being typed.
    pub field: TextField,
    /// The dialog's filter, shown under the box.
    pub filter: &'static str,
    /// The dialog's caption: [`OPEN_FILE`], or "Save As" for a `SaveFileDialog`.
    pub caption: &'static str,
}

impl PathBox {
    /// The dialog opening in `custom_fw_dir`.
    #[must_use]
    pub fn new(directory: &str, filter: &'static str) -> Self {
        let mut field = TextField::new("");
        if !directory.is_empty() && Path::new(directory).is_dir() {
            let separator = std::path::MAIN_SEPARATOR;
            field.set(format!(
                "{}{separator}",
                directory.trim_end_matches(separator)
            ));
        }
        Self {
            field,
            filter,
            caption: OPEN_FILE,
        }
    }

    /// The file chosen, if the path names one that exists.
    #[must_use]
    pub fn chosen(&self) -> Option<PathBuf> {
        let path = PathBuf::from(self.field.value());
        path.is_file().then_some(path)
    }
}

/// `custom_fw_dir = Path.GetDirectoryName(fd.FileName)`, saved in the settings.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:423-424; ConfigFirmware.cs:531-532`
pub fn remember_folder(settings: &mut Persisted, file: &Path) -> String {
    let folder = file
        .parent()
        .map(|dir| dir.display().to_string())
        .unwrap_or_default();
    settings.set(FIRMWARE_FILE_DIRECTORY, folder.clone());
    folder
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// What the background fetch sends back: the manifest, if one was had, and what went wrong.
type Fetched = (Option<Manifest>, Vec<String>);

/// `FirmwareSelection`, open over a lookup's records: its Result picker's selection.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Choosing {
    /// The records and the device.
    pub lookup: Lookup,
    /// The Result row selected.
    pub selected: Option<usize>,
}

impl Choosing {
    /// The dialog as it opens: the one row selected when there is one - a file, or the line
    /// saying there are too many or none.
    /// `// C#: test/FirmwareSelection.xaml.cs:107-126`
    fn open(lookup: Lookup) -> Self {
        let rows = Selection::new(&lookup.items, &lookup.device)
            .results()
            .len();
        Self {
            lookup,
            selected: (rows == 1).then_some(0),
        }
    }

    /// The Result picker's rows.
    #[must_use]
    pub fn results(&self) -> Vec<String> {
        Selection::new(&self.lookup.items, &self.lookup.device).results()
    }

    /// The Platform picker's choice, as the dialog opened it.
    #[must_use]
    pub fn platform(&self) -> Option<String> {
        Selection::new(&self.lookup.items, &self.lookup.device).platform
    }
}

/// The page, and the catalogue it keeps.
#[derive(Debug)]
pub struct InstallFirmware {
    /// Between `Activate` and `Deactivate`.
    open: bool,
    /// Showing `ConfigFirmwareDisabled`, because a vehicle was connected when it opened.
    connected: bool,
    /// `REL_Type`.
    release: ReleaseType,
    /// `APFirmware.Manifest`: a static in the C#, so it outlives the page, fetched once.
    manifest: Option<Arc<Manifest>>,
    /// The fetch under way.
    receiver: Option<Receiver<Fetched>>,
    /// How many fetches have been started, for a test of the once-per-process rule.
    fetches: usize,
    /// What the last fetch logged.
    errors: Vec<String>,
    /// Each picture's `Text`, in [`PICTURES`] order.
    labels: [String; 11],
    /// Each picture's `Enabled`.
    enabled: [bool; 11],
    /// `lbl_alloptions.Enabled` and `lbl_devfw.Enabled`, which go together.
    links_enabled: bool,
    /// The USB devices, as last enumerated.
    devices: Vec<DeviceInfo>,
    /// The device `LookForPort` would take, and its board ids.
    detected: Option<(DeviceInfo, Vec<i64>)>,
    /// The last picture clicked, and what `LookForPort` made of it: `None` is no port.
    picked: Option<(usize, Option<Lookup>)>,
    /// "Are you sure you want to upload ...?", for this picture.
    confirm: Option<usize>,
    /// `FirmwareSelection`, open.
    selection: Option<Choosing>,
    /// A download or a custom firmware on its way to the board.
    worker: Option<Worker>,
    /// `progress` and `lbl_status`.
    progress: Progress,
    /// Where the last flow got.
    reached: Option<Reached>,
    /// The page's own message boxes - `LookForPort`'s "Failed to detect port" - the first showing.
    messages: VecDeque<Waiting>,
    /// Load custom firmware's dialog.
    path: Option<PathBox>,
    /// `ConfigFirmwareDisabled`'s Bootloader Update: which of its two questions is showing.
    bootloader: Option<usize>,
    /// The second Yes given: `FLASH_BOOTLOADER` is owed to the vehicle.
    bootloader_command: bool,
}

impl Default for InstallFirmware {
    fn default() -> Self {
        Self {
            open: false,
            connected: false,
            release: ReleaseType::Official,
            manifest: None,
            receiver: None,
            fetches: 0,
            errors: Vec::new(),
            labels: Default::default(),
            enabled: [false; 11],
            links_enabled: false,
            devices: Vec::new(),
            detected: None,
            picked: None,
            confirm: None,
            selection: None,
            worker: None,
            progress: Progress::default(),
            reached: None,
            messages: VecDeque::new(),
            path: None,
            bootloader: None,
            bootloader_command: false,
        }
    }
}

impl InstallFirmware {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open
    }

    /// Opens the page if it is closed and closes it if it is open.
    pub fn toggle(&mut self, view: &TelemetryView) {
        if self.open {
            self.close();
        } else {
            self.open(view.connected);
        }
    }

    /// Opens the page: `ConfigFirmwareDisabled` with a link open, else `ConfigFirmwareManifest`'s
    /// `Activate`.
    pub fn open(&mut self, connected: bool) {
        self.open = true;
        self.connected = connected;
        if !connected {
            self.activate();
        }
    }

    /// `Activate`: disables the pictures and links, then labels them from the catalogue - at
    /// once when it is held, else when the fetch this starts has brought it.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:45-94`
    pub fn activate(&mut self) {
        self.enabled = [false; 11];
        self.links_enabled = false;
        self.picked = None;
        self.devices = devices();
        if self.manifest.is_some() {
            self.label();
        } else {
            self.fetch();
        }
    }

    /// `Deactivate`: back to the official release for next time. The page's boxes close with
    /// it, and a flow still running answers No to whatever it asks next.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:124-132`
    pub fn close(&mut self) {
        self.open = false;
        self.release = ReleaseType::Official;
        self.picked = None;
        self.confirm = None;
        self.selection = None;
        self.worker = None;
        self.messages.clear();
        self.path = None;
        self.bootloader = None;
    }

    /// Once a frame: takes a finished fetch, hears the flow running, and closes the page when
    /// the screen changes, as leaving Initial Setup deactivates its page.
    pub fn tick(&mut self, on_setup: bool) {
        if let Some(receiver) = &self.receiver {
            match receiver.try_recv() {
                Ok((manifest, errors)) => {
                    self.receiver = None;
                    self.errors = errors;
                    if let Some(manifest) = manifest {
                        self.manifest = Some(Arc::new(manifest));
                    }
                    if self.open && !self.connected {
                        self.label();
                    }
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => self.receiver = None,
            }
        }
        if let Some(worker) = &mut self.worker
            && let Some(reached) = worker.poll(&mut self.progress)
        {
            self.worker = None;
            self.reached = Some(reached);
        }
        if self.open && !on_setup {
            self.close();
        }
    }

    /// The fetch `Activate` starts: `GetList` over the mirror and then ardupilot.org, on a
    /// thread, as the C# runs it in `Task.Run`. One at a time.
    fn fetch(&mut self) {
        self.fetch_from(manifest::fetcher());
    }

    /// [`InstallFirmware::fetch`] from a given source: the network, or a file.
    fn fetch_from(&mut self, fetch: Box<dyn manifest::Fetch + Send + Sync>) {
        if self.receiver.is_some() {
            return;
        }
        let (sender, receiver) = channel();
        let started = std::thread::Builder::new()
            .name("mp-firmware-manifest".to_owned())
            .spawn(move || {
                let mut held = None;
                let errors = manifest::load(&mut held, fetch.as_ref());
                let _ = sender.send((held, errors.iter().map(ToString::to_string).collect()));
            });
        match started {
            Ok(_) => {
                self.receiver = Some(receiver);
                self.fetches += 1;
            }
            Err(err) => self.errors = vec![err.to_string()],
        }
    }

    /// The rest of `Activate`'s task: each picture's label from the newest firmware of its
    /// vehicle, enabling it and the links; and the board the devices name.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:69-92, 96-122`
    fn label(&mut self) {
        let Some(manifest) = self.manifest.clone() else {
            // `GetRelease` on a null manifest throws: nothing is labelled or enabled.
            return;
        };
        let newest = manifest.release_newest(self.release);
        for (index, picture) in PICTURES.iter().enumerate() {
            let Some(first) = newest
                .iter()
                .find(|a| a.mav_type.as_deref() == Some(picture.mav_type.name()))
            else {
                break;
            };
            if let Some(label) = self.labels.get_mut(index) {
                *label = icon_name(first);
            }
            if let Some(enabled) = self.enabled.get_mut(index) {
                *enabled = true;
            }
            self.links_enabled = true;
        }
        self.detected = self
            .devices
            .iter()
            .find_map(|device| Some((device.clone(), manifest.board_ids(device, true)?)));
    }

    /// Whether the page takes a click: open on the manifest page, with no box over it.
    fn live(&self) -> bool {
        self.open
            && !self.connected
            && self.confirm.is_none()
            && self.selection.is_none()
            && self.path.is_none()
            && self.messages.is_empty()
            // The C#'s handlers hold the UI thread until the flow has ended.
            && self.worker.is_none()
    }

    /// `PictureBox_Click`: "Are you sure you want to upload <label>?", caption `Continue`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:182-191`
    pub fn click(&mut self, index: usize) {
        if !self.live() || !self.enabled.get(index).copied().unwrap_or(false) {
            return;
        }
        self.confirm = Some(index);
    }

    /// The question's text, for the picture it asks about.
    fn confirm_text(&self, index: usize) -> String {
        format!(
            "{}{}{}",
            flow::ARE_YOU_SURE_YOU_WANT_TO_UPLOAD,
            self.labels.get(index).map_or("", String::as_str),
            flow::QUESTION_MARK
        )
    }

    /// The question answered: Yes runs `LookForPort` for the picture's vehicle.
    pub fn answer_confirm(&mut self, yes: bool, settings: &Persisted) {
        let Some(index) = self.confirm.take() else {
            return;
        };
        if yes {
            self.pick(index, settings);
        }
    }

    /// `LookForPort(mavtype)` over the devices as they are now: no device with a board id says
    /// "Failed to detect port"; one file is downloaded; several open `FirmwareSelection`; none
    /// says "No firmware available" and then fails to download.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:193-358`
    pub fn pick(&mut self, index: usize, settings: &Persisted) {
        let (Some(manifest), Some(picture)) = (self.manifest.clone(), PICTURES.get(index)) else {
            return;
        };
        self.devices = devices();
        let lookup =
            manifest.look_for_port(&self.devices, None, picture.mav_type, self.release, false);
        self.picked = Some((index, lookup.clone()));
        self.after_lookup(lookup, settings);
    }

    /// `lbl_alloptions_Click`: `LookForPort(Copter, true)` - the first device, or a blank one,
    /// and every record of the catalogue in `FirmwareSelection`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:564-567, 210-211, 238-239`
    pub fn all_options(&mut self, settings: &Persisted) {
        if !self.live() || !self.links_enabled {
            return;
        }
        let Some(manifest) = self.manifest.clone() else {
            return;
        };
        self.devices = devices();
        let lookup =
            manifest.look_for_port(&self.devices, None, MavType::Copter, self.release, true);
        self.picked = None;
        self.after_lookup(lookup, settings);
    }

    /// What `LookForPort` does with what it found.
    fn after_lookup(&mut self, lookup: Option<Lookup>, settings: &Persisted) {
        self.reached = None;
        let Some(lookup) = lookup else {
            self.messages.push_back(Waiting {
                text: NO_PORT.to_owned(),
                caption: flow::ERROR.to_owned(),
                buttons: None,
            });
            return;
        };
        let device = lookup.device.name.clone().unwrap_or_default();
        match lookup.outcome() {
            Outcome::One(record) => {
                let url = record.url.clone().unwrap_or_default();
                self.download(url, device, false, settings);
            }
            Outcome::Choose(_) => self.selection = Some(Choosing::open(lookup)),
            Outcome::NoFirmware => self.download(String::new(), device, true, settings),
        }
    }

    /// A Result row chosen in `FirmwareSelection`.
    pub fn select(&mut self, row: usize) {
        if let Some(choosing) = &mut self.selection
            && row < choosing.results().len()
        {
            choosing.selected = Some(row);
        }
    }

    /// "Upload Firmware": the row selected is `FinalResult`, and the download goes on; with no
    /// row selected the button does nothing.
    /// `// C#: test/FirmwareSelection.xaml.cs:200-212; ConfigFirmwareManifest.cs:247-254`
    pub fn upload_selected(&mut self, settings: &Persisted) {
        let Some(choosing) = &self.selection else {
            return;
        };
        let Some(url) = choosing
            .selected
            .and_then(|row| choosing.results().get(row).cloned())
        else {
            return;
        };
        let device = choosing.lookup.device.name.clone().unwrap_or_default();
        self.selection = None;
        self.download(url, device, false, settings);
    }

    /// `FirmwareSelection` closed without a choice: `FinalResult` is null, "user canceled".
    pub fn close_selection(&mut self) {
        self.selection = None;
    }

    /// The download and `UploadFlash`, on their thread; after "No firmware available" when
    /// `nothing` is set.
    fn download(&mut self, url: String, device: String, nothing: bool, settings: &Persisted) {
        let machine = Machine::here(settings);
        self.worker = Worker::start("mp-firmware-download", move |dialogue| {
            machine.run(dialogue, |cx, machine| {
                if nothing {
                    cx.dialogue.show(NO_FIRMWARE, flow::ERROR);
                }
                let reached = flow::download_and_flash(cx, &url, &device);
                if machine.flash {
                    let mut host = machine.host();
                    flow::flash_if_stopped(cx, &mut host, reached)
                } else {
                    reached
                }
            })
        });
    }

    /// Beta firmwares: the beta release, and `Activate` again.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:393-397`
    pub fn beta(&mut self) {
        if !self.links_enabled || !self.live() {
            return;
        }
        self.release = ReleaseType::Beta;
        self.activate();
    }

    /// Load custom firmware: its file dialog, in the folder last used.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:413-420`
    pub fn custom(&mut self, settings: &Persisted) {
        if !self.live() {
            return;
        }
        let folder = settings.get(FIRMWARE_FILE_DIRECTORY).unwrap_or_default();
        self.path = Some(PathBox::new(folder, MANIFEST_FILTER));
    }

    /// A key for the file dialog: Enter is OK, Escape Cancel.
    pub fn path_key(&mut self, event: &KeyDownEvent, settings: &mut Persisted) -> bool {
        let Some(path) = &mut self.path else {
            return false;
        };
        match path.field.key(event) {
            KeyOutcome::Changed => true,
            KeyOutcome::Submitted => {
                self.path_done(true, settings);
                true
            }
            KeyOutcome::Cancelled => {
                self.path_done(false, settings);
                true
            }
            KeyOutcome::Ignored => false,
        }
    }

    /// The file dialog closed: OK on a file that exists remembers its folder and hands it to
    /// `custom_manifest` on a thread.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:421-510`
    pub fn path_done(&mut self, ok: bool, settings: &mut Persisted) {
        let Some(path) = self.path.take() else {
            return;
        };
        let Some(file) = path.chosen().filter(|_| ok) else {
            return;
        };
        remember_folder(settings, &file);
        self.reached = None;
        let machine = Machine::here(settings);
        self.worker = Worker::start("mp-firmware-custom", move |dialogue| {
            machine.run(dialogue, |cx, machine| {
                let reached = flow::custom_manifest(
                    cx,
                    &machine.comport,
                    &file,
                    &machine.devices,
                    &machine.rows,
                );
                if machine.flash {
                    let mut host = machine.host();
                    flow::flash_if_stopped(cx, &mut host, reached)
                } else {
                    reached
                }
            })
        });
    }

    /// `ConfigFirmwareDisabled`'s Bootloader Update, with the link as it is now: nothing
    /// unless it is open, else the first of its two questions.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:18-27`
    pub fn bootloader_update(&mut self, link_open: bool) {
        if self.open && self.connected && link_open && self.bootloader.is_none() {
            self.bootloader = Some(0);
        }
    }

    /// A Bootloader Update question answered: Yes to the first asks the second; Yes to the second
    /// is `doCommand(FLASH_BOOTLOADER, 0, 0, 0, 0, 290876, 0, 0)`, which rewrites the board's
    /// bootloader from the one its firmware carries - sent by the window, which owns the link,
    /// once it sees [`InstallFirmware::take_bootloader_command`].
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:26-44`
    pub fn answer_bootloader(&mut self, yes: bool) {
        match (self.bootloader.take(), yes) {
            (Some(0), true) => self.bootloader = Some(1),
            (Some(_), true) => self.bootloader_command = true,
            _ => {}
        }
    }

    /// Whether the second Yes has been given and the command is owed, once.
    pub fn take_bootloader_command(&mut self) -> bool {
        std::mem::take(&mut self.bootloader_command)
    }

    /// The flow's box answered.
    pub fn answer_worker(&mut self, yes: bool) {
        if let Some(worker) = &mut self.worker {
            worker.answer(yes);
        }
    }

    /// The page's own message box dismissed.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// The device shown as the board: the one a click would use, or the one `LookForPort`
    /// would take.
    fn board(&self) -> Option<(&DeviceInfo, &[i64])> {
        match &self.picked {
            Some((_, Some(lookup))) => Some((&lookup.device, lookup.board_ids.as_slice())),
            Some((_, None)) => None,
            None => self
                .detected
                .as_ref()
                .map(|(device, ids)| (device, ids.as_slice())),
        }
    }

    /// The lookup the last click made.
    fn lookup(&self) -> Option<&Lookup> {
        self.picked.as_ref().and_then(|(_, lookup)| lookup.as_ref())
    }

    /// The message box showing: the flow's, or the page's own.
    fn message(&self) -> Option<&Waiting> {
        self.worker
            .as_ref()
            .and_then(|worker| worker.waiting.as_ref())
            .filter(|waiting| waiting.buttons.is_none())
            .or_else(|| self.messages.front())
    }

    /// The question showing: the page's own confirmation or Bootloader Update's, or the flow's.
    fn question(&self) -> Option<String> {
        self.confirm
            .map(|index| self.confirm_text(index))
            .or_else(|| {
                self.bootloader
                    .and_then(|at| BL_QUESTIONS.get(at))
                    .map(|text| (*text).to_owned())
            })
            .or_else(|| {
                self.worker
                    .as_ref()
                    .and_then(|worker| worker.waiting.as_ref())
                    .filter(|waiting| waiting.buttons.is_some())
                    .map(|waiting| waiting.text.clone())
            })
    }

    /// Where the catalogue stands.
    fn manifest_state(&self) -> &'static str {
        if self.manifest.is_some() {
            "loaded"
        } else if self.receiver.is_some() {
            "fetching"
        } else if self.errors.is_empty() {
            "none"
        } else {
            "failed"
        }
    }

    /// Whether the path dialog is open, for its focus.
    #[must_use]
    pub const fn path_open(&self) -> bool {
        self.path.is_some()
    }
}

/// The USB devices, as `LookForPort` lists them: the ports, each through
/// `DeviceInfo::from_port`; or the one [`DEVICE_ENV`] names.
pub fn devices() -> Vec<DeviceInfo> {
    if let Some(spec) = std::env::var_os(DEVICE_ENV) {
        let spec = spec.to_string_lossy();
        let (board, hardwareid) = spec.split_once('|').unwrap_or((&spec, ""));
        return vec![DeviceInfo::new(board, hardwareid)];
    }
    mp_transport::list_ports()
        .iter()
        .filter_map(DeviceInfo::from_port)
        .collect()
}

/// A flow's outcome as facts, under `prefix`.
pub fn record_reached(prefix: &str, reached: Option<&Reached>, running: bool) {
    use crate::facts::record;
    record(
        format!("{prefix}.flow"),
        if running {
            "running"
        } else if reached.is_some() {
            "done"
        } else {
            "idle"
        },
    );
    let text = |value: Option<String>| value.unwrap_or_else(|| "none".to_owned());
    record(
        format!("{prefix}.flow.board"),
        text(reached.and_then(|r| r.board.clone())),
    );
    record(
        format!("{prefix}.download.url"),
        text(reached.and_then(|r| r.url.clone())),
    );
    record(
        format!("{prefix}.download.file"),
        text(reached.and_then(|r| {
            r.file
                .as_ref()
                .and_then(|file| file.file_name())
                .map(|name| name.to_string_lossy().into_owned())
        })),
    );
    record(
        format!("{prefix}.download.size"),
        text(reached.and_then(|r| r.size).map(|size| size.to_string())),
    );
    record(
        format!("{prefix}.apj.board_id"),
        text(
            reached
                .and_then(|r| r.firmware.as_ref())
                .map(|f| f.board_id.to_string()),
        ),
    );
    record(
        format!("{prefix}.apj.image"),
        text(
            reached
                .and_then(|r| r.firmware.as_ref())
                .map(|f| f.image_len.to_string()),
        ),
    );
    record(
        format!("{prefix}.stopped"),
        text(reached.and_then(|r| r.stop.as_ref()).map(flow::Stop::text)),
    );
}

/// Facts for a test: which page, the catalogue, the release, the labels, the board, what a
/// click chose, the boxes showing, and where the flow got.
pub fn record_facts(page: &InstallFirmware, settings: &Persisted) {
    use crate::facts::record;
    record("config.firmware.active", page.open);
    record(
        "config.firmware.page",
        match (page.open, page.connected) {
            (false, _) => "closed",
            (true, true) => "disabled",
            (true, false) => "manifest",
        },
    );
    record("config.firmware.manifest", page.manifest_state());
    record(
        "config.firmware.manifest.records",
        page.manifest.as_ref().map_or(0, |m| m.firmware.len()),
    );
    record("config.firmware.manifest.fetches", page.fetches);
    record("config.firmware.manifest.errors", page.errors.join("; "));
    record("config.firmware.release", page.release.name());
    record("config.firmware.enabled", page.links_enabled);
    for (picture, label) in PICTURES.iter().zip(&page.labels) {
        let key = picture.id.trim_start_matches("fw-");
        record(format!("config.firmware.label.{key}"), label);
    }
    let (board, ids) = page.board().map_or_else(
        || ("none".to_owned(), "none".to_owned()),
        |(device, ids)| {
            (
                device.board.clone().unwrap_or_default(),
                ids.iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(","),
            )
        },
    );
    record("config.firmware.board", board);
    record("config.firmware.board.ids", ids);
    record(
        "config.firmware.flashed",
        page.reached
            .as_ref()
            .and_then(|reached| reached.flashed)
            .map_or_else(|| "none".to_owned(), |board| board.board_id.to_string()),
    );
    record(
        "config.firmware.vehicle",
        page.picked
            .as_ref()
            .and_then(|(index, _)| PICTURES.get(*index))
            .map_or("none", |picture| picture.mav_type.name()),
    );
    record(
        "config.firmware.candidates",
        page.lookup().map_or(0, |lookup| lookup.items.len()),
    );
    let chosen = page.lookup().and_then(Lookup::chosen);
    record(
        "config.firmware.chosen",
        chosen.and_then(|a| a.url.as_deref()).unwrap_or("none"),
    );
    record(
        "config.firmware.chosen.version",
        chosen
            .and_then(|a| a.mav_firmware_version)
            .map_or_else(|| "none".to_owned(), |v| v.to_string()),
    );
    record(
        "config.firmware.question",
        page.question().unwrap_or_else(|| "none".to_owned()),
    );
    record(
        "config.firmware.message",
        page.message().map_or_else(
            || "none".to_owned(),
            |waiting| waiting.text.replace("\r\n", " ").replace('\n', " "),
        ),
    );
    record(
        "config.firmware.selection",
        page.selection
            .as_ref()
            .map_or(0, |choosing| choosing.results().len()),
    );
    record(
        "config.firmware.selection.selected",
        page.selection
            .as_ref()
            .and_then(|choosing| {
                choosing
                    .selected
                    .and_then(|row| choosing.results().get(row).cloned())
            })
            .unwrap_or_else(|| "none".to_owned()),
    );
    record("config.firmware.progress", page.progress.value);
    record("config.firmware.status", &page.progress.status);
    record(
        "config.firmware.path",
        page.path
            .as_ref()
            .map_or_else(|| "closed".to_owned(), |path| path.field.value().to_owned()),
    );
    record(
        "config.firmware.custom.dir",
        settings.get(FIRMWARE_FILE_DIRECTORY).unwrap_or("none"),
    );
    record_reached(
        "config.firmware",
        page.reached.as_ref(),
        page.worker.is_some(),
    );
    record("config.firmware.upload", "disabled");
}

/// A box at a Designer `Location` and `Size`.
fn at(x: f32, y: f32, width: f32, height: f32) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A `ProgressBar` at its place, filled to its value.
pub fn progress_bar((x, y, width, height): (f32, f32, f32, f32), value: i32) -> Div {
    #[allow(clippy::cast_precision_loss)] // 0 to 100
    let fraction = value.clamp(0, 100) as f32 / 100.0;
    at(x, y, width, height)
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .child(
            div()
                .absolute()
                .left_0()
                .top_0()
                .h_full()
                .w(gpui::relative(fraction))
                .bg(rgb(theme::OK)),
        )
}

/// A label that is a link: at its `.resx` position, sized by its text; live, it is the accent
/// colour and takes a click, and dimmed it does neither.
pub fn link_label(
    id: &'static str,
    text: &'static str,
    (x, y): (f32, f32),
    live: bool,
    on_click: impl Fn(&mut MissionPlanner, &mut Window, &mut Context<MissionPlanner>) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    // Sized by its text rather than the label's box: the text does not fit that box at this
    // font, wrapped below it, and a click on the text then landed outside the hitbox -
    // tests/gui/config-firmware.gui found the link dead.
    let label = crate::probe::measured(id, div().absolute().left(px(x)).top(px(y)))
        .id(id)
        .whitespace_nowrap()
        .text_xs()
        .child(text);
    if live {
        label
            .text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.underline())
            .on_click(cx.listener(move |this, _event, window, cx| {
                on_click(this, window, cx);
                cx.notify();
            }))
            .into_any_element()
    } else {
        label.text_color(rgb(theme::DIM)).into_any_element()
    }
}

/// The page, while it is open.
pub fn page(firmware: &InstallFirmware, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if firmware.connected {
        return disabled_page(firmware, cx).into_any_element();
    }
    let mut body = div().relative().w(px(PAGE.0)).h(px(PAGE.1));
    for (index, picture) in PICTURES.iter().enumerate() {
        body = body.child(image_label(firmware, index, picture, cx));
    }
    // `imageLabel1`: a picture of a Cube, with no Click handler, and an empty label under it.
    // C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.Designer.cs:116-124
    body = body.child(
        at(783.0, 159.0, PICTURE, PICTURE)
            .flex()
            .flex_col()
            .rounded_md()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .child(crate::pictures::image_label(
                "firmware",
                "imageLabel1",
                CUBE_IMAGE,
                "Pixhawk 2 Cube",
                theme::DIM,
            ))
            .child(div().h(px(13.0)).mb_1()),
    );
    // `progress` and `lbl_status`, which `fw_Progress1` moves.
    body = body.child(progress_bar(
        (3.0, 315.0, 940.0, 23.0),
        firmware.progress.value,
    ));
    body = body.child(
        crate::probe::measured("fw-status", at(3.0, 341.0, 450.0, 34.0))
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child(firmware.progress.status.clone()),
    );
    let live = firmware.live();
    for (_, id, text, (x, y, _, _), _) in LINKS {
        let element = match id {
            "fw-alloptions" => link_label(
                id,
                text,
                (x, y),
                live && firmware.links_enabled,
                |this, _window, _cx| this.install_firmware.all_options(&this.persisted),
                cx,
            ),
            "fw-custom" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, window, cx| {
                    this.install_firmware.custom(&this.persisted);
                    this.firmware_focus.focus(window, cx);
                },
                cx,
            ),
            "fw-beta" => link_label(
                id,
                text,
                (x, y),
                live && firmware.links_enabled,
                |this, _window, _cx| this.install_firmware.beta(),
                cx,
            ),
            // Force Bootloader and Bootloader Update: see [`BOOTLOADER_DISABLED`].
            _ => link_label(id, text, (x, y), false, |_, _, _| {}, cx),
        };
        body = body.child(element);
    }

    panel(
        "install firmware",
        div()
            .flex()
            .flex_col()
            .gap_2()
            .child(body)
            .child(choice(firmware)),
    )
    .into_any_element()
}

/// One `ImageLabel`: the vehicle's picture, the label under it, and the click - inert until
/// `Activate` has enabled it.
fn image_label(
    firmware: &InstallFirmware,
    index: usize,
    picture: &Picture,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let enabled = firmware.enabled.get(index).copied().unwrap_or(false);
    let picked = firmware.picked.as_ref().is_some_and(|(i, _)| *i == index);
    let label = firmware.labels.get(index).cloned().unwrap_or_default();
    let colour = if enabled { theme::TEXT } else { theme::DIM };
    let body = crate::probe::measured(picture.id, at(picture.x, picture.y, PICTURE, PICTURE))
        .id(SharedString::from(picture.id))
        .flex()
        .flex_col()
        .rounded_md()
        .border_1()
        .border_color(rgb(if picked { theme::ACCENT } else { theme::BORDER }))
        .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }))
        // `PictureBox`: the rest, the `Image` zoomed.
        .child(crate::pictures::image_label(
            "firmware",
            picture.id,
            picture.image,
            picture.name,
            colour,
        ))
        // `Label`: docked to the bottom, 13 high, centred.
        .child(
            div()
                .h(px(13.0))
                .mb_1()
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(colour))
                .child(label),
        );
    if enabled {
        body.cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.install_firmware.click(index);
                cx.notify();
            }))
            .into_any_element()
    } else {
        body.into_any_element()
    }
}

/// A line of the report beneath a page.
pub fn line(label: &'static str, value: String, colour: u32) -> Div {
    div()
        .flex()
        .gap_2()
        .text_xs()
        .child(div().w(px(70.0)).text_color(rgb(theme::DIM)).child(label))
        .child(div().text_color(rgb(colour)).child(value))
}

/// Where a flow got, as lines of the report.
pub fn reached_lines(reached: &Reached) -> Vec<Div> {
    let mut lines = Vec::new();
    if let Some(board) = &reached.board {
        lines.push(line("Board", board.clone(), theme::TEXT));
    }
    if let Some(url) = &reached.url {
        lines.push(line("Downloaded", url.clone(), theme::ACCENT));
    }
    if let Some(file) = &reached.file {
        let size = reached
            .size
            .map_or_else(String::new, |size| format!(" - {size} bytes"));
        lines.push(line(
            "File",
            format!("{}{size}", file.display()),
            theme::TEXT,
        ));
    }
    if let Some(firmware) = &reached.firmware {
        lines.push(line(
            "Firmware",
            format!(
                "board id {}, {} byte image - {}",
                firmware.board_id, firmware.image_len, firmware.description
            ),
            theme::TEXT,
        ));
    }
    if let Some(length) = reached.hex_len {
        lines.push(line("Image", format!("{length} bytes"), theme::TEXT));
    }
    if let Some(stop) = &reached.stop {
        lines.push(line("Stopped", stop.text(), theme::WARN));
    }
    if let Some(board) = &reached.flashed {
        lines.push(line(
            "Flashed",
            format!(
                "board id {}, revision {}, bootloader revision {}, {} bytes of flash",
                board.board_id, board.board_revision, board.bootloader_revision, board.flash_size
            ),
            theme::OK,
        ));
    }
    lines
}

/// Beneath the page: the board, what a click chose, where the flow got, and the Upload button
/// that stays disabled.
fn choice(firmware: &InstallFirmware) -> impl IntoElement {
    let board = match firmware.board() {
        Some((device, ids)) => format!(
            "{} - board id {}",
            device.board.as_deref().unwrap_or(""),
            ids.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        ),
        None if firmware.manifest.is_none() => format!(
            "{} USB device(s); the firmware list is {}",
            firmware.devices.len(),
            firmware.manifest_state()
        ),
        None => format!(
            "none of {} USB device(s) names a board",
            firmware.devices.len()
        ),
    };
    let mut column = div()
        .flex()
        .flex_col()
        .gap_1()
        .child(line("Board", board, theme::TEXT));

    if let Some((index, lookup)) = &firmware.picked {
        let label = firmware.labels.get(*index).cloned().unwrap_or_default();
        column = column.child(line("Firmware", label, theme::TEXT));
        if let Some(chosen) = lookup.as_ref().and_then(Lookup::chosen) {
            let version = chosen
                .mav_firmware_version_str
                .clone()
                .or_else(|| chosen.mav_firmware_version.map(|v| v.to_string()))
                .unwrap_or_default();
            column = column
                .child(line("Version", version, theme::TEXT))
                .child(line(
                    "URL",
                    chosen.url.clone().unwrap_or_default(),
                    theme::ACCENT,
                ));
        }
    }
    if let Some(reached) = &firmware.reached {
        column = column.children(reached_lines(reached));
    }

    column
        .child(line(
            "Bootloader",
            BOOTLOADER_DISABLED.to_owned(),
            theme::DIM,
        ))
        .child(upload_row("fw-upload"))
}

/// The Upload button, disabled, and why.
pub fn upload_row(id: &'static str) -> Div {
    div()
        .flex()
        .items_center()
        .gap_2()
        .pt_1()
        .child(action(id, "Upload", theme::ACCENT, false, |_, _, _| {}))
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(NOT_ENABLED),
        )
}

/// `ConfigFirmwareDisabled`: the explanation, and Bootloader Update - its two questions, then
/// `MAV_CMD_FLASH_BOOTLOADER` sent by the window - the report of a flow beneath.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.resx:121-206; ConfigFirmwareDisabled.cs:18-45`
fn disabled_page(firmware: &InstallFirmware, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    let body = div()
        .relative()
        .w(px(486.0))
        .h(px(236.0))
        .child(
            at(31.0, 29.0, 411.0, 72.0)
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(DISABLED_TEXT),
        )
        .child(at(34.0, 150.0, 116.0, 23.0).child(action(
            "fw-bootloader",
            "Bootloader Update",
            theme::ACCENT,
            firmware.bootloader.is_none(),
            cx.listener(|this, _event: &(), _window, cx| {
                let open = this.telemetry.view().connected;
                this.install_firmware.bootloader_update(open);
                cx.notify();
            }),
        )))
        .child(
            at(156.0, 155.0, 313.0, 13.0)
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(DISABLED_BOOTLOADER),
        );
    let report = div()
        .flex()
        .flex_col()
        .gap_1()
        .children(firmware.reached.iter().flat_map(reached_lines));
    panel(
        "install firmware",
        div().flex().flex_col().child(body).child(report),
    )
}

// ---------------------------------------------------------------------------------------------
// The boxes over the page.
// ---------------------------------------------------------------------------------------------

/// The ids a page's boxes are drawn with.
#[derive(Debug, Clone, Copy)]
pub struct BoxIds {
    /// A question.
    pub question: &'static str,
    /// Its Yes (or OK).
    pub yes: &'static str,
    /// Its No (or Cancel).
    pub no: &'static str,
    /// A message.
    pub message: &'static str,
    /// Its OK.
    pub ok: &'static str,
    /// The file dialog.
    pub path: &'static str,
    /// Its text box.
    pub path_value: &'static str,
    /// Its OK.
    pub path_ok: &'static str,
    /// Its Cancel.
    pub path_cancel: &'static str,
}

/// This page's.
const IDS: BoxIds = BoxIds {
    question: "fw-question",
    yes: "fw-question-yes",
    no: "fw-question-no",
    message: "fw-message",
    ok: "fw-message-ok",
    path: "fw-path",
    path_value: "fw-path-value",
    path_ok: "fw-path-ok",
    path_cancel: "fw-path-cancel",
};

/// A question, Yes and No or OK and Cancel, answered through `answer`.
pub fn question_box(
    ids: BoxIds,
    caption: &str,
    text: &str,
    buttons: Buttons,
    window: &Window,
    answer: impl Fn(&mut MissionPlanner, bool) + Clone + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (yes, no) = match buttons {
        Buttons::YesNo => ("Yes", "No"),
        Buttons::OkCancel => ("OK", "Cancel"),
    };
    let on_yes = answer.clone();
    let buttons = vec![
        action(
            ids.yes,
            yes,
            theme::ACCENT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                on_yes(this, true);
                cx.notify();
            }),
        ),
        action(
            ids.no,
            no,
            theme::ACCENT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                answer(this, false);
                cx.notify();
            }),
        ),
    ];
    crate::config::servo_output::modal(ids.question, caption, text, false, buttons, window)
}

/// A message with its OK, dismissed through `ok`.
pub fn message_box(
    ids: BoxIds,
    waiting: &Waiting,
    window: &Window,
    ok: impl Fn(&mut MissionPlanner) + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let button = action(
        ids.ok,
        "OK",
        theme::ACCENT,
        true,
        cx.listener(move |this, _event: &(), _window, cx| {
            ok(this);
            cx.notify();
        }),
    );
    let text = waiting.text.replace("\r\n", "\n");
    crate::config::servo_output::modal(
        ids.message,
        &waiting.caption,
        text.trim_end(),
        waiting.caption == flow::ERROR,
        vec![button],
        window,
    )
}

/// The file dialog: its caption, the path box with the keyboard, the filter, OK and Cancel.
#[allow(clippy::too_many_arguments)]
pub fn path_box(
    ids: BoxIds,
    path: &PathBox,
    handle: &FocusHandle,
    window: &Window,
    on_key: impl Fn(&mut MissionPlanner, &KeyDownEvent) -> bool + 'static,
    on_done: impl Fn(&mut MissionPlanner, bool) + Clone + 'static,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let focused = handle.is_focused(window);
    let on_ok = on_done.clone();
    let buttons = vec![
        action(
            ids.path_ok,
            "OK",
            theme::ACCENT,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                on_ok(this, true);
                cx.notify();
            }),
        ),
        action(
            ids.path_cancel,
            "Cancel",
            theme::DIM,
            true,
            cx.listener(move |this, _event: &(), _window, cx| {
                on_done(this, false);
                cx.notify();
            }),
        ),
    ];
    let dialog = crate::probe::measured(ids.path, div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(520.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(path.caption))
        .child(crate::textfield::text_field(
            ids.path_value,
            &path.field,
            handle,
            focused,
            px(490.0),
            cx.listener(move |this, event: &KeyDownEvent, _window, cx| {
                if on_key(this, event) {
                    cx.notify();
                }
            }),
        ))
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(path.filter),
        )
        .child(div().flex().justify_end().gap_2().children(buttons));
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id(SharedString::from(format!("{}-backdrop", ids.path)))
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(dialog),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// `FirmwareSelection` over the page: its heading, the platform it opened on, the Result rows
/// (`fw-select-<n>`), Upload Firmware and Close.
fn selection_box(
    choosing: &Choosing,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut rows = div().flex().flex_col().max_h(px(400.0)).overflow_hidden();
    for (row, result) in choosing.results().into_iter().enumerate() {
        let id = format!("fw-select-{row}");
        let selected = choosing.selected == Some(row);
        rows = rows.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .px_1()
                .text_xs()
                .whitespace_nowrap()
                .bg(rgb(if selected {
                    theme::BORDER
                } else {
                    theme::PANEL
                }))
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::ACTION)))
                .child(result)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.install_firmware.select(row);
                    cx.notify();
                })),
        );
    }
    let platform = choosing
        .platform()
        .unwrap_or_else(|| "(not chosen)".to_owned());
    let buttons = vec![
        action(
            "fw-select-upload",
            UPLOAD_FIRMWARE,
            theme::ACCENT,
            choosing.selected.is_some(),
            cx.listener(|this, _event: &(), _window, cx| {
                this.install_firmware.upload_selected(&this.persisted);
                cx.notify();
            }),
        ),
        action(
            "fw-select-close",
            "Close",
            theme::DIM,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.install_firmware.close_selection();
                cx.notify();
            }),
        ),
    ];
    let dialog = crate::probe::measured("fw-select", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(550.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(MORE_THAN_ONE),
        )
        .child(line("Platform", platform, theme::TEXT))
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(PICK_A_FILE),
        )
        .child(rows)
        .child(div().flex().justify_end().gap_2().children(buttons));
    let size = window.viewport_size();
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("fw-select-backdrop")
                    .w(size.width)
                    .h(size.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(dialog),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// The box showing over the page, if any: a message, the confirmation, the flow's question,
/// `FirmwareSelection`, or the file dialog.
pub fn overlay(
    firmware: &InstallFirmware,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !firmware.open {
        return None;
    }
    if let Some(waiting) = firmware
        .worker
        .as_ref()
        .and_then(|worker| worker.waiting.as_ref())
    {
        return Some(match waiting.buttons {
            Some(buttons) => question_box(
                IDS,
                &waiting.caption,
                &waiting.text,
                buttons,
                window,
                |this, yes| this.install_firmware.answer_worker(yes),
                cx,
            ),
            None => message_box(
                IDS,
                waiting,
                window,
                |this| this.install_firmware.answer_worker(true),
                cx,
            ),
        });
    }
    if let Some(waiting) = firmware.messages.front() {
        return Some(message_box(
            IDS,
            waiting,
            window,
            |this| this.install_firmware.dismiss_message(),
            cx,
        ));
    }
    if let Some(text) = firmware.bootloader.and_then(|at| BL_QUESTIONS.get(at)) {
        return Some(question_box(
            IDS,
            BL_UPDATE,
            text,
            Buttons::YesNo,
            window,
            |this, yes| this.install_firmware.answer_bootloader(yes),
            cx,
        ));
    }
    if let Some(index) = firmware.confirm {
        return Some(question_box(
            IDS,
            flow::CONTINUE,
            &firmware.confirm_text(index),
            Buttons::YesNo,
            window,
            |this, yes| this.install_firmware.answer_confirm(yes, &this.persisted),
            cx,
        ));
    }
    if let Some(choosing) = &firmware.selection {
        return Some(selection_box(choosing, window, cx));
    }
    let path = firmware.path.as_ref()?;
    Some(path_box(
        IDS,
        path,
        handle,
        window,
        |this, event| this.install_firmware.path_key(event, &mut this.persisted),
        |this, ok| this.install_firmware.path_done(ok, &mut this.persisted),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Manifest {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/firmware/manifest.json.gz");
        let bytes = std::fs::read(&path).expect("the fixture");
        Manifest::decode(&bytes, true).expect("parses")
    }

    /// A page holding the fixture, opened, with one device.
    fn loaded(board: &str) -> InstallFirmware {
        let mut page = InstallFirmware {
            manifest: Some(Arc::new(fixture())),
            ..InstallFirmware::default()
        };
        page.open = true;
        page.devices = vec![DeviceInfo::new(board, "")];
        page.label();
        page
    }

    fn index_of(id: &str) -> usize {
        PICTURES
            .iter()
            .position(|picture| picture.id == id)
            .expect("a picture")
    }

    /// The Designer's grid: two rows of six at 156 px, the Cube in the last cell.
    #[test]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // whole multiples of 156
    fn the_pictures_sit_where_the_designer_puts_them() {
        let mut cells: Vec<(u32, u32)> = PICTURES
            .iter()
            .map(|p| ((p.x - 3.0) as u32 / 156, (p.y - 3.0) as u32 / 156))
            .collect();
        cells.push((5, 1));
        cells.sort_unstable();
        cells.dedup();
        assert_eq!(cells.len(), 12, "every cell once");
        assert!(PICTURES.iter().all(|p| p.x + PICTURE <= PAGE.0));
        // `Activate`'s order, ConfigFirmwareManifest.cs:76-92.
        let named: Vec<&str> = PICTURES.iter().map(|p| p.control).collect();
        assert_eq!(
            named,
            [
                "pictureAntennaTracker",
                "pictureBoxHeli",
                "pictureBoxSub",
                "pictureBoxRover",
                "pictureBoxOctaQuad",
                "pictureBoxOcta",
                "pictureBoxY6",
                "pictureBoxTri",
                "pictureBoxHexa",
                "pictureBoxQuad",
                "pictureBoxPlane",
            ]
        );
    }

    /// Each of the Designer's 16 wirings is a picture's `PictureBox_Click` or a link's handler,
    /// at the place and with the text this page draws.
    #[test]
    fn every_wiring_and_link_is_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigFirmwareManifest.Designer.cs",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        for picture in PICTURES {
            let line = format!(
                "this.{}.Click += new System.EventHandler(this.PictureBox_Click);",
                picture.control
            );
            assert!(designer.contains(&line), "{line}");
        }
        for (name, _, text, (x, y, w, h), handler) in LINKS {
            let line = format!("this.{name}.Click += new System.EventHandler(this.{handler});");
            assert!(designer.contains(&line), "{line}");
            let place = format!("this.{name}.Location = new System.Drawing.Point({x}, {y});");
            assert!(designer.contains(&place), "{place}");
            let size = format!("this.{name}.Size = new System.Drawing.Size({w}, {h});");
            assert!(designer.contains(&size), "{size}");
            let caption = format!("this.{name}.Text = \"{text}\";");
            assert!(designer.contains(&caption), "{caption}");
        }
        assert_eq!(
            designer.matches(" += new ").count(),
            PICTURES.len() + LINKS.len()
        );
        assert_eq!(PICTURES.len() + LINKS.len(), 16);
    }

    #[test]
    fn activate_labels_every_picture_with_the_newest_of_its_release() {
        let page = loaded("CubeOrange-BL");
        assert!(page.links_enabled);
        assert!(page.enabled.iter().all(|enabled| *enabled));
        assert_eq!(page.labels[index_of("fw-quad")], "Copter V4.7.1 OFFICIAL");
        assert_eq!(page.labels[index_of("fw-octa")], "Copter V4.7.1 OFFICIAL");
        assert_eq!(page.labels[index_of("fw-plane")], "Plane V4.7.1 OFFICIAL");
        assert_eq!(
            page.labels[index_of("fw-tracker")],
            "AntennaTracker V4.7.1 OFFICIAL"
        );
        assert_eq!(
            page.detected.as_ref().map(|(_, ids)| ids.clone()),
            Some(vec![140])
        );
    }

    #[test]
    fn a_release_missing_a_vehicle_stops_labelling_there() {
        let mut manifest = fixture();
        // No submarine in the release: the third picture's `First` throws.
        manifest
            .firmware
            .retain(|a| a.mav_type.as_deref() != Some("SUBMARINE"));
        let mut page = InstallFirmware {
            manifest: Some(Arc::new(manifest)),
            open: true,
            ..InstallFirmware::default()
        };
        page.label();
        assert_eq!(page.enabled[..2], [true, true], "tracker and heli");
        assert!(page.enabled[2..].iter().all(|enabled| !enabled));
        assert!(page.labels[2..].iter().all(String::is_empty));
        // A disabled picture ignores its click.
        page.click(index_of("fw-quad"));
        assert!(page.confirm.is_none());
    }

    /// A click asks first; No does nothing, Yes looks for the port.
    #[test]
    fn a_click_asks_are_you_sure_first() {
        let mut page = loaded("CubeOrange-BL");
        let quad = index_of("fw-quad");
        page.click(quad);
        assert_eq!(
            page.question().as_deref(),
            Some("Are you sure you want to upload Copter V4.7.1 OFFICIAL?")
        );
        assert!(!page.live(), "the question is modal");
        page.answer_confirm(false, &Persisted::at(None));
        assert!(page.question().is_none());
        assert!(page.picked.is_none(), "No: LookForPort does not run");
    }

    #[test]
    fn a_click_chooses_the_boards_firmware_and_beta_changes_it() {
        let mut page = loaded("CubeOrange-BL");
        page.devices = vec![DeviceInfo::new("CubeOrange-BL", "")];
        // `pick` re-enumerates; the enumeration here is DEVICE_ENV or the machine's ports, so
        // this test takes the lookup `pick` makes by hand from the page's own devices.
        let manifest = page.manifest.clone().unwrap();
        let quad = index_of("fw-quad");
        let lookup = manifest.look_for_port(
            &page.devices,
            None,
            PICTURES[quad].mav_type,
            page.release,
            false,
        );
        page.picked = Some((quad, lookup.clone()));
        let chosen = page.lookup().and_then(Lookup::chosen).expect("chosen");
        assert_eq!(
            chosen.url.as_deref(),
            Some("https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj")
        );
        // Three files for board 140: FirmwareSelection opens, the board's platform selected.
        page.after_lookup(lookup, &Persisted::at(None));
        let choosing = page.selection.clone().expect("FirmwareSelection");
        assert_eq!(choosing.platform().as_deref(), Some("CubeOrange"));
        assert_eq!(
            choosing.results(),
            ["https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj"]
        );
        assert_eq!(choosing.selected, Some(0));
        page.close_selection();
        assert!(page.selection.is_none(), "closed: user canceled");
        assert!(page.worker.is_none());

        page.beta();
        assert_eq!(page.release, ReleaseType::Beta);
        assert!(page.picked.is_none(), "Activate again forgets the click");
        assert_eq!(page.labels[quad], "Copter V4.7.1 BETA");

        page.close();
        assert_eq!(page.release, ReleaseType::Official, "Deactivate resets it");
        assert!(!page.is_open());
    }

    #[test]
    fn no_board_is_a_message_and_all_options_offers_everything() {
        let mut page = loaded("FT232R USB UART");
        assert!(page.detected.is_none());
        page.after_lookup(None, &Persisted::at(None));
        assert_eq!(page.message().map(|w| w.text.as_str()), Some(NO_PORT));
        assert_eq!(page.message().map(|w| w.caption.as_str()), Some("Error"));
        page.dismiss_message();
        assert!(page.message().is_none());
        // All Options: the first device whatever its ids, and every record.
        let manifest = page.manifest.clone().unwrap();
        let lookup = manifest.look_for_port(
            &[DeviceInfo::default()],
            None,
            MavType::Copter,
            ReleaseType::Official,
            true,
        );
        page.after_lookup(lookup, &Persisted::at(None));
        let choosing = page.selection.as_ref().expect("FirmwareSelection");
        assert_eq!(
            choosing.results(),
            [format!(
                "To many options - apply more filters - {}",
                manifest.firmware.len()
            )]
        );
    }

    #[test]
    fn a_connected_vehicle_gets_the_disabled_page_and_no_fetch() {
        let mut page = InstallFirmware::default();
        page.open(true);
        assert!(page.is_open());
        assert_eq!(page.fetches, 0);
        assert!(page.receiver.is_none());
    }

    /// Bootloader Update: nothing with the link closed; two questions, and Yes to both stops
    /// before the command; No to either ends it.
    #[test]
    fn bootloader_update_asks_twice_then_owes_the_command() {
        let mut page = InstallFirmware::default();
        page.open(true);
        page.bootloader_update(false);
        assert!(page.question().is_none(), "BaseStream closed: return");
        page.bootloader_update(true);
        assert_eq!(page.question().as_deref(), Some(BL_QUESTIONS[0]));
        page.answer_bootloader(false);
        assert!(page.question().is_none());
        assert!(page.reached.is_none());
        page.bootloader_update(true);
        page.answer_bootloader(true);
        assert_eq!(page.question().as_deref(), Some(BL_QUESTIONS[1]));
        page.answer_bootloader(true);
        assert!(page.question().is_none());
        assert!(page.reached.is_none(), "no stop: the command itself is owed");
        assert!(page.take_bootloader_command());
        assert!(!page.take_bootloader_command(), "owed once");
        // The manifest page has no such button of its own: its link is dimmed.
        let mut manifest = InstallFirmware::default();
        manifest.open(false);
        manifest.bootloader_update(true);
        assert!(manifest.question().is_none());
    }

    /// The product's path: the fetch on its thread, collected by `tick`, labelling the page.
    #[test]
    fn the_fetch_thread_brings_the_catalogue_and_tick_labels_the_page() {
        let mut page = InstallFirmware {
            open: true,
            ..InstallFirmware::default()
        };
        page.fetch_from(Box::new(manifest::FromFile {
            path: std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../testdata/firmware/manifest.json.gz"),
        }));
        assert_eq!(page.fetches, 1);
        assert_eq!(page.manifest_state(), "fetching");
        // A second request while one is under way starts nothing.
        page.fetch_from(Box::new(manifest::Http));
        assert_eq!(page.fetches, 1);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
        while page.receiver.is_some() && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(10));
            page.tick(true);
        }
        assert_eq!(page.manifest_state(), "loaded");
        assert_eq!(page.manifest.as_ref().map(|m| m.firmware.len()), Some(240));
        // The peripheral manifest is not fetched from a file, and that is logged, not fatal.
        assert_eq!(page.errors.len(), 1, "{:?}", page.errors);
        assert!(page.links_enabled);
        assert_eq!(page.labels[index_of("fw-rover")], "Rover V4.7.1 OFFICIAL");
    }

    #[test]
    fn the_catalogue_is_fetched_once_and_kept() {
        let mut page = InstallFirmware {
            manifest: Some(Arc::new(fixture())),
            ..InstallFirmware::default()
        };
        page.open(false);
        page.close();
        page.open(false);
        assert_eq!(page.fetches, 0, "held: nothing to fetch");
        assert!(page.links_enabled);
        // Leaving the setup screen closes it, as leaving Initial Setup deactivates it.
        page.tick(false);
        assert!(!page.is_open());
    }

    /// The flow's thread: its questions wait for their answers, its progress moves the bar, and
    /// its end is where it got - here, a download of the fixture from a mirror, read, and stopped
    /// at the reboot into the bootloader.
    #[test]
    fn a_flow_on_its_thread_is_heard_and_answered() {
        let web = std::env::temp_dir().join(format!("mp-gui-fw-flow-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&web);
        let served = web.join("web/firmware.ardupilot.org/Copter/stable/CubeOrange");
        std::fs::create_dir_all(&served).unwrap();
        let apj = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../mp-firmware/testdata/legacy/arducopter.apj");
        std::fs::copy(&apj, served.join("arducopter.apj")).unwrap();
        let machine = Machine {
            fetch: Box::new(manifest::Mirror {
                root: web.join("web"),
            }),
            user_data: web.join("data"),
            temp_dir: web.join("tmp"),
            comport: String::new(),
            baud: String::new(),
            flash: false,
            devices: Vec::new(),
            rows: Vec::new(),
        };
        let mut worker = Worker::start("test-flow", move |dialogue| {
            machine.run(dialogue, |cx, _| {
                if !cx.dialogue.ask("Go?", "Test", Buttons::YesNo) {
                    return Reached::default();
                }
                flow::download_and_flash(
                    cx,
                    "https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj",
                    "",
                )
            })
        })
        .expect("a thread");
        let mut progress = Progress::default();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        let mut done = None;
        while done.is_none() && std::time::Instant::now() < deadline {
            done = worker.poll(&mut progress);
            if let Some(waiting) = worker.waiting.clone() {
                assert_eq!(waiting.text, "Go?");
                assert_eq!(waiting.buttons, Some(Buttons::YesNo));
                worker.answer(true);
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let reached = done.expect("the flow ended");
        assert_eq!(reached.firmware.map(|f| f.board_id), Some(140));
        assert_eq!(reached.stop, Some(flow::Stop::RebootToBootloader));
        assert_eq!(progress.status, "Reading Hex File");
        assert_eq!(
            progress.value, 100,
            "-1 leaves the bar at the download's end"
        );
        let _ = std::fs::remove_dir_all(&web);
    }

    /// The file dialog opens in the folder last used, takes only a file that exists, and saves
    /// the folder of the one it takes.
    #[test]
    fn load_custom_firmware_remembers_the_folder() {
        let dir = std::env::temp_dir().join(format!("mp-gui-fw-custom-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("custom.apj");
        std::fs::write(&file, "{}").unwrap();
        let mut settings = Persisted::at(None);
        settings.set(FIRMWARE_FILE_DIRECTORY, dir.display().to_string());
        let mut page = loaded("CubeOrange-BL");
        page.custom(&settings);
        let opened = page.path.as_ref().expect("the dialog");
        assert!(opened.field.value().starts_with(&dir.display().to_string()));
        // A name that is not there: nothing.
        page.path
            .as_mut()
            .unwrap()
            .field
            .set(dir.join("none.apj").display().to_string());
        page.path_done(true, &mut settings);
        assert!(page.worker.is_none());
        // One that is: its folder saved, and the flow started.
        page.custom(&settings);
        page.path
            .as_mut()
            .unwrap()
            .field
            .set(file.display().to_string());
        page.path_done(true, &mut settings);
        assert!(page.worker.is_some());
        assert_eq!(
            settings.get(FIRMWARE_FILE_DIRECTORY),
            Some(dir.display().to_string().as_str())
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}

/// The firmware page's reboot into the bootloader, over the real link to a scripted copter.
#[cfg(test)]
mod reboot_to_bootloader_tests {
    use std::time::{Duration, Instant};

    use mp_link::ProtocolTimeouts;
    use mp_link::requests::CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    use mp_mavlink_dialects::all::MavMessage;

    use super::{LinkReboot, RebootWaits, reboot_to_bootloader};
    use crate::telemetry::scripted::{VEHICLE, Vehicle, until};

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// Each `PREFLIGHT_REBOOT_SHUTDOWN` the copter heard, in order: param1 and confirmation,
    /// each checked to be addressed to it.
    fn reboots(vehicle: &Vehicle) -> Vec<(f32, u8)> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::CommandLong(long) if long.command == CMD_PREFLIGHT_REBOOT_SHUTDOWN => {
                    assert_eq!(
                        (long.target_system, long.target_component),
                        (VEHICLE.sysid, VEHICLE.compid)
                    );
                    Some((long.param1, long.confirmation))
                }
                _ => None,
            })
            .collect()
    }

    /// `doReboot(true, false)`'s four frames: 3, 3, 1, 1, all at confirmation 0.
    const FOUR: [(f32, u8); 4] = [(3.0, 0), (3.0, 0), (1.0, 0), (1.0, 0)];

    /// Seen, then the next heartbeat waited for, then `doCommand` 3 and `doCommand` 1, each
    /// written twice and neither waited on: four frames, 3, 3, 1, 1, and nothing after them
    /// however long the copter stays quiet.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2591-2618, 2717, 2758-2763`
    #[test]
    fn four_frames_3_3_1_1_go_out_after_the_next_heartbeat() {
        let (link, mut vehicle) = Vehicle::link(fast());
        let waits = RebootWaits {
            window: Duration::from_secs(5),
            heartbeat: Duration::from_secs(4),
        };
        let reached = std::thread::scope(|scope| {
            let task = scope.spawn(|| reboot_to_bootloader(&link, Instant::now(), waits));
            // `doReboot`'s own `getHeartBeat` holds the reboots until the next heartbeat.
            std::thread::sleep(Duration::from_millis(200));
            vehicle.read();
            assert_eq!(reboots(&vehicle), [], "sent before the second heartbeat");
            vehicle.heartbeat();
            let reached = task.join().expect("the task");
            until("the four reboots", || {
                vehicle.read();
                reboots(&vehicle).len() >= 4
            });
            reached
        });
        assert_eq!(reached, LinkReboot::Rebooted);
        std::thread::sleep(fast().command.timeout * 3);
        vehicle.read();
        assert_eq!(reboots(&vehicle), FOUR);
    }

    /// No second heartbeat: `getHeartBeat` gives up after its 2.2 s, and the ids the first one
    /// set still send the four frames.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1197-1201, 2594-2614`
    #[test]
    fn the_four_frames_go_out_when_the_next_heartbeat_never_comes() {
        let (link, mut vehicle) = Vehicle::link(fast());
        let waits = RebootWaits {
            window: Duration::from_secs(5),
            heartbeat: Duration::from_millis(300),
        };
        let started = Instant::now();
        assert_eq!(
            reboot_to_bootloader(&link, started, waits),
            LinkReboot::Rebooted
        );
        assert!(started.elapsed() >= waits.heartbeat);
        until("the four reboots", || {
            vehicle.read();
            reboots(&vehicle).len() >= 4
        });
        assert_eq!(reboots(&vehicle), FOUR);
    }

    /// Longer than `task.Wait`'s window is "Please unplug the board", though the task goes on
    /// and its reboots still go out.
    /// `// C#: Utilities/Firmware.cs:803-828`
    #[test]
    fn past_the_window_is_please_unplug_though_the_reboots_went_out() {
        let (link, mut vehicle) = Vehicle::link(fast());
        let waits = RebootWaits {
            window: Duration::from_millis(100),
            heartbeat: Duration::from_millis(300),
        };
        assert_eq!(
            reboot_to_bootloader(&link, Instant::now(), waits),
            LinkReboot::NoHeartbeat
        );
        until("the four reboots", || {
            vehicle.read();
            reboots(&vehicle).len() >= 4
        });
        assert_eq!(reboots(&vehicle), FOUR);
    }

    /// No heartbeat at all: "No HeartBeat found", and nothing sent.
    /// `// C#: Utilities/Firmware.cs:810-819`
    #[test]
    fn no_heartbeat_sends_nothing() {
        let (_vehicle_side, gcs_side) = mp_transport::testing::Loopback::pair();
        let link = mp_link::Link::from_transport(
            Box::new(gcs_side),
            mp_link::LinkConfig {
                send_heartbeat: false,
                stream_rate_hz: 0,
                timeouts: fast(),
                ..mp_link::LinkConfig::default()
            },
        );
        let waits = RebootWaits {
            window: Duration::from_secs(5),
            heartbeat: Duration::from_millis(200),
        };
        assert_eq!(
            reboot_to_bootloader(&link, Instant::now(), waits),
            LinkReboot::NoHeartbeat
        );
        assert_eq!(link.stats().frames_sent, 0);
    }
}
