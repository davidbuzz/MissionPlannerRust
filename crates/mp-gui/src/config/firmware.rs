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

//! Install Firmware: the first page of Initial Setup.
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
//! clicked, or `DEV` after Ctrl+Q and its warning - enabling it as it goes. Clicking a picture
//! asks "Are you sure you want to upload ...?"; Yes runs `LookForPort`, which takes the board id a
//! bootloader reported when a device arrived while the page showed, or finds the board among the
//! USB devices, and picks its firmware - one file outright, several through `FirmwareSelection`,
//! whose pickers narrow the list from the board's own platform and which waits for "Upload
//! Firmware" - downloads it to a temporary file with the progress bar and status line following,
//! and hands it to `UploadFlash`, which reboots the board into its bootloader and writes it
//! (`mp_firmware::flow::upload_px4`). All Options is `LookForPort` over the whole catalogue. Load
//! custom firmware opens a file and hands it to `UploadFlash` by its extension
//! (`mp_firmware::flow::custom_manifest`). Force Bootloader opens the window's link and reboots
//! the board into its bootloader - the legacy page's handler too, so both pages hold the one in
//! `force_bootloader.rs`; Bootloader Update opens a link of its own, asks twice, and sends
//! `MAV_CMD_FLASH_BOOTLOADER`.
//!
//! All of it runs as the C# runs it - the flows on a thread, with their questions and messages as
//! modal boxes over the page. The layout is `ConfigFirmwareManifest.Designer.cs`'s - every control
//! at its `Location` and `Size` in a 946 x 375 page - each picture its `Image`, zoomed as
//! `ImageLabel`'s `PictureBox` zooms it ([`crate::pictures`]), and this application's colours.
//! Beneath the page are the board found, the firmware chosen, the file downloaded and what it says
//! of itself.
//!
//! What is not here, and why:
//!
//! * the uploads that are not a px4 bootloader's: DFU, the STK500 upload and probes, VRBRAIN,
//!   Parrot and Solo end where they would write to a board (`mp_firmware::flow::Stop`), the
//!   report beneath saying which step that was and that it is not ported ([`stop_text`]);
//! * `Tracking.AddFW` and `AddTiming`, Mission Planner's analytics.
//!
//! The device list is the port enumeration through `mp_firmware::detect::DeviceInfo::from_port`.
//! [`DEVICE_ENV`] replaces it with one named device, so a test does not depend on what is plugged
//! into the machine running it - and then nothing opens this machine's ports: no upload, no probe
//! on a device's arrival. With [`mp_firmware::manifest::OVERRIDE_ENV`] and
//! [`mp_firmware::manifest::MIRROR_ENV`] a test runs offline.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, TryRecvError, channel};
use web_time::{Duration, Instant};

use gpui::{
    AnyElement, Context, Div, FocusHandle, KeyDownEvent, MouseButton, SharedString, Window, div,
    prelude::*, px, rgb,
};
use mp_firmware::detect::{DeviceInfo, TransportPort, Win32SerialPort, open_serial};
use mp_firmware::flow::{self, Buttons, Dialogue, FlashHost, FoundBoard, LinkReboot, Reached};
use mp_firmware::manifest::{
    self, Filter, Lookup, Manifest, MavType, NO_FIRMWARE, NO_PORT, Outcome, Pickers, ReleaseType,
    icon_name,
};

use super::force_bootloader::{ForceBootloader, Page as ForcePage};
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::telemetry::{Report, Telemetry, TelemetryView};
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

/// What the legacy page's Upload button says for itself: its uploads are not ported.
#[cfg(test)]
pub const NOT_PORTED: &str = flow::NOT_PORTED;

/// Why the manifest page's px4 upload stops before the board while [`DEVICE_ENV`] names the
/// device: nothing opens this machine's ports then ([`Machine::flash`]).
pub const NO_BOARD_WRITTEN: &str = "no board is written while MP_FIRMWARE_DEVICE names the device";

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

/// `ConfigFirmwareDisabled.resx`'s `label2.Text`, beside Bootloader Update: the C#'s words, which
/// say nothing of flashing being disabled.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.resx:187-188`
const UPDATE_THE_BOOTLOADER: &str =
    "Update the bootloader on your hardware to the newest available.";

/// `but_bootloaderupdate_Click`'s two questions, caption "BL Update".
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:26-30`
pub const BL_QUESTIONS: [&str; 2] = [
    "Are you sure you want to upgrade the bootloader? This can brick your board",
    "Are you sure you want to upgrade the bootloader? This can brick your board, Please allow 5 \
     mins for this process",
];

/// Their caption. `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:27`
pub const BL_UPDATE: &str = "BL Update";

/// `Strings.TrunkWarning`: Ctrl+Q's box, before the `DEV` release.
/// `// C#: ExtLibs/Strings/Strings.resx:483-485; GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:403`
pub const TRUNK_WARNING: &str = "These are the latest trunk firmware, use at your own risk!!!";

/// `Strings.Trunk`: its caption. `// C#: ExtLibs/Strings/Strings.resx:480-482`
pub const TRUNK: &str = "trunk";

/// Bootloader Update's words when its link found no vehicle: a box in the C#, the status line
/// here, as Force Bootloader's [`FORCE_FAILED`](super::force_bootloader::FORCE_FAILED).
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:544`
pub const NO_DEVICE_ON_MAVLINK: &str = "Failed to find device on mavlink";

/// `FLASH_BOOTLOADER` accepted. A box in the C#; the status line here, as the window says every
/// command's outcome. `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:554; ConfigFirmwareDisabled.cs:34`
pub const UPGRADED_BOOTLOADER: &str = "Upgraded bootloader";

/// `FLASH_BOOTLOADER` refused. `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:558; ConfigFirmwareDisabled.cs:38`
pub const FAILED_TO_UPGRADE_BOOTLOADER: &str = "Failed to upgrade bootloader";

/// `doCommand`'s `TimeoutException` once its retry has gone unanswered, which the manifest
/// page's handler does not catch: the application's error handler shows it in the C#, the status
/// line here. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2781-2794`
pub const DO_COMMAND_TIMEOUT: &str = "Timeout on read - doCommand";

/// `MAVLinkInterface.CONNECT_TIMEOUT_SECONDS`' default: how long `Open` waits for heartbeats.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:327`
pub(super) const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

/// How often the manifest page's [`Watcher`] enumerates the serial ports while the page is
/// active, to see a device arrive: the C#'s `WM_DEVICECHANGE` has no counterpart here.
const ARRIVAL_POLL: Duration = Duration::from_millis(500);

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
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1199`
pub(super) const HEARTBEAT_WAIT: std::time::Duration = std::time::Duration::from_millis(2200);

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
///
/// Divergence, the owner's ruling of 2026-09-27: the vehicle is one whose heartbeat is not a
/// broadcast component's ([`rebootable_vehicle`]). `getHeartBeat` takes any heartbeat but a
/// GCS's, and a Cube with an ADS-B receiver sends two - the autopilot's as component 1, the
/// receiver's as component 0 (type ADSB, autopilot invalid) - so the C# took the receiver's
/// about half the time, and then its `compid != 0` sent no reboot and the scan found no
/// bootloader: "No Response from board" on the bench CubeOrange's second Windows flash. The
/// C#'s own connect passes those heartbeats over ("no broadcast compid's (ping adsb)"); so does
/// this.
/// `// C#: Utilities/Firmware.cs:797-837; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:823-828, 1155-1205, 2550-2556, 2588-2615, 2714, 2755-2760`
fn reboot_to_bootloader(
    link: &mp_link::Link,
    started: web_time::Instant,
    waits: RebootWaits,
) -> LinkReboot {
    use web_time::{Duration, Instant};
    let poll = Duration::from_millis(10);
    // `MainV2.comPort.getHeartBeat().Length > 0`, else "No HeartBeat found".
    let first = started + waits.heartbeat;
    let (id, handle) = loop {
        if let Some(vehicle) = rebootable_vehicle(link) {
            break vehicle;
        }
        if Instant::now() >= first {
            return LinkReboot::NoHeartbeat;
        }
        wasm_thread::sleep(poll);
    };
    // `doReboot(true, false)`: `getHeartBeat` again, the heartbeat after the one seen.
    let seen = handle.load().heartbeats;
    let again = Instant::now() + waits.heartbeat;
    while handle.load().heartbeats <= seen && Instant::now() < again {
        wasm_thread::sleep(poll);
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
            wasm_thread::sleep(Duration::from_millis(1));
        }
    }
    if started.elapsed() <= waits.window {
        LinkReboot::Rebooted
    } else {
        LinkReboot::NoHeartbeat
    }
}

/// The vehicle the reboots go to: the autopilot, component 1, if it has been heard, else any
/// component but 0 - a ping ADS-B receiver's broadcast id, which `Open`'s connect loop passes
/// over and so does this ([`reboot_to_bootloader`]'s divergence).
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:823-828`
fn rebootable_vehicle(
    link: &mp_link::Link,
) -> Option<(mp_vehicle::VehicleId, mp_vehicle::StateHandle)> {
    let ids = link.vehicles();
    let id = ids
        .iter()
        .find(|id| id.compid == 1)
        .or_else(|| ids.iter().find(|id| id.compid != 0))?;
    link.vehicle(*id).map(|handle| (*id, handle))
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
        let started = web_time::Instant::now();
        let url = format!("serial:{}:{}", self.comport, self.baud);
        let Ok(link) = mp_link::Link::connect(&url, mp_link::LinkConfig::default()) else {
            return LinkReboot::NoHeartbeat;
        };
        let reached = reboot_to_bootloader(&link, started, REBOOT_WAITS);
        // `MainV2.comPort.Close()`.
        drop(link);
        reached
    }
    fn now(&mut self) -> web_time::Instant {
        web_time::Instant::now()
    }
    fn sleep(&mut self, duration: std::time::Duration) {
        wasm_thread::sleep(duration);
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
        wasm_thread::Builder::new()
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
                Err(TryRecvError::Empty) => {
                    crate::repaint::in_flight();
                    return None;
                }
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
                .unwrap_or_else(|| mp_os::temp_dir().join("MissionPlannerRust")),
            temp_dir: mp_os::temp_dir(),
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
        if !directory.is_empty() && Path::new(directory).os_is_dir() {
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
        path.os_is_file().then_some(path)
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

/// One of `FirmwareSelection`'s pickers: a filter, or `Result`, the Firmwares list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickerId {
    /// A filter picker.
    Filter(Filter),
    /// `Result`.
    Result,
}

impl PickerId {
    /// The id its box is drawn and clicked by; its rows add `-<index>`.
    #[must_use]
    pub fn id(self) -> String {
        match self {
            Self::Filter(filter) => format!("fw-select-{}", filter.name()),
            Self::Result => "fw-select-result".to_owned(),
        }
    }
}

/// `FirmwareSelection`, open over a lookup's records: its pickers, and the one whose list is
/// dropped down with the first row it shows.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Choosing {
    /// The records and the device.
    pub lookup: Lookup,
    /// The pickers.
    pub pickers: Pickers,
    /// The picker whose list is down, and its top row.
    pub open: Option<(PickerId, usize)>,
}

impl Choosing {
    /// The constructor: the pickers for the lookup's device over its records.
    /// `// C#: test/FirmwareSelection.xaml.cs:15-59`
    fn open(lookup: Lookup) -> Self {
        let pickers = Pickers::open(&lookup.items, &lookup.device);
        Self {
            lookup,
            pickers,
            open: None,
        }
    }

    /// The Result picker's rows.
    #[must_use]
    pub fn results(&self) -> &[String] {
        &self.pickers.result.items
    }

    /// A picker.
    #[must_use]
    pub fn picker(&self, id: PickerId) -> &manifest::Picker {
        match id {
            PickerId::Filter(filter) => self.pickers.picker(filter),
            PickerId::Result => &self.pickers.result,
        }
    }

    /// A picker's list dropped down, or put away when it is the one down: scrolled, as a
    /// WinForms `ComboBox` drops its list, so the selected row shows.
    pub fn toggle(&mut self, id: PickerId) {
        if self.open.is_some_and(|(open, _)| open == id) {
            self.open = None;
            return;
        }
        let picker = self.picker(id);
        let top = picker
            .index
            .unwrap_or(0)
            .saturating_sub(LIST_ROWS - 1)
            .min(picker.items.len().saturating_sub(LIST_ROWS));
        self.open = Some((id, top));
    }

    /// The wheel over a dropped list: `lines` rows down, negative up, kept within the list.
    pub fn scroll(&mut self, lines: i32) {
        let Some((id, top)) = self.open else {
            return;
        };
        let last = self.picker(id).items.len().saturating_sub(LIST_ROWS);
        let rows = usize::try_from(lines.unsigned_abs()).unwrap_or(usize::MAX);
        let top = if lines < 0 {
            top.saturating_sub(rows)
        } else {
            top.saturating_add(rows).min(last)
        };
        self.open = Some((id, top));
    }

    /// A row of a dropped list chosen: the picker's `SelectedIndex`, and the list put away.
    pub fn choose(&mut self, id: PickerId, row: usize) {
        match id {
            PickerId::Filter(filter) => self.pickers.choose(&self.lookup.items, filter, row),
            PickerId::Result => self.pickers.choose_result(row),
        }
        self.open = None;
    }
}

/// How many rows a dropped list shows before it scrolls.
const LIST_ROWS: usize = 30;

/// A dropped list's row height.
const LIST_ROW: f32 = 16.0;

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
    /// `flashdone`: set once `LookForPort`'s download is in and `UploadFlash` begins, cleared by
    /// `Activate` and `Deactivate`; while set, a device arriving is not probed. Shared with the
    /// flow's thread, which sets it.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:63, 131, 139-140, 335, 411`
    flashdone: Arc<AtomicBool>,
    /// The serial ports the page watches for a device's arrival - this machine's - or `None`: not
    /// in a unit test, which may give its own, and not while [`DEVICE_ENV`] names the device.
    watch_ports: Option<fn() -> Vec<String>>,
    /// Arrivals heard and probed since the page was made, for a test to see one reach the probe.
    arrivals: usize,
    /// The page's flow threads still running, counted across `Activate` and `Deactivate`: a
    /// flash left running when the page was left goes on scanning the ports for its bootloader,
    /// and a probe holding one of them would fail its open. The C#'s page cannot be left mid-flash.
    flows: Arc<std::sync::atomic::AtomicUsize>,
    /// `Instance_DeviceChanged` subscribed to `DeviceChanged`: from `Activate` to `Deactivate`.
    watcher: Option<Watcher>,
    /// The probes' threads' end of the channel, handed to each probe.
    found_sender: Sender<FoundBoard>,
    /// What the probes have found.
    found_receiver: Receiver<FoundBoard>,
    /// `detectedport` and `detectedboardid`: the bootloader the last probe found, which
    /// `LookForPort` takes before the USB guess. Kept for the life of the SETUP screen, as the
    /// C# page object keeps them.
    found: Option<FoundBoard>,
    /// Ctrl+Q's warning is showing; its OK goes on to the `DEV` release.
    trunk: bool,
    /// Force Bootloader: its steps over the window's link, its box and its failure - the
    /// legacy page's handler too ([`ForceBootloader`]).
    force: ForceBootloader,
    /// Bootloader Update on this page: its own link, under way.
    bl: Option<BlLink>,
    /// The window's connection prompts are asking the transport's questions for Bootloader
    /// Update's link, not the window's.
    bl_asking: bool,
    /// What the page has to say on the window's status line.
    status: Option<String>,
}

/// Bootloader Update on the manifest page: the link `doConnect(mav, ...)` opened, and how far it
/// has got.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:535-562`
#[derive(Debug)]
pub struct BlLink {
    /// `mav`.
    pub link: Telemetry,
    /// Where it is.
    pub stage: BlStage,
}

/// Where Bootloader Update's link has got.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlStage {
    /// `Open`'s connect loop, as for Force Bootloader.
    Opening {
        /// `CONNECT_TIMEOUT_SECONDS` after it opened.
        deadline: Instant,
    },
    /// One of the two "BL Update" questions.
    Asking(usize),
    /// `doCommand(FLASH_BOOTLOADER, ...)` waiting for its answer.
    Commanding,
}

impl Default for InstallFirmware {
    fn default() -> Self {
        let (found_sender, found_receiver) = channel();
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
            flashdone: Arc::new(AtomicBool::new(false)),
            // A unit test never enumerates, let alone opens, this machine's ports.
            watch_ports: (!cfg!(test) && std::env::var_os(DEVICE_ENV).is_none())
                .then_some(port_identities as fn() -> Vec<String>),
            arrivals: 0,
            flows: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            watcher: None,
            found_sender,
            found_receiver,
            found: None,
            trunk: false,
            force: ForceBootloader::default(),
            bl: None,
            bl_asking: false,
            status: None,
        }
    }
}

/// Whether `Open`'s connect loop would be done: the vehicle shown heard twice from component 1,
/// or four times from another.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:871-893`
pub(super) fn heard_enough(view: &TelemetryView) -> bool {
    let (Some(vehicle), Some(state)) = (view.vehicle, view.state.as_deref()) else {
        return false;
    };
    (state.heartbeats >= 2 && vehicle.compid == 1) || state.heartbeats >= 4
}

/// Whether a port has appeared since the last enumeration: `DBT_DEVICEARRIVAL`, as far as the
/// serial ports show it. A divergence forced by the platform: the C# hears every device's
/// arrival from Windows' `WM_DEVICECHANGE` and probes the ports on any, where here there is no
/// such message and an arrival is seen only as a serial port new or changed (`port_identities`),
/// so a device that brings none - a memory stick, say - starts no probe.
/// `// C#: MainV2.cs:4527-4584; GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:134-137`
#[must_use]
pub fn arrived(seen: &[String], now: &[String]) -> bool {
    now.iter().any(|port| !seen.contains(port))
}

/// `MainV2.DeviceChanged` as the manifest page hears it here: a thread enumerating the serial
/// ports every [`ARRIVAL_POLL`] - off the UI thread, as the enumeration asks the OS for its
/// devices (SetupAPI on Windows) - and sending the list each time a port has appeared since the
/// enumeration before ([`arrived`]). Dropped, it stops: `DeviceChanged -= Instance_DeviceChanged`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:47-48, 126, 134-137; MainV2.cs:4527-4584`
#[derive(Debug)]
struct Watcher {
    /// Set when dropped; the thread ends at its next enumeration.
    stop: Arc<AtomicBool>,
    /// The ports at each arrival.
    arrivals: Receiver<Vec<String>>,
}

impl Watcher {
    /// Starts the thread over `enumerate`, the ports as they are now its first list. `None` when
    /// the thread cannot be started, and then no arrival is heard.
    fn start(
        enumerate: impl Fn() -> Vec<String> + Send + 'static,
        every: Duration,
    ) -> Option<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let stopped = Arc::clone(&stop);
        let (sender, arrivals) = channel();
        wasm_thread::Builder::new()
            .name("mp-firmware-ports".to_owned())
            .spawn(move || {
                let mut seen = enumerate();
                loop {
                    wasm_thread::sleep(every);
                    if stopped.load(Ordering::Relaxed) {
                        return;
                    }
                    let now = enumerate();
                    if arrived(&seen, &now) && sender.send(now.clone()).is_err() {
                        return;
                    }
                    seen = now;
                }
            })
            .ok()?;
        Some(Self { stop, arrivals })
    }
}

/// A flow thread counted in [`InstallFirmware`]'s `flows` for as long as it lives, however it
/// ends - its closure dropped unrun too, when the thread cannot start.
struct Running(Arc<std::sync::atomic::AtomicUsize>);

impl Running {
    fn new(count: &Arc<std::sync::atomic::AtomicUsize>) -> Self {
        count.fetch_add(1, Ordering::SeqCst);
        Self(Arc::clone(count))
    }
}

impl Drop for Running {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::SeqCst);
    }
}

impl Drop for Watcher {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl InstallFirmware {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open
    }

    /// Whether the page's Force Bootloader is under way over the window's link
    /// ([`ForceBootloader::forcing`]): SETUP is not shown again under it.
    #[must_use]
    pub const fn forcing(&self) -> bool {
        self.force.forcing()
    }

    /// The page's Force Bootloader, for the window to click and drive.
    pub fn force_bootloader(&mut self) -> &mut ForceBootloader {
        &mut self.force
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

    /// `Activate`: subscribes to devices' arrivals, disables the pictures and links, clears
    /// `flashdone`, then labels them from the catalogue - at once when it is held, else when the
    /// fetch this starts has brought it.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:45-94`
    pub fn activate(&mut self) {
        // `DeviceChanged -= Instance_DeviceChanged; DeviceChanged += Instance_DeviceChanged`:
        // arrivals from now on, against the ports there are now.
        self.watcher = self
            .watch_ports
            .and_then(|ports| Watcher::start(ports, ARRIVAL_POLL));
        self.enabled = [false; 11];
        self.links_enabled = false;
        // `flashdone = false`: a flow still running from before keeps its own flag.
        self.flashdone = Arc::new(AtomicBool::new(false));
        self.picked = None;
        self.devices = devices();
        if self.manifest.is_some() {
            self.label();
        } else {
            self.fetch();
        }
    }

    /// `Deactivate`: arrivals no longer heard, back to the official release for next time, and
    /// `flashdone` cleared. The page's boxes close with it, a flow still running answers No to
    /// whatever it asks next, and Bootloader Update's link closes - the C#'s handlers hold the
    /// window until they end, so nothing of theirs outlives the page there.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:124-132`
    pub fn close(&mut self) {
        self.open = false;
        // `DeviceChanged -= Instance_DeviceChanged`.
        self.watcher = None;
        self.release = ReleaseType::Official;
        self.flashdone = Arc::new(AtomicBool::new(false));
        self.picked = None;
        self.confirm = None;
        self.selection = None;
        self.worker = None;
        self.messages.clear();
        self.trunk = false;
        self.path = None;
        self.bootloader = None;
        self.force.cancel();
        self.bl = None;
    }

    /// The SETUP screen disposed and made anew - a connect, a disconnect, its tab clicked again -
    /// where leaving the page only deactivates it: a new page object, with no bootloader found.
    /// Kept across the page's own deactivations ([`Self::close`]), as the C#'s page object is.
    /// `// C#: MainV2.cs:1331, 1349, 1424, 1747, 3186; ExtLibs/Controls/MainSwitcher.cs:112-135`
    pub fn screen_disposed(&mut self) {
        self.found = None;
    }

    /// Once a frame: takes a finished fetch, hears the flow running and the probes of a device's
    /// arrival, watches the ports, and closes the page when the screen changes, as leaving
    /// Initial Setup deactivates its page - and forgets the bootloader found, as leaving disposes
    /// the screen and the page object with it (`HWConfig` is not a persistent screen).
    /// `// C#: MainV2.cs:3186; ExtLibs/Controls/MainSwitcher.cs:112-135`
    pub fn tick(&mut self, on_setup: bool) {
        while let Ok(board) = self.found_receiver.try_recv() {
            self.found_board(board);
        }
        if on_setup {
            if let Some(ports) = self.hear_arrival() {
                self.device_arrived(ports);
            }
        } else {
            self.found = None;
        }
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
                Err(TryRecvError::Empty) => crate::repaint::in_flight(),
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

    /// What the [`Watcher`] has heard since the last frame: the serial ports as they were at the
    /// latest arrival, for the probe, while the manifest page is active.
    fn hear_arrival(&mut self) -> Option<Vec<String>> {
        let watcher = self.watcher.as_ref()?;
        // Watching the ports: an arrival looked for as the timer looked for it.
        crate::repaint::in_flight();
        let latest = watcher.arrivals.try_iter().last()?;
        (self.open && !self.connected).then_some(latest)
    }

    /// `Instance_DeviceChanged(DBT_DEVICEARRIVAL)`: nothing once `flashdone`; else every serial
    /// port probed for a bootloader on threads of their own (`flow::probe_arrival`), what they
    /// find heard by [`InstallFirmware::tick`].
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:134-180`
    fn device_arrived(&mut self, ports: Vec<String>) {
        self.arrivals += 1;
        let ports = ports
            .iter()
            .map(|entry| identity_port(entry).to_owned())
            .collect();
        self.probe(ports, |port, baud| open_serial(port, baud, false));
    }

    /// [`InstallFirmware::device_arrived`] over the ports `open` opens: this machine's, or a
    /// test's.
    fn probe<P, O>(&mut self, ports: Vec<String>, open: O)
    where
        P: mp_firmware::detect::ProbePort,
        O: Fn(&str, u32) -> std::io::Result<P> + Send + Sync + 'static,
    {
        if self.flashdone.load(Ordering::Relaxed) {
            return;
        }
        // A flow of an earlier activation still running - a flash the page was left during - is
        // not the page's to probe around.
        if self.worker.is_none() && self.flows.load(Ordering::SeqCst) > 0 {
            return;
        }
        let found = self.found_sender.clone();
        // `Task.Run(() => Parallel.ForEach(SerialPort.GetPortNames(), ...))`.
        let _ = wasm_thread::Builder::new()
            .name("mp-firmware-arrival".to_owned())
            .spawn(move || {
                flow::probe_arrival(&ports, open, |board| {
                    let _ = found.send(board);
                });
            });
    }

    /// A probe's find: `lbl_status` says it - the bar left where it is - and it is
    /// `detectedport` and `detectedboardid`, which the next `LookForPort` takes.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:163-171`
    fn found_board(&mut self, board: FoundBoard) {
        self.progress.status = board.status();
        self.found = Some(board);
    }

    /// `detectedboardid`, as `LookForPort` reads it.
    fn detected_board_id(&self) -> Option<i64> {
        self.found
            .as_ref()
            .map(|found| i64::from(i32::from_le_bytes(found.board.board_id.to_le_bytes())))
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
        let started = wasm_thread::Builder::new()
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

    /// Whether Bootloader Update refuses the port the port box names: as Force Bootloader does
    /// ([`ForceBootloader::refuses_port`]) - a serial port, where a board is, while
    /// [`DEVICE_ENV`] names the device, so no script rewrites whatever is plugged into the
    /// machine running it.
    #[must_use]
    pub fn refuses_port(&self, port: &str) -> bool {
        self.force.refuses_port(port)
    }

    /// Whether the page takes a click: open on the manifest page, with no box over it.
    #[must_use]
    pub fn live(&self) -> bool {
        self.open
            && !self.connected
            && self.confirm.is_none()
            && self.selection.is_none()
            && self.path.is_none()
            && self.messages.is_empty()
            // The C#'s handlers hold the UI thread until the flow has ended: the flows, Force
            // Bootloader's and Bootloader Update's links, and the transport's questions. Force
            // Bootloader's box is a box over the page.
            && self.worker.is_none()
            && !self.force.busy()
            && self.bl.is_none()
            && !self.bl_asking
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

    /// `LookForPort(mavtype)` over the devices as they are now, with the board id a probe found
    /// when a device arrived (`devid = detectedboardid`, before the USB guess): no device with a
    /// board id says "Failed to detect port"; one file is downloaded; several open
    /// `FirmwareSelection`; none says "No firmware available" and then fails to download.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:193-358`
    pub fn pick(&mut self, index: usize, settings: &Persisted) {
        let (Some(manifest), Some(picture)) = (self.manifest.clone(), PICTURES.get(index)) else {
            return;
        };
        self.devices = devices();
        let lookup = manifest.look_for_port(
            &self.devices,
            self.detected_board_id(),
            picture.mav_type,
            self.release,
            false,
        );
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
        let lookup = manifest.look_for_port(
            &self.devices,
            self.detected_board_id(),
            MavType::Copter,
            self.release,
            true,
        );
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

    /// A picker's box in `FirmwareSelection` clicked: its list dropped down, or put away.
    pub fn toggle_picker(&mut self, id: PickerId) {
        if let Some(choosing) = &mut self.selection {
            choosing.toggle(id);
        }
    }

    /// A row of a dropped list chosen: the picker's `SelectedIndex`, and - for a filter - its
    /// `SelectedIndexChanged`.
    pub fn choose(&mut self, id: PickerId, row: usize) {
        if let Some(choosing) = &mut self.selection {
            choosing.choose(id, row);
        }
    }

    /// The wheel over a dropped list.
    pub fn scroll_picker(&mut self, lines: i32) {
        if let Some(choosing) = &mut self.selection {
            choosing.scroll(lines);
        }
    }

    /// "Upload Firmware": the Firmwares picker's selection is `FinalResult`, and the download
    /// goes on - with a URL, or with the line it holds in place of URLs, which fails as a
    /// download; with nothing selected the button does nothing.
    /// `// C#: test/FirmwareSelection.xaml.cs:200-212; ConfigFirmwareManifest.cs:247-254`
    pub fn upload_selected(&mut self, settings: &Persisted) {
        let Some(choosing) = &self.selection else {
            return;
        };
        let Some(url) = choosing.pickers.final_result().map(str::to_owned) else {
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
    /// `nothing` is set. `flashdone` is set once the file is in, before the upload.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:319-337`
    fn download(&mut self, url: String, device: String, nothing: bool, settings: &Persisted) {
        let machine = Machine::here(settings);
        let flashdone = Arc::clone(&self.flashdone);
        let running = Running::new(&self.flows);
        self.worker = Worker::start("mp-firmware-download", move |dialogue| {
            let _running = running;
            machine.run(dialogue, |cx, machine| {
                if nothing {
                    cx.dialogue.show(NO_FIRMWARE, flow::ERROR);
                }
                let reached = flow::download_and_flash(cx, &url, &device);
                if reached.file.is_some() {
                    flashdone.store(true, Ordering::Relaxed);
                }
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

    /// `ProcessCmdKey`: Ctrl+Q - Control and Q, nothing else held - shows `Strings.TrunkWarning`,
    /// and its OK takes the page to the `DEV` release and runs `Activate` again. The C# sees the
    /// key while a control of the page has the keyboard, and so it is here: the page takes the
    /// keyboard when it is clicked. Whether the key was the page's.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:399-408`
    pub fn key(&mut self, event: &KeyDownEvent) -> bool {
        let keystroke = &event.keystroke;
        let held = keystroke.modifiers;
        let ctrl_q = keystroke.key.eq_ignore_ascii_case("q")
            && held.control
            && !held.alt
            && !held.shift
            && !held.platform
            && !held.function;
        if !ctrl_q || !self.live() {
            return false;
        }
        self.trunk = true;
        self.messages.push_back(Waiting {
            text: TRUNK_WARNING.to_owned(),
            caption: TRUNK.to_owned(),
            buttons: None,
        });
        true
    }

    /// Bootloader Update on the manifest page, once `doConnect` has what it opens: the link from
    /// the port box's `url`, waiting for `Open`'s heartbeats. `None` - `doConnect` opening
    /// nothing, or a link that will not open - is "Failed to find device on mavlink" at once.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:537-546`
    pub fn bl_open(&mut self, url: Option<&str>, now: Instant) {
        if !self.open || self.connected {
            return;
        }
        match url.and_then(|url| bl_link(url).map(|link| (link, url))) {
            Some((link, _)) => {
                self.bl = Some(BlLink {
                    link,
                    stage: BlStage::Opening {
                        deadline: now + CONNECT_TIMEOUT,
                    },
                });
            }
            None => self.status = Some(NO_DEVICE_ON_MAVLINK.to_owned()),
        }
    }

    /// Bootloader Update's next step, once a frame: `Open` done is the first "BL Update"
    /// question; nothing heard in `CONNECT_TIMEOUT_SECONDS`, or the link gone, is "Failed to
    /// find device on mavlink" and the link closed; the command's answer said on the status
    /// line, and the link closed - `mav.Close()`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:542-561`
    pub fn bl_tick(&mut self, now: Instant) {
        let Some(bl) = &mut self.bl else {
            return;
        };
        match bl.stage {
            BlStage::Opening { deadline } => {
                let view = bl.link.view();
                if heard_enough(&view) {
                    bl.stage = BlStage::Asking(0);
                } else if now >= deadline || bl.link.error().is_some() || !view.connected {
                    self.bl = None;
                    self.status = Some(NO_DEVICE_ON_MAVLINK.to_owned());
                }
            }
            BlStage::Asking(_) => {}
            BlStage::Commanding => {
                if let Some(said) = bl.link.take_reports().into_iter().next() {
                    self.status = Some(said);
                    self.bl = None;
                } else if !bl.link.view().connected {
                    // The port gone under `doCommand`: its read throws, which the handler does
                    // not catch; the window's own words for a failed link - said even when the
                    // link has no error to give, as a link made over a transport has not.
                    self.status = Some(bl.link.error().map_or_else(
                        || "link failed: the link closed".to_owned(),
                        |err| format!("link failed: {err}"),
                    ));
                    self.bl = None;
                }
            }
        }
    }

    /// Whether the window's connection prompts were asking for Bootloader Update's link, once:
    /// what their last answer, or their Cancel, is for.
    pub fn take_bl_asking(&mut self) -> bool {
        std::mem::take(&mut self.bl_asking)
    }

    /// What the page has to say on the window's status line, once: its own, then its Force
    /// Bootloader's.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take().or_else(|| self.force.take_status())
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
        let running = Running::new(&self.flows);
        self.worker = Worker::start("mp-firmware-custom", move |dialogue| {
            let _running = running;
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
    /// bootloader from the one its firmware carries. On the connected page the window sends it,
    /// owning the link, once it sees [`InstallFirmware::take_bootloader_command`]; on the manifest
    /// page it goes over Bootloader Update's own link, to the vehicle its `Open` chose, and a No
    /// closes that link.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.cs:26-44; ConfigFirmwareManifest.cs:548-561`
    pub fn answer_bootloader(&mut self, yes: bool) {
        if let Some(bl) = &mut self.bl
            && let BlStage::Asking(at) = bl.stage
        {
            let vehicle = bl.link.view().vehicle;
            match (yes, at, vehicle) {
                (true, 0, _) => bl.stage = BlStage::Asking(1),
                (true, _, Some(id)) => {
                    // `doCommand`'s `TimeoutException` is not caught here, as the connected
                    // page catches it: its words go to the status line. The link is closed
                    // after it too, where the C#'s exception skips `mav?.Close()` and leaves the
                    // port open until the object is collected - not a behaviour to keep.
                    let report = Report {
                        accepted: Some(UPGRADED_BOOTLOADER.to_owned()),
                        refused: Some(FAILED_TO_UPGRADE_BOOTLOADER.to_owned()),
                        timed_out: Some(DO_COMMAND_TIMEOUT.to_owned()),
                        fallback: None,
                    };
                    bl.link.command(
                        id,
                        mp_link::requests::CMD_FLASH_BOOTLOADER,
                        [0.0, 0.0, 0.0, 0.0, 290_876.0, 0.0, 0.0],
                        report,
                    );
                    bl.stage = BlStage::Commanding;
                }
                // No, or no vehicle to send it to: `mav?.Close()`.
                _ => self.bl = None,
            }
            return;
        }
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

    /// The page's own message box dismissed; Ctrl+Q's goes on to `REL_Type = DEV` and
    /// `Activate()`.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:403-405`
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
        if self.trunk && self.messages.is_empty() {
            self.trunk = false;
            self.release = ReleaseType::Dev;
            self.activate();
        }
    }

    /// The Bootloader Update question showing: the connected page's, or the manifest page's.
    fn bl_question(&self) -> Option<&'static str> {
        let at = self.bootloader.or_else(|| {
            self.bl.as_ref().and_then(|bl| match bl.stage {
                BlStage::Asking(at) => Some(at),
                _ => None,
            })
        })?;
        BL_QUESTIONS.get(at).copied()
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
            .or_else(|| self.force.message())
    }

    /// The question showing: the page's own confirmation or Bootloader Update's, or the flow's.
    fn question(&self) -> Option<String> {
        self.confirm
            .map(|index| self.confirm_text(index))
            .or_else(|| self.bl_question().map(str::to_owned))
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

/// This machine's serial ports as the arrival watcher compares them: each port's name, a tab,
/// then what its USB device says of itself - ids, serial number, product - so a device coming
/// back under a name another had is an arrival too: after Force Bootloader the board's bootloader
/// ("CubeOrange-BL") takes the COM number the board ("CubeOrange") had, perhaps within one poll,
/// where `WM_DEVICECHANGE` tells the C# every time.
fn port_identities() -> Vec<String> {
    mp_transport::list_ports()
        .into_iter()
        .map(|port| {
            format!(
                "{}\t{:?}",
                port.name,
                (port.vid, port.pid, port.serial_number, port.product)
            )
        })
        .collect()
}

/// The port's name in one of [`port_identities`]'s entries.
fn identity_port(entry: &str) -> &str {
    entry.split('\t').next().unwrap_or(entry)
}

/// How many heartbeats the vehicle shown has sent.
pub(super) fn heartbeats(view: &TelemetryView) -> u64 {
    view.state.as_deref().map_or(0, |state| state.heartbeats)
}

/// The link `doConnect(mav, port, baud, false)` opens for Bootloader Update: recorded, as
/// `doConnect` opens a `.tlog` for every link it connects; asking for no streams and sending no
/// heartbeat, as nothing in the C# reads or announces a `MAVLinkInterface` that is not
/// `MainV2.comPort` - `doCommand` reads its own answer. `None` when it will not open.
/// `// C#: MainV2.cs:1593-1638`
fn bl_link(url: &str) -> Option<Telemetry> {
    let config = mp_link::LinkConfig {
        record_path: Telemetry::recording_path(),
        send_heartbeat: false,
        stream_rate_hz: 0,
        ..mp_link::LinkConfig::default()
    };
    mp_link::Link::connect(url, config)
        .ok()
        .map(|link| Telemetry::over(link, url))
}

/// What the Install Firmware pages say of a flow's stop: the px4 upload stops before the board
/// only while [`DEVICE_ENV`] names the device ([`NO_BOARD_WRITTEN`]); the other stops are the
/// steps that are not ported.
#[must_use]
pub fn stop_text(stop: &flow::Stop) -> String {
    match stop {
        flow::Stop::RebootToBootloader => stop.text_because(NO_BOARD_WRITTEN),
        _ => stop.text(),
    }
}

/// What a page says of its flow, for a script: `<prefix>.flow`, the board, the download, the file
/// read and where the flow stopped - a stop said by `say`.
pub fn record_reached_saying(
    prefix: &str,
    reached: Option<&Reached>,
    running: bool,
    say: fn(&flow::Stop) -> String,
) {
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
        text(reached.and_then(|r| r.stop.as_ref()).map(say)),
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
            .and_then(|choosing| choosing.pickers.final_result())
            .unwrap_or("none"),
    );
    // Each filter picker: shown or not, how many items its list holds, and its `SelectedItem`.
    for filter in Filter::LAYOUT {
        let picker = page
            .selection
            .as_ref()
            .map(|choosing| choosing.pickers.picker(filter));
        let key = format!("config.firmware.selection.{}", filter.name());
        record(
            format!("{key}.visible"),
            picker.is_some_and(|picker| picker.visible),
        );
        record(
            format!("{key}.items"),
            picker.map_or(0, |picker| picker.items.len()),
        );
        record(
            key,
            picker
                .and_then(|picker| picker.selected.as_deref())
                .unwrap_or("none"),
        );
    }
    record(
        "config.firmware.selection.open",
        page.selection
            .as_ref()
            .and_then(|choosing| choosing.open)
            .map_or_else(|| "none".to_owned(), |(id, _)| id.id()),
    );
    record(
        "config.firmware.detected",
        page.detected_board_id()
            .map_or_else(|| "none".to_owned(), |id| id.to_string()),
    );
    record("config.firmware.force", page.force.fact());
    record(
        "config.firmware.bl",
        match page.bl.as_ref().map(|bl| bl.stage) {
            None if page.bl_asking => "connecting",
            None => "none",
            Some(BlStage::Opening { .. }) => "opening",
            Some(BlStage::Asking(_)) => "asking",
            Some(BlStage::Commanding) => "commanding",
        },
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
    record_reached_saying(
        "config.firmware",
        page.reached.as_ref(),
        page.worker.is_some(),
        stop_text,
    );
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

/// The page, while it is open. The manifest page takes the keyboard when it is clicked, so its
/// `ProcessCmdKey`'s Ctrl+Q reaches it (`focus`).
pub fn page(
    firmware: &InstallFirmware,
    focus: &FocusHandle,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
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
            "fw-px4bl" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, _window, _cx| this.force_bootloader_clicked(ForcePage::Manifest),
                cx,
            ),
            // `fw-bootloaderupdate`.
            _ => link_label(
                id,
                text,
                (x, y),
                live,
                |this, window, cx| this.manifest_bootloader_update(window, cx),
                cx,
            ),
        };
        body = body.child(element);
    }

    panel(
        "install firmware",
        div()
            .id("fw-page")
            .track_focus(focus)
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                if this.install_firmware.key(event) {
                    cx.stop_propagation();
                    cx.notify();
                }
            }))
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(|this, _event, window, cx| {
                    window.focus(&this.firmware_page_focus, cx);
                }),
            )
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

/// Where a flow got, as lines of the report: a stop said as [`flow::Stop::text`] says it.
pub fn reached_lines(reached: &Reached) -> Vec<Div> {
    reached_lines_saying(reached, flow::Stop::text)
}

/// [`reached_lines`], a stop said by `say`.
fn reached_lines_saying(reached: &Reached, say: fn(&flow::Stop) -> String) -> Vec<Div> {
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
        lines.push(line("Stopped", say(stop), theme::WARN));
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

/// Beneath the page: the board, what a click chose, and where the flow got.
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
        column = column.children(reached_lines_saying(reached, stop_text));
    }
    column
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
                .child(UPDATE_THE_BOOTLOADER),
        );
    let report = div().flex().flex_col().gap_1().children(
        firmware
            .reached
            .iter()
            .flat_map(|reached| reached_lines_saying(reached, stop_text)),
    );
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
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(path.caption),
        )
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
                .flex()
                .items_center()
                .justify_between()
                .gap_2()
                .child(
                    div()
                        .min_w(px(0.0))
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child(path.filter),
                )
                // In a page, a file from the computer too: the browser's picker (page_files.rs),
                // its file put in the folder the box shows.
                .children((path.caption == OPEN_FILE).then(|| {
                    crate::page_files::browse_button(
                        format!("{}-browse", ids.path),
                        crate::page_files::accept_filter(path.filter),
                        crate::page_files::folder_of(
                            path.field.value(),
                            &mp_settings::data_directory().unwrap_or_default(),
                        ),
                        handle,
                    )
                }).flatten()),
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

/// A `Label` of `FirmwareSelection`'s stack.
fn selection_label(text: &'static str) -> Div {
    div().text_xs().text_color(rgb(theme::TEXT)).child(text)
}

/// A `Picker` of `FirmwareSelection`'s stack, as the WinForms renderer draws one - a drop-down
/// list box (`ComboBoxStyle.DropDownList`) showing the selected item, blank with none - its
/// `Margin="10"` and the stack's width; clicked, its list drops down over everything (priority 3,
/// above the dialog's 2), thirty rows showing and the wheel moving them, a row's click its
/// `SelectedIndex`.
/// `// C#: test/FirmwareSelection.xaml:9-30; ExtLibs/Xamarin.Forms.Platform.WinForms/Renderers/PickerRenderer.cs:14-107`
fn picker_box(choosing: &Choosing, id: PickerId, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let picker = choosing.picker(id);
    let name = id.id();
    let open = choosing.open.filter(|(open, _)| *open == id);
    let mut cell = crate::probe::measured(name.clone(), div())
        .id(SharedString::from(name.clone()))
        .relative()
        .m(px(10.0))
        .h(px(21.0))
        .px_1()
        .flex()
        .items_center()
        .justify_between()
        .rounded_sm()
        .border_1()
        .border_color(rgb(if open.is_some() {
            theme::ACCENT
        } else {
            theme::BORDER
        }))
        .bg(rgb(theme::BG))
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .hover(|style| style.border_color(rgb(theme::ACCENT)))
        .child(
            div()
                .whitespace_nowrap()
                .overflow_hidden()
                .child(picker.shown().unwrap_or("").to_owned()),
        )
        .child(div().text_size(px(7.0)).child("▼"))
        .on_click(cx.listener(move |this, _event, _window, cx| {
            this.install_firmware.toggle_picker(id);
            cx.notify();
        }));
    if let Some((_, top)) = open {
        let mut list = div()
            .id(SharedString::from(format!("{name}-rows")))
            .mt(px(22.0))
            .min_w(px(510.0))
            .flex()
            .flex_col()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::ACCENT))
            .occlude()
            .on_scroll_wheel(
                cx.listener(|this, event: &gpui::ScrollWheelEvent, _window, cx| {
                    // Lines or pixels, by backend: taken as rows either way, a negative y the
                    // wheel rolled towards the user - down.
                    let delta = event.delta.pixel_delta(px(LIST_ROW));
                    let rows = (f32::from(delta.y) / LIST_ROW).round();
                    #[allow(clippy::cast_possible_truncation)] // a few rows a notch
                    let rows = -(rows as i32);
                    if rows != 0 {
                        this.install_firmware.scroll_picker(rows);
                        cx.notify();
                    }
                    cx.stop_propagation();
                }),
            );
        for (row, item) in picker.items.iter().enumerate().skip(top).take(LIST_ROWS) {
            let row_id = format!("{name}-{row}");
            list = list.child(
                crate::probe::measured(row_id.clone(), div())
                    .id(SharedString::from(row_id))
                    .flex_shrink_0()
                    .h(px(LIST_ROW))
                    .px_1()
                    .whitespace_nowrap()
                    .bg(rgb(if picker.index == Some(row) {
                        theme::BORDER
                    } else {
                        theme::PANEL
                    }))
                    .text_color(rgb(theme::TEXT))
                    .hover(|style| style.bg(rgb(theme::ACTION)))
                    .child(item.clone())
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.install_firmware.choose(id, row);
                        cx.notify();
                    })),
            );
        }
        cell = cell.child(
            gpui::deferred(
                gpui::anchored()
                    .snap_to_window()
                    .child(crate::probe::measured(format!("{name}-list"), div()).child(list)),
            )
            .with_priority(3),
        );
    }
    cell.into_any_element()
}

/// `FirmwareSelection` over the page, as its `.xaml` stacks it in the 550-wide window
/// `ShowXamarinControl` makes: the heading, each picker left showing under its label, the
/// Firmwares label and picker, and Upload Firmware - with Close for the window's close box, which
/// is `FinalResult` left null, "user canceled".
/// `// C#: test/FirmwareSelection.xaml:6-32; GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:247-254; Utilities/ExtensionsMP.cs:82-104`
fn selection_box(
    choosing: &Choosing,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mut stack = div().flex().flex_col().gap(px(6.0)).child(
        div()
            .text_sm()
            .text_color(rgb(theme::TEXT))
            .child(MORE_THAN_ONE),
    );
    for filter in choosing.pickers.visible() {
        stack = stack
            .child(selection_label(filter.label()))
            .child(picker_box(choosing, PickerId::Filter(filter), cx));
    }
    stack =
        stack
            .child(selection_label(PICK_A_FILE))
            .child(picker_box(choosing, PickerId::Result, cx));
    let buttons = vec![
        action(
            "fw-select-upload",
            UPLOAD_FIRMWARE,
            theme::ACCENT,
            true,
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
        .child(stack)
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
    // Force Bootloader's instruction.
    if let Some(waiting) = firmware.force.message() {
        return Some(message_box(
            IDS,
            waiting,
            window,
            |this| this.install_firmware.force.dismiss(),
            cx,
        ));
    }
    if let Some(text) = firmware.bl_question() {
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

impl MissionPlanner {
    /// `Lbl_bootloaderupdate_Click`: `doConnect(mav, CMB_serialport.Text, CMB_baudrate.Text,
    /// false)` - for a network kind the transport's `Open` asks its questions first, through the
    /// window's prompts, as the window's CONNECT asks them - then the page's steps
    /// ([`InstallFirmware::bl_open`]). AUTO opens nothing here: `doConnect` starts the serial
    /// scan, which is not ported and would connect the window's link, never `mav`.
    ///
    /// A divergence: of what `doConnect` does to the window as it opens `mav` - the port boxes
    /// dimmed and the CONNECT button's image (both put back by `UpdateConnectIcon` within half a
    /// second, the window's own `comPort` being closed), `{port}_BAUD` saved, the title bar
    /// naming `mav`'s vehicle, the new-firmware check, and SETUP shown again, at once, as the
    /// handler runs on the UI thread (`BeginInvokeIfRequired`) - none is done here. Showing SETUP
    /// again deactivates this page, which here closes the link the page holds before its
    /// questions are asked, where the C#'s handler runs on past its disposed page; the rest
    /// describe a connection of the window's that there is not.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:535-546; MainV2.cs:1450-1847, 2466-2507; ExtLibs/Controls/ControlHelpers.cs:101-111`
    pub(crate) fn manifest_bootloader_update(
        &mut self,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        if !self.install_firmware.live() {
            return;
        }
        let port = self.connect_box.port.clone();
        let kind = crate::connect::kind(&port);
        if self.install_firmware.refuses_port(&port) {
            self.file_status = Some(NO_BOARD_WRITTEN.to_owned());
            return;
        }
        let questions = crate::connect::questions(kind);
        let Some(first) = questions.first().cloned() else {
            let url = (!port.is_empty())
                .then(|| crate::connect::url(kind, &port, &self.connect_box.baud, &[]))
                .flatten();
            self.install_firmware
                .bl_open(url.as_deref(), Instant::now());
            return;
        };
        self.install_firmware.bl_asking = true;
        self.connect_box.asking = Some(crate::connect::Asking {
            kind,
            questions,
            answers: Vec::new(),
        });
        self.connect_field
            .set(self.persisted.get(first.key).unwrap_or(first.default));
        self.connect_focus.focus(window, cx);
    }

    /// Once a frame, after the page's tick: Force Bootloader's steps over the window's link -
    /// either Install Firmware page's ([`MissionPlanner::force_bootloader_tick`]) - and Bootloader
    /// Update's over its own, and what the two pages have to say on the status line.
    pub(crate) fn install_firmware_links(&mut self) {
        let now = Instant::now();
        self.force_bootloader_tick(now);
        self.install_firmware.bl_tick(now);
        let said = [
            self.install_firmware.take_status(),
            self.firmware_legacy.take_status(),
        ];
        for status in said.into_iter().flatten() {
            self.file_status = Some(status);
        }
    }
}

#[cfg(test)]
mod tests {
    use mp_os::RecvTimeout as _;
    use mp_os::Lock as _;
    use super::*;

    fn fixture() -> Manifest {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/firmware/manifest.json.gz");
        let bytes = mp_os::fs::read(&path).expect("the fixture");
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
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
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
        assert_eq!(
            choosing.pickers.picker(Filter::Platform).shown(),
            Some("CubeOrange")
        );
        assert_eq!(
            choosing.results(),
            ["https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj"]
        );
        assert_eq!(
            choosing.pickers.final_result(),
            Some("https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj")
        );
        // One release and one version among the three: only the Platform picker shows.
        let shown: Vec<Filter> = choosing.pickers.visible().collect();
        assert_eq!(shown, [Filter::Platform]);
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
        assert!(
            page.reached.is_none(),
            "no stop: the command itself is owed"
        );
        assert!(page.take_bootloader_command());
        assert!(!page.take_bootloader_command(), "owed once");
        // The connected page's handler does nothing on the manifest page, whose Bootloader
        // Update opens a link of its own.
        let mut manifest = InstallFirmware {
            manifest: Some(Arc::new(fixture())),
            ..InstallFirmware::default()
        };
        manifest.open(false);
        manifest.bootloader_update(true);
        assert!(manifest.question().is_none());
    }

    fn key(key: &str, control: bool, shift: bool) -> KeyDownEvent {
        let mut event = KeyDownEvent {
            keystroke: gpui::Keystroke {
                modifiers: gpui::Modifiers::default(),
                key: key.to_owned(),
                key_char: Some(key.to_owned()),
            },
            is_held: false,
            prefer_character_input: false,
        };
        event.keystroke.modifiers.control = control;
        event.keystroke.modifiers.shift = shift;
        event
    }

    /// Ctrl+Q: "These are the latest trunk firmware ...", caption "trunk"; its OK is the `DEV`
    /// release and `Activate` again. Q alone, or with Shift as well, is not it; nor is it heard
    /// while a box is up. Deactivate goes back to OFFICIAL.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:399-408, 128-129`
    #[test]
    fn ctrl_q_warns_and_its_ok_offers_the_dev_release() {
        let mut page = loaded("CubeOrange-BL");
        assert!(!page.key(&key("q", false, false)), "Q alone");
        assert!(!page.key(&key("q", true, true)), "Ctrl+Shift+Q");
        assert!(!page.key(&key("s", true, false)), "Ctrl+S");
        assert!(page.message().is_none());
        assert!(page.key(&key("q", true, false)));
        assert_eq!(
            page.message()
                .map(|waiting| (waiting.text.as_str(), waiting.caption.as_str())),
            Some((TRUNK_WARNING, TRUNK))
        );
        assert_eq!(page.release, ReleaseType::Official, "not until its OK");
        assert!(
            !page.key(&key("q", true, false)),
            "the box has the keyboard"
        );
        page.dismiss_message();
        assert!(page.message().is_none());
        assert_eq!(page.release, ReleaseType::Dev);
        assert_eq!(page.labels[index_of("fw-quad")], "Copter V4.8.0-dev DEV");
        assert_eq!(
            page.labels[index_of("fw-tracker")],
            "AntennaTracker V4.8.0-dev DEV"
        );
        // Another box's OK is only that.
        page.messages.push_back(Waiting {
            text: NO_PORT.to_owned(),
            caption: flow::ERROR.to_owned(),
            buttons: None,
        });
        page.dismiss_message();
        assert_eq!(page.release, ReleaseType::Dev);
        page.close();
        assert_eq!(page.release, ReleaseType::Official);
    }

    /// All Options for a CubeOrange: its 80 files, on its own platform, and the pickers that
    /// still choose - Version Type and Version - shown. A release chosen narrows the list and
    /// refills Version; "Ignore" undoes it; a row of the Firmwares list is what Upload Firmware
    /// takes.
    /// `// C#: test/FirmwareSelection.xaml.cs:15-194`
    #[test]
    fn all_options_pickers_narrow_the_list_and_ignore_undoes_them() {
        let mut page = loaded("CubeOrange-BL");
        let manifest = page.manifest.clone().expect("the fixture");
        let lookup = manifest.look_for_port(
            &[DeviceInfo::new("CubeOrange-BL", "")],
            None,
            MavType::Copter,
            ReleaseType::Official,
            true,
        );
        page.after_lookup(lookup, &Persisted::at(None));
        let shown = |page: &InstallFirmware| {
            page.selection
                .as_ref()
                .map(|choosing| choosing.pickers.visible().collect::<Vec<_>>())
        };
        let results = |page: &InstallFirmware| {
            page.selection
                .as_ref()
                .map_or(0, |choosing| choosing.results().len())
        };
        let items = |page: &InstallFirmware, filter: Filter| {
            page.selection.as_ref().map_or_else(Vec::new, |choosing| {
                choosing.pickers.picker(filter).items.clone()
            })
        };
        assert_eq!(
            shown(&page),
            Some(vec![Filter::VersionType, Filter::Platform, Filter::Version])
        );
        assert_eq!(results(&page), 80);
        assert_eq!(
            items(&page, Filter::VersionType),
            ["BETA", "DEV", "OFFICIAL", "STABLE-4.7.1", "Ignore"]
        );
        assert_eq!(items(&page, Filter::Version), ["4.7.1", "4.8.0", "Ignore"]);

        let version_type = PickerId::Filter(Filter::VersionType);
        page.toggle_picker(version_type);
        assert_eq!(
            page.selection.as_ref().and_then(|choosing| choosing.open),
            Some((version_type, 0))
        );
        page.choose(version_type, 1);
        assert_eq!(
            page.selection.as_ref().and_then(|choosing| choosing.open),
            None,
            "a row's click puts the list away"
        );
        assert_eq!(results(&page), 28, "DEV");
        assert_eq!(items(&page, Filter::Version), ["4.8.0", "Ignore"]);
        page.choose(version_type, 4);
        assert_eq!(results(&page), 80, "Ignore");
        assert_eq!(items(&page, Filter::Version), ["4.7.1", "4.8.0", "Ignore"]);

        page.choose(PickerId::Result, 2);
        let chosen = page
            .selection
            .as_ref()
            .and_then(|choosing| choosing.pickers.final_result().map(str::to_owned));
        let third = page
            .selection
            .as_ref()
            .and_then(|choosing| choosing.results().get(2).cloned());
        assert!(chosen.is_some());
        assert_eq!(chosen, third);
    }

    /// A dropped list longer than thirty rows scrolls on the wheel, within the list.
    #[test]
    fn a_long_list_drops_down_thirty_rows_at_a_time_and_scrolls() {
        let mut page = loaded("CubeOrange-BL");
        let manifest = page.manifest.clone().expect("the fixture");
        let lookup = manifest.look_for_port(
            &[DeviceInfo::new("CubeOrange-BL", "")],
            None,
            MavType::Copter,
            ReleaseType::Official,
            true,
        );
        page.after_lookup(lookup, &Persisted::at(None));
        let result = PickerId::Result;
        let count = page
            .selection
            .as_ref()
            .map_or(0, |choosing| choosing.picker(result).items.len());
        assert_eq!(count, 80, "the CubeOrange files");
        assert!(count > LIST_ROWS);
        page.toggle_picker(result);
        page.scroll_picker(3);
        let top = |page: &InstallFirmware| {
            page.selection
                .as_ref()
                .and_then(|choosing| choosing.open)
                .map(|(_, top)| top)
        };
        assert_eq!(top(&page), Some(3));
        page.scroll_picker(1000);
        assert_eq!(top(&page), Some(count - LIST_ROWS));
        page.scroll_picker(-1000);
        assert_eq!(top(&page), Some(0));
        page.toggle_picker(result);
        assert_eq!(top(&page), None, "clicked again: put away");
    }

    /// `DeviceChanged` here: the ports enumerated on a thread of their own, the first list the
    /// ports there are at `Activate`; a port leaving is no arrival, a port appearing is one, sent
    /// with the list; dropped - `Deactivate` - the thread enumerates no more. The page hears an
    /// arrival while it is the manifest page, and not once it has closed.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:47-48, 126, 134-137`
    #[test]
    fn the_watcher_hears_a_port_appear_and_stops_with_the_page() {
        use std::sync::Mutex;
        use std::sync::atomic::AtomicUsize;
        let ports = |names: &[&str]| names.iter().map(|&n| n.to_owned()).collect::<Vec<_>>();
        let lists = Arc::new(Mutex::new(VecDeque::from([
            ports(&["/dev/ttyS0", "/dev/ttyACM0"]),
            ports(&["/dev/ttyS0", "/dev/ttyACM0"]),
            ports(&["/dev/ttyS0"]),
            ports(&["/dev/ttyS0", "/dev/ttyACM0"]),
        ])));
        let calls = Arc::new(AtomicUsize::new(0));
        let enumerate = {
            let (lists, calls) = (Arc::clone(&lists), Arc::clone(&calls));
            move || {
                calls.fetch_add(1, Ordering::SeqCst);
                lists
                    .os_lock()
                    .expect("the lists")
                    .pop_front()
                    .unwrap_or_else(|| ports(&["/dev/ttyS0", "/dev/ttyACM0"]))
            }
        };
        let mut page = loaded("FT232R USB UART");
        page.watcher = Watcher::start(enumerate, Duration::from_millis(5));
        let watcher = page.watcher.as_ref().expect("the thread");
        let arrival = watcher
            .arrivals
            .os_recv_timeout(Duration::from_secs(5))
            .expect("the port back is an arrival");
        assert_eq!(arrival, ["/dev/ttyS0", "/dev/ttyACM0"]);
        assert!(
            watcher
                .arrivals
                .os_recv_timeout(Duration::from_millis(100))
                .is_err(),
            "the same ports again, or one fewer, are none"
        );

        // The page takes the latest arrival, once, for the probe.
        lists.os_lock().expect("the lists").extend([
            ports(&["/dev/ttyS0"]),
            ports(&["/dev/ttyS0", "/dev/ttyUSB0"]),
        ]);
        crate::telemetry::scripted::until("the second arrival", || page.hear_arrival().is_some());
        assert!(page.hear_arrival().is_none(), "heard once");

        // Deactivate: the thread stops, and nothing more is heard.
        page.close();
        assert!(page.watcher.is_none());
        wasm_thread::sleep(Duration::from_millis(50));
        let after = calls.load(Ordering::SeqCst);
        wasm_thread::sleep(Duration::from_millis(50));
        assert_eq!(calls.load(Ordering::SeqCst), after, "no more enumerations");
        assert!(page.hear_arrival().is_none());
    }

    /// A device's arrival: a port that was not there before; probed only while `flashdone` is
    /// clear, every port on its own thread; what a probe finds is the status line and the board
    /// id `LookForPort` takes before the USB guess.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:134-180, 213-229`
    #[test]
    fn a_device_arriving_is_probed_unless_the_flash_is_done_and_its_board_is_taken() {
        use std::sync::atomic::AtomicUsize;
        let seen = ["/dev/ttyS0".to_owned()];
        assert!(!arrived(&seen, &["/dev/ttyS0".to_owned()]));
        assert!(arrived(
            &seen,
            &["/dev/ttyS0".to_owned(), "/dev/ttyACM0".to_owned()]
        ));
        assert!(!arrived(&seen, &[]), "a port leaving is not an arrival");
        // The same name back as another device - the board's bootloader after Force
        // Bootloader - is an arrival; the probe is given the port's name alone.
        let board = ["COM4\tCubeOrange".to_owned()];
        assert!(arrived(&board, &["COM4\tCubeOrange-BL".to_owned()]));
        assert_eq!(identity_port("COM4\tCubeOrange-BL"), "COM4");
        assert_eq!(identity_port("/dev/ttyACM0"), "/dev/ttyACM0");

        let mut page = loaded("FT232R USB UART");
        let opened = Arc::new(AtomicUsize::new(0));
        let ports = vec!["/dev/ttyS0".to_owned(), "/dev/ttyACM0".to_owned()];
        let counter = Arc::clone(&opened);
        let open = move |_port: &str, baud: u32| {
            assert_eq!(baud, 115_200);
            counter.fetch_add(1, Ordering::SeqCst);
            Err::<TransportPort<mp_transport::SerialTransport>, _>(std::io::Error::other(
                "not on this bench",
            ))
        };
        page.flashdone.store(true, Ordering::Relaxed);
        page.probe(ports.clone(), open.clone());
        wasm_thread::sleep(Duration::from_millis(100));
        assert_eq!(
            opened.load(Ordering::SeqCst),
            0,
            "flashdone: nothing opened"
        );
        page.flashdone.store(false, Ordering::Relaxed);
        page.probe(ports, open);
        crate::telemetry::scripted::until("both ports tried", || {
            opened.load(Ordering::SeqCst) == 2
        });

        // What a probe finds.
        let board = FoundBoard {
            port: "/dev/ttyACM0".to_owned(),
            board: mp_firmware::uploader::Board {
                bootloader_revision: 5,
                board_id: 140,
                board_revision: 0,
                flash_size: 2_080_768,
            },
            chip: mp_firmware::uploader::Chip::default(),
        };
        page.found_sender
            .send(board.clone())
            .expect("the page's channel");
        page.tick(true);
        assert_eq!(page.progress.status, board.status());
        assert_eq!(page.detected_board_id(), Some(140));
        // The FTDI cable names no board; the id the bootloader gave does.
        let manifest = page.manifest.clone().expect("the fixture");
        let devices = [DeviceInfo::new("FT232R USB UART", "")];
        assert!(
            manifest
                .look_for_port(
                    &devices,
                    None,
                    MavType::Copter,
                    ReleaseType::Official,
                    false
                )
                .is_none()
        );
        let lookup = manifest
            .look_for_port(
                &devices,
                page.detected_board_id(),
                MavType::Copter,
                ReleaseType::Official,
                false,
            )
            .expect("the detected board");
        assert_eq!(lookup.board_ids, [140]);
        assert_eq!(lookup.items.len(), 3);
        // Kept across Deactivate, forgotten with the SETUP screen.
        page.close();
        page.tick(true);
        assert_eq!(page.detected_board_id(), Some(140));
        page.tick(false);
        assert_eq!(page.detected_board_id(), None);

        // And forgotten when the SETUP screen is made anew while it shows - a connect or a
        // disconnect - so the next board's firmware is not chosen by this one's id.
        page.found_sender
            .send(board.clone())
            .expect("the page's channel");
        page.tick(true);
        assert_eq!(page.detected_board_id(), Some(140));
        page.close();
        page.screen_disposed();
        page.tick(true);
        assert_eq!(page.detected_board_id(), None);
    }

    /// A flash left running when the page was left keeps the ports: arrivals are not probed
    /// around it once the page is back, until its thread ends.
    #[test]
    fn a_flash_left_running_is_not_probed_around() {
        use std::sync::atomic::AtomicUsize;
        let mut page = loaded("FT232R USB UART");
        let opened = Arc::new(AtomicUsize::new(0));
        let counting = |opened: &Arc<AtomicUsize>| {
            let opened = Arc::clone(opened);
            move |_: &str, _: u32| {
                opened.fetch_add(1, Ordering::SeqCst);
                Err::<TransportPort<mp_transport::SerialTransport>, _>(std::io::Error::other(
                    "not on this bench",
                ))
            }
        };
        let left = Running::new(&page.flows);
        page.worker = None;
        page.probe(vec!["/dev/ttyACM0".to_owned()], counting(&opened));
        wasm_thread::sleep(Duration::from_millis(100));
        assert_eq!(
            opened.load(Ordering::SeqCst),
            0,
            "probed around a running flash"
        );
        drop(left);
        page.probe(vec!["/dev/ttyACM0".to_owned()], counting(&opened));
        let until = Instant::now() + Duration::from_secs(5);
        while opened.load(Ordering::SeqCst) == 0 {
            assert!(Instant::now() < until, "not probed once the flash ended");
            wasm_thread::sleep(Duration::from_millis(5));
        }
    }

    /// The path a device's arrival takes: `Activate` starts watching the ports, a port appearing
    /// is heard by the tick while the page is open, and every port is probed - here a port that
    /// is not there, so nothing is opened and nothing found. Leaving the page stops the watching.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:45-48, 126, 134-180`
    #[test]
    fn a_port_arriving_is_heard_and_probed() {
        static PLUGGED: AtomicBool = AtomicBool::new(false);
        static LOOKS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        fn ports() -> Vec<String> {
            LOOKS.fetch_add(1, Ordering::SeqCst);
            if PLUGGED.load(Ordering::SeqCst) {
                vec!["/dev/mp-test-no-such-port".to_owned()]
            } else {
                Vec::new()
            }
        }
        let mut page = loaded("CubeOrange-BL");
        page.watch_ports = Some(ports);
        page.activate();
        assert!(page.watcher.is_some(), "Activate watches the ports");
        // The port arrives after the watcher's first list of the ports there are: plugged before
        // its thread has looked, it is among them and no arrival - the race Windows' slower
        // thread start lost (CI run 37173996196).
        let until = Instant::now() + Duration::from_secs(5);
        while LOOKS.load(Ordering::SeqCst) == 0 {
            assert!(Instant::now() < until, "the watcher never looked at the ports");
            wasm_thread::sleep(Duration::from_millis(1));
        }
        PLUGGED.store(true, Ordering::SeqCst);
        let until = Instant::now() + Duration::from_secs(5);
        while page.arrivals == 0 {
            assert!(Instant::now() < until, "the arrival was never heard");
            page.tick(true);
            wasm_thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(page.arrivals, 1);
        assert_eq!(page.detected_board_id(), None, "nothing there to answer");
        page.close();
        assert!(page.watcher.is_none(), "Deactivate stops watching");
    }

    /// `flashdone` is set by `LookForPort`'s download, once the file is in; `Activate` clears
    /// it.
    #[test]
    fn activate_clears_flashdone() {
        let mut page = loaded("CubeOrange-BL");
        page.flashdone.store(true, Ordering::Relaxed);
        page.activate();
        assert!(!page.flashdone.load(Ordering::Relaxed));
    }

    /// Bootloader Update on the manifest page with no link to open - AUTO, a cancelled
    /// question - is "Failed to find device on mavlink" on the status line, not a box.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:542-546`
    #[test]
    fn bootloader_update_with_nothing_to_open_finds_no_device() {
        let mut page = loaded("CubeOrange-BL");
        page.bl_open(None, Instant::now());
        assert_eq!(page.take_status().as_deref(), Some(NO_DEVICE_ON_MAVLINK));
        assert!(page.bl.is_none());
        assert!(page.message().is_none(), "no box");
        assert!(page.take_status().is_none(), "said once");
    }

    /// While MP_FIRMWARE_DEVICE names the device, Force Bootloader and Bootloader Update open no
    /// serial port - where a real board is, the bench CubeOrange among them - and a network kind,
    /// the SITL's, is still theirs; with the machine's own devices, every port is.
    #[test]
    fn a_named_device_keeps_both_off_the_machines_serial_ports() {
        let mut page = loaded("CubeOrange-BL");
        page.force.opens_boards = false;
        assert!(page.refuses_port("/dev/ttyACM0"));
        assert!(page.refuses_port("COM4"));
        assert!(!page.refuses_port("TCP"));
        assert!(!page.refuses_port("UDP"));
        assert!(
            !page.refuses_port(""),
            "nothing to open is the C#'s own no-device"
        );
        page.force.opens_boards = true;
        assert!(!page.refuses_port("/dev/ttyACM0"));
    }

    /// What the page says of a stop: the px4 upload stops before the board only while
    /// MP_FIRMWARE_DEVICE names the device, and says so; every other stop is a step that is not
    /// ported, and says that. Nothing on this page says flashing is not enabled: this build
    /// flashes (the bench CubeOrange, 2026-09-25).
    #[test]
    fn a_stop_says_why_its_step_was_not_taken() {
        assert_eq!(
            stop_text(&flow::Stop::RebootToBootloader),
            "the next step would reboot the board into its bootloader and upload the image: no \
             board is written while MP_FIRMWARE_DEVICE names the device"
        );
        assert_eq!(
            stop_text(&flow::Stop::Dfu),
            "the next step would flash the image through DFU: not ported to this application"
        );
        for stop in [
            flow::Stop::BootloaderProbe,
            flow::Stop::OpenPort("/dev/ttyUSB0".to_owned()),
            flow::Stop::RebootToBootloader,
            flow::Stop::ArduinoUpload("/dev/ttyUSB0".to_owned()),
            flow::Stop::Vrbrain,
            flow::Stop::Parrot,
            flow::Stop::Solo,
            flow::Stop::Dfu,
            flow::Stop::FlashBootloader,
        ] {
            for said in [stop_text(&stop), stop.text()] {
                assert!(!said.contains("not enabled"), "{said}");
            }
        }
        assert_eq!(NOT_PORTED, "not ported to this application");
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
        let deadline = web_time::Instant::now() + std::time::Duration::from_secs(20);
        while page.receiver.is_some() && web_time::Instant::now() < deadline {
            wasm_thread::sleep(std::time::Duration::from_millis(10));
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
        let web = mp_os::temp_dir().join(format!("mp-gui-fw-flow-{}", mp_os::process_id()));
        let _ = mp_os::fs::remove_dir_all(&web);
        let served = web.join("web/firmware.ardupilot.org/Copter/stable/CubeOrange");
        mp_os::fs::create_dir_all(&served).unwrap();
        let apj = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../mp-firmware/testdata/legacy/arducopter.apj");
        mp_os::fs::copy(&apj, served.join("arducopter.apj")).unwrap();
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
        let deadline = web_time::Instant::now() + std::time::Duration::from_secs(10);
        let mut done = None;
        while done.is_none() && web_time::Instant::now() < deadline {
            done = worker.poll(&mut progress);
            if let Some(waiting) = worker.waiting.clone() {
                assert_eq!(waiting.text, "Go?");
                assert_eq!(waiting.buttons, Some(Buttons::YesNo));
                worker.answer(true);
            }
            wasm_thread::sleep(std::time::Duration::from_millis(2));
        }
        let reached = done.expect("the flow ended");
        assert_eq!(reached.firmware.map(|f| f.board_id), Some(140));
        assert_eq!(reached.stop, Some(flow::Stop::RebootToBootloader));
        assert_eq!(progress.status, "Reading Hex File");
        assert_eq!(
            progress.value, 100,
            "-1 leaves the bar at the download's end"
        );
        let _ = mp_os::fs::remove_dir_all(&web);
    }

    /// The file dialog opens in the folder last used, takes only a file that exists, and saves
    /// the folder of the one it takes.
    #[test]
    fn load_custom_firmware_remembers_the_folder() {
        let dir = mp_os::temp_dir().join(format!("mp-gui-fw-custom-{}", mp_os::process_id()));
        mp_os::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("custom.apj");
        mp_os::fs::write(&file, "{}").unwrap();
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
        let _ = mp_os::fs::remove_dir_all(&dir);
    }
}

/// The firmware page's reboot into the bootloader, over the real link to a scripted copter.
#[cfg(test)]
mod reboot_to_bootloader_tests {
    use web_time::{Duration, Instant};

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
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2588-2615, 2714, 2755-2760`
    #[test]
    fn four_frames_3_3_1_1_go_out_after_the_next_heartbeat() {
        let (link, mut vehicle) = Vehicle::link(fast());
        let waits = RebootWaits {
            window: Duration::from_secs(5),
            heartbeat: Duration::from_secs(4),
        };
        let reached = wasm_thread::scope(|scope| {
            let task = scope.spawn(|| reboot_to_bootloader(&link, Instant::now(), waits));
            // `doReboot`'s own `getHeartBeat` holds the reboots until the next heartbeat.
            wasm_thread::sleep(Duration::from_millis(200));
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
        wasm_thread::sleep(fast().command.timeout * 3);
        vehicle.read();
        assert_eq!(reboots(&vehicle), FOUR);
    }

    /// No second heartbeat: `getHeartBeat` gives up after its 2.2 s, and the ids the first one
    /// set still send the four frames.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1199-1203, 2591-2611`
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

    /// A Cube with an ADS-B receiver: the receiver's heartbeat, component 0, heard first is
    /// passed over - no reboot goes to it, and none goes out on it alone - and the autopilot's
    /// next heartbeat after its first sends the four frames to the autopilot. The C# would have
    /// sent nothing when the receiver's came first (the owner's ruling of 2026-09-27).
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:823-828, 2588-2611`
    #[test]
    fn an_adsb_receivers_heartbeat_is_passed_over() {
        use mp_mavlink_dialects::all::Heartbeat;
        use mp_vehicle::VehicleId;
        let adsb = VehicleId::new(1, 0);
        let receiver = MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 27,
            autopilot: 8,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 3,
        });
        let (link, mut vehicle) = Vehicle::link_silent(fast());
        let waits = RebootWaits {
            window: Duration::from_secs(5),
            heartbeat: Duration::from_secs(4),
        };
        let reached = wasm_thread::scope(|scope| {
            vehicle.send_from(adsb, &receiver);
            until("the receiver to be seen", || {
                link.vehicles().contains(&adsb)
            });
            let task = scope.spawn(|| reboot_to_bootloader(&link, Instant::now(), waits));
            wasm_thread::sleep(Duration::from_millis(200));
            vehicle.send_from(adsb, &receiver);
            wasm_thread::sleep(Duration::from_millis(200));
            vehicle.read();
            assert_eq!(reboots(&vehicle), [], "sent on the receiver's heartbeats");
            // The autopilot heard, then `doReboot`'s next heartbeat.
            vehicle.heartbeat();
            until("the autopilot to be seen", || {
                link.vehicles().contains(&VEHICLE)
            });
            wasm_thread::sleep(Duration::from_millis(100));
            vehicle.heartbeat();
            let reached = task.join().expect("the task");
            until("the four reboots", || {
                vehicle.read();
                reboots(&vehicle).len() >= 4
            });
            reached
        });
        assert_eq!(reached, LinkReboot::Rebooted);
        // `reboots` checks each is addressed to the autopilot, not the receiver.
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

/// Force Bootloader and the manifest page's Bootloader Update, over the real link to a scripted
/// copter: what goes on the wire, and what the page says.
#[cfg(test)]
mod manifest_link_tests {
    use web_time::{Duration, Instant};

    use mp_link::ProtocolTimeouts;
    use mp_link::requests::{CMD_FLASH_BOOTLOADER, CMD_PREFLIGHT_REBOOT_SHUTDOWN};
    use mp_mavlink_dialects::all::MavMessage;

    use super::{
        BL_QUESTIONS, BlLink, BlStage, DO_COMMAND_TIMEOUT, FAILED_TO_UPGRADE_BOOTLOADER,
        InstallFirmware, NO_DEVICE_ON_MAVLINK, UPGRADED_BOOTLOADER, heartbeats,
    };
    use crate::config::force_bootloader::{FORCE_FAILED, Force, ForceEnd, IGNORE_THE_UNPLUG};
    use crate::telemetry::Telemetry;
    use crate::telemetry::scripted::{VEHICLE, Vehicle, ack, until};

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// The manifest page, showing.
    fn page() -> InstallFirmware {
        InstallFirmware {
            open: true,
            ..InstallFirmware::default()
        }
    }

    /// Each `PREFLIGHT_REBOOT_SHUTDOWN` the copter heard: param1 and confirmation.
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

    /// `doReboot(true, false)`'s four frames.
    const FOUR: [(f32, u8); 4] = [(3.0, 0), (3.0, 0), (1.0, 0), (1.0, 0)];

    /// Ticks Force Bootloader until it ends, the copter's link read as it goes.
    fn force_until_end(
        page: &mut InstallFirmware,
        telemetry: &mut Telemetry,
        now: impl Fn() -> Instant,
    ) -> Option<ForceEnd> {
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            let view = telemetry.view();
            if let Some(end) = page.force.tick(telemetry, &view, now()) {
                return Some(end);
            }
            wasm_thread::sleep(Duration::from_millis(2));
        }
        None
    }

    /// The window's link already open: `Open` returns at once, and `doReboot(true, false)` holds
    /// its reboots until the copter's next heartbeat - then 3, 3, 1, 1, and the instruction's
    /// box.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:517-522; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2588-2615, 2755-2760`
    #[test]
    fn the_link_open_the_next_heartbeat_brings_the_four_reboots_and_the_box() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = page();
        let view = telemetry.view();
        let seen = heartbeats(&view);
        page.force.start(true, &view, Instant::now());
        assert!(matches!(page.force.state(), Some(Force::Heartbeat { .. })));
        assert!(!page.live(), "the handler holds the window");
        assert!(page.forcing(), "SETUP is not shown again under it");
        for _ in 0..20 {
            let view = telemetry.view();
            assert_eq!(page.force.tick(&mut telemetry, &view, Instant::now()), None);
        }
        wasm_thread::sleep(Duration::from_millis(100));
        vehicle.read();
        assert_eq!(reboots(&vehicle), [], "not before the next heartbeat");
        vehicle.heartbeat();
        until("the next heartbeat", || {
            heartbeats(&telemetry.view()) > seen
        });
        assert_eq!(
            force_until_end(&mut page, &mut telemetry, Instant::now),
            Some(ForceEnd::Rebooted)
        );
        until("the four reboots", || {
            vehicle.read();
            reboots(&vehicle).len() >= 4
        });
        wasm_thread::sleep(fast().command.timeout * 3);
        vehicle.read();
        assert_eq!(reboots(&vehicle), FOUR);
        assert_eq!(
            page.message().map(|waiting| waiting.text.as_str()),
            Some(IGNORE_THE_UNPLUG)
        );
        assert_eq!(page.take_status(), None, "no failure said");
        assert!(!page.forcing(), "done");
        assert!(!page.live(), "the box is over the page");
        page.force.dismiss();
        assert!(page.message().is_none());
        assert!(page.live(), "OK, and the page is the user's again");
    }

    /// The link opened by the click: `Open` waits for the copter's second heartbeat, then
    /// `doReboot` for the one after; without it, 2.2 s on, the reboots go out anyway.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:871-893, 1199-1203, 2591-2611`
    #[test]
    fn a_link_opened_waits_for_two_heartbeats_then_reboots_when_the_next_does_not_come() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = page();
        let started = Instant::now();
        let view = telemetry.view();
        page.force.start(false, &view, started);
        let view = telemetry.view();
        assert_eq!(page.force.tick(&mut telemetry, &view, started), None);
        assert!(
            matches!(page.force.state(), Some(Force::Opening { .. })),
            "one heartbeat is not enough"
        );
        vehicle.heartbeat();
        until("the second heartbeat", || {
            let view = telemetry.view();
            page.force.tick(&mut telemetry, &view, Instant::now());
            matches!(page.force.state(), Some(Force::Heartbeat { .. }))
        });
        // No third heartbeat: `getHeartBeat`'s 2.2 s pass.
        let later = Instant::now() + Duration::from_millis(2300);
        assert_eq!(
            force_until_end(&mut page, &mut telemetry, || later),
            Some(ForceEnd::Rebooted)
        );
        until("the four reboots", || {
            vehicle.read();
            reboots(&vehicle).len() >= 4
        });
        assert_eq!(reboots(&vehicle), FOUR);
    }

    /// Nothing heard before `CONNECT_TIMEOUT_SECONDS`: "Failed to connect and send the reboot
    /// command" on the status line - no box - and nothing sent.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:524-532`
    #[test]
    fn nothing_heard_in_time_is_failed_to_connect_on_the_status_line() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = page();
        let started = Instant::now();
        let view = telemetry.view();
        page.force.start(false, &view, started);
        let view = telemetry.view();
        assert_eq!(
            page.force
                .tick(&mut telemetry, &view, started + Duration::from_secs(31)),
            Some(ForceEnd::Failed)
        );
        assert_eq!(page.take_status().as_deref(), Some(FORCE_FAILED));
        assert!(page.message().is_none(), "no box");
        assert!(page.force.state().is_none());
        assert!(!page.forcing());
        wasm_thread::sleep(Duration::from_millis(100));
        vehicle.read();
        assert_eq!(reboots(&vehicle), []);
    }

    /// Bootloader Update's link, as `doConnect` left it.
    fn bl(telemetry: Telemetry) -> InstallFirmware {
        let mut page = page();
        page.bl = Some(BlLink {
            link: telemetry,
            stage: BlStage::Opening {
                deadline: Instant::now() + Duration::from_secs(30),
            },
        });
        page
    }

    /// Each `FLASH_BOOTLOADER` the copter heard: its param5, the magic number.
    fn flash_commands(vehicle: &Vehicle) -> Vec<f32> {
        vehicle
            .heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::CommandLong(long) if long.command == CMD_FLASH_BOOTLOADER => {
                    assert_eq!(
                        (long.target_system, long.target_component),
                        (VEHICLE.sysid, VEHICLE.compid)
                    );
                    Some(long.param5)
                }
                _ => None,
            })
            .collect()
    }

    /// Opened and heard twice: the first "BL Update" question, then the second, then
    /// `doCommand(FLASH_BOOTLOADER, 0, 0, 0, 0, 290876, 0, 0)`; the copter's answer on the status
    /// line - refused, "Failed to upgrade bootloader"; accepted, "Upgraded bootloader" - and the
    /// link closed.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:548-561`
    #[test]
    fn two_yeses_send_flash_bootloader_and_the_answer_is_said() {
        for (result, said) in [
            (4u8, FAILED_TO_UPGRADE_BOOTLOADER),
            (0, UPGRADED_BOOTLOADER),
        ] {
            let (telemetry, mut vehicle) = Vehicle::connect(fast());
            let mut page = bl(telemetry);
            page.bl_tick(Instant::now());
            assert!(
                page.question().is_none(),
                "one heartbeat is not `Open` done"
            );
            vehicle.heartbeat();
            until("the first question", || {
                page.bl_tick(Instant::now());
                page.question().is_some()
            });
            assert_eq!(page.question().as_deref(), Some(BL_QUESTIONS[0]));
            page.answer_bootloader(true);
            assert_eq!(page.question().as_deref(), Some(BL_QUESTIONS[1]));
            vehicle.read();
            assert!(
                flash_commands(&vehicle).is_empty(),
                "not before the second Yes"
            );
            page.answer_bootloader(true);
            assert!(page.question().is_none());
            until("the command", || {
                vehicle.read();
                !flash_commands(&vehicle).is_empty()
            });
            assert_eq!(flash_commands(&vehicle), [290_876.0]);
            vehicle.send(&ack(CMD_FLASH_BOOTLOADER, result));
            until("the answer", || {
                page.bl_tick(Instant::now());
                page.bl.is_none()
            });
            assert_eq!(page.take_status().as_deref(), Some(said));
            assert!(page.message().is_none(), "the status line, not a box");
        }
    }

    /// No to either question closes the link and sends nothing; nothing heard in time is
    /// "Failed to find device on mavlink".
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:542-561`
    #[test]
    fn no_closes_the_link_and_silence_finds_no_device() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = bl(telemetry);
        vehicle.heartbeat();
        until("the first question", || {
            page.bl_tick(Instant::now());
            page.question().is_some()
        });
        page.answer_bootloader(true);
        page.answer_bootloader(false);
        assert!(page.bl.is_none(), "mav.Close()");
        assert!(page.question().is_none());
        wasm_thread::sleep(Duration::from_millis(100));
        vehicle.read();
        assert!(flash_commands(&vehicle).is_empty());
        assert_eq!(page.take_status(), None);

        let (telemetry, _vehicle) = Vehicle::connect(fast());
        let mut page = bl(telemetry);
        page.bl_tick(Instant::now() + Duration::from_secs(31));
        assert!(page.bl.is_none());
        assert_eq!(page.take_status().as_deref(), Some(NO_DEVICE_ON_MAVLINK));
    }

    /// The link lost while the command waits: said on the status line in the window's words for
    /// a failed link, never left silent - a link made over a transport has no error to give.
    #[test]
    fn a_link_lost_under_the_command_is_said() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = bl(telemetry);
        vehicle.heartbeat();
        until("the first question", || {
            page.bl_tick(Instant::now());
            page.question().is_some()
        });
        page.answer_bootloader(true);
        page.answer_bootloader(true);
        vehicle.unplug();
        let deadline = Instant::now() + Duration::from_secs(10);
        while page.bl.is_some() && Instant::now() < deadline {
            page.bl_tick(Instant::now());
            wasm_thread::sleep(Duration::from_millis(5));
        }
        assert!(page.bl.is_none());
        let said = page.take_status().expect("something said");
        assert_eq!(
            said, "link failed: the link closed",
            "no error of its own to give"
        );
    }

    /// The command unanswered through its wait and its one retry: `doCommand`'s timeout, said
    /// on the status line.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2750-2754, 2781-2794`
    #[test]
    fn an_unanswered_command_times_out_on_the_status_line() {
        let (telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = bl(telemetry);
        vehicle.heartbeat();
        until("the first question", || {
            page.bl_tick(Instant::now());
            page.question().is_some()
        });
        page.answer_bootloader(true);
        page.answer_bootloader(true);
        let deadline = Instant::now() + Duration::from_secs(10);
        while page.bl.is_some() && Instant::now() < deadline {
            page.bl_tick(Instant::now());
            wasm_thread::sleep(Duration::from_millis(5));
        }
        assert!(page.bl.is_none());
        assert_eq!(page.take_status().as_deref(), Some(DO_COMMAND_TIMEOUT));
        vehicle.read();
        assert_eq!(
            flash_commands(&vehicle).len(),
            2,
            "sent, and sent once more"
        );
    }
}
