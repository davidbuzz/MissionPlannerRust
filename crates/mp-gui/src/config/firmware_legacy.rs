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

//! Install Firmware Legacy: `GCSViews/ConfigurationView/ConfigFirmware.cs`, the page Initial
//! Setup lists third while no vehicle is connected (`GCSViews/InitialSetup.cs:173`,
//! `isDisConnected`), for the catalogue Mission Planner used before the manifest -
//! `firmware2.xml` (`mp_firmware::legacy`).
//!
//! What it shows, as `ConfigFirmware.resx` places it in a 986 x 462 page: eleven vehicle
//! pictures, Rover, Plane (`pictureBoxAPM`), Quad, Hexa, Octa Quad and Sub above and Antenna
//! Tracker, Heli, Tri, Y6 and Octa below, and a picture of a Cube that links to ProfiCNC; "Copter
//! motor setup", a link to the wiki's motor order diagrams; "Please click the images above for
//! "Flight versions"" and the licence line, which links to firmware.ardupilot.org; Load custom firmware,
//! Download firmwares, Beta firmwares, Force Bootloader and Pick previous firmware, with the
//! history drop-down that replaces the last; the progress bar and status line; and "Images by
//! Max Levine".
//!
//! What it does (`ConfigFirmware.cs`):
//!
//! * `Activate`, the first time for the page object: `UpdateFWList` behind a progress dialog -
//!   `getFWList` reads the list from GitHub or ardupilot.org and each entry's version from its
//!   build's `git-version.txt` - and `updateDisplayName` gives each picture an entry: its label,
//!   and its `Tag`, which a click uploads. Every time, the advanced view's labels are shown;
//! * a picture: an entry's `findfirmware` ("Are you sure you want to upload ...?", the board
//!   detected, the entry's build for it downloaded to `firmware.hex`, `UploadFlash`); a picture
//!   with no entry says "Error loading firmware file";
//! * Pick previous firmware: the history (`FirmwareHistory.txt`) in the drop-down, which a choice
//!   loads - and binding it selects its first entry, so the newest history loads at once;
//! * Beta firmwares: "These are beta firmware, use at your own risk!!!", then the `dev` list;
//! * Load custom firmware: a file, its board by extension or detection, `UploadFlash`;
//! * Download firmwares and the licence line open firmware.ardupilot.org; the motor setup link
//!   and the Cube open theirs;
//! * Force Bootloader (`lbl_px4bl_Click`, `:619-639`): `MainV2.comPort.Open(false)`, then
//!   `doReboot(true, false)` and "Please ignore the unplug and plug back in when uploading flight
//!   firmware." - the manifest page's handler line for line, so the page holds the same
//!   [`ForceBootloader`] the manifest page does: the window's link opened from the port box, or
//!   found open, the board rebooted into its bootloader, the box over this page, and "Failed to
//!   connect and send the reboot command" on the status line (a box in the C#).
//!
//! The flows run on a thread with their questions as boxes over the page, as the manifest page's
//! do (`firmware.rs`). `UploadFlash` is the manifest page's: a px4-family board is rebooted into
//! its bootloader and written through `mp_firmware::flow::upload_px4` over the serial host - not
//! while `MP_FIRMWARE_DEVICE` stands in for the machine, when the flow stops before the reboot
//! and says so - and the uploads this application does not make (STK500, VRBRAIN, Parrot, Solo)
//! stop at their step (`mp_firmware::flow::Stop`) and say it is not ported. Not here, and why:
//!
//! * `Ctrl+Q` (the trunk list) and `Ctrl+P` (the first `px4` entry's upload), `ProcessCmdKey`
//!   (`:108-129`): the C# sees them only while a control of the page has the keyboard, and
//!   nothing on this page takes it;
//! * `Instance_DeviceChanged` (`:66-106`), which opens every serial port with the bootloader's
//!   identify when a device is plugged in - and whose answer nothing reads.
//!
//! Each picture is its `Image`, zoomed as `ImageLabel`'s `PictureBox` zooms it
//! ([`crate::pictures`]). The colours are this application's.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, TryRecvError, channel};

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_firmware::flow::{self, Reached};
use mp_firmware::legacy::{self, Picture as Slot, Software};

use super::firmware::{
    BoxIds, FIRMWARE_FILE_DIRECTORY, Machine, PathBox, Progress, Waiting, Worker, line, link_label,
    message_box, path_box, progress_bar, question_box, reached_lines, record_reached_saying,
    remember_folder, stop_text,
};
use super::force_bootloader::{ForceBootloader, Page as ForcePage};
use super::servo_output::{Combo, dropdown};
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::setup::Key;
use crate::textfield::KeyOutcome;
use crate::ui::{panel, theme};

/// The class, as the lists name it.
pub const CLASS: &str = "ConfigFirmware";

/// `$this.Size`. `// C#: GCSViews/ConfigurationView/ConfigFirmware.resx ($this.Size)`
pub const PAGE: (f32, f32) = (986.0, 462.0);

/// Load custom firmware's filter on this page.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:524`
pub const LEGACY_FILTER: &str =
    "Firmware (*.hex;*.px4;*.vrx;*.apj)|*.hex;*.px4;*.vrx;*.apj|All files (*.*)|*.*";

/// Where Download firmwares and the licence line go.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:611`
pub const FIRMWARE_SITE: &str = "https://firmware.ardupilot.org/";

/// Where "Copter motor setup" goes.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:645`
pub const MOTOR_ORDER: &str =
    "http://copter.ardupilot.com/wiki/connect-escs-and-motors/#motor_order_diagrams";

/// Where the Cube goes.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:657`
pub const PROFICNC: &str =
    "http://www.proficnc.com/?utm_source=missionplanner&utm_medium=click&utm_campaign=mission";

/// `Strings.BetaWarning` and `Strings.Beta`.
/// `// C#: ExtLibs/Strings/Strings.resx:462-467`
pub const BETA_WARNING: (&str, &str) = ("These are beta firmware, use at your own risk!!!", "Beta");

/// A vehicle picture: its Designer name, the id a test clicks, what it shows, which of
/// `updateDisplayName`'s pictures it is, and its `Location`. Every one is 150 x 150.
#[derive(Debug, Clone, Copy)]
pub struct Picture {
    /// The Designer's name: the citation, which the tests hold to the Designer.
    #[cfg_attr(not(test), allow(dead_code))]
    pub control: &'static str,
    /// The id a test clicks, and the last part of its facts.
    pub id: &'static str,
    /// What the picture shows, written in its place when its image is not carried.
    pub name: &'static str,
    /// Its `Image`, a `Properties.Resources` name ([`crate::pictures`]).
    pub image: &'static str,
    /// Which picture `updateDisplayName` knows it as.
    pub slot: Slot,
    /// `Location`.
    pub at: (f32, f32),
}

/// The eleven, in the Designer's order of declaration.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.Designer.cs:55-77, 84-260 (*.Image: 84, 94, 104, 114, 124, 150, 159, 168, 178, 237, 260); ConfigFirmware.resx (*.Location)`
pub const PICTURES: [Picture; 11] = [
    Picture {
        control: "pictureBoxAPM",
        id: "fwl-apm",
        name: "Plane",
        image: "APM_airframes_001",
        slot: Slot::Apm,
        at: (186.0, 0.0),
    },
    Picture {
        control: "pictureBoxQuad",
        id: "fwl-quad",
        name: "Quad",
        image: "FW_icons_2013_logos_04",
        slot: Slot::Quad,
        at: (342.0, 0.0),
    },
    Picture {
        control: "pictureBoxHexa",
        id: "fwl-hexa",
        name: "Hexa",
        image: "FW_icons_2013_logos_10",
        slot: Slot::Hexa,
        at: (498.0, 0.0),
    },
    Picture {
        control: "pictureBoxTri",
        id: "fwl-tri",
        name: "Tri",
        image: "FW_icons_2013_logos_08",
        slot: Slot::Tri,
        at: (342.0, 176.0),
    },
    Picture {
        control: "pictureBoxY6",
        id: "fwl-y6",
        name: "Y6",
        image: "y6a",
        slot: Slot::Y6,
        at: (498.0, 176.0),
    },
    Picture {
        control: "pictureBoxHeli",
        id: "fwl-heli",
        name: "Heli",
        image: "APM_airframes_08",
        slot: Slot::Heli,
        at: (186.0, 176.0),
    },
    Picture {
        control: "pictureBoxOcta",
        id: "fwl-octa",
        name: "Octa",
        image: "FW_icons_2013_logos_12",
        slot: Slot::Octa,
        at: (654.0, 176.0),
    },
    Picture {
        control: "pictureBoxOctaQuad",
        id: "fwl-octaquad",
        name: "Octa Quad",
        image: "x8",
        slot: Slot::OctaQuad,
        at: (654.0, 0.0),
    },
    Picture {
        control: "pictureBoxRover",
        id: "fwl-rover",
        name: "Rover",
        image: "rover_11",
        slot: Slot::Rover,
        at: (30.0, 0.0),
    },
    Picture {
        control: "pictureAntennaTracker",
        id: "fwl-tracker",
        name: "Antenna Tracker",
        image: "Antenna_Tracker_01",
        slot: Slot::AntennaTracker,
        at: (30.0, 176.0),
    },
    Picture {
        control: "pictureBoxSub",
        id: "fwl-sub",
        name: "Sub",
        image: "sub",
        slot: Slot::Sub,
        at: (810.0, 0.0),
    },
];

/// A picture's size.
const PICTURE: f32 = 150.0;

/// `imageLabel1`'s `Image`: a Cube.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.Designer.cs:269`
pub const CUBE_IMAGE: &str = "pixhawk2cube";

/// `imageLabel1`, the Cube: its place and size.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.resx (imageLabel1.Location, .Size)`
const CUBE: (f32, f32, f32, f32) = (810.0, 176.0, 150.0, 167.0);

/// A label: its Designer name, the id a test clicks, its `Text` and `Location`, and its handler
/// (`Click`, or `LinkClicked` for `linkLabel1`); `None` for the two with none.
pub type Label = (
    &'static str,
    &'static str,
    &'static str,
    (f32, f32),
    Option<&'static str>,
);

/// The labels, as the Designer adds them to the page.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.Designer.cs:131-256; ConfigFirmware.resx`
pub const LABELS: [Label; 10] = [
    ("lbl_status", "fwl-status", "Status", (3.0, 443.0), None),
    (
        "label2",
        "fwl-images-by",
        "Images by Max Levine",
        (862.0, 443.0),
        None,
    ),
    (
        "label1",
        "fwl-flight-versions",
        "Please click the images above for \"Flight versions\"",
        (397.0, 330.0),
        None,
    ),
    (
        "CMB_history_label",
        "fwl-history-label",
        "Pick previous firmware",
        (862.0, 393.0),
        Some("CMB_history_label_Click"),
    ),
    (
        "lbl_Custom_firmware_label",
        "fwl-custom",
        "Load custom firmware",
        (862.0, 373.0),
        Some("Custom_firmware_label_Click"),
    ),
    (
        "lbl_devfw",
        "fwl-beta",
        "Beta firmwares",
        (630.0, 393.0),
        Some("lbl_devfw_Click"),
    ),
    (
        "lbl_dlfw",
        "fwl-download",
        "Download firmwares",
        (738.0, 373.0),
        Some("lbl_dlfw_Click"),
    ),
    (
        "lbl_px4bl",
        "fwl-px4bl",
        "Force Bootloader",
        (738.0, 393.0),
        Some("lbl_px4bl_Click"),
    ),
    (
        "lbl_licence",
        "fwl-licence",
        "Ardupilot is free software. Click here to read the Open Source Firmware License.",
        (348.0, 349.0),
        Some("lbl_dlfw_Click"),
    ),
    (
        "linkLabel1",
        "fwl-motors",
        "Copter motor setup",
        (708.0, 0.0),
        Some("linkLabel1_LinkClicked"),
    ),
];

/// `CMB_history`: its place and size, hidden until Pick previous firmware.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.resx (CMB_history.*); Designer.cs:192-197`
const HISTORY_BOX: (f32, f32, f32, f32) = (856.0, 389.0, 121.0, 21.0);

/// `CMB_history.DropDownWidth`. `// C#: GCSViews/ConfigurationView/ConfigFirmware.Designer.cs:193`
const HISTORY_DROP_WIDTH: f32 = 160.0;

/// `progress`. `// C#: GCSViews/ConfigurationView/ConfigFirmware.resx (progress.*)`
const PROGRESS: (f32, f32, f32, f32) = (3.0, 417.0, 980.0, 23.0);

/// The boxes' ids on this page.
const IDS: BoxIds = BoxIds {
    question: "fwl-question",
    yes: "fwl-question-yes",
    no: "fwl-question-no",
    message: "fwl-message",
    ok: "fwl-message-ok",
    path: "fwl-path",
    path_value: "fwl-path-value",
    path_ok: "fwl-path-ok",
    path_cancel: "fwl-path-cancel",
};

/// What `UpdateFWList`'s thread says.
#[derive(Debug)]
enum Listed {
    /// `updateProgress`'s status.
    Status(String),
    /// `getFWList`'s answer.
    Done(Result<Vec<Software>, String>),
}

/// `UpdateFWList`'s `ProgressReporterDialogue`: the status while the list loads; the error it
/// shows when `getFWList` throws, with its Close.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:131-167; ExtLibs/Controls/ProgressReporterDialogue.cs:60-247`
#[derive(Debug)]
struct Loading {
    receiver: Option<Receiver<Listed>>,
    status: String,
    error: Option<String>,
}

/// A message box and what its OK goes on to do.
#[derive(Debug, Clone, PartialEq, Eq)]
enum AfterOk {
    /// Nothing.
    Nothing,
    /// Beta firmwares' warning: the `dev` list.
    Beta,
}

/// The page object, and the list it keeps.
#[derive(Debug)]
pub struct FirmwareLegacy {
    /// The screen the page object belongs to; a different one is a new object.
    made_for: Option<Key>,
    /// Between `Activate` and `Deactivate`.
    open: bool,
    /// `firstrun`: `Activate` has not yet loaded the list for this page object.
    firstrun: bool,
    /// `firmwareurl`: empty for the default sources.
    firmwareurl: String,
    /// `softwares`, static in the C#: the list, each entry as `updateDisplayName` left it.
    softwares: Vec<Software>,
    /// Each picture's `Text`, in [`PICTURES`] order.
    labels: [String; 11],
    /// Each picture's `Tag`: the entry it uploads - the object itself, which a list cleared or
    /// reloaded does not take from it.
    tags: [Option<Software>; 11],
    /// The progress dialog, while it shows.
    loading: Option<Loading>,
    /// `CMB_history`: the history's text by index, and the selected one.
    history: Combo,
    /// The history's keys, the lists' URLs, by the same index.
    history_keys: Vec<String>,
    /// `CMB_history.Visible`.
    history_visible: bool,
    /// `CMB_history_label.Visible`.
    history_label_visible: bool,
    /// The drop-down is down.
    history_open: bool,
    /// A vehicle's upload or a custom firmware on its way to the board.
    worker: Option<Worker>,
    /// `progress` and `lbl_status`.
    progress: Progress,
    /// Where the last flow got.
    reached: Option<Reached>,
    /// The page's own message boxes, the first showing.
    messages: VecDeque<(Waiting, AfterOk)>,
    /// Load custom firmware's dialog.
    path: Option<PathBox>,
    /// The URL the last link opened.
    opened: Option<&'static str>,
    /// Force Bootloader: its steps over the window's link, its box and its failure - the
    /// manifest page's handler too ([`ForceBootloader`]).
    force: ForceBootloader,
    /// Where the list is read from: `None` is `mp_firmware::manifest::fetcher` - the network, or
    /// the fixtures a script names - and `Some` a mirror directory, for a unit test.
    source: Option<std::path::PathBuf>,
}

impl Default for FirmwareLegacy {
    /// `InitializeComponent`: no labels, no tags, the history hidden behind its label.
    fn default() -> Self {
        Self {
            made_for: None,
            open: false,
            firstrun: true,
            firmwareurl: String::new(),
            softwares: Vec::new(),
            labels: Default::default(),
            tags: Default::default(),
            loading: None,
            history: Combo::default(),
            history_keys: Vec::new(),
            history_visible: false,
            history_label_visible: true,
            history_open: false,
            worker: None,
            progress: Progress::default(),
            reached: None,
            messages: VecDeque::new(),
            path: None,
            opened: None,
            force: ForceBootloader::default(),
            source: None,
        }
    }
}

impl FirmwareLegacy {
    /// Shows the page: a new page object for a new screen, then `Activate` - the list loaded the
    /// first time, and the advanced view's labels shown, which they always are here.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:34-64`
    pub fn activate(&mut self, key: Key) {
        if self.made_for != Some(key) {
            self.dispose();
            self.made_for = Some(key);
        }
        self.open = true;
        if self.firstrun {
            self.update_fw_list();
            self.firstrun = false;
        }
    }

    /// `Deactivate`: it unsubscribes from device arrivals, which are not heard here. Force
    /// Bootloader stops and its box closes, as they do when the manifest page is left
    /// ([`ForceBootloader::cancel`]).
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:665-673`
    pub fn deactivate(&mut self) {
        self.open = false;
        self.history_open = false;
        self.force.cancel();
    }

    /// The screen kept under a new key: Force Bootloader's `Open(false)` moved the window's link,
    /// for which `MainV2` shows no screen again, so the page object is the same one
    /// ([`crate::setup::Backstage::rekey`]).
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:623; MainV2.cs:1419-1425, 1740-1748`
    pub fn rekey(&mut self, key: Key) {
        if self.made_for.is_some() {
            self.made_for = Some(key);
        }
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

    /// What the page has to say on the window's status line, once: its Force Bootloader's.
    pub fn take_status(&mut self) -> Option<String> {
        self.force.take_status()
    }

    /// The page object let go with its screen; `softwares`, a static, stays.
    fn dispose(&mut self) {
        *self = Self {
            softwares: std::mem::take(&mut self.softwares),
            source: self.source.take(),
            ..Self::default()
        };
    }

    /// `UpdateFWList`: the progress dialog, and `getFWList(firmwareurl)` on its thread.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:131-167`
    fn update_fw_list(&mut self) {
        let fetch: Box<dyn mp_firmware::manifest::Fetch + Send + Sync> = match &self.source {
            Some(root) => Box::new(mp_firmware::manifest::Mirror { root: root.clone() }),
            None => mp_firmware::manifest::fetcher(),
        };
        self.update_fw_list_from(fetch);
    }

    /// [`FirmwareLegacy::update_fw_list`] from a given source.
    fn update_fw_list_from(&mut self, fetch: Box<dyn mp_firmware::manifest::Fetch + Send + Sync>) {
        let (sender, receiver) = channel();
        let firmwareurl = self.firmwareurl.clone();
        let started = std::thread::Builder::new()
            .name("mp-firmware-legacy-list".to_owned())
            .spawn(move || {
                let status = sender.clone();
                let list = legacy::get_fw_list(&firmwareurl, fetch.as_ref(), &mut |_, text| {
                    let _ = status.send(Listed::Status(text.to_owned()));
                });
                let _ = sender.send(Listed::Done(list));
            });
        self.loading = Some(Loading {
            receiver: started.is_ok().then_some(receiver),
            status: String::new(),
            error: started.err().map(|err| err.to_string()),
        });
    }

    /// The list arrived: `softwares`, and `updateDisplayName` for each entry, in order - a later
    /// entry's picture replacing an earlier's.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:150-167, 270-389`
    fn listed(&mut self, softwares: Vec<Software>) {
        self.softwares = softwares;
        for entry in &mut self.softwares {
            for (slot, text) in legacy::display_name(entry) {
                if let Some(at) = PICTURES.iter().position(|picture| picture.slot == slot) {
                    if let Some(label) = self.labels.get_mut(at) {
                        *label = text;
                    }
                    if let Some(tag) = self.tags.get_mut(at) {
                        *tag = Some(entry.clone());
                    }
                }
            }
        }
    }

    /// Once a frame: the list's thread heard, the flow's, and the page object let go when its
    /// screen has gone.
    pub fn tick(&mut self, key: Key, on_setup: bool) {
        if let Some(loading) = &mut self.loading
            && let Some(receiver) = &loading.receiver
        {
            let mut arrived = None;
            loop {
                match receiver.try_recv() {
                    Ok(Listed::Status(status)) => loading.status = status,
                    Ok(Listed::Done(list)) => {
                        arrived = Some(list);
                        break;
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        arrived = Some(Err("the list's thread ended".to_owned()));
                        break;
                    }
                }
            }
            match arrived {
                Some(Ok(list)) => {
                    self.loading = None;
                    self.listed(list);
                }
                Some(Err(error)) => {
                    loading.receiver = None;
                    // `ShowDoneWithError(e, null)`.
                    loading.error = Some(format!("There was an unexpected error ({error})"));
                }
                None => {}
            }
        }
        if let Some(worker) = &mut self.worker
            && let Some(reached) = worker.poll(&mut self.progress)
        {
            self.worker = None;
            self.reached = Some(reached);
        }
        if !self.open && self.made_for.is_some() && (!on_setup || self.made_for != Some(key)) {
            self.dispose();
        }
    }

    /// Whether the page takes a click: open, with nothing over it and no flow running - Force
    /// Bootloader's handler holds the window as the flows' do, and its box is over the page.
    #[must_use]
    pub fn live(&self) -> bool {
        self.open
            && self.loading.is_none()
            && self.messages.is_empty()
            && self.path.is_none()
            && self.worker.is_none()
            && !self.force.busy()
    }

    /// `pictureBoxFW_Click`: a picture with no entry says "Error loading firmware file"; one with
    /// an entry runs `findfirmware` for it, with the history's selection.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:466-475, 393-464`
    pub fn click(&mut self, index: usize, settings: &Persisted) {
        if !self.live() {
            return;
        }
        let entry = self.tags.get(index).cloned().flatten();
        let Some(entry) = entry else {
            self.messages.push_back((
                Waiting {
                    text: flow::ERROR_FIRMWARE_FILE.to_owned(),
                    caption: flow::ERROR.to_owned(),
                    buttons: None,
                },
                AfterOk::Nothing,
            ));
            return;
        };
        // `CMB_history.SelectedValue`: null - "" - until the history is bound.
        let history = self.history_value().unwrap_or_default();
        self.reached = None;
        let machine = Machine::here(settings);
        self.worker = Worker::start("mp-firmware-legacy", move |dialogue| {
            machine.run(dialogue, |cx, machine| {
                let reached = flow::find_firmware(
                    cx,
                    &machine.comport,
                    &entry,
                    &history,
                    &machine.devices,
                    &machine.rows,
                );
                // `UploadFlash`: a px4-family board written, as the manifest page writes it.
                // `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:460-463; Utilities/Firmware.cs:591-750`
                if machine.flash {
                    let mut host = machine.host();
                    flow::flash_if_stopped(cx, &mut host, reached)
                } else {
                    reached
                }
            })
        });
    }

    /// The history's selected key.
    fn history_value(&self) -> Option<String> {
        let selected = usize::try_from(self.history.selected?).ok()?;
        self.history_keys.get(selected).cloned()
    }

    /// `CMB_history_label_Click`: the history bound to the drop-down, which is shown in the
    /// label's place. Binding it selects its first entry, and WinForms raises
    /// `SelectedIndexChanged` for that, so the newest history's list loads at once.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:504-518`
    pub fn show_history(&mut self) {
        if !self.live() || !self.history_label_visible {
            return;
        }
        let names = legacy::nice_names();
        self.history_keys = names.iter().map(|(key, _)| key.clone()).collect();
        self.history.options = names
            .into_iter()
            .enumerate()
            .map(|(index, (_, text))| (i64::try_from(index).unwrap_or(i64::MAX), text))
            .collect();
        self.history.enabled = true;
        self.history.selected = None;
        self.history_visible = true;
        self.history_label_visible = false;
        self.choose_history(0);
    }

    /// The drop-down opened or closed.
    pub fn toggle_history(&mut self) {
        if !self.live() || !self.history_visible {
            return;
        }
        self.history_open = !self.history_open;
        if self.history_open {
            self.history.open_list();
        }
    }

    /// The drop-down's wheel.
    pub fn scroll_history(&mut self, lines: i32) {
        self.history.scroll_list(lines);
    }

    /// A history chosen: `CMB_history_SelectedIndexChanged` when it changes the selection -
    /// `firmwareurl` the list's URL, and the list loaded from it.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:496-501`
    pub fn choose_history(&mut self, key: i64) {
        self.history_open = false;
        if !self.history.select(key) {
            return;
        }
        let Some(url) = self.history_value() else {
            return;
        };
        self.firmwareurl = legacy::get_url(&url, "").unwrap_or_default();
        self.softwares.clear();
        self.update_fw_list();
    }

    /// `lbl_devfw_Click`: the warning, then the `dev` list, and the history hidden.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:597-604`
    pub fn beta(&mut self) {
        if !self.live() {
            return;
        }
        let (text, caption) = BETA_WARNING;
        self.messages.push_back((
            Waiting {
                text: text.to_owned(),
                caption: caption.to_owned(),
                buttons: None,
            },
            AfterOk::Beta,
        ));
    }

    /// A message box's OK, and what it goes on to.
    pub fn dismiss_message(&mut self) {
        if let Some((_, AfterOk::Beta)) = self.messages.pop_front() {
            self.firmwareurl = legacy::BETA_URLS.to_owned();
            self.softwares.clear();
            self.update_fw_list();
            self.history_visible = false;
            self.history_open = false;
        }
    }

    /// The progress dialog's Cancel, or its Close after an error: the dialog goes, and nothing
    /// more is labelled.
    pub fn close_loading(&mut self) {
        self.loading = None;
    }

    /// Load custom firmware: its file dialog, in the folder last used.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:521-528`
    pub fn custom(&mut self, settings: &Persisted) {
        if !self.live() {
            return;
        }
        let folder = settings.get(FIRMWARE_FILE_DIRECTORY).unwrap_or_default();
        self.path = Some(PathBox::new(folder, LEGACY_FILTER));
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
    /// `custom_legacy` on a thread.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:529-594`
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
        self.worker = Worker::start("mp-firmware-legacy-custom", move |dialogue| {
            machine.run(dialogue, |cx, machine| {
                let reached = flow::custom_legacy(
                    cx,
                    &machine.comport,
                    &file,
                    &machine.devices,
                    &machine.rows,
                );
                // `UploadFlash(comport, file, board)`, as a vehicle's click has it.
                // `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:590-593`
                if machine.flash {
                    let mut host = machine.host();
                    flow::flash_if_stopped(cx, &mut host, reached)
                } else {
                    reached
                }
            })
        });
    }

    /// The flow's box answered.
    pub fn answer_worker(&mut self, yes: bool) {
        if let Some(worker) = &mut self.worker {
            worker.answer(yes);
        }
    }

    /// A link's URL, noted as opened; the caller opens it.
    pub fn open_link(&mut self, url: &'static str) -> Option<&'static str> {
        if !self.live() {
            return None;
        }
        self.opened = Some(url);
        Some(url)
    }

    /// Whether the path dialog is open, for its focus.
    #[must_use]
    pub const fn path_open(&self) -> bool {
        self.path.is_some()
    }

    /// The question showing: the flow's.
    fn question(&self) -> Option<&Waiting> {
        self.worker
            .as_ref()
            .and_then(|worker| worker.waiting.as_ref())
            .filter(|waiting| waiting.buttons.is_some())
    }

    /// The message showing: the flow's, the page's own, or Force Bootloader's.
    fn message(&self) -> Option<&Waiting> {
        self.worker
            .as_ref()
            .and_then(|worker| worker.waiting.as_ref())
            .filter(|waiting| waiting.buttons.is_none())
            .or_else(|| self.messages.front().map(|(waiting, _)| waiting))
            .or_else(|| self.force.message())
    }

    /// Where the list stands.
    fn list_state(&self) -> &'static str {
        match &self.loading {
            Some(Loading { error: Some(_), .. }) => "failed",
            Some(_) => "loading",
            None if self.softwares.is_empty() => "none",
            None => "loaded",
        }
    }
}

/// Facts for a test: whether the page shows, the list, each picture's label and entry, the
/// history, the boxes showing, and where the flow got.
pub fn record_facts(page: &FirmwareLegacy, settings: &Persisted, listed: bool) {
    use crate::facts::record;
    record("config.firmware_legacy.listed", listed);
    record("config.firmware_legacy.active", page.open);
    record("config.firmware_legacy.list", page.list_state());
    record(
        "config.firmware_legacy.list.status",
        page.loading
            .as_ref()
            .map_or("none", |loading| loading.status.as_str()),
    );
    record(
        "config.firmware_legacy.list.error",
        page.loading
            .as_ref()
            .and_then(|loading| loading.error.as_deref())
            .unwrap_or("none"),
    );
    record(
        "config.firmware_legacy.url",
        if page.firmwareurl.is_empty() {
            "default"
        } else {
            page.firmwareurl.as_str()
        },
    );
    record("config.firmware_legacy.entries", page.softwares.len());
    for ((picture, label), tag) in PICTURES.iter().zip(&page.labels).zip(&page.tags) {
        let key = picture.id.trim_start_matches("fwl-");
        record(format!("config.firmware_legacy.label.{key}"), label);
        record(
            format!("config.firmware_legacy.tag.{key}"),
            tag.as_ref().map_or("none", |entry| entry.name.as_str()),
        );
    }
    record(
        "config.firmware_legacy.history",
        if page.history_visible {
            "visible"
        } else {
            "hidden"
        },
    );
    record(
        "config.firmware_legacy.history.label",
        page.history_label_visible,
    );
    record(
        "config.firmware_legacy.history.items",
        page.history.options.len(),
    );
    record(
        "config.firmware_legacy.history.selected",
        if page.history.text().is_empty() {
            "none"
        } else {
            page.history.text()
        },
    );
    record(
        "config.firmware_legacy.history.list.top",
        page.history.top_index,
    );
    record(
        "config.firmware_legacy.question",
        page.question()
            .map_or("none", |waiting| waiting.text.as_str()),
    );
    record(
        "config.firmware_legacy.message",
        page.message().map_or_else(
            || "none".to_owned(),
            |waiting| waiting.text.replace('\n', " "),
        ),
    );
    record("config.firmware_legacy.progress", page.progress.value);
    record("config.firmware_legacy.status", &page.progress.status);
    record(
        "config.firmware_legacy.path",
        page.path
            .as_ref()
            .map_or_else(|| "closed".to_owned(), |path| path.field.value().to_owned()),
    );
    record(
        "config.firmware_legacy.custom.dir",
        settings.get(FIRMWARE_FILE_DIRECTORY).unwrap_or("none"),
    );
    record(
        "config.firmware_legacy.opened",
        page.opened.unwrap_or("none"),
    );
    record("config.firmware_legacy.force", page.force.fact());
    // A stop before the reboot is the device door's, said as the manifest page says it.
    record_reached_saying(
        "config.firmware_legacy",
        page.reached.as_ref(),
        page.worker.is_some(),
        stop_text,
    );
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// A box at a `.resx` `Location` and `Size`.
fn at(x: f32, y: f32, width: f32, height: f32) -> gpui::Div {
    super::optional::at(x, y, width, height)
}

/// The page, laid out as `ConfigFirmware.resx` lays it out.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.Designer.cs:52-307; ConfigFirmware.resx`
pub fn page(firmware: &FirmwareLegacy, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let live = firmware.live();
    let mut body = div().relative().w(px(PAGE.0)).h(px(PAGE.1));
    for (index, picture) in PICTURES.iter().enumerate() {
        body = body.child(image_label(firmware, index, picture, live, cx));
    }
    // `imageLabel1`: the Cube, a link to ProfiCNC.
    let (x, y, w, h) = CUBE;
    // Its `PictureBox` and, under it, its empty `Label`.
    let cube = crate::probe::measured("fwl-cube", at(x, y, w, h))
        .id("fwl-cube")
        .flex()
        .flex_col()
        .rounded_md()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .child(crate::pictures::image_label(
            "firmware_legacy",
            "fwl-cube",
            CUBE_IMAGE,
            "Pixhawk 2 Cube",
            theme::DIM,
        ))
        .child(div().h(px(13.0)).mb_1());
    body = body.child(if live {
        cube.cursor_pointer()
            .on_click(cx.listener(|this, _event, _window, cx| {
                if let Some(url) = this.firmware_legacy.open_link(PROFICNC) {
                    cx.open_url(url);
                }
                cx.notify();
            }))
            .into_any_element()
    } else {
        cube.into_any_element()
    });
    body = body.child(progress_bar(PROGRESS, firmware.progress.value));
    for (_, id, text, (x, y), handler) in LABELS {
        let element = match id {
            "fwl-status" => crate::probe::measured(id, at(x, y, 450.0, 13.0))
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(firmware.progress.status.clone())
                .into_any_element(),
            "fwl-history-label" if !firmware.history_label_visible => continue,
            "fwl-history-label" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, _window, _cx| this.firmware_legacy.show_history(),
                cx,
            ),
            "fwl-custom" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, window, cx| {
                    this.firmware_legacy.custom(&this.persisted);
                    this.firmware_focus.focus(window, cx);
                },
                cx,
            ),
            "fwl-beta" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, _window, _cx| this.firmware_legacy.beta(),
                cx,
            ),
            "fwl-download" | "fwl-licence" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, _window, cx| {
                    if let Some(url) = this.firmware_legacy.open_link(FIRMWARE_SITE) {
                        cx.open_url(url);
                    }
                },
                cx,
            ),
            "fwl-motors" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, _window, cx| {
                    if let Some(url) = this.firmware_legacy.open_link(MOTOR_ORDER) {
                        cx.open_url(url);
                    }
                },
                cx,
            ),
            // `lbl_px4bl_Click`: the manifest page's Force Bootloader.
            // C#: GCSViews/ConfigurationView/ConfigFirmware.cs:619-639
            "fwl-px4bl" => link_label(
                id,
                text,
                (x, y),
                live,
                |this, _window, _cx| this.force_bootloader_clicked(ForcePage::Legacy),
                cx,
            ),
            // `label1` and `label2`, which do nothing.
            _ if handler.is_none() => crate::probe::measured(id, at(x, y, 400.0, 13.0))
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(text)
                .into_any_element(),
            _ => continue,
        };
        body = body.child(element);
    }
    if firmware.history_visible {
        let (x, y, w, h) = HISTORY_BOX;
        body = body.child(super::servo_output::combo_box(
            "fwl-history".to_owned(),
            &Combo {
                enabled: live,
                ..firmware.history.clone()
            },
            (x, y, w, h),
            |this| this.firmware_legacy.toggle_history(),
            cx,
        ));
        if firmware.history_open {
            body = body.child(dropdown(
                "fwl-history",
                &firmware.history,
                (x, y + h, HISTORY_DROP_WIDTH),
                |this, key| this.firmware_legacy.choose_history(key),
                |this, lines| this.firmware_legacy.scroll_history(lines),
                cx,
            ));
        }
    }

    let mut report = div().flex().flex_col().gap_1();
    if let Some(reached) = &firmware.reached {
        report = report.children(reached_lines(reached));
    } else {
        report = report.child(line(
            "Board",
            "chosen when a vehicle is clicked".to_owned(),
            theme::DIM,
        ));
    }
    panel(
        "install firmware legacy",
        div().flex().flex_col().gap_2().child(body).child(report),
    )
    .into_any_element()
}

/// One `ImageLabel`: the vehicle's picture, the entry's name under it, and the click.
fn image_label(
    firmware: &FirmwareLegacy,
    index: usize,
    picture: &Picture,
    live: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let label = firmware.labels.get(index).cloned().unwrap_or_default();
    let tagged = firmware.tags.get(index).is_some_and(Option::is_some);
    let colour = if tagged { theme::TEXT } else { theme::DIM };
    let (x, y) = picture.at;
    let body = crate::probe::measured(picture.id, at(x, y, PICTURE, PICTURE))
        .id(SharedString::from(picture.id))
        .flex()
        .flex_col()
        .rounded_md()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(if tagged { theme::ACTION } else { theme::PANEL }))
        // `PictureBox`: the rest, the `Image` zoomed.
        .child(crate::pictures::image_label(
            "firmware_legacy",
            picture.id,
            picture.image,
            picture.name,
            colour,
        ))
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
    if live {
        body.cursor_pointer()
            .hover(|style| style.bg(rgb(theme::BORDER)))
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.firmware_legacy.click(index, &this.persisted);
                cx.notify();
            }))
            .into_any_element()
    } else {
        body.into_any_element()
    }
}

/// The progress dialog: the status and Cancel while the list loads; "Error", the error and Close
/// once it has failed.
fn loading_box(loading: &Loading, window: &Window, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (caption, text, button, id) = match &loading.error {
        Some(error) => ("Error", error.clone(), "Close", "fwl-loading-close"),
        None => ("", loading.status.clone(), "Cancel", "fwl-loading-cancel"),
    };
    let close = crate::ui::action(
        id,
        button,
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), _window, cx| {
            this.firmware_legacy.close_loading();
            cx.notify();
        }),
    );
    crate::config::servo_output::modal(
        "fwl-loading",
        caption,
        &text,
        loading.error.is_some(),
        vec![close],
        window,
    )
}

/// The box showing over the page, if any.
pub fn overlay(
    firmware: &FirmwareLegacy,
    handle: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !firmware.open {
        return None;
    }
    if let Some(loading) = &firmware.loading {
        return Some(loading_box(loading, window, cx));
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
                |this, yes| this.firmware_legacy.answer_worker(yes),
                cx,
            ),
            None => message_box(
                IDS,
                waiting,
                window,
                |this| this.firmware_legacy.answer_worker(true),
                cx,
            ),
        });
    }
    if let Some((waiting, _)) = firmware.messages.front() {
        return Some(message_box(
            IDS,
            waiting,
            window,
            |this| this.firmware_legacy.dismiss_message(),
            cx,
        ));
    }
    // Force Bootloader's instruction, over this page.
    if let Some(waiting) = firmware.force.message() {
        return Some(message_box(
            IDS,
            waiting,
            window,
            |this| this.firmware_legacy.force.dismiss(),
            cx,
        ));
    }
    let path = firmware.path.as_ref()?;
    Some(path_box(
        IDS,
        path,
        handle,
        window,
        |this, event| this.firmware_legacy.path_key(event, &mut this.persisted),
        |this, ok| this.firmware_legacy.path_done(ok, &mut this.persisted),
        cx,
    ))
}

impl MissionPlanner {
    /// `Activate`, when SETUP's list shows the page.
    pub(crate) fn firmware_legacy_activate(&mut self) {
        let key = Key::of(&self.telemetry.view());
        self.firmware_legacy.activate(key);
    }

    /// Once a frame: the list's thread, the flow's, the page object, and the file dialog's focus.
    pub(crate) fn firmware_legacy_tick(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let key = Key::of(&self.telemetry.view());
        self.firmware_legacy
            .tick(key, self.screen == crate::Screen::Setup);
        let open = self.firmware_legacy.path_open() || self.install_firmware.path_open();
        if open && !self.firmware_focus.is_focused(window) {
            self.firmware_focus.focus(window, cx);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_coverage::source::{csharp, resx};

    fn fixture(name: &str) -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../mp-firmware/testdata/legacy")
            .join(name)
    }

    /// A web of fixtures: the list at its first source, the Copter version beside its build.
    fn web(name: &str) -> std::path::PathBuf {
        let root = std::env::temp_dir().join(format!("mp-gui-fwl-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        for (url, file) in [
            (
                "github.com/ArduPilot/binary/raw/master/Firmware/firmware2.xml",
                "firmware2.xml",
            ),
            (
                "firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj",
                "arducopter.apj",
            ),
            (
                "firmware.ardupilot.org/Copter/stable/fmuv3/git-version.txt",
                "git-version.txt",
            ),
        ] {
            let path = root.join(url);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::copy(fixture(file), path).unwrap();
        }
        root
    }

    fn key() -> Key {
        Key::of(&crate::telemetry::TelemetryView::disconnected("test"))
    }

    fn index_of(id: &str) -> usize {
        PICTURES
            .iter()
            .position(|picture| picture.id == id)
            .expect("a picture")
    }

    /// A page shown - `Activate` loading its list from the web - its list through the thread
    /// and `tick`.
    fn loaded(root: &std::path::Path) -> FirmwareLegacy {
        let mut page = FirmwareLegacy {
            source: Some(root.to_owned()),
            ..FirmwareLegacy::default()
        };
        page.activate(key());
        settle(&mut page);
        page
    }

    /// Ticks until the list is no longer loading.
    fn settle(page: &mut FirmwareLegacy) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while page.list_state() == "loading" && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(5));
            page.tick(key(), true);
        }
    }

    /// Every control the Designer makes is drawn here: the eleven pictures and the Cube, the ten
    /// labels, the history's drop-down and the progress bar.
    #[test]
    fn every_designer_control_is_drawn() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigFirmware.Designer.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let made: std::collections::BTreeSet<&str> = designer
            .lines()
            .filter_map(|line| line.trim().strip_prefix("this."))
            .filter(|line| line.contains(" = new ") && !line.starts_with("components"))
            .filter_map(|line| line.split_once(" = new "))
            .map(|(name, _)| name)
            .collect();
        let ours: std::collections::BTreeSet<&str> = PICTURES
            .iter()
            .map(|picture| picture.control)
            .chain(LABELS.iter().map(|(name, ..)| *name))
            .chain(["imageLabel1", "CMB_history", "progress"])
            .collect();
        assert_eq!(made, ours);
        assert_eq!(ours.len(), 11 + 10 + 3);
    }

    /// The Designer's 20 wirings: eleven pictures' `pictureBoxFW_Click`, the Cube's, and the
    /// labels' and the drop-down's handlers.
    #[test]
    fn every_wiring_is_handled() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigFirmware.Designer.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for picture in PICTURES {
            let line = format!(
                "this.{}.Click += new System.EventHandler(this.pictureBoxFW_Click);",
                picture.control
            );
            assert!(designer.contains(&line), "{line}");
        }
        let mut wired = PICTURES.len();
        for (name, _, _, _, handler) in LABELS {
            let Some(handler) = handler else {
                continue;
            };
            let line = if name == "linkLabel1" {
                format!(
                    "this.{name}.LinkClicked += new System.Windows.Forms.LinkLabelLinkClickedEventHandler(this.{handler});"
                )
            } else {
                format!("this.{name}.Click += new System.EventHandler(this.{handler});")
            };
            assert!(designer.contains(&line), "{line}");
            wired += 1;
        }
        for line in [
            "this.imageLabel1.Click += new System.EventHandler(this.picturebox_ph2_Click);",
            "this.CMB_history.SelectedIndexChanged += new System.EventHandler(this.CMB_history_SelectedIndexChanged);",
        ] {
            assert!(designer.contains(line), "{line}");
            wired += 1;
        }
        assert_eq!(designer.matches(" += new ").count(), wired);
        assert_eq!(wired, 20);
    }

    /// Every place and text is the `.resx`'s.
    #[test]
    fn every_place_and_text_is_the_resx() {
        let Some(text) = csharp("GCSViews/ConfigurationView/ConfigFirmware.resx") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = resx(&text);
        let get = |key: String| values.get(&key).cloned().unwrap_or_default();
        let pair = |(x, y): (f32, f32)| format!("{x}, {y}");
        assert_eq!(get("$this.Size".to_owned()), pair(PAGE));
        for picture in PICTURES {
            assert_eq!(
                get(format!("{}.Location", picture.control)),
                pair(picture.at),
                "{}",
                picture.control
            );
            assert_eq!(
                get(format!("{}.Size", picture.control)),
                pair((PICTURE, PICTURE))
            );
        }
        for (name, _, caption, place, _) in LABELS {
            assert_eq!(get(format!("{name}.Location")), pair(place), "{name}");
            assert_eq!(get(format!("{name}.Text")), caption, "{name}");
        }
        let (x, y, w, h) = CUBE;
        assert_eq!(get("imageLabel1.Location".to_owned()), pair((x, y)));
        assert_eq!(get("imageLabel1.Size".to_owned()), pair((w, h)));
        let (x, y, w, h) = HISTORY_BOX;
        assert_eq!(get("CMB_history.Location".to_owned()), pair((x, y)));
        assert_eq!(get("CMB_history.Size".to_owned()), pair((w, h)));
        assert_eq!(get("CMB_history.Visible".to_owned()), "False");
        let (x, y, w, h) = PROGRESS;
        assert_eq!(get("progress.Location".to_owned()), pair((x, y)));
        assert_eq!(get("progress.Size".to_owned()), pair((w, h)));
    }

    /// `Activate` loads the list once per page object, and each picture gets its entry.
    #[test]
    fn the_list_labels_each_picture_with_its_entry() {
        let root = web("labels");
        let page = loaded(&root);
        assert_eq!(page.list_state(), "loaded");
        assert_eq!(page.softwares.len(), 12);
        assert_eq!(page.labels[index_of("fwl-quad")], "ArduCopter V4.7.1 Quad");
        assert_eq!(page.labels[index_of("fwl-apm")], "ArduPlane Stable");
        assert_eq!(
            page.labels[index_of("fwl-heli")],
            "ArduCopter Heli Stable heli"
        );
        assert_eq!(page.labels[index_of("fwl-tracker")], "Antenna Tracker");
        // The Quad's tag is the entry, renamed by `temp.name += " Quad"`.
        let tag = page.tags[index_of("fwl-quad")].as_ref().expect("a tag");
        assert_eq!(tag.name, "ArduCopter V4.7.1 Quad");
        let _ = std::fs::remove_dir_all(root);
    }

    /// A list that cannot be had shows the progress dialog's error, and Close takes it away.
    #[test]
    fn a_failed_list_shows_the_dialogs_error() {
        let root = std::env::temp_dir().join(format!("mp-gui-fwl-empty-{}", std::process::id()));
        let page = &mut loaded(&root);
        assert_eq!(page.list_state(), "failed");
        let error = page
            .loading
            .as_ref()
            .and_then(|loading| loading.error.clone())
            .unwrap();
        assert!(
            error.starts_with("There was an unexpected error ("),
            "{error}"
        );
        page.close_loading();
        assert_eq!(page.list_state(), "none");
        assert!(page.labels.iter().all(String::is_empty));
    }

    /// A picture with no entry says so; the history binds and loads its first entry at once; Beta
    /// warns, then loads the `dev` list and hides the history.
    #[test]
    fn the_links_do_what_their_handlers_do() {
        // Nothing is served: each list fails, as a dead link does.
        let root = std::env::temp_dir().join(format!("mp-gui-fwl-links-{}", std::process::id()));
        let mut page = FirmwareLegacy {
            source: Some(root.clone()),
            ..FirmwareLegacy::default()
        };
        // The first `Activate` loads the list, which fails; Close takes the dialog away.
        page.activate(key());
        settle(&mut page);
        assert_eq!(page.list_state(), "failed");
        page.close_loading();
        let settings = Persisted::at(None);
        page.click(index_of("fwl-sub"), &settings);
        assert_eq!(
            page.message()
                .map(|w| (w.text.as_str(), w.caption.as_str())),
            Some(("Error loading firmware file", "Error"))
        );
        assert!(page.worker.is_none());
        page.dismiss_message();

        page.show_history();
        assert!(page.history_visible);
        assert!(!page.history_label_visible);
        assert_eq!(page.history.options.len(), 70);
        assert_eq!(page.history.text(), "AC 4.0.3 AP 4.0.5 AS 4.0.0");
        assert_eq!(
            page.firmwareurl,
            "https://github.com/diydrones/binary/raw/e94e833a5628f19137d297822343f6282af6d9cc/History/firmware2.xml"
        );
        assert!(page.loading.is_some(), "the history's list loads");
        page.close_loading();
        page.choose_history(2);
        assert_eq!(page.history.text(), "AC 4.0.1 AP 4.0.3");
        assert!(
            page.firmwareurl
                .contains("ef33cbb549fd1a77ea5e23e7ecb52bc6d0f4d915")
        );
        page.close_loading();
        // Choosing what is chosen changes nothing, and loads nothing.
        page.choose_history(2);
        assert!(page.loading.is_none());

        page.beta();
        assert_eq!(
            page.message().map(|w| w.text.as_str()),
            Some(BETA_WARNING.0)
        );
        assert!(page.firmwareurl.contains("ef33cbb"), "not until OK");
        page.dismiss_message();
        assert_eq!(page.firmwareurl, legacy::BETA_URLS);
        assert!(!page.history_visible);
        assert!(page.loading.is_some());
        page.close_loading();

        assert_eq!(page.open_link(FIRMWARE_SITE), Some(FIRMWARE_SITE));
        assert_eq!(page.opened, Some("https://firmware.ardupilot.org/"));
        let _ = std::fs::remove_dir_all(root);
    }

    /// The page object goes with its screen - `firstrun` again - but the list, a static, stays.
    #[test]
    fn a_new_screen_is_a_new_page_object() {
        let root = web("object");
        let mut page = loaded(&root);
        page.deactivate();
        page.tick(key(), false);
        assert!(page.firstrun);
        assert!(page.labels.iter().all(String::is_empty));
        assert_eq!(page.softwares.len(), 12, "softwares is static");
        let _ = std::fs::remove_dir_all(root);
    }

    /// A vehicle clicked runs `findfirmware` on its thread: the question waits for its answer.
    #[test]
    fn a_vehicle_clicked_asks_on_the_flows_thread() {
        let root = web("click");
        let mut page = loaded(&root);
        page.click(index_of("fwl-quad"), &Persisted::at(None));
        assert!(page.worker.is_some());
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while page.question().is_none() && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(5));
            page.tick(key(), true);
        }
        assert_eq!(
            page.question().map(|w| w.text.as_str()),
            Some("Are you sure you want to upload ArduCopter V4.7.1 Quad?")
        );
        page.answer_worker(false);
        while page.worker.is_some() && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(5));
            page.tick(key(), true);
        }
        assert_eq!(page.reached, Some(Reached::default()), "No: nothing more");
        let _ = std::fs::remove_dir_all(root);
    }

    /// Every fact the script asserts on is recorded - here, or by the flows' report the manifest
    /// page shares (`firmware.rs`, `record_reached`) - and each control it clicks is drawn here.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-firmware-legacy.gui");
        let source = include_str!("firmware_legacy.rs");
        let reached = include_str!("firmware.rs");
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.firmware_legacy.") => {
                    let rest = key.trim_start_matches("config.firmware_legacy.");
                    // A picture's label or tag: `config.firmware_legacy.{kind}.{key}`.
                    let picture = rest.split_once('.').is_some_and(|(kind, name)| {
                        source.contains(&format!("\"config.firmware_legacy.{kind}.{{key}}\""))
                            && PICTURES
                                .iter()
                                .any(|picture| picture.id == format!("fwl-{name}"))
                    });
                    assert!(
                        source.contains(&format!("\"{key}\""))
                            || picture
                            || reached.contains(&format!("\"{{prefix}}.{rest}\"")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("fwl-") => {
                    // A drop-down's entry is its box's id and its index.
                    let base = id
                        .rsplit_once('-')
                        .filter(|(_, tail)| tail.chars().all(|c| c.is_ascii_digit()))
                        .map_or(id, |(base, _)| base);
                    assert!(source.contains(&format!("\"{base}\"")), "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts >= 40, "{facts} facts");
        assert!(clicks >= 10, "{clicks} clicks");
        assert!(
            script.contains("\nclick fwl-px4bl\n"),
            "the script clicks Force Bootloader"
        );
    }
}

/// Force Bootloader on the legacy page - `lbl_px4bl_Click`, the manifest page's handler line for
/// line, so the page holds the same [`ForceBootloader`] - over the real link to a scripted copter:
/// the click, what goes on the wire, the box over this page and what is said on the status line.
#[cfg(test)]
mod force_bootloader_tests {
    use std::time::{Duration, Instant};

    use mp_link::ProtocolTimeouts;
    use mp_link::requests::CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    use mp_mavlink_dialects::all::MavMessage;
    use mp_vehicle::VehicleId;

    use super::FirmwareLegacy;
    use crate::config::firmware::NO_BOARD_WRITTEN;
    use crate::config::force_bootloader::{FORCE_FAILED, Force, ForceEnd, IGNORE_THE_UNPLUG};
    use crate::setup::Key;
    use crate::telemetry::scripted::{VEHICLE, Vehicle, until};
    use crate::telemetry::{Telemetry, TelemetryView};

    fn fast() -> ProtocolTimeouts {
        ProtocolTimeouts::default().faster(20)
    }

    /// The key of the window with no link.
    fn idle() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// The page showing, as `Activate` leaves it once its list is in: nothing over it.
    fn page() -> FirmwareLegacy {
        FirmwareLegacy {
            made_for: Some(idle()),
            open: true,
            firstrun: false,
            ..FirmwareLegacy::default()
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

    /// Ticks the page's Force Bootloader until it ends, as the window does once a frame.
    fn until_end(
        page: &mut FirmwareLegacy,
        telemetry: &mut Telemetry,
        now: impl Fn() -> Instant,
    ) -> Option<ForceEnd> {
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            let view = telemetry.view();
            if let Some(end) = page.force.tick(telemetry, &view, now()) {
                return Some(end);
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        None
    }

    /// The click with no link: `Open(false)` opens the port box's serial port at the baud box's
    /// rate as the window's link, waits for the copter's second heartbeat, then `doReboot` for
    /// the next - then 3, 3, 1, 1, and "Please ignore the unplug ..." over this page until its
    /// OK. The handler holds the page, and SETUP, until it is done.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:619-629; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-700, 869-891, 2591-2618, 2758-2763`
    #[test]
    fn the_click_opens_the_port_and_reboots_the_board_into_its_bootloader() {
        let (link, mut vehicle) = Vehicle::link(fast());
        let mut page = page();
        assert!(page.live());
        let mut opened = Vec::new();
        let mut telemetry = page
            .force
            .click(
                &Telemetry::idle().view(),
                "/dev/ttyACM0",
                "115200",
                Instant::now(),
                |url| {
                    opened.push(url.to_owned());
                    Some(link)
                },
            )
            .expect("the window's link, opened");
        assert_eq!(
            opened,
            ["serial:/dev/ttyACM0:115200"],
            "the port box's port at the baud box's rate"
        );
        assert!(telemetry.view().connected, "the link over the port opened");
        assert!(matches!(page.force.state(), Some(Force::Opening { .. })));
        assert!(page.forcing(), "SETUP is not shown again under it");
        assert!(!page.live(), "the handler holds the page");

        let view = telemetry.view();
        assert_eq!(page.force.tick(&mut telemetry, &view, Instant::now()), None);
        assert!(
            matches!(page.force.state(), Some(Force::Opening { .. })),
            "one heartbeat is not `Open` done"
        );
        vehicle.heartbeat();
        until("the second heartbeat", || {
            let view = telemetry.view();
            page.force.tick(&mut telemetry, &view, Instant::now());
            matches!(page.force.state(), Some(Force::Heartbeat { .. }))
        });
        std::thread::sleep(Duration::from_millis(50));
        vehicle.read();
        assert_eq!(reboots(&vehicle), [], "not before the next heartbeat");
        vehicle.heartbeat();
        assert_eq!(
            until_end(&mut page, &mut telemetry, Instant::now),
            Some(ForceEnd::Rebooted)
        );
        until("the four reboots", || {
            vehicle.read();
            reboots(&vehicle).len() >= 4
        });
        std::thread::sleep(fast().command.timeout * 3);
        vehicle.read();
        assert_eq!(reboots(&vehicle), FOUR);

        assert_eq!(
            page.message()
                .map(|waiting| (waiting.text.as_str(), waiting.caption.as_str())),
            Some((IGNORE_THE_UNPLUG, "")),
            "`CustomMessageBox.Show(text)`, over this page"
        );
        assert_eq!(page.take_status(), None, "no failure said");
        assert!(!page.forcing(), "done");
        assert!(!page.live(), "the box is over the page");
        // Its OK, as the box's button does it.
        page.force.dismiss();
        assert!(page.message().is_none());
        assert!(page.live());
    }

    /// The window's link already open: `Open` returns at once and opens nothing; `doReboot`
    /// waits for the next heartbeat, or 2.2 s, and the reboots go out.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:623-627; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-671, 1197-1201, 2594-2614`
    #[test]
    fn the_link_open_opens_nothing_and_reboots() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = page();
        let started = Instant::now();
        let view = telemetry.view();
        let opened = page
            .force
            .click(&view, "/dev/ttyACM0", "115200", started, |url| {
                panic!("{url} opened under an open link")
            });
        assert!(opened.is_none());
        assert!(matches!(page.force.state(), Some(Force::Heartbeat { .. })));
        let later = started + Duration::from_millis(2300);
        assert_eq!(
            until_end(&mut page, &mut telemetry, || later),
            Some(ForceEnd::Rebooted)
        );
        until("the four reboots", || {
            vehicle.read();
            reboots(&vehicle).len() >= 4
        });
        assert_eq!(reboots(&vehicle), FOUR);
        assert_eq!(
            page.message().map(|waiting| waiting.text.as_str()),
            Some(IGNORE_THE_UNPLUG)
        );
    }

    /// Nothing heard before `CONNECT_TIMEOUT_SECONDS`: "Failed to connect and send the reboot
    /// command" on the status line - no box - nothing sent, and the window to close its link.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:630-638; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:791-796`
    #[test]
    fn nothing_heard_in_time_is_failed_to_connect_on_the_status_line() {
        let (link, mut vehicle) = Vehicle::link(fast());
        let mut page = page();
        let started = Instant::now();
        let mut telemetry = page
            .force
            .click(&Telemetry::idle().view(), "COM4", "57600", started, |_| {
                Some(link)
            })
            .expect("opened");
        let view = telemetry.view();
        assert_eq!(
            page.force
                .tick(&mut telemetry, &view, started + Duration::from_secs(31)),
            Some(ForceEnd::Failed)
        );
        assert_eq!(page.take_status().as_deref(), Some(FORCE_FAILED));
        assert_eq!(page.take_status(), None, "said once");
        assert!(page.message().is_none(), "no box");
        assert!(!page.forcing());
        assert!(page.live());
        std::thread::sleep(Duration::from_millis(100));
        vehicle.read();
        assert_eq!(reboots(&vehicle), []);
    }

    /// A port that is no serial port - a network kind, or none - or one that will not open is
    /// the handler's `catch`: "Failed to connect and send the reboot command" on the status line
    /// (a box in the C#), and nothing under way.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:623-638`
    #[test]
    fn a_port_that_will_not_open_is_failed_to_connect_on_the_status_line() {
        let view = Telemetry::idle().view();
        for port in ["TCP", "UDP", ""] {
            let mut page = page();
            let opened = page
                .force
                .click(&view, port, "115200", Instant::now(), |url| {
                    panic!("{url}: no serial port to open")
                });
            assert!(opened.is_none());
            assert_eq!(page.take_status().as_deref(), Some(FORCE_FAILED), "{port}");
            assert!(page.message().is_none(), "no box");
            assert!(!page.forcing());
        }
        let mut page = page();
        let mut tried = 0;
        let opened = page
            .force
            .click(&view, "/dev/ttyUSB7", "115200", Instant::now(), |_| {
                tried += 1;
                None
            });
        assert!(opened.is_none());
        assert_eq!(tried, 1);
        assert_eq!(page.take_status().as_deref(), Some(FORCE_FAILED));
        assert!(!page.forcing());
    }

    /// While MP_FIRMWARE_DEVICE names the device, the legacy page opens no serial port either -
    /// where a real board is - and says why; a network kind is still the handler's own failure.
    #[test]
    fn a_named_device_opens_no_serial_port_from_this_page() {
        let view = Telemetry::idle().view();
        let mut page = page();
        page.force.opens_boards = false;
        let opened = page
            .force
            .click(&view, "/dev/ttyACM0", "115200", Instant::now(), |url| {
                panic!("{url} opened while MP_FIRMWARE_DEVICE names the device")
            });
        assert!(opened.is_none());
        assert_eq!(page.take_status().as_deref(), Some(NO_BOARD_WRITTEN));
        assert!(!page.forcing());
        let opened = page
            .force
            .click(&view, "TCP", "115200", Instant::now(), |url| {
                panic!("{url} opened")
            });
        assert!(opened.is_none());
        assert_eq!(page.take_status().as_deref(), Some(FORCE_FAILED));
    }

    /// Leaving the page stops what is under way and closes its box, as leaving the manifest page
    /// does.
    #[test]
    fn leaving_the_page_stops_it_and_closes_its_box() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(fast());
        let mut page = page();
        let view = telemetry.view();
        page.force.start(false, &view, Instant::now());
        page.deactivate();
        assert!(!page.forcing());
        vehicle.heartbeat();
        let view = telemetry.view();
        assert_eq!(page.force.tick(&mut telemetry, &view, Instant::now()), None);

        let mut page = self::page();
        let view = telemetry.view();
        page.force.start(true, &view, Instant::now());
        let later = Instant::now() + Duration::from_millis(2300);
        assert_eq!(
            until_end(&mut page, &mut telemetry, || later),
            Some(ForceEnd::Rebooted)
        );
        assert!(page.message().is_some());
        page.deactivate();
        assert!(page.message().is_none(), "the box goes with the page");
    }

    /// The link Force Bootloader opened moves SETUP's key without showing the screen again, so
    /// the page object is kept under the new key - leaving the page and coming back finds the
    /// same one, its labels and all - where a new screen would be a new object.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:623; MainV2.cs:1419-1425, 1740-1748`
    #[test]
    fn the_page_object_is_kept_under_the_link_it_opened() {
        let mut view = TelemetryView::disconnected("serial:/dev/ttyACM0:115200");
        view.connected = true;
        view.vehicle = Some(VehicleId::new(1, 1));
        let heard = Key::of(&view);
        assert_ne!(heard, idle());

        let mut kept = page();
        kept.labels[0] = "ArduPlane Stable".to_owned();
        kept.rekey(heard);
        kept.deactivate();
        kept.tick(heard, true);
        assert_eq!(kept.labels[0], "ArduPlane Stable", "the same page object");
        assert!(!kept.firstrun);

        // Not kept: the screen shown again for the link is a new page object.
        let mut shown_again = page();
        shown_again.labels[0] = "ArduPlane Stable".to_owned();
        shown_again.deactivate();
        shown_again.tick(heard, true);
        assert!(shown_again.labels[0].is_empty());
        assert!(shown_again.firstrun);

        // A page object not made is not made by it.
        let mut unmade = FirmwareLegacy::default();
        unmade.rekey(heard);
        assert_eq!(unmade.made_for, None);
    }
}
