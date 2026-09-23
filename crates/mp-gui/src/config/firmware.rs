//! Install Firmware: the first page of Initial Setup, without the flashing.
//!
//! Mission Planner lists two pages under that name and shows one: `ConfigFirmwareManifest` while
//! no vehicle is connected, `ConfigFirmwareDisabled` while one is (`GCSViews/InitialSetup.cs:
//! 165-174`, the legacy `ConfigFirmware` beside them being the older `firmware2.xml` catalogue).
//! Which one is decided when the page opens, as the C# decides it when it builds the list.
//!
//! `ConfigFirmwareManifest` is eleven vehicle pictures - Rover, Plane, Quad, Hexa, Octa Quad and
//! Sub above, Antenna Tracker, Heli, Tri, Y6 and Octa below, and a picture of a Cube that does
//! nothing - with a progress bar and a status line under them, and a row of links: All Options,
//! Bootloader Update, Force Bootloader, Load custom firmware and Beta firmwares. `Activate`
//! disables the pictures and the Beta link, fetches the firmware catalogue (`mp_firmware::
//! manifest`) on a background task, and labels each picture with the newest firmware of its
//! vehicle in the release being offered - `OFFICIAL` until Beta firmwares is clicked - enabling
//! it as it goes. Clicking a picture asks "Are you sure you want to upload ...?" and then
//! `LookForPort` finds the board among the USB devices, picks its firmware - one file outright,
//! several through `FirmwareSelection`, which starts at the board's own platform - downloads it
//! and flashes it.
//!
//! Here, clicking a picture runs `LookForPort` up to the download and stops: the board it found,
//! the firmware it would take, its version and its URL are shown beneath the page, and an Upload
//! button that is disabled says why. Nothing on this page writes to a board. The confirmation is
//! not asked, because nothing it would confirm follows it. The layout is
//! `ConfigFirmwareManifest.Designer.cs`'s - every control at its `Location` and `Size` in a
//! 946 x 375 page - with names in boxes where the `.resx` has pictures this application does not
//! carry, and this application's colours.
//!
//! What is not here, and why:
//!
//! * the upload, All Options, Bootloader Update, Force Bootloader and Load custom firmware: each
//!   ends in a write to a board or a vehicle - a flash, `MAV_CMD_FLASH_BOOTLOADER`, a reboot into
//!   the bootloader - and flashing is not enabled in this build. They are drawn, dimmed;
//! * `Ctrl+Q` for the `DEV` release (`ProcessCmdKey`, `ConfigFirmwareManifest.cs:399-408`), as
//!   Flight Modes leaves out its `Ctrl+S`; the catalogue answers for `DEV`, and `mpr firmware
//!   list --release DEV` asks it;
//! * the board id a bootloader reports when a device is plugged in while the page shows
//!   (`Instance_DeviceChanged`, `:134-180`): reading it means opening every port and sending the
//!   bootloader's identify. The board is found from the USB product string and ids alone, as
//!   `LookForPort` does when that probe has found nothing.
//!
//! The device list is the port enumeration through `mp_firmware::detect::DeviceInfo::from_port`.
//! [`DEVICE_ENV`] replaces it with one named device, so a test does not depend on what is plugged
//! into the machine running it; with [`mp_firmware::manifest::OVERRIDE_ENV`] a test runs offline.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::Arc;
use std::sync::mpsc::{Receiver, TryRecvError, channel};

use gpui::{AnyElement, Context, Div, SharedString, div, prelude::*, px, rgb};
use mp_firmware::detect::DeviceInfo;
use mp_firmware::manifest::{
    self, Lookup, Manifest, MavType, NO_FIRMWARE, NO_PORT, Outcome, ReleaseType, icon_name,
};

use crate::MissionPlanner;
use crate::telemetry::TelemetryView;
use crate::ui::{action, panel, theme};

/// Test scaffolding: the device list is this one device - its USB product string, and optionally
/// `|` and a hardware id - instead of the machine's ports.
pub const DEVICE_ENV: &str = "MP_FIRMWARE_DEVICE";

/// What the Upload button says for itself.
pub const NOT_ENABLED: &str = "flashing is not enabled in this build";

/// `FirmwareSelection`'s heading, shown when several files fit the board.
/// `// C#: test/FirmwareSelection.xaml:7`
const MORE_THAN_ONE: &str = "More than one choice exists. Please filter down to the desired \
                             selection.";

/// `ConfigFirmwareDisabled.resx`'s `label1.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.resx:134-137`
const DISABLED_TEXT: &str = "You cannot load new firmware while connected via MAVLink. \n\n\
                             Please press the Disconnect button at top right to end the current \
                             MAVLink session and enable the firmware loading screen.";

/// `ConfigFirmwareDisabled.resx`'s `label2.Text`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.resx:187-188`
const DISABLED_BOOTLOADER: &str = "Update the bootloader on your hardware to the newest available.";

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
    /// What the picture shows, written in its place.
    pub name: &'static str,
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
    name: &'static str,
    mav_type: MavType,
    x: f32,
    y: f32,
) -> Picture {
    Picture {
        control,
        id,
        name,
        mav_type,
        x,
        y,
    }
}

/// The pictures, in the order `Activate` labels them - which matters: it stops at the first
/// vehicle the release lacks (`First` throws), leaving that one and the rest unlabelled.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:32-42, 76-92; ConfigFirmwareManifest.Designer.cs:126-260`
pub const PICTURES: [Picture; 11] = [
    picture(
        "pictureAntennaTracker",
        "fw-tracker",
        "Antenna Tracker",
        MavType::AntennaTracker,
        3.0,
        159.0,
    ),
    picture(
        "pictureBoxHeli",
        "fw-heli",
        "Heli",
        MavType::Helicopter,
        159.0,
        159.0,
    ),
    picture(
        "pictureBoxSub",
        "fw-sub",
        "Sub",
        MavType::Submarine,
        783.0,
        3.0,
    ),
    picture(
        "pictureBoxRover",
        "fw-rover",
        "Rover",
        MavType::GroundRover,
        3.0,
        3.0,
    ),
    picture(
        "pictureBoxOctaQuad",
        "fw-octaquad",
        "Octa Quad",
        MavType::Copter,
        627.0,
        3.0,
    ),
    picture(
        "pictureBoxOcta",
        "fw-octa",
        "Octa",
        MavType::Copter,
        627.0,
        159.0,
    ),
    picture("pictureBoxY6", "fw-y6", "Y6", MavType::Copter, 471.0, 159.0),
    picture(
        "pictureBoxTri",
        "fw-tri",
        "Tri",
        MavType::Copter,
        315.0,
        159.0,
    ),
    picture(
        "pictureBoxHexa",
        "fw-hexa",
        "Hexa",
        MavType::Copter,
        471.0,
        3.0,
    ),
    picture(
        "pictureBoxQuad",
        "fw-quad",
        "Quad",
        MavType::Copter,
        315.0,
        3.0,
    ),
    picture(
        "pictureBoxPlane",
        "fw-plane",
        "Plane",
        MavType::FixedWing,
        159.0,
        3.0,
    ),
];

/// What the background fetch sends back: the manifest, if one was had, and what went wrong.
type Fetched = (Option<Manifest>, Vec<String>);

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

    /// `Deactivate`: back to the official release for next time.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:124-132`
    pub fn close(&mut self) {
        self.open = false;
        self.release = ReleaseType::Official;
        self.picked = None;
    }

    /// Once a frame: takes a finished fetch, and closes the page when the screen changes, as
    /// leaving Initial Setup deactivates its page.
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

    /// A picture clicked: `LookForPort` for its vehicle, over the devices as they are now, up to
    /// the download.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:182-245`
    pub fn pick(&mut self, index: usize) {
        if !self.enabled.get(index).copied().unwrap_or(false) {
            return;
        }
        let (Some(manifest), Some(picture)) = (self.manifest.clone(), PICTURES.get(index)) else {
            return;
        };
        self.devices = devices();
        let lookup =
            manifest.look_for_port(&self.devices, None, picture.mav_type, self.release, false);
        self.picked = Some((index, lookup));
    }

    /// Beta firmwares: the beta release, and `Activate` again.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:393-397`
    pub fn beta(&mut self) {
        if !self.links_enabled {
            return;
        }
        self.release = ReleaseType::Beta;
        self.activate();
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

    /// The message box the C# would show after the last click, if any.
    fn message(&self) -> Option<&'static str> {
        match &self.picked {
            Some((_, None)) => Some(NO_PORT),
            Some((_, Some(lookup))) if lookup.outcome() == Outcome::NoFirmware => Some(NO_FIRMWARE),
            _ => None,
        }
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
}

/// The USB devices, as `LookForPort` lists them: the ports, each through
/// `DeviceInfo::from_port`; or the one [`DEVICE_ENV`] names.
fn devices() -> Vec<DeviceInfo> {
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

/// Facts for a test: which page, the catalogue, the release, the labels, the board, and what a
/// click chose.
pub fn record_facts(page: &InstallFirmware) {
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
        "config.firmware.message",
        page.message()
            .map_or_else(|| "none".to_owned(), |text| text.replace("\r\n", " ")),
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

/// The page, while it is open.
pub fn page(firmware: &InstallFirmware, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if firmware.connected {
        return disabled_page().into_any_element();
    }
    let mut body = div().relative().w(px(PAGE.0)).h(px(PAGE.1));
    for (index, picture) in PICTURES.iter().enumerate() {
        body = body.child(image_label(firmware, index, picture, cx));
    }
    // `imageLabel1`: a picture of a Cube, with no Click handler.
    body = body.child(
        at(783.0, 159.0, PICTURE, PICTURE)
            .flex()
            .items_center()
            .justify_center()
            .rounded_md()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .text_sm()
            .text_color(rgb(theme::DIM))
            .child("Pixhawk 2 Cube"),
    );
    // `progress`, at zero: nothing is downloaded or flashed here.
    body = body.child(
        at(3.0, 315.0, 940.0, 23.0)
            .rounded_sm()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::BG)),
    );
    // `lbl_status`, with its Designer text: nothing here reports progress.
    body = body.child(
        at(3.0, 341.0, 450.0, 34.0)
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child("Status"),
    );
    // The links along the bottom. Only Beta firmwares does anything here.
    for (x, width, text) in [
        (459.0, 57.0, "All Options"),
        (522.0, 96.0, "Bootloader Update"),
        (624.0, 88.0, "Force Bootloader"),
        (736.0, 110.0, "Load custom firmware"),
    ] {
        body = body.child(
            at(x, 341.0, width + 4.0, 13.0)
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(text),
        );
    }
    // At the label's .resx position, sized by its text rather than the label's 80 x 13 box: the
    // text does not fit that box at this font, wrapped below it, and a click on the text then
    // landed outside the 13 px hitbox - tests/gui/config-firmware.gui found the link dead.
    let beta = crate::probe::measured("fw-beta", div().absolute().left(px(867.0)).top(px(341.0)))
        .id("fw-beta")
        .whitespace_nowrap()
        .text_xs()
        .child("Beta firmwares");
    body = body.child(if firmware.links_enabled {
        beta.text_color(rgb(theme::ACCENT))
            .cursor_pointer()
            .hover(|style| style.underline())
            .on_click(cx.listener(|this, _event, _window, cx| {
                this.install_firmware.beta();
                cx.notify();
            }))
            .into_any_element()
    } else {
        beta.text_color(rgb(theme::DIM)).into_any_element()
    });

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

/// One `ImageLabel`: the vehicle's name where its picture would be, the label under it, and the
/// click - inert until `Activate` has enabled it.
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
        .child(
            div()
                .flex_1()
                .flex()
                .items_center()
                .justify_center()
                .text_lg()
                .text_color(rgb(colour))
                .child(picture.name),
        )
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
                this.install_firmware.pick(index);
                cx.notify();
            }))
            .into_any_element()
    } else {
        body.into_any_element()
    }
}

/// Beneath the page: the board, what a click chose, and the Upload button that stays disabled.
fn choice(firmware: &InstallFirmware) -> impl IntoElement {
    let line = |label: &'static str, value: String, colour: u32| {
        div()
            .flex()
            .gap_2()
            .text_xs()
            .child(div().w(px(70.0)).text_color(rgb(theme::DIM)).child(label))
            .child(div().text_color(rgb(colour)).child(value))
    };
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
        if let Some(message) = firmware.message() {
            column = column.child(line("", message.replace("\r\n", " "), theme::ALERT));
        }
        if let Some(lookup) = lookup {
            if let Outcome::Choose(selection) = lookup.outcome()
                && selection.selected().is_none()
            {
                column = column.child(line("", MORE_THAN_ONE.to_owned(), theme::WARN));
                for result in selection.results() {
                    column = column.child(line("", result, theme::DIM));
                }
            }
            if let Some(chosen) = lookup.chosen() {
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
    }

    column.child(
        div()
            .flex()
            .items_center()
            .gap_2()
            .pt_1()
            .child(action(
                "fw-upload",
                "Upload",
                theme::ACCENT,
                false,
                |_, _, _| {},
            ))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(NOT_ENABLED),
            ),
    )
}

/// `ConfigFirmwareDisabled`: the explanation, and a Bootloader Update button that is disabled
/// here because it sends `MAV_CMD_FLASH_BOOTLOADER`.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmwareDisabled.resx:121-206; ConfigFirmwareDisabled.cs:18-45`
fn disabled_page() -> impl IntoElement {
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
            false,
            |_, _, _| {},
        )))
        .child(
            at(156.0, 155.0, 313.0, 13.0)
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(DISABLED_BOOTLOADER),
        );
    panel("install firmware", body)
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
        page.pick(index_of("fw-quad"));
        assert!(page.picked.is_none());
    }

    #[test]
    fn a_click_chooses_the_boards_firmware_and_beta_changes_it() {
        let mut page = loaded("CubeOrange-BL");
        page.devices = vec![DeviceInfo::new("CubeOrange-BL", "")];
        // `pick` re-enumerates; the enumeration here is DEVICE_ENV or the machine's ports, so
        // this test takes the lookup `pick` makes by hand from the page's own devices.
        let manifest = page.manifest.clone().unwrap();
        let quad = index_of("fw-quad");
        page.picked = Some((
            quad,
            manifest.look_for_port(
                &page.devices,
                None,
                PICTURES[quad].mav_type,
                page.release,
                false,
            ),
        ));
        let chosen = page.lookup().and_then(Lookup::chosen).expect("chosen");
        assert_eq!(
            chosen.url.as_deref(),
            Some("https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj")
        );
        assert_eq!(page.message(), None);

        page.beta();
        assert_eq!(page.release, ReleaseType::Beta);
        assert!(page.picked.is_none(), "Activate again forgets the click");
        assert_eq!(page.labels[quad], "Copter V4.7.1 BETA");

        page.close();
        assert_eq!(page.release, ReleaseType::Official, "Deactivate resets it");
        assert!(!page.is_open());
    }

    #[test]
    fn no_board_and_no_firmware_are_messages() {
        let mut page = loaded("FT232R USB UART");
        assert!(page.detected.is_none());
        page.picked = Some((index_of("fw-quad"), None));
        assert_eq!(page.message(), Some(NO_PORT));
        let manifest = page.manifest.clone().unwrap();
        page.picked = Some((
            index_of("fw-quad"),
            manifest.look_for_port(
                &[DeviceInfo::new("f103-GPS-BL", "")],
                None,
                MavType::Copter,
                ReleaseType::Official,
                false,
            ),
        ));
        assert_eq!(page.message(), Some(NO_FIRMWARE));
    }

    #[test]
    fn a_connected_vehicle_gets_the_disabled_page_and_no_fetch() {
        let mut page = InstallFirmware::default();
        page.open(true);
        assert!(page.is_open());
        assert_eq!(page.fetches, 0);
        assert!(page.receiver.is_none());
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
}
