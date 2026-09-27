//! Force Bootloader, on both Install Firmware pages. The legacy page's `lbl_px4bl_Click` and the
//! manifest page's `Lbl_px4bl_Click` are the same handler, line for line:
//! `MainV2.comPort.Open(false)`; the port open, `doReboot(true, false)` and "Please ignore the
//! unplug and plug back in when uploading flight firmware."; anything else "Failed to connect and
//! send the reboot command". So there is one of it here, [`ForceBootloader`], of which each page
//! holds its own - as each C# page has its handler - and the window drives the one clicked
//! ([`MissionPlanner::force_bootloader_clicked`], [`MissionPlanner::force_bootloader_tick`]).
//!
//! The failure is a box in the C#; here it goes on the status line - the owner's ruling of
//! 2026-09-25, no box for an error the window can show. The instruction stays a box, over the page
//! that was clicked, as it is an instruction. While [`DEVICE_ENV`] names the device no serial
//! port is opened ([`ForceBootloader::refuses_port`]), so no script reboots the board plugged into
//! the machine running it.
//! `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:619-639; GCSViews/ConfigurationView/ConfigFirmwareManifest.cs:513-533`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::time::Instant;

use mp_link::Link;

use super::firmware::{
    CONNECT_TIMEOUT, DEVICE_ENV, HEARTBEAT_WAIT, InstallFirmware, NO_BOARD_WRITTEN, Waiting,
    heard_enough, heartbeats,
};
use super::firmware_legacy::FirmwareLegacy;
use crate::MissionPlanner;
use crate::telemetry::{Report, Telemetry, TelemetryView};

/// Force Bootloader's instruction once the reboot has gone out: a box, as the C#'s.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:628; ConfigFirmwareManifest.cs:522`
pub const IGNORE_THE_UNPLUG: &str =
    "Please ignore the unplug and plug back in when uploading flight firmware.";

/// Force Bootloader's failure: a box in the C#, the status line here - the owner's ruling of
/// 2026-09-25, no box for an error the window can show.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:637; ConfigFirmwareManifest.cs:531`
pub const FORCE_FAILED: &str = "Failed to connect and send the reboot command";

/// Force Bootloader after its click: `MainV2.comPort.Open(false)` waiting for the vehicle, then
/// `doReboot(true, false)` waiting for its next heartbeat.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:619-639; ConfigFirmwareManifest.cs:513-533`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Force {
    /// `Open`'s connect loop: a vehicle heard twice (four times when it is not component 1)
    /// before the deadline.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:769-894`
    Opening {
        /// `CONNECT_TIMEOUT_SECONDS` after the click.
        deadline: Instant,
    },
    /// `doReboot(true, false)`'s `getHeartBeat`: the heartbeat after `seen`, or 2.2 s.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2591-2605`
    Heartbeat {
        /// The vehicle's heartbeats when it began.
        seen: u64,
        /// When `getHeartBeat` gives up.
        deadline: Instant,
    },
}

/// How Force Bootloader ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForceEnd {
    /// The reboots went out (or there was no vehicle to send them to, which the C# does not
    /// check): "Please ignore the unplug ..." shows.
    Rebooted,
    /// `Open` found no vehicle in time and closed the port: [`FORCE_FAILED`].
    Failed,
}

/// One page's Force Bootloader: the handler's steps over the window's link, its box, and what it
/// has to say on the status line.
#[derive(Debug)]
pub struct ForceBootloader {
    /// Where it has got, while under way.
    state: Option<Force>,
    /// "Please ignore the unplug ...", showing until its OK.
    instruction: Option<Waiting>,
    /// What it has to say on the window's status line.
    status: Option<String>,
    /// Whether it may open a serial port - where a real board is - to reboot it: not while
    /// [`DEVICE_ENV`] names the device, as no upload is then made either ([`NO_BOARD_WRITTEN`]).
    /// The manifest page's Bootloader Update asks the same ([`InstallFirmware::refuses_port`]).
    pub(super) opens_boards: bool,
}

impl Default for ForceBootloader {
    fn default() -> Self {
        Self {
            state: None,
            instruction: None,
            status: None,
            opens_boards: std::env::var_os(DEVICE_ENV).is_none(),
        }
    }
}

impl ForceBootloader {
    /// Whether it is under way over the window's link, which its `MainV2.comPort.Open(false)`
    /// opened or found open: while it is, the link opening or closing shows no screen again
    /// (`MissionPlanner::backstage_tick`, [`holds_setup`]) - `Open` is not `doConnect`, whose end
    /// shows the screen again, nor a disconnect.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:623; ConfigFirmwareManifest.cs:517; MainV2.cs:1115-1133, 1419-1425, 1740-1748; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-700, 809-814`
    #[must_use]
    pub const fn forcing(&self) -> bool {
        self.state.is_some()
    }

    /// Where it has got, while under way, for a test.
    #[cfg(test)]
    #[must_use]
    pub const fn state(&self) -> Option<Force> {
        self.state
    }

    /// Whether it holds its page: under way, as the C#'s handler holds the UI thread until it
    /// returns, or its box showing, which is modal.
    #[must_use]
    pub const fn busy(&self) -> bool {
        self.state.is_some() || self.instruction.is_some()
    }

    /// Whether it refuses the port the port box names: a serial port - where a board is - while
    /// [`DEVICE_ENV`] names the device. A network kind, the SITL's, is not refused.
    #[must_use]
    pub fn refuses_port(&self, port: &str) -> bool {
        !self.opens_boards
            && !port.is_empty()
            && crate::connect::kind(port) == crate::connect::Kind::Serial
    }

    /// The click, over the window's link as `view` shows it: `MainV2.comPort.Open(false)` - which
    /// returns at once when the link is open, else opens the port box's serial port at the baud
    /// box's rate through `connect` - then its steps ([`Self::tick`]). Returns the link it opened,
    /// for the window to take as its own. A port refused ([`Self::refuses_port`]) is
    /// [`NO_BOARD_WRITTEN`] on the status line; one that will not open is [`FORCE_FAILED`].
    ///
    /// The C# opens `comPort.BaseStream` as it was last configured: the saved port at start-up,
    /// the port last connected after that, at the baud box's rate. That is the port box's port
    /// unless it was changed without connecting, which here opens the port the box shows. Its
    /// stream after a network connect is that network link, reopened without its questions;
    /// here only a serial port is opened - the port is where the board's bootloader will
    /// appear - and a network kind in the box is [`FORCE_FAILED`], as the C#'s start-up
    /// `SerialPort` named "TCP" fails to open. `Open` records nothing (`doConnect` makes the
    /// logs), and neither does this.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:619-639; ConfigFirmwareManifest.cs:513-533; MainV2.cs:726-727, 781-792, 1561-1570, 4352-4358; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-700`
    pub fn click(
        &mut self,
        view: &TelemetryView,
        port: &str,
        baud: &str,
        now: Instant,
        connect: impl FnOnce(&str) -> Option<Link>,
    ) -> Option<Telemetry> {
        let open = view.connected && !view.target.starts_with("file:");
        if open {
            self.start(true, view, now);
            return None;
        }
        if self.refuses_port(port) {
            self.status = Some(NO_BOARD_WRITTEN.to_owned());
            return None;
        }
        let serial = !port.is_empty() && crate::connect::kind(port) == crate::connect::Kind::Serial;
        let url = format!("serial:{port}:{baud}");
        let Some(link) = serial.then(|| connect(&url)).flatten() else {
            // `catch`: the box, here the status line.
            self.status = Some(FORCE_FAILED.to_owned());
            return None;
        };
        let telemetry = Telemetry::over(link, &url);
        self.start(false, &telemetry.view(), now);
        Some(telemetry)
    }

    /// Under way once the window has opened its link, or found it open: `Open`'s wait for the
    /// vehicle, or - the link already open, as `Open` then returns at once - `doReboot`'s wait
    /// for its next heartbeat.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:623-627; ConfigFirmwareManifest.cs:517-521; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-671`
    pub fn start(&mut self, already_open: bool, view: &TelemetryView, now: Instant) {
        self.state = Some(if already_open {
            Force::Heartbeat {
                seen: heartbeats(view),
                deadline: now + HEARTBEAT_WAIT,
            }
        } else {
            Force::Opening {
                deadline: now + CONNECT_TIMEOUT,
            }
        });
    }

    /// The next step, once a frame over the window's link. `Open` done - the vehicle heard
    /// enough - is `doReboot(true, false)`'s wait for the next heartbeat, and then its reboots:
    /// `PREFLIGHT_REBOOT_SHUTDOWN` with param1 3 and then 1, each written twice
    /// (`Telemetry::command`, as `reboot_to_bootloader` sends them), to the vehicle `Open` chose;
    /// and the instruction's box, whether or not there was a vehicle, as the C# does not look at
    /// `doReboot`'s answer. `Open` finding nothing in time closes the link: [`FORCE_FAILED`] on
    /// the status line, and [`ForceEnd::Failed`] for the window to close its link.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:623-638; ConfigFirmwareManifest.cs:517-532; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:791-796, 2591-2618, 2758-2763`
    pub fn tick(
        &mut self,
        telemetry: &mut Telemetry,
        view: &TelemetryView,
        now: Instant,
    ) -> Option<ForceEnd> {
        match self.state? {
            Force::Opening { deadline } => {
                if heard_enough(view) {
                    self.state = Some(Force::Heartbeat {
                        seen: heartbeats(view),
                        deadline: now + HEARTBEAT_WAIT,
                    });
                    None
                } else if now >= deadline || telemetry.error().is_some() || !view.connected {
                    self.state = None;
                    self.status = Some(FORCE_FAILED.to_owned());
                    Some(ForceEnd::Failed)
                } else {
                    None
                }
            }
            Force::Heartbeat { seen, deadline } => {
                if heartbeats(view) <= seen && now < deadline {
                    return None;
                }
                self.state = None;
                // `if (MAV.sysid != 0 && MAV.compid != 0)`.
                if let Some(id) = view.vehicle.filter(|id| id.sysid != 0 && id.compid != 0) {
                    for message in [
                        mp_link::commands::reboot_to_bootloader(id),
                        mp_link::commands::reboot(id),
                    ] {
                        telemetry.command_message(&message, Report::default());
                    }
                }
                // `CustomMessageBox.Show(text)`: its caption empty.
                self.instruction = Some(Waiting {
                    text: IGNORE_THE_UNPLUG.to_owned(),
                    caption: String::new(),
                    buttons: None,
                });
                Some(ForceEnd::Rebooted)
            }
        }
    }

    /// The instruction's box, while it shows.
    #[must_use]
    pub const fn message(&self) -> Option<&Waiting> {
        self.instruction.as_ref()
    }

    /// The box's OK.
    pub fn dismiss(&mut self) {
        self.instruction = None;
    }

    /// What it has to say on the status line, once.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// The page deactivated: what was under way stops and its box closes, as the page's other
    /// boxes close with it - the C#'s handler holds the window until it ends, so nothing of it
    /// outlives the page there.
    pub fn cancel(&mut self) {
        self.state = None;
        self.instruction = None;
    }

    /// Where it has got, for a test.
    #[must_use]
    pub const fn fact(&self) -> &'static str {
        match self.state {
            None => "none",
            Some(Force::Opening { .. }) => "opening",
            Some(Force::Heartbeat { .. }) => "heartbeat",
        }
    }
}

/// Which Install Firmware page's Force Bootloader.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Page {
    /// `ConfigFirmwareManifest`'s `lbl_px4bl`.
    Manifest,
    /// `ConfigFirmware`'s `lbl_px4bl`.
    Legacy,
}

impl Page {
    /// Both.
    pub const ALL: [Self; 2] = [Self::Manifest, Self::Legacy];
}

/// Whether either page's Force Bootloader holds the window's link, so SETUP is not shown again
/// as it opens or closes ([`ForceBootloader::forcing`]).
#[must_use]
pub fn holds_setup(manifest: &InstallFirmware, legacy: &FirmwareLegacy) -> bool {
    manifest.forcing() || legacy.forcing()
}

impl MissionPlanner {
    /// `lbl_px4bl_Click` on the legacy page, `Lbl_px4bl_Click` on the manifest page: the page's
    /// [`ForceBootloader::click`] over the window's link and the port and baud boxes, taking the
    /// link it opens - with no parameter list and no mission asked for, as `Open(false)` asks for
    /// none (`loadwpsonconnect` is `doConnect`'s).
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:619-639; ConfigFirmwareManifest.cs:513-533; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-700`
    pub(crate) fn force_bootloader_clicked(&mut self, page: Page) {
        let live = match page {
            Page::Manifest => self.install_firmware.live(),
            Page::Legacy => self.firmware_legacy.live(),
        };
        if !live {
            return;
        }
        let view = self.telemetry.view();
        let force = match page {
            Page::Manifest => self.install_firmware.force_bootloader(),
            Page::Legacy => self.firmware_legacy.force_bootloader(),
        };
        let opened = force.click(
            &view,
            &self.connect_box.port,
            &self.connect_box.baud,
            Instant::now(),
            |url| Link::connect(url, mp_link::LinkConfig::default()).ok(),
        );
        if let Some(telemetry) = opened {
            self.telemetry = telemetry;
            // `Open(false)`: no parameter list, and no mission - reading it on connect is
            // `doConnect`'s (`loadwpsonconnect`), not `Open`'s.
            self.params_requested = true;
            self.mission_requested = true;
        }
    }

    /// Once a frame: the steps of whichever page's Force Bootloader is under way, over the
    /// window's link. `Open` failing closes the window's link, as `Open` closes the port at its
    /// deadline - which shows no screen again either, so SETUP's list and the legacy page object
    /// are kept, keyed to the link closed.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:791-796`
    pub(crate) fn force_bootloader_tick(&mut self, now: Instant) {
        for page in Page::ALL {
            let force = match page {
                Page::Manifest => self.install_firmware.force_bootloader(),
                Page::Legacy => self.firmware_legacy.force_bootloader(),
            };
            if !force.forcing() {
                continue;
            }
            // The window's view - the mission copied with it - only while one is under way,
            // not every frame of every screen.
            let view = self.telemetry.view();
            if force.tick(&mut self.telemetry, &view, now) == Some(ForceEnd::Failed) {
                self.telemetry = Telemetry::idle();
                let key = crate::setup::Key::of(&self.telemetry.view());
                self.setup_list.rekey(key);
                self.firmware_legacy.rekey(key);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The hold is either page's: SETUP is not shown again under the legacy page's Force
    /// Bootloader any more than under the manifest page's.
    #[test]
    fn either_pages_force_bootloader_holds_setup() {
        let view = TelemetryView::disconnected("test");
        let mut manifest = InstallFirmware::default();
        let mut legacy = FirmwareLegacy::default();
        assert!(!holds_setup(&manifest, &legacy));
        legacy
            .force_bootloader()
            .start(false, &view, Instant::now());
        assert!(holds_setup(&manifest, &legacy), "the legacy page's");
        legacy.force_bootloader().cancel();
        assert!(!holds_setup(&manifest, &legacy));
        manifest
            .force_bootloader()
            .start(false, &view, Instant::now());
        assert!(holds_setup(&manifest, &legacy), "the manifest page's");
    }

    /// While MP_FIRMWARE_DEVICE names the device no serial port is opened - where a real board
    /// is, the bench CubeOrange among them - and a network kind, the SITL's, is not refused;
    /// with the machine's own devices, every port may be.
    #[test]
    fn a_named_device_keeps_it_off_the_machines_serial_ports() {
        let mut force = ForceBootloader {
            opens_boards: false,
            ..ForceBootloader::default()
        };
        assert!(force.refuses_port("/dev/ttyACM0"));
        assert!(force.refuses_port("COM4"));
        assert!(!force.refuses_port("TCP"));
        assert!(!force.refuses_port("UDP"));
        assert!(
            !force.refuses_port(""),
            "nothing to open is the C#'s own failure"
        );
        force.opens_boards = true;
        assert!(!force.refuses_port("/dev/ttyACM0"));
    }
}
