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

//! Mission Planner's window-wide keys, `MainV2.ProcessCmdKey`: the F keys that switch screens
//! and connect, and the Control keys that open the tool forms.
//!
//! WinForms gives `ProcessCmdKey` a key after the focused control and its parents have had it,
//! so a page's own keys (the parameter grid's Ctrl+S and F2, Install Firmware's Ctrl+Q) come
//! first; here the main window's root takes a key on its way out, after every element inside
//! has had it and none took it.
//!
//! * `if (ConfigTerminal.SSHTerminal) return false;` has nothing to test: the Terminal page and
//!   its SSH session are out of scope (the ledger's ConfigTerminal.cs and SSHTerminal.cs rows).
//! * A key compares as `keyData ==` does: the F keys with no modifier, the letters with Control
//!   and nothing else. Control on every platform, as Mission Planner under mono on a Mac.
//!
//! `// C#: MainV2.cs:4067-4182`

use gpui::{Context, Keystroke, Window};

use crate::{MissionPlanner, Screen};

/// `FlightData.ProcessCmdKey`'s Control and a digit: the tab's place, Ctrl+1 the first and
/// Ctrl+0 the tenth. `// C#: GCSViews/FlightData.cs:865-916`
#[must_use]
pub(crate) fn fly_tab_key(keystroke: &Keystroke) -> Option<usize> {
    let modifiers = &keystroke.modifiers;
    if !modifiers.control || modifiers.alt || modifiers.shift || modifiers.platform {
        return None;
    }
    match keystroke.key.as_str() {
        "0" => Some(9),
        digit => digit
            .parse::<usize>()
            .ok()
            .filter(|n| (1..=9).contains(n))
            .map(|n| n - 1),
    }
}

/// Ctrl+Y's message once the command has been answered or refused, and when it went unanswered
/// or there was no link to send it on (`doCommand`'s exception). On the status line, as the
/// owner ruled for such boxes. `// C#: MainV2.cs:4158-4174`
const STORAGE_WRITE_DONE: &str = "Done MAV_ACTION_STORAGE_WRITE";

/// What each of `ProcessCmdKey`'s keys does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CmdKey {
    /// F12: `MenuConnect_Click`.
    Connect,
    /// F2: `MenuFlightData_Click`.
    FlightData,
    /// F3: `MenuFlightPlanner_Click`.
    FlightPlanner,
    /// F4: `MenuTuning_Click`.
    Tuning,
    /// F5: `comPort.getParamList()` and the current screen shown again.
    RefreshParams,
    /// Ctrl+F: `new temp().Show()`.
    Temp,
    /// Ctrl+P: `new PluginUI().Show()`.
    PluginManager,
    /// Ctrl+G: `new SerialOutputNMEA().Show()`.
    NmeaOut,
    /// Ctrl+X: `new GMAPCache().ShowUserControl()`.
    MapCache,
    /// Ctrl+L: `new SpectrogramUI().Show()`.
    Spectrogram,
    /// Ctrl+W: `new PropagationSettings().Show()`.
    Propagation,
    /// Ctrl+Z: `new Camera().test(comPort)`.
    CameraTest,
    /// Ctrl+T: `comPort.Open(false)`.
    OverrideConnect,
    /// Ctrl+Y: `PREFLIGHT_STORAGE` 1, then "Done MAV_ACTION_STORAGE_WRITE".
    StorageWrite,
    /// Ctrl+J: `new DevopsUI().ShowUserControl()`.
    Devops,
}

/// The key `keystroke` is to `ProcessCmdKey`, if any.
/// `// C#: MainV2.cs:4073-4177`
#[must_use]
pub(crate) fn cmd_key(keystroke: &Keystroke) -> Option<CmdKey> {
    let modifiers = &keystroke.modifiers;
    // The fn key a laptop's F row may report is not one `Keys` has.
    let others = modifiers.alt || modifiers.shift || modifiers.platform;
    let key = keystroke.key.to_ascii_lowercase();
    if !modifiers.control && !others {
        return match key.as_str() {
            "f12" => Some(CmdKey::Connect),
            "f2" => Some(CmdKey::FlightData),
            "f3" => Some(CmdKey::FlightPlanner),
            "f4" => Some(CmdKey::Tuning),
            "f5" => Some(CmdKey::RefreshParams),
            _ => None,
        };
    }
    if !modifiers.control || others {
        return None;
    }
    match key.as_str() {
        "f" => Some(CmdKey::Temp),
        "p" => Some(CmdKey::PluginManager),
        "g" => Some(CmdKey::NmeaOut),
        "x" => Some(CmdKey::MapCache),
        "l" => Some(CmdKey::Spectrogram),
        "w" => Some(CmdKey::Propagation),
        "z" => Some(CmdKey::CameraTest),
        "t" => Some(CmdKey::OverrideConnect),
        "y" => Some(CmdKey::StorageWrite),
        "j" => Some(CmdKey::Devops),
        _ => None,
    }
}

impl MissionPlanner {
    /// `ProcessCmdKey`: whether the key was one of its, and taken. The flight screen's own come
    /// first, as its control's `ProcessCmdKey` runs before the form's.
    /// `// C#: MainV2.cs:4067-4182; GCSViews/FlightData.cs:865-943`
    pub(crate) fn process_cmd_key(
        &mut self,
        keystroke: &Keystroke,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) -> bool {
        if self.screen == Screen::Fly
            && let Some(index) = fly_tab_key(keystroke)
        {
            self.fly_pages.select_index(index);
            return true;
        }
        let Some(key) = cmd_key(keystroke) else {
            return false;
        };
        match key {
            CmdKey::Connect => self.connect_clicked(window, cx),
            CmdKey::FlightData => self.choose_screen(Screen::Fly),
            CmdKey::FlightPlanner => self.choose_screen(Screen::Plan),
            // `MenuTuning_Click`: CONFIG/TUNING, as its tab shows it.
            CmdKey::Tuning => self.choose_screen(Screen::Config),
            CmdKey::RefreshParams => self.refresh_param_list(),
            // The owner's (2026-10-04): the PLUGINS tab, where the C# opens a form of its own.
            CmdKey::PluginManager => self.choose_screen(Screen::Plugins),
            CmdKey::OverrideConnect => self.override_connect(window, cx),
            CmdKey::StorageWrite => self.storage_write(),
            // The owner's (2026-10-04): the EXPERIMENTAL tab, where the C# opens the temp form.
            CmdKey::Temp => self.choose_screen(Screen::Experimental),
            // Their forms are not ported yet (NOT_DONE_YET_MATRIX.md, ProcessCmdKey's row).
            CmdKey::NmeaOut
            | CmdKey::MapCache
            | CmdKey::Spectrogram
            | CmdKey::Propagation
            | CmdKey::CameraTest
            | CmdKey::Devops => return false,
        }
        true
    }

    /// F5: `comPort.getParamList()`, then `MyView.ShowScreen(MyView.current.Name)`. With no
    /// vehicle there is nothing to fetch; the screen is shown again either way.
    /// `// C#: MainV2.cs:4093-4098`
    fn refresh_param_list(&mut self) {
        if self.telemetry.view().vehicle.is_some() {
            self.telemetry.download_parameters();
        }
        let screen = self.screen;
        self.choose_screen(screen);
    }

    /// Ctrl+T, `comPort.Open(false)`: the box's port opened past `Connect`'s checks - no
    /// question about a moving model, no disconnect - the transport asking what its `Open` asks,
    /// and the parameters not fetched. Already open, nothing: `Open` returns at once.
    /// `// C#: MainV2.cs:4146-4157; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:668-671`
    fn override_connect(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let view = self.telemetry.view();
        if view.connected && !view.target.starts_with("file:") {
            return;
        }
        self.do_connect_with(true, window, cx);
    }

    /// Ctrl+Y: `PREFLIGHT_STORAGE` 1 to the vehicle, then "Done MAV_ACTION_STORAGE_WRITE" - the
    /// C# shows it whatever the answer, and "Invalid command" when `doCommand` throws.
    /// `// C#: MainV2.cs:4158-4174`
    fn storage_write(&mut self) {
        let report = crate::telemetry::Report {
            timed_out: Some(crate::raw_params::INVALID_COMMAND.to_owned()),
            refused: Some(STORAGE_WRITE_DONE.to_owned()),
            accepted: Some(STORAGE_WRITE_DONE.to_owned()),
            fallback: None,
        };
        let sent = self.telemetry.send_handle().and_then(|(_, vehicle)| {
            self.telemetry.command(
                vehicle,
                crate::raw_params::PREFLIGHT_STORAGE,
                crate::raw_params::COMMIT,
                report,
            )
        });
        if sent.is_none() {
            self.file_status = Some(crate::raw_params::INVALID_COMMAND.to_owned());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(text: &str) -> Keystroke {
        Keystroke::parse(text).expect("a keystroke")
    }

    /// Each of `ProcessCmdKey`'s keys, as `keyData ==` compares them.
    #[test]
    fn every_key_of_process_cmd_key() {
        let table = [
            ("f12", CmdKey::Connect),
            ("f2", CmdKey::FlightData),
            ("f3", CmdKey::FlightPlanner),
            ("f4", CmdKey::Tuning),
            ("f5", CmdKey::RefreshParams),
            ("ctrl-f", CmdKey::Temp),
            ("ctrl-p", CmdKey::PluginManager),
            ("ctrl-g", CmdKey::NmeaOut),
            ("ctrl-x", CmdKey::MapCache),
            ("ctrl-l", CmdKey::Spectrogram),
            ("ctrl-w", CmdKey::Propagation),
            ("ctrl-z", CmdKey::CameraTest),
            ("ctrl-t", CmdKey::OverrideConnect),
            ("ctrl-y", CmdKey::StorageWrite),
            ("ctrl-j", CmdKey::Devops),
        ];
        for (text, expected) in table {
            assert_eq!(cmd_key(&key(text)), Some(expected), "{text}");
        }
    }

    /// The flight screen's tabs: Ctrl+1 to Ctrl+9 the first nine, Ctrl+0 the tenth; nothing else.
    #[test]
    fn control_and_a_digit_is_a_flight_tab() {
        assert_eq!(fly_tab_key(&key("ctrl-1")), Some(0));
        assert_eq!(fly_tab_key(&key("ctrl-9")), Some(8));
        assert_eq!(fly_tab_key(&key("ctrl-0")), Some(9));
        for text in ["1", "ctrl-shift-1", "alt-1", "ctrl-p", "ctrl-f1"] {
            assert_eq!(fly_tab_key(&key(text)), None, "{text}");
        }
    }

    /// Anything else is not its: a letter alone, another modifier with Control, an F key with a
    /// modifier, Ctrl+S (commented out in the C#), a key it does not name.
    #[test]
    fn other_keys_are_not_its() {
        for text in [
            "p", "f", "ctrl-shift-p", "ctrl-alt-p", "cmd-p", "shift-f2", "ctrl-f2", "ctrl-s",
            "ctrl-q", "f1", "f6", "escape",
        ] {
            assert_eq!(cmd_key(&key(text)), None, "{text}");
        }
    }
}
