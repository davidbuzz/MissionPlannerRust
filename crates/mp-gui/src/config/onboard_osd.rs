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

//! Onboard OSD: `GCSViews/ConfigurationView/ConfigOSD.cs`, the CONFIG page listed over a vehicle
//! that has any `OSD*` parameter (`SoftwareConfig.cs:206-208`, `ConfigOSD.cs:214-217`), and
//! `ExtLibs/OSDConfigurator`, the library it is a shell over: the OSD's parameters as settings,
//! each `OSDn_` group of them as a screen of items, the items drawn with the Clarity font on a
//! canvas the operator drags them about on, and every item's and screen's options as rows of
//! controls.
//!
//! The page (`ConfigOSD.cs:68-76, 219-330`):
//!
//! * `Activate` takes every parameter whose name starts with `OSD` as an [`Setting`] that keeps
//!   its original value (`OSDSetting`, `:24-66`), builds the configuration for screens 1 to 6
//!   ([`create`]) and shows it: the Settings tab of the `OSD_` options and a tab a screen;
//! * Write customization writes each changed setting with `setParam` under a "Writing changes..."
//!   progress dialog, "No Changes to Write!" when none has changed, "Your are not connected"
//!   without a link, and "Write Failed for N params: a, b, c..." for what the vehicle refused -
//!   the last three on the status line here (the owner's rule on avoidable boxes); `Deactivate`
//!   does the same silently while "Auto write on leaving" is ticked (`:233-300`);
//! * Discard all changes asks "Are you sure?" and puts every setting back (`:239-246`);
//! * Refresh asks "This will reset your changes. Continue?" while anything has changed, and then,
//!   armed, "Update Params" with its Show-me-again box; the list is fetched again and the page
//!   activated over it (`:302-329`);
//! * OSD / Telmetry Slots Config is the OSD5 and OSD6 parameter slots' dialog, in
//!   [`super::onboard_osd_slots`].
//!
//! The library (`ExtLibs/OSDConfigurator`):
//!
//! * `ConfigFactory` ([`create`]): a screen `n` is every `OSDn_` setting; an item is a name with
//!   its `_X`, `_Y` and `_EN` all three, any other `OSDn_<name>_<suffix>` of that name an extra
//!   option of it, and what is left over the screen's own options; the `OSD_` settings are the
//!   global options (`ConfigFactory.cs:7-100`);
//! * `OSDScreen.CopyTo` ([`copy_screen`]): Paste Layout, each option of each item of the copied
//!   screen into the option of the same name here, the `OSDn` prefix set aside
//!   (`Models/OSDItem.cs:52-73`);
//! * `OsdItemCaptionProvider` ([`caption`]): what an item is drawn as - its name with Show Names
//!   on, the parameter a slot of screen 5 or 6 is assigned (or "NAME (NOT SET)"), or a sample
//!   reading in the OSD's own symbols, "1.8" and the like packed into the font's digit pairs
//!   (`ConfigOSD.cs:331-524`, `GUI/Symbols.cs`);
//! * `Visualizer` ([`Geometry`], [`render`]): the character cell 24x36, 12x18 with Decrease on, and
//!   18x27 for an HD layout, the screen 30x16 or 60x22 cells, the canvas their product, the
//!   glyphs cut from `clarity.png` - 16 by 16 of 12x18 with a pixel between - and drawn a cell
//!   each, the NTSC/PAL line at row 13, the 50x18 lines of an HD layout, and the selected item's
//!   yellow rectangle five pixels out (`GUI/Visualizer.cs`);
//! * `LayoutControl` ([`OnboardOsd::press`], [`OnboardOsd::drag`]): a press on an enabled item
//!   selects it and takes its offset from the pointer, the pointer then moving it a cell at a
//!   time, never above or left of the screen (`GUI/LayoutControl.cs:453-484`);
//! * `OptionControlFactory` ([`Control`]): which control each setting gets, and the order they
//!   stack in - `_EN` above `_X` above `_Y` above the rest (`GUI/OptionControls/OptionControlFactory.cs`).
//!
//! Divergences, each at its site: the rows of equal weight keep the vehicle's order, where
//! `List.Sort` leaves theirs unspecified; the slot names of screens 5 and 6 are fetched when a
//! caption first needs them, as the C#'s `Lazy` fetches, but without holding the window - the
//! items read "(NOT SET)" until the answers are in; the glyphs are scaled to the cell by nearest
//! pixel, where GDI+ interpolates.
// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::{Arc, OnceLock};
use web_time::Instant;

use gpui::RenderImage;
use image::RgbaImage;
use mp_link::RequestId;
use mp_link::requests::RequestOutcome;

use super::onboard_osd_slots::{SlotNames, Slots};
use super::servo_output::{Combo, Designer, Number};
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::TextField;

/// The page's class, `typeof(ConfigOSD).Name`, which CONFIG's list knows it by.
pub const CLASS: &str = "ConfigOSD";
/// `Strings.OnboardOSD`, the page's title in CONFIG's list.
/// `// C#: ExtLibs/Strings/Strings.resx:649-651`
pub const TITLE: &str = "Onboard OSD";

/// `btnWrite.Text`. `// C#: GCSViews/ConfigurationView/ConfigOSD.Designer.cs:88`
pub const WRITE: &str = "Write customization";
/// `cbAutoWriteOnLeave.Text`, ticked from the Designer. `// C#: ConfigOSD.Designer.cs:97-100`
pub const AUTO_WRITE: &str = "Auto write on leaving";
/// `btnOsd56ItemsSetup.Text`, with its misspelling. `// C#: ConfigOSD.Designer.cs:69`
pub const SLOTS_CONFIG: &str = "OSD / Telmetry Slots Config";
/// `btnRefreshParameters.Text`. `// C#: ConfigOSD.Designer.cs:78`
pub const REFRESH: &str = "Refresh";
/// `btnDiscardChanges.Text`. `// C#: ConfigOSD.Designer.cs:87`
pub const DISCARD: &str = "Discard all changes";
/// `tabSettings.Text`, its padding trimmed. `// C#: GUI/OSDUserControl.Designer.cs:87`
pub const SETTINGS_TAB: &str = "Settings";
/// `grEditorOptions.Text`. `// C#: GUI/ScreenControl.Designer.cs:307`
pub const EDITOR_OPTIONS: &str = "Editor Options";
/// `groupScreenOptions.Text`. `// C#: GUI/ScreenControl.Designer.cs:260`
pub const SCREEN_OPTIONS: &str = "Screen Options";
/// `groupOptions.Text` before an item is selected. `// C#: GUI/ScreenControl.Designer.cs:294`
pub const ITEM_OPTIONS: &str = "Item Options";
/// `cbReducedView.Text`. `// C#: GUI/ScreenControl.Designer.cs:338`
pub const DECREASE: &str = "Decrease";
/// `btnCopy.Text`. `// C#: GUI/ScreenControl.Designer.cs:331`
pub const COPY_LAYOUT: &str = "Copy Layout";
/// `btnPaste.Text`. `// C#: GUI/ScreenControl.Designer.cs:325`
pub const PASTE_LAYOUT: &str = "Paste Layout";
/// `cbUseNameCaptions.Text`. `// C#: GUI/ScreenControl.Designer.cs:313`
pub const SHOW_NAMES: &str = "Show Names";
/// `btnClearAll.Text`. `// C#: GUI/ScreenControl.Designer.cs:319`
pub const CLEAR_ALL: &str = "Clear All";
/// `cbHighDefView.Text`. `// C#: GUI/ScreenControl.Designer.cs:370`
pub const HD_LAYOUT: &str = "HD Layout";

/// `CheckConnected`'s box: "Your are not connected", `Strings.ERROR` - the status line here.
/// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:78-87`
pub const NOT_CONNECTED: &str = "Your are not connected";
/// `WriteParameters`' box when nothing has changed - the status line here.
/// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:250-255`
pub const NO_CHANGES: &str = "No Changes to Write!";
/// The progress dialog's caption while writing. `// C#: ConfigOSD.cs:261`
pub const WRITING: &str = "Writing changes...";
/// `DiscardChanges`' question, OK and Cancel. `// C#: ConfigOSD.cs:241`
pub const ARE_YOU_SURE: &str = "Are you sure?";
/// `RefreshParameters`' first question, Yes and No. `// C#: ConfigOSD.cs:304-306`
pub const RESET_CHANGES: &str = "This will reset your changes. Continue?";
/// `Common.MessageShowAgain`'s caption, armed. `// C#: ConfigOSD.cs:311`
pub const REFRESH_PARAMS: &str = "Refresh Params";
/// `Strings.ErrorReceivingParams`, when the fetch fails. `// C#: ConfigOSD.cs:320`
pub const ERROR_RECEIVING: &str = "Error receiving list";

/// `Weight`: how the option rows stack, the heaviest at the top.
/// `// C#: GUI/OptionControls/OptionControlFactory.cs:12-18`
mod weight {
    pub const ENABLED: i32 = 100;
    pub const X: i32 = 80;
    pub const Y: i32 = 75;
    pub const MEDIUM: i32 = 50;
}

/// `OSD_FONT`'s choices. `// C#: OptionControlFactory.cs:43`
pub const FONTS: [&str; 5] = ["Clarity", "Clarity Medium", "BF Style", "Bold", "Digital"];
/// `OSD_UNITS`'s. `// C#: OptionControlFactory.cs:45`
pub const UNITS: [&str; 4] = ["Metric", "Imperial", "SI", "Aviation"];
/// `OSD_SW_METHOD`'s. `// C#: OptionControlFactory.cs:47`
pub const SW_METHODS: [&str; 3] = ["0: RC change", "1: PWM", "2: Low-to-High"];
/// `OSD_OPTIONS`'s bits. `// C#: OptionControlFactory.cs:49`
pub const OPTION_BITS: [&str; 3] = ["Decimal Pack", "Inverted Wind", "Inverted AH Roll"];
/// `ItemControl.ItemTypes`, an `OSDn_PARAMm_TYPE`'s choices.
/// `// C#: GUI/Osd56ItemsSetup/ItemControl.cs:17-27`
pub const ITEM_TYPES: [&str; 8] = [
    "None",
    "Serial Protocol",
    "Servo Function",
    "Aux Function",
    "Flight Mode",
    "Failsafe Act 1",
    "Failsafe Act 2",
    "Num Types",
];

// ---------------------------------------------------------------------------------------------
// The model: settings, items, screens.
// ---------------------------------------------------------------------------------------------

/// `OSDSetting`: a parameter, its value as the page holds it, and the value it had when the page
/// was activated or last written.
/// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:24-66`
#[derive(Debug, Clone, PartialEq)]
pub struct Setting {
    /// `Name`.
    pub name: String,
    value: f64,
    original: f64,
}

impl Setting {
    /// A setting as `Activate` makes it, unchanged.
    #[must_use]
    pub fn new(name: impl Into<String>, value: f64) -> Self {
        Self {
            name: name.into(),
            value,
            original: value,
        }
    }

    /// `Value`.
    #[must_use]
    pub const fn value(&self) -> f64 {
        self.value
    }

    /// `Value`'s setter. Whether it changed, which is when the C# raises `Updated`.
    #[allow(clippy::float_cmp)] // the C# compares exactly
    pub fn set_value(&mut self, value: f64) -> bool {
        if value == self.value {
            return false;
        }
        self.value = value;
        true
    }

    /// `Changed`: whether the value differs from the original.
    #[must_use]
    #[allow(clippy::float_cmp)]
    pub fn changed(&self) -> bool {
        self.value != self.original
    }

    /// `ClearChanged`, after a write: what is held is the original now.
    pub fn clear_changed(&mut self) {
        self.original = self.value;
    }

    /// `DiscardChange`: back to the original.
    pub fn discard(&mut self) {
        self.value = self.original;
    }
}

/// `OSDItem`: a name and the settings that are its options, by their index in the page's list.
/// `// C#: ExtLibs/OSDConfigurator/Models/OSDItem.cs:9-36`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Item {
    /// `Name`: `ALTITUDE` of `OSD1_ALTITUDE_X`.
    pub name: String,
    /// `Options`, in the vehicle's order.
    pub options: Vec<usize>,
}

impl Item {
    /// `EndsWith(suffix)`: the first option whose name ends with the item's name and `suffix`.
    #[must_use]
    pub fn ends_with(&self, settings: &[Setting], suffix: &str) -> Option<usize> {
        let wanted = format!("{}{suffix}", self.name);
        self.options.iter().copied().find(|&index| {
            settings
                .get(index)
                .is_some_and(|s| s.name.ends_with(&wanted))
        })
    }

    /// `Enabled`, the `_EN` option.
    #[must_use]
    pub fn enabled(&self, settings: &[Setting]) -> Option<usize> {
        self.ends_with(settings, "_EN")
    }

    /// `X`.
    #[must_use]
    pub fn x(&self, settings: &[Setting]) -> Option<usize> {
        self.ends_with(settings, "_X")
    }

    /// `Y`.
    #[must_use]
    pub fn y(&self, settings: &[Setting]) -> Option<usize> {
        self.ends_with(settings, "_Y")
    }

    /// `Enabled.Value > 0`.
    #[must_use]
    pub fn is_enabled(&self, settings: &[Setting]) -> bool {
        self.enabled(settings)
            .and_then(|index| settings.get(index))
            .is_some_and(|s| s.value() > 0.0)
    }

    /// `(int)X.Value, (int)Y.Value`.
    #[must_use]
    pub fn position(&self, settings: &[Setting]) -> (i32, i32) {
        let at = |index: Option<usize>| {
            index
                .and_then(|i| settings.get(i))
                .map_or(0, |s| truncate(s.value()))
        };
        (at(self.x(settings)), at(self.y(settings)))
    }
}

/// `OSDScreen`.
/// `// C#: ExtLibs/OSDConfigurator/Models/OSDItem.cs:39-74`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Screen {
    /// `Name`: "Screen n".
    pub name: String,
    /// `n`.
    pub number: u8,
    /// `Options`: the `OSDn_` settings that belong to no item.
    pub options: Vec<usize>,
    /// `Items`.
    pub items: Vec<Item>,
}

/// `OSDConfiguration`.
/// `// C#: ExtLibs/OSDConfigurator/Models/OSDConfiguration.cs`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Config {
    /// `Options`: the `OSD_` settings.
    pub options: Vec<usize>,
    /// `Screens`, in the order asked for, those with any setting.
    pub screens: Vec<Screen>,
}

/// `ConfigFactory.Create`: the screens asked for that have any setting, and the `OSD_` options.
/// `// C#: ExtLibs/OSDConfigurator/ConfigFactory.cs:9-20`
#[must_use]
pub fn create(settings: &[Setting], screens: impl IntoIterator<Item = u8>) -> Config {
    Config {
        options: settings
            .iter()
            .enumerate()
            .filter(|(_, s)| s.name.starts_with("OSD_"))
            .map(|(index, _)| index)
            .collect(),
        screens: screens
            .into_iter()
            .filter_map(|number| make_screen(number, settings))
            .collect(),
    }
}

/// `MakeScreen`: the `OSDn_` settings; an item for every name with `_X`, `_Y` and `_EN` (all
/// three, else dropped); every other `OSDn_<name>_<suffix>` of an item's name attached to it;
/// the rest the screen's own options. `None` for a screen with no setting.
/// `// C#: ExtLibs/OSDConfigurator/ConfigFactory.cs:22-77`
fn make_screen(number: u8, settings: &[Setting]) -> Option<Screen> {
    let prefix = format!("OSD{number}_");
    let mut left: Vec<usize> = settings
        .iter()
        .enumerate()
        .filter(|(_, s)| s.name.starts_with(&prefix))
        .map(|(index, _)| index)
        .collect();
    if left.is_empty() {
        return None;
    }
    // The primary settings, by item name, in order of first sight (`Dictionary` enumerates in
    // insertion order while nothing is removed).
    let mut items: Vec<(String, Vec<usize>)> = Vec::new();
    for &index in &left {
        let Some(setting) = settings.get(index) else {
            continue;
        };
        let Some((found_prefix, name, suffix)) = split_into_three(&setting.name) else {
            continue;
        };
        if found_prefix.eq_ignore_ascii_case(&prefix)
            && (suffix.eq_ignore_ascii_case("_X")
                || suffix.eq_ignore_ascii_case("_Y")
                || suffix.eq_ignore_ascii_case("_EN"))
        {
            match items.iter_mut().find(|(held, _)| *held == name) {
                Some((_, options)) => options.push(index),
                None => items.push((name.to_owned(), vec![index])),
            }
        }
    }
    // Remove incomplete.
    items.retain(|(_, options)| options.len() == 3);
    for (_, options) in &items {
        left.retain(|index| !options.contains(index));
    }
    // Search for extra settings (Not implemented in AP at the moment).
    for &index in &left {
        let Some(setting) = settings.get(index) else {
            continue;
        };
        if let Some((_, name, _)) = split_into_three(&setting.name)
            && let Some((_, options)) = items.iter_mut().find(|(held, _)| *held == name)
        {
            options.push(index);
        }
    }
    for (_, options) in &items {
        left.retain(|index| !options.contains(index));
    }
    Some(Screen {
        name: format!("Screen {number}"),
        number,
        options: left,
        items: items
            .into_iter()
            .map(|(name, options)| Item { name, options })
            .collect(),
    })
}

/// `SplitIntoThree`: `OSD1_BAT_VOLT_X` into `OSD1_`, `BAT_VOLT` and `_X`; nothing for fewer than
/// three parts.
/// `// C#: ExtLibs/OSDConfigurator/ConfigFactory.cs:79-93`
#[must_use]
pub fn split_into_three(name: &str) -> Option<(String, String, String)> {
    let parts: Vec<&str> = name.split('_').collect();
    if parts.len() < 3 {
        return None;
    }
    let first = parts.first()?;
    let last = parts.last()?;
    let middle = parts.get(1..parts.len() - 1)?.join("_");
    Some((format!("{first}_"), middle, format!("_{last}")))
}

/// `OSDScreen.CopyTo`: Paste Layout. For each item of the copied screen, the item of the same
/// name here takes each option's value into its option of the same name after the `OSDn`
/// prefix. Nothing for a screen pasted onto itself.
/// `// C#: ExtLibs/OSDConfigurator/Models/OSDItem.cs:52-73`
pub fn copy_screen(settings: &mut [Setting], from: &Screen, to: &Screen) {
    if from == to {
        return;
    }
    for source in &from.items {
        let Some(target) = to
            .items
            .iter()
            .find(|item| item.name.eq_ignore_ascii_case(&source.name))
        else {
            continue;
        };
        for &option in &source.options {
            let Some(source_name) = settings.get(option).map(|s| s.name.clone()) else {
                continue;
            };
            let value = settings.get(option).map_or(0.0, Setting::value);
            let tail = source_name.get(4..).unwrap_or("");
            let found = target.options.iter().copied().find(|&index| {
                settings
                    .get(index)
                    .is_some_and(|s| s.name.get(4..).unwrap_or("").eq_ignore_ascii_case(tail))
            });
            if let Some(index) = found
                && let Some(setting) = settings.get_mut(index)
            {
                setting.set_value(value);
            }
        }
    }
}

/// `TryParseScreenAndIndex`: `Regex.Match(name, "OSD(\d)\S+(\d)\S+")` - the digit after `OSD`
/// and the last digit with anything after it: `OSD5_PARAM1_EN` is screen 5, index 1.
/// `// C#: ExtLibs/OSDConfigurator/Extensions/Extensions.cs:9-21`
#[must_use]
pub fn parse_screen_and_index(name: &str) -> Option<(u8, u8)> {
    let start = name.find("OSD")?;
    let rest = name.get(start + 3..)?;
    let bytes = rest.as_bytes();
    let screen = bytes.first().filter(|b| b.is_ascii_digit())?;
    // `\S+(\d)\S+`: at least one character between, then the last digit that has at least one
    // character after it - what greedy matching settles on.
    let index = bytes
        .iter()
        .enumerate()
        .skip(2)
        .filter(|(i, b)| b.is_ascii_digit() && *i + 1 < bytes.len())
        .map(|(_, b)| b - b'0')
        .next_back()?;
    Some((screen - b'0', index))
}

// ---------------------------------------------------------------------------------------------
// Captions.
// ---------------------------------------------------------------------------------------------

/// `CaptionModes`. `// C#: GUI/IItemCaptionProvider.cs:5-9`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptionMode {
    /// A sample reading in the OSD's symbols.
    Realistic,
    /// The item's name.
    Names,
}

/// `Symbols`: the font's places for the OSD's symbols.
/// `// C#: ExtLibs/OSDConfigurator/GUI/Symbols.cs`
pub mod sym {
    pub const M: char = '\u{B9}';
    pub const ALT_M: char = '\u{B1}';
    pub const BATT_FULL: char = '\u{90}';
    pub const RSSI: char = '\u{01}';
    pub const VOLT: char = '\u{06}';
    pub const AMP: char = '\u{9A}';
    pub const MAH: char = '\u{07}';
    pub const MS: char = '\u{9F}';
    pub const KMH: char = '\u{A1}';
    pub const DEGR: char = '\u{A8}';
    pub const PCNT: char = '%';
    pub const RPM: char = '\u{E0}';
    pub const ASPD: char = '\u{E1}';
    pub const GSPD: char = '\u{E2}';
    pub const SAT_L: char = '\u{1E}';
    pub const SAT_R: char = '\u{1F}';
    pub const HDOP_L: char = '\u{BD}';
    pub const HDOP_R: char = '\u{BE}';
    pub const HOME: char = '\u{BF}';
    pub const ARROW_START: char = '\u{60}';
    pub const AH_H_START: char = '\u{80}';
    pub const AH_CENTER_LINE_LEFT: char = '\u{26}';
    pub const AH_CENTER_LINE_RIGHT: char = '\u{27}';
    pub const AH_CENTER: char = '\u{7E}';
    pub const HEADING_N: char = '\u{18}';
    pub const HEADING_S: char = '\u{19}';
    pub const HEADING_E: char = '\u{1A}';
    pub const HEADING_DIVIDED_LINE: char = '\u{1C}';
    pub const HEADING_LINE: char = '\u{1D}';
    pub const UP: char = '\u{A3}';
    pub const DEGREES_C: char = '\u{0E}';
    pub const GPS_LAT: char = '\u{A6}';
    pub const GPS_LONG: char = '\u{A7}';
    pub const DISARMED: char = '\u{E9}';
    pub const ROLLL: char = '\u{EB}';
    pub const PTCHUP: char = '\u{EC}';
    pub const PTCHDWN: char = '\u{ED}';
    pub const XERR: char = '\u{EE}';
    pub const DIST: char = '"';
    pub const FLY: char = '\u{9C}';
    pub const EFF: char = '\u{F2}';
    pub const MW: char = '\u{F4}';
    pub const CLK: char = '\u{BC}';
    pub const KILO: char = 'K';
    pub const FENCE_ENABLED: char = '\u{F5}';
    pub const RNGFD: char = '\u{F7}';
    pub const LQ: char = '\u{F8}';
    pub const SIDEBAR_A: char = '\u{13}';
}

/// A character `n` places after `start`.
fn after(start: char, n: u32) -> char {
    char::from_u32(start as u32 + n).unwrap_or(start)
}

/// `DoGetCaption`: an item's sample reading and the offset it is drawn at, by name; the name
/// itself for one not in the table.
/// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:381-521`
#[must_use]
pub fn raw_caption(name: &str) -> (String, i32) {
    use sym::*;
    let s = |text: String| (text, 0);
    match name {
        "ASPEED" | "ASPD1" | "ASPD2" => s(format!("{ASPD}10{MS}")),
        "ESCRPM" => s(format!("10{KILO}{RPM}")),
        "HDOP" => s(format!("{HDOP_L}{HDOP_R}10.2")),
        "ALTITUDE" => (format!("11{ALT_M}"), -2),
        "BAT_VOLT" | "BAT2_VLT" | "RESTVOLT" => s(format!("{}11.8{VOLT}", after(BATT_FULL, 1))),
        "RSSI" => s(format!("{RSSI}93")),
        "CURRENT" | "CURRENT2" | "ESCAMPS" => s(format!("8.3{AMP}")),
        "FLTMODE" => s(format!("STAB{DISARMED}")),
        "SATS" => s(format!("{SAT_L}{SAT_R}13")),
        "BATUSED" | "BAT2USED" => (format!("125{MAH}"), -1),
        "HORIZON" => {
            let h = after(AH_H_START, 4);
            (
                format!("{h}{h}{h}{AH_CENTER_LINE_LEFT}{AH_CENTER}{AH_CENTER_LINE_RIGHT}{h}{h}{h}"),
                4,
            )
        }
        "COMPASS" => (
            [
                HEADING_N,
                HEADING_LINE,
                HEADING_DIVIDED_LINE,
                HEADING_LINE,
                HEADING_E,
                HEADING_LINE,
                HEADING_DIVIDED_LINE,
                HEADING_LINE,
                HEADING_S,
            ]
            .iter()
            .collect(),
            4,
        ),
        "GPSLONG" => s(format!("{GPS_LONG}  30.5003901")),
        "GPSLAT" => s(format!("{GPS_LAT}  50.3534305")),
        "HOME" => s(format!("{HOME}{ARROW_START} 101{M}")),
        "GSPEED" => s(format!("{GSPD}{ARROW_START} 17{KMH}")),
        "PITCH" => s(format!("{PTCHDWN} 10{DEGR}")),
        "ROLL" => s(format!("{ROLLL}  3{DEGR}")),
        "VSPEED" => s(format!("{UP} 0{MS}")),
        "THROTTLE" => (format!("0{PCNT}"), -2),
        "HEADING" => (format!("32{DEGR}"), -1),
        "LINK_Q" => s(format!("99{LQ}")),
        "RNGF" => s(format!("{RNGFD} 1.23{M}")),
        "FENCE" => s(FENCE_ENABLED.to_string()),
        "AVGCELLV" | "CELLVOLT" => s(format!("{}3.85{VOLT}", after(BATT_FULL, 1))),
        "VTX_PWR" => s(format!("25{MW}")),
        "CALLSIGN" => s("TOPGUN".to_owned()),
        "PLUSCODE" => s("MP97+Q6H".to_owned()),
        "SIDEBARS" => (format!("{SIDEBAR_A}           {SIDEBAR_A}"), -4),
        "CLK" => s(format!("{CLK}12:00")),
        "ATEMP" | "BTEMP" | "TEMP" | "ESCTEMP" => s(format!("25{DEGREES_C}")),
        "EFF" => s(format!("{EFF}100{MAH}")),
        "CLIMBEFF" => s(format!("{PTCHUP}{EFF}2.1{M}")),
        "FLTIME" => s(format!("{FLY}03:00")),
        "DIST" => s(DIST.to_string()),
        "XTRACK" => s(XERR.to_string()),
        _ => s(name.to_owned()),
    }
}

/// `GetCommonCaption`: the sample reading with every "d.d" packed into the font's pair of a
/// digit with the point after it (192 + d) and a digit with the point before it (208 + d).
/// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:361-379`
#[must_use]
pub fn common_caption(name: &str) -> (String, i32) {
    let (raw, offset) = raw_caption(name);
    (pack_digit_points(&raw), offset)
}

/// `Regex.Replace(caption, "(\d)(\.)(\d)", DigitPointEvaluator)`, left to right, each match
/// consumed whole as `Regex.Replace` consumes it.
fn pack_digit_points(text: &str) -> String {
    const WITH_DIGIT_AT_END: u32 = 192;
    const WITH_DIGIT_AT_BEGIN: u32 = 208;
    let chars: Vec<char> = text.chars().collect();
    let mut out = String::with_capacity(text.len());
    let mut i = 0;
    while i < chars.len() {
        let (Some(a), Some(dot), Some(b)) = (chars.get(i), chars.get(i + 1), chars.get(i + 2))
        else {
            out.extend(chars.get(i..).into_iter().flatten());
            break;
        };
        if a.is_ascii_digit() && *dot == '.' && b.is_ascii_digit() {
            let d1 = *a as u32 - '0' as u32;
            let d2 = *b as u32 - '0' as u32;
            out.push(char::from_u32(WITH_DIGIT_AT_END + d1).unwrap_or(*a));
            out.push(char::from_u32(WITH_DIGIT_AT_BEGIN + d2).unwrap_or(*b));
            i += 3;
        } else {
            out.push(*a);
            i += 1;
        }
    }
    out
}

/// `GetItemCaption`: the name with Show Names on; for an item of screen 5 or 6 - one whose first
/// option names a screen above 4 - the parameter its slot is assigned, or "NAME (NOT SET)"; else
/// the common caption.
/// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:342-359`
#[must_use]
pub fn caption(
    item: &Item,
    settings: &[Setting],
    mode: CaptionMode,
    slots: &SlotNames,
) -> (String, i32) {
    if mode == CaptionMode::Names {
        return (item.name.clone(), 0);
    }
    let first = item
        .options
        .first()
        .and_then(|&index| settings.get(index))
        .map(|s| s.name.as_str())
        .unwrap_or("");
    if let Some((screen, index)) = parse_screen_and_index(first)
        && screen > 4
    {
        return (
            slots
                .name(screen, index)
                .map_or_else(|| format!("{} (NOT SET)", item.name), str::to_owned),
            0,
        );
    }
    common_caption(&item.name)
}

// ---------------------------------------------------------------------------------------------
// The visualizer: geometry and the raster.
// ---------------------------------------------------------------------------------------------

/// The font's cell, `fontCharSizePix`. `// C#: GUI/Visualizer.cs:12`
pub const FONT_CELL: (u32, u32) = (12, 18);
/// `NtscRows`, `DJICols`, `DJIRows`. `// C#: GUI/Visualizer.cs:14-16`
pub const NTSC_ROWS: u32 = 13;
pub const DJI_COLS: u32 = 50;
pub const DJI_ROWS: u32 = 18;
/// The canvas's background, `LayoutControl.BackColor`. `// C#: GUI/LayoutControl.Designer.cs:21`
pub const BACKGROUND: [u8; 4] = [127, 127, 127, 255];
/// `Pens.DimGray`.
pub const DIM_GRAY: [u8; 4] = [105, 105, 105, 255];

/// `Visualizer`'s sizes: the cell and the screen, from the Decrease and HD Layout boxes.
/// `// C#: GUI/ScreenControl.cs:155-179; GUI/Visualizer.cs:9-13`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Geometry {
    /// `cbReducedView.Checked`.
    pub reduced: bool,
    /// `cbHighDefView.Checked`.
    pub high_def: bool,
}

impl Geometry {
    /// `charSizePix`: 24x36; 12x18 reduced; 18x27 for HD, 12x18 reduced.
    #[must_use]
    pub const fn char_size(self) -> (u32, u32) {
        match (self.high_def, self.reduced) {
            (_, true) => (12, 18),
            (true, false) => (18, 27),
            (false, false) => (24, 36),
        }
    }

    /// `screenSizeChar`: 30x16, or 60x22 for HD.
    #[must_use]
    pub const fn screen_size(self) -> (u32, u32) {
        if self.high_def { (60, 22) } else { (30, 16) }
    }

    /// `GetCanvasSize`: the cells' product.
    #[must_use]
    pub const fn canvas_size(self) -> (u32, u32) {
        let (cw, ch) = self.char_size();
        let (cols, rows) = self.screen_size();
        (cw * cols, ch * rows)
    }

    /// `ToOSDLocation`: a canvas point to a cell, never left of or above the first.
    #[must_use]
    pub fn to_osd_location(self, (x, y): (i32, i32)) -> (i32, i32) {
        let (cw, ch) = self.char_size();
        (x.max(0) / to_i32(cw), y.max(0) / to_i32(ch))
    }

    /// `ToScreenPoint`: where an item at a cell, drawn `x_offset` cells to the left, starts.
    #[must_use]
    pub fn to_screen_point(self, (x, y): (i32, i32), x_offset: i32) -> (i32, i32) {
        let (cw, ch) = self.char_size();
        ((x - x_offset) * to_i32(cw), y * to_i32(ch))
    }

    /// `ToScreenRectangle`: the item's caption, a cell a character.
    #[must_use]
    pub fn to_screen_rectangle(
        self,
        position: (i32, i32),
        caption_len: usize,
        x_offset: i32,
    ) -> (i32, i32, i32, i32) {
        let (cw, ch) = self.char_size();
        let (x, y) = self.to_screen_point(position, x_offset);
        let len = u32::try_from(caption_len).unwrap_or(u32::MAX);
        (x, y, to_i32(cw).saturating_mul(to_i32(len)), to_i32(ch))
    }

    /// `Contains`.
    #[must_use]
    pub fn contains(self, rectangle: (i32, i32, i32, i32), (px, py): (i32, i32)) -> bool {
        let (x, y, w, h) = rectangle;
        px >= x && px < x + w && py >= y && py < y + h
    }
}

/// An item as the canvas draws it: its caption, cell and offset.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Drawn {
    /// The item's name, for the facts.
    pub name: String,
    /// The caption.
    pub caption: String,
    /// `(int)X.Value, (int)Y.Value`.
    pub position: (i32, i32),
    /// The caption's `xOffset`.
    pub x_offset: i32,
}

/// The Clarity font: `Resources.clarity` cut into 256 cells of 12x18, 16 a row, a pixel of
/// margin between them (`MakeFont(clarity, 16, 16, 1)`), kept once decoded.
/// `// C#: GUI/Visualizer.cs:20-22, 73-93`
pub struct Font {
    sheet: RgbaImage,
}

static FONT: OnceLock<Option<Font>> = OnceLock::new();

impl Font {
    /// The font, decoded once; `None` if `assets/osd/clarity.png` does not decode.
    pub fn get() -> Option<&'static Font> {
        FONT.get_or_init(|| {
            let bytes: &[u8] = include_bytes!("../../../../assets/osd/clarity.png");
            image::load_from_memory(bytes).ok().map(|decoded| Font {
                sheet: decoded.to_rgba8(),
            })
        })
        .as_ref()
    }

    /// The sheet's pixel of glyph `code` at `(x, y)` within its cell: the cells are 16 a row,
    /// each 12x18 with a pixel between.
    #[must_use]
    pub fn pixel(&self, code: u32, x: u32, y: u32) -> [u8; 4] {
        let col = code % 16;
        let row = (code / 16) % 16;
        let sx = col * (FONT_CELL.0 + 1) + x;
        let sy = row * (FONT_CELL.1 + 1) + y;
        if sx < self.sheet.width() && sy < self.sheet.height() {
            self.sheet.get_pixel(sx, sy).0
        } else {
            [0, 0, 0, 0]
        }
    }
}

/// `LayoutControl.Draw`: the canvas - the grey background, the NTSC/PAL line or the HD lines,
/// and every enabled item's caption a cell a character, the glyphs scaled to the cell.
/// `// C#: GUI/LayoutControl.cs:497-517; GUI/Visualizer.cs:52-59, 103-117`
#[must_use]
pub fn render(font: Option<&Font>, geometry: Geometry, drawn: &[Drawn]) -> RgbaImage {
    let (width, height) = geometry.canvas_size();
    let (cw, ch) = geometry.char_size();
    let mut image = RgbaImage::from_pixel(width, height, image::Rgba(BACKGROUND));
    let line = |image: &mut RgbaImage, x0: u32, y0: u32, x1: u32, y1: u32| {
        for x in x0.min(x1)..=x0.max(x1).min(width.saturating_sub(1)) {
            for y in y0.min(y1)..=y0.max(y1).min(height.saturating_sub(1)) {
                image.put_pixel(x, y, image::Rgba(DIM_GRAY));
            }
        }
    };
    if geometry.high_def {
        // `DrawHDBackground`: 50 columns by 18 rows marked.
        let (cols, _) = geometry.screen_size();
        line(&mut image, 0, ch * DJI_ROWS, cw * DJI_COLS, ch * DJI_ROWS);
        line(&mut image, cw * DJI_COLS, ch * DJI_ROWS, cw * DJI_COLS, 0);
        let _ = cols;
    } else {
        // `DrawSDBackground`: the NTSC bottom at row 13.
        let (cols, _) = geometry.screen_size();
        line(&mut image, 0, ch * NTSC_ROWS, cw * cols, ch * NTSC_ROWS);
    }
    for item in drawn {
        let (mut x, y) = geometry.to_screen_point(item.position, item.x_offset);
        for c in item.caption.chars() {
            if let Some(font) = font {
                blit_glyph(font, &mut image, c as u32, x, y, cw, ch);
            }
            x += to_i32(cw);
        }
    }
    image
}

/// One glyph into the canvas at `(x, y)`, scaled from the font's cell to `(cw, ch)` by nearest
/// pixel, alpha over the background.
fn blit_glyph(font: &Font, image: &mut RgbaImage, code: u32, x: i32, y: i32, cw: u32, ch: u32) {
    for dy in 0..ch {
        for dx in 0..cw {
            let tx = x + to_i32(dx);
            let ty = y + to_i32(dy);
            if tx < 0 || ty < 0 {
                continue;
            }
            let (tx, ty) = (to_u32(tx), to_u32(ty));
            if tx >= image.width() || ty >= image.height() {
                continue;
            }
            let sx = dx * FONT_CELL.0 / cw;
            let sy = dy * FONT_CELL.1 / ch;
            let src = font.pixel(code, sx, sy);
            let alpha = u32::from(src[3]);
            if alpha == 0 {
                continue;
            }
            let dst = image.get_pixel_mut(tx, ty);
            for (held, &over) in dst.0.iter_mut().zip(src.iter()).take(3) {
                let over = u32::from(over) * alpha;
                let under = u32::from(*held) * (255 - alpha);
                #[allow(clippy::cast_possible_truncation)]
                {
                    *held = ((over + under) / 255).min(255) as u8;
                }
            }
            if let Some(a) = dst.0.get_mut(3) {
                *a = 255;
            }
        }
    }
}

/// C#'s `(int)` of a double, toward zero.
#[allow(clippy::cast_possible_truncation)]
const fn truncate(value: f64) -> i32 {
    value as i32
}

#[allow(clippy::cast_possible_wrap)]
const fn to_i32(value: u32) -> i32 {
    value as i32
}

#[allow(clippy::cast_sign_loss)]
const fn to_u32(value: i32) -> u32 {
    value as u32
}

// ---------------------------------------------------------------------------------------------
// The option rows.
// ---------------------------------------------------------------------------------------------

/// `OptionControlFactory.Create(setting)`: the control a setting gets.
/// `// C#: GUI/OptionControls/OptionControlFactory.cs:40-66`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    /// `BoolSettingControl`: a check box, 1 or 0.
    Bool,
    /// `IntSpinSettingControl`: a `NumericUpDown`, WinForms' 0 to 100 a step of 1.
    Spin,
    /// `DropdownSettingControl`: the index chosen is the value.
    Dropdown(&'static [&'static str]),
    /// `BitwiseSettingControl`: the value typed, and a bit a tick.
    Bitwise(&'static [&'static str]),
    /// `IntSettingControl`: the value typed, `double.TryParse`.
    Int,
}

impl Control {
    /// The control and the weight it stacks by.
    #[must_use]
    pub fn for_setting(name: &str) -> (Self, i32) {
        match name {
            "OSD_FONT" => return (Self::Dropdown(&FONTS), weight::MEDIUM),
            "OSD_UNITS" => return (Self::Dropdown(&UNITS), weight::MEDIUM),
            "OSD_SW_METHOD" => return (Self::Dropdown(&SW_METHODS), weight::MEDIUM),
            "OSD_OPTIONS" => return (Self::Bitwise(&OPTION_BITS), weight::MEDIUM),
            "OSD_H_OFFSET" | "OSD_V_OFFSET" => return (Self::Spin, weight::MEDIUM),
            _ => {}
        }
        if name.ends_with("_EN") || name.ends_with("_ENABLE") {
            return (Self::Bool, weight::ENABLED);
        }
        if name.ends_with("_X") {
            return (Self::Spin, weight::X);
        }
        if name.ends_with("_Y") {
            return (Self::Spin, weight::Y);
        }
        if is_param_type(name) {
            return (Self::Dropdown(&ITEM_TYPES), weight::MEDIUM);
        }
        (Self::Int, weight::MEDIUM)
    }

    /// The row's height: `BitwiseSettingControl` is 125 high, the rest 42.
    /// `// C#: GUI/OptionControls/*.Designer.cs`
    #[must_use]
    pub const fn height(self) -> f32 {
        match self {
            Self::Bitwise(_) => 125.0,
            _ => 42.0,
        }
    }
}

/// `Regex.IsMatch(name, "OSD\d_PARAM\d_TYPE")`.
fn is_param_type(name: &str) -> bool {
    let bytes = name.as_bytes();
    let mut i = 0;
    while i + 16 <= bytes.len() {
        if name.get(i..i + 3) == Some("OSD")
            && bytes.get(i + 3).is_some_and(u8::is_ascii_digit)
            && name.get(i + 4..i + 10) == Some("_PARAM")
            && bytes.get(i + 10).is_some_and(u8::is_ascii_digit)
            && name.get(i + 11..i + 16) == Some("_TYPE")
        {
            return true;
        }
        i += 1;
    }
    false
}

/// `OptionControlFactory.Create(settings)`: the rows in the order they stack, the heaviest at the
/// top - `List.Sort` ascending, then each docked to the top over the last. Rows of equal weight
/// keep the vehicle's order, which the C#'s unstable sort leaves unspecified (a divergence).
/// `// C#: GUI/OptionControls/OptionControlFactory.cs:20-38`
#[must_use]
pub fn rows(settings: &[Setting], indexes: &[usize]) -> Vec<(usize, Control)> {
    let mut rows: Vec<(usize, Control, i32)> = indexes
        .iter()
        .filter_map(|&index| {
            let setting = settings.get(index)?;
            let (control, weight) = Control::for_setting(&setting.name);
            Some((index, control, weight))
        })
        .collect();
    rows.sort_by_key(|(_, _, weight)| std::cmp::Reverse(*weight));
    rows.into_iter()
        .map(|(index, control, _)| (index, control))
        .collect()
}

// ---------------------------------------------------------------------------------------------
// The page.
// ---------------------------------------------------------------------------------------------

/// Which tab shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    /// `tabSettings`.
    Settings,
    /// A screen, by its index in [`Config::screens`].
    Screen(usize),
}

/// A question the page asks, over the window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Question {
    /// "Are you sure?", OK and Cancel.
    Discard,
    /// "This will reset your changes. Continue?", Yes and No.
    ResetChanges,
    /// "Update Params", OK and Cancel, with Show me again (ticked when true).
    RefreshArmed(bool),
}

/// A text box being typed into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TextTarget {
    /// An `IntSettingControl`'s box, or a `BitwiseSettingControl`'s, for the setting.
    Setting(usize),
}

/// The writes `WriteParameters` makes, one `setParam` at a time.
/// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:248-300`
#[derive(Debug)]
pub struct Writing {
    /// The changed settings still to write.
    pub queue: VecDeque<usize>,
    /// The write under way.
    pub waiting: Option<(usize, RequestId)>,
    /// The names the vehicle refused or never echoed.
    pub failed: Vec<String>,
    /// `silent`: Deactivate's write says nothing.
    pub silent: bool,
    /// When it started, for the progress dialog's marquee.
    pub started: Instant,
}

/// The raster the canvas shows, made in `page` when what it draws changes and turned into the
/// GPU's image when painted.
#[derive(Default)]
pub struct Raster {
    /// What the image was made from.
    pub key: Option<(Geometry, Vec<Drawn>)>,
    /// A canvas made and not yet uploaded.
    pub pending: Option<RgbaImage>,
    /// The image painted.
    pub shown: Option<Arc<RenderImage>>,
    /// The canvas's origin in the window at the last paint, for the pointer.
    pub origin: (f32, f32),
}

/// A screen's editor state: `ScreenControl`'s boxes and selection.
/// `// C#: GUI/ScreenControl.cs:14-39, 50-79`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScreenState {
    /// `SelectedItem`, by index in the screen's items.
    pub selected: Option<usize>,
    /// `cbReducedView` and `cbHighDefView`.
    pub geometry: Geometry,
}

impl Default for ScreenState {
    fn default() -> Self {
        Self {
            selected: None,
            geometry: Geometry {
                reduced: false,
                high_def: false,
            },
        }
    }
}

/// The page.
pub struct OnboardOsd {
    /// Whether CONFIG's list shows the page.
    active: bool,
    /// `osdSettings`.
    pub settings: Vec<Setting>,
    /// The configuration built over them.
    pub config: Config,
    /// Each screen's editor state, as [`Config::screens`] is ordered.
    pub screens: Vec<ScreenState>,
    /// The tab showing.
    pub tab: Tab,
    /// `ScreenToCopy`: Copy Layout's screen, by index.
    pub copied: Option<usize>,
    /// `CaptionMode`, the caption provider's: one for the page, as the C#'s one provider is
    /// shared by every screen.
    pub caption_mode: CaptionMode,
    /// `cbAutoWriteOnLeave.Checked`.
    pub auto_write: bool,
    /// The slot names of screens 5 and 6, and their fetch.
    pub slots: Slots,
    /// The question showing.
    pub question: Option<Question>,
    /// The writes under way.
    pub writing: Option<Writing>,
    /// Refresh's fetch: the list it started from.
    refreshing: Option<Arc<[(String, f64)]>>,
    /// The canvas's raster and GPU image.
    pub raster: Rc<RefCell<Raster>>,
    /// `itemMoving` and `moveShift`: the item being dragged and the pointer's offset from it.
    drag: Option<(i32, i32)>,
    /// The spin boxes, by setting index.
    pub numbers: Vec<Number>,
    /// The spin box being typed into.
    pub editing: Option<usize>,
    /// The text box being typed into, and its text.
    pub typing: Option<TextTarget>,
    pub field: TextField,
    /// The combo whose list is dropped, by setting index.
    pub open_combo: Option<usize>,
    /// The three scrolling columns of rows - the Settings tab's, Screen Options' and Item
    /// Options' - whose scroll the dropped list follows.
    pub scrolls: [gpui::ScrollHandle; 3],
    /// What the status line should say, taken by the host.
    status: Option<String>,
    /// The last press, for the facts.
    last: &'static str,
}

impl Default for OnboardOsd {
    fn default() -> Self {
        Self {
            active: false,
            settings: Vec::new(),
            config: Config::default(),
            screens: Vec::new(),
            tab: Tab::Settings,
            copied: None,
            caption_mode: CaptionMode::Realistic,
            auto_write: true,
            slots: Slots::default(),
            question: None,
            writing: None,
            refreshing: None,
            raster: Rc::new(RefCell::new(Raster::default())),
            drag: None,
            numbers: Vec::new(),
            editing: None,
            typing: None,
            field: TextField::new(""),
            open_combo: None,
            scrolls: [
                gpui::ScrollHandle::new(),
                gpui::ScrollHandle::new(),
                gpui::ScrollHandle::new(),
            ],
            status: None,
            last: "",
        }
    }
}

impl std::fmt::Debug for OnboardOsd {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OnboardOsd")
            .field("active", &self.active)
            .field("settings", &self.settings.len())
            .field("screens", &self.config.screens.len())
            .field("tab", &self.tab)
            .finish_non_exhaustive()
    }
}

impl OnboardOsd {
    /// Whether CONFIG's list shows the page.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The status line's words, once.
    pub fn take_status(&mut self) -> Option<String> {
        self.status.take()
    }

    /// `GetOSDSettings`: every parameter whose name starts with `OSD`, case blind.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:207-212`
    #[must_use]
    pub fn settings_of(parameters: &[(String, f64)]) -> Vec<Setting> {
        parameters
            .iter()
            .filter(|(name, _)| {
                name.get(..3)
                    .is_some_and(|start| start.eq_ignore_ascii_case("OSD"))
            })
            .map(|(name, value)| Setting::new(name.clone(), *value))
            .collect()
    }

    /// `IsApplicable`: any `OSD` parameter.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:214-217`
    #[must_use]
    pub fn is_applicable(parameters: &[(String, f64)]) -> bool {
        !Self::settings_of(parameters).is_empty()
    }

    /// `Activate`: the settings taken from the list, the configuration for screens 1 to 6 built
    /// and shown from its Settings tab, Copy Layout's screen forgotten, the slot names fetched
    /// again when a caption next needs them.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:219-231; GUI/OSDUserControl.cs:12-21`
    pub fn activate(&mut self, parameters: &[(String, f64)]) {
        self.active = true;
        self.settings = Self::settings_of(parameters);
        self.config = create(&self.settings, 1..=6);
        self.screens = self
            .config
            .screens
            .iter()
            .map(|_| ScreenState::default())
            .collect();
        self.tab = Tab::Settings;
        self.copied = None;
        self.caption_mode = CaptionMode::Realistic;
        self.slots.forget();
        self.question = None;
        self.editing = None;
        self.typing = None;
        self.open_combo = None;
        self.drag = None;
        self.numbers = self
            .settings
            .iter()
            .map(|setting| {
                // WinForms' `NumericUpDown`: 0 to 100, a step of 1, no decimals.
                let mut number = Number::new(Designer {
                    minimum: 0.0,
                    maximum: 100.0,
                    value: setting.value(),
                    decimals: 0,
                });
                number.enabled = true;
                number.param = setting.name.clone();
                number
            })
            .collect();
        self.raster.borrow_mut().key = None;
    }

    /// The page hidden: `Deactivate`, writing silently while Auto write on leaving is ticked.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:233-237`
    pub fn deactivate(&mut self, telemetry: &Telemetry, connected: bool) {
        self.active = false;
        self.question = None;
        self.open_combo = None;
        if self.auto_write {
            self.write(telemetry, connected, true);
        }
    }

    /// The screen showing, if a screen tab does.
    #[must_use]
    pub fn current_screen(&self) -> Option<usize> {
        match self.tab {
            Tab::Screen(index) if index < self.config.screens.len() => Some(index),
            _ => None,
        }
    }

    /// A tab chosen.
    pub fn choose_tab(&mut self, tab: Tab) {
        self.tab = tab;
        self.open_combo = None;
        self.leave_boxes();
        self.last = "tab";
    }

    /// Whether any setting has changed.
    #[must_use]
    pub fn any_changed(&self) -> bool {
        self.settings.iter().any(Setting::changed)
    }

    /// How many settings have changed.
    #[must_use]
    pub fn changed_count(&self) -> usize {
        self.settings.iter().filter(|s| s.changed()).count()
    }

    /// A setting's value set, and the spin box over it told.
    pub fn set_value(&mut self, index: usize, value: f64) {
        if let Some(setting) = self.settings.get_mut(index) {
            setting.set_value(value);
        }
        if let Some(number) = self.numbers.get_mut(index) {
            number.set_value(value);
        }
    }

    /// The items the screen draws: every enabled one, with its caption.
    /// `// C#: GUI/LayoutControl.cs:509-516`
    #[must_use]
    pub fn drawn(&self, screen: usize) -> Vec<Drawn> {
        let Some(scr) = self.config.screens.get(screen) else {
            return Vec::new();
        };
        scr.items
            .iter()
            .filter(|item| item.is_enabled(&self.settings))
            .map(|item| {
                let (caption, x_offset) =
                    caption(item, &self.settings, self.caption_mode, self.slots.names());
                Drawn {
                    name: item.name.clone(),
                    caption,
                    position: item.position(&self.settings),
                    x_offset,
                }
            })
            .collect()
    }

    /// The selected item's rectangle on the canvas, inflated by 5 as `DrawSelection` draws it.
    /// `// C#: GUI/Visualizer.cs:61-68`
    #[must_use]
    pub fn selection_rectangle(&self, screen: usize) -> Option<(i32, i32, i32, i32)> {
        let state = self.screens.get(screen)?;
        let item = self
            .config
            .screens
            .get(screen)?
            .items
            .get(state.selected?)?;
        let (caption, x_offset) =
            caption(item, &self.settings, self.caption_mode, self.slots.names());
        let (x, y, w, h) = state.geometry.to_screen_rectangle(
            item.position(&self.settings),
            caption.chars().count(),
            x_offset,
        );
        Some((x - 5, y - 5, w + 10, h + 10))
    }

    /// `LayoutControlMouseDown`: the first enabled item under the point selected, the drag
    /// begun with the pointer's offset from the item's cell. Whether one was hit.
    /// `// C#: GUI/LayoutControl.cs:472-484`
    pub fn press(&mut self, screen: usize, at: (i32, i32)) -> bool {
        let Some(geometry) = self.screens.get(screen).map(|s| s.geometry) else {
            return false;
        };
        let Some(scr) = self.config.screens.get(screen) else {
            return false;
        };
        let hit = scr.items.iter().enumerate().find(|(_, item)| {
            if !item.is_enabled(&self.settings) {
                return false;
            }
            let (caption, x_offset) =
                caption(item, &self.settings, self.caption_mode, self.slots.names());
            let rectangle = geometry.to_screen_rectangle(
                item.position(&self.settings),
                caption.chars().count(),
                x_offset,
            );
            geometry.contains(rectangle, at)
        });
        let Some((index, item)) = hit else {
            return false;
        };
        let position = item.position(&self.settings);
        self.select(screen, Some(index));
        let (cx, cy) = geometry.to_screen_point(position, 0);
        self.drag = Some((cx - at.0, cy - at.1));
        self.last = "press";
        true
    }

    /// `LayoutControlMouseMove` with the button down: the selected item moved to the cell under
    /// the pointer, shifted as it was grabbed. Whether it moved.
    /// `// C#: GUI/LayoutControl.cs:457-471`
    pub fn drag(&mut self, screen: usize, at: (i32, i32)) -> bool {
        let Some((shift_x, shift_y)) = self.drag else {
            return false;
        };
        let Some(state) = self.screens.get(screen) else {
            return false;
        };
        let geometry = state.geometry;
        let Some(selected) = state.selected else {
            return false;
        };
        let Some(item) = self
            .config
            .screens
            .get(screen)
            .and_then(|s| s.items.get(selected))
        else {
            return false;
        };
        let (nx, ny) = geometry.to_osd_location((at.0 + shift_x, at.1 + shift_y));
        let (x, y) = item.position(&self.settings);
        if nx == x && ny == y {
            return false;
        }
        let (ix, iy) = (item.x(&self.settings), item.y(&self.settings));
        if let Some(ix) = ix {
            self.set_value(ix, f64::from(nx));
        }
        if let Some(iy) = iy {
            self.set_value(iy, f64::from(ny));
        }
        self.last = "drag";
        true
    }

    /// `LayoutControlMouseUp`.
    pub fn release(&mut self) {
        self.drag = None;
    }

    /// `SelectedItem`'s setter: the Item Options group retitled and refilled; a different item
    /// closes any list and box being typed into.
    /// `// C#: GUI/ScreenControl.cs:14-39`
    pub fn select(&mut self, screen: usize, index: Option<usize>) {
        if let Some(state) = self.screens.get_mut(screen)
            && state.selected != index
        {
            state.selected = index;
            self.open_combo = None;
            self.leave_boxes();
        }
        self.last = "select";
    }

    /// Decrease ticked or cleared: `SetViewSize`.
    pub fn set_reduced(&mut self, screen: usize, reduced: bool) {
        if let Some(state) = self.screens.get_mut(screen) {
            state.geometry.reduced = reduced;
        }
        self.last = "decrease";
    }

    /// HD Layout ticked or cleared: `SetHighDefView`.
    pub fn set_high_def(&mut self, screen: usize, high_def: bool) {
        if let Some(state) = self.screens.get_mut(screen) {
            state.geometry.high_def = high_def;
        }
        self.last = "hd";
    }

    /// Show Names ticked or cleared: `SetCaptionMode`, for every screen.
    pub fn set_show_names(&mut self, names: bool) {
        self.caption_mode = if names {
            CaptionMode::Names
        } else {
            CaptionMode::Realistic
        };
        self.last = "names";
    }

    /// Clear All: every item of the screen disabled.
    /// `// C#: GUI/ScreenControl.cs:145`
    pub fn clear_all(&mut self, screen: usize) {
        let enabled: Vec<usize> = self
            .config
            .screens
            .get(screen)
            .map(|s| {
                s.items
                    .iter()
                    .filter_map(|item| item.enabled(&self.settings))
                    .collect()
            })
            .unwrap_or_default();
        for index in enabled {
            self.set_value(index, 0.0);
        }
        self.last = "clear-all";
    }

    /// Copy Layout: the screen remembered, if it has items.
    /// `// C#: GUI/ScreenControl.cs:146`
    pub fn copy_layout(&mut self, screen: usize) {
        if self
            .config
            .screens
            .get(screen)
            .is_some_and(|s| !s.items.is_empty())
        {
            self.copied = Some(screen);
        }
        self.last = "copy";
    }

    /// Paste Layout: the copied screen's items' options into this screen's.
    /// `// C#: GUI/ScreenControl.cs:147`
    pub fn paste_layout(&mut self, screen: usize) {
        let Some(from) = self.copied else {
            return;
        };
        let (Some(source), Some(target)) = (
            self.config.screens.get(from).cloned(),
            self.config.screens.get(screen).cloned(),
        ) else {
            return;
        };
        if target.items.is_empty() {
            return;
        }
        copy_screen(&mut self.settings, &source, &target);
        for (index, setting) in self.settings.iter().enumerate() {
            if let Some(number) = self.numbers.get_mut(index) {
                number.set_value(setting.value());
            }
        }
        self.last = "paste";
    }

    /// An item's check box, or a `_EN`/`_ENABLE` row's: 1 or 0.
    /// `// C#: GUI/ItemControls/CommonItemControl.cs:17-20; GUI/OptionControls/BoolSettingControl.cs:14-17`
    pub fn toggle(&mut self, index: usize) {
        let on = self.settings.get(index).is_some_and(|s| s.value() > 0.0);
        self.set_value(index, if on { 0.0 } else { 1.0 });
        self.last = "toggle";
    }

    /// A dropdown row's choice: the index chosen is the value.
    /// `// C#: GUI/OptionControls/DropdownSettingControl.cs:21-24`
    pub fn choose(&mut self, index: usize, chosen: i64) {
        #[allow(clippy::cast_precision_loss)]
        self.set_value(index, chosen as f64);
        self.open_combo = None;
        self.last = "choose";
    }

    /// A bitwise row's bit ticked or cleared: `value | (1 << bit)`, or `value - (1 << bit)`.
    /// `// C#: GUI/OptionControls/BitwiseSettingControl.cs:35-50`
    pub fn toggle_bit(&mut self, index: usize, bit: u32) {
        let value = self.settings.get(index).map_or(0.0, Setting::value);
        let held = truncate(value);
        let flag = 1_i32 << bit;
        let next = if held & flag == 0 {
            held | flag
        } else {
            // `setting.Value - (1 << e.Index)`, on the double.
            return self.set_value(index, value - f64::from(flag));
        };
        self.set_value(index, f64::from(next));
        self.last = "bit";
    }

    /// The spin box's arrow: one step, the typed text read first.
    /// `// C#: GUI/OptionControls/IntSpinSettingControl.cs:14-17`
    pub fn step(&mut self, index: usize, up: bool, now: Instant) {
        if let Some(number) = self.numbers.get_mut(index) {
            let _ = number.step(up, now);
            let value = number.value();
            if let Some(setting) = self.settings.get_mut(index) {
                setting.set_value(value);
            }
        }
        self.last = "step";
    }

    /// A spin box clicked into.
    pub fn begin_number(&mut self, index: usize) {
        self.leave_boxes();
        self.editing = Some(index);
        self.open_combo = None;
    }

    /// A key in the spin box being typed into: the arrows step, Enter reads the text.
    pub fn number_key(&mut self, event: &gpui::KeyDownEvent, now: Instant) -> bool {
        let Some(index) = self.editing else {
            return false;
        };
        let Some(number) = self.numbers.get_mut(index) else {
            return false;
        };
        let (used, _) = number.key(event, now);
        let value = number.value();
        if let Some(setting) = self.settings.get_mut(index) {
            setting.set_value(value);
        }
        used
    }

    /// A text box clicked into: its text taken to type over.
    pub fn begin_text(&mut self, target: TextTarget) {
        self.leave_boxes();
        let TextTarget::Setting(index) = target;
        let text = self
            .settings
            .get(index)
            .map_or_else(String::new, |s| value_text(s.value()));
        self.field = TextField::new("");
        self.field.set(text);
        self.typing = Some(target);
        self.open_combo = None;
    }

    /// A key in the text box being typed into. Enter validates as leaving does; Escape leaves
    /// the box as it was.
    /// `// C#: GUI/OptionControls/IntSettingControl.cs:23-33; BitwiseSettingControl.cs:21-33`
    pub fn text_key(&mut self, event: &gpui::KeyDownEvent) -> bool {
        use crate::textfield::KeyOutcome;
        if self.typing.is_none() {
            return false;
        }
        match self.field.key(event) {
            KeyOutcome::Submitted => {
                self.leave_boxes();
                true
            }
            KeyOutcome::Cancelled => {
                self.typing = None;
                true
            }
            KeyOutcome::Changed => true,
            KeyOutcome::Ignored => false,
        }
    }

    /// The boxes left: a spin box's text read (`ValidateEditText`), a text box's parsed - a
    /// double for an `IntSettingControl`, an int for a bitwise one - and taken, or the box put
    /// back to the value when it does not parse.
    pub fn leave_boxes(&mut self) {
        if let Some(index) = self.editing.take()
            && let Some(number) = self.numbers.get_mut(index)
        {
            let _ = number.commit(Instant::now());
            let value = number.value();
            if let Some(setting) = self.settings.get_mut(index) {
                setting.set_value(value);
            }
        }
        if let Some(TextTarget::Setting(index)) = self.typing.take() {
            let text = self.field.value().trim().to_owned();
            let bitwise = self
                .settings
                .get(index)
                .is_some_and(|s| matches!(Control::for_setting(&s.name).0, Control::Bitwise(_)));
            let parsed = if bitwise {
                text.parse::<i32>().ok().map(f64::from)
            } else {
                text.parse::<f64>().ok()
            };
            if let Some(value) = parsed {
                self.set_value(index, value);
            }
        }
    }

    /// A dropdown row's arrow: its list dropped, or taken up.
    pub fn toggle_combo(&mut self, index: usize) {
        self.leave_boxes();
        self.open_combo = if self.open_combo == Some(index) {
            None
        } else {
            Some(index)
        };
    }

    /// A dropdown row as a [`Combo`], for the shared drawing.
    #[must_use]
    pub fn combo(&self, index: usize, choices: &[&str]) -> Combo {
        let value = self.settings.get(index).map_or(0.0, Setting::value);
        Combo {
            param: self
                .settings
                .get(index)
                .map_or_else(String::new, |s| s.name.clone()),
            options: choices
                .iter()
                .enumerate()
                .map(|(i, text)| (i64::try_from(i).unwrap_or(0), (*text).to_owned()))
                .collect(),
            selected: Some(i64::from(truncate(value))),
            enabled: true,
            top_index: 0,
        }
    }

    // ---- Write, Discard, Refresh ------------------------------------------------------------

    /// Write customization, or Deactivate's silent write: nothing to write said unless silent,
    /// no link said, else the changed settings queued for one `setParam` each.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:248-300`
    pub fn write(&mut self, telemetry: &Telemetry, connected: bool, silent: bool) {
        self.leave_boxes();
        self.last = "write";
        if !self.any_changed() {
            if !silent {
                self.status = Some(NO_CHANGES.to_owned());
            }
            return;
        }
        if !connected {
            self.status = Some(format!(
                "{}: {NOT_CONNECTED}",
                super::servo_output::ERROR_TITLE
            ));
            return;
        }
        if self.writing.is_some() {
            return;
        }
        let queue: VecDeque<usize> = self
            .settings
            .iter()
            .enumerate()
            .filter(|(_, s)| s.changed())
            .map(|(index, _)| index)
            .collect();
        let mut writing = Writing {
            queue,
            waiting: None,
            failed: Vec::new(),
            silent,
            started: Instant::now(),
        };
        self.send_next(&mut writing, telemetry);
        self.writing = Some(writing);
    }

    /// The next changed setting sent, or none left.
    fn send_next(&mut self, writing: &mut Writing, telemetry: &Telemetry) {
        while let Some(index) = writing.queue.pop_front() {
            let Some(setting) = self.settings.get(index) else {
                continue;
            };
            match telemetry.set_parameter_confirmed(&setting.name, setting.value()) {
                Some(id) => {
                    writing.waiting = Some((index, id));
                    return;
                }
                // `setParam`'s throw with no vehicle: the name failed.
                None => writing.failed.push(setting.name.clone()),
            }
        }
    }

    /// The progress dialog's Cancel: the writes left are not made.
    pub fn cancel_write(&mut self) {
        if let Some(writing) = self.writing.as_mut() {
            writing.queue.clear();
        }
        self.last = "cancel-write";
    }

    /// Once a frame: the write under way seen back - echoed, its setting's change cleared;
    /// refused or unanswered, its name kept for the box - and the next sent; when the last is
    /// done, "Write Failed for N params: ..." unless silent.
    fn advance_writes(&mut self, telemetry: &Telemetry) {
        let Some(mut writing) = self.writing.take() else {
            return;
        };
        if let Some((index, id)) = writing.waiting {
            let Some(request) = telemetry.request(id) else {
                writing.failed.push(self.name_of(index));
                writing.waiting = None;
                self.send_next(&mut writing, telemetry);
                self.writing = Some(writing);
                return;
            };
            if !request.is_finished() {
                self.writing = Some(writing);
                return;
            }
            match request.outcome() {
                Some(
                    RequestOutcome::Accepted { .. }
                    | RequestOutcome::Unchanged
                    | RequestOutcome::Sent,
                ) => {
                    if let Some(setting) = self.settings.get_mut(index) {
                        setting.clear_changed();
                    }
                }
                _ => writing.failed.push(self.name_of(index)),
            }
            writing.waiting = None;
            self.send_next(&mut writing, telemetry);
        }
        if writing.waiting.is_some() {
            self.writing = Some(writing);
            return;
        }
        // Done.
        if !writing.silent && !writing.failed.is_empty() {
            let shown: Vec<&str> = writing.failed.iter().take(3).map(String::as_str).collect();
            let more = if writing.failed.len() > 3 { "..." } else { "" };
            self.status = Some(format!(
                "Write Failed for {} params: {}{more}",
                writing.failed.len(),
                shown.join(", ")
            ));
        }
    }

    fn name_of(&self, index: usize) -> String {
        self.settings
            .get(index)
            .map_or_else(String::new, |s| s.name.clone())
    }

    /// Discard all changes: "Are you sure?".
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:239-246`
    pub fn click_discard(&mut self) {
        self.leave_boxes();
        self.question = Some(Question::Discard);
        self.last = "discard";
    }

    /// Refresh: the question about changes while any are held; else straight on to the fetch.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:302-307`
    pub fn click_refresh(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        show_again: Option<&str>,
    ) {
        self.leave_boxes();
        self.last = "refresh";
        if self.any_changed() {
            self.question = Some(Question::ResetChanges);
            return;
        }
        self.refresh_connected(telemetry, view, show_again);
    }

    /// `if (!MainV2.comPort.BaseStream.IsOpen) return;` then, armed, "Update Params" unless its
    /// Show-me-again was cleared; else the fetch.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:308-311`
    fn refresh_connected(
        &mut self,
        telemetry: &Telemetry,
        view: &TelemetryView,
        show_again: Option<&str>,
    ) {
        if !view.connected {
            return;
        }
        let armed = view.state.as_ref().is_some_and(|state| state.armed);
        // `ContainsKey(key) && GetBoolean(key) == false`.
        let suppressed =
            show_again.is_some_and(|value| !crate::raw_params::get_boolean(Some(value)));
        if armed && !suppressed {
            self.question = Some(Question::RefreshArmed(true));
            return;
        }
        self.fetch(telemetry, view);
    }

    /// `getParamList`: the list fetched again, the page activated over it when it is back.
    /// `// C#: GCSViews/ConfigurationView/ConfigOSD.cs:313-328`
    fn fetch(&mut self, telemetry: &Telemetry, view: &TelemetryView) {
        self.refreshing = Some(Arc::clone(&view.parameters));
        telemetry.download_parameters();
        self.last = "fetch";
    }

    /// Whether Refresh's fetch is under way, which disables the page as `this.Enabled = false`.
    #[must_use]
    pub const fn refreshing(&self) -> bool {
        self.refreshing.is_some()
    }

    /// The question's "Show me again?" box: the value written under [`SHOW_AGAIN_KEY`].
    pub fn toggle_show_again(&mut self) -> Option<&'static str> {
        let Some(Question::RefreshArmed(ticked)) = self.question.as_mut() else {
            return None;
        };
        *ticked = !*ticked;
        Some(if *ticked { "True" } else { "False" })
    }

    /// The question answered: Discard's OK puts every setting back; Reset's Yes goes on to the
    /// fetch; Update Params' OK fetches.
    pub fn answer(
        &mut self,
        yes: bool,
        telemetry: &Telemetry,
        view: &TelemetryView,
        show_again: Option<&str>,
    ) {
        let Some(question) = self.question.take() else {
            return;
        };
        self.last = if yes { "answer-yes" } else { "answer-no" };
        if !yes {
            return;
        }
        match question {
            Question::Discard => {
                for setting in &mut self.settings {
                    setting.discard();
                }
                for (index, setting) in self.settings.iter().enumerate() {
                    if let Some(number) = self.numbers.get_mut(index) {
                        number.set_value(setting.value());
                    }
                }
            }
            Question::ResetChanges => self.refresh_connected(telemetry, view, show_again),
            Question::RefreshArmed(_) => self.fetch(telemetry, view),
        }
    }

    /// Once a frame: the writes moved on, Refresh's list seen back and the page activated over
    /// it, the slots' fetch and update moved on.
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, now: Instant) {
        if !self.active && self.writing.is_none() && self.refreshing.is_none() {
            // Nothing runs for a page not showing but a write that outlives it.
            self.slots.tick(telemetry, view, now);
            return;
        }
        self.advance_writes(telemetry);
        if let Some(before) = &self.refreshing {
            let whole = !view.parameters.is_empty()
                && view.parameters.len() >= usize::from(view.parameters_expected);
            if !view.connected {
                self.refreshing = None;
                self.status = Some(format!(
                    "{}: {ERROR_RECEIVING}",
                    super::servo_output::ERROR_TITLE
                ));
            } else if whole && !Arc::ptr_eq(before, &view.parameters) {
                self.refreshing = None;
                self.activate(&view.parameters);
            }
        }
        // A caption of screen 5 or 6 drawn: the slot names wanted, as the C#'s `Lazy` fetches
        // them the first time `GetItemCaption` asks.
        if let Some(screen) = self.current_screen()
            && self
                .config
                .screens
                .get(screen)
                .is_some_and(|s| s.number > 4)
        {
            self.slots.want_names(telemetry, view, now);
        }
        if let Some(words) = self.slots.tick(telemetry, view, now) {
            self.status = Some(words);
        }
        if self.slots.take_refresh() {
            // `RefreshParameters()` after the slots were updated.
            self.click_refresh(telemetry, view, None);
        }
    }
}

/// `setting.Value.ToString()`: a whole number without a point, else the shortest round trip.
#[must_use]
pub fn value_text(value: f64) -> String {
    if value.fract() == 0.0 && value.abs() < 1e15 {
        format!("{value:.0}")
    } else {
        format!("{value}")
    }
}

/// The facts the GUI scripts read.
pub fn record_facts(page: &OnboardOsd, view: &TelemetryView) {
    use crate::facts::record;
    record("config.onboard_osd.active", page.active);
    record(
        "config.onboard_osd.applicable",
        OnboardOsd::is_applicable(&view.parameters),
    );
    record("config.onboard_osd.settings", page.settings.len());
    record("config.onboard_osd.changed", page.changed_count());
    record(
        "config.onboard_osd.screens",
        page.config
            .screens
            .iter()
            .map(|s| s.number.to_string())
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "config.onboard_osd.tab",
        match page.tab {
            Tab::Settings => "settings".to_owned(),
            Tab::Screen(index) => page
                .config
                .screens
                .get(index)
                .map_or_else(|| "?".to_owned(), |s| s.name.clone()),
        },
    );
    record("config.onboard_osd.auto_write", page.auto_write);
    record(
        "config.onboard_osd.names",
        page.caption_mode == CaptionMode::Names,
    );
    record(
        "config.onboard_osd.copied",
        page.copied
            .and_then(|i| page.config.screens.get(i))
            .map_or_else(|| "none".to_owned(), |s| s.name.clone()),
    );
    record(
        "config.onboard_osd.question",
        match page.question {
            None => "none",
            Some(Question::Discard) => "discard",
            Some(Question::ResetChanges) => "reset",
            Some(Question::RefreshArmed(_)) => "refresh",
        },
    );
    record("config.onboard_osd.writing", page.writing.is_some());
    record(
        "config.onboard_osd.open_combo",
        page.open_combo
            .and_then(|i| page.settings.get(i))
            .map_or("none", |s| s.name.as_str()),
    );
    record(
        "config.onboard_osd.editing",
        page.editing
            .and_then(|i| page.settings.get(i))
            .map_or("none", |s| s.name.as_str()),
    );
    record("config.onboard_osd.refreshing", page.refreshing.is_some());
    record("config.onboard_osd.last", page.last);
    record(
        "config.onboard_osd.status",
        page.status.as_deref().unwrap_or("none"),
    );
    if let Some(screen) = page.current_screen() {
        let state = page.screens.get(screen);
        let geometry = state.map(|s| s.geometry).unwrap_or_default_geometry();
        let (w, h) = geometry.canvas_size();
        record("config.onboard_osd.canvas", format!("{w}x{h}"));
        record("config.onboard_osd.reduced", geometry.reduced);
        record("config.onboard_osd.hd", geometry.high_def);
        let drawn = page.drawn(screen);
        record("config.onboard_osd.drawn", drawn.len());
        record(
            "config.onboard_osd.items",
            page.config
                .screens
                .get(screen)
                .map(|s| {
                    s.items
                        .iter()
                        .map(|item| {
                            let (x, y) = item.position(&page.settings);
                            format!(
                                "{}={}:{x},{y}",
                                item.name,
                                u8::from(item.is_enabled(&page.settings))
                            )
                        })
                        .collect::<Vec<_>>()
                        .join(" ")
                })
                .unwrap_or_default(),
        );
        let selected = state
            .and_then(|s| s.selected)
            .and_then(|i| page.config.screens.get(screen).and_then(|s| s.items.get(i)));
        record(
            "config.onboard_osd.selected",
            selected.map_or("none", |item| item.name.as_str()),
        );
        record(
            "config.onboard_osd.caption",
            selected.map_or_else(String::new, |item| {
                let (text, offset) =
                    caption(item, &page.settings, page.caption_mode, page.slots.names());
                format!(
                    "{}@{offset}",
                    text.chars()
                        .map(|c| if c.is_ascii_graphic() || c == ' ' {
                            c.to_string()
                        } else {
                            format!("\\x{:02X}", c as u32)
                        })
                        .collect::<String>()
                )
            }),
        );
    } else {
        record("config.onboard_osd.canvas", "none");
        record("config.onboard_osd.drawn", 0);
        record("config.onboard_osd.items", "");
        record("config.onboard_osd.selected", "none");
        record("config.onboard_osd.caption", "");
    }
    record(
        "config.onboard_osd.values",
        page.settings
            .iter()
            .filter(|s| s.changed())
            .map(|s| format!("{}={}", s.name, value_text(s.value())))
            .collect::<Vec<_>>()
            .join(" "),
    );
    super::onboard_osd_slots::record_facts(&page.slots);
}

/// `Option<&ScreenState>::map(...)`'s default geometry, for the facts of a screen not yet made.
trait DefaultGeometry {
    fn unwrap_or_default_geometry(self) -> Geometry;
}

impl DefaultGeometry for Option<Geometry> {
    fn unwrap_or_default_geometry(self) -> Geometry {
        self.unwrap_or(Geometry {
            reduced: false,
            high_def: false,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(names: &[(&str, f64)]) -> Vec<(String, f64)> {
        names
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    /// A vehicle's OSD list, as ArduPilot lists it: the globals, two screens of two items, a
    /// screen-level option, and a parameter screen.
    fn vehicle() -> Vec<(String, f64)> {
        params(&[
            ("OSD_TYPE", 1.0),
            ("OSD_CHAN", 0.0),
            ("OSD_OPTIONS", 5.0),
            ("OSD_FONT", 2.0),
            ("OSD_H_OFFSET", 32.0),
            ("OSD1_ENABLE", 1.0),
            ("OSD1_CHAN_MIN", 900.0),
            ("OSD1_ALTITUDE_EN", 1.0),
            ("OSD1_ALTITUDE_X", 23.0),
            ("OSD1_ALTITUDE_Y", 8.0),
            ("OSD1_BAT_VOLT_EN", 0.0),
            ("OSD1_BAT_VOLT_X", 24.0),
            ("OSD1_BAT_VOLT_Y", 1.0),
            ("OSD1_LONELY_X", 1.0),
            ("OSD2_ENABLE", 0.0),
            ("OSD2_ALTITUDE_EN", 0.0),
            ("OSD2_ALTITUDE_X", 2.0),
            ("OSD2_ALTITUDE_Y", 3.0),
            ("OSD5_ENABLE", 0.0),
            ("OSD5_PARAM1_EN", 1.0),
            ("OSD5_PARAM1_X", 2.0),
            ("OSD5_PARAM1_Y", 2.0),
            ("OSD5_PARAM1_KEY", 0.0),
            ("OSD5_PARAM1_TYPE", 0.0),
            ("OSD5_PARAM1_MIN", 0.0),
            ("OSD5_PARAM1_MAX", 1.0),
            ("OSD5_PARAM1_INCR", 0.001),
            ("FRAME_CLASS", 1.0),
            ("osdx_lower", 1.0),
        ])
    }

    #[test]
    fn the_settings_are_every_osd_parameter_case_blind_and_the_page_applicable() {
        let settings = OnboardOsd::settings_of(&vehicle());
        assert_eq!(settings.len(), 28);
        assert!(settings.iter().any(|s| s.name == "osdx_lower"));
        assert!(OnboardOsd::is_applicable(&vehicle()));
        assert!(!OnboardOsd::is_applicable(&params(&[("FRAME_CLASS", 1.0)])));
    }

    #[test]
    fn a_setting_keeps_its_original_and_knows_when_it_changed() {
        let mut setting = Setting::new("OSD1_ALTITUDE_X", 23.0);
        assert!(!setting.changed());
        assert!(setting.set_value(24.0));
        assert!(!setting.set_value(24.0), "the same value raises nothing");
        assert!(setting.changed());
        setting.discard();
        assert_eq!(setting.value(), 23.0);
        setting.set_value(25.0);
        setting.clear_changed();
        assert!(!setting.changed());
        assert_eq!(setting.value(), 25.0);
    }

    #[test]
    fn the_factory_makes_screens_of_complete_items_with_their_extras_and_the_options() {
        let settings = OnboardOsd::settings_of(&vehicle());
        let config = create(&settings, 1..=6);
        let names = |indexes: &[usize]| -> Vec<&str> {
            indexes.iter().map(|&i| settings[i].name.as_str()).collect()
        };
        assert_eq!(
            names(&config.options),
            [
                "OSD_TYPE",
                "OSD_CHAN",
                "OSD_OPTIONS",
                "OSD_FONT",
                "OSD_H_OFFSET"
            ]
        );
        assert_eq!(
            config.screens.iter().map(|s| s.number).collect::<Vec<_>>(),
            [1, 2, 5]
        );
        let one = &config.screens[0];
        assert_eq!(one.name, "Screen 1");
        // ALTITUDE and BAT_VOLT are items; LONELY has only an _X and is dropped - and, having
        // been dropped, its setting is left among the screen's options.
        assert_eq!(
            one.items
                .iter()
                .map(|i| i.name.as_str())
                .collect::<Vec<_>>(),
            ["ALTITUDE", "BAT_VOLT"]
        );
        assert_eq!(
            names(&one.options),
            ["OSD1_ENABLE", "OSD1_CHAN_MIN", "OSD1_LONELY_X"]
        );
        let altitude = &one.items[0];
        assert_eq!(altitude.position(&settings), (23, 8));
        assert!(altitude.is_enabled(&settings));
        assert!(!one.items[1].is_enabled(&settings));
        // The parameter screen's item carries its extras.
        let five = &config.screens[2];
        assert_eq!(five.items.len(), 1);
        assert_eq!(
            names(&five.items[0].options),
            [
                "OSD5_PARAM1_EN",
                "OSD5_PARAM1_X",
                "OSD5_PARAM1_Y",
                "OSD5_PARAM1_KEY",
                "OSD5_PARAM1_TYPE",
                "OSD5_PARAM1_MIN",
                "OSD5_PARAM1_MAX",
                "OSD5_PARAM1_INCR"
            ]
        );
        assert_eq!(names(&five.options), ["OSD5_ENABLE"]);
        assert_eq!(
            split_into_three("OSD1_BAT_VOLT_X"),
            Some(("OSD1_".to_owned(), "BAT_VOLT".to_owned(), "_X".to_owned()))
        );
        assert_eq!(split_into_three("OSD_TYPE"), None);
    }

    #[test]
    fn paste_layout_copies_each_option_by_its_name_after_the_prefix() {
        let mut settings = OnboardOsd::settings_of(&vehicle());
        let config = create(&settings, 1..=6);
        let (one, two) = (config.screens[0].clone(), config.screens[1].clone());
        copy_screen(&mut settings, &one, &two);
        let value = |name: &str| settings.iter().find(|s| s.name == name).unwrap().value();
        assert_eq!(value("OSD2_ALTITUDE_EN"), 1.0);
        assert_eq!(value("OSD2_ALTITUDE_X"), 23.0);
        assert_eq!(value("OSD2_ALTITUDE_Y"), 8.0);
        // BAT_VOLT is not on screen 2: nothing of it is pasted there.
        assert!(!settings.iter().any(|s| s.name.starts_with("OSD2_BAT")));
        // Onto itself: nothing.
        let before = settings.clone();
        copy_screen(&mut settings, &one, &one);
        assert_eq!(settings, before);
    }

    #[test]
    fn a_screen_and_index_are_read_as_the_regex_reads_them() {
        assert_eq!(parse_screen_and_index("OSD5_PARAM1_EN"), Some((5, 1)));
        assert_eq!(parse_screen_and_index("OSD6_PARAM9_INCR"), Some((6, 9)));
        assert_eq!(parse_screen_and_index("OSD1_ALTITUDE_X"), None);
        assert_eq!(parse_screen_and_index("OSD1_ASPD1_X"), Some((1, 1)));
        assert_eq!(parse_screen_and_index("OSD_TYPE"), None);
    }

    #[test]
    fn captions_are_the_samples_the_name_or_the_slot_and_digit_points_are_packed() {
        let settings = OnboardOsd::settings_of(&vehicle());
        let config = create(&settings, 1..=6);
        let altitude = &config.screens[0].items[0];
        let slots = SlotNames::default();
        let (text, offset) = caption(altitude, &settings, CaptionMode::Realistic, &slots);
        assert_eq!(text, "11\u{B1}");
        assert_eq!(offset, -2);
        assert_eq!(
            caption(altitude, &settings, CaptionMode::Names, &slots),
            ("ALTITUDE".to_owned(), 0)
        );
        // BAT_VOLT: the battery symbol after full, "11.8" packed, the volt sign.
        let (volt, _) = caption(
            &config.screens[0].items[1],
            &settings,
            CaptionMode::Realistic,
            &slots,
        );
        let chars: Vec<u32> = volt.chars().map(|c| c as u32).collect();
        assert_eq!(chars, [0x91, '1' as u32, 192 + 1, 208 + 8, 0x06]);
        // A name not in the table is itself.
        assert_eq!(common_caption("MESSAGE"), ("MESSAGE".to_owned(), 0));
        assert_eq!(common_caption("HORIZON").1, 4);
        assert_eq!(common_caption("HORIZON").0.chars().count(), 9);
        // "50.3534305" packs "0.3" into two glyphs: twelve glyphs where the text has thirteen.
        assert_eq!(common_caption("GPSLAT").0.chars().count(), 12);
        // The parameter screen's item: the slot's name, or NOT SET.
        let slot = &config.screens[2].items[0];
        assert_eq!(
            caption(slot, &settings, CaptionMode::Realistic, &slots).0,
            "PARAM1 (NOT SET)"
        );
        let mut named = SlotNames::default();
        named.set(vec![(5, 1, Some("RTL_ALT".to_owned()))]);
        assert_eq!(
            caption(slot, &settings, CaptionMode::Realistic, &named).0,
            "RTL_ALT"
        );
        assert_eq!(pack_digit_points("1.2.3"), "\u{C1}\u{D2}.3");
    }

    #[test]
    fn the_geometry_follows_decrease_and_hd_layout() {
        let sd = Geometry {
            reduced: false,
            high_def: false,
        };
        assert_eq!(sd.char_size(), (24, 36));
        assert_eq!(sd.canvas_size(), (720, 576));
        assert_eq!(sd.to_osd_location((-5, 37)), (0, 1));
        assert_eq!(sd.to_osd_location((47, 0)), (1, 0));
        assert_eq!(sd.to_screen_point((23, 8), -2), (600, 288));
        let hd = Geometry {
            reduced: false,
            high_def: true,
        };
        assert_eq!(hd.char_size(), (18, 27));
        assert_eq!(hd.canvas_size(), (1080, 594));
        let small = Geometry {
            reduced: true,
            high_def: true,
        };
        assert_eq!(small.char_size(), (12, 18));
        assert_eq!(small.canvas_size(), (720, 396));
        let rect = sd.to_screen_rectangle((1, 1), 3, 0);
        assert_eq!(rect, (24, 36, 72, 36));
        assert!(sd.contains(rect, (24, 36)));
        assert!(sd.contains(rect, (95, 71)));
        assert!(!sd.contains(rect, (96, 36)));
    }

    #[test]
    fn the_rows_stack_enabled_over_x_over_y_over_the_rest_with_their_controls() {
        let settings = OnboardOsd::settings_of(&vehicle());
        let config = create(&settings, 1..=6);
        let item = &config.screens[2].items[0];
        let rows = rows(&settings, &item.options);
        let names: Vec<&str> = rows
            .iter()
            .map(|(i, _)| settings[*i].name.as_str())
            .collect();
        assert_eq!(
            names,
            [
                "OSD5_PARAM1_EN",
                "OSD5_PARAM1_X",
                "OSD5_PARAM1_Y",
                "OSD5_PARAM1_KEY",
                "OSD5_PARAM1_TYPE",
                "OSD5_PARAM1_MIN",
                "OSD5_PARAM1_MAX",
                "OSD5_PARAM1_INCR"
            ]
        );
        assert_eq!(rows[0].1, Control::Bool);
        assert_eq!(rows[1].1, Control::Spin);
        assert_eq!(rows[4].1, Control::Dropdown(&ITEM_TYPES));
        assert_eq!(rows[5].1, Control::Int);
        assert_eq!(
            Control::for_setting("OSD_FONT").0,
            Control::Dropdown(&FONTS)
        );
        assert_eq!(
            Control::for_setting("OSD_OPTIONS").0,
            Control::Bitwise(&OPTION_BITS)
        );
        assert_eq!(Control::for_setting("OSD_H_OFFSET").0, Control::Spin);
        assert_eq!(Control::for_setting("OSD1_ENABLE").0, Control::Bool);
        assert_eq!(Control::for_setting("OSD_TYPE").0, Control::Int);
        assert_eq!(Control::Bitwise(&OPTION_BITS).height(), 125.0);
    }

    #[test]
    fn the_font_decodes_and_the_canvas_draws_the_enabled_items() {
        let font = Font::get().expect("clarity.png decodes");
        // The sheet: 16 columns of 12 and a pixel between is 207 wide.
        assert_eq!(font.sheet.dimensions(), (207, 303));
        // Glyph 0x01 (the RSSI symbol) has ink; its cell starts at (13, 0).
        let inked = (0..12)
            .flat_map(|x| (0..18).map(move |y| (x, y)))
            .any(|(x, y)| font.pixel(1, x, y)[3] > 0);
        assert!(inked);
        let geometry = Geometry {
            reduced: true,
            high_def: false,
        };
        let drawn = [Drawn {
            name: "ALTITUDE".to_owned(),
            caption: "11\u{B1}".to_owned(),
            position: (1, 1),
            x_offset: 0,
        }];
        let image = render(Some(font), geometry, &drawn);
        assert_eq!(image.dimensions(), (360, 288));
        // The background where nothing is drawn, the NTSC line at row 13.
        assert_eq!(image.get_pixel(0, 0).0, BACKGROUND);
        assert_eq!(image.get_pixel(100, 18 * 13).0, DIM_GRAY);
        // Something other than the background inside the glyph cells.
        let cell = (12..48).flat_map(|x| (18..36).map(move |y| (x, y)));
        assert!(
            cell.into_iter()
                .any(|(x, y)| image.get_pixel(x, y).0 != BACKGROUND)
        );
        // The same at full size is twice as large, scaled.
        let full = render(
            Some(font),
            Geometry {
                reduced: false,
                high_def: false,
            },
            &drawn,
        );
        assert_eq!(full.dimensions(), (720, 576));
        let hd = render(
            Some(font),
            Geometry {
                reduced: false,
                high_def: true,
            },
            &[],
        );
        assert_eq!(hd.get_pixel(18 * 50, 10).0, DIM_GRAY, "the 50-column line");
        assert_eq!(hd.get_pixel(10, 27 * 18).0, DIM_GRAY, "the 18-row line");
    }

    #[test]
    fn a_press_selects_the_item_under_it_and_a_drag_moves_it_a_cell_at_a_time() {
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        page.choose_tab(Tab::Screen(0));
        // ALTITUDE at (23, 8) with its caption's offset of -2: drawn from cell 25, three cells
        // of 24x36.
        assert!(!page.press(0, (0, 0)));
        assert!(
            !page.press(0, (23 * 24 + 3, 8 * 36 + 3)),
            "its own cell is bare"
        );
        assert!(page.press(0, (25 * 24 + 3, 8 * 36 + 3)));
        assert_eq!(page.screens[0].selected, Some(0));
        // `moveShift`: the item's cell less the pointer.
        assert_eq!(
            page.drag,
            Some((23 * 24 - (25 * 24 + 3), 8 * 36 - (8 * 36 + 3)))
        );
        let altitude = |page: &OnboardOsd| page.config.screens[0].items[0].position(&page.settings);
        // The pointer one cell right: X 24.
        assert!(page.drag(0, (26 * 24 + 3, 8 * 36 + 3)));
        assert_eq!(altitude(&page), (24, 8));
        assert!(!page.drag(0, (26 * 24 + 5, 8 * 36 + 3)), "the same cell");
        // Never left of the screen.
        assert!(page.drag(0, (-500, -500)));
        assert_eq!(altitude(&page), (0, 0));
        page.release();
        assert!(!page.drag(0, (100, 100)));
        assert_eq!(page.changed_count(), 2);
        // The disabled item is never hit.
        assert!(!page.press(0, (24 * 24 + 1, 36 + 1)));
    }

    #[test]
    fn the_editor_buttons_clear_copy_paste_and_toggle() {
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        assert_eq!(page.drawn(0).len(), 1);
        page.toggle(
            page.config.screens[0].items[1]
                .enabled(&page.settings)
                .unwrap(),
        );
        assert_eq!(page.drawn(0).len(), 2);
        page.copy_layout(0);
        assert_eq!(page.copied, Some(0));
        page.paste_layout(1);
        let value = |page: &OnboardOsd, name: &str| {
            page.settings
                .iter()
                .find(|s| s.name == name)
                .unwrap()
                .value()
        };
        assert_eq!(value(&page, "OSD2_ALTITUDE_EN"), 1.0);
        assert_eq!(value(&page, "OSD2_ALTITUDE_X"), 23.0);
        page.clear_all(0);
        assert_eq!(page.drawn(0).len(), 0);
        page.set_show_names(true);
        page.set_reduced(0, true);
        page.set_high_def(0, true);
        assert_eq!(page.screens[0].geometry.char_size(), (12, 18));
        let options = page.config.options.clone();
        let bits = options
            .iter()
            .copied()
            .find(|&i| page.settings[i].name == "OSD_OPTIONS")
            .unwrap();
        page.toggle_bit(bits, 1);
        assert_eq!(value(&page, "OSD_OPTIONS"), 7.0);
        page.toggle_bit(bits, 0);
        assert_eq!(value(&page, "OSD_OPTIONS"), 6.0);
        let font = options
            .iter()
            .copied()
            .find(|&i| page.settings[i].name == "OSD_FONT")
            .unwrap();
        page.choose(font, 4);
        assert_eq!(value(&page, "OSD_FONT"), 4.0);
        assert_eq!(page.combo(font, &FONTS).selected, Some(4));
        // Typing into an int box: a double taken, nonsense left.
        let kind = options
            .iter()
            .copied()
            .find(|&i| page.settings[i].name == "OSD_TYPE")
            .unwrap();
        page.begin_text(TextTarget::Setting(kind));
        assert_eq!(page.field.value(), "1");
        page.field.set("2.5");
        page.leave_boxes();
        assert_eq!(value(&page, "OSD_TYPE"), 2.5);
        page.begin_text(TextTarget::Setting(kind));
        page.field.set("x");
        page.leave_boxes();
        assert_eq!(value(&page, "OSD_TYPE"), 2.5);
        // The bitwise box takes an int only.
        page.begin_text(TextTarget::Setting(bits));
        page.field.set("1.5");
        page.leave_boxes();
        assert_eq!(value(&page, "OSD_OPTIONS"), 6.0);
        page.begin_text(TextTarget::Setting(bits));
        page.field.set("3");
        page.leave_boxes();
        assert_eq!(value(&page, "OSD_OPTIONS"), 3.0);
        assert_eq!(value_text(3.0), "3");
        assert_eq!(value_text(0.001), "0.001");
    }

    #[test]
    fn discard_puts_every_setting_back_after_its_question() {
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        page.toggle(
            page.config.screens[0].items[0]
                .enabled(&page.settings)
                .unwrap(),
        );
        assert_eq!(page.changed_count(), 1);
        page.click_discard();
        assert_eq!(page.question, Some(Question::Discard));
        let telemetry = Telemetry::idle();
        let view = TelemetryView::disconnected("test");
        page.answer(false, &telemetry, &view, None);
        assert_eq!(page.changed_count(), 1, "Cancel keeps the change");
        page.click_discard();
        page.answer(true, &telemetry, &view, None);
        assert_eq!(page.changed_count(), 0);
        assert!(page.config.screens[0].items[0].is_enabled(&page.settings));
    }

    #[test]
    fn write_says_nothing_to_write_or_not_connected_on_the_status_line() {
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        let telemetry = Telemetry::idle();
        page.write(&telemetry, false, false);
        assert_eq!(page.take_status().as_deref(), Some(NO_CHANGES));
        page.write(&telemetry, false, true);
        assert_eq!(page.take_status(), None, "silent");
        page.toggle(
            page.config.screens[0].items[0]
                .enabled(&page.settings)
                .unwrap(),
        );
        page.write(&telemetry, false, false);
        assert_eq!(
            page.take_status().as_deref(),
            Some("Error: Your are not connected")
        );
        assert!(page.writing.is_none());
        // Connected but with no vehicle to write to: every name fails, said once.
        page.write(&telemetry, true, false);
        let view = TelemetryView::disconnected("test");
        page.tick(&telemetry, &view, Instant::now());
        assert_eq!(
            page.take_status().as_deref(),
            Some("Write Failed for 1 params: OSD1_ALTITUDE_EN")
        );
        assert!(page.writing.is_none());
    }

    #[test]
    fn refresh_asks_about_changes_then_about_arming_and_fetches() {
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        let telemetry = Telemetry::idle();
        let mut view = TelemetryView::disconnected("test");
        view.connected = true;
        page.toggle(
            page.config.screens[0].items[0]
                .enabled(&page.settings)
                .unwrap(),
        );
        page.click_refresh(&telemetry, &view, None);
        assert_eq!(page.question, Some(Question::ResetChanges));
        page.answer(false, &telemetry, &view, None);
        assert!(!page.refreshing());
        page.click_refresh(&telemetry, &view, None);
        page.answer(true, &telemetry, &view, None);
        assert!(page.refreshing(), "disarmed: straight to the fetch");
        // Armed: the Update Params question, unless its Show me again was cleared.
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        let mut state = mp_vehicle::VehicleState::default();
        state.armed = true;
        view.state = Some(Arc::new(state));
        page.click_refresh(&telemetry, &view, None);
        assert_eq!(page.question, Some(Question::RefreshArmed(true)));
        assert_eq!(page.toggle_show_again(), Some("False"));
        page.answer(true, &telemetry, &view, None);
        assert!(page.refreshing());
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        page.click_refresh(&telemetry, &view, Some("False"));
        assert_eq!(page.question, None);
        assert!(page.refreshing());
        // The list back, whole and new: activated over it.
        view.parameters = vehicle().into();
        view.parameters_expected = 28;
        page.tick(&telemetry, &view, Instant::now());
        assert!(!page.refreshing());
        assert_eq!(page.settings.len(), 28);
        // Not connected: nothing happens on the click.
        let mut page = OnboardOsd::default();
        page.activate(&vehicle());
        view.connected = false;
        page.click_refresh(&telemetry, &view, None);
        assert!(!page.refreshing());
    }
}
