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

//! The display view: `MainV2.DisplayConfiguration`, an `ExtLibs/Utilities/DisplayView.cs`, whose
//! switches say which pages the SETUP and CONFIG lists add (`displayXxx`) and whether their
//! advanced pages show (`isAdvancedMode`).
//!
//! Three views, `DisplayNames`: Basic and Advanced are presets (`DisplayView.cs:276-436`); Custom
//! is the file `custom.displayview`, JSON or XML, read over a view as the constructor makes it,
//! and the Advanced preset when the file is not there or not readable (`:442-455`). The Planner
//! page's Layout box picks one (`ConfigPlanner.cs:1019-1034`), and every change is saved in
//! `config.xml` as `displayview`, the view as indented JSON (`MainV2.cs:357-366`).
//!
//! At start-up the view is Custom when the file exists and Advanced otherwise (`MainV2.cs:351-353`);
//! a true `advancedview` from an old config makes it Advanced and is removed; then a saved
//! `displayview` replaces it - a string that does not read is a view as the constructor makes it -
//! with Standard Params and Advanced Params forced off and the Full Parameter List on
//! (`MainV2.cs:900-932`). So Standard and Advanced Params appear only when a Custom view that
//! turns them on is chosen in the session.
//!
//! The lists read the view when they are built, which is when their screen is shown
//! (`MainSwitcher.ShowScreen` makes a SETUP or CONFIG screen anew each time,
//! `MainV2.cs:3186-3187`; `ExtLibs/Controls/MainSwitcher.cs:112-153`): a change shows in a list
//! when its screen is shown again - its tab clicked, the one showing included. `LayoutChanged`
//! sets `BackstageView.Advanced` at once (`MainV2.cs:597-601`), so a list's advanced pages follow
//! the view when it is next drawn.
//!
//! Where this differs from the C#, and why:
//!
//! * `custom.displayview` is read beside `config.xml`, in the user data directory, not beside
//!   the executable (`Settings.GetRunningDirectory()`): this application's executable directory
//!   is a build directory, as `logo.png`'s is (`mp_settings::migrate`);
//! * the view is the UI thread's, as the C#'s static is read on its UI thread; a thread of its
//!   own for each test keeps the tests apart;
//! * the switches of the screens this application draws without them - the FLIGHT DATA tabs, the
//!   planning screen's menus, the Simulation and Help buttons - are kept, read and written, and
//!   change nothing here: `updateLayout`'s calls into those screens (`MainV2.cs:597-630`) are not
//!   ported.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use mp_os::fs::FsExt as _;
use std::cell::RefCell;
use std::path::{Path, PathBuf};

use crate::settings::Persisted;

/// `DisplayNames`.
/// `// C#: ExtLibs/Utilities/DisplayView.cs:14-20`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayName {
    /// `Basic`, 0.
    Basic,
    /// `Advanced`, 1.
    Advanced,
    /// `Custom`, 2.
    Custom,
}

impl DisplayName {
    /// The three, in `DisplayNames`' order: the Layout box's items.
    pub const ALL: [Self; 3] = [Self::Basic, Self::Advanced, Self::Custom];

    /// Its name.
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::Basic => "Basic",
            Self::Advanced => "Advanced",
            Self::Custom => "Custom",
        }
    }

    /// Its value, as Json.NET writes an enum.
    #[must_use]
    pub const fn number(self) -> u8 {
        match self {
            Self::Basic => 0,
            Self::Advanced => 1,
            Self::Custom => 2,
        }
    }

    /// From a value or a name, as Json.NET and `XmlSerializer` read them.
    fn parse(value: &serde_json::Value) -> Option<Self> {
        match value {
            serde_json::Value::Number(number) => Self::ALL
                .into_iter()
                .find(|name| number.as_u64() == Some(u64::from(name.number()))),
            serde_json::Value::String(text) => Self::ALL
                .into_iter()
                .find(|name| name.text().eq_ignore_ascii_case(text.trim()))
                .or_else(|| {
                    let number: u64 = text.trim().parse().ok()?;
                    Self::ALL
                        .into_iter()
                        .find(|name| u64::from(name.number()) == number)
                }),
            _ => None,
        }
    }
}

/// Every switch, in the order the class declares them - the order Json.NET writes them - with
/// its value as the constructor leaves it (the property's initialiser, then the constructor),
/// in `Basic()` and in `Advanced()` (the constructor's values, then the preset's initialiser).
/// A test holds it to the C#.
/// `// C#: ExtLibs/Utilities/DisplayView.cs:37-212, 276-436`
pub const PROPERTIES: &[(&str, bool, bool, bool)] = &[
    ("displayRTKInject", true, true, true),
    ("displayGPSOrder", true, true, true),
    ("displayHWIDs", true, true, true),
    ("displayADSB", true, true, true),
    ("displaySimulation", false, true, true),
    ("displayTerminal", true, false, true),
    ("displayDonate", true, true, true),
    ("displayHelp", true, true, true),
    ("displayAnenometer", true, true, true),
    ("displayQuickTab", true, true, true),
    ("displayPreFlightTab", true, true, true),
    ("displayAdvActionsTab", false, false, true),
    ("displaySimpleActionsTab", true, true, false),
    ("displayGaugesTab", true, true, true),
    ("displayStatusTab", false, false, true),
    ("displayServoTab", false, false, true),
    ("displayScriptsTab", false, false, true),
    ("displayTelemetryTab", true, true, true),
    ("displayDataflashTab", true, true, true),
    ("displayMessagesTab", true, true, true),
    ("displayTransponderTab", true, true, true),
    ("displayAuxFunctionTab", true, true, true),
    ("displayPayloadTab", true, true, true),
    ("displayRallyPointsMenu", true, true, true),
    ("displayGeoFenceMenu", true, true, true),
    ("displaySplineCircleAutoWp", true, true, true),
    ("displayTextAutoWp", true, true, true),
    ("displayCircleSurveyAutoWp", true, true, true),
    ("displayPoiMenu", true, true, true),
    ("displayTrackerHomeMenu", true, true, true),
    ("displayCheckHeightBox", true, true, true),
    ("displayPluginAutoWp", true, true, true),
    ("displayInstallFirmware", true, true, true),
    ("displayWizard", true, true, true),
    ("displayFrameType", true, true, true),
    ("displayAccelCalibration", true, true, true),
    ("displayCompassConfiguration", true, true, true),
    ("displayRadioCalibration", true, true, true),
    ("displayEscCalibration", true, true, true),
    ("displayFlightModes", true, true, true),
    ("displayFailSafe", true, true, true),
    ("displaySikRadio", true, true, true),
    ("displayBattMonitor", true, true, true),
    ("displayCAN", true, true, true),
    ("displayCompassMotorCalib", true, true, true),
    ("displayRangeFinder", true, true, true),
    ("displayAirSpeed", true, true, true),
    ("displayPx4Flow", true, true, true),
    ("displayOpticalFlow", true, true, true),
    ("displayOsd", true, true, true),
    ("displayCameraGimbal", true, true, true),
    ("displayMotorTest", true, true, true),
    ("displayBluetooth", true, true, true),
    ("displayParachute", true, true, true),
    ("displayEsp", true, true, true),
    ("displayAntennaTracker", true, true, true),
    ("displaySerialPorts", true, true, true),
    ("displayGeoFence", true, true, true),
    ("displayBasicTuning", true, true, true),
    ("displayExtendedTuning", true, true, true),
    ("displayStandardParams", false, false, false),
    ("displayAdvancedParams", false, false, false),
    ("displayMavFTP", true, true, true),
    ("displayFullParamList", true, true, true),
    ("displayFullParamTree", true, true, true),
    ("displayParamCommitButton", false, false, false),
    ("displayBaudCMB", true, true, true),
    ("displaySerialPortCMB", true, true, true),
    ("standardFlightModesOnly", false, false, false),
    ("autoHideMenuForce", false, false, false),
    ("displayInitialParams", true, true, true),
    ("isAdvancedMode", false, false, true),
    ("displayREPL", true, true, true),
    ("displayServoOutput", true, true, true),
    ("displayJoystick", true, true, true),
    ("displayOSD", true, true, true),
    ("displayUserParam", true, true, true),
    ("displayPlannerSettings", true, true, true),
    ("displayFFTSetup", true, true, true),
    ("displayPreFlightTabEdit", true, true, true),
    ("displayPlannerLayout", true, true, true),
    ("lockQuickView", false, false, false),
];

/// Where `displayName` is among the properties: after `displayADSB`.
/// `// C#: ExtLibs/Utilities/DisplayView.cs:37-41`
pub const NAME_AT: usize = 4;

/// The file a Custom view is read from.
/// `// C#: ExtLibs/Utilities/DisplayView.cs:442`
pub const CUSTOM_FILE: &str = "custom.displayview";

/// The `config.xml` key the view is saved under.
/// `// C#: MainV2.cs:363; GCSViews/ConfigurationView/ConfigPlanner.cs:1033`
pub const SETTING: &str = "displayview";

/// A `DisplayView`: its name and its switches, in [`PROPERTIES`]' order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayView {
    /// `displayName`.
    pub name: DisplayName,
    values: Vec<bool>,
}

impl Default for DisplayView {
    /// `new DisplayView()`: "default to basic", every switch as the constructor leaves it.
    /// `// C#: ExtLibs/Utilities/DisplayView.cs:135-212`
    fn default() -> Self {
        Self {
            name: DisplayName::Basic,
            values: PROPERTIES.iter().map(|(_, value, ..)| *value).collect(),
        }
    }
}

impl DisplayView {
    /// `Basic()`.
    /// `// C#: ExtLibs/Utilities/DisplayView.cs:276-355`
    #[must_use]
    pub fn basic() -> Self {
        Self {
            name: DisplayName::Basic,
            values: PROPERTIES.iter().map(|(_, _, value, _)| *value).collect(),
        }
    }

    /// `Advanced()`.
    /// `// C#: ExtLibs/Utilities/DisplayView.cs:356-436`
    #[must_use]
    pub fn advanced() -> Self {
        Self {
            name: DisplayName::Advanced,
            values: PROPERTIES.iter().map(|(_, _, _, value)| *value).collect(),
        }
    }

    /// `Custom()`: the file read, named Custom; the Advanced preset - named Advanced - when there
    /// is no file or it does not read.
    /// `// C#: ExtLibs/Utilities/DisplayView.cs:444-455`
    #[must_use]
    pub fn custom(path: Option<&Path>) -> Self {
        let read = path
            .filter(|path| path.os_is_file())
            .and_then(|path| mp_os::fs::read_to_string(path).ok())
            .and_then(|text| Self::try_parse(&text));
        match read {
            Some(mut view) => {
                view.name = DisplayName::Custom;
                view
            }
            None => Self::advanced(),
        }
    }

    /// A preset by its name, as the Layout box's handler makes it.
    /// `// C#: GCSViews/ConfigurationView/ConfigPlanner.cs:1019-1032`
    #[must_use]
    pub fn named(name: DisplayName, custom: Option<&Path>) -> Self {
        match name {
            DisplayName::Basic => Self::basic(),
            DisplayName::Advanced => Self::advanced(),
            DisplayName::Custom => Self::custom(custom),
        }
    }

    /// A switch; one the class does not have is off.
    #[must_use]
    pub fn get(&self, flag: &str) -> bool {
        PROPERTIES
            .iter()
            .position(|(name, ..)| *name == flag)
            .and_then(|index| self.values.get(index))
            .copied()
            .unwrap_or(false)
    }

    /// Sets a switch.
    pub fn set(&mut self, flag: &str, value: bool) {
        if let Some(slot) = PROPERTIES
            .iter()
            .position(|(name, ..)| *name == flag)
            .and_then(|index| self.values.get_mut(index))
        {
            *slot = value;
        }
    }

    /// `DisplayViewExtensions.TryParse`: XML unless the text starts with `{`, then JSON; each over
    /// a view as the constructor makes it, so a switch the text leaves out keeps that value.
    /// `// C#: ExtLibs/Utilities/DisplayView.cs:227-253`
    #[must_use]
    pub fn try_parse(text: &str) -> Option<Self> {
        if text.starts_with('{') {
            Self::from_json(text)
        } else {
            Self::from_xml(text)
        }
    }

    /// `FromJSON<DisplayView>()`: Json.NET matches a property's name whatever its case, takes a
    /// boolean or its text, skips a name it does not know, and throws on anything else.
    fn from_json(text: &str) -> Option<Self> {
        let serde_json::Value::Object(map) = serde_json::from_str(text).ok()? else {
            return None;
        };
        let mut view = Self::default();
        for (key, value) in &map {
            if key.eq_ignore_ascii_case("displayName") {
                view.name = DisplayName::parse(value)?;
                continue;
            }
            let Some(index) = PROPERTIES
                .iter()
                .position(|(name, ..)| name.eq_ignore_ascii_case(key))
            else {
                continue;
            };
            let flag = match value {
                serde_json::Value::Bool(flag) => *flag,
                serde_json::Value::String(text) => {
                    match text.trim().to_ascii_lowercase().as_str() {
                        "true" => true,
                        "false" => false,
                        _ => return None,
                    }
                }
                _ => return None,
            };
            if let Some(slot) = view.values.get_mut(index) {
                *slot = flag;
            }
        }
        Some(view)
    }

    /// `XmlSerializer.Deserialize`: a `DisplayView` element whose children name the properties
    /// exactly, `true`/`false` or `1`/`0` for a switch; an element it does not know is skipped.
    fn from_xml(text: &str) -> Option<Self> {
        let document = roxmltree::Document::parse(text).ok()?;
        let root = document.root_element();
        if root.tag_name().name() != "DisplayView" {
            return None;
        }
        let mut view = Self::default();
        for element in root.children().filter(roxmltree::Node::is_element) {
            let key = element.tag_name().name();
            let value = element.text().unwrap_or("").trim();
            if key == "displayName" {
                view.name = DisplayName::ALL
                    .into_iter()
                    .find(|name| name.text() == value)?;
                continue;
            }
            let Some(index) = PROPERTIES.iter().position(|(name, ..)| *name == key) else {
                continue;
            };
            let flag = match value {
                "true" | "1" => true,
                "false" | "0" => false,
                _ => return None,
            };
            if let Some(slot) = view.values.get_mut(index) {
                *slot = flag;
            }
        }
        Some(view)
    }

    /// `ConvertToString`: `ToJSON()`, Json.NET's indented JSON - two spaces, the properties in
    /// the class's order, the name as its number, lines ended with `Environment.NewLine`, which is
    /// `\n` on this platform as under Mono.
    /// `// C#: ExtLibs/Utilities/DisplayView.cs:255-265; ExtLibs/Utilities/Extensions.cs:413-430`
    #[must_use]
    pub fn convert_to_string(&self) -> String {
        let mut lines: Vec<String> = PROPERTIES
            .iter()
            .zip(&self.values)
            .map(|((name, ..), value)| format!("  \"{name}\": {value}"))
            .collect();
        lines.insert(
            NAME_AT.min(lines.len()),
            format!("  \"displayName\": {}", self.name.number()),
        );
        format!("{{\n{}\n}}", lines.join(",\n"))
    }
}

thread_local! {
    /// `MainV2._displayConfiguration`.
    static CURRENT: RefCell<DisplayView> = RefCell::new(DisplayView::advanced());
}

/// A switch of the view showing: `MainV2.DisplayConfiguration.<flag>`.
#[must_use]
pub fn flag(name: &str) -> bool {
    CURRENT.with(|view| view.borrow().get(name))
}

/// The view showing.
#[must_use]
pub fn current() -> DisplayView {
    CURRENT.with(|view| view.borrow().clone())
}

/// The `DisplayConfiguration` setter: the view kept, and saved as `displayview`.
/// `// C#: MainV2.cs:357-366`
pub fn set(view: DisplayView, settings: &mut Persisted) {
    settings.set(SETTING, view.convert_to_string());
    CURRENT.with(|current| *current.borrow_mut() = view);
}

/// Where a Custom view is read from: beside `config.xml`.
#[must_use]
pub fn custom_path(settings: &Persisted) -> Option<PathBuf> {
    settings.directory().map(|dir| dir.join(CUSTOM_FILE))
}

/// `MainV2`'s start-up: the static's view, the old `advancedview`, then the saved `displayview`
/// with the parameter pages forced as the C# forces them.
/// `// C#: MainV2.cs:351-353, 900-935`
pub fn start(settings: &mut Persisted) {
    let custom = custom_path(settings);
    let first = if custom.as_deref().is_some_and(Path::is_file) {
        DisplayView::custom(custom.as_deref())
    } else {
        DisplayView::advanced()
    };
    CURRENT.with(|current| *current.borrow_mut() = first);
    if let Some(old) = settings.get("advancedview").map(str::to_owned) {
        // `GetBoolean`: `bool.TryParse`, whatever the case.
        if old.trim().eq_ignore_ascii_case("true") {
            set(DisplayView::advanced(), settings);
        }
        settings.remove("advancedview");
    }
    if let Some(saved) = settings.get(SETTING).map(str::to_owned) {
        // `GetDisplayView`: a string that does not read is `new DisplayView()`.
        let view = DisplayView::try_parse(&saved).unwrap_or_default();
        set(view, settings);
        CURRENT.with(|current| {
            let mut current = current.borrow_mut();
            current.set("displayAdvancedParams", false);
            current.set("displayStandardParams", false);
            current.set("displayFullParamList", true);
        });
    }
}

/// Facts a UI test asserts on.
pub fn record_facts() {
    use crate::facts::record;
    let view = current();
    record("display.view", view.name.text());
    for flag in [
        "displayStandardParams",
        "displayAdvancedParams",
        "isAdvancedMode",
        "displayTerminal",
    ] {
        record(format!("display.{flag}"), view.get(flag));
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::config_coverage::source::csharp;

    /// The code in a block: from the first `{` after `marker` to its `}`.
    fn block_after<'a>(source: &'a str, marker: &str) -> &'a str {
        let start = source.find(marker).unwrap_or_else(|| panic!("{marker}"));
        let open = start + source[start..].find('{').expect("a block");
        let mut depth = 0_i32;
        for (at, c) in source[open..].char_indices() {
            match c {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return &source[open + 1..open + at];
                    }
                }
                _ => {}
            }
        }
        panic!("the block after {marker} does not close");
    }

    /// `name = true` and `name = false`, with `;` or `,` after.
    fn assignments(block: &str) -> BTreeMap<String, bool> {
        block
            .lines()
            .filter_map(|line| {
                let (name, value) = line.trim().split_once(" = ")?;
                match value.trim_end_matches([';', ',']).trim() {
                    "true" => Some((name.trim().to_owned(), true)),
                    "false" => Some((name.trim().to_owned(), false)),
                    _ => None,
                }
            })
            .collect()
    }

    /// The table is the class: every property in its order, with the values the constructor and
    /// the two presets give it.
    #[test]
    fn the_table_is_the_csharps() {
        let Some(source) = csharp("ExtLibs/Utilities/DisplayView.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let mut order = Vec::new();
        let mut declared: BTreeMap<String, bool> = BTreeMap::new();
        for line in source.lines() {
            let line = line.trim();
            if line.starts_with("public DisplayNames displayName") {
                order.push("displayName".to_owned());
            }
            let Some(rest) = line
                .strip_prefix("public bool ")
                .or_else(|| line.strip_prefix("public Boolean "))
            else {
                continue;
            };
            if let Some((name, tail)) = rest.split_once(" { get; set; }") {
                order.push(name.to_owned());
                declared.insert(name.to_owned(), tail.trim() == "= true;");
            }
        }
        let mut constructed = declared;
        constructed.extend(assignments(block_after(&source, "public DisplayView()")));
        let preset = |marker: &str| {
            let mut values = constructed.clone();
            values.extend(assignments(block_after(&source, marker)));
            values
        };
        let basic = preset("public static DisplayView Basic(this DisplayView v)");
        let advanced = preset("public static DisplayView Advanced(this DisplayView v)");
        let ours: Vec<&str> = PROPERTIES.iter().map(|(name, ..)| *name).collect();
        let theirs: Vec<&str> = order
            .iter()
            .map(String::as_str)
            .filter(|name| *name != "displayName")
            .collect();
        assert_eq!(ours, theirs);
        assert_eq!(
            order.iter().position(|name| name == "displayName"),
            Some(NAME_AT)
        );
        for (name, default, in_basic, in_advanced) in PROPERTIES {
            assert_eq!(constructed.get(*name), Some(default), "{name} constructed");
            assert_eq!(basic.get(*name), Some(in_basic), "{name} in Basic");
            assert_eq!(advanced.get(*name), Some(in_advanced), "{name} in Advanced");
        }
    }

    /// Start-up, as `MainV2` has it: Advanced with no file and nothing saved.
    #[test]
    fn start_up_is_advanced_without_a_file_or_a_setting() {
        let dir = scratch("start");
        let mut settings = Persisted::at(Some(dir.join("config.xml")));
        start(&mut settings);
        assert_eq!(current(), DisplayView::advanced());
        assert!(settings.get(SETTING).is_none(), "nothing written");
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// With the file, the view is Custom; a saved `displayview` then replaces it, with the two
    /// parameter pages forced off - so they show only once Custom is chosen again.
    #[test]
    fn a_saved_view_is_loaded_with_the_parameter_pages_off() {
        let dir = scratch("saved");
        mp_os::fs::write(
            dir.join(CUSTOM_FILE),
            "{\"displayStandardParams\": true, \"isAdvancedMode\": true}",
        )
        .expect("the file");
        let mut settings = Persisted::at(Some(dir.join("config.xml")));
        start(&mut settings);
        assert_eq!(current().name, DisplayName::Custom);
        assert!(flag("displayStandardParams"));

        let mut saved = DisplayView::custom(custom_path(&settings).as_deref());
        saved.set("displayFullParamList", false);
        settings.set(SETTING, saved.convert_to_string());
        start(&mut settings);
        assert_eq!(current().name, DisplayName::Custom);
        assert!(!flag("displayStandardParams"), "forced off");
        assert!(flag("displayFullParamList"), "forced on");
        assert!(flag("isAdvancedMode"));
        // What was saved is the view as read, before the forcing.
        let written = settings.get(SETTING).map(str::to_owned).unwrap_or_default();
        assert!(
            DisplayView::try_parse(&written).is_some_and(|view| view.get("displayStandardParams"))
        );

        // An old `advancedview` of true: the setter saves the Advanced view over the saved one,
        // which is then what is read. The old key is removed.
        settings.set("advancedview", "True");
        start(&mut settings);
        assert_eq!(current().name, DisplayName::Advanced);
        assert!(
            settings.get("advancedview").is_none(),
            "the old key removed"
        );

        // A saved string that does not read is the constructor's view: Basic, no advanced pages.
        settings.set(SETTING, "not a view");
        start(&mut settings);
        assert_eq!(current().name, DisplayName::Basic);
        assert!(!flag("isAdvancedMode"));
        set(DisplayView::advanced(), &mut settings);
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// Custom with no file is the Advanced preset, named Advanced; a file that does not read is
    /// too. A partial file keeps the constructor's values for what it leaves out.
    #[test]
    fn custom_reads_the_file_or_is_advanced() {
        let dir = scratch("custom");
        let path = dir.join(CUSTOM_FILE);
        assert_eq!(DisplayView::custom(Some(&path)), DisplayView::advanced());
        mp_os::fs::write(&path, "{ not json").expect("the file");
        assert_eq!(DisplayView::custom(Some(&path)), DisplayView::advanced());
        mp_os::fs::write(&path, "{\"DISPLAYSTANDARDPARAMS\": true}").expect("the file");
        let view = DisplayView::custom(Some(&path));
        assert_eq!(view.name, DisplayName::Custom);
        assert!(view.get("displayStandardParams"));
        assert!(!view.get("isAdvancedMode"), "the constructor's");
        assert!(!view.get("displaySimulation"), "the constructor's");
        mp_os::fs::write(
            &path,
            "<?xml version=\"1.0\"?><DisplayView><displayName>Basic</displayName>\
             <displayAdvancedParams>true</displayAdvancedParams><isAdvancedMode>true</isAdvancedMode>\
             </DisplayView>",
        )
        .expect("the file");
        let view = DisplayView::custom(Some(&path));
        assert_eq!(
            view.name,
            DisplayName::Custom,
            "named Custom whatever it says"
        );
        assert!(view.get("displayAdvancedParams") && view.get("isAdvancedMode"));
        let _ = mp_os::fs::remove_dir_all(&dir);
    }

    /// `ConvertToString` reads back as the same view, with the name as its number in its place.
    #[test]
    fn a_view_written_reads_back() {
        let view = DisplayView::basic();
        let text = view.convert_to_string();
        assert!(text.starts_with("{\n  \"displayRTKInject\": true,\n"));
        assert!(text.contains("\"displayADSB\": true,\n  \"displayName\": 0,\n"));
        assert_eq!(DisplayView::try_parse(&text), Some(view));
        assert!(
            DisplayView::try_parse(" {}").is_none(),
            "not `{{` first: XML, which it is not"
        );
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = mp_os::temp_dir().join(format!(
            "headless-planner-displayview-{name}-{}",
            mp_os::process_id()
        ));
        let _ = mp_os::fs::remove_dir_all(&dir);
        mp_os::fs::create_dir_all(&dir).expect("a scratch folder");
        dir
    }
}
