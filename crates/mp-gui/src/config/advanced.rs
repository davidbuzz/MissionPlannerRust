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

//! Advanced: `GCSViews/ConfigurationView/ConfigAdvanced.cs`, the heading Initial Setup lists last
//! in the Advanced view, with Terminal and Script REPL under it (`GCSViews/InitialSetup.cs:340-353`).
//! It is added whether or not a vehicle is connected, and `Activate` does nothing (`:18-20`).
//!
//! What it shows, as `ConfigAdvanced.resx` places it in a 547 x 584 page: "The following pages are
//! for advanced configutation only. use with caution" at (16, 11), and under it a two-column table
//! at (73, 27), 408 x 520 - thirteen rows of 40 pixels, a button in the left column (23 % of the
//! width, the button filling it less its five-pixel margins) and what it is for in the right.
//!
//! Each button opens a window of its own (`ConfigAdvanced.cs:22-127`): the Warnings Manager, the
//! MAVLink Inspector, the proximity radar, the signing keys, the MAVLink mirror, the NMEA output,
//! Follow Me, the parameter documentation regenerated from ArduPilot's source, the moving base, a
//! log anonymised, the FFT and spectrogram plots, and the support proxy. Each is a form of its own
//! in the C# (`Warnings/WarningsManager.cs`, `Controls/MAVLinkInspector.cs` and the rest, named at
//! [`ROWS`]), a port of its own. Seven are ported, and their buttons open them:
//!
//! * the Warning Manager's, `Warnings/WarningsManager.cs` (`config/warnings_manager.rs`),
//!   `new WarningsManager().Show()`, over the warning engine's rules (`warnings.rs`), held with
//!   SETUP's other small pages (`config/extra_setup.rs`);
//! * FFT's, `Controls/fftui.cs` (`config/fftui.rs`): the same window the FFT Setup page's FFT
//!   opens, held with that page (`config/fft.rs`), since both handlers are the one line
//!   `new fftui().Show()`;
//! * the MAVLink Inspector's, `Controls/MAVLinkInspector.cs` (`config/mavlink_inspector.rs`),
//!   `new MAVLinkInspector(MainV2.comPort).Show()`, held with SETUP's other small pages
//!   (`config/extra_setup.rs`);
//! * the proximity window, `Controls/ProximityControl.cs` (`config/proximity.rs`),
//!   `new ProximityControl(MainV2.comPort.MAV).Show()`, held likewise, and given the keyboard as
//!   it opens, as a new form is activated, for its four keys;
//! * the signing keys window, `Controls/AuthKeys.cs` (`config/auth_keys.rs`),
//!   `new AuthKeys().Show()`, held likewise with the key store;
//! * the spectrogram's, `Controls/SpectrogramUI.cs` (`config/spectrogram.rs`),
//!   `new SpectrogramUI().Show()`, held there too;
//! * the Support Proxy's, `Controls/SerialSupportProxy.cs` (`config/support_proxy.rs`),
//!   `new SerialSupportProxy().Show()`, held there with the mirror it starts, which outlives it;
//!   the form opens with its number having the keyboard (`NUM_port.Select()`).
//!
//! The others are not in this application, so their buttons are drawn dimmed, with the window
//! each would open as the reason. Three of them stay that way by the owner's rulings (PLAN.md §12
//! D13): Follow Me and Moving Base (2026-09-25), and Anon Log (2026-10-02, "beta and not
//! interesting"). The rest are owed.
//!
//! What differs: `Show()` makes a modeless form, and a second click a second form beside the
//! first; each window here is drawn over SETUP, modal, and a second click replaces it with a
//! fresh one - for FFT, of this FFT or the FFT Setup page's, as `config/fft.rs` has it.
//!
//! The colours are this application's.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, Div, SharedString, div, prelude::*, px, rgb};

use super::auth_keys::AuthKeysWindow;
use super::fft::Fft;
use super::mavlink_inspector::InspectorWindow;
use super::mavlink_mirror::MavlinkMirror;
use super::nmea_output::NmeaOutput;
use super::param_gen::ParamGen;
use super::proximity::ProximityWindow;
use super::spectrogram::SpectrogramWindow;
use super::support_proxy::SupportProxy;
use super::warnings_manager::ManagerWindow;
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::ui::{panel, theme};
use crate::warnings::CustomWarning;

/// The page's title in Initial Setup's list: the call's literal.
/// `// C#: GCSViews/InitialSetup.cs:342`
pub const TITLE: &str = "Advanced";

/// `label1.Text`, spelling and all.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.resx (label1.Text)`
pub const TEXT: &str = "The following pages are for advanced configutation only. use with caution";

/// `label1.Location`.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.resx (label1.Location)`
pub const TEXT_AT: (f32, f32) = (16.0, 11.0);

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.resx ($this.Size)`
pub const PAGE_SIZE: (f32, f32) = (547.0, 584.0);

/// `tableLayoutPanel1.Location` and `.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.resx (tableLayoutPanel1.Location, .Size)`
pub const TABLE: (f32, f32, f32, f32) = (73.0, 27.0, 408.0, 520.0);

/// Each row's height: `Absolute,40` thirteen times.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.resx (tableLayoutPanel1.LayoutSettings)`
pub const ROW_HEIGHT: f32 = 40.0;

/// Where the right column starts in the table: the left is `Percent,23.03922` of 408, 94 pixels,
/// and each label sits three pixels in, as its `Location` says.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.resx (tableLayoutPanel1.LayoutSettings, label2.Location)`
pub const LABEL_X: f32 = 97.0;

/// A button's place in its cell: `Margin` five all round, `Size` 84 x 30.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.resx (but_warningmanager.Margin, .Size)`
pub const BUTTON_IN_CELL: (f32, f32, f32, f32) = (5.0, 5.0, 84.0, 30.0);

/// One row of the table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Row {
    /// The button's Designer name.
    pub button: &'static str,
    /// Its `Text`.
    pub text: &'static str,
    /// Its `Click` handler.
    pub handler: &'static str,
    /// The label beside it: its Designer name.
    pub label: &'static str,
    /// The label's `Text`.
    pub label_text: &'static str,
    /// The label's `Size`: the text wraps to it.
    pub label_size: (f32, f32),
    /// What the handler opens, and where it is in the C#.
    pub opens: &'static str,
    /// Where it is ported, under `crates/mp-gui/src/`, for the button that opens it; `None` for
    /// a window this application has not got, whose button is dimmed with `opens` as the reason.
    pub port: Option<&'static str>,
}

impl Row {
    /// Whether the button does anything here.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.port.is_some()
    }

    /// Why a dimmed button is dimmed.
    #[must_use]
    pub fn reason(&self) -> String {
        format!("{} is not ported", self.opens)
    }
}

/// A row, as the table below writes one.
#[allow(clippy::too_many_arguments)] // one per field of the row, in the table's order
const fn row(
    button: &'static str,
    text: &'static str,
    handler: &'static str,
    label: &'static str,
    label_text: &'static str,
    label_size: (f32, f32),
    opens: &'static str,
    port: Option<&'static str>,
) -> Row {
    Row {
        button,
        text,
        handler,
        label,
        label_text,
        label_size,
        opens,
        port,
    }
}

/// The thirteen rows, top to bottom.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:22-127;
/// ConfigAdvanced.resx (tableLayoutPanel1.LayoutSettings, *.Text, label*.Size)`
#[rustfmt::skip]
pub const ROWS: [Row; 13] = [
    row("but_warningmanager", "Warning Manager", "but_warningmanager_Click", "label2",
        "Enable custom warnings based on a set of conditions", (263.0, 18.0),
        "the Warnings Manager window (Warnings/WarningsManager.cs)",
        Some("config/warnings_manager.rs")),
    row("but_mavinspector", "MAVLink Inspector", "but_mavinspector_Click", "label3",
        "View decoded mavlink data being sent and received", (260.0, 18.0),
        "the MAVLink Inspector window (Controls/MAVLinkInspector.cs)",
        Some("config/mavlink_inspector.rs")),
    row("but_proximity", "Proximity", "but_proximity_Click", "label4",
        "View the data from a 360 lidar", (152.0, 18.0),
        "the proximity window (Controls/ProximityControl.cs)", Some("config/proximity.rs")),
    row("but_signkey", "Mavlink Signing", "but_signkey_Click", "label5",
        "Enable mavlink signing to secure communication with the MAV", (307.0, 18.0),
        "the signing keys window (Controls/AuthKeys.cs)", Some("config/auth_keys.rs")),
    row("BUT_outputMavlink", "Mavlink Mirror", "BUT_outputMavlink_Click", "label6",
        "Mavlink mirror to an external location. For Monitoring or control", (304.0, 18.0),
        "the MAVLink mirror window (Controls/SerialOutputPass.cs)",
        Some("config/mavlink_mirror.rs")),
    row("BUT_outputnmea", "NMEA", "BUT_outputnmea_Click", "label7",
        "Output the MAV location as a NMEA string", (213.0, 18.0),
        "the NMEA output window (Controls/SerialOutputNMEA.cs)",
        Some("config/nmea_output.rs")),
    row("BUT_follow_me", "Follow Me", "BUT_follow_me_Click", "label8",
        "Use an external NMEA gps and send guided mode waypoints to the MAV based on that location",
        (304.0, 31.0),
        "the Follow Me window (Controls/FollowMe.cs)", None),
    row("BUT_paramgen", "Param gen", "BUT_paramgen_Click", "label9",
        "Regenerage the param info used inside mp", (214.0, 18.0),
        "regenerating the parameter documentation from ArduPilot's source \
         (ExtLibs/Utilities/ParameterMetaDataParser.cs)",
        Some("config/param_gen.rs")),
    row("BUT_movingbase", "Moving Base", "BUT_movingbase_Click", "label10",
        "Show an extra icon on the map of your current location.", (273.0, 18.0),
        "the moving base window (Controls/MovingBase.cs)", None),
    // Out of scope: the owner's ruling of 2026-10-02 (PLAN.md §12 D13).
    row("but_anonlog", "Anon Log", "but_anonlog_Click", "label11",
        "Scramble lat/lng in bin or tlog", (149.0, 18.0),
        "anonymising a log (ExtLibs/Utilities/Privacy.cs)", None),
    row("but_fft", "FFT", "but_fft_Click", "label12",
        "Plot a FFT from a log", (110.0, 18.0),
        "the FFT window (Controls/fftui.cs)", Some("config/fftui.rs")),
    row("BUT_spect", "Spectrogram", "BUT_spect_Click", "label13",
        "Plot a FFT from a log", (110.0, 18.0),
        "the spectrogram window (Controls/SpectrogramUI.cs)", Some("config/spectrogram.rs")),
    row("BUT_supportproxy", "Support Proxy", "BUT_supportproxy_Click", "label14",
        "Share connection with support engineer", (200.0, 18.0),
        "the support proxy window (Controls/SerialSupportProxy.cs)",
        Some("config/support_proxy.rs")),
];

/// Facts a UI test asserts on: whether the page shows, its text, the buttons and their labels in
/// the table's order, which buttons do something, and each dimmed one and why.
pub fn record_facts(showing: bool) {
    use crate::facts::record;
    record("config.advanced.active", showing);
    record("config.advanced.text", TEXT);
    record(
        "config.advanced.buttons",
        ROWS.iter()
            .map(|row| row.text)
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "config.advanced.labels",
        ROWS.iter()
            .map(|row| row.label_text)
            .collect::<Vec<_>>()
            .join("|"),
    );
    record(
        "config.advanced.buttons.enabled",
        ROWS.iter()
            .filter(|row| row.enabled())
            .map(|row| row.text)
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "config.advanced.dimmed",
        ROWS.iter()
            .filter(|row| !row.enabled())
            .map(|row| format!("{}: {}", row.button, row.reason()))
            .collect::<Vec<_>>()
            .join("; "),
    );
}

/// The windows the page's buttons open, and the rules the Warning Manager edits.
pub struct Windows<'a> {
    /// FFT's.
    pub fft: &'a mut Fft,
    /// The MAVLink Inspector's.
    pub inspector: &'a mut InspectorWindow,
    /// The Warning Manager's.
    pub warnings: &'a mut ManagerWindow,
    /// The warning engine's rules, which the Warning Manager is built over.
    pub rules: &'a mut Vec<CustomWarning>,
}

/// A button clicked, as the page's buttons are: `but_warningmanager_Click`, `new
/// WarningsManager().Show()` over the engine's rules, and the two [`click`] opens. True when it
/// did something. Proximity's and Mavlink Signing's are [`click_keys_or_proximity`].
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:22-30, 114-117`
pub fn open(button: &str, windows: Windows<'_>, now: std::time::Instant) -> bool {
    if button == "but_warningmanager" {
        windows.warnings.show(windows.rules);
        return true;
    }
    click(button, windows.fft, windows.inspector, now)
}

/// A button clicked, for the two windows that need nothing but themselves -
/// `but_mavinspector_Click`, `new MAVLinkInspector(MainV2.comPort).Show()`, and `but_fft_Click`,
/// `new fftui().Show()` - neither asking anything of the vehicle first. True when it did
/// something. The Warning Manager needs the engine's rules: [`open`]; Proximity's and Mavlink
/// Signing's are [`click_keys_or_proximity`]; the Spectrogram's and Support Proxy's
/// [`click_window`].
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:27-30, 114-117`
pub fn click(
    button: &str,
    fft: &mut Fft,
    inspector: &mut InspectorWindow,
    now: std::time::Instant,
) -> bool {
    match button {
        "but_mavinspector" => {
            inspector.show(now);
            true
        }
        "but_fft" => {
            fft.show_window();
            true
        }
        _ => false,
    }
}

/// A button clicked, for two more whose windows are here - `but_signkey_Click`, `new AuthKeys()
/// .Show()`, and `but_proximity_Click`, `new ProximityControl(MainV2.comPort.MAV).Show()` -
/// neither asking anything of the vehicle first. True when it did something.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:37-45`
pub fn click_keys_or_proximity(
    button: &str,
    proximity: &mut ProximityWindow,
    auth_keys: &mut AuthKeysWindow,
    now: std::time::Instant,
) -> bool {
    match button {
        "but_signkey" => {
            auth_keys.show(now);
            true
        }
        "but_proximity" => {
            proximity.show();
            true
        }
        _ => false,
    }
}

/// Param gen: `BUT_paramgen_Click`, the generation started behind its dialogue, writing into
/// the user data directory. True when it did something.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:57-78`
pub fn click_paramgen(button: &str, run: &mut ParamGen, data_dir: &std::path::Path) -> bool {
    if button == "BUT_paramgen" {
        run.start(data_dir);
        return true;
    }
    false
}

/// The Mavlink Mirror's and NMEA's buttons: `BUT_outputMavlink_Click`, `new SerialOutputPass()
/// .Show()`, and `BUT_outputnmea_Click`, `new SerialOutputNMEA().Show()` - the first reading
/// its rows from the settings. True when it did something.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:32-35, 47-50`
pub fn click_outputs(
    button: &str,
    mirror: &mut MavlinkMirror,
    nmea: &mut NmeaOutput,
    persisted: &Persisted,
) -> bool {
    match button {
        "BUT_outputMavlink" => {
            mirror.show(persisted);
            true
        }
        "BUT_outputnmea" => {
            nmea.show();
            true
        }
        _ => false,
    }
}

/// The Spectrogram's and Support Proxy's buttons: `BUT_spect_Click`, `new SpectrogramUI()
/// .Show()`, and `BUT_supportproxy_Click`, `new SerialSupportProxy().Show()`, whose constructor
/// reads the settings - neither asking anything of the vehicle first. True when it did
/// something.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:119-127`
pub fn click_window(
    button: &str,
    spectrogram: &mut SpectrogramWindow,
    proxy: &mut SupportProxy,
    persisted: &Persisted,
) -> bool {
    match button {
        "BUT_spect" => {
            spectrogram.show();
            true
        }
        "BUT_supportproxy" => {
            proxy.show(persisted);
            true
        }
        _ => false,
    }
}

/// An absolutely placed box.
fn at((x, y, width, height): (f32, f32, f32, f32)) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// A button in its cell: Warning Manager's, MAVLink Inspector's, Proximity's, Mavlink Signing's,
/// FFT's, Spectrogram's and Support Proxy's opening their windows, the others dimmed - the window
/// each opens is not in this application (see `opens`).
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:22-127`
fn button(row: Row, y: f32, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (bx, by, bw, bh) = BUTTON_IN_CELL;
    let id = format!("advanced-{}", row.button);
    let base = crate::probe::measured(id.clone(), at((bx, y + by, bw, bh)))
        .flex()
        .items_center()
        .justify_center()
        .rounded_sm()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .child(row.text);
    if row.enabled() {
        base.id(SharedString::from(id))
            .bg(rgb(theme::ACTION))
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .on_click(cx.listener(move |this, _event, window, cx| {
                window.blur(cx);
                let pages = &mut this.extra;
                let now = std::time::Instant::now();
                let windows = Windows {
                    fft: &mut pages.fft,
                    inspector: &mut pages.inspector,
                    warnings: &mut pages.warnings_manager,
                    rules: &mut this.warnings.warnings,
                };
                if open(row.button, windows, now)
                    || click_keys_or_proximity(
                        row.button,
                        &mut pages.proximity,
                        &mut pages.auth_keys,
                        now,
                    )
                    || click_window(
                        row.button,
                        &mut pages.spectrogram,
                        &mut pages.support_proxy,
                        &this.persisted,
                    )
                    || click_outputs(
                        row.button,
                        &mut pages.mavlink_mirror,
                        &mut pages.nmea_output,
                        &this.persisted,
                    )
                    || click_paramgen(
                        row.button,
                        &mut pages.param_gen,
                        // `Settings.GetUserDataDirectory()`.
                        &mp_settings::user_data_directory().unwrap_or_else(std::env::temp_dir),
                    )
                {
                    // A new form is activated: the proximity window takes its keys at once.
                    if row.button == "but_proximity" {
                        this.extra_focus.proximity.focus(window, cx);
                    }
                    // `NUM_port.Select()`: the Support Proxy's number has the keyboard.
                    if row.button == "BUT_supportproxy" {
                        this.extra_focus.support_proxy.number.focus(window, cx);
                    }
                    cx.notify();
                }
            }))
            .into_any_element()
    } else {
        base.bg(rgb(theme::PANEL))
            .text_color(rgb(theme::DIM))
            .into_any_element()
    }
}

/// The page, as the `.resx` lays it out: Warning Manager, MAVLink Inspector, Proximity, Mavlink
/// Signing, FFT, Spectrogram and Support Proxy opening their windows, every other button dimmed.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.Designer.cs:29-266; ConfigAdvanced.resx`
pub fn page(cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut table = at(TABLE);
    let mut y = 0.0;
    for row in ROWS {
        table = table.child(button(row, y, cx)).child(
            // `Padding` 5, 5, 0, 0.
            crate::probe::measured(format!("advanced-{}", row.label), div())
                .absolute()
                .left(px(LABEL_X))
                .top(px(y))
                .w(px(row.label_size.0))
                .pl(px(5.0))
                .pt(px(5.0))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(row.label_text),
        );
        y += ROW_HEIGHT;
    }
    let body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(
            div()
                .absolute()
                .left(px(TEXT_AT.0))
                .top(px(TEXT_AT.1))
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(TEXT),
        )
        .child(table);
    panel(TITLE, body).into_any_element()
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;
    use crate::config_coverage::source::{csharp, resx};

    /// `x, y` as the `.resx` writes a point or a size.
    fn pair((x, y): (f32, f32)) -> String {
        format!("{x}, {y}")
    }

    /// Every control the Designer makes is drawn: `label1`, the table, and the thirteen buttons
    /// and thirteen labels in it.
    #[test]
    fn every_designer_control_is_drawn() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigAdvanced.Designer.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let made: BTreeSet<&str> = designer
            .lines()
            .filter_map(|line| line.trim().strip_prefix("this."))
            .filter_map(|line| line.split_once(" = new "))
            .map(|(name, _)| name)
            .collect();
        let ours: BTreeSet<&str> = ["label1", "tableLayoutPanel1"]
            .into_iter()
            .chain(ROWS.iter().map(|row| row.button))
            .chain(ROWS.iter().map(|row| row.label))
            .collect();
        assert_eq!(made, ours);
    }

    /// The thirteen wirings are the buttons' `Click`s, each named here with its handler.
    #[test]
    fn every_wiring_is_a_button_here() {
        let Some(designer) = csharp("GCSViews/ConfigurationView/ConfigAdvanced.Designer.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for row in ROWS {
            let line = format!(
                "this.{}.Click += new System.EventHandler(this.{});",
                row.button, row.handler
            );
            assert!(designer.contains(&line), "{line}");
        }
        assert_eq!(designer.matches(" += new ").count(), ROWS.len());
    }

    /// Every text, size and cell is the `.resx`'s: the rows in the table's order, each button in
    /// column 0 and its label in column 1 of the same row, at the row's height.
    #[test]
    fn every_place_and_text_is_the_resx() {
        let Some(text) = csharp("GCSViews/ConfigurationView/ConfigAdvanced.resx") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = resx(&text);
        let get = |key: String| values.get(&key).cloned().unwrap_or_default();
        assert_eq!(get("$this.Size".to_owned()), pair(PAGE_SIZE));
        assert_eq!(get("label1.Text".to_owned()), TEXT);
        assert_eq!(get("label1.Location".to_owned()), pair(TEXT_AT));
        let (x, y, w, h) = TABLE;
        assert_eq!(get("tableLayoutPanel1.Location".to_owned()), pair((x, y)));
        assert_eq!(get("tableLayoutPanel1.Size".to_owned()), pair((w, h)));
        let layout = get("tableLayoutPanel1.LayoutSettings".to_owned());
        assert!(layout.contains("<Columns Styles=\"Percent,23.03922,Percent,76.96078\" />"));
        assert_eq!(layout.matches("Absolute,40").count(), ROWS.len());
        let (bx, by, bw, bh) = BUTTON_IN_CELL;
        let mut row_y = 0.0_f32;
        for (index, row) in ROWS.iter().enumerate() {
            let cell = |name: &str, column: usize| {
                format!(
                    "<Control Name=\"{name}\" Row=\"{index}\" RowSpan=\"1\" Column=\"{column}\" \
                     ColumnSpan=\"1\" />"
                )
            };
            assert!(layout.contains(&cell(row.button, 0)), "{}", row.button);
            assert!(layout.contains(&cell(row.label, 1)), "{}", row.label);
            assert_eq!(get(format!("{}.Text", row.button)), row.text);
            assert_eq!(get(format!("{}.Size", row.button)), pair((bw, bh)));
            assert_eq!(
                get(format!("{}.Location", row.button)),
                pair((bx, row_y + by))
            );
            assert_eq!(get(format!("{}.Text", row.label)), row.label_text);
            assert_eq!(get(format!("{}.Size", row.label)), pair(row.label_size));
            assert_eq!(
                get(format!("{}.Location", row.label)),
                pair((LABEL_X, row_y))
            );
            row_y += ROW_HEIGHT;
        }
        assert_eq!(ROW_HEIGHT * 13.0, h);
    }

    /// Each handler opens the window, or calls the code, its row names as the reason it is
    /// dimmed; and `Activate` does nothing.
    #[test]
    fn every_handler_opens_what_its_row_says() {
        let Some(source) = csharp("GCSViews/ConfigurationView/ConfigAdvanced.cs") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for row in ROWS {
            let start = source
                .find(&format!("void {}(", row.handler))
                .unwrap_or_else(|| panic!("{}", row.handler));
            let body = &source[start..];
            let body = &body[..body.find("\n        }").expect("its end")];
            let file = row
                .opens
                .split_once('(')
                .and_then(|(_, rest)| rest.split_once(')'))
                .map(|(file, _)| file)
                .expect("a file");
            let stem = file
                .rsplit('/')
                .next()
                .and_then(|name| name.strip_suffix(".cs"))
                .expect("a .cs");
            assert!(body.contains(stem), "{} does not use {stem}", row.handler);
                        assert!(
                crate::config_coverage::source::csharp_root()
                    .is_some_and(|root| root.join(file).exists()),
                "{file}"
            );
        }
        let activate = source
            .split("public void Activate()")
            .nth(1)
            .and_then(|rest| rest.split('}').next())
            .expect("Activate");
        assert_eq!(activate.trim(), "{");
    }

    /// Every fact the GUI script asserts on is recorded here, and the only button of this page it
    /// clicks is one that does something - FFT - and it does click it.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-advanced.gui");
        let source = include_str!("advanced.rs");
        let mut facts = 0;
        let mut clicked = Vec::new();
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.advanced.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key}");
                    facts += 1;
                }
                (Some("click"), Some(id)) => {
                    if let Some(button) = id.strip_prefix("advanced-") {
                        let row = ROWS.iter().find(|row| row.button == button);
                        assert!(row.is_some_and(Row::enabled), "{id} is dimmed");
                        clicked.push(button);
                    }
                }
                _ => {}
            }
        }
        assert!(facts >= 5, "{facts} facts");
        assert_eq!(clicked, ["but_fft"]);
    }

    /// The page's windows, held as the page's button closure holds them.
    struct Pages {
        fft: Fft,
        inspector: InspectorWindow,
        warnings: ManagerWindow,
        rules: Vec<CustomWarning>,
        proximity: ProximityWindow,
        auth_keys: AuthKeysWindow,
        spectrogram: SpectrogramWindow,
        proxy: SupportProxy,
        mirror: MavlinkMirror,
        nmea: NmeaOutput,
        paramgen: ParamGen,
    }

    impl Pages {
        /// Every window closed; a key store of its own, as the application's is read from the
        /// user data directory; one rule for the Warning Manager to show.
        fn new() -> Self {
            Self {
                fft: Fft::default(),
                inspector: InspectorWindow::default(),
                warnings: ManagerWindow::default(),
                rules: vec![CustomWarning::on("alt")],
                proximity: ProximityWindow::default(),
                auth_keys: AuthKeysWindow {
                    store: Some(super::super::auth_keys::KeyStore::default()),
                    ..AuthKeysWindow::default()
                },
                spectrogram: SpectrogramWindow::default(),
                proxy: SupportProxy::default(),
                mirror: MavlinkMirror::default(),
                nmea: NmeaOutput::default(),
                paramgen: ParamGen::default(),
            }
        }

        /// A button clicked, as the page clicks it.
        fn click(&mut self, button: &str, persisted: &Persisted) -> bool {
            let now = std::time::Instant::now();
            let windows = Windows {
                fft: &mut self.fft,
                inspector: &mut self.inspector,
                warnings: &mut self.warnings,
                rules: &mut self.rules,
            };
            open(button, windows, now)
                || click_keys_or_proximity(button, &mut self.proximity, &mut self.auth_keys, now)
                || click_window(button, &mut self.spectrogram, &mut self.proxy, persisted)
                || click_outputs(button, &mut self.mirror, &mut self.nmea, persisted)
                || click_paramgen(button, &mut self.paramgen, &std::env::temp_dir())
        }
    }

    /// Warning Manager is `new WarningsManager().Show()`, which opens the manager over the
    /// engine's rules; FFT is `new fftui().Show()` - the FFT Setup page's handler word for word -
    /// so it opens the window that page opens; MAVLink Inspector is `new MAVLinkInspector(
    /// MainV2.comPort).Show()`, which opens the inspector; Proximity, Mavlink Signing,
    /// Spectrogram (`new SpectrogramUI().Show()`) and Support Proxy (`new SerialSupportProxy()
    /// .Show()`) open theirs; none needs a vehicle or a page shown first, and every other button
    /// does nothing.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:22-30, 37-45, 114-127; ConfigFFT.cs:162-165`
    #[test]
    fn the_ported_windows_open_and_the_others_nothing() {
        let now = std::time::Instant::now();
        let persisted = Persisted::at(None);
        for row in ROWS {
            let mut windows = Pages::new();
            assert_eq!(
                windows.click(row.button, &persisted),
                row.enabled(),
                "{}",
                row.button
            );
            let fft_row = row.button == "but_fft";
            let inspector_row = row.button == "but_mavinspector";
            let warnings_row = row.button == "but_warningmanager";
            let proximity_row = row.button == "but_proximity";
            let keys_row = row.button == "but_signkey";
            let spectrogram_row = row.button == "BUT_spect";
            let proxy_row = row.button == "BUT_supportproxy";
            let mirror_row = row.button == "BUT_outputMavlink";
            let nmea_row = row.button == "BUT_outputnmea";
            let paramgen_row = row.button == "BUT_paramgen";
            assert_eq!(windows.fft.window.is_some(), fft_row, "{}", row.button);
            assert_eq!(windows.fft.opened, usize::from(fft_row));
            assert!(
                !windows.fft.is_active(),
                "the FFT Setup page is not shown by it"
            );
            assert_eq!(
                windows.inspector.window.is_some(),
                inspector_row,
                "{}",
                row.button
            );
            assert_eq!(windows.inspector.opened, usize::from(inspector_row));
            assert_eq!(
                windows.warnings.window.is_some(),
                warnings_row,
                "{}",
                row.button
            );
            assert_eq!(windows.warnings.opened, usize::from(warnings_row));
            if warnings_row {
                assert_eq!(
                    windows.warnings.window.as_ref().map(|w| w.controls.len()),
                    Some(1)
                );
            }
            assert_eq!(
                windows.proximity.window.is_some(),
                proximity_row,
                "{}",
                row.button
            );
            assert_eq!(windows.proximity.opened, usize::from(proximity_row));
            assert_eq!(
                windows.auth_keys.window.is_some(),
                keys_row,
                "{}",
                row.button
            );
            assert_eq!(windows.auth_keys.opened, usize::from(keys_row));
            assert_eq!(windows.spectrogram.window.is_some(), spectrogram_row);
            assert_eq!(windows.spectrogram.opened, usize::from(spectrogram_row));
            assert_eq!(windows.proxy.window.is_some(), proxy_row, "{}", row.button);
            assert_eq!(windows.proxy.opened, usize::from(proxy_row));
            assert_eq!(
                windows.mirror.window.is_some(),
                mirror_row,
                "{}",
                row.button
            );
            assert_eq!(windows.mirror.opened, usize::from(mirror_row));
            assert_eq!(windows.nmea.window.is_some(), nmea_row, "{}", row.button);
            assert_eq!(windows.nmea.opened, usize::from(nmea_row));
            assert_eq!(windows.paramgen.started, usize::from(paramgen_row));
            // The run started reads GitHub: cancelled here, unseen.
            windows.paramgen.cancel();
            // `click` is the two that need nothing but themselves.
            let mut fft = Fft::default();
            let mut inspector = InspectorWindow::default();
            assert_eq!(
                click(row.button, &mut fft, &mut inspector, now),
                row.enabled()
                    && !warnings_row
                    && !proximity_row
                    && !keys_row
                    && !spectrogram_row
                    && !proxy_row
                    && !mirror_row
                    && !nmea_row
                    && !paramgen_row,
                "{}",
                row.button
            );
        }
        // A second click, a fresh window: the Designer's Magnitude unticked again.
        let mut windows = Pages::new();
        windows.click("but_fft", &persisted);
        if let Some(window) = windows.fft.window.as_mut() {
            window.toggle_magnitude();
        }
        windows.click("but_fft", &persisted);
        assert_eq!(windows.fft.opened, 2);
        assert!(
            windows
                .fft
                .window
                .as_ref()
                .is_some_and(|window| !window.magnitude)
        );
        // And the inspector's: "Show GCS Traffic" unticked and the history 50 again.
        windows.click("but_mavinspector", &persisted);
        if let Some(window) = windows.inspector.window.as_mut() {
            window.toggle_gcs_traffic();
            window.history = 7;
        }
        windows.click("but_mavinspector", &persisted);
        assert_eq!(windows.inspector.opened, 2);
        assert!(windows.inspector.window.as_ref().is_some_and(|window| {
            !window.gcs_traffic() && window.history == 50 && window.tree.is_empty()
        }));
        // And the proximity window's: the radius 5 m again.
        windows.click("but_proximity", &persisted);
        if let Some(window) = windows.proximity.window.as_mut() {
            window.key_press('+');
        }
        windows.click("but_proximity", &persisted);
        assert_eq!(windows.proximity.opened, 2);
        assert!(
            windows
                .proximity
                .window
                .as_ref()
                .is_some_and(|window| window.screenradius == 500.0)
        );
        // The spectrogram's: ACC1 and -80 to -20 again, nothing loaded.
        windows.click("BUT_spect", &persisted);
        if let Some(window) = windows.spectrogram.window.as_mut() {
            window.choose(5);
            window.step(super::super::spectrogram::Edit::Max, 1.0);
        }
        windows.click("BUT_spect", &persisted);
        assert_eq!(windows.spectrogram.opened, 2);
        assert!(windows.spectrogram.window.as_ref().is_some_and(|window| {
            window.sensor.value() == "ACC1"
                && window.max.field.value() == "-20"
                && window.log_name().is_none()
        }));
        // The proxy's: its constructor reads the settings - a TCP server kept.
        let mut kept = Persisted::at(None);
        kept.set("SerialSupportProxy_UDP", "False");
        kept.set("TCP_host_SerialSupportProxy", "support.example");
        windows.click("BUT_supportproxy", &kept);
        assert_eq!(windows.proxy.opened, 1);
        assert!(
            windows
                .proxy
                .window
                .as_ref()
                .is_some_and(|window| !window.udp && window.host.value() == "support.example")
        );
        let enabled: Vec<&str> = ROWS
            .iter()
            .filter(|row| row.enabled())
            .map(|row| row.button)
            .collect();
        assert_eq!(
            enabled,
            [
                "but_warningmanager",
                "but_mavinspector",
                "but_proximity",
                "but_signkey",
                "BUT_outputMavlink",
                "BUT_outputnmea",
                "BUT_paramgen",
                "but_fft",
                "BUT_spect",
                "BUT_supportproxy"
            ]
        );
    }

    /// Each enabled row's port is where it says, and its handler is the same `Show()` as the one
    /// the port was made for.
    #[test]
    fn every_port_is_the_window_its_handler_shows() {
        for row in ROWS {
            let Some(port) = row.port else {
                continue;
            };
            let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("src")
                .join(port);
            assert!(path.exists(), "{}", path.display());
        }
        let (Some(advanced), Some(setup)) = (
            csharp("GCSViews/ConfigurationView/ConfigAdvanced.cs"),
            csharp("GCSViews/ConfigurationView/ConfigFFT.cs"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let body = |source: &str| {
            let start = source.find("void but_fft_Click(").expect("but_fft_Click");
            let rest = &source[start..];
            rest[rest.find('{').expect("{")..rest.find('}').expect("}")]
                .trim_matches(|c: char| c == '{' || c.is_whitespace())
                .to_owned()
        };
        assert_eq!(body(&advanced), "new fftui().Show();");
        assert_eq!(body(&advanced), body(&setup));
    }
}
