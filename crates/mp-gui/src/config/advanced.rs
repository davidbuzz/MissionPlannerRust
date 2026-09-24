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
//! log anonymised, the FFT and spectrogram plots, and the support proxy. None of those windows is
//! in this application - each is a form of its own in the C# (`Warnings/WarningsManager.cs`,
//! `Controls/MAVLinkInspector.cs` and the rest, named at [`ROWS`]), a port of its own - so every
//! button is drawn dimmed, with the window it would open as the reason.
//!
//! The colours are this application's.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Div, div, prelude::*, px, rgb};

use crate::ui::{panel, theme};

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
    /// What the handler opens, and where it is in the C# - the reason the button is dimmed.
    pub opens: &'static str,
}

/// A row, as the table below writes one.
const fn row(
    button: &'static str,
    text: &'static str,
    handler: &'static str,
    label: &'static str,
    label_text: &'static str,
    label_size: (f32, f32),
    opens: &'static str,
) -> Row {
    Row {
        button,
        text,
        handler,
        label,
        label_text,
        label_size,
        opens,
    }
}

/// The thirteen rows, top to bottom.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:22-127;
/// ConfigAdvanced.resx (tableLayoutPanel1.LayoutSettings, *.Text, label*.Size)`
#[rustfmt::skip]
pub const ROWS: [Row; 13] = [
    row("but_warningmanager", "Warning Manager", "but_warningmanager_Click", "label2",
        "Enable custom warnings based on a set of conditions", (263.0, 18.0),
        "the Warnings Manager window (Warnings/WarningsManager.cs) is not ported"),
    row("but_mavinspector", "MAVLink Inspector", "but_mavinspector_Click", "label3",
        "View decoded mavlink data being sent and received", (260.0, 18.0),
        "the MAVLink Inspector window (Controls/MAVLinkInspector.cs) is not ported"),
    row("but_proximity", "Proximity", "but_proximity_Click", "label4",
        "View the data from a 360 lidar", (152.0, 18.0),
        "the proximity window (Controls/ProximityControl.cs) is not ported"),
    row("but_signkey", "Mavlink Signing", "but_signkey_Click", "label5",
        "Enable mavlink signing to secure communication with the MAV", (307.0, 18.0),
        "the signing keys window (Controls/AuthKeys.cs) is not ported"),
    row("BUT_outputMavlink", "Mavlink Mirror", "BUT_outputMavlink_Click", "label6",
        "Mavlink mirror to an external location. For Monitoring or control", (304.0, 18.0),
        "the MAVLink mirror window (Controls/SerialOutputPass.cs) is not ported"),
    row("BUT_outputnmea", "NMEA", "BUT_outputnmea_Click", "label7",
        "Output the MAV location as a NMEA string", (213.0, 18.0),
        "the NMEA output window (Controls/SerialOutputNMEA.cs) is not ported"),
    row("BUT_follow_me", "Follow Me", "BUT_follow_me_Click", "label8",
        "Use an external NMEA gps and send guided mode waypoints to the MAV based on that location",
        (304.0, 31.0),
        "the Follow Me window (Controls/FollowMe.cs) is not ported"),
    row("BUT_paramgen", "Param gen", "BUT_paramgen_Click", "label9",
        "Regenerage the param info used inside mp", (214.0, 18.0),
        "regenerating the parameter documentation from ArduPilot's source \
         (ExtLibs/Utilities/ParameterMetaDataParser.cs) is not ported"),
    row("BUT_movingbase", "Moving Base", "BUT_movingbase_Click", "label10",
        "Show an extra icon on the map of your current location.", (273.0, 18.0),
        "the moving base window (Controls/MovingBase.cs) is not ported"),
    row("but_anonlog", "Anon Log", "but_anonlog_Click", "label11",
        "Scramble lat/lng in bin or tlog", (149.0, 18.0),
        "anonymising a log (ExtLibs/Utilities/Privacy.cs) is not ported"),
    row("but_fft", "FFT", "but_fft_Click", "label12",
        "Plot a FFT from a log", (110.0, 18.0),
        "the FFT window (Controls/fftui.cs) is not ported"),
    row("BUT_spect", "Spectrogram", "BUT_spect_Click", "label13",
        "Plot a FFT from a log", (110.0, 18.0),
        "the spectrogram window (Controls/SpectrogramUI.cs) is not ported"),
    row("BUT_supportproxy", "Support Proxy", "BUT_supportproxy_Click", "label14",
        "Share connection with support engineer", (200.0, 18.0),
        "the support proxy window (Controls/SerialSupportProxy.cs) is not ported"),
];

/// Facts a UI test asserts on: whether the page shows, its text, the buttons and their labels in
/// the table's order, and that every button is dimmed and why.
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
    record("config.advanced.buttons.enabled", false);
    record(
        "config.advanced.dimmed",
        ROWS.iter()
            .map(|row| format!("{}: {}", row.button, row.opens))
            .collect::<Vec<_>>()
            .join("; "),
    );
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

/// The page, as the `.resx` lays it out, every button dimmed.
/// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.Designer.cs:29-266; ConfigAdvanced.resx`
pub fn page() -> AnyElement {
    let mut table = at(TABLE);
    let mut y = 0.0;
    for row in ROWS {
        let (bx, by, bw, bh) = BUTTON_IN_CELL;
        // Dimmed: the window it opens is not in this application (see `opens`).
        // `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:22-127`
        table = table
            .child(
                crate::probe::measured(
                    format!("advanced-{}", row.button),
                    at((bx, y + by, bw, bh)),
                )
                .flex()
                .items_center()
                .justify_center()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(row.text),
            )
            .child(
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
            eprintln!("skipped: the C# tree is not checked out");
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
            eprintln!("skipped: the C# tree is not checked out");
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
            eprintln!("skipped: the C# tree is not checked out");
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
            eprintln!("skipped: the C# tree is not checked out");
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
                    .join(file)
                    .exists(),
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

    /// Every fact the GUI script asserts on is recorded here, and it clicks no button.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-advanced.gui");
        let source = include_str!("advanced.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.advanced.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key}");
                    facts += 1;
                }
                (Some("click"), Some(id)) => {
                    assert!(!id.starts_with("advanced-"), "the buttons are dimmed");
                }
                _ => {}
            }
        }
        assert!(facts >= 5, "{facts} facts");
    }
}
