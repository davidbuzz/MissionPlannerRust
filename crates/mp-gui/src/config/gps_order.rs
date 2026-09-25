//! CAN GPS Order: `GCSViews/ConfigurationView/ConfigGPSOrder.cs`, an Optional Hardware page of
//! Initial Setup (`GCSViews/InitialSetup.cs:265-266`), listed once every parameter is in.
//!
//! What it shows: the heading "UAVCAN GPS Order", a sentence, and a grid of the DroneCAN GPSs -
//! Order, NodeID and Name, and two button columns, GPS1 and GPS2. `Activate` disables the page
//! for good on a vehicle without `GPS1_CAN_OVRIDE`; otherwise it reads the detected node ids
//! (`GPS_CAN_NODEID1`, `GPS_CAN_NODEID2`) and the overrides (`GPS1_CAN_OVRIDE`,
//! `GPS2_CAN_OVRIDE`) and lists each override that is set (order 1 and 2) and each detected node
//! that is neither override (order 98 and 99) (`ConfigGPSOrder.cs:21-51`). A button sets that
//! override to the row's node id and runs `Activate` again; its `catch` shows the exception
//! (`:62-86`).
//!
//! `MAV.param[name]` is null for a name the vehicle does not list, and `Activate` reads `.Value`
//! of what it gets without looking: the first such read throws a `NullReferenceException`, which
//! `BackstageView.ActivatePage` logs (`ExtLibs/Controls/BackstageView/BackstageView.cs:495-505`),
//! leaving the grid as it was. Firmware from 4.5 names the detected ids `GPS1_CAN_NODEID` and
//! `GPS2_CAN_NODEID`, and lists `GPS2_CAN_OVRIDE` only with a second GPS, so on it the grid stays
//! empty - as it does in Mission Planner. Which read threw is kept, for the facts.
//!
//! The button's `catch` is `CustomMessageBox.Show(Strings.ERROR, "Failed to set param " + ex)`:
//! the arguments are text and caption, so the box says "Error" under a caption holding the
//! exception - drawn so here. The exception is written as its first line; the stack trace after
//! it is the .NET runtime's.
//!
//! The layout is the Designer's, in a 530 x 262 page: the grid at (3, 49), 524 x 209, with its
//! 20-pixel row headers and five 100-pixel columns.
//!
//! What is not ported, and why: the button cells' text. The Designer sets the columns' `Text`,
//! "Override 1" and "Override 2", but not `UseColumnTextForButtonValue`, so WinForms draws the
//! buttons blank; they are drawn blank here. The group box under the heading is 747 pixels wide in
//! a 530-pixel page, which clips it; it is drawn to the page's edge.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, SharedString, Window, div, prelude::*, px, rgb};

use super::optional::{Event, Job, Set, SetQueue, at, heading, label, message_box, rule};
use crate::MissionPlanner;
use crate::config::servo_output::{Message, value_of};
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list, the literal `InitialSetup` passes.
/// `// C#: GCSViews/InitialSetup.cs:266`
pub const TITLE: &str = "CAN GPS Order";

/// `label6.Text`, the heading.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.Designer.cs:55`
pub const HEADING: &str = "UAVCAN GPS Order";

/// `label1.Text`, less the line break it ends with.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.Designer.cs:74`
pub const NOTE: &str = "Set the GPS order if required";

/// The column headers, in the Designer's order.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.Designer.cs:112-150`
pub const COLUMNS: [&str; 5] = ["Order", "NodeID", "Name", "GPS1", "GPS2"];

/// The parameter whose absence disables the page.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:23`
pub const OVERRIDE_1: &str = "GPS1_CAN_OVRIDE";
/// The second override.
pub const OVERRIDE_2: &str = "GPS2_CAN_OVRIDE";
/// The detected ids, as firmware before 4.5 names them.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:30-31`
pub const DETECTED_1: &str = "GPS_CAN_NODEID1";
/// The second detected id.
pub const DETECTED_2: &str = "GPS_CAN_NODEID2";

/// `Strings.ERROR`: the box's text, the handler's arguments being the wrong way round.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:84; ExtLibs/Strings/Strings.resx`
pub const ERROR_TEXT: &str = "Error";

/// The caption for GPS1's set timing out: `"Failed to set param " + ex`.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:84; ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1765`
pub const FAILED_1: &str =
    "Failed to set param System.TimeoutException: Timeout on read - setParam GPS1_CAN_OVRIDE";
/// The caption for GPS2's.
pub const FAILED_2: &str =
    "Failed to set param System.TimeoutException: Timeout on read - setParam GPS2_CAN_OVRIDE";
/// The caption when the `Activate` the handler runs after its set throws.
pub const FAILED_ACTIVATE: &str = "Failed to set param System.NullReferenceException: Object \
                                   reference not set to an instance of an object.";

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.Designer.cs:158`
const PAGE_SIZE: (f32, f32) = (530.0, 262.0);
/// `myDataGridView1`'s `Location` and `Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.Designer.cs:98-104`
const GRID: (f32, f32, f32, f32) = (3.0, 49.0, 524.0, 209.0);
/// `RowHeadersWidth`.
const ROW_HEADER: f32 = 20.0;
/// A `DataGridViewColumn`'s default width.
const COLUMN: f32 = 100.0;
/// The header row's height, `AutoSize` for one line.
const HEADER_HEIGHT: f32 = 23.0;
/// A row's, `DataGridView`'s default template.
const ROW_HEIGHT: f32 = 22.0;

/// One row: `ConfigGPSOrder.GPSCAN`.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:53-60`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpsCan {
    /// `Order`.
    pub order: i32,
    /// `Name`.
    pub name: &'static str,
    /// `NodeID`.
    pub node_id: i32,
}

/// Which button column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Column {
    /// `GPS1`: sets `GPS1_CAN_OVRIDE`.
    Gps1,
    /// `GPS2`: sets `GPS2_CAN_OVRIDE`.
    Gps2,
}

impl Column {
    /// The override it sets.
    #[must_use]
    pub const fn param(self) -> &'static str {
        match self {
            Self::Gps1 => OVERRIDE_1,
            Self::Gps2 => OVERRIDE_2,
        }
    }

    /// The `catch`'s caption for its set timing out.
    const fn failed(self) -> &'static str {
        match self {
            Self::Gps1 => FAILED_1,
            Self::Gps2 => FAILED_2,
        }
    }

    /// Its control id's part.
    const fn id(self) -> &'static str {
        match self {
            Self::Gps1 => "gps1",
            Self::Gps2 => "gps2",
        }
    }
}

/// `Activate`'s list, or the name whose `.Value` was read of null - the
/// `NullReferenceException` - in the order the C# reads them.
/// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:29-46`
pub fn rows(parameters: &[(String, f64)]) -> Result<Vec<GpsCan>, &'static str> {
    let read = |name: &'static str| value_of(parameters, name).ok_or(name);
    let id1 = value_of(parameters, DETECTED_1);
    let id2 = value_of(parameters, DETECTED_2);
    let mut list = Vec::new();
    // `(int)` of the double: towards zero.
    #[allow(clippy::cast_possible_truncation)]
    let node = |value: f64| value as i32;
    let id1ovr = read(OVERRIDE_1)?;
    if id1ovr != 0.0 {
        list.push(GpsCan {
            order: 1,
            node_id: node(id1ovr),
            name: "GPS Override 1",
        });
    }
    let id2ovr = read(OVERRIDE_2)?;
    if id2ovr != 0.0 {
        list.push(GpsCan {
            order: 2,
            node_id: node(id2ovr),
            name: "GPS Override 2",
        });
    }
    let id1 = id1.ok_or(DETECTED_1)?;
    #[allow(clippy::float_cmp)] // the C#'s exact comparisons
    if id1 != 0.0 && id1 != id1ovr && id1 != id2ovr {
        list.push(GpsCan {
            order: 98,
            node_id: node(id1),
            name: "GPS Detect 1",
        });
    }
    let id2 = id2.ok_or(DETECTED_2)?;
    #[allow(clippy::float_cmp)]
    if id2 != 0.0 && id2 != id1ovr && id2 != id2ovr {
        list.push(GpsCan {
            order: 99,
            node_id: node(id2),
            name: "GPS Detect 2",
        });
    }
    Ok(list)
}

/// The page object. `H` is the link's request handle; the tests use a bench's.
#[derive(Debug)]
pub struct GpsOrder<H = mp_link::RequestId> {
    made_for: Option<Key>,
    active: bool,
    /// The page's `Enabled`, false for good once `Activate` finds no `GPS1_CAN_OVRIDE`.
    disabled: bool,
    /// The grid's rows: the Designer's empty binding until an `Activate` completes.
    rows: Vec<GpsCan>,
    /// The name whose null the last `Activate` read, if it threw.
    threw_at: Option<&'static str>,
    messages: VecDeque<Message>,
    queue: SetQueue<H>,
}

impl<H> Default for GpsOrder<H> {
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            disabled: false,
            rows: Vec::new(),
            threw_at: None,
            messages: VecDeque::new(),
            queue: SetQueue::default(),
        }
    }
}

impl GpsOrder {
    /// Once a frame: a page object whose screen has gone is let go, and the writes move on - a
    /// set that did not throw followed by the handler's `Activate`, from the table as the set
    /// left it, whose own throw is the handler's `catch`.
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, on_setup: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            self.made_for = None;
        }
        let events = self.queue.advance(telemetry, &mut self.messages);
        self.after(&events, || telemetry.view().parameters.to_vec());
    }
}

impl<H: Copy> GpsOrder<H> {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The page's `Enabled`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        !self.disabled
    }

    /// The grid's rows.
    #[must_use]
    pub fn rows(&self) -> &[GpsCan] {
        &self.rows
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// Shows the page: a new page object for a new screen, then `Activate`.
    /// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:21-51`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let queue = std::mem::take(&mut self.queue);
            *self = Self {
                made_for: Some(key),
                messages,
                queue,
                ..Self::default()
            };
        }
        self.active = true;
        // The exception `ActivatePage` logs.
        let _ = self.run_activate(parameters);
    }

    /// `Activate` itself: `Err` with the name read of null when it throws.
    fn run_activate(&mut self, parameters: &[(String, f64)]) -> Result<(), &'static str> {
        if value_of(parameters, OVERRIDE_1).is_none() {
            self.disabled = true;
            return Ok(());
        }
        match rows(parameters) {
            Ok(rows) => {
                self.rows = rows;
                self.threw_at = None;
                Ok(())
            }
            Err(name) => {
                self.threw_at = Some(name);
                Err(name)
            }
        }
    }

    /// The page hidden. `ConfigGPSOrder` is `IActivate` only.
    pub fn hide(&mut self) {
        self.active = false;
    }

    /// A button cell clicked: the override set to the row's node id, in the handler's `try`.
    /// `// C#: GCSViews/ConfigurationView/ConfigGPSOrder.cs:62-86`
    pub fn click(&mut self, row: usize, column: Column) -> Vec<Job> {
        if !self.active || self.disabled {
            return Vec::new();
        }
        let Some(gps) = self.rows.get(row) else {
            return Vec::new();
        };
        let set = Set {
            on_throw: Some(Message {
                title: column.failed(),
                text: ERROR_TEXT.to_owned(),
            }),
            ..Set::plain(column.param(), f64::from(gps.node_id))
        };
        vec![Job::new(column.id(), [set])]
    }

    /// Queues handlers' jobs.
    pub fn push(&mut self, jobs: Vec<Job>) {
        self.queue.push(jobs);
    }

    /// The handler's `Activate()` after each set that returned.
    fn after(&mut self, events: &[Event], parameters: impl Fn() -> Vec<(String, f64)>) {
        for event in events {
            if let Event::Done { tag, threw: false } = event
                && (*tag == Column::Gps1.id() || *tag == Column::Gps2.id())
                && self.run_activate(&parameters()).is_err()
            {
                self.messages.push_back(Message {
                    title: FAILED_ACTIVATE,
                    text: ERROR_TEXT.to_owned(),
                });
            }
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &GpsOrder, view: &TelemetryView) {
    use crate::facts::record;
    record("config.gpsorder.active", page.is_active());
    record("config.gpsorder.enabled", page.enabled());
    record("config.gpsorder.heading", HEADING);
    record("config.gpsorder.note", NOTE);
    record("config.gpsorder.columns", COLUMNS.join(","));
    record("config.gpsorder.rows", page.rows().len());
    for (index, row) in page.rows().iter().enumerate() {
        record(
            format!("config.gpsorder.row.{index}"),
            format!("{},{},{}", row.order, row.node_id, row.name),
        );
    }
    record(
        "config.gpsorder.threw",
        page.threw_at.map_or_else(
            || "none".to_owned(),
            |name| format!("NullReferenceException at {name}"),
        ),
    );
    record("config.gpsorder.write", page.queue.last().unwrap_or("none"));
    record("config.gpsorder.writes.pending", page.queue.pending());
    record(
        "config.gpsorder.message",
        page.message().map_or("none", |message| message.title),
    );
    for name in [OVERRIDE_1, OVERRIDE_2, DETECTED_1, DETECTED_2] {
        if let Some(value) = value_of(&view.parameters, name) {
            record(format!("params.value.{name}"), value);
        }
    }
}

/// One cell of the grid: its text, left-aligned and vertically centred, in a bordered box.
fn cell(x: f32, y: f32, width: f32, height: f32, text: String, header: bool) -> gpui::Div {
    at(x, y, width, height)
        .flex()
        .items_center()
        .px_1()
        .border_r_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(if header { theme::ACTION } else { theme::BG }))
        .text_xs()
        .overflow_hidden()
        .whitespace_nowrap()
        .text_color(rgb(theme::TEXT))
        .child(text)
}

/// The page, laid out as the Designer lays it out.
pub fn page(order: &GpsOrder, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !order.is_active() {
        return div().into_any_element();
    }
    let enabled = order.enabled();
    let (gx, gy, gw, gh) = GRID;
    let mut grid = crate::probe::measured("gpsorder-grid", at(gx, gy, gw, gh))
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .child(cell(
            0.0,
            0.0,
            ROW_HEADER,
            HEADER_HEIGHT,
            String::new(),
            true,
        ));
    for (index, name) in COLUMNS.iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let x = ROW_HEADER + COLUMN * index as f32;
        grid = grid.child(cell(
            x,
            0.0,
            COLUMN,
            HEADER_HEIGHT,
            (*name).to_owned(),
            true,
        ));
    }
    for (row, gps) in order.rows().iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let y = HEADER_HEIGHT + ROW_HEIGHT * row as f32;
        grid = grid
            .child(cell(0.0, y, ROW_HEADER, ROW_HEIGHT, String::new(), true))
            .child(cell(
                ROW_HEADER,
                y,
                COLUMN,
                ROW_HEIGHT,
                gps.order.to_string(),
                false,
            ))
            .child(cell(
                ROW_HEADER + COLUMN,
                y,
                COLUMN,
                ROW_HEIGHT,
                gps.node_id.to_string(),
                false,
            ))
            .child(cell(
                ROW_HEADER + 2.0 * COLUMN,
                y,
                COLUMN,
                ROW_HEIGHT,
                gps.name.to_owned(),
                false,
            ));
        for (slot, column) in [(3.0, Column::Gps1), (4.0, Column::Gps2)] {
            let id = format!("gpsorder-{}-{row}", column.id());
            let button = crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .size_full()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }));
            let button = if enabled {
                button
                    .cursor_pointer()
                    .hover(|style| style.border_color(rgb(theme::ACCENT)))
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        let jobs = this.extra.gps_order.click(row, column);
                        this.extra.gps_order.push(jobs);
                        cx.notify();
                    }))
            } else {
                button
            };
            grid = grid.child(
                at(ROW_HEADER + slot * COLUMN, y, COLUMN, ROW_HEIGHT)
                    .p(px(2.0))
                    .border_r_1()
                    .border_b_1()
                    .border_color(rgb(theme::BORDER))
                    .child(button),
            );
        }
    }
    let body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(heading(3.0, 0.0, HEADING, enabled))
        .child(rule(-1.0, 18.0, PAGE_SIZE.0 + 1.0))
        .child(label(3.0, 33.0, NOTE, enabled))
        .child(grid);
    panel(TITLE, body).into_any_element()
}

/// The message box showing, over the whole window.
pub fn overlay(
    order: &GpsOrder,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let message = order.message()?;
    Some(message_box(
        "gpsorder-message",
        "gpsorder-message-ok",
        message,
        window,
        |this| this.extra.gps_order.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::flight_modes::Progress;
    use crate::config::optional::tests::Answering;
    use mp_link::requests::RequestOutcome;

    fn table(entries: &[(&str, f64)]) -> Vec<(String, f64)> {
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), *value))
            .collect()
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// A 4.4 copter with two DroneCAN GPSs, node 125 overriding the first.
    fn two_gps() -> Vec<(String, f64)> {
        table(&[
            ("GPS_CAN_NODEID1", 124.0),
            ("GPS_CAN_NODEID2", 125.0),
            ("GPS1_CAN_OVRIDE", 125.0),
            ("GPS2_CAN_OVRIDE", 0.0),
        ])
    }

    /// Runs the queue to the end, the handler's `Activate` after each set reading `after`.
    fn drain(page: &mut GpsOrder<usize>, link: &Answering, after: &[(String, f64)]) {
        let mut messages = VecDeque::new();
        for _ in 0..100 {
            let events = page.queue.advance(link, &mut messages);
            page.after(&events, || after.to_vec());
            if page.queue.pending() == 0 {
                break;
            }
        }
        page.messages.extend(messages);
    }

    /// The Designer's words, read from the tree when it is here.
    #[test]
    fn the_text_is_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigGPSOrder.Designer.cs",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        assert!(designer.contains(&format!("this.label6.Text = \"{HEADING}\";")));
        assert!(designer.contains(&format!("this.label1.Text = \"{NOTE}\\r\\n\";")));
        for name in COLUMNS {
            assert!(
                designer.contains(&format!(".HeaderText = \"{name}\";")),
                "{name}"
            );
        }
        assert!(
            designer.contains("this.myDataGridView1.Location = new System.Drawing.Point(3, 49);")
        );
        assert!(designer.contains("this.myDataGridView1.RowHeadersWidth = 20;"));
        assert!(designer.contains("this.Size = new System.Drawing.Size(530, 262);"));
        // The buttons' text is set, and never shown: no UseColumnTextForButtonValue.
        assert!(designer.contains("this.GPS1.Text = \"Override 1\";"));
        assert!(!designer.contains("UseColumnTextForButtonValue"));
    }

    /// The override listed first, then the detected node the override is not.
    #[test]
    fn activate_lists_the_overrides_then_the_other_detected_nodes() {
        let mut page = GpsOrder::<usize>::default();
        page.activate(&two_gps(), key());
        assert!(page.enabled());
        assert_eq!(
            page.rows(),
            [
                GpsCan {
                    order: 1,
                    name: "GPS Override 1",
                    node_id: 125
                },
                GpsCan {
                    order: 98,
                    name: "GPS Detect 1",
                    node_id: 124
                },
            ]
        );
        assert_eq!(page.threw_at, None);
    }

    /// No `GPS1_CAN_OVRIDE`: the page disabled, for good.
    #[test]
    fn without_the_override_the_page_is_disabled_for_good() {
        let mut page = GpsOrder::<usize>::default();
        page.activate(&table(&[("GPS_TYPE", 1.0)]), key());
        assert!(!page.enabled());
        page.hide();
        page.activate(&two_gps(), key());
        assert!(!page.enabled(), "nothing enables it again");
        assert!(page.click(0, Column::Gps1).is_empty());
    }

    /// SITL's 4.5 copter: `GPS1_CAN_OVRIDE` 0 and no `GPS2_CAN_OVRIDE` - the read of its value
    /// throws, and the grid stays empty.
    #[test]
    fn on_4_5_the_second_override_is_null_and_the_grid_stays_empty() {
        let parameters = table(&[
            ("GPS1_CAN_NODEID", 0.0),
            ("GPS1_CAN_OVRIDE", 0.0),
            ("GPS1_TYPE", 1.0),
        ]);
        assert_eq!(rows(&parameters), Err(OVERRIDE_2));
        let mut page = GpsOrder::<usize>::default();
        page.activate(&parameters, key());
        assert!(page.enabled());
        assert!(page.rows().is_empty());
        assert_eq!(page.threw_at, Some(OVERRIDE_2));
    }

    /// With both overrides but no detected ids, the throw comes after the overrides are read -
    /// and the grid keeps what it had.
    #[test]
    fn a_throw_leaves_the_grid_as_it_was() {
        let mut page = GpsOrder::<usize>::default();
        page.activate(&two_gps(), key());
        page.hide();
        page.activate(
            &table(&[("GPS1_CAN_OVRIDE", 7.0), ("GPS2_CAN_OVRIDE", 0.0)]),
            key(),
        );
        assert_eq!(page.threw_at, Some(DETECTED_1));
        assert_eq!(page.rows().len(), 2, "the last list");
    }

    /// Detected nodes that are overrides are not listed twice; zeros are not listed.
    #[test]
    fn a_detected_node_that_is_an_override_is_listed_once() {
        let parameters = table(&[
            ("GPS_CAN_NODEID1", 124.0),
            ("GPS_CAN_NODEID2", 0.0),
            ("GPS1_CAN_OVRIDE", 0.0),
            ("GPS2_CAN_OVRIDE", 124.0),
        ]);
        assert_eq!(
            rows(&parameters),
            Ok(vec![GpsCan {
                order: 2,
                name: "GPS Override 2",
                node_id: 124
            }])
        );
    }

    /// GPS2 on the detected row: `GPS2_CAN_OVRIDE` set to its node, then `Activate` again from
    /// the table the set left.
    #[test]
    fn a_button_sets_its_override_and_activates_again() {
        let mut page = GpsOrder::<usize>::default();
        page.activate(&two_gps(), key());
        let jobs = page.click(1, Column::Gps2);
        page.push(jobs);
        let link = Answering::new(&[]);
        let mut after = two_gps();
        for (name, value) in &mut after {
            if name == "GPS2_CAN_OVRIDE" {
                *value = 124.0;
            }
        }
        drain(&mut page, &link, &after);
        assert_eq!(link.taken(), [("GPS2_CAN_OVRIDE".to_owned(), 124.0)]);
        let orders: Vec<i32> = page.rows().iter().map(|row| row.order).collect();
        assert_eq!(orders, [1, 2], "both overrides, no detected node left over");
        assert!(page.message().is_none());
    }

    /// A timeout: the `catch`'s box, "Error" under the exception, and no `Activate`.
    #[test]
    fn a_timeout_shows_the_catchs_box_the_wrong_way_round() {
        let mut page = GpsOrder::<usize>::default();
        page.activate(&two_gps(), key());
        let jobs = page.click(0, Column::Gps1);
        page.push(jobs);
        let link = Answering::new(&[(
            "GPS1_CAN_OVRIDE",
            Progress::Finished(RequestOutcome::TimedOut),
        )]);
        drain(&mut page, &link, &[]);
        let message = page.message().expect("the box");
        assert_eq!(message.title, FAILED_1);
        assert_eq!(message.text, ERROR_TEXT);
        assert_eq!(page.rows().len(), 2, "no Activate ran");
    }

    /// The `Activate` after the set throwing is the handler's `catch` too.
    #[test]
    fn an_activate_that_throws_after_the_set_is_caught() {
        let mut page = GpsOrder::<usize>::default();
        page.activate(&two_gps(), key());
        let jobs = page.click(0, Column::Gps1);
        page.push(jobs);
        let link = Answering::new(&[]);
        drain(&mut page, &link, &table(&[("GPS1_CAN_OVRIDE", 125.0)]));
        assert_eq!(
            page.message().map(|message| message.title),
            Some(FAILED_ACTIVATE)
        );
    }

    /// Every fact the GUI script asserts on is one this page records.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-gpsorder.gui");
        let source = include_str!("gps_order.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            if let (Some("expect"), Some(key)) = (words.next(), words.next())
                && key.starts_with("config.gpsorder.")
            {
                assert!(
                    source.contains(&format!("\"{key}\"")),
                    "{key} is not recorded"
                );
                facts += 1;
            }
        }
        assert!(facts > 6, "{facts} facts");
    }
}
