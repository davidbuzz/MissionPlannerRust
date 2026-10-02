//! The Mavlink Mirror: `Controls/SerialOutputPass.cs`, the Advanced page's Mavlink Mirror
//! button (`new SerialOutputPass().Show()`, `ConfigAdvanced.cs:32-35`). A grid of outputs, each
//! a `MAVLinkInterface.Mirror` the link's packets are relayed to - and, with Write, whose bytes
//! are written to the link - started by its row's Go.
//!
//! What it does:
//!
//! * the form's top controls - the port and baud combos, Connect and "Write access" - are
//!   `Visible = False` in the `.resx`: only the grid shows (`SerialOutputPass.resx`), 452 by 228,
//!   the grid at (12, 69), 428 by 150, with the columns Type (Serial, TCP, UDP), Direction
//!   (Inbound, Outbound), Port, Host/Baud, Write (a check box) and Go (a button);
//! * `Load` (`:155-170`): the rows from the `serialpasslist` setting, each a JSON array of the
//!   cells' texts; a row whose index is in `Started` - the class's list of the rows started,
//!   kept across forms - shows "Started" in its Go cell;
//! * a cell edited ends in `Save` (`:140-153`): every row's cells as JSON arrays, into the
//!   setting;
//! * Go (`myDataGridView1_CellContentClick`, `:177-258`): a mirror made from the row - TCP
//!   Inbound listens on Port and takes the client; TCP Outbound connects to Host/Baud at Port,
//!   connecting again when the connection drops (`autoReconnect`); UDP Inbound binds Port; UDP
//!   Outbound sends to Host/Baud at Port; Serial opens Port at the baud in Host/Baud - with
//!   Write as its `MirrorStreamWrite`, added to the link's `Mirrors`; the Go cell reads
//!   "Started" and the row's index joins `Started`; what throws is "Error: " and its message;
//! * the hidden Connect and its stream (`BUT_connect_Click`, `:40-125`) cannot be reached: the
//!   constructor's `chk_write.Checked = MirrorStreamWrite` and its Stop text go to controls
//!   nobody sees.
//!
//! Where this is not the C# (each at its site):
//!
//! * the C#'s boxes - "Failed to load list", "Error: ..." - go on the status line (the owner's
//!   ruling of 2026-09-25);
//! * the grid's new row is a line of empty cells under the rows, a click on one adding the row;
//!   the Type and Direction cells open a list on a click, the text cells are typed into, the
//!   Write cell ticks on a click; `DataError` has nothing to do;
//! * a mirror's stream opens off the UI thread (a TCP connect can take seconds); a TCP Inbound
//!   takes one client; each mirror re-encodes the packets it relays (`mp_link::mirror`);
//! * the form is drawn over SETUP, modal; the mirrors outlive it, as the C#'s do.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::inspector::PacketSubscription;
use mp_link::mirror::{Mirror, Relayed, Reopen};
use mp_mission::dotnet::{bool_text, parse_bool};
use mp_transport::{TcpTransport, Transport};

use super::serial_output::{Kind, Opening, open, poll};
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::telemetry::Telemetry;
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::theme;

/// `this.Text`. `// C#: Controls/SerialOutputPass.resx`
pub const FORM_TEXT: &str = "SerialOutput - Mavlink";
/// `ClientSize`.
pub const CLIENT: (f32, f32) = (452.0, 228.0);
/// `myDataGridView1`.
pub const GRID_AT: (f32, f32, f32, f32) = (12.0, 69.0, 428.0, 150.0);
/// The columns: `HeaderText` and `Width`. `// C#: Controls/SerialOutputPass.resx`
pub const COLUMNS: [(&str, f32); 6] = [
    ("Type", 55.0),
    ("Direction", 80.0),
    ("Port", 60.0),
    ("Host/Baud", 80.0),
    ("Write", 40.0),
    ("Go", 40.0),
];
/// `Type.Items` and `Direction.Items`. `// C#: Controls/SerialOutputPass.Designer.cs:103-116`
pub const TYPES: [&str; 3] = ["Serial", "TCP", "UDP"];
pub const DIRECTIONS: [&str; 2] = ["Inbound", "Outbound"];
/// `configlist`. `// C#: Controls/SerialOutputPass.cs:172`
pub const SETTING: &str = "serialpasslist";
/// The Go cell's texts. `// C#: Controls/SerialOutputPass.cs:138, 165, 250`
pub const GO: &str = "Go";
pub const STARTED: &str = "Started";
/// `ConfigRef`s of the outbound streams. `// C#: Controls/SerialOutputPass.cs:209, 236`
pub const TCP_CONFIG_REF: &str = "SerialOutputPassTCP";
pub const UDP_CONFIG_REF: &str = "SerialOutputPassUDPCL";
/// The grid's row height.
const ROW_HEIGHT: f32 = 22.0;

/// A row of the grid.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Row {
    /// Type.
    pub kind: String,
    /// Direction.
    pub direction: String,
    /// Port.
    pub port: String,
    /// Host/Baud.
    pub extra: String,
    /// Write.
    pub write: bool,
}

impl Row {
    /// The cells' `FormattedValue`s, Go's included, as `Save` writes them: a JSON array.
    /// `// C#: Controls/SerialOutputPass.cs:140-153`
    #[must_use]
    pub fn json(&self, started: bool) -> String {
        let quote = |text: &str| format!("\"{}\"", text.replace('\\', "\\\\").replace('"', "\\\""));
        format!(
            "[{},{},{},{},{},{}]",
            quote(&self.kind),
            quote(&self.direction),
            quote(&self.port),
            quote(&self.extra),
            quote(bool_text(self.write)),
            quote(if started { STARTED } else { GO }),
        )
    }

    /// A row from `Load`'s JSON array: the first four cells' texts, Write parsed as the check
    /// box parses its value, anything beyond ignored; `None` for a line that is not an array.
    /// `// C#: Controls/SerialOutputPass.cs:155-170`
    #[must_use]
    pub fn parse(line: &str) -> Option<Self> {
        let inner = line.trim().strip_prefix('[')?.strip_suffix(']')?;
        let mut cells: Vec<String> = Vec::new();
        let mut chars = inner.chars().peekable();
        while let Some(c) = chars.next() {
            match c {
                '"' => {
                    let mut text = String::new();
                    while let Some(d) = chars.next() {
                        match d {
                            '\\' => {
                                if let Some(e) = chars.next() {
                                    text.push(e);
                                }
                            }
                            '"' => break,
                            other => text.push(other),
                        }
                    }
                    cells.push(text);
                }
                ',' | ' ' => {}
                other => {
                    let mut text = String::from(other);
                    while let Some(d) = chars.peek() {
                        if *d == ',' {
                            break;
                        }
                        text.push(*d);
                        chars.next();
                    }
                    cells.push(text.trim().to_owned());
                }
            }
        }
        if cells.len() < 5 {
            return None;
        }
        let cell = |index: usize| cells.get(index).cloned().unwrap_or_default();
        Some(Self {
            kind: cell(0),
            direction: cell(1),
            port: cell(2),
            extra: cell(3),
            write: parse_bool(&cell(4)).unwrap_or(false),
        })
    }
}

/// A mirror started from a row: its stream being opened, then the mirror on the link.
pub struct Started {
    /// The row's index.
    pub row: usize,
    /// Write.
    write: bool,
    /// TCP Outbound's address, connected to again when it drops.
    reopen: Option<(String, u16)>,
    opening: Option<Opening>,
    mirror: Option<Mirror>,
    subscription: Option<PacketSubscription>,
    attached: bool,
}

impl std::fmt::Debug for Started {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Started")
            .field("row", &self.row)
            .field("open", &self.mirror.as_ref().is_some_and(Mirror::is_open))
            .finish_non_exhaustive()
    }
}

/// The window, and the class's `Started` rows and their mirrors, kept across forms.
#[derive(Debug, Default)]
pub struct MavlinkMirror {
    /// The form, while it is open.
    pub window: Option<Form>,
    /// How many times it has been opened.
    pub opened: usize,
    /// `MainV2.comPort.Mirrors` as this window adds to it, and `Started`.
    pub mirrors: Vec<Started>,
    /// What the C# would have boxed, for the status line.
    status: Option<String>,
}

/// The form: the grid's rows and the cell being edited.
#[derive(Debug)]
pub struct Form {
    pub rows: Vec<Row>,
    /// The cell being edited: its row and column, and the text, for the text cells.
    pub editing: Option<(usize, usize)>,
    pub field: TextField,
    /// A Type or Direction cell whose list is down.
    pub list: Option<(usize, usize)>,
}

impl Default for Form {
    fn default() -> Self {
        Self {
            rows: Vec::new(),
            editing: None,
            field: TextField::new(""),
            list: None,
        }
    }
}

/// A text cell's column: Port and Host/Baud.
const TEXT_COLUMNS: [usize; 2] = [2, 3];

impl MavlinkMirror {
    /// `new SerialOutputPass().Show()`, with `Load`.
    /// `// C#: Controls/SerialOutputPass.cs:22-39, 155-170`
    pub fn show(&mut self, persisted: &Persisted) {
        self.opened += 1;
        let rows: Vec<Row> = crate::raw_params_grid::get_list(persisted.get(SETTING))
            .iter()
            .filter(|line| !line.is_empty())
            .filter_map(|line| Row::parse(line))
            .collect();
        self.window = Some(Form {
            rows,
            ..Form::default()
        });
    }

    /// The form's close box: the mirrors go on.
    pub fn close(&mut self) {
        self.window = None;
    }

    /// Whether row `index` is in `Started`.
    #[must_use]
    pub fn is_started(&self, index: usize) -> bool {
        self.mirrors.iter().any(|started| started.row == index)
    }

    /// `Save`: the rows as JSON arrays into the setting.
    /// `// C#: Controls/SerialOutputPass.cs:140-153`
    pub fn save(&self, persisted: &mut Persisted) {
        let Some(form) = self.window.as_ref() else {
            return;
        };
        let lines: Vec<String> = form
            .rows
            .iter()
            .enumerate()
            .map(|(index, row)| row.json(self.is_started(index)))
            .collect();
        match crate::raw_params_grid::set_list(&lines) {
            Some(value) => persisted.set(SETTING, value),
            None => persisted.remove(SETTING),
        }
    }

    /// A click on a cell: a text cell edited, a list cell's list opened, Write ticked, Go
    /// started; a click on the new row's cell adds a row first.
    pub fn click_cell(&mut self, row: usize, column: usize, persisted: &mut Persisted) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        if row == form.rows.len() {
            form.rows.push(Row {
                kind: String::new(),
                direction: String::new(),
                port: String::new(),
                extra: String::new(),
                write: false,
            });
        }
        if row >= form.rows.len() {
            return;
        }
        self.leave(persisted);
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let Some(cell) = form.rows.get_mut(row) else {
            return;
        };
        match column {
            0 | 1 => form.list = Some((row, column)),
            2 | 3 => {
                let text = if column == 2 {
                    cell.port.clone()
                } else {
                    cell.extra.clone()
                };
                form.field = TextField::new("");
                form.field.set(text);
                form.editing = Some((row, column));
            }
            4 => {
                cell.write = !cell.write;
                self.save(persisted);
            }
            5 => self.press_go(row, persisted),
            _ => {}
        }
    }

    /// A list's item chosen for the cell: `CellEndEdit`, which saves.
    pub fn choose(&mut self, index: usize, persisted: &mut Persisted) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let Some((row, column)) = form.list.take() else {
            return;
        };
        if let Some(cell) = form.rows.get_mut(row) {
            match column {
                0 => cell.kind = TYPES.get(index).map_or(String::new(), |t| (*t).to_owned()),
                1 => {
                    cell.direction = DIRECTIONS
                        .get(index)
                        .map_or(String::new(), |t| (*t).to_owned());
                }
                _ => {}
            }
        }
        self.save(persisted);
    }

    /// A key in the text cell being edited; Enter ends the edit.
    pub fn key(&mut self, event: &KeyDownEvent, persisted: &mut Persisted) -> bool {
        let Some(form) = self.window.as_mut() else {
            return false;
        };
        if form.editing.is_none() {
            return false;
        }
        match form.field.key(event) {
            KeyOutcome::Changed => true,
            KeyOutcome::Submitted | KeyOutcome::Cancelled => {
                self.leave(persisted);
                true
            }
            KeyOutcome::Ignored => false,
        }
    }

    /// The edit left: the text into its cell, `CellEndEdit`'s `Save`.
    /// `// C#: Controls/SerialOutputPass.cs:135-138`
    pub fn leave(&mut self, persisted: &mut Persisted) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        let Some((row, column)) = form.editing.take() else {
            return;
        };
        let text = form.field.value().to_owned();
        if let Some(cell) = form.rows.get_mut(row) {
            match column {
                2 => cell.port = text,
                3 => cell.extra = text,
                _ => {}
            }
        }
        self.save(persisted);
    }

    /// Go: the mirror the row describes started, the row in `Started`.
    /// `// C#: Controls/SerialOutputPass.cs:177-258`
    pub fn press_go(&mut self, index: usize, persisted: &mut Persisted) {
        let Some(row) = self
            .window
            .as_ref()
            .and_then(|form| form.rows.get(index))
            .cloned()
        else {
            return;
        };
        let port_number = || {
            row.port
                .trim()
                .parse::<u16>()
                .map_err(|_| "Input string was not in a correct format.".to_owned())
        };
        // The kind, its answers, a TCP host's port, a serial port's baud, and TCP Outbound's
        // address to connect to again.
        type Made = (Kind, Vec<String>, u16, u32, Option<(String, u16)>);
        let made: Result<Made, String> = match (row.kind.as_str(), row.direction.as_str()) {
            ("TCP", "Inbound") => {
                port_number().map(|port| (Kind::TcpHost, Vec::new(), port, 0, None))
            }
            ("TCP", "Outbound") => port_number().map(|port| {
                persisted.set(&format!("TCP_host_{TCP_CONFIG_REF}"), row.extra.clone());
                persisted.set(&format!("TCP_port_{TCP_CONFIG_REF}"), port.to_string());
                (
                    Kind::TcpClient,
                    vec![row.extra.clone(), port.to_string()],
                    0,
                    0,
                    Some((row.extra.clone(), port)),
                )
            }),
            ("UDP", "Inbound") => {
                port_number().map(|port| (Kind::UdpHost, vec![port.to_string()], 0, 0, None))
            }
            ("UDP", "Outbound") => port_number().map(|port| {
                persisted.set(&format!("UDP_host_{UDP_CONFIG_REF}"), row.extra.clone());
                persisted.set(&format!("UDP_port_{UDP_CONFIG_REF}"), port.to_string());
                (
                    Kind::UdpClient,
                    vec![row.extra.clone(), port.to_string()],
                    0,
                    0,
                    None,
                )
            }),
            ("Serial", _) => row
                .extra
                .trim()
                .parse::<u32>()
                .map_err(|_| "Input string was not in a correct format.".to_owned())
                .map(|baud| (Kind::Serial(row.port.clone()), Vec::new(), 0, baud, None)),
            // Neither TCP nor UDP nor Serial: the C# makes a mirror of nothing and adds it.
            _ => Ok((Kind::Serial(String::new()), Vec::new(), 0, 0, None)),
        };
        match made {
            Ok((kind, answers, host_port, baud, reopen)) => {
                let opening = match &kind {
                    Kind::Serial(name) if name.is_empty() => None,
                    _ => Some(open(kind, host_port, baud, answers)),
                };
                self.mirrors.push(Started {
                    row: index,
                    write: row.write,
                    reopen,
                    opening,
                    mirror: None,
                    subscription: None,
                    attached: false,
                });
                self.save(persisted);
            }
            Err(error) => self.status = Some(format!("Error: {error}")),
        }
    }

    /// Once a frame: streams opened made mirrors, each subscribed to the link and writing to it;
    /// what failed on the status line.
    pub fn tick(&mut self, telemetry: &Telemetry) -> Option<String> {
        for started in &mut self.mirrors {
            if let Some(opening) = started.opening.as_ref()
                && let Some(opened) = poll(opening)
            {
                started.opening = None;
                match opened {
                    Ok(stream) => {
                        let reopen: Option<Reopen> = started.reopen.clone().map(|(host, port)| {
                            Box::new(move || {
                                TcpTransport::connect(&host, port)
                                    .ok()
                                    .map(|s| Box::new(s) as Box<dyn Transport>)
                            }) as Reopen
                        });
                        match Mirror::start(stream, started.write, reopen) {
                            Ok(mirror) => started.mirror = Some(mirror),
                            Err(error) => self.status = Some(format!("Error: {error}")),
                        }
                    }
                    Err(error) => self.status = Some(format!("Error: {error}")),
                }
            }
            if let Some(mirror) = started.mirror.as_ref().filter(|m| m.is_open()) {
                if !started
                    .subscription
                    .as_ref()
                    .is_some_and(|subscription| telemetry.carries(subscription))
                {
                    started.subscription = telemetry.on_packet(mirror.handler());
                    started.attached = false;
                }
                if !started.attached
                    && started.subscription.is_some()
                    && let Some((sender, _)) = telemetry.send_handle()
                {
                    mirror.attach(Some(sender));
                    started.attached = true;
                }
            }
        }
        self.status.take()
    }

    /// What every mirror has relayed.
    #[must_use]
    pub fn relayed(&self) -> Relayed {
        let mut total = Relayed::default();
        for started in &self.mirrors {
            if let Some(mirror) = started.mirror.as_ref() {
                let r = mirror.relayed();
                total.up += r.up;
                total.down += r.down;
                total.dropped += r.dropped;
            }
        }
        total
    }

    /// How many mirrors are open.
    #[must_use]
    pub fn open_count(&self) -> usize {
        self.mirrors
            .iter()
            .filter(|started| started.mirror.as_ref().is_some_and(Mirror::is_open))
            .count()
    }

    /// Waits for every stream being opened, as a test does.
    #[cfg(test)]
    fn finish_opening(&mut self, telemetry: &Telemetry) -> Option<String> {
        let mut status = None;
        for _ in 0..2000 {
            if let Some(words) = self.tick(telemetry) {
                status = Some(words);
            }
            if self.mirrors.iter().all(|s| s.opening.is_none()) {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        status
    }
}

/// The facts, under `config.mirror.`.
pub fn record_facts(holder: &MavlinkMirror) {
    use crate::facts::record;
    record("config.mirror.window", holder.window.is_some());
    record("config.mirror.opened", holder.opened);
    record("config.mirror.mirrors", holder.mirrors.len());
    record("config.mirror.open", holder.open_count());
    let relayed = holder.relayed();
    record("config.mirror.relayed.up", relayed.up);
    record("config.mirror.relayed.down", relayed.down);
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record("config.mirror.rows", form.rows.len());
    for (index, row) in form.rows.iter().enumerate() {
        record(
            format!("config.mirror.row.{index}"),
            row.json(holder.is_started(index)),
        );
    }
    record(
        "config.mirror.editing",
        form.editing
            .map_or("none".to_owned(), |(r, c)| format!("{r}.{c}")),
    );
}

fn access(this: &mut MissionPlanner) -> &mut MavlinkMirror {
    &mut this.extra.mavlink_mirror
}

/// With the settings, for a handler: the window and the settings are two fields of the
/// application, borrowed apart.
fn with_settings(this: &mut MissionPlanner, act: impl FnOnce(&mut MavlinkMirror, &mut Persisted)) {
    act(&mut this.extra.mavlink_mirror, &mut this.persisted);
}

/// A cell of the grid.
#[allow(clippy::too_many_arguments)] // the cell's parts
fn cell(
    id: String,
    text: String,
    width: f32,
    editing: bool,
    field: &TextField,
    typing: &FocusHandle,
    row: usize,
    column: usize,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let base = crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .flex_shrink_0()
        .w(px(width))
        .h(px(ROW_HEIGHT))
        .flex()
        .items_center()
        .px_1()
        .border_r_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .whitespace_nowrap()
        .overflow_hidden()
        .text_color(rgb(theme::TEXT));
    if editing {
        let handle = typing.clone();
        base.bg(rgb(theme::ACTION))
            .track_focus(&handle)
            .key_context("TextField")
            .child(field.value().to_owned())
            .child(div().w(px(1.0)).h(px(12.0)).bg(rgb(theme::ACCENT)))
            .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                let mut handled = false;
                with_settings(this, |mirror, persisted| {
                    handled = mirror.key(event, persisted);
                });
                if handled {
                    cx.notify();
                }
            }))
            .into_any_element()
    } else {
        let handle = typing.clone();
        base.cursor_pointer()
            .child(text)
            .on_click(cx.listener(move |this, _event, window, cx| {
                with_settings(this, |mirror, persisted| {
                    mirror.click_cell(row, column, persisted);
                });
                if TEXT_COLUMNS.contains(&column) {
                    handle.focus(window, cx);
                }
                cx.notify();
            }))
            .into_any_element()
    }
}

/// The form over SETUP: the grid with its header, its rows and the new row.
/// `// C#: Controls/SerialOutputPass.resx`
pub fn overlay(
    holder: &MavlinkMirror,
    typing: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let form = holder.window.as_ref()?;
    let size = window.viewport_size();
    let (gx, gy, gw, gh) = GRID_AT;
    let mut header = div()
        .flex()
        .h(px(ROW_HEIGHT))
        .bg(rgb(theme::PANEL))
        .text_xs()
        .text_color(rgb(theme::DIM));
    for (name, width) in COLUMNS {
        header = header.child(
            div()
                .flex_shrink_0()
                .w(px(width))
                .h(px(ROW_HEIGHT))
                .flex()
                .items_center()
                .px_1()
                .border_r_1()
                .border_b_1()
                .border_color(rgb(theme::BORDER))
                .child(name),
        );
    }
    let mut grid = crate::probe::measured("mirror-grid", div())
        .id("mirror-grid")
        .absolute()
        .left(px(gx))
        .top(px(gy))
        .w(px(gw))
        .h(px(gh))
        .flex()
        .flex_col()
        .overflow_scroll()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .child(header);
    let empty = Row {
        kind: String::new(),
        direction: String::new(),
        port: String::new(),
        extra: String::new(),
        write: false,
    };
    let rows: Vec<(usize, &Row)> = form
        .rows
        .iter()
        .enumerate()
        .chain(std::iter::once((form.rows.len(), &empty)))
        .collect();
    let mut lists: Vec<AnyElement> = Vec::new();
    for (index, row) in rows {
        let mut line = div().flex().h(px(ROW_HEIGHT));
        let started = holder.is_started(index);
        let texts = [
            row.kind.clone(),
            row.direction.clone(),
            row.port.clone(),
            row.extra.clone(),
            if row.write { "\u{2611}" } else { "\u{2610}" }.to_owned(),
            if started { STARTED } else { GO }.to_owned(),
        ];
        for (column, (text, (_, width))) in texts.into_iter().zip(COLUMNS).enumerate() {
            let id = format!("mirror-cell-{index}-{column}");
            let editing = form.editing == Some((index, column));
            line = line.child(cell(
                id,
                text,
                width,
                editing,
                &form.field,
                typing,
                index,
                column,
                cx,
            ));
        }
        grid = grid.child(line);
        if form.list == Some((index, 0)) || form.list == Some((index, 1)) {
            let column = form.list.map_or(0, |(_, c)| c);
            let items: &[&str] = if column == 0 { &TYPES } else { &DIRECTIONS };
            #[allow(clippy::cast_precision_loss)] // a handful of rows
            let top = gy + ROW_HEIGHT * (index as f32 + 2.0);
            let left = gx + if column == 0 { 0.0 } else { COLUMNS[0].1 };
            let mut list = div()
                .absolute()
                .left(px(left))
                .top(px(top))
                .w(px(COLUMNS.get(column).map_or(55.0, |c| c.1)))
                .flex()
                .flex_col()
                .bg(rgb(theme::PANEL))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .text_xs()
                .text_color(rgb(theme::TEXT));
            for (i, item) in items.iter().enumerate() {
                let id = format!("mirror-list-{i}");
                list = list.child(
                    crate::probe::measured(id.clone(), div())
                        .id(SharedString::from(id))
                        .px_1()
                        .cursor_pointer()
                        .hover(|style| style.bg(rgb(theme::BORDER)))
                        .child(*item)
                        .on_click(cx.listener(move |this, _event, _window, cx| {
                            with_settings(this, |mirror, persisted| mirror.choose(i, persisted));
                            cx.notify();
                        })),
                );
            }
            lists.push(list.into_any_element());
        }
    }
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(grid)
        .children(lists);
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "mirror-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                with_settings(this, |mirror, persisted| mirror.leave(persisted));
                access(this).close();
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("mirror", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    let over = div()
        .id("mirror-backdrop")
        .w(size.width)
        .h(size.height)
        .flex()
        .items_center()
        .justify_center()
        .occlude()
        .child(body);
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(0.0), px(0.0)))
                .child(over),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `Save` and `Load`: a row as the cells' JSON array, Go's cell included, and read back.
    /// `// C#: Controls/SerialOutputPass.cs:140-170`
    #[test]
    fn rows_are_saved_and_loaded_as_json_arrays() {
        let row = Row {
            kind: "UDP".to_owned(),
            direction: "Outbound".to_owned(),
            port: "14560".to_owned(),
            extra: "127.0.0.1".to_owned(),
            write: true,
        };
        assert_eq!(
            row.json(false),
            "[\"UDP\",\"Outbound\",\"14560\",\"127.0.0.1\",\"True\",\"Go\"]"
        );
        assert_eq!(Row::parse(&row.json(true)), Some(row.clone()));
        assert_eq!(
            Row::parse("[\"TCP\",\"Inbound\",\"5760\",\"\",\"False\",\"Started\"]")
                .map(|r| r.write),
            Some(false)
        );
        assert_eq!(Row::parse("not a row"), None);
        let mut persisted = Persisted::at(None);
        let mut mirror = MavlinkMirror::default();
        mirror.show(&persisted);
        mirror.click_cell(0, 2, &mut persisted);
        assert_eq!(mirror.window.as_ref().map(|f| f.rows.len()), Some(1));
        mirror.window.as_mut().expect("form").field.set("14560");
        mirror.leave(&mut persisted);
        mirror.click_cell(0, 0, &mut persisted);
        mirror.choose(2, &mut persisted);
        mirror.click_cell(0, 1, &mut persisted);
        mirror.choose(1, &mut persisted);
        mirror.click_cell(0, 3, &mut persisted);
        mirror.window.as_mut().expect("form").field.set("127.0.0.1");
        mirror.leave(&mut persisted);
        mirror.click_cell(0, 4, &mut persisted);
        let saved = crate::raw_params_grid::get_list(persisted.get(SETTING));
        assert_eq!(saved, [row.json(false)]);
        // Opened again: the row is back.
        mirror.close();
        mirror.show(&persisted);
        assert_eq!(
            mirror.window.as_ref().map(|f| f.rows.clone()),
            Some(vec![row])
        );
    }

    /// Go on a UDP Outbound row: a mirror started and the row in `Started`, shown so when the
    /// form opens again; a bad port is "Error: " and the C#'s message.
    /// `// C#: Controls/SerialOutputPass.cs:177-258`
    #[test]
    fn go_starts_a_mirror_and_marks_the_row() {
        let mut persisted = Persisted::at(None);
        let (telemetry, _vehicle) =
            crate::telemetry::scripted::Vehicle::connect(mp_link::ProtocolTimeouts::default());
        let mut mirror = MavlinkMirror::default();
        persisted.set(
            SETTING,
            crate::raw_params_grid::set_list(&[
                "[\"UDP\",\"Outbound\",\"14561\",\"127.0.0.1\",\"False\",\"Go\"]".to_owned(),
                "[\"TCP\",\"Outbound\",\"x\",\"127.0.0.1\",\"False\",\"Go\"]".to_owned(),
            ])
            .expect("a list"),
        );
        mirror.show(&persisted);
        assert_eq!(mirror.window.as_ref().map(|f| f.rows.len()), Some(2));
        mirror.press_go(0, &mut persisted);
        assert!(mirror.is_started(0));
        assert_eq!(mirror.finish_opening(&telemetry), None);
        assert_eq!(mirror.open_count(), 1);
        assert_eq!(
            persisted.get(&format!("UDP_host_{UDP_CONFIG_REF}")),
            Some("127.0.0.1")
        );
        let saved = crate::raw_params_grid::get_list(persisted.get(SETTING));
        assert!(saved[0].ends_with("\"Started\"]"), "{}", saved[0]);
        mirror.press_go(1, &mut persisted);
        assert_eq!(
            mirror.tick(&telemetry),
            Some("Error: Input string was not in a correct format.".to_owned())
        );
        assert!(!mirror.is_started(1));
        mirror.close();
        mirror.show(&persisted);
        assert!(mirror.is_started(0));
    }

    /// The GUI script names only facts and controls this window has.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-mirror.gui");
        let source = include_str!("mavlink_mirror.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.mirror.") => {
                    let head: String = key.split('.').take(3).collect::<Vec<_>>().join(".");
                    assert!(
                        source.contains(&format!("\"{key}\""))
                            || source.contains(&format!("\"{head}.")),
                        "{key} is not recorded"
                    );
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("mirror-") => {
                    let fixed = id.split('@').next().unwrap_or(id);
                    let drawn = source.contains(&format!("\"{fixed}\""))
                        || fixed.starts_with("mirror-cell-")
                        || fixed.starts_with("mirror-list-");
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts >= 6, "{facts} facts");
    }
}
