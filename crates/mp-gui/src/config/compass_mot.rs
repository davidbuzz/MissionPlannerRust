//! Compass/Motor Calib: `GCSViews/ConfigurationView/ConfigCompassMot.cs`, an Optional Hardware
//! page of Initial Setup (`GCSViews/InitialSetup.cs:283-286`), listed once every parameter is in.
//!
//! **This page runs the motors.** Start sends ArduPilot's `compassmot`
//! (`MAV_CMD_PREFLIGHT_CALIBRATION`, param6 1), which spins them to the pilot's throttle while it
//! measures the interference they cause on the compass. The C# page says nothing of it: no
//! warning, no question - this port adds none.
//!
//! What it shows, at the Designer's places in a 634 x 400 page: the Start button, the vehicle's
//! messages below it, the last `COMPASSMOT_STATUS` read out beside them, and a chart of the
//! interference (percent, left) and the current (amps, right) against the throttle.
//!
//! * `Activate` sets the button to "Start" and subscribes to the vehicle's `COMPASSMOT_STATUS`
//!   (`ConfigCompassMot.cs:25-30`); `Deactivate` sends `SendAck` if the link is open - the
//!   `COMMAND_ACK` that ends a `compassmot` - unsubscribes and stops the timer (`:32-48`).
//! * The button reads "Finish" and runs `DoCompassMot`, then starts the timer (`:78-85`).
//!   `DoCompassMot` either stops - `SendAck`, the button back to "Start" - or starts: the
//!   vehicle's messages cleared, the chart's two curves emptied, and `doCommand(..., 0, 0, 0, 0,
//!   0, 1, 0)`, which the C# sends twice and does not wait for; its `catch` says "Compassmot
//!   requires AC 3.2+" (`:50-74`). `incompassmot` is not reset by `Deactivate`, so a page left
//!   mid-run and shown again says "Start" and stops on its next click, as the C#'s does.
//! * Each `COMPASSMOT_STATUS` adds a point to each list - throttle over ten against interference
//!   and against current - sorts both by throttle, writes the read-out and puts the lists back on
//!   the curves (`:87-119`). The lists are never cleared: the curves are emptied at a start, and
//!   the next status puts the whole of both lists back.
//! * The timer, every 100 ms while it runs, writes the vehicle's messages since the start into
//!   the box, one per line (`:164-173`).
//!
//! The chart is the part of ZedGraph `setupgraph` uses (`:121-162`): the title, the two curves
//! without symbols - interference red on the left axis, current green on the right - both axes
//! fixed from 0 to 100 in steps of 10, and the right axis picked from the current's range as
//! `AxisChange` picks it ([`crate::plan::elevation::Scale::pick`]).
//!
//! What is not carried over, and why:
//!
//! * the timer's interval: the box is written every frame while the timer runs, from the same
//!   messages - what it shows is the C#'s within a frame;
//! * typing into the box: the C#'s is editable and the timer overwrites it every 100 ms; here it
//!   is drawn read-only, the last lines showing as `ScrollToCaret` leaves it;
//! * `SendAck`'s 20 ms between its two packets: both go to the link at once;
//! * the vehicle's messages are the link's `STATUSTEXT` lines from this vehicle - the C#'s
//!   `cs.messages` - read from the log the application keeps, which drops the oldest past its
//!   size;
//! * ZedGraph's zoom, pan and context menu on the chart.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;

use gpui::{AnyElement, Context, SharedString, Window, div, prelude::*, px, rgb};
use mp_link::messages::LogMessage;
use mp_mavlink_dialects::all::CompassmotStatus;
use mp_vehicle::VehicleId;

use super::optional::{at, button, message_box};
use crate::MissionPlanner;
use crate::config::accel_calibration::is_command_ack_line;
use crate::config::servo_output::{ERROR_TITLE, Message};
use crate::plan::elevation::Scale;
use crate::setup::Key;
use crate::telemetry::{Report, Telemetry, TelemetryView};
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list, `backstageViewPagecompassmot.Text`.
/// `// C#: GCSViews/InitialSetup.resx:324-326`
pub const TITLE: &str = "Compass/Motor Calib";

/// `lbl_start.Text`, the button's text when idle.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.Designer.cs:70`
pub const START: &str = "Start";
/// `lbl_finish.Text`, while running.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.Designer.cs:79`
pub const FINISH: &str = "Finish";
/// `lbl_status.Text` before the first status.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.Designer.cs:88`
pub const STATUS: &str = "Compass Motor Calibration";
/// `DoCompassMot`'s `catch`.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:70`
pub const NEEDS_AC_3_2: &str = "Compassmot requires AC 3.2+";

/// The chart's title.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:126`
pub const CHART_TITLE: &str = "Compass Motor Calibration";
/// The X axis's title.
pub const X_TITLE: &str = "Throttle %";
/// The Y axis's title.
pub const Y_TITLE: &str = "Interference %";
/// The Y2 axis's title.
pub const Y2_TITLE: &str = "Amps";
/// `interference`'s label.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:15`
pub const INTERFERENCE: &str = "Interference";
/// `current`'s label.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:13`
pub const CURRENT: &str = "Current";
/// `Color.Red`, the interference.
const RED: u32 = 0xff_00_00;
/// `Color.Green`, the current: .NET's `Green` is 0, 128, 0.
const GREEN: u32 = 0x00_80_00;

/// `MAV_CMD_PREFLIGHT_CALIBRATION`'s parameters for `compassmot`: param6 1.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:66`
pub const COMPASSMOT_PARAMS: [f32; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0];

/// `$this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.Designer.cs:120`
const PAGE_SIZE: (f32, f32) = (634.0, 400.0);
/// `BUT_compassmot`.
const BUTTON_AT: (f32, f32, f32, f32) = (193.0, 9.0, 75.0, 23.0);
/// `txt_status`.
const TEXT_AT: (f32, f32, f32, f32) = (55.0, 38.0, 353.0, 111.0);
/// `lbl_status.Location`.
const LABEL_AT: (f32, f32) = (414.0, 41.0);
/// `zedGraphControl1`.
const CHART_AT: (f32, f32, f32, f32) = (3.0, 168.0, 627.0, 229.0);
/// The plot's margins inside the chart, for the title, the legend and the three axes' labels.
const MARGINS: (f32, f32, f32, f32) = (48.0, 48.0, 48.0, 36.0);

/// The X and Y axes: `Min = 0`, `Max = 100`, `MajorStep = 10`, `MinorStep = 5`.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:150-158`
pub const FIXED_AXIS: Scale = Scale {
    min: 0.0,
    max: 100.0,
    major_step: 10.0,
    minor_step: 5.0,
    mag: 0,
    decimals: 0,
};

/// A `PointPairList`, sorted by X as `Sort()` leaves it.
pub type Points = Vec<(f64, f64)>;

/// One point of a curve: throttle percent, and the value.
pub type Point = (f64, f64);

/// The page object.
#[derive(Debug)]
pub struct CompassMot {
    made_for: Option<Key>,
    active: bool,
    /// `BUT_compassmot.Text`.
    button: &'static str,
    /// `incompassmot`.
    running: bool,
    /// `sub`: the vehicle subscribed to, `sysidcurrent`/`compidcurrent` at `Activate`.
    subscription: Option<VehicleId>,
    /// `timer1.Enabled`.
    timer: bool,
    /// `cs.messages.Clear()`: the vehicle and the last log line when the messages were cleared.
    cleared: Option<(VehicleId, Option<u64>)>,
    /// `txt_status.Text`.
    text: String,
    /// `lbl_status.Text`.
    label: String,
    /// `interferencelist`.
    interference: Points,
    /// `currentlist`.
    current: Points,
    /// Whether the curves hold the lists: false from a start until the next status.
    drawn: bool,
    messages: VecDeque<Message>,
    /// What the button has sent: starts, and the `SendAck`s of a stop or a `Deactivate`.
    sent: [usize; 2],
}

impl Default for CompassMot {
    /// `InitializeComponent` and `setupgraph`.
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            button: START,
            running: false,
            subscription: None,
            timer: false,
            cleared: None,
            text: String::new(),
            label: STATUS.to_owned(),
            interference: Vec::new(),
            current: Vec::new(),
            drawn: true,
            messages: VecDeque::new(),
            sent: [0; 2],
        }
    }
}

/// The read-out `ProcessCompassMotMSG` writes.
/// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:99-104`
#[must_use]
pub fn read_out(status: &CompassmotStatus) -> String {
    let fixed = |value: f32| crate::hud::format_single_fixed(value, 2);
    format!(
        "Current: {}\nx,y,z {},{},{}\nThrottle: {}\nInterference: {}",
        fixed(status.current),
        fixed(status.compensationx),
        fixed(status.compensationy),
        fixed(status.compensationz),
        mp_log::netfmt::double(f64::from(status.throttle) / 10.0),
        status.interference
    )
}

/// `PointPairList.Add` then `Sort()`: by X.
fn add_sorted(list: &mut Points, point: (f64, f64)) {
    list.push(point);
    list.sort_by(|a, b| a.0.total_cmp(&b.0));
}

impl CompassMot {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// `BUT_compassmot.Text`.
    #[must_use]
    pub const fn button(&self) -> &'static str {
        self.button
    }

    /// `incompassmot`.
    #[must_use]
    pub const fn running(&self) -> bool {
        self.running
    }

    /// `txt_status.Text`.
    #[must_use]
    pub fn text(&self) -> &str {
        &self.text
    }

    /// `lbl_status.Text`.
    #[must_use]
    pub fn label(&self) -> &str {
        &self.label
    }

    /// The curves as drawn: empty from a start until the next status.
    #[must_use]
    pub fn curves(&self) -> (&[Point], &[Point]) {
        if self.drawn {
            (&self.interference, &self.current)
        } else {
            (&[], &[])
        }
    }

    /// The right axis as `AxisChange` picks it from the current's range - 0 to 1 with no data,
    /// `Scale.SetRange`'s default.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/Scale.cs:2684-2709`
    #[must_use]
    pub fn y2_axis(&self) -> Scale {
        let (_, current) = self.curves();
        if current.is_empty() {
            return Scale::pick(0.0, 1.0, None);
        }
        let (low, high) = current
            .iter()
            .fold((f64::MAX, f64::MIN), |(low, high), (_, y)| (low.min(*y), high.max(*y)));
        Scale::pick(low, high, None)
    }

    /// What the button has sent: starts and acks.
    #[must_use]
    pub const fn sent(&self) -> [usize; 2] {
        self.sent
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

    /// Shows the page: a new page object for a new screen, then `Activate` - the button reads
    /// "Start" and the page subscribes to the vehicle's `COMPASSMOT_STATUS`.
    /// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:25-30`
    pub fn activate(&mut self, key: Key, vehicle: Option<VehicleId>) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            *self = Self {
                made_for: Some(key),
                messages,
                sent: self.sent,
                ..Self::default()
            };
        }
        self.active = true;
        self.button = START;
        self.subscription = vehicle;
    }

    /// `Deactivate`: `SendAck` while the link is open, the subscription ended, the timer
    /// stopped. `incompassmot` is left as it is.
    /// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:32-48`
    pub fn deactivate(&mut self, telemetry: &Telemetry) {
        if telemetry.send_calibration_ack() {
            self.sent[1] += 1;
        }
        self.subscription = None;
        self.timer = false;
        self.active = false;
    }

    /// `BUT_compassmot_Click`: "Finish", `DoCompassMot`, the timer started.
    /// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:50-85`
    pub fn click(&mut self, telemetry: &mut Telemetry, messages: &[LogMessage]) {
        if !self.active {
            return;
        }
        self.button = FINISH;
        if self.running {
            if telemetry.send_calibration_ack() {
                self.sent[1] += 1;
            }
            self.running = false;
            self.button = START;
        } else {
            let vehicle = telemetry.send_handle().map(|(_, id)| id);
            // `MainV2.comPort.MAV.cs.messages.Clear()`: what the log holds now is not this run's.
            self.cleared = vehicle.map(|id| (id, messages.last().map(|line| line.seq)));
            // `interference.Clear(); current.Clear();` - the curves, not the lists.
            self.drawn = false;
            let sent = vehicle.and_then(|id| {
                telemetry.command(
                    id,
                    mp_calibration::CMD_PREFLIGHT_CALIBRATION,
                    COMPASSMOT_PARAMS,
                    Report::default(),
                )
            });
            if sent.is_some() {
                self.sent[0] += 1;
            } else {
                self.messages.push_back(Message {
                    title: ERROR_TITLE,
                    text: NEEDS_AC_3_2.to_owned(),
                });
            }
            self.running = true;
        }
        self.timer = true;
    }

    /// `ProcessCompassMotMSG` for one status.
    /// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:87-119`
    pub fn status(&mut self, status: &CompassmotStatus) {
        let throttle = f64::from(status.throttle) / 10.0;
        add_sorted(
            &mut self.interference,
            (throttle, f64::from(status.interference)),
        );
        add_sorted(&mut self.current, (throttle, f64::from(status.current)));
        self.label = read_out(status);
        self.drawn = true;
    }

    /// `timer1_Tick`: the vehicle's messages since the start, each on a line of its own.
    /// `// C#: GCSViews/ConfigurationView/ConfigCompassMot.cs:164-173`
    fn timer_tick(&mut self, messages: &[LogMessage]) {
        let Some((vehicle, since)) = self.cleared else {
            return;
        };
        let mut text = String::new();
        for line in messages {
            if since.is_some_and(|since| line.seq <= since)
                || line.from != vehicle
                || is_command_ack_line(&line.text)
            {
                continue;
            }
            text.push_str(&line.text);
            text.push_str("\r\n");
        }
        self.text = text;
    }

    /// Once a frame: a page object whose screen has gone is let go; the statuses the
    /// subscription is handed, each in turn; the timer.
    pub fn tick(&mut self, telemetry: &Telemetry, view: &TelemetryView, on_setup: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            let messages = std::mem::take(&mut self.messages);
            *self = Self {
                messages,
                sent: self.sent,
                ..Self::default()
            };
        }
        // Taken every frame, so none that arrived unsubscribed is handed over later.
        let statuses = telemetry.take_compassmot_status();
        if let Some(vehicle) = self.subscription {
            for (from, status) in statuses {
                if from == vehicle {
                    self.status(&status);
                }
            }
        }
        if self.timer {
            self.timer_tick(&view.messages);
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &CompassMot) {
    use crate::facts::record;
    record("config.compassmot.active", page.is_active());
    record("config.compassmot.button", page.button());
    record("config.compassmot.running", page.running());
    record("config.compassmot.subscribed", page.subscription.is_some());
    record("config.compassmot.timer", page.timer);
    record("config.compassmot.text", page.text().replace("\r\n", " | "));
    record("config.compassmot.label", page.label());
    let (interference, _) = page.curves();
    record("config.compassmot.points", interference.len());
    record("config.compassmot.points.held", page.interference.len());
    let y2 = page.y2_axis();
    record("config.compassmot.y2", format!("{}..{}", y2.min, y2.max));
    record("config.compassmot.starts", page.sent()[0]);
    record("config.compassmot.acks", page.sent()[1]);
    record(
        "config.compassmot.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
}

/// The chart: the title, the legend, the three axes' titles and labels, and the curves.
fn chart(page: &CompassMot) -> AnyElement {
    let (_, _, width, height) = CHART_AT;
    let (left, top, right, bottom) = MARGINS;
    let plot_width = width - left - right;
    let plot_height = height - top - bottom;
    let y2 = page.y2_axis();
    #[allow(clippy::cast_possible_truncation)]
    let across = |value: f64| FIXED_AXIS.fraction(value) as f32;
    #[allow(clippy::cast_possible_truncation)]
    let up = |scale: &Scale, value: f64| scale.fraction(value) as f32;
    let text = |content: String, colour: u32| {
        div()
            .absolute()
            .text_xs()
            .text_color(rgb(colour))
            .child(content)
    };
    let mut chart = crate::probe::measured("compassmot-chart", div())
        .absolute()
        .left(px(CHART_AT.0))
        .top(px(CHART_AT.1))
        .w(px(width))
        .h(px(height))
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .child(
            div()
                .absolute()
                .top(px(4.0))
                .left_0()
                .right_0()
                .flex()
                .justify_center()
                .text_sm()
                .font_weight(gpui::FontWeight::BOLD)
                .text_color(rgb(theme::TEXT))
                .child(CHART_TITLE),
        )
        .child(
            div()
                .absolute()
                .top(px(24.0))
                .left_0()
                .right_0()
                .flex()
                .justify_center()
                .gap_4()
                .children([(INTERFERENCE, RED), (CURRENT, GREEN)].map(|(label, colour)| {
                    div()
                        .flex()
                        .items_center()
                        .gap_1()
                        .child(div().w(px(24.0)).h(px(2.0)).bg(rgb(colour)))
                        .child(div().text_xs().text_color(rgb(theme::TEXT)).child(label))
                })),
        )
        .child(text(Y_TITLE.to_owned(), theme::TEXT).left(px(4.0)).top(px(top - 14.0)))
        .child(
            text(y2.title(Y2_TITLE), theme::TEXT)
                .right(px(4.0))
                .top(px(top - 14.0)),
        )
        .child(
            div()
                .absolute()
                .left(px(left))
                .w(px(plot_width))
                .top(px(top + plot_height + 16.0))
                .flex()
                .justify_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(X_TITLE),
        );
    for tic in FIXED_AXIS.tics() {
        chart = chart
            .child(
                div()
                    .absolute()
                    .left(px(left + across(tic) * plot_width - 20.0))
                    .w(px(40.0))
                    .top(px(top + plot_height + 2.0))
                    .flex()
                    .justify_center()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(FIXED_AXIS.label(tic)),
            )
            .child(
                div()
                    .absolute()
                    .left_0()
                    .w(px(left - 4.0))
                    .top(px(top + (1.0 - up(&FIXED_AXIS, tic)) * plot_height - 7.0))
                    .flex()
                    .justify_end()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(FIXED_AXIS.label(tic)),
            );
    }
    for tic in y2.tics() {
        chart = chart.child(
            div()
                .absolute()
                .left(px(left + plot_width + 4.0))
                .w(px(right - 4.0))
                .top(px(top + (1.0 - up(&y2, tic)) * plot_height - 7.0))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(y2.label(tic)),
        );
    }
    let (interference, current) = page.curves();
    let interference = interference.to_vec();
    let current = current.to_vec();
    chart
        .child(
            gpui::canvas(
                |_bounds, _window, _cx| (),
                move |bounds, (), window, _cx| {
                    paint(&interference, &current, y2, bounds, window);
                },
            )
            .absolute()
            .left(px(left))
            .top(px(top))
            .w(px(plot_width))
            .h(px(plot_height)),
        )
        .into_any_element()
}

/// The plot's border and the two curves, clipped to the plot as ZedGraph clips them.
fn paint(
    interference: &[(f64, f64)],
    current: &[(f64, f64)],
    y2: Scale,
    bounds: gpui::Bounds<gpui::Pixels>,
    window: &mut Window,
) {
    let width = f32::from(bounds.size.width);
    let height = f32::from(bounds.size.height);
    #[allow(clippy::cast_possible_truncation)]
    let at = |scale: &Scale, (x, y): (f64, f64)| {
        let across = (FIXED_AXIS.fraction(x) as f32).clamp(0.0, 1.0);
        let up = (scale.fraction(y) as f32).clamp(0.0, 1.0);
        gpui::point(
            bounds.origin.x + px(across * width),
            bounds.origin.y + px((1.0 - up) * height),
        )
    };
    let stroke = |window: &mut Window, points: &[gpui::Point<gpui::Pixels>], colour: u32| {
        let mut points = points.iter();
        let Some(first) = points.next() else {
            return;
        };
        let mut builder = gpui::PathBuilder::stroke(px(1.0));
        builder.move_to(*first);
        for point in points {
            builder.line_to(*point);
        }
        if let Ok(path) = builder.build() {
            window.paint_path(path, gpui::Hsla::from(rgb(colour)));
        }
    };
    let corner = |x: f32, y: f32| {
        gpui::point(
            bounds.origin.x + px(x * width),
            bounds.origin.y + px(y * height),
        )
    };
    stroke(
        window,
        &[
            corner(0.0, 0.0),
            corner(1.0, 0.0),
            corner(1.0, 1.0),
            corner(0.0, 1.0),
            corner(0.0, 0.0),
        ],
        theme::BORDER,
    );
    // ZedGraph draws the last curve added first: the current, then the interference over it.
    let current: Vec<_> = current.iter().map(|point| at(&y2, *point)).collect();
    stroke(window, &current, GREEN);
    let interference: Vec<_> = interference
        .iter()
        .map(|point| at(&FIXED_AXIS, *point))
        .collect();
    stroke(window, &interference, RED);
}

/// The page, laid out as the Designer lays it out.
pub fn page(mot: &CompassMot, cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !mot.is_active() {
        return div().into_any_element();
    }
    let (tx, ty, tw, th) = TEXT_AT;
    let lines: Vec<String> = mot
        .text()
        .split("\r\n")
        .filter(|line| !line.is_empty())
        .map(str::to_owned)
        .collect();
    let body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(button(
            "compassmot-button",
            mot.button(),
            BUTTON_AT,
            true,
            |this, _window, _cx| {
                let view = this.telemetry.view();
                this.extra.compass_mot.click(&mut this.telemetry, &view.messages);
            },
            cx,
        ))
        .child(
            crate::probe::measured("compassmot-text", at(tx, ty, tw, th))
                .id(SharedString::from("compassmot-text"))
                .flex()
                .flex_col()
                .justify_end()
                .overflow_hidden()
                .px_1()
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .children(lines.into_iter().map(|line| div().child(line))),
        )
        .child(
            div()
                .absolute()
                .left(px(LABEL_AT.0))
                .top(px(LABEL_AT.1))
                .flex()
                .flex_col()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .children(mot.label().lines().map(|line| div().child(line.to_owned()))),
        )
        .child(chart(mot));
    panel(TITLE, body).into_any_element()
}

/// The message box showing, over the whole window.
pub fn overlay(
    mot: &CompassMot,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let message = mot.message()?;
    Some(message_box(
        "compassmot-message",
        "compassmot-message-ok",
        message,
        window,
        |this| this.extra.compass_mot.dismiss_message(),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::scripted::{VEHICLE, Vehicle, until};
    use mp_link::ProtocolTimeouts;
    use mp_mavlink_dialects::all::MavMessage;

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn status(throttle: u16, current: f32, interference: u16) -> CompassmotStatus {
        CompassmotStatus {
            current,
            compensationx: 0.125,
            compensationy: -2.5,
            compensationz: 10.0,
            throttle,
            interference,
        }
    }

    fn compassmot_commands(vehicle: &Vehicle) -> usize {
        vehicle.count(|message| {
            matches!(message, MavMessage::CommandLong(long)
                if long.command == mp_calibration::CMD_PREFLIGHT_CALIBRATION
                    && (long.param6 - 1.0).abs() < f32::EPSILON)
        })
    }

    fn acks(vehicle: &Vehicle) -> usize {
        vehicle.count(|message| {
            matches!(message, MavMessage::CommandAck(ack)
                if ack.command == mp_calibration::CMD_PREFLIGHT_CALIBRATION && ack.result == 0)
        })
    }

    /// The Designer's words and places, read from the tree when it is here.
    #[test]
    fn the_text_is_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigCompassMot.Designer.cs",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        assert!(designer.contains(&format!("this.lbl_start.Text = \"{START}\";")));
        assert!(designer.contains(&format!("this.lbl_finish.Text = \"{FINISH}\";")));
        assert!(designer.contains(&format!("this.lbl_status.Text = \"{STATUS}\";")));
        assert!(designer.contains("this.BUT_compassmot.Location = new System.Drawing.Point(193, 9);"));
        assert!(designer.contains("this.txt_status.Location = new System.Drawing.Point(55, 38);"));
        assert!(designer.contains("this.zedGraphControl1.Location = new System.Drawing.Point(3, 168);"));
        assert!(designer.contains("this.Size = new System.Drawing.Size(634, 400);"));
        let Some(cs) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigCompassMot.cs",
        ) else {
            return;
        };
        for text in [NEEDS_AC_3_2, CHART_TITLE, X_TITLE, Y_TITLE, Y2_TITLE, INTERFERENCE, CURRENT] {
            assert!(cs.contains(&format!("\"{text}\"")), "{text}");
        }
        assert!(cs.contains("MAV_CMD.PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 1, 0"));
    }

    /// The read-out, a float's "0.00" and the throttle as a double.
    #[test]
    fn the_read_out_is_the_cs() {
        assert_eq!(
            read_out(&status(455, 12.345, 37)),
            "Current: 12.35\nx,y,z 0.13,-2.50,10.00\nThrottle: 45.5\nInterference: 37"
        );
        assert_eq!(
            read_out(&status(500, 3.0, 0)),
            "Current: 3.00\nx,y,z 0.13,-2.50,10.00\nThrottle: 50\nInterference: 0"
        );
    }

    /// Each status adds a point to each list, sorted by throttle; the right axis follows the
    /// current.
    #[test]
    fn statuses_are_plotted_in_throttle_order() {
        let mut page = CompassMot::default();
        page.activate(key(), Some(VEHICLE));
        assert_eq!(page.y2_axis().min, 0.0);
        page.status(&status(500, 10.0, 40));
        page.status(&status(200, 4.0, 15));
        let (interference, current) = page.curves();
        assert_eq!(interference, [(20.0, 15.0), (50.0, 40.0)]);
        assert_eq!(current, [(20.0, 4.0), (50.0, 10.0)]);
        let y2 = page.y2_axis();
        assert!(y2.min <= 4.0 && y2.max >= 10.0, "{y2:?}");
        assert!(page.label().starts_with("Current: 4.00"));
    }

    /// Start through the real link: the command twice, not waited for; the curves emptied
    /// until the next status, which brings the old points back - the lists are never cleared.
    /// Finish: `SendAck`, twice. Leaving the page acks again.
    #[test]
    fn start_and_finish_over_the_link() {
        let (mut telemetry, mut vehicle) = Vehicle::connect(ProtocolTimeouts::default());
        let mut page = CompassMot::default();
        let view = telemetry.view();
        page.activate(Key::of(&view), view.vehicle);
        assert_eq!(page.button(), START);
        page.status(&status(100, 1.0, 5));

        page.click(&mut telemetry, &[]);
        assert_eq!(page.button(), FINISH);
        assert!(page.running());
        assert_eq!(page.sent(), [1, 0]);
        assert!(page.curves().0.is_empty(), "the curves are emptied");
        until("compassmot, twice", || {
            vehicle.read();
            compassmot_commands(&vehicle) == 2
        });

        // A status from the vehicle, through the link to the page's subscription.
        vehicle.send(&MavMessage::CompassmotStatus(status(300, 6.5, 22)));
        until("the status", || {
            page.tick(&telemetry, &telemetry.view(), true);
            page.label().contains("Throttle: 30")
        });
        assert_eq!(page.curves().0.len(), 2, "the old point and the new");

        page.click(&mut telemetry, &[]);
        assert_eq!(page.button(), START);
        assert!(!page.running());
        until("SendAck, twice", || {
            vehicle.read();
            acks(&vehicle) == 2
        });

        page.deactivate(&telemetry);
        until("Deactivate's SendAck", || {
            vehicle.read();
            acks(&vehicle) == 4
        });
        assert_eq!(page.sent(), [1, 2]);
    }

    /// Without a vehicle the command is not sent: the `catch`'s box.
    #[test]
    fn with_no_vehicle_start_says_it_needs_ac_3_2() {
        let mut telemetry = Telemetry::idle();
        let mut page = CompassMot::default();
        page.activate(key(), None);
        page.click(&mut telemetry, &[]);
        assert_eq!(
            page.message().map(|message| message.text.as_str()),
            Some(NEEDS_AC_3_2)
        );
        assert!(page.running(), "incompassmot is set after the catch");
        assert_eq!(page.button(), FINISH);
    }

    /// The timer writes this vehicle's messages since the start, one a line, and not the
    /// acknowledgements the log also holds.
    #[test]
    fn the_timer_writes_the_messages_since_the_start() {
        let line = |seq: u64, from: VehicleId, text: &str| LogMessage {
            from,
            severity: mp_link::messages::Severity::Info,
            text: text.to_owned(),
            seq,
            received: 0,
        };
        let other = VehicleId::new(2, 1);
        let before = [line(1, VEHICLE, "old")];
        let mut page = CompassMot::default();
        page.activate(key(), Some(VEHICLE));
        page.cleared = Some((VEHICLE, before.last().map(|line| line.seq)));
        page.timer = true;
        let log = [
            line(1, VEHICLE, "old"),
            line(2, VEHICLE, "Starting calibration"),
            line(3, other, "someone else"),
            line(4, VEHICLE, "MAV_CMD_PREFLIGHT_CALIBRATION: accepted"),
            line(5, VEHICLE, "Calibration successful"),
        ];
        page.timer_tick(&log);
        assert_eq!(
            page.text(),
            "Starting calibration\r\nCalibration successful\r\n"
        );
    }

    /// `Deactivate` leaves `incompassmot`: shown again, the page says "Start" and its next click
    /// stops.
    #[test]
    fn a_page_left_mid_run_stops_on_its_next_click() {
        let mut telemetry = Telemetry::idle();
        let mut page = CompassMot::default();
        page.activate(key(), None);
        page.running = true;
        page.deactivate(&telemetry);
        page.activate(key(), None);
        assert_eq!(page.button(), START);
        page.click(&mut telemetry, &[]);
        assert!(!page.running());
        assert_eq!(page.button(), START);
    }

    /// Every fact the GUI script asserts on is one this page records, and every control it
    /// clicks is one this page draws.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-compassmot.gui");
        let source = include_str!("compass_mot.rs");
        assert!(
            script.contains("never with propellers fitted"),
            "the script's header says it"
        );
        let mut facts = 0;
        let mut clicks = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.compassmot.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("compassmot-") => {
                    assert!(source.contains(&format!("\"{id}\"")), "{id} is not drawn");
                    clicks += 1;
                }
                _ => {}
            }
        }
        assert!(facts > 6 && clicks >= 2, "{facts} facts, {clicks} clicks");
    }
}
