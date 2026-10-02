//! The RAW Sensor window, `Controls/RAW_Sensor.cs`: the flight screen's Raw Sensor View button
//! makes a new form and shows it with `Show()`, over the screen and beside it. A tab control fills
//! the form: Raw Sensor - the roll, pitch and yaw dials, the six check boxes, Save CSV, the
//! update-rate combo and a ten-second chart of the accelerometer and gyro - and Radio, eight
//! input bars beside eight output bars. The Designer also builds a Flight Data page
//! (`tabOrientation`, ten bars of the same values) that it never adds to the tab control, so
//! the C# never shows it and it is not drawn here.
//!
//! Two timers drive the form: every 10 ms the current state is sampled into the curves that are
//! checked (an unchecked curve is emptied) and, once Save CSV has named a file, written to it as
//! a line; every 100 ms the chart's X axis is moved on and the chart redrawn. A tab change asks
//! the vehicle for the RAW_SENSORS stream at its `ratesensors`, and the Load does so twice
//! (Radio, then Raw Sensor); the combo writes `ratesensors` and asks at the new rate. Without a
//! link, and not reading a log, the tab change says "Please connect first" and closes the form.
//!
//! Divergences, each at its site: the form is one window here, where every click of the button
//! opens another in the C#; "Please connect first" goes on the status line and the window stays
//! shut (the owner's rule for avoidable errors); the sampling runs on the frame, at most every
//! 10 ms; the chart redraws every frame; the CSV writer is closed whenever the window closes,
//! where the C# closes it only while its port is open; a failed write or a file that cannot be
//! made goes on the status line where the C# would throw.
//! `// C#: Controls/RAW_Sensor.cs, Controls/RAW_Sensor.Designer.cs, Controls/RAW_Sensor.resx,
//! GCSViews/FlightData.cs:1464-1469`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::fs::File;
use std::io::{BufWriter, Write as _};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use gpui::{AnyElement, Context, SharedString, Window, div, prelude::*, px, rgb};
use mp_chart::{Series, auto_range};
use mp_vehicle::VehicleState;

use crate::MissionPlanner;
use crate::gauge::Dial;
use crate::telemetry::TelemetryView;
use crate::ui::{action, theme};

/// `$this.Text`.
pub const TITLE: &str = "RAW Sensor";
/// `tabControl1_SelectedIndexChanged`'s box without a link. `// C#: Controls/RAW_Sensor.cs:245`
pub const PLEASE_CONNECT: &str = "Please connect first";
/// `label3`. `// C#: Controls/RAW_Sensor.resx (label3.Text)`
const NOTE: &str = "Note: There is a delay  when viewing via Xbee @ 50hz";
/// `CMB_rawupdaterate.Text` before a rate is chosen.
const RATE_PROMPT: &str = "Update Speed";
/// `CMB_rawupdaterate.Items`.
const RATES: [&str; 3] = ["3", "10", "50"];
/// Each tab page's size, `tabRadio.Size`.
const PAGE: (f32, f32) = (751.0, 478.0);
/// `timer2serial.Interval`.
const SAMPLE: Duration = Duration::from_millis(10);
/// `RollingPointPairList(10 * 50)`.
const POINTS: usize = 10 * 50;
/// Where the window sits over the screen.
const AT: (f32, f32) = (380.0, 40.0);
/// The chart's columns, one a pixel of its plot.
const COLUMNS: usize = 690;

/// `Color.Red`.
const RED: u32 = 0xff_00_00;
/// The six curves, `CreateChart`'s names and colours in its order: Red, Green, SandyBrown, Blue,
/// Black, Violet. `// C#: Controls/RAW_Sensor.cs:49-54`
const TRACES: [(&str, u32); 6] = [
    ("Accel X", RED),
    ("Accel Y", 0x00_80_00),
    ("Accel Z", 0xf4_a4_60),
    ("Gyro X", 0x00_00_ff),
    ("Gyro Y", 0x00_00_00),
    ("Gyro Z", 0xee_82_ee),
];
/// `chkax` to `chkgz`, at (685, 16) and every 23 down, all checked.
/// `// C#: Controls/RAW_Sensor.resx (chkax.Location ...)`
const CHECKS: [&str; 6] = [
    "raw-chk-ax",
    "raw-chk-ay",
    "raw-chk-az",
    "raw-chk-gx",
    "raw-chk-gy",
    "raw-chk-gz",
];

/// `HorizontalProgressBar.Minimum` on the Radio page's bars.
const BAR_MIN: i32 = 1000;
/// `HorizontalProgressBar.Maximum`.
const BAR_MAX: i32 = 2000;

/// The tab control's pages, in its order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Tab {
    /// `tabRawSensor`, "Raw Sensor".
    #[default]
    RawSensor,
    /// `tabRadio`, "Radio".
    Radio,
}

impl Tab {
    /// `Text`.
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::RawSensor => "Raw Sensor",
            Self::Radio => "Radio",
        }
    }

    const fn id(self) -> &'static str {
        match self {
            Self::RawSensor => "raw-tab-rawsensor",
            Self::Radio => "raw-tab-radio",
        }
    }
}

/// The window's state: one form, made anew each time the button opens it.
#[derive(Debug)]
pub struct RawSensor {
    /// Whether the form is showing.
    open: bool,
    /// `tabControl.SelectedTab`.
    tab: Tab,
    /// `tickStart`: when the form was made, which the curves' seconds count from.
    opened: Option<Instant>,
    /// `list1` to `list6`.
    series: Vec<Series>,
    /// `chkax.Checked` to `chkgz.Checked`.
    checked: [bool; 6],
    /// `CMB_rawupdaterate.Text`.
    rate: &'static str,
    /// Whether the combo's list is down.
    rate_open: bool,
    /// `sw`, the CSV writer, once Save CSV has named a file.
    csv: Option<BufWriter<File>>,
    /// The file it writes.
    csv_path: Option<PathBuf>,
    /// Lines written to it.
    csv_lines: usize,
    /// When the state was last sampled.
    last_sample: Option<Instant>,
    /// `ax`, `ay`, `az`, `gx`, `gy`, `gz` as last sampled.
    latest: [f32; 6],
}

impl Default for RawSensor {
    fn default() -> Self {
        Self::new()
    }
}

impl RawSensor {
    /// No form showing.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            open: false,
            tab: Tab::RawSensor,
            opened: None,
            series: Vec::new(),
            checked: [true; 6],
            rate: RATE_PROMPT,
            rate_open: false,
            csv: None,
            csv_path: None,
            csv_lines: 0,
            last_sample: None,
            latest: [0.0; 6],
        }
    }

    /// Whether the form is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open
    }

    /// `tabControl.SelectedTab`.
    #[must_use]
    pub const fn tab(&self) -> Tab {
        self.tab
    }

    /// `CMB_rawupdaterate.Text`.
    #[must_use]
    pub const fn rate(&self) -> &'static str {
        self.rate
    }

    /// `BUT_RAWSensor_Click`: `new RAW_Sensor()` - the chart made with its six empty curves and
    /// `tickStart` taken - then `Show()`, whose Load starts the 10 ms timer and selects Radio,
    /// then Raw Sensor, each selection going through `tabControl1_SelectedIndexChanged`: without
    /// a link and not reading a log that says "Please connect first" and closes the form, which
    /// is `Err` here with the window left shut; with one it asks for the RAW_SENSORS stream,
    /// which is the caller's to send. The form is new each time: every setting starts over.
    /// `// C#: GCSViews/FlightData.cs:1464-1469, Controls/RAW_Sensor.cs:22-29, 209-220, 239-267`
    pub fn open(&mut self, connected: bool) -> Result<(), &'static str> {
        *self = Self::new();
        if !connected {
            return Err(PLEASE_CONNECT);
        }
        self.open = true;
        self.opened = Some(Instant::now());
        self.series = TRACES
            .iter()
            .map(|(name, _)| Series::new(*name, POINTS))
            .collect();
        Ok(())
    }

    /// `ACM_Setup_FormClosed`: the CSV writer closed and both timers stopped. The C# closes the
    /// writer only while its port is open; it is closed here whatever the link is doing, so what
    /// was written is on disk. `// C#: Controls/RAW_Sensor.cs:222-237`
    pub fn close(&mut self) {
        *self = Self::new();
    }

    /// A click on a tab: `tabControl1_SelectedIndexChanged` for the page chosen; the stream
    /// request it makes is the caller's. `// C#: Controls/RAW_Sensor.cs:239-267`
    pub fn select(&mut self, tab: Tab) {
        self.tab = tab;
        self.rate_open = false;
    }

    /// `chkax` to `chkgz` clicked: checked or not. The curve of one unchecked is emptied at the
    /// next sample. `// C#: Controls/RAW_Sensor.cs:159-206`
    pub fn toggle(&mut self, index: usize) {
        if let Some(checked) = self.checked.get_mut(index) {
            *checked = !*checked;
        }
    }

    /// The combo's button: its list down or up.
    pub const fn toggle_rate_list(&mut self) {
        self.rate_open = !self.rate_open;
    }

    /// `CMB_rawupdaterate_SelectedIndexChanged`: the text chosen, and `int.Parse` of it for the
    /// caller to write as `ratesensors` and ask for. `// C#: Controls/RAW_Sensor.cs:269-274`
    pub fn set_rate(&mut self, text: &'static str) -> Option<i32> {
        self.rate = text;
        self.rate_open = false;
        text.parse().ok()
    }

    /// `BUT_savecsv_Click` once the dialog has a name: nothing for an empty one (the dialog
    /// closed without a file), else `.csv` added to a name with no extension (`AddExtension`,
    /// `DefaultExt = ".csv"`), the writer before it closed and a new one opened on the file.
    ///
    /// # Errors
    ///
    /// The file could not be made, as `ofd.OpenFile()` would throw.
    /// `// C#: Controls/RAW_Sensor.cs:278-298`
    pub fn save_csv_to(&mut self, text: &str) -> Result<(), String> {
        let name = text.trim();
        if name.is_empty() || Path::new(name).is_dir() {
            return Ok(());
        }
        let mut path = PathBuf::from(name);
        if path.extension().is_none() {
            path.set_extension("csv");
        }
        self.csv = None;
        let file = File::create(&path).map_err(|why| format!("{}: {why}", path.display()))?;
        self.csv = Some(BufWriter::new(file));
        self.csv_path = Some(path);
        self.csv_lines = 0;
        Ok(())
    }

    /// `timer2serial_Tick`, run on the frame and at most every 10 ms: nothing without a link
    /// that is open or a log being read; else the CSV line - `DateTime.Now`, then `ax`, `ay`,
    /// `az`, `gx`, `gy`, `gz` - when a file is named, and each value into its curve where its
    /// box is checked, the curve emptied where it is not. The seconds are counted from the form's
    /// making, as `Environment.TickCount - tickStart` counts them.
    ///
    /// Says what went wrong when the line could not be written, for the status line.
    /// `// C#: Controls/RAW_Sensor.cs:136-207`
    pub fn tick(&mut self, view: &TelemetryView) -> Option<String> {
        if !self.open || !view.connected {
            return None;
        }
        let now = Instant::now();
        if self
            .last_sample
            .is_some_and(|last| now.duration_since(last) < SAMPLE)
        {
            return None;
        }
        self.last_sample = Some(now);
        let state = view.state.as_deref()?;
        let imu = state.imu.first()?;
        let values = [
            imu.accel[0],
            imu.accel[1],
            imu.accel[2],
            imu.gyro[0],
            imu.gyro[1],
            imu.gyro[2],
        ];
        self.latest = values;
        let mut failure = None;
        if let Some(writer) = self.csv.as_mut() {
            // `string.Format("{0},{1},...", DateTime.Now.ToString(), cs.ax, ...)`: the date and
            // time as en-US writes them, each value as `float.ToString()` - its shortest form.
            let line = format!(
                "{},{},{},{},{},{},{}",
                chrono::Local::now().format("%-m/%-d/%Y %-I:%M:%S %p"),
                values[0],
                values[1],
                values[2],
                values[3],
                values[4],
                values[5]
            );
            match writeln!(writer, "{line}").and_then(|()| writer.flush()) {
                Ok(()) => self.csv_lines += 1,
                Err(why) => failure = Some(format!("Save CSV: {why}")),
            }
        }
        let time = self
            .opened
            .map_or(0.0, |opened| now.duration_since(opened).as_secs_f64());
        for (index, series) in self.series.iter_mut().enumerate() {
            if self.checked.get(index).copied().unwrap_or(false) {
                series.push(time, f64::from(values.get(index).copied().unwrap_or(0.0)));
            } else {
                series.clear();
            }
        }
        failure
    }

    /// What a script can see: whether the window is open, its tab, the combo's text, the boxes,
    /// the first curve's points, the values last sampled, the CSV file and its lines, and the
    /// Radio page's sixteen values and the dials' three.
    pub fn record_facts(&self, view: &TelemetryView) {
        crate::facts::record("raw.open", self.open);
        crate::facts::record("raw.tab", self.tab.text());
        crate::facts::record("raw.rate", self.rate);
        crate::facts::record(
            "raw.checked",
            self.checked
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(","),
        );
        crate::facts::record(
            "raw.points",
            self.series.first().map_or(0, mp_chart::Series::len),
        );
        crate::facts::record(
            "raw.values",
            self.latest
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(","),
        );
        crate::facts::record(
            "raw.csv",
            self.csv_path
                .as_ref()
                .map_or_else(|| "none".to_owned(), |path| path.display().to_string()),
        );
        crate::facts::record("raw.csv.lines", self.csv_lines);
        let state = view.state.as_deref();
        let joined = |values: [i32; 8]| {
            values
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(",")
        };
        crate::facts::record(
            "raw.rc",
            state.map_or_else(|| "none".to_owned(), |state| joined(rc_in(state))),
        );
        crate::facts::record(
            "raw.servo",
            state.map_or_else(|| "none".to_owned(), |state| joined(servo_out(state))),
        );
        crate::facts::record(
            "raw.attitude",
            state.map_or_else(
                || "none".to_owned(),
                |state| {
                    let (roll, pitch, yaw) = attitude_degrees(state);
                    format!("{roll:.1},{pitch:.1},{yaw:.1}")
                },
            ),
        );
    }
}

/// `cs.roll`, `cs.pitch` and `cs.yaw`: the attitude in degrees, as `CurrentState` keeps it.
/// `// C#: ExtLibs/ArduPilot/CurrentState.cs:3251-3253`
fn attitude_degrees(state: &VehicleState) -> (f32, f32, f32) {
    #[allow(clippy::cast_possible_truncation)] // degrees of attitude
    let degrees = |radians: mp_units::Radians| radians.to_degrees().0 as f32;
    (
        degrees(state.attitude.roll),
        degrees(state.attitude.pitch),
        degrees(state.attitude.yaw),
    )
}

/// `ch1in` to `ch8in`.
fn rc_in(state: &VehicleState) -> [i32; 8] {
    let mut out = [0; 8];
    for (value, channel) in out.iter_mut().zip(state.rc.values.iter()) {
        *value = i32::from(*channel);
    }
    out
}

/// `ch1out` to `ch8out`.
fn servo_out(state: &VehicleState) -> [i32; 8] {
    let mut out = [0; 8];
    for (value, channel) in out.iter_mut().zip(state.servo_outputs.iter()) {
        *value = i32::from(*channel);
    }
    out
}

impl MissionPlanner {
    /// `BUT_RAWSensor_Click`: the form made and shown, or "Please connect first" on the status
    /// line (the C#'s box, under the owner's rule) with no window. `ThemeManager.ApplyThemeTo`
    /// is the palette's business here. `// C#: GCSViews/FlightData.cs:1464-1469`
    pub(crate) fn raw_sensor_open(&mut self) {
        let view = self.telemetry.view();
        match self.raw_sensor.open(view.connected) {
            Ok(()) => self.raw_sensor_request_stream(&view),
            Err(text) => self.file_status = Some(text.to_owned()),
        }
    }

    /// `MainV2.comPort.requestDatastream(MAV_DATA_STREAM.RAW_SENSORS, cs.ratesensors)`.
    /// `// C#: Controls/RAW_Sensor.cs:257`
    fn raw_sensor_request_stream(&self, view: &TelemetryView) {
        let hz = view.state.as_deref().map_or_else(
            || mp_vehicle::StreamRates::backups().sensors,
            |state| state.rates.sensors,
        );
        self.telemetry.request_raw_sensors(hz);
    }

    /// A tab clicked: the page, and the stream asked for again.
    /// `// C#: Controls/RAW_Sensor.cs:239-267`
    pub(crate) fn raw_sensor_select(&mut self, tab: Tab) {
        self.raw_sensor.select(tab);
        let view = self.telemetry.view();
        self.raw_sensor_request_stream(&view);
    }

    /// `CMB_rawupdaterate_SelectedIndexChanged`: `cs.ratesensors = int.Parse(text)` on the
    /// shown vehicle, and the RAW_SENSORS stream asked for at it.
    /// `// C#: Controls/RAW_Sensor.cs:269-274`
    pub(crate) fn raw_sensor_set_rate(&mut self, text: &'static str) {
        let Some(hz) = self.raw_sensor.set_rate(text) else {
            return;
        };
        let view = self.telemetry.view();
        let mut rates = view
            .state
            .as_deref()
            .map_or_else(mp_vehicle::StreamRates::backups, |state| state.rates);
        rates.sensors = hz;
        self.telemetry.set_stream_rates(rates);
        self.telemetry.request_raw_sensors(hz);
    }

    /// `BUT_savecsv_Click`: the `SaveFileDialog`, asked as the flight screen's file dialogs are.
    /// `// C#: Controls/RAW_Sensor.cs:278-298`
    pub(crate) fn raw_sensor_save_csv(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.fly_actions.ask(crate::fly::Prompt::RawSensorCsv, "");
        self.fly_focus.prompt.focus(window, cx);
    }

    /// The dialog answered with a name.
    pub(crate) fn raw_sensor_csv_named(&mut self, text: &str) {
        if let Err(why) = self.raw_sensor.save_csv_to(text) {
            self.file_status = Some(why);
        }
    }

    /// The form's 10 ms timer, on the frame.
    pub(crate) fn raw_sensor_tick(&mut self, view: &TelemetryView) {
        if let Some(why) = self.raw_sensor.tick(view) {
            self.file_status = Some(why);
        }
    }
}

/// A box at the Designer's place and size.
fn at(x: f32, y: f32, width: f32, height: f32) -> gpui::Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// The window, while the form is showing: the tab strip over the selected page.
pub fn window(
    raw: &RawSensor,
    view: &TelemetryView,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !raw.is_open() {
        return None;
    }
    let default = VehicleState::default();
    let state = view.state.as_deref().unwrap_or(&default);
    let mut tabs = div().flex().gap_1();
    for tab in [Tab::RawSensor, Tab::Radio] {
        let selected = tab == raw.tab();
        tabs = tabs.child(
            crate::probe::measured(tab.id(), div())
                .id(tab.id())
                .px_2()
                .py_1()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .border_1()
                .border_color(rgb(if selected {
                    theme::ACCENT
                } else {
                    theme::BORDER
                }))
                .bg(rgb(if selected { theme::BG } else { theme::ACTION }))
                .cursor_pointer()
                .child(tab.text())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.raw_sensor_select(tab);
                    cx.notify();
                })),
        );
    }
    let page = match raw.tab() {
        Tab::RawSensor => raw_sensor_page(raw, state, cx),
        Tab::Radio => radio_page(state),
    };
    let body = div().flex().flex_col().gap_1().child(tabs).child(page);
    Some(crate::fly::floating_window(
        "raw-sensor",
        TITLE,
        "raw-sensor-close",
        AT,
        body,
        cx.listener(|this, _event: &(), _window, cx| {
            this.raw_sensor.close();
            cx.notify();
        }),
    ))
}

/// An empty tab page, `751 x 478`.
fn page_box() -> gpui::Div {
    div()
        .relative()
        .w(px(PAGE.0))
        .h(px(PAGE.1))
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
}

/// `tabRawSensor`: the three dials, the six boxes, Save CSV, the note, the rate combo and the
/// chart, each at the Designer's place. `// C#: Controls/RAW_Sensor.resx`
fn raw_sensor_page(
    raw: &RawSensor,
    state: &VehicleState,
    cx: &mut Context<MissionPlanner>,
) -> gpui::Div {
    let (roll, pitch, yaw) = attitude_degrees(state);
    let mut page = page_box();
    // Groll at (94, 9), Gpitch at (270, 9), aGauge1 at (446, 9), each 170 square, bound to
    // `roll`, `pitch` and `yaw`.
    for (id, x, dial, value) in [
        ("raw-gauge-roll", 94.0, crate::gauge::roll_dial(), roll),
        ("raw-gauge-pitch", 270.0, crate::gauge::pitch_dial(), pitch),
        ("raw-gauge-yaw", 446.0, crate::gauge::yaw_dial(), yaw),
    ] {
        page = page.child(dial_element(id, x, dial, value));
    }
    // `chkax` to `chkgz`, down the right.
    for (index, id) in CHECKS.iter().enumerate() {
        #[allow(clippy::cast_precision_loss)] // six boxes
        let y = 16.0 + 23.0 * index as f32;
        let checked = raw.checked.get(index).copied().unwrap_or(false);
        let text = TRACES.get(index).map_or("", |(name, _)| name);
        page = page.child(at(685.0, y, 63.0, 17.0).child(check_box(
            id,
            text,
            checked,
            cx.listener(move |this, _event, _window, cx| {
                this.raw_sensor.toggle(index);
                cx.notify();
            }),
        )));
    }
    // `BUT_savecsv`.
    page = page.child(at(6.0, 68.0, 84.0, 24.0).child(action(
        "raw-savecsv",
        "Save CSV",
        theme::ACCENT,
        true,
        cx.listener(|this, _event: &(), window, cx| {
            this.raw_sensor_save_csv(window, cx);
            cx.notify();
        }),
    )));
    // `label3`.
    page = page.child(
        at(3.0, 143.0, 117.0, 39.0)
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child(NOTE),
    );
    // `CMB_rawupdaterate`, and its list when it is down.
    page = page.child(
        crate::probe::measured("raw-rate", at(651.0, 154.0, 94.0, 21.0))
            .id("raw-rate")
            .flex()
            .items_center()
            .gap_1()
            .px_1()
            .border_1()
            .border_color(rgb(if raw.rate_open {
                theme::ACCENT
            } else {
                theme::BORDER
            }))
            .bg(rgb(theme::ACTION))
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .cursor_pointer()
            .child(div().flex_1().truncate().child(raw.rate()))
            .child(div().text_color(rgb(theme::DIM)).child("\u{25be}"))
            .on_click(cx.listener(|this, _event, _window, cx| {
                this.raw_sensor.toggle_rate_list();
                cx.notify();
            })),
    );
    if raw.rate_open {
        let mut list = at(651.0, 176.0, 94.0, 66.0)
            .flex()
            .flex_col()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::BORDER));
        for text in RATES {
            let id = format!("raw-rate-{text}");
            list = list.child(
                crate::probe::measured(id.clone(), div())
                    .id(SharedString::from(id))
                    .px_1()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .cursor_pointer()
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(text)
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.raw_sensor_set_rate(text);
                        cx.notify();
                    })),
            );
        }
        page = page.child(list);
    }
    page.child(chart(raw))
}

/// One dial, `170` square, drawn by the scene painter with its one needle at `value`.
fn dial_element(id: &'static str, x: f32, dial: Dial, value: f32) -> AnyElement {
    let needle = dial.raw_needle(value);
    crate::probe::measured(id, at(x, 9.0, 170.0, 170.0))
        .id(id)
        .child(
            gpui::canvas(
                |_bounds, _window, _cx| (),
                move |bounds, (), window, cx| {
                    let scene = dial.scene(170.0, &[needle]);
                    crate::hud::paint(&scene, bounds, window, cx);
                },
            )
            .size_full(),
        )
        .into_any_element()
}

/// A check box: a square that fills when checked, and its text.
fn check_box(
    id: &'static str,
    text: &'static str,
    checked: bool,
    on_click: impl Fn(&gpui::ClickEvent, &mut Window, &mut gpui::App) + 'static,
) -> AnyElement {
    let square = div()
        .size(px(13.0))
        .flex_shrink_0()
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .bg(rgb(theme::BG))
        .children(checked.then(|| div().size(px(7.0)).bg(rgb(theme::ACCENT))));
    crate::probe::measured(id, div())
        .id(id)
        .flex()
        .items_center()
        .gap_1()
        .text_xs()
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .child(square)
        .child(text)
        .on_click(on_click)
        .into_any_element()
}

/// `zg1` at (6, 185), 742 by 290: "Raw Sensors" over the legend of six, the plot of the last ten
/// seconds with "Raw Data" and its range in red down the left (`YAxis.Scale.FontSpec.FontColor
/// = Red`), "Time" and the seconds shown along the bottom. ZedGraph's X axis runs to one major
/// step past the newest point; the plot's ten seconds end at it here. The black curve is drawn
/// in the text's colour, which the dark plot would otherwise swallow.
/// `// C#: Controls/RAW_Sensor.cs:38-90, 112-133`
fn chart(raw: &RawSensor) -> AnyElement {
    let latest = raw
        .series
        .iter()
        .filter_map(Series::latest_time)
        .fold(0.0_f64, f64::max);
    let (from, to) = mp_chart::window(latest);
    let borrowed: Vec<&Series> = raw.series.iter().collect();
    let range = auto_range(&borrowed, from, to);
    let mut legend = div().flex().flex_wrap().gap_3().justify_center();
    for (name, colour) in TRACES {
        legend = legend.child(
            div()
                .flex()
                .items_center()
                .gap_1()
                .child(div().size_2().rounded_full().bg(rgb(trace_colour(colour))))
                .child(div().text_xs().text_color(rgb(theme::TEXT)).child(name)),
        );
    }
    let mut plot = div()
        .relative()
        .flex_1()
        .min_w(px(0.0))
        .border_1()
        .border_color(rgb(theme::DIM));
    let (top, bottom) = range.map_or_else(
        || (String::new(), String::new()),
        |range| (format!("{:.0}", range.high), format!("{:.0}", range.low)),
    );
    if let Some(range) = range {
        let lines = raw
            .series
            .iter()
            .zip(TRACES)
            .map(|(series, (_, colour))| crate::plotline::Line {
                points: crate::plotline::curve(series, range, from, to, COLUMNS),
                colour: rgb(trace_colour(colour)).into(),
                diamonds: false,
            })
            .collect();
        plot = plot.child(crate::plotline::element(lines));
    }
    crate::probe::measured("raw-chart", at(6.0, 185.0, 742.0, 290.0))
        .id("raw-chart")
        .flex()
        .flex_col()
        .gap_1()
        .p_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .child(
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .flex()
                .justify_center()
                .child("Raw Sensors"),
        )
        .child(legend)
        .child(
            div()
                .flex()
                .flex_1()
                .min_h(px(0.0))
                .gap_1()
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .justify_between()
                        .w(px(52.0))
                        .text_xs()
                        .text_color(rgb(RED))
                        .child(top)
                        .child("Raw Data")
                        .child(bottom),
                )
                .child(plot),
        )
        .child(
            div()
                .flex()
                .justify_between()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(format!("{from:.0}"))
                .child("Time")
                .child(format!("{to:.0}")),
        )
        .into_any_element()
}

/// A curve's colour on the dark plot: the C#'s, but black as the text's.
const fn trace_colour(colour: u32) -> u32 {
    if colour == 0 { theme::TEXT } else { colour }
}

/// `tabRadio`: "Radio IN" over eight bars of `ch1in` to `ch8in` at x = 142, "Servo/Motor OUT"
/// over eight of `ch1out` to `ch8out` at x = 424, the bars 55 apart from y = 30, each labelled
/// "Radio N". `// C#: Controls/RAW_Sensor.Designer.cs:105-361, Controls/RAW_Sensor.resx`
fn radio_page(state: &VehicleState) -> gpui::Div {
    let mut page = page_box();
    for (text, x) in [("Radio IN", 211.0), ("Servo/Motor OUT", 469.0)] {
        page = page.child(
            at(x, 14.0, 120.0, 13.0)
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(text),
        );
    }
    let inputs = rc_in(state);
    let outputs = servo_out(state);
    for (index, (input, output)) in inputs.iter().zip(outputs).enumerate() {
        #[allow(clippy::cast_precision_loss)] // eight bars
        let y = 30.0 + 55.0 * index as f32;
        let channel = index + 1;
        page = page
            .child(bar(
                format!("raw-bar-ch{channel}in"),
                142.0,
                y,
                &format!("Radio {channel}"),
                *input,
            ))
            .child(bar(
                format!("raw-bar-ch{channel}out"),
                424.0,
                y,
                &format!("Radio {channel}"),
                output,
            ));
    }
    page
}

/// One `HorizontalProgressBar`, 170 by 25: filled from the left by the value held to one above
/// the minimum and the maximum (`Value`'s setter), its `Label` centred 2 below it and the value
/// as set 15 below. `// C#: ExtLibs/Controls/HorizontalProgressBar.cs:74-115, 160-175`
fn bar(id: String, x: f32, y: f32, label: &str, value: i32) -> AnyElement {
    let ans = if value <= BAR_MIN {
        BAR_MIN + 1
    } else if value >= BAR_MAX {
        BAR_MAX
    } else {
        value
    };
    #[allow(clippy::cast_precision_loss)] // pulse widths
    let fraction = (ans - BAR_MIN) as f32 / (BAR_MAX - BAR_MIN) as f32;
    let caption = |top: f32, text: String| {
        div()
            .absolute()
            .left_0()
            .top(px(top))
            .w(px(170.0))
            .h(px(13.0))
            .flex()
            .justify_center()
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .child(text)
    };
    crate::probe::measured(id.clone(), at(x, y, 170.0, 53.0))
        .id(SharedString::from(id))
        .child(
            div()
                .absolute()
                .left_0()
                .top_0()
                .w(px(170.0))
                .h(px(25.0))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .bg(rgb(theme::PANEL))
                .child(
                    div()
                        .absolute()
                        .left_0()
                        .top_0()
                        .h_full()
                        .w(gpui::relative(fraction))
                        .bg(rgb(theme::OK)),
                ),
        )
        .child(caption(27.0, label.to_owned()))
        .child(caption(40.0, value.to_string()))
        .into_any_element()
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;

    fn view(connected: bool, state: Option<VehicleState>) -> TelemetryView {
        let mut view = TelemetryView::disconnected("");
        view.connected = connected;
        view.state = state.map(Arc::new);
        view
    }

    /// `new RAW_Sensor().Show()` without a link: "Please connect first" and no form; with one,
    /// the form on its Raw Sensor page with every box checked and the combo's prompt.
    #[test]
    fn the_form_opens_only_with_a_link() {
        let mut raw = RawSensor::new();
        assert_eq!(raw.open(false), Err(PLEASE_CONNECT));
        assert!(!raw.is_open());
        assert_eq!(raw.open(true), Ok(()));
        assert!(raw.is_open());
        assert_eq!(raw.tab(), Tab::RawSensor);
        assert_eq!(raw.checked, [true; 6]);
        assert_eq!(raw.rate(), "Update Speed");
        assert_eq!(raw.series.len(), 6);
        assert_eq!(raw.series[0].name, "Accel X");
    }

    /// `timer2serial_Tick`: a sample into every checked curve; an unchecked curve is emptied;
    /// nothing is sampled without a link.
    #[test]
    fn the_sample_fills_the_checked_curves_and_empties_the_rest() {
        let mut raw = RawSensor::new();
        raw.open(true).unwrap();
        let mut state = VehicleState::default();
        state.imu[0].accel = [1.0, 2.0, 3.0];
        state.imu[0].gyro = [4.0, 5.0, 6.0];
        assert_eq!(raw.tick(&view(true, Some(state))), None);
        assert_eq!(raw.latest, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert!(raw.series.iter().all(|series| series.len() == 1));
        // Too soon for another.
        assert_eq!(raw.tick(&view(true, Some(state))), None);
        assert!(raw.series.iter().all(|series| series.len() == 1));
        raw.toggle(0);
        raw.last_sample = None;
        raw.tick(&view(true, Some(state)));
        assert!(raw.series[0].is_empty());
        assert_eq!(raw.series[1].len(), 2);
        raw.last_sample = None;
        raw.tick(&view(false, Some(state)));
        assert_eq!(raw.series[1].len(), 2, "no link, no sample");
    }

    /// `BUT_savecsv_Click`: `.csv` added to a bare name, then a line a sample of the date, then
    /// `ax` to `gz`; an empty name is the dialog cancelled.
    #[test]
    fn save_csv_names_the_file_and_the_samples_fill_it() {
        let dir = std::env::temp_dir().join(format!("mp-raw-sensor-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut raw = RawSensor::new();
        raw.open(true).unwrap();
        raw.save_csv_to("").unwrap();
        assert!(raw.csv_path.is_none());
        let named = dir.join("samples");
        raw.save_csv_to(named.to_str().unwrap()).unwrap();
        assert_eq!(
            raw.csv_path.as_deref(),
            Some(dir.join("samples.csv").as_path())
        );
        let mut state = VehicleState::default();
        state.imu[0].accel = [1.5, -2.0, 3.0];
        state.imu[0].gyro = [0.25, 0.0, -6.0];
        raw.tick(&view(true, Some(state)));
        raw.close();
        let text = std::fs::read_to_string(dir.join("samples.csv")).unwrap();
        let line = text.lines().next().unwrap();
        let fields: Vec<&str> = line.split(',').collect();
        assert_eq!(fields.len(), 7);
        assert!(fields[0].contains('/') && fields[0].contains(':'), "{line}");
        assert_eq!(&fields[1..], ["1.5", "-2", "3", "0.25", "0", "-6"]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `CMB_rawupdaterate`: the text kept and parsed.
    #[test]
    fn the_rate_is_the_items_number() {
        let mut raw = RawSensor::new();
        raw.open(true).unwrap();
        assert_eq!(raw.set_rate("50"), Some(50));
        assert_eq!(raw.rate(), "50");
    }

    /// `HorizontalProgressBar.Value`: the bar held to one above the minimum and the maximum, the
    /// label reading the value as set.
    #[test]
    fn the_bars_hold_their_values_to_the_scale() {
        let mut state = VehicleState::default();
        state.rc.values[0] = 1500;
        state.rc.values[7] = 2500;
        state.servo_outputs[2] = 900;
        assert_eq!(rc_in(&state)[0], 1500);
        assert_eq!(rc_in(&state)[7], 2500);
        assert_eq!(servo_out(&state)[2], 900);
        let _ = bar("raw-bar-test".to_owned(), 0.0, 0.0, "Radio 1", 900);
    }
}
