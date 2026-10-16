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

//! The connection control's "Stats..." link and the "Link Stats" window it opens:
//! `Controls/ConnectionControl.cs`'s `linkLabel1`, `MainV2.ShowConnectionStatsForm` and
//! `Controls/ConnectionStats.cs`.
//!
//! "Stats..." sits under the port box, on the control's second row, shown only while connected
//! (`IsConnected`); its click raises `ShowLinkStats`, which shows a fixed dialog "Link Stats" holding
//! one `ConnectionStats`, or the one already shown (`ConnectionControl.cs:13-19, 37-44`;
//! `MainV2.cs:738, 1259-1283`). The control counts from its `Load`: a "Download" group - the bytes
//! received and their mean over the last three seconds, the packets, their mean, the packets lost,
//! the link's quality over the last three seconds, and the longest time between two packets - an
//! "Upload" group - the bytes sent and their mean over the last five seconds - the "Mavlink 2" and
//! "Signing" ticks, which set the shown vehicle's `MAV.mavlinkv2` and `MAV.signing`, and Reset,
//! which starts every count again (`ConnectionStats.cs:17-190`). The link's counters are
//! `mp_link::LinkStats` (`mp_link::packet_stats`); the counting from the window's start, the
//! per-second samples behind the means, and each box's text as the C# formats it are [`Stats`].
//! Disconnecting freezes the boxes and leaves the window open (`StopUpdates`, `MainV2.cs:1407-1418,
//! 2493-2497`); connecting again while it shows starts it afresh (`ResetConnectionStats`,
//! `MainV2.cs:1244-1257, 1549`).
//!
//! Divergences, each at its site: the counts are sampled each frame, the totals as they stood at
//! the last quarter second and the means at each whole second since the window's start, where the
//! C#'s Rx timers push them; the window's link is the one with a vehicle, as the C#'s connection is
//! made on its heartbeats; the dialog is drawn over the main window, where the C# opens a window
//! of its own.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::RefCell;
use std::collections::VecDeque;

use gpui::{div, point, prelude::*, px, rgb, AnyElement, Context, Div};
use web_time::{Duration, Instant};

use mp_link::{LinkSender, LinkStats};
use mp_vehicle::VehicleId;

use crate::ui::theme;
use crate::MissionPlanner;

/// `linkLabel1.Text`. `// C#: Controls/ConnectionControl.resx (linkLabel1.Text)`
pub const STATS: &str = "Stats...";

/// The dialog's title, `Strings.LinkStats`. `// C#: ExtLibs/Strings/Strings.resx (LinkStats)`
pub const TITLE: &str = "Link Stats";

/// The control the dialog holds, `$this.Size`; the dialog is made as wide
/// (`connectionStatsForm.Width = _connectionStats.Width`). `// C#: Controls/ConnectionStats.resx
/// ($this.Size); MainV2.cs:1264-1278`
pub const CLIENT_SIZE: (f32, f32) = (417.0, 136.0);

/// WinForms' 8.25 pt Microsoft Sans Serif is eleven pixels; the `.resx` sizes are laid out for it.
const FONT: f32 = 11.0;

/// Where the dialog is put over the window: `Show()` leaves the place to the system.
const WINDOW_AT: (f32, f32) = (48.0, 96.0);

/// The control's second row, under the boxes, which "Stats..." takes while connected: the C#'s
/// control is two rows high whether it shows or not (`linkLabel1` at 3, 32; `$this.Size` 238 x 47).
/// `// C#: Controls/ConnectionControl.resx (linkLabel1.Location, $this.Size)`
const LINK_ROW: f32 = 16.0;

/// `Sample(TimeSpan.FromMilliseconds(250))`, the totals' rate. `// C#: Controls/ConnectionStats.cs:64-69`
const QUARTER: Duration = Duration::from_millis(250);

/// The longest of the means' windows, five seconds: the bytes sent's.
/// `// C#: Controls/ConnectionStats.cs:112-116`
const SECONDS_KEPT: usize = 5;

// ---------------------------------------------------------------------------------------------
// The counting.
// ---------------------------------------------------------------------------------------------

/// The link's counters at one moment, as the window counts them: `BytesReceived`'s and
/// `BytesSent`'s sums, `WhenPacketReceived`'s count and `WhenPacketLost`'s sum.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counts {
    /// Bytes read from the link.
    pub bytes_received: u64,
    /// Bytes written to it.
    pub bytes_sent: u64,
    /// Packets received.
    pub packets: u64,
    /// Packets lost.
    pub lost: u64,
}

impl Counts {
    /// The link's counters now.
    #[must_use]
    pub const fn of(stats: &LinkStats) -> Self {
        Self {
            bytes_received: stats.bytes_read,
            bytes_sent: stats.bytes_written,
            packets: stats.packets_received,
            lost: stats.packets_lost,
        }
    }

    /// These less `base`: the counts since it.
    #[must_use]
    pub const fn since(self, base: Self) -> Self {
        Self {
            bytes_received: self.bytes_received.saturating_sub(base.bytes_received),
            bytes_sent: self.bytes_sent.saturating_sub(base.bytes_sent),
            packets: self.packets.saturating_sub(base.packets),
            lost: self.lost.saturating_sub(base.lost),
        }
    }

    /// These and `other`.
    #[must_use]
    pub const fn plus(self, other: Self) -> Self {
        Self {
            bytes_received: self.bytes_received.saturating_add(other.bytes_received),
            bytes_sent: self.bytes_sent.saturating_add(other.bytes_sent),
            packets: self.packets.saturating_add(other.packets),
            lost: self.lost.saturating_add(other.lost),
        }
    }
}

/// The nine boxes' text, empty while a box shows nothing.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Texts {
    /// `txt_BytesReceived`.
    pub bytes_received: String,
    /// `txt_BytesPerSecondRx`.
    pub bytes_per_second_rx: String,
    /// `txt_PacketsRx`.
    pub packets_rx: String,
    /// `txt_PacketsPerSecond`.
    pub packets_per_second: String,
    /// `txt_PacketsLost`.
    pub packets_lost: String,
    /// `txt_LinkQuality`.
    pub link_quality: String,
    /// `txt_MaxPacketInterval`.
    pub max_packet_interval: String,
    /// `txt_BytesSent`.
    pub bytes_sent: String,
    /// `txt_BytesPerSecondSent`.
    pub bytes_per_second_sent: String,
}

impl Texts {
    /// The text of the box `name` (its `.resx` name), empty for a name that is none of them.
    #[must_use]
    pub fn of(&self, name: &str) -> &str {
        match name {
            "txt_BytesReceived" => &self.bytes_received,
            "txt_BytesPerSecondRx" => &self.bytes_per_second_rx,
            "txt_PacketsRx" => &self.packets_rx,
            "txt_PacketsPerSecond" => &self.packets_per_second,
            "txt_PacketsLost" => &self.packets_lost,
            "txt_LinkQuality" => &self.link_quality,
            "txt_MaxPacketInterval" => &self.max_packet_interval,
            "txt_BytesSent" => &self.bytes_sent,
            "txt_BytesPerSecondSent" => &self.bytes_per_second_sent,
            _ => "",
        }
    }
}

/// `ConnectionStats_Load`'s subscriptions: everything counted from `started`, its boxes emptied
/// then. `// C#: Controls/ConnectionStats.cs:35-136`
#[derive(Debug, Clone)]
pub struct Stats {
    /// When the subscriptions were made: the window's `Load`, or its Reset.
    started: Instant,
    /// The link's counters then.
    base: Counts,
    /// The counts since `started` at the last quarter second taken: `Sample(250 ms)` of each
    /// total's `Scan`.
    quarter: Counts,
    /// Quarter seconds taken since `started`.
    quarters: u128,
    /// The counts since `started` at the last whole second taken.
    second: Counts,
    /// Whole seconds taken since `started`.
    seconds: u64,
    /// Each whole second's counts, oldest first, the last [`SECONDS_KEPT`]: `Buffer(1 s)`'s sums,
    /// which the three- and five-second windows read.
    per_second: VecDeque<Counts>,
    /// The longest interval between two packets since `started`, once one has come:
    /// `TimeInterval()`'s running `Math.Max`.
    max_interval: Option<Duration>,
}

impl Stats {
    /// The subscriptions made at `now`, the link's counters being `base`: every box empty.
    /// `// C#: Controls/ConnectionStats.cs:39-47`
    #[must_use]
    pub fn new(base: Counts, now: Instant) -> Self {
        Self {
            started: now,
            base,
            quarter: Counts::default(),
            quarters: 0,
            second: Counts::default(),
            seconds: 0,
            per_second: VecDeque::with_capacity(SECONDS_KEPT),
            max_interval: None,
        }
    }

    /// When the counting started.
    #[must_use]
    pub const fn started(&self) -> Instant {
        self.started
    }

    /// The link's counters `link` at `now`: the totals taken at each quarter second, each whole
    /// second's counts kept, and the link's longest interval once it counts from this start and a
    /// packet has come since.
    ///
    /// **Divergence:** sampled when the window is drawn, at least every quarter second while it
    /// counts, where the C#'s timers push each value at its moment; a whole second the window was
    /// not drawn in counts nothing and the second after it all, which leaves the means over the
    /// seconds that hold both as they were.
    pub fn sample(&mut self, link: &LinkStats, now: Instant) {
        let since = Counts::of(link).since(self.base);
        let elapsed = now.saturating_duration_since(self.started);
        let quarters = elapsed.as_millis() / QUARTER.as_millis();
        if quarters > self.quarters {
            self.quarters = quarters;
            self.quarter = since;
        }
        let seconds = elapsed.as_secs();
        if seconds > self.seconds {
            let missed = usize::try_from(seconds - self.seconds - 1)
                .unwrap_or(SECONDS_KEPT)
                .min(SECONDS_KEPT);
            for _ in 0..missed {
                self.push(Counts::default());
            }
            self.push(since.since(self.second));
            self.second = since;
            self.seconds = seconds;
        }
        // The first interval runs from the subscription: the link's from this start
        // (`LinkSender::restart_stats`), and only once a packet has come since it.
        if link.interval_since == Some(self.started) && link.max_packet_interval > Duration::ZERO {
            self.max_interval = Some(link.max_packet_interval);
        }
    }

    /// One whole second's counts kept, the oldest beyond [`SECONDS_KEPT`] let go.
    fn push(&mut self, counts: Counts) {
        if self.per_second.len() >= SECONDS_KEPT {
            self.per_second.pop_front();
        }
        self.per_second.push_back(counts);
    }

    /// The last `seconds` whole seconds' counts together, once that many have been taken.
    fn last(&self, seconds: usize) -> Option<Counts> {
        (self.per_second.len() >= seconds).then(|| {
            self.per_second
                .iter()
                .rev()
                .take(seconds)
                .fold(Counts::default(), |sum, counts| sum.plus(*counts))
        })
    }

    /// How long until the next quarter second, when the totals are taken again.
    #[must_use]
    pub fn next_due(&self, now: Instant) -> Duration {
        let into = now.saturating_duration_since(self.started).as_millis() % QUARTER.as_millis();
        QUARTER.saturating_sub(Duration::from_millis(u64::try_from(into).unwrap_or(0)))
            + Duration::from_millis(1)
    }

    /// Each box's text as the C#'s subscriptions write it, empty until they first do.
    /// `// C#: Controls/ConnectionStats.cs:62-132`
    #[must_use]
    pub fn texts(&self) -> Texts {
        let quarter = self.quarter;
        let total = |count: u64, text: fn(u64) -> String| {
            if count > 0 {
                text(count)
            } else {
                String::new()
            }
        };
        let three = self.last(3);
        Texts {
            // `BytesReceived.Scan(0, (x, y) => x + y).Sample(250 ms).Select(ToHumanReadableByteCount)`.
            bytes_received: total(quarter.bytes_received, human_bytes),
            // `bytesReceivedEverySecond.Buffer(3, 1).Select(xs => (int) xs.Average())`.
            bytes_per_second_rx: three
                .map(|sum| human_bytes(sum.bytes_received / 3))
                .unwrap_or_default(),
            // `packetsReceivedCount.Sample(250 ms)`.
            packets_rx: total(quarter.packets, |count: u64| count.to_string()),
            // `WhenPacketReceived.Buffer(3 s, 1 s).Select(xs => xs.Sum() / 3.0)`, "0".
            packets_per_second: three
                .map(|sum| whole(per_second_of(sum.packets), false).to_string())
                .unwrap_or_default(),
            // `packetsLostCount.Sample(250 ms)`.
            packets_lost: total(quarter.lost, |count: u64| count.to_string()),
            // `CalculateAverage` over `Buffer(3 s, 1 s)`, "00%".
            link_quality: three
                .map(|sum| quality(sum.packets, sum.lost))
                .unwrap_or_default(),
            // `TimeInterval()`'s running `Math.Max`, `TotalMilliseconds.ToString()`.
            max_packet_interval: self.max_interval.map(milliseconds).unwrap_or_default(),
            // `BytesSent.Scan(0, (x, y) => x + y).Sample(250 ms).Select(ToHumanReadableByteCount)`.
            bytes_sent: total(quarter.bytes_sent, human_bytes),
            // `bytesSentEverySecond.Buffer(5 s, 1 s).Select(xs => xs.Any() ? xs.Average() : 0)`.
            bytes_per_second_sent: self
                .last(SECONDS_KEPT)
                .map(|sum| human_bytes(sum.bytes_sent / 5))
                .unwrap_or_default(),
        }
    }
}

/// Three seconds' packets a second, `xs.Sum() / 3.0`.
fn per_second_of(packets: u64) -> f64 {
    #[allow(clippy::cast_precision_loss)] // a count of packets
    let packets = packets as f64;
    packets / 3.0
}

/// `ToHumanReadableByteCount`, the logs page's port of it, of a count. **Divergence:** the C#
/// counts in an `int`, which goes negative past 2 GiB; here the count past 4 GiB shows as 4 GiB.
/// `// C#: Controls/ConnectionStats.cs:165-174`
fn human_bytes(count: u64) -> String {
    crate::logdownload::human_bytes(u32::try_from(count).unwrap_or(u32::MAX))
}

/// `received / (received + (double) lost)` written `"00%"`: "NaN" with neither.
/// `// C#: Controls/ConnectionStats.cs:157-163, 88`
#[must_use]
pub fn quality(received: u64, lost: u64) -> String {
    let all = received.saturating_add(lost);
    if all == 0 {
        // `NumberFormatInfo.NaNSymbol`.
        return "NaN".to_owned();
    }
    #[allow(clippy::cast_precision_loss)] // counts of packets
    let ratio = received as f64 / all as f64;
    format!("{:02}%", whole(ratio, true))
}

/// A non-negative `value` written `"0"`, or `"00%"` when `percent`, as .NET Framework writes it:
/// its fifteen significant digits, a hundred times for a percentage, rounded half away from zero
/// to a whole number from those digits - 0.955 is "96%", where the double's own value is a hair
/// under 0.955. `// C#: clr/src/classlibnative/bcltype/number.cpp NumberToStringFormat`
#[must_use]
pub fn whole(value: f64, percent: bool) -> u64 {
    let (_, digits, scale) = mp_log::netfmt::general_digits(value, 15);
    let scale = scale + if percent { 2 } else { 0 };
    let digit = |index: i32| {
        usize::try_from(index)
            .ok()
            .and_then(|index| digits.get(index))
            .map_or(0, |digit| u64::from(*digit))
    };
    let mut whole = 0u64;
    for index in 0..scale {
        whole = whole.saturating_mul(10).saturating_add(digit(index));
    }
    if digit(scale) >= 5 {
        whole += 1;
    }
    whole
}

/// `TimeSpan.TotalMilliseconds.ToString()`: the interval's ticks of 100 ns times 1/10000, written
/// as a double is. `// C#: Controls/ConnectionStats.cs:127-132`
#[must_use]
pub fn milliseconds(interval: Duration) -> String {
    #[allow(clippy::cast_precision_loss)] // ticks, as the C#'s double makes of them
    let ticks = (interval.as_nanos() / 100) as f64;
    mp_log::netfmt::double(ticks * (1.0 / 10_000.0))
}

// ---------------------------------------------------------------------------------------------
// The window.
// ---------------------------------------------------------------------------------------------

/// `connectionStatsForm` and its `ConnectionStats`, while shown.
#[derive(Debug)]
pub struct Form {
    /// The counting, from the last `Load` or Reset.
    stats: Stats,
    /// The link it counts, `_mavlink`; `None` before the link has a vehicle, and once it has gone
    /// (`StopUpdates`), the boxes then as they last were.
    link: Option<LinkSender>,
    /// `chk_mavlink2.Checked`.
    mavlink2: bool,
    /// `chk_signing.Checked`.
    signing: bool,
    /// Resets clicked.
    resets: u32,
}

impl Form {
    /// The form made at `now`, before it has a link: every box empty, both ticks clear.
    #[must_use]
    pub fn new(now: Instant) -> Self {
        Self {
            stats: Stats::new(Counts::default(), now),
            link: None,
            mavlink2: false,
            signing: false,
            resets: 0,
        }
    }

    /// `new ConnectionStats(comPort)` on `sender`'s link: the ticks as the shown vehicle `id` has
    /// them, and `Load`'s counting from `now`. `// C#: Controls/ConnectionStats.cs:17-28; MainV2.cs:1253, 1276`
    pub fn attach(&mut self, sender: LinkSender, id: VehicleId, now: Instant) {
        self.mavlink2 = sender.mavlink2(id).unwrap_or(false);
        self.signing = sender.signing(id).is_some_and(|state| state.signing);
        self.start(&sender, now);
        self.link = Some(sender);
    }

    /// `ConnectionStats_Load`: the boxes emptied and every count started at `now`, the link's
    /// longest interval with them. `// C#: Controls/ConnectionStats.cs:35-136`
    fn start(&mut self, sender: &LinkSender, now: Instant) {
        sender.restart_stats(now);
        self.stats = Stats::new(Counts::of(&sender.stats()), now);
    }

    /// `but_reset_Click`: `StopUpdates` and `Load` again. Its link gone, the boxes are emptied and
    /// stay so, as the C#'s subscriptions to a closed port do.
    /// `// C#: Controls/ConnectionStats.cs:186-190`
    pub fn reset(&mut self, now: Instant) {
        self.resets = self.resets.saturating_add(1);
        match self.link.clone() {
            Some(sender) => self.start(&sender, now),
            None => self.stats = Stats::new(Counts::default(), now),
        }
    }

    /// One frame at `now`: `handle` the link with its shown vehicle, if it has one, `open` whether
    /// the link is open. The same link is sampled; another - a connection made since - starts the
    /// form afresh, as `ResetConnectionStats` makes it again; a closed one stops it, the boxes as
    /// they were, as `StopUpdates` does. `// C#: MainV2.cs:1244-1257, 1407-1418, 1549, 2493-2497`
    pub fn step(&mut self, handle: Option<(LinkSender, VehicleId)>, open: bool, now: Instant) {
        match handle {
            Some((sender, id)) => {
                if self
                    .link
                    .as_ref()
                    .is_some_and(|link| link.same_link(&sender))
                {
                    self.stats.sample(&sender.stats(), now);
                } else {
                    self.attach(sender, id, now);
                }
            }
            None if !open => self.link = None,
            None => {}
        }
    }

    /// Whether it is counting: it has a link.
    #[must_use]
    pub const fn updating(&self) -> bool {
        self.link.is_some()
    }

    /// The boxes' text.
    #[must_use]
    pub fn texts(&self) -> Texts {
        self.stats.texts()
    }
}

/// The window's state in the main window: the form while it is shown. Drawn from `&self`, so the
/// frame's sampling goes through a cell.
#[derive(Debug, Default)]
pub struct LinkStatsUi {
    form: RefCell<Option<Form>>,
}

impl MissionPlanner {
    /// `ShowConnectionStatsForm`: the form made and shown if it is not, counting from now on the
    /// link if it has a vehicle, or from when it has one; `Show()` of the one shown does nothing.
    /// `// C#: MainV2.cs:1259-1283`
    pub(crate) fn show_link_stats(&mut self) {
        let handle = self.telemetry.send_handle();
        let form = self.link_stats.form.get_mut();
        if form.is_some() {
            return;
        }
        let now = Instant::now();
        let mut shown = Form::new(now);
        if let Some((sender, id)) = handle {
            shown.attach(sender, id, now);
        }
        *form = Some(shown);
    }

    /// The form's close box: closed and disposed, `StopUpdates` with it; the next "Stats..." makes
    /// a new one. `// C#: Controls/ConnectionStats.cs:27; MainV2.cs:1261`
    fn close_link_stats(&mut self) {
        *self.link_stats.form.get_mut() = None;
    }

    /// Reset. `// C#: Controls/ConnectionStats.cs:186-190`
    fn reset_link_stats(&mut self) {
        if let Some(form) = self.link_stats.form.get_mut().as_mut() {
            form.reset(Instant::now());
        }
    }

    /// A tick clicked - "Mavlink 2", or "Signing" when `signing` - and the shown vehicle's state set
    /// to it, `_mavlink.MAV.mavlinkv2` or `.signing`; with no link, the tick alone.
    /// `// C#: Controls/ConnectionStats.cs:176-184`
    fn tick_link_stats(&mut self, signing: bool) {
        let handle = self.telemetry.send_handle();
        let Some(form) = self.link_stats.form.get_mut().as_mut() else {
            return;
        };
        if signing {
            form.signing = !form.signing;
        } else {
            form.mavlink2 = !form.mavlink2;
        }
        if let Some((sender, id)) = handle {
            if signing {
                sender.set_signing(id, form.signing);
            } else {
                sender.set_mavlink2(id, form.mavlink2);
            }
        }
    }
}

/// "Stats...", on the connection control's second row under the port box while `connected`, and
/// the row empty otherwise (`IsConnected`, `linkLabel1.Visible = isConnected`): drawn as this
/// application draws a link, the colour of one and underlined. Its click shows the window.
/// `// C#: Controls/ConnectionControl.cs:13-19, 37-44; Controls/ConnectionControl.resx (linkLabel1)`
pub fn stats_link(connected: bool, cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::facts::record("linkstats.link", if connected { "shown" } else { "hidden" });
    let row = div().flex().h(px(LINK_ROW)).pl(px(3.0));
    if !connected {
        return row.into_any_element();
    }
    row.child(
        crate::probe::measured("main-stats", div())
            .id("main-stats")
            .text_xs()
            .text_color(rgb(theme::LINK))
            .underline()
            .cursor_pointer()
            .hover(|style| style.text_color(rgb(theme::LINK_HOVER)))
            .child(STATS)
            .on_click(cx.listener(|this, _event: &gpui::ClickEvent, _window, cx| {
                this.show_link_stats();
                cx.notify();
            })),
    )
    .into_any_element()
}

/// Which box a control sits in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Parent {
    /// The control itself.
    Control,
    /// `groupBox1`, "Download".
    Download,
    /// `groupBox2`, "Upload".
    Upload,
}

/// A control of the `.resx`: its name, its parent, its place in that parent and its text.
type Placed = (&'static str, Parent, (f32, f32, f32, f32), &'static str);

/// The group boxes. `// C#: Controls/ConnectionStats.resx`
const GROUPS: [Placed; 2] = [
    (
        "groupBox1",
        Parent::Control,
        (4.0, 4.0, 245.0, 124.0),
        "Download",
    ),
    (
        "groupBox2",
        Parent::Control,
        (255.0, 4.0, 144.0, 83.0),
        "Upload",
    ),
];

/// The labels. `// C#: Controls/ConnectionStats.resx`
const LABELS: [Placed; 9] = [
    (
        "label1",
        Parent::Download,
        (10.0, 18.0, 33.0, 13.0),
        "Bytes",
    ),
    (
        "label2",
        Parent::Download,
        (135.0, 18.0, 43.0, 13.0),
        "Bytes/s",
    ),
    (
        "label3",
        Parent::Download,
        (6.0, 44.0, 46.0, 13.0),
        "Packets",
    ),
    (
        "label8",
        Parent::Download,
        (126.0, 43.0, 56.0, 13.0),
        "Packets/s",
    ),
    (
        "label4",
        Parent::Download,
        (3.0, 70.0, 51.0, 13.0),
        "Pkts Lost",
    ),
    (
        "label5",
        Parent::Download,
        (139.0, 68.0, 39.0, 13.0),
        "Quality",
    ),
    (
        "label10",
        Parent::Download,
        (13.0, 96.0, 156.0, 13.0),
        "Max time between packets (ms)",
    ),
    (
        "label6",
        Parent::Upload,
        (6.0, 18.0, 60.0, 13.0),
        "Total Bytes",
    ),
    (
        "label7",
        Parent::Upload,
        (23.0, 43.0, 43.0, 13.0),
        "Bytes/s",
    ),
];

/// The read-only boxes. `// C#: Controls/ConnectionStats.resx`
const BOXES: [Placed; 9] = [
    (
        "txt_BytesReceived",
        Parent::Download,
        (58.0, 16.0, 64.0, 20.0),
        "",
    ),
    (
        "txt_BytesPerSecondRx",
        Parent::Download,
        (184.0, 16.0, 51.0, 20.0),
        "",
    ),
    (
        "txt_PacketsRx",
        Parent::Download,
        (58.0, 42.0, 64.0, 20.0),
        "",
    ),
    (
        "txt_PacketsPerSecond",
        Parent::Download,
        (184.0, 42.0, 51.0, 20.0),
        "",
    ),
    (
        "txt_PacketsLost",
        Parent::Download,
        (58.0, 68.0, 64.0, 20.0),
        "",
    ),
    (
        "txt_LinkQuality",
        Parent::Download,
        (184.0, 68.0, 51.0, 20.0),
        "",
    ),
    (
        "txt_MaxPacketInterval",
        Parent::Download,
        (184.0, 94.0, 51.0, 20.0),
        "",
    ),
    (
        "txt_BytesSent",
        Parent::Upload,
        (74.0, 16.0, 64.0, 20.0),
        "",
    ),
    (
        "txt_BytesPerSecondSent",
        Parent::Upload,
        (74.0, 42.0, 64.0, 20.0),
        "",
    ),
];

/// "Mavlink 2". `// C#: Controls/ConnectionStats.resx (chk_mavlink2)`
const CHK_MAVLINK2: Placed = (
    "chk_mavlink2",
    Parent::Control,
    (259.0, 90.0, 72.0, 17.0),
    "Mavlink 2",
);
/// "Signing". `// C#: Controls/ConnectionStats.resx (chk_signing)`
const CHK_SIGNING: Placed = (
    "chk_signing",
    Parent::Control,
    (258.0, 113.0, 61.0, 17.0),
    "Signing",
);
/// Reset. `// C#: Controls/ConnectionStats.resx (but_reset)`
const BUT_RESET: Placed = (
    "but_reset",
    Parent::Control,
    (356.0, 86.0, 43.0, 23.0),
    "Reset",
);

/// A control's box at its place in the control's client area: its parent's place added to its own.
fn placed(control: &Placed) -> Div {
    let (_, parent, (x, y, w, h), _) = *control;
    let (dx, dy) = match parent {
        Parent::Control => (0.0, 0.0),
        Parent::Download => (GROUPS[0].2 .0, GROUPS[0].2 .1),
        Parent::Upload => (GROUPS[1].2 .0, GROUPS[1].2 .1),
    };
    div()
        .absolute()
        .left(px(x + dx))
        .top(px(y + dy))
        .w(px(w))
        .h(px(h))
}

/// A `GroupBox`: a border with its caption on it.
fn group_box(group: &Placed) -> AnyElement {
    placed(group)
        .child(
            div()
                .absolute()
                .left_0()
                .right_0()
                .top(px(6.0))
                .bottom_0()
                .rounded_sm()
                .border_1()
                .border_color(rgb(theme::BORDER)),
        )
        .child(
            div()
                .absolute()
                .left(px(6.0))
                .top_0()
                .px_1()
                .bg(rgb(theme::PANEL))
                .text_size(px(FONT))
                .text_color(rgb(theme::TEXT))
                .whitespace_nowrap()
                .child(group.3),
        )
        .into_any_element()
}

/// A `Label`.
fn label(control: &Placed) -> AnyElement {
    placed(control)
        .whitespace_nowrap()
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .child(control.3)
        .into_any_element()
}

/// A read-only `TextBox` with a single border, and what it holds.
fn text_box(control: &Placed, text: &str) -> AnyElement {
    placed(control)
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .child(if text.is_empty() {
            " ".to_owned()
        } else {
            text.to_owned()
        })
        .into_any_element()
}

/// A `CheckBox`, `signing` saying which.
fn checkbox(
    control: &Placed,
    checked: bool,
    signing: bool,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let mark = div()
        .w(px(11.0))
        .h(px(11.0))
        .flex_shrink_0()
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .border_color(rgb(theme::TEXT))
        .bg(rgb(theme::BG))
        .text_size(px(9.0))
        .text_color(rgb(theme::ACCENT))
        .children(checked.then_some("\u{2713}"));
    let name = control.0;
    crate::probe::measured(format!("linkstats-{name}"), placed(control))
        .id(gpui::SharedString::from(format!("linkstats-{name}")))
        .flex()
        .items_center()
        .gap_1()
        .whitespace_nowrap()
        .text_size(px(FONT))
        .text_color(rgb(theme::TEXT))
        .cursor_pointer()
        .child(mark)
        .child(control.3)
        .on_click(
            cx.listener(move |this, _event: &gpui::ClickEvent, _window, cx| {
                this.tick_link_stats(signing);
                cx.notify();
            }),
        )
        .into_any_element()
}

/// The `MyButton` Reset.
fn reset_button(cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured("linkstats-but_reset", placed(&BUT_RESET))
        .id("linkstats-but_reset")
        .flex()
        .items_center()
        .justify_center()
        .border_1()
        .rounded_sm()
        .border_color(rgb(theme::ACCENT))
        .bg(rgb(theme::ACTION))
        .text_size(px(FONT))
        .text_color(rgb(theme::ACCENT))
        .cursor_pointer()
        .hover(|style| style.bg(rgb(theme::BORDER)))
        .child(BUT_RESET.3)
        .on_click(cx.listener(|this, _event: &gpui::ClickEvent, _window, cx| {
            this.reset_link_stats();
            cx.notify();
        }))
        .into_any_element()
}

/// The facts a script reads: whether the window shows and counts, each box's text ("none" while
/// it is empty), the ticks, the resets and how long it has counted.
fn record_facts(form: Option<&Form>, texts: &Texts, now: Instant) {
    use crate::facts::record;
    record("linkstats.window", form.is_some());
    let Some(form) = form else {
        record("linkstats.updating", false);
        return;
    };
    record("linkstats.updating", form.updating());
    for control in &BOXES {
        let text = texts.of(control.0);
        record(
            format!("linkstats.{}", control.0),
            if text.is_empty() { "none" } else { text },
        );
    }
    record("linkstats.chk_mavlink2", form.mavlink2);
    record("linkstats.chk_signing", form.signing);
    record("linkstats.resets", form.resets);
    record(
        "linkstats.since_ms",
        now.saturating_duration_since(form.stats.started())
            .as_millis(),
    );
}

/// The window, while shown: sampled for this frame, then drawn over the main window - the title
/// and its close box above the control at its `.resx` size and places.
/// `// C#: MainV2.cs:1264-1282; Controls/ConnectionStats.resx`
pub fn window(this: &MissionPlanner, cx: &mut Context<MissionPlanner>) -> Option<AnyElement> {
    let now = Instant::now();
    let Ok(mut held) = this.link_stats.form.try_borrow_mut() else {
        return None;
    };
    let Some(form) = held.as_mut() else {
        record_facts(None, &Texts::default(), now);
        return None;
    };
    let open = this.telemetry.is_open();
    form.step(
        if open {
            this.telemetry.send_handle()
        } else {
            None
        },
        open,
        now,
    );
    let texts = form.texts();
    record_facts(Some(&*form), &texts, now);
    if form.updating() {
        crate::repaint::again_in(form.stats.next_due(now));
    }
    let (mavlink2, signing) = (form.mavlink2, form.signing);
    drop(held);

    let mut client = div()
        .relative()
        .w(px(CLIENT_SIZE.0))
        .h(px(CLIENT_SIZE.1))
        .bg(rgb(theme::PANEL));
    for group in &GROUPS {
        client = client.child(group_box(group));
    }
    for control in &LABELS {
        client = client.child(label(control));
    }
    for control in &BOXES {
        client = client.child(text_box(control, texts.of(control.0)));
    }
    client = client
        .child(checkbox(&CHK_MAVLINK2, mavlink2, false, cx))
        .child(checkbox(&CHK_SIGNING, signing, true, cx))
        .child(reset_button(cx));

    let title = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_sm().text_color(rgb(theme::TEXT)).child(TITLE))
        .child(
            crate::probe::measured("linkstats-close", div())
                .id("linkstats-close")
                .px_2()
                .cursor_pointer()
                .text_color(rgb(theme::DIM))
                .hover(|style| style.text_color(rgb(theme::ALERT)))
                .child("\u{2715}")
                .on_click(cx.listener(|this, _event: &gpui::ClickEvent, _window, cx| {
                    this.close_link_stats();
                    cx.notify();
                })),
        );
    Some(
        gpui::deferred(
            gpui::anchored()
                .position(point(px(WINDOW_AT.0), px(WINDOW_AT.1)))
                .child(
                    crate::probe::measured("linkstats", div())
                        .id("linkstats")
                        .flex()
                        .flex_col()
                        .bg(rgb(theme::PANEL))
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .rounded_md()
                        .occlude()
                        .child(title)
                        .child(client),
                ),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn ms(millis: u64) -> Duration {
        Duration::from_millis(millis)
    }

    /// The link's counters: bytes in and out, packets and losses, and the longest interval since
    /// `since`.
    fn link(
        bytes: (u64, u64),
        packets: (u64, u64),
        interval: Duration,
        since: Instant,
    ) -> LinkStats {
        LinkStats {
            bytes_read: bytes.0,
            bytes_written: bytes.1,
            packets_received: packets.0,
            packets_lost: packets.1,
            max_packet_interval: interval,
            interval_since: Some(since),
            ..LinkStats::default()
        }
    }

    /// Every box empty at the start; the totals at the first quarter second, counted from the
    /// start's counters; the longest interval as soon as a packet has come; the three-second means
    /// from the third second - bytes truncated, packets rounded, the quality the share received -
    /// and the bytes sent's five-second mean from the fifth.
    /// `// C#: Controls/ConnectionStats.cs:39-132`
    #[test]
    fn each_box_reads_as_the_csharp_writes_it() {
        let t0 = Instant::now();
        // The link had counted before the window opened: none of it is the window's.
        let base = link((50_000, 9_000), (400, 7), ms(900), t0);
        let mut stats = Stats::new(Counts::of(&base), t0);
        assert_eq!(stats.texts(), Texts::default());

        // 100 ms: nothing taken yet, but the interval is the link's since this start.
        stats.sample(&link((50_300, 9_000), (403, 7), ms(40), t0), t0 + ms(100));
        let texts = stats.texts();
        assert_eq!(texts.packets_rx, "");
        assert_eq!(texts.bytes_received, "");
        assert_eq!(texts.max_packet_interval, "40");

        // 250 ms: the totals. No packet lost or byte sent yet: those boxes stay empty.
        stats.sample(&link((50_600, 9_000), (406, 7), ms(40), t0), t0 + ms(250));
        let texts = stats.texts();
        assert_eq!(texts.bytes_received, "600b");
        assert_eq!(texts.packets_rx, "6");
        assert_eq!(texts.packets_lost, "");
        assert_eq!(texts.bytes_sent, "");
        assert_eq!(texts.packets_per_second, "");

        // Each whole second: 2,000 bytes and 10 packets, then 3,000 and 11, then 1,001 and 12 with
        // 3 lost; 100 bytes sent each second.
        stats.sample(&link((52_000, 9_100), (410, 7), ms(40), t0), t0 + ms(1_000));
        stats.sample(&link((55_000, 9_200), (421, 7), ms(55), t0), t0 + ms(2_000));
        assert_eq!(stats.texts().bytes_per_second_rx, "", "two seconds");
        stats.sample(
            &link((56_001, 9_300), (433, 10), ms(55), t0),
            t0 + ms(3_000),
        );
        let texts = stats.texts();
        // (2,000 + 3,000 + 1,001) / 3 = 2,000.33, `(int)` 2,000.
        assert_eq!(texts.bytes_per_second_rx, "1.95Kb");
        // 33 / 3 = 11.
        assert_eq!(texts.packets_per_second, "11");
        // 33 / 36.
        assert_eq!(texts.link_quality, "92%");
        assert_eq!(texts.packets_rx, "33");
        assert_eq!(texts.packets_lost, "3");
        assert_eq!(texts.bytes_received, "5.86Kb");
        assert_eq!(texts.bytes_sent, "300b");
        assert_eq!(texts.max_packet_interval, "55");
        assert_eq!(texts.bytes_per_second_sent, "", "three seconds of five");

        stats.sample(
            &link((57_000, 9_400), (443, 10), ms(55), t0),
            t0 + ms(4_000),
        );
        assert_eq!(stats.texts().packets_per_second, "11", "(11 + 12 + 10) / 3");
        stats.sample(
            &link((58_000, 9_550), (453, 10), ms(55), t0),
            t0 + ms(5_000),
        );
        let texts = stats.texts();
        // (100 + 100 + 100 + 100 + 150) / 5 = 110.
        assert_eq!(texts.bytes_per_second_sent, "110b");
        // (12 + 10 + 10) / 3 = 10.67, rounded.
        assert_eq!(texts.packets_per_second, "11");
        // 32 received and 3 lost over the last three seconds.
        assert_eq!(texts.link_quality, "91%");
    }

    /// A second the window was not drawn in counts nothing, the next all; the means over seconds
    /// that hold both are as they were.
    #[test]
    fn a_second_not_drawn_in_lends_its_counts_to_the_next() {
        let t0 = Instant::now();
        let mut stats = Stats::new(Counts::default(), t0);
        stats.sample(&link((300, 0), (30, 0), ms(10), t0), t0 + ms(1_000));
        stats.sample(&link((900, 0), (90, 0), ms(10), t0), t0 + ms(3_000));
        assert_eq!(
            stats.texts().packets_per_second,
            "30",
            "90 over three seconds"
        );
        assert_eq!(stats.texts().bytes_per_second_rx, "300b");
    }

    /// `"00%"` of the share received: two digits at least, rounded half away from zero from
    /// fifteen digits as .NET rounds - 191 of 200 is 0.955, "96%" - and "NaN" with nothing.
    /// `// C#: Controls/ConnectionStats.cs:88, 157-163`
    #[test]
    fn the_quality_is_a_percentage_of_two_digits() {
        assert_eq!(quality(100, 0), "100%");
        assert_eq!(quality(95, 5), "95%");
        assert_eq!(quality(191, 9), "96%");
        assert_eq!(quality(1, 19), "05%");
        assert_eq!(quality(1, 999), "00%");
        assert_eq!(quality(1, 199), "01%", "0.5% rounds up");
        assert_eq!(quality(0, 5), "00%");
        assert_eq!(quality(0, 0), "NaN");
        assert_eq!(whole(2.5, false), 3, "half away from zero");
        assert_eq!(whole(10.0 / 3.0, false), 3);
        assert_eq!(whole(0.0, false), 0);
        assert_eq!(whole(1e-5, true), 0);
    }

    /// The interval in milliseconds as a double writes itself: ticks of 100 ns, no trailing zeros.
    /// `// C#: Controls/ConnectionStats.cs:127-132`
    #[test]
    fn the_interval_is_written_in_milliseconds() {
        assert_eq!(milliseconds(Duration::from_nanos(251_284_700)), "251.2847");
        assert_eq!(milliseconds(ms(1_000)), "1000");
        assert_eq!(milliseconds(Duration::from_nanos(99)), "0", "under a tick");
        assert_eq!(milliseconds(Duration::from_micros(1_500)), "1.5");
    }

    /// The link's longest interval is the window's only while it counts from the window's start:
    /// one from before a restart, or from another, is not shown.
    #[test]
    fn an_interval_from_another_start_is_not_shown() {
        let t0 = Instant::now();
        let mut stats = Stats::new(Counts::default(), t0);
        stats.sample(&link((10, 0), (1, 0), ms(900), t0 - ms(5)), t0 + ms(300));
        assert_eq!(stats.texts().max_packet_interval, "");
        stats.sample(&link((10, 0), (1, 0), Duration::ZERO, t0), t0 + ms(301));
        assert_eq!(stats.texts().max_packet_interval, "", "no packet since");
        stats.sample(&link((20, 0), (2, 0), ms(12), t0), t0 + ms(302));
        assert_eq!(stats.texts().max_packet_interval, "12");
    }

    /// Polls until `check` holds, failing rather than hanging.
    fn until(what: &str, check: impl Fn() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !check() {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            wasm_thread::sleep(ms(1));
        }
    }

    /// A link over an in-memory vehicle that has sent `packets` heartbeats, and its handle.
    fn vehicle_link(packets: u8) -> (mp_link::Link, mp_transport::testing::LoopbackEnd) {
        use mp_transport::Transport as _;
        let (mut vehicle, gcs) = mp_transport::testing::Loopback::pair();
        let link = mp_link::Link::from_transport(
            Box::new(gcs),
            mp_link::LinkConfig {
                send_heartbeat: false,
                stream_rate_hz: 0,
                ..mp_link::LinkConfig::default()
            },
        );
        for seq in 0..packets {
            vehicle
                .write_all(&mp_link::testing::heartbeat(seq))
                .unwrap();
        }
        let sender = link.sender();
        until("the packets", || {
            sender.stats().packets_received >= u64::from(packets)
        });
        (link, vehicle)
    }

    /// The form over a real link: its ticks as the vehicle's state has them; counting what comes
    /// after it opened, not before; Reset starting the counts again; the link closed, its boxes
    /// frozen; a new link, the form afresh on it; a tick setting the vehicle's state.
    /// `// C#: Controls/ConnectionStats.cs:17-28, 176-190; MainV2.cs:1244-1257, 1407-1418`
    #[test]
    fn the_form_counts_its_link_from_when_it_opened() {
        use mp_transport::Transport as _;
        let id = VehicleId::new(1, 1);
        let (first, mut vehicle) = vehicle_link(3);
        let sender = first.sender();
        let t0 = Instant::now();
        let mut form = Form::new(t0);
        assert!(!form.updating());
        form.step(Some((first.sender(), id)), true, t0);
        assert!(form.updating());
        assert!(form.mavlink2, "a vehicle heard in MAVLink 2");
        assert!(!form.signing);
        until("the restart", || {
            sender.stats().interval_since == Some(form.stats.started())
        });

        for seq in 3..8 {
            vehicle
                .write_all(&mp_link::testing::heartbeat(seq))
                .unwrap();
        }
        until("five more", || sender.stats().packets_received >= 8);
        form.step(Some((first.sender(), id)), true, t0 + ms(300));
        assert_eq!(form.texts().packets_rx, "5", "only those since it opened");
        assert_ne!(form.texts().max_packet_interval, "");

        // Reset: the counts start again, from the link's counters now.
        form.reset(t0 + ms(400));
        assert_eq!(form.resets, 1);
        assert_eq!(form.texts(), Texts::default());
        form.step(Some((first.sender(), id)), true, t0 + ms(700));
        assert_eq!(form.texts().packets_rx, "", "nothing since the reset");

        // A tick: the vehicle's state with it.
        form.mavlink2 = false;
        sender.set_mavlink2(id, form.mavlink2);
        assert_eq!(sender.mavlink2(id), Some(false));

        // The link closed: the boxes as they were, the counting stopped.
        let before = form.texts();
        form.step(None, false, t0 + ms(800));
        assert!(!form.updating());
        assert_eq!(form.texts(), before);

        // A new link: the form afresh on it, the ticks read again.
        let (second, _vehicle) = vehicle_link(2);
        form.step(Some((second.sender(), id)), true, t0 + ms(900));
        assert!(form.updating());
        assert!(form.mavlink2);
        assert!(!form.link.as_ref().unwrap().same_link(&sender));
        assert_eq!(form.texts(), Texts::default());
    }
}
