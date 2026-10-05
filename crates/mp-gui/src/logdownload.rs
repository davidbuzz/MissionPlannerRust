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

//! The Log Downloader: `Log/LogDownloadMavLink.cs`, the window the flight screen's DataFlash Logs
//! page opens with Download DataFlash Log Via Mavlink.
//!
//! The form, as its Designer lays it out: "Log files:" over `CHK_logs`, a checked list of every
//! log the vehicle lists with its date and size; under it the buttons - Download All Logs,
//! Download Selected Logs and Clear Logs in the first column, First Person KML, Create KML and
//! .bin to .log in the second; under those the progress bar and its byte count. On the right,
//! "Output:" over `TXT_seriallog`, where each step says what it did, and the note about posting
//! the `.bin`. `Show()`, not `ShowDialog()`: the rest of the application stays usable while it
//! is open, and a download carries on when it is closed.
//! `// C#: Log/LogDownloadMavLink.cs, Log/LogDownloadMavLink.Designer.cs:116-180, Log/LogDownloadMavLink.resx`
//!
//! Clear Logs erases the vehicle's logs with `LOG_ERASE`, which the link does not send; the three
//! KML and conversion buttons are the log tools this application has elsewhere or not at all.
//! They are drawn, greyed, where the C# has them, and the window says why.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeSet;
use web_time::Instant;

use gpui::{AnyElement, Context, Window, div, prelude::*, px, rgb};
use mp_ftp::logs::LogListing;

use crate::MissionPlanner;
use crate::ui::{action, progress, theme};

/// `LogStrings`, the words the downloader says. `// C#: Log/LogStrings.resx`
pub mod strings {
    /// `LogStrings.NotConnected`.
    pub const NOT_CONNECTED: &str = "Please connect your drone";
    /// `LogStrings.FetchingLogfileList`.
    pub const FETCHING_LIST: &str = "Getting list of log files...";
    /// `LogStrings.SomeLogsFound`, with `{0}`.
    pub const SOME_LOGS: &str = "Found {0} log files, note: item sizes are just an estimate.";
    /// `LogStrings.FetchingLog`, with `{0}`.
    pub const FETCHING_LOG: &str = "Fetching log file {0} ...";
    /// `LogStrings.DownloadStarting`, with `{0}`.
    pub const DOWNLOAD_STARTING: &str = "Downloading log files to: {0}";
    /// `LogStrings.NothingSelected`.
    pub const NOTHING_SELECTED: &str =
        "Please select some logs by checking the check boxes in the list";
    /// `LogStrings.CancelDownload`.
    pub const CANCEL_DOWNLOAD: &str = "You are downloading a log file, do you want to cancel?";
    /// What `DownloadThread` says at the end. `// C#: Log/LogDownloadMavLink.cs:356`
    pub const COMPLETE: &str = "Download complete.";
    /// `Log_Load`'s warning to an armed vehicle. `// C#: Log/LogDownloadMavLink.cs:55-56`
    pub const DISARM: &str = "Please disarm the drone before downloading logs!";
    /// `LabelStatus.Text`. `// C#: Log/LogDownloadMavLink.resx`
    pub const NOTE: &str = "NOTE: When posting support querys, please send the .bin file";
}

/// `ConnectionStats.ToHumanReadableByteCount`: Gb, Mb and Kb to two places, whole bytes below.
/// `// C#: Controls/ConnectionStats.cs:165-174`
#[must_use]
pub fn human_bytes(bytes: u32) -> String {
    const K: u32 = 1024;
    let scaled = |divisor: u32| {
        #[allow(clippy::cast_precision_loss)] // a log's size, as the C#'s float division
        let value = bytes as f32 / divisor as f32;
        value
    };
    if bytes > K * K * K {
        format!("{:.2}Gb", scaled(K * K * K))
    } else if bytes > K * K {
        format!("{:.2}Mb", scaled(K * K))
    } else if bytes > K {
        format!("{:.2}Kb", scaled(K))
    } else if bytes == 0 {
        // "####" writes nothing for zero.
        "b".to_owned()
    } else {
        format!("{bytes}b")
    }
}

/// A log's line in `CHK_logs`: its number, when it started in local time as `DateTime.ToString`
/// writes it in an English culture, and its size. `// C#: Log/LogDownloadMavLink.cs:91-125`
#[must_use]
pub fn caption(listing: &LogListing) -> String {
    format!(
        "{} {}  ({})",
        listing.id,
        item_caption(listing),
        human_bytes(listing.size)
    )
}

/// `GetItemCaption`: the Unix time made local. `// C#: Log/LogDownloadMavLink.cs:121-124`
fn item_caption(listing: &LogListing) -> String {
    use chrono::TimeZone as _;
    chrono::Local
        .timestamp_opt(i64::from(listing.time_utc), 0)
        .single()
        .map_or_else(String::new, |time| {
            time.format("%-m/%-d/%Y %-I:%M:%S %p").to_string()
        })
}

/// The window's state.
#[derive(Debug, Default)]
pub struct LogDownloader {
    /// Whether it is showing.
    open: bool,
    /// The logs checked in `CHK_logs`, by number.
    checked: BTreeSet<u16>,
    /// `TXT_seriallog`, oldest first.
    output: Vec<String>,
    /// How many logs had been listed when the output last said so.
    reported: usize,
    /// Whether a list was asked for since the window opened.
    listing: bool,
    /// The logs `DownloadThread` has still to fetch, in order, with their sizes and captions.
    queue: Vec<(u16, u32, String)>,
    /// The log being fetched.
    current: Option<u16>,
    /// `totalBytes`: the batch's size.
    total: u32,
    /// `tallyBytes`: what the logs finished so far in the batch held.
    tally: u32,
    /// When the batch's first byte arrived, for the rate.
    started: Option<Instant>,
    /// "Cancel Download" is being asked.
    confirming_close: bool,
    /// The progress bar and `labelBytes`, as this frame's `UpdateProgress` left them.
    bar: Option<(f32, String)>,
}

impl LogDownloader {
    /// Whether the window is showing.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.open
    }

    /// `AppendSerialLog`.
    fn say(&mut self, line: impl Into<String>) {
        self.output.push(line.into());
    }

    /// Whether `DownloadThread` is running: `status == SerialStatus.Reading`.
    #[must_use]
    pub const fn downloading(&self) -> bool {
        self.current.is_some()
    }

    /// The last line of the output.
    #[must_use]
    pub fn last_output(&self) -> &str {
        self.output.last().map_or("", String::as_str)
    }

    /// Toggles a log's box.
    pub fn toggle(&mut self, id: u16) {
        if !self.checked.remove(&id) {
            self.checked.insert(id);
        }
    }

    /// The logs checked.
    #[must_use]
    pub fn checked(&self) -> Vec<u16> {
        self.checked.iter().copied().collect()
    }

    /// The link has `count` logs listed: once a list was asked for, the output says how many -
    /// `LoadCheckedList`'s "Found" line. The entries arrive one `LOG_ENTRY` at a time, where the
    /// C# waits for the whole list, so the line is said again, in place, as the count grows.
    /// `// C#: Log/LogDownloadMavLink.cs:91-119`
    pub fn listed(&mut self, count: usize) {
        if !self.listing || count == self.reported || count == 0 {
            return;
        }
        if self.reported != usize::MAX
            && self
                .output
                .last()
                .is_some_and(|line| line.starts_with("Found "))
        {
            self.output.pop();
        }
        self.say(strings::SOME_LOGS.replace("{0}", &count.to_string()));
        self.reported = count;
    }

    /// Starts a batch: `DownloadThread(selected)`'s opening - the sizes added up, then the
    /// first log asked for. Returns the log to ask for.
    fn start_batch(&mut self, logs: Vec<(u16, u32, String)>) -> Option<(u16, u32)> {
        self.total = logs.iter().map(|(_, size, _)| *size).sum();
        self.tally = 0;
        self.started = None;
        self.queue = logs;
        self.next()
    }

    /// The next log of the batch, said as `FetchingLog` says it.
    fn next(&mut self) -> Option<(u16, u32)> {
        if self.queue.is_empty() {
            self.current = None;
            return None;
        }
        let (id, size, caption) = self.queue.remove(0);
        self.say(strings::FETCHING_LOG.replace("{0}", &caption));
        self.current = Some(id);
        Some((id, size))
    }

    /// A log of the batch arrived whole: counted, and the next asked for - or the batch is
    /// done. `// C#: Log/LogDownloadMavLink.cs:340-357`
    pub fn finished(&mut self, id: u16, bytes: u32) -> Option<(u16, u32)> {
        if self.current != Some(id) {
            return None;
        }
        self.tally = self.tally.saturating_add(bytes);
        let next = self.next();
        if next.is_none() {
            self.say(strings::COMPLETE);
        }
        next
    }

    /// `UpdateProgress(0, totalBytes, tallyBytes + received)`: the bar's fraction and
    /// `labelBytes` - what has arrived, the percentage, the rate and the time left - or `None`
    /// once the bar is hidden, when everything has arrived.
    /// `// C#: Log/LogDownloadMavLink.cs:390-426`
    #[must_use]
    pub fn progress(&mut self, received: u32, now: Instant) -> Option<(f32, String)> {
        let current = self.tally.saturating_add(received);
        let max = self.total;
        if self.current.is_none() || current >= max {
            return None;
        }
        if current == 0 {
            self.started = Some(now);
            return Some((0.0, String::new()));
        }
        let started = *self.started.get_or_insert(now);
        let elapsed = now.duration_since(started).as_secs_f64();
        let elapsed = if elapsed == 0.0 { 1.0 } else { elapsed };
        let per = f64::from(current) / f64::from(max) * 100.0;
        let rate = f64::from(current) / elapsed;
        let rate = if rate == 0.0 { 1.0 } else { rate };
        let left = f64::from(max - current) / rate;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let (left_seconds, rate_bytes) = (left as u64, rate as u32);
        let hours = left_seconds / 3600;
        let clock = if hours > 0 {
            format!(
                "{hours}:{:02}:{:02}",
                left_seconds / 60 % 60,
                left_seconds % 60
            )
        } else {
            format!("{:02}:{:02}", left_seconds / 60 % 60, left_seconds % 60)
        };
        #[allow(clippy::cast_possible_truncation)]
        let fraction = (f64::from(current) / f64::from(max)) as f32;
        Some((
            fraction,
            format!(
                "{} {per:.1}% {}/s {clock} left",
                human_bytes(current),
                human_bytes(rate_bytes)
            ),
        ))
    }

    /// Publishes what a UI test asserts on.
    pub fn record_facts(&self, listed: usize) {
        crate::facts::record("fly.logs.open", self.is_open());
        crate::facts::record("fly.logs.items", listed);
        crate::facts::record(
            "fly.logs.checked",
            self.checked()
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(","),
        );
        crate::facts::record("fly.logs.output", self.last_output());
        crate::facts::record(
            "fly.logs.fetching",
            self.current
                .map_or_else(|| "none".to_owned(), |id| id.to_string()),
        );
        crate::facts::record("fly.logs.queued", self.queue.len());
        crate::facts::record("fly.logs.confirm", self.confirming_close);
    }
}

impl MissionPlanner {
    /// `BUT_DFMavlink_Click`, then the form's `Log_Load`: the list asked for, or why not, and
    /// the warning to an armed vehicle. `// C#: GCSViews/FlightData.cs:1204-1209,
    /// Log/LogDownloadMavLink.cs:51-89`
    pub(crate) fn logs_open(&mut self) {
        let view = self.telemetry.view();
        let logs = &mut self.fly_data.logs;
        logs.open = true;
        logs.confirming_close = false;
        if crate::fly::port_open(&view) && view.vehicle.is_some() {
            logs.say(strings::FETCHING_LIST);
            logs.listing = true;
            logs.reported = usize::MAX;
            self.telemetry.request_log_list();
        } else {
            logs.say(strings::NOT_CONNECTED);
        }
        if view.state.as_ref().is_some_and(|state| state.armed) {
            self.file_status = Some(crate::fly::error_box(strings::DISARM));
        }
    }

    /// `BUT_DLall_Click`: with nothing listed, the list asked for again; otherwise every log, in
    /// the list's order, into the log directory. `// C#: Log/LogDownloadMavLink.cs:158-197`
    fn logs_download_all(&mut self) {
        if self.fly_data.logs.downloading() {
            return;
        }
        let listings = self.telemetry.log_listings();
        if listings.is_empty() {
            self.logs_open();
            return;
        }
        let directory = Self::plan_directory();
        self.fly_data
            .logs
            .say(strings::DOWNLOAD_STARTING.replace("{0}", &directory.display().to_string()));
        let batch = listings
            .iter()
            .map(|listing| (listing.id, listing.size, item_caption(listing)))
            .collect();
        if let Some((id, size)) = self.fly_data.logs.start_batch(batch) {
            self.telemetry.download_log(id, size);
        }
    }

    /// `BUT_DLthese_Click`: the checked logs, or a reminder to check some.
    /// `// C#: Log/LogDownloadMavLink.cs:428-448`
    fn logs_download_selected(&mut self) {
        if self.fly_data.logs.downloading() {
            return;
        }
        let listings = self.telemetry.log_listings();
        let batch: Vec<(u16, u32, String)> = listings
            .iter()
            .filter(|listing| self.fly_data.logs.checked.contains(&listing.id))
            .map(|listing| (listing.id, listing.size, item_caption(listing)))
            .collect();
        if batch.is_empty() {
            self.fly_data.logs.say(strings::NOTHING_SELECTED);
            return;
        }
        if let Some((id, size)) = self.fly_data.logs.start_batch(batch) {
            self.telemetry.download_log(id, size);
        }
    }

    /// The form's close box: `OnClosing` asks first while a log is being read.
    /// `// C#: Log/LogDownloadMavLink.cs:272-285`
    fn logs_close(&mut self) {
        if self.fly_data.logs.downloading() {
            self.fly_data.logs.confirming_close = true;
        } else {
            self.fly_data.logs.open = false;
        }
    }

    /// Once a frame: the list's arrival said in the output, and a download kept moving, written
    /// out when it finishes, and followed by the next of its batch.
    ///
    /// Each log is written where this application has always put a downloaded log, the plan
    /// directory, as `log_<n>.bin`. The C# writes `<log dir>/<vehicle type>/<sysid>/<n> <date>.bin`
    /// and renames it by the log's first GPS time; that naming is not ported.
    pub(crate) fn logs_tick(&mut self) {
        let listed = self.telemetry.log_listings().len();
        self.fly_data.logs.listed(listed);
        let received = self.telemetry.log_progress().map(|(_, got, _)| got);
        self.fly_data.logs.bar = self
            .fly_data
            .logs
            .progress(received.unwrap_or(0), Instant::now());
        if received.is_none() {
            return;
        }
        if let Some((id, bytes)) = self.telemetry.finished_log() {
            self.telemetry.clear_log_download();
            let path = Self::plan_directory().join(format!("log_{id}.bin"));
            self.file_status = Some(match mp_os::fs::write(&path, &bytes) {
                Ok(()) => format!("wrote {} ({} bytes)", path.display(), bytes.len()),
                Err(err) => format!("could not write {}: {err}", path.display()),
            });
            let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
            if let Some((next, next_size)) = self.fly_data.logs.finished(id, size) {
                self.telemetry.download_log(next, next_size);
            }
        } else {
            self.telemetry.nudge_log_download();
        }
    }
}

/// The window, over the screen without a backdrop: `Show()` leaves everything behind it usable.
pub fn window(
    logs: &LogDownloader,
    listings: &[LogListing],
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if !logs.open {
        return None;
    }
    let size = window.viewport_size();
    let busy = logs.downloading();

    let mut list = div()
        .id("fly-logs-list")
        .flex()
        .flex_col()
        .h(px(244.0))
        .overflow_y_scroll()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG));
    for listing in listings {
        let id = listing.id;
        let checked = logs.checked.contains(&id);
        let key = format!("fly-log-{id}");
        list = list.child(
            crate::probe::measured(key.clone(), div())
                .id(gpui::SharedString::from(key))
                .flex()
                .gap_1()
                .px_1()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::BORDER)))
                .child(if checked { "\u{2611}" } else { "\u{2610}" })
                .child(caption(listing))
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.fly_data.logs.toggle(id);
                    cx.notify();
                })),
        );
    }

    // `tableLayoutPanel3`: two columns of three.
    let greyed = |id: &'static str, label: &'static str| {
        action(
            id,
            label,
            theme::TEXT,
            false,
            |_: &(), _: &mut Window, _: &mut gpui::App| {},
        )
    };
    let buttons = div()
        .grid()
        .grid_cols(2)
        .gap_1()
        .child(action(
            "fly-logs-dlall",
            "Download All Logs",
            theme::ACCENT,
            !busy,
            cx.listener(|this, _event: &(), _window, cx| {
                this.logs_download_all();
                cx.notify();
            }),
        ))
        .child(greyed("fly-logs-firstperson", "First Person KML"))
        .child(action(
            "fly-logs-dlthese",
            "Download Selected Logs",
            theme::ACCENT,
            !busy,
            cx.listener(|this, _event: &(), _window, cx| {
                this.logs_download_selected();
                cx.notify();
            }),
        ))
        .child(greyed("fly-logs-redokml", "Create KML"))
        .child(greyed("fly-logs-clear", "Clear Logs"))
        .child(greyed("fly-logs-bintolog", ".bin to .log"));

    let mut bar = div().flex().items_center().gap_2().h(px(20.0));
    if let Some((fraction, text)) = &logs.bar {
        bar = bar
            .child(div().flex_1().child(progress(*fraction, theme::ACCENT)))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::TEXT))
                    .child(text.clone()),
            );
    }

    let mut output = div()
        .id("fly-logs-output")
        .flex()
        .flex_col()
        .flex_1()
        .min_h(px(0.0))
        .overflow_y_scroll()
        .p_1()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG));
    for line in &logs.output {
        output = output.child(
            div()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(line.clone()),
        );
    }

    let mut body = crate::probe::measured("fly-logs", div())
        .id("fly-logs")
        .flex()
        .flex_col()
        .gap_2()
        .w(px(784.0).min(size.width - px(20.0)))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_md()
        .occlude()
        .child(
            div()
                .flex()
                .justify_between()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(theme::DIM))
                        .child("Log Downloader"),
                )
                .child(action(
                    "fly-logs-close",
                    "\u{2715}",
                    theme::TEXT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.logs_close();
                        cx.notify();
                    }),
                )),
        )
        .child(
            div()
                .flex()
                .gap_2()
                .h(px(390.0))
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .gap_1()
                        .w(px(321.0))
                        .child(
                            div()
                                .text_xs()
                                .text_color(rgb(theme::DIM))
                                .child("Log files:"),
                        )
                        .child(list)
                        .child(buttons)
                        .child(bar),
                )
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .gap_1()
                        .flex_1()
                        .min_w(px(0.0))
                        .child(div().text_xs().text_color(rgb(theme::DIM)).child("Output:"))
                        .child(output)
                        .child(
                            div()
                                .text_xs()
                                .text_color(rgb(theme::DIM))
                                .child(strings::NOTE),
                        ),
                ),
        )
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(
            "Clear Logs sends LOG_ERASE, which the link has no request for; First Person KML, \
             Create KML and .bin to .log are not ported.",
        ));
    if logs.confirming_close {
        body = body.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .p_2()
                .border_1()
                .border_color(rgb(theme::WARN))
                .rounded_md()
                .child(
                    div()
                        .flex_1()
                        .text_sm()
                        .text_color(rgb(theme::TEXT))
                        .child(strings::CANCEL_DOWNLOAD),
                )
                .child(action(
                    "fly-logs-cancel-yes",
                    "Yes",
                    theme::OK,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        // The form closes; `DownloadThread` carries on without it.
                        this.fly_data.logs.confirming_close = false;
                        this.fly_data.logs.open = false;
                        cx.notify();
                    }),
                ))
                .child(action(
                    "fly-logs-cancel-no",
                    "No",
                    theme::TEXT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.fly_data.logs.confirming_close = false;
                        cx.notify();
                    }),
                )),
        );
    }

    Some(
        gpui::deferred(
            gpui::anchored()
                .position(gpui::point(
                    ((size.width - px(784.0)) / 2.0).max(px(0.0)),
                    px(60.0),
                ))
                .child(body),
        )
        .with_priority(1)
        .into_any_element(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn sizes_read_as_the_csharp_writes_them() {
        assert_eq!(human_bytes(0), "b");
        assert_eq!(human_bytes(900), "900b");
        assert_eq!(human_bytes(1024), "1024b");
        assert_eq!(human_bytes(1536), "1.50Kb");
        assert_eq!(human_bytes(5 * 1024 * 1024 + 1), "5.00Mb");
    }

    #[test]
    fn a_log_is_listed_with_its_number_date_and_size() {
        let listing = LogListing {
            id: 3,
            size: 2048 + 512,
            time_utc: 0,
        };
        let text = caption(&listing);
        assert!(text.starts_with("3 "), "{text}");
        assert!(text.ends_with("  (2.50Kb)"), "{text}");
        // 1970, in whatever zone this machine keeps: "12/31/1969 ..." west of Greenwich.
        assert!(text.contains("/19"), "{text}");
    }

    #[test]
    fn a_batch_fetches_each_log_in_turn_and_says_so() {
        let mut logs = LogDownloader::default();
        let first = logs.start_batch(vec![(1, 100, "one".to_owned()), (2, 300, "two".to_owned())]);
        assert_eq!(first, Some((1, 100)));
        assert_eq!(logs.last_output(), "Fetching log file one ...");
        assert!(logs.downloading());
        // A finish for a log not being fetched changes nothing.
        assert_eq!(logs.finished(7, 5), None);
        assert_eq!(logs.finished(1, 100), Some((2, 300)));
        assert_eq!(logs.last_output(), "Fetching log file two ...");
        assert_eq!(logs.finished(2, 300), None);
        assert_eq!(logs.last_output(), "Download complete.");
        assert!(!logs.downloading());
    }

    #[test]
    fn the_progress_line_is_bytes_percent_rate_and_time_left() {
        let mut logs = LogDownloader::default();
        logs.start_batch(vec![(1, 20_480, "one".to_owned())]);
        let start = Instant::now();
        assert_eq!(logs.progress(0, start), Some((0.0, String::new())));
        let (fraction, text) = logs
            .progress(10_240, start + Duration::from_secs(10))
            .unwrap();
        assert!((fraction - 0.5).abs() < 1e-6);
        assert_eq!(text, "10.00Kb 50.0% 1024b/s 00:10 left");
        // Everything arrived: the bar goes.
        assert_eq!(logs.progress(20_480, start), None);
    }

    #[test]
    fn the_found_line_counts_the_list_as_it_arrives() {
        let mut logs = LogDownloader::default();
        // Nothing asked for: nothing said.
        logs.listed(3);
        assert_eq!(logs.last_output(), "");
        logs.listing = true;
        logs.reported = usize::MAX;
        logs.say(strings::FETCHING_LIST);
        logs.listed(0);
        assert_eq!(logs.last_output(), "Getting list of log files...");
        logs.listed(1);
        logs.listed(4);
        assert_eq!(
            logs.output,
            vec![
                "Getting list of log files...".to_owned(),
                "Found 4 log files, note: item sizes are just an estimate.".to_owned(),
            ]
        );
        logs.listed(4);
        assert_eq!(logs.output.len(), 2);
    }

    #[test]
    fn checking_a_log_twice_unchecks_it() {
        let mut logs = LogDownloader::default();
        logs.toggle(4);
        logs.toggle(2);
        assert_eq!(logs.checked(), vec![2, 4]);
        logs.toggle(4);
        assert_eq!(logs.checked(), vec![2]);
    }
}
