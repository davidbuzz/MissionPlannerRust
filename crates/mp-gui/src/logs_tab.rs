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

//! The LOGS tab's pages (the owner, 2026-10-04): the flight screen's Telemetry Logs and DataFlash
//! Logs pages (`tabTLogs`, `tablogbrowse` in `FlightData`'s `tabControlactions`) again here, and
//! Review a Log - the log browser (`Log/LogBrowse.cs`), which Mission Planner opens as a window of
//! its own and which this tab was until now. Mission Planner has no LOGS tab; the pages are the
//! flight screen's own (`crate::fly::playback_page`, `crate::fly::dataflash_page`), drawn from the
//! same state, so a log playing or a conversion running shows the same in both places.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{Context, IntoElement, div, prelude::*, px, rgb};

use crate::MissionPlanner;
use crate::i18n::fl;
use crate::ui::theme;

/// A page of the LOGS tab.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LogsPage {
    /// `tabTLogs`: Load Log, Play/Pause, the speed and the track bar, Tlog > Kml or Graph.
    TLogs,
    /// `tablogbrowse`: the downloader, Review a Log, the conversions, Geo Reference Images.
    DataFlash,
    /// The log browser, which the tab opens on, as it showed nothing else before.
    #[default]
    Review,
}

impl LogsPage {
    /// In the strip's order: the flight screen's two pages, then the window one of them opens.
    pub const ALL: [Self; 3] = [Self::TLogs, Self::DataFlash, Self::Review];

    /// The probe and element id of its tab.
    pub const fn id(self) -> &'static str {
        match self {
            Self::TLogs => "logs-tab-tlogs",
            Self::DataFlash => "logs-tab-dataflash",
            Self::Review => "logs-tab-review",
        }
    }

    /// The `logs.page` fact.
    pub const fn name(self) -> &'static str {
        match self {
            Self::TLogs => "tlogs",
            Self::DataFlash => "dataflash",
            Self::Review => "review",
        }
    }

    /// Its tab's text: the flight screen's page names, and the button that opens the browser.
    pub fn text(self) -> &'static str {
        match self {
            Self::TLogs => fl!("flightdata-tabTLogs-Text"),
            Self::DataFlash => fl!("flightdata-tablogbrowse-Text"),
            Self::Review => fl!("flightdata-BUT_logbrowse-Text"),
        }
    }
}

/// The tabs along the top of the LOGS tab, drawn as the flight screen's page strip draws its own.
pub fn strip(selected: LogsPage, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    let mut row = div()
        .id("logs-tabs")
        .flex()
        .flex_shrink_0()
        .items_end()
        .gap_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER));
    for page in LogsPage::ALL {
        let chosen = page == selected;
        row = row.child(
            crate::probe::measured(page.id(), div())
                .id(page.id())
                .flex_shrink_0()
                .px_2()
                .py_1()
                .rounded_t_md()
                .text_xs()
                .cursor_pointer()
                .bg(rgb(if chosen { theme::PANEL } else { theme::BG }))
                .text_color(rgb(if chosen { theme::ACCENT } else { theme::DIM }))
                .border_b_2()
                .border_color(rgb(if chosen { theme::ACCENT } else { theme::BG }))
                .hover(|style| style.text_color(rgb(theme::TEXT)))
                .child(page.text())
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.logs_page = page;
                    cx.notify();
                })),
        );
    }
    row
}

/// A flight-screen page on the LOGS tab: its own box at the top left, as wide as the flight
/// screen's column (400), scrolling if the window is shorter than it.
pub fn page_box(id: &'static str, page: gpui::AnyElement) -> impl IntoElement {
    div()
        .id(id)
        .flex()
        .flex_col()
        .flex_1()
        .min_h(px(0.0))
        .overflow_y_scroll()
        .child(
            div()
                .w(px(400.0))
                .p_2()
                .bg(rgb(theme::PANEL))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .rounded_md()
                .child(page),
        )
}

/// `logs.page`, the page showing.
pub fn record_facts(page: LogsPage) {
    crate::facts::record("logs.page", page.name());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The owner's order: Telemetry Logs, DataFlash Logs, then Review a Log, which the tab opens
    /// on; each with an id of its own.
    #[test]
    fn the_pages_are_the_flight_screens_two_then_the_browser() {
        assert_eq!(
            LogsPage::ALL,
            [LogsPage::TLogs, LogsPage::DataFlash, LogsPage::Review]
        );
        assert_eq!(LogsPage::default(), LogsPage::Review);
        let ids: std::collections::BTreeSet<_> = LogsPage::ALL.iter().map(|page| page.id()).collect();
        assert_eq!(ids.len(), 3);
        let names: Vec<_> = LogsPage::ALL.iter().map(|page| page.name()).collect();
        assert_eq!(names, ["tlogs", "dataflash", "review"]);
    }
}
