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

//! The layout tour: under `MP_TOUR`, once a vehicle's parameters are in, the application shows
//! every screen and every page within one - FLIGHT DATA's tabs, SETUP's and CONFIG's pages,
//! LOGS's pages - a few seconds each, and quits, so the layout guard's record
//! (`crate::layout_guard::RECORD_FILE`) holds every cut-off a connected vehicle brings out (the
//! owner, 2026-10-05: a sweep with no vehicle and only each screen's first page "isnt likely to
//! hit CUTOFFs now, as they mostly all need a connected vehicle or sim").
//!
//! A harness's switch, like `MP_PROBE` and `MP_FACTS`: nothing happens without it.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use web_time::{Duration, Instant};

use crate::Screen;
use crate::fly::Page;
use crate::logs_tab::LogsPage;
use crate::setup::List;

/// How long the tour waits for the parameters before it goes anyway.
const PARAMETERS_WAIT: Duration = Duration::from_secs(150);

/// One place the tour shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stop {
    /// A screen, as it opens.
    Screen(Screen),
    /// One of FLIGHT DATA's pages.
    Fly(Page),
    /// SETUP's or CONFIG's pages, every one, read from the list when the tour gets there: a list
    /// is built the first time its screen shows, so asked at the start it has none.
    Pages(List),
    /// One of SETUP's or CONFIG's pages, by its index in its list.
    Backstage(List, usize),
    /// One of LOGS's pages.
    Logs(LogsPage),
}

/// Where the tour is.
#[derive(Debug)]
pub struct Tour {
    /// How long each stop is shown.
    dwell: Duration,
    /// When the tour was switched on.
    started: Instant,
    /// The stops, once the vehicle's parameters are in; then the next one's index and when the
    /// one showing was shown.
    stops: Option<(Vec<Stop>, usize, Instant)>,
}

impl Tour {
    /// The tour `MP_TOUR` asks for: its value, the seconds each stop is shown (2 when it is not a
    /// number), or nothing without it.
    #[must_use]
    pub fn from_env() -> Option<Self> {
        let value = std::env::var("MP_TOUR").ok()?;
        let seconds = value.trim().parse::<f32>().ok().filter(|s| *s > 0.0).unwrap_or(2.0);
        Some(Self {
            dwell: Duration::from_secs_f32(seconds),
            started: Instant::now(),
            stops: None,
        })
    }

    /// Whether the tour has its stops, ready to go: the vehicle's parameters in, or waited for
    /// long enough.
    #[must_use]
    pub fn waiting(&self, parameters: usize, expected: usize, now: Instant) -> bool {
        self.stops.is_none()
            && !(expected > 0 && parameters >= expected)
            && now.duration_since(self.started) < PARAMETERS_WAIT
    }

    /// Whether the stops have been chosen.
    #[must_use]
    pub const fn has_begun(&self) -> bool {
        self.stops.is_some()
    }

    /// Starts with these stops.
    pub fn begin(&mut self, stops: Vec<Stop>, now: Instant) {
        self.stops = Some((stops, 0, now - self.dwell));
    }

    /// The stop to show now, if the one showing has had its time; `None` when it has not, and
    /// `Some(None)` once every stop has been shown.
    pub fn next(&mut self, now: Instant) -> Option<Option<Stop>> {
        let (stops, index, shown) = self.stops.as_mut()?;
        if now.duration_since(*shown) < self.dwell {
            return None;
        }
        *shown = now;
        let stop = stops.get(*index).copied();
        *index += 1;
        Some(stop)
    }

    /// Puts `stops` next, to be shown at once: a list's pages, read when its screen has shown.
    pub fn expand(&mut self, stops: Vec<Stop>, now: Instant) {
        if let Some((list, index, shown)) = self.stops.as_mut() {
            let at = (*index).min(list.len());
            list.splice(at..at, stops);
            *shown = now - self.dwell;
        }
    }

    /// `tour.at`: how far through, as `n/total`.
    #[must_use]
    pub fn progress(&self) -> String {
        self.stops.as_ref().map_or_else(
            || "waiting".to_owned(),
            |(stops, index, _)| format!("{index}/{}", stops.len()),
        )
    }
}

/// Every place to show, in the tab strip's order: each screen, and within FLIGHT DATA, SETUP,
/// CONFIG and LOGS each of its pages - SETUP's and CONFIG's read when the tour gets there.
#[must_use]
pub fn stops(fly_pages: &[Page]) -> Vec<Stop> {
    let mut out = Vec::new();
    for screen in Screen::ALL {
        out.push(Stop::Screen(screen));
        match screen {
            Screen::Fly => out.extend(fly_pages.iter().map(|page| Stop::Fly(*page))),
            Screen::Setup => out.push(Stop::Pages(List::Setup)),
            Screen::Config => out.push(Stop::Pages(List::Config)),
            Screen::Logs => out.extend(LogsPage::ALL.iter().map(|page| Stop::Logs(*page))),
            _ => {}
        }
    }
    out
}

/// A list's pages as stops.
#[must_use]
pub fn pages(list: List, indices: &[usize]) -> Vec<Stop> {
    indices
        .iter()
        .map(|index| Stop::Backstage(list, *index))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each screen, then its pages, in the strip's order; LOGS's three pages after it.
    #[test]
    fn the_tour_shows_every_screen_and_page() {
        let fly = [Page::Quick, Page::Actions];
        let stops = stops(&fly);
        assert_eq!(stops.first(), Some(&Stop::Screen(Screen::Fly)));
        assert_eq!(stops.get(1), Some(&Stop::Fly(Page::Quick)));
        assert_eq!(stops.get(2), Some(&Stop::Fly(Page::Actions)));
        let setup = stops
            .iter()
            .position(|stop| *stop == Stop::Screen(Screen::Setup))
            .expect("SETUP");
        assert_eq!(stops.get(setup + 1), Some(&Stop::Pages(List::Setup)));
        assert!(stops.contains(&Stop::Pages(List::Config)));
        assert!(stops.contains(&Stop::Logs(LogsPage::Review)));
        assert_eq!(
            stops.iter().filter(|stop| matches!(stop, Stop::Screen(_))).count(),
            Screen::ALL.len()
        );
    }

    /// It waits for the parameters, then shows each stop for its time, then says it is done.
    #[test]
    fn the_tour_waits_then_shows_each_stop_in_turn() {
        let start = Instant::now();
        let mut tour = Tour {
            dwell: Duration::from_secs(2),
            started: start,
            stops: None,
        };
        assert!(tour.waiting(10, 1_400, start));
        assert!(!tour.waiting(1_400, 1_400, start));
        assert!(!tour.waiting(0, 0, start + PARAMETERS_WAIT));
        tour.begin(vec![Stop::Screen(Screen::Fly), Stop::Fly(Page::Quick)], start);
        assert_eq!(tour.next(start), Some(Some(Stop::Screen(Screen::Fly))));
        assert_eq!(tour.next(start + Duration::from_secs(1)), None, "still showing");
        assert_eq!(
            tour.next(start + Duration::from_secs(2)),
            Some(Some(Stop::Fly(Page::Quick)))
        );
        assert_eq!(tour.progress(), "2/2");
        assert_eq!(tour.next(start + Duration::from_secs(4)), Some(None), "done");
    }

    /// A list's pages, read when its screen has shown, go in next and the first shows at once.
    #[test]
    fn a_lists_pages_go_in_where_the_tour_reads_them() {
        let start = Instant::now();
        let mut tour = Tour {
            dwell: Duration::from_secs(2),
            started: start,
            stops: None,
        };
        tour.begin(
            vec![Stop::Pages(List::Setup), Stop::Screen(Screen::Help)],
            start,
        );
        assert_eq!(tour.next(start), Some(Some(Stop::Pages(List::Setup))));
        tour.expand(pages(List::Setup, &[3, 7]), start);
        assert_eq!(tour.next(start), Some(Some(Stop::Backstage(List::Setup, 3))));
        assert_eq!(
            tour.next(start + Duration::from_secs(2)),
            Some(Some(Stop::Backstage(List::Setup, 7)))
        );
        assert_eq!(
            tour.next(start + Duration::from_secs(4)),
            Some(Some(Stop::Screen(Screen::Help)))
        );
    }
}
