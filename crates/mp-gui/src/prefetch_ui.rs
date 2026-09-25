//! Map Tool > Prefetch and Prefetch WP Path: GMap.NET's `TilePrefetcherMenu` and
//! `TilePrefetcher` forms as the planner shows them (`GCSViews/FlightPlanner.cs:3305-3365,
//! 5030-5080` @ efb0801; `ExtLibs/GMap.NET.WindowsForms/GMap.NET.WindowsForms/
//! TilePrefetcher.cs`, `TilePrefetcherMenu.cs`; GPL-3.0-or-later).
//!
//! Prefetch: with no area rubber-banded on the map - this map has no `SelectedArea`, so always -
//! "No ripp area defined, ripp displayed on screen?" (Rip, Yes/No); Yes takes the view. The menu
//! then offers a zoom range, 1 to 20 to begin with, within the map's 0 to 24, counting the tiles
//! per zoom and estimating their size; OK runs a `TilePrefetcher` for each zoom in turn, a form
//! with the progress line and bar, until the last or until cancelled. Prefetch WP Path asks
//! "max zoom" (20 offered; a word is "Invalid number entered") and runs the prefetcher over each
//! segment of the planned path at every zoom from 1 to the lesser of the answer and 24, stopped
//! by Escape.
//!
//! The counting and the walk are `mp_tiles::prefetch`; what is here is the state the screen
//! holds and the two forms drawn over it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};
use mp_tiles::TileCache;
use mp_tiles::prefetch::{self, Area, Progress};
use mp_tiles::source::TileSource;
use mp_units::LatLon;

use crate::MissionPlanner;
use crate::ui::{action, theme};

/// `MainMap.MinZoom` and `MaxZoom`, the menu's range.
/// `// C#: GCSViews/FlightPlanner.cs:187-188`
pub(crate) const MIN_ZOOM: u8 = 0;
pub(crate) const MAX_ZOOM: u8 = 24;

/// The question with no area selected, and its title.
/// `// C#: GCSViews/FlightPlanner.cs:5035-5036`
pub(crate) const RIP_TITLE: &str = "Rip";
pub(crate) const RIP_QUESTION: &str = "No ripp area defined, ripp displayed on screen?";
/// `FetchPath`'s box.
/// `// C#: GCSViews/FlightPlanner.cs:3309`
pub(crate) const MAX_ZOOM_TITLE: &str = "max zoom";
pub(crate) const MAX_ZOOM_TEXT: &str = "Enter the max zoom to prefetch to.";
pub(crate) const MAX_ZOOM_OFFERED: &str = "20";

/// `TilePrefetcherMenu`: the area and the range being chosen. The spinners start at 1 and 20
/// and refuse a minimum above the maximum or a maximum below the minimum.
/// `// C#: TilePrefetcherMenu.cs:11-25, 87-140; TilePrefetcherMenu.Designer.cs:67, 79, 145, 169`
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PrefetchMenu {
    /// The area to fetch.
    pub(crate) area: Area,
    /// `Minimum`.
    pub(crate) min: u8,
    /// `Maximum`.
    pub(crate) max: u8,
}

impl PrefetchMenu {
    /// The menu as it opens.
    #[must_use]
    pub(crate) const fn new(area: Area) -> Self {
        Self {
            area,
            min: 1,
            max: 20,
        }
    }

    /// `numericUpDownMinZoom_ValueChanged`: taken only when it is not above the maximum.
    pub(crate) fn set_min(&mut self, value: u8) {
        if (MIN_ZOOM..=self.max).contains(&value) {
            self.min = value;
        }
    }

    /// `numericUpDownMaxZoom_ValueChanged`: taken only when it is not below the minimum.
    pub(crate) fn set_max(&mut self, value: u8) {
        if (self.min..=MAX_ZOOM).contains(&value) {
            self.max = value;
        }
    }

    /// `textBoxTile` and `labelTotal`.
    #[must_use]
    pub(crate) fn text(&self) -> (Vec<String>, String) {
        prefetch::menu_text(&self.area, self.min, self.max)
    }

    /// The zooms OK runs, in order.
    #[must_use]
    pub(crate) fn zooms(&self) -> Vec<u8> {
        (self.min..=self.max).collect()
    }
}

/// What the prefetch thread and the screen share.
#[derive(Debug, Default)]
struct Shared {
    progress: Mutex<Progress>,
    /// `label2`: "saving tiles..." while a walk runs, "all tiles saved" after.
    saving: Mutex<String>,
    cancel: AtomicBool,
    finished: AtomicBool,
    /// Tiles the cache held or gained, over every walk.
    ok: Mutex<usize>,
}

/// A prefetch under way or ended: `TilePrefetcher.Start` for each (area, zoom) in turn on a
/// thread of its own, the forms' words read from it each frame.
/// `// C#: GCSViews/FlightPlanner.cs:5055-5075, 3318-3362; TilePrefetcher.cs:113-141, 178-247`
#[derive(Debug)]
pub(crate) struct PrefetchJob {
    shared: Arc<Shared>,
    handle: Option<std::thread::JoinHandle<()>>,
    /// How many walks the job was given.
    pub(crate) walks: usize,
}

impl PrefetchJob {
    /// Starts the walks. `offline` is the map's cache-only mode, where nothing is fetched and
    /// only what the cache holds counts.
    #[must_use]
    pub(crate) fn start(
        walks: Vec<(Area, u8)>,
        source: &'static TileSource,
        cache: TileCache,
        offline: bool,
    ) -> Self {
        let shared = Arc::new(Shared::default());
        let worker = Arc::clone(&shared);
        let count = walks.len();
        let handle = std::thread::Builder::new()
            .name("tile-prefetch".to_owned())
            .spawn(move || {
                let fetcher = mp_tiles::TileFetcher::new();
                let fetch = |tile: mp_units::TileId| {
                    if offline {
                        None
                    } else {
                        fetcher.fetch(source, tile).ok()
                    }
                };
                for (index, (area, zoom)) in walks.iter().enumerate() {
                    if worker.cancel.load(Ordering::Acquire) {
                        break;
                    }
                    *lock(&worker.saving) = "saving tiles...".to_owned();
                    let mut report = |progress: &Progress| {
                        *lock(&worker.progress) = progress.clone();
                    };
                    let ok = prefetch::run(
                        area,
                        *zoom,
                        source.cache_name,
                        &cache,
                        &fetch,
                        0,
                        &worker.cancel,
                        index as u64 + 1,
                        &mut report,
                    );
                    *lock(&worker.ok) += ok;
                    *lock(&worker.saving) = "all tiles saved".to_owned();
                }
                worker.finished.store(true, Ordering::Release);
            })
            .ok();
        Self {
            shared,
            handle,
            walks: count,
        }
    }

    /// `label1.Text`.
    #[must_use]
    pub(crate) fn label(&self) -> String {
        lock(&self.shared.progress).label()
    }

    /// `label2.Text`.
    #[must_use]
    pub(crate) fn saving(&self) -> String {
        lock(&self.shared.saving).clone()
    }

    /// The bar, 0 to 1.
    #[must_use]
    pub(crate) fn fraction(&self) -> f32 {
        lock(&self.shared.progress).percent.clamp(0, 100) as f32 / 100.0
    }

    /// The zoom being walked.
    #[must_use]
    pub(crate) fn zoom(&self) -> u8 {
        lock(&self.shared.progress).zoom
    }

    /// Tiles the cache held or gained so far.
    #[must_use]
    pub(crate) fn ok(&self) -> usize {
        *lock(&self.shared.ok)
    }

    /// Whether every walk has ended.
    #[must_use]
    pub(crate) fn finished(&self) -> bool {
        self.shared.finished.load(Ordering::Acquire)
    }

    /// `UserAborted` / `fetchpathrip = false`: the walk stops between two tiles and no further
    /// walk starts.
    pub(crate) fn cancel(&self) {
        self.shared.cancel.store(true, Ordering::Release);
    }

    /// Whether the user stopped it.
    #[must_use]
    pub(crate) fn cancelled(&self) -> bool {
        self.shared.cancel.load(Ordering::Acquire)
    }

    /// What the facts call the state.
    #[must_use]
    pub(crate) fn state_name(&self) -> &'static str {
        if self.cancelled() {
            "cancelled"
        } else if self.finished() {
            "done"
        } else {
            "running"
        }
    }
}

impl Drop for PrefetchJob {
    fn drop(&mut self) {
        self.cancel();
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// `FetchPath`'s walks: every zoom from 1 to the lesser of `max_zoom` and the map's maximum,
/// and at each the rectangle of every consecutive pair of `pointlist` - home, then the rows
/// with a position, in order.
/// `// C#: GCSViews/FlightPlanner.cs:3318-3362`
#[must_use]
pub(crate) fn path_walks(points: &[LatLon], max_zoom: i32) -> Vec<(Area, u8)> {
    let top = max_zoom.min(i32::from(MAX_ZOOM));
    let mut walks = Vec::new();
    if top < 1 {
        return walks;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 1..=24
    for zoom in 1..=(top as u8) {
        for pair in points.windows(2) {
            if let [from, to] = pair {
                walks.push((Area::between(*from, *to), zoom));
            }
        }
    }
    walks
}

/// The menu form, over the screen, while a range is being chosen.
/// `// C#: TilePrefetcherMenu.Designer.cs`
pub(crate) fn menu_form(menu: &PrefetchMenu, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (lines, total) = menu.text();
    let spinner = |label: &'static str,
                   value: u8,
                   down: &'static str,
                   up: &'static str,
                   value_id: &'static str,
                   is_min: bool,
                   cx: &mut Context<MissionPlanner>| {
        let step = move |this: &mut MissionPlanner, delta: i32| {
            if let Some(menu) = this.plan_menus.prefetch_menu.as_mut() {
                let current = i32::from(if is_min { menu.min } else { menu.max });
                let Ok(next) = u8::try_from(current + delta) else {
                    return;
                };
                if is_min {
                    menu.set_min(next);
                } else {
                    menu.set_max(next);
                }
            }
        };
        div()
            .flex()
            .items_center()
            .gap_2()
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .w(px(70.0))
                    .child(label),
            )
            .child(action(
                down,
                "-",
                theme::TEXT,
                true,
                cx.listener(move |this, _event: &(), _window, cx| {
                    step(this, -1);
                    cx.notify();
                }),
            ))
            .child(
                crate::probe::measured(value_id, div())
                    .w(px(28.0))
                    .text_sm()
                    .text_color(rgb(theme::TEXT))
                    .child(value.to_string()),
            )
            .child(action(
                up,
                "+",
                theme::TEXT,
                true,
                cx.listener(move |this, _event: &(), _window, cx| {
                    step(this, 1);
                    cx.notify();
                }),
            ))
    };
    let form = crate::probe::measured("plan-prefetch-menu", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(300.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child("Tile Prefetcher Menu"),
        )
        .child(spinner(
            "Min zoom : ",
            menu.min,
            "plan-prefetch-min-down",
            "plan-prefetch-min-up",
            "plan-prefetch-min",
            true,
            cx,
        ))
        .child(spinner(
            "Max zoom : ",
            menu.max,
            "plan-prefetch-max-down",
            "plan-prefetch-max-up",
            "plan-prefetch-max",
            false,
            cx,
        ))
        .child(
            crate::probe::measured("plan-prefetch-lines", div())
                .flex()
                .flex_col()
                .max_h(px(200.0))
                .overflow_hidden()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .children(lines.into_iter().map(|line| div().child(line))),
        )
        .child(
            crate::probe::measured("plan-prefetch-total", div())
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(total),
        )
        .child(
            div()
                .flex()
                .justify_end()
                .gap_2()
                .child(action(
                    "plan-prefetch-ok",
                    "OK",
                    theme::ACCENT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        menu_ok(this);
                        cx.notify();
                    }),
                ))
                .child(action(
                    "plan-prefetch-menu-cancel",
                    "Cancel",
                    theme::TEXT,
                    true,
                    cx.listener(|this, _event: &(), _window, cx| {
                        this.plan_menus.prefetch_menu = None;
                        cx.notify();
                    }),
                )),
        );
    backdrop("plan-prefetch-menu-backdrop", form.into_any_element())
}

/// The prefetcher form while a job runs or has just ended: the progress line, the bar, the
/// cache's words, and Cancel.
/// `// C#: TilePrefetcher.Designer.cs`
pub(crate) fn job_form(job: &PrefetchJob, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let ended = job.finished() || job.cancelled();
    let form = crate::probe::measured("plan-prefetch", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(340.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            crate::probe::measured("plan-prefetch-label", div())
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .child(job.label()),
        )
        .child(crate::ui::progress(job.fraction(), theme::ACCENT))
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(job.saving()),
        )
        .child(div().flex().justify_end().child(action(
            "plan-prefetch-cancel",
            if ended { "Close" } else { "Cancel" },
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                if let Some(job) = this.plan_menus.prefetch.as_ref() {
                    job.cancel();
                }
                this.plan_menus.prefetch = None;
                cx.notify();
            }),
        )));
    backdrop("plan-prefetch-backdrop", form.into_any_element())
}

/// A form over the whole window, centred, taking the clicks around it as a modal does.
fn backdrop(id: &'static str, form: AnyElement) -> AnyElement {
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id(id)
                    .w(px(100_000.0))
                    .h(px(100_000.0))
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(form),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// The forms to draw, if any: the job's over the menu's.
pub(crate) fn overlays(
    menus: &crate::plan::PlanMenus,
    cx: &mut Context<MissionPlanner>,
) -> Vec<AnyElement> {
    let mut out = Vec::new();
    if let Some(job) = menus.prefetch.as_ref() {
        out.push(job_form(job, cx));
    } else if let Some(menu) = menus.prefetch_menu.as_ref() {
        out.push(menu_form(menu, cx));
    }
    out
}

/// Yes to "ripp displayed on screen?": the menu opens over the view's area.
/// `// C#: GCSViews/FlightPlanner.cs:5039-5041, 5046-5051`
pub(crate) fn open_menu_over_view(this: &mut MissionPlanner) {
    let corners = this.map.borrow().view_corners();
    if let Some((top_left, bottom_right)) = corners {
        let area = Area::between(top_left, bottom_right);
        if !area.is_empty() {
            this.plan_menus.prefetch_menu = Some(PrefetchMenu::new(area));
        }
    }
}

/// OK on the menu: a walk per zoom over the area.
/// `// C#: GCSViews/FlightPlanner.cs:5052-5075`
pub(crate) fn menu_ok(this: &mut MissionPlanner) {
    let Some(menu) = this.plan_menus.prefetch_menu.take() else {
        return;
    };
    let walks: Vec<(Area, u8)> = menu
        .zooms()
        .into_iter()
        .map(|zoom| (menu.area, zoom))
        .collect();
    start(this, walks);
}

/// Prefetch WP Path's answer: the walks over the planned path.
/// `// C#: GCSViews/FlightPlanner.cs:3305-3365`
pub(crate) fn start_path(this: &mut MissionPlanner, max_zoom: i32) {
    let mut points: Vec<LatLon> = crate::plan::planner_map_home(&this.plan)
        .into_iter()
        .collect();
    for item in this.plan.items() {
        if let Ok(Some(position)) = item.position() {
            points.push(position);
        }
    }
    let walks = path_walks(&points, max_zoom);
    if walks.is_empty() {
        return;
    }
    start(this, walks);
}

fn start(this: &mut MissionPlanner, walks: Vec<(Area, u8)>) {
    let Some(source) = this
        .tile_source_id()
        .and_then(mp_tiles::source::source_by_id)
    else {
        return;
    };
    let cache = TileCache::new(TileCache::default_root());
    let offline = this.map.borrow().tiles_offline();
    this.plan_menus.prefetch = Some(PrefetchJob::start(walks, source, cache, offline));
}

/// What a UI test asserts on.
pub(crate) fn record_facts(menus: &crate::plan::PlanMenus) {
    use crate::facts::record;
    record(
        "plan.prefetch.state",
        menus
            .prefetch
            .as_ref()
            .map_or("idle", PrefetchJob::state_name),
    );
    record(
        "plan.prefetch.label",
        menus
            .prefetch
            .as_ref()
            .map_or_else(String::new, PrefetchJob::label),
    );
    record(
        "plan.prefetch.ok",
        menus.prefetch.as_ref().map_or(0, PrefetchJob::ok),
    );
    record(
        "plan.prefetch.zoom",
        menus.prefetch.as_ref().map_or(0, PrefetchJob::zoom),
    );
    record(
        "plan.prefetch.walks",
        menus.prefetch.as_ref().map_or(0, |job| job.walks),
    );
    let menu = menus.prefetch_menu.as_ref();
    // "none" while closed: a script cannot expect an empty value.
    record(
        "plan.prefetch.menu",
        menu.map_or_else(|| "none".to_owned(), |menu| format!("{}-{}", menu.min, menu.max)),
    );
    record(
        "plan.prefetch.menu.total",
        menu.map_or_else(String::new, |menu| menu.text().1),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_menu_starts_at_one_to_twenty_and_keeps_min_under_max() {
        let area = Area::between(
            LatLon::new(-35.36, 149.16).expect("a"),
            LatLon::new(-35.37, 149.17).expect("b"),
        );
        let mut menu = PrefetchMenu::new(area);
        assert_eq!((menu.min, menu.max), (1, 20));
        menu.set_min(25);
        assert_eq!(menu.min, 1);
        menu.set_max(3);
        menu.set_min(5);
        assert_eq!((menu.min, menu.max), (1, 3));
        menu.set_min(3);
        menu.set_max(2);
        assert_eq!((menu.min, menu.max), (3, 3));
        menu.set_max(24);
        assert_eq!(menu.max, 24);
        menu.set_max(25);
        assert_eq!(menu.max, 24);
        assert_eq!(menu.zooms(), (3..=24).collect::<Vec<_>>());
    }

    #[test]
    fn the_path_walks_every_segment_at_every_zoom_up_to_the_lesser_maximum() {
        let points: Vec<LatLon> = [(-35.36, 149.16), (-35.37, 149.17), (-35.38, 149.16)]
            .into_iter()
            .map(|(lat, lng)| LatLon::new(lat, lng).expect("point"))
            .collect();
        let walks = path_walks(&points, 3);
        assert_eq!(walks.len(), 6);
        assert_eq!(walks[0].1, 1);
        assert_eq!(walks[5].1, 3);
        assert!((walks[0].0.top - -35.36).abs() < f64::EPSILON);
        // 30 is above the map's 24.
        assert_eq!(path_walks(&points, 30).len(), 48);
        assert!(path_walks(&points[..1], 5).is_empty());
        assert!(path_walks(&points, 0).is_empty());
    }

    #[test]
    fn an_offline_job_over_an_empty_cache_ends_with_nothing_and_says_so() {
        let dir = std::env::temp_dir().join(format!("mp-prefetch-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let cache = TileCache::new(&dir);
        let area = Area::between(
            LatLon::new(-35.36, 149.16).expect("a"),
            LatLon::new(-35.37, 149.17).expect("b"),
        );
        let job = PrefetchJob::start(
            vec![(area, 3), (area, 4)],
            mp_tiles::source::default_source(),
            cache,
            true,
        );
        let started = std::time::Instant::now();
        while !job.finished() && started.elapsed() < std::time::Duration::from_secs(5) {
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(job.finished());
        assert_eq!(job.ok(), 0);
        assert_eq!(job.zoom(), 4);
        assert_eq!(job.saving(), "all tiles saved");
        assert_eq!(job.state_name(), "done");
        assert!(job.label().starts_with("Fetching tile at zoom (4): 1 of 1"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
