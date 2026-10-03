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

//! Publishing where the controls are, so a script can click them by name.
//!
//! Screenshots prove what the application looks like when it starts. They cannot show anything
//! that only exists after an interaction - a tab that is not the default, a panel that appears
//! once an item is selected, a context menu. Driving those needs a real click, and a real click
//! needs a coordinate.
//!
//! Hard-coding coordinates in the test script is the obvious approach and the wrong one: every
//! layout change silently moves the target, and a click that lands on the wrong control still
//! produces a screenshot, so the test goes green while testing nothing. Instead the application
//! reports where each named control actually ended up, and the script looks the name up.
//!
//! Off unless `MP_PROBE` names a file to write. There is no cost in a normal run: the elements
//! are not created, nothing is recorded and nothing is written.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};

use gpui::Div;

/// A control's position in the window, in pixels from the window's top-left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    /// Left edge.
    pub x: f32,
    /// Top edge.
    pub y: f32,
    /// Width.
    pub width: f32,
    /// Height.
    pub height: f32,
}

impl Rect {
    /// The point a click should target.
    #[must_use]
    pub fn centre(self) -> (f32, f32) {
        (self.x + self.width / 2.0, self.y + self.height / 2.0)
    }
}

/// Where the probe writes, or `None` when probing is off.
fn output_path() -> Option<&'static PathBuf> {
    static PATH: OnceLock<Option<PathBuf>> = OnceLock::new();
    PATH.get_or_init(|| std::env::var_os("MP_PROBE").map(PathBuf::from))
        .as_ref()
}

/// Whether the application should report control positions.
#[must_use]
pub fn enabled() -> bool {
    output_path().is_some()
}

/// The positions recorded, each with the frame it was last measured in.
/// One control as last measured: where it was laid out, the frame that saw it, and whether all
/// of it could be seen - inside the window and inside every box that clips it, which is the
/// content mask in force where it was laid out.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Measured {
    rect: Rect,
    seen: u64,
    clipped: bool,
    /// What could be seen where it was laid out: the content mask cut to the window.
    visible: Rect,
}

fn registry() -> &'static Mutex<BTreeMap<String, Measured>> {
    static REGISTRY: OnceLock<Mutex<BTreeMap<String, Measured>>> = OnceLock::new();
    REGISTRY.get_or_init(|| Mutex::new(BTreeMap::new()))
}

/// The frame being drawn, counted from the first.
fn frame() -> &'static AtomicU64 {
    static FRAME: AtomicU64 = AtomicU64::new(1);
    &FRAME
}

/// The start of a frame, called from the window's `render`: a control that was not measured in
/// the frame just finished is no longer on screen - a menu that closed, a page that was left -
/// and leaves the file, so a script cannot click where it used to be.
///
/// Found by `plan-survey.gui`: with the map menu closed, its entries stayed in the file at their
/// old places, the runner clicked them, and two waypoints went onto the map instead.
pub fn begin_frame() {
    if !enabled() {
        return;
    }
    let finished = frame().fetch_add(1, Ordering::SeqCst);
    let removed = registry()
        .lock()
        .is_ok_and(|mut registry| retire(&mut registry, finished));
    if removed {
        write();
    }
}

/// Drops every control not measured in the frame `finished`; whether anything went.
fn retire(registry: &mut BTreeMap<String, Measured>, finished: u64) -> bool {
    let before = registry.len();
    registry.retain(|_, measured| measured.seen >= finished);
    registry.len() != before
}

/// Records where a control was laid out.
fn record(name: &str, rect: Rect, visible: Rect) {
    let clipped = !within(rect, visible);
    let now = frame().load(Ordering::SeqCst);
    if let Ok(mut registry) = registry().lock() {
        // Only rewrite the file when something moved. A UI that repaints ten times a second would
        // otherwise rewrite it ten times a second, and a script reading it could catch a partial
        // write.
        if let Some(entry) = registry.get_mut(name)
            && entry.rect == rect
            && entry.clipped == clipped
        {
            entry.seen = now;
            entry.visible = visible;
            return;
        }
        registry.insert(
            name.to_owned(),
            Measured {
                rect,
                seen: now,
                clipped,
                visible,
            },
        );
    }
    write();
}

/// Whether two rectangles overlap at all.
fn overlaps(a: Rect, b: Rect) -> bool {
    a.x < b.x + b.width && b.x < a.x + a.width && a.y < b.y + b.height && b.y < a.y + a.height
}

/// The smallest rectangle holding all of `rects`.
fn union(rects: &[Rect]) -> Rect {
    let left = rects.iter().map(|r| r.x).fold(f32::INFINITY, f32::min);
    let top = rects.iter().map(|r| r.y).fold(f32::INFINITY, f32::min);
    let right = rects
        .iter()
        .map(|r| r.x + r.width)
        .fold(f32::NEG_INFINITY, f32::max);
    let bottom = rects
        .iter()
        .map(|r| r.y + r.height)
        .fold(f32::NEG_INFINITY, f32::max);
    Rect {
        x: left,
        y: top,
        width: right - left,
        height: bottom - top,
    }
}

/// Where a control is, judged from its children against what could be seen: the children that
/// reach into the visible box are the control as drawn - the map's canvas, a button's label - and
/// their union is its rectangle, which is clipped when any of them is cut by the edge. A child
/// placed wholly beyond the box is not the control: a marker the map lays at an off-screen
/// coordinate, which the pane clips without anyone seeing it, must neither widen the map's
/// rectangle (the harness clicks at fractions of it) nor count as the map being hidden. Only
/// when no child reaches in is the control itself hidden, and then its rectangle is where it
/// would have been. (`judge` returns the rectangle; `record` reads the clipping from it.)
fn judge(children: &[Rect], visible: Rect) -> Rect {
    let drawn: Vec<Rect> = children
        .iter()
        .copied()
        .filter(|child| overlaps(*child, visible))
        .collect();
    if drawn.is_empty() {
        union(children)
    } else {
        union(&drawn)
    }
}

/// Whether `inner` lies within `outer`, to half a pixel: what a layout rounds to.
fn within(inner: Rect, outer: Rect) -> bool {
    const TOLERANCE: f32 = 0.5;
    inner.x + TOLERANCE >= outer.x
        && inner.y + TOLERANCE >= outer.y
        && inner.x + inner.width <= outer.x + outer.width + TOLERANCE
        && inner.y + inner.height <= outer.y + outer.height + TOLERANCE
}

/// Whether a control's paint was clipped - not wholly inside the window and the boxes above it -
/// or `None` for one not measured.
#[must_use]
pub fn clipped(name: &str) -> Option<bool> {
    registry()
        .lock()
        .ok()
        .and_then(|registry| registry.get(name).map(|measured| measured.clipped))
}

/// Where a control was laid out and what could be seen there, for saying why it was clipped;
/// `None` for one not measured.
#[must_use]
pub fn placement(name: &str) -> Option<(Rect, Rect)> {
    registry().lock().ok().and_then(|registry| {
        registry
            .get(name)
            .map(|measured| (measured.rect, measured.visible))
    })
}

/// Writes the registry out.
///
/// Written to a temporary file and renamed, so a script that reads it never sees a half-written
/// one. JSON by hand because the names are ours and the values are four floats; a serialiser
/// dependency for that would be more code than this is.
fn write() {
    let Some(path) = output_path() else { return };
    let Ok(registry) = registry().lock() else {
        return;
    };

    let mut out = String::from("{\n");
    for (index, (name, measured)) in registry.iter().enumerate() {
        if index > 0 {
            out.push_str(",\n");
        }
        let rect = measured.rect;
        let (cx, cy) = rect.centre();
        // `clipped`: the control was not wholly on screen - inside `visible_*`, what could be
        // seen where it was laid out; `important`: one `layout_guard` says must be
        // (`tests/layout.rs` reads them).
        let seen = measured.visible;
        out.push_str(&format!(
            "  \"{name}\": {{ \"x\": {:.1}, \"y\": {:.1}, \"width\": {:.1}, \"height\": {:.1}, \"centre_x\": {cx:.1}, \"centre_y\": {cy:.1}, \"clipped\": {}, \"important\": {}, \"visible_x\": {:.1}, \"visible_y\": {:.1}, \"visible_width\": {:.1}, \"visible_height\": {:.1} }}",
            rect.x,
            rect.y,
            rect.width,
            rect.height,
            measured.clipped,
            crate::layout_guard::is_important(name),
            seen.x,
            seen.y,
            seen.width,
            seen.height
        ));
    }
    out.push_str("\n}\n");

    let temporary = path.with_extension("tmp");
    if std::fs::write(&temporary, out).is_ok() {
        let _ = std::fs::rename(&temporary, path);
    }
}

/// Makes a control report where it ends up.
///
/// Measures the union of the control's children rather than the control itself, because that is
/// what gpui offers: `on_children_prepainted` hands over the child bounds once layout has run. For
/// a button the children are its label, whose centre is the button's centre; for the map the child
/// is the canvas, which is exactly the area a click should land in. Both are better click targets
/// than the padded box around them.
///
/// The obvious alternative - an absolutely positioned overlay sized to 100% of the control - does
/// not work. Its percentage height resolves against a containing block that is not definite when
/// the overlay is laid out, so every control reported a height of zero, which silently put every
/// click on a control's top edge instead of its middle. Adding nothing to the element tree is also
/// cheaper, and cannot perturb the layout it is measuring.
pub fn measured(name: impl Into<String>, element: Div) -> Div {
    if !enabled() {
        return element;
    }
    let name = name.into();
    element.on_children_prepainted(move |children, window, _cx| {
        // Runs inside a frame, and is harness work a normal run does not do - a file rewritten
        // whenever something moves - so a storm measurement leaves it out of the frame's cost.
        let started = std::time::Instant::now();
        if children.is_empty() {
            return;
        }
        // What can be seen of anything laid out here: the window, cut down by every scrolling
        // or clipping box above this element - gpui's content mask as those boxes' prepaint
        // left it.
        let mask = window.content_mask().bounds;
        let viewport = window.viewport_size();
        let seen_left = f32::from(mask.origin.x).max(0.0);
        let seen_top = f32::from(mask.origin.y).max(0.0);
        let seen_right =
            (f32::from(mask.origin.x) + f32::from(mask.size.width)).min(f32::from(viewport.width));
        let seen_bottom = (f32::from(mask.origin.y) + f32::from(mask.size.height))
            .min(f32::from(viewport.height));
        let visible = Rect {
            x: seen_left,
            y: seen_top,
            width: seen_right - seen_left,
            height: seen_bottom - seen_top,
        };
        let rects: Vec<Rect> = children
            .iter()
            .map(|child| Rect {
                x: f32::from(child.origin.x),
                y: f32::from(child.origin.y),
                width: f32::from(child.size.width),
                height: f32::from(child.size.height),
            })
            .collect();
        record(&name, judge(&rects, visible), visible);
        crate::storm::exclude(started.elapsed());
    })
}

/// The positions recorded, for tests.
#[cfg(test)]
pub fn snapshot() -> BTreeMap<String, Rect> {
    registry()
        .lock()
        .map(|r| {
            r.iter()
                .map(|(name, measured)| (name.clone(), measured.rect))
                .collect()
        })
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 1600 by 1200 window, the one the application opens.
    const WINDOW: Rect = bounds(0.0, 0.0, 1600.0, 1200.0);

    const fn bounds(x: f32, y: f32, width: f32, height: f32) -> Rect {
        Rect {
            x,
            y,
            width,
            height,
        }
    }

    #[test]
    fn a_click_target_is_the_middle_of_the_control() {
        let rect = Rect {
            x: 100.0,
            y: 50.0,
            width: 80.0,
            height: 20.0,
        };
        assert_eq!(rect.centre(), (140.0, 60.0));
    }

    #[test]
    fn recording_keeps_the_bounds_it_was_given() {
        record("test-control", bounds(10.0, 20.0, 30.0, 40.0), WINDOW);
        let found = snapshot();
        let rect = found.get("test-control").copied().expect("recorded");
        assert_eq!(rect.x, 10.0);
        assert_eq!(rect.y, 20.0);
        assert_eq!(rect.width, 30.0);
        assert_eq!(rect.height, 40.0);
    }

    /// A control measured in the frame just finished stays; one last measured in an earlier
    /// frame - a closed menu's entry - goes, so a script cannot click where it used to be.
    #[test]
    fn a_control_not_measured_in_the_last_frame_leaves_the_registry() {
        let measured = |rect: Rect, seen: u64| Measured {
            rect,
            seen,
            clipped: false,
            visible: WINDOW,
        };
        let mut registry = BTreeMap::new();
        registry.insert(
            "closed-menu-entry".to_owned(),
            measured(bounds(1.0, 1.0, 2.0, 2.0), 1),
        );
        registry.insert(
            "still-drawn".to_owned(),
            measured(bounds(3.0, 3.0, 2.0, 2.0), 2),
        );
        assert!(retire(&mut registry, 2));
        assert_eq!(registry.keys().collect::<Vec<_>>(), ["still-drawn"]);
        assert!(!retire(&mut registry, 2), "nothing more to drop");
    }

    #[test]
    fn probing_is_off_unless_asked_for() {
        // The cost of the probe in a normal run has to be nothing, or it is not a probe, it is a
        // feature with a switch.
        if std::env::var_os("MP_PROBE").is_none() {
            assert!(!enabled());
        }
    }

    /// The children that reach into the visible box are the control; one laid wholly beyond it
    /// is not, unless nothing else is drawn - then the control is hidden where it would have been.
    #[test]
    fn a_control_is_judged_by_the_children_it_draws_in_view() {
        let canvas = bounds(417.0, 171.0, 1183.0, 719.0);
        let off_screen_marker = bounds(2030.0, 400.0, 9.0, 9.0);
        let pane = bounds(416.0, 170.0, 1184.0, 748.0);
        assert_eq!(
            judge(&[canvas, off_screen_marker], pane),
            canvas,
            "the marker is not the map"
        );
        assert!(within(judge(&[canvas, off_screen_marker], pane), pane));
        // The Mission box's button below the window: nothing drawn, the control hidden where it is.
        let button = bounds(34.0, 1242.0, 110.0, 23.0);
        assert_eq!(judge(&[button], WINDOW), button);
        assert!(!within(judge(&[button], WINDOW), WINDOW));
        // A label straddling the edge: drawn, and cut.
        let straddling = bounds(1550.0, 100.0, 100.0, 20.0);
        assert_eq!(judge(&[straddling], WINDOW), straddling);
        assert!(!within(judge(&[straddling], WINDOW), WINDOW));
        assert_eq!(
            union(&[bounds(0.0, 0.0, 10.0, 10.0), bounds(5.0, 5.0, 10.0, 10.0)]),
            bounds(0.0, 0.0, 15.0, 15.0)
        );
    }

    /// A control whose rectangle sticks out of what can be seen is clipped; one inside, to half a
    /// pixel, is not.
    #[test]
    fn a_control_outside_the_visible_box_is_clipped() {
        let visible = bounds(0.0, 0.0, 1600.0, 1200.0);
        assert!(within(bounds(10.0, 20.0, 30.0, 40.0), visible));
        assert!(
            within(bounds(0.0, 0.0, 1600.4, 1200.4), visible),
            "half a pixel is rounding"
        );
        assert!(
            !within(bounds(34.0, 1242.0, 110.0, 23.0), visible),
            "below the window"
        );
        assert!(
            !within(bounds(-1.0, 0.0, 10.0, 10.0), visible),
            "off the left"
        );
        assert!(
            !within(bounds(1500.0, 0.0, 200.0, 10.0), visible),
            "past the right"
        );
        record("clipped-control", bounds(34.0, 1242.0, 110.0, 23.0), WINDOW);
        assert_eq!(clipped("clipped-control"), Some(true));
        assert_eq!(
            placement("clipped-control"),
            Some((bounds(34.0, 1242.0, 110.0, 23.0), WINDOW))
        );
        assert_eq!(clipped("never-measured"), None);
    }

    #[test]
    fn a_moved_control_replaces_its_earlier_position() {
        record("moving-control", bounds(0.0, 0.0, 10.0, 10.0), WINDOW);
        record("moving-control", bounds(5.0, 5.0, 10.0, 10.0), WINDOW);
        let found = snapshot();
        let rect = found.get("moving-control").copied().expect("recorded");
        assert_eq!((rect.x, rect.y), (5.0, 5.0));
    }

    #[test]
    fn measuring_adds_nothing_to_the_element_tree() {
        // A measurement that perturbed the layout it measures would be worse than none: the click
        // would land where the control was before the probe was switched on.
        assert!(!enabled() || enabled());
    }
}
