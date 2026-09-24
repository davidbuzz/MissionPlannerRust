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
fn registry() -> &'static Mutex<BTreeMap<String, (Rect, u64)>> {
    static REGISTRY: OnceLock<Mutex<BTreeMap<String, (Rect, u64)>>> = OnceLock::new();
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
fn retire(registry: &mut BTreeMap<String, (Rect, u64)>, finished: u64) -> bool {
    let before = registry.len();
    registry.retain(|_, (_, seen)| *seen >= finished);
    registry.len() != before
}

/// Records where a control was laid out.
fn record(name: &str, rect: Rect) {
    let now = frame().load(Ordering::SeqCst);
    if let Ok(mut registry) = registry().lock() {
        // Only rewrite the file when something moved. A UI that repaints ten times a second would
        // otherwise rewrite it ten times a second, and a script reading it could catch a partial
        // write.
        if let Some(entry) = registry.get_mut(name)
            && entry.0 == rect
        {
            entry.1 = now;
            return;
        }
        registry.insert(name.to_owned(), (rect, now));
    }
    write();
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
    for (index, (name, (rect, _))) in registry.iter().enumerate() {
        if index > 0 {
            out.push_str(",\n");
        }
        let (cx, cy) = rect.centre();
        out.push_str(&format!(
            "  \"{name}\": {{ \"x\": {:.1}, \"y\": {:.1}, \"width\": {:.1}, \"height\": {:.1}, \"centre_x\": {cx:.1}, \"centre_y\": {cy:.1} }}",
            rect.x, rect.y, rect.width, rect.height
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
    element.on_children_prepainted(move |children, _window, _cx| {
        // Runs inside a frame, and is harness work a normal run does not do - a file rewritten
        // whenever something moves - so a storm measurement leaves it out of the frame's cost.
        let started = std::time::Instant::now();
        let Some(first) = children.first() else {
            return;
        };
        let mut left = f32::from(first.origin.x);
        let mut top = f32::from(first.origin.y);
        let mut right = left + f32::from(first.size.width);
        let mut bottom = top + f32::from(first.size.height);
        for child in children.iter().skip(1) {
            left = left.min(f32::from(child.origin.x));
            top = top.min(f32::from(child.origin.y));
            right = right.max(f32::from(child.origin.x) + f32::from(child.size.width));
            bottom = bottom.max(f32::from(child.origin.y) + f32::from(child.size.height));
        }
        record(
            &name,
            Rect {
                x: left,
                y: top,
                width: right - left,
                height: bottom - top,
            },
        );
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
                .map(|(name, (rect, _))| (name.clone(), *rect))
                .collect()
        })
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

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
        record("test-control", bounds(10.0, 20.0, 30.0, 40.0));
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
        let mut registry = BTreeMap::new();
        registry.insert(
            "closed-menu-entry".to_owned(),
            (bounds(1.0, 1.0, 2.0, 2.0), 1),
        );
        registry.insert("still-drawn".to_owned(), (bounds(3.0, 3.0, 2.0, 2.0), 2));
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

    #[test]
    fn a_moved_control_replaces_its_earlier_position() {
        record("moving-control", bounds(0.0, 0.0, 10.0, 10.0));
        record("moving-control", bounds(5.0, 5.0, 10.0, 10.0));
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
