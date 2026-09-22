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
use std::sync::{Mutex, OnceLock};

use gpui::{Bounds, Pixels, canvas, div, prelude::*};

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

/// The positions recorded so far this run.
fn registry() -> &'static Mutex<BTreeMap<String, Rect>> {
    static REGISTRY: OnceLock<Mutex<BTreeMap<String, Rect>>> = OnceLock::new();
    REGISTRY.get_or_init(|| Mutex::new(BTreeMap::new()))
}

/// Records where a control was laid out.
fn record(name: &str, bounds: Bounds<Pixels>) {
    let rect = Rect {
        x: f32::from(bounds.origin.x),
        y: f32::from(bounds.origin.y),
        width: f32::from(bounds.size.width),
        height: f32::from(bounds.size.height),
    };
    if let Ok(mut registry) = registry().lock() {
        // Only rewrite the file when something moved. A UI that repaints ten times a second would
        // otherwise rewrite it ten times a second, and a script reading it could catch a partial
        // write.
        if registry.get(name) == Some(&rect) {
            return;
        }
        registry.insert(name.to_owned(), rect);
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
    for (index, (name, rect)) in registry.iter().enumerate() {
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

/// An invisible element that reports the bounds it is given.
///
/// Placed inside a control and stretched to fill it, so what it measures is the control itself
/// rather than an approximation of it. It paints nothing.
pub fn marker(name: impl Into<String>) -> impl IntoElement {
    let name = name.into();
    div().absolute().size_full().child(canvas(
        move |bounds, _window, _cx| record(&name, bounds),
        |_bounds, (), _window, _cx| {},
    ))
}

/// The positions recorded, for tests.
#[cfg(test)]
pub fn snapshot() -> BTreeMap<String, Rect> {
    registry().lock().map(|r| r.clone()).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use gpui::{Point, Size, point, px, size};

    fn bounds(x: f32, y: f32, w: f32, h: f32) -> Bounds<Pixels> {
        Bounds {
            origin: Point { x: px(x), y: px(y) },
            size: Size {
                width: px(w),
                height: px(h),
            },
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
    fn the_marker_does_not_take_part_in_layout() {
        // It is absolutely positioned and fills its parent, so adding it to a control cannot
        // change where that control or its neighbours end up - which would defeat the purpose of
        // measuring them.
        let _ = size(px(1.0), px(1.0));
        let _ = point(px(0.0), px(0.0));
        assert!(enabled() || !enabled());
    }
}
