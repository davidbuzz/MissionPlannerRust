//! The clicks a user makes, printed on the command line as the demo's recipe lines - `screen fly`,
//! `click plan-zoomicon`, `click map@0.40x0.40`, `click map@0.50x0.50:right` - under `--demo` or
//! `--record` (the owner's word of 2026-10-17: "have the command line print click activity when
//! --demo or --record"), so a sequence he made by hand, copied from the terminal into
//! `tools/demo/owner.gui`, plays in his demo. The control is the innermost one the probe measured
//! under the pointer (probe.rs); the map carries the point's fractions of its box, as
//! tools/gui-test.sh and the tour click it; the right button is `:right`. Not in the C#.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};

use gpui::{MouseButton, MouseDownEvent};

use crate::probe::Rect;

static ON: AtomicBool = AtomicBool::new(false);

/// Starts printing: `--demo` or `--record`. The probe measures from now on, file or no file.
pub(crate) fn enable() {
    ON.store(true, Ordering::Relaxed);
    crate::probe::enable();
}

/// Whether clicks are printed.
#[must_use]
pub(crate) fn enabled() -> bool {
    ON.load(Ordering::Relaxed)
}

/// The innermost measured control holding the point - the smallest box among those that do -
/// and the point's fractions of that box.
#[must_use]
pub(crate) fn innermost(
    rects: &BTreeMap<String, Rect>,
    x: f32,
    y: f32,
) -> Option<(String, f32, f32)> {
    rects
        .iter()
        .filter(|(_, r)| {
            r.width > 0.0
                && r.height > 0.0
                && x >= r.x
                && x <= r.x + r.width
                && y >= r.y
                && y <= r.y + r.height
        })
        .min_by(|(_, a), (_, b)| {
            (a.width * a.height)
                .partial_cmp(&(b.width * b.height))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(name, r)| (name.clone(), (x - r.x) / r.width, (y - r.y) / r.height))
}

/// The recipe line for a click: the map with its fractions, any other control by name alone,
/// `:right` for the right button.
#[must_use]
pub(crate) fn line_for(name: &str, fx: f32, fy: f32, right: bool) -> String {
    let at = if name == "map" || name.ends_with("-map") {
        format!("@{fx:.2}x{fy:.2}")
    } else {
        String::new()
    };
    let button = if right { ":right" } else { "" };
    format!("click {name}{at}{button}")
}

/// A press in the window: printed as its line, and the fact `ui.clicked` for the scripts.
pub(crate) fn note(event: &MouseDownEvent) {
    if !enabled() {
        return;
    }
    let right = match event.button {
        MouseButton::Left => false,
        MouseButton::Right => true,
        _ => return,
    };
    let (x, y) = (f32::from(event.position.x), f32::from(event.position.y));
    if let Some((name, fx, fy)) = innermost(&crate::probe::snapshot(), x, y) {
        let line = line_for(&name, fx, fy, right);
        println!("{line}");
        crate::facts::record("ui.clicked", &line[6..]);
    }
}

/// A screen chosen: its line.
pub(crate) fn screen(label: &str) {
    if enabled() {
        println!("screen {label}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x: f32, y: f32, width: f32, height: f32) -> Rect {
        Rect {
            x,
            y,
            width,
            height,
        }
    }

    /// The innermost box wins: a button inside its panel inside the window.
    #[test]
    fn the_innermost_control_under_the_point_is_named() {
        let mut rects = BTreeMap::new();
        rects.insert("panel:x".to_owned(), rect(0.0, 0.0, 800.0, 600.0));
        rects.insert("tab-plan".to_owned(), rect(100.0, 10.0, 60.0, 20.0));
        rects.insert("map".to_owned(), rect(0.0, 100.0, 800.0, 500.0));
        let (name, fx, fy) = innermost(&rects, 130.0, 20.0).expect("a control");
        assert_eq!(name, "tab-plan");
        assert!((fx - 0.5).abs() < 1e-6 && (fy - 0.5).abs() < 1e-6);
        let (name, fx, fy) = innermost(&rects, 200.0, 350.0).expect("the map");
        assert_eq!(name, "map");
        assert!((fx - 0.25).abs() < 1e-6 && (fy - 0.5).abs() < 1e-6);
        assert_eq!(innermost(&rects, 900.0, 20.0), None, "outside everything");
        assert_eq!(innermost(&BTreeMap::new(), 1.0, 1.0), None);
    }

    /// The lines are the runner's: a name alone, the map with fractions, the right button.
    #[test]
    fn the_lines_are_the_scripts_own_words() {
        assert_eq!(line_for("tab-plan", 0.3, 0.7, false), "click tab-plan");
        assert_eq!(line_for("map", 0.4, 0.4, false), "click map@0.40x0.40");
        assert_eq!(line_for("map", 0.5, 0.5, true), "click map@0.50x0.50:right");
        assert_eq!(
            line_for("plan-zoomicon", 0.1, 0.9, true),
            "click plan-zoomicon:right"
        );
        assert!(!enabled(), "off until the command line says so");
    }
}
