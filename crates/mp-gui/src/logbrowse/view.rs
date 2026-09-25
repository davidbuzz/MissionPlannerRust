//! What part of the chart is on screen: ZedGraph's zoom, pan and zoom stack, as `zg1` has them.
//!
//! `LogBrowse` leaves `zg1` with ZedGraph's defaults for these (the designer sets only
//! `IsSynchronizeYAxes`, which matters with more than one pane, and the scroll ranges, whose bars
//! are hidden): a left drag draws a rectangle and zooms every axis to it, Ctrl with a left drag or
//! a middle drag pans every axis, the wheel zooms every axis by a tenth about the middle of its
//! range, and the context menu's Un-Zoom, Undo All Zoom/Pan and Set Scale to Default walk back
//! the stack each of those pushes onto. Every one of them ends in `ZoomEvent`, which
//! `zg1_ZoomEvent` answers by redrawing the labels and the map. Adding a curve zooms out all
//! the way (`ZoomOutAll`), which is where a new log's first plot starts from.
//!
//! No gpui here: the screen hands over where the pointer is as a fraction of the plotting area.
//! `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:393-480, 839-1060, 1232-1330;
//! ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.ContextMenu.cs:130-200, 600-800;
//! ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.cs:172-244; Log/LogBrowse.cs:1685-1690`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;

use mp_chart::Range;

/// `ZoomStepFraction`: a wheel notch changes each range by a tenth.
pub const ZOOM_STEP: f64 = 0.1;

/// A drag must cover more than this many pixels each way to zoom: `HandleZoomFinish`.
pub const DRAG_MINIMUM: f32 = 4.0;

/// How near a point must be to the pointer to be shown: `GraphPane.Default.NearestTol`.
pub const NEAREST_TOLERANCE: f64 = 7.0;

/// The ranges every axis shows.
#[derive(Debug, Clone, PartialEq)]
pub struct Scales {
    /// The x axis.
    pub x: (f64, f64),
    /// Each left axis, by unit.
    pub left: BTreeMap<String, Range>,
    /// The right axis.
    pub right: Option<Range>,
}

impl Scales {
    /// Every y range, left then right, for a change applied to all of them.
    fn each_y(&mut self, mut change: impl FnMut(&mut Range)) {
        for range in self.left.values_mut() {
            change(range);
        }
        if let Some(range) = self.right.as_mut() {
            change(range);
        }
    }
}

/// What pushed a state onto the zoom stack: `ZoomState.StateType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// A rectangle dragged, or Set Scale to Default.
    Zoom,
    /// The wheel.
    WheelZoom,
    /// A pan.
    Pan,
}

impl Kind {
    /// The context menu's first undo item, named for what it undoes.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.ContextMenu.cs:160-178;
    /// ExtLibs/ZedGraph/ZedGraph/ZedGraphLocale.resx`
    #[must_use]
    pub const fn undo_text(self) -> &'static str {
        match self {
            Self::Zoom | Self::WheelZoom => "Un-Zoom",
            Self::Pan => "Un-Pan",
        }
    }
}

/// The chart's zoom: the ranges shown, if the user has chosen any, and how to get back.
///
/// `None` is ZedGraph's automatic scaling - every axis fitted to what is plotted - which is where
/// a chart starts and where Set Scale to Default returns it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Zoom {
    /// The ranges shown, or `None` for automatic.
    current: Option<Scales>,
    /// `ZoomStack`: what each zoom replaced, the latest last.
    stack: Vec<(Option<Scales>, Kind)>,
    /// A pan under way: the ranges it started from, and the pointer's last place.
    panning: Option<(Option<Scales>, (f64, f64))>,
}

impl Zoom {
    /// The ranges shown, given what automatic scaling would show.
    #[must_use]
    pub fn scales(&self, automatic: Scales) -> Scales {
        self.current.clone().unwrap_or(automatic)
    }

    /// Whether the user has zoomed or panned.
    #[must_use]
    pub const fn is_zoomed(&self) -> bool {
        self.current.is_some()
    }

    /// What the context menu's first undo item would undo, if anything.
    #[must_use]
    pub fn top(&self) -> Option<Kind> {
        self.stack.last().map(|(_, kind)| *kind)
    }

    /// How deep the stack is.
    #[must_use]
    pub fn depth(&self) -> usize {
        self.stack.len()
    }

    /// `ZoomOutAll` after a curve is added, and a cleared chart: automatic scaling, no stack.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// A rectangle dragged from one place to another, each a fraction of the plotting area
    /// across and down, on an area `width` by `height` pixels. Smaller than five pixels either
    /// way it is a click, and nothing changes; otherwise every axis shows what the rectangle
    /// covered. True when it zoomed.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:1232-1330`
    pub fn drag_zoom(
        &mut self,
        automatic: Scales,
        from: (f64, f64),
        to: (f64, f64),
        size: (f32, f32),
    ) -> bool {
        let (width, height) = (f64::from(size.0), f64::from(size.1));
        let (from, to) = (clamp_unit(from), clamp_unit(to));
        if (to.0 - from.0).abs() * width <= f64::from(DRAG_MINIMUM)
            || (to.1 - from.1).abs() * height <= f64::from(DRAG_MINIMUM)
        {
            return false;
        }
        let before = self.current.clone();
        let mut scales = self.scales(automatic);
        let at = |range: (f64, f64), fraction: f64| fraction.mul_add(range.1 - range.0, range.0);
        let (x1, x2) = (at(scales.x, from.0), at(scales.x, to.0));
        scales.x = (x1.min(x2), x1.max(x2));
        // Down the screen is down the axis.
        scales.each_y(|range| {
            let y1 = (1.0 - from.1).mul_add(range.high - range.low, range.low);
            let y2 = (1.0 - to.1).mul_add(range.high - range.low, range.low);
            *range = Range {
                low: y1.min(y2),
                high: y1.max(y2),
            };
        });
        self.stack.push((before, Kind::Zoom));
        self.current = Some(scales);
        true
    }

    /// A wheel notch: towards the user zooms out, away zooms in, every axis about the middle of
    /// its range, as `IsZoomOnMouseCenter` is false.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:839-876, 900-1000`
    pub fn wheel(&mut self, automatic: Scales, towards_user: bool) {
        let fraction = 1.0 + if towards_user { 1.0 } else { -1.0 } * ZOOM_STEP;
        let before = self.current.clone();
        let mut scales = self.scales(automatic);
        let scale = |low: f64, high: f64| {
            let centre = f64::midpoint(low, high);
            let half = (high - low) * fraction / 2.0;
            (centre - half, centre + half)
        };
        scales.x = scale(scales.x.0, scales.x.1);
        scales.each_y(|range| {
            let (low, high) = scale(range.low, range.high);
            *range = Range { low, high };
        });
        self.stack.push((before, Kind::WheelZoom));
        self.current = Some(scales);
    }

    /// Ctrl and a left press, or a middle press: a pan starts where the pointer is.
    pub fn begin_pan(&mut self, at: (f64, f64)) {
        self.panning = Some((self.current.clone(), at));
    }

    /// Whether a pan is under way.
    #[must_use]
    pub const fn is_panning(&self) -> bool {
        self.panning.is_some()
    }

    /// The pointer moved during a pan: every axis moves with it.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:1004-1050`
    pub fn pan_to(&mut self, automatic: Scales, at: (f64, f64)) {
        let Some((_, last)) = self.panning.as_mut() else {
            return;
        };
        let (dx, dy) = (last.0 - at.0, last.1 - at.1);
        *last = at;
        let mut scales = self.scales(automatic);
        let shift_x = dx * (scales.x.1 - scales.x.0);
        scales.x = (scales.x.0 + shift_x, scales.x.1 + shift_x);
        scales.each_y(|range| {
            // Down the screen is down the axis, so dragging down moves the range up.
            let shift = -dy * (range.high - range.low);
            *range = Range {
                low: range.low + shift,
                high: range.high + shift,
            };
        });
        self.current = Some(scales);
    }

    /// The pan's button came up: if anything moved, what it started from goes on the stack.
    /// True when it pushed, which is when `ZoomEvent` is raised.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:1052-1068`
    pub fn end_pan(&mut self) -> bool {
        let Some((before, _)) = self.panning.take() else {
            return false;
        };
        if before == self.current {
            return false;
        }
        self.stack.push((before, Kind::Pan));
        true
    }

    /// Un-Zoom, Un-Pan: back one step. False, and nothing done, with an empty stack - the item
    /// is disabled then.
    pub fn undo(&mut self) -> bool {
        let Some((before, _)) = self.stack.pop() else {
            return false;
        };
        self.current = before;
        true
    }

    /// Undo All Zoom/Pan: back to what was shown before the first step.
    pub fn undo_all(&mut self) -> bool {
        if self.stack.is_empty() {
            return false;
        }
        let first = self.stack.swap_remove(0);
        self.stack.clear();
        self.current = first.0;
        true
    }

    /// `GoToSample`'s `movegraph`: the x axis centred on a value, its span kept - set straight on
    /// the scale, with nothing pushed and no `ZoomEvent`.
    /// `// C#: Log/LogBrowse.cs:3497-3503`
    pub fn centre_x(&mut self, shown: Scales, x: f64) {
        let mut scales = shown;
        let half = (scales.x.1 - scales.x.0) / 2.0;
        scales.x = (x - half, x + half);
        self.current = Some(scales);
    }

    /// Set Scale to Default: automatic scaling again, the old ranges pushed so Un-Zoom brings
    /// them back.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.ContextMenu.cs:638-700`
    pub fn set_default(&mut self) {
        let before = self.current.take();
        self.stack.push((before, Kind::Zoom));
    }
}

fn clamp_unit((x, y): (f64, f64)) -> (f64, f64) {
    (x.clamp(0.0, 1.0), y.clamp(0.0, 1.0))
}

/// One curve as the point search sees it: its samples, their time order, and the y range it is
/// drawn against.
#[derive(Debug, Clone, Copy)]
pub struct Curve<'a> {
    /// The samples, `(x, y)`.
    pub series: &'a mp_chart::Series,
    /// The samples' order along x, built with the series.
    pub order: &'a TimeOrder,
    /// The range of the axis it is on.
    pub range: Range,
}

/// A series' samples in order along x, so the point search reads only those within reach of the
/// pointer instead of every sample on every frame.
///
/// A log's samples are in time order but for a clock that restarts part way through, and in line
/// order always; so this is nothing - the series' own order - unless the series is out of order,
/// when it is the samples sorted, built once when the curve is plotted. A sample whose x is not a
/// number is in no range, so it is left out.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TimeOrder {
    /// The samples' indices, least x first; `None` when that is the series' own order.
    sorted: Option<Vec<usize>>,
}

impl TimeOrder {
    /// The order of `series`: checked in one pass, and sorted only when it has to be.
    #[must_use]
    pub fn of(series: &mp_chart::Series) -> Self {
        let mut previous = f64::NEG_INFINITY;
        let in_order = series.samples().all(|sample| {
            let ok = sample.at >= previous;
            previous = sample.at;
            ok
        });
        if in_order {
            return Self { sorted: None };
        }
        let mut sorted: Vec<usize> = series
            .samples()
            .enumerate()
            .filter(|(_, sample)| !sample.at.is_nan())
            .map(|(index, _)| index)
            .collect();
        let at = |index: &usize| series.get(*index).map_or(f64::NAN, |sample| sample.at);
        sorted.sort_by(|a, b| at(a).total_cmp(&at(b)));
        Self {
            sorted: Some(sorted),
        }
    }

    /// How many samples are in the order.
    fn len(&self, series: &mp_chart::Series) -> usize {
        self.sorted.as_ref().map_or(series.len(), Vec::len)
    }

    /// The index in the series of the `position`-th sample along x.
    fn index(&self, position: usize) -> usize {
        self.sorted
            .as_ref()
            .map_or(position, |sorted| sorted.get(position).copied().unwrap_or(usize::MAX))
    }

    /// The positions along x of the samples with `low <= x <= high`: two binary searches.
    fn within(&self, series: &mp_chart::Series, low: f64, high: f64) -> std::ops::Range<usize> {
        let at = |position: usize| {
            series
                .get(self.index(position))
                .map_or(f64::NAN, |sample| sample.at)
        };
        let len = self.len(series);
        let first = partition_point(len, |position| at(position) < low);
        let end = partition_point(len, |position| at(position) <= high);
        first..end.max(first)
    }
}

/// The first of `0..len` for which `before` is false, `before` being true then false along it.
fn partition_point(len: usize, before: impl Fn(usize) -> bool) -> usize {
    let (mut low, mut high) = (0, len);
    while low < high {
        let middle = low + (high - low) / 2;
        if before(middle) {
            low = middle + 1;
        } else {
            high = middle;
        }
    }
    low
}

/// The point nearest the pointer, within `NearestTol` pixels: `FindNearestPoint`.
///
/// The pointer is a fraction of the plotting area across and down; the area is `width` by
/// `height` pixels. Only points inside every range count. Returns the curve's index and the
/// point.
///
/// ZedGraph measures every point of every curve; this measures only the points within a pixel
/// more than `NearestTol` of the pointer across, found through each curve's [`TimeOrder`] - a
/// point further across is further than `NearestTol` whatever its height, so it could never be
/// the answer. The answer is the same, ties included: the first curve's point, and within a curve
/// the earliest sample, as ZedGraph's `dist >= minDist` keeps the first it met.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/GraphPane.cs:2019-2180`
#[must_use]
pub fn nearest_point(
    curves: &[Curve<'_>],
    x: (f64, f64),
    pointer: (f64, f64),
    size: (f32, f32),
) -> Option<(usize, mp_chart::Sample)> {
    search(curves, x, pointer, size, &mut 0)
}

/// [`nearest_point`], counting in `measured` the points it measured.
fn search(
    curves: &[Curve<'_>],
    x: (f64, f64),
    pointer: (f64, f64),
    size: (f32, f32),
    measured: &mut usize,
) -> Option<(usize, mp_chart::Sample)> {
    let (width, height) = (f64::from(size.0), f64::from(size.1));
    let x_span = x.1 - x.0;
    if !(x_span > 0.0 && width > 0.0 && height > 0.0) {
        return None;
    }
    let pointer_x = pointer.0.mul_add(x_span, x.0);
    // A pixel's margin over the tolerance, so rounding cannot leave out a point just inside it.
    let reach = (NEAREST_TOLERANCE + 1.0) * x_span / width;
    // (distance, curve, sample index, sample)
    let mut best: Option<(f64, usize, usize, mp_chart::Sample)> = None;
    for (curve_index, curve) in curves.iter().enumerate() {
        let y_span = curve.range.high - curve.range.low;
        if y_span <= 0.0 {
            continue;
        }
        let pointer_y = (1.0 - pointer.1).mul_add(y_span, curve.range.low);
        let low = (pointer_x - reach).max(x.0);
        let high = (pointer_x + reach).min(x.1);
        if low > high {
            continue;
        }
        for position in curve.order.within(curve.series, low, high) {
            let index = curve.order.index(position);
            let Some(sample) = curve.series.get(index) else {
                continue;
            };
            *measured += 1;
            if !(x.0..=x.1).contains(&sample.at)
                || !(curve.range.low..=curve.range.high).contains(&sample.value)
            {
                continue;
            }
            let dx = (sample.at - pointer_x) * width / x_span;
            let dy = (sample.value - pointer_y) * height / y_span;
            let distance = dx.mul_add(dx, dy * dy);
            let nearer = best.is_none_or(|(least, held_curve, held_index, _)| {
                distance < least
                    || (distance == least && held_curve == curve_index && index < held_index)
            });
            if nearer {
                best = Some((distance, curve_index, index, *sample));
            }
        }
    }
    best.filter(|(distance, ..)| *distance < NEAREST_TOLERANCE * NEAREST_TOLERANCE)
        .map(|(_, index, _, sample)| (index, sample))
}

/// The tooltip ZedGraph shows for a point: `"( " + x + ", " + y + " )"`, each written by
/// `MakeValueLabel` - a date axis in `PointDateFormat`, which `chk_time` sets to
/// `HH:mm:ss.fff`, and a number in `PointValueFormat`, `"G"`: .NET's general format, fifteen
/// significant digits for a double.
/// `// C#: ExtLibs/ZedGraph/ZedGraph/ZedGraphControl.Events.cs:628-656, 758-772;
/// ExtLibs/ZedGraph/ZedGraph/PointPairBase.cs:53; Log/LogBrowse.cs:3547`
#[must_use]
pub fn point_text(x: &str, y: f64) -> String {
    format!("( {x}, {} )", mp_log::netfmt::double(y))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn automatic() -> Scales {
        Scales {
            x: (0.0, 100.0),
            left: BTreeMap::from([(
                "deg".to_owned(),
                Range {
                    low: -10.0,
                    high: 10.0,
                },
            )]),
            right: Some(Range {
                low: 0.0,
                high: 1000.0,
            }),
        }
    }

    /// A rectangle zooms every axis to what it covered; a click does nothing.
    #[test]
    fn a_dragged_rectangle_zooms_every_axis() {
        let mut zoom = Zoom::default();
        // Four pixels across is a click; five would zoom.
        assert!(!zoom.drag_zoom(automatic(), (0.5, 0.5), (0.5075, 0.9), (400.0, 200.0)));
        assert!(!zoom.is_zoomed());
        assert!(zoom.drag_zoom(automatic(), (0.75, 0.75), (0.25, 0.25), (400.0, 200.0)));
        let scales = zoom.scales(automatic());
        assert_eq!(scales.x, (25.0, 75.0));
        assert_eq!(
            scales.left["deg"],
            Range {
                low: -5.0,
                high: 5.0
            }
        );
        assert_eq!(
            scales.right,
            Some(Range {
                low: 250.0,
                high: 750.0
            })
        );
        assert_eq!(zoom.top(), Some(Kind::Zoom));
        assert_eq!(Kind::Zoom.undo_text(), "Un-Zoom");
    }

    /// The wheel zooms by a tenth about the middle; towards the user is out.
    #[test]
    fn the_wheel_zooms_about_the_middle() {
        let mut zoom = Zoom::default();
        zoom.wheel(automatic(), false);
        let scales = zoom.scales(automatic());
        assert!((scales.x.0 - 5.0).abs() < 1e-9 && (scales.x.1 - 95.0).abs() < 1e-9);
        zoom.wheel(automatic(), true);
        let scales = zoom.scales(automatic());
        assert!((scales.x.0 - 0.5).abs() < 1e-9 && (scales.x.1 - 99.5).abs() < 1e-9);
        assert_eq!(zoom.depth(), 2);
        assert_eq!(zoom.top(), Some(Kind::WheelZoom));
    }

    /// A pan moves every axis with the pointer, and goes on the stack once, when it ends.
    #[test]
    fn a_pan_moves_every_axis_and_is_one_step() {
        let mut zoom = Zoom::default();
        zoom.begin_pan((0.5, 0.5));
        zoom.pan_to(automatic(), (0.4, 0.6));
        zoom.pan_to(automatic(), (0.3, 0.6));
        let scales = zoom.scales(automatic());
        assert!((scales.x.0 - 20.0).abs() < 1e-9, "{scales:?}");
        // Dragged down a tenth: the degrees axis shows two degrees higher.
        assert!((scales.left["deg"].low + 8.0).abs() < 1e-9, "{scales:?}");
        assert!(zoom.end_pan());
        assert_eq!(zoom.depth(), 1);
        assert_eq!(zoom.top().map(Kind::undo_text), Some("Un-Pan"));
        // A pan that went nowhere pushes nothing.
        zoom.begin_pan((0.5, 0.5));
        assert!(!zoom.end_pan());
        assert_eq!(zoom.depth(), 1);
    }

    /// Un-Zoom goes back a step, Undo All to the start, Set Scale to Default to automatic.
    #[test]
    fn the_menu_walks_back_the_stack() {
        let mut zoom = Zoom::default();
        assert!(!zoom.undo());
        assert!(!zoom.undo_all());
        zoom.wheel(automatic(), false);
        let first = zoom.scales(automatic());
        zoom.wheel(automatic(), false);
        assert!(zoom.undo());
        assert_eq!(zoom.scales(automatic()), first);
        zoom.wheel(automatic(), false);
        assert!(zoom.undo_all());
        assert!(!zoom.is_zoomed());
        assert_eq!(zoom.depth(), 0);
        zoom.wheel(automatic(), false);
        zoom.set_default();
        assert!(!zoom.is_zoomed());
        assert_eq!(zoom.depth(), 2);
        assert!(zoom.undo());
        assert!(
            zoom.is_zoomed(),
            "Un-Zoom brings back what Set Scale to Default replaced"
        );
        zoom.reset();
        assert_eq!(zoom, Zoom::default());
    }

    /// The nearest point within seven pixels, on the axis its curve is drawn against.
    #[test]
    fn the_nearest_point_is_within_seven_pixels() {
        let mut near = mp_chart::Series::new("a", 8);
        near.push(10.0, 0.0);
        near.push(50.0, 5.0);
        let mut far = mp_chart::Series::new("b", 8);
        far.push(52.0, 900.0);
        let (near_order, far_order) = (TimeOrder::of(&near), TimeOrder::of(&far));
        let curves = [
            Curve {
                series: &near,
                order: &near_order,
                range: automatic().left["deg"],
            },
            Curve {
                series: &far,
                order: &far_order,
                range: automatic().right.unwrap_or(Range {
                    low: 0.0,
                    high: 1.0,
                }),
            },
        ];
        // 50, 5 on the degrees axis is at (0.5, 0.25).
        let found = nearest_point(&curves, (0.0, 100.0), (0.505, 0.26), (400.0, 200.0));
        assert_eq!(
            found.map(|(index, sample)| (index, sample.at)),
            Some((0, 50.0))
        );
        // 52, 900 on the right axis is at (0.52, 0.1).
        let found = nearest_point(&curves, (0.0, 100.0), (0.52, 0.11), (400.0, 200.0));
        assert_eq!(found.map(|(index, _)| index), Some(1));
        // Nothing within seven pixels.
        assert_eq!(
            nearest_point(&curves, (0.0, 100.0), (0.3, 0.5), (400.0, 200.0)),
            None
        );
    }

    /// ZedGraph's walk, every point of every curve, as `nearest_point` was: what the search
    /// through the time order must answer.
    /// `// C#: ExtLibs/ZedGraph/ZedGraph/GraphPane.cs:2089-2170`
    fn walk(
        curves: &[Curve<'_>],
        x: (f64, f64),
        pointer: (f64, f64),
        size: (f32, f32),
    ) -> Option<(usize, mp_chart::Sample)> {
        let (width, height) = (f64::from(size.0), f64::from(size.1));
        let x_span = x.1 - x.0;
        let pointer_x = pointer.0.mul_add(x_span, x.0);
        let mut best: Option<(f64, usize, mp_chart::Sample)> = None;
        for (index, curve) in curves.iter().enumerate() {
            let y_span = curve.range.high - curve.range.low;
            if y_span <= 0.0 {
                continue;
            }
            let pointer_y = (1.0 - pointer.1).mul_add(y_span, curve.range.low);
            for sample in curve.series.samples() {
                if !(x.0..=x.1).contains(&sample.at)
                    || !(curve.range.low..=curve.range.high).contains(&sample.value)
                {
                    continue;
                }
                let dx = (sample.at - pointer_x) * width / x_span;
                let dy = (sample.value - pointer_y) * height / y_span;
                let distance = dx.mul_add(dx, dy * dy);
                if best.is_none_or(|(least, _, _)| distance < least) {
                    best = Some((distance, index, *sample));
                }
            }
        }
        best.filter(|(distance, _, _)| *distance < NEAREST_TOLERANCE * NEAREST_TOLERANCE)
            .map(|(_, index, sample)| (index, sample))
    }

    /// A deterministic scatter of numbers in 0..1, so the test needs no random crate.
    fn scatter(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        #[allow(clippy::cast_precision_loss)] // 53 bits of a 64-bit state
        let fraction = (*seed >> 11) as f64 / (1_u64 << 53) as f64;
        fraction
    }

    /// Two large curves - one in time order, one whose clock restarts twice, with repeated times
    /// and repeated values for ties - and the search answers what the walk answers for a
    /// nine hundred pointers, over the whole log and zoomed in.
    #[test]
    fn the_search_answers_what_the_walk_answers() {
        let mut seed = 7;
        let mut ordered = mp_chart::Series::new("ordered", 60_000);
        for index in 0..60_000_u32 {
            let value = (scatter(&mut seed) * 20.0).round();
            ordered.push(f64::from(index / 2) * 0.01, value);
        }
        let mut restarted = mp_chart::Series::new("restarted", 45_000);
        for index in 0..45_000_u32 {
            let at = f64::from(index % 18_000) * 0.02;
            restarted.push(at, scatter(&mut seed) * 20.0);
        }
        let orders = [TimeOrder::of(&ordered), TimeOrder::of(&restarted)];
        assert!(orders[0].sorted.is_none());
        assert!(orders[1].sorted.is_some());
        let range = Range {
            low: 0.0,
            high: 20.0,
        };
        let curves = [
            Curve {
                series: &ordered,
                order: &orders[0],
                range,
            },
            Curve {
                series: &restarted,
                order: &orders[1],
                range,
            },
        ];
        let mut found = 0;
        for x in [(0.0, 400.0), (150.0, 151.0), (-5.0, 3.0)] {
            for _ in 0..300 {
                let pointer = (scatter(&mut seed), scatter(&mut seed));
                let ours = nearest_point(&curves, x, pointer, (800.0, 300.0));
                assert_eq!(ours, walk(&curves, x, pointer, (800.0, 300.0)), "{x:?} {pointer:?}");
                found += usize::from(ours.is_some());
            }
        }
        assert!(found > 300, "the pointers find points: {found}");
    }

    /// What it costs: the points within reach across, not the log. A million samples over 800
    /// pixels is 1250 a pixel; the search measures the sixteen pixels around the pointer.
    #[test]
    fn the_search_measures_only_the_points_within_reach() {
        let mut series = mp_chart::Series::new("big", 1_000_000);
        for index in 0..1_000_000_u32 {
            series.push(f64::from(index) * 0.001, f64::from(index % 100));
        }
        let order = TimeOrder::of(&series);
        let curves = [Curve {
            series: &series,
            order: &order,
            range: Range {
                low: 0.0,
                high: 100.0,
            },
        }];
        let mut measured = 0;
        let found = search(&curves, (0.0, 1_000.0), (0.5, 0.5), (800.0, 300.0), &mut measured);
        assert_eq!(found, walk(&curves, (0.0, 1_000.0), (0.5, 0.5), (800.0, 300.0)));
        assert!(found.is_some());
        // Eight pixels either side, 1.25 x-units a pixel, samples a thousandth apart: about
        // 20,001 of the million.
        assert!((19_999..=20_001).contains(&measured), "{measured}");

        // Zoomed in to a second, the reach is a hundredth of a second: 21 samples.
        let mut measured = 0;
        search(&curves, (500.0, 501.0), (0.5, 0.5), (800.0, 300.0), &mut measured);
        assert!((19..=22).contains(&measured), "{measured}");
    }

    /// The order checked and built: a series in order needs none; a restart sorts it, leaving
    /// out a time that is not a number; the window is inclusive at both ends.
    #[test]
    fn the_time_order_is_the_series_sorted() {
        let mut series = mp_chart::Series::new("s", 8);
        for at in [0.0, 1.0, 1.0, 2.0] {
            series.push(at, 1.0);
        }
        let order = TimeOrder::of(&series);
        assert_eq!(order, TimeOrder::default());
        assert_eq!(order.within(&series, 1.0, 1.0), 1..3);
        assert_eq!(order.within(&series, 2.5, 3.0), 4..4);

        // `Series::push` drops a sample whose time is not a number (mp-chart's index cannot
        // place it), so the NaN never reaches the order; four samples remain, out of order.
        let mut series = mp_chart::Series::new("s", 8);
        for at in [5.0, 6.0, f64::NAN, 1.0, 2.0] {
            series.push(at, 1.0);
        }
        assert_eq!(series.len(), 4);
        let order = TimeOrder::of(&series);
        assert_eq!(order.sorted, Some(vec![2, 3, 0, 1]));
        let within: Vec<usize> = order
            .within(&series, 2.0, 5.0)
            .map(|position| order.index(position))
            .collect();
        assert_eq!(within, [3, 0]);
    }

    /// `( x, y )`, the value in .NET's general format.
    #[test]
    fn the_tooltip_is_zedgraphs() {
        assert_eq!(point_text("12:34:56.789", 1.5), "( 12:34:56.789, 1.5 )");
        assert_eq!(point_text("1200", 0.1 + 0.2), "( 1200, 0.3 )");
        assert_eq!(point_text("7", -1234.0), "( 7, -1234 )");
    }
}
