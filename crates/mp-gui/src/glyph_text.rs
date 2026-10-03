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

//! The font behind the planner's Text entry: `new Font("1CamBam_Stick_3", size * 1.35f)` and
//! `GraphicsPath.AddString(text, family, style, emSize, PointF(0, 0), StringFormat)`, read here
//! from the font file itself with `ttf-parser`.
//! `// C#: GCSViews/FlightPlanner.cs:6839-6858`
//!
//! **Where this differs from GDI+, and why.** GDI+ lays the string out with the font's own metrics
//! and, on Windows, falls back to Microsoft Sans Serif when `1CamBam_Stick_3` is not installed;
//! mono's libgdiplus lays it out with cairo and falls back to whatever fontconfig gives. So the
//! C# itself gives different waypoints on different machines. This port asks fontconfig for the
//! family the C# names (`fc-match`, which answers with its fallback when the family is missing) -
//! on Windows, which has no fontconfig, the installed font's file or GDI+'s fallback, Microsoft
//! Sans Serif - and lays the glyphs out itself: each at the pen, advanced by the glyph's horizontal advance,
//! no kerning (GDI+ applies none by default), the baseline `ascender` below the top, everything
//! scaled by `emSize / unitsPerEm`. GDI+'s layout padding and hinting are not reproduced.

use std::path::{Path, PathBuf};

use mp_mission::text_mission::Segment;

/// `"1CamBam_Stick_3"`, the family the handler asks for.
pub(crate) const FAMILY: &str = "1CamBam_Stick_3";

/// Why the outline could not be made.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum FontError {
    /// fontconfig gave no file for the family, or the file could not be read.
    NoFont(String),
    /// The file is not a font `ttf-parser` reads.
    NotAFont,
}

impl std::fmt::Display for FontError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoFont(why) => write!(f, "no font for {FAMILY}: {why}"),
            Self::NotAFont => write!(f, "the file fontconfig gave for {FAMILY} is not a font"),
        }
    }
}

impl std::error::Error for FontError {}

/// The font file for [`FAMILY`]: the font itself where it is installed, a fallback where it is
/// not - as GDI+ falls back to a default family. fontconfig's answer, or on Windows
/// [`windows_font_file`]'s.
///
/// # Errors
///
/// `fc-match` is not there or gives nothing; on Windows, neither font is installed.
pub(crate) fn font_file() -> Result<PathBuf, FontError> {
    if cfg!(windows) {
        let folder = |name: &str| std::env::var_os(name).map(PathBuf::from);
        return windows_font_file(
            folder("WINDIR").as_deref(),
            folder("LOCALAPPDATA").as_deref(),
            Path::is_file,
        );
    }
    let output = std::process::Command::new("fc-match")
        .args(["-f", "%{file}", FAMILY])
        .output()
        .map_err(|err| FontError::NoFont(err.to_string()))?;
    let path = String::from_utf8_lossy(&output.stdout).trim().to_owned();
    if path.is_empty() {
        return Err(FontError::NoFont("fc-match gave no file".to_owned()));
    }
    Ok(PathBuf::from(path))
}

/// Where GDI+ finds [`FAMILY`] on Windows: its file among the machine's fonts, then among the
/// user's (`%WINDIR%\Fonts`, `%LOCALAPPDATA%\Microsoft\Windows\Fonts`) - CamBam's stick fonts
/// install as `1CamBam_Stick_<n>.ttf` - else Microsoft Sans Serif (`micross.ttf`), the family GDI+
/// gives for one that is not installed. Windows has no fontconfig: `fc-match` failed there and the
/// Text entry said "Error" (the Windows GUI suite's `plan-text.gui`, 2026-09-26).
///
/// # Errors
///
/// Neither file is there.
fn windows_font_file(
    windows: Option<&Path>,
    local_app_data: Option<&Path>,
    exists: impl Fn(&Path) -> bool,
) -> Result<PathBuf, FontError> {
    let file = format!("{FAMILY}.ttf");
    let machine = windows.map(|folder| folder.join("Fonts"));
    let user = local_app_data.map(|folder| folder.join("Microsoft").join("Windows").join("Fonts"));
    let fallback = machine.as_ref().map(|fonts| fonts.join("micross.ttf"));
    [machine.map(|fonts| fonts.join(&file)), user.map(|fonts| fonts.join(&file)), fallback]
        .into_iter()
        .flatten()
        .find(|path| exists(path))
        .ok_or_else(|| {
            FontError::NoFont(format!("neither {file} nor Microsoft Sans Serif is installed"))
        })
}

/// The outline of `text` in a font's bytes at `em_size` units an em, laid out from (0, 0) at the
/// top left with y down, as `AddString` builds a path: every glyph's contours in order, each
/// glyph advanced by its horizontal advance.
///
/// # Errors
///
/// The bytes are not a font.
pub(crate) fn outline(font: &[u8], text: &str, em_size: f64) -> Result<Vec<Segment>, FontError> {
    let face = ttf_parser::Face::parse(font, 0).map_err(|_| FontError::NotAFont)?;
    let scale = em_size / f64::from(face.units_per_em());
    let baseline = f64::from(face.ascender()) * scale;
    let mut pen = 0.0;
    let mut segments = Vec::new();
    for character in text.chars() {
        let Some(glyph) = face.glyph_index(character) else {
            // No glyph: GDI+ draws the font's missing-glyph box; the advance is what moves on.
            pen += f64::from(face.glyph_hor_advance(ttf_parser::GlyphId(0)).unwrap_or(0)) * scale;
            continue;
        };
        let mut builder = Builder {
            segments: &mut segments,
            scale,
            pen,
            baseline,
        };
        face.outline_glyph(glyph, &mut builder);
        pen += f64::from(face.glyph_hor_advance(glyph).unwrap_or(0)) * scale;
    }
    Ok(segments)
}

/// Collects a glyph's contours, scaled and placed: font units are y up from the baseline, the
/// path is y down from the top.
struct Builder<'a> {
    segments: &'a mut Vec<Segment>,
    scale: f64,
    pen: f64,
    baseline: f64,
}

impl Builder<'_> {
    fn at(&self, x: f32, y: f32) -> (f64, f64) {
        (
            self.pen + f64::from(x) * self.scale,
            self.baseline - f64::from(y) * self.scale,
        )
    }
}

impl ttf_parser::OutlineBuilder for Builder<'_> {
    fn move_to(&mut self, x: f32, y: f32) {
        let (x, y) = self.at(x, y);
        self.segments.push(Segment::MoveTo(x, y));
    }

    fn line_to(&mut self, x: f32, y: f32) {
        let (x, y) = self.at(x, y);
        self.segments.push(Segment::LineTo(x, y));
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        self.segments.push(Segment::QuadTo {
            control: self.at(x1, y1),
            end: self.at(x, y),
        });
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        self.segments.push(Segment::CurveTo {
            control1: self.at(x1, y1),
            control2: self.at(x2, y2),
            end: self.at(x, y),
        });
    }

    fn close(&mut self) {
        self.segments.push(Segment::Close);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// On Windows the font's own file, the machine's before the user's, and Microsoft Sans Serif
    /// where it is installed in neither - GDI+'s fallback; nothing at all is an error, not a panic.
    #[test]
    fn windows_finds_the_installed_font_or_gdi_plus_fallback() {
        let windows = Path::new("C:/Windows");
        let local = Path::new("C:/Users/u/AppData/Local");
        let machine = windows.join("Fonts").join("1CamBam_Stick_3.ttf");
        let user = local
            .join("Microsoft")
            .join("Windows")
            .join("Fonts")
            .join("1CamBam_Stick_3.ttf");
        let sans = windows.join("Fonts").join("micross.ttf");
        let with = |present: Vec<PathBuf>| {
            windows_font_file(Some(windows), Some(local), move |path| {
                present.iter().any(|one| one == path)
            })
        };
        assert_eq!(
            with(vec![machine.clone(), user.clone(), sans.clone()]),
            Ok(machine)
        );
        assert_eq!(with(vec![user.clone(), sans.clone()]), Ok(user));
        assert_eq!(with(vec![sans.clone()]), Ok(sans));
        assert!(matches!(with(Vec::new()), Err(FontError::NoFont(_))));
        assert!(windows_font_file(None, None, |_| true).is_err());
    }

    /// Whatever fontconfig answers, it is a font this reader takes, and a glyph has contours.
    #[test]
    fn the_family_fontconfig_gives_outlines_a_letter() {
        let Ok(path) = font_file() else {
            eprintln!("no fontconfig here; skipped");
            return;
        };
        let bytes = std::fs::read(&path).expect("the font file fontconfig named");
        let segments = outline(&bytes, "I", 10.0 * 1.35).expect("a font");
        assert!(
            segments
                .iter()
                .any(|segment| matches!(segment, Segment::MoveTo(..))),
            "an I has a contour: {segments:?}"
        );
        let points = mp_mission::text_mission::path_points(&segments);
        // Every point of a 13.5-unit I sits inside its em box, y down from the top.
        assert!(
            points
                .iter()
                .all(|(x, y)| *x >= -1.0 && *x < 13.5 && *y >= -1.0 && *y <= 20.0),
            "{points:?}"
        );
        // Two letters: the second starts to the right of the first.
        let two =
            mp_mission::text_mission::path_points(&outline(&bytes, "II", 13.5).expect("a font"));
        let first_max = points.iter().map(|(x, _)| *x).fold(f64::MIN, f64::max);
        assert!(two.iter().any(|(x, _)| *x > first_max));
        assert!(outline(b"not a font", "I", 13.5).is_err());
    }
}
