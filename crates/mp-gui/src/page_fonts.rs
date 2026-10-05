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

//! The fonts a web page draws the planner's text with. A page has no system fonts for gpui to
//! find, so the browser build brings its own (as Zed's web examples do): IBM Plex Sans (OFL), and
//! for the symbols the planner draws that Plex has not got - the spinners' and lists' triangles,
//! the check boxes, the close cross, the warning sign - a subset of DejaVu Sans
//! (fonts/LICENSE-DejaVu.txt). gpui's page text system falls back to any loaded font that has a
//! glyph the first lacks; before the subset, each of those drew as an empty box (the browser
//! build's odds-and-ends row, "the spinners' arrow glyphs").

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

/// The fonts, in the order gpui is given them.
#[cfg(any(target_family = "wasm", test))]
pub const FONTS: [&[u8]; 3] = [
    include_bytes!("../fonts/IBMPlexSans-Regular.ttf"),
    include_bytes!("../fonts/IBMPlexSans-SemiBold.ttf"),
    include_bytes!("../fonts/DejaVuSans-Symbols.ttf"),
];

#[cfg(test)]
mod tests {
    use super::*;

    /// Characters in the planner's source that no page font has, and why each may stay so: both
    /// are inputs tests give a parser to refuse, never drawn (and gpui's page would draw an emoji
    /// through the browser's own fonts, its Canvas fallback).
    const NOT_DRAWN: &[(char, &str)] = &[
        (
            '\u{1f642}',
            "an emoji a test gives plan.rs's altitude-frame parser to refuse, never drawn",
        ),
        (
            '\u{ff11}',
            "a fullwidth digit a test gives serial_ports.rs's parser to refuse, never drawn",
        ),
    ];

    /// Every character outside ASCII in a string literal of the planner's source, with a file it
    /// is in - a rough scan (comment lines left out), which is all a test of coverage needs.
    fn drawn_characters() -> std::collections::BTreeMap<char, String> {
        let mut found = std::collections::BTreeMap::new();
        let mut folders = vec![std::path::PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/src"
        ))];
        while let Some(folder) = folders.pop() {
            for entry in mp_os::fs::read_dir(&folder).into_iter().flatten().flatten() {
                let path = entry.path();
                if path.is_dir() {
                    folders.push(path);
                    continue;
                }
                if path.extension().is_none_or(|kind| kind != "rs") {
                    continue;
                }
                let Ok(text) = mp_os::fs::read_to_string(&path) else {
                    continue;
                };
                for line in text.lines() {
                    if line.trim_start().starts_with("//") {
                        continue;
                    }
                    for literal in line.split('"').skip(1).step_by(2) {
                        for character in unescaped(literal).chars().filter(|c| !c.is_ascii()) {
                            found
                                .entry(character)
                                .or_insert_with(|| path.display().to_string());
                        }
                    }
                }
            }
        }
        found
    }

    /// A literal's `\u{...}` escapes as their characters.
    fn unescaped(literal: &str) -> String {
        let mut out = String::new();
        let mut rest = literal;
        while let Some(at) = rest.find("\\u{") {
            out.push_str(&rest[..at]);
            let after = &rest[at + 3..];
            match after.find('}').and_then(|end| {
                u32::from_str_radix(&after[..end], 16)
                    .ok()
                    .and_then(char::from_u32)
                    .map(|c| (c, end))
            }) {
                Some((character, end)) => {
                    out.push(character);
                    rest = &after[end + 1..];
                }
                None => {
                    out.push_str("\\u{");
                    rest = after;
                }
            }
        }
        out.push_str(rest);
        out
    }

    #[test]
    fn the_page_has_a_glyph_for_every_character_the_planner_draws() {
        let faces: Vec<ttf_parser::Face<'_>> = FONTS
            .iter()
            .map(|font| ttf_parser::Face::parse(font, 0).expect("a font"))
            .collect();
        let drawn = drawn_characters();
        assert!(
            drawn.contains_key(&'\u{25b2}'),
            "the scan finds the spinners' triangle"
        );
        let missing: Vec<String> = drawn
            .iter()
            .filter(|(character, _)| {
                !faces
                    .iter()
                    .any(|face| face.glyph_index(**character).is_some())
            })
            .filter(|(character, _)| !NOT_DRAWN.iter().any(|(c, _)| c == *character))
            .map(|(character, file)| {
                format!("U+{:04X} {character} in {file}", u32::from(*character))
            })
            .collect();
        assert!(
            missing.is_empty(),
            "no page font draws: {missing:?} - add it to fonts/DejaVuSans-Symbols.ttf"
        );
    }

    #[test]
    fn the_symbols_are_dejavus_and_plex_has_not_got_them() {
        let plex = ttf_parser::Face::parse(FONTS[0], 0).expect("Plex");
        let symbols = ttf_parser::Face::parse(FONTS[2], 0).expect("the symbols");
        for character in ['\u{25b2}', '\u{25bc}', '\u{2610}', '\u{2611}', '\u{2715}'] {
            assert!(plex.glyph_index(character).is_none(), "{character}");
            assert!(symbols.glyph_index(character).is_some(), "{character}");
        }
    }
}
