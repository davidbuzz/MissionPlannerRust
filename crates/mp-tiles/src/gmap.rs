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

//! The arithmetic GMap.NET's providers put into their tile URLs.
//!
//! Three small functions, each ported line for line, because each one decides which bytes a server
//! is asked for: the host a tile goes to, the "secure word" Google's servers expect after the zoom,
//! and the quadkey Bing names its tiles by. None of them is a formula a provider documents; they
//! are what GMap.NET sends, and the servers answer them.

/// The language every URL carries, `GMapProvider.LanguageStr`.
///
/// The same constant the cache's directory is named by, for the same reason: it starts as `"en"`
/// and nothing in Mission Planner assigns `GMapProvider.Language`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:350-357`
pub const LANGUAGE: &str = crate::cache::LANGUAGE;

/// `GMapProvider.GetServerNum`: which of `max` numbered hosts a tile goes to.
///
/// `(int)(pos.X + 2 * pos.Y) % max`. Chosen from the tile, so the same tile always goes to the
/// same host. `max` of zero has no servers to choose from and answers zero, where the C# would
/// divide by zero; no provider ported here passes it.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/GMapProvider.cs:463-466`
#[must_use]
pub fn server_num(x: u32, y: u32, max: u8) -> u64 {
    if max == 0 {
        return 0;
    }
    // Summed as u64: at the grid's deepest zoom 2y is below 2^23, so nothing here wraps where the
    // C#'s long arithmetic would not.
    (u64::from(x) + 2 * u64::from(y)) % u64::from(max)
}

/// `GoogleMapProviderBase.SecureWord`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:37`
const SECURE_WORD: &str = "Galileo";

/// `GoogleMapProviderBase.Sec1`, which goes after `&x=` for some rows.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:252`
const SEC1: &str = "&s=";

/// `GoogleMapProviderBase.GetSecureWords`: the two fragments Google's tile URLs carry.
///
/// `sec1` is `&s=` for rows 10000 to 99999 and empty otherwise; it goes straight after the x
/// coordinate, which is why such a URL reads `&x=59922&s=&y=39658`. `sec2` is the first
/// `(3x + y) % 8` letters of "Galileo" and goes after the final `&s=`. A remainder of 7 takes the
/// whole word; there is no eighth letter to want.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Google/GoogleMapProvider.cs:240-250`
#[must_use]
pub fn secure_words(x: u32, y: u32) -> (&'static str, &'static str) {
    let length = (3 * u64::from(x) + u64::from(y)) % 8;
    let sec2 = usize::try_from(length)
        .ok()
        .and_then(|length| SECURE_WORD.get(..length))
        .unwrap_or(SECURE_WORD);
    let sec1 = if (10_000..100_000).contains(&y) {
        SEC1
    } else {
        ""
    };
    (sec1, sec2)
}

/// `BingMapProviderBase.TileXYToQuadKey`: a tile's quadkey, one digit per zoom level.
///
/// Each digit is the tile's quadrant at that level: x's bit adds one, y's bit adds two. Zoom 0 is
/// the empty string, and the C# asks for exactly that - `tiles/a.jpeg`.
/// `// C#: ExtLibs/GMap.NET.Core/GMap.NET.MapProviders/Bing/BingMapProvider.cs:42-61`
#[must_use]
pub fn quad_key(x: u32, y: u32, zoom: u8) -> String {
    (1..=zoom)
        .rev()
        .map(|level| {
            let mask = 1_u64 << (level - 1);
            let mut digit = b'0';
            if u64::from(x) & mask != 0 {
                digit += 1;
            }
            if u64::from(y) & mask != 0 {
                digit += 2;
            }
            char::from(digit)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_server_is_x_plus_twice_y_modulo_the_count() {
        // (x + 2y) % 4, worked by hand: (59922 + 79316) % 4 = 139238 % 4 = 2.
        assert_eq!(server_num(59_922, 39_658, 4), 2);
        assert_eq!(server_num(0, 0, 4), 0);
        // (511 + 680) % 4 = 1191 % 4 = 3.
        assert_eq!(server_num(511, 340, 4), 3);
        // Not x + y, which is what the rotation in this crate's own providers uses: (1 + 1) % 4
        // would be 2, and GMap.NET's is (1 + 2) % 4 = 3.
        assert_eq!(server_num(1, 1, 4), 3);
        assert_eq!(server_num(5, 5, 0), 0);
    }

    #[test]
    fn the_secure_word_is_a_prefix_of_galileo() {
        // (3*59922 + 39658) % 8 = 219424 % 8 = 0: nothing after the final &s=.
        assert_eq!(secure_words(59_922, 39_658).1, "");
        // (3*1 + 2) % 8 = 5: "Galil".
        assert_eq!(secure_words(1, 2).1, "Galil");
        // (3*15089 + 9814) % 8 = 55081 % 8 = 1: "G".
        assert_eq!(secure_words(15_089, 9_814).1, "G");
        // A remainder of 7 is the whole word.
        assert_eq!(secure_words(2, 1).1, "Galileo");
    }

    #[test]
    fn sec1_is_only_for_five_digit_rows() {
        assert_eq!(secure_words(0, 9_999).0, "");
        assert_eq!(secure_words(0, 10_000).0, "&s=");
        assert_eq!(secure_words(0, 99_999).0, "&s=");
        assert_eq!(secure_words(0, 100_000).0, "");
    }

    #[test]
    fn a_quadkey_interleaves_the_bits_from_the_top() {
        // Microsoft's own example: tile (3, 5) at level 3 is "213".
        assert_eq!(quad_key(3, 5, 3), "213");
        assert_eq!(quad_key(0, 0, 0), "");
        assert_eq!(quad_key(1, 0, 1), "1");
        assert_eq!(quad_key(0, 1, 1), "2");
        assert_eq!(quad_key(1, 1, 1), "3");
        // One digit per level, all the way down the grid.
        assert_eq!(quad_key(0, 0, 22).len(), 22);
    }
}
