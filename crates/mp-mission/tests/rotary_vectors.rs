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

//! Rotary surveys against Mission Planner's own: PLAN.md §13.4 item 8, DELIVERABLES.md Deliverable 11.
//!
//! Every file in `testdata/grid/golden/rotary` is what `Grid.CreateRotary` returned under mono for
//! one `rotary` line of `testdata/grid/cases.txt`, with the arguments it was called with recorded
//! above the points (`tools/csharp-reference/regen-grid.sh`, and `testdata/grid/README.md` for
//! how). Each case is re-run through [`create_rotary`] - and so through the port of ClipperLib's
//! offset and union it rests on - with those arguments and must match **bit for bit**: the same
//! number of points, in the same order, with the same tags, and every latitude, longitude and
//! altitude the same double.
//!
//! A case that can match only to within rounding goes in [`CLASS_C`] with its reason, and is held
//! to PLAN.md §7.2 class C instead (1e-7 degrees). None does.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic
)]

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use mp_mission::grid::{GridPoint, GridTag, StartPosition};
use mp_mission::rotary::{RotaryArgs, create_rotary};
use mp_units::LatLon;

/// Cases held to class C rather than bit for bit, each with the reason.
///
/// Empty: every case matches the C# to the bit. Add a case here only with the rounding it
/// differs by, named, and never to make a failure go away.
const CLASS_C: &[(&str, &str)] = &[];

/// §7.2 class C: the per-point tolerance.
const DEGREES: f64 = 1e-7;

fn grid_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/grid")
}

/// One golden file: the case's inputs as `CreateRotary` received them, and what it returned.
struct Golden {
    name: String,
    polygon: Vec<LatLon>,
    args: RotaryArgs,
    points: Vec<GridPoint>,
    /// Every argument line, verbatim, for comparing two cases' arguments.
    arguments: Vec<String>,
}

fn parse_f64(value: &str, file: &str) -> f64 {
    value
        .parse()
        .unwrap_or_else(|_| panic!("{file}: {value} is not a number"))
}

fn parse_lat_lng(values: &[&str], file: &str) -> LatLon {
    let [lat, lng] = values else {
        panic!("{file}: expected lat,lng, got {values:?}");
    };
    LatLon::new(parse_f64(lat, file), parse_f64(lng, file)).unwrap()
}

fn parse_bool(value: &str, file: &str) -> bool {
    match value {
        "True" => true,
        "False" => false,
        _ => panic!("{file}: {value} is not a C# bool"),
    }
}

fn read_golden(path: &Path) -> Golden {
    let file = path.display().to_string();
    let text = std::fs::read_to_string(path).unwrap();

    let mut name = None;
    let mut polygon = Vec::new();
    let mut args = RotaryArgs::default();
    let mut declared = None;
    let mut points = Vec::new();
    let mut arguments = Vec::new();
    let mut seen = BTreeSet::new();

    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let mut fields = line.split(',');
        let key = fields.next().unwrap();
        let values: Vec<&str> = fields.collect();
        if key != "vertex" && key != "wp" {
            assert!(seen.insert(key.to_owned()), "{file}: {key} given twice");
        }
        if !matches!(key, "case" | "vertex" | "points" | "wp") {
            arguments.push(line.to_owned());
        }
        let one = || -> &str {
            assert_eq!(values.len(), 1, "{file}: {key} takes one value");
            values[0]
        };
        match key {
            "case" => name = Some(one().to_owned()),
            "vertex" => polygon.push(parse_lat_lng(&values, &file)),
            "altitude" => args.altitude = parse_f64(one(), &file),
            "distance" => args.distance = parse_f64(one(), &file),
            "startpos" => {
                args.startpos = StartPosition::from_name(one())
                    .unwrap_or_else(|| panic!("{file}: no start position {}", one()));
            }
            "HomeLocation" => args.home = parse_lat_lng(&values, &file),
            "clockwise_laps" => args.clockwise_laps = one().parse().unwrap(),
            "match_spiral_perimeter" => args.match_spiral_perimeter = parse_bool(one(), &file),
            "laps" => args.laps = one().parse().unwrap(),
            "StartPointLatLngAlt" => args.start_point = parse_lat_lng(&values, &file),
            // CreateRotary takes these and never reads them - spacing it overwrites with 0
            // (Grid.cs:198); RotaryArgs has no field for them.
            "spacing" | "angle" | "overshoot1" | "overshoot2" => {
                parse_f64(one(), &file);
            }
            "minLaneSeparation" | "leadin" => {
                one().parse::<f32>().unwrap();
            }
            "shutter" => {
                parse_bool(one(), &file);
            }
            "points" => declared = Some(one().parse::<usize>().unwrap()),
            "wp" => {
                let [lat, lng, alt, tag] = values[..] else {
                    panic!("{file}: wp takes lat,lng,alt,tag");
                };
                points.push(GridPoint {
                    lat: parse_f64(lat, &file),
                    lng: parse_f64(lng, &file),
                    alt: parse_f64(alt, &file),
                    tag: GridTag::from_str_cs(tag)
                        .unwrap_or_else(|| panic!("{file}: no grid tag {tag}")),
                });
            }
            _ => panic!("{file}: unknown key {key}"),
        }
    }

    // Every argument must have been recorded: a missing line would silently take this crate's
    // default instead of the one the C# ran with.
    for key in [
        "case",
        "altitude",
        "distance",
        "spacing",
        "angle",
        "overshoot1",
        "overshoot2",
        "startpos",
        "shutter",
        "minLaneSeparation",
        "leadin",
        "HomeLocation",
        "clockwise_laps",
        "match_spiral_perimeter",
        "laps",
        "StartPointLatLngAlt",
        "points",
    ] {
        assert!(seen.contains(key), "{file}: no {key}");
    }
    assert_eq!(declared, Some(points.len()), "{file}: truncated");
    let name = name.unwrap();
    assert_eq!(
        path.file_stem().and_then(|s| s.to_str()),
        Some(name.as_str()),
        "{file}: named for another case"
    );
    Golden {
        name,
        polygon,
        args,
        points,
        arguments,
    }
}

fn goldens() -> Vec<Golden> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(grid_dir().join("golden/rotary"))
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "csv"))
        .collect();
    paths.sort();
    paths.iter().map(|path| read_golden(path)).collect()
}

fn golden(name: &str) -> Golden {
    read_golden(&grid_dir().join(format!("golden/rotary/{name}.csv")))
}

/// The case names `cases.txt` declares with `rotary`.
fn declared_cases() -> BTreeSet<String> {
    std::fs::read_to_string(grid_dir().join("cases.txt"))
        .unwrap()
        .lines()
        .map(|line| line.split('#').next().unwrap_or(""))
        .filter_map(|line| {
            let mut words = line.split_whitespace();
            (words.next() == Some("rotary")).then(|| words.next().unwrap().to_owned())
        })
        .collect()
}

/// Longitude difference with the antimeridian taken into account.
fn lng_difference(a: f64, b: f64) -> f64 {
    let d = (a - b).abs() % 360.0;
    d.min(360.0 - d)
}

/// Why `ours` is not `theirs` bit for bit, or `None` if it is.
fn bit_mismatch(ours: &[GridPoint], theirs: &[GridPoint]) -> Option<String> {
    if ours.len() != theirs.len() {
        return Some(format!(
            "{} points where Mission Planner has {}",
            ours.len(),
            theirs.len()
        ));
    }
    for (index, (a, b)) in ours.iter().zip(theirs).enumerate() {
        if a.tag != b.tag
            || a.lat.to_bits() != b.lat.to_bits()
            || a.lng.to_bits() != b.lng.to_bits()
            || a.alt.to_bits() != b.alt.to_bits()
        {
            return Some(format!(
                "point {index}: {:?} {:?},{:?} alt {:?} where Mission Planner has {:?} {:?},{:?} \
                 alt {:?}",
                a.tag, a.lat, a.lng, a.alt, b.tag, b.lat, b.lng, b.alt
            ));
        }
    }
    None
}

/// Why `ours` is not `theirs` to class C, or `None` if it is.
fn class_c_mismatch(ours: &[GridPoint], theirs: &[GridPoint]) -> Option<String> {
    if ours.len() != theirs.len() {
        return Some(format!(
            "{} points where Mission Planner has {}",
            ours.len(),
            theirs.len()
        ));
    }
    for (index, (a, b)) in ours.iter().zip(theirs).enumerate() {
        let (dlat, dlng) = ((a.lat - b.lat).abs(), lng_difference(a.lng, b.lng));
        if a.tag != b.tag || dlat > DEGREES || dlng > DEGREES || a.alt != b.alt {
            return Some(format!(
                "point {index}: {:?} {},{} where Mission Planner has {:?} {},{}",
                a.tag, a.lat, a.lng, b.tag, b.lat, b.lng
            ));
        }
    }
    None
}

#[test]
fn every_rotary_case_has_a_golden_and_every_golden_a_case() {
    let declared = declared_cases();
    let generated: BTreeSet<String> = goldens().into_iter().map(|g| g.name).collect();
    assert!(
        declared.len() >= 30,
        "at least 30 rotary cases, cases.txt has {}",
        declared.len()
    );
    assert_eq!(
        declared, generated,
        "cases.txt and golden/rotary disagree: run tools/csharp-reference/regen-grid.sh"
    );
}

#[test]
fn every_class_c_exception_names_a_case_and_a_reason() {
    let declared = declared_cases();
    let mut names = BTreeSet::new();
    for (name, reason) in CLASS_C {
        assert!(declared.contains(*name), "{name} is not a rotary case");
        assert!(names.insert(*name), "{name} listed twice");
        assert!(!reason.trim().is_empty(), "{name} has no reason");
    }
    assert!(
        CLASS_C.len() * 10 <= declared.len(),
        "{} of {} cases need an exception; the port is wrong",
        CLASS_C.len(),
        declared.len()
    );
}

#[test]
fn the_rotary_matches_mission_planner_bit_for_bit() {
    let exceptions: BTreeSet<&str> = CLASS_C.iter().map(|(name, _)| *name).collect();
    let mut report = String::new();
    let (mut strict, mut points) = (0, 0);
    for golden in goldens() {
        if exceptions.contains(golden.name.as_str()) {
            continue;
        }
        strict += 1;
        points += golden.points.len();
        let ours = create_rotary(&golden.polygon, &golden.args).unwrap();
        if let Some(why) = bit_mismatch(&ours, &golden.points) {
            writeln!(report, "{}: {why}", golden.name).unwrap();
        }
    }
    println!("{strict} rotary cases, {points} points, compared bit for bit");
    assert!(
        report.is_empty(),
        "differs from Grid.CreateRotary:\n{report}"
    );
}

#[test]
fn class_c_exceptions_hold_class_c() {
    let exceptions: BTreeSet<&str> = CLASS_C.iter().map(|(name, _)| *name).collect();
    let mut report = String::new();
    for golden in goldens() {
        if !exceptions.contains(golden.name.as_str()) {
            continue;
        }
        let ours = create_rotary(&golden.polygon, &golden.args).unwrap();
        if let Some(why) = class_c_mismatch(&ours, &golden.points) {
            writeln!(report, "{}: {why}", golden.name).unwrap();
        }
    }
    assert!(report.is_empty(), "outside class C:\n{report}");
}

/// `CreateRotary` takes a trigger spacing, an angle, overshoots, a shutter flag, a lane separation
/// and a lead-in and never reads them, which is why [`RotaryArgs`] has no fields for them: Mission
/// Planner's own output for a case that sets them all is the output for the case that sets none.
#[test]
fn the_arguments_rotary_args_leaves_out_change_nothing_in_the_csharp() {
    let base = golden("rotary_cbr_square_home");
    let unused = golden("rotary_cbr_square_unused_args");
    assert_ne!(base.arguments, unused.arguments);
    assert_eq!(base.polygon, unused.polygon);
    assert_eq!(base.args, unused.args);
    assert!(!base.points.is_empty());
    assert_eq!(base.points, unused.points);
}

/// C# behaviours worth pinning by name, each visible in Mission Planner's own output: no laps
/// and no area give nothing; the last reversed lap revisits its first point (`Grid.cs:290-293`);
/// a matched perimeter starts the first lap at its own last point (`Grid.cs:283-287`), unless
/// exactly one lap is reversed; every point is an `S`.
#[test]
fn the_csharp_behaviours_are_in_the_goldens() {
    for name in ["rotary_cbr_square_laps0", "rotary_cbr_point"] {
        assert!(golden(name).points.is_empty(), "{name}");
    }

    // A square's laps are four corners each.
    let cw1 = golden("rotary_cbr_square_cw1").points;
    assert_eq!(cw1[0], cw1[4], "the reversed lap is closed");
    let plain = golden("rotary_cbr_square_home").points;
    assert_ne!(plain[0], plain[4]);
    assert_eq!(plain.len() + 1, cw1.len());

    let matched = golden("rotary_cbr_square_match").points;
    assert_eq!(
        matched[0], matched[4],
        "the first lap starts at its last point"
    );
    assert_eq!(matched.len(), plain.len() + 1);
    // With exactly one lap reversed there is no extra start point, only the closing one.
    let matched_cw1 = golden("rotary_cbr_square_match_cw1").points;
    assert_eq!(matched_cw1, cw1);

    for golden in goldens() {
        assert!(
            golden.points.iter().all(|p| p.tag == GridTag::Start),
            "{}",
            golden.name
        );
    }
}
