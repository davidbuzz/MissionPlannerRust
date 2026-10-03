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

//! The Survey (Grid) dialog held to Mission Planner's own code run under mono.
//!
//! `testdata/grid/golden/accept/<case>.csv` is `Grid/GridUI.cs`'s logic - the constructor,
//! `domainUpDown1_ValueChanged` with `doCalc`, the handlers each control raises, and
//! `BUT_Accept_Click` - run by the grid oracle's former `accept` verb (GridUI.cs's code re-hosted,
//! deleted on 2026-10-03; the goldens stand as written at efb0801) over a polygon from
//! `testdata/grid/cases.txt`, with the operator's changes applied in order. Each case here opens
//! [`mp_mission::gridui::Dialog`] over the same polygon, makes the same changes through the same
//! controls, and presses Accept, then compares everything the golden recorded:
//!
//! - every control's value, as the C# holds it (`decimal.ToString()`, so the scale too), and the
//!   text boxes `doCalc` writes;
//! - the thirteen Stats labels, character for character;
//! - how many times the grid was generated, and `Grid.StartPointLatLngAlt` after it;
//! - the grid's size and its tags in order;
//! - Accept's refusal, or every `AddWPtoList` and `InsertWP` call: the command and its seven
//!   numbers, bit for bit;
//! - the settings Accept saves.
//!
//! `cameras.csv` is `xmlcamera` over the shipped `camerasBuiltin.xml`, compared with
//! [`mp_mission::cameras::Cameras::builtin`] float for float.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic
)]

use std::path::{Path, PathBuf};

use mp_mission::cameras::Cameras;
use mp_mission::dotnet::{Decimal, bool_text, general_f32};
use mp_mission::grid::StartPosition;
use mp_mission::gridui::{
    Accepted, Check, Context, Dialog, Num, Planner, Refused, Stat, Step, Text, Trigger,
};
use mp_units::LatLon;

fn golden_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/grid/golden/accept")
}

/// The golden files' escaping of a text value.
fn unescape(text: &str) -> String {
    text.replace("%20", " ")
        .replace("%2C", ",")
        .replace("%25", "%")
}

fn parse(text: &str) -> f64 {
    text.parse::<f64>()
        .unwrap_or_else(|_| panic!("not a number: {text}"))
}

/// One golden, read.
struct Golden {
    name: String,
    vertices: Vec<LatLon>,
    home: (f64, f64, f64),
    plane: bool,
    rows: usize,
    wpnav_speed: Option<f64>,
    wp_spd: Option<f64>,
    sets: Vec<(String, String)>,
    lines: Vec<Vec<String>>,
}

fn read(path: &Path) -> Golden {
    let text = std::fs::read_to_string(path).expect("a golden");
    let mut golden = Golden {
        name: String::new(),
        vertices: Vec::new(),
        home: (0.0, 0.0, 0.0),
        plane: false,
        rows: 0,
        wpnav_speed: None,
        wp_spd: None,
        sets: Vec::new(),
        lines: Vec::new(),
    };
    for line in text.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<String> = line.split(',').map(ToOwned::to_owned).collect();
        let field = |i: usize| fields.get(i).map_or("", String::as_str);
        match field(0) {
            "case" => golden.name = field(1).to_owned(),
            "vertex" => golden
                .vertices
                .push(LatLon::new(parse(field(1)), parse(field(2))).expect("a vertex")),
            "HomeLocation" => {
                golden.home = (parse(field(1)), parse(field(2)), parse(field(3)));
            }
            "firmware" => golden.plane = field(1) == "ArduPlane",
            "rows" => golden.rows = field(1).parse().expect("rows"),
            "WPNAV_SPEED" => {
                golden.wpnav_speed = (field(1) != "none").then(|| parse(field(1)));
            }
            "WP_SPD" => golden.wp_spd = (field(1) != "none").then(|| parse(field(1))),
            "set" => golden.sets.push((field(1).to_owned(), unescape(field(2)))),
            _ => golden.lines.push(fields),
        }
    }
    golden
}

/// What the operator does to one control, as the harness does it.
fn apply(dialog: &mut Dialog, point: &mut String, key: &str, value: &str) {
    if let Some(num) = Num::from_name(key) {
        dialog.type_num(num, value);
    } else if let Some(check) = Check::from_name(key) {
        let want = value == "True";
        if dialog.check(check) != want {
            dialog.click_check(check);
        }
    } else if let Some(trigger) = Trigger::from_name(key) {
        dialog.click_trigger(trigger);
    } else if let Some(text) = Text::from_name(key) {
        dialog.set_text(text, value);
    } else {
        match key {
            "CMB_camera" => dialog.select_camera(value),
            "CMB_startfrom" => {
                let start = StartPosition::from_name(value).expect("a start position");
                if dialog.select_startfrom(start) {
                    dialog.answer_point(Some(point));
                }
            }
            "point" => value.clone_into(point),
            _ => panic!("unknown control {key}"),
        }
    }
}

/// The call as the golden writes it: `add,-,...` or `insert,<index>,...`.
fn call_fields(step: &Step) -> (String, String, u16, [f64; 7]) {
    let (kind, index, call) = match step {
        Step::Add(call) => ("add", "-".to_owned(), call),
        Step::Insert(index, call) => ("insert", index.to_string(), call),
    };
    let [p1, p2, p3, p4] = call.params;
    (
        kind.to_owned(),
        index,
        call.command,
        [p1, p2, p3, p4, call.x, call.y, call.z],
    )
}

/// Runs one case and returns every difference from the golden.
fn check_case(golden: &Golden) -> Vec<String> {
    let context = Context {
        home: golden.home,
        plane: golden.plane,
        start_point: LatLon::default(),
    };
    let mut dialog = Dialog::open(&golden.vertices, Cameras::builtin(), &context, &|_| None);
    let mut point = "1".to_owned();
    for (key, value) in &golden.sets {
        apply(&mut dialog, &mut point, key, value);
    }
    let accepted = dialog.accept(&Planner {
        rows: golden.rows,
        wpnav_speed: golden.wpnav_speed,
        wp_spd: golden.wp_spd,
    });

    let mut problems: Vec<String> = Vec::new();
    let differ = |what: String, want: &str, got: &str| {
        (want != got).then(|| format!("{what}: C# {want:?}, Rust {got:?}"))
    };
    let mut calls = Vec::new();
    let mut settings = Vec::new();
    for fields in &golden.lines {
        let field = |i: usize| fields.get(i).map_or("", String::as_str);
        match field(0) {
            "control" => {
                let (name, want) = (field(1), unescape(field(2)));
                let got = if let Some(num) = Num::from_name(name) {
                    dialog.num(num).to_text()
                } else if let Some(check) = Check::from_name(name) {
                    bool_text(dialog.check(check)).to_owned()
                } else if let Some(trigger) = Trigger::from_name(name) {
                    bool_text(dialog.trigger(trigger)).to_owned()
                } else if let Some(text) = Text::from_name(name) {
                    dialog.text(text).to_owned()
                } else if name == "CMB_camera" {
                    dialog.camera().to_owned()
                } else if name == "CMB_startfrom" {
                    dialog.startfrom().to_owned()
                } else {
                    panic!("unknown control {name}")
                };
                problems.extend(differ(name.to_owned(), &want, &got));
            }
            "stat" => {
                let stat = Stat::ALL
                    .into_iter()
                    .find(|stat| stat.name() == field(1))
                    .expect("a stat");
                problems.extend(differ(
                    field(1).to_owned(),
                    &unescape(field(2)),
                    dialog.stat(stat),
                ));
            }
            "recomputes" => problems.extend(differ(
                "recomputes".to_owned(),
                field(1),
                &dialog.recomputes().to_string(),
            )),
            "StartPointLatLngAlt" => {
                let start = dialog.start_point();
                let got = [start.latitude(), start.longitude()];
                let want = [parse(field(1)), parse(field(2))];
                if got.map(f64::to_bits) != want.map(f64::to_bits) {
                    problems.push(format!("StartPointLatLngAlt: C# {want:?}, Rust {got:?}"));
                }
            }
            "spacing_handlers" => problems.extend(differ(
                "spacing_handlers".to_owned(),
                field(1),
                bool_text(dialog.spacing_handlers()),
            )),
            "points" => problems.extend(differ(
                "points".to_owned(),
                field(1),
                &dialog.grid().len().to_string(),
            )),
            "tags" => {
                let mut runs: Vec<String> = Vec::new();
                let grid = dialog.grid();
                let mut i = 0;
                while i < grid.len() {
                    let tag = grid[i].tag;
                    let mut j = i;
                    while j < grid.len() && grid[j].tag == tag {
                        j += 1;
                    }
                    runs.push(format!("{}*{}", tag.as_str(), j - i));
                    i = j;
                }
                problems.extend(differ("tags".to_owned(), field(1), &runs.join(" ")));
            }
            "accept" => {
                let got = match &accepted {
                    Ok(_) => "ok".to_owned(),
                    Err(Refused::Message(text, _)) => (*text).to_owned(),
                    Err(Refused::Exception(_, text)) => (*text).to_owned(),
                };
                problems.extend(differ("accept".to_owned(), &unescape(field(1)), &got));
            }
            "call" => calls.push(fields.clone()),
            "setting" => settings.push((field(1).to_owned(), unescape(field(2)))),
            "rows_added" | "commands" => {}
            other => panic!("{}: unknown line {other}", golden.name),
        }
    }

    let steps: &[Step] = match &accepted {
        Ok(Accepted { steps, .. }) | Err(Refused::Exception(steps, _)) => steps,
        Err(Refused::Message(..)) => &[],
    };
    if steps.len() != calls.len() {
        problems.push(format!("calls: C# {}, Rust {}", calls.len(), steps.len()));
    }
    for (index, (step, fields)) in steps.iter().zip(&calls).enumerate() {
        let (kind, at, command, numbers) = call_fields(step);
        let field = |i: usize| fields.get(i).map_or("", String::as_str);
        let want_numbers: Vec<f64> = (4..11).map(|i| parse(field(i))).collect();
        if kind != field(1)
            || at != field(2)
            || command.to_string() != field(3)
            || numbers.iter().map(|n| n.to_bits()).collect::<Vec<_>>()
                != want_numbers.iter().map(|n| n.to_bits()).collect::<Vec<_>>()
        {
            problems.push(format!(
                "call {index}: C# {}, Rust {kind},{at},{command},{numbers:?}",
                fields[1..].join(",")
            ));
            break;
        }
    }
    if let Ok(accepted) = &accepted {
        let got: Vec<(String, String)> = accepted
            .settings
            .iter()
            .map(|(key, value)| ((*key).to_owned(), value.clone()))
            .collect();
        if got != settings {
            problems.push(format!("settings: C# {settings:?}, Rust {got:?}"));
        }
    }
    problems
}

#[test]
fn every_accept_golden_matches_the_dialog() {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(golden_dir())
        .expect("testdata/grid/golden/accept")
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension().is_some_and(|ext| ext == "csv")
                && path.file_name().is_some_and(|name| name != "cameras.csv")
        })
        .collect();
    paths.sort();
    assert!(paths.len() >= 40, "only {} accept goldens", paths.len());
    let mut failures = Vec::new();
    let mut calls = 0;
    for path in &paths {
        let golden = read(path);
        calls += golden
            .lines
            .iter()
            .filter(|fields| fields.first().is_some_and(|key| key == "call"))
            .count();
        let problems = check_case(&golden);
        if !problems.is_empty() {
            failures.push(format!("{}:\n  {}", golden.name, problems.join("\n  ")));
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {} cases differ:\n{}",
        failures.len(),
        paths.len(),
        failures.join("\n")
    );
    // Every call was compared: a golden read wrong would compare nothing and pass.
    assert!(calls > 5000, "only {calls} calls in the goldens");
    eprintln!(
        "{} dialogs, {calls} Accept calls, all the C#'s",
        paths.len()
    );
}

/// The shipped camera list as `xmlcamera` reads it: the names in `CMB_camera`'s order, the floats,
/// and what `CMB_camera_SelectedIndexChanged` puts in the boxes.
#[test]
fn the_camera_list_is_xmlcameras() {
    let text = std::fs::read_to_string(golden_dir().join("cameras.csv")).expect("cameras.csv");
    let cameras = Cameras::builtin();
    let rows: Vec<Vec<&str>> = text
        .lines()
        .filter(|line| line.starts_with("camera,"))
        .map(|line| line.split(',').collect())
        .collect();
    let names: Vec<String> = rows.iter().map(|row| unescape(row[1])).collect();
    assert_eq!(cameras.items(), names.as_slice());
    for row in &rows {
        let camera = cameras.get(&unescape(row[1])).expect("listed");
        let floats = [
            camera.focallen,
            camera.imagewidth,
            camera.imageheight,
            camera.sensorwidth,
            camera.sensorheight,
        ];
        for (got, want) in floats.iter().zip(&row[2..7]) {
            let want: f32 = want.parse().expect("a float");
            assert_eq!(got.to_bits(), want.to_bits(), "{}", camera.name);
        }
        assert_eq!(
            Decimal::from_f32(camera.focallen)
                .map(Decimal::to_text)
                .as_deref(),
            Some(row[7]),
            "{}",
            camera.name
        );
        let texts = [
            camera.imagewidth,
            camera.imageheight,
            camera.sensorwidth,
            camera.sensorheight,
        ]
        .map(general_f32);
        assert_eq!(texts.as_slice(), &row[8..12], "{}", camera.name);
    }
}

/// The golden the planning screen's script is built on: the square from a `.fen`, the SITL home,
/// the angle and altitude changed - so `tests/gui/plan-survey.gui` can assert the mission to the
/// item.
#[test]
fn the_gui_script_case_is_what_the_script_expects() {
    let golden = read(&golden_dir().join("accept_gui_angle_alt.csv"));
    let commands = golden
        .lines
        .iter()
        .find(|fields| fields.first().is_some_and(|key| key == "commands"))
        .and_then(|fields| fields.get(1))
        .expect("commands")
        .replace(' ', ",");
    let script = include_str!("../../../tests/gui/plan-survey.gui");
    assert!(
        script.contains(&format!("expect mission.commands {commands}\n")),
        "tests/gui/plan-survey.gui does not assert the golden's commands"
    );
}
