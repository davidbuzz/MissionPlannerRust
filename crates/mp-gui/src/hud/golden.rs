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

//! Golden frames of the display: DELIVERABLES.md D9, PLAN.md §13.6 row 75.
//!
//! Every case is a [`HudInputs`] turned into a [`super::scene`] - the geometry of `doPaint()`,
//! `// C#: ExtLibs/Controls/HUD.cs:1954-3333` - drawn by [`raster`] and compared with the image
//! committed at `testdata/hud/<case>.png`. A change that moves a line, a number or a colour of
//! the display changes the picture, and the comparison fails with the count of pixels, the box
//! they are in and, in the temporary directory, the new frame and a picture of the difference.
//!
//! Two kinds of case:
//!
//! * **Recorded flights.** `testdata/mavlink/autotest.tlog` played through the vehicle state as
//!   the link plays it, and each chosen moment drawn through [`live_inputs`] - the function the
//!   flight screen calls every frame - with the display's clocks run on the log's own time. A
//!   sheet of six frames a case: around the hardest roll, around the hardest pitch, and across
//!   arming, where ARMED shows and then goes after its eight seconds (`HUD.cs:3084-3119`).
//! * **Hard cases**, from inputs written here: level flight, a bank, the nose straight up and
//!   straight down (the gimbal lock of a pitch of ±90°), inverted, a NaN attitude, NaN
//!   readouts, an infinite speed, altitude and heading, a lost GPS fix as text and as its
//!   picture, and no vehicle at all.
//! * **Over a camera picture**, a stand-in frame drawn here: the level frame as the flight
//!   screen paints it over a capture (`hud::paint_over_camera`, `HUD.cs:1988-2013`) with Enable
//!   HUD Overlay ticked - the instruments without the sky and ground - and unticked, `hudon`
//!   false: the picture alone.
//!
//! # When the display changes on purpose
//!
//! `HUD_UPDATE_GOLDENS=1 cargo test -p mp-gui hud::golden` draws every case again and writes
//! it over its golden, then look at what changed before committing it. A golden that no case
//! draws any more fails [`no_golden_is_left_without_a_case`].

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use mp_log::TlogReader;
use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_vehicle::{DateTime, VehicleId, VehicleRegistry, VehicleState};

use super::colour::{GROUND, SKY};
use super::raster::{self, Image};
use super::{Element, HudInputs, Item, Timing, live_inputs, scene};

/// The hard cases' size: a HUD at a size the flight screen gives it.
const WIDTH: u32 = 480;
/// See [`WIDTH`].
const HEIGHT: u32 = 360;

/// A recorded sheet's frames: three across and two down, each this size.
const TILE: (u32, u32) = (320, 240);
/// Frames on a sheet.
const TILES: (u32, u32) = (3, 2);

/// The environment variable that rewrites the goldens.
const UPDATE: &str = "HUD_UPDATE_GOLDENS";

/// Where the goldens are committed.
fn golden_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/hud")
}

/// The recorded flight the sheets are drawn from.
fn autotest_log() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/mavlink/autotest.tlog")
}

/// Level cruise, armed, with a fix, every readout given: what the other hard cases change.
fn cruising() -> HudInputs {
    HudInputs {
        has_vehicle: true,
        roll: 0.0,
        pitch: 0.0,
        heading: 90.0,
        target_heading: 110.0,
        ground_course: 92.0,
        xtrack_error: 3.0,
        turn_rate: 2.0,
        airspeed: 18.0,
        ground_speed: 17.0,
        target_speed: 20.0,
        altitude: 100.0,
        target_altitude: 105.0,
        vertical_speed: 1.5,
        mode: "Auto".to_owned(),
        mode_changed_for: Some(Duration::from_secs(30)),
        wp_distance: 250.0,
        wp_number: 3,
        clock: "12:34:56".to_owned(),
        battery_voltage: 12.6,
        battery_current: 3.2,
        battery_remaining: 85,
        gps_fix: 3,
        armed: true,
        armed_for: Some(Duration::from_secs(30)),
        vibe: [3.0, 4.0, 5.0],
        cpu_load: 12.0,
        ekf_status: 0.1,
        prearm_ready: true,
        ..HudInputs::default()
    }
}

/// The hard cases, by golden name.
fn hard_cases() -> Vec<(&'static str, HudInputs)> {
    let nan = f32::NAN;
    vec![
        ("level", cruising()),
        (
            "banked",
            HudInputs {
                roll: 30.0,
                pitch: 5.0,
                ..cruising()
            },
        ),
        // The nose straight up and straight down: the ladder at its ends, and the sky or the
        // ground filling the display.
        (
            "gimbal_lock_up",
            HudInputs {
                roll: 20.0,
                pitch: 90.0,
                ..cruising()
            },
        ),
        (
            "gimbal_lock_down",
            HudInputs {
                roll: -20.0,
                pitch: -90.0,
                ..cruising()
            },
        ),
        (
            "inverted",
            HudInputs {
                roll: 180.0,
                pitch: 10.0,
                ..cruising()
            },
        ),
        // An ATTITUDE of NaNs, which the wire can carry: drawn level, heading 0, with the red
        // "NaN Error" line (HUD.cs:2018-2025, 3025-3027).
        (
            "nan_attitude",
            HudInputs {
                roll: nan,
                pitch: nan,
                heading: nan,
                ..cruising()
            },
        ),
        (
            "nan_readouts",
            HudInputs {
                airspeed: nan,
                ground_speed: nan,
                altitude: nan,
                vertical_speed: nan,
                wp_distance: nan,
                battery_voltage: nan,
                ekf_status: nan,
                ..cruising()
            },
        ),
        // A `VFR_HUD` of infinities, which a float on the wire can carry: the tapes' loops,
        // which never end for one in the C#, end, and the frame is drawn.
        (
            "infinite_speed",
            HudInputs {
                airspeed: f32::INFINITY,
                ground_speed: f32::INFINITY,
                ..cruising()
            },
        ),
        (
            "infinite_altitude",
            HudInputs {
                altitude: f32::INFINITY,
                ..cruising()
            },
        ),
        (
            "infinite_heading",
            HudInputs {
                heading: f32::INFINITY,
                ..cruising()
            },
        ),
        // The fix lost: GPS1's "No Fix" in red (HUD.cs:2926-3028).
        (
            "lost_fix",
            HudInputs {
                gps_fix: 1,
                ..cruising()
            },
        ),
        (
            "lost_fix_icons",
            HudInputs {
                gps_fix: 1,
                display_icons: true,
                ..cruising()
            },
        ),
        ("no_vehicle", HudInputs::default()),
    ]
}

/// The cases over a camera picture, by golden name: Enable HUD Overlay ticked - the instruments
/// over the picture, without the sky and ground (`bgon = false`) - and unticked, `hudon` false:
/// the picture alone. `// C#: ExtLibs/Controls/HUD.cs:1988-2013, 2067-2099`
fn camera_cases() -> Vec<(&'static str, HudInputs)> {
    vec![
        ("camera_overlay_on", cruising()),
        (
            "camera_overlay_off",
            HudInputs {
                hud_on: false,
                ..cruising()
            },
        ),
    ]
}

/// A stand-in for a camera frame at the hard cases' size: red across, green down, a grid every
/// 40 pixels. Not a colour of the sky or the ground anywhere, so a pixel of either is the
/// scene's.
fn camera_picture() -> Image {
    let mut pixels = Vec::with_capacity(usize::try_from(WIDTH * HEIGHT * 3).unwrap_or(0));
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let grid = x % 40 == 0 || y % 40 == 0;
            // Within 0..=255: x < WIDTH and y < HEIGHT.
            #[allow(clippy::cast_possible_truncation)]
            let (r, g) = ((x * 255 / WIDTH) as u8, (y * 255 / HEIGHT) as u8);
            pixels.extend(if grid { [255, 255, 255] } else { [r, g, 128] });
        }
    }
    Image {
        width: WIDTH,
        height: HEIGHT,
        pixels,
    }
}

/// A camera case drawn as the flight screen paints it over a picture: [`camera_picture`], then
/// what [`super::over_camera`] keeps of the scene for the case's `hudon`.
fn over_camera_frame(inputs: &HudInputs) -> Image {
    #[allow(clippy::cast_precision_loss)] // a few hundred pixels
    let drawn = scene(inputs, WIDTH as f32, HEIGHT as f32);
    raster::render_over(camera_picture(), super::over_camera(&drawn, inputs.hud_on))
}

/// One moment of a recorded flight: when, and the attitude and arming then.
#[derive(Debug, Clone, Copy)]
struct Moment {
    micros: u64,
    roll: f64,
    pitch: f64,
    armed: bool,
}

/// Plays a log through a vehicle registry, as the link does, and calls `at` with each record's
/// time and the flying vehicle's state after every ATTITUDE it sends. The flying vehicle is the
/// first to send an ATTITUDE.
fn play(log: &[u8], mut at: impl FnMut(u64, &VehicleState)) {
    let mut registry = VehicleRegistry::new();
    let mut decoder = FrameDecoder::new();
    let mut flying: Option<VehicleId> = None;
    let mut micros = 0u64;
    let mut reader = TlogReader::new(log);
    while let Some(record) = reader.next_record(&DIALECT) {
        micros = record.timestamp_micros.unwrap_or(micros);
        let sent = DateTime::from_tlog_micros(micros).unwrap_or(DateTime::MIN);
        decoder.push_and_drain(record.frame, &DIALECT, |frame| {
            let Some(message) = MavMessage::decode(frame.msgid, frame.payload) else {
                return;
            };
            let id = registry.apply_at(frame.sysid, frame.compid, frame.seq, &message, sent);
            if matches!(message, MavMessage::Attitude(_))
                && *flying.get_or_insert(id) == id
                && let Some(state) = registry.working(id)
            {
                at(micros, state);
            }
        });
    }
}

/// The recorded sheets: a name, and which moments to draw given every moment of the flight.
type Pick = fn(&[Moment]) -> Vec<u64>;

/// The moments `offsets` seconds from `centre`.
fn around(centre: u64, offsets: [f64; 6]) -> Vec<u64> {
    offsets
        .iter()
        .map(|seconds| {
            // Offsets of a few seconds, in microseconds: far inside both types.
            #[allow(clippy::cast_possible_truncation)]
            let shift = (seconds * 1e6) as i64;
            centre.saturating_add_signed(shift)
        })
        .collect()
}

/// Every half second from a second before to a second and a half after.
const HALF_SECONDS: [f64; 6] = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5];

fn hardest(moments: &[Moment], by: fn(&Moment) -> f64) -> u64 {
    moments
        .iter()
        .max_by(|a, b| by(a).abs().total_cmp(&by(b).abs()))
        .map_or(0, |moment| moment.micros)
}

fn recorded_cases() -> [(&'static str, Pick); 3] {
    [
        ("autotest_roll", |moments| {
            around(hardest(moments, |m| m.roll), HALF_SECONDS)
        }),
        ("autotest_pitch", |moments| {
            around(hardest(moments, |m| m.pitch), HALF_SECONDS)
        }),
        // ARMED is shown for eight seconds after arming and then goes.
        ("autotest_arming", |moments| {
            let armed = moments
                .windows(2)
                .find(|pair| matches!(pair, [before, after] if !before.armed && after.armed))
                .and_then(|pair| pair.get(1))
                .map_or(0, |moment| moment.micros);
            around(armed, [-1.0, 0.0, 2.0, 7.5, 8.5, 12.0])
        }),
    ]
}

/// The recorded flight's sheets, by golden name.
///
/// # Panics
///
/// The log is not there, or a sheet cannot be filled.
fn recorded_sheets() -> Vec<(&'static str, Image)> {
    let log = std::fs::read(autotest_log()).expect("testdata/mavlink/autotest.tlog is committed");
    let mut moments = Vec::new();
    play(&log, |micros, state| {
        moments.push(Moment {
            micros,
            roll: state.attitude.roll.0,
            pitch: state.attitude.pitch.0,
            armed: state.armed,
        });
    });
    let first = moments.first().map_or(0, |moment| moment.micros);
    recorded_cases()
        .into_iter()
        .map(|(name, pick)| {
            let wanted = pick(&moments);
            // The display's clocks run on the log's time: `now` is the start plus how far in.
            let start = Instant::now();
            let mut timing = Timing::default();
            let mut frames = Vec::new();
            play(&log, |micros, state| {
                let now = start + Duration::from_micros(micros.saturating_sub(first));
                let clock = i64::try_from(micros)
                    .ok()
                    .and_then(chrono::DateTime::from_timestamp_micros)
                    .map(|time| time.format("%H:%M:%S").to_string())
                    .unwrap_or_default();
                // Every ATTITUDE is a frame, so the clocks see each transition when the screen
                // would.
                let inputs = live_inputs(state, &mut timing, now, clock, &[]);
                if wanted.get(frames.len()).is_some_and(|due| micros >= *due) {
                    frames.push(inputs);
                }
            });
            assert_eq!(frames.len(), wanted.len(), "{name}: the log ended early");
            (name, sheet(&frames))
        })
        .collect()
}

/// Frames laid out left to right, top to bottom.
fn sheet(frames: &[HudInputs]) -> Image {
    let (tile_w, tile_h) = TILE;
    let mut out = Image::new(tile_w * TILES.0, tile_h * TILES.1, 0);
    let row_bytes = usize::try_from(tile_w * 3).unwrap_or(0);
    for (index, inputs) in (0u32..).zip(frames) {
        #[allow(clippy::cast_precision_loss)] // a few hundred pixels
        let tile = raster::render(&scene(inputs, tile_w as f32, tile_h as f32), tile_w, tile_h);
        let (left, top) = ((index % TILES.0) * tile_w, (index / TILES.0) * tile_h);
        for (row, source) in (0u32..).zip(tile.pixels.chunks_exact(row_bytes)) {
            let at = usize::try_from((top + row) * out.width * 3 + left * 3).unwrap_or(0);
            if let Some(target) = out.pixels.get_mut(at..at + row_bytes) {
                target.copy_from_slice(source);
            }
        }
    }
    out
}

/// A hard case drawn at the hard cases' size.
fn frame(inputs: &HudInputs) -> Image {
    #[allow(clippy::cast_precision_loss)] // a few hundred pixels
    let drawn = scene(inputs, WIDTH as f32, HEIGHT as f32);
    raster::render(&drawn, WIDTH, HEIGHT)
}

/// Every case's name and frame.
fn every_case() -> Vec<(&'static str, Image)> {
    let mut all: Vec<(&'static str, Image)> = hard_cases()
        .iter()
        .map(|(name, inputs)| (*name, frame(inputs)))
        .collect();
    all.extend(
        camera_cases()
            .iter()
            .map(|(name, inputs)| (*name, over_camera_frame(inputs))),
    );
    all.extend(recorded_sheets());
    all
}

fn save(image: &Image, path: &Path) -> Result<(), String> {
    let buffer = image::RgbImage::from_raw(image.width, image.height, image.pixels.clone())
        .ok_or_else(|| "the pixel buffer is not its size".to_owned())?;
    buffer.save(path).map_err(|err| err.to_string())
}

fn load(path: &Path) -> Result<Image, String> {
    let decoded = image::open(path).map_err(|err| err.to_string())?.to_rgb8();
    Ok(Image {
        width: decoded.width(),
        height: decoded.height(),
        pixels: decoded.into_raw(),
    })
}

/// Every case matches its golden - or, with [`UPDATE`] set, is written as its golden.
#[test]
fn every_frame_matches_its_golden() {
    let dir = golden_dir();
    let update = std::env::var_os(UPDATE).is_some();
    let scratch = std::env::temp_dir().join("mp-hud-golden");
    let mut failures = Vec::new();
    for (name, actual) in every_case() {
        let path = dir.join(format!("{name}.png"));
        if update {
            std::fs::create_dir_all(&dir).expect("testdata/hud");
            save(&actual, &path).expect("the golden is written");
            eprintln!("wrote {}", path.display());
            continue;
        }
        let expected = match load(&path) {
            Ok(expected) => expected,
            Err(err) => {
                failures.push(format!(
                    "{name}: no golden at {} ({err}); {UPDATE}=1 draws it",
                    path.display()
                ));
                continue;
            }
        };
        let difference = raster::compare(&expected, &actual);
        if difference.as_ref().is_ok_and(raster::Difference::matches) {
            continue;
        }
        let _ = std::fs::create_dir_all(&scratch);
        let (drawn, diff) = (
            scratch.join(format!("{name}.actual.png")),
            scratch.join(format!("{name}.diff.png")),
        );
        let _ = save(&actual, &drawn);
        if difference.is_ok() {
            let _ = save(&raster::difference_image(&expected, &actual), &diff);
        }
        failures.push(format!(
            "{name}: {difference:?} against {}; drawn now: {}, the difference: {}",
            path.display(),
            drawn.display(),
            diff.display()
        ));
    }
    assert!(
        failures.is_empty(),
        "the display no longer draws its golden frames. If the change is meant, run \
         {UPDATE}=1 cargo test -p mp-gui hud::golden and look at the new images before \
         committing them.\n{}",
        failures.join("\n")
    );
}

/// A golden no case draws is stale: nothing holds the display to it any more.
#[test]
fn no_golden_is_left_without_a_case() {
    let names: Vec<&str> = hard_cases()
        .iter()
        .map(|(name, _)| *name)
        .chain(camera_cases().iter().map(|(name, _)| *name))
        .chain(recorded_cases().iter().map(|(name, _)| *name))
        .collect();
    let mut unique = names.clone();
    unique.sort_unstable();
    unique.dedup();
    assert_eq!(
        unique.len(),
        names.len(),
        "two cases share a name: {names:?}"
    );
    let on_disk: Vec<String> = std::fs::read_dir(golden_dir())
        .expect("testdata/hud")
        .filter_map(|entry| entry.ok()?.file_name().into_string().ok())
        .collect();
    for file in &on_disk {
        let stem = file.strip_suffix(".png").unwrap_or(file);
        assert!(
            names.contains(&stem),
            "testdata/hud/{file} is drawn by no case; delete it or add its case"
        );
    }
    for name in &names {
        assert!(
            on_disk.contains(&format!("{name}.png")),
            "{name} has no golden; {UPDATE}=1 draws it"
        );
    }
}

/// The comparison sees what a golden is for. The attitude moved by one pixel, one digit
/// changed, and one word changed only in colour each fail it; the same inputs drawn again pass.
#[test]
fn the_comparison_catches_a_moved_line_a_changed_number_and_a_changed_colour() {
    let base = frame(&cruising());
    let differs = |inputs: HudInputs| {
        let difference = raster::compare(&base, &frame(&inputs)).expect("same size");
        (!difference.matches(), difference)
    };
    let again = raster::compare(&base, &frame(&cruising())).expect("same size");
    assert_eq!(again.differing, 0, "a frame is drawn the same every time");
    // One pixel of pitch: the horizon, the ladder and its numbers one pixel lower.
    #[allow(clippy::cast_precision_loss)]
    let one_pixel = 1.0 / super::pixels_per_degree(HEIGHT as f32);
    let (seen, how) = differs(HudInputs {
        pitch: one_pixel,
        ..cruising()
    });
    assert!(seen, "a one-pixel move of the horizon: {how:?}");
    // One digit: waypoint 3 is waypoint 8, the pair of digits nearest alike in the font.
    let (seen, how) = differs(HudInputs {
        wp_number: 8,
        ..cruising()
    });
    assert!(seen, "waypoint 3 to 8: {how:?}");
    // Only a colour: EKF past 0.5 is the same word in orange (HUD.cs:3209-3263).
    let (seen, how) = differs(HudInputs {
        ekf_status: 0.6,
        ..cruising()
    });
    assert!(seen, "EKF white to orange: {how:?}");
}

/// Any digit written in place of any other, at the smallest size the display uses - `Height /
/// 30` with the C#'s floor of 9 - changes more pixels than the comparison tolerates.
/// `// C#: ExtLibs/Controls/HUD.cs:2040-2041`
#[test]
fn every_digit_is_told_from_every_other_at_the_smallest_size() {
    let smallest = 9.0;
    let draw = |text: &str| {
        let scene = super::Scene {
            items: vec![Item::Label {
                text: text.to_owned(),
                at: (4.0, 2.0),
                size: smallest,
                colour: 0xff_ff_ff,
                align: super::Align::Left,
            }],
            ..super::Scene::default()
        };
        raster::render(&scene, 16, 16)
    };
    let digits: Vec<Image> = (0..10).map(|digit| draw(&digit.to_string())).collect();
    for (a, first) in digits.iter().enumerate() {
        for (b, second) in digits.iter().enumerate().skip(a + 1) {
            let difference = raster::compare(first, second).expect("same size");
            assert!(!difference.matches(), "{a} and {b}: {difference:?}");
        }
    }
}

/// Counts the pixels of exactly one colour.
fn count(image: &Image, colour: u32) -> usize {
    let [_, r, g, b] = colour.to_be_bytes();
    image
        .pixels
        .chunks_exact(3)
        .filter(|pixel| *pixel == [r, g, b])
        .count()
}

/// At a pitch of ±90° the display is all sky or all ground, whatever the roll: the horizon is
/// ninety degrees of ladder away.
#[test]
fn the_nose_straight_up_is_all_sky_and_straight_down_all_ground() {
    let cases = hard_cases();
    let case = |wanted: &str| {
        cases
            .iter()
            .find(|(name, _)| *name == wanted)
            .map(|(_, inputs)| frame(inputs))
            .expect("the case")
    };
    let total = usize::try_from(WIDTH * HEIGHT).expect("small");
    let (up, down) = (case("gimbal_lock_up"), case("gimbal_lock_down"));
    assert_eq!(count(&up, GROUND), 0);
    assert!(count(&up, SKY) > total / 3, "{}", count(&up, SKY));
    assert_eq!(count(&down, SKY), 0);
    assert!(count(&down, GROUND) > total / 3, "{}", count(&down, GROUND));
}

/// Over a camera picture the sky and ground are gone and the instruments are drawn - unless
/// `hudon` is false, when the frame is the picture, pixel for pixel.
/// `// C#: ExtLibs/Controls/HUD.cs:1988-2013, 2067-2099`
#[test]
fn over_a_picture_the_overlay_draws_the_instruments_or_nothing() {
    let picture = camera_picture();
    let cases = camera_cases();
    let case = |wanted: &str| {
        cases
            .iter()
            .find(|(name, _)| *name == wanted)
            .map(|(_, inputs)| over_camera_frame(inputs))
            .expect("the case")
    };
    let (on, off) = (case("camera_overlay_on"), case("camera_overlay_off"));
    assert!(off == picture, "hudon false: the picture alone");
    let difference = raster::compare(&picture, &on).expect("same size");
    assert!(
        !difference.matches(),
        "the instruments are drawn: {difference:?}"
    );
    assert_eq!(count(&on, SKY), 0);
    assert_eq!(count(&on, GROUND), 0);
    // Without a picture the same inputs draw the sky and the ground as ever.
    let bare = frame(&HudInputs {
        hud_on: false,
        ..cruising()
    });
    assert!(
        bare == frame(&cruising()),
        "no picture: hudon changes nothing"
    );
    assert!(count(&bare, SKY) > 1_000 && count(&bare, GROUND) > 1_000);
}

/// A lost fix changes the GPS line and nothing else on the display.
#[test]
fn a_lost_fix_changes_only_the_gps_line() {
    let lost = HudInputs {
        gps_fix: 1,
        ..cruising()
    };
    let difference = raster::compare(&frame(&cruising()), &frame(&lost)).expect("same size");
    let (left, top, right, bottom) = difference.region.expect("the GPS line changed");
    // Every changed pixel lies within the box the GPS labels' glyphs cover, in either frame.
    let mut corners = Vec::new();
    for inputs in [cruising(), lost] {
        #[allow(clippy::cast_precision_loss)]
        let drawn = scene(&inputs, WIDTH as f32, HEIGHT as f32);
        for (owner, index) in &drawn.owners {
            if let (
                Element::Gps,
                Some(Item::Label {
                    text,
                    at,
                    size,
                    align,
                    ..
                }),
            ) = (owner, drawn.items.get(*index))
            {
                corners.extend(raster::label_outline(text, *at, *size, *align).concat());
            }
        }
    }
    assert!(!corners.is_empty(), "the GPS line is a label");
    let (x0, y0, x1, y1) = corners.iter().fold(
        (f32::MAX, f32::MAX, f32::MIN, f32::MIN),
        |(x0, y0, x1, y1), (x, y)| (x0.min(*x), y0.min(*y), x1.max(*x), y1.max(*y)),
    );
    #[allow(clippy::cast_precision_loss)]
    let inside = x0 - 1.0 <= left as f32
        && right as f32 <= x1 + 1.0
        && y0 - 1.0 <= top as f32
        && bottom as f32 <= y1 + 1.0;
    assert!(
        inside,
        "changed ({left}, {top})-({right}, {bottom}), GPS line ({x0}, {y0})-({x1}, {y1})"
    );
}

/// Every shape's corners, for the tests that no shape is one gpui cannot draw.
fn corners(drawn: &super::Scene) -> Vec<(f32, f32)> {
    drawn
        .items
        .iter()
        .flat_map(|item| match item {
            Item::Fill { points, .. } | Item::Stroke { points, .. } => points.clone(),
            _ => Vec::new(),
        })
        .collect()
}

/// A NaN roll, pitch or heading - any one - is drawn as the C# draws it: all three as 0, the
/// horizon level and the tape at north, and "NaN Error" with the time in red at (50, 50), at
/// `Height / 30 + 10`. The frame is the level frame at heading 0 but for that line: every pixel
/// that differs lies within the line's glyphs. No shape has a corner gpui cannot draw.
/// `// C#: ExtLibs/Controls/HUD.cs:2018-2025, 3025-3027`
#[test]
fn a_nan_attitude_is_drawn_level_with_the_nan_error_line() {
    let level = HudInputs {
        roll: 0.0,
        pitch: 0.0,
        heading: 0.0,
        ..cruising()
    };
    let level_frame = frame(&level);
    let nan = f32::NAN;
    for inputs in [
        HudInputs {
            roll: nan,
            pitch: 25.0,
            heading: 200.0,
            ..cruising()
        },
        HudInputs {
            roll: -40.0,
            pitch: nan,
            heading: 200.0,
            ..cruising()
        },
        HudInputs {
            roll: -40.0,
            pitch: 25.0,
            heading: nan,
            ..cruising()
        },
        HudInputs {
            roll: nan,
            pitch: nan,
            heading: nan,
            ..cruising()
        },
    ] {
        let what = (inputs.roll, inputs.pitch, inputs.heading);
        #[allow(clippy::cast_precision_loss)]
        let drawn = scene(&inputs, WIDTH as f32, HEIGHT as f32);
        assert!(
            corners(&drawn)
                .iter()
                .all(|(x, y)| x.is_finite() && y.is_finite()),
            "{what:?}: a corner that is not finite"
        );
        let line = drawn.items.iter().find_map(|item| match item {
            Item::Label {
                text,
                at,
                size,
                colour,
                align,
            } if text.starts_with("NaN Error") => Some((text.clone(), *at, *size, *colour, *align)),
            _ => None,
        });
        let (text, at, size, colour, align) = line.expect("the NaN Error line");
        assert_eq!(text, "NaN Error 12:34:56", "{what:?}: the display's clock");
        assert_eq!(at, (50.0, 50.0));
        assert!((size - 22.0).abs() < f32::EPSILON, "360 / 30 + 10: {size}");
        assert_eq!(colour, super::colour::ALERT);
        let image = frame(&inputs);
        assert!(count(&image, SKY) > 1_000 && count(&image, GROUND) > 1_000);
        let difference = raster::compare(&level_frame, &image).expect("same size");
        let (left, top, right, bottom) = difference.region.expect("the NaN Error line is drawn");
        let outline = raster::label_outline(&text, at, size, align).concat();
        let (x0, y0, x1, y1) = outline.iter().fold(
            (f32::MAX, f32::MAX, f32::MIN, f32::MIN),
            |(x0, y0, x1, y1), (x, y)| (x0.min(*x), y0.min(*y), x1.max(*x), y1.max(*y)),
        );
        #[allow(clippy::cast_precision_loss)]
        let inside = x0 - 1.0 <= left as f32
            && right as f32 <= x1 + 1.0
            && y0 - 1.0 <= top as f32
            && bottom as f32 <= y1 + 1.0;
        assert!(
            inside,
            "{what:?}: changed ({left}, {top})-({right}, {bottom}), the line ({x0}, {y0})-({x1}, {y1})"
        );
    }
    // A finite attitude writes no such line.
    #[allow(clippy::cast_precision_loss)]
    let drawn = scene(&level, WIDTH as f32, HEIGHT as f32);
    assert!(!drawn.labels().iter().any(|text| text.starts_with("NaN")));
}

/// A read-out of NaN - the vertical speed, the cross-track error, the rate of turn, the angles -
/// or an infinite roll still makes a shape with a corner that is not finite, which the C# hands
/// to GDI+; the scene leaves it out, because gpui cannot draw it: a debug build stops in lyon's
/// path builder, which asserts every point is finite, and a release build's tessellator refuses
/// the fill (`PositionIsNaN`).
#[test]
fn a_shape_gpui_cannot_draw_is_left_out_of_the_scene() {
    let nan = f32::NAN;
    let inputs = HudInputs {
        vertical_speed: nan,
        xtrack_error: nan,
        turn_rate: nan,
        aoa_ssa: Some(super::AoaSsa {
            aoa: nan,
            ssa: nan,
            crit_aoa: 25.0,
        }),
        roll: f32::INFINITY,
        ..hard_cases()
            .into_iter()
            .find(|(name, _)| *name == "nan_readouts")
            .map(|(_, inputs)| inputs)
            .expect("the case")
    };
    #[allow(clippy::cast_precision_loss)]
    let drawn = scene(&inputs, WIDTH as f32, HEIGHT as f32);
    assert!(
        corners(&drawn)
            .iter()
            .all(|(x, y)| x.is_finite() && y.is_finite())
    );
    assert!(
        !drawn.labels_of(Element::Battery).is_empty(),
        "the rest is drawn"
    );
    let built = std::panic::catch_unwind(|| {
        let mut builder = gpui::PathBuilder::fill();
        builder.move_to(gpui::point(gpui::px(0.0), gpui::px(0.0)));
        builder.line_to(gpui::point(gpui::px(f32::NAN), gpui::px(0.0)));
        builder.line_to(gpui::point(gpui::px(4.0), gpui::px(4.0)));
        builder.build().is_ok()
    });
    if cfg!(debug_assertions) {
        assert!(built.is_err(), "lyon no longer asserts on a NaN point");
    } else {
        assert_eq!(built.ok(), Some(false), "gpui tessellated a NaN fill");
    }
}

/// The heading tape and the speed and altitude tapes end for every value: the infinities and
/// NaN, which the C#'s loops never end for or never start, a value past 2^24 where adding 1 to
/// a `float` changes nothing, and values past `int`'s and `long`'s range, where the C#'s casts
/// give their minimum. Each frame is drawn - scene and pixels - on another thread, which must
/// finish in time, and none draws more than the level frame, whose tapes are at their fullest.
/// `// C#: ExtLibs/Controls/HUD.cs:2268-2285, 2509-2531, 2605-2628`
#[test]
fn the_tapes_end_for_any_value() {
    let values = [
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::NAN,
        16_777_217.0,
        -16_777_217.0,
        3.0e9,
        -3.0e9,
        1.0e19,
        f32::MAX,
        f32::MIN,
    ];
    let mut cases = Vec::new();
    for value in values {
        cases.push(HudInputs {
            airspeed: value,
            ground_speed: value,
            ..cruising()
        });
        cases.push(HudInputs {
            altitude: value,
            ..cruising()
        });
        cases.push(HudInputs {
            heading: value,
            ..cruising()
        });
    }
    let (sender, receiver) = std::sync::mpsc::channel();
    let total = cases.len();
    std::thread::spawn(move || {
        for inputs in cases {
            #[allow(clippy::cast_precision_loss)]
            let drawn = scene(&inputs, WIDTH as f32, HEIGHT as f32);
            let image = raster::render(&drawn, WIDTH, HEIGHT);
            let _ = sender.send((inputs, drawn, image));
        }
    });
    #[allow(clippy::cast_precision_loss)]
    let normal = scene(&cruising(), WIDTH as f32, HEIGHT as f32).items.len();
    for _ in 0..total {
        let (inputs, drawn, image) = receiver
            .recv_timeout(Duration::from_secs(60))
            .expect("a frame with an extreme value never finished: a tape's loop did not end");
        let what = (inputs.airspeed, inputs.altitude, inputs.heading);
        assert!(
            drawn.items.len() <= normal + 10,
            "{what:?}: {} items, {normal} normally",
            drawn.items.len()
        );
        assert!(count(&image, SKY) > 1_000, "{what:?}: the frame is drawn");
    }
}

/// The recorded sheets are drawn from the flight: the moments exist, the arming sheet crosses
/// the arming, and the display's clocks put ARMED up and take it down as the screen would.
#[test]
fn the_arming_sheet_shows_armed_and_then_not() {
    let log = std::fs::read(autotest_log()).expect("testdata/mavlink/autotest.tlog is committed");
    let mut moments = Vec::new();
    play(&log, |micros, state| {
        moments.push(Moment {
            micros,
            roll: state.attitude.roll.0,
            pitch: state.attitude.pitch.0,
            armed: state.armed,
        });
    });
    assert!(moments.len() > 1_000, "{} attitudes", moments.len());
    let pick = recorded_cases()
        .iter()
        .find(|(name, _)| *name == "autotest_arming")
        .map(|(_, pick)| *pick)
        .expect("the arming sheet");
    let wanted = pick(&moments);
    let first = moments.first().map_or(0, |moment| moment.micros);
    let start = Instant::now();
    let mut timing = Timing::default();
    let mut banners = Vec::new();
    play(&log, |micros, state| {
        let now = start + Duration::from_micros(micros.saturating_sub(first));
        let inputs = live_inputs(state, &mut timing, now, String::new(), &[]);
        if wanted.get(banners.len()).is_some_and(|due| micros >= *due) {
            let drawn = scene(&inputs, 320.0, 240.0);
            banners.push(
                drawn
                    .labels()
                    .into_iter()
                    .find(|text| ["ARMED", "DISARMED"].contains(text))
                    .map(ToOwned::to_owned),
            );
        }
    });
    let banner = |text: &str| Some(text.to_owned());
    assert_eq!(
        banners,
        [
            banner("DISARMED"),
            banner("ARMED"),
            banner("ARMED"),
            banner("ARMED"),
            None,
            None
        ]
    );
}
