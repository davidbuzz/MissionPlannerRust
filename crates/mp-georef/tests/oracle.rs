//! Geo Reference Images held to Mission Planner's own `GeoRefImageBase` under mono.
//!
//! `tools/csharp-reference/regen-georef.sh` ran the C#'s "Process" and "GeoTag Images" over one
//! recorded SITL camera flight (`testdata/georef/camera.bin` and `camera.tlog`) and the 25
//! synthetic photos, in the cases below, and kept every file they wrote in `testdata/georef/
//! golden/<case>`: the eight report files, the `.xml` of positions a time-offset run leaves beside
//! the log, every geotagged copy, the lines the form's output box shows (`messages.txt`), and what
//! `GeoRefImageBase` held afterwards (`state.txt`). Each case here runs the port the way the form
//! runs it - `GeoRefImageBase::process` and `geotag_images`, over a copy of the photos and the
//! log, with the same terrain tile - and every one of those must match byte for byte.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use std::path::{Path, PathBuf};
use std::sync::Arc;

use mp_georef::georef::ProcessingMode;
use mp_georef::time::{DateTime, Kind};
use mp_georef::{FormSettings, GeoRefImageBase, Location};
use mp_terrain::{Http, HttpError, Srtm};

/// No network: a tile the oracle did not have would be queued, never fetched.
struct Offline;

impl Http for Offline {
    fn get(&self, _url: &str) -> Result<Vec<u8>, HttpError> {
        Err(HttpError("offline".to_owned()))
    }
}

fn terrain(dir: &Path) -> Srtm {
    let srtm = dir.join("srtm");
    std::fs::create_dir_all(&srtm).unwrap();
    let zip = std::fs::read(common::data().join("../srtm/S28E153.hgt.zip")).unwrap();
    mp_log::zip::extract(&zip, &srtm).unwrap();
    Srtm::without_thread(&srtm, Arc::new(Offline))
}

/// `camera.bin` with every `CAM` message relabelled `TRIG`, as the oracle's `mktrig` does: the
/// messages walked from FMT length to FMT length, the cut-off last one left alone.
fn relabel_cam_as_trig(data: &[u8]) -> Vec<u8> {
    let mut data = data.to_vec();
    let mut lengths = std::collections::HashMap::new();
    let (mut cam, mut trig) = (0u8, 0u8);
    let mut pos = 0usize;
    while pos + 3 <= data.len() {
        assert_eq!(&data[pos..pos + 2], &[0xA3, 0x95], "no header at {pos}");
        let ty = data[pos + 2];
        if ty == 0x80 {
            let defined = data[pos + 3];
            lengths.insert(defined, usize::from(data[pos + 4]));
            let name = String::from_utf8_lossy(&data[pos + 5..pos + 9])
                .trim_end_matches('\0')
                .to_owned();
            if name == "CAM" {
                cam = defined;
            }
            if name == "TRIG" {
                trig = defined;
            }
            pos += 89;
            continue;
        }
        let len = lengths[&ty];
        if pos + len > data.len() {
            break;
        }
        if ty == cam && cam != 0 && trig != 0 {
            data[pos + 2] = trig;
        }
        pos += len;
    }
    data
}

struct Case {
    name: &'static str,
    mode: ProcessingMode,
    log: &'static str,
    keys: &'static [(&'static str, &'static str)],
}

const CASES: &[Case] = &[
    Case {
        name: "cam-amsl",
        mode: ProcessingMode::CamMsg,
        log: "camera.bin",
        keys: &[],
    },
    Case {
        name: "cam-relalt-gpsalt",
        mode: ProcessingMode::CamMsg,
        log: "camera.bin",
        keys: &[("amsl", "0"), ("camgpsalt", "1")],
    },
    Case {
        name: "cam-lag",
        mode: ProcessingMode::CamMsg,
        log: "camera.bin",
        keys: &[("lag", "150")],
    },
    Case {
        name: "cam-drop",
        mode: ProcessingMode::CamMsg,
        log: "camera.bin",
        keys: &[("dropstart", "2"), ("dropend", "3"), ("minshutter", "0")],
    },
    Case {
        name: "cam-tlog",
        mode: ProcessingMode::CamMsg,
        log: "camera.tlog",
        keys: &[],
    },
    Case {
        name: "time-bin",
        mode: ProcessingMode::TimeOffset,
        log: "camera.bin",
        keys: &[("offset", "36003.3")],
    },
    Case {
        name: "time-bin-cam",
        mode: ProcessingMode::TimeOffset,
        log: "camera.bin",
        keys: &[("offset", "36004.3"), ("usecam", "1"), ("amsl", "0")],
    },
    Case {
        name: "time-tlog",
        mode: ProcessingMode::TimeOffset,
        log: "camera.tlog",
        keys: &[("offset", "36003.3"), ("amsl", "0")],
    },
    Case {
        name: "trig",
        mode: ProcessingMode::Trig,
        log: "trig.bin",
        keys: &[("triggpsalt", "1")],
    },
    Case {
        name: "cam-textlog",
        mode: ProcessingMode::CamMsg,
        log: "camera.log",
        keys: &[("lag", "150")],
    },
    Case {
        name: "cam-nogps",
        mode: ProcessingMode::CamMsg,
        log: "../dataflash.bin",
        keys: &[],
    },
    Case {
        name: "cam-nocam",
        mode: ProcessingMode::CamMsg,
        log: "../dataflash.bin",
        keys: &[("amsl", "0")],
    },
    Case {
        name: "trig-mismatch",
        mode: ProcessingMode::Trig,
        log: "trig.bin",
        keys: &[("dropend", "1")],
    },
    Case {
        name: "time-gps2",
        mode: ProcessingMode::TimeOffset,
        log: "camera.bin",
        keys: &[("offset", "36003.3"), ("gps", "GPS2")],
    },
];

fn key<'a>(case: &'a Case, name: &str, default: &'a str) -> &'a str {
    case.keys
        .iter()
        .find(|(k, _)| *k == name)
        .map_or(default, |(_, v)| *v)
}

fn g17(v: f64) -> String {
    // G17 in the invariant culture: 17 significant digits, the general format's rules.
    mp_log::netfmt::general(v, 17)
}

fn g9(v: f32) -> String {
    mp_log::netfmt::general(f64::from(v), 9)
}

fn kind(t: DateTime) -> &'static str {
    match t.kind {
        Kind::Unspecified => "Unspecified",
        Kind::Utc => "Utc",
        Kind::Local => "Local",
    }
}

fn location(l: &Location) -> String {
    format!(
        "{} {} {} {} {} {} {} {} {} {} {}",
        l.time.ticks,
        kind(l.time),
        g17(l.lat),
        g17(l.lon),
        g17(l.alt_amsl),
        g17(l.rel_alt),
        g17(l.gps_alt),
        g17(l.s_alt),
        g9(l.roll),
        g9(l.pitch),
        g9(l.yaw)
    )
}

fn state(georef: &mut GeoRefImageBase, photos: &[String]) -> String {
    let mut out = String::new();
    for (k, v) in georef.vehicle_locations.iter() {
        out.push_str(&format!("vehicle {k} {}\n", location(v)));
    }
    for (k, v) in georef.cam_locations.iter() {
        out.push_str(&format!("cam {k} {}\n", location(v)));
    }
    if let Some(pictures) = &georef.pictures_info {
        for p in pictures.values() {
            out.push_str(&format!(
                "picture {} {} {}\n",
                mp_georef::photos::file_name(&p.path),
                p.shot_time_reported_by_camera.ticks,
                location(&p.location)
            ));
        }
    }
    for photo in photos {
        let t = georef.photo_times.get(photo);
        out.push_str(&format!(
            "phototime {} {} {}\n",
            mp_georef::photos::file_name(photo),
            t.ticks,
            kind(t)
        ));
    }
    out
}

fn files_under(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            out.extend(files_under(&path));
        } else {
            out.push(path);
        }
    }
    out.sort();
    out
}

fn run_case(case: &Case) -> Vec<String> {
    let dir = common::scratch(&format!("oracle-{}", case.name));
    let work = dir.join("work");
    std::fs::create_dir_all(&work).unwrap();
    common::copy_photos(&work);
    let log_path = work.join(Path::new(case.log).file_name().unwrap());
    if case.log == "trig.bin" {
        let bin = std::fs::read(common::data().join("camera.bin")).unwrap();
        std::fs::write(&log_path, relabel_cam_as_trig(&bin)).unwrap();
    } else if case.log == "camera.log" {
        // "Convert .Bin to .Log", which mp-log holds to the C# on its own goldens.
        mp_log::convert::convert_bin_file(
            &common::data().join("camera.bin"),
            &log_path,
            &mp_log::convert::no_mode_names,
        )
        .unwrap();
    } else {
        std::fs::copy(common::data().join(case.log), &log_path).unwrap();
    }
    let srtm = terrain(&dir);

    let today = DateTime::from_parts(2026, 9, 24, 0, 0, 0, Kind::Local).unwrap();
    let mut georef = GeoRefImageBase::new(today);
    georef.use_amsl_alt = key(case, "amsl", "1") == "1";
    georef.millis_shutter_lag = key(case, "lag", "0").parse().unwrap();
    georef.minshutter = key(case, "minshutter", "0.5").parse().unwrap();
    let settings = FormSettings {
        mode: case.mode,
        offset_text: key(case, "offset", "0").to_owned(),
        use_gps2: key(case, "gps", "GPS") == "GPS2",
        use_cam_messages: key(case, "usecam", "0") == "1",
        drop_start: key(case, "dropstart", "0").parse().unwrap(),
        drop_end: key(case, "dropend", "0").parse().unwrap(),
        cam_use_gps_alt: key(case, "camgpsalt", "0") == "1",
        trig_use_gps_alt: key(case, "triggpsalt", "0") == "1",
        ..FormSettings::default()
    };
    let work_str = work.to_str().unwrap().to_owned();
    let mut messages = String::new();
    let mut append = |text: &str| messages.push_str(text);
    georef.process(
        log_path.to_str().unwrap(),
        &work_str,
        &settings,
        &mut append,
        &srtm,
    );
    georef
        .geotag_images(&work_str, &settings, &mut append)
        .unwrap();

    let mut photos: Vec<String> = (1..=25)
        .map(|n| format!("{work_str}/{}", common::photos::photo_name(n)))
        .collect();
    photos.sort();
    let mut failures = Vec::new();
    let golden = common::data().join("golden").join(case.name);

    let got_messages = messages.replace(&work_str, "{dir}");
    let want_messages = std::fs::read_to_string(golden.join("messages.txt")).unwrap();
    if got_messages != want_messages {
        failures.push(format!(
            "{}: messages differ\n--- want\n{want_messages}\n--- got\n{got_messages}",
            case.name
        ));
    }
    let got_state = state(&mut georef, &photos);
    let want_state = std::fs::read_to_string(golden.join("state.txt")).unwrap();
    if got_state != want_state {
        let first = got_state
            .lines()
            .zip(want_state.lines())
            .find(|(g, w)| g != w)
            .map(|(g, w)| format!("got  {g}\nwant {w}"))
            .unwrap_or_else(|| {
                format!(
                    "{} lines against {}",
                    got_state.lines().count(),
                    want_state.lines().count()
                )
            });
        failures.push(format!("{}: state differs\n{first}", case.name));
    }
    for want in files_under(&golden) {
        let relative = want.strip_prefix(&golden).unwrap();
        let name = relative.to_str().unwrap();
        if name == "messages.txt" || name == "state.txt" {
            continue;
        }
        let got = work.join(relative);
        let want_bytes = std::fs::read(&want).unwrap();
        match std::fs::read(&got) {
            Ok(got_bytes) if got_bytes == want_bytes => {}
            Ok(got_bytes) => {
                let at = got_bytes
                    .iter()
                    .zip(&want_bytes)
                    .position(|(a, b)| a != b)
                    .unwrap_or(got_bytes.len().min(want_bytes.len()));
                let context = |b: &[u8]| {
                    String::from_utf8_lossy(&b[at.saturating_sub(80)..(at + 80).min(b.len())])
                        .into_owned()
                };
                failures.push(format!(
                    "{}: {name} differs at byte {at} ({} against {} bytes)\n--- want\n{}\n--- got\n{}",
                    case.name,
                    got_bytes.len(),
                    want_bytes.len(),
                    context(&want_bytes),
                    context(&got_bytes)
                ));
            }
            Err(e) => failures.push(format!("{}: {name} not written: {e}", case.name)),
        }
    }
    // And nothing written that the C# did not write.
    let geotagged = work.join("geotagged");
    for got in files_under(&geotagged) {
        let relative = got.strip_prefix(&work).unwrap();
        if !golden.join(relative).exists() {
            failures.push(format!(
                "{}: {} written, not by the C#",
                case.name,
                relative.display()
            ));
        }
    }
    failures
}

#[test]
fn every_case_matches_the_csharp_byte_for_byte() {
    let mut failures = Vec::new();
    for case in CASES {
        failures.extend(run_case(case));
    }
    if !failures.is_empty() {
        let shown: Vec<&String> = failures.iter().take(12).collect();
        panic!(
            "{} differences from the C#:\n\n{}",
            failures.len(),
            shown
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join("\n\n")
        );
    }
}

#[test]
fn estimate_offset_matches_the_csharp() {
    let today = DateTime::from_parts(2026, 9, 24, 0, 0, 0, Kind::Local).unwrap();
    let mut georef = GeoRefImageBase::new(today);
    let mut messages = String::new();
    let log = common::data().join("camera.bin");
    let photos = common::data().join("photos");
    let offset = georef
        .estimate_offset(
            log.to_str().unwrap(),
            photos.to_str().unwrap(),
            "GPS",
            false,
            &mut |t: &str| messages.push_str(t),
        )
        .unwrap();
    messages.push_str(&format!(
        "Offset around :  {}\n\n",
        mp_log::netfmt::double(offset)
    ));
    let want = std::fs::read_to_string(common::data().join("golden/estimate.txt")).unwrap();
    assert_eq!(messages, want);
}
