//! `mpr georef <log> <photo folder> [options]`: the Geo Reference Images form's "Process" button
//! without the form, and with `--geotag` its "GeoTag Images" button, with `--estimate` its
//! "Estimate Offset". The report files go into the photo folder and the geotagged copies into its
//! `geotagged` folder, as the form writes them; the lines the form's output box shows are printed.
//!
//! The ground under each photo's footprint comes from the planner's own terrain cache, as
//! `srtm.getAltitude` has it: a tile not there yet is queued for download and the ground is 0 for
//! this run, as it is for the form's first run over new ground.
//! `// C#: GeoRef/georefimage.cs:140-319`

#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::process::ExitCode;

use mp_georef::georef::ProcessingMode;
use mp_georef::time::{DateTime, Kind, TICKS_PER_DAY, UNIX_EPOCH_TICKS};
use mp_georef::{FormSettings, GeoRefImageBase, Terrain};

/// The usage line `mpr help` shows.
pub(crate) const USAGE: &str =
    "mpr georef <log> <dir> [...]  Geo Reference Images (mpr georef for more)";

fn usage() -> ExitCode {
    eprintln!(
        "usage: mpr georef <log .bin|.log|.tlog> <photo folder> [options]\n\n\
         Matches the photos to the log as the Geo Reference Images form does, writes location.txt,\n\
         .csv, .kml, .gpx, .jxl, .geo, .tel into the folder, and prints the form's lines.\n\n\
         options (the form's defaults in brackets):\n  \
         --mode time|cam|trig   how photos are matched [cam]\n  \
         --offset S             seconds from the log's time to the camera's (time mode) [0]\n  \
         --gps2                 GPS2 rather than GPS lines\n  \
         --usecam               time mode: the CAM messages as the positions\n  \
         --relalt               relative altitude rather than AMSL\n  \
         --lag MS               shutter lag in milliseconds (cam mode) [0]\n  \
         --minshutter S         CAM messages closer than this are dropped [0.5]\n  \
         --dropstart N          CAM/TRIG messages dropped from the start [0]\n  \
         --dropend N            and from the end [0]\n  \
         --gpsalt               cam/trig mode: the GPS altitude\n  \
         --rotation DEG         camera rotation [90]\n  \
         --hfov DEG             horizontal field of view [200]\n  \
         --vfov DEG             vertical field of view [130]\n  \
         --basealt M            added to the AMSL altitude written into photos [0]\n  \
         --geotag               then write geotagged copies into <folder>/geotagged\n  \
         --estimate             only estimate the time offset (Estimate Offset)"
    );
    ExitCode::from(2)
}

/// `DateTime.Today`: the date now, at midnight - the machine's time taken as UTC, as the port
/// takes local time everywhere.
fn today() -> DateTime {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let days = i64::try_from(seconds / 86_400).unwrap_or(0);
    DateTime::from_ticks(UNIX_EPOCH_TICKS + days * TICKS_PER_DAY, Kind::Local)
}

/// What the command line asks for.
#[derive(Debug, Clone, PartialEq)]
struct Request {
    log: String,
    dir: String,
    settings: FormSettings,
    use_amsl_alt: bool,
    lag: i32,
    minshutter: f64,
    geotag: bool,
    estimate: bool,
}

fn parse(args: &[String]) -> Result<Request, String> {
    let (Some(log), Some(dir)) = (args.first(), args.get(1)) else {
        return Err("a log and a photo folder are needed".to_owned());
    };
    let mut request = Request {
        log: log.clone(),
        dir: dir.trim_end_matches(['/', '\\']).to_owned(),
        settings: FormSettings::default(),
        use_amsl_alt: true,
        lag: 0,
        minshutter: 0.5,
        geotag: false,
        estimate: false,
    };
    let mut rest = args.iter().skip(2);
    while let Some(flag) = rest.next() {
        let mut value = |name: &str| {
            rest.next()
                .cloned()
                .ok_or_else(|| format!("{name} needs a value"))
        };
        let number = |text: String, name: &str| {
            text.parse::<f64>()
                .map_err(|_| format!("{name}: {text} is not a number"))
        };
        let integer = |text: String, name: &str| {
            text.parse::<i32>()
                .map_err(|_| format!("{name}: {text} is not a whole number"))
        };
        match flag.as_str() {
            "--mode" => {
                request.settings.mode = match value("--mode")?.as_str() {
                    "time" => ProcessingMode::TimeOffset,
                    "cam" => ProcessingMode::CamMsg,
                    "trig" => ProcessingMode::Trig,
                    other => return Err(format!("--mode: {other} is not time, cam or trig")),
                }
            }
            // The form reads the offset as text, and so does the port.
            "--offset" => request.settings.offset_text = value("--offset")?,
            "--gps2" => request.settings.use_gps2 = true,
            "--usecam" => request.settings.use_cam_messages = true,
            "--relalt" => request.use_amsl_alt = false,
            "--lag" => request.lag = integer(value("--lag")?, "--lag")?,
            "--minshutter" => request.minshutter = number(value("--minshutter")?, "--minshutter")?,
            "--dropstart" => {
                request.settings.drop_start = integer(value("--dropstart")?, "--dropstart")?;
            }
            "--dropend" => request.settings.drop_end = integer(value("--dropend")?, "--dropend")?,
            "--gpsalt" => {
                request.settings.cam_use_gps_alt = true;
                request.settings.trig_use_gps_alt = true;
            }
            "--rotation" => {
                request.settings.camera_rotation = number(value("--rotation")?, "--rotation")?;
            }
            "--hfov" => request.settings.hfov = number(value("--hfov")?, "--hfov")?,
            "--vfov" => request.settings.vfov = number(value("--vfov")?, "--vfov")?,
            "--basealt" => request.settings.base_alt_text = value("--basealt")?,
            "--geotag" => request.geotag = true,
            "--estimate" => request.estimate = true,
            other => return Err(format!("unknown option {other}")),
        }
    }
    Ok(request)
}

/// The command over a given terrain: what `main` runs with the planner's cache, and the tests with
/// a tile of their own. Every line the form would show goes to `out`.
fn run_with(args: &[String], terrain: &dyn Terrain, out: &mut dyn FnMut(&str)) -> ExitCode {
    let request = match parse(args) {
        Ok(request) => request,
        Err(message) => {
            eprintln!("mpr georef: {message}\n");
            return usage();
        }
    };
    let mut georef = GeoRefImageBase::new(today());
    georef.use_amsl_alt = request.use_amsl_alt;
    georef.millis_shutter_lag = request.lag;
    georef.minshutter = request.minshutter;

    if request.estimate {
        // BUT_estoffset_Click (georefimage.cs:258-266).
        return match georef.estimate_offset(
            &request.log,
            &request.dir,
            request.settings.gps(),
            request.settings.use_cam_messages,
            &mut *out,
        ) {
            Ok(offset) => {
                out(&format!(
                    "Offset around :  {}\n\n",
                    mp_log::netfmt::double(offset)
                ));
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("mpr georef: {e}");
                ExitCode::FAILURE
            }
        };
    }

    if !std::path::Path::new(&request.log).is_file() {
        eprintln!("mpr georef: no log at {}", request.log);
        return ExitCode::FAILURE;
    }
    if !std::path::Path::new(&request.dir).is_dir() {
        eprintln!("mpr georef: no folder at {}", request.dir);
        return ExitCode::FAILURE;
    }
    let report = georef.process(
        &request.log,
        &request.dir,
        &request.settings,
        &mut *out,
        terrain,
    );
    if request.geotag
        && let Err(e) = georef.geotag_images(&request.dir, &request.settings, &mut *out)
    {
        eprintln!("mpr georef: {e}");
        return ExitCode::FAILURE;
    }
    if report.is_some() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// `mpr georef ...`.
#[must_use]
pub(crate) fn run(args: &[String]) -> ExitCode {
    if args.is_empty() {
        return usage();
    }
    let Some(srtm_dir) = mp_terrain::srtm_directory() else {
        eprintln!("mpr georef: no home directory to find the terrain cache under");
        return ExitCode::FAILURE;
    };
    let srtm = mp_terrain::Srtm::new(srtm_dir);
    run_with(args, &srtm, &mut |text: &str| print!("{text}"))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use super::*;
    use std::path::{Path, PathBuf};
    use std::sync::Arc;

    struct Offline;

    impl mp_terrain::Http for Offline {
        fn get(&self, _url: &str) -> Result<Vec<u8>, mp_terrain::HttpError> {
            Err(mp_terrain::HttpError("offline".to_owned()))
        }
    }

    fn testdata(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("mpr-georef-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// The photos and a log copied into a fresh folder, and the S28E153 tile the oracle's
    /// footprints used.
    fn setup(name: &str, log: &str) -> (PathBuf, PathBuf, mp_terrain::Srtm) {
        let dir = scratch(name);
        let photos = dir.join("photos");
        std::fs::create_dir_all(&photos).unwrap();
        for entry in std::fs::read_dir(testdata("georef/photos")).unwrap() {
            let path = entry.unwrap().path();
            std::fs::copy(&path, photos.join(path.file_name().unwrap())).unwrap();
        }
        let log_path = photos.join(log);
        std::fs::copy(testdata("georef").join(log), &log_path).unwrap();
        let srtm_dir = dir.join("srtm");
        std::fs::create_dir_all(&srtm_dir).unwrap();
        let zip = std::fs::read(testdata("srtm/S28E153.hgt.zip")).unwrap();
        mp_log::zip::extract(&zip, &srtm_dir).unwrap();
        let srtm = mp_terrain::Srtm::without_thread(&srtm_dir, Arc::new(Offline));
        (photos, log_path, srtm)
    }

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| (*s).to_owned()).collect()
    }

    #[test]
    fn process_and_geotag_write_what_mission_planner_writes() {
        let (photos, log, srtm) = setup("cam", "camera.bin");
        let mut lines = String::new();
        let code = run_with(
            &args(&[log.to_str().unwrap(), photos.to_str().unwrap(), "--geotag"]),
            &srtm,
            &mut |t: &str| lines.push_str(t),
        );
        assert_eq!(code, ExitCode::SUCCESS);
        let golden = testdata("georef/golden/cam-amsl");
        for name in [
            "location.txt",
            "location.csv",
            "location.kml",
            "location.tel",
            "location.geo",
            "location.jxl",
            "location.gpx",
            "loglocation.csv",
        ] {
            assert_eq!(
                std::fs::read(photos.join(name)).unwrap(),
                std::fs::read(golden.join(name)).unwrap(),
                "{name}"
            );
        }
        for n in [1, 13, 24] {
            let name = format!("IMG_{n:04}_geotag.jpg");
            assert_eq!(
                std::fs::read(photos.join("geotagged").join(&name)).unwrap(),
                std::fs::read(golden.join("geotagged").join(&name)).unwrap(),
                "{name}"
            );
        }
        let want = std::fs::read_to_string(golden.join("messages.txt")).unwrap();
        assert_eq!(lines.replace(photos.to_str().unwrap(), "{dir}"), want);
    }

    #[test]
    fn time_offset_and_estimate_from_the_command_line() {
        let (photos, log, srtm) = setup("time", "camera.tlog");
        let mut lines = String::new();
        let code = run_with(
            &args(&[
                log.to_str().unwrap(),
                photos.to_str().unwrap(),
                "--mode",
                "time",
                "--offset",
                "36003.3",
                "--relalt",
            ]),
            &srtm,
            &mut |t: &str| lines.push_str(t),
        );
        assert_eq!(code, ExitCode::SUCCESS);
        let golden = testdata("georef/golden/time-tlog");
        assert_eq!(
            std::fs::read(photos.join("location.txt")).unwrap(),
            std::fs::read(golden.join("location.txt")).unwrap()
        );
        assert_eq!(
            std::fs::read(photos.join("camera.tlog.xml")).unwrap(),
            std::fs::read(golden.join("camera.tlog.xml")).unwrap()
        );

        let (photos, log, srtm) = setup("estimate", "camera.bin");
        let mut lines = String::new();
        let code = run_with(
            &args(&[
                log.to_str().unwrap(),
                photos.to_str().unwrap(),
                "--estimate",
            ]),
            &srtm,
            &mut |t: &str| lines.push_str(t),
        );
        assert_eq!(code, ExitCode::SUCCESS);
        assert_eq!(
            lines,
            std::fs::read_to_string(testdata("georef/golden/estimate.txt")).unwrap()
        );
    }

    #[test]
    fn a_bad_offset_is_the_forms_message_and_bad_options_are_usage() {
        let (photos, log, srtm) = setup("badoffset", "camera.bin");
        let mut lines = String::new();
        let code = run_with(
            &args(&[
                log.to_str().unwrap(),
                photos.to_str().unwrap(),
                "--mode",
                "time",
                "--offset",
                "36003,3",
            ]),
            &srtm,
            &mut |t: &str| lines.push_str(t),
        );
        assert_eq!(code, ExitCode::FAILURE);
        assert_eq!(
            lines,
            "Offset number not in correct format. Use . as decimal separator\n"
        );
        assert!(parse(&args(&["a", "b", "--mode", "sideways"])).is_err());
        assert!(parse(&args(&["a"])).is_err());
        let request = parse(&args(&["a", "b/", "--lag", "150", "--gps2"])).unwrap();
        assert_eq!((request.lag, request.dir.as_str()), (150, "b"));
        assert_eq!(request.settings.gps(), "GPS2");
    }
}
