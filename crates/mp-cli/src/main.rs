//! `mpr` - a command line front end for the Mission Planner port.
//!
//! Exists so the stack can be exercised against a real vehicle long before there is a UI:
//! `mpr watch udp:14550` against SITL is the fastest way to find out whether the link, the
//! decoder and the state model actually work.

// A CLI prints; that is its job.
#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::time::{Duration, Instant};

mod dflog;
mod logs;

use mp_link::requests::RequestOutcome;
use mp_link::{Link, LinkConfig, RequestId, commands};
use mp_mavlink_dialects::all::MavMessage;
use mp_params::ParamTable;
use mp_params::param_file::{Change, ParamFile};
use mp_vehicle::{StateHandle, VehicleId};

fn main() -> std::process::ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("watch") => match args.get(1) {
            Some(url) => watch(url, args.get(2).and_then(|s| s.parse().ok())),
            None => {
                eprintln!("usage: mpr watch <url> [seconds]");
                std::process::ExitCode::from(2)
            }
        },
        Some("record") => match (args.get(1), args.get(2)) {
            (Some(url), Some(path)) => record(url, path, args.get(3).and_then(|s| s.parse().ok())),
            _ => {
                eprintln!("usage: mpr record <url> <out.tlog> [seconds]");
                std::process::ExitCode::from(2)
            }
        },
        Some("fly") => match args.get(1) {
            Some(url) => fly(url, args.get(2).map(String::as_str)),
            None => {
                eprintln!("usage: mpr fly <url> [out.tlog]");
                std::process::ExitCode::from(2)
            }
        },
        Some("params") => match args.get(1) {
            Some(url) => params(url, args.get(2).map(String::as_str)),
            None => {
                eprintln!("usage: mpr params <url> [NAME]");
                std::process::ExitCode::from(2)
            }
        },
        Some("logs") => match args.get(1) {
            Some(url) => logs(
                url,
                args.get(2).and_then(|id| id.parse().ok()),
                args.get(3).map_or(".", String::as_str),
            ),
            None => {
                eprintln!("usage: mpr logs <url> [ID] [OUT_DIR]");
                std::process::ExitCode::from(2)
            }
        },
        Some("param") => match (
            args.get(1).map(String::as_str),
            args.get(2),
            args.get(3),
            args.get(4).and_then(|value| value.parse::<f32>().ok()),
        ) {
            (Some("set"), Some(url), Some(name), Some(value)) => set_param(url, name, value),
            (Some("save"), Some(url), Some(path), _) => param_save(url, path),
            (Some("load"), Some(url), Some(path), _) => param_load(url, path),
            (Some("diff"), Some(current), Some(proposed), _) => param_diff(current, proposed),
            _ => {
                eprintln!(
                    "usage:\n  \
                     mpr param set  <url> <NAME> <VALUE>\n  \
                     mpr param save <url> <file.param>\n  \
                     mpr param load <url> <file.param>\n  \
                     mpr param diff <url|file.param> <file.param>"
                );
                std::process::ExitCode::from(2)
            }
        },
        Some("mission") => match (args.get(1), args.get(2)) {
            (Some(url), file) => mission(url, file.map(String::as_str)),
            _ => {
                eprintln!("usage: mpr mission <url> [file.waypoints]");
                std::process::ExitCode::from(2)
            }
        },
        Some("survey") => match (args.get(1), args.get(2)) {
            (Some(url), Some(out)) => survey(url, out, args.get(3).and_then(|s| s.parse().ok())),
            _ => {
                eprintln!("usage: mpr survey <url> <out.waypoints> [spacing_m]");
                std::process::ExitCode::from(2)
            }
        },
        Some("log") => match (args.get(1).map(String::as_str), args.get(2)) {
            (Some(verb), Some(file)) if dflog::VERBS.contains(&verb) => {
                dflog::run(verb, file, args.get(3).map(String::as_str))
            }
            (Some(verb), None) if dflog::VERBS.contains(&verb) => {
                eprintln!("usage: mpr log {verb} <file> [out]");
                std::process::ExitCode::from(2)
            }
            (Some(path), _) => logs::summarise(path),
            (None, _) => {
                eprintln!(
                    "usage:\n  \
                     mpr log <file.tlog|file.bin>              summarise a log\n  \
                     mpr log bintolog <file.bin> [out.log]     Convert .Bin to .Log\n  \
                     mpr log dflogtokml <file> [dir]           Create KML + gpx\n  \
                     mpr log matlab <file> [out.mat]           Create Matlab file\n  \
                     mpr log loganalysis <file> [dir]          Auto Analysis"
                );
                std::process::ExitCode::from(2)
            }
        },
        Some("firmware") => match (args.get(1).map(String::as_str), args.get(2)) {
            (Some("info"), Some(path)) => firmware_info(path),
            (Some("detect"), Some(port)) => firmware_detect(port),
            (Some("list"), _) => firmware_list(args.get(2..).unwrap_or_default()),
            _ => {
                eprintln!(
                    "usage:\n  \
                     mpr firmware info <file.apj>   describe a firmware file and its CRC\n  \
                     mpr firmware detect <port>     name the board from its USB ids, opening nothing\n  \
                     mpr firmware list [--board <id|name>] [--vehicle <type>] [--release <type>]\n    \
                     the firmware Mission Planner's catalogue would give a board\n\n\
                     Flashing is not offered here. It can brick a board and has no simulator, so\n\
                     it goes through the GUI where the board it is about to write to is on screen."
                );
                std::process::ExitCode::from(2)
            }
        },
        Some("fields") => match args.get(1) {
            Some(path) => logs::fields(path),
            None => {
                eprintln!("usage: mpr fields <log.bin>");
                std::process::ExitCode::from(2)
            }
        },
        Some("kml") => match (args.get(1), args.get(2)) {
            (Some(input), Some(out)) => logs::to_kml(input, out),
            _ => {
                eprintln!("usage: mpr kml <flight.tlog> <out.kml>");
                std::process::ExitCode::from(2)
            }
        },
        Some("ports") => ports(),
        Some("help" | "--help" | "-h") | None => {
            usage();
            std::process::ExitCode::SUCCESS
        }
        Some(other) => {
            eprintln!("unknown command: {other}\n");
            usage();
            std::process::ExitCode::from(2)
        }
    }
}

fn usage() {
    println!(
        "mpr - Mission Planner (Rust)\n\n\
         usage:\n  \
         mpr watch <url> [seconds]   connect and display live telemetry\n  \
         mpr record <url> <file> [s] record telemetry to a .tlog\n  \
         mpr fly <url> [file]        fly a scripted mission (simulator only)\n  \
         mpr params <url> [NAME]     download the parameter set, or show one parameter
  mpr param set <url> N V     set one parameter and read it back
  mpr param save <url> <file> save the parameter set to a .param file
  mpr param load <url> <file> write a .param file to the vehicle
  mpr param diff <a> <b>      compare a vehicle or file against a file\n  \
         mpr mission <url> [file]    download the mission, or upload one from a file\n  \
         mpr survey <url> <file>     generate a survey grid around the vehicle\n  \
         mpr log <file>              summarise a telemetry or dataflash log
  mpr log bintolog <bin> [out]   Convert .Bin to .Log, as the DataFlash Logs page does
  mpr log dflogtokml <log> [dir] Create KML + gpx (.kmz, .gpx, waypoints, .param, RINEX)
  mpr log matlab <log> [out]     Create Matlab file (.mat)
  mpr log loganalysis <log> [dir] Auto Analysis: run ArduPilot's LogAnalyzer, print its report
  mpr logs <url> [ID] [DIR]   list the vehicle's logs, or download one\n  \
         mpr fields <log.bin>        list what a dataflash log can plot
  mpr kml <log> <out.kml>     export a flown path for Google Earth
  mpr firmware info <file>    describe a .apj firmware file
  mpr firmware detect <port>  name the board from its USB ids
  mpr firmware list [...]     the firmware the catalogue would give a board
  mpr ports                   list serial ports\n\n\
         url forms:\n  \
         serial:/dev/ttyACM0:115200\n  \
         tcp:127.0.0.1:5760          (ArduPilot SITL)\n  \
         udp:14550                   (bind and wait for the vehicle)\n  \
         file:flight.tlog            (replay a recording)"
    );
}

fn ports() -> std::process::ExitCode {
    let ports = mp_transport::list_ports();
    if ports.is_empty() {
        println!("no serial ports found");
    }
    for port in &ports {
        println!("{}", port.label());
    }
    std::process::ExitCode::SUCCESS
}

fn watch(url: &str, seconds: Option<u64>) -> std::process::ExitCode {
    let config = LinkConfig::default();
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    println!("connected: {}", link.description());
    println!("waiting for telemetry...\n");

    let deadline = seconds.map(|s| Instant::now() + Duration::from_secs(s));
    let mut announced: Vec<VehicleId> = Vec::new();
    let mut last_draw = Instant::now();

    while link.is_running() {
        if let Some(deadline) = deadline
            && Instant::now() >= deadline
        {
            break;
        }
        std::thread::sleep(Duration::from_millis(50));

        for id in link.vehicles() {
            if !announced.contains(&id) {
                announced.push(id);
                println!("discovered vehicle {id}");
            }
        }

        if last_draw.elapsed() >= Duration::from_millis(500) {
            last_draw = Instant::now();
            draw(&link);
        }
    }

    println!("\nlink closed after {} frames", link.frames_received());
    std::process::ExitCode::SUCCESS
}

/// Flight mode and armed state, the two things a pilot checks first.
fn mode_label(state: &mp_vehicle::VehicleState) -> String {
    let mode = mp_vehicle::flight_mode_name(state.vehicle_type, state.custom_mode)
        .map_or_else(|| format!("mode {}", state.custom_mode), ToOwned::to_owned);
    if state.armed {
        format!("{mode} ARMED")
    } else {
        mode
    }
}

fn draw(link: &Link) {
    let Some((id, handle)) = link.primary_vehicle() else {
        return;
    };
    let s = handle.load();
    let stats = link.stats();

    let position = s.position.map_or_else(
        || "   no fix          ".to_owned(),
        |p| format!("{:>10.6},{:>11.6}", p.latitude(), p.longitude()),
    );

    println!(
        "{id} | {armed:<18} | {position} | alt {alt:>7.1} m | \
         spd {spd:>5.1} m/s | hdg {hdg:>5.1} | sats {sats:>2} fix {fix} | \
         batt {volts:>5.2} V | rx {rx} loss {loss:.1}% | crc_err {crc}",
        armed = mode_label(&s),
        alt = s.altitude_relative.0,
        spd = s.ground_speed.0,
        hdg = s.heading.degrees(),
        sats = s.gps.satellites_visible,
        fix = s.gps.fix_type,
        volts = s.battery.voltage,
        rx = s.link.received,
        loss = s.link.loss_percent(),
        crc = stats.decode.crc_errors,
    );
}

/// Records telemetry to a Mission Planner compatible `.tlog`.
fn record(url: &str, path: &str, seconds: Option<u64>) -> std::process::ExitCode {
    let config = LinkConfig {
        record_path: Some(path.into()),
        ..LinkConfig::default()
    };
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    println!("recording {} to {path}", link.description());
    let deadline = seconds.map(|s| Instant::now() + Duration::from_secs(s));
    let mut last = Instant::now();
    while link.is_running() {
        if deadline.is_some_and(|d| Instant::now() >= d) {
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
        if last.elapsed() >= Duration::from_secs(2) {
            last = Instant::now();
            // Both directions, because both are in the file. A count of only what arrived
            // reads as the file's frame count and is not: replaying the recording shows more
            // frames than the recorder claimed to have written.
            println!(
                "  {} frames recorded ({} received, {} sent)",
                link.frames_received() + link.stats().frames_sent,
                link.frames_received(),
                link.stats().frames_sent
            );
        }
    }
    println!(
        "recorded {} frames to {path}",
        link.frames_received() + link.stats().frames_sent
    );
    std::process::ExitCode::SUCCESS
}

/// Waits for a condition on vehicle state, printing progress.
fn await_state(
    handle: &StateHandle,
    what: &str,
    timeout: Duration,
    mut check: impl FnMut(&mp_vehicle::VehicleState) -> bool,
) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if check(&handle.load()) {
            println!("  {what}: ok");
            return true;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    eprintln!("  {what}: TIMED OUT after {timeout:?}");
    false
}

/// Flies a scripted mission. Intended for a simulator: it exists to produce flight data with real
/// dynamics, which recorded ground tests do not contain.
fn fly(url: &str, record_path: Option<&str>) -> std::process::ExitCode {
    let config = LinkConfig {
        record_path: record_path.map(Into::into),
        stream_rate_hz: 10,
        ..LinkConfig::default()
    };
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    println!("connected: {}", link.description());

    // Wait for the vehicle to appear and settle.
    let deadline = Instant::now() + Duration::from_secs(30);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let Some((id, handle)) = link.primary_vehicle() else {
        eprintln!("no vehicle appeared");
        return std::process::ExitCode::FAILURE;
    };
    println!("vehicle {id}");

    if !await_state(&handle, "3D GPS fix", Duration::from_secs(60), |s| {
        s.gps.has_3d_fix()
    }) {
        return std::process::ExitCode::FAILURE;
    }
    let Some(home) = handle.load().position else {
        eprintln!("no position");
        return std::process::ExitCode::FAILURE;
    };
    println!("  home: {:.7}, {:.7}", home.latitude(), home.longitude());

    // Let the EKF settle before asking to arm; ArduPilot refuses otherwise.
    println!("  waiting for the EKF to settle");
    std::thread::sleep(Duration::from_secs(12));

    // Sent and not waited for, as `setMode` sends it: the mode in the next heartbeat is the answer.
    // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4631-4641`
    println!("mode GUIDED");
    link.send(&commands::set_mode(id, commands::copter_mode::GUIDED));
    std::thread::sleep(Duration::from_millis(500));

    // `doARM`: the command sent again until the vehicle answers it, ten seconds a try.
    println!("arming");
    run_command(&link, &commands::arm(id, true, false));
    if !await_state(&handle, "armed", Duration::from_secs(20), |s| s.armed) {
        // Retry once: a first arm attempt often lands while a pre-arm check is still failing.
        println!("  retrying arm");
        run_command(&link, &commands::arm(id, true, false));
        if !await_state(&handle, "armed", Duration::from_secs(30), |s| s.armed) {
            return std::process::ExitCode::FAILURE;
        }
    }

    println!("takeoff to 40 m");
    run_command(&link, &commands::takeoff(id, 40.0));
    await_state(&handle, "reached 35 m", Duration::from_secs(60), |s| {
        s.altitude_relative.0 > 35.0
    });

    // Fly a box, which produces the roll, pitch and heading changes a ground test never does.
    let legs = [(0.0, 300.0), (90.0, 300.0), (180.0, 300.0), (270.0, 300.0)];
    for (bearing, metres) in legs {
        let target = home.offset(
            mp_units::Bearing(mp_units::Degrees(bearing)),
            mp_units::Metres(metres),
        );
        println!("leg: bearing {bearing}, {metres} m");
        // Sent and not waited for, as `setGuidedModeWP` sends its position target.
        // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4449-4454`
        link.send(&commands::goto_position(
            id,
            target.latitude(),
            target.longitude(),
            40.0,
        ));
        await_state(&handle, "leg complete", Duration::from_secs(90), |s| {
            s.position.is_some_and(|p| p.distance_to(target).0 < 25.0)
        });
    }

    println!("returning and landing");
    link.send(&commands::set_mode(id, commands::copter_mode::RTL));
    await_state(
        &handle,
        "disarmed after landing",
        Duration::from_secs(180),
        |s| !s.armed,
    );

    println!("flight complete: {} frames", link.frames_received());
    std::process::ExitCode::SUCCESS
}

/// Waits for a request made just now to end: bounded by the link's own retries, which end every
/// request. `None` if the link lets it go or stops first.
fn await_request(link: &Link, id: RequestId) -> Option<RequestOutcome> {
    /// How long a request may be missing before it is taken as let go. The link moves what it
    /// picks up from its queue into its table in two steps, with neither lock held in between
    /// (mp-link's `run_link`), and in that moment `Link::request` finds it in neither.
    const PICKUP_GRACE: Duration = Duration::from_millis(500);

    let made = Instant::now();
    loop {
        match link.request(id) {
            Some(request) => {
                if let Some(outcome) = request.outcome() {
                    return Some(outcome);
                }
            }
            None if made.elapsed() < PICKUP_GRACE => {}
            None => return None,
        }
        if !link.is_running() {
            return None;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// `doCommand`: a built `COMMAND_LONG` sent to the vehicle it names and sent again until that
/// vehicle acknowledges it - three more times, two seconds apart; ten seconds a try for arming -
/// and how it ended said on stderr when it did not end well. Anything but a `COMMAND_LONG` is
/// not sent.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2688-2836`
fn run_command(link: &Link, message: &MavMessage) -> Option<RequestOutcome> {
    let MavMessage::CommandLong(long) = message else {
        return None;
    };
    let id = link.command(
        VehicleId::new(long.target_system, long.target_component),
        long.command,
        [
            long.param1,
            long.param2,
            long.param3,
            long.param4,
            long.param5,
            long.param6,
            long.param7,
        ],
        true,
    );
    let outcome = await_request(link, id);
    let name = mp_mavlink_dialects::all::MavCmd(u32::from(long.command))
        .name()
        .map_or_else(|| format!("command {}", long.command), ToOwned::to_owned);
    match outcome {
        Some(RequestOutcome::Rejected(result)) => eprintln!(
            "  {name}: {}",
            mp_link::messages::command_result_name(result)
        ),
        // `doCommand`'s `TimeoutException`. `// C#: MAVLinkInterface.cs:2797`
        Some(RequestOutcome::TimedOut) | None => {
            eprintln!("  {name}: Timeout on read - doCommand");
        }
        Some(_) => {}
    }
    outcome
}

/// How a parameter write ended.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Written {
    /// The vehicle echoed it, and now holds this.
    Holds(f64),
    /// The vehicle has no parameter of that name.
    NotOnVehicle,
    /// Every retry went unanswered: `setParam`'s "Timeout on read - setParam".
    NoEcho,
}

/// `setParam`: `PARAM_SET` until the vehicle echoes the parameter, sent again every 700 ms up to
/// three times by the link. `setParam` sends only a name the vehicle has listed, and the C#
/// holds the whole list by the time anything writes; `mpr` connects and writes at once, so a
/// name not yet heard of is read first - `GetParam`, retried the same way - and one the vehicle
/// never reports is not on it.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1638-1770, 2296-2380`
fn write_param(link: &Link, id: VehicleId, name: &str, value: f64) -> Written {
    let listed = link
        .params(id)
        .is_some_and(|table| table.get(name).is_some());
    if !listed
        && !matches!(
            await_request(link, link.read_param(id, name)),
            Some(RequestOutcome::Accepted { .. })
        )
    {
        return Written::NotOnVehicle;
    }
    match await_request(link, link.set_param(id, name, value, false)) {
        Some(RequestOutcome::Accepted { value: Some(held) }) => Written::Holds(held.as_f64()),
        Some(RequestOutcome::Unchanged) => Written::Holds(value),
        Some(RequestOutcome::UnknownParameter) => Written::NotOnVehicle,
        _ => Written::NoEcho,
    }
}

/// Whether what the vehicle holds is what was written, at the precision the wire carries.
fn holds_what_was_written(held: f64, value: f64) -> bool {
    #[allow(clippy::cast_possible_truncation)] // parameters are f32 on the wire
    let (held, value) = (held as f32, value as f32);
    (held - value).abs() <= f32::EPSILON * value.abs().max(1.0)
}

/// Downloads the vehicle's parameters and prints them.
///
/// Useful on its own, and the fastest way to check the download protocol against a real vehicle:
/// a parameter set with a hole in it is a protocol bug, and the count is printed so a hole is
/// visible rather than implied.
/// Lists the vehicle's dataflash logs, or downloads one.
fn logs(url: &str, wanted: Option<u16>, out_dir: &str) -> std::process::ExitCode {
    let config = LinkConfig {
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    println!("connected: {}", link.description());

    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let Some((id, _)) = link.primary_vehicle() else {
        eprintln!("no vehicle appeared on {url}");
        return std::process::ExitCode::FAILURE;
    };

    link.request_log_list(id);
    let deadline = Instant::now() + Duration::from_secs(30);
    while link.log_listings().is_empty() {
        if Instant::now() > deadline {
            eprintln!("the vehicle listed no logs");
            return std::process::ExitCode::FAILURE;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    let listings = link.log_listings();

    let Some(wanted) = wanted else {
        println!("{:>5}  {:>12}  started", "id", "bytes");
        for listing in &listings {
            let when = if listing.has_timestamp() {
                // Seconds since the epoch, as the vehicle reported them. Not converted to a local
                // calendar date here: that needs a timezone database, and the number is enough to
                // tell one flight from another.
                format!("utc {}", listing.time_utc)
            } else {
                // A flight controller with no GPS fix and no clock reports zero. Showing
                // "1 January 1970" for every log is less useful than saying we do not know.
                "no clock".to_owned()
            };
            println!("{:>5}  {:>12}  {when}", listing.id, listing.size);
        }
        return std::process::ExitCode::SUCCESS;
    };

    let Some(listing) = listings.iter().find(|listing| listing.id == wanted) else {
        eprintln!("the vehicle has no log {wanted}");
        return std::process::ExitCode::FAILURE;
    };

    println!("downloading log {} ({} bytes)", listing.id, listing.size);
    link.download_log(id, listing.id, listing.size);

    let mut last_report = Instant::now();
    let stall_deadline = Duration::from_secs(60);
    let mut last_progress = (Instant::now(), 0u32);
    loop {
        if let Some((_, bytes)) = link.finished_log() {
            let path = std::path::Path::new(out_dir).join(format!("log_{}.bin", listing.id));
            return match std::fs::write(&path, &bytes) {
                Ok(()) => {
                    println!("wrote {} ({} bytes)", path.display(), bytes.len());
                    std::process::ExitCode::SUCCESS
                }
                Err(err) => {
                    eprintln!("could not write {}: {err}", path.display());
                    std::process::ExitCode::FAILURE
                }
            };
        }

        let (_, filled, size) = link.log_download_progress().unwrap_or((0, 0, 0));
        if filled > last_progress.1 {
            last_progress = (Instant::now(), filled);
        } else if last_progress.0.elapsed() > stall_deadline {
            eprintln!("the download stalled at {filled} of {size} bytes");
            return std::process::ExitCode::FAILURE;
        }
        if last_report.elapsed() > Duration::from_secs(2) {
            last_report = Instant::now();
            println!("  {filled} of {size} bytes");
        }

        link.nudge_log_download(id);
        std::thread::sleep(Duration::from_millis(400));
    }
}

/// Sets one parameter and reads it back.
///
/// Reading back is the point. `PARAM_SET` has no acknowledgement of its own - the vehicle answers
/// by broadcasting the parameter's new value, and a write that was rejected or clamped looks
/// exactly like one that worked until you look. ArduPilot silently clamps out-of-range values.
///
/// The write is `setParam`'s: sent again until that echo comes, and the echo's value is what the
/// vehicle holds (see [`write_param`]).
fn set_param(url: &str, name: &str, value: f32) -> std::process::ExitCode {
    let config = LinkConfig {
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    println!("connected: {}", link.description());

    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let Some((id, _)) = link.primary_vehicle() else {
        eprintln!("no vehicle appeared on {url}");
        return std::process::ExitCode::FAILURE;
    };

    println!("setting {name} = {value} on vehicle {id}");
    match write_param(&link, id, name, f64::from(value)) {
        Written::Holds(held) if holds_what_was_written(held, f64::from(value)) => {
            println!("{name} = {held}");
            std::process::ExitCode::SUCCESS
        }
        // The vehicle answered; it just did not accept what was asked. ArduPilot silently clamps
        // out-of-range values, so this is the common shape of a rejected write.
        Written::Holds(held) => {
            eprintln!("{name} = {held}; the vehicle did not accept {value}");
            std::process::ExitCode::FAILURE
        }
        Written::NotOnVehicle => {
            eprintln!(
                "the vehicle did not report {name} back; \
                 it may not exist on this firmware"
            );
            std::process::ExitCode::FAILURE
        }
        // `setParam`'s `TimeoutException`. `// C#: MAVLinkInterface.cs:1765`
        Written::NoEcho => {
            eprintln!("Timeout on read - setParam {name}");
            std::process::ExitCode::FAILURE
        }
    }
}

/// Connects, waits for a vehicle, and downloads its whole parameter set.
///
/// Shared by every command that needs one, because they all need the same twenty-second wait for a
/// heartbeat, the same progress reporting, and the same tolerance for a download that ends a few
/// parameters short. A `None` return has already said why on stderr.
fn connect_and_download(url: &str) -> Option<(Link, mp_vehicle::VehicleId, ParamTable)> {
    let config = LinkConfig {
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return None;
        }
    };
    println!("connected: {}", link.description());

    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let Some((id, _)) = link.primary_vehicle() else {
        eprintln!("no vehicle appeared on {url}");
        return None;
    };

    println!("downloading parameters from vehicle {id}");
    link.download_params(id);

    // Progress is reported so a stalled download looks different from a slow one.
    let deadline = Instant::now() + Duration::from_secs(120);
    let mut last_report = Instant::now();
    let mut last_count = 0usize;
    loop {
        let Some(table) = link.params(id) else {
            std::thread::sleep(Duration::from_millis(200));
            continue;
        };
        if table.is_complete() {
            break;
        }
        if Instant::now() >= deadline {
            eprintln!(
                "timed out with {} of {:?} parameters, missing {} indices",
                table.len(),
                table.expected(),
                table.missing().len()
            );
            break;
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            let count = table.len();
            println!(
                "  {count} of {:?}  ({:.0}%){}",
                table.expected(),
                table.progress() * 100.0,
                if count == last_count {
                    "  - waiting on gaps"
                } else {
                    ""
                }
            );
            last_count = count;
            last_report = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    let Some(table) = link.params(id) else {
        eprintln!("no parameters received");
        return None;
    };
    Some((link, id, table))
}

fn params(url: &str, filter: Option<&str>) -> std::process::ExitCode {
    let Some((_link, _id, table)) = connect_and_download(url) else {
        return std::process::ExitCode::FAILURE;
    };

    match filter {
        Some(name) => match table.get(name) {
            Some(value) => {
                println!("{name} = {} ({:?})", value.as_f64(), value.param_type());
                // The number alone is rarely what someone needs; the documentation is the point.
                if let Some(meta) = mp_params::param_meta::lookup(name) {
                    if !meta.display_name.is_empty() {
                        println!("  {}", meta.display_name);
                    }
                    if !meta.units.is_empty() {
                        println!("  units: {}", meta.units);
                    }
                    if let Some((low, high)) = meta.range {
                        let ok = if meta.accepts(value.as_f64()) {
                            ""
                        } else {
                            "  <-- OUT OF RANGE"
                        };
                        println!("  range: {low} to {high}{ok}");
                    }
                    if meta.is_enumeration() {
                        #[allow(clippy::cast_possible_truncation)]
                        let current = value.as_f64() as i64;
                        let label = meta.value_name(current).unwrap_or("unnamed value");
                        println!("  value: {current} = {label}");
                        for (number, name) in meta.values {
                            println!("    {number:>6}  {name}");
                        }
                    }
                    if meta.is_bitmask() {
                        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                        let bits = value.as_f64().max(0.0) as u32;
                        let set = meta.bit_names(bits);
                        println!(
                            "  bits set: {}",
                            if set.is_empty() {
                                "none".to_owned()
                            } else {
                                set.join(", ")
                            }
                        );
                    }
                    if meta.reboot_required {
                        println!("  changing this requires a reboot");
                    }
                    if !meta.description.is_empty() {
                        println!("  {}", meta.description);
                    }
                }
            }
            None => {
                eprintln!("no parameter named {name} ({} received)", table.len());
                return std::process::ExitCode::FAILURE;
            }
        },
        None => {
            let mut undocumented = 0usize;
            let mut out_of_range = Vec::new();
            for (name, value) in table.iter() {
                let meta = mp_params::param_meta::lookup(name);
                let units = meta.map_or("", |m| m.units);
                if meta.is_none() {
                    undocumented += 1;
                }
                if let Some(meta) = meta
                    && !meta.accepts(value.as_f64())
                {
                    out_of_range.push(name.clone());
                }
                println!(
                    "{name:<17} {:>14}  {:<7} {units}",
                    value.as_f64(),
                    format!("{:?}", value.param_type())
                );
            }

            // Drift between the bundled metadata and the running firmware is normal and worth
            // stating plainly: parameters get renamed between releases, and a description that
            // no longer matches the firmware is worse than no description at all.
            if undocumented > 0 {
                println!(
                    "\n{undocumented} of {} parameters have no bundled documentation (the \
                     metadata and the firmware are different versions)",
                    table.len()
                );
            }
            if !out_of_range.is_empty() {
                println!(
                    "{} parameter(s) outside their documented range: {}",
                    out_of_range.len(),
                    out_of_range
                        .iter()
                        .take(6)
                        .cloned()
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
        }
    }
    println!(
        "\n{} names received, {} placed in the list, vehicle reported {:?}, complete: {}",
        table.len(),
        table.indexed_len(),
        table.expected(),
        table.is_complete()
    );
    let unindexed = table.unindexed();
    if !unindexed.is_empty() {
        // Not an error: a vehicle can answer a by-name read with index 65535, and ArduPilot's
        // reported count does not always match the names it sends. Printed so the difference is
        // explainable rather than mysterious.
        println!(
            "{} name(s) arrived without a list index: {}",
            unindexed.len(),
            unindexed
                .iter()
                .take(8)
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    std::process::ExitCode::SUCCESS
}

/// Downloads the vehicle's mission, or uploads one from a `.waypoints` file and reads it back.
///
/// Reading back after an upload is the point: it is the only way to know the vehicle stored what
/// was sent, rather than what it felt like storing.
fn mission(url: &str, file: Option<&str>) -> std::process::ExitCode {
    // Telemetry is requested even though this command displays none, because the most
    // valuable validation check - is this mission built for somewhere else entirely? - needs
    // the vehicle's position. A check that silently skips is worse than one that is absent.
    let config = LinkConfig {
        stream_rate_hz: 2,
        ..LinkConfig::default()
    };
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    println!("connected: {}", link.description());

    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(100));
    }
    let Some((id, _)) = link.primary_vehicle() else {
        eprintln!("no vehicle appeared on {url}");
        return std::process::ExitCode::FAILURE;
    };

    // Wait briefly for a position so validation has a home to compare against.
    let deadline = Instant::now() + Duration::from_secs(15);
    while Instant::now() < deadline
        && link
            .primary_vehicle()
            .is_none_or(|(_, handle)| handle.load().position.is_none())
    {
        std::thread::sleep(Duration::from_millis(200));
    }

    // Upload first, if a file was given.
    if let Some(path) = file {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(err) => {
                eprintln!("could not read {path}: {err}");
                return std::process::ExitCode::FAILURE;
            }
        };
        let items = match mp_mission::read_waypoints(&text) {
            Ok(items) => items,
            Err(err) => {
                eprintln!("{path}: {err}");
                return std::process::ExitCode::FAILURE;
            }
        };
        // Validate before sending, and report it, but do not refuse. A ground station that
        // second-guesses the pilot is one they work around; a ground station that stays silent
        // about a first waypoint on another continent is one they should not trust.
        let context = mp_mission::validate::Context {
            home: link
                .primary_vehicle()
                .and_then(|(_, handle)| handle.load().position),
            vehicle_type: link
                .primary_vehicle()
                .map(|(_, handle)| handle.load().vehicle_type),
        };
        if context.home.is_none() {
            println!("  no position from the vehicle yet; site and distance checks skipped");
        }
        let findings = mp_mission::validate::validate_with(&items, context);
        for finding in &findings {
            let marker = match finding.severity {
                mp_mission::Severity::Danger => "DANGER ",
                mp_mission::Severity::Warning => "warning",
                mp_mission::Severity::Note => "note   ",
            };
            match finding.seq {
                Some(seq) => println!("  {marker} item {seq}: {}", finding.message),
                None => println!("  {marker} {}", finding.message),
            }
        }

        println!("uploading {} items from {path}", items.len());
        link.upload_mission(id, items);
        if !await_transfer(&link, id, "upload") {
            return std::process::ExitCode::FAILURE;
        }
    }

    println!("downloading mission from vehicle {id}");
    link.download_mission(id);
    if !await_transfer(&link, id, "download") {
        return std::process::ExitCode::FAILURE;
    }

    let Some(transfer) = link.mission_transfer(id) else {
        eprintln!("no transfer state");
        return std::process::ExitCode::FAILURE;
    };
    let items = transfer.items();
    println!("\nseq  cmd  frame        lat           lon          alt");
    for item in items {
        println!(
            "{:>3}  {:>3}  {:>5}  {:>12.7}  {:>12.7}  {:>8.2}",
            item.seq, item.command, item.frame, item.x, item.y, item.z
        );
    }
    println!("\n{} items", items.len());

    // Print the file form too, so it can be diffed against what was uploaded.
    if file.is_some() {
        print!("{}", mp_mission::write_waypoints(items));
    }
    std::process::ExitCode::SUCCESS
}

/// Waits for a mission transfer to finish, reporting progress.
fn await_transfer(link: &Link, id: VehicleId, what: &str) -> bool {
    let deadline = Instant::now() + Duration::from_secs(60);
    let mut last_report = Instant::now();
    loop {
        let Some(transfer) = link.mission_transfer(id) else {
            std::thread::sleep(Duration::from_millis(100));
            if Instant::now() >= deadline {
                eprintln!("{what}: never started");
                return false;
            }
            continue;
        };
        match transfer.state() {
            mp_link::mission_transfer::TransferState::Complete => {
                println!("  {what}: complete, {} items", transfer.items().len());
                return true;
            }
            mp_link::mission_transfer::TransferState::Failed(err) => {
                eprintln!("  {what} failed: {err}");
                return false;
            }
            _ => {}
        }
        if Instant::now() >= deadline {
            eprintln!("  {what}: timed out at {:.0}%", transfer.progress() * 100.0);
            return false;
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            println!("  {what}: {:.0}%", transfer.progress() * 100.0);
            last_report = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Generates a survey grid around the vehicle's current position and writes it as a mission.
///
/// Anchoring the survey on the vehicle rather than on a typed coordinate is the point: it makes
/// the generated area somewhere the aircraft can actually fly, which is what makes this useful for
/// testing the whole chain from grid to upload.
fn survey(url: &str, out_path: &str, spacing: Option<f64>) -> std::process::ExitCode {
    let config = LinkConfig {
        stream_rate_hz: 4,
        ..LinkConfig::default()
    };
    let link = match Link::connect(url, config) {
        Ok(link) => link,
        Err(err) => {
            eprintln!("could not open {url}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    let deadline = Instant::now() + Duration::from_secs(60);
    let mut centre = None;
    while Instant::now() < deadline && centre.is_none() {
        if let Some((_, handle)) = link.primary_vehicle() {
            centre = handle.load().position;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    let Some(centre) = centre else {
        eprintln!("no position from {url}; a survey needs somewhere to be");
        return std::process::ExitCode::FAILURE;
    };
    println!(
        "centre: {:.7}, {:.7}",
        centre.latitude(),
        centre.longitude()
    );

    // A square about 400 m on a side around the vehicle.
    const HALF_SIDE_M: f64 = 200.0;
    let corner = |bearing: f64| {
        centre.offset(
            mp_units::Bearing(mp_units::Degrees(bearing)),
            mp_units::Metres(HALF_SIDE_M * std::f64::consts::SQRT_2),
        )
    };
    let area = vec![corner(45.0), corner(135.0), corner(225.0), corner(315.0)];

    let options = mp_mission::GridOptions {
        spacing: spacing.unwrap_or(40.0),
        angle: 0.0,
        overshoot: 10.0,
        altitude: 50.0,
    };
    let waypoints = match mp_mission::grid(&area, &options) {
        Ok(points) => points,
        Err(err) => {
            eprintln!("could not generate a grid: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    println!(
        "generated {} survey waypoints at {} m spacing",
        waypoints.len(),
        options.spacing
    );

    // Item 0 is home, as every mission file expects.
    let mut items = vec![mp_mission::MissionItem {
        seq: 0,
        current: 1,
        frame: mp_mission::item::MAV_FRAME_GLOBAL,
        command: mp_mission::item::MAV_CMD_NAV_WAYPOINT,
        x: centre.latitude(),
        y: centre.longitude(),
        ..mp_mission::MissionItem::default()
    }];
    for (index, point) in waypoints.iter().enumerate() {
        items.push(mp_mission::MissionItem {
            seq: u16::try_from(index + 1).unwrap_or(u16::MAX),
            command: mp_mission::item::MAV_CMD_NAV_WAYPOINT,
            x: point.latitude(),
            y: point.longitude(),
            z: options.altitude,
            ..mp_mission::MissionItem::default()
        });
    }

    if let Err(err) = std::fs::write(out_path, mp_mission::write_waypoints(&items)) {
        eprintln!("could not write {out_path}: {err}");
        return std::process::ExitCode::FAILURE;
    }
    println!("wrote {} items to {out_path}", items.len());
    std::process::ExitCode::SUCCESS
}

/// Downloads the parameter set and writes it to a `.param` file.
fn param_save(url: &str, path: &str) -> std::process::ExitCode {
    let Some((_link, _id, table)) = connect_and_download(url) else {
        return std::process::ExitCode::FAILURE;
    };
    let received = table.len();
    let file = ParamFile::from_values(table.iter().map(|(name, value)| (name, value.as_f64())));
    if let Err(err) = file.save(std::path::Path::new(path)) {
        eprintln!("could not write {path}: {err}");
        return std::process::ExitCode::FAILURE;
    }
    // Everything the vehicle reported goes in, because that is what Mission Planner's
    // `SaveParamFile` does - the filtering happens when a file is read, not when it is written, so
    // a saved file is a complete record of the airframe. Which of them will be skipped on the way
    // back in is said here, so a later "why did that not apply" has an answer.
    println!("wrote {} of {received} parameters to {path}", file.len());
    let skipped_on_load: Vec<_> = table
        .iter()
        .map(|(name, _)| name.as_str())
        .filter(|name| !mp_params::param_file::is_loaded(name))
        .collect();
    if !skipped_on_load.is_empty() {
        println!(
            "{} in the file will be skipped when it is loaded, being state the vehicle keeps for \
             itself: {}",
            skipped_on_load.len(),
            skipped_on_load.join(", ")
        );
    }
    std::process::ExitCode::SUCCESS
}

/// Compares a `.param` file against the vehicle, or against a second file.
///
/// The vehicle, or the first file, is the current state; the second argument is the proposal. So
/// the output reads as what would change if the proposal were loaded.
fn param_diff(current: &str, proposed: &str) -> std::process::ExitCode {
    let proposed_file = match ParamFile::load(std::path::Path::new(proposed)) {
        Ok(file) => file,
        Err(err) => {
            eprintln!("could not read {proposed}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    report_rejected(proposed, &proposed_file);

    // A url is a vehicle and anything else is a file. Distinguished by the scheme rather than by
    // trying to open it as both, so a typo in a filename is a missing file rather than a
    // twenty-second wait for a vehicle that was never going to appear.
    // What the left-hand side is called in the output. "not on the vehicle" is wrong when the
    // comparison is between two files, and it is the line an operator reads to decide whether a
    // parameter is missing or merely from a different firmware.
    let absent_from_current = if is_url(current) {
        "not on the vehicle".to_owned()
    } else {
        format!("not in {current}")
    };
    let current_file = if is_url(current) {
        let Some((_link, _id, table)) = connect_and_download(current) else {
            return std::process::ExitCode::FAILURE;
        };
        ParamFile::from_values(table.iter().map(|(name, value)| (name, value.as_f64())))
    } else {
        match ParamFile::load(std::path::Path::new(current)) {
            Ok(file) => {
                report_rejected(current, &file);
                file
            }
            Err(err) => {
                eprintln!("could not read {current}: {err}");
                return std::process::ExitCode::FAILURE;
            }
        }
    };

    let differences = current_file.compare(&proposed_file);
    if differences.is_empty() {
        println!(
            "no differences ({} parameters compared)",
            current_file.len()
        );
        return std::process::ExitCode::SUCCESS;
    }
    println!(
        "{} differences, reading as \"{current} -> {proposed}\":\n",
        differences.len()
    );
    for difference in &differences {
        // The documentation is what turns a number into a decision. Somebody comparing a suggested
        // tune against their own wants to know what ATC_RAT_PIT_D is before changing it.
        let meta = mp_params::param_meta::lookup(&difference.name);
        let units = meta.map_or("", |m| m.units);
        let units = if units.is_empty() {
            String::new()
        } else {
            format!(" {units}")
        };
        match difference.kind {
            Change::Changed { from, to } => {
                println!("  {:<17} {from}{units} -> {to}{units}", difference.name);
            }
            Change::Added { to } => {
                println!(
                    "  {:<17} {absent_from_current} -> {to}{units}",
                    difference.name
                );
            }
            Change::Missing { from } => {
                println!(
                    "  {:<17} {from}{units} -> not in {proposed}",
                    difference.name
                );
            }
        }
        if let Some(meta) = meta
            && !meta.display_name.is_empty()
        {
            println!("                    {}", meta.display_name);
        }
    }
    std::process::ExitCode::SUCCESS
}

/// Loads a `.param` file onto the vehicle, writing only what differs.
///
/// Only what differs, because writing all fourteen hundred takes minutes and wears the flash for
/// no reason - and because a write that changes nothing still produces a `PARAM_VALUE` the
/// operator has to read past to find the ones that mattered.
fn param_load(url: &str, path: &str) -> std::process::ExitCode {
    let proposed = match ParamFile::load(std::path::Path::new(path)) {
        Ok(file) => file,
        Err(err) => {
            eprintln!("could not read {path}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    report_rejected(path, &proposed);
    if proposed.is_empty() {
        eprintln!("{path} holds no parameters");
        return std::process::ExitCode::FAILURE;
    }

    let Some((link, id, table)) = connect_and_download(url) else {
        return std::process::ExitCode::FAILURE;
    };
    let current = ParamFile::from_values(table.iter().map(|(name, value)| (name, value.as_f64())));

    let differences = current.compare(&proposed);
    let to_write: Vec<_> = differences
        .iter()
        .filter_map(|difference| match difference.kind {
            Change::Changed { to, .. } => Some((difference.name.as_str(), to)),
            // A parameter in the file that the vehicle does not have cannot be written: the
            // firmware decides what exists. Reported rather than attempted, because an attempt
            // fails silently - ArduPilot ignores a set for an unknown name.
            Change::Added { .. } | Change::Missing { .. } => None,
        })
        .collect();

    for difference in &differences {
        if let Change::Added { .. } = difference.kind {
            println!(
                "skipping {}: the vehicle has no such parameter",
                difference.name
            );
        }
    }
    if to_write.is_empty() {
        println!("the vehicle already matches {path}");
        return std::process::ExitCode::SUCCESS;
    }

    println!("writing {} parameters", to_write.len());
    let mut failed = Vec::new();
    for (name, value) in to_write {
        // One at a time, each `setParam` waiting for its echo and sent again until it comes, as
        // the C#'s Write Params loop makes them. ArduPilot accepts a burst and drops most of it; a
        // loader that does not wait for the echo reports success for a vehicle it changed a third
        // of. `// C#: GCSViews/ConfigurationView/ConfigRawParams.cs:313-363`
        match write_param(&link, id, name, value) {
            Written::Holds(held) if holds_what_was_written(held, value) => {
                println!("  {name} = {held}");
            }
            Written::Holds(held) => {
                failed.push(name.to_owned());
                eprintln!("  {name} = {held}; the vehicle did not accept {value}");
            }
            Written::NotOnVehicle | Written::NoEcho => {
                failed.push(name.to_owned());
                eprintln!("  Set {name} Failed");
            }
        }
    }

    if failed.is_empty() {
        println!("done");
        std::process::ExitCode::SUCCESS
    } else {
        eprintln!(
            "{} parameters were not confirmed: {}",
            failed.len(),
            failed.join(", ")
        );
        std::process::ExitCode::FAILURE
    }
}

/// Prints the lines of a file that did not become parameters.
///
/// Bounded, because a file that is not a parameter file at all produces one of these per line and
/// burying the useful output under a thousand of them helps nobody.
fn report_rejected(path: &str, file: &ParamFile) {
    const SHOWN: usize = 10;
    let rejected = file.rejected();
    if rejected.is_empty() {
        return;
    }
    eprintln!("{path}: {} lines were not read", rejected.len());
    for line in rejected.iter().take(SHOWN) {
        eprintln!("  line {}: {} - {}", line.line, line.reason, line.text);
    }
    if rejected.len() > SHOWN {
        eprintln!("  ... and {} more", rejected.len() - SHOWN);
    }
}

/// Whether an argument names a vehicle rather than a file.
fn is_url(argument: &str) -> bool {
    ["serial:", "tcp:", "udp:", "udpout:", "file:", "COM"]
        .iter()
        .any(|scheme| argument.starts_with(scheme))
        || argument.starts_with("/dev/")
}

/// Names the board as Mission Planner's `BoardDetect.DetectBoard` would from USB descriptors alone.
///
/// Runs the two steps of `DetectBoard` that read only the enumeration - the device list, then the
/// `Win32_SerialPort` table built from the same ports - and opens nothing. What the C# does after
/// them is said, not done: its replug probe and STK500 probes write to the port, and the rest are
/// questions for the operator.
fn firmware_detect(port: &str) -> std::process::ExitCode {
    let ports = mp_transport::list_ports();
    print!("{}", detect_report(port, &ports));
    std::process::ExitCode::SUCCESS
}

/// The text `mpr firmware detect` prints for an enumeration.
fn detect_report(port: &str, ports: &[mp_transport::PortInfo]) -> String {
    use mp_firmware::detect::{DeviceInfo, Probe, Verdict, match_ports};
    use std::fmt::Write as _;

    // The USB devices, which are the ones Mission Planner's device list holds.
    let mut out = String::new();
    for (listed, device) in ports
        .iter()
        .filter_map(|listed| Some((listed, DeviceInfo::from_port(listed)?)))
    {
        let _ = writeln!(
            out,
            "  {}  {}  {}",
            listed.name,
            device.hardwareid.as_deref().unwrap_or_default(),
            device.board.as_deref().unwrap_or_default()
        );
    }
    // Every device counts, not just the one on `port`: the C# compares none of them with it.
    let _ = match match_ports(port, ports) {
        Verdict::Board(found) => match found.chbootloader {
            Some(name) => writeln!(out, "detected: {} ({name})", found.board),
            None => writeln!(out, "detected: {}", found.board),
        },
        Verdict::Probe(probe) => writeln!(
            out,
            "probe: Mission Planner would ask for a replug and read the bootloader's board id \
             for 30 s, to tell {} (not done here)",
            match probe {
                Probe::FmuV2OrV3 => "px4v3 from px4v2",
                Probe::ChibiosOrPx4 => "px4v3, fmuv5 and px4v2 apart",
            }
        ),
        Verdict::NoMatch => writeln!(
            out,
            "not recognised from USB ids: Mission Planner would go on to ask, and to probe {port} \
             for an STK500 bootloader (not done here)"
        ),
    };
    out
}

/// `mpr firmware list`: what the Install Firmware page would offer, from the same catalogue.
///
/// The catalogue is fetched as the page fetches it - the mirror, then ardupilot.org, then
/// CubePilot's peripherals appended - on every run, because Mission Planner keeps it in memory
/// and never on disk (`mp_firmware::manifest`). `MP_FIRMWARE_MANIFEST` names a file to read
/// instead. With no vehicle it prints what the page writes under each vehicle picture; with one,
/// it runs `LookForPort` up to the download and names the file it would take. Nothing is
/// downloaded but the catalogue, and nothing is written to any board.
fn firmware_list(options: &[String]) -> std::process::ExitCode {
    let request = match ListRequest::parse(options) {
        Ok(request) => request,
        Err(why) => {
            eprintln!("{why}\n{LIST_USAGE}");
            return std::process::ExitCode::from(2);
        }
    };
    let fetch = mp_firmware::manifest::fetcher();
    let mut held = None;
    for error in mp_firmware::manifest::load(&mut held, fetch.as_ref()) {
        eprintln!("{error}");
    }
    let Some(manifest) = held else {
        eprintln!("no firmware catalogue: every source failed");
        return std::process::ExitCode::FAILURE;
    };
    let devices = match &request.board {
        Some(BoardArgument::Name(name)) => vec![mp_firmware::DeviceInfo::new(name, "")],
        // A board id stands for the bootloader's answer, which needs no device to look up.
        Some(BoardArgument::Id(_)) => vec![mp_firmware::DeviceInfo::default()],
        None => mp_transport::list_ports()
            .iter()
            .filter_map(mp_firmware::DeviceInfo::from_port)
            .collect(),
    };
    print!("{}", list_report(&manifest, &request, &devices));
    std::process::ExitCode::SUCCESS
}

/// `mpr firmware list`'s options, for a message about a wrong one.
const LIST_USAGE: &str = "usage: mpr firmware list [--board <id|name>] [--vehicle <type>] \
[--release <type>]
  --board    a board id, as a bootloader reports it, or a USB product string
             (default: the USB devices on this machine)
  --vehicle  ANTENNA_TRACKER, Copter, HELICOPTER, FIXED_WING, GROUND_ROVER, SUBMARINE
             (default: none - print what the page offers for each)
  --release  OFFICIAL (stable, the default), BETA, DEV (latest)";

/// What `--board` named.
#[derive(Debug, Clone, PartialEq, Eq)]
enum BoardArgument {
    /// A board id, standing for the one `Instance_DeviceChanged` reads from a bootloader.
    Id(i64),
    /// A USB product string, as a device reports it.
    Name(String),
}

/// `mpr firmware list`'s options.
#[derive(Debug, Clone, PartialEq, Eq)]
struct ListRequest {
    board: Option<BoardArgument>,
    vehicle: Option<mp_firmware::manifest::MavType>,
    release: mp_firmware::manifest::ReleaseType,
}

impl ListRequest {
    fn parse(options: &[String]) -> Result<Self, String> {
        use mp_firmware::manifest::{MavType, ReleaseType};
        let mut request = Self {
            board: None,
            vehicle: None,
            // `REL_Type`'s initial value. C#: ConfigFirmwareManifest.cs:24
            release: ReleaseType::Official,
        };
        let mut options = options.iter();
        while let Some(option) = options.next() {
            let value = options
                .next()
                .ok_or_else(|| format!("{option} needs a value"))?;
            match option.as_str() {
                "--board" => {
                    request.board = Some(
                        value
                            .parse()
                            .map_or_else(|_| BoardArgument::Name(value.clone()), BoardArgument::Id),
                    );
                }
                "--vehicle" => {
                    request.vehicle = Some(
                        MavType::from_name(value)
                            .ok_or_else(|| format!("not a vehicle: {value}"))?,
                    );
                }
                "--release" => {
                    request.release = ReleaseType::from_name(value)
                        .ok_or_else(|| format!("not a release type: {value}"))?;
                }
                other => return Err(format!("unknown option: {other}")),
            }
        }
        Ok(request)
    }
}

/// The text `mpr firmware list` prints for a catalogue, a request and the devices.
fn list_report(
    manifest: &mp_firmware::manifest::Manifest,
    request: &ListRequest,
    devices: &[mp_firmware::DeviceInfo],
) -> String {
    use mp_firmware::manifest::{FirmwareInfo, MavType, NO_FIRMWARE, NO_PORT, Outcome, icon_name};
    use std::fmt::Write as _;

    fn chosen(out: &mut String, item: &FirmwareInfo) {
        let _ = writeln!(
            out,
            "chosen: {} {}",
            item.mav_firmware_version
                .map(|version| version.to_string())
                .unwrap_or_default(),
            item.url.as_deref().unwrap_or("")
        );
    }

    let mut out = String::new();
    let _ = writeln!(
        out,
        "{} records, release {}",
        manifest.firmware.len(),
        request.release
    );
    let Some(vehicle) = request.vehicle else {
        // What `Activate` writes under each picture.
        for mav_type in MavType::ALL {
            let label = manifest
                .newest(request.release, mav_type)
                .map_or_else(|| "(none)".to_owned(), icon_name);
            let _ = writeln!(out, "  {:<16} {label}", mav_type.name());
        }
        return out;
    };
    let detected = match request.board {
        Some(BoardArgument::Id(id)) => Some(id),
        _ => None,
    };
    let Some(lookup) = manifest.look_for_port(devices, detected, vehicle, request.release, false)
    else {
        let _ = writeln!(out, "{}", NO_PORT.replace("\r\n", "\n"));
        return out;
    };
    let ids: Vec<String> = lookup.board_ids.iter().map(ToString::to_string).collect();
    let _ = writeln!(
        out,
        "board: {} - board id {}",
        lookup.device.board.as_deref().unwrap_or("(none)"),
        ids.join(", ")
    );
    for item in &lookup.items {
        let _ = writeln!(
            out,
            "  {:<24} {}",
            item.platform.as_deref().unwrap_or(""),
            item.url.as_deref().unwrap_or("")
        );
    }
    match lookup.outcome() {
        Outcome::One(item) => chosen(&mut out, item),
        Outcome::Choose(selection) => match selection.selected() {
            Some(item) => {
                let _ = writeln!(
                    out,
                    "FirmwareSelection opens on the {} platform",
                    selection.platform.as_deref().unwrap_or("")
                );
                chosen(&mut out, item);
            }
            None => {
                let _ = writeln!(
                    out,
                    "More than one choice exists; Mission Planner asks which:"
                );
                for result in selection.results() {
                    let _ = writeln!(out, "  {result}");
                }
            }
        },
        Outcome::NoFirmware => {
            let _ = writeln!(out, "{NO_FIRMWARE}");
        }
    }
    out
}

/// Describes a firmware file without touching any hardware.
///
/// Read-only on purpose. Flashing is the one operation in this application that can leave a
/// vehicle unable to boot, and it has no simulator to rehearse against, so it does not belong
/// behind a command that a shell-history arrow-up can repeat at the wrong board.
fn firmware_info(path: &str) -> std::process::ExitCode {
    let firmware = match mp_firmware::Firmware::load(std::path::Path::new(path)) {
        Ok(firmware) => firmware,
        Err(err) => {
            eprintln!("{path}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    println!("{path}");
    println!("  board id            {}", firmware.board_id);
    if firmware.board_revision != 0 {
        println!("  board revision      {}", firmware.board_revision);
    }
    if !firmware.description.is_empty() {
        println!("  description         {}", firmware.description);
    }
    if !firmware.version.is_empty() {
        println!("  version             {}", firmware.version);
    }
    if !firmware.git_hash.is_empty() {
        println!("  built from          {}", firmware.git_hash);
    }
    println!(
        "  image               {} bytes declared, {} after padding",
        firmware.declared_image_size,
        firmware.image.len()
    );
    if !firmware.external_image.is_empty() {
        println!(
            "  external flash      {} bytes declared, {} after padding",
            firmware.declared_external_size,
            firmware.external_image.len()
        );
    }
    // The CRC depends on the flash size it is padded to, which is the board's, so a few of the
    // common sizes are shown rather than one that would have to be guessed.
    println!("  CRC by flash size:");
    for size in [1 << 20, 2 << 20] {
        println!(
            "    {:>4} KiB          {:#010x}",
            size / 1024,
            firmware.crc(size)
        );
    }
    std::process::ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::{BoardArgument, ListRequest, detect_report, list_report};
    use mp_firmware::DeviceInfo;
    use mp_firmware::manifest::{Manifest, MavType, ReleaseType};
    use mp_transport::PortInfo;

    fn catalogue() -> Manifest {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/firmware/manifest.json.gz");
        let bytes = std::fs::read(&path).expect("the fixture");
        Manifest::decode(&bytes, true).expect("parses")
    }

    fn request(options: &[&str]) -> ListRequest {
        let options: Vec<String> = options.iter().map(|&s| s.to_owned()).collect();
        ListRequest::parse(&options).expect("valid options")
    }

    #[test]
    fn list_options_parse_as_the_page_names_things() {
        let parsed = request(&[
            "--board",
            "140",
            "--vehicle",
            "copter",
            "--release",
            "latest",
        ]);
        assert_eq!(parsed.board, Some(BoardArgument::Id(140)));
        assert_eq!(parsed.vehicle, Some(MavType::Copter));
        assert_eq!(parsed.release, ReleaseType::Dev);
        assert_eq!(request(&[]).release, ReleaseType::Official);
        assert_eq!(
            request(&["--board", "CubeOrange-BL"]).board,
            Some(BoardArgument::Name("CubeOrange-BL".to_owned()))
        );
        for bad in [
            &["--vehicle"][..],
            &["--vehicle", "boat"],
            &["--release", "rc"],
            &["-x", "1"],
        ] {
            let bad: Vec<String> = bad.iter().map(|&s| s.to_owned()).collect();
            assert!(ListRequest::parse(&bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn list_without_a_vehicle_prints_the_page_labels() {
        let report = list_report(&catalogue(), &request(&["--release", "beta"]), &[]);
        assert!(
            report.starts_with("240 records, release BETA\n"),
            "{report}"
        );
        assert!(
            report.contains("  Copter           Copter V4.7.1 BETA\n"),
            "{report}"
        );
        assert!(
            report.contains("  FIXED_WING       Plane V4.7.1 BETA\n"),
            "{report}"
        );
    }

    #[test]
    fn list_names_the_file_mission_planner_would_take() {
        let devices = [DeviceInfo::new("CubeOrange-BL", "")];
        let report = list_report(
            &catalogue(),
            &request(&["--board", "CubeOrange-BL", "--vehicle", "Copter"]),
            &devices,
        );
        assert!(
            report.contains("board: CubeOrange-BL - board id 140\n"),
            "{report}"
        );
        assert!(
            report.ends_with(
                "FirmwareSelection opens on the CubeOrange platform\n\
                 chosen: 4.7.1 https://firmware.ardupilot.org/Copter/stable/CubeOrange/arducopter.apj\n"
            ),
            "{report}"
        );

        // A bootloader's board id alone leaves the platform to the operator.
        let report = list_report(
            &catalogue(),
            &request(&["--board", "140", "--vehicle", "Copter"]),
            &[DeviceInfo::default()],
        );
        assert!(report.contains("More than one choice exists"), "{report}");
        assert!(!report.contains("chosen:"), "{report}");

        let report = list_report(
            &catalogue(),
            &request(&["--vehicle", "Copter"]),
            &[DeviceInfo::new("FT232R USB UART", "")],
        );
        assert!(
            report.contains("Failed to detect port to upload to"),
            "{report}"
        );
    }

    fn usb(name: &str, vid: u16, pid: u16, product: &str) -> PortInfo {
        PortInfo {
            name: name.to_owned(),
            vid: Some(vid),
            pid: Some(pid),
            serial_number: None,
            manufacturer: None,
            product: Some(product.to_owned()),
        }
    }

    #[test]
    fn detect_names_a_chibios_bootloader_from_its_product_string() {
        let report = detect_report(
            "/dev/ttyACM0",
            &[usb("/dev/ttyACM0", 0x2dae, 0x1016, "CubeOrange-BL")],
        );
        assert_eq!(
            report,
            "  /dev/ttyACM0  USB\\VID_2DAE&PID_1016  CubeOrange-BL\n\
             detected: chbootloader (CubeOrange)\n"
        );
    }

    #[test]
    fn detect_says_what_it_does_not_do() {
        let probe = detect_report(
            "/dev/ttyACM0",
            &[usb("/dev/ttyACM0", 0x26ac, 0x0011, "PX4 FMU v2.x")],
        );
        assert!(
            probe.ends_with("to tell px4v3 from px4v2 (not done here)\n"),
            "{probe}"
        );

        let unknown = detect_report(
            "/dev/ttyUSB0",
            &[usb("/dev/ttyUSB0", 0x0403, 0x6001, "FT232R")],
        );
        assert!(
            unknown.contains("probe /dev/ttyUSB0 for an STK500 bootloader (not done here)"),
            "{unknown}"
        );
        assert!(detect_report("COM3", &[]).starts_with("not recognised"));
    }

    #[test]
    fn detect_reports_a_board_named_by_its_usb_id_alone() {
        let report = detect_report("COM5", &[usb("COM5", 0x27ac, 0x1151, "VRBRAIN")]);
        assert!(report.ends_with("detected: vrbrainv51\n"), "{report}");
    }

    #[test]
    fn detect_lists_only_usb_devices_and_is_not_stopped_by_the_rest() {
        let report = detect_report(
            "COM4",
            &[
                PortInfo::bare("COM1".to_owned()),
                usb("COM4", 0x1209, 0x5740, "fmuv3"),
            ],
        );
        assert_eq!(
            report,
            "  COM4  USB\\VID_1209&PID_5740  fmuv3\ndetected: px4v3\n"
        );
    }
}

/// `mpr`'s sets and commands through the real link and its retries, against a scripted vehicle
/// on an in-memory transport: the arrangement of `mp-link`'s `tests/retries.rs`, where a send the
/// script does not answer is a send lost on the way.
#[cfg(test)]
mod retries {
    #![allow(clippy::unwrap_used, clippy::indexing_slicing)]

    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::{Duration, Instant};

    use mp_link::requests::MAV_RESULT_ACCEPTED;
    use mp_link::{Link, LinkConfig, ProtocolTimeouts, commands};
    use mp_mavlink::{FrameDecoder, encode_v2};
    use mp_mavlink_dialects::all::{CommandAck, DIALECT, Heartbeat, MavMessage, ParamValue};
    use mp_transport::Transport;
    use mp_transport::testing::{Loopback, LoopbackEnd};
    use mp_vehicle::VehicleId;

    use super::{RequestOutcome, Written, run_command, write_param};

    const VEHICLE: VehicleId = VehicleId::new(1, 1);
    const GCS: VehicleId = VehicleId::new(255, 190);
    /// `MAV_PARAM_TYPE_INT32`.
    const INT32: u8 = 6;

    fn frame(seq: u8, message: &MavMessage) -> Vec<u8> {
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut frame = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = encode_v2(
            &mut frame,
            seq,
            VEHICLE.sysid,
            VEHICLE.compid,
            message.id(),
            &payload[..len],
            message.crc_extra(),
            0,
        )
        .unwrap();
        frame[..n].to_vec()
    }

    /// The vehicle, on a thread of its own because `mpr` blocks while it waits: it announces
    /// itself as ArduPilot, then hands everything the link sends to `script` and sends back
    /// whatever that returns, until stopped. Returns everything it heard.
    fn vehicle(
        mut end: LoopbackEnd,
        stop: Arc<AtomicBool>,
        mut script: impl FnMut(&MavMessage) -> Option<MavMessage> + Send + 'static,
    ) -> std::thread::JoinHandle<Vec<MavMessage>> {
        std::thread::spawn(move || {
            let mut seq = 0u8;
            let mut send = |end: &mut LoopbackEnd, message: &MavMessage| {
                end.write_all(&frame(seq, message)).unwrap();
                seq = seq.wrapping_add(1);
            };
            send(
                &mut end,
                &MavMessage::Heartbeat(Heartbeat {
                    custom_mode: 0,
                    r#type: 2,
                    autopilot: 3,
                    base_mode: 81,
                    system_status: 3,
                    mavlink_version: 3,
                }),
            );
            let mut decoder = FrameDecoder::new();
            let mut heard = Vec::new();
            let mut buf = [0u8; 4096];
            while !stop.load(Ordering::Acquire) {
                let n = end.read(&mut buf).unwrap_or(0);
                if n == 0 {
                    std::thread::sleep(Duration::from_millis(1));
                    continue;
                }
                let mut fresh = Vec::new();
                decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                    if let Some(message) = MavMessage::decode(frame.msgid, frame.payload) {
                        fresh.push(message);
                    }
                });
                for message in fresh {
                    heard.push(message);
                    if let Some(answer) = script(&message) {
                        send(&mut end, &answer);
                    }
                }
            }
            heard
        })
    }

    /// A link with Mission Planner's retry counts and its waits divided by twenty, once the
    /// vehicle on the other end has been heard.
    fn link(end: LoopbackEnd) -> Link {
        let config = LinkConfig {
            send_heartbeat: false,
            stream_rate_hz: 0,
            timeouts: ProtocolTimeouts::default().faster(20),
            ..LinkConfig::default()
        };
        let link = Link::from_transport(Box::new(end), config);
        let deadline = Instant::now() + Duration::from_secs(5);
        while link.vehicle(VEHICLE).is_none() {
            assert!(Instant::now() < deadline, "the vehicle was never heard");
            std::thread::sleep(Duration::from_millis(1));
        }
        link
    }

    fn param(name: &str, value: f32) -> MavMessage {
        MavMessage::ParamValue(ParamValue {
            param_value: value,
            param_count: 1,
            param_index: 0,
            param_id: mp_params::encode_param_id(name),
            param_type: INT32,
        })
    }

    /// `mpr param set` on a vehicle whose list was never downloaded: `RTL_ALT` is read first,
    /// since `setParam` sends only a listed name; the first `PARAM_SET` is lost, the link sends it
    /// again, and the vehicle's echo of the second is what `mpr` reports it holds.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1640-1644, 1748-1770`
    #[test]
    fn a_parameter_is_read_then_written_and_a_lost_set_is_sent_again() {
        let (vehicle_side, gcs_side) = Loopback::pair();
        let stop = Arc::new(AtomicBool::new(false));
        let mut sets = 0;
        let script = vehicle(
            vehicle_side,
            Arc::clone(&stop),
            move |message| match message {
                MavMessage::ParamRequestRead(_) => Some(param("RTL_ALT", 1500.0)),
                MavMessage::ParamSet(set) => {
                    sets += 1;
                    (sets == 2).then(|| param("RTL_ALT", set.param_value))
                }
                _ => None,
            },
        );
        let link = link(gcs_side);

        let written = write_param(&link, VEHICLE, "RTL_ALT", 2000.0);
        stop.store(true, Ordering::Release);
        let heard = script.join().unwrap();

        assert_eq!(written, Written::Holds(2000.0));
        let names: Vec<&str> = heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::ParamRequestRead(_) => Some("read"),
                MavMessage::ParamSet(_) => Some("set"),
                _ => None,
            })
            .collect();
        assert_eq!(names, ["read", "set", "set"]);
    }

    /// `mpr fly`'s arm, whose first `COMMAND_LONG` is lost: sent again with its confirmation
    /// counted up, and accepted.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2783-2834`
    #[test]
    fn a_command_whose_first_send_is_lost_is_sent_again_and_accepted() {
        let (vehicle_side, gcs_side) = Loopback::pair();
        let stop = Arc::new(AtomicBool::new(false));
        let mut longs = 0;
        let script = vehicle(vehicle_side, Arc::clone(&stop), move |message| {
            let MavMessage::CommandLong(long) = message else {
                return None;
            };
            longs += 1;
            // The first is lost on the way.
            (longs == 2).then_some(MavMessage::CommandAck(CommandAck {
                command: long.command,
                result: MAV_RESULT_ACCEPTED,
                progress: 0,
                result_param2: 0,
                target_system: GCS.sysid,
                target_component: GCS.compid,
            }))
        });
        let link = link(gcs_side);

        let outcome = run_command(&link, &commands::arm(VEHICLE, true, false));
        stop.store(true, Ordering::Release);
        let heard = script.join().unwrap();

        assert_eq!(outcome, Some(RequestOutcome::Accepted { value: None }));
        let confirmations: Vec<u8> = heard
            .iter()
            .filter_map(|message| match message {
                MavMessage::CommandLong(long) => Some(long.confirmation),
                _ => None,
            })
            .collect();
        assert_eq!(confirmations, [0, 1]);
    }
}
