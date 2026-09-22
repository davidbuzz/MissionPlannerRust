//! `mpr` - a command line front end for the Mission Planner port.
//!
//! Exists so the stack can be exercised against a real vehicle long before there is a UI:
//! `mpr watch udp:14550` against SITL is the fastest way to find out whether the link, the
//! decoder and the state model actually work.

// A CLI prints; that is its job.
#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::time::{Duration, Instant};

use mp_link::{Link, LinkConfig, commands};
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
         mpr params <url> [NAME]     download the parameter set, or show one parameter\n  \
         mpr mission <url> [file]    download the mission, or upload one from a file\n  \
         mpr survey <url> <file>     generate a survey grid around the vehicle\n  \
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
            println!("  {} frames recorded", link.frames_received());
        }
    }
    println!("recorded {} frames to {path}", link.frames_received());
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

    println!("mode GUIDED");
    link.send(&commands::set_mode(id, commands::copter_mode::GUIDED));
    std::thread::sleep(Duration::from_millis(500));

    println!("arming");
    link.send(&commands::arm(id, true, false));
    if !await_state(&handle, "armed", Duration::from_secs(20), |s| s.armed) {
        // Retry once: a first arm attempt often lands while a pre-arm check is still failing.
        println!("  retrying arm");
        link.send(&commands::arm(id, true, false));
        if !await_state(&handle, "armed", Duration::from_secs(30), |s| s.armed) {
            return std::process::ExitCode::FAILURE;
        }
    }

    println!("takeoff to 40 m");
    link.send(&commands::takeoff(id, 40.0));
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

/// Downloads the vehicle's parameters and prints them.
///
/// Useful on its own, and the fastest way to check the download protocol against a real vehicle:
/// a parameter set with a hole in it is a protocol bug, and the count is printed so a hole is
/// visible rather than implied.
fn params(url: &str, filter: Option<&str>) -> std::process::ExitCode {
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
        return std::process::ExitCode::FAILURE;
    };

    match filter {
        Some(name) => match table.get(name) {
            Some(value) => {
                println!("{name} = {} ({:?})", value.as_f64(), value.param_type());
                // The number alone is rarely what someone needs; the documentation is the point.
                if let Some(meta) = mp_vehicle::param_meta::lookup(name) {
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
                let meta = mp_vehicle::param_meta::lookup(name);
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
