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
        "{id} | {armed:<8} | {position} | alt {alt:>7.1} m | \
         spd {spd:>5.1} m/s | hdg {hdg:>5.1} | sats {sats:>2} fix {fix} | \
         batt {volts:>5.2} V | rx {rx} loss {loss:.1}% | crc_err {crc}",
        armed = if s.armed { "ARMED" } else { "disarmed" },
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
