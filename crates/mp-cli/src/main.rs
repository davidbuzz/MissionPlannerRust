//! `mpr` - a command line front end for the Mission Planner port.
//!
//! Exists so the stack can be exercised against a real vehicle long before there is a UI:
//! `mpr watch udp:14550` against SITL is the fastest way to find out whether the link, the
//! decoder and the state model actually work.

// A CLI prints; that is its job.
#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::time::{Duration, Instant};

use mp_link::{Link, LinkConfig};
use mp_vehicle::VehicleId;

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
