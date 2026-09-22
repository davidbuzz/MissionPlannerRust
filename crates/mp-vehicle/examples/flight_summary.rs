//! Prints a summary of a recorded flight.
//!
//! `cargo run -p mp-vehicle --example flight_summary -- <log.tlog>`
//!
//! Useful for eyeballing a log and for diagnosing state-model questions without a debugger.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stdout)]
#![allow(clippy::indexing_slicing)] // a diagnostic example; a panic here is a visible failure

use std::collections::BTreeMap;

use mp_mavlink::FrameDecoder;
use mp_mavlink_dialects::all::{DIALECT, MavMessage};
use mp_transport::{ReplayTransport, Transport};
use mp_vehicle::VehicleRegistry;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../testdata/mavlink/autotest.tlog"
        )
        .to_owned()
    });

    let mut transport = ReplayTransport::open(&path)
        .expect("open log")
        .with_chunk_size(4096);
    let mut decoder = FrameDecoder::new();
    let mut registry = VehicleRegistry::new();
    let mut counts: BTreeMap<&'static str, u64> = BTreeMap::new();
    let mut positions_seen = 0u64;
    let mut zero_positions = 0u64;
    let mut last_nonzero: Option<(i32, i32)> = None;
    let mut max_roll = 0f32;
    let mut max_pitch = 0f32;
    let mut max_speed = 0f32;
    let mut max_alt = 0f32;
    let mut buf = [0u8; 4096];

    loop {
        let n = transport.read(&mut buf).expect("read");
        if n == 0 {
            break;
        }
        decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
            let Some(msg) = MavMessage::decode(frame.msgid, frame.payload) else {
                return;
            };
            *counts.entry(msg.name()).or_default() += 1;
            if let MavMessage::GlobalPositionInt(m) = msg {
                positions_seen += 1;
                if m.lat == 0 && m.lon == 0 {
                    zero_positions += 1;
                } else {
                    last_nonzero = Some((m.lat, m.lon));
                }
            }
            match msg {
                MavMessage::Attitude(m) => {
                    max_roll = max_roll.max(m.roll.abs());
                    max_pitch = max_pitch.max(m.pitch.abs());
                }
                MavMessage::VfrHud(m) => {
                    max_speed = max_speed.max(m.groundspeed);
                    max_alt = max_alt.max(m.alt);
                }
                _ => {}
            }
            registry.apply(frame.sysid, frame.compid, frame.seq, &msg);
        });
    }
    decoder.flush(&DIALECT, |_| {});

    println!("log: {path}");
    println!("vehicles: {:?}", registry.ids());
    println!(
        "GLOBAL_POSITION_INT: {positions_seen} total, {zero_positions} with lat/lon of exactly 0"
    );
    if let Some((lat, lon)) = last_nonzero {
        println!(
            "last non-zero position: {:.7}, {:.7}",
            f64::from(lat) / 1e7,
            f64::from(lon) / 1e7
        );
    }

    println!(
        "flight dynamics: max |roll| {max_roll:.4} rad, max |pitch| {max_pitch:.4} rad, \
         max ground speed {max_speed:.2} m/s, max alt {max_alt:.1} m"
    );

    for id in registry.ids() {
        let s = registry.working(id).expect("state");
        println!("\n--- vehicle {id} ---");
        println!("  messages applied : {}", s.messages_applied);
        println!("  type/autopilot   : {}/{}", s.vehicle_type, s.autopilot);
        println!("  armed            : {}", s.armed);
        println!("  position         : {:?}", s.position);
        println!(
            "  altitude msl/rel : {:.1} / {:.1} m",
            s.altitude_msl.0, s.altitude_relative.0
        );
        println!(
            "  attitude rpy     : {:.3} {:.3} {:.3} rad",
            s.attitude.roll.0, s.attitude.pitch.0, s.attitude.yaw.0
        );
        println!(
            "  gps              : fix {} sats {}",
            s.gps.fix_type, s.gps.satellites_visible
        );
        println!(
            "  battery          : {:.2} V {:.1} A {}%",
            s.battery.voltage, s.battery.current, s.battery.remaining_percent
        );
        println!(
            "  ground/air speed : {:.1} / {:.1} m/s",
            s.ground_speed.0, s.air_speed.0
        );
        println!(
            "  link             : {} rx, {} lost ({:.2}%)",
            s.link.received,
            s.link.lost,
            s.link.loss_percent()
        );
    }

    println!("\ntop messages:");
    let mut ranked: Vec<_> = counts.into_iter().collect();
    ranked.sort_by_key(|(_, count)| std::cmp::Reverse(*count));
    for (name, count) in ranked.iter().take(10) {
        println!("  {count:>6}  {name}");
    }
}
