//! Log inspection: summarise a recorded flight without opening a GUI.

#![allow(clippy::print_stdout, clippy::print_stderr)]
// Internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;

use mp_log::TlogReader;
use mp_log::dataflash::{DataflashReader, Value};
use mp_mavlink_dialects::all::{DIALECT, MavMessage};

/// Which kind of log a file is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogKind {
    /// Telemetry log: timestamped MAVLink frames as received by a ground station.
    Telemetry,
    /// Dataflash log: the vehicle's own onboard recording.
    Dataflash,
}

/// Identifies a log by content rather than by extension.
///
/// Extensions lie: people rename `.bin` to `.log`, and `.txt` is used for both missions and
/// telemetry dumps. The two formats have distinct signatures, so looking is cheap and correct.
#[must_use]
pub fn detect(data: &[u8]) -> Option<LogKind> {
    // A dataflash log is a stream of 0xA3 0x95 headers, and starts with an FMT message defining
    // FMT itself. A tlog is 8-byte big-endian timestamps followed by MAVLink frames.
    let dataflash_headers = data
        .windows(2)
        .take(4096)
        .filter(|w| w == b"\xA3\x95")
        .count();
    let mavlink_starts = data
        .iter()
        .take(4096)
        .filter(|b| **b == mp_mavlink::STX_V2 || **b == mp_mavlink::STX_V1)
        .count();

    if dataflash_headers >= 4 && dataflash_headers > mavlink_starts / 8 {
        Some(LogKind::Dataflash)
    } else if mavlink_starts >= 4 {
        Some(LogKind::Telemetry)
    } else {
        None
    }
}

/// Prints a summary of a log file.
pub fn summarise(path: &str) -> std::process::ExitCode {
    let data = match std::fs::read(path) {
        Ok(data) => data,
        Err(err) => {
            eprintln!("could not read {path}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    println!("{path}  ({:.1} MiB)", data.len() as f64 / (1024.0 * 1024.0));
    match detect(&data) {
        Some(LogKind::Telemetry) => summarise_tlog(&data),
        Some(LogKind::Dataflash) => summarise_dataflash(&data),
        None => {
            eprintln!("not a recognisable telemetry or dataflash log");
            return std::process::ExitCode::FAILURE;
        }
    }
    std::process::ExitCode::SUCCESS
}

fn summarise_tlog(data: &[u8]) {
    println!("format: telemetry log (.tlog)");

    let mut counts: BTreeMap<&'static str, u64> = BTreeMap::new();
    let mut first_time = None;
    let mut last_time = 0u64;
    let mut statustexts = Vec::new();
    let mut systems: BTreeMap<u8, u64> = BTreeMap::new();
    let mut max_altitude = f64::NEG_INFINITY;
    let mut max_speed = 0.0_f64;

    let mut reader = TlogReader::new(data);
    let mut frames = 0u64;
    while let Some(record) = reader.next_record(&DIALECT) {
        frames += 1;
        // The reader returns None where the preceding bytes were not a stamp, which happens for
        // the first frame of a log that opens with console text.
        if let Some(micros) = record.timestamp_micros {
            first_time.get_or_insert(micros);
            last_time = micros;
        }

        let Ok((frame, _)) = mp_mavlink::parse(record.frame, &DIALECT) else {
            continue;
        };
        *systems.entry(frame.sysid).or_default() += 1;
        let Some(message) = MavMessage::decode(frame.msgid, frame.payload) else {
            continue;
        };
        *counts.entry(message.name()).or_default() += 1;

        match message {
            MavMessage::GlobalPositionInt(m) => {
                max_altitude = max_altitude.max(f64::from(m.relative_alt) / 1000.0);
            }
            MavMessage::VfrHud(m) => max_speed = max_speed.max(f64::from(m.groundspeed)),
            MavMessage::Statustext(m) if statustexts.len() < 400 => {
                let end = m.text.iter().position(|b| *b == 0).unwrap_or(m.text.len());
                let text = m
                    .text
                    .get(..end)
                    .map(|bytes| String::from_utf8_lossy(bytes).trim().to_owned())
                    .unwrap_or_default();
                if !text.is_empty() {
                    statustexts.push((m.severity, text));
                }
            }
            _ => {}
        }
    }

    println!("frames: {frames}");
    if let Some(first) = first_time {
        let seconds = (last_time.saturating_sub(first)) as f64 / 1e6;
        println!("duration: {:.0} s ({:.1} min)", seconds, seconds / 60.0);
    }
    print_systems(&systems);
    if max_altitude > f64::NEG_INFINITY {
        println!("max altitude above home: {max_altitude:.1} m");
    }
    if max_speed > 0.0 {
        println!("max ground speed: {max_speed:.1} m/s");
    }
    print_top(&counts);
    print_statustexts(&statustexts);
}

fn summarise_dataflash(data: &[u8]) {
    println!("format: dataflash log (.bin)");

    let mut reader = DataflashReader::new(data);
    let mut counts: BTreeMap<String, u64> = BTreeMap::new();
    let mut messages = Vec::new();
    let mut max_altitude = f64::NEG_INFINITY;
    let mut modes: Vec<String> = Vec::new();

    while let Some(message) = reader.next_message() {
        *counts.entry(message.name.clone()).or_default() += 1;
        match message.name.as_str() {
            "MSG" => {
                if let Some(Value::Text(text)) = message.field("Message")
                    && messages.len() < 200
                {
                    messages.push(text.clone());
                }
            }
            "MODE" => {
                if let Some(value) = message.field("Mode").and_then(Value::as_f64) {
                    let label = format!("{value:.0}");
                    if modes.last() != Some(&label) {
                        modes.push(label);
                    }
                }
            }
            "POS" | "GPS" => {
                if let Some(alt) = message.field("Alt").and_then(Value::as_f64) {
                    max_altitude = max_altitude.max(alt);
                }
            }
            _ => {}
        }
    }

    let stats = *reader.stats();
    println!("messages: {} across {} types", stats.messages, counts.len());
    println!("format definitions: {}", stats.formats);
    if stats.resync_bytes > 0 {
        // A healthy log needs none; any at all means the recording is damaged.
        println!(
            "damaged: {} bytes skipped, {} messages of undefined type",
            stats.resync_bytes, stats.unknown_types
        );
    }
    if max_altitude > f64::NEG_INFINITY {
        println!("max altitude: {max_altitude:.1} m");
    }
    if !modes.is_empty() {
        println!("mode changes: {}", modes.join(" -> "));
    }

    let counts: BTreeMap<&str, u64> = counts.iter().map(|(k, v)| (k.as_str(), *v)).collect();
    print_top(&counts);

    if !messages.is_empty() {
        println!("\nonboard messages:");
        for text in messages.iter().take(12) {
            println!("  {text}");
        }
        if messages.len() > 12 {
            println!("  ... and {} more", messages.len() - 12);
        }
    }
}

fn print_systems(systems: &BTreeMap<u8, u64>) {
    if systems.len() > 1 {
        let listed: Vec<String> = systems
            .iter()
            .map(|(id, count)| format!("{id} ({count} frames)"))
            .collect();
        println!("systems on the link: {}", listed.join(", "));
    }
}

fn print_top<K: std::fmt::Display + Ord>(counts: &BTreeMap<K, u64>) {
    let mut ranked: Vec<_> = counts.iter().collect();
    ranked.sort_by(|a, b| b.1.cmp(a.1));
    println!("\nmost frequent messages:");
    for (name, count) in ranked.iter().take(10) {
        println!("  {count:>7}  {name}");
    }
}

fn print_statustexts(texts: &[(u8, String)]) {
    if texts.is_empty() {
        return;
    }
    // Severity 0-3 is emergency through error; those are what a pilot reviewing a flight wants
    // first, and burying them under routine chatter is how they get missed.
    let mut serious: Vec<&(u8, String)> = texts.iter().filter(|(sev, _)| *sev <= 3).collect();
    serious.dedup_by(|a, b| a.1 == b.1);

    if serious.is_empty() {
        println!("\nno errors reported");
    } else {
        println!("\nerrors and warnings:");
        for (severity, text) in serious.iter().take(12) {
            println!("  severity {severity}: {text}");
        }
    }
}

/// Writes a flown path from a telemetry log as KML.
///
/// The usual way a flight is handed to somebody without a ground station. Mission Planner writes
/// one beside every log it converts; this does the same from the command line.
pub fn to_kml(path: &str, out: &str) -> std::process::ExitCode {
    let data = match std::fs::read(path) {
        Ok(data) => data,
        Err(err) => {
            eprintln!("could not read {path}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    let mut track: Vec<mp_kml::TrackPoint> = Vec::new();
    let mut mode = String::from("unknown");
    let mut first_time: Option<u64> = None;

    let mut reader = TlogReader::new(&data);
    while let Some(record) = reader.next_record(&DIALECT) {
        let seconds = record.timestamp_micros.map_or(0.0, |micros| {
            let start = *first_time.get_or_insert(micros);
            #[allow(clippy::cast_precision_loss)] // microseconds of a flight fit a f64 exactly
            {
                (micros.saturating_sub(start)) as f64 / 1_000_000.0
            }
        });
        let Ok((frame, _)) = mp_mavlink::parse(record.frame, &DIALECT) else {
            continue;
        };
        let Some(message) = MavMessage::decode(frame.msgid, frame.payload) else {
            continue;
        };
        match message {
            // The mode number means nothing without knowing the airframe, which only the
            // heartbeat carries - so both are tracked and the name is resolved as they arrive.
            MavMessage::Heartbeat(m) => {
                if let Some(family) = mp_vehicle::VehicleFamily::from_mav_type(m.r#type)
                    && let Some((_, name)) = family
                        .modes()
                        .iter()
                        .find(|(number, _)| *number == m.custom_mode)
                {
                    mode = (*name).to_owned();
                }
            }
            MavMessage::GlobalPositionInt(m) => {
                // (0, 0) is how an unset position is transmitted. Writing it produces a leg
                // through the Gulf of Guinea in whatever the recipient opens the file with.
                if m.lat == 0 && m.lon == 0 {
                    continue;
                }
                if let Ok(position) = mp_units::LatLon::from_mavlink_e7(m.lat, m.lon) {
                    track.push(mp_kml::TrackPoint {
                        position,
                        altitude_msl: f64::from(m.alt) / 1000.0,
                        seconds,
                        mode: mode.clone(),
                    });
                }
            }
            _ => {}
        }
    }

    if track.is_empty() {
        eprintln!("{path} holds no positions to export");
        return std::process::ExitCode::FAILURE;
    }

    let name = std::path::Path::new(path).file_stem().map_or_else(
        || path.to_owned(),
        |stem| stem.to_string_lossy().into_owned(),
    );
    let kml = mp_kml::flight_path(&name, &track);
    match std::fs::write(out, &kml) {
        Ok(()) => {
            let modes: std::collections::BTreeSet<&str> =
                track.iter().map(|point| point.mode.as_str()).collect();
            println!(
                "wrote {} positions to {out} ({:.0} s, modes: {})",
                track.len(),
                track.last().map_or(0.0, |point| point.seconds),
                modes.into_iter().collect::<Vec<_>>().join(", ")
            );
            std::process::ExitCode::SUCCESS
        }
        Err(err) => {
            eprintln!("could not write {out}: {err}");
            std::process::ExitCode::FAILURE
        }
    }
}

/// Lists what a dataflash log offers to plot.
///
/// A listing, not a plot. Mission Planner plots logs with ZedGraph in `LogBrowse.cs` - a
/// WinForms chart - and there is no terminal plotting anywhere in the C#. An earlier version of
/// this drew an ASCII chart here, which was not a port of anything: it was the shape easiest to
/// check from a terminal, which is a convenience for whoever is writing the code rather than a
/// thing Mission Planner does. The plot belongs in the GUI, against `mp_chart`, where D14's
/// definition of done puts it.
///
/// What is useful from a command line is the inventory. The field list comes from the log's own
/// `FMT` messages rather than a table, so it is right for whatever firmware wrote it - a
/// hard-coded list goes stale silently, which on a diagnostic tool means a field that exists and
/// cannot be found.
pub fn fields(path: &str) -> std::process::ExitCode {
    let data = match std::fs::read(path) {
        Ok(data) => data,
        Err(err) => {
            eprintln!("could not read {path}: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    let fields = mp_log::plot::plottable(&data);
    if fields.is_empty() {
        eprintln!("{path} has nothing plottable - is it a dataflash log?");
        return std::process::ExitCode::FAILURE;
    }

    println!("{} plottable fields in {path}:\n", fields.len());
    // Grouped by message *and instance*, because a log with three IMUs has three VIBE groups and
    // printing them under one heading makes them look like one series listed three times.
    let mut heading = String::new();
    for field in &fields {
        let group = match field.instance {
            Some(instance) => format!("{}[{instance}]", field.message),
            None => field.message.clone(),
        };
        if group != heading {
            heading = group;
            println!("  {heading}");
        }
        println!("    {:<18} {:>7} samples", field.field, field.samples);
    }
    std::process::ExitCode::SUCCESS
}
