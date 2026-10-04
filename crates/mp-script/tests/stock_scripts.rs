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

//! The nineteen scripts Mission Planner ships, and what they need to run.
//!
//! D16's definition of done was that all nineteen run unmodified under the embedded Python
//! engine; the owner's ruling of 2026-09-25 (PLAN.md §12 D20) is RustPython, Python 3, with the
//! scripts changed to run - `testdata/scripts/CHANGES.md` records each change. This file asserts
//! the corpus is intact, measures the host surface the corpus touches, and runs every script
//! under the engine against a simulated vehicle, recording the verdict per script.
//!
//! The measurement matters because the obvious plan - "port `Script.cs` and run them" - is wrong,
//! and the corpus says so plainly. Fifteen of the nineteen reach past `Script` into the link, the
//! vehicle state, the application root and the screens, and fifteen reach .NET by name through
//! IronPython - which is why the engine carries a shim for the CLR's names
//! (`crates/mp-script/src/clr/shim.py`) and this file keeps the ledger of what it answers.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use mp_script::{
    CsValue, Locationwp, PositionTarget, ScriptHost, ScriptRequirements, Surface, Timeout, WpItem,
};

/// Where the shipped scripts live.
fn corpus() -> Vec<(String, String)> {
    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/scripts");
    let mut scripts: Vec<(String, String)> = std::fs::read_dir(&directory)
        .unwrap_or_else(|err| panic!("reading {}: {err}", directory.display()))
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|extension| extension == "py"))
        .map(|path| {
            let name = path
                .file_name()
                .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
            let source = std::fs::read_to_string(&path)
                .unwrap_or_else(|err| panic!("reading {}: {err}", path.display()));
            (name, source)
        })
        .collect();
    scripts.sort();
    scripts
}

/// How many scripts ship with Mission Planner.
///
/// Asserted, because D16 asks for it: a script quietly disappearing from the corpus would turn a
/// compatibility gap into a passing test.
const SHIPPED_SCRIPTS: usize = 19;

#[test]
fn the_corpus_is_all_nineteen_scripts() {
    let scripts = corpus();
    assert_eq!(
        scripts.len(),
        SHIPPED_SCRIPTS,
        "the shipped script corpus has changed size: {:?}",
        scripts.iter().map(|(name, _)| name).collect::<Vec<_>>()
    );
    for (name, source) in &scripts {
        assert!(!source.is_empty(), "{name} is empty");
    }
}

/// The surface the corpus needs, printed so the gap is a number rather than an impression.
///
/// Not an assertion about what is implemented - it is a measurement, and it fails only if the
/// corpus stops being scannable at all.
#[test]
fn the_required_surface_is_measured_and_recorded() {
    let scripts = corpus();
    let mut using_only_script_object = 0usize;
    let mut needing_nothing = 0usize;
    let mut by_surface: std::collections::BTreeMap<&'static str, usize> =
        std::collections::BTreeMap::new();

    for (name, source) in &scripts {
        let scanned = ScriptRequirements::scan(name, source);
        if scanned.requirements.is_empty() {
            needing_nothing += 1;
        } else if scanned.only_needs(&[Surface::Script]) {
            using_only_script_object += 1;
        }
        for surface in scanned.surfaces() {
            *by_surface.entry(surface.binding()).or_default() += 1;
        }
        println!(
            "{name}: {}",
            if scanned.requirements.is_empty() {
                "pure python, no host calls".to_owned()
            } else {
                scanned
                    .requirements
                    .iter()
                    .map(|r| format!("{}.{}", r.surface.binding(), r.member))
                    .collect::<Vec<_>>()
                    .join(" ")
            }
        );
    }

    println!("\nscripts needing no host at all: {needing_nothing}");
    println!("scripts needing only Script.*:  {using_only_script_object}");
    for (binding, count) in &by_surface {
        println!("scripts touching {binding:<14} {count}");
    }

    // The measurement has to have measured something, or it is passing on an empty scan.
    assert!(
        by_surface.values().sum::<usize>() > 0,
        "nothing was found in any script, so the scan is broken rather than the corpus clean"
    );
}

/// `Script.*` alone does not get the corpus running, and the number is the point.
///
/// If a future change makes this assertion fail because *more* scripts now need only `Script`,
/// that is good news and the number moves. It is here so the scope cannot quietly be forgotten.
#[test]
fn most_scripts_need_more_than_the_script_object() {
    let scripts = corpus();
    let beyond_script: Vec<&String> = scripts
        .iter()
        .filter(|(name, source)| {
            let scanned = ScriptRequirements::scan(name, source);
            !scanned.requirements.is_empty() && !scanned.only_needs(&[Surface::Script])
        })
        .map(|(name, _)| name)
        .collect();

    assert!(
        beyond_script.len() >= 10,
        "expected most of the corpus to reach past Script.*; only {} do: {beyond_script:?}",
        beyond_script.len()
    );
    println!(
        "{} of {} scripts need a surface beyond Script.*:\n  {}",
        beyond_script.len(),
        scripts.len(),
        beyond_script
            .iter()
            .map(|n| n.as_str())
            .collect::<Vec<_>>()
            .join("\n  ")
    );
}


/// What the corpus reaches of .NET, name by name, and what of it the shim answers.
///
/// PLAN.md §10.4 reasoned that the scripts are already Python, so a Python engine keeps them
/// working. That is true of the language and false of the scripts: IronPython loads every .NET
/// assembly into their namespace (`Script.cs:40-44`) and most of them use it -
/// `clr.AddReference`, `MissionPlanner.Utilities.Locationwp`, `MAVLink.MAV_CMD`,
/// `System.Byte`. No engine without a CLR behind it has those, so the engine answers to the
/// names instead (`crates/mp-script/src/clr/shim.py`), and this is the ledger: every name the
/// corpus reaches is either one the shim answers ([`mp_script::engine::CLR_NAMES`], each checked
/// to resolve by the engine's own tests) or one recorded here as out of scope with the reason.
/// A new name in the corpus, or one moving between the lists, fails this test on purpose.
#[test]
fn the_dotnet_dependence_of_the_corpus_is_recorded_per_script() {
    /// Reached by a shipped script and not given to scripts in this version, with why.
    const OUT_OF_SCOPE: &[(&str, &str)] = &[
        // datetime.py is not Python (its first line is `c#`) and stops there under IronPython too.
        ("System.Console.WriteLine", "datetime.py never gets past its first line, `c#`"),
        ("System.DateTime.Now", "datetime.py never gets past its first line, `c#`"),
        // example7: the call's own `null` is a NameError before the member would run.
        (
            "MissionPlanner.MainV2.instance.FlightPlanner",
            "the Plan screen as an object: only BUT_read_Click is named, and it refuses",
        ),
        (
            "MissionPlanner.MainV2.instance.FlightPlanner.BUT_read_Click",
            "reading the vehicle's mission into the Plan screen from a script",
        ),
        // example9: starts twenty Windows SITL processes and a link to each.
        ("System.Diagnostics.Process", "launching a process (a Windows ArduCopter.exe)"),
        ("MissionPlanner.Comms.TcpSerial", "a second link made by a script"),
        ("MissionPlanner.MAVLinkInterface", "a second link made by a script"),
        ("MissionPlanner.MainV2.Comports.Add", "a second link made by a script"),
        ("System.Net.Sockets.TcpClient", "a .NET socket; Python's own socket module is there"),
        // ui.py: a WinForms form.
        ("System.Windows.Forms.Application", "WinForms"),
        ("System.Windows.Forms.Form", "WinForms"),
        ("System.Windows.Forms.Label", "WinForms"),
        ("System.Drawing.Size", "System.Drawing"),
        ("System.Drawing.Point", "System.Drawing"),
        ("System.Drawing.Font", "System.Drawing"),
        ("System.Activator.CreateInstance", "reflection over loaded assemblies"),
        ("clr.References", "the list of loaded .NET assemblies"),
        // example10's third subscription: not a member of the C#'s MAVLINK_MSG_ID either
        // (ExtLibs/Mavlink/Mavlink.cs:423-776), and never reached - its first subscription is
        // IronPython's TypeError.
        (
            "MAVLink.MAVLINK_MSG_ID.STATUSTEXT_LONG",
            "not in the C#'s MAVLINK_MSG_ID; example10 stops before it",
        ),
    ];
    let scripts = corpus();
    let mut reaching: Vec<&String> = Vec::new();
    let mut answered: Vec<&String> = Vec::new();
    let mut unknown: Vec<String> = Vec::new();
    for (name, source) in &scripts {
        let scanned = ScriptRequirements::scan(name, source);
        if !scanned.needs_dotnet() {
            continue;
        }
        reaching.push(name);
        let mut outside = Vec::new();
        for dotnet in scanned.dotnet_names() {
            if mp_script::engine::CLR_NAMES.contains(&dotnet.as_str()) {
                continue;
            }
            match OUT_OF_SCOPE.iter().find(|(out, _)| *out == dotnet.as_str()) {
                Some((out, why)) => outside.push(format!("{out} ({why})")),
                None => unknown.push(format!("{name}: {dotnet}")),
            }
        }
        if outside.is_empty() {
            answered.push(name);
            println!("{name}: every .NET name answered");
        } else {
            println!("{name}: out of scope: {}", outside.join("; "));
        }
    }
    assert!(
        unknown.is_empty(),
        "names neither answered nor recorded as out of scope:\n  {}",
        unknown.join("\n  ")
    );
    println!(
        "\n{} of {} scripts reach .NET; the shim answers every name {} of them reach",
        reaching.len(),
        scripts.len(),
        answered.len()
    );
    // 15 reach .NET: the fourteen that `import clr` and datetime.py's `System.` lines. 10 of them
    // reach nothing the shim leaves out; datetime, example7, example9 and ui do, and example10
    // names a message id the C# does not have either.
    assert_eq!(reaching.len(), 15, "{reaching:?}");
    assert_eq!(answered.len(), 10, "{answered:?}");
    for (out, _) in OUT_OF_SCOPE {
        assert!(
            !mp_script::engine::CLR_NAMES.contains(out),
            "{out} is both answered and out of scope"
        );
    }
}

/// The link is the biggest single dependency, which decides what to build next.
#[test]
fn the_link_is_the_surface_most_scripts_need() {
    let scripts = corpus();
    let touching_link = scripts
        .iter()
        .filter(|(name, source)| {
            ScriptRequirements::scan(name, source)
                .surfaces()
                .contains(&Surface::Link)
        })
        .count();
    assert!(
        touching_link >= 5,
        "only {touching_link} scripts touch MAV, which contradicts the scan"
    );
    println!("{touching_link} of {} scripts call into MAV", scripts.len());
}


/// A simulated vehicle for the corpus: telemetry that reads as a copter on the ground at the
/// SITL home, every parameter held, every `WaitFor` satisfied once two seconds of the script's
/// own time have passed, a link that accepts everything asked of it and keeps the mission items
/// it is sent, and a clock that only the script's waits advance - ten minutes of it and the run
/// is stopped, which is how a script that loops for ever ends here. What the script asked of the
/// host is written to `log`, one line a call.
#[derive(Debug)]
struct Simulated {
    clock_ms: u64,
    abort: Arc<AtomicBool>,
    log: Arc<Mutex<Vec<String>>>,
    mission: std::collections::BTreeMap<u16, mp_script::WpItem>,
}

const SIMULATED_MINUTES: u64 = 10;

/// The SITL home, as `getWP(0)` returns it.
const HOME: (f64, f64, f32) = (-35.363_262, 149.165_237, 584.0);

impl Simulated {
    fn new(abort: &Arc<AtomicBool>, log: &Arc<Mutex<Vec<String>>>) -> Self {
        Self {
            clock_ms: 0,
            abort: Arc::clone(abort),
            log: Arc::clone(log),
            mission: std::collections::BTreeMap::new(),
        }
    }

    fn note(&self, line: String) {
        self.log.lock().unwrap().push(line);
    }
}

impl ScriptHost for Simulated {
    fn get_parameter(&self, name: &str) -> Option<f32> {
        Some(match name {
            n if n.ends_with("_MIN") => 1100.0,
            n if n.ends_with("_MAX") => 1900.0,
            n if n.ends_with("_TRIM") => 1500.0,
            _ => 1.0,
        })
    }
    fn change_param(&mut self, name: &str, value: f32) -> bool {
        self.note(format!("ChangeParam {name} {value}"));
        true
    }
    fn change_mode(&mut self, mode: &str) -> bool {
        self.note(format!("ChangeMode {mode}"));
        true
    }
    fn has_message(&self, _text: &str) -> bool {
        self.clock_ms > 2_000
    }
    fn clear_messages(&mut self) {}
    fn cs_field(&self, name: &str) -> Option<CsValue> {
        Some(match name {
            "lat" => CsValue::Number(HOME.0),
            "lng" => CsValue::Number(HOME.1),
            "alt" | "altasl" => CsValue::Number(f64::from(HOME.2)),
            "roll" | "pitch" | "yaw" | "groundspeed" | "airspeed" | "verticalspeed" => {
                CsValue::Number(0.0)
            }
            "satcount" => CsValue::Number(10.0),
            "gpshdop" => CsValue::Number(1.0),
            "battery_voltage" => CsValue::Number(12.6),
            "ber_error" | "timeInAir" => CsValue::Number(0.0),
            "mode" => CsValue::Text("Stabilize".to_owned()),
            "firmware" => CsValue::Text("ArduCopter2".to_owned()),
            "armed" | "landed" | "connected" => CsValue::Flag(false),
            _ => return None,
        })
    }
    fn send_rc(&mut self, channel: u8, pwm: u16, send_now: bool) -> bool {
        self.note(format!("SendRC {channel} {pwm} {send_now}"));
        true
    }
    fn sleep(&mut self, milliseconds: u32) {
        self.clock_ms += u64::from(milliseconds);
        if self.clock_ms > SIMULATED_MINUTES * 60_000 {
            self.abort.store(true, Ordering::Relaxed);
        }
    }
    fn link_target(&self) -> (u8, u8) {
        (1, 1)
    }
    fn is_open(&self) -> bool {
        true
    }
    fn set_param(
        &mut self,
        _target: (u8, u8),
        name: &str,
        value: f64,
        force: bool,
    ) -> Result<bool, Timeout> {
        self.note(format!("setParam {name} {value} {force}"));
        Ok(true)
    }
    fn command(
        &mut self,
        _target: (u8, u8),
        command: u16,
        params: [f32; 7],
        _require_ack: bool,
    ) -> Result<bool, Timeout> {
        self.note(format!("doCommand {command} {params:?}"));
        Ok(true)
    }
    fn set_wp_total(&mut self, _target: (u8, u8), total: u16, _kind: u8) -> Result<(), Timeout> {
        self.note(format!("setWPTotal {total}"));
        Ok(())
    }
    fn set_wp(&mut self, _target: (u8, u8), item: &WpItem) -> Result<u8, Timeout> {
        self.note(format!(
            "setWP {} cmd {} frame {} p1 {} {} {} {}",
            item.seq, item.command, item.frame, item.params[0], item.x, item.y, item.z
        ));
        self.mission.insert(item.seq, *item);
        Ok(0)
    }
    fn set_wp_ack(&mut self, _target: (u8, u8), _kind: u8) {
        self.note("setWPACK".to_owned());
    }
    fn set_wp_current(&mut self, _target: (u8, u8), seq: u16) -> Result<bool, Timeout> {
        self.note(format!("setWPCurrent {seq}"));
        Ok(true)
    }
    fn get_wp(&mut self, _target: (u8, u8), index: u16, _kind: u8) -> Result<Locationwp, Timeout> {
        self.note(format!("getWP {index}"));
        Ok(match (index, self.mission.get(&index)) {
            (_, Some(item)) => Locationwp {
                id: item.command,
                p1: item.params[0],
                lat: f64::from(item.x),
                lng: f64::from(item.y),
                alt: item.z,
                frame: item.frame,
                ..Locationwp::default()
            },
            (0, None) => Locationwp {
                id: 16,
                lat: HOME.0,
                lng: HOME.1,
                alt: HOME.2,
                ..Locationwp::default()
            },
            _ => return Err(Timeout::on("getWP")),
        })
    }
    fn set_position_target(&mut self, _target: (u8, u8), position: &PositionTarget) -> bool {
        self.note(format!(
            "setPositionTarget frame {} {} {} {}",
            position.frame, position.lat, position.lng, position.alt
        ));
        true
    }
    fn write_raw(&mut self, bytes: &[u8]) -> bool {
        self.note(format!("BaseStream.Write {bytes:02x?}"));
        true
    }
    fn speak(&mut self, text: &str) {
        self.note(format!("SpeakAsync {text}"));
    }
}

/// How a script's run ended, in the words the table below records.
fn verdict(result: &Result<(), String>, stopped: bool) -> String {
    match result {
        Ok(()) if stopped => format!("ran until stopped at {SIMULATED_MINUTES} min"),
        Ok(()) => "ran to the end".to_owned(),
        Err(text) if text.contains("No module named 'clr'") => "needs .NET (import clr)".to_owned(),
        Err(text) => match text.lines().rev().find(|line| !line.trim().is_empty()) {
            Some(line) if line.contains("is not available to scripts") => {
                let object = line.split_whitespace().nth(1).unwrap_or("?");
                format!("needs {object}")
            }
            Some(line) => format!("error: {}", line.trim()),
            None => "error".to_owned(),
        },
    }
}

/// `example6.py` follows a leader whose positions come as UDP datagrams on port 4000, and waits
/// for each in a blocking `recv` that no abort reaches. So it runs on its own thread: once it
/// says "Guided Mode" one datagram is sent, and once it has said "Waypoint Sent" for it the run
/// is waiting for the next - the verdict. Then a datagram that is not a position ends it, on
/// its own `parameters[1]`, so no thread is left in `recv`.
///
/// The port is one free on this machine at the time, put in the script's `RPORT = 4000`: a
/// fixed port is taken whenever two runs of this test overlap - two checkouts built at once -
/// and the script's `bind` failing is its `sys.exit()`, which would read as a clean end.
fn run_example6(source: &str, log: &Arc<Mutex<Vec<String>>>) -> String {
    use web_time::{Duration, Instant};
    let port = std::net::UdpSocket::bind("127.0.0.1:0")
        .and_then(|socket| socket.local_addr())
        .expect("a free port")
        .port();
    assert!(source.contains("RPORT = 4000"), "example6's port line moved");
    let source = source.replacen("RPORT = 4000", &format!("RPORT = {port}"), 1);
    let abort = Arc::new(AtomicBool::new(false));
    let mut run = mp_script::ScriptRun::start(
        "example6.py",
        source,
        Box::new(Simulated::new(&abort, log)),
    );
    let wait_for = |text: &str| {
        let deadline = Instant::now() + Duration::from_secs(60);
        while !run.output().contains(text) {
            assert!(
                run.is_running(),
                "example6 ended before printing {text:?}: {:?} {:?}",
                run.result(),
                run.output()
            );
            assert!(Instant::now() < deadline, "example6 never printed {text:?}");
            wasm_thread::sleep(Duration::from_millis(10));
        }
    };
    wait_for("Guided Mode");
    let leader = std::net::UdpSocket::bind("127.0.0.1:0").expect("a socket to send from");
    let follower = ("127.0.0.1", port);
    leader
        .send_to(b"-35.36 149.16 90 20", follower)
        .expect("the datagram");
    wait_for("Waypoint Sent");
    assert!(run.is_running());
    // Its next `recv` gets a word, not four numbers: `parameters[1]`'s IndexError ends it.
    leader.send_to(b"stop", follower).expect("the last datagram");
    run.join();
    let ended = run.result();
    assert!(
        ended
            .as_ref()
            .is_some_and(|result| result.as_ref().is_err_and(|text| text.contains("IndexError"))),
        "{ended:?}"
    );
    "waits in rsock.recv for the next datagram".to_owned()
}

/// Every script, run under RustPython against the simulated vehicle, ends as recorded here, and
/// asks the host for what is recorded here. Before the shim (the ruling of 2026-09-25) fourteen
/// of them stopped at `import clr` and wipe.py at `MAV`; now each reaches its end, its own loop,
/// a member of the C# it calls wrongly - as it does under Mission Planner - or a named member
/// out of scope:
///
/// * cubeorange, example4, example5, rc - heli, rc.py and wipe.py run to their end;
/// * example1, example3 and example8 loop until the clock stops them, example6 waits for its
///   next datagram;
/// * TAKEOFF.py and PARACHUTE LANDING APPROACH.py upload their missions and then call
///   `setWPCurrent(1)`, which has three parameters and no overload of one: IronPython's
///   TypeError, before the closing `setWPACK`;
/// * example2 and example10 call `SubscribeToPacketType` with two arguments where it takes four
///   or five: IronPython's TypeError, before anything else they do;
/// * example7 passes `null`, which Python does not have;
/// * example9 needs `System.Diagnostics` (to start twenty Windows SITLs), ui.py
///   `System.Windows.Forms`;
/// * datetime.py is a C# note that IronPython trips on the same way; debugenv.py wants
///   `distutils`, gone from Python 3.12 and RustPython's stdlib.
///
/// `// C#: Script.cs:35-53, 107-123`
#[test]
fn every_script_runs_under_the_engine_with_the_recorded_verdict() {
    const EXPECTED: &[(&str, &str)] = &[
        (
            "PARACHUTE LANDING APPROACH.py",
            "error: TypeError: setWPCurrent() takes exactly 3 arguments (1 given)",
        ),
        (
            "TAKEOFF.py",
            "error: TypeError: setWPCurrent() takes exactly 3 arguments (1 given)",
        ),
        ("cubeorange.py", "ran to the end"),
        // Not Python: its first line is `c#`, which IronPython trips on the same way.
        ("datetime.py", "error: NameError: name 'c' is not defined. Did you mean: 'cs'?"),
        // `distutils` left Python at 3.12 and is not in RustPython's stdlib.
        ("debugenv.py", "error: ModuleNotFoundError: No module named 'distutils'"),
        ("example1.py", "ran until stopped at 10 min"),
        (
            "example10.py",
            "error: TypeError: SubscribeToPacketType() takes at least 4 arguments (2 given)",
        ),
        (
            "example2.py",
            "error: TypeError: SubscribeToPacketType() takes at least 4 arguments (2 given)",
        ),
        ("example3.py", "ran until stopped at 10 min"),
        ("example4 wp.py", "ran to the end"),
        ("example5 inject data.py", "ran to the end"),
        ("example6.py", "waits in rsock.recv for the next datagram"),
        ("example7.py", "error: NameError: name 'null' is not defined"),
        ("example8 - speech.py", "ran until stopped at 10 min"),
        ("example9 - sitl.py", "needs System.Diagnostics"),
        ("rc - heli.py", "ran to the end"),
        ("rc.py", "ran to the end"),
        ("ui.py", "needs System.Windows.Forms"),
        ("wipe.py", "ran to the end"),
    ];
    /// What each script asked of the host, in order, as the simulated vehicle logs it - the
    /// calls that matter, not every one.
    const ASKED: &[(&str, &[&str])] = &[
        (
            "PARACHUTE LANDING APPROACH.py",
            &[
                "getWP 0",
                "setWPTotal 5",
                "setWP 0 cmd 16 frame 3",
                "setWP 1 cmd 16 frame 3",
                "setWP 4 cmd 16 frame 3 p1 0 -35.361465 149.16551 40",
            ],
        ),
        (
            "TAKEOFF.py",
            &[
                "getWP 0",
                "setWPTotal 6",
                "setWP 0 cmd 16 frame 3",
                "setWP 1 cmd 22 frame 3 p1 10 0 0 100",
                "setWP 5 cmd 16 frame 3 p1 0 -35.363262 149.16524 200",
            ],
        ),
        (
            "cubeorange.py",
            &[
                "setParam INS_ACC_ID 0 true",
                "setParam INS_ACC3_ID 0 true",
                "setParam INS_ACC_ID 3081250 true",
                "setParam INS_ACC3_ID 3015690 true",
            ],
        ),
        ("example3.py", &["ChangeMode Guided", "ChangeMode GUIDED", "setPositionTarget frame 0 -35 117.98 60"]),
        (
            "example4 wp.py",
            &[
                "setWPTotal 5",
                "setWP 0 cmd 16 frame 3 p1 0 -34.9805 117.8518 0",
                "setWP 1 cmd 22 frame 3 p1 15 0 0 50",
                "setWP 2 cmd 16 frame 3 p1 0 -35 117.8 50",
                "setWP 3 cmd 16 frame 3 p1 0 -35 117.89 50",
                "setWP 4 cmd 16 frame 3 p1 0 -35 117.85 20",
                "setWPACK",
            ],
        ),
        ("example5 inject data.py", &["BaseStream.Write [13, 00, 00, 00, 08, 00]"]),
        ("example6.py", &["ChangeMode Guided", "ChangeMode GUIDED", "setPositionTarget frame 0 -35.3"]),
        ("example8 - speech.py", &["SpeakAsync test 0"]),
        (
            "rc - heli.py",
            &[
                "SendRC 8 1000 true",
                "doCommand 400 [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                "ChangeMode Guided",
                "doCommand 22 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0]",
            ],
        ),
        (
            "wipe.py",
            &[
                "ChangeParam SYSID_SW_MREV 0",
                "ChangeParam FORMAT_VERSION 0",
                "doCommand 246 [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            ],
        ),
    ];
    let mut verdicts = Vec::new();
    let mut logs = std::collections::BTreeMap::new();
    for (name, source) in corpus() {
        let log = Arc::new(Mutex::new(Vec::new()));
        let verdict = if name == "example6.py" {
            run_example6(&source, &log)
        } else {
            let abort = Arc::new(AtomicBool::new(false));
            let output = Arc::new(Mutex::new(String::new()));
            let result = mp_script::engine::run_with(
                &name,
                &source,
                Box::new(Simulated::new(&abort, &log)),
                Arc::clone(&output),
                Arc::clone(&abort),
            );
            let stopped = abort.load(Ordering::Relaxed);
            if let Err(text) = &result {
                println!("    {}", text.lines().last().unwrap_or(""));
            }
            verdict(&result, stopped)
        };
        println!("{name}: {verdict}");
        verdicts.push((name.clone(), verdict));
        logs.insert(name, log.lock().unwrap().clone());
    }
    assert_eq!(verdicts.len(), SHIPPED_SCRIPTS);
    let actual: Vec<(&str, &str)> = verdicts
        .iter()
        .map(|(name, verdict)| (name.as_str(), verdict.as_str()))
        .collect();
    assert_eq!(actual, EXPECTED);
    for (name, asked) in ASKED {
        let log = &logs[*name];
        // Each expected call, in order, as the start of a logged line.
        let mut from = 0;
        for call in *asked {
            let found = log
                .iter()
                .skip(from)
                .position(|line| line.starts_with(call))
                .unwrap_or_else(|| panic!("{name} never asked {call:?} after line {from}: {log:#?}"));
            from += found + 1;
        }
    }
}
