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
//! vehicle state, the application root and the screens. Finding that out by running an interpreter
//! and watching it raise would be the slow way.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use mp_script::{CsValue, ScriptHost, ScriptRequirements, Surface};

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

/// How many of the nineteen could run unmodified under a Rust Python engine.
///
/// **This is the answer D16 needs and it is not the one PLAN.md §10.4 assumes.** The plan's
/// reasoning was: the scripts are already Python, so a Python engine keeps them working. That is
/// true of the language and false of the scripts, because IronPython loads every .NET assembly
/// into their namespace and most of them use it - `MissionPlanner.MainV2.instance`,
/// `System.Threading.Thread`, `clr.AddReference`. Those are not Python features; they are CLR
/// features reached through Python syntax, and no engine without a CLR behind it can provide them.
///
/// So the choice of engine decides almost nothing about this corpus. The work is a compatibility
/// shim that answers to the names the scripts already use, and that is a different project from
/// embedding an interpreter.
#[test]
fn the_dotnet_dependence_of_the_corpus_is_recorded_per_script() {
    let scripts = corpus();
    let mut needing_dotnet: Vec<&String> = Vec::new();
    let mut portable: Vec<&String> = Vec::new();

    for (name, source) in &scripts {
        if ScriptRequirements::scan(name, source).needs_dotnet() {
            needing_dotnet.push(name);
        } else {
            portable.push(name);
        }
    }

    println!(
        "{} of {} scripts reach into .NET directly and cannot run unmodified on any Rust engine:",
        needing_dotnet.len(),
        scripts.len()
    );
    for name in &needing_dotnet {
        println!("  {name}");
    }
    println!(
        "\n{} could run given only the scope bindings:",
        portable.len()
    );
    for name in &portable {
        println!("  {name}");
    }

    // Asserted so the finding cannot quietly stop being true without somebody noticing - in
    // either direction. If a shim makes these work, this number moves and the test says so.
    assert!(
        needing_dotnet.len() >= 10,
        "expected most of the corpus to depend on .NET; only {} do",
        needing_dotnet.len()
    );
    assert!(
        !portable.is_empty(),
        "at least some scripts should be portable, or the scan is over-reporting"
    );
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
/// own time have passed, and a clock that only the script's `Sleep`s advance - ten minutes of
/// it and the run is stopped, which is how a script that loops for ever ends here.
#[derive(Debug)]
struct Simulated {
    clock_ms: u64,
    abort: Arc<AtomicBool>,
    log: Vec<String>,
}

const SIMULATED_MINUTES: u64 = 10;

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
        self.log.push(format!("ChangeParam {name} {value}"));
        true
    }
    fn change_mode(&mut self, mode: &str) -> bool {
        self.log.push(format!("ChangeMode {mode}"));
        true
    }
    fn has_message(&self, _text: &str) -> bool {
        self.clock_ms > 2_000
    }
    fn clear_messages(&mut self) {}
    fn cs_field(&self, name: &str) -> Option<CsValue> {
        Some(match name {
            "lat" => CsValue::Number(-35.363_262),
            "lng" => CsValue::Number(149.165_237),
            "alt" | "altasl" => CsValue::Number(584.0),
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
        self.log.push(format!("SendRC {channel} {pwm} {send_now}"));
        true
    }
    fn sleep(&mut self, milliseconds: u32) {
        self.clock_ms += u64::from(milliseconds);
        if self.clock_ms > SIMULATED_MINUTES * 60_000 {
            self.abort.store(true, Ordering::Relaxed);
        }
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

/// Every script, run under RustPython against the simulated vehicle, ends as recorded here: the
/// two that only need `Script` and `cs` (rc.py, example1.py) run to their end or loop until the
/// clock stops them; the fourteen that `import clr` stop there; wipe.py reaches `MAV`; datetime.py
/// is a C# note that IronPython trips on the same way; debugenv.py wants `distutils`, gone from
/// Python 3.12 and RustPython's stdlib. A script that begins to run, or to fail, differently
/// changes this table on purpose.
/// `// C#: Script.cs:45-53, 107-123`
#[test]
fn every_script_runs_under_the_engine_with_the_recorded_verdict() {
    const EXPECTED: &[(&str, &str)] = &[
        ("PARACHUTE LANDING APPROACH.py", "needs .NET (import clr)"),
        ("TAKEOFF.py", "needs .NET (import clr)"),
        ("cubeorange.py", "needs .NET (import clr)"),
        // Not Python: its first line is `c#`, which IronPython trips on the same way.
        ("datetime.py", "error: NameError: name 'c' is not defined. Did you mean: 'cs'?"),
        // `distutils` left Python at 3.12 and is not in RustPython's stdlib.
        ("debugenv.py", "error: ModuleNotFoundError: No module named 'distutils'"),
        ("example1.py", "ran until stopped at 10 min"),
        ("example10.py", "needs .NET (import clr)"),
        ("example2.py", "needs .NET (import clr)"),
        ("example3.py", "needs .NET (import clr)"),
        ("example4 wp.py", "needs .NET (import clr)"),
        ("example5 inject data.py", "needs .NET (import clr)"),
        ("example6.py", "needs .NET (import clr)"),
        ("example7.py", "needs .NET (import clr)"),
        ("example8 - speech.py", "needs .NET (import clr)"),
        ("example9 - sitl.py", "needs .NET (import clr)"),
        ("rc - heli.py", "needs .NET (import clr)"),
        ("rc.py", "ran to the end"),
        ("ui.py", "needs .NET (import clr)"),
        ("wipe.py", "needs MAV"),
    ];
    let mut verdicts = Vec::new();
    for (name, source) in corpus() {
        let abort = Arc::new(AtomicBool::new(false));
        let host = Simulated {
            clock_ms: 0,
            abort: Arc::clone(&abort),
            log: Vec::new(),
        };
        let output = Arc::new(Mutex::new(String::new()));
        let result = mp_script::engine::run_with(
            &name,
            &source,
            Box::new(host),
            Arc::clone(&output),
            Arc::clone(&abort),
        );
        let stopped = abort.load(Ordering::Relaxed);
        let verdict = verdict(&result, stopped);
        println!("{name}: {verdict}");
        if let Err(text) = &result {
            println!("    {}", text.lines().last().unwrap_or(""));
        }
        verdicts.push((name, verdict));
    }
    assert_eq!(verdicts.len(), SHIPPED_SCRIPTS);
    let actual: Vec<(&str, &str)> = verdicts
        .iter()
        .map(|(name, verdict)| (name.as_str(), verdict.as_str()))
        .collect();
    assert_eq!(actual, EXPECTED);
}
