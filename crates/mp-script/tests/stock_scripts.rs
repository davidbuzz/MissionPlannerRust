//! The nineteen scripts Mission Planner ships, and what they need to run.
//!
//! D16's definition of done is that all nineteen run unmodified under the embedded Python engine.
//! That engine is not wired yet, so this file does the half that can be done first and that
//! decides how big the other half is: it asserts the corpus is intact, and it measures the host
//! surface the corpus touches.
//!
//! The measurement matters because the obvious plan - "port `Script.cs` and run them" - is wrong,
//! and the corpus says so plainly. Fifteen of the nineteen reach past `Script` into the link, the
//! vehicle state, the application root and the screens. Finding that out by running an interpreter
//! and watching it raise would be the slow way.

use mp_script::{ScriptRequirements, Surface};

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
