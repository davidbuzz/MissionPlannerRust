//! The crate graph, held to PLAN.md §5.1's layers over what `cargo metadata` says it is.
//!
//! §5.1 is the pivot insurance: everything below the gpui boundary has to survive a change of UI
//! framework, and that is only true while nothing down there can reach one. A rule written in a
//! document is kept by whoever remembers it; this is the rule kept by CI.
//!
//! Every workspace crate is placed in a layer by the tables below, and a crate the tables do not
//! place fails the build: a new crate cannot dodge the rules by not being mentioned. Then:
//!
//! 1. nothing below L6 reaches `gpui`, or any of its or wgpu's crates, by any path;
//! 2. nothing below L3 reaches `mp-link`;
//! 3. no L8 screen crate depends on another;
//! 4. only `mp-render` names gpui's or wgpu's internal crates;
//! 5. no crate depends on a crate in a higher layer, and `mp-link` in particular on nothing above
//!    L3;
//! 6. there are no cycles.
//!
//! Rules 5 and 6 look at the dependencies that ship - normal and build - and leave
//! dev-dependencies out: a test may use a crate the code it tests must not, and Cargo itself
//! allows a dev-dependency cycle (`mp-mavlink`'s allocation test decodes through
//! `mp-mavlink-dialects`, which depends on `mp-mavlink`). Rules 1 and 2 include a crate's own
//! dev-dependencies as well, because a framework-agnostic crate whose tests need the framework
//! does not survive the framework leaving.
//!
//! Each rule is a function returning its violations, and each has a test below that breaks the
//! real graph on purpose and checks the rule notices. A rule that passes because it cannot fail
//! is not a rule.
//!
//! The graph is the whole resolve, not only the workspace's manifests, so a third-party crate that
//! pulls in a renderer is caught too. That needs every platform's manifests, which is what
//! `cargo fetch` downloads; offline, run that first.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::process::Command;
use std::sync::OnceLock;

use serde_json::Value;

/// §5.1's layers, restricted to the crates that exist.
///
/// Written out layer by layer, in §5.1's order, so comparing the two is a read and not a search.
/// The names §5.1 gives that no crate has yet are in the comment above each row; when one is
/// created it goes in its row, and until it does `every_crate_has_a_layer` fails.
const LAYERS: &[(u8, &[&str])] = &[
    // L0. Not yet: mp-math, mp-time, mp-bus.
    (0, &["mp-units", "mp-settings"]),
    // L1. Not yet: mp-dronecan, mp-gnss, mp-adsb, mp-cot.
    (1, &["mp-mavlink", "mp-mavlink-dialects"]),
    // L2. Not yet: mp-transport-ble, mp-platform, mp-platform-linux, mp-platform-windows,
    // mp-platform-macos.
    (2, &["mp-transport"]),
    // L3. Not yet: mp-joystick (mp-input, below, is what it will be), mp-swarm, mp-antenna,
    // mp-hil.
    (
        3,
        &[
            "mp-link",
            "mp-vehicle",
            "mp-params",
            "mp-mission",
            "mp-ftp",
            "mp-firmware",
            "mp-calibration",
            // Script.cs's Python host: a service the flight screen's Scripts tab runs, not a
            // plugin layer above the application (moved down from L11 on 2026-09-25, when
            // mp-gui began to depend on it; PLAN.md section 5.1).
            "mp-script",
        ],
    ),
    // L4. Not yet, by these names: mp-log-dataflash, mp-log-tlog, mp-log-analysis, mp-logstore.
    // The first two are mp-log, below, unsplit.
    //
    // L5. Not yet: mp-geo, mp-survey, mp-nofly.
    (5, &["mp-kml", "mp-terrain", "mp-georef"]),
    // L6, above the gpui boundary. Not yet: mp-render, mp-map, mp-hud, mp-video, mp-terrain3d,
    // mp-icons.
    (6, &["mp-chart"]),
    // L7. Not yet: mp-ui, mp-theme, mp-l10n.
    //
    // L8 is `mp-screen-*`, placed by prefix in `layer`; there are none yet.
    //
    // L9. Not yet: mp-app (mp-gui, below, is it). L10. Not yet: mission-planner.
    //
    // L11. Not yet: mp-plugin-api, mp-plugin-host.
    // L12. Not yet: mp-codegen (xtask holds the generators).
    (12, &["xtask"]),
];

/// Crates §5.1 does not name, each placed where §5.1 puts the thing it holds, and why.
const UNNAMED: &[(&str, u8, &str)] = &[
    (
        "mp-input",
        3,
        "§5.1's mp-joystick: joysticks and gamepads mapped to RC channels",
    ),
    (
        "mp-video",
        3,
        "WebCamService.Capture's part: the camera devices, their formats and the frames the HUD draws under itself",
    ),
    (
        "wasm-plugin-host",
        11,
        "§5.1's mp-plugin-host as an experiment (PLAN.md §13.6 row 95): wasmtime loading, driving and sandboxing a plugin",
    ),
    (
        "fencedist",
        11,
        "the experiment's plugin: the C#'s FenceDist and menu examples built for wasm32, a member so it type-checks natively",
    ),
    (
        "mp-log",
        4,
        "§5.1's mp-log-tlog and mp-log-dataflash, unsplit, with LogBrowse's plot extraction",
    ),
    (
        "mp-tiles",
        5,
        "the half of §5.1's mp-map that draws nothing: tile fetch, decode and disk cache",
    ),
    (
        "mp-gui",
        9,
        "§5.1's mp-app, still holding its screens (L8) and its binary (L10)",
    ),
    (
        "mp-cli",
        10,
        "a second front end, a binary beside §5.1's mission-planner",
    ),
    (
        "mp-fuzz-checks",
        12,
        "the properties the fuzz targets check: tooling, like xtask",
    ),
];

/// Where the gpui boundary is: L6 and above may use a UI framework, nothing below may.
const GPUI_BOUNDARY: u8 = 6;

/// The layer of the link engine, and the floor below which nothing may reach it.
const LINK_LAYER: u8 = 3;

/// The screens: one crate per `GCSView`, never depending on each other.
const SCREEN_LAYER: u8 = 8;

/// The one crate allowed to name renderer internals (§5.1 rule 4). It does not exist yet.
const RENDERER: &str = "mp-render";

/// Renderer internals named by a crate other than `mp-render`, pinned so the list can only
/// shrink.
///
/// gpui's platform crates are selected by `mp-gui` itself because `gpui_platform`, which is the
/// crate an application is meant to use for that, does not resolve as a git dependency (see
/// `crates/mp-gui/Cargo.toml`). When `mp-render` exists they move there. An entry here whose
/// dependency has gone fails `the_renderer_exceptions_are_all_still_needed`, so the list cannot
/// outlive its reason.
const RENDERER_EXCEPTIONS: &[(&str, &str)] = &[
    ("mp-gui", "gpui_linux"),
    ("mp-gui", "gpui_windows"),
    ("mp-gui", "gpui_macos"),
];

/// The layer of a workspace crate, or `None` if nothing places it.
fn layer(name: &str) -> Option<u8> {
    if name.starts_with("mp-screen-") {
        return Some(SCREEN_LAYER);
    }
    LAYERS
        .iter()
        .find(|(_, crates)| crates.contains(&name))
        .map(|(layer, _)| *layer)
        .or_else(|| {
            UNNAMED
                .iter()
                .find(|(crate_name, _, _)| *crate_name == name)
                .map(|(_, layer, _)| *layer)
        })
}

/// A crate belonging to gpui or wgpu other than `gpui` itself: what an application should not
/// have to name.
fn is_renderer_internal(name: &str) -> bool {
    name.starts_with("gpui_")
        || name.starts_with("gpui-")
        || name == "wgpu"
        || name.starts_with("wgpu-")
        || name.starts_with("wgpu_")
}

/// A UI framework, or a piece of one.
fn is_ui_framework(name: &str) -> bool {
    name == "gpui" || is_renderer_internal(name)
}

/// How one package depends on another. One dependency can be more than one kind at once.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Kinds {
    normal: bool,
    build: bool,
    dev: bool,
}

impl Kinds {
    /// Whether the dependency is part of what gets built for a user, rather than only for a test.
    const fn ships(self) -> bool {
        self.normal || self.build
    }
}

/// The resolved dependency graph.
#[derive(Debug, Clone)]
struct Graph {
    /// Package id to package name.
    names: BTreeMap<String, String>,
    /// The workspace's own packages, by id.
    members: BTreeSet<String>,
    /// Each package's dependencies, by id, with how it depends on them.
    deps: BTreeMap<String, Vec<(String, Kinds)>>,
}

impl Graph {
    /// Reads the graph from `cargo metadata` output.
    fn parse(metadata: &Value) -> Self {
        let names = metadata["packages"]
            .as_array()
            .expect("packages")
            .iter()
            .map(|package| {
                (
                    package["id"].as_str().expect("id").to_owned(),
                    package["name"].as_str().expect("name").to_owned(),
                )
            })
            .collect();
        let members = metadata["workspace_members"]
            .as_array()
            .expect("workspace_members")
            .iter()
            .map(|id| id.as_str().expect("member id").to_owned())
            .collect();
        let deps = metadata["resolve"]["nodes"]
            .as_array()
            .expect("the resolve; `cargo metadata` must not be run with --no-deps")
            .iter()
            .map(|node| {
                let edges = node["deps"]
                    .as_array()
                    .expect("deps")
                    .iter()
                    .map(|dep| {
                        let mut kinds = Kinds::default();
                        for kind in dep["dep_kinds"].as_array().expect("dep_kinds") {
                            match kind["kind"].as_str() {
                                None => kinds.normal = true,
                                Some("build") => kinds.build = true,
                                Some("dev") => kinds.dev = true,
                                Some(other) => panic!("unknown dependency kind {other}"),
                            }
                        }
                        (dep["pkg"].as_str().expect("pkg").to_owned(), kinds)
                    })
                    .collect();
                (node["id"].as_str().expect("node id").to_owned(), edges)
            })
            .collect();
        Self {
            names,
            members,
            deps,
        }
    }

    /// The name of a package, or its id if the graph does not know it.
    fn name<'a>(&'a self, id: &'a str) -> &'a str {
        self.names.get(id).map_or(id, String::as_str)
    }

    /// The id of a workspace crate.
    fn member(&self, name: &str) -> &str {
        self.members
            .iter()
            .find(|id| self.name(id) == name)
            .unwrap_or_else(|| panic!("{name} is not a workspace crate"))
    }

    /// The workspace's crates, by name, with their ids.
    fn crates(&self) -> impl Iterator<Item = (&str, &str)> {
        self.members.iter().map(|id| (self.name(id), id.as_str()))
    }

    /// A package's direct dependencies.
    fn direct(&self, id: &str) -> impl Iterator<Item = (&str, Kinds)> {
        self.deps
            .get(id)
            .into_iter()
            .flatten()
            .map(|(to, kinds)| (to.as_str(), *kinds))
    }

    /// Every package reachable from `id` through dependencies that ship, and through `id`'s own
    /// dev-dependencies when `with_own_dev` is set. Excludes `id` itself unless a cycle returns
    /// to it.
    fn reachable(&self, id: &str, with_own_dev: bool) -> BTreeSet<&str> {
        let mut seen = BTreeSet::new();
        let mut queue: Vec<&str> = self
            .direct(id)
            .filter(|(_, kinds)| kinds.ships() || (with_own_dev && kinds.dev))
            .map(|(to, _)| to)
            .collect();
        while let Some(next) = queue.pop() {
            if seen.insert(next) {
                queue.extend(
                    self.direct(next)
                        .filter(|(_, kinds)| kinds.ships())
                        .map(|(to, _)| to),
                );
            }
        }
        seen
    }

    /// Adds a dependency, for the tests that break the graph on purpose. Creates the package if
    /// the graph does not have one of that name.
    fn add(&mut self, from: &str, to: &str, kinds: Kinds) {
        let from = self.member(from).to_owned();
        let to = match self.names.iter().find(|(_, name)| name.as_str() == to) {
            Some((id, _)) => id.clone(),
            None => {
                let id = format!("synthetic#{to}");
                self.names.insert(id.clone(), to.to_owned());
                id
            }
        };
        self.deps.entry(from).or_default().push((to, kinds));
    }
}

/// The workspace's graph, read once.
fn graph() -> &'static Graph {
    static GRAPH: OnceLock<Graph> = OnceLock::new();
    GRAPH.get_or_init(|| {
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR")).join("../Cargo.toml");
        let output = Command::new(option_env!("CARGO").unwrap_or("cargo"))
            .args(["metadata", "--format-version", "1", "--manifest-path"])
            .arg(&manifest)
            .output()
            .expect("running cargo metadata");
        assert!(
            output.status.success(),
            "cargo metadata failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        Graph::parse(&serde_json::from_slice(&output.stdout).expect("cargo metadata's JSON"))
    })
}

/// A dependency that ships.
const NORMAL: Kinds = Kinds {
    normal: true,
    build: false,
    dev: false,
};

/// A dependency for tests only.
const DEV: Kinds = Kinds {
    normal: false,
    build: false,
    dev: true,
};

/// Workspace crates no table places, and table entries that are not workspace crates.
fn unplaced(graph: &Graph) -> Vec<String> {
    let crates: BTreeSet<&str> = graph.crates().map(|(name, _)| name).collect();
    let mut problems: Vec<String> = crates
        .iter()
        .filter(|name| layer(name).is_none())
        .map(|name| format!("{name} has no layer: add it to LAYERS, or to UNNAMED with a reason"))
        .collect();
    let tabled = LAYERS
        .iter()
        .flat_map(|(_, names)| names.iter().copied())
        .chain(UNNAMED.iter().map(|(name, _, _)| *name));
    let mut once = BTreeSet::new();
    for name in tabled {
        if !crates.contains(name) {
            problems.push(format!(
                "{name} is in the table but not in the workspace: §5.1 names that do not exist \
                 yet belong in the comments"
            ));
        }
        if !once.insert(name) {
            problems.push(format!("{name} is placed twice"));
        }
    }
    problems
}

/// Rule 1: crates below the boundary that can reach a UI framework, and which parts of it.
fn ui_below_the_boundary(graph: &Graph) -> Vec<String> {
    let mut problems = Vec::new();
    for (name, id) in graph.crates() {
        if layer(name).is_none_or(|layer| layer >= GPUI_BOUNDARY) {
            continue;
        }
        for reached in graph.reachable(id, true) {
            let reached = graph.name(reached);
            if is_ui_framework(reached) {
                problems.push(format!(
                    "{name} (L{}) reaches {reached}",
                    layer(name).unwrap_or_default()
                ));
            }
        }
    }
    problems
}

/// Rule 2: crates below L3 that can reach the link.
fn link_below_l3(graph: &Graph) -> Vec<String> {
    graph
        .crates()
        .filter(|(name, _)| layer(name).is_some_and(|layer| layer < LINK_LAYER))
        .filter(|(_, id)| {
            graph
                .reachable(id, true)
                .iter()
                .any(|reached| graph.name(reached) == "mp-link")
        })
        .map(|(name, _)| format!("{name} reaches mp-link"))
        .collect()
}

/// Rule 3: screens that depend on screens.
fn screens_on_screens(graph: &Graph) -> Vec<String> {
    let mut problems = Vec::new();
    for (name, id) in graph.crates() {
        if layer(name) != Some(SCREEN_LAYER) {
            continue;
        }
        for (to, _) in graph.direct(id) {
            if graph.members.contains(to) && layer(graph.name(to)) == Some(SCREEN_LAYER) {
                problems.push(format!(
                    "{name} depends on {}; screens talk through mp-app",
                    graph.name(to)
                ));
            }
        }
    }
    problems
}

/// Rule 4: renderer internals named by anything but `mp-render`, less the pinned exceptions.
fn renderer_internals_named(graph: &Graph) -> Vec<String> {
    let mut problems = Vec::new();
    for (name, id) in graph.crates() {
        if name == RENDERER {
            continue;
        }
        for (to, _) in graph.direct(id) {
            let to = graph.name(to);
            if is_renderer_internal(to) && !RENDERER_EXCEPTIONS.contains(&(name, to)) {
                problems.push(format!("{name} names {to}, which only {RENDERER} may"));
            }
        }
    }
    problems
}

/// Rule 5: dependencies that ship and point up a layer.
fn upward(graph: &Graph) -> Vec<String> {
    let mut problems = Vec::new();
    for (name, id) in graph.crates() {
        let Some(from) = layer(name) else { continue };
        for (to, kinds) in graph.direct(id) {
            if !kinds.ships() || !graph.members.contains(to) {
                continue;
            }
            let to = graph.name(to);
            if let Some(target) = layer(to)
                && target > from
            {
                problems.push(format!("{name} (L{from}) depends on {to} (L{target})"));
            }
        }
    }
    problems
}

/// Rule 5, for the crate this layering was drawn around: what `mp-link` reaches above L3.
fn link_above_l3(graph: &Graph) -> Vec<String> {
    let link = graph.member("mp-link");
    graph
        .reachable(link, false)
        .into_iter()
        .filter(|id| graph.members.contains(*id))
        .map(|id| graph.name(id))
        .filter(|name| layer(name).is_none_or(|layer| layer > LINK_LAYER))
        .map(|name| match layer(name) {
            Some(layer) => format!("mp-link reaches {name} (L{layer})"),
            None => format!("mp-link reaches {name}, which has no layer"),
        })
        .collect()
}

/// Rule 6: cycles among the workspace's crates through dependencies that ship.
fn cycles(graph: &Graph) -> Vec<String> {
    /// Depth-first, remembering the path so a cycle can be printed rather than merely detected.
    fn visit<'g>(
        graph: &'g Graph,
        id: &'g str,
        path: &mut Vec<&'g str>,
        done: &mut BTreeSet<&'g str>,
        problems: &mut Vec<String>,
    ) {
        if let Some(start) = path.iter().position(|on_path| *on_path == id) {
            let cycle: Vec<&str> = path[start..]
                .iter()
                .chain(std::iter::once(&id))
                .map(|id| graph.name(id))
                .collect();
            problems.push(cycle.join(" -> "));
            return;
        }
        if done.contains(id) {
            return;
        }
        path.push(id);
        for (to, kinds) in graph.direct(id) {
            if kinds.ships() && graph.members.contains(to) {
                visit(graph, to, path, done, problems);
            }
        }
        path.pop();
        done.insert(id);
    }

    let mut problems = Vec::new();
    let mut done = BTreeSet::new();
    for id in &graph.members {
        visit(graph, id, &mut Vec::new(), &mut done, &mut problems);
    }
    problems
}

/// Fails with every violation listed, not just the first.
fn assert_none(rule: &str, problems: &[String]) {
    assert!(problems.is_empty(), "{rule}:\n  {}", problems.join("\n  "));
}

#[test]
fn every_crate_has_a_layer() {
    assert_none(
        "crates the layer tables do not account for",
        &unplaced(graph()),
    );
}

#[test]
fn nothing_below_the_gpui_boundary_reaches_a_ui_framework() {
    assert_none("§5.1 rule 1", &ui_below_the_boundary(graph()));
}

#[test]
fn nothing_below_l3_reaches_the_link() {
    assert_none("§5.1 rule 2", &link_below_l3(graph()));
}

#[test]
fn no_screen_depends_on_another_screen() {
    // Vacuous today: there are no mp-screen-* crates, and the screens are modules of mp-gui. The
    // rule is checked anyway so the first two screen crates are held to it from the start.
    assert_none("§5.1 rule 3", &screens_on_screens(graph()));
}

#[test]
fn only_the_renderer_names_renderer_internals() {
    assert_none("§5.1 rule 4", &renderer_internals_named(graph()));
}

#[test]
fn the_renderer_exceptions_are_all_still_needed() {
    for (from, to) in RENDERER_EXCEPTIONS {
        let id = graph().member(from);
        assert!(
            graph().direct(id).any(|(dep, _)| graph().name(dep) == *to),
            "{from} no longer names {to}: remove it from RENDERER_EXCEPTIONS"
        );
    }
}

#[test]
fn no_crate_depends_on_a_higher_layer() {
    assert_none("dependencies pointing up", &upward(graph()));
}

#[test]
fn the_link_reaches_nothing_above_l3() {
    assert_none("mp-link above L3", &link_above_l3(graph()));
}

#[test]
fn there_are_no_cycles_outside_dev_dependencies() {
    assert_none("cycles", &cycles(graph()));
}

// The rules against a graph broken on purpose. Each adds the one edge its rule exists to refuse
// to a copy of the real graph, and checks that the rule names it. They compare against what the
// rule finds in the real graph, so a real violation fails the tests above and not these as well.

/// What `rule` reports with `broken` that it does not report for the real graph.
fn introduced(rule: fn(&Graph) -> Vec<String>, broken: &Graph) -> Vec<String> {
    let before = rule(graph());
    rule(broken)
        .into_iter()
        .filter(|problem| !before.contains(problem))
        .collect()
}

/// The real graph with one more dependency.
fn with(from: &str, to: &str, kinds: Kinds) -> Graph {
    let mut broken = graph().clone();
    broken.add(from, to, kinds);
    broken
}

/// The real graph with workspace crates that do not exist.
fn with_crates(names: &[&str]) -> Graph {
    let mut broken = graph().clone();
    for name in names {
        let id = format!("synthetic#{name}");
        broken.names.insert(id.clone(), (*name).to_owned());
        broken.members.insert(id);
    }
    broken
}

#[test]
fn gpui_in_mp_params_is_refused() {
    let found = introduced(ui_below_the_boundary, &with("mp-params", "gpui", NORMAL));
    assert!(
        found.contains(&"mp-params (L3) reaches gpui".to_owned()),
        "{found:?}"
    );
}

#[test]
fn a_ui_framework_reached_through_another_crate_is_refused() {
    // mp-vehicle does not name gpui; it reaches it through mp-gui. Rule 5 sees the edge and rule 1
    // sees where it leads.
    let broken = with("mp-vehicle", "mp-gui", NORMAL);
    let found = introduced(ui_below_the_boundary, &broken);
    assert!(
        found.contains(&"mp-vehicle (L3) reaches gpui".to_owned()),
        "{found:?}"
    );
    let found = introduced(upward, &broken);
    assert!(
        found.contains(&"mp-vehicle (L3) depends on mp-gui (L9)".to_owned()),
        "{found:?}"
    );
}

#[test]
fn a_ui_framework_in_a_tests_only_dependency_is_refused_too() {
    // A test of framework-agnostic code must not need the framework either.
    let found = introduced(ui_below_the_boundary, &with("mp-mission", "wgpu", DEV));
    assert!(
        found.contains(&"mp-mission (L3) reaches wgpu".to_owned()),
        "{found:?}"
    );
}

#[test]
fn the_link_below_l3_is_refused() {
    let found = introduced(link_below_l3, &with("mp-transport", "mp-link", NORMAL));
    assert_eq!(found, vec!["mp-transport reaches mp-link"]);
}

#[test]
fn a_screen_on_a_screen_is_refused() {
    // No screen crates exist, so both are made up for the purpose.
    let mut broken = with_crates(&["mp-screen-flightdata", "mp-screen-flightplanner"]);
    broken.add("mp-screen-flightdata", "mp-screen-flightplanner", NORMAL);
    assert_eq!(
        introduced(screens_on_screens, &broken),
        vec![
            "mp-screen-flightdata depends on mp-screen-flightplanner; screens talk through mp-app"
        ]
    );
}

#[test]
fn a_renderer_internal_outside_the_renderer_is_refused() {
    let mut broken = with("mp-chart", "wgpu", NORMAL);
    broken.add("mp-gui", "gpui_wgpu", NORMAL);
    let found = introduced(renderer_internals_named, &broken);
    assert!(
        found.contains(&"mp-chart names wgpu, which only mp-render may".to_owned()),
        "{found:?}"
    );
    assert!(
        found.contains(&"mp-gui names gpui_wgpu, which only mp-render may".to_owned()),
        "the exceptions are per dependency, not a pass for the whole crate: {found:?}"
    );
}

#[test]
fn a_dependency_pointing_up_is_refused() {
    let found = introduced(upward, &with("mp-params", "mp-kml", NORMAL));
    assert_eq!(found, vec!["mp-params (L3) depends on mp-kml (L5)"]);
}

#[test]
fn a_dev_dependency_pointing_up_is_allowed() {
    // A test may use what the code it tests may not.
    assert!(introduced(upward, &with("mp-params", "mp-kml", DEV)).is_empty());
}

#[test]
fn the_link_depending_on_the_log_crate_is_refused() {
    // The edge this split removed: the link recorded through mp-log, which is L4.
    let broken = with("mp-link", "mp-log", NORMAL);
    assert_eq!(
        introduced(link_above_l3, &broken),
        vec!["mp-link reaches mp-log (L4)"]
    );
    assert_eq!(
        introduced(upward, &broken),
        vec!["mp-link (L3) depends on mp-log (L4)"]
    );
}

#[test]
fn a_cycle_is_refused_and_a_dev_cycle_is_not() {
    let found = introduced(cycles, &with("mp-units", "mp-link", NORMAL));
    assert!(
        found
            .iter()
            .any(|cycle| cycle.contains("mp-units -> mp-link")),
        "{found:?}"
    );
    assert!(introduced(cycles, &with("mp-units", "mp-link", DEV)).is_empty());
}

#[test]
fn a_crate_nobody_placed_is_refused() {
    let found = introduced(unplaced, &with_crates(&["mp-unplaced"]));
    assert_eq!(
        found,
        vec!["mp-unplaced has no layer: add it to LAYERS, or to UNNAMED with a reason"]
    );
}
