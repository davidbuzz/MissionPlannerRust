//! PLAN.md §12 D16: a C# file nothing calls is not ported. Its ledger row is `dropped`, and its
//! note says why in one of four forms:
//!
//! - `no other file names its types` / `named only by files that are themselves dead (...)`: no
//!   file outside the D16 set names a type the file declares (comments do not count, strings do,
//!   since `Type.GetType("...")` is a caller);
//! - `nothing outside <dir> names <namespace>`: a library whose namespace nobody imports or
//!   qualifies, so none of its types can be reached;
//! - `its project <p> is in no build (...)`: neither MissionPlanner.sln nor MissionPlannerLib.sln
//!   lists it or reaches it through a project reference, so nothing builds it;
//! - `compiled by no project`: every project's items (SDK globs, `Compile Include`/`Remove` in
//!   document order) leave it out.
//!
//! The D16 set is closed: a file it holds may be named by another file it holds (a form and its
//! designer half, a cycle inside a dead library), never by one outside it. Test trees
//! (`MissionPlannerTests/`, `Tests/`, `*.Tests/`) are not callers that ship and do not count.
//!
//! Every reason is re-derived from `references/missionplanner` when this machine has it, so a
//! reference update that brings a caller back, or compiles a removed file, fails here. Without the
//! tree only the rows' form is checked.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::{Path, PathBuf};

use xtask::ledger::csv;

/// What every D16 note starts with; the reference commit follows.
const HEAD: &str = "no callers in the C# tree as of ";

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn tree() -> Option<PathBuf> {
    let tree = repo().join("references/missionplanner");
    if tree.is_dir() {
        Some(tree)
    } else {
        println!(
            "references/missionplanner is absent (it is gitignored); D16's reasons are not \
             re-derived, only the rows' form is checked"
        );
        None
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Reason {
    /// No file outside the D16 set names what it declares.
    Types,
    /// Nothing outside `dir` names `token`.
    Namespace { dir: String, token: String },
    /// These projects compile it, and no build reaches them.
    Project(Vec<String>),
    /// No project compiles it.
    Uncompiled,
}

#[derive(Debug)]
struct Dead {
    path: String,
    reference: String,
    reason: Reason,
    declares: Vec<String>,
}

/// The ledger's D16 rows, and each other row's path and state.
fn ledger() -> (Vec<Dead>, BTreeMap<String, String>) {
    let text = std::fs::read_to_string(repo().join("ledger/ledger.csv")).unwrap();
    let records = csv::parse(&text).unwrap();
    let header = &records[0].fields;
    let col = |name: &str| header.iter().position(|h| h == name).unwrap();
    let (path, state, notes) = (col("path"), col("state"), col("notes"));
    let mut dead = Vec::new();
    let mut states = BTreeMap::new();
    for record in &records[1..] {
        let f = &record.fields;
        states.insert(f[path].clone(), f[state].clone());
        let Some(at) = f[notes].find(HEAD) else {
            continue;
        };
        let note = &f[notes][at + HEAD.len()..];
        let (reference, rest) = note
            .split_once(" (D16): ")
            .unwrap_or_else(|| panic!("{}: no `(D16): ` after the commit", f[path]));
        let (why, declared) = rest
            .split_once("; declares ")
            .unwrap_or_else(|| panic!("{}: the note does not say what it declares", f[path]));
        let reason = parse_reason(why).unwrap_or_else(|| panic!("{}: reason `{why}`", f[path]));
        let declares = if declared == "nothing" {
            Vec::new()
        } else {
            declared.split(", ").map(str::to_owned).collect()
        };
        assert_eq!(
            f[state], "dropped",
            "{}: a D16 note on a row that is not dropped",
            f[path]
        );
        dead.push(Dead {
            path: f[path].clone(),
            reference: reference.to_owned(),
            reason,
            declares,
        });
    }
    (dead, states)
}

fn parse_reason(why: &str) -> Option<Reason> {
    if why.starts_with("no other file names its types")
        || why.starts_with("named only by files that are themselves dead")
    {
        return Some(Reason::Types);
    }
    if let Some(rest) = why.strip_prefix("nothing outside ") {
        let (dir, token) = rest.split_once(" names ")?;
        return Some(Reason::Namespace {
            dir: dir.to_owned(),
            token: token.to_owned(),
        });
    }
    if let Some(rest) = why.strip_prefix("its project ") {
        let (projects, tail) = rest.split_once(" is in no build")?;
        if !tail.is_empty() && !tail.starts_with(" (") {
            return None;
        }
        return Some(Reason::Project(
            projects.split(" / ").map(str::to_owned).collect(),
        ));
    }
    why.starts_with("compiled by no project")
        .then_some(Reason::Uncompiled)
}

/// Rows only: every D16 row is dropped, names one reference commit, and parses. Holds without
/// the tree.
#[test]
fn every_d16_row_is_dropped_with_its_reason() {
    let (dead, _) = ledger();
    assert!(
        dead.len() > 400,
        "the D16 record shrank to {} rows",
        dead.len()
    );
    let commits: BTreeSet<&str> = dead.iter().map(|d| d.reference.as_str()).collect();
    assert_eq!(commits.len(), 1, "one reference commit, not {commits:?}");
    for d in &dead {
        assert!(d.path.ends_with(".cs"), "{}", d.path);
        if d.reason == Reason::Types {
            assert!(
                !d.declares.is_empty(),
                "{}: a types reason with nothing declared",
                d.path
            );
        }
    }
}

// ---------------------------------------------------------------- the C# tree

/// Test trees: they call, but they are not callers that ship.
fn is_test(path: &str) -> bool {
    path.split('/').any(|part| {
        part == "MissionPlannerTests"
            || part.eq_ignore_ascii_case("test")
            || part.eq_ignore_ascii_case("tests")
            || part.ends_with(".Test")
            || part.ends_with(".Tests")
    })
}

/// Text files that can name a type for the runtime: resources, XAML, IronPython scripts, config.
fn is_resource(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    [
        ".xaml",
        ".axml",
        ".resx",
        ".config",
        ".settings",
        ".py",
        ".xml",
    ]
    .iter()
    .any(|ext| lower.ends_with(ext))
}

struct Tree {
    root: PathBuf,
    cs: Vec<String>,
    resources: Vec<String>,
    projects: Vec<String>,
}

fn walk(root: &Path) -> Tree {
    let mut tree = Tree {
        root: root.to_path_buf(),
        cs: Vec::new(),
        resources: Vec::new(),
        projects: Vec::new(),
    };
    let mut stack = vec![String::new()];
    while let Some(rel) = stack.pop() {
        for entry in std::fs::read_dir(root.join(&rel)).unwrap() {
            let entry = entry.unwrap();
            let name = entry.file_name().to_string_lossy().into_owned();
            let child = if rel.is_empty() {
                name.clone()
            } else {
                format!("{rel}/{name}")
            };
            let kind = entry.file_type().unwrap();
            if kind.is_dir() {
                if name != ".git" {
                    stack.push(child);
                }
            } else if kind.is_file() {
                if name.ends_with(".cs") {
                    tree.cs.push(child);
                } else if name.ends_with(".csproj")
                    || name.ends_with(".projitems")
                    || name.ends_with(".shproj")
                {
                    tree.projects.push(child);
                } else if is_resource(&name) && entry.metadata().unwrap().len() < 2_000_000 {
                    tree.resources.push(child);
                }
            }
        }
    }
    tree.cs.sort();
    tree
}

fn read(root: &Path, rel: &str) -> String {
    let bytes = std::fs::read(root.join(rel)).unwrap();
    let text = String::from_utf8_lossy(&bytes).into_owned();
    text.strip_prefix('\u{feff}')
        .map(str::to_owned)
        .unwrap_or(text)
}

/// C# with its comments blanked, and the same with string and char literals blanked too. Strings
/// stay in the first: a type named in a string is a caller (`Type.GetType`), and a declaration in
/// a string (a code template) is not a declaration.
fn strip(text: &str) -> (String, String) {
    let b = text.as_bytes();
    let (mut keep, mut code) = (Vec::with_capacity(b.len()), Vec::with_capacity(b.len()));
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        let next = b.get(i + 1).copied();
        if c == b'/' && next == Some(b'/') {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if c == b'/' && next == Some(b'*') {
            let end = text[i + 2..].find("*/").map_or(b.len(), |e| i + 2 + e + 2);
            keep.push(b' ');
            code.push(b' ');
            i = end;
            continue;
        }
        let mut j = i;
        let mut verbatim = false;
        while j < b.len() && (b[j] == b'@' || b[j] == b'$') {
            verbatim |= b[j] == b'@';
            j += 1;
        }
        if j < b.len() && b[j] == b'"' && (j > i || c == b'"') {
            j += 1;
            while j < b.len() {
                if verbatim {
                    if b[j] == b'"' {
                        if b.get(j + 1) == Some(&b'"') {
                            j += 2;
                            continue;
                        }
                        break;
                    }
                } else if b[j] == b'\\' {
                    j += 2;
                    continue;
                } else if b[j] == b'"' || b[j] == b'\n' {
                    break;
                }
                j += 1;
            }
            let end = (j + 1).min(b.len());
            keep.extend_from_slice(&b[i..end]);
            code.extend_from_slice(b" \"\" ");
            i = end;
            continue;
        }
        if c == b'\'' {
            // 'x', '\'', 'A': short, and never spans a line
            let from = i + if b.get(i + 1) == Some(&b'\\') { 3 } else { 2 };
            let close = b
                .get(from..)
                .and_then(|r| r.iter().take(8).position(|&x| x == b'\'' || x == b'\n'))
                .map(|p| from + p);
            if let Some(close) = close.filter(|&p| b[p] == b'\'') {
                keep.extend_from_slice(&b[i..=close]);
                code.extend_from_slice(b"' '");
                i = close + 1;
                continue;
            }
        }
        keep.push(c);
        code.push(c);
        i += 1;
    }
    (
        String::from_utf8_lossy(&keep).into_owned(),
        String::from_utf8_lossy(&code).into_owned(),
    )
}

fn identifiers(text: &str) -> impl Iterator<Item = &str> {
    text.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
        .filter(|w| w.starts_with(|c: char| c.is_ascii_alphabetic() || c == '_'))
}

/// Whether `code` (comments and strings blanked) declares a type named `name`.
fn declares(code: &str, name: &str) -> bool {
    let words: Vec<&str> = identifiers(code).collect();
    words.windows(2).any(|w| {
        matches!(w[0], "class" | "struct" | "interface" | "enum" | "record") && w[1] == name
    }) || code.match_indices("delegate").any(|(at, _)| {
        let rest = &code[at..];
        let head = rest.split(['(', '<', ';', '{']).next().unwrap_or("");
        identifiers(head).last() == Some(name)
    })
}

/// Extension methods: `Name(this T x)` is called as `x.Name()`, never through its class.
fn extension_methods(code: &str) -> Vec<String> {
    let mut out = Vec::new();
    for (at, _) in code.match_indices("(this ") {
        let before = code[..at].trim_end();
        let before = before
            .strip_suffix('>')
            .map_or(before, |b| b.rfind('<').map_or(b, |open| &b[..open]));
        if let Some(name) = identifiers(before).last() {
            out.push(name.to_owned());
        }
    }
    out
}

/// A normalised path joined onto a project's directory, `\` read as `/`.
fn join(dir: &str, rel: &str) -> String {
    let mut parts: Vec<&str> = if dir.is_empty() {
        Vec::new()
    } else {
        dir.split('/').collect()
    };
    let rel = rel.replace('\\', "/");
    for part in rel.split('/') {
        match part {
            "" | "." => {}
            ".." => {
                parts.pop();
            }
            p => parts.push(p),
        }
    }
    let mut out = parts.join("/");
    out.shrink_to_fit();
    out
}

/// MSBuild's wildcards, case-insensitively: `**` any directories, `*` and `?` within one.
fn glob(pattern: &[u8], path: &[u8]) -> bool {
    match pattern.split_first() {
        None => path.is_empty(),
        Some((b'*', rest)) if rest.first() == Some(&b'*') => {
            let after = rest.get(1..).unwrap_or(&[]);
            match after.strip_prefix(b"/") {
                // `**/`: zero or more whole directories
                Some(after) => (0..=path.len())
                    .any(|k| (k == 0 || path[k - 1] == b'/') && glob(after, &path[k..])),
                None => (0..=path.len()).any(|k| glob(after, &path[k..])),
            }
        }
        Some((b'*', rest)) => (0..=path.len())
            .take_while(|&k| k == 0 || path[k - 1] != b'/')
            .any(|k| glob(rest, &path[k..])),
        Some((b'?', rest)) => path.first().is_some_and(|&c| c != b'/') && glob(rest, &path[1..]),
        Some((&p, rest)) => {
            path.first().is_some_and(|&c| c.eq_ignore_ascii_case(&p)) && glob(rest, &path[1..])
        }
    }
}

#[derive(Debug, Default)]
struct Project {
    dir: String,
    sdk: bool,
    /// `(true, pattern)` includes, `(false, pattern)` removes, in document order.
    ops: Vec<(bool, String)>,
    refs: Vec<String>,
}

fn project(tree: &Tree, path: &str) -> Project {
    let text = read(&tree.root, path);
    let dir = path.rsplit_once('/').map_or("", |(d, _)| d).to_owned();
    let mut p = Project {
        dir: dir.clone(),
        ..Project::default()
    };
    let Ok(doc) = roxmltree::Document::parse(&text) else {
        return p;
    };
    let root = doc.root_element();
    p.sdk = root.has_tag_name("Project") && root.attribute("Sdk").is_some();
    let no_defaults = root.descendants().any(|n| {
        n.tag_name().name() == "EnableDefaultCompileItems"
            && n.text()
                .is_some_and(|t| t.trim().eq_ignore_ascii_case("false"))
    });
    p.sdk &= !no_defaults;
    for node in root.descendants() {
        let name = node.tag_name().name();
        let group_condition = node.parent_element().and_then(|g| g.attribute("Condition"));
        let conditional = group_condition.is_some() || node.attribute("Condition").is_some();
        match name {
            "Compile" => {
                if let Some(inc) = node.attribute("Include") {
                    for x in inc.split(';') {
                        let x = x.replace("$(MSBuildThisFileDirectory)", "");
                        p.ops.push((true, join(&dir, &x)));
                    }
                }
                // A conditional Remove may not apply to the build that matters: ignored, so a
                // file is never called uncompiled on a condition's word.
                if let (Some(rem), false) = (node.attribute("Remove"), conditional) {
                    for x in rem.split(';') {
                        p.ops.push((false, join(&dir, x)));
                    }
                }
            }
            "ProjectReference" => {
                if let Some(inc) = node.attribute("Include") {
                    p.refs.push(join(&dir, inc));
                }
            }
            "Import" => {
                if let Some(imp) = node
                    .attribute("Project")
                    .filter(|i| i.ends_with(".projitems"))
                {
                    p.refs.push(join(&dir, imp));
                }
            }
            "HintPath" => {
                // `Updater\bin\Release\Updater.exe`: a reference to the project built there.
                let hint = node.text().unwrap_or("").replace('\\', "/");
                if let Some((built, _)) = hint.split_once("/bin/") {
                    let built = join(&dir, built);
                    p.refs.extend(
                        tree.projects
                            .iter()
                            .filter(|q| q.rsplit_once('/').map_or("", |(d, _)| d) == built)
                            .cloned(),
                    );
                }
            }
            _ => {}
        }
    }
    p
}

impl Project {
    fn compiles(&self, file: &str) -> bool {
        let under = self.dir.is_empty() || file.starts_with(&format!("{}/", self.dir));
        let mut hit = false;
        if self.sdk && under {
            let rel = if self.dir.is_empty() {
                file
            } else {
                &file[self.dir.len() + 1..]
            };
            let first = rel.split('/').next().unwrap_or("");
            hit = !(first.eq_ignore_ascii_case("bin") || first.eq_ignore_ascii_case("obj"));
        }
        for (include, pattern) in &self.ops {
            if glob(pattern.as_bytes(), file.as_bytes()) {
                hit = *include;
            }
        }
        hit
    }
}

fn solution_projects(tree: &Tree, sln: &str) -> BTreeSet<String> {
    read(&tree.root, sln)
        .lines()
        .filter(|l| l.starts_with("Project("))
        .filter_map(|l| l.split('"').nth(5))
        .filter(|p| p.ends_with("proj"))
        .map(|p| join("", p))
        .collect()
}

/// The whole tree, measured once for every row.
struct World {
    projects: BTreeMap<String, Project>,
    /// The projects the two solutions build: those they list and everything those reference.
    built: BTreeSet<String>,
    /// For each name asked about, the files (`.cs` and resource) that carry it outside comments.
    named_in: HashMap<String, BTreeSet<String>>,
    /// Comment- and string-free text of the D16 files, for their declarations.
    code: HashMap<String, String>,
}

fn world(root: &Path, dead: &[Dead], extra_names: &[&str]) -> World {
    let tree = walk(root);
    let projects: BTreeMap<String, Project> = tree
        .projects
        .iter()
        .map(|p| (p.clone(), project(&tree, p)))
        .collect();
    let mut stack: Vec<String> = solution_projects(&tree, "MissionPlanner.sln")
        .into_iter()
        .chain(solution_projects(&tree, "MissionPlannerLib.sln"))
        .chain(["MissionPlanner.csproj".to_owned()])
        .collect();
    let mut built = BTreeSet::new();
    while let Some(p) = stack.pop() {
        if let Some(project) = projects.get(&p) {
            stack.extend(project.refs.iter().cloned());
        }
        built.insert(p);
    }

    let mut code = HashMap::new();
    let mut wanted: HashSet<String> = extra_names.iter().map(|s| (*s).to_owned()).collect();
    for d in dead {
        let (_, c) = strip(&read(root, &d.path));
        wanted.extend(d.declares.iter().cloned());
        wanted.extend(extension_methods(&c));
        if let Reason::Namespace { token, .. } = &d.reason {
            wanted.insert(token.clone());
        }
        code.insert(d.path.clone(), c);
    }
    let mut named_in: HashMap<String, BTreeSet<String>> = HashMap::new();
    for file in tree.cs.iter().chain(&tree.resources) {
        let text = read(root, file);
        let text = if file.ends_with(".cs") {
            strip(&text).0
        } else {
            text
        };
        for word in identifiers(&text) {
            if wanted.contains(word) {
                named_in
                    .entry(word.to_owned())
                    .or_default()
                    .insert(file.clone());
            }
        }
    }
    World {
        projects,
        built,
        named_in,
        code,
    }
}

fn dir_and_stem(path: &str) -> (&str, &str) {
    let (dir, name) = path.rsplit_once('/').unwrap_or(("", path));
    (dir, name.split('.').next().unwrap_or(name))
}

impl World {
    /// Files outside `skip` that name `word` and could call: not in a test tree, not a resource
    /// of a skipped `.cs` (a form's own `.resx` names its form).
    fn callers(&self, word: &str, skip: &HashSet<&str>) -> Vec<String> {
        let skip_stems: HashSet<(&str, &str)> = skip.iter().map(|p| dir_and_stem(p)).collect();
        self.named_in
            .get(word)
            .into_iter()
            .flatten()
            .filter(|f| !skip.contains(f.as_str()) && !is_test(f))
            .filter(|f| f.ends_with(".cs") || !skip_stems.contains(&dir_and_stem(f)))
            .cloned()
            .collect()
    }

    fn compiled_by(&self, file: &str) -> Vec<&str> {
        self.projects
            .iter()
            .filter(|(_, p)| p.compiles(file))
            .map(|(name, _)| name.as_str())
            .collect()
    }

    /// What is wrong with one row's reason against the tree, if anything. `code` is the file
    /// with comments and strings blanked.
    fn problems(&self, d: &Dead, code: &str, dead: &HashSet<&str>) -> Vec<String> {
        let mut out = Vec::new();
        for name in &d.declares {
            if !declares(code, name) {
                out.push(format!("does not declare {name}"));
            }
        }
        match &d.reason {
            Reason::Types => {
                let mut names: Vec<String> = d.declares.clone();
                names.extend(extension_methods(code));
                for name in names {
                    let callers = self.callers(&name, dead);
                    if !callers.is_empty() {
                        out.push(format!("{name} is named by {callers:?}"));
                    }
                }
            }
            Reason::Namespace { dir, token } => {
                if !d.path.starts_with(dir.as_str()) {
                    out.push(format!("is not under {dir}"));
                }
                let callers: Vec<String> = self
                    .callers(token, dead)
                    .into_iter()
                    .filter(|f| !f.starts_with(dir.as_str()))
                    .collect();
                if !callers.is_empty() {
                    out.push(format!("{token} is named by {callers:?}"));
                }
            }
            Reason::Project(named) => {
                for p in named {
                    if !self.projects.contains_key(p) {
                        out.push(format!("{p} does not exist"));
                    }
                    if self.built.contains(p) {
                        out.push(format!("{p} is built"));
                    }
                }
                let built: Vec<&str> = self
                    .compiled_by(&d.path)
                    .into_iter()
                    .filter(|p| self.built.contains(*p))
                    .collect();
                if !built.is_empty() {
                    out.push(format!("is compiled by {built:?}, which a solution builds"));
                }
            }
            Reason::Uncompiled => {
                let by = self.compiled_by(&d.path);
                if !by.is_empty() {
                    out.push(format!("is compiled by {by:?}"));
                }
            }
        }
        out
    }
}

/// Every D16 row's reason, re-derived from the C# tree.
#[test]
fn d16_rows_still_have_no_callers() {
    let Some(root) = tree() else {
        return;
    };
    let (dead, _) = ledger();
    let controls = ["ConfigHWCompass2", "MissionPlanner"];
    let world = world(&root, &dead, &controls);
    let dead_set: HashSet<&str> = dead.iter().map(|d| d.path.as_str()).collect();

    let mut problems = Vec::new();
    for d in &dead {
        for p in world.problems(d, &world.code[&d.path], &dead_set) {
            problems.push(format!("{}: {p}", d.path));
        }
    }
    assert!(
        problems.is_empty(),
        "{} D16 row(s) no longer hold (a caller came back, or the file is built again): the \
         row goes back to `ready` and the note is removed, or the reason is corrected\n{}",
        problems.len(),
        problems.join("\n")
    );

    // The checks can fail: each, pointed at something alive, finds what keeps it alive.
    let alive = |path: &str, reason: Reason, declares: &[&str]| {
        let d = Dead {
            path: path.to_owned(),
            reference: String::new(),
            reason,
            declares: declares.iter().map(|s| (*s).to_owned()).collect(),
        };
        world.problems(&d, &strip(&read(&root, path)).1, &dead_set)
    };
    let compass = "GCSViews/ConfigurationView/ConfigHWCompass2.cs";
    let found = alive(compass, Reason::Types, &["ConfigHWCompass2"]);
    assert!(
        found.iter().any(|p| p.contains("GCSViews/InitialSetup.cs")),
        "the types check misses InitialSetup.cs's call: {found:?}"
    );
    let found = alive(
        "ExtLibs/Utilities/fft.cs",
        Reason::Namespace {
            dir: "ExtLibs/Utilities/".to_owned(),
            token: "MissionPlanner".to_owned(),
        },
        &[],
    );
    assert!(
        !found.is_empty(),
        "the namespace check finds nobody naming MissionPlanner"
    );
    let found = alive("MainV2.cs", Reason::Uncompiled, &[]);
    assert!(
        found.iter().any(|p| p.contains("MissionPlanner.csproj")),
        "the build check does not see MissionPlanner.csproj compile MainV2.cs: {found:?}"
    );
    let found = alive(
        "ExtLibs/Comms/CommsSerialPort.cs",
        Reason::Project(vec!["ExtLibs/Comms/MissionPlanner.Comms.csproj".to_owned()]),
        &[],
    );
    assert!(
        found
            .iter()
            .any(|p| p.contains("MissionPlanner.Comms.csproj is built")),
        "the project check does not see MissionPlanner.csproj reference Comms: {found:?}"
    );
}

/// The MSBuild wildcard matcher, on the shapes the tree uses.
#[test]
fn msbuild_globs() {
    let yes = [
        ("ExtLibs/**", "ExtLibs/a/b.cs"),
        (
            "ExtLibs/Zeroconf/Zeroconf/Platforms/**/*.*",
            "ExtLibs/Zeroconf/Zeroconf/Platforms/winrt/N.cs",
        ),
        ("**/AssemblyInfo.cs", "ExtLibs/X/Properties/AssemblyInfo.cs"),
        ("**/AssemblyInfo.cs", "AssemblyInfo.cs"),
        ("a/*.cs", "a/B.CS"),
    ];
    let no = [
        ("a/*.cs", "a/b/c.cs"),
        (
            "ExtLibs/Zeroconf/Zeroconf/Platforms/netstandard1.3/*.cs",
            "ExtLibs/Zeroconf/Zeroconf/Platforms/winrt/N.cs",
        ),
        ("Color.cs", "ExtLibs/Color.cs"),
    ];
    for (p, f) in yes {
        assert!(glob(p.as_bytes(), f.as_bytes()), "{p} should match {f}");
    }
    for (p, f) in no {
        assert!(
            !glob(p.as_bytes(), f.as_bytes()),
            "{p} should not match {f}"
        );
    }
    assert_eq!(
        join("ExtLibs/Xamarin/Xamarin", "..\\..\\..\\Controls\\A.cs"),
        "Controls/A.cs"
    );
}

/// The lexer keeps strings (a reflection caller) and drops comments (not a caller).
#[test]
fn comments_are_not_callers_and_strings_are() {
    let src = "// new Foo();\n/* Bar */ var t = Type.GetType(\"X.Baz\"); var u = @\"a\"\"//Qux\";\n\
               char c = '\"'; class Real {}";
    let (keep, code) = strip(src);
    let kept: Vec<&str> = identifiers(&keep).collect();
    assert!(!kept.contains(&"Foo") && !kept.contains(&"Bar"), "{kept:?}");
    assert!(kept.contains(&"Baz") && kept.contains(&"Qux"), "{kept:?}");
    assert!(declares(&code, "Real"));
    assert!(!declares(&strip("var s = \"class Fake {}\";").1, "Fake"));
    assert!(declares(
        "public delegate void Handler(object o);",
        "Handler"
    ));
    assert_eq!(
        extension_methods("public static string ToJSON<T>(this T o) { }"),
        vec!["ToJSON".to_owned()]
    );
}
